# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Every CLI call to the platform names itself, and an edge 403 is not a key verdict.

Measured 2026-11-17: Cloudflare's edge in front of www.omnilink-agents.com
answers Python's default `Python-urllib/x.y` User-Agent with 403 before the
platform sees the request -- as plain text "error code: 1010", or, when the
request asks for JSON, as an RFC 9457 body with `"cloudflare_error": true`.
`omnisim key --check` therefore called every key REJECTED, valid or not.
Engine-free; no network.
"""
from __future__ import annotations

import importlib.util
import io
import json
import urllib.error
from contextlib import redirect_stdout
from pathlib import Path

import pytest

import omnisim.key as key
import omnisim.provider_key as provider_key
from omnisim import platform_http

REPO = Path(__file__).resolve().parents[1]

EDGE_TEXT = b"error code: 1010\n"
EDGE_JSON = json.dumps({
    "type": "https://developers.cloudflare.com/support/troubleshooting/http-status-codes/"
            "cloudflare-1xxx-errors/error-1010/",
    "title": "Error 1010: Access denied", "status": 403, "error_code": 1010,
    "error_name": "browser_signature_banned", "cloudflare_error": True}).encode()
PLATFORM_403 = b'{"error":"Access denied: invalid Omni Key."}'
PLATFORM_403_NESTED = b'{"error":{"code":"OMNI_KEY_NOT_FOUND","message":"Access denied: Omni Key not found."}}'


class _Resp(io.BytesIO):
    status = 200

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def _http_error(url, code, body):
    return urllib.error.HTTPError(url, code, "Forbidden", {}, io.BytesIO(body))


def _capture(monkeypatch, module, *, body=b"{}", error=None):
    sent = []

    def fake_urlopen(req, timeout=None):
        sent.append(req)
        if error is not None:
            raise _http_error(req.full_url, *error)
        return _Resp(body)

    monkeypatch.setattr(module.urllib.request, "urlopen", fake_urlopen)
    return sent


def _ua(req) -> str:
    return req.get_header("User-agent") or ""


# ── the helper ────────────────────────────────────────────────────────────

def test_user_agent_names_omnisim_and_is_not_pythons_default():
    ua = platform_http.user_agent("key")
    assert ua.startswith("omnisim/") and "(key;" in ua
    assert "urllib" not in ua.lower()


@pytest.mark.parametrize("status,raw,verdict", [
    (403, PLATFORM_403, True),
    (403, PLATFORM_403_NESTED, True),
    (401, b'{"error":"Missing Omni Key."}', True),
    (403, EDGE_TEXT, False),
    (403, EDGE_JSON, False),
    (403, b"<html>Forbidden</html>", False),
    (403, b"", False),
    (500, PLATFORM_403, False),
])
def test_only_the_platforms_own_401_403_is_a_verdict(status, raw, verdict):
    assert platform_http.is_platform_verdict(status, raw) is verdict


# ── omnisim key --check ───────────────────────────────────────────────────

def test_key_check_sends_an_omnisim_user_agent(monkeypatch):
    sent = _capture(monkeypatch, key, body=b'{"omniKey": {"label": "k"}, "quota": []}')
    with redirect_stdout(io.StringIO()):
        key._check("olink_testkey1234")
    assert len(sent) == 1
    assert _ua(sent[0]).startswith("omnisim/")


@pytest.mark.parametrize("edge_body", [EDGE_TEXT, EDGE_JSON])
def test_an_edge_403_is_not_reported_as_a_rejected_key(monkeypatch, edge_body):
    _capture(monkeypatch, key, error=(403, edge_body))
    out = io.StringIO()
    with redirect_stdout(out):
        code = key._check("olink_testkey1234")
    text = out.getvalue()
    assert code == 2
    assert "REJECTED" not in text
    assert "not a verdict on your key" in text


@pytest.mark.parametrize("platform_body", [PLATFORM_403, PLATFORM_403_NESTED])
def test_the_platforms_own_403_still_rejects_the_key(monkeypatch, platform_body):
    _capture(monkeypatch, key, error=(403, platform_body))
    out = io.StringIO()
    with redirect_stdout(out):
        code = key._check("olink_testkey1234")
    assert code == 1
    assert "REJECTED" in out.getvalue()
    assert "Access denied" in out.getvalue()


# ── omnisim byok ──────────────────────────────────────────────────────────

def test_byok_sends_an_omnisim_user_agent(monkeypatch):
    sent = _capture(monkeypatch, provider_key, body=b'{"credentials": []}')
    with redirect_stdout(io.StringIO()):
        provider_key._list("olink_testkey1234")
    assert _ua(sent[0]).startswith("omnisim/")


@pytest.mark.parametrize("edge_body", [EDGE_TEXT, EDGE_JSON])
def test_byok_add_does_not_blame_the_omni_key_for_an_edge_403(monkeypatch, edge_body):
    _capture(monkeypatch, provider_key, error=(403, edge_body))
    out = io.StringIO()
    with redirect_stdout(out):
        code = provider_key._add("olink_testkey1234", "google", "AIza-test", "default")
    text = out.getvalue()
    assert code == 1
    assert "OMNI_KEY problem" not in text
    assert "Could not reach OmniLink" in text
    assert ("Cloudflare" in text) or ("in front of the platform" in text)


def test_byok_add_still_points_at_the_omni_key_on_a_platform_403(monkeypatch):
    _capture(monkeypatch, provider_key, error=(403, b'{"error":"Invalid Omni Key."}'))
    out = io.StringIO()
    with redirect_stdout(out):
        provider_key._add("olink_testkey1234", "google", "AIza-test", "default")
    assert "OMNI_KEY problem" in out.getvalue()


# ── the chat-demo sweep's platform door ───────────────────────────────────

def test_smoke_chat_demos_post_json_sends_a_user_agent(monkeypatch):
    path = REPO / "scripts" / "dev" / "smoke_chat_demos.py"
    spec = importlib.util.spec_from_file_location("_smoke_chat_demos_ua", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    sent = _capture(monkeypatch, mod, body=b"{}")
    mod.post_json("https://www.omnilink-agents.com/api/x", {"a": 1},
                  headers={"Authorization": "Bearer olink_x"})
    assert _ua(sent[0]) == mod.SMOKE_USER_AGENT
    assert "urllib" not in mod.SMOKE_USER_AGENT.lower()
