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

"""`omnisim key --check` must say DISABLED when the platform switched a key off.

2026-09-23: Google refused three requests for billing, OmniLink flipped the
account's key to `invalid`, and `--check` said "no model provider is
connected" while the API & Keys page listed the key. The platform now lists
disabled keys with status "invalid"; this pins what the CLI prints for them.
"""
import io
import json
from contextlib import redirect_stdout

import omnisim.key as key


class _Resp(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def _run_check(monkeypatch, quota):
    body = {"omniKey": {"label": "Omni Key"}, "quota": quota}
    monkeypatch.setattr(key.urllib.request, "urlopen",
                        lambda *a, **k: _Resp(json.dumps(body).encode()))
    out = io.StringIO()
    with redirect_stdout(out):
        code = key._check("olink_testkey1234")
    return code, out.getvalue()


def test_a_disabled_key_is_reported_as_disabled_not_absent(monkeypatch):
    code, text = _run_check(monkeypatch, [
        {"provider": "google", "status": "invalid", "lastFailureReason": "billing"}])
    assert code == 0
    assert "DISABLED" in text and "google" in text and "billing" in text
    assert "no model provider is connected" not in text
    assert "Model providers connected" not in text


def test_a_working_key_is_still_reported_as_connected(monkeypatch):
    _, text = _run_check(monkeypatch, [{"provider": "google", "status": "active"}])
    assert "Model providers connected: google" in text
    assert "DISABLED" not in text


def test_no_key_at_all_still_says_so(monkeypatch):
    _, text = _run_check(monkeypatch, [])
    assert "no model provider is connected" in text
