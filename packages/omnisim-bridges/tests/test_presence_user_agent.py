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

"""The relay heartbeat names itself, or it never reaches the platform.

Measured 2026-11-17: Cloudflare's edge in front of www.omnilink-agents.com
answers Python's default `Python-urllib/x.y` User-Agent with 403
"error code: 1010" before the platform sees the request. `_post_presence`
sent no User-Agent, so relay presence and the wake signal were refused at
the edge, silently (presence failures are swallowed by design).
"""

from __future__ import annotations

from omnisim_bridges import relay as relay_mod
from omnisim_bridges.relay import OmniLinkRelay


class _Stub:
    omni_key = "olink_testkey1234"


class _Resp:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def read(self):
        return b'{"ok": true}'


def test_platform_user_agent_is_not_pythons_default() -> None:
    ua = relay_mod.platform_user_agent()
    assert ua.startswith("omnisim-bridges/")
    assert "urllib" not in ua.lower()


def test_the_heartbeat_sends_the_bridges_user_agent(monkeypatch) -> None:
    sent = []

    def fake_urlopen(req, timeout=None):
        sent.append(req)
        return _Resp()

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)
    data = OmniLinkRelay._post_presence(_Stub(), {"agentName": "x", "detail": {}})
    assert data == {"ok": True}
    assert len(sent) == 1
    assert sent[0].full_url.endswith("/api/relay-heartbeat")
    assert sent[0].get_header("User-agent") == relay_mod.platform_user_agent()
    assert sent[0].get_header("Authorization") == "Bearer olink_testkey1234"
