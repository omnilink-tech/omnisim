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

"""A 402 from OmniLink must be reported as the refusal the platform stated.

WHY THIS EXISTS. Until 2026-09-23 the relay turned EVERY 402 into "OmniLink
needs a model-provider key (402 BYOK_REQUIRED)". The platform also answers 402
AGENT_LIMIT_REACHED when an account holds more agent profiles than its plan
allows. An account with 41 profiles on a 25-agent plan therefore saw every chat
demo told to "connect a model-provider key" -- while its Google key worked, as
an older profile on the same account proved by answering normally. A day of
diagnosis went the wrong way on that message.

The bodies below are the platform's real ones, as measured on 2026-09-23.
"""
from __future__ import annotations

import pytest

omnilink_client = pytest.importorskip("omnilink.client")
OmniLinkAPIError = omnilink_client.OmniLinkAPIError

from omnisim_bridges.relay import OmniLinkRelay       # noqa: E402
from omnisim_bridges.tool import Tool                 # noqa: E402

AGENT_LIMIT_BODY = {"error": {
    "code": "AGENT_LIMIT_REACHED",
    "message": ("Your plan includes 25 OmniLink agents. Remove an unused "
                "profile or upgrade to run this agent. Existing profiles and "
                "history are preserved."),
}}
BYOK_BODY = {"error": {"code": "BYOK_REQUIRED",
                       "message": "Connect a model provider to continue."}}


class _Raises:
    """A client whose chat() raises the platform error it was given."""

    def __init__(self, err: Exception) -> None:
        self.err = err

    def chat(self, *a, **kw):
        raise self.err


def _relay(err: Exception) -> OmniLinkRelay:
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="OmniSim-test-402", main_task="test",
        tools=[Tool("drive_forward", "move", {"type": "object", "properties": {}},
                    lambda a: {"commanded": 1.0, "achieved": 0.99})],
        surface="mobile", usage_enabled=False, memory_enabled=False)
    relay._client = _Raises(err)
    return relay


def _error(err: Exception) -> str:
    out = _relay(err)._post_chat([{"role": "user", "content": "hi"}])
    assert out.get("ok") is False
    return str(out.get("error") or "")


def test_agent_limit_is_not_reported_as_a_missing_provider_key() -> None:
    msg = _error(OmniLinkAPIError(402, AGENT_LIMIT_BODY))
    assert "BYOK" not in msg and "model-provider key (402" not in msg, (
        "a 402 AGENT_LIMIT_REACHED was reported as a missing provider key -- "
        "the exact misdiagnosis this test exists to stop: " + msg)
    assert "AGENT_LIMIT_REACHED" in msg
    assert "provider key is not the problem" in msg
    assert "25 OmniLink agents" in msg, "the platform's own words must reach the operator"


def test_a_real_byok_refusal_still_says_so() -> None:
    msg = _error(OmniLinkAPIError(402, BYOK_BODY))
    assert "BYOK_REQUIRED" in msg
    assert "omnisim byok --add google" in msg


def test_an_unknown_402_reports_the_platform_and_does_not_guess() -> None:
    msg = _error(OmniLinkAPIError(402, {"error": {"code": "SOMETHING_NEW",
                                                  "message": "A new limit."}}))
    assert "BYOK" not in msg
    assert "SOMETHING_NEW" in msg and "A new limit." in msg
