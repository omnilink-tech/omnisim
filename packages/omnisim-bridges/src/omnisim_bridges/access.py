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

"""Connection policy for OmniLink chat; direct simulator controls are separate."""
from __future__ import annotations

import os
from typing import Any

SETUP_URL = "https://www.omnilink-agents.com/agents/start"


def connection_error() -> dict[str, Any]:
    configured = bool(os.environ.get("OMNI_KEY", "").strip())
    message = (
        "OmniLink could not connect. Check your OmniKey and model connection, then restart the robot."
        if configured else
        "Connect OmniLink with your OmniKey and a model provider to send AI instructions."
    )
    return {"ok": False, "error": "omnilink_unavailable" if configured else "omnikey_required",
            "response": message, "actions": [], "setup_url": SETUP_URL}


def chat_config(relay: Any) -> dict[str, Any]:
    return {"chat_enabled": relay is not None, "omnikey_required": True,
            "setup_url": SETUP_URL,
            "connection_message": "" if relay is not None else connection_error()["response"]}


def reject_window_prompt(bridge: Any) -> None:
    bridge.queue_window("error:" + connection_error()["response"])
    bridge.queue_window("status:error")
