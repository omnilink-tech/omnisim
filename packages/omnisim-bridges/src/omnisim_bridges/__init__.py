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

"""omnisim_bridges -- OmniLink-driven bridge primitives for real robots.

Zero Webots dependency. Reuses the same Axis-normalised HTTP surface that
OmniSim's `omnilink_*_bridge` controllers expose, so an OmniLink agent
(Foreman / Picker / Roomba / Axis / your own) can keep the same HTTP tool
contract when pointed at a separately validated real-robot driver.

Quickstart:

    from omnisim_bridges import BridgeBase, serve_http

    class MyRealArm(BridgeBase):
        robot_id = "my_arm"
        model = "ACME ArmV5"

        def __init__(self):
            self.driver = AcmeArmSDK("...")

        def act_stop(self):
            self.driver.estop()
            return {"halted_at": time.time()}

        def act_reset_to_home(self):
            self.driver.move_to_home()
            return {"q": self.driver.read_q()}

        # ... implement the act_* methods that apply to your arm.

    bridge = MyRealArm()
    server = serve_http(bridge, port=8765)
    # Now point any OmniLink agent at http://127.0.0.1:8765/tool.

The package is structured around four exports:

  - `BridgeBase`: abstract base every bridge subclasses. Methods you
    don't implement default to a clean "unsupported" response.
  - `serve_http(bridge, port)`: spin up the Axis-normalised HTTP server.
  - `Tool` + `OmniLinkRelay`: optional in-bridge chat-with-tools loop
    (lift from omnilink_relay in the OmniSim demos). Lets your bridge
    host its own chat surface without round-tripping through the
    OmniLink web UI.
  - `IntentRouter`: tiny regex-based offline router. Pre-LLM fallback
    so /prompt does something useful when OMNI_KEY is unset.
"""

from .bridge_base import BridgeBase, serve_http
from .tool import Tool

try:
    from .relay import (  # noqa: F401
        OmniLinkRelay,
        OllamaRelay,
        get_omni_key,
        is_enabled,
        ollama_available,
    )
    _HAS_RELAY = True
except Exception:
    OmniLinkRelay = None  # type: ignore[assignment]
    OllamaRelay = None  # type: ignore[assignment]
    def ollama_available(timeout_s: float = 0.75) -> bool:  # type: ignore[misc]
        return False
    def is_enabled() -> bool:  # type: ignore[misc]
        return False
    def get_omni_key() -> str:  # type: ignore[misc]
        return ""
    _HAS_RELAY = False

from .intent_router import (  # noqa: F401
    IntentRouter, describe_state, is_resume, is_status,
)
from . import profile_sync  # noqa: F401

__version__ = "0.1.0"

__all__ = [
    "BridgeBase",
    "serve_http",
    "Tool",
    "OmniLinkRelay",
    "OllamaRelay",
    "ollama_available",
    "is_enabled",
    "get_omni_key",
    "profile_sync",
    "IntentRouter",
    "describe_state",
    "is_resume",
    "is_status",
    "__version__",
]
