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
  - `route` + `interpret`: the deterministic interpreter. It parses an
    operator sentence, ANSWERS what it is confident about, and abstains on
    the rest so the relay's model handles it. There is no keyword ladder
    underneath it: the `IntentRouter` that used to be exported here was
    deleted on 2026-09-22 with every bridge copy of it, because an OmniKey
    is required for every chat turn and a regex fallback for a keyless
    /prompt is exactly what that policy forbids.
"""

from .bridge_base import (
    BridgeBase,
    SimClock,
    StepBudget,
    attach_telemetry,
    safety_gate_block,
    serve_events,
    serve_http,
    telemetry_tick,
)
from .events import EventRing, WAKE_POLICY, may_wake
from .hold import HoldLease, lockstep_enabled
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

from .route import describe_state  # noqa: F401
from . import profile_sync  # noqa: F401

__version__ = "0.1.0"

__all__ = [
    "BridgeBase",
    "serve_http",
    # D1: the two clocks, and waits counted in sim steps.
    "SimClock",
    "StepBudget",
    "attach_telemetry",
    "telemetry_tick",
    # D4: the bridge's own event stream.
    "EventRing",
    "serve_events",
    "WAKE_POLICY",
    "may_wake",
    # D6: the opt-in lockstep hold.
    "HoldLease",
    "lockstep_enabled",
    # D2: `/capabilities.safety_gate` (PROTOCOL.md §5.2.1).
    "safety_gate_block",
    "Tool",
    "OmniLinkRelay",
    "OllamaRelay",
    "ollama_available",
    "is_enabled",
    "get_omni_key",
    "profile_sync",
    "describe_state",
    "__version__",
]
