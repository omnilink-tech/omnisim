"""omnisim_bridges -- OmniLink-driven bridge primitives for real robots.

Zero Webots dependency. Reuses the same Axis-normalised HTTP surface that
OmniSim's `omnilink_*_bridge` controllers expose, so an OmniLink agent
(Foreman / Picker / Roomba / Axis / your own) drives the simulated robot
today and the real robot tomorrow with no other changes.

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
    from .relay import OmniLinkRelay  # noqa: F401
    _HAS_RELAY = True
except Exception:
    OmniLinkRelay = None  # type: ignore[assignment]
    _HAS_RELAY = False

from .intent_router import IntentRouter  # noqa: F401

__version__ = "0.1.0"

__all__ = [
    "BridgeBase",
    "serve_http",
    "Tool",
    "OmniLinkRelay",
    "IntentRouter",
    "__version__",
]
