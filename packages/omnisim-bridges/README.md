# omnisim-bridges

> OmniLink-driven bridge primitives for OmniSim and real robots.

```bash
pip install -e packages/omnisim-bridges/
```

The package is not on PyPI yet; the editable install above is the supported
installation from an OmniSim checkout.

Zero Webots dependency. The same primitives OmniSim's
`omnilink_*_bridge` controllers use, lifted out so you can build an
**OmniLink-driven bridge for a real robot in under 30 lines of new
code**.

## What you get

| Export | What it is |
|---|---|
| `BridgeBase` | Abstract base class. Implement `act_stop` + any action methods your robot supports. Unsupported actions return clean `{"error": "..."}` responses. |
| `serve_http(bridge, port)` | Spin up the Axis-normalised HTTP server. Same routes as OmniSim's bridges (`/list_robots`, `/get_robot_state`, `/prompt`, `/tool`, `/usage`, `/set_tcp_target`, `/drive_forward`, ...). |
| `Tool` | One named action with a JSON-schema params spec. Used by `OmniLinkRelay`. |
| `OmniLinkRelay` | Optional in-bridge chat-with-tools loop. Wraps `OmniLinkClient.chat()` so your bridge can host its own chat surface without round-tripping through the OmniLink web UI. Drops in tokens-per-hour usage telemetry + cross-session short-term memory. |
| `IntentRouter` | Tiny regex-based offline fallback. Pre-LLM router so `/prompt` does something useful when `OMNI_KEY` is unset. |

## 30-second example

```python
import time
from omnisim_bridges import BridgeBase, serve_http

class MyRealArm(BridgeBase):
    robot_id = "my_arm"
    model = "ACME ArmV5"

    def __init__(self):
        self.driver = ...  # your robot SDK

    def act_stop(self):
        self.driver.estop()
        return {"halted_at": time.time()}

    def act_reset_to_home(self):
        self.driver.move_to_home()
        return {"q": self.driver.read_q()}

    def act_set_tcp_target(self, xyz):
        self.driver.move_linear_to(xyz)
        return {"accepted": True}

    def get_state(self):
        return {"q": self.driver.read_q(), "id": self.robot_id}

bridge = MyRealArm()
server = serve_http(bridge, port=8765)

while True:
    time.sleep(1)  # let the HTTP server thread run
```

Point any OmniLink agent (the OmniSim-Foreman, OmniSim-Picker,
OmniSim-Roomba, Axis, or your own) at `http://127.0.0.1:8765/tool` and
the agent drives your real arm. No other changes — same prompts, same
toolset, same conversation flow as the simulated version.

## With OmniLink chat-with-tools

If you want the bridge to host the chat loop itself (not the web UI):

```python
import os
from omnisim_bridges import BridgeBase, OmniLinkRelay, Tool, serve_http

class MyArm(BridgeBase): ...

bridge = MyArm()

relay = OmniLinkRelay(
    omni_key=os.environ["OMNI_KEY"],
    agent_name="MyRealArm",
    main_task="You drive ACME ArmV5...",
    tools=[
        Tool(name="reset_to_home", description="...",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_reset_to_home()),
        # ... more tools
    ],
)

# Route /prompt through OmniLink by plugging the relay into the bridge's
# act_prompt hook (the default act_prompt just echoes the prompt).
bridge.act_prompt = lambda text: relay.dispatch_sync(text)

# Tool calls land on bridge.act_* via each Tool's dispatch.
serve_http(bridge, port=8765)
```

The relay's per-turn `usage` event and cross-session short-term memory
give the OmniSim chat panel its tokens-per-hour readout and the
"continue from last session" behaviour for free.

## Status

- `0.1.0` — initial local release. The in-tree OmniSim controllers import
  this package as their canonical relay and HTTP-safety implementation. API
  may evolve as the demo bridges add tools.
- Not on PyPI yet. Build locally with `pip install -e packages/omnisim-bridges/`
  from the OmniSim repo. Publish plan: tag `omnisim-bridges-vX.Y.Z`,
  CI builds + uploads.

## Why this exists

The OmniSim demo bridges already work. But they live alongside their
Webots controllers, and external integrators kept asking "how do I get
just the OmniLink-side parts." This package is that answer:
install the local package, subclass `BridgeBase`, and you have an
OmniLink-compatible robot-control service. The agent-facing interface is the
same code at a different URL; hardware behavior still needs its own validation.

This preserves the agent-to-bridge interface between simulation and hardware.
It is not a claim that dynamics, calibration, policies, or safety behavior have
been validated on a physical robot.

## HTTP security

`serve_http` binds to `127.0.0.1` by default. It rejects malformed JSON,
non-finite motion values, oversized bodies, and untrusted browser origins.
Non-stop actions are serialized; stop remains available while another request
is active.

Set `OMNISIM_BRIDGE_TOKEN` (or pass `token=`) before using a non-loopback host.
Clients send `Authorization: Bearer <token>` or `X-OmniSim-Token`. Configure
additional trusted browser origins with comma-separated
`OMNISIM_ALLOWED_ORIGINS`. `GET /protocol` advertises OmniSim Wire 1.0.

## License

Apache-2.0. See `LICENSE` in the OmniSim repo root.
