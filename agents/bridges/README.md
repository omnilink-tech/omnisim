# Real-robot bridge starter kit

A Webots-less, OmniSim-less starting point for porting OmniLink-driven
robot control to a real fleet. **Zero Webots imports. Zero OmniSim
imports.** Stdlib + `omnilink` (optional, for the relay path).

If you've been using OmniSim's `omnilink_arm_bridge` or
`omnilink_mobile_bridge` to drive a simulated robot through OmniLink,
this is the file you'd ship for production: same HTTP surface, same
chat behaviour, same OmniLink agent (Foreman / Picker / Roomba / Axis)
pointing at it. The only difference is what `act_*` calls under the
hood — your robot's SDK instead of `motor.setPosition(...)`.

## Files

| File | What it is |
|---|---|
| [bridge_base.py](bridge_base.py) | Abstract `BridgeBase` + HTTP server. Reusable; you should not need to edit it. |
| [arm_bridge_stub.py](arm_bridge_stub.py) | Working arm bridge against a `MockArmDriver` that prints what it would have done. Copy-paste starting point for a real arm. |
| [mobile_bridge_stub.py](mobile_bridge_stub.py) | Same for a wheeled base / AGV. |
| [grippers/](grippers/) | Real-gripper drivers (Robotiq 2F/3F, OnRobot RG, Schunk EGx, vacuum, mock) behind one `RealGripperDriver` surface. Select with `--gripper`. |

## Grippers

A gripper is decoupled from the arm and attached with `--gripper <id>`.
The ids match the sim registry, so the *same* flag selects the same
physical gripper in OmniSim and on real hardware:

```bash
python agents/bridges/arm_bridge_stub.py --gripper robotiq_2f85
```

Drivers default to a `DryTransport` (no hardware — they log the Modbus
register frames / IO toggles they would send), so the whole chat → tool →
HTTP → driver path runs and is unit-testable with nothing plugged in. For
deployment, build the driver with a real transport:

```python
from grippers.robotiq import Robotiq2FDriver
from my_modbus import PymodbusTransport          # your client
g = Robotiq2FDriver(max_width=0.085, transport=PymodbusTransport(host="…"))
```

Surface (added to every bridge): `/open_gripper`, `/close_gripper`,
`/set_gripper_width` (metres), `/grasp` (force/width), `/release`.
Known ids: see `grippers.GRIPPER_SPECS`. Tests:
`python agents/bridges/grippers/test_drivers.py`.

## Quick demo

```bash
# In one terminal:
python agents/bridges/arm_bridge_stub.py

# In another terminal:
curl -X POST -H "Content-Type: application/json" \
    -d '{"text":"go home"}' http://127.0.0.1:8765/prompt
```

You should see the stub print `[MOCK_ARM] reset_to_home` and return a
JSON `{"response": "Moving to home pose.", "actions": [...]}`.

Try also:

```bash
curl -X POST -d '{"text":"open the gripper"}' \
    -H 'Content-Type: application/json' http://127.0.0.1:8765/prompt
curl -X POST -d '{"text":"go to 0.4 0.0 0.3"}' \
    -H 'Content-Type: application/json' http://127.0.0.1:8765/prompt
curl -X POST -d '{}' http://127.0.0.1:8765/list_robots
curl -X POST -d '{"tool":"reset_to_home"}' \
    -H 'Content-Type: application/json' http://127.0.0.1:8765/tool
```

## Porting to a real arm

1. **Pick the file that matches your robot:** `arm_bridge_stub.py`
   (manipulator) or `mobile_bridge_stub.py` (wheeled base).
2. **Replace the driver class** (`MockArmDriver` / `MockMobileDriver`)
   with your robot SDK's client. Each driver method's body becomes a
   one-line SDK call.
3. **Run the file.** The HTTP surface stays identical so the OmniLink
   side of the integration is unchanged.

That's the whole port. Same agents, same prompts, same `/tool`
callbacks — different robot.

## Driving it from OmniLink

The bridge exposes the same `/tool` endpoint OmniSim bridges expose.
That means the example specialist agents (`agents/templates/`) drive
this real-robot stub by pointing their bridge URL at us:

```bash
# Roomba against the real-robot stub:
export ROOMBA_BRIDGE="http://127.0.0.1:8765"
python agents/templates/roomba/register.py
# Now pick "OmniSim-Roomba" in the OmniLink web UI -- it talks to the
# stub, which prints the mock motor commands.
```

If you want the bridge itself to host an OmniLink chat-with-tools loop
(rather than relying on the platform-side UI), `pip install omnilink`
and instantiate an `omnilink_relay.OmniLinkRelay` exactly like the
OmniSim bridges do; pass the relay's `dispatch_async` to your `/prompt`
handler, and you're done. See
`projects/samples/demos/controllers/_omnilink_relay/omnilink_relay.py`
for the reference implementation -- you can import it directly or copy
the relevant ~200 lines of dispatch into your bridge.

## Why this exists

> "The sim-to-real gap doesn't exist."

That's only true if you can prove it. The stub is the proof: a
self-contained, runnable example with zero OmniSim dependencies that
behaves identically to a simulated OmniSim arm from the OmniLink
agent's perspective. Read the file, run the curl examples, and you've
seen the entire sim-to-real seam.
