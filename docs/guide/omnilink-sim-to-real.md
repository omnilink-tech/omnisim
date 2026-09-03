# OmniSim ↔ real robot: one control surface, two backends

> **The design.** The same OmniLink agent code, talking to the same chat panel,
> can address a robot in OmniSim and a real robot through the same tool surface.
> Porting is not a rewrite: it is the same wire protocol with a different
> dispatch implementation behind the bridge.

This guide names every file involved and shows where the swap happens.

> ### ⚠️ Scope of the claim — read this first
>
> This document is about **the control surface an LLM agent sees**: the tool
> names, argument schemas, return shapes, chat vocabulary, and HTTP endpoints.
> That layer is genuinely portable, and this guide shows exactly where the seam
> is.
>
> It is **not** a claim about physics transfer. OmniSim does **not** claim
> validated sim-to-real transfer:
>
> - **No policy trained in OmniSim has been validated on physical hardware.**
>   Sim-to-real transfer of learned control policies is **unproven** here, and
>   nothing in this repo should be read as evidence for it.
> - **The real-robot bridges shipped in [`agents/bridges/`](../../agents/bridges/)
>   run against mock drivers.** They log the commands they *would* send. Wiring
>   them to hardware — and validating the result — is your work, not ours.
> - Dynamics, contact, actuator response, latency, and sensor noise all differ
>   between OmniSim and any real robot. Anything that depends on those
>   (locomotion policies, force control, precise grasping) **will** need
>   real-world tuning and validation.
>
> What carries over is the *software above the driver*. That is a real and
> useful property, and it is the only thing this guide asserts.

---

## Architecture in one diagram

```
        you (typing in the side menu)
                  │
                  ▼
        OmniLink chat panel  ──┐
        (robot window, HTML/JS)│           plain HTTP /api/chat
                  │            │   ┌────────────────────────────┐
                  ▼            └──▶│      OmniLink platform     │
        omnilink_*_bridge.py       │    (omnilink-agents.com)   │
        (Python, in OmniSim)       └─────────────┬──────────────┘
                  │                              │
                  │                              ▼
                  │              structured toolCalls: [{name, args}]
                  │
                  ▼
        ┌─────────────────────────────┐    ┌─────────────────────────────┐
        │ dispatch implementation:    │    │ dispatch implementation:    │
        │   sim → OmniSim motors      │ OR │   real → robot driver       │
        │   (Supervisor.setPosition,  │    │   (ROS topic / CAN /        │
        │    Motor.setVelocity, …)    │    │    vendor SDK / …)          │
        └─────────────────────────────┘    └─────────────────────────────┘
```

Every layer above the dispatch row is **identical** between sim and real.
The agent's prompt, the tool inventory, the structured `toolCall` output,
the relay loop, the chat history — all the same. Only the dispatch
implementation differs.

That is a statement about the *software layers*, and it holds today (see
Layer 5 below). It says nothing about whether a behaviour tuned in
simulation will reproduce on hardware — see the scope box above.

---

## Layer-by-layer walkthrough

### Layer 1 — the chat UI

The user-facing chat panel.

| File | What it does |
|---|---|
| [`projects/samples/demos/plugins/robot_windows/omnilink_chat/omnilink_chat.html`](../../projects/samples/demos/plugins/robot_windows/omnilink_chat/omnilink_chat.html) | The OmniLink dark-theme side menu (transcript, prompt textarea, send/stop buttons) |
| [`projects/samples/demos/plugins/robot_windows/omnilink_chat/omnilink_chat.js`](../../projects/samples/demos/plugins/robot_windows/omnilink_chat/omnilink_chat.js) | wwi protocol with the bridge: `prompt:<text>` / `stop` / `configure` |
| [`projects/samples/demos/plugins/robot_windows/omnilink_chat/omnilink_chat.css`](../../projects/samples/demos/plugins/robot_windows/omnilink_chat/omnilink_chat.css) | OmniLink branded palette (black / cream / mimosa) |
| [`src/omnisim/gui/OmAgentHud.cpp`](../../src/omnisim/gui/OmAgentHud.cpp) | Right-side dock's "Status" + "Chat" tabs (Qt-native equivalent of the above) |

The chat UI is **simulator-side only**. In a real-world deployment, the
chat surface lives in the OmniLink web app or a custom client — same
`/api/chat` POSTs, no OmniSim involved.

### Layer 2 — the bridge

The Python service that fronts the robot and exposes a tool surface to
OmniLink.

| File | What it does |
|---|---|
| [`projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py`](../../projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py) | Mobile bridge — owns the wheel motors, defines the drive tools, dispatches them to `motor.setVelocity()` |
| [`projects/samples/demos/controllers/omnilink_quadruped_bridge/omnilink_quadruped_bridge.py`](../../projects/samples/demos/controllers/omnilink_quadruped_bridge/omnilink_quadruped_bridge.py) | OmniQuad bridge — same shape; pose presets, minimal tool surface |
| [`projects/samples/demos/controllers/mavic_omnilink_bridge/mavic_omnilink_bridge.py`](../../projects/samples/demos/controllers/mavic_omnilink_bridge/mavic_omnilink_bridge.py) | Mavic bridge — same shape; takeoff / land / translate / yaw |
| [`projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py`](../../projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py) | Per-robot mobile specs: wheel radius, half-track, max speed, layout |

The bridge owns two surfaces:

- **HTTP on `127.0.0.1:8765`** matching the Axis bridge contract (`omnisim-bridge.md`, in the OmniLink repo):
  - `POST /list_robots`, `POST /get_robot_state`, `POST /capabilities`, …
  - `POST /drive_forward`, `POST /turn`, `POST /set_velocity`, `POST /stop_robot`, `POST /reset_to_home`
  - `POST /prompt` (natural language → tool dispatch through the relay)
  - `POST /tool` (structured tool call from an OmniLink agent runner)
- **WWI (web window interface) protocol** with the chat panel.

### Layer 3 — the OmniLink relay

Glue between the bridge's tool inventory and the OmniLink chat API.

| File | What it does |
|---|---|
| [`packages/omnisim-bridges/src/omnisim_bridges/relay.py`](../../packages/omnisim-bridges/src/omnisim_bridges/relay.py) | The canonical relay class — wraps `OmniLinkClient.chat()` with single-flight async dispatch + chat-with-tools loop + event surface |

Uses the canonical `omnilink` Python client library (in the OmniLink repo):

```python
from omnilink.client import OmniLinkClient, OmniLinkAPIError

self._client = OmniLinkClient(omni_key=..., base_url=..., timeout=60)
result = self._client.chat(
    messages=messages,
    agent_name=self.agent_name,
    engine="g1-engine",
    system_instruction={
        "mainTask": "...",
        "availableToolDetails": [...],     # ← the bridge's tools
        "allowToolUse": True,
    },
)
toolCalls = result["toolCalls"]            # ← what the agent wants done
```

**This is the exact same call shape an external integrator would write.**
If you're building a desktop / phone / web client to drive your own
real robot through OmniLink, you'd `pip install omnilink` and write
that same code. The relay's only OmniSim-specific job is loop-and-dispatch.

### Layer 4 — the OmniLink platform

[omnilink-agents.com](https://www.omnilink-agents.com) — the LLM router, prompt pipeline, engine selection (Gemini / GPT / Grok / Claude), credit metering. You don't touch this layer; it's a service.

### Layer 5 — the agent

The personality / tool-use policy. Where you decide whether your agent is "Axis-the-robot-control-agent" or "MyCustomAgent."

| File | What it does |
|---|---|
| `axis/axis_agent.py` (OmniLink repo) | First-party Axis agent — robot-control specialist |
| `axis/profile.json` (OmniLink repo) | Axis's personality + tool registry |
| `axis/knowledge/omnisim-bridge.md` (OmniLink repo) | The HTTP contract Axis assumes — exactly what our bridges implement |

Axis is configured against `AXIS_BRIDGE_URL` (default `http://127.0.0.1:8765`).
Point it at OmniSim's bridge — the simulated robot moves. Point it at a
real robot's bridge — the real robot moves. Axis doesn't know the
difference.

---

## How the swap actually happens

Here's the **only thing** that changes when you go from sim to real:
the bridge's dispatch handlers.

### In simulation today

[`omnilink_mobile_bridge.py`](../../projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py) — `MobileBridge.act_set_velocity()`:

```python
def act_set_velocity(self, linear, angular):
    with self.lock:
        self.motion = ("velocity", {"l": linear, "a": angular})
    return {"accepted": True, "linear": linear, "angular": angular}
```

The bridge owns `self.motors` (a list of OmniSim `Motor` objects). The
tick loop turns the `(linear, angular)` command into per-wheel speeds
(clamped against `max_wheel_speed_radps`) and calls
`motor.setVelocity(w[i])` on each wheel.

### On a real robot tomorrow

Implement the same method against your real-robot driver. For a ROS 2 base:

```python
def act_set_velocity(self, linear, angular):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    self._cmd_vel_pub.publish(msg)
    return {"accepted": True, "linear": linear, "angular": angular}
```

For a vendor SDK:

```python
def act_set_velocity(self, linear, angular):
    self._base.drive(v=linear, omega=angular)
    return {"accepted": True, "linear": linear, "angular": angular}
```

For an OEM HTTP API:

```python
def act_set_velocity(self, linear, angular):
    r = requests.post(f"{self.robot_api_base}/cmd_vel",
                      json={"linear": linear, "angular": angular})
    return r.json()
```

The **method signature, return shape, safety-clamp logic, intent vocabulary,
chat surface, tool registry, agent code, OmniLink integration** —
all identical. You change one method.

### The pattern, abstracted

The "tool surface" is the abstraction. As long as:

- the same tools exist with the same arg schemas (`drive_forward(distance: float)`, `set_velocity(linear: float, angular: float)`, …),
- they return the same shape (`{"accepted": True, ...}` / `{"error": "..."}`),
- they enforce the same safety contract (clamp to actuator limits, refuse commands the robot can't execute, etc.),

…then **the agent code, the chat panel, the relay, the prompt vocabulary**
all carry over verbatim. The only file you write specifically for the
real robot is its dispatch implementation.

That's why we maintain the Axis bridge contract (`omnisim-bridge.md`, in the OmniLink repo) as a separate document: it's
the spec that both the OmniSim bridges AND any real-world adapter
implement. Everything above the contract is reusable.

---

## End-to-end workflow (concrete files)

You have a real robot. You want it to listen to natural language via
OmniLink. Here's the minimum path:

### 0. Prerequisites

- An OmniLink key (`olink_...`) from [omnilink-agents.com](https://www.omnilink-agents.com).
- Python 3.9+ with `pip install omnilink`.
- A way to command your real robot from Python (vendor SDK, ROS bindings, a CAN driver, an HTTP API…).

### 1. Write a real-robot bridge

Start from the OmniSim mobile bridge as a template (or from the
simulator-free stub in [`agents/bridges/`](../../agents/bridges/), which
already speaks this HTTP surface against a mock driver):

```bash
mkdir -p my_robot_omnilink/
cp projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py my_robot_omnilink/real_mobile_bridge.py
cp projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py my_robot_omnilink/
```

In `real_mobile_bridge.py`:

- Replace `from omnisim import Supervisor` with your real-robot driver imports (`rclpy`, `pyserial`, a vendor SDK, etc.).
- Replace `MobileBridge.__init__` motor-loading with whatever your driver needs.
- Reimplement `act_*` methods against your driver (see the swap example above).
- Keep `IntentRouter`, the HTTP server, and the relay wiring as-is.

The relay file is reusable verbatim:

```bash
cp -r projects/samples/demos/controllers/_omnilink_relay/ my_robot_omnilink/
```

### 2. Stand it up

```bash
export OMNI_KEY=olink_...
export OMNILINK_ENGINE=g1-engine    # or g2/g3/g4
python my_robot_omnilink/real_mobile_bridge.py --robot my_rover --port 8765
```

This serves:
- `127.0.0.1:8765/list_robots`, `/get_robot_state`, `/set_velocity`, …
- `127.0.0.1:8765/prompt` (chat-with-tools loop via OmniLink)

### 3. Talk to it

Three equivalent ways:

**Curl:**
```bash
curl -X POST -H 'Content-Type: application/json' \
     -d '{"text": "drive forward one meter"}' \
     http://127.0.0.1:8765/prompt
```

**OmniLink web UI:** Sign in at omnilink-agents.com, pick your agent, type.

**Your own UI:** Same code as our [omnilink_chat plugin](../../projects/samples/demos/plugins/robot_windows/omnilink_chat/) — post to the bridge's `/prompt` and render the response.

### 4. The same agent code can drive both

Put the OmniSim simulator on one machine, your real robot's bridge on
another. Same agent, same OmniLink key, same prompt vocabulary. Flip
between them by changing `AXIS_BRIDGE_URL`:

```bash
# Drive the simulated robot in OmniSim
export AXIS_BRIDGE_URL=http://omnisim-host:8765
python <olink-repo>\agents\axis\axis_agent.py

# Drive the real robot
export AXIS_BRIDGE_URL=http://real-robot-host:8765
python <olink-repo>\agents\axis\axis_agent.py
```

Axis is a single agent definition. The robot it talks to is determined
entirely by which `/list_robots` URL it points at.

---

## What you actually get from this design

- **Agent code written once.** Your OmniLink agent (Axis, or your own
  specialist) is the same file across sim and real.
- **Prompt vocabulary written once.** "Go home", "forward one meter",
  "turn left 90 degrees", "stop" — same in both contexts.
- **Safety policies written once.** Actuator-limit clamps, speed
  ceilings, rejection of commands the robot can't execute — implemented
  in the bridge, inherited by both deployments.
- **Logging and audit trails written once.** Every prompt → toolCall →
  response chain is uniform across sim and real, which makes
  cross-checking the sim's prediction against real-world behaviour a
  matter of diffing two transcript files.
- **Agent-level iteration carries over.** Refining the agent's `main_task`,
  its tool definitions, or its prompt vocabulary in OmniSim carries over
  to the real deployment unchanged — that work is not thrown away.

### What you do *not* get

- **Physics transfer.** The dynamics, contact, actuator response, latency,
  and sensor noise are different. A motion that looks right in OmniSim is
  not thereby validated on hardware.
- **Validated policy transfer.** No policy trained in OmniSim has been run
  on a physical robot as part of this project. Treat sim-to-real transfer
  of learned control as an open problem, not a shipped feature.
- **A hardware driver.** The bridges in [`agents/bridges/`](../../agents/bridges/)
  ship with mock drivers. You write the real one.

The precise claim is narrow and worth stating exactly: **the *control
surface* an LLM agent sees is the same in both worlds.** Everything below
that surface still has to be earned on the real robot.

---

## Related

- [Beginner guide — type-talk to a sim robot](omnilink-chat-demos.md) — start here if you've never run a demo.
- [Add your own robot to the OmniLink demos](omnilink-add-your-robot.md) — 5-step recipe.
- Axis bridge contract (`omnisim-bridge.md`, in the OmniLink repo) — the spec sim AND real bridges both implement.
- OmniLink Python client (`omnilink`, in the OmniLink repo) — the library this relay wraps.
