# OmniLink chat demos — talk to a robot, watch it move

This guide walks a complete beginner through running OmniSim's
**OmniLink chat demos**: a set of small OmniSim worlds where you open a
chat panel next to a robot, type something in plain English, and the
robot does it. There's one demo per URDF robot in the repo (9 robots,
9 worlds).

You don't need to know Python, OmniSim controllers, or robot kinematics
to get something on screen. Five minutes from "I just cloned OmniSim"
to "the robot drove where I told it to".

---

## What you get

When you open one of these worlds, you see two things:

1. **The robot** in a small dark arena (3D viewport, like any OmniSim world).
2. **A chat-style side menu** ("OmniLink robot console") that opens when
   you right-click the robot → *Show Robot Window*. It has a prompt
   textarea at the bottom, a transcript above, and a status pill at the
   top (`connected` / `thinking` / `acting`).

Type something like `home` or `drive forward 1 meter`, hit Send, and
the robot moves. The transcript shows the agent's reply
("Heading home now.") and the tool call it made under the hood
(`reset_to_home`).

---

## The 9 demos

| World file | Robot | Class | Example prompts |
|---|---|---|---|
| `omnilink_tb3_burger.wbt`   | TurtleBot3 Burger          | 2-wheel diff-drive | `forward 1 m`, `turn left 90 degrees`, `back 50 cm`, `spin`, `stop` |
| `omnilink_tb3_waffle.wbt`   | TurtleBot3 Waffle          | 2-wheel diff-drive | same as Burger |
| `omnilink_tb3_waffle_pi.wbt`| TurtleBot3 Waffle Pi       | 2-wheel diff-drive | same as Burger |
| `omnilink_husky.wbt`        | Clearpath Husky            | 4-wheel skid-steer | `forward 1 m`, `turn around`, `circle`, `stop`, `reset` |
| `omnilink_jackal.wbt`       | Clearpath Jackal           | 4-wheel skid-steer | same as Husky |
| `omnilink_rosbot.wbt`       | Husarion Rosbot            | 4-wheel diff-drive | same as Husky |
| `omnilink_rosbot_xl.wbt`    | Husarion Rosbot XL         | 4-wheel diff-drive | same as Husky |
| `omnilink_spot.wbt`         | Boston Dynamics Spot       | quadruped (poses only) | `stand`, `sit`, `wave hello`, `stop` |
| `omnilink_mavic.wbt`        | DJI Mavic 2 Pro            | quadcopter (aerial) | `takeoff`, `forward 1 m`, `up 2 m`, `turn right 90 degrees`, `land` |

All worlds live at [projects/samples/demos/worlds/chat/](../../projects/samples/demos/worlds/chat/).

> **Spot does not walk *in the chat demo*.** The chat bridge exposes a
> pose-only tool surface — Spot stands, sits, and sways. Open-loop gait
> scripting on this URDF is not reliable enough to ship as a chat verb; see
> [projects/default/controllers/spot_simple_pose/spot_simple_pose.py](../../projects/default/controllers/spot_simple_pose/spot_simple_pose.py).
>
> Spot locomotion is delivered by the **policy pipeline** instead — a learned
> controller, not a chat verb. See
> [projects/policies/](../../projects/policies/) and the canonical status in
> [rl-current-state.md](../developer/rl-current-state.md).

---

## Running your first demo (five minutes)

### 1. Open a world in OmniSim

Pick the simplest demo to start: TurtleBot3 Burger.

- **Windows**: double-click `launch.bat` and pass the world path, or run
  `launch.bat projects\samples\demos\worlds\chat\omnilink_tb3_burger.wbt`.
- **Already inside OmniSim**: *File → Open World →
  `projects/samples/demos/worlds/chat/omnilink_tb3_burger.wbt`*.

Wait for the world to load — you should see a small dark arena with a
tiny black/white robot sitting in the middle.

### 2. Open the chat panel

Right-click the robot in the 3D view → **Show Robot Window**.

A panel appears (usually docked to the right or below the viewport)
showing:

```
OmniLink                    ● connected
Robot:   TurtleBot3 Burger
Class:   mobile base
Agent:   local intent (regex)

Type a command — for example:
  "forward 1 m"
  "turn left 90 degrees"
  "back 50 cm"
  "spin"
  "stop"

[ Tell the robot what to do… ] [Send] [Stop]
```

### 3. Talk to the robot

Type `forward 1 meter` into the textarea and hit Send (or Enter).

You should see:
1. The status pill turns yellow (`thinking…`).
2. A `tool` line appears in the transcript:
   `→ drive_forward · distance=+1.00 m`.
3. The status pill turns orange (`acting…`), then back to green (`idle`).
4. The robot rolls forward in the 3D view.
5. The agent's reply appears: *"Driving forward 1.00 m (~11.2s)."*

Try a few more:

- `turn left 90 degrees`
- `back 50 cm`
- `spin`
- `stop`

That's it. You've talked to a robot.

---

## Switching the agent to a real LLM (OmniLink mode)

The default "agent" is a small regex script inside the bridge — fast,
no setup, but literal. Set one environment variable and the same chat
panel routes your prompts through **[OmniLink](https://www.omnilink-agents.com)**
to a real language model (Gemini by default).

The visible difference: the agent understands paraphrases, multi-step
plans, and questions. "Drive forward about a meter then stop" works.
"Go back to where you started" works on every wheeled base. "Where are
you right now?" gives you a state readout.

### Install the OmniLink Python client

The chat-demo bridges use the canonical OmniLink Python client
(`pip install omnilink`). One-time install:

```bash
pip install -r projects/samples/demos/controllers/_omnilink_relay/requirements.txt
```

This pulls in `omnilink>=0.6.1` plus `truststore` (for AVG/MITM TLS
on Windows) and `requests`. On startup the bridge prints a version
line:

```
[omnilink_relay] using omnilink-lib 0.6.3 (floor 0.6.1)
```

A background thread also checks PyPI for newer versions and prints a
one-time hint if you're behind:

```
[omnilink_relay] note: omnilink-lib 0.6.1 is installed, but 0.6.3 is
                       on PyPI. Upgrade with `pip install -U omnilink`.
```

Silence with `OMNILINK_VERSION_CHECK=0`.

### Get a key

Sign up at [omnilink-agents.com](https://www.omnilink-agents.com),
generate an Omni Key. It starts with `olink_`.

### Launch with the key set

**Windows (cmd):**
```bat
set OMNI_KEY=olink_YOUR_KEY_HERE
launch.bat projects\samples\demos\worlds\chat\omnilink_tb3_burger.wbt
```

**Windows (PowerShell):**
```powershell
$env:OMNI_KEY = "olink_YOUR_KEY_HERE"
.\launch.bat projects\samples\demos\worlds\chat\omnilink_tb3_burger.wbt
```

**Linux / macOS:**
```bash
OMNI_KEY=olink_YOUR_KEY_HERE bin/omnisim-bin projects/samples/demos/worlds/chat/omnilink_tb3_burger.wbt
```

When the bridge starts, its log line tells you which mode is active:

```
[omnilink_mobile_bridge] OmniLink relay ON (agent='OmniSim-tb3_burger')
[omnilink_mobile_bridge] TurtleBot3 Burger ready as 'tb3_burger' layout=2wheel r=0.033 ht=0.080 (OmniLink)
```

And the side menu's agent line now reads **`OmniLink relay (g1-engine)`**.

### Tunable env vars

| Variable | Default | Purpose |
|---|---|---|
| `OMNI_KEY`               | _required_ | Your OmniLink key. Empty → falls back to local regex mode. |
| `OMNILINK_BASE_URL`      | `https://www.omnilink-agents.com` | Override for self-hosted OmniLink. |
| `OMNILINK_ENGINE`        | `g1-engine` | Engine to use. `g1-engine` (Gemini), `g2-engine` (GPT), `g3-engine` (Grok), `g4-engine` (Claude). |
| `OMNILINK_TEMPERATURE`   | `0.1`       | Sampling temperature. |
| `OMNILINK_MAX_TURNS`     | `16`        | Max chat-with-tools turns per prompt. Bigger = the agent can plan multi-step chains. |
| `OMNILINK_HISTORY_LIMIT` | `40`        | How many prior turns the agent sees. Bigger = better continuity ("now go back"). |
| `OMNILINK_TIMEOUT`       | `120`       | Per-request HTTP timeout (s) to OmniLink. Raise if you see "operation timed out" on slow tool turns. |
| `OMNILINK_RETRIES`       | `2`         | Retries for one chat round-trip on a read timeout / network blip / 429 / 5xx. 4xx (auth, BYOK) is never retried. |
| `OMNILINK_VERSION_CHECK` | `1`         | Set to `0` to skip the background PyPI lookup that prints a hint when a newer `omnilink` is available. |
| `OMNILINK_VOICE_OUT`     | `0`         | Set to `1` to synthesize the agent's text reply as MP3 via Chirp3-HD and play it back through the chat panel (in addition to the text). The mic button (click to talk, click to send) is shown whenever OMNI_KEY is set. |
| `OMNILINK_MEMORY`        | `1`         | Short-term memory: the bridge primes `self.history` from `OmniLinkClient.get_memory()` on construction and writes the running transcript back after each turn. Reload the world and the agent picks up at "now go forward another meter". Set `0` to start fresh each time. |
| `OMNILINK_USAGE`         | `1`         | Per-turn token / credit telemetry. The bridge polls the platform's usage rollup before and after each chat turn and emits a `usage` event the chat panel renders as a dashed footer line. `GET /usage` on the bridge exposes the latest cached delta. Set `0` to skip the rollup polls. |
| `OMNILINK_PROFILE_SYNC`  | `1`         | On boot every bridge auto-registers an `OmniSim-<robot_id>` profile on the platform; pick it in the omnilink-agents.com web UI and chat from there. Set `0` to skip the push. |

### What the agent can actually do

Each robot's bridge advertises a small set of structured tools to
OmniLink. The agent has to choose one of these — it can't invent new
tools and it can't bypass safety clamps.

**Mobile bases**:
`drive_forward(distance)`, `turn(angle_rad)`, `set_velocity(linear, angular)`,
`stop_robot`, `reset_to_home`, `get_robot_state`.

**Spot**:
`stand`, `sit`, `wave`, `stop_robot`, `get_robot_state`.

The full schemas (including parameter docs) are in
`build_mobile_tools()` / `build_quadruped_tools()`
in each bridge's Python file under
[projects/samples/demos/controllers/](../../projects/samples/demos/controllers/).

---

## How it works under the hood

```
   you in the side menu
        |
        |  "drive forward 1 meter"
        v
   bridge controller (Python, in OmniSim)
        |
        | -- if OMNI_KEY is set: -->  POST /api/chat to OmniLink
        |                              with the bridge's tool surface
        |                              <-- toolCalls: [{drive_forward, distance: 1}]
        |
        v
   bridge dispatches drive_forward(1.0)
        |
        v
   robot motors spin → robot rolls forward
        |
        v
   "Driving forward 1.00 m" → back to the side menu
```

Three pieces of code, all in this repo:

- **[resources/projects/plugins/robot_windows/omnilink_chat/](../../resources/projects/plugins/robot_windows/omnilink_chat/)** —
  the side-menu UI (HTML + JS + CSS). OmniSim loads it from a robot's
  `window` field. Communicates with the bridge over the standard
  `wwi` (web window interface) message channel.

- **Generic bridges**, one per robot class, in
  [projects/samples/demos/controllers/](../../projects/samples/demos/controllers/):
  - `omnilink_mobile_bridge/` — driven by every wheeled base.
    Parameterised by `_mobile_configs.py` (wheel layout, radii, max speed).
  - `omnilink_quadruped_bridge/` — Spot-specific (poses only).
  - `mavic_omnilink_bridge/` — the DJI Mavic 2 Pro (aerial).

- **[projects/samples/demos/controllers/_omnilink_relay/](../../projects/samples/demos/controllers/_omnilink_relay/)** —
  shared OmniLink integration. Wraps the
  [`omnilink` Python library](https://www.omnilink-agents.com) (already
  installed in this repo's Python — the `omnilink-lib` package from the separate OmniLink repo) in a
  worker-thread relay that runs the chat-with-tools loop and pushes
  status / tool / agent / error events back to the side menu.

### Adding a new robot (about 30 lines of code)

1. **Add a config**: drop a dict into `_mobile_configs.py` with the
   robot's wheel layout, wheel radius, half-track and max speed.
   Adding the entry is usually 10–15 lines.
2. **Add a world**: copy any `omnilink_*.wbt` next to the existing
   worlds, change the `url` to your robot's URDF and the
   `controllerArgs` to your new robot id. Roughly 20 lines.

That's it — no new controller, no new bridge code.

---

## The bridge's HTTP surface (for agents and integration tests)

Each bridge also exposes an HTTP API on `127.0.0.1:8765` that matches
the Axis bridge contract documented in the OmniLink repo's
`agents/axis/knowledge/omnisim-bridge.md`.
The OmniLink Axis agent can drive every one of these demos unchanged
by pointing `AXIS_BRIDGE_URL` at port 8765.

Quick smoke check while a demo is running:

```bash
# What robot is on the wire?
curl -s -X POST http://127.0.0.1:8765/list_robots

# Current state (pose, mode)
curl -s -X POST http://127.0.0.1:8765/get_robot_state

# Send a free-form prompt (routes via OmniLink if OMNI_KEY is set,
# otherwise via the local regex router)
curl -s -X POST -H 'Content-Type: application/json' \
     -d '{"text":"drive forward one meter"}' \
     http://127.0.0.1:8765/prompt
```

The same endpoint set works for every demo — the only thing that
changes per robot is the toolset advertised in `/list_robots`.

---

## Troubleshooting

### The side menu shows "No controller" or stays blank.

The bridge controller isn't running. Check the OmniSim console for an
error from `omnilink_mobile_bridge` / `omnilink_quadruped_bridge` /
`mavic_omnilink_bridge`. The most common cause: the world's
controller path is wrong (it should be `omnilink_*_bridge`, not the
old `*_omnilink_bridge` controllers).

### The robot doesn't move and the status pill is stuck on "thinking".

`OMNI_KEY` is set but OmniLink isn't responding. Causes:
- The key is wrong (you'll see a 401 in the OmniSim console).
- AVG or another local proxy is intercepting TLS — the relay already
  injects the OS trust store via `truststore`, but if AVG is missing
  its root cert in the trust store you'll see a TLS error.
- You're offline — drop the `OMNI_KEY` env var and the bridge will fall
  back to the local regex router.

### "The agent calls the wrong tool" (e.g. asks Spot to walk).

The bridge tells the agent exactly what tools exist via
`availableToolDetails` on each `/api/chat` request, so it can't really
hallucinate a new one. If it's calling the *wrong* tool, the
`main_task` string in the bridge could be sharper — those strings are
in `build_*_main_task()` and are easy to tweak.

### "I edited the bridge but my change didn't take effect".

OmniSim only re-launches the controller when the simulation reloads,
not when you save the file. Use *Reset Simulation* (Ctrl+Shift+T) or
*Reload World* (Ctrl+Shift+R) to pick up a Python edit.

### "I want to drive multiple demos at once".

The bridge HTTP port is `8765` by default — that conflicts if two
demos run side-by-side. Override per world with `controllerArgs
["--port", "8766"]` and so on.

---

## Where to look in the code

- **Side menu UI**: [resources/projects/plugins/robot_windows/omnilink_chat/](../../resources/projects/plugins/robot_windows/omnilink_chat/)
- **Mobile bridge + configs**: [projects/samples/demos/controllers/omnilink_mobile_bridge/](../../projects/samples/demos/controllers/omnilink_mobile_bridge/)
- **Quadruped bridge**: [projects/samples/demos/controllers/omnilink_quadruped_bridge/](../../projects/samples/demos/controllers/omnilink_quadruped_bridge/)
- **OmniLink relay**: [projects/samples/demos/controllers/_omnilink_relay/](../../projects/samples/demos/controllers/_omnilink_relay/)
- **Shared stage PROTO**: [projects/samples/demos/protos/OmniLinkStage.proto](../../projects/samples/demos/protos/OmniLinkStage.proto)
- **Smoke tests** (offline + live OmniLink): [tests/smoke_omnilink_demos.sh](../../tests/smoke_omnilink_demos.sh), [tests/smoke_omnilink_demos_live.sh](../../tests/smoke_omnilink_demos_live.sh)
- **Relay unit tests**: [tests/test_omnilink_relay.py](../../tests/test_omnilink_relay.py)

The wider OmniLink ecosystem (the platform, the Python client library,
Axis the robot-control agent that drives these same bridges from the
OmniLink web UI) lives in the separate OmniLink repository, outside this
repo: the `omnilink-lib` client library (API reference) and the
productized `axis` robot-control agent.

---

## Going further

- **Agent gallery** (visual one-pager): [`docs/showcase/agents.html`](../showcase/agents.html) — every chat demo + specialist agent + the real-robot port in one place.
- **Add your own robot** to the demos: [`omnilink-add-your-robot.md`](omnilink-add-your-robot.md) — 5-step recipe, ~50 lines of new code per robot.
- **Drive a real robot with the same agent**: [`omnilink-sim-to-real.md`](omnilink-sim-to-real.md) — the architecture and the workflow. The *agent control surface* is portable; physics and policy transfer are **not** claimed (that guide's scope box says exactly what does and does not carry over).
- **Example specialist agents** (Roomba, Husky swarm, Mission Control): [`agents/templates/`](../../agents/templates/) — copy-paste starting points for purpose-built OmniLink agents.
- **Real-robot bridge starter kit** (no simulator required): [`agents/bridges/`](../../agents/bridges/) — the sim-to-real seam in runnable form. Arm + mobile stubs that speak the same HTTP surface as the demo bridges, backed by **mock drivers** that log the commands they would send. Swapping in a real driver is your work.
- **omnisim-bridges** pip-installable package: [`packages/omnisim-bridges/`](../../packages/omnisim-bridges/) — `BridgeBase`, `serve_http`, `Tool`, `OmniLinkRelay`, `IntentRouter` lifted out as a standalone library with no simulator dependency. `pip install -e packages/omnisim-bridges/` and `from omnisim_bridges import ...`.
- **Voice I/O**: when `OMNI_KEY` is set the chat panel shows a **mic button** — click to talk, click again to send; the bridge transcribes it (STT) and routes the text as a prompt. Set `OMNILINK_VOICE_OUT=1` to also have the agent reply spoken back via Chirp3-HD.
- **Benchmark suite** (single-prompt task evaluation): [`tests/benchmarks/omnilink_tasks/`](../../tests/benchmarks/omnilink_tasks/) — each task (e.g. `mobile_drive_1m`) launches its world headlessly, posts a prompt, and grades the resulting bridge state. Run with `python tests/benchmarks/omnilink_tasks/run.py`.
