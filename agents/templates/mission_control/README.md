# MissionControl — 6-Husky fleet dispatcher on a campus

A heavyweight OmniLink agent that dispatches a 6-Husky fleet across a
30 m × 30 m simulated logistics campus with 12 named zones. The
operator types free-form natural-language missions in the OmniLink web
UI; the agent decomposes them across the fleet on the fly.

The demo exists to answer a single question: **what does a robot
fleet look like when the operator's interface is a chat box, not a
scripted task list?**

## Why scripted control fails here

The world has 12 zones, 6 robots, and an open instruction space.
Every operator command is a free-form sentence. A regex-based or
hand-coded planner runs into the combinatorial explosion immediately:

| Instruction the operator might say | What scripted control would need |
|---|---|
| *"Send the closest idle robot to the south gate."* | Sort robots by distance, filter by status, dispatch the head. Requires live state queries the moment the command lands. |
| *"Have three robots cover the gates clockwise."* | Read the gate list, pick three idle robots, partition the gates between them in geographic order, dispatch in parallel. |
| *"Forget that, recall everyone."* | Cancel all in-flight tasks, then dispatch to parking. |
| *"Inspect the warehouse and the lab in parallel, tell me what you find."* | Two parallel dispatches with downstream `record_observation` calls so the operator can `list_observations` after. |
| *"Who's at dock_b right now?"* | `get_robot` per robot, filter by position. |
| *"Save my normal sweep as a route, then run it on husky_1 every time I ask."* | Persistent named-route memory + a run primitive. |

Each one of these is unique. There's no finite tool surface a script
can pre-enumerate. The LLM, with 16 primitives, composes any of these
at runtime.

## What ships

  - **World**: [`omnilink_mission_control.wbt`](../../../projects/samples/demos/worlds/omnilink_mission_control.wbt)
    — 30 × 30 m asphalt campus, painted yellow road network, 12 zone
    totems (colored emissive caps on aluminium pillars), four
    visual-only buildings as landmarks, three-light atmospheric setup,
    and 6 Huskies parked at the south edge.
  - **Scene supervisor**: [`mission_control_scene`](../../../projects/samples/demos/controllers/mission_control_scene/)
    — single point of truth. Owns zone catalogue, per-robot live state,
    dispatch queue, named routes, observations, activity log. OmniSim
    Supervisor ops are queued from HTTP handlers and drained by the
    main tick (the Python API isn't thread-safe).
  - **Agent**: [`mission_control_agent.py`](mission_control_agent.py)
    — pushes the MissionControl profile to omnilink-agents.com,
    hosts the `/tool` callback, and translates every tool the LLM
    fires into a request against the scene supervisor.

## The 12 zones

| Name | Position | Color | Purpose |
|---|---|---|---|
| `north_gate` | (0, +12) | yellow | Main entrance, north |
| `south_gate` | (0, -12) | yellow | Service entrance, south |
| `east_gate`  | (+12, 0) | yellow | Vehicle access |
| `west_gate`  | (-12, 0) | yellow | Vehicle access |
| `dock_a` | (-8, +8) | blue | Loading dock A (NW corner) |
| `dock_b` | (+8, +8) | blue | Loading dock B (NE corner) |
| `dock_c` | (-8, -8) | blue | Loading dock C (SW corner) |
| `dock_d` | (+8, -8) | blue | Loading dock D (SE corner) |
| `warehouse` | (0, +5) | gray | Central storage |
| `lab` | (-5, 0) | green | Research lab |
| `cafeteria` | (+5, 0) | orange | Workers' rest area |
| `charging_station` | (0, 0) | white | Robot charging hub, campus centre |

## The 16 tools

**Discovery (5):** `list_zones`, `get_zone`, `list_robots`,
`get_robot`, `distances_to`.

**Dispatch (4):** `dispatch`, `cancel`, `cancel_all`, `reset_fleet`.

**Routes (4):** `save_route`, `list_routes`, `run_route`,
`forget_route`.

**Observations (2):** `record_observation`, `list_observations`.

**Observability (1):** `get_activity_log`.

## Showcase prompts

Every prompt below works at the live `omnilink-agents.com` chat with
the MissionControl profile. None of them has a dedicated tool — they
all compose primitives. A regex-based router would need a unique
handler per phrasing; an LLM with the 16 primitives handles all of
them at runtime:

```text
# Spatial reasoning
"Send the two closest idle robots to dock_b."

# Parallel multi-robot
"Have four robots cover the four gates in parallel."

# Route memory + replay
"Save the perimeter as a route: north_gate, east_gate, south_gate,
 west_gate. Run it on husky_1."

# Conditional + cancellation
"Forget what I asked. Recall everyone to charging_station."

# Multi-stage with status check
"Send husky_3 to inspect the warehouse, then -- only after it arrives --
 send husky_4 to the lab. Tell me when both are done."

# Fleet query
"Where's each husky right now? Just the busy ones."

# Observation + retrieval
"Inspect all four docks in parallel; record what each robot finds.
 When done, give me the summary."

# Pattern dispatch
"Three huskies on gate patrol clockwise, two on dock patrol
 counter-clockwise; the last one holds at charging_station."

# Adaptive replan
"Add the cafeteria to the perimeter route as the last stop and
 re-run it from where husky_1 is now."
```

## Run

```bash
# 1. Start the world (window):
launch.bat projects\samples\demos\worlds\omnilink_mission_control.wbt

# Wait for: 6 husky bridges on :8765-:8770 + scene supervisor on :8780.

# 2. Start the MissionControl agent (separate terminal):
export OMNI_KEY="olink_..."
pip install omnilink
python agents/templates/mission_control/mission_control_agent.py

# Output: "Profile created: MissionControl ..."
# "Ready. Open https://omnilink-agents.com and pick 'MissionControl'."

# 3. Open https://omnilink-agents.com, pick MissionControl, chat.
```

## Verified end-to-end

The agent + scene + world are smoke-tested against the live OmniLink
platform with these 10 scenarios:

  * `list_zones` returns 12 zones with correct coords + colors
  * `list_robots` returns 6 huskies, all idle, at south parking
  * `distances_to("north_gate")` returns sorted list with closest = husky_3
  * `dispatch(husky_3, dock_b)` → husky_3 ends at (8, 8)
  * Four parallel dispatches → 4 huskies at 4 gates simultaneously
  * `save_route` + `run_route(husky_6, corner_sweep)` → husky_6 visits dock_a/b/d/c in order
  * `record_observation` + `list_observations` round-trip
  * `cancel_all` + `reset_fleet` → all 6 huskies back at south parking

## Honest caveat

Husky transport uses supervisor teleport via the scene controller.
OmniSim skid-steer open-loop nav is too noisy for reliable point-to-
point dispatch in this physics setup — closing the loop with PID +
odometry is a real-deployment concern, not the demo's question. The
demo's *control logic* is what the LLM agent is here to prove out.
Both the agent and any future scripted baseline see the same
teleport primitive, so the comparison stays honest.

## Environment variables

| Variable | Default | Description |
|---|---|---|
| `OMNI_KEY` | _required_ | OmniLink API key. |
| `MISSION_CONTROL_PORT` | `51530` | Tool callback HTTP port. |
| `MC_SCENE_URL` | `http://127.0.0.1:8780` | Scene supervisor URL. |
| `MISSION_CONTROL_POLL_S` | `3` | Main-loop idle interval. |

## File layout

| File | Purpose |
|---|---|
| `profile.json` | MissionControl OmniLink profile (mainTask + tool surface) |
| `mission_control_agent.py` | The runner. Pushes the profile + serves the /tool callback. |
| `README.md` | This file. |

Companion files outside this directory:

| File | Purpose |
|---|---|
| `projects/samples/demos/worlds/omnilink_mission_control.wbt` | The campus world. |
| `projects/samples/demos/controllers/mission_control_scene/mission_control_scene.py` | Scene supervisor — owns zones, dispatch, state, routes, observations. |
