# HuskySwarm — 34-tool OmniLink swarm coordinator

A heavyweight specialist OmniLink agent that controls four Clearpath
Huskies in OmniSim's
[`omnilink_husky_swarm.wbt`](../../../projects/samples/demos/worlds/flagship/omnilink_husky_swarm.wbt)
world. It exists to **fully showcase what an OmniLink agent can do**:
parallel multi-robot execution, persistent spatial memory, learned
routines, cross-session facts, observability, self-discoverable tool
surface, and runner-side safety gates.

The pattern is lifted from OmniLink's own first-party agents (Axis and its assistant sibling)
(tool-registry + tiered manifest + loop budget + persistent SQLite +
meta-tools) and adapted for multi-robot orchestration.

## What's powerful about this demo

The Huskies move on their own — but the **agent runtime** does the
hard parts:

| Capability | How |
|---|---|
| **Parallel multi-robot execution** | `execute_parallel(actions=[...])` dispatches N tool calls concurrently — one tool call, one round-trip, all 4 Huskies in motion. |
| **Persistent spatial memory** | `save_waypoint` / `drive_to_waypoint` — agent builds a vocabulary of named locations that survives OmniSim restarts. |
| **Teach-and-replay routines** | `save_routine(name, steps=[...])` + `run_routine(name)` — once a useful sequence works, it's a one-call replay. Reports elapsed time so the agent can compare runs. |
| **Cross-session facts** | `save_memory` / `recall_memory` — operator prefs, arena features, anything worth carrying forward. |
| **Self-improvement seed** | `report_outcome` / `recall_past_attempts` — the agent reads its own history before trying a known-tricky task. |
| **Self-discoverable surface** | `list_tools(tag?)` / `find_tools(query)` / `invoke_tool(name, args)` — the agent introspects its own 34-tool registry instead of being told everything upfront. |
| **Observability** | `get_activity_log`, `get_swarm_metrics` — distance travelled, halt counts, per-Husky tool-call counters. |
| **Runner-side safety** | Tool-loop budget (12 calls / 30s window, 6 per tool), arena-bound guard (`drive_husky` refuses moves past ±7 m). |

All 34 tools live in `swarm_agent.py`; persistent state lives in
`~/.omnisim/husky_swarm.sqlite3` (override with `HUSKY_SWARM_DB`).

## Tool layers

**Layer 1 — Bridge primitives (11):** `list_huskies`, `get_swarm_state`,
`get_husky_status`, `delegate_to_husky`, `drive_husky`, `turn_husky`,
`set_husky_velocity`, `stop_husky`, `reset_husky_to_home`,
`wait_for_husky_idle`, `halt_all`.

**Layer 2 — Parallel (1):** `execute_parallel`.

**Layer 3 — Waypoints (5):** `save_waypoint`, `list_waypoints`,
`recall_waypoint`, `forget_waypoint`, `drive_to_waypoint`.

**Layer 4 — Routines (5):** `save_routine`, `list_routines`,
`describe_routine`, `run_routine`, `forget_routine`.

**Layer 5 — Memory (4):** `save_memory`, `recall_memory`,
`list_memories`, `forget_memory`.

**Layer 6 — Learning (2):** `report_outcome`, `recall_past_attempts`.

**Layer 7 — Observability + meta (6):** `get_activity_log`,
`get_swarm_metrics`, `reset_swarm_metrics`, `list_tools`, `find_tools`,
`invoke_tool`.

## Showcase prompts

Every prompt below is a **composition** of the 34 primitives —
nothing is pre-baked. A regex-based intent router cannot do any of
these.

```text
# Discovery + parallel
"Check which Huskies are online, then have all four fan out
 1.5 m forward at the same time."

# Persistent waypoints + drive-to
"Save the current spawn pose of each Husky as a waypoint
 (spawn_ne, spawn_nw, spawn_se, spawn_sw). Then mark the centre
 (0, 0) as 'dock'. Have husky_ne and husky_sw drive to dock at
 the same time; the others hold."

# Teach-and-replay routine
"Define a routine called 'sweep_quadrants' that has every Husky
 drive 2 m forward, turn 180, drive 2 m back, in parallel. Then
 run sweep_quadrants and tell me how long it took. Save the
 result so we can compare next time."

# Self-improvement
"Before running sweep_quadrants, check past_attempts for it. If
 the last run failed, slow it down. If it succeeded, try to beat
 the time."

# Cross-session preference
"Remember as a memory that I prefer the swarm to patrol clockwise.
 From now on, when I ask for a 'sweep', do clockwise."

# Fault handling
"Get the swarm state. If any Husky is faulted, halt all and
 surface a warning. Otherwise scatter each Husky 1.5 m further
 from the centre in parallel."

# Meta + discovery
"I want to do something with continuous motion — find the relevant
 tools and explain when each applies."

# Compound choreography
"Build a routine 'corner_dance':
   1. all 4 Huskies drive 2 m toward the centre in parallel
   2. all 4 turn 90 left, in parallel
   3. all 4 drive 1 m forward, in parallel
   4. all 4 return to home
 Save it. Run it. Report metrics. Save the elapsed time as a memory."
```

These work because the agent composes `execute_parallel`,
`save_routine`, `run_routine`, `drive_to_waypoint`,
`recall_past_attempts`, `report_outcome`, `save_memory`, etc. at
runtime. The runner just dispatches.

## How to run

```bash
# 1. Start the world in OmniSim:
#    File → Open World → projects/samples/demos/worlds/flagship/omnilink_husky_swarm.wbt
#    Four Husky bridges come up on ports 8765 / 8766 / 8767 / 8768.

# 2. In a separate terminal, start the HuskySwarm coordinator:
export OMNI_KEY="olink_..."
pip install omnilink
python agents/templates/husky_swarm/swarm_agent.py

# 3. Open https://omnilink-agents.com, pick "HuskySwarm", and chat.
```

The coordinator's HTTP server exposes:
- `POST /tool` — platform tool callback (where OmniLink POSTs the agent's tool calls)
- `GET /health` — alive + tool count + db path
- `GET /metrics` — current per-Husky metrics
- `GET /activity` — last 100 tool calls

## Environment variables

| Variable | Default | Description |
|---|---|---|
| `OMNI_KEY` | _required_ | OmniLink API key. |
| `HUSKY_SWARM_PORT` | `51520` | Tool callback HTTP port. |
| `HUSKY_SWARM_BRIDGES` | (defaults to the 4 ports of the swarm world) | Comma-separated `id=url` list. |
| `HUSKY_SWARM_BRIDGE_TIMEOUT_S` | `10` | Per-call timeout to each Husky. |
| `HUSKY_SWARM_DB` | `~/.omnisim/husky_swarm.sqlite3` | Persistent store path. |
| `HUSKY_SWARM_ARENA_BOUND_M` | `7.0` | Arena half-extent (used by `drive_husky` guard). |
| `HUSKY_SWARM_TOOL_BUDGET` | `12` | Max tool calls in any 30 s window. |
| `HUSKY_SWARM_TOOL_BUDGET_PER` | `6` | Max same-tool calls in the window. |

## Sim-to-real

The coordinator has no Webots imports. Replace `HUSKY_SWARM_BRIDGES`
URLs with real-robot bridges that speak the same Axis-normalised HTTP
surface and the same 34 tools drive your real fleet. See
[`docs/guide/omnilink-sim-to-real.md`](../../../docs/guide/omnilink-sim-to-real.md).
