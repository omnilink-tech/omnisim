# husky_maze

[![Try in cloud](https://img.shields.io/badge/Try%20in%20cloud-omnilink--agents.com-F6E905?labelColor=000000)](https://www.omnilink-agents.com)
[![One-command run](https://img.shields.io/badge/run-python%20scripts%2Fdev%2Fomnisim__run__agent.py%20--agent%20husky__maze-5DADE2?labelColor=000000)](../../../scripts/dev/omnisim_run_agent.py)

> **Reading order:** start with [`docs/OVERVIEW.md`](docs/OVERVIEW.md) for the synthesis (what it is, what works, what doesn't, costs). Then [`docs/why-an-agent.md`](docs/why-an-agent.md) for the discriminator argument. Then [`docs/RESULTS.md`](docs/RESULTS.md) for verified runs. This README is the quick-start.

**Mission.** Drive the Clearpath Husky in an OmniSim maze world to satisfy whatever **mission brief** the operator has set on the world. Briefs are free-form natural language read from `WorldInfo.info`; the agent's value is reading them, interpreting them, choosing the right navigation strategy, and deciding when the mission is satisfied via `complete_mission`.

Five worlds ship with the demo, each adding a discriminator layer:

- [`projects/samples/demos/worlds/flagship/husky_maze.wbt`](../../../projects/samples/demos/worlds/flagship/husky_maze.wbt) — seed-7. Brief: *"drive to the SE goal cell"*. Map exposed → agent runs **BFS**. Trivial brief; both `solve.py` and the agent satisfy it.
- [`projects/samples/demos/worlds/flagship/husky_maze_unknown.wbt`](../../../projects/samples/demos/worlds/flagship/husky_maze_unknown.wbt) — seed-19. Same destination, map gated → agent **wall-follows on lidar**. Both `solve.py` and the agent satisfy it.
- [`projects/samples/demos/worlds/flagship/husky_maze_corners.wbt`](../../../projects/samples/demos/worlds/flagship/husky_maze_corners.wbt) — seed-31. Brief: *"visit each of the four corners and return to your start"*. Multi-objective. `solve.py` reaches the hardcoded SE corner but `mission_complete` stays `false` because it can't read the brief. **Only the agent satisfies it** — by reading the brief, planning a tour, executing, and calling `complete_mission` with a rationale + claimed cells.
- [`projects/samples/demos/worlds/flagship/husky_maze_visual.wbt`](../../../projects/samples/demos/worlds/flagship/husky_maze_visual.wbt) — seed-100. Three coloured cylinders (red, green, blue) + a `husky_eye` sidecar Robot with a real OmniSim `Camera`. Brief: *"drive to the cell that holds the RED cylinder — the bridge does not tell you which colour is at which cell, you must look through `read_camera`"*. Map + lidar still exposed; vision is for marker identification only. The agent calls `read_camera`, the engine decodes the inline image, the agent narrates what it sees and navigates by BFS once it knows which cell is red.
- [`projects/samples/demos/worlds/flagship/husky_maze_blind.wbt`](../../../projects/samples/demos/worlds/flagship/husky_maze_blind.wbt) — seed-100, identical layout to the visual world but title contains "Blind". The bridge gates **both** `/maze` (`map_available: false`) **and** `/lidar` (`lidar_available: false`). The husky_eye sidecar ships **four cardinal cameras** (front/right/back/left at 160×120); a pure-Python BGRA analyser in the sidecar produces ~1.3 KB of structured tags per cardinal (`wall_close`, `marker`, `marker_centroid`, …) and the bridge exposes them as a `scan_surroundings` perception tool. **The agent navigates from symbolic tags, not pixels** — the recognition (is this a wall? what colour is the cylinder? where is it in the frame?) runs as deterministic Python in the sidecar, and the LLM consumes the result the same way it consumes lidar ranges. The agent's contribution on this world is brief interpretation ("RED, not green") and symbolic navigation (which cardinal to commit to next given the tags), not vision. The architecture buys ~100× per-turn cost reduction vs raw `read_camera` and demonstrates a perception-as-tool split that mirrors how production robotics stacks separate CV from planning.

See [`docs/why-an-agent.md`](docs/why-an-agent.md) for the discriminator argument and [`docs/RESULTS.md`](docs/RESULTS.md) for verified end-to-end runs.

## Status

Live OmniLink agent with cross-session memory + structured operator visibility, verified end-to-end:

- Known maze (seed-7): **goal_reached** in 72 steps via BFS (standalone solver).
- Unknown maze (seed-19): **goal_reached** in 138 steps via lidar wall-follow (standalone solver).
- Live OmniLink chat loop drove the BFS strategy through the same tools — `chat_drive.py` initiates the chat, the runner's local tool-callback server dispatches each call, agent narrates with a `[STATUS]` line per reply.
- 17 tools registered: 11 motion + bridge + lidar + map, plus `save_local_memory` / `search_local_memory` / `list_local_memories` / `forget_local_memory` / `recall` / `search_knowledge` for cross-session memory.
- Per-tool typed activity feed (`info`/`success`/`warning`/`critical` + one-line detail) and a `/status` endpoint so operators (and other agents) can see what's happening at a glance.

See [`docs/RESULTS.md`](docs/RESULTS.md) for full traces and [`docs/why-an-agent.md`](docs/why-an-agent.md) for the strategy-selection thesis.

## How it drives

The OmniSim side runs [`husky_omnilink_bridge`](../../../projects/samples/demos/controllers/husky_omnilink_bridge/). The bridge:

- Owns the four wheel motors and the supervisor handle on the Husky URDF
- Exposes a JSON HTTP surface on `127.0.0.1:6070`
- Implements primitive motion (`set_velocity`, `drive_forward`, `turn`, `goto_cell`) plus admin actions (`stop`, `reset`, `snap_to_cell`)
- Serves `/lidar` (16-ray raycasts against `Wall` AABBs) and `/maze` (gated by world title)
- Accepts `/admin/reload` to hot-reload its own code or world-swap, so the operator never has to touch the OmniSim window

The agent:

- Reads `capabilities.map_available` once at session start
- Branches on it: BFS plan + execute, or lidar wall-follow
- For each cell move, runs the same motion pattern: `turn → snap_to_cell (re-anchor) → drive_forward → snap_to_cell (re-anchor)`. The snap is a deliberate concession because skid-steer pivots in OmniSim accumulate ~0.5 m drift per 90° turn — see the bridge contract for the rationale.

## Files

- [`variants/<name>/profile.json`](variants/) — per-variant OmniLink agent-profile (name, mainTask, engine settings)
- [`variants/<name>/system.md`](variants/) — per-variant system prompt with the variant's calling convention and behavioural rules
- [`specs/architecture.md`](specs/architecture.md) — execution model + integration shape
- [`docs/OVERVIEW.md`](docs/OVERVIEW.md) — comprehensive synthesis: architecture, what works, limitations, costs (start here)
- [`docs/why-an-agent.md`](docs/why-an-agent.md) — the discriminator argument (3 stacked layers: strategy / brief / vision)
- [`docs/RESULTS.md`](docs/RESULTS.md) — verified end-to-end runs across all phases
- [`docs/DIRECTIONS.md`](docs/DIRECTIONS.md) — menu of next-step options ranked, with effort/wow/risk and decision matrices
- [`knowledge/`](knowledge/) — bridge contract, maze layout (queryable via `recall` / `search_knowledge`)
- [`long_term_memory/`](long_term_memory/) — agent-written notes, hybrid-indexed (Ollama + BM25 + RRF)
- [`tools/`](tools/) — auto-discovered tools (17): bridge proxies, lidar, map, plus `recall` / `save_local_memory` / `search_local_memory` / `list_local_memories` / `forget_local_memory` / `search_knowledge`
- [`maze.py`](maze.py) — local maze parser used by `solve.py` for offline plan inspection
- [`husky_maze_agent.py`](husky_maze_agent.py) — OmniLink runner (profile push + tool-callback HTTP server + `/status` endpoint + typed activity feed)
- [`solve.py`](solve.py) — standalone dual-strategy solver that drives the bridge without OmniLink
- [`scripts/generate_maze_world.py`](scripts/generate_maze_world.py) — recursive-backtracker maze generator
- [`scripts/chat_drive.py`](scripts/chat_drive.py) — programmatic OmniLink chat driver that runs the tool-execution loop locally
- [`roadmap.md`](roadmap.md) — phased plan

## Running

### Dual-strategy demo without OmniLink

```bash
# 1. Start the simulator on the known maze:
launch.bat projects\samples\demos\worlds\flagship\husky_maze.wbt

# 2. Solve it (BFS strategy, picked from map_available):
python agents/production/husky_maze/solve.py

# 3. Hot-swap to the unknown world without restarting OmniSim:
curl -X POST http://127.0.0.1:6070/admin/reload \
     -H 'Content-Type: application/json' \
     -d '{"world": "projects/samples/demos/worlds/flagship/husky_maze_unknown.wbt"}'

# 4. Solve it again (now lidar wall-follow, same script):
python agents/production/husky_maze/solve.py
```

### As an OmniLink agent

The runner ships with three variants that share this directory but
configure the agent very differently. Pick one with `--variant`:

| variant | profile name | port | strategy |
|---|---|---|---|
| `v1` *(default)* | `Husky Maze` | 51517 | cell-by-cell, agent owns turn / snap / drive / snap |
| `v2` | `Husky Maze v2` | 51518 | bridge-side blocking `goto_cell` with auto-snap (one round-trip per cell) |
| `v3` | `Husky Maze v3` | 51519 | bridge-side `execute_path` batch driver (one round-trip drives the whole BFS plan) |

```bash
set OMNI_KEY=olink_YOUR_KEY
set HUSKY_BRIDGE_URL=http://127.0.0.1:6070

REM v1 (default)
python agents\production\husky_maze\husky_maze_agent.py
python agents\production\husky_maze\scripts\chat_drive.py --clear-memory \
    "Solve the maze. Drive the husky from its current cell to the goal."

REM v2
python agents\production\husky_maze\husky_maze_agent.py --variant v2
python agents\production\husky_maze\scripts\chat_drive.py --variant v2 --clear-memory \
    "Solve the maze. Drive the husky from its current cell to the goal."

REM v3
python agents\production\husky_maze\husky_maze_agent.py --variant v3
python agents\production\husky_maze\scripts\chat_drive.py --variant v3 --clear-memory --max-turns 30 \
    "Solve the maze. Drive the husky from its current cell to the goal."
```

The runner pushes the variant's profile (from `variants/<name>/profile.json`)
to https://www.omnilink-agents.com, starts a local tool-callback server
on the variant's port, and waits for chat instructions. Pick the
matching profile name in the UI, or use `chat_drive.py` to run the loop
locally.

End-to-end measurements and the architectural rationale for each variant
are in [docs/v1_v2_v3_comparison.md](docs/v1_v2_v3_comparison.md).

## Operator expectations

- `stop_husky` is always available and never gated.
- The agent will refuse `goto_cell` to a cell that isn't an immediate 4-neighbour. With a known map, the BFS plan ensures this; with lidar only, the agent confirms the cardinal ray is clear before driving.
- The agent will never claim the goal was reached unless `state.goal_reached == true`.
- After every cell move the agent calls `snap_to_cell` to re-anchor — this is the demo concession that lets the strategy-selection story stay the focus rather than locomotion polish.
