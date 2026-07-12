# Why an OmniLink agent (and not just `solve.py`)?

This demo originally shipped with two driver paths:

- [`solve.py`](../solve.py) — a hardcoded BFS executor. Reads the world file, parses the maze graph, computes the shortest path, drives the bridge cell-by-cell.
- [`husky_maze_agent.py`](../husky_maze_agent.py) — an OmniLink agent with the same tools, plus a system prompt and standing orders.

For the *original* task (drive from a known start cell to a known goal in a known maze), `solve.py` is sufficient. So why bother with the agent layer?

This document is the honest answer to that question, in two parts: (1) what each layer does and where the agent earns its keep, and (2) which scenarios the system handles today vs. needs work for vs. needs a different system entirely.

## 1. What each layer is for

### The bridge ([`husky_omnilink_bridge`](../../../../projects/samples/demos/controllers/husky_omnilink_bridge/))

Owns OmniSim — every `Supervisor.getSelf()`, every `motor.setVelocity()`, every `step()`. Exposes a tiny JSON HTTP surface (`/state`, `/capabilities`, `/action`). Implements *primitive* motion only: `set_velocity`, `drive_forward`, `turn`, `goto_cell`, `stop`, `reset`. Plus sensors that the operator controls (lidar via supervisor raycasts, in the dual-strategy build).

The bridge is intentionally dumb. It clamps speeds, runs the heading loop, reports `goal_reached`. It has no opinion on which cell to drive to next, or which strategy to use. That's a feature.

### The script (`solve.py`)

A linear program. Step 1: parse the world. Step 2: BFS. Step 3: for each cell in the path, call `goto_cell`. Step 4: exit on `goal_reached` or fault.

It works perfectly — for the one task it was written for.

### The OmniLink agent (`husky_maze_agent.py`)

A typed tool registry pushed to the OmniLink platform. The agent reasons, picks tools, polls memory, runs standing orders. It's continuously on. An operator can chat with it.

## 2. Where the agent earns its keep

| capability                                | `solve.py` | OmniLink agent |
|-------------------------------------------|------------|----------------|
| drive a known start → known goal          | yes        | yes            |
| natural-language ops ("go to (5,5)")      | no         | yes            |
| replan after a fault                      | aborts     | retries / replans |
| continuous operation, multi-task          | exits      | runs forever   |
| audit trail of decisions + rationale      | print logs | activity feed + memory |
| pick between strategies at runtime        | hardcoded  | yes (the dual-strategy build) |
| compose with other agents                 | no         | yes (other OmniLink agents can call its tools) |
| world-agnostic (different maze, no edits) | edit + rerun | swap the world, agent re-derives |

**The interesting row is "pick between strategies at runtime".** That's the row where the agent stops being optional and starts being load-bearing.

## 3. Generality — what the system handles

| scenario                                            | script handles | agent handles | what it takes                                                                 |
|-----------------------------------------------------|----------------|---------------|------------------------------------------------------------------------------|
| any start position in `husky_maze.wbt`              | yes            | yes           | both BFS from current cell                                                   |
| any maze with the same `H_*`/`V_*` wall convention  | yes            | yes           | lidar build works without the map                                            |
| a maze the agent has *never seen* (no map data)     | yes            | yes           | lidar + wall-following / frontier exploration                                 |
| destination given as natural-language brief         | **NO**         | **YES**       | requires interpreting the brief — `solve.py` only knows hardcoded (10,0)     |
| multi-objective brief (visit corners, patrol)       | **NO**         | **YES**       | the corners-tour world; `mission_complete` requires agent to claim it         |
| different cell sizes / wall shapes                  | partial        | yes           | parameterise the maze parser                                                  |
| arbitrary world (warehouse, outdoor, free obstacles)| no             | partial       | continuous-space planner; agent reasons but needs new motion vocab            |
| unknown world with no supervisor pose               | no             | no            | SLAM, localization, mapping; weeks of work                                    |

## 4. The dual-strategy demo (this build)

To make the agent do something `solve.py` *fundamentally cannot*:

1. The bridge exposes both:
   - `try_get_known_map()` — returns `{available: false}` for the unknown world, the parsed graph for the known one.
   - `read_lidar()` — supervisor raycasts against every `Wall` node's AABB, returns 16 ranges around the husky. **The agent never sees the wall list, only the ranges.**

2. The agent's system prompt instructs it to:
   - First call `try_get_known_map`.
   - If a map is available → BFS from current cell to the goal, drive cell by cell.
   - If not → use `read_lidar` to determine which 4-neighbour cells are open, pick a direction (right-hand rule for guaranteed-solvable mazes; frontier exploration for partial maps), drive one cell, repeat.

3. Two world files in the repo:
   - `husky_maze.wbt` — the seed-7 maze the parser knows.
   - `husky_maze_unknown.wbt` — a different-seed maze; the agent has never seen its wall list.

4. The solver script is now `solve.py` for the known-map case **only**. The unknown-map case has no script equivalent — it's the agent or nothing.

This is the answer to "why an OmniLink agent": the agent is the layer that **chooses** which algorithm to apply based on what it can observe. A script can't do that without becoming an agent itself.

## 5. The mission-brief discriminator (Phase 3 onwards)

The dual-strategy demo proved an agent picks better than a script when the *strategy* is uncertain. But for the strategy-known case (BFS over the seed-7 maze) `solve.py` still works, because the destination is hardcoded as cell `(10, 0)`. To make the demo *only possible* via an agent, the destination itself has to come from natural language.

The bridge now reads `WorldInfo.info` as a **mission brief** and exposes it on `/mission`. Each world ships with its own brief:

- `husky_maze.wbt` — "drive to the goal cell at the SE corner". Trivial brief; both script and agent satisfy it.
- `husky_maze_unknown.wbt` — same destination, different navigation primitive. Both script and agent satisfy it.
- `husky_maze_corners.wbt` — **"visit each of the four corner cells and return to your start"**. Multi-objective. The hardcoded `goal_reached` flag (geometric, fixed (10,0)) cannot represent it. Only the agent's `complete_mission {rationale, claimed_cells}` does.

`solve.py` on `husky_maze_corners.wbt` runs BFS to `(10, 0)`, sets `goal_reached=true`, and exits — `mission_complete` remains `false`. The mission is **not** satisfied.

The agent on the same world reads the brief, says (verbatim):

> *"I need to visit all four corners of the maze: NW (0,10), NE (10,10), SE (10,0), and SW (0,0). I'm starting at NW. After visiting the other three, I'll need to return to NW."*

Then plans a tour and executes it. When done, the agent calls `complete_mission(rationale="visited NW + NE + SE + SW + returned to NW", claimed_cells=[[0,10],[10,10],[10,0],[0,0],[0,10]])` and the bridge logs the claim for operator audit.

A script can be rewritten to do the corners tour, but every new mission shape (riddle, conditional, operator-defined) requires another rewrite. Once the brief is natural language, the agent is the only stable implementation. **That's the agent-only thesis as a load-bearing piece of the system, not as a marketing claim.**

## 6. Vision-only navigation (Phase 3B, landed)

The `husky_maze_visual.wbt` world ships three coloured cylinders (red at `(5,3)`, green at `(3,7)`, blue at `(8,8)`) and a brief that says *"drive to the cell that holds the RED cylinder. The bridge does not tell you which colour is at which cell — you must look through `read_camera`."*

A real OmniSim `Camera` is mounted on a sidecar Robot named `husky_eye` (controller `husky_eye`) that pose-tracks the husky every tick and serves frames over `127.0.0.1:6071`. The main bridge proxies its `/camera` endpoint to the eye. `chat_drive.py` attaches each frame as an inline image part so the engine actually decodes the pixels.

Verified: the agent calls `read_camera`, **sees the rendered frame**, and narrates back: *"From the image, I can see a maze corridor stretching out in front of the husky. The floor has a distinct checkered pattern of light brown and dark brown squares…"*

A script cannot do this. Even a script with a `read_camera` shim gets back base64 PNG bytes and has no way to interpret them without bringing an LLM (or a hand-rolled colour-segmentation pipeline) into the loop. The agent is load-bearing **structurally**, not just for taste.

Three discriminator layers now stack on top of each other:

1. **Strategy choice** — agent picks BFS vs lidar wall-follow from `capabilities.map_available`.
2. **Mission brief interpretation** — agent reads `WorldInfo.info`, infers destinations, claims completion via `complete_mission {rationale, claimed_cells}`.
3. **Visual scene understanding** — on `husky_maze_visual.wbt`, agent decodes camera frames and reasons about colours/shapes/text. This is the genuine pixel-driven discriminator.

Remove any one and the demo still works for the simpler cases. The visual world is the agent-only thesis at maximum strength for *recognition*.

## 6b. The perception-as-tool variant (`husky_maze_blind.wbt`) — what it does and doesn't claim

`husky_maze_blind.wbt` looks at first like "layer 3 turned up to eleven": both `/maze` AND `/lidar` are gated, so vision is the only sensor. But the actual architecture is different in a way that matters.

The husky_eye sidecar runs four cardinal cameras and a pure-Python BGRA analyser. The bridge exposes `scan_surroundings`, which returns ~1.3 KB of structured tags per cardinal (`wall_close`, `wall_close_score`, `marker`, `marker_centroid`, `floor_visible`). The agent **never sees pixels** on this world. It consumes symbolic sensor output the same way it consumes lidar ranges.

That has two honest consequences:

- **The pixel-driven discriminator argument from §6 does not apply here.** `scan_surroundings` *is* the colour-segmentation pipeline that §6 says a script would need to write to clear the agent. We just shipped it as a tool. A non-LLM frontier-explorer could plausibly consume the same tags and solve the maze.
- **What the agent uniquely contributes on this world** is brief interpretation ("RED, not green or blue"), tolerance to noisy perception output (the `wall_close < patch_std 50` heuristic misclassifies in low-contrast lighting), and the same `complete_mission` claiming behaviour from layer B. It is layer 1 + layer 2 + careful symbolic-sensor consumption, not layer 3.

So why ship the blind world at all? Because it makes a different argument worth making: **for cost-sensitive agents on vision-rich worlds, push perception into tools.** Raw `read_camera`-driven navigation cost ~120 K input tokens per cell and burned chat budget at ~30 s/turn × dozens of turns. `scan_surroundings` drops the per-turn cost ~100× by doing the recognition in deterministic local code and serving the LLM a small structured digest. That mirrors how production robotics stacks split learned/classical CV from planning/control — the LLM is the planner, not the perception model.

If you want a strict pixel-driven test that even removes `scan_surroundings`, the blind world's `/scan` endpoint is gated by a single capability flag and could be turned off in a one-line change — but the cost shape would make it impractical to run, which is precisely the point the perception-as-tool architecture solves.

## 7. What this still doesn't show

- **Cross-agent orchestration** — a planner agent calling `Husky Maze` as a sub-skill alongside another specialist (e.g. `Warehouse Picker`) on a multi-robot task.
- **Operator-in-the-loop** — pausing mid-mission to ask "I see two equally-good frontiers, which do you want?".

Each is a clean next demo. Phase 4 adds composability.
