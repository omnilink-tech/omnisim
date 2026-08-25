# Husky Maze: v1 vs v2 vs v3

Three variants of the same maze-solving agent, each with a different
allocation of work between **the LLM** and **the bridge**.

| | LLM decides | Bridge does |
|---|---|---|
| **v1** | every cell move + every snap + every poll | one motor command at a time |
| **v2** | every cell move (one tool call per cell) | per-move pose-settle wait + drift-gated auto-snap |
| **v3** | the whole BFS plan, once | every cell move with per-cell pre-snap, settle wait, and drift-gated auto-snap |

All three solve the seed-7 `husky_maze.omniworld` world: NW corner -> SE corner,
**73-cell shortest path** (72 moves), perfect maze.

## Architecture diff

### v1 — cell-by-cell with explicit snap

```
                                                      LLM-side (counts)
agent: get_capabilities                                       1 turn
       try_get_known_map                                      1 turn
   for each of 72 cells:
       get_state                  (read pose)
       turn(angle)                (90° pivot if direction change)
       (wait via get_state poll)
       snap_to_cell(prev, target_yaw)
       drive_forward(2 m)
       (wait via get_state poll)
       snap_to_cell(next, target_yaw)
                                  ~4 LLM round-trips/cell
       complete_mission                                       1 turn
                                                      ───────────────
                                                       ~ 290 LLM turns
```

Each move is a sequence of distinct tool calls separated by `get_state`
polls until `mode == "idle"`. The agent owns motion correctness (snap
after every pivot, snap after every drive).

### v2 — one synchronous goto_cell per cell

```
agent: get_capabilities                                       1 turn
       try_get_known_map                                      1 turn
   for each of 72 cells:
       goto_cell(col, row, wait=true,
                 prev_cell=[c,r],
                 auto_snap_threshold_m=0.10)                  1 turn
       (bridge blocks until arrived/faulted)
       (bridge auto-snaps if drift > 0.10 m)
       complete_mission                                       1 turn
                                                      ───────────────
                                                       ~  75 LLM turns
```

Bridge does the pose-settle wait (no `get_state` poll loop), and the
bridge auto-snaps on drift, so `snap_to_cell` is no longer in the agent's
working set. The agent still chooses the next cell each round. The
strategy-selection story (BFS vs lidar wall-follow) is preserved.

### v3 — plan once, execute once

```
agent: get_capabilities                                       1 turn
       try_get_known_map  (driver returns shortest_path)      1 turn
       execute_path(cells=path[1:], speed=0.5)                1 turn
       (bridge drives all 72 cells, pre-snaps each, auto-snaps drift)
       complete_mission                                       1 turn
                                                      ───────────────
                                                       ~   4 LLM turns
```

The LLM commits to the full BFS plan in one tool call. The bridge handles
*everything* about locomotion: pre-snap to cell N's centre with the next
cardinal yaw before each move, run goto_cell, settle, snap on drift.

The pre-snap is what makes batched execution work — without it, the
in-place 90° pivot inside a single `goto_cell` accumulates ~0.5 m of
skid-steer drift and the move blows past its sim_time deadline. v1
masked this by snapping after every pivot; v3 reproduces that anchor
behaviour inside the bridge so the agent never has to think about it.

## Measured metrics (clean runs, seed-7, lib v0.6.1, g1-engine)

| metric | v1 | v2 | v3 |
|---|---:|---:|---:|
| LLM chat turns | 150 (cap) | **75** | **4** |
| Tool calls | ~150 | ~75 | **3** |
| Goal reached | **NO** (stuck near row 6) | **NO** (cell ~(7,1), 90% of path, OmniLink connection aborted on turn 76) | **YES** (cells_executed 72/72) |
| Wall-clock | ~9 min (hit max_turns) | **3 096 s** (~51 min) | **355 s** (~5.9 min) |
| Bridge sim_seconds | n/a (incomplete) | ~2 737 s (~46 min) | **337 s** (~5.6 min) |
| Cells reached | ~30 / 72 | ~65 / 72 | **72 / 72** |
| Snaps inside the bridge | n/a | per cell when drift > 0.30 m (1 turn / cell from agent) | 0 (drift_threshold=0.30; pre-snap absorbed all drift) |
| Approx. input tokens / turn | 4–5 K (flat) | ~4.5 K (flat) | ~5 K |
| Approx. total input tokens | ~700 K (over 150 turns) | ~340 K (over 75 turns) | ~15 K (over 3 turns) |

Notes:
- v1 result is from the `--max-turns 150` run on lib 0.6.1. The agent
  spent the first ~100 turns making BFS progress, then got into a wedge-
  recovery loop around row 6 trying to reach (2,6) and burned the rest
  of the turn budget without finishing.
- v2 result is from the `--max-turns 200` run. Made it 90% of the way
  (cell ~(7,1)) before turn 76's `chat()` call returned a TCP
  connection-reset from omnilink-agents.com. The first 75 turns succeeded
  at ~41 s wall-clock per turn (LLM ~3-5 s + bridge wait for goto_cell
  to settle ~10-30 sim-seconds). The agent occasionally emergency-halted
  and recovered, costing extra turns. Did not reach goal but was
  trending toward completion.
- v3 wall-clock = LLM-side wall-clock for the chat_drive loop. The
  bridge-side `execute_path` itself ran for 337 s of sim_time, and the
  LLM call (chat_drive's `chat()` HTTP) waited that long for the tool
  result to come back. Total = 4 LLM round-trips with one 5+ minute
  tool call in the middle.
- The first v3 run we attempted hit a 5-second `BRIDGE_TIMEOUT` in
  `tools/_base.py` that cut the bridge call short on the agent side
  while the bridge kept driving. The clean number above is from a
  re-run with `BRIDGE_TIMEOUT = 900`.

### Headline numbers

For the same maze (seed-7 known map, 73-cell shortest path), end-to-end
wall-clock (OmniSim launch -> goal reached or run-end):

```
v1   9 min   30/72 cells     150 LLM turns      ~700 K tokens   no goal
v2  51 min   65/72 cells      75 LLM turns      ~340 K tokens   no goal (network drop on turn 76)
v3   6 min   72/72 cells       4 LLM turns       ~15 K tokens   GOAL  ✓
```

v3 used **~47x fewer tokens** and **~37x fewer chat turns** than v1, and
finished while neither v1 nor v2 did. v2 was on track to finish but the
OmniLink shared-pool connection cut out before it could.

## Token economics

The most interesting thing for the OmniLink quota story is *not* the
token cost per call but the **number of LLM round-trips**.

- v1: 150 turns observed at ~4–5 K input tokens each (lib 0.6.1 keeps
  message history flat). That's ~700 K input tokens per attempt, and the
  attempt never reaches the goal.
- v2: 1 turn per cell + bookkeeping ≈ 75 turns × ~5 K = ~375 K.
- v3: 4 turns × ~5 K = ~20 K total.

The shared-pool rate-limit on g1-engine (historical bug report at
commit `af2dd34`, no longer reproducing under lib v0.6.1) was the
binding constraint when we first measured these variants. v3 is the
only variant that fits comfortably under one shared-pool window.

## Tradeoffs

### When v1 is the right choice
- The mission *requires* per-cell agent reasoning. Vision-only worlds
  (`husky_maze_visual.omniworld`): the agent must `read_camera` between cells
  to decide direction. Multi-objective worlds where goal updates
  mid-run.
- You want the activity feed to show the agent's per-cell decisions for
  operator audit. v1's prose narration is one entry per cell; v3's is
  ~4 prose entries for the whole run.

### When v2 is the right choice
- BFS is known but you want an LLM in the loop on each step (e.g.
  conditional re-plans on lidar data, or human-in-the-loop pause/resume
  per cell).
- Lidar wall-follow on the unknown world: each step depends on the new
  lidar reading, so batching the whole route is impossible. v2 is the
  correct shape for that mission.

### When v3 is the right choice
- Mission is a known plan with no per-step contingencies. The seed-7
  known-map case is the canonical fit.
- You're under a rate limit (token quota, RPM cap, paid API budget) and
  every LLM round-trip costs.
- You want a deterministic, repeatable wall-clock — v3's runtime is
  almost entirely the bridge's locomotion time, with no LLM jitter.

## Ports + naming

All three variants live in the **same folder** —
`agents/production/husky_maze/` — and share the same runner script. The
per-variant configuration (profile + system prompt) lives under
`variants/<name>/`. Pick a variant with `--variant`:

| variant | profile name | runner port | variant dir |
|---|---|---|---|
| v1 *(default)* | `Husky Maze`    | 51517 | [`variants/v1`](../variants/v1) |
| v2             | `Husky Maze v2` | 51518 | [`variants/v2`](../variants/v2) |
| v3             | `Husky Maze v3` | 51519 | [`variants/v3`](../variants/v3) |

All three drive the **same bridge** (port 6070). The bridge is fully
backward-compatible: v1's calls still work unchanged; v2 adds the
`wait` / `auto_snap_threshold_m` / `prev_cell` parameters to the existing
`goto_cell` action; v3 adds a new `execute_path` action.

## Reproducing

```
# Bridge: launch OmniSim on the known maze
launch.bat projects\samples\demos\worlds\flagship\husky_maze.omniworld
```

Then for each variant:

```
set OMNI_KEY=olink_...

REM v1 (default — no --variant flag needed)
python agents\production\husky_maze\husky_maze_agent.py
python agents\production\husky_maze\scripts\chat_drive.py --clear-memory ^
       "Solve the maze. Drive the husky from its current cell to the goal."

REM v2 (different port + profile, same script)
python agents\production\husky_maze\husky_maze_agent.py --variant v2
python agents\production\husky_maze\scripts\chat_drive.py --variant v2 --clear-memory ^
       "Solve the maze. Drive the husky from its current cell to the goal."

REM v3
python agents\production\husky_maze\husky_maze_agent.py --variant v3
python agents\production\husky_maze\scripts\chat_drive.py --variant v3 --clear-memory --max-turns 30 ^
       "Solve the maze. Drive the husky from its current cell to the goal."
```

Reset the husky between variants with:
```
curl -X POST http://127.0.0.1:6070/action -H "Content-Type: application/json" -d "{\"action\": \"reset\"}"
```
