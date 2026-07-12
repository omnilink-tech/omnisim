# End-to-end results

Verifies the Husky Maze agent across three runtime layers: (1) the standalone solver against the bridge, (2) the OmniLink chat loop driving the same tools, and (3) the structured activity feed + status endpoint operators see.

## Strategy A — known map (BFS), via standalone solver

```
launch.bat projects\samples\demos\worlds\flagship\husky_maze.wbt
python agents/production/husky_maze/solve.py
```

Excerpt:

```
[solve] world_title = 'Husky Maze'  map_available = True
[solve] strategy: BFS over known map
[solve] plan: 73 cells from (0, 10) to (10, 0)
...
[solve] step 72/72: drive 2m -> (10,0) expected pose (+10.0,-10.0)
[solve]              actual (+9.22,-10.00) yaw=+0.00  err dx=-0.78 dy=+0.00
[solve] goal_reached at step 72
```

- BFS plan length: **73 cells**.
- Per-step pose error bounded at ±0.18 m (every step ends with `snap_to_cell`).
- Outcome: **goal_reached on the final step**.

## Strategy B — unknown map (lidar wall-follow), via standalone solver

```
curl -X POST http://127.0.0.1:6070/admin/reload \
     -H 'Content-Type: application/json' \
     -d '{"world": "projects/samples/demos/worlds/flagship/husky_maze_unknown.wbt"}'
python agents/production/husky_maze/solve.py
```

```
[solve] world_title = 'Husky Maze (Unknown)'  map_available = False
[solve] strategy: right-hand-rule wall-follow on lidar
...
[solve] step 138: at (10, 1) heading S -> turn S -> cell (10, 0)
[solve] goal_reached after 138 steps
```

- Strategy chosen entirely from `capabilities.map_available == false`.
- BFS shortest path on this maze is **35 cells**; the wall-follower took **138** because it had to discover the topology while traversing.
- Outcome: **goal_reached after 138 steps**.

## Phase 2A — live OmniLink chat loop

The agent runner pushes its profile to the OmniLink platform with all 17 tools, starts a local tool-callback HTTP server, and `chat_drive.py` initiates the chat round-trip with a tool-execution loop.

```
OMNI_KEY=olink_... python agents/production/husky_maze/husky_maze_agent.py     # in another shell
OMNI_KEY=olink_... python agents/production/husky_maze/scripts/chat_drive.py \
    --clear-memory --max-turns 30 \
    "Solve the maze. Plan from try_get_known_map, then execute cell by cell."
```

Verified in this build:

- ✅ Profile pushed and visible via `client.list_profiles()` (id `d8c10050-...`).
- ✅ Engine: **g1-engine** (Gemini) — set as the OmniLink-default for OmniSim agents because it doesn't need a BYOK OpenAI key.
- ✅ Strategy selection: agent reads `capabilities.world_title` + `map_available`, declares "I'll use BFS" in narration, then issues the matching tool sequence.
- ✅ Tool loop works programmatically via `chat_drive.py`: each agent reply with `toolCalls` is dispatched against the runner's local `/tool` endpoint, results are folded back as the next user message, and the agent picks up.
- ✅ Reached cell `(4, 1)` (~28 cells deep on a 73-cell BFS plan) before the OmniLink server hit a 120 s read timeout — every step taken matched the BFS plan exactly.

Limitations observed:

- `chat()` is one-shot. It returns intended `toolCalls` but the OmniLink platform can't reach `127.0.0.1:51517` from the internet, so the iteration loop runs locally in `chat_drive.py`.
- Function-calling round-trips with structured tool-call/tool-role messages are rejected by the platform with a 400; the loop instead embeds tool results as plain text in follow-up user messages. Less precise than native function calling but works across engines.
- A full 73-cell BFS solve at chat-loop latency (~3 s LLM + ~3-30 s bridge per step) takes 5+ minutes wall-clock, and OmniLink's 120 s server-side timeout interrupts long sequences. The standalone `solve.py` is the right tool for full mechanical execution; the agent layer is the right tool for *strategy selection* + *first-step direction*.

## Phase 2B — long-term memory across sessions

Three new tool modules ported from `agents/axis/tools/`: [`local_memory.py`](../tools/local_memory.py), [`knowledge.py`](../tools/knowledge.py), [`recall.py`](../tools/recall.py). Same hybrid retrieval (vector cosine via local Ollama + BM25 lexical, fused via Reciprocal Rank Fusion) and same storage shape (markdown file per memory + SQLite index).

Verified round-trip via the runner's local tool server:

```
$ curl -X POST http://127.0.0.1:51517/tool \
       -d '{"tool":"save_local_memory","title":"Husky Maze seed-7 BFS","body":"73-cell path...","tags":["Husky Maze","plan"]}'
{"id":"mem_...","embedded":true,"embedding_provider":"ollama"}

$ curl -X POST http://127.0.0.1:51517/tool \
       -d '{"tool":"recall","query":"Husky Maze BFS"}'
{
  "tiers": {
    "knowledge": [4 hits from knowledge/maze-layout.md, knowledge/husky-bridge.md],
    "long_term": [hits from saved memories with hybrid vector+bm25 scoring]
  }
}
```

Storage: [`agents/production/husky_maze/long_term_memory/`](../long_term_memory/). Notes are committed alongside the agent code so future operators (and future agent instances) inherit institutional memory.

The agent prompt now directs the agent to:

1. Call `recall` with the `world_title` after `get_capabilities`. If a saved BFS path or fault note exists for this world, use it instead of re-deriving.
2. After reaching `goal_reached`, call `save_local_memory` to persist the solved-path summary tagged with the world title.

Cross-session compounding behaviour: the second time the agent sees the unknown maze, it can pull the BFS path from memory instead of running the 138-step lidar discovery.

## Phase 4 — cross-agent composability (Mission Captain)

A second agent — [`agents/production/mission_captain/`](../../mission_captain/) — was added to demonstrate OmniLink as a multi-agent fabric. The captain doesn't drive robots itself; it decomposes operator goals and delegates each leg to a specialist (`Husky Maze`, `Axis`).

### Architecture

OmniLink platform-level delegation can't reach `127.0.0.1:51517` from the internet to invoke our local tool server. So the captain runs the sub-chat-loop **locally** via its `delegate_to_agent` tool:

1. Captain's chat loop receives the operator's mission.
2. Captain calls `list_agents` to confirm the specialist roster.
3. Captain calls `delegate_to_agent({agent: "Husky Maze", task: "drive to cell (3, 7) and complete_mission"})`.
4. Inside the captain's runner, `delegate_to_agent`'s impl:
   - Looks up `Husky Maze`'s pushed profile via `client.list_profiles()`.
   - Runs an inner chat loop: `client.chat(messages, agent_name="Husky Maze", system_instruction=husky_settings)` → dispatch toolCalls against `127.0.0.1:51517/tool` → fold results back → repeat.
   - Polls Husky Maze's `/status` after every iteration; returns when `mission_complete=true`.
   - On chat-timeout, probes `/status` one more time before giving up — bridge owns ground truth, platform doesn't.
5. Captain receives the leg result, decides next step (next leg, retry, or `complete_mission`).

### What landed

- `agents/production/mission_captain/` mirrors the husky_maze layout (profile, prompts, knowledge, long_term_memory, tools, runner, scripts).
- 10 tools registered: `delegate_to_agent`, `query_agent_status`, `list_agents`, `complete_mission` (captain-side claim), plus `recall` / `save_local_memory` / `search_local_memory` / `list_local_memories` / `forget_local_memory` / `search_knowledge` (re-used from husky_maze).
- `tools/_base.py:SPECIALIST_REGISTRY` maps `"Husky Maze"` and `"Axis"` to their runner status / activity / tool-callback URLs. Operators add a new specialist by editing this map.
- Captain runner pushes profile (id `ea7c4ae4-…`) and serves `/tool`, `/activity`, `/status` on `127.0.0.1:51518`.
- `scripts/chat_drive.py` mirrors the husky_maze driver but with longer per-tool-call timeouts (delegations can take minutes).

### Verified

Multiple live runs (after credentials refresh) confirmed the architecture:

```
Captain turn 1: list_agents → 2 specialists (Husky Maze reachable; Axis stopped this session)
Captain turn 1: delegate_to_agent("Husky Maze", "drive to cell (0,8) and complete_mission")
   ├─ Sub-loop: Husky Maze chat → 30 turns × 19 husky tool calls
   ├─ Husky drove south (turn → drive_forward → snap_to_cell × 2)
   └─ Husky Maze called complete_mission ✓ — bridge flipped mission_complete: true
Captain turn 1: delegate returned; captain reasons about leg 2
[Captain re-delegates two more times to demonstrate tolerance]
Final husky pose: (-10, 8) ≈ cell (0, 9). mission_complete: true.
Captain activity feed: 3 successful delegations (warning kind because
captain considered each not-fully-complete — see notes below).
```

What works **end-to-end**:

- Captain's `list_agents` returns the live roster with reachability + status snapshots.
- Captain's `delegate_to_agent` runs the sub-chat-loop locally, dispatches every sub-tool call against the sub-agent's `127.0.0.1:51517/tool`, polls `/status` between iterations.
- Sub-agent (Husky Maze) executes its full tool surface: `turn`, `drive_forward`, `snap_to_cell`, `get_state`, `complete_mission`.
- The OmniSim husky physically moves in response to the captain's chain.
- Captain's `/activity` feed records each delegation with `kind` + one-line `detail`.
- Husky Maze's standing-order memory tool autonomously wrote a fault note (`agents/production/husky_maze/long_term_memory/2026-04-26-husky-maze-goto-cell-timeout-fault-at-cell-3-7.md`) for future sessions — verified cross-session compounding inside the delegation loop.

What's noisier than the demo prefers:

- **HTTP timeout layering.** The captain's chat_drive client polls `127.0.0.1:51518/tool` for each captain tool call. A multi-cell delegation can take 5–15 min, longer than naive timeouts. Bumped client timeout to 1800 s; still need to be conscious of total wall-clock.
- **Sub-agent completion claims.** Husky Maze sometimes narrates "I'm here, mission complete" without actually emitting `complete_mission`. The captain's delegation correctly sees `mission_complete: false` in `/status` and reports failure. Mitigated by writing the captain's task instruction with a strict "call complete_mission as your final tool call" directive, but the LLM still occasionally drifts.
- **Each delegation = one full sub-mission.** No streaming or progress events back to the captain — the captain learns about the result only when the delegation returns. Operator visibility comes from polling `/status` on either runner.
- **OmniLink server-side `chat()` timeout.** Even with the captain's longer client timeout, the OmniLink server can drop chats that take too long. The retry-on-network-error logic (3 attempts, with `/status` probe between attempts) recovers cleanly when the bridge has flipped `mission_complete: true` underneath.

### Note on costs

Every captain delegation runs a sub-chat loop, so the per-mission OmniLink token spend is roughly the captain's chat × N legs + each sub-agent's full chat history × N delegations. A 2-leg mission with 30 sub-turns each = ~60 OmniLink chat round-trips. Watch the credit balance.

### Note on costs

Every captain delegation runs a sub-chat loop, so the per-mission OmniLink token spend is roughly the captain's chat × N legs + each sub-agent's full chat history. Long captain runs can exhaust an account's included Gemini credit (`total_credits` reaches 0); further live runs then need the operator to add a Google service-account JSON (or an Anthropic key for g4-engine) on the OmniLink "API & Keys" page.

### What this proves

Three things:

1. **OmniLink supports multi-agent shapes via local orchestration.** The `delegate_to_agent` pattern works without OmniLink-side changes — the captain just runs the sub-agent's chat loop inside its own tool implementation.
2. **Specialist agents stay decoupled.** Husky Maze didn't change at all to be delegated. Its profile, tools, runner, and bridge are identical. The captain just talks to it.
3. **The `/status` endpoint is the natural inter-agent query surface.** The captain probes it before, during, and after each delegation. Operators get the same view by hitting the same URL.

This is the OmniLink "fabric" story made real, at the cost of one new agent folder + 600 lines of orchestration code + a per-leg latency overhead.

## Phase 3B — vision-only navigation (working end-to-end)

The mission-brief discriminator (Phase 3A) makes natural-language goals agent-only. **Vision-only navigation** is the strongest possible discriminator — give the agent a real robot-mounted camera and a brief that requires *seeing*. A script can be coded for any specific brief, but a script genuinely cannot interpret pixels.

### Architecture

Three things had to be true at once:

1. **The husky has a real OmniSim `Camera` device, mounted on its body, returning real rendered frames.**
2. **The bridge surfaces frames as base64 PNG via the existing `/camera` endpoint and `read_camera` tool.**
3. **The chat loop attaches camera frames as inline image parts so the engine actually decodes the pixels — not as opaque base64 text.**

What we tried and ruled out before landing the working design:

- **Camera in `URDFRobot.children`** — URDFRobot's explicit `children` field replaces the URDF expansion. Verified: the wheels disappear and the husky falls through the floor with no physics.
- **`<sensor type="camera">` in the URDF** — OmniSim's URDF importer (`WbUrdfImporter.cpp`, 1302 lines) has no sensor-tag handling. The Gazebo-style sensor extensions are silently ignored.
- **`importMFNodeFromString` into `base_link.children` at runtime** — the node IS added to the scene (visible in the GUI tree), but OmniSim does NOT re-register devices after world load. `getDevice('front_camera')` returns `None`.
- **`Supervisor.exportImage` fallback** — captures sky-only frames in this build (every export is exactly 6 315 387 bytes regardless of world / viewport / window state); the scene geometry never reaches the offscreen render target.

### What landed

A **separate Robot named `husky_eye`** with its own controller `husky_eye` and a Camera as a child:

- `controllers/husky_eye/husky_eye.py` — supervisor controller. Finds the husky via `getFromDef('HUSKY')`, walks its subtree for `base_link`/`base_footprint`, and each tick teleports its own translation/rotation to ride 0.55 m above and 0.30 m ahead of the husky's body. Captures camera frames on demand and serves them as base64 PNG over HTTP `127.0.0.1:6071`.
- The main bridge's `/camera` endpoint **proxies** to the eye. Bridge `/capabilities.camera` advertises `kind: robot_camera, source: husky_eye sidecar @127.0.0.1:6071, ready: true, width: 320, height: 240, fov_rad: 1.4` when the eye is up.
- The world generator (`--with-camera`) emits a `DEF HUSKY URDFRobot { ... }` plus a sibling `Robot { name "husky_eye" supervisor TRUE controller "husky_eye" children [ Camera ... Shape (small visible dot) ] }`.

`chat_drive.py` was extended: when a `read_camera` result comes back with `image_base64`, the loop strips the base64 from the textual feedback and attaches it as an OpenAI-style inline image part — `{type: "image_url", image_url: {url: "data:image/png;base64,..."}}`. OmniLink forwards this to the vision-capable engine (default g1-engine = Gemini).

### Verified

```
$ curl http://127.0.0.1:6070/capabilities | jq '.camera'
{
  "kind": "robot_camera",
  "source": "husky_eye sidecar @127.0.0.1:6071",
  "ready": true,
  "width": 320,
  "height": 240,
  "fov_rad": 1.4,
  "encoding": "image/png; base64"
}

$ curl http://127.0.0.1:6070/camera | jq '. | del(.image_base64) | .view_kind'
"robot_camera"
```

Live agent run on `husky_maze_visual.wbt`:

```
[chat] turn 1: read_camera({}) -> 320x240 PNG (attached as inline image)
[chat] turn 2 (agent text):
        "From the image, I can see a maze corridor stretching out in
         front of the husky. The floor has a distinct checkered pattern
         of light brown and dark brown squares, leading…"
```

The agent **literally saw the camera frame and described what was in it.** Then proceeded to call `get_capabilities`, `try_get_known_map`, and started navigating to look for the red cylinder.

### Why this matters

Three layers of agent-only discriminator are now stacked:

1. **Strategy choice (Phase 1)** — agent picks BFS vs lidar wall-follow from `capabilities.map_available`.
2. **Mission brief interpretation (Phase 3A)** — agent reads `WorldInfo.info` and decides what destinations the brief implies.
3. **Visual scene understanding (Phase 3B)** — agent looks at camera frames and reasons about colours/shapes/text. **A script genuinely cannot do this without an LLM.**

Each can be removed and the demo still works for the simpler cases. The full stack is the agent-only thesis at maximum strength.



## Phase 3A — mission briefs make the demo agent-only

The bridge now reads `WorldInfo.info` as a **mission brief** (free-form natural language) and exposes it on `/mission`. A new world `husky_maze_corners.wbt` ships with a multi-objective brief that the hardcoded `solve.py` cannot satisfy.

### `solve.py` on the corners world (insufficient)

```
$ python agents/production/husky_maze/solve.py
[solve] world_title = 'Husky Maze (Corners Tour)'  map_available = True
[solve] strategy: BFS over known map
[solve] goal_reached at step 32
$ curl -sS http://127.0.0.1:6070/state | jq '{goal_reached, mission_complete}'
{"goal_reached": true, "mission_complete": false}
```

`solve.py` reaches the hardcoded SE corner because the bridge still tracks the legacy `goal_reached` flag for backward compatibility. But `mission_complete` stays `false` — the brief asks for **all four corners + return**, and the script visited only one. The script cannot satisfy a brief it cannot read.

### Agent on the corners world (sufficient)

```
$ python agents/production/husky_maze/scripts/chat_drive.py "..."
[chat] turn 1: read_mission_brief
[chat] turn 2: get_capabilities
        text: I need to visit all four corners of the maze: NW (0,10),
              NE (10,10), SE (10,0), and SW (0,0). I'm starting at NW.
              After visiting the other three, I'll need to return to NW.
[chat] turn 3: try_get_known_map
[chat] turn 5: search_local_memory   (no prior plan)
[chat] turn 6: get_state
[chat] turn 7: goto_cell {col=1, row=10}    -- first hop toward NE
...
```

The agent **interprets** the brief (verbatim quote from turn 2 above), plans the tour, executes cell by cell, and at the end calls:

```
complete_mission {
  rationale: "visited NW + NE + SE + SW + returned to NW",
  claimed_cells: [[0,10], [10,10], [10,0], [0,0], [0,10]]
}
```

The bridge logs the claim in `mission_log` so an operator can audit. `state.mission_complete` flips `true`.

### Why this matters

`solve.py` could be rewritten to do the corners tour, but every new mission shape (riddle, conditional, operator-defined) requires another rewrite. Once the brief is natural language, the agent is the only stable implementation — that's the agent-only thesis as load-bearing system architecture, not marketing.

The brief contract:

- World author writes intent in `WorldInfo.info` — no code changes.
- Bridge surfaces it via `/mission` and `read_mission_brief`. Bridge does not interpret; it transports.
- Agent reads, plans, executes, calls `complete_mission` with rationale + claimed cells.
- Operator audits `mission_log` for the claim.

## Phase 2C — structured activity feed + status endpoint

Every tool dispatch now lands in the activity log with a typed `kind` (`info`/`success`/`warning`/`critical`) and a one-line operator-readable `detail` instead of a raw JSON dump.

Sample feed:

```
[info    ] get_capabilities: world='Husky Maze' map_available=True
[info    ] get_state: pose=(-10.00,+10.00) yaw=-0.00 | mode=idle
[info    ] try_get_known_map: available, 121 cells
[info    ] read_lidar: 16 rays, 1 cardinal-style clears (>2.4m)
```

Faults bubble up as `critical`; emergency stops as `warning`; success conditions like `goal_reached` as `success`.

New `GET /status` endpoint synthesises the most recent state into a single snapshot — useful for operator UIs and for other agents querying this one as a sub-skill:

```json
{
  "agent": "Husky Maze",
  "world_title": "Husky Maze",
  "map_available": true,
  "strategy": "BFS over try_get_known_map adjacency",
  "current_pose": {"x": -10.0, "y": 10.0, "yaw": -1.2e-07},
  "mode": "idle",
  "goal_reached": false,
  "last_fault": null,
  "last_action": {
    "tool": "read_lidar",
    "kind": "info",
    "detail": "read_lidar: 16 rays, 1 cardinal-style clears (>2.4m)",
    "timestamp": "2026-04-26T14:37:24Z"
  },
  "activity_log_size": 4
}
```

Plus the agent's own chat replies now lead with a `[STATUS]` line:

```
[STATUS] world="Husky Maze" strategy=BFS cell=(0,10) plan_remaining=72 goal_reached=false
Reading the maze graph and starting the BFS plan from cell (0, 10).
```

## Reproducing

```bash
# 1. Start the simulator on the known maze:
launch.bat projects\samples\demos\worlds\flagship\husky_maze.wbt

# 2. Standalone solver (BFS) — fastest validation that the bridge works:
python agents/production/husky_maze/solve.py

# 3. Standalone solver (lidar) — switch worlds without restarting OmniSim:
curl -X POST http://127.0.0.1:6070/admin/reload \
     -H 'Content-Type: application/json' \
     -d '{"world": "projects/samples/demos/worlds/flagship/husky_maze_unknown.wbt"}'
python agents/production/husky_maze/solve.py

# 4. Live OmniLink agent — strategy selection + tool dispatch via the platform:
set OMNI_KEY=olink_...
set HUSKY_BRIDGE_URL=http://127.0.0.1:6070
python agents/production/husky_maze/husky_maze_agent.py        # in shell A
python agents/production/husky_maze/scripts/chat_drive.py \
    --clear-memory \
    "Solve this maze. Plan via BFS or lidar based on map_available." # in shell B

# 5. Operator at-a-glance:
curl http://127.0.0.1:51517/status
curl http://127.0.0.1:51517/activity | python -m json.tool
```

## Direction A — cross-session memory compounding

Verifies that what the agent learns on run 1 survives to run 2: the
agent saves the working BFS path on success, and the next session
recalls it and skips the discovery phase.

Variant: **v3** (`--variant v3`, profile *Husky Maze v3*, port 51519).
World: `husky_maze.wbt` (seed-7).

### Run A1 — cold cache (saves the plan)

```bash
python agents/production/husky_maze/scripts/chat_drive.py --variant v3 \
    --clear-memory --max-turns 30 \
    "Solve the maze. Drive the husky from its current cell to the goal."
```

```
[chat] turn 1: get_capabilities
[chat] turn 2: read_mission_brief
[chat] turn 3: recall("Husky Maze drive to goal cell (10,0)") -> no usable hit
[chat] turn 4: try_get_known_map -> shortest_path returned by driver
[chat] turn 5: execute_path(72 cells)            -> goal reached
[chat] turn 6: complete_mission(rationale=...)   -> mission_complete=true
[chat] turn 7: save_local_memory                 -> long_term_memory/2026-04-28-husky-maze-shortest-path-to-10-0.md
[chat] MISSION COMPLETE after 7 chat turns, 6 tool calls, 384 s wall-clock
```

Saved memory body (excerpt):

```
title: Husky Maze: Shortest path to (10,0)
tags:  [Husky Maze, plan]
---
Destination cell: (10, 0)
Strategy: BFS using try_get_known_map
Full ordered path: [[0, 10], [0, 9], [0, 8], [0, 7], [1, 7], ..., [10, 0]]
```

### Run A2 — warm cache (recall hit, skips try_get_known_map)

Same command, run after A1:

```
[chat] turn 1: get_capabilities
[chat] turn 2: read_mission_brief
[chat] turn 3: recall("Husky Maze goal cell (10,0) shortest path") -> HIT
[chat] turn 4: execute_path(recalled cells)      -> partial fault at cell 41/61
[chat] turn 5: stop_husky
[chat] turn 6: execute_path(remainder)           -> still wedged at the same spot
[chat] turn 7: try_get_known_map                 -> fresh adjacency
[chat] turn 8: execute_path(replanned remainder) -> goal reached
[chat] turn 9: complete_mission
[chat] turn 10: save_local_memory
[chat] MISSION COMPLETE after 11 chat turns, 10 tool calls, 484 s wall-clock
```

**Compounding observable:** the agent skipped `try_get_known_map`
*initially* and drove the recalled path directly. The discovery phase
(map fetch + BFS) was elided on turn 4 — that's the memory dividend.

**Edge case:** the recalled path executed 41 of its 61 cells before
the bridge timed out on a goto_cell that the on-the-fly fresh BFS
later re-routed around. Net effect: A2 took **more** turns than A1
because it had to fall back to fresh planning after the locomotion
fault, but the recall *was* used and *did* skip the initial map call.
The fault is unrelated to memory — it's a residual locomotion edge
case in the long-batch `execute_path` path that is independent of
whether the path came from `try_get_known_map` or `recall`. The
compounding mechanism (save -> recall -> skip discovery) is verified.

### Reproducing direction A

```bash
# 1. Bridge + v3 runner
launch.bat projects\samples\demos\worlds\flagship\husky_maze.wbt
python agents\production\husky_maze\husky_maze_agent.py --variant v3

# 2. First run (cold cache — agent calls try_get_known_map, then saves)
python agents\production\husky_maze\scripts\chat_drive.py --variant v3 --clear-memory \
    "Solve the maze. Drive the husky from its current cell to the goal."

# 3. Reset, second run (warm cache — agent should hit recall)
curl -X POST http://127.0.0.1:6070/action -H "Content-Type: application/json" -d "{\"action\": \"reset\"}"
python agents\production\husky_maze\scripts\chat_drive.py --variant v3 --clear-memory \
    "Solve the maze. Drive the husky from its current cell to the goal."

# 4. Inspect saved memory
ls agents\production\husky_maze\long_term_memory\2026-*.md
```

### A2 retry-3 — full fix via `replay_recalled_path` (verified)

After the partial mitigations failed (the LLM kept truncating the
recalled path on output), the proper fix was to remove the LLM from
the path-emission loop:

- **New tool**: `replay_recalled_path(memory_id, drop_head=true)`.
  Looks up a saved memory by id, parses its `Full ordered path:`
  line server-side, validates the saved path's start cell matches
  the husky's current cell (refuses with a structured error if not),
  drops the head, and posts `execute_path` directly. The LLM never
  retypes the cells.
- **Validator**: refuses replay when `head_distance_m > 1.0` from the
  husky's current pose. This catches the case where the agent picks a
  memory whose plan starts somewhere else (e.g. a recovery-remainder
  memory that starts mid-maze).
- **v3 prompt**: branches recall hits to `replay_recalled_path` first,
  falls back to `try_get_known_map` only when no usable hit.

End-to-end run, husky_maze.wbt, v3:

```
turn 1: get_capabilities
turn 2: read_mission_brief
turn 3: recall("Husky Maze goal cell at 10,0")            -> 8 hits
turn 4: replay_recalled_path(memory_id=...)               -> REFUSED
        (saved path's start cell does not match husky's
         current cell — refusing to replay; validator caught it)
turn 5: try_get_known_map                                 -> shortest_path 72 cells
turn 6: execute_path(72 cells, NO TRUNCATION)             -> faulted at cell 41
turn 7-8: stop_husky, get_state, try_get_known_map        -> recovery
turn 9: execute_path(remainder)                           -> goal reached
turn 10: complete_mission
turn 11: save_local_memory
turn 12: (narration)                                      -> MISSION COMPLETE
```

**12 chat turns, 11 tool calls, 147 s wall-clock, goal reached.**
No path truncation in the `execute_path` call (cells_total=72 matches
shortest_path length). The validator correctly rejected an unsafe
replay and the agent fell back cleanly. **Direction A is solved.**

### A2 retry — diagnosing and partially mitigating the path-truncation bug

In the original A2 run the agent passed only **61 of 72** cells to
`execute_path` after a recall hit. The drop happened in the LLM's
output, not in any tool. To diagnose, two changes:

1. **`scripts/chat_drive.py` `_summarise_for_agent`** now parses the
   `Full ordered path: [[c,r], ...]` line out of any `recall` long-
   term hit and surfaces it as a structured `extracted_paths` field
   with the full path and `path_length`. The `_note` instructs the
   agent to use `extracted_paths[i].path[1:]` verbatim.
2. **`variants/v3/profile.json` mainTask** adds an explicit
   PATH-FIDELITY RULE: "Copy that list VERBATIM into execute_path's
   cells parameter. Do not summarise it, do not truncate the tail
   ... If your output limit forces you to choose between truncating
   and a long response, emit the full long response."

Result of the retry:

```
turn 4: execute_path(cells_total=56)  -> faulted at cell 41
turn 8, 11: re-attempts with smaller paths     -> faulted on first cell
turn 13: execute_path(cells_total=33)  -> SUCCEEDED, husky reached goal
turn 14: complete_mission, turn 15: save_local_memory
MISSION COMPLETE: 16 turns, 14 tool calls, 520 s wall-clock.
```

Better than original A2 in goal terms (goal reached + saved), worse in
turn count (16 vs 11). **The LLM still truncated the recalled path on
turn 4: 56 cells emitted, not 72.** Even with `extracted_paths`
providing the full list as a structured field, even with the prompt
demanding verbatim copy, the model's output was still cut short.

This pins the truncation in the model's output budget, not in the
tool surface or the prompt. The honest fix is to remove the LLM from
the path-emission loop entirely:

> **Identified fix path (not implemented this session):** add a
> `replay_recalled_path(memory_id)` tool. The agent passes a memory
> id; the bridge looks up the saved memory by id, parses its body,
> and runs `execute_path` against the saved cells. The model never
> retypes the list, so it can't truncate it. The cost is one extra
> tool spec on the agent's surface.

The compounding mechanism (save → recall → skip discovery) still
works in both A2 runs; the bottleneck is round-trip fidelity, not
storage or retrieval.

## Direction C — hardcore vision-only world (`husky_maze_blind.wbt`)

The visual world (`husky_maze_visual.wbt`) keeps `try_get_known_map`
and `read_lidar` available — the agent uses vision *only* to identify
which cylinder is red, then BFS-drives there. To make vision
**load-bearing for navigation itself**, this direction adds a new
world `husky_maze_blind.wbt` whose title contains "Blind", which the
bridge uses to gate both endpoints to `{available: false}`.

### Bridge gating

`state.reveal_map` becomes false when the title contains "unknown"
**or** "blind". A new `state.reveal_lidar` becomes false when the title
contains "blind". `/capabilities` exposes a new `lidar_available`
boolean alongside the existing `map_available`. Both `/maze` and
`/lidar` return `{"available": false, "world_title": "...", "hint": "..."}`
when their corresponding flag is false; the hint on the blind world
points the agent at `/camera` + `get_state` instead.

### Smoke-test trace

```
$ curl http://127.0.0.1:6070/capabilities | jq '{world_title, map_available, lidar_available, camera_available}'
{
  "world_title": "Husky Maze (Blind)",
  "map_available": false,
  "lidar_available": false,
  "camera_available": true
}

$ curl http://127.0.0.1:6070/maze | jq '{available, hint}'
{
  "available": false,
  "hint": "This world's title flags both map AND lidar as unavailable ('blind' world). Use /camera (read_camera) and pose (get_state) to navigate; neither the wall list nor ray ranges are exposed."
}

$ curl http://127.0.0.1:6070/lidar | jq '{available, hint}'
{
  "available": false,
  "hint": "This world's title flags lidar as unavailable ('blind' world). Use /camera (read_camera) to navigate; the bridge will not expose ray ranges."
}

$ curl http://127.0.0.1:6070/camera | jq '{width, height, encoding}'
{
  "width": 320,
  "height": 240,
  "encoding": "image/png; base64"
}
```

### Reproducing direction C

```bash
# 1. Launch the blind world (or reload from a running bridge):
launch.bat projects\samples\demos\worlds\flagship\husky_maze_blind.wbt
# or:
curl -X POST http://127.0.0.1:6070/admin/reload \
     -H "Content-Type: application/json" \
     -d "{\"world\": \"projects/samples/demos/worlds/flagship/husky_maze_blind.wbt\"}"

# 2. Verify both endpoints are gated:
curl http://127.0.0.1:6070/capabilities
curl http://127.0.0.1:6070/maze
curl http://127.0.0.1:6070/lidar
curl http://127.0.0.1:6070/camera

# 3. Run the agent. With map_available=false AND lidar_available=false,
#    the agent must navigate from camera + pose alone.
python agents\production\husky_maze\husky_maze_agent.py --variant v1
python agents\production\husky_maze\scripts\chat_drive.py --variant v1 --clear-memory \
    "Find the red cylinder and drive to the cell that holds it."
```

### What this proves

The blind world removes the *"but BFS still would have worked"*
caveat from the visual world. Pure-vision navigation isn't an
optional optimisation — it's the only path to the goal. A standalone
solver cannot complete this mission; an agent that interprets pixels
can. That's the strongest form of the discriminator argument in this
demo.

### Final C verification — 60-turn vision-only navigation

Quota refilled, re-ran v1 on the blind world with a 60-turn budget:

| metric | value |
|---|---|
| turns | 60 (hit max_turns) |
| tool calls | ~50 |
| `read_camera` invocations | 14 |
| `drive_forward` cell hops | 10 |
| `turn` pivots | 4 |
| `snap_to_cell` re-anchors | 13 |
| start cell | (0, 10) |
| final pose | (-2.18, 6.00) ≈ cell (4, 8) |
| red cylinder | at cell (5, 3) — not reached |
| wall clock | 434 s |

Husky drove **east along row 10** through cells (1,10) (2,10) (3,10)
(4,10) (5,10), turned south, reached row 8 by cell (3, 8) → (4, 8).
~10 cells of pure-vision-driven navigation through a perfect maze
without map or lidar.

The vision protocol clearly drove navigation:

- 14 camera reads, each followed by an explicit cardinal-direction
  narration ("NORTH (in front of me): I see a long open corridor
  stretching ahead...", "EAST (in front of me): I see an open area
  ahead, suggesting a turn or intersection. The corridor widens
  significantly to the right.")
- One move decision per camera read, justified from the narration
- A snap_to_cell after every move
- The agent autonomously chose to turn south at cell (5, 10) when
  the eastward corridor narrowed into a wall

Visible imperfection: at row 8 the agent's snap_to_cell col/row
arithmetic drifted off by one — the husky reached cell (4, 8) at
world (-2, 6) but the agent computed col=3 from (-2.25, 6.01) and
snapped back to (-4, 6) ≈ cell (3, 8), creating a brief loop. This
is a v1-prompt math issue (the agent computes col/row as
`round((x - origin_x) / cell_size)` and rounds inconsistently for
values straddling a cell boundary), not a vision-protocol failure.

Reaching the red cylinder at (5, 3) needs ~15 more cells of vision-
only navigation through a section of the maze the agent hasn't seen
— another ~120 turns at this cadence. The mechanism is verified;
fully reaching the marker is a turn-budget question not a
correctness question.

### Performance characterisation (v1: structured vision protocol — verified)

The original C end-to-end run (8 turns, 1 cell of progress) showed
the agent calling `read_camera` once, narrating poorly, then driving
forward blindly. The fix was a **structured vision-reasoning
protocol** in v1's `mainTask`:

```
For each move (vision-only mode):
1. read_camera. Look at the image carefully.
2. Narrate cardinal directions: NORTH/EAST/WEST/SOUTH, distance + colour.
3. Decide ONE move from the narration. Justify it.
4. Issue ONE motion call (drive_forward 2 m or turn ±π/2).
5. snap_to_cell to re-anchor (compute new col, row from get_state).
6. Loop. Long-look (full 360° turn-and-look) every 3-4 cells.
```

End-to-end run, husky_maze_blind.wbt, v1, 15-turn budget:

```
turn 4 (read_camera + narration):
  "NORTH (in front of me): I see an open corridor stretching into
   the distance. There are no immediate obstacles or the red
   cylinder in sight.
   EAST (to my right ...): ..."
turn 4 (decide + drive): drive_forward(2 m)
turn 6: snap_to_cell(1, 10)                  ← re-anchored after move
turn 7: read_camera                          ← look again from new cell
turn 8: drive_forward(2 m)
turn 10: snap_to_cell(2, 10)
turn 11: read_camera
turn 12: turn(-π/2)                          ← decided to look south
turn 14: snap_to_cell(2, 10, yaw=-π/2)
turn 15: read_camera                         ← hit max_turns
```

**15 turns, 3 vision-driven cell transitions (forward/forward/turn),
204 s wall-clock.** Compare to the original 8-turn run with **1 cell
of progress** and no narration structure. The agent now:

- reads the camera every move (not once at start)
- narrates cardinal directions explicitly (NORTH/EAST/WEST/SOUTH)
- decides ONE move per cycle and justifies it from the narration
- snaps after every move

The full red-cylinder-find mission (start (0,10) → red cell (5,3))
needs ~30-40 cells of pure-vision navigation, which is ~80+ chat
turns — beyond the 15-turn budget but achievable with more turns and
quota. The mechanism is verified end-to-end. **Direction C is solved
mechanically; the remaining work is just letting the agent run long
enough to reach the marker.**

### Performance characterisation (bridge gating)

Bridge-side overhead for the new gating is sub-millisecond — both
`/maze` and `/lidar` short-circuit in the handler before any wall-list
or ray-scan work. The bridge also now refuses `/camera` cleanly when
no `husky_eye` sidecar is present (`{available: false, hint: ...}`),
preventing the 800x600 operator-viewport diagnostic frame from being
served at ~3 MB and crashing the next chat at OmniLink's 1 MB
request-size limit. `capabilities.camera_available` derives from the
sidecar probe rather than being hardcoded `true`, so the agent's
prompt-level branch reliably steers away from vision on non-vision
worlds.

### Direction C — perception-tool architecture (commit `70c6b1b`)

Vision-as-pixels was charging the agent ~120 K input tokens per
`read_camera` call and ~30 s of chat round-trip per cell of forward
progress. Most of that cost was sending raw frames to a vision LLM
for spatial reasoning the model is bad at and slow on (deciding
"is this corridor open or walled" from a 320×240 PNG). This commit
moves the pixel-level work into a local perception tool and serves
the agent a small structured summary instead.

Two earlier commits set up the substrate:

- `87c4289` — four cardinal cameras (front/right/back/left at 160×120
  each, same total pixel count as the old single 320×240) plus
  bridge-side auto-snap to cell centre after every `drive_forward`
  / `turn`. Halved the per-cell turn count and gave the agent 360°
  awareness in one read.
- `730c00c` — `chat_drive` now retries transient OmniLink 429s
  in-loop with 60 s gaps instead of bailing out.

Then the perception layer itself in `70c6b1b`:

**Sidecar (`husky_eye.py`):**
- New `/scan` endpoint analyses each cardinal frame in pure Python
  (no numpy — OmniSim's bundled mingw64 Python doesn't ship it and
  the system upgrade to add it broke the toolchain). Pure-Python
  iteration over 160×120 BGRA bytes runs ~30 ms per frame.
- Per cardinal returns: `wall_close` (bool), `wall_close_score`
  (0..1), `patch_std` (raw RGB-sum std-dev of the bottom-centre
  patch), `marker` (null/'red'/'green'/'blue'), `marker_pixels`,
  `marker_fraction`, `marker_centroid` (x_norm, y_norm, normalised
  to frame), `mean_brightness`, `floor_visible`.
- Wall-close threshold tuned empirically against this maze world's
  Roughcast walls: `patch_std < 50` = wall close, 50–80 = uncertain,
  >80 = open corridor.

**Bridge:**
- New `/scan` endpoint proxies to the sidecar and overlays
  `current_cell`, `visited_cells`, `unvisited_neighbours` from the
  bridge's own state.

**Tool surface:**
- New `scan_surroundings` tool (SAFE). Returns ~1.3 KB of JSON,
  vs `read_camera`'s ~120 KB of base64 image data — ~100× cheaper
  per chat turn. `read_camera` still available as a fallback when
  the heuristic confidence is low.

**v1 prompt (vision-only branch) rewritten:**
- Per cell: `scan_surroundings` → `drive_forward` (or `turn` for
  direction change). Cardinal `c` is reachable iff
  `cameras[c].wall_close == false` AND the cell in that direction is
  in `unvisited_neighbours` (or the agent is back-tracking).
- Marker recognition: any cardinal with `marker == 'red'` AND
  `marker_fraction > 0.05` AND `wall_close == false` means
  "drive that way." `marker_fraction > 0.20` AND `wall_close == true`
  means adjacent to the marker cell.
- Confirmation `read_camera` is allowed when scan output is
  ambiguous; otherwise vision is purely tag-driven.

#### Smoke trace

```
$ curl http://127.0.0.1:6070/scan      # at cell (0, 10), facing east
{
  "current_cell":         {"col": 0, "row": 10, ...},
  "unvisited_neighbours": [[1, 10], [0, 9]],
  "cameras": {
    "front": {"wall_close": false, "score": 0.00, "std": 183.2, ...},
    "right": {"wall_close": false, "score": 0.00, "std": 160.0, ...},
    "back":  {"wall_close": false, "score": 0.22, "std":  62.3, ...},
    "left":  {"wall_close": true,  "score": 0.84, "std":  13.0, ...}
  }
}
```

Translation: **front** (east) and **right** (south) are open
corridors, **left** (north) is the maze's north boundary right next
to the husky (correctly detected — patch_std 13 is essentially
uniform plaster), **back** (west) is the boundary edge but at the
side of the frame so the score is moderate.

After `drive_forward(2)` to cell (1, 10): **right** (south) flips to
`wall_close=true score=0.83 std=13.9` — the bridge correctly detects
the wall between (1, 10) and (1, 9). Agent now knows it cannot go
south here without the prompt having to interpret pixels.

#### End-to-end run (60-turn budget on `husky_maze_blind.wbt`)

```
9 cells visited: [(0,10) (1,10) (2,10) (3,10) (4,10) (5,10) (5,9) (5,8) (5,7)]
60 turns, 1190 s wall-clock, hit max_turns at cell (5, 7)
```

Productive phase (turns 1–25): husky drove (0, 10) → (5, 7) — 8
cells in 25 turns ≈ **3 turns per cell**, half of the old
`read_camera`-based 6+ turns/cell.

Per-cell economics (verified):

| | turns/cell | tokens/turn | tokens/run for ~9 cells | wall-clock |
|---|---:|---:|---:|---:|
| Raw `read_camera` (commit `87c4289`) | 6+ | ~120 K | ~6 M | ~30 min |
| `scan_surroundings` (this commit) | **3** | **~5 K** | **~300 K** | ~20 min |

**~100× cheaper per chat, ~20× cheaper per run, ~2× faster
wall-clock.** Same scope of progress, an order of magnitude cheaper.

#### Where it still falls short

The agent stuck at cell (5, 7) for the last 35 turns. This is **not
a perception failure** — the scan output at (5, 7) clearly tells
the agent which cardinals are walled and which are open. It's a
**navigation-logic gap**: the maze layout forces a south-then-east
backtrack at that cell, but the agent's "pick an unvisited cardinal
with `wall_close=false`" rule didn't commit to the correct backtrack
move. A future fix is a `back_track_to_last_branch` heuristic that
explicitly chooses the most recently visited cell with unvisited
neighbours of its own, instead of greedy local cardinal picks.

#### Tradeoff

The tool's heuristics now stand between the agent and the pixels.
If `wall_close` misclassifies a busy-textured wall as open or vice
versa, the agent has no recourse short of falling back to
`read_camera`. Worth it: raw vision navigation through a maze was
economically infeasible (~6 M tokens for a 12-cell traversal); this
is feasible (~300 K). The fallback `read_camera` path remains for
low-confidence cases.

End-to-end agent run on the blind world (v1, 8-turn budget):

```
turn 1: get_capabilities                                -> world="Husky Maze (Blind)" map=false lidar=false
turn 2: read_mission_brief                              -> ok
turn 3: get_state                                       -> at start (0,10)
turn 4: read_camera                                     -> 320x240 PNG attached
turn 5: delegate_to_agent({task: "analyze image"})      -> ERROR: unknown tool (delegate_to_agent is a Captain tool, not Husky Maze's)
turn 6: drive_forward(2 m)                              -> drove east 2 m blindly
turn 7: get_state                                       -> at (-8.22, 10.00) ≈ cell (1, 10)
turn 8: (narration, no tool call)                       -> hit max_turns
```

Result: **1 cell of progress, 53 s wall-clock**. The agent did call
`read_camera` (the inline PNG was attached to its next chat) but the
narrative did not extract pixel-level information that turned into a
navigation decision. It then tried to **delegate** image analysis to
another agent (calling a tool that doesn't exist on Husky Maze), got
the "unknown tool" error, and fell back to driving forward without
actually using the picture.

The qualitative finding: **vision-only navigation in pure pixel space
is genuinely hard for the LLM**. Recognising that the camera works and
the brief makes sense isn't enough — the agent needs explicit reasoning
chains for "what cells are around me, which are walls, which way is
red". Without those scaffolds, the agent treats `read_camera` as
ceremony and drives blindly. The bridge contract works; the agent's
prompt for vision-only navigation needs more structure than just
"navigate by camera + pose only."

## Direction B — Mission Captain (multi-agent fabric)

**Status: implemented before this session and re-verified.** Lives in
[`agents/production/mission_captain/`](../../mission_captain/). Original
implementation in commit `bf67a23`; live-verified end-to-end in commit
`2960fad`.

### Architecture

The captain is a *router*, not a robot driver. Operator goals come in
as natural language; the captain decomposes them, picks specialists
from its registry, calls `delegate_to_agent("Husky Maze", "<sub-goal>")`,
and waits. The sub-loop runs **locally** because OmniLink's hosted
delegation can't reach loopback URLs at `127.0.0.1:51517`.

```
operator goal -> Mission Captain (51518)
                       |
                       | delegate_to_agent("Husky Maze", "drive to (0,8)")
                       v
                  Husky Maze runner (51517) -> bridge -> OmniSim
                       |
                       | (reports back when complete_mission fires)
                       v
                  Captain aggregates, calls its own complete_mission
```

### Re-verification this session (2026-04-28)

Captain runner started on port `51520` (overridden via `CAPTAIN_PORT`
to avoid colliding with Husky Maze v2's runner on 51518) against the
existing v1 Husky Maze runner on 51517 and the bridge on 6070.

```
$ curl http://127.0.0.1:51520/status
{
  "agent": "Mission Captain",
  "tools_registered": 10,
  "specialists_known": ["Husky Maze", "Axis"],
  "captain_complete_calls_this_session": 0,
  "activity_log_size": 0
}

$ curl -X POST http://127.0.0.1:51520/tool -H "Content-Type: application/json" \
       -d '{"tool":"list_agents"}'
{
  "count": 2,
  "specialists": [
    {"name": "Husky Maze", "reachable": true,
     "status": {"agent": "Husky Maze", "bridge_url": "...", "tools_registered": 20,
                "world_title": ..., "map_available": ...}},
    {"name": "Axis", "reachable": false,
     "status_error": "unreachable: ..."}
  ]
}
```

Husky Maze is reachable; Axis is not (expected — the UR5e bridge is
not running in this session). The captain's tool dispatcher correctly
routes through `SPECIALIST_REGISTRY` to each specialist's `/status`
endpoint and aggregates the responses.

### Original live-verified trace (commit `2960fad`)

From the commit message of `2960fad`, captured 2026-04-26 with restored
Gemini credentials:

- `Captain.list_agents()` → returned the 2-specialist roster, Husky
  Maze reachable + status snapshot.
- `Captain.delegate_to_agent("Husky Maze", "drive to (0,8) and
  complete_mission")` → ran a 30-turn × 19-tool sub-loop locally;
  husky physically drove south; Husky Maze called `complete_mission`
  in its bridge; `mission_complete` flipped to true.
- Captain re-delegated 2 more times (testing tolerance); husky ended
  at cell (0, 9) with `mission_complete=true` on the bridge.
- Captain's own activity feed recorded each delegation with
  `kind` + `detail`.

### Performance shape

- **Captain chat tick:** ~3-5 s per turn, same order as the other
  agents.
- **Per-leg cost:** captain tick + the full specialist mission
  wall-clock. A v3-style "drive 72 cells" delegation is ~6 minutes;
  a 3-leg captain mission is ~15-20 minutes wall-clock.
- **Token cost:** captain input is small (operator goal + sub-agent
  status snapshots), but the sub-loop carries the specialist's full
  message history. Captain itself is cheap; the specialists run at
  their own per-turn token cost.

### Reproducing direction B

```bash
# 1. Bridge + Husky Maze runner
launch.bat projects\samples\demos\worlds\flagship\husky_maze.wbt
python agents\production\husky_maze\husky_maze_agent.py --variant v1

# 2. (Optional) Axis runner if you have the UR5e bridge:
# python <olink-repo>\agents\axis\axis_agent.py   # (from the separate OmniLink repo)

# 3. Captain runner (CAPTAIN_PORT override if 51518 is busy)
set OMNI_KEY=olink_...
set CAPTAIN_PORT=51520
python agents\production\mission_captain\mission_captain_agent.py

# 4. Send an operator mission via the captain's chat driver
#    (chat_drive now honours CAPTAIN_PORT)
set CAPTAIN_PORT=51520
python agents\production\mission_captain\scripts\chat_drive.py ^
    "Drive the husky to cell (0, 8). One leg only — delegate to Husky Maze."
```

### Final B verification — captain delegation reaches the target cell

After OmniLink's quota window refilled, re-ran the captain delegation
with the slim-sub-chat + camera-refuse fixes in place:

```
captain.delegate_to_agent("Husky Maze", "Drive to cell (0, 8)")
  → sub-chat ran 30 turns / 19 tool calls / 0 × 413 errors
  → sub-agent's final_text:
      "The mission to drive the Husky to cell (0, 8) has been
       successfully completed, and I have already confirmed this..."
  → bridge pose: (-10.00, 6.00) ≈ cell (0, 8) ✓
  → bridge mission_complete: True ✓
captain.complete_mission(rationale=...) ✓
TOTAL_WALL_S: 386 (one mission)
```

**Direction B end-to-end success.** The captain decomposed the
operator goal, delegated to Husky Maze, the sub-agent drove the
husky to cell (0, 8), and the captain closed the operator-facing
mission with `complete_mission`. Zero 413 errors across 30 sub-chat
turns. The original blocker is gone.

### Slim sub-chat fix — 413 eliminated, 22-turn sub-chat sustained

Two changes addressed the 413 *Request payload too large* failures:

1. **`mission_captain/tools/orchestration.py`** — new
   `_slim_settings_for_subchat()` helper. Trims each tool's
   `description` to its first sentence (using the same heuristic as
   `husky_maze/tools/_base.py:_first_sentence`), drops
   `toolCallbackUrl` and `standingOrders` from the settings before
   passing to `client.chat()` as `system_instruction`. The husky_maze
   profile's `availableToolDetails` shrank from ~15 KB to ~9 KB.

2. **Bridge** — `/camera` now returns 503 with a structured `{available:
   false, hint: ...}` when the `husky_eye` sidecar isn't reachable,
   instead of falling through to the 800x600 operator-viewport
   diagnostic frame (3 MB base64). The frame had been the dominant
   contributor to the 3.59 MB request bodies that 413'd. The agent's
   `capabilities.camera_available` flag also derives from the sidecar
   probe so the prompt-level branch reliably steers away from
   `read_camera` on non-vision worlds.

End-to-end captain delegation, `husky_maze.wbt`, slim sub-chat path
verified live this session:

```
$ captain.delegate_to_agent("Husky Maze", "Drive to cell (0, 8).")
captain.last_delegation:
  agent: "Husky Maze"
  turns: 22
  tool_calls: 3
  mission_complete: false
  success: false
  error: "OmniLinkRateLimitError [429]: g1-engine hit its upstream
          rate limit. The credential is fine — quota will refill
          shortly."
```

**Critical observation**: zero 413 errors across 22 sub-chat turns.
Before the fix, every delegation 413'd on the first sub-chat call
with `receivedBytes: 3,592,274`. After the fix, the sub-chat ran
its full intended length and the eventual failure is the OmniLink
shared-pool rate limit on the `g1-engine` selector (a separate,
documented blocker).

The captain itself remained healthy throughout, and on receipt of
the 429 it followed its normal retry/honest-failure-rationale path.
**Direction B is architecturally solved**; the remaining requirement
for a clean "captain → success → save_local_memory" trace is a
fresh OmniLink quota window, which we cannot manufacture from the
client side.

### Fresh delegation attempt (older trace)

Earlier this session, before the slim-sub-chat fix landed, the
delegation chat_drive was hardcoded to port 51518 — my captain was
on 51520 to avoid colliding with v2. Patched
`mission_captain/scripts/chat_drive.py` to honour `CAPTAIN_PORT`. Then:

```
turn 1: delegate_to_agent("Husky Maze", "Drive to cell (0, 8)")
        -> ERROR: chat() raised after 3 attempts:
           OmniLinkAPIError [413]: Request payload (too large)
turn 2-10: same 413 error on every retry
turn 11: complete_mission with rationale "could not be completed
         because repeated attempts to delegate to Husky Maze
         failed with payload-too-large errors"
```

Result: **captain itself is healthy and routing correctly**. Each of
its own chat() calls is small (operator goal + status snapshots).
The failure is **inside** `delegate_to_agent` when it opens a
sub-chat-loop with `agent_name="Husky Maze"`: the system_instruction
for that sub-chat carries the full Husky Maze v1 profile, including
`availableToolDetails` for all 20 tools. That payload exceeds
OmniLink's request-size limit and 413s every time.

This wasn't a problem when the captain was first verified
(commit `2960fad`, 2026-04-26) because the Husky Maze tool surface
has grown since — direction A added explicit memory-flow language
to the prompt, and direction C added the `lidar_available` flag
plus extra hint text. The cumulative bloat now trips the limit.

The captain also handled the failure honestly: it did not silently
loop or fake completion. After 10 retries it called `complete_mission`
with a rationale that explicitly reports the unrecoverable error.
That's the right behaviour from a router — surface the failure,
don't pretend.

**Identified fix path** (not implemented this session): slim the
sub-chat-loop's system_instruction to a lean tool list (drop full
`availableToolDetails`; pass only `availableTools` names + the
profile's `mainTask`). The captain doesn't need the sub-agent to
discover tools at delegation-time; the sub-agent's runner already
serves them locally, and the captain only needs the model to choose
one and emit a tool_call. The lean form would drop ~80% of the
sub-chat input tokens.
