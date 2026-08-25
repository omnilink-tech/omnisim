# Overview — OmniLink agent integration in OmniSim

A balanced description of what this integration is, what it actually does, where it works, where it doesn't, and the cost-and-constraint shape of running it day-to-day. No marketing.

## TL;DR

We took OmniSim's existing simulator (Webots fork) and bolted on:

1. A small HTTP **bridge** that exposes an OmniSim robot's pose + sensors + motion primitives over JSON.
2. An OmniLink **agent profile** that calls those primitives via tools.
3. A series of **maze worlds** with progressively harder briefs that force the agent to do more of the work.
4. A **sidecar Robot** (`husky_eye`) that owns a real OmniSim Camera and tracks the husky's pose, giving the agent vision input.

End-to-end, an agent on the OmniLink platform can read a natural-language mission brief, look at camera frames, plan a path through a maze, drive a Clearpath Husky cell-by-cell, persist what it learned for next session, and claim mission completion with a rationale the operator can audit.

There are 5 worlds, 20+ tools, and one main blocker we hit (OmniSim's `Supervisor.exportImage` returning sky-only frames) that we worked around with a sidecar Robot. The fifth world (`husky_maze_blind`) shows the perception-as-tool architecture explicitly: the agent navigates from symbolic tags emitted by a sidecar CV pipeline, not from pixels.

## What's in the repo

| path | what |
|---|---|
| `projects/samples/demos/controllers/husky_omnilink_bridge/` | The main bridge controller. Owns the husky's motors + supervisor handle, exposes HTTP on `127.0.0.1:6070`, ~1100 lines. |
| `projects/samples/demos/controllers/husky_eye/` | Vision sidecar controller. Owns the Camera, tracks the husky, serves PNG frames on `127.0.0.1:6071`, ~280 lines. |
| `projects/samples/demos/worlds/flagship/husky_maze.omniworld` | Trivial brief world (drive to SE goal). Both script and agent satisfy it. |
| `projects/samples/demos/worlds/flagship/husky_maze_unknown.omniworld` | Same destination but map gated → lidar wall-follow. |
| `projects/samples/demos/worlds/flagship/husky_maze_corners.omniworld` | Multi-objective brief. **Agent only.** |
| `projects/samples/demos/worlds/flagship/husky_maze_visual.omniworld` | Vision-only marker ID — find the red cylinder by sight, then BFS to it. **Agent only, structurally.** |
| `projects/samples/demos/worlds/flagship/husky_maze_blind.omniworld` | Map AND lidar gated. Agent navigates from `scan_surroundings` (sidecar-side perception → ~1.3 KB tags per cardinal). Architectural cost-shape demo: perception lives in the tool, agent does brief interpretation + symbolic navigation. |
| `agents/production/husky_maze/` | The OmniLink agent: profile, prompts, knowledge, long-term memory, 29 tools, runner, standalone solver. |

## Architecture, in three layers

```
┌─────────────────────────────────────────────────────────────┐
│ Operator                                                     │
│   ▼                                                          │
│ OmniLink platform (chat() endpoint)  ─── network ───►        │
│   ▼                                                          │
│ Husky Maze agent profile (system instruction + tools list)   │
│   ▼ engine emits toolCalls                                   │
│ chat_drive.py (the local tool-execution loop)                │
└──────────────────────┬──────────────────────────────────────┘
                       │ POST /tool        ┌──────────────────┐
                       ▼                   │ husky_eye        │
┌─────────────────────────────┐ proxies   │ supervisor       │
│ husky_omnilink_bridge       │  ────────►│   ▼              │
│ (the URDFRobot's controller)│           │ Camera device    │
│ HTTP @ :6070                │           │ HTTP @ :6071     │
└──────────┬──────────────────┘           └──────────────────┘
           ▼ OmniSim Supervisor / motors
┌─────────────────────────────┐
│ OmniSim simulator           │
│   URDFRobot (husky)         │
│   Walls, markers, viewport  │
└─────────────────────────────┘
```

### Layer 1 — bridge (the simulator side)

A pure Python Supervisor controller. Owns:

- The husky's four wheel motors (skid-steer kinematics).
- Pose readout via `Supervisor.getSelf()` → walks the URDF subtree to find `base_link` (the URDFRobot wrapper itself returns NaN).
- A 16-ray supervisor-raycast lidar against every `Wall` node in the scene. The agent sees only ranges; the bridge holds the wall list.
- The maze adjacency graph derived from those walls (only revealed when the world's title doesn't say "Unknown").
- A small set of motion primitives: `set_velocity`, `drive_forward`, `turn`, `goto_cell`, `stop`, `reset`, `snap_to_cell`. All clamped to wheel/angular limits. Each has a settle-gate so commands don't stack momentum across cells.
- The mission brief loaded from `WorldInfo.info`, plus a `complete_mission` action the agent calls to claim completion.
- An `/admin/reload` endpoint that hot-reloads the world (so a developer driving via tools doesn't have to touch the OmniSim window).

### Layer 2 — eye sidecar (the vision side)

A second OmniSim Robot named `husky_eye` with its own controller. Why a second Robot? Because:

- Putting a `Camera` in the husky URDFRobot's `children` field replaces the URDF expansion (verified — wheels disappear, husky has no physics).
- OmniSim's URDF importer doesn't parse Gazebo-style `<sensor type="camera">` tags.
- Adding a Camera node at runtime via `importMFNodeFromString` works at the scene-tree level but OmniSim doesn't re-register devices after world load (`getDevice('front_camera')` returns `None`).
- `Supervisor.exportImage` in this OmniSim build captures sky-only frames (every export is exactly 6 315 387 bytes regardless of viewport).

So the eye is a separate Robot. It owns a real Camera device, finds the husky via `getFromDef("HUSKY")`, walks its subtree for `base_link`/`base_footprint`, and each tick teleports its own translation/rotation to ride 0.55 m above and 0.30 m ahead of the husky's body. Captures frames on demand and serves them as base64 PNG over `127.0.0.1:6071`. The main bridge proxies its `/camera` endpoint to the eye.

This is a workaround, not an elegant design. But it's a deterministic one and it works.

### Layer 3 — agent (the OmniLink side)

A productized OmniLink agent profile pushed to `https://www.omnilink-agents.com`. Mirrors the layout of OmniLink's first-party `axis` agent (in the separate OmniLink repo):

- `profile.json` — engine, standing orders, available commands.
- `prompts/system.md` — the agent's main task: read the brief first, pick a strategy, execute, claim completion.
- `knowledge/` — curated grounding (bridge contract, maze layout). Searched via `search_knowledge`.
- `long_term_memory/` — agent-written notes, hybrid-indexed (Ollama embeddings + BM25 + Reciprocal Rank Fusion). Persists across sessions.
- `tools/` — 29 tools, auto-discovered, each with tier (`safe` / `guarded`) and a single-line `kind`-aware activity-feed classifier.
- `husky_maze_agent.py` — the runner. Pushes the profile, starts a local tool-callback HTTP server on `127.0.0.1:51517`, polls memory, exposes `/status` and `/activity` for operator visibility.
- `scripts/chat_drive.py` — the local tool-execution loop. OmniLink's `chat()` is one-shot (returns `toolCalls`, doesn't dispatch them — the platform can't reach loopback URLs from the internet), so we run the dispatch loop locally and send results back as the next user message. Camera frames are attached as inline `image_url` parts so the engine actually decodes the pixels.
- `solve.py` — the standalone reference solver. Drives the bridge directly, no OmniLink involvement. Useful as a baseline to show what's *possible* without an agent and what genuinely requires one.

## Discriminator layers (what makes the agent worth its keep)

Each successive world adds a layer where the script-only solution either fails or grows into something agent-shaped — except for layer D, which is honestly a different kind of demo (architectural cost-shape) and is called out as such.

### Layer A — strategy choice

`husky_maze.omniworld` (map exposed) and `husky_maze_unknown.omniworld` (map gated) have the same destination — drive from cell `(0, 10)` to cell `(10, 0)`. The bridge's `capabilities.map_available` tells the caller which navigation strategy applies.

- **Script can do this** with hardcoded `if map_available: bfs() else: lidar_wall_follow()`. The `solve.py` in this repo does exactly that. Verified: 72 BFS steps on the seed-7 map; 138 wall-follow steps on the seed-19 map.
- **Agent can also do this** — it reads the same flag, narrates the choice, and runs the same primitives.

The agent doesn't *win* this round, but it doesn't lose either.

### Layer B — mission brief interpretation

`husky_maze_corners.omniworld` ships a multi-objective brief in `WorldInfo.info`:

> *"Mission: visit each of the four corner cells of the maze and then return to your start. The four corners are: NW = (0,10) … NE = (10,10) … SE = (10,0) … SW = (0,0). Call complete_mission with rationale='visited NW + NE + SE + SW + returned to NW' once you finish."*

- `solve.py` runs BFS to its hardcoded `(10, 0)`, sets `goal_reached=true` (legacy geometric flag), and exits — but `mission_complete` stays `false` because the script can't read the natural-language brief. We verified this live.
- The agent reads the brief and in turn 2 of the chat says verbatim: *"I need to visit all four corners of the maze: NW (0,10), NE (10,10), SE (10,0), and SW (0,0). I'm starting at NW. After visiting the other three, I'll need to return to NW."* Then plans the tour and starts driving.

This is the first row in the comparison table where the agent is *necessary* and not just *nice*. A script could be rewritten for any *specific* brief, but every new brief shape requires another rewrite. Once the brief is natural language, the agent layer is the only stable implementation.

### Layer C — visual scene understanding (`husky_maze_visual.omniworld`)

`husky_maze_visual.omniworld` ships three coloured cylinders (red at `(5,3)`, green at `(3,7)`, blue at `(8,8)`) and a brief that says: *"drive to the cell that holds the RED cylinder. The bridge does not tell you which colour is at which cell — you must look through `read_camera`."* Map and lidar stay exposed; vision is only required for marker identification, after which the agent BFS-drives to the chosen cell.

- A script with a `read_camera` shim gets back a 320×240 PNG as base64. To extract "red cylinder is at (5,3)" from those bytes the script needs an image-processing pipeline: decode → segment by colour → identify cylinder shape → estimate world position from camera intrinsics + pose. That's a CV project. Or it needs to call an LLM — at which point it IS an agent.
- The agent calls `read_camera`, the engine decodes the inline image part, and the agent narrates back what's in the frame. Verified live in turn 2: *"From the image, I can see a maze corridor stretching out in front of the husky. The floor has a distinct checkered pattern of light brown and dark brown squares, leading…"*.

On this world the agent layer is *structurally* load-bearing for the recognition step itself. There's no rewrite that gets a script past it without bringing an LLM into the loop.

### Layer D — perception-as-tool, agent does navigation logic (`husky_maze_blind.omniworld`)

`husky_maze_blind.omniworld` is the same maze layout as the visual world but its title triggers the bridge to gate **both** `/maze` and `/lidar`. The husky still has the four-cardinal-camera sidecar, but instead of the agent consuming pixels, the sidecar runs a pure-Python BGRA analyser per frame and the bridge exposes the result as `scan_surroundings` — ~1.3 KB of structured tags (`wall_close`, `marker`, `marker_centroid`, `floor_visible`) per cardinal direction.

This is a **different shape of discriminator from layer C**, and it's worth being honest about what is and isn't agent-only here:

- **The agent is *not* doing vision.** It never sees pixels. It consumes symbol-level perception output the same way it consumes lidar ranges. A non-LLM script could call `scan_surroundings` and run a frontier-explorer over the tags just as well as the LLM does — the perception layer is the part that turns pixels into meaning, and that part is deterministic Python.
- **What the agent *is* doing** is brief interpretation ("RED, not green or blue"), symbolic navigation given sparse and sometimes-wrong perception tags, and decisions about when `marker_fraction > 0.20 AND wall_close == true` justifies calling `complete_mission`. This is layer B (brief interpretation) plus careful handling of an imperfect symbolic sensor — it is not layer C.
- **The architectural point this world makes** is the cost shape. Raw `read_camera`-driven navigation cost ~120 K input tokens per cell (a 320×240 PNG decoded to image tokens) and broke the chat at ~30 s/turn × dozens of turns. `scan_surroundings` drops the per-turn cost ~100× by doing the recognition in deterministic local code and serving the LLM a structured digest. That's the lesson worth taking away: **for cost-sensitive agents, push perception into tools and let the LLM consume symbols.** It mirrors how real robotics stacks split CV (often learned models) from planning/control (often classical or LLM-shaped reasoning).

If you want the strict pixel-driven discriminator, gate `/scan` off too — the visual world (layer C) is also still there. Layer D is honest about being an architecture demo, not a vision demo.

## What works (verified live, in this session)

- Bridge HTTP surface: 12 endpoints across `/state`, `/capabilities`, `/lidar`, `/camera`, `/maze`, `/mission`, `/action`, `/admin/reload`. All round-trip-tested.
- Husky locomotion: turn + drive_forward + snap_to_cell pattern reliably moves the husky cell-by-cell on the 11×11 grid. Bounded ±0.18 m drift per cell with snap-to-grid re-anchoring.
- BFS solver (`solve.py` in BFS mode): seed-7 maze, **goal reached in 72 steps**.
- Lidar wall-follower (`solve.py` in unknown-map mode): seed-19 maze, **goal reached in 138 steps** without ever seeing the wall list.
- World hot-swap via `/admin/reload`: switches between worlds in a few seconds without touching the OmniSim window.
- OmniLink profile push: 20 tools advertised, profile id `d8c10050-…` updated successfully via `client.update_profile`.
- `g1-engine` (Gemini) chat round-trip: agent receives messages, structured `toolCalls` come back, dispatched locally against the runner's `/tool` server, results folded into the next user message.
- Long-term memory: `save_local_memory` → `recall` round-trip via the runner's HTTP. Hybrid retrieval (vector + BM25 + RRF) confirmed working with local Ollama embeddings and a BM25 fallback.
- Mission-brief interpretation on `husky_maze_corners.omniworld`: agent paraphrased the four corners back in turn 2.
- Vision pipeline on `husky_maze_visual.omniworld`: agent called `read_camera` in turn 1, described the maze corridor verbatim in turn 2.
- Bridge `/admin/reload` switching to a different world without restarting OmniSim.
- Per-tool typed activity feed (`info/success/warning/critical` + one-line `detail`) and `/status` synthesis endpoint.

## Limitations (honest)

### Latency

Each agent turn is ~3–5 s of LLM time on g1-engine plus ~3–10 s of bridge motion. A full 73-cell BFS solve via chat would take 5–10 minutes wall-clock. **The OmniLink server enforces a 120 s read timeout per `chat()` call** which cuts long sequences. We routinely hit it after ~28 cells of progress.

The standalone `solve.py` runs the same primitives without the LLM round-trip and finishes in ~90 s. For mechanical execution the script is just faster. The agent is *strategy + first-step*, the script is *full-cell-loop*.

### OmniLink platform quirks

- `chat()` is **one-shot**. It returns intended `toolCalls` but does not dispatch them. The platform documents a `toolCallbackUrl` mechanism, but the OmniLink server can't reach `127.0.0.1:51517` from the internet, so we run the dispatch loop locally in `chat_drive.py`.
- Structured tool-call/tool-role messages get a 400 from the server. We embed tool results as plain text in follow-up user messages instead, which works across engines but loses some structure.
- `g2-engine` (GPT) requires the operator's OpenAI BYOK key. Defaulted everything to `g1-engine` (Gemini), which doesn't.
- The platform sometimes emits no `toolCalls` even when the agent's narration says "I'll call X next". `chat_drive.py` has a one-shot nudge that says "do not narrate, call the next tool now".

### OmniSim specifics that bit us

- **`URDFRobot.children` is destructive when set explicitly** — replaces the URDF expansion. Camera attached this way kills the husky's wheels.
- **URDF importer doesn't parse `<sensor>` tags** — the `OmUrdfImporter.cpp` (1302 lines) has no sensor-tag handling.
- **No runtime device registration** — `importMFNodeFromString` adds nodes but `getDevice` only sees what was registered at world load.
- **`Supervisor.exportImage` produces sky-only frames in this build** — every export is exactly 6 315 387 bytes regardless of viewport, world, or window state. Worked around with the `husky_eye` sidecar.
- **Pose source confusion** — `URDFRobot.getPosition()` returns NaN for the wrapper, so we walk the subtree for `base_link`. With a Camera in `children` (which we no longer do), the fallback `find_first_physical_child` would pick the camera's local translation as "the husky's position".
- **Skid-steer pivots drift** — OmniSim's wheel friction makes 90° pivots accumulate ~0.5 m of body translation per turn. We added `snap_to_cell` to re-anchor to the grid after each step. This is a deliberate demo concession; a real-world controller wouldn't need it.

### Costs

- Each `chat()` round-trip costs OmniLink credits (and Anthropic/Google API spend on the OmniLink-side BYOK if applicable). A full BFS solve at chat cadence is dozens of turns × non-trivial token counts (each turn carries the full conversation history).
- Inline camera frames cost more — a 320×240 PNG is ~100 KB and contributes to per-turn token usage on vision-capable engines.
- Local Ollama embedding (for `local_memory`) requires a running Ollama daemon; falls back to BM25-only when it's not available.

### Things we did NOT verify

- Standing orders firing under load (telemetry_tick, fault_watchdog) — the runner has the polling shape, but a multi-hour stress test wasn't done.
- Cross-agent composability (a planner agent calling Husky Maze as a sub-skill alongside Axis) — the `/status` endpoint is the natural query surface but we didn't build the planner.
- Real-world transfer (real Husky robot, real cameras) — the entire stack assumes OmniSim and a known maze grid. Continuous-space worlds (warehouse, outdoor) need different motion primitives.
- Multi-husky concurrent runs — the bridge binds port 6070 fixed; multi-husky needs port multiplexing.

## Where the agent is overkill

For `husky_maze.omniworld` with the trivial "drive to (10, 0)" brief, the agent runs ~28 cells in 28 chat turns and 5 minutes wall-clock. The script runs all 72 cells in ~90 seconds with no platform dependency. **Use the script.** The agent layer for this world is a more expensive way to get the same outcome.

For `husky_maze_unknown.omniworld` (lidar wall-follow, same destination), the same logic applies. The wall-follower is a 138-step mechanical loop with no decisions an LLM adds value to.

## Where the agent earns its keep

For `husky_maze_corners.omniworld` and `husky_maze_visual.omniworld`, the agent is doing work no script can do — interpreting natural language, reasoning about visual content, deciding when to claim completion. Even if the chat-loop latency is high, *there is no faster path that produces the same outcome*. The script-only path produces the wrong outcome (`mission_complete=false`).

For `husky_maze_blind.omniworld` the honest claim is narrower: the perception layer (deterministic Python in the husky_eye sidecar) is doing the recognition, and a non-LLM script could in principle consume the same `scan_surroundings` tags. What the agent earns its keep on here is brief interpretation (knowing "RED" matters, not green or blue) plus tolerant navigation over imperfect symbolic perception — and it does so at ~100× the per-turn cost of the raw `read_camera` path. The blind world is best read as the **architecture demo** for cost-effective agents on vision-rich worlds, not as a stronger version of layer C.

For *future* worlds with operator-defined missions (a chat operator says "drive to where the human is sitting" or "patrol until I tell you to stop"), the agent layer is the only entry point — there's nothing to predeclare in a script.

## Which engines work

| engine | works | notes |
|---|---|---|
| g1-engine (Gemini) | ✅ default | No BYOK required. Good vision support. |
| g2-engine (GPT) | needs BYOK | Returns 402 BYOK_REQUIRED if no OpenAI key on the operator's OmniLink account. |
| g3-engine (Grok) | untested | |
| g4-engine (Claude) | untested | Should work for vision; may also need BYOK depending on account. |

## Numbers worth knowing

- **29 tools** registered (**15 safe, 14 guarded**) — re-derived from `tools.load_all()`, not hand-counted.
- **5 worlds** with progressive discriminator layers.
- **3 controllers** in the OmniSim side: husky_omnilink_bridge, husky_eye, plus the existing `<none>` placeholder.
- **2 HTTP services** local: bridge :6070 and eye :6071.
- **1 OmniLink profile** pushed to `omnilink-agents.com`, id `<profile-id>`.
- **120 s** OmniLink server-side read timeout per `chat()` call.
- **~3–10 s** bridge motion per cell (turn + drive + snap).
- **~3–5 s** LLM time per chat turn.
- **±0.18 m** typical pose error per cell with snap-to-grid re-anchoring.
- **0.99 m/s** max linear husky speed; **3.47 rad/s** max angular.

## Future paths

The structured menu of next-step options — with effort estimates, wow factor, risks, and decision matrices by audience and time-box — lives in [`DIRECTIONS.md`](DIRECTIONS.md). Read that document when deciding what to push next.

A short list for context:

- **Cross-agent composability** — a planner agent that calls Husky Maze as a sub-skill alongside another specialist (e.g. `Warehouse Picker`). The `/status` endpoint is ready as the query surface.
- **Operator-in-the-loop dialogue** — agent pauses mid-mission to ask "two equally-good frontiers, which?". OmniLink's chat shape supports this but the demo doesn't exercise it.
- **Real OmniSim Camera in the URDF** — would require writing C++ in `OmUrdfImporter.cpp` to parse `<sensor type="camera">` tags. Removes the sidecar.
- **Continuous-space worlds** — strip the cell grid, navigate by free-space waypoints. Needs RRT/A\* or potential fields in the bridge.
- **Multi-husky** — port multiplexing on the bridge; one bridge per husky or one bridge for many.
- **Vision-only worlds with hidden maps** — strip both `try_get_known_map` AND `read_lidar`, only camera + pose. The current `husky_maze_visual.omniworld` keeps the map available; the strict-vision-only variant is a one-line gate change.
- **Native function-calling round-trips** — currently we embed tool results as text because OmniLink's `chat()` rejects OpenAI-style `tool_calls`/`tool` messages with HTTP 400. Server-side fix or an `agent_chat` shim with engine-specific conversion.

## Honest verdict

**As an OmniLink integration:** it works. Pushing a profile, getting structured tool calls back, dispatching locally, feeding results forward, and seeing the agent reason at each step is all real. The 20-tool surface is comprehensive and the runner mirrors the `axis` reference closely enough that anyone who has worked with OmniLink agents will recognise the shape.

**As a robot demo:** the locomotion is reliable but not pretty. The snap-to-grid concession matters because OmniSim's skid-steer pivots drift. A real robot or a different simulator would let the locomotion stand on its own. The mission-brief and vision discriminators are the parts I'd point at if asked "why is this an agent and not a script".

**As a piece of code:** the bridge is one big file (~1100 lines) that has accumulated decisions. The eye sidecar is small (~280 lines) but exists only to work around an OmniSim quirk. The agent side is well-organised and follows the OmniLink conventions cleanly. The world generator is a single Python script that emits valid `.wbt` files for 4 mission shapes.

**As a thesis:** the agent layer earns its keep at layers B and C of the discriminator stack — interpreting briefs and seeing pixels. At layer A it's a more expensive way to get the same answer the script gives. That's the honest line. We don't claim the agent is always better; we claim there are specific places where it's the only thing that works.

**As an investment:** a few hundred MB of OmniSim simulator state, a single-digit MB of new code, a handful of OmniLink credits to demo the end-to-end loop, and an OmniSim window that needs to stay open and not minimised (else `exportImage` returns sky-only frames — and yes, that took a while to figure out).

## Where to read more

- [`why-an-agent.md`](why-an-agent.md) — the discriminator argument in detail (this is the philosophy file).
- [`RESULTS.md`](RESULTS.md) — verified end-to-end runs with command lines and trace excerpts.
- [`../README.md`](../README.md) — agent quick-start.
- [`../knowledge/husky-bridge.md`](../knowledge/husky-bridge.md) — the HTTP contract.
- [`../knowledge/maze-layout.md`](../knowledge/maze-layout.md) — geometry, world conventions.
- [`../variants/v1/system.md`](../variants/v1/system.md) — the agent's system instruction (per-variant; v2/v3 alongside).
- [`../roadmap.md`](../roadmap.md) — phases done + next.
