# Roadmap — Husky Maze

## Phase 0 — bootstrap (done)

- [x] OmniSim controller `husky_omnilink_bridge` (HTTP on 127.0.0.1:6070)
- [x] Wire `husky_maze.wbt` to the bridge with `supervisor TRUE`
- [x] OLink-agents folder + `husky_maze` agent scaffold
- [x] Tools: `get_capabilities`, `get_state`, `stop_husky`, `reset_husky`, `set_velocity`, `drive_forward`, `turn`, `goto_cell`
- [x] Standalone `solve.py` that drives the bridge end-to-end without OmniLink
- [x] Headless verification of `husky_maze.wbt` after the controller swap

## Phase 1 — make the agent earn its keep (done)

The original demo only handled "drive A → B with a known map" — a script could do that. Phase 1 added a scenario where the agent's runtime strategy selection is *load-bearing*.

- [x] Add lidar (supervisor raycasts against `Wall` AABBs) on `/lidar`
- [x] Gate `/maze` on `WorldInfo.title` so the agent doesn't see the wall list for "Unknown" worlds
- [x] Generate `husky_maze_unknown.wbt` (seed=19, different layout)
- [x] Update agent system prompt to branch on `capabilities.map_available`:
  - Strategy A — BFS over `try_get_known_map` adjacency
  - Strategy B — right-hand-rule wall-follow on `read_lidar`
- [x] Solver `solve.py` demonstrates both strategies (selected at runtime)
- [x] Bridge `/admin/reload` endpoint so the operator (and the agent) can hot-reload or world-swap without touching the OmniSim window
- [x] `snap_to_cell` action to re-anchor pose after each step (skid-steer pivots in OmniSim accumulate ~0.5 m drift per 90° turn — see `docs/why-an-agent.md`)
- [x] Signed-progress `drive_forward` controller with active brake + settle gate
- [x] End-to-end verification on both worlds — see `docs/RESULTS.md`

## Phase 2 — deepen the OmniLink integration (done)

- [x] Live agent-side run: `husky_maze_agent.py` with `OMNI_KEY` pushes profile, `chat_drive.py` runs the chat loop and dispatches tools locally. Engine standardised on **g1-engine** (Gemini, no BYOK) as the OmniSim-default.
- [x] Wire the `recall` / `local_memory` / `knowledge` / `search_knowledge` tool pattern from `agents/axis/tools/`. Hybrid retrieval (Ollama embeddings + BM25 + RRF). Storage in [`long_term_memory/`](long_term_memory/), indexed in `_index.sqlite`. Round-trip verified via the runner's local tool server.
- [x] Activity-feed polish: typed `kind` (`info`/`success`/`warning`/`critical`) per tool dispatch, with operator-readable one-line `detail` instead of raw JSON. `classify_tool_result()` per-tool in `husky_maze_agent.py`.
- [x] Operator at-a-glance: `GET /status` synthesises strategy + pose + last action + last fault from the activity log. `[STATUS]` narration directive in the system prompt forces every chat reply to lead with a parsable status line.

## Phase 3A — make the demo agent-only via mission briefs (done)

- [x] Bridge reads `WorldInfo.info` as a free-form mission brief; surfaces it on `/mission` and in `/capabilities.mission_brief`.
- [x] `complete_mission {rationale, claimed_cells}` action — agent claims completion; bridge logs in `mission_log` for operator audit; `state.mission_complete` flips true.
- [x] Generator accepts `--info` for the brief; new world `husky_maze_corners.wbt` with multi-corner brief.
- [x] Tools `read_mission_brief` + `complete_mission` registered (registry: 17 → 19).
- [x] System prompt: read brief first, plan from brief + map_available, call `complete_mission` when satisfied.
- [x] Discriminator demonstrated: `solve.py` on corners world reaches hardcoded `goal_reached` but `mission_complete` stays false because it can't read the brief. The agent reads the brief, paraphrases the four corners back, plans the tour.

## Phase 3B — vision-only navigation (done)

- [x] Verified Path A (URDF edit, OmniSim ignores `<sensor>` tags) and runtime injection (OmniSim doesn't re-register devices) are dead ends.
- [x] **Landed Path B**: separate `husky_eye` Robot with its own `husky_eye` controller. Owns a real OmniSim `Camera`, tracks the husky's pose every tick via `getFromDef('HUSKY')`, serves base64 PNG over HTTP `127.0.0.1:6071`.
- [x] Bridge `/camera` proxies to the eye. `/capabilities.camera` reports the eye's status (kind, ready, dimensions, fov).
- [x] `husky_maze_visual.wbt` (seed 100) ships three coloured cylinders + the brief.
- [x] `chat_drive.py` attaches camera frames as inline OpenAI-style `image_url` parts so the engine sees the pixels.
- [x] **End-to-end verified**: agent calls `read_camera`, the engine decodes the inline image, and the agent narrates back the contents of the camera frame ("a maze corridor stretching out... checkered floor of light brown and dark brown squares...").

See `docs/RESULTS.md` Phase 3B for the verified architecture and live trace, and `docs/why-an-agent.md` §6 for the agent-only-thesis-at-maximum-strength argument.

## Phase 4 — broader generality + cross-agent composability

The full menu of next-step options (with effort estimates, wow factor, risks, decision matrices by audience and time-box) lives in [`docs/DIRECTIONS.md`](docs/DIRECTIONS.md). Operator picks; we execute.

Short list:

- [ ] **A** — Cross-session memory verification (~half day). Run visual mission twice; second run hits `recall` and skips vision.
- [x] **B** — Cross-agent composability. `Mission Captain` agent at [`agents/production/mission_captain/`](../mission_captain/), local orchestration via `delegate_to_agent`. Verified live: captain decomposed a multi-leg mission, delegated to `Husky Maze`, sub-agent drove the husky and reached `mission_complete=true`. See `docs/RESULTS.md` Phase 4.
- [ ] **C** — Hardcore vision-only world (~2 hours). Strip both `try_get_known_map` and `read_lidar`.
- [ ] **D** — Operator dialogue mid-mission (~1 day). `chat_drive.py` becomes an interactive REPL.
- [ ] **E** — Voice interface (~1 day). OmniLink STT/TTS push-to-talk.
- [ ] **F** — Web operator dashboard (~1 day). Static HTML+JS over `/status` + `/activity`.
- [ ] **G** — Sim-to-real bridge contract (~1–2 days). Extract `BridgeProtocol`, stub ROS2 backend.

Plus the leftover items from earlier phases:

- [ ] Standing-order shake-down: confirm `telemetry_tick` updates `husky_state` memory and `fault_watchdog` actually fires `stop_husky` under simulated stale telemetry.
- [ ] OpenAI-style native function-calling round-trips: today the chat loop embeds tool results as text because OmniLink's `chat()` rejects tool/tool_call message shapes. Either fix server-side, or add an `agent_chat` shim with engine-specific conversion.

## Phase 4 — broader generality

- [ ] Bridge config externalised so any 11×N or M×N grid maze works without code changes (read maze constants from a sidecar JSON or a `customData` payload on the husky)
- [ ] Add a real OmniSim `Lidar` device to the husky URDF as an alternative to the supervisor-raycast cheat — gives a more honest sensor surface
- [ ] Bigger maze worlds (21×21, 31×31) to stress-test BFS plan length and lidar update frequency
- [ ] World-swap orchestration tools: `list_worlds`, `load_world` exposed to the agent so it can drive its own evaluation harness
- [ ] Reuse the bridge for `warehouse_husky.wbt` (no maze, but same motion primitives + a different world title that hides the map)

## Open design questions

- Should the bridge expose continuous-space waypoint following (drive to (x,y)) so the agent can navigate non-grid worlds with the same tool set?
- Should the lidar be a separate OmniSim `Lidar` device (real sensor) or stay as a supervisor-raycast cheat (simpler, no URDF edits)?
- For multi-husky scenarios, should each husky run its own bridge instance on a different port, or should one bridge multiplex?
