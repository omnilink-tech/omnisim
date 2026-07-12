# Drone Surveyor — build plan

Iteration-by-iteration scope for the Drone Surveyor demo. Mirrors the structure of `warehouse_foreman/README.md`'s build trajectory and `warehouse_patrol/docs/PLAN.md`'s iteration cadence.

Read [`agents/AGENT_PATTERNS.md`](../../../AGENT_PATTERNS.md) before starting any iteration — it's the cross-demo cheat sheet for "patterns that paid off, defaults to use, mistakes to skip."

## Iteration 0 — world + bridge + LLM-free solve.py *(this commit)*

Goal: have a runnable Mavic + warehouse + coloured-marker world, a working motion + perception bridge, and a deterministic solver that proves the perimeter survey + projection + dedup pipeline works end-to-end.

- [x] `projects/samples/demos/worlds/chat/omnilink_mavic.wbt` — 60 × 60 m floor, central warehouse footprint (16 × 10 × 6 m), 8 coloured ground markers (3 RED, 5 distractors), Mavic2Pro at south start position with `supervisor=TRUE controller="mavic_omnilink_bridge"`.
- [x] `projects/samples/demos/controllers/mavic_omnilink_bridge/` — single-process bridge owning motion, gimbal, camera capture, perception classifier, world-coordinate projection. HTTP surface: `/state`, `/capabilities`, `/mission`, `/scan`, `/image`, `/solid`, `POST /action {takeoff|land|hover|goto_waypoint|set_gimbal_pitch|set_yaw|stop|reset|complete_mission}`. PID values copy-pasted from the stock `mavic2pro.py` for stability.
- [x] `agents/production/drone_surveyor/` skeleton — README, docs/PLAN.md, profile.json placeholder, prompts/system.md placeholder, knowledge/ + tools/ + scripts/ skeleton dirs.
- [x] `agents/production/drone_surveyor/solve.py` — LLM-free perimeter survey: takeoff → 4-corner waypoints → scan_for_markers at each → dedup within 1.5 m → land → print red count + world positions.
- [x] Update `agents/ROADMAP.md` to mark iter-0 shipped + reorder build trajectory.
- [x] Update `agents/README.md` Agents section to list drone_surveyor.

## Iteration 1 — RESULTS.md from solve.py runs *(verified end-to-end 2026-05-02)*

Goal: prove the bridge + perception pipeline are tight enough that a deterministic strategy reliably finds all 3 RED markers, and document the cost shape (projection accuracy, classifier reliability).

- [x] First run with the original 4-corner perimeter at (±16, ±10) returned 0/3 markers — corner waypoints don't overlap any marker (camera footprint at 12 m alt is ~10 m × 6 m, so corners look at empty ground). Diagnosis: probe with the drone parked over MARKER_RED_3 confirmed the bridge + projection work (12 cm error from a single overhead vantage).
- [x] Replaced the 4-corner perimeter in `solve.py` with a 6-waypoint survey grid (3 columns × 2 rows) positioned so each red marker falls inside ≥1 camera footprint.
- [x] Re-run with the new grid — **3/3 red markers detected, mean error 0.06 m vs. ground truth, 0 false positives.** Yellow and magenta distractors correctly classified and filtered. Full table in [`RESULTS.md`](RESULTS.md).
- [x] No tuning needed — projection accurate to <0.15 m at 12 m altitude, classifier hits all reds at the first try, no distractor false positives. The `_classify_blobs` thresholds copied from `husky_eye._analyze_bgra` are tight enough.
- [x] [`RESULTS.md`](RESULTS.md) written with the verified run table.
- [ ] Tighten `/capabilities.perception_hint` based on what the LLM agent will need (carries over to iter 2 once we see the agent reading the hint live).

## Iteration 2 — productized agent *(shipped 2026-05-02)*

Goal: replace `solve.py`'s hard-coded waypoint loop with an OmniLink agent that plans the survey from `capabilities.mission_brief`, calls the bridge tool surface, and reports via `complete_mission`.

- [x] `agents/production/drone_surveyor/profile.json` — `Drone Surveyor` profile with full `mainTask` (workflow steps, hint-first scanning rule, strict honesty contract from `AGENT_PATTERNS.md`#3, fault-recovery branch). 3,048-char prompt.
- [x] `agents/production/drone_surveyor/prompts/system.md` — long-form rationale (why an agent, why each workflow step, the strict-honesty contract restated).
- [x] `agents/production/drone_surveyor/tools/`:
  - [x] `_base.py` — `BRIDGE_URL` (defaults to MAVIC_BRIDGE_URL env var → http://127.0.0.1:6090), `bridge_get`/`bridge_post`, `ToolSpec` with lean/full description modes.
  - [x] `__init__.py` — auto-discover loader (verbatim from husky_maze).
  - [x] `drone.py` — 15 drone-specific tools: `get_capabilities`, `get_state`, `read_mission_brief`, `scan_for_markers`, `read_camera`, `check_marker_position`, `takeoff`, `land`, `hover`, `goto_waypoint`, `set_gimbal_pitch`, `set_yaw`, `stop_drone`, `reset_drone`, `complete_mission`.
  - [x] `recall.py`, `local_memory.py`, `knowledge.py` — verbatim copies from husky_maze; paths resolve via `__file__` so they work as-is in any agent folder.
- [x] `agents/production/drone_surveyor/knowledge/` — `mavic-bridge.md` (HTTP contract), `world-layout.md` (chat/omnilink_mavic.wbt geometry + verified waypoint pattern + failed alternatives), `flight-envelope.md` (PID constants, projection geometry, failure modes).
- [x] `agents/production/drone_surveyor/drone_surveyor_agent.py` — runner with /tool /activity /status endpoints, default port 51521, drone-specific `classify_tool_result` + `build_status_snapshot` + memory polling for stop_drone / reset_drone / land memory commands.
- [x] `agents/production/drone_surveyor/scripts/chat_drive.py` — CLI driver mirroring husky_maze's: pushes prompt through `client.chat()`, runs the local tool dispatch loop, compacts old tool-result messages, image-attachment for `read_camera` fallback, end-of-run usage summary.
- [x] Smoke-test: 21 tools registered (14 SAFE / 7 GUARDED), runner imports + dispatcher works against the live bridge (dry-run for guarded), `check_marker_position(MARKER_RED_1)` returns ground truth `(-10, 6, 0.025)`.

## Pre-flight checklist re-evaluated after iter 2

- [x] **Ground-truth verification endpoint** — `/solid?def=MARKER_RED_*` shipped iter 0; `check_marker_position` tool wraps it for the agent.
- [x] **Perception sidecar** — `/scan` shipped iter 0; `scan_for_markers` is the agent's preferred sensor; `read_camera` is the explicit fallback (system prompt specifies when to escalate).
- [x] **Hint-first scanning** — `mainTask` step 4 references the verified waypoint pattern + scaling rule; agent reasons from `mission_brief` rather than dumping a fixed grid.
- [x] **Strict honesty contract in mainTask** — written into `profile.json` + `prompts/system.md`. Forbids manufacturing the result from `check_marker_position`; requires re-scanning from a different vantage when scan returns 0.
- [ ] **Caching plumbed end-to-end** — runner inherits the husky_maze profile-push shape; Supabase shared cache benefits will accrue automatically once `g1-engine` issues per-call cache hits. Lands properly when iter 3's measured run shows hit-rate.
- [x] **Runner /status surfaces tokens/hour, cache hit %, and dollars/hour** — UsageMeter wired in iter 2 runner, same shape as husky_maze.

## Iteration 3 — end-to-end LLM run + RESULTS.md update

Goal: prove the agent finds the 3 RED markers from one operator sentence, no hard-coded waypoints, and capture the cost.

- [ ] `python -m drone_surveyor_agent` + open OmniLink → operator says *"Fly the perimeter, count the red markers, report their positions."*
- [ ] Agent should: takeoff, point gimbal down, plan 4 waypoints around the warehouse, scan at each, aggregate + dedup red detections, land, complete_mission with `payload={red_count: 3, red_positions: [...]}`.
- [ ] Compare run-cost (tokens, $, sim-time) against the husky_maze v3 + warehouse_foreman baselines. The drone is a richer perception problem than the maze — expect higher per-mission cost than v3 husky but in the same order of magnitude.
- [ ] Update `docs/RESULTS.md` with the agent run alongside the solve.py baseline. Headline metric: agent's detection accuracy vs. the deterministic solver's.

## Pre-flight checklist (from AGENT_PATTERNS.md)

- [x] **Ground-truth verification endpoint exists** — `/solid?def=MARKER_*` returns world position of every marker DEF in the world. Bridge advertises the DEF list in `/capabilities.ground_truth_def_names`.
- [x] **Perception sidecar (or equivalent)** — `/scan` returns structured `{color, world_x, world_y, fraction, centroid_norm, distance_m}` per detected blob. `/image` is the explicit fallback for ambiguous frames. Pure-Python BGRA classifier, no LLM-pixel-decoding cost.
- [ ] **Hint-first scanning** — the agent's `mainTask` should pass `mission_brief` mention of "RED markers" through to its workflow; on each scan it filters to `color == "red"` first. (Lands iter 2.)
- [ ] **Strict honesty contract in mainTask** — "Only call complete_mission with positions you observed via scan_for_markers AND that survived dedup. Never invent positions or extrapolate from `/capabilities.ground_truth_def_names` (that's a sanity-check tool for the operator, not the source of truth for the survey)." (Lands iter 2.)
- [ ] **Caching plumbed end-to-end** — inherit the Supabase shared cache from warehouse_foreman work via the runner. (Lands iter 2.)
- [ ] **Runner /status surfaces tokens/hour, cache hit %, and dollars/hour** — inherit from husky_maze runner. (Lands iter 2.)
