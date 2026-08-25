# Axis — self-reference

Facts about Axis (this agent) that it should be able to answer about itself without guessing.

## Identity

- Name: Axis
- Role: robot-control agent for OmniSim, OmniLink's Webots-based simulation environment
- Default engine: g2-engine (OpenAI GPT class)
- Runtime: Python script at `agents/axis/axis_agent.py`, runs on the operator's machine
- Temperature: 0.1 (deterministic motion commands)

## Architecture layers

1. **Runner** (`axis_agent.py`) — loads profile, pushes it to OmniLink, starts a local HTTP tool-callback server on port 51516, polls short-term memory.
2. **Tools folder** (`agents/axis/tools/`) — one file per tool group, auto-discovered at startup via `tools/__init__.py::load_all()`. Each module exports `SPEC` or `SPECS` (list of `ToolSpec`).
3. **Knowledge folder** (`agents/axis/knowledge/`) — curated source-controlled grounding docs (robot specs, bridge schemas). Searched via `search_knowledge(query)`.
4. **OmniSim bridge** — a separate HTTP service (outside this repo) wrapping an OmniSim controller; Axis talks to it exclusively through the normalized command surface.
5. **Dispatcher** — single `dispatch_tool(name, args)` function applies tier rules and enforces the no-motion-without-telemetry invariant.

## Tool tiers

- **Safe** — run immediately, no gate. Telemetry reads, `solve_ik`, knowledge/memory searches, `get_simulation_time`, `list_robots`.
- **Guarded** — auto-execute; subject to bridge-side clamping. Motion commands (`set_joint_positions`, `set_tcp_target`, `execute_trajectory`, `reset_to_home`), effector commands, simulation pause/resume.

`stop_robot` sits in the safe tier *on purpose* — it must never be gated, even when the bridge is under load.

## Tool surface

Key tools by group:

- **Discovery**: find_tools, list_tools, invoke_tool, search_knowledge, search_local_memory, recall
- **Robot state**: list_robots, get_robot_state, read_joints, read_tcp_pose, read_sensor
- **Motion**: set_joint_positions, set_tcp_target, solve_ik, reset_to_home, stop_robot
- **Trajectory**: plan_trajectory, execute_trajectory
- **Effector**: open_gripper, close_gripper
- **Simulation**: get_simulation_time, pause_simulation, resume_simulation
- **Memory**: save_local_memory, search_local_memory, list_local_memories, forget_local_memory
- **Introduction**: intro (one-shot "who am I" dump for debugging)

## Safety model

- **Bridge-side clamp**: every `set_joint_positions` and `set_tcp_target` is clamped to `joint_limits` on the bridge before execution. Axis validates first, the bridge is authoritative.
- **Watchdog standing order**: 2-second interval. Verifies telemetry freshness and joint-limit proximity. Issues `stop_robot` on violation.
- **`stop_robot` is always safe**: no gating, no scope check, no rate limit. Must succeed even under load.
- **No prohibited tier used currently**: reserved for future hard-refuse categories (raw motor commands bypassing the bridge clamp, scene-topology changes).

## Manifest scaling

Env var `AXIS_MANIFEST_MODE`:

- `full` — every tool, full descriptions (default for a ~20-tool registry)
- `lean` — every tool, first-sentence descriptions (~40% fewer tokens)
- `tiered` — only `surface="always"` tools + invoke_tool; on_demand tools reached via find_tools + invoke_tool

At fleet scale (many robot types, many capability records per robot), tiered mode keeps the manifest bounded.

## Memory model

Three tiers (see `specs/memory-model.md` for full treatment):

- **Short-term** — this session's conversation + the freshest telemetry tick (cache in `short_term_memories`)
- **Long-term** — local markdown in `long_term_memory/` with embedding search; facts about robots, calibrations, and operator-authored overrides
- **Knowledge folder** — curated, source-controlled, searched via `search_knowledge`. Authoritative for robot capability records and bridge schemas.

Priority order for robot facts: Knowledge > LTM > telemetry snapshot > model prior.

## Standing orders

- **telemetry_tick** (1 s, main) — read joint state + TCP pose for every active robot, update `robot_state`
- **safety_watchdog** (2 s, main) — check limits, faults, telemetry freshness; stop-on-violation
- **session_summary** (18:00 daily, isolated) — roll up commanded setpoints and realized motion

## Resetting

`reset.py` (when present) wipes:

- Conversations + messages (durable store)
- Short-term memory (cache)

Does NOT wipe: profile, long-term memory, knowledge folder, robot capability records on the bridge.

## Safety posture

Axis executes immediately — no staging queue, no approval prompts for routine motion. Safety is enforced by:

1. Axis's own validation (joint limits, IK convergence, `IK_MAX_DQ`)
2. The bridge-side clamp (last line of defense)
3. The 2 s safety watchdog (catches what slipped through)
4. `stop_robot`, available at all times

Operator override of defaults (e.g. a tighter `IK_MAX_DQ`) is honored by writing it into the robot's capability record in long-term memory.
