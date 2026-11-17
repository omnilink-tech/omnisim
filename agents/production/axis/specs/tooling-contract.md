# Axis Tooling Contract

## Tooling principles

Axis uses tools to drive real (simulated) motion. Every tool call has a physical consequence — a motor rotates, a TCP moves, a fault is raised. The bar for *"should I call this tool right now?"* is higher than a text-only agent's.

- Prefer direct telemetry reads over inference when state can be checked.
- Use the fewest tools needed to advance the task — every command is a motion risk.
- Summarize outcomes in operator language (SI units, joint names), not raw bridge JSON.
- Track whether a tool result is authoritative (bridge-confirmed) or predictive (IK solution, not yet executed).
- For motion: observe first (`read_joints` / `read_tcp_pose`), command second, re-observe to confirm.

## Priority tool classes

### Robot-control tools

Bounded actuation on the OmniSim bridge:

- `list_robots` — enumerate known robots and their capability records
- `get_robot_state(id)` — full state snapshot: `q`, `qdot`, TCP, fault, `last_tick_at`
- `read_joints(id)` — joint angles only, freshest read
- `read_tcp_pose(id)` — TCP xyz + rpy, freshest read
- `set_joint_positions(id, q)` — joint-space setpoint; bridge clamps to limits
- `set_tcp_target(id, xyz, rpy?)` — task-space target; bridge solves IK and steps
- `solve_ik(id, xyz, rpy?)` — IK only, no motion (returns `q`, `err_norm`)
- `stop_robot(id)` — dedicated emergency stop; always available
- `reset_to_home(id)` — move to `home_pose` via interpolated trajectory
- `open_gripper(id)` / `close_gripper(id)` — end-effector actions when exposed
- `read_sensor(id, sensor)` — force/torque, camera, or custom sensor readouts
- `get_simulation_time` — sim-time clock; used to stamp every setpoint

### Simulation-control tools

Gated behind `sim.admin` scope:

- `pause_simulation` / `resume_simulation` — pause or resume the OmniSim world clock

### Trajectory tools

- `plan_trajectory(id, waypoints, constraints?)` — generate a time-parameterized path
- `execute_trajectory(id, traj_id)` — stream the planned trajectory to the bridge

### Memory tools

- `save_local_memory(title, body, tags?)` — write a durable per-robot or per-deployment note
- `search_local_memory(query, k?, tags?, mode?)` — hybrid vector + BM25 retrieval
- `list_local_memories(tags?, limit?)` — browse metadata
- `forget_local_memory(id)` — delete a specific memory

### Knowledge tools

- `search_knowledge(query)` — scan the curated knowledge folder (robot docs, bridge schemas, OmniLink architecture)
- `recall(query)` — unified retrieval across knowledge + long-term memory

### Registry tools

- `find_tools(query)` — locate a tool by topic when the manifest is large
- `list_tools` — full registry enumeration
- `invoke_tool(name, args)` — meta-dispatcher for on-demand tools

## Command mapping

Initial Axis command vocabulary (UI buttons, not AI-callable):

### Motion
- `list_robots`
- `get_robot_state`
- `read_joints`
- `read_tcp_pose`
- `set_joint_positions`
- `set_tcp_target`
- `solve_ik`
- `plan_trajectory`
- `execute_trajectory`
- `stop_robot`
- `reset_to_home`

### End-effector
- `open_gripper`
- `close_gripper`

### Simulation
- `get_simulation_time`
- `pause_simulation`
- `resume_simulation`

### Perception
- `read_sensor`

## Action tiers

Axis has no tiered approval at the tool layer — every call executes immediately. Instead:

### Always safe (no gate)

- all telemetry reads (`read_joints`, `read_tcp_pose`, `get_robot_state`, `read_sensor`)
- `solve_ik` — IK computation with no motion side-effect
- `list_robots`, `get_simulation_time`, knowledge + memory searches
- `stop_robot` — always allowed, by design

### Guarded (runs immediately; subject to bridge-side limits)

- `set_joint_positions`, `set_tcp_target` — the bridge clamps to joint limits and enforces `IK_MAX_DQ`; Axis validates first but the bridge is authoritative
- `execute_trajectory` — streams commanded setpoints through the same bridge-side clamp
- `reset_to_home` — bridge plans a bounded path to `home_pose`
- `open_gripper` / `close_gripper` — effector state changes
- `pause_simulation` / `resume_simulation` — requires `sim.admin`

### Prohibited without explicit override

- disabling the safety watchdog standing order
- bypassing `stop_robot` (no mechanism to do so is exposed)
- issuing raw motor commands that skip the bridge-side clamp
- changing simulation topology (spawning objects, moving goal markers) — out of scope; routes to a scene-control agent

## Motion safety loop

On every motion command:

1. **Sense** — read the robot's current `q` and TCP pose. If the last tick is stale (`> 2 * tick_period`), refuse and raise `telemetry_stale`.
2. **Validate** — check the target against `joint_limits`, workspace bounds, and the active task's preconditions. For TCP targets, call `solve_ik` first and refuse on non-convergence.
3. **Step-bound** — verify no joint step exceeds `IK_MAX_DQ`. If a larger move is required, break it into a trajectory.
4. **Command** — send `set_joint_positions` / `set_tcp_target` through the bridge.
5. **Confirm** — on the next telemetry tick, verify the realized pose is within tolerance of the commanded pose. If not, decide: retry, stop, or escalate.
6. **Log** — record the commanded setpoint, source, rationale, and realized result into `robot_state.history`.

If telemetry drifts between step 1 and step 4 (stale read, new fault flag), abort and restart from step 1.

## Logging contract

Every motion command records:

- `timestamp` (UTC, millisecond precision)
- `robot_id`
- `source` (`operator` / `standing_order` / `planner`)
- `command` (`set_joint_positions`, `set_tcp_target`, etc.)
- `target` (the numeric payload)
- `realized` (from the next telemetry tick, or `pending` until observed)
- `rationale` (one short line)
- `outcome` (`succeeded` / `partial` / `failed` / `aborted`)

The log is the authoritative trace for incident review; working memory keeps only the recent tail.
