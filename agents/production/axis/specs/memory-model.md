# Axis Memory Model

Axis's memory is narrower than a general assistant's: the agent reasons about physical systems, so memory is dominated by *robot state* and *motion history* rather than conversation. Entries are tagged by domain (`robot` / `task` / `safety`) so downstream tools can filter without parsing content.

## Memory tiers

### Working memory

Short-lived context for the current control tick or operator command:

- active robot id and capability record
- latest `q`, `qdot`, and TCP pose pulled by the telemetry tick
- current commanded setpoint and the source (`operator` / `standing_order` / `planner`)
- the active task's next waypoint, progress fraction, and retry budget
- any alert raised this tick that has not yet been acknowledged

### Session memory

Context that should survive between adjacent conversations but does not describe the robot forever:

- the trajectory currently executing, its goal, and its waypoint log
- faults raised and cleared during the session
- manual overrides the operator has issued in the last hour
- pending operator confirmations (e.g. a proposed concrete delta awaiting go-ahead)
- the running session summary under assembly for the 18:00 roll-up

### Long-term memory

Stable per-robot and per-deployment knowledge:

- robot capability records — `joint_names`, `joint_limits`, `home_pose`, TCP offset, IK constants
- known-good setups (calibration baselines, nominal payloads, preferred cycle times)
- recurring failure signatures — which joints regularly approach a limit, which targets historically cause singularities
- operator-preferred safety thresholds (stricter than the defaults, if the operator has tightened them)
- deployment-specific metadata — which simulation hosts which robot, which scenes are active

## What Axis should remember

- every commanded setpoint with timestamp, robot id, source, and realized result
- every fault raised, the ticks it persisted, and the resolution path
- operator-authored corrections to default thresholds (e.g. "cap `IK_MAX_DQ` at 0.05 for omniarm6_03")
- successful trajectory recipes that can be replayed (pick-and-place templates, home cycles)
- the mapping between physical robot ids, OmniSim bridge endpoints, and scene files

## What Axis should not remember blindly

- one-off joint-state snapshots with no associated decision
- every raw telemetry tick (the log is authoritative; memory stores deltas and anomalies)
- simulation-internal wall time or world-clock drift (derive from `get_simulation_time` when needed)
- secrets — OmniSim bridge API tokens, scope tokens, or anything backed by a credential store
- operator chat transcripts beyond the rationale attached to a specific commanded setpoint

## Memory write policy

Write memory only when information affects a future tick, a future command, or an operator decision. Every durable memory should answer at least one of:

- Will this change a future safety decision?
- Will forgetting this cause a failed motion to recur silently?
- Is this a stable robot or deployment fact (capability, calibration, topology)?
- Is this an operator-authored override of a default threshold?

## Memory card format

Each durable memory captures:

- timestamp or time range
- domain tag (`robot` / `task` / `safety` / `deployment`)
- robot id (where applicable)
- subject
- fact, decision, or observation
- why it matters (e.g. "commanded setpoint diverged from realized by 0.07 rad — retune or lower `IK_MAX_DQ`")
- confidence level
- source (telemetry read, operator statement, bridge response, standing-order rollup)
