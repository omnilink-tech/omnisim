# Robot reference

Capability records for robots Axis knows how to drive. Each entry is the set of facts Axis assumes when operator commands reference the robot by id. Bridge-side capability records are authoritative; entries here are the documented defaults.

## OmniArm 6 (reference robot)

The OmniArm 6 in OmniSim is the reference deployment. Robot id: `omniarm6`. Driven by the generic controller `projects/samples/demos/controllers/omnilink_arm_bridge/omnilink_arm_bridge.py`, launched with `controllerArgs ["--robot" "omniarm6" ...]`. Default target world: `projects/samples/demos/worlds/chat/omnilink_omniarm6.omniworld`.

- **Kinematics**: 6 revolute joints
- **Joint names**: `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`
- **Home pose** (rad): `[-0.1732, 0.1855, 0.7417, 0.0, 2.2144, 0.0]`
- **Joint limits** (rad, per joint):
  - `joint1`: `[-6.28, 6.28]`
  - `joint2`: `[-3.14, 3.14]`
  - `joint3`: `[-3.14, 3.14]`
  - `joint4`: `[-6.28, 6.28]`
  - `joint5`: `[-3.14, 3.14]`
  - `joint6`: `[-6.28, 6.28]`
- **IK**: damped least squares
  - `IK_MAX_ITERS = 100`
  - `IK_TOL = 5e-3`
  - `IK_DAMPING = 0.06`
  - `IK_MAX_DQ = 0.08` (rad per control tick)
- **Control tick**: 1 s telemetry, 2 s watchdog

## Adding a new robot

When a new robot is wired into OmniSim:

1. The bridge registers its capability record under `robot_state.robots[id].capabilities`.
2. Axis pulls the record on first reference; if it's missing, `set_joint_positions` / `set_tcp_target` refuse with `no_capabilities`.
3. Add a section to this file documenting joint names, limits, home pose, IK constants, and anything non-obvious.
4. If the robot's behavior diverges from the OmniArm 6 pattern (non-6DOF, mobile base, orientation-sensitive IK), note it explicitly.

## What is NOT in scope for Axis

- Scene props, goal markers, spawned objects — those are a scene-control agent's responsibility.
- URDF / meshes — linked from the robot's OmniSim package, not indexed here.
- Recorded trajectories — stored under `agents/axis/long_term_memory/` when they're reusable recipes; ephemeral paths stay in session memory only.
