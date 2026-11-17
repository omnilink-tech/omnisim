# OmniSim bridge

The OmniSim bridge is the HTTP service that sits between Axis and OmniSim. Axis never imports the `controller` module directly — the bridge owns every `Robot()`, `getDevice`, `setPosition`, and `robot.step` call.

## Integration shape

```
Axis Agent
  |
  v
OmniLink Automation HTTP Endpoints
  |
  └── OmniSim Bridge (per simulation host)
        |
        v
      OmniSim Controller (omnilink_arm_bridge.py --robot omniarm6)
        |-- controller.Robot()              (OmniSim Python API)
        |-- robot.getDevice("<joint>_motor")
        |-- motor.setPosition(q_i)          (joint command)
        |-- forward_kinematics / ik_step    (from omnilink_arm_bridge.py)
        `-- robot.step(time_step)
```

The bridge is the only component that imports `controller` from OmniSim. Axis reasons; the bridge translates.

## Normalized command surface

All endpoints are POST JSON unless otherwise noted.

| Endpoint | Payload | Response |
|---|---|---|
| `list_robots` | — | `[{ id, model, capabilities }]` |
| `get_robot_state` | `{ id }` | `{ q, qdot, tcp, fault, last_tick_at }` |
| `read_joints` | `{ id }` | `{ q }` |
| `read_tcp_pose` | `{ id }` | `{ xyz, rpy }` |
| `set_joint_positions` | `{ id, q: float[6] }` | `{ accepted, clamped_q }` |
| `set_tcp_target` | `{ id, xyz, rpy? }` | `{ accepted, solved_q, err_norm }` |
| `solve_ik` | `{ id, xyz, rpy? }` | `{ q, err_norm }` (no motion) |
| `stop_robot` | `{ id }` | `{ halted_at }` |
| `reset_to_home` | `{ id }` | `{ q: HOME_POSE }` |
| `open_gripper` / `close_gripper` | `{ id }` | `{ state }` |
| `read_sensor` | `{ id, sensor }` | `{ value, unit, timestamp }` |
| `pause_simulation` / `resume_simulation` | — | `{ sim_time }` |
| `get_simulation_time` | — | `{ sim_time }` |

## IK contract

The bridge uses damped least squares with the same defaults as `omnilink_arm_bridge.py --robot omniarm6` unless a robot capability record overrides them:

- `IK_MAX_ITERS = 100` — stop after this many iterations
- `IK_TOL = 5e-3` — position error threshold for convergence
- `IK_DAMPING = 0.06` — damping factor in the pseudo-inverse
- `IK_MAX_DQ = 0.08` — per-tick joint step cap

`solve_ik` is read-only; `set_tcp_target` runs IK and then commands the solved `q` in a single call.

## Safety contract

- Every `set_joint_positions` and `set_tcp_target` is clamped to `joint_limits` before the bridge hands it to the motor controller. The bridge is authoritative.
- `stop_robot` is a dedicated endpoint, never a flag on another command. It MUST succeed even under load and even when other commands are being rejected.
- If the bridge loses contact with its OmniSim child, it raises `fault = "controller_lost"` on the affected robots and starts returning 503 for motion endpoints. `stop_robot` still returns 200 — idempotent, since a disconnected robot is already "halted".
- The bridge binds to loopback by default. Remote access requires explicit operator opt-in in the bridge config.

## Scope / auth

- Robot commands require an authenticated OmniLink session with `robot.control` scope.
- Telemetry reads require `robot.read`.
- `pause_simulation` / `resume_simulation` require `sim.admin`.
- `stop_robot` is allowed with any authenticated session — no scope check.

## Fault codes (standardized)

- `controller_lost` — bridge cannot reach its OmniSim child
- `joint_limit` — commanded setpoint exceeded a joint limit (bridge clamped)
- `ik_nonconvergent` — IK did not converge within `IK_MAX_ITERS`
- `ik_singular` — Jacobian lost rank; target near a singularity
- `telemetry_stale` — most recent tick older than `2 * tick_period`
- `unreachable_target` — TCP target outside reachable workspace
- `effector_unavailable` — `open_gripper` / `close_gripper` called on a robot without a registered gripper

Axis converts these into human-readable alerts before surfacing to the operator.
