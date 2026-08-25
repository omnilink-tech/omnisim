# Axis Architecture

## Overview

Axis is OmniLink's robot-control agent. It drives robots inside OmniSim — the OmniLink simulation environment built on Webots — without running inside the OmniSim process itself. Instead it sends commands through OmniLink's automation HTTP endpoints to a thin bridge that wraps an OmniSim controller script (see `projects/samples/demos/controllers/omnilink_arm_bridge/omnilink_arm_bridge.py`, launched with `controllerArgs ["--robot" "omniarm6" ...]`, for the reference kinematics and motor-command pattern).

## Execution Model: Standing Orders

Axis uses OmniLink's standing-orders system for its always-on safety loop, with operator-triggered commands on top for task work:

```
Standing Orders System
  |
  |-- Telemetry tick  (every 1s) -----> Axis: sense + update robot_state
  |-- Safety watchdog (every 2s) -----> Axis: validate + stop on violation
  |-- Session summary (18:00 daily) --> Axis: roll up logs
  |
  |    Operator / planner commands (ad-hoc) -----> Axis: plan + execute
  v
Each execution: Sense -> Validate -> Decide -> Act -> Remember
```

Why standing orders for a robot agent?
- The watchdog survives Axis process restarts — every tick is independent.
- Telemetry persistence lets operator commands assume fresh state without re-reading.
- The same pattern scales from one OmniArm 6 to a fleet of mixed robots.

## State Management

```
robot_state
  ├── robots/
  │   ├── omniarm6_01: {
  │   │     capabilities: ["arm_6dof", "ik_position_only"],
  │   │     joint_names: ["joint1", "joint2", ...],
  │   │     joint_limits: [[-6.28, 6.28], [-3.14, 3.14], ...],
  │   │     home_pose:   [-0.1732, 0.1855, 0.7417, 0.0, 2.2144, 0.0],
  │   │     last_q:      [...],
  │   │     last_tcp:    {xyz: [...], rpy: [...]},
  │   │     last_setpoint: {...},
  │   │     last_tick_at: 2026-04-14T12:00:00Z,
  │   │     fault:       null
  │   │   }
  │   └── ...
  ├── tasks/
  │   ├── active:   [{ id, robot_id, goal, waypoints, progress }]
  │   └── history:  [...]
  └── alerts/
      ├── active:   [{ type, severity, robot_id, since }]
      └── history:  [...]
```

## Device Integration Layer

```
Axis Agent
  |
  v
OmniLink Automation HTTP Endpoints
  |
  └── OmniSim Bridge (per simulation)
        |
        v
      OmniSim Controller (omnilink_arm_bridge.py --robot omniarm6)
        |-- controller.Robot()              (OmniSim Python API)
        |-- robot.getDevice("<joint>_motor")
        |-- motor.setPosition(q_i)          (joint command)
        |-- forward_kinematics / ik_step    (from omnilink_arm_bridge.py)
        `-- robot.step(time_step)
```

The bridge is the only component that imports `controller` from OmniSim. Axis never does. This matches Haven's Home-Assistant adapter pattern: the agent reasons, the adapter translates.

### Normalized command surface

| Command | Payload | Response |
|---|---|---|
| `list_robots` | – | `[{ id, model, capabilities }]` |
| `get_robot_state(id)` | – | `{ q, qdot, tcp, fault, last_tick_at }` |
| `read_joints(id)` | – | `{ q }` |
| `read_tcp_pose(id)` | – | `{ xyz, rpy }` |
| `set_joint_positions(id, q)` | `q: float[6]` | `{ accepted, clamped_q }` |
| `set_tcp_target(id, xyz, rpy?)` | position target | `{ accepted, solved_q, err_norm }` |
| `solve_ik(id, xyz, rpy?)` | target | `{ q, err_norm }` (no motion) |
| `stop_robot(id)` | – | `{ halted_at }` |
| `reset_to_home(id)` | – | `{ q: HOME_POSE }` |
| `open_gripper(id)` / `close_gripper(id)` | – | `{ state }` |
| `pause_simulation` / `resume_simulation` | – | `{ sim_time }` |

IK on the bridge side uses the same damped-least-squares routine as `omnilink_arm_bridge.py --robot omniarm6` (`IK_MAX_ITERS = 100`, `IK_TOL = 5e-3`, `IK_DAMPING = 0.06`, `IK_MAX_DQ = 0.08`). Axis assumes these constants unless a robot capability record overrides them.

## Safety Model

- Every `set_joint_positions` and `set_tcp_target` is clamped on the bridge to `JOINT_LIMITS` before execution. Axis validates too, but the bridge is the last line of defense.
- `stop_robot` is a dedicated endpoint, not a flag on another command — it must always succeed, even under load.
- The safety watchdog standing order runs at 2× the telemetry tick rate. If `now - last_tick_at > 2 * tick_period`, Axis issues `stop_robot` and raises a `telemetry_stale` alert.
- Operator sessions requiring simulation topology changes (spawning objects, moving goal markers) are out of scope for Axis — route those to a scene-control agent.

## Notification Strategy

| Severity | Channel | Trigger |
|---|---|---|
| Critical | Push + operator console | Joint-limit breach, self-collision, emergency stop invoked |
| High | Push | Telemetry stale, IK singularity, unreachable target |
| Medium | In-app summary | Drift > 0.05 rad per joint between setpoint and realized |
| Low | Daily session summary | Routine motion logs, per-robot motion time |

## Security Model

- Robot commands require an authenticated OmniLink session with a `robot.control` scope.
- `stop_robot` is always allowed, regardless of scope.
- `pause_simulation` / `resume_simulation` require `sim.admin` scope.
- Telemetry reads require `robot.read`.
- The OmniSim bridge binds only to the loopback interface by default; exposing it beyond the host requires explicit operator opt-in in the bridge config.
