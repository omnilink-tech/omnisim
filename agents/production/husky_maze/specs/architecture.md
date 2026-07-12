# Husky Maze — architecture

## Integration shape

```
Operator / OmniLink
        |
        v
Husky Maze agent (this folder, husky_maze_agent.py)
        |
        v
Husky OmniLink bridge (HTTP, 127.0.0.1:6070)
        |
        v
OmniSim controller "husky_omnilink_bridge" (Supervisor)
        |
        v
URDFRobot "husky" in husky_maze.wbt
```

The agent never imports `controller`. The bridge owns:

- `Supervisor.getSelf()` for pose
- `getDevice("<wheel>_motor").setVelocity(...)` for motion
- `setSFVec3f` / `setSFRotation` on the URDF's translation/rotation fields for `reset`
- `step(time_step)` for per-tick physics

The agent owns:

- maze planning (BFS over the cell adjacency graph)
- next-cell selection
- success detection (via `state.goal_reached`)
- fault response

## Execution model

Each turn:

1. **Sense** — `get_state`. Pose, mode, fault, target, goal_reached.
2. **Validate** — current cell within tolerance of expected cell? task progressing?
3. **Decide** — next cell on the planned path; or stop on fault.
4. **Act** — `goto_cell {col, row}` (preferred); or `set_velocity` / `drive_forward` / `turn` for recovery.
5. **Remember** — write current cell + `goal_reached` to memory under key `husky_state`.

## State shape (memory)

The agent persists this dict under key `husky_state`:

```json
{
  "current_cell": {"col": int, "row": int},
  "next_cell": {"col": int, "row": int},
  "remaining_path": [{"col": int, "row": int}, ...],
  "goal_reached": bool,
  "last_fault": null | "drive_forward_timeout" | "off_corridor" | ...,
  "last_tick_at": float
}
```

Restored on every standing-order tick from `get_state` to stay in sync with reality.

## Standing orders

- **telemetry_tick** (1 s) — refresh `husky_state` from `/state`.
- **fault_watchdog** (2 s) — `stop_husky` on fault or stale telemetry.

## Tool tiers

- **safe** (read-only or always-allowed): `get_capabilities`, `get_maze_graph`, `get_state`, `stop_husky`
- **guarded** (motion side-effect): `goto_cell`, `drive_forward`, `turn`, `set_velocity`, `reset_husky`

`stop_husky` is intentionally classified safe. It must succeed even when other commands are being rejected, and it is idempotent.
