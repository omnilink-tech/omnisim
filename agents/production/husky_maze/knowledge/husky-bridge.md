# husky_omnilink_bridge — contract

The Husky Maze agent talks to the OmniSim-side controller `husky_omnilink_bridge` over loopback HTTP. Source: `projects/samples/demos/controllers/husky_omnilink_bridge/husky_omnilink_bridge.py`.

## Wiring

`husky_maze.wbt` (and `husky_maze_unknown.wbt`) has:

```
URDFRobot {
  url "../../../../projects/robots/clearpath/husky_description/urdf/husky.urdf"
  translation -10 10 0.1
  name "husky"
  supervisor TRUE
  controller "husky_omnilink_bridge"
}
```

`supervisor TRUE` lets the bridge read pose with `getSelf()`, lidar-raycast against `Wall` AABBs, and teleport the husky on `reset` / `snap_to_cell`.

The URDFRobot wrapper itself has no physics body, so its `getPosition()` returns NaN; the bridge walks the URDF subtree at startup and locks onto `base_link` for pose reads.

## HTTP surface

Bind: `127.0.0.1:6070` (loopback only).

| Method | Path                | Body | Returns |
|--------|---------------------|------|---------|
| GET    | `/state`            | —    | `{x, y, yaw, v_linear, v_angular, left_speed, right_speed, mode, fault, sim_time, last_tick_at, target, goal_reached}` |
| GET    | `/capabilities`     | —    | `{robot_id, model, wheel_motors, max_wheel_speed_radps, wheel_radius_m, half_track_m, max_linear_m_s, max_angular_r_s, tick_period_s, maze, world_title, map_available, lidar}` |
| GET    | `/lidar`            | —    | `{angles_rad: float[16], ranges_m: float[16], max_range_m, pose, hits?}` (`hits` only when `map_available`) |
| GET    | `/maze`             | —    | gated by `world_title`. If the title contains "Unknown", returns `{available: false, world_title, hint}`. Otherwise `{available: true, cells, cell_size_m, origin_x, origin_y, start, goal, adjacency: {"col,row": [[c,r], ...]}}` |
| POST   | `/action`           | `{action: ...}` | dispatches actions below |
| POST   | `/admin/reload`     | `{world?: "abs/path.wbt"}` | reloads current world or switches to a new one. **Kills this controller process**, so the response may not reach the caller — treat connection-reset as success |

### `/action` types

| action            | params                             | effect |
|-------------------|------------------------------------|--------|
| `stop`            | —                                  | zero both wheels, mode → `stopped`. Always available. |
| `reset`           | —                                  | teleport husky to `(MAZE.start.x, MAZE.start.y, yaw=0)` |
| `snap_to_cell`    | `{col, row, yaw}`                  | teleport husky to cell centre at the given cardinal yaw. **Demo concession**: skid-steer pivots in OmniSim accumulate ~0.5 m of drift per 90° turn, which compounds across cells. The agent (and `solve.py`) call this after each successful step to re-anchor to the grid. Real-world nav would use a better controller. |
| `set_velocity`    | `{linear, angular}`                | open-loop body twist; held until next action |
| `drive_forward`   | `{distance, speed?}`               | closed-loop drive along current heading; **signed-progress** controller — overshoot triggers reverse to settle on the commanded distance |
| `turn`            | `{angle, speed?}`                  | closed-loop rotation, settle gate |
| `goto_cell`       | `{col, row, speed?}`               | two-phase pure-pursuit; less reliable than `turn` + `drive_forward` for tight maze corners |

## Lidar contract

- 16 rays, evenly spaced around the husky's body frame.
- `angles_rad[i] = (2*pi*i / 16) - pi`. So:
  - i=0: behind (`-pi`)
  - i=4: right (`-pi/2`)
  - i=8: forward (`0`)
  - i=12: left (`+pi/2`)
- Max range: 8 m.
- Implementation: bridge walks the world's `Wall` nodes at startup, caches `(cx, cy, sx, sy)` AABBs, then ray-casts per request.
- A neighbour cell (2 m centre-to-centre) is **reachable** when the cardinal-direction ray returns a range greater than `cell_size_m + ~0.4 m clearance ≈ 2.4 m`.
- The agent never sees the wall list — only the ranges. That's what makes "the agent doesn't know the map" honest in the unknown-world demo.

## Mode lifecycle

`mode` values:

- `idle` — no task in flight
- `velocity` — `set_velocity` is held until cleared
- `drive_forward` / `turn` / `goto_cell` — closed-loop motion in flight
- `stopped` — explicitly halted

The bridge transitions back to `idle` (and clears `target`) when a closed-loop motion completes. Each task uses a **settle gate**: it only marks done once both `dist < tolerance` AND `|v_linear| < SETTLE_LIN_M_S` AND `|v_angular| < SETTLE_ANG_R_S`. This prevents momentum from one cell carrying into the next.

## Fault codes

- `drive_forward_timeout` — motion didn't reach the commanded distance within `GOTO_TIMEOUT_S` (30 s)
- `turn_timeout` — same for turn
- `goto_cell_timeout` — same for goto_cell

The agent should `stop_husky` on any non-null fault and replan.

## World metadata

`/capabilities.world_title` and `/capabilities.map_available` come from the loaded world's `WorldInfo.title`:

- `husky_maze.wbt` → title `"Husky Maze"` → `map_available: true`
- `husky_maze_unknown.wbt` → title `"Husky Maze (Unknown)"` → `map_available: false`

The check is `"unknown" in title.lower()`. To author a new world that the agent has never seen, include "Unknown" in the title.

## Self-managed reload

The bridge can reload its own world without the operator touching OmniSim:

```bash
# Reload the current world (e.g. after editing the bridge .py)
curl -X POST http://127.0.0.1:6070/admin/reload

# Switch to a different world entirely
curl -X POST http://127.0.0.1:6070/admin/reload \
     -H 'Content-Type: application/json' \
     -d '{"world": "projects/samples/demos/worlds/flagship/husky_maze_unknown.wbt"}'
```

After the call the controller process is killed and respawned by OmniSim. The bridge comes back up on `127.0.0.1:6070` within a few seconds.

## Frame conventions

- World: `+x` east, `+y` north, `+z` up.
- `yaw = atan2(orient[3], orient[0])`; positive = CCW around `+z`.
- Cardinal headings: `0 = east, +pi/2 = north, +/-pi = west, -pi/2 = south`.
- Wheel ordering: `(front_left, rear_left, front_right, rear_right)`.

## Maze constants

- `cell_size_m: 2.0`
- `cols: 11`, `rows: 11`
- `origin_x: -10.0`, `origin_y: -10.0` (world position of cell `(0, 0)`)
- `start: {col: 0, row: 10, x: -10.0, y: 10.0}`
- `goal: {col: 10, row: 0, x: 10.0, y: -10.0}`
- `goal_radius_m: 0.8`

`state.goal_reached` flips true when `hypot(x - 10, y + 10) <= 0.8 m`.
