# Mavic 2 Pro flight envelope + projection notes

## Flight envelope (from bridge constants in [`mavic_omnilink_bridge.py`](../../../../projects/samples/demos/controllers/mavic_omnilink_bridge/mavic_omnilink_bridge.py))

| Parameter | Value | Source |
|---|---|---|
| Mass | 0.9 kg | Real Mavic 2 Pro spec |
| K_VERTICAL_THRUST | 68.5 | Stock `mavic2pro.py` |
| K_VERTICAL_OFFSET | 0.6 | Stock `mavic2pro.py` |
| K_VERTICAL_P | 3.0 | Stock `mavic2pro.py` |
| K_ROLL_P | 50.0 | Stock `mavic2pro.py` |
| K_PITCH_P | 30.0 | Stock `mavic2pro.py` |
| MAX_YAW_DISTURBANCE | 0.4 | Stock `mavic2pro.py` |
| MAX_PITCH_DISTURBANCE | -1.0 | Stock `mavic2pro.py` (negative = pitch nose down to fly forward) |
| WAYPOINT_REACH_TOL_M | 0.6 m | Bridge — goto_waypoint considers arrived within this radius |
| ALTITUDE_REACH_TOL_M | 0.4 m | Bridge — takeoff considers reached within this |
| GIMBAL_PITCH_MIN | -0.5 rad | Per Mavic gimbal spec |
| GIMBAL_PITCH_MAX | 1.7 rad | Per Mavic gimbal spec |
| GIMBAL_DOWN_RAD | pi/2 (~1.5708) | Bridge default on init |
| DEFAULT_TAKEOFF_ALTITUDE | 12 m | Bridge default; advertised as `capabilities.default_takeoff_altitude_m` |

DO NOT REBALANCE the K_* constants without verifying stable hover first — a 2 % change in K_VERTICAL_THRUST tips the system into oscillation.

## Practical limits (verified in `docs/RESULTS.md`)

- **Cruise speed at altitude:** ~1-2 m/s typical, ~5 m/s peak. Each goto_waypoint leg in the validated 6-waypoint survey takes 10-30 s wall-clock depending on distance.
- **Takeoff time to 12 m:** ~5-15 s wall-clock with `--mode=fast`.
- **Altitude hold accuracy:** within ~0.01 m of target during cruise (verified by `RESULTS.md`).
- **Goto-waypoint arrival accuracy:** within `WAYPOINT_REACH_TOL_M = 0.6 m` of target on every leg in the verified run.

## Camera + perception

- Camera lives in `Mavic2Pro.cameraSlot` (default 400 × 240 frame, 0.785 rad H FoV).
- Vertical FoV is computed by the bridge: `fov_v = 2 * atan(tan(fov_h/2) * h/w) ≈ 0.487 rad`.
- Footprint at altitude `h`:
  - width  ≈ `2 * h * tan(fov_h/2)` ≈ `0.83 * h`
  - height ≈ `2 * h * tan(fov_v/2)` ≈ `0.50 * h`
  - At 12 m: ~10 m × 6 m. At 6 m: ~5 m × 3 m. At 24 m: ~20 m × 12 m.
- Projection accuracy: best at the frame centre, worst at the corners. The verified 0.06 m mean error at 12 m altitude is across detections at varying centroids — corner detections may be 2-3× worse.

## Projection geometry

For a centroid at image-space (`x_norm`, `y_norm`) where (0,0) is top-left and (1,1) is bottom-right:

```
tan_h = (x_norm - 0.5) * 2 * tan(fov_h / 2)     # angular offset right of optical axis
tan_v = (y_norm - 0.5) * 2 * tan(fov_v / 2)     # angular offset down from optical axis

# With gimbal pointed straight down (pitch = pi/2):
body_dx = -tan_v * altitude    # +x_body = forward (image-up = forward)
body_dy = -tan_h * altitude    # +y_body = left   (image-right = body-right)

# Rotate body-frame into world-frame using yaw:
world_dx = body_dx * cos(yaw) - body_dy * sin(yaw)
world_dy = body_dx * sin(yaw) + body_dy * cos(yaw)
world_x = drone_x + world_dx
world_y = drone_y + world_dy
```

For gimbal pitch != pi/2, the projection skews along the optical axis. Bridge uses a small-angle approximation: deviations of ±0.1 rad from pi/2 introduce <1 m error at 12 m altitude. Larger deviations are flagged with `projection_valid: false` (gimbal_pitch < 0.05 rad — nearly horizontal — punts with NaN).

## Failure modes seen during iter-1 verification

- **0 detections at every perimeter waypoint** — root cause: corner waypoints at (±16, ±10) had camera footprints (~10 m × 6 m) that didn't overlap any marker. Fix: replanned to a 6-waypoint 3×2 grid that overflies every marker.
- **(no other failure modes observed)**

The bridge's PID is well-tuned for the Mavic2Pro proto — no goto_waypoint faults, no land faults, no takeoff issues across the validation runs.
