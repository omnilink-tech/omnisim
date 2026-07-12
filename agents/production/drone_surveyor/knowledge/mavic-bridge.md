# mavic_omnilink_bridge — HTTP contract

The bridge is an OmniSim controller running inside the Mavic 2 Pro's process (Mavic2Pro proto with `supervisor=TRUE controller="mavic_omnilink_bridge"`). It owns motion (PID stabiliser adapted from stock `mavic2pro.py`), gimbal pitch control, camera capture, and a pure-Python BGRA classifier that returns world-projected marker detections.

The bridge listens on `127.0.0.1:6090` (loopback only). Override per-instance via `controllerArgs ["--port" "6091"]` if running multiple drones.

Source: [`projects/samples/demos/controllers/mavic_omnilink_bridge/mavic_omnilink_bridge.py`](../../../../projects/samples/demos/controllers/mavic_omnilink_bridge/mavic_omnilink_bridge.py).

## GET /state

Live drone telemetry. Updated every OmniSim tick (~8 ms).

```json
{
  "x": 0.0, "y": -12.0, "z": 0.07,
  "roll": 0.0, "pitch": -0.07, "yaw": 1.57,
  "v_xy": 0.0, "v_z": 0.0,
  "target_altitude_m": 0.0,
  "gimbal_pitch_rad": 1.5707963267948966,
  "gimbal_pitch_target_rad": 1.5707963267948966,
  "mode": "idle",          // idle | takeoff | hover | goto | land | landed
  "fault": null,
  "sim_time": 22.2,
  "last_tick_at": 1777743089.4,
  "target": null,          // {x, y, altitude} when mode=goto/hover
  "mission_complete": false
}
```

`mode` transitions: `idle -> takeoff -> hover` (after takeoff arrives at altitude); `hover -> goto -> hover` (each goto_waypoint); `hover -> land -> landed`.

`fault` is non-null when the controller can't reach a setpoint within its deadline. Common causes: `goto_waypoint_timeout` if the drone gets caught at the edge of the controllable envelope, or `land_timeout` if vertical descent stalls.

## GET /capabilities

```json
{
  "robot_id": "mavic2pro",
  "model": "DJI Mavic 2 Pro",
  "mass_kg": 0.9,
  "max_horizontal_speed_m_s": 18,
  "max_vertical_speed_m_s": 5,
  "default_takeoff_altitude_m": 12.0,
  "camera": {
    "width": 400,
    "height": 240,
    "fov_h_rad": 0.785,
    "fov_v_rad": 0.487,
    "pitch_range_rad": [-0.5, 1.7],
    "down_pitch_rad": 1.5708
  },
  "world_title": "Drone Surveyor",
  "mission_brief": "Mission: ...",
  "mission_complete": false,
  "ground_truth_def_names": ["WAREHOUSE", "MARKER_RED_1", ...],
  "perception_hint": "Prefer /scan over /image — ..."
}
```

`ground_truth_def_names` is the list of DEF'd Solids in the world the bridge will accept on `GET /solid?def=NAME`. Use these for sanity-checking — never as the survey's source of truth.

## GET /scan

The PREFERRED perception path. Bridge captures the latest camera frame, runs a pure-Python BGRA classifier (six recognised colours: red, green, blue, yellow, magenta, cyan), and projects each detected blob's image-space centroid to world (x, y) coordinates using the drone pose + gimbal pitch + camera FoV.

```json
{
  "markers": [
    {
      "color": "red",
      "pixels": 1052,
      "fraction": 0.011,
      "centroid_x_norm": 0.49,    // 0..1 (left to right in image)
      "centroid_y_norm": 0.61,    // 0..1 (top to bottom in image)
      "world_x": -2.97,
      "world_y": -7.99,
      "distance_m": 12.68,        // slant range from drone to ground point
      "projection_valid": true
    }
  ],
  "frame_summary": {
    "width": 400, "height": 240,
    "mean_brightness": 49.9,
    "frame_age_s": 0.08
  },
  "pose": {"x": -3.59, "y": -7.72, "z": 12.00, "yaw": 2.88, "gimbal_pitch_rad": 1.5708},
  "hint": "world_x/world_y are the projected ground positions ..."
}
```

Threshold: blobs covering <0.05 % of the frame are dropped (noise floor). At 12 m altitude with the Mavic's default FoV, an 0.8 m × 0.8 m ground marker subtends ~0.6 % of the frame — comfortably above the threshold.

Projection accuracy: <0.15 m at 12 m altitude when the gimbal is at pi/2 (verified in `docs/RESULTS.md`). Degrades proportional to `1 / sin(gimbal_pitch_rad)` — hold the gimbal close to straight down for ground surveys.

## GET /image (FALLBACK ONLY)

Returns the latest camera frame as base64 PNG plus pose. ~30 KB base64 → ~70 K input tokens at the model. Use only when /scan returns 0 detections at a vantage where you expect markers, or when the operator explicitly asks "what does the camera see?".

## GET /solid?def=NAME

Ground-truth pose lookup for any DEF'd Solid. Names are case-sensitive and must appear in `capabilities.ground_truth_def_names`.

```json
{
  "def": "MARKER_RED_1",
  "world_position": [-10.0, 6.0, 0.025],
  "world_orientation_3x3_row_major": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
}
```

USE FOR: spot-checking a `/scan` detection's projected position. NEVER USE FOR: lifting the survey's red-marker list directly out of the world file (that defeats the demo).

## GET /mission

```json
{
  "world_title": "Drone Surveyor",
  "brief": "Mission: ...",
  "complete": false,
  "log": [],
  "hint": "Read 'brief' in natural language. Plan + execute. ..."
}
```

`log` is an append-only list of `complete_mission` entries — operators audit it for honesty.

## POST /action

Body: `{"action": "<verb>", ...params}`. All actions return `{"status": "ok", ...}` on success or `{"error": "...", ...}` on failure.

| action | params | semantics |
|---|---|---|
| `takeoff` | `{altitude: float, wait?: bool, timeout_s?: float}` | Climb to altitude. With wait=true, blocks until within 0.4 m. |
| `land` | `{wait?: bool, timeout_s?: float}` | Controlled descent; idle motors below 0.4 m. |
| `hover` | `{}` | Pin to current xy + altitude. |
| `goto_waypoint` | `{x: float, y: float, altitude?: float, wait?: bool, timeout_s?: float, yaw_to?: [lx, ly]}` | Fly to (x, y, altitude). yaw_to aims nose at (lx, ly) on arrival. With wait=true, blocks until within 0.6 m. |
| `set_gimbal_pitch` | `{pitch_rad: float}` | 0=forward, pi/2=down. Range per capabilities.camera.pitch_range_rad. |
| `set_yaw` | `{yaw_rad: float}` | Absolute world yaw. |
| `stop` | `{}` | Cut all attitude inputs; idle motors. Last resort. |
| `reset` | `{}` | Teleport to start pose; clear mission_complete + log. |
| `complete_mission` | `{rationale: str, payload?: object}` | Log mission claim. Bridge does not score. |
