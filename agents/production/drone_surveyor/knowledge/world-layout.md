# chat/omnilink_mavic.wbt — world layout

Source: [`projects/samples/demos/worlds/chat/omnilink_mavic.wbt`](../../../../projects/samples/demos/worlds/chat/omnilink_mavic.wbt).

## Geometry

- **Floor:** 60 m × 60 m asphalt, centred at origin.
- **Background:** NightSky (dark starfield) + two DirectionalLight pairs.
- **Warehouse footprint** (DEF `WAREHOUSE`, no physics, no boundingObject — drone never touches it):
  - Centred at (0, 0, 3).
  - 16 m east-west × 10 m north-south × 6 m tall.
  - Distinctive dark roof stripe on top so operators can spot it in top-down views.
- **Mavic 2 Pro** (DEF `MAVIC`):
  - Start pose: (0, -12, 0.1), facing north (+y).
  - `supervisor=TRUE controller="mavic_omnilink_bridge"`.
  - Default Camera (400 × 240) in `cameraSlot` — bridge enables it on init and pitches the gimbal to pi/2 (straight down).

## Markers

Eight flat ground markers. Each is a 0.80 m × 0.80 m × 0.05 m emissive box positioned at z = 0.025. Emissive colour pushes pixel intensity well above the BGRA classifier's gates regardless of the directional-light angle.

| DEF | Colour | World (x, y) | Notes |
|---|---|---|---|
| `MARKER_RED_1` | red | (-10.0, +6.0) | Count target |
| `MARKER_RED_2` | red | (+12.0, -4.0) | Count target |
| `MARKER_RED_3` | red |  (-3.0, -8.0) | Count target |
| `MARKER_GREEN` | green | (+6.0, +7.0) | Distractor |
| `MARKER_BLUE` | blue | (-12.0, -4.0) | Distractor |
| `MARKER_YELLOW` | yellow | (+10.0, +5.0) | Distractor |
| `MARKER_MAGENTA` | magenta | (+5.0, -7.0) | Distractor |
| `MARKER_CYAN` | cyan | (-7.0, +2.0) | Distractor |

The mission brief mentions orange + white markers as "distractors that get classified as their nearest colour" — the world doesn't actually ship those, the brief is testing the agent's robustness to specifications it can't fulfill (the agent should report what it actually saw, not what the brief said it might see).

**Three RED markers** is the canonical count target. They are placed on three different sides of the warehouse so a single-pass perimeter survey actually has to look at all three regions — a one-corner shortcut won't catch them all.

## Survey waypoint patterns

Camera footprint at 12 m altitude with the Mavic's default FoV (0.785 rad H × 0.487 rad V):
- `2 * 12 * tan(0.785/2)` ≈ **9.94 m wide**
- `2 * 12 * tan(0.487/2)` ≈ **5.97 m tall**

To cover all 8 markers (spread across roughly ±13 m × ±8 m), use a 3×2 grid:

| Waypoint | Covers |
|---|---|
| (-10, +5) | MARKER_RED_1, MARKER_CYAN |
| (0, +6) | MARKER_GREEN |
| (+10, +5) | MARKER_YELLOW, MARKER_GREEN edge |
| (+10, -4) | MARKER_RED_2, MARKER_MAGENTA edge |
| (0, -7) | MARKER_RED_3, MARKER_MAGENTA |
| (-10, -4) | MARKER_BLUE |

This pattern is verified in [`../solve.py`](../solve.py) and [`../docs/RESULTS.md`](../docs/RESULTS.md): 3/3 RED detected, mean projection error 0.06 m at 12 m altitude.

A 4-corner perimeter at (±16, ±10) does NOT work — corner waypoints look at empty ground past the marker layout. The first solve.py iteration tried this and got 0/3.

## Failed alternatives (saved here so future agents don't repeat)

- **4-corner perimeter at (±16, ±10):** misses every marker. Camera footprint of ~10×6 m at 12 m doesn't reach the ±13 m × ±8 m marker region from corner vantages.
- **Single overhead at (0, 0):** sees only the warehouse roof. Markers are around the building, not on top of it.
- **Lower altitude (e.g. 6 m):** camera footprint shrinks to ~5×3 m — would need 12+ waypoints to cover the same area. Trade-off: tighter projection accuracy (<0.05 m) vs. more flight time. Not worth it for the demo's accuracy requirements.
