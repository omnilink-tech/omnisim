# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Composition helpers — rule-of-thirds, headroom, look-room.

Pure math. Given a subject's world position and a lens, compute camera
position offsets that produce well-composed frames without the caller
hand-tuning numbers. Camera primitives in `camera.py` call into these
for default framing; storyboards can override by passing explicit
positions.
"""

from __future__ import annotations

import math
from typing import Literal

Vec3 = tuple[float, float, float]

FrameSize = Literal["xcu", "cu", "mcu", "ms", "ws", "establish"]
"""Standard cinematography frame sizes:
- xcu: extreme close-up (just the detail — eyes, gripper)
- cu: close-up (head/end-effector)
- mcu: medium close-up (head + shoulders / wrist + payload)
- ms: medium shot (waist up / arm + base)
- ws: wide shot (full body + immediate context)
- establish: very wide (subject in environment)
"""

# Distance from camera to subject, in multiples of the subject's
# characteristic dimension, for each frame size + nominal lens (50mm).
# Wider lenses need to be closer to give the same frame; teles further.
FRAME_DIST_50MM: dict[FrameSize, float] = {
    "xcu": 1.0,
    "cu":  1.8,
    "mcu": 3.0,
    "ms":  5.0,
    "ws":  9.0,
    "establish": 18.0,
}


def subject_distance(
    char_dim_m: float, frame: FrameSize, lens_focal_mm: float,
) -> float:
    """How far back the camera should sit, in world meters, to achieve the
    requested frame size with this lens. Scales linearly with focal length
    relative to a 50mm reference (so 100mm doubles the distance, 25mm halves)."""
    base_dist = FRAME_DIST_50MM[frame] * char_dim_m
    return base_dist * (lens_focal_mm / 50.0)


def rule_of_thirds_offset(
    distance_m: float, side: Literal["left", "right", "center"] = "center",
    fov_rad: float = math.radians(40),
) -> float:
    """How far to dolly the camera laterally to put the subject on a
    rule-of-thirds line at the given distance and field of view.

    Returns a signed lateral offset in meters. side='center' returns 0.
    """
    if side == "center":
        return 0.0
    half_width = distance_m * math.tan(fov_rad / 2.0)
    # rule of thirds: subject at 1/3 of frame width from one edge means
    # camera is offset by 1/6 of frame width.
    offset = half_width / 3.0
    return offset if side == "right" else -offset


def headroom_target(
    subject_pos: Vec3, eye_height_offset_m: float,
    frame: FrameSize = "mcu",
) -> Vec3:
    """Where the camera should look for natural headroom. For tight frames
    the target sits at eye level; for wider frames it drops toward body center
    so subject's feet aren't cropped."""
    sx, sy, sz = subject_pos
    # eye_height_offset_m is the subject's "eye height" above its translation
    # origin. For mcu/cu we target there; for wider frames we drop toward body
    # center so the subject reads as a whole.
    factor = {
        "xcu": 1.0, "cu": 1.0, "mcu": 0.85, "ms": 0.6, "ws": 0.4, "establish": 0.3,
    }.get(frame, 0.7)
    return (sx, sy, sz + eye_height_offset_m * factor)


def camera_from_angle(
    subject_pos: Vec3, distance_m: float, azimuth_deg: float,
    elevation_deg: float,
) -> Vec3:
    """Place camera at (distance, azimuth, elevation) around the subject in
    spherical coordinates. azimuth=0 looks from subject's +X side; elevation
    is degrees above the horizon. Standard photographer's frame.

    Returns world XYZ for the camera. Pair with `headroom_target` for the
    `target` argument to `/capture/camera`.
    """
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)
    horiz = distance_m * math.cos(el)
    dx = horiz * math.cos(az)
    dy = horiz * math.sin(az)
    dz = distance_m * math.sin(el)
    sx, sy, sz = subject_pos
    return (sx + dx, sy + dy, sz + dz)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def lerp_vec(a: Vec3, b: Vec3, t: float) -> Vec3:
    return (lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t))
