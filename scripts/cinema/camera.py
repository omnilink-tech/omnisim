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

"""Cinematic camera-move primitives.

Each primitive is a pure function: `(subject, params, duration_s) -> list of
keyframes`. Keyframes are `{"t": float, "position": Vec3, "target": Vec3}`
dicts that drop straight into a capture-service shot list.

This is the heart of the cinema package — the difference between
"set the camera at (-3,-3,2) and shoot" and "track OmniQuad from his left
flank as he walks". Camera placement is computed from the subject's
pose + a cinematography vocabulary, not from world coordinates.

When in doubt about which primitive to reach for, see the docstrings —
each names the kind of beat it serves (establish, build, reveal, etc.).
"""

from __future__ import annotations

import math
from typing import Literal

from . import composition as comp
from .subjects import Subject

Vec3 = tuple[float, float, float]
Keyframe = dict


def _kf(t: float, position: Vec3, target: Vec3) -> Keyframe:
    return {"t": float(t), "position": list(position), "target": list(target)}


def _front_of_subject(subject: Subject, distance_m: float, height_m: float) -> Vec3:
    """Place camera `distance_m` in front of where the subject is facing,
    elevated `height_m`. Uses subject heading from its rotation."""
    yaw = subject.heading_yaw_rad
    sx, sy, sz = subject.position
    fx = math.cos(yaw)
    fy = math.sin(yaw)
    return (sx + fx * distance_m, sy + fy * distance_m, sz + height_m)


def _side_of_subject(
    subject: Subject, side: Literal["left", "right"], distance_m: float, height_m: float,
) -> Vec3:
    """Place camera off subject's left or right flank at the given distance."""
    yaw = subject.heading_yaw_rad
    sign = -1.0 if side == "left" else 1.0
    side_yaw = yaw + sign * math.pi / 2
    sx, sy, sz = subject.position
    sx_o = sx + math.cos(side_yaw) * distance_m
    sy_o = sy + math.sin(side_yaw) * distance_m
    return (sx_o, sy_o, sz + height_m)


# ---------------------------------------------------------------------------
# Static-ish frames (establishing, beauty)
# ---------------------------------------------------------------------------

def establish(
    subject: Subject, *, lens_focal_mm: float = 24.0, duration_s: float = 4.0,
    azimuth_deg: float = 45.0, elevation_deg: float = 25.0,
    frame: comp.FrameSize = "establish",
) -> list[Keyframe]:
    """Wide static frame placing subject in environment. Use as opener.
    Camera holds — no movement — but with a slight slow drift so a freeze
    doesn't kill momentum (cinema beats with no movement feel like stills).
    """
    dist = comp.subject_distance(subject.profile.char_dim_m, frame, lens_focal_mm)
    start = comp.camera_from_angle(subject.position, dist, azimuth_deg, elevation_deg)
    # Drift 2% closer over the hold — imperceptible push-in keeps it alive.
    end = comp.camera_from_angle(
        subject.position, dist * 0.98, azimuth_deg, elevation_deg,
    )
    target = comp.headroom_target(subject.position, subject.profile.eye_height_m, frame)
    return [_kf(0.0, start, target), _kf(duration_s, end, target)]


def beauty(
    subject: Subject, *, lens_focal_mm: float = 85.0, duration_s: float = 4.0,
    azimuth_deg: float = -30.0, elevation_deg: float = 8.0,
) -> list[Keyframe]:
    """Tight head-and-shoulders hero frame, slight low angle. Telephoto
    flattens the subject's proportions — classic 'hero' compression."""
    dist = comp.subject_distance(subject.profile.char_dim_m, "mcu", lens_focal_mm)
    start = comp.camera_from_angle(subject.position, dist, azimuth_deg, elevation_deg)
    end = comp.camera_from_angle(subject.position, dist * 0.95, azimuth_deg + 5, elevation_deg)
    target = comp.headroom_target(subject.position, subject.profile.eye_height_m, "mcu")
    return [_kf(0.0, start, target), _kf(duration_s, end, target)]


# ---------------------------------------------------------------------------
# Camera moves
# ---------------------------------------------------------------------------

def orbit(
    subject: Subject, *, lens_focal_mm: float = 35.0, duration_s: float = 6.0,
    radius_chardim: float = 4.0, height_chardim: float = 1.2,
    sweep_deg: float = 90.0, start_azimuth_deg: float = -45.0,
    frame: comp.FrameSize = "ms",
) -> list[Keyframe]:
    """Arc the camera around the subject. Use to reveal scale and 3D form.
    radius_chardim and height_chardim are in multiples of the subject's
    characteristic dimension, so an orbit of OmniQuad vs an orbit of a Husky
    fleet stays correctly framed by default.
    """
    dist = max(
        radius_chardim * subject.profile.char_dim_m,
        comp.subject_distance(subject.profile.char_dim_m, frame, lens_focal_mm),
    )
    height = height_chardim * subject.profile.char_dim_m
    target = comp.headroom_target(subject.position, subject.profile.eye_height_m, frame)
    # Sample at ~30° steps so the Catmull-Rom on the renderer side stays smooth.
    n_samples = max(3, int(abs(sweep_deg) / 30.0) + 1)
    kfs: list[Keyframe] = []
    for i in range(n_samples):
        t_frac = i / (n_samples - 1)
        az = start_azimuth_deg + sweep_deg * t_frac
        el_deg = math.degrees(math.atan2(height, dist))
        pos = comp.camera_from_angle(subject.position, dist, az, el_deg)
        kfs.append(_kf(duration_s * t_frac, pos, target))
    return kfs


def push_in(
    subject: Subject, *, lens_focal_mm: float = 50.0, duration_s: float = 5.0,
    start_frame: comp.FrameSize = "ws", end_frame: comp.FrameSize = "mcu",
    azimuth_deg: float = -20.0, elevation_deg: float = 6.0,
) -> list[Keyframe]:
    """Dolly the camera toward the subject — wide to tight. Classic
    'pull viewer into the moment' move. Used for hero / build beats."""
    start_dist = comp.subject_distance(subject.profile.char_dim_m, start_frame, lens_focal_mm)
    end_dist = comp.subject_distance(subject.profile.char_dim_m, end_frame, lens_focal_mm)
    start_pos = comp.camera_from_angle(subject.position, start_dist, azimuth_deg, elevation_deg)
    end_pos = comp.camera_from_angle(subject.position, end_dist, azimuth_deg, elevation_deg)
    start_target = comp.headroom_target(
        subject.position, subject.profile.eye_height_m, start_frame,
    )
    end_target = comp.headroom_target(
        subject.position, subject.profile.eye_height_m, end_frame,
    )
    return [_kf(0.0, start_pos, start_target), _kf(duration_s, end_pos, end_target)]


def pull_out(
    subject: Subject, *, lens_focal_mm: float = 35.0, duration_s: float = 6.0,
    start_frame: comp.FrameSize = "mcu", end_frame: comp.FrameSize = "establish",
    azimuth_deg: float = -20.0, elevation_deg: float = 18.0,
) -> list[Keyframe]:
    """Reverse of push_in — start tight, retreat to wide reveal. Best as
    a 'resolve' beat. End elevation lifts as we pull back, so the final
    frame becomes a slight god's-eye establish."""
    start_dist = comp.subject_distance(subject.profile.char_dim_m, start_frame, lens_focal_mm)
    end_dist = comp.subject_distance(subject.profile.char_dim_m, end_frame, lens_focal_mm)
    start_pos = comp.camera_from_angle(subject.position, start_dist, azimuth_deg, 6.0)
    end_pos = comp.camera_from_angle(subject.position, end_dist, azimuth_deg, elevation_deg)
    start_target = comp.headroom_target(subject.position, subject.profile.eye_height_m, start_frame)
    end_target = comp.headroom_target(subject.position, subject.profile.eye_height_m, end_frame)
    return [_kf(0.0, start_pos, start_target), _kf(duration_s, end_pos, end_target)]


def tracking_side(
    subject: Subject, *, lens_focal_mm: float = 50.0, duration_s: float = 6.0,
    side: Literal["left", "right"] = "left",
    distance_chardim: float = 3.0, height_chardim: float = 0.6,
    travel_chardim: float = 4.0,
) -> list[Keyframe]:
    """Camera moves parallel to subject's heading, off one flank — the
    classic 'side dolly' that follows a walking robot. travel_chardim
    sets how far the camera moves over the duration; aim for the same
    speed as the subject so it stays in frame."""
    side_pos = _side_of_subject(
        subject, side,
        distance_chardim * subject.profile.char_dim_m,
        height_chardim * subject.profile.char_dim_m,
    )
    yaw = subject.heading_yaw_rad
    travel = travel_chardim * subject.profile.char_dim_m
    end_pos = (
        side_pos[0] + math.cos(yaw) * travel,
        side_pos[1] + math.sin(yaw) * travel,
        side_pos[2],
    )
    # Target moves with the subject by the same amount so framing stays locked.
    sx, sy, sz = subject.position
    start_target = (sx, sy, sz + subject.profile.eye_height_m)
    end_target = (
        sx + math.cos(yaw) * travel,
        sy + math.sin(yaw) * travel,
        sz + subject.profile.eye_height_m,
    )
    return [_kf(0.0, side_pos, start_target), _kf(duration_s, end_pos, end_target)]


def tracking_chase(
    subject: Subject, *, lens_focal_mm: float = 50.0, duration_s: float = 6.0,
    distance_chardim: float = 4.0, height_chardim: float = 1.0,
    travel_chardim: float = 4.0,
) -> list[Keyframe]:
    """Camera trails behind subject in its direction of travel. The 'over
    the shoulder' walk — feels purposeful, intimate."""
    yaw = subject.heading_yaw_rad
    sx, sy, sz = subject.position
    # Camera behind subject = opposite of heading direction.
    bx = sx - math.cos(yaw) * distance_chardim * subject.profile.char_dim_m
    by = sy - math.sin(yaw) * distance_chardim * subject.profile.char_dim_m
    start_pos = (bx, by, sz + height_chardim * subject.profile.char_dim_m)
    travel = travel_chardim * subject.profile.char_dim_m
    end_pos = (
        start_pos[0] + math.cos(yaw) * travel,
        start_pos[1] + math.sin(yaw) * travel,
        start_pos[2],
    )
    start_target = (sx, sy, sz + subject.profile.eye_height_m)
    end_target = (
        sx + math.cos(yaw) * travel,
        sy + math.sin(yaw) * travel,
        sz + subject.profile.eye_height_m,
    )
    return [_kf(0.0, start_pos, start_target), _kf(duration_s, end_pos, end_target)]


def crane_up(
    subject: Subject, *, lens_focal_mm: float = 28.0, duration_s: float = 6.0,
    start_height_chardim: float = 0.5, end_height_chardim: float = 8.0,
    horizontal_chardim: float = 4.0, azimuth_deg: float = -45.0,
) -> list[Keyframe]:
    """Vertical reveal — camera rises from near ground to high overhead.
    Subject stays centered in frame the whole way (target tracks the rise).
    Great as a closer."""
    sx, sy, sz = subject.position
    az = math.radians(azimuth_deg)
    horiz = horizontal_chardim * subject.profile.char_dim_m
    dx = math.cos(az) * horiz
    dy = math.sin(az) * horiz
    start = (sx + dx, sy + dy, sz + start_height_chardim * subject.profile.char_dim_m)
    end = (sx + dx, sy + dy, sz + end_height_chardim * subject.profile.char_dim_m)
    target = (sx, sy, sz + subject.profile.eye_height_m * 0.6)
    return [_kf(0.0, start, target), _kf(duration_s, end, target)]


def bird_eye(
    subject: Subject, *, lens_focal_mm: float = 24.0, duration_s: float = 5.0,
    altitude_chardim: float = 12.0, drift_chardim: float = 1.5,
) -> list[Keyframe]:
    """Top-down (or near-top-down) with a slow lateral drift. Reveals
    layout and scale. The drift is critical — a perfectly static top-down
    feels like a screenshot."""
    sx, sy, sz = subject.position
    alt = altitude_chardim * subject.profile.char_dim_m
    drift = drift_chardim * subject.profile.char_dim_m
    start = (sx - drift, sy - drift, sz + alt)
    end = (sx + drift, sy + drift, sz + alt)
    target = (sx, sy, sz)
    return [_kf(0.0, start, target), _kf(duration_s, end, target)]


def reveal_tilt(
    subject: Subject, *, lens_focal_mm: float = 35.0, duration_s: float = 4.0,
    distance_chardim: float = 3.5, azimuth_deg: float = 0.0,
) -> list[Keyframe]:
    """Camera holds position, target moves from ground up to subject's
    eyeline — a tilt-up reveal. Use when the surrounding environment is
    interesting (e.g. desert ruins, warehouse floor) before introducing
    the subject."""
    dist = max(
        distance_chardim * subject.profile.char_dim_m,
        comp.subject_distance(subject.profile.char_dim_m, "ms", lens_focal_mm),
    )
    pos = comp.camera_from_angle(subject.position, dist, azimuth_deg, 4.0)
    sx, sy, sz = subject.position
    # Target tilts from a point on the ground in front of subject up to its eyes.
    ground_target = (sx, sy, sz)
    eye_target = (sx, sy, sz + subject.profile.eye_height_m)
    return [_kf(0.0, pos, ground_target), _kf(duration_s, pos, eye_target)]


# ---------------------------------------------------------------------------
# Dispatcher
# ---------------------------------------------------------------------------

PRIMITIVES = {
    "establish": establish,
    "beauty": beauty,
    "orbit": orbit,
    "push_in": push_in,
    "pull_out": pull_out,
    "tracking_side": tracking_side,
    "tracking_chase": tracking_chase,
    "crane_up": crane_up,
    "bird_eye": bird_eye,
    "reveal_tilt": reveal_tilt,
}


def render_keyframes(
    primitive_name: str, subject: Subject, duration_s: float,
    lens_focal_mm: float, params: dict | None = None,
) -> list[Keyframe]:
    """Resolve a primitive by name and return its keyframes. `params` are
    forwarded as kwargs — invalid keys silently dropped so storyboards stay
    permissive."""
    fn = PRIMITIVES.get(primitive_name)
    if fn is None:
        raise ValueError(
            f"unknown shot primitive {primitive_name!r}; "
            f"known: {sorted(PRIMITIVES.keys())}"
        )
    import inspect
    sig = inspect.signature(fn)
    allowed = set(sig.parameters.keys())
    kwargs = {k: v for k, v in (params or {}).items() if k in allowed}
    kwargs["duration_s"] = duration_s
    if "lens_focal_mm" in allowed:
        kwargs["lens_focal_mm"] = lens_focal_mm
    return fn(subject, **kwargs)
