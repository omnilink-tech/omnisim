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

"""spatial — camera framing, orbiting and screen projection for the harness.

Pure math. No IPC, no I/O beyond loading the reference module below.

Single source of truth
----------------------
The framing convention (``look_at`` orientation, and the distance that fits a
bounding sphere in frame) is defined ONCE, in
``src/python/omniworld/viewpoint.py`` — the same module the world generators
and ``scripts/dev/set_viewpoint.py`` use to bake ``Viewpoint`` blocks into
``.wbt`` files. This module **loads that file directly** rather than
re-deriving the math, so a harness-framed camera and a generator-baked camera
can never disagree. (It is loaded by path, bypassing ``omniworld/__init__.py``,
so the harness keeps its zero-dependency, stdlib-only property.)

Conventions this module implements, all inherited from the engine:

* **Camera local frame is +X forward, +Y left, +Z up**; worlds are Z-up
  (``WorldInfo.coordinateSystem`` defaults to ``"ENU"``).
* **``Viewpoint.fieldOfView`` is VRML semantics**: the angle subtended on the
  *larger* viewport dimension, not the vertical one
  (``OmViewpoint::updateFieldOfViewY``). On a landscape window it is therefore
  the horizontal FOV and the vertical one is the tighter constraint.
* **Screen projection matches ``OmViewpoint::eyeToPixels``**: pixel origin at
  the top-left, ``y`` growing downward.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_REFERENCE_PATH = _REPO_ROOT / "src" / "python" / "omniworld" / "viewpoint.py"


def _load_reference():
    """Load ``omniworld.viewpoint`` as a standalone module.

    Loading by path skips ``omniworld/__init__.py`` (which pulls in the whole
    world-generation library) while still using the *identical* file, so there
    is exactly one implementation of the framing math in the tree.
    """
    if _REFERENCE_PATH.is_file():
        spec = importlib.util.spec_from_file_location(
            "omnisim_harness_viewpoint_ref", _REFERENCE_PATH
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    # Installed layout without src/: fall back to the package import.
    src_python = str(_REPO_ROOT / "src" / "python")
    if src_python not in sys.path:
        sys.path.insert(0, src_python)
    from omniworld import viewpoint as module  # noqa: PLC0415
    return module


reference = _load_reference()

look_at = reference.look_at
frame_distance = reference._frame_distance  # noqa: SLF001 — deliberate single source
hero_view = reference.hero_view
top_down_view = reference.top_down_view
HERO_DIRECTION = reference.HERO_DIRECTION
DEFAULT_FOV = reference.DEFAULT_FOV
DEFAULT_ASPECT = reference.DEFAULT_ASPECT

# Subject-relative camera placements. Each entry is (direction, up):
# `direction` is the unit vector FROM the subject TO the eye, expressed in the
# subject's own frame (+X forward, +Y left, +Z up — the robot convention), and
# `up` is the world-up hint handed to look_at.
VIEW_MODES: dict = {
    "hero": (HERO_DIRECTION, (0.0, 0.0, 1.0)),
    "front": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "back": ((-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "left": ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    "right": ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0)),
    "top": ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
    "top_down": ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
    "bottom": ((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
}
# Aliases callers reach for.
VIEW_MODE_ALIASES = {
    "topdown": "top_down",
    "overview": "top_down",
    "3/4": "hero",
    "default": "hero",
    "side": "left",
}

DEFAULT_MARGIN = 1.3


# --- vectors ----------------------------------------------------------------

def normalize(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(v, s):
    return (v[0] * s, v[1] * s, v[2] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def length(v):
    return math.sqrt(dot(v, v))


def axis_angle_to_matrix(rot):
    """Axis-angle ``[x, y, z, angle]`` -> row-major 3x3 rotation matrix.

    Columns of the result are the camera's world-space axes when ``rot`` is a
    ``Viewpoint.orientation``: column 0 = forward (+X local), column 1 = left,
    column 2 = up.
    """
    ax, ay, az, angle = (float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3]))
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12 or abs(angle) < 1e-15:
        return (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    ax, ay, az = ax / n, ay / n, az / n
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return (
        t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay,
        t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax,
        t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c,
    )


def camera_axes(orientation):
    """``(forward, left, up)`` world unit vectors for a Viewpoint orientation."""
    m = axis_angle_to_matrix(orientation)
    forward = (m[0], m[3], m[6])
    left = (m[1], m[4], m[7])
    up = (m[2], m[5], m[8])
    return forward, left, up


# --- field of view ----------------------------------------------------------

def fov_axes(fov: float, aspect: float) -> dict:
    """Split ``Viewpoint.fieldOfView`` into its horizontal + vertical angles.

    Mirrors ``OmViewpoint::updateFieldOfViewY``: ``fieldOfView`` is the angle on
    the LARGER viewport dimension, so on a landscape viewport (aspect >= 1) it
    is the horizontal FOV and ``tan(fovY/2) = tan(fov/2) / aspect``.
    """
    fov = max(min(float(fov), math.pi - 1e-6), 1e-6)
    aspect = max(float(aspect), 1e-6)
    tan_half = math.tan(0.5 * fov)
    if aspect < 1.0:
        tan_half_v = tan_half
    else:
        tan_half_v = tan_half / aspect
    tan_half_h = tan_half_v * aspect
    return {
        "aspect": aspect,
        "field_of_view": fov,
        "tan_half_v": tan_half_v,
        "tan_half_h": tan_half_h,
        "fov_h_rad": 2.0 * math.atan(tan_half_h),
        "fov_v_rad": 2.0 * math.atan(tan_half_v),
        "fov_h_deg": math.degrees(2.0 * math.atan(tan_half_h)),
        "fov_v_deg": math.degrees(2.0 * math.atan(tan_half_v)),
        "half_fov_h_deg": math.degrees(math.atan(tan_half_h)),
        "half_fov_v_deg": math.degrees(math.atan(tan_half_v)),
    }


# --- projection -------------------------------------------------------------

def to_eye(point, eye, orientation):
    """World point -> camera-local coordinates (x forward, y left, z up).

    Matches ``OmViewpoint::toPixels``: ``eye = R^T * (point - position)``.
    """
    m = axis_angle_to_matrix(orientation)
    d = sub(point, eye)
    return (m[0] * d[0] + m[3] * d[1] + m[6] * d[2],
            m[1] * d[0] + m[4] * d[1] + m[7] * d[2],
            m[2] * d[0] + m[5] * d[1] + m[8] * d[2])


def project(point, eye, orientation, fov, aspect, width=None, height=None,
            near=0.05):
    """Project a world point into normalized + pixel screen coordinates.

    Returns a dict describing where the point lands and, when it does not land
    on screen, how far off it is in degrees. Normalized coordinates ``ndc_x`` /
    ``ndc_y`` run 0..1 across the viewport with the origin at the TOP-LEFT,
    which is the convention ``OmViewpoint::eyeToPixels`` uses.
    """
    axes = fov_axes(fov, aspect)
    ex, ey, ez = to_eye(point, eye, orientation)
    distance = math.sqrt(ex * ex + ey * ey + ez * ez)
    # Signed angles off the view axis. Positive yaw = to the LEFT of centre,
    # positive pitch = ABOVE centre (matching the +Y-left / +Z-up camera frame).
    yaw = math.degrees(math.atan2(ey, ex)) if (ex or ey) else 0.0
    horizontal = math.sqrt(ex * ex + ey * ey)
    pitch = math.degrees(math.atan2(ez, horizontal)) if (ez or horizontal) else 0.0
    behind = ex <= near
    out = {
        "eye_coords": [round(ex, 6), round(ey, 6), round(ez, 6)],
        "distance": round(distance, 6),
        "depth": round(ex, 6),
        "behind_camera": bool(behind),
        "yaw_deg": round(yaw, 3),
        "pitch_deg": round(pitch, 3),
        "angle_off_axis_deg": round(
            math.degrees(math.acos(max(-1.0, min(1.0, ex / distance)))) if distance > 1e-9 else 0.0,
            3),
        "half_fov_h_deg": round(axes["half_fov_h_deg"], 3),
        "half_fov_v_deg": round(axes["half_fov_v_deg"], 3),
    }
    if behind:
        out["in_frame"] = False
        out["ndc_x"] = None
        out["ndc_y"] = None
        out["pixel"] = None
        return out
    factor = 0.5 / (ex * axes["tan_half_v"])
    h = -factor * ez
    w = -factor * ey / axes["aspect"]
    ndc_x = w + 0.5
    ndc_y = h + 0.5
    out["ndc_x"] = round(ndc_x, 6)
    out["ndc_y"] = round(ndc_y, 6)
    out["in_frame"] = bool(0.0 <= ndc_x <= 1.0 and 0.0 <= ndc_y <= 1.0)
    if width and height:
        out["pixel"] = [round(ndc_x * float(width), 2), round(ndc_y * float(height), 2)]
    else:
        out["pixel"] = None
    return out


def offset_hint(proj: dict, partial: bool | None = None) -> str:
    """A short human/agent-usable description of where something is.

    ``partial`` lets the caller distinguish "the centroid is outside the frame
    but part of the object is still on screen" from a fully off-screen object —
    the projection alone only knows about the centroid.
    """
    if proj.get("behind_camera"):
        return "off-screen: behind the camera"
    yaw = proj.get("yaw_deg") or 0.0
    pitch = proj.get("pitch_deg") or 0.0
    parts = []
    if abs(yaw) >= 0.5:
        parts.append(f"{abs(yaw):.0f} deg to the {'left' if yaw > 0 else 'right'}")
    if abs(pitch) >= 0.5:
        parts.append(f"{abs(pitch):.0f} deg {'up' if pitch > 0 else 'down'}")
    where = ", ".join(parts) if parts else "on the view axis"
    if proj.get("in_frame"):
        return f"in frame: {where}"
    if partial:
        return f"partly in frame: centre is {where}"
    return f"off-screen: {where}"


# --- framing ----------------------------------------------------------------

def resolve_mode(mode: str | None) -> str:
    m = (mode or "hero").strip().lower()
    m = VIEW_MODE_ALIASES.get(m, m)
    if m not in VIEW_MODES:
        raise ValueError(
            f"unknown mode {mode!r}; expected one of "
            f"{sorted(set(VIEW_MODES) | set(VIEW_MODE_ALIASES))}"
        )
    return m


def frame_pose(center, radius, mode="hero", fov=None, aspect=None,
               margin=DEFAULT_MARGIN, subject_rotation=None):
    """Eye position + orientation that frames a sphere.

    ``subject_rotation`` is an optional row-major 3x3 world rotation of the
    subject; when given, the mode's direction is interpreted in the subject's
    own frame (so ``"front"`` really means the subject's front, not world +X).

    Distance comes from ``omniworld.viewpoint._frame_distance`` — the tight
    viewport axis, not the ``fieldOfView`` axis — so a tall subject on a
    landscape viewport does not overflow vertically.
    """
    mode = resolve_mode(mode)
    fov = DEFAULT_FOV if fov is None else float(fov)
    aspect = DEFAULT_ASPECT if aspect is None else float(aspect)
    radius = max(float(radius), 1e-3)
    direction, up = VIEW_MODES[mode]
    if subject_rotation is not None:
        m = subject_rotation
        direction = (m[0] * direction[0] + m[1] * direction[1] + m[2] * direction[2],
                     m[3] * direction[0] + m[4] * direction[1] + m[5] * direction[2],
                     m[6] * direction[0] + m[7] * direction[1] + m[8] * direction[2])
    direction = normalize(direction)
    dist = frame_distance(radius, fov, margin, aspect)
    eye = (center[0] + direction[0] * dist,
           center[1] + direction[1] * dist,
           center[2] + direction[2] * dist)
    orientation = look_at(eye, center, up=up)
    return list(eye), list(orientation), {
        "mode": mode,
        "distance": round(dist, 6),
        "direction": [round(v, 6) for v in direction],
        "margin": margin,
        "aspect": aspect,
        "field_of_view": fov,
        "radius": round(radius, 6),
    }


def framing_verification(center, radius, eye, orientation, fov, aspect,
                         width=None, height=None):
    """Prove (numerically) that a subject sphere is inside the frame.

    Compares the subject's angular radius against the available half-FOV on
    both axes, and projects its centre. ``fits`` is the honest verdict: it is
    true only when the whole sphere clears both axes.
    """
    proj = project(center, eye, orientation, fov, aspect, width, height)
    axes = fov_axes(fov, aspect)
    dist = length(sub(center, eye))
    if dist > radius:
        angular_radius = math.degrees(math.asin(max(-1.0, min(1.0, radius / dist))))
    else:
        angular_radius = 90.0
    margin_h = axes["half_fov_h_deg"] - (abs(proj["yaw_deg"]) + angular_radius)
    margin_v = axes["half_fov_v_deg"] - (abs(proj["pitch_deg"]) + angular_radius)
    return {
        "center_projection": proj,
        "hint": offset_hint(proj),
        "subject_angular_radius_deg": round(angular_radius, 3),
        "half_fov_h_deg": round(axes["half_fov_h_deg"], 3),
        "half_fov_v_deg": round(axes["half_fov_v_deg"], 3),
        "headroom_h_deg": round(margin_h, 3),
        "headroom_v_deg": round(margin_v, 3),
        "center_in_frame": bool(proj["in_frame"]),
        "fits": bool(proj["in_frame"] and margin_h >= 0.0 and margin_v >= 0.0),
        "eye_to_center_distance": round(dist, 6),
    }


# --- orbiting ---------------------------------------------------------------

def spherical_from_offset(offset):
    """``(radius, azimuth_deg, elevation_deg)`` for an eye-minus-centre vector.

    Azimuth is measured in the world XY plane from +X toward +Y; elevation is
    the angle above that plane. Both in degrees.
    """
    r = length(offset)
    if r < 1e-9:
        return 0.0, 0.0, 0.0
    az = math.degrees(math.atan2(offset[1], offset[0]))
    el = math.degrees(math.asin(max(-1.0, min(1.0, offset[2] / r))))
    return r, az, el


def offset_from_spherical(radius, azimuth_deg, elevation_deg):
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)
    c = math.cos(el)
    return (radius * c * math.cos(az), radius * c * math.sin(az),
            radius * math.sin(el))


MAX_ELEVATION_DEG = 89.0


def orbit_pose(eye, orientation, center, azimuth_deg=0.0, elevation_deg=0.0,
               dolly=1.0, pan=None, up=(0.0, 0.0, 1.0)):
    """Nudge the camera relative to its CURRENT pose, around ``center``.

    ``azimuth_deg`` swings around the world +Z axis, ``elevation_deg`` raises
    or lowers the eye (clamped to +/-89 deg so the look-at never degenerates),
    ``dolly`` multiplies the orbit radius (>1 pulls back, <1 pushes in), and
    ``pan`` is ``[dx, dy]`` metres in SCREEN space (right, up) applied to the
    eye and the centre together.

    Returns ``(eye, orientation, meta)``.
    """
    eye = (float(eye[0]), float(eye[1]), float(eye[2]))
    center = (float(center[0]), float(center[1]), float(center[2]))
    if pan:
        _forward, left, cam_up = camera_axes(orientation)
        right = scale(left, -1.0)
        shift = add(scale(right, float(pan[0])), scale(cam_up, float(pan[1])))
        eye = add(eye, shift)
        center = add(center, shift)
    radius, az, el = spherical_from_offset(sub(eye, center))
    if radius < 1e-9:
        radius = 1e-3
    new_radius = max(1e-3, radius * max(float(dolly), 1e-6))
    new_az = az + float(azimuth_deg)
    new_el = max(-MAX_ELEVATION_DEG,
                 min(MAX_ELEVATION_DEG, el + float(elevation_deg)))
    new_eye = add(center, offset_from_spherical(new_radius, new_az, new_el))
    orientation_out = look_at(new_eye, center, up=up)
    return list(new_eye), list(orientation_out), {
        "center": [round(v, 6) for v in center],
        "radius_before": round(radius, 6),
        "radius_after": round(new_radius, 6),
        "azimuth_deg_before": round(az, 3),
        "azimuth_deg_after": round(new_az, 3),
        "elevation_deg_before": round(el, 3),
        "elevation_deg_after": round(new_el, 3),
        "elevation_clamped": bool(abs(el + float(elevation_deg)) > MAX_ELEVATION_DEG),
    }
