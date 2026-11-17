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

"""Camera viewpoint framing — the single source for OmniSim's default world camera.

OmniSim worlds are Z-up (``coordinateSystem "ENU"``, the WorldInfo default —
``"NUE"`` is the Y-up system, and no world in this tree selects it). The camera's
local frame is +X forward, +Y left, +Z up. A ``Viewpoint`` node is
defined by an eye ``position`` (xyz, metres) and an ``orientation`` in
axis-angle form (``x y z angle`` with the angle in radians). When a world omits
a good ``Viewpoint`` the engine falls back to a fixed ``position -10 0 0``, so
the subject is almost always off-screen or tiny — that is the "opens in a random
place" symptom.

This module computes a framing that *centres the subject*. The convention is
documented in ``docs/developer/viewpoint-convention.md``; this code is its
single implementation, shared by the world generators
(``omniworld.emit.wbt``, ``scripts/dev/gen_*``) and the retrofit tool
(``scripts/dev/set_viewpoint.py``).

Two framings:

* ``hero`` — **the default.** An elevated 3/4 view along :data:`HERO_DIRECTION`
  that shows a robot's full vertical structure. Use for single-robot and
  manipulation worlds.
* ``top_down`` — a straight overhead view (up = +X to avoid the Z-up
  degeneracy). An opt-in for navigation / overview / fleet / city worlds where
  the layout matters more than any one robot's silhouette.

Both ``hero_view`` and ``top_down_view`` take a subject *centre* and a framing
*radius* (the radius of the bounding sphere you want to fill the frame) and
return ``(eye_position, orientation)``. Distance is derived from the field of
view so the subject fills the frame regardless of scene scale.
"""
from __future__ import annotations

import math

Vec3 = tuple[float, float, float]
Rotation = tuple[float, float, float, float]

# Canonical hero direction: the eye sits along this vector *from* the subject.
# Front-right, ~33 deg elevation (azimuth ~ -47 deg off +X toward -Y) — the same
# flattering 3/4 look the city generator uses. Only the direction matters; the
# distance is computed to fit the subject, so the magnitude here is irrelevant.
HERO_DIRECTION: Vec3 = (0.62, -0.66, 0.58)

# Field of view (radians). Matches the Viewpoint schema default (pi/4).
#
# NOTE the VRML semantics the engine implements (WbViewpoint::updateFieldOfViewY):
# ``fieldOfView`` is the angle subtended on the *larger* viewport dimension, not
# the vertical one. On a landscape window it is therefore the HORIZONTAL fov and
# the (tighter) vertical fov is derived from the aspect ratio. Framing distance
# must be computed from the tight axis or the subject overflows the frame —
# see :func:`_frame_distance`.
DEFAULT_FOV: float = math.pi / 4.0

# Viewport aspect ratio (width / height) assumed when baking a framing distance
# into a ``.wbt``. The engine uses the real 3D-view aspect at runtime; 16:9 is
# the common case and the conservative choice for a windowed default.
DEFAULT_ASPECT: float = 16.0 / 9.0

# Default framing presets by subject class. ``radius`` is the bounding-sphere
# radius (metres) to fill the frame; ``look_z`` is how far above the spawn point
# to aim so the camera centres on the body, not the feet. ``radius=None`` means
# "size from the scene extent" (fleet/scene worlds). Keep these in sync with the
# table in docs/developer/viewpoint-convention.md.
SUBJECT_PRESETS: dict[str, dict] = {
    "arm":       {"radius": 1.0, "look_z": 0.45, "mode": "hero"},
    "humanoid":  {"radius": 1.3, "look_z": 0.75, "mode": "hero"},
    "quadruped": {"radius": 1.4, "look_z": 0.45, "mode": "hero"},
    "mobile":    {"radius": 1.2, "look_z": 0.25, "mode": "hero"},
    "fleet":     {"radius": None, "look_z": 0.30, "mode": "topdown"},
    "scene":     {"radius": None, "look_z": 0.50, "mode": "hero"},
}

__all__ = [
    "HERO_DIRECTION",
    "DEFAULT_FOV",
    "DEFAULT_ASPECT",
    "SUBJECT_PRESETS",
    "look_at",
    "hero_view",
    "top_down_view",
    "frame",
    "format_orientation",
    "format_position",
    "viewpoint_block",
]


def _normalize(v: tuple[float, float, float]) -> Vec3:
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def look_at(eye, target, up: Vec3 = (0.0, 0.0, 1.0)) -> Rotation:
    """Axis-angle orientation for a camera at ``eye`` looking at ``target``.

    Returns ``(x, y, z, angle)`` with the angle in radians, matching the
    convention used across OmniSim ``.wbt`` Viewpoint nodes. ``up`` is
    re-orthogonalised; if it is (anti)parallel to the view direction — e.g. a
    straight top-down view with the default +Z up — it falls back to +Y then +X
    so the result never degenerates.
    """
    f = _normalize((target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]))
    if f == (0.0, 0.0, 0.0):
        return (0.0, 0.0, 1.0, 0.0)

    up = _normalize(up)
    if 1.0 - abs(f[0] * up[0] + f[1] * up[1] + f[2] * up[2]) < 1e-6:
        up = (0.0, 1.0, 0.0)
        if 1.0 - abs(f[0] * up[0] + f[1] * up[1] + f[2] * up[2]) < 1e-6:
            up = (1.0, 0.0, 0.0)

    d = f[0] * up[0] + f[1] * up[1] + f[2] * up[2]
    u = _normalize((up[0] - d * f[0], up[1] - d * f[1], up[2] - d * f[2]))
    # right-handed: y = u x f
    y = (
        u[1] * f[2] - u[2] * f[1],
        u[2] * f[0] - u[0] * f[2],
        u[0] * f[1] - u[1] * f[0],
    )
    R = [
        [f[0], y[0], u[0]],
        [f[1], y[1], u[1]],
        [f[2], y[2], u[2]],
    ]
    tr = R[0][0] + R[1][1] + R[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    if ang < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)

    s = 2.0 * math.sin(ang)
    if abs(s) < 1e-9:
        # ~180 deg: extract the axis from the diagonal (R = 2 a a^T - I).
        diag = (R[0][0], R[1][1], R[2][2])
        i = diag.index(max(diag))
        a = [0.0, 0.0, 0.0]
        a[i] = math.sqrt(max(0.0, (R[i][i] + 1.0) / 2.0))
        for j in range(3):
            if j != i:
                a[j] = (R[i][j] + R[j][i]) / (4.0 * a[i]) if a[i] > 1e-9 else 0.0
        ax = _normalize((a[0], a[1], a[2]))
        return (ax[0], ax[1], ax[2], ang)

    return ((R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, ang)


def _frame_distance(radius: float, fov: float, margin: float,
                    aspect: float = DEFAULT_ASPECT) -> float:
    """Eye distance that fits a sphere of ``radius`` inside the frame.

    ``radius / sin(half)`` is the tangent fit to the *sphere* (not a flat disc
    of that radius at the target plane — that would be ``tan`` and under-frames
    by ``cos(half)``).

    ``fov`` is the engine's ``fieldOfView``: the angle on the LARGER viewport
    dimension. The binding constraint is the tighter (smaller) axis, so derive
    it from the aspect ratio before fitting. Omitting this term is what made
    hero shots of tall subjects overflow the frame vertically on a 16:9 window.
    """
    radius = max(float(radius), 1e-3)
    half = max(min(fov, math.pi - 1e-3), 1e-3) / 2.0
    aspect = max(float(aspect), 1e-3)
    # Half-angle on the tight axis. tan(half_tight) = tan(half_large) / (long/short).
    tan_tight = math.tan(half) / (aspect if aspect >= 1.0 else 1.0 / aspect)
    half_tight = math.atan(tan_tight)
    return radius / math.sin(half_tight) * margin


def hero_view(center, radius, fov: float = DEFAULT_FOV, margin: float = 1.3,
              direction: Vec3 = HERO_DIRECTION, aspect: float = DEFAULT_ASPECT):
    """Eye position + orientation for the default angled 3/4 "hero" framing.

    Frames a sphere of ``radius`` around ``center`` viewed from
    :data:`HERO_DIRECTION`. Returns ``(eye, orientation)``.
    """
    dist = _frame_distance(radius, fov, margin, aspect)
    u = _normalize(direction)
    eye = (center[0] + u[0] * dist, center[1] + u[1] * dist, center[2] + u[2] * dist)
    return eye, look_at(eye, center)


def top_down_view(center, radius, fov: float = DEFAULT_FOV, margin: float = 1.15,
                  aspect: float = DEFAULT_ASPECT):
    """Eye position + orientation for a straight overhead framing.

    Up is +X so the world's +X (forward) axis points "up" on screen. Returns
    ``(eye, orientation)``. Use for navigation / overview / fleet / city worlds.
    """
    dist = _frame_distance(radius, fov, margin, aspect)
    eye = (center[0], center[1], center[2] + dist)
    return eye, look_at(eye, center, up=(1.0, 0.0, 0.0))


def frame(center, radius, mode: str = "hero", **kwargs):
    """Dispatch to :func:`hero_view` (default) or :func:`top_down_view`."""
    if mode in ("topdown", "top_down", "overview"):
        return top_down_view(center, radius, **kwargs)
    return hero_view(center, radius, **kwargs)


def format_orientation(o: Rotation) -> str:
    return "{:.6f} {:.6f} {:.6f} {:.6f}".format(*o)


def format_position(p: Vec3) -> str:
    # 3 decimals (mm) is plenty for a camera and keeps the .wbt diff tidy.
    return "{:.3f} {:.3f} {:.3f}".format(*p)


def viewpoint_block(center, radius, mode: str = "hero", fov: float | None = None,
                    extra_fields: list[tuple[str, str]] | None = None,
                    indent: str = "  ", **kwargs) -> str:
    """Render a complete ``Viewpoint { ... }`` block as text.

    ``extra_fields`` are appended verbatim as ``(key, value)`` pairs (e.g.
    ``[("follow", '"spot"'), ("followType", '"Mounted Shot"')]``).
    """
    if fov is not None:
        kwargs["fov"] = fov
    eye, o = frame(center, radius, mode=mode, **kwargs)
    lines = ["Viewpoint {",
             f"{indent}orientation {format_orientation(o)}",
             f"{indent}position {format_position(eye)}"]
    if fov is not None and abs(fov - DEFAULT_FOV) > 1e-6:
        lines.append(f"{indent}fieldOfView {fov:.4f}")
    for key, value in (extra_fields or []):
        lines.append(f"{indent}{key} {value}")
    lines.append("}")
    return "\n".join(lines)
