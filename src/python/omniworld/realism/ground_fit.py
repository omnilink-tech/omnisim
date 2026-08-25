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

"""Ground fitting: tilt a prop to match the local terrain normal.

The plan's T1.9 physics settling is the real answer to "props float or
clip on slopes" — but full physics settling means spawning a headless
OmniSim subprocess for each prop, which is expensive enough that the
plan only deploys it with aggressive caching.

This module is the cheap approximation: sample the terrain heightmap
under the prop's footprint, fit a plane to the samples, derive the
plane normal, and produce a rotation that lines the prop's up axis
with that normal plus a small per-instance random twist. It does not
account for contact with other props, but it eliminates 90% of the
"rock pasted on a hillside" effect that a single-height lookup
produces.

The output rotation is in OmniSim axis-angle form
``(ax, ay, az, angle)``, suitable for dropping straight into
``PlacedProto.rotation``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from ..primitives.heightmap import Heightmap


@dataclass(frozen=True)
class GroundFit:
    """Result of ground-fitting one prop."""

    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]  # ax, ay, az, angle
    slope_deg: float                              # for diagnostics


def _sample_grid_xy(
    hm: Heightmap,
    world_x: float,
    world_y: float,
    *,
    origin: tuple[float, float],
    x_spacing: float,
    y_spacing: float,
    height_scale: float = 1.0,
) -> float:
    gx = (world_x - origin[0]) / x_spacing
    gy = (world_y - origin[1]) / y_spacing
    return hm.sample(gx, gy) * height_scale


def _fit_plane_normal(
    samples: Sequence[tuple[float, float, float]],
) -> tuple[float, float, float]:
    """Return the up-facing unit normal of a least-squares plane fit.

    ``samples`` is a list of ``(x, y, z)`` points. For the small sample
    counts we use (4-8 points) the normal equation
    ``(X^T X) beta = X^T z`` is fine — we do not need SVD. Returns
    a normal whose z component is positive (fits will occasionally flip
    otherwise).
    """
    if len(samples) < 3:
        return (0.0, 0.0, 1.0)

    # Plane: z = a*x + b*y + c. Build normal equations.
    sx = sy = sz = sxx = syy = sxy = sxz = syz = 0.0
    n = float(len(samples))
    for x, y, z in samples:
        sx += x
        sy += y
        sz += z
        sxx += x * x
        syy += y * y
        sxy += x * y
        sxz += x * z
        syz += y * z

    # 3x3 matrix:
    # [ sxx sxy sx ] [a]   [sxz]
    # [ sxy syy sy ] [b] = [syz]
    # [ sx  sy  n  ] [c]   [sz ]
    # Solve by hand.
    m00, m01, m02 = sxx, sxy, sx
    m10, m11, m12 = sxy, syy, sy
    m20, m21, m22 = sx, sy, n

    det = (
        m00 * (m11 * m22 - m12 * m21)
        - m01 * (m10 * m22 - m12 * m20)
        + m02 * (m10 * m21 - m11 * m20)
    )
    if abs(det) < 1e-12:
        return (0.0, 0.0, 1.0)

    b0, b1, b2 = sxz, syz, sz
    inv_det = 1.0 / det

    a = (
        b0 * (m11 * m22 - m12 * m21)
        - m01 * (b1 * m22 - m12 * b2)
        + m02 * (b1 * m21 - m11 * b2)
    ) * inv_det
    b = (
        m00 * (b1 * m22 - m12 * b2)
        - b0 * (m10 * m22 - m12 * m20)
        + m02 * (m10 * b2 - b1 * m20)
    ) * inv_det
    # c is not needed for the normal.

    # Plane z = a*x + b*y + c has normal (-a, -b, 1); normalise.
    nx, ny, nz = -a, -b, 1.0
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    return (nx / length, ny / length, nz / length)


def _align_up_to_normal(
    normal: tuple[float, float, float],
    twist_rad: float = 0.0,
) -> tuple[float, float, float, float]:
    """Return a OmniSim axis-angle rotation that sends +Z to ``normal``,
    optionally composed with a twist around the normal.

    When ``normal`` is already +Z the result is a pure twist. When it
    differs we pick the rotation axis ``+Z x normal`` and the angle
    ``acos(Z . normal)``.
    """
    nx, ny, nz = normal
    # Normalise defensively.
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length == 0.0:
        return (0.0, 0.0, 1.0, twist_rad)
    nx, ny, nz = nx / length, ny / length, nz / length

    cos_angle = max(-1.0, min(1.0, nz))
    angle_to_up = math.acos(cos_angle)
    if angle_to_up < 1e-6:
        # Normal ≈ +Z; just apply the twist.
        return (0.0, 0.0, 1.0, twist_rad)

    # Axis = cross(+Z, normal) = (-ny, nx, 0); normalise.
    ax, ay, az = -ny, nx, 0.0
    axl = math.sqrt(ax * ax + ay * ay + az * az)
    if axl == 0.0:
        return (0.0, 0.0, 1.0, twist_rad)
    ax, ay, az = ax / axl, ay / axl, az / axl

    # If no twist, the up-alignment alone is the result.
    if abs(twist_rad) < 1e-9:
        return (ax, ay, az, angle_to_up)

    # Compose: first align up, then twist around the new (normal) axis.
    # For OmniSim' ``Solid.rotation`` we can get away with a *single*
    # axis-angle, but composing two rotations into one requires
    # quaternion multiplication. Do it.
    qa = _quat_from_axis_angle(ax, ay, az, angle_to_up)
    qb = _quat_from_axis_angle(nx, ny, nz, twist_rad)
    q = _quat_mul(qb, qa)
    return _quat_to_axis_angle(q)


def _quat_from_axis_angle(ax: float, ay: float, az: float, angle: float) -> tuple[float, float, float, float]:
    half = angle * 0.5
    s = math.sin(half)
    return (math.cos(half), ax * s, ay * s, az * s)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_to_axis_angle(q) -> tuple[float, float, float, float]:
    w, x, y, z = q
    # Clamp for safety.
    if w > 1.0:
        w = 1.0
    elif w < -1.0:
        w = -1.0
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-6:
        return (0.0, 0.0, 1.0, 0.0)
    return (x / s, y / s, z / s, angle)


def ground_fit_on_heightmap(
    world_xy: tuple[float, float],
    hm: Heightmap,
    *,
    origin: tuple[float, float],
    x_spacing: float,
    y_spacing: float,
    height_scale: float = 1.0,
    footprint_radius: float = 0.35,
    sample_count: int = 6,
    sink_offset: float = 0.0,
    twist_rad: float = 0.0,
    max_tilt_rad: float | None = None,
) -> GroundFit:
    """Fit a prop to the heightmap at ``world_xy``.

    - Samples terrain at ``sample_count`` points around
      ``footprint_radius``, plus the centre. A plane is fitted to the
      samples; the prop is placed at the centre z (optionally sunk by
      ``sink_offset`` metres) with a rotation that aligns its +Z with
      the plane normal.
    - ``max_tilt_rad`` optionally caps the tilt magnitude — on very
      steep sections (above angle-of-repose) we clamp so big rocks
      do not end up nearly horizontal.
    - ``twist_rad`` is an additional per-instance yaw, so rocks face
      different directions even on identical slopes.

    Returns a ``GroundFit`` carrying the corrected translation,
    OmniSim axis-angle rotation, and the diagnostic slope in degrees.
    """
    wx, wy = world_xy

    # Centre + ring of samples around the footprint.
    samples: list[tuple[float, float, float]] = []
    centre_z = _sample_grid_xy(
        hm, wx, wy, origin=origin,
        x_spacing=x_spacing, y_spacing=y_spacing, height_scale=height_scale,
    )
    samples.append((wx, wy, centre_z))
    for k in range(sample_count):
        theta = 2.0 * math.pi * k / sample_count
        sx = wx + footprint_radius * math.cos(theta)
        sy = wy + footprint_radius * math.sin(theta)
        sz = _sample_grid_xy(
            hm, sx, sy, origin=origin,
            x_spacing=x_spacing, y_spacing=y_spacing, height_scale=height_scale,
        )
        samples.append((sx, sy, sz))

    normal = _fit_plane_normal(samples)
    nx, ny, nz = normal

    slope_rad = math.acos(max(-1.0, min(1.0, nz)))
    if max_tilt_rad is not None and slope_rad > max_tilt_rad:
        # Clamp: shrink (nx, ny) so tilt magnitude stops at the cap.
        # Keep direction of tilt.
        new_nz = math.cos(max_tilt_rad)
        horiz_mag = math.sqrt(nx * nx + ny * ny)
        if horiz_mag > 0.0:
            target_horiz = math.sin(max_tilt_rad)
            scale = target_horiz / horiz_mag
            nx *= scale
            ny *= scale
        nz = new_nz
        slope_rad = max_tilt_rad

    rotation = _align_up_to_normal((nx, ny, nz), twist_rad=twist_rad)
    translation = (wx, wy, centre_z + sink_offset)
    return GroundFit(
        translation=translation,
        rotation=rotation,
        slope_deg=math.degrees(slope_rad),
    )


__all__ = ["GroundFit", "ground_fit_on_heightmap"]
