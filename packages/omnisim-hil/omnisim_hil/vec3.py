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

"""Minimal 3-vector and rotation helpers, stdlib only.

Vectors are plain ``(x, y, z)`` tuples and rotations are row-major 9-tuples, so
nothing here depends on numpy: this code runs inside the interpreter OmniSim
spawns for controllers, which is not guaranteed to have it.

Frame convention for everything in :mod:`omnisim_hil`
----------------------------------------------------
Body frame is **FLU**: +X forward (nose), +Y left, +Z up. That matches URDF and
the frame OmniSim gives a Solid, so no conversion is needed between this model
and the simulator. MAVLink speaks **FRD/NED** instead, and that conversion is
done once, explicitly, at the protocol boundary in :mod:`omnisim_hil.frames` --
never silently in the middle of the physics.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[float, float, float, float, float, float, float, float, float]

ZERO: Vec3 = (0.0, 0.0, 0.0)
X_AXIS: Vec3 = (1.0, 0.0, 0.0)
Y_AXIS: Vec3 = (0.0, 1.0, 0.0)
Z_AXIS: Vec3 = (0.0, 0.0, 1.0)

IDENTITY: Mat3 = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def neg(a: Vec3) -> Vec3:
    return (-a[0], -a[1], -a[2])


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a: Vec3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def normalize(a: Vec3, fallback: Vec3 = X_AXIS) -> Vec3:
    n = norm(a)
    if n < 1e-12:
        return fallback
    return (a[0] / n, a[1] / n, a[2] / n)


def project_out(v: Vec3, axis_unit: Vec3) -> Vec3:
    """Component of ``v`` perpendicular to the unit vector ``axis_unit``.

    Used to strip spanwise flow before computing angle of attack: flow running
    along a wing's span does not turn into lift, and including it inflates both
    the dynamic pressure and the angle in a sideslip.
    """
    d = dot(v, axis_unit)
    return (v[0] - d * axis_unit[0], v[1] - d * axis_unit[1], v[2] - d * axis_unit[2])


def mat_vec(m: Mat3, v: Vec3) -> Vec3:
    """``m @ v`` with ``m`` row-major."""
    return (
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    )


def mat_t_vec(m: Mat3, v: Vec3) -> Vec3:
    """``m.T @ v``. With ``m`` a body-to-world rotation this maps world to body."""
    return (
        m[0] * v[0] + m[3] * v[1] + m[6] * v[2],
        m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
        m[2] * v[0] + m[5] * v[1] + m[8] * v[2],
    )


def mat_from_rows(rows: Sequence[Sequence[float]]) -> Mat3:
    return (
        float(rows[0][0]), float(rows[0][1]), float(rows[0][2]),
        float(rows[1][0]), float(rows[1][1]), float(rows[1][2]),
        float(rows[2][0]), float(rows[2][1]), float(rows[2][2]),
    )


def mat_from_euler(roll: float, pitch: float, yaw: float) -> Mat3:
    """Body-to-world rotation for an intrinsic Z-Y-X (yaw, pitch, roll) sequence.

    This is the inverse of :func:`euler_from_mat`; together they round-trip
    anywhere outside the pitch = +/-90 deg gimbal singularity.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    )


def euler_from_mat(m: Mat3) -> Vec3:
    """(roll, pitch, yaw) in radians from a body-to-world rotation matrix."""
    pitch = math.asin(max(-1.0, min(1.0, -m[6])))
    if abs(m[6]) > 0.999999:
        # Gimbal lock: roll and yaw are degenerate, so pin roll and put the
        # whole rotation into yaw rather than returning a noisy split.
        return (0.0, pitch, math.atan2(-m[1], m[4]))
    return (math.atan2(m[7], m[8]), pitch, math.atan2(m[3], m[0]))


def quat_from_mat(m: Mat3) -> Tuple[float, float, float, float]:
    """(w, x, y, z) from a body-to-world rotation matrix, via Shepperd's method.

    Branching on the largest diagonal term keeps the square root away from zero,
    which a naive trace-only formula does not.
    """
    trace = m[0] + m[4] + m[8]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (0.25 * s, (m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s)
    if m[0] > m[4] and m[0] > m[8]:
        s = math.sqrt(1.0 + m[0] - m[4] - m[8]) * 2.0
        return ((m[7] - m[5]) / s, 0.25 * s, (m[1] + m[3]) / s, (m[2] + m[6]) / s)
    if m[4] > m[8]:
        s = math.sqrt(1.0 + m[4] - m[0] - m[8]) * 2.0
        return ((m[2] - m[6]) / s, (m[1] + m[3]) / s, 0.25 * s, (m[5] + m[7]) / s)
    s = math.sqrt(1.0 + m[8] - m[0] - m[4]) * 2.0
    return ((m[3] - m[1]) / s, (m[2] + m[6]) / s, (m[5] + m[7]) / s, 0.25 * s)


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else (high if value > high else value)
