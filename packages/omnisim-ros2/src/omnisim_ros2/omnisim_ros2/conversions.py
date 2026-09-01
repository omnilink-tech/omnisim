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

"""Geometry conversions between OmniSim's harness JSON and ROS 2 messages.

THE ROTATION TRAP
-----------------
OmniSim uses **two different rotation encodings** under confusingly similar key
names, and getting this backwards silently produces plausible-but-wrong poses:

===========================================  ===========  ==================================
Where                                        Key          Encoding
===========================================  ===========  ==================================
``/scene/tree`` node, ``/robots`` entry,     ``orientation``  9 floats, row-major 3x3 matrix
``/scene/node/<def>`` top level
``/scene/node/<def>`` -> ``fields``          ``rotation``     4 floats, axis-angle [x,y,z,rad]
``/scene/set_pose``, ``/scene/spawn``        ``rotation``     4 floats, axis-angle [x,y,z,rad]
``/scene/viewpoint``, ``/scene/look_at``     ``orientation``  4 floats, axis-angle
===========================================  ===========  ==================================

So *reads* of scene/robot nodes give a matrix, while every *write* takes
axis-angle. Both directions are implemented here and the tests pin the
round-trip.

COORDINATE FRAME
----------------
OmniSim worlds are Z-up / ENU (``WorldInfo.coordinateSystem`` defaults to
``"ENU"``), which is exactly REP-103. **No axis remapping is applied or needed.**

The one caveat, disclosed rather than papered over: ``coordinateSystem`` is not
queryable over the harness HTTP surface, so a world that overrides it to ``NUE``
cannot be detected at runtime and its poses would be reported in the engine's
frame without correction. See the README's "Known gaps".
"""

from __future__ import annotations

import math
from typing import Any, Sequence

# Rotation matrices coming back from the engine are float32-ish; anything inside
# this tolerance of the degenerate cases is treated as that case.
_EPS = 1e-9


def matrix_to_quaternion(m: Sequence[float]) -> tuple[float, float, float, float]:
    """Convert a row-major flattened 3x3 rotation matrix to ``(x, y, z, w)``.

    Uses Shepperd's method: pick the branch whose divisor is largest, which
    avoids the catastrophic cancellation the naive trace formula suffers near
    180-degree rotations.
    """
    if len(m) != 9:
        raise ValueError(f"expected 9 matrix elements, got {len(m)}")
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = (float(v) for v in m)
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < _EPS:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def axis_angle_to_quaternion(r: Sequence[float]) -> tuple[float, float, float, float]:
    """Convert OmniSim's SFRotation ``[ax, ay, az, angle_rad]`` to ``(x, y, z, w)``."""
    if len(r) != 4:
        raise ValueError(f"expected 4 axis-angle elements, got {len(r)}")
    ax, ay, az, angle = (float(v) for v in r)
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < _EPS:
        # A zero axis is the engine's identity rotation regardless of angle.
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = ax / norm, ay / norm, az / norm
    half = angle * 0.5
    s = math.sin(half)
    return (ax * s, ay * s, az * s, math.cos(half))


def quaternion_to_axis_angle(q: Sequence[float]) -> list[float]:
    """Convert ``(x, y, z, w)`` to OmniSim's SFRotation ``[ax, ay, az, angle_rad]``.

    A zero-length quaternion is treated as identity rather than raising, because
    an unset ``geometry_msgs/Quaternion`` is all zeros and callers routinely send
    one when they mean "do not rotate".
    """
    if len(q) != 4:
        raise ValueError(f"expected 4 quaternion elements, got {len(q)}")
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < _EPS:
        return [0.0, 0.0, 1.0, 0.0]
    x, y, z, w = x / n, y / n, z / n, w / n
    # Canonicalise to the w >= 0 hemisphere so the returned angle is in [0, pi].
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    sin_half = math.sqrt(x * x + y * y + z * z)
    if sin_half < _EPS:
        # Identity: any axis works, but the engine expects a unit one.
        return [0.0, 0.0, 1.0, 0.0]
    angle = 2.0 * math.atan2(sin_half, w)
    return [x / sin_half, y / sin_half, z / sin_half, angle]


def is_valid_quaternion(q: Sequence[float]) -> bool:
    """True when ``q`` is usable as a rotation (finite and not zero-length)."""
    if len(q) != 4:
        return False
    try:
        vals = [float(v) for v in q]
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(v) for v in vals):
        return False
    return math.sqrt(sum(v * v for v in vals)) >= _EPS


def orientation_to_quaternion(value: Any) -> tuple[float, float, float, float]:
    """Convert whichever orientation encoding the harness returned.

    Accepts the 9-element matrix (scene/robot reads), the 4-element axis-angle
    (viewpoint reads and all writes), or ``None``/``[null, ...]`` for a node that
    has no pose, which becomes identity.
    """
    if not value:
        return (0.0, 0.0, 0.0, 1.0)
    seq = list(value)
    if any(v is None for v in seq):
        return (0.0, 0.0, 0.0, 1.0)
    if len(seq) == 9:
        return matrix_to_quaternion(seq)
    if len(seq) == 4:
        return axis_angle_to_quaternion(seq)
    raise ValueError(f"unrecognised orientation with {len(seq)} elements")


def position_to_xyz(value: Any) -> tuple[float, float, float]:
    """Convert a harness ``position``/``translation`` triple to floats.

    ``[null, null, null]`` (a node with no pose, e.g. a Group) becomes the
    origin, matching how the harness itself reports such nodes.
    """
    if not value:
        return (0.0, 0.0, 0.0)
    seq = list(value)
    if len(seq) != 3 or any(v is None for v in seq):
        return (0.0, 0.0, 0.0)
    return (float(seq[0]), float(seq[1]), float(seq[2]))


def sim_time_ms_to_ros(sim_time_ms: float) -> tuple[int, int]:
    """Split a harness ``sim_time_ms`` into ROS ``(sec, nanosec)``.

    ``nanosec`` is clamped below 1e9 so a rounding artefact at the boundary can
    never produce the invalid ``(s, 1000000000)`` that ROS time rejects.
    """
    total_ns = int(round(float(sim_time_ms) * 1_000_000))
    if total_ns < 0:
        total_ns = 0
    sec, nanosec = divmod(total_ns, 1_000_000_000)
    return int(sec), int(nanosec)


def relative_transform(
    parent_position: Sequence[float],
    parent_matrix: Sequence[float],
    child_position: Sequence[float],
    child_matrix: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Express a child's world pose relative to its parent's world pose.

    The harness reports every node's pose in **world** coordinates, but TF wants
    each transform relative to its parent frame. Given the two world poses::

        R_rel = R_parent^T . R_child
        t_rel = R_parent^T . (t_child - t_parent)

    A rotation matrix is orthonormal, so the transpose *is* the inverse and no
    general matrix inversion is needed.

    Returns ``((x, y, z), (qx, qy, qz, qw))``.
    """
    px, py, pz = position_to_xyz(parent_position)
    cx, cy, cz = position_to_xyz(child_position)
    rp = _matrix_or_identity(parent_matrix)
    rc = _matrix_or_identity(child_matrix)

    dx, dy, dz = cx - px, cy - py, cz - pz
    # R_parent^T . d -- reading rp column-wise gives the transpose's rows.
    tx = rp[0] * dx + rp[3] * dy + rp[6] * dz
    ty = rp[1] * dx + rp[4] * dy + rp[7] * dz
    tz = rp[2] * dx + rp[5] * dy + rp[8] * dz

    # R_parent^T . R_child
    rel = [0.0] * 9
    for i in range(3):
        for j in range(3):
            rel[i * 3 + j] = sum(rp[k * 3 + i] * rc[k * 3 + j] for k in range(3))
    return (tx, ty, tz), matrix_to_quaternion(rel)


def _matrix_or_identity(m: Any) -> list[float]:
    """Coerce a harness ``orientation`` to a usable 3x3, defaulting to identity.

    Accepts the 9-element matrix directly, converts a 4-element axis-angle, and
    falls back to identity for a pose-less node's ``[null] * 9``.
    """
    if not m:
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    seq = list(m)
    if len(seq) == 9 and all(v is not None for v in seq):
        return [float(v) for v in seq]
    if len(seq) == 4 and all(v is not None for v in seq):
        return quaternion_to_matrix(axis_angle_to_quaternion(seq))
    return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def quaternion_to_matrix(q: Sequence[float]) -> list[float]:
    """Convert ``(x, y, z, w)`` to a row-major flattened 3x3 rotation matrix."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < _EPS:
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
    ]


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Rotation about +Z (up, under ENU/REP-103) as ``(x, y, z, w)``."""
    half = float(yaw) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def select_lidar_layer(values: Sequence[Any], layers: int, per: int) -> int:
    """Pick the lidar layer with the most finite returns.

    OmniSim's ``Lidar`` defaults to **4** layers and the URDF importer only
    writes ``numberOfLayers`` when the URDF asks for more than one, so a
    URDF-declared *planar* scanner still arrives as a 4-layer device. Whole
    layers can then be empty: measured on the shipped Husky demo world, 3 of 4
    layers return nothing and only one sees the floor. Picking the populated
    layer makes ``/scan`` useful; the caller logs which one it got.
    """
    best, best_n = 0, -1
    for layer in range(max(layers, 1)):
        chunk = values[layer * per:(layer + 1) * per]
        n = sum(1 for v in chunk if v is not None)
        if n > best_n:
            best, best_n = layer, n
    return best


def lidar_layer_ranges(
    values: Sequence[Any], layers: int, per: int, layer: int, reverse: bool = True
) -> list[float]:
    """Extract one layer of a lidar range image as ROS ``LaserScan.ranges``.

    ``None`` becomes ``inf``. That mapping is the whole point: a ray that hits
    nothing reads ``+inf``, which is not valid JSON, so the bridge sends
    ``null``. ``inf`` is what ROS consumers already treat as out-of-range,
    whereas ``0.0`` reads as an obstacle touching the sensor.

    ``reverse`` accounts for Webots handing back a scan leftmost-first while
    ROS starts at ``angle_min`` and increases.
    """
    layer = max(0, min(int(layer), max(layers, 1) - 1))
    chunk = list(values[layer * per:(layer + 1) * per])
    out = [float("inf") if v is None else float(v) for v in chunk]
    if reverse:
        out.reverse()
    return out


def finite_triplet(value: Any) -> list[float] | None:
    """Coerce a 3-axis sensor reading to three finite floats, or ``None``.

    The gate for publishing ``Imu.angular_velocity`` / ``linear_acceleration``
    as *measured* (covariance[0] = 0.0) rather than *absent* (-1). Defensive on
    purpose: the bridge's JSON sanitizer maps every non-finite float to
    ``null``, so a device that is enabled but not yet sampling — or a stale
    pre-2026-09-01 engine whose Accelerometer still yields NaN — arrives here
    as ``None`` or as a vector containing nulls. Anything short of exactly
    three finite numbers is "not measured", and is never zeroed.
    """
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        out = [float(v) for v in value]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in out):
        return None
    return out


def sanitize_frame_id(name: str) -> str:
    """Make a DEF name safe to use as a TF frame id.

    TF forbids a leading '/' and treats whitespace poorly. OmniSim DEF names for
    anonymous nodes are of the form ``#123``, which is legal in TF but confusing,
    so it becomes ``node_123``.
    """
    frame = str(name).strip().lstrip("/")
    if frame.startswith("#"):
        frame = "node_" + frame[1:]
    return frame.replace(" ", "_") or "unnamed"
