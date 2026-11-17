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

"""Camera-path math for the OmniSim capture service.

Pure functions. No OmniSim, no IPC. Used both by the long-running capture
service (to drive `set_camera_pose` per frame during `/capture/sequence`)
and by render.py's offline shot-list executor.

A `Keyframe` is a dict with keys `t` (seconds, float, monotonically
increasing), `position` ([x,y,z]), and either `orientation` ([ax,ay,az,
angle]) or `target` ([tx,ty,tz]). Targets are converted to orientations
via the same axis-angle math as the harness's compute_look_at_orientation.

Interpolation between keyframes uses:
  - Catmull-Rom spline for positions (C1-continuous, passes through every
    keyframe; the first/last segments use the endpoint as a virtual
    control point so the curve doesn't shoot off into space).
  - Slerp for orientations, applied to the unit quaternion equivalent of
    each axis-angle, then converted back to axis-angle. This avoids the
    gimbal-flip artifacts you get from naive component-wise lerp on
    axis-angle 4-tuples.
  - An optional per-segment easing function applied to the segment-local
    parameter u in [0, 1]. Default is `smoothstep`. Available eases:
    "linear", "smoothstep", "smootherstep", "ease_in", "ease_out",
    "ease_in_out".
"""

from __future__ import annotations

import math
from typing import Callable


Vec3 = list[float]
AxisAngle = list[float]
Quat = tuple[float, float, float, float]


# ---------------------------------------------------------------------------
# Easing
# ---------------------------------------------------------------------------


def _ease_linear(u: float) -> float:
    return u


def _ease_smoothstep(u: float) -> float:
    return u * u * (3.0 - 2.0 * u)


def _ease_smootherstep(u: float) -> float:
    return u * u * u * (u * (u * 6.0 - 15.0) + 10.0)


def _ease_in(u: float) -> float:
    return u * u


def _ease_out(u: float) -> float:
    return 1.0 - (1.0 - u) ** 2


def _ease_in_out(u: float) -> float:
    if u < 0.5:
        return 2.0 * u * u
    return 1.0 - 2.0 * (1.0 - u) ** 2


EASE: dict[str, Callable[[float], float]] = {
    "linear": _ease_linear,
    "smoothstep": _ease_smoothstep,
    "smootherstep": _ease_smootherstep,
    "ease_in": _ease_in,
    "ease_out": _ease_out,
    "ease_in_out": _ease_in_out,
}


def get_ease(name: str | None) -> Callable[[float], float]:
    if name is None:
        return _ease_smoothstep
    fn = EASE.get(name)
    if fn is None:
        raise ValueError(f"unknown ease: {name!r}; available: {sorted(EASE)}")
    return fn


# ---------------------------------------------------------------------------
# Look-at math (duplicated from harness — keeps this module dependency-free)
# ---------------------------------------------------------------------------


def look_at_orientation(
    position: Vec3,
    target: Vec3,
    up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> AxisAngle:
    """Aim the capture camera at ``target`` while keeping its horizon level.

    OmniSim's camera frame is +X forward, +Y left, +Z up.  A shortest-arc
    rotation from +X aims at the subject, but leaves roll unconstrained; that
    produced visibly tilted marketing captures.  Build the full camera basis
    from an explicit world-up vector instead.  Keep this numerically identical
    to ``omniworld.viewpoint.look_at`` and the harness copy.
    """
    px, py, pz = position
    tx, ty, tz = target
    fx, fy, fz = tx - px, ty - py, tz - pz
    n = math.sqrt(fx * fx + fy * fy + fz * fz)
    if n < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    fx, fy, fz = fx / n, fy / n, fz / n

    ux, uy, uz = up
    un = math.sqrt(ux * ux + uy * uy + uz * uz)
    if un < 1e-12:
        ux, uy, uz = 0.0, 0.0, 1.0
        un = 1.0
    ux, uy, uz = ux / un, uy / un, uz / un
    if 1.0 - abs(fx * ux + fy * uy + fz * uz) < 1e-6:
        ux, uy, uz = 0.0, 1.0, 0.0
        if 1.0 - abs(fx * ux + fy * uy + fz * uz) < 1e-6:
            ux, uy, uz = 1.0, 0.0, 0.0

    dot = fx * ux + fy * uy + fz * uz
    ux, uy, uz = ux - dot * fx, uy - dot * fy, uz - dot * fz
    un = math.sqrt(ux * ux + uy * uy + uz * uz)
    if un < 1e-12:
        return [0.0, 0.0, 1.0, 0.0]
    ux, uy, uz = ux / un, uy / un, uz / un
    yx = uy * fz - uz * fy
    yy = uz * fx - ux * fz
    yz = ux * fy - uy * fx

    rotation = [
        [fx, yx, ux],
        [fy, yy, uy],
        [fz, yz, uz],
    ]
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    if angle < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]

    scale = 2.0 * math.sin(angle)
    if abs(scale) < 1e-9:
        diagonal = (rotation[0][0], rotation[1][1], rotation[2][2])
        index = diagonal.index(max(diagonal))
        axis = [0.0, 0.0, 0.0]
        axis[index] = math.sqrt(max(0.0, (rotation[index][index] + 1.0) / 2.0))
        for j in range(3):
            if j != index:
                axis[j] = (
                    (rotation[index][j] + rotation[j][index]) / (4.0 * axis[index])
                    if axis[index] > 1e-9
                    else 0.0
                )
        axis_norm = math.sqrt(sum(component * component for component in axis)) or 1.0
        return [component / axis_norm for component in axis] + [angle]

    return [
        (rotation[2][1] - rotation[1][2]) / scale,
        (rotation[0][2] - rotation[2][0]) / scale,
        (rotation[1][0] - rotation[0][1]) / scale,
        angle,
    ]


# ---------------------------------------------------------------------------
# Quaternion <-> axis-angle, slerp
# ---------------------------------------------------------------------------


def axis_angle_to_quat(aa: AxisAngle) -> Quat:
    ax, ay, az, angle = aa
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    ax, ay, az = ax / n, ay / n, az / n
    half = angle * 0.5
    s = math.sin(half)
    return (math.cos(half), ax * s, ay * s, az * s)


def quat_to_axis_angle(q: Quat) -> AxisAngle:
    w, x, y, z = q
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-12:
        return [0.0, 0.0, 1.0, 0.0]
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    if w > 1.0:
        w = 1.0
    elif w < -1.0:
        w = -1.0
    angle = 2.0 * math.acos(w)
    s = math.sqrt(1.0 - w * w)
    if s < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    return [x / s, y / s, z / s, angle]


def slerp_quat(a: Quat, b: Quat, u: float) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    dot = aw * bw + ax * bx + ay * by + az * bz
    # Take the shorter arc.
    if dot < 0.0:
        bw, bx, by, bz = -bw, -bx, -by, -bz
        dot = -dot
    if dot > 0.9995:
        # Endpoints too close — linear blend + renormalize is more numerically
        # stable than acos near 0.
        rw = aw + u * (bw - aw)
        rx = ax + u * (bx - ax)
        ry = ay + u * (by - ay)
        rz = az + u * (bz - az)
        n = math.sqrt(rw * rw + rx * rx + ry * ry + rz * rz)
        return (rw / n, rx / n, ry / n, rz / n) if n > 1e-12 else (1.0, 0.0, 0.0, 0.0)
    theta_0 = math.acos(dot)
    theta = theta_0 * u
    sin_theta_0 = math.sin(theta_0)
    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    return (s0 * aw + s1 * bw, s0 * ax + s1 * bx, s0 * ay + s1 * by, s0 * az + s1 * bz)


# ---------------------------------------------------------------------------
# Catmull-Rom for positions
# ---------------------------------------------------------------------------


def catmull_rom(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, u: float) -> Vec3:
    """Standard centripetal-friendly form (uniform parameterization). The
    curve passes through p1 at u=0 and p2 at u=1; p0 and p3 are tangent
    references.
    """
    u2 = u * u
    u3 = u2 * u
    out: Vec3 = [0.0, 0.0, 0.0]
    for i in range(3):
        out[i] = 0.5 * (
            (2.0 * p1[i])
            + (-p0[i] + p2[i]) * u
            + (2.0 * p0[i] - 5.0 * p1[i] + 4.0 * p2[i] - p3[i]) * u2
            + (-p0[i] + 3.0 * p1[i] - 3.0 * p2[i] + p3[i]) * u3
        )
    return out


# ---------------------------------------------------------------------------
# Path normalization + sampling
# ---------------------------------------------------------------------------


def normalize_keyframes(keyframes: list[dict]) -> list[dict]:
    """Validate, fill in orientations from `target`, sort by `t`. Returns a
    new list of dicts with keys {t, position, orientation}.
    """
    if not isinstance(keyframes, list) or len(keyframes) < 1:
        raise ValueError("keyframes must be a non-empty list")
    out: list[dict] = []
    for i, k in enumerate(keyframes):
        if not isinstance(k, dict):
            raise ValueError(f"keyframe {i} is not a dict")
        if "t" not in k:
            raise ValueError(f"keyframe {i} missing 't'")
        try:
            t = float(k["t"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"keyframe {i} 't' must be a number: {exc}") from exc
        position = k.get("position")
        if not isinstance(position, list) or len(position) != 3:
            raise ValueError(f"keyframe {i} 'position' must be a 3-list")
        position = [float(x) for x in position]
        orientation = k.get("orientation")
        target = k.get("target")
        if orientation is None and target is None:
            raise ValueError(f"keyframe {i} needs 'orientation' or 'target'")
        if orientation is None:
            if not isinstance(target, list) or len(target) != 3:
                raise ValueError(f"keyframe {i} 'target' must be a 3-list")
            orientation = look_at_orientation(position, [float(x) for x in target])
        else:
            if not isinstance(orientation, list) or len(orientation) != 4:
                raise ValueError(f"keyframe {i} 'orientation' must be a 4-list")
            orientation = [float(x) for x in orientation]
        out.append({"t": t, "position": position, "orientation": orientation})
    out.sort(key=lambda k: k["t"])
    return out


def sample_path(
    keyframes: list[dict],
    t: float,
    ease: str | None = "smoothstep",
) -> tuple[Vec3, AxisAngle]:
    """Return (position, orientation) at time `t` along the path defined by
    `keyframes`. Clamps to endpoints outside [t0, tN].
    """
    if not keyframes:
        raise ValueError("empty keyframes")
    if len(keyframes) == 1:
        k = keyframes[0]
        return list(k["position"]), list(k["orientation"])

    if t <= keyframes[0]["t"]:
        k = keyframes[0]
        return list(k["position"]), list(k["orientation"])
    if t >= keyframes[-1]["t"]:
        k = keyframes[-1]
        return list(k["position"]), list(k["orientation"])

    # Find the segment [k_i, k_{i+1}] containing t.
    i = 0
    for j in range(len(keyframes) - 1):
        if keyframes[j]["t"] <= t <= keyframes[j + 1]["t"]:
            i = j
            break
    k0 = keyframes[i]
    k1 = keyframes[i + 1]
    span = max(1e-12, k1["t"] - k0["t"])
    u_raw = (t - k0["t"]) / span
    eased = get_ease(ease)(max(0.0, min(1.0, u_raw)))

    # Catmull-Rom needs neighbors on either side; clamp to endpoints when the
    # segment is at the boundary (the segment endpoints stay fixed).
    p0 = keyframes[max(0, i - 1)]["position"]
    p1 = k0["position"]
    p2 = k1["position"]
    p3 = keyframes[min(len(keyframes) - 1, i + 2)]["position"]
    pos = catmull_rom(p0, p1, p2, p3, eased)

    q0 = axis_angle_to_quat(k0["orientation"])
    q1 = axis_angle_to_quat(k1["orientation"])
    qi = slerp_quat(q0, q1, eased)
    ori = quat_to_axis_angle(qi)
    return pos, ori


def sample_path_uniform(
    keyframes: list[dict],
    duration_s: float,
    fps: int,
    ease: str | None = "smoothstep",
) -> list[tuple[Vec3, AxisAngle]]:
    """Sample (position, orientation) at every frame of a fixed-fps render.
    Used by render.py for offline frame-sequence mode.
    """
    if duration_s <= 0:
        raise ValueError("duration_s must be > 0")
    if fps <= 0:
        raise ValueError("fps must be > 0")
    n_frames = max(1, int(round(duration_s * fps)))
    out: list[tuple[Vec3, AxisAngle]] = []
    for i in range(n_frames):
        # Map frame index to absolute path time. If the keyframes' t-range is
        # (t0, tN), we stretch the duration to span it.
        t0 = keyframes[0]["t"]
        tN = keyframes[-1]["t"]
        u = i / max(1, n_frames - 1)
        t = t0 + u * (tN - t0)
        out.append(sample_path(keyframes, t, ease=ease))
    return out
