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
"""What the warehouse-shift film adds to the benchmark's world.

Visual-only floor markings for the shift's four stations and two keep-out
zones (no boundingObject, no physics: they cannot touch the run), and one
camera robot carrying two fixed cameras -- a 16:9 wide shot and a 9:16 tall
shot -- that save frames on simulator time (controller `film_camera`).

Coordinates are the shift's own (`shift_holdout_v2.json`, robot home at the
origin). `python scene.py` prints where each landmark lands in each image.
"""
from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

STATIONS = {"press line": (-4.2, 2.6), "tool crib": (1.0, 4.0), "QA bay": (-3.9, -3.8), "paint shop": (3.2, -3.6)}
ZONES = {"forklift lane": ((-3.0, -2.2), (2.0, -1.2)), "weld cell": ((-2.6, 1.2), (-1.2, 3.2))}

# The native renderer's exposure lifts flat colours a lot (a 0.96/0.82/0.26
# pad rendered pale cream), so these are deliberately dark and saturated.
MIMOSA = (0.90, 0.62, 0.02)
INK = (0.05, 0.05, 0.06)
# Keep-out zones are hazard-striped: (colour a, colour b) in bands across the long side.
ZONE_STRIPES = {"forklift lane": ((0.90, 0.60, 0.00), INK), "weld cell": ((0.70, 0.04, 0.03), (0.85, 0.85, 0.82))}
BAND_M = 0.25

# name: (position, look-at target, horizontal field of view, width, height)
CAMERAS: Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float], float, int, int]] = {
    # Chosen by a sweep (2026-10-20): every station pad, its label and both
    # zones inside the frame with margins; the wide shot's floor kept above the
    # bottom card (y 76..643 of 900), the tall shot's below the top card.
    "wide": ((-0.5, -13.0, 8.0), (-0.5, -2.5, 0.0), 0.90, 1600, 900),
    "tall": ((-0.5, -3.0, 11.0), (-0.5, 0.6, 0.0), 0.90, 900, 1600),
}


def _look_rotation(pos: Sequence[float], target: Sequence[float]) -> List[List[float]]:
    """Rotation matrix whose +x looks at target with +z up (camera convention: x forward)."""
    dx, dy, dz = (target[i] - pos[i] for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))     # positive = looking down
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    # Rz(yaw) @ Ry(pitch): Ry(+pitch) tips +x toward -z.
    return [[cy * cp, -sy, cy * sp],
            [sy * cp, cy, sy * sp],
            [-sp, 0.0, cp]]


def _axis_angle(R: List[List[float]]) -> Tuple[float, float, float, float]:
    angle = math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)))
    if abs(angle) < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)
    s = 2 * math.sin(angle)
    return ((R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, angle)


def project(camera: str, point: Sequence[float]) -> Tuple[float, float]:
    """World point -> pixel (u right, v down) in a camera's image."""
    pos, target, fov, w, h = CAMERAS[camera]
    R = _look_rotation(pos, target)
    d = [point[i] - pos[i] for i in range(3)]
    fwd = sum(R[i][0] * d[i] for i in range(3))
    left = sum(R[i][1] * d[i] for i in range(3))
    up = sum(R[i][2] * d[i] for i in range(3))
    f = (w / 2) / math.tan(fov / 2)
    return (w / 2 - f * left / fwd, h / 2 - f * up / fwd)


def _pad(name: str, x: float, y: float, sx: float, sy: float, rgb, z: float = 0.004) -> str:
    r, g, b = rgb
    return (f'Solid {{ name "film_{name}" translation {x:.3f} {y:.3f} {z:.3f} children [ Shape {{ '
            f'appearance PBRAppearance {{ baseColor {r:.3f} {g:.3f} {b:.3f} roughness 1 metalness 0 }} '
            f'geometry Box {{ size {sx:.3f} {sy:.3f} 0.004 }} }} ] }}\n')


def _striped(name: str, x0: float, y0: float, x1: float, y1: float, colours) -> List[str]:
    """Hazard bands across the zone's long side, alternating two colours."""
    along_x = (x1 - x0) >= (y1 - y0)
    length = (x1 - x0) if along_x else (y1 - y0)
    n = max(2, round(length / BAND_M))
    band = length / n
    out = []
    for k in range(n):
        c = colours[k % 2]
        if along_x:
            out.append(_pad(f"{name}_{k}", x0 + band * (k + 0.5), (y0 + y1) / 2, band, y1 - y0, c))
        else:
            out.append(_pad(f"{name}_{k}", (x0 + x1) / 2, y0 + band * (k + 0.5), x1 - x0, band, c))
    return out


def vrml(period_ms: int = 125) -> str:
    out = ["\n# ---- warehouse-shift film: visual-only markings and cameras ----\n"]
    for i, (name, (x, y)) in enumerate(STATIONS.items()):
        # Layers 4 mm apart with no overlap: overlapping boxes z-fight at this distance.
        out.append(_pad(f"station_{i}", x, y, 1.0, 1.0, INK, z=0.003))
        out.append(_pad(f"station_{i}_core", x, y, 0.7, 0.7, MIMOSA, z=0.009))
    for i, (name, ((x0, y0), (x1, y1))) in enumerate(ZONES.items()):
        out += _striped(f"zone_{i}", x0, y0, x1, y1, ZONE_STRIPES[name])
    cams = []
    for name, (pos, target, fov, w, h) in CAMERAS.items():
        ax, ay, az, angle = _axis_angle(_look_rotation(pos, target))
        cams.append(f'Camera {{ name "{name}" translation {pos[0]} {pos[1]} {pos[2]} '
                    f'rotation {ax:.6f} {ay:.6f} {az:.6f} {angle:.6f} fieldOfView {fov} '
                    f'width {w} height {h} far 60 }}')
    out.append(f'Robot {{ name "film_camera" controller "film_camera" controllerArgs [ "{period_ms}" ] '
               f'children [ {" ".join(cams)} ] }}\n')
    return "".join(out)


if __name__ == "__main__":
    points = {**{f"station {k}": (*v, 0.0) for k, v in STATIONS.items()},
              **{f"{k} {c}": (x, y, 0.0) for k, ((x0, y0), (x1, y1)) in ZONES.items()
                 for c, (x, y) in (("min", (x0, y0)), ("max", (x1, y1)))},
              "robot home": (0.0, 0.0, 0.0)}
    for cam, (_, _, _, w, h) in CAMERAS.items():
        print(f"== {cam} ({w}x{h})")
        for name, p in points.items():
            u, v = project(cam, p)
            ok = 0.04 * w <= u <= 0.96 * w and 0.04 * h <= v <= 0.96 * h
            print(f"   {name:22} u={u:7.0f} v={v:7.0f} {'' if ok else '  <-- near/outside edge'}")
