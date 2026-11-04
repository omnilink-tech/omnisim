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

"""The pieces every manufacturing world generator writes: numbers, boxes, cylinders, a camera.

Shared by cnc/world.py, printing/world.py and the product worlds, so a machine's
world reads the same whatever process it runs.
"""
from __future__ import annotations

import math
from typing import Sequence


def num(v: float) -> str:
    """A length in metres to 0.1 um. At 10 um the CNC twin's cutter tip landed 4 um low
    and every pocket floor was 4 um too deep (cnc/inspect_part.py, 2026-11-01)."""
    return f"{v:.7f}".rstrip("0").rstrip(".") if v else "0"


def box(t, size, color, metal=0.1, rough=0.5) -> str:
    return (f"Pose {{ translation {num(t[0])} {num(t[1])} {num(t[2])} children [ Shape {{ appearance PBRAppearance "
            f"{{ baseColor {color} metalness {metal} roughness {rough} }} geometry Box {{ size {num(size[0])} "
            f"{num(size[1])} {num(size[2])} }} }} ] }}")


def cyl(t, radius, height, color, metal=0.6) -> str:
    return (f"Pose {{ translation {num(t[0])} {num(t[1])} {num(t[2])} children [ Shape {{ appearance PBRAppearance "
            f"{{ baseColor {color} metalness {metal} roughness .35 }} geometry Cylinder {{ radius {num(radius)} "
            f"height {num(height)} }} }} ] }}")


def look_at(position: Sequence[float], target: Sequence[float]) -> str:
    """Viewpoint orientation (axis-angle) looking from position to target; +X forward, +Z up."""
    d = [target[i] - position[i] for i in range(3)]
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    r = [[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0.0, cp]]   # Rz(yaw) @ Ry(pitch)
    angle = math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2)))
    if angle < 1e-9:
        return "0 0 1 0"
    k = 2 * math.sin(angle)
    axis = [(r[2][1] - r[1][2]) / k, (r[0][2] - r[2][0]) / k, (r[1][0] - r[0][1]) / k]
    return " ".join(num(v) for v in axis) + f" {num(angle)}"
