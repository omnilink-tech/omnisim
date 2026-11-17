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

"""Prop-overlap validator.

Tier 1 scope: every placed URDFRobot / DEF Transform must be at least
``MIN_SEPARATION`` metres from every other placement. A real
bounding-volume check requires the asset catalog (T1.5); until then, a
conservative distance check catches the overlaps the scaffold can
reasonably produce.
"""

from __future__ import annotations

import math
from pathlib import Path

from ._wbt_scan import scan_placements

MIN_SEPARATION = 0.3  # metres


def check_prop_overlap(path: Path, text: str):
    from . import CheckResult

    del path
    placements = scan_placements(text)
    collisions: list[str] = []
    for i, a in enumerate(placements):
        for b in placements[i + 1 :]:
            ax, ay, az = a.translation
            bx, by, bz = b.translation
            d = math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)
            if d < MIN_SEPARATION:
                collisions.append(
                    f"{_label(a)} <-> {_label(b)} at {d:.3f} m "
                    f"(< {MIN_SEPARATION} m)"
                )
    if collisions:
        detail = "; ".join(collisions[:3])
        if len(collisions) > 3:
            detail += f" (+{len(collisions) - 3} more)"
        return CheckResult(name="prop_overlap", passed=False, detail=detail)
    return CheckResult(name="prop_overlap", passed=True)


def _label(p) -> str:
    parts = [p.node_type]
    if p.def_name:
        parts.append(f"DEF={p.def_name}")
    if p.name:
        parts.append(f"name={p.name!r}")
    return "[" + " ".join(parts) + "]"
