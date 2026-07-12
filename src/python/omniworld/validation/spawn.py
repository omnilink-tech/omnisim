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

"""Spawn-reachability validator.

Tier 1 scope: every URDFRobot placement must have a finite translation
above the floor. A true reachability check (0.5 m navigable radius,
terrain lookup, prop clearance) needs the asset catalog plus the
heightmap primitives and lands with T1.7.
"""

from __future__ import annotations

import math
from pathlib import Path

from ._wbt_scan import scan_placements


def check_spawn(path: Path, text: str):
    from . import CheckResult

    del path
    issues: list[str] = []
    robots = [p for p in scan_placements(text) if p.node_type == "URDFRobot"]
    for p in robots:
        x, y, z = p.translation
        label = p.name or p.def_name or "<unnamed>"
        if not all(math.isfinite(c) for c in (x, y, z)):
            issues.append(f"{label!r}: non-finite translation ({x}, {y}, {z})")
            continue
        if z < 0.0:
            issues.append(f"{label!r}: spawn below floor at z={z:.3f}")
    # A world with no URDFRobot is valid — not every world has a robot
    # (the stub defaults to no spawn).
    if issues:
        return CheckResult(
            name="spawn_reachability",
            passed=False,
            detail="; ".join(issues),
        )
    return CheckResult(name="spawn_reachability", passed=True)
