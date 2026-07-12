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

"""Parse omnisim_log.txt's "step N b0=(...)..." lines and compute the
distance between each child body and the joint anchor on its parent.
A joint that's holding gives a constant distance. A diverging distance
means the constraint is violated and the body is being "yanked off."

Anchors (from the Webots log on this run):
  chassis->hip_x:  parent=body 0, child=body 1 (and 4, 7, 10)
                   anchor in chassis frame: (+-0.29785, +-0.055, 0)
  hip->upper:      parent=body 1 (/4/7/10), child=body 2 (/5/8/11)
                   anchor in hip frame: (0, +-0.110945, 0)
  upper->lower:    parent=body 2 (/5/8/11), child=body 3 (/6/9/12)
                   anchor in upper frame: (0.025, 0, -0.3205)
"""
from __future__ import annotations
import re
import sys
from pathlib import Path

LOG = Path("omnisim_log.txt")

# (parent, child, anchor_in_parent_frame)
JOINTS = [
    (0, 1, (+0.29785, +0.055, 0.0)),
    (0, 4, (+0.29785, -0.055, 0.0)),
    (0, 7, (-0.29785, +0.055, 0.0)),
    (0, 10, (-0.29785, -0.055, 0.0)),
    (1, 2, (0.0, +0.110945, 0.0)),
    (4, 5, (0.0, -0.110945, 0.0)),
    (7, 8, (0.0, +0.110945, 0.0)),
    (10, 11, (0.0, -0.110945, 0.0)),
    (2, 3, (0.025, 0.0, -0.3205)),
    (5, 6, (0.025, 0.0, -0.3205)),
    (8, 9, (0.025, 0.0, -0.3205)),
    (11, 12, (0.025, 0.0, -0.3205)),
]

# Static joint anchors give just a *minimum* distance from the parent to the
# child body origin (when the child's local-frame anchor is the origin).
# Since we don't have rotation info per body parsed here, use the simpler
# euclidean distance ignoring rotation:
#   |child_world_origin - parent_world_origin| should change when the joint
# rotates, but the distance from parent_anchor_world to child_world_origin
# should stay constant (== |child-frame anchor| which we don't have).
# Easier proxy: track parent-to-child Euclidean distance. It varies with
# joint angle, but for small joint motion around nominal it shouldn't blow
# up. If it does, the constraint is broken.


def parse_step_line(line: str):
    """Return (step, {body_idx: (x,y,z)}) or None."""
    m = re.match(r"INFO: \[WbNewtonBackend\] step (\d+) .*", line)
    if not m:
        return None
    step = int(m.group(1))
    bodies = {}
    for bm in re.finditer(r"b(\d+)=\(([-\d.]+),([-\d.]+),([-\d.]+)\)", line):
        idx = int(bm.group(1))
        x, y, z = float(bm.group(2)), float(bm.group(3)), float(bm.group(4))
        bodies[idx] = (x, y, z)
    return (step, bodies)


def main() -> int:
    text = LOG.read_text(errors="replace")
    # Each "INFO: [WbNewtonBackend] step ..." can span multiple physical
    # lines due to Webots' Qt log line-wrapping. Re-flow.
    lines = re.split(r"(?=INFO: \[WbNewtonBackend\] step )", text)
    steps = []
    for chunk in lines:
        if not chunk.startswith("INFO: [WbNewtonBackend] step "):
            continue
        # Join wrapped lines
        flat = re.sub(r"\s+", " ", chunk)
        r = parse_step_line(flat)
        if r is not None:
            steps.append(r)

    if not steps:
        print("no step lines parsed")
        return 1

    print(f"parsed {len(steps)} step snapshots")
    print()
    print(f"{'step':>6} | parent-child distances (m), ratio vs step 1")
    print("-" * 110)
    # baseline = first snapshot
    base_step, base_bodies = steps[0]
    base_dists = {}
    for (p, c, _a) in JOINTS:
        if p in base_bodies and c in base_bodies:
            bx, by, bz = base_bodies[p]
            cx, cy, cz = base_bodies[c]
            base_dists[(p, c)] = ((cx-bx)**2 + (cy-by)**2 + (cz-bz)**2) ** 0.5

    for (step, bodies) in steps:
        row = f"{step:>6} |"
        for (p, c, _a) in JOINTS:
            if p in bodies and c in bodies:
                bx, by, bz = bodies[p]
                cx, cy, cz = bodies[c]
                d = ((cx-bx)**2 + (cy-by)**2 + (cz-bz)**2) ** 0.5
                base = base_dists.get((p, c), 0.0)
                ratio = d / base if base > 1e-6 else 0.0
                marker = "**" if ratio > 1.5 or ratio < 0.67 else "  "
                row += f" {p}-{c}:{d:.3f}{marker}"
        print(row)
    print()
    print("** marks joints where the parent-child distance has changed by >50%")
    print("(joint constraint likely violated)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
