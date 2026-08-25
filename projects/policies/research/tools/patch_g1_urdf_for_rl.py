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

"""Patch the Unitree G1 URDF for OmniSim/Newton RL.

Issue: Newton's URDF importer picks up only the FIRST <collision> shape
per <link>. G1's feet (ankle_roll_link) are defined as 4 corner spheres
forming a 17×6 cm support polygon, but the importer takes just the
first one → 5 mm point contact → impossible to balance on.

Fix: replace the 4 corner spheres with a single bounding box that
approximates the real support polygon.

  x range: [-0.05, +0.12]  → center 0.035, half-extent 0.085
  y range: [-0.03, +0.03]  → center 0, half-extent 0.030
  z position: -0.03 (constant)
  thickness: small (~5 mm) so the box is essentially a flat plate

Writes to <urdf>_omnisim.urdf next to the source. Mesh paths are
unchanged (point at the same meshes/ directory).
"""
from __future__ import annotations

import re
import sys
from pathlib import Path


REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
DEFAULT_SRC = REPO / "projects" / "robots" / "unitree" / "g1" / "urdf" / "g1_23dof.urdf"


def patch(src: Path, dest: Path) -> None:
    text = src.read_text()

    # Foot box replacement template — replaces the entire <collision>
    # ... </collision> ... </link> block for each ankle_roll_link.
    foot_box = (
        '    <collision>\n'
        '      <origin xyz="0.035 0 -0.030" rpy="0 0 0"/>\n'
        '      <geometry>\n'
        '        <box size="0.17 0.06 0.012"/>\n'
        '      </geometry>\n'
        '    </collision>\n'
    )

    new_text = text
    for side in ("left", "right"):
        link_name = f"{side}_ankle_roll_link"
        # Match the link block (lazy), then within it replace all
        # consecutive <collision>...</collision> blocks with the
        # single foot_box.
        link_pattern = re.compile(
            r'(<link name="' + link_name + r'">.*?)'
            r'(    <collision>.*?</collision>\s*)+'
            r'(  </link>)', re.DOTALL,
        )

        def sub(m):
            return m.group(1) + foot_box + "  " + m.group(3)

        new_text, n = link_pattern.subn(sub, new_text)
        if n == 0:
            print(f"WARN: no collisions to patch in {link_name}")
        else:
            print(f"patched {link_name}: replaced corner spheres with foot box")

    # Sanity check: only one collision should remain under each ankle_roll.
    for side in ("left", "right"):
        link_name = f"{side}_ankle_roll_link"
        m = re.search(
            r'<link name="' + link_name + r'">(.*?)</link>',
            new_text, re.DOTALL)
        n_coll = m.group(1).count("<collision>") if m else 0
        print(f"  {link_name}: {n_coll} collision blocks after patch")

    dest.write_text(new_text)
    print(f"wrote {dest}")


def main():
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_SRC
    dest = src.with_name(src.stem + "_omnisim.urdf")
    patch(src, dest)


if __name__ == "__main__":
    main()
