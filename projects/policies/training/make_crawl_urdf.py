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

"""Generate the G1 *crawl* URDF variant from the canonical prim URDF.

WHY THIS EXISTS
---------------
The in-engine trainer compiles its contact model from
`g1_23dof_omnisim_prim.urdf` (see `backends/g1_physics.json` -> urdf.prim).
That URDF only carries `<collision>` geometry on the two ankle/foot links —
every other link (hands, forearms, knees, torso, pelvis) is *visual mesh
only*, so it cannot bear ground contact. A walking G1 stands on its feet, so
that is correct for locomotion; but a CRAWL needs the hands and knees to
touch the floor. Without colliders there, a hands-and-knees G1 sinks its
arms and shins straight through the ground and there is nothing to crawl on.

This script emits `g1_23dof_omnisim_crawl.urdf` — an exact copy of the prim
URDF with added `<collision>` boxes on the crawl-contact surfaces. It is a
SEPARATE file: the walking/standing/carry policies and the physics-spec CI
conformance keep compiling the untouched prim URDF.

HIGH-CRAWL stage (this file): hands + knees/shins only. The torso stays off
the floor, so no belly collider yet. When we lower to a belly LOW-crawl we
extend `CONTACT_COLLIDERS` with the forearm (elbow_link), torso and pelvis.

Box dimensions are taken from the hand-authored `g1_full.mjcf.xml` geoms
(which were fit to the same meshes); URDF `<box size>` is FULL extents, so
the MuJoCo half-extents there are doubled here.

Run:  python projects/policies/training/make_crawl_urdf.py
"""
from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
SRC = REPO / "projects/robots/unitree/g1/urdf/g1_23dof_omnisim_prim.urdf"
DST = REPO / "projects/robots/unitree/g1/urdf/g1_23dof_omnisim_crawl.urdf"

# link name -> collision block (origin xyz + box size, both in metres).
# Dims doubled from g1_full.mjcf.xml half-extents; origins copied as-is.
# Covers BOTH crawl postures: high crawl uses hands+knees; low crawl also
# rests on forearms + torso/belly + pelvis. The extra low-crawl surfaces are
# harmless to high crawl (they simply stay off the ground there).
CONTACT_COLLIDERS = {
    # --- high crawl (hands-and-knees) ---
    # knees / shins — the shank box (knee to ankle), the kneeling support.
    "left_knee_link":  ("0.012 0 -0.1375", "0.104 0.072 0.355"),
    "right_knee_link": ("0.012 0 -0.1375", "0.104 0.072 0.355"),
    # hands — the palm/hand box forward of the wrist, the forward support.
    "left_wrist_roll_rubber_hand":  ("0.1267 -0.0062 0.0117", "0.253 0.072 0.107"),
    "right_wrist_roll_rubber_hand": ("0.1267  0.0062 0.0117", "0.253 0.072 0.107"),
    # --- low crawl (belly + forearms) ---
    # forearms — the elbow_link box; lies flat on the ground and pulls.
    "left_elbow_link":  ("0.035  0.0018 -0.0078", "0.130 0.066 0.076"),
    "right_elbow_link": ("0.035 -0.0018 -0.0078", "0.130 0.066 0.076"),
    # torso/chest + pelvis — the belly that rests and carries most weight.
    "torso_link": ("0.0085 0 0.145", "0.150 0.215 0.310"),
    "pelvis":     ("0 0 -0.02", "0.120 0.160 0.120"),
}


def _collision_xml(origin: str, size: str) -> str:
    return (
        "    <collision>\n"
        f"      <origin xyz=\"{origin}\" rpy=\"0 0 0\" />\n"
        "      <geometry>\n"
        f"        <box size=\"{size}\" />\n"
        "      </geometry>\n"
        "    </collision>\n"
    )


def main() -> None:
    text = SRC.read_text()
    added = []
    for link, (origin, size) in CONTACT_COLLIDERS.items():
        # Each link block ends with the sequence "</visual>\n    </link>" that
        # is preceded (a few lines up) by this link's unique mesh filename.
        # Anchor on the mesh filename so the insertion target is unambiguous.
        anchor = f"meshes/{link}.STL"
        idx = text.find(anchor)
        if idx < 0:
            raise SystemExit(f"anchor not found for {link!r}: {anchor}")
        # From the anchor, find this link's closing "</link>" and insert the
        # collision block just before it.
        close = text.find("</link>", idx)
        if close < 0:
            raise SystemExit(f"</link> not found after {link!r}")
        text = text[:close] + _collision_xml(origin, size) + "    " + text[close:]
        added.append(link)

    DST.write_text(text)
    print(f"wrote {DST.relative_to(REPO)}")
    print(f"added {len(added)} crawl-contact colliders:")
    for link in added:
        print(f"  + {link}")


if __name__ == "__main__":
    main()
