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

"""Write a PRIMITIVE-COLLISION variant of the deploy G1 URDF.

ROOT CAUSE of the G1 deploy collapse (FALL@~1.07s, policy/DR/gain/cold-start
independent): the GPU trainer batches on SolverMuJoCo, which forces PRIMITIVE
collision (build_g1_native_prim strips every `<collision>` MESH and keeps only
the two primitive foot boxes -- the detailed STL colliders overflow the
collision broad-phase when replicated to N worlds). But the DEPLOY world loads
g1_23dof_omnisim.urdf with ALL 21 STL mesh colliders (incl. an STL collider on
each foot ALONGSIDE the box). So the policy is trained on box-only foot contact
and a contact-free leg skeleton, then deployed onto full STL mesh contact +
21 extra colliders (self-collision between legs, near-ground shin/knee contact,
constraint-solver pressure). For a biped at the inverted-pendulum stability
edge that contact mismatch is enough to topple it -- and being a COLLISION
difference, it is invisible to policy/DR/gain/cold-start changes, which is the
exact signature we observed.

FIX: make the deploy use the SAME primitive collision the policy learned on.
This writes g1_23dof_omnisim_prim.urdf: every `<collision>` whose geometry is a
`<mesh>` is removed; the two primitive foot `<box>` colliders are kept; `<visual>`
and `<inertial>` are untouched (the deploy still renders the full robot and the
dynamics/inertias are identical -- ONLY the collision set changes, exactly like
build_g1_native_prim does in-memory for the trainer).
"""
from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())))
# SINGLE SOURCE OF TRUTH for the foot collision box. This generator does NOT
# re-declare the box -- it PRESERVES the one authored in the source URDF -- so
# we VERIFY each kept box against the spec instead, turning a silent preserve
# into an explicit SoT conformance check (the source URDF can't silently drift
# from g1_physics.json / the trainer's in-memory strip).
from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402

SRC = Path("projects/robots/unitree/g1/urdf/g1_23dof_omnisim.urdf")
DST = Path("projects/robots/unitree/g1/urdf/g1_23dof_omnisim_prim.urdf")


def _box_matches_spec(col: ET.Element) -> bool:
    """True iff this <collision>'s box size+origin equal SPEC.FOOT_BOX_*."""
    geom = col.find("geometry")
    box = geom.find("box") if geom is not None else None
    if box is None:
        return False
    try:
        size = tuple(float(x) for x in (box.get("size") or "").split())
    except ValueError:
        return False
    origin = col.find("origin")
    xyz_str = origin.get("xyz") if origin is not None else "0 0 0"
    try:
        xyz = tuple(float(x) for x in (xyz_str or "0 0 0").split())
    except ValueError:
        return False
    return (size == tuple(float(c) for c in SPEC.FOOT_BOX_SIZE)
            and xyz == tuple(float(c) for c in SPEC.FOOT_BOX_ORIGIN))


def main() -> int:
    # Keep ONLY the primitive foot-link box colliders, matching the trainer's
    # _build_g1_full_prim_builder (gpu_newton_g1_walk_trainer.py) EXACTLY: it
    # strips every <collision> and re-adds a box only on the two ankle_roll
    # links. Earlier this script kept ALL non-mesh collisions, which left 4
    # extra shoulder primitive colliders the trainer never had -- a model
    # mismatch that breaks fine-tune transfer (the held arms self-collide).
    FOOT_LINKS = ("left_ankle_roll_link", "right_ankle_roll_link")
    tree = ET.parse(SRC)
    root = tree.getroot()
    n_links = 0
    n_removed_mesh = 0
    n_kept_box = 0
    n_box_offspec = 0
    kept_box_links = []
    for link in root.findall("link"):
        n_links += 1
        name = link.get("name")
        for col in list(link.findall("collision")):
            geom = col.find("geometry")
            is_box = geom is not None and geom.find("box") is not None
            keep = is_box and name in FOOT_LINKS
            if keep:
                n_kept_box += 1
                kept_box_links.append(name)
                # SoT conformance: the kept foot box must equal the spec.
                if not _box_matches_spec(col):
                    n_box_offspec += 1
            else:
                link.remove(col)
                n_removed_mesh += 1
    # Preserve the XML declaration; ET writes a compact form which the URDF
    # parser (and newton add_urdf) reads fine.
    tree.write(DST, encoding="utf-8", xml_declaration=True)
    print(f"[prim-urdf] src   = {SRC}")
    print(f"[prim-urdf] dst   = {DST}")
    print(f"[prim-urdf] links = {n_links}")
    print(f"[prim-urdf] removed MESH <collision> = {n_removed_mesh}")
    print(f"[prim-urdf] kept primitive <collision> = {n_kept_box} on {kept_box_links}")
    print(f"[prim-urdf] spec foot box: size={tuple(SPEC.FOOT_BOX_SIZE)} "
          f"origin={tuple(SPEC.FOOT_BOX_ORIGIN)}  off-spec kept boxes={n_box_offspec}")
    if n_kept_box < 2:
        print("[prim-urdf] WARNING: expected >=2 primitive foot colliders; "
              "robot may fall through the floor!", file=sys.stderr)
        return 2
    if n_box_offspec:
        print("[prim-urdf] ERROR: a kept foot box does NOT match the SoT "
              "(g1_physics_spec.FOOT_BOX_*). The source URDF has drifted from "
              "the spec -- fix one or the other before deploying.", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
