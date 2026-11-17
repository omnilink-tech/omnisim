#!/usr/bin/env python3
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

"""Generate the PX4 x500 package from PX4-gazebo-models at a pinned commit.

    python projects/robots/px4/x500/make_x500.py            # fetch + generate
    python projects/robots/px4/x500/make_x500.py --src DIR  # use a local copy
                                                            # of models/x500_base

Deterministic: the same upstream commit gives byte-identical outputs. See
PROVENANCE.md for what is taken, what is left out and why.

Writes, next to this file:
  LICENSE.upstream          the upstream BSD-3-Clause licence, verbatim
  meshes/parts/<Part>.dae   the frame mesh split into one file per part (the two
                            heaviest simplified to .obj, see DECIMATE), so each
                            can carry its own material (OmniSim's Mesh node
                            cannot select a sub-mesh). Only the frame's own
                            geometry is kept: the upstream model's NXP /
                            RDDRONE decals are separate textured planes in
                            model.sdf and are NOT reproduced.
  meshes/5010Base.dae, 5010Bell.dae, 1345_prop_cw.stl, 1345_prop_ccw.stl, CF.png
  urdf/x500.urdf            poses, collision boxes, mass and inertia from
                            model.sdf; joint and sensor names from the Mavic
                            URDF so mavic_omnilink_bridge flies it unchanged
"""
from __future__ import annotations

import argparse
import copy
import math
import shutil
import tempfile
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
COMMIT = "d679f3a77999002f01f7e404fc365d124be15221"
RAW = f"https://raw.githubusercontent.com/PX4/PX4-gazebo-models/{COMMIT}/models/x500_base"
FILES = ["LICENSE", "meshes/NXP-HGD-CF.dae", "meshes/5010Base.dae", "meshes/5010Bell.dae",
         "meshes/1345_prop_cw.stl", "meshes/1345_prop_ccw.stl", "meshes/CF.png"]
COLLADA = "http://www.collada.org/2005/11/COLLADASchema"

# Part colours (URDF materials) for the split frame. CarbonFiber carries the
# upstream carbon texture; the rest are flat, matched to the parts' roles.
PARTS = {
    "CarbonFiber": ("1 1 1 1", "../meshes/CF.png", (0.35, 0.0)),
    "Metal": ("0.6 0.61 0.63 1", None, (0.3, 0.9)),
    "LandingFoam": ("0.22 0.22 0.23 1", None, None),
    "LandingRubber": ("0.05 0.05 0.05 1", None, None),
    "RailsRubber": ("0.05 0.05 0.05 1", None, None),
    "FMURubber": ("0.05 0.05 0.05 1", None, None),
    "FMUK66": ("0.08 0.09 0.09 1", None, None),
    "RailsAntennaHolder": ("0.07 0.07 0.07 1", None, None),
    "LandingPlastic": ("0.06 0.06 0.07 1", None, None),
}
# Parts the renderer warns about (> 100k vertices: it counts ~3 per triangle)
# are simplified to this many faces and written as OBJ. Both are untextured,
# so only fine surface detail is lost; bounds are preserved.
DECIMATE = {"FMUK66": 30000, "LandingPlastic": 30000}
A = 0.174          # motor arm half-span (model.sdf rotor poses)
ROTOR_Z = 0.06
# name -> (x, y, prop mesh). Front = +x. model.sdf's rotor_0..3 mapped onto the
# Mavic joint names the bridge looks up; FL / RR spin one way, FR / RL the other.
ROTORS = {"front left": (A, A, "cw"), "front right": (A, -A, "ccw"),
          "rear left": (-A, A, "ccw"), "rear right": (-A, -A, "cw")}


def fetch(dest: Path) -> None:
    for rel in FILES:
        out = dest / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(f"{RAW}/{rel}", timeout=300) as r:
            out.write_bytes(r.read())


def split_frame(src: Path, out_dir: Path) -> list[str]:
    """One .dae per <node> of the frame's visual scene, keeping only that
    node's geometry. Returns the part names written."""
    ET.register_namespace("", COLLADA)
    q = lambda tag: f"{{{COLLADA}}}{tag}"
    root = ET.parse(src).getroot()
    scene = root.find(q("library_visual_scenes")).find(q("visual_scene"))
    names = [n.get("name") for n in scene.findall(q("node")) if n.find(q("instance_geometry")) is not None]
    out_dir.mkdir(parents=True, exist_ok=True)
    for name in names:
        r = copy.deepcopy(root)
        sc = r.find(q("library_visual_scenes")).find(q("visual_scene"))
        for n in list(sc.findall(q("node"))):
            if n.get("name") != name:
                sc.remove(n)
        keep = sc.find(q("node")).find(q("instance_geometry")).get("url").lstrip("#")
        lg = r.find(q("library_geometries"))
        for g in list(lg.findall(q("geometry"))):
            if g.get("id") != keep:
                lg.remove(g)
        ET.ElementTree(r).write(out_dir / f"{name}.dae", xml_declaration=True, encoding="utf-8")
    return names


def decimate(parts_dir: Path) -> None:
    """Simplify the DECIMATE parts to OBJ (needs trimesh + fast_simplification)."""
    import numpy as np
    import fast_simplification
    import trimesh
    for name, faces in DECIMATE.items():
        src = parts_dir / f"{name}.dae"
        mesh = trimesh.load(src, force="mesh")
        # Collada stores these triangles UNWELDED (114k vertices for 47k faces):
        # simplified as-is, every triangle is its own island and the part shreds.
        mesh.merge_vertices(merge_tex=True, merge_norm=True)
        v, f = fast_simplification.simplify(np.asarray(mesh.vertices, dtype=np.float32),
                                            np.asarray(mesh.faces), target_count=faces)
        out = trimesh.Trimesh(v, f, process=True)
        out.fix_normals()
        out.export(parts_dir / f"{name}.obj")
        src.unlink()


def part_file(name: str) -> str:
    return f"{name}.obj" if name in DECIMATE else f"{name}.dae"


def material(name: str, rgba: str, texture=None, pbr=None) -> str:
    inner = f'<color rgba="{rgba}"/>'
    if texture:
        inner += f'<texture filename="{texture}"/>'
    if pbr:
        inner += f'<omnisim roughness="{pbr[0]}" metalness="{pbr[1]}"/>'
    return f'  <material name="{name}">{inner}</material>\n'


def visual(geometry: str, xyz=(0, 0, 0), rpy=(0, 0, 0), mat: str = "x500_frame") -> str:
    return (f'    <visual>\n      <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>\n'
            f'      <geometry>{geometry}</geometry>\n      <material name="{mat}"/>\n    </visual>\n')


def collision(size, xyz=(0, 0, 0), rpy=(0, 0, 0)) -> str:
    return (f'    <collision>\n      <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>\n'
            f'      <geometry><box size="{size[0]} {size[1]} {size[2]}"/></geometry>\n    </collision>\n')


def mavic_gimbal_and_sensors() -> str:
    """The Mavic URDF's camera gimbal chain and sensor blocks, moved under the
    x500's frame: the bridge needs its camera / IMU / GPS devices by name."""
    src = (REPO / "projects/robots/dji/mavic/urdf/mavic_2_pro.urdf").read_text(encoding="utf-8")
    start = src.index("  <!-- =========================================================\n       Camera gimbal")
    part = src[start:src.index("</robot>")]
    part = part.replace('<origin xyz="0.0412774 -0.00469654 -0.00405862" rpy="0 0 0"/>',
                        '<origin xyz="0.10 0 -0.02" rpy="0 0 0"/>')
    assert '<origin xyz="0.10 0 -0.02"' in part
    return part


def write_urdf(path: Path, parts: list[str]) -> None:
    out = ['<?xml version="1.0"?>\n<!--\n'
           f'PX4 x500 for OmniSim. GENERATED by make_x500.py from PX4-gazebo-models\n'
           f'models/x500_base @ {COMMIT[:9]} (BSD-3-Clause, (c) 2022 Rudis Laboratories;\n'
           'see ../LICENSE.upstream and ../PROVENANCE.md). Do not hand-edit.\n\n'
           'Flown by mavic_omnilink_bridge: joint and sensor names are the Mavic\'s,\n'
           'and the rotor dynamics (thrust coefficient, rotor anchors centred on this\n'
           'robot\'s CoM) are declared by the world in the robot\'s customData.\n'
           'Rotor blur discs start fully transparent; the bridge fades them in with\n'
           'rotor speed (airframe key rotor_disc_material = "x500_prop_disc").\n'
           'Propeller and gimbal links carry no collision (Mavic issue #10).\n-->\n'
           '<robot name="x500">\n']
    for p in parts:
        rgba, tex, pbr = PARTS[p]
        out.append(material(f"x500_{p}", rgba, tex, pbr))
    out.append(material("x500_frame", "0.09 0.09 0.10 1"))
    out.append(material("x500_motor", "0.30 0.31 0.33 1"))
    out.append(material("x500_prop", "0.05 0.05 0.05 1", pbr=(0.35, 0.0)))
    out.append(material("x500_prop_disc", "0.08 0.08 0.09 0", pbr=(0.3, 0.0)))
    out.append(material("cam_dark", "0.08 0.08 0.08 1"))
    out.append('  <link name="base_link">\n')
    for p in parts:
        out.append(visual(f'<mesh filename="../meshes/parts/{part_file(p)}"/>', (0, 0, 0.025),
                          (0, 0, round(math.pi, 9)), f"x500_{p}"))
    for (x, y, _) in ROTORS.values():
        out.append(visual('<mesh filename="../meshes/5010Base.dae"/>', (x, y, 0.032), (0, 0, -0.45), "x500_motor"))
    # Imported meshes cast no shadow; these primitives, hidden inside the
    # frame, give the drone an X-shaped shadow to read its position by.
    out.append(visual('<box size="0.11 0.09 0.03"/>', (0, 0, 0.03)))
    for (x, y, _) in ROTORS.values():
        out.append(visual('<box size="0.21 0.012 0.012"/>', (x / 2, y / 2, 0.025),
                          (0, 0, round(math.atan2(y, x), 6))))
    out.append(collision((0.35355339059327373, 0.35355339059327373, 0.05), (0, 0, 0.007)))
    out.append(collision((0.015, 0.015, 0.21), (0, -0.098, -0.123), (-0.35, 0, 0)))
    out.append(collision((0.015, 0.015, 0.21), (0, 0.098, -0.123), (0.35, 0, 0)))
    out.append(collision((0.25, 0.015, 0.015), (0, -0.132, -0.2195)))
    out.append(collision((0.25, 0.015, 0.015), (0, 0.132, -0.2195)))
    out.append('    <inertial>\n      <origin xyz="0 0 0" rpy="0 0 0"/>\n      <mass value="2.0"/>\n'
               '      <inertia ixx="0.02166666666666667" ixy="0" ixz="0" iyy="0.02166666666666667" iyz="0" '
               'izz="0.04000000000000001"/>\n    </inertial>\n  </link>\n')
    for name, (x, y, spin) in ROTORS.items():
        link = name.replace(" ", "_") + "_propeller_link"
        out.append(f'  <joint name="{name} propeller" type="continuous">\n    <parent link="base_link"/>\n'
                   f'    <child link="{link}"/>\n    <origin xyz="{x} {y} {ROTOR_Z}" rpy="0 0 0"/>\n'
                   '    <axis xyz="0 0 1"/>\n    <limit effort="30.0" velocity="576.0"/>\n'
                   '    <dynamics damping="0.0" friction="0.0"/>\n  </joint>\n'
                   f'  <link name="{link}">\n')
        out.append(visual('<cylinder radius="0.165" length="0.002"/>', (0, 0, 0.004), mat="x500_prop_disc"))
        out.append(visual(f'<mesh filename="../meshes/1345_prop_{spin}.stl" scale="0.8461538461538461 '
                          f'0.8461538461538461 0.8461538461538461"/>', (-0.022, -0.14638461538461536, -0.016),
                          mat="x500_prop"))
        out.append(visual('<mesh filename="../meshes/5010Bell.dae"/>', (0, 0, -0.032), mat="x500_motor"))
        out.append('    <inertial>\n      <mass value="0.016076923076923075"/>\n'
                   '      <inertia ixx="3.8464910483993325e-07" ixy="0" ixz="0" iyy="2.6115851691700804e-05" '
                   'iyz="0" izz="2.649858234714004e-05"/>\n    </inertial>\n  </link>\n')
    out.append(mavic_gimbal_and_sensors())
    out.append("</robot>\n")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(out), encoding="utf-8", newline="\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--src", type=Path, help="local copy of models/x500_base (else fetched)")
    args = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        src = args.src
        if src is None:
            src = Path(tmp)
            fetch(src)
        shutil.copyfile(src / "LICENSE", HERE / "LICENSE.upstream")
        meshes = HERE / "meshes"
        meshes.mkdir(exist_ok=True)
        for f in ("5010Base.dae", "5010Bell.dae", "1345_prop_cw.stl", "1345_prop_ccw.stl", "CF.png"):
            shutil.copyfile(src / "meshes" / f, meshes / f)
        parts = split_frame(src / "meshes" / "NXP-HGD-CF.dae", meshes / "parts")
        decimate(meshes / "parts")
    assert sorted(parts) == sorted(PARTS), parts
    write_urdf(HERE / "urdf" / "x500.urdf", parts)
    print(f"x500 generated from {COMMIT[:9]}: {len(parts)} frame parts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
