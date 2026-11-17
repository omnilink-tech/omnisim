#!/usr/bin/env python

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

"""Generate the in-repo OmniSim Deep Robotics packages from the upstream
DeepRoboticsLab/deep_robotics_model repository.

Upstream: https://github.com/DeepRoboticsLab/deep_robotics_model (BSD-3-Clause,
(c) 2024 DeepRoboticsLab). Seven robots -- Lite3, X30 (quadrupeds), M20, M20S,
M20_Piper (wheeled-legged quadrupeds, the last with an AgileX Piper arm), and
the DR02 Standard / DR02 Pro humanoids -- each shipped as URDF + binary STL.

Transforms applied to every upstream URDF, in order:

  1. Mesh refs  ./meshes/<f>  ->  package://<pkg>/meshes/<f>
     so the OmniSim importer resolves them by walking up from the URDF
     directory (the same layout as projects/robots/unitree/go2). The M20, M20S
     and M20_Piper leg meshes are byte-identical upstream (verified by md5),
     so only m20/meshes/ carries them and the other two point at it.
  2. A <rest>VALUE</rest> child (OmniSim extension, see OmUrdfImporter.cpp) is
     injected into every leg joint of the quadrupeds so they SPAWN in a
     standing crouch. At q=0 the Lite3 / X30 knee is OUTSIDE its own range
     (lower limit 0.524 / 0.349 rad) and the M20 family stands straight-legged
     on its wheels; both load poses explode or topple. The angles come from
     the leg geometry (thigh / shank lengths in the URDF) with the foot placed
     under the hip -- see docs in PROVENANCE.md for the derivation.
  3. The Lite3 / X30 foot links become bare frames: their <inertial> (an
     all-zero / 1e-12 tensor with 0.02 / 0.06 kg of mass) and <collision> go,
     and that mass is folded into the shank. With the inertial kept, the importer makes the foot a 0.02 kg body
     welded to the shank whose sphere coincides with the one injected in step
     4; the pair is not self-collision-filtered and the permanent overlap
     shoves the leg. Without it the foot is an inert frame, as on the Go2.
  4. The Lite3 / X30 foot contact sphere is injected onto the shank link at
     the foot offset, FIRST in the link's collision list: an inert foot frame
     carries no collider, the shank's own cylinders end short of the foot tip,
     and only the first collider of a link is registered unless
     WorldInfo.newtonCompoundColliders is TRUE. Same fix as the Go2.
  5. M20_Piper: `upper="0。611"` (a full-width ideographic period, U+3002) on
     fl_hipx_joint is `upper="0.611"` -- every other M20 URDF has the ASCII
     value. Reported upstream.

Run from anywhere:  python make_urdfs.py [path/to/deep_robotics_model]
Default upstream path is a sparse clone under $TEMP/deep_robotics_model:

    git clone --depth 1 --filter=blob:none --sparse \
        https://github.com/DeepRoboticsLab/deep_robotics_model.git
    git -C deep_robotics_model sparse-checkout set --no-cone '/*' '!/*/usd' '!/*/mjcf'

The outputs are committed, so this script only needs re-running to refresh
from upstream. It refuses to run if the upstream tree is missing a file it
expects, and asserts the number of <rest> tags it wrote.
"""
from __future__ import annotations

import hashlib
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

# Standing crouch per family. Hip-pitch and knee axes are (0,-1,0) in every
# Deep Robotics URDF, so a NEGATIVE hip-pitch swings the thigh backward and a
# POSITIVE knee flexes the shank forward -- the "<" leg every Deep Robotics dog
# stands in. Foot lands within 2 cm of directly under the hip (planar FK):
#   Lite3  thigh 0.200  shank 0.210  -> hip 0.254 m above the foot centre
#   X30    thigh 0.300  shank 0.310  -> hip 0.379 m
#   M20    thigh 0.250  shank 0.250  -> hip 0.348 m above the wheel axle
REST_LITE3 = {"HipX_joint": 0.0, "HipY_joint": -1.0, "Knee_joint": 1.8}
REST_X30 = {"HipX_joint": 0.0, "HipY_joint": -0.9, "Knee_joint": 1.8}
# The M20 family's REAR joint ranges are the mirror image of the front ones
# (fl_hipy [-2.583, 2.286] vs hl_hipy [-2.286, 2.583], likewise the knees), so
# the rear legs are folded the other way: knees toward the body centre.
REST_M20 = {"fl_hipx_joint": 0.0, "fl_hipy_joint": -0.8, "fl_knee_joint": 1.6,
            "fr_hipx_joint": 0.0, "fr_hipy_joint": -0.8, "fr_knee_joint": 1.6,
            "hl_hipx_joint": 0.0, "hl_hipy_joint": 0.8, "hl_knee_joint": -1.6,
            "hr_hipx_joint": 0.0, "hr_hipy_joint": 0.8, "hr_knee_joint": -1.6}

# Per-package build recipe. `meshes` = (upstream dir, package-relative dir) or
# None when the package points at another package's meshes (`mesh_pkg`).
ROBOTS = [
    dict(pkg="lite3", src="Lite3/urdf/Lite3.urdf", out="lite3/urdf/lite3.urdf",
         meshes=("Lite3/urdf/meshes", "lite3/meshes"), mesh_pkg="lite3",
         rest=REST_LITE3, n_rest=12,
         foot=dict(links=("FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"), mass=0.02, r=0.022,
                   shanks=("FL_SHANK", "FR_SHANK", "HL_SHANK", "HR_SHANK"), z=-0.21012)),
    dict(pkg="x30", src="X30/urdf/X30.urdf", out="x30/urdf/x30.urdf",
         meshes=("X30/urdf/meshes", "x30/meshes"), mesh_pkg="x30",
         rest=REST_X30, n_rest=12,
         foot=dict(links=("FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"), mass=0.06, r=0.036,
                   shanks=("FL_SHANK", "FR_SHANK", "HL_SHANK", "HR_SHANK"), z=-0.31)),
    dict(pkg="m20", src="M20/urdf/M20.urdf", out="m20/urdf/m20.urdf",
         meshes=("M20/urdf/meshes", "m20/meshes"), mesh_pkg="m20",
         rest=REST_M20, n_rest=12),
    dict(pkg="m20s", src="M20S/urdf/M20S.urdf", out="m20s/urdf/m20s.urdf",
         meshes=None, mesh_pkg="m20", shared_with="M20S/urdf/meshes",
         rest=REST_M20, n_rest=12),
    dict(pkg="m20_piper", src="M20_Piper/urdf/M20_Piper.urdf", out="m20_piper/urdf/m20_piper.urdf",
         meshes=("M20_Piper/urdf/meshes", "m20_piper/meshes"), mesh_pkg="m20_piper",
         shared_with="M20_Piper/urdf/meshes", shared_pkg="m20",
         rest=REST_M20, n_rest=12,
         fixups=[("0。611", "0.611")]),
    dict(pkg="dr02", src="DR02/urdf/standard/DR02-std.urdf", out="dr02/urdf/dr02_std.urdf",
         meshes=("DR02/urdf/standard/meshes", "dr02/meshes/std"), mesh_pkg="dr02", mesh_sub="std",
         rest={}, n_rest=0),
    dict(pkg="dr02", src="DR02/urdf/pro/DR02-pro.urdf", out="dr02/urdf/dr02_pro.urdf",
         meshes=("DR02/urdf/pro/meshes", "dr02/meshes/pro"), mesh_pkg="dr02", mesh_sub="pro",
         rest={}, n_rest=0),
    dict(pkg="dr02", src="DR02/urdf/pro/DR02-pro_fix_joints.urdf", out="dr02/urdf/dr02_pro_fixed_wrists.urdf",
         meshes=None, mesh_pkg="dr02", mesh_sub="pro",
         rest={}, n_rest=0),
]


def find_upstream() -> Path:
    if len(sys.argv) > 1:
        return Path(sys.argv[1])
    tmp = os.environ.get("TEMP") or os.environ.get("TMP") or "/tmp"
    return Path(tmp) / "deep_robotics_model"


def md5(path: Path) -> str:
    return hashlib.md5(path.read_bytes()).hexdigest()


def upstream_commit(up: Path) -> str:
    try:
        return subprocess.check_output(["git", "-C", str(up), "rev-parse", "HEAD"],
                                       text=True).strip()
    except Exception:
        return "(not a git checkout)"


def inject_rest(txt: str, rest: dict) -> tuple[str, int]:
    """Add <rest>V</rest> before </joint> of every joint whose name ends with a
    key of `rest`. Matches the Go2 generator's regex; every Deep Robotics
    joint block is flat (no nested </joint>)."""
    count = 0

    def repl(m: re.Match) -> str:
        nonlocal count
        block, jname = m.group(0), m.group(1)
        for suffix, value in rest.items():
            if jname.endswith(suffix):
                count += 1
                return block[:-len("</joint>")] + f"    <rest>{value}</rest>\n    </joint>"
        return block

    txt = re.sub(r'<joint name="([^"]+)" type="(?:revolute|continuous)">.*?</joint>',
                 repl, txt, flags=re.DOTALL)
    return txt, count


def strip_foot_inertial(txt: str, links: tuple, shanks: tuple) -> str:
    """Turn each foot link into a bare frame: drop its <inertial> (so the
    importer makes it an inert, physics-less child, the Go2 layout) and its
    <collision> (the shank carries the contact sphere, step 4), and fold its
    mass into the shank so the robot's total mass is unchanged. A foot that
    keeps its inertial becomes a 0.02 kg body welded to the shank whose sphere
    sits exactly on the injected one; that pair is not filtered as
    self-collision and the permanent overlap shoves the leg (measured
    2026-09-08: the Lite3 skated backward off its feet and flipped)."""
    for link, shank in zip(links, shanks):
        pat = re.compile(r'(<link name="' + re.escape(link) + r'">.*?)<inertial>.*?</inertial>(.*?</link>)', re.DOTALL)
        m = pat.search(txt)
        assert m, f"foot link {link}: inertial element not found"
        mass = float(re.search(r'<mass value="([^"]+)"', m.group(0)).group(1))
        txt = pat.sub(lambda mm: mm.group(1).rstrip() + mm.group(2), txt, count=1)
        cpat = re.compile(r'(<link name="' + re.escape(link) + r'">)\s*<collision>.*?</collision>', re.DOTALL)
        txt, n = cpat.subn(lambda mm: mm.group(1), txt, count=1)
        assert n == 1, f"foot link {link}: collision element not found"
        spat = re.compile(r'(<link name="' + re.escape(shank) + r'">.*?<mass value=")([^"]+)(")', re.DOTALL)
        sm = spat.search(txt)
        assert sm, f"shank link {shank}: mass not found"
        txt = spat.sub(lambda mm: mm.group(1) + ("%.5g" % (float(mm.group(2)) + mass)) + mm.group(3), txt, count=1)
    return txt


def inject_shank_foot(txt: str, shanks: tuple, z: float, r: float) -> str:
    """Put the foot contact sphere on the shank link itself (see docstring 4)."""
    coll = ('\n        <collision>\n'
            f'            <origin xyz="0 0 {z}"/>\n'
            '            <geometry>\n'
            f'                <sphere radius="{r}"/>\n'
            '            </geometry>\n'
            '        </collision>')
    for shank in shanks:
        pat = re.compile(r'(<link name="' + re.escape(shank) + r'">)')
        txt, n = pat.subn(lambda m: m.group(1) + coll, txt, count=1)
        assert n == 1, f"shank link {shank} not found"
    return txt


def rewrite_meshes(txt: str, spec: dict, shared_names: set) -> str:
    """./meshes/<f> -> package://<pkg>/meshes[/<sub>]/<f>, routing files that
    exist in the shared M20 mesh set to that package instead."""
    def repl(m: re.Match) -> str:
        name = m.group(1)
        if name in shared_names and spec.get("shared_pkg"):
            return f'filename="package://{spec["shared_pkg"]}/meshes/{name}"'
        sub = spec.get("mesh_sub")
        base = f'package://{spec["mesh_pkg"]}/meshes' + (f"/{sub}" if sub else "")
        return f'filename="{base}/{name}"'

    txt, n = re.subn(r'filename="\./meshes/([^"]+)"', repl, txt)
    assert n > 0, "no ./meshes/ references rewritten"
    return txt


def copy_meshes(src_dir: Path, dst_dir: Path, skip_names: set) -> int:
    dst_dir.mkdir(parents=True, exist_ok=True)
    n = 0
    for f in sorted(src_dir.iterdir()):
        if f.is_file() and f.name not in skip_names:
            shutil.copy2(f, dst_dir / f.name)
            n += 1
    return n


def main() -> None:
    up = find_upstream()
    if not (up / "LICENSE.txt").exists():
        raise SystemExit(f"upstream clone not found at {up}\n(see the module docstring for the sparse-clone recipe)")
    print(f"upstream {up} @ {upstream_commit(up)}")

    # The M20 mesh set every M20-family URDF shares (byte-identical upstream).
    m20_meshes = up / "M20/urdf/meshes"
    m20_names = {f.name for f in m20_meshes.iterdir() if f.is_file()}
    m20_md5 = {f.name: md5(f) for f in m20_meshes.iterdir() if f.is_file()}

    shutil.copy2(up / "LICENSE.txt", HERE / "LICENSE.upstream")

    for spec in ROBOTS:
        src = up / spec["src"]
        if not src.exists():
            raise SystemExit(f"missing upstream file: {src}")
        txt = src.read_text(encoding="utf-8")

        for old, new in spec.get("fixups", []):
            assert old in txt, f"{spec['pkg']}: expected to fix {old!r} but it is not there"
            txt = txt.replace(old, new)

        shared = set()
        if spec.get("shared_with"):
            # Verify the claim before relying on it: every leg mesh this URDF
            # ships must be byte-identical to the M20 copy.
            own = up / spec["shared_with"] if spec["meshes"] is None else up / spec["meshes"][0]
            for f in own.iterdir():
                if f.name in m20_md5:
                    assert md5(f) == m20_md5[f.name], f"{spec['pkg']}: {f.name} differs from M20's"
                    shared.add(f.name)
            if spec["meshes"] is None:
                assert shared == m20_names, f"{spec['pkg']}: mesh set is not the M20 set"

        txt = rewrite_meshes(txt, spec, shared)

        if spec.get("foot"):
            ft = spec["foot"]
            txt = strip_foot_inertial(txt, ft["links"], ft["shanks"])
            txt = inject_shank_foot(txt, ft["shanks"], ft["z"], ft["r"])

        txt, n_rest = inject_rest(txt, spec["rest"])
        assert n_rest == spec["n_rest"], f"{spec['pkg']}: expected {spec['n_rest']} rest tags, wrote {n_rest}"

        out = HERE / spec["out"]
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(txt, encoding="utf-8", newline="\n")

        n_mesh = 0
        if spec["meshes"]:
            n_mesh = copy_meshes(up / spec["meshes"][0], HERE / spec["meshes"][1], shared)

        (HERE / spec["pkg"] / "omnisim.yaml").write_text("publish: true\n", encoding="utf-8")
        print(f"wrote {out.relative_to(HERE)}  ({n_rest} <rest> tags, {n_mesh} meshes copied)")


if __name__ == "__main__":
    main()
