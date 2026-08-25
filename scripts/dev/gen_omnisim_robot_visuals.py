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
"""Author the visual shells of OmniSim's own generic robot packages.

Every robot under ``projects/robots/omnisim/`` is defined *entirely* by
primitive geometry authored here -- boxes, cylinders and spheres composed
into a shell per link.  Nothing in those packages is imported, tessellated
or derived from third-party CAD, which is the whole point: the packages
ship in an Apache-2.0 tree and their geometry has to be ours.

The kinematic skeleton (joint origins, axes, limits) and the dynamics
(masses, inertia tensors, collision primitives) are NOT touched by this
script -- it rewrites only the ``<visual>`` blocks of the named links, so
worlds, controllers, trained policies and ghosts are unaffected.

Usage::

    python scripts/dev/gen_omnisim_robot_visuals.py --robot omniarm6
    python scripts/dev/gen_omnisim_robot_visuals.py --all --check

``--check`` verifies the files on disk already match what this script
would emit (a CI-friendly "the shells are still the authored ones" gate).
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]

# --- palette -------------------------------------------------------------
# OmniSim house colours: cream shell, near-black joint housings, mimosa
# accent collars.  Deliberately distinct from any vendor livery.
MATERIALS = {
    # rgba, then the PBR the URDF importer honours via an <omnisim/> child.
    # A cobot does not read as real without this: matte paint on the shells,
    # satin anodised metal on the joint housings, a soft metallic accent collar.
    "omnisim_shell": ("0.898 0.894 0.882 1.0", 0.42, 0.0),
    "omnisim_joint": ("0.129 0.129 0.137 1.0", 0.28, 0.85),
    "omnisim_accent": ("0.965 0.780 0.204 1.0", 0.35, 0.20),
}

# Axis helpers: a URDF cylinder is Z-aligned; rotate it to lie along Y or X.
AXIS_Z = "0 0 0"
AXIS_Y = "1.5708 0 0"
AXIS_X = "0 1.5708 0"


def cyl(r, l, xyz, rpy=AXIS_Z, mat="omnisim_shell"):
    return {"kind": "cylinder", "r": r, "l": l, "xyz": xyz, "rpy": rpy, "mat": mat}


def box(size, xyz, rpy=AXIS_Z, mat="omnisim_shell"):
    return {"kind": "box", "size": size, "xyz": xyz, "rpy": rpy, "mat": mat}


def mesh(path, mat="omnisim_shell"):
    """A generated shell from gen_omniarm_meshes.py, placed in the link frame."""
    return {"kind": "mesh", "path": path, "xyz": "0 0 0", "rpy": AXIS_Z, "mat": mat}


def sph(r, xyz, mat="omnisim_shell"):
    return {"kind": "sphere", "r": r, "xyz": xyz, "rpy": AXIS_Z, "mat": mat}


# --- OmniArm 6 -----------------------------------------------------------
# Generic 6-axis cobot, 800 mm reach.  Skeleton (link frame):
#   j1 Z @ link0 z=0 | j2 Y @ link1 z=0.22 | j3 Y @ link2 z=0.38
#   j4 Z @ link3 z=0 | j5 Y @ link4 z=0.42 | j6 Z @ link5 z=0
#   flange @ link6 z=0.1655
OMNIARM6 = {
    # Smooth surfaces of revolution from scripts/dev/gen_omniarm_meshes.py:
    # tapers and filleted rims, which bare cylinders cannot express. One visual
    # per (link, material) so the palette survives. Collision geometry and every
    # inertial are untouched -- this is the cosmetic layer only.
    "link0": [mesh("meshes/link0_shell.obj"), mesh("meshes/link0_accent.obj", "omnisim_accent")],
    "link1": [mesh("meshes/link1_shell.obj"), mesh("meshes/link1_joint.obj", "omnisim_joint")],
    "link2": [mesh("meshes/link2_shell.obj"), mesh("meshes/link2_accent.obj", "omnisim_accent"),
              mesh("meshes/link2_joint.obj", "omnisim_joint")],
    "link3": [mesh("meshes/link3_shell.obj"), mesh("meshes/link3_accent.obj", "omnisim_accent")],
    "link4": [mesh("meshes/link4_shell.obj"), mesh("meshes/link4_accent.obj", "omnisim_accent"),
              mesh("meshes/link4_joint.obj", "omnisim_joint")],
    "link5": [mesh("meshes/link5_shell.obj"), mesh("meshes/link5_accent.obj", "omnisim_accent")],
    "link6": [mesh("meshes/link6_shell.obj"), mesh("meshes/link6_joint.obj", "omnisim_joint")],
}


# --- OmniArm 7 ----------------------------------------------------------
# Generic 7-axis cobot in the y-along-arm convention.  Skeleton (link frame):
#   j1 Y @ link0 (base rpy +90deg about X, so link1 rises along world Z)
#   j2 X @ link1 y=0.438 | j3 Y @ link2 y=0 | j4 X @ link3 y=0.700
#   j5 Y @ link4 y=0     | j6 X @ link5 y=0.700 | j7 Y @ link6 y=0
#   flange @ link7 y=0.115
OMNIARM7 = {
    "link0": [
        cyl(0.135, 0.040, "0 0 0.020"),
        cyl(0.120, 0.120, "0 0 0.100"),
        cyl(0.104, 0.018, "0 0 0.169", mat="omnisim_accent"),
    ],
    "link1": [
        cyl(0.118, 0.340, "0 0.200 0", rpy=AXIS_Y),
        cyl(0.120, 0.245, "0 0.438 0", rpy=AXIS_X, mat="omnisim_joint"),
    ],
    "link2": [
        cyl(0.100, 0.220, "0 0.015 0", rpy=AXIS_Y),
    ],
    "link3": [
        cyl(0.086, 0.620, "0 0.340 0", rpy=AXIS_Y),
        cyl(0.090, 0.020, "0 0.560 0", rpy=AXIS_Y, mat="omnisim_accent"),
        cyl(0.096, 0.205, "0 0.700 0", rpy=AXIS_X, mat="omnisim_joint"),
    ],
    "link4": [
        cyl(0.082, 0.170, "0 0.015 0", rpy=AXIS_Y),
    ],
    "link5": [
        cyl(0.072, 0.620, "0 0.340 0", rpy=AXIS_Y),
        cyl(0.076, 0.018, "0 0.560 0", rpy=AXIS_Y, mat="omnisim_accent"),
        cyl(0.080, 0.170, "0 0.700 0", rpy=AXIS_X, mat="omnisim_joint"),
    ],
    "link6": [
        cyl(0.066, 0.200, "0 -0.050 0", rpy=AXIS_Y),
    ],
    "link7": [
        cyl(0.056, 0.090, "0 0.050 0", rpy=AXIS_Y),
        cyl(0.048, 0.024, "0 0.103 0", rpy=AXIS_Y, mat="omnisim_joint"),
    ],
}

# --- 140 mm parallel gripper -------------------------------------------
# A generic two-finger adaptive parallel gripper sized to the 2F-140 class:
# ~144 mm from coupling face to finger pivot, fingers on prismatic joints
# along +/-Y with a -0.030..0.070 m travel. The finger COLLISION boxes were
# already hand-authored in this repo and are untouched; these shells simply
# enclose them so the visual matches what the solver sees.
def _finger(sign):
    """sign=+1 for the link whose knuckle sits at +Y, -1 for its mirror."""
    s = sign
    return [
        box("0.024 0.021 0.072", f"0 0 0.0328", mat="omnisim_joint"),
        box("0.030 0.034 0.032", f"0 {0.00825*s:.5f} -0.015", mat="omnisim_joint"),
        box("0.020 0.013 0.058", f"0 {-0.019*s:.5f} -0.038", mat="omnisim_shell"),
        box("0.037 0.023 0.026", f"0 {-0.0465*s:.5f} -0.060", mat="omnisim_accent"),
    ]

GRIPPER140_BASE = [
    cyl(0.0375, 0.012, "0 0 0.006", mat="omnisim_joint"),
    cyl(0.0425, 0.032, "0 0 0.028", mat="omnisim_shell"),
    box("0.100 0.074 0.064", "0 0 0.076", mat="omnisim_shell"),
    box("0.102 0.076 0.008", "0 0 0.104", mat="omnisim_accent"),
    cyl(0.016, 0.116, "0 0 0.1437", rpy=AXIS_Y, mat="omnisim_joint"),
]
GRIPPER140 = {
    "base":  GRIPPER140_BASE,
    "left":  _finger(+1),
    "right": _finger(-1),
}

# --- OmniQuad -----------------------------------------------------------
# Generic 12-DOF quadruped: rectangular chassis, four fore/aft hip drums,
# four thigh plates, four shins ending in a spherical foot pad.
#
# The COLLISION primitives below are not eyeballed: each was fitted to the
# convex hull the MuJoCo solver actually uses (it convexifies every mesh
# collider) by minimising the voxel SYMMETRIC DIFFERENCE against that hull.
# The resulting volumes land within 1.3% of the hull they replace, so the
# contact geometry is preserved rather than merely "looks similar".
# The four foot pads were already primitives and are deliberately untouched.
#
# Skeleton (link frame), per leg L in {front,rear} x {left,right}:
#   hip_x  X @ body   (+-0.29785, +-0.055, 0)
#   hip_y  Y @ hip    (0, +-0.110945, 0)
#   knee   Y @ thigh  (0.025, 0, -0.3205)
#   foot pad          (0, 0, -0.32)


def _oq_hip(xc, ys):
    """One hip housing.  xc = fore/aft centre, ys = +1 left / -1 right."""
    return [
        cyl(0.056, 0.144, f"{xc:+.4f} {0.0060*ys:+.4f} 0.0000",
            rpy=AXIS_X, mat="omnisim_joint"),
        cyl(0.044, 0.056, f"{xc:+.4f} {0.0400*ys:+.4f} 0.0000",
            rpy=AXIS_Y, mat="omnisim_shell"),
        cyl(0.030, 0.012, f"{xc:+.4f} {0.0660*ys:+.4f} 0.0000",
            rpy=AXIS_Y, mat="omnisim_accent"),
    ]


def _oq_thigh(ys):
    """One upper leg.  ys = +1 left / -1 right (the plate is offset in Y)."""
    y = 0.0084 * ys
    return [
        cyl(0.050, 0.116, f"0.0000 {y:+.4f} 0.0000", rpy=AXIS_Y,
            mat="omnisim_joint"),
        box("0.084 0.076 0.290", f"0.0000 {y:+.4f} -0.1750"),
        box("0.088 0.080 0.016", f"0.0000 {y:+.4f} -0.1100",
            mat="omnisim_accent"),
        cyl(0.032, 0.080, f"0.0200 {y:+.4f} -0.3205", rpy=AXIS_Y,
            mat="omnisim_joint"),
    ]


_OQ_SHIN = [
    cyl(0.034, 0.070, "0 0 0", rpy=AXIS_Y, mat="omnisim_joint"),
    cyl(0.026, 0.150, "0 0 -0.0750"),
    cyl(0.028, 0.014, "0 0 -0.1500", mat="omnisim_accent"),
    cyl(0.019, 0.170, "0 0 -0.2350"),
    sph(0.035, "0 0 -0.3200", mat="omnisim_joint"),
]

OMNIQUAD = {
    "body": [
        box("0.700 0.215 0.176", "0.0064 0 -0.0039"),
        box("0.140 0.230 0.150", "0.2980 0 -0.0039", mat="omnisim_joint"),
        box("0.140 0.230 0.150", "-0.2980 0 -0.0039", mat="omnisim_joint"),
        box("0.090 0.160 0.120", "0.3720 0 -0.0039"),
        box("0.090 0.160 0.120", "-0.3590 0 -0.0039"),
        box("0.440 0.130 0.014", "0.0064 0 0.0830", mat="omnisim_accent"),
        cyl(0.028, 0.030, "0.4020 0 0.0300", rpy=AXIS_X,
            mat="omnisim_accent"),
    ],
    "front_left_hip": _oq_hip(-0.0242, +1),
    "front_right_hip": _oq_hip(-0.0242, -1),
    "rear_left_hip": _oq_hip(+0.0240, +1),
    "rear_right_hip": _oq_hip(+0.0240, -1),
    "front_left_upper_leg": _oq_thigh(-1),
    "front_right_upper_leg": _oq_thigh(+1),
    "rear_left_upper_leg": _oq_thigh(-1),
    "rear_right_upper_leg": _oq_thigh(+1),
    "front_left_lower_leg": _OQ_SHIN,
    "front_right_lower_leg": _OQ_SHIN,
    "rear_left_lower_leg": _OQ_SHIN,
    "rear_right_lower_leg": _OQ_SHIN,
}

# Hull-fitted collision primitives.  ratio = primitive volume / hull volume
# of the mesh collider each one replaces (see PROVENANCE.md for the table).


def _oq_hip_col(xc, ys):
    return [cyl(0.0570, 0.1480, f"{xc:+.4f} {0.0118*ys:+.4f} -0.0002",
                rpy=AXIS_X)]


def _oq_thigh_col(ys):
    return [box("0.1063 0.1175 0.3940", f"0.0000 {0.0084*ys:+.4f} -0.1352")]


OMNIQUAD_COLLISION = {
    "body": [box("0.8200 0.2150 0.1880", "0.0064 0.0000 -0.0039")],
    "front_left_hip": _oq_hip_col(-0.0242, +1),
    "front_right_hip": _oq_hip_col(-0.0242, -1),
    "rear_left_hip": _oq_hip_col(+0.0240, +1),
    "rear_right_hip": _oq_hip_col(+0.0240, -1),
    "front_left_upper_leg": _oq_thigh_col(-1),
    "front_right_upper_leg": _oq_thigh_col(+1),
    "rear_left_upper_leg": _oq_thigh_col(-1),
    "rear_right_upper_leg": _oq_thigh_col(+1),
}


SPECS = {
    "omniarm6": {
        "shells": OMNIARM6,
        "urdfs": [
            "projects/robots/omnisim/omniarm6/omniarm6.urdf",
            "projects/robots/omnisim/omniarm6/omniarm6_2f85.urdf",
            "projects/robots/omnisim/omniarm6/omniarm6_2f85_grip.urdf",
            "projects/robots/omnisim/omniarm6/omniarm6_gumgrip.urdf",
            "projects/robots/omnisim/omniarm6/omniarm6_suction.urdf",
            "projects/robots/omnisim/omniarm6/omniarm6_suction_long.urdf",
            "projects/robots/omnisim/omniarm6/omniarm6_2f140_grip.urdf",
        ],
    },
    "omniarm7": {
        "shells": OMNIARM7,
        "urdfs": ["projects/robots/omnisim/omniarm7/urdf/omniarm7.urdf"],
    },
    "omniquad": {
        "shells": OMNIQUAD,
        "collisions": OMNIQUAD_COLLISION,
        "urdfs": [
            "projects/robots/omnisim/omniquad/urdf/omniquad.urdf",
            "projects/robots/omnisim/omniquad/urdf/omniquad.classic.urdf",
            "projects/robots/omnisim/omniquad/urdf/omniquad_bigfoot.urdf",
            "projects/robots/omnisim/omniquad/urdf/omniquad_ghost.urdf",
        ],
    },
    "gripper140": {
        "shells": GRIPPER140,
        "urdfs": [
            # (path, {spec key: actual link name in that file})
            ("projects/robots/robotiq/2f140/urdf/robotiq_2f140.urdf",
             {"base": "base_link", "left": "neg_link", "right": "pos_link"}),
            ("projects/robots/omnisim/omniarm6/omniarm6_2f140_grip.urdf",
             {"base": "robotiq_2f140_base_link",
              "left": "robotiq_2f140_left_finger_link",
              "right": "robotiq_2f140_right_finger_link"}),
        ],
    },
}


def render_visual(v, indent="\t\t"):
    i, i2, i3 = indent, indent + "\t", indent + "\t\t"
    if v["kind"] == "cylinder":
        geom = f'<cylinder radius="{v["r"]}" length="{v["l"]}" />'
    elif v["kind"] == "box":
        geom = f'<box size="{v["size"]}" />'
    elif v["kind"] == "mesh":
        geom = f'<mesh filename="{v["path"]}" />'
    else:
        geom = f'<sphere radius="{v["r"]}" />'
    rgba, rough, metal = MATERIALS[v["mat"]]
    return (
        f'{i}<visual>\n'
        f'{i2}<origin xyz="{v["xyz"]}" rpy="{v["rpy"]}" />\n'
        f'{i2}<geometry>\n'
        f'{i3}{geom}\n'
        f'{i2}</geometry>\n'
        f'{i2}<material name="{v["mat"]}">\n'
        f'{i3}<color rgba="{rgba}" />\n'
        f'{i3}<omnisim roughness="{rough}" metalness="{metal}" />\n'
        f'{i2}</material>\n'
        f'{i}</visual>'
    )

def render_collision(v, indent="\t\t"):
    """A collision block: same primitives, no material, own <origin>."""
    i, i2, i3 = indent, indent + "\t", indent + "\t\t"
    if v["kind"] == "cylinder":
        geom = f'<cylinder radius="{v["r"]}" length="{v["l"]}" />'
    elif v["kind"] == "box":
        geom = f'<box size="{v["size"]}" />'
    else:
        geom = f'<sphere radius="{v["r"]}" />'
    return (
        f'{i}<collision>\n'
        f'{i2}<origin xyz="{v["xyz"]}" rpy="{v["rpy"]}" />\n'
        f'{i2}<geometry>\n'
        f'{i3}{geom}\n'
        f'{i2}</geometry>\n'
        f'{i}</collision>'
    )


def rewrite_collisions(text, collisions, alias=None):
    """Replace each named link's <collision> blocks with authored primitives.

    A link carrying no <collision> at all is skipped silently -- the ghost
    URDF is visual-only by construction, and shares this spec.
    """
    changed = 0
    for key, prims in collisions.items():
        link = (alias or {}).get(key, key)
        pat = re.compile(
            r'(<link\s+name="' + re.escape(link) + r'"\s*>)(.*?)(</link>)',
            re.DOTALL,
        )
        m = pat.search(text)
        if not m:
            raise SystemExit(f"link '{link}' not found")
        body = m.group(2)
        if "<collision" not in body:
            continue
        block = "\n".join(render_collision(v) for v in prims)
        # Eat the block and its own trailing newline only -- NOT the
        # indentation of whatever follows, or the re-inserted block lands
        # against the left margin.
        stripped = re.sub(
            r'[ \t]*<collision>.*?</collision>[ \t]*(?:\r?\n[ \t]*)*\r?\n?',
            "", body, flags=re.DOTALL)
        # Re-insert where the first collision used to be: immediately after
        # the visuals, i.e. before <inertial> when there is one.
        if "<inertial" in stripped:
            idx = stripped.index("<inertial")
            line_start = stripped.rfind("\n", 0, idx) + 1
            stripped = stripped[:line_start] + block + "\n" + stripped[line_start:]
        else:
            stripped = stripped.rstrip() + "\n" + block + "\n\t"
        text = text[: m.start(2)] + stripped + text[m.end(2):]
        changed += 1
    return text, changed


def rewrite(text, shells, alias=None):
    """Replace each named link's <visual> blocks with the authored shells."""
    changed = 0
    for key, visuals in shells.items():
        link = (alias or {}).get(key, key)
        pat = re.compile(
            r'(<link\s+name="' + re.escape(link) + r'"\s*>)(.*?)(</link>)',
            re.DOTALL,
        )
        m = pat.search(text)
        if not m:
            raise SystemExit(f"link '{link}' not found")
        body = m.group(2)
        if "<visual" not in body:
            raise SystemExit(f"link '{link}' has no <visual> to replace")
        block = "\n".join(render_visual(v) for v in visuals)
        # Drop every existing <visual>...</visual>, then insert the shell
        # immediately before the first <collision> (or at the end).
        stripped = re.sub(r'[ \t]*<visual>.*?</visual>[ \t]*\n?', "", body, flags=re.DOTALL)
        if "<collision" in stripped:
            idx = stripped.index("<collision")
            line_start = stripped.rfind("\n", 0, idx) + 1
            stripped = stripped[:line_start] + block + "\n" + stripped[line_start:]
        else:
            stripped = stripped.rstrip() + "\n" + block + "\n\t"
        text = text[: m.start(2)] + stripped + text[m.end(2) :]
        changed += 1
    return text, changed


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", choices=sorted(SPECS))
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--check", action="store_true",
                    help="fail if on-disk files differ from the authored shells")
    args = ap.parse_args()

    names = sorted(SPECS) if args.all else ([args.robot] if args.robot else [])
    if not names:
        ap.error("pass --robot <name> or --all")

    rc = 0
    for name in names:
        spec = SPECS[name]
        for entry in spec["urdfs"]:
            rel, alias = entry if isinstance(entry, tuple) else (entry, None)
            p = REPO / rel
            if not p.exists():
                print(f"SKIP  {rel} (absent)")
                continue
            raw = p.read_bytes()
            nl = "\r\n" if b"\r\n" in raw else "\n"
            src = raw.decode("utf-8").replace("\r\n", "\n")
            out, n = rewrite(src, spec["shells"], alias)
            if spec.get("collisions"):
                out, _ = rewrite_collisions(out, spec["collisions"], alias)
            if args.check:
                if out != src:
                    print(f"DRIFT {rel}")
                    rc = 1
                else:
                    print(f"ok    {rel}")
            else:
                p.write_bytes(out.replace("\n", nl).encode("utf-8"))
                print(f"wrote {rel} ({n} links)")
    return rc


if __name__ == "__main__":
    sys.exit(main())
