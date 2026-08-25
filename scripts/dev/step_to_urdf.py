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

"""step_to_urdf.py - convert a STEP/STP CAD assembly into an OmniSim URDF robot.

Given a `.step`/`.stp` file it:

  1. tessellates the CAD with cascadio (OpenCASCADE) -> a glTF/GLB,
  2. groups the triangles by CAD colour, exporting one mesh per colour,
  3. applies an orientation (which CAD axis is "up") and recenters,
  4. writes a URDF whose single `base_link` has one `<visual>` per colour
     group (so the OmniSim importer emits one PBRAppearance per part and the
     original CAD colours survive), plus `omnisim.yaml`,
  5. prints a report: bounding box, an orientation sanity check, the per-part
     colours, and a ready-to-paste `URDFRobot { ... }` world snippet.

The link carries no <collision>/<inertial>, so the importer emits no Physics
and no boundingObject -> a static, kinematic-only prop you can place in a world
or drive from a supervisor (see docs/developer/step-to-urdf.md).

Dependency (one-time):  python -m pip install cascadio trimesh

Examples
--------
  # A wheeled rover whose CAD is already Z-up, sitting on the ground (wheels at
  # the bottom -> rest the lowest point on z=0):
  python scripts/dev/step_to_urdf.py 3d-models/acme_rover.step \
      projects/robots/acme/rover --name rover --up z --ground bottom

  # A part whose CAD up-axis is Y (common in glTF exporters), shown as a
  # centered display prop:
  python scripts/dev/step_to_urdf.py widget.step projects/robots/acme/widget \
      --name widget --up y --ground center

IMPORTANT - verify the orientation. The single hardest thing to get right is
which CAD axis is "up". This tool will not guess; pass --up and then VERIFY by
eye (the report flags it if "height" is not your smallest extent). The robust
check is to load the generated world and render the model from all six axis
directions, then confirm the wheels / feet / base are on the bottom. See the
"Orientation" section of docs/developer/step-to-urdf.md.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path


def _eprint(*a):
    print(*a, file=sys.stderr, flush=True)


def _import_deps():
    try:
        import cascadio  # noqa: F401
        import numpy as np  # noqa: F401
        import trimesh  # noqa: F401
    except ImportError as e:
        _eprint(f"[step_to_urdf] missing dependency: {e}")
        _eprint("[step_to_urdf] install with:  python -m pip install cascadio trimesh")
        sys.exit(2)


def _up_rotation(up: str):
    """Return a 4x4 transform mapping the chosen source axis to +Z (OmniSim up)."""
    import trimesh
    table = {
        "z": (0.0, [0, 0, 1]),          # already Z-up -> identity
        "-z": (math.pi, [1, 0, 0]),     # flip
        "y": (math.pi / 2, [1, 0, 0]),  # +Y -> +Z
        "-y": (-math.pi / 2, [1, 0, 0]),
        "x": (-math.pi / 2, [0, 1, 0]),  # +X -> +Z
        "-x": (math.pi / 2, [0, 1, 0]),
    }
    if up not in table:
        _eprint(f"[step_to_urdf] --up must be one of {sorted(table)}")
        sys.exit(2)
    ang, axis = table[up]
    return trimesh.transformations.rotation_matrix(ang, axis)


def _color_key(rgba):
    return tuple(int(round(c)) for c in rgba)


# Link groups a --split-axis run can produce, in emission order.
_LINK_ORDER = ("base", "neg", "pos")


def _side_of(mesh, axis, deadband):
    """Which link group a placed solid belongs to, by the sign of its centroid.

    A CAD assembly of a two-jaw mechanism is mirror-symmetric about one axis:
    everything on the -side moves with one jaw, everything on the +side with
    the other, and everything straddling the plane is the fixed body. That is
    enough to cut a single-body tessellation into the three links a real
    gripper needs, WITHOUT hand-editing meshes. `deadband` is the half-width of
    the "body" slab, in metres, applied to the solid's bbox centre.
    """
    if axis is None:
        return "base"
    i = "xyz".index(axis)
    c = 0.5 * (mesh.bounds[0][i] + mesh.bounds[1][i])
    if abs(c) <= deadband:
        return "base"
    return "neg" if c < 0 else "pos"


def _friendly_name(rgba, used):
    r, g, b = (c / 255.0 for c in rgba[:3])
    if r > 0.9 and g > 0.9 and b > 0.9:
        base = "white"
    elif r > 0.7 and g < 0.3 and b < 0.3:
        base = "red"
    elif b > 0.7 and r < 0.3 and g < 0.3:
        base = "blue"
    elif g > 0.7 and r < 0.4 and b < 0.4:
        base = "green"
    elif max(r, g, b) < 0.15:
        base = "black"
    elif min(r, g, b) > 0.4 and abs(r - g) < 0.12 and abs(g - b) < 0.12:
        base = "grey"
    else:
        base = "part"
    used[base] = used.get(base, 0) + 1
    return base if used[base] == 1 else f"{base}_{used[base]}"


def convert(args) -> int:
    import cascadio
    import numpy as np
    import trimesh

    src = Path(args.input)
    if not src.is_file():
        _eprint(f"[step_to_urdf] input not found: {src}")
        return 2
    out_dir = Path(args.output)
    name = args.name or src.stem.lower().replace(" ", "_")
    meshes_dir = out_dir / "meshes"
    urdf_dir = out_dir / "urdf"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    urdf_dir.mkdir(parents=True, exist_ok=True)

    glb = out_dir / f"_{name}_cad.glb"
    print(f"[step_to_urdf] tessellating {src.name} (tol={args.tol}) ...", flush=True)
    cascadio.step_to_glb(str(src), str(glb), float(args.tol), float(args.angular_tol))
    scene = trimesh.load(str(glb))
    if not isinstance(scene, trimesh.Scene):
        scene = trimesh.Scene(scene)
    print(f"[step_to_urdf]   {len(scene.geometry)} CAD geometr(ies); units={scene.units}")

    R = _up_rotation(args.up)

    # ⚠ APPLY THE SCENE-GRAPH TRANSFORMS. `scene.geometry` hands back each mesh
    # in its own LOCAL frame; the assembly's placements live in `scene.graph`.
    # Iterating the geometry dict therefore collapses a CAD assembly onto the
    # origin, and -- worse -- an INSTANCED part (the same solid placed twice
    # with different transforms, which is how every mirrored mechanism is
    # authored) is emitted once, on top of itself, so half the model silently
    # vanishes. MEASURED on 3d_models/2F-140_Assy_Open_20191022.STEP: the
    # geometry-only extent is 0.148 x 0.211 x 0.075 m (ONE finger), the
    # graph-applied extent 0.207 x 0.213 x 0.075 m (both, symmetric about x=0).
    # `nodes_geometry` yields one node per PLACEMENT, so instances survive.
    placements = [(scene.graph[n][1], scene.graph[n][0])
                  for n in scene.graph.nodes_geometry]
    if not placements:                      # degenerate GLB: no graph at all
        placements = [(k, None) for k in scene.geometry]

    # Group geometries by CAD colour (fewer, cleaner visuals).
    buckets: dict = {}
    for gname, T in placements:
        geo = scene.geometry.get(gname)
        if geo is None:
            continue
        m = geo.copy()
        if T is not None:
            m.apply_transform(T)
        m.apply_transform(R)
        mat = getattr(m.visual, "material", None)
        col = getattr(mat, "baseColorFactor", None) if mat is not None else None
        if col is None:
            col = [200, 200, 200, 255]
        col = [float(c) for c in col]
        if max(col) <= 1.0:  # some exporters give 0..1
            col = [c * 255.0 for c in col]
        side = _side_of(m, args.split_axis, args.split_deadband)
        # ⚠ keyed on the PLACEMENT, not id(geo): two instances of one solid are
        # two parts, and id(geo) would merge them back into one.
        key = (side, _color_key(col)) if args.group_by_color else (side, len(buckets))
        if key not in buckets:
            buckets[key] = {"rgba": col, "side": side, "meshes": []}
        buckets[key]["meshes"].append(m)

    groups = []
    allmin = np.array([np.inf] * 3)
    allmax = np.array([-np.inf] * 3)
    for b in buckets.values():
        mesh = trimesh.util.concatenate(b["meshes"]) if len(b["meshes"]) > 1 else b["meshes"][0]
        allmin = np.minimum(allmin, mesh.bounds[0])
        allmax = np.maximum(allmax, mesh.bounds[1])
        groups.append({"mesh": mesh, "rgba": b["rgba"], "side": b["side"]})
    groups.sort(key=lambda g: _LINK_ORDER.index(g["side"]))

    # Recenter: X,Y centered; Z either bottom->0 (sits on ground) or centered.
    if args.ground == "bottom":
        offset = np.array([-(allmin[0] + allmax[0]) / 2,
                           -(allmin[1] + allmax[1]) / 2,
                           -allmin[2]])
    else:  # center
        offset = -(allmin + allmax) / 2.0
    ext = allmax - allmin

    used: dict = {}
    parts = []
    for i, g in enumerate(groups):
        m = g["mesh"]
        m.apply_translation(offset)
        tag = "" if args.split_axis is None else f"_{g['side']}"
        fn = f"{name}{tag}_part{i}.stl"
        m.export(str(meshes_dir / fn))
        rgba01 = [round(c / 255.0, 4) for c in g["rgba"]]
        nm = _friendly_name(g["rgba"], used)
        parts.append({"file": fn, "rgba": rgba01, "mat": nm, "side": g["side"],
                      "faces": int(len(m.faces)),
                      "bbox_min": [round(float(v), 5) for v in m.bounds[0]],
                      "bbox_max": [round(float(v), 5) for v in m.bounds[1]],
                      "zmin": float(m.bounds[0][2]), "zmax": float(m.bounds[1][2])})

    # --- write URDF -------------------------------------------------------
    def _visual(p, ind):
        return f'''{ind}<visual>
{ind}  <origin xyz="0 0 0" rpy="0 0 0"/>
{ind}  <geometry>
{ind}    <mesh filename="../meshes/{p['file']}"/>
{ind}  </geometry>
{ind}  <material name="{name}_{p['mat']}">
{ind}    <color rgba="{p['rgba'][0]} {p['rgba'][1]} {p['rgba'][2]} {p['rgba'][3]}"/>
{ind}  </material>
{ind}</visual>'''

    if args.split_axis is None:
        visuals = "\n".join(_visual(p, "    ") for p in parts)
        body = f"""  <link name="base_link">
{visuals}
  </link>"""
    else:
        # One link per side, joined by FIXED joints: the emitted robot is still
        # a static prop (this tool does not know a mechanism's joint axes or
        # limits), but the meshes are already cut so a hand-authored URDF can
        # hang each side off a real joint. That is the whole point of the flag.
        chunks = []
        for side in _LINK_ORDER:
            sp = [p for p in parts if p["side"] == side]
            if not sp:
                continue
            link = "base_link" if side == "base" else f"{side}_link"
            chunks.append(f"""  <link name="{link}">
{chr(10).join(_visual(p, '    ') for p in sp)}
  </link>""")
            if side != "base":
                chunks.append(f"""  <joint name="{side}_fixed" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="{link}"/>
  </joint>""")
        body = "\n".join(chunks)
    ground_note = ("lowest point rests at mesh z=0 (place at world translation "
                   "z=0 to sit on the floor)") if args.ground == "bottom" else \
                  "bounding-box centre at the origin"
    urdf = f'''<?xml version="1.0"?>
<!--
  {name} - visual URDF generated from {src.name} by scripts/dev/step_to_urdf.py.

  Tessellated with cascadio (OpenCASCADE) -> trimesh; {len(parts)} CAD colour
  group(s), each its own <visual>+<material> so the original colours survive
  the OmniSim import (one PBRAppearance per visual). No <collision>/<inertial>
  -> a static, kinematic-only prop (drive it from a supervisor if you want it
  to move). Metres, Z-up, {ground_note}.
  Footprint (X x Y x Z): {ext[0]:.3f} x {ext[1]:.3f} x {ext[2]:.3f} m.
-->
<robot name="{name}">
{body}
</robot>
'''
    (urdf_dir / f"{name}.urdf").write_text(urdf, encoding="utf-8", newline="\n")
    yaml_path = out_dir / "omnisim.yaml"
    if not yaml_path.exists():
        yaml_path.write_text("publish: true\n", encoding="utf-8", newline="\n")

    if not args.keep_glb:
        try:
            glb.unlink()
        except OSError:
            pass

    # --- report -----------------------------------------------------------
    smallest = "XYZ"[int(np.argmin(ext))]
    print()
    print("=" * 64)
    print(f"  wrote {urdf_dir / (name + '.urdf')}")
    print(f"  wrote {len(parts)} mesh(es) under {meshes_dir}")
    print(f"  footprint  W(X)={ext[0]:.3f}  D/L(Y)={ext[1]:.3f}  H(Z)={ext[2]:.3f} m")
    print(f"  smallest axis = {smallest}"
          + ("  (Z = height is smallest -> object lies flat/low, typical for a"
             " grounded robot)" if smallest == "Z"
             else f"  (!) height(Z) is NOT the smallest extent - if this should"
                  f" sit flat, the --up axis is probably wrong; you may be"
                  f" standing it on its face."))
    print("  per colour group:")
    for i, p in enumerate(parts):
        floor = " <- touches z=0" if p["zmin"] < 0.02 else ""
        side = "" if args.split_axis is None else f" link={p['side']:<4s}"
        print(f"    part{i}: {p['file']:28s} rgba={p['rgba']} "
              f"faces={p['faces']:>7d}{side}{floor}")
    print()
    print("  paste into a .wbt (canonical lighting per docs/WORLD_RECIPE.md):")
    rel = os.path.relpath(urdf_dir / f"{name}.urdf",
                          out_dir / "worlds").replace("\\", "/")
    print(f"""    DEF {name.upper()} URDFRobot {{
      url "{rel}"
      translation 0 0 {0.0 if args.ground == 'bottom' else round(ext[2] / 2, 3)}
      name "{name}"
      controller "<none>"
    }}""")
    print()
    print("  NEXT: VERIFY ORIENTATION. Render the model from all six axes and")
    print("  confirm wheels/feet/base are on the bottom. If not, re-run with a")
    print("  different --up. See docs/developer/step-to-urdf.md.")
    print("=" * 64)

    if args.report_json:
        Path(args.report_json).write_text(json.dumps(
            {"name": name, "extents": ext.tolist(), "ground": args.ground,
             "up": args.up, "parts": parts}, indent=2), encoding="utf-8")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Convert a STEP CAD assembly into an OmniSim URDF robot.")
    ap.add_argument("input", help="path to the .step/.stp file")
    ap.add_argument("output", help="robot project dir to write (meshes/, urdf/, omnisim.yaml)")
    ap.add_argument("--name", default="", help="robot name (default: from filename)")
    ap.add_argument("--up", default="z", choices=["x", "-x", "y", "-y", "z", "-z"],
                    help="which CAD axis points up (mapped to OmniSim +Z). Default z. "
                         "VERIFY by rendering 6 views - this is the usual mistake.")
    ap.add_argument("--ground", default="bottom", choices=["bottom", "center"],
                    help="bottom: lowest point at z=0 (sits on floor, default). "
                         "center: bbox centre at origin (display prop).")
    ap.add_argument("--group-by-color", dest="group_by_color", action="store_true",
                    default=True, help="merge same-colour geometries (default on)")
    ap.add_argument("--no-group-by-color", dest="group_by_color", action="store_false",
                    help="one mesh per CAD solid (can be many)")
    ap.add_argument("--tol", default=0.5, type=float,
                    help="linear tessellation tolerance in CAD units (default 0.5)")
    ap.add_argument("--angular-tol", default=0.5, type=float,
                    help="angular tessellation tolerance (default 0.5)")
    ap.add_argument("--split-axis", default=None, choices=["x", "y", "z"],
                    help="cut the assembly into three links by the sign of each "
                         "solid's centroid on this OUTPUT-frame axis: base (within "
                         "--split-deadband of the plane), neg, pos. For a "
                         "mirror-symmetric two-jaw mechanism this separates the "
                         "body from the two moving jaws so a hand-authored URDF "
                         "can hang each on a real joint. Default: one link.")
    ap.add_argument("--split-deadband", default=0.005, type=float,
                    help="half-width in metres of the 'base' slab for --split-axis "
                         "(default 0.005)")
    ap.add_argument("--keep-glb", action="store_true", help="keep the intermediate GLB")
    ap.add_argument("--report-json", default="", help="also write a JSON report here")
    args = ap.parse_args(argv)

    _import_deps()
    return convert(args)


if __name__ == "__main__":
    raise SystemExit(main())
