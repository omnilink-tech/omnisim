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
"""Build kitting-cell props in Blender. Original work, no CAD input.

    blender --background --factory-startup --python build_meshes.py -- [--preview out.png]

Writes, next to this file, one OBJ per (prop, material):
  tote_<mat>.obj              open plastic tote, 0.32 x 0.42 x 0.16, origin at
                              the bottom centre; inner floor at +0.02
  bench_<name>_<mat>.obj      steel workbench, origin at the floor centre, top
                              surface at its stated height (BENCHES below)
  part.obj                    a 50 mm machined block (one material; the world
                              gives each part its colour)
Visual only: worlds keep their own simple colliders, sized to match.
"""
import math
import sys
from pathlib import Path

import bmesh
import bpy

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[3] / "robots" / "omnisim" / "omnitug500" / "meshes"))
from build_meshes import box, cyl, prism, rrect  # noqa: E402  (the OmniTug's shape helpers)

# name: (size x, size y, top height)
BENCHES = {"shelf": (0.26, 0.46, 0.40), "stand": (0.38, 0.48, 0.30)}
PARTS = {}


def add(group, obj):
    PARTS.setdefault(group, []).append(obj)
    return obj


def taper(obj, factor, below):
    """Scale every vertex under z=below towards the axis by `factor` (draft angle)."""
    for v in obj.data.vertices:
        if v.co.z < below:
            v.co.x *= factor
            v.co.y *= factor


def boolean(target, cutter):
    m = target.modifiers.new("cut", "BOOLEAN")
    m.operation, m.object, m.solver = "DIFFERENCE", cutter, "EXACT"
    bpy.ops.object.select_all(action="DESELECT")
    target.select_set(True)
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier="cut")
    bpy.data.objects.remove(cutter, do_unlink=True)


def tote():
    lx, ly, h = 0.32, 0.42, 0.16
    outer = prism("tote", rrect(lx, ly, 0.035), 0.0, h)
    taper(outer, 0.93, 0.001)
    inner = prism("tote_in", rrect(lx - 0.024, ly - 0.024, 0.025), 0.02, h + 0.05)
    taper(inner, 0.93, 0.021)
    boolean(outer, inner)
    add("tote_plastic", outer)
    rim = prism("rim", rrect(lx + 0.02, ly + 0.02, 0.04), h - 0.014, h)
    rim_in = prism("rim_in", rrect(lx - 0.02, ly - 0.02, 0.025), h - 0.05, h + 0.05)
    boolean(rim, rim_in)
    add("tote_plastic", rim)
    # Vertical ribs on the long sides, handle slots on the short ends.
    for sy in (-1, 1):
        for k in range(-2, 3):
            add("tote_plastic", box(f"rib{sy}{k}", (0.012, 0.010, h - 0.03),
                                    (k * 0.06, sy * (ly / 2 - 0.004), h / 2 - 0.005), bevel=0.003))
    for sx in (-1, 1):
        add("tote_dark", box(f"handle{sx}", (0.006, 0.11, 0.03), (sx * (lx / 2 + 0.004), 0, h - 0.04), bevel=0.008))
        add("tote_dark", box(f"label{sx}", (0.004, 0.14, 0.05), (sx * (lx / 2 - 0.004), 0, 0.06)))


def bench(name, sx, sy, top):
    t, leg = 0.03, 0.035
    add(f"bench_{name}_steel", prism(f"{name}_top", rrect(sx, sy, 0.012), top - t, top, bevel=0.004))
    add(f"bench_{name}_mat", prism(f"{name}_mat", rrect(sx - 0.03, sy - 0.03, 0.01), top - 0.002, top + 0.002))
    add(f"bench_{name}_frame", prism(f"{name}_apron", rrect(sx - 0.03, sy - 0.03, 0.01), top - t - 0.05, top - t,
                                     bevel=0.003))
    add(f"bench_{name}_steel", prism(f"{name}_apron_in", rrect(sx - 0.07, sy - 0.07, 0.01), top - t - 0.049, top - t))
    for lx in (-1, 1):
        for ly in (-1, 1):
            x, y = lx * (sx / 2 - leg / 2 - 0.012), ly * (sy / 2 - leg / 2 - 0.012)
            add(f"bench_{name}_frame", box(f"{name}_leg{lx}{ly}", (leg, leg, top - t - 0.02),
                                           (x, y, (top - t) / 2 + 0.01), bevel=0.004))
            add(f"bench_{name}_feet", cyl(f"{name}_foot{lx}{ly}", 0.026, 0.02, (x, y, 0.01), bevel=0.004))
    add(f"bench_{name}_steel", prism(f"{name}_shelf", rrect(sx - 0.02, sy - 0.02, 0.008), 0.11, 0.125, bevel=0.003))


def part():
    obj = add("part", box("part", (0.05, 0.05, 0.05), (0, 0, 0), bevel=0.005))
    hole = cyl("bore", 0.008, 0.06, (0, 0, 0.02), seg=24)
    boolean(obj, hole)


def finish():
    for objs in PARTS.values():
        for o in objs:
            bpy.ops.object.select_all(action="DESELECT")
            o.select_set(True)
            bpy.context.view_layer.objects.active = o
            try:
                bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
            except Exception:
                bpy.ops.object.shade_smooth()
    for group, objs in PARTS.items():
        bpy.ops.object.select_all(action="DESELECT")
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        bpy.ops.wm.obj_export(filepath=str(HERE / f"{group}.obj"), export_selected_objects=True,
                              forward_axis="Y", up_axis="Z", apply_modifiers=True, export_materials=False,
                              export_uv=False, export_normals=True, export_triangulated_mesh=True)
        print(f"wrote {group}.obj ({len(objs)} parts)")


def preview(path):
    colours = {"tote_plastic": (0.08, 0.30, 0.62), "tote_dark": (0.03, 0.10, 0.22), "part": (0.85, 0.15, 0.1)}
    for group, objs in PARTS.items():
        c = colours.get(group) or {"steel": (0.6, 0.62, 0.64), "mat": (0.05, 0.05, 0.05), "frame": (0.10, 0.27, 0.52),
                                   "feet": (0.05, 0.05, 0.05)}[group.rsplit("_", 1)[1]]
        m = bpy.data.materials.new(group)
        m.diffuse_color = (*c, 1)
        for o in objs:
            o.data.materials.clear()
            o.data.materials.append(m)
    # Lay the props out side by side for the picture.
    offsets = {"tote": (0.9, 0, 0.30), "bench_shelf": (0, 0, 0), "bench_stand": (0.9, 0, 0), "part": (0, 0, 0.425)}
    for group, objs in PARTS.items():
        key = next(k for k in offsets if group.startswith(k))
        for o in objs:
            o.location = [a + b for a, b in zip(o.location, offsets[key])]
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.show_cavity = True
    scene.render.resolution_x, scene.render.resolution_y = 1200, 900
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    cam.location = (1.6, -1.5, 1.3)
    d = [0.45 - 1.6, 0 + 1.5, 0.25 - 1.3]
    cam.rotation_euler = (math.atan2(math.hypot(d[0], d[1]), -d[2]), 0, math.atan2(d[1], d[0]) - math.pi / 2)
    cam.data.lens = 40
    scene.camera = cam
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    bpy.ops.wm.read_factory_settings(use_empty=True)
    tote()
    for name, (sx, sy, top) in BENCHES.items():
        bench(name, sx, sy, top)
    part()
    finish()
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if "--preview" in argv:
        preview(argv[argv.index("--preview") + 1])
