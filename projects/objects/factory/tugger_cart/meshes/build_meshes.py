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
"""Build the tugger cart's visual meshes in Blender. Original work, no CAD input.

    blender --background --factory-startup --python build_meshes.py -- [--preview out.png]

One OBJ per material next to this file (tugger_cart_<material>.obj). The
geometry contract is the OmniTug trolley's (omnilink_mobile_bridge): 0.70 m
square, deck top at z 0.325, the tow eye at local (0.70, 0, 0.25) on +X, casters
on the floor at z = 0. Collision stays a plain box in the world file.
"""
import math
import sys
from pathlib import Path

import bpy

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[3] / "robots" / "omnisim" / "omnitug500" / "meshes"))
from build_meshes import box, cyl, prism, rrect  # noqa: E402  (the OmniTug's shape helpers)

S = 0.70                 # deck side
DECK_TOP = 0.325
EYE = (0.70, 0.0, 0.25)
PARTS = {}


def add(mat, obj):
    PARTS.setdefault(mat, []).append(obj)
    return obj


def tube_path(name, pts, r, closed=False):
    """A round tube along a polyline (a bevelled curve, converted to a mesh)."""
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth, cu.bevel_resolution, cu.use_fill_caps = r, 4, True
    sp = cu.splines.new("POLY")
    sp.points.add(len(pts) - 1)
    for p, (x, y, z) in zip(sp.points, pts):
        p.co = (x, y, z, 1)
    sp.use_cyclic_u = closed
    obj = bpy.data.objects.new(name, cu)
    bpy.context.collection.objects.link(obj)
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.convert(target="MESH")
    return bpy.context.active_object


def build():
    # Deck plate (dark steel) and the frame under it (blue powder coat).
    add("deck", prism("deck", rrect(S, S, 0.03), DECK_TOP - 0.03, DECK_TOP, bevel=0.006))
    add("frame", prism("frame", rrect(S - 0.02, S - 0.02, 0.025), DECK_TOP - 0.075, DECK_TOP - 0.03, bevel=0.006))
    add("deck", prism("frame_hole", rrect(S - 0.10, S - 0.10, 0.02), DECK_TOP - 0.074, DECK_TOP - 0.031))
    # Perimeter rail: posts at the corners and mid-sides, one round top rail.
    h, inset = 0.10, 0.03
    a = S / 2 - inset
    for x, y in ((a, a), (-a, a), (-a, -a), (a, -a), (0, a), (0, -a), (a, 0), (-a, 0)):
        add("frame", cyl(f"post{x}{y}", 0.011, h, (x, y, DECK_TOP + h / 2)))
    ring = rrect(2 * a, 2 * a, 0.05, seg=6)
    add("frame", tube_path("rail", [(x, y, DECK_TOP + h) for x, y in ring], 0.013, closed=True))
    # Rubber corner guards on the deck edge.
    for sx in (-1, 1):
        for sy in (-1, 1):
            add("rubber", box(f"guard{sx}{sy}", (0.06, 0.06, 0.05), (sx * (S / 2 - 0.02), sy * (S / 2 - 0.02),
                                                                      DECK_TOP - 0.03), bevel=0.012))
    # Swivel casters: mounting plate, fork, polyurethane wheel, zinc hub.
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * 0.27, sy * 0.27
            add("steel", box(f"cplate{sx}{sy}", (0.08, 0.09, 0.012), (x, y, DECK_TOP - 0.081), bevel=0.003))
            add("steel", cyl(f"swivel{sx}{sy}", 0.022, 0.03, (x, y, DECK_TOP - 0.1)))
            # The cart is towed along its +x axis, so each wheel rolls in x:
            # axle along y, fork plates either side of it in y.
            for side in (-1, 1):
                add("steel", box(f"fork{sx}{sy}{side}", (0.07, 0.008, 0.14), (x, y + side * 0.028, 0.13), bevel=0.003))
            add("wheel", cyl(f"wheel{sx}{sy}", 0.06, 0.04, (x, y, 0.06), axis="Y", bevel=0.008))
            add("steel", cyl(f"hub{sx}{sy}", 0.022, 0.05, (x, y, 0.06), axis="Y"))
            add("steel", cyl(f"axle{sx}{sy}", 0.008, 0.066, (x, y, 0.06), axis="Y"))
    # Tow bar (safety yellow) from under the front edge to the eye at EYE.
    add("accent", box("bar_bracket", (0.08, 0.16, 0.05), (S / 2 - 0.03, 0, DECK_TOP - 0.06), bevel=0.006))
    add("accent", tube_path("bar", [(S / 2 - 0.02, 0, DECK_TOP - 0.065), (EYE[0] - 0.05, 0, EYE[2])], 0.016))
    bpy.ops.mesh.primitive_torus_add(major_radius=0.034, minor_radius=0.011, location=EYE)
    add("steel", bpy.context.active_object)
    # Two reflectors on the rear face.
    for sy in (-1, 1):
        add("reflector", box(f"refl{sy}", (0.006, 0.07, 0.025), (-S / 2 - 0.001, sy * 0.2, DECK_TOP - 0.05)))


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
    for mat, objs in PARTS.items():
        bpy.ops.object.select_all(action="DESELECT")
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        bpy.ops.wm.obj_export(filepath=str(HERE / f"tugger_cart_{mat}.obj"), export_selected_objects=True,
                              forward_axis="Y", up_axis="Z", apply_modifiers=True, export_materials=False,
                              export_uv=False, export_normals=True, export_triangulated_mesh=True)
        print(f"wrote tugger_cart_{mat}.obj ({len(objs)} parts)")


COLOURS = {"deck": (0.22, 0.23, 0.25), "frame": (0.10, 0.27, 0.52), "rubber": (0.05, 0.05, 0.05),
           "steel": (0.66, 0.67, 0.69), "wheel": (0.78, 0.78, 0.76), "accent": (0.965, 0.78, 0.2),
           "reflector": (0.85, 0.12, 0.05)}


def preview(path):
    for mat, objs in PARTS.items():
        m = bpy.data.materials.new(mat)
        m.diffuse_color = (*COLOURS[mat], 1)
        for o in objs:
            o.data.materials.clear()
            o.data.materials.append(m)
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.show_cavity = True
    scene.render.resolution_x, scene.render.resolution_y = 1200, 900
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    cam.location = (1.4, -1.3, 1.0)
    d = [0.1 - 1.4, 0 + 1.3, 0.2 - 1.0]
    cam.rotation_euler = (math.atan2(math.hypot(d[0], d[1]), -d[2]), 0, math.atan2(d[1], d[0]) - math.pi / 2)
    cam.data.lens = 38
    scene.camera = cam
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    bpy.ops.wm.read_factory_settings(use_empty=True)
    build()
    finish()
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if "--preview" in argv:
        preview(argv[argv.index("--preview") + 1])
