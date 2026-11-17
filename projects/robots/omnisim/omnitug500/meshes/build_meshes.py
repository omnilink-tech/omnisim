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
"""Build the OmniTug 500's visual meshes in Blender. Original work, no CAD input.

    blender --background --factory-startup --python build_meshes.py -- [--preview out.png]

Writes one OBJ per material next to this file (omnitug500_<material>.obj); the
URDF gives each its own PBR material. Frame: +Y is FORWARD (the bridge, the
tug's controllers and every OmniTug world drive it along local +Y), +Z up,
wheel contact plane at z = 0. Envelope kept from the primitive model it
replaces: 1.259 m long (Y), 0.716 m wide (X), deck top 0.296 m. The tow pin
sits on the centre line 0.63 m behind the centre (the bridge's rear_offset_m).
"""
import math
import sys
from pathlib import Path

import bmesh
import bpy

HERE = Path(__file__).resolve().parent
L, Wd = 1.259, 0.716          # length (Y), width (X)
DECK_TOP = 0.296
REAR_PIN_Y = -0.63

PARTS = {}                    # material -> [objects]


def add(mat, obj):
    PARTS.setdefault(mat, []).append(obj)
    return obj


def rrect(sx, sy, r, seg=8):
    """Rounded-rectangle outline (x, y) points, counter-clockwise."""
    pts = []
    for cx, cy, a0 in ((sx / 2 - r, sy / 2 - r, 0), (-sx / 2 + r, sy / 2 - r, 90),
                       (-sx / 2 + r, -sy / 2 + r, 180), (sx / 2 - r, -sy / 2 + r, 270)):
        for k in range(seg + 1):
            a = math.radians(a0 + 90 * k / seg)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def prism(name, outline, z0, z1, bevel=0.0, bevel_seg=3, at=(0, 0, 0)):
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bottom = [bm.verts.new((x + at[0], y + at[1], z0 + at[2])) for x, y in outline]
    face = bm.faces.new(bottom)
    ext = bmesh.ops.extrude_face_region(bm, geom=[face])
    top = [v for v in ext["geom"] if isinstance(v, bmesh.types.BMVert)]
    bmesh.ops.translate(bm, vec=(0, 0, z1 - z0), verts=top)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(obj)
    if bevel:
        m = obj.modifiers.new("bevel", "BEVEL")
        m.width, m.segments, m.limit_method, m.angle_limit = bevel, bevel_seg, "ANGLE", math.radians(40)
    return obj


def box(name, size, at, bevel=0.0):
    sx, sy, sz = size
    return prism(name, [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)],
                 at[2] - sz / 2, at[2] + sz / 2, bevel=bevel, at=(at[0], at[1], 0))


def cyl(name, r, h, at, axis="Z", seg=40, bevel=0.0):
    bpy.ops.mesh.primitive_cylinder_add(vertices=seg, radius=r, depth=h, location=at)
    obj = bpy.context.active_object
    obj.name = name
    if axis == "X":
        obj.rotation_euler = (0, math.pi / 2, 0)
    elif axis == "Y":
        obj.rotation_euler = (math.pi / 2, 0, 0)
    if bevel:
        m = obj.modifiers.new("bevel", "BEVEL")
        m.width, m.segments, m.limit_method = bevel, 3, "ANGLE"
    return obj


def text(name, body, size, at, rot):
    bpy.ops.object.text_add(location=at, rotation=rot)
    obj = bpy.context.active_object
    obj.name = name
    obj.data.body = body
    obj.data.size = size
    obj.data.extrude = 0.0015
    obj.data.align_x, obj.data.align_y = "CENTER", "CENTER"
    bpy.ops.object.convert(target="MESH")
    return obj


def build():
    # ── body ──────────────────────────────────────────────────────────
    # Lower skirt (charcoal), inset from the shell, carries the drive bays.
    add("chassis", prism("skirt", rrect(Wd - 0.05, L - 0.08, 0.07), 0.035, 0.115, bevel=0.01))
    # Shell (warm white), large corner radius, soft top edge.
    add("shell", prism("shell", rrect(Wd, L - 0.03, 0.09), 0.112, 0.262, bevel=0.022, bevel_seg=4))
    # Deck mat (dark rubber) inset into the shell top, with raised grip ribs.
    add("rubber", prism("deck", rrect(Wd - 0.09, L - 0.17, 0.05), 0.258, 0.272, bevel=0.004))
    for k in range(-7, 8):
        add("rubber", box(f"rib{k}", (Wd - 0.15, 0.012, 0.006), (0, k * 0.068, 0.274), bevel=0.002))
    # Yellow accent band around the shell's lower edge, and corner caps.
    add("accent", prism("band", rrect(Wd + 0.006, L - 0.024, 0.092), 0.118, 0.142, bevel=0.004))
    # Status light strip (cyan), front and both flanks, just under the band.
    add("light", prism("lightring", rrect(Wd - 0.004, L - 0.034, 0.089), 0.094, 0.113))
    # ── bumpers (black rubber), front and rear, wrapping the corners ─────
    for sgn, nm in ((1, "front"), (-1, "rear")):
        y = sgn * (L / 2 - 0.035)
        add("rubber", box(f"bumper_{nm}", (Wd - 0.03, 0.07, 0.085), (0, y, 0.08), bevel=0.02))
    # ── front fascia: dark glass window with a stereo camera pair ─────
    add("glass", box("fascia", (0.36, 0.012, 0.07), (0, L / 2 - 0.012, 0.19), bevel=0.006))
    for dx in (-0.09, 0.09):
        add("steel", cyl(f"cam{dx}", 0.016, 0.01, (dx, L / 2 - 0.004, 0.19), axis="Y"))
    # ── safety lidars in the front-left and rear-right corner notches ──
    for (sx, sy), nm in (((-1, 1), "fl"), ((1, -1), "rr")):
        x, y = sx * (Wd / 2 - 0.06), sy * (L / 2 - 0.07)
        add("chassis", cyl(f"lidar_{nm}", 0.042, 0.06, (x, y, 0.07), bevel=0.004))
        add("glass", cyl(f"lidar_win_{nm}", 0.043, 0.022, (x, y, 0.072)))
    # ── e-stops on the two rear corners of the deck ───────────────────
    for sx in (-1, 1):
        x, y = sx * (Wd / 2 - 0.07), -(L / 2 - 0.07)
        add("accent", box(f"estop_base{sx}", (0.075, 0.075, 0.03), (x, y, 0.281), bevel=0.008))
        add("estop", cyl(f"estop{sx}", 0.026, 0.022, (x, y, 0.305), bevel=0.008))
    # ── drive wheels in side bays, and four corner casters ────────────
    for sx in (-1, 1):
        x = sx * (Wd / 2 - 0.07)
        add("rubber", cyl(f"wheel{sx}", 0.09, 0.06, (x, 0, 0.09), axis="X", bevel=0.01))
        add("steel", cyl(f"hub{sx}", 0.045, 0.064, (x, 0, 0.09), axis="X"))
    for sx in (-1, 1):
        for sy in (-1, 1):
            add("rubber", cyl(f"caster{sx}{sy}", 0.035, 0.03, (sx * 0.24, sy * 0.45, 0.035), axis="X"))
    # ── tow pin (steel) on the rear centre line at REAR_PIN_Y ─────────
    add("steel", box("pin_plate", (0.16, 0.10, 0.02), (0, REAR_PIN_Y + 0.05, 0.275), bevel=0.006))
    add("steel", cyl("pin", 0.016, 0.11, (0, REAR_PIN_Y + 0.03, 0.27), bevel=0.003))
    add("accent", cyl("pin_cap", 0.024, 0.016, (0, REAR_PIN_Y + 0.03, 0.33), bevel=0.004))
    # ── name on both flanks ───────────────────────────────────────────
    for sx in (-1, 1):
        add("chassis", text(f"name{sx}", "OMNITUG 500", 0.05, (sx * (Wd / 2 + 0.0008), 0.05, 0.19),
                            (math.pi / 2, 0, sx * math.pi / 2)))


def export():
    for mat, objs in PARTS.items():
        bpy.ops.object.select_all(action="DESELECT")
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        bpy.ops.wm.obj_export(filepath=str(HERE / f"omnitug500_{mat}.obj"), export_selected_objects=True,
                              forward_axis="Y", up_axis="Z", apply_modifiers=True, export_materials=False,
                              export_uv=False, export_normals=True, export_triangulated_mesh=True)
        print(f"wrote omnitug500_{mat}.obj ({len(objs)} parts)")


def smooth():
    for objs in PARTS.values():
        for o in objs:
            bpy.ops.object.select_all(action="DESELECT")
            o.select_set(True)
            bpy.context.view_layer.objects.active = o
            try:
                bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
            except Exception:
                bpy.ops.object.shade_smooth()


COLOURS = {"shell": (0.90, 0.89, 0.87), "chassis": (0.13, 0.13, 0.14), "rubber": (0.05, 0.05, 0.05),
           "accent": (0.965, 0.78, 0.2), "light": (0.35, 0.9, 1.0), "glass": (0.02, 0.03, 0.04),
           "steel": (0.62, 0.64, 0.66), "estop": (0.78, 0.05, 0.04)}


def preview(path):
    for mat, objs in PARTS.items():
        m = bpy.data.materials.new(mat)
        m.diffuse_color = (*COLOURS[mat], 1)
        for o in objs:
            o.data.materials.append(m)
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.show_cavity = True
    scene.render.resolution_x, scene.render.resolution_y = 1400, 900
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    cam.location = (1.6, 1.7, 1.05)
    d = [0 - 1.6, 0 - 1.7, 0.15 - 1.05]
    cam.rotation_euler = (math.atan2(math.hypot(d[0], d[1]), -d[2]), 0, math.atan2(d[1], d[0]) - math.pi / 2)
    cam.data.lens = 40
    scene.camera = cam
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)


if __name__ == "__main__":
    bpy.ops.wm.read_factory_settings(use_empty=True)
    build()
    smooth()
    export()
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if "--preview" in argv:
        preview(argv[argv.index("--preview") + 1])
