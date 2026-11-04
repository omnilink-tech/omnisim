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

"""Generate an OmniSim world for a 3-axis gantry from a machine config.

Layout is a moving gantry (Y gantry on side rails, X carriage on the beam, Z
on the carriage), which covers routers, plasma tables, lasers and plotters by
motion. A moving-bed machine such as a 3018 has the same axis motion with a
different visual. Geometry is sized from each axis's rail length; nothing is
taken from CAD.

Each joint's zero is the carriage's power-up position (`s0`), so the authored
endpoint pose and the joint agree at load. The `grbl_twin` controller maps
rail positions s (mm) to joint positions (s - s0) / 1000 m.

    python world.py --machine ../machines/example_3018/example_3018_with_switches.grbl \
        --out ../../samples/demos/worlds/manufacturing/cnc_3018_twin.omniworld
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Dict, Optional, Sequence

HERE = Path(__file__).resolve().parent
for _p in (HERE, HERE.parent / "core"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import machine as machine_mod  # noqa: E402
from machine import AXES, Machine  # noqa: E402
import tool_table as tools_mod  # noqa: E402
from twin import END_STOP_MARGIN_MM  # noqa: E402
from worldkit import box as _box, cyl as _cyl, look_at, num as _f  # noqa: E402

ROOT = HERE.parents[2]
SKIN_MM = 0.2          # painted top skin above the stock core (see _stock)
# The axis drives. OmniSim builds a position servo with ke = 10 x maxForce (N/m) and
# kd = 0.5 x maxForce, so the force only sets STIFFNESS; the 0.05 s ke/kd time constant is
# fixed and the controller feeds it forward (grbl_twin SERVO_TAU_S). A lead-screw stepper
# drive is effectively rigid: at 600 N the 12.5 kg LongMill gantry trailed or overshot by up
# to 4.4 mm (measured 2026-11-01) and the carved walls showed it; at 20 kN the remaining
# acceleration error is mass x accel / ke, about 0.05 mm.
SERVO_FORCE_N = 20000
# The window draws at most this many frames per second. A frame that has not changed is
# already held (render-on-change), so this only bites while the machine moves: measured
# 2026-11-01 on the LongMill star job, a 60 fps window kept the RTX 3060 Laptop GPU
# ~40 % busy, almost all of it shading the machine. Physics still runs every 8 ms.
RENDER_FPS = 30


def build_world(m: Machine, machine_ref: str, s0: Optional[Sequence[float]] = None,
                rails: Optional[Dict[str, float]] = None, port: int = 2323) -> str:
    lengths = {a: float((rails or {}).get(a, m.axes[a].travel)) for a in AXES}
    s0 = list(s0) if s0 is not None else [lengths[a] / 2 for a in AXES]
    tx, ty, tz = (lengths[a] / 1000.0 for a in AXES)
    sx, sy, sz = (v / 1000.0 for v in s0)
    margin = END_STOP_MARGIN_MM / 1000.0
    rail_x = tx / 2 + 0.06            # side rails sit outside the X travel
    bed_top = 0.10
    beam_z = bed_top + tz + 0.12
    tool_len = 0.03                   # the tool tip touches the bed top at s_z = 0
    dark, steel, alu, accent = ".12 .13 .15", ".55 .57 .60", ".78 .79 .80", ".95 .55 .12"

    def slider(axis_vec, name, s_init, length, max_rate, endpoint):
        lo = -s_init / 1000.0 - margin
        hi = (length - s_init) / 1000.0 + margin
        vmax = max_rate / 60.0 / 1000.0 * 2.0
        return (f"SliderJoint {{ jointParameters JointParameters {{ axis {axis_vec} }}\n"
                f"  device [ LinearMotor {{ name \"{name}_motor\" minPosition {_f(lo)} maxPosition {_f(hi)} "
                f"maxForce {SERVO_FORCE_N} maxVelocity {_f(vmax)} }} PositionSensor {{ name \"{name}_sensor\" }} ]\n"
                f"  endPoint {endpoint} }}")

    # The spindle frame's origin is the tool tip, at world z = bed_top + s_z.
    spindle = (f"DEF SPINDLE Solid {{ name \"spindle\" translation 0 -0.09 {_f(bed_top + sz - beam_z)}\n"
               f"  children [ {_cyl((0, 0, 0.06), 0.026, 0.09, alu)} {_cyl((0, 0, tool_len / 2), 0.003, tool_len, steel, 0.9)}\n"
               f"    {_cyl((0, 0, 0.012), 0.01, 0.012, dark)}\n"
               f"    {_box((0, 0.03, 0.08), (0.07, 0.008, 0.14), accent, 0.2, 0.45)} ]\n"
               f"  boundingObject Pose {{ translation 0 0 0.06 children [ Box {{ size 0.05 0.05 0.09 }} ] }}\n"
               f"  physics Physics {{ density -1 mass 0.5 }} }}")
    z_joint = slider("0 0 1", "z", s0[2], lengths["z"], m.axes["z"].max_rate, spindle)
    carriage = (f"DEF CARRIAGE Solid {{ name \"carriage\" translation {_f(-tx / 2 + sx)} 0 0\n"
                f"  children [ {_box((0, -0.045, 0), (0.09, 0.03, 0.12), accent, 0.2, 0.45)}\n    {z_joint} ]\n"
                f"  boundingObject Pose {{ translation 0 -0.045 0 children [ Box {{ size 0.09 0.03 0.12 }} ] }}\n"
                f"  physics Physics {{ density -1 mass 0.8 }} }}")
    x_joint = slider("1 0 0", "x", s0[0], lengths["x"], m.axes["x"].max_rate, carriage)
    upright_h = beam_z - bed_top + 0.03
    gantry = (f"DEF GANTRY Solid {{ name \"gantry\" translation 0 {_f(-ty / 2 + sy)} {_f(beam_z)}\n"
              f"  children [ {_box((0, 0, 0), (2 * rail_x + 0.04, 0.05, 0.06), dark, 0.3, 0.4)}\n"
              f"    {_box((-rail_x, 0, -upright_h / 2), (0.03, 0.08, upright_h), dark, 0.3, 0.4)}\n"
              f"    {_box((rail_x, 0, -upright_h / 2), (0.03, 0.08, upright_h), dark, 0.3, 0.4)}\n"
              f"    {x_joint} ]\n"
              f"  boundingObject Box {{ size {_f(2 * rail_x + 0.04)} 0.05 0.06 }}\n"
              f"  physics Physics {{ density -1 mass 2.5 }} }}")
    y_joint = slider("0 1 0", "y", s0[1], lengths["y"], m.axes["y"].max_rate, gantry)

    bed_w, bed_d = tx + 0.10, ty + 0.12
    frame = [
        _box((0, 0, bed_top - 0.015), (bed_w, bed_d, 0.03), ".42 .33 .24", 0.0, 0.85),        # spoilboard
        _box((0, 0, bed_top - 0.05), (2 * rail_x + 0.06, ty + 0.2, 0.04), dark, 0.3, 0.5),   # frame
        _box((-rail_x, 0, bed_top + 0.01), (0.02, ty + 0.2, 0.02), steel, 0.8, 0.3),         # rails
        _box((rail_x, 0, bed_top + 0.01), (0.02, ty + 0.2, 0.02), steel, 0.8, 0.3),
    ]
    for i in range(4):
        x = (-1) ** i * (rail_x - 0.01)
        y = (-1) ** (i // 2) * (ty / 2 + 0.08)
        frame.append(_box((x, y, (bed_top - 0.07) / 2), (0.04, 0.04, bed_top - 0.07), dark))
    # Visual-only stock block, centred; no boundingObject (it is not a collider).
    stock = (f"Solid {{ name \"stock\" translation 0 0 {_f(bed_top + 0.006)} children [ "
             f"{_box((0, 0, 0), (min(0.12, tx * 0.4), min(0.08, ty * 0.4), 0.012), '.80 .68 .48', 0.0, 0.8)} ] }}")

    reach = max(tx, ty)
    eye = (reach * 1.15 + 0.25, -reach * 1.45 - 0.3, beam_z + reach * 0.9 + 0.2)
    target = (0, 0, bed_top + 0.05)
    s0_arg = ",".join(_f(v) for v in s0)
    args = ['"--machine"', f'"{machine_ref}"', '"--port"', f'"{port}"', '"--s0"', f'"{s0_arg}"']
    if rails:
        args += ['"--rails"', '"' + ",".join(_f(lengths[a]) for a in AXES) + '"']
    lines = [
        "#OMNISIM R2025a utf8",
        'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
        f"# Generated by projects/manufacturing/cnc/world.py from {machine_ref}. Regenerate; do not hand-edit.",
        f'WorldInfo {{ basicTimeStep 8 title "OmniSim CNC twin - {m.name}" newtonSolver "mujoco" newtonSubsteps 4 FPS {RENDER_FPS} }}',
        f"Viewpoint {{ position {' '.join(_f(v) for v in eye)} orientation {look_at(eye, target)} fieldOfView .7 }}",
        "OmniSimSky { }",
        "DEF SUN OmniSimSun { }",
        "DEF SUN_MARKER OmniSimSunMarker { }",
        'DEF FLOOR Solid { name "floor" translation 0 0 -.05 children [ Shape { appearance PBRAppearance '
        '{ baseColor .30 .31 .32 roughness .9 metalness 0 } geometry Box { size 4 4 .1 } } ] boundingObject Box { size 4 4 .1 } }',
        stock,
        f'DEF CNC Robot {{ name "cnc-twin" controller "grbl_twin" controllerArgs [ {" ".join(args)} ]',
        "  children [ " + "\n    ".join(frame),
        "    " + y_joint + " ] }",
        "",
    ]
    return "\n".join(lines)


def build_pack_world(m: Machine, machine_ref: str, pack: dict, mesh_prefix: str,
                     s0: Optional[Sequence[float]] = None, rails: Optional[Dict[str, float]] = None,
                     port: int = 2323, stock: Optional[dict] = None,
                     table: Optional[Dict[int, "tools_mod.Tool"]] = None,
                     laser: Optional[dict] = None) -> str:
    """A twin world from a CAD mesh pack (`cad_pack.py`) instead of schematic boxes.

    `laser` (projects/manufacturing/laser/world.py) fits a laser instead of the cutter:
    {"head": VRML drawn on the Z body in place of the tools, "node": the sheet's
    MillingStock, "args": its controller args, "centre": the camera target, "title"}.

    Every body frame coincides with the pack frame at the CAD snapshot's carriage
    positions, so a body's meshes sit at identity and its Solid is offset along
    its axis by (power-up position - CAD position). Colliders are 2 cm tokens at
    each body's centre (Newton needs a shape to register the body); the machine
    does not collide with itself or the stock.
    """
    lengths = {a: float((rails or {}).get(a, m.axes[a].travel)) for a in AXES}
    s0 = list(s0) if s0 is not None else [lengths[a] / 2 for a in AXES]
    margin = END_STOP_MARGIN_MM / 1000.0
    cad = pack["cad_pose_s"]
    masses = {"gantry": 3.0, "carriage": 1.0, "z": 1.5, "bed": 3.0, **pack.get("masses_kg", {})}

    def shapes(group: str) -> str:
        out = []
        for e in pack["groups"][group]["meshes"]:
            r, g, b, a = e["rgba"]
            transparency = f" transparency {_f(1 - a)}" if a < 0.999 else ""
            out.append(f'Shape {{ appearance PBRAppearance {{ baseColor {_f(r)} {_f(g)} {_f(b)} metalness 0.3 '
                       f'roughness 0.5{transparency} }} geometry Mesh {{ url "{mesh_prefix}/{e["file"]}" }} }}')
        return "\n    ".join(out)

    def body(spec: dict) -> str:
        group, axis = spec["group"], spec["axis"]
        i = AXES.index(axis)
        d = pack["axis_dirs"][axis]
        delta = (s0[i] - cad[i]) / 1000.0
        lo, hi = pack["groups"][group]["bbox"]
        centre = [(lo[k] + hi[k]) / 2 for k in range(3)]
        children = [shapes(group)] + [body(c) for c in pack["bodies"] if c["parent"] == group]
        if axis == "z" and laser is not None:
            children.append(laser["head"])
        elif axis == "z" and pack.get("collet") and pack.get("tool_tip"):
            # The pack left the CAD's own bit out: draw the collet and the table's tools.
            children.append(_tool_nodes(pack, table or tools_mod.default_tools(
                float((stock or {}).get("tool_d", 6.35)))))
        solid = (f'DEF {group.upper()} Solid {{ name "{group}" translation {" ".join(_f(c * delta) for c in d)}\n'
                 f"  children [ " + "\n    ".join(children) + " ]\n"
                 f"  boundingObject Pose {{ translation {' '.join(_f(c) for c in centre)} children [ Box {{ size 0.02 0.02 0.02 }} ] }}\n"
                 f"  physics Physics {{ density -1 mass {_f(masses.get(group, 1.0))} "
                 f"centerOfMass [ {' '.join(_f(c) for c in centre)} ] }} }}")
        lo_lim = -s0[i] / 1000.0 - margin
        hi_lim = (lengths[axis] - s0[i]) / 1000.0 + margin
        vmax = m.axes[axis].max_rate / 60.0 / 1000.0 * 2.0
        return (f"SliderJoint {{ jointParameters JointParameters {{ axis {' '.join(_f(c) for c in d)} }}\n"
                f'  device [ LinearMotor {{ name "{axis}_motor" minPosition {_f(lo_lim)} maxPosition {_f(hi_lim)} '
                f'maxForce {SERVO_FORCE_N} maxVelocity {_f(vmax)} }} PositionSensor {{ name "{axis}_sensor" }} ]\n'
                f"  endPoint {solid} }}")

    roots = [body(c) for c in pack["bodies"] if c["parent"] == "frame"]
    ex, ey, ez = pack["extent_m"]
    reach = max(ex, ey, ez)
    eye = (reach * 1.1 + 0.2, -reach * 1.4 - 0.2, ez + reach * 0.8)
    target = (0, 0, ez * 0.45)
    s0_arg = ",".join(_f(v) for v in s0)
    args = ['"--machine"', f'"{machine_ref}"', '"--port"', f'"{port}"', '"--s0"', f'"{s0_arg}"']
    if rails:
        args += ['"--rails"', '"' + ",".join(_f(lengths[a]) for a in AXES) + '"']
    stock_node = ""
    if stock or laser:
        stock_node, stock_args, stock_centre = (laser["node"], laser["args"], laser["centre"]) if laser else             _stock(pack, s0, stock)
        args += stock_args
        # Frame the work, not the whole machine: the part is what a milling demo is about.
        target = stock_centre
        eye = (target[0] + 0.32, target[1] - 0.46, target[2] + 0.34)
    src = pack["source"]
    lines = [
        "#OMNISIM R2025a utf8",
        'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
        f"# Generated by projects/manufacturing/cnc/world.py from {machine_ref} and the "
        f"{pack['name']} mesh pack. Regenerate; do not hand-edit.",
        f"# Geometry: {src.get('attribution', '')} ({src.get('licence', 'licence not stated')}) {src.get('url', '')}",
        f'WorldInfo {{ basicTimeStep 8 title "{(laser or {}).get("title") or "OmniSim CNC twin - " + pack["name"]}" '
        f'newtonSolver "mujoco" newtonSubsteps 4 FPS {RENDER_FPS} }}',
        f"Viewpoint {{ position {' '.join(_f(v) for v in eye)} orientation {look_at(eye, target)} fieldOfView .7 }}",
        "OmniSimSky { }",
        "DEF SUN OmniSimSun { }",
        "DEF SUN_MARKER OmniSimSunMarker { }",
        'DEF FLOOR Solid { name "floor" translation 0 0 -.05 children [ Shape { appearance PBRAppearance '
        '{ baseColor .30 .31 .32 roughness .9 metalness 0 } geometry Box { size 6 6 .1 } } ] boundingObject Box { size 6 6 .1 } }',
        stock_node,
        f'DEF CNC Robot {{ name "cnc-twin" controller "grbl_twin" controllerArgs [ {" ".join(args)} ]'
        + (" supervisor TRUE" if stock or laser else ""),
        "  children [ " + shapes("frame"),
        "    " + "\n    ".join(roots) + " ] }",
        "",
    ]
    return "\n".join(line for i, line in enumerate(lines) if line or i == len(lines) - 1)


# ---- the tools you see --------------------------------------------------------------
# A tool is drawn as a lathe mesh: its cutting length carries two helical flutes, so its
# turning shows; above that, a plain shank up into the collet. Visual only: what cuts is
# MillingStock's cutter, which grbl_twin sets from the same tool table.
HIDDEN_RISE_M = 0.12     # a tool not in the spindle waits this far up, inside the router body
_RING = 32               # points around a ring


def _lathe(profile: Sequence[tuple], flute_top_mm: float = 0.0, diameter_mm: float = 0.0,
           flutes: int = 2) -> tuple:
    """Points (m) and faces of a surface of revolution about +Z through the origin.

    `profile` is (z_mm, r_mm) from the bottom up; the first and last rings are capped.
    Below `flute_top_mm` the radius is cut by `flutes` helical grooves (30-degree helix).
    """
    lead = math.pi * diameter_mm / math.tan(math.radians(30.0)) if diameter_mm else 1.0
    apex = profile[0][0]
    profile = [(z, r) for z, r in profile if r > 1e-6]     # a pointed end is the bottom centre
    pts, faces = [], []
    for z, r in profile:
        twist = 2 * math.pi * z / lead
        for k in range(_RING):
            t = 2 * math.pi * k / _RING
            rr = r
            if z < flute_top_mm and r > 0:
                groove = max(0.0, math.cos(flutes * (t - twist))) ** 6
                rr = r * (1.0 - 0.3 * groove)
            pts.append((rr * math.cos(t) / 1000.0, rr * math.sin(t) / 1000.0, z / 1000.0))
    rings = len(profile)
    for j in range(rings - 1):
        for k in range(_RING):
            a, b = j * _RING + k, j * _RING + (k + 1) % _RING
            # Triangles: a fluted quad is not planar.
            faces += [(a, b, b + _RING), (a, b + _RING, a + _RING)]
    bottom, top = len(pts), len(pts) + 1
    pts += [(0.0, 0.0, apex / 1000.0), (0.0, 0.0, profile[-1][0] / 1000.0)]
    last = (rings - 1) * _RING
    for k in range(_RING):
        faces.append((bottom, (k + 1) % _RING, k))
        faces.append((top, last + k, last + (k + 1) % _RING))
    return pts, faces


def _ifs(pts, faces, colour: str, metalness: float, roughness: float, transparency: float = 0.0) -> str:
    coords = " ".join(f"{_f(x)} {_f(y)} {_f(z)}" for x, y, z in pts)
    index = " ".join(" ".join(str(i) for i in f) + " -1" for f in faces)
    clear = f" transparency {_f(transparency)}" if transparency else ""
    return (f"Shape {{ appearance PBRAppearance {{ baseColor {colour} metalness {metalness} roughness {roughness}{clear} }} "
            f"geometry IndexedFaceSet {{ coord Coordinate {{ point [ {coords} ] }} coordIndex [ {index} ] "
            f"creaseAngle 0.9 }} castShadows FALSE }}")


def tool_profile(tool: "tools_mod.Tool", length_mm: float) -> list:
    """(z, r) from the tip (z 0) up `length_mm`, in mm."""
    R = tool.radius_mm
    shank = (tool.shank_mm or tool.diameter_mm) / 2.0
    if tool.shape == "vbit":
        tip_h = R / math.tan(math.radians(max(1.0, min(179.0, tool.angle_deg))) / 2.0)
        cut = max(tool.flute_mm, tip_h)
        prof = [(0.0, 0.0)] + [(tip_h * f, R * f) for f in (0.25, 0.5, 0.75, 1.0)]
        prof += [(tip_h + (cut - tip_h) * f, R) for f in (0.5, 1.0) if cut > tip_h]
    elif tool.shape == "ball":
        cut = tool.flute_mm or 3 * tool.diameter_mm
        prof = [(R - R * math.cos(a), R * math.sin(a)) for a in [math.pi / 2 * f / 8 for f in range(9)]]
        n = max(2, int(math.ceil((cut - R) / 1.5)))
        prof += [(R + (cut - R) * f / n, R) for f in range(1, n + 1)]
    else:
        cut = tool.flute_mm or 3 * tool.diameter_mm
        n = max(2, int(math.ceil(cut / 1.5)))
        prof = [(0.0, R)] + [(cut * f / n, R) for f in range(1, n + 1)]
    prof.append((cut + 0.4, shank))
    prof.append((max(length_mm, cut + 1.0), shank))
    return prof


def _tool_nodes(pack: dict, table: Dict[int, "tools_mod.Tool"]) -> str:
    """The collet and every tool of the table, in the tool body's frame (= the pack frame).

    Each tool hangs `stickout_mm` lower than tool 1, up into the collet. Tool 1 shows at
    power-up; grbl_twin swaps them and spins TOOL_SPIN while the spindle runs.
    """
    c, tip = pack["collet"], pack["tool_tip"]
    rgba = c.get("rgba", [0.22, 0.22, 0.22, 1.0])
    nose_mm = (c["nose_z_m"] - tip[2]) * 1000.0
    collet = _lathe([((c["nose_z_m"] - tip[2]) * 1000.0, c["nose_radius_m"] * 1000.0 + 0.3),
                     ((c["body_z_m"] - tip[2]) * 1000.0, c["radius_m"] * 1000.0),
                     ((c["top_z_m"] - tip[2]) * 1000.0, c["radius_m"] * 1000.0)])
    tools, blurs = [], []
    for n in sorted(table):
        t = table[n]
        flute_top = t.flute_mm or 3 * t.diameter_mm
        profile = tool_profile(t, nose_mm + t.stickout_mm + 2.0)
        pts, faces = _lathe(profile, flute_top_mm=flute_top, diameter_mm=t.diameter_mm, flutes=max(1, t.flutes))
        z = -t.stickout_mm / 1000.0 + (0.0 if n == 1 else HIDDEN_RISE_M)
        tools.append(f"DEF TOOL_{n} Pose {{ translation 0 0 {_f(z)} "
                     f"children [ {_ifs(pts, faces, '.80 .81 .83', 0.6, 0.3)} ] }}")
        # The blur a spinning cutter shows the eye: a still, translucent sleeve over its flutes.
        # grbl_twin shows it while the spindle runs, so a machine standing still with the
        # spindle on can hold its frames instead of redrawing every frame to turn the bit.
        sleeve = [(zz, r * 1.12 + 0.1) for zz, r in profile if zz <= flute_top + 0.5]
        blurs.append(f"DEF BLUR_{n} Pose {{ translation 0 0 {_f(-t.stickout_mm / 1000.0 + HIDDEN_RISE_M)} "
                     f"children [ {_ifs(*_lathe(sleeve), '.92 .93 .95', 0.1, 0.5, transparency=0.5)} ] }}")
    at = " ".join(_f(v) for v in tip)
    return (f"DEF COLLET Pose {{ translation {at} children [ "
            f"{_ifs(*collet, ' '.join(_f(v) for v in rgba[:3]), 0.6, 0.35)}\n      "
            + "\n      ".join(blurs) + " ] }\n    "
            f"DEF TOOL_SPIN Pose {{ translation {at} rotation 0 0 1 0 children [\n      "
            + "\n      ".join(tools) + " ] }")


def _stock(pack: dict, s0: Sequence[float], stock: dict):
    """The stock block (ElevationGrid the controller carves) and the controller args it needs.

    `stock`: {"size": [x, y, thickness] mm, "spacing": mm, "tool_d": mm, "centre": [x, y] mm (optional)}.
    By default the block is centred under the tool at power-up, on the spoilboard.
    """
    if pack.get("tool_tip") is None or pack.get("bed_top_m") is None:
        raise ValueError("a stock block needs the pack's tool_tip and bed_top_m")
    sx, sy, thick = (float(v) for v in stock["size"])
    spacing = float(stock.get("spacing", 1.0))
    cad = pack["cad_pose_s"]
    dirs = [pack["axis_dirs"][a] for a in AXES]
    tip_cad = [v * 1000.0 for v in pack["tool_tip"]]
    tip_s0 = [tip_cad[k] + sum(dirs[a][k] * (s0[a] - cad[a]) for a in range(3)) for k in range(3)]
    cx, cy = stock.get("centre") or tip_s0[:2]
    origin = [cx - sx / 2, cy - sy / 2, pack["bed_top_m"] * 1000.0]
    if stock.get("renderer", "native") == "native":
        node = _native_stock(pack, origin, sx, sy, thick, stock)
    else:
        node = _grid_stock(origin, sx, sy, thick, spacing)
    # `--key=value`: a value starting with "-" (a negative coordinate) is otherwise read
    # by argparse as an option of its own.
    args = ['"--stock=' + ",".join(_f(v) for v in origin + [sx, sy, thick, spacing]) + '"',
            f'"--tool-d={_f(float(stock.get("tool_d", 6.35)))}"',
            *([f'"--tools={stock["tools"]}"'] if stock.get("tools") else []),
            *([f'"--material={stock["material"]}"'] if stock.get("material") else []),
            '"--tool-tip=' + ",".join(_f(v) for v in tip_cad) + '"',
            '"--cad-s=' + ",".join(_f(v) for v in cad) + '"',
            '"--axis-dirs=' + ";".join(",".join(_f(c) for c in d) for d in dirs) + '"']
    if stock.get("bridge_port", 8765):
        # The OmniLink bridge, with the work origin on the stock's top centre (the tool
        # powers up over it, tip_s0[2] - stock top above it).
        above = tip_s0[2] - (origin[2] + thick)
        args += [f'"--bridge-port={int(stock.get("bridge_port", 8765))}"', f'"--zero-above-stock={_f(above)}"']
    centre = (cx / 1000.0, cy / 1000.0, (origin[2] + thick) / 1000.0)
    return node, args, centre


# MillingStock colours by material (the node's defaults are a painted wood block). Metal:
# a duller mill-finish top over a brighter cut, so every cut still shows.
STOCK_LOOK = {
    "aluminium_alloy": " coreColor .86 .87 .89 skinColor .58 .6 .63 roughness .35",
    "brass": " coreColor .88 .72 .38 skinColor .6 .48 .25 roughness .35",
}


def _native_stock(pack: dict, origin, sx: float, sy: float, thick: float, stock: dict) -> str:
    """The engine's MillingStock node: carved in the engine every physics step, streamed by tile.

    The cutter is the Solid that carries the tool group (the bodies are named after their
    groups); its tip, in that Solid's own frame, is the pack's tool tip, because every body
    frame coincides with the pack frame at the CAD pose.
    """
    tool_group = stock.get("tool_group", "z")
    tip = pack["tool_tip"]
    look = STOCK_LOOK.get(stock.get("material") or "", "")
    return (f'DEF STOCK MillingStock {{ translation {" ".join(_f(v / 1000.0) for v in origin)} '
            f'size {_f(sx / 1000.0)} {_f(sy / 1000.0)} {_f(thick / 1000.0)} '
            f'resolution {_f(float(stock.get("resolution_mm", 0.2)) / 1000.0)}{look}\n'
            f'  toolSolid "{tool_group}" toolTip {" ".join(_f(v) for v in tip)} '
            f'toolShape "{stock.get("tool_shape", "flat")}" '
            f'toolDiameter {_f(float(stock.get("tool_d", 6.35)) / 1000.0)} }}')


def _grid_stock(origin, sx: float, sy: float, thick: float, spacing: float) -> str:
    """Fallback for an engine without MillingStock: ElevationGrids the controller edits."""
    nx, ny = int(round(sx / spacing)) + 1, int(round(sy / spacing)) + 1
    top = _f(thick / 1000.0)
    heights = []
    for j in range(ny):
        row = ["0" if j in (0, ny - 1) or i in (0, nx - 1) else top for i in range(nx)]
        heights.append(" ".join(row))
    # A painted top skin, 0.2 mm above the core. The controller drops a skin vertex to 0
    # (inside the core, out of sight) once the tool has cut there, so every cut shows the
    # light core against the dark top: a flat-colour pocket is nearly invisible under sky
    # light (measured 2026-10-31).
    skin = " ".join([_f((thick + SKIN_MM) / 1000.0)] * (nx * ny))
    grid = (f"ElevationGrid {{ xDimension {nx} yDimension {ny} "
            f"xSpacing {_f(spacing / 1000.0)} ySpacing {_f(spacing / 1000.0)}\n      height [\n")
    return (f'DEF STOCK Solid {{ name "stock" translation {" ".join(_f(v / 1000.0) for v in origin)}\n'
            f"  children [ Shape {{ appearance PBRAppearance {{ baseColor .80 .64 .44 roughness .85 metalness 0 }}\n"
            f"    geometry DEF STOCK_GRID {grid}" + "\n".join(heights) + " ] } }\n"
            f"    Shape {{ appearance PBRAppearance {{ baseColor .10 .14 .20 roughness .6 metalness 0 }}\n"
            f"    geometry DEF STOCK_SKIN {grid}{skin} ] }} }} ] }}")


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Generate a CNC twin world from a machine config.")
    p.add_argument("--machine", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--pack", help="mesh pack directory from cad_pack.py (default: schematic boxes)")
    p.add_argument("--s0", help="power-up carriage position on each rail, mm (default: mid-rail)")
    p.add_argument("--rails", help="as-built rail lengths x,y,z in mm, if they differ from $130-$132")
    p.add_argument("--port", type=int, default=2323)
    p.add_argument("--stock", help="stock block x,y,thickness in mm, carved live (pack worlds only)")
    p.add_argument("--stock-spacing", type=float, default=1.0,
                   help="the controller's own accounting height map, mm (and the grid fallback's)")
    p.add_argument("--stock-renderer", choices=("native", "grid"), default="native",
                   help="native: the engine's MillingStock node; grid: supervisor-edited ElevationGrids")
    p.add_argument("--stock-resolution", type=float, default=0.2, help="MillingStock height-map spacing, mm")
    p.add_argument("--tool-d", type=float, default=6.35, help="cutter diameter, mm")
    p.add_argument("--tool-shape", choices=("flat", "ball", "vbit"), default="flat")
    p.add_argument("--tools", help="tool table JSON (tool_table.py), repo-relative; tool 1 is loaded at power-up")
    p.add_argument("--material", help="stock material for the cutting-load model (cutting.py MATERIALS)")
    p.add_argument("--bridge-port", type=int, default=8765,
                   help="OmniLink CNC bridge HTTP port when a stock is given (0: no bridge)")
    args = p.parse_args(argv)
    m = machine_mod.load(args.machine)
    machine_path = Path(args.machine).resolve()
    try:
        ref = machine_path.relative_to(ROOT).as_posix()
    except ValueError:
        ref = machine_path.as_posix()
    s0 = [float(v) for v in args.s0.split(",")] if args.s0 else None
    rails = dict(zip(AXES, (float(v) for v in args.rails.split(",")))) if args.rails else None
    out = Path(args.out)
    if args.pack:
        pack_dir = Path(args.pack).resolve()
        pack = json.loads((pack_dir / "pack.json").read_text(encoding="utf-8"))
        prefix = Path(os.path.relpath(pack_dir, out.resolve().parent)).as_posix()
        stock = None
        if args.stock:
            stock = {"size": [float(v) for v in args.stock.split(",")], "spacing": args.stock_spacing,
                     "tool_d": args.tool_d, "tool_shape": args.tool_shape,
                     "renderer": args.stock_renderer, "resolution_mm": args.stock_resolution,
                     "bridge_port": args.bridge_port, "material": args.material}
            if args.tools:
                stock["tools"] = Path(args.tools).resolve().relative_to(ROOT).as_posix()
                first = tools_mod.load(Path(args.tools))[1]
                stock["tool_d"], stock["tool_shape"] = first.diameter_mm, first.shape
        table = tools_mod.load(Path(args.tools)) if args.tools else None
        text = build_pack_world(m, ref, pack, prefix, s0, rails, args.port, stock, table)
    else:
        text = build_world(m, ref, s0, rails, args.port)
    out.write_text(text, encoding="utf-8", newline="\n")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
