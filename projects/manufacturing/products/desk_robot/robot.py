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

"""The desk robot as a robot: its parts assembled into an OmniSim world that drives on Newton.

Every made part is drawn from its own mesh (`meshes/`, written by this file from
design.py's shapes: the plates as cut, the printed parts as sliced, the hubs as milled
in the drawing), every bought part at its datasheet size, and the mass and its centre come
from the bill of materials, part by part. Collisions are what touches the world: the two
tyres and the caster ball. The ball is a frictionless sphere fixed to the chassis, which
is what a ball caster is to the robot's motion; its own rolling is not modelled.

The motors are the Pololu #3065's straight-line DC characteristic, applied by the
controller every step: available torque = stall torque x (1 - speed / no-load speed),
1.6 kg cm at stall, nothing at 330 rpm.

    python projects/manufacturing/products/desk_robot/robot.py
        # writes meshes/ and projects/samples/demos/worlds/manufacturing/desk_robot_drive_test.omniworld
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

import bought_parts
import design
from design import BOUGHT, Robot

HERE = Path(__file__).resolve().parent
ROOT = design.ROOT
MESHES = HERE / "meshes"
WORLD = ROOT / "projects/samples/demos/worlds/manufacturing/desk_robot_drive_test.omniworld"
G = 9.81


def _f(v: float) -> str:
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def _v(*vals: float) -> str:
    return " ".join(_f(v) for v in vals)


def torus_triangles(R: float, r: float, n: int = 96, m: int = 16) -> np.ndarray:
    """A torus about z (mm): the O-ring tyre."""
    tris = []
    for i in range(n):
        for j in range(m):
            def p(a, b):
                u, w = 2 * math.pi * a / n, 2 * math.pi * b / m
                return ((R + r * math.cos(w)) * math.cos(u), (R + r * math.cos(w)) * math.sin(u), r * math.sin(w))
            a, b, c, d = p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1)
            tris += [[a, b, c], [a, c, d]]
    return np.asarray(tris)


def write_meshes(r: Robot, out: Path = MESHES) -> dict:
    """The made parts as metre-unit STLs, in the frames robot.py places them in."""
    out.mkdir(parents=True, exist_ok=True)
    parts = {
        "bottom_plate": design.part_triangles([(0.0, r.plate_t, design.bottom_plate(r))]),
        "top_plate": design.part_triangles([(0.0, r.plate_t, design.top_plate(r))]),
        "engraving": design.part_triangles([(r.plate_t, r.plate_t + 0.05, design.name_engraving(r))]),
        "motor_clamp": design.part_triangles(design.motor_clamp(r)),
        "rim": design.part_triangles(design.rim(r, step=0.25)),
        "tray": design.part_triangles(design.tray(r)),
        "hub": design.part_triangles([(0.0, r.hub_t, design.hub_outline(r))]),
        "tyre": torus_triangles(r.groove_root_d / 2 + r.tyre_cs_mm / 2, r.tyre_cs_mm / 2),
    }
    for name, tris in parts.items():
        design.write_stl(out / f"{name}.stl", tris * 0.001, f"desk robot {name}")
    if out == MESHES:
        bought_parts.write_drawn()           # the bought parts' drawn models, beside them
    return {k: f"{k}.stl" for k in parts}


def mass_items(r: Robot) -> List[Tuple[str, float, Tuple[float, float, float]]]:
    """(what, grams, centre in the robot frame, mm) for everything on the chassis body."""
    made = {p["part"]: p for p in design.made_parts(r)}
    m = BOUGHT["gearmotor"]
    top0 = r.plate_top_z + r.standoff_len
    items = [("bottom_plate", made["bottom_plate"]["g"], (0, 0, r.plate_top_z - r.plate_t / 2)),
             ("top_plate", made["top_plate"]["g"], (0, 0, top0 + r.plate_t / 2))]
    for j in (-1, 1):
        items.append(("gearmotor", m["g"], (r.axle_x, j * (r.faceplate_y - m["body_mm"][2] / 2), r.axle_z)))
        items.append(("motor_clamp", made["motor_clamp"]["g"], (r.axle_x, j * r.clamp_y, r.plate_top_z + 6)))
    sx, sy = r.standoff_xy
    for i in (-1, 1):
        for j in (-1, 1):
            items.append(("standoff", BOUGHT["standoff"]["g"], (i * sx, j * sy, r.plate_top_z + r.standoff_len / 2)))
    items += [("caster", BOUGHT["caster"]["g"], (r.caster_x, 0, 7.0)),
              ("tray", made["tray"]["g"], (r.tray_x, 0, r.plate_top_z + 2.5)),
              ("controller", BOUGHT["controller"]["g"], (r.tray_x - 11.65, 0, r.plate_top_z + 3)),
              ("driver", BOUGHT["driver"]["g"], (r.tray_x + 11.9, 0, r.plate_top_z + 3)),
              ("battery_holder", BOUGHT["battery_holder"]["g"], (0, 0, top0 + r.plate_t + 8)),
              ("cells", BOUGHT["cells"]["g"] * BOUGHT["cells"]["qty"], (0, 0, top0 + r.plate_t + 8.5))]
    screws = sum(BOUGHT[k]["g"] * BOUGHT[k]["qty"] for k in ("screw_m3", "screw_m2_clamp", "screw_m16_caster", "screw_m2_tray")) + BOUGHT["strap"]["g"]
    items.append(("screws_and_strap", screws, (0, 0, 27.0)))
    # The electrical design (electrical.py): switch, regulator, fuse, capacitors, connectors,
    # passives and the routed wires, taken at the harness's centre of length.
    import electrical
    import wiring
    lengths, centre = None, (-10.0, 5.0, r.plate_top_z + 6.0)
    if wiring.ROUTED.exists():
        wires = [w for w in json.loads(wiring.ROUTED.read_text(encoding="utf-8")) if "points_mm" in w]
        lengths = {w["id"]: w["cut_mm"] for w in wires}
        pts = np.vstack([np.asarray(w["points_mm"]) for w in wires])
        centre = tuple(float(v) for v in pts.mean(0))
    items.append(("electrical", electrical.electrical_mass_g(r, lengths)["total"], centre))
    return items


def wheel_mass_g(r: Robot) -> float:
    made = {p["part"]: p for p in design.made_parts(r)}
    return made["rim"]["g"] + made["hub"]["g"] + BOUGHT["tyre"]["g"] + 3 * BOUGHT["screw_m2_hub"]["g"]


def chassis(r: Robot) -> dict:
    items = mass_items(r)
    total = sum(g for _, g, _ in items)
    com = [sum(g * c[k] for _, g, c in items) / total for k in range(3)]
    return {"g": total, "com_mm": com, "items": items}


def _shape(url: str, colour: str, rough: float = 0.6, metal: float = 0.0) -> str:
    return (f'Shape {{ appearance PBRAppearance {{ baseColor {colour} roughness {rough} metalness {metal} }} '
            f'geometry Mesh {{ url "{url}" }} }}')


def _box(size, colour, rough=0.6, metal=0.0) -> str:
    return (f"Shape {{ appearance PBRAppearance {{ baseColor {colour} roughness {rough} metalness {metal} }} "
            f"geometry Box {{ size {_v(*size)} }} }}")


def _at(t, child, rot: str = "") -> str:
    return f"Pose {{ translation {_v(*t)}{(' rotation ' + rot) if rot else ''} children [ {child} ] }}"


PLY, PLY_TOP, PLA, ALU, BLACK, RUBBER = ".84 .70 .50", ".86 .73 .54", ".12 .45 .80", ".80 .81 .83", ".10 .10 .11", ".05 .05 .05"


# The assembly order (ASSEMBLY.md): what each step adds.
STEPS = ["The bottom plate", "The caster, under the plate", "The motors, in their clamps", "The electronics tray",
         "The standoffs", "The wheels: hub, rim, tyre", "The top plate and the battery"]


def _bought(placed, mesh_url: str, prefer_vendor: bool) -> str:
    """One bought part (bought_parts.Placed) as a posed group of its meshes."""
    _, entries = bought_parts.model_meshes(placed.model, prefer_vendor)
    metal = placed.model.startswith(("shcs", "nut", "standoff"))
    shapes = " ".join(_shape(f"{mesh_url}/{e['file']}", _v(*e["rgb"]), 0.35 if metal else 0.6, 0.85 if metal else 0.0)
                      for e in entries)
    ax = bought_parts.axis_angle(placed.R)
    return _at(tuple(v * 0.001 for v in placed.t), shapes, _v(*ax) if ax[3] else "")


def assembly(r: Robot, mesh_url: str, prefer_vendor: bool = False):
    """(step, VRML) for every chassis part, and (step, side, VRML) for each wheel's visuals.

    The bought parts are bought_parts.py's solids: the vendors' CAD where it has been
    fetched and `prefer_vendor`, our drawn models otherwise (what a committed world uses,
    so it opens on a fresh clone)."""
    mm = 0.001
    top0 = r.plate_top_z + r.standoff_len
    parts = [(0, _at((0, 0, (r.plate_top_z - r.plate_t) * mm), _shape(f"{mesh_url}/bottom_plate.stl", PLY, 0.85))),
             (6, _at((0, 0, top0 * mm), _shape(f"{mesh_url}/top_plate.stl", PLY_TOP, 0.85))),
             (6, _at((0, 0, top0 * mm), _shape(f"{mesh_url}/engraving.stl", ".12 .08 .05", 0.9)))]
    for j in (-1, 1):
        parts.append((2, _at((r.axle_x * mm, j * r.clamp_y * mm, r.plate_top_z * mm), _shape(f"{mesh_url}/motor_clamp.stl", PLA, 0.5))))
    parts.append((3, _at((r.tray_x * mm, 0, r.plate_top_z * mm), _shape(f"{mesh_url}/tray.stl", PLA, 0.5))))
    parts += [(pl.step, _bought(pl, mesh_url, prefer_vendor)) for pl in bought_parts.placements(r)]
    parts += harness(r, mesh_url)
    wheels = []
    for side, j in (("left", 1), ("right", -1)):
        # The rim's inner face (its z = 0) toward the chassis; the hub between it and the gearbox.
        rot = "1 0 0 -1.5708" if j > 0 else "1 0 0 1.5708"
        rim = _at((0, -j * r.rim_width / 2 * mm, 0), _shape(f"{mesh_url}/rim.stl", PLA, 0.5), rot)
        hub = _at((0, -j * (r.rim_width / 2 + r.hub_t) * mm, 0), _shape(f"{mesh_url}/hub.stl", ALU, 0.3, 0.9), rot)
        tyre = _at((0, 0, 0), _shape(f"{mesh_url}/tyre.stl", RUBBER, 0.8), "1 0 0 1.5708")
        turn = bought_parts.WHEEL_TURN[side]          # the right wheel half a turn on: its D-bore meets the flat
        wheel = f"{rim} {hub}" if not turn else _at((0, 0, 0), f"{rim} {hub}", f"0 1 0 {_f(turn)}")
        wheels.append((5, side, f"{wheel} {tyre}"))
    return parts, wheels


def harness(r: Robot, mesh_url: str) -> list:
    """(step, VRML) for every routed wire (wiring.py's release/electrical/wires_routed.json)
    and the parts that hang on the harness (fuse, bulk capacitor, diode, the JST SM pairs).
    The battery's wires go in with the top plate (step 6); the rest with the tray (step 3)."""
    import json
    import electrical
    import wiring
    if not wiring.ROUTED.exists():
        return []
    out = []
    for w in json.loads(wiring.ROUTED.read_text(encoding="utf-8")):
        if "points_mm" not in w or not (wiring.WIRE_MESHES / f"{w['id']}.stl").exists():
            continue
        step = 6 if "BT1" in (w["from"][0], w["to"][0]) or w["id"] in ("W3",) else 3
        out.append((step, _shape(f"{mesh_url}/wires/{w['id']}.stl", wiring.COLOURS[w["colour"]], 0.5)))
    pos = electrical.positions(r)
    mid = lambda a, b: tuple((pa + pb) / 2 * 0.001 for pa, pb in zip(pos[a], pos[b]))

    def hung(model, at, step):
        _, entries = bought_parts.model_meshes(model, prefer_vendor=False)
        shapes = " ".join(_shape(f"{mesh_url}/{e['file']}", _v(*e["rgb"]), 0.5) for e in entries)
        return (step, _at(at, shapes))
    out.append(hung("fuse", mid(("F1", "1"), ("F1", "2")), 6))
    out.append(hung("diode", mid(("D1", "A"), ("D1", "K")), 3))
    va, vg = pos[("U2", "VIN")], pos[("U2", "GND")]
    out.append(hung("cap_bulk", ((va[0] + vg[0]) / 2 * 0.001, (va[1] - 5.5) * 0.001, (va[2] + 3.2) * 0.001), 3))
    for j, step in (("J1", 6), ("J2", 3), ("J3", 3)):
        out.append(hung("jst_sm", mid((j, "1"), (j, "2")), step))
    return out


def _ball(r: Robot) -> str:
    """The caster's ball as the robot's contact with the desk: frictionless, drawn by the caster."""
    ball_r = 9.525 / 2
    return (f'Solid {{ name "caster_ball" translation {_v(r.caster_x * 0.001, 0, ball_r * 0.001)} newtonFriction 0 '
            f'boundingObject Sphere {{ radius {_f(ball_r * 0.001)} subdivision 3 }} }}')


def robot_node(r: Robot, mesh_url: str, controller: str = "desk_robot_drive", args: Sequence[str] = (),
               prefer_vendor: bool = False) -> str:
    mm = 0.001
    m = BOUGHT["gearmotor"]
    c = chassis(r)
    parts, visuals = assembly(r, mesh_url, prefer_vendor)
    kids = [v for _, v in parts] + [_ball(r)]
    wheels = []
    wm = wheel_mass_g(r) / 1000.0
    for _, side, vis in visuals:
        wy = (1 if side == "left" else -1) * r.wheel_y
        wheels.append(
            f"HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 1 0 anchor {_v(r.axle_x * mm, wy * mm, r.axle_z * mm)} }}\n"
            f'  device [ RotationalMotor {{ name "{side}_motor" maxVelocity {_f(m["no_load_rpm"] * 2 * math.pi / 60)} '
            f'maxTorque {_f(m["stall_kgcm"] * G / 100)} }} PositionSensor {{ name "{side}_sensor" }} ]\n'
            f'  endPoint Solid {{ name "{side}_wheel" translation {_v(r.axle_x * mm, wy * mm, r.axle_z * mm)}\n'
            f"    children [ {vis} ]\n"
            f"    boundingObject Pose {{ rotation 1 0 0 1.5708 children [ Cylinder {{ radius {_f(r.wheel_d / 2 * mm)} "
            f"height {_f(r.tyre_cs_mm * mm)} subdivision 48 }} ] }}\n"
            f"    physics Physics {{ density -1 mass {_f(wm)} }} }} }}")
    com = [v * mm for v in c["com_mm"]]
    arg_list = " ".join(f'"{a}"' for a in args)
    return (f'DEF DESK_ROBOT Robot {{ name "desk-robot" controller "{controller}" controllerArgs [ {arg_list} ] supervisor TRUE\n'
            f"  children [\n    " + "\n    ".join(kids + wheels) + " ]\n"
            f"  boundingObject Pose {{ translation 0 0 {_f((r.plate_top_z - r.plate_t / 2) * mm)} children [ "
            f"Box {{ size {_v(r.plate_x * mm, r.plate_y * mm, r.plate_t * mm)} }} ] }}\n"
            f"  physics Physics {{ density -1 mass {_f(c['g'] / 1000.0)} centerOfMass [ {_v(*com)} ] }} }}")


def stage_node(r: Robot, mesh_url: str, step: int, prefer_vendor: bool = False) -> str:
    """The robot as it stands after assembly step `step`: a still model, no physics."""
    mm = 0.001
    parts, visuals = assembly(r, mesh_url, prefer_vendor)
    kids = [v for k, v in parts if k <= step]
    for k, side, vis in visuals:
        if k <= step:
            wy = (1 if side == "left" else -1) * r.wheel_y
            kids.append(_at((r.axle_x * mm, wy * mm, r.axle_z * mm), vis))
    return "Pose { children [\n    " + "\n    ".join(kids) + " ] }"


def world(r: Robot, mesh_url: str, args: Sequence[str] = (), prefer_vendor: bool = False) -> str:
    eye, target = (0.45, -0.55, 0.32), (0.25, 0.0, 0.02)
    import sys
    sys.path.insert(0, str(design.MANUFACTURING / "core"))
    from worldkit import look_at
    return "\n".join([
        "#OMNISIM R2025a utf8",
        'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
        "# Generated by projects/manufacturing/products/desk_robot/robot.py from design.py. Regenerate; do not hand-edit.",
        'WorldInfo { basicTimeStep 2 title "OmniSim desk robot - drive test" newtonSolver "mujoco" newtonSubsteps 2 FPS 30 }',
        f"Viewpoint {{ position {_v(*eye)} orientation {look_at(eye, target)} fieldOfView .7 }}",
        "OmniSimSky { }",
        "DEF SUN OmniSimSun { }",
        "DEF SUN_MARKER OmniSimSunMarker { }",
        'DEF DESK Solid { name "desk" translation 0.25 0 -0.01 children [ Shape { appearance PBRAppearance '
        '{ baseColor .55 .42 .30 roughness .7 metalness 0 } geometry Box { size 1.4 1 .02 } } ] '
        'boundingObject Box { size 1.4 1 .02 } }',
        robot_node(r, mesh_url, args=args, prefer_vendor=prefer_vendor),
        "",
    ])


def predictions(r: Robot) -> dict:
    m = BOUGHT["gearmotor"]
    w0 = m["no_load_rpm"] * 2 * math.pi / 60
    rad = r.wheel_d / 2000.0
    return {"no_load_speed_m_s": round(w0 * rad, 4), "spin_no_load_rad_s": round(2 * w0 * rad / (r.track / 1000.0), 4),
            "wheel_radius_mm": round(r.wheel_d / 2, 3), "track_mm": r.track,
            "mass_g": round(chassis(r)["g"] + 2 * wheel_mass_g(r), 1), "com_mm": [round(v, 2) for v in chassis(r)["com_mm"]],
            "stall_force_n": round(2 * m["stall_kgcm"] * G / 100 / rad, 3)}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Write the desk robot's meshes and its drive-test world.")
    p.add_argument("--out", default=str(WORLD))
    p.add_argument("--result", help="where the drive test writes its result (default: .tmp/desk_robot/drive.json)")
    a = p.parse_args(argv)
    r = Robot()
    write_meshes(r)
    out = Path(a.out)
    import os
    url = Path(os.path.relpath(MESHES, out.resolve().parent)).as_posix()
    args = [f"--result={a.result}"] if a.result else []
    out.write_text(world(r, url, args), encoding="utf-8", newline="\n")
    print(json.dumps(predictions(r), indent=1))
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
