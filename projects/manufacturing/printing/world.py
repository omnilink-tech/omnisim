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

"""Generate an OmniSim world for a flying-gantry CoreXY printer from its profile.

The layout is the Voron 2.4 family's (the Sovol SV08 is one): the bed is fixed, the
whole XY gantry rides up and down on four Z drives, the X beam runs along Y on the
gantry's side rails and the toolhead runs along the beam. CoreXY's two belts move the
toolhead in X and Y; the twin drives those Cartesian axes directly (the belts' A/B
mixing is a wiring fact, not a motion one).

Geometry is either schematic, sized from the travel and bed in the profile, or the
machine's own CAD as a mesh pack (`--pack`, built by core/cad_pack.py; for the
SV08 see projects/manufacturing/machines/sovol_sv08/cad/).

Each joint's zero is the axis's power-up position (`s0`), so the authored pose and the
joint agree at load. The `fdm_twin` controller maps a machine position s (mm) to the
joint position (s - s0) / 1000 m. The PrintedPart on the bed lays plastic where the
toolhead's nozzle tip really goes.

    python world.py --printer ../machines/sovol_sv08/sovol_sv08.json --gcode <job.gcode> \\
        --out ../../samples/demos/worlds/manufacturing/sv08_printer_twin.omniworld
    python world.py --gcode <job.gcode> --pack ../machines/sovol_sv08/cad \\
        --out ../../samples/demos/worlds/manufacturing/sv08_printer_twin_cad.omniworld
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import List, Optional, Sequence

HERE = Path(__file__).resolve().parent
for _p in (HERE, HERE.parent / "core"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import printer as printer_mod  # noqa: E402
from drives import accel_feedforward  # noqa: E402
from printer import AXES  # noqa: E402
from worldkit import box as _box, cyl as _cyl, look_at, num as _f  # noqa: E402

ROOT = HERE.parents[2]
# The axis drives (core/drives.py): OmniSim's position servo has ke = 10 x maxForce,
# kd = 0.5 x maxForce, and the controller feeds the motion forward. A belt-and-stepper
# printer axis is stiff at these scales; the force only sets how stiff.
SERVO_FORCE_N = 20000
# Masses, ESTIMATED (the vendor publishes none): a direct-drive toolhead with a planetary
# extruder, a 350 mm X beam with its rail, and the gantry frame with its Y rails and motors.
# Each axis moves like at least dt * kd (10 kg) anyway, see _controller_args.
MASS_KG = {"toolhead": 0.65, "beam": 0.6, "gantry": 3.0}
BED_TOP_M = 0.16        # the bed surface above the floor (schematic world)
NOZZLE_BELOW_BEAM_M = 0.045
RENDER_FPS = 30
# Physics at 2 ms: a nozzle at 300 mm/s moves 0.6 mm per step, and the bead between two
# steps is laid straight, so corners would be cut at 8 ms. Klipper slows to the square
# corner velocity at a sharp corner anyway, so steps there are microns apart.
BASIC_TIME_STEP_MS = 2
NEWTON_SUBSTEPS = 2


def _slider(axis_vec: str, name: str, s_init: float, lo_mm: float, hi_mm: float, max_rate: float,
            endpoint: str) -> str:
    """A motorised slider; `s_init` in metres along the axis, limits in mm, `max_rate` mm/min."""
    # The controller's target leads the motion by the servo's 0.05 s times the speed
    # (fdm_twin SERVO_TAU_S), so near an end the TARGET passes the rail end by that much.
    lead = 0.05 * max_rate / 60.0 / 1000.0
    lo = (lo_mm / 1000.0 - s_init) - 0.002 - lead
    hi = (hi_mm / 1000.0 - s_init) + 0.002 + lead
    return (f"SliderJoint {{ jointParameters JointParameters {{ axis {axis_vec} }}\n"
            f"  device [ LinearMotor {{ name \"{name}_motor\" minPosition {_f(lo)} maxPosition {_f(hi)} "
            f"maxForce {SERVO_FORCE_N} maxVelocity {_f(max_rate / 1000.0 * 2.0)} }} "
            f"PositionSensor {{ name \"{name}_sensor\" }} ]\n  endPoint {endpoint} }}")


def _controller_args(printer_ref: str, gcode_ref: Optional[str], s0: Sequence[float], trace: Optional[str]) -> str:
    s0_arg = ",".join(f"{v:g}" for v in s0)
    # The acceleration feed-forward (core/drives.py): each axis moves its own Solid and
    # everything it carries (X the toolhead, Y the beam too, Z the whole gantry), at no
    # less than the servo-armature bound's dt * kd.
    substep = BASIC_TIME_STEP_MS / 1000.0 / NEWTON_SUBSTEPS
    moved = [MASS_KG["toolhead"], MASS_KG["toolhead"] + MASS_KG["beam"], sum(MASS_KG.values())]
    accel_ff = ",".join(f"{accel_feedforward(m, SERVO_FORCE_N, substep):.3g}" for m in moved)
    args = ['"--printer"', f'"{printer_ref}"', '"--s0"', f'"{s0_arg}"', '"--accel-ff"', f'"{accel_ff}"']
    if gcode_ref:
        args += ['"--gcode"', f'"{gcode_ref}"']
    if trace:
        args += ['"--trace"', f'"{trace}"']
    return " ".join(args)


def _printed_part(p: printer_mod.Printer, origin: Sequence[float], tip: Sequence[float] = (0.0, 0.0, 0.0)) -> str:
    """The plastic on the bed: `origin` is machine X0 Y0 on the bed surface (world metres),
    `tip` the nozzle tip in the toolhead Solid's frame."""
    height = (p.axes["z"].travel + p.axes["z"].position_min) / 1000.0
    return (f"DEF PRINT PrintedPart {{ translation {' '.join(_f(v) for v in origin)} "
            f"size {_f(p.axes['x'].position_max / 1000.0)} {_f(p.axes['y'].position_max / 1000.0)} {_f(height)} "
            f"nozzleSolid \"toolhead\" nozzleTip {' '.join(_f(v) for v in tip)} "
            f"nozzleDiameter {_f(p.extruder.nozzle_diameter / 1000.0)} }}")


def _world(p: printer_mod.Printer, header: List[str], eye: Sequence[float], target: Sequence[float],
           printed: str, args: str, robot_children: str) -> str:
    lines = [
        "#OMNISIM R2025a utf8",
        'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
        *header,
        f'WorldInfo {{ basicTimeStep {BASIC_TIME_STEP_MS} title "OmniSim printer twin - {p.name}" newtonSolver "mujoco" '
        f'newtonSubsteps {NEWTON_SUBSTEPS} FPS {RENDER_FPS} }}',
        f"Viewpoint {{ position {' '.join(_f(v) for v in eye)} orientation {look_at(eye, target)} fieldOfView .7 }}",
        "OmniSimSky { }",
        "DEF SUN OmniSimSun { }",
        "DEF SUN_MARKER OmniSimSunMarker { }",
        'DEF FLOOR Solid { name "floor" translation 0 0 -.05 children [ Shape { appearance PBRAppearance '
        '{ baseColor .30 .31 .32 roughness .9 metalness 0 } geometry Box { size 4 4 .1 } } ] boundingObject Box { size 4 4 .1 } }',
        printed,
        f'DEF PRINTER Robot {{ name "printer-twin" supervisor TRUE controller "fdm_twin" controllerArgs [ {args} ]',
        "  children [ " + robot_children + " ] }",
        "",
    ]
    return "\n".join(lines)


def build_world(p: printer_mod.Printer, printer_ref: str, gcode_ref: Optional[str], s0: Sequence[float],
                trace: Optional[str] = None) -> str:
    """The printer from schematic boxes, sized from its travel and bed."""
    bed = p.profile.get("bed", {})
    bx, by = (v / 1000.0 for v in bed.get("size_mm", [p.axes["x"].travel, p.axes["y"].travel]))
    ox, oy = (v / 1000.0 for v in bed.get("origin_mm", [0.0, 0.0]))
    # World frame: the bed centre at x = y = 0. Machine X/Y = 0 sits at the bed's origin corner.
    mx0, my0 = -bx / 2 - ox, -by / 2 - oy
    sx, sy, sz = (v / 1000.0 for v in s0)
    rail_x = max(mx0 + p.axes["x"].position_max / 1000.0, bx / 2) + 0.045   # gantry side rails
    y_front, y_back = my0 + p.axes["y"].position_min / 1000.0 - 0.04, my0 + p.axes["y"].position_max / 1000.0 + 0.04
    top = BED_TOP_M + p.axes["z"].position_max / 1000.0 + NOZZLE_BELOW_BEAM_M + 0.08
    dark, alu, steel, accent, bed_col = ".10 .11 .12", ".72 .73 .75", ".55 .57 .60", ".93 .18 .16", ".16 .15 .13"

    # The toolhead: its origin is the nozzle tip.
    toolhead = (f"DEF TOOLHEAD Solid {{ name \"toolhead\" translation {_f(mx0 + sx)} 0 0\n"
                f"  children [ {_box((0, 0, 0.045), (0.05, 0.055, 0.07), accent, 0.1, 0.5)}\n"
                f"    {_box((0, -0.03, 0.06), (0.044, 0.008, 0.045), dark, 0.2, 0.4)}\n"
                f"    {_cyl((0, 0, 0.006), 0.0035, 0.012, '.80 .62 .25', 0.9)}\n"
                f"    Pose {{ translation 0 0 .0005 rotation 1 0 0 3.14159 children [ Shape {{ appearance PBRAppearance "
                f"{{ baseColor .80 .62 .25 metalness .9 roughness .3 }} geometry Cone {{ bottomRadius .002 height .001 }} }} ] }} ]\n"
                f"  boundingObject Pose {{ translation 0 0 0.045 children [ Box {{ size 0.05 0.055 0.07 }} ] }}\n"
                f"  physics Physics {{ density -1 mass {MASS_KG['toolhead']} }} }}")
    x_joint = _slider("1 0 0", "x", sx, p.axes["x"].position_min, p.axes["x"].position_max,
                      p.max_velocity * 60.0, toolhead)
    beam_z = NOZZLE_BELOW_BEAM_M + 0.02
    beam = (f"DEF BEAM Solid {{ name \"beam\" translation 0 {_f(my0 + sy)} 0\n"
            f"  children [ {_box((0, 0.012, beam_z), (2 * rail_x, 0.02, 0.04), dark, 0.3, 0.4)}\n"
            f"    {_box((0, 0.012, beam_z + 0.0225), (2 * rail_x - 0.03, 0.009, 0.005), steel, 0.8, 0.3)}\n"
            f"    {x_joint} ]\n"
            f"  boundingObject Pose {{ translation 0 0.012 {_f(beam_z)} children [ Box {{ size {_f(2 * rail_x)} 0.02 0.04 }} ] }}\n"
            f"  physics Physics {{ density -1 mass {MASS_KG['beam']} }} }}")
    y_joint = _slider("0 1 0", "y", sy, p.axes["y"].position_min, p.axes["y"].position_max,
                      p.max_velocity * 60.0, beam)
    side_len = y_back - y_front
    gantry_parts = [
        _box((-rail_x - 0.015, (y_front + y_back) / 2, beam_z), (0.03, side_len, 0.03), dark, 0.3, 0.4),
        _box((rail_x + 0.015, (y_front + y_back) / 2, beam_z), (0.03, side_len, 0.03), dark, 0.3, 0.4),
        _box((0, y_front - 0.015, beam_z), (2 * rail_x + 0.06, 0.03, 0.03), dark, 0.3, 0.4),
        _box((0, y_back + 0.015, beam_z), (2 * rail_x + 0.06, 0.03, 0.03), dark, 0.3, 0.4),
    ]
    gantry = (f"DEF GANTRY Solid {{ name \"gantry\" translation 0 0 {_f(BED_TOP_M + sz)}\n"
              f"  children [ " + "\n    ".join(gantry_parts) + f"\n    {y_joint} ]\n"
              f"  boundingObject Pose {{ translation {_f(-rail_x - 0.015)} {_f((y_front + y_back) / 2)} {_f(beam_z)} "
              f"children [ Box {{ size 0.03 {_f(side_len)} 0.03 }} ] }}\n"
              f"  physics Physics {{ density -1 mass {MASS_KG['gantry']} }} }}")
    z_joint = _slider("0 0 1", "z", sz, p.axes["z"].position_min, p.axes["z"].position_max,
                      p.max_z_velocity * 60.0, gantry)

    half_w, half_d = rail_x + 0.06, max(abs(y_front), abs(y_back)) + 0.06
    frame = [
        _box((0, 0, 0.03), (2 * half_w + 0.04, 2 * half_d + 0.04, 0.06), dark, 0.2, 0.6),           # base
        _box((0, 0, BED_TOP_M - 0.006), (bx + 0.01, by + 0.01, 0.008), ".08 .08 .09", 0.4, 0.4),    # bed plate
        _box((0, 0, BED_TOP_M - 0.001), (bx, by, 0.002), bed_col, 0.2, 0.7),                       # textured PEI sheet
        _box((0, 0, (0.06 + BED_TOP_M - 0.01) / 2), (0.06, 0.06, BED_TOP_M - 0.07), dark),            # bed support
    ]
    for i in range(4):
        x = (-1) ** i * half_w
        y = (-1) ** (i // 2) * half_d
        frame.append(_box((x, y, top / 2), (0.03, 0.03, top), alu, 0.7, 0.35))
    for y in (-half_d, half_d):
        frame.append(_box((0, y, top - 0.015), (2 * half_w, 0.03, 0.03), alu, 0.7, 0.35))
    for x in (-half_w, half_w):
        frame.append(_box((x, 0, top - 0.015), (0.03, 2 * half_d, 0.03), alu, 0.7, 0.35))

    eye = (0.12, -0.72, BED_TOP_M + 0.32)          # between the front posts, over the gantry rail
    target = (0.0, 0.0, BED_TOP_M + 0.03)
    header = [f"# Generated by projects/manufacturing/printing/world.py from {printer_ref}. Regenerate; do not hand-edit."]
    return _world(p, header, eye, target, _printed_part(p, (mx0, my0, BED_TOP_M)),
                  _controller_args(printer_ref, gcode_ref, s0, trace), "\n    ".join(frame) + "\n    " + z_joint)


# A vendor's CAD carries its modeller's placeholder appearances, not the product's finish:
# the SV08's is mostly SolidWorks' default blues, lilacs and greens. Those are drawn as a
# dark printed plastic; greys, metals, blacks and whites (the sheet's logo) are kept as
# drawn. The colours are ours, not Sovol's.
PLACEHOLDER_SATURATION = 0.35
DARK_PLASTIC = (0.11, 0.11, 0.12)


def _finish(rgba: Sequence[float]) -> List[float]:
    r, g, b, a = rgba
    hi, lo = max(r, g, b), min(r, g, b)
    if hi > 0 and (hi - lo) / hi > PLACEHOLDER_SATURATION:
        return [*DARK_PLASTIC, a]
    return [r, g, b, a]


def build_pack_world(p: printer_mod.Printer, printer_ref: str, gcode_ref: Optional[str], s0: Sequence[float],
                     pack: dict, mesh_prefix: str, trace: Optional[str] = None, frame_shadows: bool = True) -> str:
    """The printer from its CAD mesh pack (core/cad_pack.py) instead of schematic boxes.

    As in the CNC twin's build_pack_world: every body frame coincides with the pack frame
    at the CAD snapshot's axis positions, so meshes sit at identity and each Solid is
    offset along its axis by (power-up position - CAD position). The nozzle tip is the
    pack's `tool_tip`, and the bed surface is where the CAD's nozzle stood above it
    (cad_pose_s z), so the PrintedPart lands on the CAD's own bed sheet.

    `frame_shadows=False` keeps the static frame out of the shadow passes: while the
    machine moves every frame is redrawn, shadow maps included, and the frame is most of
    the triangles.
    """
    cad = pack["cad_pose_s"]
    tip = pack["tool_tip"]
    dirs = pack["axis_dirs"]
    # Machine X0 Y0 Z0 in the pack frame: the CAD's nozzle tip less its machine position.
    origin = [tip[k] - sum(dirs[a][k] * cad[i] / 1000.0 for i, a in enumerate(AXES)) for k in range(3)]
    rates = {"x": p.max_velocity, "y": p.max_velocity, "z": p.max_z_velocity}

    def shapes(group: str) -> str:
        out = []
        shadows = "" if frame_shadows or group != "frame" else " castShadows FALSE"
        for e in pack["groups"][group]["meshes"]:
            r, g, b, a = _finish(e["rgba"])
            transparency = f" transparency {_f(1 - a)}" if a < 0.999 else ""
            out.append(f'Shape {{ appearance PBRAppearance {{ baseColor {_f(r)} {_f(g)} {_f(b)} metalness 0.3 '
                       f'roughness 0.5{transparency} }} geometry Mesh {{ url "{mesh_prefix}/{e["file"]}" }}{shadows} }}')
        return "\n    ".join(out)

    def body(spec: dict) -> str:
        group, axis = spec["group"], spec["axis"]
        i = AXES.index(axis)
        delta = (s0[i] - cad[i]) / 1000.0
        lo, hi = pack["groups"][group]["bbox"]
        centre = [(lo[k] + hi[k]) / 2 for k in range(3)]
        children = [shapes(group)] + [body(c) for c in pack["bodies"] if c["parent"] == group]
        solid = (f'DEF {group.upper()} Solid {{ name "{group}" translation {" ".join(_f(c * delta) for c in dirs[axis])}\n'
                 f"  children [ " + "\n    ".join(children) + " ]\n"
                 f"  boundingObject Pose {{ translation {' '.join(_f(c) for c in centre)} children [ Box {{ size 0.02 0.02 0.02 }} ] }}\n"
                 f"  physics Physics {{ density -1 mass {_f(MASS_KG[group])} "
                 f"centerOfMass [ {' '.join(_f(c) for c in centre)} ] }} }}")
        ax = p.axes[axis]
        return _slider(" ".join(_f(c) for c in dirs[axis]), axis, s0[i] / 1000.0, ax.position_min, ax.position_max,
                       rates[axis] * 60.0, solid)

    roots = [body(c) for c in pack["bodies"] if c["parent"] == "frame"]
    # A front three-quarter view from outside the frame: the whole bed, the gantry where it
    # powers up (100 mm over the bed) and the frame posts clear of the middle. The fan duct
    # hangs a few mm over the nozzle, so as on the real machine the top layers stay under
    # it and the part shows below the toolhead as it grows.
    target = (origin[0] + 0.175, origin[1] + 0.175, origin[2] + 0.07)     # over the bed centre
    eye = (target[0] + 0.42, target[1] - 0.78, target[2] + 0.23)
    src = pack["source"]
    header = [f"# Generated by projects/manufacturing/printing/world.py from {printer_ref} and the "
              f"{pack['name']} mesh pack. Regenerate; do not hand-edit.",
              f"# Geometry: {src.get('attribution', '')} ({src.get('licence', 'licence not stated')}) {src.get('url', '')}"]
    return _world(p, header, eye, target, _printed_part(p, origin, tip),
                  _controller_args(printer_ref, gcode_ref, s0, trace),
                  shapes("frame") + "\n    " + "\n    ".join(roots))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--printer", default=str(printer_mod.DEFAULT_PROFILE))
    ap.add_argument("--gcode", help="the job the controller runs at start (repository-relative)")
    ap.add_argument("--s0", help="x,y,z machine mm at power-up (default: centred, 100 mm up)")
    ap.add_argument("--trace", help="where the controller writes its trace (default .tmp/printer-twin/trace.jsonl)")
    ap.add_argument("--pack", help="mesh pack directory from core/cad_pack.py (default: schematic boxes)")
    ap.add_argument("--no-frame-shadows", action="store_true", help="the pack's static frame casts no shadows")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    p = printer_mod.load(args.printer)
    s0 = [float(v) for v in args.s0.split(",")] if args.s0 else \
        [p.axes["x"].position_max / 2, p.axes["y"].position_max / 2, 100.0]
    ref = Path(args.printer)
    ref = (ref if ref.is_absolute() else (HERE / ref)).resolve()
    try:
        printer_ref = ref.relative_to(ROOT).as_posix()
    except ValueError:
        printer_ref = ref.as_posix()
    out = Path(args.out)
    if args.pack:
        pack_dir = Path(args.pack).resolve()
        pack = json.loads((pack_dir / "pack.json").read_text(encoding="utf-8"))
        prefix = os.path.relpath(pack_dir, out.resolve().parent).replace(os.sep, "/")
        text = build_pack_world(p, printer_ref, args.gcode, s0, pack, prefix, args.trace,
                                frame_shadows=not args.no_frame_shadows)
    else:
        text = build_world(p, printer_ref, args.gcode, s0, args.trace)
    out.write_text(text, encoding="utf-8")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
