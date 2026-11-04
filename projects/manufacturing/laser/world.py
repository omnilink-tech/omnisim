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

"""Generate a laser twin world: a CNC's CAD pack with a laser module in place of the cutter.

The machine is the CNC twin's own (cnc/world.py build_pack_world): the same CAD, axes and
drives. What changes is on the Z body and on the bed:

- The HEAD is schematic (a nozzle, heatsink, fan and bracket of plausible size), mounted
  in front of the router. The vendor publishes no licensed CAD for it, so its look and
  mounting point are not the real part's; the optics are (spot, lens, focal distance).
- The SHEET is a MillingStock with toolShape "laser": the engine burns it where the beam
  really went, at the power grbl_twin --laser sets every step.
- Z powers up with the focal point on the sheet's top at its centre, so a job's
  `G10 L20 P1 X0 Y0 Z0` puts work zero there, as a user focusing and setting the origin
  would.

    python projects/manufacturing/laser/world.py \\
        --machine projects/manufacturing/machines/longmill_mk2_30x30/longmill_mk2_30x30.grbl \\
        --pack projects/manufacturing/machines/longmill_mk2_30x30/cad \\
        --laser projects/manufacturing/machines/longmill_mk2_30x30/sienci_laserbeam_7w.json \\
        --out projects/samples/demos/worlds/manufacturing/laser_longmill_laserbeam.omniworld
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
from pathlib import Path
from typing import List, Sequence

HERE = Path(__file__).resolve().parent
MANUFACTURING = HERE.parent
for _p in (HERE, MANUFACTURING / "cnc", MANUFACTURING / "core"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import machine as machine_mod  # noqa: E402
from laser import MATERIALS, LaserModule  # noqa: E402
from worldkit import box, cyl, look_at, num as _f  # noqa: E402
# The CNC's world.py, by path: this file is world.py too, so `import world` would be
# whichever of the two comes first on sys.path.
_spec = importlib.util.spec_from_file_location("cnc_world", MANUFACTURING / "cnc/world.py")
cnc_world = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cnc_world)

ROOT = MANUFACTURING.parents[1]
# The head, in the pack frame at the CAD pose (mm): centred this far in front of (-Y) the
# CAD's tool axis, nozzle bottom this far below the router's collet nose.
HEAD_FRONT_MM = 75.0
HEAD_BELOW_NOSE_MM = 10.0
HEAD_W_MM = 34.0
NOZZLE_MM = (6.0, 8.0)          # radius, length
HEATSINK_H_MM = 46.0
FAN_H_MM = 12.0


def head_geometry(pack: dict, module: dict):
    """The schematic head's VRML (pack frame) and the focal point it focuses at (mm)."""
    tip = [v * 1000.0 for v in pack["tool_tip"]]
    nose = pack["collet"]["nose_z_m"] * 1000.0
    cx, cy = tip[0], tip[1] - HEAD_FRONT_MM
    bottom = nose - HEAD_BELOW_NOSE_MM
    focal = sum(module["focal_mm"]) / 2.0          # bottom of the head to the work
    focus = [cx, cy, bottom - focal]
    m = 0.001
    r, ln = NOZZLE_MM
    sink_z = bottom + ln
    fan_z = sink_z + HEATSINK_H_MM
    dark, metal = ".06 .06 .07", ".62 .63 .66"
    parts = [
        cyl((cx * m, cy * m, (bottom + ln / 2) * m), r * m, ln * m, ".12 .12 .13", 0.6),
        box((cx * m, cy * m, (sink_z + HEATSINK_H_MM / 2) * m), (HEAD_W_MM * m, HEAD_W_MM * m, HEATSINK_H_MM * m),
            dark, 0.5),
        box((cx * m, cy * m, (fan_z + FAN_H_MM / 2) * m), (HEAD_W_MM * m, HEAD_W_MM * m, FAN_H_MM * m), ".15 .15 .16", 0.2),
        # The bracket back to the router mount (the front of the Z body's CAD).
        box((cx * m, (cy + HEAD_W_MM / 2 + 10) * m, (fan_z - 8) * m), (40 * m, 22 * m, 10 * m), metal, 0.8),
    ]
    # The beam's glow at the focal point: grbl_twin shows it while the laser is lit.
    glow = (f"DEF LASER_GLOW Pose {{ translation {_f(focus[0] * m)} {_f(focus[1] * m)} {_f((focus[2] + 1.0) * m)} "
            f"children [ Shape {{ castShadows FALSE appearance PBRAppearance {{ baseColor .35 .45 1 "
            f"emissiveColor .35 .45 1 emissiveIntensity 4 transparency 1 roughness 1 metalness 0 }} "
            f"geometry Sphere {{ radius .0012 subdivision 2 }} }} ] }}")
    return "\n    ".join(parts + [glow]), focus


def build(machine_path: Path, pack_dir: Path, module_path: Path, out: Path, material_name: str = "plywood",
          sheet_mm: Sequence[float] = (80.0, 80.0, 3.0), resolution_mm: float = 0.05, port: int = 2323) -> str:
    m = machine_mod.load(machine_path)
    pack = json.loads((pack_dir / "pack.json").read_text(encoding="utf-8"))
    module = json.loads(module_path.read_text(encoding="utf-8"))
    laser = LaserModule.load(module_path)
    material = MATERIALS[material_name]
    head, focus = head_geometry(pack, module)
    cad = pack["cad_pose_s"]
    sx, sy, thick = (float(v) for v in sheet_mm)
    bed = pack["bed_top_m"] * 1000.0
    # X/Y power up where the CAD has them; Z where the focal point lands on the sheet's top.
    s0 = [cad[0], cad[1], cad[2] + (bed + thick - focus[2])]
    if not 0.0 <= s0[2] <= m.axes["z"].travel:
        raise ValueError(f"the head cannot focus on a {thick} mm sheet: Z would power up at {s0[2]:.1f} mm")
    origin = [focus[0] - sx / 2, focus[1] - sy / 2, bed]
    rel = lambda p: p.resolve().relative_to(ROOT).as_posix()   # noqa: E731
    c, k = material.color, material.char_color
    node = (f'DEF STOCK MillingStock {{ translation {" ".join(_f(v / 1000.0) for v in origin)} '
            f'size {_f(sx / 1000.0)} {_f(sy / 1000.0)} {_f(thick / 1000.0)} resolution {_f(resolution_mm / 1000.0)}\n'
            f'  toolSolid "z" toolTip {" ".join(_f(v / 1000.0) for v in focus)} toolShape "laser"\n'
            f'  laserSpot {_f(laser.spot_mm[0] / 1000.0)} {_f(laser.spot_mm[1] / 1000.0)} '
            f'beamHalfAngle {_f(laser.beam_half_angle_rad)}\n'
            f'  ablationEnergy {_f(material.ablation_j_mm3 * 1e9)} charFluence {_f(material.char_j_mm2 * 1e6)}\n'
            f'  skinColor {" ".join(_f(v) for v in c)} coreColor {" ".join(_f(v) for v in c)} '
            f'charColor {" ".join(_f(v) for v in k)} roughness .85 }}')
    args = [f'"--laser={rel(module_path)}"', f'"--laser-material={material_name}"']
    centre = (focus[0] / 1000.0, focus[1] / 1000.0, (bed + thick) / 1000.0)
    prefix = Path(os.path.relpath(pack_dir.resolve(), out.resolve().parent)).as_posix()
    text = cnc_world.build_pack_world(
        m, rel(machine_path), pack, prefix, s0, None, port, None, None,
        laser={"head": head, "node": node, "args": args, "centre": centre,
               "title": f"OmniSim laser twin - {pack['name']} + {laser.name}"})
    # Frame the sheet and the head over it (build_pack_world frames a cutter's stock).
    eye = (centre[0] + 0.20, centre[1] - 0.30, centre[2] + 0.17)
    target = (centre[0], centre[1] + 0.01, centre[2] + 0.02)
    lines: List[str] = []
    for line in text.split("\n"):
        if line.startswith("Viewpoint "):
            line = f"Viewpoint {{ position {' '.join(_f(v) for v in eye)} orientation {look_at(eye, target)} fieldOfView .7 }}"
        elif line.startswith("# Generated by projects/manufacturing/cnc/world.py"):
            line = (f"# Generated by projects/manufacturing/laser/world.py from {rel(machine_path)}, the "
                    f"{pack['name']} mesh pack and {rel(module_path)}. Regenerate; do not hand-edit.\n"
                    f"# The laser head is SCHEMATIC (no licensed CAD for it); its optics are the vendor's.")
        lines.append(line)
    return "\n".join(lines)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Generate a laser twin world from a CNC's CAD pack and a laser module.")
    p.add_argument("--machine", required=True)
    p.add_argument("--pack", required=True, help="the machine's mesh pack directory (cad_pack.py)")
    p.add_argument("--laser", required=True, help="laser module JSON")
    p.add_argument("--material", default="plywood", choices=sorted(MATERIALS))
    p.add_argument("--sheet", default="80,80,3", help="sheet x,y,thickness in mm, centred under the focus")
    p.add_argument("--resolution", type=float, default=0.05, help="sheet height-map spacing, mm")
    p.add_argument("--port", type=int, default=2323)
    p.add_argument("--out", required=True)
    a = p.parse_args(argv)
    out = Path(a.out)
    text = build(Path(a.machine), Path(a.pack), Path(a.laser), out, a.material,
                 [float(v) for v in a.sheet.split(",")], a.resolution, a.port)
    out.write_text(text, encoding="utf-8", newline="\n")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
