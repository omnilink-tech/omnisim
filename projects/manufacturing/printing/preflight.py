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

"""Run a print file through the twin's firmware and hotend, without the engine.

Reports what Klipper would refuse (the print stops there), the print time, filament,
the extent of what is printed against the bed, the flow each feature asks of the hotend
and where it exceeds the melt limit, and how much of the requested plastic would really
leave the nozzle. Engine-free: the same Klipper and hotend models the in-engine twin runs.

    python preflight.py part.gcode --printer ../machines/sovol_sv08/sovol_sv08.json
    python preflight.py part.gcode --json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, Optional, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import klipper  # noqa: E402
import printer as printer_mod  # noqa: E402
from hotend import Hotend  # noqa: E402

SAMPLE_S = 0.002


def preflight(p: printer_mod.Printer, gcode: str, start: Optional[Sequence[float]] = None,
              material: str = "PLA") -> dict:
    job = klipper.load_job(p, gcode, start=start)
    traj = klipper.Trajectory(job, p)
    area = p.extruder.filament_area
    h = Hotend.for_printer(p)
    requested = sum(max(0.0, m.end[3] - m.start[3]) for m in job.moves if m.is_kinematic) * area
    # Flow asked of the hotend per feature, at each move's cruise speed.
    flow: Dict[str, float] = {}
    over = []
    for m in job.moves:
        if not m.is_kinematic or m.axes_r[3] <= 0 or not (m.end[0] != m.start[0] or m.end[1] != m.start[1]):
            continue
        q = m.cruise_v * m.axes_r[3] * area
        key = m.feature or "(none)"
        flow[key] = max(flow.get(key, 0.0), q)
        if q > h.max_mm3_s * 1.001:
            over.append({"line": m.line, "feature": key, "flow_mm3_s": round(q, 2)})
    # The hotend, sampled like the engine steps it.
    ts = np.arange(0.0, job.duration + SAMPLE_S, SAMPLE_S)
    e_motor = traj.extruder(ts) if len(job.moves) else np.zeros_like(ts)
    for e in e_motor:
        h.step(float(e), SAMPLE_S)
    printed = [m for m in job.moves if m.is_kinematic and m.axes_r[3] > 0 and (m.end[0] != m.start[0] or m.end[1] != m.start[1])]
    lo = np.min([[min(m.start[i], m.end[i]) for i in range(3)] for m in printed], axis=0) if printed else None
    hi = np.max([[max(m.start[i], m.end[i]) for i in range(3)] for m in printed], axis=0) if printed else None
    bed = p.profile.get("bed", {})
    bed_lo = bed.get("origin_mm", [0, 0])
    bed_hi = [bed_lo[i] + bed.get("size_mm", [p.axes[a].travel for a in "xy"])[i] for i in range(2)]
    off_bed = printed and (lo[0] < bed_lo[0] - 1e-6 or lo[1] < bed_lo[1] - 1e-6 or hi[0] > bed_hi[0] + 1e-6 or hi[1] > bed_hi[1] + 1e-6)
    layers = [e for e in job.events if e.kind == "layer"]
    density = klipper.FILAMENT_DENSITY_G_CM3.get(material.upper(), 1.24)
    return {
        "printer": p.name,
        "stopped": [{"line": e.line, "error": e.message} for e in job.errors],
        "time_s": round(job.duration, 1),
        "first_layer_at_s": round(layers[0].t, 1) if layers else None,
        "layers": len(layers),
        "moves": len(job.moves),
        "filament_mm": round(requested / area, 1),
        "filament_g": round(requested / 1000.0 * density, 2),
        "extent_mm": {"min": [round(v, 3) for v in lo], "max": [round(v, 3) for v in hi]} if printed else None,
        "off_bed": bool(off_bed),
        "peak_flow_mm3_s": {k: round(v, 2) for k, v in sorted(flow.items())},
        "hotend_limit_mm3_s": h.max_mm3_s,
        "over_limit_moves": len(over),
        "over_limit_first": over[:5],
        "deposited_mm3": round(h.deposited, 1),
        "requested_mm3": round(requested, 1),
        "skipped_mm3": round(h.skipped, 2),
        "unknown_commands": job.unknown,
        "not_modelled": job.notes,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("gcode")
    ap.add_argument("--printer", default=str(printer_mod.DEFAULT_PROFILE))
    ap.add_argument("--start", help="x,y,z where the toolhead is at power-up (default mid-travel)")
    ap.add_argument("--material", default="PLA")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)
    p = printer_mod.load(args.printer)
    start = [float(v) for v in args.start.split(",")] if args.start else None
    report = preflight(p, Path(args.gcode).read_text(encoding="utf-8", errors="replace"), start, args.material)
    if args.json:
        print(json.dumps(report, indent=2))
        return 1 if report["stopped"] else 0
    for k, v in report.items():
        print(f"{k:>20}: {v}")
    return 1 if report["stopped"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
