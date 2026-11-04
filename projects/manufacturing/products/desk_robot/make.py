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

"""Make the desk robot's parts on the twins, one machine at a time, and measure each.

    python projects/manufacturing/products/desk_robot/make.py --out .tmp/desk_robot --only laser
    python projects/manufacturing/products/desk_robot/make.py --out .tmp/desk_robot --only cnc
    python projects/manufacturing/products/desk_robot/make.py --out .tmp/desk_robot --only print

Each step writes its world beside the other manufacturing worlds as a hidden, gitignored
copy (`.desk_robot_<part>.omniworld`: OmniSim finds a world's controllers from its
folder, so a world under .tmp would find none), its run (the engine's dump of the part, the
controller's trace) into <out>/runs/<part>, and its measurements into <out>/made.json,
which the next steps add to. One engine runs at a time.

- laser: each deck plate cut from its own 98 x 88 mm sheet of 3 mm plywood (laser/world.py),
  streamed over TCP (laser/run_job.py), then measured: did it cut free, its size against
  the drawing, every hole's diameter, the engraving.
- cnc: both hubs from a 48 x 26 x 4 mm 6061 blank at 0.025 mm with the fine tool table,
  then measured: did they cut free, their diameter, and whether the motor's D-shaft goes
  into the bore the 1 mm end mill really cut.
- print: the print plate sliced for the SV08 and printed (printing/world.py), then
  measured against its STL (printing/inspect_print.py).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy import ndimage
from shapely.geometry import Point, Polygon

HERE = Path(__file__).resolve().parent
MANUFACTURING = HERE.parents[1]
ROOT = MANUFACTURING.parents[1]
sys.path.insert(0, str(HERE))
for _p in (MANUFACTURING / "laser", MANUFACTURING / "cnc"):
    sys.path.insert(0, str(_p))

import design  # noqa: E402
import inspect_cut  # noqa: E402
import run_job  # noqa: E402

LONGMILL = MANUFACTURING / "machines/longmill_mk2_30x30"
LASER_MODULE = LONGMILL / "sienci_laserbeam_7w.json"
FINE_TOOLS = LONGMILL / "longmill_mk2_tools_aluminium_fine.json"
SHEET_MM = (98.0, 88.0, 3.0)
WORLDS = ROOT / "projects/samples/demos/worlds/manufacturing"
# The twins' GRBL port for these runs: not the demos' 2323, which a window or another
# session's capture of the same machine may hold.
PORT = 2333
HUB_S0_Z = 40.0           # Z at power-up for the hub job: the tip 22.8 mm over the 4 mm blank


def _world(name: str) -> Path:
    return WORLDS / f".desk_robot_{name}.omniworld"


def _rel(p: Path) -> str:
    return p.resolve().relative_to(ROOT).as_posix()


def _save(out: Path, key: str, value) -> dict:
    path = out / "made.json"
    made = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    made[key] = value
    path.write_text(json.dumps(made, indent=1, default=float), encoding="utf-8")
    return made


def _cells(z: dict):
    """Cell-centre coordinates in work mm (the dump's centre is 0, 0)."""
    r = z["res"] * 1000.0
    xs = (np.arange(z["nx"]) - (z["nx"] - 1) / 2) * r
    ys = (np.arange(z["ny"]) - (z["ny"] - 1) / 2) * r
    return np.meshgrid(xs, ys)


def _through_at(z: dict, through: np.ndarray, x: float, y: float):
    """The through-cut region (4-connected) holding the point, or None."""
    r = z["res"] * 1000.0
    i = int(round(x / r + (z["nx"] - 1) / 2))
    j = int(round(y / r + (z["ny"] - 1) / 2))
    labels, _ = ndimage.label(through)
    k = labels[j, i]
    return None if k == 0 else labels == k


def measure_plate(z: dict, drawn: Polygon) -> dict:
    """The plate that fell out, against its drawing: its size, and each hole as cut.

    A laser cuts a hole by tracing its outline, so the hole's middle comes away as a slug
    (a loose piece of its own). The hole is the VOID it leaves in the plate: the plate's
    outline filled, minus the plate, around the hole's centre."""
    cell = (z["res"] * 1000.0) ** 2
    loose = [p for p in inspect_cut.pieces(z) if p["fell_out"]]
    if not loose:
        return {"cut_free": False}
    part = max(loose, key=lambda p: p["area_mm2"])
    x0, y0, x1, y1 = drawn.bounds
    rep = {"cut_free": True, "size_mm": part["size_mm"], "drawn_mm": [round(x1 - x0, 3), round(y1 - y0, 3)],
           "area_mm2": part["area_mm2"], "drawn_area_mm2": round(drawn.area, 2),
           "slugs": sum(1 for p in loose if p is not part), "holes": []}
    voids, _ = ndimage.label(ndimage.binary_fill_holes(part["_mask"]) & ~part["_mask"])
    r = z["res"] * 1000.0
    for ring in drawn.interiors:
        hole = Polygon(ring)
        c = hole.centroid if hole.centroid.within(hole) else hole.representative_point()
        k = voids[int(round(c.y / r + (z["ny"] - 1) / 2)), int(round(c.x / r + (z["nx"] - 1) / 2))]
        entry = {"at": [round(c.x, 2) + 0.0, round(c.y, 2) + 0.0], "cut": bool(k)}
        if k:
            area = float((voids == k).sum() * cell)
            entry.update(drawn_area_mm2=round(hole.area, 3), cut_area_mm2=round(area, 3),
                         drawn_d_mm=round(2 * math.sqrt(hole.area / math.pi), 3),
                         cut_d_mm=round(2 * math.sqrt(area / math.pi), 3))
        rep["holes"].append(entry)
    cut = [h for h in rep["holes"] if h["cut"]]
    rep["holes_cut"] = f"{len(cut)}/{len(rep['holes'])}"
    if cut:
        err = [h["cut_d_mm"] - h["drawn_d_mm"] for h in cut]
        rep["hole_d_error_mm"] = {"min": round(min(err), 3), "max": round(max(err), 3)}
    return rep


def measure_engraving(z: dict, shape: Polygon) -> dict:
    """Depth and char inside the letters, and how far the char spills outside them."""
    import shapely
    X, Y = _cells(z)
    x0, y0, x1, y1 = shape.bounds
    win = (X >= x0 - 1) & (X <= x1 + 1) & (Y >= y0 - 1) & (Y <= y1 + 1)
    inside = np.zeros_like(win)
    inside[win] = shapely.contains_xy(shape, X[win], Y[win])
    char = 1.0 - np.exp(-z["fluence"] / z["char_fluence"])
    depth = (z["top"] - z["h"]) * 1000.0
    ring = win & ~inside
    return {"letters_area_mm2": round(float(inside.sum() * (z["res"] * 1000) ** 2), 2), "drawn_area_mm2": round(shape.area, 2),
            "depth_mm_in_letters": round(float(depth[inside].mean()), 4),
            "char_in_letters": round(float(char[inside].mean()), 3),
            "char_around_letters": round(float(char[ring].mean()), 3)}


def make_laser(r: design.Robot, out: Path) -> dict:
    sys.path.insert(0, str(MANUFACTURING / "laser"))
    import importlib.util
    spec = importlib.util.spec_from_file_location("laser_world", MANUFACTURING / "laser/world.py")
    laser_world = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(laser_world)
    jobs = design.laser_jobs(r)
    drawn = {"bottom_plate": design.bottom_plate(r), "top_plate": design.top_plate(r)}
    results = {}
    for name, text in jobs.items():
        world = _world(f"laser_{name}")
        world.write_text(laser_world.build(LONGMILL / "longmill_mk2_30x30.grbl", LONGMILL / "cad", LASER_MODULE, world,
                                           "plywood", SHEET_MM, 0.05, PORT), encoding="utf-8", newline="\n")
        gcode = out / "parts" / f"{name}.gcode"
        gcode.parent.mkdir(parents=True, exist_ok=True)
        gcode.write_text(text, encoding="utf-8", newline="\n")
        run = out / "runs" / name
        r_ = run_job.run(world, gcode, run, port=PORT, timeout_s=3600)
        z = inspect_cut.load(run / "sheet.zmap")
        rep = measure_plate(z, drawn[name])
        rep["job"] = r_.get("job")
        rep["wall_s"] = r_.get("wall_s")
        if name == "top_plate":
            rep["engraving"] = measure_engraving(z, design.name_engraving(r))
        results[name] = rep
        print(name, json.dumps({k: v for k, v in rep.items() if k != "holes"}), flush=True)
    return _save(out, "laser", results)


def _tip_above_stock(s0_z: float, thickness: float) -> float:
    """The tool tip's height above the blank's top at power-up, as cnc/world.py places it."""
    pack = json.loads((LONGMILL / "cad/pack.json").read_text(encoding="utf-8"))
    tip = pack["tool_tip"][2] * 1000.0 + (s0_z - pack["cad_pose_s"][2])
    return tip - (pack["bed_top_m"] * 1000.0 + thickness)


def measure_hubs(z: dict, r: design.Robot) -> dict:
    loose = [p for p in inspect_cut.pieces(z) if p["fell_out"] and p["area_mm2"] > 50.0]
    rep = {"cut_free": len(loose), "hubs": []}
    shaft = design.shaft_section(r)
    X, Y = _cells(z)
    through = z["h"] <= 1e-7
    for p in sorted(loose, key=lambda p: p["centre_mm"][0]):
        cx, cy = min(design.HUB_CENTRES, key=lambda c: abs(c[0] - p["centre_mm"][0]))
        filled = ndimage.binary_fill_holes(p["_mask"])
        cell = (z["res"] * 1000.0) ** 2
        od = 2 * math.sqrt(filled.sum() * cell / math.pi)
        bore = _through_at(z, through, cx, cy)
        # The motor's D-shaft, at the bore's design pose: every cell it covers must be cut through.
        inside = np.zeros_like(through)
        sx0, sy0, sx1, sy1 = shaft.bounds
        win = (X >= cx + sx0 - 0.1) & (X <= cx + sx1 + 0.1) & (Y >= cy + sy0 - 0.1) & (Y <= cy + sy1 + 0.1)
        for j, i in zip(*np.nonzero(win)):
            if shaft.contains(Point(X[j, i] - cx, Y[j, i] - cy)):
                inside[j, i] = True
        blocked = int((inside & ~through).sum())
        drawn_bore = design.hole(0, 0, r.hub_bore_d).intersection(design.box(-5, -5, 5, r.hub_flat))
        screws = []
        for k in range(3):
            ang = math.radians((30, 150, 270)[k])
            hx, hy = cx + r.hub_screw_pcd / 2 * math.cos(ang), cy + r.hub_screw_pcd / 2 * math.sin(ang)
            region = _through_at(z, through, hx, hy)
            screws.append(round(2 * math.sqrt(region.sum() * cell / math.pi), 3) if region is not None else None)
        rep["hubs"].append({"centre_mm": p["centre_mm"], "od_mm": round(od, 3), "drawn_od_mm": r.hub_d,
                            "bore_area_mm2": round(float(bore.sum() * cell), 4) if bore is not None else None,
                            "drawn_bore_area_mm2": round(drawn_bore.area, 4),
                            "screw_hole_d_mm": screws, "drawn_screw_hole_d_mm": design.CLEARANCE["M2"],
                            "shaft_cells": int(inside.sum()), "shaft_cells_blocked": blocked,
                            "shaft_fits": blocked == 0})
    return rep


def make_cnc(r: design.Robot, out: Path) -> dict:
    world = _world("cnc_hubs")
    sx, sy, t = design.HUB_STOCK_MM
    subprocess.run([sys.executable, str(MANUFACTURING / "cnc/world.py"), "--machine", str(LONGMILL / "longmill_mk2_30x30.grbl"),
                    "--pack", str(LONGMILL / "cad"), "--s0", f"394,441,{HUB_S0_Z:g}", "--stock", f"{sx:g},{sy:g},{t:g}",
                    "--stock-resolution", "0.025", "--tools", str(FINE_TOOLS), "--material", "aluminium_alloy",
                    "--bridge-port", "0", "--port", str(PORT), "--out", str(world)], check=True, cwd=ROOT)
    program = design.hub_program(r, _tip_above_stock(HUB_S0_Z, t))
    nc = out / "parts" / "hubs.nc"
    nc.parent.mkdir(parents=True, exist_ok=True)
    nc.write_text(program, encoding="utf-8", newline="\n")
    run = out / "runs" / "hubs"
    r_ = run_job.run(world, nc, run, port=PORT, timeout_s=3600)
    z = inspect_cut.load(run / "sheet.zmap")
    rep = measure_hubs(z, r)
    rep["job"], rep["wall_s"] = r_.get("job"), r_.get("wall_s")
    print("hubs", json.dumps(rep), flush=True)
    return _save(out, "cnc", rep)


def make_print(r: design.Robot, out: Path, timeout_s: float = 7200.0) -> dict:
    parts = out / "parts"
    parts.mkdir(parents=True, exist_ok=True)
    stl = parts / "print_plate.stl"
    design.write_stl(stl, design.print_plate(r), "desk robot print plate")
    gcode = parts / "print_plate.gcode"
    gcode.unlink(missing_ok=True)          # this step's own output: cura_slice.py never overwrites
    subprocess.run([sys.executable, str(MANUFACTURING / "printing/cura_slice.py"), str(stl), "--out", str(gcode)],
                   check=True, cwd=ROOT, capture_output=True)
    run = out / "runs" / "print_plate"
    run.mkdir(parents=True, exist_ok=True)
    trace = run / "trace.jsonl"
    # This run's own record only: a "done" left in an earlier run's trace ends the wait at once.
    for old in (trace, run / "plate.beads"):
        old.unlink(missing_ok=True)
    world = _world("print_plate")
    subprocess.run([sys.executable, str(MANUFACTURING / "printing/world.py"), "--gcode", _rel(gcode),
                    "--trace", trace.resolve().as_posix(), "--pack", str(MANUFACTURING / "machines/sovol_sv08/cad"),
                    "--out", str(world)], check=True, cwd=ROOT)
    env = dict(os.environ, OMNISIM_PRINTED_PART_DUMP=str((run / "plate.beads").resolve()),
               OMNISIM_LOG_PATH=str((run / "omnisim_log.txt").resolve()))
    log = (run / "engine.out").open("w", encoding="utf-8")
    proc = subprocess.Popen([sys.executable, "-m", "omnisim", "run-headless", str(world), "--duration", str(int(timeout_s))],
                            cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    started = time.time()
    done, seen = None, 0
    try:
        while done is None:
            if proc.poll() is not None:
                raise RuntimeError(f"the engine exited before the print finished (see {run / 'engine.out'})")
            if time.time() - started > timeout_s:
                raise RuntimeError("the print did not finish in time")
            time.sleep(5.0)
            if trace.exists():
                # every line since the last look: the twin keeps sampling after "done",
                # so a tail of the file soon no longer holds it
                with trace.open("rb") as f:
                    f.seek(seen)
                    chunk = f.read()
                chunk = chunk[:chunk.rfind(b"\n") + 1]
                seen += len(chunk)
                for line in chunk.decode("utf-8", "replace").splitlines():
                    if '"kind": "done"' in line:
                        done = json.loads(line)
        time.sleep(3.0)          # the engine's last bead dump
    finally:
        run_job._stop(proc)
        log.close()
    res = subprocess.run([sys.executable, str(MANUFACTURING / "printing/inspect_print.py"), str(run / "plate.beads"),
                          str(stl), "--json"], cwd=ROOT, capture_output=True, text=True)
    rep = {"done": done, "wall_s": round(time.time() - started, 1)}
    try:
        rep["inspect"] = json.loads(res.stdout)
    except ValueError:
        rep["inspect_error"] = (res.stdout + res.stderr)[-2000:]
    print("print", json.dumps({k: v for k, v in rep.items() if k != "inspect"}), flush=True)
    return _save(out, "print", rep)


def remeasure_laser(r: design.Robot, out: Path) -> dict:
    """Measure the plates again from their saved sheets (no new run)."""
    made = json.loads((out / "made.json").read_text(encoding="utf-8"))
    drawn = {"bottom_plate": design.bottom_plate(r), "top_plate": design.top_plate(r)}
    for name, rep in made.get("laser", {}).items():
        z = inspect_cut.load(out / "runs" / name / "sheet.zmap")
        fresh = measure_plate(z, drawn[name])
        if name == "top_plate":
            fresh["engraving"] = measure_engraving(z, design.name_engraving(r))
        fresh["job"], fresh["wall_s"] = rep.get("job"), rep.get("wall_s")
        made["laser"][name] = fresh
    return _save(out, "laser", made["laser"])


def remeasure_cnc(r: design.Robot, out: Path) -> dict:
    """Measure the hubs again from their saved stock (no new run)."""
    made = json.loads((out / "made.json").read_text(encoding="utf-8"))
    rep = measure_hubs(inspect_cut.load(out / "runs/hubs/sheet.zmap"), r)
    rep["job"], rep["wall_s"] = made["cnc"].get("job"), made["cnc"].get("wall_s")
    return _save(out, "cnc", rep)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Make the desk robot's parts on the twins and measure them.")
    p.add_argument("--out", required=True)
    p.add_argument("--only", choices=("laser", "cnc", "print", "remeasure-laser", "remeasure-cnc"), action="append")
    a = p.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    r = design.Robot()
    for step in a.only or ("laser", "cnc", "print"):
        {"laser": make_laser, "cnc": make_cnc, "print": make_print, "remeasure-laser": remeasure_laser,
         "remeasure-cnc": remeasure_cnc}[step](r, out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
