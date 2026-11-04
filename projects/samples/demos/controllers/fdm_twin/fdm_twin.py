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

"""Runs a print job on a simulated FDM printer (projects/manufacturing/printing).

The job is planned the way the printer's Klipper firmware plans it (klipper.py), with
its input shaper and pressure advance. Each step the slider motors follow that plan as
position servos, fed forward exactly as the CNC twin's drives are (grbl_twin), and the
hotend model (hotend.py) turns the extruder motor's motion into the plastic that leaves
the nozzle. That volume goes to the world's PrintedPart (DEF PRINT), which lays it along
the nozzle's real path.

controllerArgs: --printer <profile.json> [--gcode <file>] [--s0 x,y,z]
                [--trace <jsonl>] [--start-delay s]
"""
import argparse
import json
import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "projects/manufacturing/printing"))
sys.path.insert(0, str(ROOT / "projects/manufacturing/core"))

from omnisim import Supervisor  # noqa: E402

import klipper  # noqa: E402
import printer as printer_mod  # noqa: E402
from hotend import Hotend  # noqa: E402
from printer import AXES  # noqa: E402
from drives import FedForwardDrives  # noqa: E402

TRACE_PERIOD_S = 0.05
# The slider motors are OmniSim position servos, driven like steppers by the shared drive
# model (projects/manufacturing/core/drives.py).
# grbl_twin trims that lead by half its 2 ms substep; at this world's 1 ms substeps the
# trim measured no better than none (2026-11-02, the 20 mm cube).
VELOCITY_LEAD_TRIM_S = 0.0
# The acceleration feed-forward (--accel-ff, s^2) is each axis's EFFECTIVE inertia over
# the servo stiffness, which world.py computes: Newton gives a light position servo
# armature up to dt * kd (the servo-armature bound, AGENTS.md), so every axis of this
# printer moves like 10 kg, not like its 0.65-4.25 kg. Fed the bare masses, the axes
# overshot every acceleration by up to 0.3 mm; fed 5e-5, the error over the cube's walls
# is 9 um median, 16 um at the 99th percentile (2026-11-02).


def _triple(text):
    return [float(v) for v in text.split(",")] if text else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--printer", required=True)
    ap.add_argument("--gcode")
    ap.add_argument("--s0")
    ap.add_argument("--trace", default=os.environ.get("OMNISIM_PRINTER_TWIN_TRACE",
                                                      str(ROOT / ".tmp/printer-twin/trace.jsonl")))
    ap.add_argument("--start-delay", type=float, default=0.5, help="seconds of stillness before the job starts")
    ap.add_argument("--accel-ff", default="5e-5,5e-5,5e-5", help="x,y,z acceleration feed-forward, effective inertia / ke (s^2)")
    args = ap.parse_args()

    accel_ff = _triple(args.accel_ff)
    robot = Supervisor()
    step_ms = int(robot.getBasicTimeStep())
    dt = step_ms / 1000.0
    path = Path(args.printer)
    p = printer_mod.load(path if path.is_absolute() else ROOT / path)
    s0 = _triple(args.s0) or [p.axes["x"].position_max / 2, p.axes["y"].position_max / 2, 100.0]

    motors, sensors = [], []
    for a in AXES:
        motor = robot.getDevice(f"{a}_motor")
        motor.setPosition(0.0)
        sensor = robot.getDevice(f"{a}_sensor")
        sensor.enable(step_ms)
        motors.append(motor)
        sensors.append(sensor)
    part = robot.getFromDef("PRINT")
    volume_field = part.getField("extrudedVolume") if part is not None else None
    deposited_field = part.getField("depositedVolume") if part is not None else None
    crashes_field = part.getField("nozzleCrashes") if part is not None else None

    job = traj = None
    if args.gcode:
        gpath = Path(args.gcode)
        gpath = gpath if gpath.is_absolute() else ROOT / gpath
        job = klipper.load_job(p, gpath.read_text(encoding="utf-8", errors="replace"), start=s0)
        traj = klipper.Trajectory(job, p)
        print(f"[fdm_twin] {p.name}: {gpath.name}, {len(job.moves)} moves, {job.duration:.1f} s planned"
              + (f", STOPS at line {job.errors[0].line}: {job.errors[0].message}" if job.errors else ""), flush=True)
        for note in job.notes:
            print(f"[fdm_twin] not modelled: {note}", flush=True)
    hotend = Hotend.for_printer(p)

    trace_path = Path(args.trace)
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    trace = trace_path.open("w", encoding="utf-8", buffering=1)
    trace.write(json.dumps({"kind": "start", "printer": p.summary(), "s0": s0, "gcode": args.gcode,
                            "planned_s": job.duration if job else None,
                            "errors": [str(e) for e in job.errors] if job else []}) + "\n")
    layers = [(e.t, e.value) for e in job.events if e.kind == "layer"] if job else []

    # The job at the step times (job step k ends at k * dt), sampled a window at a time:
    # sampled whole up front it took 1.55 GB for a 3.3 h print (klipper.StepSampler).
    sampler = klipper.StepSampler(traj, dt) if job else None
    delay = int(round(args.start_delay / dt))

    def command_at(k):
        if job is None or k < 0:
            return list(s0)
        return sampler.toolhead(k)

    n = 0
    drives = FedForwardDrives(3, VELOCITY_LEAD_TRIM_S, accel_ff)
    next_trace = 0.0
    finished = False
    layer_i = 0
    while robot.step(step_ms) != -1:
        n += 1
        k = n - delay                                    # the job step that just ended
        tj = k * dt
        measured = [s0[i] + sensors[i].getValue() * 1000.0 for i in range(3)]
        # The coming step runs from step k to k + 1: its target is centred on it, with the
        # velocity across it and the acceleration at its middle (grbl_twin's scheme).
        prev, cur, nxt, after = (command_at(k + j) for j in (-1, 0, 1, 2))
        for i, target in enumerate(drives.targets(prev, cur, nxt, after, measured, dt)):
            motors[i].setPosition((target - s0[i]) / 1000.0)
        # What leaves the nozzle during the coming step, laid by the engine along where the
        # nozzle really goes during it.
        if job is not None and k + 1 >= 0 and not finished:
            hotend.step(sampler.extruder(k + 1), dt)
            if volume_field is not None:
                volume_field.setSFFloat(hotend.deposited * 1e-9)
        if job is not None and not finished and tj > job.duration + 1.0:
            finished = True
            deposited = deposited_field.getSFFloat() * 1e9 if deposited_field else None
            summary = {"kind": "done", "t": round(tj, 3), "planned_s": round(job.duration, 3),
                       "extruded_mm3": round(hotend.deposited, 2), "skipped_mm3": round(hotend.skipped, 3),
                       "laid_mm3": round(deposited, 2) if deposited is not None else None,
                       "nozzle_crashes": int(crashes_field.getSFFloat()) if crashes_field else None}
            trace.write(json.dumps(summary) + "\n")
            print(f"[fdm_twin] done: {json.dumps(summary)}", flush=True)
        while layer_i < len(layers) and layers[layer_i][0] <= tj:
            trace.write(json.dumps({"kind": "layer", "t": round(tj, 3), "layer": layers[layer_i][1]}) + "\n")
            layer_i += 1
        if tj >= next_trace:
            next_trace = tj + TRACE_PERIOD_S
            trace.write(json.dumps({
                "kind": "sample", "t": round(tj, 4),
                "command": [round(v, 4) for v in cur], "measured": [round(v, 4) for v in measured],
                "extruded_mm3": round(hotend.deposited, 3),
            }) + "\n")
    trace.close()


if __name__ == "__main__":
    main()
