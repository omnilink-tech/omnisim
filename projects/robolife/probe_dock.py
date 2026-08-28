#!/usr/bin/env python3
"""RoboLife docking gate driver (DESIGN.md "Verification gates", A).

Writes the probe world (1 Husky running `robolife_probe_dock` + 2 loose
modules + walls) with rl/worldgen.py, runs ONE headless engine on it, and
prints the measured gate: presence before/after lock(), isLocked, max
socket<->plug separation over a 4 m tow, module displacement after unlock(),
engine ms/step, the 2 m drive and 90 deg turn {commanded, achieved, error},
and braking distance bare vs. with the 6 kg battery docked.

    python projects/robolife/probe_dock.py [--duration 60] [--no-run]

Result: projects/robolife/_run/probe_dock_result.json (written by the
controller) plus the engine log + .newton.json sidecar next to it.
"""
import argparse
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from rl import worldgen  # noqa: E402

RUN = os.path.join(HERE, "_run")
WORLD = os.path.join(RUN, "probe_dock.omniworld")
RESULT = os.path.join(RUN, "probe_dock_result.json").replace("\\", "/")
LOG = os.path.join(RUN, "probe_dock_log.txt")


def _paths(tag):
    """Per-arm file names so a sweep never clobbers the canonical result."""
    global WORLD, RESULT, LOG
    suffix = ("_" + tag) if tag else ""
    WORLD = os.path.join(RUN, "probe_dock%s.omniworld" % suffix)
    RESULT = os.path.join(RUN, "probe_dock_result%s.json" % suffix).replace("\\", "/")
    LOG = os.path.join(RUN, "probe_dock_log%s.txt" % suffix)


def write_probe_world(arena=24.0, ground_mu=worldgen.DEFAULT_GROUND_MU,
                      wheel_torque=worldgen.DEFAULT_WHEEL_TORQUE, quick=False,
                      dt=8, ke=8000, kd=200, cone="elliptic", impratio=10,
                      substeps=worldgen.DEFAULT_SUBSTEPS,
                      wheel_collider="cylinder", pose_wrap=True, dock_only=False):
    args = ["--out", RESULT, "--module", "MODULE_0", "--module-type", "battery"]
    if quick:
        args.append("--quick")
    if dock_only:
        args.append("--dock-only")
    robots = [{"slot": 0, "pos": (-7.0, -4.0), "yaw": 0.0, "supervisor": True,
               "controller_args": args}]
    modules = [{"id": 0, "type": "battery", "pos": (2.0, 0.0), "yaw": 0.0, "loose": True},
               {"id": 1, "type": "armor", "pos": (2.0, 4.0), "yaw": 1.0, "loose": True}]
    return worldgen.write_world(robots, modules, WORLD, controller="robolife_probe_dock",
                                arena=arena, title="RoboLife docking probe",
                                director_controller="<none>", ground_mu=ground_mu,
                                wheel_torque=wheel_torque, dt=dt, ke=ke, kd=kd, cone=cone,
                                impratio=impratio, substeps=substeps,
                                wheel_collider=wheel_collider, pose_wrap=pose_wrap)


def run(duration, extra_env=()):
    if os.path.exists(RESULT):
        os.remove(RESULT)
    env = dict(os.environ)
    for kv in extra_env:
        k, _, v = kv.partition("=")
        env[k] = v
    env["OMNISIM_LOG_PATH"] = LOG
    env["OMNISIM_HOME"] = REPO
    cmd = [sys.executable, "-m", "omnisim", "run-headless", os.path.relpath(WORLD, REPO),
           "--duration", str(int(round(duration)))]
    print("[probe_dock] $", " ".join(cmd))
    proc = subprocess.run(cmd, cwd=REPO, env=env, capture_output=True, text=True,
                          timeout=duration + 120)
    return proc


def summarise(proc):
    out = {"exit_code": proc.returncode if proc else None}
    side = LOG + ".newton.json"
    out["sidecar"] = json.load(open(side, encoding="utf-8")) if os.path.exists(side) else None
    if os.path.exists(LOG):
        txt = open(LOG, encoding="utf-8", errors="replace").read()
        out["registered"] = re.findall(r"registered \d+ dynamic[^\n]*", txt)[:3]
        bad = [ln for ln in txt.splitlines() if re.search(r"\b(WARNING|ERROR)\b", ln)]
        out["warnings_errors"] = {"count": len(bad), "first": bad[:12]}
    out["result"] = json.load(open(RESULT, encoding="utf-8")) if os.path.exists(RESULT) else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--no-run", action="store_true", help="only write the world")
    ap.add_argument("--mu", type=float, default=worldgen.DEFAULT_GROUND_MU,
                    help="WorldInfo.newtonGroundMu (skid-steer turning depends on it)")
    ap.add_argument("--torque", type=float, default=worldgen.DEFAULT_WHEEL_TORQUE,
                    help="wheel RotationalMotor.maxTorque [N m]")
    ap.add_argument("--tag", default="", help="suffix for world/log/result files")
    ap.add_argument("--quick", action="store_true", help="wheel-law tests only (skip docking)")
    ap.add_argument("--dt", type=int, default=8)
    ap.add_argument("--ke", type=float, default=8000)
    ap.add_argument("--kd", type=float, default=200)
    ap.add_argument("--cone", default="elliptic")
    ap.add_argument("--impratio", type=float, default=10)
    ap.add_argument("--substeps", type=int, default=worldgen.DEFAULT_SUBSTEPS,
                    help="WorldInfo.newtonSubsteps")
    ap.add_argument("--env", action="append", default=[], help="KEY=VAL for the engine")
    ap.add_argument("--wheel", default="cylinder", choices=("cylinder", "sphere"),
                    help="wheel collider primitive")
    ap.add_argument("--no-pose-wrap", action="store_true", help="author robots/modules top-level")
    ap.add_argument("--dock-only", action="store_true", help="skip the wheel-law tests")
    a = ap.parse_args()
    os.makedirs(RUN, exist_ok=True)
    _paths(a.tag)
    w = write_probe_world(ground_mu=a.mu, wheel_torque=a.torque, quick=a.quick, dt=a.dt,
                          ke=a.ke, kd=a.kd, cone=a.cone, impratio=a.impratio,
                          substeps=a.substeps, wheel_collider=a.wheel,
                          pose_wrap=not a.no_pose_wrap, dock_only=a.dock_only)
    print("[probe_dock] config", json.dumps({k: w[k] for k in ("ground_mu", "wheel_torque")}),
          "dt", a.dt, "substeps", a.substeps, "ke", a.ke, "kd", a.kd, a.cone,
          "impratio", a.impratio, "wheel", a.wheel, "env", a.env)
    print("[probe_dock] wrote", w["path"])
    if a.no_run:
        return 0
    proc = run(a.duration, a.env)
    tail = (proc.stdout or "")[-3000:]
    print(tail)
    if proc.returncode != 0:
        print((proc.stderr or "")[-3000:])
    s = summarise(proc)
    r = s["result"]
    print("[probe_dock] exit=%s sidecar=%s registered=%s warn/err=%s" % (
        s["exit_code"], (s["sidecar"] or {}).get("finalised"), s.get("registered"),
        (s.get("warnings_errors") or {}).get("count")))
    if r is None:
        print("[probe_dock] NO RESULT FILE -- the controller never finished")
        return 2
    for k in ("drive_2m", "turn_90", "brake_bare", "creep", "lock", "tow_4m", "brake_docked",
              "unlock", "engine_ms_per_step", "verdict"):
        print("  %-18s %s" % (k, json.dumps(r.get(k))))
    return 0 if r.get("verdict", {}).get("pass") else 1


if __name__ == "__main__":
    sys.exit(main())
