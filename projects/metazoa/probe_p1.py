#!/usr/bin/env python3
"""Metazoa P1 gate driver (DESIGN.md "P1 gate"). Owner: A.

Writes the two probe worlds with mz/worldgen.py, runs ONE headless engine on
each under the thermal protocol (GPU <= 70 C before launch, one engine at a
time, --duration <= 60, no orphan afterwards), and prints every gate number
from the controller's JSON plus the engine-log evidence (registered bodies,
motorized hinges, weld lines, WARNING/ERROR count, .newton.json sidecar).

    python projects/metazoa/probe_p1.py                 # both worlds
    python projects/metazoa/probe_p1.py --only probe    # 12-cell gate world
    python projects/metazoa/probe_p1.py --only cost     # 24-cell cost world
    python projects/metazoa/probe_p1.py --no-run        # just write the worlds
    python projects/metazoa/probe_p1.py --A 0.9 --omega 6 --dphi 1.2 --tag sweep1

Results: projects/metazoa/_run/probe_p1.json (the controller writes the
gate numbers; this driver merges the cost world's numbers under "cost" and
the log evidence under "engine"), logs next to it.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from mz import cell as C          # noqa: E402
from mz import worldgen as W      # noqa: E402

RUN = os.path.join(HERE, "_run")
WORLDS = os.path.join(HERE, "worlds")
ARENA_PROBE = 12.0
ARENA_COST = 12.0
GAP = 0.01
NEG_GAP = 0.10

PALETTE = [(0.95, 0.35, 0.25), (0.25, 0.75, 0.95), (0.95, 0.8, 0.2), (0.6, 0.9, 0.3),
           (0.85, 0.4, 0.9), (0.95, 0.6, 0.2)]


def gpu_temp():
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader"],
                             capture_output=True, text=True, timeout=20).stdout.strip()
        return int(out.splitlines()[0].strip())
    except Exception:
        return None


def thermal_gate(limit=70):
    """Binding: never launch above `limit` C; wait 60 s and re-check."""
    while True:
        t = gpu_temp()
        print("[probe_p1] GPU %s C" % t, flush=True)
        if t is None or t <= limit:
            return t
        print("[probe_p1] above %d C -- waiting 60 s" % limit, flush=True)
        time.sleep(60)


def engines_alive():
    """`Get-Process -Name omnisim-bin` -- pgrep does not exist here."""
    try:
        out = subprocess.run(["powershell.exe", "-NoProfile", "-Command",
                              "@(Get-Process -Name omnisim-bin -ErrorAction SilentlyContinue).Count"],
                             capture_output=True, text=True, timeout=30).stdout.strip()
        return int(out or "0")
    except Exception as exc:
        print("[probe_p1] Get-Process failed: %s" % exc, flush=True)
        return -1


SPAWN = {"z": C.SPAWN_Z}


def probe_cells():
    """12 cells: chain A (0-3), alternating chain B (4-7), negative pair
    (8, 9), lone flipper (10), sign witness (11)."""
    cells = []
    for k, p in enumerate(C.chain_poses((-2.5, 3.0, SPAWN["z"], 0.0, 0.0), 4, GAP, [0, 0, 0, 0])):
        cells.append({"id": k, "pos": p["pos"], "yaw": p["yaw"], "roll": p["roll"], "colour": PALETTE[0]})
    for k, p in enumerate(C.chain_poses((-2.5, 0.5, SPAWN["z"], 0.0, 0.0), 4, GAP, [0, 1, 0, 1])):
        cells.append({"id": 4 + k, "pos": p["pos"], "yaw": p["yaw"], "roll": p["roll"], "colour": PALETTE[1]})
    for k, p in enumerate(C.chain_poses((-2.5, -2.0, SPAWN["z"], 0.0, 0.0), 2, NEG_GAP, [0, 0])):
        cells.append({"id": 8 + k, "pos": p["pos"], "yaw": p["yaw"], "roll": p["roll"], "colour": PALETTE[2]})
    cells.append({"id": 10, "pos": [-2.5, -4.0, SPAWN["z"]], "yaw": 0.0, "roll": 0.0, "colour": PALETTE[3]})
    # Own row: a v2 cell on rollers has no rolling resistance, and the sign
    # witness's single fold sent it 5.75 m along -x into the negative pair
    # (P1b sweep runs s1-s4, README). Nothing else lives at y = -5.
    cells.append({"id": 11, "pos": [3.5, -5.0, SPAWN["z"]], "yaw": 0.0, "roll": 0.0, "colour": PALETTE[4]})
    return cells


def cost_cells(n=24):
    return [{"id": i, "pos": [x, y, SPAWN["z"]], "yaw": 0.0, "roll": 0.0,
             "colour": PALETTE[i % len(PALETTE)]}
            for i, (x, y) in enumerate(W.grid_positions(n, ARENA_COST, spacing=1.0))]


def write_worlds(a, tag):
    suffix = ("_" + tag) if tag else ""
    probe_world = os.path.join(WORLDS, "probe_p1%s.omniworld" % suffix)
    cost_world = os.path.join(WORLDS, "probe_p1_cost%s.omniworld" % suffix)
    out_probe = os.path.join(RUN, "probe_p1%s.json" % suffix).replace("\\", "/")
    out_cost = os.path.join(RUN, "probe_p1_cost%s.json" % suffix).replace("\\", "/")
    args = ["--out", out_probe, "--A", str(a.A), "--omega", str(a.omega), "--dphi", str(a.dphi),
            "--steer", str(a.steer), "--fade", str(a.fade), "--flip-sign", a.flip_sign,
            "--flip-period", str(a.flip_period), "--flip-fold", str(a.flip_fold)]
    if a.no_lock:
        args.append("--no-lock")
    if a.no_flip:
        args.append("--no-flip")
    rp = W.write_world(probe_cells(), probe_world, controller="metazoa_probe_p1", arena=ARENA_PROBE,
                       controller_args=args, substeps=a.substeps, rollers=a.rollers,
                       title="Metazoa P1%s probe -- 12 cells" % {False: "", True: "b", "v3": "c"}[a.rollers])
    rc = W.write_world(cost_cells(a.cost_cells), cost_world, controller="metazoa_probe_p1",
                       arena=ARENA_COST, controller_args=["--cost", "--out", out_cost, "--cells",
                                                          str(a.cost_cells)],
                       substeps=a.substeps, rollers=a.rollers,
                       title="Metazoa P1%s cost -- %d resting cells"
                       % ({False: "", True: "b", "v3": "c"}[a.rollers], a.cost_cells))
    for res in (rp, rc):
        print("[probe_p1] wrote %s  (%d cells -> expect %d dynamic, %d motorized)"
              % (os.path.relpath(res["path"], REPO), res["n"], res["expect"]["dynamic_bodies"],
                 res["expect"]["motorized"]), flush=True)
    return {"probe": (probe_world, out_probe), "cost": (cost_world, out_cost)}


def run_engine(world, out_json, log, duration, extra_env=()):
    if os.path.exists(out_json):
        os.remove(out_json)
    for p in (log, log + ".newton.json"):
        if os.path.exists(p):
            os.remove(p)
    if engines_alive() > 0:
        raise SystemExit("[probe_p1] an omnisim-bin is already running -- one engine at a time")
    temp = thermal_gate()
    env = dict(os.environ)
    env["OMNISIM_LOG_PATH"] = log
    env["OMNISIM_HOME"] = REPO
    for kv in extra_env:
        k, _, v = kv.partition("=")
        env[k] = v
    cmd = [sys.executable, "-m", "omnisim", "run-headless", os.path.relpath(world, REPO),
           "--duration", str(int(min(60, duration)))]
    print("[probe_p1] $", " ".join(cmd), flush=True)
    t0 = time.time()
    proc = subprocess.run(cmd, cwd=REPO, env=env, capture_output=True, text=True,
                          timeout=duration + 180)
    wall = time.time() - t0
    for _ in range(10):
        if engines_alive() == 0:
            break
        time.sleep(1)
    alive = engines_alive()
    if alive:
        print("[probe_p1] WARNING: %d omnisim-bin still alive after the run" % alive, flush=True)
    return {"exit_code": proc.returncode, "wall_s": round(wall, 1), "gpu_temp_before": temp,
            "engines_alive_after": alive,
            "stdout_tail": proc.stdout[-1500:], "stderr_tail": proc.stderr[-800:]}


def log_evidence(log):
    ev = {"log": log}
    side = log + ".newton.json"
    ev["sidecar"] = json.load(open(side, encoding="utf-8")) if os.path.exists(side) else None
    if not os.path.exists(log):
        ev["log_missing"] = True
        return ev
    txt = open(log, encoding="utf-8", errors="replace").read()
    ev["registered"] = re.findall(r"registered \d+ dynamic[^\n]*", txt)[:3]
    ev["motorized"] = len(re.findall(r"motorized", txt))
    ev["weld_lines"] = [ln.strip() for ln in txt.splitlines() if re.search(r"weld", ln, re.I)][:12]
    ev["connector_lines"] = [ln.strip() for ln in txt.splitlines()
                             if re.search(r"Connector", ln) and re.search(r"WARN|ERROR", ln)][:12]
    bad = [ln.strip() for ln in txt.splitlines() if re.search(r"\b(WARNING|ERROR)\b", ln)]
    ev["warnings_errors"] = {"count": len(bad), "first": bad[:12]}
    probe_lines = [ln.strip() for ln in txt.splitlines() if "[probe_p1]" in ln]
    ev["probe_lines"] = probe_lines[-20:]
    return ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", choices=("probe", "cost"), default=None)
    ap.add_argument("--no-run", action="store_true")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--tag", default="")
    ap.add_argument("--A", type=float, default=0.8)
    ap.add_argument("--omega", type=float, default=5.0)
    ap.add_argument("--dphi", type=float, default=1.57)
    ap.add_argument("--steer", type=float, default=0.5)
    ap.add_argument("--fade", type=float, default=1.0)
    ap.add_argument("--flip-sign", default="auto", choices=("auto", "+", "-"))
    ap.add_argument("--flip-period", type=float, default=2.0)
    ap.add_argument("--flip-fold", type=float, default=2.4)
    ap.add_argument("--cost-cells", type=int, default=24)
    ap.add_argument("--no-lock", action="store_true", help="control arm: never write isLocked")
    ap.add_argument("--max-torque", type=float, default=None,
                    help="override mz.cell.MAX_TORQUE (DESIGN 0.6) for a weld-load A/B")
    ap.add_argument("--substeps", type=int, default=4, help="WorldInfo.newtonSubsteps (4 = 2 ms solver step)")
    ap.add_argument("--v2", action="store_true", help="cell v2: passive belly rollers, maxTorque 0.35")
    ap.add_argument("--v3", action="store_true", help="cell v3: belly + side rollers (dock rotations {0,1})")
    ap.add_argument("--no-flip", action="store_true", help="drop the lone-flip gate (P1b decision)")
    ap.add_argument("--env", action="append", default=[], help="KEY=VAL for the engine")
    a = ap.parse_args()
    os.makedirs(RUN, exist_ok=True)
    if a.max_torque is not None:
        C.MAX_TORQUE = C.MAX_TORQUE_V2 = a.max_torque
    a.rollers = "v3" if a.v3 else bool(a.v2)
    SPAWN["z"] = C.spawn_z(bool(a.rollers))
    worlds = write_worlds(a, a.tag)
    if a.no_run:
        return
    suffix = ("_" + a.tag) if a.tag else ""
    summary = {"tag": a.tag, "params": vars(a)}
    lanes = [a.only] if a.only else ["probe", "cost"]
    for lane in lanes:
        world, out_json = worlds[lane]
        log = os.path.join(RUN, "probe_p1_%s%s.log" % (lane, suffix))
        run = run_engine(world, out_json, log, a.duration, a.env)
        ev = log_evidence(log)
        res = json.load(open(out_json, encoding="utf-8")) if os.path.exists(out_json) else None
        summary[lane] = {"run": run, "engine": ev, "result_file": out_json,
                         "result_present": res is not None}
        print("[probe_p1] %s: exit %s, %.1f s wall, sidecar %s, %s, motorized %s, warn/err %s"
              % (lane, run["exit_code"], run["wall_s"],
                 (ev.get("sidecar") or {}).get("finalised"), ev.get("registered"),
                 ev.get("motorized"), ev.get("warnings_errors", {}).get("count")), flush=True)
        for ln in ev.get("probe_lines", []):
            print("   ", ln[-160:])
        if res is None:
            print("[probe_p1] %s: NO RESULT FILE -- read %s" % (lane, log), flush=True)
            print(run["stdout_tail"][-800:])
        elif lane == "cost":
            print("[probe_p1] COST:", json.dumps(res.get("cost"), indent=1)[:800])
        else:
            for key in ("sign", "chain_a", "chain_b", "negative", "flip", "engine_ms_per_step"):
                blk = dict(res.get(key) or {})
                blk.pop("track", None)
                blk.pop("sep_trace", None)
                print("[probe_p1] %s: %s" % (key.upper(), json.dumps(blk, default=str)))
    # merge: the cost world's numbers + the engine evidence into probe_p1.json
    merged_path = worlds["probe"][1]
    merged = json.load(open(merged_path, encoding="utf-8")) if os.path.exists(merged_path) else {}
    if "cost" in summary and summary["cost"]["result_present"]:
        merged["cost"] = json.load(open(worlds["cost"][1], encoding="utf-8")).get("cost")
        merged["cost_engine"] = summary["cost"]["engine"]
        merged["cost_run"] = summary["cost"]["run"]
    if "probe" in summary:
        merged["engine"] = summary["probe"]["engine"]
        merged["run"] = summary["probe"]["run"]
    if merged:
        with open(merged_path, "w", encoding="utf-8") as f:
            json.dump(merged, f, indent=1)
        print("[probe_p1] merged -> %s" % os.path.relpath(merged_path, REPO))
    print("[probe_p1] engines alive at exit: %d" % engines_alive())


if __name__ == "__main__":
    main()
