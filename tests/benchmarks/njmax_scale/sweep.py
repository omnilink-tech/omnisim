"""Run a batch of fleet configs and print one row per run."""
from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run as R  # noqa: E402

HDR = ("%-16s %4s %-11s %6s %6s | %8s %8s %8s %7s | %6s %-14s %s"
       % ("tag", "N", "solver", "njmax", "sub", "disp_min", "disp_max",
          "spread", "w_min", "wall", "peak/cap", "OVF"))


def one(tag, n, solver="mujoco_warp", njmax=0, nconmax=0, substeps=2,
        z=0.3, spacing=12.0, duration_ms=12000, timeout=900, stats=10,
        env_extra=None, ctrl=None, gen_extra=None):
    kw = dict(solver=solver, njmax=njmax, nconmax=nconmax, substeps=substeps,
              z=z, spacing=spacing)
    if ctrl:
        kw["ctrl"] = ctrl
    if gen_extra:
        kw.update(gen_extra)
    w = R.gen(tag, n, **kw)
    r = R.run(tag, w, duration_ms=duration_ms, timeout=timeout, stats=stats,
              env_extra=env_extra)
    s = R.score(r)
    s["newton"] = R.newton_peaks(r["nlog"])
    s["log_hits"] = R.log_greps(r["log"])
    s["cfg"] = dict(n=n, solver=solver, njmax=njmax, nconmax=nconmax,
                    substeps=substeps, z=z, spacing=spacing,
                    duration_ms=duration_ms, env=env_extra or {})
    with open(os.path.join(R.OUT, tag + ".score.json"), "w") as f:
        json.dump(s, f, indent=1)
    s["connect_fails"] = R.connect_fails(r["log"])
    # engine-side finalize wall time: gap between process start and the first
    # newton_solver.log timestamp is not available, so use the log's own marks.
    pk = s["newton"].get("peak_last") or ""
    cap = ""
    if "nefc=" in pk:
        cap = pk.split("nefc=")[1].split(" ")[0]
    print("%-16s %4d %-11s %6s %6s | %8s %8s %8s %7s | %6s %-14s %-9s cfail=%d"
          % (tag, n, solver, njmax or "def", substeps,
             s.get("disp_min"), s.get("disp_max"), s.get("disp_spread"),
             s.get("wheel_w_min"), s.get("wall_s"), cap,
             "OVERFLOW" if s["newton"]["overflow_lines"] else
             ("-" if s["newton"]["n_peaks"] else "no-telem"),
             s["connect_fails"]),
          flush=True)
    return s


if __name__ == "__main__":
    print(HDR, flush=True)
    arg = sys.argv[1]
    if os.path.exists(arg):
        with open(arg) as f:
            plan = json.load(f)
    else:
        plan = json.loads(arg)
    for item in plan:
        one(**item)
