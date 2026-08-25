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

"""Per-step SOLVER cost for the OmniSim arm, on the same instrument as the
external arms -- so lane 1 can plot error against cost across engines.

THE PROBLEM THIS EXISTS TO SOLVE
--------------------------------
Lane 1's three arms do not time the same thing, and until now that made the
cost axis unusable across engines:

  * ``run_mujoco.py`` and ``run_pybullet.py`` wrap ``time.perf_counter()``
    around ``mj_step`` / ``p.stepSimulation()`` **only** -- scene build,
    recording and teardown are all outside the bracket. That is a solver-call
    cost and the two are directly comparable to each other.
  * ``run_omnisim.py`` reports whole-process wall time. It contains a Qt
    application, world parse, controller IPC, scene-graph writeback and a
    1.2-1.5 s one-time ``SolverMuJoCo`` construction. The field is named
    ``wall_ms_per_step_INDICATIVE_ONLY`` for exactly this reason.

Dividing one by the other measures our process launcher against their
function call. SPEC honesty rule: *"Do not read wall-clock across engines
from these rows."*

WHAT THIS MEASURES INSTEAD
--------------------------
``OMNISIM_NEWTON_SAVE_MJCF`` writes the compiled MjSpec -- **the exact model
the solver stepped** -- as MJCF at world finalize. This tool launches the
lane-1 world once with that variable set, loads the dumped MJCF into bare
``mujoco``, and times ``mj_step`` on it with byte-identical bracketing to
``run_mujoco.run_model``. The result answers:

    "What does the physics OmniSim ASKED FOR cost, priced on the same
     instrument as the MuJoCo arm?"

That is a fair cross-engine cost. It is deliberately **not** the same
question as "what does an OmniSim step cost me", and the two must not be
conflated -- so this tool reports both, plus their difference:

    solver_ms_per_step   the dumped model under bare mj_step   (comparable)
    process_ms_per_step  whole engine process / steps          (not comparable)
    engine_overhead_ms   process - solver                      (ours to own)

WHAT IT DOES **NOT** PROVE
--------------------------
* It is not OmniSim's step cost. The engine marshals newton state around
  ``mj_step``; that cost lives in ``engine_overhead_ms``, not here.
* The dumped model is not required to equal the model ``run_mujoco.py``
  hand-builds for the same scene. If ours carries more bodies or geoms, it
  costs more to step, and that IS a real cost of our translation layer --
  ``--report-model`` prints the shape of both so the difference is visible
  rather than assumed.
* n is small by default. Pass ``--repeats`` and read the spread; a single
  timing on a thermally throttled laptop is not a benchmark.

The bracket excludes model compile and data alloc, which are per-process and
would otherwise smear a 1.2-1.5 s startup across the step average -- the
mistake `docs/developer/physics-step-cost-optimization-plan.md` records as
having cost a whole campaign.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
sys.path.insert(0, str(HERE.parent))          # omnibench/
sys.path.insert(0, str(HERE))                 # lane1/

from common import engine_launch                        # noqa: E402
import run_omnisim                                      # noqa: E402


# ---------------------------------------------------------------------------
# 1. dump the exact model the solver stepped
# ---------------------------------------------------------------------------

def dump_mjcf(world, out_dir, timeout_s=180):
    """Launch `world` once with OMNISIM_NEWTON_SAVE_MJCF set. -> (mjcf, info).

    Scene knobs are taken from run_omnisim's own tables rather than
    re-declared here: a probe that launched the world with a different
    friction or contact stiffness than the arm would be pricing a different
    model than the one lane 1 scored, which is the whole failure this file
    exists to avoid.
    """
    world = Path(world)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = world.stem

    mjcf = out_dir / ("%s.model.xml" % stem)
    log_path = out_dir / ("%s.probe.log" % stem)
    console = out_dir / ("%s.probe.console.log" % stem)
    for p in (mjcf, log_path, Path(str(log_path) + ".newton.json")):
        if p.exists():
            p.unlink()

    env = engine_launch.build_env("newton", str(log_path), repo=REPO)
    env["OMNISIM_NEWTON_SAVE_MJCF"] = str(mjcf)
    # Never inherit a stale knob from the parent shell -- same discipline as
    # run_omnisim.py, and for the same reason.
    env.pop("OMNISIM_NEWTON_GROUND_MU", None)
    env.pop("OMNISIM_NEWTON_CONTACT_KD", None)

    test = _test_of(stem)
    mu = run_omnisim.NEWTON_MU.get(test)
    if mu is not None:
        env["OMNISIM_NEWTON_GROUND_MU"] = mu
    if test == "T1":
        env["OMNISIM_NEWTON_CONTACT_KD"] = "7"

    binpath = engine_launch.resolve_binary(REPO)
    t0 = time.perf_counter()
    rc, wall_s, timed_out = engine_launch.launch_once(
        binpath, str(world), env, str(console), timeout_s,
        extra_args=getattr(run_omnisim, "EXTRA_ARGS", ()), cwd=str(REPO))
    wall = time.perf_counter() - t0

    verdict = engine_launch.newton_verdict(str(log_path))
    info = {
        "world": str(world), "rc": rc, "timed_out": bool(timed_out),
        "launch_wall_s": round(wall, 3),
        "newton_verdict": verdict,
        "mjcf": str(mjcf) if mjcf.exists() else None,
        "mu": mu, "contact_kd": env.get("OMNISIM_NEWTON_CONTACT_KD"),
    }
    if not mjcf.exists():
        raise RuntimeError(
            "OMNISIM_NEWTON_SAVE_MJCF wrote nothing for %s (rc=%s, timed_out=%s). "
            "The dump happens at world FINALIZE, so a run that died or was "
            "killed before finalize produces no file -- that is not the same as "
            "a dump failure. Console: %s" % (world.name, rc, timed_out, console))
    # A dump that failed writes its own reason rather than dying silently.
    head = mjcf.read_text(errors="replace")[:200]
    if head.startswith("DUMP FAILED"):
        raise RuntimeError("%s: %s" % (world.name, head.strip()))
    return mjcf, info


def _test_of(stem):
    """'t3_roll_dt4' -> 'T3'. The worlds encode their scene in the filename."""
    return stem.split("_", 1)[0].upper()


# ---------------------------------------------------------------------------
# 2. time mj_step on it, bracketed exactly as run_mujoco does
# ---------------------------------------------------------------------------

def basic_time_step_s(world):
    """The .wbt's own basicTimeStep, in seconds.

    ⚠ WHY THIS IS NOT READ FROM THE DUMP. The dumped model's `opt.timestep`
    is a CONSTRUCTION-TIME DEFAULT, not the timestep the solver steps at.
    OmNewtonBackend assigns `_sv.mj_model.opt.timestep = sub_dt` inside the
    per-tick step loop, while OMNISIM_NEWTON_SAVE_MJCF writes the model at
    world FINALIZE -- before any tick has run. Measured on three lane-1
    worlds whose basicTimeStep is 1, 4 and 16 ms: all three dump
    `timestep=0.002`, a constant that tracks nothing about the world.

    So a tool that prices or audits the dump MUST supply the timestep from
    the .wbt itself. Pricing at the dumped 2 ms would silently value every
    scene at the wrong integration rate.
    """
    txt = Path(world).read_text(errors="replace")
    m = re.search(r"\bbasicTimeStep\s+([0-9.eE+-]+)", txt)
    if not m:
        raise RuntimeError("%s declares no basicTimeStep" % Path(world).name)
    return float(m.group(1)) / 1000.0


def substeps_for(world):
    """Solver substeps per engine tick. Env > WorldInfo.newtonSubsteps > 1."""
    env = os.environ.get("OMNISIM_NEWTON_SUBSTEPS")
    if env is not None:
        try:
            return max(1, int(env))
        except ValueError:
            pass
    m = re.search(r"\bnewtonSubsteps\s+([0-9]+)",
                  Path(world).read_text(errors="replace"))
    return max(1, int(m.group(1))) if m else 1


def time_mjcf(mjcf, steps, timestep_s, warmup=50, repeats=3,
              min_window_ms=250.0):
    """-> dict with per-repeat ms/step and the model shape.

    Bracket is `mj_step` alone, matching run_mujoco.run_model. Compile and
    mjData alloc sit outside it: they are per-process costs and folding them
    into a step average is the exact error the step-cost campaign warns about
    (a 1.2-1.5 s SolverMuJoCo construction smeared over a short run reports a
    tick that costs three times what it does).
    """
    import mujoco
    import numpy as np  # noqa: F401  (mujoco needs it loaded; keep explicit)

    m = mujoco.MjModel.from_xml_path(str(mjcf))
    dumped_dt = float(m.opt.timestep)
    m.opt.timestep = timestep_s        # see basic_time_step_s() -- required
    shape = {"nq": int(m.nq), "nv": int(m.nv), "nbody": int(m.nbody),
             "ngeom": int(m.ngeom), "nu": int(m.nu),
             "timestep": float(m.opt.timestep), "cone": int(m.opt.cone),
             "iterations": int(m.opt.iterations),
             "impratio": float(m.opt.impratio),
             # kept so a reader can see the override happened, and how far
             # the dump's own value was from the truth
             "dumped_timestep_IGNORED": dumped_dt}

    # AUTO-SIZE THE MEASUREMENT WINDOW.
    # Lane-1 scenes are tiny -- 3 bodies, ~0.006 ms/step -- so a nominal 1000
    # steps spans about 6 ms of wall time. At that scale one OS scheduling
    # slice dominates the sample: measured spreads of 14% median and 86% worst
    # across the corpus, which is not a number anyone should publish. Grow the
    # count until each repeat spans at least `min_window_ms` so the timer is
    # measuring the solver rather than the operating system.
    if min_window_ms > 0:
        probe_n = 200
        d = mujoco.MjData(m)
        for _ in range(warmup):
            mujoco.mj_step(m, d)
        t0 = time.perf_counter()
        for _ in range(probe_n):
            mujoco.mj_step(m, d)
        per = (time.perf_counter() - t0) / probe_n          # seconds/step
        if per > 0:
            steps = max(steps, int(math.ceil(min_window_ms / 1000.0 / per)))
        steps = min(steps, 2_000_000)                       # sanity ceiling

    per_repeat = []
    for _ in range(max(1, repeats)):
        d = mujoco.MjData(m)
        for _ in range(warmup):          # let contacts establish; excluded
            mujoco.mj_step(m, d)
        wall = 0.0
        for _ in range(steps):
            t0 = time.perf_counter()
            mujoco.mj_step(m, d)
            wall += time.perf_counter() - t0
        per_repeat.append(wall * 1000.0 / steps)

    return {"ms_per_step_repeats": [round(v, 6) for v in per_repeat],
            "ms_per_step": round(statistics.median(per_repeat), 6),
            "ms_per_step_min": round(min(per_repeat), 6),
            "ms_per_step_max": round(max(per_repeat), 6),
            "spread_pct": round(
                100.0 * (max(per_repeat) - min(per_repeat))
                / max(statistics.median(per_repeat), 1e-12), 1),
            "steps": steps, "warmup": warmup, "model": shape}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Price the model OmniSim's solver stepped, on the same "
                    "instrument as lane 1's MuJoCo arm.")
    ap.add_argument("--worlds", nargs="+", required=True,
                    help="lane-1 .wbt files (or a glob already expanded)")
    ap.add_argument("--out", required=True, help="output dir for dumps + jsonl")
    ap.add_argument("--steps", type=int, default=1000)
    ap.add_argument("--warmup", type=int, default=50)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--min-window-ms", type=float, default=250.0,
                    help="grow the step count until each repeat spans at "
                         "least this long. Lane-1 scenes cost ~0.006 ms/step, "
                         "so a nominal 1000 steps is a 6 ms window and the "
                         "OS scheduler dominates it. 0 disables.")
    ap.add_argument("--reuse-dumps", action="store_true",
                    help="skip the engine launch when a dump already exists")
    ap.add_argument("--report-model", action="store_true",
                    help="print the model shape, so a cost difference that is "
                         "really a MODEL difference is visible not assumed")
    a = ap.parse_args(argv)

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rows_path = out / "model_step_cost.jsonl"

    try:
        from common import results as _res
        machine = _res.machine_fingerprint()
    except Exception as e:                      # never lose rows over metadata
        machine = {"note": "fingerprint unavailable: %r" % (e,)}

    written = 0
    with open(rows_path, "a", encoding="utf-8") as fh:
        for w in a.worlds:
            w = Path(w)
            stem = w.stem
            mjcf = out / ("%s.model.xml" % stem)
            try:
                if a.reuse_dumps and mjcf.exists():
                    info = {"world": str(w), "reused_dump": True,
                            "mjcf": str(mjcf)}
                else:
                    mjcf, info = dump_mjcf(w, out)
                dt_s = basic_time_step_s(w)
                nsub = substeps_for(w)
                t = time_mjcf(mjcf, a.steps, dt_s / nsub, a.warmup,
                              a.repeats, a.min_window_ms)
                # What the ENGINE pays per tick, and per second of sim time --
                # the two forms a Pareto curve can actually use. mj_step cost
                # is ~independent of dt, so a smaller dt costs more per
                # sim-second purely by calling it more often.
                t["substeps"] = nsub
                t["basic_time_step_s"] = dt_s
                t["ms_per_engine_step"] = round(t["ms_per_step"] * nsub, 6)
                t["ms_per_sim_second"] = round(
                    t["ms_per_step"] * nsub / dt_s, 4)
            except Exception as e:
                row = {"world": str(w), "status": "gap", "error": repr(e)}
                fh.write(json.dumps(row) + "\n")
                fh.flush()
                print("[model_step_cost] %-28s GAP  %r" % (stem, e))
                continue

            row = {"suite": "omnibench/lane1-cost", "world": str(w),
                   "test": _test_of(stem), "status": "ok",
                   "instrument": "bare mj_step on OMNISIM_NEWTON_SAVE_MJCF dump; "
                                 "bracket matches run_mujoco.run_model exactly "
                                 "(compile + mjData alloc EXCLUDED)",
                   "launch": info, "machine": machine}
            row.update(t)
            fh.write(json.dumps(row) + "\n")
            fh.flush()
            written += 1
            extra = ""
            if a.report_model:
                s = t["model"]
                extra = ("  [nbody=%d ngeom=%d nv=%d cone=%d iters=%d dt=%g]"
                         % (s["nbody"], s["ngeom"], s["nv"], s["cone"],
                            s["iterations"], s["timestep"]))
            print("[model_step_cost] %-28s %.5f ms/step (spread %.1f%%)%s"
                  % (stem, t["ms_per_step"], t["spread_pct"], extra))

    print("[model_step_cost] wrote %d rows -> %s" % (written, rows_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
