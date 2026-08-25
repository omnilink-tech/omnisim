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

"""OmniBench lane 3a — determinism grading (SPEC.md lane 3).

Runs the identical multi-contact world (5 spheres onto a box, seed 42) TWICE
from cold (fresh engine process each time), records full per-step trajectories
(position + velocity, %.17g), diffs them, and grades:

    bitwise    every recorded value identical (text-equal at %.17g, i.e. the
               doubles round-trip identically)
    tolerance  max |delta| < 1e-9
    divergent  otherwise

reporting the max deviation and the first step where any divergence appears.
Then a COLD-vs-WARM comparison: ONE engine process that loads, records,
worldReload()s, records again — run1 (cold) vs run2 (warm reload) diffed the
same way.

Backend selection / verification:
    --backend newton  OMNISIM_REQUIRE_NEWTON=1 (fail loudly if Newton cannot
                      initialise); verified by the sidecar existing with
                      degraded == false.
    --backend ode     RETIRED 2026-08-08 and REFUSED. src/ode was DELETED
                      (commit bdc02139). It also rested on a bad inference:
                      "ODE drove it" was concluded from the ABSENCE of the
                      Newton sidecar, but absence also describes a run that
                      never reached world-finalize. Worse, bitwise determinism
                      on ODE was one of this suite's headline results
                      (docs/benchmarks/determinism-scope.md) — that claim is
                      now HISTORICAL and cannot be re-measured here.

Exit code 0 proves nothing about a child run — the recording must exist and
have >= steps/2 rows, else the run is retried (controller launch race, see
scripts/dev/physics_oracle.py) and finally reported as a failure.

NOTE (Windows): launch from PowerShell, not MSYS bash, for the newton backend —
an MSYS environment can make the engine's embedded interpreter miss warp and
silently fall back to ODE (the sidecar check will catch it, but the run wastes
time). See AGENTS.md.

Usage:
    python tests/benchmarks/omnibench/lane3/determinism.py --backend newton
    python tests/benchmarks/omnibench/lane3/determinism.py --backend ode
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _row  # noqa: E402
from common import engine_launch  # noqa: E402  (_row put omnibench on sys.path)

REPO = _row.repo_root()
WORLD = _row.LANE3 / "worlds" / "lane3_determinism.wbt"
DT_MS = 4.0          # WorldInfo.basicTimeStep in the world file

# --world/--dt-ms override these at runtime. The default scene is deliberately
# light (5 spheres), which is the WRONG scene for the question "is the GPU
# solver deterministic": GPU nondeterminism comes from many contacts being
# accumulated concurrently, where thread scheduling changes the summation
# order. Pass a contact-heavy world on the mujoco_warp path to test that.
_WORLD_OVERRIDE = None
TOL = 1e-9
# 8 attempts with growing settle (2,4,...,14 s): matches the calibrated margin in
# scripts/dev/physics_oracle.py for the back-to-back-launch controller race
# (observed here: 5 attempts were NOT enough right after two cold runs).
ATTEMPTS = 8
RUN_TIMEOUT_S = 300


def warn_if_already_running():
    """A concurrent omnisim-bin can hold controller comms resources -> a flaky
    'no CSV'. We never kill it (it may be the user's); we just warn."""
    try:
        out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq omnisim-bin.exe"],
                             timeout=15, capture_output=True, text=True)
        if "omnisim-bin.exe" in (out.stdout or ""):
            print("[l3-det] WARNING: an omnisim-bin is already running; a 'no result'")
            print("         below may be a resource collision, not real nondeterminism.")
    except Exception:
        pass


def _sidecar_verdict(log_path: str):
    """Return (dict-or-None). OmLog deletes a stale sidecar at startup, so the
    file's presence == Newton drove THIS run."""
    v = engine_launch.newton_verdict(log_path)
    if not v.get("present"):
        return None
    if v.get("error"):
        return {"parse_error": True}
    return v


def run_once(backend: str, out_dir: str, steps: int, mode: str, label: str):
    """One engine launch. Returns (wall_s, sidecar, err)."""
    bin_path = _row.omnisim_binary()
    log_path = os.path.join(out_dir, "engine_%s.log.txt" % label)
    # Shared launcher owns the env hygiene + backend select (+ Linux runtime
    # vars / xvfb-run); the lane's recorder knobs pass through via `extra`.
    env = engine_launch.build_env(backend, log_path, repo=REPO,
                                  extra={"OMNIBENCH_L3_OUT_DIR": out_dir,
                                         "OMNIBENCH_L3_STEPS": str(steps),
                                         "OMNIBENCH_L3_MODE": mode})
    # Launch shape notes (physics_oracle-proven, now owned by engine_launch):
    # no --port (a local controller does not inherit it and then can't
    # connect); stdout to a REAL file (a DEVNULL handle correlates with the
    # flaky no-CSV first launch on Windows).
    stdout_log = os.path.join(out_dir, "stdout_%s.log" % label)
    rc, wall, timed_out = engine_launch.launch_once(
        bin_path, _WORLD_OVERRIDE or WORLD, env, stdout_log, RUN_TIMEOUT_S)
    err = ""
    if timed_out:
        err = ("timeout after %ds (CSV is flushed per step; partial data may "
               "exist)" % RUN_TIMEOUT_S)
    return wall, _sidecar_verdict(log_path), err


def load_csv(path):
    """CSV -> (header, {step: [raw string tokens]}). Strings kept for the
    bitwise verdict; floats parsed on demand for deviation measurement."""
    p = Path(path)
    if not p.exists():
        return None, None
    rows = p.read_text(encoding="utf-8").splitlines()
    if len(rows) < 2:
        return None, None
    header = rows[0].split(",")
    data = {}
    for r in rows[1:]:
        toks = r.split(",")
        if len(toks) != len(header):
            continue
        try:
            data[int(toks[0])] = toks[1:]
        except ValueError:
            continue
    return header, data


def capture(backend, out_dir, steps, mode, label, expected_files):
    """Run + retry on the known controller-launch race (growing settle —
    the shared engine_launch retry loop)."""

    def attempt_fn(attempt):
        # clean slate for this attempt
        for f in expected_files + ["run_w1.done"]:
            fp = os.path.join(out_dir, f)
            if os.path.exists(fp):
                os.remove(fp)
        wall, sidecar, err = run_once(backend, out_dir, steps, mode,
                                      "%s_a%d" % (label, attempt))
        results = [load_csv(os.path.join(out_dir, f)) for f in expected_files]
        # Was `steps // 2`, which silently graded a run that died halfway as a
        # full result. A short run is now a retry, and if it persists the caller
        # reports it rather than grading half a window.
        ok = all(h is not None and len(d) >= max(2, int(steps * 0.95))
                 for h, d in results)
        if ok and attempt > 1:
            print("[l3-det]   %s recovered on attempt %d" % (label, attempt))
        elif not ok:
            print("[l3-det]   %s attempt %d yielded no/short data (%s)"
                  % (label, attempt, err or "empty CSV"))
        return results, wall, sidecar, err, ok

    (results, wall, sidecar, err, ok), _ = engine_launch.launch_with_retries(
        attempts=ATTEMPTS, attempt_fn=attempt_fn,
        success_fn=lambda r: r[4], label="l3-det %s" % label)
    if ok:
        return results, wall, sidecar, err
    return results, wall, sidecar, err or (
        "no/short recording after %d attempts" % ATTEMPTS)


def diff(run_a, run_b):
    """Grade two recordings. Returns dict with grade, max_abs_dev, first steps."""
    ha, da = run_a
    hb, db = run_b
    if ha is None or hb is None:
        return {"grade": "no_data", "max_abs_dev": None,
                "first_div_step": None, "first_tol_exceed_step": None,
                "compared_steps": 0}
    if ha != hb:
        return {"grade": "divergent", "max_abs_dev": None,
                "first_div_step": 0, "first_tol_exceed_step": 0,
                "compared_steps": 0, "note": "header mismatch"}
    common = sorted(set(da) & set(db))

    # GUARD -- a recording in which NOTHING MOVES cannot demonstrate anything.
    # This is not hypothetical: a 10-robot GPU world once graded `bitwise` on
    # two identical tables of a STATIC sun marker, because the recorder skipped
    # every undeffed robot. The engine's own trace from those runs showed the
    # robots diverging by 0.33 m. A determinism check that passes on a
    # stationary scene is not a check.
    moved = False
    if len(common) >= 2:
        first, last = da[common[0]], da[common[-1]]
        for x, y in zip(first, last):
            try:
                if abs(float(x) - float(y)) > 1e-9:
                    moved = True
                    break
            except (TypeError, ValueError):
                continue
    if not moved:
        return {"grade": "no_motion", "max_abs_dev": None,
                "first_div_step": None, "first_tol_exceed_step": None,
                "compared_steps": len(common),
                "note": "nothing in the recording moved between the first and "
                        "last compared step -- this run proves NOTHING about "
                        "determinism. Check that the recorder actually tracked "
                        "the bodies of interest (see _targets)."}

    bitwise = True
    max_dev = 0.0
    first_div = None
    first_tol = None
    for s in common:
        ta, tb = da[s], db[s]
        if ta != tb:
            bitwise = False
            if first_div is None:
                first_div = s
            for x, y in zip(ta, tb):
                if x == y:
                    continue
                d = abs(float(x) - float(y))
                if d > max_dev:
                    max_dev = d
                if d >= TOL and first_tol is None:
                    first_tol = s
    if bitwise:
        grade = "bitwise"
    elif max_dev < TOL:
        grade = "tolerance"
    else:
        grade = "divergent"
    return {"grade": grade, "max_abs_dev": max_dev,
            "first_div_step": -1 if first_div is None else first_div,
            "first_tol_exceed_step": -1 if first_tol is None else first_tol,
            "compared_steps": len(common)}


def check_backend(backend, sidecar, deviations):
    """Assert the sidecar verdict matches the requested backend; record honestly."""
    if backend == "newton":
        if sidecar is None:
            deviations.append("newton sidecar MISSING — cannot prove Newton drove the run "
                              "(run reached finalize? MSYS env?); result NOT attributable to newton")
            return False
        if sidecar.get("degraded"):
            deviations.append("newton sidecar degraded=true (solver fell back: %s)"
                              % sidecar.get("solver"))
            return False
        deviations.append("newton verified via sidecar: solver=%s" % sidecar.get("solver"))
        return True
    else:
        # Unreachable: main() refuses any backend other than "newton"
        # (src/ode deleted, bdc02139). Kept as a hard stop, not a fallback.
        deviations.append("backend %r is not available (ODE deleted in bdc02139)" % backend)
        return False


def main():
    ap = argparse.ArgumentParser(description="OmniBench lane 3a determinism grading")
    ap.add_argument("--backend", required=True, choices=["ode", "newton"])
    ap.add_argument("--steps", type=int, default=400)
    ap.add_argument("--out", default=str(_row.default_out("determinism.jsonl")))
    ap.add_argument("--skip-coldwarm", action="store_true")
    ap.add_argument("--skip-coldcold", action="store_true",
                    help="run only the cold-vs-warm comparison")
    ap.add_argument("--keep-tmp", action="store_true")
    ap.add_argument("--world", default=None,
                    help="override the scene; use a CONTACT-HEAVY world to "
                         "probe GPU solver determinism (the default 5-sphere "
                         "scene cannot expose it)")
    ap.add_argument("--dt-ms", type=float, default=None,
                    help="basicTimeStep of --world, in ms (must match the "
                         "world file or the sim-time bookkeeping is wrong)")
    args = ap.parse_args()

    if args.backend == "ode":
        print("[l3-det] --backend ode is RETIRED: ODE was DELETED (src/ode, "
              "commit bdc02139) and Newton is the only physics backend.")
        print("[l3-det] The ODE determinism result (bitwise reproducible "
              "run-to-run) is now HISTORICAL — see "
              "docs/benchmarks/determinism-scope.md, which is the source of "
              "truth for what is still claimable and must be scoped to a "
              "SOLVER, never to 'OmniSim'.")
        print("[l3-det] Re-run with --backend newton.")
        return 2

    bin_path = _row.omnisim_binary()
    if not bin_path.exists():
        print("[l3-det] omnisim-bin not found at %s — build first." % bin_path)
        return 2

    engine = "omnisim-%s" % args.backend
    warn_if_already_running()
    tmp = tempfile.mkdtemp(prefix="omnibench_l3_det_%s_" % args.backend)
    global _WORLD_OVERRIDE, DT_MS
    if args.world:
        _WORLD_OVERRIDE = Path(args.world)
        if not _WORLD_OVERRIDE.is_file():
            raise SystemExit("--world does not exist: %s" % _WORLD_OVERRIDE)
    if args.dt_ms:
        DT_MS = float(args.dt_ms)
    _w = _WORLD_OVERRIDE or WORLD
    print("[l3-det] backend=%s steps=%d world=%s dt_ms=%s"
          % (args.backend, args.steps, _w.name, DT_MS))
    print("[l3-det] tmp: %s" % tmp)

    rc = 0

    # ---------------- cold-vs-cold: two fresh processes ----------------
    if not args.skip_coldcold:
        deviations = ["wall_ms_per_step includes engine startup + world load (fresh process)"]
        d1 = os.path.join(tmp, "cold1"); os.makedirs(d1, exist_ok=True)
        d2 = os.path.join(tmp, "cold2"); os.makedirs(d2, exist_ok=True)
        print("[l3-det] cold run 1/2 ...")
        (res1,), wall1, sc1, err1 = capture(args.backend, d1, args.steps, "single", "cold1", ["run.csv"])
        ok1 = check_backend(args.backend, sc1, deviations)
        print("[l3-det] cold run 2/2 ...")
        (res2,), wall2, sc2, err2 = capture(args.backend, d2, args.steps, "single", "cold2", ["run.csv"])
        ok2 = check_backend(args.backend, sc2, deviations)
        for e in (err1, err2):
            if e:
                deviations.append(e)

        m = diff(res1, res2)
        if m["grade"] == "no_data" or not (ok1 and ok2):
            rc = 1
        wall_per_step = 1000.0 * (wall1 + wall2) / (2 * max(1, args.steps))
        row = _row.make_row(
            test="determinism_cold_cold", engine=engine, dt_ms=DT_MS,
            metrics=m, wall_ms_per_step=round(wall_per_step, 4),
            steps=args.steps, sim_seconds=args.steps * DT_MS / 1000.0,
            deviations=deviations)
        _row.write_row(args.out, row)
        print("[l3-det] COLD-vs-COLD  grade=%s  max|d|=%s  first_div_step=%s  (%d steps compared)"
              % (m["grade"], m["max_abs_dev"], m["first_div_step"], m["compared_steps"]))

    # ---------------- cold-vs-warm: one process, worldReload ----------------
    if not args.skip_coldwarm:
        deviations_w = ["run1=cold load, run2=warm worldReload() in the SAME engine process"]
        dw = os.path.join(tmp, "coldwarm"); os.makedirs(dw, exist_ok=True)
        print("[l3-det] cold-vs-warm run (load, record, reload, record) ...")
        results, wallw, scw, errw = capture(
            args.backend, dw, args.steps, "coldwarm", "coldwarm",
            ["run_w1.csv", "run_w2.csv"])
        okw = check_backend(args.backend, scw, deviations_w)
        if errw:
            deviations_w.append(errw)
        mw = diff(results[0], results[1])
        if mw["grade"] == "no_data" or not okw:
            rc = 1
        row = _row.make_row(
            test="determinism_cold_warm", engine=engine, dt_ms=DT_MS,
            metrics=mw, wall_ms_per_step=round(1000.0 * wallw / (2 * max(1, args.steps)), 4),
            steps=args.steps, sim_seconds=args.steps * DT_MS / 1000.0,
            deviations=deviations_w)
        _row.write_row(args.out, row)
        print("[l3-det] COLD-vs-WARM  grade=%s  max|d|=%s  first_div_step=%s  (%d steps compared)"
              % (mw["grade"], mw["max_abs_dev"], mw["first_div_step"], mw["compared_steps"]))

    print("[l3-det] rows appended to %s" % args.out)
    if not args.keep_tmp:
        print("[l3-det] raw CSVs kept at %s (delete manually if unneeded)" % tmp)
    return rc


if __name__ == "__main__":
    sys.exit(main())
