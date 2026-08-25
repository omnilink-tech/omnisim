#!/usr/bin/env python3
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

"""What does ONE engine tick cost, on the same instrument?

!! THE ODE ARM IS RETIRED (2026-08-08) AND THIS BENCH LOST ITS HEADLINE.
src/ode was DELETED (commit bdc02139). Everything below that speaks of "two
curves that cross", the `N/ODE` ratio column, and the crossover search describes
a measurement that can no longer be made: there is one backend, so there is one
curve. The comparative results — Newton/ODE 40.7x at 1 body, 23.9x at 5, 7.6x
at 20, ~1.6–2.2x from 50 to 200, converging but never crossing, and ODE's
entire physics phase (1.68 ms at 200 bodies) costing less than `mj_step` alone
(2.09 ms) — are HISTORICAL and live in
docs/benchmarks/step-cost-2026-08-06.md. Do not re-derive them here and do not
quote them as current.

What this bench still measures, and it is worth keeping for: Newton's own
absolute per-tick cost, split into `physics` / `postPhysics`, with the
two-window differencing that cancels the 1.9–4.9 s `finalizeWorld()` term, plus
its marginal per-body cost as `--sweep` grows the scene.

WHY THIS EXISTS
---------------
Three consecutive optimisation commits (899eb425, 7b3762f7, and the a6aa9e54
revert) were steered by the figure "Newton ~0.74 ms/step against ODE's 0.12
ms/step on the same scene". No script in the tree produced either number --
they were ad-hoc -- which is the exact "unreproducible headline" failure
OmniBench was built to prevent, and it left the optimisation target unfalsifi-
able. This is the rerunnable bench for that quantity.

It also answers the question that decides whether the target is reachable at
all: Newton's per-tick cost is dominated by a FIXED conversion overhead that
does not scale with the scene (mj_step measured 0.034 ms on both a 1-body and
a 3-link world), while ODE's cost scales with bodies and contacts. Two curves
that cross. `--sweep` finds where.

THE MEASUREMENT, AND WHY IT IS SHAPED THIS WAY
----------------------------------------------
`--log-performance=<csv>,<N>` makes the engine log its own phase timers for
exactly the first N basic timesteps and then stop. So the same world, run
twice with two different N, yields two totals over two nested windows:

    per_step = (TOT_long - TOT_short) / (N_long - N_short)
    one_time =  TOT_short - N_short * per_step

Differencing two windows CANCELS every one-off cost exactly, without having to
know what it is or where it lands. That matters here more than it usually
would: Newton's `finalizeWorld()` runs INSIDE the physics bracket on the first
tick and measured ~1.8 s on a 3-solid world, so a single-window average over
1250 steps reports 2.287 ms/step for a tick that actually costs 0.824. Every
short-run Newton figure in this repo is inflated by that term.

Both runs execute the IDENTICAL number of steps (the controller is told
N_long + margin either way) -- only the logging window differs. So the two
processes do the same work and differ in nothing but what they recorded.

WHAT IS AND IS NOT ATTRIBUTED
-----------------------------
`physics` brackets OmSimulationWorld.cpp:243-301: the Newton registration
flush, motor-target push, `newton->step()`, AND `mCluster->step()` -- the ODE
pass, which runs unconditionally even in a fully-Newton world. `postPhysics`
holds the per-Solid pose readback (`getBodyXform`/`getBodyVelocity`), which is
where Newton's per-body FFI lands. Both are reported; `total` is their sum and
is the honest "what one tick costs the engine" number.

The controller column is reported but NOT included: it is the controller IPC
round trip, identical in shape on both backends, and folding it in is what
makes an ODE tick look like 0.12 ms when its physics phase costs 0.003.

    python tests/benchmarks/omnibench/step_cost/run_step_cost.py
    python tests/benchmarks/omnibench/step_cost/run_step_cost.py --sweep 1,5,20,50,100,200
    python tests/benchmarks/omnibench/step_cost/run_step_cost.py --bodies 5 --repeats 5 --json out.json
    python tests/benchmarks/omnibench/step_cost/run_step_cost.py --airborne   # contacts off

Newton rows are proven by the `.newton.json` verdict sidecar -- a row whose
backend could not be proven is dropped, never guessed. (The old "ODE rows by its
absence" rule went with the backend, and it was weak anyway: an absent sidecar
also describes a run that never reached world-finalize.)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
OMNIBENCH = HERE.parent
REPO = OMNIBENCH.parents[2]
WORLDS = HERE / "worlds"

sys.path.insert(0, str(OMNIBENCH))
from common.engine_launch import (  # noqa: E402
    backend_proven, build_env, launch_once, newton_verdict, resolve_binary)

DEFAULT_SWEEP = (1, 5, 20, 50, 100, 200)
#: Short window is long enough to be past warm-up jitter, short enough that a
#: 200-body Newton run is not punitive. The DIFFERENCE is what is measured, so
#: the absolute placement of the short window only affects the reported
#: one_time, never per_step.
N_SHORT = 200
N_LONG = 1200
#: The controller runs this many past N_LONG so the long window always closes
#: on a live simulation rather than at world end.
STEP_MARGIN = 50


# --------------------------------------------------------------------------
# World generation. Deterministic: same args -> byte-identical .wbt.
# --------------------------------------------------------------------------

def _fmt(x):
    s = ("%.6f" % float(x)).rstrip("0").rstrip(".")
    return s if s not in ("", "-") else "0"


def gen_world(path, bodies, dt_ms, steps, airborne=False, solver="mujoco"):
    """N unit boxes over a static floor, laid out so none of them touch.

    Resting (default) is the representative robotics case: every body carries a
    live floor contact, so the contact pipeline scales with N the way it does
    in a real scene. `--airborne` lifts them clear instead, which isolates the
    body-count cost from the contact-solving cost.

    The floor's top face is at z = 0.5, deliberately ABOVE Newton's implicit
    z = 0 ground plane, so both backends rest the boxes on the same authored
    geometry and the implicit plane is never the thing being measured.
    """
    half = 0.05
    spacing = 0.5                      # >> box size: no box-box contacts
    side = int(math.ceil(math.sqrt(bodies)))
    z = 0.5 + half if not airborne else 6.0

    solids = []
    for i in range(bodies):
        gx = (i % side - (side - 1) / 2.0) * spacing
        gy = (i // side - (side - 1) / 2.0) * spacing
        solids.append("""DEF BOX%d Solid {
  translation %s %s %s
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.4 0.2 roughness 1 metalness 0 }
      geometry Box { size %s %s %s }
    }
  ]
  name "box%d"
  boundingObject Box { size %s %s %s }
  physics Physics { density -1 mass 1 }
}
""" % (i, _fmt(gx), _fmt(gy), _fmt(z),
       _fmt(half * 2), _fmt(half * 2), _fmt(half * 2), i,
       _fmt(half * 2), _fmt(half * 2), _fmt(half * 2)))

    text = """#VRML_SIM R2025a utf8

# GENERATED by tests/benchmarks/omnibench/step_cost/run_step_cost.py -- do not hand-edit.
# %d dynamic boxes %s a static floor. See the module docstring for the contract.

WorldInfo {
  gravity 9.81
  basicTimeStep %s
  FPS 30
  physicsDisableTime 0
  randomSeed 42
  coordinateSystem "ENU"
%s  newtonStatics TRUE
}
Viewpoint {
  position -8 -8 5
  orientation -0.15 0.05 0.99 1.2
}
Background {
  skyColor [ 0.15 0.17 0.22 ]
}
DEF FLOOR Solid {
  translation 0 0 0
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.3 0.35 roughness 1 metalness 0 }
      geometry Box { size 40 40 1 }
    }
  ]
  name "floor"
  boundingObject Box { size 40 40 1 }
}
DEF STEP_RUNNER Robot {
  name "step_cost_runner"
  controller "step_cost_runner"
  controllerArgs [ "--steps" "%d" ]
  supervisor TRUE
}
%s""" % (bodies, "resting on" if not airborne else "falling clear of",
         _fmt(float(dt_ms)),
         # solver="" omits the field entirely -> the SHIPPED default, which is
         # SolverMuJoCo (CPU mj_step) since the 2026-08-07 flip; XPBD was removed
         # outright in 94f04222, so there is no longer a second arm to contrast.
         ('  newtonSolver "%s"\n' % solver) if solver else "",
         steps, "".join(solids))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


# --------------------------------------------------------------------------
# Perf-CSV parsing
# --------------------------------------------------------------------------

def parse_perf_csv(path):
    """-> {"steps": int, "<col>": float, ...} from the TOT row (totals in ms).

    Column indices come from the engine's own header line, not a hard-coded
    order, so a new phase timer upstream cannot silently shift the reading.
    """
    try:
        lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None
    header = tot = None
    for ln in lines:
        if ln.startswith("<mode>"):
            header = ln.split()
        elif ln.startswith("TOT") and header is not None:
            tot = ln.split()
            break
    if not header or not tot or len(tot) < len(header):
        return None
    out = {"steps": int(tot[1])}
    for i, raw in enumerate(header):
        if i < 2 or i >= len(tot):
            continue
        name = raw.strip("<>").split("(")[0]
        try:
            out[name] = float(tot[i])
        except ValueError:
            out[name] = float("nan")
    return out


# --------------------------------------------------------------------------
# One (bodies, backend) cell
# --------------------------------------------------------------------------

def run_window(binpath, world, backend, n_log, tag, tmp, timeout_s):
    csv = tmp / ("perf_%s.csv" % tag)
    log = tmp / ("engine_%s.log" % tag)
    console = tmp / ("console_%s.log" % tag)
    for p in (csv, log, console):
        if p.exists():
            p.unlink()
    env = build_env(backend, log, repo=REPO)
    rc, wall, timed_out = launch_once(
        binpath, world, env, console, timeout_s,
        extra_args=("--stdout", "--stderr", "--log-performance=%s,%d" % (csv, n_log)))
    row = parse_perf_csv(csv)
    verdict = newton_verdict(log)
    return {"rc": rc, "wall_s": wall, "timed_out": timed_out, "row": row,
            "verdict": verdict, "proven": backend_proven(backend, verdict),
            "csv": str(csv)}


def measure_cell(binpath, bodies, backend, args, tmp, note):
    """Two nested windows -> (per_step, one_time) for each phase."""
    steps = N_LONG + STEP_MARGIN
    solver = getattr(args, "solver", "mujoco")
    world = gen_world(WORLDS / ("sc_%s_b%d%s%s.wbt"
                                % (backend, bodies, "_air" if args.airborne else "",
                                   "_" + (solver or "default"))),
                      bodies, args.dt,
                      steps=steps, airborne=args.airborne, solver=solver)
    samples = []
    for rep in range(args.repeats):
        pair = {}
        ok = True
        for label, n_log in (("short", N_SHORT), ("long", N_LONG)):
            tag = "%s_b%d_%s_r%d" % (backend, bodies, label, rep)
            res = run_window(binpath, world, backend, n_log, tag, tmp, args.timeout)
            row = res["row"]
            if row is None or not res["proven"] or row["steps"] != n_log:
                note("    [drop] %s rep%d %s: rc=%s proven=%s steps=%s%s"
                     % (backend, rep, label, res["rc"], res["proven"],
                        row["steps"] if row else None,
                        " TIMEOUT" if res["timed_out"] else ""))
                ok = False
                break
            pair[label] = row
        if not ok:
            continue
        s, l = pair["short"], pair["long"]
        dn = l["steps"] - s["steps"]
        cell = {}
        for phase in ("physics", "postPhysics", "prePhysics"):
            per = (l[phase] - s[phase]) / dn
            cell[phase] = per
            cell[phase + "_one_time"] = s[phase] - s["steps"] * per
        cell["total"] = cell["physics"] + cell["postPhysics"] + cell["prePhysics"]
        cell["load_ms"] = l.get("loading", float("nan"))
        samples.append(cell)
        note("    %s rep%d: physics %.4f  post %.4f  total %.4f ms/step "
             "(one-time physics %.1f ms)"
             % (backend, rep, cell["physics"], cell["postPhysics"],
                cell["total"], cell["physics_one_time"]))
    if not samples:
        return None
    med = {}
    for k in samples[0]:
        med[k] = statistics.median(s[k] for s in samples)
    med["n"] = len(samples)
    if len(samples) > 1:
        med["total_spread_pct"] = 100.0 * (max(s["total"] for s in samples)
                                           - min(s["total"] for s in samples)) \
            / max(1e-12, med["total"])
    return med


# --------------------------------------------------------------------------

def main():
    # `description=__doc__` means the module docstring is what `--help` WRITES to
    # stdout, so the docstring must stay inside the narrowest console encoding
    # this repo runs on: Windows cp1252. A single decorative U+26A0 U+FE0F in the
    # banner made `--help` itself die with UnicodeEncodeError on this machine --
    # the one command a stranger runs first. Keep this docstring ASCII.
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep", default=None,
                    help="comma-separated body counts (default: %s)"
                         % ",".join(str(b) for b in DEFAULT_SWEEP))
    ap.add_argument("--bodies", type=int, default=None,
                    help="single body count (shorthand for --sweep N)")
    # RETIRED 2026-08-08: the default was "ode,newton". src/ode is DELETED
    # (bdc02139), so the ODE arm -- and with it the N/ODE ratio and the
    # crossover search this bench exists to report -- cannot be re-measured.
    ap.add_argument("--backends", default="newton",
                    help="'newton' is the only value; 'ode' is refused "
                         "(src/ode deleted, bdc02139).")
    ap.add_argument("--dt", type=float, default=4.0, help="basicTimeStep ms")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=600.0)
    ap.add_argument("--airborne", action="store_true",
                    help="lift the boxes clear of the floor (contacts off)")
    ap.add_argument("--solver", default="mujoco",
                    help="WorldInfo.newtonSolver: 'mujoco' (CPU mj_step, the "
                         "default here), 'mujoco_warp', or '' to OMIT the field "
                         "and get the SHIPPED default -- which since the "
                         "2026-08-07 flip (7b431e81) is SolverMuJoCo (CPU "
                         "mj_step), i.e. the same arm as 'mujoco'. 'xpbd' is "
                         "GONE: it was removed in 94f04222.")
    ap.add_argument("--json", default=None, help="write results here")
    ap.add_argument("--keep-worlds", action="store_true")
    args = ap.parse_args()

    if args.bodies is not None:
        sweep = [args.bodies]
    elif args.sweep:
        sweep = [int(x) for x in args.sweep.split(",") if x.strip()]
    else:
        sweep = list(DEFAULT_SWEEP)
    backends = [b.strip() for b in args.backends.split(",") if b.strip()]
    dead = [b for b in backends if b != "newton"]
    if dead:
        print("--backends %s is RETIRED: ODE was DELETED (src/ode, commit "
              "bdc02139) and Newton is the only physics backend."
              % ",".join(dead))
        print("This bench's headline was the RATIO (Newton/ODE 40.7x at 1 body "
              "down to ~1.6-2.2x at 50-200, never crossing). That comparison is "
              "now HISTORICAL -- see docs/benchmarks/step-cost-2026-08-06.md. "
              "What is still measurable here is Newton's own per-step cost and "
              "its marginal per-body cost.")
        print("Re-run with --backends newton.")
        return 2

    binpath = resolve_binary(REPO)
    if binpath is None:
        print("SKIP: no omnisim-bin found; build first.")
        return 0

    tmp = REPO / ".build_tmp" / "step_cost"
    tmp.mkdir(parents=True, exist_ok=True)

    def note(msg):
        print(msg, flush=True)

    print("omnibench/step_cost -- engine per-tick cost, %s"
          % ("contacts OFF (airborne)" if args.airborne else "contacts ON (resting)"))
    print("  binary   : %s" % binpath)
    print("  windows  : %d and %d steps, differenced; %d repeats; dt %g ms"
          % (N_SHORT, N_LONG, args.repeats, args.dt))
    # AGENTS.md: attribute every result to a MACHINE -- "local" is unresolvable
    # after the fact. env_fingerprint's id folds in the GPU model, CPU and a
    # HASHED hostname on purpose: a raw hostname is a personal identifier and
    # these rows get committed (one reached the 2026-07-24 results and had to be
    # scrubbed). Never fall back to COMPUTERNAME/nodename here.
    try:
        sys.path.insert(0, str(REPO))
        from projects.policies.common.env_fingerprint import (  # noqa: E402
            _machine_identity, _nvidia_gpu)
        ident = _machine_identity(_nvidia_gpu())
        mid = "%s (%s, %s)" % (ident["id"], ident["gpu"], ident["host"])
    except Exception as exc:
        ident = {"id": "unknown", "error": repr(exc)}
        mid = "UNKNOWN -- env_fingerprint unavailable (%r)" % (exc,)
    print("  machine  : %s" % mid)
    print()

    results = {"machine": ident, "dt_ms": args.dt, "airborne": args.airborne,
               "n_short": N_SHORT, "n_long": N_LONG, "repeats": args.repeats,
               "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
               "cells": []}

    for bodies in sweep:
        print("  bodies = %d" % bodies)
        row = {"bodies": bodies}
        for backend in backends:
            cell = measure_cell(binpath, bodies, backend, args, tmp, note)
            row[backend] = cell
            if cell is None:
                note("    %s: NO VALID SAMPLE" % backend)
        results["cells"].append(row)
        print()

    # ---- report -----------------------------------------------------------
    print("=" * 78)
    print("PER-TICK ENGINE COST (median of %d, ms/step)" % args.repeats)
    print("=" * 78)
    # SINGLE-ARM TABLE. The old layout had an ODE column and an N/ODE ratio,
    # plus a crossover search over the two curves. Both went with src/ode
    # (bdc02139): with one backend there is one curve and no ratio to report.
    # The historical comparison is in docs/benchmarks/step-cost-2026-08-06.md.
    hdr = "%8s | %-28s | %14s" % ("bodies", "NEWTON (phys+post=total)",
                                  "ms per body")
    print(hdr)
    print("-" * len(hdr))
    prev_row = None
    for row in results["cells"]:
        n = row.get("newton")
        if n is None:
            print("%8d | %-28s | %14s" % (row["bodies"], "  -- NO VALID SAMPLE", "--"))
            prev_row = None
            continue
        cell = "%7.4f+%6.4f=%7.4f" % (n["physics"], n["postPhysics"], n["total"])
        # Marginal cost between adjacent sweep points -- the quantity that
        # actually decides whether Newton scales, and it needs no second arm.
        marginal = "--"
        if prev_row is not None:
            db = row["bodies"] - prev_row["bodies"]
            if db > 0:
                marginal = "%14.4f" % ((n["total"] - prev_row["cell"]["total"]) / db)
        print("%8d | %s | %s" % (row["bodies"], cell,
                                 marginal if marginal == "--" else marginal.strip().rjust(14)))
        prev_row = {"bodies": row["bodies"], "cell": n}
    print()
    for row in results["cells"]:
        n = row.get("newton")
        if n and not math.isnan(n.get("physics_one_time", float("nan"))):
            print("  bodies=%-4d Newton one-time world finalize (inside the physics "
                  "bracket): %.0f ms" % (row["bodies"], n["physics_one_time"]))

    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=2), encoding="utf-8")
        print("\nwrote %s" % args.json)
    if not args.keep_worlds and WORLDS.exists():
        shutil.rmtree(WORLDS, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
