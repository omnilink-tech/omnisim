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

"""Does this world corpus LOAD AND STEP on the Newton default? A per-world tally.

WHY THIS EXISTS (ODE-retirement readiness, campaign item A5)
------------------------------------------------------------
The 289 group-runner worlds under tests/ run on whatever backend `auto`
resolves to, carry ODE-era golden values, and nobody has ever published their
pass rate on Newton. The group runner (tests/test_suite.py) cannot produce that
number: it chains a whole group through ONE engine process, so the first world
that FATALs kills the chain -- measured 2026-08-07, tests/physics died at
world #1 (ball_joint_reset.omniworld, a solver-independent newton-enforce refusal)
and reported nothing about the other 34.

This sweep runs every world in ITS OWN engine process and tallies verdicts, so
one refusal is one row, not a lost campaign. It answers load-and-step only --
a PASS here is "loaded, stepped, logged no ERROR/FATAL, and the backend was
what we asked" -- NOT "the golden values match" (that stays the group runner's
job, on worlds this sweep proves loadable).

Per-world it records: verdict, the first FATAL/ERROR line, the backend verdict
sidecar ({backend, solver, degraded}), and wall seconds. Failures are grouped
by their first FATAL line so the output reads as a defect list, not a wall of
FAILs.

    python scripts/dev/newton_readiness_sweep.py --glob "tests/physics/worlds/*.wbt"
    python scripts/dev/newton_readiness_sweep.py --glob "tests/api/worlds/*.wbt" --jobs 4
    python scripts/dev/newton_readiness_sweep.py --glob "projects/samples/demos/worlds/**/*.wbt" \
        --json _scratch/readiness_demos.json

Parallel-safe: each child gets its own OMNISIM_LOG_PATH (the AGENTS.md rule)
and the engine auto-scans its port. Default --jobs 4 -- this laptop brief.
"""
from __future__ import annotations

import argparse
import glob as globmod
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
BIN = REPO / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
if not BIN.exists():
    BIN = REPO / "bin" / "omnisim-bin"

#: Worlds that need MORE than load-and-step (controllers with special deps,
#: interactive windows) can be listed here with a reason; they are reported
#: as SKIP, never silently dropped.
SKIP: dict[str, str] = {}


def run_world(world: Path, tmp: Path, duration_s: float, extra_env: dict) -> dict:
    tag = "%s_%s" % (world.parent.parent.name, world.stem)
    log = tmp / ("%s.log" % tag)
    if log.exists():
        log.unlink()
    sidecar = Path(str(log) + ".newton.json")
    if sidecar.exists():
        sidecar.unlink()

    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["WEBOTS_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    # A stale knob in the parent shell must not steer a readiness verdict. The two ODE knobs
    # are RETIRED (src/ode deleted, bdc02139) and are the worst offenders: honouring one would
    # give every world a physics-free run that still loads and steps, i.e. a PASS that means
    # nothing. Scrubbing them is load-bearing, not tidiness.
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    env.update(extra_env)

    t0 = time.perf_counter()
    proc = subprocess.Popen(
        [str(BIN), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    timed_out = False
    try:
        proc.wait(timeout=duration_s)
        rc = proc.returncode
    except subprocess.TimeoutExpired:
        # Still running at the deadline = it loaded and stepped without dying.
        timed_out = True
        rc = 0
        if os.name == "nt":
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                           capture_output=True)
        else:
            proc.kill()
        proc.wait()
    wall = time.perf_counter() - t0

    first_bad = None
    warnings = 0
    if log.exists():
        for ln in log.read_text(errors="replace").splitlines():
            if ln.startswith("WARNING:"):
                warnings += 1
            if first_bad is None and (ln.startswith("FATAL:") or ln.startswith("ERROR:")):
                first_bad = ln[:300]
    verdict_side = None
    if sidecar.exists():
        try:
            verdict_side = json.loads(sidecar.read_text())
        except ValueError:
            pass

    ok = (rc == 0 or timed_out) and first_bad is None
    return {
        "world": str(world.relative_to(REPO)),
        "verdict": "PASS" if ok else "FAIL",
        "rc": rc, "ran_full_duration": timed_out, "wall_s": round(wall, 1),
        "first_bad": first_bad, "warnings": warnings,
        "newton": verdict_side,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--glob", required=True, action="append",
                    help="world glob(s), repo-relative (repeatable)")
    ap.add_argument("--duration", type=float, default=12.0,
                    help="seconds each world may run before being counted "
                         "alive and killed (default 12)")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--env", action="append", default=[],
                    help="extra KEY=VALUE for every child (repeatable) -- e.g. "
                         "OMNISIM_NEWTON_SUBSTEPS=4. NOT OMNISIM_FORCE_ODE: that named a "
                         "control arm while ODE shipped, and src/ode was deleted (bdc02139)")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    worlds = []
    for g in args.glob:
        worlds += [Path(p) for p in globmod.glob(str(REPO / g), recursive=True)]
    worlds = sorted(set(w for w in worlds if w.suffix in (".wbt", ".omniworld")))
    if not worlds:
        print("no worlds matched")
        return 2

    extra_env = dict(kv.split("=", 1) for kv in args.env)
    tmp = REPO / ".build_tmp" / "readiness_sweep"
    tmp.mkdir(parents=True, exist_ok=True)

    print("newton readiness sweep: %d worlds, %d jobs, %.0fs budget each"
          % (len(worlds), args.jobs, args.duration))
    if extra_env:
        print("  extra env: %s" % extra_env)
    print()

    results = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futs = {pool.submit(run_world, w, tmp, args.duration, extra_env): w
                for w in worlds}
        for fut in as_completed(futs):
            r = fut.result()
            results.append(r)
            mark = "." if r["verdict"] == "PASS" else "F"
            sys.stdout.write(mark)
            sys.stdout.flush()
    print("\n")
    results.sort(key=lambda r: r["world"])

    passed = [r for r in results if r["verdict"] == "PASS"]
    failed = [r for r in results if r["verdict"] == "FAIL"]
    print("PASS %d / %d  (%.0f%%)" % (len(passed), len(results),
                                      100.0 * len(passed) / len(results)))
    if failed:
        # Group failures by their first FATAL/ERROR line -> a defect list.
        by_reason: dict[str, list] = {}
        for r in failed:
            key = (r["first_bad"] or ("exit rc=%s, no ERROR line" % r["rc"]))[:160]
            by_reason.setdefault(key, []).append(r["world"])
        print("\nFAILURES, grouped by first FATAL/ERROR line:")
        for reason, ws in sorted(by_reason.items(), key=lambda kv: -len(kv[1])):
            print("\n  [%d] %s" % (len(ws), reason))
            for w in ws[:8]:
                print("        %s" % w)
            if len(ws) > 8:
                print("        ... +%d more" % (len(ws) - 8))

    if args.json:
        out = Path(args.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(
            {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
             "duration_s": args.duration, "extra_env": extra_env,
             "results": results}, indent=1))
        print("\nwrote %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
