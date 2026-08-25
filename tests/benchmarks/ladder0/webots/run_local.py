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

"""run_local.py -- drive THIS arm alone and print its table.

``ladder0/run_ladder.py`` is the real entry point and runs every arm; this is
the single-arm driver used while bringing the Webots arm up, and for re-running
one rung without touching the other two lanes' engines.  It adds no judgement
of its own: it calls ``arm.run``, reduces with the shared ``analysis.py`` and
scores with the shared ``rungs.check_rung``, exactly as ``run_ladder.py`` does.

    python tests/benchmarks/ladder0/webots/run_local.py            # all rungs
    python tests/benchmarks/ladder0/webots/run_local.py 3 4        # a subset
    python tests/benchmarks/ladder0/webots/run_local.py 4 --fault slide
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time

HERE = os.path.abspath(os.path.dirname(__file__))
LADDER0 = os.path.dirname(HERE)
if LADDER0 not in sys.path:
    sys.path.insert(0, LADDER0)

import analysis                                     # noqa: E402
import rungs                                        # noqa: E402


def _load_sibling(name, filename):
    """This arm's modules, by path, under an arm-qualified name.

    Deliberately not ``import arm``: every arm has one, ``sys.modules`` is
    global, and a bare import here would resolve to whichever arm the process
    loaded first.  CONTRACT.md section 2 has the argument.
    """
    sp = importlib.util.spec_from_file_location(name,
                                                os.path.join(HERE, filename))
    mod = importlib.util.module_from_spec(sp)
    sys.modules[name] = mod
    sp.loader.exec_module(mod)
    return mod


arm = _load_sibling("ladder0_webots_arm", "arm.py")

RESULTS = os.path.join(HERE, "results")


def fmt(v, nd=6):
    if v is None:
        return "None"
    if isinstance(v, float):
        s = ("%.*f" % (nd, v)).rstrip("0").rstrip(".")
        return s if s not in ("", "-") else "0"
    return str(v)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("rungs", nargs="*", type=int)
    ap.add_argument("--fault", default="none")
    ap.add_argument("--timeout", type=float, default=arm.DEFAULT_TIMEOUT_S)
    ap.add_argument("--tag", default="")
    args = ap.parse_args(argv)
    todo = args.rungs or list(rungs.RUNGS)

    ok, why = arm.available()
    if not ok:
        print("arm unavailable: %s" % why)
        return 2

    rows = []
    for rung in todo:
        print("\n=== rung %d -- %s%s"
              % (rung, rungs.RUNG_TITLE[rung],
                 "" if args.fault == "none" else "   [fault: %s]" % args.fault))
        sys.stdout.flush()

        out_dir = os.path.join(RESULTS, "rung%d%s" % (
            rung, "" if args.fault == "none" else "_" + args.fault))
        samples, meta = arm.run(rung, out_dir, fault=args.fault,
                                timeout_s=args.timeout)
        m = analysis.reduce_samples(samples, exit_code=meta.get("exit_code"))
        checks = [c.as_dict() for c in rungs.check_rung(rung, m)]
        wall = analysis.wall_timings(samples, meta.get("proc_t0"),
                                     meta.get("proc_t1"))

        print("    spawn->first step %ss   total %ss   stepping %ss   rc=%s%s"
              % (fmt(wall["startup_s"], 2), fmt(wall["total_s"], 2),
                 fmt(wall["step_s"], 2), meta.get("exit_code"),
                 "   TIMED OUT" if meta.get("timed_out") else ""))
        if meta.get("error"):
            print("    ERROR: %s" % meta["error"])
        for c in meta.get("engine_complaints") or []:
            print("    engine: %s" % c)
        for note in samples.get("notes") or []:
            print("    note: %s" % note)
        for c in checks:
            print("    [%s] %-22s measured %-15s expected %-15s tol %-9s "
                  "margin %-12s %s"
                  % ("PASS" if c["ok"] else "FAIL", c["name"],
                     fmt(c["measured"]), fmt(c["expected"]), fmt(c["tol"]),
                     fmt(c["margin"]), c["unit"]))

        rows.append({"rung": rung, "sim": "webots", "fault": args.fault,
                     "title": rungs.RUNG_TITLE[rung], "measured": m,
                     "checks": checks, "wall": wall,
                     "passed": all(c["ok"] for c in checks),
                     "meta": {k: v for k, v in meta.items()
                              if k not in ("launcher_output",)}})

    os.makedirs(RESULTS, exist_ok=True)
    suffix = args.tag or (args.fault if args.fault != "none" else "")
    out = os.path.join(RESULTS, "ladder0_webots%s.json"
                       % (("_" + suffix) if suffix else ""))
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"arm": "webots", "engine": "upstream Webots R2025a",
                   "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                                  time.gmtime()),
                   "fault": args.fault, "rows": rows}, f, indent=2)

    good = sum(1 for r in rows if r["passed"])
    print("\n%d/%d green   ->  %s" % (good, len(rows), out))
    return 0 if good == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
