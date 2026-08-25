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

"""exp_ledger.py -- the EXPERIMENT LEDGER for structured RL training.

Turns every training run into a structured, comparable record {config, learning curve,
outcome} so we ACCUMULATE knowledge instead of re-discovering. Over runs the ledger
becomes an empirical scaling/sensitivity table -- "config X reaches surv Y in Z iters" --
so a new config's outcome can be ESTIMATED before launching.

  python exp_ledger.py add <log.txt> [--tag NAME] [--outcome "..."]
  python exp_ledger.py rank [--metric asymptote|best_surv|final_surv]   # ranked, showing what CHANGED
  python exp_ledger.py show <tag>

Records live in _scratch/exp_ledger/<tag>.json. The `rank` view shows only the config
keys that VARY across runs (the ablated variables) next to each run's outcome -- so you
can read off "which change helped".
"""
import argparse
import glob
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from deploy_curve import parse_log, fit_saturating, running_max

LEDGER = os.path.join(os.environ.get("OMNISIM_HOME", "."), "_scratch", "exp_ledger")
_CFG_RE = re.compile(r"\bCONFIG\s+(.*)$")


def _read_config(path):
    with open(path, "r", errors="ignore") as f:
        for line in f:
            m = _CFG_RE.search(line)
            if m:
                d = {}
                for kv in m.group(1).split():
                    if "=" in kv:
                        k, v = kv.split("=", 1); d[k] = v
                return d
    return {}


def add(log, tag=None, outcome=""):
    os.makedirs(LEDGER, exist_ok=True)
    tag = tag or os.path.splitext(os.path.basename(log))[0].replace("_rl", "")
    iters, cols, _ = parse_log(log)
    rec = {"tag": tag, "log": os.path.abspath(log), "config": _read_config(log),
           "n_evals": int(iters.size), "outcome": outcome}
    if iters.size:
        surv = cols["surv"]; env = running_max(surv)
        rec.update(final_it=int(iters[-1]), final_surv=round(float(surv[-1]), 3),
                   best_surv=round(float(env[-1]), 3),
                   final_fall=round(float(cols["fall"][-1]), 3),
                   final_sat=round(float(cols["sat"][-1]), 3),
                   final_fidel=round(float(cols["fidel"][-1]), 3),
                   final_drift=round(float(cols["drift"][-1]), 3))
        dcol = cols["depth"]
        if dcol.size and not all(math.isnan(x) for x in dcol):
            last = float(dcol[-1])
            rec["final_depth"] = round(last, 3) if not math.isnan(last) else None
            rec["best_depth"] = round(max(x for x in dcol if not math.isnan(x)), 3)
        fit = fit_saturating(iters, surv)
        if fit:
            rec.update(asymptote=round(fit[0], 3), tau=round(fit[1], 1))
    with open(os.path.join(LEDGER, tag + ".json"), "w") as f:
        json.dump(rec, f, indent=2)
    print("logged '%s': best_surv=%s asymptote=%s n_evals=%d  %s"
          % (tag, rec.get("best_surv", "-"), rec.get("asymptote", "-"), rec["n_evals"], outcome))
    return rec


def _load_all():
    recs = []
    for p in sorted(glob.glob(os.path.join(LEDGER, "*.json"))):
        try:
            recs.append(json.load(open(p)))
        except Exception:
            pass
    return recs


def rank(metric="asymptote"):
    recs = _load_all()
    if not recs:
        print("(ledger empty -- add runs with: exp_ledger.py add <log>)"); return
    recs.sort(key=lambda r: r.get(metric, -1), reverse=True)
    # find config keys that VARY across runs (the ablated variables)
    allkeys = set().union(*[set(r.get("config", {}).keys()) for r in recs])
    varying = sorted(k for k in allkeys
                     if len({r.get("config", {}).get(k) for r in recs}) > 1)
    print("=" * 92)
    print("  EXPERIMENT LEDGER  (ranked by %s; %d runs)  --  varied knobs: %s"
          % (metric, len(recs), ", ".join(varying) or "(none)"))
    print("=" * 92)
    hdr = "  %-14s %7s %6s %6s %5s %5s %6s  %s" % ("tag", "asympt", "best", "final", "fall", "sat", "depth", "changed")
    print(hdr); print("  " + "-" * 88)
    for r in recs:
        chg = " ".join("%s=%s" % (k, r.get("config", {}).get(k, "-")) for k in varying)
        print("  %-14s %7s %6s %6s %5s %5s %6s  %s"
              % (r["tag"][:14], r.get("asymptote", "-"), r.get("best_surv", "-"),
                 r.get("final_surv", "-"), r.get("final_fall", "-"), r.get("final_sat", "-"),
                 r.get("best_depth", "-"), chg))
    print("=" * 92)


def show(tag):
    p = os.path.join(LEDGER, tag + ".json")
    if not os.path.exists(p):
        print("no such run:", tag); return
    print(json.dumps(json.load(open(p)), indent=2))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("add"); a.add_argument("log"); a.add_argument("--tag"); a.add_argument("--outcome", default="")
    r = sub.add_parser("rank"); r.add_argument("--metric", default="asymptote")
    s = sub.add_parser("show"); s.add_argument("tag")
    args = ap.parse_args()
    if args.cmd == "add":
        add(args.log, args.tag, args.outcome)
    elif args.cmd == "rank":
        rank(args.metric)
    elif args.cmd == "show":
        show(args.tag)


if __name__ == "__main__":
    main()
