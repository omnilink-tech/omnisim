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

"""Harvest a BATON horizon experiment into one dataset the paper's figures read.

Consumes the per-cycle verdicts the deploy now emits:
    BATON-CYCLE k=<i> ok=<0|1> segs=<d>/<t> t=<tick> dur=<s> minz=<m>

Produces _scratch/baton_horizon/results.json:
    {"cycles": N, "seeds": S,
     "arms": {"engineered": {"runs": [[ok_c0, ok_c1, ...], ...],
                             "survival": [k, ...],        # cycles survived per seed
                             "success_rate": [r_c0, ...]}, # fraction of seeds alive AT cycle k
              "naive": {...}}}

SURVIVAL SEMANTICS, deliberately strict: a run "survives cycle k" only if it
completed cycles 0..k with ok=1 and never fell. A fall is terminal -- a G1 on the
floor does not get up, and letting a stalled course time out through its remaining
segments would otherwise manufacture "completed" cycles from a robot lying down
(the first smoke run advanced 8/8 segments at pelvis z = 0.09).

An absent verdict is NOT a pass. If a run produced no BATON-CYCLE line at all, it
scores zero cycles and says so.
"""
import json
import os
import re
import sys

CYCLE_RE = re.compile(
    r"BATON-CYCLE k=(\d+) ok=(\d) segs=(\d+)/(\d+) t=(\d+) dur=([\d.]+) minz=([-\d.]+)")


def cycles_of(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path, errors="replace") as f:
        for ln in f:
            m = CYCLE_RE.search(ln)
            if m:
                out.append(dict(k=int(m.group(1)), ok=int(m.group(2)),
                                dur=float(m.group(6)), minz=float(m.group(7))))
    return out


def main(outdir, ncycles, nseeds):
    data = {"cycles": ncycles, "seeds": nseeds, "arms": {}}
    for arm in ("engineered", "naive"):
        runs, survival = [], []
        for s in range(1, nseeds + 1):
            cs = cycles_of(os.path.join(outdir, f"baton_horizon_{arm}_s{s}_rl.txt"))
            by_k = {c["k"]: c for c in cs}
            row, alive = [], True
            for k in range(ncycles):
                c = by_k.get(k)
                ok = 1 if (alive and c is not None and c["ok"] == 1) else 0
                if not ok:
                    alive = False
                row.append(ok)
            runs.append(row)
            survival.append(sum(row))
        rate = [sum(r[k] for r in runs) / max(1, len(runs)) for k in range(ncycles)]
        data["arms"][arm] = {"runs": runs, "survival": survival, "success_rate": rate}

    p = os.path.join(outdir, "results.json")
    with open(p, "w") as f:
        json.dump(data, f, indent=1)

    print(f"\nwrote {p}\n")
    print("SUCCESS RATE vs HORIZON  (fraction of seeds still alive at cycle k)\n")
    hdr = "  arm           " + "".join(f"  c{k:<4d}" for k in range(ncycles))
    print(hdr)
    for arm, d in data["arms"].items():
        print(f"  {arm:<13s} " + "".join(f"  {r:.2f} " for r in d["success_rate"])
              + f"   mean survival {sum(d['survival']) / max(1, len(d['survival'])):.1f}/{ncycles}")
    print()
    if all(sum(d["survival"]) == 0 for d in data["arms"].values()):
        print("  ⚠️  EVERY RUN SCORED ZERO CYCLES. There is no result here -- do not plot")
        print("      this and do not describe it as a comparison. Fix the runs first.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "_scratch/baton_horizon",
         int(sys.argv[2]) if len(sys.argv) > 2 else 6,
         int(sys.argv[3]) if len(sys.argv) > 3 else 5)
