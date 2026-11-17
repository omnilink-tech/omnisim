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
"""The preregistered analysis of an ops-bench holdout run.

holdout/PREREGISTRATION.md, decision rule per pair (OmniLink vs X):
  1. the 95% bootstrap interval of the task success-rate difference has a
     lower bound above 0 (tasks resampled with their repeats, 2,000 draws,
     seed 250925);
  2. OmniLink's unsafe episodes <= X's;
  3. neither arm has more than 10% of its episodes as ERROR.
A claim holds only against EVERY arm of its tier.

Only complete task blocks (one task, one repeat, all arms) are analysed. An
ERROR episode drops that block from both sides of its pairs. A sensitivity
result (FREEZE_AMENDMENT_02.md) also drops blocks with any episode whose
worst 5 s real-time window fell below 0.9x.

    python tests/benchmarks/robot_ops/analyze_holdout.py <run dir> [<run dir> ...]
"""
from __future__ import annotations

import collections
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))   # the repo root

SEED, DRAWS = 250925, 2000
LAG_THRESHOLD, LAG_WINDOW_S = 0.9, 5.0
TIERS = {"A (out of the box)": ["plain_basic", "langgraph_basic", "lobster_basic"],
         "B (equal surface)": ["plain_full", "langgraph_full", "lobster_full"]}


def worst_window(row):
    tr = row.get("trace") or {}
    s = [x for x in (tr.get("samples") if isinstance(tr, dict) else tr) or [] if x.get("sim") is not None]
    if len(s) < 10:
        return None
    worst, j = float("inf"), 0
    for i in range(len(s)):
        while j < len(s) and s[j]["t"] - s[i]["t"] < LAG_WINDOW_S:
            j += 1
        if j >= len(s):
            break
        worst = min(worst, (s[j]["sim"] - s[i]["sim"]) / (s[j]["t"] - s[i]["t"]))
    return None if worst == float("inf") else worst


def load(dirs):
    from omnisim.ops_bench.runner import load_trace
    rows = []
    for d in dirs:
        for line in (Path(d) / "rows.jsonl").read_text(encoding="utf-8").splitlines():
            if line.strip():
                r = json.loads(line)
                r["trace"] = load_trace(r, d) if r.get("trace") else None
                r["_worst"] = worst_window(r)
                r.pop("trace", None)
                rows.append(r)
    return rows


def pair(rows, arm_x, blocks, arm_o="omnilink"):
    """Per-task success rates for both arms over the blocks both completed."""
    by = {(r["task"], r["repeat"], r["arm"]): r for r in rows}
    rates = collections.defaultdict(lambda: [[], []])
    for task, rep in blocks:
        o, x = by.get((task, rep, arm_o)), by.get((task, rep, arm_x))
        if not o or not x or "ERROR" in (o["outcome"], x["outcome"]):
            continue
        rates[task][0].append(o["outcome"] == "PASS")
        rates[task][1].append(x["outcome"] == "PASS")
    tasks = sorted(rates)
    ro = np.array([np.mean(rates[t][0]) for t in tasks])
    rx = np.array([np.mean(rates[t][1]) for t in tasks])
    return tasks, ro, rx


def verdict(rows, arm_x, blocks):
    tasks, ro, rx = pair(rows, arm_x, blocks)
    if not tasks:
        return {"arm": arm_x, "tasks": 0}
    diff = ro - rx
    rng = np.random.default_rng(SEED)
    boot = [diff[rng.integers(0, len(diff), len(diff))].mean() for _ in range(DRAWS)]
    lo, hi = np.percentile(boot, [2.5, 97.5])
    point = diff.mean()
    kept = {(t, r) for t, r in blocks}
    ep = lambda arm: [r for r in rows if r["arm"] == arm and (r["task"], r["repeat"]) in kept]
    unsafe_o = sum(bool(r["unsafe"]) for r in ep("omnilink"))
    unsafe_x = sum(bool(r["unsafe"]) for r in ep(arm_x))
    err = lambda arm: sum(r["outcome"] == "ERROR" for r in ep(arm)) / max(1, len(ep(arm)))
    clear = lo > 0 and unsafe_o <= unsafe_x and err("omnilink") <= 0.10 and err(arm_x) <= 0.10
    if clear:
        word = "clear win"
    elif hi < 0:
        word = "loss"
    elif lo <= 0 <= hi and abs(point) <= 0.05:
        word = "tie"
    else:
        word = "higher, not clear" if point > 0 else "lower, not clear"
    return {"arm": arm_x, "tasks": len(tasks), "omnilink_rate": round(float(ro.mean()), 3),
            "other_rate": round(float(rx.mean()), 3), "diff": round(float(point), 3),
            "ci95": [round(float(lo), 3), round(float(hi), 3)],
            "unsafe_omnilink": unsafe_o, "unsafe_other": unsafe_x,
            "error_share_omnilink": round(err("omnilink"), 3), "error_share_other": round(err(arm_x), 3),
            "verdict": word}


def analyse(rows, drop_lagged=False):
    arms = sorted({r["arm"] for r in rows})
    per_block = collections.defaultdict(dict)
    for r in rows:
        per_block[(r["task"], r["repeat"])][r["arm"]] = r
    complete = sorted(b for b, a in per_block.items() if set(a) == set(arms))
    if drop_lagged:
        complete = [b for b in complete
                    if all((r["_worst"] or 1.0) >= LAG_THRESHOLD for r in per_block[b].values())]
    out = {"blocks": len(complete), "claims": {}}
    for tier, others in TIERS.items():
        vs = [verdict(rows, x, complete) for x in others]
        out["claims"][tier] = {"met": all(v.get("verdict") == "clear win" for v in vs), "pairs": vs}
    return out


def main(dirs):
    rows = load(dirs)
    arms = sorted({r["arm"] for r in rows})
    blocks = collections.defaultdict(set)
    for r in rows:
        blocks[(r["task"], r["repeat"])].add(r["arm"])
    incomplete = sorted(b for b, a in blocks.items() if set(a) != set(arms))
    report = {"episodes": len(rows), "incomplete_blocks_dropped": incomplete,
              "main": analyse(rows), "sensitivity_realtime": analyse(rows, drop_lagged=True)}
    fam = collections.defaultdict(lambda: collections.defaultdict(lambda: [0, 0]))
    per_arm = collections.defaultdict(collections.Counter)
    for r in rows:
        if (r["task"], r["repeat"]) in incomplete:
            continue
        fam[r["family"]][r["arm"]][0] += r["outcome"] == "PASS"
        fam[r["family"]][r["arm"]][1] += 1
        a = per_arm[r["arm"]]
        a[r["outcome"]] += 1; a["unsafe"] += bool(r["unsafe"])
        a["usd"] += (r.get("cost") or {}).get("usd_estimated", 0)
        a["requests"] += (r.get("cost") or {}).get("requests", 0)
        a["wall_s"] += r.get("elapsed_s") or 0
        a["lagged"] += (r["_worst"] or 1.0) < LAG_THRESHOLD
    report["per_arm"] = {a: {**{k: (round(v, 4) if isinstance(v, float) else v) for k, v in c.items()},
                             "usd_per_success": round(c["usd"] / c["PASS"], 4) if c["PASS"] else None,
                             "wall_s_per_success": round(c["wall_s"] / c["PASS"], 1) if c["PASS"] else None}
                         for a, c in per_arm.items()}
    report["per_family"] = {f: {a: f"{p}/{n}" for a, (p, n) in v.items()} for f, v in sorted(fam.items())}
    report["per_task"] = {}
    for r in rows:
        report["per_task"].setdefault(r["task"], {}).setdefault(r["arm"], []).append(r["outcome"])
    print(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    main(sys.argv[1:])
