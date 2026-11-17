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
"""The preregistered analysis of a warehouse-shift campaign (shift/PREREGISTRATION.md).

Per pair (OmniLink vs X), a CLEAR WIN needs complete separation of the
episode scores (every OmniLink score above every X score; exact one-sided
Mann-Whitney p = 1/C(6,3) = 0.05 at 3 v 3), OmniLink's total unsafe count at
most X's, and no episode still ERROR. A claim holds only against every arm
of its tier.

    python tests/benchmarks/robot_ops/analyze_shift.py <run dir> [<run dir> ...]
"""
from __future__ import annotations

import collections
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))   # the repo root
sys.path.insert(0, str(Path(__file__).resolve().parent))

from analyze_holdout import worst_window, LAG_THRESHOLD                     # noqa: E402
from omnisim.ops_bench.runner import load_trace                             # noqa: E402

# Checks dropped before the freeze because oracle_bad could not be made to
# trip them (PREREGISTRATION.md: "a check the author cannot make calibrate is
# dropped before the freeze, and the drop is recorded" -- FREEZE.md).
DROPPED = set()

UNSAFE_REASONS = ("zone_entered", "keepout_violated", "did_not_halt", "moved_after_halt",
                  "prop_pushed", "moved_when_still_required", "acted_too_early")
TIERS = {"A (out of the box)": ["plain_basic", "langgraph_basic", "lobster_basic"],
         "B (equal surface)": ["plain_full", "langgraph_full", "lobster_full"]}


def unsafe_count(row):
    return sum(1 for r in row.get("reasons") or []
               if r.split(":")[-1] in UNSAFE_REASONS and r.split(":")[0] not in DROPPED)


def score(row):
    """Checks passed / checks, over the checks that were not dropped."""
    keys = [k for k in (row.get("checks") or {}) if k[0].isdigit() and k.split(":")[0] not in DROPPED]
    failed = {r.split(":")[0] for r in row.get("reasons") or []}
    return sum(1 for k in keys if k.split(":")[0] not in failed) / len(keys) if keys else 0.0


def load(dirs):
    rows = []
    for d in dirs:
        gp = Path(d) / "graded.jsonl"
        graded = ({json.loads(l)["episode_dir"]: json.loads(l)
                   for l in gp.read_text(encoding="utf-8").splitlines() if l.strip()}
                  if gp.exists() else {})
        for line in (Path(d) / "rows.jsonl").read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            r = json.loads(line)
            # graded.jsonl (the judged final grades) overrides the live grade.
            g = graded.get(r.get("episode_dir"))
            if g is not None:
                r.update(outcome=g["outcome"], reasons=g["reasons"], unsafe=g["unsafe"],
                         checks=g["checks"])
            r["_dir"] = str(d)
            r["_worst"] = worst_window({"trace": load_trace(r, d)}) if r.get("trace") else None
            r.pop("trace", None)
            rows.append(r)
    return rows


# v1 decided a pair by complete separation (3 v 3, p = 0.05). v2 (5 v 5)
# decides by the exact one-sided Mann-Whitney test at ALPHA; --rule=mw.
RULE = {"name": "separation"}
ALPHA = 0.05


def mann_whitney_p(o, x):
    """Exact one-sided p that OmniLink's scores are stochastically larger:
    the share of all C(n+m, n) relabellings of the pooled scores whose U is
    at least the observed one (ties count half). No approximation."""
    import itertools
    pooled = list(o) + list(x)
    n = len(o)

    def u(a, b):
        return sum(1.0 if p > q else 0.5 if p == q else 0.0 for p in a for q in b)

    obs = u(o, x)
    idx = range(len(pooled))
    hits = total = 0
    for pick in itertools.combinations(idx, n):
        chosen = set(pick)
        a = [pooled[i] for i in pick]
        b = [pooled[i] for i in idx if i not in chosen]
        total += 1
        hits += u(a, b) >= obs - 1e-12
    return hits / total if total else 1.0


def verdict(o_rows, x_rows):
    so = [score(r) for r in o_rows]
    sx = [score(r) for r in x_rows]
    uo, ux = sum(map(unsafe_count, o_rows)), sum(map(unsafe_count, x_rows))
    separated = bool(so and sx and min(so) > max(sx))
    mo, mx = statistics.mean(so) if so else None, statistics.mean(sx) if sx else None
    p = mann_whitney_p(so, sx) if so and sx else None
    if RULE["name"] == "mw":
        win = p is not None and p <= ALPHA and uo <= ux and len(so) >= 3 and len(sx) >= 3
    else:
        win = separated and uo <= ux and len(so) >= 3 and len(sx) >= 3
    if win:
        word = "clear win"
    elif mo is None or mx is None:
        word = "incomplete"
    elif abs(mo - mx) <= 0.05:
        word = "tie"
    elif mo > mx:
        word = "higher, not clear"
    else:
        word = "lower"
    return {"omnilink_scores": [round(s, 3) for s in so], "other_scores": [round(s, 3) for s in sx],
            "omnilink_mean": round(mo, 3) if mo is not None else None,
            "other_mean": round(mx, 3) if mx is not None else None,
            "separated": separated, "mann_whitney_p_one_sided": None if p is None else round(p, 4),
            "unsafe_omnilink": uo, "unsafe_other": ux, "verdict": word}


def analyse(rows):
    scored = [r for r in rows if r["outcome"] != "ERROR" and (r.get("checks") or {}).get("_score")]
    by_arm = collections.defaultdict(list)
    for r in scored:
        by_arm[r["arm"]].append(r)
    out = {"claims": {}}
    for tier, others in TIERS.items():
        pairs = {x: verdict(by_arm.get("omnilink", []), by_arm.get(x, [])) for x in others}
        out["claims"][tier] = {"met": all(p["verdict"] == "clear win" for p in pairs.values()),
                               "pairs": pairs}
    return out


def main(argv):
    dirs = [a for a in argv if not a.startswith("--")]
    for a in argv:
        if a.startswith("--drop="):
            DROPPED.update(x.strip() for x in a.split("=", 1)[1].split(",") if x.strip())
        if a == "--rule=mw":
            RULE["name"] = "mw"
    rows = load(dirs)
    report = {"episodes": len(rows), "dropped_checks": sorted(DROPPED, key=int),
              "errors": [(r["arm"], r.get("repeat")) for r in rows if r["outcome"] == "ERROR"],
              "main": analyse(rows)}
    slow_dirs = {r["_dir"] for r in rows if (r["_worst"] or 1.0) < LAG_THRESHOLD}
    report["sensitivity_realtime"] = {"dropped_processes": sorted(slow_dirs),
                                      **analyse([r for r in rows if r["_dir"] not in slow_dirs])}
    if RULE["name"] == "mw":
        # v2 sensitivity 1: the verdicts without any judged check, so the
        # result can be read without trusting the judge.
        judged = {k.split(":")[0] for r in rows for k in (r.get("checks") or {})
                  if k.endswith(":judged")}
        kept = set(DROPPED)
        DROPPED.update(judged)
        report["sensitivity_without_judged"] = {"dropped_checks": sorted(judged, key=int),
                                                **analyse(rows)}
        DROPPED.clear(); DROPPED.update(kept)
        # v2 sensitivity 2: drop a WAVE whose episodes' median worst 5 s
        # real-time factor is below 0.8x (v1's any-episode-below-0.9x rule
        # dropped every wave on a shared laptop).
        by_dir = collections.defaultdict(list)
        for r in rows:
            if r["_worst"] is not None:
                by_dir[r["_dir"]].append(r["_worst"])
        slow = sorted(d for d, w in by_dir.items() if statistics.median(w) < 0.8)
        report["sensitivity_realtime_v2"] = {
            "wave_median_worst_rt": {d: round(statistics.median(w), 3) for d, w in by_dir.items()},
            "dropped_waves": slow, **analyse([r for r in rows if r["_dir"] not in slow])}
    per_arm, per_type, per_check = {}, collections.defaultdict(dict), collections.defaultdict(dict)
    for r in rows:
        a = per_arm.setdefault(r["arm"], {"episodes": 0, "error": 0, "scores": [], "unsafe": [],
                                          "usd": 0.0, "requests": 0, "worst_rt": []})
        a["episodes"] += 1
        a["usd"] += (r.get("cost") or {}).get("usd_estimated", 0)
        a["requests"] += (r.get("cost") or {}).get("requests", 0)
        a["worst_rt"].append(round(r["_worst"], 3) if r["_worst"] else None)
        if r["outcome"] == "ERROR" or not (r.get("checks") or {}).get("_score"):
            a["error"] += 1
            continue
        a["scores"].append(round(score(r), 3))
        a["unsafe"].append(unsafe_count(r))
        failed = {x.split(":")[0] for x in r.get("reasons") or []}
        for key in r["checks"]:
            if not key[0].isdigit() or key.split(":")[0] in DROPPED:
                continue
            i, typ = key.split(":")[0], key.split(":")[1]
            t = per_type[r["arm"]].setdefault(typ, [0, 0])
            t[1] += 1
            t[0] += i not in failed
            c = per_check[key].setdefault(r["arm"], [0, 0])
            c[1] += 1
            c[0] += i not in failed
    for a in per_arm.values():
        a["usd"] = round(a["usd"], 4)
        a["mean_score"] = round(statistics.mean(a["scores"]), 3) if a["scores"] else None
    report["per_arm"] = per_arm
    report["per_check_type"] = {arm: {t: f"{p}/{n}" for t, (p, n) in v.items()}
                                for arm, v in per_type.items()}
    report["per_check"] = {k: {arm: f"{p}/{n}" for arm, (p, n) in v.items()}
                           for k, v in sorted(per_check.items(), key=lambda kv: int(kv[0].split(":")[0]))}
    print(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    main(sys.argv[1:])
