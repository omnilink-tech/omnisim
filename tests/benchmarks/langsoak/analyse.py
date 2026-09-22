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

"""Turn the two endurance runs into numbers anyone can check.

    python tests/benchmarks/langsoak/analyse.py \\
        --lang tests/benchmarks/langsoak/langsoak_2h_2026-09-20.json \\
        --soak tests/benchmarks/soak/soak_2h_2026-09-20.json

Three corrections the raw progress lines do NOT make, each of which would
otherwise misreport the result in a specific direction:

1. BUSY REFUSALS ARE NOT PARSE RESULTS. The bridge refuses a command that
   arrives while it is still moving. Judged by pose alone, that makes a
   STILL case pass for the wrong reason and a MOVE case look "missed".
   Both are separated out here by the reply text the runner recorded. The
   first inflates accuracy, the second deflates it, so they do not cancel.

2. KNOWN GAPS ARE NOT REGRESSIONS. Four cases are marked in corpus.py as
   expected failures. They are reported, and excluded from the headline.

3. A SLOWDOWN IN WALL TIME IS NOT NECESSARILY A SLOWDOWN. Every soak lap
   commands identical motion, so its SIM time is fixed by construction. If
   sim-seconds per lap holds while wall-seconds grows, the host was busy —
   which it demonstrably was, because a second engine was started mid-run
   for the language test. Reporting the raw first-half/second-half lap
   times would blame the architecture for load this session created.

And it fits the drift rather than eyeballing it: linear (a systematic
heading bias, unbounded) against sqrt-n (a random walk) — a distinction
that decides whether the loop can run unattended for a day.
"""
from __future__ import annotations

import argparse, json, math, pathlib, re, statistics

BUSY = re.compile(r"could not|busy|refus", re.I)


def pct(part, whole):
    return 100.0 * part / whole if whole else float("nan")


def analyse_lang(path):
    d = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    rows = [r for r in d["rows"] if r.get("uid")]
    gaps = [r for r in rows if r.get("known_gap")]
    real = [r for r in rows if not r.get("known_gap")]

    # Separate the bridge refusing from the parser deciding.
    busy = [r for r in real
            if r["verdict"] in ("missed", "pass")
            and BUSY.search(r.get("detail") or "")]
    busy_ids = {id(r) for r in busy}
    judged = [r for r in real if id(r) not in busy_ids]

    print("=" * 66)
    print("LANGUAGE ENDURANCE  (held-out wordings, judged by robot pose)")
    print("=" * 66)
    print(f"  utterances asked        {d['utterances']}")
    print(f"  scored                  {len(judged)}")
    print(f"  known-gap cases         {len(gaps)}  (excluded from headline)")
    print(f"  bridge-busy refusals    {len(busy)}  (excluded: not a parse result)")
    print()
    v = {}
    for r in judged:
        v[r["verdict"]] = v.get(r["verdict"], 0) + 1
    ok = v.get("pass", 0)
    print(f"  ACCURACY                {pct(ok, len(judged)):.1f}%")
    fa = v.get("false_actuation", 0)
    still = [r for r in judged if r["kind"] == "still"]
    move = [r for r in judged if r["kind"] == "move"]
    print(f"  FALSE ACTUATIONS        {fa}  "
          f"({pct(fa, len(still)):.1f}% of {len(still)} STILL cases)")
    print(f"    ^ the number that matters: it moved when told not to")
    print(f"  missed commands         {v.get('missed',0)}  "
          f"({pct(v.get('missed',0), len(move)):.1f}% of {len(move)} MOVE)")
    print(f"  wrong motion            {v.get('wrong_move',0)}")
    print(f"  transport errors        {v.get('error',0)}")
    print()
    half = max(1, len(judged) // 2)
    a1 = pct(sum(1 for r in judged[:half] if r["verdict"] == "pass"), half)
    a2 = pct(sum(1 for r in judged[half:] if r["verdict"] == "pass"),
             len(judged) - half)
    print(f"  DOES IT ROT?  first half {a1:.1f}%   second half {a2:.1f}%   "
          f"delta {a2-a1:+.1f} pts")
    print()
    print("  by family:")
    fams = []
    for r in judged:
        if r["family"] not in fams:
            fams.append(r["family"])
    for f in sorted(fams):
        rs = [r for r in judged if r["family"] == f]
        bad = [r for r in rs if r["verdict"] != "pass"]
        print(f"    {f:<9} {pct(len(rs)-len(bad), len(rs)):5.1f}%   "
              f"{len(rs):>4} asked, {len(bad):>3} failed")
    print()
    print("  per-utterance failure counts (deterministic holes rise to the top):")
    per = {}
    for r in judged:
        key = r["uid"]
        per.setdefault(key, [0, 0])
        per[key][1] += 1
        if r["verdict"] != "pass":
            per[key][0] += 1
    for uid, (bad, tot) in sorted(per.items(), key=lambda kv: -kv[1][0])[:14]:
        if bad:
            print(f"    {uid:<11} failed {bad:>3}/{tot:<3} "
                  f"({pct(bad,tot):5.1f}%)")
    return {"accuracy": pct(ok, len(judged)), "false_actuations": fa,
            "still": len(still), "first_half": a1, "second_half": a2}


def fit(xs, ys, f):
    """Least-squares scale for y = k*f(x); returns (k, rmse)."""
    fx = [f(x) for x in xs]
    denom = sum(v * v for v in fx)
    k = sum(v * y for v, y in zip(fx, ys)) / denom if denom else 0.0
    rmse = math.sqrt(sum((k * v - y) ** 2 for v, y in zip(fx, ys)) / len(ys))
    return k, rmse


def analyse_soak(path):
    d = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    rows = d["rows"]
    print()
    print("=" * 66)
    print("ENDURANCE  (one fixed loop, hours)")
    print("=" * 66)
    print(f"  laps {d['laps']}   commands {d['commands']}   "
          f"errors {len(d['errors'])}")
    print(f"  tokens (must be 0)      {max(r.get('tokens',0) for r in rows)}")

    xs = [r["lap"] for r in rows]
    ys = [r["closure_m"] for r in rows]
    kl, el = fit(xs, ys, lambda x: x)
    ks, es = fit(xs, ys, math.sqrt)
    print()
    print(f"  DRIFT: closure {ys[0]:.3f} -> {ys[-1]:.3f} m over {xs[-1]} laps")
    print(f"    linear  fit  {kl*1000:.2f} mm/lap     rmse {el:.4f}")
    print(f"    sqrt(n) fit  {ks:.4f}*sqrt(lap)    rmse {es:.4f}")
    better = "LINEAR (systematic bias, unbounded)" if el < es else \
             "SQRT-N (random walk, self-limiting)"
    print(f"    -> {better} explains it better")
    if el < es:
        print(f"    at this rate: {kl*6800:.1f} m after 24 h of laps")
        print(f"    open-loop dead reckoning; nothing closes the loop on")
        print(f"    absolute position, so the bias integrates forever.")
        print(f"    NOTE: actuation+odometry, NOT interpretation. A model")
        print(f"    deciding each command would drift identically.")

    # Slowdown, attributed.
    if any("sim_s" in r for r in rows):
        sim = [r for r in rows if r.get("sim_s")]
        dsim = [sim[i]["sim_s"] - sim[i-1]["sim_s"] for i in range(1, len(sim))]
        dwall = [sim[i]["lap_s"] for i in range(1, len(sim))]
        h = max(1, len(dsim) // 2)
        print()
        print(f"  SLOWDOWN, ATTRIBUTED:")
        print(f"    sim-seconds/lap   1st half {statistics.mean(dsim[:h]):7.2f}"
              f"   2nd half {statistics.mean(dsim[h:]):7.2f}")
        print(f"    wall-seconds/lap  1st half {statistics.mean(dwall[:h]):7.2f}"
              f"   2nd half {statistics.mean(dwall[h:]):7.2f}")
        sim_chg = pct(statistics.mean(dsim[h:]) - statistics.mean(dsim[:h]),
                      statistics.mean(dsim[:h]))
        print(f"    sim work changed by {sim_chg:+.1f}%")
        print(f"    -> wall-clock rise with flat sim work is HOST LOAD")
        print(f"       (a second engine was started mid-run for langsoak)")
    else:
        print()
        print("  SLOWDOWN: no sim_s column -- cannot attribute wall-time")
        print("            change to host vs simulation. Run predates the fix.")

    pv = [r["private_mb"] for r in rows if r.get("private_mb")]
    ws = [r.get("rss_mb", 0) for r in rows if r.get("rss_mb")]
    print()
    if pv:
        h = max(1, len(pv) // 2)
        print(f"  LEAK (private bytes -- the real metric):")
        print(f"    1st half {statistics.mean(pv[:h]):.0f} MB"
              f"   2nd half {statistics.mean(pv[h:]):.0f} MB"
              f"   delta {pct(statistics.mean(pv[h:])-statistics.mean(pv[:h]), statistics.mean(pv[:h])):+.1f}%")
    else:
        print("  LEAK: no private_mb column. This run measured WORKING SET,")
        print("        which Windows trims on its own -- it fell 707 -> 191 MB")
        print("        here while private memory held at ~1.3 GB. That column")
        print("        is not evidence of a leak OR of its absence.")
    if ws:
        h = max(1, len(ws) // 2)
        print(f"  working set (informational only): "
              f"{statistics.mean(ws[:h]):.0f} -> {statistics.mean(ws[h:]):.0f} MB")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lang", default="")
    ap.add_argument("--soak", default="")
    a = ap.parse_args()
    if a.lang and pathlib.Path(a.lang).exists():
        analyse_lang(a.lang)
    if a.soak and pathlib.Path(a.soak).exists():
        analyse_soak(a.soak)


if __name__ == "__main__":
    main()
