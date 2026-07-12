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

"""Closed-loop train<->deploy divergence diagnostic: BUG vs CHAOS.

THE standard check to run whenever a policy behaves differently in the trainer
than in deploy and you don't know why. It runs the SAME policy on two sides and
reads the *shape* of how their trajectories diverge to tell you which of two
fundamentally different things you are looking at:

  * SYSTEMATIC BUG  -- the two diverge from tick ~1, before chaos can grow. A real
    pipeline/physics/obs difference (wrong frame, wrong gain, wrong constant). The
    per-component obs diff names the culprit. FIXABLE -- go fix it.
  * PHYSICAL CHAOS  -- the two match to the float floor for the first ~0.3 s, then
    diverge EXPONENTIALLY. This is NOT a bug: a free-standing / balancing robot is an
    unstable system (positive Lyapunov exponent), so float32 rounding + tiny obs
    noise get amplified. It can never be zero for a free base. The fix is policy
    ROBUSTNESS (domain randomization / margin), not "close the gap more."

How to read the verdict: a clean pipeline matches in the EARLY window (default 10
ticks) to within --bug-eps; divergence after that, growing roughly exponentially,
is chaos. A nonzero EARLY divergence is the bug signature -- look at the dominant
obs component and the first ticks of the state table.

INPUTS: two per-tick traces of the SAME policy + SAME control law on two sides,
each a JSON `{"ticks":[{"k","phase","q","base_pos","base_rot","obs"?}, ...]}`.
For G1 these come from:
  - trainer : projects/policies/research/training/g1_policy_eval_addurdf.py   (certified SolverMuJoCo)
  - deploy  : projects/policies/controllers/g1_policy_probe/...       (real omnisim-bin)
The control law + obs construction MUST be identical on both sides or you are
measuring that difference, not chaos. (`obs` is optional; without it you still get
the state-divergence BUG/CHAOS verdict, just not the per-component obs breakdown.)

CONTROL: run the SAME policy on a WELDED base too -- with the base pinned the
inverted-pendulum instability is gone, so a clean pipeline matches to ~1e-5 for the
whole episode. If the welded lane matches but the free lane diverges late, that is
positive proof the divergence is chaos, not a model gap.

Usage:
    python projects/policies/research/training/closed_loop_parity_compare.py \
        --trainer _scratch/parity/cl_trainer_free.json \
        --deploy  _scratch/parity/cl_deploy_free.json [--dt 0.016]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

# Obs layout for the G1 32-dim parity obs (only used for the per-component obs
# breakdown; the BUG/CHAOS verdict itself is obs-independent -- it reads state).
SEG = [("q-nom", 0, 13), ("proj_g", 13, 16), ("rate", 16, 19), ("last_act", 19, 32)]


def _load(path):
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    return {(t["phase"], t["k"]): t for t in d["ticks"]}, d.get("meta", {})


def _series(tr, dp, keys):
    """Per-probe-tick divergence series between the two traces."""
    out = {"k": [], "dq": [], "dxy": [], "dz": [], "dobs": [],
           "seg": {n: [] for n, _, _ in SEG}}
    have_obs = "obs" in tr[keys[0]] and "obs" in dp[keys[0]]
    for ph, k in keys:
        a, b = tr[(ph, k)], dp[(ph, k)]
        out["k"].append(k)
        out["dq"].append(float(np.max(np.abs(np.array(a["q"]) - np.array(b["q"])))))
        out["dxy"].append(float(np.hypot(a["base_pos"][0] - b["base_pos"][0],
                                         a["base_pos"][1] - b["base_pos"][1])))
        out["dz"].append(float(abs(a["base_pos"][2] - b["base_pos"][2])))
        if have_obs:
            oa, ob = np.array(a["obs"]), np.array(b["obs"])
            out["dobs"].append(float(np.max(np.abs(oa - ob))))
            for n, lo, hi in SEG:
                out["seg"][n].append(float(np.max(np.abs(oa[lo:hi] - ob[lo:hi]))))
    out["have_obs"] = have_obs
    for kk in ("k", "dq", "dxy", "dz", "dobs"):
        out[kk] = np.array(out[kk], dtype=float)
    return out


def _efold_rate(k, d, lo, hi):
    """Least-squares slope of log(d) vs tick over the growth band [lo,hi] -> the
    Lyapunov-like exponential rate (per tick). Returns (rate, r2) or (None,None)."""
    m = (d >= lo) & (d <= hi) & (d > 0)
    if m.sum() < 4:
        return None, None
    x = k[m]; y = np.log(d[m])
    A = np.vstack([x, np.ones_like(x)]).T
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    rate = float(sol[0])
    yhat = A @ sol
    ss_res = float(np.sum((y - yhat) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2)) or 1.0
    return rate, 1.0 - ss_res / ss_tot


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--trainer", required=True)
    ap.add_argument("--deploy", required=True)
    ap.add_argument("--dt", type=float, default=0.016, help="control timestep (s)")
    ap.add_argument("--early-ticks", type=int, default=10,
                    help="window in which a CLEAN pipeline must match (bug detector)")
    ap.add_argument("--bug-eps", type=float, default=0.02,
                    help="max base-position divergence (m) in the early window for "
                         "'no systematic bug'")
    ap.add_argument("--max-rows", type=int, default=16)
    args = ap.parse_args(argv)

    tr, tmeta = _load(args.trainer)
    dp, dmeta = _load(args.deploy)
    print(f"trainer: {tmeta.get('construction','?')}  ({len(tr)} ticks)")
    print(f"deploy : {dmeta.get('construction','?')}  ({len(dp)} ticks)")
    keys = sorted([k for k in (set(tr) & set(dp)) if k[0] == "probe"], key=lambda kp: kp[1])
    if len(keys) < 5:
        print("need >=5 overlapping probe ticks"); return 1
    s = _series(tr, dp, keys)
    k, dxy, dq = s["k"], s["dxy"], s["dq"]

    # ---- divergence-over-time table (the thing to eyeball) ----
    hdr = "  tick   t(s)   |d_base_xy|  |d_base_z|   max|dq|"
    if s["have_obs"]:
        hdr += "   max|dobs|"
    print("\n" + hdr)
    shown = [i for i in range(len(k)) if k[i] < args.early_ticks + 2 or k[i] % max(1, len(k) // args.max_rows) == 0]
    for i in shown[:args.max_rows + 6]:
        line = f"  {int(k[i]):4d}  {k[i]*args.dt:5.2f}   {dxy[i]:.4e}  {s['dz'][i]:.4e}  {dq[i]:.3e}"
        if s["have_obs"]:
            line += f"   {s['dobs'][i]:.3e}"
        print(line)

    # ---- early-window match (BUG detector) ----
    early = (k >= 1) & (k < args.early_ticks)
    early_xy = float(dxy[early].max()) if early.any() else float("nan")
    early_q = float(dq[early].max()) if early.any() else float("nan")
    pipeline_clean = early_xy <= args.bug_eps

    # ---- growth analysis (CHAOS quantifier) ----
    floor = max(early_xy, 1e-4)
    rate, r2 = _efold_rate(k, dxy, lo=3 * floor, hi=0.5)   # band from the noise floor to 0.5 m
    onset = next((int(k[i]) for i in range(len(k)) if dxy[i] > 10 * floor and dxy[i] > 0.02), None)

    # ---- obs-pipeline caution (catches obs bugs that hide UNDER the chaos) ----
    # An obs bug shows up in the obs at tick ~1 but takes a few ticks to corrupt
    # the base trajectory, so the STATE shape can look chaos-like. A position-like
    # obs channel (q-nom, proj_g) has NO finite-diff amplification -> early
    # divergence there is a real obs bug. A velocity/rate channel is inherently
    # noisy (finite-diff floor ~0.2), but a GROSS early divergence there is the
    # signature of a wrong-frame velocity term (this is exactly how the engine
    # ang-vel bug was found: rate ~2 rad/s at tick 1 while the pose matched).
    caution = None
    if s["have_obs"]:
        seg_early = {n: float(np.max(np.array(s['seg'][n])[early])) for n, _, _ in SEG}
        dom = max(seg_early, key=seg_early.get)
        if seg_early.get("q-nom", 0) > 0.1 or seg_early.get("proj_g", 0) > 0.1:
            c = "q-nom" if seg_early.get("q-nom", 0) >= seg_early.get("proj_g", 0) else "proj_g"
            caution = (f"obs '{c}' diverges {seg_early[c]:.2f} in the early window with NO "
                       f"finite-diff excuse -> obs-pipeline bug in that channel")
        elif seg_early.get("rate", 0) > 1.5:
            caution = (f"obs 'rate' diverges {seg_early['rate']:.2f} early (gross, well past the "
                       f"finite-diff floor) -> suspect a wrong-frame velocity/rate term")

    print("\n" + "=" * 64)
    print(f"early window (ticks 1..{args.early_ticks-1}):  max|d_base_xy|={early_xy:.4f} m   max|dq|={early_q:.3e}")
    if s["have_obs"]:
        print("early obs divergence by component:  " +
              "  ".join(f"{n}={seg_early[n]:.2e}" for n, _, _ in SEG) + f"   (dominant: {dom})")

    final_xy = float(dxy[-1])
    grows = bool((rate and rate > 0 and final_xy > 10 * max(early_xy, 1e-4)) or onset is not None)
    if not pipeline_clean:
        dommsg = f" -- dominant obs term '{dom}'" if s["have_obs"] else ""
        print(f"\nVERDICT:  [BUG] SYSTEMATIC. The two diverge by {early_xy:.3f} m within the first "
              f"{args.early_ticks} ticks,\n          before chaos can grow{dommsg}. This is a real "
              f"pipeline/physics/obs\n          difference -- find and fix it (it is NOT chaos).")
    elif grows and caution:
        print(f"\nVERDICT:  [BUG?] The STATE diverges late (looks chaos-like), BUT an obs channel is\n"
              f"          anomalous early: {caution}.\n"
              f"          An obs-pipeline bug can hide under chaos -- it corrupts the obs at tick ~1\n"
              f"          but takes a few ticks to bend the trajectory. FIX the obs construction so it\n"
              f"          is identical on both sides, then re-run; only THEN trust a CHAOS verdict.")
    elif grows:
        efold = (1.0 / rate) if (rate and rate > 0) else None
        rmsg = (f"e-folding every {efold:.0f} ticks ({efold*args.dt:.2f} s), doubling every "
                f"{efold*0.693:.0f} ticks (fit R2={r2:.2f})") if efold else \
               f"crossing 0.02 m at tick {onset} ({onset*args.dt:.2f} s)"
        print(f"\nVERDICT:  [CHAOS] PIPELINE clean -> the divergence is PHYSICAL CHAOS, not a bug.\n"
              f"          Matched to {early_xy:.4f} m for the first {args.early_ticks} ticks "
              f"({args.early_ticks*args.dt:.2f} s), then diverged\n          EXPONENTIALLY -- {rmsg}, "
              f"reaching {final_xy:.2f} m by the end.\n"
              f"          => not a train/deploy bug; it is intrinsic instability (positive Lyapunov\n"
              f"          exponent). The lever is controller MARGIN (a stand with a wider stability\n"
              f"          basin, or self-correcting motion like a walk), or accept it. NOTE: heavy\n"
              f"          domain randomization does NOT help a same-engine sim->sim chaos gap and\n"
              f"          measurably HURTS (it over-robustifies -> over-reactive -> tips sooner); DR is\n"
              f"          for a real sim->HARDWARE domain gap. CONFIRM chaos with a WELDED-base run.")
    else:
        print(f"\nVERDICT:  [MATCH] PIPELINE clean AND stays matched -- max|d_base_xy|={final_xy:.4f} m "
              f"through the\n          whole run. No bug AND no meaningful chaos (a stable / welded "
              f"lane). This is\n          the gold-standard parity result: trainer and deploy are the "
              f"same system.")
    print("=" * 64)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
