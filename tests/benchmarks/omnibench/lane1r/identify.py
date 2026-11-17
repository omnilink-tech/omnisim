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

"""Identify contact parameters against measured tosses -- and say what that
identification reveals, which is the actual point.

WHY THIS EXISTS
---------------
Three reasons, and only the first is about the score improving.

1. **The published comparison is not like-for-like today.** Acosta et al.
   identified contact parameters PER SIMULATOR before scoring -- their fitted
   friction came out Drake 0.10, MuJoCo 0.22, Bullet 0.36. Lane 1R's headline
   run used the URDF's mu = 0.15 unfitted, so a tuned baseline is being
   compared against our untuned engine.

2. **It answers "tunable or structural?"** If fitting moves us materially,
   the gap was parameters. If it barely moves, the contact MODEL cannot
   represent the physics and no amount of tuning reaches Drake -- whose
   advantage comes from compliant/hydroelastic contact, a formulation
   difference. That fork decides what work is worth doing next, and this is
   the cheapest instrument that resolves it.

3. **The distance between the fitted and the MEASURED parameter is itself a
   fidelity metric.** The cube's friction was physically measured at
   mu = 0.18 by tilt test, and not one engine's best-fit value matched it;
   they disagree with each other by 3.6x. A friction parameter that has to
   travel a long way from the real coefficient is absorbing modelling error
   from elsewhere. So `|mu_fit - 0.18|` is reported as a first-class result,
   not a diagnostic afterthought.

METHOD
------
Staged coordinate search rather than a blind grid, because the parameters are
not equally uncertain and one of them has a physical prior.

`kd` is seeded from the damped-spring model rather than guessed: Newton has
no restitution coefficient, so e is emergent from contact compliance, and
    e = exp(-pi*zeta/sqrt(1-zeta^2)),  zeta = kd / (2*sqrt(ke*m))
inverts at the measured e = 0.125 to zeta = 0.552, i.e. kd ~ 34 for the
default ke = 2500 and m = 0.37. The sweep brackets that.

TRAIN / HELD-OUT IS NOT OPTIONAL. Fitting and reporting on the same tosses
produces a number with no predictive content. The split is a fixed seeded
permutation, the search only ever sees the training indices, and the reported
result is the HELD-OUT score. If fitted parameters do not generalise, the
model lacks the right degrees of freedom -- and that is a more useful finding
than any error percentage.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dataset as D                    # noqa: E402
import run as R                        # noqa: E402
import score as S                      # noqa: E402

#: Physically measured by tilt test; the value a fitted mu should approach if
#: the contact model is honest.
MU_MEASURED = 0.18
DEFAULT_KE = 2500.0


def kd_for_restitution(e, ke=DEFAULT_KE, m=D.CUBE_MASS_KG):
    """Damped-spring inverse: what kd realises a coefficient of restitution e."""
    le = math.log(max(e, 1e-6))
    r = -le / math.pi                       # zeta / sqrt(1 - zeta^2)
    zeta = r / math.sqrt(1.0 + r * r)
    return 2.0 * zeta * math.sqrt(ke * m)


def evaluate(params, indices, out_dir, jobs=4, tag=""):
    """Run `indices` at `params`, score them, return the mean position error.

    Scores immediately and deletes the recordings: a full search otherwise
    leaves thousands of npz behind for a number that is one float.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    suffix = "_" + tag if tag else ""

    fails = []

    def one(i):
        try:
            rec = R.run_one(i, out_dir, scale="none", contact=params,
                            tag_suffix=suffix)
        except Exception as e:                    # a raise here used to vanish
            fails.append("toss %d: run raised %r" % (i, e))
            return None
        if rec["status"] != "ok":
            fails.append("toss %d: %s" % (i, rec.get("error", "?")[:160]))
            return None
        npz = Path(rec["npz"])
        try:
            sc = S.score_one(npz, i, scale="none")
        except Exception as e:
            fails.append("toss %d: score raised %r" % (i, e))
            return None
        finally:
            for p in (npz, Path(str(npz) + ".phase.txt")):
                if p.exists():
                    p.unlink()
        if sc.get("status") != "ok":
            fails.append("toss %d: score %s -- %s"
                         % (i, sc.get("status"), sc.get("note", "")[:120]))
            return None
        return sc

    with ThreadPoolExecutor(max_workers=jobs) as ex:
        rows = [r for r in ex.map(one, indices) if r]

    if not rows:
        # "FAILED (no scored runs)" with no reason is the mute-failure pattern
        # this lane keeps getting bitten by. Say what actually happened.
        print("      no scored runs; first failures:")
        for m in fails[:3]:
            print("        " + m)
        return None
    if fails:
        print("      (%d/%d tosses dropped, e.g. %s)"
              % (len(fails), len(indices), fails[0][:110]))
    pos = np.array([r["pos_pct_mean"] for r in rows])
    rot = np.array([r["rot_deg_mean"] for r in rows])
    return {"params": dict(params), "n": len(rows),
            "pos_pct_mean": float(pos.mean()), "pos_pct_sd": float(pos.std()),
            "rot_deg_mean": float(rot.mean()), "rot_deg_sd": float(rot.std())}


def sweep(name, values, base, indices, out_dir, jobs, log):
    """One coordinate: try `values` for `name`, return the best base."""
    print("\n-- sweeping %s over %s" % (name, values))
    best, best_v = None, None
    for v in values:
        p = dict(base)
        p[name] = v
        t0 = time.time()
        r = evaluate(p, indices, out_dir, jobs=jobs,
                     tag="%s%g" % (name, v))
        if r is None:
            print("   %s=%-8g FAILED (no scored runs)" % (name, v))
            continue
        r["stage"] = name
        log.append(r)
        print("   %s=%-8g pos %.2f%% +/- %.2f   rot %.1f deg   (%.0fs)"
              % (name, v, r["pos_pct_mean"], r["pos_pct_sd"],
                 r["rot_deg_mean"], time.time() - t0))
        if best is None or r["pos_pct_mean"] < best["pos_pct_mean"]:
            best, best_v = r, v
    if best_v is None:
        return base
    out = dict(base)
    out[name] = best_v
    print("   -> best %s = %g (%.2f%%)" % (name, best_v, best["pos_pct_mean"]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Identify contact parameters on measured cube tosses.")
    ap.add_argument("--out", required=True)
    ap.add_argument("--train", type=int, default=40,
                    help="training tosses (the search only ever sees these)")
    ap.add_argument("--holdout", type=int, default=150,
                    help="held-out tosses for the reported result; 0 = all")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--seed", type=int, default=42)
    a = ap.parse_args(argv)

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    scratch = out / "_scratch"

    rng = np.random.default_rng(a.seed)
    perm = rng.permutation(D.N_TRAJ)
    train = sorted(int(i) for i in perm[:a.train])
    rest = [int(i) for i in perm[a.train:]]
    holdout = sorted(rest if a.holdout == 0 else rest[:a.holdout])
    print("train %d tosses, held-out %d (seed %d) -- the search never sees "
          "the held-out set" % (len(train), len(holdout), a.seed))

    kd0 = kd_for_restitution(D.CUBE_RESTITUTION)
    print("damped-spring prior: e=%.3f at ke=%g, m=%.2f -> kd ~ %.1f"
          % (D.CUBE_RESTITUTION, DEFAULT_KE, D.CUBE_MASS_KG, kd0))

    base = {"mu": D.CUBE_MU_STATIC, "ke": DEFAULT_KE, "kd": round(kd0, 1)}
    log = []

    t0 = time.time()
    base = sweep("mu", [0.10, 0.15, 0.22, 0.30, 0.40], base, train,
                 scratch, a.jobs, log)
    base = sweep("kd", [round(kd0 * f, 1) for f in (0.3, 0.6, 1.0, 2.0, 3.5)],
                 base, train, scratch, a.jobs, log)
    base = sweep("ke", [1000.0, 2500.0, 10000.0], base, train,
                 scratch, a.jobs, log)
    print("\nidentified: %s   (search %.1f min)"
          % (base, (time.time() - t0) / 60.0))

    # --- the reported numbers: held-out, authored vs identified ------------
    authored = {"mu": D.CUBE_MU_STATIC, "ke": None, "kd": None}
    print("\nevaluating on the HELD-OUT set (%d tosses)..." % len(holdout))
    ho_auth = evaluate(authored, holdout, scratch, jobs=a.jobs, tag="ho_auth")
    ho_id = evaluate(base, holdout, scratch, jobs=a.jobs, tag="ho_id")
    tr_id = next((r for r in reversed(log)
                  if r["params"] == base), None)

    res = {"train_indices": train, "holdout_indices": holdout, "seed": a.seed,
           "authored": authored, "identified": base,
           "holdout_authored": ho_auth, "holdout_identified": ho_id,
           "train_identified": tr_id, "sweeps": log,
           "mu_measured": MU_MEASURED,
           "mu_fit_minus_measured": (base["mu"] - MU_MEASURED),
           "kd_prior_from_restitution": kd0}
    (out / "identification.json").write_text(json.dumps(res, indent=2),
                                             encoding="utf-8")
    if scratch.exists():
        shutil.rmtree(scratch, ignore_errors=True)

    print("\n" + "=" * 66)
    print("HELD-OUT (%d tosses, never seen by the search)" % len(holdout))
    print("  as authored  mu=%.2f, engine-default ke/kd : %.2f%% +/- %.2f"
          % (D.CUBE_MU_STATIC, ho_auth["pos_pct_mean"], ho_auth["pos_pct_sd"]))
    print("  identified   mu=%.2f ke=%g kd=%.1f          : %.2f%% +/- %.2f"
          % (base["mu"], base["ke"], base["kd"],
             ho_id["pos_pct_mean"], ho_id["pos_pct_sd"]))
    delta = ho_auth["pos_pct_mean"] - ho_id["pos_pct_mean"]
    print("  identification bought %+.2f percentage points" % delta)
    print()
    print("  fitted mu %.2f vs MEASURED %.2f (tilt test): %+.2f"
          % (base["mu"], MU_MEASURED, base["mu"] - MU_MEASURED))
    print("  a fitted mu far from the measured one is absorbing modelling")
    print("  error from elsewhere; Acosta's fits were 0.10 / 0.22 / 0.36")
    print("=" * 66)
    if tr_id:
        gen = ho_id["pos_pct_mean"] - tr_id["pos_pct_mean"]
        print("  generalisation: train %.2f%% -> held-out %.2f%% (%+.2f pp)"
              % (tr_id["pos_pct_mean"], ho_id["pos_pct_mean"], gen))
        print("  a large positive gap here means the fit is absorbing noise, "
              "not physics.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
