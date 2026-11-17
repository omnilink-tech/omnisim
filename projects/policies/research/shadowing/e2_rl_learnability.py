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

"""E2 (RL learnability): does the feasibility CERTIFICATE SCORE predict RL learnability?

The strongest form of the E2 claim, with a real RL policy in the loop. We sweep a quadruped
trot's commanded forward speed vx (the feasibility knob), and at each vx:

  1. CERTIFY the kinematic trot ghost (RL-INDEPENDENT) -> score + verdict (seconds, CPU).
  2. TRAIN a residual RL policy to track that reference (fixed config: 200 iters, 4096 envs,
     ~50 s GPU) and EVAL it -> learnability metrics: first-fall time, forward distance before
     the first fall, falls/env, and the achieved/target speed ratio (tracking fidelity).

If the certificate is a learnability predictor, its score should rank-correlate with the
trained policy's learnability across the sweep. This is the causal E1/E2 spine with a number:
SAME learner + SAME compute at every vx, only the reference's feasibility differs.

  python projects/policies/research/shadowing/e2_rl_learnability.py            # full sweep (~12 min GPU)
  python projects/policies/research/shadowing/e2_rl_learnability.py --quick    # fewer vx
"""
import argparse
import json
import os
import re
import subprocess
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.feasibility_certificate import certify
from projects.policies.research.shadowing.e2_graded_feasibility import build_trot_ghost, MJCF as GO2_CERT_MJCF

PY = sys.executable
TRAINER = os.path.join(REPO, "projects/policies/research/training/gpu_mjwarp_go2_walk_trainer.py")
TR_MJCF = os.path.join(REPO, "projects/robots/unitree/go2/urdf/go2_planner.mjcf.xml")
EXP = os.path.join(REPO, "_scratch/exp")
os.makedirs(EXP, exist_ok=True)


def _run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True).stdout


def _parse_eval(out):
    d = {}
    m = re.search(r"FIRST-FALL: mean=([\-\d.]+)s\s+median=([\-\d.]+)s\s+never_fell=([\d.]+)%\s+falls/env=([\d.]+)", out)
    if m:
        d.update(first_fall=float(m.group(1)), never_fell=float(m.group(3)), falls_env=float(m.group(4)))
    m = re.search(r"fwd dist before first fall: mean=([\-\d.]+) m", out)
    if m:
        d["dist"] = float(m.group(1))
    m = re.search(r"fwd speed while alive: ([\-\d.]+) m/s \(target ([\d.]+)\)", out)
    if m:
        d.update(speed=float(m.group(1)), target=float(m.group(2)))
    return d


def train_and_eval(vx, iters, envs):
    save = os.path.join(EXP, f"go2_lc_vx{vx}.pt")
    _run([PY, TRAINER, "--mjcf", TR_MJCF, "--envs", str(envs), "--iters", str(iters),
          "--vx-target", str(vx), "--save", save])
    ev = _parse_eval(_run([PY, TRAINER, "--mjcf", TR_MJCF, "--envs", str(envs),
                           "--vx-target", str(vx), "--eval", "--save", save]))
    return ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--envs", type=int, default=4096)
    a = ap.parse_args()
    vxs = [0.4, 0.9, 1.6, 2.5] if a.quick else [0.3, 0.5, 0.7, 0.9, 1.2, 1.6, 2.0, 2.5]

    rows = []
    print(f"{'vx':>4} {'cert':>5} {'score':>5} {'base%':>5} {'tau%':>5} | "
          f"{'1stfall':>7} {'dist':>6} {'falls':>5} {'speed':>6} {'track%':>6}")
    print("-" * 76)
    for vx in vxs:
        ghost, _ = build_trot_ghost(vx)
        passed, score, mets = certify(ghost, GO2_CERT_MJCF, motion="walk", verbose=False)
        ev = train_and_eval(vx, a.iters, a.envs)
        track = (ev.get("speed", 0.0) / vx) if vx else 0.0
        row = dict(vx=vx, cert_pass=bool(passed), score=score,
                   base=mets["base_frac_stance"], tau=mets["tau95"],
                   first_fall=ev.get("first_fall"), dist=ev.get("dist"),
                   falls_env=ev.get("falls_env"), speed=ev.get("speed"),
                   speed_track=track)
        rows.append(row)
        print(f"{vx:4.1f} {'PASS' if passed else 'FAIL':>5} {score:5.2f} "
              f"{mets['base_frac_stance']*100:5.1f} {mets['tau95']*100:5.0f} | "
              f"{ev.get('first_fall', -1):7.2f} {ev.get('dist', -1):6.2f} "
              f"{ev.get('falls_env', -1):5.1f} {ev.get('speed', -1):6.3f} {track*100:6.1f}")
    print("-" * 76)

    # rank correlation of the RL-INDEPENDENT certificate score with RL learnability
    sc = np.array([r["score"] for r in rows], float)
    try:
        from scipy.stats import spearmanr
        for key, name in [("dist", "fwd distance before fall"),
                          ("first_fall", "first-fall time"),
                          ("speed_track", "speed-tracking ratio"),
                          ("falls_env", "falls/env (neg)")]:
            y = np.array([(r[key] if r[key] is not None else 0.0) for r in rows], float)
            if key == "falls_env":
                y = -y
            rho, p = spearmanr(sc, y)
            print(f"Spearman(cert score, {name:26s}) = {rho:+.3f}  (p={p:.3f})")
    except Exception as e:
        print(f"[corr] scipy unavailable: {e}")

    out = os.path.join(EXP, "e2_rl_learnability.json")
    json.dump(rows, open(out, "w"), indent=2, default=str)
    print(f"\nsaved {out}")
    print("CLAIM: the RL-independent certificate SCORE predicts RL learnability across the")
    print("feasibility sweep (same learner + same compute at every vx; only the reference differs).")
    return rows


if __name__ == "__main__":
    main()
