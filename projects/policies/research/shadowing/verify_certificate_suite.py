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

"""Certificate suite -- the regenerable proof of Shadowing Contribution 3.

The paper's headline empirical claim is that the *ghost-feasibility certificate*
separates dynamically-feasible references (shadowable) from infeasible ones
(unshadowable), INDEPENDENTLY of any RL. This script demonstrates exactly that,
across morphologies, as a self-checking artifact (it exits non-zero on any
mismatch -- so the claim is a test, not prose):

  * GO2 / B2 / G1   feasible flat-ground walk ghost  -> certificate PASS
                    the SAME ghost lifted to levitate -> certificate FAIL

The feasible-vs-levitate pair is a clean controlled A/B: identical joints/gait,
only the base trajectory's dynamic consistency differs -- isolating feasibility
as the discriminated property.

  python projects/policies/research/shadowing/verify_certificate_suite.py
"""
import os
import sys

import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.ghost_verifier_dyn import verify_legged, _build_kinematic_ghost
from projects.policies.research.shadowing.ghost_verifier import verify_arm
from projects.policies.research.shadowing.certify_quadruped_biped import build_b2_walk_ghost, build_g1_walk_ghost

SC = os.path.join(REPO, "_scratch")
os.makedirs(SC, exist_ok=True)
M = lambda p: os.path.join(REPO, p)


def _levitate(in_npz, out_npz, lift=0.6, t_full=2.0):
    """Derive an INFEASIBLE twin: keep the joints/gait identical, ramp the base
    straight up off the ground (feet leave contact, base accelerates upward with
    nothing to push against -> requires a phantom ~mg base force)."""
    g = dict(np.load(in_npz, allow_pickle=True))
    q, qvel, dt = g["q"].copy(), g["qvel"].copy(), float(g["dt"])
    N = q.shape[0]
    z0 = q[:, 2].copy()
    for k in range(N):
        q[k, 2] = z0[k] + lift * min(1.0, (k * dt) / t_full)
    for k in range(N):
        km, kp = max(0, k - 1), min(N - 1, k + 1)
        qvel[k, 2] = (q[kp, 2] - q[km, 2]) / ((kp - km) * dt or 1.0)
    np.savez(out_npz, q=q, qvel=qvel, dt=np.float64(dt))


def _legged_case(name, mjcf, build_fn):
    feas = os.path.join(SC, f"{name}_feasible.npz")
    levi = os.path.join(SC, f"{name}_levitate.npz")
    build_fn(mjcf, feas)
    _levitate(feas, levi)
    pf, sf, _ = verify_legged(np.load(feas), mjcf, verbose=False)
    pl, sl, _ = verify_legged(np.load(levi), mjcf, verbose=False)
    return [(f"{name} walk (feasible)", pf, sf, True),
            (f"{name} walk (levitating)", pl, sl, False)]


def _go2_builder(mjcf, out):
    _build_kinematic_ghost(mjcf, lambda t: 0.322, out)


def main():
    B2 = M("projects/robots/unitree/b2/urdf/b2_planner.mjcf.xml")
    GO2 = M("projects/robots/unitree/go2/urdf/go2_planner.mjcf.xml")
    G1 = M("projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml")

    results = []
    results += _legged_case("go2", GO2, _go2_builder)
    results += _legged_case("b2", B2, build_b2_walk_ghost)
    results += _legged_case("g1", G1, build_g1_walk_ghost)


    print(f"{'case':32s} {'verdict':8s} {'score':>6s}  expected  {'OK?'}")
    print("-" * 70)
    ok = True
    for tag, passed, score, want in results:
        match = bool(passed) == want
        ok = ok and match
        print(f"{tag:32s} {'PASS' if passed else 'FAIL':8s} {score:6.2f}  "
              f"{'feasible' if want else 'INFEAS':8s}  {'ok' if match else 'MISMATCH'}")
    print("-" * 70)
    print(f"[certificate-suite] {'ALL CORRECT -- the certificate separates feasible from infeasible' if ok else 'MISMATCH -- certificate failed to separate'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
