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

"""Certify the OmniQuad walk/STOP/walk ghost is DYNAMICALLY FEASIBLE (Shadowing Component 2).

Honest gap this closes: the OmniQuad ghost reference is the analytic foot-space TROT MODEL
(kinematic IK), NOT an MPPI forward-rollout, so it is NOT "feasible by construction" in the
shadowing sense and was never run through ghost_verifier. This script does it properly:

  1. GENERATE a feasible-BY-CONSTRUCTION ghost: forward-roll the trot walk/stop/walk
     POSITION targets (scaled by the velocity command s, exactly as the deploy ghost/policy
     reference) open-loop over the DEPLOY-MATCHED MuJoCo model (omniquad_newton_fixed2.xml,
     kp=500/kv=60). The recorded q[t]/qvel[t] is then, by construction, the result of
     applying realizable position-target controls -> a proper shadowing ghost.
  2. CERTIFY with the shadowing verifier's metrics:
       * PRIMARY gate (open-loop sustainability): the forward roll itself -- did it stay
         upright through walk, the stop, and the transitions? (falling => INFEASIBLE).
       * inverse-dynamics certificate (reused from ghost_verifier): base-DOF residual as a
         fraction of body weight (ZMP/balance feasibility) + actuated torque vs limits.
         (Experimental per ghost_verifier -- contacts at snapshots under-report; reported
         as a diagnostic, the open-loop gate is the decision.)

Run: python projects/policies/research/tools/verify_omniquad_walk_ghost.py [--walk 5 --stand 5 --secs 20]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np
import mujoco

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(_REPO))
from projects.policies.control.gait import omniquad_trot_gait as stg          # noqa: E402
from projects.policies.research.shadowing.ghost_verifier import _inverse_dynamics_cert  # noqa: E402

NJ = 12
LEGS = ("FL", "FR", "RL", "RR")
JOINTS = ("hip_x", "hip_y", "knee")


def _tilt(qpos):
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qpos[4] ** 2 + qpos[5] ** 2)))))


def _classify(m, d):
    """Map controller order FL,FR,RL,RR x (hip_x,hip_y,knee) -> (qposadr, ctrl_pos_index).
    Same classification as gpu_mjwarp_omniquad_walk_trainer (anonymous joint_N in the dump):
    leg by body xy sign; hip_x = the X-axis hinge; knee = strongly-negative lower range."""
    label = {}
    k_hinge = 0
    for j in range(m.njnt):
        if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        ax = m.jnt_axis[j]
        rng = m.jnt_range[j]
        pos = d.xpos[m.jnt_bodyid[j]]
        leg = ("FL" if (pos[0] > 0 and pos[1] > 0) else
               "FR" if pos[0] > 0 else "RL" if pos[1] > 0 else "RR")
        joint = ("hip_x" if abs(ax[0]) > 0.5 else
                 "knee" if rng[0] < -0.9 else "hip_y")
        label[(leg, joint)] = (m.jnt_qposadr[j], 2 * k_hinge)   # ctrl interleaved [pos,vel]
        k_hinge += 1
    qpos_adr = np.zeros(NJ, dtype=int)
    ctrl_pos = np.zeros(NJ, dtype=int)
    for ci, leg in enumerate(LEGS):
        for cj, joint in enumerate(JOINTS):
            qa, cp = label[(leg, joint)]
            qpos_adr[ci * 3 + cj] = qa
            ctrl_pos[ci * 3 + cj] = cp
    return qpos_adr, ctrl_pos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default=str(_REPO / "projects/policies/research/training/mjcf/omniquad_newton_fixed2.xml"))
    ap.add_argument("--walk", type=float, default=5.0)
    ap.add_argument("--stand", type=float, default=5.0)
    ap.add_argument("--secs", type=float, default=20.0)
    ap.add_argument("--dt", type=float, default=0.016)
    ap.add_argument("--substeps", type=int, default=8)
    ap.add_argument("--out", default=str(_REPO / "_scratch" / "omniquad_walk_ghost.npz"))
    a = ap.parse_args()

    DT, SUB = a.dt, a.substeps
    m = mujoco.MjModel.from_xml_path(a.mjcf)
    m.opt.timestep = DT / SUB
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    qpos_adr, ctrl_pos = _classify(m, d)

    gp = stg.GaitParams(vx=0.4, freq=1.4, duty=0.6, step_height=0.06,
                        body_height=0.55, ramp_s=1.0)
    nominal = stg.standing_pose(gp).astype(np.float64)

    # init at the standing pose, feet just above the floor
    mujoco.mj_resetData(m, d)
    d.qpos[0:3] = [0.0, 0.0, gp.body_height + 0.01]
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i in range(NJ):
        d.qpos[qpos_adr[i]] = nominal[i]
    mujoco.mj_forward(m, d)

    full = np.zeros(m.nu)

    def apply(tgt):
        full[:] = 0.0
        for i in range(NJ):
            full[ctrl_pos[i]] = tgt[i]
        d.ctrl[:] = full

    # settle on the standing pose
    apply(nominal)
    for _ in range(int(round(0.5 / (DT / SUB)))):
        mujoco.mj_step(m, d)

    def want_stand(e):
        p = a.walk + a.stand
        return (e % p) >= a.walk if p > 0 else False

    n_steps = int(round(a.secs / DT))
    Q, QV, C = [], [], []
    w = 1.0
    walk_dist = 0.0
    fell_at = None
    tilt_max = 0.0
    bz_min = 9.9
    seg = {"walk_vx": [], "stand_vx": []}
    prev_x = float(d.qpos[0])

    for k in range(n_steps):
        e = k * DT
        w_tgt = 0.0 if want_stand(e) else 1.0
        dw = DT / 1.0
        w += max(-dw, min(dw, w_tgt - w))
        s = w
        walk_dist += s * gp.vx * DT
        t_since = walk_dist / max(gp.vx, 1e-3)
        phase = stg.QS_PHASE + 2.0 * math.pi * gp.freq * t_since
        legs, sw = stg.targets_np(phase, gp, t_since_start=t_since)
        legs, _ = stg.speed_scale(legs, sw, nominal, s)
        apply(legs)
        for _ in range(SUB):
            mujoco.mj_step(m, d)
        Q.append(d.qpos.copy())
        QV.append(d.qvel.copy())
        C.append(np.array([legs[i] for i in range(NJ)]))
        bz = float(d.qpos[2])
        tl = _tilt(d.qpos)
        vx = (float(d.qpos[0]) - prev_x) / DT
        prev_x = float(d.qpos[0])
        (seg["stand_vx"] if want_stand(e) else seg["walk_vx"]).append(vx)
        tilt_max = max(tilt_max, tl)
        bz_min = min(bz_min, bz)
        if bz < 0.30 or tl > 60.0:
            fell_at = e
            break

    g = dict(q=np.array(Q), qvel=np.array(QV), ctrl=np.array(C), dt=DT)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    np.savez(a.out, **g)

    # ---- PRIMARY gate: open-loop sustainability (the forward roll itself) ----
    no_fall = fell_at is None
    walk_vx = float(np.mean(seg["walk_vx"])) if seg["walk_vx"] else float("nan")
    stand_vx = float(np.mean(seg["stand_vx"])) if seg["stand_vx"] else float("nan")
    print("=" * 74)
    print("OMNIQUAD WALK/STOP/WALK GHOST -- DYNAMIC FEASIBILITY CERTIFICATE (shadowing C2)")
    print("=" * 74)
    print(f"feasible-by-construction ghost: forward-rolled {len(Q)} steps "
          f"@ {DT*1000:.0f}ms over {a.mjcf.split(chr(92))[-1]}")
    print(f"[open-loop gate] {'SUSTAINS open-loop (never fell)' if no_fall else f'FELL at t={fell_at:.2f}s'}"
          f"  bz_min={bz_min:.3f}  tilt_max={tilt_max:.1f}deg")
    print(f"  bare-ghost forward speed:  walk={walk_vx:+.3f} m/s   stand={stand_vx:+.3f} m/s "
          f"(the IDEAL the robot shadows)")

    # ---- secondary: inverse-dynamics certificate (reused from ghost_verifier) ----
    try:
        base_ok, tor_ok, idm = _inverse_dynamics_cert(g, a.mjcf, DT / SUB, verbose=True)
    except Exception as ex:
        base_ok = tor_ok = None
        print(f"  [inverse-dyn] skipped ({ex})")

    print("-" * 74)
    verdict = "PASS -- dynamically feasible (sustains open-loop) -> valid shadowing ghost" \
        if no_fall else "FAIL -- collapses open-loop -> the ghost is NOT feasible, fix it"
    print(f"VERDICT: {verdict}")
    print(f"saved ghost -> {a.out}")
    return 0 if no_fall else 1


if __name__ == "__main__":
    sys.exit(main())
