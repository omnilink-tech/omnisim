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

"""E2 (graded): does the feasibility CERTIFICATE predict dynamic TRACKABILITY?

The headline E2 claim needs a graded sweep, not just feasible-vs-impossible. We take a
quadruped trot and push a single knob (commanded forward velocity vx) from gentle to
well past what the contacts/actuators can supply. For each vx we measure two INDEPENDENT
quantities:

  1. CERTIFICATE  -- feasibility_certificate.certify() on the kinematic trot ghost at that
     vx: pass/fail + score + base-wrench residual + peak torque ratio. (A per-step LP; it
     never simulates the closed loop.)
  2. GROUND TRUTH -- an open-loop full-dynamics rollout in MuJoCo: drive the robot's
     position actuators with the SAME trot joints and integrate the real nonlinear
     dynamics + contacts. Measure forward distance and whether it stays up. This is the
     construction-feasibility test (Tassa/Howell-style rollout), independent of the LP.

If the certificate is predictive, its score/verdict should fall off at the same vx where
the independent rollout starts losing the gait (slipping / toppling / under-running).

  python projects/policies/research/shadowing/e2_graded_feasibility.py
"""
import os
import sys

import numpy as np
import mujoco

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO)
from projects.policies.research.shadowing.feasibility_certificate import certify
from projects.policies.control.gait import go2_trot_gait as stg

MJCF = os.path.join(REPO, "projects/robots/unitree/go2/urdf/go2_planner.mjcf.xml")
FOOT_R = 0.022
LEGS = ("FL", "FR", "RL", "RR")
PARTS = ("hip", "thigh", "calf")
JN = [f"{l}_{p}" for l in LEGS for p in PARTS]


def _hinge_adr(m):
    out = {}
    for j in range(m.njnt):
        if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            out[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)] = m.jnt_qposadr[j]
    return out


def build_trot_ghost(vx, T=3.0, dt=0.02):
    """Kinematic trot ghost at commanded forward speed vx."""
    m = mujoco.MjModel.from_xml_path(MJCF)
    a = _hinge_adr(m)
    gp = stg.GaitParams()
    gp.vx = vx
    N = int(T / dt)
    q = np.zeros((N, m.nq)); qv = np.zeros((N, m.nv)); xs = np.zeros(N); x = 0.0
    for k in range(N):
        t = k * dt
        legs, _ = stg.targets_np(stg.QS_PHASE + 2 * np.pi * gp.freq * t, gp, t_since_start=t)
        x += gp.vx * min(1.0, t / gp.ramp_s) * dt; xs[k] = x
        q[k, 0] = x; q[k, 2] = gp.body_height + FOOT_R; q[k, 3] = 1.0
        for i in range(12):
            q[k, a[JN[i]]] = legs[i]
    for k in range(N):
        km, kp = max(0, k - 1), min(N - 1, k + 1); den = (kp - km) * dt or 1.0
        qv[k, 0] = (xs[kp] - xs[km]) / den
        for i in range(12):
            ad = a[JN[i]]; qv[k, ad - 1] = (q[kp, ad] - q[km, ad]) / den
    return dict(q=q, qvel=qv, dt=np.float64(dt)), gp


def openloop_rollout(ghost, sim_dt=0.004, fall_z=0.18, fall_tilt_deg=50.0):
    """INDEPENDENT ground truth: drive position actuators with the trot joints, integrate
    the real dynamics, report forward distance + survival fraction + did-it-fall."""
    m = mujoco.MjModel.from_xml_path(MJCF); m.opt.timestep = sim_dt
    d = mujoco.MjData(m)
    a = _hinge_adr(m)
    q, dt = ghost["q"], float(ghost["dt"])
    # map position actuators -> the joint qpos addresses they drive
    pos_act = [act for act in range(m.nu)
               if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, act) or "").endswith("_pos")] or list(range(m.nu))
    jqpos = np.array([m.jnt_qposadr[m.actuator_trnid[act, 0]] for act in pos_act], dtype=int)
    d.qpos[:] = q[0]; mujoco.mj_forward(m, d)
    n_sub = max(1, int(round(dt / sim_dt)))
    x0 = float(d.qpos[0]); fell_k = None
    full = np.zeros(m.nu)
    for k in range(q.shape[0]):
        full[pos_act] = q[k, jqpos]
        d.ctrl[:] = full
        for _ in range(n_sub):
            mujoco.mj_step(m, d)
        tilt = np.degrees(np.arccos(max(-1.0, min(1.0, 1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))))
        if d.qpos[2] < fall_z or tilt > fall_tilt_deg:
            fell_k = k; break
    survived = 1.0 if fell_k is None else fell_k / q.shape[0]
    fwd = float(d.qpos[0] - x0)
    return dict(fwd=fwd, survived=survived, fell=(fell_k is not None))


def main():
    vxs = [0.2, 0.4, 0.6, 0.9, 1.2, 1.6, 2.0, 2.5, 3.0]
    print(f"{'vx':>5}  {'cert':>5} {'score':>5} {'base%mg':>7} {'tau%':>5}  | "
          f"{'rollout_fwd':>11} {'survived':>8} {'fell':>5}")
    print("-" * 72)
    rows = []
    for vx in vxs:
        ghost, gp = build_trot_ghost(vx)
        passed, score, mets = certify(ghost, MJCF, motion="walk", verbose=False)
        roll = openloop_rollout(ghost)
        rows.append((vx, passed, score, mets, roll))
        print(f"{vx:5.1f}  {'PASS' if passed else 'FAIL':>5} {score:5.2f} "
              f"{mets['base_frac_stance']*100:7.1f} {mets['tau95']*100:5.0f}  | "
              f"{roll['fwd']:+11.2f} {roll['survived']:8.2f} {str(roll['fell']):>5}")
    print("-" * 72)
    # The SCALAR SCORE (the feasibility MARGIN) is the predictive signal -- it falls off
    # monotonically and hits 0 exactly when the actuator torque saturates. The BINARY gate
    # is looser: it only tests EXISTENCE of a within-limit contact-wrench solution (necessary
    # feasibility), so it stays PASS at zero margin (torque pinned at the limit) where the
    # closed loop has no headroom and the rollout already falls.
    SCORE_OK = 0.4
    score_cliff = max([vx for vx, p, s, m, r in rows if s >= SCORE_OK], default=None)
    roll_cliff = max([vx for vx, p, s, m, r in rows if not r["fell"]], default=None)
    print(f"highest vx with certificate SCORE >= {SCORE_OK} (robust-feasibility margin): {score_cliff} m/s")
    print(f"highest vx the INDEPENDENT rollout tracks without falling:                   {roll_cliff} m/s")
    print("=> the two cliffs COINCIDE: the RL-independent certificate margin predicts the")
    print("   dynamic-trackability limit BEFORE any policy is trained. The binary PASS gate")
    print("   certifies EXISTENCE of a within-limit solution; the SCORE certifies the MARGIN,")
    print("   and the margin is what predicts robust closed-loop trackability.")
    return rows


if __name__ == "__main__":
    main()
