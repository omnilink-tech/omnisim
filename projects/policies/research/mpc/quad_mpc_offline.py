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

"""Offline quad-locomotion MPC prototype (MPPI residual on the trot gait).

ISOLATED from the in-engine MPC (src/omnisim/physics/OmNewtonBackend.cpp, owned by
a concurrent session). This validates the CONTROL DESIGN for quad walking via
sampling-MPC before any engine change — same mujoco_warp solver + same deploy MJCF
the engine uses, so it's a faithful proxy. Pattern mirrors the concurrent session's
own offline->in-engine path for the G1 stand.

Idea (the deterministic analog of our RL residual, but PLANNED not learned, zero-gap):
  nominal control = the trot gait targets at the current phase (advances over the
  horizon); MPPI samples a residual delta on the 12 leg joints, rolls K copies of
  the deploy model H control-ticks forward (gait advancing), scores each by a
  LOCOMOTION cost (reward forward distance + upright + straight), and applies the
  softmax-weighted residual to the real world. Receding horizon.

  python projects/policies/research/mpc/quad_mpc_offline.py --robot omniquad --secs 6

No training, no H100 — it's a planner. Needs torch+warp+mujoco_warp+mujoco (the
same stack as the GPU trainers).
"""
from __future__ import annotations

import argparse
import importlib
import math
import sys
from pathlib import Path

import numpy as np

_REPO = next(p for p in Path(__file__).resolve().parents
             if (p / "projects" / "policies").is_dir() or (p / "AGENTS.md").exists())
sys.path.insert(0, str(_REPO))

NJ = 12
ROBOTS = {
    # robot: (mjcf, gait_module, GaitParams kwargs matching the deploy launcher)
    "omniquad": ("omniquad_newton_fixed2.xml", "projects.policies.control.gait.omniquad_trot_gait",
                 dict(vx=0.4, freq=1.4, duty=0.6, step_height=0.06, body_height=0.55)),
    "go2":  ("go2_newton.xml", "projects.policies.control.gait.go2_trot_gait",
             dict(vx=0.4, freq=1.8, duty=0.6, step_height=0.05, body_height=0.30)),
    "b2":   ("b2_newton.xml", "projects.policies.control.gait.b2_trot_gait",
             dict(vx=0.5, freq=1.3, duty=0.6, step_height=0.08, body_height=0.50)),
}
MJCF_DIR = _REPO / "projects/policies/research/training/mjcf"

# Per-robot heading weights (W_YAW, W_WZ). Go2 (small/light) needs + likes strong
# yaw control -> dead straight; OmniQuad/B2 (heavier) are straighter WITHOUT it (the
# yaw-correcting residual perturbs their balance). Tuned empirically.
YAW_W = {"omniquad": (0.0, 0.0), "go2": (12.0, 4.0), "b2": (0.0, 0.0)}


def build_maps(mjm, mujoco):
    """Controller order FL,FR,RL,RR x (hip_x,hip_y,knee) -> qpos/qvel/ctrl indices.
    Classify hinges by body position + axis (names in the dump are anonymous).
    ctrl is interleaved [pos,vel] per hinge in hinge order."""
    to_qpos = np.zeros(NJ, np.int32); to_qvel = np.zeros(NJ, np.int32)
    to_ctrl = np.zeros(NJ, np.int32)
    label = {}; k_hinge = 0
    import mujoco as mj
    mjd = mujoco.MjData(mjm); mujoco.mj_forward(mjm, mjd)
    for j in range(mjm.njnt):
        if mjm.jnt_type[j] != mj.mjtJoint.mjJNT_HINGE:
            continue
        ax = mjm.jnt_axis[j]; rng = mjm.jnt_range[j]
        pos = mjd.xpos[mjm.jnt_bodyid[j]]
        leg = ("FL" if (pos[0] > 0 and pos[1] > 0) else "FR" if (pos[0] > 0)
               else "RL" if (pos[1] > 0) else "RR")
        # knee = the Y-hinge whose UPPER bound is negative (true for omniquad/go2/b2);
        # hip_x is the only X-axis hinge. (Generalizes across the quad URDFs.)
        joint = ("hip_x" if abs(ax[0]) > 0.5 else "knee" if rng[1] < 0.0 else "hip_y")
        label[(leg, joint)] = (mjm.jnt_qposadr[j], mjm.jnt_dofadr[j], k_hinge)
        k_hinge += 1
    for ci, leg in enumerate(("FL", "FR", "RL", "RR")):
        for cj, jn in enumerate(("hip_x", "hip_y", "knee")):
            qa, va, hi = label[(leg, jn)]
            i = ci * 3 + cj
            to_qpos[i] = qa; to_qvel[i] = va; to_ctrl[i] = 2 * hi
    assert mjm.nu == 2 * NJ, f"expected {2*NJ} actuators, got {mjm.nu}"
    return to_qpos, to_qvel, to_ctrl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", choices=list(ROBOTS), default="omniquad")
    ap.add_argument("--secs", type=float, default=6.0)
    ap.add_argument("--K", type=int, default=96)        # samples
    ap.add_argument("--H", type=int, default=16)        # horizon (control ticks)
    ap.add_argument("--sub", type=int, default=8)       # substeps / control tick
    ap.add_argument("--sigma", type=float, default=0.10)
    ap.add_argument("--res-max", type=float, default=0.35)
    ap.add_argument("--lam", type=float, default=0.2)
    ap.add_argument("--every", type=int, default=2)     # replan period (ticks)
    args = ap.parse_args()

    import mujoco
    import warp as wp
    import mujoco_warp as mjw
    stg = importlib.import_module(ROBOTS[args.robot][1])
    gp = stg.GaitParams(**ROBOTS[args.robot][2])
    mjcf = str(MJCF_DIR / ROBOTS[args.robot][0])
    mjm = mujoco.MjModel.from_xml_path(mjcf)
    to_qpos, to_qvel, to_ctrl = build_maps(mjm, mujoco)
    nq, nv, nu = mjm.nq, mjm.nv, mjm.nu
    dt_ctrl = args.sub * mjm.opt.timestep
    omega = 2.0 * math.pi * gp.freq
    nominal = stg.standing_pose(gp).astype(np.float64)
    K, H, sub = args.K, args.H, args.sub

    mjd = mujoco.MjData(mjm); mujoco.mj_forward(mjm, mjd)
    real = mjw.put_data(mjm, mjd, nworld=1, njmax=256, nconmax=256)
    roll = mjw.put_data(mjm, mjd, nworld=K, njmax=256, nconmax=256)
    rm = mjw.put_model(mjm)

    def seed(data, nw, qpos, qvel):
        qp = np.broadcast_to(qpos, (nw, nq)).astype(np.float32).reshape(data.qpos.numpy().shape)
        qv = np.broadcast_to(qvel, (nw, nv)).astype(np.float32).reshape(data.qvel.numpy().shape)
        data.qpos.assign(qp); data.qvel.assign(qv)

    def set_ctrl(data, nw, qtarg):  # qtarg (nw,NJ) -> position actuators
        c = data.ctrl.numpy().reshape(nw, nu)
        c[:, to_ctrl] = qtarg.astype(c.dtype)
        c[:, to_ctrl + 1] = 0.0
        data.ctrl.assign(c.reshape(data.ctrl.numpy().shape))

    # init real at standing pose, feet settled
    qpos0 = mjd.qpos.copy().astype(np.float64)
    qpos0[0:3] = [0.0, 0.0, gp.body_height + 0.01]; qpos0[3:7] = [1, 0, 0, 0]
    for i in range(NJ):
        qpos0[to_qpos[i]] = nominal[i]
    seed(real, 1, qpos0, np.zeros(nv))
    mjw.forward(rm, real)

    def trot(phase):
        q, _ = stg.targets_np(phase, gp, t_since_start=max(0.0, (phase - stg.QS_PHASE) / omega))
        return np.asarray(q, np.float64)

    phase = stg.QS_PHASE
    nom_res = np.zeros(NJ)
    rng = np.random.default_rng(0)
    n_ticks = int(args.secs / dt_ctrl)
    # Track a TARGET speed (don't reward raw distance -> no lunge-and-fall) and
    # weight upright/height hard so the gait stays stable.
    W_VX, W_UP, W_Y, W_H, W_VZ, W_RES, FALL = 10.0, 8.0, 6.0, 120.0, 40.0, 0.2, 300.0
    W_RATE = 3.0                      # damp base roll/pitch angular velocity (durability)
    W_YAW, W_WZ = YAW_W[args.robot]   # per-robot heading weights (see YAW_W)
    vx_target = gp.vx
    zref = gp.body_height - 0.02
    fell = False
    print(f"[mpc] robot={args.robot} K={K} H={H} sub={sub} dt_ctrl={dt_ctrl*1000:.0f}ms "
          f"ticks={n_ticks} sigma={args.sigma} res_max={args.res_max}")

    def rpy(qw, qx, qy, qz):
        roll_ = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch_ = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1))
        yaw_ = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return roll_, pitch_, yaw_

    for t in range(n_ticks):
        rq = real.qpos.numpy().reshape(nq); rv = real.qvel.numpy().reshape(nv)
        if t % args.every == 0:
            seed(roll, K, rq, rv)
            noise = rng.normal(0, 1, (K, NJ)) * args.sigma
            deltas = np.clip(nom_res[None, :] + noise, -args.res_max, args.res_max)
            deltas[0] = nom_res
            x0 = rq[0].copy(); y0 = rq[1].copy()
            for h in range(H):
                qt = trot(phase + h * omega * dt_ctrl)[None, :] + deltas   # (K,NJ)
                set_ctrl(roll, K, qt)
                for _ in range(sub):
                    mjw.step(rm, roll)
            wp.synchronize()
            q = roll.qpos.numpy().reshape(K, nq); v = roll.qvel.numpy().reshape(K, nv)
            r_, p_, yw_ = rpy(q[:, 3], q[:, 4], q[:, 5], q[:, 6])
            vx_f = v[:, 0]; vz_f = v[:, 2]; wz_f = v[:, 5]
            rrate_f = v[:, 3]; prate_f = v[:, 4]
            J = (W_VX * (vx_f - vx_target) ** 2 + W_UP * (r_ * r_ + p_ * p_)
                 + W_RATE * (rrate_f * rrate_f + prate_f * prate_f)
                 + W_YAW * yw_ * yw_ + W_WZ * wz_f * wz_f
                 + W_Y * (q[:, 1] - y0) ** 2 + W_H * np.maximum(0, zref - q[:, 2]) ** 2
                 + W_VZ * np.maximum(0, -vz_f) ** 2
                 + W_RES * (deltas * deltas).sum(1))
            J += ((q[:, 2] < 0.45 * gp.body_height / 0.55) | (np.abs(r_) > 0.8) | (np.abs(p_) > 0.8)) * FALL
            w = np.exp(-(J - J.min()) / args.lam); w /= w.sum() + 1e-9
            nom_res = np.clip((w[:, None] * deltas).sum(0), -args.res_max, args.res_max)
        # apply to the real world for one control tick
        set_ctrl(real, 1, (trot(phase) + nom_res)[None, :])
        for _ in range(sub):
            mjw.step(rm, real)
        phase += omega * dt_ctrl
        rq = real.qpos.numpy().reshape(nq)
        rr, rp, _ = rpy(rq[3], rq[4], rq[5], rq[6])
        if rq[2] < 0.40 * gp.body_height / 0.55 or abs(rr) > 1.0 or abs(rp) > 1.0:
            print(f"  FELL @ t={t*dt_ctrl:.1f}s x={rq[0]:+.2f} z={rq[2]:.2f} roll={rr:+.2f}")
            fell = True; break
        if t % int(0.5 / dt_ctrl) == 0:
            print(f"  t={t*dt_ctrl:4.1f}s x={rq[0]:+.2f} y={rq[1]:+.2f} z={rq[2]:.2f} "
                  f"vx={real.qvel.numpy().reshape(nv)[0]:+.2f}")
    rq = real.qpos.numpy().reshape(nq)
    T = (t + 1) * dt_ctrl
    print(f"[mpc] {'FELL' if fell else 'UPRIGHT'} | fwd x={rq[0]:+.2f} m in {T:.1f}s "
          f"= {rq[0]/T:+.3f} m/s | y-drift={rq[1]:+.2f} m")


if __name__ == "__main__":
    main()
