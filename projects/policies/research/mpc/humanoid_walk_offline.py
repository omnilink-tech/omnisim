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

"""Offline HUMANOID-locomotion MPC prototype (MPPI residual on the human gait).

Robot-agnostic biped sibling of `quad_mpc_offline.py` (started life as
`g1_walk_offline.py`). SAME deterministic recipe that walks the quads (MPPI residual
planned in mujoco_warp on a nominal gait, zero model gap, NO training/H100) — for any
humanoid that ships a `*_human_gait` module + a deploy-matched MJCF.

  nominal control = <robot>_human_gait.targets_np(phase) at the current gait clock
  (ADVANCES over the horizon); MPPI samples a residual delta on the actuated joints,
  rolls K copies of the MJCF H control-ticks forward in mujoco_warp, scores by a
  BIPED-locomotion cost (track target speed + upright + height + rate-damp + straight),
  applies the softmax-weighted residual, receding horizon.

The gait modules share a 13-slot leg+waist layout (_L_HP=0,_L_HR,_L_HY,_L_KN,_L_AP,
_L_AR=5, _R_*=6..11, _WAIST=12) + a 10-slot arm layout (L/R shoulder p/r/y, elbow,
wrist). MJCF joint ORDER differs per robot (G1 contiguous; H1 hip_yaw-first, single
ankle, 4-DOF arms), so joints are mapped by NAME, not position.

  python projects/policies/research/mpc/humanoid_walk_offline.py --robot g1 --secs 8
  python projects/policies/research/mpc/humanoid_walk_offline.py --robot h1 --full --secs 8
  python projects/policies/research/mpc/humanoid_walk_offline.py --robot h1 --baseline

No training, no H100 — it's a planner. Needs torch+warp+mujoco_warp+mujoco.
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

ROBOTS = {
    "g1": dict(gait="projects.policies.control.gait.g1_human_gait",
               legs="projects/robots/unitree/g1/urdf/g1_legs_kp100.mjcf.xml",
               full="projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml"),
    # H1's only deploy MJCF (h1_legs_newton) already includes arms, so legs==full;
    # --full just adds the arm joints to the residual/nominal.
    "h1": dict(gait="projects.policies.control.gait.h1_human_gait",
               legs="projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml",
               full="projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml"),
}

# joint-name -> gait LEG slot (0..12) and ARM index (0..9). Side base + type offset.
_LEG_OFF = {"hip_pitch": 0, "hip_roll": 1, "hip_yaw": 2, "knee": 3,
            "ankle_pitch": 4, "ankle_roll": 5, "ankle": 4}   # H1 single ankle == pitch
_ARM_OFF = {"shoulder_pitch": 0, "shoulder_roll": 1, "shoulder_yaw": 2,
            "elbow": 3, "wrist_roll": 4, "wrist": 4}


def classify(name):
    """MJCF joint name -> ('leg', slot) | ('arm', idx) | None (skip)."""
    n = name.replace("_joint", "")
    if "torso" in n or "waist" in n:
        return ("leg", 12)
    side = 0 if n.startswith("left") else 6 if n.startswith("right") else None
    asid = 0 if n.startswith("left") else 5 if n.startswith("right") else None
    core = n.replace("left_", "").replace("right_", "")
    if any(k in core for k in ("hip", "knee", "ankle")) and side is not None:
        # longest key first so "ankle_pitch" beats "ankle"
        for key in sorted(_LEG_OFF, key=len, reverse=True):
            if core == key:
                return ("leg", side + _LEG_OFF[key])
        return None
    if any(k in core for k in ("shoulder", "elbow", "wrist")) and asid is not None:
        for key in sorted(_ARM_OFF, key=len, reverse=True):
            if core == key:
                return ("arm", asid + _ARM_OFF[key])
    return None


def build_ctrl(mjm, mujoco, full):
    """Map every actuated hinge -> its (qpos, qvel, ctrl-pos idx, source). Skip arms
    unless `full`. Returns a list of dicts + arrays. Actuators are interleaved
    [pos,vel] per hinge in hinge order, so ctrl-pos idx = 2*hinge_rank."""
    import mujoco as mj
    hinge = [j for j in range(mjm.njnt) if mjm.jnt_type[j] == mj.mjtJoint.mjJNT_HINGE]
    ctrl = []
    for rank, j in enumerate(hinge):
        nm = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)
        cl = classify(nm)
        if cl is None:
            continue
        kind, idx = cl
        if kind == "arm" and not full:
            continue
        ctrl.append(dict(name=nm, qpos=int(mjm.jnt_qposadr[j]), qvel=int(mjm.jnt_dofadr[j]),
                         ctrl=2 * rank, kind=kind, src=idx))
    return ctrl


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", choices=list(ROBOTS), default="g1")
    ap.add_argument("--secs", type=float, default=8.0)
    ap.add_argument("--K", type=int, default=96)
    ap.add_argument("--H", type=int, default=22)
    ap.add_argument("--sub", type=int, default=8)
    ap.add_argument("--sigma", type=float, default=0.05)
    ap.add_argument("--res-max", type=float, default=0.18)
    ap.add_argument("--lam", type=float, default=0.10)
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--vx", type=float, default=0.20)
    ap.add_argument("--freq", type=float, default=1.0)
    ap.add_argument("--style", default="ik")
    ap.add_argument("--lateral", default="lipm")
    ap.add_argument("--ramp-s", type=float, default=2.0)
    ap.add_argument("--baseline", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--w-vx", type=float, default=8.0)
    ap.add_argument("--w-up", type=float, default=30.0)
    ap.add_argument("--w-rate", type=float, default=5.0)
    ap.add_argument("--w-y", type=float, default=40.0)
    ap.add_argument("--w-yaw", type=float, default=8.0)
    ap.add_argument("--w-over", type=float, default=0.0)
    ap.add_argument("--full", action="store_true", help="arms in the residual (CAM windmill)")
    ap.add_argument("--arm-res-max", type=float, default=0.8)
    ap.add_argument("--arm-sigma", type=float, default=0.18)
    # DETERMINISTIC realtime balance (no rollouts): gait + reactive ankle/hip/heading PD.
    # This is the realtime-able controller the MPPI validated; it ports straight in-engine.
    ap.add_argument("--det", action="store_true", help="deterministic balance, NO MPPI (realtime)")
    ap.add_argument("--kp-p", type=float, default=2.0)   # pitch -> ankle_pitch lean
    ap.add_argument("--kd-p", type=float, default=0.3)
    ap.add_argument("--kp-r", type=float, default=2.0)   # roll -> hip_roll
    ap.add_argument("--kd-r", type=float, default=0.3)
    ap.add_argument("--ky", type=float, default=1.2)     # lateral drift -> hip_roll
    ap.add_argument("--kvy", type=float, default=0.4)
    ap.add_argument("--kp-yaw", type=float, default=1.5) # heading -> hip_yaw
    ap.add_argument("--kd-yaw", type=float, default=0.2)
    ap.add_argument("--sgn-ap", type=float, default=1.0)
    ap.add_argument("--sgn-hr", type=float, default=1.0)
    ap.add_argument("--sgn-yaw", type=float, default=1.0)
    ap.add_argument("--k-vx", type=float, default=0.0)   # speed regulation -> hip_pitch lean-back
    ap.add_argument("--sgn-vx", type=float, default=1.0)
    ap.add_argument("--step-width", type=float, default=0.0)  # >0 widens stance (lateral margin)
    ap.add_argument("--sway", type=float, default=-1.0)       # >=0 overrides gait sway amplitude
    args = ap.parse_args()

    import mujoco
    import warp as wp
    import mujoco_warp as mjw
    rc = ROBOTS[args.robot]
    stg = importlib.import_module(rc["gait"])
    try:
        gp = stg.GaitParams(vx=args.vx, freq=args.freq, style=args.style,
                            lateral=args.lateral, ramp_s=args.ramp_s)
    except TypeError:   # some gaits lack a knob -> fall back to the supported subset
        gp = stg.GaitParams(vx=args.vx, freq=args.freq, ramp_s=args.ramp_s)
    if args.step_width > 0 and hasattr(gp, "step_width"):
        gp.step_width = args.step_width
    if args.sway >= 0 and hasattr(gp, "sway"):
        gp.sway = args.sway
    mjm = mujoco.MjModel.from_xml_path(str(_REPO / rc["full" if args.full else "legs"]))
    ctrl = build_ctrl(mjm, mujoco, args.full)
    NJ = len(ctrl)
    to_qpos = np.array([c["qpos"] for c in ctrl], np.int32)
    to_qvel = np.array([c["qvel"] for c in ctrl], np.int32)
    to_ctrl = np.array([c["ctrl"] for c in ctrl], np.int32)
    is_leg = np.array([c["kind"] == "leg" for c in ctrl])
    src = np.array([c["src"] for c in ctrl], np.int32)
    nq, nv, nu = mjm.nq, mjm.nv, mjm.nu
    dt_ctrl = args.sub * mjm.opt.timestep
    omega = 2.0 * math.pi * gp.freq
    K, H, sub = args.K, args.H, args.sub
    res_max = np.where(is_leg, args.res_max, args.arm_res_max)
    sigma = np.where(is_leg, args.sigma, args.arm_sigma)
    # gait-slot -> control-vector index (legs only), for the deterministic balance law.
    slot2ci = {c["src"]: i for i, c in enumerate(ctrl) if c["kind"] == "leg"}
    AP, HR, HY, HP = (4, 10), (1, 7), (2, 8), (0, 6)  # ankle_pitch/hip_roll/hip_yaw/hip_pitch L,R

    def det_res(rq, rv):
        """Deterministic balance residual on the gait: ankle-pitch lean (forward balance),
        hip-roll (lateral balance + drift correction), hip-yaw (heading hold). No rollouts."""
        qw, qx, qy, qz = rq[3], rq[4], rq[5], rq[6]
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        wx, wy, wz = float(rv[3]), float(rv[4]), float(rv[5])
        y, vy, vx = float(rq[1]), float(rv[1]), float(rv[0])
        r = np.zeros(NJ)
        ap = args.sgn_ap * (args.kp_p * pitch + args.kd_p * wy)
        for s in AP:
            if s in slot2ci:
                r[slot2ci[s]] += ap
        hr = args.sgn_hr * (args.kp_r * roll + args.kd_r * wx + args.ky * y + args.kvy * vy)
        for s in HR:
            if s in slot2ci:
                r[slot2ci[s]] += hr
        yw = args.sgn_yaw * (args.kp_yaw * yaw + args.kd_yaw * wz)
        if HY[0] in slot2ci:
            r[slot2ci[HY[0]]] -= yw
        if HY[1] in slot2ci:
            r[slot2ci[HY[1]]] -= yw
        vxc = args.sgn_vx * args.k_vx * (vx - vx_target)   # over-speed -> hip lean-back
        for s in HP:
            if s in slot2ci:
                r[slot2ci[s]] += vxc
        return np.clip(r, -res_max, res_max)

    mjd = mujoco.MjData(mjm); mujoco.mj_forward(mjm, mjd)
    real = mjw.put_data(mjm, mjd, nworld=1, njmax=256, nconmax=256)
    rm = mjw.put_model(mjm)
    roll = None if args.baseline else mjw.put_data(mjm, mjd, nworld=K, njmax=256, nconmax=256)

    def seed(data, nw, qpos, qvel):
        qp = np.broadcast_to(qpos, (nw, nq)).astype(np.float32).reshape(data.qpos.numpy().shape)
        qv = np.broadcast_to(qvel, (nw, nv)).astype(np.float32).reshape(data.qvel.numpy().shape)
        data.qpos.assign(qp); data.qvel.assign(qv)

    def set_ctrl(data, nw, qtarg):
        c = data.ctrl.numpy().reshape(nw, nu)
        c[:, to_ctrl] = qtarg.astype(c.dtype)
        c[:, to_ctrl + 1] = 0.0
        data.ctrl.assign(c.reshape(data.ctrl.numpy().shape))

    def gait(phase, tss):
        """Map the gait's leg(13)+arm(10) slots onto the NJ controlled joints by name."""
        legs, arms, _sw = stg.targets_np(phase, gp, t_since_start=tss)
        legs = np.asarray(legs, np.float64); arms = np.asarray(arms, np.float64)
        out = np.where(is_leg, legs[src], arms[np.clip(src, 0, len(arms) - 1)])
        return out

    # init at the gait's DOUBLE-SUPPORT standing pose, base at pelvis height.
    stand = gait(stg.DS_PHASE, 0.0)
    qpos0 = mjd.qpos.copy().astype(np.float64)
    qpos0[0:3] = [0.0, 0.0, gp.pelvis_height]; qpos0[3:7] = [1, 0, 0, 0]
    for i in range(NJ):
        qpos0[to_qpos[i]] = stand[i]
    seed(real, 1, qpos0, np.zeros(nv))
    mjw.forward(rm, real)

    phase = stg.DS_PHASE
    nom_res = np.zeros(NJ)
    rng = np.random.default_rng(args.seed)
    n_ticks = int(args.secs / dt_ctrl)
    W_VX, W_UP, W_H, W_VZ, W_YAW, W_RES = args.w_vx, args.w_up, 150.0, 40.0, args.w_yaw, 0.2
    W_Y, W_RATE = args.w_y, args.w_rate
    vx_target = gp.vx
    zref = gp.pelvis_height - 0.03
    z_fall = 0.55 * (gp.pelvis_height / 0.755)   # scale the fall floor by robot height
    fell = False
    mode = ("BASELINE (no MPC)" if args.baseline else "DETERMINISTIC balance (realtime, no rollouts)"
            if args.det else f"MPPI K={K} H={H}")
    print(f"[walk] robot={args.robot} {mode} nj={NJ} (arms={'on' if args.full else 'off'}) "
          f"style={args.style} vx*={vx_target} freq={gp.freq} zref={zref:.2f} z_fall={z_fall:.2f} "
          f"ticks={n_ticks}")

    def rpy(qw, qx, qy, qz):
        r_ = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        p_ = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1))
        y_ = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return r_, p_, y_

    for t in range(n_ticks):
        rq = real.qpos.numpy().reshape(nq); rv = real.qvel.numpy().reshape(nv)
        tss = t * dt_ctrl
        if args.det:                                  # realtime path: no rollouts
            nom_res = det_res(rq, rv)
        elif not args.baseline and t % args.every == 0:
            seed(roll, K, rq, rv)
            noise = rng.normal(0, 1, (K, NJ)) * sigma[None, :]
            deltas = np.clip(nom_res[None, :] + noise, -res_max[None, :], res_max[None, :])
            deltas[0] = nom_res
            for h in range(H):
                qt = gait(phase + h * omega * dt_ctrl, tss + h * dt_ctrl)[None, :] + deltas
                set_ctrl(roll, K, qt)
                for _ in range(sub):
                    mjw.step(rm, roll)
            wp.synchronize()
            q = roll.qpos.numpy().reshape(K, nq); v = roll.qvel.numpy().reshape(K, nv)
            r_, p_, yw_ = rpy(q[:, 3], q[:, 4], q[:, 5], q[:, 6])
            vx_f = v[:, 0]; vz_f = v[:, 2]; rrate = v[:, 3]; prate = v[:, 4]
            J = (W_VX * (vx_f - vx_target) ** 2 + W_UP * (r_ * r_ + p_ * p_)
                 + W_RATE * (rrate * rrate + prate * prate) + W_YAW * yw_ * yw_
                 + W_Y * q[:, 1] ** 2 + W_H * np.maximum(0, zref - q[:, 2]) ** 2
                 + W_VZ * np.maximum(0, -vz_f) ** 2
                 + args.w_over * (np.maximum(0, vx_f - vx_target) ** 2 + np.maximum(0, p_) ** 2)
                 + W_RES * (deltas * deltas).sum(1))
            J += ((q[:, 2] < z_fall) | (np.abs(r_) > 0.7) | (np.abs(p_) > 0.7)) * 300.0
            w = np.exp(-(J - J.min()) / args.lam); w /= w.sum() + 1e-9
            nom_res = np.clip((w[:, None] * deltas).sum(0), -res_max, res_max)
        res = np.zeros(NJ) if args.baseline else nom_res
        set_ctrl(real, 1, (gait(phase, tss) + res)[None, :])
        for _ in range(sub):
            mjw.step(rm, real)
        phase += omega * dt_ctrl
        rq = real.qpos.numpy().reshape(nq)
        rr, rpch, _ = rpy(rq[3], rq[4], rq[5], rq[6])
        if rq[2] < z_fall or abs(rr) > 1.0 or abs(rpch) > 1.0:
            print(f"  FELL @ t={tss:.1f}s x={rq[0]:+.2f} z={rq[2]:.2f} roll={rr:+.2f} pit={rpch:+.2f}")
            fell = True; break
        if t % int(0.5 / dt_ctrl) == 0:
            print(f"  t={tss:4.1f}s x={rq[0]:+.2f} y={rq[1]:+.2f} z={rq[2]:.2f} "
                  f"vx={real.qvel.numpy().reshape(nv)[0]:+.2f} roll={rr:+.2f} pit={rpch:+.2f}")
    rq = real.qpos.numpy().reshape(nq)
    T = (t + 1) * dt_ctrl
    print(f"[walk] {args.robot} {'FELL' if fell else 'UPRIGHT'} | fwd x={rq[0]:+.2f} m in {T:.1f}s "
          f"= {rq[0]/T:+.3f} m/s | y-drift={rq[1]:+.2f} m")


if __name__ == "__main__":
    main()
