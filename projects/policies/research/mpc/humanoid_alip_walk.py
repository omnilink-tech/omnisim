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

"""Offline DETERMINISTIC ALIP / H-LIP foot-placement walker for the Unitree H1 (no RL).

The research verdict (reference_humanoid_walk_sota): pure sampling-MPC cannot durably
walk a biped; the proven deterministic approach is an explicit ALIP/capture-point
LATERAL step-placement law as the PRIMARY balance authority. Lateral balance is the
hard, actively-controlled axis (period-2, step-WIDTH regulated).

This is the honest gate: does ALIP foot-placement let H1 sustain single-support and walk
durably, OR does it hit the documented single-support wall? Offline-first in the same
mujoco_warp the deploy uses, so a positive result ports in-engine.

ALIP model (Gong & Grizzle 2021), one frontal/sagittal axis, CoM at height H, mass m:
  state s=[p ; L]  (p = CoM pos relative to stance foot, L = angular momentum about the
  contact point).  d/dt[p;L] = [[0, 1/(mH)],[mg, 0]][p;L],  eigenmode lambda=sqrt(g/H).
  Over a step of duration T:  s(T) = A s(0),
     A = [[cosh(lT),       sinh(lT)/(mH l)],
          [mH l sinh(lT),  cosh(lT)      ]].
  Foot exchange: new stance foot placed at displacement u from old -> p+ = p(T) - u, L cont.
  S2S: s_{k+1} = A s_k + B u_k, B=[-1;0]. Track a reference orbit (P1 sagittal forward,
  P2 lateral alternating step width) with deadbeat/LQR foot placement u = u* + K(s - s*).

  python projects/policies/research/mpc/humanoid_alip_walk.py --secs 12
  python projects/policies/research/mpc/humanoid_alip_walk.py --stand-check   # foundation only
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

_REPO = next(p for p in Path(__file__).resolve().parents
             if (p / "projects" / "policies").is_dir() or (p / "AGENTS.md").exists())
sys.path.insert(0, str(_REPO))
MJCF = _REPO / "projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml"

# H1 leg joint names (5-dof, no ankle-roll); ankle is slaved flat.
LEG_JOINTS = {
    "left":  ["left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
              "left_knee_joint", "left_ankle_joint"],
    "right": ["right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
              "right_knee_joint", "right_ankle_joint"],
}
FOOT_BODY = {"left": "left_ankle_link", "right": "right_ankle_link"}


def _alip_AB(lam, m, H, T):
    """ALIP step-to-step A,B for one axis (p=CoM rel stance, L about contact)."""
    c, s = math.cosh(lam * T), math.sinh(lam * T)
    A = np.array([[c, s / (m * H * lam)], [m * H * lam * s, c]])
    B = np.array([-1.0, 0.0])
    return A, B


def _deadbeat_K(A, B):
    """Scalar-input deadbeat: place both eigenvalues of (A+B K) at 0 (K is 1x2).
    For 2 states + 1 input, exact deadbeat in 2 steps via Ackermann."""
    # controllability [B, A B]
    AB = A @ B
    C = np.column_stack([B, AB])
    # desired char poly = s^2 (both poles 0) -> phi(A) = A@A
    phiA = A @ A
    e2 = np.array([0.0, 1.0])
    K = -e2 @ np.linalg.solve(C, phiA)   # Ackermann: K = -e2^T C^{-1} phi(A)
    return K


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=12.0)
    ap.add_argument("--sub", type=int, default=8)
    ap.add_argument("--T", type=float, default=0.40, help="step period (s)")
    ap.add_argument("--vx", type=float, default=0.0, help="desired forward speed (m/s)")
    ap.add_argument("--step-width", type=float, default=0.24, help="nominal lateral foot separation")
    ap.add_argument("--com-h", type=float, default=0.95, help="CoM height for the LIP")
    ap.add_argument("--lift", type=float, default=0.06, help="swing apex clearance")
    ap.add_argument("--kfp", type=float, default=1.0, help="foot-placement feedback scale (0..1)")
    ap.add_argument("--ank-bias", type=float, default=-0.06, help="backward ankle bias (CoM-centering)")
    ap.add_argument("--init-z", type=float, default=0.98, help="initial pelvis height")
    ap.add_argument("--mu", type=float, default=1.5, help="ground friction (match deploy ~2.0)")
    ap.add_argument("--start-shift", type=float, default=1.2, help="DS weight-shift seconds before stepping")
    # stance-leg balance feedback (H1 has NO ankle-roll -> lateral balance = hip_roll + foot placement)
    ap.add_argument("--kbal-r", type=float, default=0.8, help="roll -> stance hip_roll")
    ap.add_argument("--kbal-rd", type=float, default=0.15)
    ap.add_argument("--kbal-p", type=float, default=0.6, help="pitch -> stance ankle")
    ap.add_argument("--kbal-pd", type=float, default=0.12)
    ap.add_argument("--sgn-r", type=float, default=1.0)
    ap.add_argument("--stand-check", action="store_true", help="hold the stand pose, no stepping")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    import mujoco
    import warp as wp
    import mujoco_warp as mjw
    import mujoco as mj
    mjm = mujoco.MjModel.from_xml_path(str(MJCF))
    try:
        mjm.geom_friction[:, 0] = args.mu     # tangential friction (match deploy ground_mu) BEFORE put_model
    except Exception:
        pass
    nq, nv, nu = mjm.nq, mjm.nv, mjm.nv  # nu set below
    nu = mjm.nu
    g = 9.81
    H = args.com_h
    lam = math.sqrt(g / H)
    m_tot = float(sum(mjm.body_mass))
    T = args.T
    dt = mjm.opt.timestep
    dt_ctrl = args.sub * dt

    # --- joint/actuator/qpos maps ---
    jid = {mujoco.mj_id2name(mjm, mj.mjtObj.mjOBJ_JOINT, j): j for j in range(mjm.njnt)}
    def qadr(name): return int(mjm.jnt_qposadr[jid[name]])
    def dadr(name): return int(mjm.jnt_dofadr[jid[name]])
    # position actuator index per joint (interleaved [pos,vel])
    act_of = {}
    for a in range(nu):
        if int(mjm.actuator_trntype[a]) == int(mj.mjtTrn.mjTRN_JOINT):
            jj = int(mjm.actuator_trnid[a, 0]); nm = mujoco.mj_id2name(mjm, mj.mjtObj.mjOBJ_JOINT, jj)
            if float(mjm.actuator_biasprm[a, 1]) != 0.0 and nm not in act_of:
                act_of[nm] = a
    bid = {s: mujoco.mj_id2name and mujoco.mj_name2id(mjm, mj.mjtObj.mjOBJ_BODY, FOOT_BODY[s]) for s in FOOT_BODY}

    # --- CPU leg IK (exact, via MuJoCo FK + body jacobian) ---
    dik = mujoco.MjData(mjm)
    def leg_ik(qfull, leg, foot_world_target, q_seed):
        """Solve [hip_roll, hip_pitch, knee] of `leg` so its foot body reaches
        foot_world_target (world xyz). hip_yaw fixed at 0; ankle slaved flat
        (ankle = -(hip_pitch+knee)). Returns the 5 leg joint angles."""
        names = LEG_JOINTS[leg]
        dik.qpos[:] = qfull
        q = np.array(q_seed, float)   # [hy,hr,hp,kn,an]
        free = [1, 2, 3]              # hip_roll, hip_pitch, knee
        fb = bid[leg]
        for _ in range(30):
            q[0] = 0.0
            q[4] = -(q[2] + q[3])     # flat foot
            for k, nm in enumerate(names):
                dik.qpos[qadr(nm)] = q[k]
            mujoco.mj_forward(mjm, dik)
            f = np.array(dik.xpos[fb], float)
            err = foot_world_target - f
            if np.linalg.norm(err) < 1e-4:
                break
            Jp = np.zeros((3, nv)); Jr = np.zeros((3, nv))
            mujoco.mj_jacBody(mjm, dik, Jp, Jr, fb)
            cols = [dadr(names[i]) for i in free]
            Jf = Jp[:, cols]
            dq = Jf.T @ np.linalg.solve(Jf @ Jf.T + 1e-6 * np.eye(3), err)
            for c, i in enumerate(free):
                q[i] += float(np.clip(dq[c], -0.3, 0.3))
        q[0] = 0.0; q[4] = -(q[2] + q[3])
        return q

    # --- ALIP S2S + deadbeat gains ---
    A, B = _alip_AB(lam, m_tot, H, T)
    K = _deadbeat_K(A, B) * args.kfp
    # P2 lateral reference orbit: alternating step width. At touchdown the CoM is offset
    # from the new stance foot by +-(W/2) laterally, with the orbit's pre-impact L.
    # Solve the periodic P2: s*_{k+1}=A s*_k + B u*_k with u* alternating, p* alternating +-W/2.
    W = args.step_width
    # nominal lateral foot displacement alternates +-W (foot crosses from one side to other)
    # post-impact CoM-rel-pos alternates -+W/2 (CoM sits between the feet)
    p_star = W / 2.0
    # pre-impact pos from orbit periodicity: p_pre = A00 p0 + A01 L0 ; choose L0 so the
    # motion is symmetric (L flips sign): L0 = -L_pre. Solve: L_pre = A10 p0 + A11 L0,
    # with L0=-L_pre -> L_pre = A10 p0 - A11 L_pre -> L_pre=(A10 p0)/(1+A11).
    L0_lat = -(A[1, 0] * p_star) / (1.0 + A[1, 1])     # post-impact L for the P2 orbit
    s_star_lat_post = np.array([-p_star, L0_lat])      # CoM sits to -side of the +side foot... sign tuned at runtime
    # Sagittal P1: net forward u* = vx*T; choose orbit so velocity averages vx.
    L_sag_des = m_tot * H * args.vx                    # target sagittal angular momentum ~ m H v

    mjd = mujoco.MjData(mjm); mujoco.mj_forward(mjm, mjd)
    real = mjw.put_data(mjm, mjd, nworld=1, njmax=256, nconmax=256)
    rm = mjw.put_model(mjm)
    to_ctrl = {nm: act_of[nm] for leg in LEG_JOINTS for nm in LEG_JOINTS[leg]}

    # stand pose (spec): hip_pitch -0.30, knee 0.60, ankle -0.30 + ank_bias (CoM-centering)
    STAND = {"hip_pitch": -0.30, "knee": 0.60, "ankle": -0.30 + args.ank_bias}
    qpos0 = mjd.qpos.copy().astype(np.float64)
    qpos0[0:3] = [0.0, 0.0, args.init_z]; qpos0[3:7] = [1, 0, 0, 0]
    for leg in LEG_JOINTS:
        for nm in LEG_JOINTS[leg]:
            v = 0.0
            for key, val in STAND.items():
                if key in nm:
                    v = val
            qpos0[qadr(nm)] = v
    def set_real(qp):
        real.qpos.assign(np.broadcast_to(qp, (1, nq)).astype(np.float32).reshape(real.qpos.numpy().shape))
    set_real(qpos0); real.qvel.assign(np.zeros_like(real.qvel.numpy())); mjw.forward(rm, real)

    # nominal stand leg targets
    stand_q = {leg: np.array([qpos0[qadr(nm)] for nm in LEG_JOINTS[leg]]) for leg in LEG_JOINTS}
    ctrl = real.ctrl.numpy().reshape(1, nu).copy()
    def apply_leg(leg, q5):
        for k, nm in enumerate(LEG_JOINTS[leg]):
            a = to_ctrl[nm]; ctrl[0, a] = q5[k]; ctrl[0, a + 1] = 0.0
    for leg in LEG_JOINTS:
        apply_leg(leg, stand_q[leg])
    real.ctrl.assign(ctrl.reshape(real.ctrl.numpy().shape))

    def com_state():
        mujoco.mj_forward(mjm, mjd)  # mjd synced below
    def rpy(qw, qx, qy, qz):
        r = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        p = math.asin(max(-1, min(1, 2 * (qw * qy - qz * qx))))
        return r, p

    print(f"[alip] H1 m={m_tot:.1f} H={H} lam={lam:.2f} T={T} W={W} vx={args.vx} "
          f"K={np.round(K,2).tolist()} mode={'STAND' if args.stand_check else 'WALK'}")

    n_ticks = int(args.secs / dt_ctrl)
    # gait state
    stance = "right"; swing = "left"
    step_t = 0.0
    # foot world targets (start under hips)
    foot_w = {}
    rq = real.qpos.numpy().reshape(nq)
    for leg in LEG_JOINTS:
        mujoco.mj_forward(mjm, mjd)
    # init foot positions from FK
    dik.qpos[:] = rq
    mujoco.mj_forward(mjm, dik)
    for leg in LEG_JOINTS:
        foot_w[leg] = np.array(dik.xpos[bid[leg]], float)
    swing_from = foot_w[swing].copy(); swing_to = foot_w[swing].copy()
    k_step = 0
    fell = False

    for t in range(n_ticks):
        rq = real.qpos.numpy().reshape(nq); rv = real.qvel.numpy().reshape(nv)
        # sync a CPU MjData for CoM + angular momentum
        mjd.qpos[:] = rq; mjd.qvel[:] = rv
        mujoco.mj_forward(mjm, mjd)
        try:
            mujoco.mj_subtreeVel(mjm, mjd)
        except Exception:
            pass
        com = np.array(mjd.subtree_com[0], float)
        com_v = np.array(mjd.subtree_linvel[0], float)
        qw, qx, qy, qz = rq[3], rq[4], rq[5], rq[6]
        roll, pit = rpy(qw, qx, qy, qz)
        roll_rate, pit_rate = float(rv[3]), float(rv[4])

        # stance-leg balance offsets (H1: lateral=hip_roll, sagittal=ankle). Applied to the
        # stance leg's IK pose to push the CoP / lean the pelvis back to upright.
        def stance_bal():
            hr = args.sgn_r * (args.kbal_r * roll + args.kbal_rd * roll_rate)
            ap_ = args.kbal_p * pit + args.kbal_pd * pit_rate
            return hr, ap_

        def apply_leg_bal(leg, q5, is_stance):
            q = np.array(q5, float)
            if is_stance and not args.stand_check:
                hr, ap_ = stance_bal()
                q[1] += hr           # hip_roll
                q[4] += ap_          # ankle
            apply_leg(leg, q)

        # STARTUP: hold the stand in double support + lean the CoM over the FIRST stance
        # foot (so the first swing lift is stable -- the support-removal paradox fix).
        start_ticks = int(args.start_shift / dt_ctrl)
        if t < start_ticks and not args.stand_check:
            lean = (t / max(1, start_ticks)) * 0.10   # ramp a hip_roll lean toward stance(right)
            for lg in LEG_JOINTS:
                q = stand_q[lg].copy()
                q[1] += (-lean if lg == "right" else -lean)   # both hip_rolls -> shift pelvis +y? sign tuned
                apply_leg(lg, q)
            real.ctrl.assign(ctrl.reshape(real.ctrl.numpy().shape))
            for _ in range(args.sub):
                mjw.step(rm, real)
            if t % int(0.5 / dt_ctrl) == 0:
                print(f"  t={t*dt_ctrl:4.1f}s [startup] z={rq[2]:.3f} roll={roll:+.3f} pit={pit:+.3f}")
            continue
        if not args.stand_check:
            if not getattr(main, "_started", False):
                step_t = 0.0
                main._started = True
                dik.qpos[:] = rq; mujoco.mj_forward(mjm, dik)
                for lg in LEG_JOINTS:
                    foot_w[lg] = np.array(dik.xpos[bid[lg]], float)
                stance, swing = "right", "left"
            # foot placement update at the START of each step
            if step_t == 0.0:
                stf = foot_w[stance]
                # ALIP state per axis: p = com - stance_foot, L ~ m H v (about contact)
                px, py = com[0] - stf[0], com[1] - stf[1]
                Lx = -m_tot * H * com_v[1]    # lateral angular momentum about contact (x-axis)
                Ly = m_tot * H * com_v[0]     # sagittal angular momentum (y-axis)
                # predict end-of-step ALIP state
                sx = A @ np.array([px, Ly]);  syl = A @ np.array([py, Lx])
                # sagittal: desired post-next CoM-rel = small; foot u so next L -> L_sag_des
                u_sag = sx[0] - (0.0)  # place foot under predicted CoM (capture) + vel term
                u_sag += args.vx * T   # nominal forward stride
                u_sag += (K[0] * (Ly - L_sag_des)) * 0.0  # (sagittal fb folded via stride for now)
                # lateral: P2 orbit. target alternates side by step parity.
                side = +1.0 if swing == "left" else -1.0
                p_star_k = side * (W / 2.0)
                L_star_k = side * abs(L0_lat) * (-1.0)   # orbit L sign (tuned)
                s_meas = np.array([py, Lx])
                s_star = np.array([p_star_k, L_star_k])
                u_lat = (side * W) + float(K @ (s_meas - s_star))
                # swing foot target (world): place relative to the STANCE foot
                tgt = stf.copy()
                tgt[0] = stf[0] + u_sag
                tgt[1] = stf[1] + u_lat
                tgt[2] = 0.02
                swing_from = foot_w[swing].copy()
                swing_to = tgt
                k_step += 1
            # swing trajectory (smooth + lift)
            ph = min(1.0, step_t / T)
            s = ph * ph * (3 - 2 * ph)
            fp = swing_from + (swing_to - swing_from) * s
            fp = fp.copy(); fp[2] += args.lift * math.sin(math.pi * ph)
            # IK swing leg to fp; stance holds
            q_sw = leg_ik(rq, swing, fp, stand_q[swing])
            apply_leg_bal(swing, q_sw, False)
            apply_leg_bal(stance, stand_q[stance], True)
            real.ctrl.assign(ctrl.reshape(real.ctrl.numpy().shape))
            step_t += dt_ctrl
            if step_t >= T:   # touchdown -> switch stance
                foot_w[swing] = swing_to.copy()
                stance, swing = swing, stance
                step_t = 0.0

        for _ in range(args.sub):
            mjw.step(rm, real)
        rq = real.qpos.numpy().reshape(nq)
        if rq[2] < 0.55 or abs(roll) > 0.8 or abs(pit) > 0.8:
            print(f"  FELL @ t={t*dt_ctrl:.1f}s x={rq[0]:+.2f} y={rq[1]:+.2f} z={rq[2]:.2f} roll={roll:+.2f} pit={pit:+.2f} steps={k_step}")
            fell = True; break
        if t % int(0.5 / dt_ctrl) == 0:
            print(f"  t={t*dt_ctrl:4.1f}s x={rq[0]:+.2f} y={rq[1]:+.2f} z={rq[2]:.3f} "
                  f"roll={roll:+.3f} pit={pit:+.3f} steps={k_step} stance={stance}")
    rq = real.qpos.numpy().reshape(nq)
    Tt = (t + 1) * dt_ctrl
    print(f"[alip] {'FELL' if fell else 'UPRIGHT'} | x={rq[0]:+.2f}m y={rq[1]:+.2f}m in {Tt:.1f}s steps={k_step}")


if __name__ == "__main__":
    main()
