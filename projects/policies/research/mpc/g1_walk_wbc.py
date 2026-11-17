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

"""Deterministic torque-level whole-body WALKING / push-recovery controller for the
Unitree G1, run as an external in-engine module (loaded via the engine hook
OMNISIM_INENGINE_PYMOD="projects.policies.research.mpc.g1_walk_wbc:walk_step").

It runs INSIDE OmniSim's Newton/mujoco_warp engine, so it reads the live full
rigid-body dynamics (mass matrix, Jacobians, gravity bias, contacts) from the same
solver that steps the deploy, and writes joint TORQUES (control.joint_f, EFFORT
mode). No RL.

Architecture (filled in incrementally; see g1-walking design):
  * capture-point / DCM footstep planner  -> decides WHEN + WHERE to step
  * whole-body QP/TSID with contact switching -> joint torques that balance
    (CoM/centroidal momentum + base orientation + posture) AND track the swing-foot
    Cartesian target, subject to the floating-base dynamics, stance-foot contact
    constraints and the friction cone.

Run (torque mode, the engine hook):
  OMNISIM_NEWTON_TORQUE_MODE=1 OMNISIM_INENGINE_PYMOD=projects.policies.research.mpc.g1_walk_wbc:walk_step
This module is plain Python + numpy + mujoco (the engine's interpreter has them);
edit + re-launch to iterate (no C++ rebuild).
"""
import os
import math
import numpy as np


def _envf(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def _log(world, msg):
    try:
        world._mpc_log("walk: " + msg)
    except Exception:
        import sys
        sys.stderr.write("[g1_walk] " + msg + "\n")


# ---------------------------------------------------------------------------
# One-time setup: joint maps, foot bodies, CPU MjData, gravity fix.
# ---------------------------------------------------------------------------
def _build(world):
    if getattr(world, "_walk_ready", False):
        return world._walk_ok
    world._walk_ready = True
    world._walk_ok = False
    try:
        import mujoco as mj
    except Exception as e:
        _log(world, "mujoco import failed %r" % (e,)); return False
    sol = world.solver
    mjm = getattr(sol, "mj_model", None)
    live = getattr(sol, "mjw_data", None)
    m2nj = getattr(sol, "mjc_jnt_to_newton_dof", None)
    if mjm is None or live is None or m2nj is None:
        _log(world, "solver missing mj_model/mjw_data/jntmap"); return False

    _mj = mj
    _arr = m2nj.numpy(); _arr = _arr[0] if _arr.ndim == 2 else _arr
    base = None; bqadr = 0; act = []; feet = {}
    # Newton DOF layout: free base = 0-5, robot revolutes = 6-28 (URDF order):
    #   6-11 left leg (hip_pitch/roll/yaw, knee, ankle_pitch, ankle_roll)
    #   12-17 right leg, 18 waist_yaw, 19-23 left arm, 24-28 right arm.
    for jj in range(int(mjm.njnt)):
        nd = int(_arr[jj])
        if nd < 0:
            continue
        if int(mjm.jnt_type[jj]) == int(_mj.mjtJoint.mjJNT_FREE) and nd == 0:
            base = list(range(int(mjm.jnt_dofadr[jj]), int(mjm.jnt_dofadr[jj]) + 6))
            bqadr = int(mjm.jnt_qposadr[jj])
        elif 6 <= nd <= 28:
            act.append((nd, int(mjm.jnt_dofadr[jj]), int(mjm.jnt_qposadr[jj])))
        if nd == 11:
            feet["left"] = int(mjm.jnt_bodyid[jj])
        if nd == 17:
            feet["right"] = int(mjm.jnt_bodyid[jj])
    act.sort()
    if not act or "left" not in feet or "right" not in feet:
        _log(world, "bad maps nact=%d feet=%s" % (len(act), feet)); return False

    # Force physical gravity on the CPU model (Newton applies gravity via its own warp
    # path; the CPU planning model can have opt.gravity == 0 -> qfrc_bias gravity-free).
    try:
        if abs(float(mjm.opt.gravity[2])) < 1.0:
            mjm.opt.gravity[0] = 0.0; mjm.opt.gravity[1] = 0.0; mjm.opt.gravity[2] = -9.81
    except Exception:
        pass

    world._w_mj = _mj
    world._w_d = _mj.MjData(mjm)
    world._w_nv = int(mjm.nv); world._w_nq = int(mjm.nq)
    world._w_base = base if base is not None else [0, 1, 2, 3, 4, 5]
    world._w_bqadr = bqadr
    world._w_act_nd = [a[0] for a in act]    # newton dof (control.joint_f index)
    world._w_act_md = [a[1] for a in act]    # mjc dof
    world._w_act_qa = [a[2] for a in act]    # mjc qpos adr
    world._w_feet = feet                     # leg -> mjc body id
    world._w_nact = len(act)
    # nd -> index in the actuated arrays (for targeting specific joints)
    world._w_nd2i = {nd: i for i, nd in enumerate(world._w_act_nd)}

    # Self-seed the squat into the LIVE solver state (torque mode spawns straight-legged
    # otherwise -> no gravity bias -> collapse). Squat by Newton DOF (== spec nominal):
    # hip_pitch -0.30, knee 0.52, ankle_pitch -0.23, shoulder_roll +/-0.20.
    squat = {6: -0.30, 9: 0.52, 10: -0.23, 12: -0.30, 15: 0.52, 16: -0.23, 20: 0.20, 25: -0.20}
    basez = _envf("WALK_SEED_BASEZ", 0.0)   # 0 = leave the world spawn height (squat feet ~at ground)
    # Build the qnom (mjc qpos) for the WBC/PD targets first (always available).
    try:
        _qp0 = world.solver.mjw_data.qpos.numpy().reshape(-1).copy()
        for i in range(world._w_nact):
            nd = world._w_act_nd[i]
            if nd in squat:
                _qp0[world._w_act_qa[i]] = squat[nd]
        world._w_qnom = _qp0
    except Exception:
        world._w_qnom = None
    # Seed the AUTHORITATIVE Newton joint state (model.joint_q + state buffers + eval_fk).
    # The mjw_data.qpos alone is overwritten from the Newton state on the first solver
    # step (the engine seeded straight legs in torque mode), so the knee snapped 0.52->0
    # and the PD launched. Newton joint_q layout: free base = q[0:7] (3 pos + 4 quat),
    # actuated joint with newton DOF nd -> joint_q index nd+1 (one extra q for the quat).
    try:
        import newton
        jq = world.model.joint_q.numpy()
        for nd, ang in squat.items():
            qi = nd + 1
            if 0 <= qi < len(jq):
                jq[qi] = ang
        if basez > 0.0 and len(jq) > 2:
            jq[2] = basez
        world.model.joint_q.assign(jq)
        for _st in (world.state_a, getattr(world, "state_b", None)):
            if _st is not None and hasattr(_st, "joint_q"):
                try:
                    _st.joint_q.assign(jq)
                except Exception:
                    pass
        newton.eval_fk(world.model, world.model.joint_q, world.model.joint_qd, world.state_a)
        # zero velocities
        try:
            jqd = world.model.joint_qd.numpy(); jqd[:] = 0.0
            world.model.joint_qd.assign(jqd)
            for _st in (world.state_a, getattr(world, "state_b", None)):
                if _st is not None and hasattr(_st, "joint_qd"):
                    try:
                        _st.joint_qd.assign(jqd)
                    except Exception:
                        pass
        except Exception:
            pass
        world._mjc_dirty = True
        _log(world, "self-seeded squat into Newton joint_q (base_z=%.2f, jq[7..10]=%.2f,%.2f)"
             % (basez, jq[7] if len(jq) > 7 else 0.0, jq[10] if len(jq) > 10 else 0.0))
    except Exception as e:
        _log(world, "Newton-state seed failed %r" % (e,))

    world._walk_ok = True
    _log(world, "ready nv=%d nact=%d base=%s feet=%s grav_z=%.2f"
         % (world._w_nv, world._w_nact, world._w_base, feet, float(mjm.opt.gravity[2])))
    return True


def _read_state(world):
    """Copy the live solver state into the CPU MjData and run mj_forward; return the
    CPU MjData with kinematics + dynamics quantities populated."""
    sol = world.solver
    mjm = sol.mj_model
    live = sol.mjw_data
    d = world._w_d
    _mj = world._w_mj
    d.qpos[:] = live.qpos.numpy().reshape(-1)[:world._w_nq]
    d.qvel[:] = live.qvel.numpy().reshape(-1)[:world._w_nv]
    _mj.mj_forward(mjm, d)
    try:
        _mj.mj_subtreeVel(mjm, d)
    except Exception:
        pass
    return d


def _apply_tau(world, tau_by_actidx):
    """Write actuated-joint torques (array indexed like _w_act_nd) to control.joint_f."""
    if world.control is None:
        world.control = world.model.control()
    sgn = _envf("WALK_TAU_SIGN", 1.0)   # Newton joint_f sign vs mjc tau (flip if it launches)
    jf = world.control.joint_f.numpy()
    jf[:] = 0.0
    nd = world._w_act_nd
    for i in range(world._w_nact):
        k = nd[i]
        if 0 <= k < len(jf):
            jf[k] = float(sgn * tau_by_actidx[i])
    world.control.joint_f.assign(jf)
    world._mjc_dirty = True


def _foot_contacts(world, d):
    """Return {'left': bool, 'right': bool} whether each foot is loaded, from the
    mujoco contact set (fallback: foot body height near ground)."""
    out = {"left": True, "right": True}
    try:
        mjm = world.solver.mj_model
        fb = world._w_feet
        # body height proxy: ankle body z near the ground => in contact
        for lg, bid in fb.items():
            out[lg] = bool(d.xpos[bid][2] < 0.12)
    except Exception:
        pass
    return out


# ---------------------------------------------------------------------------
# Per-tick entry point (called by the engine hook every 62.5 Hz tick).
# The WBC + footstep planner are filled in below once the design is finalized.
# ---------------------------------------------------------------------------
def walk_step(world):
    if not _build(world):
        return
    try:
        d = _read_state(world)
        _control(world, d)
    except Exception as e:
        _log(world, "step error: %r" % (e,))


def _control(world, d):
    """WBC-QP (double-support stand = Phase 1). Decision y=[q̈_robot(29), foot wrenches
    (6 per foot)]. HARD equalities: floating-base dynamics (6) + foot no-accel (6/foot).
    Cost: CoM(+height) + base-orientation + posture tasks, regularized. Solve the
    regularized KKT system (NOT lstsq -> no SVD blowup), friction-project the wrench,
    recover tau = M_a q̈ + h_a - Jc_a^T f."""
    mj = world._w_mj; mjm = world.solver.mj_model
    nv = world._w_nv
    base = world._w_base; amd = world._w_act_md; aqa = world._w_act_qa; bq = world._w_bqadr
    feet = world._w_feet; nact = world._w_nact
    rdofs = list(base) + list(amd)          # 29 robot mjc dofs: [base 0-5, actuated 6..]
    nr = len(rdofs)

    Kp_c = _envf("WALK_KP_COM", 60.0); Kd_c = _envf("WALK_KD_COM", 16.0)
    Kp_z = _envf("WALK_KP_Z", 140.0); Kd_z = _envf("WALK_KD_Z", 22.0)
    Kp_o = _envf("WALK_KP_ORI", 160.0); Kd_o = _envf("WALK_KD_ORI", 26.0)
    Kp_p = _envf("WALK_KP_POST", 30.0); Kd_p = _envf("WALK_KD_POST", 6.0)
    w_com = _envf("WALK_W_COM", 100.0); w_ori = _envf("WALK_W_ORI", 40.0); w_post = _envf("WALK_W_POST", 1.0)
    lam_qdd = _envf("WALK_REG_QDD", 1e-3); lam_f = _envf("WALK_REG_F", 1e-2)
    taumax = _envf("WALK_TAUMAX", 120.0); mu = _envf("WALK_MU", 0.7); cop = _envf("WALK_COP", 0.06)

    qp = np.array(d.qpos, dtype=np.float64); qd = np.array(d.qvel, dtype=np.float64)
    h = np.array(d.qfrc_bias, dtype=np.float64)

    # --- SETTLE phase: robust torque-PID gravity-comp hold (NO contact-force
    # assumptions) so the feet load and real contact establishes before the WBC's
    # rigid-contact assumptions engage. Avoids the seed-transient contact mismatch. ---
    world._w_ctr2 = getattr(world, "_w_ctr2", 0) + 1
    settle = int(_envf("WALK_SETTLE_TICKS", 40))
    if world._w_ctr2 <= settle:
        # High-Kp joint PD, NO gravity-comp h: the FEET carry the weight via contact,
        # so adding full gravity-comp at the joints double-supports and launches the
        # robot. This is the position-servo's torque-space equivalent (the proven stand).
        Kp = _envf("WALK_SET_KP", 400.0); Kd = _envf("WALK_SET_KD", 14.0)
        tm = _envf("WALK_TAUMAX", 120.0)
        if getattr(world, "_w_qnom_act", None) is None:
            world._w_qnom_act = np.array([world._w_qnom[aqa[i]] for i in range(nact)])
        tau = np.array([Kp*(world._w_qnom_act[i] - qp[aqa[i]]) - Kd*qd[amd[i]]
                        for i in range(nact)])
        tau = np.clip(tau, -tm, tm)
        _apply_tau(world, tau)
        if world._w_ctr2 <= 3 or world._w_ctr2 % 120 == 1:
            _log(world, "settle t=%d base_z=%.3f Lknee q=%.3f tgt=%.3f Lhip q=%.3f tgt=%.3f |tau|=%.1f" % (
                world._w_ctr2, qp[bq+2], qp[aqa[3]], world._w_qnom_act[3],
                qp[aqa[0]], world._w_qnom_act[0], float(np.abs(tau).max())))
        return

    # (model, dst, d.qM) -- the transposed form raises TypeError on mujoco
    # 3.8.1; verified empirically 2026-08-09.
    M = np.zeros((nv, nv))
    # mujoco <=3.8 wants (m, dst, d.qM); >=3.9 wants (m, d, dst) and may not
    # expose d.qM at all. Try modern first, fall back -- verified empirically
    # on both vendored builds during the newton 1.2 -> 1.5 migration.
    try:
        mj.mj_fullM(mjm, d, M)
    except (TypeError, AttributeError):
        mj.mj_fullM(mjm, M, d.qM)

    Mr = M[np.ix_(rdofs, rdofs)]; hr = h[rdofs]; qdr = qd[rdofs]

    com = np.array(d.subtree_com[1], dtype=np.float64)
    cvel = np.array(d.subtree_linvel[1], dtype=np.float64)
    af = _envf("WALK_VFILT", 0.4)
    _prev = getattr(world, "_w_cvf", None)
    world._w_cvf = cvel if _prev is None else (1.0 - af) * _prev + af * cvel
    com_vel = world._w_cvf
    if getattr(world, "_w_com_ref", None) is None:
        world._w_com_ref = com.copy()
    Jcom = np.zeros((3, nv)); mj.mj_jacSubtreeCom(mjm, d, Jcom, 1)
    Jcom_r = Jcom[:, rdofs]
    a_com = Kp_c * (world._w_com_ref - com) - Kd_c * com_vel
    a_com[2] = Kp_z * (world._w_com_ref[2] - com[2]) - Kd_z * com_vel[2]

    qw, qx, qy, qz = qp[bq+3], qp[bq+4], qp[bq+5], qp[bq+6]
    roll = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
    pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy-qz*qx))))
    wb = qdr[3:6]
    a_ori = np.array([-Kp_o*roll - Kd_o*wb[0], -Kp_o*pitch - Kd_o*wb[1], -Kd_o*wb[2]])

    if getattr(world, "_w_qnom_act", None) is None:
        world._w_qnom_act = np.array([world._w_qnom[aqa[i]] for i in range(nact)])
    a_post = np.array([Kp_p*(world._w_qnom_act[i] - qp[aqa[i]]) - Kd_p*qd[amd[i]] for i in range(nact)])

    legs = ["left", "right"]
    Jc = {}
    for lg in legs:
        Jp = np.zeros((3, nv)); Jr = np.zeros((3, nv))
        mj.mj_jacBodyCom(mjm, d, Jp, Jr, feet[lg])
        Jc[lg] = np.vstack([Jp[:, rdofs], Jr[:, rdofs]])    # 6 x nr

    nf = 6 * len(legs); ny = nr + nf
    H = np.zeros((ny, ny)); g = np.zeros(ny)

    def add_task(Jrt, a_des, w):
        Jt = np.zeros((Jrt.shape[0], ny)); Jt[:, :nr] = Jrt
        H[:, :] += w * (Jt.T @ Jt)
        g[:] += w * (Jt.T @ (-a_des))

    add_task(Jcom_r, a_com, w_com)
    E_o = np.zeros((3, nr)); E_o[0, 3] = E_o[1, 4] = E_o[2, 5] = 1.0
    add_task(E_o, a_ori, w_ori)
    E_p = np.zeros((nact, nr))
    for i in range(nact):
        E_p[i, 6 + i] = 1.0
    add_task(E_p, a_post, w_post)
    H[:nr, :nr] += lam_qdd * np.eye(nr)
    H[nr:, nr:] += lam_f * np.eye(nf)

    rows = []; bvals = []
    Ab = np.zeros((6, ny)); Ab[:, :nr] = Mr[0:6, :]
    for k, lg in enumerate(legs):
        Ab[:, nr+6*k:nr+6*k+6] = -Jc[lg][:, 0:6].T
    rows.append(Ab); bvals.append(-hr[0:6])
    for k, lg in enumerate(legs):
        Af = np.zeros((6, ny)); Af[:, :nr] = Jc[lg]
        rows.append(Af); bvals.append(np.zeros(6))
    A_eq = np.vstack(rows); b_eq = np.concatenate(bvals)

    ne = A_eq.shape[0]
    KKT = np.zeros((ny+ne, ny+ne))
    KKT[:ny, :ny] = H; KKT[:ny, ny:] = A_eq.T; KKT[ny:, :ny] = A_eq
    rhs = np.concatenate([-g, b_eq])
    try:
        sol = np.linalg.solve(KKT, rhs)
    except Exception:
        sol = np.linalg.lstsq(KKT, rhs, rcond=None)[0]
    y = sol[:ny]; qddr = y[:nr]; fvec = y[nr:]

    if not getattr(world, "_w_dbg", False):
        world._w_dbg = True
        _log(world, "DBG fin qp=%d qd=%d h=%d Mr=%d Jcom=%d Jc=%d Aeq=%d H=%d g=%d y=%d | "
                    "normMr=%.1f normJcom=%.2f normAeq=%.1f bq=%d nq=%d"
             % (int(np.isfinite(qp).all()), int(np.isfinite(qd).all()), int(np.isfinite(h).all()),
                int(np.isfinite(Mr).all()), int(np.isfinite(Jcom_r).all()),
                int(np.isfinite(Jc['left']).all() and np.isfinite(Jc['right']).all()),
                int(np.isfinite(A_eq).all()), int(np.isfinite(H).all()), int(np.isfinite(g).all()),
                int(np.isfinite(y).all()),
                float(np.linalg.norm(Mr)), float(np.linalg.norm(Jcom_r)), float(np.linalg.norm(A_eq)),
                bq, world._w_nq))

    for k in range(len(legs)):
        wv = fvec[6*k:6*k+6]
        Fz = max(0.0, wv[2]); wv[2] = Fz
        ft = math.hypot(wv[0], wv[1])
        if ft > mu*Fz and ft > 1e-9:
            s = (mu*Fz)/ft; wv[0] *= s; wv[1] *= s
        wv[3] = min(max(wv[3], -Fz*cop), Fz*cop)
        wv[4] = min(max(wv[4], -Fz*cop), Fz*cop)
        fvec[6*k:6*k+6] = wv

    tau = Mr[6:6+nact, :] @ qddr + hr[6:6+nact]
    for k, lg in enumerate(legs):
        tau = tau - Jc[lg][:, 6:6+nact].T @ fvec[6*k:6*k+6]
    if not np.all(np.isfinite(tau)):
        _log(world, "non-finite tau -> skip"); return
    tau = np.clip(tau, -taumax, taumax)
    _apply_tau(world, tau)

    world._w_ctr = getattr(world, "_w_ctr", 0) + 1
    if world._w_ctr % 60 == 1:
        _log(world, "roll=%.3f pitch=%.3f com_err=(%.3f,%.3f,%.3f) Fz=(%.0f,%.0f) |tau|=%.1f"
             % (roll, pitch, world._w_com_ref[0]-com[0], world._w_com_ref[1]-com[1],
                world._w_com_ref[2]-com[2], fvec[2], fvec[8], float(np.abs(tau).max())))
