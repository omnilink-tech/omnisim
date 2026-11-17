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

"""In-engine driver for the centroidal-MPC + WBC stack (the Newton-MPC-stack plan).

Loaded by OmniSim's engine hook (runs INSIDE the mujoco_warp solver, so the model the MPC
reads/realizes IS the deployed model -> zero sim-to-deploy obs gap, the key advantage over
RL). It reads the live centroidal state, runs CentroidalMPC -> per-foot GRFs, maps them to
joint feedforward torques via WBC, and adds them on top of the deterministic position servo.

Run:
  OMNISIM_INENGINE_PYMOD=projects.policies.research.mpc.g1_centroidal.driver:cmpc_step
  (+ the humanoid_stand_deploy controller providing the squat servo, ke=400)

Discipline (plan ladder): CMPC_STAGE selects M4 (stand), M5 (lift), M6 (step). This file
ships M4 (double-support balance); the gait/swing for M5/M6 plug into _plan_contacts.
"""
import math
import os

import numpy as np

from .cam import ArmCAM
from .centroidal_mpc import CentroidalMPC
from .robot_io import CentroidalState
from .wbc import WBC


def _f(k, d):
    v = os.environ.get(k)
    try:
        return float(v) if v not in (None, "") else d
    except ValueError:
        return d


def _i(k, d):
    return int(_f(k, d))


def _log(world, msg):
    try:
        world._mpc_log("cmpc: " + msg)
    except Exception:
        import sys
        sys.stderr.write("[g1_cmpc] " + msg + "\n")


def _build(world):
    if getattr(world, "_cmpc_ready", None) is not None:
        return world._cmpc_ready
    world._cmpc_ready = False
    try:
        import mujoco as mj
    except Exception as e:
        _log(world, "mujoco import failed %r" % (e,)); return False
    sol = world.solver
    mjm = getattr(sol, "mj_model", None)
    m2nd = getattr(sol, "mjc_jnt_to_newton_dof", None)
    if mjm is None or m2nd is None:
        _log(world, "solver lacks mj_model/jntmap"); return False
    m2nd = m2nd.numpy()
    if m2nd.ndim == 2:
        m2nd = m2nd[0]
    world._cmpc_mj = mj
    # gravity onto the CPU model (Newton applies gravity via warp; CPU opt.gravity can be 0)
    try:
        if abs(float(mjm.opt.gravity[2])) < 1.0:
            mjm.opt.gravity[:] = [0.0, 0.0, -9.81]
    except Exception:
        pass
    # actuated joints (newton dof 6-28) -> (newton dof, mjc dof, qpos adr); + free base.
    act = []
    for j in range(int(mjm.njnt)):
        nd = int(m2nd[j])
        if 6 <= nd <= 28:
            act.append((nd, int(mjm.jnt_dofadr[j]), int(mjm.jnt_qposadr[j])))
    act.sort()
    world._cmpc_act_nd = [a[0] for a in act]
    world._cmpc_act_md = [a[1] for a in act]
    world._cmpc_act_qa = [a[2] for a in act]
    # geometric leg map (breadth-first newton layout): foot bodies via the ankle_roll joints.
    dcl = mj.MjData(mjm); mj.mj_forward(mjm, dcl)
    pelvis_z = 0.7
    for j in range(int(mjm.njnt)):
        if int(mjm.jnt_type[j]) == int(mj.mjtJoint.mjJNT_FREE):
            pelvis_z = float(dcl.xpos[int(mjm.jnt_bodyid[j])][2]); break
    cand = []
    for j in range(int(mjm.njnt)):
        nd = int(m2nd[j])
        if not (6 <= nd <= 28):
            continue
        bid = int(mjm.jnt_bodyid[j]); pos = np.array(dcl.xpos[bid], float)
        if pos[2] >= pelvis_z - 0.02:
            continue
        ax = np.abs(np.array(mjm.jnt_axis[j], float))
        cand.append((nd, bid, pos, int(np.argmax(ax)), int(mjm.jnt_qposadr[j])))
    # full per-leg 6-dof map (geometric, breadth-first newton layout): axis y -> hip_pitch
    # (top)/knee(mid)/ankle_pitch(bottom); axis x -> hip_roll(top)/ankle_roll(bottom); axis z
    # -> hip_yaw. order = [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll].
    order6 = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"]
    leg = {}
    for side, ys in (("left", 1.0), ("right", -1.0)):
        sj = [c for c in cand if c[2][1] * ys > 0]
        pit = sorted([c for c in sj if c[3] == 1], key=lambda c: -c[2][2])
        rol = sorted([c for c in sj if c[3] == 0], key=lambda c: -c[2][2])
        yaw = [c for c in sj if c[3] == 2]
        if len(pit) >= 3 and len(rol) >= 2 and len(yaw) >= 1:
            ch = {"hip_pitch": pit[0], "hip_roll": rol[0], "hip_yaw": yaw[0],
                  "knee": pit[1], "ankle_pitch": pit[2], "ankle_roll": rol[-1]}
            leg[side] = {"dof": [ch[k][0] for k in order6],
                         "qadr": [ch[k][4] for k in order6],
                         "foot_bid": ch["ankle_roll"][1]}
    if len(leg) != 2:
        _log(world, "leg map incomplete %s" % list(leg)); return False
    world._cmpc_leg = leg
    world._cmpc_nv = int(mjm.nv); world._cmpc_nq = int(mjm.nq)
    world._cmpc_d = mj.MjData(mjm)
    # total mass
    try:
        world._cmpc_mass = float(sum(mjm.body_mass))
    except Exception:
        world._cmpc_mass = 35.0
    cfg = {
        "kp_com": _f("CMPC_KP_COM", 60.0), "kd_com": _f("CMPC_KD_COM", 16.0),
        "kp_z": _f("CMPC_KP_Z", 140.0), "kd_z": _f("CMPC_KD_Z", 22.0),
        "kp_L": _f("CMPC_KP_L", 18.0), "kd_L": _f("CMPC_KD_L", 6.0),
        "w_force": _f("CMPC_W_FORCE", 6.0), "w_moment": _f("CMPC_W_MOMENT", 3.0),
        "mu": _f("CMPC_MU", 0.7), "reg": _f("CMPC_REG", 1e-3),
    }
    world._cmpc_mpc = CentroidalMPC(cfg)
    world._cmpc_wbc = WBC(world, leg, world._cmpc_act_md)
    # arm CAM (hip/arm angular-momentum strategy): moment beyond the ankle CoP limit.
    if _i("CMPC_CAM", 0):
        world._cmpc_cam = ArmCAM({
            "kp_pitch": _f("CMPC_CAM_KPP", 7.0), "kd_pitch": _f("CMPC_CAM_KDP", 0.9),
            "kp_roll": _f("CMPC_CAM_KPR", 5.0), "kd_roll": _f("CMPC_CAM_KDR", 0.6),
            "clamp": _f("CMPC_CAM_CLAMP", 1.8), "elbow_gain": _f("CMPC_CAM_ELBOW", 0.6),
            "sign": _f("CMPC_CAM_SIGN", 1.0)})
    else:
        world._cmpc_cam = None
    # capture_step (leg FK/IK) for the swing foot, + the gait scheduler (M5+).
    import sys as _sys
    _csd = os.path.join(os.environ.get("OMNISIM_HOME", "."), "projects", "policies",
                        "controllers", "humanoid_stand_deploy")
    if _csd not in _sys.path:
        _sys.path.insert(0, _csd)
    try:
        import capture_step as _cs
        world._cmpc_cs = _cs
    except Exception as e:
        world._cmpc_cs = None
        _log(world, "capture_step import failed %r" % (e,))
    from .gait import GaitScheduler
    if _i("CMPC_STAGE", 4) >= 5 and world._cmpc_cs is not None:
        world._cmpc_gait = GaitScheduler({
            "t_shift": _i("CMPC_T_SHIFT", 60), "t_swing": _i("CMPC_T_SWING", 40),
            "t_hold": _i("CMPC_T_HOLD", 40), "lift_h": _f("CMPC_LIFT_H", 0.05),
            "first_swing": os.environ.get("CMPC_FIRST_SWING", "right")})
    else:
        world._cmpc_gait = None
    world._cmpc_ctr = 0
    world._cmpc_comref = None
    world._cmpc_dt = float(getattr(world, "_n_substeps", 4)) * float(mjm.opt.timestep)
    world._cmpc_ready = True
    _log(world, "ready mass=%.1f nact=%d feet=%s pelvis_z=%.2f"
         % (world._cmpc_mass, len(act), {s: leg[s]["foot_bid"] for s in leg}, pelvis_z))
    return True


def _read(world):
    mj = world._cmpc_mj; mjm = world.solver.mj_model; live = world.solver.mjw_data
    d = world._cmpc_d
    d.qpos[:] = live.qpos.numpy().reshape(-1)[:world._cmpc_nq]
    d.qvel[:] = live.qvel.numpy().reshape(-1)[:world._cmpc_nv]
    mj.mj_forward(mjm, d)
    try:
        mj.mj_subtreeVel(mjm, d)              # fills subtree_linvel + subtree_angmom
    except Exception:
        pass
    st = CentroidalState()
    st.mass = world._cmpc_mass
    st.com = np.array(d.subtree_com[1], float)
    st.com_vel = np.array(d.subtree_linvel[1], float)
    try:
        st.L = np.array(d.subtree_angmom[1], float)
    except Exception:
        st.L = np.zeros(3)
    q = d.qpos
    st.quat = np.array([q[3], q[4], q[5], q[6]], float)
    st.omega = np.array(d.qvel[3:6], float)
    for lg in world._cmpc_leg:
        bid = world._cmpc_leg[lg]["foot_bid"]
        st.feet[lg] = np.array(d.xpos[bid], float)
        st.contact[lg] = bool(d.xpos[bid][2] < 0.12)
    st.qpos = d.qpos; st.qvel = d.qvel
    return st, d


def _plan_contacts(world, st):
    """Contact schedule. M4: double support (both feet). M5/M6 override via the gait."""
    stage = _i("CMPC_STAGE", 4)
    if stage <= 4:
        return ["left", "right"]
    # M5/M6 gait plugs in here (single support during swing).
    return [lg for lg in world._cmpc_leg if st.contact.get(lg, True)]


def _swing_targets(world, st, swing_leg, swing_z):
    """Lift the swing foot by swing_z (in place) via leg IK -> {newton_dof: q_target}."""
    cs = world._cmpc_cs
    lg = world._cmpc_leg[swing_leg]
    q0 = np.array([float(st.qpos[lg["qadr"][k]]) for k in range(6)])
    f0 = cs.leg_fk(q0, swing_leg)
    ftgt = f0.copy(); ftgt[2] += swing_z              # raise the foot (z up in pelvis frame)
    q = cs.leg_ik(ftgt, swing_leg, q0=q0)
    return {int(lg["dof"][k]): float(q[k]) for k in range(6)}


def cmpc_step(world):
    if not _build(world):
        return
    warm = _i("CMPC_WARM_TICKS", 200)
    world._cmpc_ctr += 1
    c = world._cmpc_ctr
    if c < warm:
        return
    try:
        st, d = _read(world)
        if world._cmpc_comref is None:
            world._cmpc_comref = st.com.copy()
            _log(world, "captured com_ref=%s mass=%.1f" % (np.round(world._cmpc_comref, 3).tolist(), st.mass))
        # M4-exit demo: lateral CoM-reference oscillation (proves dynamic CoM control / the
        # weight-shift primitive M5 needs). com_ref_y sweeps +/- amp at CMPC_SHIFT_HZ.
        com_ref = world._cmpc_comref.copy()
        amp = _f("CMPC_SHIFT_Y", 0.0)
        if amp > 0.0:
            hz = _f("CMPC_SHIFT_HZ", 0.25)
            tt = (c - _i("CMPC_WARM_TICKS", 200)) * float(world._cmpc_dt if hasattr(world, "_cmpc_dt") else 0.008)
            com_ref[1] = world._cmpc_comref[1] + amp * math.sin(2 * math.pi * hz * tt)
        # M5+ gait: shift com_ref over the stance foot + contact schedule + swing-foot lift.
        gait = getattr(world, "_cmpc_gait", None)
        swing_ovr = None
        if gait is not None:
            g = gait.update()
            sxy = st.feet[g["stance_foot"]][:2]
            cf = g["com_frac"]
            com_ref[0] = (1 - cf) * world._cmpc_comref[0] + cf * sxy[0]
            com_ref[1] = (1 - cf) * world._cmpc_comref[1] + cf * sxy[1]
            stance = g["stance"]
            if g["swing"]:
                try:
                    swing_ovr = _swing_targets(world, st, g["swing"], g["swing_z"])
                except Exception as e:
                    _log(world, "swing ik err %r" % (e,))
            world._cmpc_g = g
        else:
            stance = _plan_contacts(world, st)
        world._cmpc_com_ref_live = com_ref
        forces = world._cmpc_mpc.solve(st, com_ref, stance)
        tau = world._cmpc_wbc.forces_to_tau(world, d, forces)
        tau = np.clip(tau, -_f("CMPC_TAUMAX", 120.0), _f("CMPC_TAUMAX", 120.0))
        sgn = _f("CMPC_TAU_SIGN", 1.0)
        if world.control is None:
            world.control = world.model.control()
        jf = world.control.joint_f.numpy()
        jf[:] = 0.0
        for i, nd in enumerate(world._cmpc_act_nd):
            if 0 <= nd < len(jf):
                jf[nd] = float(sgn * tau[i])
        world.control.joint_f.assign(jf)
        # arm CAM: add a vigorous arm-windmill deviation on top of the controller's arm
        # command (position target) -> reaction moment beyond the ankle CoP.
        if world._cmpc_cam is not None:
            roll, pitch, _y = st.roll_pitch_yaw()
            dev = world._cmpc_cam.deviations(roll, pitch, float(st.omega[0]), float(st.omega[1]))
            tp = world.control.joint_target_q.numpy()
            for dd, vv in dev.items():
                if 0 <= dd < len(tp):
                    tp[dd] = float(tp[dd]) + float(vv)
            world.control.joint_target_q.assign(tp)
        # M5+ swing foot: override the swing leg's targets with the lift IK (servo tracks it).
        if swing_ovr:
            tp = world.control.joint_target_q.numpy()
            for dd, vv in swing_ovr.items():
                if 0 <= dd < len(tp):
                    tp[dd] = vv
            world.control.joint_target_q.assign(tp)
        world._mjc_dirty = True
        if c % 125 == 1:
            roll, pitch, _yaw = st.roll_pitch_yaw()
            _log(world, "t=%d z=%.3f r=%.3f p=%.3f comY=%.3f refY=%.3f errY=%.3f Fz=%.0f resF=%.1f resM=%.1f |tau|=%.1f"
                 % (c, st.com[2], roll, pitch, st.com[1], com_ref[1], com_ref[1] - st.com[1],
                    forces.get("_Fz", 0.0), forces.get("_resF", -1), forces.get("_resM", -1),
                    float(np.abs(tau).max())))
    except Exception as e:
        _log(world, "step error: %r" % (e,))
