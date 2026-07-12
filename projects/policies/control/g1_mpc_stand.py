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

"""g1_mpc_stand.py -- a sampling-MPC version of the G1 cube-defense STAND, with a
head-to-head against the deterministic reactive stand, in the OFFLINE MuJoCo harness.

WHY THIS EXISTS
---------------
The shipped cube-defense stand (projects/policies/controllers/humanoid_stand_deploy/)
is PURELY REACTIVE: a stiff pose hold + reactive ankle lean + arm-swing balance +
hip auto-trim. It survives the standard 8-cube barrage passively. The open question
(see docs/developer/g1-mpc-deterministic-brain-research.md) is whether a PREDICTIVE
controller -- one that only commits to a balance correction a true physics rollout
shows survives the next ~0.3 s -- recovers from BIGGER single shoves than the
reactive law can.

This is the STAND analogue of g1_mpc.py (which did the WALK). Standing is an EASIER
MPC target than walking: it is regulation around a fixed setpoint, so the horizon
collapses from H=55 (needed to plan a forward gait) to H~=20 (just beat the
sqrt(z/g)~=0.28 s inverted-pendulum divergence), and there is no gait direction for
the MuJoCo->Newton dynamics gap to flip.

WHAT IT DOES (apples-to-apples)
-------------------------------
Both controllers run in the SAME plant (the deploy-matched kp400 MJCF by default --
kp400 == the deploy's OMNISIM_NEWTON_TARGET_KE) and take the SAME impulse "shoves".
A shove is one control tick of momentum injected into the floating base: linear
(J/M) + a tip about the horizontal axis (J*h/I), parameterised in "cube-equivalents"
(1 cube = the real 0.5 kg @ 4 m/s = 2.0 kg.m/s). We sweep the shove magnitude and
report, for each controller, the largest shove it recovers from without falling.

  MPC      : sampling MPPI, residual on 8 leg + 4 shoulder balance joints, true
             physics rollouts, pure-regulation cost (upright + zero-velocity +
             height hold + effort). Deterministic (seeded).
  REACTIVE : a faithful, compact port of humanoid_stand_deploy's law (the gains
             from specs/g1_armsdown.json: lean kv/kp/kd, arm-balance kp/ki, hip
             trim) PLUS an ankle-roll PD on the weak (lateral) axis for fairness.

  python projects/policies/control/g1_mpc_stand.py hold                 # passive-hold sanity
  python projects/policies/control/g1_mpc_stand.py demo --ctrl mpc --cubes 20 --dir lat
  python projects/policies/control/g1_mpc_stand.py compare              # the head-to-head table
  python projects/policies/control/g1_mpc_stand.py compare --dir fwd --plant kp100
"""
from __future__ import annotations
import sys, os, math, time, argparse

if __name__ == "__main__" or __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))))  # repo root

import numpy as np
import mujoco
from mujoco import rollout

import projects.policies.control.g1_brain_sim as H
from projects.policies.control.gait import g1_human_gait as ghg
from projects.policies.control.g1_brain import LIM_LO, LIM_HI

FULL = mujoco.mjtState.mjSTATE_FULLPHYSICS
_REPO = H._REPO

PLANTS = {
    "kp400": _REPO / "projects/robots/unitree/g1/urdf/g1_full_kp400.mjcf.xml",
    "kp200": _REPO / "projects/robots/unitree/g1/urdf/g1_full_kp200.mjcf.xml",
    "kp100": _REPO / "projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml",
    # deploy-derived MJCF (same source as the Newton deploy's URDF) -> the most
    # Newton-faithful sidecar; its kp100 actuators get overridden to the deploy kp.
    "deploy": _REPO / "projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf",
}

# Real cube momentum: 0.5 kg @ 4 m/s. "cubes" in the CLI = this many cubes' worth
# of impulse delivered in one shove.
CUBE_P = 0.5 * 4.0          # 2.0 kg.m/s
ROBOT_M = 34.13            # G1 total mass (g1_physics.json)
TIP_ARM = 0.25            # impact height above CoM (m) -> tip moment arm
I_PITCH = ROBOT_M * 0.35 ** 2   # ~4.2 kg.m^2 effective trunk inertia

# Indices into LEGS_JOINTS (== g1_human_gait order): the 8 leg balance joints.
RES_LEG_J = np.array([0, 6, 1, 7, 4, 10, 5, 11])   # Lhp Rhp Lhr Rhr Lap Rap Lar Rar
# Indices into ARM_JOINTS: shoulder pitch L/R, shoulder roll L/R.
RES_ARM_J = np.array([0, 5, 1, 6])
SIGMA = np.array([0.10, 0.10, 0.08, 0.08, 0.10, 0.10, 0.06, 0.06,   # legs
                  0.16, 0.16, 0.12, 0.12])                          # shoulders


# --------------------------------------------------------------------------- #
#  Plant (g1_brain_sim.Plant, but with a selectable MJCF)                      #
# --------------------------------------------------------------------------- #
class StandPlant(H.Plant):
    def __init__(self, mjcf, foot_shift_x=0.0, kp=None, kd=None):
        self._mjcf = str(mjcf)
        # replicate H.Plant.__init__ with our MJCF
        self.m = mujoco.MjModel.from_xml_path(self._mjcf)
        self.m.opt.timestep = H.PHYS_DT
        # NEWTON-FAITHFUL SIDECAR: the OmniSim URDF importer (the deploy) places the
        # foot ~35mm further BACK than this MJCF, so the deploy CoM sits forward of
        # the foot front and tips forward where this model is stable. Shift the foot
        # collision geoms back by foot_shift_x so the planner sees the SAME stability
        # margin the Newton deploy has, and plans the correct CoM-back balance.
        if foot_shift_x:
            for bname in ("left_ankle_roll_link", "right_ankle_roll_link"):
                bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, bname)
                if bid < 0:
                    continue
                for gid in range(self.m.ngeom):
                    if self.m.geom_bodyid[gid] == bid:
                        self.m.geom_pos[gid][0] -= foot_shift_x
        self.d = mujoco.MjData(self.m)
        m = self.m

        def aid(name):
            i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if i < 0:
                raise RuntimeError(f"actuator {name} not found")
            return i

        def jadr(name):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            return m.jnt_qposadr[jid], m.jnt_dofadr[jid]

        self.leg_pos_act = np.array([aid(j + "_pos") for j in H.LEGS_JOINTS])
        self.leg_vel_act = np.array([aid(j + "_vel") for j in H.LEGS_JOINTS])
        self.arm_pos_act = np.array([aid(j + "_pos") for j in H.ARM_JOINTS])
        self.arm_vel_act = np.array([aid(j + "_vel") for j in H.ARM_JOINTS])
        self.leg_qadr = np.array([jadr(j)[0] for j in H.LEGS_JOINTS])
        self.leg_dadr = np.array([jadr(j)[1] for j in H.LEGS_JOINTS])
        self.arm_qadr = np.array([jadr(j)[0] for j in H.ARM_JOINTS])
        self.arm_dadr = np.array([jadr(j)[1] for j in H.ARM_JOINTS])
        # Override the position/velocity actuator gains to the deploy stiffness so the
        # sidecar's PD matches Newton's OMNISIM_NEWTON_TARGET_KE/KD (the deploy-derived
        # MJCF ships kp=100; the deploy stand runs kp=400).
        if kp is not None:
            for idx in (*self.leg_pos_act, *self.arm_pos_act):
                m.actuator_gainprm[idx, 0] = kp
                m.actuator_biasprm[idx, 1] = -kp
        if kd is not None:
            for idx in (*self.leg_vel_act, *self.arm_vel_act):
                m.actuator_gainprm[idx, 0] = kd
                m.actuator_biasprm[idx, 2] = -kd


def shove_dv(cubes, direction):
    """Return (dv_lin[3], dw_ang[3]) world-frame base velocity kick for a shove of
    `cubes` cube-equivalents from `direction` ('fwd' = pushed +x, 'lat' = pushed +y)."""
    J = cubes * CUBE_P
    dv_lin = np.zeros(3); dw = np.zeros(3)
    if direction == "fwd":
        dv_lin[0] = J / ROBOT_M           # pushed forward
        dw[1] = +J * TIP_ARM / I_PITCH    # pitch (about y)
    else:  # lateral -- the weak axis
        dv_lin[1] = J / ROBOT_M           # pushed sideways (+y)
        dw[0] = -J * TIP_ARM / I_PITCH    # roll (about x)
    return dv_lin, dw


def apply_shove(plant, cubes, direction):
    dv, dw = shove_dv(cubes, direction)
    plant.d.qvel[0:3] += dv
    plant.d.qvel[3:6] += dw


# --------------------------------------------------------------------------- #
#  Reactive stand -- compact port of humanoid_stand_deploy (g1_armsdown.json)  #
# --------------------------------------------------------------------------- #
class ReactiveStand:
    """Stiff pose hold + reactive ankle lean + arm-swing balance + hip auto-trim,
    plus an ankle-roll PD on the lateral axis (the deploy uses arms-for-roll; we
    add ankle-roll so the baseline isn't strawmanned on lateral shoves).

    NOTE: the deploy gains (lean kv=0.14/kp=1.6, arm kp=6.0) self-OSCILLATE on this
    offline g1_full MuJoCo plant (they were tuned for the Newton deploy). The values
    below are re-tuned (grid search) for the strongest possible rest survival HERE so
    the comparison isn't rigged against the baseline. Even so, no fixed/reactive
    config holds a durable rest stand on this plant -- the documented ~2.3 s reactive
    ceiling (docs/developer/g1-deterministic-brain.md). That is the point of the MPC."""
    # ankle lean (sagittal) -- re-tuned for this plant
    KV, KP, KD, LEAN_CLAMP, DEAD, HIP = 0.0, 1.2, 0.3, 0.15, 0.008, 0.35
    # arm balance -- re-tuned for this plant
    A_KP, A_KI, A_KR, A_KIR, A_CLAMP, A_ICLAMP = 1.5, 0.2, 1.2, 0.4, 1.5, 1.2
    # hip auto-trim
    T_KP, T_KI, T_CLAMP = 0.5, 0.30, 0.22
    # ankle roll PD (lateral, from g1_brain BrainConfig)
    R_KP, R_KD, R_CLAMP = 1.6, 0.12, 0.30

    def __init__(self, nominal_legs):
        self.nom_l = nominal_legs.astype(np.float64)
        self.nom_a = H.ARM_NOMINAL.astype(np.float64)
        self.arm_ip = self.arm_ir = self.trim_pi = 0.0

    def step(self, roll, pitch, rrate, prate, vx, dt):
        legs = self.nom_l.copy(); arms = self.nom_a.copy()
        # reactive ankle lean (sagittal)
        fwd = self.KV * vx + self.KD * (-prate) + self.KP * (-pitch)
        if abs(fwd) <= self.DEAD:
            fwd = 0.0
        else:
            fwd = math.copysign(abs(fwd) - self.DEAD, fwd)
        fwd = max(-self.LEAN_CLAMP, min(self.LEAN_CLAMP, fwd))
        legs[4] -= fwd; legs[10] -= fwd                 # ankle pitch L/R
        legs[0] += self.HIP * fwd; legs[6] += self.HIP * fwd   # hip pitch follows
        # ankle-roll PD (lateral)
        ar = self.R_KP * roll + self.R_KD * rrate
        ar = max(-self.R_CLAMP, min(self.R_CLAMP, ar))
        legs[5] += ar; legs[11] += ar                   # ankle roll L/R
        # hip auto-trim (slow sagittal return)
        self.trim_pi = max(-self.T_CLAMP, min(self.T_CLAMP,
                            self.trim_pi + self.T_KI * pitch * dt))
        cp = max(-self.T_CLAMP, min(self.T_CLAMP, self.T_KP * pitch + self.trim_pi))
        legs[0] += cp; legs[6] += cp
        # arm-swing balance (the primary fast lever)
        self.arm_ip = max(-self.A_ICLAMP, min(self.A_ICLAMP,
                          self.arm_ip + self.A_KI * pitch * dt))
        self.arm_ir = max(-self.A_ICLAMP, min(self.A_ICLAMP,
                          self.arm_ir + self.A_KIR * roll * dt))
        ap = max(-self.A_CLAMP, min(self.A_CLAMP, self.A_KP * pitch + self.arm_ip))
        arr = max(-self.A_CLAMP, min(self.A_CLAMP, self.A_KR * roll + self.arm_ir))
        arms[0] += ap; arms[5] += ap                    # shoulder pitch L/R
        arms[1] += arr; arms[6] += arr                  # shoulder roll L/R
        np.clip(legs, LIM_LO, LIM_HI, out=legs)
        return legs, arms


# --------------------------------------------------------------------------- #
#  Standing MPC                                                                #
# --------------------------------------------------------------------------- #
CFG = dict(
    K=96, H=22, iters=2, lam=0.3, nthread=8, res_max=0.5,
    w_tilt=4.0, w_roll=5.0, w_vx=2.0, w_vy=3.0, w_yaw=2.0,
    w_rate=0.4, w_bz=8.0, z_ref=0.74, w_res=0.4, w_term=8.0,
    # offset-free integral action on the SETPOINT (deploy only): a slow integral of
    # measured lean shifts the pose the MPC plans around, so it absorbs a steady
    # sidecar<->Newton CoM mismatch the model can't see. It moves the SETPOINT (not a
    # competing command), so it stays coherent with the MPC's fast balance.
    trim_ki=0.6, trim_kir=0.0, trim_clamp=0.18, trim_hip=0.35,
)


def _quat_rpy(q):
    qw, qx, qy, qz = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def _quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


class G1StandMPC:
    def __init__(self, plant, nominal_legs, cfg=None, nominal_arms=None):
        self.cfg = dict(CFG, **(cfg or {}))
        c = self.cfg
        self.plant = plant
        self.m = plant.m
        self.nom_l = nominal_legs.astype(np.float64)
        self.nom_a = (H.ARM_NOMINAL if nominal_arms is None else nominal_arms).astype(np.float64)
        self.nq = self.m.nq
        self.nstate = mujoco.mj_stateSize(self.m, FULL)
        self.lpa, self.apa = plant.leg_pos_act, plant.arm_pos_act
        self.res_act = np.concatenate([plant.leg_pos_act[RES_LEG_J],
                                       plant.arm_pos_act[RES_ARM_J]])
        self.NR = len(self.res_act)
        self.SUB = H.SUBSTEPS
        self.datas = [mujoco.MjData(self.m) for _ in range(int(c["nthread"]))]
        self.roll = rollout.Rollout(nthread=int(c["nthread"]))
        self.nominal = np.zeros(self.NR)        # warm-started residual knot
        # static base control over the horizon (hold the stand pose)
        self.base_l = self.nom_l.copy()         # untrimmed leg nominal
        self.ifwd = 0.0                          # sagittal setpoint-trim integral
        self.iroll = 0.0                         # lateral setpoint-trim integral
        self.base1 = np.zeros(self.m.nu)
        self.base1[self.lpa] = self.nom_l
        self.base1[self.apa] = self.nom_a

    def close(self):
        self.roll.close()

    def plan(self, rng):
        c = self.cfg; K, Hh, nu = c["K"], c["H"], self.m.nu
        base = np.broadcast_to(self.base1, (Hh, nu))
        s0 = np.zeros(self.nstate); mujoco.mj_getState(self.m, self.plant.d, s0, FULL)
        s0b = np.broadcast_to(s0, (K, self.nstate)).copy()
        for _ in range(c["iters"]):
            noise = rng.normal(0, 1, size=(K, self.NR)) * SIGMA
            knots = np.clip(self.nominal[None] + noise, -c["res_max"], c["res_max"])
            knots[0] = self.nominal                          # incumbent
            ctrl = np.broadcast_to(base, (K, Hh, nu)).copy()
            ctrl[:, :, self.res_act] += knots[:, None, :]
            ctrl[:, :, self.lpa] = np.clip(ctrl[:, :, self.lpa], LIM_LO, LIM_HI)
            control = np.repeat(ctrl, self.SUB, axis=1)
            st, _ = self.roll.rollout(self.m, self.datas, s0b, control)
            st = st[:, self.SUB - 1::self.SUB, :]            # per control-step end
            roll, pitch, yaw = _quat_rpy(st[..., 4:8])
            self._mppi(knots, st[..., 3], roll, pitch, yaw,
                       st[..., 1 + self.nq], st[..., 2 + self.nq],   # vx, vy
                       st[..., 4 + self.nq], st[..., 5 + self.nq])   # rrate, prate
        return self.nominal

    def _mppi(self, knots, bz, roll, pitch, yaw, vx, vy, rrate, prate):
        c = self.cfg; K, Hh = c["K"], c["H"]
        Js = (c["w_tilt"] * pitch * pitch + c["w_roll"] * roll * roll
              + c["w_yaw"] * yaw * yaw
              + c["w_vx"] * vx * vx + c["w_vy"] * vy * vy
              + c["w_rate"] * (prate * prate + rrate * rrate)
              + c["w_bz"] * np.maximum(0, c["z_ref"] - bz) ** 2)
        fallen = (bz < 0.45) | (np.abs(roll) > 0.8) | (np.abs(pitch) > 0.8)
        firstfall = np.where(fallen.any(axis=1), fallen.argmax(axis=1), Hh)
        valid = np.arange(Hh)[None, :] < firstfall[:, None]
        J = (Js * valid).sum(axis=1) + 50.0 * (Hh - firstfall)
        ti = np.clip(firstfall - 1, 0, Hh - 1); ar = np.arange(K)
        J += c["w_term"] * (pitch[ar, ti] ** 2 + roll[ar, ti] ** 2
                            + vx[ar, ti] ** 2 + vy[ar, ti] ** 2)
        J += c["w_res"] * (knots ** 2).sum(axis=1)
        w = np.exp(-(J - J.min()) / c["lam"]); w /= w.sum() + 1e-9
        self.nominal = np.clip((w[:, None] * knots).sum(axis=0),
                               -c["res_max"], c["res_max"])

    def targets(self, r0):
        legs = self.nom_l.copy(); arms = self.nom_a.copy()
        legs[RES_LEG_J] += r0[:8]; arms[RES_ARM_J] += r0[8:]
        np.clip(legs, LIM_LO, LIM_HI, out=legs)
        return legs, arms

    # ---- deploy use: drive the sidecar from a live Newton-state reading ---- #
    def sync_state(self, bz, roll, pitch, yaw, vx, vy, vz,
                   rrate, prate, yrate, joint_q, joint_qd):
        """Copy the real (Newton) robot state into the MuJoCo planning sidecar.
        joint_q/joint_qd are 13-vectors in H.LEGS_JOINTS order."""
        d = self.plant.d
        d.qpos[0:3] = (0.0, 0.0, bz)                 # absolute x/y irrelevant
        d.qpos[3:7] = _quat_from_rpy(roll, pitch, yaw)
        d.qpos[self.plant.leg_qadr] = np.asarray(joint_q, dtype=np.float64)
        d.qpos[self.plant.arm_qadr] = self.nom_a
        d.qvel[0:3] = (vx, vy, vz)
        d.qvel[3:6] = (rrate, prate, yrate)
        d.qvel[self.plant.leg_dadr] = np.asarray(joint_qd, dtype=np.float64)
        d.qvel[self.plant.arm_dadr] = 0.0
        mujoco.mj_forward(self.plant.m, d)

    def update_trim(self, roll, pitch, dt):
        """Offset-free integral action: slowly shift the SETPOINT (the pose the MPC
        plans around) to drive a steady measured lean to zero. Forward lean
        (pitch<0) -> lean back via ankle (more negative) + a little hip; roll
        likewise on ankle-roll. Coherent with the MPC (same nominal drives both the
        planning base and the output), unlike a competing reactive overlay."""
        c = self.cfg; tc = c["trim_clamp"]
        self.ifwd = float(np.clip(self.ifwd + c["trim_ki"] * (-pitch) * dt, -tc, tc))
        self.iroll = float(np.clip(self.iroll + c["trim_kir"] * roll * dt, -tc, tc))
        self.nom_l = self.base_l.copy()
        self.nom_l[4] -= self.ifwd; self.nom_l[10] -= self.ifwd          # ankle pitch
        self.nom_l[0] += c["trim_hip"] * self.ifwd                        # hip pitch L
        self.nom_l[6] += c["trim_hip"] * self.ifwd                        # hip pitch R
        self.nom_l[5] += self.iroll; self.nom_l[11] += self.iroll         # ankle roll
        np.clip(self.nom_l, LIM_LO, LIM_HI, out=self.nom_l)
        self.base1[self.lpa] = self.nom_l

    def plan_targets(self, rng):
        """Plan against the (already-synced) sidecar; return (legs13, arms10)."""
        return self.targets(self.plan(rng))


# Deploy pose: g1_armsdown spec nominal (deep squat + arms hanging down),
# in H.LEGS_JOINTS / H.ARM_JOINTS order. ank_bias folds into the ankle pitch.
def deploy_nominal(ank_bias=-0.10):
    legs = np.zeros(13)
    legs[0] = legs[6] = -0.30          # hip pitch L/R
    legs[3] = legs[9] = 0.52           # knee L/R
    legs[4] = legs[10] = -0.23 + ank_bias   # ankle pitch L/R
    arms = np.zeros(10)
    arms[0] = arms[5] = 1.5708         # shoulder pitch L/R (arms down)
    arms[3] = arms[8] = 0.5            # elbow L/R
    return legs, arms


# --------------------------------------------------------------------------- #
#  Trial driver                                                               #
# --------------------------------------------------------------------------- #
def nominal_stand():
    gp = ghg.GaitParams(style="ik", vx=0.0, freq=1.3, x0=0.0)
    return ghg.standing_pose(gp).astype(np.float64)


def run_trial(ctrl_kind, plant_key="kp400", cubes=0.0, direction="lat",
              seconds=4.0, settle_s=0.6, seed=0, trace=False):
    plant = StandPlant(PLANTS[plant_key])
    nom = nominal_stand()
    plant.reset(nom, H.ARM_NOMINAL)
    for _ in range(int(round(settle_s / H.CTRL_DT))):
        plant.set_ctrl(nom, H.ARM_NOMINAL); plant.substep()

    mpc = react = None
    if ctrl_kind == "mpc":
        mpc = G1StandMPC(plant, nom); rng = np.random.default_rng(seed)
    else:
        react = ReactiveStand(nom)

    if cubes > 0:
        apply_shove(plant, cubes, direction)

    n = int(round(seconds / H.CTRL_DT)); peak = 0.0; fell = None
    for k in range(n):
        t = k * H.CTRL_DT
        s = plant.read_state(t)
        if mpc is not None:
            r0 = mpc.plan(rng); legs, arms = mpc.targets(r0)
        else:
            legs, arms = react.step(s.roll, s.pitch, s.roll_rate,
                                    s.pitch_rate, s.vx, H.CTRL_DT)
        plant.set_ctrl(legs, arms); plant.substep()
        s2 = plant.read_state(t)
        tilt = math.hypot(s2.roll, s2.pitch); peak = max(peak, tilt)
        if (s2.bz < 0.45 or abs(s2.roll) > 0.8 or abs(s2.pitch) > 0.8) and fell is None:
            fell = t; break
        if trace and k % 12 == 0:
            print(f"    t={t:4.2f}s bz={s2.bz:.3f} roll={s2.roll:+.3f} "
                  f"pitch={s2.pitch:+.3f} |tilt|={tilt:.3f}")
    if mpc is not None:
        mpc.close()
    s_end = plant.read_state(0.0)
    final_tilt = math.hypot(s_end.roll, s_end.pitch)
    recovered = fell is None and final_tilt < 0.12
    return dict(fell=fell, peak=peak, final_tilt=final_tilt, recovered=recovered)


def cmd_hold(args):
    print(f"# passive-hold sanity ({args.plant}, no shove, {args.seconds:.0f}s)")
    for ck in ("reactive", "mpc"):
        r = run_trial(ck, args.plant, cubes=0.0, seconds=args.seconds, trace=args.trace)
        tag = "FELL@%.2fs" % r["fell"] if r["fell"] else "STANDS"
        print(f"  {ck:9s}: {tag}  peak|tilt|={r['peak']:.3f}  final|tilt|={r['final_tilt']:.3f}")


def cmd_demo(args):
    print(f"# {args.ctrl} stand, {args.cubes:g}-cube {args.dir} shove ({args.plant})")
    t0 = time.time()
    r = run_trial(args.ctrl, args.plant, cubes=args.cubes, direction=args.dir,
                  seconds=args.seconds, trace=True)
    tag = "FELL@%.2fs" % r["fell"] if r["fell"] else "RECOVERED" if r["recovered"] else "STAYED-UP(tilted)"
    print(f"  -> {tag}  peak|tilt|={r['peak']:.3f}  final|tilt|={r['final_tilt']:.3f}  "
          f"({time.time()-t0:.0f}s wall)")


def cmd_compare(args):
    levels = [0.0] + [float(x) for x in args.cubes.split(",")]
    print(f"# HEAD-TO-HEAD: survival vs single-shove impulse")
    print(f"# plant={args.plant}  direction={args.dir}  window={args.seconds:.0f}s  "
          f"(1 cube = 0.5kg@4m/s = 2.0 kg.m/s)\n")
    print(f"{'cubes':>6} {'impulse':>9} {'REACTIVE':>22} {'MPC':>22}")
    print(f"{'':>6} {'kg.m/s':>9} {'':>22} {'':>22}")
    react_lim = mpc_lim = 0.0
    for cu in levels:
        rr = run_trial("reactive", args.plant, cubes=cu, direction=args.dir, seconds=args.seconds)
        t0 = time.time()
        rm = run_trial("mpc", args.plant, cubes=cu, direction=args.dir, seconds=args.seconds)
        def fmt(r):
            if r["fell"]:
                return f"FELL@{r['fell']:.2f}s pk{r['peak']:.2f}"
            return ("recovered" if r["recovered"] else "up,tilt") + f" pk{r['peak']:.2f}"
        if not rr["fell"]:
            react_lim = cu
        if not rm["fell"]:
            mpc_lim = cu
        print(f"{cu:>6g} {cu*CUBE_P:>9.1f} {fmt(rr):>22} {fmt(rm):>22}   ({time.time()-t0:.0f}s)")
    print(f"\n# max single shove survived:  REACTIVE = {react_lim:g} cubes "
          f"({react_lim*CUBE_P:.1f} kg.m/s)   MPC = {mpc_lim:g} cubes ({mpc_lim*CUBE_P:.1f} kg.m/s)")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    h = sub.add_parser("hold"); h.add_argument("--plant", default="kp400")
    h.add_argument("--seconds", type=float, default=3.0); h.add_argument("--trace", action="store_true")
    h.set_defaults(fn=cmd_hold)
    d = sub.add_parser("demo")
    d.add_argument("--ctrl", choices=["mpc", "reactive"], default="mpc")
    d.add_argument("--plant", default="kp400"); d.add_argument("--cubes", type=float, default=20.0)
    d.add_argument("--dir", choices=["fwd", "lat"], default="lat")
    d.add_argument("--seconds", type=float, default=4.0); d.set_defaults(fn=cmd_demo)
    c = sub.add_parser("compare")
    c.add_argument("--plant", default="kp400"); c.add_argument("--dir", choices=["fwd", "lat"], default="lat")
    c.add_argument("--cubes", default="1,5,10,20,40,80")
    c.add_argument("--seconds", type=float, default=4.0); c.set_defaults(fn=cmd_compare)
    args = ap.parse_args(); args.fn(args)


if __name__ == "__main__":
    main()
