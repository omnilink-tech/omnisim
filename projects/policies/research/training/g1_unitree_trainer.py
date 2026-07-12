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

"""Unitree-faithful G1 walk trainer (reverse-engineered legged_gym recipe, on OmniSim's mujoco_warp).

GOAL: train a policy that reproduces Unitree's pre_train/g1 motion.pt -- i.e. an
OmniSim-native trainer whose env is byte-faithful to unitree_rl_gym's legged_gym G1
config, so a policy it produces drops straight into the verified g1_unitree_deploy
world and walks.

FAITHFUL to Unitree's recipe (every number extracted from unitree_rl_gym):
  * Model: their EXACT g1_12dof (32.11 kg, 4-sphere feet) -- the same model the
    deploy runs (g1_12dof_scene.xml here == g1_12dof_clean.urdf there). Verified:
    motion.pt walks 18 m on this model.
  * Control: TORQUE-PD, tau = (target-q)*kp - qd*kd, applied to the 12 torque
    motors every physics step -- byte-identical to deploy_mujoco / g1_unitree_deploy.
    per-joint kp=[100,100,100,150,40,40]x2, kd=[2,2,2,4,2,2]x2.
  * Timing: sim 200 Hz (dt 0.005), policy 50 Hz (decimation 4).
  * Default (knees-bent): [-0.1,0,0,0.3,-0.2,0]x2.  action_scale 0.25.
  * Obs (47): ang_vel_body*0.25, proj_gravity, cmd*[2,2,0.25], (q-default),
    qd*0.05, last_action, sin/cos(2pi*phase) (period 0.8s). + their noise model.
  * Reward (16 terms, their exact scales, only_positive_rewards): tracking_lin_vel
    1.0, tracking_ang_vel 0.5, lin_vel_z -2, ang_vel_xy -0.05, orientation -1,
    base_height -10 (0.78), dof_acc -2.5e-7, dof_vel -1e-3, action_rate -0.01,
    dof_pos_limits -5, torques -1e-5, alive 0.15, hip_pos -1, contact_no_vel -0.2,
    feet_swing_height -20 (0.08), contact 0.18 (gait-phase contact match).
  * Termination: roll>0.8 | pitch>1.0 | base too low.  episode 20 s.
  * DR: per-env pushes (~every 5 s, 1.5 m/s), obs noise, RSI; friction/mass per-run
    (per-env model DR is a mujoco_warp refinement -- see _resample_model_dr).
  * PPO (rsl_rl-faithful): 4096 envs x 24 steps, lr 1e-3 adaptive (KL 0.01),
    gamma 0.99, lam 0.95, clip 0.2, ent 0.01, 5 epochs, 4 minibatches.

Runs LOCALLY on the local GPU (the trainer picks cuda:0 when torch sees a CUDA
device, else falls back to CPU -- there is no device flag and no cloud path).
Deps: projects/policies/research/training/requirements-train.txt.

Usage (smoke):  python projects/policies/research/training/g1_unitree_trainer.py --envs 64 --iters 3
Usage (full) :  python projects/policies/research/training/g1_unitree_trainer.py \
                    --envs 4096 --iters 10000 \
                    --save projects/policies/research/training/runs/g1_unitree/policy.pt
"""
import os
import sys
import math
import time
import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())

# ── Unitree G1 legged_gym recipe constants (exact) ──────────────────────────
LEGS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
)
NJ = 12
KP = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], dtype=np.float32)
KD = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], dtype=np.float32)
DEFAULT = np.array([-0.1, 0, 0, 0.3, -0.2, 0, -0.1, 0, 0, 0.3, -0.2, 0], dtype=np.float32)
HIP_LAT_IDX = (1, 2, 7, 8)          # hip roll + hip yaw (anti-splay)
FOOT_BODIES = ("left_ankle_roll_link", "right_ankle_roll_link")

SIM_DT = float(os.environ.get("G1_SIM_DT", "0.005"))      # 200 Hz physics (Unitree training)
DECIMATION = int(os.environ.get("G1_DECIM", "4"))         # -> 50 Hz policy
POLICY_DT = SIM_DT * DECIMATION
ACT_SCALE = 0.25
GAIT_PERIOD = 0.8
PHASE_OFFSET = 0.5
SWING_HEIGHT = 0.08
BASE_HEIGHT_TGT = 0.78
SPAWN_Z = 0.80
TRACKING_SIGMA = 0.25
SOFT_LIMIT = 0.9
EP_LEN_S = 20.0
FALL_Z = 0.50

MODEL = os.environ.get(
    "G1_TRAIN_MJCF",
    str(REPO / "projects/robots/unitree/g1/g1_12dof/g1_12dof_scene.xml"))

# Reward scales (legged_gym G1, multiplied by POLICY_DT in _prepare like rsl_rl).
REWARD = dict(
    tracking_lin_vel=1.0, tracking_ang_vel=0.5,
    lin_vel_z=-2.0, ang_vel_xy=-0.05, orientation=-1.0, base_height=-10.0,
    dof_acc=-2.5e-7, dof_vel=-1e-3, action_rate=-0.01, dof_pos_limits=-5.0,
    torques=-1.0e-5, alive=0.15, hip_pos=-1.0, contact_no_vel=-0.2,
    feet_swing_height=-20.0, contact=0.18,
)
# Observation noise (legged_gym: noise_scales * noise_level * obs_scale)
NOISE = dict(ang_vel=0.2 * 0.25, gravity=0.05, dof_pos=0.01, dof_vel=1.5 * 0.05)

# Command ranges (legged_gym G1)
CMD_VX = (-1.0, 1.0)
CMD_VY = (-1.0, 1.0)
CMD_WZ = (-1.0, 1.0)
RESAMPLE_S = 10.0

# Domain randomization
PUSH_INTERVAL_S = 5.0
PUSH_VEL = 1.5
FRICTION_RANGE = (0.1, 1.25)
ADDED_MASS_RANGE = (-1.0, 3.0)


def quat_to_grav(q):  # q: (...,4) wxyz -> projected gravity in body frame
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return torch.stack([2 * (-z * x + w * y),
                        -2 * (z * y + w * x),
                        1 - 2 * (w * w + z * z)], dim=-1)


def quat_rotate_inv(q, v):  # R^T @ v  (world->body), q wxyz
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    # rotation matrix rows (body->world); inverse = transpose
    r00 = 1 - 2 * (y * y + z * z); r01 = 2 * (x * y - w * z); r02 = 2 * (x * z + w * y)
    r10 = 2 * (x * y + w * z); r11 = 1 - 2 * (x * x + z * z); r12 = 2 * (y * z - w * x)
    r20 = 2 * (x * z - w * y); r21 = 2 * (y * z + w * x); r22 = 1 - 2 * (x * x + y * y)
    vx, vy, vz = v[..., 0], v[..., 1], v[..., 2]
    return torch.stack([r00 * vx + r10 * vy + r20 * vz,
                        r01 * vx + r11 * vy + r21 * vz,
                        r02 * vx + r12 * vy + r22 * vz], dim=-1)


def quat_yaw(q):  # heading (yaw) from quat wxyz
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class G1UnitreeEnv:
    """Vectorised mujoco_warp G1 env, byte-faithful to unitree_rl_gym's legged_gym G1."""

    def __init__(self, n, device="cuda:0", dr=True, seed=0):
        import warp as wp
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mujoco, self.mjw = wp, mujoco, mjw
        self.n = n
        self.dr = dr
        self.device = wp.get_device(device)
        self.td = torch.device("cuda:0" if "cuda" in str(self.device).lower() else "cpu")
        self.max_ep = int(round(EP_LEN_S / POLICY_DT))

        mjm = mujoco.MjModel.from_xml_path(MODEL)
        mjm.opt.timestep = SIM_DT
        # PHYSICS PARITY: Unitree's g1_12dof.xml carries no <option>, so it defaults
        # to the EULER integrator -- stable on mujoco-CPU (the standalone walk) but
        # NOT on mujoco_warp's stiff-PD GPU path (motion.pt falls). The mujoco_warp
        # trainers that DO walk use implicitfast. Match that + give the solver enough
        # iterations for firm foot contact. All env-tunable for parity sweeps.
        _integ = {"euler": mujoco.mjtIntegrator.mjINT_EULER,
                  "rk4": mujoco.mjtIntegrator.mjINT_RK4,
                  "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
                  "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST}
        mjm.opt.integrator = _integ.get(os.environ.get("G1_INTEGRATOR", "implicitfast"),
                                        mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
        # Match the Newton-built deploy model's solver (Newton, 100/50 iters) -- the
        # parity gap is the solver, not the contact params (those are mujoco defaults).
        _solv = {"pgs": mujoco.mjtSolver.mjSOL_PGS, "cg": mujoco.mjtSolver.mjSOL_CG,
                 "newton": mujoco.mjtSolver.mjSOL_NEWTON}
        mjm.opt.solver = _solv.get(os.environ.get("G1_SOLVER", "newton"),
                                   mujoco.mjtSolver.mjSOL_NEWTON)
        mjm.opt.iterations = int(os.environ.get("G1_SOLVER_ITERS", "100"))
        mjm.opt.ls_iterations = int(os.environ.get("G1_LS_ITERS", "50"))
        if os.environ.get("G1_CONTACT_SOLREF"):  # e.g. "0.01 1" -> firmer contact
            sr = [float(x) for x in os.environ["G1_CONTACT_SOLREF"].split()]
            mjm.geom_solref[:, :len(sr)] = sr
        self.mjm = mjm
        self.nq, self.nv, self.nu = mjm.nq, mjm.nv, mjm.nu

        # index maps (controller order -> qpos/qvel/ctrl)
        self.c2q = np.array([mjm.jnt_qposadr[mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)]
                             for j in LEGS], dtype=np.int64)
        self.c2v = np.array([mjm.jnt_dofadr[mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)]
                             for j in LEGS], dtype=np.int64)
        # actuator (torque motor) index per leg joint
        self.c2u = np.array([mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, j)
                             for j in LEGS], dtype=np.int64)
        if (self.c2u < 0).any():
            raise RuntimeError("expected one torque motor named per leg joint in the MJCF")
        fb = [j for j in range(mjm.njnt) if mjm.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE][0]
        self.bq = mjm.jnt_qposadr[fb]   # base qpos start (xyz + quat)
        self.bv = mjm.jnt_dofadr[fb]    # base qvel start (lin + ang)
        self.fb_l = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_BODY, FOOT_BODIES[0])
        self.fb_r = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_BODY, FOOT_BODIES[1])

        # joint soft limits
        lo = np.array([mjm.jnt_range[mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)][0] for j in LEGS])
        hi = np.array([mjm.jnt_range[mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)][1] for j in LEGS])
        mid, half = (lo + hi) * 0.5, (hi - lo) * 0.5 * SOFT_LIMIT
        td = self.td
        self.soft_lo = torch.tensor(mid - half, dtype=torch.float32, device=td)
        self.soft_hi = torch.tensor(mid + half, dtype=torch.float32, device=td)

        mjd = mujoco.MjData(mjm)
        mujoco.mj_forward(mjm, mjd)
        with wp.ScopedDevice(self.device):
            self.mw_m = mjw.put_model(mjm)
            self.mw_d = mjw.put_data(mjm, mjd, nworld=n,
                                     njmax=int(os.environ.get("G1_NJMAX", "256")),
                                     nconmax=int(os.environ.get("G1_NCONMAX", "256")))
        self.qpos = wp.to_torch(self.mw_d.qpos).view(n, self.nq)
        self.qvel = wp.to_torch(self.mw_d.qvel).view(n, self.nv)
        self.ctrl = wp.to_torch(self.mw_d.ctrl).view(n, self.nu)
        self.xpos = wp.to_torch(self.mw_d.xpos).view(n, mjm.nbody, 3)

        self.kp = torch.tensor(KP, device=td)
        self.kd = torch.tensor(KD, device=td)
        self.default = torch.tensor(DEFAULT, device=td)
        self.hip_lat = torch.tensor(HIP_LAT_IDX, dtype=torch.long, device=td)
        self.c2q_t = torch.tensor(self.c2q, dtype=torch.long, device=td)
        self.c2v_t = torch.tensor(self.c2v, dtype=torch.long, device=td)
        self.c2u_t = torch.tensor(self.c2u, dtype=torch.long, device=td)

        # seed pose
        seed_q = mjd.qpos.copy().astype(np.float32)
        seed_q[self.bq:self.bq + 3] = [0, 0, SPAWN_Z]
        seed_q[self.bq + 3:self.bq + 7] = [1, 0, 0, 0]
        for i in range(NJ):
            seed_q[self.c2q[i]] = DEFAULT[i]
        self.seed_q = torch.tensor(seed_q, device=td)

        # episodic state
        self.last_action = torch.zeros(n, NJ, device=td)
        self.prev_qd = torch.zeros(n, NJ, device=td)
        self.prev_foot = torch.zeros(n, 2, 3, device=td)
        self.ep_step = torch.zeros(n, dtype=torch.int32, device=td)
        self.phase = torch.zeros(n, device=td)
        self.cmd = torch.zeros(n, 3, device=td)
        self.push_cd = torch.zeros(n, device=td)
        self.OBS_DIM = 47

        self._cuda_graph = None
        self._resample_model_dr()
        self._try_graph()
        self._reset(torch.arange(n, device=td))

    # ── DR ──
    def _resample_model_dr(self):
        # mujoco_warp shares ONE model across worlds, so friction/mass model DR is
        # per-RUN here (a known mujoco_warp limitation, see project memory). The
        # dominant per-env durability perturbations -- pushes, obs noise, RSI -- ARE
        # per-env below. Per-env model DR is the documented follow-up.
        if not self.dr:
            return
        mu = float(np.random.uniform(*FRICTION_RANGE))
        try:
            self.mw_m.geom_friction.numpy()[:, 0] = mu
        except Exception:
            pass

    def _try_graph(self):
        wp = self.wp
        if os.environ.get("G1_NO_GRAPH") == "1":
            return
        try:
            with wp.ScopedDevice(self.device):
                wp.capture_begin()
                for _ in range(DECIMATION):
                    self.mjw.step(self.mw_m, self.mw_d)
                self._cuda_graph = wp.capture_end()
        except Exception as e:
            print(f"[g1_unitree_trainer] CUDA graph capture failed ({e}); direct step", flush=True)
            self._cuda_graph = None

    def _physics(self):
        wp = self.wp
        if self._cuda_graph is not None:
            with wp.ScopedDevice(self.device):
                wp.capture_launch(self._cuda_graph)
        else:
            for _ in range(DECIMATION):
                self.mjw.step(self.mw_m, self.mw_d)

    def _resample_cmd(self, ids):
        k = ids.numel()
        if k == 0:
            return
        td = self.td
        self.cmd[ids, 0] = torch.empty(k, device=td).uniform_(*CMD_VX)
        self.cmd[ids, 1] = torch.empty(k, device=td).uniform_(*CMD_VY)
        self.cmd[ids, 2] = torch.empty(k, device=td).uniform_(*CMD_WZ)
        # ~stand: small commands zeroed (legged_gym zeros tiny commands)
        small = self.cmd[ids].norm(dim=1) < 0.2
        self.cmd[ids[small]] = 0.0

    def _reset(self, ids):
        if ids.numel() == 0:
            return
        td = self.td
        self.qpos[ids] = self.seed_q
        self.qvel[ids] = 0.0
        if self.dr:  # RSI jitter
            j = (torch.rand(ids.numel(), NJ, device=td) - 0.5) * 0.1
            self.qpos[ids[:, None], self.c2q_t[None, :]] += j
            self.qvel[ids, self.bv:self.bv + 2] += (torch.rand(ids.numel(), 2, device=td) - 0.5) * 0.2
        self.last_action[ids] = 0.0
        self.prev_qd[ids] = 0.0
        self.ep_step[ids] = 0
        # phase starts at 0 (standing); only randomized under DR (RSI). A random
        # start phase desyncs the gait from a standing pose -> startup stumble.
        self.phase[ids] = torch.rand(ids.numel(), device=td) if self.dr else torch.zeros(ids.numel(), device=td)
        self.push_cd[ids] = PUSH_INTERVAL_S
        self._resample_cmd(ids)
        with self.wp.ScopedDevice(self.device):
            self.mjw.forward(self.mw_m, self.mw_d)
        self.prev_foot[ids, 0] = self.xpos[ids, self.fb_l]
        self.prev_foot[ids, 1] = self.xpos[ids, self.fb_r]

    def reset(self):
        self._reset(torch.arange(self.n, device=self.td))
        return self._obs()

    # ── obs ──
    def _q(self):
        return self.qpos[:, self.c2q_t]

    def _qd(self):
        return self.qvel[:, self.c2v_t]

    def _obs(self, noisy=True):
        q = self._q()
        qd = self._qd()
        quat = self.qpos[:, self.bq + 3:self.bq + 7]
        ang_body = self.qvel[:, self.bv + 3:self.bv + 6]   # mujoco free-joint ang vel is body-frame
        grav = quat_to_grav(quat)
        ph = (2 * math.pi * self.phase)
        obs = torch.cat([
            ang_body * 0.25,
            grav,
            self.cmd * torch.tensor([2.0, 2.0, 0.25], device=self.td),
            (q - self.default),
            qd * 0.05,
            self.last_action,
            torch.stack([torch.sin(ph), torch.cos(ph)], dim=-1),
        ], dim=-1)
        if noisy and self.dr:
            nz = torch.zeros(self.OBS_DIM, device=self.td)
            nz[0:3] = NOISE["ang_vel"]; nz[3:6] = NOISE["gravity"]
            nz[9:21] = NOISE["dof_pos"]; nz[21:33] = NOISE["dof_vel"]
            obs = obs + (2 * torch.rand_like(obs) - 1) * nz
        return torch.nan_to_num(obs.clamp(-100, 100), nan=0.0)

    # ── step ──
    def step(self, action):
        td = self.td
        action = action.clamp(-1.0, 1.0)
        target = self.default + ACT_SCALE * action
        # torque PD recomputed EVERY physics substep from the live q,qd (Unitree
        # pd_control) -- held-torque-over-decimation destabilises the PD and breaks
        # even the known-good motion.pt (falls ~0.8 s). Per-substep is correctness-
        # critical; the fast in-graph route is pos/vel actuators (see _GRAPH note).
        for _ in range(DECIMATION):
            q = self._q(); qd = self._qd()
            tau = (target - q) * self.kp - qd * self.kd
            self.ctrl[:, self.c2u_t] = tau
            with self.wp.ScopedDevice(self.device):
                self.mjw.step(self.mw_m, self.mw_d)
        self.ep_step += 1
        self.phase = (self.phase + POLICY_DT / GAIT_PERIOD) % 1.0

        # pushes (per-env, ~every PUSH_INTERVAL_S)
        if self.dr:
            self.push_cd -= POLICY_DT
            due = self.push_cd <= 0
            if due.any():
                ids = due.nonzero(as_tuple=False).squeeze(-1)
                self.qvel[ids, self.bv:self.bv + 2] += (torch.rand(ids.numel(), 2, device=td) - 0.5) * 2 * PUSH_VEL
                self.push_cd[ids] = PUSH_INTERVAL_S

        rew, terminated = self._reward_and_done(action)
        timeout = self.ep_step >= self.max_ep
        done = terminated | timeout
        self.last_action = action
        self.prev_qd = self._qd().clone()
        self.prev_foot[:, 0] = self.xpos[:, self.fb_l]
        self.prev_foot[:, 1] = self.xpos[:, self.fb_r]
        # resample commands on schedule
        res = (self.ep_step % int(RESAMPLE_S / POLICY_DT) == 0)
        if res.any():
            self._resample_cmd(res.nonzero(as_tuple=False).squeeze(-1))
        ids = done.nonzero(as_tuple=False).squeeze(-1)
        self._reset(ids)
        return self._obs(), rew, terminated, timeout, done

    def _reward_and_done(self, action):
        td = self.td
        q = self._q(); qd = self._qd()
        quat = self.qpos[:, self.bq + 3:self.bq + 7]
        lin_world = self.qvel[:, self.bv:self.bv + 3]
        lin_body = quat_rotate_inv(quat, lin_world)
        ang_body = self.qvel[:, self.bv + 3:self.bv + 6]
        grav = quat_to_grav(quat)
        base_z = self.qpos[:, self.bq + 2]

        # foot state
        foot_z = torch.stack([self.xpos[:, self.fb_l, 2], self.xpos[:, self.fb_r, 2]], dim=-1)
        foot_now = torch.stack([self.xpos[:, self.fb_l], self.xpos[:, self.fb_r]], dim=1)
        foot_vel = (foot_now - self.prev_foot) / POLICY_DT
        contact = foot_z < 0.06     # foot-height proxy for ground contact (force>1N)

        # gait phase per leg
        ph_l = self.phase
        ph_r = (self.phase + PHASE_OFFSET) % 1.0
        leg_ph = torch.stack([ph_l, ph_r], dim=-1)
        is_stance = leg_ph < 0.55

        R = REWARD
        r = {}
        r["tracking_lin_vel"] = R["tracking_lin_vel"] * torch.exp(
            -((self.cmd[:, :2] - lin_body[:, :2]) ** 2).sum(-1) / TRACKING_SIGMA)
        r["tracking_ang_vel"] = R["tracking_ang_vel"] * torch.exp(
            -((self.cmd[:, 2] - ang_body[:, 2]) ** 2) / TRACKING_SIGMA)
        r["lin_vel_z"] = R["lin_vel_z"] * lin_world[:, 2] ** 2
        r["ang_vel_xy"] = R["ang_vel_xy"] * (ang_body[:, :2] ** 2).sum(-1)
        r["orientation"] = R["orientation"] * (grav[:, :2] ** 2).sum(-1)
        r["base_height"] = R["base_height"] * (base_z - BASE_HEIGHT_TGT) ** 2
        dof_acc = (self.prev_qd - qd) / POLICY_DT
        r["dof_acc"] = R["dof_acc"] * (dof_acc ** 2).sum(-1)
        r["dof_vel"] = R["dof_vel"] * (qd ** 2).sum(-1)
        r["action_rate"] = R["action_rate"] * ((action - self.last_action) ** 2).sum(-1)
        over = (q - self.soft_hi).clamp(min=0) + (self.soft_lo - q).clamp(min=0)
        r["dof_pos_limits"] = R["dof_pos_limits"] * over.sum(-1)
        tau = (self.default + ACT_SCALE * action - q) * self.kp - qd * self.kd
        r["torques"] = R["torques"] * (tau ** 2).sum(-1)
        r["alive"] = R["alive"] * torch.ones(self.n, device=td)
        r["hip_pos"] = R["hip_pos"] * (q[:, self.hip_lat] ** 2).sum(-1)
        r["contact_no_vel"] = R["contact_no_vel"] * (
            (foot_vel * contact.unsqueeze(-1)) ** 2).sum(dim=(1, 2))
        r["feet_swing_height"] = R["feet_swing_height"] * (
            ((foot_z - SWING_HEIGHT) ** 2) * (~contact)).sum(-1)
        r["contact"] = R["contact"] * (contact == is_stance).float().sum(-1)

        total = sum(v for v in r.values()) * POLICY_DT
        total = total.clamp(min=0.0)   # only_positive_rewards

        roll = torch.atan2(2 * (quat[:, 0] * quat[:, 1] + quat[:, 2] * quat[:, 3]),
                           1 - 2 * (quat[:, 1] ** 2 + quat[:, 2] ** 2))
        pitch = torch.asin((2 * (quat[:, 0] * quat[:, 2] - quat[:, 3] * quat[:, 1])).clamp(-1, 1))
        terminated = (roll.abs() > 0.8) | (pitch.abs() > 1.0) | (base_z < FALL_Z)
        return total, terminated


# ── Actor-Critic (MLP) ──────────────────────────────────────────────────────
class ActorCritic(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden=(256, 128), init_std=0.8):
        super().__init__()
        def mlp(o):
            layers, d = [], obs_dim
            for h in hidden:
                layers += [nn.Linear(d, h), nn.ELU()]; d = h
            layers += [nn.Linear(d, o)]
            return nn.Sequential(*layers)
        self.actor = mlp(act_dim)
        self.critic = mlp(1)
        self.log_std = nn.Parameter(np.log(init_std) * torch.ones(act_dim))

    def act(self, obs):
        mean = self.actor(obs)
        std = self.log_std.exp()
        dist = torch.distributions.Normal(mean, std)
        a = dist.sample()
        return a, dist.log_prob(a).sum(-1), self.critic(obs).squeeze(-1)

    def evaluate(self, obs, act):
        mean = self.actor(obs)
        dist = torch.distributions.Normal(mean, self.log_std.exp())
        return dist.log_prob(act).sum(-1), dist.entropy().sum(-1), self.critic(obs).squeeze(-1)


def train(args):
    dev = "cuda:0" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(args.seed); np.random.seed(args.seed)
    env = G1UnitreeEnv(args.envs, device=dev, dr=not args.no_dr, seed=args.seed)
    td = env.td
    ac = ActorCritic(env.OBS_DIM, NJ, init_std=args.init_std).to(td)
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)
    lr = args.lr
    obs = env.reset()
    T, B = args.steps, args.envs
    print(f"[g1_unitree_trainer] envs={B} steps={T} obs={env.OBS_DIM} act={NJ} dev={td} dr={not args.no_dr}", flush=True)

    for it in range(args.iters):
        t0 = time.time()
        o_buf = torch.zeros(T, B, env.OBS_DIM, device=td)
        a_buf = torch.zeros(T, B, NJ, device=td)
        lp_buf = torch.zeros(T, B, device=td)
        v_buf = torch.zeros(T, B, device=td)
        r_buf = torch.zeros(T, B, device=td)
        d_buf = torch.zeros(T, B, device=td)
        to_buf = torch.zeros(T, B, device=td)
        with torch.no_grad():
            for t in range(T):
                a, lp, v = ac.act(obs)
                o_buf[t] = obs; a_buf[t] = a; lp_buf[t] = lp; v_buf[t] = v
                obs, rew, term, timeout, done = env.step(a)
                r_buf[t] = rew; d_buf[t] = done.float(); to_buf[t] = timeout.float()
            _, _, last_v = ac.act(obs)

        # GAE with timeout value-bootstrap (rsl_rl style)
        adv = torch.zeros(T, B, device=td)
        gae = torch.zeros(B, device=td)
        for t in reversed(range(T)):
            nv = last_v if t == T - 1 else v_buf[t + 1]
            r_t = r_buf[t] + args.gamma * nv * to_buf[t]   # bootstrap clean timeouts
            nonterm = 1.0 - d_buf[t]
            delta = r_t + args.gamma * nv * nonterm - v_buf[t]
            gae = delta + args.gamma * args.lam * nonterm * gae
            adv[t] = gae
        ret = adv + v_buf
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)

        of = o_buf.reshape(-1, env.OBS_DIM); af = a_buf.reshape(-1, NJ)
        lpf = lp_buf.reshape(-1); advf = adv.reshape(-1); retf = ret.reshape(-1)
        N = of.shape[0]; mb = N // args.minibatches
        approx_kl = 0.0
        for _ in range(args.epochs):
            idx = torch.randperm(N, device=td)
            for s in range(0, N, mb):
                j = idx[s:s + mb]
                nlp, ent, v = ac.evaluate(of[j], af[j])
                ratio = (nlp - lpf[j]).exp()
                pl = -torch.min(ratio * advf[j], ratio.clamp(1 - args.clip, 1 + args.clip) * advf[j]).mean()
                vl = (v - retf[j]).pow(2).mean()
                loss = pl + args.vcoef * vl - args.entcoef * ent.mean()
                opt.zero_grad(); loss.backward()
                nn.utils.clip_grad_norm_(ac.parameters(), 1.0)
                opt.step()
                with torch.no_grad():
                    approx_kl = ((ratio - 1) - (nlp - lpf[j])).mean().item()
        # KL-adaptive LR
        if approx_kl > 2 * args.kl:
            lr = max(1e-5, lr / 1.5)
        elif 0 < approx_kl < 0.5 * args.kl:
            lr = min(1e-2, lr * 1.5)
        for g in opt.param_groups:
            g["lr"] = lr

        fps = (T * B) / (time.time() - t0)
        if it % args.log_every == 0 or it == args.iters - 1:
            print(f"it {it:5d} | rew/step {r_buf.mean().item():.3f} | ret {retf.mean().item():.2f} "
                  f"| kl {approx_kl:.4f} | lr {lr:.1e} | {fps:,.0f} steps/s", flush=True)
        if args.save and (it % args.save_every == 0 or it == args.iters - 1):
            _save(ac, env, args.save)
    if args.save:
        _save(ac, env, args.save)
        print(f"[g1_unitree_trainer] saved {args.save}", flush=True)


class _Pol(nn.Module):
    """Deploy head: actor MLP -> clamp [-1,1] (the deploy contract)."""
    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, obs):
        return self.actor(obs).clamp(-1.0, 1.0)


def _save(ac, env, path):
    import copy
    # 1) state_dict for resume/inspection
    torch.save(ac.state_dict(), str(Path(path).with_suffix(".state.pt")))
    # 2) THE deploy artifact: a CPU torch.jit module the EXISTING g1_unitree_deploy
    #    loads unchanged (torch.jit.load -> policy(obs[1,47]) -> clamped [1,12]).
    #    Export a CPU COPY so the training actor (on GPU) is untouched + the deploy
    #    (which runs the policy on CPU) can load it.
    pol = _Pol(copy.deepcopy(ac.actor)).cpu().eval()
    dummy = torch.zeros(1, env.OBS_DIM)
    with torch.no_grad():
        ts = torch.jit.trace(pol, dummy)
    torch.jit.save(ts, path)
    # 3) ONNX (best-effort; deploy uses the jit .pt)
    try:
        torch.onnx.export(pol, dummy, str(Path(path).with_suffix(".onnx")),
                          input_names=["obs"], output_names=["action"], opset_version=18,
                          dynamic_axes={"obs": {0: "b"}, "action": {0: "b"}})
    except Exception as e:
        print(f"[g1_unitree_trainer] ONNX export skipped: {e}", flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=4096)
    p.add_argument("--steps", type=int, default=24)
    p.add_argument("--iters", type=int, default=10000)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--lam", type=float, default=0.95)
    p.add_argument("--clip", type=float, default=0.2)
    p.add_argument("--entcoef", type=float, default=0.01)
    p.add_argument("--vcoef", type=float, default=1.0)
    p.add_argument("--epochs", type=int, default=5)
    p.add_argument("--minibatches", type=int, default=4)
    p.add_argument("--kl", type=float, default=0.01)
    p.add_argument("--init-std", type=float, default=0.8)
    p.add_argument("--no-dr", action="store_true")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--save", type=str, default="")
    p.add_argument("--save-every", type=int, default=500)
    p.add_argument("--log-every", type=int, default=10)
    train(p.parse_args())


if __name__ == "__main__":
    main()
