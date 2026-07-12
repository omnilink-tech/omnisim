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

"""GPU mujoco_warp trainer for the G1 SEATED arm-mimic + dynamic balance policy.

Phase-3 of the seated arm-mimic effort (docs/developer/g1-sit-mimic-plan.md):
the deterministic demo pins the pelvis. This trainer instead learns a policy that
keeps the G1 dynamically balanced SEATED ON A CHAIR (free root, no pin) while its
arms follow the ghost's wave.

Design (a focused adaptation of the canonical gpu_mjwarp_g1_stand_trainer.py):
  - The 13 leg+waist joints are the BALANCE policy (residual on the seated pose),
    exactly the stand trainer's action/obs surface (NJ=13, OBS_DIM=48) so the
    deploy obs-builder is reused.
  - The 10 ARM joints are driven OPEN-LOOP by the ghost reference wave
    (projects/policies/control/gait/g1_sit_gesture, the SAME motion the deterministic demo and
    the ghost play). The moving arms are the disturbance the leg policy must
    absorb -> "follow the ghost WHILE dynamically balancing".
  - The robot sits on a chair: g1_sit.mjcf.xml = g1_full.mjcf.xml + a static
    chair-seat box collider at the deploy seat height. The robot rests butt/thighs
    on the seat + feet on the floor (a wide, statically-stable support); the
    policy holds it upright under the arm reaction + DR pushes.

Reward = alive + upright - base-velocity - seat-drift - action regularization.
Terminate on fall (pelvis too low / torso tilted past threshold).

Usage:
    python projects/policies/research/training/gpu_mjwarp_g1_sit_trainer.py \\
        --envs 4096 --iters 400 --rollout 16 \\
        --save projects/policies/research/training/runs/gpu_g1_sit/policy.pt

Exports policy.pt + policy.onnx for a seated deploy controller (legs = NOMINAL +
res-scale * policy(obs); arms = ghost wave; G1_SIT_PIN=0).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))

# Shared seated reference (single source of truth with the deterministic demo +
# the ghost). We rebuild the leg NOMINAL and the arm wave from it.
from projects.policies.control.gait.g1_sit_gesture import SEATED_POSE, WAVE_HZ, RAISE_S

# ────────────────────────────────────────────────────────────────────
# Layout — mirrors gpu_mjwarp_g1_stand_trainer.py (deploy-compatible).
# ────────────────────────────────────────────────────────────────────
NJ = 13            # 6 left + 6 right leg joints + 1 waist (the balance policy)
# 48 base + 20 ARM FEEDFORWARD: the 10 arm reference angles NOW + 10 a short
# lookahead ahead, so the policy ANTICIPATES the arm-wave reaction torque and
# pre-counters it (instead of reacting late and rocking). MUST match the deploy
# obs builder in g1_seated_mimic.
ARM_LOOKAHEAD = 0.12   # s ahead for the feedforward snapshot
OBS_DIM = 68       # 48 + arm_now(10) + arm_ahead(10)
RES_SCALE = 0.3    # ±0.3 rad residual on the seated leg pose
DT = 0.016         # env-step dt = OmniSim basicTimeStep 16 ms
SUBSTEPS = 4
PHYS_DT = 0.004

LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)

# Seated leg+waist nominal, pulled from the shared SEATED_POSE so it can never
# drift from the demo/ghost.
NOMINAL = np.array([SEATED_POSE[j] for j in LEGS_JOINTS], dtype=np.float32)
assert NOMINAL.shape == (NJ,)

# Seated spawn: pelvis ~0.47 (butt on the ~0.33 chair seat, feet near the floor).
SPAWN_Z = 0.47
# Fall thresholds: pelvis dropping well below the settled seated height (~0.44)
# means it slid/tipped off the seat; large tilt = toppling.
BZ_FAIL = 0.28
ROLL_FAIL = 0.9
PITCH_FAIL = 0.9
MAX_EP = 500

# Joint position limits (URDF <limit>), legs+waist order (same as stand trainer).
JOINT_LIMITS_LO = np.array([
    -2.531, -0.524, -2.758, -0.087, -0.873, -0.262,
    -2.531, -2.967, -2.758, -0.087, -0.873, -0.262,
    -2.618,
], dtype=np.float32)
JOINT_LIMITS_HI = np.array([
    +2.880, +2.967, +2.758, +2.880, +0.524, +0.262,
    +2.880, +0.524, +2.758, +2.880, +0.524, +0.262,
    +2.618,
], dtype=np.float32)

# The 10 arm joints, driven OPEN-LOOP by the ghost wave (never policy-controlled).
ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
)
# Static seated base for the arms (the SEATED_POSE values); the wave adds on top.
ARM_BASE = np.array([SEATED_POSE[j] for j in ARM_JOINTS], dtype=np.float32)


def arm_targets_torch(t, device):
    """Vectorized ghost arm wave -> (n, 10) targets in ARM_JOINTS order.

    MUST stay in sync with projects/policies/control/gait/g1_sit_gesture.arm_gesture(): right
    arm raises out and waves (elbow + wrist), left arm does a slow lift. `t` is a
    (n,) tensor of per-env sim time (s)."""
    w = 2.0 * math.pi * WAVE_HZ
    x = torch.clamp(t / RAISE_S, 0.0, 1.0)
    r = x * x * (3.0 - 2.0 * x)                       # smoothstep raise-in
    s = torch.sin(w * t)
    lift = (1.0 - torch.cos(2.0 * math.pi * t / 6.0)) * 0.5
    base = torch.tensor(ARM_BASE, dtype=torch.float32, device=device)
    out = base.unsqueeze(0).repeat(t.shape[0], 1)    # (n, 10)
    # indices into ARM_JOINTS
    out[:, 1] = 0.18 + 0.70 * lift                   # left_shoulder_roll
    out[:, 3] = 0.30 + 0.55 * lift                   # left_elbow
    out[:, 5] = -0.25 * r                            # right_shoulder_pitch
    out[:, 6] = -1.30 * r                            # right_shoulder_roll
    out[:, 8] = (1.05 + 0.50 * s) * r                # right_elbow
    out[:, 9] = 0.55 * s * r                         # right_wrist_roll
    return out


def proj_gravity_np(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    gx = -2 * (x * z - w * y)
    gy = -2 * (y * z + w * x)
    gz = -(1 - 2 * (x * x + y * y))
    return np.stack([gx, gy, gz], axis=1).astype(np.float32)


# ────────────────────────────────────────────────────────────────────
# Batched seated env — N parallel G1-on-a-chair instances on GPU.
# ────────────────────────────────────────────────────────────────────
class BatchedG1SitEnv:
    def __init__(self, n, mjcf, device="cuda:0", reward_cfg=None, sim_dt=0.0,
                 dr_cfg=None, wave=True):
        import warp as wp
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mjw = wp, mjw
        self.n = n
        self.wave = wave
        self.device = wp.get_device(device)
        self.dr = dr_cfg or {}

        self.mjm = mujoco.MjModel.from_xml_path(mjcf)
        self.mjm.opt.timestep = float(sim_dt) if sim_dt and sim_dt > 0 else PHYS_DT

        # Per-run domain randomization on the model (same scheme as stand trainer).
        rng = np.random.default_rng(self.dr.get("seed", 0))
        if self.dr.get("mass_scale", 0.0) > 0:
            b = self.dr["mass_scale"]
            sc = rng.uniform(1 - b, 1 + b, size=self.mjm.body_mass.shape).astype(np.float32)
            self.mjm.body_mass[:] *= sc
            self.mjm.body_inertia[:] *= sc[:, None]
        if self.dr.get("friction_scale", 0.0) > 0:
            b = self.dr["friction_scale"]
            self.mjm.geom_friction[:, 0] *= float(rng.uniform(1 - b, 1 + b))
        if self.dr.get("damping_scale", 0.0) > 0:
            b = self.dr["damping_scale"]
            ds = rng.uniform(1 - b, 1 + b, size=self.mjm.dof_damping.shape).astype(np.float32)
            self.mjm.dof_damping[:] *= ds
        akp = self.dr.get("actuator_kp_scale", 0.0)
        akv = self.dr.get("actuator_kv_scale", 0.0)
        if akp > 0 or akv > 0:
            for ai in range(self.mjm.nu):
                gp = self.mjm.actuator_gainprm[ai]; bp = self.mjm.actuator_biasprm[ai]
                if abs(bp[1]) > 1e-6 and akp > 0:
                    s = float(rng.uniform(1 - akp, 1 + akp)); gp[0] *= s; bp[1] *= s
                elif abs(bp[2]) > 1e-6 and akv > 0:
                    s = float(rng.uniform(1 - akv, 1 + akv)); gp[0] *= s; bp[2] *= s
        if self.dr.get("gravity_scale", 0.0) > 0:
            b = self.dr["gravity_scale"]
            self.mjm.opt.gravity[2] *= float(rng.uniform(1 - b, 1 + b))

        mjd = mujoco.MjData(self.mjm)
        mujoco.mj_forward(self.mjm, mjd)
        with wp.ScopedDevice(self.device):
            self.mw_m = mjw.put_model(self.mjm)
            import os as _njos
            _njmax = int(_njos.environ.get("G1_NJMAX", "256"))
            _nconmax = int(_njos.environ.get("G1_NCONMAX", "256"))
            self.mw_d = mjw.put_data(self.mjm, mjd, nworld=n, njmax=_njmax, nconmax=_nconmax)

        # controller-order -> mjcf indices for legs+waist
        self.c2qpos = np.zeros(NJ, dtype=np.int32)
        self.c2qvel = np.zeros(NJ, dtype=np.int32)
        self.c2ctrlpos = np.zeros(NJ, dtype=np.int32)
        for i, jn in enumerate(LEGS_JOINTS):
            jid = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                raise RuntimeError(f"joint {jn} not in MJCF")
            self.c2qpos[i] = self.mjm.jnt_qposadr[jid]
            self.c2qvel[i] = self.mjm.jnt_dofadr[jid]
            pid = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_pos")
            if pid < 0:
                raise RuntimeError(f"actuator {jn}_pos not in MJCF")
            self.c2ctrlpos[i] = pid
        # arm joints -> qpos + ctrl-pos
        self.arm_qpos = np.zeros(len(ARM_JOINTS), dtype=np.int32)
        self.arm_ctrlpos = np.zeros(len(ARM_JOINTS), dtype=np.int32)
        for i, jn in enumerate(ARM_JOINTS):
            jid = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                raise RuntimeError(f"arm joint {jn} not in MJCF (use g1_sit.mjcf.xml)")
            self.arm_qpos[i] = self.mjm.jnt_qposadr[jid]
            pid = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_pos")
            if pid < 0:
                raise RuntimeError(f"actuator {jn}_pos not in MJCF")
            self.arm_ctrlpos[i] = pid

        self.r = reward_cfg or {}
        self.nq, self.nv, self.nu = self.mjm.nq, self.mjm.nv, self.mjm.nu

        # Seed pose: seated, at SPAWN_Z, identity orientation.
        self.seed_qpos = mjd.qpos.copy().astype(np.float32)
        self.seed_qpos[0:3] = [0.0, 0.0, SPAWN_Z]
        self.seed_qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for i, jn in enumerate(LEGS_JOINTS):
            self.seed_qpos[self.c2qpos[i]] = NOMINAL[i]
        for i in range(len(ARM_JOINTS)):
            self.seed_qpos[self.arm_qpos[i]] = ARM_BASE[i]

        # GPU views + constants
        self.tdev = torch.device("cuda:0" if "cuda" in str(self.device).lower() else "cpu")
        self.qpos_t = wp.to_torch(self.mw_d.qpos).view(n, self.nq)
        self.qvel_t = wp.to_torch(self.mw_d.qvel).view(n, self.nv)
        self.ctrl_t = wp.to_torch(self.mw_d.ctrl).view(n, self.nu)
        d = self.tdev
        self.nominal_t = torch.tensor(NOMINAL, device=d)
        self.jl_lo_t = torch.tensor(JOINT_LIMITS_LO, device=d)
        self.jl_hi_t = torch.tensor(JOINT_LIMITS_HI, device=d)
        self.qpos_idx_t = torch.tensor(self.c2qpos, dtype=torch.long, device=d)
        self.qvel_idx_t = torch.tensor(self.c2qvel, dtype=torch.long, device=d)
        self.ctrl_pos_idx_t = torch.tensor(self.c2ctrlpos, dtype=torch.long, device=d)
        self.arm_ctrl_pos_idx_t = torch.tensor(self.arm_ctrlpos, dtype=torch.long, device=d)
        self.arm_base_t = torch.tensor(ARM_BASE, device=d).unsqueeze(0).expand(n, -1).contiguous()
        self.seed_qpos_t = torch.tensor(self.seed_qpos, device=d)

        self.ep_step_t = torch.zeros(n, dtype=torch.int32, device=d)
        # Per-env WAVE phase offset (seconds), separate from ep_step so the
        # episode runs its full length while envs still see all wave phases.
        self.phase_off_t = torch.zeros(n, device=d)
        self.last_action_t = torch.zeros(n, NJ, device=d)

        self._push_p = float(self.dr.get("push_prob", 0.0))
        self._push_vmax = float(self.dr.get("push_vmax", 0.0))
        self._obs_noise = float(self.dr.get("obs_noise", 0.0))
        self._init_q_band = float(self.dr.get("init_q_band", 0.05))
        self._init_tilt = float(self.dr.get("init_tilt_band", 0.0))
        self._init_vel = float(self.dr.get("init_vel_band", 0.0))

        self._cuda_graph = None
        self._try_capture_graph()
        self._reset_all()

    def _try_capture_graph(self):
        import os as _os
        if _os.environ.get("OMNISIM_NEWTON_NO_GRAPH"):
            return
        try:
            with self.wp.ScopedDevice(self.device):
                for _ in range(SUBSTEPS):
                    self.mjw.step(self.mw_m, self.mw_d)
                self.wp.synchronize()
                self.wp.capture_begin(force_module_load=False)
                for _ in range(SUBSTEPS):
                    self.mjw.step(self.mw_m, self.mw_d)
                self._cuda_graph = self.wp.capture_end()
            print(f"[env] CUDA graph captured ({SUBSTEPS} substeps)")
        except Exception as e:
            print(f"[env] CUDA graph capture failed ({e}); direct step")
            self._cuda_graph = None

    def _reset_envs(self, env_mask=None):
        if env_mask is None:
            idx = torch.arange(self.n, device=self.tdev)
        else:
            idx = torch.nonzero(env_mask, as_tuple=False).squeeze(-1)
            if idx.numel() == 0:
                return
        m = idx.shape[0]
        self.qpos_t[idx] = self.seed_qpos_t.unsqueeze(0).expand(m, -1)
        self.qvel_t[idx] = 0.0
        if self._init_q_band > 0:
            jit = (torch.rand(m, NJ, device=self.tdev) * 2 - 1) * self._init_q_band
            bi = idx.unsqueeze(1).expand(-1, NJ)
            ci = self.qpos_idx_t.unsqueeze(0).expand(m, -1)
            self.qpos_t[bi, ci] += jit
        if self._init_tilt > 0:
            hr = (torch.rand(m, device=self.tdev) * 2 - 1) * (self._init_tilt * 0.5)
            hp = (torch.rand(m, device=self.tdev) * 2 - 1) * (self._init_tilt * 0.5)
            cr, sr, cp, sp = torch.cos(hr), torch.sin(hr), torch.cos(hp), torch.sin(hp)
            self.qpos_t[idx, 3] = cr * cp
            self.qpos_t[idx, 4] = sr * cp
            self.qpos_t[idx, 5] = cr * sp
            self.qpos_t[idx, 6] = sr * sp
        if self._init_vel > 0:
            self.qvel_t[idx, 0:6] += (torch.rand(m, 6, device=self.tdev) * 2 - 1) * self._init_vel
        # Episode counter from 0 (so MAX_EP gives a full-length episode); the
        # wave gets its OWN random phase offset for coverage.
        self.ep_step_t[idx] = 0
        if self.wave:
            self.phase_off_t[idx] = torch.randint(
                0, MAX_EP, (m,), device=self.tdev).to(torch.float32) * DT
        self.last_action_t[idx] = 0.0
        with self.wp.ScopedDevice(self.device):
            self.mjw.forward(self.mw_m, self.mw_d)

    def _reset_all(self):
        self._reset_envs(None)

    def _build_obs_t(self):
        qp, qv = self.qpos_t, self.qvel_t
        vlin, vang = qv[:, 0:3], qv[:, 3:6]
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        gx = -2 * (x * z - w * y)
        gy = -2 * (y * z + w * x)
        gz = -(1 - 2 * (x * x + y * y))
        pg = torch.stack([gx, gy, gz], dim=1)
        q = qp.index_select(1, self.qpos_idx_t) - self.nominal_t.unsqueeze(0)
        qd = qv.index_select(1, self.qvel_idx_t)
        # Arm feedforward: the arm reference NOW + a short lookahead, so the policy
        # can anticipate the wave's reaction torque.
        if self.wave:
            t = self.ep_step_t.to(torch.float32) * DT + self.phase_off_t
            arm_now = arm_targets_torch(t, self.tdev)
            arm_ahead = arm_targets_torch(t + ARM_LOOKAHEAD, self.tdev)
        else:
            arm_now = self.arm_base_t
            arm_ahead = self.arm_base_t
        obs = torch.cat([vlin, vang, pg, q, qd, self.last_action_t,
                         arm_now, arm_ahead], dim=1)
        if self._obs_noise > 0:
            obs = obs + torch.randn_like(obs) * self._obs_noise
        obs = torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        return torch.clamp(obs, -10.0, 10.0)

    def reset(self):
        self._reset_all()
        return self._build_obs_t()

    def step(self, action_t):
        action_t = torch.clamp(action_t, -1.0, 1.0)
        # legs+waist = seated NOMINAL + residual
        targets = torch.clamp(self.nominal_t.unsqueeze(0) + RES_SCALE * action_t,
                              self.jl_lo_t, self.jl_hi_t)
        self.ctrl_t.zero_()
        self.ctrl_t.index_copy_(1, self.ctrl_pos_idx_t, targets)
        # arms = ghost wave (open-loop), driven from each env's sim time
        # (ep_step + its random phase offset)
        if self.wave:
            t = self.ep_step_t.to(torch.float32) * DT + self.phase_off_t
            arm_tg = arm_targets_torch(t, self.tdev)
        else:
            arm_tg = self.arm_base_t
        self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, arm_tg)

        if self._push_p > 0 and self._push_vmax > 0:
            hit = torch.rand(self.n, device=self.tdev) < self._push_p
            if hit.any():
                th = torch.rand(self.n, device=self.tdev) * (2 * math.pi)
                mag = torch.rand(self.n, device=self.tdev) * self._push_vmax
                self.qvel_t[hit, 0] += (torch.cos(th) * mag)[hit]
                self.qvel_t[hit, 1] += (torch.sin(th) * mag)[hit]

        with self.wp.ScopedDevice(self.device):
            if self._cuda_graph is not None:
                self.wp.capture_launch(self._cuda_graph)
            else:
                for _ in range(SUBSTEPS):
                    self.mjw.step(self.mw_m, self.mw_d)

        self.ep_step_t = self.ep_step_t + 1
        prev_action = self.last_action_t
        self.last_action_t = action_t

        bz = self.qpos_t[:, 2]
        w, x, y, z = self.qpos_t[:, 3], self.qpos_t[:, 4], self.qpos_t[:, 5], self.qpos_t[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1.0, 1.0))
        vlin_n = torch.linalg.norm(self.qvel_t[:, 0:3], dim=1)
        vang_n = torch.linalg.norm(self.qvel_t[:, 3:6], dim=1)
        # base xy drift from the seat centre (stay ON the chair)
        xy_drift = torch.linalg.norm(self.qpos_t[:, 0:2], dim=1)

        r = self.r
        ones = torch.ones(self.n, device=self.tdev)
        # ── IMITATION reward: the robot must sit ALMOST IDENTICAL to the ghost,
        # not merely stay up. The legs+waist should match the reference seated
        # pose (NOMINAL); the pelvis should sit at the reference height; the
        # torso upright. The policy's residual is then only the *minimum*
        # correction needed to balance, so the deployed pose tracks the ghost. ──
        qleg_dev = self.qpos_t.index_select(1, self.qpos_idx_t) - self.nominal_t.unsqueeze(0)
        pose_err = (qleg_dev * qleg_dev).mean(dim=1)            # mean-sq leg/waist dev
        tilt2 = roll * roll + pitch * pitch                    # squared torso tilt
        # SIT STRAIGHT is the dominant objective: a SHARP exp penalty on any
        # torso tilt (the old 1-roll^2-pitch^2 was ~0.85 even at 22 deg, so the
        # policy happily slouched). Plus pelvis at the ghost height/centred.
        # Pose-match (legs->reference) is a regularizer so the legs stay close
        # to the ghost where balance allows.
        r_up = r.get("upright", 1.2) * torch.exp(-r.get("upright_k", 12.0) * tilt2)
        r_base = r.get("base", 0.8) * torch.exp(
            -r.get("base_k", 40.0) * ((bz - SPAWN_Z) ** 2 + xy_drift ** 2))
        r_pose = r.get("pose", 0.5) * torch.exp(-r.get("pose_k", 6.0) * pose_err)
        r_alive = r.get("alive", 0.2) * ones
        r_lin = r.get("lin", -0.05) * vlin_n
        r_ang = r.get("ang", -0.02) * vang_n
        r_act = r.get("act", -0.005) * (action_t * action_t).sum(dim=1)
        r_rate = r.get("act_rate", -0.02) * ((action_t - prev_action) ** 2).sum(dim=1)
        reward = r_alive + r_up + r_base + r_pose + r_lin + r_ang + r_act + r_rate

        fall = (torch.abs(roll) > ROLL_FAIL) | (torch.abs(pitch) > PITCH_FAIL) | (bz < BZ_FAIL)
        done = fall | (self.ep_step_t >= MAX_EP)
        reward = reward + fall.float() * r.get("term", -1.0)
        if done.any():
            self._reset_envs(done)
        return self._build_obs_t(), reward, done, {"bz": bz, "roll": roll, "pitch": pitch}


# ────────────────────────────────────────────────────────────────────
# PPO loop (same shape as the stand trainer).
# ────────────────────────────────────────────────────────────────────
def main():
    import torch.nn as nn

    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=4096)
    p.add_argument("--iters", type=int, default=400)
    p.add_argument("--rollout", type=int, default=16)
    p.add_argument("--mjcf", default=str(REPO / "projects/robots/unitree/g1/urdf/g1_sit.mjcf.xml"))
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save", default=str(REPO / "projects/policies/research/training/runs/gpu_g1_sit/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=512)
    p.add_argument("--no-wave", action="store_true",
                   help="hold the arms at the static seated pose (debug: balance "
                        "without the wave disturbance)")
    # Imitation reward (sit STRAIGHT + ALMOST IDENTICAL to the ghost).
    p.add_argument("--upright", type=float, default=1.2,
                   help="weight: SIT STRAIGHT (dominant) -- sharp exp on torso tilt")
    p.add_argument("--upright-k", type=float, default=12.0)
    p.add_argument("--base", type=float, default=0.8,
                   help="weight: pelvis at the reference height + centred")
    p.add_argument("--base-k", type=float, default=40.0)
    p.add_argument("--pose", type=float, default=0.5,
                   help="weight: legs+waist near the reference seated pose (regularizer)")
    p.add_argument("--pose-k", type=float, default=6.0)
    p.add_argument("--alive", type=float, default=0.2)
    p.add_argument("--lin", type=float, default=-0.05)
    p.add_argument("--ang", type=float, default=-0.15,
                   help="penalty on torso angular velocity -- DAMPS the arm-wave wobble")
    p.add_argument("--act", type=float, default=-0.005)
    p.add_argument("--act-rate", type=float, default=-0.04)
    p.add_argument("--term", type=float, default=-1.0)
    p.add_argument("--sim-dt", type=float, default=0.0)
    p.add_argument("--init-from", default=None)
    p.add_argument("--dr-mass-scale", type=float, default=0.20)
    p.add_argument("--dr-friction-scale", type=float, default=0.40)
    p.add_argument("--dr-damping-scale", type=float, default=0.40)
    p.add_argument("--dr-actuator-kp-scale", type=float, default=0.30)
    p.add_argument("--dr-actuator-kv-scale", type=float, default=0.30)
    p.add_argument("--dr-gravity-scale", type=float, default=0.05)
    p.add_argument("--dr-push-prob", type=float, default=0.03,
                   help="per-step probability of a base push (the balance disturbance)")
    p.add_argument("--dr-push-vmax", type=float, default=0.6)
    p.add_argument("--dr-obs-noise", type=float, default=0.02)
    p.add_argument("--dr-init-q-band", type=float, default=0.08)
    p.add_argument("--dr-init-tilt-band", type=float, default=0.12)
    p.add_argument("--dr-init-vel-band", type=float, default=0.15)
    p.add_argument("--dr-seed", type=int, default=0)
    p.add_argument("--no-dr", action="store_true")
    args = p.parse_args()

    if not Path(args.mjcf).exists():
        raise SystemExit(f"MJCF not found: {args.mjcf}")

    reward_cfg = dict(upright=args.upright, upright_k=args.upright_k,
                      base=args.base, base_k=args.base_k, pose=args.pose,
                      pose_k=args.pose_k, alive=args.alive, lin=args.lin,
                      ang=args.ang, act=args.act, act_rate=args.act_rate,
                      term=args.term)
    if args.no_dr:
        dr_cfg = {}
    else:
        dr_cfg = dict(
            mass_scale=args.dr_mass_scale, friction_scale=args.dr_friction_scale,
            damping_scale=args.dr_damping_scale, actuator_kp_scale=args.dr_actuator_kp_scale,
            actuator_kv_scale=args.dr_actuator_kv_scale, gravity_scale=args.dr_gravity_scale,
            push_prob=args.dr_push_prob, push_vmax=args.dr_push_vmax,
            obs_noise=args.dr_obs_noise, init_q_band=args.dr_init_q_band,
            init_tilt_band=args.dr_init_tilt_band, init_vel_band=args.dr_init_vel_band,
            seed=args.dr_seed)
        print(f"[DR] {dr_cfg}")

    env = BatchedG1SitEnv(args.envs, args.mjcf, reward_cfg=reward_cfg,
                          sim_dt=args.sim_dt, dr_cfg=dr_cfg, wave=not args.no_wave)
    print(f"[sit] wave={'on' if not args.no_wave else 'OFF'}  NOMINAL={NOMINAL.tolist()}")
    N = args.envs
    tdev = env.tdev

    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, NJ))
            self.v = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                   nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, 1))
            self.log_std = nn.Parameter(-1.0 * torch.ones(NJ))

        def forward(self, obs):
            return self.pi(obs), self.v(obs).squeeze(-1), self.log_std

    torch.manual_seed(0)
    ac = AC().to(tdev)
    if args.init_from and Path(args.init_from).exists():
        ac.load_state_dict(torch.load(args.init_from, map_location=tdev))
        print(f"warm-start from {args.init_from}")
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)

    if args.eval:
        ac.load_state_dict(torch.load(args.save, map_location=tdev))
        obs = env.reset()
        survived = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        pose_acc = 0.0; bz_acc = 0.0; tilt_acc = 0.0; pitch_acc = 0.0; roll_acc = 0.0; nacc = 0
        for step in range(args.eval_steps):
            with torch.no_grad():
                mu, _, _ = ac(obs)
            obs, _, done, _ = env.step(mu)
            survived += (~done).to(torch.int32)
            newly = done & (first_fall == 0)
            first_fall = torch.where(newly, torch.full_like(first_fall, step + 1), first_fall)
            # ghost-match: mean |leg/waist angle - reference| (rad) + pelvis z + tilt
            qleg = env.qpos_t.index_select(1, env.qpos_idx_t) - env.nominal_t.unsqueeze(0)
            pose_acc += qleg.abs().mean().item(); bz_acc += env.qpos_t[:, 2].mean().item()
            qw = env.qpos_t[:, 3]; qx = env.qpos_t[:, 4]; qy = env.qpos_t[:, 5]; qz = env.qpos_t[:, 6]
            tilt = torch.acos(torch.clamp(1 - 2 * (qx * qx + qy * qy), -1.0, 1.0))
            pitch = torch.asin(torch.clamp(2 * (qw * qy - qz * qx), -1.0, 1.0))
            roll = torch.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
            tilt_acc += tilt.mean().item(); pitch_acc += pitch.mean().item(); roll_acc += roll.mean().item()
            nacc += 1
        s = survived.cpu().numpy(); ff = first_fall.cpu().numpy(); never = (ff == 0)
        print(f"[gpu-eval] policy={args.save} envs={env.n} steps={args.eval_steps}")
        print(f"  survival steps: mean={s.mean():.1f} median={np.median(s):.0f} "
              f"frac_full={(s >= args.eval_steps).mean():.2f}  never_fell={never.mean():.2f}")
        print(f"  GHOST-MATCH: mean|leg dev|={pose_acc / nacc:.4f} rad  "
              f"mean pelvis z={bz_acc / nacc:.3f} (ref {SPAWN_Z})  "
              f"mean torso tilt={math.degrees(tilt_acc / nacc):.1f} deg "
              f"(pitch={math.degrees(pitch_acc / nacc):+.1f} roll={math.degrees(roll_acc / nacc):+.1f}; "
              f"+pitch=lean BACK, +roll=lean RIGHT)")
        return

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    rollout = args.rollout
    obs = env.reset()
    total_steps = 0
    t0 = time.time()
    obs_buf = torch.zeros(rollout, N, OBS_DIM, device=tdev)
    act_buf = torch.zeros(rollout, N, NJ, device=tdev)
    logp_buf = torch.zeros(rollout, N, device=tdev)
    rew_buf = torch.zeros(rollout, N, device=tdev)
    done_buf = torch.zeros(rollout, N, device=tdev)
    val_buf = torch.zeros(rollout, N, device=tdev)

    for it in range(1, args.iters + 1):
        for k in range(rollout):
            with torch.no_grad():
                mu, v, log_std = ac(obs)
                dist = torch.distributions.Normal(mu, log_std.exp())
                a = dist.sample()
                logp = dist.log_prob(a).sum(-1)
            obs_buf[k] = obs; act_buf[k] = a; logp_buf[k] = logp; val_buf[k] = v
            obs, rr, done, _ = env.step(a)
            rew_buf[k] = rr; done_buf[k] = done.float()
            total_steps += N
        with torch.no_grad():
            _, last_v, _ = ac(obs)
        gamma, lam = 0.99, 0.95
        adv = torch.zeros_like(rew_buf)
        last_gae = torch.zeros(N, device=tdev)
        for k in reversed(range(rollout)):
            nonterm = 1.0 - done_buf[k]
            nextv = last_v if k == rollout - 1 else val_buf[k + 1]
            delta = rew_buf[k] + gamma * nextv * nonterm - val_buf[k]
            last_gae = delta + gamma * lam * nonterm * last_gae
            adv[k] = last_gae
        ret = adv + val_buf
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        obs_flat = obs_buf.reshape(-1, OBS_DIM); act_flat = act_buf.reshape(-1, NJ)
        logp_flat = logp_buf.reshape(-1); adv_flat = adv.reshape(-1); ret_flat = ret.reshape(-1)
        for _epoch in range(4):
            mu, v, log_std = ac(obs_flat)
            dist = torch.distributions.Normal(mu, log_std.exp())
            new_logp = dist.log_prob(act_flat).sum(-1)
            ratio = (new_logp - logp_flat).exp()
            surr1 = ratio * adv_flat
            surr2 = torch.clamp(ratio, 0.8, 1.2) * adv_flat
            pi_loss = -torch.min(surr1, surr2).mean()
            v_loss = ((v - ret_flat) ** 2).mean()
            ent = dist.entropy().sum(-1).mean()
            loss = pi_loss + 0.5 * v_loss - 0.01 * ent
            opt.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5)
            opt.step()
        if it % 5 == 0 or it == 1:
            fps = total_steps / max(time.time() - t0, 1e-6)
            print(f"it {it:4d}  ep_rew/step~{rew_buf.mean().item():+.3f}  "
                  f"meanV {val_buf.mean().item():+.2f}  steps {total_steps:,}  "
                  f"{fps:,.0f} env-steps/s")

    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total_steps:,} steps in {time.time() - t0:.1f}s)")

    onnx_path = Path(args.save).with_suffix(".onnx")

    class DeployPolicy(torch.nn.Module):
        def __init__(self, pi):
            super().__init__()
            self.pi = pi

        def forward(self, obs):
            return torch.tanh(self.pi(obs))

    cpu_ac = AC()
    cpu_ac.load_state_dict({k: v.cpu() for k, v in ac.state_dict().items()})
    wrapped = DeployPolicy(cpu_ac.pi)
    wrapped.eval()
    try:
        import io, contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            torch.onnx.export(wrapped, torch.zeros(1, OBS_DIM), str(onnx_path),
                              input_names=["obs"], output_names=["action"],
                              dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
                              opset_version=17)
        print(f"exported ONNX -> {onnx_path}")
    except Exception as e:
        print(f"ONNX export failed (non-fatal): {e}")


if __name__ == "__main__":
    main()
