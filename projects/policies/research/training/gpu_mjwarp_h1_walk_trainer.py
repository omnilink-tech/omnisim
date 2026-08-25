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

"""GPU mujoco_warp WALK trainer for the Unitree H1 (~1.0 m leg, 47 kg).

Built on the shared humanoid walk template (the G1 human-gait recipe). The
non-trivial change vs G1: H1 has 5-DOF legs (NO ankle-roll), so it drives
NJ=11 joints (5 leg x2 + waist), not 13. The h1_human_gait shadow still emits
the 13-slot layout [L: HP,HR,HY,KN,AP,AR; R..; waist]; we keep the 11 driven
slots via GAIT_KEEP (drop ankle-roll = slots 5, 11). The MJCF is full-body
(arms present for mass, held at 0 by the zero ctrl). H1-specific:

    MJCF        projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml
                (the EXACT OmniSim Newton dump; kp800/kd60 baked = deploy KE/KD,
                 so trainer==deploy by construction -- import_newton_mjcf_h1.py)
    gait        projects/policies/control/gait/h1_human_gait.py (calibrated from the URDF)
    NJ          11   OBS_DIM 44   SPAWN_Z 1.01   BZ_FAIL 0.55
    deploy      projects/policies/controllers/humanoid_walk_deploy (the residual
                plugs onto the same shadow; HUMANOID_WALK_ROBOT=h1)

Usage:
    python projects/policies/research/training/gpu_mjwarp_h1_walk_trainer.py \\
        --gait-model human --iters 600 --envs 8192 \\
        --seed-gait-pose --rest-start-frac 0.3 \\
        --save projects/policies/research/training/runs/gpu_h1_walk1/policy.pt
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


# ────────────────────────────────────────────────────────────────────
# Walk layout constants.
# ────────────────────────────────────────────────────────────────────
NJ = 11            # H1 drives 5-DOF legs (NO ankle-roll) x2 + waist = 11
QPOS_J0 = 7
QVEL_J0 = 6
OBS_DIM = 44       # lin(3)+ang(3)+proj_g(3)+q(11)+qd(11)+last_action(11)+gait_phase(2)
import os as _rsos
RES_SCALE = float(_rsos.environ.get("H1_RES_SCALE", "0.1"))
# The H1 shadow outputs the 13-slot gait layout [L: HP,HR,HY,KN,AP,AR; R..; waist].
# H1 has no ankle-roll, so we KEEP the 11 driven slots (drop ankle-roll = 5, 11).
GAIT_KEEP = [0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 12]
DT = 0.016
SUBSTEPS = 4
PHYS_DT = 0.004

LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_joint",
    "torso_joint",
)

# H1 flat-foot stand (gait order, no ankle-roll). The human gait model overrides
# this at runtime with its own standing_pose, sliced to the 11 driven slots.
NOMINAL = np.array([
    -0.30, +0.00, +0.00, +0.60, -0.30,    # left leg (hip_p, hip_r, hip_y, knee, ankle)
    -0.30, +0.00, +0.00, +0.60, -0.30,    # right leg
    +0.00,                                # waist
], dtype=np.float32)
assert NOMINAL.shape == (NJ,)

# Analytic ankle balance PD — DEFAULT OFF for the H1 (stand-journey lesson).
import os as _balos
KP_ANKLE_PITCH = float(_balos.environ.get("H1_TRAIN_BAL_KP_P", "0.0"))
KD_ANKLE_PITCH = float(_balos.environ.get("H1_TRAIN_BAL_KD_P", "0.0"))
KP_ANKLE_ROLL = 0.0   # H1 has no ankle-roll joint
KD_ANKLE_ROLL = 0.0
BAL_CLAMP = float(_balos.environ.get("H1_TRAIN_BAL_CLAMP", "0.2"))

_L_AP = LEGS_JOINTS.index("left_ankle_joint")
_R_AP = LEGS_JOINTS.index("right_ankle_joint")
_L_HP = LEGS_JOINTS.index("left_hip_pitch_joint")
_R_HP = LEGS_JOINTS.index("right_hip_pitch_joint")
_L_KN = LEGS_JOINTS.index("left_knee_joint")
_R_KN = LEGS_JOINTS.index("right_knee_joint")
_L_HR = LEGS_JOINTS.index("left_hip_roll_joint")
_R_HR = LEGS_JOINTS.index("right_hip_roll_joint")
_L_HY = LEGS_JOINTS.index("left_hip_yaw_joint")
_R_HY = LEGS_JOINTS.index("right_hip_yaw_joint")
GAIT_A_LAT = 0.0

# Sine-CPG defaults (legacy mode; the human gait model is primary).
GAIT_FREQ = 1.2       # Hz (H1 shadow cadence)
GAIT_A_HIP = 0.35
GAIT_A_KNEE = 0.45
VX_TARGET = 0.45      # m/s (H1 shadow vx)

# Episode termination — H1 heights.
SPAWN_Z = 1.01        # H1 spawn / walk standing-pose height
BZ_FAIL = 0.55
ROLL_FAIL = 0.8
PITCH_FAIL = 0.8
MAX_EP = 500

# H1 joint limits, gait order (HP,HR,HY,KN,AP per leg, then waist), no ankle-roll.
JOINT_LIMITS_LO = np.array([
    -3.14, -0.43, -0.43, -0.26, -0.87,
    -3.14, -0.43, -0.43, -0.26, -0.87,
    -2.35,
], dtype=np.float32)
JOINT_LIMITS_HI = np.array([
    +2.53, +0.43, +0.43, +2.05, +0.52,
    +2.53, +0.43, +0.43, +2.05, +0.52,
    +2.35,
], dtype=np.float32)

# Full-body mode (--hold-arms): H1's 8 arm joints (shoulder p/r/y + elbow per
# arm, NO wrist), held at nominal (arms down). The matched MJCF is full-body.
ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
)
ARM_NOMINAL = np.array([
    +0.00, +0.00, +0.00, +0.00,
    +0.00, +0.00, +0.00, +0.00,
], dtype=np.float32)


# ────────────────────────────────────────────────────────────────────
class BatchedH1WalkEnv:
    """Walk-task batched env — gait reference in the baseline, residual
    policy on top. Identical structure to the G1 walk env."""

    def __init__(self, n, mjcf, device="cuda:0", reward_cfg=None, sim_dt=0.0,
                 dr_cfg=None, hold_arms=False, obs_history=1):
        import warp as wp
        # closed-loop: frame-stack the last K obs frames so the policy can read
        # velocity/accel TRENDS (a memoryless single-frame MLP went open-loop).
        self._hist_k = max(1, int(obs_history))
        self._obs_hist = None          # [n, K, OBS_DIM], lazily built on first obs
        self._reset_idx = None         # envs reset this step -> refill their history
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mjw = wp, mjw
        self.n = n
        self.hold_arms = hold_arms
        self.device = wp.get_device(device)
        self.dr = dr_cfg or {}

        self.mjm = mujoco.MjModel.from_xml_path(mjcf)
        self.mjm.opt.timestep = float(sim_dt) if sim_dt and sim_dt > 0 else PHYS_DT

        # Optional PD-gain override (reward_cfg kp/kv): the dump bakes the
        # STAND gain point (kp600/kv60), but walking wants SOFTER joints —
        # G1 walked at kp100 while standing at ke400. Deploy must set
        # OMNISIM_NEWTON_TARGET_KE/_KD to the SAME values.
        kp_over = float((reward_cfg or {}).get("kp", 0.0))
        kv_over = float((reward_cfg or {}).get("kv", 0.0))
        if kp_over > 0 or kv_over > 0:
            for ai in range(self.mjm.nu):
                gp_ = self.mjm.actuator_gainprm[ai]
                bp_ = self.mjm.actuator_biasprm[ai]
                if abs(bp_[1]) > 1e-6 and kp_over > 0:
                    gp_[0] = kp_over
                    bp_[1] = -kp_over
                elif abs(bp_[2]) > 1e-6 and kv_over > 0:
                    gp_[0] = kv_over
                    bp_[2] = -kv_over
            print(f"[env] actuator gains overridden: kp={kp_over} kv={kv_over}")

        rng = np.random.default_rng(self.dr.get("seed", 0))
        mass_scale_band = self.dr.get("mass_scale", 0.0)
        fric_band = self.dr.get("friction_scale", 0.0)
        damp_band = self.dr.get("damping_scale", 0.0)
        actuator_kp_band = self.dr.get("actuator_kp_scale", 0.0)
        actuator_kv_band = self.dr.get("actuator_kv_scale", 0.0)
        gravity_band = self.dr.get("gravity_scale", 0.0)
        if mass_scale_band > 0:
            scales = rng.uniform(1.0 - mass_scale_band, 1.0 + mass_scale_band,
                                 size=self.mjm.body_mass.shape).astype(np.float32)
            self.mjm.body_mass[:] *= scales
            self.mjm.body_inertia[:] *= scales[:, None]
        if fric_band > 0:
            fs = float(rng.uniform(1.0 - fric_band, 1.0 + fric_band))
            self.mjm.geom_friction[:, 0] *= fs
        if damp_band > 0:
            ds = rng.uniform(1.0 - damp_band, 1.0 + damp_band,
                             size=self.mjm.dof_damping.shape).astype(np.float32)
            self.mjm.dof_damping[:] *= ds
        if actuator_kp_band > 0 or actuator_kv_band > 0:
            for ai in range(self.mjm.nu):
                gp = self.mjm.actuator_gainprm[ai]
                bp = self.mjm.actuator_biasprm[ai]
                if abs(bp[1]) > 1e-6 and actuator_kp_band > 0:
                    s = float(rng.uniform(1.0 - actuator_kp_band,
                                          1.0 + actuator_kp_band))
                    gp[0] *= s
                    bp[1] *= s
                elif abs(bp[2]) > 1e-6 and actuator_kv_band > 0:
                    s = float(rng.uniform(1.0 - actuator_kv_band,
                                          1.0 + actuator_kv_band))
                    gp[0] *= s
                    bp[2] *= s
        if gravity_band > 0:
            gs = float(rng.uniform(1.0 - gravity_band, 1.0 + gravity_band))
            self.mjm.opt.gravity[2] *= gs

        mjd = mujoco.MjData(self.mjm)
        mujoco.mj_forward(self.mjm, mjd)
        with wp.ScopedDevice(self.device):
            self.mw_m = mjw.put_model(self.mjm)
            import os as _njos
            _njmax = int(_njos.environ.get("NE1_NJMAX", "256"))
            _nconmax = int(_njos.environ.get("NE1_NCONMAX", "256"))
            self.mw_d = mjw.put_data(self.mjm, mjd, nworld=n,
                                     njmax=_njmax, nconmax=_nconmax)

        self.controller_to_qpos = np.zeros(NJ, dtype=np.int32)
        self.controller_to_qvel = np.zeros(NJ, dtype=np.int32)
        self.controller_to_ctrl_pos = np.zeros(NJ, dtype=np.int32)
        self.controller_to_ctrl_vel = np.zeros(NJ, dtype=np.int32)
        for i, jn in enumerate(LEGS_JOINTS):
            jid = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                raise RuntimeError(f"joint {jn} not in MJCF")
            self.controller_to_qpos[i] = self.mjm.jnt_qposadr[jid]
            self.controller_to_qvel[i] = self.mjm.jnt_dofadr[jid]
            pos_id = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_pos")
            vel_id = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_vel")
            if pos_id < 0 or vel_id < 0:
                raise RuntimeError(
                    f"actuators {jn}_pos/_vel not in MJCF — see "
                    f"projects/policies/research/training/import_newton_mjcf_h1.py")
            self.controller_to_ctrl_pos[i] = pos_id
            self.controller_to_ctrl_vel[i] = vel_id

        self.arm_ctrl_pos = np.zeros(len(ARM_JOINTS), dtype=np.int32)
        self.arm_qpos = np.zeros(len(ARM_JOINTS), dtype=np.int32)
        if self.hold_arms:
            for i, jn in enumerate(ARM_JOINTS):
                jid = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid < 0:
                    raise RuntimeError(
                        f"--hold-arms: arm joint {jn} not in MJCF {mjcf}.")
                self.arm_qpos[i] = self.mjm.jnt_qposadr[jid]
                pos_id = mujoco.mj_name2id(
                    self.mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_pos")
                if pos_id < 0:
                    raise RuntimeError(f"--hold-arms: actuator {jn}_pos missing")
                self.arm_ctrl_pos[i] = pos_id

        self.r = reward_cfg or {}

        self.nominal = np.array(self.r.get("nominal", NOMINAL), dtype=np.float32)
        assert self.nominal.shape == (NJ,)

        self.seed_qpos = mjd.qpos.copy().astype(np.float32)
        self.seed_qpos[0:3] = [0.0, 0.0, SPAWN_Z]
        self.seed_qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for i, jn in enumerate(LEGS_JOINTS):
            self.seed_qpos[self.controller_to_qpos[i]] = self.nominal[i]
        if self.hold_arms:
            for i in range(len(ARM_JOINTS)):
                self.seed_qpos[self.arm_qpos[i]] = ARM_NOMINAL[i]
        self.nq = self.mjm.nq
        self.nv = self.mjm.nv
        self.nu = self.mjm.nu

        self.tdev = torch.device("cuda:0" if "cuda" in str(self.device).lower()
                                 else "cpu")
        self.qpos_t = wp.to_torch(self.mw_d.qpos).view(n, self.nq)
        self.qvel_t = wp.to_torch(self.mw_d.qvel).view(n, self.nv)
        self.ctrl_t = wp.to_torch(self.mw_d.ctrl).view(n, self.nu)

        self.nominal_t = torch.tensor(self.nominal, dtype=torch.float32, device=self.tdev)
        self.jl_lo_t = torch.tensor(JOINT_LIMITS_LO, dtype=torch.float32, device=self.tdev)
        self.jl_hi_t = torch.tensor(JOINT_LIMITS_HI, dtype=torch.float32, device=self.tdev)
        self.qpos_idx_t = torch.tensor(self.controller_to_qpos, dtype=torch.long, device=self.tdev)
        self.qvel_idx_t = torch.tensor(self.controller_to_qvel, dtype=torch.long, device=self.tdev)
        self.ctrl_pos_idx_t = torch.tensor(self.controller_to_ctrl_pos, dtype=torch.long, device=self.tdev)
        self.gait_keep_t = torch.tensor(GAIT_KEEP, dtype=torch.long, device=self.tdev)
        self.seed_qpos_t = torch.tensor(self.seed_qpos, dtype=torch.float32, device=self.tdev)
        if self.hold_arms:
            self.arm_ctrl_pos_idx_t = torch.tensor(
                self.arm_ctrl_pos, dtype=torch.long, device=self.tdev)
            self.arm_targets_t = torch.tensor(
                ARM_NOMINAL, dtype=torch.float32, device=self.tdev
            ).unsqueeze(0).expand(n, -1).contiguous()

        self.ep_step_t = torch.zeros(n, dtype=torch.int32, device=self.tdev)
        self.last_action_t = torch.zeros(n, NJ, dtype=torch.float32, device=self.tdev)
        self.prev_roll_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.prev_pitch_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.phase_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)

        # H1_ENV_CORE: train the obs qd as the deploy DOES it -- finite-diff from
        # achieved joint positions (shared h1_env_core.JointVelEstimator) -- instead
        # of exact mjw qvel. The MjModel field-diff proved the model is byte-faithful
        # to the deploy; the remaining genuine obs gap is qd (mean ~0.347 rad/s
        # raw-vs-finite-diff on G1). Default OFF = legacy exact-qvel obs, byte-identical.
        import os as _ec_os
        self._env_core = bool(int(_ec_os.environ.get("H1_ENV_CORE", "0")))
        self._qd_est = None
        if self._env_core:
            from projects.policies.research.backends.h1_env_core import JointVelEstimator
            self._qd_est = JointVelEstimator(dt=DT)
            print(f"[H1_ENV_CORE] obs qd = deploy finite-diff (dt={DT}), not exact qvel")
        self.gait_freq = float(self.r.get("gait_freq", GAIT_FREQ))
        self.gait_a_hip = float(self.r.get("gait_a_hip", GAIT_A_HIP))
        self.gait_a_knee = float(self.r.get("gait_a_knee", GAIT_A_KNEE))
        self.gait_a_lat = float(self.r.get("gait_a_lat", GAIT_A_LAT))
        self.cp_gain = float(self.r.get("cp_gain", 0.0))
        self.vx_target = float(self.r.get("vx_target", VX_TARGET))
        self.max_ep = int(self.r.get("max_ep", MAX_EP))
        self.res_scale = float(self.r.get("res_scale", RES_SCALE))
        # H1_PURE_RL: full-authority RL (legged_gym/DeepMimic) -- targets = nominal +
        # act_scale*action, no ghost tracking. See docs/developer/locomotion-shadowing-vs-pure-rl.md.
        import os as _prlos
        self._pure_rl = bool(int(_prlos.environ.get("H1_PURE_RL", "0")))
        self._act_scale = float(_prlos.environ.get("H1_ACT_SCALE", "1.0"))
        if self._pure_rl:
            print(f"[H1_PURE_RL] full-authority RL: targets = nominal + {self._act_scale}*action (no ghost)")
        self.gait_a_ankle = float(self.r.get("gait_a_ankle", 0.0))
        self.seed_gait = bool(self.r.get("seed_gait", False))
        self.gait_a_arm = float(self.r.get("gait_a_arm", 0.0))
        self.gait_a_push = float(self.r.get("gait_a_push", 0.0))
        self.rest_start_frac = float(self.r.get("rest_start_frac", 0.0))
        # HUMAN GAIT MODEL (H1-recalibrated module).
        self.gait_model = str(self.r.get("gait_model", ""))
        self.gp = None
        if self.gait_model == "human":
            from projects.policies.control.gait import h1_human_gait as ghg
            self._ghg = ghg
            gpd = self.r.get("gait_params", {}) or {}
            self.gp = ghg.GaitParams(**gpd)
            # standing_pose is the 13-slot gait layout; keep H1's 11 driven slots.
            self.nominal = ghg.standing_pose(self.gp).astype(np.float32)[GAIT_KEEP]
            self.vx_target = self.gp.vx
            self.gait_freq = self.gp.freq
            self.phase_dt = 2.0 * math.pi * self.gait_freq * DT
            for i in range(NJ):
                self.seed_qpos[self.controller_to_qpos[i]] = self.nominal[i]
            self.seed_qpos_t = torch.tensor(self.seed_qpos, dtype=torch.float32,
                                            device=self.tdev)
            self.nominal_t = torch.tensor(self.nominal, dtype=torch.float32,
                                          device=self.tdev)
        self._ramp_t0 = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self._swingL = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self._swingR = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.rw_sched = float(self.r.get("rw_sched", 0.0))
        self.rw_slip = float(self.r.get("rw_slip", 0.0))
        # Foot contact / swing height thresholds (metres), H1 scale.
        self.foot_z_contact = float(self.r.get("foot_z_contact", 0.07))
        self.foot_z_swing = float(self.r.get("foot_z_swing", 0.19))
        self.phase_dt = 2.0 * math.pi * self.gait_freq * DT

        self.bid_lfoot = mujoco.mj_name2id(
            self.mjm, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_link")
        self.bid_rfoot = mujoco.mj_name2id(
            self.mjm, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_link")
        self.xpos_t = wp.to_torch(self.mw_d.xpos).view(n, self.mjm.nbody, 3)
        self.prev_foot_xy_t = torch.zeros(n, 2, 2, dtype=torch.float32,
                                          device=self.tdev)

        self.max_latency_ticks = int(self.dr.get("action_latency_max", 0))
        self.action_buffer_t = torch.zeros(
            n, max(1, self.max_latency_ticks + 1), NJ,
            dtype=torch.float32, device=self.tdev)
        self.action_delay_t = torch.zeros(n, dtype=torch.long, device=self.tdev)

        self._push_p = float(self.dr.get("push_prob", 0.0))
        self._push_vmax = float(self.dr.get("push_vmax", 0.0))
        self._obs_noise = float(self.dr.get("obs_noise", 0.0))
        self._init_q_band = float(self.dr.get("init_q_band", 0.05))
        self._init_xy_band = float(self.dr.get("init_xy_band", 0.03))
        self._init_z_band = float(self.dr.get("init_z_band", 0.0))

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
            print(f"[env] CUDA graph capture failed ({e}); using direct step")
            self._cuda_graph = None

    def _reset_envs(self, env_mask=None):
        if env_mask is None:
            idx = torch.arange(self.n, device=self.tdev)
        else:
            idx = torch.nonzero(env_mask, as_tuple=False).squeeze(-1)
            if idx.numel() == 0:
                return
        m = idx.shape[0]
        self._reset_idx = idx          # obs-history: refill these envs' stacks

        self.qpos_t[idx] = self.seed_qpos_t.unsqueeze(0).expand(m, -1)
        self.qvel_t[idx] = 0.0

        if self._init_q_band > 0:
            jitter = (torch.rand(m, NJ, device=self.tdev) * 2 - 1) * self._init_q_band
            base_idx = idx.unsqueeze(1).expand(-1, NJ)
            col_idx = self.qpos_idx_t.unsqueeze(0).expand(m, -1)
            jittered = self.qpos_t[base_idx, col_idx] + jitter
            jittered = torch.clamp(jittered,
                                   self.jl_lo_t.unsqueeze(0) + 0.02,
                                   self.jl_hi_t.unsqueeze(0) - 0.02)
            self.qpos_t[base_idx, col_idx] = jittered
        if self._init_xy_band > 0:
            self.qpos_t[idx, 0] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_xy_band
            self.qpos_t[idx, 1] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_xy_band
        if self._init_z_band > 0:
            self.qpos_t[idx, 2] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_z_band

        tilt = float(self.dr.get("init_tilt_band", 0.0))
        if tilt > 0:
            hr = (torch.rand(m, device=self.tdev) * 2 - 1) * (tilt * 0.5)
            hp = (torch.rand(m, device=self.tdev) * 2 - 1) * (tilt * 0.5)
            cr = torch.cos(hr); sr = torch.sin(hr)
            cp = torch.cos(hp); sp = torch.sin(hp)
            self.qpos_t[idx, 3] = cr * cp
            self.qpos_t[idx, 4] = sr * cp
            self.qpos_t[idx, 5] = cr * sp
            self.qpos_t[idx, 6] = sr * sp
        vband = float(self.dr.get("init_vel_band", 0.0))
        if vband > 0:
            self.qvel_t[idx, 0:6] += (torch.rand(m, 6, device=self.tdev) * 2 - 1) * vband
        vxbias = float(self.dr.get("init_vx_bias", 0.0))
        if vxbias > 0:
            self.qvel_t[idx, 0] += torch.rand(m, device=self.tdev) * vxbias

        self.ep_step_t[idx] = 0
        self.last_action_t[idx] = 0.0
        self.prev_roll_t[idx] = 0.0
        self.prev_pitch_t[idx] = 0.0
        self.phase_t[idx] = torch.rand(m, device=self.tdev) * (2.0 * math.pi)
        if self.seed_gait and self.gait_model == "human":
            th = self.phase_t[idx]
            full = torch.full((idx.shape[0],), 1e6, device=self.tdev)
            legs0, _, _, _ = self._ghg.targets_torch(th, self.gp, full)
            legs1, _, _, _ = self._ghg.targets_torch(th + self.phase_dt, self.gp, full)
            legs0 = legs0[:, self.gait_keep_t]   # 13-slot gait -> H1's 11 driven slots
            legs1 = legs1[:, self.gait_keep_t]
            qd_ref = (legs1 - legs0) / DT
            base_idx = idx.unsqueeze(1).expand(-1, NJ)
            self.qpos_t[base_idx, self.qpos_idx_t.unsqueeze(0).expand(idx.shape[0], -1)] = legs0
            self.qvel_t[base_idx, self.qvel_idx_t.unsqueeze(0).expand(idx.shape[0], -1)] = qd_ref
            self._ramp_t0[idx] = 1e6
        elif self.seed_gait:
            th = self.phase_t[idx]
            sL = torch.sin(th); cL = torch.cos(th)
            sR = -sL; cR = -cL
            om = 2.0 * math.pi * self.gait_freq
            qcol = self.qpos_idx_t
            vcol = self.qvel_idx_t
            qp = self.qpos_t; qv = self.qvel_t
            qp[idx, qcol[_L_HP]] += -self.gait_a_hip * sL
            qp[idx, qcol[_R_HP]] += -self.gait_a_hip * sR
            qp[idx, qcol[_L_KN]] += self.gait_a_knee * torch.clamp(sL, min=0.0)
            qp[idx, qcol[_R_KN]] += self.gait_a_knee * torch.clamp(sR, min=0.0)
            qv[idx, vcol[_L_HP]] += -self.gait_a_hip * cL * om
            qv[idx, vcol[_R_HP]] += -self.gait_a_hip * cR * om
            qv[idx, vcol[_L_KN]] += self.gait_a_knee * cL * om * (sL > 0)
            qv[idx, vcol[_R_KN]] += self.gait_a_knee * cR * om * (sR > 0)
            if self.gait_a_ankle != 0.0:
                qp[idx, qcol[_L_AP]] += self.gait_a_ankle * sL
                qp[idx, qcol[_R_AP]] += self.gait_a_ankle * sR
                qv[idx, vcol[_L_AP]] += self.gait_a_ankle * cL * om
                qv[idx, vcol[_R_AP]] += self.gait_a_ankle * cR * om
            if self.gait_a_lat != 0.0:
                sway = self.gait_a_lat * sL
                sway_d = self.gait_a_lat * cL * om
                qp[idx, qcol[_L_HR]] += sway
                qp[idx, qcol[_R_HR]] += sway
                qv[idx, vcol[_L_HR]] += sway_d
                qv[idx, vcol[_R_HR]] += sway_d
        if self.rest_start_frac > 0:
            rest = torch.rand(idx.shape[0], device=self.tdev) < self.rest_start_frac
            ridx = idx[rest]
            if ridx.numel() > 0:
                rm = ridx.shape[0]
                self.qpos_t[ridx] = self.seed_qpos_t.unsqueeze(0).expand(rm, -1)
                self.qvel_t[ridx] = 0.0
                rj = (torch.rand(rm, NJ, device=self.tdev) * 2 - 1) * 0.03
                # GRAVITY SAG (deploy settle steady-state error tau/kp) at
                # the WALK gain point kp250. Measured in deploy (2026-06-12):
                # settle bz 1.032->0.993, pitch -0.094 — the first h-ladder
                # band (0.06, written for kp600) UNDERCOVERED it and the
                # deploy launch was out-of-distribution (fell at 2.06 s while
                # the trainer rest-start eval was perfect). G1's kp100 values
                # scaled: knee +0.20, ankle -0.10.
                sag = torch.rand(rm, device=self.tdev)
                for kn_i, ak_i in ((_L_KN, _L_AP), (_R_KN, _R_AP)):
                    rj[:, kn_i] += 0.20 * sag
                    rj[:, ak_i] += -0.10 * sag
                ri = ridx.unsqueeze(1).expand(-1, NJ)
                rc = self.qpos_idx_t.unsqueeze(0).expand(rm, -1)
                self.qpos_t[ri, rc] += rj
                # Base tilt band must COVER the deploy's settled backward
                # lean (pitch -0.094 measured): pitch in [-0.12, +0.04],
                # roll small.
                hr = (torch.rand(rm, device=self.tdev) * 2 - 1) * 0.02
                hp2 = (torch.rand(rm, device=self.tdev) * 0.16 - 0.12) * 0.5
                cr = torch.cos(hr); sr = torch.sin(hr)
                cp2 = torch.cos(hp2); sp2 = torch.sin(hp2)
                self.qpos_t[ridx, 3] = cr * cp2
                self.qpos_t[ridx, 4] = sr * cp2
                self.qpos_t[ridx, 5] = cr * sp2
                self.qpos_t[ridx, 6] = sr * sp2
                self.qvel_t[ridx, 0:6] += (torch.rand(rm, 6, device=self.tdev) * 2 - 1) * 0.05
                self.phase_t[ridx] = (self._ghg.DS_PHASE
                                      if self.gait_model == "human" else 0.0)
                self._ramp_t0[ridx] = 0.0
        self.action_buffer_t[idx] = 0.0
        if self.max_latency_ticks > 0:
            self.action_delay_t[idx] = torch.randint(
                0, self.max_latency_ticks + 1, (m,),
                dtype=torch.long, device=self.tdev)
        else:
            self.action_delay_t[idx] = 0

        # Refresh xpos via KINEMATICS, not a full forward. Some env resets
        # nearly every step, so this runs ~once/step; a full mjw.forward
        # (collision+constraint solve) here was ~60% of total train time. xpos
        # is a pure function of qpos via forward kinematics -> bit-identical for
        # the foot xy the slip tracker reads, but ~15x cheaper (the G1 trainer's
        # throughput patch; nothing reads cvel/contacts before the next step).
        with self.wp.ScopedDevice(self.device):
            self.mjw.kinematics(self.mw_m, self.mw_d)
        feet = self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), 0:2]
        self.prev_foot_xy_t[idx] = feet[idx]

        # H1_ENV_CORE: re-seed the finite-diff qd estimator for the reset envs so
        # the first post-reset qd is 0 (no teleport velocity), matching the deploy.
        if self._env_core and self._qd_est is not None:
            q_ach = self.qpos_t.index_select(1, self.qpos_idx_t)
            mask = torch.zeros(self.n, dtype=torch.bool, device=self.tdev)
            mask[idx] = True
            self._qd_est.reset_rows(q_ach, mask)

    def _reset_all(self):
        self._reset_envs(env_mask=None)

    def _build_frame_t(self):
        qp = self.qpos_t
        qv = self.qvel_t
        vlin = qv[:, 0:3]
        vang = qv[:, 3:6]
        w = qp[:, 3]; x = qp[:, 4]; y = qp[:, 5]; z = qp[:, 6]
        gx = -2 * (x * z - w * y)
        gy = -2 * (y * z + w * x)
        gz = -(1 - 2 * (x * x + y * y))
        pg = torch.stack([gx, gy, gz], dim=1)
        q_ach = qp.index_select(1, self.qpos_idx_t)        # achieved leg q
        q = q_ach - self.nominal_t.unsqueeze(0)
        if getattr(self, "_env_core", False):
            qd = self._qd_est.update(q_ach)                # deploy finite-diff qd
        else:
            qd = qv.index_select(1, self.qvel_idx_t)       # exact mjw qvel (legacy)
        gait = torch.stack([torch.sin(self.phase_t), torch.cos(self.phase_t)], dim=1)
        obs = torch.cat([vlin, vang, pg, q, qd, self.last_action_t, gait], dim=1)
        if self._obs_noise > 0:
            obs = obs + torch.randn_like(obs) * self._obs_noise
        obs = torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = torch.clamp(obs, -10.0, 10.0)
        return obs

    def _build_obs_t(self):
        """Single frame if K==1, else the last K frames concatenated (oldest first).
        Reset envs get their whole history refilled with the current frame so no
        stale pre-reset frames leak across an episode boundary (that would be OOD)."""
        frame = self._build_frame_t()                      # [n, OBS_DIM]
        K = self._hist_k
        if K <= 1:
            return frame
        if self._obs_hist is None:
            self._obs_hist = frame.unsqueeze(1).repeat(1, K, 1)
        else:
            self._obs_hist = torch.cat(
                [self._obs_hist[:, 1:, :], frame.unsqueeze(1)], dim=1)
        ridx = self._reset_idx
        if ridx is not None and ridx.numel() > 0:
            self._obs_hist[ridx] = frame[ridx].unsqueeze(1).expand(-1, K, -1)
        self._reset_idx = None
        return self._obs_hist.reshape(self.n, K * frame.shape[1])

    def _baseline_targets_t(self):
        qp = self.qpos_t
        w = qp[:, 3]; x = qp[:, 4]; y = qp[:, 5]; z = qp[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sinp = torch.clamp(2 * (w * y - z * x), -1.0, 1.0)
        pitch = torch.asin(sinp)
        roll_rate = (roll - self.prev_roll_t) / DT
        pitch_rate = (pitch - self.prev_pitch_t) / DT
        self.prev_roll_t = roll
        self.prev_pitch_t = pitch

        if KP_ANKLE_PITCH != 0.0 or KP_ANKLE_ROLL != 0.0:
            ap = torch.clamp(KP_ANKLE_PITCH * pitch + KD_ANKLE_PITCH * pitch_rate,
                             -BAL_CLAMP, BAL_CLAMP)
            ar = torch.clamp(KP_ANKLE_ROLL * roll + KD_ANKLE_ROLL * roll_rate,
                             -BAL_CLAMP, BAL_CLAMP)
        else:
            ap = ar = None
        if self.gait_model == "human":
            t_since = self._ramp_t0 + self.ep_step_t.to(torch.float32) * DT
            legs, arms, swL, swR = self._ghg.targets_torch(
                self.phase_t, self.gp, t_since_start_t=t_since,
                v_meas_t=self.qvel_t[:, 0])     # capture-point step placement (cp_gain>0)
            self._model_arms = arms
            self._swingL, self._swingR = swL, swR
            # gait outputs the 13-slot layout; keep H1's 11 driven slots.
            targets = legs[:, self.gait_keep_t].contiguous()
            if ap is not None:                       # H1 has no ankle-roll
                targets[:, _L_AP] = targets[:, _L_AP] + ap
                targets[:, _R_AP] = targets[:, _R_AP] + ap
            return targets, roll, pitch
        targets = self.nominal_t.unsqueeze(0).expand(self.n, -1).contiguous()
        if ap is not None:                           # H1 has no ankle-roll
            targets[:, _L_AP] = targets[:, _L_AP] + ap
            targets[:, _R_AP] = targets[:, _R_AP] + ap
        th = self.phase_t
        sL = torch.sin(th); sR = torch.sin(th + math.pi)
        targets[:, _L_HP] = targets[:, _L_HP] - self.gait_a_hip * sL
        targets[:, _R_HP] = targets[:, _R_HP] - self.gait_a_hip * sR
        targets[:, _L_KN] = targets[:, _L_KN] + self.gait_a_knee * torch.clamp(sL, min=0.0)
        targets[:, _R_KN] = targets[:, _R_KN] + self.gait_a_knee * torch.clamp(sR, min=0.0)
        if self.gait_a_ankle != 0.0:
            targets[:, _L_AP] = targets[:, _L_AP] + self.gait_a_ankle * sL
            targets[:, _R_AP] = targets[:, _R_AP] + self.gait_a_ankle * sR
        if self.gait_a_push != 0.0:
            pushL = torch.clamp(torch.sin(th - 1.5 * math.pi), min=0.0) ** 2
            pushR = torch.clamp(torch.sin(th - 0.5 * math.pi), min=0.0) ** 2
            targets[:, _L_AP] = targets[:, _L_AP] + self.gait_a_push * pushL
            targets[:, _R_AP] = targets[:, _R_AP] + self.gait_a_push * pushR
        if self.gait_a_lat != 0.0:
            sway = self.gait_a_lat * sL
            targets[:, _L_HR] = targets[:, _L_HR] + sway
            targets[:, _R_HR] = targets[:, _R_HR] + sway
        if self.cp_gain != 0.0:
            cp = self.cp_gain * self.qvel_t[:, 1]
            targets[:, _L_HR] = targets[:, _L_HR] + cp * torch.clamp(sL, min=0.0)
            targets[:, _R_HR] = targets[:, _R_HR] + cp * torch.clamp(sR, min=0.0)
        return targets, roll, pitch

    def reset(self):
        self._reset_all()
        return self._build_obs_t()

    def step(self, action_t):
        action_t = torch.clamp(action_t, -1.0, 1.0)

        if self.max_latency_ticks > 0:
            self.action_buffer_t = torch.roll(self.action_buffer_t, 1, dims=1)
            self.action_buffer_t[:, 0, :] = action_t
            row_idx = torch.arange(self.n, device=self.tdev)
            applied = self.action_buffer_t[row_idx, self.action_delay_t]
        else:
            applied = action_t

        baseline, _, _ = self._baseline_targets_t()   # advances phase + sets _model_arms
        if getattr(self, "_pure_rl", False):
            # FULL-AUTHORITY RL (legged_gym/DeepMimic): policy owns the whole joint
            # target (nominal + act_scale*action); the gait baseline is NOT tracked.
            # Rhythm = phase clock (obs) + contact-schedule reward; balance = policy's.
            targets = torch.clamp(self.nominal_t.unsqueeze(0) + self._act_scale * applied,
                                  self.jl_lo_t, self.jl_hi_t)
        else:
            targets = torch.clamp(baseline + self.res_scale * applied,
                                  self.jl_lo_t, self.jl_hi_t)
        self.ctrl_t.zero_()
        self.ctrl_t.index_copy_(1, self.ctrl_pos_idx_t, targets)
        if self.hold_arms:
            if self.gait_model == "human":
                self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, self._model_arms)
            elif self.gait_a_arm != 0.0:
                if not hasattr(self, "_arm_swing_buf"):
                    self._arm_swing_buf = self.arm_targets_t.clone()
                sw = self.gait_a_arm * torch.sin(self.phase_t)
                buf = self._arm_swing_buf
                buf.copy_(self.arm_targets_t)
                buf[:, 0] = buf[:, 0] + sw
                buf[:, 5] = buf[:, 5] - sw
                self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, buf)
            else:
                self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, self.arm_targets_t)

        if self._push_p > 0 and self._push_vmax > 0:
            hit = torch.rand(self.n, device=self.tdev) < self._push_p
            if hit.any():
                theta = torch.rand(self.n, device=self.tdev) * (2 * math.pi)
                mag = torch.rand(self.n, device=self.tdev) * self._push_vmax
                dvx = torch.cos(theta) * mag
                dvy = torch.sin(theta) * mag
                self.qvel_t[hit, 0] = self.qvel_t[hit, 0] + dvx[hit]
                self.qvel_t[hit, 1] = self.qvel_t[hit, 1] + dvy[hit]

        with self.wp.ScopedDevice(self.device):
            if self._cuda_graph is not None:
                self.wp.capture_launch(self._cuda_graph)
            else:
                for _ in range(SUBSTEPS):
                    self.mjw.step(self.mw_m, self.mw_d)

        self.ep_step_t = self.ep_step_t + 1
        prev_action_t = self.last_action_t
        self.last_action_t = action_t
        th_used = self.phase_t
        self.phase_t = torch.remainder(self.phase_t + self.phase_dt, 2.0 * math.pi)

        bz = self.qpos_t[:, 2]
        w = self.qpos_t[:, 3]; x = self.qpos_t[:, 4]
        y = self.qpos_t[:, 5]; z = self.qpos_t[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sinp = torch.clamp(2 * (w * y - z * x), -1.0, 1.0)
        pitch = torch.asin(sinp)

        r = self.r
        vx = self.qvel_t[:, 0]
        vy = self.qvel_t[:, 1]
        wz = self.qvel_t[:, 5]
        r_alive = r.get("alive", 1.0) * torch.ones(self.n, device=self.tdev)
        upright = torch.clamp(1.0 - roll * roll - pitch * pitch, min=0.0)
        r_up = r.get("upright", 0.5) * upright
        vsig = r.get("vel_sigma", 0.10)
        r_vel = r.get("vel", 2.0) * torch.exp(-((vx - self.vx_target) ** 2) / vsig)
        r_vel = r_vel + r.get("vel_l1", 0.0) * torch.abs(vx - self.vx_target)
        over = torch.clamp(vx - self.vx_target, min=0.0)        # anti-runaway
        r_vel = r_vel + r.get("overspeed", 0.0) * over * over
        r_lat = r.get("lat", -0.5) * torch.abs(vy)
        r_yaw = r.get("yaw", -0.5) * torch.abs(wz)
        z_ref = r.get("z_ref", 1.02)
        r_height = r.get("height", -10.0) * (bz - z_ref) ** 2
        r_act = r.get("act", -0.005) * (action_t * action_t).sum(dim=1)
        r_rate = r.get("act_rate", -0.01) * ((action_t - prev_action_t) ** 2).sum(dim=1)
        reward = (r_alive + r_up + r_vel + r_lat + r_yaw + r_height + r_act + r_rate)

        if self.rw_sched != 0.0 or self.rw_slip != 0.0:
            feet = self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), :]
            foot_z = feet[:, :, 2]
            foot_xy = feet[:, :, 0:2]
            if self.gait_model == "human":
                swing = torch.stack([self._swingL, self._swingR], dim=1)
            else:
                sL = torch.sin(th_used)
                swing = torch.stack([torch.clamp(sL, min=0.0),
                                     torch.clamp(-sL, min=0.0)], dim=1)
            stance = 1.0 - swing
            if self.rw_sched != 0.0:
                z_hi = torch.clamp(foot_z - self.foot_z_contact, min=0.0)
                z_lo = torch.clamp(self.foot_z_swing - foot_z, min=0.0)
                sched_pen = (stance * z_hi + swing * z_lo).sum(dim=1)
                reward = reward + self.rw_sched * sched_pen
            if self.rw_slip != 0.0:
                v_xy = (foot_xy - self.prev_foot_xy_t) / DT
                contact = (foot_z < self.foot_z_contact).float()
                slip_pen = (contact * torch.linalg.norm(v_xy, dim=2)).sum(dim=1)
                reward = reward + self.rw_slip * slip_pen
            self.prev_foot_xy_t = foot_xy.clone()

        fall = (torch.abs(roll) > ROLL_FAIL) | (torch.abs(pitch) > PITCH_FAIL) | (bz < BZ_FAIL)
        done = fall | (self.ep_step_t >= self.max_ep)
        reward = reward + fall.float() * r.get("term", -10.0)

        if done.any():
            self._reset_envs(env_mask=done)

        obs = self._build_obs_t()
        info = {"bz": bz, "roll": roll, "pitch": pitch}
        return obs, reward, done, info


# ────────────────────────────────────────────────────────────────────
# PPO loop.
# ────────────────────────────────────────────────────────────────────
def main():
    import torch
    import torch.nn as nn

    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=4096)
    p.add_argument("--iters", type=int, default=300)
    p.add_argument("--rollout", type=int, default=12)
    p.add_argument("--mjcf",
                   default=str(REPO / "projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml"))
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save",
                   default=str(REPO / "projects/policies/research/training/runs/gpu_h1_walk/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=512)
    p.add_argument("--save-every", type=int, default=0,
                   help="if >0, also dump a numbered policy_it<NNNN>.pt every N "
                        "iters (cheap, no ONNX) so a peak-then-decline run keeps "
                        "its best checkpoint; re-export the chosen one to ONNX after.")
    p.add_argument("--obs-history", type=int, default=1,
                   help="frame-stack the last K obs frames (closed-loop: lets a "
                        "memoryless MLP sense velocity/accel TRENDS so it can use "
                        "feedback instead of an open-loop clock). Deploy MUST mirror "
                        "via HUMANOID_WALK_OBS_HISTORY. K=1 = legacy single-frame.")
    p.add_argument("--overspeed", type=float, default=0.0,
                   help="quadratic penalty weight on max(0, vx - vx_target) -- the "
                        "anti-runaway / speed-regulating term (negative, e.g. -5). "
                        "Forces the policy to throttle when it senses it is too fast.")
    p.add_argument("--alive", type=float, default=1.0)
    p.add_argument("--upright", type=float, default=0.5)
    p.add_argument("--act", type=float, default=-0.005)
    p.add_argument("--act-rate", type=float, default=-0.01)
    p.add_argument("--term", type=float, default=-10.0)
    p.add_argument("--vel", type=float, default=2.0)
    p.add_argument("--vel-sigma", type=float, default=0.10)
    p.add_argument("--vel-l1", type=float, default=0.0)
    p.add_argument("--vx-target", type=float, default=VX_TARGET)
    p.add_argument("--lat", type=float, default=-0.5)
    p.add_argument("--yaw", type=float, default=-0.5)
    p.add_argument("--height", type=float, default=-10.0)
    p.add_argument("--z-ref", type=float, default=1.00,
                   help="target pelvis height during walk (H1)")
    p.add_argument("--gait-freq", type=float, default=GAIT_FREQ)
    p.add_argument("--gait-a-hip", type=float, default=GAIT_A_HIP)
    p.add_argument("--gait-a-knee", type=float, default=GAIT_A_KNEE)
    p.add_argument("--gait-a-lat", type=float, default=0.0)
    p.add_argument("--cp-gain", type=float, default=0.0)
    p.add_argument("--max-ep", type=int, default=MAX_EP)
    p.add_argument("--gait-a-ankle", type=float, default=0.0)
    p.add_argument("--seed-gait-pose", action="store_true")
    p.add_argument("--rw-sched", type=float, default=0.0)
    p.add_argument("--rw-slip", type=float, default=0.0)
    p.add_argument("--foot-z-contact", type=float, default=0.07)
    p.add_argument("--foot-z-swing", type=float, default=0.19)
    p.add_argument("--res-scale", type=float, default=RES_SCALE,
                   help="residual action scale rad. Deploy MUST set "
                        "NE1_ACT_SCALE to the same value.")
    p.add_argument("--nominal-hip", type=float, default=float(NOMINAL[0]))
    p.add_argument("--nominal-knee", type=float, default=float(NOMINAL[3]))
    p.add_argument("--nominal-ankle", type=float, default=float(NOMINAL[4]))
    p.add_argument("--gait-a-arm", type=float, default=0.0)
    p.add_argument("--gait-a-push", type=float, default=0.0)
    p.add_argument("--rest-start-frac", type=float, default=0.0)
    p.add_argument("--gait-model", default="", choices=["", "human"])
    p.add_argument("--gait-duty", type=float, default=0.6)
    p.add_argument("--gait-step-height", type=float, default=0.06)
    p.add_argument("--gait-pelvis-h", type=float, default=1.00)
    p.add_argument("--gait-bob", type=float, default=0.025)
    p.add_argument("--gait-x0", type=float, default=-0.02)
    p.add_argument("--gait-ramp-s", type=float, default=1.0)
    p.add_argument("--gait-elbow", type=float, default=0.15)
    p.add_argument("--gait-ankle-clear", type=float, default=0.08)
    p.add_argument("--gait-style", default="ik", choices=["ik", "winter"])
    p.add_argument("--kp", type=float, default=0.0,
                   help="override actuator kp (0 = keep the dump's 600). "
                        "Walking wants softer joints; deploy must set "
                        "OMNISIM_NEWTON_TARGET_KE to the same value.")
    p.add_argument("--kv", type=float, default=0.0,
                   help="override actuator kv (deploy: OMNISIM_NEWTON_TARGET_KD)")
    p.add_argument("--sim-dt", type=float, default=0.0)
    p.add_argument("--init-from", default=None)
    p.add_argument("--ent-coef", type=float, default=0.003)
    p.add_argument("--log-std-init", type=float, default=-2.0,
                   help="stiff-joint robots: the G1's -1.0 injects too much "
                        "exploration torque (stand-journey lesson)")
    p.add_argument("--log-std-clamp", type=float, default=None)
    p.add_argument("--dr-mass-scale", type=float, default=0.30)
    p.add_argument("--dr-friction-scale", type=float, default=0.50)
    p.add_argument("--dr-damping-scale", type=float, default=0.50)
    p.add_argument("--dr-actuator-kp-scale", type=float, default=0.20)
    p.add_argument("--dr-actuator-kv-scale", type=float, default=0.20)
    p.add_argument("--dr-gravity-scale", type=float, default=0.05)
    p.add_argument("--dr-push-prob", type=float, default=0.02)
    p.add_argument("--dr-push-vmax", type=float, default=1.0)
    p.add_argument("--dr-obs-noise", type=float, default=0.03)
    p.add_argument("--dr-action-latency-max", type=int, default=3)
    p.add_argument("--dr-init-q-band", type=float, default=0.10)
    p.add_argument("--dr-init-xy-band", type=float, default=0.05)
    p.add_argument("--dr-init-z-band", type=float, default=0.02)
    p.add_argument("--dr-init-tilt-band", type=float, default=0.05)
    p.add_argument("--dr-init-vel-band", type=float, default=0.1)
    p.add_argument("--dr-init-vx-bias", type=float, default=0.0)
    p.add_argument("--dr-seed", type=int, default=0)
    p.add_argument("--no-dr", action="store_true")
    p.add_argument("--hold-arms", action="store_true")
    args = p.parse_args()

    if not Path(args.mjcf).exists():
        raise SystemExit(
            f"MJCF not found: {args.mjcf}. Produce it via\n"
            f"  $env:OMNISIM_NEWTON_SAVE_MJCF = '_scratch/h1_newton_dump.mjcf.xml'\n"
            f"  powershell -File scripts/dev/run_humanoid_walk_deploy.ps1 -Robot h1 -Duration 16\n"
            f"  python projects/policies/research/training/import_newton_mjcf_h1.py")

    reward_cfg = dict(alive=args.alive, upright=args.upright,
                      act=args.act, act_rate=args.act_rate, term=args.term,
                      vel=args.vel, vel_sigma=args.vel_sigma, vel_l1=args.vel_l1,
                      overspeed=args.overspeed,
                      vx_target=args.vx_target,
                      lat=args.lat, yaw=args.yaw, height=args.height, z_ref=args.z_ref,
                      gait_freq=args.gait_freq, gait_a_hip=args.gait_a_hip,
                      gait_a_knee=args.gait_a_knee, gait_a_lat=args.gait_a_lat,
                      cp_gain=args.cp_gain, max_ep=args.max_ep,
                      gait_a_ankle=args.gait_a_ankle,
                      seed_gait=args.seed_gait_pose,
                      rw_sched=args.rw_sched, rw_slip=args.rw_slip,
                      foot_z_contact=args.foot_z_contact,
                      foot_z_swing=args.foot_z_swing,
                      res_scale=args.res_scale,
                      gait_a_arm=args.gait_a_arm,
                      gait_a_push=args.gait_a_push,
                      rest_start_frac=args.rest_start_frac,
                      kp=args.kp, kv=args.kv)
    nominal = NOMINAL.copy()
    for base in (0, 5):                  # H1 11-slot: right leg starts at 5 (no ankle-roll)
        nominal[base + 0] = args.nominal_hip
        nominal[base + 3] = args.nominal_knee
        nominal[base + 4] = args.nominal_ankle
    reward_cfg["nominal"] = nominal.tolist()
    if args.gait_model:
        reward_cfg["gait_model"] = args.gait_model
        reward_cfg["gait_params"] = dict(
            vx=args.vx_target, freq=args.gait_freq, duty=args.gait_duty,
            step_height=args.gait_step_height, pelvis_height=args.gait_pelvis_h,
            bob=args.gait_bob, sway=args.gait_a_lat, arm_swing=args.gait_a_arm,
            elbow_bend=args.gait_elbow, ankle_clear=args.gait_ankle_clear,
            x0=args.gait_x0, ramp_s=args.gait_ramp_s, style=args.gait_style)
    if args.no_dr:
        dr_cfg = {}
    else:
        dr_cfg = dict(
            mass_scale=args.dr_mass_scale,
            friction_scale=args.dr_friction_scale,
            damping_scale=args.dr_damping_scale,
            actuator_kp_scale=args.dr_actuator_kp_scale,
            actuator_kv_scale=args.dr_actuator_kv_scale,
            gravity_scale=args.dr_gravity_scale,
            push_prob=args.dr_push_prob,
            push_vmax=args.dr_push_vmax,
            obs_noise=args.dr_obs_noise,
            action_latency_max=args.dr_action_latency_max,
            init_q_band=args.dr_init_q_band,
            init_xy_band=args.dr_init_xy_band,
            init_z_band=args.dr_init_z_band,
            init_tilt_band=args.dr_init_tilt_band,
            init_vel_band=args.dr_init_vel_band,
            init_vx_bias=args.dr_init_vx_bias,
            seed=args.dr_seed,
        )
        print(f"[DR] {dr_cfg}")
    env = BatchedH1WalkEnv(args.envs, args.mjcf, reward_cfg=reward_cfg,
                            sim_dt=args.sim_dt, dr_cfg=dr_cfg,
                            hold_arms=args.hold_arms, obs_history=args.obs_history)
    if args.gait_model == "human":
        print(f"[gait] human model: nominal={np.round(env.nominal, 3).tolist()}")
    N = args.envs
    eff_obs = OBS_DIM * max(1, int(args.obs_history))     # frame-stacked input width
    if args.obs_history > 1:
        print(f"[obs] frame-stack K={args.obs_history} -> policy input {eff_obs}")

    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = nn.Sequential(nn.Linear(eff_obs, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(),
                                    nn.Linear(128, NJ))
            self.v = nn.Sequential(nn.Linear(eff_obs, 256), nn.Tanh(),
                                   nn.Linear(256, 128), nn.Tanh(),
                                   nn.Linear(128, 1))
            self.log_std = nn.Parameter(args.log_std_init * torch.ones(NJ))

        def forward(self, obs):
            return self.pi(obs), self.v(obs).squeeze(-1), self.log_std

    tdev = env.tdev
    torch.manual_seed(0)
    ac = AC().to(tdev)
    if args.init_from and Path(args.init_from).exists():
        ac.load_state_dict(torch.load(args.init_from, map_location=tdev))
        print(f"warm-start from {args.init_from}")
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)

    if args.eval:
        ac.eval()
        ac.load_state_dict(torch.load(args.save, map_location=tdev))
        obs = env.reset()
        survived = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        n_falls = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        dist = torch.zeros(env.n, device=tdev)
        vx_sum = torch.zeros(env.n, device=tdev)
        alive_steps = torch.zeros(env.n, device=tdev)
        for step in range(args.eval_steps):
            with torch.no_grad():
                mu, _, _ = ac(obs)
            alive = (first_fall == 0)
            obs, _, done, info = env.step(mu)
            step_alive = (alive & (~done)).float()
            vx = env.qvel_t[:, 0]
            dist += vx * DT * step_alive
            vx_sum += vx * step_alive
            alive_steps += step_alive
            survived += (~done).to(torch.int32)
            n_falls += done.to(torch.int32)
            newly = done & (first_fall == 0)
            first_fall = torch.where(newly, torch.full_like(first_fall, step + 1), first_fall)
        s = survived.cpu().numpy()
        ff = first_fall.cpu().numpy()
        nf = n_falls.cpu().numpy()
        d = dist.cpu().numpy()
        mean_vx = (vx_sum / torch.clamp(alive_steps, min=1.0)).cpu().numpy()
        never = (ff == 0)
        ff_fell = ff[~never]
        print(f"[gpu-eval] policy={args.save} envs={env.n} steps={args.eval_steps}")
        print(f"  survival steps: mean={s.mean():.1f}  "
              f"median={np.median(s):.0f}  max={s.max()}  "
              f"frac_full={(s >= args.eval_steps).mean():.2f}")
        print(f"  FIRST-FALL step: mean={ff_fell.mean() if ff_fell.size else -1:.1f}  "
              f"never_fell_frac={never.mean():.2f}  mean_n_falls={nf.mean():.1f}")
        print(f"  WALK fwd dist (m, before first fall): mean={d.mean():.2f}  "
              f"median={np.median(d):.2f}  max={d.max():.2f}")
        print(f"  WALK fwd speed (m/s while alive): mean={mean_vx.mean():.3f}  "
              f"target={env.vx_target:.2f}")
        return

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)

    rollout = args.rollout
    obs = env.reset()
    total_steps = 0
    t0 = time.time()

    obs_buf = torch.zeros(rollout, N, eff_obs, device=tdev)
    act_buf = torch.zeros(rollout, N, NJ, device=tdev)
    logp_buf = torch.zeros(rollout, N, device=tdev)
    rew_buf = torch.zeros(rollout, N, device=tdev)
    done_buf = torch.zeros(rollout, N, device=tdev)
    val_buf = torch.zeros(rollout, N, device=tdev)

    for it in range(1, args.iters + 1):
        for k in range(rollout):
            with torch.no_grad():
                mu, v, log_std = ac(obs)
                std = log_std.exp()
                dist = torch.distributions.Normal(mu, std)
                a = dist.sample()
                logp = dist.log_prob(a).sum(-1)
            obs_buf[k] = obs
            act_buf[k] = a
            logp_buf[k] = logp
            val_buf[k] = v
            obs, r, done, _ = env.step(a)
            rew_buf[k] = r
            done_buf[k] = done.float()
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

        obs_flat = obs_buf.reshape(-1, eff_obs)
        act_flat = act_buf.reshape(-1, NJ)
        logp_flat = logp_buf.reshape(-1)
        adv_flat = adv.reshape(-1)
        ret_flat = ret.reshape(-1)

        clip_eps = 0.2
        for _epoch in range(4):
            mu, v, log_std = ac(obs_flat)
            std = log_std.exp()
            dist = torch.distributions.Normal(mu, std)
            new_logp = dist.log_prob(act_flat).sum(-1)
            ratio = (new_logp - logp_flat).exp()
            surr1 = ratio * adv_flat
            surr2 = torch.clamp(ratio, 1 - clip_eps, 1 + clip_eps) * adv_flat
            pi_loss = -torch.min(surr1, surr2).mean()
            v_loss = ((v - ret_flat) ** 2).mean()
            ent = dist.entropy().sum(-1).mean()
            loss = pi_loss + 0.5 * v_loss - args.ent_coef * ent
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5)
            opt.step()
            if args.log_std_clamp is not None:
                with torch.no_grad():
                    ac.log_std.clamp_(max=args.log_std_clamp)

        if it % 5 == 0 or it == 1:
            ep_rps = rew_buf.mean().item()
            mean_v = val_buf.mean().item()
            fps = total_steps / max(time.time() - t0, 1e-6)
            vx_now = env.qvel_t[:, 0].mean().item()
            print(f"it {it:4d}  ep_rew/step~{ep_rps:+.3f}  "
                  f"meanV {mean_v:+.2f}  vx~{vx_now:+.2f}  steps {total_steps:,}  "
                  f"{fps:,.0f} env-steps/s")

        if args.save_every > 0 and it % args.save_every == 0 and it < args.iters:
            ckpt = Path(args.save).with_name(
                Path(args.save).stem + f"_it{it:04d}.pt")
            torch.save(ac.state_dict(), ckpt)
            print(f"  [ckpt] saved {ckpt}", flush=True)

    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total_steps:,} steps in "
          f"{time.time() - t0:.1f}s)")

    # ONNX export — CLAMP, not tanh (the env's exact squashing).
    onnx_path = Path(args.save).with_suffix(".onnx")

    class DeployPolicy(torch.nn.Module):
        def __init__(self, pi):
            super().__init__()
            self.pi = pi

        def forward(self, obs):
            return torch.clamp(self.pi(obs), -1.0, 1.0)

    cpu_ac = AC()
    cpu_ac.load_state_dict({k: v.cpu() for k, v in ac.state_dict().items()})
    wrapped = DeployPolicy(cpu_ac.pi)
    wrapped.eval()
    dummy = torch.zeros(1, eff_obs, dtype=torch.float32)
    try:
        import io
        import contextlib
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            torch.onnx.export(
                wrapped, dummy, str(onnx_path),
                input_names=["obs"], output_names=["action"],
                dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
                opset_version=17,
            )
        sys.stdout.write(buf.getvalue().encode("ascii", "replace").decode("ascii"))
        print(f"exported ONNX -> {onnx_path}")
    except Exception as e:
        print(f"ONNX export failed (non-fatal): {e}")


if __name__ == "__main__":
    main()
