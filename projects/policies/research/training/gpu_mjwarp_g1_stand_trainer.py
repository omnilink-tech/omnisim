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

"""GPU mujoco_warp trainer for the G1 standing policy.

THIS FILE IS THE CANONICAL TEMPLATE for the OmniSim sim-to-deploy
RL recipe. Copy it as the starting point for any new robot or task
that deploys to OmniSim Newton — see:

    docs/developer/sim-to-deploy-rl-recipe.md  (general recipe)
    docs/developer/g1-stand-rl-playbook.md     (case study + journey)

Three things make this trainer reliable for sim-to-deploy:

  1. Heavy domain randomization. Body mass / friction / damping /
     actuator kp / kv / gravity / pushes / obs noise / action latency
     / initial pose are all randomized aggressively. The deploy
     wrapper's specific quirks land inside the training distribution
     instead of needing to be matched in code.

  2. Five stacked GPU speedups so heavy-DR runs are cheap (3-4 min
     for 30 M-step PPO on an RTX 5070, 132 k env-steps/s):
        - Actor + rollout buffers on cuda (requires CUDA torch).
        - wp.to_torch zero-copy views of mw_d.qpos/qvel/ctrl, all env
          computations in torch on cuda, zero CPU↔GPU traffic.
        - CUDA graph capture of the SUBSTEPS physics loop.
        - 4096 envs (vs the typical 2048) for better GPU saturation.
        - SUBSTEPS=4 × 4 ms physics dt (vs 8 × 2 ms) — same env-step
          semantics, half the physics work.

  3. Train against the EXACT MJCF Newton builds. Get it via:
        OMNISIM_NEWTON_SAVE_MJCF=<path>
     on an OmniSim launch with the deploy world, then rename
     anonymous joints/bodies via projects/policies/research/training/import_newton_mjcf.py.

What's G1-specific (change for your robot): LEGS_JOINTS, NJ, NOMINAL,
JOINT_LIMITS_*, _L_AP / _R_AP / _L_AR / _R_AR (ankle indices for the
baseline PD), and `_baseline_targets_t`. Everything else is reusable.

Optional full-body mode (--hold-arms): train the same 13-DOF legs+waist
policy against the FULL 23-DOF G1 (arms present in the MJCF, pinned at
ARM_NOMINAL, never policy-controlled). The arm mass (+6.1 kg, ~18% of
body) becomes a passive balance load the leg policy must compensate. The
resulting policy is drop-in for g1_stand_arms_deploy. Use with the
full-body MJCF g1_full.mjcf.xml (dump + rename via
import_newton_mjcf_g1_full.py). The legs-only standing policy faceplants
the full body in <1 s because it never saw the arm mass.

Result for G1 standing: ~98 % survival in the mujoco_warp trainer
(commit 806753dc). NOTE — this is the TRAINER result, not deploy. The
OmniSim Newton DEPLOY currently stands to t ≈ 1.55 s then loses balance
(characterized 2026-05-29 limitation; NOT "stands forever"). Deploy
needs OMNISIM_NEWTON_STATICS=1 + OMNISIM_NEWTON_SUBSTEPS=4. See
docs/developer/rl-current-state.md and the "Floor-contact regression"
note in g1-stand-rl-playbook.md before trusting any "44 s" figure.

Usage:
    python projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py \\
        --envs 4096 --iters 600 --rollout 12 \\
        --mjcf projects/robots/unitree/g1/urdf/g1_legs.mjcf.xml \\
        --save projects/policies/research/training/runs/gpu_g1_stand_robust/policy.pt

Exports both a .pt (PyTorch state_dict) and .onnx for the OmniSim
deploy controller to consume.

Deploy env vars (set on the OmniSim launch):
    OMNISIM_URDF_USE_INERTIA=1       # mandatory for any URDF robot
    OMNISIM_NEWTON_FORCE_MUJOCO=1    # pick SolverMuJoCo (not XPBD)
    OMNISIM_NEWTON_MJWARP=1          # GPU mujoco_warp under SolverMuJoCo
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))


# ────────────────────────────────────────────────────────────────────
# G1 stand layout constants — must match projects/policies/research/envs/g1_stand_env.py
# ────────────────────────────────────────────────────────────────────
NJ = 13            # 6 left + 6 right leg joints + 1 waist
QPOS_J0 = 7        # leg joints start in qpos (after 7-dim free joint)
QVEL_J0 = 6        # leg joints start in qvel (after 6-dim free joint dofs)
# Parity mode (G1_PARITY_OBS=1): a reduced POSITION-ONLY obs for the train==deploy
# closed-loop validation -- drops base lin/ang velocity AND joint velocity, the two
# obs terms that are hardest to reproduce identically in deploy (finite-diff qd,
# world-vs-body frame). What's left (q-NOMINAL, proj_gravity, last_action) is
# trivially identical on both sides, so a non-falling policy transfers bit-for-bit.
_PARITY_OBS = bool(os.environ.get("G1_PARITY_OBS"))
# Parity obs includes base ANGULAR velocity (tipping-rate -> durability) but NOT
# joint velocity or base LINEAR velocity. Base ang-vel is reproducible in deploy
# (getVelocity -> R^T body frame, the unified obs), unlike finite-diff joint qd.
OBS_DIM = (32 if _PARITY_OBS         # q(13)+proj_g(3)+ang_vel(3)+last_action(13)
           else 48)    # lin(3)+ang(3)+proj_g(3)+q(13)+qd(13)+last_action(13)
RES_SCALE = float(os.environ.get("G1_RES_SCALE", "0.3"))  # ±rad residual; raise (env) to let the policy STEP for big pushes
DT = 0.016         # env-step dt = OmniSim basicTimeStep 16 ms
SUBSTEPS = 4       # 4 substeps * 4 ms physics timestep = 16 ms env-step
                   # (was 8 substeps * 2 ms; mujoco_warp is stable for
                   # human-scale dynamics at 4 ms, halves physics work)
PHYS_DT = 0.004    # physics timestep — written into MJCF at load time

# Joint name order matches g1_robot_spec.JOINT_NAMES (legs subset).
LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)

# Nominal stand pose for legs+waist. DEEPER squat than g1_robot_spec.NOMINAL_POSE
# (hip -0.30 / knee 0.52 vs -0.20 / 0.42): the deploy model's CoM_x sits 5 mm AHEAD
# of its foot front at the shallow pose (the OmniSim URDF importer places the foot
# ~35 mm further back than newton's native add_urdf), so the shallow pose TIPS
# FORWARD at ~1.3 s under ANY control -- the real reason G1 deploy fell (verified in
# plain mujoco, NOT a sim2sim gap). The deeper squat recentres the CoM behind the
# foot front -> sagittally stable; the policy then only has to hold lateral balance.
# MUST stay in sync with g1_stand_deploy.py NOMINAL_LEGS.
NOMINAL = np.array([
    -0.30, +0.00, +0.00, +0.52, -0.23, +0.00,    # left leg
    -0.30, +0.00, +0.00, +0.52, -0.23, +0.00,    # right leg
    +0.00,                                        # waist
], dtype=np.float32)
assert NOMINAL.shape == (NJ,)

# Analytic ankle balance PD baseline. The residual policy adds on top. Env-tunable
# so it can be DISABLED (set all = 0): the analytic roll PD DESTABILISES the deploy
# (its finite-diff roll_rate kicks ankle_roll at handover -> the deploy fell in ROLL
# even though the deeper-squat NOMINAL is statically stable). Training with the PD
# OFF makes the policy learn ALL balance on top of pure NOMINAL, matching a deploy
# run with G1_BAL_*=0. Keep train and deploy gains in sync.
import os as _balos
KP_ANKLE_PITCH = float(_balos.environ.get("G1_TRAIN_BAL_KP_P", "-1.5"))
KD_ANKLE_PITCH = float(_balos.environ.get("G1_TRAIN_BAL_KD_P", "-0.2"))
KP_ANKLE_ROLL = float(_balos.environ.get("G1_TRAIN_BAL_KP_R", "-1.5"))
KD_ANKLE_ROLL = float(_balos.environ.get("G1_TRAIN_BAL_KD_R", "-0.2"))
BAL_CLAMP = float(_balos.environ.get("G1_TRAIN_BAL_CLAMP", "0.2"))

# Joint indices into LEGS_JOINTS (i.e. controller order).
_L_AP = LEGS_JOINTS.index("left_ankle_pitch_joint")
_R_AP = LEGS_JOINTS.index("right_ankle_pitch_joint")
_L_AR = LEGS_JOINTS.index("left_ankle_roll_joint")
_R_AR = LEGS_JOINTS.index("right_ankle_roll_joint")

# Episode termination.
SPAWN_Z = 0.78
BZ_FAIL = 0.45
ROLL_FAIL = 0.8
PITCH_FAIL = 0.8
MAX_EP = 500

# Joint position limits (URDF <limit> values).
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

# ── Optional full-body mode (--hold-arms) ──────────────────────────
# The 10 arm joints. The policy NEVER controls these (action stays
# 13-dim, obs stays 48-dim — deploy-compatible with g1_stand_deploy).
# When --hold-arms is set they are simply present in the MJCF and pinned
# to ARM_NOMINAL every step, so the policy learns to balance the +6.1 kg
# of arm mass (17.9% of body) that the legs-only body lacks. Train this
# against the full-body MJCF (projects/robots/unitree/g1/urdf/g1_full.mjcf.xml).
ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
)
# Arms hanging straight down (elbow=0), small ±0.2 shoulder-roll splay —
# matches g1_stand_arms_deploy.py _ARM_DOWN exactly. CoM-neutral.
ARM_NOMINAL = np.array([
    +0.00, +0.20, +0.00, +0.00, +0.00,   # left  arm
    +0.00, -0.20, +0.00, +0.00, +0.00,   # right arm
], dtype=np.float32)
assert ARM_NOMINAL.shape == (len(ARM_JOINTS),)


# ────────────────────────────────────────────────────────────────────
# Small numpy helpers (same as Spot trainer).
# ────────────────────────────────────────────────────────────────────
def quat_to_rp(q):
    """q: [N, 4] (w, x, y, z) -> roll, pitch (each [N,])."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    return roll, pitch


def proj_gravity(q):
    """Body-frame gravity unit vector, [N, 3]."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    gx = -2 * (x * z - w * y)
    gy = -2 * (y * z + w * x)
    gz = -(1 - 2 * (x * x + y * y))
    return np.stack([gx, gy, gz], axis=1).astype(np.float32)


# ────────────────────────────────────────────────────────────────────
# Batched env — N parallel G1 standing instances on GPU mujoco_warp.
# ────────────────────────────────────────────────────────────────────
class BatchedG1StandEnv:
    """Standing-task batched env. No gait, no IK; baseline is NOMINAL +
    ankle balance PD. Residual policy adds ±RES_SCALE rad on top.
    """

    def __init__(self, n, mjcf, device="cuda:0", reward_cfg=None, sim_dt=0.0,
                 dr_cfg=None, hold_arms=False, wave_ref=""):
        import warp as wp
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mjw = wp, mjw
        self.n = n
        # Full-body mode: arms present in the MJCF, pinned at nominal,
        # not controlled by the policy. See ARM_JOINTS / ARM_NOMINAL.
        self.hold_arms = hold_arms
        # stand+WAVE mode: drive the arms along a ghost-replay reference (looped) instead of
        # pinning at nominal, so the legs policy learns to balance THROUGH the waving arm.
        self.wave_ref_path = wave_ref or ""
        self.wave_arms = bool(self.wave_ref_path)
        self.device = wp.get_device(device)
        # Domain randomization config. Closes the MuJoCo->Newton sim2sim
        # gap by training over a band of physics params + perturbations
        # so the policy is robust to the slight differences between the
        # MJ solver (used by mujoco_warp) and Newton's XPBD solver.
        self.dr = dr_cfg or {}

        self.mjm = mujoco.MjModel.from_xml_path(mjcf)
        # Force physics dt to PHYS_DT (4 ms). With SUBSTEPS=4 this gives a
        # 16-ms env-step matching OmniSim basicTimeStep, but halves the
        # number of physics ticks per env-step vs the previous
        # SUBSTEPS=8 × 2 ms config.
        self.mjm.opt.timestep = float(sim_dt) if sim_dt and sim_dt > 0 else PHYS_DT

        # Parity: force the MJCF position/velocity actuator gains to the DEPLOY's
        # OMNISIM_NEWTON_TARGET_KE/KD (default 400/60) so the trainer's PD matches
        # the binary's exactly. The MJCF ships kp=20/kv=3 (the historical 20x
        # sim2deploy gain mismatch) -- without this the closed-loop trajectories
        # cannot match. Position actuator: gainprm[0]=ke, biasprm[1]=-ke; velocity:
        # gainprm[0]=kd, biasprm[2]=-kd (matches build_g1_mjcf's layout).
        _fke = os.environ.get("G1_FORCE_KE")
        _fkd = os.environ.get("G1_FORCE_KD")
        if _fke or _fkd:
            for ai in range(self.mjm.nu):
                bp = self.mjm.actuator_biasprm[ai]
                gp = self.mjm.actuator_gainprm[ai]
                if abs(bp[1]) > 1e-6 and _fke:      # position actuator
                    gp[0] = float(_fke); bp[1] = -float(_fke)
                elif abs(bp[2]) > 1e-6 and _fkd:    # velocity actuator
                    gp[0] = float(_fkd); bp[2] = -float(_fkd)

        # Domain randomization on the MJCF model BEFORE putting it on
        # the device. mujoco_warp doesn't support per-env model params
        # cheaply, so each TRAINING RUN samples one point in the
        # manifold. Run-to-run variation + per-step perturbations
        # together produce a policy robust to a band of physics.
        rng = np.random.default_rng(self.dr.get("seed", 0))
        mass_scale_band = self.dr.get("mass_scale", 0.0)   # ±fraction
        # CoM-offset DR (the single most important standing-transfer knob per HuB/HoST):
        # jitter each body's local CoM (body_ipos) by ±com_offset metres. A standing
        # humanoid is an inverted pendulum balancing right at the foot-support edge, so a
        # few-cm CoM error is exactly what tips it -- and the deploy runtime's link-COM
        # convention is a known mismatch (body_ipos differs by up to 0.154 m unless
        # OMNISIM_NEWTON_USE_LINK_COM=1; the deploy MUST set that for nominal parity, and
        # this DR makes the policy robust to the residual per-link CoM uncertainty).
        com_offset_band = self.dr.get("com_offset", 0.0)   # ±metres on body_ipos
        fric_band = self.dr.get("friction_scale", 0.0)
        damp_band = self.dr.get("damping_scale", 0.0)
        actuator_kp_band = self.dr.get("actuator_kp_scale", 0.0)
        actuator_kv_band = self.dr.get("actuator_kv_scale", 0.0)
        gravity_band = self.dr.get("gravity_scale", 0.0)
        if mass_scale_band > 0:
            # Per-body scale, not global — Newton's wrapper applies
            # body-by-body inertia conversion that can drift slightly
            # per link, so the policy needs to be robust to per-body
            # mass distribution, not just total mass.
            scales = rng.uniform(1.0 - mass_scale_band, 1.0 + mass_scale_band,
                                 size=self.mjm.body_mass.shape).astype(np.float32)
            self.mjm.body_mass[:] *= scales
            self.mjm.body_inertia[:] *= scales[:, None]
        if com_offset_band > 0:
            # Per-body, per-axis uniform CoM shift on the local inertial origin.
            # Skip the world body (index 0). body_ipos is (nbody, 3).
            coff = rng.uniform(-com_offset_band, com_offset_band,
                               size=self.mjm.body_ipos.shape).astype(np.float32)
            coff[0] = 0.0
            self.mjm.body_ipos[:] += coff
        if fric_band > 0:
            fs = float(rng.uniform(1.0 - fric_band, 1.0 + fric_band))
            self.mjm.geom_friction[:, 0] *= fs
        if damp_band > 0:
            # Per-DOF damping jitter so different joints feel different
            # damping (Newton's per-joint XPBD damping is per-joint).
            ds = rng.uniform(1.0 - damp_band, 1.0 + damp_band,
                             size=self.mjm.dof_damping.shape).astype(np.float32)
            self.mjm.dof_damping[:] *= ds
        if actuator_kp_band > 0 or actuator_kv_band > 0:
            # Position actuators have gainprm[0]=kp and biasprm[1]=-kp.
            # Velocity actuators have gainprm[0]=kv and biasprm[2]=-kv.
            # Detect by which biasprm slot is non-zero.
            for ai in range(self.mjm.nu):
                gp = self.mjm.actuator_gainprm[ai]
                bp = self.mjm.actuator_biasprm[ai]
                if abs(bp[1]) > 1e-6 and actuator_kp_band > 0:  # position
                    s = float(rng.uniform(1.0 - actuator_kp_band,
                                          1.0 + actuator_kp_band))
                    gp[0] *= s
                    bp[1] *= s
                elif abs(bp[2]) > 1e-6 and actuator_kv_band > 0:  # velocity
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
            # njmax/nconmax: mujoco_warp auto-estimates these too small for the
            # G1 foot-ground contact ("nefc overflow - increase njmax to ~80"),
            # which DROPS constraints -> incomplete foot grip -> the model is only
            # marginally stable (falls ~every 1 s) whereas plain mj_step (which
            # sizes the constraint buffer correctly) holds the SAME pose 15 s.
            # Generous fixed caps remove the overflow. Env-overridable.
            import os as _njos
            _njmax = int(_njos.environ.get("G1_NJMAX", "256"))
            _nconmax = int(_njos.environ.get("G1_NCONMAX", "256"))
            self.mw_d = mjw.put_data(self.mjm, mjd, nworld=n,
                                     njmax=_njmax, nconmax=_nconmax)

        # Map controller order -> MJCF qpos / qvel / ctrl indices.
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
            pos_name = f"{jn}_pos"
            vel_name = f"{jn}_vel"
            pos_id = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, pos_name)
            vel_id = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, vel_name)
            if pos_id < 0 or vel_id < 0:
                raise RuntimeError(
                    f"actuators {pos_name}/{vel_name} not in MJCF — see "
                    f"projects/policies/research/training/build_g1_mjcf.py")
            self.controller_to_ctrl_pos[i] = pos_id
            self.controller_to_ctrl_vel[i] = vel_id

        # Full-body mode: map the arm joints' ctrl-pos actuators so we can
        # pin them each step, and remember their qpos slots to seed them.
        self.arm_ctrl_pos = np.zeros(len(ARM_JOINTS), dtype=np.int32)
        self.arm_qpos = np.zeros(len(ARM_JOINTS), dtype=np.int32)
        if self.hold_arms:
            for i, jn in enumerate(ARM_JOINTS):
                jid = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid < 0:
                    raise RuntimeError(
                        f"--hold-arms: arm joint {jn} not in MJCF {mjcf}. "
                        f"Use the full-body MJCF (g1_full.mjcf.xml).")
                self.arm_qpos[i] = self.mjm.jnt_qposadr[jid]
                pos_id = mujoco.mj_name2id(
                    self.mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_pos")
                if pos_id < 0:
                    raise RuntimeError(f"--hold-arms: actuator {jn}_pos missing")
                self.arm_ctrl_pos[i] = pos_id

        self.r = reward_cfg or {}

        # Seed pose: at SPAWN_Z, identity orientation, NOMINAL leg pose.
        self.seed_qpos = mjd.qpos.copy().astype(np.float32)
        self.seed_qpos[0:3] = [0.0, 0.0, SPAWN_Z]
        self.seed_qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for i, jn in enumerate(LEGS_JOINTS):
            self.seed_qpos[self.controller_to_qpos[i]] = NOMINAL[i]
        if self.hold_arms:
            for i in range(len(ARM_JOINTS)):
                self.seed_qpos[self.arm_qpos[i]] = ARM_NOMINAL[i]
        self.nq = self.mjm.nq
        self.nv = self.mjm.nv
        self.nu = self.mjm.nu

        # ─── GPU-native env state ───
        # torch.device matching the warp device so we share GPU memory.
        self.tdev = torch.device("cuda:0" if "cuda" in str(self.device).lower()
                                 else "cpu")
        # Torch views of mujoco_warp's qpos/qvel/ctrl arrays. wp.to_torch
        # shares GPU memory zero-copy — writes propagate. This is the
        # core trick: physics runs on the warp side, policy runs on the
        # torch side, both see the same GPU buffer.
        self.qpos_t = wp.to_torch(self.mw_d.qpos).view(n, self.nq)
        self.qvel_t = wp.to_torch(self.mw_d.qvel).view(n, self.nv)
        self.ctrl_t = wp.to_torch(self.mw_d.ctrl).view(n, self.nu)

        # Constants moved to GPU once at init.
        self.nominal_t = torch.tensor(NOMINAL, dtype=torch.float32, device=self.tdev)
        self.jl_lo_t = torch.tensor(JOINT_LIMITS_LO, dtype=torch.float32, device=self.tdev)
        self.jl_hi_t = torch.tensor(JOINT_LIMITS_HI, dtype=torch.float32, device=self.tdev)
        self.qpos_idx_t = torch.tensor(self.controller_to_qpos, dtype=torch.long, device=self.tdev)
        self.qvel_idx_t = torch.tensor(self.controller_to_qvel, dtype=torch.long, device=self.tdev)
        self.ctrl_pos_idx_t = torch.tensor(self.controller_to_ctrl_pos, dtype=torch.long, device=self.tdev)
        self.seed_qpos_t = torch.tensor(self.seed_qpos, dtype=torch.float32, device=self.tdev)
        if self.hold_arms:
            self.arm_ctrl_pos_idx_t = torch.tensor(
                self.arm_ctrl_pos, dtype=torch.long, device=self.tdev)
            # (n, 10) target block, constant — built once.
            self.arm_targets_t = torch.tensor(
                ARM_NOMINAL, dtype=torch.float32, device=self.tdev
            ).unsqueeze(0).expand(n, -1).contiguous()
        if self.wave_arms:
            # Load the wave's arm trajectory (N_frames, 10) in ARM_JOINTS order, and give each
            # env a FIXED random phase offset so all wave phases are covered across the batch
            # every step (the policy learns to balance at every point of the wave).
            import csv as _csv
            with open(self.wave_ref_path) as _f:
                _rows = list(_csv.DictReader(_f))
            wave = np.array([[float(r[jn]) for jn in ARM_JOINTS] for r in _rows],
                            dtype=np.float32)
            self.wave_n = int(wave.shape[0])
            self.wave_ref_t = torch.tensor(wave, dtype=torch.float32, device=self.tdev)  # (N,10)
            g = torch.Generator(device=self.tdev)
            g.manual_seed(0)
            self.wave_phase0_t = torch.randint(0, self.wave_n, (n,), generator=g,
                                               dtype=torch.long, device=self.tdev)
            self.arm_qpos_idx_t = torch.tensor(self.arm_qpos, dtype=torch.long, device=self.tdev)
            print(f"[stand+wave] wave ref {self.wave_n} frames, {wave.shape[1]} arm joints")

        self.ep_step_t = torch.zeros(n, dtype=torch.int32, device=self.tdev)
        self.last_action_t = torch.zeros(n, NJ, dtype=torch.float32, device=self.tdev)
        self.prev_roll_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.prev_pitch_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)

        # Per-env action-latency buffer (now on GPU).
        self.max_latency_ticks = int(self.dr.get("action_latency_max", 0))
        self.action_buffer_t = torch.zeros(
            n, max(1, self.max_latency_ticks + 1), NJ,
            dtype=torch.float32, device=self.tdev)
        self.action_delay_t = torch.zeros(n, dtype=torch.long, device=self.tdev)

        # Reward + push knobs cached as floats for fast hot-path access.
        self._push_p = float(self.dr.get("push_prob", 0.0))
        self._push_vmax = float(self.dr.get("push_vmax", 0.0))
        self._obs_noise = float(self.dr.get("obs_noise", 0.0))
        self._init_q_band = float(self.dr.get("init_q_band", 0.05))
        self._init_xy_band = float(self.dr.get("init_xy_band", 0.03))
        self._init_z_band = float(self.dr.get("init_z_band", 0.0))

        # CUDA graph capture for the substep physics loop. Captured once
        # at init; replayed each env-step via wp.capture_launch(). Cuts
        # the ~200 kernel-launch overhead mujoco_warp pays per step
        # times SUBSTEPS times per-env-step times.
        self._cuda_graph = None
        self._try_capture_graph()

        self._reset_all()

    def _try_capture_graph(self):
        """Capture the SUBSTEPS-step physics loop into a CUDA graph.
        Disable via OMNISIM_NEWTON_NO_GRAPH=1 if it ever misbehaves.
        """
        import os as _os
        if _os.environ.get("OMNISIM_NEWTON_NO_GRAPH"):
            return
        try:
            with self.wp.ScopedDevice(self.device):
                # Warm up once before capture so JIT / autotune don't
                # land inside the graph.
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
        """Reset a subset of envs (or all if env_mask is None).
        Writes new qpos/qvel directly into the GPU buffers via the
        torch views — zero CPU↔GPU transfer.
        """
        if env_mask is None:
            idx = torch.arange(self.n, device=self.tdev)
        else:
            idx = torch.nonzero(env_mask, as_tuple=False).squeeze(-1)
            if idx.numel() == 0:
                return
        m = idx.shape[0]

        # Tile seed pose for the rows being reset.
        self.qpos_t[idx] = self.seed_qpos_t.unsqueeze(0).expand(m, -1)
        self.qvel_t[idx] = 0.0

        # stand+WAVE: seed each reset env's arms at ITS wave phase so the episode begins with
        # the arms already at the right wave pose (no instant arm jump at reset).
        if self.wave_arms:
            arm_seed = self.wave_ref_t[self.wave_phase0_t[idx]]                 # (m, 10)
            self.qpos_t[idx.unsqueeze(1), self.arm_qpos_idx_t.unsqueeze(0)] = arm_seed

        # Per-env spawn jitter.
        if self._init_q_band > 0:
            jitter = (torch.rand(m, NJ, device=self.tdev) * 2 - 1) * self._init_q_band
            # Scatter into the right qpos slots.
            base_idx = idx.unsqueeze(1).expand(-1, NJ)               # (m, NJ)
            col_idx = self.qpos_idx_t.unsqueeze(0).expand(m, -1)     # (m, NJ)
            self.qpos_t[base_idx, col_idx] += jitter
        if self._init_xy_band > 0:
            self.qpos_t[idx, 0] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_xy_band
            self.qpos_t[idx, 1] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_xy_band
        if self._init_z_band > 0:
            self.qpos_t[idx, 2] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_z_band

        # Base-tilt jitter (random roll+pitch on the free-joint quaternion) so the
        # policy learns to recover a TILTED start -- the deploy spawns straight-legged
        # and FOLDS into the squat, handing the policy a forward-pitched (+ slightly
        # moving) base. Without this the handover state is out-of-distribution and the
        # policy tips. quat (w,x,y,z) for roll(x) r then pitch(y) p:
        # (cr*cp, sr*cp, cr*sp, sr*sp), half-angles.
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
        # Base-velocity jitter (lin m/s + ang rad/s) so the handover's residual
        # motion is in-distribution.
        vband = float(self.dr.get("init_vel_band", 0.0))
        if vband > 0:
            self.qvel_t[idx, 0:6] += (torch.rand(m, 6, device=self.tdev) * 2 - 1) * vband

        self.ep_step_t[idx] = 0
        self.last_action_t[idx] = 0.0
        self.prev_roll_t[idx] = 0.0
        self.prev_pitch_t[idx] = 0.0
        self.action_buffer_t[idx] = 0.0
        if self.max_latency_ticks > 0:
            self.action_delay_t[idx] = torch.randint(
                0, self.max_latency_ticks + 1, (m,),
                dtype=torch.long, device=self.tdev)
        else:
            self.action_delay_t[idx] = 0

        with self.wp.ScopedDevice(self.device):
            # Throughput patch (opt-in, default behaviour unchanged): the reset
            # only needs kinematics re-derived, not a full forward-dynamics pass.
            # mjw.kinematics is a bit-identical drop-in for the seeded state and
            # ~2x faster on the hot per-step done-env reset. Gated by
            # OMNISIM_FAST_RESET so it never changes anyone else's default runs.
            if _balos.environ.get("OMNISIM_FAST_RESET", "0") != "0":
                self.mjw.kinematics(self.mw_m, self.mw_d)
            else:
                self.mjw.forward(self.mw_m, self.mw_d)

    def _reset_all(self):
        self._reset_envs(env_mask=None)

    def _build_obs_t(self):
        """All-GPU obs vector. Reads zero-copy torch views of mw_d, no
        CPU↔GPU transfer.
        """
        qp = self.qpos_t
        qv = self.qvel_t
        vlin = qv[:, 0:3]
        vang = qv[:, 3:6]
        # proj_gravity from quaternion (matches the numpy version
        # bit-for-bit since both compute the same closed-form).
        w = qp[:, 3]; x = qp[:, 4]; y = qp[:, 5]; z = qp[:, 6]
        gx = -2 * (x * z - w * y)
        gy = -2 * (y * z + w * x)
        gz = -(1 - 2 * (x * x + y * y))
        pg = torch.stack([gx, gy, gz], dim=1)
        # Joint q/qd in controller order, q minus nominal so policy sees
        # a centered signal.
        q = qp.index_select(1, self.qpos_idx_t) - self.nominal_t.unsqueeze(0)
        qd = qv.index_select(1, self.qvel_idx_t)
        if _PARITY_OBS:
            # parity obs (32): q-NOMINAL, proj_gravity, base ang-vel, last_action.
            obs = torch.cat([q, pg, vang, self.last_action_t], dim=1)
        else:
            obs = torch.cat([vlin, vang, pg, q, qd, self.last_action_t], dim=1)
        if self._obs_noise > 0:
            obs = obs + torch.randn_like(obs) * self._obs_noise
        obs = torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = torch.clamp(obs, -10.0, 10.0)
        return obs

    def _baseline_targets_t(self):
        """NOMINAL + ankle balance PD, all on GPU. Returns (n, NJ)."""
        qp = self.qpos_t
        w = qp[:, 3]; x = qp[:, 4]; y = qp[:, 5]; z = qp[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sinp = torch.clamp(2 * (w * y - z * x), -1.0, 1.0)
        pitch = torch.asin(sinp)
        roll_rate = (roll - self.prev_roll_t) / DT
        pitch_rate = (pitch - self.prev_pitch_t) / DT
        self.prev_roll_t = roll
        self.prev_pitch_t = pitch

        ap = torch.clamp(KP_ANKLE_PITCH * pitch + KD_ANKLE_PITCH * pitch_rate,
                         -BAL_CLAMP, BAL_CLAMP)
        ar = torch.clamp(KP_ANKLE_ROLL * roll + KD_ANKLE_ROLL * roll_rate,
                         -BAL_CLAMP, BAL_CLAMP)
        targets = self.nominal_t.unsqueeze(0).expand(self.n, -1).contiguous()
        targets[:, _L_AP] = targets[:, _L_AP] + ap
        targets[:, _R_AP] = targets[:, _R_AP] + ap
        targets[:, _L_AR] = targets[:, _L_AR] + ar
        targets[:, _R_AR] = targets[:, _R_AR] + ar
        return targets, roll, pitch

    def reset(self):
        self._reset_all()
        return self._build_obs_t()

    # ── GPU-native step: action in/out are torch tensors on cuda. ──
    def step(self, action_t):
        action_t = torch.clamp(action_t, -1.0, 1.0)

        # Action-latency buffer: roll, store new at slot 0, pull each
        # env's effective action from its assigned delay slot.
        if self.max_latency_ticks > 0:
            self.action_buffer_t = torch.roll(self.action_buffer_t, 1, dims=1)
            self.action_buffer_t[:, 0, :] = action_t
            row_idx = torch.arange(self.n, device=self.tdev)
            applied = self.action_buffer_t[row_idx, self.action_delay_t]
        else:
            applied = action_t

        baseline, _, _ = self._baseline_targets_t()
        targets = torch.clamp(baseline + RES_SCALE * applied,
                              self.jl_lo_t, self.jl_hi_t)
        # Write to mw_d.ctrl through the zero-copy torch view.
        self.ctrl_t.zero_()
        # ctrl_t[:, ctrl_pos_idx] = targets via index_copy on dim 1.
        self.ctrl_t.index_copy_(1, self.ctrl_pos_idx_t, targets)
        # Full-body mode: pin the arm position actuators at nominal so
        # the arms hold their pose (their vel actuators stay 0 from
        # zero_()). The arm mass is then a passive balance load on the
        # leg policy.
        if self.wave_arms:
            # Time-varying arm targets: each env reads the wave at (ep_step + its phase offset),
            # looped -> the policy experiences (and learns to reject) the wave's CoM shift.
            idx = (self.ep_step_t.long() + self.wave_phase0_t) % self.wave_n
            self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, self.wave_ref_t[idx])
        elif self.hold_arms:
            self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, self.arm_targets_t)

        # Domain-randomized pushes — apply velocity impulses on GPU.
        if self._push_p > 0 and self._push_vmax > 0:
            hit = torch.rand(self.n, device=self.tdev) < self._push_p
            if hit.any():
                theta = torch.rand(self.n, device=self.tdev) * (2 * math.pi)
                mag = torch.rand(self.n, device=self.tdev) * self._push_vmax
                dvx = torch.cos(theta) * mag
                dvy = torch.sin(theta) * mag
                self.qvel_t[hit, 0] = self.qvel_t[hit, 0] + dvx[hit]
                self.qvel_t[hit, 1] = self.qvel_t[hit, 1] + dvy[hit]

        # Physics: replay the captured CUDA graph for SUBSTEPS substeps,
        # falling back to direct steps if capture wasn't possible.
        with self.wp.ScopedDevice(self.device):
            if self._cuda_graph is not None:
                self.wp.capture_launch(self._cuda_graph)
            else:
                for _ in range(SUBSTEPS):
                    self.mjw.step(self.mw_m, self.mw_d)

        self.ep_step_t = self.ep_step_t + 1
        self.last_action_t = action_t

        # Reward + done all on GPU.
        bz = self.qpos_t[:, 2]
        w = self.qpos_t[:, 3]; x = self.qpos_t[:, 4]
        y = self.qpos_t[:, 5]; z = self.qpos_t[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sinp = torch.clamp(2 * (w * y - z * x), -1.0, 1.0)
        pitch = torch.asin(sinp)
        vlin_norm = torch.linalg.norm(self.qvel_t[:, 0:3], dim=1)
        vang_norm = torch.linalg.norm(self.qvel_t[:, 3:6], dim=1)

        r = self.r
        r_alive = r.get("alive", 1.0) * torch.ones(self.n, device=self.tdev)
        upright = torch.clamp(1.0 - roll * roll - pitch * pitch, min=0.0)
        r_up = r.get("upright", 0.5) * upright
        r_lin = r.get("lin", -0.1) * vlin_norm
        r_ang = r.get("ang", -0.05) * vang_norm
        r_act = r.get("act", -0.01) * (action_t * action_t).sum(dim=1)
        # Action-RATE penalty ||a_t - a_{t-1}||^2: without it the policy learns a
        # bang-bang/saturated residual that "stands" in the trainer but excites the
        # deploy's contact solver and KICKS the body (verified: |act|~0.98 every tick,
        # angB spikes at deploy). Penalising the rate forces a SMOOTH residual that
        # lets the stiff baseline hold near nominal and transfers to deploy.
        r_rate = r.get("act_rate", 0.0) * ((action_t - self.last_action_t) ** 2).sum(dim=1)
        reward = r_alive + r_up + r_lin + r_ang + r_act + r_rate

        fall = (torch.abs(roll) > ROLL_FAIL) | (torch.abs(pitch) > PITCH_FAIL) | (bz < BZ_FAIL)
        done = fall | (self.ep_step_t >= MAX_EP)
        reward = reward + fall.float() * r.get("term", -1.0)

        if done.any():
            self._reset_envs(env_mask=done)

        obs = self._build_obs_t()
        info = {"bz": bz, "roll": roll, "pitch": pitch}
        return obs, reward, done, info


# ────────────────────────────────────────────────────────────────────
# PPO loop — same compact shape as gpu_mjwarp_residual_trainer.main.
# ────────────────────────────────────────────────────────────────────
def main():
    import torch
    import torch.nn as nn

    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=4096,
                   help="parallel envs (was 2048; GPU usually underutilized at 2k)")
    p.add_argument("--iters", type=int, default=300)
    p.add_argument("--rollout", type=int, default=12,
                   help="rollout per env per iter (was 24; halved to keep "
                        "samples-per-update constant with envs doubled)")
    p.add_argument("--mjcf",
                   default=str(REPO / "projects/robots/unitree/g1/urdf/g1_legs.mjcf.xml"))
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save",
                   default=str(REPO / "projects/policies/research/training/runs/gpu_g1_stand/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=512)
    p.add_argument("--dump-trace", default=None,
                   help="eval: write env-0's closed-loop trajectory (leg q + base pose"
                        " per tick) as a parity-probe trace JSON for g1_parity_compare")
    p.add_argument("--eval-settle", type=int, default=0,
                   help="eval: hold NOMINAL (action=0) this many steps before the policy"
                        " engages -> settle to the NOMINAL PD equilibrium (match deploy)")
    p.add_argument("--alive", type=float, default=1.0)
    p.add_argument("--upright", type=float, default=0.5)
    p.add_argument("--lin", type=float, default=-0.1)
    p.add_argument("--ang", type=float, default=-0.05)
    p.add_argument("--act", type=float, default=-0.01)
    p.add_argument("--act-rate", type=float, default=0.0,
                   help="penalty on ||a_t - a_{t-1}||^2 (e.g. -0.05) -> SMOOTH residual "
                        "that transfers to deploy instead of bang-bang that kicks it")
    p.add_argument("--term", type=float, default=-1.0)
    p.add_argument("--sim-dt", type=float, default=0.0)
    p.add_argument("--init-from", default=None)
    # Domain randomization — defaults tuned for sim-to-deploy robustness.
    # The deploy wrapper introduces 1-tick control-delay + per-step state
    # sync that the trainer's raw mjw.step doesn't see. Training over the
    # union of mass/friction/gain/latency/push variation produces a
    # policy that doesn't care about which specific wrapper runs it.
    p.add_argument("--dr-mass-scale", type=float, default=0.30,
                   help="±fraction PER-BODY mass+inertia jitter")
    p.add_argument("--dr-com-offset", type=float, default=0.0,
                   help="±metres PER-BODY CoM (body_ipos) jitter -- the key standing-"
                        "transfer knob (HuB/HoST). Try 0.03. Deploy MUST run with "
                        "OMNISIM_NEWTON_USE_LINK_COM=1 for nominal CoM parity.")
    p.add_argument("--dr-friction-scale", type=float, default=0.50,
                   help="±fraction ground friction jitter")
    p.add_argument("--dr-damping-scale", type=float, default=0.50,
                   help="±fraction PER-DOF joint damping jitter")
    p.add_argument("--dr-actuator-kp-scale", type=float, default=0.40,
                   help="±fraction position-actuator kp jitter, e.g. kp=20 ±40")
    p.add_argument("--dr-actuator-kv-scale", type=float, default=0.40,
                   help="±fraction velocity-actuator kv jitter")
    p.add_argument("--dr-gravity-scale", type=float, default=0.05,
                   help="±fraction gravity jitter")
    p.add_argument("--dr-push-prob", type=float, default=0.02,
                   help="per-step probability of external pelvis push")
    p.add_argument("--dr-push-vmax", type=float, default=1.5,
                   help="max push velocity impulse m/s (random horizontal dir)")
    p.add_argument("--dr-obs-noise", type=float, default=0.03,
                   help="gaussian obs noise std")
    p.add_argument("--dr-action-latency-max", type=int, default=3,
                   help="max action-latency ticks (per-env uniform random 0..N)")
    p.add_argument("--dr-init-q-band", type=float, default=0.15,
                   help="±rad initial joint q jitter on reset")
    p.add_argument("--dr-init-xy-band", type=float, default=0.05,
                   help="±m initial base xy jitter on reset")
    p.add_argument("--dr-init-z-band", type=float, default=0.02,
                   help="±m initial base z jitter on reset")
    p.add_argument("--dr-init-tilt-band", type=float, default=0.0,
                   help="±rad initial base roll+pitch jitter (teaches recovery from "
                        "the deploy's folded/tilted handover; try 0.35)")
    p.add_argument("--dr-init-vel-band", type=float, default=0.0,
                   help="± initial base 6-DOF velocity jitter m/s & rad/s (try 0.4)")
    p.add_argument("--dr-seed", type=int, default=0,
                   help="seed for the per-run model-param draws")
    p.add_argument("--no-dr", action="store_true",
                   help="disable all domain randomization")
    p.add_argument("--hold-arms", action="store_true",
                   help="full-body mode: arms present in MJCF, pinned at "
                        "nominal, not policy-controlled (use with the "
                        "full-body MJCF g1_full.mjcf.xml)")
    p.add_argument("--wave-ref", type=str, default="",
                   help="stand+WAVE mode: drive the arms along this ghost-replay CSV (looped, "
                        "random per-env phase) instead of pinning at nominal, so the legs policy "
                        "learns to balance THROUGH the waving arm. Implies --hold-arms.")
    args = p.parse_args()
    if args.wave_ref:
        args.hold_arms = True

    if not Path(args.mjcf).exists():
        raise SystemExit(
            f"MJCF not found: {args.mjcf}. Build it first by running\n"
            f"  python projects/policies/research/training/build_g1_mjcf.py")

    reward_cfg = dict(alive=args.alive, upright=args.upright,
                      lin=args.lin, ang=args.ang, act=args.act,
                      act_rate=args.act_rate, term=args.term)
    if args.no_dr:
        dr_cfg = {}
    else:
        dr_cfg = dict(
            mass_scale=args.dr_mass_scale,
            com_offset=args.dr_com_offset,
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
            seed=args.dr_seed,
        )
        print(f"[DR] {dr_cfg}")
    env = BatchedG1StandEnv(args.envs, args.mjcf, reward_cfg=reward_cfg,
                            sim_dt=args.sim_dt, dr_cfg=dr_cfg,
                            hold_arms=args.hold_arms, wave_ref=args.wave_ref)
    if args.wave_ref:
        print(f"[stand+wave] driving {len(ARM_JOINTS)} arm joints along {args.wave_ref} "
              f"(looped, random per-env phase); policy learns to balance THROUGH the wave")
    elif args.hold_arms:
        print(f"[full-body] holding {len(ARM_JOINTS)} arm joints at nominal "
              f"(policy still 13-DOF legs+waist)")
    N = args.envs

    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(),
                                    nn.Linear(128, NJ))
            self.v = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                   nn.Linear(256, 128), nn.Tanh(),
                                   nn.Linear(128, 1))
            # log_std starts small so the residual is nearly zero on
            # day 1 — the analytic baseline carries the initial state.
            self.log_std = nn.Parameter(-1.0 * torch.ones(NJ))

        def forward(self, obs):
            return self.pi(obs), self.v(obs).squeeze(-1), self.log_std

    # All-GPU PPO. Actor + rollouts live on cuda; env returns torch
    # tensors directly (no numpy round-trip per step).
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
        # Settle: hold NOMINAL (zero residual) so env-0 reaches the NOMINAL PD
        # equilibrium before the policy engages -- matches the deploy controller's
        # settle so both sides start the policy from the same state.
        if args.eval_settle > 0:
            zero_a = torch.zeros(env.n, NJ, device=tdev)
            for _ in range(args.eval_settle):
                obs, _, _, _ = env.step(zero_a)
        survived = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        # First-fall step per env (0 = never fell). Disambiguates "stands a long
        # time then tips once" from "tips every ~1 s and gets auto-reset" -- the
        # cumulative survived count alone can't tell them apart.
        first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        n_falls = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        _trace = [] if args.dump_trace else None

        def _wxyz_to_rotmat(qw, qx, qy, qz):
            xx, yy, zz = qx*qx, qy*qy, qz*qz
            xy, xz, yz = qx*qy, qx*qz, qy*qz
            wx, wy, wz = qw*qx, qw*qy, qw*qz
            return [1-2*(yy+zz), 2*(xy-wz), 2*(xz+wy),
                    2*(xy+wz), 1-2*(xx+zz), 2*(yz-wx),
                    2*(xz-wy), 2*(yz+wx), 1-2*(xx+yy)]

        for step in range(args.eval_steps):
            with torch.no_grad():
                mu, _, _ = ac(obs)
            obs, _, done, info = env.step(mu)
            if _trace is not None:
                qp0 = env.qpos_t[0].detach().cpu().numpy()
                a0 = torch.clamp(mu[0], -1.0, 1.0).detach().cpu().numpy()
                legq = [float(qp0[int(env.controller_to_qpos[i])]) for i in range(NJ)]
                tgt = np.clip(NOMINAL + RES_SCALE * a0, JOINT_LIMITS_LO, JOINT_LIMITS_HI)
                _trace.append({"k": step, "phase": "probe",
                               "target": [float(x) for x in tgt], "q": legq,
                               "base_pos": [float(qp0[0]), float(qp0[1]), float(qp0[2])],
                               "base_rot": _wxyz_to_rotmat(float(qp0[3]), float(qp0[4]),
                                                           float(qp0[5]), float(qp0[6]))})
            survived += (~done).to(torch.int32)
            n_falls += done.to(torch.int32)
            newly = done & (first_fall == 0)
            first_fall = torch.where(newly, torch.full_like(first_fall, step + 1), first_fall)
        s = survived.cpu().numpy()
        ff = first_fall.cpu().numpy()
        nf = n_falls.cpu().numpy()
        never = (ff == 0)
        ff_fell = ff[~never]
        print(f"[gpu-eval] policy={args.save} envs={env.n} steps={args.eval_steps}")
        print(f"  survival steps: mean={s.mean():.1f}  "
              f"median={np.median(s):.0f}  max={s.max()}  "
              f"frac_full={(s >= args.eval_steps).mean():.2f}")
        print(f"  FIRST-FALL step: mean={ff_fell.mean() if ff_fell.size else -1:.1f}  "
              f"median={np.median(ff_fell) if ff_fell.size else -1:.0f}  "
              f"never_fell_frac={never.mean():.2f}  mean_n_falls={nf.mean():.1f}")
        if _trace is not None:
            import json
            out = {"schema": 1, "side": "trainer",
                   "meta": {"robot": "g1", "construction": "raw mujoco_warp MJCF (trainer)",
                            "sequence": "policy", "njoints": NJ,
                            "joint_order": list(LEGS_JOINTS), "obs_dim": OBS_DIM,
                            "ke": os.environ.get("G1_FORCE_KE", "mjcf"),
                            "kd": os.environ.get("G1_FORCE_KD", "mjcf")},
                   "ticks": _trace}
            Path(args.dump_trace).parent.mkdir(parents=True, exist_ok=True)
            Path(args.dump_trace).write_text(json.dumps(out), encoding="utf-8")
            print(f"[dump-trace] wrote {args.dump_trace} ({len(_trace)} ticks, "
                  f"env-0 closed-loop)")
        return

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)

    rollout = args.rollout
    obs = env.reset()
    total_steps = 0
    t0 = time.time()

    # GPU rollout buffers — no per-step CPU↔GPU traffic.
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

        obs_flat = obs_buf.reshape(-1, OBS_DIM)
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
            loss = pi_loss + 0.5 * v_loss - 0.01 * ent
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5)
            opt.step()

        if it % 5 == 0 or it == 1:
            ep_rps = rew_buf.mean().item()
            mean_v = val_buf.mean().item()
            fps = total_steps / max(time.time() - t0, 1e-6)
            print(f"it {it:4d}  ep_rew/step~{ep_rps:+.3f}  "
                  f"meanV {mean_v:+.2f}  steps {total_steps:,}  "
                  f"{fps:,.0f} env-steps/s")

    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total_steps:,} steps in "
          f"{time.time() - t0:.1f}s)")

    # ONNX export: copy to CPU first (the deploy controller runs ORT on
    # CPU), wrap action head with tanh to match env action space.
    onnx_path = Path(args.save).with_suffix(".onnx")
    class DeployPolicy(torch.nn.Module):
        def __init__(self, pi):
            super().__init__()
            self.pi = pi

        def forward(self, obs):
            # G1_ONNX_CLAMP=1 -> clamp (matches the deploy + the trainer's
            # deterministic eval, which both clamp); default tanh is the legacy
            # path and DOUBLE-squashes against a clamping deploy (known gotcha).
            if os.environ.get("G1_ONNX_CLAMP"):
                return torch.clamp(self.pi(obs), -1.0, 1.0)
            return torch.tanh(self.pi(obs))

    cpu_ac = AC()
    cpu_ac.load_state_dict({k: v.cpu() for k, v in ac.state_dict().items()})
    wrapped = DeployPolicy(cpu_ac.pi)
    wrapped.eval()
    dummy = torch.zeros(1, OBS_DIM, dtype=torch.float32)
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
