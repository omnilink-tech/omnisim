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
that deploys to OmniSim Newton â€” see:

    docs/developer/sim-to-deploy-rl-recipe.md  (general recipe)
    docs/developer/g1-stand-rl-playbook.md     (case study + journey)

Three things make this trainer reliable for sim-to-deploy:

  1. Heavy domain randomization. Body mass / friction / damping /
     actuator kp / kv / gravity / pushes / obs noise / action latency
     / initial pose are all randomized aggressively. The deploy
     wrapper's specific quirks land inside the training distribution
     instead of needing to be matched in code.

  2. Six stacked GPU speedups so heavy-DR runs are cheap:
        - Actor + rollout buffers on cuda (requires CUDA torch).
        - wp.to_torch zero-copy views of mw_d.qpos/qvel/ctrl, all env
          computations in torch on cuda, zero CPUâ†”GPU traffic.
        - CUDA graph capture of the SUBSTEPS physics loop.
        - 4096 envs (vs the typical 2048) for better GPU saturation.
        - SUBSTEPS=4 Ã— 4 ms physics dt (vs 8 Ã— 2 ms) â€” same env-step
          semantics, half the physics work.
        - reset refreshes xpos via mjw.KINEMATICS, not mjw.forward.
          Some env resets nearly every step, so the per-reset refresh
          runs ~once/step; a full forward (collision+constraint solve)
          there was ~60% of total train time. xpos is a pure function
          of qpos via kinematics (bit-identical), so this is a ~2.5x
          end-to-end speedup at zero behavioural change (RTX 5070 Ti
          Laptop, full-body kp100: ~39k -> ~98k env-steps/s). The
          sibling stand/walk trainers still carry the old forward.

  Profiling: set G1_PROF=1 for a rollout-vs-PPO ms/it split, or
  G1_PROF=2 to also break the env-step into ctrl/phys/rew/reset/obs
  and the reset into idx/seed/fwd. Zero overhead when unset.

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
(commit 806753dc). NOTE â€” this is the TRAINER result, not deploy. The
OmniSim Newton DEPLOY currently stands to t â‰ˆ 1.55 s then loses balance
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

# SINGLE SOURCE OF TRUTH for the G1 Newton walk physics model. The flat physics
# constants below (DT/SUBSTEPS/SPAWN_Z/RES_SCALE) are SOURCED from the spec so
# the trainer, the deploy, and the conformance test can never drift. Values are
# UNCHANGED (proven old==new before the refactor); see g1_physics.json.
from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402

# SINGLE SOURCE for the deploy-faithful obs/IC layer. Used ONLY behind the
# DEFAULT-OFF G1_ENV_CORE flag (see BatchedG1StandEnv.use_env_core): the shared
# JointVelEstimator (finite-diff qd, matching the deploy's position-only sensors)
# and the gravity-settle initial condition. The default code path never touches
# this import's behaviour, so the legacy obs/IC stays byte-identical.
from projects.policies.research.backends import g1_env_core  # noqa: E402


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# G1 stand layout constants â€” must match projects/policies/research/envs/g1_stand_env.py
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
NJ = 13            # 6 left + 6 right leg joints + 1 waist
QPOS_J0 = 7        # leg joints start in qpos (after 7-dim free joint)
QVEL_J0 = 6        # leg joints start in qvel (after 6-dim free joint dofs)
OBS_DIM = 50       # lin(3)+ang(3)+proj_g(3)+q(13)+qd(13)+last_action(13)+gait_phase(2)
RES_SCALE = SPEC.ACT_SCALE  # Â±0.3 rad residual (on NOMINAL + gait reference)
DT = SPEC.DT                # env-step dt = OmniSim basicTimeStep 16 ms
SUBSTEPS = SPEC.SUBSTEPS    # 4 substeps * 4 ms physics timestep = 16 ms env-step
                           # (was 8 substeps * 2 ms; mujoco_warp is stable for
                           # human-scale dynamics at 4 ms, halves physics work)
PHYS_DT = DT / SUBSTEPS    # physics timestep â€” written into MJCF at load time
                           # (== 0.004; derived from spec DT/SUBSTEPS, was literal 0.004)

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

# ── DETERMINISTIC STAND LEAN (G1_STAND_LEAN=1) ──────────────────────────────
# Ports the humanoid_stand_deploy reactive ankle/hip LEAN into the trainer
# baseline so a gait-off STAND is ACTIVELY balanced (the same controller that
# makes the deploy stand hold 120 s+). The fast forward-velocity term (kv*vx) is
# the lever the ankle-only balance PD lacks -- it catches the forward-CoM tip
# before pitch develops. Pitch-only (roll lean destabilised in deploy too).
# Defaults == g1.json lean. Train a residual ON TOP of this stable base; the
# deploy runs the identical lean, so train base == deploy base. Default OFF.
STAND_LEAN = _balos.environ.get("G1_STAND_LEAN", "0") == "1"
LEAN_KV = float(_balos.environ.get("G1_STAND_LEAN_KV", "0.14"))      # fwd-velocity gain (fast)
LEAN_KP = float(_balos.environ.get("G1_STAND_LEAN_KP", "1.6"))       # fwd-tilt gain
LEAN_KD = float(_balos.environ.get("G1_STAND_LEAN_KD", "0.25"))      # tilt-rate damping
LEAN_HIP = float(_balos.environ.get("G1_STAND_LEAN_HIP", "0.35"))    # hip share of the lean
LEAN_CLAMP = float(_balos.environ.get("G1_STAND_LEAN_CLAMP", "0.30"))

# Joint indices into LEGS_JOINTS (i.e. controller order).
_L_AP = LEGS_JOINTS.index("left_ankle_pitch_joint")
_R_AP = LEGS_JOINTS.index("right_ankle_pitch_joint")
_L_AR = LEGS_JOINTS.index("left_ankle_roll_joint")
_R_AR = LEGS_JOINTS.index("right_ankle_roll_joint")
_L_HP = LEGS_JOINTS.index("left_hip_pitch_joint")
_R_HP = LEGS_JOINTS.index("right_hip_pitch_joint")
_L_KN = LEGS_JOINTS.index("left_knee_joint")
_R_KN = LEGS_JOINTS.index("right_knee_joint")
_L_HR = LEGS_JOINTS.index("left_hip_roll_joint")
_R_HR = LEGS_JOINTS.index("right_hip_roll_joint")
_L_HY = LEGS_JOINTS.index("left_hip_yaw_joint")
_R_HY = LEGS_JOINTS.index("right_hip_yaw_joint")
GAIT_A_LAT = 0.0      # lateral hip-roll sway amplitude (default 0 = sagittal-only)

# â”€â”€ WALK gait reference (open-loop CPG the policy residual refines) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# A periodic hip-pitch + knee reference added to NOMINAL drives stepping; the RL
# residual + the velocity/upright reward stabilise + propel. Per leg, phase th:
#   hip_pitch += -A_HIP*sin(th)  (mid-swing th=pi/2 -> thigh forward; mid-stance back)
#   knee      += +A_KNEE*max(0,sin(th))  (bend to lift the foot during swing only)
# Right leg runs at th+pi (anti-phase). Tunable via CLI / reward_cfg.
GAIT_FREQ = 1.3       # Hz (steps/s per leg cycle)
GAIT_A_HIP = 0.35     # rad hip-pitch swing amplitude
GAIT_A_KNEE = 0.45    # rad knee lift amplitude
VX_TARGET = 0.4       # m/s forward velocity command

# Episode termination.
SPAWN_Z = SPEC.SPAWN_Z   # 0.78 (sourced from the spec; value unchanged)
BZ_FAIL = 0.45
ROLL_FAIL = 0.8
PITCH_FAIL = 0.8
MAX_EP = 500       # default; override per-run via --max-ep (reward_cfg["max_ep"])

# Joint position limits (URDF <limit> values).
#
# RESIDUAL (NOT migrated to g1_physics_spec.leg_limits()): these are the URDF
# limits ROUNDED TO 3 DECIMALS as the winning walker was trained with them.
# SPEC.leg_limits() reads the FULL URDF precision (e.g. -2.5307 vs -2.531),
# which differs by up to ~4e-4 rad. Routing these through SPEC would change the
# joint-clamp boundaries the policy was trained against, so this is left as a
# documented train<->deploy residual rather than silently altering the walker.
# To unify: replace with SPEC.leg_limits() lo/hi AND retrain (then both sides
# clamp to the identical full-precision URDF limits). See g1_physics.json
# "_residuals". DO NOT change without a retrain.
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

# â”€â”€ Optional full-body mode (--hold-arms) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# The 10 arm joints. The policy NEVER controls these (action stays
# 13-dim, obs stays 48-dim â€” deploy-compatible with g1_stand_deploy).
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
# Arms hanging straight down (elbow=0), small Â±0.2 shoulder-roll splay â€”
# matches g1_stand_arms_deploy.py _ARM_DOWN exactly. CoM-neutral.
ARM_NOMINAL = np.array([
    +0.00, +0.20, +0.00, +0.00, +0.00,   # left  arm
    +0.00, -0.20, +0.00, +0.00, +0.00,   # right arm
], dtype=np.float32)
assert ARM_NOMINAL.shape == (len(ARM_JOINTS),)


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Small numpy helpers (same as OmniQuad trainer).
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Batched env â€” N parallel G1 standing instances on GPU mujoco_warp.
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
_SP_ON = _balos.environ.get("G1_PROF") in ("2", "step")
_STEP_PROF = {"ctrl": 0.0, "phys": 0.0, "rew": 0.0, "reset": 0.0, "obs": 0.0, "n": 0}
_RST_PROF = {"idx": 0.0, "seed": 0.0, "fwd": 0.0, "m": 0}


class BatchedG1StandEnv:
    """Standing-task batched env. No gait, no IK; baseline is NOMINAL +
    ankle balance PD. Residual policy adds Â±RES_SCALE rad on top.
    """

    def __init__(self, n, mjcf, device="cuda:0", reward_cfg=None, sim_dt=0.0,
                 dr_cfg=None, hold_arms=False):
        import warp as wp
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mjw = wp, mjw
        self.n = n
        # Full-body mode: arms present in the MJCF, pinned at nominal,
        # not controlled by the policy. See ARM_JOINTS / ARM_NOMINAL.
        self.hold_arms = hold_arms
        self.device = wp.get_device(device)
        # Domain randomization config. Closes the MuJoCo->Newton sim2sim
        # gap by training over a band of physics params + perturbations
        # so the policy is robust to the slight differences between the
        # MJ solver (used by mujoco_warp) and Newton's XPBD solver.
        self.dr = dr_cfg or {}

        # â”€â”€ DEPLOY-FAITHFUL obs/IC (DEFAULT OFF) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        # When G1_ENV_CORE=1, two genuine trainer<->deploy divergences are
        # closed via the shared projects/policies/research/backends/g1_env_core module:
        #   (1) obs qd is FINITE-DIFFERENCED from the achieved leg positions
        #       (the deploy has only position sensors) instead of read from the
        #       exact MuJoCo qvel; via g1_env_core.JointVelEstimator.
        #   (2) the reset initial condition is the deploy's ~0.3 s gravity
        #       settle (a forward lean + residual base velocity + sagged leg q)
        #       instead of pitch=0 / qvel=0, precomputed ONCE at first reset.
        # With the flag UNSET every code path below is the legacy one, so the
        # default run is byte-identical (no regression).
        self.use_env_core = os.environ.get("G1_ENV_CORE", "0") == "1"
        self._qd_est = None        # lazily-created JointVelEstimator (use_env_core)
        self._settle_ic = None     # cached (lean_quat[4], base_qvel[6], leg_q[13])

        self.mjm = mujoco.MjModel.from_xml_path(mjcf)
        # Force physics dt to PHYS_DT (4 ms). With SUBSTEPS=4 this gives a
        # 16-ms env-step matching OmniSim basicTimeStep, but halves the
        # number of physics ticks per env-step vs the previous
        # SUBSTEPS=8 Ã— 2 ms config.
        self.mjm.opt.timestep = float(sim_dt) if sim_dt and sim_dt > 0 else PHYS_DT

        # Domain randomization on the MJCF model BEFORE putting it on
        # the device. mujoco_warp doesn't support per-env model params
        # cheaply, so each TRAINING RUN samples one point in the
        # manifold. Run-to-run variation + per-step perturbations
        # together produce a policy robust to a band of physics.
        rng = np.random.default_rng(self.dr.get("seed", 0))
        mass_scale_band = self.dr.get("mass_scale", 0.0)   # Â±fraction
        fric_band = self.dr.get("friction_scale", 0.0)
        damp_band = self.dr.get("damping_scale", 0.0)
        actuator_kp_band = self.dr.get("actuator_kp_scale", 0.0)
        actuator_kv_band = self.dr.get("actuator_kv_scale", 0.0)
        gravity_band = self.dr.get("gravity_scale", 0.0)
        if mass_scale_band > 0:
            # Per-body scale, not global â€” Newton's wrapper applies
            # body-by-body inertia conversion that can drift slightly
            # per link, so the policy needs to be robust to per-body
            # mass distribution, not just total mass.
            scales = rng.uniform(1.0 - mass_scale_band, 1.0 + mass_scale_band,
                                 size=self.mjm.body_mass.shape).astype(np.float32)
            self.mjm.body_mass[:] *= scales
            self.mjm.body_inertia[:] *= scales[:, None]
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
        # CONTACT-softness DR: randomize the contact solref (time constant +
        # damping ratio) and solimp. This is THE knob the warp<->Newton gap
        # turns on -- the Newton wrapper sets up contacts differently, and
        # tight lateral gaits that overfit one contact stiffness collapse under
        # another. Per-run (shared), so the chunk-to-chunk dr-seed sweep makes
        # the policy span a band of contact behaviours.
        solref_band = self.dr.get("solref_scale", 0.0)
        if solref_band > 0:
            tc = rng.uniform(1.0 - solref_band, 1.0 + solref_band,
                             size=self.mjm.geom_solref[:, 0].shape).astype(np.float32)
            dr = rng.uniform(1.0 - solref_band, 1.0 + solref_band,
                             size=self.mjm.geom_solref[:, 1].shape).astype(np.float32)
            self.mjm.geom_solref[:, 0] *= tc        # contact time constant (softness)
            self.mjm.geom_solref[:, 1] *= dr        # contact damping ratio
            # Stability floor: timeconst must stay >= ~2*dt or the contact
            # constraint goes unstable. Lets us widen the band safely.
            self.mjm.geom_solref[:, 0] = np.clip(
                self.mjm.geom_solref[:, 0], 2.5 * float(self.mjm.opt.timestep), 0.1)
            si = float(rng.uniform(1.0 - 0.5 * solref_band, 1.0 + 0.5 * solref_band))
            self.mjm.geom_solimp[:, :2] = np.clip(
                self.mjm.geom_solimp[:, :2] * si, 0.0, 0.9999)   # impedance d0,d1

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
                    f"actuators {pos_name}/{vel_name} not in MJCF â€” see "
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

        # GAIT-V2: the nominal pose is configurable (reward_cfg["nominal"],
        # list of NJ floats). The deep-squat default was a STANDING
        # stability fix (CoM vs the importer's foot placement); a walking
        # policy balances dynamically and can carry a TALL, human-like
        # posture -- the crouch is what makes the old gait look wrong.
        self.nominal = np.array(self.r.get("nominal", NOMINAL), dtype=np.float32)
        assert self.nominal.shape == (NJ,)

        # Seed pose: at SPAWN_Z, identity orientation, NOMINAL leg pose.
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

        # â”€â”€â”€ GPU-native env state â”€â”€â”€
        # torch.device matching the warp device so we share GPU memory.
        self.tdev = torch.device("cuda:0" if "cuda" in str(self.device).lower()
                                 else "cpu")
        # Torch views of mujoco_warp's qpos/qvel/ctrl arrays. wp.to_torch
        # shares GPU memory zero-copy â€” writes propagate. This is the
        # core trick: physics runs on the warp side, policy runs on the
        # torch side, both see the same GPU buffer.
        self.qpos_t = wp.to_torch(self.mw_d.qpos).view(n, self.nq)
        self.qvel_t = wp.to_torch(self.mw_d.qvel).view(n, self.nv)
        self.ctrl_t = wp.to_torch(self.mw_d.ctrl).view(n, self.nu)

        # Constants moved to GPU once at init.
        self.nominal_t = torch.tensor(self.nominal, dtype=torch.float32, device=self.tdev)
        self.jl_lo_t = torch.tensor(JOINT_LIMITS_LO, dtype=torch.float32, device=self.tdev)
        self.jl_hi_t = torch.tensor(JOINT_LIMITS_HI, dtype=torch.float32, device=self.tdev)
        self.qpos_idx_t = torch.tensor(self.controller_to_qpos, dtype=torch.long, device=self.tdev)
        self.qvel_idx_t = torch.tensor(self.controller_to_qvel, dtype=torch.long, device=self.tdev)
        # VELOCITY-CONDITIONED walking: per-env forward-speed COMMAND. When
        # vx_cmd_max > 0 the policy is told a target speed (sampled in
        # [0, vx_cmd_max], INCLUDING 0 = stand); the gait amplitude scales with
        # it and the reward tracks it, so commanding 0 makes the robot
        # decelerate and STAND, and any speed makes it walk -- one policy that
        # starts/stops on command (the stop-in-the-middle milestone).
        self.vx_cmd_max = float(self.r.get("vx_cmd_max", 0.0))
        self.vx_cond = self.vx_cmd_max > 0.0
        self.vx_cmd_t = torch.full((n,), float(self.r.get("vx_target", VX_TARGET)),
                                   dtype=torch.float32, device=self.tdev)
        # Speed below which the gait CLOCK freezes (step frequency drops with
        # speed). At vx_cmd=0 the phase stops advancing -> a STATIC stand
        # instead of a cycling phase that drives a residual micro-step/creep.
        self.vx_phase_freeze = float(self.r.get("vx_phase_freeze", 0.10))
        self.ctrl_pos_idx_t = torch.tensor(self.controller_to_ctrl_pos, dtype=torch.long, device=self.tdev)
        self.seed_qpos_t = torch.tensor(self.seed_qpos, dtype=torch.float32, device=self.tdev)
        if self.hold_arms:
            self.arm_ctrl_pos_idx_t = torch.tensor(
                self.arm_ctrl_pos, dtype=torch.long, device=self.tdev)
            # (n, 10) target block, constant â€” built once.
            self.arm_targets_t = torch.tensor(
                ARM_NOMINAL, dtype=torch.float32, device=self.tdev
            ).unsqueeze(0).expand(n, -1).contiguous()

        self.ep_step_t = torch.zeros(n, dtype=torch.int32, device=self.tdev)
        self.last_action_t = torch.zeros(n, NJ, dtype=torch.float32, device=self.tdev)
        self.prev_roll_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.prev_pitch_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.prev_roll_rate_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.prev_pitch_rate_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        # WALK: per-env gait phase (rad) + cached gait params (from reward_cfg).
        self.phase_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.gait_freq = float(self.r.get("gait_freq", GAIT_FREQ))
        self.gait_a_hip = float(self.r.get("gait_a_hip", GAIT_A_HIP))
        self.gait_a_knee = float(self.r.get("gait_a_knee", GAIT_A_KNEE))
        self.gait_a_lat = float(self.r.get("gait_a_lat", GAIT_A_LAT))
        self.cp_gain = float(self.r.get("cp_gain", 0.0))
        self.vx_target = float(self.r.get("vx_target", VX_TARGET))
        self.max_ep = int(self.r.get("max_ep", MAX_EP))
        # Residual authority. 0.3 suits gait-residual mode; raise to ~0.5 for
        # policy-owned-gait mode (all gait amplitudes 0, the policy must create
        # the stepping itself to satisfy the phase-based contact schedule).
        self.res_scale = float(self.r.get("res_scale", RES_SCALE))
        # Per-joint residual authority. Scaling DOWN hip-roll/yaw stops the
        # policy from splaying ~22deg on top of the shadow's commanded weight
        # transfer (the deploy splay crutch) -- it can still trim, just not
        # override. frontal_res_scale=1.0 -> uniform (unchanged). Deploy MUST
        # match via G1_FRONTAL_RES_SCALE.
        fr = float(self.r.get("frontal_res_scale", 1.0))
        rsv = np.full(NJ, self.res_scale, dtype=np.float32)
        for j in (_L_HR, _R_HR, _L_HY, _R_HY):
            rsv[j] *= fr
        self.res_scale_vec = torch.tensor(rsv, device=self.tdev)
        # Ankle-pitch counter-rotation: keeps the foot FLAT as the hip swings
        # (sagittal chain foot pitch ~ hip+knee+ankle; without this the stiff
        # ankle pins the foot at NOMINAL and fights ground rollover -- the
        # open-loop CPG even drifts BACKWARD; with a_ankle~0.35 it walks
        # forward; see _scratch/g1_gait_sweep.py 2026-06-10).
        self.gait_a_ankle = float(self.r.get("gait_a_ankle", 0.0))
        # Reset seeding: start each episode IN the gait pose at its sampled
        # phase (q + qd consistent with the reference) instead of NOMINAL.
        # At stiff gains a NOMINAL start + random phase = instant 0.35-0.45
        # rad target snap that knocks the robot over before it ever walks.
        self.seed_gait = bool(self.r.get("seed_gait", False))
        # GAIT-V2: counter-phase arm swing (hold-arms mode; shoulder_pitch of
        # each arm swings opposite its same-side leg, like a human) and an
        # ankle push-off bump at late stance. Signed amplitudes; default 0.
        self.gait_a_arm = float(self.r.get("gait_a_arm", 0.0))
        self.gait_a_push = float(self.r.get("gait_a_push", 0.0))
        # REST-START mixing: this fraction of episodes starts EXACTLY like
        # the deploy handover -- standing at NOMINAL, zero velocity, phase 0,
        # no init jitter/vx-bias. The mid-stride seeding (seed_gait) covers
        # SUSTAINING the walk but leaves the deploy's standing LAUNCH
        # out-of-distribution -- the recurring deploy failure mode.
        self.rest_start_frac = float(self.r.get("rest_start_frac", 0.0))
        # HUMAN GAIT MODEL: foot-space planned, IK-realized reference
        # (projects/policies/control/gait/g1_human_gait.py) replacing the joint-space sine
        # CPG entirely. NOMINAL becomes the model's standing pose (== its
        # phase-0/zero-stride output, so a standing start has no snap) and a
        # per-env stride RAMP grows the step length out of standing.
        self.gait_model = str(self.r.get("gait_model", ""))
        self.gp = None
        if self.gait_model == "human":
            from projects.policies.control.gait import g1_human_gait as ghg
            self._ghg = ghg
            # FK constants for the holistic SHAPE reward (sagittal keypoints).
            self._fk = dict(L1=ghg.L1, L2=ghg.L2, TH=ghg.THIGH_OFF,
                            SH=ghg.SHANK_OFF, HS=ghg.HIP_SIGN,
                            KS=ghg.KNEE_SIGN, LF=0.14)
            gpd = self.r.get("gait_params", {}) or {}
            self.gp = ghg.GaitParams(**gpd)
            self.nominal = ghg.standing_pose(self.gp).astype(np.float32)
            self.vx_target = self.gp.vx
            self.gait_freq = self.gp.freq
            self.phase_dt = 2.0 * math.pi * self.gait_freq * DT
            # Rebuild the nominal-derived tensors (they were created above
            # from the sine-CPG nominal).
            for i in range(NJ):
                self.seed_qpos[self.controller_to_qpos[i]] = self.nominal[i]
            self.seed_qpos_t = torch.tensor(self.seed_qpos, dtype=torch.float32,
                                            device=self.tdev)
            self.nominal_t = torch.tensor(self.nominal, dtype=torch.float32,
                                          device=self.tdev)
        # Per-env stride-ramp clock origin: rest-start episodes ramp the
        # stride in from 0 (like the deploy); mid-stride episodes start at
        # full stride. Used only by the human gait model.
        self._ramp_t0 = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self._swingL = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self._swingR = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        # Foot-aware rewards (contact schedule / slip) -- the standard levers
        # that make gait policies actually STEP. Need mw_d.xpos (body pos).
        self.rw_sched = float(self.r.get("rw_sched", 0.0))
        self.rw_slip = float(self.r.get("rw_slip", 0.0))
        self.obs_stack = int(self.r.get("obs_stack", 1))
        self._obs_buf = None
        # Reference LOOKAHEAD (the "forecast" obs, standard in imitation
        # locomotion): the model targets at +dt seconds, EXACT because the
        # reference is deterministic. Appended AFTER the frame stack (the
        # future of past frames is redundant). List of seconds.
        self.obs_look = [float(x) for x in self.r.get("obs_lookahead", [])]
        # ASYMMETRIC actor-critic (basal-ganglia analogue): the CRITIC sees
        # privileged sim-only signals the deployable actor cannot -- true base
        # height + foot contacts/heights -- for a lower-variance value
        # estimate. Actor obs (and the exported ONNX) are UNCHANGED.
        self.asym = bool(self.r.get("asym_critic", False))
        self.priv_extra = 5 if self.asym else 0
        # Per-joint imitation weight: hips+knees (the visible human signature)
        # full weight, ankles half (they carry the balance PD so demanding
        # exact tracking would fight balance), waist zero.
        _tw = np.ones(NJ, dtype=np.float32)
        _tw[_L_AP] = _tw[_R_AP] = 0.5
        _tw[_L_AR] = _tw[_R_AR] = 0.5
        _tw[NJ - 1] = 0.0           # waist_yaw: not part of the gait shape
        self._track_w = torch.tensor(_tw, device=self.tdev)
        self._hipkn_idx = torch.tensor([_L_HP, _L_KN, _R_HP, _R_KN],
                                       device=self.tdev)
        self.foot_z_contact = float(self.r.get("foot_z_contact", 0.05))
        self.foot_z_swing = float(self.r.get("foot_z_swing", 0.14))
        self.phase_dt = 2.0 * math.pi * self.gait_freq * DT

        # Torch view of body positions for the foot rewards. ankle_roll links
        # ARE the feet (sole is 0.036 m below the body origin when flat).
        self.bid_lfoot = mujoco.mj_name2id(
            self.mjm, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_roll_link")
        self.bid_rfoot = mujoco.mj_name2id(
            self.mjm, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_roll_link")
        self.xpos_t = wp.to_torch(self.mw_d.xpos).view(n, self.mjm.nbody, 3)
        self.prev_foot_xy_t = torch.zeros(n, 2, 2, dtype=torch.float32,
                                          device=self.tdev)

        # Per-env action-latency buffer (now on GPU).
        self.max_latency_ticks = int(self.dr.get("action_latency_max", 0))
        self.action_buffer_t = torch.zeros(
            n, max(1, self.max_latency_ticks + 1), NJ,
            dtype=torch.float32, device=self.tdev)
        self.action_delay_t = torch.zeros(n, dtype=torch.long, device=self.tdev)

        # Per-env residual-authority gain (DR): each env scales the policy
        # residual by act_gain_t (resampled per episode in _reset_envs). 1.0 =
        # no jitter. Mimics the warp<->Newton gain-drift PER-ENV (unlike the
        # per-RUN --dr-actuator-kp-scale).
        self._act_gain = float(self.dr.get("act_gain", 0.0))
        self.act_gain_t = torch.ones(n, dtype=torch.float32, device=self.tdev)

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

    def _sample_vx_cmd(self, m):
        """Sample m forward-speed commands in [0, vx_cmd_max]. The milestone
        only needs two speeds to be ROBUST -- 0 (stand) and vx_target (nominal
        walk) -- so we concentrate mass on those two atoms and spread the rest
        uniformly for generalisation. A flat uniform[0,vx_cmd_max] dilutes the
        nominal-walk experience to a thin slice near 0.4 and erodes the warm-
        started c8 walking robustness (the c1 deploy regression). Mix:
          ~30% exactly vx_target (preserve the c8 walk),
          ~28% exactly 0        (a solid stand),
          ~42% uniform[0,vx_cmd_max].
        """
        u = torch.rand(m, device=self.tdev)
        vc = torch.rand(m, device=self.tdev) * self.vx_cmd_max   # the uniform tail
        vc = torch.where(u < 0.30,
                         torch.full_like(vc, self.vx_target), vc)
        vc = torch.where((u >= 0.30) & (u < 0.58),
                         torch.zeros_like(vc), vc)
        return vc

    def _ensure_settle_ic(self):
        """Precompute the deploy's gravity-settle launch state ONCE (G1_ENV_CORE).

        The deploy controller lets the robot settle under gravity ~0.3 s while
        the kp=100 joints hold the nominal pose, so the first policy tick sees a
        slightly forward-leaned base, a small residual base velocity, and
        gravity-sagged leg joints -- NOT the trainer's pitch=0 / qvel=0 teleport.
        This runs a single batched settle on the live mujoco_warp data (all
        worlds start from the seed pose, which _reset_all has just tiled), holds
        the seed-pose targets through ``g1_env_core.settle_steps()`` env-steps,
        reads world 0's resulting base quat / base qvel / achieved leg q, caches
        them, then RESTORES the seed pose on every world so the in-progress reset
        continues from a clean state. Cached, so the cost is paid once per run.

        Limitation: the settle is run with DR perturbations + the captured CUDA
        graph active (same physics the rollout uses) but WITHOUT the per-env DR
        jitter/pushes; it captures the deterministic gravity response of the
        nominal pose on world 0 and uses that single IC for all envs (the
        per-env DR jitter in _reset_envs then re-widens the distribution on top).
        """
        if self._settle_ic is not None:
            return
        n_settle = g1_env_core.settle_steps(DT)
        # Start every world from the clean seed pose (qpos) / zero qvel, then
        # hold the seed leg targets so the joints don't drift while the base
        # settles. Operate on the live buffers (caller is mid-reset; we restore
        # the seed pose at the end so the caller's subsequent seeding is clean).
        self.qpos_t[:] = self.seed_qpos_t.unsqueeze(0)
        self.qvel_t[:] = 0.0
        seed_targets = self.seed_qpos_t.index_select(0, self.qpos_idx_t)  # (NJ,)
        for _ in range(n_settle):
            self.ctrl_t.zero_()
            self.ctrl_t.index_copy_(
                1, self.ctrl_pos_idx_t,
                seed_targets.unsqueeze(0).expand(self.n, -1).contiguous())
            if self.hold_arms:
                self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, self.arm_targets_t)
            with self.wp.ScopedDevice(self.device):
                if self._cuda_graph is not None:
                    self.wp.capture_launch(self._cuda_graph)
                else:
                    for _ in range(SUBSTEPS):
                        self.mjw.step(self.mw_m, self.mw_d)
        # Read world 0's settled launch state.
        lean_quat = self.qpos_t[0, 3:7].clone()
        base_qvel = self.qvel_t[0, 0:6].clone()
        leg_q = self.qpos_t[0].index_select(0, self.qpos_idx_t).clone()
        self._settle_ic = (lean_quat, base_qvel, leg_q)
        # Restore the clean seed pose on all worlds so the caller's reset
        # seeding (which assumes a fresh seed pose) is not corrupted by the
        # settle's drift.
        self.qpos_t[:] = self.seed_qpos_t.unsqueeze(0)
        self.qvel_t[:] = 0.0
        self.ctrl_t.zero_()
        with self.wp.ScopedDevice(self.device):
            self.mjw.kinematics(self.mw_m, self.mw_d)
        print(f"[env-core] settle IC ({n_settle} env-steps): "
              f"lean_quat={lean_quat.detach().cpu().numpy().round(4).tolist()} "
              f"base_v={base_qvel.detach().cpu().numpy().round(3).tolist()}")

    def _reset_envs(self, env_mask=None):
        """Reset a subset of envs (or all if env_mask is None).
        Writes new qpos/qvel directly into the GPU buffers via the
        torch views â€” zero CPUâ†”GPU transfer.
        """
        if _SP_ON:
            torch.cuda.synchronize(); _rt = time.time()
        if env_mask is None:
            idx = torch.arange(self.n, device=self.tdev)
        else:
            idx = torch.nonzero(env_mask, as_tuple=False).squeeze(-1)
            if idx.numel() == 0:
                return
        m = idx.shape[0]
        if _SP_ON:
            torch.cuda.synchronize(); _RST_PROF["idx"] += time.time() - _rt
            _RST_PROF["m"] += m; _rt = time.time()

        # Tile seed pose for the rows being reset.
        self.qpos_t[idx] = self.seed_qpos_t.unsqueeze(0).expand(m, -1)
        self.qvel_t[idx] = 0.0

        # DEPLOY-FAITHFUL initial condition (G1_ENV_CORE only): instead of the
        # pitch=0 / qvel=0 teleport above, seed the deploy's post-settle launch
        # state -- a forward base lean (quat), a residual base velocity, and the
        # gravity-sagged leg q -- precomputed ONCE (see _ensure_settle_ic). The
        # existing per-env DR jitter below then still applies ON TOP, matching a
        # stabilised deploy session's handover. Seeded BEFORE the jitter so the
        # jitter widens the distribution around the settle, not around pitch 0.
        # G1_STAND_NO_SETTLE_IC: keep the finite-diff-qd OBS parity (use_env_core)
        # but SKIP the open-loop gravity-settle IC. The settle holds the seed pose
        # with NO active balance, so for the actively-balanced STAND (lean+roll PD)
        # it launches the policy already tipping in roll (deterministic ~1.4 s fall).
        # A clean teleport reset lets the active base hold from t=0 -- matching how
        # the deploy stand is balanced throughout, not the walk's settle handover.
        if self.use_env_core and os.environ.get("G1_STAND_NO_SETTLE_IC", "0") != "1":
            self._ensure_settle_ic()
            lean_quat, base_qvel, leg_q = self._settle_ic
            self.qpos_t[idx, 3:7] = lean_quat.unsqueeze(0)
            self.qvel_t[idx, 0:6] = base_qvel.unsqueeze(0)
            base_idx_ic = idx.unsqueeze(1).expand(-1, NJ)
            col_idx_ic = self.qpos_idx_t.unsqueeze(0).expand(m, -1)
            self.qpos_t[base_idx_ic, col_idx_ic] = leg_q.unsqueeze(0)

        # Velocity command for the reset envs (see _sample_vx_cmd).
        if self.vx_cond:
            self.vx_cmd_t[idx] = self._sample_vx_cmd(m)

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
        # WALK: forward init-velocity bias in [0, init_vx_bias]. Most episodes then
        # START already moving forward, so the policy mostly practises SUSTAINING a
        # walk (the achievable part) instead of the hard rest->walk launch where it
        # fell ~1 s. 0 stays in-range so the deploy's rest start is still covered.
        vxbias = float(self.dr.get("init_vx_bias", 0.0))
        if vxbias > 0:
            self.qvel_t[idx, 0] += torch.rand(m, device=self.tdev) * vxbias

        self.ep_step_t[idx] = 0
        self.last_action_t[idx] = 0.0
        self.prev_roll_t[idx] = 0.0
        self.prev_pitch_t[idx] = 0.0
        # WALK: randomise gait phase per env (decorrelate envs -> the policy sees
        # all gait phases each batch; better than every env stepping in lockstep).
        self.phase_t[idx] = torch.rand(m, device=self.tdev) * (2.0 * math.pi)
        if self.seed_gait and self.gait_model == "human":
            # Mid-stride seeding from the HUMAN gait model: q from the
            # reference at the sampled phase (full stride), qd via finite
            # difference of the reference over one tick.
            th = self.phase_t[idx]
            full = torch.full((idx.shape[0],), 1e6, device=self.tdev)
            legs0, _, _, _ = self._ghg.targets_torch(th, self.gp, full)
            legs1, _, _, _ = self._ghg.targets_torch(th + self.phase_dt, self.gp, full)
            qd_ref = (legs1 - legs0) / DT
            base_idx = idx.unsqueeze(1).expand(-1, NJ)
            self.qpos_t[base_idx, self.qpos_idx_t.unsqueeze(0).expand(idx.shape[0], -1)] = legs0
            self.qvel_t[base_idx, self.qvel_idx_t.unsqueeze(0).expand(idx.shape[0], -1)] = qd_ref
            self._ramp_t0[idx] = 1e6          # mid-stride: full stride length
        elif self.seed_gait:
            # Seed q AND qd to the gait reference at the sampled phase so the
            # episode starts mid-stride instead of snapping from NOMINAL.
            th = self.phase_t[idx]
            sL = torch.sin(th); cL = torch.cos(th)
            sR = -sL; cR = -cL          # sin/cos(th + pi)
            om = 2.0 * math.pi * self.gait_freq   # dth/dt rad/s
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
        # REST-START mixing: override a random subset to the deploy's exact
        # handover state -- clean NOMINAL pose, zero velocity, phase 0. Runs
        # LAST so it wins over every jitter/seeding above.
        if self.rest_start_frac > 0:
            rest = torch.rand(idx.shape[0], device=self.tdev) < self.rest_start_frac
            ridx = idx[rest]
            if ridx.numel() > 0:
                rm = ridx.shape[0]
                self.qpos_t[ridx] = self.seed_qpos_t.unsqueeze(0).expand(rm, -1)
                self.qvel_t[ridx] = 0.0
                # The DEPLOY handover is NOT perfectly clean -- the robot
                # folds from its spawn pose into the settle and arrives with
                # residual joint error, a slight base tilt and motion. Train
                # rest-starts with the same small wobble (a perfectly-clean
                # rest-start trains a launch the deploy never sees).
                rj = (torch.rand(rm, NJ, device=self.tdev) * 2 - 1) * 0.03
                # GRAVITY SAG: after the deploy's settle, the kp=100 joints
                # hold NOMINAL with a steady-state error (tau/kp): knees sag
                # ~ +0.15-0.18, ankles ~ -0.08 (measured in the deploy obs).
                # Rest-starts must cover that band or the deploy launch pose
                # is out-of-distribution.
                sag = torch.rand(rm, device=self.tdev)          # 0..1 of full sag
                for kn_i, ak_i in ((_L_KN, _L_AP), (_R_KN, _R_AP)):
                    rj[:, kn_i] += 0.20 * sag
                    rj[:, ak_i] += -0.10 * sag
                ri = ridx.unsqueeze(1).expand(-1, NJ)
                rc = self.qpos_idx_t.unsqueeze(0).expand(rm, -1)
                self.qpos_t[ri, rc] += rj
                hr = (torch.rand(rm, device=self.tdev) * 2 - 1) * 0.01
                hp2 = (torch.rand(rm, device=self.tdev) * 2 - 1) * 0.01
                cr = torch.cos(hr); sr = torch.sin(hr)
                cp2 = torch.cos(hp2); sp2 = torch.sin(hp2)
                self.qpos_t[ridx, 3] = cr * cp2
                self.qpos_t[ridx, 4] = sr * cp2
                self.qpos_t[ridx, 5] = cr * sp2
                self.qpos_t[ridx, 6] = sr * sp2
                self.qvel_t[ridx, 0:6] += (torch.rand(rm, 6, device=self.tdev) * 2 - 1) * 0.05
                # Human model: start the clock in DOUBLE SUPPORT (at phase 0
                # the right foot is mid-swing -- a rest-start there stands on
                # a lifted foot and tips over).
                self.phase_t[ridx] = (self._ghg.DS_PHASE
                                      if self.gait_model == "human" else 0.0)
                self._ramp_t0[ridx] = 0.0     # stride ramps in from standing
        self.action_buffer_t[idx] = 0.0
        if self.max_latency_ticks > 0:
            self.action_delay_t[idx] = torch.randint(
                0, self.max_latency_ticks + 1, (m,),
                dtype=torch.long, device=self.tdev)
        else:
            self.action_delay_t[idx] = 0
        # Per-env residual-authority gain: resample for the reset rows.
        if self._act_gain > 0.0:
            self.act_gain_t[idx] = (
                1.0 + (torch.rand(m, device=self.tdev) * 2 - 1) * self._act_gain)

        # DEPLOY-FAITHFUL qd (G1_ENV_CORE only): reset the finite-diff estimator
        # rows for the just-reset envs to the FINAL post-reset achieved leg q so
        # the first post-reset qd is 0 (no fake teleport-velocity spike). Done
        # LAST, after every pose seeding/jitter/rest-start override above, so
        # prev_q matches what _build_obs_t will read next tick. The estimator
        # may not exist yet (created lazily on the first _build_obs_t call) -- in
        # that case there is nothing to reset; it seeds itself on creation.
        if self.use_env_core and self._qd_est is not None:
            q_after = self.qpos_t.index_select(1, self.qpos_idx_t)
            if env_mask is None:
                self._qd_est.reset(q_after)
            else:
                self._qd_est.reset_rows(q_after, env_mask)

        if _SP_ON:
            torch.cuda.synchronize(); _RST_PROF["seed"] += time.time() - _rt; _rt = time.time()
        # Refresh xpos (body world positions) for the just-teleported reset
        # rows so the slip-tracker baseline and the asym critic's foot signals
        # see the new pose this step. xpos is a pure function of qpos via
        # forward KINEMATICS -- it does NOT depend on the collision/constraint
        # solve that dominates a full mjw.forward. Using kinematics here is
        # bit-identical for xpos but ~15x cheaper; the full forward was ~60%
        # of total train time because it ran every step (some env almost
        # always resets). Nothing reads cvel/contacts/sensordata before the
        # next mjw.step recomputes the full pipeline.
        with self.wp.ScopedDevice(self.device):
            self.mjw.kinematics(self.mw_m, self.mw_d)
        # Refresh the slip-tracker so the first step after reset doesn't see a
        # fake teleport velocity (xpos is current after mjw.kinematics).
        feet = self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), 0:2]
        self.prev_foot_xy_t[idx] = feet[idx]
        if _SP_ON:
            torch.cuda.synchronize(); _RST_PROF["fwd"] += time.time() - _rt

    def _reset_all(self):
        self._reset_envs(env_mask=None)

    def _build_obs_t(self):
        """All-GPU obs vector. Reads zero-copy torch views of mw_d, no
        CPUâ†”GPU transfer.
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
        q_achieved = qp.index_select(1, self.qpos_idx_t)
        q = q_achieved - self.nominal_t.unsqueeze(0)
        if not self.use_env_core:
            # DEFAULT: exact MuJoCo joint velocity (byte-identical legacy path).
            qd = qv.index_select(1, self.qvel_idx_t)
        else:
            # DEPLOY-FAITHFUL qd: finite-difference the ACHIEVED leg positions
            # through the shared estimator (the deploy has only position
            # sensors). Lazily create it, seeded with the current achieved q so
            # the state tensors match self.device + dtype and the first qd is 0.
            if self._qd_est is None:
                self._qd_est = g1_env_core.JointVelEstimator(
                    DT, tau=float(os.environ.get("G1_QD_TAU", "0.0")))
                self._qd_est.reset(q_achieved)
            qd = self._qd_est.update(q_achieved)
        # WALK: gait phase as sin/cos so the policy is phase-aware (knows where in
        # the step cycle it is) and can phase-lock its residual to the gait.
        gait = torch.stack([torch.sin(self.phase_t), torch.cos(self.phase_t)], dim=1)
        obs = torch.cat([vlin, vang, pg, q, qd, self.last_action_t, gait], dim=1)
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
        self.prev_roll_rate_t = roll_rate     # for the adaptive-phase gate
        self.prev_pitch_rate_t = pitch_rate

        ap = torch.clamp(KP_ANKLE_PITCH * pitch + KD_ANKLE_PITCH * pitch_rate,
                         -BAL_CLAMP, BAL_CLAMP)
        ar = torch.clamp(KP_ANKLE_ROLL * roll + KD_ANKLE_ROLL * roll_rate,
                         -BAL_CLAMP, BAL_CLAMP)
        if self.gait_model == "human":
            # Foot-space human gait reference (replaces the sine CPG below);
            # the ankle balance PD still adds on top.
            t_since = self._ramp_t0 + self.ep_step_t.to(torch.float32) * DT
            legs, arms, swL, swR = self._ghg.targets_torch(
                self.phase_t, self.gp, t_since_start_t=t_since)
            self._model_arms = arms
            # VELOCITY-CONDITIONED: scale the gait amplitude by the commanded
            # speed s = vx_cmd/vx_nominal. legs -> nominal + s*(legs - nominal),
            # so s=0 collapses the gait to the STANDING pose (feet planted, no
            # stride) and s=1 is the full gait. The swing weights scale too so
            # the foot-contact/style rewards expect no stepping at s=0.
            if self.vx_cond:
                s = torch.clamp(self.vx_cmd_t / max(self.vx_target, 1e-3),
                                0.0, 1.25).unsqueeze(1)
                legs = self.nominal_t.unsqueeze(0) + s * (legs - self.nominal_t.unsqueeze(0))
                arms = s * arms
                s1 = s.squeeze(1)
                swL = swL * s1
                swR = swR * s1
                self._model_arms = arms
            self._swingL, self._swingR = swL, swR
            # The PURE model legs (exactly what the kinematic GHOST plays) --
            # the imitation reward pulls the actual pose onto THIS.
            self._model_legs = legs.clone()
            # Reference joint VELOCITIES (Disney/BD track these too -- gait
            # crispness): finite-difference of the model one step ahead.
            if self.r.get("rw_track_vel", 0.0) != 0.0:
                legs_n, _, _, _ = self._ghg.targets_torch(
                    self.phase_t + self.phase_dt, self.gp,
                    t_since_start_t=t_since + DT)
                self._model_legs_qd = (legs_n - self._model_legs) / DT
            targets = legs
            targets[:, _L_AP] = targets[:, _L_AP] + ap
            targets[:, _R_AP] = targets[:, _R_AP] + ap
            targets[:, _L_AR] = targets[:, _L_AR] + ar
            targets[:, _R_AR] = targets[:, _R_AR] + ar
            return targets, roll, pitch
        targets = self.nominal_t.unsqueeze(0).expand(self.n, -1).contiguous()
        targets[:, _L_AP] = targets[:, _L_AP] + ap
        targets[:, _R_AP] = targets[:, _R_AP] + ap
        targets[:, _L_AR] = targets[:, _L_AR] + ar
        targets[:, _R_AR] = targets[:, _R_AR] + ar
        if STAND_LEAN:
            # Deterministic reactive lean (== humanoid_stand_deploy): lean the
            # ankles BACK + thighs back when pitching/moving forward. kv*vx is the
            # fast term that catches the forward-CoM tip the ankle PD can't hold.
            vx = self.qvel_t[:, 0]
            fwd = torch.clamp(LEAN_KV * vx + LEAN_KP * (-pitch) + LEAN_KD * (-pitch_rate),
                              -LEAN_CLAMP, LEAN_CLAMP)
            targets[:, _L_AP] = targets[:, _L_AP] - fwd      # negative ankle = lean back
            targets[:, _R_AP] = targets[:, _R_AP] - fwd
            targets[:, _L_HP] = targets[:, _L_HP] + fwd * LEAN_HIP   # thigh back = CoM back
            targets[:, _R_HP] = targets[:, _R_HP] + fwd * LEAN_HIP
        # WALK: open-loop gait reference (left leg phase th, right leg th+pi).
        #   hip_pitch += -A_HIP*sin(th)  ; knee += A_KNEE*relu(sin(th))
        th = self.phase_t
        sL = torch.sin(th); sR = torch.sin(th + math.pi)
        targets[:, _L_HP] = targets[:, _L_HP] - self.gait_a_hip * sL
        targets[:, _R_HP] = targets[:, _R_HP] - self.gait_a_hip * sR
        targets[:, _L_KN] = targets[:, _L_KN] + self.gait_a_knee * torch.clamp(sL, min=0.0)
        targets[:, _R_KN] = targets[:, _R_KN] + self.gait_a_knee * torch.clamp(sR, min=0.0)
        if self.gait_a_ankle != 0.0:
            # Counter-rotate the ankle so the foot stays flat through the hip
            # swing (cancels the -A_HIP*sin term in the sagittal chain).
            targets[:, _L_AP] = targets[:, _L_AP] + self.gait_a_ankle * sL
            targets[:, _R_AP] = targets[:, _R_AP] + self.gait_a_ankle * sR
        if self.gait_a_push != 0.0:
            # GAIT-V2 push-off: a squared half-sine plantarflex bump peaking
            # at the END of each leg's stance (left stance = th in [pi,2pi],
            # peak at 2pi; right offset by pi). Signed amplitude (ankle
            # plantarflex direction depends on the model's sign convention).
            pushL = torch.clamp(torch.sin(th - 1.5 * math.pi), min=0.0) ** 2
            pushR = torch.clamp(torch.sin(th - 0.5 * math.pi), min=0.0) ** 2
            targets[:, _L_AP] = targets[:, _L_AP] + self.gait_a_push * pushL
            targets[:, _R_AP] = targets[:, _R_AP] + self.gait_a_push * pushR
        # Optional lateral weight-shift (hip-roll sway, both legs same sign so the
        # pelvis translates toward the stance foot). Default 0; enable for v3 if the
        # policy keeps falling sideways during single-support.
        if self.gait_a_lat != 0.0:
            # Lateral weight-shift sway (sine; both legs same sign -> pelvis shifts
            # toward the stance foot). The big sway (a_lat~0.22) is what let the G1
            # walk on its narrow 6cm foot. (A tanh "square" sway was tried and was
            # slightly worse, so the simple sine is kept.)
            sway = self.gait_a_lat * sL
            targets[:, _L_HR] = targets[:, _L_HR] + sway
            targets[:, _R_HR] = targets[:, _R_HR] + sway
        if self.cp_gain != 0.0:
            # CAPTURE-POINT lateral foot placement: place the SWING foot in the
            # direction of the lateral CoM velocity to CATCH a sideways fall (you
            # can't balance a 6cm foot with the ankle alone -> you must step). Applied
            # to whichever leg is currently swinging (relu of its sin).
            cp = self.cp_gain * self.qvel_t[:, 1]   # vy = lateral world velocity
            targets[:, _L_HR] = targets[:, _L_HR] + cp * torch.clamp(sL, min=0.0)
            targets[:, _R_HR] = targets[:, _R_HR] + cp * torch.clamp(sR, min=0.0)
        return targets, roll, pitch

    def _stack_obs(self, obs_now, reset_mask=None):
        """Frame-stacked observation (MEMORY probe): concat the last K obs,
        newest first. K=1 -> passthrough. Envs in reset_mask get their
        history refilled with the fresh obs (no cross-episode leakage).
        Deploy must stack in the SAME newest-first order (G1_OBS_STACK)."""
        K = self.obs_stack
        if K <= 1:
            return obs_now
        if self._obs_buf is None or self._obs_buf.shape[0] != obs_now.shape[0]:
            self._obs_buf = obs_now.unsqueeze(1).repeat(1, K, 1).contiguous()
        else:
            self._obs_buf = torch.roll(self._obs_buf, 1, dims=1)
            self._obs_buf[:, 0] = obs_now
            if reset_mask is not None and reset_mask.any():
                self._obs_buf[reset_mask] = \
                    obs_now[reset_mask].unsqueeze(1).expand(-1, K, -1)
        return self._obs_buf.reshape(obs_now.shape[0], -1)

    def _lookahead_obs(self):
        """EXACT future reference: model leg targets at +dt for each
        configured lookahead. Real info beyond phase sin/cos during the
        launch ramp (t_since dependency) and a capacity shortcut after.
        Deploy must compute the SAME block (G1_OBS_LOOKAHEAD)."""
        t_since = self._ramp_t0 + self.ep_step_t.to(torch.float32) * DT
        om = 2.0 * math.pi * self.gait_freq
        # velocity-conditioned: the forecast scales with the commanded speed
        # (matches the scaled baseline; 0 -> the standing reference).
        s = (torch.clamp(self.vx_cmd_t / max(self.vx_target, 1e-3), 0.0, 1.25)
             .unsqueeze(1) if self.vx_cond else 1.0)
        parts = []
        for dt_s in self.obs_look:
            legs, _, _, _ = self._ghg.targets_torch(
                torch.remainder(self.phase_t + om * dt_s, 2.0 * math.pi),
                self.gp, t_since_start_t=t_since + dt_s)
            parts.append(s * (legs - self.nominal_t.unsqueeze(0)))
        return torch.cat(parts, dim=1)

    def _obs_full(self, reset_mask=None):
        obs = self._stack_obs(self._build_obs_t(), reset_mask=reset_mask)
        if self.obs_look and self.gait_model == "human":
            obs = torch.cat([obs, self._lookahead_obs()], dim=1)
        if self.vx_cond:
            # the commanded speed (normalised), appended ONCE at the end so an
            # old (non-conditioned) checkpoint warm-starts by zero-padding 1 col
            obs = torch.cat([obs, (self.vx_cmd_t / self.vx_cmd_max).unsqueeze(1)],
                            dim=1)
        return obs

    def _leg_kp(self, q, hp, kn, ap, hr):
        """3D forward kinematics: leg joint angles -> Cartesian knee & toe
        positions relative to the hip, in BOTH the sagittal (x,z) and the
        FRONTAL (y, from hip ROLL) planes. The holistic SHAPE reward matches
        limb POSITIONS (the silhouette). Including hip roll captures the
        stance WIDTH / leg spread -- the frontal-plane shape a sagittal-only
        match left free (-> the legs splayed ~25 cm/side too wide)."""
        f = self._fk
        tt = f["TH"] + f["HS"] * q[:, hp]
        ts = tt + f["SH"] + f["KS"] * q[:, kn]
        kx = f["L1"] * torch.sin(tt)
        kz = -f["L1"] * torch.cos(tt)
        ax = kx + f["L2"] * torch.sin(ts)
        az = kz - f["L2"] * torch.cos(ts)
        fp = ts - q[:, ap]                     # world foot pitch (0 = flat)
        tx = ax + f["LF"] * torch.cos(fp)
        tz = az - f["LF"] * torch.sin(fp)
        # hip ROLL rotates the whole leg about the forward (x) axis -> lateral
        # (y) offset = -z*sin(roll). This is the stance width / leg spread.
        rr = q[:, hr]
        sr = torch.sin(rr); cr = torch.cos(rr)
        ky = -kz * sr; kz = kz * cr
        ty = -tz * sr; tz = tz * cr
        return kx, ky, kz, tx, ty, tz

    def priv_obs(self, actor_obs):
        """Critic-only input: actor obs + privileged sim signals (true base
        height, per-foot contact flag + height). Hidden from / only
        indirectly sensed by the 13-DOF actor; cheap from live tensors."""
        if not self.asym:
            return actor_obs
        bz = self.qpos_t[:, 2:3]
        fz = self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), 2]   # (n,2)
        contact = (fz < self.foot_z_contact).float()
        return torch.cat([actor_obs, bz, contact, fz], dim=1)

    def reset(self):
        self._reset_all()
        self._obs_buf = None
        return self._obs_full()

    # â”€â”€ GPU-native step: action in/out are torch tensors on cuda. â”€â”€
    def step(self, action_t):
        if _SP_ON:
            torch.cuda.synchronize(); _ts = time.time()
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
        targets = torch.clamp(
            baseline + self.act_gain_t.unsqueeze(1) * self.res_scale_vec * applied,
            self.jl_lo_t, self.jl_hi_t)
        # Write to mw_d.ctrl through the zero-copy torch view.
        self.ctrl_t.zero_()
        # ctrl_t[:, ctrl_pos_idx] = targets via index_copy on dim 1.
        self.ctrl_t.index_copy_(1, self.ctrl_pos_idx_t, targets)
        # Full-body mode: pin the arm position actuators at nominal so
        # the arms hold their pose (their vel actuators stay 0 from
        # zero_()). The arm mass is then a passive balance load on the
        # leg policy. GAIT-V2: with gait_a_arm != 0 the shoulder-pitch of
        # each arm SWINGS counter-phase to its same-side leg (deterministic
        # reference, still not policy-controlled) -- the human arm swing.
        if self.hold_arms:
            if self.gait_model == "human":
                # Arm targets come from the gait model (counter-phase swing +
                # soft elbows), computed in _baseline_targets_t this tick.
                self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, self._model_arms)
            elif self.gait_a_arm != 0.0:
                if not hasattr(self, "_arm_swing_buf"):
                    self._arm_swing_buf = self.arm_targets_t.clone()
                sw = self.gait_a_arm * torch.sin(self.phase_t)
                buf = self._arm_swing_buf
                buf.copy_(self.arm_targets_t)
                buf[:, 0] = buf[:, 0] + sw      # left_shoulder_pitch  (~ +a*sin(th))
                buf[:, 5] = buf[:, 5] - sw      # right_shoulder_pitch (anti-phase)
                self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, buf)
            else:
                self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, self.arm_targets_t)

        # Domain-randomized pushes â€” apply velocity impulses on GPU.
        if self._push_p > 0 and self._push_vmax > 0:
            hit = torch.rand(self.n, device=self.tdev) < self._push_p
            if hit.any():
                theta = torch.rand(self.n, device=self.tdev) * (2 * math.pi)
                mag = torch.rand(self.n, device=self.tdev) * self._push_vmax
                dvx = torch.cos(theta) * mag
                dvy = torch.sin(theta) * mag
                self.qvel_t[hit, 0] = self.qvel_t[hit, 0] + dvx[hit]
                self.qvel_t[hit, 1] = self.qvel_t[hit, 1] + dvy[hit]

        if _SP_ON:
            torch.cuda.synchronize(); _STEP_PROF["ctrl"] += time.time() - _ts; _ts = time.time()
        # Physics: replay the captured CUDA graph for SUBSTEPS substeps,
        # falling back to direct steps if capture wasn't possible.
        with self.wp.ScopedDevice(self.device):
            if self._cuda_graph is not None:
                self.wp.capture_launch(self._cuda_graph)
            else:
                for _ in range(SUBSTEPS):
                    self.mjw.step(self.mw_m, self.mw_d)
        if _SP_ON:
            torch.cuda.synchronize(); _STEP_PROF["phys"] += time.time() - _ts; _ts = time.time()

        self.ep_step_t = self.ep_step_t + 1
        prev_action_t = self.last_action_t
        self.last_action_t = action_t
        # Phase that produced THIS step's targets (reward schedule uses it).
        th_used = self.phase_t
        # ── ADAPTIVE / STATE-DEPENDENT PHASE (architecture change) ──
        # The gait clock is no longer fixed: it SLOWS when the body is
        # off-balance (tilted / tilting) so the robot LINGERS in the current
        # support to recover, and runs at full rate when balanced -> "step
        # when ready, hold when not". The policy modulates its own timing
        # indirectly: the better it keeps the torso upright, the faster the
        # gait advances. gate=1 (no slowdown) when the gate weights are 0, so
        # this is a no-op unless enabled. Uses the pre-physics tilt cached by
        # _baseline_targets_t (prev_roll/pitch + their rates).
        kt = self.r.get("phase_gate_tilt", 0.0)
        kr = self.r.get("phase_gate_rate", 0.0)
        if kt != 0.0 or kr != 0.0:
            tilt2 = self.prev_roll_t ** 2 + self.prev_pitch_t ** 2
            rate2 = self.prev_roll_rate_t ** 2 + self.prev_pitch_rate_t ** 2
            gate = torch.clamp(1.0 - kt * tilt2 - kr * rate2,
                               self.r.get("phase_gate_floor", 0.2), 1.0)
            self._phase_gate = gate            # logged / available to rewards
        else:
            gate = 1.0
        # VELOCITY-CONDITIONED phase freeze: scale the clock rate by the speed
        # command so it FREEZES as vx_cmd -> 0. A stand command then gives a
        # static stand (the legs are already at nominal via the s=0 amplitude
        # scaling); without this the phase keeps cycling at vx_cmd=0 and the
        # policy emits a periodic residual -> a ~0.13 m/s forward creep in
        # deploy. Real walking speeds (vx_cmd >= vx_phase_freeze) keep full rate.
        if self.vx_cond and self.vx_phase_freeze > 0.0:
            g_vx = torch.clamp(self.vx_cmd_t / self.vx_phase_freeze, 0.0, 1.0)
            gate = gate * g_vx
        # WALK: advance the gait clock for the next obs/baseline.
        self.phase_t = torch.remainder(self.phase_t + self.phase_dt * gate,
                                       2.0 * math.pi)

        # Velocity-command: occasionally RE-sample mid-episode so the policy
        # trains the start/stop TRANSITIONS (decelerate-to-stand, accelerate-
        # to-walk), not just steady speeds. ~once / 2.5 s per env.
        if self.vx_cond:
            chg = torch.rand(self.n, device=self.tdev) < (DT / 2.5)
            if chg.any():
                nc = self._sample_vx_cmd(self.n)
                self.vx_cmd_t = torch.where(chg, nc, self.vx_cmd_t)

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
        # WALK reward: track a forward velocity command while staying upright; the
        # gait reference (in the baseline) supplies the stepping rhythm, the policy
        # residual stabilises + propels. Forward = world +x (no turning command yet).
        vx = self.qvel_t[:, 0]            # world forward lin vel
        vy = self.qvel_t[:, 1]            # world lateral lin vel
        wz = self.qvel_t[:, 5]            # body yaw rate
        r_alive = r.get("alive", 1.0) * torch.ones(self.n, device=self.tdev)
        upright = torch.clamp(1.0 - roll * roll - pitch * pitch, min=0.0)
        r_up = r.get("upright", 0.5) * upright
        # Gaussian velocity-tracking: peak when vx == the COMMAND (per-env
        # vx_cmd_t when velocity-conditioned, else the fixed vx_target).
        _vtgt = self.vx_cmd_t if self.vx_cond else self.vx_target
        vsig = r.get("vel_sigma", 0.10)
        # COMMAND-SCALED sigma: at a stand command (vx_cmd=0) the gaussian is
        # too WIDE (vsig=0.10 -> a 0.13 m/s creep still scores 0.84, almost no
        # gradient to 0 -> the stand plateaus at a creep). Tighten the sigma as
        # vx_cmd -> 0 so the stand is tracked HARD, while walking stays loose.
        # vsig_eff = vsig_stand + (vsig - vsig_stand)*clamp(vx_cmd/vx_target).
        if self.vx_cond:
            vsig_stand = r.get("vel_sigma_stand", vsig)
            _f = torch.clamp(self.vx_cmd_t / max(self.vx_target, 1e-3), 0.0, 1.0)
            vsig = vsig_stand + (vsig - vsig_stand) * _f
        r_vel = r.get("vel", 2.0) * torch.exp(-((vx - _vtgt) ** 2) / vsig)
        # L1 velocity term: the gaussian is FLAT ZERO beyond |vx-target|~0.6,
        # so a runaway (vx 1.6 while target 0.4, observed) gets no velocity
        # gradient at all -- the L1 term supplies one everywhere.
        r_vel = r_vel + r.get("vel_l1", 0.0) * torch.abs(vx - _vtgt)
        r_lat = r.get("lat", -0.5) * torch.abs(vy)
        r_yaw = r.get("yaw", -0.5) * torch.abs(wz)
        z_ref = r.get("z_ref", 0.74)
        r_height = r.get("height", -10.0) * (bz - z_ref) ** 2
        r_act = r.get("act", -0.005) * (action_t * action_t).sum(dim=1)
        # (act_rate previously compared action_t against itself -- last_action_t
        # was overwritten above before this read -- so the smoothness penalty
        # was silently ZERO for every run up to and including v5/walk12.)
        r_rate = r.get("act_rate", -0.01) * ((action_t - prev_action_t) ** 2).sum(dim=1)
        reward = (r_alive + r_up + r_vel + r_lat + r_yaw + r_height + r_act + r_rate)

        # â”€â”€ IMITATION reward: pay the policy to keep its ACTUAL pose on the
        # gait MODEL (exactly what the kinematic ghost plays). Without this the
        # reference is only a feed-forward baseline the +-RES_SCALE residual can
        # override, and the reward (vel + upright + foot schedule) is happy with
        # ANY stable forward gait -- so the policy drifts off the human shape.
        # err = mean squared (q_actual - q_model) over the tracked leg joints;
        # gaussian so perfect tracking = +rw_track, large deviation -> 0.
        # â”€â”€ SWING-leg STYLE reward: the measured style gap is AMPLITUDE (w5
        # does the human waveform at 0.25x knee / 0.38x hip size, shape corr
        # ~0.8). The swing leg is UNLOADED, so demanding it match the model
        # posture costs no balance; the stance leg (where balance lives) stays
        # free, and so does timing. Pays swing_weight * exp(-err/sigma) per
        # leg over hip pitch + knee only.
        rw_sw = r.get("rw_swing_track", 0.0)
        if rw_sw != 0.0 and self.gait_model == "human" \
                and getattr(self, "_model_legs", None) is not None:
            q_act_sw = self.qpos_t.index_select(1, self.qpos_idx_t)
            dm = q_act_sw - self._model_legs
            sig_sw = r.get("track_sigma", 0.04)
            errL = dm[:, _L_HP] ** 2 + dm[:, _L_KN] ** 2
            errR = dm[:, _R_HP] ** 2 + dm[:, _R_KN] ** 2
            # Optional: swing ankle-pitch in the style err. Deploy shows the
            # ankle OVERSHOOTS the model 1.3-1.5x (balance-PD flap); a gentle
            # pull calms it during swing where the PD has no work to do.
            w_ank = r.get("swing_track_ankle", 0.0)
            if w_ank != 0.0:
                errL = errL + w_ank * dm[:, _L_AP] ** 2
                errR = errR + w_ank * dm[:, _R_AP] ** 2
            r_sw = rw_sw * (self._swingL * torch.exp(-errL / sig_sw)
                            + self._swingR * torch.exp(-errR / sig_sw))
            reward = reward + r_sw

        # ── HOLISTIC SHAPE reward: match the ghost's SILHOUETTE, not specific
        # joints. FK the swing leg's hip/knee/ankle to the sagittal positions
        # of the KNEE and TOE, for both actual robot and model, and pay ONE
        # distance over the whole shape. The robot is free to hit that
        # silhouette with any joint combination (redundancy -> balance), and
        # matching the TOE pins foot ORIENTATION so the ankle can't run away
        # without moving the visible toe -- fixes the over-drive via the shape.
        rw_shape = r.get("rw_shape", 0.0)
        if rw_shape != 0.0 and self.gait_model == "human" \
                and getattr(self, "_model_legs", None) is not None:
            qa = self.qpos_t.index_select(1, self.qpos_idx_t)
            qm = self._model_legs
            sig_sh = r.get("shape_sigma", 0.004)
            # 3D keypoints: (kx, ky, kz, tx, ty, tz) per leg.
            akL = self._leg_kp(qa, _L_HP, _L_KN, _L_AP, _L_HR)
            mkL = self._leg_kp(qm, _L_HP, _L_KN, _L_AP, _L_HR)
            akR = self._leg_kp(qa, _R_HP, _R_KN, _R_AP, _R_HR)
            mkR = self._leg_kp(qm, _R_HP, _R_KN, _R_AP, _R_HR)
            # SAGITTAL match (x,z = idx 0,2,3,5): swing-weighted, leaves the
            # stance leg free sagittally for balance (as before).
            sag = (0, 2, 3, 5)
            errL = sum((akL[i] - mkL[i]) ** 2 for i in sag)
            errR = sum((akR[i] - mkR[i]) ** 2 for i in sag)
            r_shape = rw_shape * (self._swingL * torch.exp(-errL / sig_sh)
                                  + self._swingR * torch.exp(-errR / sig_sh))
            reward = reward + r_shape
            # FRONTAL/LATERAL match (y = idx 1,4 = the stance WIDTH): applied
            # to BOTH legs, NOT swing-weighted -- the leg-spread is a stance
            # problem, so the planted leg must be pulled narrow too. Model
            # hip-roll ~0 -> this pulls the feet back under the hips.
            rw_lat = r.get("rw_shape_lat", 0.0)
            if rw_lat != 0.0:
                sig_lat = r.get("shape_lat_sigma", sig_sh)
                latL = (akL[1] - mkL[1]) ** 2 + (akL[4] - mkL[4]) ** 2
                latR = (akR[1] - mkR[1]) ** 2 + (akR[4] - mkR[4]) ** 2
                reward = reward + rw_lat * (torch.exp(-latL / sig_lat)
                                            + torch.exp(-latR / sig_lat))

        # ── COM-OVER-STANCE-FOOT reward: the literal "balance over the stance
        # leg". During single support the body's LATERAL position (pelvis y,
        # COM proxy) should be over the PLANTED foot -- when the left leg
        # swings the COM should be over the RIGHT foot (weight=swingL), and
        # vice versa. This is the deliberate WEIGHT TRANSFER the robot never
        # learned (the root cause of the leg-spread + drunk gait); a soft
        # capture-point/ZMP objective. Should narrow the stance NATURALLY (a
        # supported COM doesn't need a wide base) instead of forcing it.
        rw_com = r.get("rw_com", 0.0)
        if rw_com != 0.0 and getattr(self, "_swingL", None) is not None:
            py = self.qpos_t[:, 1]                                  # pelvis y
            fy = self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), 1]
            sig_c = r.get("com_sigma", 0.004)
            dR = (py - fy[:, 1]) ** 2          # COM-to-RIGHT-foot (left swings)
            dL = (py - fy[:, 0]) ** 2          # COM-to-LEFT-foot  (right swings)
            reward = reward + rw_com * (self._swingL * torch.exp(-dR / sig_c)
                                        + self._swingR * torch.exp(-dL / sig_c))

        # ── TORSO-STILLNESS penalty: kill the drunk wobble. Penalize the
        # body's roll+pitch ANGULAR VELOCITY (not just tilt) -> a steady,
        # carried posture instead of constant reactive catching.
        rw_torso = r.get("rw_torso", 0.0)
        if rw_torso != 0.0:
            reward = reward + rw_torso * (self.prev_roll_rate_t ** 2
                                          + self.prev_pitch_rate_t ** 2)

        # ── FRONTAL-track: pull hip-ROLL + hip-YAW onto the (improved) shadow.
        # The improved shadow (lateral/yaw modes) carries a principled, FEASIBLE
        # lateral weight transfer + hip rotation; this pays the policy to keep
        # its actual hip-roll/yaw ON that target instead of cancelling it with
        # the residual or splaying. Unlike the old "narrow to zero" lateral
        # rewards, the target IS the balance motion, so it should not fight
        # balance. Applied to BOTH legs (lateral lives in stance).
        rw_front = r.get("rw_frontal_track", 0.0)
        if rw_front != 0.0 and getattr(self, "_model_legs", None) is not None:
            qf = self.qpos_t.index_select(1, self.qpos_idx_t)
            dmf = qf - self._model_legs
            sig_f = r.get("frontal_sigma", 0.01)
            errL = dmf[:, _L_HR] ** 2 + dmf[:, _L_HY] ** 2
            errR = dmf[:, _R_HR] ** 2 + dmf[:, _R_HY] ** 2
            reward = reward + rw_front * (torch.exp(-errL / sig_f)
                                          + torch.exp(-errR / sig_f))

        rw_track = r.get("rw_track", 0.0)
        rw_tv = r.get("rw_track_vel", 0.0)
        track_et = r.get("track_et", 0.0)
        self._et_mask = None
        if (rw_track != 0.0 or rw_tv != 0.0 or track_et > 0.0) \
                and getattr(self, "_model_legs", None) is not None:
            q_act = self.qpos_t.index_select(1, self.qpos_idx_t)
            # hips + knees are the visible human signature; weight ankles less
            # (they carry the balance PD) and skip the waist.
            err = (q_act - self._model_legs) ** 2
            w_trk = self._track_w                       # (NJ,) per-joint weight
            track_err = (err * w_trk).sum(dim=1) / w_trk.sum()
            if rw_track != 0.0:
                r_track = rw_track * torch.exp(-track_err / r.get("track_sigma", 0.05))
                reward = reward + r_track
            # Joint-velocity imitation (Disney: small weight; crispness).
            if rw_tv != 0.0 and getattr(self, "_model_legs_qd", None) is not None:
                qd_act = self.qvel_t.index_select(1, self.qvel_idx_t)
                errv = ((qd_act - self._model_legs_qd) ** 2 * w_trk).sum(dim=1) \
                    / w_trk.sum()
                reward = reward + rw_tv * torch.exp(-errv / r.get("track_vel_sigma", 4.0))
            # DeepMimic-style deviation early-termination: exploration that
            # wanders off the style is cut, so "stylish AND stable" is the
            # only optimum. Mean |err| over hips+knees; grace for launch/sag.
            if track_et > 0.0:
                dev = (q_act - self._model_legs)[:, self._hipkn_idx].abs().mean(dim=1)
                self._et_mask = dev > track_et

        # â”€â”€ Foot-aware shaping (gated; needs xpos) â”€â”€
        if self.rw_sched != 0.0 or self.rw_slip != 0.0:
            feet = self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), :]  # (n,2,3)
            foot_z = feet[:, :, 2]
            foot_xy = feet[:, :, 0:2]
            if self.gait_model == "human":
                # Swing weights straight from the gait model (60/40 duty),
                # computed in _baseline_targets_t from the SAME phase.
                swing = torch.stack([self._swingL, self._swingR], dim=1)
            else:
                sL = torch.sin(th_used)
                swing = torch.stack([torch.clamp(sL, min=0.0),
                                     torch.clamp(-sL, min=0.0)], dim=1)   # (n,2)
            stance = 1.0 - swing
            if self.rw_sched != 0.0:
                # Penalize: foot OFF the ground during its stance half, foot
                # NOT lifted toward the clearance target at its swing peak.
                z_hi = torch.clamp(foot_z - self.foot_z_contact, min=0.0)
                z_lo = torch.clamp(self.foot_z_swing - foot_z, min=0.0)
                sched_pen = (stance * z_hi + swing * z_lo).sum(dim=1)
                reward = reward + self.rw_sched * sched_pen
            if self.rw_slip != 0.0:
                # Penalize planted-foot xy velocity (slip / skating).
                v_xy = (foot_xy - self.prev_foot_xy_t) / DT
                contact = (foot_z < self.foot_z_contact).float()
                slip_pen = (contact * torch.linalg.norm(v_xy, dim=2)).sum(dim=1)
                reward = reward + self.rw_slip * slip_pen
            self.prev_foot_xy_t = foot_xy.clone()

        fall = (torch.abs(roll) > ROLL_FAIL) | (torch.abs(pitch) > PITCH_FAIL) | (bz < BZ_FAIL)
        if getattr(self, "_et_mask", None) is not None:
            # grace period: rest-starts + sag jitter begin off-reference and
            # need the whole launch ramp to converge onto the gait -- ET
            # inside the ramp makes the LAUNCH unlearnable (m2 fell at 1s).
            _grace = int(r.get("track_et_grace", 3.0) / DT)
            fall = fall | (self._et_mask & (self.ep_step_t > _grace))
        done = fall | (self.ep_step_t >= self.max_ep)
        reward = reward + fall.float() * r.get("term", -1.0)

        if _SP_ON:
            torch.cuda.synchronize(); _STEP_PROF["rew"] += time.time() - _ts; _ts = time.time()
        if done.any():
            self._reset_envs(env_mask=done)
        if _SP_ON:
            torch.cuda.synchronize(); _STEP_PROF["reset"] += time.time() - _ts; _ts = time.time()

        obs = self._obs_full(reset_mask=done)
        if _SP_ON:
            torch.cuda.synchronize(); _STEP_PROF["obs"] += time.time() - _ts; _STEP_PROF["n"] += 1
        info = {"bz": bz, "roll": roll, "pitch": pitch}
        return obs, reward, done, info


# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# PPO loop â€” same compact shape as gpu_mjwarp_residual_trainer.main.
# â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
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
                   default=str(REPO / "projects/policies/research/training/runs/gpu_g1_walk/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=512)
    p.add_argument("--alive", type=float, default=1.0)
    p.add_argument("--upright", type=float, default=0.5)
    p.add_argument("--lin", type=float, default=-0.1)
    p.add_argument("--ang", type=float, default=-0.05)
    p.add_argument("--act", type=float, default=-0.005)
    p.add_argument("--act-rate", type=float, default=-0.01,
                   help="penalty on ||a_t - a_{t-1}||^2 -> SMOOTH residual")
    p.add_argument("--term", type=float, default=-10.0,
                   help="terminal fall penalty (strong so the policy values not falling)")
    # â”€â”€ WALK reward + gait knobs â”€â”€
    p.add_argument("--vel", type=float, default=2.0, help="forward-velocity tracking weight")
    p.add_argument("--vel-sigma", type=float, default=0.10, help="velocity-tracking gaussian width")
    p.add_argument("--vel-sigma-stand", type=float, default=0.10,
                   help="VELOCITY-CONDITIONED: the velocity-tracking gaussian "
                        "width AT a stand command (vx_cmd=0); interpolates up to "
                        "--vel-sigma at vx_cmd>=vx_target. Smaller = a TIGHTER "
                        "stand (kills the stand creep). E.g. 0.015.")
    p.add_argument("--vel-l1", type=float, default=0.0,
                   help="L1 |vx-target| penalty weight (NEGATIVE, try -0.3): global "
                        "velocity gradient where the gaussian saturates to zero")
    p.add_argument("--vx-target", type=float, default=0.4, help="commanded forward velocity m/s")
    p.add_argument("--vx-cmd-max", type=float, default=0.0,
                   help="VELOCITY-CONDITIONED walking: if >0, the policy is "
                        "given a per-env speed COMMAND in [0, this] (incl. 0 = "
                        "stand); gait amplitude + reward track it. Commanding 0 "
                        "-> decelerate and stand; the stop/start milestone in "
                        "ONE policy. Set ~vx_target (e.g. 0.45). Obs += 1.")
    p.add_argument("--vx-phase-freeze", type=float, default=0.10,
                   help="VELOCITY-CONDITIONED: speed below which the gait CLOCK "
                        "freezes (step frequency drops with speed). At vx_cmd=0 "
                        "the phase stops -> a STATIC stand instead of a cycling "
                        "phase that drives a residual creep. Deploy must match "
                        "(G1_VX_PHASE_FREEZE).")
    p.add_argument("--lat", type=float, default=-0.5, help="lateral-velocity penalty weight")
    p.add_argument("--yaw", type=float, default=-0.5, help="yaw-rate penalty weight")
    p.add_argument("--height", type=float, default=-10.0, help="pelvis-height deviation penalty weight")
    p.add_argument("--z-ref", type=float, default=0.74, help="target pelvis height during walk")
    p.add_argument("--gait-freq", type=float, default=1.3, help="gait reference frequency Hz")
    p.add_argument("--gait-a-hip", type=float, default=0.35, help="gait hip-pitch amplitude rad")
    p.add_argument("--gait-a-knee", type=float, default=0.45, help="gait knee-lift amplitude rad")
    p.add_argument("--gait-a-lat", type=float, default=0.0,
                   help="lateral hip-roll sway amplitude rad (0=off; try 0.06 if it falls sideways)")
    p.add_argument("--cp-gain", type=float, default=0.0,
                   help="capture-point lateral foot-placement gain (swing hip-roll += gain*vy). "
                        "try 0.5; flip sign if it falls sideways worse")
    p.add_argument("--max-ep", type=int, default=MAX_EP,
                   help="episode cap in env-steps (500=8s; raise to 1250+ so the "
                        "policy practises SUSTAINED walking, not just the first 8 s)")
    p.add_argument("--gait-a-ankle", type=float, default=0.0,
                   help="ankle-pitch counter-rotation amplitude rad (keeps the foot "
                        "flat through the hip swing; 0.35 turns the open-loop CPG "
                        "from backward-drifting to forward-walking at stiff gains)")
    p.add_argument("--seed-gait-pose", action="store_true",
                   help="seed each reset IN the gait pose (q+qd at the sampled "
                        "phase) instead of NOMINAL -- kills the reset target-snap "
                        "artifact at stiff gains")
    p.add_argument("--rw-sched", type=float, default=0.0,
                   help="contact-schedule reward weight (NEGATIVE penalty, try -5: "
                        "stance foot must be down, swing foot must lift)")
    p.add_argument("--rw-slip", type=float, default=0.0,
                   help="planted-foot slip penalty weight (NEGATIVE, try -0.5)")
    p.add_argument("--rw-track", type=float, default=0.0,
                   help="IMITATION reward weight (POSITIVE, try 1.0-2.0): pays "
                        "the policy to keep its actual leg pose ON the gait MODEL "
                        "(what the ghost plays) so the real robot looks human, not "
                        "just stable. exp(-mse/track_sigma).")
    p.add_argument("--track-sigma", type=float, default=0.05,
                   help="imitation gaussian width (rad^2); smaller = stricter")
    p.add_argument("--obs-lookahead", type=str, default="",
                   help="comma seconds of EXACT future-reference targets in "
                        "the obs (e.g. '0.1,0.4') -- the forecast block. "
                        "Deploy must set G1_OBS_LOOKAHEAD identically.")
    p.add_argument("--asym-critic", action="store_true",
                   help="ASYMMETRIC actor-critic: critic sees privileged sim "
                        "signals (base height, foot contacts/heights) for a "
                        "lower-variance value. Actor + exported ONNX UNCHANGED.")
    p.add_argument("--hidden-dims", type=str, default="256,128",
                   help="comma MLP hidden layer sizes for BOTH actor and critic "
                        "(default '256,128'; the long run uses '512,512,512').")
    p.add_argument("--obs-stack", type=int, default=1,
                   help="frame-stack K: policy sees the last K obs (newest "
                        "first) -- MEMORY for actuator-lag compensation. "
                        "Deploy must set G1_OBS_STACK to the same K.")
    p.add_argument("--rw-track-vel", type=float, default=0.0,
                   help="joint-VELOCITY imitation weight (Disney-style, small "
                        "e.g. 0.3): crispness of the gait motion")
    p.add_argument("--track-vel-sigma", type=float, default=4.0,
                   help="velocity-imitation gaussian width ((rad/s)^2)")
    p.add_argument("--track-et", type=float, default=0.0,
                   help="DeepMimic-style deviation early-termination: episode "
                        "ends (as a fall) when mean |q-q_model| over hips+knees "
                        "exceeds this (rad, try 0.5); 0 = off")
    p.add_argument("--track-et-grace", type=float, default=3.0,
                   help="seconds before deviation-ET arms (must exceed the "
                        "launch ramp so rest-starts stay learnable)")
    p.add_argument("--gait-hip-scale", type=float, default=0.75,
                   help="winter hip amplitude scale (default 0.75 = nominal). "
                        "Raise (~0.9) for feed-forward pre-compensation: the "
                        "soft joints under-achieve, so a bigger commanded arc "
                        "lands the ACHIEVED arc on the nominal model. Deploy "
                        "must set G1_GAIT_HIP_SCALE to the same value.")
    p.add_argument("--swing-track-ankle", type=float, default=0.0,
                   help="weight of ankle-pitch inside the swing style err "
                        "(try 0.5): calms the deploy ankle overshoot")
    p.add_argument("--rw-shape", type=float, default=0.0,
                   help="HOLISTIC SHAPE reward (POSITIVE, try 2.5): match the "
                        "ghost SILHOUETTE -- FK the swing leg to knee+toe "
                        "Cartesian positions and pay one distance, vs matching "
                        "specific joint angles. Frees the robot to hit the "
                        "shape in a balanced way + the toe pins the ankle.")
    p.add_argument("--shape-sigma", type=float, default=0.01,
                   help="shape gaussian width (m^2 over 4 keypoint coords)")
    p.add_argument("--rw-com", type=float, default=0.0,
                   help="COM-OVER-STANCE-FOOT reward (POSITIVE, try 1.5-2.5): "
                        "during single support, pay for the pelvis (COM) being "
                        "laterally over the PLANTED foot -- the deliberate "
                        "weight transfer that fixes the drunk gait + leg spread "
                        "at the root. exp(-(py-stance_foot_y)^2/com_sigma).")
    p.add_argument("--com-sigma", type=float, default=0.004,
                   help="COM-over-stance gaussian width (m^2)")
    p.add_argument("--rw-torso", type=float, default=0.0,
                   help="torso-stillness penalty (NEGATIVE, try -0.5): penalize "
                        "roll+pitch angular velocity -> kill the wobble")
    p.add_argument("--phase-gate-tilt", type=float, default=0.0,
                   help="ADAPTIVE PHASE: slow the gait clock by this * "
                        "(roll^2+pitch^2) when off-balance (try 3-6) -> the "
                        "robot lingers in support to recover, steps when "
                        "upright. Deploy must set G1_PHASE_GATE_TILT to match.")
    p.add_argument("--phase-gate-rate", type=float, default=0.0,
                   help="adaptive phase: slow by this * (roll_rate^2+"
                        "pitch_rate^2) when TILTING (try 0.02-0.05)")
    p.add_argument("--phase-gate-floor", type=float, default=0.2,
                   help="min gait-clock rate factor (never freeze; default 0.2)")
    p.add_argument("--rw-shape-lat", type=float, default=0.0,
                   help="FRONTAL-plane shape reward (POSITIVE, try 2.0-3.0): "
                        "match the lateral (y) knee+toe position of BOTH legs "
                        "to the model -> pulls the splayed legs back under the "
                        "hips (fixes the ~25cm/side over-wide stance). Needs "
                        "the deploy to handle narrow-stance lateral balance.")
    p.add_argument("--shape-lat-sigma", type=float, default=0.01,
                   help="frontal shape gaussian width (m^2 over 2 y-coords)")
    p.add_argument("--rw-swing-track", type=float, default=0.0,
                   help="SWING-leg style reward (POSITIVE, try 2.0): match the "
                        "model hip+knee posture on the UNLOADED swing leg only -- "
                        "fixes amplitude muting without fighting balance (stance "
                        "leg + timing stay free)")
    p.add_argument("--foot-z-contact", type=float, default=0.05,
                   help="ankle_roll z below which the foot counts as planted (m)")
    p.add_argument("--foot-z-swing", type=float, default=0.14,
                   help="ankle_roll z the swing foot should reach at peak (m)")
    p.add_argument("--res-scale", type=float, default=RES_SCALE,
                   help="residual action scale rad (0.3 for gait-residual; ~0.5 "
                        "for policy-owned gait with all gait amplitudes 0). "
                        "Deploy MUST set G1_ACT_SCALE to the same value.")
    # â”€â”€ GAIT-V2 (natural gait) knobs â”€â”€
    p.add_argument("--nominal-hip", type=float, default=float(NOMINAL[0]),
                   help="nominal hip-pitch rad (deep-squat default -0.30; tall "
                        "human posture ~ -0.16). Deploy: G1_NOM_HIP.")
    p.add_argument("--nominal-knee", type=float, default=float(NOMINAL[3]),
                   help="nominal knee rad (default 0.52; tall ~0.32). Deploy: G1_NOM_KNEE.")
    p.add_argument("--nominal-ankle", type=float, default=float(NOMINAL[4]),
                   help="nominal ankle-pitch rad (default -0.23; keep foot flat: "
                        "~ -(hip+knee)/1). Deploy: G1_NOM_ANKLE.")
    p.add_argument("--gait-a-arm", type=float, default=0.0,
                   help="counter-phase shoulder-pitch arm-swing amplitude rad "
                        "(hold-arms mode; try 0.25; signed). Deploy: G1_GAIT_A_ARM.")
    p.add_argument("--gait-a-push", type=float, default=0.0,
                   help="late-stance ankle push-off bump amplitude rad (signed; "
                        "try -0.15). Deploy: G1_GAIT_A_PUSH.")
    p.add_argument("--rest-start-frac", type=float, default=0.0,
                   help="fraction of episodes starting EXACTLY like the deploy "
                        "handover (standing at NOMINAL, zero vel, phase 0, no "
                        "jitter). Fixes the recurring deploy LAUNCH gap; try 0.3.")
    # â”€â”€ HUMAN GAIT MODEL (foot-space planned, IK-realized; replaces the CPG) â”€â”€
    p.add_argument("--gait-model", default="", choices=["", "human"],
                   help="'human' = projects/policies/control/gait/g1_human_gait.py reference: "
                        "stance foot under the pelvis at -vx, quintic swing arc, "
                        "60/40 duty, inverted-pendulum pelvis bob (TALL stance), "
                        "IK legs, counter-phase arms, stride ramp from standing. "
                        "Overrides --nominal-*/--gait-a-* (uses --vx-target, "
                        "--gait-freq, --gait-a-lat as sway, --gait-a-arm).")
    p.add_argument("--gait-duty", type=float, default=0.6,
                   help="human model: stance fraction of the leg cycle")
    p.add_argument("--gait-step-height", type=float, default=0.05,
                   help="human model: swing apex clearance m")
    p.add_argument("--gait-pelvis-h", type=float, default=0.755,
                   help="human model: MEAN pelvis height m (bob adds on top)")
    p.add_argument("--gait-bob", type=float, default=0.020,
                   help="human model: pelvis vertical bob amplitude m")
    p.add_argument("--gait-x0", type=float, default=-0.02,
                   help="human model: stride center vs hip anchor m")
    p.add_argument("--gait-ramp-s", type=float, default=1.0,
                   help="human model: stride ramp-in seconds from standing")
    p.add_argument("--gait-elbow", type=float, default=0.15,
                   help="human model: constant elbow bend rad")
    p.add_argument("--gait-ankle-clear", type=float, default=0.08,
                   help="human model: swing toe-up foot pitch rad")
    p.add_argument("--gait-style", default="ik", choices=["ik", "winter"],
                   help="human-model reference style: 'ik' = foot-space plan + "
                        "leg IK; 'winter' = MEASURED human joint kinematics "
                        "(Winter normative curves: knee double-bend + ankle "
                        "push-off -- the signatures the eye reads as human). "
                        "Deploy: G1_GAIT_STYLE.")
    # ── IMPROVED-SHADOW (frontal/transverse) -- see docs/developer/g1-improved-shadow.md
    p.add_argument("--gait-lateral", default="sway", choices=["sway", "lipm", "human"],
                   help="frontal-plane mode of the gait reference: 'sway' (legacy "
                        "tiny hip-roll sine), 'lipm' (A: LIPM weight transfer + "
                        "flat-foot ankle-roll), 'human' (C: measured hip ab/adduction). "
                        "Deploy: G1_GAIT_LATERAL.")
    p.add_argument("--gait-yaw", default="none", choices=["none", "human"],
                   help="transverse-plane (hip-yaw) mode: 'none' or 'human' (C: "
                        "measured hip rotation -- fills the previously-zero yaw). "
                        "Deploy: G1_GAIT_YAW.")
    p.add_argument("--gait-lat-hip-amp", type=float, default=0.09,
                   help="(A lipm) peak hip-roll for LIPM weight transfer rad")
    p.add_argument("--gait-step-width", type=float, default=0.12,
                   help="(A lipm) lateral foot separation m (LIPM shape)")
    p.add_argument("--rw-frontal-track", type=float, default=0.0,
                   help="reward to pull hip-ROLL+YAW onto the improved shadow "
                        "(both legs). Use WITH --gait-lateral/--gait-yaw so the "
                        "policy tracks the new lateral motion instead of splaying.")
    p.add_argument("--frontal-sigma", type=float, default=0.01,
                   help="gaussian width (rad^2) for --rw-frontal-track")
    p.add_argument("--frontal-res-scale", type=float, default=1.0,
                   help="scale the policy RESIDUAL authority on hip-roll/yaw "
                        "(1.0=unchanged). <1 stops the policy splaying on top of "
                        "the commanded weight transfer. Deploy: G1_FRONTAL_RES_SCALE.")
    p.add_argument("--sim-dt", type=float, default=0.0)
    p.add_argument("--init-from", default=None)
    p.add_argument("--ent-coef", type=float, default=0.01,
                   help="PPO entropy bonus. At 0.01 the learned log_std GROWS "
                        "(0.37->0.43 observed) and the per-step action noise "
                        "(~0.13 rad on every joint) destabilises the walker during "
                        "rollouts -- anneal to 0.003 or 0 in later warm-start chunks")
    p.add_argument("--log-std-clamp", type=float, default=None,
                   help="upper bound on log_std (e.g. -1.2 caps action noise std "
                        "at 0.30); applied after each update")
    # Domain randomization â€” defaults tuned for sim-to-deploy robustness.
    # The deploy wrapper introduces 1-tick control-delay + per-step state
    # sync that the trainer's raw mjw.step doesn't see. Training over the
    # union of mass/friction/gain/latency/push variation produces a
    # policy that doesn't care about which specific wrapper runs it.
    p.add_argument("--dr-mass-scale", type=float, default=0.30,
                   help="Â±fraction PER-BODY mass+inertia jitter")
    p.add_argument("--dr-friction-scale", type=float, default=0.50,
                   help="Â±fraction ground friction jitter")
    p.add_argument("--dr-damping-scale", type=float, default=0.50,
                   help="Â±fraction PER-DOF joint damping jitter")
    p.add_argument("--dr-actuator-kp-scale", type=float, default=0.40,
                   help="Â±fraction position-actuator kp jitter, e.g. kp=20 Â±40")
    p.add_argument("--dr-actuator-kv-scale", type=float, default=0.40,
                   help="Â±fraction velocity-actuator kv jitter")
    p.add_argument("--dr-gravity-scale", type=float, default=0.05,
                   help="Â±fraction gravity jitter")
    p.add_argument("--dr-solref-scale", type=float, default=0.0,
                   help="±fraction CONTACT solref (time const + damping ratio) "
                        "+ solimp jitter -- the warp<->Newton contact-stiffness "
                        "gap. Sweep dr-seed across chunks to span the band.")
    p.add_argument("--dr-push-prob", type=float, default=0.02,
                   help="per-step probability of external pelvis push")
    p.add_argument("--dr-push-vmax", type=float, default=1.5,
                   help="max push velocity impulse m/s (random horizontal dir)")
    p.add_argument("--dr-obs-noise", type=float, default=0.03,
                   help="gaussian obs noise std")
    p.add_argument("--dr-action-latency-max", type=int, default=3,
                   help="max action-latency ticks (per-env uniform random 0..N)")
    p.add_argument("--dr-act-gain", type=float, default=0.0,
                   help="PER-ENV residual-authority jitter: each env scales the "
                        "policy residual by U(1-a,1+a), resampled per episode, so "
                        "the policy must work when its action produces 0.6-1.4x the "
                        "effect (try 0.4). Mimics the warp<->Newton gain-drift "
                        "PER-ENV -- unlike --dr-actuator-kp-scale which is one "
                        "sample for the whole run. THE per-env DR lever.")
    p.add_argument("--dr-init-q-band", type=float, default=0.15,
                   help="Â±rad initial joint q jitter on reset")
    p.add_argument("--dr-init-xy-band", type=float, default=0.05,
                   help="Â±m initial base xy jitter on reset")
    p.add_argument("--dr-init-z-band", type=float, default=0.02,
                   help="Â±m initial base z jitter on reset")
    p.add_argument("--dr-init-tilt-band", type=float, default=0.0,
                   help="Â±rad initial base roll+pitch jitter (teaches recovery from "
                        "the deploy's folded/tilted handover; try 0.35)")
    p.add_argument("--dr-init-vel-band", type=float, default=0.0,
                   help="Â± initial base 6-DOF velocity jitter m/s & rad/s (try 0.4)")
    p.add_argument("--dr-init-vx-bias", type=float, default=0.0,
                   help="WALK: forward init vx in [0, this] so most episodes start "
                        "already moving forward (practise SUSTAINING the walk). try 0.4")
    p.add_argument("--dr-seed", type=int, default=0,
                   help="seed for the per-run model-param draws")
    p.add_argument("--no-dr", action="store_true",
                   help="disable all domain randomization")
    p.add_argument("--hold-arms", action="store_true",
                   help="full-body mode: arms present in MJCF, pinned at "
                        "nominal, not policy-controlled (use with the "
                        "full-body MJCF g1_full.mjcf.xml)")
    args = p.parse_args()

    if not Path(args.mjcf).exists():
        raise SystemExit(
            f"MJCF not found: {args.mjcf}. Build it first by running\n"
            f"  python projects/policies/research/training/build_g1_mjcf.py")

    reward_cfg = dict(alive=args.alive, upright=args.upright,
                      act=args.act, act_rate=args.act_rate, term=args.term,
                      vel=args.vel, vel_sigma=args.vel_sigma,
                      vel_sigma_stand=args.vel_sigma_stand, vel_l1=args.vel_l1,
                      vx_target=args.vx_target,
                      lat=args.lat, yaw=args.yaw, height=args.height, z_ref=args.z_ref,
                      gait_freq=args.gait_freq, gait_a_hip=args.gait_a_hip,
                      gait_a_knee=args.gait_a_knee, gait_a_lat=args.gait_a_lat,
                      cp_gain=args.cp_gain, max_ep=args.max_ep,
                      gait_a_ankle=args.gait_a_ankle,
                      seed_gait=args.seed_gait_pose,
                      rw_sched=args.rw_sched, rw_slip=args.rw_slip,
                      rw_track=args.rw_track, track_sigma=args.track_sigma,
                      rw_swing_track=args.rw_swing_track,
                      swing_track_ankle=args.swing_track_ankle,
                      rw_frontal_track=args.rw_frontal_track,
                      frontal_sigma=args.frontal_sigma,
                      frontal_res_scale=args.frontal_res_scale,
                      rw_shape=args.rw_shape, shape_sigma=args.shape_sigma,
                      rw_shape_lat=args.rw_shape_lat,
                      shape_lat_sigma=args.shape_lat_sigma,
                      phase_gate_tilt=args.phase_gate_tilt,
                      phase_gate_rate=args.phase_gate_rate,
                      phase_gate_floor=args.phase_gate_floor,
                      rw_com=args.rw_com, com_sigma=args.com_sigma,
                      rw_torso=args.rw_torso,
                      obs_stack=args.obs_stack,
                      obs_lookahead=[float(x) for x in args.obs_lookahead.split(",") if x.strip()],
                      vx_cmd_max=args.vx_cmd_max,
                      vx_phase_freeze=args.vx_phase_freeze,
                      asym_critic=args.asym_critic,
                      rw_track_vel=args.rw_track_vel,
                      track_vel_sigma=args.track_vel_sigma,
                      track_et=args.track_et,
                      track_et_grace=args.track_et_grace,
                      foot_z_contact=args.foot_z_contact,
                      foot_z_swing=args.foot_z_swing,
                      res_scale=args.res_scale,
                      gait_a_arm=args.gait_a_arm,
                      gait_a_push=args.gait_a_push,
                      rest_start_frac=args.rest_start_frac)
    # GAIT-V2: rebuild the nominal pose from the CLI knobs (hip/knee/ankle
    # apply to BOTH legs; the waist stays 0).
    nominal = NOMINAL.copy()
    for base in (0, 6):                      # left leg, right leg
        nominal[base + 0] = args.nominal_hip
        nominal[base + 3] = args.nominal_knee
        nominal[base + 4] = args.nominal_ankle
    reward_cfg["nominal"] = nominal.tolist()
    # HUMAN GAIT MODEL config (overrides the CPG terms + nominal in the env).
    if args.gait_model:
        reward_cfg["gait_model"] = args.gait_model
        reward_cfg["gait_params"] = dict(
            vx=args.vx_target, freq=args.gait_freq, duty=args.gait_duty,
            step_height=args.gait_step_height, pelvis_height=args.gait_pelvis_h,
            bob=args.gait_bob, sway=args.gait_a_lat, arm_swing=args.gait_a_arm,
            elbow_bend=args.gait_elbow, ankle_clear=args.gait_ankle_clear,
            x0=args.gait_x0, ramp_s=args.gait_ramp_s, style=args.gait_style,
            winter_hip_scale=args.gait_hip_scale,
            lateral=args.gait_lateral, yaw=args.gait_yaw,
            lat_hip_amp=args.gait_lat_hip_amp, step_width=args.gait_step_width)
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
            solref_scale=args.dr_solref_scale,
            push_prob=args.dr_push_prob,
            push_vmax=args.dr_push_vmax,
            obs_noise=args.dr_obs_noise,
            action_latency_max=args.dr_action_latency_max,
            act_gain=args.dr_act_gain,
            init_q_band=args.dr_init_q_band,
            init_xy_band=args.dr_init_xy_band,
            init_z_band=args.dr_init_z_band,
            init_tilt_band=args.dr_init_tilt_band,
            init_vel_band=args.dr_init_vel_band,
            init_vx_bias=args.dr_init_vx_bias,
            seed=args.dr_seed,
        )
        print(f"[DR] {dr_cfg}")
    env = BatchedG1StandEnv(args.envs, args.mjcf, reward_cfg=reward_cfg,
                            sim_dt=args.sim_dt, dr_cfg=dr_cfg,
                            hold_arms=args.hold_arms)
    if args.hold_arms:
        print(f"[full-body] holding {len(ARM_JOINTS)} arm joints at nominal "
              f"(policy still 13-DOF legs+waist)")
    N = args.envs

    _n_look = len([x for x in args.obs_lookahead.split(",") if x.strip()])
    OBS_IN = OBS_DIM * max(1, args.obs_stack) + NJ * _n_look  # stack + forecast
    OBS_IN += 1 if env.vx_cond else 0                         # + velocity command
    PRIV_IN = OBS_IN + env.priv_extra                          # critic-only width
    ASYM = env.asym

    HID = [int(x) for x in args.hidden_dims.split(",") if x.strip()] or [256, 128]

    def _mlp(d_in, d_out):
        layers, d = [], d_in
        for h in HID:
            layers += [nn.Linear(d, h), nn.Tanh()]
            d = h
        layers += [nn.Linear(d, d_out)]
        return nn.Sequential(*layers)

    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = _mlp(OBS_IN, NJ)
            # Asymmetric critic: wider input (actor obs + privileged signals).
            # PRIV_IN == OBS_IN when --asym-critic is off, so this is a no-op
            # in that case and old checkpoints still load.
            self.v = _mlp(PRIV_IN, 1)
            # log_std starts small so the residual is nearly zero on
            # day 1 â€” the analytic baseline carries the initial state.
            self.log_std = nn.Parameter(-1.0 * torch.ones(NJ))

        def forward(self, obs, priv=None):
            if priv is None:
                priv = obs
            return self.pi(obs), self.v(priv).squeeze(-1), self.log_std

    # All-GPU PPO. Actor + rollouts live on cuda; env returns torch
    # tensors directly (no numpy round-trip per step).
    tdev = env.tdev
    torch.manual_seed(0)
    ac = AC().to(tdev)
    if args.init_from and Path(args.init_from).exists():
        _sd = torch.load(args.init_from, map_location=tdev)
        # Warm-start across an obs-width change (e.g. +1 for the velocity
        # command): zero-PAD the first Linear of pi & v so the new input
        # columns start ignored, then learned. Other layers load as-is.
        cur = ac.state_dict()
        for k in ("pi.0.weight", "v.0.weight"):
            if k in _sd and k in cur and _sd[k].shape[1] != cur[k].shape[1]:
                w = cur[k].clone(); w.zero_()
                c = min(w.shape[1], _sd[k].shape[1])
                w[:, :c] = _sd[k][:, :c]
                _sd[k] = w
                print(f"  [warm] zero-padded {k}: {_sd[k].shape}")
        ac.load_state_dict(_sd, strict=False)
        print(f"warm-start from {args.init_from}")
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)

    if args.eval:
        ac.eval()
        ac.load_state_dict(torch.load(args.save, map_location=tdev))
        obs = env.reset()
        survived = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        n_falls = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        # WALK metrics: forward distance + mean forward speed accumulated only
        # while the env has NOT yet fallen (auto-reset teleports x, so integrate
        # vx*dt and stop at each env's first fall).
        dist = torch.zeros(env.n, device=tdev)
        vx_sum = torch.zeros(env.n, device=tdev)
        alive_steps = torch.zeros(env.n, device=tdev)
        for step in range(args.eval_steps):
            with torch.no_grad():
                mu = ac.pi(obs)           # value head unused; avoids priv-dim
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
    _prof = _balos.environ.get("G1_PROF")
    _acc = {"roll": 0.0, "ppo": 0.0}

    # GPU rollout buffers â€” no per-step CPUâ†”GPU traffic.
    obs_buf = torch.zeros(rollout, N, OBS_IN, device=tdev)
    priv_buf = torch.zeros(rollout, N, PRIV_IN, device=tdev)
    act_buf = torch.zeros(rollout, N, NJ, device=tdev)
    logp_buf = torch.zeros(rollout, N, device=tdev)
    rew_buf = torch.zeros(rollout, N, device=tdev)
    done_buf = torch.zeros(rollout, N, device=tdev)
    val_buf = torch.zeros(rollout, N, device=tdev)

    for it in range(1, args.iters + 1):
        if _prof:
            torch.cuda.synchronize(); _t_roll = time.time()
        for k in range(rollout):
            priv = env.priv_obs(obs) if ASYM else obs
            with torch.no_grad():
                mu, v, log_std = ac(obs, priv)
                std = log_std.exp()
                dist = torch.distributions.Normal(mu, std)
                a = dist.sample()
                logp = dist.log_prob(a).sum(-1)
            obs_buf[k] = obs
            priv_buf[k] = priv
            act_buf[k] = a
            logp_buf[k] = logp
            val_buf[k] = v
            obs, r, done, _ = env.step(a)
            rew_buf[k] = r
            done_buf[k] = done.float()
            total_steps += N

        if _prof:
            torch.cuda.synchronize()
            _acc["roll"] += time.time() - _t_roll
            _t_ppo = time.time()

        with torch.no_grad():
            _, last_v, _ = ac(obs, env.priv_obs(obs) if ASYM else obs)

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

        obs_flat = obs_buf.reshape(-1, OBS_IN)
        priv_flat = priv_buf.reshape(-1, PRIV_IN)
        act_flat = act_buf.reshape(-1, NJ)
        logp_flat = logp_buf.reshape(-1)
        adv_flat = adv.reshape(-1)
        ret_flat = ret.reshape(-1)

        clip_eps = 0.2
        for _epoch in range(4):
            mu, v, log_std = ac(obs_flat, priv_flat)
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

        if _prof:
            torch.cuda.synchronize()
            _acc["ppo"] += time.time() - _t_ppo

        if it % 5 == 0 or it == 1:
            ep_rps = rew_buf.mean().item()
            mean_v = val_buf.mean().item()
            fps = total_steps / max(time.time() - t0, 1e-6)
            print(f"it {it:4d}  ep_rew/step~{ep_rps:+.3f}  "
                  f"meanV {mean_v:+.2f}  steps {total_steps:,}  "
                  f"{fps:,.0f} env-steps/s")
            if _prof:
                _rt, _pt = _acc["roll"], _acc["ppo"]
                _tot = _rt + _pt
                print(f"       [prof] rollout {_rt/it*1000:6.1f} ms/it "
                      f"({100*_rt/_tot:4.1f}%)  ppo {_pt/it*1000:6.1f} ms/it "
                      f"({100*_pt/_tot:4.1f}%)  -> {_tot/it*1000:6.1f} ms/it")
                if _STEP_PROF["n"] > 0:
                    _sp = _STEP_PROF
                    _spt = _sp["ctrl"] + _sp["phys"] + _sp["rew"] + _sp["reset"] + _sp["obs"]
                    _pol = max(_rt - _spt, 0.0)
                    print(f"              step-segments ms/it: ctrl {_sp['ctrl']/it*1000:5.1f}"
                          f"  phys {_sp['phys']/it*1000:6.1f}  rew {_sp['rew']/it*1000:6.1f}"
                          f"  reset {_sp['reset']/it*1000:6.1f}  obs {_sp['obs']/it*1000:6.1f}"
                          f"  | policy+oh {_pol/it*1000:6.1f}")
                    _rp = _RST_PROF
                    print(f"              reset-internal ms/it: idx(nonzero) {_rp['idx']/it*1000:5.1f}"
                          f"  seed {_rp['seed']/it*1000:6.1f}  fwd(kinematics) {_rp['fwd']/it*1000:6.1f}"
                          f"  | resets/it {_rp['m']/it:7.0f}")

    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total_steps:,} steps in "
          f"{time.time() - t0:.1f}s)")

    # ONNX export: copy to CPU first (the deploy controller runs ORT on
    # CPU). The action head is CLAMPED to [-1,1] -- the EXACT squashing the
    # env applies in training (torch.clamp in step()). The old tanh wrap
    # was a silent train/deploy mismatch: policies output |mu|~2-3 with
    # constant mid-range corrections, and tanh weakens every mid-range
    # action by up to 24% (tanh(1.0)=0.76) -- enough to destabilise a
    # finely-balanced walk that survives in the trainer.
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
    dummy = torch.zeros(1, OBS_IN, dtype=torch.float32)
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
