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

"""Clean canonical humanoid-walk RL trainer (G1 + H1), GPU mujoco_warp.

WHY THIS FILE EXISTS
--------------------
The previous G1/H1 walk trainers were bespoke "Shadowing" designs (a kinematic
ghost + a bounded residual) with the single most important durability mechanism
*inverted*. A four-agent audit (2026-06-25) found every walk attempt hit the same
~2 m / ~2 s wall because:

  1. TIMEOUT-vs-FALL value bootstrapping was BROKEN. Both old trainers lumped a
     clean episode timeout and a real fall into one `done` flag and zeroed the GAE
     value bootstrap for BOTH. That literally teaches the policy that *surviving to
     the time limit is worthless* -- it prefers "do something good then fall" over
     "keep going". THIS is the central durability bug. (rsl_rl/legged_gym bootstrap
     the value on a timeout and only treat a fall as terminal.)
  2. The gait-shaping + balance rewards were OFF by default; the alive bonus was
     ~7x too large; per-step reward was NOT clipped to >=0 (so falling early to
     escape negative penalty was profitable).
  3. The actor saw base LINEAR velocity (a sim-only crutch the deploy can't measure)
     instead of putting it in the critic only.
  4. The policy was a residual on a kinematic ghost -- the architecture the field
     consensus AND our own notes say is wrong for *walking* (a reference describes a
     gait; it does not solve balance).

This trainer implements the proven `unitree_rl_gym` / `mujoco_playground` recipe
FAITHFULLY: pure RL (no injected reference), lower-body only (G1=12, H1=10 DOF),
the canonical observation/reward/termination, CORRECT timeout-vs-fall bootstrap,
and an HONEST in-loop durability metric (survival time + distance-before-first-fall,
NEVER reward curves -- the auto-reset eval-metric trap hides falls inside reward).

It REUSES the proven GPU plumbing from gpu_mjwarp_g1_walk_trainer.py verbatim
(MJCF -> mujoco_warp put_model/put_data, zero-copy wp.to_torch views, CUDA-graph
substep capture, kinematics-not-forward reset, PPO update, clamp-not-tanh ONNX
export). Only the recipe (obs/reward/termination/bootstrap) is fresh.

USAGE
-----
  # G1 short local validation (RTX 5070 Ti, ~minutes): does survival climb?
  python projects/policies/research/training/gpu_humanoid_walk_trainer.py --robot g1 \
      --envs 4096 --iters 400 --save projects/policies/research/training/runs/g1_walk_canon/policy.pt

  # evaluate the honest metric
  python projects/policies/research/training/gpu_humanoid_walk_trainer.py --robot g1 --eval \
      --save projects/policies/research/training/runs/g1_walk_canon/policy.pt

  # H1
  python projects/policies/research/training/gpu_humanoid_walk_trainer.py --robot h1 ...

The honest signal we are chasing: `never_fell_frac` -> 1.0 over a 20 s episode at a
commanded forward speed. That is the durable walk we have never had. Deploy (gain
matching + obs-pipeline parity in the OmniSim controller) is a SEPARATE milestone.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np
import torch

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))


# ────────────────────────────────────────────────────────────────────────────
# Per-robot configuration. Lower-body ONLY: `policy_joints` are policy-controlled;
# `held_joints` are PD-held at their default (torso/waist) so the upper body is a
# passive balance load, exactly like unitree_rl_gym.
# ────────────────────────────────────────────────────────────────────────────
@dataclass
class RobotCfg:
    name: str
    mjcf: str                         # MJCF the trainer loads (has {jn}_pos/{jn}_vel actuators)
    policy_joints: Sequence[str]      # ordered, policy-controlled (12 G1 / 10 H1)
    held_joints: Sequence[str]        # PD-held at default (waist/torso)
    default_pose: Sequence[float]     # canonical knees-bent seed, len == len(policy_joints)
    kp: Sequence[float]               # per policy-joint position gain
    kd: Sequence[float]               # per policy-joint velocity gain
    held_kp: float                    # uniform hold gain for held joints
    held_kd: float
    spawn_z: float                    # pelvis height at spawn
    base_height_target: float         # reward target
    fall_z: float                     # pelvis below this == fall
    foot_bodies: Sequence[str]        # (left, right) body names for foot-height rewards
    hip_lateral_idx: Sequence[int]    # indices into policy_joints of hip roll+yaw (hip_pos penalty)
    limits_lo: Sequence[float] = field(default=())   # per policy-joint, filled at load if empty
    limits_hi: Sequence[float] = field(default=())


def g1_cfg() -> RobotCfg:
    # LEGS order matches g1_robot_spec / the MJCF builder.
    legs = (
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    )
    # Default == a STATICALLY-STABLE nominal for THIS G1 URDF. The canonical
    # unitree knees-bent pose (hip -0.1/knee 0.3) is NOT stable for OmniSim's G1
    # (documented forward-CoM: pelvis CoM ~5 mm ahead of the foot front); the
    # proven-stable pose is the deeper squat hip -0.30/knee 0.52/ankle -0.23
    # (project_g1_newton_deploy_regression). The policy explores +-0.25 rad around
    # this stable basin instead of fighting a tipping nominal.
    default = (
        -0.30, 0.0, 0.0, 0.52, -0.23, 0.0,
        -0.30, 0.0, 0.0, 0.52, -0.23, 0.0,
    )
    # DEPLOY-PARITY GAINS: OmniSim's G1 Newton deploy applies a single UNIFORM
    # kp/kd (OMNISIM_NEWTON_TARGET_KE=100/KD=5) -- it cannot express canonical
    # per-joint gains today. To kill the #1 sim->deploy dead-end (gain mismatch),
    # train with EXACTLY the deployable uniform 100/5. (Per-joint hip100/knee150/
    # ankle40 would need new deploy plumbing; revisit only if walk quality demands.)
    # STABILIZING gains. The G1 squat has a forward CoM and is NOT passively stable
    # at the walk-spec kp100/kd5 (tips ~1.7s in plain MuJoCo) -> pure-RL can't
    # bootstrap balance. The deploy STAND uses kp400/kd60 (run_g1_stand_deploy.ps1)
    # and "holds 15s in plain mujoco". kp300/kd15 is the walk-friendly sweet spot:
    # STANDS 15s statically (so the policy starts survivable) yet low enough damping
    # to allow leg motion. Deploy MUST set OMNISIM_NEWTON_TARGET_KE=300/KD=15 to match.
    _kp = float(os.environ.get("G1_KP", "300"))
    _kd = float(os.environ.get("G1_KD", "15"))
    kp = tuple([_kp] * 12)
    kd = tuple([_kd] * 12)
    _mjcf = os.environ.get("G1_MJCF",
                           str(REPO / "projects/robots/unitree/g1/urdf/g1_legs_kp100.mjcf.xml"))
    return RobotCfg(
        name="g1",
        mjcf=_mjcf,
        policy_joints=legs,
        held_joints=("waist_yaw_joint",),
        default_pose=default,
        kp=kp, kd=kd, held_kp=_kp, held_kd=_kd,
        spawn_z=0.78, base_height_target=0.78, fall_z=0.50,
        foot_bodies=("left_ankle_roll_link", "right_ankle_roll_link"),
        # hip roll (idx 1,7) + hip yaw (idx 2,8)
        hip_lateral_idx=(1, 2, 7, 8),
    )


def h1_cfg() -> RobotCfg:
    # H1 legs are 5-DOF each (NO ankle_roll). Joint names per h1_legs_newton.mjcf.xml.
    legs = (
        "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
        "left_knee_joint", "left_ankle_joint",
        "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
        "right_knee_joint", "right_ankle_joint",
    )
    # canonical knees-bent seed (H1 order: yaw, roll, pitch, knee, ankle)
    default = (
        0.0, 0.0, -0.10, 0.30, -0.20,
        0.0, 0.0, -0.10, 0.30, -0.20,
    )
    # canonical H1 gains: hip 150/2, knee 200/4, ankle 40/2
    kp = (150., 150., 150., 200., 40.,  150., 150., 150., 200., 40.)
    kd = (2.,   2.,   2.,   4.,   2.,   2.,   2.,   2.,   4.,   2.)
    return RobotCfg(
        name="h1",
        mjcf=str(REPO / "projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml"),
        policy_joints=legs,
        held_joints=("torso_joint",),
        default_pose=default,
        kp=kp, kd=kd, held_kp=300.0, held_kd=6.0,
        spawn_z=1.03, base_height_target=1.03, fall_z=0.65,
        foot_bodies=("left_ankle_link", "right_ankle_link"),
        # hip yaw (idx 0,5) + hip roll (idx 1,6)
        hip_lateral_idx=(0, 1, 5, 6),
    )


CFGS = {"g1": g1_cfg, "h1": h1_cfg}


# ────────────────────────────────────────────────────────────────────────────
# Physics / control constants. Keep OmniSim's native 16 ms basicTimeStep so no
# coupled deploy-timestep change is needed (62.5 Hz control is fine; the recipe is
# robust to control rate within reason).
# ────────────────────────────────────────────────────────────────────────────
DT = 0.016           # env-step dt == OmniSim basicTimeStep
SUBSTEPS = 4         # 4 x 4 ms physics steps per env-step
PHYS_DT = DT / SUBSTEPS
GAIT_PERIOD = 0.8    # seconds per gait cycle (canonical)
ACT_SCALE = 0.25     # action = default + ACT_SCALE * a  (canonical)
SWING_HEIGHT = 0.08  # target swing-foot apex (canonical)


# Canonical reward weights (unitree_rl_gym / mujoco_playground synthesis).
DEFAULT_REWARD = dict(
    tracking_lin_vel=1.0,    # exp(-err^2 / sigma)
    tracking_ang_vel=0.5,
    alive=0.15,              # SMALL -- a large alive bonus farms "stand still"
    feet_phase=1.0,          # match swing-foot height to the gait phase profile
    feet_air_time=2.0,       # reward real swings (foot airborne ~0.4s before landing)
                             # -- the canonical term that FORCES striding vs shuffling
    lin_vel_z=-2.0,
    ang_vel_xy=-0.05,
    orientation=-1.0,        # projected-gravity horizontal (don't tip)
    base_height=-10.0,       # don't crouch-collapse
    action_rate=-0.01,
    dof_acc=-2.5e-7,
    dof_pos_limits=-5.0,
    hip_pos=-1.0,            # keep hip roll/yaw near 0 (anti-splay)
    feet_slip=-0.20,         # stance foot shouldn't slide
    termination=-0.0,        # optional extra fall penalty (default off; clip>=0 handles it)
    # STRONG penalty for not moving when commanded to -- breaks the "stand still
    # and never fall" local optimum that a marginal-stability biped falls into
    # (standing is the safe high-reward option unless not-moving is made costly).
    # -3.0 * speed-shortfall drives the standing-when-commanded reward to ~0 (after
    # the >=0 clip), so the policy MUST learn to move to earn reward.
    no_progress=-3.0,
    # NON-SATURATING forward-progress reward: pays linearly for forward speed up to
    # the command, so "faster" is ALWAYS worth more (the exp velocity-tracking
    # saturates and lets the policy hedge to slow). Off by default; enable via
    # --forward-rew for the fast-walk cycle.
    forward=0.0,
    tracking_sigma=0.25,
    soft_limit=0.9,
)


# ════════════════════════════════════════════════════════════════════════════
# Env
# ════════════════════════════════════════════════════════════════════════════
class HumanoidWalkEnv:
    """Batched mujoco_warp humanoid-walk env implementing the canonical recipe.

    Actor obs (deploy-honest, NO base linear velocity):
        ang_vel*0.25 (3), proj_gravity (3), command*[2,2,0.25] (3),
        (q - default) (NJ), qd*0.05 (NJ), last_action (NJ), sin/cos(phase) (2)
      = 11 + 3*NJ
    Critic (privileged) obs = actor obs + base_lin_vel*2 (3) + base_height (1)
        + foot_contact (2)  -> +6
    """

    def __init__(self, n, cfg: RobotCfg, device="cuda:0", reward_cfg=None,
                 dr=True, cmd_vx=(0.0, 1.0), cmd_vy=0.0, cmd_wz=0.0,
                 max_ep_s=20.0, asym=True, bal_kp=0.0, bal_kd=0.0, bal_clamp=0.3,
                 stand_frac=0.15, push_vmax=1.5, ghost_npz=None, ghost_w=0.0,
                 qd_fd=False):
        import warp as wp
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mjw, self.mujoco = wp, mjw, mujoco
        self.n = n
        self.cfg = cfg
        self.rc = dict(DEFAULT_REWARD)
        if reward_cfg:
            self.rc.update(reward_cfg)
        self.dr = dr
        self.asym = asym
        self.cmd_vx = cmd_vx
        self.stand_frac = float(stand_frac)
        self._push_vmax_cfg = float(push_vmax)
        self.cmd_vy = float(cmd_vy)
        self.cmd_wz = float(cmd_wz)
        self.max_ep = int(round(max_ep_s / DT))

        self.device = wp.get_device(device)
        self.tdev = torch.device("cuda:0" if "cuda" in str(self.device).lower() else "cpu")

        self.NJ = len(cfg.policy_joints)
        self.OBS_DIM = 11 + 3 * self.NJ
        self.PRIV_EXTRA = 6 if asym else 0
        self.vx_target = cmd_vx[1]

        # ── Load MJCF, force physics dt, impose canonical per-joint gains ──
        mjm = mujoco.MjModel.from_xml_path(cfg.mjcf)
        mjm.opt.timestep = PHYS_DT
        self._impose_gains(mjm)
        # OmniSim Newton DEPLOY (default, no USE_LINK_COM) places each link's COM at
        # its frame ORIGIN, not the true URDF COM. The trainer MJCF carries the TRUE
        # (forward) COMs, which makes the G1 tip forward and DIFFERS from the shipping
        # deploy model -> the core sim-to-deploy gap. Zeroing body_ipos matches the
        # deploy's COM-at-origin model (and makes the deep squat statically stable).
        if os.environ.get("HUMANOID_ZERO_COM", "0") == "1":
            mjm.body_ipos[:] = 0.0
        self.mjm = mjm

        mjd = mujoco.MjData(mjm)
        mujoco.mj_forward(mjm, mjd)
        with wp.ScopedDevice(self.device):
            self.mw_m = mjw.put_model(mjm)
            njmax = int(os.environ.get("HUMANOID_NJMAX", "256"))
            nconmax = int(os.environ.get("HUMANOID_NCONMAX", "256"))
            self.mw_d = mjw.put_data(mjm, mjd, nworld=n, njmax=njmax, nconmax=nconmax)

        # ── Index maps: controller order -> qpos/qvel/ctrl ──
        self._build_index_maps(mjm)
        self.nq, self.nv, self.nu = mjm.nq, mjm.nv, mjm.nu

        # ── Zero-copy torch views of the GPU physics buffers ──
        self.qpos_t = wp.to_torch(self.mw_d.qpos).view(n, self.nq)
        self.qvel_t = wp.to_torch(self.mw_d.qvel).view(n, self.nv)
        self.ctrl_t = wp.to_torch(self.mw_d.ctrl).view(n, self.nu)
        self.xpos_t = wp.to_torch(self.mw_d.xpos).view(n, mjm.nbody, 3)

        td = self.tdev
        self.default_t = torch.tensor(cfg.default_pose, dtype=torch.float32, device=td)
        self.kp_t = torch.tensor(cfg.kp, dtype=torch.float32, device=td)
        self.kd_t = torch.tensor(cfg.kd, dtype=torch.float32, device=td)
        # joint limits (per policy joint) from the compiled model
        lo = np.array([mjm.jnt_range[mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)][0]
                       for j in cfg.policy_joints], dtype=np.float32)
        hi = np.array([mjm.jnt_range[mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)][1]
                       for j in cfg.policy_joints], dtype=np.float32)
        self.lim_lo_t = torch.tensor(lo, device=td)
        self.lim_hi_t = torch.tensor(hi, device=td)
        mid = (lo + hi) * 0.5
        half = (hi - lo) * 0.5 * float(self.rc["soft_limit"])
        self.soft_lo_t = torch.tensor(mid - half, device=td)
        self.soft_hi_t = torch.tensor(mid + half, device=td)

        # foot body ids
        self.bid_lfoot = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_BODY, cfg.foot_bodies[0])
        self.bid_rfoot = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_BODY, cfg.foot_bodies[1])
        if self.bid_lfoot < 0 or self.bid_rfoot < 0:
            raise RuntimeError(f"foot bodies {cfg.foot_bodies} not in MJCF {cfg.mjcf}")

        self.hip_lat_idx_t = torch.tensor(cfg.hip_lateral_idx, dtype=torch.long, device=td)

        # ── Analytic balance assist (bootstraps balance on non-passively-stable
        #    models; the policy still has full authority on top). Indices derived
        #    from joint names so it works for G1 (ankle_pitch/roll) and H1 (ankle). ──
        pj = list(cfg.policy_joints)
        ap = [i for i, j in enumerate(pj) if "ankle_pitch" in j] or \
             [i for i, j in enumerate(pj) if "ankle" in j and "roll" not in j]
        ar = [i for i, j in enumerate(pj) if "ankle_roll" in j]
        hp = [i for i, j in enumerate(pj) if "hip_pitch" in j]
        self.idx_ank_pitch = torch.tensor(ap, dtype=torch.long, device=td)
        self.idx_ank_roll = torch.tensor(ar, dtype=torch.long, device=td)
        self.idx_hip_pitch = torch.tensor(hp, dtype=torch.long, device=td)
        self.bal_kp, self.bal_kd, self.bal_clamp = bal_kp, bal_kd, bal_clamp
        self.prev_roll_t = torch.zeros(n, device=td)
        self.prev_pitch_t = torch.zeros(n, device=td)

        # ── Seed pose (free joint + nominal legs) ──
        seed = mjd.qpos.copy().astype(np.float32)
        seed[0:3] = [0.0, 0.0, cfg.spawn_z]
        seed[3:7] = [1.0, 0.0, 0.0, 0.0]
        for i, j in enumerate(cfg.policy_joints):
            seed[self.c2qpos[i]] = cfg.default_pose[i]
        for k, j in enumerate(cfg.held_joints):
            seed[self.held_qpos[k]] = 0.0
        self.seed_qpos_t = torch.tensor(seed, dtype=torch.float32, device=td)

        # ── Per-env episodic state ──
        self.last_action_t = torch.zeros(n, self.NJ, device=td)
        self.prev_qd_t = torch.zeros(n, self.NJ, device=td)
        # DEPLOY-PARITY qd: the deploy has no velocity sensor and finite-diffs joint
        # SENSORS (lagged/noisy) where the trainer reads exact MuJoCo qvel. Training on
        # the SAME finite-diff qd closes the documented binding trainer->deploy obs gap.
        self.qd_fd = bool(qd_fd)
        self.prev_q_obs_t = self.default_t.unsqueeze(0).expand(n, self.NJ).contiguous()
        self.prev_foot_xy_t = torch.zeros(n, 2, 2, device=td)  # (n, foot, xy)
        self.feet_air_time_t = torch.zeros(n, 2, device=td)    # per-foot airborne time
        self.ep_step_t = torch.zeros(n, dtype=torch.int32, device=td)
        self.phase_t = torch.zeros(n, device=td)               # gait phase [0,1)
        self.cmd_t = torch.zeros(n, 3, device=td)              # vx, vy, wz

        # ── GHOST imitation (Shadowing): a GENERATED feasible leg-pose reference
        #    q_ghost(phase). The policy keeps FULL authority; the ghost enters only as
        #    a dense tracking REWARD (DeepMimic-style), NOT a residual feedforward
        #    (which the field consensus + our own notes say destabilises walking).
        #    Default off (ghost_w=0) -> identical to the canonical pure-RL trainer. ──
        self.ghost_w = float(ghost_w)
        self.ghost_t = None
        if ghost_npz and self.ghost_w > 0:
            g = np.load(str(ghost_npz))["q"].astype(np.float32)   # (W, NJ)
            assert g.shape[1] == self.NJ, f"ghost NJ {g.shape[1]} != {self.NJ}"
            self.ghost_t = torch.tensor(g, device=td)
            self.ghost_W = g.shape[0]
            print(f"[ghost] imitation ON: {ghost_npz} q{tuple(g.shape)} w={self.ghost_w}")

        # per-run DR sample (mujoco_warp can't cheaply do per-env model params)
        self._dr_resample()

        self._cuda_graph = None
        self._try_capture_graph()
        self._reset_all()

    # ── setup helpers ──
    def _impose_gains(self, mjm):
        """Override actuator gains to canonical per-joint values (legs) + hold (waist)."""
        mj = self.mujoco
        def set_pos(aid, kp):
            mjm.actuator_gainprm[aid][0] = kp
            mjm.actuator_biasprm[aid][1] = -kp
        def set_vel(aid, kd):
            mjm.actuator_gainprm[aid][0] = kd
            mjm.actuator_biasprm[aid][2] = -kd
        for i, j in enumerate(self.cfg.policy_joints):
            ap = mj.mj_name2id(mjm, mj.mjtObj.mjOBJ_ACTUATOR, f"{j}_pos")
            av = mj.mj_name2id(mjm, mj.mjtObj.mjOBJ_ACTUATOR, f"{j}_vel")
            if ap < 0 or av < 0:
                raise RuntimeError(f"actuators {j}_pos/{j}_vel not in MJCF")
            set_pos(ap, float(self.cfg.kp[i]))
            set_vel(av, float(self.cfg.kd[i]))
        for j in self.cfg.held_joints:
            ap = mj.mj_name2id(mjm, mj.mjtObj.mjOBJ_ACTUATOR, f"{j}_pos")
            av = mj.mj_name2id(mjm, mj.mjtObj.mjOBJ_ACTUATOR, f"{j}_vel")
            if ap >= 0:
                set_pos(ap, self.cfg.held_kp)
            if av >= 0:
                set_vel(av, self.cfg.held_kd)

    def _build_index_maps(self, mjm):
        mj = self.mujoco
        self.c2qpos = np.zeros(self.NJ, dtype=np.int64)
        self.c2qvel = np.zeros(self.NJ, dtype=np.int64)
        c2ctrl_pos = np.zeros(self.NJ, dtype=np.int64)
        for i, j in enumerate(self.cfg.policy_joints):
            jid = mj.mj_name2id(mjm, mj.mjtObj.mjOBJ_JOINT, j)
            if jid < 0:
                raise RuntimeError(f"joint {j} not in MJCF")
            self.c2qpos[i] = mjm.jnt_qposadr[jid]
            self.c2qvel[i] = mjm.jnt_dofadr[jid]
            c2ctrl_pos[i] = mj.mj_name2id(mjm, mj.mjtObj.mjOBJ_ACTUATOR, f"{j}_pos")
        # held joints
        self.held_qpos = np.zeros(len(self.cfg.held_joints), dtype=np.int64)
        held_ctrl_pos = np.zeros(len(self.cfg.held_joints), dtype=np.int64)
        for k, j in enumerate(self.cfg.held_joints):
            jid = mj.mj_name2id(mjm, mj.mjtObj.mjOBJ_JOINT, j)
            self.held_qpos[k] = mjm.jnt_qposadr[jid] if jid >= 0 else 0
            held_ctrl_pos[k] = mj.mj_name2id(mjm, mj.mjtObj.mjOBJ_ACTUATOR, f"{j}_pos")
        td = torch.device("cuda:0" if "cuda" in str(self.device).lower() else "cpu")
        self.qpos_idx_t = torch.tensor(self.c2qpos, dtype=torch.long, device=td)
        self.qvel_idx_t = torch.tensor(self.c2qvel, dtype=torch.long, device=td)
        self.ctrl_pos_idx_t = torch.tensor(c2ctrl_pos, dtype=torch.long, device=td)
        self.held_ctrl_pos_idx_t = torch.tensor(held_ctrl_pos, dtype=torch.long, device=td)

    def _try_capture_graph(self):
        if os.environ.get("OMNISIM_NEWTON_NO_GRAPH"):
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
        except Exception as e:  # pragma: no cover
            print(f"[env] CUDA graph capture failed ({e}); direct step")
            self._cuda_graph = None

    def _dr_resample(self):
        """One DR sample per call (per training run / periodic). Cheap & sufficient
        to start; per-env model DR is a later refinement."""
        if not self.dr:
            self.push_prob, self.push_vmax = 0.0, 0.0
            self.act_latency = 0
            self.obs_noise = 0.0
            return
        self.push_prob, self.push_vmax = 0.02, self._push_vmax_cfg  # gentle if cfg low
        self.act_latency = 0                            # 0 -> matches the deploy (no delay)
        self.obs_noise = 0.02                           # robustifies vs deploy qd noise
        # NOTE: model-parameter DR (friction/mass) must be set BEFORE put_model to
        # reach the GPU model; per-env model DR is a later refinement. The
        # effective DR here is pushes + action latency + obs noise + RSI jitter,
        # which already covers the dominant durability-relevant perturbations.

    # ── obs ──
    def _proj_gravity(self, qp):
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        gx = -2 * (x * z - w * y)
        gy = -2 * (y * z + w * x)
        gz = -(1 - 2 * (x * x + y * y))
        return torch.stack([gx, gy, gz], dim=1)

    def _foot_z(self):
        return self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), 2]   # (n, 2)

    def _build_obs(self):
        qp, qv = self.qpos_t, self.qvel_t
        ang = qv[:, 3:6] * 0.25
        pg = self._proj_gravity(qp)
        cmd = self.cmd_t * torch.tensor([2.0, 2.0, 0.25], device=self.tdev)
        q_raw = qp.index_select(1, self.qpos_idx_t)
        q = q_raw - self.default_t.unsqueeze(0)
        if self.qd_fd:
            qd = (q_raw - self.prev_q_obs_t) / DT * 0.05      # deploy-parity finite-diff
            self.prev_q_obs_t = q_raw.detach()
        else:
            qd = qv.index_select(1, self.qvel_idx_t) * 0.05
        ph = self.phase_t * (2.0 * math.pi)
        gait = torch.stack([torch.sin(ph), torch.cos(ph)], dim=1)
        obs = torch.cat([ang, pg, cmd, q, qd, self.last_action_t, gait], dim=1)
        if self.obs_noise > 0:
            obs = obs + torch.randn_like(obs) * self.obs_noise
        # cache privileged extras for the critic
        if self.asym:
            lin = qv[:, 0:3] * 2.0
            bz = qp[:, 2:3]
            fz = self._foot_z()
            contact = (fz < 0.05).float()
            self._priv_extra = torch.cat([lin, bz, contact], dim=1)
        return torch.nan_to_num(obs, 0.0, 0.0, 0.0).clamp_(-10.0, 10.0)

    def priv_obs(self, obs):
        if not self.asym:
            return obs
        return torch.cat([obs, self._priv_extra], dim=1)

    def reset(self):
        self._reset_all()
        return self._build_obs()

    def _reset_all(self):
        self._reset_envs(None)

    def _resample_cmd(self, idx, m):
        vx = torch.rand(m, device=self.tdev) * (self.cmd_vx[1] - self.cmd_vx[0]) + self.cmd_vx[0]
        vy = (torch.rand(m, device=self.tdev) * 2 - 1) * self.cmd_vy
        wz = (torch.rand(m, device=self.tdev) * 2 - 1) * self.cmd_wz
        # a fraction of envs commanded to STAND (cmd ~ 0) so the policy also holds still
        stand = torch.rand(m, device=self.tdev) < self.stand_frac
        vx = torch.where(stand, torch.zeros_like(vx), vx)
        self.cmd_t[idx, 0] = vx
        self.cmd_t[idx, 1] = vy
        self.cmd_t[idx, 2] = wz

    def _reset_envs(self, env_mask):
        if env_mask is None:
            idx = torch.arange(self.n, device=self.tdev)
        else:
            idx = torch.nonzero(env_mask, as_tuple=False).squeeze(-1)
            if idx.numel() == 0:
                return
        m = idx.shape[0]
        self.qpos_t[idx] = self.seed_qpos_t.unsqueeze(0).expand(m, -1)
        self.qvel_t[idx] = 0.0
        # small RSI jitter (helps robustness)
        jit = (torch.rand(m, self.NJ, device=self.tdev) * 2 - 1) * 0.05
        base_idx = idx.unsqueeze(1).expand(-1, self.NJ)
        col_idx = self.qpos_idx_t.unsqueeze(0).expand(m, -1)
        self.qpos_t[base_idx, col_idx] += jit
        # small base tilt/vel jitter
        self.qvel_t[idx, 0:2] += (torch.rand(m, 2, device=self.tdev) * 2 - 1) * 0.1
        self.ep_step_t[idx] = 0
        self.last_action_t[idx] = 0.0
        self.prev_qd_t[idx] = 0.0
        if self.qd_fd:                                   # clean finite-diff after reset
            self.prev_q_obs_t[idx] = self.qpos_t.index_select(1, self.qpos_idx_t)[idx].detach()
        self.feet_air_time_t[idx] = 0.0
        self.phase_t[idx] = torch.rand(m, device=self.tdev)
        self._resample_cmd(idx, m)
        # refresh xpos cheaply (kinematics, not full forward)
        with self.wp.ScopedDevice(self.device):
            self.mjw.kinematics(self.mw_m, self.mw_d)
        feet = self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), 0:2]
        self.prev_foot_xy_t[idx] = feet[idx]

    # ── step ──
    def step(self, action):
        a = torch.clamp(action, -1.0, 1.0)
        if self.act_latency > 0:
            # 1-step action delay: apply last step's action, store this one
            applied = self.last_action_t
        else:
            applied = a
        targets = self.default_t.unsqueeze(0) + ACT_SCALE * applied
        # analytic balance assist: counter body tilt at the ankles (+ a hip strategy
        # term) so a model that tips in ~1.5s gives the policy a survivable start.
        if self.bal_kp != 0.0 or self.bal_kd != 0.0:
            roll, pitch = self._roll_pitch(self.qpos_t)
            rr = (roll - self.prev_roll_t) / DT
            pr = (pitch - self.prev_pitch_t) / DT
            self.prev_roll_t, self.prev_pitch_t = roll, pitch
            d_p = torch.clamp(-self.bal_kp * pitch - self.bal_kd * pr,
                              -self.bal_clamp, self.bal_clamp)
            d_r = torch.clamp(-self.bal_kp * roll - self.bal_kd * rr,
                              -self.bal_clamp, self.bal_clamp)
            if self.idx_ank_pitch.numel():
                targets[:, self.idx_ank_pitch] += d_p.unsqueeze(1)
            if self.idx_ank_roll.numel():
                targets[:, self.idx_ank_roll] += d_r.unsqueeze(1)
            if self.idx_hip_pitch.numel():   # hip strategy: counter-rotate hips on big tilt
                targets[:, self.idx_hip_pitch] += (0.5 * d_p).unsqueeze(1)
        targets = torch.clamp(targets, self.lim_lo_t, self.lim_hi_t)

        self.ctrl_t.zero_()
        self.ctrl_t.index_copy_(1, self.ctrl_pos_idx_t, targets)
        # hold the upper body at default (0)
        if self.held_ctrl_pos_idx_t.numel() > 0:
            self.ctrl_t.index_fill_(1, self.held_ctrl_pos_idx_t, 0.0)

        # random pushes (DR)
        if self.push_prob > 0:
            hit = torch.rand(self.n, device=self.tdev) < self.push_prob
            if hit.any():
                kick = (torch.rand(self.n, 2, device=self.tdev) * 2 - 1) * self.push_vmax
                self.qvel_t[:, 0:2] += kick * hit.unsqueeze(1).float()

        with self.wp.ScopedDevice(self.device):
            if self._cuda_graph is not None:
                self.wp.capture_launch(self._cuda_graph)
            else:
                for _ in range(SUBSTEPS):
                    self.mjw.step(self.mw_m, self.mw_d)

        self.ep_step_t += 1
        # advance gait phase only when commanded to move (freeze for standing)
        moving = (self.cmd_t[:, 0].abs() + self.cmd_t[:, 1].abs()) > 0.05
        self.phase_t = torch.where(
            moving, (self.phase_t + DT / GAIT_PERIOD) % 1.0, self.phase_t)

        reward, fall = self._reward(a, moving)
        timeout = self.ep_step_t >= self.max_ep
        done = fall | timeout

        self.last_action_t = a
        obs = self._build_obs()
        # auto-reset done envs (obs already reflects pre-reset terminal state for reward;
        # rebuild after reset so the returned obs is the fresh episode's first obs)
        if done.any():
            self._reset_envs(done)
            obs = self._build_obs()
        return obs, reward, fall, timeout, {}

    def _reward(self, action, moving):
        rc = self.rc
        qp, qv = self.qpos_t, self.qvel_t
        sig = float(rc["tracking_sigma"])

        vx, vy = qv[:, 0], qv[:, 1]
        wz = qv[:, 5]
        cvx, cvy, cwz = self.cmd_t[:, 0], self.cmd_t[:, 1], self.cmd_t[:, 2]
        lin_err = (vx - cvx) ** 2 + (vy - cvy) ** 2
        r_track_lin = rc["tracking_lin_vel"] * torch.exp(-lin_err / sig)
        r_track_ang = rc["tracking_ang_vel"] * torch.exp(-(wz - cwz) ** 2 / sig)
        # non-saturating forward progress (linear up to the command)
        r_forward = rc["forward"] * torch.clamp(torch.minimum(vx, cvx), min=0.0)

        pg = self._proj_gravity(qp)
        r_orient = rc["orientation"] * (pg[:, 0] ** 2 + pg[:, 1] ** 2)
        bz = qp[:, 2]
        r_height = rc["base_height"] * (bz - self.cfg.base_height_target) ** 2
        r_vz = rc["lin_vel_z"] * qv[:, 2] ** 2
        r_angxy = rc["ang_vel_xy"] * (qv[:, 3] ** 2 + qv[:, 4] ** 2)

        qd = qv.index_select(1, self.qvel_idx_t)
        r_acc = rc["dof_acc"] * (((qd - self.prev_qd_t) / DT) ** 2).sum(1)
        self.prev_qd_t = qd
        r_arate = rc["action_rate"] * ((action - self.last_action_t) ** 2).sum(1)

        q = qp.index_select(1, self.qpos_idx_t)
        over_hi = torch.clamp(q - self.soft_hi_t, min=0.0)
        over_lo = torch.clamp(self.soft_lo_t - q, min=0.0)
        r_lim = rc["dof_pos_limits"] * (over_hi + over_lo).sum(1)

        hip_lat = q.index_select(1, self.hip_lat_idx_t)
        r_hip = rc["hip_pos"] * (hip_lat ** 2).sum(1)

        # feet phase: swing foot tracks a sin apex; stance foot stays low.
        fz = self._foot_z()                      # (n,2)
        ph = self.phase_t
        # left leg uses phase, right leg offset half-cycle
        lp = ph
        rp = (ph + 0.5) % 1.0
        def swing_target(p):
            # stance [0,0.5): 0 ; swing [0.5,1): sin apex SWING_HEIGHT
            sw = (p >= 0.5).float()
            return sw * SWING_HEIGHT * torch.sin(math.pi * torch.clamp((p - 0.5) / 0.5, 0, 1))
        zt = torch.stack([swing_target(lp), swing_target(rp)], dim=1)
        # only shape the gait when moving; standing => both feet target 0
        zt = zt * moving.unsqueeze(1).float()
        feet_err = ((fz - zt) ** 2).sum(1)
        r_feet = rc["feet_phase"] * torch.exp(-feet_err / 0.0025)

        # feet slip: stance foot (low) horizontal speed penalty
        feet_xy = self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), 0:2]
        foot_v = ((feet_xy - self.prev_foot_xy_t) / DT)            # (n,2,2)
        self.prev_foot_xy_t = feet_xy
        stance = (fz < 0.05).float()
        r_slip = rc["feet_slip"] * ((foot_v ** 2).sum(2) * stance).sum(1)

        # feet AIR TIME: reward each foot for a real swing (airborne >~0.35s) the
        # moment it lands. Cannot be earned by standing or shuffling -> this is what
        # converts a creep into a genuine stride. (legged_gym/unitree canonical.)
        contact = fz < 0.06
        first_contact = contact & (self.feet_air_time_t > 0.0)
        self.feet_air_time_t = self.feet_air_time_t + DT
        r_airtime = (rc["feet_air_time"]
                     * ((self.feet_air_time_t - 0.35) * first_contact.float()).sum(1)
                     * moving.float())
        self.feet_air_time_t = self.feet_air_time_t * (~contact).float()

        r_alive = torch.full((self.n,), rc["alive"], device=self.tdev)

        # no-progress penalty: when commanded to move, penalize the forward-speed
        # shortfall so standing still is NOT a viable high-reward strategy.
        cmd_speed = cvx.abs() + cvy.abs()
        move_cmd = (cmd_speed > 0.1).float()
        shortfall = torch.clamp(cvx - vx, min=0.0)        # only penalize too-slow
        r_noprog = rc["no_progress"] * move_cmd * shortfall

        # GHOST imitation: dense reward for matching the generated leg-pose reference
        # at the current gait phase (only when moving -- the ghost encodes a stride).
        r_ghost = torch.zeros(self.n, device=self.tdev)
        if self.ghost_t is not None:
            gb = (self.phase_t * self.ghost_W).long().clamp(0, self.ghost_W - 1)
            g_ref = self.ghost_t[gb]                          # (n,NJ)
            gerr = ((q - g_ref) ** 2).mean(1)
            # GATE on ACTUAL forward translation (fraction of commanded speed): matching
            # the ghost POSE only pays if the robot is genuinely moving forward. Without
            # this the policy games the ghost by cycling its legs IN PLACE (pose matches,
            # zero translation) -> a slow shuffle. fwd_frac kills that exploit.
            fwd_frac = torch.clamp(vx / (cvx.abs() + 1e-3), 0.0, 1.0)
            r_ghost = self.ghost_w * torch.exp(-gerr / 0.05) * moving.float() * fwd_frac

        reward = (r_track_lin + r_track_ang + r_forward + r_alive + r_feet + r_airtime
                  + r_orient + r_height + r_vz + r_angxy
                  + r_acc + r_arate + r_lim + r_hip + r_slip + r_noprog + r_ghost)

        # CLIP per-step reward to >=0 so falling early to escape penalty is never
        # profitable (the canonical only_positive_rewards=True).
        reward = reward.clamp(min=0.0)

        roll, pitch = self._roll_pitch(qp)
        fall = (bz < self.cfg.fall_z) | (roll.abs() > 0.8) | (pitch.abs() > 0.8)
        if rc["termination"] < 0:
            reward = reward + rc["termination"] * fall.float()
        return reward, fall

    def _roll_pitch(self, qp):
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1.0, 1.0))
        return roll, pitch


# ════════════════════════════════════════════════════════════════════════════
# Actor-Critic + PPO
# ════════════════════════════════════════════════════════════════════════════
def _mlp(d_in, d_out, hid):
    import torch.nn as nn
    layers, d = [], d_in
    for h in hid:
        layers += [nn.Linear(d, h), nn.ELU()]
        d = h
    layers += [nn.Linear(d, d_out)]
    return nn.Sequential(*layers)


def build_ac(obs_in, priv_in, nj, hid, init_log_std=-1.0):
    import torch.nn as nn

    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = _mlp(obs_in, nj, hid)
            self.v = _mlp(priv_in, 1, hid)
            self.log_std = nn.Parameter(init_log_std * torch.ones(nj))

        def forward(self, obs, priv=None):
            if priv is None:
                priv = obs
            return self.pi(obs), self.v(priv).squeeze(-1), self.log_std

    return AC()


@torch.no_grad()
def evaluate(env, ac, steps, deterministic=True):
    """HONEST durability metric: survival steps, first-fall, distance-before-fall.
    Integrates vx*dt only while alive (auto-reset teleports x), freezing at first
    fall, so the auto-reset eval-metric trap cannot inflate it."""
    tdev = env.tdev
    was_training = ac.training
    ac.eval()
    obs = env.reset()
    n = env.n
    survived = torch.zeros(n, device=tdev)
    first_fall = torch.zeros(n, dtype=torch.int32, device=tdev)
    n_falls = torch.zeros(n, device=tdev)
    dist = torch.zeros(n, device=tdev)
    vx_sum = torch.zeros(n, device=tdev)
    alive_steps = torch.zeros(n, device=tdev)
    for step in range(steps):
        mu = ac.pi(obs)          # deterministic mean action; skip the value head
        alive = (first_fall == 0)
        obs, _, fall, timeout, _ = env.step(mu)
        done = fall | timeout
        step_alive = (alive & (~done)).float()
        vx = env.qvel_t[:, 0]
        dist += vx * DT * step_alive
        vx_sum += vx * step_alive
        alive_steps += step_alive
        survived += (~done).float()
        n_falls += fall.float()
        newly = fall & (first_fall == 0)
        first_fall = torch.where(newly, torch.full_like(first_fall, step + 1), first_fall)
    if was_training:
        ac.train()
    ff = first_fall.cpu().numpy()
    never = (ff == 0)
    ff_fell = ff[~never]
    d = dist.cpu().numpy()
    mean_vx = (vx_sum / torch.clamp(alive_steps, min=1.0)).cpu().numpy()
    # HONEST survival = steps before FIRST fall (alive_steps), NOT the raw
    # not-done counter which keeps counting after the auto-reset (the trap).
    asv = alive_steps.cpu().numpy()
    return dict(
        survival_mean=float(asv.mean()),
        survival_med=float(np.median(asv)),
        never_fell_frac=float(never.mean()),
        first_fall_mean=float(ff_fell.mean()) if ff_fell.size else float(steps),
        dist_mean=float(d.mean()), dist_max=float(d.max()),
        speed_mean=float(np.mean(mean_vx)), target=env.vx_target, steps=steps,
    )


def train(args):
    cfg = CFGS[args.robot]()
    if not Path(cfg.mjcf).exists():
        raise SystemExit(f"MJCF not found: {cfg.mjcf}")
    env = HumanoidWalkEnv(
        args.envs, cfg,
        reward_cfg={"no_progress": args.no_progress, "forward": args.forward_rew},
        dr=not args.no_dr, push_vmax=args.push_vmax,
        cmd_vx=(args.cmd_vx_min, args.cmd_vx_max), cmd_vy=args.cmd_vy,
        cmd_wz=args.cmd_wz, max_ep_s=args.max_ep_s, asym=not args.no_asym,
        bal_kp=args.bal_kp, bal_kd=args.bal_kd, bal_clamp=args.bal_clamp,
        stand_frac=args.stand_frac, ghost_npz=args.ghost, ghost_w=args.ghost_w,
        qd_fd=args.qd_fd)
    tdev = env.tdev
    NJ = env.NJ
    OBS_IN = env.OBS_DIM
    PRIV_IN = OBS_IN + env.PRIV_EXTRA
    hid = [int(x) for x in args.hidden.split(",") if x.strip()] or [256, 128]

    torch.manual_seed(args.seed)
    ac = build_ac(OBS_IN, PRIV_IN, NJ, hid, init_log_std=args.init_log_std).to(tdev)
    if args.init_from and Path(args.init_from).exists():
        ac.load_state_dict(torch.load(args.init_from, map_location=tdev), strict=False)
        print(f"warm-start from {args.init_from}")

    print(f"[train] robot={cfg.name} NJ={NJ} obs={OBS_IN} priv={PRIV_IN} "
          f"envs={args.envs} max_ep={env.max_ep} ({args.max_ep_s}s)")

    if args.eval:
        ac.load_state_dict(torch.load(args.save, map_location=tdev))
        m = evaluate(env, ac, args.eval_steps)
        _print_eval("EVAL", 0, m)
        return

    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)
    rollout, N = args.rollout, args.envs
    Path(args.save).parent.mkdir(parents=True, exist_ok=True)

    obs_buf = torch.zeros(rollout, N, OBS_IN, device=tdev)
    priv_buf = torch.zeros(rollout, N, PRIV_IN, device=tdev)
    act_buf = torch.zeros(rollout, N, NJ, device=tdev)
    logp_buf = torch.zeros(rollout, N, device=tdev)
    rew_buf = torch.zeros(rollout, N, device=tdev)
    done_buf = torch.zeros(rollout, N, device=tdev)
    val_buf = torch.zeros(rollout, N, device=tdev)

    gamma, lam = 0.99, 0.95
    obs = env.reset()
    total_steps, t0 = 0, time.time()
    best_never = -1.0

    for it in range(1, args.iters + 1):
        # velocity curriculum: ramp commanded vx_max 0 -> target so the policy
        # masters standing/balance before it is asked to walk fast.
        if args.cmd_curriculum_iters > 0:
            frac = min(1.0, it / args.cmd_curriculum_iters)
            env.cmd_vx = (args.cmd_vx_min, frac * args.cmd_vx_max)
            env.vx_target = env.cmd_vx[1]
        for k in range(rollout):
            priv = env.priv_obs(obs)
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
            obs, r, fall, timeout, _ = env.step(a)
            # ── THE DURABILITY FIX: bootstrap the value on TIMEOUT (a clean
            #    truncation), terminate (no bootstrap) only on a real FALL.
            #    rsl_rl/legged_gym style: inject gamma*V(s_k) into the timeout
            #    step's reward, then treat done=fall|timeout for the GAE chain. ──
            r = r + gamma * v * timeout.float()
            rew_buf[k] = r
            done_buf[k] = (fall | timeout).float()
            total_steps += N

        with torch.no_grad():
            _, last_v, _ = ac(obs, env.priv_obs(obs))
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

        obs_f = obs_buf.reshape(-1, OBS_IN)
        priv_f = priv_buf.reshape(-1, PRIV_IN)
        act_f = act_buf.reshape(-1, NJ)
        logp_f = logp_buf.reshape(-1)
        adv_f = adv.reshape(-1)
        ret_f = ret.reshape(-1)
        B = obs_f.shape[0]
        mb = max(1, B // args.minibatches)

        # KL-adaptive LR (rsl_rl): hold approx-KL near desired_kl
        for _epoch in range(args.epochs):
            perm = torch.randperm(B, device=tdev)
            for s in range(0, B, mb):
                bi = perm[s:s + mb]
                mu, v, log_std = ac(obs_f[bi], priv_f[bi])
                std = log_std.exp()
                d = torch.distributions.Normal(mu, std)
                new_logp = d.log_prob(act_f[bi]).sum(-1)
                ratio = (new_logp - logp_f[bi]).exp()
                s1 = ratio * adv_f[bi]
                s2 = torch.clamp(ratio, 0.8, 1.2) * adv_f[bi]
                pi_loss = -torch.min(s1, s2).mean()
                v_loss = ((v - ret_f[bi]) ** 2).mean()
                ent = d.entropy().sum(-1).mean()
                loss = pi_loss + 0.5 * v_loss - args.ent_coef * ent
                opt.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(ac.parameters(), 1.0)
                opt.step()
            # adapt LR on the mean KL after each epoch
            with torch.no_grad():
                mu, _, log_std = ac(obs_f, priv_f)
                std = log_std.exp()
                d = torch.distributions.Normal(mu, std)
                kl = (logp_f - d.log_prob(act_f).sum(-1)).mean().item()
            if kl > 2.0 * args.desired_kl:
                for g in opt.param_groups:
                    g["lr"] = max(1e-5, g["lr"] / 1.5)
            elif 0 < kl < 0.5 * args.desired_kl:
                for g in opt.param_groups:
                    g["lr"] = min(1e-2, g["lr"] * 1.5)

        if it % args.log_every == 0 or it == 1:
            fps = total_steps / max(time.time() - t0, 1e-6)
            cur_lr = opt.param_groups[0]["lr"]
            print(f"it {it:4d}  rew/step~{rew_buf.mean():+.3f}  meanV {val_buf.mean():+.2f}  "
                  f"lr {cur_lr:.1e}  {fps:,.0f} steps/s  total {total_steps:,}")

        if it % args.eval_every == 0:
            m = evaluate(env, ac, args.eval_steps)
            _print_eval(f"it {it}", it, m)
            obs = env.reset()      # eval perturbs env; restart cleanly
            if m["never_fell_frac"] > best_never:
                best_never = m["never_fell_frac"]
                _save(ac, args.save, env, best=True)

    _save(ac, args.save, env, best=False)
    m = evaluate(env, ac, args.eval_steps)
    _print_eval("FINAL", args.iters, m)


def _print_eval(tag, it, m):
    print(f"[honest-eval {tag}] survival_mean={m['survival_mean']:.0f}/{m['steps']} "
          f"never_fell={m['never_fell_frac']:.2f}  first_fall_mean={m['first_fall_mean']:.0f}  "
          f"dist_mean={m['dist_mean']:.2f}m max={m['dist_max']:.2f}m  "
          f"speed={m['speed_mean']:.2f}/{m['target']:.2f} m/s")


def _save(ac, path, env, best=False):
    torch.save(ac.state_dict(), path)
    # ONNX export with CLAMP-not-tanh squashing (matches env.step clamp exactly).
    import torch.nn as nn
    onnx_path = Path(path).with_suffix(".onnx")

    class Deploy(nn.Module):
        def __init__(self, pi):
            super().__init__()
            self.pi = pi

        def forward(self, obs):
            return torch.clamp(self.pi(obs), -1.0, 1.0)

    try:
        cpu = build_ac(env.OBS_DIM, env.OBS_DIM + env.PRIV_EXTRA, env.NJ,
                       [l.out_features for l in ac.pi if hasattr(l, "out_features")][:-1])
        cpu.load_state_dict({k: v.cpu() for k, v in ac.state_dict().items()})
        wrapped = Deploy(cpu.pi).eval()
        dummy = torch.zeros(1, env.OBS_DIM)
        import io, contextlib
        with contextlib.redirect_stdout(io.StringIO()):
            torch.onnx.export(wrapped, dummy, str(onnx_path),
                              input_names=["obs"], output_names=["action"],
                              dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
                              opset_version=17)
    except Exception as e:
        print(f"ONNX export failed (non-fatal): {e}")
    tag = " (best)" if best else ""
    print(f"saved {path}{tag}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot", choices=list(CFGS), default="g1")
    p.add_argument("--envs", type=int, default=4096)
    p.add_argument("--iters", type=int, default=400)
    p.add_argument("--rollout", type=int, default=24)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--desired-kl", type=float, default=0.01)
    p.add_argument("--ent-coef", type=float, default=0.01)
    p.add_argument("--init-log-std", type=float, default=-1.0,
                   help="initial action-noise log std (exp(-1.0)=0.37). Lower = gentler "
                        "exploration; high noise topples a model that tips in ~1.7s.")
    p.add_argument("--epochs", type=int, default=5)
    p.add_argument("--minibatches", type=int, default=4)
    p.add_argument("--hidden", type=str, default="256,128")
    p.add_argument("--max-ep-s", type=float, default=20.0)
    p.add_argument("--cmd-vx-min", type=float, default=0.0)
    p.add_argument("--cmd-vx-max", type=float, default=1.0)
    p.add_argument("--cmd-curriculum-iters", type=int, default=200,
                   help="ramp commanded vx_max from 0 to --cmd-vx-max over this many "
                        "iters so the policy learns to stand/balance before sprinting; "
                        "0 disables the curriculum")
    p.add_argument("--cmd-vy", type=float, default=0.0)
    p.add_argument("--cmd-wz", type=float, default=0.0)
    p.add_argument("--forward-rew", type=float, default=0.0,
                   help="non-saturating forward-progress reward weight (try 1-3). Pays "
                        "linearly for forward speed up to the command -> fights the "
                        "hedge-to-slow optimum on a marginal model.")
    p.add_argument("--push-vmax", type=float, default=1.5,
                   help="max random-push velocity (m/s) when DR is on. Use ~0.3 for "
                        "GENTLE DR that robustifies a walk without toppling a marginal "
                        "model (full 1.5 breaks bare-model learning).")
    p.add_argument("--no-progress", type=float, default=-3.0,
                   help="penalty weight for forward-speed shortfall when commanded to "
                        "move (breaks the stand-still optimum). Set 0 for a pure "
                        "balance-learning phase.")
    p.add_argument("--stand-frac", type=float, default=0.15,
                   help="fraction of envs commanded to stand (cmd~0). Lower it (e.g. "
                        "0.05) to force walking once standing is learned.")
    p.add_argument("--no-dr", action="store_true")
    p.add_argument("--no-asym", action="store_true")
    p.add_argument("--ghost", type=str, default=None,
                   help="GENERATED ghost npz (q(W,NJ) leg-pose reference). When set with "
                        "--ghost-w>0 the policy is rewarded for tracking it (Shadowing): a "
                        "from-scratch walk BUILT from the ghost, full-authority + imitation "
                        "reward. e.g. _scratch/g1_squat_ghost.npz")
    p.add_argument("--ghost-w", type=float, default=0.0,
                   help="ghost imitation reward weight (try 1.0-2.0). 0 == pure-RL canonical.")
    p.add_argument("--qd-fd", action="store_true",
                   help="DEPLOY-PARITY: train on finite-diff joint velocity (matching the "
                        "deploy's sensor finite-diff) instead of exact MuJoCo qvel. Closes "
                        "the documented trainer->deploy obs gap. Use WITH DR (no --no-dr).")
    p.add_argument("--bal-kp", type=float, default=0.0,
                   help="analytic balance assist proportional gain (try 4-8). Counters "
                        "body tilt at ankles+hips so a non-passively-stable model gives "
                        "the policy a survivable start. Policy keeps full authority.")
    p.add_argument("--bal-kd", type=float, default=0.0)
    p.add_argument("--bal-clamp", type=float, default=0.3)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--log-every", type=int, default=10)
    p.add_argument("--eval-every", type=int, default=50)
    p.add_argument("--eval-steps", type=int, default=1250)
    p.add_argument("--eval", action="store_true")
    p.add_argument("--init-from", type=str, default="")
    p.add_argument("--save", type=str,
                   default=str(REPO / "projects/policies/research/training/runs/humanoid_walk/policy.pt"))
    args = p.parse_args()
    train(args)


if __name__ == "__main__":
    main()
