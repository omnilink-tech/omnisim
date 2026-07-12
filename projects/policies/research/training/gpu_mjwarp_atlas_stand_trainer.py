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

"""GPU mujoco_warp trainer for the Atlas standing policy.

Direct copy of `gpu_mjwarp_g1_stand_trainer.py` with G1-specific
constants swapped for Atlas's 30-DOF body. Demonstrates the recipe
ports cleanly to a different robot with only the joint-set /
nominal-pose / ankle-index changes — see the general recipe at:

    docs/developer/sim-to-deploy-rl-recipe.md  (general recipe)
    docs/developer/g1-stand-rl-playbook.md     (G1 case study)

What's Atlas-specific (vs the G1 template): ATLAS_JOINTS, NJ,
NOMINAL, JOINT_LIMITS_*, _L_AP / _R_AP / _L_AR / _R_AR (still ankles,
just at new indices into the longer joint list), and SPAWN_Z.
Everything else — heavy DR knobs, the 5 GPU speedups, the PPO loop —
is unchanged from the G1 trainer.

Usage:
    python projects/policies/research/training/gpu_mjwarp_atlas_stand_trainer.py \\
        --envs 4096 --iters 600 --rollout 12 \\
        --mjcf projects/robots/boston_dynamics/atlas/urdf/atlas.mjcf.xml \\
        --save projects/policies/research/training/runs/gpu_atlas_stand_robust/policy.pt

Deploy env vars (set on the OmniSim launch):
    OMNISIM_URDF_USE_INERTIA=1       # mandatory for any URDF robot
    OMNISIM_NEWTON_FORCE_MUJOCO=1    # pick SolverMuJoCo (not XPBD)
    OMNISIM_NEWTON_MJWARP=1          # GPU mujoco_warp under SolverMuJoCo
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
# Atlas stand layout constants — joint set + nominal mirror
# projects/policies/research/backends/atlas_robot_spec.py.
# ────────────────────────────────────────────────────────────────────
NJ = 30            # 3 back + 1 neck + 14 arms + 12 legs
QPOS_J0 = 7        # joints start in qpos after the 7-dim free joint
QVEL_J0 = 6        # joints start in qvel after the 6-dim free dofs
OBS_DIM = 99       # lin(3)+ang(3)+proj_g(3)+q(30)+qd(30)+last_action(30)
RES_SCALE = 0.05   # ±0.05 rad residual. With the strong coupled
                   # baseline harvesting ~88 % of the per-step reward
                   # ceiling, PPO's mean mu stays near zero regardless
                   # of RES_SCALE (the act² penalty + heavy DR noise
                   # both push it back). Tested 0.05 vs 0.10: same V
                   # trajectory through iter 110. Keep small to bound
                   # the worst case if PPO does drift the mean.
DT = 0.016         # env-step dt = OmniSim basicTimeStep 16 ms
SUBSTEPS = 4       # 4 × 4 ms physics = 16 ms env-step
PHYS_DT = 0.004

# Joint name order matches the Newton-dumped MJCF actuator order
# (back → l_arm → neck → r_arm → l_leg → r_leg). The trainer
# resolves indices by name so this order is internal-only;
# the deploy controller must enumerate joints in the same order
# so the policy's action[i] reaches the right actuator.
ATLAS_JOINTS = (
    # Back (3)
    "back_bkz", "back_bky", "back_bkx",
    # Left arm (7)
    "l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx",
    "l_arm_uwy", "l_arm_mwx", "l_arm_lwy",
    # Neck (1)
    "neck_ay",
    # Right arm (7)
    "r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx",
    "r_arm_uwy", "r_arm_mwx", "r_arm_lwy",
    # Left leg (6) hip → ankle
    "l_leg_hpz", "l_leg_hpx", "l_leg_hpy",
    "l_leg_kny", "l_leg_aky", "l_leg_akx",
    # Right leg (6)
    "r_leg_hpz", "r_leg_hpx", "r_leg_hpy",
    "r_leg_kny", "r_leg_aky", "r_leg_akx",
)
assert len(ATLAS_JOINTS) == NJ
# Alias for the index-resolution code shared with the G1 template.
LEGS_JOINTS = ATLAS_JOINTS

# Nominal stand pose — upright with mild knee bend and arms hanging.
# The original deep-squat NOMINAL (taken from G1's recipe) put Atlas's
# CoM well forward of the foot center and the deep arm tuck caused
# convex-hull self-collisions. Mild bend keeps CoM near ankle column
# and shifts joint torque budget away from saturated knees.
NOMINAL = np.array([
    # Back upright.
    +0.00, +0.00, +0.00,
    # Left arm: hang at side (arms have NULL physics anyway after the
    # import script's collision strip — pose just needs to be inside
    # joint limits and not visually broken).
    +0.00, -0.30, +0.30, +0.10, +0.00, +0.00, +0.00,
    # Neck level.
    +0.00,
    # Right arm (mirror).
    +0.00, +0.30, +0.30, -0.10, +0.00, +0.00, +0.00,
    # Left leg: hip yaw 0, tiny outward roll, mild forward squat.
    +0.00, +0.03, -0.10, +0.20, -0.10, -0.03,
    # Right leg (mirror).
    +0.00, -0.03, -0.10, +0.20, -0.10, +0.03,
], dtype=np.float32)
assert NOMINAL.shape == (NJ,)

# Analytic balance PD for Atlas — coupled ankle + hip + back strategy.
#
# MJCF actuators all have kp=20 N·m/rad (tiny for a 175 kg body).
# To get the ~85 N·m of ankle torque needed to balance Atlas's
# CoM-gravity moment, the baseline must command ~4 rad of joint
# position error — which the actuator then converts to torque and
# the joint's effort limit (360 N·m for ankles) clamps. So the
# baseline KPs below are intentionally large; the per-channel
# clamps below set the actuator's saturation envelope.
#
# Sign convention: pitch>0 means pelvis forward of feet. A
# NEGATIVE delta on ankle pitch / hip pitch / back pitch rotates
# each joint to push pelvis BACK toward zero.
KP_ANKLE_PITCH = -20.0; KD_ANKLE_PITCH = -3.0
KP_ANKLE_ROLL  = -20.0; KD_ANKLE_ROLL  = -3.0
KP_HIP_PITCH   = -12.0; KD_HIP_PITCH   = -2.0
KP_HIP_ROLL    = -8.0;  KD_HIP_ROLL    = -1.2
KP_BACK_PITCH  = -8.0;  KD_BACK_PITCH  = -1.2
KP_BACK_ROLL   = -5.0;  KD_BACK_ROLL   = -0.8
ANKLE_CLAMP = 1.0
HIP_CLAMP   = 1.0
BACK_CLAMP  = 0.75
BAL_CLAMP = ANKLE_CLAMP   # back-compat alias for journey doc references

# Joint indices into ATLAS_JOINTS for the joints involved in balance.
_L_AP = ATLAS_JOINTS.index("l_leg_aky")
_R_AP = ATLAS_JOINTS.index("r_leg_aky")
_L_AR = ATLAS_JOINTS.index("l_leg_akx")
_R_AR = ATLAS_JOINTS.index("r_leg_akx")
_L_HP = ATLAS_JOINTS.index("l_leg_hpy")
_R_HP = ATLAS_JOINTS.index("r_leg_hpy")
_L_HR = ATLAS_JOINTS.index("l_leg_hpx")
_R_HR = ATLAS_JOINTS.index("r_leg_hpx")
_BKY  = ATLAS_JOINTS.index("back_bky")
_BKX  = ATLAS_JOINTS.index("back_bkx")

# Episode termination. New upright NOMINAL settles at bz ≈ 0.93;
# spawn slightly above so feet contact in 1-2 substeps.
SPAWN_Z = 0.95
BZ_FAIL = 0.55
ROLL_FAIL = 0.8
PITCH_FAIL = 0.8
MAX_EP = 500

# Joint position limits from the URDF (mirrors atlas_robot_spec.JOINT_LIMITS).
JOINT_LIMITS_LO = np.array([
    # back
    -0.663225, -0.219388, -0.523599,
    # l_arm
    -1.5708, -1.5708,  0.0,     0.0,    -3.011, -1.7628, -2.9671,
    # neck
    -0.602139,
    # r_arm
    -0.785398, -1.5708,  0.0,    -2.35619, -3.011, -1.7628, -2.9671,
    # l_leg
    -0.174358, -0.523599, -1.61234,  0.0, -1.0,  -0.8,
    # r_leg
    -0.786794, -0.523599, -1.61234,  0.0, -1.0,  -0.8,
], dtype=np.float32)
assert JOINT_LIMITS_LO.shape == (NJ,)
JOINT_LIMITS_HI = np.array([
    # back
    +0.663225, +0.538783, +0.523599,
    # l_arm
    +0.785398, +1.5708, +3.14159, +2.35619, +3.011, +1.7628, +2.9671,
    # neck
    +1.14319,
    # r_arm
    +1.5708,    +1.5708, +3.14159,  0.0,    +3.011, +1.7628, +2.9671,
    # l_leg
    +0.786794, +0.523599, +0.65764, +2.35637, +0.7, +0.8,
    # r_leg
    +0.174358, +0.523599, +0.65764, +2.35637, +0.7, +0.8,
], dtype=np.float32)
assert JOINT_LIMITS_HI.shape == (NJ,)


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
# Batched env — N parallel Atlas standing instances on GPU mujoco_warp.
# ────────────────────────────────────────────────────────────────────
class BatchedAtlasStandEnv:
    """Standing-task batched env. No gait, no IK; baseline is NOMINAL +
    ankle balance PD. Residual policy adds ±RES_SCALE rad on top.
    """

    def __init__(self, n, mjcf, device="cuda:0", reward_cfg=None, sim_dt=0.0,
                 dr_cfg=None):
        import warp as wp
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mjw = wp, mjw
        self.n = n
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

        # Domain randomization on the MJCF model BEFORE putting it on
        # the device. mujoco_warp doesn't support per-env model params
        # cheaply, so each TRAINING RUN samples one point in the
        # manifold. Run-to-run variation + per-step perturbations
        # together produce a policy robust to a band of physics.
        rng = np.random.default_rng(self.dr.get("seed", 0))
        mass_scale_band = self.dr.get("mass_scale", 0.0)   # ±fraction
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
            # njmax/nconmax must be passed explicitly — mujoco_warp's
            # put_data() does NOT read the <size njmax="..."> in the MJCF.
            # The default (derived from nv only) is 53 for Atlas, so the
            # solver overflows ~once per minute when transient stand
            # collapses produce 60-70 contact constraint rows. 256 is the
            # value we wrote in the import script for consistency.
            self.mw_d = mjw.put_data(self.mjm, mjd, nworld=n,
                                     njmax=256, nconmax=128)

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

        self.r = reward_cfg or {}

        # Seed pose: at SPAWN_Z, identity orientation, NOMINAL leg pose.
        self.seed_qpos = mjd.qpos.copy().astype(np.float32)
        self.seed_qpos[0:3] = [0.0, 0.0, SPAWN_Z]
        self.seed_qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for i, jn in enumerate(LEGS_JOINTS):
            self.seed_qpos[self.controller_to_qpos[i]] = NOMINAL[i]
        # The Newton-dumped MJCF carries two `_placeholder_free_N` free
        # joints (bodies that don't correspond to Atlas links — Newton's
        # XPBD bookkeeping artefacts). The dump's default position for
        # one of them is at world origin (0,0,0) with a 0.12 m contact
        # sphere, which intersects the ground plane and produces a huge
        # first-step impulse propagating through Atlas via global
        # constraint solver coupling. Park them far above/below the
        # arena so they never contact anything.
        for jname in ("_placeholder_free_0", "_placeholder_free_1"):
            jid = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid >= 0:
                qa = self.mjm.jnt_qposadr[jid]
                # Free joint: [x, y, z, qw, qx, qy, qz]
                self.seed_qpos[qa + 0] = 0.0
                self.seed_qpos[qa + 1] = 0.0
                self.seed_qpos[qa + 2] = 1.0e6   # high enough to never contact
                self.seed_qpos[qa + 3] = 1.0
                self.seed_qpos[qa + 4] = 0.0
                self.seed_qpos[qa + 5] = 0.0
                self.seed_qpos[qa + 6] = 0.0
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

        # Per-env spawn jitter on the 30 controller joints.
        # CRITICAL: clamp the realized qpos to an INTERIOR envelope of
        # the URDF joint limits — Atlas's NOMINAL puts l_arm_elx and
        # r_arm_elx only 0.10 rad from a joint limit, so init_q_band=0.15
        # uncovered ~30 % of envs starting out of range. mujoco_warp then
        # fires a big Baumgarte limit-restoring impulse on step 1 that
        # propagates up the arm chain into the 84 kg utorso and corrupts
        # PPO gradients over training. The DR knob `init_q_band` is
        # unchanged at the spec level; only the realized sample is
        # truncated. Same idea every modern legged-RL trainer uses.
        if self._init_q_band > 0:
            jitter = (torch.rand(m, NJ, device=self.tdev) * 2 - 1) * self._init_q_band
            base_idx = idx.unsqueeze(1).expand(-1, NJ)               # (m, NJ)
            col_idx = self.qpos_idx_t.unsqueeze(0).expand(m, -1)     # (m, NJ)
            q_new = self.nominal_t.unsqueeze(0).expand(m, -1) + jitter
            # 0.02 rad ≈ 1.1° clearance from each limit wall.
            q_new = torch.clamp(q_new, self.jl_lo_t + 0.02, self.jl_hi_t - 0.02)
            self.qpos_t[base_idx, col_idx] = q_new
        if self._init_xy_band > 0:
            self.qpos_t[idx, 0] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_xy_band
            self.qpos_t[idx, 1] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_xy_band
        if self._init_z_band > 0:
            self.qpos_t[idx, 2] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_z_band

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
        obs = torch.cat([vlin, vang, pg, q, qd, self.last_action_t], dim=1)
        if self._obs_noise > 0:
            obs = obs + torch.randn_like(obs) * self._obs_noise
        obs = torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        obs = torch.clamp(obs, -10.0, 10.0)
        return obs

    def _baseline_targets_t(self):
        """NOMINAL + COUPLED ankle/hip/back balance PD, all on GPU.

        Distributes pitch/roll correction across the ankle, hip and
        lumbar joints (vs the ankle-only baseline that worked for G1).
        Atlas's 175 kg pelvis needs torque authority from the whole
        leg chain to survive G1-class DR long enough for PPO to find
        signal. Returns (n, NJ) joint targets.
        """
        qp = self.qpos_t
        w = qp[:, 3]; x = qp[:, 4]; y = qp[:, 5]; z = qp[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sinp = torch.clamp(2 * (w * y - z * x), -1.0, 1.0)
        pitch = torch.asin(sinp)
        roll_rate = (roll - self.prev_roll_t) / DT
        pitch_rate = (pitch - self.prev_pitch_t) / DT
        self.prev_roll_t = roll
        self.prev_pitch_t = pitch

        # Ankle strategy — fast, small range.
        ap = torch.clamp(KP_ANKLE_PITCH * pitch + KD_ANKLE_PITCH * pitch_rate,
                         -ANKLE_CLAMP, ANKLE_CLAMP)
        ar = torch.clamp(KP_ANKLE_ROLL * roll + KD_ANKLE_ROLL * roll_rate,
                         -ANKLE_CLAMP, ANKLE_CLAMP)
        # Hip strategy — bigger torque arm to pelvis CoM.
        hp = torch.clamp(KP_HIP_PITCH * pitch + KD_HIP_PITCH * pitch_rate,
                        -HIP_CLAMP, HIP_CLAMP)
        hr = torch.clamp(KP_HIP_ROLL * roll + KD_HIP_ROLL * roll_rate,
                        -HIP_CLAMP, HIP_CLAMP)
        # Back/lumbar — pulls the heavy utorso (84 kg) back over hips.
        bp = torch.clamp(KP_BACK_PITCH * pitch + KD_BACK_PITCH * pitch_rate,
                        -BACK_CLAMP, BACK_CLAMP)
        br = torch.clamp(KP_BACK_ROLL * roll + KD_BACK_ROLL * roll_rate,
                        -BACK_CLAMP, BACK_CLAMP)

        targets = self.nominal_t.unsqueeze(0).expand(self.n, -1).contiguous()
        targets[:, _L_AP] = targets[:, _L_AP] + ap
        targets[:, _R_AP] = targets[:, _R_AP] + ap
        targets[:, _L_AR] = targets[:, _L_AR] + ar
        targets[:, _R_AR] = targets[:, _R_AR] + ar
        targets[:, _L_HP] = targets[:, _L_HP] + hp
        targets[:, _R_HP] = targets[:, _R_HP] + hp
        targets[:, _L_HR] = targets[:, _L_HR] + hr
        targets[:, _R_HR] = targets[:, _R_HR] + hr
        targets[:, _BKY]  = targets[:, _BKY]  + bp
        targets[:, _BKX]  = targets[:, _BKX]  + br
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

        # Reward + done all on GPU. Sanitize qpos/qvel first — Atlas at
        # 175 kg under heavy DR can occasionally produce NaN qpos/qvel
        # when an init-perturbed pose causes deep penetration on the
        # first physics step. Treat those envs as falls.
        nan_state = (torch.isnan(self.qpos_t).any(dim=1)
                     | torch.isnan(self.qvel_t).any(dim=1)
                     | torch.isinf(self.qpos_t).any(dim=1)
                     | torch.isinf(self.qvel_t).any(dim=1))
        bz = torch.nan_to_num(self.qpos_t[:, 2], nan=0.0, posinf=10.0, neginf=-10.0)
        w = torch.nan_to_num(self.qpos_t[:, 3], nan=1.0)
        x = torch.nan_to_num(self.qpos_t[:, 4], nan=0.0)
        y = torch.nan_to_num(self.qpos_t[:, 5], nan=0.0)
        z = torch.nan_to_num(self.qpos_t[:, 6], nan=0.0)
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sinp = torch.clamp(2 * (w * y - z * x), -1.0, 1.0)
        pitch = torch.asin(sinp)
        qvel_safe = torch.nan_to_num(self.qvel_t, nan=0.0, posinf=10.0, neginf=-10.0)
        vlin_norm = torch.linalg.norm(qvel_safe[:, 0:3], dim=1)
        vang_norm = torch.linalg.norm(qvel_safe[:, 3:6], dim=1)

        r = self.r
        r_alive = r.get("alive", 1.0) * torch.ones(self.n, device=self.tdev)
        # Quadratic upright (G1 shape). The exp(-4θ²) variant was tried
        # alongside term=-10 in attempt 5 and made PPO worse, not better
        # — the much larger fall penalty pushed the policy to thrash.
        upright = torch.clamp(1.0 - roll * roll - pitch * pitch, min=0.0)
        r_up = r.get("upright", 0.5) * upright
        r_lin = r.get("lin", -0.1) * vlin_norm
        r_ang = r.get("ang", -0.05) * vang_norm
        r_act = r.get("act", -0.01) * (action_t * action_t).sum(dim=1)
        reward = r_alive + r_up + r_lin + r_ang + r_act
        # Bounded per-step reward — Atlas at 175 kg under heavy DR can
        # produce huge velocity terms in the first iterations that
        # otherwise blow up the value-function loss / gradients.
        reward = torch.clamp(reward, -3.0, 3.0)

        fall = (torch.abs(roll) > ROLL_FAIL) | (torch.abs(pitch) > PITCH_FAIL) | (bz < BZ_FAIL) | nan_state
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
                   default=str(REPO / "projects/robots/boston_dynamics/atlas/urdf/atlas.mjcf.xml"))
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save",
                   default=str(REPO / "projects/policies/research/training/runs/gpu_atlas_stand_robust/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=512)
    p.add_argument("--validate-baseline", action="store_true",
                   help="run zero-action (baseline-only) survival measure "
                        "under whatever DR is configured, then exit. Use "
                        "to gate baseline strength before a full PPO run.")
    p.add_argument("--validate-steps", type=int, default=300)
    p.add_argument("--passive", action="store_true",
                   help="(validate only) zero out balance feedback — "
                        "hold raw NOMINAL pose. Diagnoses seed stability.")
    p.add_argument("--alive", type=float, default=1.0)
    p.add_argument("--upright", type=float, default=0.5)
    p.add_argument("--lin", type=float, default=-0.1)
    p.add_argument("--ang", type=float, default=-0.05)
    p.add_argument("--act", type=float, default=-0.01)
    p.add_argument("--term", type=float, default=-1.0)
    p.add_argument("--sim-dt", type=float, default=0.0)
    p.add_argument("--init-from", default=None)
    # Domain randomization — defaults tuned for sim-to-deploy robustness.
    # The deploy wrapper introduces 1-tick control-delay + per-step state
    # sync that the trainer's raw mjw.step doesn't see. Training over the
    # union of mass/friction/gain/latency/push variation produces a
    # policy that doesn't care about which specific wrapper runs it.
    # Atlas-safe default: 0.30 (G1's value) drives mujoco_warp into NaN
    # on Atlas's 30-DOF, 175 kg body within the first physics step.
    # 0.20 is the empirical safe ceiling — still meaningfully diverse,
    # 4× wider than the 0.05 you'd get from naive mass-noise.
    p.add_argument("--dr-mass-scale", type=float, default=0.20,
                   help="±fraction PER-BODY mass+inertia jitter")
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
    p.add_argument("--dr-seed", type=int, default=0,
                   help="seed for the per-run model-param draws")
    p.add_argument("--no-dr", action="store_true",
                   help="disable all domain randomization")
    # PPO-side knobs. Defaults chosen for Atlas-class bipeds where the
    # baseline already produces near-max reward at iter 1 — a high
    # entropy bonus then prevents log_std from decaying, leaving the
    # residual stuck at ~0.05 rad RMS per joint, which destabilizes
    # the strong baseline and drives V down. log_std_init=-2 starts
    # quieter; ent_coef=0 lets the policy lock in to deterministic.
    p.add_argument("--log-std-init", type=float, default=-2.0,
                   help="initial log_std for the Gaussian policy")
    p.add_argument("--ent-coef", type=float, default=0.0,
                   help="entropy bonus coefficient")
    args = p.parse_args()

    if not Path(args.mjcf).exists():
        raise SystemExit(
            f"MJCF not found: {args.mjcf}. Build it first by running\n"
            f"  python projects/policies/research/training/build_g1_mjcf.py")

    reward_cfg = dict(alive=args.alive, upright=args.upright,
                      lin=args.lin, ang=args.ang, act=args.act,
                      term=args.term)
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
            seed=args.dr_seed,
        )
        print(f"[DR] {dr_cfg}")
    env = BatchedAtlasStandEnv(args.envs, args.mjcf, reward_cfg=reward_cfg,
                               sim_dt=args.sim_dt, dr_cfg=dr_cfg)
    if getattr(args, "passive", False):
        # Zero every balance gain so the baseline returns NOMINAL only.
        # Used to diagnose seed stability vs feedback sign.
        global KP_ANKLE_PITCH, KD_ANKLE_PITCH, KP_ANKLE_ROLL, KD_ANKLE_ROLL
        global KP_HIP_PITCH, KD_HIP_PITCH, KP_HIP_ROLL, KD_HIP_ROLL
        global KP_BACK_PITCH, KD_BACK_PITCH, KP_BACK_ROLL, KD_BACK_ROLL
        KP_ANKLE_PITCH = KD_ANKLE_PITCH = 0.0
        KP_ANKLE_ROLL = KD_ANKLE_ROLL = 0.0
        KP_HIP_PITCH = KD_HIP_PITCH = 0.0
        KP_HIP_ROLL = KD_HIP_ROLL = 0.0
        KP_BACK_PITCH = KD_BACK_PITCH = 0.0
        KP_BACK_ROLL = KD_BACK_ROLL = 0.0
        print("[passive] all balance gains zeroed — holding raw NOMINAL")
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
            # Zero-init the final pi layer so the residual mean μ is
            # EXACTLY zero at iter 1. The baseline already produces
            # near-max reward on its own; any non-zero μ at init only
            # destabilizes that. PPO will move μ off zero only when
            # genuine advantage signal exceeds the act² anchor.
            with torch.no_grad():
                pi_last = self.pi[-1]
                pi_last.weight.zero_()
                pi_last.bias.zero_()
                # Same trick on the value head — V starts at 0, climbs
                # cleanly to true returns rather than chasing random init.
                v_last = self.v[-1]
                v_last.weight.zero_()
                v_last.bias.zero_()
            # log_std starts small so the residual is nearly zero on
            # day 1 — the analytic baseline carries the initial state.
            # -2 is quieter than G1's -1 — Atlas's strong coupled
            # baseline leaves little room for residual to help, so
            # too much initial noise destabilizes learning.
            self.log_std = nn.Parameter(args.log_std_init * torch.ones(NJ))

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

    if args.validate_baseline:
        # Zero action everywhere → policy output is zero → only
        # baseline (+ residual scaled by 0) reaches actuators. We
        # measure how many env-steps each env survives under the
        # configured DR. This is the gate that tells us if the
        # analytic balance controller is strong enough for the heavy
        # DR profile *before* we burn a PPO run.
        obs = env.reset()
        zero = torch.zeros(env.n, NJ, device=tdev)
        survived = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        fallen = torch.zeros(env.n, dtype=torch.bool, device=tdev)
        for step in range(args.validate_steps):
            obs, _, done, _ = env.step(zero)
            survived += (~fallen).to(torch.int32)
            fallen = fallen | done
            if fallen.all():
                break
        s = survived.cpu().numpy()
        nan_envs = int((s == 0).sum())
        # Quantiles tell us the distribution — median ≥ 30 is the
        # gate target (matches G1 baseline survival under G1 DR).
        q = np.quantile(s, [0.10, 0.25, 0.50, 0.75, 0.90])
        print(f"[baseline-validate] envs={env.n} steps={args.validate_steps}")
        print(f"  survival steps: "
              f"mean={s.mean():.1f}  median={int(q[2])}  "
              f"min={s.min()}  max={s.max()}")
        print(f"  quantiles 10/25/50/75/90 = "
              f"{int(q[0])}/{int(q[1])}/{int(q[2])}/{int(q[3])}/{int(q[4])}")
        print(f"  fell_in_0_steps={nan_envs}  "
              f"frac_full={(s >= args.validate_steps).mean():.2f}")
        # Gate: median ≥ 30 steps under the configured DR.
        ok = int(q[2]) >= 30
        print(f"  gate (median>=30): {'PASS' if ok else 'FAIL'}")
        return

    if args.eval:
        ac.eval()
        ac.load_state_dict(torch.load(args.save, map_location=tdev))
        obs = env.reset()
        survived = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        for step in range(args.eval_steps):
            with torch.no_grad():
                mu, _, _ = ac(obs)
            obs, _, done, info = env.step(mu)
            survived += (~done).to(torch.int32)
        s = survived.cpu().numpy()
        print(f"[gpu-eval] policy={args.save} envs={env.n} steps={args.eval_steps}")
        print(f"  survival steps: mean={s.mean():.1f}  "
              f"median={np.median(s):.0f}  max={s.max()}  "
              f"frac_full={(s >= args.eval_steps).mean():.2f}")
        return

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)

    rollout = args.rollout
    obs = env.reset()
    total_steps = 0
    t0 = time.time()
    # Track the best policy by smoothed per-step reward — Atlas PPO
    # rolls off after iter ~30 (the residual gradient is mostly noise on
    # a near-saturating baseline), so the final weights are usually
    # worse than peak. Save both: `args.save` = best-yet, `last.pt` =
    # final iter. Deploy picks `args.save`.
    best_ep_rew = float("-inf")
    rew_ema = None

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
                # Diagnostic: catch NaN/Inf flowing in from the env before
                # they corrupt the policy gradients. With Atlas at 30 DOFs
                # and aggressive DR, mujoco_warp can occasionally explode
                # an env (joints violate limits hard, body penetrates ground,
                # etc.) and the bad qpos propagates into obs.
                if it == 1:
                    nan_obs = torch.isnan(obs).any().item()
                    inf_obs = torch.isinf(obs).any().item()
                    nan_mu = torch.isnan(mu).any().item()
                    if nan_obs or inf_obs or nan_mu:
                        which = torch.isnan(obs).any(dim=1).nonzero().flatten()[:5].tolist()
                        print(f"[diag] it={it} k={k} nan_obs={nan_obs} "
                              f"inf_obs={inf_obs} nan_mu={nan_mu} "
                              f"obs_max={obs.abs().max().item():.3f} "
                              f"nan_env_ids[:5]={which}")
                        raise SystemExit("NaN detected — see [diag]")
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

        if it == 1:
            print(f"[diag-rollout-end] "
                  f"rew nan={torch.isnan(rew_buf).any().item()} "
                  f"inf={torch.isinf(rew_buf).any().item()} "
                  f"max={rew_buf.abs().max().item():.3f} | "
                  f"val nan={torch.isnan(val_buf).any().item()} "
                  f"max={val_buf.abs().max().item():.3f} | "
                  f"obs nan={torch.isnan(obs_buf).any().item()} "
                  f"max={obs_buf.abs().max().item():.3f} | "
                  f"last_obs nan={torch.isnan(obs).any().item()} "
                  f"max={obs.abs().max().item():.3f}")

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
        # Diagnostic: print return statistics on it==1 so we can see if
        # values are exploding before the gradient step.
        if it == 1:
            print(f"[diag-pre-update] ret stats: "
                  f"min={ret_flat.min().item():.2f} "
                  f"max={ret_flat.max().item():.2f} "
                  f"mean={ret_flat.mean().item():.2f} "
                  f"std={ret_flat.std().item():.2f} | "
                  f"adv min={adv_flat.min().item():.2f} "
                  f"max={adv_flat.max().item():.2f}")
        for _epoch in range(4):
            mu, v, log_std = ac(obs_flat)
            if torch.isnan(mu).any() or torch.isnan(v).any():
                print(f"[nan] it={it} epoch={_epoch} mu_nan="
                      f"{torch.isnan(mu).any().item()} v_nan="
                      f"{torch.isnan(v).any().item()} "
                      f"log_std={log_std.detach().cpu().tolist()}")
                raise SystemExit("NaN in actor output during update")
            std = log_std.exp()
            dist = torch.distributions.Normal(mu, std)
            new_logp = dist.log_prob(act_flat).sum(-1)
            ratio = (new_logp - logp_flat).exp()
            surr1 = ratio * adv_flat
            surr2 = torch.clamp(ratio, 1 - clip_eps, 1 + clip_eps) * adv_flat
            pi_loss = -torch.min(surr1, surr2).mean()
            # Clip value targets to the same band as per-step rewards
            # so a single exploding env can't dominate the value loss
            # on iteration 1. Without this, Atlas (175 kg + heavy DR)
            # spikes |v_loss| → ∞ → NaN params on the first step.
            v_target = torch.clamp(ret_flat, -50.0, 50.0)
            v_loss = ((v - v_target) ** 2).mean()
            ent = dist.entropy().sum(-1).mean()
            loss = pi_loss + 0.25 * v_loss - args.ent_coef * ent
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5)
            opt.step()

        # Smoothed reward (EMA) for best-checkpoint tracking — a single
        # noisy iter shouldn't latch as "best."
        ep_rps_now = rew_buf.mean().item()
        rew_ema = ep_rps_now if rew_ema is None else 0.9 * rew_ema + 0.1 * ep_rps_now
        if rew_ema > best_ep_rew:
            best_ep_rew = rew_ema
            Path(args.save).parent.mkdir(parents=True, exist_ok=True)
            torch.save(ac.state_dict(), args.save)

        if it % 5 == 0 or it == 1:
            mean_v = val_buf.mean().item()
            fps = total_steps / max(time.time() - t0, 1e-6)
            print(f"it {it:4d}  ep_rew/step~{ep_rps_now:+.3f}  "
                  f"ema {rew_ema:+.3f}  best {best_ep_rew:+.3f}  "
                  f"meanV {mean_v:+.2f}  steps {total_steps:,}  "
                  f"{fps:,.0f} env-steps/s")

    # Save the FINAL state too — useful for debugging post-mortem.
    last_path = Path(args.save).with_name("last.pt")
    torch.save(ac.state_dict(), last_path)
    print(f"final iter saved at {last_path}; best (by EMA reward) "
          f"already at {args.save}")
    # Load the BEST policy back for ONNX export so the deployed ONNX
    # corresponds to the best-yet checkpoint.
    ac.load_state_dict(torch.load(args.save, map_location=tdev))
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
            # dynamo=False forces the legacy torchscript-based exporter
            # path. Newer torch defaults to dynamo=True, which conflicts
            # with `dynamic_axes` and was observed to hang indefinitely
            # on the 30-DOF Atlas actor at end-of-training.
            torch.onnx.export(
                wrapped, dummy, str(onnx_path),
                input_names=["obs"], output_names=["action"],
                dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
                opset_version=17,
                dynamo=False,
            )
        sys.stdout.write(buf.getvalue().encode("ascii", "replace").decode("ascii"))
        print(f"exported ONNX -> {onnx_path}")
    except Exception as e:
        print(f"ONNX export failed (non-fatal): {e}")


if __name__ == "__main__":
    main()
