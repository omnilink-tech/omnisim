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

"""BATCHED ON-ENGINE RL FINE-TUNE for the G1 walk policy on Newton's
SolverMuJoCo -- the EXACT OmniSim deploy solver.

WHY THIS FILE EXISTS
--------------------
gpu_mjwarp_g1_walk_trainer.py trains against raw ``mjw.step`` (plain
mujoco_warp). The OmniSim deploy steps through ``newton.solvers.SolverMuJoCo``,
which wraps mjw.step with extra control-application + contact-conversion +
state-sync (the "Newton gap"). A policy that survives in the mjwarp trainer
loses balance ~1 s into deploy. This trainer FINE-TUNES a warm-started mjwarp
policy directly on SolverMuJoCo so it adapts to the deploy dynamics and
transfers.

It is NOT a from-scratch trainer (from-scratch PPO under Newton fails -- see
project_spot_newton_residual_recipe): it warm-starts the proven 226->512x3->13
walk actor and adapts it with light DR.

THE CORRECTNESS CRUX -- the index map (verified, see _verify_index_map)
----------------------------------------------------------------------
Built via build_g1_native_prim (ke=100, kd=5 to match the deploy) ->
ModelBuilder.replicate(world_count=N) -> finalize -> SolverMuJoCo. State is read
from ``solver.mjw_data.qpos`` / ``.qvel`` (mujoco_warp data) as ZERO-COPY torch
views, shape ``[N, nq]`` / ``[N, nv]``. Probed on newton 1.2.0 / mjw 3.8.0.3:

  qpos [N, 20]:  [0:3]=base pos, [3:7]=base quat (w,x,y,z), [7:20]=13 joints
  qvel [N, 19]:  [0:3]=base lin vel (world), [3:6]=base ang vel, [6:19]=13 qd

The 13 joints land in qpos at slots 7+i / qvel 6+i, in the SAME order as
LEGS_JOINTS (LHP,LHR,LHY,LKN,LAP,LAR, RHP,RHR,RHY,RKN,RAP,RAR, waist) -- but we
DO NOT trust that blindly: we map BY NAME through the solver's companion CPU
MjModel (``solver.mj_model``; joints carry a ``g1_legs_`` prefix in the merged
MJCF) and ASSERT the seeded NOMINAL reads back (_verify_index_map). The quat is
(w,x,y,z) and velocities are world-frame -- IDENTICAL to the MuJoCo convention
the mjwarp env's _build_obs_t already assumes, so the 226-d obs / baseline /
reward code is reused unchanged.

Control: ``control.joint_target_q`` is newton-DOF-indexed and flat over worlds
(shape [N*19]); per world w, joint j -> slot 19*w + 6 + j (6 free DOFs first).
We build that flat write-index once and scatter targets each step.

Run from PowerShell (native warp/CUDA). Build is done with build_g1_native_prim
which strips mesh colliders to PRIMITIVE feet so the broad-phase doesn't overflow
under replication.

Usage (smoke):
    python projects/policies/research/training/gpu_newton_g1_walk_trainer.py \
        --init-from projects/policies/research/training/runs/gpu_g1_walk_aggdr_c1/policy.pt \
        --envs 256 --iters 5 \
        --save projects/policies/research/training/runs/gpu_newton_g1_walk_smoke/policy.pt

Fine-tune (matches the aggdr_c1 warm-start config -- MUST be identical or the
obs layout won't match the policy):
    python projects/policies/research/training/gpu_newton_g1_walk_trainer.py \
        --init-from projects/policies/research/training/runs/gpu_g1_walk_aggdr_c1/policy.pt \
        --envs 2048 --iters 300 --obs-stack 4 --obs-lookahead 0.1,0.4 \
        --hidden-dims 512,512,512 --hold-arms --gait-model human \
        --gait-style winter --gait-a-arm 0.25 --gait-a-lat 0.05 --vx-target 0.4 \
        --gait-freq 1.3 --gait-ramp-s 2.0 --gait-hip-scale 0.9 \
        --rest-start-frac 0.4 --res-scale 0.3 \
        --save projects/policies/research/training/runs/gpu_newton_g1_walk_ft/policy.pt
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

# Reuse the mjwarp env's layout constants, obs/baseline/reward METHODS, the
# numpy helpers, the gait, and the PPO/network/ONNX shape. We subclass the env
# so the obs is byte-identical to what the warm-start policy expects -- the
# whole point of the fine-tune.
from projects.policies.research.training import gpu_mjwarp_g1_walk_trainer as MJ  # noqa: E402
# SINGLE SOURCE OF TRUTH for the physics model (train<->deploy). Gains, substeps,
# the foot collision box, and the clamp velocity limits are SOURCED from the spec
# so they can never drift from the deploy. Every value is UNCHANGED (proven
# old==new before this refactor); see projects/policies/research/backends/g1_physics.json.
from projects.policies.research.backends import g1_physics_spec as SPEC  # noqa: E402

NJ = MJ.NJ                # 13
OBS_DIM = MJ.OBS_DIM      # 50
DT = MJ.DT                # 0.016
RES_SCALE = MJ.RES_SCALE  # 0.3
LEGS_JOINTS = MJ.LEGS_JOINTS
ARM_JOINTS = MJ.ARM_JOINTS
ARM_NOMINAL = MJ.ARM_NOMINAL
NOMINAL = MJ.NOMINAL
SPAWN_Z = MJ.SPAWN_Z

# Deploy gains (NOT the build_g1_native default 20/3). The deploy world runs
# ke=100/kd=5 -- sourced from the spec (SPEC.KE/SPEC.KD) so train and deploy
# share one number. The G1_NEWTON_KE/KD env overrides are preserved.
KE_DEPLOY = float(os.environ.get("G1_NEWTON_KE", SPEC.KE))
KD_DEPLOY = float(os.environ.get("G1_NEWTON_KD", SPEC.KD))

# Newton deploy substep loop (matches build_g1_native_prim / the deploy).
SUBSTEPS = SPEC.SUBSTEPS
SUB_DT = DT / SUBSTEPS

# Per-world generalized layout (free joint first), verified via the probes.
FREE_QPOS = 7   # base pos(3) + quat(4)
FREE_QVEL = 6   # base lin(3) + ang(3)
FREE_DOF = 6    # control.joint_target_pos free-DOF offset

# Full-body (23-DOF) deploy URDF: 13 legs+waist + 10 arms. --hold-arms trains
# the 13-DOF leg policy against the FULL arm mass (the deploy body), arms pinned
# at ARM_NOMINAL -- matches the warm-start (aggdr_c1 was --hold-arms).
_G1_FULL_URDF = REPO / "projects/robots/unitree/g1/urdf/g1_23dof_omnisim.urdf"
_FOOT_LINKS = ("left_ankle_roll_link", "right_ankle_roll_link")


# ── MIRROR-SYMMETRY maps (sagittal-plane reflection; left<->right) ──────────
# A mirrored state must yield a mirrored action (pi(mirror(s)) == mirror(pi(s)))
# for a symmetric gait. We enforce it with a soft loss (--mirror-loss): the
# dominant remaining ghost-fidelity gap is L/R asymmetry (R leg deviates from
# the ghost far more than L). Joint permutation + sign for the 13-DOF
# LEGS_JOINTS frame (action AND the q/qd/last_action/lookahead obs blocks).
# Sign SOURCE OF TRUTH: g1_human_gait.py msign=[1,-1,-1,1,1,-1] over
# (hip_pitch,roll,yaw,knee,ankle_pitch,ankle_roll) -- roll/yaw flip; waist_yaw
# (transverse) also flips. This is a pre-existing, self-tested convention.
_MIRROR_P = [6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 12]          # L<->R swap, waist self
_MIRROR_S = [1.0, -1.0, -1.0, 1.0, 1.0, -1.0,
             1.0, -1.0, -1.0, 1.0, 1.0, -1.0, -1.0]             # roll/yaw/waist flip


def _build_mirror_obs_op(obs_in, obs_dim, nj, n_look, tdev):
    """Returns mirror_obs(obs)->obs for the FULL actor input. Layout (must match
    _build_obs_t + the stack/lookahead assembly): obs_stack copies of the 50-d
    base obs [vlin3, vang3, pg3, q13, qd13, last_action13, gait2], then n_look
    13-d lookahead blocks. Assumes vx_cond OFF (no trailing command column)."""
    p = torch.tensor(_MIRROR_P, dtype=torch.long, device=tdev)
    s = torch.tensor(_MIRROR_S, dtype=torch.float32, device=tdev)
    stack = (obs_in - nj * n_look) // obs_dim     # = obs_stack (vx_cond off)

    def mirror(obs):
        out = obs.clone()
        for f in range(int(stack)):
            b = f * obs_dim
            # base lin vel: flip vy (idx1); ang vel (pseudovector): flip wx,wz
            # (idx3,5); proj gravity: flip gy (idx7). (Sagittal-plane reflect.)
            out[:, b + 1] = -obs[:, b + 1]
            out[:, b + 3] = -obs[:, b + 3]
            out[:, b + 5] = -obs[:, b + 5]
            out[:, b + 7] = -obs[:, b + 7]
            # q, qd, last_action: each a 13-block -> L<->R permute + sign
            for off in (9, 9 + nj, 9 + 2 * nj):
                blk = obs[:, b + off: b + off + nj]
                out[:, b + off: b + off + nj] = blk.index_select(1, p) * s
            # gait [sin,cos] at 9+3*nj: phase += pi (half-cycle L/R swap) -> negate
            g = b + 9 + 3 * nj
            out[:, g] = -obs[:, g]
            out[:, g + 1] = -obs[:, g + 1]
        # lookahead blocks (13-d each, LEGS_JOINTS order) after the stacked frames
        base = int(stack) * obs_dim
        for li in range(n_look):
            off = base + li * nj
            blk = obs[:, off: off + nj]
            out[:, off: off + nj] = blk.index_select(1, p) * s
        return out

    return mirror


def _vec3_str(v) -> str:
    """Format a 3-vector as a URDF/XML attribute string ('x y z').

    The URDF is re-parsed into floats, so only the parsed value matters (not the
    textual form): -0.030 and -0.03 are the identical float. Used to emit the
    SPEC foot-box size/origin into the in-memory collision strip.
    """
    return " ".join(repr(float(c)) for c in v)


def _build_g1_full_prim_builder(ke, kd):
    """Full-body G1 (23 revolute joints) as a NATIVE Newton articulation with
    PRIMITIVE collision (strip visuals + mesh colliders, keep the two foot
    boxes), all revolute DOFs as POSITION_VELOCITY actuators at the deploy
    gains. Mirrors build_g1_native_prim but for the 23-DOF deploy URDF so the
    arm mass (+6.1 kg) is present as a passive balance load. Returns the
    un-finalized builder (replicate-able). Per-joint SEED + TARGETS are set BY
    NAME later in the env, so DOF ordering is irrelevant here.

    SINGLE-SOURCE-OF-TRUTH STATUS (Stage 1 URDF): the ideal end-state is loading
    SPEC.URDF_PRIM (the prebaked prim URDF the deploy uses) DIRECTLY, so there is
    literally one collider file. That switch is GATED on model-equivalence and is
    NOT taken, by design. Measured on this machine (newton 1.13 / mjwarp, CPU
    ModelBuilder.add_urdf, both routes): body_count(33), joint_count(33),
    joint_dof_count(29) and PER-BODY MASS are IDENTICAL, and the COLLISION set is
    identical too -- both expose exactly 3 shapes with the COLLIDE_SHAPES flag
    (2 foot BOXes + 1 ground PLANE). BUT total shape_count differs: this in-memory
    strip removes every <visual>, giving shape_count=3; add_urdf(SPEC.URDF_PRIM)
    KEEPS the 29 <visual> meshes (the prim generator strips only MESH <collision>),
    loading them as 29 VISIBLE-only (non-colliding) MESH shapes -> shape_count=32.
    Those 29 render meshes are physics-inert for ONE deploy robot, but here the
    builder is replicate()'d to N (up to 2048) worlds -> ~29*N extra mesh shapes,
    which is exactly the broad-phase blowup build_g1_native_prim was created to
    avoid. So switching would risk the (months-to-train) walker for no physics
    gain. TODO(sot): to truly unify the file, teach make_g1_deploy_prim_urdf to
    ALSO emit a render-stripped variant (or have add_urdf skip <visual> for the
    trainer), re-run the equivalence check incl. shape_count, THEN switch. Until
    then the foot box -- the only collider that matters -- is sourced from SPEC
    on BOTH sides, so the colliders cannot drift even though the file is loaded
    two ways."""
    import xml.etree.ElementTree as ET
    import warp as wp
    import newton
    tree = ET.parse(str(_G1_FULL_URDF))
    root = tree.getroot()
    for link in root.findall("link"):
        name = link.get("name")
        for vis in list(link.findall("visual")):
            link.remove(vis)
        for col in list(link.findall("collision")):
            link.remove(col)
        if name in _FOOT_LINKS:
            col = ET.SubElement(link, "collision")
            origin = ET.SubElement(col, "origin")
            geom = ET.SubElement(col, "geometry")
            box = ET.SubElement(geom, "box")
            # Foot collision box from the SINGLE SOURCE OF TRUTH so the trainer's
            # in-memory strip can never drift from the deploy's prebaked prim URDF
            # (was hardcoded "0.035 0.0 -0.030" / "0.17 0.06 0.012"; values UNCHANGED).
            origin.set("xyz", _vec3_str(SPEC.FOOT_BOX_ORIGIN))
            origin.set("rpy", "0 0 0")
            box.set("size", _vec3_str(SPEC.FOOT_BOX_SIZE))
    urdf_xml = ET.tostring(root, encoding="unicode")
    mb = newton.ModelBuilder()
    mb.add_urdf(urdf_xml,
                xform=wp.transform((0.0, 0.0, SPAWN_Z), (0.0, 0.0, 0.0, 1.0)),
                floating=True)
    pv = int(newton.JointTargetMode.POSITION_VELOCITY)
    # configure EVERY actuated (revolute) DOF: dofs after the 6 free dofs.
    for d in range(FREE_DOF, mb.joint_dof_count):
        mb.joint_target_ke[d] = ke
        mb.joint_target_kd[d] = kd
        mb.joint_target_mode[d] = pv
    mb.add_ground_plane()
    return mb


class BatchedG1NewtonEnv(MJ.BatchedG1StandEnv):
    """G1 walk env stepping through Newton SolverMuJoCo (the deploy solver).

    Reuses BatchedG1StandEnv's _build_obs_t / _baseline_targets_t / _lookahead_obs
    / _stack_obs / _obs_full / reward in step(). We OVERRIDE __init__ (to build
    the Newton model + SolverMuJoCo and the index map instead of put_model/
    put_data), the physics + state plumbing in step(), and _reset_envs (Newton
    has no per-env teleport via a single qpos buffer write + kinematics -- we
    write the mjw_data views and re-sync).

    qpos_t / qvel_t are torch VIEWS of solver.mjw_data, so every obs/reward/
    baseline expression in the parent class operates on the live Newton state.
    """

    def __init__(self, n, reward_cfg=None, dr_cfg=None, hold_arms=False,
                 foot="urdf"):
        import warp as wp
        import newton
        import mujoco
        self.wp = wp
        self.newton = newton
        self.n = n
        self.hold_arms = hold_arms
        self.dr = dr_cfg or {}
        self.r = reward_cfg or {}
        self.device = wp.get_device("cuda:0")
        self.tdev = torch.device("cuda:0")

        # ── Build the batched Newton model: prim G1 (deploy-faithful dynamics,
        # primitive feet) replicated to N worlds, finalized, SolverMuJoCo. ──
        if foot == "task":
            os.environ["OMNISIM_G1PRIM_FOOT"] = "task"
        if hold_arms:
            # full-body (23-DOF) deploy body: arms present as passive mass.
            g1 = _build_g1_full_prim_builder(ke=KE_DEPLOY, kd=KD_DEPLOY)
        else:
            from projects.policies.research.training.build_g1_native_prim import build_g1_prim_builder
            g1 = build_g1_prim_builder(ke=KE_DEPLOY, kd=KD_DEPLOY, add_ground=True)
        main_b = newton.ModelBuilder()
        main_b.replicate(g1, world_count=n, spacing=(3.0, 3.0, 0.0))
        self.model = main_b.finalize()
        assert self.model.world_count == n, \
            f"world_count {self.model.world_count} != n {n}"
        # njmax/nconmax: SolverMuJoCo auto-estimates these too small for the G1
        # full-body foot+ground contact set ("nefc overflow - increase njmax to
        # ~65"), which DROPS constraints -> incomplete foot grip -> the model is
        # only marginally stable. Generous caps remove the overflow (mirrors the
        # mjwarp trainer's G1_NJMAX/G1_NCONMAX=256). Env-overridable.
        _njmax = int(os.environ.get("G1_NJMAX", "128"))
        _nconmax = int(os.environ.get("G1_NCONMAX", "128"))
        self.solver = newton.solvers.SolverMuJoCo(
            self.model, use_mujoco_cpu=False, njmax=_njmax, nconmax=_nconmax)
        self.state_a = self.model.state()
        self.state_b = self.model.state()
        self.control = self.model.control()
        self.contacts = self.model.contacts()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd,
                       self.state_a)

        # ── THE INDEX MAP (mapped BY NAME, then verified). ──
        mjm = self.solver.mj_model                 # companion CPU MjModel
        self.nq = int(mjm.nq)                      # per-world qpos width (20)
        self.nv = int(mjm.nv)                      # per-world qvel width (19)
        self.dof_per_world = self.model.joint_dof_count // n   # 19
        name2qpos, name2dof = {}, {}
        for j in range(mjm.njnt):
            jn = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_JOINT, j)
            name2qpos[jn] = int(mjm.jnt_qposadr[j])
            name2dof[jn] = int(mjm.jnt_dofadr[j])

        def _find(suffix):
            # joints may carry a model-name prefix (e.g. 'g1_legs_'); match the
            # unique joint whose name endswith the controller name.
            hit = [k for k in name2qpos if k == suffix or k.endswith("_" + suffix)
                   or k.endswith(suffix)]
            # prefer exact, else the unique endswith
            if suffix in name2qpos:
                return suffix
            cand = [k for k in name2qpos if k.endswith(suffix)]
            if len(cand) != 1:
                raise RuntimeError(
                    f"joint name '{suffix}' did not map uniquely: {cand}")
            return cand[0]

        self.controller_to_qpos = np.array(
            [name2qpos[_find(j)] for j in LEGS_JOINTS], dtype=np.int64)
        self.controller_to_qvel = np.array(
            [name2dof[_find(j)] for j in LEGS_JOINTS], dtype=np.int64)
        # actuated newton-DOF index per controller joint (per-world-local).
        self.controller_to_dof = np.array(
            [name2dof[_find(j)] for j in LEGS_JOINTS], dtype=np.int64)  # == qvel idx
        if self.hold_arms:
            self.arm_to_qpos = np.array(
                [name2qpos[_find(j)] for j in ARM_JOINTS], dtype=np.int64)
            self.arm_to_dof = np.array(
                [name2dof[_find(j)] for j in ARM_JOINTS], dtype=np.int64)

        # ── Torch views of mjw_data (the live Newton state). ──
        self.qpos_t = wp.to_torch(self.solver.mjw_data.qpos).view(n, self.nq)
        self.qvel_t = wp.to_torch(self.solver.mjw_data.qvel).view(n, self.nv)
        # control.joint_target_pos is flat over worlds (newton-DOF indexed).
        self.ctrl_pos_t = wp.to_torch(self.control.joint_target_q).view(-1)
        assert self.ctrl_pos_t.numel() == n * self.dof_per_world

        # Flat write-index into ctrl_pos_t for the 13 actuated joints of all
        # worlds: world w, joint j -> w*dof_per_world + FREE_DOF + controller_to_dof_local
        # (controller_to_dof is the per-world-local dof = FREE_DOF + j). We use
        # the mapped local dof directly (it already includes the +6 free offset).
        world_off = (torch.arange(n, device=self.tdev) * self.dof_per_world
                     ).unsqueeze(1)                                  # (n,1)
        dof_local = torch.tensor(self.controller_to_dof, dtype=torch.long,
                                 device=self.tdev).unsqueeze(0)      # (1,NJ)
        self.ctrl_write_idx = (world_off + dof_local).reshape(-1)    # (n*NJ,)
        if self.hold_arms:
            arm_local = torch.tensor(self.arm_to_dof, dtype=torch.long,
                                     device=self.tdev).unsqueeze(0)
            self.arm_write_idx = (world_off + arm_local).reshape(-1)

        # ── Constants on GPU (mirror the parent's __init__ tail). ──
        self.qpos_idx_t = torch.tensor(self.controller_to_qpos, dtype=torch.long,
                                       device=self.tdev)
        self.qvel_idx_t = torch.tensor(self.controller_to_qvel, dtype=torch.long,
                                       device=self.tdev)

        # Reuse the parent's reward/gait/obs config setup by calling its helper-
        # heavy tail. The parent __init__ does too much engine-specific work to
        # call directly, so we replicate the relevant config-driven state here.
        self._init_task_state()

        # Seed the model joint_q + control targets to NOMINAL (deploy spawn).
        self._seed_model_nominal()

        # CUDA-graph capture of the substep loop (big speedup).
        self._cuda_graph = None
        self._try_capture_graph()

        # VERIFY THE INDEX MAP before anything trains on it (the crux gate).
        self._verify_index_map()

        self._obs_buf = None
        self._reset_all()

    # ── task/reward/gait config (the subset of parent __init__ that is engine-
    # agnostic; values pulled from reward_cfg exactly like the parent). ──
    def _init_task_state(self):
        r = self.r
        self.nominal = np.array(r.get("nominal", NOMINAL), dtype=np.float32)
        assert self.nominal.shape == (NJ,)
        self.nominal_t = torch.tensor(self.nominal, dtype=torch.float32,
                                      device=self.tdev)
        # Position-limit clamp bounds. INTENTIONALLY sourced from MJ
        # (3-decimal-rounded URDF limits the winning walker was trained with),
        # NOT SPEC.leg_limits() (full URDF precision). They differ by up to
        # ~4e-4 rad; migrating them would change the trained clamp boundary, so
        # this is a documented train<->deploy residual (see g1_physics.json
        # "_residuals" + the JOINT_LIMITS note in gpu_mjwarp_g1_walk_trainer.py).
        self.jl_lo_t = torch.tensor(MJ.JOINT_LIMITS_LO, dtype=torch.float32,
                                    device=self.tdev)
        self.jl_hi_t = torch.tensor(MJ.JOINT_LIMITS_HI, dtype=torch.float32,
                                    device=self.tdev)

        n = self.n
        self.vx_cmd_max = float(r.get("vx_cmd_max", 0.0))
        self.vx_cond = self.vx_cmd_max > 0.0
        self.vx_target = float(r.get("vx_target", MJ.VX_TARGET))
        self.vx_cmd_t = torch.full((n,), self.vx_target, dtype=torch.float32,
                                   device=self.tdev)
        self.vx_phase_freeze = float(r.get("vx_phase_freeze", 0.10))

        self.ep_step_t = torch.zeros(n, dtype=torch.int32, device=self.tdev)
        self.last_action_t = torch.zeros(n, NJ, dtype=torch.float32, device=self.tdev)
        self.prev_roll_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.prev_pitch_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.prev_roll_rate_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.prev_pitch_rate_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.phase_t = torch.zeros(n, dtype=torch.float32, device=self.tdev)
        self.gait_freq = float(r.get("gait_freq", MJ.GAIT_FREQ))
        self.gait_a_hip = float(r.get("gait_a_hip", MJ.GAIT_A_HIP))
        self.gait_a_knee = float(r.get("gait_a_knee", MJ.GAIT_A_KNEE))
        self.gait_a_lat = float(r.get("gait_a_lat", MJ.GAIT_A_LAT))
        self.cp_gain = float(r.get("cp_gain", 0.0))
        self.max_ep = int(r.get("max_ep", MJ.MAX_EP))
        self.res_scale = float(r.get("res_scale", RES_SCALE))
        fr = float(r.get("frontal_res_scale", 1.0))
        rsv = np.full(NJ, self.res_scale, dtype=np.float32)
        for j in (MJ._L_HR, MJ._R_HR, MJ._L_HY, MJ._R_HY):
            rsv[j] *= fr
        self.res_scale_vec = torch.tensor(rsv, device=self.tdev)
        self.gait_a_ankle = float(r.get("gait_a_ankle", 0.0))
        self.seed_gait = bool(r.get("seed_gait", False))
        self.gait_a_arm = float(r.get("gait_a_arm", 0.0))
        self.gait_a_push = float(r.get("gait_a_push", 0.0))
        self.rest_start_frac = float(r.get("rest_start_frac", 0.0))

        self.gait_model = str(r.get("gait_model", ""))
        self.gp = None
        if self.gait_model == "human":
            from projects.policies.control.gait import g1_human_gait as ghg
            self._ghg = ghg
            self._fk = dict(L1=ghg.L1, L2=ghg.L2, TH=ghg.THIGH_OFF,
                            SH=ghg.SHANK_OFF, HS=ghg.HIP_SIGN,
                            KS=ghg.KNEE_SIGN, LF=0.14)
            gpd = r.get("gait_params", {}) or {}
            self.gp = ghg.GaitParams(**gpd)
            self.nominal = ghg.standing_pose(self.gp).astype(np.float32)
            self.vx_target = self.gp.vx
            self.gait_freq = self.gp.freq
            self.nominal_t = torch.tensor(self.nominal, dtype=torch.float32,
                                          device=self.tdev)
        self.phase_dt = 2.0 * math.pi * self.gait_freq * DT

        self._ramp_t0 = torch.zeros(self.n, dtype=torch.float32, device=self.tdev)
        self._swingL = torch.zeros(self.n, dtype=torch.float32, device=self.tdev)
        self._swingR = torch.zeros(self.n, dtype=torch.float32, device=self.tdev)
        self.rw_sched = float(r.get("rw_sched", 0.0))
        self.rw_slip = float(r.get("rw_slip", 0.0))
        self.obs_stack = int(r.get("obs_stack", 1))
        self.obs_look = [float(x) for x in r.get("obs_lookahead", [])]
        self.asym = bool(r.get("asym_critic", False))
        self.priv_extra = 5 if self.asym else 0
        _tw = np.ones(NJ, dtype=np.float32)
        # Ankle imitation weight (CLI --track-ankle-w, default 0.5). The ankles
        # do the fine balancing, and the imitation target (_model_legs) is the
        # PURE open-loop ghost (no ankle PD folded in), so imitating ankles
        # punishes the exact deviation that keeps the biped up. Set 0.0 to free
        # the ankles (hips+knees are the eye-read gait signature anyway).
        _aw = float(r.get("track_ankle_w", 0.5))
        _tw[MJ._L_AP] = _tw[MJ._R_AP] = _aw
        _tw[MJ._L_AR] = _tw[MJ._R_AR] = _aw
        # waist_yaw imitation weight (CLI --track-waist-w, default 0.0 = legacy
        # untracked). The ghost holds the waist at 0; a >0 weight makes the policy
        # hold it still too (it otherwise wobbles ~0.18 rad), matching the ghost.
        _tw[NJ - 1] = float(r.get("track_waist_w", 0.0))
        # SAGITTAL swing up-weight (CLI --track-sagittal-w, default 1.0=legacy).
        # rw_track normalizes by w_trk.sum(), so >1.0 here shifts the imitation
        # error budget onto hip_pitch + knee -- the eye-read swing whose SHAPE
        # the policy otherwise damps (it parks the hip flexed to avoid the
        # CoM-bob that the height penalty punishes). This is the missing lever to
        # raise hip_pitch fidelity (it had no per-joint weight before).
        _sw = float(r.get("track_sagittal_w", 1.0))
        _tw[MJ._L_HP] = _tw[MJ._R_HP] = _sw
        _tw[MJ._L_KN] = _tw[MJ._R_KN] = _sw
        self._track_w = torch.tensor(_tw, device=self.tdev)
        self._hipkn_idx = torch.tensor([MJ._L_HP, MJ._L_KN, MJ._R_HP, MJ._R_KN],
                                       device=self.tdev)
        self.foot_z_contact = float(r.get("foot_z_contact", 0.05))
        self.foot_z_swing = float(r.get("foot_z_swing", 0.14))

        # Foot bodies for the foot-aware rewards: read newton body world
        # positions. Newton xpos is per-body in state_a.body_q[:, 0:3] but that
        # is flat over all bodies (per-world body blocks). The foot rewards are
        # only active when rw_sched/rw_slip/rw_com/asym are set; for the smoke +
        # the documented config they are 0, so we install a lazy body-pos view
        # ONLY if needed (it requires a per-world body index map).
        self._xpos_ready = False
        if (self.rw_sched != 0.0 or self.rw_slip != 0.0
                or float(r.get("rw_com", 0.0)) != 0.0
                or float(r.get("rw_shape", 0.0)) != 0.0 or self.asym):
            self._setup_foot_xpos()

        # action-latency buffer (DR)
        self.max_latency_ticks = int(self.dr.get("action_latency_max", 0))
        self.action_buffer_t = torch.zeros(
            self.n, max(1, self.max_latency_ticks + 1), NJ,
            dtype=torch.float32, device=self.tdev)
        self.action_delay_t = torch.zeros(self.n, dtype=torch.long, device=self.tdev)
        self._act_gain = float(self.dr.get("act_gain", 0.0))
        self.act_gain_t = torch.ones(self.n, dtype=torch.float32, device=self.tdev)
        self._push_p = float(self.dr.get("push_prob", 0.0))
        self._push_vmax = float(self.dr.get("push_vmax", 0.0))
        self._obs_noise = float(self.dr.get("obs_noise", 0.0))
        # Faithful-parity: mirror the deploy's post-step joint clamp. URDF leg
        # velocity limits in LEGS_JOINTS order (hip_pitch/roll/yaw, knee,
        # ankle_pitch/roll x2 legs, then waist): hip/waist 32, knee 20, ankle 30.
        # Sourced LIVE from the prim URDF via the SoT (SPEC.leg_limits()[2]) so
        # the trainer clamp and the deploy clamp can never drift; the array is
        # PROVEN bit-identical to the previous hardcoded [32,32,32,20,30,30,...].
        self._joint_clamp = bool(self.dr.get("train_joint_clamp", False))
        _vel_lim = SPEC.leg_limits()[2]   # float32, LEGS_JOINTS order
        self.vel_lim_t = torch.tensor(
            _vel_lim, dtype=torch.float32, device=self.tdev)
        self._init_q_band = float(self.dr.get("init_q_band", 0.05))
        self._init_xy_band = float(self.dr.get("init_xy_band", 0.03))
        self._init_z_band = float(self.dr.get("init_z_band", 0.0))

        if self.hold_arms:
            self.arm_targets_t = torch.tensor(
                ARM_NOMINAL, dtype=torch.float32, device=self.tdev
            ).unsqueeze(0).expand(self.n, -1).contiguous()

        # Seed pose (NOMINAL squat at SPAWN_Z, identity upright), per-world qpos.
        self.seed_qpos = np.zeros(self.nq, dtype=np.float32)
        self.seed_qpos[0:3] = [0.0, 0.0, SPAWN_Z]
        self.seed_qpos[3:7] = [1.0, 0.0, 0.0, 0.0]   # (w,x,y,z)
        for i in range(NJ):
            self.seed_qpos[self.controller_to_qpos[i]] = self.nominal[i]
        if self.hold_arms:
            for i in range(len(ARM_JOINTS)):
                self.seed_qpos[self.arm_to_qpos[i]] = ARM_NOMINAL[i]
        self.seed_qpos_t = torch.tensor(self.seed_qpos, dtype=torch.float32,
                                        device=self.tdev)

    def _setup_foot_xpos(self):
        """Per-world foot world-position view from newton state_a.body_q.

        state_a.body_q is flat [body_count, 7] with per-world body blocks. We
        find the ankle_roll bodies' per-world-local indices, then build a
        gather index for all worlds so self.xpos_t[:, (lfoot,rfoot), :] works
        like the mjwarp env (which used mw_d.xpos)."""
        import mujoco
        mjm = self.solver.mj_model
        bodies_per_world = self.model.body_count // self.n

        def _bid(suffix):
            cand = []
            for b in range(mjm.nbody):
                bn = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_BODY, b)
                if bn and bn.endswith(suffix):
                    cand.append(b)
            if len(cand) != 1:
                raise RuntimeError(f"body '{suffix}' not unique: {cand}")
            return cand[0]
        # The mjw body index is per-world-local; newton body index = world*bpw + local.
        self._lfoot_local = _bid("left_ankle_roll_link")
        self._rfoot_local = _bid("right_ankle_roll_link")
        self._bodies_per_world = bodies_per_world
        # gather: (n, 2) newton body indices for [lfoot, rfoot] per world
        wofs = (np.arange(self.n) * bodies_per_world)[:, None]
        # mjw body idx and newton body idx can differ; use solver.mjc_body_to_newton_body
        m = getattr(self.solver, "mjc_body_to_newton_body", None)
        if m is not None:
            mb = m.numpy()  # (nworld, nbody_mjc) -> newton body
            self._foot_newton_idx = torch.tensor(
                np.stack([mb[:, self._lfoot_local], mb[:, self._rfoot_local]], axis=1),
                dtype=torch.long, device=self.tdev)
        else:
            self._foot_newton_idx = torch.tensor(
                wofs + np.array([self._lfoot_local, self._rfoot_local])[None, :],
                dtype=torch.long, device=self.tdev)
        self._body_q_t = self.wp.to_torch(self.state_a.body_q)  # (body_count, 7)
        self.bid_lfoot, self.bid_rfoot = 0, 1   # columns into the gathered view
        self.prev_foot_xy_t = torch.zeros(self.n, 2, 2, dtype=torch.float32,
                                          device=self.tdev)
        self._xpos_ready = True

    @property
    def xpos_t(self):
        """(n, 2, 3) world positions of [lfoot, rfoot], gathered from newton
        state_a.body_q. Only valid when _setup_foot_xpos ran. The parent's
        foot rewards index self.xpos_t[:, (bid_lfoot, bid_rfoot), :]; with
        bid_lfoot/bid_rfoot = 0/1 those select our two gathered columns."""
        pos = self._body_q_t[:, 0:3]                       # (body_count, 3)
        return pos[self._foot_newton_idx]                  # (n, 2, 3)

    # ── seed the underlying newton model + mjw_data to NOMINAL ──
    def _seed_model_nominal(self):
        n = self.n
        # write NOMINAL into the mjw_data views (all worlds), zero vel.
        self.qpos_t[:] = self.seed_qpos_t.unsqueeze(0).expand(n, -1)
        self.qvel_t[:] = 0.0
        # control targets -> NOMINAL for the actuated joints (all worlds).
        flat_nom = self.nominal_t.unsqueeze(0).expand(n, -1).reshape(-1)
        self.ctrl_pos_t.index_copy_(0, self.ctrl_write_idx, flat_nom)
        if self.hold_arms:
            flat_arm = self.arm_targets_t.reshape(-1)
            self.ctrl_pos_t.index_copy_(0, self.arm_write_idx, flat_arm)

    # ── CUDA-graph capture of the SUBSTEPS deploy loop ──
    def _try_capture_graph(self):
        if os.environ.get("OMNISIM_NEWTON_NO_GRAPH"):
            return
        wp, newton = self.wp, self.newton
        try:
            with wp.ScopedDevice(self.device):
                # warm up (kernel compile / autotune outside capture)
                self._substeps_direct()
                wp.synchronize()
                wp.capture_begin(force_module_load=False)
                self._substeps_direct()
                self._cuda_graph = wp.capture_end()
            print(f"[newton-env] CUDA graph captured ({SUBSTEPS} substeps)")
        except Exception as e:
            print(f"[newton-env] CUDA graph capture failed ({e}); direct step")
            self._cuda_graph = None

    def _substeps_direct(self):
        for _ in range(SUBSTEPS):
            self.state_a.clear_forces()
            self.model.collide(self.state_a, self.contacts)
            self.solver.step(self.state_a, self.state_b, self.control,
                             self.contacts, SUB_DT)
            self.state_a, self.state_b = self.state_b, self.state_a

    def _physics(self):
        with self.wp.ScopedDevice(self.device):
            if self._cuda_graph is not None:
                self.wp.capture_launch(self._cuda_graph)
            else:
                self._substeps_direct()

    # ── correctness gate: the seeded NOMINAL must read back through the map ──
    def _verify_index_map(self):
        self._seed_model_nominal()
        self.wp.synchronize()
        q = self.qpos_t.index_select(1, self.qpos_idx_t)          # (n,NJ)
        dev = q.abs().max(dim=0).values  # unused; per-joint
        nom = self.nominal_t.unsqueeze(0)
        err = (q - nom).abs().max().item()
        # proj_gravity must be ~(0,0,-1) at identity-upright spawn.
        qp = self.qpos_t
        w_ = qp[:, 3]; x = qp[:, 4]; y = qp[:, 5]; z = qp[:, 6]
        gz = -(1 - 2 * (x * x + y * y))
        gz_err = (gz + 1.0).abs().max().item()
        base_z = self.qpos_t[:, 2].mean().item()
        print(f"[verify] qpos_idx={self.controller_to_qpos.tolist()}")
        print(f"[verify] qvel_idx={self.controller_to_qvel.tolist()}")
        print(f"[verify] dof_per_world={self.dof_per_world} "
              f"ctrl_write_idx[:13]={self.ctrl_write_idx[:NJ].tolist()}")
        print(f"[verify] read-back NOMINAL max|q-NOM| = {err:.2e} (want ~0)")
        print(f"[verify] proj_gravity_z max|gz+1| = {gz_err:.2e} (want ~0)")
        print(f"[verify] base_z mean = {base_z:.3f} (want {SPAWN_Z})")
        nom_rb = self.qpos_t[0].index_select(0, self.qpos_idx_t).cpu().numpy()
        print(f"[verify] world0 read-back q = {np.round(nom_rb, 4).tolist()}")
        print(f"[verify] NOMINAL            = {np.round(self.nominal, 4).tolist()}")
        assert err < 1e-3, f"INDEX MAP WRONG: read-back NOMINAL err {err}"
        assert gz_err < 1e-3, f"proj_gravity wrong at spawn: {gz_err}"
        assert abs(base_z - SPAWN_Z) < 1e-2, f"base_z {base_z} != {SPAWN_Z}"
        print("[verify] INDEX MAP OK")

    # ── reset: write the seed pose (+jitter/gait/rest-start) into mjw_data ──
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
        if self.vx_cond:
            self.vx_cmd_t[idx] = self._sample_vx_cmd(m)

        if self._init_q_band > 0:
            jitter = (torch.rand(m, NJ, device=self.tdev) * 2 - 1) * self._init_q_band
            base_idx = idx.unsqueeze(1).expand(-1, NJ)
            col_idx = self.qpos_idx_t.unsqueeze(0).expand(m, -1)
            self.qpos_t[base_idx, col_idx] += jitter
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
            full = torch.full((m,), 1e6, device=self.tdev)
            legs0, _, _, _ = self._ghg.targets_torch(th, self.gp, full)
            legs1, _, _, _ = self._ghg.targets_torch(th + self.phase_dt, self.gp, full)
            qd_ref = (legs1 - legs0) / DT
            base_idx = idx.unsqueeze(1).expand(-1, NJ)
            self.qpos_t[base_idx, self.qpos_idx_t.unsqueeze(0).expand(m, -1)] = legs0
            self.qvel_t[base_idx, self.qvel_idx_t.unsqueeze(0).expand(m, -1)] = qd_ref
            self._ramp_t0[idx] = 1e6

        if self.rest_start_frac > 0:
            rest = torch.rand(m, device=self.tdev) < self.rest_start_frac
            ridx = idx[rest]
            if ridx.numel() > 0:
                rm = ridx.shape[0]
                self.qpos_t[ridx] = self.seed_qpos_t.unsqueeze(0).expand(rm, -1)
                self.qvel_t[ridx] = 0.0
                rj = (torch.rand(rm, NJ, device=self.tdev) * 2 - 1) * 0.03
                sag = torch.rand(rm, device=self.tdev)
                for kn_i, ak_i in ((MJ._L_KN, MJ._L_AP), (MJ._R_KN, MJ._R_AP)):
                    rj[:, kn_i] += 0.20 * sag
                    rj[:, ak_i] += -0.10 * sag
                ri = ridx.unsqueeze(1).expand(-1, NJ)
                rc = self.qpos_idx_t.unsqueeze(0).expand(rm, -1)
                self.qpos_t[ri, rc] += rj
                self.qpos_t[ridx, 3:7] = torch.tensor([1.0, 0.0, 0.0, 0.0],
                                                      device=self.tdev)
                self.phase_t[ridx] = (self._ghg.DS_PHASE
                                      if self.gait_model == "human" else 0.0)
                self._ramp_t0[ridx] = 0.0

        self.action_buffer_t[idx] = 0.0
        self.action_delay_t[idx] = 0
        if self.max_latency_ticks > 0:
            self.action_delay_t[idx] = torch.randint(
                0, self.max_latency_ticks + 1, (m,), dtype=torch.long, device=self.tdev)
        if self._act_gain > 0.0:
            self.act_gain_t[idx] = (
                1.0 + (torch.rand(m, device=self.tdev) * 2 - 1) * self._act_gain)

        # CRITICAL: SolverMuJoCo's source of truth is newton state_a.joint_q /
        # body_q -- with update_data_interval=1 (default) solver.step re-derives
        # mjw_data.qpos/qvel from state_a EVERY step (_update_mjc_data). So
        # writing mjw_data.qpos directly (above) is CLOBBERED by the next step
        # unless we back-propagate it into newton state. We use the solver's own
        # conversion kernel (_update_newton_state: mjw_data -> state.joint_q /
        # body_q), which also handles the quaternion-convention difference
        # (mjw qpos is w,x,y,z; newton body_q is x,y,z,w). Without this, a
        # non-first reset leaves state_a holding the STALE (e.g. fallen) pose and
        # the next step explodes (verified: 2nd reset -> roll=pi on step 0).
        self.solver._update_newton_state(
            self.model, self.state_a, self.solver.mjw_data, state_prev=self.state_a)
        if self._xpos_ready:
            feet = self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), 0:2]
            self.prev_foot_xy_t[idx] = feet[idx]

    # ── step: same control/reward as the parent, Newton physics in between ──
    def step(self, action_t):
        action_t = torch.clamp(action_t, -1.0, 1.0)
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
            self.jl_lo_t, self.jl_hi_t)                            # (n, NJ)
        # write the 13 actuated targets of all worlds into control (flat DOF).
        self.ctrl_pos_t.index_copy_(0, self.ctrl_write_idx, targets.reshape(-1))
        if self.hold_arms:
            if self.gait_model == "human":
                arms = self._model_arms
            elif self.gait_a_arm != 0.0:
                arms = self.arm_targets_t.clone()
                sw = self.gait_a_arm * torch.sin(self.phase_t)
                arms[:, 0] = arms[:, 0] + sw
                arms[:, 5] = arms[:, 5] - sw
            else:
                arms = self.arm_targets_t
            self.ctrl_pos_t.index_copy_(0, self.arm_write_idx, arms.reshape(-1))

        # DR pushes (velocity impulse on the base lin vel).
        if self._push_p > 0 and self._push_vmax > 0:
            hit = torch.rand(self.n, device=self.tdev) < self._push_p
            if hit.any():
                theta = torch.rand(self.n, device=self.tdev) * (2 * math.pi)
                mag = torch.rand(self.n, device=self.tdev) * self._push_vmax
                self.qvel_t[hit, 0] = self.qvel_t[hit, 0] + (torch.cos(theta) * mag)[hit]
                self.qvel_t[hit, 1] = self.qvel_t[hit, 1] + (torch.sin(theta) * mag)[hit]

        # PHYSICS: the EXACT deploy substep loop through SolverMuJoCo.
        self._physics()

        # FAITHFUL PARITY: mirror the deploy's post-step joint clamp
        # (OmNewtonBackend.cpp:1940-2036), which is LOAD-BEARING in deploy
        # (disabling it made the walk WORSE). Clamp achieved leg qpos to the
        # URDF limits, clamp qvel to +/-velocity_limit, and ZERO the qvel
        # component driving into a position stop -- then back-propagate into
        # newton state_a (else update_data_interval=1 re-derives mjw_data from
        # the unclamped state next step, same pattern as _reset line ~660).
        if self._joint_clamp:
            qp = self.qpos_t.index_select(1, self.qpos_idx_t)        # (n,NJ)
            qv = self.qvel_t.index_select(1, self.qvel_idx_t)
            qv = torch.clamp(qv, -self.vel_lim_t, self.vel_lim_t)
            lo_hit = qp <= self.jl_lo_t
            hi_hit = qp >= self.jl_hi_t
            qp = torch.clamp(qp, self.jl_lo_t, self.jl_hi_t)
            zero = torch.zeros_like(qv)
            qv = torch.where(lo_hit & (qv < 0), zero, qv)
            qv = torch.where(hi_hit & (qv > 0), zero, qv)
            self.qpos_t.index_copy_(1, self.qpos_idx_t, qp)
            self.qvel_t.index_copy_(1, self.qvel_idx_t, qv)
            self.solver._update_newton_state(
                self.model, self.state_a, self.solver.mjw_data,
                state_prev=self.state_a)

        self.ep_step_t = self.ep_step_t + 1
        prev_action_t = self.last_action_t
        self.last_action_t = action_t
        th_used = self.phase_t

        # adaptive phase gate (same as parent)
        kt = self.r.get("phase_gate_tilt", 0.0)
        kr = self.r.get("phase_gate_rate", 0.0)
        if kt != 0.0 or kr != 0.0:
            tilt2 = self.prev_roll_t ** 2 + self.prev_pitch_t ** 2
            rate2 = self.prev_roll_rate_t ** 2 + self.prev_pitch_rate_t ** 2
            gate = torch.clamp(1.0 - kt * tilt2 - kr * rate2,
                               self.r.get("phase_gate_floor", 0.2), 1.0)
            self._phase_gate = gate
        else:
            gate = 1.0
        if self.vx_cond and self.vx_phase_freeze > 0.0:
            g_vx = torch.clamp(self.vx_cmd_t / self.vx_phase_freeze, 0.0, 1.0)
            gate = gate * g_vx
        self.phase_t = torch.remainder(self.phase_t + self.phase_dt * gate,
                                       2.0 * math.pi)
        if self.vx_cond and not getattr(self, "_vx_freeze_resample", False):
            chg = torch.rand(self.n, device=self.tdev) < (DT / 2.5)
            if chg.any():
                nc = self._sample_vx_cmd(self.n)
                self.vx_cmd_t = torch.where(chg, nc, self.vx_cmd_t)

        # ── reward (verbatim from the parent's step body) ──
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
        _vtgt = self.vx_cmd_t if self.vx_cond else self.vx_target
        vsig = r.get("vel_sigma", 0.10)
        if self.vx_cond:
            vsig_stand = r.get("vel_sigma_stand", vsig)
            _f = torch.clamp(self.vx_cmd_t / max(self.vx_target, 1e-3), 0.0, 1.0)
            vsig = vsig_stand + (vsig - vsig_stand) * _f
        r_vel = r.get("vel", 2.0) * torch.exp(-((vx - _vtgt) ** 2) / vsig)
        r_vel = r_vel + r.get("vel_l1", 0.0) * torch.abs(vx - _vtgt)
        r_lat = r.get("lat", -0.5) * torch.abs(vy)
        r_yaw = r.get("yaw", -0.5) * torch.abs(wz)
        z_ref = r.get("z_ref", 0.74)
        r_height = r.get("height", -10.0) * (bz - z_ref) ** 2
        r_act = r.get("act", -0.005) * (action_t * action_t).sum(dim=1)
        r_rate = r.get("act_rate", -0.01) * ((action_t - prev_action_t) ** 2).sum(dim=1)
        reward = (r_alive + r_up + r_vel + r_lat + r_yaw + r_height + r_act + r_rate)

        # imitation / shape / com / torso / frontal-track rewards reuse the
        # parent helpers via _model_legs etc set in _baseline_targets_t. We call
        # the parent's reward-tail by delegating to a shared method to avoid
        # duplicating ~120 lines; the foot-aware terms need xpos (gated above).
        reward = self._reward_tail(reward, action_t, th_used, roll, pitch, r)

        fall = (torch.abs(roll) > MJ.ROLL_FAIL) | (torch.abs(pitch) > MJ.PITCH_FAIL) | (bz < MJ.BZ_FAIL)
        if getattr(self, "_et_mask", None) is not None:
            _grace = int(r.get("track_et_grace", 3.0) / DT)
            fall = fall | (self._et_mask & (self.ep_step_t > _grace))
        done = fall | (self.ep_step_t >= self.max_ep)
        reward = reward + fall.float() * r.get("term", -1.0)

        if done.any():
            self._reset_envs(env_mask=done)

        obs = self._obs_full(reset_mask=done)
        info = {"bz": bz, "roll": roll, "pitch": pitch}
        return obs, reward, done, info

    def _reward_tail(self, reward, action_t, th_used, roll, pitch, r):
        """The style/imitation/foot reward terms (copied from the parent's
        step). Kept here so the Newton step can run them on the same _model_legs
        / _swingL etc the parent's _baseline_targets_t produced. Foot-aware
        terms require self._xpos_ready."""
        rw_sw = r.get("rw_swing_track", 0.0)
        if rw_sw != 0.0 and self.gait_model == "human" \
                and getattr(self, "_model_legs", None) is not None:
            q_act_sw = self.qpos_t.index_select(1, self.qpos_idx_t)
            dm = q_act_sw - self._model_legs
            sig_sw = r.get("track_sigma", 0.04)
            errL = dm[:, MJ._L_HP] ** 2 + dm[:, MJ._L_KN] ** 2
            errR = dm[:, MJ._R_HP] ** 2 + dm[:, MJ._R_KN] ** 2
            w_ank = r.get("swing_track_ankle", 0.0)
            if w_ank != 0.0:
                errL = errL + w_ank * dm[:, MJ._L_AP] ** 2
                errR = errR + w_ank * dm[:, MJ._R_AP] ** 2
            reward = reward + rw_sw * (self._swingL * torch.exp(-errL / sig_sw)
                                       + self._swingR * torch.exp(-errR / sig_sw))

        rw_track = r.get("rw_track", 0.0)
        rw_tv = r.get("rw_track_vel", 0.0)
        track_et = r.get("track_et", 0.0)
        self._et_mask = None
        if (rw_track != 0.0 or rw_tv != 0.0 or track_et > 0.0) \
                and getattr(self, "_model_legs", None) is not None:
            q_act = self.qpos_t.index_select(1, self.qpos_idx_t)
            err = (q_act - self._model_legs) ** 2
            w_trk = self._track_w
            track_err = (err * w_trk).sum(dim=1) / w_trk.sum()
            if rw_track != 0.0:
                reward = reward + rw_track * torch.exp(-track_err / r.get("track_sigma", 0.05))
            if rw_tv != 0.0 and getattr(self, "_model_legs_qd", None) is not None:
                qd_act = self.qvel_t.index_select(1, self.qvel_idx_t)
                errv = ((qd_act - self._model_legs_qd) ** 2 * w_trk).sum(dim=1) / w_trk.sum()
                reward = reward + rw_tv * torch.exp(-errv / r.get("track_vel_sigma", 4.0))
            if track_et > 0.0:
                dev = (q_act - self._model_legs)[:, self._hipkn_idx].abs().mean(dim=1)
                self._et_mask = dev > track_et

        rw_torso = r.get("rw_torso", 0.0)
        if rw_torso != 0.0:
            reward = reward + rw_torso * (self.prev_roll_rate_t ** 2
                                          + self.prev_pitch_rate_t ** 2)

        # ── COM-OVER-STANCE-FOOT reward (ported from the parent's step,
        # gpu_mjwarp_g1_walk_trainer.py ~1340-1348). During single support the
        # pelvis lateral position (COM proxy) should sit over the PLANTED foot:
        # when the left leg swings (weight=_swingL) the COM should be over the
        # RIGHT foot, and vice versa. A soft capture-point/ZMP objective that
        # narrows the stance NATURALLY (a supported COM needs no wide base)
        # instead of forcing a static hip-roll splay. Needs xpos (gated in
        # __init__ via the rw_com!=0 branch that calls _setup_foot_xpos).
        _dbg_com = _dbg_front = None
        rw_com = r.get("rw_com", 0.0)
        if rw_com != 0.0 and self._xpos_ready \
                and getattr(self, "_model_legs", None) is not None:
            py = self.qpos_t[:, 1]                                  # pelvis y
            fy = self.xpos_t[:, (self.bid_lfoot, self.bid_rfoot), 1]
            sig_c = r.get("com_sigma", 0.004)
            dR = (py - fy[:, 1]) ** 2          # COM-to-RIGHT-foot (left swings)
            dL = (py - fy[:, 0]) ** 2          # COM-to-LEFT-foot  (right swings)
            _r_com = rw_com * (self._swingL * torch.exp(-dR / sig_c)
                               + self._swingR * torch.exp(-dL / sig_c))
            reward = reward + _r_com
            _dbg_com = _r_com

        rw_front = r.get("rw_frontal_track", 0.0)
        if rw_front != 0.0 and getattr(self, "_model_legs", None) is not None:
            qf = self.qpos_t.index_select(1, self.qpos_idx_t)
            dmf = qf - self._model_legs
            sig_f = r.get("frontal_sigma", 0.01)
            errL = dmf[:, MJ._L_HR] ** 2 + dmf[:, MJ._L_HY] ** 2
            errR = dmf[:, MJ._R_HR] ** 2 + dmf[:, MJ._R_HY] ** 2
            _r_front = rw_front * (torch.exp(-errL / sig_f)
                                   + torch.exp(-errR / sig_f))
            reward = reward + _r_front
            _dbg_front = _r_front

        # One-time proof the wired terms are ACTIVE (set OMNISIM_RW_DEBUG=1).
        if os.environ.get("OMNISIM_RW_DEBUG") and not getattr(
                self, "_rw_dbg_done", False):
            def _m(t):
                return "off" if t is None else f"mean={t.mean().item():+.5f}"
            print(f"[rw-debug] rw_com={rw_com} -> {_m(_dbg_com)} | "
                  f"rw_frontal_track={rw_front} -> {_m(_dbg_front)}",
                  flush=True)
            self._rw_dbg_done = True

        # foot-aware shaping (needs xpos)
        if (self.rw_sched != 0.0 or self.rw_slip != 0.0) and self._xpos_ready:
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
                reward = reward + self.rw_sched * (stance * z_hi + swing * z_lo).sum(dim=1)
            if self.rw_slip != 0.0:
                v_xy = (foot_xy - self.prev_foot_xy_t) / DT
                contact = (foot_z < self.foot_z_contact).float()
                reward = reward + self.rw_slip * (contact * torch.linalg.norm(v_xy, dim=2)).sum(dim=1)
            self.prev_foot_xy_t = foot_xy.clone()
        return reward

    def reset(self):
        self._reset_all()
        self._obs_buf = None
        return self._obs_full()


# ──────────────────────────────────────────────────────────────────────────
# GHOST-SIMILARITY EVAL  --  "how X% similar is the deployed gait to the ghost?"
# ──────────────────────────────────────────────────────────────────────────
# Additive, training-untouched eval. Loads a policy, runs the SAME parity env the
# deploy uses (DR off), and for every UPRIGHT env over a STEADY window (post
# ramp-in) accumulates, per controlled leg joint j:
#   a_j(t) = achieved leg q   = qpos_t[:, qpos_idx_t]   (what the body did)
#   g_j(t) = PURE ghost q      = env._model_legs         (the kinematic target,
#            the EXACT same tensor the imitation reward pulls toward -- NOT
#            re-derived here)
# It then computes the three metrics defined in the task: amplitude similarity
# (headline), shape similarity (Pearson r), and a per-joint breakdown.
#
# Pairing/timing: in step(), _baseline_targets_t() sets self._model_legs for the
# current phase, THEN physics runs, THEN phase advances. So immediately after
# env.step() returns, _model_legs is the ghost target the just-applied action was
# driving toward and qpos_t is the resulting achieved pose -- a correctly paired
# (ghost target, achieved) sample at the same phase.
# ──────────────────────────────────────────────────────────────────────────

# hips+knees+waist = the imitation-tracked subset (ankles freed via
# --track-ankle-w 0.0). Indices into LEGS_JOINTS / the 13-wide leg vector.
_ANKLE_IDX = (MJ._L_AP, MJ._L_AR, MJ._R_AP, MJ._R_AR)
_HIPKNWAIST_IDX = tuple(j for j in range(NJ) if j not in _ANKLE_IDX)

# SAGITTAL gait subset = the joints the ghost actually SWINGS (hip_pitch, knee,
# ankle_pitch on both legs). With the default gait (lateral="sway", yaw="none")
# these are the only joints with a real trajectory; the lateral/transverse joints
# (hip_yaw, hip_roll, ankle_roll, waist) sit ~static. This subset is the honest
# answer to "does the leg gait LOOK like the ghost?".
_SAGITTAL_IDX = (MJ._L_HP, MJ._L_KN, MJ._L_AP, MJ._R_HP, MJ._R_KN, MJ._R_AP)

# FAIR amplitude-normalisation floor (rad). 1 - RMSE/PTP is meaningless when the
# ghost barely moves (PTP -> 0 makes any tiny tremor read as 0% similar). For
# joints whose ghost PTP is below PTP_FLOOR we normalise by PTP_FLOOR instead, so
# the score answers "how much does the body deviate, relative to a typical joint
# excursion" rather than "relative to a near-zero reference". 0.35 rad ~ a normal
# walking hip/knee swing. Joints normalised this way are flagged STATIC.
PTP_FLOOR = 0.35


def _ghost_similarity_selftest(env, tol=1e-3):
    """Sanity gate: feed the EXACT ghost pose as both achieved and ghost and
    confirm the amplitude metric returns 100% (RMSE=0 -> sim_amp=1) and the
    fair metric agrees. Catches any pairing / normalisation bug before we trust
    a real number. Uses a synthetic 2-cycle ghost sweep so PTP is non-degenerate.
    Returns (ok, detail)."""
    import numpy as _np
    NS = 64
    # Synthetic per-joint sweeps with a RANGE of amplitudes incl. tiny ones, so
    # the static-floor path is exercised too.
    amps = _np.linspace(0.0, 0.8, NJ)
    t = _np.linspace(0, 4 * _np.pi, NS)
    g = _np.stack([0.1 + a * _np.sin(t) for a in amps], axis=1)   # (NS, NJ)
    a = g.copy()                                                  # achieved == ghost
    d = (a - g) ** 2
    rmse = _np.sqrt(d.mean(axis=0))                               # all zero
    ptp = _np.clip(g.max(0) - g.min(0), 1e-4, None)
    sim_orig = _np.clip(1.0 - rmse / ptp, 0.0, None)
    ptp_fair = _np.maximum(ptp, PTP_FLOOR)
    sim_fair = _np.clip(1.0 - rmse / ptp_fair, 0.0, None)
    ok = bool(_np.all(sim_orig > 1.0 - tol) and _np.all(sim_fair > 1.0 - tol))
    detail = (f"identical-input self-test: min sim_orig={sim_orig.min():.4f} "
              f"min sim_fair={sim_fair.min():.4f} (expect ~1.0)")
    return ok, detail


def eval_ghost_similarity(env, ac, eval_steps, ramp_s, window_s=0.0,
                          min_window_frac=0.6, seed=0, diag_traces=True):
    """Run the policy in the (DR-off) parity env and return the ghost-similarity
    metrics. Returns a dict; the caller prints. Deterministic (greedy mean
    action, fixed seed).

    Steady window: [ramp_steps, win_end). ramp_s of ramp-in is skipped. win_end
    is ramp_steps + window_s/DT (or eval_steps if window_s<=0). An env's samples
    are accumulated ONLY while it is upright (has not fallen yet this episode) --
    once it falls it is reset to a fresh ramp-in pose, so post-fall samples would
    pollute the steady stats and are excluded. An env is KEPT for the final
    averages iff it stayed upright for >= min_window_frac of the window (so a
    short-lived env that toppled early near the start does not skew the mean)."""
    torch.manual_seed(seed)
    np.random.seed(seed)
    tdev = env.tdev
    n = env.n
    ac.eval()

    # SANITY GATE: the metric must return ~100% when achieved == ghost. Run it
    # before trusting any real number -- a pairing/normalisation bug would show up
    # here as <100% on identical inputs.
    st_ok, st_detail = _ghost_similarity_selftest(env)
    print(f"[ghost-eval] {st_detail}  -> {'PASS' if st_ok else 'FAIL'}")
    if not st_ok:
        raise RuntimeError(
            "ghost-similarity metric self-test FAILED (identical achieved==ghost "
            "did not score ~100%); the metric is buggy -- aborting.")

    ramp_steps = int(round(ramp_s / DT))     # steady window starts here
    if window_s and window_s > 0.0:
        win_end = min(eval_steps, ramp_steps + int(round(window_s / DT)))
    else:
        win_end = eval_steps
    win_len = max(1, win_end - ramp_steps)

    obs = env.reset()

    # Per-env accumulators over the steady window, but ONLY summed for env-steps
    # where the env is upright (alive). Sufficient statistics (no per-step
    # storage): sum a, sum g, sum a*a, sum g*g, sum a*g, count -- per (env,joint).
    sum_a = torch.zeros(n, NJ, device=tdev, dtype=torch.float64)
    sum_g = torch.zeros(n, NJ, device=tdev, dtype=torch.float64)
    sum_aa = torch.zeros(n, NJ, device=tdev, dtype=torch.float64)
    sum_gg = torch.zeros(n, NJ, device=tdev, dtype=torch.float64)
    sum_ag = torch.zeros(n, NJ, device=tdev, dtype=torch.float64)
    sum_dd = torch.zeros(n, NJ, device=tdev, dtype=torch.float64)   # (a-g)^2
    cnt = torch.zeros(n, device=tdev, dtype=torch.float64)
    g_min = torch.full((n, NJ), float("inf"), device=tdev, dtype=torch.float64)
    g_max = torch.full((n, NJ), float("-inf"), device=tdev, dtype=torch.float64)
    # an env contributes only while it has NEVER fallen (a fall resets it, which
    # would mix a fresh ramp-in episode into the steady stats).
    fell = torch.zeros(n, dtype=torch.bool, device=tdev)

    # DIAGNOSTIC: capture the achieved-vs-ghost time series for env 0 over the
    # steady window so we can SEE whether a deviation is a real wobbly gait or an
    # index/sign/offset artifact. Stored as lists of (NJ,) numpy rows.
    trace_a, trace_g = [], []

    for step in range(eval_steps):
        with torch.no_grad():
            mu = ac.pi(obs)
        obs, _, done, _ = env.step(mu)
        # paired samples at the phase the action targeted:
        a = env.qpos_t.index_select(1, env.qpos_idx_t).to(torch.float64)   # (n,NJ)
        g = env._model_legs.to(torch.float64)                              # (n,NJ)
        in_window = (step >= ramp_steps) and (step < win_end)
        if diag_traces and in_window and (not bool(fell[0].item())):
            trace_a.append(a[0].cpu().numpy())
            trace_g.append(g[0].cpu().numpy())
        # mask: in steady window AND env has not fallen up to now.
        alive_now = (~fell)
        if in_window:
            m = alive_now.to(torch.float64).unsqueeze(1)                   # (n,1)
            sum_a += a * m
            sum_g += g * m
            sum_aa += (a * a) * m
            sum_gg += (g * g) * m
            sum_ag += (a * g) * m
            sum_dd += ((a - g) ** 2) * m
            cnt += alive_now.to(torch.float64)
            # ptp over the window, only counting alive samples
            g_in = torch.where(alive_now.unsqueeze(1), g,
                               torch.full_like(g, float("inf")))
            g_min = torch.minimum(g_min, g_in)
            g_ax = torch.where(alive_now.unsqueeze(1), g,
                               torch.full_like(g, float("-inf")))
            g_max = torch.maximum(g_max, g_ax)
        # mark falls (env was reset inside step -> exclude it from here on)
        fell = fell | done

    # keep envs upright for >= min_window_frac of the steady window.
    min_cnt = max(2.0, min_window_frac * win_len)
    good = cnt >= min_cnt
    n_good = int(good.sum().item())
    if n_good == 0:
        raise RuntimeError(
            f"no env stayed upright for >={min_window_frac:.0%} of the steady "
            f"window ({win_len} steps) -- cannot measure gait similarity. "
            f"max upright-window steps over envs was {int(cnt.max().item())}. "
            f"Try a shorter --eval-window-s or check the policy.")

    gi = torch.nonzero(good, as_tuple=False).squeeze(-1)
    cN = cnt[gi].unsqueeze(1)                                  # (G,1)
    sa = sum_a[gi]; sg = sum_g[gi]; saa = sum_aa[gi]
    sgg = sum_gg[gi]; sag = sum_ag[gi]; sdd = sum_dd[gi]
    gmn = g_min[gi]; gmx = g_max[gi]

    # Per (env, joint) statistics.
    rmse = torch.sqrt(torch.clamp(sdd / cN, min=0.0))                 # (G,NJ)
    ptp = torch.clamp(gmx - gmn, min=1e-4)                            # floor eps
    # ORIGINAL amplitude similarity: 1 - RMSE/PTP, PTP = the GHOST's own range.
    # Fair only where the ghost actually moves; blows up to 0 on near-static
    # joints (PTP -> 0). Kept for back-compat / full disclosure.
    sim_amp = torch.clamp(1.0 - rmse / ptp, min=0.0)                  # (G,NJ)
    # FAIR amplitude similarity: normalise by max(PTP, PTP_FLOOR). For a joint
    # the ghost SWINGS (PTP >= PTP_FLOOR) this is identical to the original. For a
    # near-static ghost joint it instead asks "how big is the deviation relative
    # to a normal joint excursion (PTP_FLOOR rad)" -- so a small tremor no longer
    # reads as 0% similar. A joint is STATIC iff its (mean) ghost PTP < PTP_FLOOR.
    ptp_fair = torch.clamp(ptp, min=PTP_FLOOR)
    sim_amp_fair = torch.clamp(1.0 - rmse / ptp_fair, min=0.0)        # (G,NJ)

    # Pearson r per (env, joint).
    mean_a = sa / cN; mean_g = sg / cN
    cov = sag / cN - mean_a * mean_g
    var_a = torch.clamp(saa / cN - mean_a * mean_a, min=0.0)
    var_g = torch.clamp(sgg / cN - mean_g * mean_g, min=0.0)
    denom = torch.sqrt(var_a * var_g)
    r = torch.where(denom > 1e-9, cov / denom, torch.zeros_like(cov))
    r = torch.clamp(r, -1.0, 1.0)
    r_pos = torch.clamp(r, min=0.0)

    # Average across the upright envs -> per-joint vectors.
    sim_amp_j = sim_amp.mean(dim=0).cpu().numpy()        # (NJ,) ORIGINAL
    sim_amp_fair_j = sim_amp_fair.mean(dim=0).cpu().numpy()   # (NJ,) FAIR
    r_j = r.mean(dim=0).cpu().numpy()                    # signed, for the table
    r_pos_j = r_pos.mean(dim=0).cpu().numpy()            # (NJ,) for shape%
    rmse_j = rmse.mean(dim=0).cpu().numpy()
    ptp_j = ptp.mean(dim=0).cpu().numpy()

    # STATIC mask: ghost barely moves this joint over the window -> the original
    # metric is unfair there; report these joints separately.
    static_j = ptp_j < PTP_FLOOR                         # (NJ,) bool

    hk = list(_HIPKNWAIST_IDX)
    sag = list(_SAGITTAL_IDX)
    moving = [j for j in range(NJ) if not static_j[j]]   # the joints the ghost swings
    # ORIGINAL headlines (PTP self-normalised; static joints drag it to ~0).
    headline_all = 100.0 * float(np.mean(sim_amp_j))
    headline_hk = 100.0 * float(np.mean(sim_amp_j[hk]))
    # FAIR headlines (PTP_FLOOR-normalised on static joints).
    headline_all_fair = 100.0 * float(np.mean(sim_amp_fair_j))
    headline_hk_fair = 100.0 * float(np.mean(sim_amp_fair_j[hk]))
    headline_sag_fair = 100.0 * float(np.mean(sim_amp_fair_j[sag]))
    # MOVING-only headline (the honest "does the gait look like the ghost where
    # the ghost actually moves"): excludes static joints entirely.
    headline_moving_fair = (100.0 * float(np.mean(sim_amp_fair_j[moving]))
                            if moving else float("nan"))
    shape_all = 100.0 * float(np.mean(r_pos_j))
    shape_hk = 100.0 * float(np.mean(r_pos_j[hk]))
    shape_sag = 100.0 * float(np.mean(r_pos_j[sag]))

    return dict(
        n_good=n_good, n_total=n, mean_window_steps=float(cnt[gi].mean().item()),
        ramp_steps=ramp_steps, win_len=win_len,
        headline_all=headline_all, headline_hk=headline_hk,
        headline_all_fair=headline_all_fair, headline_hk_fair=headline_hk_fair,
        headline_sag_fair=headline_sag_fair,
        headline_moving_fair=headline_moving_fair,
        shape_all=shape_all, shape_hk=shape_hk, shape_sag=shape_sag,
        sim_amp_j=sim_amp_j, sim_amp_fair_j=sim_amp_fair_j,
        r_j=r_j, r_pos_j=r_pos_j, static_j=static_j,
        rmse_j=rmse_j, ptp_j=ptp_j, hk_idx=hk, sag_idx=sag,
        selftest_ok=st_ok, selftest_detail=st_detail,
        trace_a=(np.stack(trace_a) if trace_a else None),
        trace_g=(np.stack(trace_g) if trace_g else None),
        ptp_floor=PTP_FLOOR)


def _print_ghost_similarity(res, policy_path):
    """Pretty-print the metrics (ORIGINAL + FAIR), the per-joint breakdown, and
    the achieved-vs-ghost diagnostic traces for hip_yaw / hip_roll / hip_pitch."""
    hk = set(res["hk_idx"])
    sag = set(res["sag_idx"])
    static_j = res["static_j"]
    floor = res.get("ptp_floor", PTP_FLOOR)
    print("")
    print("=" * 78)
    print(f"GHOST-SIMILARITY EVAL  policy={policy_path}")
    print(f"  self-test: {res.get('selftest_detail','(n/a)')}  -> "
          f"{'PASS' if res.get('selftest_ok') else 'FAIL'}")
    print(f"  steady window: {res['win_len']} steps ({res['win_len']*DT:.2f}s) "
          f"after {res['ramp_steps']} ramp steps ({res['ramp_steps']*DT:.2f}s)")
    print(f"  upright envs used: {res['n_good']}/{res['n_total']}  "
          f"mean steady-window steps/env: {res['mean_window_steps']:.0f}")
    print("-" * 78)
    print("  AMPLITUDE SIMILARITY (1 - RMSE/PTP), %:")
    print(f"    {'subset':<26} {'ORIGINAL':>10} {'FAIR':>10}    (FAIR floors PTP "
          f"at {floor:.2f} rad)")
    print(f"    {'all 13 legs':<26} {res['headline_all']:9.2f}% "
          f"{res['headline_all_fair']:9.2f}%")
    print(f"    {'hips+knees+waist (9)':<26} {res['headline_hk']:9.2f}% "
          f"{res['headline_hk_fair']:9.2f}%")
    print(f"    {'sagittal hp/kn/ap (6)':<26} {'--':>9}  "
          f"{res['headline_sag_fair']:9.2f}%   <- the swinging gait")
    print(f"    {'moving joints only':<26} {'--':>9}  "
          f"{res['headline_moving_fair']:9.2f}%   <- RECOMMENDED headline")
    print("  SHAPE (mean max(0, Pearson r), %):")
    print(f"    all 13 legs ............ {res['shape_all']:6.2f} %")
    print(f"    hips+knees+waist (9) ... {res['shape_hk']:6.2f} %")
    print(f"    sagittal hp/kn/ap (6) .. {res['shape_sag']:6.2f} %")
    print("-" * 78)
    print("  PER-JOINT BREAKDOWN  (STATIC = ghost PTP < floor -> FAIR uses floor)")
    print(f"    {'idx':>3} {'joint':<26} {'set':<6} {'sim_orig':>8} "
          f"{'sim_fair':>8} {'r':>7} {'rmse':>7} {'ptp':>7}")
    for j, name in enumerate(LEGS_JOINTS):
        tag = ("STATIC" if static_j[j]
               else ("sagit" if j in sag else ("HKW" if j in hk else "ankle")))
        print(f"    {j:>3} {name:<26} {tag:<6} "
              f"{res['sim_amp_j'][j]:8.3f} {res['sim_amp_fair_j'][j]:8.3f} "
              f"{res['r_j'][j]:7.3f} {res['rmse_j'][j]:7.4f} "
              f"{res['ptp_j'][j]:7.4f}")
    # Diagnostic time series: achieved vs ghost for hip_yaw / hip_roll / hip_pitch
    # (both legs). Decides real-deviation vs index/sign/offset artifact.
    ta, tg = res.get("trace_a"), res.get("trace_g")
    if ta is not None and len(ta) > 0:
        print("-" * 78)
        print(f"  ACHIEVED vs GHOST time series (env 0, first ~{min(len(ta),40)} "
              f"steps of steady window; rad)")
        probe = [("L_hip_yaw", MJ._L_HY), ("R_hip_yaw", MJ._R_HY),
                 ("L_hip_roll", MJ._L_HR), ("R_hip_roll", MJ._R_HR),
                 ("L_hip_pitch", MJ._L_HP), ("R_hip_pitch", MJ._R_HP)]
        for lab, idx in probe:
            ach = ta[:, idx]; gho = tg[:, idx]
            off = float(np.mean(ach - gho))
            print(f"    {lab:<12} idx={idx:<2}  "
                  f"ach[min/mean/max]={ach.min():+.3f}/{ach.mean():+.3f}/{ach.max():+.3f}  "
                  f"gho[min/mean/max]={gho.min():+.3f}/{gho.mean():+.3f}/{gho.max():+.3f}  "
                  f"mean(a-g)={off:+.3f}")
        # A compact sparkline-ish dump of the first few steps for the two hips that
        # the task flagged (yaw, roll) so a human can eyeball the pairing.
        ncol = min(len(ta), 12)
        for lab, idx in [("L_hip_yaw", MJ._L_HY), ("L_hip_roll", MJ._L_HR),
                         ("L_hip_pitch", MJ._L_HP)]:
            ach = " ".join(f"{ta[k,idx]:+.2f}" for k in range(ncol))
            gho = " ".join(f"{tg[k,idx]:+.2f}" for k in range(ncol))
            print(f"    {lab} ach: {ach}")
            print(f"    {lab} gho: {gho}")
    print("=" * 78)
    print("")


# ──────────────────────────────────────────────────────────────────────────
# Build the (B) "achieved" feasible ghost FROM A NEWTON WALKER, in the deploy
# solver -- so the reference is Newton-feasible (build_achieved_gait.py extracts
# in mjwarp, which the Newton deploy can't reproduce). Roll the greedy policy,
# phase-bin achieved leg+waist q, L/R-symmetrise (half-cycle mirror) + circular
# smooth, save (_W_N,13)+stand. Same math as build_achieved_gait.py.
# ──────────────────────────────────────────────────────────────────────────
def _build_achieved_gait(env, ac, out_path, steps=2000, ramp_skip=200) -> None:
    from projects.policies.control.gait import g1_human_gait as ghg
    W_N = ghg._W_N
    tdev = env.tdev
    ac.eval()
    obs = env.reset()
    qsum = torch.zeros(W_N, NJ, device=tdev)
    qcnt = torch.zeros(W_N, device=tdev)
    first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
    for k in range(steps):
        with torch.no_grad():
            mu = ac.pi(obs)
        obs, _, done, _ = env.step(mu)
        alive = (first_fall == 0)
        if k > ramp_skip:
            q = env.qpos_t.index_select(1, env.qpos_idx_t)
            phi = torch.remainder(env.phase_t / (2.0 * math.pi), 1.0)
            b = torch.clamp((phi * W_N).long(), 0, W_N - 1)
            qsum.index_add_(0, b[alive], q[alive])
            qcnt.index_add_(0, b[alive], torch.ones(int(alive.sum().item()), device=tdev))
        newly = done & (first_fall == 0)
        first_fall = torch.where(newly, torch.full_like(first_fall, k + 1), first_fall)
    qmean = (qsum / torch.clamp(qcnt, min=1.0).unsqueeze(1)).cpu().numpy()
    nvis = int((qcnt.cpu().numpy() > 0).sum())
    print(f"[build-achieved] phase bins visited {nvis}/{W_N}")

    def csmooth(tab, win=25):
        w = np.hanning(win); w /= w.sum()
        ext = np.concatenate([tab[-(win // 2):], tab, tab[:win // 2]], axis=0)
        return np.stack([np.convolve(ext[:, j], w, mode="same")[win // 2:-(win // 2)]
                         for j in range(tab.shape[1])], axis=1)

    qs = csmooth(qmean)
    half = W_N // 2
    msign = np.array([1, -1, -1, 1, 1, -1], dtype=np.float64)   # HP,HR,HY,KN,AP,AR
    left = qs[:, 0:6].copy()
    right = qs[:, 6:12].copy()
    left_sym = 0.5 * (left + np.roll(right, -half, axis=0) * msign)
    sym = np.zeros((W_N, NJ))
    sym[:, 0:6] = left_sym
    sym[:, 6:12] = np.roll(left_sym, half, axis=0) * msign
    sym[:, 12] = 0.0
    ds_i = int(round((ghg.DS_PHASE / (2.0 * math.pi)) % 1.0 * W_N)) % W_N
    ds_row = sym[ds_i]
    stand = np.zeros(NJ)
    stand[0:6] = ds_row[0:6]
    stand[6:12] = ds_row[0:6] * msign
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    np.savez(out_path, q=sym.astype(np.float32), stand=stand.astype(np.float32))
    surv = first_fall.cpu().numpy().astype(float)
    surv[surv == 0] = steps
    amp = np.degrees(np.abs(sym[:, 0:6] - sym[:, 0:6].mean(0)).max(0))
    print(f"[build-achieved] saved {out_path}  survival {surv.mean()*DT:.1f}s  "
          f"hip-roll splay {amp[1]:.1f}deg  hip-pitch {amp[0]:.1f}deg")


# ──────────────────────────────────────────────────────────────────────────
# SCRIPTED MOTION eval: drive a vx-command SCHEDULE (stand/walk/stop) and score
# per-segment ghost-similarity + durability over the WHOLE clip. The user's
# "follow a complete motion" idea: a stand-first clip should be deploy-stable
# (standing is solved) AND bound the walk to within G1's ~7s durability.
# ──────────────────────────────────────────────────────────────────────────
def eval_scripted_motion(env, ac, segs, seed=0):
    """segs = [(vx, dur_s), ...] e.g. [(0,5),(0.4,5),(0,5)] = stand/walk/stop.
    Drives a synchronous vx schedule on all envs (resample frozen), greedy.
    Needs a vx-conditioned policy (train with --vx-cmd-max>0)."""
    torch.manual_seed(seed); np.random.seed(seed)
    tdev = env.tdev; n = env.n; ac.eval()
    if not env.vx_cond:
        raise RuntimeError("eval_scripted_motion needs a vx-conditioned policy "
                           "(train with --vx-cmd-max > 0).")
    env._vx_freeze_resample = True
    sched = []; segid = []
    for si, (vx, dur) in enumerate(segs):
        ns = max(1, int(round(dur / DT)))
        sched += [float(vx)] * ns; segid += [si] * ns
    total = len(sched); nseg = len(segs)
    sdd = torch.zeros(nseg, n, NJ, device=tdev, dtype=torch.float64)
    gmn = torch.full((nseg, n, NJ), float("inf"), device=tdev, dtype=torch.float64)
    gmx = torch.full((nseg, n, NJ), float("-inf"), device=tdev, dtype=torch.float64)
    cnt = torch.zeros(nseg, n, device=tdev, dtype=torch.float64)
    fell = torch.zeros(n, dtype=torch.bool, device=tdev)
    fall_step = torch.zeros(n, dtype=torch.int32, device=tdev)
    obs = env.reset()
    for k in range(total):
        env.vx_cmd_t[:] = sched[k]
        with torch.no_grad():
            mu = ac.pi(obs)
        obs, _, done, _ = env.step(mu)
        a = env.qpos_t.index_select(1, env.qpos_idx_t).to(torch.float64)
        g = env._model_legs.to(torch.float64)
        m = (~fell); mm = m.to(torch.float64).unsqueeze(1); si = segid[k]
        sdd[si] += ((a - g) ** 2) * mm
        gmn[si] = torch.minimum(gmn[si], torch.where(m.unsqueeze(1), g, torch.full_like(g, float("inf"))))
        gmx[si] = torch.maximum(gmx[si], torch.where(m.unsqueeze(1), g, torch.full_like(g, float("-inf"))))
        cnt[si] += m.to(torch.float64)
        newly = done & (~fell)
        fall_step = torch.where(newly, torch.full_like(fall_step, k + 1), fall_step)
        fell = fell | done
    never = (~fell)
    surv = torch.where(never, torch.full_like(fall_step, total), fall_step).float() * DT
    print(f"\n[scripted] motion {total*DT:.0f}s ({', '.join(f'{v}@{d}s' for v, d in segs)})")
    print(f"[scripted] upright through WHOLE motion: {int(never.sum().item())}/{n}  "
          f"mean survival {surv.mean().item():.1f}s / {total*DT:.0f}s")
    seg_steps = [max(1, int(round(d / DT))) for _, d in segs]
    for si, (vx, dur) in enumerate(segs):
        good = cnt[si] >= 0.5 * seg_steps[si]
        ng = int(good.sum().item())
        if ng == 0:
            print(f"[scripted] seg{si} vx={vx} {dur}s: (no env survived >=half)")
            continue
        gi = torch.nonzero(good, as_tuple=False).squeeze(-1)
        c = cnt[si][gi].unsqueeze(1)
        rmse = torch.sqrt(torch.clamp(sdd[si][gi] / c, min=0.0))
        ptp = torch.clamp(gmx[si][gi] - gmn[si][gi], min=1e-4)
        fair = torch.clamp(1.0 - rmse / torch.clamp(ptp, min=PTP_FLOOR), min=0.0).mean().item()
        kind = "STAND" if vx == 0 else "walk"
        print(f"[scripted] seg{si} {kind} vx={vx} {dur}s: FAIR all-13 {100*fair:.1f}%  upright {ng}/{n}")


# ──────────────────────────────────────────────────────────────────────────
# Stage 0: persist the resolved physics config next to a trained policy.
# ──────────────────────────────────────────────────────────────────────────
def _save_physics_config(args, total_steps) -> None:
    """Write runs/<name>/physics_config.json = SPEC.resolved(extra=run-meta).

    extra carries the full argv + parsed args, the git SHA, a sha256 of the prim
    URDF bytes, and run stats -- so a trained policy is reproducible from disk
    alone. Wrapped so a dump failure logs a warning but never kills a run."""
    try:
        import json
        import hashlib
        import subprocess

        out_path = Path(args.save).with_name("physics_config.json")

        # git SHA (best-effort).
        try:
            git_sha = subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=str(REPO),
                stderr=subprocess.DEVNULL).decode().strip()
        except Exception:
            git_sha = None

        # sha256 of the prim URDF (the SoT collider file) bytes.
        try:
            prim_sha256 = hashlib.sha256(
                SPEC.URDF_PRIM.read_bytes()).hexdigest()
        except Exception:
            prim_sha256 = None

        extra = {
            "argv": list(sys.argv),
            "args": vars(args),
            "git_sha": git_sha,
            "prim_urdf": str(SPEC.URDF_PRIM).replace("\\", "/"),
            "prim_urdf_sha256": prim_sha256,
            "trainer": "gpu_newton_g1_walk_trainer.py",
            "save": str(args.save).replace("\\", "/"),
            "total_steps": int(total_steps),
            # physics constants ACTUALLY used by this run (may be env-overridden).
            "ke_used": KE_DEPLOY,
            "kd_used": KD_DEPLOY,
            "substeps_used": SUBSTEPS,
            "res_scale_used": float(args.res_scale),
        }
        snap = SPEC.resolved(extra=extra)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w", encoding="utf-8") as fh:
            json.dump(snap, fh, indent=2, default=str)
        print(f"saved physics_config -> {out_path}")
    except Exception as e:  # never let reproducibility-dump kill a run
        print(f"WARNING: physics_config.json dump failed (non-fatal): {e}")


# ──────────────────────────────────────────────────────────────────────────
# PPO fine-tune (mirrors gpu_mjwarp_g1_walk_trainer.main; warm-start required).
# ──────────────────────────────────────────────────────────────────────────
def main():
    import torch.nn as nn

    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=2048)
    p.add_argument("--iters", type=int, default=300)
    p.add_argument("--rollout", type=int, default=12)
    p.add_argument("--lr", type=float, default=1e-4,
                   help="LOWER than the mjwarp trainer (3e-4): this is a "
                        "fine-tune, keep the warm-start near its basin")
    p.add_argument("--init-from", required=True,
                   help="warm-start policy.pt (REQUIRED: from-scratch under "
                        "Newton fails -- this trainer ADAPTS a proven policy)")
    p.add_argument("--save",
                   default=str(REPO / "projects/policies/research/training/runs/gpu_newton_g1_walk/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=512)
    # GHOST-SIMILARITY EVAL (additive, no training). Loads the given policy.pt,
    # runs the DR-off parity env, and prints the amplitude/shape/per-joint
    # similarity-to-the-ghost metrics, then exits. Pass the deploy gait params
    # (--gait-model human --gait-style winter ... --obs-stack 4 etc) so the obs
    # layout matches the policy. --eval-steps controls the run length.
    p.add_argument("--eval-ghost-similarity", type=str, default=None,
                   help="path to a policy.pt; compute its gait's %% similarity "
                        "to the kinematic ghost and exit (no PPO).")
    p.add_argument("--build-achieved", type=str, default="",
                   help="path to write the (B) 'achieved' feasible ghost npz: "
                        "roll the --init-from policy in THIS Newton solver, "
                        "phase-bin + L/R-symmetrise + smooth its gait, save, exit. "
                        "Newton-feasible reference for --gait-style achieved.")
    p.add_argument("--eval-scripted", type=str, default="",
                   help="path to a vx-conditioned policy.pt; drive a scripted "
                        "stand/walk/stop vx schedule (--scripted-segs) and report "
                        "per-segment ghost-similarity + durability, then exit.")
    p.add_argument("--scripted-segs", type=str, default="0:5,0.4:5,0:5",
                   help="scripted motion as 'vx:dur_s,...' (default stand 5s, "
                        "walk@0.4 5s, stop 5s).")
    p.add_argument("--eval-ramp-s", type=float, default=2.0,
                   help="skip this many seconds of ramp-in before the steady "
                        "window used for the ghost-similarity stats")
    p.add_argument("--eval-window-s", type=float, default=3.0,
                   help="length (s) of the steady window after the ramp; the "
                        "stats use [ramp, ramp+window]. <=0 => to eval-steps. "
                        "Bound it below the typical fall time so the steady-walk "
                        "stats aren't polluted by the topple tail.")
    p.add_argument("--eval-min-window-frac", type=float, default=0.6,
                   help="an env is kept iff it stayed upright for >= this "
                        "fraction of the steady window")
    p.add_argument("--eval-seed", type=int, default=0)
    p.add_argument("--foot", default="urdf", choices=["urdf", "task"])
    # reward + gait knobs (subset that the documented warm-start config uses).
    p.add_argument("--alive", type=float, default=1.0)
    p.add_argument("--upright", type=float, default=0.5)
    p.add_argument("--act", type=float, default=-0.005)
    p.add_argument("--act-rate", type=float, default=-0.01)
    p.add_argument("--term", type=float, default=-10.0)
    p.add_argument("--vel", type=float, default=2.0)
    p.add_argument("--vel-sigma", type=float, default=0.10)
    p.add_argument("--vx-target", type=float, default=0.4)
    p.add_argument("--vx-cmd-max", type=float, default=0.0,
                   help="velocity-CONDITION the policy: it's told a target speed "
                        "sampled in [0,vx_cmd_max] (INCLUDING 0=stand), resampled "
                        "mid-episode so it learns stand<->walk<->stop transitions. "
                        "vx_cmd=0 freezes the gait phase -> static stand. 0=off "
                        "(fixed-speed). The mechanism for a scripted stand/walk/stop.")
    p.add_argument("--lat", type=float, default=-0.5)
    p.add_argument("--yaw", type=float, default=-0.5)
    p.add_argument("--height", type=float, default=-10.0)
    p.add_argument("--z-ref", type=float, default=0.74)
    p.add_argument("--gait-freq", type=float, default=1.3)
    p.add_argument("--gait-a-lat", type=float, default=0.05)
    p.add_argument("--max-ep", type=int, default=MJ.MAX_EP)
    p.add_argument("--res-scale", type=float, default=RES_SCALE)
    # IMITATION reward (ghost fidelity). The reward code reads these (lines
    # ~827-838) but reward_cfg never set them -> rw_track defaulted to 0.0, so
    # the policy was NEVER paid to look like the ghost (only survive + move).
    # --rw-track 1.0-2.0 pays it to keep its leg pose ON the ghost gait model.
    p.add_argument("--rw-track", type=float, default=0.0)
    p.add_argument("--track-sigma", type=float, default=0.05)
    p.add_argument("--rw-track-vel", type=float, default=0.0)
    p.add_argument("--track-vel-sigma", type=float, default=4.0)
    # COM-over-stance-foot + frontal-plane tracking rewards. The reward code in
    # _reward_tail READS these (rw_com ~line 916, rw_frontal_track ~line 925)
    # but main() never set them from CLI -> they defaulted to 0.0 and were dead
    # (same bug class as the historical --rw-track wiring). Defaults match the
    # parent mjwarp trainer + Agent af9eb909's recommendation.
    p.add_argument("--rw-com", type=float, default=0.0,
                   help="COM-OVER-STANCE-FOOT reward (POSITIVE, try 1.5-2.5): "
                        "during single support, pay for the pelvis (COM) being "
                        "laterally over the PLANTED foot -- the deliberate "
                        "weight transfer that fixes the drunk gait + leg spread "
                        "at the root WITHOUT a static hip-roll splay. "
                        "exp(-(py-stance_foot_y)^2/com_sigma).")
    p.add_argument("--com-sigma", type=float, default=0.004,
                   help="COM-over-stance gaussian width (m^2)")
    p.add_argument("--rw-frontal-track", type=float, default=0.0,
                   help="reward to pull hip-ROLL+YAW onto the improved shadow "
                        "(both legs). Use WITH --gait-lateral/--gait-yaw so the "
                        "policy tracks the lateral motion instead of splaying.")
    p.add_argument("--frontal-sigma", type=float, default=0.02,
                   help="gaussian width (rad^2) for --rw-frontal-track")
    # IMITATION CURRICULUM: ramp rw_track (and rw_track_vel) 0 -> target over the
    # first FRAC of training. 0 = constant (legacy). Constant imitation from step
    # 0 fights balance and collapses the deploy walk; annealing lets the policy
    # learn balance first, then gradually track the ghost.
    p.add_argument("--rw-track-warmup-frac", type=float, default=0.0)
    # Per-joint imitation: ankle imitation weight (0.0 frees the balancing ankles
    # from the open-loop-ghost imitation pull; hips+knees stay weight 1.0).
    p.add_argument("--track-ankle-w", type=float, default=0.5)
    p.add_argument("--track-waist-w", type=float, default=0.0)
    p.add_argument("--track-sagittal-w", type=float, default=1.0,
                   help="imitation weight on the SAGITTAL swing joints "
                        "(hip_pitch + knee, both legs). >1.0 (try 1.5-2.5) "
                        "concentrates the normalized rw_track budget on the hip "
                        "swing to raise its fidelity (the policy otherwise damps "
                        "it ~5x below the ghost). The key sagittal-fidelity lever.")
    p.add_argument("--obs-stack", type=int, default=4)
    p.add_argument("--obs-lookahead", type=str, default="0.1,0.4")
    p.add_argument("--hidden-dims", type=str, default="512,512,512")
    p.add_argument("--hold-arms", action="store_true")
    p.add_argument("--gait-model", default="", choices=["", "human"])
    p.add_argument("--gait-style", default="winter",
                   choices=["ik", "winter", "achieved"])
    p.add_argument("--gait-a-arm", type=float, default=0.25)
    p.add_argument("--gait-hip-scale", type=float, default=0.9)
    p.add_argument("--gait-ramp-s", type=float, default=2.0)
    p.add_argument("--rest-start-frac", type=float, default=0.4)
    p.add_argument("--asym-critic", action="store_true")
    p.add_argument("--ent-coef", type=float, default=0.003,
                   help="lower than the mjwarp default 0.01 -- a fine-tune "
                        "wants less exploration noise on the deploy engine")
    p.add_argument("--log-std-clamp", type=float, default=-1.2)
    p.add_argument("--mirror-loss", type=float, default=0.0,
                   help="L/R gait symmetry regularizer weight W (try 0.5-2.0). "
                        "Adds W*mean||pi(s) - mirror(pi(mirror(s)))||^2 to the "
                        "PPO objective so a mirrored state yields a mirrored "
                        "action. 0 = off (legacy). Targets the dominant "
                        "remaining ghost-fidelity gap (R leg deviates from L).")
    p.add_argument("--eval-every", type=int, default=0,
                   help="print the CLEAN ghost-similarity %% every N iters during "
                        "training (greedy, DR muted, steady-state -- the same "
                        "metric as --eval-ghost-similarity, run inline so you see "
                        "the gait close on the ghost live). 0 = off. Adds a short "
                        "eval rollout every N iters; env state is restored after.")
    # light DR (fine-tune on the real engine -> small bands)
    p.add_argument("--dr-push-prob", type=float, default=0.01)
    p.add_argument("--dr-push-vmax", type=float, default=0.8)
    p.add_argument("--dr-obs-noise", type=float, default=0.02)
    p.add_argument("--dr-action-latency-max", type=int, default=2)
    p.add_argument("--dr-act-gain", type=float, default=0.0)
    p.add_argument("--dr-init-q-band", type=float, default=0.08)
    p.add_argument("--dr-init-xy-band", type=float, default=0.03)
    p.add_argument("--dr-init-tilt-band", type=float, default=0.0)
    p.add_argument("--dr-init-vel-band", type=float, default=0.0)
    p.add_argument("--train-joint-clamp", action="store_true",
                   help="mirror the deploy's post-step joint clamp (clamp achieved "
                        "leg qpos to URDF limits, qvel to +/-velocity_limit, zero "
                        "into-stop velocity) so trainer physics == deploy physics. "
                        "The deploy clamp is load-bearing (disabling it in deploy "
                        "made the walk WORSE), so train WITH it.")
    p.add_argument("--no-dr", action="store_true")
    # human-gait params (forwarded to GaitParams; match the warm-start config)
    p.add_argument("--gait-duty", type=float, default=0.6)
    p.add_argument("--gait-step-height", type=float, default=0.05)
    p.add_argument("--gait-pelvis-h", type=float, default=0.755)
    p.add_argument("--gait-bob", type=float, default=0.020)
    p.add_argument("--gait-x0", type=float, default=-0.02)
    p.add_argument("--gait-elbow", type=float, default=0.15)
    p.add_argument("--gait-ankle-clear", type=float, default=0.08)
    p.add_argument("--gait-lateral", default="sway", choices=["sway", "lipm", "human"])
    p.add_argument("--gait-yaw", default="none", choices=["none", "human"])
    # LIPM frontal-plane shape (matches the deploy's G1_GAIT_LAT_HIP_AMP /
    # G1_GAIT_STEP_WIDTH env knobs). Forwarded into GaitParams so the lateral
    # amplitude / stance width the trainer's ghost uses == the deploy's.
    p.add_argument("--gait-lat-hip-amp", type=float, default=0.09,
                   help="(A lipm) peak hip-roll for LIPM weight transfer (rad)")
    p.add_argument("--gait-step-width", type=float, default=0.12,
                   help="(A lipm) lateral foot separation (m) -- LIPM shape")
    args = p.parse_args()

    reward_cfg = dict(
        alive=args.alive, upright=args.upright, act=args.act, act_rate=args.act_rate,
        term=args.term, vel=args.vel, vel_sigma=args.vel_sigma, vx_target=args.vx_target,
        vx_cmd_max=args.vx_cmd_max,
        lat=args.lat, yaw=args.yaw, height=args.height, z_ref=args.z_ref,
        gait_freq=args.gait_freq, gait_a_lat=args.gait_a_lat, max_ep=args.max_ep,
        res_scale=args.res_scale, obs_stack=args.obs_stack,
        obs_lookahead=[float(x) for x in args.obs_lookahead.split(",") if x.strip()],
        asym_critic=args.asym_critic, rest_start_frac=args.rest_start_frac,
        gait_a_arm=args.gait_a_arm,
        rw_track=args.rw_track, track_sigma=args.track_sigma,
        rw_track_vel=args.rw_track_vel, track_vel_sigma=args.track_vel_sigma,
        rw_com=args.rw_com, com_sigma=args.com_sigma,
        rw_frontal_track=args.rw_frontal_track, frontal_sigma=args.frontal_sigma,
        track_ankle_w=args.track_ankle_w,
        track_waist_w=args.track_waist_w,
        track_sagittal_w=args.track_sagittal_w)
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
            push_prob=args.dr_push_prob, push_vmax=args.dr_push_vmax,
            obs_noise=args.dr_obs_noise, action_latency_max=args.dr_action_latency_max,
            act_gain=args.dr_act_gain, init_q_band=args.dr_init_q_band,
            init_xy_band=args.dr_init_xy_band, init_tilt_band=args.dr_init_tilt_band,
            init_vel_band=args.dr_init_vel_band)
        print(f"[DR] {dr_cfg}")
    # Faithful-parity flag lives in dr_cfg so it survives --no-dr too.
    dr_cfg["train_joint_clamp"] = bool(args.train_joint_clamp)
    if args.train_joint_clamp:
        print("[parity] post-step joint clamp ON in trainer (mirrors deploy)")

    env = BatchedG1NewtonEnv(args.envs, reward_cfg=reward_cfg, dr_cfg=dr_cfg,
                             hold_arms=args.hold_arms, foot=args.foot)
    N = args.envs

    _n_look = len([x for x in args.obs_lookahead.split(",") if x.strip()])
    OBS_IN = OBS_DIM * max(1, args.obs_stack) + NJ * _n_look
    OBS_IN += 1 if env.vx_cond else 0
    PRIV_IN = OBS_IN + env.priv_extra
    ASYM = env.asym
    print(f"[obs] OBS_DIM={OBS_DIM} stack={args.obs_stack} lookahead={_n_look} "
          f"-> OBS_IN={OBS_IN} (warm-start expects 226)")

    # L/R symmetry regularizer (off by default). Built once; applied in the PPO
    # epoch loop. mirror() is self-inverse (verified), so it can only pull the
    # policy toward symmetry, never fight a correct one.
    mirror_obs_op = None
    if args.mirror_loss > 0.0:
        if env.vx_cond:
            raise SystemExit("--mirror-loss assumes vx_cond OFF (no trailing "
                             "command column); extend _build_mirror_obs_op first.")
        mirror_obs_op = _build_mirror_obs_op(OBS_IN, OBS_DIM, NJ, _n_look, env.tdev)
        print(f"[mirror] symmetry loss ON  W={args.mirror_loss}  "
              f"P={_MIRROR_P}  S={_MIRROR_S}")

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
            self.v = _mlp(PRIV_IN, 1)
            self.log_std = nn.Parameter(-1.0 * torch.ones(NJ))

        def forward(self, obs, priv=None):
            if priv is None:
                priv = obs
            return self.pi(obs), self.v(priv).squeeze(-1), self.log_std

    tdev = env.tdev
    torch.manual_seed(0)
    ac = AC().to(tdev)
    _sd = torch.load(args.init_from, map_location=tdev)
    cur = ac.state_dict()
    for k in ("pi.0.weight", "v.0.weight"):
        if k in _sd and k in cur and _sd[k].shape[1] != cur[k].shape[1]:
            ww = cur[k].clone(); ww.zero_()
            c = min(ww.shape[1], _sd[k].shape[1])
            ww[:, :c] = _sd[k][:, :c]
            _sd[k] = ww
            print(f"  [warm] zero-padded {k}: {_sd[k].shape}")
    missing = ac.load_state_dict(_sd, strict=False)
    print(f"warm-start from {args.init_from}  (missing/unexpected: {missing})")
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)

    # ── GHOST-SIMILARITY EVAL (additive; no training) ──
    if args.eval_ghost_similarity:
        esd = torch.load(args.eval_ghost_similarity, map_location=tdev)
        miss = ac.load_state_dict(esd, strict=False)
        print(f"[ghost-eval] loaded policy {args.eval_ghost_similarity} "
              f"(missing/unexpected: {miss})")
        res = eval_ghost_similarity(env, ac, args.eval_steps, args.eval_ramp_s,
                                    window_s=args.eval_window_s,
                                    min_window_frac=args.eval_min_window_frac,
                                    seed=args.eval_seed)
        _print_ghost_similarity(res, args.eval_ghost_similarity)
        return

    if args.build_achieved:
        # ac already holds the --init-from policy (warm-start above).
        _build_achieved_gait(env, ac, args.build_achieved)
        return

    if args.eval_scripted:
        esd = torch.load(args.eval_scripted, map_location=tdev)
        ac.load_state_dict(esd, strict=False)
        segs = []
        for tok in args.scripted_segs.split(","):
            v, d = tok.split(":")
            segs.append((float(v), float(d)))
        eval_scripted_motion(env, ac, segs, seed=args.eval_seed)
        return

    if args.eval:
        ac.eval()
        ac.load_state_dict(torch.load(args.save, map_location=tdev), strict=False)
        obs = env.reset()
        first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        dist_acc = torch.zeros(env.n, device=tdev)
        for step in range(args.eval_steps):
            with torch.no_grad():
                mu = ac.pi(obs)
            alive = (first_fall == 0)
            obs, _, done, _ = env.step(mu)
            dist_acc += env.qvel_t[:, 0] * DT * alive.float() * (~done).float()
            newly = done & (first_fall == 0)
            first_fall = torch.where(newly, torch.full_like(first_fall, step + 1), first_fall)
        ff = first_fall.cpu().numpy()
        never = (ff == 0)
        print(f"[newton-eval] never_fell={never.mean():.2f} "
              f"mean_first_fall={ff[~never].mean() if (~never).any() else -1:.1f} "
              f"fwd_dist mean={dist_acc.mean().item():.2f}m")
        return

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    rollout = args.rollout
    obs = env.reset()

    # CORRECTNESS GATE: the warm-start mean action at the spawn obs must be a
    # SMALL residual (|mu| order ~ a fraction of 1, not random spasm), and the
    # first batch of greedy rollout should NOT instantly fall on every env.
    with torch.no_grad():
        mu0 = ac.pi(obs)
    print(f"[gate] spawn obs: shape={tuple(obs.shape)} "
          f"finite={torch.isfinite(obs).all().item()} "
          f"|obs|max={obs.abs().max().item():.2f}")
    print(f"[gate] warm-start mu at spawn: |mu|mean={mu0.abs().mean().item():.3f} "
          f"|mu|max={mu0.abs().max().item():.3f} "
          f"(small/bounded => obs matches the policy)")

    total_steps = 0
    t0 = time.time()
    obs_buf = torch.zeros(rollout, N, OBS_IN, device=tdev)
    priv_buf = torch.zeros(rollout, N, PRIV_IN, device=tdev)
    act_buf = torch.zeros(rollout, N, NJ, device=tdev)
    logp_buf = torch.zeros(rollout, N, device=tdev)
    rew_buf = torch.zeros(rollout, N, device=tdev)
    done_buf = torch.zeros(rollout, N, device=tdev)
    val_buf = torch.zeros(rollout, N, device=tdev)
    nan_seen = False

    for it in range(1, args.iters + 1):
        # IMITATION CURRICULUM: ramp rw_track 0 -> target over the first
        # warmup_frac of iters (balance learned first, ghost-tracking added
        # gradually). env.r is the live reward_cfg the per-step reward reads.
        if args.rw_track_warmup_frac and args.rw_track_warmup_frac > 0.0:
            _f = min(1.0, (it - 1) / max(1.0, args.rw_track_warmup_frac * args.iters))
            env.r["rw_track"] = args.rw_track * _f
            env.r["rw_track_vel"] = args.rw_track_vel * _f
            # COM-over-stance + frontal-track are part of the same imitation
            # curriculum (style added after balance) -> ramp them together.
            env.r["rw_com"] = args.rw_com * _f
            env.r["rw_frontal_track"] = args.rw_frontal_track * _f
        for k in range(rollout):
            priv = env.priv_obs(obs) if ASYM else obs
            with torch.no_grad():
                mu, v, log_std = ac(obs, priv)
                std = log_std.exp()
                d = torch.distributions.Normal(mu, std)
                a = d.sample()
                logp = d.log_prob(a).sum(-1)
            obs_buf[k] = obs; priv_buf[k] = priv; act_buf[k] = a
            logp_buf[k] = logp; val_buf[k] = v
            obs, rw, done, _ = env.step(a)
            rew_buf[k] = rw; done_buf[k] = done.float()
            total_steps += N
        if not torch.isfinite(rew_buf).all() or not torch.isfinite(obs).all():
            nan_seen = True
            print(f"[WARN] NaN/Inf detected at it {it}")

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
        # mirror maps for the ACTION output (same P,S as the joints), built once.
        if mirror_obs_op is not None:
            _act_p = torch.tensor(_MIRROR_P, dtype=torch.long, device=tdev)
            _act_s = torch.tensor(_MIRROR_S, dtype=torch.float32, device=tdev)
        sym_loss_last = 0.0
        for _epoch in range(4):
            mu, v, log_std = ac(obs_flat, priv_flat)
            std = log_std.exp()
            d = torch.distributions.Normal(mu, std)
            new_logp = d.log_prob(act_flat).sum(-1)
            ratio = (new_logp - logp_flat).exp()
            surr1 = ratio * adv_flat
            surr2 = torch.clamp(ratio, 1 - clip_eps, 1 + clip_eps) * adv_flat
            pi_loss = -torch.min(surr1, surr2).mean()
            v_loss = ((v - ret_flat) ** 2).mean()
            ent = d.entropy().sum(-1).mean()
            loss = pi_loss + 0.5 * v_loss - args.ent_coef * ent
            # MIRROR-SYMMETRY REGULARIZER: pi(mirror(s)) should equal
            # mirror(pi(s)).  loss_sym = mean|| pi(s) - mirror(pi(mirror(s))) ||^2
            # (uses the policy mean mu; mirror() is self-inverse so the term is
            # symmetric in s and exactly 0 on a perfectly symmetric gait).
            if mirror_obs_op is not None:
                mu_mir = ac.pi(mirror_obs_op(obs_flat))             # pi(mirror(s))
                mu_mir_back = mu_mir.index_select(1, _act_p) * _act_s  # mirror(.)
                sym_loss = ((mu - mu_mir_back) ** 2).sum(-1).mean()
                loss = loss + args.mirror_loss * sym_loss
                sym_loss_last = float(sym_loss.detach())
            opt.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5)
            opt.step()
            if args.log_std_clamp is not None:
                with torch.no_grad():
                    ac.log_std.clamp_(max=args.log_std_clamp)

        if it % 1 == 0 or it == 1:
            ep_rps = rew_buf.mean().item()
            mean_v = val_buf.mean().item()
            fall_rate = done_buf.mean().item()
            fps = total_steps / max(time.time() - t0, 1e-6)
            print(f"it {it:4d}  ep_rew/step~{ep_rps:+.3f}  meanV {mean_v:+.2f}  "
                  f"fall/step {fall_rate:.3f}  steps {total_steps:,}  "
                  f"{fps:,.0f} env-steps/s"
                  + (f"  sym {sym_loss_last:.4f}" if mirror_obs_op is not None else ""))

        # ── LIVE GHOST-SIMILARITY READOUT (the %, inline) ──
        # The similarity % is a CLEAN greedy/steady-state measure -- NOT the
        # reward (which is a stochastic, DR-on, transient-contaminated weighted
        # sum). So we measure it with a short dedicated rollout: mute DR, run the
        # exact eval metric on the greedy policy, then RESTORE env state + reset
        # so training continues cleanly. Non-fatal (early on, all envs may fall).
        if args.eval_every and it % args.eval_every == 0:
            _sv = (env._push_p, env._push_vmax, env._obs_noise,
                   env.max_latency_ticks, env._act_gain,
                   env._init_q_band, env._init_xy_band, env._init_z_band, env.dr)
            env._push_p = env._push_vmax = env._obs_noise = 0.0
            env.max_latency_ticks = 0; env._act_gain = 0.0
            env._init_q_band = env._init_xy_band = env._init_z_band = 0.0
            env.dr = {}                     # mutes init_tilt/vel/vx (read fresh)
            try:
                _r = eval_ghost_similarity(
                    env, ac, args.eval_steps, args.eval_ramp_s,
                    window_s=args.eval_window_s,
                    min_window_frac=args.eval_min_window_frac,
                    seed=args.eval_seed, diag_traces=False)
                print(f"  [ghost@{it}] FAIR all-13 {_r['headline_all_fair']:.1f}%"
                      f"  moving {_r['headline_moving_fair']:.1f}%"
                      f"  SHAPE all-13 {_r['shape_all']:.1f}%"
                      f"  upright {_r['n_good']}/{_r['n_total']}", flush=True)
            except Exception as _e:
                print(f"  [ghost@{it}] eval skipped: {_e}", flush=True)
            finally:
                (env._push_p, env._push_vmax, env._obs_noise,
                 env.max_latency_ticks, env._act_gain,
                 env._init_q_band, env._init_xy_band, env._init_z_band,
                 env.dr) = _sv
                ac.train()
                obs = env.reset()

    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total_steps:,} steps in {time.time()-t0:.1f}s)  "
          f"NaN_seen={nan_seen}")

    # ── Stage 0: persist the FULLY-RESOLVED physics config next to the policy so
    # a run is reproducible from disk alone (closes the "no run config is ever
    # saved" hole). Robust: a dump failure must NEVER kill a training run. ──
    _save_physics_config(args, total_steps)

    # ── ONNX export (identical to the mjwarp trainer: clamped head, same
    # input name, dynamic batch axis) so g1_walk_deploy can load it. ──
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
                opset_version=17)
        print(f"exported ONNX -> {onnx_path}")
    except Exception as e:
        print(f"ONNX export failed (non-fatal): {e}")


if __name__ == "__main__":
    main()
