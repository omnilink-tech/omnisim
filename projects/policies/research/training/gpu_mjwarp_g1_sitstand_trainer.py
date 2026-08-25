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

"""GPU mujoco_warp trainer for the G1 SIT -> STAND -> stand(5s) -> SIT task.

Steps up the seated trainer: the robot starts SEATED on the chair, must RISE to
standing in front of it, hold standing 5 s, then SIT back down -- all while
balancing (free root, no pin). Sit-to-stand is a hard dynamic-balance maneuver.

Design (adapts gpu_mjwarp_g1_sit_trainer.py):
  - The reference is TIME-VARYING (projects/policies/control/gait/g1_sitstand): a smoothstep
    blend seated<->standing + a reference PELVIS HEIGHT (0.44->0.78). The 13
    leg+waist joints are the policy (residual on the time-varying reference); the
    10 arms follow the reference open-loop.
  - r_height (pelvis tracks the reference height) is what turns "extend the legs"
    into "actually rise" without falling; r_track keeps the legs on the sit-stand
    trajectory; mild upright (the rise leans forward -- don't over-penalize);
    alive. Terminate on a fall RELATIVE to the reference height (so a failed rise
    terminates) or a large tilt.
  - Per the recipe rule, this reference is the TRAINING GUIDE; the deployed ghost
    will REPLAY the robot's achieved motion (record/replay) so it's achievable.

Usage:
    python projects/policies/research/training/gpu_mjwarp_g1_sitstand_trainer.py \\
        --envs 4096 --iters 1500 --save .../runs/gpu_g1_sitstand/policy.pt
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

import projects.policies.control.gait.g1_sitstand as GS
from projects.policies.control.gait.g1_sitstand import SEATED_POSE, STANDING_POSE, T_TOTAL, Z_SEATED, Z_STAND

NJ = 13
# 9 base + q-ref(13) + qd(13) + last_action(13) + phase[b, b_ahead, z_err, x_err,
# pitch_err](5) = 53. The policy TRACKS the designed ghost (g1_sitstand): the obs
# tells it where it is in the motion + its base error vs the reference.
OBS_DIM = 53
RES_SCALE = 0.3
DT = 0.016
SUBSTEPS = 4
PHYS_DT = 0.004
PHASE_LOOKAHEAD = 0.3   # s ahead for the phase feedforward

LEGS_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
)
ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
)
# Keyframe arrays (seated + standing) in leg / arm order.
SEAT_LEGS = np.array([SEATED_POSE[j] for j in LEGS_JOINTS], dtype=np.float32)
STAND_LEGS = np.array([STANDING_POSE[j] for j in LEGS_JOINTS], dtype=np.float32)
SEAT_ARMS = np.array([SEATED_POSE[j] for j in ARM_JOINTS], dtype=np.float32)
STAND_ARMS = np.array([STANDING_POSE[j] for j in ARM_JOINTS], dtype=np.float32)

SPAWN_Z = 0.57       # seated spawn just above the settle (feet clear the floor;
                     # settles ~0.55). The old 0.47 sank the feet 7cm THROUGH the
                     # floor -> contact-solver launch at spawn.
JOINT_LIMITS_LO = np.array([
    -2.531, -0.524, -2.758, -0.087, -0.873, -0.262,
    -2.531, -2.967, -2.758, -0.087, -0.873, -0.262, -2.618], dtype=np.float32)
JOINT_LIMITS_HI = np.array([
    +2.880, +2.967, +2.758, +2.880, +0.524, +0.262,
    +2.880, +0.524, +2.758, +2.880, +0.524, +0.262, +2.618], dtype=np.float32)

LOOKAHEAD_STEPS = int(round(PHASE_LOOKAHEAD / DT))
ROLL_FAIL = 0.9
PITCH_DEV_FAIL = 0.6   # |pitch - reference lean| this large = a fall. The reference now
                       # ENCODES the (dynamic) forward lean of the step, so this measures
                       # deviation FROM the step profile -- 0.6 allows the dynamic step swing
                       # while still catching a real forward collapse.
Z_FALL_MARGIN = 0.18   # pelvis this far below the reference height = a fall

# The reference (the GHOST) is a lookup table indexed by ep_step: per-step 13 leg q,
# 10 arm q, base (x, z, lean-pitch), and stand fraction b. The trainer just TRACKS it.
#   SITSTAND_REF_NPZ=<file> -> load a PLANNER (trajectory-optimization) reference, a
#     DYNAMICALLY-feasible motion (g1_sitstand_trajopt). This is the ghost-tracking
#     pipeline's Stage B output -- the right reference (no seated-start knife-edge).
#   else -> the hand-drawn IK reference from g1_sitstand (quasi-static; legacy).
import os as _os_ref
_REF_NPZ = _os_ref.environ.get("SITSTAND_REF_NPZ")
if _REF_NPZ:
    _d = np.load(_REF_NPZ, allow_pickle=True)
    _traj = _d["traj"]; _pdt = float(_d["dt"]); _pnames = [str(s) for s in _d["joints"]]
    _tr = _traj[:: max(1, int(round(DT / _pdt)))]                 # downsample to the trainer DT
    MAX_EP = _tr.shape[0]
    _ni = {n: k for k, n in enumerate(_pnames)}                   # joint name -> column
    REF_LEGS = np.array([[_tr[i, _ni[j]] for j in LEGS_JOINTS] for i in range(MAX_EP)], dtype=np.float32)
    REF_ARMS = np.array([[_tr[i, _ni[j]] for j in ARM_JOINTS] for i in range(MAX_EP)], dtype=np.float32)
    REF_X = _tr[:, 23].astype(np.float32)
    REF_Z = _tr[:, 24].astype(np.float32)
    REF_PITCH = (-_tr[:, 25]).astype(np.float32)                  # planner saved -pitch; obs uses +pitch
    REF_B = np.clip((REF_Z - Z_SEATED) / (Z_STAND - Z_SEATED), 0.0, 1.0).astype(np.float32)
    # DeepMimic velocity tracking: if the ref carries velocities (2*nj+6 cols), load the
    # leg-joint velocities + base linear velocity (the missing imitation ingredient).
    _NJ_REF = len(_pnames)
    if _tr.shape[1] >= 2 * _NJ_REF + 6:
        REF_QD = np.array([[_tr[i, _NJ_REF + 3 + _ni[j]] for j in LEGS_JOINTS]
                           for i in range(MAX_EP)], dtype=np.float32)
        REF_BVEL = _tr[:, 2 * _NJ_REF + 3:2 * _NJ_REF + 6].astype(np.float32)
        print(f"[sitstand] reference carries VELOCITIES -> DeepMimic vel-tracking ON")
    else:
        REF_QD = REF_BVEL = None
    print(f"[sitstand] FEASIBLE (planner) reference: {_REF_NPZ} -> {MAX_EP} steps")
else:
    REF_QD = REF_BVEL = None
    MAX_EP = int(round(T_TOTAL / DT)) + 30   # one full sit-stand-sit cycle + margin
    _FT = [GS.full_targets(i * DT) for i in range(MAX_EP)]
    REF_LEGS = np.array([[ft[j] for j in LEGS_JOINTS] for ft in _FT], dtype=np.float32)
    REF_ARMS = np.array([[ft[j] for j in ARM_JOINTS] for ft in _FT], dtype=np.float32)
    REF_X = np.array([GS.ref_pelvis_x(i * DT) for i in range(MAX_EP)], dtype=np.float32)
    REF_Z = np.array([GS.ref_pelvis_z(i * DT) for i in range(MAX_EP)], dtype=np.float32)
    REF_PITCH = np.array([GS.ref_pelvis_pitch(i * DT) for i in range(MAX_EP)], dtype=np.float32)
    REF_B = np.array([GS.blend(i * DT) for i in range(MAX_EP)], dtype=np.float32)


class BatchedG1SitStandEnv:
    def __init__(self, n, mjcf, device="cuda:0", reward_cfg=None, sim_dt=0.0, dr_cfg=None,
                 rsi_frac=0.0, res_scale=RES_SCALE):
        import warp as wp
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mjw = wp, mjw
        self.n = n
        self.device = wp.get_device(device)
        self.dr = dr_cfg or {}
        # reference-state init: fraction of resets that start at a RANDOM phase of
        # the reference (some already standing) so the policy directly experiences
        # standing-balance instead of having to discover the rise by luck from
        # seated -- escapes the deep-crouch local optimum.
        self._rsi_frac = float(rsi_frac)
        # REVERSE CURRICULUM: RSI start phases are sampled from [lo_frac, 1]*MAX_EP.
        # Annealed from high (start near the STAND -> easy) down to 0 (full range incl.
        # the hard dead-seated LAUNCH) over training, so the policy learns the motion
        # back-to-front and the launch is a small extension of an already-mastered rise.
        self._rsi_lo_frac = 0.0
        # CONCENTRATED reverse curriculum: when _rsi_band > 0, start phases are sampled from a
        # NARROW band [lo, lo+band] that sweeps from near-stand back to seated as lo->0 -- so the
        # policy MASTERS the receding frontier (incl. the dead-seated LAUNCH) instead of seeing
        # it rarely (uniform [lo,1] left phase~0 at ~1/MAX_EP probability -> launch never learned,
        # even at 16384 envs). _rsi_band==0 -> the old uniform[lo,1] behavior.
        self._rsi_band = 0.0
        # residual authority: leg_target = ref_legs(t) + res_scale * action. TIGHT
        # (e.g. 0.15) forces the legs to FOLLOW the seated<->stand reference (the
        # policy only fine-tunes balance) -- prevents the policy overriding the
        # phase to park at a safe compromise height (the v3 failure).
        self._res_scale = float(res_scale)
        self.mjm = mujoco.MjModel.from_xml_path(mjcf)
        self.mjm.opt.timestep = float(sim_dt) if sim_dt and sim_dt > 0 else PHYS_DT

        rng = np.random.default_rng(self.dr.get("seed", 0))
        if self.dr.get("mass_scale", 0.0) > 0:
            b = self.dr["mass_scale"]
            sc = rng.uniform(1 - b, 1 + b, size=self.mjm.body_mass.shape).astype(np.float32)
            self.mjm.body_mass[:] *= sc; self.mjm.body_inertia[:] *= sc[:, None]
        if self.dr.get("friction_scale", 0.0) > 0:
            b = self.dr["friction_scale"]
            self.mjm.geom_friction[:, 0] *= float(rng.uniform(1 - b, 1 + b))
        if self.dr.get("damping_scale", 0.0) > 0:
            b = self.dr["damping_scale"]
            ds = rng.uniform(1 - b, 1 + b, size=self.mjm.dof_damping.shape).astype(np.float32)
            self.mjm.dof_damping[:] *= ds
        akp = self.dr.get("actuator_kp_scale", 0.0); akv = self.dr.get("actuator_kv_scale", 0.0)
        if akp > 0 or akv > 0:
            for ai in range(self.mjm.nu):
                gp = self.mjm.actuator_gainprm[ai]; bp = self.mjm.actuator_biasprm[ai]
                if abs(bp[1]) > 1e-6 and akp > 0:
                    s = float(rng.uniform(1 - akp, 1 + akp)); gp[0] *= s; bp[1] *= s
                elif abs(bp[2]) > 1e-6 and akv > 0:
                    s = float(rng.uniform(1 - akv, 1 + akv)); gp[0] *= s; bp[2] *= s
        if self.dr.get("gravity_scale", 0.0) > 0:
            b = self.dr["gravity_scale"]; self.mjm.opt.gravity[2] *= float(rng.uniform(1 - b, 1 + b))
        # CONTACT-softness DR: randomize the contact solref (time constant +
        # damping ratio) and solimp. THE knob that crosses the warp<->Newton gap
        # (the deploy collapsed/over-leaned because Newton contact is softer/
        # different than warp's; randomizing it makes the policy robust). Same
        # mechanism as the walk trainer.
        solref_band = self.dr.get("solref_scale", 0.0)
        if solref_band > 0:
            tc = rng.uniform(1.0 - solref_band, 1.0 + solref_band,
                             size=self.mjm.geom_solref[:, 0].shape).astype(np.float32)
            dratio = rng.uniform(1.0 - solref_band, 1.0 + solref_band,
                                 size=self.mjm.geom_solref[:, 1].shape).astype(np.float32)
            self.mjm.geom_solref[:, 0] *= tc
            self.mjm.geom_solref[:, 1] *= dratio
            self.mjm.geom_solref[:, 0] = np.clip(
                self.mjm.geom_solref[:, 0], 2.5 * float(self.mjm.opt.timestep), 0.1)
            # keep the damping ratio sane (underdamped contact -> bouncy blow-ups).
            self.mjm.geom_solref[:, 1] = np.clip(self.mjm.geom_solref[:, 1], 0.6, 3.0)
            si = float(rng.uniform(1.0 - 0.5 * solref_band, 1.0 + 0.5 * solref_band))
            self.mjm.geom_solimp[:, :2] = np.clip(
                self.mjm.geom_solimp[:, :2] * si, 0.0, 0.9999)

        mjd = mujoco.MjData(self.mjm)
        mujoco.mj_forward(self.mjm, mjd)
        with wp.ScopedDevice(self.device):
            self.mw_m = mjw.put_model(self.mjm)
            import os as _njos
            _njmax = int(_njos.environ.get("G1_NJMAX", "256"))
            _nconmax = int(_njos.environ.get("G1_NCONMAX", "256"))
            self.mw_d = mjw.put_data(self.mjm, mjd, nworld=n, njmax=_njmax, nconmax=_nconmax)

        self.c2qpos = np.zeros(NJ, dtype=np.int32); self.c2qvel = np.zeros(NJ, dtype=np.int32)
        self.c2ctrlpos = np.zeros(NJ, dtype=np.int32)
        for i, jn in enumerate(LEGS_JOINTS):
            jid = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_JOINT, jn)
            self.c2qpos[i] = self.mjm.jnt_qposadr[jid]; self.c2qvel[i] = self.mjm.jnt_dofadr[jid]
            self.c2ctrlpos[i] = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_pos")
        self.arm_qpos = np.zeros(len(ARM_JOINTS), dtype=np.int32)
        self.arm_ctrlpos = np.zeros(len(ARM_JOINTS), dtype=np.int32)
        for i, jn in enumerate(ARM_JOINTS):
            jid = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_JOINT, jn)
            self.arm_qpos[i] = self.mjm.jnt_qposadr[jid]
            self.arm_ctrlpos[i] = mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{jn}_pos")

        self.r = reward_cfg or {}
        self.nq, self.nv, self.nu = self.mjm.nq, self.mjm.nv, self.mjm.nu
        # seed = the reference's START frame (planner or IK): base x/z + lean + legs
        # + arms, so the dead-seated spawn matches the (feasible) reference at t=0.
        self.seed_qpos = mjd.qpos.copy().astype(np.float32)
        _p0 = float(REF_PITCH[0])
        self.seed_qpos[0:3] = [float(REF_X[0]), 0.0, float(REF_Z[0]) + 0.005]
        self.seed_qpos[3:7] = [math.cos(_p0 * 0.5), 0.0, math.sin(_p0 * 0.5), 0.0]
        for i in range(len(LEGS_JOINTS)):
            self.seed_qpos[self.c2qpos[i]] = float(REF_LEGS[0][i])
        for i in range(len(ARM_JOINTS)):
            self.seed_qpos[self.arm_qpos[i]] = float(REF_ARMS[0][i])

        self.tdev = torch.device("cuda:0" if "cuda" in str(self.device).lower() else "cpu")
        self.qpos_t = wp.to_torch(self.mw_d.qpos).view(n, self.nq)
        self.qvel_t = wp.to_torch(self.mw_d.qvel).view(n, self.nv)
        self.ctrl_t = wp.to_torch(self.mw_d.ctrl).view(n, self.nu)
        # body world positions (for the foot-under-body / anti-slide reward)
        self.xpos_t = wp.to_torch(self.mw_d.xpos).view(n, self.mjm.nbody, 3)
        self.ankle_bids = torch.tensor(
            [mujoco.mj_name2id(self.mjm, mujoco.mjtObj.mjOBJ_BODY, b)
             for b in ("left_ankle_roll_link", "right_ankle_roll_link")],
            dtype=torch.long, device=self.tdev)
        d = self.tdev
        self.seat_legs_t = torch.tensor(SEAT_LEGS, device=d)
        self.stand_legs_t = torch.tensor(STAND_LEGS, device=d)
        self.seat_arms_t = torch.tensor(SEAT_ARMS, device=d)
        self.stand_arms_t = torch.tensor(STAND_ARMS, device=d)
        # precomputed ghost reference tables (indexed by ep_step)
        self.ref_legs_t = torch.tensor(REF_LEGS, device=d)     # [MAX_EP, 13]
        self.ref_arms_t = torch.tensor(REF_ARMS, device=d)     # [MAX_EP, 10]
        self.ref_x_t = torch.tensor(REF_X, device=d)           # [MAX_EP]
        self.ref_z_t = torch.tensor(REF_Z, device=d)
        self.ref_pitch_t = torch.tensor(REF_PITCH, device=d)
        self.ref_b_t = torch.tensor(REF_B, device=d)
        # DeepMimic velocity references (leg-joint vels + base lin vel), or None
        self.ref_qd_t = torch.tensor(REF_QD, device=d) if REF_QD is not None else None       # [MAX_EP,13]
        self.ref_bvel_t = torch.tensor(REF_BVEL, device=d) if REF_BVEL is not None else None  # [MAX_EP,3]
        self.jl_lo_t = torch.tensor(JOINT_LIMITS_LO, device=d)
        self.jl_hi_t = torch.tensor(JOINT_LIMITS_HI, device=d)
        self.qpos_idx_t = torch.tensor(self.c2qpos, dtype=torch.long, device=d)
        self.qvel_idx_t = torch.tensor(self.c2qvel, dtype=torch.long, device=d)
        self.ctrl_pos_idx_t = torch.tensor(self.c2ctrlpos, dtype=torch.long, device=d)
        self.arm_ctrl_pos_idx_t = torch.tensor(self.arm_ctrlpos, dtype=torch.long, device=d)
        self.arm_qpos_t = torch.tensor(self.arm_qpos, dtype=torch.long, device=d)
        self.seed_qpos_t = torch.tensor(self.seed_qpos, device=d)
        self.ep_step_t = torch.zeros(n, dtype=torch.int32, device=d)
        self.last_action_t = torch.zeros(n, NJ, device=d)
        self._push_p = float(self.dr.get("push_prob", 0.0))
        self._push_vmax = float(self.dr.get("push_vmax", 0.0))
        self._obs_noise = float(self.dr.get("obs_noise", 0.0))
        self._init_q_band = float(self.dr.get("init_q_band", 0.05))
        self._cuda_graph = None
        self._try_capture_graph()
        self._reset_all()

    def _ref(self, step):
        """Ghost reference at ep_step (int tensor): (ref_legs[n,13], ref_arms[n,10],
        ref_x[n], ref_z[n], ref_pitch[n], b[n]) from the precomputed lookup tables."""
        idx = step.clamp(0, MAX_EP - 1).long()
        return (self.ref_legs_t[idx], self.ref_arms_t[idx], self.ref_x_t[idx],
                self.ref_z_t[idx], self.ref_pitch_t[idx], self.ref_b_t[idx])

    def _try_capture_graph(self):
        import os as _os
        if _os.environ.get("OMNISIM_NEWTON_NO_GRAPH"):
            return
        try:
            with self.wp.ScopedDevice(self.device):
                for _ in range(SUBSTEPS):
                    self.mjw.step(self.mw_m, self.mw_d)
                self.wp.synchronize(); self.wp.capture_begin(force_module_load=False)
                for _ in range(SUBSTEPS):
                    self.mjw.step(self.mw_m, self.mw_d)
                self._cuda_graph = self.wp.capture_end()
            print(f"[env] CUDA graph captured ({SUBSTEPS} substeps)")
        except Exception as e:
            print(f"[env] CUDA graph capture failed ({e}); direct step"); self._cuda_graph = None

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
        self.ep_step_t[idx] = 0
        # --- RSI: start a fraction of resets at a random phase of the reference,
        # posed (joints + arms + pelvis height + forward x) to that phase, so the
        # policy experiences standing-balance + every transition directly. ---
        if self._rsi_frac > 0.0:
            rsi_sel = torch.rand(m, device=self.tdev) < self._rsi_frac
            mr = int(rsi_sel.sum().item())
            if mr > 0:
                ridx = idx[rsi_sel]
                _lo = self._rsi_lo_frac
                _span = self._rsi_band if self._rsi_band > 0.0 else (1.0 - _lo)
                step0 = ((_lo + torch.rand(mr, device=self.tdev) * _span) * float(MAX_EP)).clamp(0, MAX_EP - 1).to(torch.int32)
                self.ep_step_t[ridx] = step0
                si = step0.clamp(0, MAX_EP - 1).long()
                rl = self.ref_legs_t[si]; ra = self.ref_arms_t[si]       # pose from the ghost table
                bi = ridx.unsqueeze(1).expand(-1, NJ); ci = self.qpos_idx_t.unsqueeze(0).expand(mr, -1)
                self.qpos_t[bi, ci] = rl
                ba = ridx.unsqueeze(1).expand(-1, len(ARM_JOINTS)); ca = self.arm_qpos_t.unsqueeze(0).expand(mr, -1)
                self.qpos_t[ba, ca] = ra
                self.qpos_t[ridx, 0] = self.ref_x_t[si]                  # forward x (over the planted feet)
                self.qpos_t[ridx, 2] = self.ref_z_t[si]                  # pelvis height
                p = self.ref_pitch_t[si]                                 # base lean (quat about y)
                self.qpos_t[ridx, 3] = torch.cos(p * 0.5)
                self.qpos_t[ridx, 4] = 0.0
                self.qpos_t[ridx, 5] = torch.sin(p * 0.5)
                self.qpos_t[ridx, 6] = 0.0
        if self._init_q_band > 0:
            jit = (torch.rand(m, NJ, device=self.tdev) * 2 - 1) * self._init_q_band
            bi = idx.unsqueeze(1).expand(-1, NJ); ci = self.qpos_idx_t.unsqueeze(0).expand(m, -1)
            self.qpos_t[bi, ci] += jit
        self.last_action_t[idx] = 0.0
        # Reset only needs xpos refreshed from the new qpos (qvel=0), NOT a full
        # dynamics/constraint solve. mjw.kinematics is a bit-identical drop-in for
        # mjw.forward here and ~2.5x faster overall (the per-step reset forward was
        # ~60% of train time). Same patch as the walk trainer.
        with self.wp.ScopedDevice(self.device):
            self.mjw.kinematics(self.mw_m, self.mw_d)

    def _reset_all(self):
        self._reset_envs(None)

    def _build_obs_t(self, ref_legs, ref_x, ref_z, ref_pitch, b):
        qp, qv = self.qpos_t, self.qvel_t
        vlin, vang = qv[:, 0:3], qv[:, 3:6]
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        pg = torch.stack([-2 * (x * z - w * y), -2 * (y * z + w * x), -(1 - 2 * (x * x + y * y))], dim=1)
        q = qp.index_select(1, self.qpos_idx_t) - ref_legs            # dev from the ghost ref
        qd = qv.index_select(1, self.qvel_idx_t)
        ahead = (self.ep_step_t + LOOKAHEAD_STEPS).clamp(0, MAX_EP - 1).long()
        b_ahead = self.ref_b_t[ahead]
        z_err = qp[:, 2] - ref_z
        x_err = qp[:, 0] - ref_x
        pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1.0, 1.0))
        pitch_err = pitch - ref_pitch
        phase = torch.stack([b, b_ahead, z_err, x_err, pitch_err], dim=1)
        obs = torch.cat([vlin, vang, pg, q, qd, self.last_action_t, phase], dim=1)
        if self._obs_noise > 0:
            obs = obs + torch.randn_like(obs) * self._obs_noise
        return torch.clamp(torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0), -10.0, 10.0)

    def reset(self):
        self._reset_all()
        rl, ra, rx, rz, rp, b = self._ref(self.ep_step_t)
        return self._build_obs_t(rl, rx, rz, rp, b)

    def step(self, action_t):
        action_t = torch.clamp(action_t, -1.0, 1.0)
        _idx0 = self.ep_step_t.clamp(0, MAX_EP - 1).long()      # ref index for THIS step (vel-tracking)
        ref_legs, ref_arms, ref_x, ref_z, ref_pitch, b = self._ref(self.ep_step_t)
        targets = torch.clamp(ref_legs + self._res_scale * action_t, self.jl_lo_t, self.jl_hi_t)
        self.ctrl_t.zero_()
        self.ctrl_t.index_copy_(1, self.ctrl_pos_idx_t, targets)
        self.ctrl_t.index_copy_(1, self.arm_ctrl_pos_idx_t, ref_arms)
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

        bz = self.qpos_t[:, 2]; bx = self.qpos_t[:, 0]
        w, x, y, z = self.qpos_t[:, 3], self.qpos_t[:, 4], self.qpos_t[:, 5], self.qpos_t[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1.0, 1.0))
        vlin_n = torch.linalg.norm(self.qvel_t[:, 0:3], dim=1)
        vang_n = torch.linalg.norm(self.qvel_t[:, 3:6], dim=1)
        qleg_dev = self.qpos_t.index_select(1, self.qpos_idx_t) - ref_legs

        # ── TRACK THE GHOST ── match the 13 leg+waist joints + base (x, z, lean) to
        # the designed reference. The motion is achievable (CoM over the feet), so
        # tracking it reproduces the designed sit->stand->sit. All the cleverness is
        # in the reference (g1_sitstand); the reward is just "match the ghost".
        r = self.r
        ones = torch.ones(self.n, device=self.tdev)
        r_track = r.get("track", 1.0) * torch.exp(-r.get("track_k", 4.0) * (qleg_dev * qleg_dev).mean(dim=1))
        r_z = r.get("height", 1.0) * torch.exp(-r.get("height_k", 20.0) * (bz - ref_z) ** 2)
        r_x = r.get("xtrack", 0.6) * torch.exp(-r.get("xtrack_k", 15.0) * (bx - ref_x) ** 2)
        r_pitch = r.get("pitchtrack", 0.6) * torch.exp(-r.get("pitchtrack_k", 5.0) * (pitch - ref_pitch) ** 2)
        r_alive = r.get("alive", 0.3) * ones
        r_vel = r.get("vel", -0.01) * (vlin_n + vang_n)
        r_act = r.get("act", -0.005) * (action_t * action_t).sum(dim=1)
        r_rate = r.get("act_rate", -0.02) * ((action_t - prev_action) ** 2).sum(dim=1)
        reward = r_track + r_z + r_x + r_pitch + r_alive + r_vel + r_act + r_rate
        # ── DeepMimic VELOCITY tracking ── match the reference joint velocities + base linear
        # velocity (the missing imitation ingredient: a feasible motion is only learnable if the
        # policy reproduces its VELOCITIES, not just poses -- esp. the upward rise velocity).
        if self.ref_qd_t is not None:
            qd_legs = self.qvel_t.index_select(1, self.qvel_idx_t)
            r_jvel = r.get("jveltrack", 0.0) * torch.exp(
                -r.get("jveltrack_k", 0.1) * ((qd_legs - self.ref_qd_t[_idx0]) ** 2).mean(dim=1))
            r_bvel = r.get("bveltrack", 0.0) * torch.exp(
                -r.get("bveltrack_k", 5.0) * ((self.qvel_t[:, 0:3] - self.ref_bvel_t[_idx0]) ** 2).mean(dim=1))
            reward = reward + r_jvel + r_bvel

        # Blow-up guard (contact DR can make a few envs go non-finite; NaN compares
        # False, so the env would never reset and its NaN would poison the gradient).
        # A "fall" = far off the reference orientation/height -> reset.
        blew = ~(torch.isfinite(bz) & torch.isfinite(roll) & torch.isfinite(pitch))
        fall = ((torch.abs(roll) > ROLL_FAIL) | (bz < ref_z - Z_FALL_MARGIN)
                | (torch.abs(pitch - ref_pitch) > PITCH_DEV_FAIL) | blew)
        done = fall | (self.ep_step_t >= MAX_EP)
        reward = reward + fall.float() * r.get("term", -1.0)
        reward = torch.nan_to_num(reward, nan=0.0, posinf=0.0, neginf=0.0)
        if done.any():
            self._reset_envs(done)
        rl2, ra2, rx2, rz2, rp2, b2 = self._ref(self.ep_step_t)
        return self._build_obs_t(rl2, rx2, rz2, rp2, b2), reward, done, {"bz": bz, "roll": roll, "pitch": pitch}


def main():
    import torch.nn as nn
    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=4096)
    p.add_argument("--iters", type=int, default=1500)
    p.add_argument("--rollout", type=int, default=24)
    p.add_argument("--mjcf", default=str(REPO / "projects/robots/unitree/g1/urdf/g1_sit_kp100.mjcf.xml"))
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save", default=str(REPO / "projects/policies/research/training/runs/gpu_g1_sitstand/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=None)
    p.add_argument("--init-from", default=None)
    p.add_argument("--rsi", type=float, default=0.0,
                   help="fraction of resets that start at a RANDOM reference phase "
                        "(reference-state init; escapes the half-rise/crouch local optimum)")
    p.add_argument("--rsi-curric", type=float, default=0.0,
                   help="REVERSE CURRICULUM: initial RSI start-phase lower bound (e.g. 0.85 = "
                        "start in the last 15%% near the stand), annealed to 0 over --rsi-curric-iters")
    p.add_argument("--rsi-curric-iters", type=float, default=1.0,
                   help="iters over which --rsi-curric anneals to 0 (set ~60%% of --iters)")
    p.add_argument("--rsi-band", type=float, default=0.0,
                   help="CONCENTRATED reverse curriculum: sample RSI start phases in a narrow "
                        "band [lo, lo+band] that sweeps to seated (set ~0.2). 0 = uniform[lo,1].")
    p.add_argument("--res-scale", type=float, default=RES_SCALE,
                   help="residual authority on the leg reference; TIGHTEN (0.15) to force phase-tracking")
    p.add_argument("--track", type=float, default=0.6)
    p.add_argument("--track-k", type=float, default=4.0)
    p.add_argument("--xtrack", type=float, default=0.5)
    p.add_argument("--xtrack-k", type=float, default=6.0)
    p.add_argument("--pitchtrack", type=float, default=0.6,
                   help="reward weight for matching the reference lean (upright while standing)")
    p.add_argument("--pitchtrack-k", type=float, default=5.0)
    p.add_argument("--jveltrack", type=float, default=0.0,
                   help="DeepMimic: reward weight for matching reference JOINT velocities")
    p.add_argument("--jveltrack-k", type=float, default=0.1)
    p.add_argument("--bveltrack", type=float, default=0.0,
                   help="DeepMimic: reward weight for matching reference BASE linear velocity (the rise)")
    p.add_argument("--bveltrack-k", type=float, default=5.0)
    p.add_argument("--height", type=float, default=1.6)
    p.add_argument("--height-k", type=float, default=25.0)
    p.add_argument("--upright", type=float, default=0.6)
    p.add_argument("--upright-k", type=float, default=5.0)
    p.add_argument("--footstack", type=float, default=-3.0,
                   help="penalty weight for the feet being ahead of the pelvis while "
                        "standing (forces rising OVER the feet -> upright, not a bow)")
    p.add_argument("--foot-margin", type=float, default=0.15,
                   help="allowed foot-ahead-of-pelvis distance before the footstack penalty")
    p.add_argument("--stand-bonus", type=float, default=0.6,
                   help="bonus for being fully UP during the stand phase (escapes the half-rise optimum)")
    p.add_argument("--alive", type=float, default=0.3)
    p.add_argument("--vel", type=float, default=-0.02)
    p.add_argument("--act", type=float, default=-0.005)
    p.add_argument("--act-rate", type=float, default=-0.02)
    p.add_argument("--term", type=float, default=-1.0)
    p.add_argument("--sim-dt", type=float, default=0.0)
    p.add_argument("--dr-mass-scale", type=float, default=0.15)
    p.add_argument("--dr-friction-scale", type=float, default=0.3)
    p.add_argument("--dr-damping-scale", type=float, default=0.3)
    p.add_argument("--dr-actuator-kp-scale", type=float, default=0.25)
    p.add_argument("--dr-actuator-kv-scale", type=float, default=0.25)
    p.add_argument("--dr-gravity-scale", type=float, default=0.05)
    p.add_argument("--dr-solref-scale", type=float, default=0.0,
                   help="+-fraction CONTACT solref/solimp jitter -- THE warp<->Newton "
                        "contact-stiffness gap crosser (try 0.4-0.6)")
    p.add_argument("--dr-push-prob", type=float, default=0.01)
    p.add_argument("--dr-push-vmax", type=float, default=0.3)
    p.add_argument("--dr-obs-noise", type=float, default=0.02)
    p.add_argument("--dr-init-q-band", type=float, default=0.06)
    p.add_argument("--dr-seed", type=int, default=0)
    p.add_argument("--no-dr", action="store_true")
    args = p.parse_args()
    if not Path(args.mjcf).exists():
        raise SystemExit(f"MJCF not found: {args.mjcf}")

    reward_cfg = dict(track=args.track, track_k=args.track_k, height=args.height,
                      height_k=args.height_k, xtrack=args.xtrack, xtrack_k=args.xtrack_k,
                      pitchtrack=args.pitchtrack, pitchtrack_k=args.pitchtrack_k,
                      jveltrack=args.jveltrack, jveltrack_k=args.jveltrack_k,
                      bveltrack=args.bveltrack, bveltrack_k=args.bveltrack_k,
                      upright=args.upright, upright_k=args.upright_k,
                      stand_bonus=args.stand_bonus, footstack=args.footstack,
                      foot_margin=args.foot_margin, alive=args.alive, vel=args.vel,
                      act=args.act, act_rate=args.act_rate, term=args.term)
    dr_cfg = {} if args.no_dr else dict(
        mass_scale=args.dr_mass_scale, friction_scale=args.dr_friction_scale,
        damping_scale=args.dr_damping_scale, actuator_kp_scale=args.dr_actuator_kp_scale,
        actuator_kv_scale=args.dr_actuator_kv_scale, gravity_scale=args.dr_gravity_scale,
        push_prob=args.dr_push_prob, push_vmax=args.dr_push_vmax, obs_noise=args.dr_obs_noise,
        init_q_band=args.dr_init_q_band, solref_scale=args.dr_solref_scale, seed=args.dr_seed)
    if dr_cfg:
        print(f"[DR] {dr_cfg}")
    env = BatchedG1SitStandEnv(args.envs, args.mjcf, reward_cfg=reward_cfg,
                               sim_dt=args.sim_dt, dr_cfg=dr_cfg, rsi_frac=args.rsi,
                               res_scale=args.res_scale)
    env._rsi_band = float(args.rsi_band)
    print(f"[sitstand] MAX_EP={MAX_EP} ({T_TOTAL:.1f}s cycle)  SEAT_LEGS={SEAT_LEGS.tolist()}")
    N = args.envs; tdev = env.tdev

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
        ac.load_state_dict(torch.load(args.init_from, map_location=tdev)); print(f"warm-start {args.init_from}")
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)

    if args.eval:
        ac.load_state_dict(torch.load(args.save, map_location=tdev))
        import os as _osL
        # SITSTAND_LAUNCH_OL=N: open-loop the first N steps (zero residual -> targets follow the
        # ghost reference exactly) to clear the dead-seated push-off-the-chair launch, THEN hand
        # to the policy (which tracks well from there). Hybrid feedforward-launch + RL tracking.
        _launch_ol = int(_osL.environ.get("SITSTAND_LAUNCH_OL", "0"))
        obs = env.reset()
        steps = args.eval_steps or MAX_EP
        # per-phase pelvis-height error vs the reference (did it actually rise + sit?)
        zerr = []; tilt = []; bz_tr = []; bx_tr = []; rz_tr = []; alive_tr = []
        for _k in range(steps):
            with torch.no_grad():
                mu, _, _ = ac(obs)
            act = torch.zeros_like(mu) if _k < _launch_ol else mu
            obs, _, done, _ = env.step(act)
            _, _, _, rz, _, _ = env._ref(env.ep_step_t)
            zerr.append((env.qpos_t[:, 2] - rz).abs().mean().item())
            qx, qy = env.qpos_t[:, 4], env.qpos_t[:, 5]
            tilt.append(math.degrees(torch.acos(torch.clamp(1 - 2 * (qx * qx + qy * qy), -1, 1)).mean().item()))
            bz_tr.append(env.qpos_t[:, 2].mean().item())
            bx_tr.append(env.qpos_t[:, 0].mean().item())
            rz_tr.append(rz.mean().item())
            alive_tr.append((1.0 - done.float()).mean().item())
        print(f"[gpu-eval] {args.save} envs={env.n} steps={steps}")
        print(f"  mean |pelvis_z - ref| = {np.mean(zerr):.3f} m   mean tilt = {np.mean(tilt):.1f} deg")
        print(f"  peak pelvis_z = {max(bz_tr):.3f} m   (ref peak = {max(rz_tr):.3f} m)")
        # CLEAN per-phase trace from env 0 ONLY, keyed by its TRUE phase (ep_step_t),
        # so resets don't smear it. Re-run a fresh single trajectory.
        obs = env.reset()
        phase_z = {}; phase_x = {}; phase_knee = {}; phase_tilt = {}
        phase_hip = {}; phase_ankle = {}
        import os as _osE
        _passive = _osE.environ.get("SITSTAND_PASSIVE")  # zero action -> bare reference
        # SITSTAND_VIEW=1: open a LIVE MuJoCo viewer on env-0 -- the pelvis is a FREE
        # body driven by real contact physics (NOT teleported), so you watch the actual
        # simulation that produced the recording. Loops; close the window to stop.
        if _osE.environ.get("SITSTAND_VIEW"):
            import mujoco
            import mujoco.viewer as _mjv
            import time as _tv
            _mjd = mujoco.MjData(env.mjm)
            obs = env.reset()
            _step = 0
            with _mjv.launch_passive(env.mjm, _mjd) as _viewer:
                while _viewer.is_running():
                    _t0 = _tv.time()
                    with torch.no_grad():
                        mu, _, _ = ac(obs)
                    if _passive:
                        mu = torch.zeros_like(mu)
                    obs, _, done, _ = env.step(mu)
                    _mjd.qpos[:] = env.qpos_t[0].detach().cpu().numpy()
                    _mjd.qvel[:] = env.qvel_t[0].detach().cpu().numpy()
                    mujoco.mj_forward(env.mjm, _mjd)
                    _viewer.sync()
                    _step += 1
                    if bool(done[0].item()) or _step >= MAX_EP:
                        _tv.sleep(0.6)            # pause on the final pose so the fall is visible
                        obs = env.reset(); _step = 0
                    _slp = DT - (_tv.time() - _t0)
                    if _slp > 0:
                        _tv.sleep(_slp)
            return
        # SITSTAND_DUMP=<csv>: write env-0's ACHIEVED cycle in the ghost-replay format
        # (x,z,axis-angle rotation + 23 joints) so the ghost can replay the real
        # achievable motion (the achievable-ghost artifact).
        _dump_f = None
        _dump_path = _osE.environ.get("SITSTAND_DUMP")
        if _dump_path:
            _dnames = list(LEGS_JOINTS) + list(ARM_JOINTS)
            _didx = list(env.c2qpos) + list(env.arm_qpos)
            _dump_f = open(_dump_path, "w")
            _dump_f.write("x,z,rx,ry,rz,ra," + ",".join(_dnames) + "\n")
        for _ in range(MAX_EP):
            with torch.no_grad():
                mu, _, _ = ac(obs)
            if _passive:
                mu = torch.zeros_like(mu)
            obs, _, done, _ = env.step(mu)
            if _dump_f is not None:
                qp = env.qpos_t[0]
                w = float(qp[3]); qx = float(qp[4]); qy = float(qp[5]); qz = float(qp[6])
                ang = 2.0 * math.acos(max(-1.0, min(1.0, w)))
                s = math.sqrt(max(1e-9, 1.0 - w * w))
                if s < 1e-6:
                    ax, ay, az, ang = 0.0, 0.0, 1.0, 0.0
                else:
                    ax, ay, az = qx / s, qy / s, qz / s
                jv = [float(env.qpos_t[0, ii]) for ii in _didx]
                _dump_f.write(f"{float(qp[0]):.5f},{float(qp[2]):.5f},{ax:.5f},{ay:.5f},{az:.5f},{ang:.5f},"
                              + ",".join(f"{v:.5f}" for v in jv) + "\n")
            t0 = float(env.ep_step_t[0].item()) * DT
            phase_z[round(t0, 1)] = float(env.qpos_t[0, 2].item())
            phase_x[round(t0, 1)] = float(env.qpos_t[0, 0].item())
            kq = float(env.qpos_t[0, env.qpos_idx_t[3]].item())   # left_knee_joint
            hq = float(env.qpos_t[0, env.qpos_idx_t[0]].item())   # left_hip_pitch_joint
            aq = float(env.qpos_t[0, env.qpos_idx_t[4]].item())   # left_ankle_pitch_joint
            qx0, qy0 = float(env.qpos_t[0, 4].item()), float(env.qpos_t[0, 5].item())
            tl = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qx0 * qx0 + qy0 * qy0)))))
            phase_knee[round(t0, 1)] = kq; phase_tilt[round(t0, 1)] = tl
            phase_hip[round(t0, 1)] = hq; phase_ankle[round(t0, 1)] = aq
        print("  env-0 phase trace (pelvis_z vs ref | hip/knee/ankle rad | torso tilt deg):")
        for ts in [0.0, 0.5, 1.0, 2.0, 3.0, 3.5, 5.0, 6.0, 8.0, 8.5, 10.0, 11.5]:
            key = round(ts, 1)
            if key in phase_z:
                from projects.policies.control.gait.g1_sitstand import ref_pelvis_z as _rz, ref_pelvis_x as _rx
                print(f"    t={ts:4.1f}s  pz={phase_z[key]:.3f}(ref{_rz(ts):.3f}) x={phase_x[key]:+.3f}(ref{_rx(ts):+.2f}) hip={phase_hip[key]:+.2f} knee={phase_knee[key]:.2f} ankle={phase_ankle[key]:+.2f} tilt={phase_tilt[key]:4.1f}")
        if _dump_f is not None:
            _dump_f.close(); print(f"  dumped achieved cycle -> {_dump_path}")
        return

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    rollout = args.rollout
    obs = env.reset(); total_steps = 0; t0 = time.time()
    obs_buf = torch.zeros(rollout, N, OBS_DIM, device=tdev); act_buf = torch.zeros(rollout, N, NJ, device=tdev)
    logp_buf = torch.zeros(rollout, N, device=tdev); rew_buf = torch.zeros(rollout, N, device=tdev)
    done_buf = torch.zeros(rollout, N, device=tdev); val_buf = torch.zeros(rollout, N, device=tdev)
    for it in range(1, args.iters + 1):
        # Reverse curriculum: anneal the RSI start-phase lower bound from CURRIC_HI
        # (start near the stand) to 0 (full range incl. the dead-seated launch) over
        # the first CURRIC_FRAC of training. Off when --rsi-curric 0.
        if args.rsi_curric > 0.0:
            env._rsi_lo_frac = max(0.0, args.rsi_curric * (1.0 - it / max(1.0, args.rsi_curric_iters)))
        for k in range(rollout):
            with torch.no_grad():
                mu, v, log_std = ac(obs)
                dist = torch.distributions.Normal(mu, log_std.exp())
                a = dist.sample(); logp = dist.log_prob(a).sum(-1)
            obs_buf[k] = obs; act_buf[k] = a; logp_buf[k] = logp; val_buf[k] = v
            obs, rr, done, _ = env.step(a); rew_buf[k] = rr; done_buf[k] = done.float(); total_steps += N
        with torch.no_grad():
            _, last_v, _ = ac(obs)
        gamma, lam = 0.99, 0.95
        adv = torch.zeros_like(rew_buf); last_gae = torch.zeros(N, device=tdev)
        for k in reversed(range(rollout)):
            nonterm = 1.0 - done_buf[k]
            nextv = last_v if k == rollout - 1 else val_buf[k + 1]
            delta = rew_buf[k] + gamma * nextv * nonterm - val_buf[k]
            last_gae = delta + gamma * lam * nonterm * last_gae; adv[k] = last_gae
        ret = adv + val_buf; adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        of = obs_buf.reshape(-1, OBS_DIM); af = act_buf.reshape(-1, NJ)
        lf = logp_buf.reshape(-1); advf = adv.reshape(-1); retf = ret.reshape(-1)
        for _e in range(4):
            mu, v, log_std = ac(of)
            dist = torch.distributions.Normal(mu, log_std.exp())
            nlp = dist.log_prob(af).sum(-1); ratio = (nlp - lf).exp()
            s1 = ratio * advf; s2 = torch.clamp(ratio, 0.8, 1.2) * advf
            loss = -torch.min(s1, s2).mean() + 0.5 * ((v - retf) ** 2).mean() - 0.01 * dist.entropy().sum(-1).mean()
            opt.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5); opt.step()
        if it % 10 == 0 or it == 1:
            fps = total_steps / max(time.time() - t0, 1e-6)
            print(f"it {it:4d}  ep_rew/step~{rew_buf.mean().item():+.3f}  meanV {val_buf.mean().item():+.2f}  "
                  f"steps {total_steps:,}  {fps:,.0f} env-steps/s")

    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total_steps:,} steps in {time.time() - t0:.1f}s)")
    onnx_path = Path(args.save).with_suffix(".onnx")

    class DeployPolicy(torch.nn.Module):
        def __init__(self, pi):
            super().__init__(); self.pi = pi

        def forward(self, obs):
            return torch.tanh(self.pi(obs))

    cpu_ac = AC(); cpu_ac.load_state_dict({k: v.cpu() for k, v in ac.state_dict().items()})
    wrapped = DeployPolicy(cpu_ac.pi); wrapped.eval()
    try:
        import io, contextlib
        with contextlib.redirect_stdout(io.StringIO()):
            torch.onnx.export(wrapped, torch.zeros(1, OBS_DIM), str(onnx_path),
                              input_names=["obs"], output_names=["action"],
                              dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}}, opset_version=17)
        print(f"exported ONNX -> {onnx_path}")
    except Exception as e:
        print(f"ONNX export failed (non-fatal): {e}")


if __name__ == "__main__":
    main()
