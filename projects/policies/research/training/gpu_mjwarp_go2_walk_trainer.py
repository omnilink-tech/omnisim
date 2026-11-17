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

"""GPU mujoco_warp trainer for the Unitree Go2 WALKING policy.

THE GO2 PORT OF gpu_mjwarp_omniquad_walk_trainer.py (itself the OmniQuad port of the
canonical G1 sim-to-deploy template). Identical architecture -- an open-loop
FOOT-SPACE TROT reference (projects/policies/control/gait/go2_trot_gait.py) + a 12-DOF
residual policy trained with PPO on mujoco_warp against the EXACT
deploy-matched MJCF. Only the gait module, the Go2 joint limits, and the
gait-param defaults differ from the OmniQuad trainer. Go2 deploy-matched
actuators: kp=80/kv=2.0 (OMNISIM_NEWTON_TARGET_KE=80/KD=2.0 -- Go2's
hip/thigh motors are 23.7 Nm vs OmniQuad's 80, so the servo is softer).

The G1 lessons baked in:
  1. GAINS PARITY: train on the OMNISIM_NEWTON_SAVE_MJCF dump of the
     deploy world; kp/kv diffed and matched before any training.
  2. Mid-stride resets (--seed-gait-pose: q AND qd on the reference) +
     --dr-init-vx-bias so episodes practise SUSTAINING the walk; REST-START
     mixing (--rest-start-frac) so the deploy's standing launch is
     in-distribution; the clock starts at QS_PHASE (all four feet down).
  3. Foot-aware rewards (--rw-sched / --rw-slip via xpos+xquat of the
     lower-leg bodies): stance foot down, swing foot lifted, no skating.
  4. --vel-l1 supplies a velocity gradient where the gaussian saturates.
  5. Entropy anneal + --log-std-clamp for late chunks.
  6. Gentle DR (the heavy stand profile blocks gait learning).
  7. Honest eval: per-env FIRST-fall metrics, distance integrated only
     while alive (auto-resets can't inflate it).
  8. ONNX exports with CLAMP(-1,1) -- exactly the training squash (the
     tanh wrap was a silent 24% mid-range weakening on G1).
  9. Deploy-faithful qd obs: finite-difference of q over the control
     period (the deploy reads position sensors, not physics qvel).

Select checkpoints by DEPLOY samples, not trainer eval (G1 lesson #6).

Usage (chunked warm-starts, ~400 iters x 4096 envs per chunk):
    python projects/policies/research/training/gpu_mjwarp_omniquad_walk_trainer.py \\
        --envs 4096 --iters 400 --rollout 12 \\
        --seed-gait-pose --rest-start-frac 0.25 --dr-init-vx-bias 0.3 \\
        --rw-sched -5 --rw-slip -0.5 --vel-l1 -0.3 \\
        --save projects/policies/research/training/runs/gpu_omniquad_walk1/policy.pt
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

from projects.policies.control.gait import go2_trot_gait as stg  # noqa: E402
from projects.policies.research.shadowing.hill_terrain import RoughProfile  # noqa: E402

# ── Layout constants ────────────────────────────────────────────────
NJ = 12            # 4 legs x (hip_x, hip_y, knee)
QPOS_J0 = 7
QVEL_J0 = 6
OBS_DIM = 48       # vlin(3)+vang(3)+proj_g(3)+q(12)+qd(12)+last_action(12)+gait(2)+wz_cmd(1)
RES_SCALE = 0.15   # rad joint-space residual authority (kp500 is stiff)
DT = 0.016         # OmniSim basicTimeStep
SUBSTEPS = 8       # 8 x 2 ms = 16 ms -- matches OMNISIM_NEWTON_SUBSTEPS=8
PHYS_DT = 0.002

# Episode termination.
ROLL_FAIL = 0.8
PITCH_FAIL = 0.8
BZ_FAIL = 0.18
MAX_EP = 750       # 12 s default; raise via --max-ep for late chunks

# MJCF joint ranges (Go2 deploy dump). Controller order FL,FR,RL,RR x
# (hip, thigh, calf). thigh lower uses the tighter rear bound (-0.5236);
# the trot thigh stays in [0.55,1.05] so this never clips the gait.
JOINT_LIMITS_LO = np.array([-1.0472, -0.5236, -2.7227] * 4, dtype=np.float32)
JOINT_LIMITS_HI = np.array([+1.0472, +3.1316, -0.83776] * 4, dtype=np.float32)

LOWER_LEG_BODIES = (4, 7, 10, 13)   # FL,FR,RL,RR calf bodies (Go2 == OmniQuad nesting)
FOOT_OFFSET = (0.0, 0.0, -stg.L2)   # foot tip in the lower-leg frame


class RoughRef:
    """Blind rough-terrain ground_h (self-contained): inject random bump bars
    into the MJCF and expose ground_h(xq) so reward/fall-test are ground-relative.
    No ghost, no obs change."""

    def __init__(self, src_mjcf, out_mjcf, device, amp=0.06, seed=0,
                 x_start=2.0, x_end=40.0):
        self.rp = RoughProfile(amp=amp, seed=seed, x_start=x_start, x_end=x_end)
        self.rp.inject_mjcf(src_mjcf, out_mjcf)
        self.mjcf = out_mjcf
        self.gx = torch.tensor(self.rp._gx, device=device)
        self.gh = torch.tensor(self.rp._gh, device=device)

    @staticmethod
    def _interp(xq, xs, ys):
        xq = torch.clamp(xq, float(xs[0]), float(xs[-1]))
        idx = torch.searchsorted(xs, xq).clamp(1, xs.shape[0] - 1)
        x0 = xs[idx - 1]
        f = (xq - x0) / (xs[idx] - x0).clamp(min=1e-6)
        return ys[idx - 1] + f * (ys[idx] - ys[idx - 1])

    def ground_h(self, xq):
        return RoughRef._interp(xq, self.gx, self.gh)


class BatchedSpotWalkEnv:
    """Trot-reference + residual batched env, all-GPU (torch views of
    mujoco_warp buffers, CUDA-graph substeps)."""

    def __init__(self, n, mjcf, device="cuda:0", reward_cfg=None, sim_dt=0.0,
                 dr_cfg=None, rough_cfg=None):
        import warp as wp
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mjw = wp, mjw
        self.n = n
        self.device = wp.get_device(device)
        self.dr = dr_cfg or {}

        self.terrain = None
        if rough_cfg:
            _tdev = torch.device("cuda:0" if "cuda" in str(self.device).lower() else "cpu")
            rough_mjcf = str(Path(mjcf).with_name(Path(mjcf).stem + "_rough.xml"))
            self.terrain = RoughRef(mjcf, rough_mjcf, _tdev,
                                    amp=float(rough_cfg.get("amp", 0.06)),
                                    seed=int(rough_cfg.get("seed", 0)),
                                    x_start=float(rough_cfg.get("x_start", 2.0)),
                                    x_end=float(rough_cfg.get("x_end", 40.0)))
            mjcf = self.terrain.mjcf
            print(f"[rough] injected {len(self.terrain.rp.bars)} bump bars "
                  f"(amp={float(rough_cfg.get('amp', 0.06)):.3f} m) -> {mjcf}")

        self.mjm = mujoco.MjModel.from_xml_path(mjcf)
        self.mjm.opt.timestep = float(sim_dt) if sim_dt and sim_dt > 0 else PHYS_DT

        # Per-run model-space DR (mujoco_warp can't do per-env params).
        rng = np.random.default_rng(self.dr.get("seed", 0))
        if self.dr.get("mass_scale", 0.0) > 0:
            b = self.dr["mass_scale"]
            s = rng.uniform(1 - b, 1 + b, self.mjm.body_mass.shape).astype(np.float32)
            self.mjm.body_mass[:] *= s
            self.mjm.body_inertia[:] *= s[:, None]
        if self.dr.get("friction_scale", 0.0) > 0:
            self.mjm.geom_friction[:, 0] *= float(
                rng.uniform(1 - self.dr["friction_scale"], 1 + self.dr["friction_scale"]))
        if self.dr.get("actuator_kp_scale", 0.0) or self.dr.get("actuator_kv_scale", 0.0):
            for ai in range(self.mjm.nu):
                gp = self.mjm.actuator_gainprm[ai]
                bp = self.mjm.actuator_biasprm[ai]
                if abs(bp[1]) > 1e-6 and self.dr.get("actuator_kp_scale", 0.0) > 0:
                    s = float(rng.uniform(1 - self.dr["actuator_kp_scale"],
                                          1 + self.dr["actuator_kp_scale"]))
                    gp[0] *= s; bp[1] *= s
                elif abs(bp[2]) > 1e-6 and self.dr.get("actuator_kv_scale", 0.0) > 0:
                    s = float(rng.uniform(1 - self.dr["actuator_kv_scale"],
                                          1 + self.dr["actuator_kv_scale"]))
                    gp[0] *= s; bp[2] *= s
        if self.dr.get("gravity_scale", 0.0) > 0:
            self.mjm.opt.gravity[2] *= float(
                rng.uniform(1 - self.dr["gravity_scale"], 1 + self.dr["gravity_scale"]))

        mjd = mujoco.MjData(self.mjm)
        mujoco.mj_forward(self.mjm, mjd)
        with wp.ScopedDevice(self.device):
            self.mw_m = mjw.put_model(self.mjm)
            import os as _njos
            _njmax = int(_njos.environ.get("OMNIQUAD_NJMAX", "256"))
            _nconmax = int(_njos.environ.get("OMNIQUAD_NCONMAX", "256"))
            self.mw_d = mjw.put_data(self.mjm, mjd, nworld=n,
                                     njmax=_njmax, nconmax=_nconmax)

        # ── Joint mapping: classify MJCF hinges by body position + axis
        # (names in the dump are anonymous joint_N). Controller order
        # FL,FR,RL,RR x (hip_x, hip_y, knee).
        self.controller_to_qpos = np.zeros(NJ, dtype=np.int32)
        self.controller_to_qvel = np.zeros(NJ, dtype=np.int32)
        self.controller_to_ctrl_pos = np.zeros(NJ, dtype=np.int32)
        mjcf_to_label = {}
        k_hinge = 0
        for j in range(self.mjm.njnt):
            if self.mjm.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                continue
            ax = self.mjm.jnt_axis[j]
            rng_j = self.mjm.jnt_range[j]
            pos = mjd.xpos[self.mjm.jnt_bodyid[j]]
            leg = ("FL" if (pos[0] > 0 and pos[1] > 0) else
                   "FR" if (pos[0] > 0) else
                   "RL" if (pos[1] > 0) else "RR")
            # Axis test FIRST (hip_x is the only X hinge); among the two Y
            # hinges the calf/knee is the one whose UPPER bound is negative
            # ([-2.72,-0.84]) while the thigh upper is +3.13. (Go2's FRONT
            # thigh lower is -1.57, so a lower-bound test misclassifies it.)
            joint = ("hip_x" if abs(ax[0]) > 0.5 else
                     "knee" if rng_j[1] < 0.0 else "hip_y")
            mjcf_to_label[(leg, joint)] = (self.mjm.jnt_qposadr[j],
                                           self.mjm.jnt_dofadr[j], k_hinge)
            k_hinge += 1
        for ci, leg in enumerate(("FL", "FR", "RL", "RR")):
            for cj, joint in enumerate(("hip_x", "hip_y", "knee")):
                qadr, vadr, hidx = mjcf_to_label[(leg, joint)]
                i = ci * 3 + cj
                self.controller_to_qpos[i] = qadr
                self.controller_to_qvel[i] = vadr
                # ctrl is interleaved [pos, vel] per hinge in hinge order.
                self.controller_to_ctrl_pos[i] = 2 * hidx
        assert self.mjm.nu == 2 * NJ, f"expected 24 actuators, got {self.mjm.nu}"

        self.r = reward_cfg or {}
        gpd = self.r.get("gait_params", {}) or {}
        self.gp = stg.GaitParams(**gpd)
        self.nominal = stg.standing_pose(self.gp).astype(np.float32)
        self.vx_target = self.gp.vx
        # VELOCITY CONDITIONING (the G1 walk29_vc recipe, ported). When
        # vx_cmd_max>0 the policy is told a target speed (sampled in
        # [0,vx_cmd_max], INCLUDING 0=stand) appended to the obs; the gait
        # amplitude + swing weights + velocity reward all scale with
        # s=vx_cmd/vx_target. At vx_cmd=0 the trot collapses to the standing
        # pose -- statically stable for a quadruped. Obs gains +1 dim.
        self.vx_cmd_max = float(self.r.get("vx_cmd_max", 0.0))
        self.vx_cond = self.vx_cmd_max > 0.0
        self.obs_dim = OBS_DIM + (1 if self.vx_cond else 0)
        self.phase_dt = 2.0 * math.pi * self.gp.freq * DT
        self.max_ep = int(self.r.get("max_ep", MAX_EP))
        self.res_scale = float(self.r.get("res_scale", RES_SCALE))
        self.seed_gait = bool(self.r.get("seed_gait", False))
        self.rest_start_frac = float(self.r.get("rest_start_frac", 0.0))
        self.wz_range = float(self.r.get("wz_range", 0.0))
        self.rw_sched = float(self.r.get("rw_sched", 0.0))
        self.rw_slip = float(self.r.get("rw_slip", 0.0))
        self.foot_z_contact = float(self.r.get("foot_z_contact", 0.05))
        self.foot_z_swing = float(self.r.get("foot_z_swing", 0.07))

        # Seed pose: standing pose, feet on the ground (+1 cm settle gap).
        self.spawn_z = self.gp.body_height + 0.03
        self.seed_qpos = mjd.qpos.copy().astype(np.float32)
        self.seed_qpos[0:3] = [0.0, 0.0, self.spawn_z]
        self.seed_qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for i in range(NJ):
            self.seed_qpos[self.controller_to_qpos[i]] = self.nominal[i]
        self.nq, self.nv, self.nu = self.mjm.nq, self.mjm.nv, self.mjm.nu

        # ── GPU-native state ──
        self.tdev = torch.device("cuda:0" if "cuda" in str(self.device).lower()
                                 else "cpu")
        self.qpos_t = wp.to_torch(self.mw_d.qpos).view(n, self.nq)
        self.qvel_t = wp.to_torch(self.mw_d.qvel).view(n, self.nv)
        self.ctrl_t = wp.to_torch(self.mw_d.ctrl).view(n, self.nu)
        self.xpos_t = wp.to_torch(self.mw_d.xpos).view(n, self.mjm.nbody, 3)
        self.xquat_t = wp.to_torch(self.mw_d.xquat).view(n, self.mjm.nbody, 4)

        self.nominal_t = torch.tensor(self.nominal, device=self.tdev)
        self.jl_lo_t = torch.tensor(JOINT_LIMITS_LO, device=self.tdev)
        self.jl_hi_t = torch.tensor(JOINT_LIMITS_HI, device=self.tdev)
        self.qpos_idx_t = torch.tensor(self.controller_to_qpos, dtype=torch.long, device=self.tdev)
        self.qvel_idx_t = torch.tensor(self.controller_to_qvel, dtype=torch.long, device=self.tdev)
        self.ctrl_pos_idx_t = torch.tensor(self.controller_to_ctrl_pos, dtype=torch.long, device=self.tdev)
        self.seed_qpos_t = torch.tensor(self.seed_qpos, device=self.tdev)
        self.foot_off_t = torch.tensor(FOOT_OFFSET, device=self.tdev)

        self.ep_step_t = torch.zeros(n, dtype=torch.int32, device=self.tdev)
        self.wz_cmd_t = torch.zeros(n, device=self.tdev)
        self.vx_cmd_t = torch.full((n,), self.vx_target, device=self.tdev)
        self.last_action_t = torch.zeros(n, NJ, device=self.tdev)
        self.phase_t = torch.zeros(n, device=self.tdev)
        self._ramp_t0 = torch.zeros(n, device=self.tdev)
        self._prev_q = torch.zeros(n, NJ, device=self.tdev)
        self.prev_foot_xy_t = torch.zeros(n, 4, 2, device=self.tdev)

        self.max_latency_ticks = int(self.dr.get("action_latency_max", 0))
        self.action_buffer_t = torch.zeros(
            n, max(1, self.max_latency_ticks + 1), NJ, device=self.tdev)
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

    def _foot_pos_t(self):
        """World foot-tip positions (n, 4, 3) from the lower-leg bodies'
        xpos + xquat (foot tip is (0,0,-L2) in the lower-leg frame)."""
        p = self.xpos_t[:, LOWER_LEG_BODIES, :]            # (n,4,3)
        q = self.xquat_t[:, LOWER_LEG_BODIES, :]           # (n,4,4) wxyz
        w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
        ox, oy, oz = FOOT_OFFSET
        # quat rotate (0,0,oz): standard expansion with vx=vy=0.
        rx = oz * (2 * (x * z + w * y))
        ry = oz * (2 * (y * z - w * x))
        rz = oz * (1 - 2 * (x * x + y * y))
        return p + torch.stack([rx, ry, rz], dim=-1)

    def _sample_vx_cmd(self, m):
        """Sample m forward-speed commands in [0, vx_cmd_max]. The milestone
        needs two speeds to be ROBUST -- 0 (stand) and vx_target (nominal
        walk) -- so concentrate mass there and spread the rest uniformly
        (the G1 _sample_vx_cmd, verbatim mix):
          ~30% exactly vx_target (preserve the warm-started walk),
          ~28% exactly 0        (a solid stand),
          ~42% uniform[0,vx_cmd_max].
        """
        u = torch.rand(m, device=self.tdev)
        vc = torch.rand(m, device=self.tdev) * self.vx_cmd_max
        vc = torch.where(u < 0.30, torch.full_like(vc, self.vx_target), vc)
        vc = torch.where((u >= 0.30) & (u < 0.58), torch.zeros_like(vc), vc)
        return vc

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
        if self._init_xy_band > 0:
            self.qpos_t[idx, 0] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_xy_band
            self.qpos_t[idx, 1] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_xy_band
        if self._init_z_band > 0:
            self.qpos_t[idx, 2] += (torch.rand(m, device=self.tdev) * 2 - 1) * self._init_z_band
        tilt = float(self.dr.get("init_tilt_band", 0.0))
        if tilt > 0:
            hr = (torch.rand(m, device=self.tdev) * 2 - 1) * (tilt * 0.5)
            hp = (torch.rand(m, device=self.tdev) * 2 - 1) * (tilt * 0.5)
            cr, sr = torch.cos(hr), torch.sin(hr)
            cp, sp = torch.cos(hp), torch.sin(hp)
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
        if self.vx_cond:
            self.vx_cmd_t[idx] = self._sample_vx_cmd(m)
        # Per-episode yaw-rate command (uniform +-wz_range, 1/3 pinned 0 so
        # straight walking stays well-covered).
        if self.wz_range > 0:
            self.wz_cmd_t[idx] = (torch.rand(m, device=self.tdev) * 2 - 1) * self.wz_range
            zero = torch.rand(m, device=self.tdev) < 0.34
            self.wz_cmd_t[idx[zero]] = 0.0
        # Random gait phase per env (decorrelates the batch).
        self.phase_t[idx] = torch.rand(m, device=self.tdev) * (2.0 * math.pi)
        self._ramp_t0[idx] = 1e6          # mid-stride: full stride

        if self.seed_gait:
            # Mid-stride seeding: q from the reference at the sampled phase,
            # qd by finite difference of the reference over one tick.
            th = self.phase_t[idx]
            full = torch.full((m,), 1e6, device=self.tdev)
            legs0, _ = stg.targets_torch(th, self.gp, full)
            legs1, _ = stg.targets_torch(th + self.phase_dt, self.gp, full)
            qd_ref = (legs1 - legs0) / DT
            bi = idx.unsqueeze(1).expand(-1, NJ)
            self.qpos_t[bi, self.qpos_idx_t.unsqueeze(0).expand(m, -1)] = legs0
            self.qvel_t[bi, self.qvel_idx_t.unsqueeze(0).expand(m, -1)] = qd_ref

        # REST-START mixing: the deploy handover -- standing pose, zero
        # velocity, clock at QS_PHASE (all four feet down), stride ramps in.
        if self.rest_start_frac > 0:
            rest = torch.rand(m, device=self.tdev) < self.rest_start_frac
            ridx = idx[rest]
            if ridx.numel() > 0:
                rm = ridx.shape[0]
                self.qpos_t[ridx] = self.seed_qpos_t.unsqueeze(0).expand(rm, -1)
                self.qvel_t[ridx] = 0.0
                rj = (torch.rand(rm, NJ, device=self.tdev) * 2 - 1) * 0.03
                ri = ridx.unsqueeze(1).expand(-1, NJ)
                rc = self.qpos_idx_t.unsqueeze(0).expand(rm, -1)
                self.qpos_t[ri, rc] += rj
                hr = (torch.rand(rm, device=self.tdev) * 2 - 1) * 0.01
                hp2 = (torch.rand(rm, device=self.tdev) * 2 - 1) * 0.01
                cr, sr = torch.cos(hr), torch.sin(hr)
                cp2, sp2 = torch.cos(hp2), torch.sin(hp2)
                self.qpos_t[ridx, 3] = cr * cp2
                self.qpos_t[ridx, 4] = sr * cp2
                self.qpos_t[ridx, 5] = cr * sp2
                self.qpos_t[ridx, 6] = sr * sp2
                self.qvel_t[ridx, 0:6] += (torch.rand(rm, 6, device=self.tdev) * 2 - 1) * 0.05
                self.phase_t[ridx] = stg.QS_PHASE
                self._ramp_t0[ridx] = 0.0

        self.action_buffer_t[idx] = 0.0
        if self.max_latency_ticks > 0:
            self.action_delay_t[idx] = torch.randint(
                0, self.max_latency_ticks + 1, (m,), dtype=torch.long, device=self.tdev)

        with self.wp.ScopedDevice(self.device):
            self.mjw.forward(self.mw_m, self.mw_d)
        self._prev_q[idx] = self.qpos_t[idx].index_select(1, self.qpos_idx_t)
        feet = self._foot_pos_t()
        self.prev_foot_xy_t[idx] = feet[idx, :, 0:2]

    def _reset_all(self):
        self._reset_envs(env_mask=None)

    def _build_obs_t(self):
        qp = self.qpos_t
        qv = self.qvel_t
        vlin = qv[:, 0:3]
        vang = qv[:, 3:6]
        w, x, y, z = qp[:, 3], qp[:, 4], qp[:, 5], qp[:, 6]
        gx = -2 * (x * z - w * y)
        gy = -2 * (y * z + w * x)
        gz = -(1 - 2 * (x * x + y * y))
        pg = torch.stack([gx, gy, gz], dim=1)
        q = qp.index_select(1, self.qpos_idx_t)
        # Deploy-faithful qd: finite difference over the control period
        # (the deploy reads 16 ms position-sensor samples, not qvel).
        qd = (q - self._prev_q) / DT
        self._prev_q = q.clone()
        q_c = q - self.nominal_t.unsqueeze(0)
        gait = torch.stack([torch.sin(self.phase_t), torch.cos(self.phase_t)], dim=1)
        obs = torch.cat([vlin, vang, pg, q_c, qd, self.last_action_t, gait,
                         self.wz_cmd_t.unsqueeze(1)], dim=1)
        if self.vx_cond:
            # Normalised speed command as the LAST obs element (matches deploy).
            obs = torch.cat(
                [obs, (self.vx_cmd_t / self.vx_cmd_max).unsqueeze(1)], dim=1)
        if self._obs_noise > 0:
            obs = obs + torch.randn_like(obs) * self._obs_noise
        obs = torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        return torch.clamp(obs, -10.0, 10.0)

    def _baseline_targets_t(self):
        """The pure trot-model reference at the current phase (with the
        per-env stride ramp). Returns (targets (n,12), swings (n,4))."""
        t_since = self._ramp_t0 + self.ep_step_t.to(torch.float32) * DT
        legs, swings = stg.targets_torch(self.phase_t, self.gp,
                                         t_since_start_t=t_since,
                                         wz_t=self.wz_cmd_t)
        # VELOCITY CONDITIONING: blend the model toward the standing pose by
        # s=vx_cmd/vx_target (s=0 -> feet planted, no stride; s=1 -> full
        # trot). Swing weights scale too so the foot rewards expect no
        # stepping at a stand command. Same two lines as the G1 trainer.
        if self.vx_cond:
            s = torch.clamp(self.vx_cmd_t / max(self.vx_target, 1e-3),
                            0.0, 1.25).unsqueeze(1)
            legs = self.nominal_t.unsqueeze(0) + s * (legs - self.nominal_t.unsqueeze(0))
            swings = swings * s          # s is (n,1), broadcasts over the 4 legs
        return legs, swings

    def reset(self):
        self._reset_all()
        return self._build_obs_t()

    def step(self, action_t):
        action_t = torch.clamp(action_t, -1.0, 1.0)
        if self.max_latency_ticks > 0:
            self.action_buffer_t = torch.roll(self.action_buffer_t, 1, dims=1)
            self.action_buffer_t[:, 0, :] = action_t
            row = torch.arange(self.n, device=self.tdev)
            applied = self.action_buffer_t[row, self.action_delay_t]
        else:
            applied = action_t

        baseline, swings = self._baseline_targets_t()
        targets = torch.clamp(baseline + self.res_scale * applied,
                              self.jl_lo_t, self.jl_hi_t)
        self.ctrl_t.zero_()
        self.ctrl_t.index_copy_(1, self.ctrl_pos_idx_t, targets)

        if self._push_p > 0 and self._push_vmax > 0:
            hit = torch.rand(self.n, device=self.tdev) < self._push_p
            if hit.any():
                theta = torch.rand(self.n, device=self.tdev) * (2 * math.pi)
                mag = torch.rand(self.n, device=self.tdev) * self._push_vmax
                self.qvel_t[hit, 0] = self.qvel_t[hit, 0] + (torch.cos(theta) * mag)[hit]
                self.qvel_t[hit, 1] = self.qvel_t[hit, 1] + (torch.sin(theta) * mag)[hit]

        with self.wp.ScopedDevice(self.device):
            if self._cuda_graph is not None:
                self.wp.capture_launch(self._cuda_graph)
            else:
                for _ in range(SUBSTEPS):
                    self.mjw.step(self.mw_m, self.mw_d)

        self.ep_step_t = self.ep_step_t + 1
        prev_action_t = self.last_action_t
        self.last_action_t = action_t
        self.phase_t = torch.remainder(self.phase_t + self.phase_dt, 2.0 * math.pi)

        bz = self.qpos_t[:, 2]
        w, x = self.qpos_t[:, 3], self.qpos_t[:, 4]
        y, z = self.qpos_t[:, 5], self.qpos_t[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1.0, 1.0))

        r = self.r
        vx = self.qvel_t[:, 0]
        vy = self.qvel_t[:, 1]
        wz = self.qvel_t[:, 5]
        r_alive = r.get("alive", 1.0) * torch.ones(self.n, device=self.tdev)
        upright = torch.clamp(1.0 - roll * roll - pitch * pitch, min=0.0)
        r_up = r.get("upright", 0.5) * upright
        vsig = r.get("vel_sigma", 0.10)
        # Track the COMMANDED speed when velocity-conditioned (vx_cmd=0 -> the
        # reward pulls vx to 0 = a stand), else the fixed nominal.
        _vtgt = self.vx_cmd_t if self.vx_cond else self.vx_target
        r_vel = r.get("vel", 2.0) * torch.exp(-((vx - _vtgt) ** 2) / vsig)
        r_vel = r_vel + r.get("vel_l1", 0.0) * torch.abs(vx - _vtgt)
        r_lat = r.get("lat", -0.5) * torch.abs(vy)
        r_yaw = r.get("yaw", -0.5) * torch.abs(wz - self.wz_cmd_t)
        if self.terrain is not None:
            grnd = self.terrain.ground_h(self.qpos_t[:, 0])
        else:
            grnd = torch.zeros_like(bz)
        z_ref = grnd + r.get("z_ref", self.gp.body_height - 0.02)
        r_height = r.get("height", -10.0) * (bz - z_ref) ** 2
        r_act = r.get("act", -0.005) * (action_t * action_t).sum(dim=1)
        r_rate = r.get("act_rate", -0.01) * ((action_t - prev_action_t) ** 2).sum(dim=1)
        reward = r_alive + r_up + r_vel + r_lat + r_yaw + r_height + r_act + r_rate

        if self.rw_sched != 0.0 or self.rw_slip != 0.0:
            feet = self._foot_pos_t()                      # (n,4,3)
            foot_z = feet[:, :, 2]
            foot_xy = feet[:, :, 0:2]
            if self.terrain is not None:
                foot_z = foot_z - self.terrain.ground_h(
                    foot_xy[:, :, 0].reshape(-1)).reshape(self.n, 4)
            stance = 1.0 - swings
            if self.rw_sched != 0.0:
                z_hi = torch.clamp(foot_z - self.foot_z_contact, min=0.0)
                z_lo = torch.clamp(self.foot_z_swing - foot_z, min=0.0)
                reward = reward + self.rw_sched * (stance * z_hi + swings * z_lo).sum(dim=1)
            if self.rw_slip != 0.0:
                v_xy = (foot_xy - self.prev_foot_xy_t) / DT
                contact = (foot_z < self.foot_z_contact).float()
                reward = reward + self.rw_slip * (
                    contact * torch.linalg.norm(v_xy, dim=2)).sum(dim=1)
            self.prev_foot_xy_t = foot_xy.clone()

        fall = (torch.abs(roll) > ROLL_FAIL) | (torch.abs(pitch) > PITCH_FAIL) | ((bz - grnd) < BZ_FAIL)
        done = fall | (self.ep_step_t >= self.max_ep)
        reward = reward + fall.float() * r.get("term", -1.0)

        if done.any():
            self._reset_envs(env_mask=done)
        obs = self._build_obs_t()
        return obs, reward, done, {"bz": bz, "roll": roll, "pitch": pitch}


# ────────────────────────────────────────────────────────────────────
# PPO loop -- the G1 walk trainer's, verbatim shape.
# ────────────────────────────────────────────────────────────────────
def main():
    import torch
    import torch.nn as nn

    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=4096)
    p.add_argument("--iters", type=int, default=400)
    p.add_argument("--rollout", type=int, default=12)
    p.add_argument("--mjcf", default=r"C:\tmp\go2_newton.xml")
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save",
                   default=str(REPO / "projects/policies/research/training/runs/gpu_go2_walk/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=2500)
    p.add_argument("--bare", action="store_true",
                   help="eval the BARE gait model (zero residual) -- the G1 "
                        "bare-CPG takeoff check")
    p.add_argument("--alive", type=float, default=1.0)
    p.add_argument("--upright", type=float, default=0.5)
    p.add_argument("--act", type=float, default=-0.005)
    p.add_argument("--act-rate", type=float, default=-0.01)
    p.add_argument("--term", type=float, default=-10.0)
    p.add_argument("--vel", type=float, default=2.0)
    p.add_argument("--vel-sigma", type=float, default=0.10)
    p.add_argument("--vel-l1", type=float, default=0.0)
    p.add_argument("--lat", type=float, default=-0.5)
    p.add_argument("--yaw", type=float, default=-0.5)
    p.add_argument("--height", type=float, default=-10.0)
    p.add_argument("--z-ref", type=float, default=0.0,
                   help="target body height (0 = body_height - 0.02)")
    p.add_argument("--max-ep", type=int, default=MAX_EP)
    p.add_argument("--res-scale", type=float, default=RES_SCALE,
                   help="residual rad. Deploy MUST set OMNIQUAD_ACT_SCALE to match.")
    p.add_argument("--seed-gait-pose", action="store_true")
    p.add_argument("--rest-start-frac", type=float, default=0.0)
    p.add_argument("--wz-range", type=float, default=0.0,
                   help="per-episode yaw-rate command band rad/s (try 0.3); "
                        "obs[-1] carries the command, the gait model steers "
                        "by tangential stance sweep, the yaw reward tracks it")
    p.add_argument("--rw-sched", type=float, default=0.0)
    p.add_argument("--rw-slip", type=float, default=0.0)
    p.add_argument("--foot-z-contact", type=float, default=0.05)
    p.add_argument("--foot-z-swing", type=float, default=0.07)
    # gait model knobs (deploy mirrors via OMNIQUAD_GAIT_*)
    p.add_argument("--vx-target", type=float, default=0.4)
    p.add_argument("--vx-cmd-max", type=float, default=0.0,
                   help="VELOCITY CONDITIONING: if >0, a per-env forward-speed "
                        "command (sampled in [0,vx-cmd-max], INCLUDING 0=stand) "
                        "is appended to the obs (obs+1) and scales the trot "
                        "model toward the standing pose. One policy that walks "
                        "AND stops on command. Set ~vx-target (e.g. 0.45).")
    p.add_argument("--gait-freq", type=float, default=1.8)
    p.add_argument("--gait-duty", type=float, default=0.6)
    p.add_argument("--gait-step-height", type=float, default=0.05)
    p.add_argument("--gait-body-h", type=float, default=0.30)
    p.add_argument("--gait-x0", type=float, default=0.0)
    p.add_argument("--gait-ramp-s", type=float, default=1.0)
    p.add_argument("--sim-dt", type=float, default=0.0)
    p.add_argument("--init-from", default=None)
    p.add_argument("--ent-coef", type=float, default=0.01)
    p.add_argument("--log-std-clamp", type=float, default=None)
    # Gentle DR -- the G1 walk15 profile.
    p.add_argument("--dr-mass-scale", type=float, default=0.10)
    p.add_argument("--dr-friction-scale", type=float, default=0.20)
    p.add_argument("--dr-damping-scale", type=float, default=0.0)
    p.add_argument("--dr-actuator-kp-scale", type=float, default=0.15)
    p.add_argument("--dr-actuator-kv-scale", type=float, default=0.15)
    p.add_argument("--dr-gravity-scale", type=float, default=0.03)
    p.add_argument("--dr-push-prob", type=float, default=0.01)
    p.add_argument("--dr-push-vmax", type=float, default=0.5)
    p.add_argument("--dr-obs-noise", type=float, default=0.01)
    p.add_argument("--dr-action-latency-max", type=int, default=1)
    p.add_argument("--dr-init-q-band", type=float, default=0.05)
    p.add_argument("--dr-init-xy-band", type=float, default=0.03)
    p.add_argument("--dr-init-z-band", type=float, default=0.01)
    p.add_argument("--dr-init-tilt-band", type=float, default=0.0)
    p.add_argument("--dr-init-vel-band", type=float, default=0.0)
    p.add_argument("--dr-init-vx-bias", type=float, default=0.0)
    p.add_argument("--dr-seed", type=int, default=0)
    p.add_argument("--no-dr", action="store_true")
    p.add_argument("--rough-amp", type=float, default=0.0,
                   help="max bump height (m). >0 enables blind rough-terrain training.")
    p.add_argument("--rough-seed", type=int, default=0)
    p.add_argument("--rough-x-start", type=float, default=2.0)
    p.add_argument("--rough-x-end", type=float, default=40.0)
    args = p.parse_args()

    if not Path(args.mjcf).exists():
        raise SystemExit(
            f"MJCF not found: {args.mjcf}. Dump it from the deploy world via "
            f"OMNISIM_NEWTON_SAVE_MJCF (and verify kp=500/kv=60).")

    reward_cfg = dict(alive=args.alive, upright=args.upright,
                      act=args.act, act_rate=args.act_rate, term=args.term,
                      vel=args.vel, vel_sigma=args.vel_sigma, vel_l1=args.vel_l1,
                      lat=args.lat, yaw=args.yaw, height=args.height,
                      max_ep=args.max_ep, res_scale=args.res_scale,
                      seed_gait=args.seed_gait_pose,
                      rest_start_frac=args.rest_start_frac,
                      wz_range=args.wz_range,
                      rw_sched=args.rw_sched, rw_slip=args.rw_slip,
                      foot_z_contact=args.foot_z_contact,
                      foot_z_swing=args.foot_z_swing,
                      vx_cmd_max=args.vx_cmd_max,
                      gait_params=dict(
                          vx=args.vx_target, freq=args.gait_freq,
                          duty=args.gait_duty,
                          step_height=args.gait_step_height,
                          body_height=args.gait_body_h,
                          x0=args.gait_x0, ramp_s=args.gait_ramp_s))
    if args.z_ref > 0:
        reward_cfg["z_ref"] = args.z_ref
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
            seed=args.dr_seed)
        print(f"[DR] {dr_cfg}")

    rough_cfg = None
    if args.rough_amp > 0:
        rough_cfg = dict(amp=args.rough_amp, seed=args.rough_seed,
                         x_start=args.rough_x_start, x_end=args.rough_x_end)
    env = BatchedSpotWalkEnv(args.envs, args.mjcf, reward_cfg=reward_cfg,
                             sim_dt=args.sim_dt, dr_cfg=dr_cfg, rough_cfg=rough_cfg)
    N = args.envs
    print(f"[gait] standing pose FL=({env.nominal[0]:+.3f},{env.nominal[1]:+.3f},"
          f"{env.nominal[2]:+.3f})  vx={env.vx_target}  freq={env.gp.freq}  "
          f"duty={env.gp.duty}  spawn_z={env.spawn_z:.3f}")

    OBS_IN = env.obs_dim    # 48, or 49 when velocity-conditioned (vx_cmd_max>0)
    if env.vx_cond:
        print(f"[VC] velocity-conditioned: obs={OBS_IN} vx_cmd_max={env.vx_cmd_max}")

    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = nn.Sequential(nn.Linear(OBS_IN, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(),
                                    nn.Linear(128, NJ))
            self.v = nn.Sequential(nn.Linear(OBS_IN, 256), nn.Tanh(),
                                   nn.Linear(256, 128), nn.Tanh(),
                                   nn.Linear(128, 1))
            self.log_std = nn.Parameter(-1.0 * torch.ones(NJ))

        def forward(self, obs):
            return self.pi(obs), self.v(obs).squeeze(-1), self.log_std

    tdev = env.tdev
    torch.manual_seed(0)
    ac = AC().to(tdev)
    if args.init_from and Path(args.init_from).exists():
        sd = torch.load(args.init_from, map_location=tdev)
        # Warm-start across an obs-width change (48 -> 49 for velocity
        # conditioning): zero-pad the new input columns of the first layers so
        # the policy starts as the loaded walker, ignoring the new vx_cmd input
        # until it learns to use it (the G1 47->48 warm-start, same trick).
        cur = ac.state_dict()
        for k in ("pi.0.weight", "v.0.weight"):
            if (k in sd and k in cur and sd[k].shape[0] == cur[k].shape[0]
                    and sd[k].shape[1] < cur[k].shape[1]):
                w = cur[k].clone()
                w[:, :sd[k].shape[1]] = sd[k]
                w[:, sd[k].shape[1]:] = 0.0
                sd[k] = w
                print(f"  warm-start padded {k}: {tuple(sd[k].shape)} "
                      f"(new input cols zeroed)")
        ac.load_state_dict(sd)
        print(f"warm-start from {args.init_from}")
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)

    if args.eval or args.bare:
        ac.eval()
        if not args.bare:
            ac.load_state_dict(torch.load(args.save, map_location=tdev))
        obs = env.reset()
        first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        n_falls = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        dist = torch.zeros(env.n, device=tdev)
        vx_sum = torch.zeros(env.n, device=tdev)
        alive_steps = torch.zeros(env.n, device=tdev)
        for step in range(args.eval_steps):
            if args.bare:
                mu = torch.zeros(env.n, NJ, device=tdev)
            else:
                with torch.no_grad():
                    mu, _, _ = ac(obs)
            alive = (first_fall == 0)
            obs, _, done, info = env.step(mu)
            step_alive = (alive & (~done)).float()
            vx = env.qvel_t[:, 0]
            dist += vx * DT * step_alive
            vx_sum += vx * step_alive
            alive_steps += step_alive
            n_falls += done.to(torch.int32)
            newly = done & (first_fall == 0)
            first_fall = torch.where(newly, torch.full_like(first_fall, step + 1),
                                     first_fall)
        ff = first_fall.cpu().numpy()
        d = dist.cpu().numpy()
        mean_vx = (vx_sum / torch.clamp(alive_steps, min=1.0)).cpu().numpy()
        never = (ff == 0)
        ff_fell = ff[~never]
        label = "BARE GAIT" if args.bare else args.save
        print(f"[gpu-eval] {label} envs={env.n} steps={args.eval_steps} "
              f"({args.eval_steps * DT:.0f}s)")
        print(f"  FIRST-FALL: mean={(ff_fell.mean() * DT) if ff_fell.size else -1:.2f}s  "
              f"median={(np.median(ff_fell) * DT) if ff_fell.size else -1:.2f}s  "
              f"never_fell={never.mean():.2%}  falls/env={n_falls.cpu().numpy().mean():.1f}")
        print(f"  fwd dist before first fall: mean={d.mean():.2f} m  "
              f"median={np.median(d):.2f} m  max={d.max():.2f} m")
        print(f"  fwd speed while alive: {mean_vx.mean():.3f} m/s "
              f"(target {env.vx_target})")
        return

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    rollout = args.rollout
    obs = env.reset()
    total_steps = 0
    t0 = time.time()
    obs_buf = torch.zeros(rollout, N, OBS_IN, device=tdev)
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

        obs_flat = obs_buf.reshape(-1, OBS_IN)
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
            fps = total_steps / max(time.time() - t0, 1e-6)
            print(f"it {it:4d}  ep_rew/step~{rew_buf.mean().item():+.3f}  "
                  f"meanV {val_buf.mean().item():+.2f}  steps {total_steps:,}  "
                  f"{fps:,.0f} env-steps/s", flush=True)

    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total_steps:,} steps in {time.time() - t0:.1f}s)")

    # ONNX export -- CLAMP head (the exact training squash; NEVER tanh).
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
        sys.stdout.write(buf.getvalue().encode("ascii", "replace").decode("ascii"))
        print(f"exported ONNX -> {onnx_path}")
    except Exception as e:
        print(f"ONNX export failed (non-fatal): {e}")


if __name__ == "__main__":
    main()
