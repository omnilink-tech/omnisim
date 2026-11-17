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

"""Durable G1 STAND trainer on the CERTIFIED add_urdf + SolverMuJoCo physics --
the SAME solver the deploy binary runs (the binary-parity probe proved
add_urdf+SolverMuJoCo == the deploy to machine precision). The canonical stand
trainer (gpu_mjwarp_g1_stand_trainer.py) steps RAW mujoco_warp on an MJCF, which
the closed-loop validation showed does NOT match the binary (mjw.step !=
SolverMuJoCo.step). Training here removes that gap: a policy durable in THIS
trainer is durable in deploy and behaves the same.

Durability fixes vs the first attempt: (1) base ANGULAR velocity in the obs
(tipping-rate; reproducible in deploy via getVelocity, unlike finite-diff joint
qd) -> G1_PARITY_OBS gives the 32-dim obs; (2) the analytic ankle-balance baseline
ON (reactive balance the position-only policy lacked); (3) train longer.

Reuses BatchedG1NewtonEnv (build_g1_prim_builder replicate -> SolverMuJoCo, the
working batched reset + index map), overriding step() for the stand task.

Run from PowerShell (native warp/CUDA):
  $env:G1_PARITY_OBS=1; $env:G1_NEWTON_KE=400; $env:G1_NEWTON_KD=60
  python projects/policies/research/training/gpu_newton_g1_stand_trainer.py --envs 1024 --iters 400 \
    --save projects/policies/research/training/runs/g1_newton_stand/policy.pt
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))

import projects.policies.research.training.gpu_mjwarp_g1_stand_trainer as MJ  # noqa: E402
from projects.policies.research.training.gpu_newton_g1_walk_trainer import BatchedG1NewtonEnv  # noqa: E402

NJ = MJ.NJ
OBS_DIM = MJ.OBS_DIM           # 32 with G1_PARITY_OBS=1
RES_SCALE = MJ.RES_SCALE
DT = MJ.DT
BZ_FAIL, ROLL_FAIL, PITCH_FAIL, MAX_EP = MJ.BZ_FAIL, MJ.ROLL_FAIL, MJ.PITCH_FAIL, MJ.MAX_EP


class G1NewtonStandEnv(BatchedG1NewtonEnv):
    """Stand task on the certified Newton SolverMuJoCo physics. Inherits the
    batched build / reset / 32-dim parity obs / NOMINAL+ankle-PD baseline; only
    step() (control + STAND reward) is task-specific."""

    def _build_obs_t(self):
        # 32-dim parity obs computed directly (BatchedG1NewtonEnv overrides the
        # parent's with the WALK obs; we want the plain stand one). Reads the live
        # Newton state views. proj_gravity from the (w,x,y,z) base quat.
        #
        # The angular-rate channel is the finite-difference of proj_gravity, NOT
        # the engine base ang-vel (qvel[3:6]). The closed-loop parity test proved
        # the engine ang-vel is in a DIFFERENT frame/scale in the binary deploy
        # (R^T*getVelocity) -- it diverges by ~2 rad/s at tick 1 while the pose
        # matches to 3e-3, which is the dominant cause of the deploy fall. d(proj_g)
        # is reproducible to ~3e-3 on both sides (proj_g itself matches), encodes
        # the balance-relevant roll/pitch rates, and drops the unreproducible yaw.
        qp = self.qpos_t
        w = qp[:, 3]; x = qp[:, 4]; y = qp[:, 5]; z = qp[:, 6]
        pg = torch.stack([-2 * (x * z - w * y), -2 * (y * z + w * x),
                          -(1 - 2 * (x * x + y * y))], dim=1)
        q = qp.index_select(1, self.qpos_idx_t) - self.nominal_t.unsqueeze(0)
        if getattr(self, "_prev_pg", None) is None or self._prev_pg.shape != pg.shape:
            self._prev_pg = pg.detach().clone()
        dpg = (pg - self._prev_pg) / DT
        fresh = (self.ep_step_t == 0)            # no valid previous frame yet
        if fresh.any():
            dpg = torch.where(fresh.unsqueeze(1), torch.zeros_like(dpg), dpg)
        self._prev_pg = pg.detach().clone()
        obs = torch.cat([q, pg, dpg, self.last_action_t], dim=1)
        on = float(self.dr.get("obs_noise", 0.0))
        if on > 0:
            obs = obs + torch.randn_like(obs) * on
        obs = torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        return torch.clamp(obs, -10.0, 10.0)

    def _init_phys_dr(self):
        # HEAVY domain randomization, per-env, applied entirely through the
        # state/control channels (no shared-model edits). The headline term is a
        # PERSISTENT random base acceleration (`bias_amax`, m/s^2, fixed per
        # episode, random direction) -- the robot must learn to PLANT and stand
        # still against an unknown constant push, which is exactly what the
        # train<->deploy residual differences look like, and because the direction
        # is random, drifting cannot help. Plus per-env actuator-gain scaling.
        self._bias_amax = float(self.dr.get("bias_amax", 0.0))
        self._gain_band = float(self.dr.get("gain_band", 0.0))
        self._bias_accel = torch.zeros(self.n, 2, device=self.tdev)
        self._dr_gain = torch.ones(self.n, device=self.tdev)
        self._resample_phys_dr(None)

    def _resample_phys_dr(self, mask):
        # resample the per-episode DR params for envs in `mask` (None = all).
        import math
        if mask is None:
            mask = torch.ones(self.n, dtype=torch.bool, device=self.tdev)
        m = int(mask.sum().item())
        if m == 0:
            return
        if self._bias_amax > 0:
            theta = torch.rand(m, device=self.tdev) * (2 * math.pi)
            mag = torch.rand(m, device=self.tdev) * self._bias_amax
            acc = torch.zeros(m, 2, device=self.tdev)
            acc[:, 0] = torch.cos(theta) * mag
            acc[:, 1] = torch.sin(theta) * mag
            self._bias_accel[mask] = acc
        if self._gain_band > 0:
            self._dr_gain[mask] = 1.0 + (torch.rand(m, device=self.tdev) * 2 - 1) * self._gain_band

    # ---- CONTRACTION (finite-time Lyapunov) reward, Benettin twin-rollout ----
    # Worlds are paired ref[k]=k, twin[k]=k+P (P=n//2). The twin starts eps0 away;
    # each step we penalise the GROWTH of the ref<->twin separation (= the per-step
    # finite-time Lyapunov exponent), so the policy learns a CONTRACTIVE closed loop
    # (lambda<=0): perturbations decay instead of e-folding. That is exactly deploy
    # robustness to the train<->deploy chaos -- and a contractive stand also cannot
    # drift (it returns to its attractor).
    def _sync_to_newton(self):
        # back-propagate mjw_data (qpos_t/qvel_t) writes into newton state_a, or
        # solver.step clobbers them next step (same fix _reset_envs uses).
        self.solver._update_newton_state(
            self.model, self.state_a, self.solver.mjw_data, state_prev=self.state_a)

    def _cstate(self):
        # separation metric = FALL-RELEVANT, STABILIZABLE coords only: base height
        # (1) + tilt = proj_gravity xy (2) + joint angles (13). DELIBERATELY excludes
        # base xy and yaw -- those are translation/rotation invariant (the robot can
        # stand anywhere / facing anywhere), so their separation cannot contract;
        # including them floods lambda with un-contractible diffusion.
        qp = self.qpos_t
        w = qp[:, 3]; x = qp[:, 4]; y = qp[:, 5]; z = qp[:, 6]
        tilt = torch.stack([-2 * (x * z - w * y), -2 * (y * z + w * x)], dim=1)
        return torch.cat([qp[:, 2:3], tilt, qp.index_select(1, self.qpos_idx_t)], dim=1)

    def _pair_twins(self, pair_mask):
        # re-sync twin = ref (ALL coords) then perturb only the JOINTS by eps0. Done
        # at pairing, on a fall, and every K steps (Benettin renormalization): it
        # keeps the probe in the linear regime and resets the un-contractible xy/yaw
        # so only the fall-relevant divergence is measured. pair_mask: [P] bool.
        P = self._P
        if pair_mask is None:
            pair_mask = torch.ones(P, dtype=torch.bool, device=self.tdev)
        m = int(pair_mask.sum().item())
        if m == 0:
            return
        ref_idx = torch.arange(P, device=self.tdev)[pair_mask]
        tw_idx = ref_idx + P
        new_tw = self.qpos_t[ref_idx].clone()
        dj = torch.randn(m, NJ, device=self.tdev)
        dj = dj / (dj.norm(dim=1, keepdim=True) + 1e-9) * self._contract_eps
        new_tw[:, self.qpos_idx_t] = new_tw[:, self.qpos_idx_t] + dj
        self.qpos_t[tw_idx] = new_tw
        self.qvel_t[tw_idx] = self.qvel_t[ref_idx].clone()
        self._cd_prev[pair_mask] = self._contract_eps

    def _ensure_stand_state(self):
        # foot world-pos view + per-env home xy + prev foot xy + DR buffers.
        # Lazy so eval paths work without reset().
        if not self._xpos_ready:
            self._setup_foot_xpos()
        if getattr(self, "_home_xy", None) is None:
            self._home_xy = self.qpos_t[:, 0:2].detach().clone()
            self._prev_feet_xy = self.xpos_t[:, :, 0:2].detach().clone()
        if getattr(self, "_bias_accel", None) is None:
            self._init_phys_dr()
        if getattr(self, "_contract_w", None) is None:
            self._contract_w = float(self.r.get("contract_w", 0.0))
            self._contract_eps = float(self.r.get("contract_eps", 1e-2))
            self._contract_k = int(self.r.get("contract_k", 8))
            self._P = self.n // 2
            self._cd_prev = torch.full((self._P,), self._contract_eps, device=self.tdev)
            if self._contract_w > 0:
                self._pair_twins(None)

    def reset(self):
        # plain 32-dim parity obs (no walk lookahead/frame-stacking).
        if not self._xpos_ready:
            self._setup_foot_xpos()
        self._reset_envs(env_mask=None)
        self._home_xy = self.qpos_t[:, 0:2].detach().clone()
        self._prev_feet_xy = self.xpos_t[:, :, 0:2].detach().clone()
        self._init_phys_dr()
        self._ensure_stand_state()           # inits contraction buffers + pairs twins
        if self._contract_w > 0:
            self._pair_twins(None)
            self._sync_to_newton()
        return self._build_obs_t()

    def step(self, action_t):
        action_t = torch.clamp(action_t, -1.0, 1.0)
        self._ensure_stand_state()
        prev_action = self.last_action_t       # for the action-rate penalty
        # baseline = NOMINAL + analytic ankle balance PD (reactive balance).
        baseline, _, _ = self._baseline_targets_t()
        gain = self._dr_gain.unsqueeze(1)        # per-env actuator-strength DR
        targets = torch.clamp(baseline + (self.res_scale_vec * gain) * action_t,
                              self.jl_lo_t, self.jl_hi_t)
        self.ctrl_pos_t.index_copy_(0, self.ctrl_write_idx, targets.reshape(-1))
        if self.hold_arms:
            self.ctrl_pos_t.index_copy_(0, self.arm_write_idx,
                                        self.arm_targets_t.reshape(-1))
        # DR pushes (optional, impulsive).
        if self._push_p > 0 and self._push_vmax > 0:
            import math
            hit = torch.rand(self.n, device=self.tdev) < self._push_p
            if hit.any():
                theta = torch.rand(self.n, device=self.tdev) * (2 * math.pi)
                mag = torch.rand(self.n, device=self.tdev) * self._push_vmax
                self.qvel_t[hit, 0] = self.qvel_t[hit, 0] + (torch.cos(theta) * mag)[hit]
                self.qvel_t[hit, 1] = self.qvel_t[hit, 1] + (torch.sin(theta) * mag)[hit]
        # HEAVY DR: persistent per-env base acceleration bias (m/s^2 * dt each
        # step = a sustained unknown push the policy must plant against).
        if getattr(self, "_bias_amax", 0.0) > 0:
            self.qvel_t[:, 0] = self.qvel_t[:, 0] + self._bias_accel[:, 0] * DT
            self.qvel_t[:, 1] = self.qvel_t[:, 1] + self._bias_accel[:, 1] * DT

        self._physics()                       # EXACT deploy substep loop (SolverMuJoCo)

        self.ep_step_t = self.ep_step_t + 1
        self.last_action_t = action_t

        bz = self.qpos_t[:, 2]
        w = self.qpos_t[:, 3]; x = self.qpos_t[:, 4]
        y = self.qpos_t[:, 5]; z = self.qpos_t[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sinp = torch.clamp(2 * (w * y - z * x), -1.0, 1.0)
        pitch = torch.asin(sinp)
        vlin = torch.linalg.norm(self.qvel_t[:, 0:3], dim=1)
        vang = torch.linalg.norm(self.qvel_t[:, 3:6], dim=1)

        # --- STAND-IN-PLACE shaping (privileged, reward-only; the policy never
        # sees global position -- it must learn a drift-free posture from local
        # obs). base DRIFT from home discourages net translation; FOOT-SLIP
        # penalises horizontal speed of feet IN GROUND CONTACT, while lifted feet
        # (z above the contact threshold) move freely -> stepping to balance is
        # allowed, sliding is not.
        base_xy = self.qpos_t[:, 0:2]
        drift = torch.linalg.norm(base_xy - self._home_xy, dim=1)        # (n,) m
        # deadzone: balancing steps within `drift_deadzone` are FREE (so the
        # policy keeps the foot-repositioning slack it needs to survive the
        # train<->deploy differences); only wandering BEYOND it is penalised.
        drift_pen = torch.clamp(drift - float(self.r.get("drift_deadzone", 0.0)), min=0.0)
        feet = self.xpos_t                                               # (n,2,3)
        feet_xy = feet[:, :, 0:2]
        in_contact = (feet[:, :, 2] < self.foot_z_contact).float()       # (n,2)
        foot_v = torch.linalg.norm((feet_xy - self._prev_feet_xy) / DT, dim=2)  # (n,2) m/s
        slip = (in_contact * foot_v).sum(dim=1)                          # (n,)
        self._prev_feet_xy = feet_xy.detach().clone()

        r = self.r
        reward = (r.get("alive", 1.0) * torch.ones(self.n, device=self.tdev)
                  + r.get("upright", 0.5) * torch.clamp(1.0 - roll * roll - pitch * pitch, min=0.0)
                  + r.get("lin", -0.3) * vlin
                  + r.get("ang", -0.1) * vang
                  + r.get("pos_hold", -1.2) * drift_pen
                  + r.get("foot_slip", -0.5) * slip
                  + r.get("act", -0.01) * (action_t * action_t).sum(dim=1)
                  + r.get("act_rate", -0.02) * ((action_t - prev_action) ** 2).sum(dim=1))

        # CONTRACTION (finite-time Lyapunov) reward: penalise growth of the
        # ref<->twin separation = the per-step Lyapunov exponent. Drives lambda<=0.
        lam_mean = 0.0
        if self._contract_w > 0:
            cs = self._cstate()
            P = self._P
            sep = cs[:P] - cs[P:2 * P]
            d = torch.linalg.norm(sep, dim=1).clamp_min(1e-9)          # (P,)
            growth = torch.log(d / self._cd_prev.clamp_min(1e-9))      # per-step ln-growth
            lam_mean = float((growth.mean() / DT).item())             # ~ Lyapunov exp (1/s)
            cpen = -self._contract_w * torch.clamp(growth, min=0.0)    # penalise divergence
            reward[:P] = reward[:P] + cpen
            reward[P:2 * P] = reward[P:2 * P] + cpen
            self._cd_prev = d.detach()

        drift_fail = float(r.get("drift_fail", 1.0))
        fall = ((torch.abs(roll) > ROLL_FAIL) | (torch.abs(pitch) > PITCH_FAIL)
                | (bz < BZ_FAIL) | (drift > drift_fail))
        done = fall | (self.ep_step_t >= MAX_EP)
        reward = reward + fall.float() * r.get("term", -1.0)
        if done.any():
            self._reset_envs(env_mask=done)
            # refresh per-env home + prev-foot + resample the per-episode DR for
            # the freshly reset envs (new bias direction + actuator gain).
            self._home_xy[done] = self.qpos_t[done, 0:2].detach()
            self._prev_feet_xy[done] = self.xpos_t[:, :, 0:2][done].detach()
            self._resample_phys_dr(done)
        if self._contract_w > 0:
            # Benettin renormalize every K steps (re-pair ALL) + re-pair any pair
            # disrupted by a fall, so the probe stays in the linear regime.
            P = self._P
            disrupted = done[:P] | done[P:2 * P]
            renorm = (int(self.ep_step_t[0].item()) % self._contract_k == 0)
            mask = torch.ones(P, dtype=torch.bool, device=self.tdev) if renorm else disrupted
            if mask.any():
                self._pair_twins(mask)
                self._sync_to_newton()
        return self._build_obs_t(), reward, done, {"bz": bz, "roll": roll, "pitch": pitch,
                                                   "drift": drift, "slip": slip, "lam": lam_mean}


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


def _wxyz_to_rotmat(qw, qx, qy, qz):
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return [1-2*(yy+zz), 2*(xy-wz), 2*(xz+wy),
            2*(xy+wz), 1-2*(xx+zz), 2*(yz-wx),
            2*(xz-wy), 2*(yz+wx), 1-2*(xx+yy)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=1024)
    p.add_argument("--iters", type=int, default=400)
    p.add_argument("--rollout", type=int, default=24)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save", default=str(REPO / "projects/policies/research/training/runs/g1_newton_stand/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=400)
    p.add_argument("--eval-settle", type=int, default=40)
    p.add_argument("--dump-trace", default=None)
    p.add_argument("--no-dr", action="store_true")
    p.add_argument("--pos-hold", type=float, default=-1.2,
                   help="reward weight on base xy drift from home (stay in place)")
    p.add_argument("--foot-slip", type=float, default=-0.5,
                   help="reward weight on horizontal speed of in-contact feet (no sliding)")
    p.add_argument("--drift-fail", type=float, default=1.0,
                   help="terminate the episode if base drifts > this many metres")
    p.add_argument("--lin-vel", type=float, default=-0.3,
                   help="reward weight on base linear-velocity norm (gentle, robust anti-drift)")
    p.add_argument("--act-rate", type=float, default=-0.02,
                   help="reward weight on action-rate (smoothness)")
    p.add_argument("--drift-deadzone", type=float, default=0.0,
                   help="free-movement radius (m): drift within it is unpenalised "
                        "(keeps balancing slack); only beyond it is penalised")
    p.add_argument("--heavy-dr", action="store_true",
                   help="HEAVY domain randomization: persistent random base force + "
                        "actuator-gain jitter + bigger pushes/noise/latency, to make a "
                        "PLANTED stand robust to the train<->deploy divergence")
    p.add_argument("--dr-scale", type=float, default=1.0,
                   help="scale all heavy-DR magnitudes (sweep 'as heavy as it can stand')")
    p.add_argument("--contraction", action="store_true",
                   help="CHAOS-MINIMIZATION: pair worlds (ref+twin eps apart) and reward "
                        "the policy for SHRINKING their separation (finite-time Lyapunov) "
                        "-> a contractive closed loop that resists the train<->deploy chaos")
    p.add_argument("--contract-w", type=float, default=0.3,
                   help="weight on the per-step Lyapunov (separation-growth) penalty")
    p.add_argument("--contract-eps", type=float, default=1e-2,
                   help="twin separation set-point (Benettin renormalization scale)")
    p.add_argument("--contract-k", type=int, default=8,
                   help="renormalize the twin every K control steps")
    args = p.parse_args()

    if not os.environ.get("G1_PARITY_OBS"):
        raise SystemExit("set G1_PARITY_OBS=1 (32-dim parity obs)")
    # Stand reward, NO gait/vx -> the env runs as a pure stand. Light DR for
    # robustness (off with --no-dr for the cleanest parity eval).
    reward_cfg = dict(alive=1.0, upright=0.6, lin=args.lin_vel, ang=-0.1, act=-0.01,
                      act_rate=args.act_rate, term=-1.0, gait_model="", vx_cmd_max=0.0,
                      res_scale=RES_SCALE,
                      pos_hold=args.pos_hold, foot_slip=args.foot_slip,
                      drift_fail=args.drift_fail, drift_deadzone=args.drift_deadzone,
                      contract_w=(args.contract_w if args.contraction else 0.0),
                      contract_eps=args.contract_eps, contract_k=args.contract_k)
    if args.contraction and args.envs % 2 != 0:
        raise SystemExit("--contraction needs an even --envs (worlds are paired)")
    # Domain randomization for sim-to-deploy ROBUSTNESS: the policy must survive
    # the tiny train<->binary differences (obs noise, push, init spread, latency,
    # tilt). Without it (--no-dr) the policy overfits the exact trainer dynamics
    # and a marginal stand tips in deploy.
    if args.eval or args.no_dr:
        dr_cfg = {}
    elif args.heavy_dr:
        sc = args.dr_scale
        # HEAVY: the persistent base-force bias (bias_amax) is the headline term;
        # gain_band randomizes actuator strength; pushes/noise/init are amplified.
        dr_cfg = dict(
            push_prob=min(0.25, 0.15 * sc), push_vmax=2.0 * sc, obs_noise=0.03 * sc,
            init_q_band=0.14 * sc, init_z_band=0.02 * sc, init_tilt_band=0.18 * sc,
            init_vel_band=0.25 * sc, action_latency_max=min(4, round(3 * sc)),
            bias_amax=1.6 * sc, gain_band=min(0.6, 0.30 * sc))
    else:
        dr_cfg = dict(
            push_prob=0.05, push_vmax=1.2, obs_noise=0.02, init_q_band=0.10,
            init_z_band=0.01, init_tilt_band=0.12, init_vel_band=0.15,
            action_latency_max=2)
    env = G1NewtonStandEnv(args.envs, reward_cfg=reward_cfg, dr_cfg=dr_cfg)
    N, tdev = args.envs, env.tdev
    torch.manual_seed(0)
    ac = AC().to(tdev)
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)

    if args.eval:
        ac.load_state_dict(torch.load(args.save, map_location=tdev)); ac.eval()
        obs = env.reset()
        zero = torch.zeros(env.n, NJ, device=tdev)
        trace = [] if args.dump_trace else None
        rec_settle = os.environ.get("DUMP_SETTLE", "0") != "0"

        def _capture(step, phase):
            qp0 = env.qpos_t[0].detach().cpu().numpy()
            legq = [float(qp0[int(env.controller_to_qpos[i])]) for i in range(NJ)]
            trace.append({"k": step, "phase": phase, "target": legq, "q": legq,
                          "base_pos": [float(qp0[0]), float(qp0[1]), float(qp0[2])],
                          "base_rot": _wxyz_to_rotmat(float(qp0[3]), float(qp0[4]),
                                                      float(qp0[5]), float(qp0[6]))})

        for si in range(args.eval_settle):
            obs, _, _, _ = env.step(zero)
            if trace is not None and rec_settle:
                _capture(si - args.eval_settle, "settle")
        survived = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        max_drift = torch.zeros(env.n, device=tdev)
        for step in range(args.eval_steps):
            with torch.no_grad():
                mu, _, _ = ac(obs)
            obs, _, done, info = env.step(mu)
            alive = (first_fall == 0)
            max_drift = torch.where(alive, torch.maximum(max_drift, info["drift"]), max_drift)
            if trace is not None:
                qp0 = env.qpos_t[0].detach().cpu().numpy()
                a0 = torch.clamp(mu[0], -1, 1).detach().cpu().numpy()
                legq = [float(qp0[int(env.controller_to_qpos[i])]) for i in range(NJ)]
                base, _, _ = env._baseline_targets_t()
                tgt = np.clip(base[0].detach().cpu().numpy() + RES_SCALE * a0,
                              MJ.JOINT_LIMITS_LO, MJ.JOINT_LIMITS_HI)
                trace.append({"k": step, "phase": "probe", "target": [float(v) for v in tgt],
                              "q": legq, "base_pos": [float(qp0[0]), float(qp0[1]), float(qp0[2])],
                              "base_rot": _wxyz_to_rotmat(float(qp0[3]), float(qp0[4]),
                                                          float(qp0[5]), float(qp0[6]))})
            survived += (~done).to(torch.int32)
            newly = done & (first_fall == 0)
            first_fall = torch.where(newly, torch.full_like(first_fall, step + 1), first_fall)
        s = survived.cpu().numpy(); ff = first_fall.cpu().numpy()
        nf = (ff == 0); md = max_drift.cpu().numpy()
        md_nf = md[nf] if nf.any() else md
        print(f"[newton-stand-eval] envs={env.n} steps={args.eval_steps} "
              f"survival mean={s.mean():.1f} median={np.median(s):.0f} max={s.max()} "
              f"never_fell_frac={nf.mean():.2f}  "
              f"max_drift(never_fell) mean={md_nf.mean():.3f}m p95={np.percentile(md_nf,95):.3f}m")
        if trace is not None:
            out = {"schema": 1, "side": "trainer",
                   "meta": {"robot": "g1", "construction": "add_urdf + SolverMuJoCo (certified)",
                            "sequence": "policy", "njoints": NJ,
                            "joint_order": list(MJ.LEGS_JOINTS), "obs_dim": OBS_DIM},
                   "ticks": trace}
            Path(args.dump_trace).parent.mkdir(parents=True, exist_ok=True)
            Path(args.dump_trace).write_text(json.dumps(out), encoding="utf-8")
            print(f"[dump-trace] wrote {args.dump_trace} ({len(trace)} ticks)")
        return

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    obs = env.reset()
    obs_buf = torch.zeros(args.rollout, N, OBS_DIM, device=tdev)
    act_buf = torch.zeros(args.rollout, N, NJ, device=tdev)
    logp_buf = torch.zeros(args.rollout, N, device=tdev)
    rew_buf = torch.zeros(args.rollout, N, device=tdev)
    done_buf = torch.zeros(args.rollout, N, device=tdev)
    val_buf = torch.zeros(args.rollout, N, device=tdev)
    total, t0 = 0, time.time()
    lam_ema = None
    for it in range(1, args.iters + 1):
        for k in range(args.rollout):
            with torch.no_grad():
                mu, v, log_std = ac(obs)
                dist = torch.distributions.Normal(mu, log_std.exp())
                a = dist.sample(); logp = dist.log_prob(a).sum(-1)
            obs_buf[k], act_buf[k], logp_buf[k], val_buf[k] = obs, a, logp, v
            obs, rew, done, info = env.step(a)
            rew_buf[k], done_buf[k] = rew, done.float(); total += N
            if args.contraction:
                lv = info.get("lam", 0.0)
                lam_ema = lv if lam_ema is None else 0.99 * lam_ema + 0.01 * lv
        with torch.no_grad():
            _, last_v, _ = ac(obs)
        gamma, lam = 0.99, 0.95
        adv = torch.zeros_like(rew_buf); gae = torch.zeros(N, device=tdev)
        for k in reversed(range(args.rollout)):
            nonterm = 1.0 - done_buf[k]
            nextv = last_v if k == args.rollout - 1 else val_buf[k + 1]
            delta = rew_buf[k] + gamma * nextv * nonterm - val_buf[k]
            gae = delta + gamma * lam * nonterm * gae; adv[k] = gae
        ret = adv + val_buf
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        of, af = obs_buf.reshape(-1, OBS_DIM), act_buf.reshape(-1, NJ)
        lf, advf, rf = logp_buf.reshape(-1), adv.reshape(-1), ret.reshape(-1)
        for _e in range(4):
            mu, v, log_std = ac(of)
            dist = torch.distributions.Normal(mu, log_std.exp())
            nl = dist.log_prob(af).sum(-1); ratio = (nl - lf).exp()
            s1 = ratio * advf; s2 = torch.clamp(ratio, 0.8, 1.2) * advf
            loss = (-torch.min(s1, s2).mean() + 0.5 * ((v - rf) ** 2).mean()
                    - 0.01 * dist.entropy().sum(-1).mean())
            opt.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5); opt.step()
        if it % 10 == 0 or it == 1:
            lam_str = f"  lambda~{lam_ema:+.2f}/s" if (args.contraction and lam_ema is not None) else ""
            print(f"it {it:4d}  ep_rew/step~{rew_buf.mean().item():+.3f}  "
                  f"meanV {val_buf.mean().item():+.2f}{lam_str}  steps {total:,}  "
                  f"{total/max(time.time()-t0,1e-6):,.0f}/s")
    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save} ({total:,} steps, {time.time()-t0:.1f}s)")
    # ONNX (clamp -> matches deploy).
    onnx_path = Path(args.save).with_suffix(".onnx")
    cpu = AC(); cpu.load_state_dict({k: v.cpu() for k, v in ac.state_dict().items()})

    class Deploy(nn.Module):
        def __init__(s, pi): super().__init__(); s.pi = pi
        def forward(s, o): return torch.clamp(s.pi(o), -1.0, 1.0)
    import io, contextlib
    with contextlib.redirect_stdout(io.StringIO()):
        torch.onnx.export(Deploy(cpu.pi).eval(), torch.zeros(1, OBS_DIM), str(onnx_path),
                          input_names=["obs"], output_names=["action"],
                          dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
                          opset_version=17)
    print(f"exported ONNX -> {onnx_path}")


if __name__ == "__main__":
    main()
