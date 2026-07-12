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

"""From-scratch G1 WALK on the CERTIFIED physics with a REPRODUCIBLE obs.

The from-scratch walk hits a ~1.3 s deploy wall for the SAME reason the stand did:
its obs feeds engine velocities the deploy can't reproduce (base linear velocity +
base angular velocity from getVelocity, which the binary-parity probe proved is a
different frame/scale than the trainer's mjw qvel). This trainer applies the stand
fix to walking:

  * a WALK = the stand's balance residual riding on a sine-CPG gait baseline (the
    gait does the stepping; the policy only has to BALANCE the moving base -- which
    is why a walk deploys MORE robustly than a planted stand: it self-corrects every
    footstep).
  * REPRODUCIBLE 34-dim obs, identical on both sides:
        [ q - NOMINAL (13), proj_gravity (3), d(proj_gravity)/dt (3),
          gait sin/cos (2), last_action (13) ]
    NO base linear velocity (Unitree drops it too), NO engine base ang-vel (replaced
    by the reproducible proj_gravity finite-diff, exactly as in the stand fix).

Extends G1NewtonStandEnv (certified add_urdf+SolverMuJoCo physics, the deploy
binary's solver) so the physics + the reproducible-rate obs are inherited; only the
GAIT baseline, the gait obs, and the WALK reward are new.

Run (PowerShell, native warp/CUDA):
  $env:G1_PARITY_OBS=1; $env:G1_NEWTON_KE=400; $env:G1_NEWTON_KD=60
  python projects/policies/research/training/gpu_newton_g1_walk_repro_trainer.py --envs 1024 --iters 800 \
    --save projects/policies/research/training/runs/g1_walk_repro/policy.pt
"""
from __future__ import annotations

import argparse
import io
import contextlib
import math
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
from projects.policies.research.training.gpu_newton_g1_stand_trainer import G1NewtonStandEnv  # noqa: E402

NJ = MJ.NJ
DT = MJ.DT
RES_SCALE = MJ.RES_SCALE
BZ_FAIL, ROLL_FAIL, PITCH_FAIL, MAX_EP = MJ.BZ_FAIL, MJ.ROLL_FAIL, MJ.PITCH_FAIL, MJ.MAX_EP
WALK_OBS_DIM = 34          # q13 + proj_g3 + dproj_g3 + gait2 + last_action13
# 0 entropy bonus + low log_std init -> the policy commits to its MEAN (closes the
# stochastic-vs-deterministic gap that left the deploy-relevant mean policy shuffling
# while the noisy training policy "walked").
_ENT_COEF = float(os.environ.get("G1_ENT_COEF", "0.0"))

# sine-CPG gait (walk15 recipe): left leg phase theta, right leg theta+pi.
A_HIP = float(os.environ.get("G1_GAIT_A_HIP", "0.35"))
A_KNEE = float(os.environ.get("G1_GAIT_A_KNEE", "0.45"))
A_ANKLE = float(os.environ.get("G1_GAIT_A_ANKLE", "0.35"))
A_PUSH = float(os.environ.get("G1_GAIT_A_PUSH", "0.40"))    # plantarflex push-off (propulsion)
A_LAT = float(os.environ.get("G1_GAIT_A_LAT", "0.12"))      # lateral weight-shift sway (narrow foot)
GAIT_FREQ = float(os.environ.get("G1_GAIT_FREQ", "1.3"))     # Hz
GAIT_DPHASE = 2.0 * math.pi * GAIT_FREQ * DT                 # rad / control step
# leg joint indices into the 13-vector (see MJ.LEGS_JOINTS)
_LHP, _LHR, _LK, _LAP = 0, 1, 3, 4
_RHP, _RHR, _RK, _RAP = 6, 7, 9, 10


class G1NewtonWalkEnv(G1NewtonStandEnv):
    """Gait-residual WALK on the certified physics + reproducible obs."""

    # ---- reproducible 34-dim obs: stand obs (q, proj_g, d-proj_g, last_action)
    # with the gait sin/cos spliced in before last_action ----
    def _build_obs_t(self):
        qp = self.qpos_t
        w = qp[:, 3]; x = qp[:, 4]; y = qp[:, 5]; z = qp[:, 6]
        pg = torch.stack([-2 * (x * z - w * y), -2 * (y * z + w * x),
                          -(1 - 2 * (x * x + y * y))], dim=1)
        q = qp.index_select(1, self.qpos_idx_t) - self.nominal_t.unsqueeze(0)
        if getattr(self, "_prev_pg", None) is None or self._prev_pg.shape != pg.shape:
            self._prev_pg = pg.detach().clone()
        dpg = (pg - self._prev_pg) / DT
        fresh = (self.ep_step_t == 0)
        if fresh.any():
            dpg = torch.where(fresh.unsqueeze(1), torch.zeros_like(dpg), dpg)
        self._prev_pg = pg.detach().clone()
        gait = torch.stack([torch.sin(self.phase_t), torch.cos(self.phase_t)], dim=1)
        obs = torch.cat([q, pg, dpg, gait, self.last_action_t], dim=1)
        obs = torch.nan_to_num(obs, nan=0.0, posinf=10.0, neginf=-10.0)
        return torch.clamp(obs, -10.0, 10.0)

    # ---- baseline = NOMINAL + analytic ankle-balance PD (inherited) + gait CPG ----
    def _gait_baseline(self):
        # matches the walk15 sine-CPG (gpu_mjwarp_g1_walk_trainer): hip swing +
        # knee lift + ankle counter-rotation + END-OF-STANCE PUSH-OFF (the
        # propulsion) + same-sign lateral sway (pelvis shifts to the stance foot).
        base, _, _ = self._baseline_targets_t()    # NOMINAL + ankle PD
        base = base.clone()
        th = self.phase_t
        sL = torch.sin(th); sR = torch.sin(th + math.pi)
        base[:, _LHP] = base[:, _LHP] - A_HIP * sL
        base[:, _RHP] = base[:, _RHP] - A_HIP * sR
        base[:, _LK] = base[:, _LK] + A_KNEE * torch.clamp(sL, min=0.0)
        base[:, _RK] = base[:, _RK] + A_KNEE * torch.clamp(sR, min=0.0)
        base[:, _LAP] = base[:, _LAP] + A_ANKLE * sL
        base[:, _RAP] = base[:, _RAP] + A_ANKLE * sR
        if A_PUSH != 0.0:                          # push-off propulsion (end of stance)
            pushL = torch.clamp(torch.sin(th - 1.5 * math.pi), min=0.0) ** 2
            pushR = torch.clamp(torch.sin(th - 0.5 * math.pi), min=0.0) ** 2
            base[:, _LAP] = base[:, _LAP] + A_PUSH * pushL
            base[:, _RAP] = base[:, _RAP] + A_PUSH * pushR
        sway = A_LAT * sL                          # same sign both legs
        base[:, _LHR] = base[:, _LHR] + sway
        base[:, _RHR] = base[:, _RHR] + sway
        return base

    def reset(self):
        self._reset_envs(env_mask=None)
        self.phase_t = torch.zeros(self.n, device=self.tdev)
        self._prev_pg = None
        return self._build_obs_t()

    def step(self, action_t):
        action_t = torch.clamp(action_t, -1.0, 1.0)
        if getattr(self, "phase_t", None) is None:
            self.phase_t = torch.zeros(self.n, device=self.tdev)
        prev_action = self.last_action_t
        baseline = self._gait_baseline()
        targets = torch.clamp(baseline + self.res_scale_vec * action_t,
                              self.jl_lo_t, self.jl_hi_t)
        self.ctrl_pos_t.index_copy_(0, self.ctrl_write_idx, targets.reshape(-1))
        if self.hold_arms:
            self.ctrl_pos_t.index_copy_(0, self.arm_write_idx,
                                        self.arm_targets_t.reshape(-1))
        self._physics()
        self.ep_step_t = self.ep_step_t + 1
        self.last_action_t = action_t
        self.phase_t = (self.phase_t + GAIT_DPHASE) % (2.0 * math.pi)

        bz = self.qpos_t[:, 2]
        w = self.qpos_t[:, 3]; x = self.qpos_t[:, 4]
        y = self.qpos_t[:, 5]; z = self.qpos_t[:, 6]
        roll = torch.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        sinp = torch.clamp(2 * (w * y - z * x), -1.0, 1.0)
        pitch = torch.asin(sinp)
        vx = self.qvel_t[:, 0]; vy = self.qvel_t[:, 1]; wz = self.qvel_t[:, 5]

        r = self.r
        vx_cmd = float(r.get("vx_cmd", 0.4))
        reward = (r.get("alive", 1.0) * torch.ones(self.n, device=self.tdev)
                  + r.get("upright", 0.5) * torch.clamp(1.0 - roll * roll - pitch * pitch, min=0.0)
                  + r.get("vel", 2.0) * torch.exp(-((vx - vx_cmd) ** 2) / r.get("vel_sig", 0.25))
                  + r.get("fwd_lin", 0.0) * torch.clamp(vx, min=-0.2, max=vx_cmd)   # direct forward push
                  + r.get("lat", -0.5) * torch.abs(vy)
                  + r.get("yaw", -0.5) * torch.abs(wz)
                  + r.get("height", -8.0) * (bz - r.get("bz_ref", 0.72)) ** 2
                  + r.get("act", -0.005) * (action_t * action_t).sum(dim=1)
                  + r.get("act_rate", -0.01) * ((action_t - prev_action) ** 2).sum(dim=1))
        fall = (torch.abs(roll) > ROLL_FAIL) | (torch.abs(pitch) > PITCH_FAIL) | (bz < BZ_FAIL)
        done = fall | (self.ep_step_t >= MAX_EP)
        reward = reward + fall.float() * r.get("term", -1.0)
        if done.any():
            self._reset_envs(env_mask=done)
            self.phase_t[done] = 0.0
        return self._build_obs_t(), reward, done, {"bz": bz, "roll": roll, "pitch": pitch, "vx": vx}


class AC(nn.Module):
    def __init__(self):
        super().__init__()
        self.pi = nn.Sequential(nn.Linear(WALK_OBS_DIM, 256), nn.Tanh(),
                                nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, NJ))
        self.v = nn.Sequential(nn.Linear(WALK_OBS_DIM, 256), nn.Tanh(),
                               nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, 1))
        self.log_std = nn.Parameter(
            float(os.environ.get("G1_LOG_STD_INIT", "-1.5")) * torch.ones(NJ))

    def forward(self, obs):
        return self.pi(obs), self.v(obs).squeeze(-1), self.log_std


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=1024)
    p.add_argument("--iters", type=int, default=800)
    p.add_argument("--rollout", type=int, default=24)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save", default=str(REPO / "projects/policies/research/training/runs/g1_walk_repro/policy.pt"))
    p.add_argument("--vx-cmd", type=float, default=0.4)
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=400)
    p.add_argument("--no-dr", action="store_true")
    args = p.parse_args()

    if not os.environ.get("G1_PARITY_OBS"):
        raise SystemExit("set G1_PARITY_OBS=1")
    reward_cfg = dict(alive=0.5, upright=0.5, vel=1.5, vel_sig=0.20, fwd_lin=6.0,
                      lat=-0.5, yaw=-0.3, height=-4.0, bz_ref=0.72, act=-0.005,
                      act_rate=-0.01, term=-1.0,
                      vx_cmd=args.vx_cmd, gait_model="", vx_cmd_max=0.0, res_scale=RES_SCALE)
    dr_cfg = {} if (args.eval or args.no_dr) else dict(
        obs_noise=0.02, init_q_band=0.08, init_z_band=0.01, init_tilt_band=0.08,
        init_vel_band=0.10, action_latency_max=2)
    env = G1NewtonWalkEnv(args.envs, reward_cfg=reward_cfg, dr_cfg=dr_cfg)
    N, tdev = args.envs, env.tdev
    torch.manual_seed(0)
    ac = AC().to(tdev)
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)

    if args.eval:
        ac.load_state_dict(torch.load(args.save, map_location=tdev)); ac.eval()
        obs = env.reset()
        survived = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        first_fall = torch.zeros(env.n, dtype=torch.int32, device=tdev)
        dist = torch.zeros(env.n, device=tdev)
        for step in range(args.eval_steps):
            with torch.no_grad():
                mu, _, _ = ac(obs)
            alive = (first_fall == 0)
            obs, _, done, info = env.step(mu)
            dist += info["vx"] * DT * alive.float()
            survived += (~done).to(torch.int32)
            newly = done & (first_fall == 0)
            first_fall = torch.where(newly, torch.full_like(first_fall, step + 1), first_fall)
        s = survived.cpu().numpy(); ff = first_fall.cpu().numpy(); d = dist.cpu().numpy()
        nf = (ff == 0)
        print(f"[walk-repro-eval] envs={env.n} steps={args.eval_steps} "
              f"survival mean={s.mean():.1f} median={np.median(s):.0f} "
              f"never_fell={nf.mean():.2f}  fwd_dist mean={d.mean():.2f}m median={np.median(d):.2f}m "
              f"max={d.max():.2f}m")
        return

    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    obs = env.reset()
    obs_buf = torch.zeros(args.rollout, N, WALK_OBS_DIM, device=tdev)
    act_buf = torch.zeros(args.rollout, N, NJ, device=tdev)
    logp_buf = torch.zeros(args.rollout, N, device=tdev)
    rew_buf = torch.zeros(args.rollout, N, device=tdev)
    done_buf = torch.zeros(args.rollout, N, device=tdev)
    val_buf = torch.zeros(args.rollout, N, device=tdev)
    vx_buf = torch.zeros(args.rollout, N, device=tdev)
    total, t0 = 0, time.time()
    for it in range(1, args.iters + 1):
        for k in range(args.rollout):
            with torch.no_grad():
                mu, v, log_std = ac(obs)
                dist_ = torch.distributions.Normal(mu, log_std.exp())
                a = dist_.sample(); logp = dist_.log_prob(a).sum(-1)
            obs_buf[k], act_buf[k], logp_buf[k], val_buf[k] = obs, a, logp, v
            obs, rew, done, info = env.step(a)
            rew_buf[k], done_buf[k] = rew, done.float(); vx_buf[k] = info["vx"]; total += N
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
        of, af = obs_buf.reshape(-1, WALK_OBS_DIM), act_buf.reshape(-1, NJ)
        lf, advf, rf = logp_buf.reshape(-1), adv.reshape(-1), ret.reshape(-1)
        for _e in range(4):
            mu, v, log_std = ac(of)
            dist_ = torch.distributions.Normal(mu, log_std.exp())
            nl = dist_.log_prob(af).sum(-1); ratio = (nl - lf).exp()
            s1 = ratio * advf; s2 = torch.clamp(ratio, 0.8, 1.2) * advf
            loss = (-torch.min(s1, s2).mean() + 0.5 * ((v - rf) ** 2).mean()
                    - _ENT_COEF * dist_.entropy().sum(-1).mean())
            opt.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(ac.parameters(), 0.5); opt.step()
        if it % 10 == 0 or it == 1:
            print(f"it {it:4d}  ep_rew/step~{rew_buf.mean().item():+.3f}  "
                  f"meanV {val_buf.mean().item():+.2f}  vx~{vx_buf.mean().item():+.2f}  "
                  f"steps {total:,}  {total/max(time.time()-t0,1e-6):,.0f}/s")
    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save} ({total:,} steps, {time.time()-t0:.1f}s)")
    onnx_path = Path(args.save).with_suffix(".onnx")
    cpu = AC(); cpu.load_state_dict({k: v.cpu() for k, v in ac.state_dict().items()})

    class Deploy(nn.Module):
        def __init__(s, pi): super().__init__(); s.pi = pi
        def forward(s, o): return torch.clamp(s.pi(o), -1.0, 1.0)
    with contextlib.redirect_stdout(io.StringIO()):
        torch.onnx.export(Deploy(cpu.pi).eval(), torch.zeros(1, WALK_OBS_DIM), str(onnx_path),
                          input_names=["obs"], output_names=["action"],
                          dynamic_axes={"obs": {0: "batch"}, "action": {0: "batch"}},
                          opset_version=17)
    print(f"exported ONNX -> {onnx_path}")


if __name__ == "__main__":
    main()
