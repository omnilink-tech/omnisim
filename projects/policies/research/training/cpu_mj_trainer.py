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

"""CPU MuJoCo (mj_step) PPO trainer for the Newton-deploy Spot.

WHY THIS EXISTS (read before touching the GPU trainer):
The GPU trainer (gpu_mjwarp_trainer.py) trains in raw mujoco_warp. A policy
robust there does NOT transfer to OmniSim: OmniSim deploys via Newton's
SolverMuJoCo with use_mujoco_cpu=True (the reference C `mj_step`), and that
engine's contact/friction behaviour differs enough from raw mujoco_warp that
the learned forward gait produces ZERO net motion in deploy (verified with
projects/policies/research/tools/_eval_spot_solver.py: the same CPG+policy moves 0.38 m/s in
raw mjw but stands still under SolverMuJoCo-cpu). mujoco_warp-via-SolverMuJoCo
is even worse -- it can't hold an open-loop stance (falls in ~0.46 s).

So we train on the EXACT deploy engine: plain `mujoco` C `mj_step`, on the
EXACT exported MJCF, at opt.timestep = 0.016 with ONE step per control tick
(Newton's SolverMuJoCo.step(dt) takes a single dt-sized step, no substepping).
Batched across N independent MjData in a thread pool (mj_step releases the
GIL) -> ~70k env-steps/s on this box, on par with the GPU trainer but faithful.

Env logic (obs / CPG+pitch-trim feedforward / residual control / reward /
termination / auto-reset) and the PPO loop are reused verbatim from the GPU
trainer so the only difference is the physics backend.

Run:
    python projects/policies/research/training/cpu_mj_trainer.py --envs 256 --iters 400 \
        --mjcf C:/tmp/spot_newton_fixed.xml --save .../policy_cpu.pt
    python projects/policies/research/training/cpu_mj_trainer.py --eval --save .../policy_cpu.pt
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO / "projects" / "rl" / "training"))

# Reuse all the shared logic from the GPU trainer (obs layout, CPG, joint
# classification, reward shaping, constants) so train-vs-train parity is
# guaranteed and only the physics backend differs.
from gpu_mjwarp_trainer import (  # noqa: E402
    NJ, QPOS_J0, QVEL_J0, OBS_DIM, ACTION_SCALE, PITCH_TRIM, STAND_Z,
    ROLL_FAIL, PITCH_FAIL, BZ_FAIL, MAX_EP, CPG_FREQ, CPG_HIP_Y, CPG_KNEE,
    quat_to_rp, proj_gravity, classify_joints,
)


class CpuMjEnv:
    """Batched plain-mujoco mj_step env. Same obs/CPG/reward/control as the
    GPU trainer's BatchedQuadEnv, but stepped on the reference C solver at
    opt.timestep = sim_dt with one step per control tick (matches Newton's
    SolverMuJoCo-cpu deploy exactly)."""

    def __init__(self, n, mjcf, fixed_vx=0.5, reward_cfg=None,
                 cpg_freq=CPG_FREQ, cpg_hipy=CPG_HIP_Y, cpg_knee=CPG_KNEE,
                 sim_dt=0.016, threads=8):
        import mujoco
        self.mujoco = mujoco
        self.n = n
        self.dt = sim_dt
        self.cpg_freq, self.cpg_hipy, self.cpg_knee = cpg_freq, cpg_hipy, cpg_knee
        self.mjm = mujoco.MjModel.from_xml_path(mjcf)
        self.mjm.opt.timestep = sim_dt  # one dt-step per control tick (Newton parity)
        self.nominal, self.phase_off, self.is_hipy, self.is_knee = classify_joints(self.mjm)
        self.trim_vec = self.is_hipy.astype(np.float32) * PITCH_TRIM
        self.nq, self.nv, self.nu = self.mjm.nq, self.mjm.nv, self.mjm.nu
        self.datas = [mujoco.MjData(self.mjm) for _ in range(n)]
        self.pool = ThreadPoolExecutor(max_workers=threads)
        # standing seed
        d0 = mujoco.MjData(self.mjm)
        mujoco.mj_forward(self.mjm, d0)
        self.seed_qpos = d0.qpos.copy().astype(np.float64)
        self.seed_qpos[0:3] = [0, 0, STAND_Z]
        self.seed_qpos[3:7] = [1, 0, 0, 0]
        self.seed_qpos[QPOS_J0:QPOS_J0 + NJ] = self.nominal
        self.fixed_vx = fixed_vx
        self.r = reward_cfg or {}
        self.phase = np.zeros(n, np.float32)
        self.ep_step = np.zeros(n, np.int32)
        self.last_action = np.zeros((n, NJ), np.float32)
        self.vel_cmd = np.zeros((n, 3), np.float32)
        self.vel_cmd[:, 0] = fixed_vx
        self._reset_idx(np.arange(n))

    def _reset_idx(self, idx):
        mj = self.mujoco
        for i in idx:
            d = self.datas[i]
            d.qpos[:] = self.seed_qpos
            d.qvel[:] = 0.0
            d.ctrl[:] = 0.0
            mj.mj_forward(self.mjm, d)
        self.phase[idx] = 0.0
        self.ep_step[idx] = 0
        self.last_action[idx] = 0.0

    def _gather(self):
        qpos = np.stack([d.qpos for d in self.datas]).astype(np.float32)
        qvel = np.stack([d.qvel for d in self.datas]).astype(np.float32)
        return qpos, qvel

    def _build_obs(self, qpos, qvel):
        vlin = qvel[:, 0:3]
        vang = qvel[:, 3:6]
        pg = proj_gravity(qpos[:, 3:7])
        q = qpos[:, QPOS_J0:QPOS_J0 + NJ]
        qd = qvel[:, QVEL_J0:QVEL_J0 + NJ]
        clock = (self.phase % 1.0)[:, None]
        obs = np.concatenate([vlin, vang, pg, q, qd, self.last_action,
                              self.vel_cmd, clock], axis=1).astype(np.float32)
        return np.clip(np.nan_to_num(obs, nan=0.0, posinf=10, neginf=-10), -10, 10)

    def reset(self):
        self._reset_idx(np.arange(self.n))
        qpos, qvel = self._gather()
        return self._build_obs(qpos, qvel)

    def _step_one(self, args):
        i, ctrl_i = args
        d = self.datas[i]
        d.ctrl[:] = ctrl_i
        self.mujoco.mj_step(self.mjm, d)

    def step(self, action):
        action = np.clip(action, -1, 1).astype(np.float32)
        self.phase += self.cpg_freq * self.dt
        ph = self.phase[:, None] + self.phase_off[None, :]
        theta = 2 * math.pi * ph
        s = np.maximum(0.0, np.sin(theta))
        cpg = (self.is_hipy[None, :] * self.cpg_hipy * np.cos(theta)
               - self.is_knee[None, :] * self.cpg_knee * (s * s)).astype(np.float32)
        targets = self.nominal[None, :] + self.trim_vec[None, :] + cpg + ACTION_SCALE * action
        ctrl = np.zeros((self.n, self.nu), np.float32)
        ctrl[:, 0::2] = targets
        list(self.pool.map(self._step_one, ((i, ctrl[i]) for i in range(self.n))))
        qpos, qvel = self._gather()
        self.ep_step += 1
        self.last_action = action

        bz = qpos[:, 2]
        roll, pitch = quat_to_rp(qpos[:, 3:7])
        vx = qvel[:, 0]
        r = self.r
        v_xy = qvel[:, 0:2]
        lin_err = np.sum((v_xy - self.vel_cmd[:, 0:2]) ** 2, axis=1)
        r_lin = r.get("lin", 1.0) * np.exp(-lin_err / r.get("lin_scale", 0.25))
        vx_eff = np.maximum(0.0, vx - 0.05)
        cap = r.get("vx_cap", 0.0)
        if cap > 0:
            vx_eff = np.minimum(vx_eff, cap)
        r_vxb = r.get("vx_bonus", 0.0) * vx_eff
        r_alive = r.get("alive", 1.0) * np.ones(self.n, np.float32)
        r_vz = r.get("vz", -0.3) * qvel[:, 2] ** 2
        r_rp = r.get("rp", -0.1) * (roll ** 2 + pitch ** 2)
        r_wxy = r.get("wxy", -0.05) * (qvel[:, 3] ** 2 + qvel[:, 4] ** 2)
        r_act = r.get("act", 0.0) * np.sum(action ** 2, axis=1)
        reward = (r_lin + r_vxb + r_alive + r_vz + r_rp + r_wxy + r_act).astype(np.float32)

        hard = (np.abs(roll) > ROLL_FAIL) | (np.abs(pitch) > PITCH_FAIL) | (bz < BZ_FAIL)
        done = hard | (self.ep_step >= MAX_EP)
        reward = reward + done.astype(np.float32) * r.get("term", -1.0) * hard.astype(np.float32)
        if done.any():
            self._reset_idx(np.where(done)[0])
            qpos, qvel = self._gather()
        obs = self._build_obs(qpos, qvel)
        return obs, reward, done, {"vx": vx, "bz": bz, "roll": roll, "pitch": pitch}


def main():
    import torch
    import torch.nn as nn

    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=256)
    p.add_argument("--iters", type=int, default=400)
    p.add_argument("--rollout", type=int, default=24)
    p.add_argument("--threads", type=int, default=8)
    p.add_argument("--mjcf", default=r"C:\tmp\spot_newton_fixed.xml")
    p.add_argument("--fixed-vx", type=float, default=0.5)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save", default=str(REPO / "projects/policies/research/training/runs/cpu_spot/policy.pt"))
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-steps", type=int, default=750)
    p.add_argument("--vx-bonus", type=float, default=4.0)
    p.add_argument("--alive", type=float, default=1.0)
    p.add_argument("--lin", type=float, default=2.0)
    p.add_argument("--lin-scale", type=float, default=0.1)
    p.add_argument("--init-from", default=None)
    p.add_argument("--cpg-freq", type=float, default=2.6)
    p.add_argument("--cpg-hipy", type=float, default=0.16)
    p.add_argument("--cpg-knee", type=float, default=0.22)
    p.add_argument("--sim-dt", type=float, default=0.016)
    p.add_argument("--act-pen", type=float, default=0.0)
    args = p.parse_args()

    reward_cfg = dict(lin=args.lin, lin_scale=args.lin_scale, vx_bonus=args.vx_bonus,
                      vx_cap=args.fixed_vx, alive=args.alive, vz=-0.3, rp=-0.1,
                      wxy=-0.05, term=-1.0, act=args.act_pen)
    env = CpuMjEnv(args.envs, args.mjcf, fixed_vx=args.fixed_vx, reward_cfg=reward_cfg,
                   cpg_freq=args.cpg_freq, cpg_hipy=args.cpg_hipy, cpg_knee=args.cpg_knee,
                   sim_dt=args.sim_dt, threads=args.threads)
    N = args.envs

    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, NJ))
            self.vf = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, 1))
            self.logstd = nn.Parameter(-0.5 * torch.ones(NJ))

        def forward(self, o):
            mu = self.pi(o)
            return mu, self.logstd.exp().expand_as(mu), self.vf(o).squeeze(-1)

    ac = AC()
    if args.eval:
        ac.load_state_dict(torch.load(args.save, map_location="cpu"))
        ac.eval()
        obs = env.reset()
        alive = np.ones(N, bool); ep_len = np.zeros(N, np.int32); vx_sum = np.zeros(N)
        for t in range(args.eval_steps):
            with torch.no_grad():
                mu, _, _ = ac(torch.from_numpy(obs))
            obs, rew, done, info = env.step(mu.numpy())
            still = alive & (t > 20)
            vx_sum += info["vx"] * still
            ep_len += alive.astype(np.int32)
            alive = alive & ~done
            if not alive.any():
                break
        vx_mean = vx_sum / np.maximum(ep_len - 20, 1)
        print(f"[cpu-eval] {Path(args.save).name}  envs={N}  eval_steps={args.eval_steps}")
        print(f"  survival steps: mean={ep_len.mean():.0f} median={np.median(ep_len):.0f} "
              f"max={ep_len.max()} frac_full={(ep_len>=args.eval_steps).mean():.2f}")
        print(f"  forward vx (m/s): mean={vx_mean.mean():+.3f} median={np.median(vx_mean):+.3f} "
              f"(target {args.fixed_vx})  frac in[0.3,0.7]={((vx_mean>0.3)&(vx_mean<0.7)).mean():.2f}")
        return
    if args.init_from and Path(args.init_from).exists():
        ac.load_state_dict(torch.load(args.init_from, map_location="cpu"))
        print(f"[warm-start] loaded {args.init_from}")
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)
    gamma, lam, clip, epochs, mb = 0.99, 0.95, 0.2, 4, 8

    obs = env.reset()
    t0 = time.time(); total_steps = 0
    for it in range(1, args.iters + 1):
        O, A, LP, V, R, D = [], [], [], [], [], []
        for _ in range(args.rollout):
            ot = torch.from_numpy(obs)
            with torch.no_grad():
                mu, std, val = ac(ot)
                dist = torch.distributions.Normal(mu, std)
                act = dist.sample()
                lp = dist.log_prob(act).sum(-1)
            a = act.numpy()
            nobs, rew, done, info = env.step(a)
            O.append(obs); A.append(a); LP.append(lp.numpy()); V.append(val.numpy())
            R.append(rew); D.append(done.astype(np.float32))
            obs = nobs; total_steps += N
        with torch.no_grad():
            _, _, lastv = ac(torch.from_numpy(obs))
        lastv = lastv.numpy()
        O = np.stack(O); A = np.stack(A); LP = np.stack(LP)
        V = np.stack(V); R = np.stack(R); D = np.stack(D)
        adv = np.zeros_like(R); gae = np.zeros(N, np.float32)
        for t in reversed(range(args.rollout)):
            nv = lastv if t == args.rollout - 1 else V[t + 1]
            delta = R[t] + gamma * nv * (1 - D[t]) - V[t]
            gae = delta + gamma * lam * (1 - D[t]) * gae
            adv[t] = gae
        ret = adv + V
        bO = torch.from_numpy(O.reshape(-1, OBS_DIM))
        bA = torch.from_numpy(A.reshape(-1, NJ))
        bLP = torch.from_numpy(LP.reshape(-1))
        bAdv = torch.from_numpy(((adv - adv.mean()) / (adv.std() + 1e-8)).reshape(-1))
        bRet = torch.from_numpy(ret.reshape(-1))
        nN = bO.shape[0]
        for _ in range(epochs):
            perm = torch.randperm(nN)
            for sidx in range(0, nN, nN // mb):
                bi = perm[sidx:sidx + nN // mb]
                mu, std, val = ac(bO[bi])
                dist = torch.distributions.Normal(mu, std)
                lp = dist.log_prob(bA[bi]).sum(-1)
                ratio = (lp - bLP[bi]).exp()
                a1 = ratio * bAdv[bi]
                a2 = torch.clamp(ratio, 1 - clip, 1 + clip) * bAdv[bi]
                ploss = -torch.min(a1, a2).mean()
                vloss = ((val - bRet[bi]) ** 2).mean()
                loss = ploss + 0.5 * vloss
                opt.zero_grad(); loss.backward()
                nn.utils.clip_grad_norm_(ac.parameters(), 0.5); opt.step()
        if it % 5 == 0 or it == 1:
            sps = total_steps / (time.time() - t0)
            print(f"it {it:4d}  rew/step~{R.mean():+.3f}  meanV {V.mean():+.2f}  "
                  f"vx~{np.stack([i for i in [info['vx']]]).mean():+.3f}  "
                  f"steps {total_steps:,}  {sps:,.0f} env-steps/s", flush=True)
    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total_steps:,} steps in {time.time()-t0:.1f}s)")


if __name__ == "__main__":
    main()
