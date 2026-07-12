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

"""Multiprocess PPO trainer on the EXACT OmniSim deploy engine: Newton's
SolverMuJoCo (use_mujoco_cpu=True). This is the only fully faithful engine --
plain `mujoco` mj_step and mujoco_warp both diverge from SolverMuJoCo.step()
(verified: a policy trained on plain-mujoco hits 0.5 m/s there but stands
still under SolverMuJoCo; gait-only even flips sign). SolverMuJoCo.step() does
extra control-application + contact-conversion + state-sync that changes the
contact dynamics, so we train on it directly.

SolverMuJoCo-cpu is ~1456 steps/s single-env and does NOT speed up with
threads (GIL/warp contention). So we fan out across PROCESSES: W worker
processes, each holding K independent single-env solvers, running the policy
(tiny numpy MLP -- no torch in workers) + physics locally and returning
rollouts. Main process runs the PPO update on the pooled W*K envs. Workers
force CPU-only warp (CUDA_VISIBLE_DEVICES="") so they don't touch the GPU a
concurrent session may be using.

Run:
    python projects/policies/research/training/newton_solver_trainer.py \
        --workers 8 --envs-per-worker 8 --iters 400 --rollout 24 \
        --init-from projects/policies/research/training/runs/cpu_spot/policy_cpu.pt \
        --save projects/policies/research/training/runs/newton_spot/policy.pt
    python projects/policies/research/training/newton_solver_trainer.py --eval \
        --save projects/policies/research/training/runs/newton_spot/policy.pt
"""
from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(Path(__file__).resolve().parent))  # so workers can import spot_native

# ---- shared layout (mirror of the trainers / deploy controller) ----
NJ = 12
QPOS_J0 = 7
QVEL_J0 = 6
OBS_DIM = 49
ACTION_SCALE = 0.15
PITCH_TRIM = 0.20
STAND_Z = 0.7
ROLL_FAIL = 1.0
PITCH_FAIL = 1.0
BZ_FAIL = 0.30
MAX_EP = 1024
LEGS = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_ORDER = [(leg, j) for leg in LEGS for j in ("hip_x", "hip_y", "knee")]
NOMINAL = np.zeros(NJ, np.float32)
for _i, (_leg, _j) in enumerate(JOINT_ORDER):
    NOMINAL[_i] = (0.30 if "left" in _leg else -0.30) if _j == "hip_x" \
        else (0.30 if _j == "hip_y" else -0.60)
CPG_PHASE = np.array([0.0 if l in ("front_left", "rear_right") else 0.5
                      for (l, _j) in JOINT_ORDER], np.float32)
IS_HIPY = np.array([j == "hip_y" for (_l, j) in JOINT_ORDER], bool)
IS_KNEE = np.array([j == "knee" for (_l, j) in JOINT_ORDER], bool)
TRIM_VEC = IS_HIPY.astype(np.float32) * PITCH_TRIM

# default CPG (sidecar values for the Spot)
CPG_FREQ = 2.6
CPG_HIPY = 0.16
CPG_KNEE = 0.22


def quat_to_R(q):  # (x,y,z,w)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def quat_rp(q):  # roll, pitch from (x,y,z,w)
    R = quat_to_R(q)
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    return roll, pitch


# ============================ worker side ============================
_W = {}  # worker globals


class NewtonEnv:
    """One faithful Spot env on Newton SolverMuJoCo (use_mujoco_cpu=True)."""

    def __init__(self, mjcf, cfg):
        import warp as wp
        import newton
        from spot_native import build_spot_native
        self.wp, self.newton = wp, newton
        # NATIVE Newton joints (target_ke/target_kd POSITION_VELOCITY) -- the
        # exact deploy control path. add_mjcf would import MuJoCo actuators
        # that joint_target_pos cannot drive (control silently ignored).
        self.model = build_spot_native(mu=cfg.get("mu", 1.0),
                                       ke=cfg.get("ke", 20.0), kd=cfg.get("kd", 3.0))
        self.solver = newton.solvers.SolverMuJoCo(self.model, use_mujoco_cpu=True)
        self.state_a = self.model.state()
        self.state_b = self.model.state()
        self.control = self.model.control()
        self.contacts = self.model.contacts() if hasattr(self.model, "contacts") else None
        self.cfg = cfg
        jq = self.model.joint_q.numpy()
        jq[0:3] = [0, 0, STAND_Z]
        jq[3:7] = [0, 0, 0, 1.0]
        jq[7:7 + NJ] = NOMINAL
        self.seed_jq = jq.copy()
        self.dt = cfg["dt"]
        self.reset()

    def reset(self):
        self.model.joint_q.assign(self.seed_jq)
        zqd = self.model.joint_qd.numpy() * 0.0
        for st in (self.state_a, self.state_b):
            st.joint_q.assign(self.seed_jq)
            st.joint_qd.assign(zqd)
        self.newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_a)
        self.phase = 0.0
        self.ep = 0
        self.last_action = np.zeros(NJ, np.float32)
        return self._obs()

    def _obs(self):
        bq = self.state_a.body_q.numpy()[0]
        bqd = self.state_a.body_qd.numpy()[0]
        R = quat_to_R(bq[3:7])
        v_lin = bqd[0:3].astype(np.float32)
        v_ang = (R.T @ bqd[3:6]).astype(np.float32)
        pg = (R.T @ np.array([0.0, 0.0, -1.0])).astype(np.float32)
        jq = self.state_a.joint_q.numpy()
        jqd = self.state_a.joint_qd.numpy()
        q = jq[7:7 + NJ].astype(np.float32)
        qd = jqd[6:6 + NJ].astype(np.float32)
        clock = (self.phase % 1.0)
        obs = np.concatenate([v_lin, v_ang, pg, q, qd, self.last_action,
                              [self.cfg["vx"], 0.0, 0.0], [clock]]).astype(np.float32)
        self._cache = (bq, bqd, R, q, qd)
        return np.clip(np.nan_to_num(obs, nan=0, posinf=10, neginf=-10), -10, 10)

    def step(self, action):
        action = np.clip(action, -1, 1).astype(np.float32)
        self.phase += self.cfg["cpg_freq"] * self.dt
        th = 2 * math.pi * (self.phase + CPG_PHASE)
        s = np.maximum(0.0, np.sin(th))
        cpg = (IS_HIPY * self.cfg["cpg_hipy"] * np.cos(th)
               - IS_KNEE * self.cfg["cpg_knee"] * (s * s)).astype(np.float32)
        target = NOMINAL + TRIM_VEC + cpg + ACTION_SCALE * action
        tp = self.control.joint_target_pos.numpy()
        tp[6:6 + NJ] = target
        self.control.joint_target_pos.assign(tp)
        self.state_a.clear_forces()
        if self.contacts is not None:
            self.model.collide(self.state_a, self.contacts)
            self.solver.step(self.state_a, self.state_b, self.control, self.contacts, self.dt)
        else:
            self.solver.step(self.state_a, self.state_b, self.control, None, self.dt)
        self.state_a, self.state_b = self.state_b, self.state_a
        self.ep += 1
        self.last_action = action

        bq = self.state_a.body_q.numpy()[0]
        bqd = self.state_a.body_qd.numpy()[0]
        bz = float(bq[2])
        roll, pitch = quat_rp(bq[3:7])
        vx = float(bqd[0]); vy = float(bqd[1])
        c = self.cfg
        lin_err = (vx - c["vx"]) ** 2 + vy ** 2
        r_lin = c["lin"] * math.exp(-lin_err / c["lin_scale"])
        vx_eff = min(max(0.0, vx - 0.05), c["vx"])
        # Couple the survival ("alive") credit to forward progress so that
        # STANDING STILL is no longer a safe high-value optimum. A frozen
        # stance gets only 0.3*alive; moving at the target speed gets full
        # alive. Without this, PPO converges to "stand still" because that
        # guarantees the flat alive bonus with zero fall risk (verified: the
        # deterministic policy stood still at vx=-0.03 with 100% survival).
        fwd_frac = min(max(vx / max(c["vx"], 1e-3), 0.0), 1.0)
        r_alive = c["alive"] * (0.3 + 0.7 * fwd_frac)
        # Yaw-rate + lateral penalties keep it walking STRAIGHT (without them
        # the policy walks forward but curves -> circles in OmniSim). bqd[5] =
        # world yaw rate; vy = lateral velocity.
        r_yaw = c.get("yaw", 0.0) * (float(bqd[5]) ** 2)
        r_lat = c.get("lat", 0.0) * (vy ** 2)
        # NATURAL-GAIT shaping (commit "journey"): a robot that walks looks
        # JUMPY when the chassis bobs vertically and the policy commands
        # jerky residuals. Penalising body-Z deviation from nominal stand
        # height + action rate-of-change produces a smoother, more
        # quadruped-looking trot.
        r_bz = c.get("bz", 0.0) * ((bz - c.get("bz_target", 0.62)) ** 2)
        vz = float(bqd[2])
        r_vz = c.get("vz", -0.3) * (vz ** 2)
        r_ar = c.get("act_rate", 0.0) * float(np.sum((action - self.last_action) ** 2))
        reward = (r_lin + c["vx_bonus"] * vx_eff + r_alive
                  + r_vz - 0.1 * (roll ** 2 + pitch ** 2)
                  - 0.05 * (float(bqd[3]) ** 2 + float(bqd[4]) ** 2)
                  + r_yaw + r_lat + r_bz + r_ar
                  + c["act"] * float(np.sum(action ** 2)))
        hard = (abs(roll) > ROLL_FAIL or abs(pitch) > PITCH_FAIL or bz < BZ_FAIL
                or not np.isfinite(bz))
        done = hard or (self.ep >= c.get("max_ep", MAX_EP))
        if hard:
            reward += c["term"]
        if done:
            obs = self.reset()
        else:
            obs = self._obs()
        return obs, np.float32(reward), done, np.float32(vx)


def _mlp_forward(obs, params):
    """Pure-numpy actor+critic. params = (pi_w, pi_b, vf_w, vf_b, logstd).
    pi_w/vf_w are lists of weight matrices [out,in]; tanh between layers."""
    pi_w, pi_b, vf_w, vf_b, logstd = params

    def run(x, ws, bs):
        h = x
        for k in range(len(ws) - 1):
            h = np.tanh(h @ ws[k].T + bs[k])
        return h @ ws[-1].T + bs[-1]
    mu = run(obs, pi_w, pi_b)
    val = run(obs, vf_w, vf_b)[:, 0]
    std = np.exp(logstd)
    return mu, std, val


def _worker_init(mjcf, k, cfg):
    os.environ["CUDA_VISIBLE_DEVICES"] = ""  # CPU-only warp; don't touch GPU
    os.environ.setdefault("WARP_CACHE_PATH", os.path.join(os.environ.get("TEMP", "/tmp"), f"warp_w{os.getpid()}"))
    import warp as wp
    wp.init()
    _W["envs"] = [NewtonEnv(mjcf, cfg) for _ in range(k)]
    _W["obs"] = np.stack([e._obs() for e in _W["envs"]]).astype(np.float32)
    _W["k"] = k


def _worker_rollout(args):
    params, R, seed = args
    rng = np.random.default_rng(seed)
    envs = _W["envs"]; k = _W["k"]
    obs = _W["obs"]
    O = np.zeros((R, k, OBS_DIM), np.float32)
    A = np.zeros((R, k, NJ), np.float32)
    LP = np.zeros((R, k), np.float32)
    V = np.zeros((R, k), np.float32)
    RW = np.zeros((R, k), np.float32)
    D = np.zeros((R, k), np.float32)
    VX = np.zeros((R, k), np.float32)
    for t in range(R):
        mu, std, val = _mlp_forward(obs, params)
        act = mu + std * rng.standard_normal(mu.shape).astype(np.float32)
        logp = (-0.5 * (((act - mu) / std) ** 2) - np.log(std) - 0.5 * math.log(2 * math.pi)).sum(1)
        O[t] = obs; A[t] = act; LP[t] = logp; V[t] = val
        nobs = np.empty_like(obs)
        for i, e in enumerate(envs):
            no, r, dn, vx = e.step(act[i])
            nobs[i] = no; RW[t, i] = r; D[t, i] = 1.0 if dn else 0.0; VX[t, i] = vx
        obs = nobs
    _W["obs"] = obs
    # bootstrap value
    _, _, lastv = _mlp_forward(obs, params)
    return O, A, LP, V, RW, D, VX, lastv.astype(np.float32)


# ============================ main / PPO ============================
def _params_from_torch(ac):
    pi_w = [ac.pi[0].weight.detach().numpy(), ac.pi[2].weight.detach().numpy(), ac.pi[4].weight.detach().numpy()]
    pi_b = [ac.pi[0].bias.detach().numpy(), ac.pi[2].bias.detach().numpy(), ac.pi[4].bias.detach().numpy()]
    vf_w = [ac.vf[0].weight.detach().numpy(), ac.vf[2].weight.detach().numpy(), ac.vf[4].weight.detach().numpy()]
    vf_b = [ac.vf[0].bias.detach().numpy(), ac.vf[2].bias.detach().numpy(), ac.vf[4].bias.detach().numpy()]
    logstd = ac.logstd.detach().numpy()
    return (pi_w, pi_b, vf_w, vf_b, logstd)


def main():
    import torch
    import torch.nn as nn

    p = argparse.ArgumentParser()
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--envs-per-worker", type=int, default=8)
    p.add_argument("--iters", type=int, default=400)
    p.add_argument("--rollout", type=int, default=24)
    p.add_argument("--mjcf", default=r"C:\tmp\spot_newton_fixed.xml")
    p.add_argument("--vx", type=float, default=0.5)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save", default=str(REPO / "projects/policies/research/training/runs/newton_spot/policy.pt"))
    p.add_argument("--init-from", default=None)
    p.add_argument("--eval", action="store_true")
    p.add_argument("--eval-secs", type=float, default=12.0)
    p.add_argument("--vx-bonus", type=float, default=4.0)
    p.add_argument("--alive", type=float, default=1.0)
    p.add_argument("--lin", type=float, default=2.0)
    p.add_argument("--lin-scale", type=float, default=0.1)
    p.add_argument("--act-pen", type=float, default=-0.002)
    p.add_argument("--cpg-freq", type=float, default=CPG_FREQ)
    p.add_argument("--cpg-hipy", type=float, default=CPG_HIPY)
    p.add_argument("--cpg-knee", type=float, default=CPG_KNEE)
    p.add_argument("--dt", type=float, default=0.016)
    p.add_argument("--ent", type=float, default=0.0, help="entropy bonus coeff (keeps exploration alive)")
    p.add_argument("--reset-std", type=float, default=None, help="re-init logstd after warm-start (e.g. -0.7)")
    p.add_argument("--ckpt-every", type=int, default=0, help="save numbered checkpoint every N iters (0=off)")
    p.add_argument("--train-ke", type=float, default=20.0, help="joint target_ke (match deploy OMNISIM_NEWTON_TARGET_KE)")
    p.add_argument("--train-kd", type=float, default=3.0, help="joint target_kd (match deploy OMNISIM_NEWTON_TARGET_KD)")
    p.add_argument("--mu", type=float, default=1.0, help="ground/foot friction (match deploy)")
    p.add_argument("--yaw-pen", type=float, default=-0.15, help="yaw-rate penalty (keeps it walking straight)")
    p.add_argument("--lat-pen", type=float, default=-0.5, help="lateral-velocity penalty (keeps it walking straight)")
    p.add_argument("--max-ep", type=int, default=MAX_EP, help="episode length in steps (longer = train long-horizon stability)")
    p.add_argument("--bz-pen", type=float, default=0.0, help="penalty on (z-bz_target)^2 -> reduces vertical bob")
    p.add_argument("--bz-target", type=float, default=0.62, help="nominal stand-height target for bz penalty")
    p.add_argument("--vz-pen", type=float, default=-0.3, help="penalty on vertical-velocity^2 (more negative = less bouncing)")
    p.add_argument("--act-rate-pen", type=float, default=0.0, help="penalty on action delta^2 -> smoother control")
    args = p.parse_args()

    class AC(nn.Module):
        def __init__(self):
            super().__init__()
            self.pi = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, NJ))
            self.vf = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                    nn.Linear(256, 128), nn.Tanh(), nn.Linear(128, 1))
            self.logstd = nn.Parameter(-0.5 * torch.ones(NJ))

    ac = AC()
    if args.init_from and Path(args.init_from).exists():
        ac.load_state_dict(torch.load(args.init_from, map_location="cpu"))
        print(f"[warm-start] {args.init_from}")
    if args.reset_std is not None:
        with torch.no_grad():
            ac.logstd.fill_(args.reset_std)
        print(f"[reset-std] logstd <- {args.reset_std}")

    cfg = dict(vx=args.vx, dt=args.dt, cpg_freq=args.cpg_freq, cpg_hipy=args.cpg_hipy,
               cpg_knee=args.cpg_knee, lin=args.lin, lin_scale=args.lin_scale,
               vx_bonus=args.vx_bonus, alive=args.alive, act=args.act_pen, term=-1.0,
               ke=args.train_ke, kd=args.train_kd, mu=args.mu,
               yaw=args.yaw_pen, lat=args.lat_pen, max_ep=args.max_ep,
               bz=args.bz_pen, bz_target=args.bz_target, vz=args.vz_pen,
               act_rate=args.act_rate_pen)

    if args.eval:
        ac.load_state_dict(torch.load(args.save, map_location="cpu"))
        print(f"[newton-eval] loaded {args.save}")
        # single-process deterministic eval (reuse worker env in-process)
        _worker_init(args.mjcf, 16, cfg)
        envs = _W["envs"]; k = _W["k"]; obs = _W["obs"]
        params = _params_from_torch(ac)
        nsteps = int(args.eval_secs / args.dt)
        alive = np.ones(k, bool); ep_len = np.zeros(k, int); vx_sum = np.zeros(k)
        for t in range(nsteps):
            mu, _, _ = _mlp_forward(obs, params)
            nobs = np.empty_like(obs)
            for i, e in enumerate(envs):
                no, r, dn, vx = e.step(mu[i])
                nobs[i] = no
                if alive[i] and t > 20:
                    vx_sum[i] += vx
                if alive[i]:
                    ep_len[i] += 1
                if dn:
                    alive[i] = False
            obs = nobs
            if not alive.any():
                break
        vx_mean = vx_sum / np.maximum(ep_len - 20, 1)
        print(f"[newton-eval] survival mean={ep_len.mean():.0f}/{nsteps} frac_full={(ep_len>=nsteps).mean():.2f}  "
              f"vx mean={vx_mean.mean():+.3f} median={np.median(vx_mean):+.3f} (target {args.vx})")
        return

    W, K = args.workers, args.envs_per_worker
    N = W * K
    ctx = mp.get_context("spawn")
    pool = ctx.Pool(W, initializer=_worker_init, initargs=(args.mjcf, K, cfg))
    print(f"[newton-train] {W} workers x {K} envs = {N} faithful SolverMuJoCo envs")

    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)
    gamma, lam, clip, epochs, mb = 0.99, 0.95, 0.2, 4, 8
    t0 = time.time(); total = 0
    for it in range(1, args.iters + 1):
        params = _params_from_torch(ac)
        res = pool.map(_worker_rollout, [(params, args.rollout, it * 1000 + w) for w in range(W)])
        # stack workers along env axis
        O = np.concatenate([r[0] for r in res], axis=1)
        A = np.concatenate([r[1] for r in res], axis=1)
        LP = np.concatenate([r[2] for r in res], axis=1)
        V = np.concatenate([r[3] for r in res], axis=1)
        R = np.concatenate([r[4] for r in res], axis=1)
        D = np.concatenate([r[5] for r in res], axis=1)
        VX = np.concatenate([r[6] for r in res], axis=1)
        lastv = np.concatenate([r[7] for r in res], axis=0)
        total += args.rollout * N
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
                mu = ac.pi(bO[bi]); std = ac.logstd.exp().expand_as(mu)
                dist = torch.distributions.Normal(mu, std)
                lp = dist.log_prob(bA[bi]).sum(-1)
                ratio = (lp - bLP[bi]).exp()
                a1 = ratio * bAdv[bi]
                a2 = torch.clamp(ratio, 1 - clip, 1 + clip) * bAdv[bi]
                ploss = -torch.min(a1, a2).mean()
                val = ac.vf(bO[bi]).squeeze(-1)
                vloss = ((val - bRet[bi]) ** 2).mean()
                ent = dist.entropy().sum(-1).mean()
                loss = ploss + 0.5 * vloss - args.ent * ent
                opt.zero_grad(); loss.backward()
                nn.utils.clip_grad_norm_(ac.parameters(), 0.5); opt.step()
        if it % 5 == 0 or it == 1:
            sps = total / (time.time() - t0)
            print(f"it {it:4d}  rew/step~{R.mean():+.3f}  vx~{VX[VX!=0].mean() if (VX!=0).any() else 0:+.3f}  "
                  f"meanV {V.mean():+.1f}  steps {total:,}  {sps:,.0f} env-steps/s", flush=True)
        if args.ckpt_every and it % args.ckpt_every == 0:
            ckpt = Path(args.save).with_name(Path(args.save).stem + f"_it{it}.pt")
            ckpt.parent.mkdir(parents=True, exist_ok=True)
            torch.save(ac.state_dict(), ckpt)
            print(f"  [ckpt] {ckpt.name}", flush=True)
    pool.close(); pool.join()
    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total:,} steps in {time.time()-t0:.1f}s)")


if __name__ == "__main__":
    main()
