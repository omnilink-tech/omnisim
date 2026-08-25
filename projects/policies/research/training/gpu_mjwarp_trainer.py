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

"""GPU-batched MuJoCo-Warp PPO trainer for the Newton quadruped.

Physics: mujoco_warp on cuda:0 (N parallel worlds). Model: exported from
Newton's SolverMuJoCo(save_to_mjcf=...) so it's the EXACT physics as the
OmniSim/Newton deploy target (no sim-to-sim gap). Env logic (CPG+pitch-trim
feedforward, residual position control, 49-dim obs, recipe reward,
roll/pitch/height termination, per-world auto-reset) runs in numpy; the
policy is a small MLP (CPU torch -- torch is CPU-only on this box, but the
MLP is tiny so the GPU physics is the win). PPO is a compact batched loop.

This first targets the representative quad (newton_friction_probe.build) to
validate the trainer end-to-end at GPU speed. Porting to the real OmniQuad
model = export OmniQuad's MJCF the same way + match the controller's nominal
pose / CPG phasing exactly.

Run:
    python projects/policies/research/training/gpu_mjwarp_trainer.py --envs 2048 --iters 200
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO / "projects" / "rl" / "tools"))

# ---- joint / obs layout (quad: free root + 12 leg joints) ----
NJ = 12
QPOS_J0 = 7      # leg joints start in qpos
QVEL_J0 = 6      # leg joints start in qvel
OBS_DIM = 49
# nominal stance per joint (build order: 4 legs x [hip_x, hip_y, knee])
LEG_SIGNS = [(+1, +1), (+1, -1), (-1, +1), (-1, -1)]  # (ax, ay) sign
NOMINAL = np.array(
    [v for (ax, ay) in LEG_SIGNS
     for v in ((0.30 if ax > 0 else -0.30), 0.30, -0.60)],
    dtype=np.float32,
)
# trot diagonal phasing per leg (FL,RR together; FR,RL half-cycle)
LEG_PHASE = np.array([0.0, 0.5, 0.5, 0.0], dtype=np.float32)
ACTION_SCALE = 0.15
PITCH_TRIM = 0.20
CPG_FREQ = 1.75
CPG_HIP_Y = 0.13
CPG_KNEE = 0.22
DT = 0.016
STAND_Z = 0.7
ROLL_FAIL = 1.0
PITCH_FAIL = 1.0
BZ_FAIL = 0.30
MAX_EP = 1024


def quat_to_rp(q):  # q: [N,4] (w,x,y,z) -> roll, pitch
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    return roll, pitch


def proj_gravity(q):  # body-frame gravity unit vector, [N,3]
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    # R^T @ (0,0,-1)
    gx = -2 * (x * z - w * y)
    gy = -2 * (y * z + w * x)
    gz = -(1 - 2 * (x * x + y * y))
    return np.stack([gx, gy, gz], axis=1).astype(np.float32)


def classify_joints(mjm):
    """Derive per-leg-joint config from the exported MJCF so the trainer
    matches the EXACT OmniQuad (or quad) model: nominal stance, which joints
    are hip_y (fore/aft, get CPG cos + pitch-trim) vs knee (get CPG lift),
    and each joint's trot phase. Returns (nominal[12], phase[12],
    is_hipy[12], is_knee[12]) in qpos order. Trot pairing: diagonal legs
    (FL,RR)=0.0, (FR,RL)=0.5, keyed off child-body world x,y sign."""
    import mujoco
    d = mujoco.MjData(mjm); mujoco.mj_forward(mjm, d)
    nominal = np.zeros(NJ, np.float32); phase = np.zeros(NJ, np.float32)
    is_hipy = np.zeros(NJ, bool); is_knee = np.zeros(NJ, bool)
    k = 0
    for j in range(1, mjm.njnt):  # skip free root
        if mjm.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        ax = mjm.jnt_axis[j]; rng = mjm.jnt_range[j]
        pos = d.xpos[mjm.jnt_bodyid[j]]
        front = pos[0] > 0; left = pos[1] > 0
        # diagonal trot: FL & RR in phase, FR & RL half-cycle offset
        phase[k] = 0.0 if (front == left) else 0.5
        if rng[0] < -0.9:                        # knee
            nominal[k] = -0.60; is_knee[k] = True
        elif abs(ax[0]) > 0.5:                   # hip_x (lateral, static)
            nominal[k] = 0.30 if rng[1] > 0.1 else -0.30
        else:                                    # hip_y (fore/aft)
            nominal[k] = 0.30; is_hipy[k] = True
        k += 1
    return nominal, phase, is_hipy, is_knee


class BatchedQuadEnv:
    def __init__(self, n, mjcf, fixed_vx=0.5, device="cuda:0", reward_cfg=None,
                 cpg_freq=CPG_FREQ, cpg_hipy=CPG_HIP_Y, cpg_knee=CPG_KNEE, sim_dt=0.0):
        self.cpg_freq, self.cpg_hipy, self.cpg_knee = cpg_freq, cpg_hipy, cpg_knee
        import warp as wp
        import mujoco
        import mujoco_warp as mjw
        self.wp, self.mjw = wp, mjw
        self.n = n
        self.device = wp.get_device(device)
        self.mjm = mujoco.MjModel.from_xml_path(mjcf)
        # Match the deploy control rate: Webots/OmniSim steps Newton at its
        # basicTimeStep (16 ms) and runs one control decision per tick. The
        # MJCF default opt.timestep is 2 ms, so training 1 step/control would
        # give the policy 8x finer control than deploy -> it tips in OmniSim.
        # Setting opt.timestep = sim_dt makes 1 trainer step == 1 deploy tick.
        if sim_dt and sim_dt > 0:
            self.mjm.opt.timestep = sim_dt
        # derive nominal stance + CPG roles from the model (works for the
        # quad and the real OmniQuad; joint order = controller JOINT_ORDER)
        self.nominal, self.phase_off, self.is_hipy, self.is_knee = classify_joints(self.mjm)
        self.trim_vec = (self.is_hipy.astype(np.float32) * PITCH_TRIM)
        mjd = mujoco.MjData(self.mjm)
        mujoco.mj_forward(self.mjm, mjd)
        with wp.ScopedDevice(self.device):
            self.mw_m = mjw.put_model(self.mjm)
            self.mw_d = mjw.put_data(self.mjm, mjd, nworld=n)
        self.fixed_vx = fixed_vx
        self.r = reward_cfg or {}
        # standing seed qpos/qvel (one world template)
        self.seed_qpos = mjd.qpos.copy().astype(np.float32)
        self.seed_qpos[0:3] = [0, 0, STAND_Z]
        self.seed_qpos[3:7] = [1, 0, 0, 0]
        self.seed_qpos[QPOS_J0:QPOS_J0 + NJ] = self.nominal
        self.nq = self.mjm.nq
        self.nv = self.mjm.nv
        self.nu = self.mjm.nu
        self.phase = np.zeros(n, dtype=np.float32)
        self.ep_step = np.zeros(n, dtype=np.int32)
        self.last_action = np.zeros((n, NJ), dtype=np.float32)
        self.vel_cmd = np.zeros((n, 3), dtype=np.float32)
        self.vel_cmd[:, 0] = fixed_vx
        self._reset_all()

    def _write_qpos_qvel(self, qpos, qvel):
        self.mw_d.qpos.assign(qpos)
        self.mw_d.qvel.assign(qvel)

    def _reset_all(self):
        qpos = np.tile(self.seed_qpos, (self.n, 1)).astype(np.float32)
        qvel = np.zeros((self.n, self.nv), dtype=np.float32)
        self._write_qpos_qvel(qpos, qvel)
        self.phase[:] = 0.0
        self.ep_step[:] = 0
        self.last_action[:] = 0.0
        with self.wp.ScopedDevice(self.device):
            self.mjw.forward(self.mw_m, self.mw_d)

    def _read_state(self):
        qpos = self.mw_d.qpos.numpy()
        qvel = self.mw_d.qvel.numpy()
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
        self._reset_all()
        qpos, qvel = self._read_state()
        return self._build_obs(qpos, qvel)

    def step(self, action):  # action: [N,12] in [-1,1]
        action = np.clip(action, -1, 1).astype(np.float32)
        self.phase += self.cpg_freq * DT
        # CPG: hip_y joints get cos fore/aft swing; knees get squared-sine
        # lift; phasing per joint (diagonal trot). All vectorized over joints.
        ph = (self.phase[:, None] + self.phase_off[None, :])
        theta = 2 * math.pi * ph
        s = np.maximum(0.0, np.sin(theta))
        cpg = (self.is_hipy[None, :] * self.cpg_hipy * np.cos(theta)
               - self.is_knee[None, :] * self.cpg_knee * (s * s)).astype(np.float32)
        targets = self.nominal[None, :] + self.trim_vec[None, :] + cpg + ACTION_SCALE * action
        # write interleaved ctrl: [pos,vel] per joint
        ctrl = np.zeros((self.n, self.nu), dtype=np.float32)
        ctrl[:, 0::2] = targets
        self.mw_d.ctrl.assign(ctrl)
        with self.wp.ScopedDevice(self.device):
            self.mjw.step(self.mw_m, self.mw_d)
        qpos, qvel = self._read_state()
        self.ep_step += 1
        self.last_action = action

        bz = qpos[:, 2]
        roll, pitch = quat_to_rp(qpos[:, 3:7])
        vx = qvel[:, 0]
        # ---- reward (recipe) ----
        r = self.r
        v_xy = qvel[:, 0:2]
        lin_err = np.sum((v_xy - self.vel_cmd[:, 0:2]) ** 2, axis=1)
        r_lin = r.get("lin", 1.0) * np.exp(-lin_err / r.get("lin_scale", 0.25))
        vx_eff = np.maximum(0.0, vx - 0.05)
        cap = r.get("vx_cap", 0.0)
        if cap > 0:
            vx_eff = np.minimum(vx_eff, cap)
        r_vxb = r.get("vx_bonus", 0.0) * vx_eff
        r_alive = r.get("alive", 1.0) * np.ones(self.n, dtype=np.float32)
        r_vz = r.get("vz", -0.3) * qvel[:, 2] ** 2
        r_rp = r.get("rp", -0.1) * (roll ** 2 + pitch ** 2)
        r_wxy = r.get("wxy", -0.05) * (qvel[:, 3] ** 2 + qvel[:, 4] ** 2)
        # action-magnitude penalty: discourages railing actions to +/-1
        # (the dt16 policy did this -> jerky/fragile). Smaller, smoother
        # residuals = a more robust gait that survives longer.
        r_act = r.get("act", 0.0) * np.sum(action ** 2, axis=1)
        reward = (r_lin + r_vxb + r_alive + r_vz + r_rp + r_wxy + r_act).astype(np.float32)

        done = ((np.abs(roll) > ROLL_FAIL) | (np.abs(pitch) > PITCH_FAIL) |
                (bz < BZ_FAIL) | (self.ep_step >= MAX_EP))
        # term penalty + auto-reset done worlds
        reward = reward + done.astype(np.float32) * r.get("term", -1.0) * \
            (((np.abs(roll) > ROLL_FAIL) | (np.abs(pitch) > PITCH_FAIL) | (bz < BZ_FAIL)).astype(np.float32))
        if done.any():
            qpos2 = qpos.copy()
            qvel2 = qvel.copy()
            idx = np.where(done)[0]
            qpos2[idx] = self.seed_qpos
            qvel2[idx] = 0.0
            self._write_qpos_qvel(qpos2, qvel2)
            self.phase[idx] = 0.0
            self.ep_step[idx] = 0
            self.last_action[idx] = 0.0
            with self.wp.ScopedDevice(self.device):
                self.mjw.forward(self.mw_m, self.mw_d)
            qpos, qvel = self._read_state()
        obs = self._build_obs(qpos, qvel)
        return obs, reward, done, {"vx": vx, "bz": bz, "roll": roll, "pitch": pitch}


# ----------------------- compact batched PPO (CPU torch) -----------------------
def main():
    import torch
    import torch.nn as nn

    p = argparse.ArgumentParser()
    p.add_argument("--envs", type=int, default=2048)
    p.add_argument("--iters", type=int, default=150)
    p.add_argument("--rollout", type=int, default=24)
    p.add_argument("--mjcf", default=r"C:\tmp\quad_newton_export.xml")
    p.add_argument("--fixed-vx", type=float, default=0.5)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--save", default=str(REPO / "projects/policies/research/training/runs/gpu_quad/policy.pt"))
    p.add_argument("--eval", action="store_true", help="load --save and run a deterministic eval")
    p.add_argument("--eval-steps", type=int, default=1024)
    p.add_argument("--vx-bonus", type=float, default=4.0, help="capped forward-speed reward weight")
    p.add_argument("--alive", type=float, default=1.0)
    p.add_argument("--lin", type=float, default=2.0, help="velocity-tracking weight")
    p.add_argument("--lin-scale", type=float, default=0.1)
    p.add_argument("--init-from", default=None, help="warm-start policy .pt (velocity curriculum)")
    p.add_argument("--cpg-freq", type=float, default=CPG_FREQ)
    p.add_argument("--cpg-hipy", type=float, default=CPG_HIP_Y)
    p.add_argument("--sim-dt", type=float, default=0.0,
                   help="override MuJoCo opt.timestep to match deploy (Webots=0.016); 0=leave MJCF default")
    p.add_argument("--act-pen", type=float, default=0.0, help="action-magnitude penalty weight (<=0) for a smoother/robust gait")
    args = p.parse_args()

    # ensure the quad MJCF exists (export from Newton if missing)
    if not Path(args.mjcf).exists():
        import warp as wp, newton
        from newton_friction_probe import build
        wp.init()
        newton.solvers.SolverMuJoCo(build(mu=2.0)[0], use_mujoco_cpu=True, save_to_mjcf=args.mjcf)

    reward_cfg = dict(lin=args.lin, lin_scale=args.lin_scale, vx_bonus=args.vx_bonus,
                      vx_cap=args.fixed_vx, alive=args.alive, vz=-0.3, rp=-0.1,
                      wxy=-0.05, term=-1.0, act=args.act_pen)
    env = BatchedQuadEnv(args.envs, args.mjcf, fixed_vx=args.fixed_vx, reward_cfg=reward_cfg,
                         cpg_freq=args.cpg_freq, cpg_hipy=args.cpg_hipy, sim_dt=args.sim_dt)
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
        surv = ep_len
        print(f"[gpu-eval] {Path(args.save).name}  envs={N}  eval_steps={args.eval_steps}")
        print(f"  survival steps: mean={surv.mean():.0f} median={np.median(surv):.0f} "
              f"max={surv.max()} frac_full={(surv>=args.eval_steps).mean():.2f}")
        print(f"  forward vx (m/s): mean={vx_mean.mean():+.3f} median={np.median(vx_mean):+.3f} "
              f"(target {args.fixed_vx})  frac in[0.3,0.7]={((vx_mean>0.3)&(vx_mean<0.7)).mean():.2f}")
        return
    if args.init_from and Path(args.init_from).exists():
        ac.load_state_dict(torch.load(args.init_from, map_location="cpu"))
        print(f"[warm-start] loaded {args.init_from}")
    opt = torch.optim.Adam(ac.parameters(), lr=args.lr)
    gamma, lam, clip, epochs, mb = 0.99, 0.95, 0.2, 4, 8

    obs = env.reset()
    t0 = time.time()
    total_steps = 0
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
            obs = nobs
            total_steps += N
        with torch.no_grad():
            _, _, lastv = ac(torch.from_numpy(obs))
        lastv = lastv.numpy()
        # GAE
        O = np.stack(O); A = np.stack(A); LP = np.stack(LP)
        V = np.stack(V); R = np.stack(R); D = np.stack(D)
        adv = np.zeros_like(R); gae = np.zeros(N, dtype=np.float32)
        for t in reversed(range(args.rollout)):
            nv = lastv if t == args.rollout - 1 else V[t + 1]
            delta = R[t] + gamma * nv * (1 - D[t]) - V[t]
            gae = delta + gamma * lam * (1 - D[t]) * gae
            adv[t] = gae
        ret = adv + V
        # flatten
        bO = torch.from_numpy(O.reshape(-1, OBS_DIM))
        bA = torch.from_numpy(A.reshape(-1, NJ))
        bLP = torch.from_numpy(LP.reshape(-1))
        bAdv = torch.from_numpy(((adv - adv.mean()) / (adv.std() + 1e-8)).reshape(-1))
        bRet = torch.from_numpy(ret.reshape(-1))
        n = bO.shape[0]
        for _ in range(epochs):
            perm = torch.randperm(n)
            for s in range(0, n, n // mb):
                bi = perm[s:s + n // mb]
                mu, std, val = ac(bO[bi])
                dist = torch.distributions.Normal(mu, std)
                lp = dist.log_prob(bA[bi]).sum(-1)
                ratio = (lp - bLP[bi]).exp()
                a1 = ratio * bAdv[bi]
                a2 = torch.clamp(ratio, 1 - clip, 1 + clip) * bAdv[bi]
                ploss = -torch.min(a1, a2).mean()
                vloss = ((val - bRet[bi]) ** 2).mean()
                ent = dist.entropy().sum(-1).mean()
                loss = ploss + 0.5 * vloss - 0.0 * ent
                opt.zero_grad(); loss.backward()
                nn.utils.clip_grad_norm_(ac.parameters(), 0.5); opt.step()
        if it % 5 == 0 or it == 1:
            sps = total_steps / (time.time() - t0)
            print(f"it {it:4d}  ep_rew/step~{R.mean():+.3f}  meanV {V.mean():+.2f}  "
                  f"steps {total_steps:,}  {sps:,.0f} env-steps/s", flush=True)
    Path(args.save).parent.mkdir(parents=True, exist_ok=True)
    torch.save(ac.state_dict(), args.save)
    print(f"saved {args.save}  ({total_steps:,} steps in {time.time()-t0:.1f}s)")


if __name__ == "__main__":
    main()
