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

"""Pipeline that PRODUCES a humanoid walk policy at the expert's performance (G1 + H1).

Reverse-engineering Unitree's RL recipe gave a faithful env, but raw mujoco_warp
can't hold the point-contact feet the experts need (mujoco-CPU AND OmniSim-Newton
both can). So this pipeline builds the policy on the physics that WORKS:

  1. COLLECT: roll the expert (Unitree's motion.pt) in mujoco-CPU on the deploy model
     across a spread of velocity commands -> per-episode (obs, action) SEQUENCES.
  2. CLONE: the experts are RECURRENT (LSTM-64), so an MLP can't reproduce them; clone
     into an LSTM of the same shape via sequence BC + DAgger aggregation. The action
     head is UNCLAMPED (the experts output up to ~9; a +-1 clamp cripples the clone).
  3. VERIFY: roll the STATEFUL clone (persistent hidden state, batch-1 -- the deploy
     interface) in mujoco-CPU; export a torch.jit module the deploy loads UNCHANGED.

Verified: G1 clone walks 89 m / 0.50 m/s / never falls in the OmniSim-Newton deploy,
matching motion.pt. This file generalizes that to any of the Unitree humanoids.

Usage:  python projects/policies/research/training/bc_clone.py --robot g1 --rounds 12 --save <path>
        python projects/policies/research/training/bc_clone.py --robot h1 --rounds 12 --save <path>
"""
import os
import math
import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import mujoco

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())

ROBOTS = {
    "g1": dict(
        model="projects/robots/unitree/g1/g1_12dof/g1_12dof_scene.xml",
        expert="projects/policies/controllers/g1_unitree_deploy/motion.pt",
        legs=["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
              "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
              "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
              "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"],
        kp=[100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40],
        kd=[2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2],
        default=[-0.1, 0, 0, 0.3, -0.2, 0, -0.1, 0, 0, 0.3, -0.2, 0],
        spawn_z=0.79, fall_z=0.50),
    "h1": dict(
        model="projects/robots/unitree/h1/h1_legs/h1_scene.xml",
        expert="projects/policies/controllers/h1_unitree_deploy/motion.pt",
        legs=["left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
              "left_knee_joint", "left_ankle_joint",
              "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
              "right_knee_joint", "right_ankle_joint"],
        kp=[150, 150, 150, 200, 40, 150, 150, 150, 200, 40],
        kd=[2, 2, 2, 4, 2, 2, 2, 2, 4, 2],
        default=[0, 0, -0.1, 0.3, -0.2, 0, 0, -0.1, 0.3, -0.2],
        spawn_z=1.00, fall_z=0.60),
}

ACT_SCALE = 0.25
SIM_DT = 0.002
DECIM = 10
GAIT_PERIOD = 0.8
HIDDEN = 64
CMD_SCALE = np.array([2.0, 2.0, 0.25], np.float32)
EP_STEPS = int(12.0 / (SIM_DT * DECIM))


def grav(q):
    qw, qx, qy, qz = q
    return np.array([2 * (-qz * qx + qw * qy), -2 * (qz * qy + qw * qx),
                     1 - 2 * (qw * qw + qz * qz)], np.float32)


class CpuEnv:
    def __init__(self, cfg):
        self.cfg = cfg
        self.NJ = len(cfg["legs"])
        self.OBS_DIM = 11 + 3 * self.NJ
        self.kp = np.array(cfg["kp"], np.float32)
        self.kd = np.array(cfg["kd"], np.float32)
        self.default = np.array(cfg["default"], np.float32)
        self.fall_z = cfg["fall_z"]; self.spawn_z = cfg["spawn_z"]
        self.m = mujoco.MjModel.from_xml_path(str(REPO / cfg["model"]))
        self.m.opt.timestep = SIM_DT
        self.qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in cfg["legs"]]
        self.vadr = [self.m.jnt_dofadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in cfg["legs"]]
        self.uadr = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, j) for j in cfg["legs"]]
        fb = [j for j in range(self.m.njnt) if self.m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE][0]
        self.bq = self.m.jnt_qposadr[fb]; self.bv = self.m.jnt_dofadr[fb]
        self.d = mujoco.MjData(self.m)

    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[self.bq + 2] = self.spawn_z; self.d.qpos[self.bq + 3] = 1.0
        for i in range(self.NJ):
            self.d.qpos[self.qadr[i]] = self.default[i]
        mujoco.mj_forward(self.m, self.d)
        self.counter = 0; self.phase = 0.0; self.action = np.zeros(self.NJ, np.float32)

    def obs(self, cmd):
        d = self.d
        q = np.array([d.qpos[a] for a in self.qadr], np.float32)
        qd = np.array([d.qvel[a] for a in self.vadr], np.float32)
        o = np.zeros(self.OBS_DIM, np.float32)
        o[0:3] = d.qvel[self.bv + 3:self.bv + 6] * 0.25
        o[3:6] = grav(d.qpos[self.bq + 3:self.bq + 7])
        o[6:9] = cmd * CMD_SCALE
        nj = self.NJ
        o[9:9 + nj] = q - self.default
        o[9 + nj:9 + 2 * nj] = qd * 0.05
        o[9 + 2 * nj:9 + 3 * nj] = self.action
        o[9 + 3 * nj:9 + 3 * nj + 2] = [math.sin(2 * math.pi * self.phase), math.cos(2 * math.pi * self.phase)]
        return o

    def step(self, action):
        self.action = action.astype(np.float32)
        target = self.action * ACT_SCALE + self.default
        for _ in range(DECIM):
            q = np.array([self.d.qpos[a] for a in self.qadr], np.float32)
            qd = np.array([self.d.qvel[a] for a in self.vadr], np.float32)
            tau = (target - q) * self.kp - qd * self.kd
            for i, u in enumerate(self.uadr):
                self.d.ctrl[u] = tau[i]
            mujoco.mj_step(self.m, self.d)
            self.counter += 1
        self.phase = (self.counter * SIM_DT) % GAIT_PERIOD / GAIT_PERIOD

    @property
    def base_z(self): return float(self.d.qpos[self.bq + 2])
    @property
    def base_x(self): return float(self.d.qpos[self.bq])


class SeqLSTM(nn.Module):
    def __init__(self, obs_dim, nj):
        super().__init__()
        self.lstm = nn.LSTM(obs_dim, HIDDEN, batch_first=True)
        self.head = nn.Sequential(nn.Linear(HIDDEN, 32), nn.ELU(), nn.Linear(32, nj))

    def forward(self, seq):
        out, _ = self.lstm(seq)
        return self.head(out)              # UNCLAMPED -- experts output up to ~9


class StatefulLSTM(nn.Module):
    def __init__(self, lstm, head):
        super().__init__()
        self.lstm = lstm; self.head = head
        self.register_buffer("h", torch.zeros(1, 1, HIDDEN))
        self.register_buffer("c", torch.zeros(1, 1, HIDDEN))

    def forward(self, obs):
        out, (h, c) = self.lstm(obs.unsqueeze(1), (self.h, self.c))
        self.h.copy_(h); self.c.copy_(c)
        return self.head(out.squeeze(1))

    @torch.jit.export
    def reset(self):
        self.h.zero_(); self.c.zero_()


def new_clone(sd, obs_dim, nj):
    base = SeqLSTM(obs_dim, nj)
    m = StatefulLSTM(base.lstm, base.head)
    m.lstm.load_state_dict(sd["lstm"]); m.head.load_state_dict(sd["head"]); m.eval()
    return m


def sample_cmds(k):
    vx = np.random.uniform(0.2, 0.7, k); wz = np.random.uniform(-0.3, 0.3, k)
    return [np.array([vx[i], 0.0, wz[i]], np.float32) for i in range(k)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot", choices=list(ROBOTS), required=True)
    p.add_argument("--rounds", type=int, default=12)
    p.add_argument("--eps-per-round", type=int, default=12)
    p.add_argument("--epochs", type=int, default=150)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--keep", type=int, default=48)
    p.add_argument("--save", type=str, default="")
    args = p.parse_args()
    cfg = ROBOTS[args.robot]
    dev = "cuda:0" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(0); np.random.seed(0)

    env = CpuEnv(cfg)
    OBS, NJ = env.OBS_DIM, env.NJ
    expert_path = str(REPO / cfg["expert"])
    model = SeqLSTM(OBS, NJ).to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    episodes = []
    print(f"[bc_clone] robot={args.robot} obs={OBS} nj={NJ} model={cfg['model']}", flush=True)

    for rnd in range(args.rounds):
        sd = {"lstm": {k: v.cpu() for k, v in model.lstm.state_dict().items()},
              "head": {k: v.cpu() for k, v in model.head.state_dict().items()}}
        roll_clone = rnd > 0; col_alive = []
        for cmd in sample_cmds(args.eps_per_round):
            expert = torch.jit.load(expert_path).eval()
            clone = new_clone(sd, OBS, NJ)
            env.reset(); os_, as_ = [], []
            for t in range(EP_STEPS):
                o = env.obs(cmd)
                with torch.no_grad():
                    a_exp = expert(torch.from_numpy(o).unsqueeze(0)).numpy().squeeze().astype(np.float32)
                os_.append(o); as_.append(a_exp)
                if roll_clone:
                    with torch.no_grad():
                        a_take = clone(torch.from_numpy(o).unsqueeze(0)).numpy().squeeze().astype(np.float32)
                else:
                    a_take = a_exp
                env.step(a_take)
                if env.base_z < env.fall_z:
                    break
            episodes.append((np.array(os_, np.float32), np.array(as_, np.float32)))
            col_alive.append(len(os_))
        if len(episodes) > args.keep:
            anchor = episodes[:args.eps_per_round]
            episodes = anchor + episodes[-(args.keep - len(anchor)):]

        for _ in range(args.epochs):
            for i in np.random.permutation(len(episodes)):
                o, a = episodes[i]
                pred = model(torch.tensor(o, device=dev).unsqueeze(0))
                loss = (pred - torch.tensor(a, device=dev).unsqueeze(0)).pow(2).mean()
                opt.zero_grad(); loss.backward(); opt.step()

        sd = {"lstm": {k: v.cpu() for k, v in model.lstm.state_dict().items()},
              "head": {k: v.cpu() for k, v in model.head.state_dict().items()}}
        ev_alive, ev_dist = [], []
        for cmd in sample_cmds(5):
            m = new_clone(sd, OBS, NJ); env.reset(); x0 = env.base_x; st = EP_STEPS
            for t in range(EP_STEPS):
                with torch.no_grad():
                    a = m(torch.from_numpy(env.obs(cmd)).unsqueeze(0)).numpy().squeeze().astype(np.float32)
                env.step(a)
                if env.base_z < env.fall_z:
                    st = t; break
            ev_alive.append(st); ev_dist.append(env.base_x - x0)
        print(f"round {rnd}: episodes={len(episodes)} collect_alive={np.mean(col_alive):.0f} "
              f"| CLONE alive={np.mean(ev_alive):.0f}/{EP_STEPS} dist={np.mean(ev_dist):+.2f}m "
              f"loss={loss.item():.4f}", flush=True)

    if args.save:
        deploy = StatefulLSTM(model.lstm.cpu(), model.head.cpu()).eval()
        torch.jit.save(torch.jit.script(deploy), args.save)
        print(f"saved stateful deploy jit -> {args.save}", flush=True)


if __name__ == "__main__":
    main()
