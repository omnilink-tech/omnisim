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

"""Gait-parameter optimization for the G1 human gait model.

Searches GaitParams around the current operating point, scoring each
candidate with the deployed policy riding the reference in the GPU batched
env (deterministic, no-DR):

  - survival: mean first-fall step over the eval horizon (hard gate);
  - cost of transport: CoT = positive mechanical power / (m * g * v) --
    THE standard optimality metric of legged locomotion (dimensionless;
    human walking ~0.2, the lower the better);
  - velocity tracking: |mean vx - commanded vx|.

Joint torque is reconstructed from the position/velocity actuator model
(tau = kp*(ctrl - q) - kv*qd, kp=100 kv=5 -- the deploy-matched gains), so
power = sum_j max(tau_j * qd_j, 0).

NOTE the honest caveat: candidates are evaluated under the FIXED policy
(gpu_g1_walk18_human_h12), which was trained at the base parameters -- this
is a LOCAL (trust-region) search; large parameter moves are penalised by
policy mismatch, not just by gait quality. The winner gets one fine-tune
chunk before deploy verification.

Usage:
  python optimize_gait_params.py base          # evaluate the base point
  python optimize_gait_params.py 0 1 2 3       # evaluate candidates by index
  python optimize_gait_params.py report        # print the results table
Results accumulate in _scratch/gait_opt_results.jsonl.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO))

from projects.policies.research.training.gpu_mjwarp_g1_walk_trainer import (  # noqa: E402
    BatchedG1StandEnv, DT, NJ, OBS_DIM,
)

MJCF = str(REPO / "projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml")
POLICY = str(REPO / "projects/policies/research/training/runs/gpu_g1_walk18_human_h12/policy.pt")
RESULTS = REPO / "_scratch/gait_opt_results.jsonl"
MASS = 34.134
G = 9.81
KP, KV = 100.0, 5.0
EVAL_STEPS = 2000
N_ENVS = 1024

# Base = the shipped h12 operating point.
BASE = dict(vx=0.4, freq=1.3, duty=0.6, step_height=0.05, pelvis_height=0.755,
            bob=0.020, sway=0.05, arm_swing=0.25, elbow_bend=0.15,
            ankle_clear=0.08, x0=-0.02, ramp_s=2.0)

# One-at-a-time coordinate perturbations around BASE.
CANDIDATES = [
    {"vx": 0.45},            # 0  a bit faster
    {"vx": 0.35},            # 1  a bit slower
    {"freq": 1.2},           # 2  slower cadence (longer stride at same vx)
    {"freq": 1.4},           # 3  quicker cadence (shorter stride)
    {"duty": 0.57},          # 4  shorter stance
    {"duty": 0.63},          # 5  longer stance
    {"step_height": 0.04},   # 6  lower swing (less lift work)
    {"step_height": 0.065},  # 7  higher swing (more clearance)
    {"pelvis_height": 0.765},  # 8  taller
    {"pelvis_height": 0.745},  # 9  lower
    {"bob": 0.014},          # 10 flatter pelvis arc
    {"bob": 0.026},          # 11 stronger pelvis arc
    {"sway": 0.04},          # 12 less lateral sway
    {"sway": 0.065},         # 13 more lateral sway
    {"x0": -0.03},           # 14 stride center further back
    # Composites of the winning directions (round 2):
    {"vx": 0.35, "x0": -0.03},                            # 15
    {"vx": 0.35, "x0": -0.03, "pelvis_height": 0.765},    # 16
    {"x0": -0.03, "pelvis_height": 0.765},                # 17
]


class AC(nn.Module):
    def __init__(self):
        super().__init__()
        self.pi = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                                nn.Linear(256, 128), nn.Tanh(),
                                nn.Linear(128, NJ))
        self.v = nn.Sequential(nn.Linear(OBS_DIM, 256), nn.Tanh(),
                               nn.Linear(256, 128), nn.Tanh(),
                               nn.Linear(128, 1))
        self.log_std = nn.Parameter(-1.0 * torch.ones(NJ))


def evaluate(params: dict, label: str) -> dict:
    reward_cfg = dict(gait_model="human", gait_params=params,
                      seed_gait=True, max_ep=EVAL_STEPS + 1)
    env = BatchedG1StandEnv(N_ENVS, MJCF, reward_cfg=reward_cfg,
                            dr_cfg={}, hold_arms=True)
    obs = env.reset()
    ac = AC().to(env.tdev)
    ac.load_state_dict(torch.load(POLICY, map_location=env.tdev))
    ac.eval()

    first_fall = torch.zeros(env.n, dtype=torch.int32, device=env.tdev)
    alive_steps = torch.zeros(env.n, device=env.tdev)
    vx_sum = torch.zeros(env.n, device=env.tdev)
    pow_sum = torch.zeros(env.n, device=env.tdev)
    for k in range(EVAL_STEPS):
        with torch.no_grad():
            mu = ac.pi(obs)
        # capture pre-step q/qd + post-write ctrl for torque reconstruction
        obs, r, done, info = env.step(mu)
        q = env.qpos_t.index_select(1, env.qpos_idx_t)
        qd = env.qvel_t.index_select(1, env.qvel_idx_t)
        ctrl = env.ctrl_t.index_select(1, env.ctrl_pos_idx_t)
        tau = KP * (ctrl - q) - KV * qd
        p = torch.clamp(tau * qd, min=0.0).sum(dim=1)      # positive mech power
        alive = (first_fall == 0).float()
        vx_sum += env.qvel_t[:, 0] * alive
        pow_sum += p * alive
        alive_steps += alive
        newly = done & (first_fall == 0)
        first_fall = torch.where(newly, torch.full_like(first_fall, k + 1), first_fall)

    ff = first_fall.cpu().numpy().astype(float)
    ff[ff == 0] = EVAL_STEPS                                # survived the horizon
    a = torch.clamp(alive_steps, min=1.0)
    mean_vx = (vx_sum / a).cpu().numpy()
    mean_pow = (pow_sum / a).cpu().numpy()
    v = float(np.clip(mean_vx.mean(), 1e-3, None))
    cot = float(mean_pow.mean() / (MASS * G * v))
    res = dict(label=label, params=params,
               survival_steps=float(ff.mean()),
               survival_s=float(ff.mean() * DT),
               mean_vx=float(mean_vx.mean()),
               vx_err=float(abs(mean_vx.mean() - params.get("vx", 0.4))),
               mech_power_w=float(mean_pow.mean()),
               cot=cot)
    del env
    torch.cuda.empty_cache()
    return res


def main():
    args = sys.argv[1:]
    if args and args[0] == "report":
        rows = [json.loads(l) for l in RESULTS.read_text().splitlines() if l.strip()]
        rows.sort(key=lambda r: (-r["survival_steps"], r["cot"]))
        print(f"{'label':14s} {'surv_s':>7s} {'vx':>6s} {'CoT':>6s} {'P(W)':>7s}  changed")
        for r in rows:
            ch = {k: v for k, v in r["params"].items() if BASE.get(k) != v}
            print(f"{r['label']:14s} {r['survival_s']:7.1f} {r['mean_vx']:6.3f} "
                  f"{r['cot']:6.3f} {r['mech_power_w']:7.1f}  {ch}")
        return
    todo = []
    if args and args[0] == "base":
        todo.append((dict(BASE), "base"))
        args = args[1:]
    for a in args:
        i = int(a)
        p = dict(BASE)
        p.update(CANDIDATES[i])
        todo.append((p, f"cand{i:02d}"))
    with open(RESULTS, "a") as f:
        for p, label in todo:
            res = evaluate(p, label)
            f.write(json.dumps(res) + "\n")
            f.flush()
            print(f"{label}: surv {res['survival_s']:.1f}s vx {res['mean_vx']:.3f} "
                  f"CoT {res['cot']:.3f}", flush=True)


if __name__ == "__main__":
    main()
