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

"""Compare the shadow-C policy against previous G1 walk milestones. Each policy
is evaluated UNDER ITS OWN training shadow (in-distribution), and we report the
cross-comparable OUTCOME metrics: survival, walking speed, and the drunk-gait
metrics hip-roll SPLAY + hip-yaw (mean |q|, both legs, deg). Also the sagittal
hip+knee tracking error vs that policy's own shadow. Auto-skips policies whose
obs dim doesn't match the env (older pre-stack architectures).

Rescued from _scratch into the tracked tree (analysis tooling for the
G1-improved-shadow work; see docs/developer/g1-improved-shadow.md). Paths are
repo-relative.

    python projects/policies/control/gait/tools/compare_shadows.py [extra_run_dir lateral yaw]
"""
import os
import sys

import numpy as np
import torch
import torch.nn as nn

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, _REPO)
from projects.policies.research.training.gpu_mjwarp_g1_walk_trainer import BatchedG1StandEnv, NJ, DT

MJCF = os.path.join(_REPO, "projects/robots/unitree/g1/urdf/g1_full_kp100.mjcf.xml")
RUNS = os.path.join(_REPO, "projects/policies/research/training/runs")
L_HR, R_HR, L_HY, R_HY = 1, 7, 2, 8
HIP_KN = [0, 3, 6, 9]
N_ENV, N_STEP = 384, 1500

# (label, run_dir, lateral, yaw). Old walkers = sway/none; the new one = human.
CONFIGS = [
    ("walk18_human_h12", "gpu_g1_walk18_human_h12", "sway", "none"),
    ("walk20_winter_w5",  "gpu_g1_walk20_winter_w5", "sway", "none"),
    ("walk22_swing_t5",   "gpu_g1_walk22_swing_t5",  "sway", "none"),
    ("walk23_style_v3",   "gpu_g1_walk23_style_v3",  "sway", "none"),
    ("walk25_long_c16",   "gpu_g1_walk25_long_c16",  "sway", "none"),
    ("walk26_shape_c8 *champion*", "gpu_g1_walk26_shape_c8", "sway", "none"),
]
# Append the newest shadow-C chunk that exists.
for c in range(8, 0, -1):
    if os.path.exists(f"{RUNS}/gpu_g1_walk30_shadowC_c{c}/policy.pt"):
        CONFIGS.append((f"walk30_shadowC_c{c} *NEW*",
                        f"gpu_g1_walk30_shadowC_c{c}", "human", "human"))
        break
if len(sys.argv) >= 4:   # manual extra: run_dir lateral yaw
    CONFIGS.append((sys.argv[1], sys.argv[1], sys.argv[2], sys.argv[3]))


def build_pi(sd):
    idxs = sorted({int(k.split(".")[1]) for k in sd if k.startswith("pi.")})
    layers = []
    for j, i in enumerate(idxs):
        w = sd[f"pi.{i}.weight"]
        layers.append(nn.Linear(w.shape[1], w.shape[0]))
        if j < len(idxs) - 1:
            layers.append(nn.Tanh())
    net = nn.Sequential(*layers)
    net.load_state_dict({k.split("pi.")[1]: v for k, v in sd.items()
                         if k.startswith("pi.")})
    return net, layers[0].in_features


def eval_policy(run, lateral, yaw):
    reward_cfg = dict(
        gait_model="human",
        gait_params=dict(vx=0.4, freq=1.3, sway=0.05, arm_swing=0.25,
                         style="winter", ramp_s=2.0, winter_hip_scale=0.9,
                         lateral=lateral, yaw=yaw),
        seed_gait=True, max_ep=3000, obs_stack=4, obs_lookahead=[0.1, 0.4],
    )
    env = BatchedG1StandEnv(N_ENV, MJCF, reward_cfg=reward_cfg, dr_cfg={},
                            hold_arms=True)
    obs = env.reset()
    sd = torch.load(f"{RUNS}/{run}/policy.pt", map_location=env.tdev)
    pi, in_dim = build_pi(sd)
    if in_dim != obs.shape[1]:
        return None, in_dim, obs.shape[1]
    pi = pi.to(env.tdev).eval()
    ff = torch.zeros(env.n, dtype=torch.int32, device=env.tdev)
    splay = torch.zeros(env.n, device=env.tdev)
    yawm = torch.zeros(env.n, device=env.tdev)
    sper = torch.zeros(env.n, device=env.tdev)
    vxs = torch.zeros(env.n, device=env.tdev)
    hkerr = torch.zeros(env.n, device=env.tdev)
    alive_n = torch.zeros(env.n, device=env.tdev)
    for k in range(N_STEP):
        with torch.no_grad():
            mu = pi(obs)
        obs, r, done, info = env.step(mu)
        alive = (ff == 0).float()
        q = env.qpos_t.index_select(1, env.qpos_idx_t)
        splay += 0.5 * (q[:, L_HR].abs() + q[:, R_HR].abs()) * alive
        yawm += 0.5 * (q[:, L_HY].abs() + q[:, R_HY].abs()) * alive
        sper += 0.5 * (q[:, L_HR].abs() + q[:, R_HR].abs()) * 0  # placeholder
        vxs += env.qvel_t[:, 0] * alive
        hk = (q[:, HIP_KN] - env._model_legs[:, HIP_KN]).abs().mean(1)
        hkerr += hk * alive
        alive_n += alive
        newly = done & (ff == 0)
        ff = torch.where(newly, torch.full_like(ff, k + 1), ff)
    a = torch.clamp(alive_n, min=1.0)
    ffd = ff.cpu().numpy().astype(float); ffd[ffd == 0] = N_STEP
    return dict(
        surv=ffd.mean() * DT,
        reach=(ffd >= N_STEP).mean() * 100,
        speed=float((vxs / a).mean()),
        splay=float(np.degrees((splay / a).mean().cpu())),
        yaw=float(np.degrees((yawm / a).mean().cpu())),
        hkerr=float(np.degrees((hkerr / a).mean().cpu())),
    ), in_dim, obs.shape[1]


print(f"\n{'policy':<28}{'obs':>5}{'surv(s)':>9}{'%24s':>6}{'speed':>7}"
      f"{'splay°':>8}{'yaw°':>7}{'hip+kn°':>9}")
print("-" * 79)
for label, run, lat, yaw in CONFIGS:
    if not os.path.exists(f"{RUNS}/{run}/policy.pt"):
        print(f"{label:<28}  (missing)")
        continue
    res, indim, envdim = eval_policy(run, lat, yaw)
    if res is None:
        print(f"{label:<28}{indim:>5}  SKIP (obs {indim}!={envdim}, pre-stack arch)")
        continue
    print(f"{label:<28}{indim:>5}{res['surv']:>9.1f}{res['reach']:>6.0f}"
          f"{res['speed']:>7.2f}{res['splay']:>8.1f}{res['yaw']:>7.1f}"
          f"{res['hkerr']:>9.1f}")
print("-" * 79)
print("splay = mean |hip-roll| (drunk-gait metric; human target ~6°, old ~30°). "
      "Each policy evaluated under ITS OWN shadow.")
