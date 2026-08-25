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

"""Architecture-aware in-trainer STYLE probe for long-run chunks. Infers the
MLP shape from the checkpoint, builds the matching env (stack4 + lookahead
[0.1,0.4] + hip-scale 0.9), phase-bins the achieved leg waveform vs the
model, and prints amplitude ratios (the goal = -> 1.0x). In-trainer style
UNDER-reads deploy style (proxy) but is a valid TREND signal across chunks.

Usage: python measure_style_chunk.py <policy.pt>
"""
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())

sys.path.insert(0, str(_REPO))
from projects.policies.research.training.gpu_mjwarp_g1_walk_trainer import (
    BatchedG1StandEnv, NJ, OBS_DIM,
)

MJCF = str(_REPO / "projects" / "robots" / "unitree" / "g1" / "urdf" / "g1_full_kp100.mjcf.xml")
POLICY = sys.argv[1]
NBINS = 36

# infer pi MLP dims from the checkpoint
sd = torch.load(POLICY, map_location="cpu")
pil = sorted([k for k in sd if k.startswith("pi.") and k.endswith(".weight")],
             key=lambda k: int(k.split(".")[1]))
dims = [sd[pil[0]].shape[1]] + [sd[k].shape[0] for k in pil]
OBS_IN = dims[0]
K = OBS_IN // OBS_DIM if OBS_IN % OBS_DIM == 0 else 4
# OBS_IN = OBS_DIM*K + NJ*L  -> solve for stack/lookahead (long run = 4,2)
for stack in (4, 3, 2, 1):
    rem = OBS_IN - OBS_DIM * stack
    if rem >= 0 and rem % NJ == 0:
        STACK, NLOOK = stack, rem // NJ
        break


def build_pi(dims):
    layers, d = [], dims[0]
    for h in dims[1:-1]:
        layers += [nn.Linear(d, h), nn.Tanh()]
        d = h
    layers += [nn.Linear(d, dims[-1])]
    return nn.Sequential(*layers)


reward_cfg = dict(
    gait_model="human",
    gait_params=dict(vx=0.4, freq=1.3, sway=0.05, arm_swing=0.25,
                     style="winter", ramp_s=2.0, winter_hip_scale=0.9),
    seed_gait=True, max_ep=3000, obs_stack=STACK,
    obs_lookahead=[0.1, 0.4][:NLOOK])
env = BatchedG1StandEnv(512, MJCF, reward_cfg=reward_cfg, dr_cfg={}, hold_arms=True)
obs = env.reset()
assert obs.shape[1] == OBS_IN, f"env obs {obs.shape[1]} != ckpt {OBS_IN}"

pi = build_pi(dims).to(env.tdev)
pi.load_state_dict({k[len("pi."):]: v for k, v in sd.items() if k.startswith("pi.")})
pi.eval()

sum_act = torch.zeros(NBINS, NJ, device=env.tdev)
sum_mod = torch.zeros(NBINS, NJ, device=env.tdev)
cnt = torch.zeros(NBINS, device=env.tdev)
fallen = torch.zeros(env.n, dtype=torch.bool, device=env.tdev)
for k in range(1800):
    ph = env.phase_t.clone()
    with torch.no_grad():
        mu = pi(obs)
    obs, r, done, info = env.step(mu)
    if k < 300:
        fallen |= done
        continue
    alive = ~fallen
    qa = env.qpos_t.index_select(1, env.qpos_idx_t)
    b = torch.clamp((ph / (2 * np.pi) * NBINS).long(), 0, NBINS - 1)
    sum_act.index_add_(0, b[alive], qa[alive])
    sum_mod.index_add_(0, b[alive], env._model_legs[alive])
    cnt.index_add_(0, b[alive], torch.ones(int(alive.sum()), device=env.tdev))

wa = (sum_act / torch.clamp(cnt, min=1).unsqueeze(1)).cpu().numpy()
wm = (sum_mod / torch.clamp(cnt, min=1).unsqueeze(1)).cpu().numpy()
name = {0: "hip", 3: "knee", 4: "ankle"}
tag = POLICY.split("\\")[-2]
amps = []
out = []
for j, nm in name.items():
    amp = wa[:, j].std() / max(wm[:, j].std(), 1e-9)
    amps.append(amp)
    out.append(f"{nm} {amp:.2f}x")
# stance width = leg spread from hip-roll (idx 1=L, 7=R); model ~0
sw = 0.64 * (np.abs(np.sin(wa[:, 1])).mean() + np.abs(np.sin(wa[:, 7])).mean()) * 100
out.append(f"stance {sw:.0f}cm")
print(f"{tag}: " + " | ".join(out) + f"   (stack={STACK} look={NLOOK} "
      f"hid={dims[1:-1]})")
