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

"""Quantify gait fidelity: mean |q_actual - q_model| over the leg joints
during a no-DR eval rollout, for a given policy. Lower = the real robot
tracks the kinematic ghost more closely (looks more human)."""
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())

sys.path.insert(0, str(_REPO))
from projects.policies.research.training.gpu_mjwarp_g1_walk_trainer import (
    BatchedG1StandEnv, DT, NJ, OBS_DIM,
)

MJCF = str(_REPO / "projects" / "robots" / "unitree" / "g1" / "urdf" / "g1_full_kp100.mjcf.xml")
POLICY = sys.argv[1]
HIP_KN = [0, 3, 6, 9]          # hips + knees (the visible human signature)


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


reward_cfg = dict(gait_model="human",
                  gait_params=dict(vx=0.4, freq=1.3, sway=0.05, arm_swing=0.25,
                                   style="winter", ramp_s=2.0),
                  seed_gait=True, max_ep=3000)
env = BatchedG1StandEnv(512, MJCF, reward_cfg=reward_cfg, dr_cfg={}, hold_arms=True)
obs = env.reset()
ac = AC().to(env.tdev)
ac.load_state_dict(torch.load(POLICY, map_location=env.tdev))
ac.eval()

abs_err = torch.zeros(env.n, NJ, device=env.tdev)
alive_steps = torch.zeros(env.n, device=env.tdev)
first_fall = torch.zeros(env.n, dtype=torch.int32, device=env.tdev)
for k in range(1500):
    with torch.no_grad():
        mu = ac.pi(obs)
    obs, r, done, info = env.step(mu)
    alive = (first_fall == 0).float()
    q_act = env.qpos_t.index_select(1, env.qpos_idx_t)
    err = (q_act - env._model_legs).abs()
    abs_err += err * alive.unsqueeze(1)
    alive_steps += alive
    newly = done & (first_fall == 0)
    first_fall = torch.where(newly, torch.full_like(first_fall, k + 1), first_fall)

a = torch.clamp(alive_steps, min=1.0).unsqueeze(1)
mean_err = (abs_err / a).mean(dim=0).cpu().numpy()    # per joint, rad
ff = first_fall.cpu().numpy().astype(float)
ff[ff == 0] = 1500
names = ["L_hip", "L_hipR", "L_hipY", "L_knee", "L_ankP", "L_ankR",
         "R_hip", "R_hipR", "R_hipY", "R_knee", "R_ankP", "R_ankR", "waist"]
print(f"policy={POLICY.split(chr(92))[-2]}")
print(f"  mean survival: {ff.mean()*DT:.1f}s")
print(f"  hip+knee tracking error (deg): "
      f"{np.degrees(mean_err[HIP_KN]).mean():.1f}  "
      f"(L_hip {np.degrees(mean_err[0]):.1f}, L_knee {np.degrees(mean_err[3]):.1f}, "
      f"R_hip {np.degrees(mean_err[6]):.1f}, R_knee {np.degrees(mean_err[9]):.1f})")
print(f"  all-leg mean tracking error (deg): {np.degrees(mean_err[:12]).mean():.1f}")
