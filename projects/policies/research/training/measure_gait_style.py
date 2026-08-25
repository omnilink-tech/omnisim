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

"""GAIT-STYLE metric: does the robot walk LIKE the model (same waveform /
posture over the stride), as opposed to matching it tick-for-tick?

Bins actual joint angles by gait phase into a cycle waveform, circularly
aligns it to the model waveform (best phase shift), then reports per joint:
  - phase lag (how late the robot runs vs the ghost -- invisible to the eye)
  - amplitude ratio (1.0 = same motion size; <1 = muted)
  - shape correlation (1.0 = same waveform shape)
  - residual RMSE after alignment (the TRUE style gap)
Also checks the knee double-bend in the ACTUAL waveform.

Usage: python measure_gait_style.py <policy.pt> [out.png]
"""
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
OUT_PNG = sys.argv[2] if len(sys.argv) > 2 else None
NBINS = 36


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

sum_act = torch.zeros(NBINS, NJ, device=env.tdev)
sum_mod = torch.zeros(NBINS, NJ, device=env.tdev)
cnt = torch.zeros(NBINS, device=env.tdev)
fallen = torch.zeros(env.n, dtype=torch.bool, device=env.tdev)

WARM = 300              # skip launch ramp: style is the steady gait
for k in range(1800):
    ph = env.phase_t.clone()                      # phase generating this step
    with torch.no_grad():
        mu = ac.pi(obs)
    obs, r, done, info = env.step(mu)
    if k < WARM:
        fallen |= done
        continue
    alive = ~fallen
    q_act = env.qpos_t.index_select(1, env.qpos_idx_t)
    q_mod = env._model_legs
    b = torch.clamp((ph / (2 * np.pi) * NBINS).long(), 0, NBINS - 1)
    for arr, src in ((sum_act, q_act), (sum_mod, q_mod)):
        arr.index_add_(0, b[alive], src[alive])
    cnt.index_add_(0, b[alive], torch.ones(alive.sum(), device=env.tdev))
    fallen |= done

wa = (sum_act / torch.clamp(cnt, min=1).unsqueeze(1)).cpu().numpy()   # actual
wm = (sum_mod / torch.clamp(cnt, min=1).unsqueeze(1)).cpu().numpy()   # model

names = ["L_hip", "L_hipR", "L_hipY", "L_knee", "L_ankP", "L_ankR",
         "R_hip", "R_hipR", "R_hipY", "R_knee", "R_ankP", "R_ankR", "waist"]
KEY = [0, 3, 4]          # L hip pitch, knee, ankle pitch (style signature)

print(f"policy={POLICY.split(chr(92))[-2]}  (waveforms over {NBINS} phase bins)")
report = {}
for j in KEY:
    a, m = wa[:, j], wm[:, j]
    am, mm = a - a.mean(), m - m.mean()
    # best circular alignment by cross-correlation
    xc = [np.dot(np.roll(am, s), mm) for s in range(NBINS)]
    s = int(np.argmax(xc))
    a_al = np.roll(am, s)
    lag_pct = (s if s <= NBINS // 2 else s - NBINS) / NBINS * 100
    amp = a.std() / max(m.std(), 1e-9)
    corr = float(np.dot(a_al, mm) / max(np.linalg.norm(a_al) * np.linalg.norm(mm), 1e-9))
    rmse = float(np.sqrt(((a_al - mm) ** 2).mean()))
    report[j] = (a, m, s)
    print(f"  {names[j]:6s}: phase lag {lag_pct:+5.1f}% of cycle | "
          f"amplitude {amp:4.2f}x | shape corr {corr:5.2f} | "
          f"aligned RMSE {np.degrees(rmse):4.1f} deg")

# knee double-bend in the ACTUAL waveform
kn = wa[:, 3]
kn_s = np.convolve(np.r_[kn, kn, kn], np.ones(3) / 3, "same")[NBINS:2 * NBINS]
peaks = [i for i in range(NBINS)
         if kn_s[i] > kn_s[i - 1] and kn_s[i] > kn_s[(i + 1) % NBINS]
         and kn_s[i] > kn_s.min() + 0.05]
print(f"  ACTUAL knee: {len(peaks)} peak(s) -> "
      f"{'DOUBLE-BEND PRESENT' if len(peaks) >= 2 else 'double-bend MISSING'} "
      f"(swing peak {np.degrees(kn_s.max()):.0f} deg vs model "
      f"{np.degrees(wm[:, 3].max()):.0f} deg)")

if OUT_PNG:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    pct = np.arange(NBINS) / NBINS * 100
    for ax, j in zip(axes, KEY):
        a, m, s = report[j]
        ax.plot(pct, np.degrees(m), "--", color="#999", lw=2.5, label="ghost (model)")
        ax.plot(pct, np.degrees(a), "-", color="#d62728", lw=2.5, label="robot actual")
        ax.plot(pct, np.degrees(np.roll(a, s) - a.mean() + m.mean()), ":",
                color="#1f77b4", lw=2, label="actual, phase-aligned")
        ax.set_title(names[j]); ax.set_xlabel("% gait cycle"); ax.grid(alpha=0.3)
    axes[0].set_ylabel("deg"); axes[0].legend(fontsize=9)
    fig.suptitle("Gait STYLE: robot waveform vs the ghost (same shape = same gait, "
                 "even if time-shifted)")
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(OUT_PNG, dpi=110)
    print(f"saved {OUT_PNG}")
