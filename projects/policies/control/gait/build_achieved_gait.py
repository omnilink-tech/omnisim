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

"""Build the (B) DYNAMICALLY-CONSISTENT shadow: distill a reference from a walk
the robot actually produced. Roll out the deployed champion
(gpu_g1_walk26_shape_c8), bin its ACHIEVED leg+waist joint angles by gait phase,
L/R-symmetrise (the half-cycle mirror) and circularly smooth, then save
(_W_N, 13) + a symmetric stand anchor to datasets/g1_achieved_gait.npz. Feasible
by construction -> tracking error against THIS reference has a real floor near
zero (it inherits the recorded splay, by design -- see docs/developer/
g1-improved-shadow.md). Re-run to rebuild after retraining the champion:

    python projects/policies/control/gait/build_achieved_gait.py [path/to/policy.pt]
"""
import os
import sys

import numpy as np
import torch
import torch.nn as nn

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _REPO)
from projects.policies.research.training.gpu_mjwarp_g1_walk_trainer import BatchedG1StandEnv, NJ
import projects.policies.control.gait.g1_human_gait as ghg

MJCF = os.path.join(_REPO, "projects", "robots", "unitree", "g1", "urdf",
                    "g1_full_kp100.mjcf.xml")
POLICY = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    _REPO, "projects", "rl", "training", "runs", "gpu_g1_walk26_shape_c8", "policy.pt")
OUT = os.path.join(os.path.dirname(ghg.__file__), "datasets", "g1_achieved_gait.npz")
W_N = ghg._W_N
MIRROR_SIGN = np.array([1, -1, -1, 1, 1, -1], dtype=np.float64)   # HP,HR,HY,KN,AP,AR

reward_cfg = dict(
    gait_model="human",
    gait_params=dict(vx=0.4, freq=1.3, sway=0.05, arm_swing=0.25,
                     style="winter", ramp_s=2.0, winter_hip_scale=0.9),
    seed_gait=True, max_ep=3000, obs_stack=4, obs_lookahead=[0.1, 0.4],
)
env = BatchedG1StandEnv(512, MJCF, reward_cfg=reward_cfg, dr_cfg={}, hold_arms=True)
obs = env.reset()


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
    return net


pi = build_pi(torch.load(POLICY, map_location=env.tdev)).to(env.tdev).eval()

qsum = torch.zeros(W_N, NJ, device=env.tdev)
qcnt = torch.zeros(W_N, device=env.tdev)
first_fall = torch.zeros(env.n, dtype=torch.int32, device=env.tdev)
for k in range(2000):
    with torch.no_grad():
        mu = pi(obs)
    obs, r, done, info = env.step(mu)
    alive = (first_fall == 0)
    if k > 200:                       # skip the launch ramp -> steady gait only
        q = env.qpos_t.index_select(1, env.qpos_idx_t)
        phi = torch.remainder(env.phase_t / (2.0 * np.pi), 1.0)
        b = torch.clamp((phi * W_N).long(), 0, W_N - 1)
        qsum.index_add_(0, b[alive], q[alive])
        qcnt.index_add_(0, b[alive], torch.ones(int(alive.sum()), device=env.tdev))
    newly = done & (first_fall == 0)
    first_fall = torch.where(newly, torch.full_like(first_fall, k + 1), first_fall)

qmean = (qsum / torch.clamp(qcnt, min=1.0).unsqueeze(1)).cpu().numpy()
assert (qcnt.cpu().numpy() > 0).all(), "some phase bins never visited"


def csmooth(tab, win=13):
    w = np.hanning(win); w /= w.sum()
    ext = np.concatenate([tab[-(win // 2):], tab, tab[:win // 2]], axis=0)
    return np.stack([np.convolve(ext[:, j], w, mode="same")[win // 2:-(win // 2)]
                     for j in range(tab.shape[1])], axis=1)


qs = csmooth(qmean)
half = W_N // 2
left, right = qs[:, 0:6].copy(), qs[:, 6:12].copy()
left_sym = 0.5 * (left + np.roll(right, -half, axis=0) * MIRROR_SIGN)
sym = np.zeros((W_N, NJ))
sym[:, 0:6] = left_sym
sym[:, 6:12] = np.roll(left_sym, half, axis=0) * MIRROR_SIGN
sym[:, 12] = 0.0
# symmetric double-support stand anchor (left leg at DS_PHASE, mirrored).
ds_row = sym[int(round((ghg.DS_PHASE / (2.0 * np.pi)) % 1.0 * W_N)) % W_N]
stand = np.zeros(NJ)
stand[0:6] = ds_row[0:6]
stand[6:12] = ds_row[0:6] * MIRROR_SIGN

os.makedirs(os.path.dirname(OUT), exist_ok=True)
np.savez(OUT, q=sym.astype(np.float32), stand=stand.astype(np.float32))
surv = first_fall.cpu().numpy().astype(float); surv[surv == 0] = 2000
amp = np.degrees(np.abs(sym[:, 0:6] - sym[:, 0:6].mean(0)).max(0))
print(f"saved {OUT}  (survival {surv.mean()*0.016:.1f}s)")
print(f"achieved hip-roll splay amp L {amp[1]:.1f} deg, hip-pitch amp {amp[0]:.1f} deg")
