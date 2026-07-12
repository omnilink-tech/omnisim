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

"""g1_distill.py -- distill the MPC brain into a fast policy (shared definitions).

The MPC (g1_mpc.py) walks but is too slow for real-time (its planning rolls
physics forward ~220 sequential steps per tick). This module defines the
observation + the small policy used to DISTILL it: the MPC is the teacher, a tiny
MLP learns `state -> MPC residual`, and at deploy the MLP runs in microseconds
(pure-numpy forward pass) -> real-time.

ONE obs definition lives here so the data collector and the deploy brain agree.
The policy outputs the 8-dim balance residual on RES_J (same as the MPC's r0),
applied on top of the g1_human_gait reference.
"""
from __future__ import annotations
import math
import numpy as np

# 8 balance joints (indices into LEGS_JOINTS) -- must match g1_mpc.RES_J.
RES_J = np.array([0, 6, 1, 7, 4, 10, 5, 11])     # Lhp Rhp Lhr Rhr Lap Rap Lar Rar
ACT_DIM = len(RES_J)

# obs: yaw-frame CoM vel (3) + roll,pitch (2) + body rates (3) + bz (1)
#      + joint_q (13) + joint_qd (13) + sin/cos phase (2) = 37
OBS_DIM = 37
HIDDEN = (256, 256)


def make_obs(roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate,
             vx, vy, vz, bz, joint_q, joint_qd, phase) -> np.ndarray:
    """Build the policy observation. Velocities rotated into the yaw-aligned
    frame so the policy is heading-invariant (the MPC plans in that frame too)."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    com_vx = cy * vx + sy * vy
    com_vy = -sy * vx + cy * vy
    return np.concatenate([
        np.array([com_vx, com_vy, vz, roll, pitch,
                  roll_rate, pitch_rate, yaw_rate, bz], dtype=np.float32),
        np.asarray(joint_q, dtype=np.float32),
        np.asarray(joint_qd, dtype=np.float32),
        np.array([math.sin(phase), math.cos(phase)], dtype=np.float32),
    ]).astype(np.float32)


# ---- pure-numpy MLP forward (deploy: no torch dependency) ----
def numpy_policy(weights: dict, obs: np.ndarray) -> np.ndarray:
    """Forward pass through the distilled MLP from saved numpy weights.
    weights: W0,b0,W1,b1,...,Wn,bn (tanh between layers, linear output).
    Includes input standardisation (mean/std) baked in."""
    x = (obs - weights["obs_mean"]) / weights["obs_std"]
    n = weights["n_layers"]
    for i in range(n):
        x = x @ weights[f"W{i}"] + weights[f"b{i}"]
        if i < n - 1:
            x = np.tanh(x)
    return x.astype(np.float32)            # the 8-dim residual


# ---- torch MLP (training only; imported lazily) ----
def build_torch_mlp():
    import torch.nn as nn
    layers, d = [], OBS_DIM
    for h in HIDDEN:
        layers += [nn.Linear(d, h), nn.Tanh()]; d = h
    layers += [nn.Linear(d, ACT_DIM)]
    return nn.Sequential(*layers)


def export_numpy(model, obs_mean, obs_std) -> dict:
    """Pull a trained torch Sequential into the numpy-weight dict numpy_policy uses."""
    import torch.nn as nn
    w = {"obs_mean": np.asarray(obs_mean, np.float32),
         "obs_std": np.asarray(obs_std, np.float32)}
    i = 0
    for m in model:
        if isinstance(m, nn.Linear):
            w[f"W{i}"] = m.weight.detach().cpu().numpy().T.astype(np.float32)
            w[f"b{i}"] = m.bias.detach().cpu().numpy().astype(np.float32)
            i += 1
    w["n_layers"] = i
    return w
