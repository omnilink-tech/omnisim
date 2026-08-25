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

"""SINGLE SOURCE for the H1 walk OBSERVATION + INITIAL-CONDITION layer.

The H1 counterpart of ``g1_env_core`` (see that file + ``train-deploy-unification.md``
for the full rationale). It owns the *loop around the model* -- the obs vector
and the launch state -- which the H1 MjModel field-diff proved was the remaining
trainer<->deploy divergence (the model itself is byte-faithful: nq/nv/nu, joints,
kp800/kd70 actuators, friction/solref/contacts all match the dumped deploy MJCF).

The two genuine obs/IC gaps this closes for H1 (same as G1):
  * **joint velocity qd** -- the deploy (``humanoid_walk_deploy.py``) has only
    position sensors so it FINITE-DIFFERENCES qd = (q - q_prev)/dt at the 16 ms
    control tick; the trainer reads EXACT mjw ``qvel``. The policy is deployed
    against finite-diff qd, so the trainer must train on it. ``JointVelEstimator``
    is the shared definition (dt = 0.016 s, matching the deploy basicTimeStep).
  * **initial condition** -- the deploy settles ~0.3 s under gravity before the
    first policy tick (arriving leaned + moving); the trainer teleport-resets to
    pitch 0 / qvel 0. ``settle_steps`` codifies the settle length.

Backend-agnostic: every function takes/returns numpy (deploy, [D]/[1,D]) or torch
(trainer, [N,D]) and dispatches on input type -- identical math both sides.

H1 core obs layout (matches the trainer ``_build_obs_t`` + the deploy
``np.concatenate``)::

    obs = [ lin_vel(3, world) | ang_vel(3, body) | proj_gravity(3) |
            q_leg - nominal(11) | qd_leg(11) | last_action(11) |
            gait sin/cos(2) ]                                    == 44

H1 has 5-DOF legs (NO ankle-roll) -> N_LEG = 11 (vs G1's 13).
"""
from __future__ import annotations

import math
from typing import Sequence

N_LEG = 11                       # H1 LEGS_JOINTS (5-DOF legs x2 + waist)
CORE_OBS_DIM = 3 + 3 + 3 + N_LEG + N_LEG + N_LEG + 2   # == 44
DT = 0.016                       # H1 control tick = deploy basicTimeStep 16 ms
SETTLE_SECONDS = 0.3             # deploy settle before tick 0


# --- backend dispatch (numpy or torch, one implementation) -----------------
def _is_torch(a) -> bool:
    return type(a).__module__.split(".", 1)[0] == "torch"


def _stack(arrs, dim: int):
    if _is_torch(arrs[0]):
        import torch
        return torch.stack(arrs, dim=dim)
    import numpy as np
    return np.stack(arrs, axis=dim)


# --- projected gravity (body-frame gravity unit vector) --------------------
def proj_gravity_from_quat(quat_wxyz):
    """Body-frame projected gravity from a body->world quat [...,4] (w,x,y,z).
    Bit-for-bit the closed form the trainer obs uses; == R^T (0,0,-1)."""
    q = quat_wxyz
    w = q[..., 0]; x = q[..., 1]; y = q[..., 2]; z = q[..., 3]
    gx = -2.0 * (x * z - w * y)
    gy = -2.0 * (y * z + w * x)
    gz = -(1.0 - 2.0 * (x * x + y * y))
    return _stack([gx, gy, gz], dim=-1)


def proj_gravity_from_matrix(R_rowmajor: Sequence[float]):
    """Deploy path: R^T (0,0,-1) = (-R[6],-R[7],-R[8]) from a row-major 3x3."""
    o = R_rowmajor
    return (-float(o[6]), -float(o[7]), -float(o[8]))


def world_ang_to_body_matrix(R_rowmajor: Sequence[float], omega_world: Sequence[float]):
    """omega_body = R^T omega_world (the deploy R^T fix; matches the trainer's
    already-body-frame qvel[3:6]). Pure-python; returns a 3-tuple."""
    o = R_rowmajor
    wx, wy, wz = float(omega_world[0]), float(omega_world[1]), float(omega_world[2])
    return (o[0] * wx + o[3] * wy + o[6] * wz,
            o[1] * wx + o[4] * wy + o[7] * wz,
            o[2] * wx + o[5] * wy + o[8] * wz)


# --- joint velocity: the deploy finite-diff(+low-pass), the canonical qd ----
def qd_alpha(dt: float, tau: float) -> float:
    if tau <= 0.0:
        return 1.0
    return max(0.05, min(1.0, dt / (dt + tau)))


class JointVelEstimator:
    """Finite-difference joint velocity with optional low-pass -- the SHARED
    definition for both the deploy controller and the trainer.

    qd_raw = (q - prev_q)/dt ; qd = alpha*qd_raw + (1-alpha)*qd. Backend-agnostic;
    holds prev_q/smoothed as the same type as the first reset/update input. The
    deploy finite-differences on the 11 LEGS_JOINTS at dt=DT, tau=0 (raw)."""

    def __init__(self, dt: float = DT, tau: float = 0.0):
        self.dt = float(dt)
        self.alpha = qd_alpha(self.dt, float(tau))
        self.prev_q = None
        self.smoothed = None

    def reset(self, q0):
        self.prev_q = q0 * 1            # copy (np + torch)
        self.smoothed = q0 * 0.0
        return self.smoothed

    def reset_rows(self, q_rows, mask):
        """Per-env reset (trainer): for envs in ``mask`` set prev_q to the current
        pose + zero smoothed, so the first post-reset qd is 0 (no teleport vel)."""
        if self.prev_q is None:
            return self.reset(q_rows * 1)
        self.prev_q[mask] = q_rows[mask]
        self.smoothed[mask] = q_rows[mask] * 0.0
        return self.smoothed

    def update(self, q):
        if self.prev_q is None:
            self.reset(q * 1)
            return self.smoothed
        qd_raw = (q - self.prev_q) / max(self.dt, 1e-6)
        self.smoothed = self.alpha * qd_raw + (1.0 - self.alpha) * self.smoothed
        self.prev_q = q * 1
        return self.smoothed


def gait_sincos(phase):
    if _is_torch(phase):
        import torch
        return torch.stack([torch.sin(phase), torch.cos(phase)], dim=-1)
    import numpy as np
    return np.stack([np.sin(phase), np.cos(phase)], axis=-1)


# --- initial condition: the deploy ~0.3 s gravity settle, codified ----------
def settle_steps(dt: float = DT, settle_s: float = SETTLE_SECONDS) -> int:
    """env-steps the deploy settles under gravity before tick 0 = ceil(settle/dt)."""
    return math.ceil(settle_s / float(dt))


__all__ = [
    "N_LEG", "CORE_OBS_DIM", "DT", "SETTLE_SECONDS",
    "proj_gravity_from_quat", "proj_gravity_from_matrix",
    "world_ang_to_body_matrix", "qd_alpha", "JointVelEstimator",
    "gait_sincos", "settle_steps",
]
