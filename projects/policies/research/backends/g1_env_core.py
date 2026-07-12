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

"""SINGLE SOURCE for the G1 walk OBSERVATION + INITIAL-CONDITION layer.

Companion to ``g1_physics_spec`` (the single source for the *physics model*).
This module is the single source for the *loop around the model* -- the obs
vector and the launch state -- the layer the structural/golden parity work
proved was the remaining trainer<->deploy divergence (see
``g1_step_obs_parity.py`` and ``docs/developer/g1-single-source-of-truth.md``
section "the H1 corollary").

Why this file exists
--------------------
The trainer (``gpu_mjwarp_g1_walk_trainer._build_obs_t``) and the deploy
controller (``g1_walk_deploy.py``) each assemble the same 50-d observation, but
they were assembled by two independent code paths and silently diverged on:

  * **base angular velocity frame** -- the trainer reads MuJoCo free-joint
    ``qvel[3:6]`` which is ALREADY body-frame; the deploy reads ``getVelocity()``
    which is WORLD-frame and must rotate it ``omega_body = R^T omega_world``
    (the 2026-06-09 "OBS FIX" in g1_walk_deploy). This file is the ONE place
    that rotation is defined, so it can never drift again.
  * **joint velocity qd** -- the deploy has only position sensors so it
    FINITE-DIFFERENCES qd (+ optional low-pass); the trainer reads EXACT
    ``qvel``. The policy is deployed against finite-diff qd, so the trainer must
    train on the same. ``JointVelEstimator`` is the shared definition.
  * **initial condition** -- the deploy settles under gravity ~0.3 s before the
    first policy tick (arriving with a forward lean + residual velocity); the
    trainer teleport-resets to pitch 0 / qvel 0. ``settle_steps`` /
    ``apply_settle_hold`` codify the deploy settle so the trainer reset can
    reproduce it.

Backend-agnostic
----------------
Every function accepts and returns either numpy arrays (deploy, single env,
shape ``[D]`` or ``[1, D]``) or torch tensors (trainer, batched, shape
``[N, D]``) and dispatches on the input type. The math is identical, so a
conformance test can assert numpy==torch bit-for-(float)-bit.

Layout (matches ``_build_obs_t`` / the deploy ``np.concatenate``)::

    obs = [ lin_vel(3, world) | ang_vel(3, body) | proj_gravity(3) |
            q_leg - nominal(13) | qd_leg(13) | last_action(13) |
            gait sin/cos(2) ]                                    == 50

The frame-stack / lookahead / vx-cmd tail that ``_obs_full`` appends is
trainer-recipe-specific and stays in the trainer; this module owns the 50-d
*core* both sides share.
"""
from __future__ import annotations

import math
from typing import Sequence

from projects.policies.research.backends import g1_physics_spec as SPEC

# Core obs block widths (the 50-d shared vector).
N_LEG = 13                       # LEGS_JOINTS (legs + waist)
CORE_OBS_DIM = 3 + 3 + 3 + N_LEG + N_LEG + N_LEG + 2   # == 50


# ---------------------------------------------------------------------------
# Backend dispatch -- numpy (deploy) or torch (trainer), one implementation.
# ---------------------------------------------------------------------------
def _is_torch(a) -> bool:
    return type(a).__module__.split(".", 1)[0] == "torch"


def _cat(arrs, dim: int):
    if _is_torch(arrs[0]):
        import torch
        return torch.cat(arrs, dim=dim)
    import numpy as np
    return np.concatenate(arrs, axis=dim)


def _stack(arrs, dim: int):
    if _is_torch(arrs[0]):
        import torch
        return torch.stack(arrs, dim=dim)
    import numpy as np
    return np.stack(arrs, axis=dim)


def _clamp(a, lo: float, hi: float):
    if _is_torch(a):
        import torch
        return torch.clamp(a, lo, hi)
    import numpy as np
    return np.clip(a, lo, hi)


def _nan_to_num(a, posinf: float, neginf: float):
    if _is_torch(a):
        import torch
        return torch.nan_to_num(a, nan=0.0, posinf=posinf, neginf=neginf)
    import numpy as np
    return np.nan_to_num(a, nan=0.0, posinf=posinf, neginf=neginf)


# ---------------------------------------------------------------------------
# Projected gravity (body-frame gravity unit vector).
# ---------------------------------------------------------------------------
def proj_gravity_from_quat(quat_wxyz):
    """Body-frame projected gravity from a body->world quat ``[..., 4]`` (w,x,y,z).

    Bit-for-bit the closed form ``_build_obs_t`` uses (gx,gy,gz). Equivalent to
    ``R^T (0,0,-1)`` where R is body->world.
    """
    q = quat_wxyz
    w = q[..., 0]; x = q[..., 1]; y = q[..., 2]; z = q[..., 3]
    gx = -2.0 * (x * z - w * y)
    gy = -2.0 * (y * z + w * x)
    gz = -(1.0 - 2.0 * (x * x + y * y))
    return _stack([gx, gy, gz], dim=-1)


def proj_gravity_from_matrix(R_rowmajor: Sequence[float]):
    """Body-frame projected gravity from a row-major body->world 3x3 (9 floats).

    The deploy's ``_proj_gravity_from_ori``: ``R^T (0,0,-1) = (-R[6],-R[7],-R[8])``.
    Pure-python (deploy reads getOrientation() as a 9-list). Returns a 3-tuple.
    """
    o = R_rowmajor
    return (-float(o[6]), -float(o[7]), -float(o[8]))


# ---------------------------------------------------------------------------
# Angular-velocity frame: the 2026-06-09 deploy OBS FIX, codified once.
# ---------------------------------------------------------------------------
def world_ang_to_body_matrix(R_rowmajor: Sequence[float], omega_world: Sequence[float]):
    """omega_body = R^T omega_world, R row-major body->world (deploy path).

    Mirrors g1_walk_deploy:769-773 exactly (so the trainer's already-body-frame
    qvel[3:6] and the deploy's rotated getVelocity() are the SAME quantity).
    Pure-python; returns a 3-tuple.
    """
    o = R_rowmajor
    wx, wy, wz = float(omega_world[0]), float(omega_world[1]), float(omega_world[2])
    return (
        o[0] * wx + o[3] * wy + o[6] * wz,
        o[1] * wx + o[4] * wy + o[7] * wz,
        o[2] * wx + o[5] * wy + o[8] * wz,
    )


def quat_wxyz_to_R_rowmajor(quat_wxyz: Sequence[float]):
    """body->world rotation as a row-major 9-tuple, from a (w,x,y,z) quat.

    Lets a quat-only consumer (the trainer / a test) reuse the matrix-based
    deploy helpers above.
    """
    w, x, y, z = (float(quat_wxyz[0]), float(quat_wxyz[1]),
                  float(quat_wxyz[2]), float(quat_wxyz[3]))
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return (
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
        2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y),
    )


# ---------------------------------------------------------------------------
# Joint velocity: the deploy finite-diff(+low-pass), the canonical qd.
# ---------------------------------------------------------------------------
def qd_alpha(dt: float, tau: float) -> float:
    """Low-pass coefficient for finite-diff qd. tau<=0 -> 1.0 (raw finite-diff,
    the default; matches g1_walk_deploy:508 and g1_stand_deploy)."""
    if tau <= 0.0:
        return 1.0
    return max(0.05, min(1.0, dt / (dt + tau)))


class JointVelEstimator:
    """Finite-difference joint velocity with optional low-pass, the SHARED
    definition for both the deploy controller and the trainer.

    qd_raw = (q - prev_q) / dt ; qd = alpha*qd_raw + (1-alpha)*qd  (per-call).
    Backend-agnostic: ``update`` returns the same array type passed in. Holds its
    state as the same type as the first ``reset``/``update`` input.

    Deploy uses this on the 13 LEGS_JOINTS at dt=SPEC.DT, tau=G1_QD_TAU (default
    0 -> raw finite-diff). The trainer, to match, must NOT feed exact qvel -- it
    feeds the achieved joint positions through this estimator.
    """

    def __init__(self, dt: float, tau: float = 0.0):
        self.dt = float(dt)
        self.alpha = qd_alpha(self.dt, float(tau))
        self.prev_q = None
        self.smoothed = None

    def reset(self, q0):
        """Seed prev_q (and zero the smoothed state) from the reset pose. For
        the batched trainer, q0 is ``[N, NJ]``; a per-env reset mask can pass a
        masked assign by calling reset on the whole tensor each full reset, or
        use ``reset_rows``."""
        self.prev_q = q0 * 1  # copy (works for np and torch)
        self.smoothed = q0 * 0.0
        return self.smoothed

    def reset_rows(self, q_rows, mask):
        """Per-env reset (trainer): for envs in ``mask`` (bool index), set
        prev_q to the current pose and zero the smoothed velocity, so the first
        post-reset qd is 0 (no fake teleport velocity)."""
        if self.prev_q is None:
            return self.reset(q_rows * 1)
        self.prev_q[mask] = q_rows[mask]
        self.smoothed[mask] = q_rows[mask] * 0.0
        return self.smoothed

    def update(self, q):
        """Advance one tick with the achieved joint positions ``q``; return qd."""
        if self.prev_q is None:
            self.reset(q * 1)
            return self.smoothed
        qd_raw = (q - self.prev_q) / max(self.dt, 1e-6)
        self.smoothed = self.alpha * qd_raw + (1.0 - self.alpha) * self.smoothed
        self.prev_q = q * 1
        return self.smoothed


# ---------------------------------------------------------------------------
# Assemble the 50-d core observation (the one block both sides share).
# ---------------------------------------------------------------------------
def assemble_walk_obs(lin_vel_world, ang_vel_body, proj_g,
                      q_leg_minus_nominal, qd_leg, last_action, gait_sincos,
                      *, sanitize: bool = True):
    """Concatenate the 50-d core obs in the canonical order. All inputs are
    ``[N, w]`` (N=1 allowed) of the SAME backend; returns ``[N, 50]``.

      lin_vel_world          [N,3]  world-frame base linear velocity
      ang_vel_body           [N,3]  body-frame base angular velocity (R^T fix)
      proj_g                 [N,3]  proj_gravity_from_quat / _from_matrix
      q_leg_minus_nominal    [N,13] leg joint pos - NOMINAL_LEGS
      qd_leg                 [N,13] JointVelEstimator output (finite-diff)
      last_action            [N,13] previous policy action
      gait_sincos            [N,2]  (sin phase, cos phase)

    sanitize mirrors _build_obs_t's tail: nan_to_num then clamp to [-10,10].
    """
    obs = _cat([lin_vel_world, ang_vel_body, proj_g,
                q_leg_minus_nominal, qd_leg, last_action, gait_sincos], dim=-1)
    if sanitize:
        obs = _nan_to_num(obs, posinf=10.0, neginf=-10.0)
        obs = _clamp(obs, -10.0, 10.0)
    return obs


def gait_sincos(phase):
    """(sin phase, cos phase) as ``[..., 2]`` -- same order both sides."""
    if _is_torch(phase):
        import torch
        return torch.stack([torch.sin(phase), torch.cos(phase)], dim=-1)
    import numpy as np
    return np.stack([np.sin(phase), np.cos(phase)], axis=-1)


# ---------------------------------------------------------------------------
# Initial condition: the deploy's ~0.3 s gravity settle, codified.
# ---------------------------------------------------------------------------
SETTLE_SECONDS = 0.3


def settle_steps(dt: float | None = None, settle_s: float = SETTLE_SECONDS) -> int:
    """Number of env-steps the deploy settles under gravity before tick 0.

    ceil(settle_s / dt). The trainer reset should advance the physics this many
    steps holding the nominal pose so it launches from the deploy's lean +
    residual velocity instead of pitch=0 / qvel=0.
    """
    dt = SPEC.DT if dt is None else float(dt)
    return math.ceil(settle_s / dt)


def nominal_full():
    """The full nominal joint vector (legs+waist then arms) in SPEC order, as a
    plain list -- the pose held during the settle and the policy nominal."""
    return list(SPEC.NOMINAL_LEGS) + list(SPEC.ARM_NOMINAL)


__all__ = [
    "N_LEG", "CORE_OBS_DIM", "SETTLE_SECONDS",
    "proj_gravity_from_quat", "proj_gravity_from_matrix",
    "world_ang_to_body_matrix", "quat_wxyz_to_R_rowmajor",
    "qd_alpha", "JointVelEstimator",
    "assemble_walk_obs", "gait_sincos",
    "settle_steps", "nominal_full",
]
