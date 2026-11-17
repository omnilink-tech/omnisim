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

"""Vectorized trot foot-trajectory generator — N envs at once.

Same math as `omniquad_gait.foot_targets` (stance: linear slide -step/2
to +step/2 over duty fraction; swing: parabolic z arc, x slides back
to +step/2) but every operation is broadcast over a leading envs
axis so the GPU mjwarp trainer can compute all 4 foot targets for
1024 envs without a Python loop.

Inputs:
    t:        (N,)        — sim time per env (s)
    vx:       (N,)        — commanded forward velocity per env (m/s)
    vy:       (N,)        — lateral
    wz:       (N,)        — yaw rate
    params:   GaitParams  — single set of constants shared by all envs

Output:
    foot_targets: (N, 4, 3) — body-frame xyz per leg per env
                              legs in canonical order FL, FR, RL, RR
"""
from __future__ import annotations
import numpy as np

from .omniquad_gait import GaitParams, TROT_PHASE_OFFSET

# Phase offsets per leg in canonical order FL, FR, RL, RR.
_LEG_ORDER = ("FL", "FR", "RL", "RR")
PHASE_OFFSETS = np.array(
    [TROT_PHASE_OFFSET[leg] for leg in _LEG_ORDER], dtype=np.float64
)


def neutral_foot_positions_vec(p: GaitParams) -> np.ndarray:
    """Return shape (4, 3) array of neutral foot positions per leg."""
    nx_f, nx_r = p.neutral_front_x, p.neutral_rear_x
    ny = p.neutral_lateral_y
    z = p.ground_z
    return np.array([
        [nx_f, +ny, z],    # FL
        [nx_f, -ny, z],    # FR
        [nx_r, +ny, z],    # RL
        [nx_r, -ny, z],    # RR
    ], dtype=np.float64)


def foot_targets_batched(t: np.ndarray, vx: np.ndarray, vy: np.ndarray,
                         wz: np.ndarray, p: GaitParams) -> np.ndarray:
    """Vectorized foot-target generator.

    All time/velocity inputs are shape (N,). Returns (N, 4, 3).
    """
    t = np.asarray(t, dtype=np.float64)
    vx = np.asarray(vx, dtype=np.float64)
    vy = np.asarray(vy, dtype=np.float64)
    wz = np.asarray(wz, dtype=np.float64)
    N = t.shape[0]

    duty = p.duty
    period = p.period_s
    step_height = p.step_height
    stance_duration = p.stance_duration

    step_x = vx * stance_duration            # (N,)
    step_y = vy * stance_duration            # (N,)

    # Neutral foot positions broadcast to (N, 4, 3).
    neutrals = neutral_foot_positions_vec(p)[None, :, :]   # (1, 4, 3)
    neutrals = np.broadcast_to(neutrals, (N, 4, 3)).copy()

    # Yaw-rate steering: rotate each leg's neutral about body z by
    # half-cycle yaw change. Per env per leg.
    if np.any(wz != 0.0):
        half_yaw = (wz * stance_duration * 0.5)[:, None]  # (N, 1)
        c = np.cos(half_yaw); s = np.sin(half_yaw)
        nx0 = neutrals[..., 0]
        ny0 = neutrals[..., 1]
        neutrals[..., 0] = nx0 * c - ny0 * s
        neutrals[..., 1] = nx0 * s + ny0 * c

    # Phase per leg per env: (t/period + phase_off[leg]) mod 1.
    phase = (t[:, None] / period + PHASE_OFFSETS[None, :]) % 1.0   # (N, 4)

    # In-stance mask + per-phase normalized progress s.
    in_stance = phase < duty                                       # (N, 4)
    s_stance = np.where(in_stance, phase / duty, 0.0)
    s_swing = np.where(in_stance, 0.0, (phase - duty) / (1.0 - duty))

    # Step-x and step-y per env, broadcast across legs as (N, 1).
    sx = step_x[:, None]
    sy = step_y[:, None]

    nx = neutrals[..., 0]                                         # (N, 4)
    ny = neutrals[..., 1]
    nz = neutrals[..., 2]

    # Stance: foot slides backward through body frame (foot planted,
    # body moves forward over it). Swing: foot lifts and returns
    # forward. Matches scalar `omniquad_gait.foot_targets`.
    fx_st = nx + sx * 0.5 - sx * s_stance
    fy_st = ny + sy * 0.5 - sy * s_stance
    fz_st = nz

    fx_sw = nx - sx * 0.5 + sx * s_swing
    fy_sw = ny - sy * 0.5 + sy * s_swing
    fz_sw = nz + step_height * 4.0 * s_swing * (1.0 - s_swing)

    fx = np.where(in_stance, fx_st, fx_sw)
    fy = np.where(in_stance, fy_st, fy_sw)
    fz = np.where(in_stance, fz_st, fz_sw)

    out = np.stack([fx, fy, fz], axis=-1).astype(np.float32)   # (N, 4, 3)
    return out


def _selftest() -> None:
    from .omniquad_gait import foot_targets as scalar_foot_targets

    p = GaitParams()
    N = 16
    t = np.linspace(0.0, p.period_s, N, dtype=np.float64)
    vx = np.full(N, 0.5, dtype=np.float64)
    vy = np.zeros(N, dtype=np.float64)
    wz = np.zeros(N, dtype=np.float64)

    batched = foot_targets_batched(t, vx, vy, wz, p)   # (N, 4, 3)

    # Compare per env per leg against the scalar reference.
    max_err = 0.0
    for n in range(N):
        scalar = scalar_foot_targets(
            float(t[n]), vx=0.5, vy=0.0, wz=0.0, p=p)
        for li, leg in enumerate(_LEG_ORDER):
            ref = np.array(scalar[leg])
            err = float(np.max(np.abs(batched[n, li] - ref)))
            max_err = max(max_err, err)
    print(f"batched-vs-scalar gait round-trip on {N*4} samples:")
    print(f"  max abs error: {max_err:.2e}")
    assert max_err < 1e-6, "vectorized gait disagrees with scalar"
    print("  OK")


if __name__ == "__main__":
    _selftest()
