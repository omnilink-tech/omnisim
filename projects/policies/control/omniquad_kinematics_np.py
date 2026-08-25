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

"""Vectorized numpy IK + FK for OmniQuad — N envs at once.

Same closed-form analytic solution as `omniquad_kinematics.py` but every
operation is broadcast over a leading "envs" axis so the GPU mjwarp
trainer can compute joint targets for 1024 envs without a Python loop.

Inputs / outputs are shaped (..., 4, 3):
  ... = arbitrary leading env axes (typically just N)
   4  = 4 legs in canonical order (FL, FR, RL, RR)
   3  = xyz in body frame, or (hip_x, hip_y, knee) joint angles

Leg geometry constants are imported from `omniquad_kinematics` so the
two implementations can't drift apart -- if anything changes (URDF
update), update the scalar module and the vectorized one inherits
the new dimensions automatically.
"""
from __future__ import annotations

import numpy as np

from .omniquad_kinematics import (
    L1, L2, KNEE_X, HIPY_OFFSET, HIP_X_POS,
)

# Leg order canonical to the project: FL, FR, RL, RR.
LEG_ORDER = ("FL", "FR", "RL", "RR")

# Hip-x mount positions broadcast as shape (4, 3): one row per leg.
HIP_X_POS_VEC = np.array([HIP_X_POS[leg] for leg in LEG_ORDER],
                         dtype=np.float64)

# Per-leg sign of the hip_y joint offset (+1 left, -1 right).
HIPY_SIGN_VEC = np.array([+1.0, -1.0, +1.0, -1.0], dtype=np.float64)


def inverse_kinematics_batched(target_body: np.ndarray) -> np.ndarray:
    """Vectorized IK.

    Args:
        target_body: shape (..., 4, 3). Foot xyz targets in body frame
            for each of the 4 legs.

    Returns:
        joint_angles: shape (..., 4, 3). (hip_x, hip_y, knee) per leg.
            NaN entries where the target is outside the reachable
            workspace -- the caller should clamp / handle these.
    """
    t = np.asarray(target_body, dtype=np.float64)
    if t.shape[-2:] != (4, 3):
        raise ValueError(f"target_body must end with (4, 3); got {t.shape}")

    # Shift target relative to each leg's hip_x joint.
    # HIP_X_POS_VEC shape (4, 3) broadcasts against the trailing (4, 3).
    T = t - HIP_X_POS_VEC                       # (..., 4, 3)
    Tx = T[..., 0]
    Ty = T[..., 1]
    Tz = T[..., 2]

    # H is per-leg hip_y offset: +0.111 for left legs, -0.111 for right.
    H = HIPY_SIGN_VEC * HIPY_OFFSET             # (4,)

    # ── Step 1: hip_x (alpha). Need Ty*cos(a) + Tz*sin(a) = H.
    # Solution: a = atan2(Tz, Ty) + acos(H / sqrt(Ty^2 + Tz^2)).
    r_yz = np.hypot(Ty, Tz)                     # (..., 4)
    # Clamp the acos argument so we get NaN-free output on edge cases.
    cos_arg_a = np.divide(H, r_yz, out=np.full_like(r_yz, np.nan),
                          where=r_yz > 1e-9)
    cos_arg_a = np.clip(cos_arg_a, -1.0, 1.0)
    alpha = np.arctan2(Tz, Ty) + np.arccos(cos_arg_a)
    # Wrap to [-pi, pi].
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

    # ── Step 2: rotate target into hip frame.
    sa = np.sin(alpha); ca = np.cos(alpha)
    # T_hip.y == H by construction; T_hip.z = -Ty*sa + Tz*ca.
    T_hip_z = -Ty * sa + Tz * ca
    Qx = Tx
    Qz = T_hip_z

    # ── Step 3: knee (gamma) via cosine law + knee-x-offset.
    Q2 = Qx * Qx + Qz * Qz
    R_link = np.hypot(L1, KNEE_X)
    phi_k = np.arctan2(KNEE_X, L1)
    C = (Q2 - KNEE_X * KNEE_X - L1 * L1 - L2 * L2) / (2.0 * L2)
    cos_arg_g = np.clip(C / R_link, -1.0, 1.0)
    gamma = -np.arccos(cos_arg_g) - phi_k

    # ── Step 4: hip_y (beta) — in-plane rotation.
    Sx = KNEE_X - L2 * np.sin(gamma)
    Sz = -L1 - L2 * np.cos(gamma)
    angle_S = np.arctan2(Sz, Sx)
    angle_Q = np.arctan2(Qz, Qx)
    beta = angle_S - angle_Q

    out = np.stack([alpha, beta, gamma], axis=-1)  # (..., 4, 3)

    # Mark unreachable (where the original problem had |H/r| > 1
    # before clipping, or |C/R| > 1 — clipping masked these).
    unreachable = (np.abs(H / np.where(r_yz > 1e-9, r_yz, 1.0)) > 1.0) | \
                  (np.abs(C / R_link) > 1.0)
    if np.any(unreachable):
        out = out.copy()
        out[unreachable] = np.nan
    return out.astype(np.float32)


def _selftest() -> None:
    """Round-trip against the scalar reference."""
    from .omniquad_kinematics import inverse_kinematics, forward_kinematics

    rng = np.random.default_rng(0)
    N = 64

    # Build N random feasible targets per leg using FK on random angles.
    nominal_q = np.array([
        [+0.30, +0.30, -0.60],   # FL
        [-0.30, +0.30, -0.60],   # FR
        [+0.30, +0.30, -0.60],   # RL
        [-0.30, +0.30, -0.60],   # RR
    ], dtype=np.float64)
    # Sample joint angles near nominal, get foot positions via scalar FK.
    targets = np.zeros((N, 4, 3), dtype=np.float64)
    for n in range(N):
        for li, leg in enumerate(LEG_ORDER):
            q = nominal_q[li] + rng.uniform(-0.05, 0.05, 3)
            from .omniquad_kinematics import JointAngles
            foot = forward_kinematics(leg, JointAngles(*q))
            targets[n, li] = foot

    # Run batched IK.
    q_batched = inverse_kinematics_batched(targets)

    # Compare against scalar IK leg by leg, env by env.
    max_err = 0.0
    fail = 0
    for n in range(N):
        for li, leg in enumerate(LEG_ORDER):
            q_scalar = inverse_kinematics(leg, tuple(targets[n, li]))
            if q_scalar is None:
                continue  # scalar said unreachable; skip
            scalar_q = np.array([q_scalar.hip_x, q_scalar.hip_y, q_scalar.knee])
            if np.any(np.isnan(q_batched[n, li])):
                fail += 1
                continue
            err = float(np.max(np.abs(q_batched[n, li] - scalar_q)))
            max_err = max(max_err, err)
    print(f"batched-vs-scalar IK round-trip on {N*4} targets:")
    print(f"  max abs error: {max_err:.2e}")
    print(f"  unreachable (NaN): {fail}")
    assert max_err < 1e-5, "batched IK disagrees with scalar"
    print("  OK")


if __name__ == "__main__":
    _selftest()
