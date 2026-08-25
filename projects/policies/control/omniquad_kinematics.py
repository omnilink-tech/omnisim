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

"""Analytic forward + inverse kinematics for OmniQuad legs.

Each leg is a 3-link chain: hip_x (X-axis), hip_y (Y-axis), knee (Y-axis).
Dimensions extracted from projects/robots/omnisim/omniquad/urdf/omniquad.urdf:

    body
      -> hip_x  joint at body-frame (HIP_X, HIP_Y, 0)     axis (1,0,0)
      -> hip    link
      -> hip_y  joint at (0, ±HIPY_OFFSET, 0)              axis (0,1,0)
      -> upper_leg link
      -> knee   joint at (KNEE_X, 0, -L1)                  axis (0,1,0)
      -> lower_leg link
      -> foot tip at (0, 0, -L2)

Frame convention: body frame is +X forward, +Y left, +Z up.
For the LEFT side, HIPY_OFFSET = +0.110945; for RIGHT, -0.110945.

The IK is closed-form. Given a target foot position in the body frame, it
returns the three joint angles or None if the target is unreachable. The
FK is provided for round-trip verification and for debugging the gait.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional, Tuple

# ── Geometry (meters) ────────────────────────────────────────────────────
L1 = 0.3205           # thigh: hip_y joint -> knee joint, z component magnitude
L2 = 0.32             # shank: knee joint -> foot tip
KNEE_X = 0.025        # small forward offset of the knee joint in upper-leg frame
HIPY_OFFSET = 0.110945  # lateral offset of hip_y joint from hip_x joint

# Hip_x joint positions in body frame, keyed by leg id.
# Leg ids match the agent's JOINT_ORDER convention:
#   FL = front_left, FR = front_right, RL = rear_left, RR = rear_right
HIP_X_POS = {
    "FL": (+0.29785, +0.05500, 0.0),
    "FR": (+0.29785, -0.05500, 0.0),
    "RL": (-0.29785, +0.05500, 0.0),
    "RR": (-0.29785, -0.05500, 0.0),
}

# Side sign of the hip_y offset for each leg (left = +, right = -).
HIPY_SIGN = {"FL": +1.0, "FR": -1.0, "RL": +1.0, "RR": -1.0}


@dataclass(frozen=True)
class JointAngles:
    """One leg's three joint angles, in URDF convention."""
    hip_x: float
    hip_y: float
    knee: float

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.hip_x, self.hip_y, self.knee)


# ── Forward kinematics ───────────────────────────────────────────────────

def forward_kinematics(leg: str, q: JointAngles) -> Tuple[float, float, float]:
    """Return the foot tip position in the body frame.

    Chain: foot = hip_x_pos + R_x(α) [ (0,±HIPY_OFFSET,0)
                                       + R_y(β) [ (KNEE_X,0,-L1)
                                                  + R_y(γ) (0,0,-L2) ] ]
    """
    α, β, γ = q.as_tuple()
    # 1. shank vector in lower-leg frame -> upper-leg frame (rotate by γ around Y)
    cg, sg = math.cos(γ), math.sin(γ)
    shank_x = -L2 * sg
    shank_z = -L2 * cg
    # 2. add knee offset -> point in upper-leg frame
    upper_x = KNEE_X + shank_x
    upper_z = -L1 + shank_z
    # 3. rotate by β around Y -> point in hip frame
    cb, sb = math.cos(β), math.sin(β)
    hip_x_pt = upper_x * cb + upper_z * sb
    hip_z_pt = -upper_x * sb + upper_z * cb
    # 4. add hip_y offset -> point in hip-x frame
    H = HIPY_SIGN[leg] * HIPY_OFFSET
    hipframe_y = H
    # 5. rotate by α around X -> point in body frame (relative to hip_x joint)
    ca, sa = math.cos(α), math.sin(α)
    body_x = hip_x_pt
    body_y = hipframe_y * ca - hip_z_pt * sa
    body_z = hipframe_y * sa + hip_z_pt * ca
    # 6. add hip_x joint position
    Hx, Hy, Hz = HIP_X_POS[leg]
    return (Hx + body_x, Hy + body_y, Hz + body_z)


# ── Inverse kinematics ───────────────────────────────────────────────────

def inverse_kinematics(
    leg: str, target_body: Tuple[float, float, float]
) -> Optional[JointAngles]:
    """Return the joint angles that put the foot tip at `target_body`, or
    None if the target is outside the leg's reachable workspace.

    Closed-form solution:
      1. α (hip_x) comes from constraining the foot's component along the
         hip-frame Y axis to equal the hip_y joint offset (the leg below
         hip_y has no Y component in the upper-leg frame).
      2. γ (knee) comes from the cosine law on the upper-leg / shank /
         hip-to-foot triangle (accounting for the small knee x-offset).
      3. β (hip_y) is the in-plane rotation that places the shank-endpoint
         vector S to the target relative position Q.
    """
    Hx, Hy, Hz = HIP_X_POS[leg]
    tx, ty, tz = target_body
    # Express target relative to the hip_x joint origin.
    Tx, Ty, Tz = tx - Hx, ty - Hy, tz - Hz
    H = HIPY_SIGN[leg] * HIPY_OFFSET

    # ── Step 1: hip_x ─────────────────────────────────────────────────
    # We need R_x(-α) · T to have y-component equal to H. Equivalently:
    #     Ty * cos(α) + Tz * sin(α) = H
    # which is r * cos(α - φ) = H with r = sqrt(Ty²+Tz²), φ = atan2(Tz, Ty).
    r_yz = math.hypot(Ty, Tz)
    if r_yz < 1e-9 or abs(H / r_yz) > 1.0:
        return None  # foot directly under hip_x joint or outside reach
    φ = math.atan2(Tz, Ty)
    # Two solutions ±acos; pick the one that keeps the leg below (foot
    # below hip), which for both sides is the one with α near zero rather
    # than ~±π. We add (not subtract) acos: the agent's joint limits
    # confirm this branch.
    α = φ + math.acos(H / r_yz)
    # Wrap to a sensible range.
    while α > math.pi: α -= 2 * math.pi
    while α < -math.pi: α += 2 * math.pi

    # ── Step 2: transform target into hip frame ───────────────────────
    sa, ca = math.sin(α), math.cos(α)
    T_hip_z = -Ty * sa + Tz * ca
    # T_hip_y == H by construction; subtract hip_y offset to get Q.
    Qx = Tx
    Qz = T_hip_z  # T_hip - h_off has y=0 by construction

    # ── Step 3: knee (γ) via cosine law accounting for KNEE_X ─────────
    # S = (KNEE_X - L2 sin γ, 0, -L1 - L2 cos γ)
    # |S|² = KNEE_X² + L1² + L2² + 2 L2 (L1 cos γ - KNEE_X sin γ)
    # Let R = sqrt(L1² + KNEE_X²), φ = atan2(KNEE_X, L1):
    #     L1 cos γ - KNEE_X sin γ = R cos(γ + φ)
    Q2 = Qx * Qx + Qz * Qz
    R_link = math.hypot(L1, KNEE_X)
    phi_k = math.atan2(KNEE_X, L1)
    C = (Q2 - KNEE_X * KNEE_X - L1 * L1 - L2 * L2) / (2.0 * L2)
    cos_arg = C / R_link
    if abs(cos_arg) > 1.0:
        return None  # foot too far or too close
    # OmniQuad's knee bends backward (γ < 0) so the foot moves forward and up
    # of the knee. Choose -acos to pick that branch.
    γ = -math.acos(cos_arg) - phi_k

    # ── Step 4: hip_y (β) ─────────────────────────────────────────────
    Sx = KNEE_X - L2 * math.sin(γ)
    Sz = -L1 - L2 * math.cos(γ)
    angle_S = math.atan2(Sz, Sx)
    angle_Q = math.atan2(Qz, Qx)
    # R_y(β) acts on (x,z) like a standard 2D rotation by -β. So:
    β = angle_S - angle_Q

    return JointAngles(hip_x=α, hip_y=β, knee=γ)


# ── Round-trip self test ─────────────────────────────────────────────────

def _selftest() -> None:
    """Verify fk(ik(target)) ≈ target for the nominal pose + random feasible
    targets across all four legs."""
    import random
    random.seed(0)
    # Nominal stand: matches NOMINAL_POSE in omniquad_rl_agent.py
    nominal = {
        "FL": JointAngles(+0.30, +0.30, -0.60),
        "FR": JointAngles(-0.30, +0.30, -0.60),
        "RL": JointAngles(+0.30, +0.30, -0.60),
        "RR": JointAngles(-0.30, +0.30, -0.60),
    }
    print("== nominal-pose foot positions (body frame) ==")
    for leg, q in nominal.items():
        foot = forward_kinematics(leg, q)
        back = inverse_kinematics(leg, foot)
        if back is None:
            print(f"  {leg}: FK={foot}  IK=UNREACHABLE")
            continue
        foot2 = forward_kinematics(leg, back)
        err = max(abs(a - b) for a, b in zip(foot, foot2))
        print(f"  {leg}: foot=({foot[0]:+.4f},{foot[1]:+.4f},{foot[2]:+.4f})  "
              f"q_in=({q.hip_x:+.3f},{q.hip_y:+.3f},{q.knee:+.3f}) "
              f"q_out=({back.hip_x:+.3f},{back.hip_y:+.3f},{back.knee:+.3f}) "
              f"round-trip err={err:.2e}")

    print("\n== random reachable targets per leg ==")
    n_tests = 50
    max_err = 0.0
    fails = 0
    for leg in HIP_X_POS:
        for _ in range(n_tests):
            # Sample a target near the nominal foot, varying ±10cm in xyz.
            nfoot = forward_kinematics(leg, nominal[leg])
            jitter = tuple(random.uniform(-0.1, 0.1) for _ in range(3))
            target = (nfoot[0] + jitter[0], nfoot[1] + jitter[1], nfoot[2] + jitter[2])
            q = inverse_kinematics(leg, target)
            if q is None:
                fails += 1
                continue
            foot = forward_kinematics(leg, q)
            err = max(abs(a - b) for a, b in zip(target, foot))
            max_err = max(max_err, err)
    print(f"  {n_tests*4} random targets ({n_tests}/leg): "
          f"max round-trip err = {max_err:.2e}, unreachable = {fails}")
    assert max_err < 1e-6, "IK round-trip failed"
    print("\n  OK — round-trip error < 1 micron across all legs.")


if __name__ == "__main__":
    _selftest()
