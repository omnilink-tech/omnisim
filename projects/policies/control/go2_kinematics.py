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

"""Analytic forward + inverse kinematics for Unitree Go2 legs.

THE GO2 PORT OF projects/policies/control/spot_kinematics.py -- identical 3-link
leg topology (hip_x abduction, hip_y/thigh pitch, knee/calf pitch), only the
geometry constants differ. Dimensions extracted from
projects/robots/unitree/go2/urdf/go2.urdf (==unitree_ros go2_description):

    base
      -> hip   joint at body-frame (HIP_X, HIP_Y, 0)        axis (1,0,0)
      -> hip   link
      -> thigh joint at (0, ±HIPY_OFFSET, 0)                axis (0,1,0)
      -> thigh link
      -> calf  joint at (KNEE_X=0, 0, -L1)                  axis (0,1,0)
      -> calf  link
      -> foot tip at (0, 0, -L2)

Go2 leg is symmetric: L1 == L2 == 0.213 m, no knee x-offset (KNEE_X = 0).
The hip abduction joint sits HIPY_OFFSET = 0.0955 m inboard of the thigh
plane. Frame convention: body frame is +X forward, +Y left, +Z up.

Go2 joint names per leg <L> in {FL,FR,RL,RR}: <L>_hip_joint (abduction),
<L>_thigh_joint (hip pitch), <L>_calf_joint (knee). Controller order
everywhere: FL,FR,RL,RR x (hip, thigh, calf) -- the same order the deploy
bridge and the GPU trainers use.

Same closed-form IK math as spot_kinematics; `python go2_kinematics.py`
self-tests fk(ik(target)) ~ target across all four legs.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional, Tuple

# ── Geometry (meters) -- from go2.urdf ───────────────────────────────────
L1 = 0.213            # thigh: thigh joint -> calf joint, z component magnitude
L2 = 0.213            # calf:  calf joint  -> foot tip
KNEE_X = 0.0          # Go2 calf is inline with the thigh (no forward offset)
HIPY_OFFSET = 0.0955  # lateral offset of the thigh joint from the hip joint

# Hip (abduction) joint positions in body frame, keyed by leg id.
#   FL = front_left, FR = front_right, RL = rear_left, RR = rear_right
HIP_X_POS = {
    "FL": (+0.1934, +0.0465, 0.0),
    "FR": (+0.1934, -0.0465, 0.0),
    "RL": (-0.1934, +0.0465, 0.0),
    "RR": (-0.1934, -0.0465, 0.0),
}

# Side sign of the thigh-plane offset for each leg (left = +, right = -).
HIPY_SIGN = {"FL": +1.0, "FR": -1.0, "RL": +1.0, "RR": -1.0}


@dataclass(frozen=True)
class JointAngles:
    """One leg's three joint angles, in URDF convention."""
    hip_x: float   # abduction (<L>_hip_joint)
    hip_y: float   # thigh / hip pitch (<L>_thigh_joint)
    knee: float    # calf / knee (<L>_calf_joint)

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.hip_x, self.hip_y, self.knee)


# ── Forward kinematics ───────────────────────────────────────────────────

def forward_kinematics(leg: str, q: JointAngles) -> Tuple[float, float, float]:
    """Return the foot tip position in the body frame.

    Chain: foot = hip_pos + R_x(α) [ (0,±HIPY_OFFSET,0)
                                      + R_y(β) [ (KNEE_X,0,-L1)
                                                 + R_y(γ) (0,0,-L2) ] ]
    """
    α, β, γ = q.as_tuple()
    cg, sg = math.cos(γ), math.sin(γ)
    shank_x = -L2 * sg
    shank_z = -L2 * cg
    upper_x = KNEE_X + shank_x
    upper_z = -L1 + shank_z
    cb, sb = math.cos(β), math.sin(β)
    hip_x_pt = upper_x * cb + upper_z * sb
    hip_z_pt = -upper_x * sb + upper_z * cb
    H = HIPY_SIGN[leg] * HIPY_OFFSET
    hipframe_y = H
    ca, sa = math.cos(α), math.sin(α)
    body_x = hip_x_pt
    body_y = hipframe_y * ca - hip_z_pt * sa
    body_z = hipframe_y * sa + hip_z_pt * ca
    Hx, Hy, Hz = HIP_X_POS[leg]
    return (Hx + body_x, Hy + body_y, Hz + body_z)


# ── Inverse kinematics ───────────────────────────────────────────────────

def inverse_kinematics(
    leg: str, target_body: Tuple[float, float, float]
) -> Optional[JointAngles]:
    """Return the joint angles that put the foot tip at `target_body`, or
    None if the target is outside the leg's reachable workspace.

    Closed-form (same derivation as spot_kinematics.inverse_kinematics):
      1. α (hip abduction) constrains the foot's hip-frame Y component to
         equal the thigh-plane offset H.
      2. γ (calf) from the cosine law on the thigh/calf/hip-to-foot triangle.
      3. β (thigh) is the in-plane rotation placing the shank endpoint at the
         target relative position.
    """
    Hx, Hy, Hz = HIP_X_POS[leg]
    tx, ty, tz = target_body
    Tx, Ty, Tz = tx - Hx, ty - Hy, tz - Hz
    H = HIPY_SIGN[leg] * HIPY_OFFSET

    r_yz = math.hypot(Ty, Tz)
    if r_yz < 1e-9 or abs(H / r_yz) > 1.0:
        return None
    φ = math.atan2(Tz, Ty)
    α = φ + math.acos(H / r_yz)
    while α > math.pi: α -= 2 * math.pi
    while α < -math.pi: α += 2 * math.pi

    sa, ca = math.sin(α), math.cos(α)
    T_hip_z = -Ty * sa + Tz * ca
    Qx = Tx
    Qz = T_hip_z

    Q2 = Qx * Qx + Qz * Qz
    R_link = math.hypot(L1, KNEE_X)
    phi_k = math.atan2(KNEE_X, L1)
    C = (Q2 - KNEE_X * KNEE_X - L1 * L1 - L2 * L2) / (2.0 * L2)
    cos_arg = C / R_link
    if abs(cos_arg) > 1.0:
        return None
    # Go2's knee bends backward (γ < 0, valid range [-2.72,-0.84]): pick -acos.
    γ = -math.acos(cos_arg) - phi_k

    Sx = KNEE_X - L2 * math.sin(γ)
    Sz = -L1 - L2 * math.cos(γ)
    angle_S = math.atan2(Sz, Sx)
    angle_Q = math.atan2(Qz, Qx)
    β = angle_S - angle_Q

    return JointAngles(hip_x=α, hip_y=β, knee=γ)


# ── Round-trip self test ─────────────────────────────────────────────────

def _selftest() -> None:
    import random
    random.seed(0)
    # A plausible Go2 stand (thigh pitch ~0.8, calf ~-1.5); round-trip works
    # for any reachable pose. The gait module computes the canonical NOMINAL.
    nominal = {
        "FL": JointAngles(0.0, +0.80, -1.50),
        "FR": JointAngles(0.0, +0.80, -1.50),
        "RL": JointAngles(0.0, +0.80, -1.50),
        "RR": JointAngles(0.0, +0.80, -1.50),
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
            nfoot = forward_kinematics(leg, nominal[leg])
            jitter = tuple(random.uniform(-0.06, 0.06) for _ in range(3))
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
