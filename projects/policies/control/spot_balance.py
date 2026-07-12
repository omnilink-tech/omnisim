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

"""Body-pose PD stabilizer for Spot.

Reads body roll and pitch each control tick and returns a per-leg z
offset to add to the gait-engine foot target. The offsets level the
body by EXTENDING legs on the high side / CONTRACTING legs on the low
side, which (with the foot planted) rotates the body back to level.

Sign convention (must be empirically verified against Webots'
getOrientation output):
  pitch > 0  ≡ body nose-up
    correction: CONTRACT front (z target up) + EXTEND rear (z target down)
    so the body rotates nose-down -> back to level
  roll > 0   ≡ body banking right (right side down -- check Webots)
    correction: EXTEND right (down) + CONTRACT left (up)
    so the body rotates left -> back to level

Each correction has its own gain (kp_roll, kp_pitch) and damping on
the body angular rate (kd_*). The total offset per leg is clipped to
prevent the IK from going out of reach.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict


# Per-leg sign of the pitch / roll correction.
# Pitch: front legs get +1 (contract when pitched up), rear legs -1.
# Roll: right legs get +1 (extend when rolled right), left legs -1.
# These signs are encoded so dz_leg = sign_pitch * pitch_corr + sign_roll * roll_corr.
LEG_PITCH_SIGN: Dict[str, float] = {
    "FL": +1.0, "FR": +1.0,
    "RL": -1.0, "RR": -1.0,
}
LEG_ROLL_SIGN: Dict[str, float] = {
    "FL": -1.0, "RL": -1.0,
    "FR": +1.0, "RR": +1.0,
}


@dataclass
class BalanceParams:
    """Gains and limits for the body-pose PD."""
    kp_pitch: float = 0.10   # m per rad of pitch
    kd_pitch: float = 0.02   # m per (rad/s) of pitch rate
    kp_roll: float = 0.10
    kd_roll: float = 0.02
    # Clip each leg's total balance dz to ±this so the IK target stays
    # in reach. 0.05 m is plenty for body-pose corrections without
    # pushing the leg near its workspace boundary.
    max_dz: float = 0.05


def balance_offsets(
    roll: float, pitch: float,
    roll_rate: float, pitch_rate: float,
    p: BalanceParams,
) -> Dict[str, float]:
    """Return {leg: dz_offset} additive to the gait foot z target."""
    pitch_term = p.kp_pitch * pitch + p.kd_pitch * pitch_rate
    roll_term = p.kp_roll * roll + p.kd_roll * roll_rate
    out: Dict[str, float] = {}
    for leg in ("FL", "FR", "RL", "RR"):
        dz = LEG_PITCH_SIGN[leg] * pitch_term + LEG_ROLL_SIGN[leg] * roll_term
        # Clip to keep the IK target inside the reachable workspace.
        if dz > p.max_dz:
            dz = p.max_dz
        elif dz < -p.max_dz:
            dz = -p.max_dz
        out[leg] = dz
    return out
