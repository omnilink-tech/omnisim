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

"""Trot foot-trajectory generator for Spot.

Produces a target foot position in the body frame for each of the four
legs as a function of time and commanded body velocity. The body knows
where each foot should be; this module knows how to move feet so that
the body moves at `(vx, vy, wz)` without slip.

Trot specifics:
  - Diagonal pairs swing together: FL+RR are in phase, FR+RL are
    half-cycle offset. At any instant the body is supported by exactly
    two diagonally-opposite feet (and the body can't tip in the
    sagittal+lateral plane spanned by them).
  - Duty factor: fraction of the gait period each foot spends in
    stance. 0.5 means swing and stance are equal — the standard trot.
  - Step length is set by speed × stance duration so that, with the
    foot planted, the body moves the step length exactly during stance.
    Result: no slip, no body deceleration.
  - Swing height is a small parabolic arc so the foot clears the
    ground without overshooting (which costs forward velocity).

Inputs: time, body velocity command. Outputs: 4 foot positions in
body frame, ready to feed to `spot_kinematics.inverse_kinematics`.

Stance/swing math (per leg, with phase φ ∈ [0, 1)):

    if φ < duty:            # STANCE
        s = φ / duty           # 0 -> 1
        foot_x = neutral_x + step_x/2 - step_x * s
        foot_y = neutral_y + step_y/2 - step_y * s
        foot_z = ground_z      # planted on ground
    else:                    # SWING
        s = (φ - duty) / (1 - duty)   # 0 -> 1
        foot_x = neutral_x - step_x/2 + step_x * s
        foot_y = neutral_y - step_y/2 + step_y * s
        foot_z = ground_z + step_height * 4 * s * (1 - s)

`step_x = vx * stance_duration` is the per-cycle linear displacement
needed for slip-free walking. `step_y` lets us drift sideways.
Heading change (wz) is added by yaw-rotating each foot's neutral
position about the body — feet on the inside of the turn step shorter,
feet on the outside step longer.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Dict, Tuple

from .spot_kinematics import HIP_X_POS

LEGS = ("FL", "FR", "RL", "RR")

# Trot diagonal pairs: FL+RR share phase 0.0, FR+RL share phase 0.5
TROT_PHASE_OFFSET: Dict[str, float] = {
    "FL": 0.00,
    "RR": 0.00,
    "FR": 0.50,
    "RL": 0.50,
}


@dataclass
class GaitParams:
    period_s: float = 0.50         # full cycle time (2 Hz)
    duty: float = 0.50             # fraction in stance per leg
    step_height: float = 0.06      # peak swing height (m)
    # Neutrals tuned empirically against ODE+kp=20 to actually engage
    # the ground each stance cycle. With body settling at ~0.67 m under
    # the agent's PD gains, ground_z=-0.56 puts the foot at world z=0.11
    # at the neutral position; the body sinks dynamically during stance
    # so the foot contacts the floor each cycle and pushes the body
    # forward. Matched-nominal foot geometry (z=-0.559) leaves feet
    # floating; legs-narrower neutrals (e.g. lateral_y=0.18) put the
    # leg into a near-vertical singular pose. These values produced
    # 4.19m / 30s at +/-0.67m lateral with no policy.
    ground_z: float = -0.56
    neutral_lateral_y: float = 0.30
    neutral_front_x: float = 0.30
    neutral_rear_x: float = -0.28

    @property
    def stance_duration(self) -> float:
        return self.period_s * self.duty

    @property
    def swing_duration(self) -> float:
        return self.period_s * (1.0 - self.duty)


def neutral_foot_positions(p: GaitParams) -> Dict[str, Tuple[float, float, float]]:
    """The "rest" foot position per leg in body frame, around which the
    stance/swing cycle pivots. Symmetric front/back and left/right."""
    nx_f = p.neutral_front_x
    nx_r = p.neutral_rear_x
    ny = p.neutral_lateral_y
    z = p.ground_z
    return {
        "FL": (nx_f, +ny, z),
        "FR": (nx_f, -ny, z),
        "RL": (nx_r, +ny, z),
        "RR": (nx_r, -ny, z),
    }


def foot_targets(
    t: float,
    vx: float,
    vy: float,
    wz: float,
    p: GaitParams,
) -> Dict[str, Tuple[float, float, float]]:
    """Return a dict {leg: (foot_x, foot_y, foot_z)} of foot targets in
    the body frame at time `t`.

    `vx`, `vy`: commanded body translational velocity (m/s).
    `wz`:       commanded body yaw rate (rad/s, positive = turn left).
    """
    step_x = vx * p.stance_duration
    step_y = vy * p.stance_duration
    neutrals = neutral_foot_positions(p)
    out: Dict[str, Tuple[float, float, float]] = {}

    # Yaw-rate steering: a planted foot is fixed in the WORLD while the
    # body yaws at +wz (CCW), so in the BODY frame the foot must sweep
    # tangentially about the body center at -wz — from +yaw_sweep/2 at
    # stance start to -yaw_sweep/2 at touch-down-relative stance end —
    # and sweep back (+) during swing, exactly analogous to the step_x
    # slide. (The previous implementation rotated the neutral by a
    # CONSTANT half-angle at every phase: that only statically reorients
    # the stance and produces NO sustained yaw — measured on the Newton
    # backend, commanded wz=+0.4 yielded -0.065 rad/s and wz=-0.4 yielded
    # +0.214 rad/s, i.e. inverted/asymmetric slip residue — which made
    # every heading-hold PD AMPLIFY drift instead of correcting it.)
    yaw_sweep = wz * p.stance_duration  # body yaw per stance phase (rad)

    for leg in LEGS:
        nx, ny, nz = neutrals[leg]

        phase = (t / p.period_s + TROT_PHASE_OFFSET[leg]) % 1.0
        if phase < p.duty:
            # Stance: foot starts ahead of neutral by step/2, slides
            # back to neutral - step/2 as body moves forward.
            s = phase / p.duty  # 0 -> 1
            ang = yaw_sweep * (0.5 - s)  # +half -> -half over stance
            if ang != 0.0:
                c, sn = math.cos(ang), math.sin(ang)
                nx, ny = nx * c - ny * sn, nx * sn + ny * c
            foot_x = nx + step_x * 0.5 - step_x * s
            foot_y = ny + step_y * 0.5 - step_y * s
            foot_z = nz
        else:
            # Swing: foot lifts off the ground, swings from
            # neutral - step/2 back to neutral + step/2 (ready for next
            # stance). Parabolic z arc 4*s*(1-s) peaks at 1.0 at s=0.5.
            s = (phase - p.duty) / (1.0 - p.duty)  # 0 -> 1
            ang = yaw_sweep * (s - 0.5)  # -half -> +half over swing
            if ang != 0.0:
                c, sn = math.cos(ang), math.sin(ang)
                nx, ny = nx * c - ny * sn, nx * sn + ny * c
            foot_x = nx - step_x * 0.5 + step_x * s
            foot_y = ny - step_y * 0.5 + step_y * s
            foot_z = nz + p.step_height * 4.0 * s * (1.0 - s)

        out[leg] = (foot_x, foot_y, foot_z)

    return out


def _selftest() -> None:
    """Print one cycle of foot trajectories at vx=0.5 to verify shape."""
    p = GaitParams()
    print(f"period={p.period_s}s duty={p.duty} stance={p.stance_duration}s "
          f"swing={p.swing_duration}s")
    print(f"vx=0.5 -> step_x={0.5 * p.stance_duration:.3f}m  "
          f"step_height={p.step_height}m")
    print()
    print(f"{'t':>5s} | {'leg':>3s} | {'phase':>5s} | "
          f"{'phase':>5s} | {'fx':>6s} {'fy':>6s} {'fz':>6s}")
    print("-" * 60)
    for k in range(0, 11):
        t = k * p.period_s / 10  # one full period in 10 ticks
        feet = foot_targets(t, vx=0.5, vy=0.0, wz=0.0, p=p)
        for leg in LEGS:
            phase = (t / p.period_s + TROT_PHASE_OFFSET[leg]) % 1.0
            ph_label = "STANCE" if phase < p.duty else "swing"
            fx, fy, fz = feet[leg]
            print(f"{t:>5.2f} | {leg:>3s} | {phase:>5.2f} | "
                  f"{ph_label:>5s} | {fx:+.3f} {fy:+.3f} {fz:+.3f}")
        print()


if __name__ == "__main__":
    _selftest()
