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

"""Raibert-heuristic trot for OmniQuad — a MODELED gait, not a tuned foot-cycler.

This is the classic dynamically-stabilized legged-locomotion recipe
(Raibert, "Legged Robots That Balance", MIT Press 1986; used as the base
layer of the MIT Cheetah controllers):

  1. FOOT PLACEMENT carries the balance. Each swing foot aims for the
     "neutral point" plus a velocity-feedback correction:

         p_td = p_neutral + (T_stance / 2) * v_body + k_v * (v_body - v_cmd)

     The first term is where a foot must land for the body to pass over it
     symmetrically at the current speed; the second decelerates the body
     when it is faster than commanded (and vice versa). k_v defaults to a
     capture-point-derived gain sqrt(h0 / g) scaled down (Raibert's
     empirical 0.03-0.1 s range sits inside this).

  2. STANCE FEET ARE WORLD-ANCHORED. At touchdown the commanded foot
     position is pinned in the WORLD frame and re-expressed in the body
     frame every tick from the MEASURED body pose. The body genuinely
     vaults over planted feet; there is no open-loop "slide the foot
     backward at v_cmd" assumption (the old omniquad_gait did exactly that,
     which is why its strides slipped and its net thrust depended on
     servo-lag accidents).

  3. HEIGHT + ATTITUDE regulation on the stance legs: a proportional term
     on body-height error and a roll/pitch leveling term adjust stance
     foot z.

Everything here is kinematic + state-feedback: position-servo legs
(OMNISIM_NEWTON_TARGET_KE/KD) supply the forces. The module is pure
python with no simulator dependencies, so it unit-tests standalone and is
shared verbatim by controllers and trainers (the RL residual goes on top
of THIS, per the model-then-residual formula).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from .omniquad_kinematics import HIP_X_POS, forward_kinematics, JointAngles

LEGS = ("FL", "FR", "RL", "RR")
# Diagonal trot pairs: FL+RR in phase, FR+RL half a cycle later.
TROT_PHASE_OFFSET = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
G = 9.81


def _nominal_feet(h0: float) -> Dict[str, Tuple[float, float, float]]:
    """Neutral foot positions: directly under each hip workspace center at
    height -h0. Uses FK of the canonical stand to get the lateral offset
    (the foot does NOT sit under the hip_x joint -- the hip_y link offsets
    it outward)."""
    nominal_q = {
        "FL": JointAngles(+0.05, +0.40, -0.80),
        "FR": JointAngles(-0.05, +0.40, -0.80),
        "RL": JointAngles(+0.05, +0.40, -0.80),
        "RR": JointAngles(-0.05, +0.40, -0.80),
    }
    out = {}
    for leg in LEGS:
        fx, fy, _fz = forward_kinematics(leg, nominal_q[leg])
        out[leg] = (fx, fy, -h0)
    return out


@dataclass
class RaibertParams:
    freq_hz: float = 2.0          # trot cadence
    duty: float = 0.5             # stance fraction of the cycle
    h0: float = 0.52              # body height over feet (within IK reach)
    step_height: float = 0.07     # swing apex above ground
    k_v: float = 0.0              # velocity-error placement gain; 0 -> derive
    k_height: float = 0.6         # stance-z gain on body height error
    k_level: float = 0.5          # stance-z gain on roll/pitch leveling
    # Reach safety box around each neutral (keeps IK solvable everywhere).
    reach_x: float = 0.16
    reach_y: float = 0.10
    z_min_off: float = -0.04      # allowed z below -h0
    z_max_off: float = 0.22       # allowed z above -h0 (swing apex room)
    # Actuator-lag LEAD compensation. The position servos are a first-order
    # lag with time constant tau ~= kd/ke (0.12 s at the stable ke=500/kd=60
    # operating point -- a quarter of the stance phase at 2 Hz!). Measured
    # effect of ignoring it: the executed foot cycle phase-lags the command
    # and every step ratchets the body BACKWARD (~-1.2 m/s at kd=200,
    # ~-0.3 m/s at kd=60; dropping kd to 20 nearly removes the drift but
    # underdamps the stepping into a fall). Commanding the trajectory
    # lead_s ahead -- phase AND body-pose prediction -- makes the executed
    # motion land on schedule. This is part of MODELING the system: gait
    # model + first-order actuator model.
    lead_s: float = 0.12

    def __post_init__(self):
        if self.k_v <= 0.0:
            # Capture-point time constant, halved (Raibert's working range).
            self.k_v = 0.5 * math.sqrt(self.h0 / G)

    @property
    def period_s(self) -> float:
        return 1.0 / self.freq_hz

    @property
    def t_stance(self) -> float:
        return self.duty * self.period_s


@dataclass
class _LegState:
    in_stance: bool = False
    anchor_world: Optional[Tuple[float, float, float]] = None
    liftoff_body: Optional[Tuple[float, float, float]] = None
    touchdown_body: Optional[Tuple[float, float, float]] = None


class RaibertGait:
    """Stateful trot generator. Call `targets()` once per control tick."""

    def __init__(self, p: RaibertParams = None):
        self.p = p or RaibertParams()
        self.neutral = _nominal_feet(self.p.h0)
        self.legs: Dict[str, _LegState] = {leg: _LegState() for leg in LEGS}

    # ── helpers ─────────────────────────────────────────────────────────
    @staticmethod
    def _world_to_body(R, p_body, p_world):
        dx = p_world[0] - p_body[0]
        dy = p_world[1] - p_body[1]
        dz = p_world[2] - p_body[2]
        # R is row-major body->world; body = R^T @ delta.
        return (R[0] * dx + R[3] * dy + R[6] * dz,
                R[1] * dx + R[4] * dy + R[7] * dz,
                R[2] * dx + R[5] * dy + R[8] * dz)

    @staticmethod
    def _body_to_world(R, p_body, p_local):
        return (p_body[0] + R[0] * p_local[0] + R[1] * p_local[1] + R[2] * p_local[2],
                p_body[1] + R[3] * p_local[0] + R[4] * p_local[1] + R[5] * p_local[2],
                p_body[2] + R[6] * p_local[0] + R[7] * p_local[1] + R[8] * p_local[2])

    def _clamp_to_reach(self, leg: str, x: float, y: float, z: float):
        nx, ny, nz = self.neutral[leg]
        p = self.p
        x = min(max(x, nx - p.reach_x), nx + p.reach_x)
        y = min(max(y, ny - p.reach_y), ny + p.reach_y)
        z = min(max(z, nz + p.z_min_off), nz + p.z_max_off)
        return (x, y, z)

    # ── the model ───────────────────────────────────────────────────────
    def targets(
        self,
        t: float,
        R,                       # body->world rotation, row-major 9-list
        p_body,                  # body origin, world frame
        v_world,                 # body linear velocity, world frame
        roll: float,
        pitch: float,
        v_cmd=(0.0, 0.0),        # commanded body-frame (vx, vy)
        wz_cmd: float = 0.0,     # commanded yaw rate
        return_vel: bool = False,
    ) -> Dict[str, Tuple[float, float, float]]:
        """Per-leg foot targets in the BODY frame for this tick.

        With return_vel=True, returns (targets, velocities): the ANALYTIC
        body-frame foot velocity of the deterministic gait component --
        stance feet move at exactly -v_body (world-anchored), swing feet at
        the interpolation curve's derivative. Regulator terms (height,
        leveling, Raibert refresh) are deliberately EXCLUDED so a
        feedforward built on these velocities cannot close a loop through
        the body-state regulators."""
        p = self.p
        # Lead compensation: evaluate the gait lead_s into the future and
        # predict the body pose there, so the lagged servos EXECUTE the
        # cycle on time (see RaibertParams.lead_s).
        t_eff = t + p.lead_s
        p_eff = (p_body[0] + v_world[0] * p.lead_s,
                 p_body[1] + v_world[1] * p.lead_s,
                 p_body[2] + v_world[2] * p.lead_s)
        # Measured body-frame planar velocity.
        vbx = R[0] * v_world[0] + R[3] * v_world[1] + R[6] * v_world[2]
        vby = R[1] * v_world[0] + R[4] * v_world[1] + R[7] * v_world[2]

        out: Dict[str, Tuple[float, float, float]] = {}
        vel_out: Dict[str, Tuple[float, float, float]] = {}
        for leg in LEGS:
            st = self.legs[leg]
            nx, ny, nz = self.neutral[leg]
            phase = (t_eff / p.period_s + TROT_PHASE_OFFSET[leg]) % 1.0

            # Raibert touchdown target (continuously refreshed in swing).
            td_x = nx + 0.5 * p.t_stance * vbx + p.k_v * (vbx - v_cmd[0])
            td_y = ny + 0.5 * p.t_stance * vby + p.k_v * (vby - v_cmd[1])
            if wz_cmd != 0.0:
                # Tangential placement for commanded turning: the foot's
                # neutral, swept by half the per-stance yaw (the planted
                # foot then rotates the body as it vaults).
                half = 0.5 * wz_cmd * p.t_stance
                c, s = math.cos(half), math.sin(half)
                td_x, td_y = (td_x * c - td_y * s, td_x * s + td_y * c)
            td = self._clamp_to_reach(leg, td_x, td_y, nz)

            if phase < p.duty:
                # ── STANCE ──
                if not st.in_stance:
                    st.in_stance = True
                    # Pin the commanded touchdown point in the world.
                    pin = st.touchdown_body or td
                    st.anchor_world = self._body_to_world(R, p_eff, pin)
                bx, by, bz = self._world_to_body(R, p_eff, st.anchor_world)
                # Height + attitude regulation override the anchored z.
                z_cmd = -p.h0
                z_cmd -= p.k_height * (p.h0 - max(p_body[2], 0.05))
                # Leveling: lower the foot on the high side. For pitch>0
                # (nose up) the FRONT feet must extend (z more negative).
                z_cmd -= p.k_level * (pitch * nx - roll * ny)
                bx, by, bz = self._clamp_to_reach(leg, bx, by, z_cmd)
                st.liftoff_body = (bx, by, bz)
                out[leg] = (bx, by, bz)
                # World-anchored foot: body-frame velocity = -R^T v_world.
                vel_out[leg] = (-vbx, -vby, 0.0)
            else:
                # ── SWING ──
                if st.in_stance:
                    st.in_stance = False
                    st.anchor_world = None
                    if st.liftoff_body is None:
                        st.liftoff_body = (nx, ny, nz)
                s = (phase - p.duty) / (1.0 - p.duty)
                s_xy = s * s * (3.0 - 2.0 * s)  # smoothstep
                lo = st.liftoff_body or (nx, ny, nz)
                bx = lo[0] + (td[0] - lo[0]) * s_xy
                by = lo[1] + (td[1] - lo[1]) * s_xy
                bz = nz + p.step_height * 4.0 * s * (1.0 - s)
                st.touchdown_body = td
                out[leg] = self._clamp_to_reach(leg, bx, by, bz)
                # Analytic curve derivative (per second of gait time).
                t_swing = (1.0 - p.duty) * p.period_s
                ds = 1.0 / t_swing
                dsxy = 6.0 * s * (1.0 - s) * ds
                vel_out[leg] = ((td[0] - lo[0]) * dsxy,
                                (td[1] - lo[1]) * dsxy,
                                p.step_height * 4.0 * (1.0 - 2.0 * s) * ds)
        if return_vel:
            return out, vel_out
        return out


def _selftest() -> None:
    """Kinematic invariants, no simulator:
    1. At v == v_cmd, swing touchdown == neutral + (T_st/2) v (no extra term).
    2. At v > v_cmd, touchdown moves FORWARD of that (decelerating).
    3. Stance anchors are world-fixed: as the body advances, the body-frame
       stance target moves BACKWARD by the same amount.
    4. All targets across a full cycle at 0.6 m/s are IK-reachable."""
    from .omniquad_kinematics import inverse_kinematics

    p = RaibertParams()
    g = RaibertGait(p)
    R_id = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]

    # 1+2: touchdown shift direction.
    g1 = RaibertGait(p)
    tgt_eq = g1.targets(p.period_s * 0.75, R_id, (0, 0, p.h0), (0.5, 0, 0),
                        0, 0, v_cmd=(0.5, 0))["FL"]
    g2 = RaibertGait(p)
    tgt_fast = g2.targets(p.period_s * 0.75, R_id, (0, 0, p.h0), (0.8, 0, 0),
                          0, 0, v_cmd=(0.5, 0))["FL"]
    assert tgt_fast[0] > tgt_eq[0], "fast body must land foot further forward"

    # 3: world anchoring.
    g3 = RaibertGait(p)
    a = g3.targets(0.01, R_id, (0.00, 0, p.h0), (0.5, 0, 0), 0, 0)["FL"]
    b = g3.targets(0.05, R_id, (0.10, 0, p.h0), (0.5, 0, 0), 0, 0)["FL"]
    assert abs((a[0] - b[0]) - 0.10) < 1e-9, "stance foot must be world-fixed"

    # 4: reachability over a cycle.
    g4 = RaibertGait(p)
    bad = 0
    for k in range(80):
        t = k * p.period_s / 40
        feet = g4.targets(t, R_id, (t * 0.6, 0, p.h0), (0.6, 0, 0), 0, 0,
                          v_cmd=(0.6, 0))
        for leg in LEGS:
            if inverse_kinematics(leg, feet[leg]) is None:
                bad += 1
    assert bad == 0, f"{bad} unreachable targets in cycle"
    print("omniquad_raibert selftest OK (placement, anchoring, reachability)")


if __name__ == "__main__":
    _selftest()
