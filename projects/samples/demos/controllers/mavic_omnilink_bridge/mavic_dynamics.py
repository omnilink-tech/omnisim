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

"""Rotor dynamics for the URDF Mavic 2 Pro.

The legacy Mavic2Pro PROTO embedded Webots `Propeller` nodes whose
thrustConstants / torqueConstants fields produced lift + reaction
torque automatically as a function of motor angular velocity. URDF has
no Propeller analogue — the continuous-rotation prop joints in the URDF
spin the visual rotors but generate no aerodynamic forces.

This module replaces that bit. Each control tick the bridge calls
`RotorDynamics.step(fl, fr, rl, rr)` with the four motor target
velocities it just sent to `motor.setVelocity()`. We then apply, via
the Webots supervisor API:

  - per-propeller lift force F_i = k_thrust * |ω_i|² along the body
    +Z axis at the propeller's anchor (body-relative)
  - per-tick yaw torque proportional to the asymmetry between the two
    diagonal pairs of propellers, modelling the reaction-torque
    differential a real quadcopter uses to steer

The thrust + torque coefficients are copied verbatim from the legacy
PROTO so a properly-tuned controller produces equivalent flight to
what it did against the PROTO.
"""

from __future__ import annotations

from typing import Tuple


# FORCE anchor offsets in the base_link frame. The x values are the URDF joint
# origins MINUS their centroid (-0.061163), NOT the raw origins: the URDF's CoM
# sits at the link origin, and applying hover thrust at the raw anchors put a
# constant ~0.6 N*m nose-up torque on the airframe -- more than the
# stabiliser's whole authority, so every takeoff pitched over (public issue
# #10). A real quad's CoM sits at the prop-rectangle centre by design; the
# URDF cannot express that here (an inertial-origin offset explodes the
# imported body at rest, measured in the same issue), so the correction lives
# on the force side: same spans, centroid moved onto the CoM.
_PROP_FL: Tuple[float, float, float] = (0.116017, 0.151294, -0.003)
_PROP_FR: Tuple[float, float, float] = (0.116017, -0.151294, -0.003)
_PROP_RL: Tuple[float, float, float] = (-0.116016, 0.127453, -0.032)
_PROP_RR: Tuple[float, float, float] = (-0.116016, -0.127453, -0.032)


class RotorDynamics:
    """Supervisor-driven thrust/torque applier for a URDF quadcopter.

    Construct once at controller startup with the supervisor handle and
    the robot node (i.e. `supervisor.getSelf()`). Call `step(...)` once
    per control tick AFTER the bridge has issued its `setVelocity()`
    calls but BEFORE the next `supervisor.step()`.
    """

    # Lift coefficient per Webots Propeller convention: F = K * ω².
    # ⚠ NOT the PROTO's 0.00026 any more (public issue #10): that constant was
    # paired with the legacy PROTO's airframe, and this bridge flies the URDF,
    # whose authored masses sum to 1.0333 kg (10.14 N). At the controller's
    # hover setpoint ω = K_VERTICAL_THRUST = 68.5 rad/s, 0.00026 gives
    # 4 × 0.00026 × 68.5² = 4.88 N — 48% of the weight — so the demo sat on
    # the ground for ever (measured: z pinned at ~0.12 m, tipped to |roll|=π,
    # skidded 6.5 m). Calibrate k to the airframe instead of the other way
    # round: k = m g / (4 ω_h²) = 1.0333 × 9.81 / (4 × 68.5²) = 0.00054, so
    # ω = 68.5 hovers exactly and the verbatim mavic2pro.py control law keeps
    # its operating point. Attitude torque per unit motor-delta scales with k
    # (×2.08); the measured hover/climb after this change is in issue #10.
    k_thrust: float = 0.00054

    # Reaction-torque coefficient. PROTO had torqueConstants 0.0000052
    # for all four props (yaw resolved by spin-direction signs at the
    # motor.setVelocity layer; we read the same signs back here).
    k_torque: float = 0.0000052

    def __init__(self, robot_node) -> None:
        self.robot = robot_node

    def step(self, fl: float, fr: float, rl: float, rr: float) -> None:
        """Apply rotor forces + yaw torque for one control tick.

        Arguments are the four motor target velocities the bridge most
        recently passed to `motor.setVelocity()` (front_left,
        front_right, rear_left, rear_right). Signs are preserved so the
        yaw-torque term inherits the bridge's CW/CCW convention.
        """
        # SIGNED squared velocity, u·|u| -- the Webots Propeller convention
        # (thrust = thrustConstants[0] · ω · |ω|). The old u² made a NEGATIVE
        # mixer output lift UPWARD: the stabiliser's corrections saturate
        # through zero under a large attitude error, so the side that should
        # have pushed down pushed up, and every big correction amplified
        # itself (public issue #10's ±2 rad oscillation).
        fl2 = fl * abs(fl)
        fr2 = fr * abs(fr)
        rl2 = rl * abs(rl)
        rr2 = rr * abs(rr)

        # Per-propeller lift. addForceWithOffset(force, offset, relative=True)
        # applies the force in body coordinates at the body-relative offset.
        self.robot.addForceWithOffset([0.0, 0.0, self.k_thrust * fl2], list(_PROP_FL), True)
        self.robot.addForceWithOffset([0.0, 0.0, self.k_thrust * fr2], list(_PROP_FR), True)
        self.robot.addForceWithOffset([0.0, 0.0, self.k_thrust * rl2], list(_PROP_RL), True)
        self.robot.addForceWithOffset([0.0, 0.0, self.k_thrust * rr2], list(_PROP_RR), True)

        # Yaw torque from diagonal-pair asymmetry. The bridge already
        # encoded the yaw command into pair magnitudes (FR + RL bigger
        # for positive yaw, FL + RR bigger for negative yaw); we read
        # the same magnitudes here so the sign comes out right without
        # tracking spin direction separately.
        yaw_tau = self.k_torque * ((fr2 + rl2) - (fl2 + rr2))
        self.robot.addTorque([0.0, 0.0, yaw_tau], True)
