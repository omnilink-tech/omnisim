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


# Propeller anchor offsets in the base_link frame (must match the URDF
# joint origins exactly).
_PROP_FL: Tuple[float, float, float] = (0.054854, 0.151294, -0.003)
_PROP_FR: Tuple[float, float, float] = (0.054854, -0.151294, -0.003)
_PROP_RL: Tuple[float, float, float] = (-0.177179, 0.127453, -0.032)
_PROP_RR: Tuple[float, float, float] = (-0.177179, -0.127453, -0.032)


class RotorDynamics:
    """Supervisor-driven thrust/torque applier for a URDF quadcopter.

    Construct once at controller startup with the supervisor handle and
    the robot node (i.e. `supervisor.getSelf()`). Call `step(...)` once
    per control tick AFTER the bridge has issued its `setVelocity()`
    calls but BEFORE the next `supervisor.step()`.
    """

    # Lift coefficient per Webots Propeller convention: F = K * ω².
    # PROTO had thrustConstants ±0.00026 — sign was paired with prop
    # spin direction; here we always treat lift as a scalar magnitude
    # and apply it along +z body axis.
    k_thrust: float = 0.00026

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
        fl2 = fl * fl
        fr2 = fr * fr
        rl2 = rl * rl
        rr2 = rr * rr

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
