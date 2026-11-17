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

import json
import math
import random
from typing import Optional, Tuple


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

    def __init__(self, robot_node, airframe: Optional[dict] = None) -> None:
        """`airframe` (optional) overrides the Mavic constants for another
        URDF quadcopter: {"k_thrust", "k_torque", "props": {"fl", "fr",
        "rl", "rr": [x, y, z]}} -- force anchors in the base_link frame,
        calibrated the same way as above (k_thrust = m g / (4
        K_VERTICAL_THRUST^2), m = the WHOLE robot's mass). Omitted keys
        keep the Mavic values, so a robot that declares nothing flies
        exactly as before. See airframe_from_custom_data().

        ⚠️ Centre the anchors on the WHOLE robot's CoM (every link's mass,
        gimbal included), not on the link origin -- the same correction the
        Mavic constants carry. Measured on a PX4 x500 (2026-09-23): a 2.6 mm
        gap (a 54 g gimbal chain forward of a 2.1 kg frame) made it creep
        0.35 m forward on every landing, because the attitude trim that
        balances the gap works through rotor-speed DIFFERENCES whose
        authority scales with rotor speed, and a descent cuts rotor speed.
        Anchors shifted onto the CoM: 0.013 m."""
        self.robot = robot_node
        self.props = (_PROP_FL, _PROP_FR, _PROP_RL, _PROP_RR)
        # Optional realism terms (2026-09-25), each OFF unless the airframe
        # declares it -- the Mavic declares none and flies exactly as before:
        #   motor_tau_s                   first-order rotor spin-up lag
        #   ground_effect_rotor_radius_m  per-rotor ground effect (Cheeseman-
        #                                 Bennett), ground plane at ground_z
        #   drag_coeff                    quadratic drag, N per (m/s)^2, on the
        #                                 velocity RELATIVE TO THE AIR
        #   wind {mean: [x,y,z] m/s, gust_sigma m/s, tau_s, seed,
        #         torque_sigma N*m}       seeded Ornstein-Uhlenbeck wind, felt
        #                                 through the drag, plus a small seeded
        #                                 turbulence torque. MODELLED, not
        #                                 measured air: say so wherever shown.
        self.motor_tau = None
        self.ge_radius = None
        self.ground_z = 0.0
        self.drag = 0.0
        self.wind = None
        self._w = [0.0, 0.0, 0.0, 0.0]
        self._last = (0.0, 0.0, 0.0, 0.0)
        if airframe:
            self.k_thrust = float(airframe.get("k_thrust", self.k_thrust))
            self.k_torque = float(airframe.get("k_torque", self.k_torque))
            p = airframe.get("props") or {}
            self.props = tuple(tuple(float(c) for c in p.get(k, d))
                               for k, d in zip(("fl", "fr", "rl", "rr"), self.props))
            if airframe.get("motor_tau_s"):
                self.motor_tau = float(airframe["motor_tau_s"])
            if airframe.get("ground_effect_rotor_radius_m"):
                self.ge_radius = float(airframe["ground_effect_rotor_radius_m"])
            self.ground_z = float(airframe.get("ground_z", 0.0))
            self.drag = float(airframe.get("drag_coeff", 0.0))
            w = airframe.get("wind")
            if isinstance(w, dict):
                self.wind = {
                    "mean": [float(c) for c in (w.get("mean") or [0.0, 0.0, 0.0])][:3],
                    "sigma": float(w.get("gust_sigma", 0.0)),
                    "tau": max(1e-3, float(w.get("tau_s", 2.0))),
                    "tq_sigma": float(w.get("torque_sigma", 0.0)),
                    "rng": random.Random(int(w.get("seed", 0))),
                }
                self.wind["gust"] = [0.0, 0.0, 0.0]
                self.wind["tq"] = [0.0, 0.0, 0.0]

    def rotor_speed(self) -> float:
        """Mean |rotor speed| (rad/s) the thrust was computed from last tick."""
        return sum(abs(w) for w in self._last) / 4.0

    def idle(self) -> None:
        """Motors off (on the ground): the lagged rotor speeds spin down to 0,
        so the next takeoff spools up from rest, not from the last hover."""
        self._w = [0.0, 0.0, 0.0, 0.0]
        self._last = (0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def ground_effect(radius: float, height: float) -> float:
        """Thrust ratio in / out of ground effect, 1 / (1 - (R / 4z)^2).
        Capped at R / 4z = 0.5 (x1.33): the formula diverges as z -> 0 and
        the cap is where published hover measurements flatten out."""
        ratio = min(0.5, radius / (4.0 * max(height, 1e-3)))
        return 1.0 / (1.0 - ratio * ratio)

    def _ou(self, x: float, mean: float, sigma: float, tau: float, dt: float) -> float:
        """One Ornstein-Uhlenbeck step: mean-reverting, stationary std `sigma`."""
        rng = self.wind["rng"]
        return (x + (mean - x) * dt / tau
                + sigma * math.sqrt(2.0 * dt / tau) * rng.gauss(0.0, 1.0))

    def step(self, fl: float, fr: float, rl: float, rr: float,
             dt: Optional[float] = None) -> None:
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
        if self.motor_tau and dt:
            # Rotors cannot change speed instantly: lag the commanded speed.
            a = 1.0 - math.exp(-dt / self.motor_tau)
            self._w = [w + (c - w) * a for w, c in zip(self._w, (fl, fr, rl, rr))]
            fl, fr, rl, rr = self._w
        self._last = (fl, fr, rl, rr)
        fl2 = fl * abs(fl)
        fr2 = fr * abs(fr)
        rl2 = rl * abs(rl)
        rr2 = rr * abs(rr)

        # Per-propeller lift. addForceWithOffset(force, offset, relative=True)
        # applies the force in body coordinates at the body-relative offset.
        p_fl, p_fr, p_rl, p_rr = self.props
        ge = (1.0, 1.0, 1.0, 1.0)
        if self.ge_radius:
            pos, rot = self.robot.getPosition(), self.robot.getOrientation()
            ge = tuple(self.ground_effect(
                self.ge_radius,
                pos[2] + rot[6] * p[0] + rot[7] * p[1] + rot[8] * p[2] - self.ground_z)
                for p in self.props)
        self.robot.addForceWithOffset([0.0, 0.0, self.k_thrust * fl2 * ge[0]], list(p_fl), True)
        self.robot.addForceWithOffset([0.0, 0.0, self.k_thrust * fr2 * ge[1]], list(p_fr), True)
        self.robot.addForceWithOffset([0.0, 0.0, self.k_thrust * rl2 * ge[2]], list(p_rl), True)
        self.robot.addForceWithOffset([0.0, 0.0, self.k_thrust * rr2 * ge[3]], list(p_rr), True)

        # Yaw torque from diagonal-pair asymmetry. The bridge already
        # encoded the yaw command into pair magnitudes (FR + RL bigger
        # for positive yaw, FL + RR bigger for negative yaw); we read
        # the same magnitudes here so the sign comes out right without
        # tracking spin direction separately.
        yaw_tau = self.k_torque * ((fr2 + rl2) - (fl2 + rr2))
        torque = [0.0, 0.0, yaw_tau]

        if self.wind and dt:
            wd = self.wind
            wd["gust"] = [self._ou(g, 0.0, wd["sigma"], wd["tau"], dt) for g in wd["gust"]]
            wd["tq"] = [self._ou(t, 0.0, wd["tq_sigma"], wd["tau"] / 4.0, dt) for t in wd["tq"]]
            # Turbulence rocks the airframe in roll and pitch; its yaw share is
            # small (a quad's yaw is set by rotor drag, which gusts barely move).
            tq = (wd["tq"][0], wd["tq"][1], 0.05 * wd["tq"][2])
            torque = [a + b for a, b in zip(torque, tq)]
        if self.drag:
            v = self.robot.getVelocity()[:3]
            air = [0.0, 0.0, 0.0]
            if self.wind:
                air = [m + g for m, g in zip(self.wind["mean"], self.wind["gust"])]
            rel = [vi - ai for vi, ai in zip(v, air)]
            speed = math.sqrt(sum(r * r for r in rel))
            self.robot.addForce([-self.drag * speed * r for r in rel], False)
        self.robot.addTorque(torque, True)


def airframe_from_custom_data(custom_data: str) -> Optional[dict]:
    """The `rotor_dynamics` object from a robot's customData JSON, or None.

    A world flies a different URDF quadcopter through this bridge by
    declaring its airframe on the robot, e.g.
      customData "{\"rotor_dynamics\": {\"k_thrust\": 0.00111, ...}}"
    (URDFRobot passes customData through to the expanded Robot). Anything
    that is not a JSON object carrying that key -- including the empty
    string every existing world has -- returns None: the Mavic constants.
    """
    try:
        data = json.loads(custom_data or "")
    except ValueError:
        return None
    rd = data.get("rotor_dynamics") if isinstance(data, dict) else None
    return rd if isinstance(rd, dict) else None
