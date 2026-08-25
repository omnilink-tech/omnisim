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

"""Aerodynamic forces for a fixed-wing airframe, as a component build-up.

OmniSim has no aerodynamics. Its rigid-body solver knows about contact, joints
and gravity; ``Fluid`` and ``ImmersionProperties`` went out with the ODE
deletion and now hard-ERROR at parse. So a wing is a brick unless something
puts a force on it, and this module is that something.

Why a build-up rather than stability derivatives
------------------------------------------------
A stability-derivative model states the whole aircraft's moment coefficients as
numbers fitted about one trim point. A build-up instead computes a force on
each surface and applies it at that surface's own location, letting the moments
fall out of the geometry. It costs more arithmetic and buys three things that
matter for a hardware-in-the-loop rig:

* Pitch, roll and yaw damping appear for free. Each surface sees ``v + omega x r``,
  so a rotating airframe meets its own tail. Without this an aircraft oscillates
  forever and every autopilot tuned against it is tuned against a fiction.
* Control coupling is real. Ailerons are modelled as opposite deflections on two
  wing panels, so roll comes from the force difference and adverse yaw comes
  from the drag difference -- both from one mechanism, neither hand-added.
* The model degrades honestly off the trim point. A derivative set fitted at
  cruise says nothing truthful about a 60-degree-alpha stall on approach; a
  build-up with a stall blend still says something defensible.

What this is not
----------------
It is not CFD, and it is not a certified flight model. Surfaces do not shade one
another, there is no propeller slipstream over the tail, no ground effect, no
compressibility, and no unsteady (rate-of-alpha) terms. Each of those is a real
effect that a real delivery aircraft exhibits. The claim here is narrower and
worth stating plainly: this produces a dynamically plausible airframe whose
trim, stability and control response are of the right sign and roughly the right
magnitude -- enough to exercise flight software, not enough to certify it.

Frames
------
Everything is body frame FLU (+X nose, +Y left, +Z up), matching what OmniSim
hands a Solid. See :mod:`omnisim_hil.vec3`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .vec3 import (
    Vec3,
    ZERO,
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    add,
    clamp,
    cross,
    dot,
    norm,
    normalize,
    project_out,
    scale,
    sub,
)

#: One applied wrench: a force in body frame, at an offset in body frame.
#: This is exactly the shape ``Supervisor.addForceWithOffset(force, offset,
#: relative=True)`` consumes, so nothing has to be re-derived at the call site.
AppliedForce = Tuple[Vec3, Vec3]


def lift_coefficient(
    alpha: float,
    cl0: float,
    cl_alpha: float,
    stall_alpha: float,
    blend_rate: float = 50.0,
) -> float:
    """Lift coefficient with a stall blend (Beard & McLain, eq. 4.9-4.10).

    Below the stall angle this is the linear ``cl0 + cl_alpha * alpha``. Beyond
    it, a logistic blend hands over to a flat-plate model,
    ``2 sign(a) sin^2(a) cos(a)``, which is what a wing that has stopped being a
    wing actually does: it still generates force, just badly and with enormous
    drag.

    Modelling the stall at all is the point. An autopilot that has never seen
    lift stop increasing with alpha will happily command its way into a
    departure on a heavy-payload approach, and a linear-only model will report
    that manoeuvre as a success.
    """
    # exp() overflows for large arguments long before the blend saturates, so
    # clamp the exponent rather than the angle: clamping the angle would move
    # the stall point.
    def _sig(x: float) -> float:
        return math.exp(clamp(x, -60.0, 60.0))

    num = 1.0 + _sig(-blend_rate * (alpha - stall_alpha)) + _sig(blend_rate * (alpha + stall_alpha))
    den = (1.0 + _sig(-blend_rate * (alpha - stall_alpha))) * (
        1.0 + _sig(blend_rate * (alpha + stall_alpha))
    )
    sigma = num / den if den > 1e-30 else 1.0
    sigma = clamp(sigma, 0.0, 1.0)

    linear = cl0 + cl_alpha * alpha
    sign = 1.0 if alpha >= 0.0 else -1.0
    flat_plate = 2.0 * sign * math.sin(alpha) ** 2 * math.cos(alpha)
    return (1.0 - sigma) * linear + sigma * flat_plate


def drag_coefficient(
    cl_linear: float,
    cd0: float,
    aspect_ratio: float,
    oswald: float,
) -> float:
    """Parasitic plus induced drag, from the LINEAR lift coefficient.

    Taking ``cl_linear`` as an argument rather than recomputing it from alpha is
    what makes adverse yaw emerge. A deflected aileron changes its panel's lift,
    that lift change belongs in ``CL^2/(pi e AR)``, and the up-going panel
    therefore drags more than the down-going one -- which is the entire physical
    origin of adverse yaw. Recomputing the coefficient from alpha alone silently
    drops the control term and makes the two panels identical, so the aircraft
    rolls with no yaw at all and every turn is perfectly coordinated for free.

    The term stays LINEAR past the stall on purpose: the flat-plate branch of
    :func:`lift_coefficient` already carries the post-stall drag rise through
    its own geometry, and feeding the blended value back in double-counts it.
    """
    induced = (cl_linear * cl_linear) / (math.pi * max(oswald, 1e-3) * max(aspect_ratio, 1e-3))
    return cd0 + induced


@dataclass
class Surface:
    """One lifting surface: a wing panel, a stabiliser, a fin.

    ``position`` is the surface's aerodynamic centre as an offset from the
    aircraft's centre of mass, in body frame. That offset is what creates every
    moment this model produces, so it is the single most important number here:
    a tail at the wrong station gives an aircraft the wrong static margin and
    the autopilot tuned on it will be wrong in the same direction on the real
    airframe.
    """

    name: str
    position: Vec3
    area: float
    #: Unit vector along the chord, pointing forward. Usually +X.
    chord_axis: Vec3 = X_AXIS
    #: Unit vector along the span. +Y (left) for horizontal surfaces, +Z for fins.
    span_axis: Vec3 = Y_AXIS
    cl0: float = 0.0
    cl_alpha: float = 5.0
    cd0: float = 0.02
    aspect_ratio: float = 6.0
    oswald: float = 0.85
    stall_alpha_rad: float = 0.35
    blend_rate: float = 50.0
    #: Rigging angle: the surface is mounted at this angle to the body X axis.
    incidence_rad: float = 0.0
    #: Name of the control input that deflects this surface, if any.
    control: Optional[str] = None
    #: dCL per radian of control deflection.
    control_effectiveness: float = 0.0
    #: Sign applied to the control input. Ailerons use +1 on one panel and -1 on
    #: the other, which is what makes roll and adverse yaw both emerge from the
    #: same deflection rather than being added by hand.
    control_sign: float = 1.0

    def normal_axis(self) -> Vec3:
        """The surface's own "up", completing a right-handed chord/span triad."""
        return normalize(cross(self.chord_axis, self.span_axis), Z_AXIS)

    def force(self, v_air_body: Vec3, rho: float, control_value: float = 0.0) -> AppliedForce:
        """Force from this surface given the local airflow, in body frame.

        ``v_air_body`` is the velocity of this surface THROUGH THE AIR, already
        including any rotational contribution -- see :meth:`Airframe.forces`,
        which is what computes it.
        """
        span = normalize(self.span_axis, Y_AXIS)
        # Spanwise flow neither lifts nor (much) drags: strip it before doing
        # any angle arithmetic, or a sideslip inflates both the angle of attack
        # and the dynamic pressure on every horizontal surface at once.
        v_perp = project_out(v_air_body, span)
        speed = norm(v_perp)
        if speed < 0.5:
            # Below roughly taxi speed the angles are numerically meaningless
            # (atan2 of two near-zero components) and the forces are negligible
            # anyway. Returning zero here is what keeps a parked aircraft from
            # jittering on random near-singular angles.
            return (ZERO, self.position)

        chord = normalize(self.chord_axis, X_AXIS)
        normal = self.normal_axis()
        alpha = math.atan2(-dot(v_perp, normal), dot(v_perp, chord)) + self.incidence_rad

        cl = lift_coefficient(alpha, self.cl0, self.cl_alpha, self.stall_alpha_rad, self.blend_rate)
        cl_linear = self.cl0 + self.cl_alpha * alpha
        profile_penalty = 0.0
        if self.control is not None and self.control_effectiveness:
            deflection = control_value * self.control_sign
            delta_cl = self.control_effectiveness * deflection
            cl += delta_cl
            # Into the INDUCED term as well, not just the lift: the up-going
            # aileron panel must drag more than the down-going one.
            cl_linear += delta_cl
            # A deflected surface is also a dirtier surface. Symmetric in the
            # deflection sign, so it contributes no yaw of its own.
            profile_penalty = 0.02 * deflection * deflection

        cd = drag_coefficient(cl_linear, self.cd0, self.aspect_ratio, self.oswald) + profile_penalty

        q = 0.5 * rho * speed * speed * self.area
        drag_dir = scale(normalize(v_perp), -1.0)
        # Perpendicular to the local flow, in the chord/normal plane. Derived
        # rather than assumed: cross(v, span) points along +normal at zero alpha
        # for a forward-flying surface, which is the sign convention lift needs.
        lift_dir = normalize(cross(v_perp, span), normal)

        force = add(scale(lift_dir, q * cl), scale(drag_dir, q * cd))
        return (force, self.position)


@dataclass
class PropellerModel:
    """Thrust and torque from a propeller, WITH the inflow term.

    The inflow term is the whole reason this class exists rather than deferring
    to OmniSim's ``Propeller`` node. That node computes ``T = t1 * |omega| *
    omega`` and nothing else: its speed-of-advance term is pinned to zero at
    ``OmPropeller.cpp:229-234`` because the body point-velocity read it needed
    went out with ODE. The consequence is that an OmniSim propeller delivers
    static thrust at every airspeed -- an aircraft accelerates until drag alone
    stops it, and level-flight trim, climb rate and throttle-to-speed response
    are all wrong in the same optimistic direction.

    Here thrust falls with advance ratio ``J = V / (n D)`` and reaches zero at
    ``j_zero_thrust``, which is what lets an aircraft actually settle at a cruise
    speed instead of running away from its own propeller.
    """

    diameter_m: float = 0.28
    #: Static thrust coefficient, T = ct * rho * n^2 * D^4 at J = 0.
    ct0: float = 0.10
    #: Advance ratio at which thrust reaches zero.
    j_zero_thrust: float = 0.6
    #: Torque coefficient, Q = cq * rho * n^2 * D^5.
    cq0: float = 0.008
    #: Where the thrust acts, body frame.
    position: Vec3 = ZERO
    #: Thrust direction, body frame. +X is a tractor or pusher on the nose axis.
    axis: Vec3 = X_AXIS
    #: +1 or -1. Reaction torque opposes rotation, so this sets which way the
    #: airframe is twisted and therefore which rudder trim the autopilot needs.
    spin_sign: float = 1.0
    max_rev_per_s: float = 200.0

    def thrust_and_torque(self, rev_per_s: float, axial_airspeed: float, rho: float) -> Tuple[float, float]:
        """Return ``(thrust_N, reaction_torque_Nm)``.

        ``axial_airspeed`` is the component of the aircraft's airspeed along the
        shaft. A negative value (flying backwards along the shaft) is clamped to
        zero rather than credited as extra thrust, which the linear CT model
        would otherwise do.
        """
        n = clamp(rev_per_s, 0.0, self.max_rev_per_s)
        if n < 1e-6:
            return (0.0, 0.0)
        j = max(axial_airspeed, 0.0) / (n * self.diameter_m)
        # Linear CT(J) falling to zero at j_zero_thrust. Clamped at zero: a
        # windmilling propeller does produce negative thrust, but modelling that
        # as a straight-line extrapolation of the cruise fit is worse than
        # admitting the model stops here.
        ct = self.ct0 * max(0.0, 1.0 - j / max(self.j_zero_thrust, 1e-6))
        thrust = ct * rho * n * n * (self.diameter_m ** 4)
        torque = self.cq0 * rho * n * n * (self.diameter_m ** 5)
        return (thrust, torque * self.spin_sign)

    def forces(self, rev_per_s: float, v_air_body: Vec3, rho: float) -> List[AppliedForce]:
        axis = normalize(self.axis, X_AXIS)
        thrust, torque = self.thrust_and_torque(rev_per_s, dot(v_air_body, axis), rho)
        out: List[AppliedForce] = [(scale(axis, thrust), self.position)]
        if abs(torque) > 1e-9:
            # A pure couple, expressed as two opposed forces on a 1 m arm so it
            # can travel through the same addForceWithOffset path as everything
            # else. Supervisor.addTorque exists, but routing every wrench
            # through one verb means one thing to verify rather than two.
            arm = normalize(cross(axis, Z_AXIS), Y_AXIS)
            if norm(cross(axis, Z_AXIS)) < 1e-6:
                arm = normalize(cross(axis, X_AXIS), Y_AXIS)
            couple = scale(normalize(cross(axis, arm)), torque * 0.5)
            out.append((couple, add(self.position, arm)))
            out.append((scale(couple, -1.0), sub(self.position, arm)))
        return out


@dataclass
class Airframe:
    """A complete aircraft: surfaces, propeller, and the body's own drag.

    :meth:`forces` is the whole interface. Give it the aircraft's velocity
    through the air and its rotation rate, both in body frame, and it returns
    the list of ``(force, offset)`` pairs to hand to
    ``Supervisor.addForceWithOffset``. Those forces must be re-applied EVERY
    tick: OmniSim's Newton runtime consumes its external-wrench dict at the end
    of each step (``omnisim_newton_runtime.py:8423``), which is deliberate
    ``dBodyAddForce`` semantics and not a bug to work around.
    """

    name: str = "aircraft"
    mass_kg: float = 2.5
    surfaces: List[Surface] = field(default_factory=list)
    propeller: Optional[PropellerModel] = None
    #: Parasitic drag area (Cd * A) of everything that is not a lifting
    #: surface: fuselage, gear, payload pod. Acts at the centre of mass.
    fuselage_drag_area: float = 0.02
    #: Reference geometry, reported for documentation and used by nothing here.
    wing_span_m: float = 1.4
    wing_chord_m: float = 0.22

    def wing_area(self) -> float:
        return sum(s.area for s in self.surfaces if abs(dot(normalize(s.span_axis, Y_AXIS), Y_AXIS)) > 0.7)

    def forces(
        self,
        v_air_body: Vec3,
        omega_body: Vec3,
        rho: float,
        controls: Optional[Dict[str, float]] = None,
        throttle_rev_per_s: float = 0.0,
    ) -> List[AppliedForce]:
        """Every aerodynamic and propulsive wrench, in body frame.

        Gravity is NOT included: the simulator already applies it, and adding it
        here would double it. That is the single easiest way to get a plausible
        looking aircraft that sinks at exactly 2g, so it is worth stating.
        """
        controls = controls or {}
        out: List[AppliedForce] = []

        for surface in self.surfaces:
            # The local flow at a surface differs from the aircraft's flow by
            # the rotational term. This is what damps pitch, roll and yaw, and
            # leaving it out is the difference between an airframe that settles
            # and one that rings.
            v_local = add(v_air_body, cross(omega_body, surface.position))
            value = controls.get(surface.control, 0.0) if surface.control else 0.0
            force, offset = surface.force(v_local, rho, value)
            if norm(force) > 1e-9:
                out.append((force, offset))

        speed = norm(v_air_body)
        if speed > 0.5 and self.fuselage_drag_area > 0.0:
            drag = 0.5 * rho * speed * speed * self.fuselage_drag_area
            out.append((scale(normalize(v_air_body), -drag), ZERO))

        if self.propeller is not None and throttle_rev_per_s > 0.0:
            out.extend(self.propeller.forces(throttle_rev_per_s, v_air_body, rho))

        return out

    def trim_airspeed(self, rho: float = 1.225, g: float = 9.80665) -> float:
        """Airspeed at which the wing carries the aircraft's weight at CL0.

        A rough but genuinely useful number: it is the speed the aircraft wants
        to fly at hands-off, and if an autopilot's cruise setpoint is far from
        it the aircraft will spend the flight fighting itself.
        """
        area = self.wing_area()
        if area <= 0.0:
            return float("nan")
        # Include the RIGGING ANGLE, not just cl0. A wing mounted at a few
        # degrees of incidence is already making lift when the fuselage is
        # level, and omitting that overstates the trim speed by the square root
        # of the ratio -- 2 degrees of incidence on this airframe is a 65 per
        # cent lift error, which is enough to put a cruise setpoint below the
        # real stall speed.
        cl = sum(
            (s.cl0 + s.cl_alpha * s.incidence_rad) * s.area
            for s in self.surfaces
            if abs(dot(normalize(s.span_axis, Y_AXIS), Y_AXIS)) > 0.7
        )
        cl = cl / area if area else 0.0
        if cl <= 1e-6:
            return float("nan")
        return math.sqrt(2.0 * self.mass_kg * g / (rho * area * cl))


def delivery_aircraft(mass_kg: float = 2.5, payload_kg: float = 0.0) -> Airframe:
    """A small fixed-wing delivery aircraft: 1.4 m span, pusher propeller.

    The numbers are representative of a light electric UAV in this class rather
    than measured from any real aircraft, and no part of this is derived from
    any manufacturer's data. They are chosen so the aircraft trims near 14 m/s,
    which puts the whole flight envelope inside the range where the
    incompressible assumptions above hold.

    ``payload_kg`` adds mass without adding wing, which is the trade a delivery
    aircraft actually makes: every gram of parcel raises the stall speed.
    """
    span = 1.4
    chord = 0.22
    wing_area = span * chord
    panel_area = wing_area / 2.0
    panel_y = span / 4.0
    aspect = (span * span) / wing_area

    surfaces = [
        Surface(
            name="wing_left",
            position=(0.0, panel_y, 0.02),
            area=panel_area,
            span_axis=Y_AXIS,
            cl0=0.28,
            cl_alpha=5.2,
            cd0=0.016,
            aspect_ratio=aspect,
            oswald=0.85,
            stall_alpha_rad=0.30,
            incidence_rad=0.035,
            control="aileron",
            control_effectiveness=0.9,
            control_sign=1.0,
        ),
        Surface(
            name="wing_right",
            position=(0.0, -panel_y, 0.02),
            area=panel_area,
            span_axis=Y_AXIS,
            cl0=0.28,
            cl_alpha=5.2,
            cd0=0.016,
            aspect_ratio=aspect,
            oswald=0.85,
            stall_alpha_rad=0.30,
            incidence_rad=0.035,
            control="aileron",
            control_effectiveness=0.9,
            control_sign=-1.0,
        ),
        Surface(
            name="htail",
            # Behind the centre of mass: this negative station is the entire
            # source of pitch stability, and its magnitude is the tail moment
            # arm every longitudinal mode depends on.
            position=(-0.55, 0.0, 0.03),
            area=0.055,
            span_axis=Y_AXIS,
            cl0=0.0,
            cl_alpha=3.9,
            cd0=0.012,
            aspect_ratio=4.0,
            stall_alpha_rad=0.35,
            control="elevator",
            control_effectiveness=1.6,
        ),
        Surface(
            name="vtail",
            position=(-0.55, 0.0, 0.08),
            area=0.035,
            span_axis=Z_AXIS,
            cl0=0.0,
            cl_alpha=3.5,
            cd0=0.012,
            aspect_ratio=2.5,
            stall_alpha_rad=0.40,
            control="rudder",
            control_effectiveness=1.4,
        ),
    ]

    return Airframe(
        name="delivery_aircraft",
        mass_kg=mass_kg + payload_kg,
        surfaces=surfaces,
        propeller=PropellerModel(
            diameter_m=0.25,
            ct0=0.105,
            j_zero_thrust=0.62,
            cq0=0.0075,
            position=(-0.62, 0.0, 0.03),
            axis=X_AXIS,
            spin_sign=1.0,
            max_rev_per_s=180.0,
        ),
        fuselage_drag_area=0.018,
        wing_span_m=span,
        wing_chord_m=chord,
    )
