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

"""Physics validation for the aerodynamic model.

These assert PHYSICAL PROPERTIES, not recorded numbers. A golden-trajectory test
would pin whatever the model does today, including its bugs; asserting that an
aircraft is pitch-stable, that a stall reduces lift, and that a propeller loses
thrust with airspeed will fail if the model stops being an aircraft, which is
the only failure worth catching here.

Runs offline: no simulator, no engine, no GPU.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from omnisim_hil import atmosphere as atm
from omnisim_hil.aero import (
    Airframe,
    PropellerModel,
    Surface,
    delivery_aircraft,
    drag_coefficient,
    lift_coefficient,
)
from omnisim_hil.vec3 import Y_AXIS, Z_AXIS, cross, norm

RHO = 1.225


def flow(speed: float, alpha: float = 0.0, beta: float = 0.0):
    """Body-frame velocity through the air for a given alpha and sideslip.

    Inverts the model's own angle definitions: alpha = atan2(-w, u) and
    beta = atan2(-v, u) in FLU, so a positive beta means the aircraft is
    slipping to its right.
    """
    u = speed * math.cos(alpha) * math.cos(beta)
    v = -speed * math.sin(beta)
    w = -speed * math.sin(alpha)
    return (u, v, w)


def total_force(applied):
    fx = sum(f[0] for f, _ in applied)
    fy = sum(f[1] for f, _ in applied)
    fz = sum(f[2] for f, _ in applied)
    return (fx, fy, fz)


def total_moment(applied):
    """Sum of r x F about the centre of mass, body frame."""
    mx = my = mz = 0.0
    for force, offset in applied:
        m = cross(offset, force)
        mx += m[0]
        my += m[1]
        mz += m[2]
    return (mx, my, mz)


def nose_up_moment(applied) -> float:
    """Pitching moment, positive nose UP.

    In FLU a positive moment about +Y (left) rotates the top of the aircraft
    forward, which is nose DOWN -- so the conventional nose-up sense is the
    negative of the Y component. Getting this backwards silently inverts every
    stability conclusion, which is why it is one named function and not an
    inline sign.
    """
    return -total_moment(applied)[1]


def yaw_left_moment(applied) -> float:
    """Yawing moment, positive nose LEFT (about +Z up, right-hand rule)."""
    return total_moment(applied)[2]


def roll_right_moment(applied) -> float:
    """Rolling moment, positive right wing DOWN.

    A positive moment about +X (nose) carries +Y (left) toward +Z (up), lifting
    the left wing and dropping the right one.
    """
    return total_moment(applied)[0]


# --- coefficient behaviour ---------------------------------------------------


def test_lift_is_linear_below_stall():
    cl_a, cl_b = 0.3, 5.0
    for alpha in (0.0, 0.05, 0.10, 0.15):
        expected = cl_a + cl_b * alpha
        got = lift_coefficient(alpha, cl_a, cl_b, stall_alpha=0.35)
        assert abs(got - expected) < 0.02, (alpha, got, expected)


def test_lift_peaks_and_falls_past_the_stall():
    stall = 0.30
    curve = [(a, lift_coefficient(a, 0.28, 5.2, stall)) for a in
             [i * 0.01 for i in range(0, 121)]]
    peak_alpha, peak_cl = max(curve, key=lambda p: p[1])

    assert stall - 0.12 < peak_alpha < stall + 0.12, (
        "lift should peak near the stall angle, peaked at %.3f rad" % peak_alpha)
    deep = lift_coefficient(0.9, 0.28, 5.2, stall)
    assert deep < peak_cl, "lift must fall past the stall: %.3f vs peak %.3f" % (deep, peak_cl)
    assert deep > 0.0, "a stalled wing still makes some lift, got %.3f" % deep


def test_lift_curve_is_odd_about_zero_for_a_symmetric_section():
    for alpha in (0.1, 0.4, 0.8):
        up = lift_coefficient(alpha, 0.0, 5.0, 0.3)
        down = lift_coefficient(-alpha, 0.0, 5.0, 0.3)
        assert abs(up + down) < 1e-6, (alpha, up, down)


def test_lift_coefficient_is_finite_at_extreme_angles():
    # The blend uses exp(); an unclamped exponent overflows long before the
    # angle stops being meaningful.
    for alpha in (-3.2, -1.6, 1.6, 3.2, 10.0):
        value = lift_coefficient(alpha, 0.28, 5.2, 0.30)
        assert math.isfinite(value), alpha


def test_drag_rises_with_lift():
    low = drag_coefficient(0.0, 0.02, 6.0, 0.85)
    high = drag_coefficient(1.25, 0.02, 6.0, 0.85)
    assert high > low
    assert abs(low - 0.02) < 1e-9, "at zero lift only parasitic drag remains"


def test_induced_drag_is_quadratic_in_lift():
    """Doubling the lift coefficient must quadruple the induced drag."""
    single = drag_coefficient(1.0, 0.0, 6.0, 0.85)
    double = drag_coefficient(2.0, 0.0, 6.0, 0.85)
    assert abs(double / single - 4.0) < 1e-9, double / single


# --- whole-airframe statics --------------------------------------------------


def test_a_parked_aircraft_feels_nothing():
    craft = delivery_aircraft()
    applied = craft.forces((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), RHO)
    assert norm(total_force(applied)) < 1e-9


def test_lift_opposes_gravity_in_cruise():
    craft = delivery_aircraft()
    applied = craft.forces(flow(16.0, alpha=0.03), (0.0, 0.0, 0.0), RHO)
    fz = total_force(applied)[2]
    weight = craft.mass_kg * 9.80665
    assert fz > 0.0, "the wing must push up"
    # Within a factor of two of weight at a plausible cruise: this is a
    # sanity band, not a trim solution.
    assert 0.5 * weight < fz < 2.0 * weight, (fz, weight)


def test_drag_opposes_motion():
    craft = delivery_aircraft()
    applied = craft.forces(flow(16.0, alpha=0.03), (0.0, 0.0, 0.0), RHO)
    assert total_force(applied)[0] < 0.0, "with no thrust, net X force must be rearward"


def test_trim_airspeed_carries_the_weight():
    craft = delivery_aircraft()
    v_trim = craft.trim_airspeed(RHO)
    assert math.isfinite(v_trim) and 8.0 < v_trim < 25.0, v_trim
    applied = craft.forces(flow(v_trim, alpha=0.0), (0.0, 0.0, 0.0), RHO)
    lift = total_force(applied)[2]
    weight = craft.mass_kg * 9.80665
    assert abs(lift - weight) < 0.35 * weight, (
        "at the trim airspeed lift should be near weight: %.2f N vs %.2f N" % (lift, weight))


def test_payload_raises_the_stall_speed():
    light = delivery_aircraft(payload_kg=0.0).trim_airspeed(RHO)
    heavy = delivery_aircraft(payload_kg=1.5).trim_airspeed(RHO)
    assert heavy > light, (light, heavy)


# --- static stability: the properties that make it an aircraft ---------------


def test_pitch_is_statically_stable():
    """A nose-up disturbance must produce a nose-down moment.

    This is Cm_alpha < 0, the single property that separates an aircraft from a
    dart thrown backwards. It comes entirely from the horizontal tail sitting
    behind the centre of mass.
    """
    craft = delivery_aircraft()
    zero = (0.0, 0.0, 0.0)
    m_low = nose_up_moment(craft.forces(flow(16.0, alpha=0.00), zero, RHO))
    m_high = nose_up_moment(craft.forces(flow(16.0, alpha=0.10), zero, RHO))
    assert m_high < m_low, (
        "increasing alpha must reduce the nose-up moment: %.4f -> %.4f" % (m_low, m_high))


def test_yaw_is_weathercock_stable():
    """Sideslip must swing the nose into the relative wind."""
    craft = delivery_aircraft()
    zero = (0.0, 0.0, 0.0)
    m_zero = yaw_left_moment(craft.forces(flow(16.0, beta=0.0), zero, RHO))
    m_slip = yaw_left_moment(craft.forces(flow(16.0, beta=0.15), zero, RHO))
    # Positive beta is a slip to the right, so the nose must yaw right
    # (negative yaw-left moment) to point back into the wind.
    assert m_slip < m_zero, (
        "a right sideslip must yaw the nose right: %.4f -> %.4f" % (m_zero, m_slip))


def test_pitch_rate_is_damped():
    """A pitching airframe must meet its own tail and be slowed by it."""
    craft = delivery_aircraft()
    still = nose_up_moment(craft.forces(flow(16.0), (0.0, 0.0, 0.0), RHO))
    # Positive omega_y in FLU is a nose-down rotation rate.
    pitching = nose_up_moment(craft.forces(flow(16.0), (0.0, 1.0, 0.0), RHO))
    assert pitching > still, (
        "a nose-down rate must generate a nose-up (opposing) moment: %.4f -> %.4f"
        % (still, pitching))


def test_yaw_rate_is_damped():
    craft = delivery_aircraft()
    still = yaw_left_moment(craft.forces(flow(16.0), (0.0, 0.0, 0.0), RHO))
    yawing = yaw_left_moment(craft.forces(flow(16.0), (0.0, 0.0, 1.0), RHO))
    assert yawing < still, (
        "a nose-left yaw rate must generate an opposing moment: %.4f -> %.4f" % (still, yawing))


# --- controls ----------------------------------------------------------------


def test_elevator_commands_pitch():
    craft = delivery_aircraft()
    zero = (0.0, 0.0, 0.0)
    neutral = nose_up_moment(craft.forces(flow(16.0), zero, RHO, {"elevator": 0.0}))
    deflected = nose_up_moment(craft.forces(flow(16.0), zero, RHO, {"elevator": 0.2}))
    assert abs(deflected - neutral) > 1e-3, "the elevator must do something"


def test_ailerons_roll_the_aircraft():
    craft = delivery_aircraft()
    zero = (0.0, 0.0, 0.0)
    neutral = roll_right_moment(craft.forces(flow(16.0), zero, RHO, {"aileron": 0.0}))
    deflected = roll_right_moment(craft.forces(flow(16.0), zero, RHO, {"aileron": 0.3}))
    assert abs(deflected - neutral) > 1e-2, (neutral, deflected)


def test_ailerons_produce_adverse_yaw():
    """The rolling wing must also yaw the aircraft the WRONG way.

    Adverse yaw is not a defect to be tuned out of the model: it is why a
    coordinated turn needs rudder, and an autopilot that never meets it in
    simulation will fly uncoordinated on the real aircraft. It emerges here
    from the two panels having different drag, with nothing added by hand.
    """
    craft = delivery_aircraft()
    zero = (0.0, 0.0, 0.0)
    applied = craft.forces(flow(16.0), zero, RHO, {"aileron": 0.35})
    roll = roll_right_moment(applied)
    yaw = yaw_left_moment(applied)
    assert abs(yaw) > 1e-4, "expected a yawing moment from asymmetric drag, got %.6f" % yaw
    assert roll > 0.0, "positive aileron should roll right wing down, got %.4f" % roll
    # ADVERSE, not proverse: rolling right must yaw the nose LEFT, away from the
    # turn. Both quantities are positive in these sign conventions, so the
    # product is the test -- a model that produced proverse yaw would give a
    # negative product and fail here rather than passing on a bare magnitude.
    assert roll * yaw > 0.0, (
        "yaw must oppose the roll direction (adverse): roll=%.4f yaw=%.4f" % (roll, yaw))


def test_controls_scale_with_dynamic_pressure():
    """Control authority must fall off at low speed -- that is what makes a
    slow approach hard, and an autopilot with fixed gains must feel it."""
    craft = delivery_aircraft()
    zero = (0.0, 0.0, 0.0)
    slow = abs(nose_up_moment(craft.forces(flow(8.0), zero, RHO, {"elevator": 0.2}))
               - nose_up_moment(craft.forces(flow(8.0), zero, RHO, {"elevator": 0.0})))
    fast = abs(nose_up_moment(craft.forces(flow(24.0), zero, RHO, {"elevator": 0.2}))
               - nose_up_moment(craft.forces(flow(24.0), zero, RHO, {"elevator": 0.0})))
    assert fast > 3.0 * slow, (
        "authority should rise roughly with V^2: %.4f at 8 m/s vs %.4f at 24 m/s" % (slow, fast))


# --- propulsion --------------------------------------------------------------


def test_propeller_thrust_falls_with_airspeed():
    """The term OmniSim's own Propeller node is missing.

    OmPropeller.cpp:229-234 pins the speed of advance to zero, so its thrust is
    identical at 0 and 30 m/s. This model must not reproduce that.
    """
    prop = PropellerModel(diameter_m=0.25, ct0=0.105, j_zero_thrust=0.62, max_rev_per_s=180.0)
    n = 120.0
    static, _ = prop.thrust_and_torque(n, 0.0, RHO)
    cruise, _ = prop.thrust_and_torque(n, 12.0, RHO)
    assert static > cruise > 0.0, (static, cruise)


def test_propeller_thrust_reaches_zero_at_the_zero_thrust_advance_ratio():
    prop = PropellerModel(diameter_m=0.25, ct0=0.105, j_zero_thrust=0.6, max_rev_per_s=400.0)
    n = 100.0
    v_zero = 0.6 * n * 0.25
    thrust, _ = prop.thrust_and_torque(n, v_zero, RHO)
    assert abs(thrust) < 1e-9, thrust
    beyond, _ = prop.thrust_and_torque(n, v_zero * 1.5, RHO)
    assert beyond >= 0.0, "thrust is clamped at zero rather than extrapolated negative"


def test_propeller_thrust_rises_with_rpm():
    prop = PropellerModel(max_rev_per_s=400.0)
    low, _ = prop.thrust_and_torque(50.0, 0.0, RHO)
    high, _ = prop.thrust_and_torque(150.0, 0.0, RHO)
    assert high > low > 0.0
    # T ~ n^2, so tripling n should give roughly 9x.
    assert 6.0 < high / low < 12.0, high / low


def test_propeller_reaction_torque_is_a_pure_couple():
    """The couple must add no net force, or the aircraft gains free thrust."""
    prop = PropellerModel(position=(-0.6, 0.0, 0.0), max_rev_per_s=400.0)
    applied = prop.forces(120.0, (0.0, 0.0, 0.0), RHO)
    thrust_only, _ = prop.thrust_and_torque(120.0, 0.0, RHO)
    fx, fy, fz = total_force(applied)
    assert abs(fx - thrust_only) < 1e-6, (fx, thrust_only)
    assert abs(fy) < 1e-6 and abs(fz) < 1e-6, (fy, fz)


def test_propeller_idle_produces_nothing():
    prop = PropellerModel()
    assert prop.thrust_and_torque(0.0, 10.0, RHO) == (0.0, 0.0)


# --- atmosphere --------------------------------------------------------------


def test_sea_level_matches_isa():
    assert abs(atm.density(0.0) - 1.225) < 0.001
    assert abs(atm.pressure_pa(0.0) - 101325.0) < 1.0
    assert abs(atm.temperature_k(0.0) - 288.15) < 1e-6


def test_density_falls_with_altitude():
    assert atm.density(0.0) > atm.density(1000.0) > atm.density(5000.0) > atm.density(12000.0)


def test_pressure_altitude_round_trips():
    for h in (0.0, 250.0, 1500.0, 8000.0, 15000.0):
        assert abs(atm.pressure_altitude_m(atm.pressure_pa(h)) - h) < 1.0, h


def test_hot_day_thins_the_air():
    """ISA+20 must reduce density -- the reason a heavy lift fails in summer."""
    standard = atm.Atmosphere()
    hot = atm.Atmosphere(temperature_offset_k=20.0)
    assert hot.density(0.0) < standard.density(0.0)
    assert hot.density_altitude_m(0.0) > 0.0


def test_turbulence_is_reproducible_and_bounded():
    a = atm.Wind(steady=(3.0, 0.0, 0.0), turbulence_sigma_m_s=2.0, seed=7)
    b = atm.Wind(steady=(3.0, 0.0, 0.0), turbulence_sigma_m_s=2.0, seed=7)
    sa = [a.sample(0.01, 15.0) for _ in range(400)]
    sb = [b.sample(0.01, 15.0) for _ in range(400)]
    assert sa == sb, "same seed must replay identically or a regression cannot be bisected"
    spread = max(abs(s[0] - 3.0) for s in sa)
    assert spread < 12.0, "gusts should stay within a few sigma, saw %.2f" % spread


def test_turbulence_intensity_does_not_depend_on_step_size():
    """Stationary variance must be sigma^2 regardless of dt.

    A naive first-order filter makes turbulence intensity an artefact of the
    controller rate, so the same flight software would meet different weather
    at 100 Hz and 250 Hz.
    """
    def rms(dt, n):
        w = atm.Wind(turbulence_sigma_m_s=2.0, seed=11)
        # Discard the transient while the process reaches its stationary
        # distribution from a zero initial state.
        for _ in range(n // 4):
            w.sample(dt, 15.0)
        xs = [w.sample(dt, 15.0)[0] for _ in range(n)]
        return math.sqrt(sum(x * x for x in xs) / len(xs))

    fine = rms(0.004, 20000)
    coarse = rms(0.02, 20000)
    assert abs(fine - coarse) < 0.5, (fine, coarse)


def test_dynamic_pressure_matches_the_definition():
    v, h = 18.0, 100.0
    expected = 0.5 * atm.density(h) * v * v
    assert abs(atm.dynamic_pressure_pa(v, h) - expected) < 1e-9


def test_wind_shear_falls_off_near_the_ground():
    assert atm.wind_shear_factor(2.0) < atm.wind_shear_factor(10.0) < atm.wind_shear_factor(60.0)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
