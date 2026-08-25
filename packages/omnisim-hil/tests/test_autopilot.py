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

"""Offline tests for the autopilot: no engine, no GPU, no socket.

Nothing here binds a port. The control laws are pure functions and the mission
logic is a pure state machine, which is the whole reason they were written that
way: a flight-software defect that only appears with a simulator attached is a
defect you find late, on a machine that is thermally limited and shared.

What these assert is the SHAPE of the design -- limits that must hold at every
input, a floor that must outrank the loop above it, an index that must never go
backwards, an angle that must wrap. They deliberately do not assert tuned gain
values, which would freeze a tuning rather than a contract.
"""

import json
import math
import os
import sys

import pytest

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PKG)
sys.path.insert(0, os.path.join(_PKG, "autopilot"))

from omnisim_autopilot import (  # noqa: E402
    ARMED,
    BANK_LIMIT_RAD,
    CHANNEL_SIGN_AILERON,
    CHANNEL_SIGN_ELEVATOR,
    CHANNEL_SIGN_RUDDER,
    CLIMB_LIMIT_M_S,
    LOITER,
    NAV,
    PITCH_LIMIT_RAD,
    RTB,
    STALL_FLOOR_M_S,
    WAIT_FOR_SENSORS,
    Autopilot,
    Integrator,
    Navigator,
    VehicleState,
    Waypoint,
    altitude_to_climb_rate,
    apply_stall_floor,
    bearing_to,
    default_mission_path,
    distance_to,
    euler_from_quaternion,
    heading_to_bank,
    load_mission,
    pitch_axis,
    roll_axis,
    wrap_pi,
    yaw_axis,
)


def wp(east, north, alt=45.0, label="WP", acceptance=30.0, rtb=False):
    return Waypoint(east, north, alt, label, acceptance, rtb)


def flying_state(**over):
    s = VehicleState()
    s.have_sensor = True
    s.have_attitude = True
    s.t = 10.0
    s.altitude = 45.0
    s.airspeed = 16.0
    s.true_airspeed = 16.0
    s.vn = 16.0
    s.ve = 0.0
    s.vd = 0.0
    for key, value in over.items():
        setattr(s, key, value)
    return s


# ==========================================================================
# angles
# ==========================================================================


def test_wrap_pi_folds_into_the_half_open_interval():
    assert wrap_pi(0.0) == pytest.approx(0.0)
    assert wrap_pi(math.pi) == pytest.approx(math.pi)
    assert wrap_pi(-math.pi) == pytest.approx(math.pi)
    assert wrap_pi(3.0 * math.pi) == pytest.approx(math.pi)
    assert wrap_pi(1.5 * math.pi) == pytest.approx(-0.5 * math.pi)
    for raw in (-20.0, -7.0, -0.3, 0.0, 0.3, 7.0, 20.0, 1e4):
        assert -math.pi < wrap_pi(raw) <= math.pi + 1e-12


def test_heading_error_wraps_the_short_way_across_the_antimeridian():
    # 179 deg to -179 deg is two degrees of turn, not 358. Getting this wrong
    # does not look like a bug: the aircraft simply rolls to its bank limit and
    # goes the long way round, which reads as a badly tuned heading loop.
    east_of = math.radians(179.0)
    west_of = math.radians(-179.0)
    assert wrap_pi(west_of - east_of) == pytest.approx(math.radians(2.0), abs=1e-9)
    assert wrap_pi(east_of - west_of) == pytest.approx(math.radians(-2.0), abs=1e-9)


def test_bearing_is_a_compass_angle_from_north():
    assert bearing_to(0.0, 0.0, 0.0, 100.0) == pytest.approx(0.0)                 # north
    assert bearing_to(0.0, 0.0, 100.0, 0.0) == pytest.approx(math.pi / 2)         # east
    assert bearing_to(0.0, 0.0, 0.0, -100.0) == pytest.approx(math.pi)            # south
    assert bearing_to(0.0, 0.0, -100.0, 0.0) == pytest.approx(-math.pi / 2)       # west
    assert bearing_to(0.0, 0.0, 100.0, 100.0) == pytest.approx(math.pi / 4)       # NE


def test_bearing_is_relative_to_the_aircraft_not_the_origin():
    assert bearing_to(50.0, 50.0, 50.0, 150.0) == pytest.approx(0.0)
    assert bearing_to(-120.0, 150.0, -120.0, 50.0) == pytest.approx(math.pi)


def test_bearing_error_stays_small_for_a_target_just_across_south():
    # Flying due south (yaw = pi) at a waypoint a hair to the west of south.
    # The raw difference is near 2*pi and only the wrap makes it a 1 degree
    # correction rather than a full reversal.
    yaw = math.pi
    target = bearing_to(0.0, 0.0, -1.75, -100.0)      # ~ -179 deg
    assert abs(math.degrees(wrap_pi(target - yaw))) < 2.0


def test_distance_is_horizontal():
    assert distance_to(0.0, 0.0, 3.0, 4.0) == pytest.approx(5.0)


# ==========================================================================
# limits
# ==========================================================================


def test_bank_command_never_exceeds_the_limit():
    for error_deg in (-720, -180, -95, -30, 0, 30, 95, 180, 720):
        bank = heading_to_bank(math.radians(error_deg))
        assert abs(bank) <= BANK_LIMIT_RAD + 1e-12
    assert heading_to_bank(math.radians(180)) == pytest.approx(BANK_LIMIT_RAD)
    assert heading_to_bank(math.radians(-180)) == pytest.approx(-BANK_LIMIT_RAD)


def test_bank_command_keeps_the_sign_of_the_error():
    assert heading_to_bank(0.2) > 0.0
    assert heading_to_bank(-0.2) < 0.0
    assert heading_to_bank(0.0) == pytest.approx(0.0)


def test_climb_command_never_exceeds_the_limit():
    for error in (-1e4, -100.0, -3.0, 0.0, 3.0, 100.0, 1e4):
        assert abs(altitude_to_climb_rate(error)) <= CLIMB_LIMIT_M_S + 1e-12
    assert altitude_to_climb_rate(1e4) == pytest.approx(CLIMB_LIMIT_M_S)
    assert altitude_to_climb_rate(-1e4) == pytest.approx(-CLIMB_LIMIT_M_S)


def test_inner_loops_saturate_inside_the_normalised_channel_range():
    for fn in (roll_axis, pitch_axis):
        for error in (-50.0, -1.0, 0.0, 1.0, 50.0):
            for rate in (-50.0, 0.0, 50.0):
                assert -1.0 <= fn(error, rate) <= 1.0
    for rate in (-50.0, 0.0, 50.0):
        for bank in (-1.0, 0.0, 1.0):
            assert -1.0 <= yaw_axis(rate, bank) <= 1.0


def test_rate_terms_oppose_the_rate():
    # Same angle error, more rate toward it -> less command. Losing this sign is
    # how an inner loop turns into an oscillator that the outer loops then get
    # blamed for.
    assert roll_axis(0.2, 1.0) < roll_axis(0.2, 0.0) < roll_axis(0.2, -1.0)
    assert pitch_axis(0.2, 1.0) < pitch_axis(0.2, 0.0) < pitch_axis(0.2, -1.0)
    assert yaw_axis(1.0, 0.0) < yaw_axis(0.0, 0.0) < yaw_axis(-1.0, 0.0)


# ==========================================================================
# the stall floor -- the limit that outranks the altitude loop
# ==========================================================================


def test_stall_floor_is_transparent_above_the_floor():
    for airspeed in (STALL_FLOOR_M_S, STALL_FLOOR_M_S + 0.01, 20.0, 40.0):
        for cmd in (-0.3, 0.0, 0.3):
            assert apply_stall_floor(cmd, airspeed) == cmd


def test_stall_floor_overrides_a_nose_up_altitude_command():
    # The case that matters: the altitude loop is demanding the maximum climb it
    # is allowed to, and the aircraft is below its airspeed floor. The floor
    # must win, and the resulting command must be nose DOWN.
    demanded = PITCH_LIMIT_RAD
    guarded = apply_stall_floor(demanded, STALL_FLOOR_M_S - 4.0)
    assert guarded < 0.0
    assert guarded < demanded


def test_stall_floor_bites_harder_the_slower_the_aircraft_is():
    a = apply_stall_floor(PITCH_LIMIT_RAD, STALL_FLOOR_M_S - 1.0)
    b = apply_stall_floor(PITCH_LIMIT_RAD, STALL_FLOOR_M_S - 5.0)
    assert b < a < 0.0


def test_stall_floor_is_one_sided_and_can_never_raise_a_command():
    # It is allowed to decline a climb. It is not allowed to fly the aircraft:
    # if it could raise a pitch command it would be a controller in its own
    # right, with no loop above it and no limit below it.
    for airspeed in (0.0, 5.0, 12.9, 13.0, 25.0):
        for cmd in (-PITCH_LIMIT_RAD, -0.2, 0.0, 0.2, PITCH_LIMIT_RAD):
            assert apply_stall_floor(cmd, airspeed) <= cmd + 1e-15


def test_a_slow_aircraft_is_commanded_nose_down_through_the_full_cascade():
    ap = Autopilot(Navigator([wp(0.0, 500.0, alt=100.0)]))
    slow = flying_state(altitude=40.0, airspeed=STALL_FLOOR_M_S - 5.0)
    ap.state = NAV                       # past the startup states
    ap.state_since = 0.0
    ap.tick(slow, 0.008)
    # 60 m below the commanded altitude, so the altitude loop is asking for its
    # maximum climb, and the aircraft is 5 m/s below the floor.
    assert ap.debug["climb_cmd"] == pytest.approx(CLIMB_LIMIT_M_S)
    assert ap.debug["pitch_cmd"] < 0.0
    assert ap.last_command["throttle"] == pytest.approx(1.0)


def test_pitch_command_never_exceeds_the_pitch_limit():
    ap = Autopilot(Navigator([wp(0.0, 500.0, alt=400.0)]))
    ap.state = NAV
    for altitude in (0.0, 45.0, 390.0, 800.0):
        for roll in (0.0, BANK_LIMIT_RAD, -BANK_LIMIT_RAD):
            s = flying_state(altitude=altitude, roll=roll, airspeed=20.0)
            ap.tick(s, 0.008)
            assert abs(ap.debug["pitch_cmd"]) <= PITCH_LIMIT_RAD + 1e-12


# ==========================================================================
# integrator
# ==========================================================================


def test_integrator_output_is_clamped_to_its_limit():
    i = Integrator(gain=0.02, limit=0.12)
    for _ in range(10000):
        out = i.update(5.0, 0.01)
        assert out <= 0.12 + 1e-12
    assert i.update(5.0, 0.01) == pytest.approx(0.12)


def test_integrator_unwinds_promptly_after_saturating():
    # Anti-windup means the accumulator is clamped, not the output: a clamp on
    # the output alone leaves an unbounded internal value that takes as long to
    # unwind as it took to build.
    i = Integrator(gain=0.02, limit=0.12)
    for _ in range(10000):
        i.update(5.0, 0.01)
    for _ in range(1200):
        i.update(-5.0, 0.01)
    assert i.update(0.0, 0.0) < 0.0


def test_integrator_resets():
    i = Integrator(gain=0.02, limit=0.12)
    i.update(5.0, 1.0)
    i.reset()
    assert i.update(0.0, 0.0) == pytest.approx(0.0)


# ==========================================================================
# navigation
# ==========================================================================


def test_navigator_needs_at_least_one_waypoint():
    with pytest.raises(ValueError):
        Navigator([])


def test_navigator_advances_inside_the_acceptance_radius():
    nav = Navigator([wp(0.0, 100.0, acceptance=30.0), wp(0.0, 300.0)])
    assert nav.update(0.0, 60.0, 0.0, 16.0) is False       # 40 m out
    assert nav.index == 0
    assert nav.update(0.0, 75.0, 0.0, 16.0) is True        # 25 m out
    assert nav.index == 1
    assert nav.active.east == 0.0 and nav.active.north == 300.0


def test_navigator_advances_at_most_one_waypoint_per_update():
    # Three coincident waypoints. A follower that loops internally would skip
    # all of them on one call and report a mission flown that was never flown.
    nav = Navigator([wp(0.0, 0.0), wp(0.0, 0.0), wp(0.0, 0.0)])
    assert nav.update(0.0, 0.0, 0.0, 16.0) is True
    assert nav.index == 1
    assert nav.update(0.0, 0.0, 0.0, 16.0) is True
    assert nav.index == 2


def test_navigator_index_never_goes_backwards():
    nav = Navigator([wp(0.0, 100.0), wp(0.0, 300.0)])
    nav.update(0.0, 100.0, 0.0, 16.0)
    assert nav.index == 1
    seen = []
    # Fly all the way back past the first waypoint. It must not be re-armed:
    # a follower that can go backwards can be trapped between two waypoints and
    # never finish a mission.
    for north in (100.0, 50.0, 0.0, -100.0, 100.0):
        nav.update(0.0, north, 0.0, -16.0)
        seen.append(nav.index)
    assert seen == sorted(seen)
    assert nav.index == 1


def test_navigator_captures_a_waypoint_it_has_flown_past():
    # 40 m beyond a 30 m acceptance circle, still tracking north, so the
    # waypoint is behind. Without this the aircraft orbits a circle it can no
    # longer enter -- a turn radius at 35 deg of bank is 42 m, bigger than the
    # circle itself.
    nav = Navigator([wp(0.0, 100.0, acceptance=30.0), wp(0.0, 300.0)])
    assert nav.update(0.0, 140.0, 0.0, 16.0) is True
    assert nav.index == 1


def test_navigator_does_not_capture_from_outside_the_capture_radius():
    nav = Navigator([wp(0.0, 100.0, acceptance=30.0), wp(0.0, 300.0)])
    # 200 m past it: behind the aircraft, but far outside the capture radius,
    # so the aircraft must still be told to turn round and go get it.
    assert nav.update(0.0, 300.0, 0.0, 16.0) is False
    assert nav.index == 0


def test_navigator_capture_needs_a_ground_track():
    # A stationary aircraft has no track, so "behind me" is undefined and the
    # abeam test must not fire on the resulting zero vector.
    nav = Navigator([wp(0.0, 100.0, acceptance=30.0), wp(0.0, 300.0)])
    assert nav.update(0.0, 140.0, 0.0, 0.0) is False
    assert nav.index == 0


def test_navigator_records_what_it_reached_and_when():
    nav = Navigator([wp(0.0, 100.0, label="A"), wp(0.0, 300.0, label="B")])
    nav.update(0.0, 100.0, 0.0, 16.0, t=12.5)
    nav.update(0.0, 300.0, 0.0, 16.0, t=25.0)
    assert [label for label, _ in nav.reached] == ["A", "B"]
    assert [t for _, t in nav.reached] == [12.5, 25.0]
    assert nav.complete


def test_navigator_stops_when_complete():
    nav = Navigator([wp(0.0, 0.0)])
    nav.update(0.0, 0.0, 0.0, 16.0)
    assert nav.complete
    assert nav.update(0.0, 0.0, 0.0, 16.0) is False
    assert nav.index == 1


# ==========================================================================
# missions
# ==========================================================================


def test_shipped_mission_visits_both_zones_and_returns():
    name, waypoints = load_mission(default_mission_path())
    assert name
    labels = [w.label for w in waypoints]
    assert labels == ["ZONE_A", "ZONE_B", "DEPOT"]
    assert (waypoints[0].east, waypoints[0].north) == (140.0, 60.0)
    assert (waypoints[1].east, waypoints[1].north) == (-120.0, 150.0)
    assert (waypoints[2].east, waypoints[2].north) == (0.0, 0.0)
    assert waypoints[2].is_rtb
    assert not waypoints[0].is_rtb and not waypoints[1].is_rtb


def test_mission_without_an_explicit_rtb_flag_returns_to_its_last_waypoint(tmp_path):
    path = tmp_path / "m.json"
    path.write_text(json.dumps({
        "name": "two", "waypoints": [{"x": 1, "y": 2}, {"x": 3, "y": 4}]}))
    _, waypoints = load_mission(str(path))
    assert [w.is_rtb for w in waypoints] == [False, True]
    assert waypoints[0].label == "WP1"


def test_mission_with_no_waypoints_is_refused(tmp_path):
    path = tmp_path / "empty.json"
    path.write_text(json.dumps({"name": "nothing", "waypoints": []}))
    with pytest.raises(ValueError):
        load_mission(str(path))


# ==========================================================================
# attitude decode
# ==========================================================================


def test_identity_quaternion_is_level_and_pointing_north():
    roll, pitch, yaw = euler_from_quaternion(1.0, 0.0, 0.0, 0.0)
    assert (roll, pitch, yaw) == pytest.approx((0.0, 0.0, 0.0))


def test_quaternion_decode_matches_a_known_rotation():
    for axis, index in ((1, 0), (2, 1), (3, 2)):
        angle = math.radians(20.0)
        q = [math.cos(angle / 2), 0.0, 0.0, 0.0]
        q[axis] = math.sin(angle / 2)
        got = euler_from_quaternion(*q)
        assert got[index] == pytest.approx(angle, abs=1e-9)
        for other in range(3):
            if other != index:
                assert got[other] == pytest.approx(0.0, abs=1e-9)


def test_degenerate_quaternion_does_not_raise():
    assert euler_from_quaternion(0.0, 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)


# ==========================================================================
# state machine and channel mapping
# ==========================================================================


def test_nothing_is_commanded_before_the_first_sensor_message():
    ap = Autopilot(Navigator([wp(0.0, 100.0)]))
    blank = VehicleState()
    command = ap.tick(blank, 0.008)
    assert ap.state == WAIT_FOR_SENSORS
    assert command == {"aileron": 0.0, "elevator": 0.0, "rudder": 0.0, "throttle": 0.0}


def test_attitude_alone_does_not_arm_the_autopilot():
    ap = Autopilot(Navigator([wp(0.0, 100.0)]))
    s = VehicleState()
    s.have_attitude = True
    ap.tick(s, 0.008)
    assert ap.state == WAIT_FOR_SENSORS


def test_state_machine_runs_wait_armed_cruise_nav_rtb_loiter():
    nav = Navigator([wp(200.0, 0.0, alt=45.0, label="A"),
                     wp(0.0, 0.0, alt=45.0, label="HOME", rtb=True)])
    ap = Autopilot(nav, arm_settle_s=1.0)
    seen = [ap.state]

    s = flying_state(t=0.0, east=-200.0, north=0.0, yaw=math.pi / 2)
    ap.tick(s, 0.0)
    seen.append(ap.state)                                   # ARMED

    s.t = 2.0
    ap.tick(s, 0.008)
    seen.append(ap.state)                                   # CRUISE -> NAV

    s.t, s.east = 4.0, 195.0                                # at waypoint A
    ap.tick(s, 0.008)
    seen.append(ap.state)                                   # RTB

    s.t, s.east = 6.0, 5.0                                  # home
    ap.tick(s, 0.008)
    seen.append(ap.state)                                   # LOITER

    assert seen[0] == WAIT_FOR_SENSORS
    assert ARMED in seen
    assert RTB in seen
    assert seen[-1] == LOITER
    assert ap.nav.complete
    assert [label for label, _ in nav.reached] == ["A", "HOME"]
    assert ap.mission_complete_t == 6.0


def test_cruise_waits_for_the_commanded_altitude_before_navigating():
    ap = Autopilot(Navigator([wp(200.0, 0.0, alt=120.0)]), arm_settle_s=0.5)
    s = flying_state(t=0.0, altitude=45.0)
    ap.tick(s, 0.0)
    s.t = 1.0
    ap.tick(s, 0.008)
    assert ap.state == "CRUISE"                # 75 m below the commanded altitude
    s.t, s.altitude = 5.0, 118.0
    ap.tick(s, 0.008)
    assert ap.state == NAV


def test_channel_signs_are_applied_once_and_only_once():
    # The aero model's channels are not in the aerospace sense: positive
    # elevator is nose DOWN and positive rudder is nose LEFT. The control laws
    # are written in the aerospace sense, so exactly one negation must happen,
    # in the channel map. Two negations, or none, both fly.
    ap = Autopilot(Navigator([wp(0.0, 500.0, alt=45.0)]))
    ap.state = NAV
    s = flying_state(altitude=25.0, pitch=0.0, airspeed=18.0)   # 20 m low
    command = ap.tick(s, 0.008)
    assert ap.debug["pitch_cmd"] > 0.0                          # wants nose up
    assert command["elevator"] < 0.0                            # so: negative channel
    assert CHANNEL_SIGN_ELEVATOR == -1.0
    assert CHANNEL_SIGN_RUDDER == -1.0
    assert CHANNEL_SIGN_AILERON == 1.0


def test_a_waypoint_to_the_right_commands_right_bank():
    ap = Autopilot(Navigator([wp(500.0, 0.0, alt=45.0)]))
    ap.state = NAV
    s = flying_state(east=0.0, north=0.0, yaw=0.0)   # heading north, target east
    command = ap.tick(s, 0.008)
    assert ap.debug["heading_error"] == pytest.approx(math.pi / 2)
    assert ap.debug["bank_cmd"] == pytest.approx(BANK_LIMIT_RAD)
    assert command["aileron"] > 0.0                  # positive channel = right bank


def test_every_channel_stays_normalised_across_a_wide_state_sweep():
    ap = Autopilot(Navigator([wp(140.0, 60.0, alt=50.0), wp(0.0, 0.0, rtb=True)]))
    ap.state = NAV
    for altitude in (0.0, 45.0, 200.0):
        for airspeed in (0.0, 8.0, 16.0, 40.0):
            for roll in (-1.2, 0.0, 1.2):
                for pitch in (-1.0, 0.0, 1.0):
                    for yaw in (-3.0, 0.0, 3.0):
                        s = flying_state(altitude=altitude, airspeed=airspeed,
                                         roll=roll, pitch=pitch, yaw=yaw,
                                         p=3.0, q=-3.0, r=2.0)
                        command = ap.tick(s, 0.008)
                        assert -1.0 <= command["aileron"] <= 1.0
                        assert -1.0 <= command["elevator"] <= 1.0
                        assert -1.0 <= command["rudder"] <= 1.0
                        assert 0.0 <= command["throttle"] <= 1.0


def test_a_rewound_clock_does_not_integrate_backwards():
    ap = Autopilot(Navigator([wp(0.0, 500.0)]))
    ap.state = NAV
    s = flying_state(altitude=20.0)
    for _ in range(50):
        ap.tick(s, 0.008)
    wound = ap.pitch_i.value
    assert wound != 0.0
    # dt is clamped at the loop boundary, so a negative interval cannot subtract
    # a spurious chunk of integral here.
    ap.tick(s, -5.0)
    assert ap.pitch_i.value == pytest.approx(wound)
