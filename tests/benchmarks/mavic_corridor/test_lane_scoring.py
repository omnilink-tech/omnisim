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

"""lane_scoring's three rules, on synthetic trajectories.

No engine. Every case here is geometry chosen so the expected value is exact,
which is the point: the numbers this module produces were disputed between two
machines while the rules lived in prose, and the same sample admitted readings
of +0.49, +1.25 and +3.02 depending on which crossing of a station the reader
took. `test_station_is_the_first_crossing_after_the_anchor` is that case, pinned.

    python -m pytest tests/benchmarks/mavic_corridor/test_lane_scoring.py -v
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import lane_scoring as ls           # noqa: E402

# The benchmark's route. Leg index 3 runs (4.0, -0.5) -> (9.0, -0.5), heading
# +x, so left of travel is +y and that is the side shelf 2 stands on.
ROUTE = [(-0.5, -0.5), (-0.5, 4.5), (4.0, 4.5), (4.0, -0.5),
         (9.0, -0.5), (9.0, 4.5), (13.5, 4.5), (13.5, -0.5)]
LEG = 3
FLYING_Z = 1.2


def sample(t, x, y, z=FLYING_Z, yaw=0.0):
    return {"t": t, "x": x, "y": y, "z": z, "yaw": yaw, "mode": "goto",
            "fault": None}


def test_left_of_travel_is_positive():
    a, b = ls.legs_of(ROUTE)[LEG]
    # the leg runs +x along y = -0.5; a point at y = 0.0 is 0.5 m to its left
    assert ls.cross_track((6.0, 0.0), a, b) == pytest.approx(0.5)
    assert ls.cross_track((6.0, -1.0), a, b) == pytest.approx(-0.5)
    assert ls.along_track((6.0, 0.0), a, b) == pytest.approx(2.0)


def test_constant_offset_is_reported_exactly():
    traj = [sample(i * 0.1, 4.8 + i * 0.1, -0.25) for i in range(30)]
    st = ls.leg_summary(traj, ROUTE)[LEG]
    assert st["n"] == 30
    assert st["mean_abs_m"] == pytest.approx(0.25)
    assert st["max_abs_m"] == pytest.approx(0.25)
    assert st["mean_signed_m"] == pytest.approx(0.25)


def test_a_turn_is_not_an_excursion():
    """Samples inside CORNER_SKIP_M of a waypoint are dropped, not averaged in."""
    lane = [sample(1.0 + i * 0.1, 4.8 + i * 0.1, -0.25) for i in range(30)]
    in_turn = [sample(0.1, 4.05, 1.40), sample(0.2, 4.20, 0.30)]
    clean = ls.leg_summary(lane, ROUTE)[LEG]
    withturn = ls.leg_summary(in_turn + lane, ROUTE)[LEG]
    assert withturn["n"] == clean["n"]
    assert withturn["max_abs_m"] == pytest.approx(clean["max_abs_m"])


def test_ground_samples_are_ignored():
    flying = [sample(i * 0.1, 4.8 + i * 0.1, -0.25) for i in range(10)]
    fallen = [sample(9.0 + i * 0.1, 5.0 + i * 0.1, 2.0, z=0.04) for i in range(10)]
    assert ls.leg_summary(flying + fallen, ROUTE)[LEG]["n"] == 10


def test_leg_assignment_never_goes_backwards():
    """A serpentine's parallel lanes must not claim each other's samples."""
    traj = ([sample(i * 0.1, -0.5, -0.4 + i * 0.1) for i in range(40)]
            + [sample(4.0 + i * 0.1, -0.4 + i * 0.1, 4.5) for i in range(40)]
            + [sample(8.0 + i * 0.1, 4.0, 4.4 - i * 0.1) for i in range(40)])
    indices = [leg for leg, _ in ls.assign_legs(traj, ROUTE)]
    assert indices == sorted(indices)


# The corner overshoot: three crossings of one station, on one pass.
#
# The aircraft rounds (4.0, -0.5) and settles onto the lane while oscillating in
# x, so it reaches 0.9 m along the leg three times at three different offsets.
# Only the first after the anchor is the outbound pass, and it is the reading.
THREE_CROSSINGS = [
    sample(0.0, 4.10, -0.50),   # anchor: closest approach to the waypoint
    sample(1.0, 4.50, 0.10),    # still turning, nearest the previous leg
    sample(2.0, 4.95, 0.30),    # crossing 1, offset +0.80  <- the reading
    sample(3.0, 4.70, 0.05),    # falls back short of the station
    sample(4.0, 4.92, -0.10),   # crossing 2, offset +0.40
    sample(5.0, 4.85, -0.30),   # falls back again
    sample(6.0, 4.91, -0.35),   # crossing 3, offset +0.15
    sample(7.0, 5.60, -0.45),
]


def test_station_is_the_first_crossing_after_the_anchor():
    r = ls.station(THREE_CROSSINGS, ROUTE, LEG, 0.9)
    assert r is not None
    assert r["t"] == 2.0
    assert r["offset_m"] == pytest.approx(0.80)
    # the other two crossings are real and are NOT what a station reading means
    assert r["offset_m"] != pytest.approx(0.40)
    assert r["offset_m"] != pytest.approx(0.15)


def test_anchor_is_the_closest_approach_to_the_start_waypoint():
    anchor, along = ls.leg_anchor(THREE_CROSSINGS, ROUTE, LEG)
    assert anchor["t"] == 0.0
    assert along == pytest.approx(0.10)


def test_station_inside_the_turn_is_not_a_number():
    """The anchor is already past 0.05 m, so a reading there measures the turn."""
    assert ls.station(THREE_CROSSINGS, ROUTE, LEG, 0.05) is None
    assert ls.station_table(THREE_CROSSINGS, ROUTE, LEG, [0.05])[0.05] == "in-turn"


def test_a_station_beyond_the_flight_is_not_reached():
    table = ls.station_table(THREE_CROSSINGS, ROUTE, LEG, [3.0])
    assert table[3.0] == "not reached"


def test_a_leg_the_flight_never_entered_is_not_reached():
    """A flight wedged on leg 3 has no leg-5 reading, and must not invent one."""
    wedged = [sample(i * 0.1, 4.0, 2.4 - i * 0.001) for i in range(60)]
    assert ls.leg_anchor(wedged, ROUTE, 5) is None
    assert ls.station_table(wedged, ROUTE, 5, [0.9])[0.9] == "not reached"


def test_the_corner_ball_drops_a_sample_the_leg_would_otherwise_claim():
    """CORNER_SKIP_M, pinned on a sample that leg assignment does NOT exclude.

    A sample near the corner is usually nearest the PREVIOUS leg, so it never
    reaches the corner test at all. This one is nearer leg 3 than leg 2 (0.30 m
    against 0.45 m) and 0.54 m from the waypoint, so it is the ball that drops it
    and nothing else: at skip 0 it is counted, and it is the largest excursion.
    """
    lane = [sample(1.0 + i * 0.1, 4.8 + i * 0.1, -0.25) for i in range(30)]
    corner = [sample(0.5, 4.45, -0.20)]
    assert ls.assign_legs(corner + lane, ROUTE)[0][0] == LEG
    kept = ls.leg_summary(corner + lane, ROUTE)[LEG]
    counted = ls.leg_summary(corner + lane, ROUTE, corner_skip_m=0.0)[LEG]
    assert kept["n"] == 30 and kept["max_abs_m"] == pytest.approx(0.25)
    assert counted["n"] == 31 and counted["max_abs_m"] == pytest.approx(0.30)


def _recovery(period_s, speed=0.85, amplitude=0.5, decay=0.5, n=60):
    """A lane recovery sampled at one rate: offset 0.5*exp(-s/0.5) along leg 4."""
    return [sample(i * period_s,
                   4.0 + i * period_s * speed,
                   -0.5 + amplitude * math.exp(-(i * period_s * speed) / decay))
            for i in range(n)]


def test_the_station_reading_is_sampling_rate_independent():
    """Why `offset_interp_m` exists: `offset_m` moves with the sample period.

    The sample reading is taken a fraction of a period PAST the station, and on
    a recovering aircraft that is always the low side, so coarsening the sampling
    biases it one way. Interpolating onto the station removes the dependence.
    """
    fine, coarse = _recovery(0.05), _recovery(0.20, n=15)
    exact = 0.5 * math.exp(-0.9 / 0.5)
    readings = [ls.station(t, ROUTE, LEG, 0.9) for t in (fine, coarse)]
    assert all(r is not None for r in readings)
    for r in readings:
        assert r["offset_interp_m"] == pytest.approx(exact, abs=0.005)
    # the sample reading is low on both, and lower on the coarser sampling
    assert readings[0]["offset_m"] < exact
    assert readings[1]["offset_m"] < readings[0]["offset_m"] - 0.01


def test_score_run_serialises_both_halves():
    scored = ls.score_run(THREE_CROSSINGS, ROUTE, LEG, [0.6, 0.9])
    assert LEG in scored["legs"]
    assert scored["stations"]["leg"] == LEG
    assert set(scored["stations"]["readings"]) == {0.6, 0.9}
