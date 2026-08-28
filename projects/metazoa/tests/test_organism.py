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

"""Unit tests for the pure organism module (no engine, no omnisim import).

Run: python -m pytest projects/metazoa/tests/test_organism.py -q
"""
import math
import os
import random
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..")))

from mz import organism as O  # noqa: E402


# ------------------------------------------------------------------ fixtures
def make_genome(**over):
    g = {"A": 0.8, "omega": 5.0, "dphi": 1.57, "bias_pitch": 0.1, "bias_yaw": -0.2,
         "branch_phase": 0.7, "branch_scale": 0.5, "steer_gain": 0.4}
    g.update(over)
    return g


def make_bodyplan(pattern=(0, 0, 0, 0), target_length=4, branch_rule="none"):
    return {"target_length": target_length,
            "dock_rotation_pattern": list(pattern),
            "branch_rule": branch_rule}


def wrap(a):
    return O.wrap_angle(a)


# ------------------------------------------------------------------ genome
def test_random_genome_in_range_and_valid():
    rng = random.Random(1)
    for _ in range(200):
        g = O.random_genome(rng)
        assert set(g) == set(O.GENOME_KEYS)
        assert O.validate(g) == []
        for k, (lo, hi) in O.GENOME_RANGES.items():
            assert lo <= g[k] <= hi


def test_mutate_stays_in_range_and_leaves_parent():
    rng = random.Random(2)
    parent = make_genome()
    before = dict(parent)
    for rate in (0.5, 1.0, 3.0):
        child = parent
        for _ in range(100):
            child = O.mutate(child, rng, rate=rate)
            assert O.validate(child) == [], O.validate(child)
    assert parent == before
    # mutation actually changes something
    assert O.mutate(parent, random.Random(3)) != parent


def test_validate_rejects_out_of_range_missing_nan_and_unknown():
    assert O.validate(make_genome(A=1.3)) != []
    assert O.validate(make_genome(omega=1.0)) != []
    assert O.validate(make_genome(dphi=2.5)) != []
    assert O.validate(make_genome(bias_pitch=-0.7)) != []
    assert O.validate(make_genome(bias_yaw=0.61)) != []
    assert O.validate(make_genome(branch_scale=1.01)) != []
    assert O.validate(make_genome(steer_gain=-0.1)) != []
    assert O.validate(make_genome(A=float("nan"))) != []
    g = make_genome()
    del g["dphi"]
    assert any("missing dphi" in p for p in O.validate(g))
    assert any("unknown" in p for p in O.validate(make_genome(extra=1.0)))
    assert O.validate("nope") != []
    assert O.validate(make_genome()) == []


# ------------------------------------------------------------------ body plan
def test_random_bodyplan_valid():
    rng = random.Random(4)
    seen_branch = False
    for _ in range(300):
        bp = O.random_bodyplan(rng)
        assert O.validate_bodyplan(bp) == [], O.validate_bodyplan(bp)
        seen_branch |= bp["branch_rule"] != "none"
    assert seen_branch


def test_mutate_bodyplan_stays_valid():
    rng = random.Random(5)
    bp = make_bodyplan((0, 1), target_length=2)
    before = {"target_length": 2, "dock_rotation_pattern": [0, 1], "branch_rule": "none"}
    child = bp
    lengths = set()
    for _ in range(500):
        child = O.mutate_bodyplan(child, rng, rate=2.0)
        assert O.validate_bodyplan(child) == [], O.validate_bodyplan(child)
        lengths.add(child["target_length"])
    assert bp == before
    assert len(lengths) > 1


def test_validate_bodyplan_rejects():
    assert O.validate_bodyplan(make_bodyplan(target_length=9)) != []
    assert O.validate_bodyplan(make_bodyplan(target_length=1)) != []
    assert O.validate_bodyplan(make_bodyplan(pattern=())) != []
    assert O.validate_bodyplan(make_bodyplan(pattern=(0, 4))) != []
    assert O.validate_bodyplan(make_bodyplan(pattern=(0, 1, 2, 3, 0))) != []
    assert O.validate_bodyplan(make_bodyplan(branch_rule={"at": 4, "sides": ["L"]})) != []
    assert O.validate_bodyplan(make_bodyplan(branch_rule={"at": 1, "sides": []})) != []
    assert O.validate_bodyplan(make_bodyplan(branch_rule={"at": 1, "sides": ["L", "L"]})) != []
    assert O.validate_bodyplan(make_bodyplan(branch_rule={"at": 1, "sides": ["X"]})) != []
    assert O.validate_bodyplan(make_bodyplan(branch_rule="ring")) != []
    assert O.validate_bodyplan(make_bodyplan(branch_rule={"at": 1, "sides": ["L", "R"]})) == []


# ------------------------------------------------------------------ axes
def test_axis_pattern_alternating():
    bp = make_bodyplan((0, 1, 0, 1))
    assert O.axes(bp, 4) == ["pitch", "yaw", "pitch", "yaw"]
    # cycled beyond the pattern length
    assert O.axes(bp, 6) == ["pitch", "yaw", "pitch", "yaw", "pitch", "yaw"]


def test_axis_pattern_180_keeps_class():
    assert O.axes(make_bodyplan((0, 2, 0, 2)), 4) == ["pitch"] * 4
    assert O.axes(make_bodyplan((1, 3, 1, 3)), 4) == ["yaw"] * 4
    assert O.axes(make_bodyplan((3,)), 3) == ["yaw"] * 3


def test_relative_dock_rotation():
    bp = make_bodyplan((0, 1, 0, 1))
    assert [O.relative_dock_rotation(bp, i) for i in range(4)] == [0, 1, 3, 1]
    assert O.relative_dock_rotation(make_bodyplan((0, 2)), 1) == 2


# ------------------------------------------------------------------ gait
def test_wave_phase_progresses_by_dphi_per_cell():
    g = make_genome(bias_pitch=0.0, bias_yaw=0.0)
    bp = make_bodyplan((0,))
    n = 5
    for t in (0.0, 0.37, 1.9):
        targets = O.chain_targets(g, bp, n, t)
        assert len(targets) == n
        for i in range(n):
            expected = g["A"] * math.sin(g["omega"] * t + i * g["dphi"])
            assert targets[i] == pytest.approx(expected, abs=1e-12)
    # cell i+1 at t0 equals cell i at t0 + dphi/omega: the crest reaches cell
    # i+1 EARLIER than cell i, so the wave travels head -> tail (decreasing index)
    t0 = 0.4
    lag = g["dphi"] / g["omega"]
    a = O.chain_targets(g, bp, n, t0)
    b = O.chain_targets(g, bp, n, t0 + lag)
    for i in range(n - 1):
        assert a[i + 1] == pytest.approx(b[i], abs=1e-9)


def test_axis_biases_applied_by_class():
    g = make_genome(A=0.5, bias_pitch=0.3, bias_yaw=-0.25)
    bp = make_bodyplan((0, 1, 0, 1))
    targets = O.chain_targets(g, bp, 4, 0.0)
    for i, ax in enumerate(O.axes(bp, 4)):
        wave = 0.5 * math.sin(i * g["dphi"])
        bias = 0.3 if ax == "pitch" else -0.25
        assert targets[i] == pytest.approx(bias + wave, abs=1e-12)


def test_steer_raises_yaw_bias_only():
    g = make_genome(steer_gain=0.4)
    bp = make_bodyplan((0, 1, 0, 1))
    t = 0.123
    base = O.chain_targets(g, bp, 4, t, steer=0.0)
    plus = O.chain_targets(g, bp, 4, t, steer=0.5)
    minus = O.chain_targets(g, bp, 4, t, steer=-0.5)
    for i, ax in enumerate(O.axes(bp, 4)):
        if ax == "yaw":
            assert plus[i] - base[i] == pytest.approx(0.4 * 0.5, abs=1e-12)
            assert minus[i] - base[i] == pytest.approx(-0.4 * 0.5, abs=1e-12)
        else:
            assert plus[i] == base[i]
            assert minus[i] == base[i]
    # steer is clamped to [-1, 1]
    sat = O.chain_targets(g, bp, 4, t, steer=7.0)
    one = O.chain_targets(g, bp, 4, t, steer=1.0)
    assert sat == one


def test_targets_never_exceed_ceiling():
    rng = random.Random(6)
    bp = make_bodyplan((0, 1, 2, 3), branch_rule={"at": 2, "sides": ["L", "R"]})
    for _ in range(300):
        g = O.random_genome(rng)
        t = rng.uniform(0.0, 20.0)
        for steer in (-1.0, 0.0, 1.0, 5.0):
            for v in O.chain_targets(g, bp, 8, t, steer=steer, branches=("L", "R")):
                assert -O.TARGET_CEIL <= v <= O.TARGET_CEIL
    # extreme corners of the genome envelope
    g = make_genome(A=1.2, bias_yaw=0.6, bias_pitch=0.6, steer_gain=0.6)
    for t in (0.0, 0.1, 0.3, 0.7):
        for v in O.chain_targets(g, bp, 8, t, steer=1.0, branches=("L",)):
            assert -O.TARGET_CEIL <= v <= O.TARGET_CEIL
    assert O.TARGET_CEIL < O.MOTOR_LIMIT


def test_branch_cells_use_branch_phase_and_scale():
    g = make_genome(A=1.0, branch_scale=0.5, branch_phase=0.7)
    bp = make_bodyplan((0,), branch_rule={"at": 2, "sides": ["L", "R"]})
    t = 0.31
    targets = O.chain_targets(g, bp, 4, t, branches=("L", "R"))
    assert len(targets) == 6
    wt = g["omega"] * t + 2 * g["dphi"]
    assert targets[4] == pytest.approx(0.5 * math.sin(wt + 0.7), abs=1e-12)
    assert targets[5] == pytest.approx(0.5 * math.sin(wt - 0.7), abs=1e-12)
    # branch_scale 0 silences the branches; branch_rule none yields 0.0
    g0 = make_genome(branch_scale=0.0)
    assert O.chain_targets(g0, bp, 4, t, branches=("L",))[4] == 0.0
    assert O.chain_targets(g, make_bodyplan((0,)), 4, t, branches=("R",))[4] == 0.0


def test_chain_targets_reuses_out_list():
    g = make_genome()
    bp = make_bodyplan((0, 1))
    buf = [9.0, 9.0, 9.0]
    res = O.chain_targets(g, bp, 4, 0.5, out=buf)
    assert res is buf and len(buf) == 4
    assert buf == O.chain_targets(g, bp, 4, 0.5)


def test_wave_speed_estimate():
    g = make_genome(omega=5.0, dphi=1.57)
    # wavelength = 0.12 * 2pi/1.57 = 0.4803 m ; frequency = 5/2pi = 0.7958 Hz
    assert O.wave_speed_estimate(g) == pytest.approx(0.12 * 5.0 / 1.57)
    assert O.wave_speed_estimate(g, cell_len=0.13) == pytest.approx(0.13 * 5.0 / 1.57)
    # wavelength x frequency, explicitly
    wavelength = 0.12 * 2.0 * math.pi / 1.57
    freq = 5.0 / (2.0 * math.pi)
    assert O.wave_speed_estimate(g) == pytest.approx(wavelength * freq)


# ------------------------------------------------------------------ flip
def test_flip_sequence_shape():
    P, F = 2.0, 2.4
    assert O.flip_sequence(0.0, P, F) == 0.0
    # reaches fold inside the first 25 % ...
    assert O.flip_sequence(0.25 * P, P, F) == pytest.approx(F)
    assert O.flip_sequence(0.1 * P, P, F) < F
    # ... monotone on the way up
    prev = -1.0
    for k in range(26):
        v = O.flip_sequence(k / 100.0 * P, P, F)
        assert v >= prev
        prev = v
    # holds for 10 %
    for u in (0.26, 0.30, 0.349):
        assert O.flip_sequence(u * P, P, F) == pytest.approx(F)
    # slow unfold, monotone, back to 0 by the period
    prev = F + 1.0
    for k in range(35, 101):
        v = O.flip_sequence(k / 100.0 * P, P, F)
        assert v <= prev + 1e-12
        prev = v
    assert O.flip_sequence(0.999 * P, P, F) == pytest.approx(0.0, abs=0.01)
    assert O.flip_sequence(P, P, F) == pytest.approx(0.0, abs=1e-12)
    # periodic, and the fold slope stays under the motor's 5 rad/s
    assert O.flip_sequence(2.3 * P, P, F) == pytest.approx(O.flip_sequence(0.3 * P, P, F))
    assert F / (0.25 * P) < 5.0
    # never above the ceiling with the defaults
    for k in range(200):
        assert 0.0 <= O.flip_sequence(k * 0.01 * P) <= O.TARGET_CEIL


# ------------------------------------------------------------------ geometry
def test_face_pose_world():
    cell = (1.0, 2.0, math.pi / 2.0)
    x, y, yaw = O.face_pose(cell, "f_nose")
    assert (x, y) == pytest.approx((1.0, 2.09))
    assert wrap(yaw - math.pi / 2.0) == pytest.approx(0.0)
    x, y, yaw = O.face_pose(cell, "f_left")
    # nose block centre at +0.06 along +y, left face 0.03 toward -x, normal +y rotated: -x
    assert (x, y) == pytest.approx((0.97, 2.06))
    assert wrap(yaw - math.pi) == pytest.approx(0.0)


def test_approach_pose_free_face_at_90deg():
    gap = 0.01
    # a free cell at the origin facing +y: its nose face is at (0, 0.09) with normal +y
    head = O.approach_pose((0.0, 0.0, math.pi / 2.0), "f_nose", gap)
    assert wrap(head[2] + math.pi / 2.0) == pytest.approx(0.0)      # faces -90 deg
    assert head[0] == pytest.approx(0.0, abs=1e-12)
    assert head[1] == pytest.approx(0.09 + 0.09 + gap)             # root offset
    # the head's nose face lands gap away from the free face, normals opposed
    hf = O.face_pose(head, "f_nose")
    ff = O.face_pose((0.0, 0.0, math.pi / 2.0), "f_nose")
    assert O.distance_xy(hf, ff) == pytest.approx(gap)
    assert wrap(hf[2] - ff[2] - math.pi) == pytest.approx(0.0)
    # block centres (free nose block, head nose block) are BLOCK + gap apart
    free_nose_centre = O.compose_pose((0.0, 0.0, math.pi / 2.0), (O.NOSE_X, 0.0, 0.0))
    head_nose_centre = O.compose_pose(head, (O.NOSE_X, 0.0, 0.0))
    assert O.distance_xy(free_nose_centre, head_nose_centre) == pytest.approx(O.BLOCK + gap)
    # the same face handed in as a world pose
    head2 = O.approach_pose(ff, None, gap)
    assert head2 == pytest.approx(head)


def test_approach_pose_side_face_and_tail_face():
    gap = 0.02
    cell = (0.5, -0.3, 0.4)
    for face in ("f_left", "f_right", "f_tail"):
        head = O.approach_pose(cell, face, gap)
        hf = O.face_pose(head, "f_nose")
        ff = O.face_pose(cell, face)
        assert O.distance_xy(hf, ff) == pytest.approx(gap)
        assert wrap(hf[2] - ff[2] - math.pi) == pytest.approx(0.0, abs=1e-12)
    # mating the head's TAIL face instead (appending at the organism's tail)
    head = O.approach_pose(cell, "f_nose", gap, head_face="f_tail")
    hf = O.face_pose(head, "f_tail")
    ff = O.face_pose(cell, "f_nose")
    assert O.distance_xy(hf, ff) == pytest.approx(gap)
    assert wrap(hf[2] - ff[2] - math.pi) == pytest.approx(0.0, abs=1e-12)
    # nose-to-tail: the head is yawed like the free cell, 0.12 + gap ahead
    assert wrap(head[2] - cell[2]) == pytest.approx(0.0, abs=1e-12)
    assert O.distance_xy(head, cell) == pytest.approx(O.CELL_LEN + gap)


def test_heading_error_sign_and_wrap():
    head = (0.0, 0.0, 0.0)
    assert O.heading_error(head, (1.0, 1.0)) == pytest.approx(math.pi / 4.0)   # left = +
    assert O.heading_error(head, (1.0, -1.0)) == pytest.approx(-math.pi / 4.0)
    assert O.heading_error((0.0, 0.0, math.pi), (-1.0, 0.0)) == pytest.approx(0.0)
    e = O.heading_error((0.0, 0.0, 3.0), (-1.0, -0.1))
    assert -math.pi < e <= math.pi
    assert O.heading_error(head, (0.0, 0.0)) == 0.0


def test_steer_from_error():
    assert O.steer_from_error(0.05) == 0.0
    assert O.steer_from_error(None) == 0.0
    assert O.steer_from_error(0.3) == pytest.approx(0.5)
    assert O.steer_from_error(-2.0) == -1.0
    assert O.steer_from_error(0.3, sign=-1.0) == pytest.approx(-0.5)
