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

"""Frame conversions, pinned against named physical cases.

These exist because the module previously used ONE matrix for both the world
and the body conversion, and that mistake survives every property an
involution test can check: the wrong matrix is also its own inverse, is also a
proper rotation, and also round-trips. What it does not do is keep the nose
pointing forward. So every assertion below names a physical configuration and
states the number a pilot would read off an instrument, which is the only kind
of check that could have caught it.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from omnisim_hil.frames import (  # noqa: E402
    enu_to_ned,
    euler_frd_ned,
    flu_to_frd,
    global_to_local,
    local_to_global,
    ned_to_enu,
    quaternion_frd_ned,
    rotation_flu_enu_to_frd_ned,
    swap_flu_frd,
    swap_ned_enu,
)
from omnisim_hil.vec3 import euler_from_mat, mat_vec  # noqa: E402

IDENTITY = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


def _rot_x(t):
    c, s = math.cos(t), math.sin(t)
    return (1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c)


def _rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return (c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c)


def _rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return (c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)


def _deg(v):
    return tuple(math.degrees(x) for x in v)


# -- world vectors ---------------------------------------------------------


def test_enu_to_ned_names_the_axes():
    assert enu_to_ned((1.0, 0.0, 0.0)) == (0.0, 1.0, -0.0)   # east
    assert enu_to_ned((0.0, 1.0, 0.0)) == (1.0, 0.0, -0.0)   # north
    assert enu_to_ned((0.0, 0.0, 1.0)) == (0.0, 0.0, -1.0)   # up -> -down


def test_world_conversion_is_an_involution():
    v = (3.0, -7.0, 11.0)
    assert ned_to_enu(enu_to_ned(v)) == v
    assert swap_ned_enu is enu_to_ned


# -- body vectors ----------------------------------------------------------


def test_flu_to_frd_keeps_forward_and_flips_the_other_two():
    # The regression. Under the old shared matrix, forward mapped to (0,1,0):
    # an aircraft accelerating along its nose reported it as lateral.
    assert flu_to_frd((1.0, 0.0, 0.0)) == (1.0, -0.0, -0.0)
    assert flu_to_frd((0.0, 1.0, 0.0)) == (0.0, -1.0, -0.0)  # left -> -right
    assert flu_to_frd((0.0, 0.0, 1.0)) == (0.0, -0.0, -1.0)  # up   -> -down


def test_body_conversion_is_an_involution():
    v = (3.0, -7.0, 11.0)
    assert flu_to_frd(flu_to_frd(v)) == v
    assert swap_flu_frd is flu_to_frd


def test_world_and_body_conversions_are_different_matrices():
    # Stated as its own test because the two being aliases of one function is
    # precisely the defect, and it reads as harmless in a diff.
    assert flu_to_frd((1.0, 0.0, 0.0)) != enu_to_ned((1.0, 0.0, 0.0))


# -- attitude --------------------------------------------------------------


def test_level_flight_pointing_east_reads_ninety_degrees():
    roll, pitch, yaw = _deg(euler_frd_ned(IDENTITY))
    assert abs(roll) < 1e-9
    assert abs(pitch) < 1e-9
    assert abs(yaw - 90.0) < 1e-9


def test_level_flight_pointing_north_reads_zero():
    # OmniSim yaw is counter-clockwise from EAST; a compass heading is
    # clockwise from NORTH. Nose north is the case where the two disagree by
    # the full 90 degrees and a missing conversion is invisible in neither.
    roll, pitch, yaw = _deg(euler_frd_ned(_rot_z(math.pi / 2.0)))
    assert abs(yaw) < 1e-9
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9


def test_nose_up_is_positive_pitch():
    # Nose up is a NEGATIVE rotation about the FLU +Y (left) axis and a
    # POSITIVE aerospace pitch. The sign flip between them is the reason this
    # conversion is not optional.
    roll, pitch, yaw = _deg(euler_frd_ned(_rot_y(math.radians(-10.0))))
    assert abs(pitch - 10.0) < 1e-6
    assert abs(roll) < 1e-6


def test_right_wing_down_is_positive_roll():
    roll, pitch, yaw = _deg(euler_frd_ned(_rot_x(math.radians(20.0))))
    assert abs(roll - 20.0) < 1e-6
    assert abs(pitch) < 1e-6


def test_attitude_conversion_agrees_with_transforming_the_vectors():
    # The independent check: converting the MATRIX must give the same answer as
    # converting a body vector and the resulting world vector separately.
    # R' v_frd == enu_to_ned(R flu_to_frd(v_frd)) for every v.
    m = (0.4330127, -0.7500000, 0.5000000,
         0.7500000, 0.6250000, 0.2165064,
         -0.5000000, 0.2165064, 0.8437500)
    rp = rotation_flu_enu_to_frd_ned(m)
    for v in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (0.3, -0.5, 0.8)):
        got = mat_vec(rp, v)
        want = enu_to_ned(mat_vec(m, flu_to_frd(v)))
        for a, b in zip(got, want):
            assert abs(a - b) < 1e-9


def test_attitude_conversion_preserves_orthonormality():
    m = _rot_z(0.7)
    rp = rotation_flu_enu_to_frd_ned(m)
    cols = [(rp[0], rp[3], rp[6]), (rp[1], rp[4], rp[7]), (rp[2], rp[5], rp[8])]
    for i, a in enumerate(cols):
        assert abs(math.sqrt(sum(x * x for x in a)) - 1.0) < 1e-12
        for b in cols[i + 1:]:
            assert abs(sum(x * y for x, y in zip(a, b))) < 1e-12


def test_quaternion_matches_the_matrix_it_came_from():
    m = _rot_x(0.3)
    w, x, y, z = quaternion_frd_ned(m)
    assert abs(math.sqrt(w * w + x * x + y * y + z * z) - 1.0) < 1e-12
    # Rebuild the matrix from the quaternion and compare against the euler read.
    rebuilt = (
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
    )
    for a, b in zip(euler_from_mat(rebuilt), euler_frd_ned(m)):
        assert abs(a - b) < 1e-9


# -- geodesy ---------------------------------------------------------------


def test_local_to_global_round_trips():
    lat, lon, alt = local_to_global(1234.0, -567.0, 45.0, 47.397742, 8.545594)
    e, n, u = global_to_local(lat, lon, alt, 47.397742, 8.545594)
    assert abs(e - 1234.0) < 1e-6
    assert abs(n + 567.0) < 1e-6
    assert abs(u - 45.0) < 1e-9


def test_local_to_global_moves_north_for_positive_north():
    lat, lon, _ = local_to_global(0.0, 1000.0, 0.0, 47.397742, 8.545594)
    assert lat > 47.397742
    assert abs(lon - 8.545594) < 1e-12
