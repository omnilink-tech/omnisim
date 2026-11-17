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

"""Tests for the TF relative-transform math. No ROS and no simulator required.

The harness reports every node's pose in WORLD coordinates, but TF needs each
transform relative to its parent. Getting that composition wrong produces a tree
that looks plausible in RViz and is silently wrong, so it is pinned here.
"""

import math

import pytest

from omnisim_ros2.conversions import (
    matrix_to_quaternion,
    quaternion_to_matrix,
    relative_transform,
    yaw_to_quaternion,
)

IDENT = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]


def _quat_close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b)) or all(
        abs(x + y) < tol for x, y in zip(a, b)
    )


def test_identity_parent_passes_child_through():
    (t, q) = relative_transform([0, 0, 0], IDENT, [1, 2, 3], IDENT)
    assert t == pytest.approx((1.0, 2.0, 3.0))
    assert _quat_close(q, (0, 0, 0, 1))


def test_translated_parent_subtracts():
    (t, _) = relative_transform([1, 1, 1], IDENT, [4, 5, 6], IDENT)
    assert t == pytest.approx((3.0, 4.0, 5.0))


def test_rotated_parent_rotates_the_offset_into_its_frame():
    """A parent yawed +90 deg sees a child that is +1 in world X as +1 in its
    own -Y... i.e. R^T applied to the offset, not R."""
    (t, q) = relative_transform([0, 0, 0], _rot_z(math.pi / 2), [1, 0, 0], IDENT)
    assert t == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)
    # And the child's orientation relative to the parent is -90 deg about Z.
    assert _quat_close(q, yaw_to_quaternion(-math.pi / 2), 1e-9)


def test_child_matching_parent_is_the_identity_transform():
    """A child at exactly its parent's pose must produce a null transform,
    whatever that pose is."""
    for angle in (0.0, 0.7, math.pi / 2, math.pi, -2.1):
        m = _rot_z(angle)
        (t, q) = relative_transform([3, -2, 7], m, [3, -2, 7], m)
        assert t == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
        assert _quat_close(q, (0, 0, 0, 1), 1e-9)


def test_composition_round_trips_through_the_parent():
    """Recomposing parent (x) relative must recover the child's world pose."""
    parent_p, parent_m = [1.0, 2.0, 0.5], _rot_z(0.6)
    child_p, child_m = [4.0, -1.0, 2.5], _rot_z(1.9)
    (t, q) = relative_transform(parent_p, parent_m, child_p, child_m)

    # world_child = parent_R . t_rel + parent_t
    r = parent_m
    recomposed = [
        r[0] * t[0] + r[1] * t[1] + r[2] * t[2] + parent_p[0],
        r[3] * t[0] + r[4] * t[1] + r[5] * t[2] + parent_p[1],
        r[6] * t[0] + r[7] * t[1] + r[8] * t[2] + parent_p[2],
    ]
    assert recomposed == pytest.approx(child_p, abs=1e-9)

    # world_R_child = parent_R . rel_R
    rel = quaternion_to_matrix(q)
    composed = [
        sum(r[i * 3 + k] * rel[k * 3 + j] for k in range(3))
        for i in range(3)
        for j in range(3)
    ]
    assert _quat_close(matrix_to_quaternion(composed), matrix_to_quaternion(child_m), 1e-8)


def test_poseless_parent_orientation_falls_back_to_identity():
    """A [null]*9 orientation must not crash the transform."""
    (t, q) = relative_transform([0, 0, 0], [None] * 9, [1, 0, 0], [None] * 9)
    assert t == pytest.approx((1.0, 0.0, 0.0))
    assert _quat_close(q, (0, 0, 0, 1))


def test_axis_angle_parent_orientation_is_accepted():
    """The viewpoint endpoints return 4-element axis-angle; accept both forms."""
    (t, _) = relative_transform([0, 0, 0], [0, 0, 1, math.pi / 2], [1, 0, 0], IDENT)
    assert t == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)


def test_quaternion_to_matrix_round_trip():
    for yaw in (0.0, 0.4, math.pi / 2, -1.3, math.pi):
        q = yaw_to_quaternion(yaw)
        assert _quat_close(matrix_to_quaternion(quaternion_to_matrix(q)), q, 1e-9)


def test_yaw_to_quaternion_is_about_z():
    x, y, z, w = yaw_to_quaternion(math.pi / 2)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert z == pytest.approx(math.sin(math.pi / 4))
    assert w == pytest.approx(math.cos(math.pi / 4))
