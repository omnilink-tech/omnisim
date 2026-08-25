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

"""Unit tests for the geometry conversions. No ROS and no simulator required."""

import math

import pytest

from omnisim_ros2.conversions import (
    axis_angle_to_quaternion,
    is_valid_quaternion,
    matrix_to_quaternion,
    orientation_to_quaternion,
    position_to_xyz,
    quaternion_to_axis_angle,
    sanitize_frame_id,
    sim_time_ms_to_ros,
)

IDENTITY_MATRIX = [1, 0, 0, 0, 1, 0, 0, 0, 1]


def _quat_close(a, b, tol=1e-9):
    """Compare quaternions up to sign, since q and -q are the same rotation."""
    same = all(abs(x - y) < tol for x, y in zip(a, b))
    flipped = all(abs(x + y) < tol for x, y in zip(a, b))
    return same or flipped


def test_identity_matrix_is_identity_quaternion():
    assert _quat_close(matrix_to_quaternion(IDENTITY_MATRIX), (0.0, 0.0, 0.0, 1.0))


def test_matrix_rejects_wrong_length():
    with pytest.raises(ValueError):
        matrix_to_quaternion([1, 0, 0, 0, 1, 0])


@pytest.mark.parametrize(
    "axis,angle",
    [
        ((0, 0, 1), math.pi / 2),
        ((0, 0, 1), math.pi),          # 180 deg: the branch the naive trace formula breaks on
        ((1, 0, 0), math.pi),
        ((0, 1, 0), math.pi),
        ((1, 1, 1), 2.0),
        ((0, 0, 1), -math.pi / 3),
    ],
)
def test_axis_angle_matrix_quaternion_agree(axis, angle):
    """Building a rotation matrix from an axis-angle and converting it must
    produce the same quaternion as converting the axis-angle directly."""
    n = math.sqrt(sum(c * c for c in axis))
    x, y, z = (c / n for c in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    m = [
        t * x * x + c, t * x * y - s * z, t * x * z + s * y,
        t * x * y + s * z, t * y * y + c, t * y * z - s * x,
        t * x * z - s * y, t * y * z + s * x, t * z * z + c,
    ]
    assert _quat_close(matrix_to_quaternion(m), axis_angle_to_quaternion([x, y, z, angle]), 1e-8)


@pytest.mark.parametrize(
    "axis,angle",
    [((0, 0, 1), 1.0), ((1, 0, 0), 2.5), ((1, 1, 0), 0.3), ((0, 0, 1), math.pi)],
)
def test_quaternion_axis_angle_roundtrip(axis, angle):
    n = math.sqrt(sum(c * c for c in axis))
    unit = [c / n for c in axis]
    q = axis_angle_to_quaternion([*unit, angle])
    back = quaternion_to_axis_angle(q)
    again = axis_angle_to_quaternion(back)
    assert _quat_close(q, again, 1e-9)


def test_zero_axis_is_identity():
    assert _quat_close(axis_angle_to_quaternion([0, 0, 0, 1.7]), (0.0, 0.0, 0.0, 1.0))


def test_zero_quaternion_becomes_identity_axis_angle():
    """An unset geometry_msgs/Quaternion is all zeros; treat it as 'no rotation'
    rather than raising, because callers send it when they mean exactly that."""
    assert quaternion_to_axis_angle([0.0, 0.0, 0.0, 0.0]) == [0.0, 0.0, 1.0, 0.0]


def test_quaternion_to_axis_angle_canonicalises_sign():
    """q and -q are the same rotation; the returned angle must be in [0, pi]."""
    ax = quaternion_to_axis_angle([0.0, 0.0, -0.7071068, -0.7071068])
    assert 0.0 <= ax[3] <= math.pi + 1e-9


def test_orientation_dispatches_on_length():
    assert _quat_close(orientation_to_quaternion(IDENTITY_MATRIX), (0, 0, 0, 1))
    assert _quat_close(orientation_to_quaternion([0, 0, 1, 0.0]), (0, 0, 0, 1))


def test_orientation_of_poseless_node_is_identity():
    """The harness reports [null]*9 for nodes with no pose."""
    assert _quat_close(orientation_to_quaternion([None] * 9), (0, 0, 0, 1))
    assert _quat_close(orientation_to_quaternion(None), (0, 0, 0, 1))


def test_orientation_rejects_unknown_length():
    with pytest.raises(ValueError):
        orientation_to_quaternion([1, 2, 3])


def test_position_of_poseless_node_is_origin():
    assert position_to_xyz([None, None, None]) == (0.0, 0.0, 0.0)
    assert position_to_xyz(None) == (0.0, 0.0, 0.0)
    assert position_to_xyz([1, 2, 3]) == (1.0, 2.0, 3.0)


def test_is_valid_quaternion():
    assert is_valid_quaternion([0, 0, 0, 1])
    assert not is_valid_quaternion([0, 0, 0, 0])
    assert not is_valid_quaternion([0, 0, 1])
    assert not is_valid_quaternion([float("nan"), 0, 0, 1])


@pytest.mark.parametrize(
    "ms,expected",
    [
        (0.0, (0, 0)),
        (1000.0, (1, 0)),
        (1500.5, (1, 500_500_000)),
        (-5.0, (0, 0)),           # never emit a negative ROS time
        (16.0, (0, 16_000_000)),
    ],
)
def test_sim_time_conversion(ms, expected):
    assert sim_time_ms_to_ros(ms) == expected


def test_sim_time_nanosec_never_reaches_one_second():
    """A rounding artefact must not produce the invalid (s, 1e9)."""
    for ms in (999.9999999, 1999.9999999, 0.9999999999):
        _, nanosec = sim_time_ms_to_ros(ms)
        assert 0 <= nanosec < 1_000_000_000


def test_sanitize_frame_id():
    assert sanitize_frame_id("HUSKY") == "HUSKY"
    assert sanitize_frame_id("/HUSKY") == "HUSKY"
    assert sanitize_frame_id("#185") == "node_185"
    assert sanitize_frame_id("my robot") == "my_robot"
    assert sanitize_frame_id("") == "unnamed"
