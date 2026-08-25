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

"""Unit tests for the sensor-path pure logic (no ROS, no simulator).

The lidar helpers live in ``conversions`` rather than in ``sensor_node`` for
exactly this reason: ``sensor_node`` imports ``rclpy`` at module scope, so
anything left inside it cannot be tested without a ROS environment.
"""

import math

import pytest

from omnisim_ros2.conversions import lidar_layer_ranges, select_lidar_layer


# -- select_lidar_layer -------------------------------------------------------

def test_single_layer_is_layer_zero():
    assert select_lidar_layer([1.0, 2.0, 3.0], layers=1, per=3) == 0


def test_picks_the_populated_layer():
    """The shipped Husky case: 4 layers, only one of them sees anything."""
    per = 4
    values = ([None] * per) + ([None] * per) + ([None] * per) + [1.0, 2.0, 3.0, 4.0]
    assert select_lidar_layer(values, layers=4, per=per) == 3


def test_picks_the_layer_with_the_most_returns():
    per = 4
    values = [1.0, None, None, None] + [1.0, 2.0, 3.0, None] + [None] * 4
    assert select_lidar_layer(values, layers=3, per=per) == 1


def test_all_empty_falls_back_to_zero():
    assert select_lidar_layer([None] * 8, layers=2, per=4) == 0


# -- lidar_layer_ranges -------------------------------------------------------

def test_null_becomes_inf_never_zero():
    """A no-return MUST NOT become 0.0, which reads as an obstacle on the lens."""
    out = lidar_layer_ranges([1.0, None, 3.0], layers=1, per=3, layer=0, reverse=False)
    assert out[0] == 1.0
    assert math.isinf(out[1]) and out[1] > 0
    assert out[2] == 3.0
    assert 0.0 not in out


def test_reverse_matches_ros_scan_ordering():
    """Webots hands back a scan leftmost-first; ROS starts at angle_min."""
    fwd = lidar_layer_ranges([1.0, 2.0, 3.0], layers=1, per=3, layer=0, reverse=False)
    rev = lidar_layer_ranges([1.0, 2.0, 3.0], layers=1, per=3, layer=0, reverse=True)
    assert fwd == [1.0, 2.0, 3.0]
    assert rev == [3.0, 2.0, 1.0]


def test_extracts_the_requested_layer_only():
    values = [1.0, 1.1] + [2.0, 2.1] + [3.0, 3.1]
    assert lidar_layer_ranges(values, 3, 2, layer=1, reverse=False) == [2.0, 2.1]


def test_layer_index_is_clamped_not_an_indexerror():
    """An out-of-range lidar_layer parameter must degrade, not crash a timer."""
    values = [1.0, 2.0, 3.0, 4.0]
    assert lidar_layer_ranges(values, 2, 2, layer=99, reverse=False) == [3.0, 4.0]
    assert lidar_layer_ranges(values, 2, 2, layer=-5, reverse=False) == [1.0, 2.0]


@pytest.mark.parametrize("value", [0.0, 0.15])
def test_a_genuine_short_range_survives(value):
    """0.0 must only ever appear when the device really reported it."""
    out = lidar_layer_ranges([value], layers=1, per=1, layer=0, reverse=False)
    assert out == [value]
