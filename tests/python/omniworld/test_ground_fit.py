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

"""Ground-fit realism pass tests."""

from __future__ import annotations

import math

import pytest

from omniworld.primitives.heightmap import Heightmap
from omniworld.realism.ground_fit import GroundFit, ground_fit_on_heightmap


def _flat_heightmap(w: int, h: int, value: float = 0.0) -> Heightmap:
    return Heightmap(w, h, [value] * (w * h))


def _linear_ramp_x(w: int, h: int, slope: float) -> Heightmap:
    """Flat in y, rising linearly in x: z = slope * x."""
    data = []
    for j in range(h):
        for i in range(w):
            data.append(slope * i)
    return Heightmap(w, h, data)


def test_ground_fit_on_flat_returns_up_axis():
    hm = _flat_heightmap(16, 16, value=0.7)
    fit = ground_fit_on_heightmap(
        (5.0, 5.0), hm,
        origin=(0.0, 0.0), x_spacing=1.0, y_spacing=1.0,
    )
    assert fit.translation[2] == pytest.approx(0.7)
    assert fit.slope_deg < 0.1
    # Axis should be effectively +Z (no tilt).
    ax, ay, _, angle = fit.rotation
    assert abs(ax) < 1e-3 and abs(ay) < 1e-3
    assert angle < 1e-3


def test_ground_fit_on_slope_tilts_rotation():
    # 1 unit of z per 1 unit of x -> 45-degree slope.
    hm = _linear_ramp_x(16, 16, slope=1.0)
    fit = ground_fit_on_heightmap(
        (5.0, 5.0), hm,
        origin=(0.0, 0.0), x_spacing=1.0, y_spacing=1.0,
    )
    # Slope should be ~45 degrees.
    assert 40.0 < fit.slope_deg < 50.0
    ax, ay, _, angle = fit.rotation
    # Axis of rotation is perpendicular to both +Z and the slope normal,
    # which lies in the x-z plane; so the axis should be along +y.
    assert abs(ay) > 0.8


def test_ground_fit_respects_max_tilt():
    hm = _linear_ramp_x(16, 16, slope=1.0)  # ~45 degrees
    fit = ground_fit_on_heightmap(
        (5.0, 5.0), hm,
        origin=(0.0, 0.0), x_spacing=1.0, y_spacing=1.0,
        max_tilt_rad=math.radians(20.0),
    )
    # Tilt should be clamped to 20 deg (with ~1 deg slack for plane fit).
    assert fit.slope_deg <= 21.0


def test_ground_fit_sink_offset_lowers_z():
    hm = _flat_heightmap(8, 8, value=2.0)
    normal = ground_fit_on_heightmap(
        (3.0, 3.0), hm,
        origin=(0.0, 0.0), x_spacing=1.0, y_spacing=1.0,
        sink_offset=0.0,
    )
    sunk = ground_fit_on_heightmap(
        (3.0, 3.0), hm,
        origin=(0.0, 0.0), x_spacing=1.0, y_spacing=1.0,
        sink_offset=-0.2,
    )
    assert sunk.translation[2] == pytest.approx(normal.translation[2] - 0.2)


def test_ground_fit_twist_is_applied_on_slope():
    """On a sloped surface, applying a twist must change the rotation
    from the pure up-align case."""
    hm = _linear_ramp_x(16, 16, slope=0.3)
    no_twist = ground_fit_on_heightmap(
        (5.0, 5.0), hm,
        origin=(0.0, 0.0), x_spacing=1.0, y_spacing=1.0,
        twist_rad=0.0,
    )
    with_twist = ground_fit_on_heightmap(
        (5.0, 5.0), hm,
        origin=(0.0, 0.0), x_spacing=1.0, y_spacing=1.0,
        twist_rad=math.pi / 4,
    )
    assert no_twist.rotation != with_twist.rotation


def test_ground_fit_deterministic_given_same_inputs():
    hm = _linear_ramp_x(16, 16, slope=0.25)
    a = ground_fit_on_heightmap(
        (4.5, 3.2), hm,
        origin=(0.0, 0.0), x_spacing=1.0, y_spacing=1.0,
        twist_rad=0.3,
    )
    b = ground_fit_on_heightmap(
        (4.5, 3.2), hm,
        origin=(0.0, 0.0), x_spacing=1.0, y_spacing=1.0,
        twist_rad=0.3,
    )
    assert a == b
