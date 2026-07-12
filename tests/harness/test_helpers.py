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

"""Tests for the pure-math helpers in scripts/harness/omnisim_harness.py.

Covers compute_look_at_orientation (the look-at math used by /scene/look_at)
and compute_render_stats (the brightness statistics used by
/world/render_stats).

Run with:
    pytest tests/harness/test_helpers.py
"""

from __future__ import annotations

import io
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "harness"))

from omnisim_harness import (  # noqa: E402
    compute_look_at_orientation,
    compute_render_stats,
)


# ---- compute_look_at_orientation ---------------------------------------------

def _apply_axis_angle(axis: list[float], angle: float, vec: list[float]) -> list[float]:
    """Apply axis-angle rotation to a 3-vector via Rodrigues' formula. Used to
    verify that the harness's computed orientation actually points the
    Webots default forward (+X) at the target.
    """
    ax, ay, az = axis
    cx, cy, cz = vec
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    # k = cross(axis, vec)
    kx = ay * cz - az * cy
    ky = az * cx - ax * cz
    kz = ax * cy - ay * cx
    # dot(axis, vec)
    dot = ax * cx + ay * cy + az * cz
    return [
        cx * c + kx * s + ax * dot * one_minus_c,
        cy * c + ky * s + ay * dot * one_minus_c,
        cz * c + kz * s + az * dot * one_minus_c,
    ]


def _direction(position: list[float], target: list[float]) -> list[float]:
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    dz = target[2] - position[2]
    n = math.sqrt(dx * dx + dy * dy + dz * dz)
    return [dx / n, dy / n, dz / n]


@pytest.mark.parametrize("position,target", [
    ([-10, 0, 0], [0, 0, 0]),       # default-ish view
    ([0, -12, 5], [0, 0, 1]),       # warehouse: south end looking north + down
    ([-10, -16, 10], [0, 0, 1]),    # warehouse: outside-SW elevated 3/4
    ([5, 5, 5], [-5, -5, 0]),       # arbitrary
    ([0, 0, 10], [0, 0, 0]),        # straight down
    ([10, 0, 0], [0, 0, 0]),        # antiparallel to default forward
])
def test_orientation_actually_points_at_target(position, target):
    orientation = compute_look_at_orientation(position, target)
    expected = _direction(position, target)
    actual = _apply_axis_angle(orientation[:3], orientation[3], [1.0, 0.0, 0.0])
    for axis_idx in range(3):
        assert actual[axis_idx] == pytest.approx(expected[axis_idx], abs=1e-6), (
            f"axis {'xyz'[axis_idx]}: orientation {orientation} produced {actual}, "
            f"expected {expected}"
        )


def test_zero_distance_returns_identity():
    orientation = compute_look_at_orientation([0, 0, 0], [0, 0, 0])
    assert orientation == [0.0, 0.0, 1.0, 0.0]


def test_already_pointing_forward_returns_identity():
    # Camera at origin looking at +X — already aligned with default forward.
    orientation = compute_look_at_orientation([0, 0, 0], [10, 0, 0])
    assert orientation == [0.0, 0.0, 1.0, 0.0]


def test_axis_is_unit_length():
    orientation = compute_look_at_orientation([-10, -16, 10], [0, 0, 1])
    ax, ay, az, _ = orientation
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    assert norm == pytest.approx(1.0, abs=1e-9)


# ---- compute_render_stats ----------------------------------------------------

def _png_bytes(color: tuple[int, int, int], width: int = 8, height: int = 8) -> bytes:
    from PIL import Image
    image = Image.new("RGB", (width, height), color=color)
    buf = io.BytesIO()
    image.save(buf, format="PNG")
    return buf.getvalue()


def test_render_stats_solid_grey():
    stats = compute_render_stats(_png_bytes((128, 128, 128)))
    assert stats["pixels"] == 64
    assert stats["mean_brightness"] == 128.0
    assert stats["mean_rgb"] == [128.0, 128.0, 128.0]
    assert stats["max_rgb"] == [128, 128, 128]
    assert stats["saturated_pct"] == 0.0
    assert stats["black_pct"] == 0.0
    assert stats["warnings"] == []


def test_render_stats_white_warns_blown_out():
    stats = compute_render_stats(_png_bytes((255, 255, 255)))
    assert stats["saturated_pct"] == 100.0
    assert stats["mean_brightness"] == 255.0
    assert any("blown out" in w for w in stats["warnings"])


def test_render_stats_black_warns_underexposed():
    stats = compute_render_stats(_png_bytes((0, 0, 0)))
    assert stats["black_pct"] == 100.0
    assert stats["mean_brightness"] == 0.0
    assert any("underexposed" in w for w in stats["warnings"])


def test_render_stats_dark_grey_no_warnings():
    stats = compute_render_stats(_png_bytes((40, 40, 40)))
    assert stats["mean_brightness"] == 40.0
    assert stats["warnings"] == []


def test_render_stats_color_channel_split():
    stats = compute_render_stats(_png_bytes((200, 100, 50)))
    assert stats["mean_rgb"] == [200.0, 100.0, 50.0]
    assert stats["max_rgb"] == [200, 100, 50]
