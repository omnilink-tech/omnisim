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

"""Pure-math tests for scripts/capture/camera_path.py.

No Webots, no IPC. Verifies that the path samples respect keyframe
endpoints, easing changes the in-between values without changing the
endpoints, and slerp produces unit-quaternion-equivalent rotations.

Run with:
    pytest tests/capture/test_camera_path.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "capture"))

from camera_path import (  # noqa: E402
    EASE,
    axis_angle_to_quat,
    catmull_rom,
    get_ease,
    look_at_orientation,
    normalize_keyframes,
    quat_to_axis_angle,
    sample_path,
    sample_path_uniform,
    slerp_quat,
)


# ---- easing ------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(EASE))
def test_ease_endpoints(name):
    fn = get_ease(name)
    assert math.isclose(fn(0.0), 0.0, abs_tol=1e-9)
    assert math.isclose(fn(1.0), 1.0, abs_tol=1e-9)


def test_ease_unknown():
    with pytest.raises(ValueError):
        get_ease("nonsense")


# ---- catmull-rom -------------------------------------------------------------


def test_catmull_rom_passes_through_endpoints():
    p0 = [0.0, 0.0, 0.0]
    p1 = [1.0, 2.0, 3.0]
    p2 = [4.0, 1.0, 5.0]
    p3 = [6.0, 0.0, 7.0]
    assert catmull_rom(p0, p1, p2, p3, 0.0) == pytest.approx(p1)
    assert catmull_rom(p0, p1, p2, p3, 1.0) == pytest.approx(p2)


# ---- quaternion + slerp ------------------------------------------------------


def test_axis_angle_quat_roundtrip():
    aa = [0.0, 1.0, 0.0, math.pi / 3]
    q = axis_angle_to_quat(aa)
    aa2 = quat_to_axis_angle(q)
    # Axis may flip sign with angle reflected; compare unit-rotated forward vec.
    q2 = axis_angle_to_quat(aa2)
    for a, b in zip(q, q2):
        assert math.isclose(a, b, abs_tol=1e-6) or math.isclose(a, -b, abs_tol=1e-6)


def test_slerp_endpoints():
    q0 = axis_angle_to_quat([0.0, 0.0, 1.0, 0.0])
    q1 = axis_angle_to_quat([0.0, 0.0, 1.0, math.pi / 2])
    s0 = slerp_quat(q0, q1, 0.0)
    s1 = slerp_quat(q0, q1, 1.0)
    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(s0, q0))
    assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(s1, q1))


def test_slerp_short_arc():
    # Going from -pi/4 to +pi/4 around Z. Short arc passes through 0.
    q0 = axis_angle_to_quat([0.0, 0.0, 1.0, -math.pi / 4])
    q1 = axis_angle_to_quat([0.0, 0.0, 1.0, math.pi / 4])
    mid = slerp_quat(q0, q1, 0.5)
    aa = quat_to_axis_angle(mid)
    # Midpoint should be near identity (angle ~ 0).
    assert math.isclose(aa[3], 0.0, abs_tol=1e-6)


# ---- keyframe normalization --------------------------------------------------


def test_normalize_keyframes_target_to_orientation():
    kfs = normalize_keyframes([
        {"t": 0.0, "position": [-5, 0, 0], "target": [0, 0, 0]},
        {"t": 1.0, "position": [0, -5, 0], "target": [0, 0, 0]},
    ])
    assert len(kfs) == 2
    assert "orientation" in kfs[0]
    # Looking from -X to origin -> forward direction +X is already the default,
    # so the angle should be ~0.
    assert math.isclose(kfs[0]["orientation"][3], 0.0, abs_tol=1e-6)


def test_normalize_keyframes_sort_by_t():
    kfs = normalize_keyframes([
        {"t": 2.0, "position": [0, 0, 0], "target": [1, 0, 0]},
        {"t": 0.0, "position": [1, 0, 0], "target": [2, 0, 0]},
    ])
    assert kfs[0]["t"] == 0.0
    assert kfs[1]["t"] == 2.0


def test_normalize_keyframes_rejects_bad_input():
    with pytest.raises(ValueError):
        normalize_keyframes([])
    with pytest.raises(ValueError):
        normalize_keyframes([{"t": 0.0, "position": [0, 0]}])  # too short
    with pytest.raises(ValueError):
        normalize_keyframes([{"t": 0.0, "position": [0, 0, 0]}])  # missing orientation/target


# ---- sample_path -------------------------------------------------------------


def test_sample_path_clamps_outside_range():
    kfs = normalize_keyframes([
        {"t": 0.0, "position": [0, 0, 0], "target": [1, 0, 0]},
        {"t": 1.0, "position": [10, 0, 0], "target": [11, 0, 0]},
    ])
    pos_before, _ = sample_path(kfs, -5.0)
    pos_after, _ = sample_path(kfs, 5.0)
    assert pos_before == pytest.approx([0, 0, 0])
    assert pos_after == pytest.approx([10, 0, 0])


def test_sample_path_passes_through_keyframes():
    kfs = normalize_keyframes([
        {"t": 0.0, "position": [0, 0, 0], "target": [1, 0, 0]},
        {"t": 1.0, "position": [10, 0, 0], "target": [11, 0, 0]},
        {"t": 2.0, "position": [20, 5, 0], "target": [21, 5, 0]},
    ])
    for k in kfs:
        pos, _ = sample_path(kfs, k["t"], ease="linear")
        assert pos == pytest.approx(k["position"], abs=1e-6)


def test_sample_path_uniform_frame_count():
    kfs = normalize_keyframes([
        {"t": 0.0, "position": [0, 0, 0], "target": [1, 0, 0]},
        {"t": 1.0, "position": [10, 0, 0], "target": [11, 0, 0]},
    ])
    samples = sample_path_uniform(kfs, duration_s=1.0, fps=30)
    assert len(samples) == 30


def test_sample_path_uniform_rejects_bad_args():
    kfs = normalize_keyframes([
        {"t": 0.0, "position": [0, 0, 0], "target": [1, 0, 0]},
        {"t": 1.0, "position": [10, 0, 0], "target": [11, 0, 0]},
    ])
    with pytest.raises(ValueError):
        sample_path_uniform(kfs, duration_s=0.0, fps=30)
    with pytest.raises(ValueError):
        sample_path_uniform(kfs, duration_s=1.0, fps=0)


# ---- look_at -----------------------------------------------------------------


def test_look_at_zero_distance():
    aa = look_at_orientation([1, 2, 3], [1, 2, 3])
    assert aa == [0.0, 0.0, 1.0, 0.0]


def test_look_at_along_x():
    aa = look_at_orientation([0, 0, 0], [5, 0, 0])
    assert math.isclose(aa[3], 0.0, abs_tol=1e-6)
