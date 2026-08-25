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

"""Tests for the capture service shell — supervisor stanza shape and
shot-list parsing. Doesn't launch OmniSim.

Run with:
    pytest tests/capture/test_capture_service.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "capture"))

from omnisim_capture import (  # noqa: E402
    DEFAULT_FOV,
    DEFAULT_HEIGHT,
    DEFAULT_PORT,
    DEFAULT_WIDTH,
    SUPERVISOR_PORT,
    resolve_settle_steps,
    supervisor_stanza,
)


def test_default_ports_distinct_from_harness():
    # Harness lives on 6789/6790; capture must not collide.
    assert DEFAULT_PORT == 6791
    assert SUPERVISOR_PORT == 6792


def test_supervisor_stanza_includes_camera_resolution():
    stanza = supervisor_stanza(3840, 2160, DEFAULT_FOV)
    assert "Robot {" in stanza
    assert 'controller "capture_supervisor"' in stanza
    assert "Camera {" in stanza
    assert "width 3840" in stanza
    assert "height 2160" in stanza
    assert "antiAliasing TRUE" in stanza


def test_supervisor_stanza_uses_synchronous_stepping():
    """The capture supervisor MUST be sync=TRUE — with FALSE, our step(N)
    calls don't actually gate the simulator and settle_steps_per_frame
    becomes a no-op. The validation harness uses FALSE because it doesn't
    need deterministic stepping; the capture service does.
    """
    stanza = supervisor_stanza(DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_FOV)
    assert "synchronization TRUE" in stanza
    assert "synchronization FALSE" not in stanza


def test_supervisor_stanza_uses_supplied_fov():
    stanza = supervisor_stanza(DEFAULT_WIDTH, DEFAULT_HEIGHT, 1.2345)
    assert "fieldOfView 1.2345" in stanza


def test_resolve_settle_explicit_wins():
    settle, ratio = resolve_settle_steps(
        explicit_settle=4, playback_speed=0.5, fps=30, basic_time_step_ms=16,
    )
    assert settle == 4
    # 4 ticks * 16ms = 64ms / 33.33ms wall = 1.92x time-lapse
    assert ratio == pytest.approx(1.92, abs=0.01)


def test_resolve_settle_playback_speed_half():
    """0.5x playback at 30fps with 16ms basic step should resolve to 1 tick
    per frame (actual ratio ~0.48x).
    """
    settle, ratio = resolve_settle_steps(
        explicit_settle=None, playback_speed=0.5, fps=30, basic_time_step_ms=16,
    )
    assert settle == 1
    assert ratio == pytest.approx(0.48, abs=0.01)


def test_resolve_settle_playback_speed_real_time():
    """1.0x at 30fps with 16ms basic step rounds to 2 ticks (~0.96x actual)."""
    settle, ratio = resolve_settle_steps(
        explicit_settle=None, playback_speed=1.0, fps=30, basic_time_step_ms=16,
    )
    assert settle == 2
    assert ratio == pytest.approx(0.96, abs=0.01)


def test_resolve_settle_playback_speed_timelapse():
    """5x time-lapse should advance many sim ticks per frame."""
    settle, _ratio = resolve_settle_steps(
        explicit_settle=None, playback_speed=5.0, fps=30, basic_time_step_ms=16,
    )
    # 5.0 * 33.33ms = 166ms target / 16ms basic = ~10
    assert settle == 10


def test_resolve_settle_clamps_to_one():
    """Very slow playback or high fps shouldn't produce 0 ticks per frame."""
    settle, _ratio = resolve_settle_steps(
        explicit_settle=None, playback_speed=0.1, fps=120, basic_time_step_ms=32,
    )
    assert settle == 1


def test_resolve_settle_default_when_neither_given():
    settle, _ratio = resolve_settle_steps(
        explicit_settle=None, playback_speed=None, fps=30, basic_time_step_ms=16,
    )
    assert settle == 1


def test_example_shotlist_parses():
    """The shipped example shot list should round-trip through normalize_keyframes
    without errors — it's documentation as much as data, so a typo here would be
    a real bug.
    """
    sys.path.insert(0, str(REPO_ROOT / "scripts" / "capture"))
    from camera_path import normalize_keyframes  # noqa: E402

    path = REPO_ROOT / "scripts" / "capture" / "shotlists" / "orbit_warehouse.json"
    with path.open() as fh:
        data = json.load(fh)
    assert "shots" in data and data["shots"]
    for shot in data["shots"]:
        kfs = normalize_keyframes(shot["path_keyframes"])
        assert len(kfs) >= 1
        assert kfs[0]["t"] == pytest.approx(shot["path_keyframes"][0]["t"])
