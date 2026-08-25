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

"""Tests for ffmpeg command-line construction in scripts/capture/encode.py.

We don't run ffmpeg here — we just verify that build_ffmpeg_command emits
the right argv for each codec preset, since a typo in encoder flags is
a silent way for production renders to come out wrong.

Run with:
    pytest tests/capture/test_encode.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "capture"))

from encode import build_ffmpeg_command  # noqa: E402


def test_h264_default_flags():
    cmd = build_ffmpeg_command(
        "ffmpeg", frames_glob="frames/frame_%06d.png", fps=60,
        output="out.mp4", codec="h264", crf=16,
    )
    assert cmd[0] == "ffmpeg"
    assert "-framerate" in cmd
    assert "60" in cmd
    assert "-c:v" in cmd
    assert "libx264" in cmd
    assert "-crf" in cmd
    assert "16" in cmd
    assert "-pix_fmt" in cmd
    assert "yuv420p" in cmd
    assert cmd[-1] == "out.mp4"


def test_h264_lossless_replaces_crf():
    cmd = build_ffmpeg_command(
        "ffmpeg", frames_glob="f", fps=24, output="o", codec="h264", lossless=True,
    )
    # Lossless mode swaps -crf for -qp 0; both should not appear together.
    assert "-qp" in cmd
    qp_idx = cmd.index("-qp")
    assert cmd[qp_idx + 1] == "0"
    assert "-crf" not in cmd


def test_h265_lossless_uses_x265_params():
    cmd = build_ffmpeg_command(
        "ffmpeg", frames_glob="f", fps=24, output="o", codec="h265", lossless=True,
    )
    assert "-x265-params" in cmd
    idx = cmd.index("-x265-params")
    assert "lossless=1" in cmd[idx + 1]
    assert "-crf" not in cmd


def test_preset_passthrough():
    cmd = build_ffmpeg_command(
        "ffmpeg", frames_glob="f", fps=24, output="o", codec="h264",
        preset="veryslow",
    )
    assert "-preset" in cmd
    pre_idx = cmd.index("-preset")
    assert cmd[pre_idx + 1] == "veryslow"


def test_prores_uses_422hq_profile():
    cmd = build_ffmpeg_command(
        "ffmpeg", frames_glob="frames/frame_%06d.png", fps=24,
        output="master.mov", codec="prores",
    )
    assert "prores_ks" in cmd
    assert "-profile:v" in cmd
    # 422 HQ is profile 3.
    profile_idx = cmd.index("-profile:v")
    assert cmd[profile_idx + 1] == "3"
    assert "yuv422p10le" in cmd


def test_h265_uses_hvc1_tag():
    cmd = build_ffmpeg_command(
        "ffmpeg", frames_glob="f/%06d.png", fps=30, output="out.mp4", codec="h265"
    )
    assert "libx265" in cmd
    assert "hvc1" in cmd


def test_unknown_codec_rejected():
    with pytest.raises(ValueError):
        build_ffmpeg_command(
            "ffmpeg", frames_glob="x", fps=24, output="o", codec="theora"
        )


def test_input_args_precede_input_flag():
    cmd = build_ffmpeg_command(
        "ffmpeg", frames_glob="f", fps=24, output="o", codec="h264",
        extra_input_args=["-thread_queue_size", "512"],
    )
    i_idx = cmd.index("-i")
    tq_idx = cmd.index("-thread_queue_size")
    assert tq_idx < i_idx
