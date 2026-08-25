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

"""ffmpeg wrappers for the OmniSim capture service.

Encodes a directory of `frame_%06d.png` images into a video. Two presets:

- `h264`: x264 + yuv420p, broadly compatible (web/YouTube/social).
  CRF-controlled (default 16 = visually lossless).
- `prores`: Apple ProRes 422 HQ, large but ideal as a master/edit source
  for downstream NLE work.

The shape is intentionally narrow: we don't try to expose every ffmpeg
flag — render.py and the HTTP service take a small dict of knobs and we
turn them into a command line. If you need something exotic, drop the
PNG sequence into ffmpeg yourself.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path
from typing import Iterable


def find_ffmpeg() -> str | None:
    """Resolve ffmpeg via PATH; falls back to the common Windows install
    location used in this repo's launch scripts.
    """
    found = shutil.which("ffmpeg")
    if found:
        return found
    candidates = [
        r"C:\ffmpeg\bin\ffmpeg.exe",
        "/usr/local/bin/ffmpeg",
        "/opt/homebrew/bin/ffmpeg",
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return None


def build_ffmpeg_command(
    ffmpeg: str,
    frames_glob: str,
    fps: int,
    output: str,
    codec: str = "h264",
    crf: int = 16,
    pixel_format: str | None = None,
    preset: str = "slow",
    lossless: bool = False,
    extra_input_args: Iterable[str] = (),
    extra_output_args: Iterable[str] = (),
) -> list[str]:
    """Build the argv for an offline frame-sequence encode. `frames_glob`
    points at the PNG sequence, e.g. `<dir>/frame_%06d.png`.

    Quality knobs:
      crf       Lower = better. Visually-lossless h264 ~= 14-16; web ~= 20-23.
                Default 16 (visually lossless on most content).
      preset    Encoder speed/quality tradeoff. "slow" (default) is the right
                spot for typical renders; "veryslow" buys ~5-10% smaller files
                at much higher encode cost.
      lossless  Mathematically lossless. h264: -qp 0 (very large files). h265:
                -x265-params lossless=1. ProRes is already a master format and
                ignores this. vp9: -lossless 1.
      pixel_format  Override the default. Note that yuv420p is required for
                broad device compatibility (Apple, web); yuv444p preserves
                more chroma but won't play in QuickTime / mobile browsers.
    """
    if codec not in {"h264", "prores", "vp9", "h265"}:
        raise ValueError(f"unsupported codec: {codec!r}")

    cmd: list[str] = [
        ffmpeg,
        "-y",
        "-framerate", str(int(fps)),
        *list(extra_input_args),
        "-i", frames_glob,
        # Force even width/height — libx264 + yuv420p require it, and
        # viewport-export frames can be odd-sized (e.g. 1896x1185).
        "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
    ]

    if codec == "h264":
        pix = pixel_format or "yuv420p"
        cmd += [
            "-c:v", "libx264",
            "-preset", preset,
        ]
        if lossless:
            cmd += ["-qp", "0"]
        else:
            cmd += ["-crf", str(int(crf))]
        cmd += [
            "-pix_fmt", pix,
            "-movflags", "+faststart",
        ]
    elif codec == "h265":
        pix = pixel_format or "yuv420p"
        cmd += [
            "-c:v", "libx265",
            "-preset", preset,
        ]
        if lossless:
            cmd += ["-x265-params", "lossless=1"]
        else:
            cmd += ["-crf", str(int(crf))]
        cmd += [
            "-pix_fmt", pix,
            "-tag:v", "hvc1",
        ]
    elif codec == "prores":
        # ProRes 422 HQ: profile 3. Pixel format must be yuv422p10le.
        cmd += [
            "-c:v", "prores_ks",
            "-profile:v", "3",
            "-pix_fmt", pixel_format or "yuv422p10le",
            "-vendor", "apl0",
        ]
    elif codec == "vp9":
        pix = pixel_format or "yuv420p"
        cmd += [
            "-c:v", "libvpx-vp9",
            "-pix_fmt", pix,
        ]
        if lossless:
            cmd += ["-lossless", "1"]
        else:
            cmd += ["-crf", str(int(crf)), "-b:v", "0"]

    cmd += list(extra_output_args)
    cmd += [output]
    return cmd


def encode_sequence(
    frames_dir: Path,
    output: Path,
    *,
    fps: int,
    codec: str = "h264",
    crf: int = 16,
    preset: str = "slow",
    lossless: bool = False,
    frame_pattern: str = "frame_%06d.png",
    ffmpeg: str | None = None,
) -> dict:
    """Run ffmpeg on a directory of PNG frames. Raises RuntimeError if
    ffmpeg isn't available or the encode fails. Returns a dict with the
    output path, command line that was run, and ffmpeg's stderr tail
    (useful for surfacing failures to the user).
    """
    bin_path = ffmpeg or find_ffmpeg()
    if not bin_path:
        raise RuntimeError(
            "ffmpeg not found on PATH and no fallback location matched. "
            "Install ffmpeg or set the location explicitly."
        )
    frames_glob = str(frames_dir / frame_pattern)
    output.parent.mkdir(parents=True, exist_ok=True)
    cmd = build_ffmpeg_command(
        bin_path,
        frames_glob=frames_glob,
        fps=fps,
        output=str(output),
        codec=codec,
        crf=crf,
        preset=preset,
        lossless=lossless,
    )
    proc = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        check=False,
    )
    stderr_tail = "\n".join(proc.stderr.splitlines()[-20:])
    if proc.returncode != 0:
        raise RuntimeError(
            f"ffmpeg encode failed (exit {proc.returncode}):\n{stderr_tail}"
        )
    return {
        "output": str(output),
        "codec": codec,
        "fps": fps,
        "command": cmd,
        "stderr_tail": stderr_tail,
    }
