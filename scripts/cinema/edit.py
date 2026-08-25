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

"""Editor — concatenate shots, apply look grade, brand, emit multi-aspect.

ffmpeg is the only dependency. Each shot mp4 lives at the viewport's
native size (1896×1184 typically); the editor re-encodes with the
look's grade filter while padding/cropping for each target aspect.

Aspect ratio map:
  "16:9"   — 1920×1080  landscape, standard
  "9:16"   —  720×1280  vertical (Reels / Shorts)
  "1:1"    —  1080×1080 square (Instagram feed)
  "2.39:1" — 2048×858  cinemascope (landing-page hero)
  "4:5"    — 1080×1350 Instagram portrait
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path

from . import brand
from .grade import combine_filters, safe_chroma_pad
from .looks import Look
from .storyboard import Storyboard


ASPECT_DIMS: dict[str, tuple[int, int]] = {
    "16:9":   (1920, 1080),
    "9:16":   ( 720, 1280),
    "1:1":    (1080, 1080),
    "2.39:1": (2048,  858),
    "4:5":    (1080, 1350),
}


def _ffmpeg() -> str:
    p = shutil.which("ffmpeg")
    if p:
        return p
    fallback = "C:/ffmpeg/bin/ffmpeg.exe"
    if Path(fallback).exists():
        return fallback
    raise RuntimeError("ffmpeg not found on PATH")


def _aspect_scale_pad_filter(w: int, h: int) -> str:
    """Scale to fit inside w×h preserving aspect, then pad to exactly w×h.
    Black bars added where the source is the 'wrong' shape (letterbox for
    16:9 → 1:1, pillarbox for 16:9 → 9:16)."""
    return (
        f"scale=w='if(gt(a,{w}/{h}),{w},-2)':h='if(gt(a,{w}/{h}),-2,{h})',"
        f"pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:color=0x0a0a0a"
    )


def _aspect_crop_fill_filter(w: int, h: int) -> str:
    """Scale to FILL the target frame (preserving aspect), cropping
    overflow. Better when the action is centered and you don't want
    letterbox bars — costs you edge content."""
    return (
        f"scale=w='if(gt(a,{w}/{h}),-2,{w})':h='if(gt(a,{w}/{h}),{h},-2)',"
        f"crop={w}:{h}"
    )


def concat_videos(
    inputs: list[Path], out_path: Path, look: Look,
    aspect: str = "16:9", fps: int = 30,
    crop_fill: bool = False,
) -> Path:
    """Concatenate shot clips into one mp4, applying the look's color
    grade + aspect-ratio reframing during the same encode.
    """
    if not inputs:
        raise ValueError("no input shots to concat")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    w, h = ASPECT_DIMS.get(aspect, (1920, 1080))
    aspect_filter = (
        _aspect_crop_fill_filter(w, h) if crop_fill
        else _aspect_scale_pad_filter(w, h)
    )
    full_vf = safe_chroma_pad(combine_filters(look.grade_filter, aspect_filter))

    # Use the concat demuxer (no re-encode of audio; video re-encoded with
    # our filter chain). Build a tmp manifest file.
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False, encoding="utf-8") as f:
        for p in inputs:
            safe = str(p).replace("\\", "/").replace("'", "'\\''")
            f.write(f"file '{safe}'\n")
        manifest = Path(f.name)

    try:
        cmd = [
            _ffmpeg(), "-y",
            "-f", "concat", "-safe", "0", "-i", str(manifest),
            "-vf", full_vf,
            "-r", str(int(fps)),
            "-c:v", "libx264", "-preset", "slow", "-crf", "16",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(out_path),
        ]
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        if res.returncode != 0:
            raise RuntimeError(
                f"ffmpeg concat failed (exit {res.returncode}):\n{res.stderr[-2000:]}"
            )
    finally:
        try:
            manifest.unlink()
        except OSError:
            pass
    return out_path


def assemble(
    raw_shots: list[Path], storyboard: Storyboard, look: Look, out_dir: Path,
) -> dict[str, Path]:
    """Build every aspect-ratio variant. Returns {label: path}.

    Title card + end slate are pre-rendered once at 16:9 then re-cropped
    per aspect by the same scale-and-pad filter as the shots — keeps the
    brand mark on-frame in any aspect.
    """
    if not raw_shots:
        raise ValueError("no shots to assemble")
    out_dir.mkdir(parents=True, exist_ok=True)

    title_clip = end_clip = None
    if storyboard.brand.title_card or storyboard.brand.end_slate:
        if storyboard.brand.title_card:
            title_clip = out_dir / "_brand_title.mp4"
            brand.render_title_card(
                title=storyboard.title,
                subtitle="by OmniLink, for OmniLink agents",
                out_path=title_clip, fps=storyboard.fps,
            )
        if storyboard.brand.end_slate:
            end_clip = out_dir / "_brand_end.mp4"
            brand.render_end_slate(
                primary="OmniSim", secondary="by OmniLink",
                out_path=end_clip, fps=storyboard.fps,
            )

    sequence: list[Path] = []
    if title_clip:
        sequence.append(title_clip)
    sequence.extend(raw_shots)
    if end_clip:
        sequence.append(end_clip)

    deliverables: dict[str, Path] = {}
    for aspect in storyboard.aspect_ratios:
        if aspect not in ASPECT_DIMS:
            print(f"[edit] WARN: unknown aspect {aspect!r}; skipping")
            continue
        suffix = aspect.replace(":", "x").replace(".", "_")
        out_file = out_dir / f"{storyboard.slug}_{suffix}.mp4"
        try:
            concat_videos(
                inputs=sequence, out_path=out_file, look=look,
                aspect=aspect, fps=storyboard.fps,
                # Square + vertical: better to crop and fill than letterbox.
                crop_fill=aspect in ("1:1", "9:16", "4:5"),
            )
            if storyboard.brand.watermark:
                wm_file = out_file.with_name(out_file.stem + "_wm.mp4")
                brand.apply_watermark(out_file, wm_file)
                out_file = wm_file
            deliverables[aspect] = out_file
        except Exception as exc:
            print(f"[edit] {aspect} failed: {exc}")
    return deliverables
