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

"""Brand cards — title intro + end slate + (optional) watermark.

Per the OmniSim brand book ([resources/branding/omnisim/BRAND.md][1]):
near-black background, ink + mimosa accents. Generated on the fly via
ffmpeg `color` source + `drawtext`. Wherever the card says "OmniSim" —
the end slate and the watermark — it overlays the wordmark PNG
(`resources/branding/omnisim/wordmark/`, Sora SemiBold, "Omni" ink and
"Sim" mimosa) instead of typing the name: the brand never re-types the
wordmark in a fallback font. Users with custom brand cards can drop a
PNG at `assets/brand/title_card.png` and `end_slate.png` and we'll use
those instead.

OmniSim palette (canonical, from BRAND.md):
  bg:     #0A0A06   (near-black canvas)
  ink:    #F6F4EF   (body text)
  mimosa: #F6E905   (accent — marks, CTAs, title text)

[1]: ../../resources/branding/omnisim/BRAND.md
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

from .grade import safe_chroma_pad


PALETTE = {
    "black": "0x0a0a06",
    "cream": "0xf6f4ef",
    "mimosa": "0xf6e905",
}

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
ASSETS_DIR = REPO_ROOT / "assets" / "brand"
# scripts/branding/render_omnisim_wordmark.py; cropped to the letters, transparent.
WORDMARK_PNG = REPO_ROOT / "resources" / "branding" / "omnisim" / "wordmark" / "wordmark.png"
WORDMARK_TEXT = "OmniSim"
# The brand's display + body face, for every line that is not the wordmark.
BODY_FONT = REPO_ROOT / "resources" / "branding" / "omnisim" / "fonts" / "SpaceGrotesk.ttf"


def _font_opt() -> str:
    """`fontfile=...:` for drawtext, or "" (ffmpeg's default face) if absent."""
    if not BODY_FONT.exists():
        return ""
    return "fontfile='" + BODY_FONT.as_posix().replace(":", "\\:") + "':"


def _ffmpeg() -> str:
    p = shutil.which("ffmpeg")
    if p:
        return p
    fallback = "C:/ffmpeg/bin/ffmpeg.exe"
    if Path(fallback).exists():
        return fallback
    raise RuntimeError("ffmpeg not found on PATH; install it or set up C:/ffmpeg")


def render_title_card(
    title: str, subtitle: str, out_path: Path,
    duration_s: float = 1.8, width: int = 1920, height: int = 1080,
    fps: int = 30,
) -> Path:
    """Generate a black + mimosa title card video clip. drawtext is
    available in standard ffmpeg builds; if your build lacks the font,
    fall back to a plain color clip with the title encoded in metadata."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    # Escape colons and special chars in drawtext arg.
    def esc(s: str) -> str:
        return (s.replace("\\", "\\\\")
                 .replace(":", "\\:")
                 .replace("'", "\\'"))
    title_e = esc(title)
    subtitle_e = esc(subtitle)
    # Two stacked drawtext layers — title in mimosa, subtitle in cream.
    vf = (
        f"drawtext={_font_opt()}text='{title_e}':"
        f"fontcolor={PALETTE['mimosa']}:fontsize=h/12:x=(w-text_w)/2:y=h/2-h/12,"
        f"drawtext={_font_opt()}text='{subtitle_e}':"
        f"fontcolor={PALETTE['cream']}:fontsize=h/28:x=(w-text_w)/2:y=h/2+h/40"
    )
    cmd = [
        _ffmpeg(), "-y",
        "-f", "lavfi",
        "-i", f"color=c={PALETTE['black']}:s={width}x{height}:r={fps}:d={duration_s}",
        "-vf", safe_chroma_pad(vf),
        "-c:v", "libx264", "-preset", "fast", "-crf", "18",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(out_path),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if res.returncode != 0:
        # drawtext can fail when no default font is found. Fall back to a
        # plain color clip so the pipeline doesn't die over brand polish.
        cmd_plain = [
            _ffmpeg(), "-y",
            "-f", "lavfi",
            "-i", f"color=c={PALETTE['black']}:s={width}x{height}:r={fps}:d={duration_s}",
            "-vf", safe_chroma_pad(""),
            "-c:v", "libx264", "-preset", "fast", "-crf", "18",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            "-metadata", f"title={title}", "-metadata", f"comment={subtitle}",
            str(out_path),
        ]
        subprocess.run(cmd_plain, capture_output=True, text=True, timeout=120, check=True)
    return out_path


def render_end_slate(
    primary: str = "OmniSim", secondary: str = "by OmniLink",
    out_path: Path | None = None,
    duration_s: float = 2.5, width: int = 1920, height: int = 1080, fps: int = 30,
) -> Path:
    """End slate — same recipe as title card but slower and centered. Reads
    'OmniSim, by OmniLink' by default per the brand memory; an "OmniSim"
    primary is the wordmark image, not text."""
    if out_path is None:
        raise ValueError("out_path required for render_end_slate")
    if primary == WORDMARK_TEXT and WORDMARK_PNG.exists():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        sub = secondary.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")
        # The wordmark's letters are ~h/11 tall; it sits above the centre
        # line, the secondary line below it, as on the title card.
        graph = (
            f"[1:v]scale=-1:{round(height / 9)}[wm];"
            "[0:v][wm]overlay=x=(W-w)/2:y=H/2-h-H/60:shortest=1,"
            f"drawtext={_font_opt()}text='{sub}':fontcolor={PALETTE['cream']}:fontsize=h/28:"
            "x=(w-text_w)/2:y=h/2+h/40,"
            "scale=trunc(iw/2)*2:trunc(ih/2)*2"
        )
        cmd = [
            _ffmpeg(), "-y",
            "-f", "lavfi",
            "-i", f"color=c={PALETTE['black']}:s={width}x{height}:r={fps}:d={duration_s}",
            "-loop", "1", "-i", str(WORDMARK_PNG),
            "-filter_complex", graph,
            "-c:v", "libx264", "-preset", "fast", "-crf", "18",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(out_path),
        ]
        if subprocess.run(cmd, capture_output=True, text=True, timeout=120).returncode == 0:
            return out_path
    return render_title_card(
        title=primary, subtitle=secondary, out_path=out_path,
        duration_s=duration_s, width=width, height=height, fps=fps,
    )


def apply_watermark(
    in_path: Path, out_path: Path, text: str = "OmniSim",
) -> Path:
    """Burn a small watermark in the bottom-right, at 70 % opacity: the
    wordmark image for "OmniSim", mimosa text otherwise.
    Used when `brand.watermark = true` in the storyboard."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    def esc(s: str) -> str:
        return s.replace(":", "\\:").replace("'", "\\'")
    if text == WORDMARK_TEXT and WORDMARK_PNG.exists():
        graph = (
            "[1:v][0:v]scale2ref=w=oh*mdar:h=main_h/32[wm][base];"
            "[wm]format=rgba,colorchannelmixer=aa=0.7[wma];"
            "[base][wma]overlay=x=W-w-W/40:y=H-h-H/40,"
            "scale=trunc(iw/2)*2:trunc(ih/2)*2"
        )
        cmd = [
            _ffmpeg(), "-y", "-i", str(in_path), "-i", str(WORDMARK_PNG),
            "-filter_complex", graph, "-map", "0:a?",
            "-c:v", "libx264", "-preset", "fast", "-crf", "18",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(out_path),
        ]
    else:
        vf = (
            f"drawtext={_font_opt()}text='{esc(text)}':"
            f"fontcolor={PALETTE['mimosa']}@0.7:"
            "fontsize=h/40:"
            "x=w-text_w-w/40:y=h-text_h-h/40,"
            f"{safe_chroma_pad('')}"
        )
        cmd = [
            _ffmpeg(), "-y", "-i", str(in_path),
            "-vf", vf,
            "-c:v", "libx264", "-preset", "fast", "-crf", "18",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(out_path),
        ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    if res.returncode != 0:
        # drawtext font missing — fall back to a pass-through copy so the
        # pipeline continues. Watermark is cosmetic.
        cmd_pass = [
            _ffmpeg(), "-y", "-i", str(in_path),
            "-c", "copy", str(out_path),
        ]
        subprocess.run(cmd_pass, capture_output=True, text=True, timeout=120, check=True)
    return out_path
