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

Per the OmniLink + OmniSim brand guide ([resources/branding/omnisim/BRAND.md][1],
canonical, approved 2026-11-16): near-black canvas, ink text, one mimosa
accent. Titles and names are Sora SemiBold, captions and subtitles Inter
(the static OFL files in `resources/branding/omnisim/fonts/static/`).
Generated on the fly via ffmpeg `color` source + `drawtext` + `overlay`.

Names are never typed: the default end slate is the outlined co-lockup
`png/lockup_omnilink.png` ("OmniSim, by OmniLink"), an "OmniSim" slate
with another line is the outlined OmniSim lockup `png/lockup_horizontal.png`,
and the "OmniSim" watermark is the outlined wordmark `wordmark/wordmark.png`.
A lockup is sized so its sphere stays above 128 px (BRAND.md §8: the orb
above 128 px, the glyph at 128 and below). Users with custom brand cards
can drop a PNG at `assets/brand/title_card.png` and `end_slate.png` and
we'll use those instead.

Palette (canonical, from BRAND.md §6):
  bg:       #0A0A06   (near-black canvas)
  ink:      #F6F4EF   (titles and text)
  ink-mute: #B4B3A4   (secondary text)
  mimosa:   #F6E905   (the one accent — marks, the second word of a name, highlights)

Retired here: Space Grotesk (the old body face) and mimosa title text.

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
    "mute": "0xb4b3a4",
    "mimosa": "0xf6e905",
}

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
ASSETS_DIR = REPO_ROOT / "assets" / "brand"
BRAND_DIR = REPO_ROOT / "resources" / "branding" / "omnisim"
# scripts/branding/render_omnisim_wordmark.py; cropped to the letters, transparent.
WORDMARK_PNG = BRAND_DIR / "wordmark" / "wordmark.png"          # "OmniSim"
# Outlined renders (scripts/branding/render_brand_system.py --png), transparent, padded.
LOCKUP_PNG = BRAND_DIR / "png" / "lockup_horizontal.png"        # orb + "OmniSim"
CO_LOCKUP_PNG = BRAND_DIR / "png" / "lockup_omnilink.png"       # "OmniSim, by OmniLink"
WORDMARK_TEXT = "OmniSim"
CO_LOCKUP_TEXT = "by OmniLink"
# BRAND.md §5: Sora SemiBold for names and titles, Inter for captions and body.
TITLE_FONT = BRAND_DIR / "fonts" / "static" / "Sora-SemiBold.ttf"
BODY_FONT = BRAND_DIR / "fonts" / "static" / "Inter-Regular.ttf"
# In the lockup PNGs the sphere is 166 of the 240-unit frame height (BRAND.md §3). Scale a lockup so the
# sphere is at least this tall: the orb is drawn above 128 px, the glyph at 128 and below (BRAND.md §8).
MIN_ORB_PX = 136


def _font_opt(font: Path = BODY_FONT) -> str:
    """`fontfile=...:` for drawtext, or "" (ffmpeg's default face) if absent."""
    if not font.exists():
        return ""
    return "fontfile='" + font.as_posix().replace(":", "\\:") + "':"


def _lockup_height(art: Path, width: int, height: int) -> int:
    """Output height of a lockup PNG (its full padded frame) on a `width` x `height` card: about a sixth of
    the frame and never so small that the orb would drop under the glyph threshold, but always inside 90 %
    of the width (on a frame narrower than ~1780 px the co-lockup's orb then falls under 128 px)."""
    from PIL import Image
    with Image.open(art) as im:
        aspect = im.width / im.height
    want = max(round(height / 6), round(MIN_ORB_PX * 240 / 166))
    return min(want, int(0.9 * width / aspect)) // 2 * 2


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
    """Generate a near-black title card video clip: the title in Sora
    SemiBold (ink), the subtitle in Inter (ink-mute). drawtext is
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
    # Two stacked drawtext layers — title in ink (Sora), subtitle in ink-mute (Inter).
    vf = (
        f"drawtext={_font_opt(TITLE_FONT)}text='{title_e}':"
        f"fontcolor={PALETTE['cream']}:fontsize=h/12:x=(w-text_w)/2:y=h/2-h/12,"
        f"drawtext={_font_opt()}text='{subtitle_e}':"
        f"fontcolor={PALETTE['mute']}:fontsize=h/28:x=(w-text_w)/2:y=h/2+h/40"
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
    """End slate — centred on near-black. The default ("OmniSim", "by OmniLink")
    is the co-lockup image `png/lockup_omnilink.png`, which reads "OmniSim,
    by OmniLink" (BRAND.md §3), with no text typed. An "OmniSim" primary with
    any other secondary is the OmniSim lockup image with the secondary line in
    Inter (ink-mute) under it. Anything else falls back to the title card."""
    if out_path is None:
        raise ValueError("out_path required for render_end_slate")
    co = primary == WORDMARK_TEXT and secondary == CO_LOCKUP_TEXT
    art = CO_LOCKUP_PNG if co else LOCKUP_PNG
    if primary == WORDMARK_TEXT and art.exists():
        out_path.parent.mkdir(parents=True, exist_ok=True)
        lh = _lockup_height(art, width, height)
        if co:
            # The co-lockup alone, centred.
            graph = (
                f"[1:v]scale=-1:{lh}[wm];"
                "[0:v][wm]overlay=x=(W-w)/2:y=(H-h)/2:shortest=1,"
                "scale=trunc(iw/2)*2:trunc(ih/2)*2"
            )
        else:
            sub = secondary.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")
            # The lockup sits above the centre line, the secondary line below it, as on the title card.
            graph = (
                f"[1:v]scale=-1:{lh}[wm];"
                "[0:v][wm]overlay=x=(W-w)/2:y=H/2-h:shortest=1,"
                f"drawtext={_font_opt()}text='{sub}':fontcolor={PALETTE['mute']}:fontsize=h/28:"
                "x=(w-text_w)/2:y=h/2+h/40,"
                "scale=trunc(iw/2)*2:trunc(ih/2)*2"
            )
        cmd = [
            _ffmpeg(), "-y",
            "-f", "lavfi",
            "-i", f"color=c={PALETTE['black']}:s={width}x{height}:r={fps}:d={duration_s}",
            "-loop", "1", "-i", str(art),
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
    outlined wordmark image for "OmniSim" (no mark: at watermark size the
    orb would be under the glyph threshold), Inter ink text otherwise.
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
            f"fontcolor={PALETTE['cream']}@0.7:"
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
