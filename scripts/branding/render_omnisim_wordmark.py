#!/usr/bin/env python3
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
"""Render the OmniSim wordmark SVGs to PNG.

The wordmark ("OmniSim" in Sora SemiBold, "Omni" ink, "Sim" mimosa) lives in
``resources/branding/omnisim/svg/wordmark*.svg`` as outlines, written by
``outline_omnisim_wordmark.py``. Surfaces that cannot draw SVG paths — the Qt
splash screen, ffmpeg brand cards — use these PNGs instead of re-typing the
name in a fallback font (BRAND.md, Typography).

Each PNG is cropped to the letters, on a transparent background.

Outputs:
  resources/branding/omnisim/wordmark/wordmark.png             (ink + mimosa)
  resources/branding/omnisim/wordmark/wordmark_mono_light.png  (ink)
  resources/branding/omnisim/wordmark/wordmark_mono_dark.png   (near-black)
  resources/images/splash_wordmark.png                          (the GUI splash)

Usage:
  python scripts/branding/render_omnisim_wordmark.py
"""

from __future__ import annotations

import io
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import PathPatch  # noqa: E402
from matplotlib.path import Path as MplPath  # noqa: E402
from PIL import Image  # noqa: E402
from svgelements import SVG, Close, Move  # noqa: E402
from svgelements import Path as SvgPath  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
SVG_DIR = REPO / "resources" / "branding" / "omnisim" / "svg"
OUT_DIR = REPO / "resources" / "branding" / "omnisim" / "wordmark"

# (source SVG, output PNG, cap height in pixels). The splash draws its copy at
# half size (OmSplashScreen.cpp), so it is rendered at 2x for hi-DPI screens.
TARGETS = [
    ("wordmark.svg", OUT_DIR / "wordmark.png", 400),
    ("wordmark_mono_light.svg", OUT_DIR / "wordmark_mono_light.png", 400),
    ("wordmark_mono_dark.svg", OUT_DIR / "wordmark_mono_dark.png", 400),
    ("wordmark.svg", REPO / "resources" / "images" / "splash_wordmark.png", 120),
]
CAP_HEIGHT_UNITS = 91  # outline_omnisim_wordmark.py TARGETS, the wordmark*.svg files
SAMPLES_PER_SEGMENT = 24
PAD = 0.04  # of the cropped height, each side


def _mpl_path(path: SvgPath) -> MplPath:
    """Flatten one SVG path (curves sampled) into a matplotlib compound path."""
    verts, codes = [], []
    for seg in path:
        if isinstance(seg, Move):
            verts.append((seg.end.x, seg.end.y))
            codes.append(MplPath.MOVETO)
        elif isinstance(seg, Close):
            verts.append(verts[-1])
            codes.append(MplPath.CLOSEPOLY)
        elif seg.start is not None:
            n = 1 if type(seg).__name__ == "Line" else SAMPLES_PER_SEGMENT
            for i in range(1, n + 1):
                p = seg.point(i / n)
                verts.append((p.x, p.y))
                codes.append(MplPath.LINETO)
    return MplPath(verts, codes)


def render(svg_path: Path, cap_px: int) -> Image.Image:
    doc = SVG.parse(str(svg_path), reify=True)
    vb = doc.viewbox
    scale = cap_px / CAP_HEIGHT_UNITS
    w, h = round(vb.width * scale), round(vb.height * scale)
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(vb.x, vb.x + vb.width)
    ax.set_ylim(vb.y + vb.height, vb.y)
    ax.axis("off")
    for el in doc.elements():
        if isinstance(el, SvgPath) and el.fill is not None and el.fill.value is not None:
            ax.add_patch(PathPatch(_mpl_path(el), facecolor=el.fill.hexrgb, edgecolor="none", lw=0))
    buf = io.BytesIO()
    fig.savefig(buf, format="png", transparent=True, dpi=100)
    plt.close(fig)
    img = Image.open(buf).convert("RGBA")
    x0, y0, x1, y1 = img.getchannel("A").getbbox()
    pad = round((y1 - y0) * PAD)
    return img.crop((max(0, x0 - pad), max(0, y0 - pad), min(img.width, x1 + pad), min(img.height, y1 + pad)))


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, out, cap_px in TARGETS:
        img = render(SVG_DIR / name, cap_px)
        img.save(out)
        print(f"  wrote {out.relative_to(REPO)} ({img.width}×{img.height})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
