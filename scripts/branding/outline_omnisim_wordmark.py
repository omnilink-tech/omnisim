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
"""Write the OmniSim wordmark into the brand SVGs as outlines.

The wordmark is "OmniSim" in Sora SemiBold (600): "Omni" in ink, "Sim" in
mimosa. It is shaped with HarfBuzz (so Sora's own kerning applies) and drawn
as SVG paths, so every SVG renders the same whether or not Sora is installed.

Each target SVG carries the wordmark between ``<!-- wordmark:begin -->`` and
``<!-- wordmark:end -->``; the first run replaces the old ``<text>`` element
that spelled OMNI/SIM, later runs replace the marked block. The orb dots are
not touched (``render_omnisim_orb.py --regenerate-svg`` owns those).

Usage:
  python scripts/branding/outline_omnisim_wordmark.py
"""

from __future__ import annotations

import io
import re
from pathlib import Path

import uharfbuzz as hb
from fontTools.pens.svgPathPen import SVGPathPen
from fontTools.ttLib import TTFont
from fontTools.varLib import instancer

REPO = Path(__file__).resolve().parents[2]
BRAND = REPO / "resources" / "branding" / "omnisim"
FONT = BRAND / "fonts" / "Sora.ttf"
WEIGHT = 600
INK, MIMOSA, BG = "#F6F4EF", "#F6E905", "#0A0A06"

# file -> (x, baseline y, cap height in viewBox units, anchor, fills, right padding or None)
# A right padding refits the viewBox width to the new wordmark; None keeps it.
# Cap heights match the cap height the Space Grotesk caps had in each file.
TARGETS = {
    "wordmark.svg": (0, 125, 91, "start", (INK, MIMOSA), 4),
    "wordmark_mono_light.svg": (0, 125, 91, "start", (INK, INK), 4),
    "wordmark_mono_dark.svg": (0, 125, 91, "start", (BG, BG), 4),
    "lockup_horizontal.svg": (260, 160, 84, "start", (INK, MIMOSA), 120),
    "lockup_stacked.svg": (360, 560, 67, "middle", (INK, MIMOSA), None),
    "lockup_omnilink.svg": (620, 120, 45, "start", (INK, MIMOSA), 80),
}
BEGIN, END = "<!-- wordmark:begin -->", "<!-- wordmark:end -->"
OLD_TEXT = re.compile(r"[ \t]*<text\b[^>]*>(?:(?!</text>).)*(?:OMNI|OMNISIM)(?:(?!</text>).)*</text>\n?", re.S)
MARKED = re.compile(re.escape(BEGIN) + r".*?" + re.escape(END) + r"\n?", re.S)


def _shape(word: str):
    blob = hb.Blob.from_file_path(str(FONT))
    font = hb.Font(hb.Face(blob))
    font.set_variations({"wght": WEIGHT})
    buf = hb.Buffer()
    buf.add_str(word)
    buf.guess_segment_properties()
    hb.shape(font, buf, {"kern": True, "liga": False})
    return list(zip(buf.glyph_infos, buf.glyph_positions))


def _instance() -> TTFont:
    static = instancer.instantiateVariableFont(TTFont(str(FONT)), {"wght": WEIGHT})
    data = io.BytesIO()
    static.save(data)
    data.seek(0)
    return TTFont(data)


def wordmark_group(x: float, baseline: float, cap: float, anchor: str, fills) -> tuple[str, float]:
    """Return (svg group, width) for "OmniSim" with its cap height = ``cap``."""
    tt = _instance()
    order = tt.getGlyphOrder()
    glyphs = tt.getGlyphSet()
    scale = cap / tt["OS/2"].sCapHeight
    pen_x = 0
    paths = []
    for i, (info, pos) in enumerate(_shape("OmniSim")):
        pen = SVGPathPen(glyphs)
        glyphs[order[info.codepoint]].draw(pen)
        d = pen.getCommands()
        if d:
            fill = fills[0] if i < 4 else fills[1]
            gx = pen_x + pos.x_offset
            paths.append(f'    <path fill="{fill}" transform="translate({gx} {pos.y_offset})" d="{d}"/>')
        pen_x += pos.x_advance
    width = pen_x * scale
    x0 = x - width / 2 if anchor == "middle" else x
    group = (f"{BEGIN}\n"
             f'  <g aria-hidden="true" transform="translate({x0:.2f} {baseline}) scale({scale:.6f} {-scale:.6f})">\n'
             + "\n".join(paths) + "\n  </g>\n" + f"{END}\n")
    return group, width


def main() -> None:
    for name, (x, y, cap, anchor, fills, pad) in TARGETS.items():
        path = BRAND / "svg" / name
        svg = path.read_text(encoding="utf-8")
        group, width = wordmark_group(x, y, cap, anchor, fills)
        if MARKED.search(svg):
            svg = MARKED.sub(lambda _m: group, svg, count=1)
        else:
            svg, n = OLD_TEXT.subn(lambda _m: group, svg, count=1)
            if not n:
                raise SystemExit(f"{name}: no OMNI/SIM <text> and no wordmark markers")
        if pad is not None:
            right = x + width + pad
            svg = re.sub(r'viewBox="0 0 [\d.]+ ([\d.]+)"', lambda m: f'viewBox="0 0 {right:.0f} {m.group(1)}"', svg, count=1)
        svg = svg.replace('aria-label="OMNISIM"', 'aria-label="OmniSim"')
        svg = re.sub(r"<title>OMNISIM wordmark", "<title>OmniSim wordmark", svg)
        path.write_text(svg, encoding="utf-8", newline="\n")
        print(f"{name}: wordmark {width:.1f} wide")


if __name__ == "__main__":
    main()
