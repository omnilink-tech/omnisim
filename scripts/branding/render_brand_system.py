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
"""Render the OmniLink half of the unified brand, built exactly like the OmniSim half.

The brand guide is ``resources/branding/omnisim/BRAND.md``. Two marks, one family:
the circuit sphere is OmniLink, the particle orb is OmniSim. Both names are set
the same way: Sora SemiBold (600), mixed case, "Omni" in ink and the second word
in mimosa, outlined so no font is needed to render them.

Writes into ``resources/branding/omnisim/``:

  svg/omnilink_mark.svg          the circuit sphere, tight frame (no padding)
  svg/omnilink_mark_ink.svg      the same in near-black, for light paper
  svg/omnilink_wordmark.svg      "OmniLink" (+ _mono_light / _mono_dark)
  svg/omnilink_lockup.svg        mark + wordmark, the same geometry as lockup_horizontal.svg
  svg/lockup_omnilink.svg        "OmniSim, by OmniLink": both lockups, a hairline between
  fonts/static/*.ttf             static Sora 600 and Inter 400/500/600 (Qt, PDFs)
  fonts/web/*.woff2              the same as WOFF2 (the website)

and copies the OmniLink mark over ``resources/branding/omnilink/svg/mark_yellow.svg``'s
padded frame as ``mark_yellow_tight.svg``. PNGs are rendered with headless Chrome
(``--png``), since no SVG rasteriser ships in the Python environment.

Usage:
  python scripts/branding/render_brand_system.py [--png]
"""

from __future__ import annotations

import io
import re
import subprocess
import sys
from pathlib import Path

from fontTools.ttLib import TTFont
from fontTools.varLib import instancer

sys.path.insert(0, str(Path(__file__).resolve().parent))
import outline_omnisim_wordmark as ow  # noqa: E402  (shaping + outlining helpers)

REPO = Path(__file__).resolve().parents[2]
BRAND = REPO / "resources" / "branding" / "omnisim"
SVG = BRAND / "svg"
FONTS = BRAND / "fonts"
OMNILINK_SVG = REPO / "resources" / "branding" / "omnilink" / "svg"
INK, MIMOSA, BG = ow.INK, ow.MIMOSA, ow.BG

# The circuit sphere's own artwork: a 1080 square whose mark spans about 578 x 588,
# centred on (540, 540). Measured on the 2251 px raster: x 250.9-829.1, y 245.6-833.9.
MARK_CENTRE, MARK_DIAMETER = (540.0, 539.8), 588.0
# lockup_horizontal.svg: the orb's sphere is r = 100 at scale 0.83 -> 166 units, centred (120, 120);
# the wordmark starts at x 260, baseline 160, cap height 84. The OmniLink lockup copies it.
LOCK_DIAM, LOCK_CX, LOCK_CY, WM_X, WM_BASE, WM_CAP, WM_PAD = 166.0, 120.0, 120.0, 260, 160, 84, 120


def mark_paths() -> str:
    svg = (OMNILINK_SVG / "mark_yellow.svg").read_text(encoding="utf-8")
    paths = re.findall(r"<path\b[^>]*\bd=\"[^\"]+\"[^>]*/?>", svg)
    if not paths:
        raise SystemExit("mark_yellow.svg: no <path>")
    return "\n".join(re.sub(r'\s*class="[^"]*"', "", p) for p in paths)


def word_group(word: str, x: float, baseline: float, cap: float, fills) -> tuple[str, float]:
    """Outline ``word`` in Sora 600; the first four letters ("Omni") take fills[0]."""
    tt = ow._instance()
    order, glyphs = tt.getGlyphOrder(), tt.getGlyphSet()
    scale = cap / tt["OS/2"].sCapHeight
    pen_x, paths = 0, []
    for i, (info, pos) in enumerate(ow._shape_word(word) if hasattr(ow, "_shape_word") else _shape(word)):
        pen = ow.SVGPathPen(glyphs)
        glyphs[order[info.codepoint]].draw(pen)
        d = pen.getCommands()
        if d:
            paths.append(f'    <path fill="{fills[0] if i < 4 else fills[1]}" '
                         f'transform="translate({pen_x + pos.x_offset} {pos.y_offset})" d="{d}"/>')
        pen_x += pos.x_advance
    g = (f'  <g aria-hidden="true" transform="translate({x:.2f} {baseline}) scale({scale:.6f} {-scale:.6f})">\n'
         + "\n".join(paths) + "\n  </g>\n")
    return g, pen_x * scale


def _shape(word: str):
    blob = ow.hb.Blob.from_file_path(str(ow.FONT))
    font = ow.hb.Font(ow.hb.Face(blob))
    font.set_variations({"wght": ow.WEIGHT})
    buf = ow.hb.Buffer()
    buf.add_str(word)
    buf.guess_segment_properties()
    ow.hb.shape(font, buf, {"kern": True, "liga": False})
    return list(zip(buf.glyph_infos, buf.glyph_positions))


def mark_group(cx: float, cy: float, diameter: float, fill: str) -> str:
    s = diameter / MARK_DIAMETER
    body = mark_paths().replace("<path ", f'<path fill="{fill}" ')
    return (f'  <g aria-hidden="true" transform="translate({cx} {cy}) scale({s:.5f}) '
            f'translate({-MARK_CENTRE[0]} {-MARK_CENTRE[1]})">\n{body}\n  </g>\n')


def write(name: str, body: str, view: str, label: str) -> None:
    (SVG / name).write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<!-- Generated by scripts/branding/render_brand_system.py. Brand guide: resources/branding/omnisim/BRAND.md -->\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{view}" role="img" aria-label="{label}">\n'
        f"  <title>{label}</title>\n{body}</svg>\n", encoding="utf-8", newline="\n")
    print("wrote", name)


def orb_block() -> str:
    """The orb's dots exactly as lockup_horizontal.svg draws them."""
    svg = (SVG / "lockup_horizontal.svg").read_text(encoding="utf-8")
    m = re.search(r'(<g transform="translate\(120 120\) scale\(0\.83\)">.*?</g>\s*</g>)', svg, re.S)
    if not m:
        raise SystemExit("lockup_horizontal.svg: orb group not found")
    return m.group(1)


def main(png: bool) -> None:
    half = MARK_DIAMETER / 2 + 6
    view = f"{MARK_CENTRE[0] - half:.1f} {MARK_CENTRE[1] - half:.1f} {2 * half:.1f} {2 * half:.1f}"
    raw = mark_paths()
    write("omnilink_mark.svg", raw.replace("<path ", f'<path fill="{MIMOSA}" ') + "\n", view, "OmniLink")
    write("omnilink_mark_ink.svg", raw.replace("<path ", f'<path fill="{BG}" ') + "\n", view, "OmniLink")
    (OMNILINK_SVG / "mark_yellow_tight.svg").write_text((SVG / "omnilink_mark.svg").read_text(encoding="utf-8"),
                                                       encoding="utf-8", newline="\n")

    for suffix, fills in (("", (INK, MIMOSA)), ("_mono_light", (INK, INK)), ("_mono_dark", (BG, BG))):
        g, w = word_group("OmniLink", 0, 125, 91, fills)
        write(f"omnilink_wordmark{suffix}.svg", g, f"0 0 {w + 4:.0f} 160", "OmniLink")

    g, w = word_group("OmniLink", WM_X, WM_BASE, WM_CAP, (INK, MIMOSA))
    lock_w = WM_X + w + WM_PAD
    write("omnilink_lockup.svg", mark_group(LOCK_CX, LOCK_CY, LOCK_DIAM, MIMOSA) + g, f"0 0 {lock_w:.0f} 240", "OmniLink")

    # Co-lockup: OmniSim lockup, hairline, OmniLink lockup, both at full weight.
    sim_w = float(re.search(r'viewBox="0 0 ([\d.]+) 240"', (SVG / "lockup_horizontal.svg").read_text()).group(1))
    gs, ws = word_group("OmniSim", WM_X, WM_BASE, WM_CAP, (INK, MIMOSA))
    gap = 70
    x2 = sim_w + gap
    body = (f'  {orb_block()}\n' + gs
            + f'  <rect x="{sim_w + gap / 2 - 1.5:.1f}" y="50" width="3" height="140" fill="#3A3A30"/>\n'
            + f'  <g transform="translate({x2:.1f} 0)">\n'
            + mark_group(LOCK_CX, LOCK_CY, LOCK_DIAM, MIMOSA)
            + word_group("OmniLink", WM_X, WM_BASE, WM_CAP, (INK, MIMOSA))[0] + "  </g>\n")
    write("lockup_omnilink.svg", body, f"0 0 {x2 + lock_w:.0f} 240", "OmniSim, by OmniLink")

    # Fonts: static instances for Qt and PDFs, WOFF2 for the web.
    (FONTS / "static").mkdir(exist_ok=True)
    (FONTS / "web").mkdir(exist_ok=True)
    for src, axes, stem in (("Sora.ttf", {"wght": 600}, "Sora-SemiBold"),
                            ("Inter.ttf", {"wght": 400, "opsz": 14}, "Inter-Regular"),
                            ("Inter.ttf", {"wght": 500, "opsz": 14}, "Inter-Medium"),
                            ("Inter.ttf", {"wght": 600, "opsz": 14}, "Inter-SemiBold")):
        static = instancer.instantiateVariableFont(TTFont(str(FONTS / src)), axes, updateFontNames=True)
        static.save(str(FONTS / "static" / f"{stem}.ttf"))
        static.flavor = "woff2"
        static.save(str(FONTS / "web" / f"{stem}.woff2"))
        print("font", stem)

    if png:
        render_pngs()


def render_pngs() -> None:
    chrome = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
    out = BRAND / "png"
    out.mkdir(exist_ok=True)
    for name, height in (("omnilink_mark", 512), ("omnilink_lockup", 240), ("lockup_horizontal", 240),
                         ("lockup_omnilink", 240), ("omnilink_wordmark", 160), ("wordmark", 160)):
        svg = SVG / f"{name}.svg"
        vb = [float(v) for v in re.search(r'viewBox="([^"]+)"', svg.read_text()).group(1).split()]
        scale = 4 if height <= 240 else 1
        w, h = int(round(vb[2] / vb[3] * height * scale)), int(height * scale)
        html = out / f".{name}.html"
        html.write_text(f'<html><body style="margin:0;background:transparent">'
                        f'<img src="{svg.as_uri()}" style="width:{w}px;height:{h}px;display:block"></body></html>')
        subprocess.run([chrome, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                        "--default-background-color=00000000", f"--window-size={w},{h}",
                        f"--screenshot={out / (name + '.png')}", html.as_uri()], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        html.unlink()
        print("png", name, w, h)


if __name__ == "__main__":
    main("--png" in sys.argv)
