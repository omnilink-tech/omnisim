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

"""Render the OmniSim brand mark (orb + glyph) to PNG.

Unlike ``render_omnilink_orb.py`` — which generates an OmniLink Fibonacci
lattice procedurally — this script rasterizes the **canonical SVG sources**
under ``resources/branding/omnisim/svg/`` so the PNG output is bit-for-bit
the same dot pattern that the brand book defines. The brand book is the
authority; PNGs are just rasterizations of it.

Outputs (full orb, ≥48 px):
  resources/branding/omnisim/orb/orb.png       (1024×1024 RGBA on transparent)
  resources/branding/omnisim/orb/orb_512.png   (512×512)
  resources/branding/omnisim/orb/orb_256.png   (256×256)
  resources/branding/omnisim/orb/orb_128.png   (128×128)
  resources/branding/omnisim/orb/orb_64.png    (64×64)
  resources/images/splash_orb.png              (640×640, the GUI splash screen)

Outputs (small-size glyph, ≤48 px — used below the dissolve threshold):
  resources/branding/omnisim/glyph/glyph_48.png
  resources/branding/omnisim/glyph/glyph_32.png
  resources/branding/omnisim/glyph/glyph_16.png

Plus a contact-sheet preview for PRs:
  resources/branding/omnisim/preview/orb_preview.png

The brand book's clear-space convention is 1× orb radius. We render the
sphere into 0.83 × canvas (matching the brand book's mark-stage layout)
so the dots — which extend to viewBox ±100 of a ±120 frame — sit
comfortably with breathing room.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from xml.etree import ElementTree as ET

from PIL import Image, ImageDraw, ImageFilter

REPO = Path(__file__).resolve().parents[2]
BRAND_DIR = REPO / "resources" / "branding" / "omnisim"
SVG_DIR = BRAND_DIR / "svg"
ORB_DIR = BRAND_DIR / "orb"
GLYPH_DIR = BRAND_DIR / "glyph"
PREVIEW_DIR = BRAND_DIR / "preview"
SPLASH_ORB = REPO / "resources" / "images" / "splash_orb.png"

# Sphere occupies ±100 of the ±120 viewBox — leave a small margin so the
# halo dots near the rim don't clip when rendered to a square canvas.
VIEWBOX_HALF = 120.0
SPHERE_R = 100.0

MIMOSA = (246, 233, 5)
SUNK = (6, 6, 4)

# The lattice: dots on latitude rows (alternate rows offset half a step), the
# sphere tilted so the rows read as arcs, lit from the upper left. Each dot's
# size follows its depth and its brightness follows the light, nothing is
# jittered — that is what makes the mark read as engineered, not as glitter.
# `rows` and the radius range are per mark: the glyph has fewer, larger dots
# so it survives at 32–128 px and keeps only the near side plus the rim
# (`z_cut`), since the far hemisphere is noise at that size; `floor` is the
# darkest a lit dot gets, raised on the small marks so the shadowed edge
# still closes the circle. The micro lattice is for 16–24 px icons only and
# lives here, not in an SVG.
ORB_LATTICE = {"rows": 14, "r_min": 1.2, "r_max": 5.0, "z_cut": -1.0, "floor": 0.55}
GLYPH_LATTICE = {"rows": 8, "r_min": 3.6, "r_max": 9.6, "z_cut": -0.25, "floor": 0.72}
MICRO_LATTICE = {"rows": 5, "r_min": 8.0, "r_max": 14.0, "z_cut": -0.2, "floor": 0.8}
_TILT = -0.38
_YAW = 0.5
_LIGHT = (-0.55, -0.6, 0.58)  # screen space (y down), toward the viewer

# App icons sit the mark on a near-black rounded tile (corners 21 % of the
# side, soft mimosa vignette upper-left), so the icon keeps an edge on any
# wallpaper; below this size the glyph replaces the full orb.
GLYPH_MAX_PX = 128
MICRO_MAX_PX = 24
TILE_MARK_FRAC = 0.80  # the ±120 viewBox spans this fraction of the tile


def _lattice_points(rows: int) -> list[tuple[float, float, float]]:
    pts = []
    for j in range(rows):
        lat = -math.pi / 2 + math.pi * (j + 0.5) / rows
        ring = max(1, round(2 * rows * math.cos(lat)))
        offset = 0.5 * (j % 2)
        for k in range(ring):
            lon = 2 * math.pi * (k + offset) / ring
            pts.append((math.cos(lat) * math.cos(lon), math.sin(lat), math.cos(lat) * math.sin(lon)))
    return pts


def lattice_circles(rows: int, r_min: float, r_max: float, z_cut: float, floor: float):
    """(cx, cy, r, rgb, opacity) per dot in viewBox units, far dots first."""
    ln = math.sqrt(sum(c * c for c in _LIGHT))
    light = tuple(c / ln for c in _LIGHT)
    projected = []
    for x, y, z in _lattice_points(rows):
        x, z = x * math.cos(_YAW) + z * math.sin(_YAW), -x * math.sin(_YAW) + z * math.cos(_YAW)
        y, z = y * math.cos(_TILT) - z * math.sin(_TILT), y * math.sin(_TILT) + z * math.cos(_TILT)
        projected.append((x, -y, z))  # flip to screen y (down)
    projected.sort(key=lambda p: p[2])
    circles = []
    for x, y, z in projected:
        if z < z_cut:
            continue
        depth = (z + 1) / 2
        r = r_min + (r_max - r_min) * depth ** 1.3
        if z < 0 and z_cut <= -1.0:
            rgb, opacity = MIMOSA, 0.10 + 0.60 * depth
        else:
            lam = max(0.0, x * light[0] + y * light[1] + z * light[2])
            rgb = tuple(int(c * (floor + (1 - floor) * lam)) for c in MIMOSA)
            opacity = min(1.0, floor + (1 - floor) * (0.35 + 0.65 * lam))
        circles.append((x * SPHERE_R, y * SPHERE_R, r, rgb, opacity))
    return circles


def _circles_block(circles, indent: str = "              ") -> str:
    lines = ['<g aria-hidden="true">']
    for cx, cy, r, rgb, opacity in circles:
        fill = "#%02x%02x%02x" % rgb
        op = "" if opacity >= 0.995 else f' fill-opacity="{opacity:.2f}"'
        lines.append(f'{indent}<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{r:.2f}" fill="{fill}"{op}></circle>')
    lines.append("            </g>")
    return "\n".join(lines)


# Every brand SVG that embeds a mark, and which mark it embeds. The mark is
# the one `<g aria-hidden="true">` circle block in each file.
SVG_MARKS = {
    "orb.svg": ORB_LATTICE,
    "lockup_horizontal.svg": ORB_LATTICE,
    "lockup_stacked.svg": ORB_LATTICE,
    "lockup_omnilink.svg": ORB_LATTICE,
    "glyph.svg": GLYPH_LATTICE,
    "favicon.svg": GLYPH_LATTICE,
}
_BLOCK_RE = re.compile(r'<g aria-hidden="true">.*?</g>', re.S)


def write_svgs() -> None:
    """Rewrite the mark inside every brand SVG from the lattice above."""
    for name, params in SVG_MARKS.items():
        path = SVG_DIR / name
        text = path.read_text(encoding="utf-8")
        block = _circles_block(lattice_circles(**params))
        new, n = _BLOCK_RE.subn(lambda _m: block, text, count=1)
        if n != 1:
            raise SystemExit(f"error: no mark block in {path.relative_to(REPO)}")
        path.write_text(new, encoding="utf-8", newline="\n")
        print(f"  wrote {path.relative_to(REPO)}")


def _parse_svg_circles(svg_path: Path) -> list[tuple[float, float, float, tuple[int, int, int], float]]:
    """Pull (cx, cy, r, rgb, opacity) out of every <circle> in an SVG file."""
    tree = ET.parse(svg_path)
    root = tree.getroot()
    circles: list[tuple[float, float, float, tuple[int, int, int], float]] = []
    # Match plain <circle> and namespaced — the brand SVGs use the default ns.
    for circle in root.iter():
        if not circle.tag.endswith("circle"):
            continue
        cx = float(circle.attrib.get("cx", "0"))
        cy = float(circle.attrib.get("cy", "0"))
        r = float(circle.attrib.get("r", "1"))
        fill = circle.attrib.get("fill", "#F6E905")
        rgb = _hex_to_rgb(fill)
        opacity = float(circle.attrib.get("fill-opacity", "1"))
        circles.append((cx, cy, r, rgb, opacity))
    return circles


def _hex_to_rgb(hex_color: str) -> tuple[int, int, int]:
    h = hex_color.strip().lstrip("#")
    if len(h) == 3:
        h = "".join(c * 2 for c in h)
    return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))


def render_svg_to_png(svg_path: Path, resolution: int) -> Image.Image:
    """Rasterize an OmniSim brand SVG (orb.svg or glyph.svg) to a square
    transparent PNG. The dot pattern is preserved exactly — only the canvas
    size changes."""
    return _render_circles(_parse_svg_circles(svg_path), resolution)


def _render_circles(circles, resolution: int) -> Image.Image:
    # The SVG uses viewBox -120..120, so 1 unit = resolution / 240 pixels.
    # Render at 4× then Lanczos down — gives clean anti-aliased dots without
    # depending on cairosvg.
    super_res = resolution * 4
    super_img = Image.new("RGBA", (super_res, super_res), (0, 0, 0, 0))
    scale = super_res / (2 * VIEWBOX_HALF)
    cx0 = cy0 = super_res / 2

    # SVG y-axis goes down (same as PIL), so don't flip. Each dot is
    # composited on its own layer so a far dot shows through a near one's
    # fill-opacity the way an SVG renderer would.
    for cx, cy, r, rgb, opacity in circles:
        x = cx0 + cx * scale
        y = cy0 + cy * scale
        rp = r * scale
        if rp < 0.5:
            # Sub-pixel dot at super-res — clamp so it shows up after downsample.
            rp = 0.5
        box = (int(x - rp) - 1, int(y - rp) - 1, int(x + rp) + 2, int(y + rp) + 2)
        dot = Image.new("RGBA", (box[2] - box[0], box[3] - box[1]), (0, 0, 0, 0))
        ImageDraw.Draw(dot).ellipse((x - rp - box[0], y - rp - box[1], x + rp - box[0], y + rp - box[1]),
                                    fill=rgb + (round(255 * opacity),))
        super_img.alpha_composite(dot, (box[0], box[1]))

    return super_img.resize((resolution, resolution), Image.LANCZOS)


def render_app_icon(resolution: int) -> Image.Image:
    """The mark on its rounded near-black tile — every shipped app icon.

    The glyph stands in for the orb at GLYPH_MAX_PX and below, the micro
    lattice for the glyph at MICRO_MAX_PX and below."""
    S = resolution * 4
    mask = Image.new("L", (S, S), 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, S - 1, S - 1), radius=round(S * 0.21), fill=255)
    bg = Image.new("RGBA", (S, S), SUNK + (255,))
    vignette = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    ImageDraw.Draw(vignette).ellipse((-S * 0.2, -S * 0.25, S * 0.8, S * 0.75), fill=(46, 44, 20, 255))
    bg.alpha_composite(vignette.filter(ImageFilter.GaussianBlur(S * 0.22)))
    tile = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    tile.paste(bg, (0, 0), mask)
    if resolution >= 32:
        # A hairline rim separates the tile from a dark wallpaper.
        ImageDraw.Draw(tile).rounded_rectangle((2, 2, S - 3, S - 3), radius=round(S * 0.21),
                                               outline=(255, 255, 255, 30), width=4)
    tile = tile.resize((resolution, resolution), Image.LANCZOS)
    mark_px = round(resolution * TILE_MARK_FRAC)
    if resolution <= MICRO_MAX_PX:
        mark = _render_circles(lattice_circles(**MICRO_LATTICE), mark_px)
    else:
        svg = SVG_DIR / ("glyph.svg" if resolution <= GLYPH_MAX_PX else "orb.svg")
        mark = render_svg_to_png(svg, mark_px)
    off = (resolution - mark_px) // 2
    tile.alpha_composite(mark, (off, off))
    return tile


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--regenerate-svg", action="store_true",
                        help="rewrite the mark inside every brand SVG from the lattice first")
    args = parser.parse_args()
    if args.regenerate_svg:
        print("regenerating the mark in the brand SVGs…")
        write_svgs()

    for d in (ORB_DIR, GLYPH_DIR, PREVIEW_DIR):
        d.mkdir(parents=True, exist_ok=True)

    orb_svg = SVG_DIR / "orb.svg"
    glyph_svg = SVG_DIR / "glyph.svg"
    if not orb_svg.exists() or not glyph_svg.exists():
        print(f"error: missing SVG sources in {SVG_DIR.relative_to(REPO)}")
        return 1

    # Full orb — for sizes where the 250-dot pattern survives.
    orb_sizes = [
        (1024, ORB_DIR / "orb.png"),
        (512, ORB_DIR / "orb_512.png"),
        (256, ORB_DIR / "orb_256.png"),
        (128, ORB_DIR / "orb_128.png"),
        (64, ORB_DIR / "orb_64.png"),
        # The GUI splash (OmSplashScreen.cpp) draws the orb at 320 logical px.
        # resources/branding/ is not packaged, so its copy lives with the other
        # GUI images, at 2x for hi-DPI screens.
        (640, SPLASH_ORB),
    ]
    print(f"rendering OmniSim orb from {orb_svg.relative_to(REPO)}…")
    for size, out in orb_sizes:
        img = render_svg_to_png(orb_svg, size)
        img.save(out)
        print(f"  wrote {out.relative_to(REPO)} ({size}×{size})")

    # Small-size glyph — below the dissolve threshold.
    glyph_sizes = [
        (48, GLYPH_DIR / "glyph_48.png"),
        (32, GLYPH_DIR / "glyph_32.png"),
        (16, GLYPH_DIR / "glyph_16.png"),
        (256, GLYPH_DIR / "glyph_256.png"),  # macOS app-icon source
    ]
    print(f"rendering OmniSim glyph from {glyph_svg.relative_to(REPO)}…")
    for size, out in glyph_sizes:
        if size <= MICRO_MAX_PX:
            img = _render_circles(lattice_circles(**MICRO_LATTICE), size)
        else:
            img = render_svg_to_png(glyph_svg, size)
        img.save(out)
        print(f"  wrote {out.relative_to(REPO)} ({size}×{size})")

    # Contact sheet preview for PRs — the bare orb and glyph on the brand
    # near-black surface, then the app icon at every size it ships at, on a
    # mid-grey desktop so the tile's edge shows.
    print("building preview sheet…")
    icon_sizes = [256, 128, 64, 48, 32, 24, 16]
    sheet_w = max(32 + 256 + 32 + 256 + 32, 32 + sum(s + 24 for s in icon_sizes))
    sheet = Image.new("RGBA", (sheet_w, 32 + 256 + 32 + 256 + 64), (10, 10, 6, 255))
    sheet.alpha_composite(render_svg_to_png(orb_svg, 256), (32, 32))
    sheet.alpha_composite(render_svg_to_png(glyph_svg, 256), (320, 32))
    ImageDraw.Draw(sheet).rectangle((0, 320, sheet_w, sheet.height), fill=(40, 40, 48, 255))
    x = 32
    for sz in icon_sizes:
        sheet.alpha_composite(render_app_icon(sz), (x, 352))
        x += sz + 24
    preview_path = PREVIEW_DIR / "orb_preview.png"
    sheet.save(preview_path)
    print(f"  wrote {preview_path.relative_to(REPO)}")
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
