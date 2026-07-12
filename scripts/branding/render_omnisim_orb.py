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

import re
from pathlib import Path
from xml.etree import ElementTree as ET

from PIL import Image, ImageDraw

REPO = Path(__file__).resolve().parents[2]
BRAND_DIR = REPO / "resources" / "branding" / "omnisim"
SVG_DIR = BRAND_DIR / "svg"
ORB_DIR = BRAND_DIR / "orb"
GLYPH_DIR = BRAND_DIR / "glyph"
PREVIEW_DIR = BRAND_DIR / "preview"

# Sphere occupies ±100 of the ±120 viewBox — leave a small margin so the
# halo dots near the rim don't clip when rendered to a square canvas.
VIEWBOX_HALF = 120.0


def _parse_svg_circles(svg_path: Path) -> list[tuple[float, float, float, tuple[int, int, int]]]:
    """Pull (cx, cy, r, rgb) out of every <circle> in an SVG file."""
    tree = ET.parse(svg_path)
    root = tree.getroot()
    ns = {"svg": "http://www.w3.org/2000/svg"}
    circles: list[tuple[float, float, float, tuple[int, int, int]]] = []
    # Match plain <circle> and namespaced — the brand SVGs use the default ns.
    for circle in root.iter():
        if not circle.tag.endswith("circle"):
            continue
        cx = float(circle.attrib.get("cx", "0"))
        cy = float(circle.attrib.get("cy", "0"))
        r = float(circle.attrib.get("r", "1"))
        fill = circle.attrib.get("fill", "#F6E905")
        rgb = _hex_to_rgb(fill)
        circles.append((cx, cy, r, rgb))
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
    circles = _parse_svg_circles(svg_path)
    img = Image.new("RGBA", (resolution, resolution), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img, "RGBA")

    # The SVG uses viewBox -120..120, so 1 unit = resolution / 240 pixels.
    # Render at 4× then Lanczos down — gives clean anti-aliased dots without
    # depending on cairosvg.
    super_res = resolution * 4
    super_img = Image.new("RGBA", (super_res, super_res), (0, 0, 0, 0))
    super_draw = ImageDraw.Draw(super_img, "RGBA")
    scale = super_res / (2 * VIEWBOX_HALF)
    cx0 = cy0 = super_res / 2

    # SVG y-axis goes down (same as PIL), so don't flip.
    for cx, cy, r, rgb in circles:
        x = cx0 + cx * scale
        y = cy0 + cy * scale
        rp = r * scale
        if rp < 0.5:
            # Sub-pixel dot at super-res — clamp so it shows up after downsample.
            rp = 0.5
        super_draw.ellipse((x - rp, y - rp, x + rp, y + rp), fill=rgb + (255,))

    return super_img.resize((resolution, resolution), Image.LANCZOS)


def main() -> int:
    for d in (ORB_DIR, GLYPH_DIR, PREVIEW_DIR):
        d.mkdir(parents=True, exist_ok=True)

    orb_svg = SVG_DIR / "orb.svg"
    glyph_svg = SVG_DIR / "glyph.svg"
    if not orb_svg.exists() or not glyph_svg.exists():
        print(f"error: missing SVG sources in {SVG_DIR.relative_to(REPO)}")
        return 1

    # Full orb — for sizes where the 340-dot pattern survives.
    orb_sizes = [
        (1024, ORB_DIR / "orb.png"),
        (512, ORB_DIR / "orb_512.png"),
        (256, ORB_DIR / "orb_256.png"),
        (128, ORB_DIR / "orb_128.png"),
        (64, ORB_DIR / "orb_64.png"),
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
        img = render_svg_to_png(glyph_svg, size)
        img.save(out)
        print(f"  wrote {out.relative_to(REPO)} ({size}×{size})")

    # Contact sheet preview for PRs — orb at 256 + glyph at 64/32/16 side-by-side
    # on the brand near-black surface.
    print("building preview sheet…")
    sheet_w, sheet_h = 720, 320
    sheet = Image.new("RGBA", (sheet_w, sheet_h), (10, 10, 6, 255))
    orb256 = render_svg_to_png(orb_svg, 256)
    sheet.paste(orb256, (32, 32), orb256)
    for i, sz in enumerate([64, 32, 16]):
        g = render_svg_to_png(glyph_svg, sz)
        sheet.paste(g, (340, 48 + i * 80 + (64 - sz) // 2), g)
    preview_path = PREVIEW_DIR / "orb_preview.png"
    sheet.save(preview_path)
    print(f"  wrote {preview_path.relative_to(REPO)}")
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
