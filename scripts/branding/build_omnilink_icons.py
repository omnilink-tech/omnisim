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

"""Regenerate the GUI's app-icon PNGs and Windows .ico from the OmniLink orb.

The OmniLink visual identity lives in ``resources/branding/omnilink/`` (see
``BRAND.md`` there). The canonical brand visual is the **orb** — the spherical
Fibonacci-lattice of Mimosa-yellow dots from the OmniLink home page.

The C++ GUI loads icons from ``images:omnisim.png`` /
``icons/core/omnisim*.png`` / ``src/omnisim/gui/omnisim.ico``. This script
renders the OmniLink orb at each target icon size and writes it into those
paths, so the GUI shows the OmniLink orb consistently across platforms.

We render *fresh* at each target size — never resize a master — so the
resolution-aware dot-count and per-dot size in ``render_omnilink_orb._params_for``
actually fire per icon. (A 1024-px lattice downsampled to 32 px would put
each dot at sub-pixel scale and Lanczos averaging would dim the icon to
near-invisibility on dark taskbars.)
"""

from __future__ import annotations

import sys
from pathlib import Path

from PIL import Image

REPO = Path(__file__).resolve().parents[2]
BRAND = REPO / "resources" / "branding" / "omnilink"
# The 1024×1024 master orb — the canonical OmniSim brand mark.
SOURCE_PNG = BRAND / "orb" / "orb.png"

# Re-use the orb renderer so we can rasterize at exact icon sizes — the
# renderer is resolution-aware, so each icon size gets dot-count and per-dot
# size tuned to stay legible at that scale. Adding the scripts/branding dir
# to sys.path keeps this tool standalone.
sys.path.insert(0, str(Path(__file__).parent))
try:
    import render_omnilink_orb as orb_renderer  # noqa: E402
    HAVE_RENDERER = True
except ImportError:
    HAVE_RENDERER = False

# Paths the GUI / packaging code references — every one of these
# is overwritten with a render of the OmniLink mark.
ICON_TARGETS: list[tuple[Path, int]] = [
    (REPO / "resources" / "images" / "omnisim.png", 256),
    (REPO / "resources" / "icons" / "core" / "omnisim.png", 32),
    (REPO / "resources" / "icons" / "core" / "omnisim64x64.png", 64),
    (REPO / "resources" / "icons" / "core" / "omnisim_doc.png", 256),
    (REPO / "resources" / "web" / "streaming_viewer" / "omnisim_icon.png", 256),
    (REPO / "scripts" / "packaging" / "omnisim.png", 256),
    (REPO / "scripts" / "packaging" / "omnisim_doc.png", 256),
]

ICO_TARGETS: list[Path] = [
    REPO / "src" / "omnisim" / "gui" / "omnisim.ico",
    REPO / "resources" / "icons" / "core" / "omnisim_doc.ico",
]
ICO_SIZES = [16, 24, 32, 48, 64, 128, 256]


def _trim_to_square(img: Image.Image) -> Image.Image:
    """Crop transparent margins and return a centered square so the mark fills the icon."""
    img = img.convert("RGBA")
    bbox = img.getbbox()
    if bbox is None:
        return img
    cropped = img.crop(bbox)
    side = max(cropped.size)
    canvas = Image.new("RGBA", (side, side), (0, 0, 0, 0))
    ox = (side - cropped.size[0]) // 2
    oy = (side - cropped.size[1]) // 2
    canvas.paste(cropped, (ox, oy), cropped)
    return canvas


def _render_at(size: int) -> Image.Image:
    """Render the dot-sphere fresh at the requested icon size.

    The renderer is resolution-aware: at small sizes it drops the dot count
    and bumps per-dot size so the icon stays legible on a dark taskbar (a
    1024-px lattice downsampled to 32 px would put each dot at sub-pixel
    scale and Lanczos would average it into invisibility). Rendering fresh
    at the target size lets every icon use the right population.

    Falls back to a Lanczos resize of the cached master if the renderer
    module isn't importable.
    """
    if HAVE_RENDERER:
        # Render at 2× target then downsample once for crisp anti-aliasing.
        img = orb_renderer.render(resolution=size * 2)
        img = img.resize((size, size), Image.LANCZOS)
    else:
        if _MASTER is None:
            globals()["_MASTER"] = Image.open(SOURCE_PNG).convert("RGBA")
        img = _MASTER.resize((size, size), Image.LANCZOS)
    return img


_MASTER: Image.Image | None = None


def main() -> int:
    if not SOURCE_PNG.exists() and not HAVE_RENDERER:
        print(
            f"error: source missing: {SOURCE_PNG} — run render_omnilink_orb.py first",
            file=sys.stderr,
        )
        return 1

    # Cache one render per unique target size.
    unique_sizes = sorted({size for _, size in ICON_TARGETS} | set(ICO_SIZES))
    cache: dict[int, Image.Image] = {}
    for size in unique_sizes:
        print(f"  rendering orb @ {size}px")
        cache[size] = _render_at(size)

    for path, size in ICON_TARGETS:
        path.parent.mkdir(parents=True, exist_ok=True)
        cache[size].save(path, format="PNG")
        print(f"  wrote {path.relative_to(REPO)}  ({size}px)")

    for ico_path in ICO_TARGETS:
        ico_path.parent.mkdir(parents=True, exist_ok=True)
        # PIL's .ico writer accepts a base image plus a list of sizes — but the
        # results are higher-quality if we hand-pack one rendered image per
        # size. We pass the largest as base, then PIL embeds the others on
        # save with quality determined by Image.save(append_images=...).
        base = cache[max(ICO_SIZES)]
        append = [cache[s] for s in ICO_SIZES if s != max(ICO_SIZES)]
        base.save(ico_path, format="ICO", sizes=[(s, s) for s in ICO_SIZES],
                  append_images=append)
        print(f"  wrote {ico_path.relative_to(REPO)}  (multi-size .ico)")

    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
