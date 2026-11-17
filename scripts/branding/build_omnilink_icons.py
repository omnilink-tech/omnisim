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

"""Regenerate the GUI's app-icon PNGs and Windows .ico from the OmniSim mark.

The OmniSim identity lives in ``resources/branding/omnisim/`` (see ``BRAND.md``
there): the dot-lattice orb, and the glyph that replaces it at small sizes.
Every app icon is that mark on its rounded near-black tile, rendered by
``render_omnisim_orb.render_app_icon``. The tile is what keeps the icon legible
on any wallpaper: the bare dots on a transparent background (the OmniLink orb
this script used to render) dissolved into a busy or starry desktop.

The C++ GUI loads icons from ``images:omnisim.png`` /
``icons/core/omnisim*.png`` / ``src/omnisim/gui/omnisim.ico``, and the
packagers read ``scripts/packaging/omnisim*.png``. This script writes all of
them.

We render *fresh* at each target size — never resize a master — so the
orb / glyph / micro-lattice switch in ``render_app_icon`` fires per icon.
(A 1024-px lattice downsampled to 32 px would put each dot at sub-pixel scale
and Lanczos averaging would dim the icon to near-invisibility.)

The script keeps its historical name because the build docs point at it.
"""

from __future__ import annotations

import sys
from pathlib import Path

from PIL import Image

REPO = Path(__file__).resolve().parents[2]

sys.path.insert(0, str(Path(__file__).parent))
import render_omnisim_orb as omnisim_mark  # noqa: E402

# Paths the GUI / packaging code references — every one of these
# is overwritten with a render of the OmniSim app icon.
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


def _render_at(size: int) -> Image.Image:
    """Render the app icon fresh at the requested size."""
    return omnisim_mark.render_app_icon(size)


def main() -> int:
    # Cache one render per unique target size.
    unique_sizes = sorted({size for _, size in ICON_TARGETS} | set(ICO_SIZES))
    cache: dict[int, Image.Image] = {}
    for size in unique_sizes:
        print(f"  rendering app icon @ {size}px")
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
