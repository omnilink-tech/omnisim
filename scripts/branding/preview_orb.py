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

"""Render a side-by-side preview sheet of the OmniLink orb at every icon size,
plus a mocked-up About-box composition. Output goes to a single PNG that the
brand-guide and PR descriptions can link to.
"""

from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "resources" / "branding" / "omnilink" / "preview" / "orb_preview.png"
OUT.parent.mkdir(parents=True, exist_ok=True)

# panels: (label, path, drawn-at-size)
ICONS = [
    ("16 px",  REPO / "resources" / "icons" / "core" / "omnisim.png", 16),
    ("32 px",  REPO / "resources" / "icons" / "core" / "omnisim.png", 32),
    ("64 px",  REPO / "resources" / "icons" / "core" / "omnisim64x64.png", 64),
    ("128 px", REPO / "resources" / "images" / "omnisim.png", 128),
    ("256 px", REPO / "resources" / "images" / "omnisim.png", 256),
]


def font(size: int) -> ImageFont.ImageFont:
    candidates = [
        REPO / "resources" / "branding" / "omnilink" / "fonts" / "Montserrat-Medium.otf",
        REPO / "resources" / "branding" / "omnilink" / "fonts" / "Montserrat-Regular.ttf",
    ]
    for p in candidates:
        try:
            return ImageFont.truetype(str(p), size)
        except OSError:
            continue
    return ImageFont.load_default()


def draw_icon_row() -> Image.Image:
    pad = 24
    label_h = 22
    cell_h = 256 + pad * 2 + label_h
    cell_w = 256 + pad * 2
    total_w = cell_w * len(ICONS)
    sheet = Image.new("RGBA", (total_w, cell_h), (10, 10, 12, 255))
    draw = ImageDraw.Draw(sheet)
    f = font(13)

    for i, (label, path, size) in enumerate(ICONS):
        cell_x = i * cell_w
        # cell background
        draw.rectangle(
            [(cell_x + 4, 4), (cell_x + cell_w - 4, cell_h - 4)],
            fill=(0, 0, 0, 255),
            outline=(40, 40, 40, 255),
            width=1,
        )
        src = Image.open(path).convert("RGBA")
        ico = src.resize((size, size), Image.LANCZOS)
        ix = cell_x + (cell_w - size) // 2
        iy = pad + (256 - size) // 2
        sheet.paste(ico, (ix, iy), ico)
        # label
        bbox = draw.textbbox((0, 0), label, font=f)
        tw = bbox[2] - bbox[0]
        draw.text(
            (cell_x + (cell_w - tw) // 2, cell_h - label_h - 4),
            label,
            font=f,
            fill=(238, 238, 224, 255),  # cream
        )

    return sheet


def draw_about_mock() -> Image.Image:
    w, h = 720, 320
    canvas = Image.new("RGBA", (w, h), (28, 28, 28, 255))
    draw = ImageDraw.Draw(canvas)

    # title bar
    draw.rectangle([(0, 0), (w, 32)], fill=(45, 45, 48, 255))
    f_title = font(13)
    draw.text((12, 8), "About OmniSim", font=f_title, fill=(238, 238, 224, 255))

    # left: orb mark at 128 px
    orb = Image.open(REPO / "resources" / "images" / "omnisim.png").convert("RGBA")
    orb_128 = orb.resize((128, 128), Image.LANCZOS)
    canvas.paste(orb_128, (40, 90), orb_128)

    # right: brand copy
    f_h = font(22)
    f_b = font(14)
    f_s = font(11)
    draw.text((200, 70), "OmniSim R2025a", font=f_h, fill=(238, 238, 224, 255))
    draw.text((200, 100), "by OmniLink · April 30, 2026", font=f_s, fill=(167, 167, 147, 255))
    draw.text(
        (200, 138),
        "The simulator built by OmniLink, for OmniLink agents.",
        font=f_b,
        fill=(238, 238, 224, 255),
    )
    draw.text(
        (200, 162),
        "Free and open-source, built on top of the Webots engine",
        font=f_b,
        fill=(238, 238, 224, 255),
    )
    draw.text(
        (200, 184),
        "by Cyberbotics Ltd. Licensed under Apache 2.0.",
        font=f_b,
        fill=(238, 238, 224, 255),
    )
    draw.text(
        (200, 240),
        "© OmniLink · Upstream Webots © Cyberbotics 1998-2026",
        font=f_s,
        fill=(140, 140, 140, 255),
    )

    return canvas


def main() -> int:
    icons = draw_icon_row()
    about = draw_about_mock()

    pad = 32
    out_w = max(icons.width, about.width) + pad * 2
    out_h = icons.height + about.height + pad * 3
    sheet = Image.new("RGBA", (out_w, out_h), (5, 5, 8, 255))
    sheet.paste(icons, ((out_w - icons.width) // 2, pad), icons)
    sheet.paste(about, ((out_w - about.width) // 2, pad * 2 + icons.height), about)

    sheet.convert("RGB").save(OUT)
    print(f"wrote {OUT.relative_to(REPO)}  ({sheet.size[0]}x{sheet.size[1]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
