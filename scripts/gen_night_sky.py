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

# =============================================================================
# RETIRED one-off (2026-07-09) — superseded by the canonical OmniSimSky lighting
# recipe (docs/WORLD_RECIPE.md: OmniSimSky + OmniSimSun + OmniSimSunMarker).
# Do NOT use this for new worlds. Kept for provenance of the existing generated
# textures / the completed migration.
# =============================================================================

"""
Generate a 6-face cubemap of a dark starfield for use as a Webots Background.

Output: projects/objects/backgrounds/textures/night_sky/{back,bottom,front,left,right,top}.jpg

Stars are sampled uniformly on a unit sphere then projected onto the cube's
six faces, so the seams across faces are visually continuous.
"""
import math
import os
import random
from pathlib import Path
from PIL import Image, ImageDraw

random.seed(2026)

SIZE = 1024
OUT_DIR = Path(__file__).resolve().parent.parent / "projects" / "objects" / "backgrounds" / "textures" / "night_sky"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# Webots cube faces. We follow the same naming as the six-face cubemap naming convention (_left/_right/_top/_bottom/_front/_back).
FACES = ("back", "bottom", "front", "left", "right", "top")

# Pre-allocate one image per face.
images = {f: Image.new("RGB", (SIZE, SIZE), (3, 5, 12)) for f in FACES}
draws = {f: ImageDraw.Draw(im) for f, im in images.items()}


def project_sphere_to_face(x, y, z):
    """Map a unit-sphere point to (face, u, v) in [0, SIZE)."""
    ax, ay, az = abs(x), abs(y), abs(z)
    if ax >= ay and ax >= az:
        # left/right face dominates
        if x > 0:
            face = "right"
            u = -z / ax
            v = -y / ax
        else:
            face = "left"
            u = z / ax
            v = -y / ax
    elif ay >= ax and ay >= az:
        if y > 0:
            face = "top"
            u = x / ay
            v = z / ay
        else:
            face = "bottom"
            u = x / ay
            v = -z / ay
    else:
        if z > 0:
            face = "front"
            u = x / az
            v = -y / az
        else:
            face = "back"
            u = -x / az
            v = -y / az
    px = int((u * 0.5 + 0.5) * (SIZE - 1))
    py = int((v * 0.5 + 0.5) * (SIZE - 1))
    return face, px, py


def random_unit_vec():
    """Uniform random point on unit sphere."""
    while True:
        x = random.uniform(-1, 1)
        y = random.uniform(-1, 1)
        z = random.uniform(-1, 1)
        n2 = x * x + y * y + z * z
        if 0.001 < n2 <= 1.0:
            n = math.sqrt(n2)
            return x / n, y / n, z / n


def draw_star(face, px, py, brightness, radius, tint=(255, 255, 255)):
    """Draw a Gaussian-ish bright dot with optional cross spike."""
    im = images[face]
    pixels = im.load()
    r2 = radius * radius
    ir = int(math.ceil(radius))
    for dx in range(-ir, ir + 1):
        for dy in range(-ir, ir + 1):
            d2 = dx * dx + dy * dy
            if d2 > r2:
                continue
            xx, yy = px + dx, py + dy
            if not (0 <= xx < SIZE and 0 <= yy < SIZE):
                continue
            falloff = math.exp(-d2 / max(0.5, r2 * 0.4))
            cur = pixels[xx, yy]
            add_r = int(tint[0] * brightness * falloff / 255)
            add_g = int(tint[1] * brightness * falloff / 255)
            add_b = int(tint[2] * brightness * falloff / 255)
            pixels[xx, yy] = (
                min(255, cur[0] + add_r),
                min(255, cur[1] + add_g),
                min(255, cur[2] + add_b),
            )


def main():
    # Faint starfield - lots of dim points.
    for _ in range(40000):
        x, y, z = random_unit_vec()
        face, px, py = project_sphere_to_face(x, y, z)
        b = random.randint(40, 130)
        # Slight color jitter (most stars are white, a few cool/warm).
        if random.random() < 0.05:
            tint = (255, 220, 200)  # warm
        elif random.random() < 0.05:
            tint = (200, 215, 255)  # cool
        else:
            tint = (255, 255, 255)
        # Single-pixel dim stars.
        im = images[face]
        cur = im.getpixel((px, py))
        nr = min(255, cur[0] + b * tint[0] // 255)
        ng = min(255, cur[1] + b * tint[1] // 255)
        nb = min(255, cur[2] + b * tint[2] // 255)
        im.putpixel((px, py), (nr, ng, nb))

    # Medium stars - small Gaussian blobs.
    for _ in range(1500):
        x, y, z = random_unit_vec()
        face, px, py = project_sphere_to_face(x, y, z)
        brightness = random.randint(140, 220)
        radius = random.uniform(1.0, 1.8)
        draw_star(face, px, py, brightness, radius)

    # Bright stars - larger glow with optional color tint.
    for _ in range(120):
        x, y, z = random_unit_vec()
        face, px, py = project_sphere_to_face(x, y, z)
        brightness = random.randint(220, 255)
        radius = random.uniform(2.2, 4.0)
        roll = random.random()
        if roll < 0.15:
            tint = (255, 200, 170)  # red giant
        elif roll < 0.30:
            tint = (180, 210, 255)  # blue giant
        else:
            tint = (255, 250, 240)  # white
        draw_star(face, px, py, brightness, radius, tint)

    # Very faint nebula patches - low-frequency noise overlaid.
    nebula_count = 6
    for _ in range(nebula_count):
        cx, cy, cz = random_unit_vec()
        nebula_face, ncx, ncy = project_sphere_to_face(cx, cy, cz)
        nebula_r = random.randint(120, 220)
        tint = random.choice([(20, 15, 35), (15, 25, 40), (35, 18, 30)])
        im = images[nebula_face]
        pixels = im.load()
        for dy in range(-nebula_r, nebula_r):
            for dx in range(-nebula_r, nebula_r):
                d2 = dx * dx + dy * dy
                if d2 > nebula_r * nebula_r:
                    continue
                xx, yy = ncx + dx, ncy + dy
                if not (0 <= xx < SIZE and 0 <= yy < SIZE):
                    continue
                falloff = math.exp(-d2 / (nebula_r * nebula_r * 0.35))
                # Add jitter for cloud-like texture.
                jitter = random.uniform(0.5, 1.0)
                cur = pixels[xx, yy]
                pixels[xx, yy] = (
                    min(255, cur[0] + int(tint[0] * falloff * jitter)),
                    min(255, cur[1] + int(tint[1] * falloff * jitter)),
                    min(255, cur[2] + int(tint[2] * falloff * jitter)),
                )

    for face, im in images.items():
        path = OUT_DIR / f"night_sky_{face}.jpg"
        im.save(path, quality=88, optimize=True)
        print(f"  wrote {path} ({path.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
