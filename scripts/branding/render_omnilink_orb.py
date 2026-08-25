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

"""Render the OmniLink orb — the canonical OmniSim brand mark.

A spherical Fibonacci lattice of matte Mimosa-yellow dots on transparent
background, matching the static reference on the OmniLink home page:

  * uniform Fibonacci distribution of dots on the surface of a sphere
  * a small population of interior dots for subtle depth through the front face
  * each dot is a clean anti-aliased filled circle in matte Mimosa yellow
  * dot size and color scale with z so the sphere reads as 3D without effects
  * NO bloom, NO halo, NO central glow — pure discrete dots on black

The renderer is **resolution-aware** (see ``_params_for``): at small icon
sizes it drops the dot count and bumps the relative dot radius so the orb
stays legible on dark taskbars. Without this, a 1024-px lattice downsampled
to 32 px puts each dot at sub-pixel scale and Lanczos averages the icon to
near-invisibility.

Outputs:
  resources/branding/omnilink/orb/orb.png       (1024×1024 RGBA master)
  resources/branding/omnilink/orb/orb_512.png   (512×512 cached half-size)
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

REPO = Path(__file__).resolve().parents[2]
ORB_DIR = REPO / "resources" / "branding" / "omnilink" / "orb"

# --- canvas / camera ----------------------------------------------------------
RESOLUTION = 1024
SPHERE_RADIUS_PX = RESOLUTION * 0.34  # tight enough to leave breathing room
CENTER_PX = RESOLUTION // 2

# --- dot population (resolution-aware via _params_for) ------------------------
# Reference values at 1024 px master resolution. Below ~256 px we drop dot
# count and bump per-dot size proportionally so each dot stays visible — at
# 32 px a 1024-px lattice would put each dot at sub-pixel scale and the
# whole icon would dim out to invisibility.
SURFACE_DOTS_REF = 220
INTERIOR_DOTS_REF = 80
INTERIOR_RADIUS_FRACTION = 0.92  # how far in from the surface interior dots can sit

# --- dot appearance -----------------------------------------------------------
# Mimosa yellow. We push closer to the brand accent (#F6E905 ≈ 246,233,5) for
# the front dots so they have enough chroma to survive on dark taskbars at
# small sizes. Back-of-sphere dots stay muted for the 3D read but never go
# darker than ~50% luma, otherwise they vanish under Lanczos.
DOT_COLOR_FRONT = np.array([246, 233, 50], dtype=np.float32)
DOT_COLOR_BACK = np.array([150, 138, 38], dtype=np.float32)
DOT_SIZE_PX_FRONT_REF = 8.0   # at z = +r, at 1024 px
DOT_SIZE_PX_BACK_REF = 2.4    # at z = -r, at 1024 px
INTERIOR_SIZE_SCALE = 0.72


def _params_for(resolution: int) -> dict:
    """Return resolution-aware orb parameters.

    The strategy is to keep each dot **at least ~1.6 px** at the smallest icon
    sizes by reducing the dot count and bumping the relative dot radius.
    Below 96 px we also drop the interior population so the surface lattice
    stays uncluttered at icon scale.
    """
    if resolution >= 512:
        return dict(
            surface=SURFACE_DOTS_REF,
            interior=INTERIOR_DOTS_REF,
            front_px=DOT_SIZE_PX_FRONT_REF * (resolution / 1024.0),
            back_px=DOT_SIZE_PX_BACK_REF * (resolution / 1024.0),
            color_front=DOT_COLOR_FRONT,
            color_back=DOT_COLOR_BACK,
        )
    if resolution >= 192:
        # Mid sizes — keep depth contrast but reduce population so each dot
        # has elbow room.
        return dict(
            surface=140,
            interior=40,
            front_px=max(4.5, resolution * 0.020),
            back_px=max(1.6, resolution * 0.008),
            color_front=DOT_COLOR_FRONT,
            color_back=DOT_COLOR_BACK,
        )
    if resolution >= 64:
        # Small icons — drop the interior, flatten the depth gradient, push
        # all dots toward the brighter front color so the icon reads at all.
        return dict(
            surface=80,
            interior=0,
            front_px=max(2.6, resolution * 0.045),
            back_px=max(1.4, resolution * 0.024),
            color_front=DOT_COLOR_FRONT,
            color_back=np.array([200, 184, 44], dtype=np.float32),
        )
    # Tiny icons (16–32 px). Even fewer dots, uniformly bright — at 16 px the
    # cluster reads as a small yellow ball with just enough granularity to
    # signal "sphere of dots."
    return dict(
        surface=46,
        interior=0,
        front_px=max(1.7, resolution * 0.075),
        back_px=max(1.2, resolution * 0.055),
        color_front=DOT_COLOR_FRONT,
        color_back=np.array([220, 204, 48], dtype=np.float32),
    )


def _fibonacci_sphere(n: int) -> np.ndarray:
    """Return n unit-vectors uniformly distributed on the surface of a sphere
    via the spherical-Fibonacci lattice (golden-angle method)."""
    indices = np.arange(n, dtype=np.float64) + 0.5
    phi = np.arccos(1 - 2 * indices / n)  # polar angle
    theta = math.pi * (1 + 5 ** 0.5) * indices  # golden angle
    x = np.cos(theta) * np.sin(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(phi)
    return np.stack([x, y, z], axis=-1).astype(np.float32)


def _interior_points(n: int, rng: np.random.Generator) -> np.ndarray:
    """Sample n points uniformly inside the unit ball, scaled to a radius
    fraction so they're visibly inside the surface lattice."""
    pts = []
    while len(pts) < n:
        batch = rng.uniform(-1.0, 1.0, size=(n * 3, 3))
        keep = batch[np.sum(batch * batch, axis=1) <= 1.0]
        pts.extend(keep.tolist())
    arr = np.asarray(pts[:n], dtype=np.float32)
    return arr * INTERIOR_RADIUS_FRACTION


def _draw_dot(draw: ImageDraw.ImageDraw, cx: float, cy: float, r: float,
              color: tuple[int, int, int]) -> None:
    """Anti-aliased filled circle. PIL's ellipse is fine here — we don't need
    sub-pixel splat blending because the dots are deliberately discrete."""
    draw.ellipse(
        (cx - r, cy - r, cx + r, cy + r),
        fill=color + (255,),
    )


def _color_at(z_norm: float, color_front: np.ndarray, color_back: np.ndarray) -> tuple[int, int, int]:
    """Lerp between back/front colors. z_norm in [-1, 1] (0 = sphere center)."""
    t = (z_norm + 1.0) * 0.5
    rgb = color_back * (1 - t) + color_front * t
    return tuple(int(round(v)) for v in np.clip(rgb, 0, 255))


def _size_at(z_norm: float, front_px: float, back_px: float, scale: float = 1.0) -> float:
    """Smooth (cosine-eased) size lerp for a 3D-feeling falloff."""
    t = (z_norm + 1.0) * 0.5
    eased = 0.5 - 0.5 * math.cos(math.pi * t)
    return (back_px * (1 - eased) + front_px * eased) * scale


def render(resolution: int = RESOLUTION) -> Image.Image:
    rng = np.random.default_rng(42)
    img = Image.new("RGBA", (resolution, resolution), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img, "RGBA")

    p = _params_for(resolution)
    surface_n = p["surface"]
    interior_n = p["interior"]
    front_px = p["front_px"]
    back_px = p["back_px"]
    color_front = p["color_front"]
    color_back = p["color_back"]

    # Slightly tighter sphere at small sizes so the lattice has room to breathe
    # next to the icon's edge.
    radius_frac = 0.34 if resolution >= 192 else 0.40
    radius_px = resolution * radius_frac
    cx_px = cy_px = resolution // 2

    # Build a single combined point list with a `scale` factor so we can sort
    # back-to-front and let front dots paint over back ones.
    surface = _fibonacci_sphere(surface_n)
    if interior_n > 0:
        interior = _interior_points(interior_n, rng) * 0.95
        pts = np.concatenate([surface, interior], axis=0)
        scales = np.concatenate([
            np.full(surface_n, 1.0, dtype=np.float32),
            np.full(interior_n, INTERIOR_SIZE_SCALE, dtype=np.float32),
        ])
    else:
        pts = surface
        scales = np.full(surface_n, 1.0, dtype=np.float32)

    # Sort back-to-front so closer dots render last (on top).
    order = np.argsort(pts[:, 2])
    for i in order:
        x, y, z = pts[i]
        # Subtle perspective projection: dots closer to camera (positive z)
        # shift slightly outward, but the effect is small — the reference is
        # essentially orthographic.
        persp = 1.0 + 0.04 * z
        sx = cx_px + x * radius_px * persp
        sy = cy_px - y * radius_px * persp  # flip y so +y is up
        size = _size_at(float(z), front_px, back_px, scale=float(scales[i]))
        color = _color_at(float(z), color_front, color_back)
        _draw_dot(draw, sx, sy, size, color)

    return img


def main() -> int:
    ORB_DIR.mkdir(parents=True, exist_ok=True)
    print(f"rendering OmniLink orb at {RESOLUTION}×{RESOLUTION}…")
    img = render()
    master = ORB_DIR / "orb.png"
    img.save(master)
    print(f"  wrote {master.relative_to(REPO)}")
    half = img.resize((512, 512), Image.LANCZOS)
    half_path = ORB_DIR / "orb_512.png"
    half.save(half_path)
    print(f"  wrote {half_path.relative_to(REPO)}")
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
