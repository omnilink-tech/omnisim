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

"""Generate the seamless knit-jersey texture set a `Cloth { appearance ... }` uses.

WHY THIS IS A SCRIPT AND NOT THREE COMMITTED PNGs. The repo's rule for generated
assets is that the generator is the reviewable artefact (cf.
scripts/dev/usd_to_cloth_obj.py): a binary blob with no provenance cannot be
re-derived, re-scaled, or re-coloured, and nobody can tell whether it is licensed.
This produces the files from arithmetic, deterministically -- same arguments give
byte-identical PNGs -- so the committed textures are a build product with a
recipe, and a different fabric is one flag away.

WHAT IT MAKES, and why a cloth needs all three:

  <name>_basecolor.png   albedo. Carries the knit's own light/dark modulation, so
                         the weave is visible even head-on under flat light.
  <name>_normal.png      tangent-space normal map. THIS is the one that makes a
                         616-particle garment read as fabric: the simulated mesh
                         can only resolve ~4 cm wrinkles, and every stitch-scale
                         highlight in the render comes from here. WREN's PBR
                         fragment shader derives the tangent frame per pixel with
                         screen-space derivatives (pbr.frag::cotangentFrame), so
                         this works on a mesh that stores NO tangent attribute --
                         which a WrDynamicMesh cannot store anyway.
  <name>_roughness.png   red channel = perceptual roughness. Jersey is matte with
                         slightly shinier stitch crowns; a constant roughness
                         makes fabric look like painted plastic.

SEAMLESSNESS is by construction, not by a blur: every term is a function of
frac(u * wales) / frac(v * courses) with integer wale and course counts, and the
noise is a tiled random lattice sampled with wraparound. That matters because the
garment's UV set repeats ~3.6 x 4.0 times over the shirt (see
usd_to_cloth_obj.py::arap_panel_uvs), so any edge discontinuity would appear as a
visible grid across the whole thing.

⚠ SEAMLESSNESS IS ALSO WHAT LETS THE GARMENT'S MAP FOLD. The shirt's unwrap is
2-to-1 on purpose -- it has to be continuous over a closed tube, which is only
possible if the map mirrors -- so the front and back panels sample the SAME texels
and the fold line runs along the side seam. A directional or non-tiling texture
would make that mirror visible; an isotropic tiling weave does not, which is the
whole reason a fabric material is the right thing to put on this asset.

  python scripts/dev/make_cloth_textures.py                    # the shipped set
  python scripts/dev/make_cloth_textures.py --base-color 0.8 0.2 0.2 --name red_jersey
"""

import argparse
import os
import sys

DEFAULT_OUT = os.path.join("projects", "samples", "demos", "worlds", "physics", "textures")


def _tiled_value_noise(rng, size, cells):
    """Seamless value noise: a random `cells` x `cells` lattice, bilinearly
    upsampled with the sample grid wrapping back onto index 0."""
    import numpy as np
    lattice = rng.random((cells, cells))
    t = np.arange(size, dtype=np.float64) * (cells / float(size))
    i0 = np.floor(t).astype(np.int64) % cells
    i1 = (i0 + 1) % cells
    f = t - np.floor(t)
    f = f * f * (3.0 - 2.0 * f)                      # smoothstep, so the lattice does not show
    a = lattice[np.ix_(i0, i0)] * (1 - f)[:, None] * (1 - f)[None, :]
    b = lattice[np.ix_(i1, i0)] * f[:, None] * (1 - f)[None, :]
    c = lattice[np.ix_(i0, i1)] * (1 - f)[:, None] * f[None, :]
    d = lattice[np.ix_(i1, i1)] * f[:, None] * f[None, :]
    return a + b + c + d


def knit_height(size, wales, courses, seed):
    """Height field of a plain-jersey knit, in [0, 1], seamless over [0, 1)^2.

    A jersey face stitch reads as a 'V': two yarn legs running from the bottom
    corners of the cell up to the middle of the row above. Modelling it as the
    distance to those two legs (rather than as a sine grid) is what gives the
    fabric its direction -- a knit looks different along the wales and along the
    courses, and a symmetric bump grid looks like a golf ball.
    """
    import numpy as np
    rng = np.random.default_rng(seed)
    u = (np.arange(size) + 0.5) / size
    v = (np.arange(size) + 0.5) / size
    cu = np.mod(u[None, :] * wales, 1.0)
    cv = np.mod(v[:, None] * courses, 1.0)

    # The two legs of the V, as u-offsets that depend on how far up the cell we
    # are. Distances are taken cyclically in cu so the stitch wraps into its
    # neighbour instead of ending at the cell wall.
    def cyc(d):
        return np.minimum(np.abs(d), 1.0 - np.abs(d))

    left = cyc(cu - (0.5 - 0.5 * cv))
    right = cyc(cu - (0.5 + 0.5 * cv))
    leg = np.minimum(left, right)
    h = np.exp(-((leg / 0.17) ** 2))

    # The head of the loop: where the two legs of the row BELOW cross over, a
    # jersey has a horizontal bar. Without it the V's read as unconnected chevrons.
    head = np.exp(-(((cv - 0.06) / 0.10) ** 2)) * np.exp(-((cyc(cu - 0.5) / 0.34) ** 2))
    h = np.maximum(h, 0.85 * head)

    # Yarn fibre. Two octaves: the coarse one is the yarn's own twist, the fine
    # one is the fuzz that keeps the specular from looking like moulded rubber.
    h = h * (0.88 + 0.24 * _tiled_value_noise(rng, size, wales * 2))
    h = h + 0.05 * (_tiled_value_noise(rng, size, size // 4) - 0.5)
    h = h - h.min()
    return h / max(float(h.max()), 1e-9)


def normal_map(h, strength):
    """Tangent-space normal from a wrapped central difference of the height."""
    import numpy as np
    dx = (np.roll(h, -1, axis=1) - np.roll(h, 1, axis=1)) * 0.5
    dy = (np.roll(h, -1, axis=0) - np.roll(h, 1, axis=0)) * 0.5
    nx = -dx * strength
    ny = -dy * strength
    nz = np.ones_like(h)
    inv = 1.0 / np.sqrt(nx * nx + ny * ny + nz * nz)
    return np.stack([nx * inv, ny * inv, nz * inv], axis=-1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--name", default="knit_jersey")
    ap.add_argument("--size", type=int, default=512)
    ap.add_argument("--wales", type=int, default=24, help="stitch columns per tile")
    ap.add_argument("--courses", type=int, default=32, help="stitch rows per tile")
    ap.add_argument("--base-color", type=float, nargs=3, default=(0.24, 0.36, 0.66),
                    help="LINEAR rgb of the yarn (the PNG is written in sRGB)")
    ap.add_argument("--normal-strength", type=float, default=7.0)
    ap.add_argument("--roughness", type=float, nargs=2, default=(0.70, 0.94),
                    help="roughness at a stitch crown and in a stitch valley")
    ap.add_argument("--seed", type=int, default=20260815)
    args = ap.parse_args()

    try:
        import numpy as np
        from PIL import Image
    except ImportError as exc:                                   # pragma: no cover
        print("needs numpy and Pillow: %s" % exc, file=sys.stderr)
        return 2

    h = knit_height(args.size, args.wales, args.courses, args.seed)

    # ---- base colour. The knit's own shading is baked in at low contrast: it has
    # to survive being multiplied by a light that may be nearly grazing, and a
    # high-contrast albedo would fight the normal map instead of reinforcing it.
    lin = np.array(args.base_color, dtype=np.float64)[None, None, :]
    shade = (0.74 + 0.42 * h)[..., None]
    rgb_lin = np.clip(lin * shade, 0.0, 1.0)
    srgb = np.where(rgb_lin <= 0.0031308, rgb_lin * 12.92, 1.055 * rgb_lin ** (1 / 2.4) - 0.055)
    Image.fromarray((np.clip(srgb, 0, 1) * 255.0 + 0.5).astype(np.uint8), "RGB").save(
        os.path.join(args.out, "%s_basecolor.png" % args.name))

    # ---- normal. Written RAW (no sRGB curve) because it is a vector, not a colour.
    n = normal_map(h, args.normal_strength)
    Image.fromarray(((n * 0.5 + 0.5) * 255.0 + 0.5).astype(np.uint8), "RGB").save(
        os.path.join(args.out, "%s_normal.png" % args.name))

    # ---- roughness. WREN's pbr.frag reads the RED channel (inputTextures[1].r),
    # so a grey image is the honest thing to write even though only R is sampled.
    crown, valley = args.roughness
    rough = valley + (crown - valley) * h
    g = (np.clip(rough, 0, 1) * 255.0 + 0.5).astype(np.uint8)
    Image.fromarray(np.stack([g, g, g], axis=-1), "RGB").save(
        os.path.join(args.out, "%s_roughness.png" % args.name))

    print("wrote %s_{basecolor,normal,roughness}.png (%dx%d, %d wales x %d courses) to %s"
          % (args.name, args.size, args.size, args.wales, args.courses, args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
