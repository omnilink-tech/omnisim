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

"""Generate the seamless PBR texture sets for the sponge and woven-cloth demos.

This is the sibling of `make_cloth_textures.py` (which owns the *knit* jersey the
T-shirt wears) and follows the same rule the repo applies to every generated
asset: THE GENERATOR IS THE REVIEWABLE ARTEFACT. A committed PNG with no recipe
cannot be re-derived, re-scaled, re-coloured, or licensed-audited. Everything
here comes out of arithmetic, so the same arguments give byte-identical files and
a different sponge is one flag away.

    python scripts/dev/gen_surface_textures.py                  # the shipped sets
    python scripts/dev/gen_surface_textures.py --set sponge
    python scripts/dev/gen_surface_textures.py --size 2048 --preview-dir /tmp/tex

WHAT IT MAKES

  sponge_cellulose_{basecolor,normal,roughness,occlusion}.png
      The yellow cellulose kitchen sponge. Open-cell foam: irregular cavities of
      roughly one size with rounded rims and varying depth, built from a jittered
      Worley lattice rather than a bump grid -- a regular grid reads as a golf
      ball or a waffle, never as foam.

  sponge_scrubpad_{basecolor,normal,roughness,occlusion}.png
      The dark green abrasive side. A nonwoven tangled-filament mat, not a foam.
      ⚠ UNUSED BY ANY WORLD AS SHIPPED -- a `Shape` carries exactly one
      appearance, so a two-sided sponge needs two Shapes (or two Solids) sharing
      one boundingObject. Generated because it is nearly free and the demo may
      want it; delete the four files if it stays unused.

  woven_cotton_{basecolor,normal,roughness,occlusion}.png
      Plain weave (1/1) at thread scale, neutral so a world can tint it with the
      `baseColor` field. This is the default cloth set.

  woven_cotton_twill_{basecolor,normal,roughness,occlusion}.png
      2/2 twill. ⚠ Twill is DIRECTIONAL by definition -- the diagonal wale is the
      whole point of the weave -- so under a physics solver that shears the UVs it
      will visibly skew where the plain weave will not. Prefer the plain set for
      `Cloth` nodes; the twill is for rigid upholstery and drapery.

WHY EACH MAP EARNS ITS BYTES

  _basecolor   sRGB albedo. ⚠ WREN MULTIPLIES it by the `baseColor` FIELD
               (pbr.frag:259, `baseColor.rgb * material.baseColorAndTransparency.rgb`),
               it does not replace it. So the sponge set -- which carries its own
               yellow -- must be used with the field left at its `1 1 1` default,
               and the woven set is deliberately near-neutral so a world CAN tint
               it. Getting this backwards double-tints and reads as mud.
  _normal      tangent-space normal. THE map that matters. A `Cloth` is a few
               hundred particles: it can resolve a 4 cm fold and nothing smaller,
               so every pore and every thread in the render comes from here.
               WREN derives the tangent frame per pixel with screen-space
               derivatives (pbr.frag::cotangentFrame), so this works on a mesh
               carrying no tangent attribute -- which a WrDynamicMesh cannot store.
  _roughness   RED channel only (pbr.frag:230, `roughnessSample.r`). Written grey
               anyway, because a one-channel PNG that claims to be three is the
               kind of thing that bites the next person. Constant roughness is the
               single biggest reason a material reads as painted plastic.
  _occlusion   RED channel only (pbr.frag:378), and it modulates AMBIENT only,
               scaled by `occlusionMapStrength`. This is what sells the porosity
               of the sponge: a pore that does not darken when the key light
               swings away is a painted dot, not a hole.

NORMAL-MAP CONVENTION -- READ THIS BEFORE REGENERATING

  Green is DOWN (the "DirectX" / -Y labelling an art tool would give the file).
  That is not a preference, it is what this engine's plumbing requires:

    * `OmImageTexture` uploads `QImage::bits()` with no `mirrored()`
      (OmImageTexture.cpp:321), and QImage row 0 is the TOP row. So the PNG's top
      row lands at texture coordinate v = 0, i.e. v INCREASES DOWNWARD through
      the image as displayed.
    * `cotangentFrame` (pbr.frag:96, the Schueler derivative frame) builds
      B = d(position)/d(texUv.y), so the green channel is measured along
      increasing v -- which is downward.

  Hence ny = -dh/d(row_index), which is what `normal_from_height` below computes
  and what the shipped `make_cloth_textures.py` already does. Feed this engine a
  stock OpenGL/+Y map from a texture library and every bump lights as a dent.

SEAMLESSNESS is by construction, never by a blur or a mirrored edge:
  * value noise is a periodic lattice sampled with wraparound;
  * the Worley feature points live on a toroidal lattice and the search walks
    neighbour cells modulo the lattice;
  * every weave term is a function of an integer thread index taken modulo the
    thread count, or of a phase with an INTEGER number of cycles per tile;
  * every derivative, blur and box filter uses `np.roll` / wrapped indexing.
  `--verify` measures it rather than asserting it (see `seam_report`).

PHYSICAL SCALE. A normal map is a slope field, so it is meaningless without a
tile size: the same height field at 4 cm and at 40 cm are different materials.
The assumptions baked into the defaults are

    sponge   one tile = 4 cm of sponge, 1.6 mm peak-to-valley relief
             (~14 pores across the tile => ~2.9 mm pores, which is a real
             cellulose sponge)
    scrubpad one tile = 4 cm, 1.0 mm relief
    woven    one tile = 2 cm of cloth, 0.35 mm relief
             (48 threads across 2 cm => 0.42 mm pitch, ~60 threads/inch, which
             is ordinary shirting)

⚠ WHAT THE TILE SIZE DOES AND DOES NOT CONTROL -- the intuitive version of this
is wrong. A normal map stores a SLOPE, and slope is invariant under tiling: at
`textureTransform { scale 6 6 }` over a 0.8 m sheet (a 13 cm tile, 6.6x the
design size) the features become 6.6x wider AND 6.6x taller together, so the
shading is IDENTICAL. Tiling coarser does not make the map "too strong".

What the stated tile size actually buys is (a) a relief figure that is physically
honest at that size, and (b) the arithmetic for `textureTransform`: to get real
thread pitch on a 0.8 m sheet you want `scale 40 40` (0.8 / 0.02), and 6 would be
a 13 cm tile whose implied thread pitch is 2.8 mm -- upholstery webbing, not
shirting. Choose repeats for the cloth you want; `normalMapFactor` (the shipped
knit uses 0.55) is then a pure LOOK control for how deep the relief reads, not a
correction for a scale error.
"""

import argparse
import math
import os
import sys

DEFAULT_OUT = os.path.join("projects", "samples", "demos", "worlds", "physics", "textures")

SETS = ("sponge", "scrubpad", "woven", "twill")
SHIPPED = ("sponge", "woven")            # the two a world actually references


# --------------------------------------------------------------------------- #
# seamless primitives                                                          #
# --------------------------------------------------------------------------- #

def _value_noise(rng, size, cells):
    """Seamless value noise: a random `cells` x `cells` periodic lattice,
    bilinearly upsampled with the sample grid wrapping back onto index 0."""
    import numpy as np
    cells = max(2, int(cells))
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


def _fbm(rng, size, base_cells, octaves, gain=0.5, lacunarity=2):
    """Fractal sum of `_value_noise`, in [0, 1]. Every octave keeps an integer
    lattice count, so the sum stays exactly periodic."""
    import numpy as np
    total = np.zeros((size, size), dtype=np.float64)
    amp, norm, cells = 1.0, 0.0, int(base_cells)
    for _ in range(octaves):
        total += amp * _value_noise(rng, size, cells)
        norm += amp
        amp *= gain
        cells = int(cells * lacunarity)
        if cells > size // 2:                        # past Nyquist there is nothing to add
            break
    return total / max(norm, 1e-9)


def _box_blur_wrap(a, r):
    """Wrapped separable box blur via a cumulative sum. Wrapped because a blur
    that clamps at the border would make the AO map non-tiling even though the
    height it came from tiles."""
    import numpy as np
    r = int(r)
    if r < 1:
        return a
    n = a.shape[0]
    k = 2 * r + 1
    idx = np.arange(-r, n + r) % n
    ext = a[:, idx]
    c = np.concatenate([np.zeros((n, 1)), np.cumsum(ext, axis=1)], axis=1)
    out = (c[:, k:] - c[:, :-k]) / k
    ext = out[idx, :]
    c = np.concatenate([np.zeros((1, n)), np.cumsum(ext, axis=0)], axis=0)
    return (c[k:, :] - c[:-k, :]) / k


def _blur_wrap(a, r, passes=3):
    """Three box passes approximate a Gaussian closely enough for an AO term."""
    out = a
    for _ in range(passes):
        out = _box_blur_wrap(out, r)
    return out


def _smax(a, b, k):
    """Polynomial smooth maximum (Quilez). Rounds the seam where two cavities or
    two threads intersect -- a hard `np.maximum` leaves a crease whose 1-pixel
    gradient blows up into a bright line in the normal map."""
    import numpy as np
    if k <= 0:
        return np.maximum(a, b)
    h = np.clip(0.5 + 0.5 * (b - a) / k, 0.0, 1.0)
    return a * (1 - h) + b * h + k * h * (1 - h)


def _smoothstep(x):
    return x * x * (3.0 - 2.0 * x)


def _norm01(a):
    import numpy as np
    lo = float(a.min())
    hi = float(a.max())
    return (a - lo) / max(hi - lo, 1e-9)


# --------------------------------------------------------------------------- #
# sponge: open-cell cellulose foam                                             #
# --------------------------------------------------------------------------- #

def _cavity_layer(size, cells, rng, jitter, radius, depth_lo, depth_hi, blend, reach=2):
    """One layer of open-cell cavities on a toroidal jittered lattice.

    Returns (depth, tint) where `depth` in [0, 1] is how far BELOW the intact
    surface each pixel has been eaten away, and `tint` carries a per-pore random
    value so the base colour can vary from pore to pore instead of only within
    one.

    The profile is (1 - t^2)^2: zero slope at the pore centre AND zero slope at
    the rim, which is what gives a rounded rim instead of a cone. Overlapping
    radii (radius > 1 cell) are deliberate -- that is what turns the walls between
    pores into thin ridges and occasionally breaks one open, which is the
    structural difference between an OPEN cell foam and a sheet of dimples.
    """
    import numpy as np
    cells = int(cells)
    ci = np.arange(cells)
    jx = jitter * (rng.random((cells, cells)) - 0.5)
    jy = jitter * (rng.random((cells, cells)) - 0.5)
    px = (ci[None, :] + 0.5 + jx) / cells                       # [cell_y, cell_x]
    py = (ci[:, None] + 0.5 + jy) / cells
    rad = rng.uniform(radius[0], radius[1], (cells, cells)) / cells
    dep = rng.uniform(depth_lo, depth_hi, (cells, cells))
    tnt = rng.random((cells, cells))

    u = ((np.arange(size) + 0.5) / size)[None, :]                # column -> u
    v = ((np.arange(size) + 0.5) / size)[:, None]                # row    -> v
    cu = np.floor(u * cells).astype(np.int64)
    cv = np.floor(v * cells).astype(np.int64)

    acc = np.zeros((size, size))
    tint = np.zeros((size, size))
    for dy in range(-reach, reach + 1):
        for dx in range(-reach, reach + 1):
            # toroidal neighbour: the index wraps, and the feature point is
            # carried across the wrap by a whole tile so the distance stays right
            ix = np.floor_divide(cu + dx, cells)                 # -1 / 0 / +1
            iy = np.floor_divide(cv + dy, cells)
            jx = (cu + dx) % cells
            jy = (cv + dy) % cells
            fx = px[jy, jx] + ix
            fy = py[jy, jx] + iy
            d = np.sqrt((u - fx) ** 2 + (v - fy) ** 2)
            t = np.clip(d / np.maximum(rad[jy, jx], 1e-9), 0.0, 1.0)
            bowl = dep[jy, jx] * (1.0 - t * t) ** 2
            tint = np.where(bowl > acc, tnt[jy, jx], tint)
            # ⚠ the smooth max is only valid where BOTH cavities are really
            # present: fed two zeros it returns k/4, so an unguarded running
            # _smax over 25 neighbours accumulates a bias that varies with how
            # many neighbours happened to miss. Round the real intersections,
            # take a plain max everywhere else.
            both = (acc > 1e-6) & (bowl > 1e-6)
            acc = np.where(both, _smax(acc, bowl, blend), np.maximum(acc, bowl))
    return np.clip(acc, 0.0, 1.0), tint


def sponge_height(size, cells, seed):
    """Height field + per-pore tint for open-cell cellulose sponge, seamless."""
    import numpy as np
    rng = np.random.default_rng(seed)

    # Primary pores. jitter 0.9 of a cell and a 0.62..1.15 cell radius spread is
    # what stops the lattice from being legible -- with jitter 0 you can count the
    # rows, which is the failure mode this whole function exists to avoid.
    big, tint = _cavity_layer(size, cells, rng, jitter=0.90,
                              radius=(0.62, 1.15), depth_lo=0.55, depth_hi=1.00,
                              blend=0.10)
    # Secondary pores in the walls. Cellulose sponge is bimodal: big holes with a
    # visibly porous skin between them.
    small, _ = _cavity_layer(size, max(3, int(cells * 2.4)), rng, jitter=0.95,
                             radius=(0.45, 0.95), depth_lo=0.18, depth_hi=0.40,
                             blend=0.06)

    eaten = _smax(big, small, 0.07)
    h = 1.0 - np.clip(eaten, 0.0, 1.0)

    # Cellulose fibre. Low amplitude on purpose: this is differentiated at one
    # pixel, so amplitude/wavelength -- not amplitude -- is what reaches the normal
    # map, and a loud fine octave turns cells into mush.
    h = h + 0.030 * (_fbm(rng, size, cells * 3, 3) - 0.5)
    # The slab is not flat; a very low frequency undulation keeps a flat-lit
    # render from looking like a decal.
    h = h + 0.055 * (_value_noise(rng, size, 3) - 0.5)
    return _norm01(h), tint, np.clip(eaten, 0.0, 1.0)


# --------------------------------------------------------------------------- #
# scrub pad: nonwoven tangled filament mat                                     #
# --------------------------------------------------------------------------- #

def scrubpad_height(size, seed, filaments=16, base_freq=13):
    """Tangled-filament abrasive mat, seamless by integer frequencies.

    A nonwoven pad is not a foam and not a weave: it is filaments laid down in
    every direction and bonded where they cross. Each filament layer here is a
    ridge of a plane wave with an INTEGER number of cycles per tile in both u and
    v, so it tiles exactly no matter what direction it runs; taking a smooth max
    over many random directions gives crossing strands with no preferred axis --
    which also happens to be the property that survives UV shear.
    """
    import numpy as np
    rng = np.random.default_rng(seed)
    u = ((np.arange(size) + 0.5) / size)[None, :]
    v = ((np.arange(size) + 0.5) / size)[:, None]

    h = np.zeros((size, size))
    for _ in range(filaments):
        ang = rng.uniform(0.0, math.pi)
        f = base_freq * rng.uniform(0.75, 1.35)
        # snap to integers so the wave closes on the torus
        p = max(1, int(round(f * math.cos(ang))))
        q = max(1, int(round(f * math.sin(ang))))
        if rng.random() < 0.5:
            q = -q
        phase = rng.uniform(0.0, 2 * math.pi)
        s = np.sin(2 * math.pi * (p * u + q * v) + phase)
        ridge = np.power(np.clip(1.0 - np.abs(s), 0.0, 1.0), 0.35)   # thin strand
        ridge = ridge * rng.uniform(0.65, 1.0)
        both = (h > 1e-6) & (ridge > 1e-6)                            # see _cavity_layer
        h = np.where(both, _smax(h, ridge, 0.16), np.maximum(h, ridge))

    # Voids: an abrasive pad is mostly air, so knock holes through the mat.
    voids, _ = _cavity_layer(size, 9, rng, jitter=0.95, radius=(0.55, 1.10),
                             depth_lo=0.35, depth_hi=0.75, blend=0.10)
    h = np.clip(h - voids, 0.0, None)
    h = h + 0.05 * (_fbm(rng, size, 40, 3) - 0.5)
    return _norm01(h)


# --------------------------------------------------------------------------- #
# woven cloth                                                                  #
# --------------------------------------------------------------------------- #

def _weave_pattern(n, kind):
    """Boolean float matrix W[iv, iu] = +1 where the WARP floats over the weft."""
    import numpy as np
    ii = np.arange(n)
    if kind == "plain":
        m = (ii[None, :] + ii[:, None]) % 2 == 0
    elif kind == "twill":
        m = (ii[None, :] - ii[:, None]) % 4 < 2
    else:                                                        # pragma: no cover
        raise ValueError("unknown weave %r" % kind)
    return np.where(m, 1.0, -1.0)


def _thread_profile(t, width):
    """Cross-section across a thread, as a PARABOLA rather than a semicircle.

    ⚠ A semicircle is the intuitive yarn cross-section and it is the wrong one
    here: sqrt(1 - x^2) meets the surface with INFINITE slope at x = +/-1, so
    every thread edge becomes a vertical cliff. Measured on the first attempt
    that cost 33 texel columns at a 32/255 step and drove the normal map's 99th
    percentile tilt to 78.5 degrees -- i.e. most of the map's dynamic range was
    being spent on wall texels nobody sees, which also made the PNG nearly
    incompressible.

    A parabola has finite slope at the edge, and -- because it is even in x and
    `t` is a symmetric sawtooth -- it continues CONTINUOUSLY into negative values
    past the thread edge, which forms the valley between threads for free. So
    there is no mask, no floor value, and no discontinuity anywhere in the field.
    """
    x = (t - 0.5) / (0.5 * width)
    return 1.0 - x * x


def weave_height(size, threads, kind, seed, width=0.93, amp=0.52, rad=0.62):
    """Height field of a real over-under weave, seamless over [0,1)^2.

    The float pattern is a small n x n matrix; each thread's CENTRELINE elevation
    is that matrix smoothly interpolated ALONG the thread. Doing it that way
    rather than with a closed-form cosine is what lets plain and twill share one
    code path -- and it is exactly periodic because the interpolation indexes the
    matrix modulo n.

    The surface is the smooth maximum of the warp sheet and the weft sheet, so
    whichever thread is floating is the one you see, and the crossover is rounded
    instead of creased.
    """
    import numpy as np
    n = int(threads)
    rng = np.random.default_rng(seed)
    W = _weave_pattern(n, kind)

    U = ((np.arange(size) + 0.5) / size) * n                     # along columns
    V = ((np.arange(size) + 0.5) / size) * n                     # along rows
    iu = np.floor(U).astype(np.int64) % n
    iv = np.floor(V).astype(np.int64) % n
    tu = U - np.floor(U)
    tv = V - np.floor(V)

    # centres of the float cells sit at half-integer thread coordinates
    Uc, Vc = U - 0.5, V - 0.5
    ju0 = np.floor(Uc).astype(np.int64) % n
    jv0 = np.floor(Vc).astype(np.int64) % n
    ju1, jv1 = (ju0 + 1) % n, (jv0 + 1) % n
    fu = _smoothstep(Uc - np.floor(Uc))
    fv = _smoothstep(Vc - np.floor(Vc))

    # warp runs along v: interpolate the pattern down the rows at fixed column
    e_warp = (W[np.ix_(jv0, iu)] * (1 - fv)[:, None] + W[np.ix_(jv1, iu)] * fv[:, None])
    # weft runs along u: interpolate across the columns at fixed row, complemented
    e_weft = -(W[np.ix_(iv, ju0)] * (1 - fu)[None, :] + W[np.ix_(iv, ju1)] * fu[None, :])

    p_warp = _thread_profile(tu, width)                          # varies along columns
    p_weft = _thread_profile(tv, width)                          # varies along rows

    # per-thread slub: real yarn is not a constant diameter. Indexed by thread
    # number, so it repeats with the tile by construction.
    slub_w = rng.uniform(0.90, 1.10, n)[iu]
    slub_f = rng.uniform(0.90, 1.10, n)[iv]

    # yarn twist: fine helical ridging along each thread, integer cycles per tile
    tw = int(max(1, round(n * 2.5)))
    ph_w = rng.uniform(0, 2 * math.pi, n)[iu]
    ph_f = rng.uniform(0, 2 * math.pi, n)[iv]
    # the twist rides ON the thread, so it is masked by the (clipped) profile --
    # otherwise the ridging carries on across the gaps and reads as corduroy
    crown_w = np.clip(p_warp, 0.0, 1.0)[None, :]
    crown_f = np.clip(p_weft, 0.0, 1.0)[:, None]
    twist_w = 0.055 * crown_w * np.sin(2 * math.pi * tw * (V / n)[:, None] + ph_w[None, :])
    twist_f = 0.055 * crown_f * np.sin(2 * math.pi * tw * (U / n)[None, :] + ph_f[:, None])

    h_warp = amp * e_warp + rad * (p_warp[None, :] * slub_w[None, :]) + twist_w
    h_weft = amp * e_weft + rad * (p_weft[:, None] * slub_f[:, None]) + twist_f

    h = _smax(h_warp, h_weft, 0.10)
    # ⚠ keep the fuzz octave COARSE. It is differentiated at one texel, so what
    # reaches the normal map is amplitude/wavelength, not amplitude: an n*4
    # lattice (5 px per cell at 1024) buried the weave in grain and roughly
    # doubled the PNG.
    h = h + 0.018 * (_fbm(rng, size, max(4, n // 2), 3) - 0.5)
    return _norm01(h), (h_warp > h_weft)


# --------------------------------------------------------------------------- #
# map assembly                                                                 #
# --------------------------------------------------------------------------- #

def normal_from_height(h01, relief_m, tile_m):
    """Tangent-space normal from a WRAPPED central difference, in real slope units.

    ⚠ ny = -dh/d(row). See the module docstring: this engine's v increases DOWN
    the image, so this is the "green is down" / DirectX labelling. It is the
    correct map for WREN, and a stock OpenGL/+Y map would light every bump as a
    dent.
    """
    import numpy as np
    size = h01.shape[0]
    dpix = tile_m / float(size)                                  # metres per texel
    hm = h01 * relief_m
    dhdx = (np.roll(hm, -1, axis=1) - np.roll(hm, 1, axis=1)) / (2.0 * dpix)
    dhdy = (np.roll(hm, -1, axis=0) - np.roll(hm, 1, axis=0)) / (2.0 * dpix)
    nx, ny = -dhdx, -dhdy
    nz = np.ones_like(h01)
    inv = 1.0 / np.sqrt(nx * nx + ny * ny + nz * nz)
    return np.stack([nx * inv, ny * inv, nz * inv], axis=-1)


def ambient_occlusion(h01, radii, strength, floor):
    """Cheap but honest AO: how far a texel sits below its own neighbourhood,
    averaged over several radii so both a pore and a pore-cluster darken."""
    import numpy as np
    occ = np.zeros_like(h01)
    for r in radii:
        occ += np.clip((_blur_wrap(h01, r) - h01) * strength, 0.0, 1.0)
    occ /= max(len(radii), 1)
    return np.clip(1.0 - occ, floor, 1.0)


def _srgb(lin):
    import numpy as np
    lin = np.clip(lin, 0.0, 1.0)
    return np.where(lin <= 0.0031308, lin * 12.92, 1.055 * lin ** (1 / 2.4) - 0.055)


def _u8(a):
    import numpy as np
    return (np.clip(a, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)


def _save(img_arr, path, mode="RGB"):
    from PIL import Image
    Image.fromarray(img_arr, mode).save(path, optimize=True)
    return os.path.getsize(path)


def write_set(out_dir, name, base_lin_rgb, normal, rough, ao, verbose=True):
    """Write the four PNGs and return {suffix: bytes}."""
    import numpy as np
    sizes = {}
    os.makedirs(out_dir, exist_ok=True)

    srgb = _srgb(base_lin_rgb)
    sizes["basecolor"] = _save(_u8(srgb), os.path.join(out_dir, "%s_basecolor.png" % name))

    # RAW, no sRGB curve: a normal is a vector, not a colour.
    sizes["normal"] = _save(_u8(normal * 0.5 + 0.5), os.path.join(out_dir, "%s_normal.png" % name))

    g = _u8(rough)
    sizes["roughness"] = _save(np.stack([g, g, g], -1), os.path.join(out_dir, "%s_roughness.png" % name))

    g = _u8(ao)
    sizes["occlusion"] = _save(np.stack([g, g, g], -1), os.path.join(out_dir, "%s_occlusion.png" % name))

    if verbose:
        total = sum(sizes.values())
        print("  %-24s %s  (%.0f KiB total)"
              % (name, " ".join("%s=%.0fK" % (k, v / 1024.0) for k, v in sizes.items()),
                 total / 1024.0))
    return sizes


# --------------------------------------------------------------------------- #
# the four sets                                                                #
# --------------------------------------------------------------------------- #

def build_sponge(size, seed, base_lin, cells=14):
    import numpy as np
    h, tint, eaten = sponge_height(size, cells, seed)

    # Base colour. Two independent variations, because a sponge has both:
    # value modulation WITHIN a pore (from `h`) and a slight difference BETWEEN
    # pores (from `tint`, the per-pore id). Kept low contrast so it reinforces
    # the normal map under grazing light instead of fighting it.
    rng = np.random.default_rng(seed ^ 0x5EED)
    mottle = _fbm(rng, size, 6, 4)
    shade = 0.66 + 0.30 * h + 0.10 * (tint - 0.5) + 0.10 * (mottle - 0.5)
    lin = np.array(base_lin, dtype=np.float64)[None, None, :]
    rgb = lin * shade[..., None]
    # pore interiors desaturate slightly toward the wet-cellulose grey
    rgb = rgb * (1.0 - 0.18 * eaten[..., None]) + 0.035 * eaten[..., None]

    normal = normal_from_height(h, relief_m=0.0016, tile_m=0.04)
    rough = 0.95 + (0.74 - 0.95) * h + 0.02 * (mottle - 0.5)
    ao = ambient_occlusion(h, radii=(max(2, size // 128), max(4, size // 42)),
                           strength=2.6, floor=0.16)
    return rgb, normal, np.clip(rough, 0.0, 1.0), ao


def build_scrubpad(size, seed, base_lin):
    import numpy as np
    h = scrubpad_height(size, seed)
    rng = np.random.default_rng(seed ^ 0xABBA)
    mottle = _fbm(rng, size, 8, 4)
    shade = 0.52 + 0.62 * h + 0.16 * (mottle - 0.5)
    lin = np.array(base_lin, dtype=np.float64)[None, None, :]
    rgb = lin * shade[..., None]

    normal = normal_from_height(h, relief_m=0.0010, tile_m=0.04)
    rough = 0.98 + (0.80 - 0.98) * h
    ao = ambient_occlusion(h, radii=(max(2, size // 160), max(4, size // 48)),
                           strength=3.0, floor=0.10)
    return rgb, normal, np.clip(rough, 0.0, 1.0), ao


def build_woven(size, seed, base_lin, threads, kind):
    import numpy as np
    h, warp_up = weave_height(size, threads, kind, seed)
    rng = np.random.default_rng(seed ^ 0x1234)

    # per-thread colour jitter: warp and weft are never dyed identically, and
    # that tiny difference is a big part of why cloth reads as cloth
    n = threads
    U = ((np.arange(size) + 0.5) / size) * n
    V = ((np.arange(size) + 0.5) / size) * n
    iu = np.floor(U).astype(np.int64) % n
    iv = np.floor(V).astype(np.int64) % n
    jw = rng.uniform(0.955, 1.045, n)[iu][None, :]
    jf = rng.uniform(0.955, 1.045, n)[iv][:, None]
    yarn = np.where(warp_up, np.broadcast_to(jw, (size, size)),
                    np.broadcast_to(jf, (size, size)))

    shade = (0.70 + 0.36 * h) * yarn
    lin = np.array(base_lin, dtype=np.float64)[None, None, :]
    rgb = lin * shade[..., None]

    normal = normal_from_height(h, relief_m=0.00035, tile_m=0.02)
    rough = 0.88 + (0.66 - 0.88) * h
    ao = ambient_occlusion(h, radii=(max(2, size // 200), max(3, size // 80)),
                           strength=2.2, floor=0.30)
    return rgb, normal, np.clip(rough, 0.0, 1.0), ao


# --------------------------------------------------------------------------- #
# verification                                                                 #
# --------------------------------------------------------------------------- #

def _step_profile(a, axis):
    """Mean |difference| between every adjacent line pair, WRAPPED, so element 0
    is the tile seam and 1..N-1 are the interior."""
    import numpy as np
    d = np.abs(a - np.roll(a, 1, axis=axis))
    return d.mean(axis=1 - axis)


def seam_report(label, arr):
    """Is the wrap edge indistinguishable from the interior edges of the SAME image?

    ⚠ THE OBVIOUS METRIC IS WRONG AND IT COST A ROUND HERE. Comparing the seam
    step to the MEAN interior step flags any texture with sharp periodic features
    as broken: the plain weave's tile edge necessarily lands on a thread boundary,
    thread boundaries are the sharpest edges in the image, and the mean is
    dominated by the smooth texels between them. That scored 4.19 on a texture
    whose seam step was measurably SMALLER than 33 of its own interior steps.

    So the verdict here is `seam / max(interior)`. A real discontinuity is the
    largest step in the image by a wide margin (ratio >> 1); a seam that merely
    lands on a feature edge sits at or below the other instances of that same
    feature (ratio <= 1). The percentile rank is reported alongside because it
    is the number that explains the ratio, and the mean ratio is kept purely as
    a descriptive figure -- it is NOT the pass/fail.
    """
    import numpy as np
    a = arr.astype(np.float64)
    if a.ndim == 3:
        a = a.mean(axis=2)
    out = {"label": label}
    ok = True
    for tag, axis in (("h", 1), ("v", 0)):
        st = _step_profile(a, axis)
        seam, interior = st[0], st[1:]
        rmax = seam / max(float(interior.max()), 1e-12)
        out["ratio_mean_" + tag] = seam / max(float(interior.mean()), 1e-12)
        out["ratio_max_" + tag] = rmax
        out["rank_" + tag] = float((interior < seam).mean())
        ok = ok and rmax <= 1.15
    out["ok"] = ok
    return out


def normal_report(label, n_float, path=None):
    """Sanity of the encoded normal map: unit length after the 8-bit round trip,
    z strictly positive, and a tilt distribution that is neither flat nor mush."""
    import numpy as np
    dec = n_float
    if path is not None:
        from PIL import Image
        dec = np.asarray(Image.open(path).convert("RGB"), dtype=np.float64) / 255.0 * 2.0 - 1.0
    ln = np.sqrt((dec ** 2).sum(axis=2))
    tilt = np.degrees(np.arccos(np.clip(dec[..., 2] / np.maximum(ln, 1e-9), -1, 1)))
    flat = float((tilt < 1.0).mean())
    return {"label": label,
            "len_min": float(ln.min()), "len_max": float(ln.max()),
            "z_min": float(dec[..., 2].min()),
            "tilt_p50": float(np.percentile(tilt, 50)),
            "tilt_p99": float(np.percentile(tilt, 99)),
            "tilt_max": float(tilt.max()),
            "flat_frac": flat,
            "ok": abs(ln.min() - 1.0) < 0.02 and abs(ln.max() - 1.0) < 0.02
                  and dec[..., 2].min() > 0.0 and flat < 0.25}


# --------------------------------------------------------------------------- #
# previews                                                                     #
# --------------------------------------------------------------------------- #

def _tile2x2(a):
    import numpy as np
    return np.concatenate([np.concatenate([a, a], axis=1)] * 2, axis=0)


def contact_sheet(out_png, name, base_srgb_u8, normal_u8, rough_u8, ao_u8, cell=384):
    """2x2-tiled thumbnails of all four maps, side by side, seam crosshairs drawn.

    The point of tiling each map 2x2 is that a seam becomes a straight line
    through the MIDDLE of the thumbnail, where the eye actually notices it,
    instead of sitting at the edge where it is invisible.
    """
    import numpy as np
    from PIL import Image, ImageDraw

    panels = []
    for arr in (base_srgb_u8, normal_u8, rough_u8, ao_u8):
        t = _tile2x2(arr)
        im = Image.fromarray(t, "RGB").resize((cell, cell), Image.LANCZOS)
        panels.append(im)

    pad, top = 8, 18
    sheet = Image.new("RGB", (4 * cell + 5 * pad, cell + top + 2 * pad), (24, 24, 26))
    d = ImageDraw.Draw(sheet)
    for i, (im, lab) in enumerate(zip(panels, ("basecolor", "normal", "roughness", "occlusion"))):
        x = pad + i * (cell + pad)
        sheet.paste(im, (x, top + pad))
        d.text((x + 2, 4), "%s  %s  (2x2 tiled)" % (name, lab), fill=(210, 210, 214))
        # crosshair on the seam that the 2x2 tiling put at the centre
        d.line([(x + cell // 2, top + pad), (x + cell // 2, top + pad + cell)], fill=(255, 64, 64))
        d.line([(x, top + pad + cell // 2), (x + cell, top + pad + cell // 2)], fill=(255, 64, 64))
    sheet.save(out_png)
    return os.path.getsize(out_png)


def lit_preview(out_png, name, base_lin, normal, rough, ao, size=512,
                light=(-0.42, -0.50, 0.76), tiles=2):
    """A real shading pass -- Lambert + GGX + AO-modulated ambient -- over a 2x2
    tiling, so the maps are judged the way the engine will use them.

    A contact sheet cannot tell you whether a normal map reads as geometry; only
    a specular lobe can. The frame here matches the engine's: x is +u (right),
    y is +v (DOWN the image), z is out of the surface.
    """
    import numpy as np
    from PIL import Image, ImageDraw

    N = _tile2x2(normal)
    R = _tile2x2(rough)
    A = _tile2x2(ao)
    C = _tile2x2(base_lin)
    N = N / np.maximum(np.sqrt((N ** 2).sum(-1, keepdims=True)), 1e-9)

    L = np.array(light, dtype=np.float64)
    L /= np.linalg.norm(L)
    Vv = np.array([0.0, 0.0, 1.0])
    H = L + Vv
    H /= np.linalg.norm(H)

    NdL = np.clip((N * L).sum(-1), 0, 1)
    NdV = np.clip((N * Vv).sum(-1), 1e-4, 1)
    NdH = np.clip((N * H).sum(-1), 0, 1)
    VdH = max(float(np.dot(Vv, H)), 1e-4)

    a2 = np.clip(R, 0.04, 1.0) ** 4
    denom = NdH * NdH * (a2 - 1.0) + 1.0
    D = a2 / (math.pi * denom * denom)
    kk = (R + 1.0) ** 2 / 8.0
    G = (NdL / (NdL * (1 - kk) + kk + 1e-9)) * (NdV / (NdV * (1 - kk) + kk + 1e-9))
    F = 0.04 + 0.96 * (1.0 - VdH) ** 5
    spec = (D * G * F) / (4.0 * NdL * NdV + 1e-6) * NdL

    key = 2.9
    diffuse = C * (NdL[..., None] / math.pi) * key
    ambient = C * 0.16 * A[..., None]
    out = diffuse + ambient + (spec * key)[..., None] * 0.75
    out = _srgb(out / (1.0 + out))                                # Reinhard, then sRGB
    full = _u8(out)                                               # 2*size square

    # ⚠ TWO PANELS, because one cannot answer both questions and a single
    # downsampled overview silently lies about the second. POINT-SAMPLING the
    # overview aliased 96 threads into 512 px and turned a working weave into a
    # flat grey screen -- which reads as "the normal map is mush" when the map
    # was fine. So: the overview is AREA-AVERAGED (a real mip, so it shows what
    # the material looks like at distance and whether the seam shows), and the
    # detail crop is 1:1 with no resampling at all (so it shows the structure).
    n2 = full.shape[0]
    k = max(1, n2 // size)
    if n2 % size == 0 and k > 1:
        ov = full.reshape(size, k, size, k, 3).mean(axis=(1, 3))
        ov = ov.astype(np.uint8)
    else:
        ov = full[::k, ::k]
    o0 = (n2 - size) // 2
    detail = full[o0:o0 + size, o0:o0 + size]

    pad, top = 8, 18
    sheet = Image.new("RGB", (2 * size + 3 * pad, size + top + 2 * pad), (24, 24, 26))
    d = ImageDraw.Draw(sheet)
    sheet.paste(Image.fromarray(ov, "RGB"), (pad, top + pad))
    sheet.paste(Image.fromarray(detail, "RGB"), (2 * pad + size, top + pad))
    d.text((pad, 4), "%s   lit 2x2, area-averaged   (red = tile seam)" % name,
           fill=(230, 230, 234))
    d.text((2 * pad + size, 4), "%s   lit 1:1 detail crop (no resampling)" % name,
           fill=(230, 230, 234))
    cx, cy = pad + size // 2, top + pad + size // 2
    d.line([(cx, top + pad), (cx, top + pad + size)], fill=(255, 70, 70))
    d.line([(pad, cy), (pad + size, cy)], fill=(255, 70, 70))
    sheet.save(out_png)
    return os.path.getsize(out_png)


# --------------------------------------------------------------------------- #

def main():
    # ⚠ `--help` CRASHES WITHOUT THIS ON A STOCK WINDOWS CONSOLE. The docstring
    # below is printed by argparse, it contains non-ASCII (the warning glyphs),
    # and a cp1252 stdout raises UnicodeEncodeError -- so the script dies on its
    # own help text before it can do anything. Measured here on Python 3.12 /
    # cp1252. (The sibling make_cloth_textures.py has the same latent bug.)
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            try:
                stream.reconfigure(encoding="utf-8", errors="replace")
            except (ValueError, OSError):                        # pragma: no cover
                pass

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--set", dest="sets", action="append", choices=SETS + ("all",),
                    help="repeatable. Default is the two SHIPPED sets (sponge, "
                         "woven); `--set all` adds the two optional variants "
                         "(scrubpad, twill), which cost ~5 MiB more in a git repo "
                         "and are not referenced by any world.")
    ap.add_argument("--size", type=int, default=1024)
    ap.add_argument("--seed", type=int, default=20260815)
    ap.add_argument("--sponge-pores", type=int, default=14,
                    help="primary pore cells across the tile (default 14 over 4 cm)")
    ap.add_argument("--sponge-color", type=float, nargs=3, default=(0.93, 0.78, 0.16),
                    help="LINEAR rgb of the cellulose; matches the demo's "
                         "diffuseColor, which WREN also consumes linearly")
    ap.add_argument("--scrubpad-color", type=float, nargs=3, default=(0.030, 0.115, 0.055))
    ap.add_argument("--weave-threads", type=int, default=48,
                    help="threads across the tile; must be even (plain) / a "
                         "multiple of 4 (twill) or the pattern will not tile")
    ap.add_argument("--weave-color", type=float, nargs=3, default=(0.82, 0.815, 0.80),
                    help="LINEAR rgb; near-neutral on purpose so a world can tint "
                         "it with the PBRAppearance baseColor field")
    ap.add_argument("--preview-dir", default=None,
                    help="also write contact sheets + lit previews here")
    ap.add_argument("--verify", action="store_true", default=True)
    ap.add_argument("--no-verify", dest="verify", action="store_false")
    args = ap.parse_args()

    try:
        import numpy as np
    except ImportError as exc:                                   # pragma: no cover
        print("needs numpy and Pillow: %s" % exc, file=sys.stderr)
        return 2
    try:
        from PIL import Image  # noqa: F401
    except ImportError as exc:                                   # pragma: no cover
        print("needs Pillow (see scripts/harness/README.md for which interpreter "
              "on Windows): %s" % exc, file=sys.stderr)
        return 2

    wanted = args.sets or list(SHIPPED)
    if "all" in wanted:
        wanted = list(SETS)

    n = args.weave_threads
    if "woven" in wanted and n % 2:
        print("--weave-threads must be even for a plain weave", file=sys.stderr)
        return 2
    if "twill" in wanted and n % 4:
        print("--weave-threads must be a multiple of 4 for a 2/2 twill", file=sys.stderr)
        return 2

    os.makedirs(args.out, exist_ok=True)
    if args.preview_dir:
        os.makedirs(args.preview_dir, exist_ok=True)

    recipes = {
        "sponge":   ("sponge_cellulose",     lambda: build_sponge(args.size, args.seed, args.sponge_color, args.sponge_pores)),
        "scrubpad": ("sponge_scrubpad",      lambda: build_scrubpad(args.size, args.seed + 7, args.scrubpad_color)),
        "woven":    ("woven_cotton",         lambda: build_woven(args.size, args.seed + 11, args.weave_color, n, "plain")),
        "twill":    ("woven_cotton_twill",   lambda: build_woven(args.size, args.seed + 13, args.weave_color, n, "twill")),
    }

    print("writing %d px sets to %s" % (args.size, args.out))
    grand_total = 0
    seams, normals = [], []
    for key in wanted:
        name, fn = recipes[key]
        rgb_lin, normal, rough, ao = fn()
        sizes = write_set(args.out, name, rgb_lin, normal, rough, ao)
        grand_total += sum(sizes.values())

        if args.verify:
            base_u8 = _u8(_srgb(rgb_lin))
            nrm_u8 = _u8(normal * 0.5 + 0.5)
            for lab, arr in (("basecolor", base_u8), ("normal", nrm_u8),
                             ("roughness", _u8(rough)), ("occlusion", _u8(ao))):
                seams.append(seam_report("%s/%s" % (name, lab), arr))
            normals.append(normal_report(name, None,
                                         path=os.path.join(args.out, "%s_normal.png" % name)))

        if args.preview_dir:
            base_u8 = _u8(_srgb(rgb_lin))
            nrm_u8 = _u8(normal * 0.5 + 0.5)
            r3 = np.stack([_u8(rough)] * 3, -1)
            a3 = np.stack([_u8(ao)] * 3, -1)
            contact_sheet(os.path.join(args.preview_dir, "%s_sheet.png" % name),
                          name, base_u8, nrm_u8, r3, a3)
            lit_preview(os.path.join(args.preview_dir, "%s_lit.png" % name),
                        name, rgb_lin, normal, rough, ao)

    print("total added bytes: %.2f MiB" % (grand_total / 1048576.0))

    if args.verify:
        print("\nSEAM CHECK   verdict = seam step / LARGEST interior step "
              "(<=1.15 passes: the")
        print("             wrap is no sharper than an edge the texture already "
              "contains).")
        print("             mean-ratio and rank are descriptive only -- see "
              "seam_report().")
        print("  %-38s %-19s %-19s" % ("", "horizontal", "vertical"))
        worst = 0.0
        for s in seams:
            worst = max(worst, s["ratio_max_h"], s["ratio_max_v"])
            print("  %-38s /max %.3f rank %2.0f%%  /max %.3f rank %2.0f%%  "
                  "(/mean %.2f, %.2f)  %s"
                  % (s["label"], s["ratio_max_h"], 100 * s["rank_h"],
                     s["ratio_max_v"], 100 * s["rank_v"],
                     s["ratio_mean_h"], s["ratio_mean_v"],
                     "OK" if s["ok"] else "FAIL"))
        print("  worst seam/max ratio: %.3f" % worst)

        print("\nNORMAL CHECK  (decoded from the written PNG)")
        for r in normals:
            print("  %-24s |n| %.4f..%.4f  z_min %.3f  tilt p50 %4.1f deg  p99 %4.1f  "
                  "max %4.1f  flat %.1f%%  %s"
                  % (r["label"], r["len_min"], r["len_max"], r["z_min"],
                     r["tilt_p50"], r["tilt_p99"], r["tilt_max"], 100 * r["flat_frac"],
                     "OK" if r["ok"] else "FAIL"))

        bad = [s for s in seams if not s["ok"]] + [r for r in normals if not r["ok"]]
        if bad:
            print("\n%d check(s) FAILED" % len(bad), file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
