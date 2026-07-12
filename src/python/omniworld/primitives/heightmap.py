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

"""Heightmap primitives.

This module is the T1.2 deliverable from the procedural-world-generation
plan. Everything outdoor that needs a surface — forests, deserts, river
valleys — starts here, and later tiers layer on (splatmap, erosion with
sediment, learned styles).

Contents:

- ``Heightmap`` — a float-grid with row-major storage and bilinear
  ``sample(x, y)``. Optionally backed by numpy when available, list-of-float
  otherwise, with byte-identical results on a single (platform, Python)
  pair.
- Noise: ``fbm``, ``ridged``, ``worley`` — seeded deterministically via a
  blake2b-based hash so two invocations with the same seed produce the
  same grid without reliance on ``random.Random`` iteration order.
- Masks: ``mask_radial``, ``mask_rect``, ``mask_polygon`` (convex only
  at Tier 1; general polygons via scanline in a later tier).
- Composition: ``blend``, ``apply_mask``, ``stamp`` (add / sub / max /
  min / replace).
- Erosion: ``erode_thermal`` (talus angle redistribution) and
  ``erode_hydraulic`` (a small particle simulator). Neither pretends to
  be geology; both are good enough to break the "symmetrically-generated"
  tell of raw fBm.

Determinism contract:

- Every generator takes an explicit ``seed: int``. No hidden global RNG.
- Outputs are byte-stable on a single platform.
- Cross-platform equivalence is best-effort (float summation order is
  deterministic, but platform ``math`` tiny differences may show up; the
  test suite asserts bit-identity on one platform and within-tolerance
  equivalence on others).
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from typing import Iterable, Sequence

try:  # pragma: no cover - optional import
    import numpy as _np  # type: ignore
    _HAS_NUMPY = True
except ImportError:  # pragma: no cover
    _np = None  # type: ignore
    _HAS_NUMPY = False


# ---------------------------------------------------------------------------
# Heightmap container
# ---------------------------------------------------------------------------


@dataclass
class Heightmap:
    """Row-major float grid. ``data[j * width + i]`` for column ``i``, row ``j``.

    Coordinates: ``i`` runs 0..width-1 along x, ``j`` runs 0..height-1
    along y. ``(0, 0)`` is the origin corner. Conversion to / from world
    coordinates is a recipe responsibility — this class only knows about
    its own grid.
    """

    width: int
    height: int
    data: list[float]

    def __post_init__(self) -> None:
        expected = self.width * self.height
        if len(self.data) != expected:
            raise ValueError(
                f"data length {len(self.data)} does not match "
                f"{self.width}x{self.height} = {expected}"
            )

    # --- classmethods ---

    @classmethod
    def zeros(cls, width: int, height: int) -> "Heightmap":
        return cls(width, height, [0.0] * (width * height))

    @classmethod
    def constant(cls, width: int, height: int, value: float) -> "Heightmap":
        return cls(width, height, [float(value)] * (width * height))

    # --- element access ---

    def get(self, i: int, j: int) -> float:
        return self.data[j * self.width + i]

    def set(self, i: int, j: int, v: float) -> None:
        self.data[j * self.width + i] = v

    def copy(self) -> "Heightmap":
        return Heightmap(self.width, self.height, list(self.data))

    # --- bilinear sampling ---

    def sample(self, x: float, y: float) -> float:
        """Bilinear interpolation at fractional grid coordinates.

        ``x`` and ``y`` are in grid units (0 <= x <= width - 1, same for
        y). Out-of-range values are clamped to the edge.
        """
        if x < 0.0:
            x = 0.0
        elif x > self.width - 1:
            x = float(self.width - 1)
        if y < 0.0:
            y = 0.0
        elif y > self.height - 1:
            y = float(self.height - 1)

        i0 = int(x)
        j0 = int(y)
        i1 = min(i0 + 1, self.width - 1)
        j1 = min(j0 + 1, self.height - 1)
        tx = x - i0
        ty = y - j0

        h00 = self.get(i0, j0)
        h10 = self.get(i1, j0)
        h01 = self.get(i0, j1)
        h11 = self.get(i1, j1)

        a = h00 * (1.0 - tx) + h10 * tx
        b = h01 * (1.0 - tx) + h11 * tx
        return a * (1.0 - ty) + b * ty

    # --- stats ---

    def min(self) -> float:
        return min(self.data)

    def max(self) -> float:
        return max(self.data)

    def range(self) -> tuple[float, float]:
        return self.min(), self.max()

    def normalise(self, lo: float = 0.0, hi: float = 1.0) -> "Heightmap":
        """Return a new Heightmap rescaled into ``[lo, hi]``.

        If the source is uniform the output is filled with ``lo`` — the
        caller probably has a bug, but we do not raise.
        """
        src_lo, src_hi = self.range()
        span = src_hi - src_lo
        if span == 0.0:
            return Heightmap.constant(self.width, self.height, lo)
        scale = (hi - lo) / span
        return Heightmap(
            self.width,
            self.height,
            [(v - src_lo) * scale + lo for v in self.data],
        )


# ---------------------------------------------------------------------------
# Deterministic per-cell value hash (used by noise generators)
# ---------------------------------------------------------------------------


def _hash_value(seed: int, i: int, j: int, salt: int = 0) -> float:
    """Deterministic uniform sample in ``[0, 1)`` keyed by (seed, i, j, salt).

    Blake2b so the output does not depend on ``PYTHONHASHSEED`` and is
    stable across platforms.
    """
    key = (seed & 0xFFFFFFFFFFFFFFFF).to_bytes(8, "little", signed=False)
    # Allow large negative indices (after frequency scaling) by using signed
    # encoding; salt lets callers (e.g. ridged vs fbm) decorrelate streams
    # cheaply without bumping the seed.
    msg = (
        i.to_bytes(8, "little", signed=True)
        + j.to_bytes(8, "little", signed=True)
        + salt.to_bytes(4, "little", signed=False)
    )
    digest = hashlib.blake2b(msg, digest_size=8, key=key).digest()
    n = int.from_bytes(digest, "little", signed=False)
    # [0, 1), 53-bit mantissa.
    return (n >> 11) * (1.0 / (1 << 53))


def _cell_value(seed: int, i: int, j: int, salt: int = 0) -> float:
    """Value noise gradient value in ``[-1, 1)``."""
    return _hash_value(seed, i, j, salt) * 2.0 - 1.0


def _smoothstep(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def _value_noise_2d(x: float, y: float, seed: int, salt: int = 0) -> float:
    """Classic value noise on an integer lattice with smoothstep interpolation.

    Returns a value in approximately ``[-1, 1)``.
    """
    i0 = math.floor(x)
    j0 = math.floor(y)
    tx = _smoothstep(x - i0)
    ty = _smoothstep(y - j0)

    v00 = _cell_value(seed, i0, j0, salt)
    v10 = _cell_value(seed, i0 + 1, j0, salt)
    v01 = _cell_value(seed, i0, j0 + 1, salt)
    v11 = _cell_value(seed, i0 + 1, j0 + 1, salt)

    a = v00 * (1.0 - tx) + v10 * tx
    b = v01 * (1.0 - tx) + v11 * tx
    return a * (1.0 - ty) + b * ty


# ---------------------------------------------------------------------------
# Noise: fbm, ridged, worley
# ---------------------------------------------------------------------------


def fbm(
    width: int,
    height: int,
    seed: int,
    *,
    octaves: int = 5,
    lacunarity: float = 2.0,
    gain: float = 0.5,
    base_period: float | None = None,
) -> Heightmap:
    """Fractal Brownian Motion — multi-octave value noise.

    - ``base_period`` is the side length in cells of one period of the
      lowest-frequency octave. Defaults to ``max(width, height)`` so the
      result has about one feature across the map.
    - ``octaves`` doubles the frequency each step; typical 4-6.
    - ``gain`` and ``lacunarity`` are the usual Musgrave parameters.

    Output is normalised into ``[0, 1]``.
    """
    if width <= 0 or height <= 0:
        raise ValueError("fbm: width and height must be positive")
    if octaves < 1:
        raise ValueError("fbm: octaves must be >= 1")
    base = float(base_period if base_period is not None else max(width, height))

    hm = Heightmap.zeros(width, height)
    for j in range(height):
        for i in range(width):
            amplitude = 1.0
            frequency = 1.0 / base
            total = 0.0
            norm = 0.0
            for octave in range(octaves):
                x = i * frequency
                y = j * frequency
                total += amplitude * _value_noise_2d(x, y, seed, salt=octave)
                norm += amplitude
                amplitude *= gain
                frequency *= lacunarity
            hm.set(i, j, total / norm if norm != 0.0 else 0.0)
    return hm.normalise(0.0, 1.0)


def ridged(
    width: int,
    height: int,
    seed: int,
    *,
    octaves: int = 5,
    lacunarity: float = 2.0,
    gain: float = 0.5,
    base_period: float | None = None,
    offset: float = 1.0,
) -> Heightmap:
    """Ridged multifractal — ``abs(noise)`` inverted into peaks.

    The classic Musgrave-style ridge operator: each octave becomes
    ``(offset - abs(n))**2``, multiplied by a signal that decays with
    amplitude. Produces sharp canyon / mesa ridges where fbm only gives
    rolling hills.
    """
    if width <= 0 or height <= 0:
        raise ValueError("ridged: width and height must be positive")
    if octaves < 1:
        raise ValueError("ridged: octaves must be >= 1")
    base = float(base_period if base_period is not None else max(width, height))

    hm = Heightmap.zeros(width, height)
    for j in range(height):
        for i in range(width):
            amplitude = 1.0
            frequency = 1.0 / base
            total = 0.0
            weight = 1.0
            for octave in range(octaves):
                x = i * frequency
                y = j * frequency
                n = _value_noise_2d(x, y, seed, salt=0x100 + octave)
                signal = offset - abs(n)
                signal = signal * signal * weight
                weight = max(0.0, min(1.0, signal * gain))
                total += signal * amplitude
                amplitude *= gain
                frequency *= lacunarity
            hm.set(i, j, total)
    return hm.normalise(0.0, 1.0)


def worley(
    width: int,
    height: int,
    seed: int,
    *,
    points_per_cell: int = 1,
    cell_size: float | None = None,
    metric: str = "euclidean",
    variant: str = "f1",
) -> Heightmap:
    """Cellular / Worley noise via a jittered-grid point distribution.

    For each output pixel, scan the local 3x3 block of feature points and
    return the distance to the nearest one (``f1``), second nearest
    (``f2``), or their difference (``f2_f1``).

    - ``cell_size`` is the side length in pixels of one feature cell.
      Default: 1/8 of the smaller dimension.
    - ``metric`` is ``"euclidean"`` or ``"manhattan"``.
    - Output is normalised into ``[0, 1]``.
    """
    if variant not in {"f1", "f2", "f2_f1"}:
        raise ValueError(f"worley: unknown variant {variant!r}")
    if metric not in {"euclidean", "manhattan"}:
        raise ValueError(f"worley: unknown metric {metric!r}")

    cs = float(cell_size if cell_size is not None else max(min(width, height) / 8.0, 1.0))

    def distance(dx: float, dy: float) -> float:
        if metric == "euclidean":
            return math.sqrt(dx * dx + dy * dy)
        return abs(dx) + abs(dy)

    hm = Heightmap.zeros(width, height)
    for j in range(height):
        cy = j / cs
        cj = int(math.floor(cy))
        for i in range(width):
            cx = i / cs
            ci = int(math.floor(cx))
            f1 = math.inf
            f2 = math.inf
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    ncj = cj + dj
                    nci = ci + di
                    for p in range(points_per_cell):
                        # Place a jittered point inside this cell.
                        px = nci + _hash_value(seed, nci, ncj, salt=2 * p)
                        py = ncj + _hash_value(seed, nci, ncj, salt=2 * p + 1)
                        d = distance(cx - px, cy - py)
                        if d < f1:
                            f2 = f1
                            f1 = d
                        elif d < f2:
                            f2 = d
            if variant == "f1":
                v = f1
            elif variant == "f2":
                v = f2
            else:
                v = f2 - f1
            hm.set(i, j, v)
    return hm.normalise(0.0, 1.0)


# ---------------------------------------------------------------------------
# Masks
# ---------------------------------------------------------------------------


def mask_radial(
    width: int,
    height: int,
    *,
    centre: tuple[float, float] | None = None,
    radius: float | None = None,
    falloff: str = "smoothstep",
) -> Heightmap:
    """A mask that is 1 at the centre and falls to 0 at the radius.

    Useful as the ``flat_centre`` shaping mask in outdoor recipes (the
    existing Omnibot demo uses a Gaussian for the same effect).
    """
    cx, cy = (
        centre
        if centre is not None
        else ((width - 1) / 2.0, (height - 1) / 2.0)
    )
    r = float(radius if radius is not None else min(width, height) / 2.0)
    if r <= 0.0:
        raise ValueError("mask_radial: radius must be positive")

    hm = Heightmap.zeros(width, height)
    for j in range(height):
        for i in range(width):
            d = math.sqrt((i - cx) ** 2 + (j - cy) ** 2)
            t = max(0.0, min(1.0, 1.0 - d / r))
            if falloff == "smoothstep":
                t = _smoothstep(t)
            elif falloff == "linear":
                pass
            elif falloff == "gaussian":
                t = math.exp(-((d / r) ** 2) * 4.0)
            else:
                raise ValueError(f"mask_radial: unknown falloff {falloff!r}")
            hm.set(i, j, t)
    return hm


def mask_rect(
    width: int,
    height: int,
    *,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    feather: int = 0,
) -> Heightmap:
    """A mask that is 1 inside ``[x0, x1) x [y0, y1)`` and 0 outside, with
    a linear feather in both directions."""
    if x0 >= x1 or y0 >= y1:
        raise ValueError("mask_rect: empty rectangle")
    if feather < 0:
        raise ValueError("mask_rect: feather must be >= 0")

    hm = Heightmap.zeros(width, height)
    for j in range(height):
        for i in range(width):
            if i < x0 or i >= x1 or j < y0 or j >= y1:
                hm.set(i, j, 0.0)
                continue
            if feather == 0:
                hm.set(i, j, 1.0)
                continue
            dx = min(i - x0, x1 - 1 - i)
            dy = min(j - y0, y1 - 1 - j)
            d = min(dx, dy)
            t = max(0.0, min(1.0, d / feather))
            hm.set(i, j, _smoothstep(t))
    return hm


def mask_polygon(
    width: int,
    height: int,
    polygon: Sequence[tuple[float, float]],
) -> Heightmap:
    """A mask that is 1 inside a general polygon (even-odd rule), 0 outside.

    Uses a scanline rasteriser: for each row, count intersections with
    the polygon edges. No antialiasing — callers that want a feathered
    edge combine with ``mask_rect`` or blur externally.
    """
    n = len(polygon)
    if n < 3:
        raise ValueError("mask_polygon: polygon must have >= 3 vertices")

    hm = Heightmap.zeros(width, height)
    for j in range(height):
        y = j + 0.5
        # Compute x-intersections for this scanline.
        xs: list[float] = []
        for k in range(n):
            ax, ay = polygon[k]
            bx, by = polygon[(k + 1) % n]
            if (ay > y) != (by > y):
                # Edge crosses scanline. Solve for x at y.
                t = (y - ay) / (by - ay)
                xs.append(ax + t * (bx - ax))
        xs.sort()
        # Fill between pairs.
        for p in range(0, len(xs) - 1, 2):
            x_lo = xs[p]
            x_hi = xs[p + 1]
            i_lo = max(0, int(math.ceil(x_lo - 0.5)))
            i_hi = min(width - 1, int(math.floor(x_hi - 0.5)))
            for i in range(i_lo, i_hi + 1):
                hm.set(i, j, 1.0)
    return hm


# ---------------------------------------------------------------------------
# Composition: blend, apply_mask, stamp
# ---------------------------------------------------------------------------


def _same_shape(a: Heightmap, b: Heightmap, op: str) -> None:
    if a.width != b.width or a.height != b.height:
        raise ValueError(
            f"{op}: shape mismatch {a.width}x{a.height} vs {b.width}x{b.height}"
        )


def blend(a: Heightmap, b: Heightmap, mask: Heightmap) -> Heightmap:
    """Per-cell ``a * (1 - m) + b * m`` where ``m = clamp(mask, 0, 1)``."""
    _same_shape(a, b, "blend")
    _same_shape(a, mask, "blend")
    out = Heightmap.zeros(a.width, a.height)
    for idx in range(len(a.data)):
        m = mask.data[idx]
        if m <= 0.0:
            out.data[idx] = a.data[idx]
        elif m >= 1.0:
            out.data[idx] = b.data[idx]
        else:
            out.data[idx] = a.data[idx] * (1.0 - m) + b.data[idx] * m
    return out


def apply_mask(
    h: Heightmap,
    mask: Heightmap,
    *,
    strength: float = 1.0,
    mode: str = "multiply",
) -> Heightmap:
    """Attenuate ``h`` by ``mask``. Modes:

    - ``multiply``: ``h_out = h * (1 - s) + h * mask * s``.
    - ``add``: ``h_out = h + mask * s``.
    - ``replace``: ``h_out = h * (1 - mask) + strength * mask``.
    """
    _same_shape(h, mask, "apply_mask")
    out = Heightmap.zeros(h.width, h.height)
    for idx in range(len(h.data)):
        v = h.data[idx]
        m = mask.data[idx]
        if mode == "multiply":
            out.data[idx] = v * (1.0 - strength) + (v * m) * strength
        elif mode == "add":
            out.data[idx] = v + m * strength
        elif mode == "replace":
            out.data[idx] = v * (1.0 - m) + strength * m
        else:
            raise ValueError(f"apply_mask: unknown mode {mode!r}")
    return out


def crater_pattern(
    radius: int,
    depth: float,
    *,
    rim_height: float | None = None,
    rim_width: float = 0.25,
) -> Heightmap:
    """Build a circular crater pattern centred in its grid.

    Output is a ``(2*radius+1) x (2*radius+1)`` heightmap suitable for
    ``stamp(..., mode="add")``:

    - Inside the crater bowl the value is negative (dig down), reaching
      ``-depth`` at the centre via a smooth bowl profile.
    - A raised rim of height ``rim_height`` (defaults to ``0.35 * depth``)
      circles the outer edge, with ``rim_width`` controlling how wide
      the rim band is (as a fraction of radius).
    - Outside the circle the value is 0 so ``add`` does not disturb the
      surrounding terrain.

    Matches the "bowl with a raised rim" profile every planetary impact
    crater has — cheap, deterministic, and composable with the other
    heightmap primitives.
    """
    if radius <= 0:
        raise ValueError("crater_pattern: radius must be positive")
    if depth <= 0.0:
        raise ValueError("crater_pattern: depth must be positive")
    rim = rim_height if rim_height is not None else depth * 0.35
    if rim_width <= 0.0 or rim_width >= 1.0:
        raise ValueError("crater_pattern: rim_width must be in (0, 1)")

    size = 2 * radius + 1
    data = [0.0] * (size * size)
    r2 = radius * radius
    rim_inner = 1.0 - rim_width  # normalised radial position where rim starts
    for j in range(size):
        for i in range(size):
            dx = i - radius
            dy = j - radius
            d2 = dx * dx + dy * dy
            if d2 > r2:
                continue
            # Normalised radial position: 0 at centre, 1 at outer edge.
            t = math.sqrt(d2) / radius
            if t <= rim_inner:
                # Bowl: smooth dip using a half-cosine so the bottom is
                # smooth rather than pointy. ``u`` maps [0, rim_inner]
                # to [0, 1], with 0 at the rim-edge and 1 at centre.
                u = 1.0 - (t / rim_inner)
                value = -depth * _smoothstep(u)
            else:
                # Rim: a bump peaking mid-rim-band.
                u = (t - rim_inner) / rim_width   # 0 at bowl edge, 1 at outer edge
                rim_bump = math.sin(math.pi * u)  # peaks at u=0.5
                value = rim * rim_bump
            data[j * size + i] = value
    return Heightmap(size, size, data)


def stamp(
    base: Heightmap,
    pattern: Heightmap,
    x: int,
    y: int,
    *,
    mode: str = "add",
) -> Heightmap:
    """Stamp ``pattern`` onto ``base`` at offset ``(x, y)``.

    The pattern is clipped against ``base`` bounds — no wrap, no error if
    the stamp is partially off-grid. Modes:

    - ``add`` — sum
    - ``sub`` — subtract
    - ``max`` — per-cell max (build a mountain)
    - ``min`` — per-cell min (carve a valley)
    - ``replace`` — overwrite
    """
    if mode not in {"add", "sub", "max", "min", "replace"}:
        raise ValueError(f"stamp: unknown mode {mode!r}")

    out = base.copy()
    for pj in range(pattern.height):
        bj = y + pj
        if bj < 0 or bj >= base.height:
            continue
        for pi in range(pattern.width):
            bi = x + pi
            if bi < 0 or bi >= base.width:
                continue
            pv = pattern.get(pi, pj)
            bv = out.get(bi, bj)
            if mode == "add":
                out.set(bi, bj, bv + pv)
            elif mode == "sub":
                out.set(bi, bj, bv - pv)
            elif mode == "max":
                out.set(bi, bj, max(bv, pv))
            elif mode == "min":
                out.set(bi, bj, min(bv, pv))
            else:
                out.set(bi, bj, pv)
    return out


# ---------------------------------------------------------------------------
# Erosion
# ---------------------------------------------------------------------------


def erode_thermal(
    h: Heightmap,
    *,
    steps: int = 20,
    talus: float = 0.01,
    rate: float = 0.5,
) -> Heightmap:
    """Thermal erosion by talus-angle redistribution.

    For each step: every cell that is higher than a neighbour by more
    than ``talus`` donates ``rate / 4`` of the excess to the lowest
    neighbour. Cheap (O(steps * width * height)) and good enough to
    break smooth fBm ridges into plausible faceted slopes.
    """
    if steps < 0:
        raise ValueError("erode_thermal: steps must be >= 0")
    current = h.copy()
    w, ht = current.width, current.height

    for _ in range(steps):
        deltas = [0.0] * (w * ht)
        for j in range(ht):
            for i in range(w):
                centre = current.get(i, j)
                # Find the steepest downward neighbour (4-connected).
                best_drop = 0.0
                best_ni = i
                best_nj = j
                for dj, di in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    ni, nj = i + di, j + dj
                    if ni < 0 or ni >= w or nj < 0 or nj >= ht:
                        continue
                    drop = centre - current.get(ni, nj)
                    if drop > best_drop:
                        best_drop = drop
                        best_ni, best_nj = ni, nj
                if best_drop > talus:
                    move = (best_drop - talus) * rate * 0.25
                    deltas[j * w + i] -= move
                    deltas[best_nj * w + best_ni] += move
        for idx in range(len(current.data)):
            current.data[idx] += deltas[idx]
    return current


def erode_hydraulic(
    h: Heightmap,
    seed: int,
    *,
    droplets: int = 1000,
    max_steps: int = 30,
    inertia: float = 0.05,
    capacity: float = 4.0,
    deposition: float = 0.1,
    erosion: float = 0.1,
    min_slope: float = 0.01,
    gravity: float = 4.0,
    evaporation: float = 0.01,
) -> Heightmap:
    """Particle-based hydraulic erosion. Budget target: < 500 ms on
    256x256 at ``droplets=1000, max_steps=30`` on the mid tier.

    Not a geology simulator — but enough to turn an fBm heightmap into
    something with dendritic drainage and visible cuts.

    Determinism: particle spawn positions derive from ``seed``; every
    float op below is a pure function of the starting state.
    """
    if droplets < 0:
        raise ValueError("erode_hydraulic: droplets must be >= 0")

    current = h.copy()
    w = current.width
    ht = current.height

    def height_at(x: float, y: float) -> tuple[float, float, float]:
        """Return (height, gradient_x, gradient_y) at fractional cell."""
        if x < 0.0:
            x = 0.0
        elif x > w - 1:
            x = float(w - 1)
        if y < 0.0:
            y = 0.0
        elif y > ht - 1:
            y = float(ht - 1)
        i0 = int(x)
        j0 = int(y)
        i1 = min(i0 + 1, w - 1)
        j1 = min(j0 + 1, ht - 1)
        tx = x - i0
        ty = y - j0
        h00 = current.get(i0, j0)
        h10 = current.get(i1, j0)
        h01 = current.get(i0, j1)
        h11 = current.get(i1, j1)
        height_val = (
            h00 * (1 - tx) * (1 - ty)
            + h10 * tx * (1 - ty)
            + h01 * (1 - tx) * ty
            + h11 * tx * ty
        )
        gx = (h10 - h00) * (1 - ty) + (h11 - h01) * ty
        gy = (h01 - h00) * (1 - tx) + (h11 - h10) * tx
        return height_val, gx, gy

    def deposit(x: float, y: float, amount: float) -> None:
        i0 = int(x)
        j0 = int(y)
        i1 = min(i0 + 1, w - 1)
        j1 = min(j0 + 1, ht - 1)
        i0 = max(0, min(i0, w - 1))
        j0 = max(0, min(j0, ht - 1))
        tx = x - i0
        ty = y - j0
        tx = max(0.0, min(1.0, tx))
        ty = max(0.0, min(1.0, ty))
        current.set(i0, j0, current.get(i0, j0) + amount * (1 - tx) * (1 - ty))
        current.set(i1, j0, current.get(i1, j0) + amount * tx * (1 - ty))
        current.set(i0, j1, current.get(i0, j1) + amount * (1 - tx) * ty)
        current.set(i1, j1, current.get(i1, j1) + amount * tx * ty)

    def erode_at(x: float, y: float, amount: float) -> None:
        # Symmetric to deposit but subtracts.
        deposit(x, y, -amount)

    for d in range(droplets):
        # Deterministic spawn via the module-level hash.
        x = _hash_value(seed, d, 0, salt=1) * (w - 1)
        y = _hash_value(seed, d, 0, salt=2) * (ht - 1)
        dir_x = 0.0
        dir_y = 0.0
        speed = 1.0
        water = 1.0
        sediment = 0.0
        for _ in range(max_steps):
            ix = int(x)
            iy = int(y)
            if ix < 0 or ix >= w - 1 or iy < 0 or iy >= ht - 1:
                break
            height_old, gx, gy = height_at(x, y)
            dir_x = dir_x * inertia - gx * (1 - inertia)
            dir_y = dir_y * inertia - gy * (1 - inertia)
            mag = math.sqrt(dir_x * dir_x + dir_y * dir_y)
            if mag == 0.0:
                break
            dir_x /= mag
            dir_y /= mag
            x_new = x + dir_x
            y_new = y + dir_y
            if (
                x_new < 0
                or x_new >= w - 1
                or y_new < 0
                or y_new >= ht - 1
            ):
                break
            height_new, _, _ = height_at(x_new, y_new)
            delta = height_new - height_old
            cap = max(-delta, min_slope) * speed * water * capacity
            if sediment > cap or delta > 0:
                drop_amount = (
                    (sediment - cap) * deposition
                    if sediment > cap
                    else min(delta, sediment)
                )
                if drop_amount > 0.0:
                    deposit(x, y, drop_amount)
                    sediment -= drop_amount
            else:
                take = min((cap - sediment) * erosion, -delta)
                if take > 0.0:
                    erode_at(x, y, take)
                    sediment += take
            speed = math.sqrt(max(0.0, speed * speed + delta * -gravity))
            water *= 1.0 - evaporation
            x = x_new
            y = y_new
    return current


__all__ = [
    "Heightmap",
    "fbm",
    "ridged",
    "worley",
    "mask_radial",
    "mask_rect",
    "mask_polygon",
    "blend",
    "apply_mask",
    "stamp",
    "crater_pattern",
    "erode_thermal",
    "erode_hydraulic",
]
