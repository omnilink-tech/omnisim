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

"""Scatter primitives.

T1.3 deliverable. Every biome — forests, rock fields, orchards,
warehouses, urban blocks — composes out of these. They are 2D-first
(x, y) because almost every scatter decision is a 2D one; the vertical
coordinate is projected from a surface in ``on_surface``.

Contents:

- ``poisson_annulus`` — scatter inside a ring centred at ``(0, 0)``.
  The shape the existing Omnibot demo produces.
- ``poisson_polygon`` — scatter inside an arbitrary 2D polygon. Uses a
  spatial-hash-backed rejection sampler so asymptotic cost is near
  ``O(count)`` even for stubby / concave inputs.
- ``grid_jittered`` — axis-aligned grid inside a polygon, each cell
  perturbed by ``jitter`` of its cell size. Right primitive for
  orchards, pallet racks, street furniture.
- ``clustered`` — Gaussian spread around pre-supplied centres. Right
  primitive for tree stands, rock piles, vehicle parking clusters.
- ``along_path`` — points placed along a polyline at a target linear
  density, optionally offset sideways. Roadside trees, warehouse
  aisles, street lights.
- ``on_surface`` — lifts a 2D point list to 3D by sampling a
  ``Heightmap``. Optional slope culling.
- ``pick`` / ``pick_weighted`` — weighted choice helpers.

Determinism contract:

- All generators take an explicit ``rng: random.Random``. No global RNG.
- Rejection order is fixed (inserted in loop iteration order), so the
  same inputs produce the same output on every platform / Python
  version that agrees on ``random.Random.random`` (all CPython builds).
"""

from __future__ import annotations

import math
import random
from typing import Iterable, Sequence

from .heightmap import Heightmap


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------


Point2 = tuple[float, float]
Point3 = tuple[float, float, float]


def _dist2(a: Point2, b: Point2) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def _polygon_bbox(polygon: Sequence[Point2]) -> tuple[float, float, float, float]:
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return min(xs), min(ys), max(xs), max(ys)


def _point_in_polygon(p: Point2, polygon: Sequence[Point2]) -> bool:
    """Even-odd rule (same as mask_polygon)."""
    x, y = p
    inside = False
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        if (ay > y) != (by > y):
            t = (y - ay) / (by - ay)
            if x < ax + t * (bx - ax):
                inside = not inside
    return inside


class _SpatialHash:
    """Uniform-cell spatial hash for near-neighbour lookup.

    Cell size = ``min_dist``. A rejection check needs to inspect only
    the 3x3 block of cells around the candidate, independent of total
    point count. Critical for making ``poisson_polygon`` practical
    above a few hundred points.
    """

    __slots__ = ("cell_size", "grid")

    def __init__(self, cell_size: float) -> None:
        if cell_size <= 0.0:
            raise ValueError("spatial hash cell_size must be positive")
        self.cell_size = cell_size
        self.grid: dict[tuple[int, int], list[Point2]] = {}

    def _key(self, p: Point2) -> tuple[int, int]:
        return (int(math.floor(p[0] / self.cell_size)),
                int(math.floor(p[1] / self.cell_size)))

    def insert(self, p: Point2) -> None:
        k = self._key(p)
        self.grid.setdefault(k, []).append(p)

    def any_within(self, p: Point2, dist: float) -> bool:
        d2 = dist * dist
        cx, cy = self._key(p)
        # Scan the 3x3 neighbourhood. ``dist <= cell_size`` guarantees
        # any closer neighbour lives in this block.
        for ki in (cx - 1, cx, cx + 1):
            for kj in (cy - 1, cy, cy + 1):
                for other in self.grid.get((ki, kj), ()):
                    if _dist2(p, other) < d2:
                        return True
        return False


# ---------------------------------------------------------------------------
# poisson_annulus — the scatter the existing demo uses
# ---------------------------------------------------------------------------


def poisson_annulus(
    rng: random.Random,
    count: int,
    *,
    r_min: float,
    r_max: float,
    min_dist: float,
    centre: Point2 = (0.0, 0.0),
    existing: Iterable[Point2] | None = None,
    max_attempts_per_point: int = 30,
) -> list[Point2]:
    """Scatter up to ``count`` points in an annulus.

    Rejection sampling: pick a candidate uniformly in the annulus; accept
    if it is at least ``min_dist`` from every previously accepted point
    and every element of ``existing``. Stop at ``count`` successes or
    when ``count * max_attempts_per_point`` candidates have been tried.

    Returns fewer than ``count`` points if the annulus cannot fit
    ``count`` at the given spacing — the caller decides what to do.
    """
    if count < 0:
        raise ValueError("poisson_annulus: count must be >= 0")
    if r_min < 0.0 or r_max < r_min:
        raise ValueError("poisson_annulus: need 0 <= r_min <= r_max")
    if min_dist < 0.0:
        raise ValueError("poisson_annulus: min_dist must be >= 0")

    accepted: list[Point2] = []
    hash_bucket = _SpatialHash(max(min_dist, 1e-6))
    for ex in existing or ():
        hash_bucket.insert(ex)

    attempts_budget = count * max_attempts_per_point
    r2_min = r_min * r_min
    r2_max = r_max * r_max

    while len(accepted) < count and attempts_budget > 0:
        attempts_budget -= 1
        # Uniform-in-area disk sample, then gated by r_min.
        t = rng.random()
        r = math.sqrt(r2_min + t * (r2_max - r2_min))
        theta = rng.random() * 2.0 * math.pi
        p = (centre[0] + r * math.cos(theta), centre[1] + r * math.sin(theta))
        if min_dist > 0.0 and hash_bucket.any_within(p, min_dist):
            continue
        accepted.append(p)
        hash_bucket.insert(p)
    return accepted


# ---------------------------------------------------------------------------
# poisson_polygon
# ---------------------------------------------------------------------------


def poisson_polygon(
    rng: random.Random,
    count: int,
    polygon: Sequence[Point2],
    *,
    min_dist: float,
    existing: Iterable[Point2] | None = None,
    max_attempts_per_point: int = 30,
) -> list[Point2]:
    """Rejection-sample up to ``count`` points inside a 2D polygon.

    Polygon uses the even-odd rule and may be convex or concave.
    Returns fewer than ``count`` if the polygon cannot accommodate them.
    """
    if count < 0:
        raise ValueError("poisson_polygon: count must be >= 0")
    if min_dist < 0.0:
        raise ValueError("poisson_polygon: min_dist must be >= 0")
    if len(polygon) < 3:
        raise ValueError("poisson_polygon: polygon must have >= 3 vertices")

    x0, y0, x1, y1 = _polygon_bbox(polygon)
    if x1 <= x0 or y1 <= y0:
        return []

    hash_bucket = _SpatialHash(max(min_dist, 1e-6))
    for ex in existing or ():
        hash_bucket.insert(ex)

    accepted: list[Point2] = []
    attempts_budget = count * max_attempts_per_point

    while len(accepted) < count and attempts_budget > 0:
        attempts_budget -= 1
        p = (x0 + rng.random() * (x1 - x0), y0 + rng.random() * (y1 - y0))
        if not _point_in_polygon(p, polygon):
            continue
        if min_dist > 0.0 and hash_bucket.any_within(p, min_dist):
            continue
        accepted.append(p)
        hash_bucket.insert(p)

    return accepted


# ---------------------------------------------------------------------------
# grid_jittered
# ---------------------------------------------------------------------------


def grid_jittered(
    rng: random.Random,
    polygon: Sequence[Point2],
    *,
    cell_size: float,
    jitter: float = 0.25,
) -> list[Point2]:
    """Axis-aligned grid of points covering ``polygon``.

    Each cell contributes one point at its centre plus a uniform
    perturbation of ``±jitter * cell_size`` on each axis. Points outside
    the polygon are dropped.

    Intended for orchard-like (orderly with breathing room) and
    aisle-like (pallet racks, parking) layouts.
    """
    if cell_size <= 0.0:
        raise ValueError("grid_jittered: cell_size must be positive")
    if jitter < 0.0 or jitter > 0.5:
        raise ValueError("grid_jittered: jitter must be in [0, 0.5]")
    if len(polygon) < 3:
        raise ValueError("grid_jittered: polygon must have >= 3 vertices")

    x0, y0, x1, y1 = _polygon_bbox(polygon)
    result: list[Point2] = []
    jmax = jitter * cell_size

    y = y0 + 0.5 * cell_size
    while y <= y1 - 0.5 * cell_size + 1e-9:
        x = x0 + 0.5 * cell_size
        while x <= x1 - 0.5 * cell_size + 1e-9:
            px = x + (rng.random() * 2.0 - 1.0) * jmax
            py = y + (rng.random() * 2.0 - 1.0) * jmax
            if _point_in_polygon((px, py), polygon):
                result.append((px, py))
            x += cell_size
        y += cell_size
    return result


# ---------------------------------------------------------------------------
# clustered
# ---------------------------------------------------------------------------


def clustered(
    rng: random.Random,
    centres: Sequence[Point2],
    *,
    count_per: int,
    sigma: float,
    min_dist: float = 0.0,
    existing: Iterable[Point2] | None = None,
    max_attempts_per_point: int = 30,
) -> list[Point2]:
    """Scatter ``count_per`` points around each supplied centre.

    Each point is drawn from an isotropic Gaussian of standard deviation
    ``sigma`` around its centre. Rejection filter at ``min_dist``
    against all previously accepted points and ``existing``.

    Right primitive for tree stands, rock piles, vehicle clusters — the
    "visibly clumped distribution" the plan's M1.3.b asks for.
    """
    if count_per < 0:
        raise ValueError("clustered: count_per must be >= 0")
    if sigma <= 0.0:
        raise ValueError("clustered: sigma must be positive")
    if min_dist < 0.0:
        raise ValueError("clustered: min_dist must be >= 0")

    accepted: list[Point2] = []
    hash_bucket = _SpatialHash(max(min_dist, 1e-6))
    for ex in existing or ():
        hash_bucket.insert(ex)

    for cx, cy in centres:
        attempts = count_per * max_attempts_per_point
        placed = 0
        while placed < count_per and attempts > 0:
            attempts -= 1
            p = (rng.gauss(cx, sigma), rng.gauss(cy, sigma))
            if min_dist > 0.0 and hash_bucket.any_within(p, min_dist):
                continue
            accepted.append(p)
            hash_bucket.insert(p)
            placed += 1
    return accepted


# ---------------------------------------------------------------------------
# along_path
# ---------------------------------------------------------------------------


def _polyline_length(path: Sequence[Point2]) -> float:
    total = 0.0
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        total += math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
    return total


def _point_at_arc(path: Sequence[Point2], s: float) -> tuple[Point2, Point2]:
    """Return (position, forward_unit_tangent) at arc length ``s``."""
    if len(path) < 2:
        raise ValueError("path must have >= 2 vertices")
    remaining = s
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        seg_len = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
        if seg_len == 0.0:
            continue
        if remaining <= seg_len:
            t = remaining / seg_len
            x = ax + t * (bx - ax)
            y = ay + t * (by - ay)
            return (x, y), ((bx - ax) / seg_len, (by - ay) / seg_len)
        remaining -= seg_len
    # ``s`` past the end — clamp to the final vertex.
    ax, ay = path[-2]
    bx, by = path[-1]
    seg_len = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
    if seg_len == 0.0:
        return path[-1], (1.0, 0.0)
    return path[-1], ((bx - ax) / seg_len, (by - ay) / seg_len)


def along_path(
    rng: random.Random,
    path: Sequence[Point2],
    *,
    density: float,
    offset: float = 0.0,
    offset_jitter: float = 0.0,
    along_jitter: float = 0.0,
    side: str = "both",
) -> list[Point2]:
    """Place points along a polyline.

    - ``density`` points per metre of arc length.
    - ``offset`` — perpendicular distance from the path. 0 = on the path.
    - ``offset_jitter`` — uniform jitter added to ``offset`` per point.
    - ``along_jitter`` — uniform jitter on arc-length position (in metres).
    - ``side`` — ``"left"``, ``"right"``, ``"both"``. ``both`` alternates,
      which is what a roadside plantation looks like.
    """
    if density <= 0.0:
        raise ValueError("along_path: density must be positive")
    if side not in {"left", "right", "both"}:
        raise ValueError(f"along_path: unknown side {side!r}")

    length = _polyline_length(path)
    if length == 0.0:
        return []

    spacing = 1.0 / density
    n = int(length / spacing)
    # Include the half-spacing end so both ends get a point when density
    # divides length cleanly.
    result: list[Point2] = []
    for i in range(n + 1):
        s = (i + 0.5) * spacing + (rng.random() * 2.0 - 1.0) * along_jitter
        if s < 0.0 or s > length:
            continue
        (px, py), (tx, ty) = _point_at_arc(path, s)
        # Perpendicular (left of forward).
        nx, ny = -ty, tx
        if side == "left":
            sign = 1.0
        elif side == "right":
            sign = -1.0
        else:
            sign = 1.0 if (i % 2 == 0) else -1.0
        o = offset + rng.random() * offset_jitter
        result.append((px + sign * o * nx, py + sign * o * ny))
    return result


# ---------------------------------------------------------------------------
# on_surface
# ---------------------------------------------------------------------------


def on_surface(
    points: Iterable[Point2],
    hm: Heightmap,
    *,
    grid_origin: Point2 = (0.0, 0.0),
    x_spacing: float = 1.0,
    y_spacing: float = 1.0,
    height_scale: float = 1.0,
    max_slope: float | None = None,
) -> list[Point3]:
    """Lift 2D world points to 3D by sampling ``hm``.

    ``(grid_origin, x_spacing, y_spacing)`` map world x,y to heightmap
    grid coordinates: ``i = (x - origin_x) / x_spacing``. Points outside
    the grid are clamped by the heightmap's own clamping logic.

    ``max_slope`` (optional): reject points whose local slope magnitude
    exceeds the threshold. Useful for "don't put a building on a
    cliff". Slope is estimated from a central finite difference on the
    grid with world-space spacing.
    """
    if x_spacing <= 0.0 or y_spacing <= 0.0:
        raise ValueError("on_surface: spacings must be positive")

    ox, oy = grid_origin
    result: list[Point3] = []
    for x, y in points:
        gx = (x - ox) / x_spacing
        gy = (y - oy) / y_spacing
        z = hm.sample(gx, gy) * height_scale
        if max_slope is not None:
            eps = 0.5
            zx_p = hm.sample(gx + eps, gy)
            zx_m = hm.sample(gx - eps, gy)
            zy_p = hm.sample(gx, gy + eps)
            zy_m = hm.sample(gx, gy - eps)
            slope_x = (zx_p - zx_m) * height_scale / (2.0 * eps * x_spacing)
            slope_y = (zy_p - zy_m) * height_scale / (2.0 * eps * y_spacing)
            if math.sqrt(slope_x * slope_x + slope_y * slope_y) > max_slope:
                continue
        result.append((x, y, z))
    return result


# ---------------------------------------------------------------------------
# Weighted pick helpers
# ---------------------------------------------------------------------------


def pick_weighted(rng: random.Random, weights: Sequence[float]) -> int:
    """Return an index into ``weights`` with probability proportional to
    the weight. All weights must be non-negative; the total must be > 0.
    """
    total = 0.0
    for w in weights:
        if w < 0.0:
            raise ValueError("pick_weighted: weights must be non-negative")
        total += w
    if total <= 0.0:
        raise ValueError("pick_weighted: weights sum to zero")
    r = rng.random() * total
    cumulative = 0.0
    for i, w in enumerate(weights):
        cumulative += w
        if r < cumulative:
            return i
    return len(weights) - 1  # float slop fallback


def pick(rng: random.Random, items: Sequence, weights: Sequence[float] | None = None):
    """Weighted choice of one element. ``weights=None`` is uniform."""
    if not items:
        raise ValueError("pick: items must be non-empty")
    if weights is None:
        return items[rng.randrange(len(items))]
    if len(weights) != len(items):
        raise ValueError("pick: weights must match items in length")
    return items[pick_weighted(rng, weights)]


__all__ = [
    "Point2",
    "Point3",
    "poisson_annulus",
    "poisson_polygon",
    "grid_jittered",
    "clustered",
    "along_path",
    "on_surface",
    "pick",
    "pick_weighted",
]
