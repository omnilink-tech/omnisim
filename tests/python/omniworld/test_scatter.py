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

"""T1.3 scatter primitive tests.

Milestones covered:

- M1.3.a: ``poisson_polygon`` produces valid, non-overlapping placements
  on narrow / concave polygons.
- M1.3.b: ``clustered`` produces visibly clumped distributions — the
  intra-cluster spread is much tighter than the inter-cluster spread.
- M1.3.c: catalog-aware collision rejection is covered by the
  ``existing`` pathway (catalog lookup itself lands with T1.5).

Plus determinism under a fixed RNG seed, because the whole library
depends on it.
"""

from __future__ import annotations

import math
import random

import pytest

from omniworld.primitives.heightmap import Heightmap, fbm
from omniworld.primitives.scatter import (
    along_path,
    clustered,
    grid_jittered,
    on_surface,
    pick,
    pick_weighted,
    poisson_annulus,
    poisson_polygon,
)


def _min_pairwise_dist(points):
    best = math.inf
    for i, p in enumerate(points):
        for q in points[i + 1 :]:
            dx = p[0] - q[0]
            dy = p[1] - q[1]
            d = math.sqrt(dx * dx + dy * dy)
            if d < best:
                best = d
    return best


# --- poisson_annulus -----------------------------------------------------


def test_poisson_annulus_respects_min_dist():
    rng = random.Random(42)
    pts = poisson_annulus(rng, count=60, r_min=1.0, r_max=5.0, min_dist=0.5)
    assert len(pts) > 0
    assert _min_pairwise_dist(pts) >= 0.5


def test_poisson_annulus_respects_radii():
    rng = random.Random(7)
    pts = poisson_annulus(rng, count=40, r_min=2.0, r_max=4.0, min_dist=0.3)
    for x, y in pts:
        r = math.sqrt(x * x + y * y)
        assert 2.0 - 1e-9 <= r <= 4.0 + 1e-9


def test_poisson_annulus_deterministic():
    a = poisson_annulus(random.Random(1), count=30, r_min=1.0, r_max=4.0, min_dist=0.5)
    b = poisson_annulus(random.Random(1), count=30, r_min=1.0, r_max=4.0, min_dist=0.5)
    assert a == b


def test_poisson_annulus_existing_repels():
    rng = random.Random(0)
    existing = [(0.0, 3.0), (0.0, -3.0)]
    pts = poisson_annulus(
        rng, count=50, r_min=0.0, r_max=5.0, min_dist=1.5, existing=existing
    )
    for p in pts:
        for e in existing:
            dx = p[0] - e[0]
            dy = p[1] - e[1]
            assert math.sqrt(dx * dx + dy * dy) >= 1.5 - 1e-9


# --- poisson_polygon -----------------------------------------------------


_L_POLY = [  # L-shape (concave).
    (0.0, 0.0), (4.0, 0.0), (4.0, 2.0),
    (2.0, 2.0), (2.0, 4.0), (0.0, 4.0),
]


def test_poisson_polygon_points_inside_concave():
    rng = random.Random(11)
    pts = poisson_polygon(rng, count=80, polygon=_L_POLY, min_dist=0.3)
    # None of the points land in the concavity (x >= 2, y >= 2).
    for x, y in pts:
        assert not (x >= 2.0 and y >= 2.0), f"point {(x, y)} in the concave cut"


def test_poisson_polygon_respects_min_dist():
    rng = random.Random(99)
    pts = poisson_polygon(rng, count=60, polygon=_L_POLY, min_dist=0.4)
    assert _min_pairwise_dist(pts) >= 0.4


def test_poisson_polygon_narrow_strip_degrades_gracefully():
    """A 10x0.1 strip cannot fit 500 points at spacing 0.5 — the function
    must return fewer, not loop forever."""
    narrow = [(0.0, 0.0), (10.0, 0.0), (10.0, 0.1), (0.0, 0.1)]
    rng = random.Random(4)
    pts = poisson_polygon(rng, count=500, polygon=narrow, min_dist=0.5)
    # We should get some points but well below 500.
    assert 0 < len(pts) < 500


def test_poisson_polygon_rejects_degenerate_polygon():
    with pytest.raises(ValueError):
        poisson_polygon(random.Random(0), count=1, polygon=[(0.0, 0.0), (1.0, 1.0)], min_dist=1.0)


# --- grid_jittered -------------------------------------------------------


def test_grid_jittered_all_inside():
    rng = random.Random(1)
    poly = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    pts = grid_jittered(rng, poly, cell_size=2.0, jitter=0.1)
    # 5x5 grid expected.
    assert len(pts) == 25
    for x, y in pts:
        assert 0.0 < x < 10.0 and 0.0 < y < 10.0


def test_grid_jittered_rejects_bad_jitter():
    poly = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)]
    with pytest.raises(ValueError):
        grid_jittered(random.Random(0), poly, cell_size=1.0, jitter=0.6)


# --- clustered -----------------------------------------------------------


def test_clustered_is_clumpier_than_uniform():
    """Intra-cluster mean distance should be much smaller than
    inter-cluster mean distance. The spec says M1.3.b is about a
    'clumpiness metric matches target'; here we use a simple proxy."""
    rng = random.Random(0)
    centres = [(-10.0, -10.0), (10.0, 10.0)]
    pts = clustered(rng, centres, count_per=30, sigma=0.5, min_dist=0.0)
    assert len(pts) == 60
    # Partition by nearest centre.
    cluster_a = [p for p in pts if _dist2_to(centres[0], p) < _dist2_to(centres[1], p)]
    cluster_b = [p for p in pts if p not in cluster_a]
    intra = sum(_mean_pairwise(cluster_a) + _mean_pairwise(cluster_b)) / 2.0
    # distance between centres is ~28.28, most pairs should cross that.
    inter = 0.0
    for pa in cluster_a:
        for pb in cluster_b:
            inter += math.sqrt(_dist2_to(pa, pb))
    inter /= max(1, len(cluster_a) * len(cluster_b))
    assert intra < inter / 5.0, f"intra={intra}, inter={inter}"


def _dist2_to(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def _mean_pairwise(pts):
    if len(pts) < 2:
        return [0.0]
    s = 0.0
    n = 0
    for i, p in enumerate(pts):
        for q in pts[i + 1 :]:
            s += math.sqrt(_dist2_to(p, q))
            n += 1
    return [s / n]


def test_clustered_deterministic():
    centres = [(0.0, 0.0), (5.0, 5.0)]
    a = clustered(random.Random(3), centres, count_per=10, sigma=0.3)
    b = clustered(random.Random(3), centres, count_per=10, sigma=0.3)
    assert a == b


# --- along_path ----------------------------------------------------------


def test_along_path_density():
    # Straight 10 m segment, density 1 pt/m, no jitter: expect ~10 points.
    path = [(0.0, 0.0), (10.0, 0.0)]
    pts = along_path(random.Random(0), path, density=1.0)
    assert 9 <= len(pts) <= 11


def test_along_path_offset_sides():
    path = [(0.0, 0.0), (10.0, 0.0)]
    left = along_path(random.Random(0), path, density=1.0, offset=1.0, side="left")
    right = along_path(random.Random(0), path, density=1.0, offset=1.0, side="right")
    # Left of +x direction is +y.
    assert all(p[1] > 0.0 for p in left)
    assert all(p[1] < 0.0 for p in right)


def test_along_path_both_sides_alternate():
    path = [(0.0, 0.0), (20.0, 0.0)]
    pts = along_path(random.Random(0), path, density=0.5, offset=1.0, side="both")
    signs = [1 if p[1] > 0 else -1 for p in pts]
    # Expect alternating signs.
    assert signs[0] != signs[1]


# --- on_surface ----------------------------------------------------------


def test_on_surface_projects_z():
    # Simple ramp: z = x/10 over a 10x10 world, grid 11x11.
    w = 11
    data = []
    for j in range(w):
        for i in range(w):
            data.append(i / (w - 1))
    hm = Heightmap(w, w, data)
    pts = on_surface(
        [(0.0, 0.0), (10.0, 0.0), (5.0, 5.0)],
        hm,
        grid_origin=(0.0, 0.0),
        x_spacing=1.0,
        y_spacing=1.0,
        height_scale=2.0,
    )
    assert pts[0][2] == pytest.approx(0.0)
    assert pts[1][2] == pytest.approx(2.0)
    assert pts[2][2] == pytest.approx(1.0)


def test_on_surface_max_slope_filters():
    # Build a heightmap with a sharp cliff in the middle column.
    w = 11
    data = []
    for j in range(w):
        for i in range(w):
            data.append(0.0 if i < 5 else 1.0)
    hm = Heightmap(w, w, data)
    pts = on_surface(
        [(1.0, 5.0), (5.0, 5.0), (9.0, 5.0)],
        hm,
        x_spacing=1.0,
        y_spacing=1.0,
        height_scale=1.0,
        max_slope=0.4,
    )
    # The midpoint sits on the cliff; the flat-shelf points survive.
    assert (5.0, 5.0) not in [(p[0], p[1]) for p in pts]
    assert any(abs(p[0] - 1.0) < 1e-9 for p in pts)
    assert any(abs(p[0] - 9.0) < 1e-9 for p in pts)


# --- pick / pick_weighted ------------------------------------------------


def test_pick_uniform_covers_every_index():
    rng = random.Random(0)
    items = ["a", "b", "c"]
    seen = {pick(rng, items) for _ in range(200)}
    assert seen == set(items)


def test_pick_weighted_respects_weights():
    rng = random.Random(1)
    counts = {0: 0, 1: 0}
    for _ in range(10_000):
        counts[pick_weighted(rng, [3.0, 1.0])] += 1
    # Expect roughly 3:1.
    ratio = counts[0] / counts[1]
    assert 2.3 < ratio < 3.7


def test_pick_weighted_rejects_bad_weights():
    with pytest.raises(ValueError):
        pick_weighted(random.Random(0), [-1.0, 1.0])
    with pytest.raises(ValueError):
        pick_weighted(random.Random(0), [0.0, 0.0])


# --- cross-primitive: outdoor-recipe-shaped pipeline --------------------


def test_scatter_then_surface_is_runnable():
    """A tiny end-to-end: heightmap -> poisson_annulus -> on_surface.
    Precursor to the outdoor_forest recipe (T1.6)."""
    rng = random.Random(5)
    hm = fbm(32, 32, seed=5, octaves=3)
    pts2d = poisson_annulus(rng, count=15, r_min=2.0, r_max=10.0, min_dist=1.0)
    pts3d = on_surface(
        pts2d, hm,
        grid_origin=(-16.0, -16.0),
        x_spacing=1.0, y_spacing=1.0,
        height_scale=3.0,
    )
    assert len(pts3d) == len(pts2d)
    for p in pts3d:
        assert 0.0 <= p[2] <= 3.0 + 1e-9
