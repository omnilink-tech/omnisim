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

"""The wire router's compiled search and windowed distance update give the reference
answers (desk_robot/wiring.py; engine-free)."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "projects/manufacturing/products/desk_robot"))

import wiring  # noqa: E402

pytestmark = pytest.mark.skipif(not wiring.JIT, reason="numba not installed: only the reference search runs")


def _space(seed=3, n=36, pitch=1.0):
    from scipy import ndimage
    rng = np.random.default_rng(seed)
    occ = np.zeros((n, n, n), bool)
    for _ in range(14):                       # boxes to route round
        lo = rng.integers(2, n - 10, 3)
        hi = lo + rng.integers(3, 10, 3)
        occ[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] = True
    occ[:, :, 0] = True
    sp = wiring.Space.__new__(wiring.Space)
    sp.lo, sp.pitch, sp.occ, sp.shape = np.zeros(3), pitch, occ, occ.shape
    sp.dist = ndimage.distance_transform_edt(~occ) * pitch
    sp.dist0 = sp.dist.copy()
    return sp


def _ends(sp, k, seed=7):
    rng = np.random.default_rng(seed)
    cells = np.argwhere(sp.dist > 2.0)
    pick = rng.choice(len(cells), 2 * k, replace=False)
    return [(tuple(map(int, cells[pick[2 * i]])), tuple(map(int, cells[pick[2 * i + 1]]))) for i in range(k)]


def test_compiled_search_is_the_reference_search():
    sp = _space()
    free = sp.dist > 1.1
    for a, b in _ends(sp, 12):
        try:
            ref = wiring._astar_py(sp, a, b, free, 2.6)
        except RuntimeError:
            with pytest.raises(RuntimeError):
                wiring._astar(sp, a, b, free, 2.6)
            continue
        assert wiring._astar(sp, a, b, free, 2.6) == ref


def test_compiled_line_test_is_the_reference_line_test():
    sp = _space(seed=5)
    free = sp.dist > 1.1
    wiring.JIT = False
    try:
        rng = np.random.default_rng(13)
        cases = _ends(sp, 40, seed=11)          # long lines: most leave the surfaces
        cases += [(a, tuple(int(np.clip(v + d, 0, sp.shape[0] - 1)) for v, d in zip(a, rng.integers(-3, 4, 3))))
                  for a, _ in cases]           # short ones (FREE_SPAN_MM): most are free
        ref = [wiring._line_free(free, a, b, sp.dist, 2.6, sp.pitch) for a, b in cases]
    finally:
        wiring.JIT = True
    assert [wiring._line_free(free, a, b, sp.dist, 2.6, sp.pitch) for a, b in cases] == ref
    assert any(ref) and not all(ref)


def test_windowed_distance_update_matches_a_full_transform_below_the_window():
    from scipy import ndimage
    sp = _space(seed=9, n=60)
    a, b = _ends(sp, 1, seed=2)[0]
    pts = [sp.point(c) for c in wiring._astar(sp, a, b, sp.dist > 1.1, 2.6)]
    wiring.occupy(sp, pts, 1.6)
    full = ndimage.distance_transform_edt(~sp.occ) * sp.pitch
    near = full <= wiring.EDT_WINDOW_MM
    assert np.array_equal(sp.dist[near], full[near])
    assert (sp.dist >= full).all()             # beyond the window a distance can only be too large
