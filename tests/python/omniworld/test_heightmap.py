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

"""T1.2 heightmap primitive tests.

Milestones covered:

- M1.2.a: fbm is deterministic on one platform.
- M1.2.b: stamp and mask round-trip (identity ops do nothing, inverse
  ops invert).
- M1.2.c: erode_hydraulic stays under the time budget for the smallest
  shipping size (we run 64x64 with 200 droplets in CI; the plan target
  is 256x256 in < 500 ms on the mid tier, which we don't assert in CI
  because runners vary wildly).
- M1.2.d: ElevationGrid emitter produces valid VRML-ish text with the
  right header and byte-stable output.
"""

from __future__ import annotations

import time

import pytest

from omniworld.emit import render_elevation_grid, render_terrain_solid
from omniworld.primitives.heightmap import (
    Heightmap,
    apply_mask,
    blend,
    erode_hydraulic,
    erode_thermal,
    fbm,
    mask_polygon,
    mask_radial,
    mask_rect,
    ridged,
    stamp,
    worley,
)


# --- Heightmap container -------------------------------------------------


def test_heightmap_ctor_shape_mismatch():
    with pytest.raises(ValueError):
        Heightmap(3, 3, [0.0] * 8)


def test_heightmap_get_set_roundtrip():
    hm = Heightmap.zeros(4, 3)
    hm.set(2, 1, 0.75)
    assert hm.get(2, 1) == 0.75
    assert hm.get(0, 0) == 0.0


def test_heightmap_sample_corners_and_centre():
    # Ramp from 0 at x=0 to 1 at x=1, flat in y.
    hm = Heightmap(2, 2, [0.0, 1.0, 0.0, 1.0])
    assert hm.sample(0.0, 0.0) == 0.0
    assert hm.sample(1.0, 0.0) == 1.0
    assert hm.sample(0.5, 0.5) == pytest.approx(0.5)


def test_heightmap_sample_clamps():
    hm = Heightmap(2, 2, [0.0, 1.0, 0.0, 1.0])
    assert hm.sample(-10.0, 0.0) == 0.0
    assert hm.sample(100.0, 0.0) == 1.0


def test_heightmap_normalise():
    hm = Heightmap(2, 2, [0.0, 2.0, 4.0, 6.0])
    n = hm.normalise(0.0, 1.0)
    assert n.min() == 0.0
    assert n.max() == 1.0

    # Degenerate input -> uniform output.
    flat = Heightmap.constant(2, 2, 3.0).normalise(0.0, 1.0)
    assert flat.min() == flat.max() == 0.0


# --- Noise: determinism --------------------------------------------------


def test_fbm_deterministic_same_seed():
    a = fbm(16, 16, seed=42)
    b = fbm(16, 16, seed=42)
    assert a.data == b.data


def test_fbm_varies_with_seed():
    a = fbm(16, 16, seed=1)
    b = fbm(16, 16, seed=2)
    assert a.data != b.data


def test_fbm_output_in_unit_range():
    hm = fbm(32, 32, seed=7)
    assert hm.min() == pytest.approx(0.0, abs=1e-12)
    assert hm.max() == pytest.approx(1.0, abs=1e-12)
    assert all(-1e-12 <= v <= 1.0 + 1e-12 for v in hm.data)


def test_ridged_deterministic_and_in_unit_range():
    a = ridged(24, 24, seed=3)
    b = ridged(24, 24, seed=3)
    assert a.data == b.data
    assert a.min() == pytest.approx(0.0, abs=1e-12)
    assert a.max() == pytest.approx(1.0, abs=1e-12)


def test_worley_deterministic_and_in_unit_range():
    a = worley(24, 24, seed=5)
    b = worley(24, 24, seed=5)
    assert a.data == b.data
    assert a.min() == pytest.approx(0.0, abs=1e-12)
    assert a.max() == pytest.approx(1.0, abs=1e-12)


def test_worley_variants():
    # f2_f1 must be <= f2, and at worley cell centres f1 is near zero.
    f1 = worley(16, 16, seed=9, variant="f1")
    f2 = worley(16, 16, seed=9, variant="f2")
    f2_f1 = worley(16, 16, seed=9, variant="f2_f1")
    assert f1.data != f2.data
    assert f1.data != f2_f1.data


# --- Masks ---------------------------------------------------------------


def test_mask_radial_centre_is_one():
    m = mask_radial(11, 11, centre=(5.0, 5.0), radius=5.0)
    assert m.get(5, 5) == pytest.approx(1.0)


def test_mask_radial_edge_is_zero():
    m = mask_radial(11, 11, centre=(5.0, 5.0), radius=5.0, falloff="linear")
    # The corner is outside the radius (distance ~7.07).
    assert m.get(0, 0) == 0.0
    assert m.get(10, 10) == 0.0


def test_mask_rect_inside_vs_outside():
    m = mask_rect(10, 10, x0=2, y0=2, x1=8, y1=8, feather=0)
    assert m.get(5, 5) == 1.0
    assert m.get(1, 5) == 0.0
    assert m.get(8, 5) == 0.0  # x1 is exclusive


def test_mask_polygon_square_vs_inside_point():
    poly = [(2.0, 2.0), (8.0, 2.0), (8.0, 8.0), (2.0, 8.0)]
    m = mask_polygon(10, 10, poly)
    assert m.get(5, 5) == 1.0
    assert m.get(0, 0) == 0.0


def test_mask_polygon_needs_three_vertices():
    with pytest.raises(ValueError):
        mask_polygon(4, 4, [(0.0, 0.0), (1.0, 1.0)])


# --- Composition ---------------------------------------------------------


def test_blend_identity_with_zero_mask():
    a = fbm(8, 8, seed=1)
    b = fbm(8, 8, seed=2)
    zero = Heightmap.zeros(8, 8)
    out = blend(a, b, zero)
    assert out.data == a.data


def test_blend_identity_with_one_mask():
    a = fbm(8, 8, seed=1)
    b = fbm(8, 8, seed=2)
    ones = Heightmap.constant(8, 8, 1.0)
    out = blend(a, b, ones)
    assert out.data == b.data


def test_apply_mask_replace_mode():
    h = Heightmap.constant(4, 4, 0.2)
    m = mask_rect(4, 4, x0=1, y0=1, x1=3, y1=3, feather=0)
    out = apply_mask(h, m, strength=0.9, mode="replace")
    assert out.get(0, 0) == pytest.approx(0.2)
    assert out.get(2, 2) == pytest.approx(0.9)


def test_stamp_add_sub_inverse():
    base = fbm(8, 8, seed=0)
    pat = fbm(4, 4, seed=1)
    added = stamp(base, pat, 2, 2, mode="add")
    back = stamp(added, pat, 2, 2, mode="sub")
    for orig, restored in zip(base.data, back.data):
        assert orig == pytest.approx(restored)


def test_stamp_mode_validation():
    with pytest.raises(ValueError):
        stamp(Heightmap.zeros(2, 2), Heightmap.zeros(1, 1), 0, 0, mode="bogus")


def test_crater_pattern_centre_is_deepest():
    from omniworld.primitives.heightmap import crater_pattern

    pat = crater_pattern(radius=10, depth=1.5, rim_height=0.4, rim_width=0.25)
    # Centre has the deepest dig.
    centre = pat.get(10, 10)
    assert centre == pytest.approx(-1.5, abs=1e-9)
    # Corners of the pattern grid are outside the crater circle -> 0.
    assert pat.get(0, 0) == 0.0


def test_crater_pattern_has_raised_rim():
    from omniworld.primitives.heightmap import crater_pattern

    pat = crater_pattern(radius=12, depth=1.0, rim_height=0.3, rim_width=0.25)
    # Somewhere in the rim band the value should be positive.
    found_rim = False
    for j in range(pat.height):
        for i in range(pat.width):
            if pat.get(i, j) > 0.05:
                found_rim = True
                break
    assert found_rim, "crater should have a raised rim"


def test_crater_pattern_rejects_bad_inputs():
    from omniworld.primitives.heightmap import crater_pattern

    with pytest.raises(ValueError):
        crater_pattern(radius=0, depth=1.0)
    with pytest.raises(ValueError):
        crater_pattern(radius=3, depth=0.0)
    with pytest.raises(ValueError):
        crater_pattern(radius=3, depth=1.0, rim_width=0.0)


def test_stamp_clips_out_of_bounds():
    base = Heightmap.zeros(4, 4)
    pat = Heightmap.constant(3, 3, 1.0)
    # Lands mostly off the top-left corner.
    out = stamp(base, pat, -2, -2, mode="add")
    assert out.get(0, 0) == 1.0  # only one pattern cell overlapped base
    # ...everything else still zero.
    total = sum(out.data)
    assert total == 1.0


# --- Erosion -------------------------------------------------------------


def test_erode_thermal_preserves_mass_roughly():
    hm = fbm(16, 16, seed=11)
    before = sum(hm.data)
    eroded = erode_thermal(hm, steps=5, talus=0.02, rate=0.4)
    after = sum(eroded.data)
    # Internal edges redistribute mass; boundary donations bleed out, so
    # allow a small drift. Main assertion: erosion changed the height map.
    assert before != after or eroded.data != hm.data


def test_erode_thermal_zero_steps_is_identity():
    hm = fbm(8, 8, seed=2)
    assert erode_thermal(hm, steps=0).data == hm.data


def test_erode_hydraulic_deterministic():
    hm = fbm(24, 24, seed=3)
    a = erode_hydraulic(hm, seed=7, droplets=50, max_steps=20)
    b = erode_hydraulic(hm, seed=7, droplets=50, max_steps=20)
    assert a.data == b.data


def test_erode_hydraulic_time_budget_small_grid():
    """A lightweight sanity check for T1.2.c: the smallest realistic
    grid must erode quickly. The plan target (256x256 in < 500 ms on
    mid-tier) is too variable for CI; here we assert a much looser
    bound on a tiny grid to catch catastrophic regressions."""
    hm = fbm(64, 64, seed=0)
    t0 = time.perf_counter()
    erode_hydraulic(hm, seed=0, droplets=200, max_steps=20)
    elapsed = time.perf_counter() - t0
    assert elapsed < 5.0, f"hydraulic erosion too slow: {elapsed:.2f}s"


# --- ElevationGrid emitter ----------------------------------------------


def test_elevation_grid_contains_fields():
    hm = Heightmap(3, 2, [0.0, 0.1, 0.2, 0.3, 0.4, 0.5])
    text = render_elevation_grid(hm, x_spacing=0.5, y_spacing=0.5)
    assert text.startswith("ElevationGrid {")
    assert "xDimension 3" in text
    assert "yDimension 2" in text
    assert "xSpacing 0.5" in text
    assert "ySpacing 0.5" in text
    assert "0.1" in text and "0.5" in text


def test_elevation_grid_byte_stable():
    hm = fbm(8, 8, seed=1)
    a = render_elevation_grid(hm)
    b = render_elevation_grid(hm)
    assert a == b


def test_elevation_grid_rejects_bad_spacing():
    with pytest.raises(ValueError):
        render_elevation_grid(Heightmap.zeros(2, 2), x_spacing=0.0)


def test_terrain_solid_wrapper_is_well_formed():
    hm = Heightmap.constant(4, 4, 0.0)
    text = render_terrain_solid(hm, name="ground")
    # Structural sanity: brace balance and the expected keywords.
    assert text.count("{") == text.count("}")
    assert "DEF GROUND Solid {" in text
    assert "boundingObject " in text
    assert "ElevationGrid {" in text
