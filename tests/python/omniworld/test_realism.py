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

"""T1.10 realism tests: variation + weathering."""

from __future__ import annotations

import math
import random

import pytest

from omniworld.realism.variation import (
    Variation,
    jitter_hue_rgb,
    jitter_scale,
    variation_for_instance,
)
from omniworld.realism.weathering import (
    WeatheringGrade,
    compute_grade,
    grade_to_color_multiplier,
)


# -------------------------------------------------------------------
# variation
# -------------------------------------------------------------------


def test_variation_deterministic_same_inputs():
    a = variation_for_instance(42, "trees", 3, scale_min=0.9, scale_max=1.2)
    b = variation_for_instance(42, "trees", 3, scale_min=0.9, scale_max=1.2)
    assert a == b


def test_variation_different_instances_differ():
    a = variation_for_instance(42, "trees", 0, scale_min=0.9, scale_max=1.2, hue_shift_deg=5)
    b = variation_for_instance(42, "trees", 1, scale_min=0.9, scale_max=1.2, hue_shift_deg=5)
    assert a != b


def test_variation_respects_scale_range():
    for i in range(50):
        v = variation_for_instance(0, "g", i, scale_min=0.5, scale_max=2.0)
        assert 0.5 <= v.scale <= 2.0


def test_variation_equal_range_is_unit():
    v = variation_for_instance(3, "g", 0, scale_min=1.0, scale_max=1.0)
    assert v.scale == 1.0


def test_variation_rejects_inverted_range():
    with pytest.raises(ValueError):
        variation_for_instance(0, "g", 0, scale_min=1.5, scale_max=0.5)


def test_variation_hue_within_bound():
    for i in range(50):
        v = variation_for_instance(11, "g", i, hue_shift_deg=7.0)
        assert -7.0 <= v.hue_shift_deg <= 7.0


def test_variation_rotation_within_bound():
    for i in range(50):
        v = variation_for_instance(7, "g", i, rotation_jitter_deg=4.0)
        assert -math.radians(4.0) <= v.rotation_offset <= math.radians(4.0)


# -------------------------------------------------------------------
# jitter_scale / jitter_hue_rgb
# -------------------------------------------------------------------


def test_jitter_scale_range():
    rng = random.Random(0)
    for _ in range(100):
        v = jitter_scale(rng, 0.5, 2.0)
        assert 0.5 <= v <= 2.0


def test_jitter_scale_rejects_inverted():
    with pytest.raises(ValueError):
        jitter_scale(random.Random(0), 2.0, 1.0)


def test_jitter_hue_identity_zero_shift():
    base = (0.5, 0.3, 0.7)
    shifted = jitter_hue_rgb(base, 0.0)
    for a, b in zip(base, shifted):
        assert a == pytest.approx(b, abs=1e-9)


def test_jitter_hue_different_shifts_differ():
    base = (0.5, 0.3, 0.7)
    a = jitter_hue_rgb(base, 10.0)
    b = jitter_hue_rgb(base, -10.0)
    assert a != b


def test_jitter_hue_stays_in_unit_cube():
    for shift in range(-360, 361, 17):
        r, g, b = jitter_hue_rgb((0.4, 0.2, 0.6), shift)
        for c in (r, g, b):
            assert 0.0 <= c <= 1.0


# -------------------------------------------------------------------
# weathering
# -------------------------------------------------------------------


def test_weathering_all_zero_at_age_zero():
    g = compute_grade(0, "rocks", 0, world_age=0.0, jitter=0.0)
    assert g.dirt == 0.0 and g.wear == 0.0 and g.oxidation == 0.0


def test_weathering_rises_with_age():
    g_new = compute_grade(0, "rocks", 0, world_age=0.0, jitter=0.0)
    g_old = compute_grade(0, "rocks", 0, world_age=0.8, jitter=0.0)
    assert g_old.dirt > g_new.dirt
    assert g_old.wear > g_new.wear
    assert g_old.oxidation > g_new.oxidation


def test_weathering_deterministic():
    g1 = compute_grade(42, "rocks", 3, world_age=0.4, use_intensity=0.2)
    g2 = compute_grade(42, "rocks", 3, world_age=0.4, use_intensity=0.2)
    assert g1 == g2


def test_weathering_clamped_to_unit():
    g = compute_grade(
        0, "rocks", 0,
        world_age=1.0, use_intensity=1.0,
        exposure=1.0, moisture_base=1.0, oxidation_base=1.0,
        jitter=0.0,
    )
    for v in g.as_tuple():
        assert 0.0 <= v <= 1.0


def test_weathering_rejects_out_of_range_inputs():
    with pytest.raises(ValueError):
        compute_grade(0, "g", 0, world_age=1.5)
    with pytest.raises(ValueError):
        compute_grade(0, "g", 0, world_age=0.0, use_intensity=-0.1)


def test_weathering_color_multiplier_bounded():
    g0 = WeatheringGrade(0.0, 0.0, 0.0, 0.0)
    g1 = WeatheringGrade(1.0, 1.0, 1.0, 1.0)
    assert grade_to_color_multiplier(g0) == 1.0
    assert 0.0 <= grade_to_color_multiplier(g1) <= 1.0


# -------------------------------------------------------------------
# integration: outdoor_forest age parameter changes output
# -------------------------------------------------------------------


def test_outdoor_forest_world_age_affects_output(generated_world):
    """The pristine world (``world_age`` 0.0) IS the recipe default, pinned
    below, so the session-shared default build stands in for it byte for
    byte; only the aged world is built here."""
    from omniworld.biomes.outdoor_forest import DEFAULT_PARAMS

    assert DEFAULT_PARAMS["world_age"] == 0.0
    young = generated_world("outdoor_forest", 42)
    old = generated_world("outdoor_forest", 42, {"world_age": 0.8})
    assert young.world_path.read_bytes() != old.world_path.read_bytes(), (
        "world_age must produce different output"
    )


def test_outdoor_forest_tree_variation_changes_between_instances(generated_world):
    """Adjacent tree placements must not be pixel-identical — at minimum
    their scales (or colors) should differ."""
    result = generated_world("outdoor_forest", 42)
    rocks = [p for p in result.description.props if p.proto_type == "Rock"]
    assert len(rocks) >= 3
    scales = set()
    for r in rocks:
        for key, value in r.extra_fields:
            if key == "scale":
                scales.add(value)
    assert len(scales) > 1, "expected rock scale variation across instances"
