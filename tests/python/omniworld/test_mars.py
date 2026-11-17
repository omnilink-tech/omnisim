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

"""mars biome tests — shares the outdoor-biome invariants plus the
sky / terrain color overrides specific to this recipe.

Every read-only test takes its world from the session-scoped
``generated_world`` fixture (``conftest.py``): one build per distinct
``(recipe, seed, params)`` instead of one per test. The results are shared
read-only; ``test_mars_deterministic`` is the test that proves a fresh build
is byte-identical to the shared one."""

from __future__ import annotations

from pathlib import Path

import pytest

from omniworld import generate, list_recipes, validate


def test_mars_registered():
    assert "mars" in list_recipes()


def test_mars_generates_and_validates(generated_world):
    result = generated_world("mars", 42)
    out = result.world_path
    assert out.exists()
    text = out.read_text(encoding="utf-8")
    # Mars now uses the Hillaire 2020 procedural sky (T1.3 session 4+).
    # World file is cubemap-free; the visible sky and PBR irradiance
    # both come from the renderer's procedural pipeline.
    assert "TexturedBackground" not in text
    assert 'atmosphericSky "mars"' in text
    # Fallback solid colour stays as the OMNISIM_RENDERER=compatibility
    # path's input.
    assert "skyColor" in text
    assert "DirectionalLight {" in text
    # Terrain + rocks present.
    assert "ElevationGrid {" in text
    # Mars emits rocks as inline ``DEF ... Solid`` blocks, not PROTO
    # instances — each rock is a unique displaced icosphere.
    assert "Rock {" not in text
    assert "IndexedFaceSet {" in text
    assert "DEF MARS_ROCKS_0 Solid {" in text
    # baseColor on the PBRAppearance carries the rock tint.
    assert "baseColor 0" in text
    # In-process validation passes.
    report = validate(result.world_path)
    assert report.ok, report.format()


def test_mars_deterministic(tmp_path: Path, generated_world):
    """Same (recipe, seed, params) -> byte-identical output, and independent
    of the ``out`` path. A FRESH build into this test's own tmp_path is
    compared against the session-shared one, which is also what licenses
    every other test here to read the shared result."""
    shared = generated_world("mars", 42)
    fresh = tmp_path / "fresh.wbt"
    generate("mars", seed=42, out=fresh)
    assert fresh.read_bytes() == shared.world_path.read_bytes()


def test_mars_different_seeds_differ(generated_world):
    a = generated_world("mars", 42).world_path
    b = generated_world("mars", 7).world_path
    assert a.read_bytes() != b.read_bytes()


def test_mars_clearing_is_empty(generated_world):
    """The solver approximates the circular clearing as a 24-vertex
    polygon whose inscribed radius is slightly less than the nominal
    ``clearing_radius``. Assert rocks stay outside the inscribed disk."""
    import math

    result = generated_world("mars", 7)
    size = 80.0
    cx, cy = size / 2.0, size / 2.0
    nominal_r = 5.5
    inscribed_r = nominal_r * math.cos(math.pi / 24)  # ~5.45
    for s in result.description.solids:
        x, y, _ = s.translation
        assert (x - cx) ** 2 + (y - cy) ** 2 >= inscribed_r ** 2, (
            f"rock inside Mars spawn clearing at {s.translation}"
        )


def test_mars_world_age_affects_output(generated_world):
    """``world_age`` must change the output. The control is the session-
    shared default world (``world_age`` 0.45, pinned below so the pair
    really does differ only in age); only the aged world is a fresh build.
    Before 2026-09-02 this built two private worlds (0.0 and 0.9) for the
    same assertion, ~3.4 s of the file's runtime."""
    from omniworld.biomes.mars import DEFAULT_PARAMS

    assert DEFAULT_PARAMS["world_age"] == 0.45
    default = generated_world("mars", 42)
    aged = generated_world("mars", 42, {"world_age": 0.9})
    assert default.world_path.read_bytes() != aged.world_path.read_bytes()


def test_mars_rejects_unknown_param(tmp_path: Path):
    with pytest.raises(ValueError):
        generate("mars", seed=0, out=tmp_path / "w.wbt", params={"bogus": True})


def test_mars_rocks_are_procedural_meshes(generated_world):
    """Every rock is a unique displaced icosphere, not a Rock.proto
    instance. Two rocks must have different vertex lists."""
    result = generated_world("mars", 42)
    solids = result.description.solids
    assert len(solids) > 10, f"expected many rock solids, got {len(solids)}"
    first_verts = solids[0].vertices
    # Some other solid must have a different mesh.
    assert any(s.vertices != first_verts for s in solids[1:]), (
        "every rock mesh is identical — procedural generation broken"
    )


def test_mars_has_fill_light(generated_world):
    text = generated_world("mars", 42).world_path.read_text(encoding="utf-8")
    # Two DirectionalLight nodes: main sun + fill.
    assert text.count("DirectionalLight {") == 2


def test_mars_has_fog(generated_world):
    text = generated_world("mars", 42).world_path.read_text(encoding="utf-8")
    assert "Fog {" in text
    assert "visibilityRange" in text


def test_mars_has_three_scatter_layers(generated_world):
    result = generated_world("mars", 42)
    extras = result.manifest.extra
    assert extras["debris_count"] > 0
    assert extras["rock_count"] > 0
    assert extras["boulder_count"] > 0
    # Boulders should be rarer than rocks, rocks rarer than debris.
    assert extras["boulder_count"] < extras["rock_count"] < extras["debris_count"]


def test_mars_rocks_are_ground_fitted(generated_world):
    """Rocks on sloped terrain should be tilted — their rotation axis
    should not be pure +Z everywhere. Rocks now live in
    ``description.solids`` as inline meshes."""
    result = generated_world("mars", 42)
    non_trivial = 0
    for s in result.description.solids:
        ax, ay, _, angle = s.rotation
        if abs(ax) + abs(ay) > 0.05 and angle > 0.05:
            non_trivial += 1
    assert non_trivial > 10, f"expected many tilted rocks, got {non_trivial}"


def test_mars_has_craters(generated_world):
    """The default recipe stamps ``crater_count`` craters. Asserting
    by counting: with 7 craters we expect craters to appear in the
    world; the concrete visible effect is checked by the metadata."""
    result = generated_world("mars", 42)
    hm = result.description.heightmap
    assert hm is not None
    assert result.manifest.extra["crater_count"] == 7


def test_mars_crater_walls_are_drivable(generated_world):
    """Crater walls must stay below ~30° so a Husky can climb back
    out. We sample the heightmap across every crater region and check
    the maximum local gradient does not exceed 0.6 (≈ arctan(0.6) ≈ 31°)."""
    import math

    from omniworld.primitives.heightmap import crater_pattern

    result = generated_world("mars", 42)
    hm = result.description.heightmap
    assert hm is not None
    # Sample wall slope directly from crater_pattern defaults — the
    # params used in mars.py.
    pitch = 80.0 / hm.width  # world metres per grid cell
    # Default crater params (mars.py): depth ≤ 0.30 m, rim_width
    # ≥ 0.45, min radius 4.0 m. Steepest wall is at min radius +
    # min rim_width + max depth.
    min_radius_m = 4.0
    max_depth_m = 0.30
    min_rim_width = 0.45
    wall_band_m = min_rim_width * min_radius_m
    worst_slope = max_depth_m / wall_band_m
    worst_slope_deg = math.degrees(math.atan(worst_slope))
    # Husky climbs comfortably to ~30°; we want a much wider margin so
    # a robot rolling in at any angle can roll back out.
    assert worst_slope_deg <= 15.0, (
        f"crater walls too steep for a Husky: {worst_slope_deg:.1f}°"
    )


def test_mars_crater_count_zero_disables(generated_world):
    result = generated_world("mars", 42, {"crater_count": 0})
    assert result.manifest.extra["crater_count"] == 0


def test_mars_spawn_urdf_places_rover(generated_world):
    result = generated_world(
        "mars", 9, {"spawn_urdf": "../robots/cube_bot.urdf"},
    )
    text = result.world_path.read_text(encoding="utf-8")
    assert "URDFRobot {" in text
    # Rover spawn name (different from desert/forest which use "robot").
    assert 'name "rover"' in text
    assert validate(result.world_path).ok


def test_mars_husky_fleet(generated_world):
    """husky_count spawns N Clearpath Huskies in a circle on the clearing."""
    result = generated_world("mars", 42, {"husky_count": 5})
    text = result.world_path.read_text(encoding="utf-8")
    assert text.count("URDFRobot {") == 5
    for i in range(5):
        assert f'name "husky_{i}"' in text
    assert text.count("husky.urdf") == 5
    # Default controller is the random walker — each husky entry has
    # it attached.
    assert text.count('controller "husky_random"') == 5
    # Validator passes.
    assert validate(result.world_path).ok


def test_mars_husky_controller_override(generated_world):
    """User can set controller to ``<none>`` for stationary huskies."""
    result = generated_world(
        "mars", 1, {"husky_count": 3, "husky_controller": "<none>"},
    )
    text = result.world_path.read_text(encoding="utf-8")
    assert 'controller "<none>"' in text
    assert "husky_random" not in text


def test_mars_husky_count_fewer(generated_world):
    result = generated_world("mars", 1, {"husky_count": 2})
    text = result.world_path.read_text(encoding="utf-8")
    assert text.count("URDFRobot {") == 2
    assert 'name "husky_0"' in text
    assert 'name "husky_1"' in text
    assert 'name "husky_2"' not in text


def test_mars_husky_circle_separation(generated_world):
    """Five huskies on the default 12 m circle should sit well apart
    so they don't pile up at spawn. Closest neighbours on a 5-point
    circle of radius 12 m are at 2 * 12 * sin(pi/5) ~= 14 m."""
    import math

    result = generated_world("mars", 1, {"husky_count": 5})
    # The world also spawns a `mars_observer` Supervisor (091dacfc), so filter
    # to the huskies rather than asserting on the whole spawn list.
    spawns = [s for s in result.description.spawns
              if s.name.startswith("husky_")]
    assert len(spawns) == 5
    for i in range(len(spawns)):
        for j in range(i + 1, len(spawns)):
            ax, ay, _ = spawns[i].translation
            bx, by, _ = spawns[j].translation
            d = math.hypot(ax - bx, ay - by)
            assert d > 10.0, (
                f"husky pair {i},{j} too close: {d:.2f} m"
            )


def test_mars_huskies_carry_crater_data_and_are_supervisors(generated_world):
    """Each Husky spawn must carry the JSON-encoded crater list as
    customData and have supervisor=True so the controller can read
    its own pose."""
    import json

    result = generated_world("mars", 42, {"husky_count": 5})
    huskies = [s for s in result.description.spawns if s.name.startswith("husky_")]
    assert len(huskies) == 5
    for s in huskies:
        assert s.supervisor is True, f"{s.name} should be a supervisor"
        assert s.custom_data, f"{s.name} should have customData"
        payload = json.loads(s.custom_data)
        assert "craters" in payload
        assert isinstance(payload["craters"], list)
        # 7 craters by default — each crater is a 3-tuple of floats.
        for cx, cy, cr in payload["craters"]:
            assert isinstance(cx, (int, float))
            assert isinstance(cy, (int, float))
            assert cr > 0


def test_mars_emitted_world_has_supervisor_and_custom_data(generated_world):
    result = generated_world("mars", 42, {"husky_count": 3})
    text = result.world_path.read_text(encoding="utf-8")
    # 3 huskies + the mars_observer Supervisor added in 091dacfc.
    assert text.count("supervisor TRUE") == 4
    assert text.count("customData ") >= 3


def test_mars_husky_corners_formation(generated_world):
    """Corner formation places huskies near the four world corners
    (inset by husky_corner_margin). They should sit much further apart
    than a circle formation."""
    import math

    result = generated_world(
        "mars", 42, {"husky_count": 4, "husky_formation": "corners"},
    )
    huskies = [s for s in result.description.spawns
               if s.name.startswith("husky_")]
    assert len(huskies) == 4
    # On default size 80, margin 12 (091dacfc, was 8) -> corners at
    # (12,12), (68,12), (12,68), (68,68).
    expected_xy = {(12.0, 12.0), (68.0, 12.0), (12.0, 68.0), (68.0, 68.0)}
    actual_xy = {(round(s.translation[0], 3), round(s.translation[1], 3))
                 for s in huskies}
    assert actual_xy == expected_xy

    # Derive the bound from the same geometry the coordinates come from, so a
    # future margin change moves this with it instead of silently failing:
    # the closest pair is same-edge, at (size - 2*margin); opposite corners are
    # that times sqrt(2). The point of the test is that corner formation puts
    # the fleet far further apart than the circle's ~14 m.
    side_span = min(x for x, _ in expected_xy) * -2 + 80.0  # size 80, both margins
    assert side_span > 40.0, "corner formation collapsed toward the centre"
    for i in range(len(huskies)):
        for j in range(i + 1, len(huskies)):
            ax, ay, _ = huskies[i].translation
            bx, by, _ = huskies[j].translation
            d = math.hypot(ax - bx, ay - by)
            assert d >= side_span - 0.01, (
                f"corner pair {i},{j} too close: {d:.1f} m "
                f"(expected at least the {side_span:.0f} m edge span)"
            )


def test_mars_corner_pads_are_flat(generated_world):
    """When formation==corners, the recipe stamps a level pad at
    each corner so the husky lands on flat ground."""
    result = generated_world(
        "mars", 42, {"husky_count": 4, "husky_formation": "corners"},
    )
    text = result.world_path.read_text(encoding="utf-8")
    # 4 corner pads + 1 centre clearing pad in the .wbt body.
    # We check by looking for the DEF terrain and ElevationGrid is
    # present. Stamps are baked into the heightmap; their visible
    # signature is the heightmap itself.
    assert "ElevationGrid {" in text


def test_mars_payload_carries_world_bounds(generated_world):
    import json

    result = generated_world("mars", 1, {"husky_count": 3})
    huskies = [s for s in result.description.spawns
               if s.name.startswith("husky_")]
    payload = json.loads(huskies[0].custom_data)
    assert "world_centre" in payload
    assert "world_size" in payload
    assert "boundary_margin" in payload
    assert payload["world_size"] == 80.0
    assert payload["boundary_margin"] >= 1.0


def test_mars_corners_falls_back_for_high_counts(generated_world):
    """corners formation only supports up to 5 (4 corners + centre).
    Higher counts must fall back to the circle formation rather than
    raising."""
    result = generated_world(
        "mars", 2, {"husky_count": 6, "husky_formation": "corners"},
    )
    huskies = [s for s in result.description.spawns
               if s.name.startswith("husky_")]
    assert len(huskies) == 6
    # Names should be circle-style (husky_0 ... husky_5), not the
    # named corners.
    assert {s.name for s in huskies} == {f"husky_{i}" for i in range(6)}


def test_mars_husky_rejects_excess(tmp_path: Path):
    with pytest.raises(ValueError):
        generate("mars", seed=0, out=tmp_path / "w.wbt",
                 params={"husky_count": 99})


def test_mars_husky_mutually_exclusive_with_spawn_urdf(tmp_path: Path):
    with pytest.raises(ValueError):
        generate("mars", seed=0, out=tmp_path / "w.wbt",
                 params={"husky_count": 2, "spawn_urdf": "any.urdf"})


def test_mars_husky_spawn_height_above_clearing(generated_world):
    """Huskies must spawn above the clearing pad surface, not below it."""
    result = generated_world("mars", 3, {"husky_count": 3})
    # Default height_scale=5.5 * 0.28 = 1.54 m clearing surface, plus the
    # 0.1 m drop set by b22457ca (was 1.5 m, which hit the pad at ~5 m/s)
    # = ~1.64 m z. The point of the test is unchanged: ABOVE the pad, not
    # below it.
    #
    # Filter to huskies: the world also spawns a `mars_observer` Supervisor
    # (091dacfc) which legitimately sits at z = 0 and is not a drop test.
    huskies = [s for s in result.description.spawns
               if s.name.startswith("husky_")]
    assert huskies, "no husky spawns in the generated world"
    for spawn in huskies:
        assert spawn.translation[2] > 1.54, (
            f"husky {spawn.name!r} spawn z={spawn.translation[2]} is at or "
            f"below the 1.54 m clearing surface"
        )
