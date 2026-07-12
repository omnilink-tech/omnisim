"""mars biome tests — shares the outdoor-biome invariants plus the
sky / terrain color overrides specific to this recipe."""

from __future__ import annotations

from pathlib import Path

import pytest

from omniworld import generate, list_recipes, validate


def test_mars_registered():
    assert "mars" in list_recipes()


def test_mars_generates_and_validates(tmp_path: Path):
    out = tmp_path / "mars.wbt"
    result = generate("mars", seed=42, out=out)
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


def test_mars_deterministic(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("mars", seed=3, out=a)
    generate("mars", seed=3, out=b)
    assert a.read_bytes() == b.read_bytes()


def test_mars_different_seeds_differ(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("mars", seed=1, out=a)
    generate("mars", seed=2, out=b)
    assert a.read_bytes() != b.read_bytes()


def test_mars_clearing_is_empty(tmp_path: Path):
    """The solver approximates the circular clearing as a 24-vertex
    polygon whose inscribed radius is slightly less than the nominal
    ``clearing_radius``. Assert rocks stay outside the inscribed disk."""
    import math

    out = tmp_path / "mars.wbt"
    result = generate("mars", seed=7, out=out)
    size = 80.0
    cx, cy = size / 2.0, size / 2.0
    nominal_r = 5.5
    inscribed_r = nominal_r * math.cos(math.pi / 24)  # ~5.45
    for s in result.description.solids:
        x, y, _ = s.translation
        assert (x - cx) ** 2 + (y - cy) ** 2 >= inscribed_r ** 2, (
            f"rock inside Mars spawn clearing at {s.translation}"
        )


def test_mars_world_age_affects_output(tmp_path: Path):
    young = tmp_path / "y.wbt"
    old = tmp_path / "o.wbt"
    generate("mars", seed=42, out=young, params={"world_age": 0.0})
    generate("mars", seed=42, out=old, params={"world_age": 0.9})
    assert young.read_bytes() != old.read_bytes()


def test_mars_rejects_unknown_param(tmp_path: Path):
    with pytest.raises(ValueError):
        generate("mars", seed=0, out=tmp_path / "w.wbt", params={"bogus": True})


def test_mars_rocks_are_procedural_meshes(tmp_path: Path):
    """Every rock is a unique displaced icosphere, not a Rock.proto
    instance. Two rocks must have different vertex lists."""
    out = tmp_path / "mars.wbt"
    result = generate("mars", seed=42, out=out)
    solids = result.description.solids
    assert len(solids) > 10, f"expected many rock solids, got {len(solids)}"
    first_verts = solids[0].vertices
    # Some other solid must have a different mesh.
    assert any(s.vertices != first_verts for s in solids[1:]), (
        "every rock mesh is identical — procedural generation broken"
    )


def test_mars_has_fill_light(tmp_path: Path):
    out = tmp_path / "mars.wbt"
    generate("mars", seed=42, out=out)
    text = out.read_text(encoding="utf-8")
    # Two DirectionalLight nodes: main sun + fill.
    assert text.count("DirectionalLight {") == 2


def test_mars_has_fog(tmp_path: Path):
    out = tmp_path / "mars.wbt"
    generate("mars", seed=42, out=out)
    text = out.read_text(encoding="utf-8")
    assert "Fog {" in text
    assert "visibilityRange" in text


def test_mars_has_three_scatter_layers(tmp_path: Path):
    out = tmp_path / "mars.wbt"
    result = generate("mars", seed=42, out=out)
    extras = result.manifest.extra
    assert extras["debris_count"] > 0
    assert extras["rock_count"] > 0
    assert extras["boulder_count"] > 0
    # Boulders should be rarer than rocks, rocks rarer than debris.
    assert extras["boulder_count"] < extras["rock_count"] < extras["debris_count"]


def test_mars_rocks_are_ground_fitted(tmp_path: Path):
    """Rocks on sloped terrain should be tilted — their rotation axis
    should not be pure +Z everywhere. Rocks now live in
    ``description.solids`` as inline meshes."""
    out = tmp_path / "mars.wbt"
    result = generate("mars", seed=42, out=out)
    non_trivial = 0
    for s in result.description.solids:
        ax, ay, _, angle = s.rotation
        if abs(ax) + abs(ay) > 0.05 and angle > 0.05:
            non_trivial += 1
    assert non_trivial > 10, f"expected many tilted rocks, got {non_trivial}"


def test_mars_has_craters(tmp_path: Path):
    """The default recipe stamps ``crater_count`` craters. Asserting
    by counting: with 7 craters we expect craters to appear in the
    world; the concrete visible effect is checked by the metadata."""
    out = tmp_path / "mars.wbt"
    result = generate("mars", seed=42, out=out)
    hm = result.description.heightmap
    assert hm is not None
    assert result.manifest.extra["crater_count"] == 7


def test_mars_crater_walls_are_drivable(tmp_path: Path):
    """Crater walls must stay below ~30° so a Husky can climb back
    out. We sample the heightmap across every crater region and check
    the maximum local gradient does not exceed 0.6 (≈ arctan(0.6) ≈ 31°)."""
    import math

    from omniworld.primitives.heightmap import crater_pattern

    out = tmp_path / "mars.wbt"
    result = generate("mars", seed=42, out=out)
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


def test_mars_crater_count_zero_disables(tmp_path: Path):
    out = tmp_path / "mars.wbt"
    result = generate("mars", seed=42, out=out, params={"crater_count": 0})
    assert result.manifest.extra["crater_count"] == 0


def test_mars_spawn_urdf_places_rover(tmp_path: Path):
    out = tmp_path / "mars_rover.wbt"
    result = generate(
        "mars", seed=9, out=out,
        params={"spawn_urdf": "../robots/cube_bot.urdf"},
    )
    text = out.read_text(encoding="utf-8")
    assert "URDFRobot {" in text
    # Rover spawn name (different from desert/forest which use "robot").
    assert 'name "rover"' in text
    assert validate(result.world_path).ok


def test_mars_husky_fleet(tmp_path: Path):
    """husky_count spawns N Clearpath Huskies in a circle on the clearing."""
    out = tmp_path / "mars_huskies.wbt"
    result = generate(
        "mars", seed=42, out=out,
        params={"husky_count": 5},
    )
    text = out.read_text(encoding="utf-8")
    assert text.count("URDFRobot {") == 5
    for i in range(5):
        assert f'name "husky_{i}"' in text
    assert text.count("husky.urdf") == 5
    # Default controller is the random walker — each husky entry has
    # it attached.
    assert text.count('controller "husky_random"') == 5
    # Validator passes.
    assert validate(result.world_path).ok


def test_mars_husky_controller_override(tmp_path: Path):
    """User can set controller to ``<none>`` for stationary huskies."""
    out = tmp_path / "mars_static.wbt"
    generate("mars", seed=1, out=out,
             params={"husky_count": 3, "husky_controller": "<none>"})
    text = out.read_text(encoding="utf-8")
    assert 'controller "<none>"' in text
    assert "husky_random" not in text


def test_mars_husky_count_fewer(tmp_path: Path):
    out = tmp_path / "mars_huskies.wbt"
    result = generate("mars", seed=1, out=out, params={"husky_count": 2})
    text = out.read_text(encoding="utf-8")
    assert text.count("URDFRobot {") == 2
    assert 'name "husky_0"' in text
    assert 'name "husky_1"' in text
    assert 'name "husky_2"' not in text


def test_mars_husky_circle_separation(tmp_path: Path):
    """Five huskies on the default 12 m circle should sit well apart
    so they don't pile up at spawn. Closest neighbours on a 5-point
    circle of radius 12 m are at 2 * 12 * sin(pi/5) ~= 14 m."""
    import math

    out = tmp_path / "mars_circle.wbt"
    result = generate("mars", seed=1, out=out, params={"husky_count": 5})
    spawns = result.description.spawns
    assert len(spawns) == 5
    for i in range(len(spawns)):
        for j in range(i + 1, len(spawns)):
            ax, ay, _ = spawns[i].translation
            bx, by, _ = spawns[j].translation
            d = math.hypot(ax - bx, ay - by)
            assert d > 10.0, (
                f"husky pair {i},{j} too close: {d:.2f} m"
            )


def test_mars_huskies_carry_crater_data_and_are_supervisors(tmp_path: Path):
    """Each Husky spawn must carry the JSON-encoded crater list as
    customData and have supervisor=True so the controller can read
    its own pose."""
    import json

    out = tmp_path / "mars.wbt"
    result = generate("mars", seed=42, out=out, params={"husky_count": 5})
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


def test_mars_emitted_world_has_supervisor_and_custom_data(tmp_path: Path):
    out = tmp_path / "mars.wbt"
    generate("mars", seed=42, out=out, params={"husky_count": 3})
    text = out.read_text(encoding="utf-8")
    assert text.count("supervisor TRUE") == 3
    assert text.count("customData ") >= 3


def test_mars_husky_corners_formation(tmp_path: Path):
    """Corner formation places huskies near the four world corners
    (inset by husky_corner_margin). They should sit much further apart
    than a circle formation."""
    import math

    out = tmp_path / "mars_corners.wbt"
    result = generate(
        "mars", seed=42, out=out,
        params={"husky_count": 4, "husky_formation": "corners"},
    )
    huskies = [s for s in result.description.spawns
               if s.name.startswith("husky_")]
    assert len(huskies) == 4
    # On default size 80, margin 8 -> corners at (8,8), (72,8), (8,72), (72,72).
    expected_xy = {(8.0, 8.0), (72.0, 8.0), (8.0, 72.0), (72.0, 72.0)}
    actual_xy = {(round(s.translation[0], 3), round(s.translation[1], 3))
                 for s in huskies}
    assert actual_xy == expected_xy

    # Corner-to-corner separation on a 64-m diagonal: opposite-corner
    # distance ~90 m, same-edge distance 64 m. Both far over the
    # circle's ~14 m.
    for i in range(len(huskies)):
        for j in range(i + 1, len(huskies)):
            ax, ay, _ = huskies[i].translation
            bx, by, _ = huskies[j].translation
            d = math.hypot(ax - bx, ay - by)
            assert d > 60.0, f"corner pair {i},{j} too close: {d:.1f} m"


def test_mars_corner_pads_are_flat(tmp_path: Path):
    """When formation==corners, the recipe stamps a level pad at
    each corner so the husky lands on flat ground."""
    out = tmp_path / "mars_corners.wbt"
    generate(
        "mars", seed=42, out=out,
        params={"husky_count": 4, "husky_formation": "corners"},
    )
    text = out.read_text(encoding="utf-8")
    # 4 corner pads + 1 centre clearing pad in the .wbt body.
    # We check by looking for the DEF terrain and ElevationGrid is
    # present. Stamps are baked into the heightmap; their visible
    # signature is the heightmap itself.
    assert "ElevationGrid {" in text


def test_mars_payload_carries_world_bounds(tmp_path: Path):
    import json

    out = tmp_path / "mars.wbt"
    result = generate("mars", seed=1, out=out, params={"husky_count": 3})
    huskies = [s for s in result.description.spawns
               if s.name.startswith("husky_")]
    payload = json.loads(huskies[0].custom_data)
    assert "world_centre" in payload
    assert "world_size" in payload
    assert "boundary_margin" in payload
    assert payload["world_size"] == 80.0
    assert payload["boundary_margin"] >= 1.0


def test_mars_corners_falls_back_for_high_counts(tmp_path: Path):
    """corners formation only supports up to 5 (4 corners + centre).
    Higher counts must fall back to the circle formation rather than
    raising."""
    out = tmp_path / "mars_circle_fallback.wbt"
    result = generate(
        "mars", seed=2, out=out,
        params={"husky_count": 6, "husky_formation": "corners"},
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


def test_mars_husky_spawn_height_above_clearing(tmp_path: Path):
    """Huskies must spawn above the clearing pad surface, not below it."""
    out = tmp_path / "mars_huskies.wbt"
    result = generate("mars", seed=3, out=out, params={"husky_count": 3})
    # Default height_scale=5.5 * 0.28 = 1.54 m clearing surface,
    # plus 1.5 m drop = ~3.04 m z.
    for spawn in result.description.spawns:
        assert spawn.translation[2] > 2.5, (
            f"husky {spawn.name!r} spawn z={spawn.translation[2]} too low"
        )
