"""T1.6 urban_block recipe tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from omniworld import generate, list_recipes, validate


def test_urban_registered():
    assert "urban_block" in list_recipes()


def test_urban_generates_and_validates(tmp_path: Path):
    out = tmp_path / "urban.wbt"
    result = generate("urban_block", seed=42, out=out)
    assert out.exists()
    text = out.read_text(encoding="utf-8")
    # 4 buildings, at least some street furniture.
    assert text.count("Building {") == 4
    assert "ControlledStreetLight {" in text
    # No terrain — urban is flat.
    assert "ElevationGrid {" not in text
    # Validation passes.
    report = validate(result.world_path)
    assert report.ok, report.format()


def test_urban_deterministic(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("urban_block", seed=3, out=a)
    generate("urban_block", seed=3, out=b)
    assert a.read_bytes() == b.read_bytes()


def test_urban_different_seeds_differ(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("urban_block", seed=1, out=a)
    generate("urban_block", seed=2, out=b)
    assert a.read_bytes() != b.read_bytes()


def test_urban_manifest_counts(tmp_path: Path):
    out = tmp_path / "urban.wbt"
    result = generate("urban_block", seed=1, out=out)
    extras = result.manifest.extra
    assert extras["building_count"] == 4
    assert extras["street_furniture_count"] >= 4


def test_urban_intersection_clear_of_furniture(tmp_path: Path):
    """No street light or bench may sit inside the intersection square."""
    out = tmp_path / "urban.wbt"
    result = generate("urban_block", seed=7, out=out)
    size = 80.0
    road_half = 4.0  # default road_width / 2
    cx = size / 2.0
    for p in result.description.props:
        if p.proto_type not in ("ControlledStreetLight", "Bench"):
            continue
        x, y, _ = p.translation
        assert not (
            cx - road_half <= x <= cx + road_half
            and cx - road_half <= y <= cx + road_half
        ), f"{p.proto_type} inside intersection at {(x, y)}"


def test_urban_spawn_urdf_places_robot(tmp_path: Path):
    out = tmp_path / "urban_spawn.wbt"
    result = generate(
        "urban_block",
        seed=9,
        out=out,
        params={"spawn_urdf": "../robots/cube_bot.urdf"},
    )
    assert "URDFRobot {" in out.read_text(encoding="utf-8")
    assert validate(result.world_path).ok


def test_urban_rejects_unknown_param(tmp_path: Path):
    with pytest.raises(ValueError):
        generate(
            "urban_block",
            seed=0,
            out=tmp_path / "w.wbt",
            params={"bogus": True},
        )


def test_urban_streetlight_controller_silenced(tmp_path: Path):
    """ControlledStreetLight defaults its controller to
    'defective_street_light', which exits status 1 and spams
    warnings under headless runs. The recipe must override to
    '<none>' so batch / CI runs stay clean."""
    out = tmp_path / "urban.wbt"
    generate("urban_block", seed=1, out=out)
    text = out.read_text(encoding="utf-8")
    assert "ControlledStreetLight {" in text
    assert "defective_street_light" not in text
    assert text.count('controller "<none>"') >= 4  # at least one per road side


def test_urban_buildings_fit_inside_lots(tmp_path: Path):
    """Each building's footprint cannot exceed its lot's bounds."""
    out = tmp_path / "urban.wbt"
    result = generate("urban_block", seed=13, out=out)
    size = 80.0
    road_half = 4.0
    buildings = [p for p in result.description.props if p.proto_type == "Building"]
    for b in buildings:
        x, y, _ = b.translation
        # Building centres must lie well inside the nominal lot
        # rectangles (not in the road corridor).
        assert not (abs(x - size / 2.0) <= road_half), (
            f"building at x={x} inside road corridor"
        )
        assert not (abs(y - size / 2.0) <= road_half), (
            f"building at y={y} inside road corridor"
        )
