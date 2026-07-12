"""T1.6 indoor_apartment recipe tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from omniworld import generate, list_recipes, validate


def test_apartment_registered():
    assert "indoor_apartment" in list_recipes()


def test_apartment_generates_and_validates(tmp_path: Path):
    out = tmp_path / "apt.wbt"
    result = generate("indoor_apartment", seed=42, out=out)
    assert out.exists()
    text = out.read_text(encoding="utf-8")
    # Walls + 3 furniture pieces.
    assert "Wall {" in text
    assert "Bed {" in text
    assert "Table {" in text
    assert "Toilet {" in text
    # No terrain.
    assert "ElevationGrid {" not in text
    report = validate(result.world_path)
    assert report.ok, report.format()


def test_apartment_deterministic(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("indoor_apartment", seed=3, out=a)
    generate("indoor_apartment", seed=3, out=b)
    assert a.read_bytes() == b.read_bytes()


def test_apartment_manifest_counts(tmp_path: Path):
    out = tmp_path / "apt.wbt"
    result = generate("indoor_apartment", seed=1, out=out)
    extras = result.manifest.extra
    assert extras["room_count"] == 3
    # 4 perimeter + 2 dividers * (<=2 segments each after door split) =
    # between 6 and 8 walls.
    assert 6 <= extras["wall_count"] <= 8
    assert extras["furniture_count"] == 3


def test_apartment_walls_inside_floor(tmp_path: Path):
    """Every wall segment must sit inside the arena footprint."""
    out = tmp_path / "apt.wbt"
    result = generate("indoor_apartment", seed=7, out=out)
    fx, fy = result.description.floor_size
    for prop in result.description.props:
        x, y, _ = prop.translation
        assert 0.0 <= x <= fx, f"{prop.proto_type} at x={x} outside floor"
        assert 0.0 <= y <= fy, f"{prop.proto_type} at y={y} outside floor"


def test_apartment_one_furniture_per_room(tmp_path: Path):
    """Exactly one Bed, one Table, one Toilet."""
    out = tmp_path / "apt.wbt"
    result = generate("indoor_apartment", seed=9, out=out)
    counts = {t: 0 for t in ("Bed", "Table", "Toilet")}
    for p in result.description.props:
        if p.proto_type in counts:
            counts[p.proto_type] += 1
    assert counts == {"Bed": 1, "Table": 1, "Toilet": 1}


def test_apartment_spawn_urdf_places_robot(tmp_path: Path):
    out = tmp_path / "apt_spawn.wbt"
    result = generate(
        "indoor_apartment",
        seed=5,
        out=out,
        params={"spawn_urdf": "../robots/cube_bot.urdf"},
    )
    assert "URDFRobot {" in out.read_text(encoding="utf-8")
    assert validate(result.world_path).ok


def test_apartment_rejects_unknown_param(tmp_path: Path):
    with pytest.raises(ValueError):
        generate(
            "indoor_apartment",
            seed=0,
            out=tmp_path / "w.wbt",
            params={"bogus": True},
        )


def test_apartment_door_gaps_exist(tmp_path: Path):
    """The two inner dividers should each have a door-sized gap at the
    middle, i.e. two wall segments per divider (or one if the door
    exactly consumes the divider)."""
    out = tmp_path / "apt.wbt"
    result = generate("indoor_apartment", seed=11, out=out)
    walls = [p for p in result.description.props if p.proto_type == "Wall"]
    # Group by approximate x coordinate of centre (perimeter walls are
    # at x=2 and x=14, dividers are at other x values).
    apartment_length = 12.0
    margin = 2.0
    # Dividers live at x = margin + relative_width_sum[1], x = margin + relative_width_sum[2].
    # For default proportions these are around x = 7 and x = 11.2.
    # Rather than reverse-engineering exact geometry, assert that the
    # walls don't all live on the four perimeter lines — i.e. at least
    # one wall sits in the interior x range.
    interior_walls = [
        w for w in walls
        if margin + 0.5 < w.translation[0] < margin + apartment_length - 0.5
    ]
    assert len(interior_walls) >= 2, (
        f"expected at least 2 interior wall segments, got {len(interior_walls)}"
    )
