"""T1.6 outdoor_desert recipe tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from omniworld import generate, list_recipes, validate


def test_desert_registered():
    assert "outdoor_desert" in list_recipes()


def test_desert_generates_and_validates(tmp_path: Path):
    out = tmp_path / "desert.wbt"
    result = generate("outdoor_desert", seed=42, out=out)
    assert out.exists()
    text = out.read_text(encoding="utf-8")
    assert "ElevationGrid {" in text
    assert "Rock {" in text
    # Desert has no trees.
    assert "Oak {" not in text
    assert "Pine {" not in text
    # Rock instances carry the tint we injected.
    assert "color 0.86" in text
    # Validation passes.
    report = validate(result.world_path)
    assert report.ok, report.format()


def test_desert_deterministic(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("outdoor_desert", seed=3, out=a)
    generate("outdoor_desert", seed=3, out=b)
    assert a.read_bytes() == b.read_bytes()


def test_desert_different_seeds_differ(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("outdoor_desert", seed=1, out=a)
    generate("outdoor_desert", seed=2, out=b)
    assert a.read_bytes() != b.read_bytes()


def test_desert_clearing_is_empty(tmp_path: Path):
    out = tmp_path / "desert.wbt"
    result = generate("outdoor_desert", seed=7, out=out)
    size = 50.0
    centre = (size / 2.0, size / 2.0)
    clearing_radius = 5.0
    for prop in result.description.props:
        x, y, _ = prop.translation
        dx, dy = x - centre[0], y - centre[1]
        assert dx * dx + dy * dy >= clearing_radius ** 2, (
            f"rock inside desert clearing at {prop.translation}"
        )


def test_desert_rejects_unknown_param(tmp_path: Path):
    with pytest.raises(ValueError):
        generate(
            "outdoor_desert",
            seed=0,
            out=tmp_path / "w.wbt",
            params={"not_a_param": True},
        )


def test_desert_spawn_urdf_places_robot(tmp_path: Path):
    out = tmp_path / "desert_spawn.wbt"
    result = generate(
        "outdoor_desert",
        seed=5,
        out=out,
        params={"spawn_urdf": "../robots/cube_bot.urdf"},
    )
    assert "URDFRobot {" in out.read_text(encoding="utf-8")
    assert validate(result.world_path).ok
