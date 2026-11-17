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

"""T1.6 outdoor_desert recipe tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from omniworld import generate, list_recipes, validate


def test_desert_registered():
    assert "outdoor_desert" in list_recipes()


def test_desert_generates_and_validates(generated_world):
    result = generated_world("outdoor_desert", 42)
    out = result.world_path
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


def test_desert_deterministic(tmp_path: Path, generated_world):
    """A FRESH build into this test's own tmp_path is byte-identical to the
    session-shared one -- which is also what licenses the other tests here
    to read the shared result."""
    shared = generated_world("outdoor_desert", 42)
    fresh = tmp_path / "fresh.wbt"
    generate("outdoor_desert", seed=42, out=fresh)
    assert fresh.read_bytes() == shared.world_path.read_bytes()


def test_desert_different_seeds_differ(generated_world):
    a = generated_world("outdoor_desert", 42).world_path
    b = generated_world("outdoor_desert", 7).world_path
    assert a.read_bytes() != b.read_bytes()


def test_desert_clearing_is_empty(generated_world):
    result = generated_world("outdoor_desert", 7)
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
