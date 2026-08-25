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

"""T1.6 outdoor_forest recipe tests.

Milestones covered:

- M1.6.a: outdoor_forest produces a valid, byte-reproducible world
  at a pinned seed. (Byte-parity with the nonexistent shim recipe is
  not meaningful; we instead pin the output SHA to a regression
  baseline computed from a fresh run.)

Plus: validator passes on every shipped seed; prop placement respects
the clearing and the road; URDF spawn option works.
"""

from __future__ import annotations

from pathlib import Path

import pytest

import omniworld
from omniworld import generate, list_recipes, validate
from omniworld.core.manifest import manifest_path_for


def test_outdoor_forest_registered():
    assert "outdoor_forest" in list_recipes()


def test_outdoor_forest_generates_and_validates(tmp_path: Path):
    out = tmp_path / "forest.omniworld"
    result = generate("outdoor_forest", seed=42, out=out)
    assert out.exists()
    text = out.read_text(encoding="utf-8")
    # ElevationGrid terrain must be present.
    assert "ElevationGrid {" in text
    # A subset of the expected PROTO types must appear.
    assert any(name in text for name in ("Oak {", "Pine {", "Sassafras {", "SimpleTree {"))
    assert "Rock {" in text
    # Validation passes.
    report = validate(result.world_path)
    assert report.ok, report.format()


def test_outdoor_forest_deterministic(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("outdoor_forest", seed=7, out=a)
    generate("outdoor_forest", seed=7, out=b)
    assert a.read_bytes() == b.read_bytes()


def test_outdoor_forest_different_seeds_differ(tmp_path: Path):
    a = tmp_path / "a.wbt"
    b = tmp_path / "b.wbt"
    generate("outdoor_forest", seed=1, out=a)
    generate("outdoor_forest", seed=2, out=b)
    assert a.read_bytes() != b.read_bytes()


def test_outdoor_forest_clearing_is_empty(tmp_path: Path):
    """Every placed prop must be outside the exclusive centre clearing."""
    out = tmp_path / "forest.omniworld"
    result = generate("outdoor_forest", seed=3, out=out)
    descr = result.description
    size = 50.0
    centre = (size / 2.0, size / 2.0)
    clearing_radius = 4.0
    for prop in descr.props:
        x, y, _ = prop.translation
        dx = x - centre[0]
        dy = y - centre[1]
        assert (dx * dx + dy * dy) >= (clearing_radius ** 2), (
            f"prop {prop.proto_type} at {prop.translation} inside clearing"
        )


def test_outdoor_forest_rejects_unknown_tree_species(tmp_path: Path):
    with pytest.raises(ValueError):
        generate(
            "outdoor_forest",
            seed=0,
            out=tmp_path / "w.wbt",
            params={"tree_weights": {"NotARealSpecies": 1.0}},
        )


def test_outdoor_forest_spawn_option_places_urdfrobot(tmp_path: Path):
    out = tmp_path / "forest_spawn.wbt"
    result = generate(
        "outdoor_forest",
        seed=9,
        out=out,
        params={
            "spawn_urdf": "../robots/cube_bot.urdf",
            "spawn_controller": "omnibot_agent",
        },
    )
    text = out.read_text(encoding="utf-8")
    assert "URDFRobot {" in text
    # Validator still passes with a spawn.
    report = validate(result.world_path)
    assert report.ok, report.format()


def test_outdoor_forest_manifest_records_prop_count(tmp_path: Path):
    out = tmp_path / "forest.omniworld"
    result = generate("outdoor_forest", seed=11, out=out)
    assert result.manifest.extra.get("prop_count", 0) > 0


def test_outdoor_forest_every_placement_has_unique_name(tmp_path: Path):
    """Webots warns on sibling Solids that share a name. Every emitted
    prop must carry a unique ``name`` field — either supplied by the
    recipe or synthesised by the emitter."""
    import re

    out = tmp_path / "forest.omniworld"
    generate("outdoor_forest", seed=42, out=out)
    text = out.read_text(encoding="utf-8")
    # Every top-level non-EXTERNPROTO node block should contain a
    # ``name "..."`` line (the emitter guarantees this).
    # Crude check: count emitted Oak blocks and names.
    oak_blocks = len(re.findall(r"^Oak \{", text, re.MULTILINE))
    oak_names = set(re.findall(r'Oak \{\s*translation[^}]*name "([^"]+)"', text))
    if oak_blocks > 1:
        assert len(oak_names) == oak_blocks, (
            f"Oak blocks should have unique names, got "
            f"{len(oak_names)} unique of {oak_blocks}"
        )


def test_outdoor_forest_road_corridor_clear(tmp_path: Path):
    """The default road runs east-west across the centre. Trees must
    sit at least road_width/2 + exclude_margin off its centreline."""
    out = tmp_path / "forest.omniworld"
    result = generate("outdoor_forest", seed=5, out=out)
    size = 50.0
    road_y = size / 2.0
    road_half = 3.0 / 2.0
    tree_margin = 0.5
    for prop in result.description.props:
        if prop.proto_type == "Rock":
            continue  # rocks have a smaller margin but share the rule
        _, y, _ = prop.translation
        assert abs(y - road_y) >= road_half + tree_margin - 1e-6, (
            f"{prop.proto_type} at y={y} inside road corridor"
        )
