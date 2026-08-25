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

"""T1.4 Layout DSL + solver + schema tests.

Milestones covered:

- M1.4.a: a hand-coded Layout with a few zones + a prop group produces
  the expected placements on each seed (proxy for the shim-recipe
  parity the plan calls out).
- M1.4.b: zone priority / exclusivity drops prop candidates inside an
  exclusive higher-priority zone.
- M1.4.c: Layout round-trips through JSON without loss — both fields
  and referential integrity survive.
"""

from __future__ import annotations

import json

import pytest

from omniworld.core.layout import (
    Layout,
    Path,
    PropGroup,
    Stamp,
    Zone,
    distance_point_polyline,
    point_in_polygon,
)
from omniworld.core.recipe import Spawn
from omniworld.core.schema import from_dict, from_json, to_dict, to_json
from omniworld.core.solver import solve


# -------------------------------------------------------------------
# Layout DSL
# -------------------------------------------------------------------


def test_layout_rejects_duplicate_zone_names():
    layout = Layout(world_size=(100.0, 100.0))
    layout.add_zone(Zone(name="grass", polygon=((0, 0), (10, 0), (10, 10), (0, 10))))
    with pytest.raises(ValueError):
        layout.add_zone(
            Zone(name="grass", polygon=((5, 5), (6, 5), (6, 6), (5, 6)))
        )


def test_layout_validates_prop_group_references():
    layout = Layout(world_size=(50.0, 50.0))
    layout.add_zone(Zone(name="grass", polygon=((0, 0), (10, 0), (10, 10), (0, 10))))
    with pytest.raises(ValueError):
        layout.add_prop_group(
            PropGroup(
                name="trees",
                proto_weights=(("OakTree", 1.0),),
                scatter="poisson",
                include_zones=("nope",),
            )
        )


def test_stamp_must_reference_known_zone():
    layout = Layout(world_size=(10.0, 10.0))
    with pytest.raises(ValueError):
        layout.add_stamp(Stamp(name="hill", zone="missing"))


# -------------------------------------------------------------------
# Geometry helpers
# -------------------------------------------------------------------


def test_point_in_polygon_basic():
    poly = ((0, 0), (10, 0), (10, 10), (0, 10))
    assert point_in_polygon((5, 5), poly)
    assert not point_in_polygon((-1, 5), poly)


def test_distance_point_polyline():
    line = ((0.0, 0.0), (10.0, 0.0))
    assert distance_point_polyline((5.0, 3.0), line) == pytest.approx(3.0)
    assert distance_point_polyline((-5.0, 0.0), line) == pytest.approx(5.0)
    # Single-point / empty polyline returns +inf.
    assert distance_point_polyline((0.0, 0.0), ()) == float("inf")


# -------------------------------------------------------------------
# Solver: basic prop placement
# -------------------------------------------------------------------


def _make_simple_layout() -> Layout:
    layout = Layout(world_size=(50.0, 50.0))
    layout.add_zone(Zone(
        name="forest",
        polygon=((5, 5), (45, 5), (45, 45), (5, 45)),
        priority=0,
    ))
    layout.add_prop_group(PropGroup(
        name="trees",
        proto_weights=(("OakTree", 0.7), ("BirchTree", 0.3)),
        scatter="poisson",
        include_zones=("forest",),
        params=(("count", 30), ("min_dist", 2.0)),
    ))
    return layout


def test_solver_basic_placement_determinism():
    layout = _make_simple_layout()
    a = solve(layout, seed=42)
    b = solve(layout, seed=42)
    assert a.props == b.props


def test_solver_different_seed_gives_different_props():
    layout = _make_simple_layout()
    a = solve(layout, seed=1)
    b = solve(layout, seed=2)
    assert a.props != b.props


def test_solver_places_inside_zone():
    layout = _make_simple_layout()
    solved = solve(layout, seed=3)
    for p in solved.props:
        x, y, _ = p.translation
        assert 5 <= x <= 45
        assert 5 <= y <= 45


# -------------------------------------------------------------------
# Solver: exclusive zone priority
# -------------------------------------------------------------------


def test_exclusive_higher_priority_zone_clears_props():
    layout = Layout(world_size=(50.0, 50.0))
    layout.add_zone(Zone(
        name="forest",
        polygon=((0, 0), (50, 0), (50, 50), (0, 50)),
        priority=0,
    ))
    # A spawn clearing at the centre, with higher priority and
    # exclusive semantics.
    layout.add_zone(Zone(
        name="spawn_clearing",
        polygon=((20, 20), (30, 20), (30, 30), (20, 30)),
        priority=10,
        exclusive=True,
    ))
    layout.add_prop_group(PropGroup(
        name="trees",
        proto_weights=(("OakTree", 1.0),),
        scatter="poisson",
        include_zones=("forest",),
        params=(("count", 200), ("min_dist", 1.0)),
    ))
    solved = solve(layout, seed=0)
    assert solved.props, "forest should have produced some trees"
    for p in solved.props:
        x, y, _ = p.translation
        inside_clearing = 20 <= x <= 30 and 20 <= y <= 30
        assert not inside_clearing, f"tree inside exclusive clearing at {p.translation}"


def test_explicit_exclude_zone_clears_props():
    layout = Layout(world_size=(50.0, 50.0))
    layout.add_zone(Zone(
        name="forest",
        polygon=((0, 0), (50, 0), (50, 50), (0, 50)),
    ))
    layout.add_zone(Zone(
        name="pond",
        polygon=((15, 15), (25, 15), (25, 25), (15, 25)),
    ))
    layout.add_prop_group(PropGroup(
        name="trees",
        proto_weights=(("OakTree", 1.0),),
        scatter="poisson",
        include_zones=("forest",),
        exclude_zones=("pond",),
        params=(("count", 200), ("min_dist", 1.0)),
    ))
    solved = solve(layout, seed=0)
    for p in solved.props:
        x, y, _ = p.translation
        assert not (15 <= x <= 25 and 15 <= y <= 25), (
            f"tree in pond at {p.translation}"
        )


# -------------------------------------------------------------------
# Solver: path exclusion
# -------------------------------------------------------------------


def test_path_exclusion_pushes_props_off_road():
    layout = Layout(world_size=(50.0, 50.0))
    layout.add_zone(Zone(
        name="forest",
        polygon=((0, 0), (50, 0), (50, 50), (0, 50)),
    ))
    layout.add_path(Path(
        name="road",
        polyline=((0.0, 25.0), (50.0, 25.0)),
        width=4.0,
    ))
    layout.add_prop_group(PropGroup(
        name="trees",
        proto_weights=(("OakTree", 1.0),),
        scatter="poisson",
        include_zones=("forest",),
        exclude_paths=("road",),
        exclude_margin=0.5,
        params=(("count", 100), ("min_dist", 1.0)),
    ))
    solved = solve(layout, seed=0)
    for p in solved.props:
        _, y, _ = p.translation
        # width/2 + margin = 2.5 m clearance.
        assert abs(y - 25.0) >= 2.5, (
            f"tree at y={y} inside road corridor"
        )


# -------------------------------------------------------------------
# Solver: terrain stamp
# -------------------------------------------------------------------


def test_terrain_fbm_produces_heightmap():
    layout = Layout(
        world_size=(20.0, 20.0),
        terrain_params={
            "kind": "fbm",
            "grid_resolution": 32,
            "height_scale": 2.0,
            "octaves": 3,
        },
    )
    solved = solve(layout, seed=99)
    assert solved.heightmap is not None
    assert solved.heightmap.width == 32
    assert solved.heightmap.max() <= 2.0 + 1e-9
    assert solved.heightmap.min() >= 0.0 - 1e-9


def test_terrain_stamp_raises_elevation():
    layout = Layout(
        world_size=(20.0, 20.0),
        terrain_params={
            "kind": "fbm",
            "grid_resolution": 32,
            "height_scale": 1.0,
            "octaves": 3,
        },
    )
    layout.add_zone(Zone(
        name="hill",
        polygon=((5, 5), (15, 5), (15, 15), (5, 15)),
    ))
    layout.add_stamp(Stamp(
        name="hill_stamp", zone="hill", mode="add", height=3.0,
    ))
    solved = solve(layout, seed=7)
    # Max must have climbed above the base +1.0 range.
    assert solved.heightmap is not None
    assert solved.heightmap.max() > 1.5


# -------------------------------------------------------------------
# Schema: JSON round-trip
# -------------------------------------------------------------------


def _full_layout_for_round_trip() -> Layout:
    layout = Layout(
        world_size=(50.0, 50.0),
        terrain_params={
            "kind": "fbm", "grid_resolution": 16, "height_scale": 1.0,
        },
    )
    layout.add_zone(Zone(
        name="forest",
        polygon=((0, 0), (50, 0), (50, 50), (0, 50)),
        priority=0,
        tags=frozenset({"natural", "outdoor"}),
        params=(("density", 0.3),),
    ))
    layout.add_zone(Zone(
        name="clearing",
        polygon=((20, 20), (30, 20), (30, 30), (20, 30)),
        priority=5,
        exclusive=True,
    ))
    layout.add_path(Path(
        name="trail",
        polyline=((0.0, 25.0), (50.0, 25.0)),
        width=1.5,
        tags=frozenset({"footpath"}),
    ))
    layout.add_stamp(Stamp(name="bump", zone="forest", mode="add", height=0.5))
    layout.add_prop_group(PropGroup(
        name="trees",
        proto_weights=(("OakTree", 0.5), ("BirchTree", 0.5)),
        scatter="poisson",
        include_zones=("forest",),
        exclude_paths=("trail",),
        exclude_margin=0.2,
        params=(("count", 40), ("min_dist", 1.0)),
    ))
    layout.add_spawn(Spawn(
        name="robot",
        translation=(25.0, 25.0, 0.1),
        urdf_url="../robots/cube_bot.urdf",
        controller="omnibot_agent",
    ))
    return layout


def test_schema_round_trip_dict():
    layout = _full_layout_for_round_trip()
    payload = to_dict(layout)
    restored = from_dict(payload)
    assert to_dict(restored) == payload


def test_schema_round_trip_json():
    layout = _full_layout_for_round_trip()
    text = to_json(layout)
    restored = from_json(text)
    assert to_json(restored) == text


def test_schema_rejects_unknown_keys():
    layout = _full_layout_for_round_trip()
    payload = to_dict(layout)
    payload["zones"][0]["mystery_field"] = "oops"
    with pytest.raises(ValueError):
        from_dict(payload)


def test_schema_rejects_future_version():
    layout = _full_layout_for_round_trip()
    payload = to_dict(layout)
    payload["schema_version"] = 9999
    with pytest.raises(ValueError):
        from_dict(payload)


def test_schema_round_trip_then_solve_is_equivalent():
    """A recipe can dump a layout to disk and reload it without
    changing what the solver produces."""
    layout = _full_layout_for_round_trip()
    restored = from_json(to_json(layout))
    a = solve(layout, seed=12)
    b = solve(restored, seed=12)
    assert a.props == b.props
