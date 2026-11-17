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

"""``warehouse`` — axis-aligned warehouse biome.

Flat floor, N rows of pallet-stack "racks" running east-west with
aisles between them, sparse cardboard-box scatter in the aisles, and a
loading-dock clearing at one end where the robot spawns.

The racks are computed algorithmically (axis-aligned grid) so the
solver's stochastic machinery only handles box scatter. Everything
deterministic-but-mechanical goes through direct ``PlacedProto``
emission; everything stochastic goes through the Layout+solver path.
That split keeps the racks perfectly aligned — warehouses look wrong
if racks are jittered — while boxes retain the organic "someone just
unloaded a truck" feel.

Indoor: no terrain. The emitter uses ``RectangleArena`` with the
warehouse's full footprint.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ..catalog import get_url
from ..core.layout import Layout, PropGroup, Zone
from ..core.recipe import PlacedProto, Spawn, WorldDescription
from ..core.registry import register_recipe
from ..core.rng import derive_seed, rng_from_seed
from ..core.solver import solve
from ..primitives.nested_scatter import scatter_on_surface


DEFAULT_PARAMS: dict[str, Any] = {
    # Arena size (metres): x is the long axis, y is the short axis.
    "size_x": 40.0,
    "size_y": 24.0,
    # Rack rows. Each row is a line of ``rack_count`` pallet stacks
    # spaced ``rack_pitch`` metres apart. Rows are spaced ``aisle_width``
    # apart, starting ``aisle_width`` from the south wall.
    "rack_rows": 4,
    "rack_count": 12,
    "rack_pitch": 2.4,
    "aisle_width": 3.0,
    "rack_pallets_per_stack": 6,
    # Loading-dock clearing at the -x end.
    "dock_length": 6.0,
    # Boxes scattered in aisles.
    "box_count": 40,
    "box_min_dist": 0.9,
    # Nested-scatter clutter on top of rack stacks (T1.11).
    "rack_top_clutter_fraction": 0.6,   # fraction of racks that get boxes on top
    "rack_top_box_count": 2,             # boxes per cluttered rack
    "rack_top_box_min_dist": 0.35,
    # Optional URDF spawn.
    "spawn_urdf": None,
    "spawn_controller": None,
    "spawn_height": 0.2,
}


def _rack_positions(
    size_x: float,
    size_y: float,
    rack_rows: int,
    rack_count: int,
    rack_pitch: float,
    aisle_width: float,
    dock_length: float,
) -> list[tuple[float, float]]:
    """Compute rack stack centres in world coordinates.

    Racks occupy the east side of the arena; the west ``dock_length``
    metres are reserved for the loading dock.
    """
    positions: list[tuple[float, float]] = []
    # Total vertical span consumed by racks: rows * aisle_width + (rows - 1) * gap.
    # Simpler: lay down racks centred in y, with aisles between.
    total_rows_height = rack_rows * 1.2 + (rack_rows + 1) * aisle_width
    start_y = (size_y - total_rows_height) / 2.0 + aisle_width + 0.6
    start_x = dock_length + aisle_width
    usable_x = size_x - start_x - aisle_width
    if rack_count > 1:
        effective_pitch = min(rack_pitch, usable_x / (rack_count - 1))
    else:
        effective_pitch = 0.0
    for row in range(rack_rows):
        y = start_y + row * (1.2 + aisle_width)
        for col in range(rack_count):
            x = start_x + col * effective_pitch
            positions.append((x, y))
    return positions


def _rect(x0: float, y0: float, x1: float, y1: float):
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))


def _rack_footprints(positions, fx: float = 1.2, fy: float = 1.0) -> list:
    """Return per-rack exclusion rectangles so boxes do not spawn on top."""
    rects = []
    hx, hy = fx * 0.5, fy * 0.5
    for i, (x, y) in enumerate(positions):
        rects.append((f"rack_footprint_{i}", _rect(x - hx, y - hy, x + hx, y + hy)))
    return rects


def _build_layout(
    params: Mapping[str, Any], rack_positions: list[tuple[float, float]]
) -> Layout:
    size_x = float(params["size_x"])
    size_y = float(params["size_y"])

    layout = Layout(world_size=(size_x, size_y))

    # Floor fills the whole arena.
    layout.add_zone(Zone(
        name="floor",
        polygon=_rect(0.0, 0.0, size_x, size_y),
        tags=frozenset({"indoor"}),
    ))

    # Loading dock: the west strip.
    dock_length = float(params["dock_length"])
    layout.add_zone(Zone(
        name="loading_dock",
        polygon=_rect(0.0, 0.0, dock_length, size_y),
        priority=5,
        exclusive=True,
        tags=frozenset({"dock", "spawn"}),
    ))

    # Rack footprints as explicit exclude zones for the box scatter.
    rack_footprints = _rack_footprints(rack_positions)
    for name, poly in rack_footprints:
        layout.add_zone(Zone(name=name, polygon=poly, priority=3, exclusive=True))

    # Boxes: Poisson in the floor, excluding dock + racks (exclusive
    # priority takes care of that implicitly).
    layout.add_prop_group(PropGroup(
        name="boxes",
        proto_weights=(
            ("CardboardBox", 0.6),
            ("PlasticCrate", 0.25),
            ("WoodenBox", 0.15),
        ),
        scatter="poisson",
        include_zones=("floor",),
        params=(
            ("count", int(params["box_count"])),
            ("min_dist", float(params["box_min_dist"])),
        ),
    ))

    if params["spawn_urdf"] is not None:
        layout.add_spawn(Spawn(
            name="robot",
            translation=(dock_length / 2.0, size_y / 2.0, float(params["spawn_height"])),
            rotation=(0.0, 0.0, 1.0, 0.0),
            urdf_url=str(params["spawn_urdf"]),
            controller=(
                str(params["spawn_controller"])
                if params["spawn_controller"] is not None
                else None
            ),
        ))
    return layout


def _box_url(proto_type: str) -> str:
    return get_url(proto_type)


@dataclass
class Warehouse:
    name: str = "warehouse"

    def default_params(self) -> dict[str, Any]:
        return dict(DEFAULT_PARAMS)

    def build(self, seed: int, params: Mapping[str, Any]) -> WorldDescription:
        merged = dict(DEFAULT_PARAMS)
        for key, value in params.items():
            if key not in merged:
                raise ValueError(
                    f"unknown param for recipe {self.name!r}: {key!r}; "
                    f"known: {sorted(merged)}"
                )
            merged[key] = value

        rack_positions = _rack_positions(
            float(merged["size_x"]),
            float(merged["size_y"]),
            int(merged["rack_rows"]),
            int(merged["rack_count"]),
            float(merged["rack_pitch"]),
            float(merged["aisle_width"]),
            float(merged["dock_length"]),
        )

        layout = _build_layout(merged, rack_positions)
        solved = solve(layout, seed=seed)

        # Hand-place racks. Per-stack rotation jitter tiny (+/- 2 deg)
        # so the warehouse looks lived-in without breaking the grid.
        rack_rng = rng_from_seed(derive_seed(seed, "warehouse.rack_rotation"))
        import math
        rack_pallet_count = int(merged["rack_pallets_per_stack"])

        clutter_rng = rng_from_seed(derive_seed(seed, "warehouse.rack_top_clutter"))
        clutter_fraction = float(merged["rack_top_clutter_fraction"])
        top_box_count = int(merged["rack_top_box_count"])
        top_box_min_dist = float(merged["rack_top_box_min_dist"])
        top_box_choices: list[tuple[str, float]] = [
            ("CardboardBox", 0.7),
            ("PlasticCrate", 0.3),
        ]

        props: list[PlacedProto] = []
        rack_placements: list[PlacedProto] = []
        for (x, y) in rack_positions:
            yaw = (rack_rng.random() * 2.0 - 1.0) * (2.0 * math.pi / 180.0)
            rack = PlacedProto(
                proto_url=get_url("WoodenPalletStack"),
                proto_type="WoodenPalletStack",
                translation=(x, y, 0.0),
                rotation=(0.0, 0.0, 1.0, yaw),
                extra_fields=(
                    ("palletNumber", str(rack_pallet_count)),
                ),
            )
            props.append(rack)
            rack_placements.append(rack)

        # T1.11: nested-scatter clutter on top of a subset of racks.
        if clutter_fraction > 0.0 and top_box_count > 0:
            for i, rack in enumerate(rack_placements):
                if clutter_rng.random() >= clutter_fraction:
                    continue
                children = scatter_on_surface(
                    clutter_rng,
                    rack,
                    "top",
                    count=top_box_count,
                    min_dist=top_box_min_dist,
                    children=top_box_choices,
                )
                props.extend(children)

        # Boxes from the solver. Per-box yaw already in placement.rotation.
        for placement in solved.props:
            props.append(PlacedProto(
                proto_url=_box_url(placement.proto),
                proto_type=placement.proto,
                translation=placement.translation,
                rotation=placement.rotation,
            ))

        return WorldDescription(
            title="warehouse (omniworld)",
            floor_size=(float(merged["size_x"]), float(merged["size_y"])),
            props=props,
            spawns=list(solved.spawns),
            metadata={
                "recipe": self.name,
                "rack_count": len(rack_positions),
                "box_count": len(solved.props),
            },
        )


register_recipe(Warehouse())
