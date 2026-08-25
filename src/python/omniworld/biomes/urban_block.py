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

"""``urban_block`` — a 2x2 city block with buildings, road grid, and
street furniture.

Flat floor, a central cross road dividing the world into four lots,
one configurable ``Building`` PROTO per lot with randomised floor
count and footprint, street lights along the roads, benches along
sidewalks. Spawn at the central intersection.

Four buildings is not a city. It is the smallest configuration that
exercises: multi-lot placement, a Path primitive with multiple
segments, the ``along_path`` scatter for street furniture, and
Building PROTO parameterisation. Larger urban biomes compose by
tiling this block — a job for T2 real-world-import once OSM / DEM
ingress lands.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ..catalog import get_url
from ..core.layout import Layout, Path as LPath, PropGroup, Zone
from ..core.recipe import PlacedProto, Spawn, WorldDescription
from ..core.registry import register_recipe
from ..core.rng import derive, derive_seed, rng_from_seed
from ..core.solver import solve


DEFAULT_PARAMS: dict[str, Any] = {
    "size": 80.0,
    "road_width": 8.0,
    # Per-building randomisation bounds.
    "min_floors": 2,
    "max_floors": 5,
    "floor_height": 3.0,
    "min_footprint": 12.0,
    "max_footprint": 22.0,
    # Street furniture density (items per metre of road).
    "streetlight_density": 0.08,
    "streetlight_offset": 4.5,
    "bench_density": 0.025,
    "bench_offset": 3.0,
    # Optional URDF spawn at the central intersection.
    "spawn_urdf": None,
    "spawn_controller": None,
    "spawn_height": 0.2,
}


def _rect(x0, y0, x1, y1):
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))


def _lot_rects(size: float, road_half: float):
    """Return the four building-lot rectangles."""
    margin = 2.0  # sidewalk breathing room
    return {
        "lot_sw": _rect(
            margin, margin, size / 2.0 - road_half - margin, size / 2.0 - road_half - margin
        ),
        "lot_se": _rect(
            size / 2.0 + road_half + margin, margin,
            size - margin, size / 2.0 - road_half - margin
        ),
        "lot_nw": _rect(
            margin, size / 2.0 + road_half + margin,
            size / 2.0 - road_half - margin, size - margin
        ),
        "lot_ne": _rect(
            size / 2.0 + road_half + margin, size / 2.0 + road_half + margin,
            size - margin, size - margin
        ),
    }


def _building_corners(footprint: float) -> str:
    """Return the MFVec2f ``corners`` field value for a square Building."""
    h = footprint / 2.0
    # Building PROTO expects 2D corners in local coords, CCW from bottom-left.
    return f"[{h} {h}, {h} {-h}, {-h} {-h}, {-h} {h}]"


def _build_layout(params: Mapping[str, Any]) -> Layout:
    size = float(params["size"])
    road_half = float(params["road_width"]) / 2.0

    layout = Layout(world_size=(size, size))
    layout.add_zone(Zone(name="plot", polygon=_rect(0.0, 0.0, size, size)))

    # Central cross road (H + V).
    layout.add_path(LPath(
        name="road_h",
        polyline=((0.0, size / 2.0), (size, size / 2.0)),
        width=float(params["road_width"]),
        tags=frozenset({"road"}),
    ))
    layout.add_path(LPath(
        name="road_v",
        polyline=((size / 2.0, 0.0), (size / 2.0, size)),
        width=float(params["road_width"]),
        tags=frozenset({"road"}),
    ))

    # Intersection exclusive clearing so the spawn is not under a
    # street-light pole.
    xy_c = size / 2.0
    layout.add_zone(Zone(
        name="intersection",
        polygon=_rect(xy_c - road_half, xy_c - road_half, xy_c + road_half, xy_c + road_half),
        priority=10,
        exclusive=True,
    ))

    # Four lot zones (for documentation / future scatter; the building
    # placement is hand-done so no prop group references them).
    for name, poly in _lot_rects(size, road_half).items():
        layout.add_zone(Zone(name=name, polygon=poly, priority=5, exclusive=True))

    # Street lights + benches placed by the solver, along the roads,
    # offset onto the sidewalk.
    layout.add_prop_group(PropGroup(
        name="streetlights",
        proto_weights=(("ControlledStreetLight", 1.0),),
        scatter="along_path",
        along_path_ref="road_h",
        exclude_zones=("intersection",),
        params=(
            ("density", float(params["streetlight_density"])),
            ("offset", float(params["streetlight_offset"])),
            ("side", "both"),
        ),
    ))
    layout.add_prop_group(PropGroup(
        name="streetlights_v",
        proto_weights=(("ControlledStreetLight", 1.0),),
        scatter="along_path",
        along_path_ref="road_v",
        exclude_zones=("intersection",),
        params=(
            ("density", float(params["streetlight_density"])),
            ("offset", float(params["streetlight_offset"])),
            ("side", "both"),
        ),
    ))
    layout.add_prop_group(PropGroup(
        name="benches",
        proto_weights=(("Bench", 1.0),),
        scatter="along_path",
        along_path_ref="road_h",
        exclude_zones=("intersection",),
        params=(
            ("density", float(params["bench_density"])),
            ("offset", float(params["bench_offset"])),
            ("side", "both"),
        ),
    ))

    if params["spawn_urdf"] is not None:
        layout.add_spawn(Spawn(
            name="robot",
            translation=(xy_c, xy_c, float(params["spawn_height"])),
            rotation=(0.0, 0.0, 1.0, 0.0),
            urdf_url=str(params["spawn_urdf"]),
            controller=(
                str(params["spawn_controller"])
                if params["spawn_controller"] is not None
                else None
            ),
        ))
    return layout


def _prop_url(proto_type: str) -> str:
    return get_url(proto_type)


def _place_buildings(params: Mapping[str, Any], seed: int) -> list[PlacedProto]:
    """Hand-placed Buildings, one per lot."""
    size = float(params["size"])
    road_half = float(params["road_width"]) / 2.0
    lots = _lot_rects(size, road_half)

    rng = rng_from_seed(derive_seed(seed, "urban_block.buildings"))
    min_f = int(params["min_floors"])
    max_f = int(params["max_floors"])
    min_fp = float(params["min_footprint"])
    max_fp = float(params["max_footprint"])

    out: list[PlacedProto] = []
    for lot_name, poly in lots.items():
        x0, y0 = poly[0]
        x1, y1 = poly[2]
        lot_w = x1 - x0
        lot_d = y1 - y0
        footprint = min(
            min_fp + (max_fp - min_fp) * rng.random(),
            lot_w - 1.0, lot_d - 1.0,
        )
        cx = (x0 + x1) / 2.0
        cy = (y0 + y1) / 2.0
        floors = rng.randint(min_f, max_f)
        yaw = rng.choice((0.0, 1.5707963267948966, 3.141592653589793, 4.71238898038469))
        out.append(PlacedProto(
            proto_url=get_url("ConstructionFrameBuilding"),
            proto_type="ConstructionFrameBuilding",
            translation=(cx, cy, 0.0),
            rotation=(0.0, 0.0, 1.0, yaw),
            extra_fields=(
                ("floors", str(floors)),
                ("floorHeight", format(float(params["floor_height"]), ".4g")),
                ("baysX", "3"),
                ("baysY", "3"),
                ("bay", format(footprint / 3.0, ".4g")),
                ("wrapFloors", str(max(0, floors - 2))),
                ("name", f"\"building_{lot_name}\""),
            ),
        ))
    return out


@dataclass
class UrbanBlock:
    name: str = "urban_block"

    def default_params(self) -> dict[str, Any]:
        return dict(DEFAULT_PARAMS)

    def build(self, seed: int, params: Mapping[str, Any]) -> WorldDescription:
        merged = dict(DEFAULT_PARAMS)
        for k, v in params.items():
            if k not in merged:
                raise ValueError(
                    f"unknown param for recipe {self.name!r}: {k!r}; "
                    f"known: {sorted(merged)}"
                )
            merged[k] = v

        layout = _build_layout(merged)
        solved = solve(layout, seed=seed)

        props: list[PlacedProto] = []
        props.extend(_place_buildings(merged, seed))
        for placement in solved.props:
            # ``ControlledStreetLight`` defaults its controller to
            # ``"defective_street_light"``, a demo controller that
            # immediately exits with status 1. Override to ``"<none>"``
            # so a generated urban world is quiet under headless runs.
            extra_fields: tuple[tuple[str, str], ...] = ()
            if placement.proto == "ControlledStreetLight":
                extra_fields = (("controller", "\"<none>\""),)
            props.append(PlacedProto(
                proto_url=_prop_url(placement.proto),
                proto_type=placement.proto,
                translation=placement.translation,
                rotation=placement.rotation,
                extra_fields=extra_fields,
            ))

        return WorldDescription(
            title="urban_block (omniworld)",
            floor_size=(float(merged["size"]), float(merged["size"])),
            props=props,
            spawns=list(solved.spawns),
            metadata={
                "recipe": self.name,
                "building_count": 4,
                "street_furniture_count": len(solved.props),
            },
        )


register_recipe(UrbanBlock())
