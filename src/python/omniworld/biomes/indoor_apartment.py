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

"""``indoor_apartment`` — the first interior biome.

Three rooms arranged in a strip: living room, bedroom, bathroom.
Perimeter walls plus two inner dividers with door openings. One
furniture prop per room — Table, Bed, Toilet. Flat floor,
``RectangleArena`` emission.

Tier 1 scope is deliberately narrow: one fixed three-room layout with
seed-driven randomisation of room sizes, door placement, and
furniture rotation. The WFC-driven multi-room layouts the plan calls
out for T2.3 land later.

Every wall segment is a ``Wall`` PROTO from
``projects/objects/apartment_structure/``. Walls are hand-computed
from the room rectangles — a future refactor may fold this into a
dedicated primitive, but at three rooms the bespoke math is clearer
than inventing an abstraction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

from ..catalog import get_url
from ..core.layout import Layout, Zone
from ..core.recipe import PlacedProto, Spawn, WorldDescription
from ..core.registry import register_recipe
from ..core.rng import derive_seed, rng_from_seed
from ..core.solver import solve


DEFAULT_PARAMS: dict[str, Any] = {
    # Apartment extent. The interior is placed inside a floor_size of
    # (apartment_length + 4, apartment_depth + 4).
    "apartment_length": 12.0,
    "apartment_depth": 4.0,
    "wall_thickness": 0.2,
    "wall_height": 2.4,
    "door_width": 1.0,
    # Relative widths of the three rooms (normalised internally).
    "living_room_width": 5.0,
    "bedroom_width": 4.0,
    "bathroom_width": 3.0,
    "spawn_urdf": None,
    "spawn_controller": None,
    "spawn_height": 0.2,
}


def _wall_segment(
    x0: float, y0: float, x1: float, y1: float,
    thickness: float, height: float,
    name: str,
) -> PlacedProto:
    """Emit one Wall PROTO spanning from ``(x0, y0)`` to ``(x1, y1)``.

    The Wall PROTO's ``size`` is (thickness, length, height). We place
    it with local +y pointing along the wall, then rotate around Z so
    the +y axis aligns with the segment direction.
    """
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    if length <= 0.0:
        raise ValueError("wall segment has zero length")
    cx = (x0 + x1) / 2.0
    cy = (y0 + y1) / 2.0
    # Angle from +y axis to segment direction.
    theta = math.atan2(dx, dy) if dy >= 0.0 else math.atan2(-dx, -dy)
    # Easier: atan2(dy, dx) gives angle from +x; we want angle from +y,
    # which is (pi/2 - that).
    yaw = math.atan2(dy, dx) - math.pi / 2.0
    size_field = f"{thickness:.4g} {length:.4g} {height:.4g}"
    return PlacedProto(
        proto_url=get_url("Wall"),
        proto_type="Wall",
        translation=(cx, cy, 0.0),
        rotation=(0.0, 0.0, 1.0, yaw),
        extra_fields=(
            ("size", size_field),
            ("name", f"\"{name}\""),
        ),
    )


def _split_wall_around_door(
    axis: str, along_fixed: float, start: float, end: float,
    door_centre: float, door_half: float,
    thickness: float, height: float, base_name: str,
) -> list[PlacedProto]:
    """Emit a wall along ``axis`` with a door-sized gap centred at
    ``door_centre``. ``axis='y'`` means the wall runs in +y with fixed
    x = along_fixed; ``axis='x'`` is the transpose.
    """
    segs: list[PlacedProto] = []
    lo_end = door_centre - door_half
    hi_start = door_centre + door_half
    if lo_end > start:
        if axis == "y":
            segs.append(_wall_segment(
                along_fixed, start, along_fixed, lo_end,
                thickness, height, f"{base_name}_lo",
            ))
        else:
            segs.append(_wall_segment(
                start, along_fixed, lo_end, along_fixed,
                thickness, height, f"{base_name}_lo",
            ))
    if hi_start < end:
        if axis == "y":
            segs.append(_wall_segment(
                along_fixed, hi_start, along_fixed, end,
                thickness, height, f"{base_name}_hi",
            ))
        else:
            segs.append(_wall_segment(
                start, along_fixed, end, along_fixed,
                thickness, height, f"{base_name}_hi",
            ))
    return segs


def _room_centre(x0, x1, y0, y1):
    return (x0 + x1) / 2.0, (y0 + y1) / 2.0


def _rect(x0, y0, x1, y1):
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))


def _build_layout(params: Mapping[str, Any]) -> tuple[Layout, list[tuple[str, float, float, float, float]]]:
    """Return (layout, [(room_name, x0, y0, x1, y1) per room])."""
    length = float(params["apartment_length"])
    depth = float(params["apartment_depth"])
    widths = [
        float(params["living_room_width"]),
        float(params["bedroom_width"]),
        float(params["bathroom_width"]),
    ]
    total = sum(widths)
    # Normalise widths to apartment_length.
    widths = [w * length / total for w in widths]

    xs = [0.0]
    for w in widths:
        xs.append(xs[-1] + w)

    rooms = [
        ("living_room", xs[0], 0.0, xs[1], depth),
        ("bedroom",     xs[1], 0.0, xs[2], depth),
        ("bathroom",    xs[2], 0.0, xs[3], depth),
    ]
    # Arena is slightly larger than the apartment so walls are visible.
    margin = 2.0
    layout = Layout(world_size=(length + 2 * margin, depth + 2 * margin))
    # Offset the apartment from the arena corner by the margin — store
    # the offset in terrain_params so the emitter / recipe can apply.
    layout.terrain_params = {"apartment_origin": [margin, margin]}

    for name, x0, y0, x1, y1 in rooms:
        layout.add_zone(Zone(
            name=name,
            polygon=_rect(x0 + margin, y0 + margin, x1 + margin, y1 + margin),
            tags=frozenset({"indoor", name}),
        ))

    # The apartment origin offsets everything; we return rooms in
    # arena-space coords so the recipe does not double-shift.
    rooms_arena = [
        (name, x0 + margin, y0 + margin, x1 + margin, y1 + margin)
        for name, x0, y0, x1, y1 in rooms
    ]
    return layout, rooms_arena


def _place_walls_and_furniture(
    params: Mapping[str, Any],
    rooms: list[tuple[str, float, float, float, float]],
    seed: int,
) -> list[PlacedProto]:
    thickness = float(params["wall_thickness"])
    height = float(params["wall_height"])
    door_half = float(params["door_width"]) / 2.0

    rng = rng_from_seed(derive_seed(seed, "indoor_apartment.furniture"))

    # Bounding box of the three rooms.
    xs0 = min(r[1] for r in rooms)
    ys0 = min(r[2] for r in rooms)
    xs1 = max(r[3] for r in rooms)
    ys1 = max(r[4] for r in rooms)

    walls: list[PlacedProto] = []
    # Perimeter walls.
    walls.append(_wall_segment(xs0, ys0, xs1, ys0, thickness, height, "perim_south"))
    walls.append(_wall_segment(xs0, ys1, xs1, ys1, thickness, height, "perim_north"))
    walls.append(_wall_segment(xs0, ys0, xs0, ys1, thickness, height, "perim_west"))
    walls.append(_wall_segment(xs1, ys0, xs1, ys1, thickness, height, "perim_east"))

    # Inner dividers with door openings. Rooms are sorted left-to-right.
    rooms_sorted = sorted(rooms, key=lambda r: r[1])
    depth = ys1 - ys0
    for i in range(len(rooms_sorted) - 1):
        shared_x = rooms_sorted[i][3]  # right edge of room i == left edge of room i+1
        door_centre = ys0 + depth / 2.0
        walls.extend(_split_wall_around_door(
            axis="y",
            along_fixed=shared_x,
            start=ys0,
            end=ys1,
            door_centre=door_centre,
            door_half=door_half,
            thickness=thickness,
            height=height,
            base_name=f"divider_{i}",
        ))

    # Furniture: one piece per room.
    furniture_map = {
        "living_room": (get_url("Table"), "Table"),
        "bedroom":     (get_url("Bed"), "Bed"),
        "bathroom":    (get_url("Toilet"), "Toilet"),
    }
    furniture: list[PlacedProto] = []
    for name, x0, y0, x1, y1 in rooms:
        url, proto_type = furniture_map[name]
        cx, cy = _room_centre(x0, x1, y0, y1)
        yaw = rng.choice((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0))
        furniture.append(PlacedProto(
            proto_url=url,
            proto_type=proto_type,
            translation=(cx, cy, 0.0),
            rotation=(0.0, 0.0, 1.0, yaw),
        ))

    return walls + furniture


@dataclass
class IndoorApartment:
    name: str = "indoor_apartment"

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

        layout, rooms = _build_layout(merged)

        # We don't actually need the solver for this recipe (no stochastic
        # scatter), but we still route through it so the Layout is
        # validated and any future scatter groups would work.
        solve(layout, seed=seed)

        props = _place_walls_and_furniture(merged, rooms, seed)

        spawns: list[Spawn] = []
        if merged["spawn_urdf"] is not None:
            # Spawn in the living room (first room).
            _, x0, y0, x1, y1 = rooms[0]
            cx, cy = _room_centre(x0, x1, y0, y1)
            spawns.append(Spawn(
                name="robot",
                translation=(cx, cy, float(merged["spawn_height"])),
                rotation=(0.0, 0.0, 1.0, 0.0),
                urdf_url=str(merged["spawn_urdf"]),
                controller=(
                    str(merged["spawn_controller"])
                    if merged["spawn_controller"] is not None
                    else None
                ),
            ))

        world_w, world_d = layout.world_size
        return WorldDescription(
            title="indoor_apartment (omniworld)",
            floor_size=(world_w, world_d),
            props=props,
            spawns=spawns,
            metadata={
                "recipe": self.name,
                "room_count": len(rooms),
                "wall_count": sum(1 for p in props if p.proto_type == "Wall"),
                "furniture_count": sum(1 for p in props if p.proto_type != "Wall"),
            },
        )


register_recipe(IndoorApartment())
