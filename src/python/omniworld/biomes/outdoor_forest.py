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

"""``outdoor_forest`` — a reproducible forested outdoor biome.

This is the first real biome shipping under the Tier 1 scaffold. It
composes the pieces from T1.2 / T1.3 / T1.4: a fBm heightmap with a
radial flat-centre stamp, Poisson-disk scatter of trees constrained
to an annular ring, a smaller rock scatter, and an optional robot
spawn at the clearing.

Parameters follow the shape the plan calls out: ``size`` (metres), a
tree count, a rock count, min-dist rules, and a seed-stable per-biome
catalog of tree PROTOs. The catalog is hand-authored here (a minimal
T1.5 stand-in); once the full ``build_asset_catalog`` scanner lands
recipes will consume it through the catalog query API instead.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ..catalog import get_url, iter_tagged
from ..core.layout import Layout, PropGroup, Stamp, Zone
from ..core.recipe import PlacedProto, Spawn, WorldDescription
from ..core.registry import register_recipe
from ..core.solver import solve
from ..realism import (
    compute_grade,
    grade_to_color_multiplier,
    variation_for_instance,
)


# Tree species allowed for this recipe. Pulled from the shared
# catalog — adding a new species to ``catalog/assets.json`` with the
# right tags picks it up automatically on the next tree_weights
# refresh.
_ALLOWED_TREE_SPECIES: frozenset[str] = frozenset(
    a.name for a in iter_tagged("tree", "forest")
)


DEFAULT_PARAMS: dict[str, Any] = {
    # World edge length in metres. Square. Maps to (size, size).
    "size": 50.0,
    # Heightmap grid resolution (cells per side).
    "grid_resolution": 128,
    # Vertical scale applied to the unit-range fBm output.
    "height_scale": 2.0,
    # fBm knobs — defaults match the forest of the existing demos.
    "octaves": 5,
    "gain": 0.5,
    "lacunarity": 2.0,
    # Clearing radius at the centre (metres). Spawn zone is exclusive.
    "clearing_radius": 4.0,
    # Road width in metres. Set to 0 to omit the road.
    "road_width": 3.0,
    # Tree scatter knobs.
    "tree_count": 80,
    "tree_min_dist": 1.8,
    "tree_r_inner": 4.5,
    # Rock scatter knobs.
    "rock_count": 30,
    "rock_min_dist": 1.2,
    # Per-instance variation (T1.10). Scale bracket is applied to
    # every tree, rocks get their own wider bracket.
    "tree_scale_min": 0.9,
    "tree_scale_max": 1.15,
    "tree_hue_shift_deg": 6.0,
    "rock_scale_min": 0.6,
    "rock_scale_max": 1.4,
    # World-level weathering (T1.10). 0 pristine -> 1 abandoned. Drives
    # a color darkening on rock surfaces via the PROTO's color field.
    "world_age": 0.0,
    # Weighted tree species selection. Subset must be a key of
    # ``_TREE_CATALOG``. Unknown keys are rejected.
    "tree_weights": {"Oak": 0.45, "Pine": 0.30, "Sassafras": 0.15, "SimpleTree": 0.10},
    # Optional URDF to spawn at the clearing. None => no robot.
    "spawn_urdf": None,
    "spawn_controller": None,
    "spawn_height": 0.2,
}


def _square_polygon(size: float) -> tuple[tuple[float, float], ...]:
    return ((0.0, 0.0), (size, 0.0), (size, size), (0.0, size))


def _disk_polygon(
    centre: tuple[float, float], radius: float, segments: int = 24
) -> tuple[tuple[float, float], ...]:
    import math

    cx, cy = centre
    return tuple(
        (
            cx + radius * math.cos(2 * math.pi * i / segments),
            cy + radius * math.sin(2 * math.pi * i / segments),
        )
        for i in range(segments)
    )


def _validate_tree_weights(weights: Mapping[str, float]) -> None:
    for name in weights:
        if name not in _ALLOWED_TREE_SPECIES:
            raise ValueError(
                f"outdoor_forest: unknown tree species {name!r}; "
                f"known: {sorted(_ALLOWED_TREE_SPECIES)}"
            )
    if any(w < 0.0 for w in weights.values()):
        raise ValueError("outdoor_forest: tree_weights must be non-negative")
    if sum(weights.values()) <= 0.0:
        raise ValueError("outdoor_forest: tree_weights sum to zero")


def _build_layout(params: Mapping[str, Any]) -> Layout:
    size = float(params["size"])
    centre = (size / 2.0, size / 2.0)

    layout = Layout(
        world_size=(size, size),
        terrain_params={
            "kind": "fbm",
            "grid_resolution": int(params["grid_resolution"]),
            "height_scale": float(params["height_scale"]),
            "octaves": int(params["octaves"]),
            "gain": float(params["gain"]),
            "lacunarity": float(params["lacunarity"]),
        },
    )

    # Forest covers the whole lot.
    layout.add_zone(Zone(
        name="forest",
        polygon=_square_polygon(size),
        priority=0,
        tags=frozenset({"natural", "scatter_host"}),
    ))

    # Spawn clearing — exclusive, high priority. Acts as an implicit
    # excluder for the tree prop group because it out-ranks ``forest``.
    clearing_radius = float(params["clearing_radius"])
    layout.add_zone(Zone(
        name="spawn_clearing",
        polygon=_disk_polygon(centre, clearing_radius, segments=24),
        priority=10,
        exclusive=True,
        tags=frozenset({"spawn"}),
    ))

    # A level pad under the clearing — stamp levels the terrain to a
    # fixed elevation so the robot spawns on solid ground.
    layout.add_stamp(Stamp(
        name="clearing_pad",
        zone="spawn_clearing",
        mode="replace",
        height=float(params["height_scale"]) * 0.5,
    ))

    # Optional road through the middle.
    road_width = float(params["road_width"])
    exclude_paths: tuple[str, ...] = ()
    if road_width > 0.0:
        from ..core.layout import Path

        layout.add_path(Path(
            name="road",
            polyline=((0.0, centre[1]), (size, centre[1])),
            width=road_width,
            tags=frozenset({"road"}),
        ))
        exclude_paths = ("road",)

    # Trees: Poisson scatter inside the forest zone, excluding the
    # road corridor and (implicitly) the exclusive clearing.
    weights = dict(params["tree_weights"])
    _validate_tree_weights(weights)
    tree_proto_weights = tuple((name, weight) for name, weight in weights.items())
    layout.add_prop_group(PropGroup(
        name="trees",
        proto_weights=tree_proto_weights,
        scatter="poisson",
        include_zones=("forest",),
        exclude_paths=exclude_paths,
        exclude_margin=0.5,
        params=(
            ("count", int(params["tree_count"])),
            ("min_dist", float(params["tree_min_dist"])),
        ),
    ))

    # Rocks: sparse Poisson on the same forest region.
    layout.add_prop_group(PropGroup(
        name="rocks",
        proto_weights=(("Rock", 1.0),),
        scatter="poisson",
        include_zones=("forest",),
        exclude_paths=exclude_paths,
        exclude_margin=0.3,
        params=(
            ("count", int(params["rock_count"])),
            ("min_dist", float(params["rock_min_dist"])),
        ),
    ))

    # Robot spawn at the clearing.
    if params["spawn_urdf"] is not None:
        layout.add_spawn(Spawn(
            name="robot",
            translation=(centre[0], centre[1], float(params["spawn_height"])),
            rotation=(0.0, 0.0, 1.0, 0.0),
            urdf_url=str(params["spawn_urdf"]),
            controller=(
                str(params["spawn_controller"])
                if params["spawn_controller"] is not None
                else None
            ),
        ))

    return layout


def _proto_url(name: str) -> str:
    return get_url(name)


@dataclass
class OutdoorForest:
    name: str = "outdoor_forest"

    def default_params(self) -> dict[str, Any]:
        return {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULT_PARAMS.items()}

    def build(self, seed: int, params: Mapping[str, Any]) -> WorldDescription:
        merged = self.default_params()
        for key, value in params.items():
            if key not in merged:
                raise ValueError(
                    f"unknown param for recipe {self.name!r}: {key!r}; "
                    f"known: {sorted(merged)}"
                )
            merged[key] = value

        layout = _build_layout(merged)
        solved = solve(layout, seed=seed)

        # Ship the heightmap + props through the WorldDescription so
        # the emitter can produce a valid ElevationGrid world.
        size = float(merged["size"])
        hm = solved.heightmap
        assert hm is not None, "outdoor_forest always produces a heightmap"
        spacing = (size / hm.width, size / hm.height)

        world_age = float(merged["world_age"])
        if not (0.0 <= world_age <= 1.0):
            raise ValueError("outdoor_forest: world_age must be in [0, 1]")
        # Base rock colour (matches the PROTO default) so hue shifts are
        # anchored to something meaningful.
        base_rock_color = (0.5, 0.5, 0.5)

        # Group placements by index within their group so per-instance
        # variation is a pure function of (seed, group, index).
        per_group_counters: dict[str, int] = {}

        props: list[PlacedProto] = []
        for placement in solved.props:
            idx = per_group_counters.get(placement.group, 0)
            per_group_counters[placement.group] = idx + 1

            if placement.group == "trees":
                variation = variation_for_instance(
                    seed, placement.group, idx,
                    scale_min=float(merged["tree_scale_min"]),
                    scale_max=float(merged["tree_scale_max"]),
                    hue_shift_deg=float(merged["tree_hue_shift_deg"]),
                )
                props.append(PlacedProto(
                    proto_url=_proto_url(placement.proto),
                    proto_type=placement.proto,
                    translation=placement.translation,
                    rotation=placement.rotation,
                    # Tree PROTOs have no direct color field, so hue is
                    # dropped on the floor today. Scale does apply via
                    # the per-PROTO ``scale`` field introduced by the
                    # individual tree PROTOs in later catalog entries.
                    # For now we only forward scale to PROTOs known to
                    # accept it; trees don't, so we emit nothing here.
                    extra_fields=(),
                ))
            elif placement.group == "rocks":
                variation = variation_for_instance(
                    seed, placement.group, idx,
                    scale_min=float(merged["rock_scale_min"]),
                    scale_max=float(merged["rock_scale_max"]),
                )
                grade = compute_grade(
                    seed, placement.group, idx,
                    world_age=world_age,
                    exposure=0.8,     # rocks are outdoor, exposed
                    moisture_base=0.1,
                )
                from ..realism.variation import jitter_hue_rgb
                shifted = jitter_hue_rgb(base_rock_color, variation.hue_shift_deg)
                mult = grade_to_color_multiplier(grade)
                color = tuple(c * mult for c in shifted)
                props.append(PlacedProto(
                    proto_url=_proto_url(placement.proto),
                    proto_type=placement.proto,
                    translation=placement.translation,
                    rotation=placement.rotation,
                    extra_fields=(
                        ("scale", format(variation.scale, ".4g")),
                        ("color", " ".join(format(c, ".4g") for c in color)),
                    ),
                ))
            else:
                props.append(PlacedProto(
                    proto_url=_proto_url(placement.proto),
                    proto_type=placement.proto,
                    translation=placement.translation,
                    rotation=placement.rotation,
                ))

        return WorldDescription(
            title="outdoor_forest (omniworld)",
            heightmap=hm,
            heightmap_spacing=spacing,
            heightmap_origin=(0.0, 0.0),
            basic_time_step_ms=int(DEFAULT_PARAMS.get("basic_time_step_ms", 16)),
            props=props,
            spawns=list(solved.spawns),
            metadata={"recipe": self.name, "prop_count": len(props)},
        )


register_recipe(OutdoorForest())
