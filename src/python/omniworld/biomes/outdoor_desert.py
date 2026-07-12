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

"""``outdoor_desert`` — the second outdoor biome.

Same scaffolding as ``outdoor_forest`` but with a ridged / fbm blended
heightmap (mesa-and-roll feel), zero trees, and sparse Rock scatter
with a sandy tint. The desert is a compositional rerun rather than a
new subsystem — which is exactly what the plan asks of the primitives
layer, so the biome is small on purpose.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ..catalog import get_url
from ..core.layout import Layout, Stamp, Zone
from ..core.layout import PropGroup as _PropGroup
from ..core.recipe import PlacedProto, Spawn, WorldDescription
from ..core.registry import register_recipe
from ..core.solver import solve


DEFAULT_PARAMS: dict[str, Any] = {
    "size": 50.0,
    "grid_resolution": 128,
    "height_scale": 3.0,
    "octaves": 5,
    "gain": 0.5,
    "lacunarity": 2.0,
    # Blend of ridged peaks over fbm base. 0 = pure rolling dunes,
    # 1 = sharp mesa ridges.
    "mesa_mix": 0.55,
    # Spawn clearing at the centre — desert scenes typically care about
    # a flat staging area for the robot.
    "clearing_radius": 5.0,
    # Rock scatter.
    "rock_count": 40,
    "rock_min_dist": 1.8,
    # Rock tint ("sandy"), passed through to the PROTO's ``color`` field.
    "rock_color": (0.86, 0.78, 0.60),
    # Rock scale range (uniform between lo / hi, deterministic per seed).
    "rock_scale_min": 0.7,
    "rock_scale_max": 1.4,
    # Optional URDF spawn at the clearing.
    "spawn_urdf": None,
    "spawn_controller": None,
    "spawn_height": 0.2,
}


def _square_polygon(size: float):
    return ((0.0, 0.0), (size, 0.0), (size, size), (0.0, size))


def _disk_polygon(centre, radius, segments=24):
    import math

    cx, cy = centre
    return tuple(
        (
            cx + radius * math.cos(2 * math.pi * i / segments),
            cy + radius * math.sin(2 * math.pi * i / segments),
        )
        for i in range(segments)
    )


def _build_layout(params: Mapping[str, Any]) -> Layout:
    size = float(params["size"])
    centre = (size / 2.0, size / 2.0)

    layout = Layout(
        world_size=(size, size),
        terrain_params={
            "kind": "ridged_plus_fbm",
            "mix": float(params["mesa_mix"]),
            "grid_resolution": int(params["grid_resolution"]),
            "height_scale": float(params["height_scale"]),
            "octaves": int(params["octaves"]),
            "gain": float(params["gain"]),
            "lacunarity": float(params["lacunarity"]),
        },
    )

    layout.add_zone(Zone(
        name="desert",
        polygon=_square_polygon(size),
        tags=frozenset({"natural", "scatter_host", "arid"}),
    ))
    layout.add_zone(Zone(
        name="spawn_clearing",
        polygon=_disk_polygon(centre, float(params["clearing_radius"])),
        priority=10,
        exclusive=True,
        tags=frozenset({"spawn"}),
    ))
    layout.add_stamp(Stamp(
        name="clearing_pad",
        zone="spawn_clearing",
        mode="replace",
        height=float(params["height_scale"]) * 0.3,
    ))
    layout.add_prop_group(_PropGroup(
        name="rocks",
        proto_weights=(("Rock", 1.0),),
        scatter="poisson",
        include_zones=("desert",),
        params=(
            ("count", int(params["rock_count"])),
            ("min_dist", float(params["rock_min_dist"])),
        ),
    ))

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


def _fmt_color(c):
    return " ".join(format(float(x), ".4g") for x in c)


@dataclass
class OutdoorDesert:
    name: str = "outdoor_desert"

    def default_params(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for k, v in DEFAULT_PARAMS.items():
            if isinstance(v, tuple):
                out[k] = list(v)
            else:
                out[k] = v
        return out

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
        hm = solved.heightmap
        assert hm is not None
        size = float(merged["size"])
        spacing = (size / hm.width, size / hm.height)

        # Deterministic per-instance scale via a local RNG keyed on seed.
        from ..core.rng import derive, rng_from_seed
        from ..core.rng import derive_seed

        scale_rng = rng_from_seed(derive_seed(seed, "outdoor_desert.rock_scale"))
        lo = float(merged["rock_scale_min"])
        hi = float(merged["rock_scale_max"])
        color_str = _fmt_color(merged["rock_color"])

        props: list[PlacedProto] = []
        for placement in solved.props:
            scale = lo + (hi - lo) * scale_rng.random()
            props.append(PlacedProto(
                proto_url=get_url("Rock"),
                proto_type="Rock",
                translation=placement.translation,
                rotation=placement.rotation,
                extra_fields=(
                    ("type", "\"flat\""),
                    ("scale", format(scale, ".4g")),
                    ("color", color_str),
                ),
            ))

        return WorldDescription(
            title="outdoor_desert (omniworld)",
            heightmap=hm,
            heightmap_spacing=spacing,
            heightmap_origin=(0.0, 0.0),
            props=props,
            spawns=list(solved.spawns),
            metadata={"recipe": self.name, "prop_count": len(props)},
        )


register_recipe(OutdoorDesert())
