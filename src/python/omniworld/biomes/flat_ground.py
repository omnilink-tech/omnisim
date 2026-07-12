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

"""``flat_ground`` — the Tier 1 scaffold stub recipe.

Produces a flat rectangular arena with an optional URDFRobot spawn. Ships
exclusively so the end-to-end generator pipeline (generate -> emit ->
manifest -> validate) has a deterministic recipe to exercise from day one,
before the heightmap and scatter primitives land in T1.2 / T1.3.

This recipe is intentionally boring. It is *not* the ``outdoor_forest``
biome. Once T1.6 ships the real forest recipe, ``flat_ground`` stays in
the registry as the cheapest possible "did my pipeline survive?" check.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from ..core.recipe import Spawn, WorldDescription
from ..core.registry import register_recipe


DEFAULT_PARAMS: dict[str, Any] = {
    # Arena edge length in metres. Square.
    "size": 10.0,
    # World time step (ms) written into WorldInfo.basicTimeStep.
    "basic_time_step_ms": 16,
    # Optional URDF to spawn at the arena centre. None => no robot.
    "spawn_urdf": None,
    # Spawn controller name (only meaningful when spawn_urdf is set).
    "spawn_controller": None,
    # Spawn height above the floor (the centre of the robot base). 0.1 m
    # matches the hand-authored demos.
    "spawn_height": 0.1,
}


@dataclass
class FlatGround:
    name: str = "flat_ground"

    def default_params(self) -> dict[str, Any]:
        return dict(DEFAULT_PARAMS)

    def build(self, seed: int, params: Mapping[str, Any]) -> WorldDescription:
        # Merge defaults + user overrides. Unknown keys are an error: it is
        # too easy to typo ``sizxe`` and get a silent default.
        merged = dict(DEFAULT_PARAMS)
        for key, value in params.items():
            if key not in DEFAULT_PARAMS:
                raise ValueError(
                    f"unknown param for recipe {self.name!r}: {key!r}; "
                    f"known: {sorted(DEFAULT_PARAMS)}"
                )
            merged[key] = value

        # Seed is accepted but unused at Tier 1 — the stub is deterministic
        # regardless. Later tiers will use it for scatter/heightmap.
        del seed

        size = float(merged["size"])
        spawns: list[Spawn] = []
        if merged["spawn_urdf"] is not None:
            spawns.append(
                Spawn(
                    name="robot",
                    translation=(0.0, 0.0, float(merged["spawn_height"])),
                    rotation=(0.0, 0.0, 1.0, 0.0),
                    urdf_url=str(merged["spawn_urdf"]),
                    controller=(
                        str(merged["spawn_controller"])
                        if merged["spawn_controller"] is not None
                        else None
                    ),
                )
            )

        return WorldDescription(
            title="flat_ground (omniworld stub)",
            floor_size=(size, size),
            basic_time_step_ms=int(merged["basic_time_step_ms"]),
            spawns=spawns,
            metadata={"recipe": self.name, "stub": True},
        )


register_recipe(FlatGround())
