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

"""Weathering pass.

Closes the second of the plan's T1.10 perceptual lies: every surface
ships at factory-new. A world-level ``age`` parameter (0.0 pristine ->
1.0 abandoned) combines with a per-zone ``use_intensity`` and a tiny
per-instance jitter to produce a 4-tuple ``WeatheringGrade`` for each
placement.

The canonical consumer is the renderer's extended-BRDF material
perturbation. In the Tier 1 scaffold the renderer does not yet have
that shader; the fallback is a base-color darken:

    color_multiplier = 1.0 - 0.25 * dirt - 0.15 * wear

applied via the PROTO's ``color`` field. This is a weak proxy for a
real weathering shader but is enough to see the world change as age
rises.
"""

from __future__ import annotations

import random
from dataclasses import dataclass

from ..core.rng import derive_seed, rng_from_seed


@dataclass(frozen=True)
class WeatheringGrade:
    """The 4-channel weathering grade for one placement.

    Each component is in ``[0, 1]``. The tuple shape matches what the
    plan's T1.10 renderer shader will consume — the caller that
    eventually hooks up the extended BRDF can forward these values
    directly.
    """

    dirt: float
    wear: float
    moisture: float
    oxidation: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.dirt, self.wear, self.moisture, self.oxidation)


def _clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))


def compute_grade(
    parent_seed: int,
    group_name: str,
    instance_index: int,
    *,
    world_age: float,
    use_intensity: float = 0.0,
    exposure: float = 0.0,
    moisture_base: float = 0.0,
    oxidation_base: float = 0.0,
    jitter: float = 0.05,
) -> WeatheringGrade:
    """Compute the weathering grade for one placement.

    Weights are deliberately simple. Each pillar is driven by one or
    two inputs plus a small per-instance jitter:

    - ``dirt`` ~ world_age + 0.5 * use_intensity
    - ``wear`` ~ world_age + use_intensity
    - ``moisture`` ~ moisture_base + 0.3 * exposure
    - ``oxidation`` ~ oxidation_base + 0.4 * world_age (metal oxidises
      on the clock; plastic and wood don't).

    Callers pick per-biome / per-prop ``moisture_base`` and
    ``oxidation_base`` — e.g. rocks near water get moisture, metal
    items in humid rooms get oxidation.
    """
    if not (0.0 <= world_age <= 1.0):
        raise ValueError("compute_grade: world_age must be in [0, 1]")
    if not (0.0 <= use_intensity <= 1.0):
        raise ValueError("compute_grade: use_intensity must be in [0, 1]")
    if not (0.0 <= exposure <= 1.0):
        raise ValueError("compute_grade: exposure must be in [0, 1]")

    sub_seed = derive_seed(parent_seed, f"weathering.{group_name}.{instance_index}")
    rng = rng_from_seed(sub_seed)

    def _j() -> float:
        return (rng.random() * 2.0 - 1.0) * jitter

    dirt = _clamp01(world_age + 0.5 * use_intensity + _j())
    wear = _clamp01(world_age + use_intensity + _j())
    moisture = _clamp01(moisture_base + 0.3 * exposure + _j())
    oxidation = _clamp01(oxidation_base + 0.4 * world_age + _j())
    return WeatheringGrade(dirt, wear, moisture, oxidation)


def grade_to_color_multiplier(grade: WeatheringGrade) -> float:
    """Fallback renderer hook: a single multiplicative darkening factor
    derived from the grade.

    Used until the extended-BRDF shader lands. A caller applies it to
    a prop's base colour via the ``color`` PROTO field.
    """
    # Saturates gently so even abandoned worlds retain some color.
    darken = 0.25 * grade.dirt + 0.15 * grade.wear + 0.05 * grade.oxidation
    return _clamp01(1.0 - darken)


__all__ = [
    "WeatheringGrade",
    "compute_grade",
    "grade_to_color_multiplier",
]
