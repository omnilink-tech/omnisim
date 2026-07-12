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

"""Lens presets — focal length in mm + derived horizontal field of view.

OmniSim's viewport thinks in FoV radians; cinematographers think
in millimeters. This module is the conversion + a curated preset list so
storyboards can ask for "85mm" (telephoto, compressed, hero) or "24mm"
(wide, establishing, immersive) instead of guessing radians.

FoV is computed for a 35mm full-frame sensor (36mm horizontal). Crop
factors aren't modeled — the simulator doesn't have one and adding a
fake crop just complicates the math.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


FULL_FRAME_HORIZ_MM = 36.0


@dataclass(frozen=True)
class Lens:
    """A named lens preset. `mood` is a free-form hint for the storyboard
    layer ('intimate', 'expansive', etc.) — purely advisory."""
    name: str
    focal_mm: float
    mood: str
    """One of: 'expansive' | 'natural' | 'flattering' | 'intimate' | 'compressed'."""

    @property
    def horizontal_fov_rad(self) -> float:
        return 2.0 * math.atan(FULL_FRAME_HORIZ_MM / (2.0 * self.focal_mm))

    @property
    def horizontal_fov_deg(self) -> float:
        return math.degrees(self.horizontal_fov_rad)


LENSES: dict[str, Lens] = {
    "16mm":  Lens("16mm",  16.0,  "expansive"),
    "24mm":  Lens("24mm",  24.0,  "expansive"),
    "28mm":  Lens("28mm",  28.0,  "natural"),
    "35mm":  Lens("35mm",  35.0,  "natural"),
    "50mm":  Lens("50mm",  50.0,  "natural"),
    "85mm":  Lens("85mm",  85.0,  "flattering"),
    "100mm": Lens("100mm", 100.0, "flattering"),
    "135mm": Lens("135mm", 135.0, "intimate"),
    "200mm": Lens("200mm", 200.0, "compressed"),
}


def get(name: str) -> Lens:
    """Look up a lens by preset name (e.g. '50mm'). Raises if not found."""
    needle = name.lower().strip()
    if needle in LENSES:
        return LENSES[needle]
    # also accept "50" without "mm"
    if needle + "mm" in LENSES:
        return LENSES[needle + "mm"]
    raise ValueError(f"unknown lens {name!r}; known: {sorted(LENSES.keys())}")


def from_focal_mm(mm: float) -> Lens:
    """Construct an ad-hoc lens for an arbitrary focal length."""
    mood = (
        "expansive" if mm < 28 else
        "natural" if mm < 60 else
        "flattering" if mm < 110 else
        "intimate" if mm < 150 else
        "compressed"
    )
    return Lens(name=f"{int(mm)}mm", focal_mm=float(mm), mood=mood)


def default_for(personality: str) -> Lens:
    """The lens that flatters this kind of subject by default. Storyboards
    can override per shot — this is the fallback when nothing is specified."""
    return {
        "hero": LENSES["50mm"],
        "industrial": LENSES["35mm"],
        "agile": LENSES["28mm"],
        "swarm": LENSES["24mm"],
        "object": LENSES["85mm"],
    }.get(personality, LENSES["35mm"])
