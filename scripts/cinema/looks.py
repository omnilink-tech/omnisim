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

"""Named looks — lens + color-grade preset pairings.

A "look" bundles a recommended lens with an ffmpeg filter chain that gets
applied to the encoded clip in post. Storyboards reference looks by name
(`"look": "golden_hour"`) and the director resolves the rest.

The filter chains use only standard ffmpeg filters — no 3D LUT files —
so this works on any build of ffmpeg without bundling extra assets.
For finer control, drop in a `.cube` LUT and reference it from a look.
"""

from __future__ import annotations

from dataclasses import dataclass

from . import lenses


@dataclass(frozen=True)
class Look:
    """A complete visual treatment: lens + color grade + framing tendency."""
    name: str
    description: str
    preferred_lens: str
    """Lens preset name. Overridden if the shot specifies its own lens."""
    grade_filter: str
    """ffmpeg -vf filter string applied during encode. Empty = no grade."""
    mood_keywords: tuple[str, ...]


LOOKS: dict[str, Look] = {
    "natural": Look(
        name="natural",
        description="Clean, untreated render. The OmniSim default — use when "
                    "you want viewers to focus on the sim, not the cinematography.",
        preferred_lens="35mm",
        grade_filter="",
        mood_keywords=("clean", "documentary", "technical"),
    ),
    "teal_orange": Look(
        name="teal_orange",
        description="Hollywood blockbuster teal-shadow / orange-skin grade. "
                    "Punchy contrast, complementary palette — reads as 'cinematic' "
                    "even on flat-lit sim renders.",
        preferred_lens="50mm",
        grade_filter=(
            "eq=contrast=1.15:saturation=1.1,"
            "colorbalance=rs=-0.08:gs=-0.02:bs=0.08:rm=0:gm=0:bm=0:"
            "rh=0.08:gh=0.02:bh=-0.05,"
            "curves=preset=increase_contrast"
        ),
        mood_keywords=("blockbuster", "hero", "trailer"),
    ),
    "golden_hour": Look(
        name="golden_hour",
        description="Warm low-angle sun cast. Boosts reds and yellows, drops "
                    "blue mids slightly. Best on outdoor demos with sky.",
        preferred_lens="50mm",
        grade_filter=(
            "eq=brightness=0.03:saturation=1.2,"
            "colorbalance=rs=0.05:gs=0.03:bs=-0.05:rh=0.1:gh=0.05:bh=-0.08,"
            "vignette=PI/5"
        ),
        mood_keywords=("warm", "outdoor", "establishing"),
    ),
    "industrial_steel": Look(
        name="industrial_steel",
        description="Desaturated, cool, slightly green-tinted — reads as "
                    "'factory floor' or 'mission control'. Pairs with arms, "
                    "warehouse, conveyor demos.",
        preferred_lens="35mm",
        grade_filter=(
            "eq=saturation=0.65:contrast=1.08,"
            "colorbalance=rs=-0.05:gs=0.05:bs=0.08"
        ),
        mood_keywords=("industrial", "factory", "cool"),
    ),
    "noir": Look(
        name="noir",
        description="High-contrast monochrome. Use sparingly — great for a "
                    "single beat in an otherwise color piece (a 'specs' shot).",
        preferred_lens="85mm",
        grade_filter="hue=s=0,eq=contrast=1.35:brightness=-0.04",
        mood_keywords=("monochrome", "moody", "stylized"),
    ),
    "neon_night": Look(
        name="neon_night",
        description="Pushed blues + magentas with crushed blacks. Pairs with "
                    "NightSky-background worlds (the OmniSim default).",
        preferred_lens="35mm",
        grade_filter=(
            "eq=contrast=1.2:saturation=1.25:brightness=-0.05,"
            "colorbalance=rs=-0.1:gs=-0.05:bs=0.15:rh=0.05:gh=-0.05:bh=0.05"
        ),
        mood_keywords=("night", "futurist", "synthwave"),
    ),
    "documentary": Look(
        name="documentary",
        description="Slight contrast bump and a touch of warmth. Calm, "
                    "non-distracting — best for talking-head-style explainers.",
        preferred_lens="50mm",
        grade_filter="eq=contrast=1.06:saturation=1.05,colorbalance=rh=0.03:bh=-0.03",
        mood_keywords=("calm", "explanatory", "neutral"),
    ),
    "high_key": Look(
        name="high_key",
        description="Bright, soft, lifted shadows — feels like a glossy "
                    "product render. Pairs with white/light environments.",
        preferred_lens="50mm",
        grade_filter="eq=brightness=0.08:contrast=0.95:saturation=1.05",
        mood_keywords=("product", "bright", "glossy"),
    ),
}


def get(name: str) -> Look:
    """Look up a named look. Raises ValueError if unknown."""
    needle = name.lower().strip()
    if needle in LOOKS:
        return LOOKS[needle]
    raise ValueError(f"unknown look {name!r}; known: {sorted(LOOKS.keys())}")


def resolve_lens(look: Look, override: str | None = None) -> "lenses.Lens":
    """Resolve the effective lens for a shot: explicit override beats the
    look's preferred lens. Returns a Lens object."""
    if override:
        return lenses.get(override)
    return lenses.get(look.preferred_lens)
