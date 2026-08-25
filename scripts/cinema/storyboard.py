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

"""Storyboard schema — the agent-facing DSL.

A storyboard is a JSON document describing the video: what world to
load, who the subject is, what look to apply, what beats to hit in
sequence, and what aspect ratios to deliver. The director consumes
this and orchestrates rendering / editing / branding.

Schema (all fields shown; many are optional with sensible defaults):

    {
      "title": "OmniQuad RL hero",
      "slug": "omniquad_hero",                    # filename stem; auto from title
      "world": "projects/policies/research/worlds/omniquad_rl_deploy.omniworld",
      "subject": "omniquad",                       # subjects.PROFILES key/alias
      "look": "teal_orange",                  # looks.LOOKS key
      "aspect_ratios": ["16:9", "9:16", "1:1"],
      "brand": {"title_card": true, "end_slate": true, "watermark": false},
      "world_settle_steps": 80,
      "shots": [
        {
          "beat": "establish",                # beats.BEATS key
          "duration_s": 5,                    # overrides beat default
          "lens": "24mm",                     # overrides look default
          "shot": "establish",                # overrides beat default
          "params": {"azimuth_deg": 45}       # forwarded to camera primitive
        },
        {"beat": "action"},
        {"beat": "hero"},
        {"beat": "resolve"}
      ]
    }
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

from . import beats, lenses, looks, subjects


@dataclass
class ShotSpec:
    """One shot within a storyboard. All fields optional except beat."""
    beat: str
    duration_s: float | None = None
    lens: str | None = None
    shot: str | None = None
    params: dict = field(default_factory=dict)
    look_override: str | None = None
    """Per-shot look override. Rare — usually the storyboard-level look wins."""

    def resolve_duration_s(self) -> float:
        if self.duration_s is not None:
            return float(self.duration_s)
        return beats.get(self.beat).default_duration_s

    def resolve_shot_primitive(self) -> str:
        if self.shot:
            return self.shot
        return beats.get(self.beat).default_shot

    def resolve_lens(self, story_look: looks.Look) -> lenses.Lens:
        return looks.resolve_lens(story_look, self.lens)


@dataclass
class BrandSpec:
    title_card: bool = True
    end_slate: bool = True
    watermark: bool = False
    """If true, overlay a small OmniSim mark in a corner the whole way."""


@dataclass
class Storyboard:
    title: str
    slug: str
    world: str
    subject_name: str
    look_name: str
    aspect_ratios: list[str]
    brand: BrandSpec
    shots: list[ShotSpec]
    world_settle_steps: int = 60
    fps: int = 30

    def resolved_look(self) -> looks.Look:
        return looks.get(self.look_name)

    def total_duration_s(self) -> float:
        return sum(s.resolve_duration_s() for s in self.shots)


def _slugify(s: str) -> str:
    s = re.sub(r"[^a-zA-Z0-9_-]+", "_", s.strip().lower()).strip("_")
    return s or "untitled"


def parse(payload: dict | str | Path) -> Storyboard:
    """Parse a storyboard from a dict, JSON string, or path to a JSON file.

    Validates that referenced beats/looks/subject exist so the director
    doesn't fail halfway through rendering with a cryptic error.
    """
    if isinstance(payload, Path) or (isinstance(payload, str) and Path(payload).is_file()):
        raw = Path(payload).read_text(encoding="utf-8")
        data = json.loads(raw)
    elif isinstance(payload, str):
        data = json.loads(payload)
    elif isinstance(payload, dict):
        data = payload
    else:
        raise TypeError(f"unsupported storyboard input: {type(payload)}")

    title = str(data.get("title") or "Untitled")
    slug = str(data.get("slug") or _slugify(title))
    world = str(data["world"])  # required
    subject_name = str(data["subject"])  # required
    look_name = str(data.get("look") or "natural")
    aspect_ratios = list(data.get("aspect_ratios") or ["16:9"])
    fps = int(data.get("fps") or 30)
    settle = int(data.get("world_settle_steps") or 60)

    # Validate the catalog references early so failures are loud + at the right layer.
    if subjects.find_profile(subject_name) is None:
        raise ValueError(
            f"unknown subject {subject_name!r}; known: {sorted(subjects.PROFILES.keys())}"
        )
    looks.get(look_name)

    brand_d = data.get("brand") or {}
    brand = BrandSpec(
        title_card=bool(brand_d.get("title_card", True)),
        end_slate=bool(brand_d.get("end_slate", True)),
        watermark=bool(brand_d.get("watermark", False)),
    )

    shots_raw = data.get("shots") or []
    if not shots_raw:
        raise ValueError("storyboard has no shots")
    shots: list[ShotSpec] = []
    for i, sd in enumerate(shots_raw):
        beat_name = sd.get("beat")
        if not beat_name:
            raise ValueError(f"shot #{i} missing 'beat'")
        beats.get(beat_name)  # validate
        shots.append(ShotSpec(
            beat=beat_name,
            duration_s=sd.get("duration_s"),
            lens=sd.get("lens"),
            shot=sd.get("shot"),
            params=sd.get("params") or {},
            look_override=sd.get("look"),
        ))

    return Storyboard(
        title=title, slug=slug, world=world, subject_name=subject_name,
        look_name=look_name, aspect_ratios=aspect_ratios,
        brand=brand, shots=shots,
        world_settle_steps=settle, fps=fps,
    )


def template(title: str = "New Cinema Piece", subject: str = "omniquad",
             world: str = "projects/policies/research/worlds/omniquad_rl_deploy.omniworld") -> dict:
    """Return a starter storyboard the agent can edit. Hits a clean 4-beat
    structure (establish → action → hero → resolve) at 30s total."""
    return {
        "title": title,
        "subject": subject,
        "world": world,
        "look": "teal_orange",
        "aspect_ratios": ["16:9", "9:16"],
        "brand": {"title_card": True, "end_slate": True, "watermark": False},
        "fps": 30,
        "world_settle_steps": 80,
        "shots": [
            {"beat": "establish", "duration_s": 6, "params": {"azimuth_deg": 35, "elevation_deg": 18}},
            {"beat": "action", "duration_s": 8, "params": {"side": "left"}},
            {"beat": "hero", "duration_s": 5},
            {"beat": "resolve", "duration_s": 8},
        ],
    }
