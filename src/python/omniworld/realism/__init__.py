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

"""Realism passes — the spine of the plan's realism goal.

A set of post-solver transforms that turn correct-but-sterile
placement into correct-and-believable placement. Each pass is a
separate module, each is individually toggleable, each is
deterministic given a seed.

Landed (T1.10):

- ``variation`` — per-instance scale, hue, rotation jitter.
- ``weathering`` — world-level ``age`` parameter driving a 4-tuple
  grade (dirt, wear, moisture, oxidation) per placement.

Not yet landed: ``settle`` (T1.9), ``clutter`` / ``surface_manifest``
(T1.11), ``ecology`` (T2.8), ``traces`` (T2.9), ``world_state``
(T2.10).
"""

from __future__ import annotations

from .ground_fit import GroundFit, ground_fit_on_heightmap
from .variation import Variation, jitter_hue_rgb, jitter_scale, variation_for_instance
from .weathering import WeatheringGrade, compute_grade, grade_to_color_multiplier

__all__ = [
    "GroundFit",
    "Variation",
    "WeatheringGrade",
    "compute_grade",
    "grade_to_color_multiplier",
    "ground_fit_on_heightmap",
    "jitter_hue_rgb",
    "jitter_scale",
    "variation_for_instance",
]
