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

"""Emitters: turn a WorldDescription into on-disk OmniSim artifacts.

- ``wbt`` writes the ``.wbt`` text file.
- ``elevation_grid`` (T1.2) emits ``ElevationGrid`` terrain geometry.

Later tiers add: ``splatmap`` (T2.1), ``snow_accumulation`` (T2.10).
"""

from __future__ import annotations

from .elevation_grid import render_elevation_grid, render_terrain_solid
from .wbt import render_wbt

__all__ = [
    "render_wbt",
    "render_elevation_grid",
    "render_terrain_solid",
]
