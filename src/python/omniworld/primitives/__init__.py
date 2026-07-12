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

"""Primitives layer: the generator building blocks.

Landed:

- ``heightmap`` (T1.2) — Heightmap container, bilinear sample, fBm /
  ridged / worley noise, masks, blending, stamping, thermal and
  hydraulic erosion.
- ``scatter`` (T1.3) — poisson_annulus, poisson_polygon, grid_jittered,
  clustered, along_path, on_surface, pick / pick_weighted.
- ``nested_scatter`` (T1.11) — place children on a parent's declared
  surface. Surface manifests live on catalog entries.
"""

from __future__ import annotations

from .heightmap import (
    Heightmap,
    apply_mask,
    blend,
    crater_pattern,
    erode_hydraulic,
    erode_thermal,
    fbm,
    mask_polygon,
    mask_radial,
    mask_rect,
    ridged,
    stamp,
    worley,
)
from .nested_scatter import scatter_on_surface
from .rock_mesh import RockMesh, generate_rock, icosphere
from .scatter import (
    along_path,
    clustered,
    grid_jittered,
    on_surface,
    pick,
    pick_weighted,
    poisson_annulus,
    poisson_polygon,
)

__all__ = [
    "Heightmap",
    "apply_mask",
    "blend",
    "crater_pattern",
    "erode_hydraulic",
    "erode_thermal",
    "fbm",
    "mask_polygon",
    "mask_radial",
    "mask_rect",
    "ridged",
    "stamp",
    "worley",
    "along_path",
    "clustered",
    "grid_jittered",
    "on_surface",
    "pick",
    "pick_weighted",
    "poisson_annulus",
    "poisson_polygon",
    "scatter_on_surface",
    "RockMesh",
    "generate_rock",
    "icosphere",
]
