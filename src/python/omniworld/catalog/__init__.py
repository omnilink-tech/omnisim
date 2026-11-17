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

"""Asset catalog.

Hand-authored JSON catalog of every PROTO the shipped biomes may
place. Biomes look up short names through ``query.get_url`` instead
of hardcoding ``omnisim://`` URLs. Catalogue-driven tag queries
(``iter_tagged("tree", "outdoor")``) are the path the
``pick_from_biome`` helper in the plan's T1.5 evolves into.

Tier 1 scope is a consolidation: ``assets.json`` is hand-authored,
validated by CI, and consumed by every biome. Full PROTO scanning
(the ``build_asset_catalog`` script that derives bounding boxes,
collision tiers, and surface manifests from the raw ``.proto`` files)
is still a later slice.
"""

from __future__ import annotations

from .query import (
    Asset,
    CATALOG_SCHEMA_VERSION,
    CatalogError,
    Surface,
    get,
    get_url,
    iter_all,
    iter_tagged,
    load_catalog,
)

__all__ = [
    "Asset",
    "CATALOG_SCHEMA_VERSION",
    "CatalogError",
    "Surface",
    "get",
    "get_url",
    "iter_all",
    "iter_tagged",
    "load_catalog",
]
