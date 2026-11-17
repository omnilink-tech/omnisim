#!/usr/bin/env python3
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

"""Validate (and, in future, build) the omniworld asset catalog.

T1.5 scope today: **validate** only. We ship a hand-authored
``src/python/omniworld/catalog/assets.json`` and this script enforces
that every entry has the required fields, that URLs follow the
``omnisim://`` scheme, and that the shipped biomes can still load
through the catalog after any edit.

T1.5 full scope (later): scan ``projects/objects/`` for PROTO files
and emit ``assets.json`` automatically, including bounding boxes,
collision tiers, and surface manifest hints derived from the PROTO
body. The catalog schema is versioned so the scanner can bump it
without breaking existing biomes.

Exit status:
- 0: catalog is valid.
- 1: catalog is invalid; diagnostic printed.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PY_SRC = REPO_ROOT / "src" / "python"
if str(PY_SRC) not in sys.path:
    sys.path.insert(0, str(PY_SRC))

from omniworld.catalog import Asset, CatalogError, iter_all, load_catalog  # noqa: E402


REQUIRED_BY_BIOMES = {
    # flat_ground uses no catalog entries.
    "outdoor_forest": {"Oak", "Pine", "Sassafras", "Cypress", "SimpleTree", "Rock"},
    "outdoor_desert": {"Rock"},
    "warehouse": {
        "WoodenPalletStack", "CardboardBox", "PlasticCrate", "WoodenBox",
    },
    "urban_block": {"ConstructionFrameBuilding", "ControlledStreetLight", "Bench"},
    "indoor_apartment": {"Wall", "Bed", "Table", "Toilet"},
}


def _validate_entry(asset: Asset, errors: list[str]) -> None:
    if not asset.name:
        errors.append("entry has empty name")
    if not asset.url:
        errors.append(f"{asset.name!r}: empty url")
    elif not asset.url.startswith("omnisim://"):
        errors.append(
            f"{asset.name!r}: url must start with omnisim:// (got {asset.url!r})"
        )


def main() -> int:
    errors: list[str] = []
    try:
        catalog = load_catalog()
    except CatalogError as exc:
        print(f"catalog load failed: {exc}", file=sys.stderr)
        return 1
    for asset in catalog.values():
        _validate_entry(asset, errors)

    known = {a.name for a in iter_all()}
    for biome, required in REQUIRED_BY_BIOMES.items():
        missing = required - known
        if missing:
            errors.append(
                f"biome {biome!r} needs catalog entries for "
                f"{sorted(missing)}"
            )

    if errors:
        for e in errors:
            print(f"ERROR: {e}", file=sys.stderr)
        return 1

    print(f"catalog ok ({len(catalog)} entries)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
