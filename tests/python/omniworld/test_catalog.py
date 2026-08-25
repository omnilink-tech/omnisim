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

"""T1.5 asset catalog tests."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from omniworld import catalog


REPO_ROOT = Path(__file__).resolve().parents[3]
CATALOG_VALIDATOR = REPO_ROOT / "scripts" / "dev" / "build_asset_catalog.py"


def test_catalog_loads():
    assets = catalog.load_catalog()
    assert assets
    assert "Oak" in assets
    assert "WoodenPalletStack" in assets
    assert "Wall" in assets


def test_catalog_urls_all_webots_scheme():
    for asset in catalog.iter_all():
        assert asset.url.startswith("omnisim://"), (
            f"{asset.name}: non-webots URL {asset.url!r}"
        )


def test_catalog_get_url_matches_entry():
    assert catalog.get_url("Oak") == catalog.get("Oak").url


def test_catalog_get_unknown_raises():
    with pytest.raises(KeyError):
        catalog.get("NotARealProto")


def test_catalog_iter_tagged():
    trees = catalog.iter_tagged("tree", "outdoor")
    assert trees, "expected at least one outdoor tree"
    assert all("tree" in a.tags and "outdoor" in a.tags for a in trees)

    # Empty-tag query returns every asset.
    all_assets = catalog.iter_tagged()
    assert len(all_assets) == len(list(catalog.iter_all()))


def test_catalog_no_duplicate_names(tmp_path: Path):
    """A duplicate-named entry must raise at parse time."""
    payload = {
        "schema_version": catalog.CATALOG_SCHEMA_VERSION,
        "assets": [
            {"name": "X", "url": "omnisim://foo", "tags": [], "settle": False},
            {"name": "X", "url": "omnisim://bar", "tags": [], "settle": False},
        ],
    }
    path = tmp_path / "dup.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(catalog.CatalogError):
        catalog.load_catalog(path)


def test_catalog_schema_version_mismatch_rejected(tmp_path: Path):
    payload = {"schema_version": 9999, "assets": []}
    path = tmp_path / "wrong.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(catalog.CatalogError):
        catalog.load_catalog(path)


def test_catalog_validator_script_exits_zero():
    r = subprocess.run(
        [sys.executable, str(CATALOG_VALIDATOR)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert r.returncode == 0, (
        f"validator failed:\nstdout: {r.stdout}\nstderr: {r.stderr}"
    )


def test_catalog_covers_all_shipped_biomes():
    """Every PROTO type the currently shipped biomes reference in their
    output must resolve through the catalog.

    We generate a world of each biome, collect the proto_type values,
    and verify each is in the catalog.
    """
    from omniworld import generate, list_recipes

    import tempfile

    shipped = list_recipes()
    names_seen: set[str] = set()
    with tempfile.TemporaryDirectory() as td:
        for recipe in shipped:
            path = Path(td) / f"{recipe}.wbt"
            result = generate(recipe, seed=42, out=path)
            for prop in result.description.props:
                names_seen.add(prop.proto_type)
    known = {a.name for a in catalog.iter_all()}
    missing = names_seen - known
    assert not missing, f"catalog missing entries for: {sorted(missing)}"
