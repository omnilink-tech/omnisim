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

"""Catalog query API.

The catalog is the single source of truth for every PROTO the
generator may place. It answers two questions biomes repeatedly ask:

- "Given a short proto name, what is the ``omnisim://`` URL?"
- "Give me every proto matching this biome / tag combination."

At Tier 1 the catalog is hand-authored ([assets.json](./assets.json)).
T1.5's full ambition — scanning PROTOs under
``projects/objects/`` and emitting metadata automatically — lands as
``scripts/dev/build_asset_catalog.py`` in a later iteration. Today's
validator ensures the shipped JSON stays consistent and covers every
PROTO the shipped biomes reference.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Iterable


CATALOG_SCHEMA_VERSION = 1
_CATALOG_PATH = Path(__file__).resolve().parent / "assets.json"


@dataclass(frozen=True)
class Surface:
    """A named flat region on the top / side of a PROTO, in the
    PROTO's local frame.

    - ``z_offset``: height above the PROTO's origin where the surface
      sits (metres).
    - ``half_extents``: (x, y) half-lengths of the usable rectangle
      on the surface.
    """

    name: str
    z_offset: float
    half_extents: tuple[float, float]


@dataclass(frozen=True)
class Asset:
    """Single catalog entry."""

    name: str
    url: str
    tags: frozenset[str]
    settle: bool = False
    # Named flat regions the nested-scatter primitive can target.
    surfaces: tuple[Surface, ...] = ()

    def surface(self, name: str) -> Surface:
        for s in self.surfaces:
            if s.name == name:
                return s
        raise KeyError(
            f"asset {self.name!r} has no surface {name!r}; "
            f"known: {sorted(s.name for s in self.surfaces)}"
        )


class CatalogError(ValueError):
    """Raised on malformed catalog files."""


def _parse(payload: dict) -> dict[str, Asset]:
    version = int(payload.get("schema_version", 0))
    if version != CATALOG_SCHEMA_VERSION:
        raise CatalogError(
            f"catalog schema_version {version}; this build expects "
            f"{CATALOG_SCHEMA_VERSION}"
        )
    by_name: dict[str, Asset] = {}
    for entry in payload.get("assets", []):
        name = entry["name"]
        if name in by_name:
            raise CatalogError(f"catalog has duplicate entry: {name!r}")
        surfaces_payload = entry.get("surfaces") or {}
        surfaces = tuple(
            Surface(
                name=sname,
                z_offset=float(sdata["z_offset"]),
                half_extents=(
                    float(sdata["half_extents"][0]),
                    float(sdata["half_extents"][1]),
                ),
            )
            for sname, sdata in sorted(surfaces_payload.items())
        )
        by_name[name] = Asset(
            name=name,
            url=entry["url"],
            tags=frozenset(entry.get("tags", [])),
            settle=bool(entry.get("settle", False)),
            surfaces=surfaces,
        )
    return by_name


@lru_cache(maxsize=1)
def _load_default() -> dict[str, Asset]:
    return _parse(json.loads(_CATALOG_PATH.read_text(encoding="utf-8")))


def load_catalog(path: str | Path | None = None) -> dict[str, Asset]:
    """Parse a catalog file; caches the default path."""
    if path is None:
        return dict(_load_default())
    p = Path(path)
    return _parse(json.loads(p.read_text(encoding="utf-8")))


def get(name: str) -> Asset:
    """Look up a single entry by short name. Raises ``KeyError`` if
    the catalog does not contain the name."""
    catalog = _load_default()
    try:
        return catalog[name]
    except KeyError:
        known = ", ".join(sorted(catalog))
        raise KeyError(
            f"unknown catalog entry {name!r}; known: {known}"
        ) from None


def get_url(name: str) -> str:
    """Convenience: look up just the URL."""
    return get(name).url


def iter_all() -> Iterable[Asset]:
    return list(_load_default().values())


def iter_tagged(*tags: str) -> list[Asset]:
    """Return every asset that has ALL of the given tags.

    Empty tag list returns every asset.
    """
    required = frozenset(tags)
    return [a for a in iter_all() if required.issubset(a.tags)]


__all__ = [
    "Asset",
    "CatalogError",
    "CATALOG_SCHEMA_VERSION",
    "get",
    "get_url",
    "iter_all",
    "iter_tagged",
    "load_catalog",
]
