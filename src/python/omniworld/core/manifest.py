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

"""Seed manifest emission.

Every generator run emits a JSON manifest alongside the ``.wbt`` file.
The manifest captures everything a reader needs to reproduce or audit the
output: library version, recipe name, seed, resolved parameters, and a
SHA256 of the generated world bytes.

The manifest schema is versioned by ``MANIFEST_SCHEMA_VERSION``. Bump when
fields are added, removed, or reinterpreted. Additions that are optional
and backwards-compatible do NOT require a version bump.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .._version import __version__

MANIFEST_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class SeedManifest:
    """Everything needed to reproduce one generator invocation.

    Contains no absolute paths — two machines generating from the same
    (recipe, seed, params) must produce byte-identical manifests so CI
    regressions can diff them. ``world_filename`` is just the basename.
    """

    schema_version: int
    library_version: str
    recipe: str
    seed: int
    params: dict[str, Any]
    world_sha256: str
    world_filename: str
    # Extra structured metadata the recipe wanted to expose (e.g. which
    # sub-algorithm was picked, which priors were loaded). Reserved for use
    # by later tiers; the field is stable, the contents are recipe-defined.
    extra: dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> str:
        # sort_keys for byte-stable output across dict-insertion orders.
        return json.dumps(asdict(self), indent=2, sort_keys=True) + "\n"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def build_manifest(
    *,
    recipe: str,
    seed: int,
    params: dict[str, Any],
    world_path: Path,
    world_bytes: bytes,
    extra: dict[str, Any] | None = None,
) -> SeedManifest:
    return SeedManifest(
        schema_version=MANIFEST_SCHEMA_VERSION,
        library_version=__version__,
        recipe=recipe,
        seed=seed,
        params=dict(params),
        world_sha256=sha256_bytes(world_bytes),
        world_filename=Path(world_path).name,
        extra=dict(extra or {}),
    )


def manifest_path_for(world_path: Path) -> Path:
    """Manifest sits next to the world: ``<world>.seed.json``."""
    return world_path.with_suffix(world_path.suffix + ".seed.json")


def write_manifest(manifest: SeedManifest, path: Path) -> None:
    path.write_text(manifest.to_json(), encoding="utf-8")
