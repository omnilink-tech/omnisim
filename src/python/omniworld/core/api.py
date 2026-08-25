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

"""The public ``generate`` entry point.

Orchestrates: recipe lookup -> recipe.build -> emit .wbt -> write file ->
write seed manifest. All deterministic given (recipe_name, seed, params).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from ..emit import render_wbt
from .manifest import (
    SeedManifest,
    build_manifest,
    manifest_path_for,
    write_manifest,
)
from .recipe import WorldDescription
from .registry import get_recipe


@dataclass
class GenerateResult:
    """Return value of ``generate``. Captures every path the run touched."""

    world_path: Path
    manifest_path: Path
    manifest: SeedManifest
    description: WorldDescription


def generate(
    recipe: str,
    seed: int,
    out: str | Path,
    params: Mapping[str, Any] | None = None,
) -> GenerateResult:
    """Generate a world.

    Arguments:
        recipe: registered recipe name (see ``list_recipes``).
        seed: parent seed. All sub-RNGs derive deterministically from this.
        out: path to the ``.wbt`` file to write. Parent directory must exist.
        params: recipe-specific overrides. Must be a subset of the recipe's
            declared parameters.

    Returns:
        A ``GenerateResult`` whose paths are both written on disk.
    """
    recipe_obj = get_recipe(recipe)
    resolved_params = dict(params or {})

    # Recipes build deterministically from (seed, params). The emitter is
    # a pure function of the WorldDescription.
    world = recipe_obj.build(seed=seed, params=resolved_params)
    world_text = render_wbt(world)
    world_bytes = world_text.encode("utf-8")

    # SINGLE-WRITE: a generated world is a new file, so it gets the current
    # extension. An explicitly-supplied world extension is honoured as given.
    world_path = Path(out)
    if world_path.suffix not in ('.omniworld', '.wbt'):
        world_path = world_path.with_name(world_path.name + '.omniworld')
    world_path.write_bytes(world_bytes)

    manifest = build_manifest(
        recipe=recipe,
        seed=seed,
        params=resolved_params,
        world_path=world_path,
        world_bytes=world_bytes,
        extra=world.metadata,
    )
    mpath = manifest_path_for(world_path)
    write_manifest(manifest, mpath)

    return GenerateResult(
        world_path=world_path,
        manifest_path=mpath,
        manifest=manifest,
        description=world,
    )
