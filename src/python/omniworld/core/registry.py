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

"""Recipe registry.

Recipes self-register at import time. The ``biomes`` subpackage is
imported by ``omniworld/__init__.py``, which pulls every shipped biome
module in and triggers their ``register_recipe`` calls.
"""

from __future__ import annotations

from .recipe import Recipe

_REGISTRY: dict[str, Recipe] = {}


def register_recipe(recipe: Recipe) -> None:
    name = recipe.name
    if name in _REGISTRY and _REGISTRY[name] is not recipe:
        raise ValueError(f"recipe name already registered: {name!r}")
    _REGISTRY[name] = recipe


def get_recipe(name: str) -> Recipe:
    try:
        return _REGISTRY[name]
    except KeyError as e:
        known = ", ".join(sorted(_REGISTRY)) or "<none>"
        raise KeyError(f"unknown recipe {name!r}; known: {known}") from e


def list_recipes() -> list[str]:
    return sorted(_REGISTRY)
