"""OmniSim procedural world generation library.

Public entry points:

    omniworld.generate(recipe, seed, out, params=None) -> GenerateResult
    omniworld.validate(world_path) -> ValidationReport
    omniworld.list_recipes() -> list[str]
    omniworld.get_recipe(name) -> Recipe

The shape of the library is defined by the procedural-world-generation-plan
under docs/developer/. This module is the Tier 1 scaffold; primitives, the
asset catalog, and biome recipes are filled in by later tier deliverables.
"""

from __future__ import annotations

from ._version import __version__
from .core.api import GenerateResult, generate
from .core.recipe import Recipe
from .core.registry import get_recipe, list_recipes, register_recipe
from .validation import ValidationReport, validate

# Importing the biomes subpackage registers the shipped recipes as a side
# effect. Keep this last so the registry is fully populated after `import
# omniworld`.
from . import biomes  # noqa: F401  (import for side effects)

__all__ = [
    "__version__",
    "GenerateResult",
    "Recipe",
    "ValidationReport",
    "generate",
    "get_recipe",
    "list_recipes",
    "register_recipe",
    "validate",
]
