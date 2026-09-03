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

"""Pytest config: put ``src/python`` on sys.path so tests can import omniworld
without requiring an editable install, and share generated worlds.

``generated_world`` is a session-scoped memo of ``omniworld.generate``. The
generator is deterministic BY CONTRACT (same ``(recipe, seed, params)`` ->
byte-identical output, and the output does not depend on the ``out`` path --
both are asserted by the per-biome ``*_deterministic`` tests), so handing two
tests the same result cannot change what either of them asserts. It only
removes the repeat work: before this fixture ``test_mars.py`` alone built the
Mars terrain 28 times for 15 distinct parameter sets (MEASURED 2026-09-02:
219 s for 29 tests).

The shared result and its files are READ-ONLY. A test that needs to mutate a
``GenerateResult`` (or rewrite the ``.wbt`` on disk) must copy it first, or
call ``omniworld.generate`` itself into its own ``tmp_path``.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Callable, Mapping

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PY_SRC = REPO_ROOT / "src" / "python"
if str(PY_SRC) not in sys.path:
    sys.path.insert(0, str(PY_SRC))


def _freeze_params(params: Mapping[str, Any] | None) -> str:
    """A hashable, order-independent key for a recipe's ``params`` mapping."""
    return json.dumps(dict(params or {}), sort_keys=True, default=repr)


@pytest.fixture(scope="session")
def generated_world(tmp_path_factory: pytest.TempPathFactory) -> Callable[..., Any]:
    """``generated_world(recipe, seed, params=None) -> GenerateResult``.

    Generates each distinct ``(recipe, seed, params)`` ONCE per session, into
    its own directory under the session temp root, and returns the same
    ``GenerateResult`` to every later caller. The world file is written as
    ``<recipe>.wbt`` (the extension the tests always used), so
    ``result.world_path`` / ``result.manifest_path`` are the files to read.

    Read-only by convention -- see the module docstring.
    """
    from omniworld import generate

    root = tmp_path_factory.mktemp("omniworld_shared")
    cache: dict[tuple[str, int, str], Any] = {}

    def _get(recipe: str, seed: int, params: Mapping[str, Any] | None = None):
        key = (recipe, int(seed), _freeze_params(params))
        result = cache.get(key)
        if result is None:
            out_dir = root / f"{len(cache):03d}_{recipe}_{seed}"
            out_dir.mkdir()
            # A fresh dict per call: a recipe must never see (or keep) the
            # caller's mapping, and the key above already pins its contents.
            result = generate(
                recipe, seed=seed, out=out_dir / f"{recipe}.wbt",
                params=dict(params or {}),
            )
            cache[key] = result
        return result

    return _get
