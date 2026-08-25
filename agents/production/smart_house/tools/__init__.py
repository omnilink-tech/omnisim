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

"""SmartHouse agent — tool registry.

Each module exports `SPEC` or `SPECS` (a list). `load_all()` aggregates every
spec into a registry the runner consumes. Drop in a new file to add a tool.

Mirrors the loader in ``agents/production/husky_maze/tools/__init__.py``.
"""

from __future__ import annotations

import importlib
import pkgutil
from pathlib import Path
from typing import Dict, List

from ._base import ToolSpec  # noqa: F401  re-exported for tool authors


def load_all() -> Dict[str, ToolSpec]:
    registry: Dict[str, ToolSpec] = {}
    pkg_path = Path(__file__).resolve().parent
    package_name = __name__

    for info in pkgutil.iter_modules([str(pkg_path)]):
        if info.name.startswith("_"):
            continue
        module = importlib.import_module(f"{package_name}.{info.name}")
        specs: List[ToolSpec] = []
        if hasattr(module, "SPECS") and isinstance(module.SPECS, list):
            specs.extend(module.SPECS)
        if hasattr(module, "SPEC") and isinstance(module.SPEC, ToolSpec):
            specs.append(module.SPEC)
        for spec in specs:
            if spec.name in registry:
                raise RuntimeError(
                    f"duplicate tool name {spec.name!r} (in {info.name})"
                )
            registry[spec.name] = spec
    return registry


__all__ = ["ToolSpec", "load_all"]
