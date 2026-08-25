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

"""Repository adapter for policy assets.

The core policy types accept ordinary dictionaries and paths.  This small adapter is
the only layer that knows OmniSim's source-tree layout, which keeps the algorithms
usable when the package is installed separately from a checkout.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def _object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return value


@dataclass(frozen=True)
class PolicyRepository:
    """Locate policy manifests and assets without importing training code."""

    root: Path

    @property
    def policies(self) -> Path:
        return self.root / "projects" / "policies"

    @property
    def skills(self) -> Path:
        return self.policies / "skills"

    def skill_paths(self) -> list[Path]:
        return sorted(self.skills.glob("*/*/skill.json"))

    def sequence_paths(self) -> list[Path]:
        return sorted((self.skills / "sequences").glob("*.json"))

    def skill_catalog(self) -> dict[str, dict[str, Any]]:
        result: dict[str, dict[str, Any]] = {}
        for path in self.skill_paths():
            raw = _object(path)
            name = str(raw.get("name", ""))
            if not name:
                raise ValueError(f"{path}: missing skill name")
            if name in result:
                raise ValueError(f"duplicate skill name {name!r}")
            result[name] = {**raw, "_path": path.as_posix()}
        return result

    def sequence_catalog(self) -> dict[str, dict[str, Any]]:
        result: dict[str, dict[str, Any]] = {}
        for path in self.sequence_paths():
            raw = _object(path)
            name = str(raw.get("name", ""))
            if not name:
                raise ValueError(f"{path}: missing sequence name")
            if name in result:
                raise ValueError(f"duplicate sequence name {name!r}")
            result[name] = {**raw, "_path": path.as_posix()}
        return result

    def motion_catalog(self) -> dict[str, dict[str, Any]]:
        path = self.policies / "motions" / "catalog.json"
        data = _object(path)
        motions = data.get("motions")
        if data.get("schema") != 1 or not isinstance(motions, dict):
            raise ValueError(f"{path}: expected motion catalog schema 1")
        return motions

    def resolve(self, value: str | Path) -> Path:
        path = Path(value)
        return path if path.is_absolute() else self.root / path
