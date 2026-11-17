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

"""Cross-morphology policy benchmark matrix validation."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .repository import PolicyRepository


def load(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: benchmark matrix must be an object")
    return data


def validate(repo: PolicyRepository, matrix: dict[str, Any]) -> list[str]:
    issues: list[str] = []
    if matrix.get("schema") != 1:
        issues.append(f"benchmark matrix schema must be 1, got {matrix.get('schema')!r}")
    skills, sequences = repo.skill_catalog(), repo.sequence_catalog()
    seen: set[str] = set()
    morphologies: set[str] = set()
    for index, case in enumerate(matrix.get("cases", [])):
        name = str(case.get("name", ""))
        where = name or f"case[{index}]"
        if not name or name in seen:
            issues.append(f"{where}: benchmark name must be non-empty and unique")
        seen.add(name)
        target = case.get("target") or {}
        kind, target_name = target.get("kind"), target.get("name")
        catalog = skills if kind == "skill" else sequences if kind == "sequence" else {}
        if target_name not in catalog:
            issues.append(f"{where}: unknown {kind!r} target {target_name!r}")
        morphology = str(case.get("morphology", ""))
        if not morphology:
            issues.append(f"{where}: morphology is required")
        morphologies.add(morphology)
        if not case.get("support"):
            issues.append(f"{where}: support disclosure is required")
        if not case.get("measurements"):
            issues.append(f"{where}: at least one measurement is required")
        if case.get("status") == "verified" and not case.get("evidence"):
            issues.append(f"{where}: verified matrix entry requires evidence")
    required = set(matrix.get("required_morphologies", []))
    missing = sorted(required - morphologies)
    if missing:
        issues.append(f"benchmark matrix misses required morphologies: {missing}")
    return issues
