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

"""Content-addressed policy artifact records and fail-closed promotion gates."""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Callable

from .repository import PolicyRepository
from .skill_graph import SkillGraph


def canonical_digest(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class Artifact:
    role: str
    path: str
    sha256: str
    size: int


def _artifact(repo: PolicyRepository, role: str, value: str) -> Artifact | None:
    if not value:
        return None
    path = repo.resolve(value)
    if not path.is_file():
        return None
    try:
        relative = path.relative_to(repo.root).as_posix()
    except ValueError:
        relative = path.as_posix()
    return Artifact(role, relative, file_digest(path), path.stat().st_size)


def _skill_artifacts(repo: PolicyRepository, skill: dict[str, Any]) -> list[Artifact]:
    values = [
        ("manifest", skill.get("_path", "")),
        ("ghost", (skill.get("ghost") or {}).get("lut", "")),
        ("checkpoint", (skill.get("policy") or {}).get("checkpoint", "")),
        ("world", (skill.get("deploy") or {}).get("world", "")),
        ("launcher", (skill.get("deploy") or {}).get("launcher", "")),
    ]
    return [item for role, value in values if (item := _artifact(repo, role, str(value)))]


def build_record(
    repo: PolicyRepository,
    kind: str,
    name: str,
    *,
    assembled_env: dict[str, str] | None = None,
    evidence_paths: list[str] | None = None,
) -> dict[str, Any]:
    skills = repo.skill_catalog()
    sequences = repo.sequence_catalog()
    artifacts: list[Artifact] = []
    raw: dict[str, Any]
    graph_digest = ""
    if kind == "skill":
        if name not in skills:
            raise KeyError(f"unknown skill {name!r}")
        raw = skills[name]
        artifacts.extend(_skill_artifacts(repo, raw))
    elif kind == "sequence":
        if name not in sequences:
            raise KeyError(f"unknown sequence {name!r}")
        raw = sequences[name]
        for role, value in (
            ("manifest", raw.get("_path", "")),
            ("world", raw.get("world", "")),
            ("launcher", (raw.get("deploy") or {}).get("launcher", "")),
            ("reproduces", raw.get("reproduces", "")),
            ("profile", f"projects/policies/skills/profiles/{raw.get('profile', '')}.json"),
        ):
            item = _artifact(repo, role, str(value))
            if item:
                artifacts.append(item)
        for skill_name in [raw.get("primary"), *(raw.get("skills") or [])]:
            if skill_name in skills:
                artifacts.extend(_skill_artifacts(repo, skills[skill_name]))
        graph = SkillGraph.from_sequence(raw, skills)
        graph_digest = canonical_digest(graph.to_dict())
    else:
        raise ValueError("kind must be 'skill' or 'sequence'")
    # Promotion binds the policy to the implementation and ABI it was verified with,
    # not just to a checkpoint filename. Missing platform-specific candidates are ignored.
    for role, value in (
        ("policy-core", "omnisim/policy/baton.py"),
        ("motion-catalog", "projects/policies/motions/catalog.json"),
        ("engine", "msys64/mingw64/bin/omnisim-bin.exe"),
        ("engine", "bin/omnisim-bin"),
        ("controller-abi", "lib/controller/Controller.dll"),
        ("controller-abi", "lib/controller/libController.so"),
    ):
        item = _artifact(repo, role, value)
        if item:
            artifacts.append(item)
    for value in evidence_paths or []:
        item = _artifact(repo, "benchmark-result", value)
        if item:
            artifacts.append(item)
    unique = {(item.role, item.path): item for item in artifacts}
    ordered = sorted(unique.values(), key=lambda item: (item.role, item.path))
    identity = {
        "kind": kind,
        "name": name,
        "manifest_status": raw.get("status", "experimental"),
        "artifacts": [asdict(item) for item in ordered],
        "deploy_env_sha256": canonical_digest(assembled_env) if assembled_env is not None else "",
        "skill_graph_sha256": graph_digest,
    }
    return {
        "schema": 1,
        **identity,
        "content_id": canonical_digest(identity),
    }


def evaluate_promotion(
    record: dict[str, Any],
    raw: dict[str, Any],
    benchmark_cases: list[dict[str, Any]],
) -> dict[str, Any]:
    """Return an evidence tier without mutating a manifest.

    ``catalogued`` means addressable; ``qualified`` additionally requires a verified
    manifest with structured verification; ``release`` additionally requires a
    versioned PASS benchmark for this exact target.  Missing evidence never promotes.
    """
    reasons: list[str] = []
    tier = "catalogued"
    if record.get("manifest_status") == "verified" and isinstance(raw.get("verification"), dict):
        tier = "qualified"
    else:
        reasons.append("verified manifest with structured verification is required")
    passed = [case for case in benchmark_cases if case.get("status") == "verified"]
    if tier == "qualified" and passed:
        tier = "release"
    elif not passed:
        reasons.append("versioned PASS benchmark for this target is required")
    evidence = []
    for case in passed:
        reference = case.get("reference_evidence") or {}
        evidence.append({
            "benchmark": case.get("name"),
            "machine_id": reference.get("machine_id", ""),
            "venue": reference.get("venue", ""),
            "engine": reference.get("engine", ""),
            "result_file": reference.get("result_file", ""),
        })
    return {
        **record,
        "promotion_tier": tier,
        "promotion_reasons": reasons,
        "benchmark_evidence": evidence,
        "verification": raw.get("verification", {}),
    }
