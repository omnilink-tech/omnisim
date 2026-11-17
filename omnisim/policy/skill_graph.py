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

"""Typed BATON skill graphs with lossless legacy-schedule adapters."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


def _mode(token: str, kind: str) -> str:
    if kind == "schedule":
        return token.split(":", 1)[0].strip()
    value = token.split(",", 1)[0].strip()
    return value[:-2] if value.endswith("to") else value


@dataclass(frozen=True)
class SkillNode:
    id: str
    skill: str
    mode: str
    command: str


@dataclass(frozen=True)
class SkillEdge:
    source: str
    target: str
    trigger: str
    handover: str
    gate: str
    preconditions: tuple[str, ...] = ()
    postconditions: tuple[str, ...] = ()
    recovery: str = "hold-last-safe"


@dataclass(frozen=True)
class SkillGraph:
    name: str
    entry: str
    nodes: tuple[SkillNode, ...]
    edges: tuple[SkillEdge, ...]
    arbiter_kind: str
    source_value: str
    schema: int = 1
    metadata: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_sequence(cls, sequence: dict[str, Any], skills: dict[str, dict[str, Any]]) -> "SkillGraph":
        arbiter = sequence.get("arbiter") or {}
        kind = str(arbiter.get("kind", "course"))
        value = str(arbiter.get("value", ""))
        sep = "," if kind == "schedule" else ";"
        tokens = [item.strip() for item in value.split(sep) if item.strip()]
        mode_to_skill: dict[str, str] = {}
        for name in [sequence.get("primary"), *(sequence.get("skills") or [])]:
            raw = skills.get(str(name), {})
            baton = raw.get("baton") or {}
            mode_to_skill[str(baton.get("mode", name))] = str(name)
        nodes: list[SkillNode] = []
        for index, token in enumerate(tokens):
            mode = _mode(token, kind)
            nodes.append(SkillNode(f"n{index:02d}_{mode}", mode_to_skill.get(mode, ""), mode, token))
        edges: list[SkillEdge] = []
        for source, target in zip(nodes, nodes[1:]):
            previous = skills.get(source.skill, {})
            incoming = skills.get(target.skill, {})
            pb = previous.get("baton") or {}
            ib = incoming.get("baton") or {}
            if ib.get("blend") == "solo_swap":
                handover = "solo"
            elif pb.get("attractor", "locomotion") == "stand" and ib.get("attractor", "locomotion") == "locomotion":
                handover = "cold"
            else:
                handover = "warm"
            moving = previous.get("motion_class") in {"cyclic", "sequence"}
            gate = "support-window" if moving else "immediate"
            pre = ("support-stable",) if gate == "support-window" else ()
            edges.append(SkillEdge(
                source.id, target.id, f"complete:{source.command}", handover, gate,
                preconditions=pre, postconditions=(f"mode:{target.mode}",),
                recovery="stand-or-hold" if "stand" in mode_to_skill else "hold-last-safe",
            ))
        entry = nodes[0].id if nodes else ""
        return cls(
            name=str(sequence.get("name", "")), entry=entry, nodes=tuple(nodes),
            edges=tuple(edges), arbiter_kind=kind, source_value=value,
            metadata={"robot": sequence.get("robot"), "class": sequence.get("class")},
        )

    def compile_legacy(self) -> dict[str, str]:
        """Compile to the exact current BATON contract; no deploy bytes change."""
        sep = "," if self.arbiter_kind == "schedule" else ";"
        return {"kind": self.arbiter_kind, "value": sep.join(n.command for n in self.nodes)}

    def validate(self) -> list[str]:
        issues: list[str] = []
        if self.schema != 1:
            issues.append(f"skill graph schema must be 1, got {self.schema!r}")
        if not self.name or not self.nodes or not self.entry:
            issues.append("graph requires a name, entry, and at least one node")
        ids = [node.id for node in self.nodes]
        if len(ids) != len(set(ids)):
            issues.append("graph node ids must be unique")
        if self.entry and self.entry not in ids:
            issues.append(f"entry node {self.entry!r} is missing")
        for node in self.nodes:
            if not node.skill:
                issues.append(f"node {node.id}: mode {node.mode!r} has no skill binding")
        expected = list(zip(ids, ids[1:]))
        actual = [(edge.source, edge.target) for edge in self.edges]
        if actual != expected:
            issues.append("compatibility graph must have one ordered edge between every command")
        if self.compile_legacy().get("value") != self.source_value:
            issues.append("graph does not round-trip to the legacy arbiter value")
        return issues

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": self.schema, "name": self.name, "entry": self.entry,
            "arbiter": {"kind": self.arbiter_kind, "value": self.source_value},
            "nodes": [asdict(node) for node in self.nodes],
            "edges": [asdict(edge) for edge in self.edges],
            "metadata": self.metadata,
        }
