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

"""Tool sets as declarative, diffable manifests (AgentBench SPEC 4.1/4.2).

The fairness mechanism of the whole benchmark is that the ``shell`` tool set
is **byte-identical for every simulator**. An assertion in prose does not
establish that; a hash does. So a tool set is a *manifest* -- a list of
``{name, description, input_schema}`` plus an environment policy -- and it
carries two hashes:

``tools_sha256``
    Over the tool definitions **only** (name, description, schema, in declared
    order). This is the cross-simulator equality check: the ``shell`` manifest
    must produce the same ``tools_sha256`` on OmniSim, Gazebo, Webots and
    Isaac, or the run is not comparable. Nothing simulator-specific may enter
    a hashed field.

``manifest_sha256``
    Over the whole dump *including* the environment policy and notes. This is
    what a result row records, because "same tools, different environment" is
    still a different condition and a reviewer must be able to see it.

The environment policy is deliberately **outside** ``tools_sha256`` and
deliberately **inside** ``manifest_sha256``: ``OMNISIM_HOME`` is inherent to
"the simulator is installed in the image" and cannot be identical across
simulators, but it is also not free -- it must be visible and diffable.

Adding a ``gazebo`` or ``webots`` tool set is a new module that builds a
``ToolSet`` and registers it; the loop, the budget, the trace and the
manifest hashing are untouched.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from typing import Any, Callable


def canonical_json(obj) -> str:
    """The one serialization used for every hash in this package.

    Sorted keys, no incidental whitespace, unicode preserved. Anything that
    hashes a dict goes through here so a hash can be recomputed by hand.
    """
    return json.dumps(obj, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False)


def sha256_of(obj) -> str:
    return hashlib.sha256(canonical_json(obj).encode("utf-8")).hexdigest()


@dataclass
class ToolSpec:
    """One tool: what the model sees, plus how we execute it.

    ``handler`` is **not** part of any hash and never serialized -- two
    simulators may implement ``run_shell`` differently (different shell
    binary, different container) while presenting a byte-identical definition
    to the model, and that is exactly the property we want to be able to
    assert.
    """

    name: str
    description: str
    input_schema: dict
    handler: Callable[..., Any] | None = None
    # Free-form provenance: where this definition came from. Recorded in the
    # dump so a reviewer can tell a hand-written tool from a generated one.
    source: str = "handwritten"

    def definition(self) -> dict:
        """The model-facing definition -- the only part that is hashed."""
        return {"name": self.name, "description": self.description,
                "input_schema": self.input_schema}

    def as_anthropic_tool(self) -> dict:
        """Anthropic Messages API tool shape (identical to ``definition``)."""
        return self.definition()


@dataclass
class ToolSet:
    """A named, hashable collection of tools plus its environment policy."""

    id: str
    description: str
    specs: list[ToolSpec] = field(default_factory=list)
    # What the agent's subprocesses are allowed to see. Keys only for secrets
    # -- see isolation.py; values here are literal and safe to publish.
    env_policy: dict = field(default_factory=dict)
    # Honest caveats: anything the benchmark does that a user of the shipped
    # surface would not get. Hashed into manifest_sha256.
    notes: list[str] = field(default_factory=list)

    # -- lookup ----------------------------------------------------------
    def __len__(self):
        return len(self.specs)

    @property
    def names(self) -> list[str]:
        return [s.name for s in self.specs]

    def get(self, name) -> ToolSpec | None:
        for s in self.specs:
            if s.name == name:
                return s
        return None

    # -- the model-facing surface ---------------------------------------
    def definitions(self) -> list[dict]:
        return [s.definition() for s in self.specs]

    def as_anthropic_tools(self) -> list[dict]:
        return [s.as_anthropic_tool() for s in self.specs]

    def tool_surface_text(self) -> str:
        """The factual "here is your tool surface" block (SPEC 4.4).

        The *only* per-simulator text allowed in the system prompt, and it is
        generated from the manifest rather than written by hand so it cannot
        drift into per-simulator prompt tuning.
        """
        lines = []
        for s in self.specs:
            first = s.description.strip().splitlines()[0].strip()
            lines.append("  - %s: %s" % (s.name, first))
        return "\n".join(lines)

    # -- hashing / dumping ----------------------------------------------
    @property
    def tools_sha256(self) -> str:
        """Hash over tool definitions only -- the cross-sim fairness hash."""
        return sha256_of(self.definitions())

    def to_json(self, *, include_manifest_sha=True) -> dict:
        out = {
            "schema": "agentbench/tool_set/v1",
            "tool_set": self.id,
            "description": self.description,
            "n_tools": len(self.specs),
            "tools": self.definitions(),
            "tool_sources": {s.name: s.source for s in self.specs},
            "tools_sha256": self.tools_sha256,
            "env_policy": self.env_policy,
            "notes": list(self.notes),
        }
        if include_manifest_sha:
            out["manifest_sha256"] = sha256_of(out)
        return out

    def dump(self, path) -> str:
        """Write the manifest to ``path``; returns ``manifest_sha256``."""
        from pathlib import Path
        payload = self.to_json()
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False)
                              + "\n", encoding="utf-8")
        return payload["manifest_sha256"]

    @property
    def manifest_sha256(self) -> str:
        return self.to_json()["manifest_sha256"]
