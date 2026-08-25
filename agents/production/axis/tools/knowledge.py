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

"""Search axis's curated knowledge folder (shim).

Scan/chunk/score logic is shared in `_lib.agent_knowledge`; this module binds
it to this agent's `knowledge/` folder and declares the tool spec.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

from _lib.agent_knowledge import search_knowledge as _search_knowledge

from ._base import ALWAYS, SAFE, ToolSpec

_KB_ROOT = Path(__file__).resolve().parent.parent / "knowledge"


def _impl_search_knowledge(
    query: str = "",
    max_results: int = 8,
    max_snippet_chars: int = 800,
    **_: Any,
) -> Dict[str, Any]:
    return _search_knowledge(
        _KB_ROOT,
        query=query,
        max_results=max_results,
        max_snippet_chars=max_snippet_chars,
    )


SPEC = ToolSpec(
    name="search_knowledge",
    tier=SAFE,
    surface=ALWAYS,
    description=(
        "Search Axis's curated knowledge folder (agents/axis/knowledge/) — "
        "source-controlled authoritative reference material about robots, the "
        "OmniSim bridge, capability records, and OmniLink architecture. CALL "
        "THIS any time an operator asks about a robot's limits, home pose, IK "
        "constants, the bridge's endpoint shapes, or anything domain-specific "
        "that isn't part of common knowledge. Pass an empty query to sample "
        "what's in the folder. Curated files are the source of truth; long-term "
        "memory is agent-written and may miss or stale. Returns {file, heading, "
        "start_line, end_line, text, score}."
    ),
    parameters={
        "type": "object",
        "properties": {
            "query": {"type": "string", "description": "Keywords describing what you're looking for."},
            "max_results": {"type": "integer", "description": "Max chunks to return.", "default": 8},
            "max_snippet_chars": {"type": "integer", "description": "Cap per-chunk text length.", "default": 800},
        },
    },
    impl=_impl_search_knowledge,
)
