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

"""Local long-term memory for warehouse_patrol (shim).

Hybrid retrieval + storage live in `_lib.agent_memory.LocalMemoryStore`; this
module binds one store to this agent's `long_term_memory/` folder and declares
the tool specs. The module-level ``_impl_*`` aliases are preserved for tools
that import them directly.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List

from _lib.agent_memory import LocalMemoryStore

from ._base import ALWAYS, SAFE, ToolSpec

_MEMORY_ROOT = Path(__file__).resolve().parent.parent / "long_term_memory"
_STORE = LocalMemoryStore(_MEMORY_ROOT)

_impl_save_local_memory = _STORE.save
_impl_search_local_memory = _STORE.search
_impl_list_local_memories = _STORE.list_memories
_impl_forget_local_memory = _STORE.forget


def search_local_memory_for_recall(query: str, limit: int = 5) -> List[Dict[str, Any]]:
    return _STORE.search_for_recall(query, limit)


SPECS = [
    ToolSpec(
        name="save_local_memory",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Persist a durable note into Husky Maze's local long-term "
            "memory. Use for facts that should survive across sessions: "
            "the BFS shortest path you computed for a given world title, "
            "recurring fault signatures (e.g. \"snap_to_cell required after "
            "every pivot or husky wedges on H-walls\"), operator overrides, "
            "world-specific quirks. The note is saved as markdown under "
            "agents/production/husky_maze/long_term_memory/ and indexed with "
            "an embedding (via local Ollama when available; BM25 fallback). "
            "One concrete fact per memory — do not dump whole session logs. "
            "Tag with the world_title so future sessions can find it."
        ),
        parameters={
            "type": "object",
            "properties": {
                "title": {"type": "string", "description": "Short, specific headline — the primary search signal."},
                "body": {"type": "string", "description": "Full note. One fact per memory."},
                "tags": {"type": "array", "items": {"type": "string"}, "description": "Optional tags: world_title, 'plan', 'fault', 'override', etc."},
            },
            "required": ["title", "body"],
        },
        impl=_impl_save_local_memory,
    ),
    ToolSpec(
        name="search_local_memory",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Search Husky Maze's local long-term memory. Default mode is "
            "hybrid — runs vector cosine (via Ollama embeddings) AND BM25 "
            "lexical ranking, then fuses results via Reciprocal Rank "
            "Fusion. Most useful right after get_capabilities: search for "
            "the world_title (e.g. \"Husky Maze (Unknown)\") to see if "
            "you've solved this layout before — if yes, the saved BFS "
            "path beats re-discovering it via lidar. Prefer `recall` for "
            "cross-tier lookups; this is local-memory-only."
        ),
        parameters={
            "type": "object",
            "properties": {
                "query": {"type": "string"},
                "k": {"type": "integer", "default": 5},
                "tags": {"type": "array", "items": {"type": "string"}},
                "mode": {
                    "type": "string",
                    "enum": ["hybrid", "vector", "bm25"],
                    "default": "hybrid",
                },
            },
            "required": ["query"],
        },
        impl=_impl_search_local_memory,
    ),
    ToolSpec(
        name="list_local_memories",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "List recently-updated local memories (most recent first). "
            "Optionally filter by tags (e.g. a robot id). Returns metadata "
            "only — use search_local_memory to get snippets."
        ),
        parameters={
            "type": "object",
            "properties": {
                "tags": {"type": "array", "items": {"type": "string"}},
                "limit": {"type": "integer", "default": 50},
            },
        },
        impl=_impl_list_local_memories,
    ),
    ToolSpec(
        name="forget_local_memory",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Delete a local memory by id. Removes the markdown file and "
            "its index row. Irreversible; only use when the operator "
            "explicitly asks or a saved fact was clearly wrong."
        ),
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
        impl=_impl_forget_local_memory,
    ),
]
