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

"""Meta-tools for introspecting and searching Axis's tool registry.

When the tool count grows past what fits comfortably in a single chat
system-instruction payload (roughly 50+ tools), the scaling answer is
tool-on-demand lookup: keep a small set of always-on tools plus a search
meta-tool, and let the agent pull specific specs when needed.
`find_tools(query)` is that meta-tool. Mirrors OmniLink's first-party assistant agent's implementation
one-for-one.
"""

from __future__ import annotations

import re
from typing import Any, Dict, List

from ._base import ALWAYS, SAFE, ToolSpec

# Populated by the runner at startup.
REGISTRY_REF: Dict[str, Any] = {}
DISPATCH_REF: Any = None


def _score_spec(spec: Any, query_terms: List[str]) -> int:
    if not query_terms:
        return 1
    name = getattr(spec, "name", "").lower()
    desc = getattr(spec, "description", "").lower()
    tags = " ".join(getattr(spec, "tags", [])).lower()
    score = 0
    for term in query_terms:
        if not term:
            continue
        if term == name:
            score += 10
        elif term in name:
            score += 6
        if term in tags:
            score += 4
        hits = desc.count(term)
        score += min(hits, 3) * 2
    return score


def _impl_find_tools(query: str = "", limit: int = 10, **_: Any) -> Dict[str, Any]:
    registry = REGISTRY_REF or {}
    if not registry:
        return {"status": "ok", "matches": [], "note": "Registry not attached."}

    terms = [t for t in re.split(r"\s+", query.lower()) if t]
    max_limit = max(1, min(int(limit), 50))

    scored: List[tuple[int, Any]] = []
    for spec in registry.values():
        s = _score_spec(spec, terms)
        if terms and s <= 0:
            continue
        scored.append((s, spec))

    scored.sort(key=lambda x: (-x[0], x[1].name))
    top = scored[:max_limit]

    return {
        "status": "ok",
        "query": query,
        "total_registered": len(registry),
        "returned": len(top),
        "matches": [
            {
                "name": s.name,
                "tier": s.tier,
                "description": s.description,
                "parameters": s.parameters,
                "score": score,
            }
            for score, s in top
        ],
    }


def _impl_list_tools(**_: Any) -> Dict[str, Any]:
    registry = REGISTRY_REF or {}
    return {
        "status": "ok",
        "total": len(registry),
        "tools": [
            {
                "name": s.name,
                "tier": s.tier,
                "surface": getattr(s, "surface", "always"),
                "description": s.description[:120],
            }
            for s in sorted(registry.values(), key=lambda x: (x.tier, x.name))
        ],
    }


def _impl_invoke_tool(name: str = "", args: Any = None, **_: Any) -> Dict[str, Any]:
    if not name:
        return {
            "error": "name is required",
            "hint": "Call find_tools first if you don't know the tool name.",
        }
    if name == "invoke_tool":
        return {"error": "cannot invoke_tool(invoke_tool) — call the real tool by name"}
    if DISPATCH_REF is None:
        return {"error": "runner dispatcher not attached — this should not happen in production"}
    if args is None:
        args = {}
    if not isinstance(args, dict):
        return {"error": "args must be an object"}
    return DISPATCH_REF(name, dict(args))


SPECS = [
    ToolSpec(
        name="find_tools",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Search Axis's own tool registry by keyword. Ranks tools by name / "
            "description match and returns the top N with their full descriptions "
            "AND parameter schemas — the parameters shape is what you need to call "
            "invoke_tool correctly. At scale, most tools are on_demand and hidden "
            "from the chat-request manifest; find_tools is how you discover them."
        ),
        parameters={
            "type": "object",
            "properties": {
                "query": {"type": "string", "description": "Keywords describing the task."},
                "limit": {"type": "integer", "description": "Max results.", "default": 10},
            },
        },
        impl=_impl_find_tools,
    ),
    ToolSpec(
        name="list_tools",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "List every tool Axis has registered, grouped by tier / surface. "
            "Use for a complete overview; prefer find_tools(query) when you know "
            "roughly what you're looking for."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_list_tools,
    ),
    ToolSpec(
        name="invoke_tool",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Universal dispatcher for any registered tool. Use to call tools that "
            "aren't in your current manifest (i.e. on_demand tools at scale). Flow: "
            "call find_tools(query) to discover name + parameter schema, then call "
            "invoke_tool(name=..., args={...}) with the exact arg shape. The runner "
            "routes through the same tier rules as a direct call."
        ),
        parameters={
            "type": "object",
            "properties": {
                "name": {"type": "string", "description": "Registered tool name from find_tools / list_tools."},
                "args": {"type": "object", "description": "Arguments matching the tool's parameters schema."},
            },
            "required": ["name"],
        },
        impl=_impl_invoke_tool,
    ),
]
