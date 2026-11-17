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

"""Shared unified-recall implementation for production agents.

The single body behind every agent's ``tools/recall.py`` shim. One call
queries the curated knowledge folder AND the agent's local long-term memory,
labels each hit by source, and flags contradictions when both tiers answer.

The two backend impls are passed in (rather than read from module globals as
the per-agent copies did) so this stays stateless; each shim keeps its
``SEARCH_KNOWLEDGE_IMPL`` / ``SEARCH_LOCAL_MEMORY_IMPL`` module globals (wired
by the agent at startup) and forwards them here at call time.
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional


_DEFAULT_EMPTY_QUERY_HINT = (
    "Pass the topic you want to look up, e.g. 'ur5e_01 home pose' or 'IK_MAX_DQ override'."
)


def recall(
    query: str = "",
    limit_per_tier: int = 5,
    search_knowledge_impl: Optional[Callable[..., Any]] = None,
    search_local_memory_impl: Optional[Callable[..., Any]] = None,
    empty_query_hint: str = _DEFAULT_EMPTY_QUERY_HINT,
    **_: Any,
) -> Dict[str, Any]:
    if not query:
        return {
            "error": "query is required",
            "hint": empty_query_hint,
        }

    knowledge_hits: List[Dict[str, Any]] = []
    if search_knowledge_impl is not None:
        try:
            kb = search_knowledge_impl(query=query, max_results=int(limit_per_tier))
            if isinstance(kb, dict) and kb.get("status") == "ok":
                knowledge_hits = [
                    {
                        "source": f"knowledge/{h['file']}#{h.get('heading') or 'chunk'}",
                        "lines": f"{h.get('start_line')}-{h.get('end_line')}",
                        "text": h.get("text", ""),
                        "score": h.get("score", 0),
                    }
                    for h in kb.get("hits", [])
                ]
        except Exception:
            pass

    local_hits: List[Dict[str, Any]] = []
    if search_local_memory_impl is not None:
        try:
            local_hits = search_local_memory_impl(query, int(limit_per_tier)) or []
        except Exception:
            local_hits = []

    review_for_contradictions = bool(knowledge_hits) and bool(local_hits)

    hint_lines = [
        "Knowledge (curated markdown) is the authoritative tier when it has a hit — robot specs, bridge schemas, capability records.",
        "Local long-term memory holds notes Axis wrote itself — calibrations, operator overrides, failure signatures. Trust these, but verify against knowledge when they conflict.",
        "Short-term memory is already in this conversation's context — cross-check it against the other tiers yourself.",
    ]
    if review_for_contradictions:
        hint_lines.append(
            "Both knowledge and local long-term memory returned hits. Compare them. "
            "If they disagree (e.g. a capability record here says IK_MAX_DQ=0.08 but "
            "an operator-authored memory tightens it to 0.05), the operator override "
            "in long-term memory is usually what the operator wants enforced — raise "
            "the disagreement explicitly and confirm which applies for this motion."
        )

    return {
        "status": "ok",
        "query": query,
        "tiers": {
            "knowledge": knowledge_hits,
            "long_term": local_hits,
            "short_term": {
                "note": "Already present in this conversation's context. Cross-check what you've just read against the knowledge and long_term tiers."
            },
        },
        "counts": {
            "knowledge": len(knowledge_hits),
            "long_term": len(local_hits),
        },
        "review_for_contradictions": review_for_contradictions,
        "hint": " ".join(hint_lines),
    }
