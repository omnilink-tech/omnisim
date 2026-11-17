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

"""Unified recall across axis's memory tiers (shim).

Fusion logic is shared in `_lib.agent_recall`; the agent wires the two backend
impls onto the module globals below at startup and this shim forwards them.
"""

from __future__ import annotations

from typing import Any, Dict

from _lib.agent_recall import recall as _recall

from ._base import ALWAYS, SAFE, ToolSpec

# Wired by the agent at startup (see <agent>_agent.py).
SEARCH_KNOWLEDGE_IMPL: Any = None
SEARCH_LOCAL_MEMORY_IMPL: Any = None


def _impl_recall(
    query: str = "",
    limit_per_tier: int = 5,
    **_: Any,
) -> Dict[str, Any]:
    return _recall(
        query=query,
        limit_per_tier=limit_per_tier,
        search_knowledge_impl=SEARCH_KNOWLEDGE_IMPL,
        search_local_memory_impl=SEARCH_LOCAL_MEMORY_IMPL,
        empty_query_hint="Pass the topic you want to look up, e.g. 'omniarm6_01 home pose' or 'IK_MAX_DQ override'.",
    )


SPEC = ToolSpec(
    name="recall",
    tier=SAFE,
    surface=ALWAYS,
    description=(
        "Unified retrieval across Axis's memory tiers — curated knowledge "
        "folder (robot specs, bridge schemas) and local long-term memory "
        "(calibrations, operator overrides, failure signatures). Use this "
        "INSTEAD of search_knowledge or search_local_memory whenever you "
        "need to answer a factual question about a robot, a capability "
        "record, or a deployment detail. Returns ranked hits from each "
        "tier labeled by source, plus a `review_for_contradictions` flag "
        "when multiple tiers answered. If an operator-authored memory "
        "tightens a default from the knowledge folder, surface the "
        "disagreement and confirm before proceeding."
    ),
    parameters={
        "type": "object",
        "properties": {
            "query": {"type": "string", "description": "The topic you want to look up across all memory tiers."},
            "limit_per_tier": {"type": "integer", "description": "Max results from each of knowledge and long_term tiers.", "default": 5},
        },
        "required": ["query"],
    },
    impl=_impl_recall,
)
