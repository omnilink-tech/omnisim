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

"""Foreman-only meta-tools: read_mission_brief + complete_mission.

The Foreman is an orchestrator — it doesn't drive robots itself, so its
own tool surface is small:

  - read_mission_brief reads the warehouse world's mission brief from
    husky_omnilink_bridge so the Foreman can confirm the operator's
    request before delegating. Same bridge endpoint as the Picker uses;
    the Foreman just gets to read it without going through the Picker.

  - complete_mission records that the Foreman considers the operator's
    mission satisfied, mirrors the captain's claim primitive: no bridge
    side-effect, just appends to the local activity feed for audit.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List

from ._base import ALWAYS, GUARDED, SAFE, ToolSpec, bridge_get


_LAST_CLAIMS: List[Dict[str, Any]] = []


def _impl_read_mission_brief(**_: Any) -> Dict[str, Any]:
    """Operator's free-form mission brief, read from WorldInfo.info on
    the warehouse bridge. Same endpoint the Picker reads — the Foreman
    just gets it directly so it can decide WHICH specialist to delegate
    to without burning a delegation round-trip just to learn the task."""
    return bridge_get("mission")


def _impl_complete_mission(
    rationale: str = "",
    legs: Any = None,
    **_: Any,
) -> Dict[str, Any]:
    if not rationale or not rationale.strip():
        return {"error": "rationale is required: a one-sentence summary of what the operator asked + how the legs satisfied it"}
    claim = {
        "timestamp": time.time(),
        "rationale": rationale.strip(),
        "legs": legs or [],
    }
    _LAST_CLAIMS.append(claim)
    return {
        "status": "ok",
        "foreman_mission_complete": True,
        "claim": claim,
        "total_claims_this_session": len(_LAST_CLAIMS),
    }


SPECS = [
    ToolSpec(
        name="read_mission_brief",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Read the operator's mission brief from the warehouse "
            "world's WorldInfo.info — tells you which colour pallet to "
            "fetch and where to deliver it. Always call this first so "
            "you can decompose into the right specialist task before "
            "delegating."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_read_mission_brief,
        tags=["mission"],
    ),
    ToolSpec(
        name="complete_mission",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Record that the Foreman considers the operator's mission "
            "satisfied. Pass `rationale` (one-sentence summary of "
            "operator's goal + the legs that satisfied it) and optional "
            "`legs` (list of {specialist, task, success, summary}). The "
            "Foreman has no bridge to claim against — this just appends "
            "to the local activity feed and increments the session-claim "
            "counter so the operator can audit. Call exactly once per "
            "operator mission, AFTER every delegated leg returned "
            "success. Never claim if any leg failed."
        ),
        parameters={
            "type": "object",
            "properties": {
                "rationale": {
                    "type": "string",
                    "description": "One-sentence summary: what did the operator ask + how did the legs satisfy it.",
                },
                "legs": {
                    "type": "array",
                    "items": {"type": "object"},
                    "description": "Optional: list of {specialist, task, success, summary} for the audit log.",
                },
            },
            "required": ["rationale"],
        },
        impl=_impl_complete_mission,
        tags=["mission"],
    ),
]
