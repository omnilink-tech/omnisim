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

"""Captain-only meta-tool: complete_mission.

The captain doesn't have a robot bridge to claim completion against, so
this tool just records the captain's claim in long-term memory + the
local activity feed. The operator audits via the runner's /activity
endpoint.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List

from ._base import GUARDED, ALWAYS, ToolSpec


_LAST_CLAIMS: List[Dict[str, Any]] = []


def _impl_complete_mission(
    rationale: str = "",
    plan_id: str = "",
    legs: Any = None,
    **_: Any,
) -> Dict[str, Any]:
    if not rationale or not rationale.strip():
        return {"error": "rationale is required: a one-sentence summary of what the operator asked + how the legs satisfied it"}
    if not plan_id or not plan_id.strip():
        return {
            "error": "plan_id is required",
            "hint": (
                "Run execute_mission_plan first. A captain claim must bind "
                "to its mechanically verified delegation ledger."
            ),
        }
    from .orchestration import claim_mission_plan

    # Consuming accessor: verifies AND stamps the plan, so a plan_id can back
    # exactly one claim. Checking `completed and verified` alone let a stale
    # plan_id evidence an unrelated later mission.
    plan, error = claim_mission_plan(plan_id.strip(), rationale=rationale)
    if error is not None:
        return error
    assert plan is not None
    claim = {
        "timestamp": time.time(),
        "rationale": rationale.strip(),
        "plan_id": plan_id.strip(),
        "objective": plan.get("objective"),
        "legs": legs or plan.get("ledger", {}).get("steps", []),
    }
    _LAST_CLAIMS.append(claim)
    return {
        "status": "ok",
        "captain_mission_complete": True,
        "claim": claim,
        "total_claims_this_session": len(_LAST_CLAIMS),
    }


SPEC = ToolSpec(
    name="complete_mission",
    tier=GUARDED,
    surface=ALWAYS,
    description=(
        "Record that the captain considers the operator's mission "
        "satisfied. Pass `rationale` (one-sentence summary of operator's "
        "goal + the legs that satisfied it) and optional `legs` (list of "
        "{specialist, task, success}). The captain has no bridge to "
        "claim against — this just appends to the local activity feed "
        "and increments the session-claim counter so the operator can "
        "audit. Call exactly once per operator mission, AFTER every "
        "delegated leg returned success. `plan_id` is mandatory, must "
        "refer to a completed execute_mission_plan ledger, and is "
        "SINGLE-USE -- a plan_id that already backed a claim is refused "
        "with mission_plan_already_claimed, so every mission needs its own "
        "execute_mission_plan run."
    ),
    parameters={
        "type": "object",
        "properties": {
            "rationale": {
                "type": "string",
                "description": "One-sentence summary: what did the operator ask + how did the legs satisfy it.",
            },
            "plan_id": {
                "type": "string",
                "description": "Verified plan_id returned by execute_mission_plan.",
            },
            "legs": {
                "type": "array",
                "items": {"type": "object"},
                "description": "Optional: list of {specialist, task, success, summary} for the audit log.",
            },
        },
        "required": ["rationale", "plan_id"],
    },
    impl=_impl_complete_mission,
    tags=["mission"],
)
