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

"""Mission Captain — OmniLink orchestration agent runner.

Thin runtime built on the agents/production/_lib SDK (OmniLinkAgentRunner):
the HTTP tool server, profile push, usage metering, and main loop come from
the shared runner; this module supplies the Captain's tool dispatch, result
classification, and status snapshot.

Run:

    set OMNI_KEY=olink_YOUR_KEY_HERE
    python agents/production/mission_captain/mission_captain_agent.py

The captain pushes its profile to OmniLink, starts a tool-callback server
on port 51518 (one above Husky Maze's 51517), and waits.

Specialists (Husky Maze on :51517, Axis on :51516) must already be running
their own runners so the captain's `delegate_to_agent` tool can find their
profiles and invoke their tool servers.

Environment:

    OMNI_KEY                       — required.
    CAPTAIN_PORT                   — tool-server port (default 51518).
    CAPTAIN_DRY_RUN                — "0" to execute live (default), any
                                     other value logs guarded intent only.
    CAPTAIN_OMNILINK_LIB           — override path to omnilink-lib/src.
    CAPTAIN_DEFAULT_ENGINE         — engine for chat() and sub-delegations
                                     (default g1-engine).
    CAPTAIN_SUB_MAX_TURNS          — per-delegation cap (default 30).
    CAPTAIN_SUB_CHAT_TIMEOUT       — per-chat timeout in seconds (default 120).
    HUSKY_MAZE_STATUS_URL          — Husky Maze runner /status URL.
    HUSKY_MAZE_TOOL_URL            — Husky Maze runner /tool URL.
    AXIS_STATUS_URL                — Axis runner /status URL.
    AXIS_TOOL_URL                  — Axis runner /tool URL.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Dict, List

_THIS = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS))                 # tools/
sys.path.insert(0, str(_THIS.parent))          # agents/production/_lib

from _lib import OmniLinkAgentRunner                       # noqa: E402
from tools import ToolSpec, load_all                       # noqa: E402
from tools._base import GUARDED, SAFE, SPECIALIST_REGISTRY  # noqa: E402

DRY_RUN = os.environ.get("CAPTAIN_DRY_RUN", "0").strip() not in ("0", "false", "no", "")

REGISTRY: Dict[str, ToolSpec] = load_all()
SAFE_TOOLS = {n for n, s in REGISTRY.items() if s.tier == SAFE}
GUARDED_TOOLS = {n for n, s in REGISTRY.items() if s.tier == GUARDED}
QUERY_TOOLS: List[Dict[str, Any]] = [s.to_query_tool(mode="full") for s in REGISTRY.values()]


# Wire recall to its dependencies.
try:
    from tools import recall as _recall_mod          # type: ignore
    from tools import local_memory as _local_mem_mod  # type: ignore
    _sk = REGISTRY.get("search_knowledge")
    if _sk is not None:
        _recall_mod.SEARCH_KNOWLEDGE_IMPL = _sk.impl
    _recall_mod.SEARCH_LOCAL_MEMORY_IMPL = _local_mem_mod.search_local_memory_for_recall
except Exception as _exc:
    print(f"  [recall] wire failed: {_exc}")


def dispatch(tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    """Tier-gated tool dispatch. SAFE tools run; GUARDED tools honour DRY_RUN."""
    spec = REGISTRY.get(tool_name)
    if spec is None:
        return {
            "error": f"unknown tool: {tool_name}",
            "hint": "Use only tools from 'known_tools'. Do not invent tool names.",
            "known_tools": sorted(REGISTRY.keys()),
        }
    if spec.tier == SAFE:
        return spec.impl(**args)
    if spec.tier == GUARDED:
        if DRY_RUN:
            print(f"  [DRY_RUN] would execute {tool_name}({args})")
            return {"status": "dry_run", "tool": tool_name, "args": args}
        print(f"  [EXECUTE] {tool_name}({args})")
        return spec.impl(**args)
    return {"error": f"tool {tool_name!r} has unknown tier {spec.tier!r}"}


def classify_result(tool: str, args: Dict[str, Any], result: Any):
    if not isinstance(result, dict):
        return "info", f"{tool} -> {result!r}"[:120]
    err = result.get("error")
    if err:
        return ("warning", f"{tool} -> error: {err}")
    if tool == "delegate_to_agent":
        ok = result.get("success")
        agent = result.get("agent", "?")
        turns = result.get("turns", "?")
        kind = "success" if ok else "warning"
        return kind, (
            f"delegate -> {agent}: success={ok} turns={turns} "
            f"complete={result.get('mission_complete')}"
        )
    if tool == "execute_mission_plan":
        ok = result.get("completed")
        ledger = result.get("ledger") or {}
        return ("success" if ok else "warning"), (
            f"mission plan {result.get('plan_id', '?')}: completed={ok} "
            f"steps={ledger.get('completed_steps', 0)}/"
            f"{ledger.get('requested_steps', 0)}"
        )
    if tool == "list_agents":
        n = result.get("count", 0)
        unreachable = sum(1 for a in result.get("specialists", []) if not a.get("reachable"))
        kind = "warning" if unreachable else "info"
        return kind, f"list_agents: {n} specialists ({unreachable} unreachable)"
    if tool == "query_agent_status":
        return "info", f"query_agent_status: world={result.get('world_title')} mode={result.get('mode')}"
    if tool == "complete_mission":
        return "success", f"captain complete_mission: {args.get('rationale','')[:80]}"
    if tool == "save_local_memory":
        return "info", f"save_local_memory: {args.get('title','')[:60]}"
    return "info", f"{tool}: ok"


def status_snapshot(activity_log: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Captain-specific /status payload. The runner injects `usage` itself."""
    last_action = None
    last_delegation = None
    completes = 0
    for entry in reversed(activity_log[-50:]):
        d = entry.get("data") or {}
        tool = d.get("tool")
        if last_action is None:
            last_action = {
                "tool": tool, "kind": entry.get("kind"),
                "detail": entry.get("detail"), "timestamp": entry.get("timestamp"),
            }
        if tool == "delegate_to_agent" and last_delegation is None:
            last_delegation = d.get("result")
        if tool == "execute_mission_plan" and last_delegation is None:
            last_delegation = d.get("result")
        if tool == "complete_mission":
            completes += 1
    return {
        "agent": "Mission Captain",
        "tools_registered": len(REGISTRY),
        "specialists_known": list(SPECIALIST_REGISTRY.keys()),
        "last_action": last_action,
        "last_delegation": last_delegation,
        "captain_complete_calls_this_session": completes,
        "activity_log_size": len(activity_log),
    }


if __name__ == "__main__":
    OmniLinkAgentRunner(
        agent_name="Mission Captain",
        profile_path=_THIS / "profile.json",
        port=51518,
        dispatch=dispatch,
        query_tools=QUERY_TOOLS,
        port_env="CAPTAIN_PORT",
        lib_env="CAPTAIN_OMNILINK_LIB",
        dry_run_env="CAPTAIN_DRY_RUN",
        classify_result=classify_result,
        status_snapshot=status_snapshot,
    ).run()
