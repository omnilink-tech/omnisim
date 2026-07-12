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

"""Patrol Husky — OmniLink agent runner.

Thin runtime built on the agents/production/_lib SDK (OmniLinkAgentRunner):
the HTTP tool server, profile push, usage metering, and main loop come from
the shared runner; this module supplies the Patrol-specific tool dispatch,
result classification, and status snapshot.

Run:

    set OMNI_KEY=olink_YOUR_KEY_HERE
    python agents/production/warehouse_patrol/warehouse_patrol_agent.py

The Patrol Husky runner pushes its profile to OmniLink and starts a
tool-callback HTTP server. Two instances run side-by-side for the
two-husky patrol demo: the north runner listens on 51530 and talks to
the bridge at 6070; the south runner listens on 51531 and talks to
6080. Sector assignment + bridge URL come from PATROL_SECTOR and
PATROL_BRIDGE_URL env vars set per-runner.

Bridges (configured in warehouse_patrol.wbt):
    127.0.0.1:6070  husky_omnilink_bridge for HUSKY_NORTH
    127.0.0.1:6071  husky_eye sidecar for HUSKY_NORTH
    127.0.0.1:6080  husky_omnilink_bridge for HUSKY_SOUTH
    127.0.0.1:6081  husky_eye sidecar for HUSKY_SOUTH

Both bridges expose the same /state, /action, /scan, /solid surface;
they just track different huskies.

Environment:
    OMNI_KEY                       — required.
    PATROL_SECTOR                  — "north" (default) or "south". Surfaced
                                     on /status so chat_drive scripts can
                                     identify which runner they're talking to.
    PATROL_BRIDGE_URL              — bridge URL for this runner's husky
                                     (default http://127.0.0.1:6070 = north).
                                     South runner overrides to 6080.
    PATROL_PORT                    — tool-server port (default 51530).
                                     South runner overrides to 51531.
    PATROL_DRY_RUN                 — "0" to execute live (default), any
                                     other value logs guarded intent only.
    PATROL_OMNILINK_LIB            — override path to omnilink-lib/src.
    PATROL_TOOL_DESCRIPTIONS       — "lean" (default) or "full".
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Dict, List

_THIS = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS))                 # tools/
sys.path.insert(0, str(_THIS.parent))          # agents/production/_lib

from _lib import OmniLinkAgentRunner            # noqa: E402
from tools import ToolSpec, load_all            # noqa: E402
from tools._base import GUARDED, SAFE           # noqa: E402

DRY_RUN = os.environ.get("PATROL_DRY_RUN", "0").strip() not in ("0", "false", "no", "")
# PATROL_SECTOR scopes the agent to a north or south sector. Surfaced on
# /status so chat_drive can identify which runner it's talking to.
PATROL_SECTOR = os.environ.get("PATROL_SECTOR", "north").strip().lower()
if PATROL_SECTOR not in ("north", "south"):
    print(f"WARNING: PATROL_SECTOR={PATROL_SECTOR!r} is not 'north' or 'south' — sweep_summary calls will likely error")
# Lean tool descriptions by default — first-sentence only, ~50% smaller
# system instruction. Same lever as Husky Maze's HUSKY_AGENT_TOOL_DESCRIPTIONS.
_TOOL_DESC_MODE = os.environ.get("PATROL_TOOL_DESCRIPTIONS", "lean").strip() or "lean"

REGISTRY: Dict[str, ToolSpec] = load_all()
SAFE_TOOLS = {n for n, s in REGISTRY.items() if s.tier == SAFE}
GUARDED_TOOLS = {n for n, s in REGISTRY.items() if s.tier == GUARDED}
QUERY_TOOLS: List[Dict[str, Any]] = [s.to_query_tool(mode=_TOOL_DESC_MODE) for s in REGISTRY.values()]


# Wire recall to its dependencies (knowledge + local memory backends).
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
    """Map a tool result to (kind, one-line detail) for the activity feed."""
    if not isinstance(result, dict):
        return "info", f"{tool} -> {result!r}"[:120]
    if result.get("error"):
        return "warning", f"{tool} -> error: {result['error']}"[:120]
    if tool == "drive_to_waypoint":
        done = result.get("done")
        dist = result.get("distance_remaining_m", "?")
        kind = "success" if done else "warning"
        return kind, (
            f"drive_to_waypoint -> done={done} "
            f"x={args.get('x')} y={args.get('y')} dist_remaining={dist}"
        )
    if tool == "complete_mission":
        return "success", f"complete_mission: {args.get('rationale','')[:80]}"
    if tool == "get_state":
        return "info", (
            f"get_state: pose=({result.get('x',0):+.2f},{result.get('y',0):+.2f}) "
            f"mode={result.get('mode')}"
        )
    if tool == "read_mission_brief":
        return "info", f"read_mission_brief: {(result.get('brief','') or '')[:80]}"
    if tool == "get_capabilities":
        return "info", f"get_capabilities: world='{result.get('world_title')}'"
    if tool == "read_camera":
        b64 = result.get("image_base64") or ""
        pose = result.get("tracking_pose") or {}
        return "info", (
            f"read_camera: {result.get('width')}x{result.get('height')} "
            f"png ({len(b64)//1024} KB b64) from pose=("
            f"{pose.get('x',0):+.2f},{pose.get('y',0):+.2f})"
        )
    return "info", f"{tool}: ok"


def status_snapshot(activity_log: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Patrol-specific /status payload. The runner injects `usage` itself."""
    last_state = None
    last_caps = None
    last_action = None
    completes = 0
    waypoints_reached = 0
    sweeps_run = 0
    diffs_run = 0
    for entry in reversed(activity_log[-50:]):
        d = entry.get("data") or {}
        tool = d.get("tool")
        result = d.get("result") or {}
        if last_action is None:
            last_action = {
                "tool": tool, "kind": entry.get("kind"),
                "detail": entry.get("detail"), "timestamp": entry.get("timestamp"),
            }
        if tool == "get_state" and last_state is None:
            last_state = result
        if tool == "get_capabilities" and last_caps is None:
            last_caps = result
        if tool == "complete_mission":
            completes += 1
        if tool == "drive_to_waypoint" and result.get("done"):
            waypoints_reached += 1
        if tool == "sweep_summary":
            sweeps_run += 1
        if tool == "diff_sweeps":
            diffs_run += 1
    return {
        "agent": "Patrol Husky",
        "sector": PATROL_SECTOR,
        "bridge_url": os.environ.get("PATROL_BRIDGE_URL", "http://127.0.0.1:6070"),
        "sweeps_this_session": sweeps_run,
        "diffs_this_session": diffs_run,
        "tools_registered": len(REGISTRY),
        "world_title": (last_caps or {}).get("world_title"),
        "current_pose": ({
            "x": (last_state or {}).get("x"),
            "y": (last_state or {}).get("y"),
            "yaw": (last_state or {}).get("yaw"),
        } if last_state else None),
        "mode": (last_state or {}).get("mode"),
        "waypoints_reached_this_session": waypoints_reached,
        "complete_calls_this_session": completes,
        "last_action": last_action,
        "activity_log_size": len(activity_log),
    }


if __name__ == "__main__":
    OmniLinkAgentRunner(
        agent_name="Patrol Husky",
        profile_path=_THIS / "profile.json",
        port=51530,
        dispatch=dispatch,
        query_tools=QUERY_TOOLS,
        port_env="PATROL_PORT",
        lib_env="PATROL_OMNILINK_LIB",
        dry_run_env="PATROL_DRY_RUN",
        classify_result=classify_result,
        status_snapshot=status_snapshot,
    ).run()
