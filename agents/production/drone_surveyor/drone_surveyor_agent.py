# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Drone Surveyor — OmniLink aerial-survey agent runner.

Thin runtime built on the agents/production/_lib SDK (OmniLinkAgentRunner):
the HTTP tool server, profile push, usage metering, and main loop come from
the shared runner; this module supplies the drone-specific tool dispatch,
result classification, and status snapshot.

Run:

    export OMNI_KEY="olink_YOUR_KEY_HERE"
    python agents/production/drone_surveyor/drone_surveyor_agent.py

Environment:
    OMNI_KEY                       - required.
    DRONE_AGENT_DRY_RUN            - "1" to log guarded tools without executing.
    DRONE_AGENT_PORT               - tool-callback server port (default 51524).
    DRONE_AGENT_OMNILINK_LIB       - path to omnilink-lib/src; auto-detected
                                     among OmniSim siblings otherwise.
    DRONE_AGENT_TOOL_DESCRIPTIONS  - "lean" (first-sentence) or "full". Default lean.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Dict, List

_THIS = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS))                 # tools/
sys.path.insert(0, str(_THIS.parent))          # agents/production/_lib

from _lib import OmniLinkAgentRunner, locate_omnilink_lib  # noqa: E402

locate_omnilink_lib(env_var="DRONE_AGENT_OMNILINK_LIB")

from tools import ToolSpec, load_all              # noqa: E402
from tools._base import BRIDGE_URL, GUARDED, SAFE  # noqa: E402

DRY_RUN = os.environ.get("DRONE_AGENT_DRY_RUN", "0").strip() not in ("0", "false", "no", "")
_TOOL_DESC_MODE = os.environ.get("DRONE_AGENT_TOOL_DESCRIPTIONS", "lean").strip() or "lean"

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
    if not isinstance(result, dict):
        return "info", f"{tool} -> {result!r:.80}"

    err = result.get("error")
    if err:
        kind = "warning"
        if "unreachable" in str(err).lower() or "fault" in str(err).lower():
            kind = "critical"
        return kind, f"{tool} -> error: {err}"

    if tool == "get_state":
        x = result.get("x"); y = result.get("y"); z = result.get("z")
        mode = result.get("mode"); fault = result.get("fault")
        complete = result.get("mission_complete")
        kind = "critical" if fault else ("success" if complete else "info")
        if all(v is not None for v in (x, y, z)):
            pose = f"pose=({x:+.2f},{y:+.2f},{z:.1f}m)"
        else:
            pose = "pose=?"
        bits = [pose, f"mode={mode}"]
        if fault:
            bits.append(f"FAULT={fault}")
        if complete:
            bits.append("MISSION_COMPLETE")
        return kind, f"get_state: {' | '.join(bits)}"

    if tool == "get_capabilities":
        wt = result.get("world_title", "?")
        alt = result.get("default_takeoff_altitude_m", "?")
        return "info", f"get_capabilities: world={wt!r} default_alt={alt}m"

    if tool == "scan_for_markers":
        markers = result.get("markers") or []
        colors = sorted({m.get("color", "?") for m in markers})
        return "info", f"scan_for_markers: {len(markers)} blobs colors={colors}"

    if tool == "takeoff":
        alt = args.get("altitude") or result.get("target_altitude")
        if result.get("waited"):
            done = result.get("done")
            z = result.get("z")
            return ("success" if done else "warning"), \
                   f"takeoff: target={alt}m done={done} reached_z={z}"
        return "info", f"takeoff: requested altitude={alt}m"

    if tool == "land":
        if result.get("waited"):
            return ("success" if result.get("landed") else "warning"), \
                   f"land: landed={result.get('landed')} z={result.get('z')}"
        return "info", "land: requested"

    if tool == "goto_waypoint":
        if result.get("waited"):
            done = result.get("done")
            d = result.get("distance_remaining_m")
            return ("info" if done else "warning"), \
                   f"goto_waypoint: target=({args.get('x')},{args.get('y')}) done={done} dist_remaining={d}"
        return "info", f"goto_waypoint: target=({args.get('x')},{args.get('y')})"

    if tool == "set_gimbal_pitch":
        return "info", f"set_gimbal_pitch: {args.get('pitch_rad'):.3f} rad"

    if tool == "complete_mission":
        rationale = (args.get("rationale") or "")[:80]
        return "success", f"complete_mission: {rationale!r}"

    if tool == "stop_drone":
        return "warning", "stop_drone: emergency stop"

    if tool == "reset_drone":
        return "warning", "reset_drone: teleport to start"

    if tool == "save_local_memory":
        return "info", f"save_local_memory: {args.get('title','')!r:.60} (id={(result.get('id') or '')[:20]})"

    if tool in ("recall", "search_local_memory"):
        n = result.get("count") or sum((result.get("counts") or {}).values()) if isinstance(result.get("counts"), dict) else result.get("count", 0)
        return "info", f"{tool}: {n} hits for {args.get('query', '')!r:.60}"

    return "info", f"{tool}: ok"


def status_snapshot(activity_log: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Compress recent activity into an at-a-glance status. Runner adds usage."""
    last_state = None
    last_caps = None
    last_fault = None
    last_action = None
    for entry in reversed(activity_log[-50:]):
        d = entry.get("data") or {}
        tool = d.get("tool")
        result = d.get("result") or {}
        if last_action is None:
            last_action = {
                "tool": tool,
                "kind": entry.get("kind"),
                "detail": entry.get("detail"),
                "timestamp": entry.get("timestamp"),
            }
        if tool == "get_state" and last_state is None:
            last_state = result
            if result.get("fault") and last_fault is None:
                last_fault = result.get("fault")
        if tool == "get_capabilities" and last_caps is None:
            last_caps = result
    return {
        "agent": "Drone Surveyor",
        "bridge_url": BRIDGE_URL,
        "tools_registered": len(REGISTRY),
        "world_title": (last_caps or {}).get("world_title"),
        "default_takeoff_altitude_m": (last_caps or {}).get("default_takeoff_altitude_m"),
        "current_pose": (
            {
                "x": (last_state or {}).get("x"),
                "y": (last_state or {}).get("y"),
                "z": (last_state or {}).get("z"),
                "yaw": (last_state or {}).get("yaw"),
            } if last_state else None
        ),
        "mode": (last_state or {}).get("mode"),
        "mission_complete": (last_state or {}).get("mission_complete"),
        "last_fault": last_fault,
        "last_action": last_action,
        "activity_log_size": len(activity_log),
    }


if __name__ == "__main__":
    OmniLinkAgentRunner(
        agent_name="Drone Surveyor",
        profile_path=_THIS / "profile.json",
        port=51524,
        dispatch=dispatch,
        query_tools=QUERY_TOOLS,
        port_env="DRONE_AGENT_PORT",
        lib_env="DRONE_AGENT_OMNILINK_LIB",
        dry_run_env="DRONE_AGENT_DRY_RUN",
        classify_result=classify_result,
        status_snapshot=status_snapshot,
    ).run()
