# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Husky Maze — OmniLink maze-solving agent runner.

Thin runtime built on the agents/production/_lib SDK (OmniLinkAgentRunner):
the HTTP tool server, profile push, usage metering, and main loop come from
the shared runner; this module supplies the maze-specific tool dispatch,
result classification, and status snapshot.

The agent ships in three variants that share this runner:
  v1 - cell-by-cell, agent owns turn/snap/drive/snap (~290 LLM round-trips)
  v2 - blocking goto_cell, bridge-side auto-snap (~75 round-trips)
  v3 - execute_path batch driver, single LLM round-trip drives 72 cells
Each variant's profile + system prompt live under variants/<name>/. Pick one
with --variant or HUSKY_VARIANT (default v1). Each variant defaults to a
distinct port (51517/51518/51519) so all three runners can coexist.

Run:

    export OMNI_KEY="olink_YOUR_KEY_HERE"
    python agents/production/husky_maze/husky_maze_agent.py --variant v3

Environment:
    OMNI_KEY                       - required.
    HUSKY_VARIANT                  - v1 | v2 | v3 (default v1; --variant wins).
    HUSKY_AGENT_PORT               - tool-server port (default per variant).
    HUSKY_AGENT_DRY_RUN            - "1" to log guarded tools without executing.
    HUSKY_AGENT_OMNILINK_LIB       - path to omnilink-lib/src (else auto-detected).
    HUSKY_AGENT_TOOL_DESCRIPTIONS  - "lean" (default) or "full".
    HUSKY_BRIDGE_URL               - bridge URL surfaced on /status.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

_THIS = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS))                 # tools/
sys.path.insert(0, str(_THIS.parent))          # agents/production/_lib

from _lib import OmniLinkAgentRunner, locate_omnilink_lib  # noqa: E402

locate_omnilink_lib(env_var="HUSKY_AGENT_OMNILINK_LIB")

from tools import ToolSpec, load_all                    # noqa: E402
from tools._base import ALWAYS, GUARDED, SAFE           # noqa: E402

ENGINE = "g1-engine"
DRY_RUN = os.environ.get("HUSKY_AGENT_DRY_RUN", "0").strip() not in ("0", "false", "no", "")
_TOOL_DESC_MODE = os.environ.get("HUSKY_AGENT_TOOL_DESCRIPTIONS", "lean").strip() or "lean"


def _parse_variant() -> str:
    v = ""
    for i, a in enumerate(sys.argv):
        if a == "--variant" and i + 1 < len(sys.argv):
            v = sys.argv[i + 1]
            break
        if a.startswith("--variant="):
            v = a.split("=", 1)[1]
            break
    if not v:
        v = os.environ.get("HUSKY_VARIANT", "v1").strip()
    if v not in {"v1", "v2", "v3"}:
        raise SystemExit(f"Unknown variant {v!r}. Use one of v1, v2, v3.")
    return v


VARIANT = _parse_variant()
VARIANT_DIR = _THIS / "variants" / VARIANT
PROFILE_PATH = VARIANT_DIR / "profile.json"
_DEFAULT_PORTS = {"v1": 51517, "v2": 51518, "v3": 51519}
DEFAULT_PORT = int(os.environ.get("HUSKY_AGENT_PORT", str(_DEFAULT_PORTS[VARIANT])))

REGISTRY: Dict[str, ToolSpec] = load_all()
SAFE_TOOLS = {n for n, s in REGISTRY.items() if s.tier == SAFE}
GUARDED_TOOLS = {n for n, s in REGISTRY.items() if s.tier == GUARDED}
# Surface filter: only tools marked surface=ALWAYS are advertised to the
# planner. ON_DEMAND tools (e.g. read_camera, 30-70K input tokens/call) stay
# dispatchable but invisible — operators can still POST them for diagnostics.
QUERY_TOOLS: List[Dict[str, Any]] = [
    s.to_query_tool(mode=_TOOL_DESC_MODE)
    for s in REGISTRY.values()
    if s.surface == ALWAYS
]


# Wire recall to its dependencies (knowledge + local long-term memory).
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
        if "unreachable" in str(err).lower() or "fault" in str(err).lower() or "crash" in str(err).lower():
            kind = "critical"
        return kind, f"{tool} -> error: {err}"

    if tool == "get_state":
        x = result.get("x"); y = result.get("y"); yaw = result.get("yaw")
        mode = result.get("mode"); fault = result.get("fault")
        goal = result.get("goal_reached")
        kind = "critical" if fault else ("success" if goal else "info")
        bits = [f"pose=({x:+.2f},{y:+.2f}) yaw={yaw:+.2f}" if all(v is not None for v in (x, y, yaw)) else "pose=?"]
        bits.append(f"mode={mode}")
        if fault:
            bits.append(f"FAULT={fault}")
        if goal:
            bits.append("GOAL_REACHED")
        return kind, f"get_state: {' | '.join(bits)}"

    if tool == "get_capabilities":
        wt = result.get("world_title", "?")
        ma = result.get("map_available")
        return "info", f"get_capabilities: world={wt!r} map_available={ma}"

    if tool == "try_get_known_map":
        if result.get("available"):
            cells = len(result.get("adjacency") or {})
            return "info", f"try_get_known_map: available, {cells} cells"
        return "info", f"try_get_known_map: unavailable ({result.get('hint', '')[:60]})"

    if tool == "read_lidar":
        ranges = result.get("ranges_m") or []
        nclear = sum(1 for r in ranges if r > 2.4)
        return "info", f"read_lidar: {len(ranges)} rays, {nclear} cardinal-style clears (>2.4m)"

    if tool == "goto_cell":
        col = args.get("col"); row = args.get("row")
        return "info", f"goto_cell -> ({col},{row}) target=({result.get('x')},{result.get('y')})"

    if tool == "drive_forward":
        return "info", f"drive_forward: distance={args.get('distance')}m at speed={result.get('speed_m_s', '?'):.2f}m/s"

    if tool == "turn":
        deg = (args.get('angle') or 0) * 180.0 / 3.141592653589793
        return "info", f"turn: {deg:+.0f}°"

    if tool == "snap_to_cell":
        snapped = result.get("snapped_to") or {}
        return "info", f"snap_to_cell: ({snapped.get('col')},{snapped.get('row')}) yaw={snapped.get('yaw'):+.2f}"

    if tool == "stop_husky":
        return "warning", "stop_husky: emergency halt"

    if tool == "reset_husky":
        return "warning", "reset_husky: teleport to start"

    if tool == "save_local_memory":
        return "info", f"save_local_memory: {args.get('title','')!r:.60} (id={result.get('id','')[:20]})"

    if tool == "search_local_memory" or tool == "recall":
        n = result.get("count") or sum(result.get("counts", {}).values()) if isinstance(result.get("counts"), dict) else result.get("count", 0)
        return "info", f"{tool}: {n} hits for {args.get('query', '')!r:.60}"

    return "info", f"{tool}: ok"


def _infer_strategy(caps: Optional[Dict[str, Any]]) -> Optional[str]:
    if not caps:
        return None
    if caps.get("map_available"):
        return "BFS over try_get_known_map adjacency"
    return "lidar wall-follow (no map)"


def status_snapshot(activity_log: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Compress the recent activity log into an at-a-glance status. Runner adds usage."""
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
        "agent": "Husky Maze",
        "variant": VARIANT,
        "bridge_url": os.environ.get("HUSKY_BRIDGE_URL", "http://127.0.0.1:6070"),
        "tools_registered": len(REGISTRY),
        "world_title": (last_caps or {}).get("world_title"),
        "map_available": (last_caps or {}).get("map_available"),
        "strategy": _infer_strategy(last_caps),
        "current_pose": (
            {
                "x": (last_state or {}).get("x"),
                "y": (last_state or {}).get("y"),
                "yaw": (last_state or {}).get("yaw"),
            } if last_state else None
        ),
        "mode": (last_state or {}).get("mode"),
        "goal_reached": (last_state or {}).get("goal_reached"),
        "last_fault": last_fault,
        "last_action": last_action,
        "activity_log_size": len(activity_log),
    }


if __name__ == "__main__":
    OmniLinkAgentRunner(
        agent_name="Husky Maze",
        profile_path=PROFILE_PATH,
        port=DEFAULT_PORT,
        dispatch=dispatch,
        query_tools=QUERY_TOOLS,
        port_env="HUSKY_AGENT_PORT",
        lib_env="HUSKY_AGENT_OMNILINK_LIB",
        dry_run_env="HUSKY_AGENT_DRY_RUN",
        classify_result=classify_result,
        status_snapshot=status_snapshot,
    ).run()
