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

"""Axis — robot-control agent for OmniSim.

Thin runtime built on the agents/production/_lib SDK (OmniLinkAgentRunner):
it auto-discovers tool specs from the `tools/` package, then hands the HTTP
tool server, profile push, usage metering, and main loop to the shared
runner. Every tool lives in its own module under `tools/`; to add a tool,
drop a new file there — no changes needed here.

Run:

    export OMNI_KEY="olink_YOUR_KEY_HERE"
    export AXIS_BRIDGE_URL="http://127.0.0.1:8765"   # OmniSim OmniArm 6 bridge
    # optional: export AXIS_DRY_RUN=1   # log guarded-tool intent only
    python agents/production/axis/axis_agent.py

Then open https://www.omnilink-agents.com, pick "Axis", and chat.

Environment:

    OMNI_KEY            — your Omni Key (required).
    AXIS_BRIDGE_URL     — OmniSim bridge URL (default http://127.0.0.1:8765).
    AXIS_BRIDGE_TIMEOUT — per-call bridge timeout in seconds (default 5.0).
    AXIS_DRY_RUN        — "0" to execute guarded tools live (default),
                          any other value to log-only.
    AXIS_PORT           — fixed tool-server port (default 51516).
    AXIS_MANIFEST_MODE  — full | lean | tiered (default full).
    AXIS_OMNILINK_LIB   — override path to omnilink-lib/src (else
                          discovered among siblings of the OmniSim
                          checkout: olink/, OmniLink/, omnilink/).
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

# Put omnilink-lib on sys.path up front so tool modules that import it at
# load time resolve during load_all() below (the runner also locates it,
# but that happens later inside run()).
locate_omnilink_lib(env_var="AXIS_OMNILINK_LIB")

from tools import ToolSpec, load_all              # noqa: E402
from tools._base import GUARDED, SAFE             # noqa: E402

ENGINE = "g2-engine"  # Axis ships on g2-engine, not the g1 default.

DRY_RUN = os.environ.get("AXIS_DRY_RUN", "0").strip() not in ("0", "false", "no", "")
MANIFEST_MODE = os.environ.get("AXIS_MANIFEST_MODE", "full").strip().lower()
if MANIFEST_MODE not in ("full", "lean", "tiered"):
    MANIFEST_MODE = "full"

REGISTRY: Dict[str, ToolSpec] = load_all()
SAFE_TOOLS = {n for n, s in REGISTRY.items() if s.tier == SAFE}
GUARDED_TOOLS = {n for n, s in REGISTRY.items() if s.tier == GUARDED}


def _manifest_specs() -> List[ToolSpec]:
    # In "tiered" mode only the always-on tools are advertised to the LLM;
    # every tool stays dispatchable. full/lean advertise the whole set.
    if MANIFEST_MODE == "tiered":
        return [s for s in REGISTRY.values() if getattr(s, "surface", "always") == "always"]
    return list(REGISTRY.values())


_desc_mode = "full" if MANIFEST_MODE == "full" else "lean"
QUERY_TOOLS: List[Dict[str, Any]] = [s.to_query_tool(mode=_desc_mode) for s in _manifest_specs()]


def dispatch(tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    """Tier-gated tool dispatch. SAFE tools run; GUARDED tools honour DRY_RUN.

    Axis has no shell denylist — there's no run_shell tool. Bridge-side
    clamping is the motion-safety enforcement and stop_robot (SAFE) is
    always available.
    """
    spec = REGISTRY.get(tool_name)
    if spec is None:
        return {
            "error": f"unknown tool: {tool_name}",
            "hint": "This tool is not implemented. Use only tools from 'known_tools'. Do not invent tool names.",
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


# Wire the meta-tools (registry introspection) to the registry + dispatcher.
try:
    from tools import registry_tools as _registry_tools  # type: ignore
    _registry_tools.REGISTRY_REF = REGISTRY
    _registry_tools.DISPATCH_REF = dispatch
except Exception:
    pass

# recall queries local long-term memory and the knowledge folder.
try:
    from tools import recall as _recall_mod          # type: ignore
    from tools import local_memory as _local_mem_mod  # type: ignore
    _sk = REGISTRY.get("search_knowledge")
    if _sk is not None:
        _recall_mod.SEARCH_KNOWLEDGE_IMPL = _sk.impl
    _recall_mod.SEARCH_LOCAL_MEMORY_IMPL = _local_mem_mod.search_local_memory_for_recall
except Exception as _exc:
    print(f"  [recall] wire failed: {_exc}")


def classify_result(tool: str, args: Dict[str, Any], result: Any):
    if not isinstance(result, dict):
        return "info", f"{tool} -> {result!r}"[:120]
    if result.get("error"):
        return "warning", f"{tool} -> error: {result['error']}"[:120]
    if result.get("status") == "dry_run":
        return "info", f"{tool}: dry_run"
    return "info", f"{tool}: ok"


def status_snapshot(activity_log: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Axis /status payload (the original had no /status). Runner adds usage."""
    always = sum(1 for s in REGISTRY.values() if getattr(s, "surface", "always") == "always")
    last_action = None
    for entry in reversed(activity_log[-50:]):
        d = entry.get("data") or {}
        last_action = {
            "tool": d.get("tool"), "kind": entry.get("kind"),
            "detail": entry.get("detail"), "timestamp": entry.get("timestamp"),
        }
        break
    return {
        "agent": "Axis",
        "tools_registered": len(REGISTRY),
        "safe": len(SAFE_TOOLS),
        "guarded": len(GUARDED_TOOLS),
        "surface": {"always": always, "on_demand": len(REGISTRY) - always},
        "manifest_mode": MANIFEST_MODE,
        "manifest_tools": len(QUERY_TOOLS),
        "last_action": last_action,
        "activity_log_size": len(activity_log),
    }


if __name__ == "__main__":
    OmniLinkAgentRunner(
        agent_name="Axis",
        profile_path=_THIS / "profile.json",
        port=51516,
        dispatch=dispatch,
        query_tools=QUERY_TOOLS,
        engine=ENGINE,
        port_env="AXIS_PORT",
        lib_env="AXIS_OMNILINK_LIB",
        dry_run_env="AXIS_DRY_RUN",
        classify_result=classify_result,
        status_snapshot=status_snapshot,
    ).run()
