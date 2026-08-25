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

# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""OmniTug 500 Warehouse Courier — OmniLink agent runner.

Thin runtime built on the agents/production/_lib SDK: it loads the courier tool
specs, pushes the agent profile to OmniLink, starts the local tool-callback HTTP
server, and polls. The agent's tools drive the omnitug500_courier bridge
(127.0.0.1:8765) brought up by omnitug500_courier.omniworld.

Run:

    # 1. open the world (windowed or headless)
    launch.bat projects\\robots\\omnisim\\omnitug500\\worlds\\omnitug500_courier.omniworld
    # 2. point the agent at it
    set OMNI_KEY=olink_YOUR_KEY
    python agents/production/omnitug500_warehouse/omnitug500_warehouse_agent.py

Or both in one step via the launcher:

    set OMNI_KEY=olink_YOUR_KEY
    python -m omnisim run-agent --agent omnitug500_warehouse

Then open https://www.omnilink-agents.com, pick "OmniTug 500 Warehouse Courier", and
chat: "take the package from bay B to dock 2".

Env:
    OMNI_KEY               your Omni Key (required)
    OMNITUG500_BRIDGE_URL      bridge URL (default http://127.0.0.1:8765)
    OMNITUG500_AGENT_PORT      tool-callback port (default 51532)
    OMNILINK_LIB           absolute path to omnilink-lib/src (else sibling-repo search)
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, List

_THIS = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS))                 # tools/
sys.path.insert(0, str(_THIS.parent))          # agents/production/_lib

from _lib import OmniLinkAgentRunner            # noqa: E402
from tools import ToolSpec, load_all            # noqa: E402

REGISTRY: Dict[str, ToolSpec] = load_all()
QUERY_TOOLS: List[Dict[str, Any]] = [s.to_query_tool() for s in REGISTRY.values()]


def dispatch(tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    spec = REGISTRY.get(tool_name)
    if spec is None:
        return {"error": f"unknown tool: {tool_name}",
                "known_tools": sorted(REGISTRY.keys())}
    return spec.impl(**args)


def classify_result(tool: str, args: Dict[str, Any], result: Any):
    if not isinstance(result, dict):
        return "info", f"{tool} -> {result!r:.80}"
    if result.get("error"):
        return "warning", f"{tool} -> error: {result['error']}"
    if tool == "get_courier_state":
        return ("info", f"state: pos=({result.get('x')},{result.get('y')}) "
                f"mode={result.get('mode')} carrying={result.get('carrying')} "
                f"queue={result.get('queue')} :: {result.get('last_event')}")
    if tool == "run_route":
        return "info", f"run_route: {result.get('steps')} steps -> {result.get('route')}"
    if tool in ("pick_package", "deliver_package", "goto_station"):
        return "info", f"{tool}: {result.get('op', 'ok')} {result.get('station', '')}"
    return "info", f"{tool}: ok"


def status_snapshot(activity_log: List[Dict[str, Any]]) -> Dict[str, Any]:
    last_state = None
    for entry in reversed(activity_log[-50:]):
        d = entry.get("data") or {}
        if d.get("tool") == "get_courier_state":
            last_state = d.get("result") or {}
            break
    return {
        "agent": "OmniTug 500 Warehouse Courier",
        "tools_registered": len(REGISTRY),
        "rover": ({"pos": [last_state.get("x"), last_state.get("y")],
                   "mode": last_state.get("mode"),
                   "carrying": last_state.get("carrying"),
                   "queue": last_state.get("queue")} if last_state else None),
        "activity_log_size": len(activity_log),
    }


if __name__ == "__main__":
    OmniLinkAgentRunner(
        agent_name="OmniTug 500 Warehouse Courier",
        profile_path=_THIS / "profile.json",
        port=51532,
        dispatch=dispatch,
        query_tools=QUERY_TOOLS,
        port_env="OMNITUG500_AGENT_PORT",
        classify_result=classify_result,
        status_snapshot=status_snapshot,
    ).run()
