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

"""SmartHouse — OmniLink house-manager agent runner.

Thin runtime built on the agents/production/_lib SDK (OmniLinkAgentRunner):
the HTTP tool server, profile push, usage metering, and main loop come from
the shared runner; this module supplies the smart-house tool dispatch,
result classification, and status snapshot.

The agent proxies 19 Haven-shaped tools to the smart-house hub — the
``smart_house_bridge`` controller inside ``omnilink_smart_house.omniworld``
(default ``http://127.0.0.1:8766``). For offline development without the
simulator, ``benchmark/mock_hub.py`` serves the same contract:

    python agents/production/smart_house/benchmark/mock_hub.py --port 8766

Run:

    export OMNI_KEY="olink_YOUR_KEY_HERE"    # or rely on the key-file fallback
    python agents/production/smart_house/smart_house_agent.py

Environment:
    OMNI_KEY                     - platform key. If unset, the runner reads
                                   OMNI_KEY_FILE, else else
                                   ~/.omnilink/omni_key.txt, and sets the env
                                   var itself. The key is never printed.
    SMART_HOUSE_AGENT_PORT       - tool-server port (default 51534).
    SMART_HOUSE_AGENT_DRY_RUN    - "1" to log guarded tools without executing.
    SMART_HOUSE_OMNILINK_LIB     - path to omnilink-lib/src (else auto-detected).
    SMART_HOUSE_TOOL_DESCRIPTIONS- "full" (default) or "lean".
    SMART_HOUSE_HUB_URL          - hub URL (default http://127.0.0.1:8766).
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

locate_omnilink_lib(env_var="SMART_HOUSE_OMNILINK_LIB")

from tools import ToolSpec, load_all                                   # noqa: E402
from tools._base import ALWAYS, CONFIRM_REQUIRED, GUARDED, SAFE        # noqa: E402

ENGINE = "g1-engine"
DEFAULT_PORT = 51534
PROFILE_PATH = _THIS / "profile.json"
SYSTEM_PROMPT_PATH = _THIS / "prompts" / "system.md"
# Key-file fallbacks, tried in order, and only when OMNI_KEY is unset and
# OMNI_KEY_FILE names nothing. Only a portable, machine-independent location is
# listed: a path on one developer's drive is dead weight in a public repository
# and discloses their local layout for no benefit. Point OMNI_KEY_FILE at a key
# store elsewhere if yours does not live here.
DEFAULT_KEY_FILES = (
    Path.home() / ".omnilink" / "omni_key.txt",
)

DRY_RUN = os.environ.get("SMART_HOUSE_AGENT_DRY_RUN", "0").strip() not in ("0", "false", "no", "")
_TOOL_DESC_MODE = os.environ.get("SMART_HOUSE_TOOL_DESCRIPTIONS", "full").strip() or "full"

REGISTRY: Dict[str, ToolSpec] = load_all()
QUERY_TOOLS: List[Dict[str, Any]] = [
    s.to_query_tool(mode=_TOOL_DESC_MODE)
    for s in REGISTRY.values()
    if s.surface == ALWAYS
]


def ensure_omni_key() -> None:
    """Populate OMNI_KEY from a local key file when the env var is unset.

    Never prints the key. OMNI_KEY_FILE overrides the search entirely;
    otherwise DEFAULT_KEY_FILES is tried in order. When no source yields a key,
    the shared runner prints its usual hard-exit guidance.
    """
    if os.environ.get("OMNI_KEY", "").strip():
        return
    override = os.environ.get("OMNI_KEY_FILE", "").strip()
    candidates = (Path(override),) if override else DEFAULT_KEY_FILES
    for key_file in candidates:
        try:
            key = key_file.read_text(encoding="utf-8").strip()
        except OSError:
            continue
        if key:
            os.environ["OMNI_KEY"] = key
            print(f"  OMNI_KEY loaded from {key_file} (value not shown)")
            return


def dispatch(tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    """Tier-gated tool dispatch.

    SAFE tools always run. GUARDED tools honour DRY_RUN. CONFIRM_REQUIRED
    tools additionally refuse CLIENT-SIDE inside their impl when no
    ``authorization`` arg is present (the request never reaches the hub) —
    the dispatcher only adds the DRY_RUN gate on top.
    """
    spec = REGISTRY.get(tool_name)
    if spec is None:
        return {
            "error": f"unknown tool: {tool_name}",
            "hint": "Use only tools from 'known_tools'. Do not invent tool names.",
            "known_tools": sorted(REGISTRY.keys()),
        }
    if spec.tier == SAFE:
        return spec.impl(**args)
    if spec.tier in (GUARDED, CONFIRM_REQUIRED):
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
        err_s = str(err).lower()
        if "authorization" in err_s:
            return "warning", f"{tool} -> refused: authorization required"
        kind = "critical" if "unreachable" in err_s else "warning"
        return kind, f"{tool} -> error: {err}"

    if result.get("accepted") is False:
        return "warning", f"{tool} -> rejected: {result.get('message', result.get('error', ''))!s:.80}"

    if tool == "check_anomalies":
        active = result.get("active") or []
        if active:
            types = ", ".join(str(a.get("type", "?")) for a in active[:4])
            return "critical", f"check_anomalies: {len(active)} ACTIVE ({types})"
        return "info", "check_anomalies: clear"

    if tool == "notify_occupant":
        sev = args.get("severity", "medium")
        kind = "critical" if sev == "critical" else ("warning" if sev == "high" else "info")
        return kind, f"notify_occupant [{sev}]: {args.get('message', '')!s:.80}"

    if tool in ("set_device", "toggle_device"):
        state = result.get("realized_state", result.get("new_state"))
        return "success", f"{tool}: {args.get('id')} -> {state!r:.60}"

    if tool == "set_scene":
        n = len(result.get("affected") or [])
        return "success", f"set_scene {args.get('scene')!r}: {n} devices"

    if tool == "adjust_thermostat":
        return "success", f"adjust_thermostat: target={result.get('target')} mode={result.get('mode')}"

    if tool in ("lock_door", "arm_security"):
        return "success", f"{tool}: {result.get('state', 'ok')}"

    if tool in ("unlock_door", "disarm_security"):
        return "warning", f"{tool}: EXECUTED (authorized)"

    if tool == "get_energy_report":
        return "info", f"get_energy_report: total {result.get('total_kwh')} kWh"

    return "info", f"{tool}: ok"


def status_snapshot(activity_log: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Compress the recent activity log into an at-a-glance status."""
    last_action = None
    last_anomalies = None
    notifications = 0
    refusals = 0
    for entry in reversed(activity_log[-100:]):
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
        if tool == "check_anomalies" and last_anomalies is None and isinstance(result, dict):
            last_anomalies = result.get("active")
        if tool == "notify_occupant":
            notifications += 1
        if isinstance(result, dict) and "authorization" in str(result.get("error", "")):
            refusals += 1
    return {
        "agent": "SmartHouse",
        "hub_url": os.environ.get("SMART_HOUSE_HUB_URL", "http://127.0.0.1:8766"),
        "tools_registered": len(REGISTRY),
        "dry_run": DRY_RUN,
        "last_action": last_action,
        "active_anomalies_last_seen": last_anomalies,
        "notifications_sent": notifications,
        "authorization_refusals": refusals,
        "activity_log_size": len(activity_log),
    }


class SmartHouseRunner(OmniLinkAgentRunner):
    """Runner that keeps settings.mainTask synced from prompts/system.md.

    prompts/system.md is the canonical mandate (it is also the benchmark's
    inline systemInstructionRequest); profile.json carries a short copy for
    offline readers. Refreshing at load time means the two can never drift
    on the platform.
    """

    def _load_profile(self) -> Dict[str, Any]:
        doc = super()._load_profile()
        try:
            mandate = SYSTEM_PROMPT_PATH.read_text(encoding="utf-8").strip()
        except OSError:
            return doc
        if mandate:
            doc.setdefault("settings", {})["mainTask"] = mandate
        return doc


if __name__ == "__main__":
    ensure_omni_key()
    SmartHouseRunner(
        agent_name="SmartHouse",
        profile_path=PROFILE_PATH,
        port=int(os.environ.get("SMART_HOUSE_AGENT_PORT", str(DEFAULT_PORT))),
        dispatch=dispatch,
        query_tools=QUERY_TOOLS,
        engine=ENGINE,
        port_env="SMART_HOUSE_AGENT_PORT",
        lib_env="SMART_HOUSE_OMNILINK_LIB",
        dry_run_env="SMART_HOUSE_AGENT_DRY_RUN",
        classify_result=classify_result,
        status_snapshot=status_snapshot,
    ).run()
