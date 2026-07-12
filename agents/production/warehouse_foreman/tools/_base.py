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

"""Shared helpers for Warehouse Foreman tools.

Sister of mission_captain/tools/_base.py. The two diverge on:

  - SPECIALIST_REGISTRY — Foreman knows about the Warehouse Picker (and
    later the Loader/Axis at the dock); the captain knows about Husky
    Maze + Axis. Each orchestrator carries its own roster so a single
    operator session can run multiple orchestrators side by side without
    them stomping on each other's specialists.

  - bridge_get/bridge_post — Foreman can read the warehouse mission
    brief directly from the husky_omnilink_bridge (the same bridge the
    Picker drives) so it can confirm the operator's task before
    delegating. The Captain has no such bridge.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

SAFE = "safe"
GUARDED = "guarded"
ALWAYS = "always"
ON_DEMAND = "on_demand"


@dataclass
class ToolSpec:
    name: str
    tier: str
    description: str
    parameters: Dict[str, Any]
    impl: Callable[..., Dict[str, Any]]
    tags: List[str] = field(default_factory=list)
    surface: str = ALWAYS

    def to_query_tool(self, mode: str = "full") -> Dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "parameters": self.parameters,
        }


# Specialist registry. Keys are the specialist's profile name on
# OmniLink (case-insensitive lookup). Operator can override the URLs
# via env vars when running the Foreman against a remote specialist.
SPECIALIST_REGISTRY: Dict[str, Dict[str, str]] = {
    "Warehouse Picker": {
        "runner_status_url": os.environ.get(
            "PICKER_STATUS_URL", "http://127.0.0.1:51520/status"
        ),
        "runner_activity_url": os.environ.get(
            "PICKER_ACTIVITY_URL", "http://127.0.0.1:51520/activity"
        ),
        "tool_callback_url": os.environ.get(
            "PICKER_TOOL_URL", "http://127.0.0.1:51520/tool"
        ),
        "what_it_does": (
            "Drives the Clearpath Husky on warehouse_logistics.wbt — "
            "continuous-space waypoint navigation, front-camera tag "
            "identification (red/green/blue/yellow/magenta/cyan), "
            "and complete_mission with a one-sentence rationale."
        ),
    },
    # NOTE: the dock-side Loader arm specialist was removed when OmniSim
    # stopped shipping an arm. The Warehouse Foreman now runs the mission
    # Picker-only — the Husky pushes the tagged pallet onto the dock, with
    # no arm leg to delegate.
}


BRIDGE_URL = os.environ.get("FOREMAN_BRIDGE_URL", "http://127.0.0.1:6070")


def http_get(url: str, timeout: float = 3.0) -> Dict[str, Any]:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            raw = r.read().decode("utf-8", errors="replace")
    except urllib.error.URLError as e:
        return {"error": f"unreachable: {e.reason if hasattr(e, 'reason') else e}", "url": url}
    except Exception as e:
        return {"error": f"{e.__class__.__name__}: {e}", "url": url}
    try:
        return json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        return {"error": "non-json response", "raw": raw[:500], "url": url}


def http_post(url: str, payload: Optional[Dict[str, Any]] = None, timeout: float = 60.0) -> Dict[str, Any]:
    body = json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode("utf-8", errors="replace")
    except urllib.error.URLError as e:
        return {"error": f"unreachable: {e.reason if hasattr(e, 'reason') else e}", "url": url}
    except Exception as e:
        return {"error": f"{e.__class__.__name__}: {e}", "url": url}
    try:
        return json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        return {"error": "non-json response", "raw": raw[:500], "url": url}


def bridge_get(path: str, timeout: float = 5.0) -> Dict[str, Any]:
    return http_get(f"{BRIDGE_URL.rstrip('/')}/{path.lstrip('/')}", timeout=timeout)


def bridge_post(path: str, payload: Optional[Dict[str, Any]] = None,
                timeout: float = 60.0) -> Dict[str, Any]:
    return http_post(f"{BRIDGE_URL.rstrip('/')}/{path.lstrip('/')}", payload, timeout=timeout)


_THIS_DIR = Path(__file__).resolve().parent
AGENT_ROOT = _THIS_DIR.parent
