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

"""Shared helpers for Mission Captain tools.

The captain has no robot bridge — it only orchestrates. So this base
module is much smaller than husky_maze's: just the tool dataclass,
tier constants, and a tiny HTTP helper for hitting specialist
runners' /status endpoints.
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


# Specialist registry: name -> /status URL of the specialist's local runner.
# The captain knows where each specialist is reachable; the operator can
# add/remove specialists by editing this map (or via env vars).
SPECIALIST_REGISTRY: Dict[str, Dict[str, str]] = {
    "Husky Maze": {
        "runner_status_url": os.environ.get(
            "HUSKY_MAZE_STATUS_URL", "http://127.0.0.1:51517/status"
        ),
        "runner_activity_url": os.environ.get(
            "HUSKY_MAZE_ACTIVITY_URL", "http://127.0.0.1:51517/activity"
        ),
        "tool_callback_url": os.environ.get(
            "HUSKY_MAZE_TOOL_URL", "http://127.0.0.1:51517/tool"
        ),
        "what_it_does": (
            "Drives the Clearpath Husky in maze worlds. Strategies: "
            "BFS-on-known-map, lidar wall-follow, vision-based marker "
            "hunting. Claims completion via complete_mission."
        ),
    },
    # Add further specialists here: give each one its runner's /status,
    # /activity and /tool URLs and a one-line "what_it_does" the captain
    # can route against.
}


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


_THIS_DIR = Path(__file__).resolve().parent
AGENT_ROOT = _THIS_DIR.parent
