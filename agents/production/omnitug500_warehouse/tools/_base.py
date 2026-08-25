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

"""Shared types + bridge HTTP client for the OmniTug 500 warehouse-courier agent.

Mirrors agents/production/husky_maze/tools/_base.py. The bridge is the
omnitug500_courier controller's HTTP surface (default 127.0.0.1:8765), which the
omnitug500_courier.omniworld world brings up. Every tool here is a thin wrapper over one
of its endpoints.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

# --- Tier + surface constants ----------------------------------------------
SAFE = "safe"          # read-only (state, capabilities)
GUARDED = "guarded"    # commands motion / moves packages

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
        return {"name": self.name, "description": self.description,
                "parameters": self.parameters}


# --- Bridge HTTP client ----------------------------------------------------
BRIDGE_URL = os.environ.get("OMNITUG500_BRIDGE_URL", "http://127.0.0.1:8765").rstrip("/")
BRIDGE_TIMEOUT = float(os.environ.get("OMNITUG500_BRIDGE_TIMEOUT", "15"))

_UNREACHABLE_HINT = ("Launch omnitug500_courier.omniworld and confirm the omnitug500_courier "
                     "controller is running (HTTP on 8765).")


def _request(url: str, payload: Optional[Dict[str, Any]], method: str) -> Dict[str, Any]:
    data = json.dumps(payload or {}).encode("utf-8") if method == "POST" else None
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=BRIDGE_TIMEOUT) as r:
            raw = r.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as e:
        detail = ""
        try:
            detail = e.read().decode("utf-8", errors="replace")
        except Exception:
            pass
        return {"error": f"bridge HTTP {e.code}", "bridge": url, "detail": detail[:400]}
    except urllib.error.URLError as e:
        return {"error": f"bridge unreachable: {getattr(e, 'reason', e)}",
                "bridge": url, "hint": _UNREACHABLE_HINT}
    except Exception as e:
        return {"error": f"bridge call failed: {e.__class__.__name__}: {e}", "bridge": url}
    try:
        return json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        return {"error": "bridge returned non-JSON", "bridge": url, "raw": raw[:400]}


def bridge_get(endpoint: str) -> Dict[str, Any]:
    return _request(f"{BRIDGE_URL}/{endpoint.lstrip('/')}", None, "GET")


def bridge_post(endpoint: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    return _request(f"{BRIDGE_URL}/{endpoint.lstrip('/')}", payload, "POST")
