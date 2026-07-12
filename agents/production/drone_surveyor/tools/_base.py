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

"""Shared types, helpers, and the bridge HTTP client for the drone_surveyor agent.

Mirrors `agents/production/husky_maze/tools/_base.py` one-for-one — same ToolSpec
shape, same SAFE / GUARDED tiering, same lean-vs-full description mode, same
bridge_get/bridge_post helpers. The only differences:
  - BRIDGE_URL defaults to http://127.0.0.1:6090 (mavic_omnilink_bridge port)
  - Env var override is MAVIC_BRIDGE_URL
  - Timeout is 120 s — flying a goto_waypoint at 12 m altitude over 20 m
    horizontal distance can take ~30-60 s in real wall-clock.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional


# --- Tier + surface constants ---------------------------------------------

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
        desc = self.description
        if mode == "lean":
            desc = _first_sentence(desc)
        return {
            "name": self.name,
            "description": desc,
            "parameters": self.parameters,
        }


_ABBREVS = {"e.g", "i.e", "etc", "vs", "a.m", "p.m", "mr", "mrs", "dr"}


def _first_sentence(text: str) -> str:
    import re as _re
    s = text.strip()
    for m in _re.finditer(r"[.!?]", s):
        end = m.end()
        pre = s[:m.start()]
        last_word = _re.search(r"(\w+)$", pre)
        if last_word and last_word.group(1).lower() in _ABBREVS:
            continue
        tail = s[end:]
        if not tail or tail[0].isspace():
            after = tail.lstrip()
            if not after or after[0].isupper() or after[0].isdigit():
                return s[:end].strip()
    return s


# --- Bridge HTTP client ---------------------------------------------------

BRIDGE_URL = os.environ.get("MAVIC_BRIDGE_URL", "http://127.0.0.1:6090").rstrip("/")
BRIDGE_TIMEOUT = float(os.environ.get("MAVIC_BRIDGE_TIMEOUT", "120"))


def bridge_get(endpoint: str) -> Dict[str, Any]:
    url = f"{BRIDGE_URL}/{endpoint.lstrip('/')}"
    req = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=BRIDGE_TIMEOUT) as r:
            raw = r.read().decode("utf-8", errors="replace")
    except urllib.error.URLError as e:
        return {
            "error": f"bridge unreachable: {e.reason if hasattr(e, 'reason') else e}",
            "bridge": url,
            "hint": "Launch chat/omnilink_mavic.wbt and confirm mavic_omnilink_bridge is running on :6090.",
        }
    except Exception as e:
        return {"error": f"bridge call failed: {e.__class__.__name__}: {e}", "bridge": url}
    try:
        return json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        return {"error": "bridge returned non-JSON", "bridge": url, "raw": raw[:500]}


def bridge_post(endpoint: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    url = f"{BRIDGE_URL}/{endpoint.lstrip('/')}"
    body = json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST")
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
        return {
            "error": f"bridge HTTP {e.code}",
            "bridge": url,
            "detail": detail[:500],
        }
    except urllib.error.URLError as e:
        return {
            "error": f"bridge unreachable: {e.reason}",
            "bridge": url,
            "hint": "Launch chat/omnilink_mavic.wbt and confirm mavic_omnilink_bridge is running on :6090.",
        }
    except Exception as e:
        return {"error": f"bridge call failed: {e.__class__.__name__}: {e}", "bridge": url}
    try:
        return json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        return {"error": "bridge returned non-JSON", "bridge": url, "raw": raw[:500]}


# --- Paths -----------------------------------------------------------------

_THIS_DIR = Path(__file__).resolve().parent
AGENT_ROOT = _THIS_DIR.parent
OLINK_AGENTS_ROOT = AGENT_ROOT.parent
OMNISIM_ROOT = OLINK_AGENTS_ROOT.parent
WORLD_PATH = OMNISIM_ROOT / "projects" / "samples" / "demos" / "worlds" / "chat" / "omnilink_mavic.wbt"
