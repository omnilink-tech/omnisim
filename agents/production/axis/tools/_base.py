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

"""Shared types, helpers, and optional imports for Axis's tool modules.

A tool module defines:

    from ._base import ToolSpec, SAFE, GUARDED  # tiers
    ...
    SPEC = ToolSpec(
        name="my_tool",
        tier=SAFE,                          # or GUARDED
        description="...",
        parameters={"type": "object", "properties": {...}},
        impl=my_impl_function,
    )

or exports a list via `SPECS = [ToolSpec(...), ...]` for files that host
related tools (e.g. all the motion commands in one module).

The runner loads every SPEC into its dispatcher. Guarded tools execute
immediately but are subject to bridge-side clamping; safe tools have no
motion side-effect.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

# ── Tier + surface constants ────────────────────────────────────────

SAFE = "safe"
GUARDED = "guarded"

# Surface controls whether a tool is declared in the chat system
# instruction every turn ("always") or kept behind the find_tools /
# invoke_tool discovery path ("on_demand"). At small registries, set
# everything to "always". At scale, most tools shift to "on_demand"
# so the manifest stays tiny and Axis pulls in what it needs.
ALWAYS = "always"
ON_DEMAND = "on_demand"


@dataclass
class ToolSpec:
    """Everything the runner needs to route and invoke a tool.

    `tier`        — "safe" runs directly; "guarded" runs through the
                    dispatcher (DRY_RUN gate, bridge-side clamp).
    `description` — what the AI sees. Be specific about when to reach
                    for this tool over alternatives.
    `parameters`  — OpenAI-style JSON Schema. Use
                    `{"type": "object", "properties": {}}` for no-arg tools.
    `impl`        — callable taking **kwargs and returning a dict.
                    Return `{"error": "..."}` for failures (never raise
                    through to the HTTP handler).
    `surface`     — "always" (default) ships the tool in every chat
                    request's system instruction. "on_demand" keeps it
                    out of the manifest and only reachable via
                    find_tools + invoke_tool.
    """

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


# ── OmniSim bridge ──────────────────────────────────────────────────

# The bridge URL is read once at import. Override via AXIS_BRIDGE_URL.
# Default port 8765 matches the omnilink_arm_bridge controller shipped
# in projects/samples/demos/controllers/omnilink_arm_bridge/, launched
# with controllerArgs ["--robot" "omniarm6" "--port" "8765"].
BRIDGE_URL = os.environ.get("AXIS_BRIDGE_URL", "http://127.0.0.1:8765").rstrip("/")
BRIDGE_TIMEOUT = float(os.environ.get("AXIS_BRIDGE_TIMEOUT", "5.0"))


def bridge_call(endpoint: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """POST JSON to the OmniSim bridge. Returns a dict.

    On transport error, returns a `{"error": "..."}` dict with a `bridge`
    key so the caller can distinguish bridge problems from command
    rejections. Never raises — tools are expected to return shaped dicts
    even under failure.
    """
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
            "hint": "Start the OmniSim bridge and set AXIS_BRIDGE_URL.",
        }
    except Exception as e:
        return {"error": f"bridge call failed: {e.__class__.__name__}: {e}", "bridge": url}

    try:
        return json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        return {"error": "bridge returned non-JSON", "bridge": url, "raw": raw[:500]}


# ── Paths (shared) ──────────────────────────────────────────────────

_THIS_DIR = Path(__file__).resolve().parent
AGENT_ROOT = _THIS_DIR.parent          # .../agents/production/axis
AGENTS_DIR = AGENT_ROOT.parent         # .../omnilink-agents
REPO_ROOT = AGENTS_DIR.parent          # .../omnisim
