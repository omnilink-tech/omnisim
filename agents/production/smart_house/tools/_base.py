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

"""Shared types + the hub HTTP client for the smart_house agent.

Mirrors the layout of ``agents/production/husky_maze/tools/_base.py`` and the
tier convention of OmniLink's Haven agent, but is deliberately
self-contained: nothing here imports from the omnilink repo.

The hub is the OmniSim ``smart_house_bridge`` controller — or, for
offline development and CI, ``benchmark/mock_hub.py``, which serves the same
contract. Every hub endpoint is POST, JSON body, JSON response
(see the frozen contract; Haven's ``knowledge/adapters.md`` is the ancestor).

Tier convention (Haven's, verbatim):

  - SAFE             — sensor reads, lock_door, arm_security, notifications
                       (tightening security posture is always allowed)
  - GUARDED          — routine device control (lights, thermostat, blinds,
                       scenes) — auto-executes, hub validates
  - CONFIRM_REQUIRED — destructive or security-sensitive (unlock, disarm,
                       shut mains). The tool impl refuses CLIENT-SIDE until
                       the args include an ``authorization`` token.
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

# --- Tier + surface constants ----------------------------------------------

SAFE = "safe"
GUARDED = "guarded"
CONFIRM_REQUIRED = "confirm_required"

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
        pre = s[: m.start()]
        last_word = _re.search(r"(\w+)$", pre)
        if last_word and last_word.group(1).lower() in _ABBREVS:
            continue
        tail = s[end:]
        if not tail or tail[0].isspace():
            after = tail.lstrip()
            if not after or after[0].isupper() or after[0].isdigit():
                return s[:end].strip()
    return s


# --- Hub HTTP client ---------------------------------------------------------

DEFAULT_HUB_URL = "http://127.0.0.1:8766"
HUB_TIMEOUT_S = 5.0


def hub_url() -> str:
    """Resolve the hub base URL at CALL time (not import time).

    The benchmark points the whole tool registry at an in-process mock hub by
    setting ``SMART_HOUSE_HUB_URL`` after import — a module-level constant
    would freeze the wrong value.
    """
    return os.environ.get("SMART_HOUSE_HUB_URL", DEFAULT_HUB_URL).rstrip("/")


def hub_call(endpoint: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """POST ``{hub}/{endpoint}`` with a JSON body. Never raises.

    Transport failures come back as ``{"error": ..., "hub": url}`` dicts so a
    planner sees a structured failure instead of a stack trace. A top-level
    JSON array response (the contract's list_rooms / list_devices shape) is
    wrapped as ``{"items": [...]}`` so every tool result is a dict.
    """
    base = hub_url()
    url = f"{base}/{endpoint.lstrip('/')}"
    body = json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=HUB_TIMEOUT_S) as r:
            raw = r.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as e:
        detail = ""
        try:
            detail = e.read().decode("utf-8", errors="replace")
        except Exception:
            pass
        return {
            "error": f"hub HTTP {e.code}",
            "hub": url,
            "detail": detail[:500],
        }
    except urllib.error.URLError as e:
        return {
            "error": f"hub unreachable: {getattr(e, 'reason', e)}",
            "hub": url,
            "hint": (
                "Launch omnilink_smart_house.omniworld (smart_house_bridge on :8766) "
                "or run benchmark/mock_hub.py for an offline house."
            ),
        }
    except Exception as e:  # noqa: BLE001 - transport wrapper must not raise
        return {"error": f"hub call failed: {e.__class__.__name__}: {e}", "hub": url}
    try:
        parsed = json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        return {"error": "hub returned non-JSON", "hub": url, "raw": raw[:500]}
    if isinstance(parsed, list):
        return {"items": parsed}
    if isinstance(parsed, dict):
        return parsed
    return {"value": parsed}
