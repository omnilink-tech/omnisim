#!/usr/bin/env python3
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

"""plan_walls_probe — demonstrate the OmniLink plan entitlement walls, live.

The showcase of honest pricing: the platform REFUSES what the plan does not
include, with a structured, explainable error — it never silently degrades.
This probe (run by the ORCHESTRATOR, never by builder lanes) shows both
Builder-plan walls on standing orders:

1. ``POST /api/standing-orders`` with a 5-minute interval
   -> expect **402 WAKE_CADENCE_NOT_ON_PLAN** (sub-hourly wakes are not on
   the Builder plan). The structured refusal payload is printed verbatim.
2. Create hourly-interval orders — each on a DISTINCT agent, because the
   limit counts distinct agents running unattended, not orders — until
   -> expect **402 PERSISTENT_AGENT_LIMIT_REACHED** (Builder limit: 3
   persistent agents). Again the payload is the point.
3. **Cleanup is mandatory**: every order AND every throwaway agent profile
   this probe created is DELETEd, with a GET list before and after so the
   account leaves exactly as it entered.

Usage::

    python plan_walls_probe.py                 # uses OMNI_KEY / --key-file
    python plan_walls_probe.py --base-url ...  # non-default platform

Endpoint shapes (verified live against the platform, 2026-08-19):
  POST   /api/standing-orders   create -> 201 {"order": {...}}
  GET    /api/standing-orders   list   -> 200 {"orders": [...]}
  DELETE /api/standing-orders   body {"id": ...} -> {"deleted": true, ...}
  POST   /api/agent-profiles    create -> 201 {"ok": true, "profile": {...}}
  DELETE /api/agent-profiles    body {"id": ...} -> {"ok": true, ...}
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

DEFAULT_BASE_URL = "https://www.omnilink-agents.com"
# Key-file fallbacks, tried in order, and only when OMNI_KEY is unset and
# OMNI_KEY_FILE names nothing. Only a portable, machine-independent location is
# listed: a path on one developer's drive is dead weight in a public repository
# and discloses their local layout for no benefit. Point OMNI_KEY_FILE at a key
# store elsewhere if yours does not live here.
DEFAULT_KEY_FILES = (
    Path.home() / ".omnilink" / "omni_key.txt",
)
# Wall 1 rides on this name (any profile works — cadence is checked first).
AGENT_NAME = "SmartHouse-Bench"
# Wall 2 needs DISTINCT agents: the persistent-agent limit counts agents
# running unattended, not orders, so each hourly order gets its own
# throwaway profile named with this prefix.
AGENT_NAME_PREFIX = "SmartHouse-ProbeSlot"
MAX_HOURLY_ATTEMPTS = 6   # Builder limit is 3; leave headroom, never loop forever


def resolve_key(key_file: Optional[str] = None) -> Optional[str]:
    """OMNI_KEY, else --key-file / OMNI_KEY_FILE, else DEFAULT_KEY_FILES in order."""
    key = os.environ.get("OMNI_KEY", "").strip()
    if key:
        return key
    override = (key_file or os.environ.get("OMNI_KEY_FILE", "")).strip()
    candidates = (Path(override),) if override else DEFAULT_KEY_FILES
    for path in candidates:
        try:
            key = path.read_text(encoding="utf-8").strip()
        except OSError:
            continue
        if key:
            return key
    return None


class Api:
    def __init__(self, base_url: str, key: str) -> None:
        self.base_url = base_url.rstrip("/")
        self.key = key

    def call(self, method: str, path: str,
             body: Optional[Dict[str, Any]] = None
             ) -> Tuple[int, Any]:
        url = f"{self.base_url}{path}"
        data = json.dumps(body).encode("utf-8") if body is not None else None
        req = urllib.request.Request(url, data=data, method=method)
        req.add_header("Authorization", f"Bearer {self.key}")
        if data is not None:
            req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                raw = r.read().decode("utf-8", errors="replace")
                status = r.status
        except urllib.error.HTTPError as e:
            raw = e.read().decode("utf-8", errors="replace")
            status = e.code
        try:
            payload = json.loads(raw) if raw.strip() else {}
        except json.JSONDecodeError:
            payload = {"raw": raw[:1000]}
        return status, payload


def show(title: str, status: int, payload: Any) -> None:
    print(f"\n--- {title}")
    print(f"    HTTP {status}")
    print(json.dumps(payload, indent=2, ensure_ascii=False))


def order_body(name: str, expression: str,
               agent_name: str = AGENT_NAME) -> Dict[str, Any]:
    return {
        "agent_name": agent_name,
        "name": name,
        "instructions": (
            "House heartbeat: check anomalies and sensors; act per the "
            "SmartHouse mandate; notify the occupant only when something "
            "needed fixing."
        ),
        "schedule_type": "interval",
        "schedule_expression": expression,
        "execution_mode": "isolated",
        "execution_timeout": 120,
        "notification_method": "none",
    }


def extract_id(payload: Any) -> Optional[str]:
    if not isinstance(payload, dict):
        return None
    for key in ("id", "order_id", "standing_order_id"):
        if payload.get(key):
            return str(payload[key])
    inner = payload.get("standingOrder") or payload.get("order")
    if isinstance(inner, dict) and inner.get("id"):
        return str(inner["id"])
    return None


def error_code(payload: Any) -> str:
    """The machine code of a refusal. Platform 402s are flat
    {"error": "<human message>", "code": "<CODE>", ...} — prefer the code;
    the human message never contains it."""
    if isinstance(payload, dict):
        code = payload.get("code")
        if code:
            return str(code)
        err = payload.get("error")
        if isinstance(err, dict):  # nested {"error": {"code": ...}} shape
            return str(err.get("code") or "")
        return str(err or "")
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--base-url", default=DEFAULT_BASE_URL)
    ap.add_argument("--key-file", default=None,
                    help="read the OmniLink key from this file (overrides "
                         "OMNI_KEY_FILE and the default search)")
    args = ap.parse_args()

    key = resolve_key(args.key_file)
    if not key:
        print("No OMNI_KEY in the environment and no key file readable — "
              "refusing to probe the live platform.")
        return 2
    api = Api(args.base_url, key)
    created_orders: List[str] = []
    created_profiles: List[str] = []

    try:
        status, payload = api.call("GET", "/api/standing-orders")
        show("Standing orders BEFORE", status, payload)

        # Wall 1: sub-hourly cadence -> 402 WAKE_CADENCE_NOT_ON_PLAN
        status, payload = api.call(
            "POST", "/api/standing-orders",
            order_body("smart-house-probe-5min", "5m"))
        show("Wall 1 — 5-minute interval (expect 402 "
             "WAKE_CADENCE_NOT_ON_PLAN)", status, payload)
        oid = extract_id(payload)
        if status < 300 and oid:
            print("    UNEXPECTED: the platform accepted a 5-minute cadence. "
                  "Tracking it for cleanup.")
            created_orders.append(oid)

        # Wall 2: hourly orders, each on its own throwaway agent (the limit
        # counts distinct unattended AGENTS, not orders), until
        # 402 PERSISTENT_AGENT_LIMIT_REACHED.
        hit_limit = False
        for i in range(1, MAX_HOURLY_ATTEMPTS + 1):
            slot_name = f"{AGENT_NAME_PREFIX}-{i}"
            status, payload = api.call(
                "POST", "/api/agent-profiles",
                {"name": slot_name,
                 "settings": {"agentName": slot_name,
                              "mainTask": "Entitlement probe slot. Reply: ok",
                              "allowToolUse": False}})
            profile = payload.get("profile") if isinstance(payload, dict) else None
            pid = profile.get("id") if isinstance(profile, dict) else None
            if status < 300 and pid:
                created_profiles.append(str(pid))
            else:
                show(f"Wall 2 — profile create #{i} FAILED, stopping",
                     status, payload)
                break
            status, payload = api.call(
                "POST", "/api/standing-orders",
                order_body(f"smart-house-probe-hourly-{i}", "1h",
                           agent_name=slot_name))
            show(f"Wall 2 — hourly order #{i} on agent {slot_name}",
                 status, payload)
            oid = extract_id(payload)
            if status < 300 and oid:
                created_orders.append(oid)
                continue
            if status == 402 and "PERSISTENT_AGENT_LIMIT" in error_code(payload):
                hit_limit = True
                print(f"    Limit wall reached after {len(created_orders)} "
                      f"agent slot(s) — the Builder plan caps persistent agents.")
                break
            print("    Stopping: refusal was not the expected limit wall.")
            break
        if not hit_limit and len(created_orders) >= MAX_HOURLY_ATTEMPTS:
            print("    UNEXPECTED: never hit PERSISTENT_AGENT_LIMIT_REACHED "
                  f"after {MAX_HOURLY_ATTEMPTS} orders.")
    finally:
        # Cleanup is mandatory — delete everything this probe created.
        # Platform convention (verified live): DELETE with a JSON body {"id"}.
        print(f"\n--- Cleanup: deleting {len(created_orders)} order(s) and "
              f"{len(created_profiles)} probe profile(s)")
        for oid in created_orders:
            status, payload = api.call(
                "DELETE", "/api/standing-orders", {"id": oid})
            print(f"    DELETE order {oid}: HTTP {status} "
                  f"{json.dumps(payload, ensure_ascii=False)[:200]}")
        for pid in created_profiles:
            status, payload = api.call(
                "DELETE", "/api/agent-profiles", {"id": pid})
            print(f"    DELETE profile {pid}: HTTP {status} "
                  f"{json.dumps(payload, ensure_ascii=False)[:200]}")
        status, payload = api.call("GET", "/api/standing-orders")
        show("Standing orders AFTER (should match BEFORE)", status, payload)
    return 0


if __name__ == "__main__":
    sys.exit(main())
