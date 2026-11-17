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

"""Arm a persistent (scheduled) OmniLink agent against the simulated OmniArm 6.

This is the end-to-end exercise of OmniLink's persistence path:

    OmniLink cron tick  (their infrastructure, on a schedule)
      -> POST /api/chat  (returns tool_calls, with the profile's tools attached)
      -> a `tool` frame down the edge WebSocket your machine holds open
      -> omnilink.edge_connector POSTs it to the arm bridge on 127.0.0.1:8765
      -> the OmniArm 6 moves

Nothing runs on OmniLink's servers except the wake-up and the model call. The
arm, the bridge and the tools are all on this machine -- which is the whole
reason a bridge on 127.0.0.1 is reachable at all.

Prerequisites, in order (see README.md):
  1. OMNI_KEY exported
  2. the OmniArm 6 chat world running, so the bridge registers the profile
  3. the edge connector running (ideally installed as a service)

Usage:
    python create_order.py                    # every 5 minutes
    python create_order.py --every 2m
    python create_order.py --list             # show existing orders + last runs
    python create_order.py --delete
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request

BASE = os.environ.get("OMNILINK_BASE_URL", "https://www.omnilink-agents.com").rstrip("/")

# The arm bridge registers itself under this name -- see
# setup_omnilink_relay() in omnilink_arm_bridge.py, which builds
# f"OmniSim-{bridge.robot_id}" and the world passes --robot omniarm6.
AGENT_NAME = "OmniSim-omniarm6"
ORDER_NAME = "OmniArm 6 shift check"

# Deliberately small and observable. Every tool named here is one the arm
# bridge actually publishes (build_arm_tools): get_robot_state, wave,
# reset_to_home, set_joint_positions, set_tcp_target, stop_robot, grasp,
# open_gripper, close_gripper. Asking for anything else would be refused by
# the connector's allow-list, which is the correct behaviour but a confusing
# first demo.
INSTRUCTIONS = (
    "You are checking on the OmniArm 6 arm while nobody is watching.\n"
    "1. Call get_robot_state and note the joint positions.\n"
    "2. Call wave, so a human walking past can see you ran.\n"
    "3. Call reset_to_home to leave the arm where you found it.\n"
    "Then reply with one short line: the time you ran, and the joint "
    "positions you saw. Keep it under 200 characters."
)


def request(method: str, path: str, key: str, body: dict | None = None) -> tuple[int, dict]:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f"{BASE}{path}", data=data, method=method)
    req.add_header("Authorization", f"Bearer {key}")
    if data:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        raw = e.read()
        try:
            return e.code, json.loads(raw or b"{}")
        except json.JSONDecodeError:
            return e.code, {"error": raw.decode(errors="replace")[:400]}


def find_profile(key: str) -> dict | None:
    status, payload = request("GET", "/api/agent-profiles", key)
    if status != 200:
        sys.exit(f"Could not list agent profiles ({status}): {payload.get('error', payload)}")
    profiles = payload.get("profiles") or payload.get("data") or []
    for p in profiles:
        if (p.get("name") or "").lower() == AGENT_NAME.lower():
            return p
    return None


def preflight(key: str) -> None:
    """Fail with something actionable rather than arming an agent that can't act."""
    profile = find_profile(key)
    if not profile:
        sys.exit(
            f"No agent profile named {AGENT_NAME!r}.\n"
            "The arm bridge creates it on startup, so launch the world first:\n"
            "    launch.bat projects\\samples\\demos\\worlds\\chat\\omnilink_omniarm6.omniworld\n"
            "with OMNI_KEY set in the same shell. Without OMNI_KEY the bridge\n"
            "skips profile sync entirely (see profile_sync.is_enabled)."
        )

    settings = profile.get("settings") or {}
    callback = settings.get("toolCallbackUrl")
    tools = settings.get("availableToolDetails") or []

    print(f"  profile         {AGENT_NAME} (id={profile.get('id')})")
    print(f"  toolCallbackUrl {callback or '-- MISSING --'}")
    print(f"  tools           {len(tools)} declared")

    if not callback:
        sys.exit(
            "The profile has no toolCallbackUrl, so a scheduled run would have\n"
            "nowhere to send its tool calls. Restart the world with OMNI_KEY set."
        )
    if not tools:
        sys.exit(
            "The profile declares no tools. The connector refuses any tool the\n"
            "agent has not declared, so every scheduled run would fail closed."
        )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--every", default="5m", help="interval: 30s, 5m, 1h, 1d (default 5m)")
    ap.add_argument("--list", action="store_true", help="show existing orders and their last runs")
    ap.add_argument("--delete", action="store_true", help="remove the order")
    args = ap.parse_args()

    key = os.environ.get("OMNI_KEY", "").strip()
    if not key:
        sys.exit("OMNI_KEY is not set. Export your Omni Key and try again.")

    status, payload = request("GET", "/api/standing-orders", key)
    if status != 200:
        sys.exit(f"Could not list standing orders ({status}): {payload.get('error', payload)}")
    orders = payload.get("orders") or []
    existing = next((o for o in orders if o.get("name") == ORDER_NAME), None)

    if args.list:
        if not orders:
            print("No standing orders.")
            return
        for o in orders:
            print(f"- {o.get('name')}  [{o.get('status')}]  {o.get('schedule_type')}:"
                  f"{o.get('schedule_expression')}  agent={o.get('agent_name')}")
            print(f"    next run: {o.get('next_run_at') or '-'}   last: {o.get('last_run_at') or 'never'}")
        return

    if args.delete:
        if not existing:
            print("Nothing to delete.")
            return
        status, payload = request("DELETE", "/api/standing-orders", key, {"id": existing["id"]})
        print("Deleted." if status == 200 else f"Delete failed ({status}): {payload}")
        return

    print("Preflight:")
    preflight(key)

    body = {
        "agent_name": AGENT_NAME,
        "name": ORDER_NAME,
        "instructions": INSTRUCTIONS,
        "schedule_type": "interval",
        "schedule_expression": args.every,
        # The tick matches cron fields against its own host clock and ignores
        # this value (calculateNextRun, api/_standing-orders.ts). Interval
        # schedules are unaffected, but send UTC rather than imply otherwise.
        "schedule_timezone": "UTC",
    }

    if existing:
        body["id"] = existing["id"]
        body["status"] = "active"
        status, payload = request("PUT", "/api/standing-orders", key, body)
        verb = "Updated"
    else:
        status, payload = request("POST", "/api/standing-orders", key, body)
        verb = "Created"

    if status in (200, 201):
        order = payload.get("order") or {}
        print(f"\n{verb} {ORDER_NAME!r} -- runs every {args.every}.")
        print(f"  next run: {order.get('next_run_at') or 'scheduled'}")
        print("\nFor it to actually move the arm, all three must be true:")
        print("  - the OmniArm 6 world is running (the bridge owns 127.0.0.1:8765)")
        print("  - the edge connector is running   (python -m omnilink.edge_service status)")
        print("  - this machine is on")
        print("\nWatch it with:  python create_order.py --list")
        return

    # The entitlement gate is the interesting failure, so name it plainly.
    detail = payload.get("persistentAgents") or {}
    code = payload.get("code")
    if status == 402:
        print(f"\nRefused ({code}).")
        print(f"  plan:  {detail.get('planName') or detail.get('planId')}")
        print(f"  using: {detail.get('current')} of {detail.get('limit')} persistent agents")
        suggested = detail.get("suggestedPlan") or {}
        if suggested:
            print(f"  {suggested.get('name')} ({suggested.get('price')}/mo) allows "
                  f"{suggested.get('persistentAgentLimit')}.")
        print("\nFree agents run only while you are connected; scheduled agents are a paid tier.")
    elif status == 503:
        print(f"\nCould not confirm your plan ({code}). This is an outage, not a billing "
              "decision -- try again shortly.")
    else:
        print(f"\nFailed ({status}): {payload.get('error', payload)}")
    sys.exit(1)


if __name__ == "__main__":
    main()
