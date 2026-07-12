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

"""Drive the Warehouse Picker agent via OmniLink chat() with a local
tool loop.

Same architecture as agents/production/mission_captain/scripts/chat_drive.py:
the OmniLink chat() endpoint is one-shot (returns toolCalls but can't
reach loopback URLs to dispatch them), so we run the dispatch loop
locally.

The picker's mission is short — one or two drive_to_waypoint calls and
a complete_mission — so MAX_TURNS defaults to 12.

Usage:
    OMNI_KEY=olink_... python agents/production/warehouse_picker/scripts/chat_drive.py \\
        "Drive to the green pallet at (3, 5) then return to the dock."

The Picker runner (warehouse_picker_agent.py) must already be running
so its profile is on the platform and its tool-callback server is up.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.request
from pathlib import Path

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="backslashreplace", line_buffering=True)  # type: ignore[attr-defined]
    except Exception:
        pass

REPO_ROOT = Path(__file__).resolve().parents[3]
LIB_SRC = REPO_ROOT.parent / "olink" / "omnilink-lib" / "src"
if LIB_SRC.exists() and str(LIB_SRC) not in sys.path:
    sys.path.insert(0, str(LIB_SRC))

from omnilink.client import OmniLinkClient  # noqa: E402

AGENT_NAME = "Warehouse Picker"
ENGINE = "g1-engine"
_PICKER_PORT = int(os.environ.get("PICKER_PORT", "51520"))
TOOL_SERVER_URL = f"http://127.0.0.1:{_PICKER_PORT}/tool"
ACTIVITY_URL = f"http://127.0.0.1:{_PICKER_PORT}/activity"
STATUS_URL = f"http://127.0.0.1:{_PICKER_PORT}/status"
MAX_TURNS = int(os.environ.get("PICKER_CHAT_MAX_TURNS", "12"))


def fetch_status():
    try:
        with urllib.request.urlopen(STATUS_URL, timeout=2) as r:
            return json.loads(r.read())
    except Exception:
        return {}


def execute_tool(name: str, args: dict) -> dict:
    payload = dict(args); payload["tool"] = name
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(TOOL_SERVER_URL, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    # drive_to_waypoint is synchronous server-side: a 14 m warehouse
    # traversal takes ~40 s end-to-end. Default timeout sized for two
    # such legs back-to-back plus headroom.
    timeout = int(os.environ.get("PICKER_DRIVE_TOOL_TIMEOUT", "300"))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            wrap = json.loads(r.read())
            return wrap.get("result", wrap)
    except Exception as exc:
        return {"error": f"local tool dispatch failed: {exc.__class__.__name__}: {exc}"}


def _summarise(name: str, result: dict) -> str:
    if name == "drive_to_waypoint" and isinstance(result, dict):
        compact = {
            k: v for k, v in result.items()
            if k in ("done", "fault", "x", "y", "distance_remaining_m",
                     "arrival_tolerance_m", "final_pose")
        }
        return json.dumps(compact, default=str)[:600]
    return json.dumps(result, default=str)[:800]


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("prompt", nargs="?",
                   default=("Read the mission brief, then drive the husky to a "
                            "vantage south of the green pallet (around (3, 3)), "
                            "then return to the loading dock at (-11, 0). "
                            "Then call complete_mission."))
    p.add_argument("--timeout", type=int, default=120)
    p.add_argument("--clear-memory", action="store_true")
    p.add_argument("--max-turns", type=int, default=MAX_TURNS)
    args = p.parse_args()

    omni_key = os.environ.get("OMNI_KEY", "").strip()
    if not omni_key:
        print("Error: set OMNI_KEY"); return 1
    client = OmniLinkClient(omni_key=omni_key, timeout=args.timeout)

    profile = next(
        (p for p in client.list_profiles()
         if (p.get("name") or "").lower() == AGENT_NAME.lower()),
        None,
    )
    if profile is None:
        print(f"Error: profile {AGENT_NAME!r} not found. "
              f"Start warehouse_picker_agent.py first.")
        return 1
    settings = profile.get("settings") or {}
    print(f"[chat] using profile {AGENT_NAME!r} (id={profile.get('id')}) "
          f"with {len(settings.get('availableToolDetails', []))} tools, engine={ENGINE}")

    if args.clear_memory:
        try:
            client.clear_memory(AGENT_NAME)
            print(f"[chat] cleared memory for {AGENT_NAME!r}")
        except Exception as e:
            print(f"[chat] clear_memory failed: {e}")

    pre = fetch_status()
    print(f"[chat] pre-status: {pre.get('complete_calls_this_session', 0)} prior completes, "
          f"{pre.get('waypoints_reached_this_session', 0)} prior waypoints")
    print(f"[chat] sending: {args.prompt!r}")
    print()

    messages = [{"role": "user", "content": args.prompt}]
    turn = 0
    total_tools = 0

    while turn < args.max_turns:
        turn += 1
        t0 = time.time()
        try:
            response = client.chat(
                messages=messages,
                agent_name=AGENT_NAME,
                engine=ENGINE,
                system_instruction=settings,
            )
        except Exception as exc:
            print(f"[chat] turn {turn}: chat() raised: {exc.__class__.__name__}: {exc}")
            return 2
        elapsed = time.time() - t0
        text = (response.get("text") or "").strip()
        tool_calls = response.get("toolCalls") or []
        print(f"[chat] turn {turn}: {elapsed:.1f}s, {len(tool_calls)} tool calls")
        if text:
            short = text[:240] + ("…" if len(text) > 240 else "")
            print(f"        text: {short}")

        if not tool_calls:
            st = fetch_status()
            if st.get("complete_calls_this_session", 0) > pre.get("complete_calls_this_session", 0):
                print(f"[chat] picker claimed mission complete after {turn} turns, {total_tools} tool calls")
                return 0
            print("[chat] no tool calls; nudging")
            messages.append({"role": "assistant", "content": text or "(narration)"})
            messages.append({
                "role": "user",
                "content": ("Don't just narrate. Call your next tool now "
                            "(drive_to_waypoint, get_state, or complete_mission)."),
            })
            continue

        result_lines = []
        image_parts: list = []
        for tc in tool_calls:
            name = tc.get("name", "")
            tc_args = tc.get("arguments") or {}
            result = execute_tool(name, tc_args)
            total_tools += 1
            print(f"        -> {name}({json.dumps(tc_args)[:80]}) -> {_summarise(name, result)[:200]}")
            if name == "read_camera" and isinstance(result, dict) and "image_base64" in result:
                # Strip the base64 from the textual feedback and attach
                # as an inline image part so the engine actually sees the
                # pixels — without this the agent gets opaque base64.
                meta = {k: v for k, v in result.items() if k != "image_base64"}
                # Drop the per-camera dict too — it duplicates image_base64.
                meta.pop("cameras", None)
                result_lines.append(
                    f"`read_camera` returned (image attached as inline part):\n"
                    f"```json\n{json.dumps(meta, default=str)}\n```"
                )
                image_parts.append(result["image_base64"])
            else:
                result_lines.append(f"`{name}` returned:\n```json\n{_summarise(name, result)}\n```")

        st = fetch_status()
        if st.get("complete_calls_this_session", 0) > pre.get("complete_calls_this_session", 0):
            print(f"[chat] picker claimed mission complete in turn {turn}; total tools: {total_tools}")
            return 0

        feedback_text = "\n\n".join(result_lines) + (
            "\n\nNow either drive to the next waypoint, snap a tag, or — if "
            "the mission is satisfied — call complete_mission with a "
            "one-sentence rationale."
        )
        messages.append({"role": "assistant", "content": text or "(working)"})
        if image_parts:
            content = [{"type": "text", "text": feedback_text}]
            for img_b64 in image_parts:
                content.append({
                    "type": "image_url",
                    "image_url": {"url": f"data:image/png;base64,{img_b64}"},
                })
            messages.append({"role": "user", "content": content})
        else:
            messages.append({"role": "user", "content": feedback_text})

    print(f"[chat] hit max_turns={args.max_turns}")
    return 4


if __name__ == "__main__":
    sys.exit(main())
