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

"""Drive the Warehouse Foreman agent via OmniLink chat() with a local
tool loop.

Same architecture as agents/production/mission_captain/scripts/chat_drive.py.
The Foreman's `delegate_to_agent` runs its own sub-chat-loop against
the Picker, so a single Foreman delegation can take dozens of seconds
(the Foreman's chat ticks once, but its delegation ticks the Picker
end-to-end before returning a single result).

Usage:
    OMNI_KEY=olink_... python agents/production/warehouse_foreman/scripts/chat_drive.py \\
        "Move the green pallet to the loading dock."

The Warehouse Foreman runner (warehouse_foreman_agent.py) must already
be running, AND every specialist runner the Foreman might delegate to
(currently: Warehouse Picker on :51520).
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

AGENT_NAME = "Warehouse Foreman"
ENGINE = "g1-engine"
_FOREMAN_PORT = int(os.environ.get("FOREMAN_PORT", "51521"))
TOOL_SERVER_URL = f"http://127.0.0.1:{_FOREMAN_PORT}/tool"
ACTIVITY_URL = f"http://127.0.0.1:{_FOREMAN_PORT}/activity"
STATUS_URL = f"http://127.0.0.1:{_FOREMAN_PORT}/status"
MAX_TURNS = int(os.environ.get("FOREMAN_CHAT_MAX_TURNS", "20"))


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
    # Per-tool timeout. delegate_to_agent runs whole sub-missions which
    # can take 5-15 min for a Picker run that drives across the warehouse
    # twice (e.g. red vantage -> green vantage -> dock); set generously.
    timeout = int(os.environ.get("FOREMAN_DRIVE_TOOL_TIMEOUT", "1800"))
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            wrap = json.loads(r.read())
            return wrap.get("result", wrap)
    except Exception as exc:
        return {"error": f"local tool dispatch failed: {exc.__class__.__name__}: {exc}"}


def _summarise(name: str, result: dict) -> str:
    if name == "delegate_to_agent" and isinstance(result, dict):
        compact = {
            k: v for k, v in result.items()
            if k in ("success", "agent", "turns", "tool_calls", "mission_complete", "final_text", "error", "reason")
        }
        return json.dumps(compact, default=str)[:1200]
    return json.dumps(result, default=str)[:800]


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("prompt", nargs="?",
                   default="Move the green-tagged pallet to the loading dock.")
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
              f"Start warehouse_foreman_agent.py first.")
        return 1
    settings = profile.get("settings") or {}
    print(f"[chat] using profile {AGENT_NAME!r} (id={profile.get('id')}) "
          f"with {len(settings.get('availableToolDetails', []))} tools, engine={ENGINE}")

    if args.clear_memory:
        # Clear the foreman's memory AND every specialist the foreman knows
        # about. Without this, a Picker that completed the same mission in
        # an earlier session would see its own complete_mission rationale in
        # OmniLink's long-term memory and short-circuit the new delegation
        # by re-claiming completion before doing any actual work.
        for name in (AGENT_NAME, "Warehouse Picker", "Axis"):
            try:
                client.clear_memory(name)
                print(f"[chat] cleared memory for {name!r}")
            except Exception as e:
                print(f"[chat] clear_memory({name!r}) failed: {e}")

    pre = fetch_status()
    print(f"[chat] pre-status: {pre.get('foreman_complete_calls_this_session', 0)} prior foreman claims, "
          f"specialists={pre.get('specialists_known', [])}")
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
            if st.get("foreman_complete_calls_this_session", 0) > pre.get("foreman_complete_calls_this_session", 0):
                print(f"[chat] foreman claimed mission complete after {turn} turns, {total_tools} tool calls")
                return 0
            print("[chat] no tool calls; nudging")
            messages.append({"role": "assistant", "content": text or "(narration)"})
            messages.append({
                "role": "user",
                "content": ("Don't just narrate. Call your next tool now "
                            "(read_mission_brief, delegate_to_agent, or complete_mission)."),
            })
            continue

        result_lines = []
        for tc in tool_calls:
            name = tc.get("name", "")
            tc_args = tc.get("arguments") or {}
            result = execute_tool(name, tc_args)
            total_tools += 1
            print(f"        -> {name}({json.dumps(tc_args)[:80]}) -> {_summarise(name, result)[:200]}")
            result_lines.append(f"`{name}` returned:\n```json\n{_summarise(name, result)}\n```")

        st = fetch_status()
        if st.get("foreman_complete_calls_this_session", 0) > pre.get("foreman_complete_calls_this_session", 0):
            print(f"[chat] foreman claimed mission complete in turn {turn}; total tools: {total_tools}")
            return 0

        feedback = "\n\n".join(result_lines) + (
            "\n\nNow either delegate the next leg, or — if every leg has "
            "succeeded — call complete_mission with a one-sentence rationale."
        )
        messages.append({"role": "assistant", "content": text or "(working)"})
        messages.append({"role": "user", "content": feedback})

    print(f"[chat] hit max_turns={args.max_turns}")
    return 4


if __name__ == "__main__":
    sys.exit(main())
