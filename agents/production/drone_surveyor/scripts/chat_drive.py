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

"""Drive the Drone Surveyor agent via OmniLink chat() with a local tool loop.

Mirrors `agents/production/husky_maze/scripts/chat_drive.py` — the OmniLink
platform can't reach a local toolCallbackUrl from the internet, so this
script runs the chat -> tool-dispatch -> chat loop locally.

Prereqs (in separate terminals, in this order):

    # 1. Start OmniSim with chat/omnilink_mavic.wbt:
    launch.bat projects\\samples\\demos\\worlds\\chat\\omnilink_mavic.wbt

    # 2. Start the agent runner (pushes the profile + opens :51521):
    set OMNI_KEY=olink_YOUR_KEY_HERE
    python agents/production/drone_surveyor/drone_surveyor_agent.py

    # 3. Drive a chat:
    set OMNI_KEY=olink_YOUR_KEY_HERE
    python agents/production/drone_surveyor/scripts/chat_drive.py \\
        "Fly the perimeter, count the red markers, report their positions."
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
from omnilink.usage_meter import UsageMeter  # noqa: E402

AGENT_NAME = "Drone Surveyor"
ENGINE = "g1-engine"
TOOL_PORT = int(os.environ.get("DRONE_AGENT_PORT", "51524"))
TOOL_SERVER_URL = f"http://127.0.0.1:{TOOL_PORT}/tool"
ACTIVITY_URL = f"http://127.0.0.1:{TOOL_PORT}/activity"
BRIDGE_URL = os.environ.get("MAVIC_BRIDGE_URL", "http://127.0.0.1:6090").rstrip("/")
MAX_TURNS = int(os.environ.get("DRONE_CHAT_MAX_TURNS", "120"))


def fetch_state():
    try:
        with urllib.request.urlopen(f"{BRIDGE_URL}/state", timeout=2) as r:
            return json.loads(r.read())
    except Exception:
        return {}


def fetch_activity_count():
    try:
        with urllib.request.urlopen(ACTIVITY_URL, timeout=2) as r:
            return len(json.loads(r.read()).get("entries", []))
    except Exception:
        return None


def execute_tool(name: str, args: dict) -> dict:
    payload = dict(args)
    payload["tool"] = name
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(TOOL_SERVER_URL, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    # 180 s covers a slow goto_waypoint + wait. Land/takeoff with wait=true
    # are also synchronous on the bridge side — pad the timeout above the
    # bridge's own timeout_s so the local read does not give up first.
    try:
        with urllib.request.urlopen(req, timeout=180) as r:
            wrap = json.loads(r.read())
            return wrap.get("result", wrap)
    except Exception as exc:
        return {"error": f"local tool dispatch failed: {exc.__class__.__name__}: {exc}"}


def _summarise_for_agent(tool_name: str, result: dict) -> str:
    """Compress huge tool results so the chat history stays manageable.

    For drone_surveyor the main offender is read_camera (the explicit
    fallback path); chat_drive handles its image_base64 by attaching as
    an inline image part below, so the textual representation here just
    drops it. Most other tools return small JSON already."""
    if tool_name == "read_camera" and isinstance(result, dict):
        # Drop the giant base64 from the textual summary; image is attached
        # separately as a multimodal part.
        slim = {k: v for k, v in result.items() if k != "image_base64"}
        return json.dumps(slim, default=str)
    if tool_name == "scan_for_markers" and isinstance(result, dict):
        # The hint string is large; drop it after the agent has seen it once.
        # We keep the first 3 turns' hint to preserve the framing, then strip.
        # Cheap default: always strip — the system prompt + bridge schema
        # docs already cover the 'world_x/world_y' meaning.
        slim = dict(result)
        slim.pop("hint", None)
        return json.dumps(slim, default=str)
    return json.dumps(result, default=str)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument(
        "prompt", nargs="?",
        default="Fly the perimeter of the warehouse, count the red ground markers, "
                "and report their world (x, y) positions.",
    )
    p.add_argument("--timeout", type=int, default=240,
                   help="HTTP timeout per chat() call (default 240 s)")
    p.add_argument("--clear-memory", action="store_true",
                   help="Clear OmniLink server-side memory before sending")
    p.add_argument("--max-turns", type=int, default=MAX_TURNS,
                   help=f"Max chat turns (default {MAX_TURNS})")
    args = p.parse_args()

    omni_key = os.environ.get("OMNI_KEY", "").strip()
    if not omni_key:
        print("Error: set OMNI_KEY")
        return 1
    client = OmniLinkClient(omni_key=omni_key, timeout=args.timeout)

    profile = next(
        (p for p in client.list_profiles()
         if (p.get("name") or "").lower() == AGENT_NAME.lower()),
        None,
    )
    if profile is None:
        print(f"Error: profile {AGENT_NAME!r} not found. Start drone_surveyor_agent.py first.")
        return 1
    settings = profile.get("settings") or {}
    print(f"[chat] using profile {AGENT_NAME!r} (id={profile.get('id')}) "
          f"with {len(settings.get('availableToolDetails', []))} tools, engine={ENGINE}")

    if args.clear_memory:
        try:
            client.clear_memory(AGENT_NAME)
            print(f"[chat] cleared OmniLink memory for {AGENT_NAME!r}")
        except Exception as e:
            print(f"[chat] clear_memory failed (non-fatal): {e}")

    pre_state = fetch_state()
    print(f"[chat] pre-state: pose=({pre_state.get('x', 0):+.2f},{pre_state.get('y', 0):+.2f},{pre_state.get('z', 0):.1f}m) "
          f"mode={pre_state.get('mode')} mission_complete={pre_state.get('mission_complete')}")

    usage_meter = UsageMeter(client)
    try:
        usage_baseline = usage_meter.start()
        print(f"[chat] usage baseline: input_units_24h={usage_baseline.input_units_24h:,.0f} "
              f"output_units_24h={usage_baseline.output_units_24h:,.0f} "
              f"credits_24h={usage_baseline.credits_24h:.4f}")
    except Exception as e:
        print(f"[chat] usage_meter.start() failed (non-fatal): {e}")
        usage_baseline = None

    run_t0 = time.time()
    print(f"[chat] sending: {args.prompt!r}")
    print()

    def _compact_old_tool_messages(messages, keep_recent_turns=4):
        """Replace tool-result text in user messages older than the last
        keep_recent_turns rounds with a one-line summary. Same shape as
        husky_maze's chat_drive — long scan/state dumps from many turns
        ago add no value once the agent has moved on."""
        n = len(messages)
        if n < 6:
            return
        asst_seen = 0
        boundary = 0
        for i in range(n - 1, 0, -1):
            if messages[i].get("role") == "assistant":
                asst_seen += 1
                if asst_seen >= keep_recent_turns:
                    boundary = i
                    break
        if boundary <= 1:
            return
        import re as _re
        for j in range(1, boundary):
            msg = messages[j]
            if msg.get("role") != "user":
                continue
            content = msg.get("content")
            if not isinstance(content, str) or not content.startswith("`"):
                continue
            if content.startswith("[older turn"):
                continue
            tools_called = _re.findall(r"`([a-z_][a-z0-9_]*)` returned:", content)
            tools_str = ", ".join(tools_called) if tools_called else "tools"
            msg["content"] = f"[older turn — {tools_str} — full result elided]"

    messages = [{"role": "user", "content": args.prompt}]
    turn = 0
    total_tools = 0

    while turn < args.max_turns:
        turn += 1
        _compact_old_tool_messages(messages, keep_recent_turns=4)
        t0 = time.time()
        response = None
        last_exc = None
        for attempt in range(4):
            try:
                response = client.chat(
                    messages=messages,
                    agent_name=AGENT_NAME,
                    engine=ENGINE,
                    system_instruction=settings,
                )
                break
            except Exception as exc:
                last_exc = exc
                msg = str(exc)
                if "429" in msg or "rate_limit" in msg.lower() or "PRIMARY_ENGINE_FAILED" in msg:
                    if attempt < 3:
                        print(f"[chat] turn {turn}: 429 on attempt {attempt+1}/4, sleeping 60s")
                        time.sleep(60)
                        continue
                break
        if response is None:
            print(f"[chat] turn {turn}: chat() raised after retries: {last_exc.__class__.__name__}: {last_exc}")
            return 2
        elapsed = time.time() - t0
        text = (response.get("text") or "").strip()
        tool_calls = response.get("toolCalls") or []
        st = fetch_state()
        print(f"[chat] turn {turn}: {elapsed:.1f}s, {len(tool_calls)} tool calls, "
              f"pose=({st.get('x', 0):+.1f},{st.get('y', 0):+.1f},{st.get('z', 0):.1f}m) "
              f"mode={st.get('mode')} mission_complete={st.get('mission_complete')}")
        if text:
            short = text[:200] + ("..." if len(text) > 200 else "")
            print(f"        text: {short}")

        if st.get("mission_complete") and not tool_calls:
            wall_clock_s = time.time() - run_t0
            print(f"[chat] MISSION COMPLETE after {turn} chat turns, {total_tools} tool calls, "
                  f"{wall_clock_s:.1f}s wall clock")
            _print_usage_summary(usage_meter, usage_baseline, turn)
            return 0

        if not tool_calls:
            print("[chat] no tool calls — nudging agent to act")
            messages.append({"role": "assistant", "content": text or "(narration)"})
            if st.get("mode") == "landed" and not st.get("mission_complete"):
                nudge = (
                    "The drone is on the ground (mode=landed). Now finish the "
                    "mission: call complete_mission with a one-sentence "
                    "rationale and a payload {target_count, target_positions, "
                    "all_detections}, then call save_local_memory to persist "
                    "the working pattern for future sessions. Do both, in "
                    "that order."
                )
            else:
                nudge = (
                    "Do not narrate. Call the next tool now. The drone is at "
                    f"pose ({st.get('x',0):+.2f},{st.get('y',0):+.2f},{st.get('z',0):.1f}m) "
                    f"mode {st.get('mode')} mission_complete {st.get('mission_complete')}. "
                    "Pick exactly one tool and call it now."
                )
            messages.append({"role": "user", "content": nudge})
            continue

        messages.append({"role": "assistant", "content": text or "(working)"})

        result_lines = []
        image_parts = []
        for tc in tool_calls:
            name = tc.get("name", "")
            tc_args = tc.get("arguments") or {}
            result = execute_tool(name, tc_args)
            total_tools += 1
            r_str = json.dumps(result, default=str)
            preview = r_str[:120]
            print(f"        -> {name}({json.dumps(tc_args)[:60]}) = {preview}")
            if name == "read_camera" and isinstance(result, dict) and "image_base64" in result:
                meta = {k: v for k, v in result.items() if k != "image_base64"}
                result_lines.append(
                    f"`read_camera` returned (image attached as inline part):\n"
                    f"```json\n{json.dumps(meta, default=str)}\n```"
                )
                image_parts.append(result["image_base64"])
            else:
                r_for_agent = _summarise_for_agent(name, result)
                result_lines.append(f"`{name}` returned:\n```json\n{r_for_agent}\n```")
        feedback_text = (
            "\n\n".join(result_lines)
            + "\n\nNow call the next tool. Don't stop or summarise — keep "
              "calling tools until you've satisfied the mission and called "
              "complete_mission."
        )
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

    wall_clock_s = time.time() - run_t0
    print(f"[chat] hit max_turns={args.max_turns} without mission_complete "
          f"({wall_clock_s:.1f}s wall clock)")
    _print_usage_summary(usage_meter, usage_baseline, args.max_turns)
    return 4


def _print_usage_summary(meter, baseline, turns_used):
    if meter is None or baseline is None:
        return
    try:
        delta = meter.snapshot(baseline=baseline)
    except Exception as exc:
        print(f"[chat] usage_meter.snapshot() failed: {exc}")
        return
    print(f"[chat] {delta.report(prefix='usage: ')}")
    if turns_used:
        per_turn = delta.total_units / max(1, turns_used)
        print(f"[chat] avg tokens/turn: {per_turn:,.0f}")


if __name__ == "__main__":
    sys.exit(main())
