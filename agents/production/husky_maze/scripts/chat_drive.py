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

"""Drive the Husky Maze agent via OmniLink chat() with a local tool loop.

OmniLink's `chat` endpoint is one-shot: it returns text + a list of
intended tool calls. The OmniLink web UI runs the iteration loop, but
the platform can't reach a local `toolCallbackUrl` like
`http://127.0.0.1:51517/tool` from the internet. So for programmatic
local runs we run the loop here:

    1. Send the user prompt.
    2. Receive {text, toolCalls?}.
    3. If toolCalls is empty -> done.
    4. Execute each tool call locally (POST to the runner's /tool URL).
    5. Append assistant message + tool result messages.
    6. Send chat again with the updated messages array.
    7. Goto 2.

Usage:
    OMNI_KEY=olink_... HUSKY_BRIDGE_URL=http://127.0.0.1:6070 \
        python agents/production/husky_maze/scripts/chat_drive.py \
            "Solve the maze. Drive the husky to the goal."

The agent runner (husky_maze_agent.py) must already be running so its
profile is on the platform and its tool-server is up locally.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.request
from pathlib import Path

# Force UTF-8 on Windows stdout/stderr so the agent's prose narration
# (which routinely includes π, ±, →, ° etc.) doesn't crash the loop on
# the default cp1252 console codec. Mirrors husky_maze_agent.py.
for _stream in (sys.stdout, sys.stderr):
    try:
        # line_buffering=True so the per-turn `[chat] turn N: ...` lines
        # become visible to operators and Monitor watchers in real time.
        # Without this, Python defaults to fully-buffered stdout on
        # Windows-bash invocations and the run looks frozen for minutes.
        _stream.reconfigure(encoding="utf-8", errors="backslashreplace", line_buffering=True)  # type: ignore[attr-defined]
    except Exception:
        pass

REPO_ROOT = Path(__file__).resolve().parents[3]
LIB_SRC = REPO_ROOT.parent / "olink" / "omnilink-lib" / "src"
if LIB_SRC.exists() and str(LIB_SRC) not in sys.path:
    sys.path.insert(0, str(LIB_SRC))

from omnilink.client import OmniLinkClient  # noqa: E402
from omnilink.usage_meter import UsageMeter  # noqa: E402

def _parse_variant() -> str:
    v = ""
    for i, a in enumerate(sys.argv):
        if a == "--variant" and i + 1 < len(sys.argv):
            v = sys.argv[i + 1]
            sys.argv.pop(i); sys.argv.pop(i); break
        if a.startswith("--variant="):
            v = a.split("=", 1)[1]
            sys.argv.pop(i); break
    if not v:
        v = os.environ.get("HUSKY_VARIANT", "v1").strip()
    if v not in {"v1", "v2", "v3"}:
        raise SystemExit(f"Unknown variant {v!r}. Use one of v1, v2, v3.")
    return v


VARIANT = _parse_variant()
_AGENT_NAMES = {"v1": "Husky Maze", "v2": "Husky Maze v2", "v3": "Husky Maze v3"}
_PORTS = {"v1": 51517, "v2": 51518, "v3": 51519}
AGENT_NAME = _AGENT_NAMES[VARIANT]
ENGINE = "g1-engine"  # OmniSim-default; g2-engine requires BYOK OpenAI key.
TOOL_SERVER_URL = f"http://127.0.0.1:{_PORTS[VARIANT]}/tool"
ACTIVITY_URL = f"http://127.0.0.1:{_PORTS[VARIANT]}/activity"
BRIDGE_URL = os.environ.get("HUSKY_BRIDGE_URL", "http://127.0.0.1:6070").rstrip("/")
MAX_TURNS = int(os.environ.get("HUSKY_CHAT_MAX_TURNS", "150"))


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
    """Send a tool call to the runner's local tool server."""
    payload = dict(args)
    payload["tool"] = name
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(TOOL_SERVER_URL, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    # 45 s is plenty for v1/v2 single-cell goto_cell, but v3's execute_path
    # drives the full BFS plan in one bridge call (~5 min wall-clock for the
    # seed-7 maze). Bump well above the bridge's worst-case so the local HTTP
    # read does not give up before the bridge returns.
    try:
        with urllib.request.urlopen(req, timeout=900) as r:
            wrap = json.loads(r.read())
            return wrap.get("result", wrap)
    except Exception as exc:
        return {"error": f"local tool dispatch failed: {exc.__class__.__name__}: {exc}"}


def _summarise_for_agent(tool_name: str, result: dict) -> str:
    """Compress huge tool results so the chat history stays manageable."""
    if tool_name == "recall" and isinstance(result, dict):
        # Recall hits store the BFS path inside the markdown body of each
        # long_term memory under a "Full ordered path: [[c,r], ...]" line.
        # Pulling it back out of free-text via the LLM is fragile — Gemini
        # truncated a 72-cell path to 61 cells in one observed run, which
        # wedged execute_path. Parse the path eagerly here and attach it
        # as a structured `extracted_paths` field the agent can pass to
        # execute_path verbatim.
        import re as _re
        try:
            tiers = result.get("tiers") or {}
            long_term = tiers.get("long_term") or []
            extracted = []
            for hit in long_term:
                body = hit.get("body") or ""
                m = _re.search(r"Full ordered path:\s*(\[\[.*?\]\])", body, flags=_re.DOTALL)
                if m:
                    try:
                        path = json.loads(m.group(1))
                        if isinstance(path, list) and path and all(
                            isinstance(c, list) and len(c) == 2 for c in path
                        ):
                            extracted.append({
                                "title": hit.get("title"),
                                "tags": hit.get("tags"),
                                "path": [[int(c[0]), int(c[1])] for c in path],
                                "path_length": len(path),
                            })
                    except Exception:
                        pass
            if extracted:
                # Attach extracted_paths but leave the original tiers intact
                # so the agent still sees other body fields it might need.
                summarised = dict(result)
                summarised["extracted_paths"] = extracted
                summarised["_note"] = (
                    "extracted_paths was parsed by the driver from the "
                    "long_term-tier hits' Full ordered path line. To replay "
                    "a stored plan with execute_path, pass "
                    "`cells = extracted_paths[i].path[1:]` (drop the start "
                    "cell). Use this verbatim — do not retype from the body."
                )
                return json.dumps(summarised, default=str)
        except Exception:
            pass
        return json.dumps(result, default=str)

    if tool_name in ("walk_one_cell", "follow_corridor") and isinstance(result, dict):
        # Drop the verbose `final_pose` floats and the empty `first_refusal`
        # field on success — the digest's `current_cell` is what the agent
        # actually navigates from. Saves ~30% of the response size per turn.
        slim = dict(result)
        slim.pop("final_pose", None)
        if not slim.get("first_refusal"):
            slim.pop("first_refusal", None)
        return json.dumps(slim, default=str)
    if tool_name == "try_get_known_map" and isinstance(result, dict) and result.get("available"):
        # Pre-compute the BFS shortest path (for single-goal missions like
        # the BFS demo, that's all the agent needs). KEEP the adjacency
        # dict intact for multi-leg missions like the corners tour where
        # the agent has to plan path-N→path-N+1 itself: without
        # adjacency it gets only the start→goal path and re-tries the
        # same broken plan when a leg hits a wall it didn't know about.
        # Cost: ~5 KB / ~1.3 K tokens extra per call (typically 1-2 calls
        # per mission). Cheap insurance for multi-leg flexibility.
        from collections import deque
        adj = result.get("adjacency") or {}
        start = result.get("start") or {}
        goal = result.get("goal") or {}
        s = (start.get("col"), start.get("row"))
        g = (goal.get("col"), goal.get("row"))
        seen = {s: None}
        q = deque([s])
        while q:
            here = q.popleft()
            if here == g:
                break
            for nb in adj.get(f"{here[0]},{here[1]}", []):
                t = (nb[0], nb[1])
                if t in seen:
                    continue
                seen[t] = here
                q.append(t)
        path = []
        cur = g
        while cur is not None and cur in seen:
            path.append(cur)
            cur = seen[cur]
        path.reverse()
        return json.dumps({
            "available": True,
            "world_title": result.get("world_title"),
            "start": start,
            "goal": goal,
            "shortest_path": [list(p) for p in path],
            "path_length": len(path),
            "adjacency": adj,
            "_note": (
                "shortest_path is the BFS plan from start to the legacy goal cell. "
                "For multi-leg missions (e.g. corners tour), use the adjacency "
                "dict to BFS from your current cell to each next destination. "
                "Adjacency keys are 'col,row' strings; values are lists of "
                "open 4-neighbour [col, row] pairs. Re-plan from current pose "
                "after every fault — never retry a path that just hit a wall."
            ),
        })
    return json.dumps(result, default=str)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("prompt", nargs="?",
                   default="Solve the maze. Drive the husky from its current cell to the goal.")
    p.add_argument("--timeout", type=int, default=120,
                   help="HTTP timeout per chat() call (default 120 s)")
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
        print(f"Error: profile {AGENT_NAME!r} not found. Start husky_maze_agent.py first.")
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
    print(f"[chat] pre-state: pose=({pre_state.get('x', 0):+.2f},{pre_state.get('y', 0):+.2f}) "
          f"goal_reached={pre_state.get('goal_reached')}")

    # Snapshot the platform's usage rollup BEFORE the run so we can report
    # tokens-per-hour and credits-per-hour against just this run's window.
    # The meter polls /api/omni-key-usage (cheap GET) — it does not parse
    # individual chat() responses.
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
        keep_recent_turns rounds with a one-line summary. Each round is one
        (assistant, user-with-results) pair after the initial user prompt.
        Compounded scan/walk results from many turns earlier add no value
        once the agent has moved on — `visited_cells` and `current_cell`
        in the latest scan carry the only state the agent actually
        navigates from."""
        n = len(messages)
        if n < 6:  # need ≥ keep_recent_turns rounds before compaction is useful
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
        for j in range(1, boundary):  # skip the operator's initial prompt at 0
            msg = messages[j]
            if msg.get("role") != "user":
                continue
            content = msg.get("content")
            # Multimodal content (list of parts): drop every image_url part
            # past the boundary. The tags-driven `scan_surroundings` flow
            # keeps text only, but a stray `read_camera` call would attach
            # a 100 KB+ PNG that compounds turn-over-turn; strip it.
            if isinstance(content, list):
                kept = [p for p in content if p.get("type") != "image_url"]
                if len(kept) != len(content):
                    msg["content"] = kept if kept else "[older turn — image elided]"
                # Continue to text-summarisation below if any text remains.
                if isinstance(msg["content"], list):
                    continue
                content = msg["content"]
            if not isinstance(content, str) or not content.startswith("`"):
                continue
            if content.startswith("[older turn"):
                continue
            tools_called = _re.findall(r"`([a-z_][a-z0-9_]*)` returned:", content)
            cells_seen = _re.findall(r"\"to_cell\":\s*\[(\d+),\s*(\d+)\]", content)
            cells_str = ""
            if cells_seen:
                tail = cells_seen[-1]
                cells_str = f" reached ({tail[0]},{tail[1]})"
            tools_str = ", ".join(tools_called) if tools_called else "tools"
            msg["content"] = f"[older turn — {tools_str}{cells_str} — full result elided]"

    # Tool-loop. The OmniLink chat API validates OpenAI-style
    # {role, content} messages, but server-side conversion to the engine's
    # native function-calling format (Gemini parts, OpenAI tool_calls,
    # etc.) is fragile in the round-trip case. So we keep the wire format
    # as plain user/assistant text and embed tool results as text inside
    # follow-up user messages. The agent still emits structured toolCalls
    # in each response — we just feed results back as text. Less precise
    # than function-calling round-trips but works across engines.
    messages = [{"role": "user", "content": args.prompt}]
    turn = 0
    total_tools = 0
    # Loop-detector: tracks the last few (tool_name, canonical_args) pairs.
    # When the agent re-issues an identical call twice in a row, we inject
    # a hard nudge into the next user message. Three identical calls in a
    # row abort the run so we don't burn credits. Identity is computed off
    # the JSON-sorted argument dict, so re-orderings of the same args still
    # match. Pose changes between calls do NOT reset this — if the agent
    # asked the same thing without moving, it's stuck.
    recent_calls: list = []
    LOOP_NUDGE_AT = 2     # 2 same-call-in-a-row -> nudge
    LOOP_ABORT_AT = 4     # 4 same-call-in-a-row -> abort
    # Track pose at start of each turn. If pose is identical between turns
    # AND tool result is identical, the agent is genuinely spinning.
    last_pose_signature: str = ""

    while turn < args.max_turns:
        turn += 1
        _compact_old_tool_messages(messages, keep_recent_turns=4)
        t0 = time.time()
        # Retry transient OmniLink rate-limits in-place. The shared
        # g1-engine pool's per-account window can fire mid-run; the
        # platform's own message says "Retry in ~1 min", so up to 4
        # retries with 60 s gaps lets a single 429 not kill the whole
        # chat_drive run.
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
                # non-429 or final attempt: bail
                break
        if response is None:
            print(f"[chat] turn {turn}: chat() raised after retries: {last_exc.__class__.__name__}: {last_exc}")
            return 2
        elapsed = time.time() - t0
        text = (response.get("text") or "").strip()
        tool_calls = response.get("toolCalls") or []
        st = fetch_state()
        print(f"[chat] turn {turn}: {elapsed:.1f}s, {len(tool_calls)} tool calls, "
              f"pose=({st.get('x', 0):+.1f},{st.get('y', 0):+.1f}) "
              f"goal={st.get('goal_reached')}")
        if text:
            short = text[:200] + ("..." if len(text) > 200 else "")
            print(f"        text: {short}")

        # Exit conditions: mission_complete is the agent's *honest* assertion
        # of completion (set by complete_mission). goal_reached is the
        # bridge's geometric proxy for the legacy world. We let the agent
        # finish closeout (complete_mission + save_local_memory) before
        # returning, otherwise direction-A memory compounding cannot work.
        if st.get("mission_complete") and not tool_calls:
            wall_clock_s = time.time() - run_t0
            print(f"[chat] MISSION COMPLETE after {turn} chat turns, {total_tools} tool calls, "
                  f"{wall_clock_s:.1f}s wall clock")
            _print_usage_summary(usage_meter, usage_baseline, turn)
            return 0

        if not tool_calls:
            # Agent narrated without a tool call. If goal is already reached
            # but mission_complete is still false, nudge specifically toward
            # closeout so the agent calls complete_mission + save_local_memory
            # rather than chasing more navigation.
            print("[chat] no tool calls — nudging agent to act")
            messages.append({"role": "assistant", "content": text or "(narration)"})
            if st.get("goal_reached") and not st.get("mission_complete"):
                nudge = (
                    "The husky has reached the goal cell — `state.goal_reached` "
                    "is true. Now finish the mission: call `complete_mission` "
                    "with a one-sentence rationale, then call `save_local_memory` "
                    "to persist the working plan for future sessions. Do both, "
                    "in that order."
                )
            else:
                nudge = (
                    "Do not narrate. Call the next tool now. The bridge is "
                    f"at pose ({st.get('x',0):+.2f},{st.get('y',0):+.2f}), "
                    f"yaw {st.get('yaw',0):+.2f}, mode {st.get('mode')}, "
                    f"goal_reached {st.get('goal_reached')}. "
                    "Pick exactly one tool and call it now."
                )
            messages.append({"role": "user", "content": nudge})
            continue

        # Record assistant turn as plain narration (no "Tool calls:" line —
        # that confuses the model's next planning step).
        messages.append({"role": "assistant", "content": text or "(working)"})

        # Execute every tool locally, gather results. Large results
        # (the maze adjacency dict is ~5 KB) get summarised so the
        # agent's next turn isn't drowned in JSON. Camera frames are
        # attached as inline image parts so the engine actually sees
        # the pixels — without this the agent gets opaque base64.
        result_lines = []
        image_parts = []
        # Build a signature for THIS turn's tool calls (sorted JSON of
        # name+args). Compare against recent turns to detect loops.
        turn_signature_parts = []
        for tc in tool_calls:
            n = tc.get("name", "")
            a = tc.get("arguments") or {}
            turn_signature_parts.append(f"{n}({json.dumps(a, sort_keys=True)})")
        turn_signature = "|".join(turn_signature_parts)
        # Pose signature is CELL-level, not pose-level. The wheel controller
        # can wiggle the husky 0.01-0.05 m during wedge escape without making
        # real progress; coordinate-precision pose signatures don't catch
        # this and the loop detector never aborts. Cell coords flip only
        # when the husky actually moves to a new tile, which is the only
        # progress that matters here.
        cc = st.get("current_cell") or {}
        pose_signature = f"cell={cc.get('col')},{cc.get('row')}"
        # A repeat counts only when both the calls AND the pose are
        # unchanged — a moving husky calling plan_path twice is fine.
        is_repeat = bool(
            recent_calls
            and recent_calls[-1] == turn_signature
            and pose_signature == last_pose_signature
        )
        if is_repeat:
            recent_calls.append(turn_signature)
        else:
            recent_calls = [turn_signature]
        last_pose_signature = pose_signature
        repeat_count = len(recent_calls)
        if repeat_count >= LOOP_ABORT_AT:
            print(f"[chat] LOOP ABORT: same tool call ({turn_signature[:80]}) "
                  f"issued {repeat_count}x with zero pose change. "
                  f"Aborting to save credits.")
            return 5
        for tc in tool_calls:
            name = tc.get("name", "")
            tc_args = tc.get("arguments") or {}
            result = execute_tool(name, tc_args)
            total_tools += 1
            r_str = json.dumps(result, default=str)
            preview = r_str[:120]
            print(f"        -> {name}({json.dumps(tc_args)[:60]}) = {preview}")
            if name == "read_camera" and isinstance(result, dict) and "image_base64" in result:
                # Strip the giant base64 from the textual feedback and
                # attach the image as a separate inline part instead.
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
        if repeat_count >= LOOP_NUDGE_AT:
            feedback_text += (
                f"\n\n[LOOP DETECTED] You just made the same tool call "
                f"{repeat_count} turns in a row and the husky's pose has NOT "
                f"changed. The result you keep getting will not change unless "
                f"you change what you do. Pick a DIFFERENT next action: read "
                f"`get_state` to learn the husky's actual current cell, then "
                f"call `plan_path` from THAT cell to your next waypoint, then "
                f"pass the returned `execute_path_cells` straight into "
                f"`execute_path`. Do NOT repeat the same call. "
                f"Two more repeats and this run aborts."
            )
        if image_parts:
            # OpenAI-style multimodal content: a list of parts where each
            # part is {type: text, text} or {type: image_url, image_url:
            # {url: 'data:image/png;base64,...'}}. OmniLink forwards this
            # to vision-capable engines (g1-gemini, g4-claude) which
            # decode the inline images.
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
    print(f"[chat] hit max_turns={args.max_turns} without goal_reached "
          f"({wall_clock_s:.1f}s wall clock)")
    _print_usage_summary(usage_meter, usage_baseline, args.max_turns)
    return 4


def _print_usage_summary(meter, baseline, turns_used):
    """Read the usage delta and print a single-line summary suitable for
    operator-facing output. Cost-per-hour is the headline metric; raw
    token counts let the operator sanity-check."""
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
