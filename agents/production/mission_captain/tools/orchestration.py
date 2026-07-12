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

"""Orchestration tools: delegate_to_agent, query_agent_status, list_agents.

These are the captain's ONLY actuation surface. The captain doesn't call
robot motion tools directly — it delegates to specialists who own them.

`delegate_to_agent` is the centrepiece. It runs a sub-chat-loop locally
against the named specialist's profile, dispatches the specialist's tool
calls against the specialist's local /tool endpoint, and returns when
the specialist either claims completion or hits a max-turn limit.

This is local orchestration (not OmniLink server-side delegation) because
the OmniLink platform can't reach 127.0.0.1:51517 from the internet to
invoke our local tool server. The captain runs the sub-loop ourselves.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from ._base import ALWAYS, GUARDED, SAFE, SPECIALIST_REGISTRY, ToolSpec, http_get, http_post

# OmniLinkClient is the platform handle the captain uses to chat as the
# specialist. We import it once and reuse across delegations.
_LIB_SRC = Path(__file__).resolve().parents[3].parent / "olink" / "omnilink-lib" / "src"
if _LIB_SRC.exists() and str(_LIB_SRC) not in sys.path:
    sys.path.insert(0, str(_LIB_SRC))
try:
    from omnilink.client import OmniLinkClient  # type: ignore
except Exception as exc:
    OmniLinkClient = None  # type: ignore[assignment]
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None

DEFAULT_ENGINE = os.environ.get("CAPTAIN_DEFAULT_ENGINE", "g1-engine")
SUB_AGENT_MAX_TURNS = int(os.environ.get("CAPTAIN_SUB_MAX_TURNS", "30"))
# Per-chat timeout. OmniLink's server-side default is 120 s; higher values
# get the long-mission story working at the cost of slower failure-to-give-up.
SUB_AGENT_CHAT_TIMEOUT = int(os.environ.get("CAPTAIN_SUB_CHAT_TIMEOUT", "180"))


# ──────────────────────────────────────────────────────────────────────
# Specialist registry / status
# ──────────────────────────────────────────────────────────────────────

def _impl_list_agents(**_: Any) -> Dict[str, Any]:
    """Return the captain's specialist roster + reachability."""
    out: List[Dict[str, Any]] = []
    for name, info in SPECIALIST_REGISTRY.items():
        st = http_get(info["runner_status_url"], timeout=1.5)
        reachable = "error" not in st
        out.append({
            "name": name,
            "what_it_does": info["what_it_does"],
            "reachable": reachable,
            "status": st if reachable else None,
            "status_error": st.get("error") if not reachable else None,
        })
    return {"status": "ok", "specialists": out, "count": len(out)}


def _impl_query_agent_status(agent: str = "", **_: Any) -> Dict[str, Any]:
    if not agent:
        return {"error": "agent name is required"}
    info = SPECIALIST_REGISTRY.get(agent)
    if info is None:
        return {
            "error": f"unknown specialist {agent!r}",
            "known": sorted(SPECIALIST_REGISTRY.keys()),
        }
    return http_get(info["runner_status_url"], timeout=2.0)


# ──────────────────────────────────────────────────────────────────────
# delegate_to_agent — local sub-chat-loop with the named specialist
# ──────────────────────────────────────────────────────────────────────

def _summarise_result(result: Any, max_len: int = 240) -> str:
    s = json.dumps(result, default=str) if not isinstance(result, str) else result
    return s if len(s) <= max_len else s[:max_len] + "…"


_ABBREVS_FIRST_SENT = {"e.g", "i.e", "etc", "vs", "a.m", "p.m", "mr", "mrs", "dr"}


def _first_sentence(text: str) -> str:
    """Trim a tool description to its first sentence — same heuristic as
    husky_maze's tools/_base.py:_first_sentence. Used to slim
    availableToolDetails before forwarding to OmniLink so the sub-chat
    request doesn't exceed the platform's 413 payload limit."""
    import re as _re
    s = (text or "").strip()
    for m in _re.finditer(r"[.!?]", s):
        end = m.end()
        pre = s[:m.start()]
        last_word = _re.search(r"(\w+)$", pre)
        if last_word and last_word.group(1).lower() in _ABBREVS_FIRST_SENT:
            continue
        tail = s[end:]
        if not tail or tail[0].isspace():
            after = tail.lstrip()
            if not after or after[0].isupper() or after[0].isdigit():
                return s[:end].strip()
    return s


def _slim_settings_for_subchat(settings: Dict[str, Any]) -> Dict[str, Any]:
    """Build a sub-chat-safe copy of the specialist's profile settings.

    The captain pushes/forwards each specialist's full profile settings
    (including `availableToolDetails` with verbose, multi-paragraph
    descriptions for every tool). When delegate_to_agent opens a chat
    with that as system_instruction, the cumulative payload often
    exceeds OmniLink's 413 request-size limit — we observed every
    delegation 413-ing on a 20-tool Husky Maze surface after directions
    A and C added prose to the prompts.

    The model still needs to know what tools exist (to emit structured
    tool_calls), so we keep `availableToolDetails` but compress each
    description to its first sentence. Callable parameters and tool
    names survive untouched. We also drop `toolCallbackUrl` (irrelevant
    in the local sub-loop) and any `standingOrders` (the captain runs
    the loop, not OmniLink's scheduler)."""
    if not isinstance(settings, dict):
        return settings
    slim = dict(settings)
    details = slim.get("availableToolDetails")
    if isinstance(details, list):
        slim_details = []
        for spec in details:
            if not isinstance(spec, dict):
                slim_details.append(spec)
                continue
            d = dict(spec)
            desc = d.get("description")
            if isinstance(desc, str) and len(desc) > 200:
                d["description"] = _first_sentence(desc)
            slim_details.append(d)
        slim["availableToolDetails"] = slim_details
    slim.pop("toolCallbackUrl", None)
    slim.pop("standingOrders", None)
    return slim


def _summarise_for_subagent(tool_name: str, result: Any) -> str:
    """Compress huge tool results for the sub-agent's next turn so its
    chat() doesn't blow past the OmniLink 120 s server timeout. Same
    treatment as husky_maze/scripts/chat_drive.py applies to known-map
    payloads — replace the 5 KB adjacency dict with the BFS path."""
    if tool_name == "try_get_known_map" and isinstance(result, dict) and result.get("available"):
        try:
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
                "_note": "Adjacency dict elided; shortest_path is the BFS plan to execute cell by cell.",
            })
        except Exception:
            pass
    return json.dumps(result, default=str)


def _impl_delegate_to_agent(
    agent: str = "",
    task: str = "",
    max_turns: int = 0,
    **_: Any,
) -> Dict[str, Any]:
    """Run a sub-chat-loop against the named specialist and return its result.

    Uses the SAME sub-agent's pushed profile (settings) as system instruction
    so the sub-agent has access to all its own tools. Dispatches every tool
    call against the sub-agent's local /tool endpoint (the runner's
    tool-callback server). Loops until the sub-agent emits no more tool
    calls (it's done) or `max_turns` is hit.

    Returns:
        {
            success: bool,           # true iff the sub-agent terminated normally
            agent: str,              # the sub-agent name
            turns: int,              # how many chat turns it took
            tool_calls: int,         # how many tool calls were dispatched
            final_text: str,         # sub-agent's last narration
            mission_complete: bool,  # taken from sub-agent's /status if available
            sub_status: dict | None, # sub-agent's /status snapshot at the end
        }
    """
    if OmniLinkClient is None:
        return {"error": f"omnilink-lib not importable: {_IMPORT_ERROR}"}
    if not agent or not agent.strip():
        return {"error": "agent name is required"}
    if not task or not task.strip():
        return {"error": "task is required (a self-contained instruction the specialist can execute)"}
    info = SPECIALIST_REGISTRY.get(agent)
    if info is None:
        return {
            "error": f"unknown specialist {agent!r}",
            "known": sorted(SPECIALIST_REGISTRY.keys()),
        }

    omni_key = os.environ.get("OMNI_KEY", "").strip()
    if not omni_key:
        return {"error": "OMNI_KEY not set in captain process environment"}
    client = OmniLinkClient(omni_key=omni_key, timeout=SUB_AGENT_CHAT_TIMEOUT)

    profile = next(
        (p for p in client.list_profiles()
         if (p.get("name") or "").lower() == agent.lower()),
        None,
    )
    if profile is None:
        return {
            "error": f"no profile named {agent!r} pushed to OmniLink",
            "hint": f"start the specialist's runner first (e.g. {agent.lower().replace(' ', '_')}_agent.py)",
        }
    settings = profile.get("settings") or {}
    slim_settings = _slim_settings_for_subchat(settings)

    cap = max_turns if max_turns and max_turns > 0 else SUB_AGENT_MAX_TURNS

    print(f"  [CAPTAIN] delegating to {agent!r}: {task!r:.120}")
    print(f"  [CAPTAIN] system_instruction tool details: "
          f"full={len(json.dumps(settings.get('availableToolDetails') or []))} bytes, "
          f"slim={len(json.dumps(slim_settings.get('availableToolDetails') or []))} bytes")
    messages = [{"role": "user", "content": task}]
    turns = 0
    tool_calls_total = 0
    last_text = ""

    while turns < cap:
        turns += 1
        # Retry chat() on transient network errors. The server may have
        # finished the LLM work by the time the client times out, in which
        # case a re-issue with the same messages comes back fast.
        response = None
        last_exc: Optional[Exception] = None
        for attempt in range(3):
            try:
                response = client.chat(
                    messages=messages,
                    agent_name=agent,
                    engine=DEFAULT_ENGINE,
                    system_instruction=slim_settings,
                )
                break
            except Exception as exc:
                last_exc = exc
                # Before retrying, check if the bridge has flipped
                # mission_complete in the meantime — if so, we're done.
                mid_status = http_get(info["runner_status_url"], timeout=1.0)
                if isinstance(mid_status, dict) and mid_status.get("mission_complete"):
                    print(f"  [CAPTAIN] {agent!r} bridge flipped mission_complete during chat retry; success")
                    return {
                        "success": True,
                        "agent": agent,
                        "turns": turns,
                        "tool_calls": tool_calls_total,
                        "final_text": last_text,
                        "mission_complete": True,
                        "sub_status": mid_status,
                        "note": f"chat raised on attempt {attempt + 1} ({exc.__class__.__name__}); recovered from /status",
                    }
                if attempt < 2:
                    print(f"  [CAPTAIN] chat() raised ({exc.__class__.__name__}); retry attempt {attempt + 2}/3")
                    time.sleep(2.0)
        if response is None:
            late_status = http_get(info["runner_status_url"], timeout=2.0)
            late_complete = isinstance(late_status, dict) and bool(late_status.get("mission_complete"))
            if late_complete:
                return {
                    "success": True,
                    "agent": agent,
                    "turns": turns,
                    "tool_calls": tool_calls_total,
                    "final_text": last_text,
                    "mission_complete": True,
                    "sub_status": late_status,
                    "note": f"chat raised after retries ({last_exc.__class__.__name__}); recovered from /status",
                }
            return {
                "success": False,
                "agent": agent,
                "turns": turns,
                "tool_calls": tool_calls_total,
                "mission_complete": False,
                "sub_status": late_status if isinstance(late_status, dict) else None,
                "error": f"chat() raised after 3 attempts: {last_exc.__class__.__name__}: {last_exc}",
            }
        text = (response.get("text") or "").strip()
        tool_calls = response.get("toolCalls") or []
        if text:
            last_text = text

        # Pull a fresh status snapshot every turn so the captain (and
        # the sub-agent's next prompt) can react to new mission_complete /
        # fault state without an extra round-trip.
        sub_status = http_get(info["runner_status_url"], timeout=1.0)
        if isinstance(sub_status, dict) and sub_status.get("mission_complete"):
            print(f"  [CAPTAIN] {agent!r} reports mission_complete after {turns} turns")
            return {
                "success": True,
                "agent": agent,
                "turns": turns,
                "tool_calls": tool_calls_total,
                "final_text": last_text,
                "mission_complete": True,
                "sub_status": sub_status,
            }

        if not tool_calls:
            # No tool calls AND no mission_complete: nudge once before
            # giving up. The sub-agent may have just narrated a plan.
            messages.append({"role": "assistant", "content": text or "(narration)"})
            messages.append({
                "role": "user",
                "content": (
                    "Do not just narrate. Call your next tool now. If the "
                    "task is complete, call complete_mission. If it can't "
                    "be completed, explain why and stop."
                ),
            })
            continue

        # Dispatch each tool call locally.
        result_lines = []
        image_parts = []
        for tc in tool_calls:
            name = tc.get("name", "")
            args = tc.get("arguments") or {}
            payload = dict(args)
            payload["tool"] = name
            tool_call_result = http_post(info["tool_callback_url"], payload, timeout=45.0)
            actual_result = tool_call_result.get("result", tool_call_result)
            tool_calls_total += 1
            print(f"      [{agent!r}] {name}({_summarise_result(args, 80)}) -> {_summarise_result(actual_result, 100)}")
            if name == "read_camera" and isinstance(actual_result, dict) and "image_base64" in actual_result:
                meta = {k: v for k, v in actual_result.items() if k != "image_base64"}
                result_lines.append(
                    f"`read_camera` returned (image attached as inline part):\n"
                    f"```json\n{json.dumps(meta, default=str)[:400]}\n```"
                )
                image_parts.append(actual_result["image_base64"])
            else:
                # Use the dedicated summariser so big tool results (notably
                # try_get_known_map's 5 KB adjacency) don't bloat the next
                # chat() call past the server-side 120 s timeout.
                summarized = _summarise_for_subagent(name, actual_result)
                # Hard cap at 1500 chars regardless — sub-agent should not
                # carry kilobytes of tool output across turns.
                if len(summarized) > 1500:
                    summarized = summarized[:1500] + "…"
                result_lines.append(f"`{name}` returned:\n```json\n{summarized}\n```")

        # Append assistant turn + user feedback for next iteration.
        messages.append({"role": "assistant", "content": text or "(working)"})
        feedback_text = "\n\n".join(result_lines) + (
            "\n\nNow call the next tool. Don't stop until you've called "
            "complete_mission or the task is genuinely impossible."
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

    sub_status = http_get(info["runner_status_url"], timeout=1.0)
    return {
        "success": False,
        "agent": agent,
        "turns": turns,
        "tool_calls": tool_calls_total,
        "final_text": last_text,
        "mission_complete": isinstance(sub_status, dict) and bool(sub_status.get("mission_complete")),
        "sub_status": sub_status,
        "reason": f"hit max_turns={cap} without sub-agent claiming completion",
    }


# ──────────────────────────────────────────────────────────────────────
# SPECS
# ──────────────────────────────────────────────────────────────────────

SPECS = [
    ToolSpec(
        name="list_agents",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "List the captain's registered specialist agents and their "
            "reachability. Returns each specialist's name, one-line "
            "description, whether the runner is reachable, and a fresh "
            "/status snapshot when it is. Call this at session start to "
            "confirm the roster, and any time you suspect a specialist "
            "may have crashed."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_list_agents,
        tags=["roster"],
    ),
    ToolSpec(
        name="query_agent_status",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Get a specialist's live /status snapshot — strategy, current "
            "pose, mission_complete, last fault, last action. Use to "
            "verify a specialist is healthy before delegating, and to "
            "audit progress mid-mission without scanning their full "
            "activity feed."
        ),
        parameters={
            "type": "object",
            "properties": {
                "agent": {"type": "string", "description": "Specialist name (e.g. 'Husky Maze' or 'Axis')."},
            },
            "required": ["agent"],
        },
        impl=_impl_query_agent_status,
        tags=["roster", "telemetry"],
    ),
    ToolSpec(
        name="delegate_to_agent",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Hand a self-contained sub-mission to a named specialist. "
            "Runs a sub-chat-loop locally against that specialist's "
            "OmniLink profile, dispatches every tool call the specialist "
            "emits against the specialist's runner /tool endpoint, and "
            "returns when the specialist claims completion (mission_complete=true) "
            "or hits the per-delegation max_turns. Returns "
            "{success, agent, turns, tool_calls, final_text, "
            "mission_complete, sub_status}. The sub-agent has access to "
            "its own full tool surface — you don't need to anticipate "
            "what it'll call. Pass `task` as a self-contained instruction "
            "(spell out cells, colours, world names — the specialist "
            "doesn't share your operator-context). One delegation = one "
            "leg. Sequence delegations for multi-leg missions."
        ),
        parameters={
            "type": "object",
            "properties": {
                "agent": {
                    "type": "string",
                    "description": "Specialist name from list_agents (e.g. 'Husky Maze' or 'Axis').",
                },
                "task": {
                    "type": "string",
                    "description": "Self-contained instruction for the specialist. Should describe what success looks like in terms the specialist can verify (e.g. cell coords, joint targets, mission_complete rationale).",
                },
                "max_turns": {
                    "type": "integer",
                    "description": "Cap for this specific delegation. Default from CAPTAIN_SUB_MAX_TURNS env var (currently 30).",
                },
            },
            "required": ["agent", "task"],
        },
        impl=_impl_delegate_to_agent,
        tags=["delegation", "orchestration"],
    ),
]
