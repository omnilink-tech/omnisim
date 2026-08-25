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

# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Drive the HuskySwarm coordinator via OmniLink chat() with a local tool loop.

This is the **local orchestration** path: OmniLink is the LLM backend and the
profile registry, while this script runs the tool loop itself and dispatches
every returned tool call to the agent's own callback server on
``127.0.0.1:51520``. That indirection exists because OmniLink's hosted API
cannot reach a loopback address -- the same reason ``mission_captain`` runs its
own sub-loop rather than using server-side delegation.

Usage
-----
    # shell A -- the world (4 Huskies on 8865-8868)
    python -m omnisim run-world projects/samples/demos/worlds/flagship/omnilink_husky_swarm.omniworld

    # shell B -- the coordinator
    set OMNI_KEY=olink_...
    python agents/production/husky_swarm/swarm_agent.py

    # shell C -- drive it
    python agents/production/husky_swarm/scripts/chat_drive.py \\
        "Patrol all four quadrants in parallel: 2 m out, turn around, 2 m back."

Termination: the loop ends when the model answers with text and no tool calls
(HuskySwarm is a standing coordinator -- unlike husky_maze it has no
``complete_mission`` tool), or on ``--max-turns``, or when the loop detector
sees the same call repeated with no change in swarm pose.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Tuple

_REPO_ROOT = Path(__file__).resolve().parents[4]

AGENT_NAME = "HuskySwarm"
DEFAULT_TOOL_URL = "http://127.0.0.1:51520/tool"
DEFAULT_HEALTH_URL = "http://127.0.0.1:51520/health"

# Repeat the same tool call with an unchanged swarm pose this many times and
# we nudge; one more and we abort rather than burn the operator's BYOK tokens.
LOOP_NUDGE_AT = 2
LOOP_ABORT_AT = 4

# Results longer than this are truncated before going back to the model --
# get_swarm_state on 4 robots plus an activity log can run to several KB.
MAX_RESULT_CHARS = 1800

# The model narrates "they're on their way" and stops; if the swarm is still
# in motion we wait this long for it to settle and nudge for a real answer.
MAX_SETTLE_NUDGES = 2
SETTLE_TIMEOUT_S = 30.0


def _utc_now_iso() -> str:
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).isoformat()


def _post_json(url: str, body: Dict[str, Any], timeout: float) -> Dict[str, Any]:
    req = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def execute_tool(tool_url: str, name: str, args: Dict[str, Any],
                 timeout: float) -> Dict[str, Any]:
    """Dispatch one tool call to the agent's local callback server."""
    body = dict(args or {})
    body["tool"] = name
    try:
        return _post_json(tool_url, body, timeout)
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", "replace")[:400]
        return {"error": f"HTTP {exc.code}", "detail": detail}
    except Exception as exc:  # noqa: BLE001 - surface anything to the model
        return {"error": f"{type(exc).__name__}: {exc}"}


def _summarise(result: Any) -> str:
    text = json.dumps(result, default=str)
    if len(text) <= MAX_RESULT_CHARS:
        return text
    return text[:MAX_RESULT_CHARS] + f"... [truncated, {len(text)} chars total]"


def _swarm_pose_signature(tool_url: str, timeout: float) -> str:
    """Coarse pose fingerprint for all four Huskies.

    Rounded to 0.1 m so that motor jitter while a robot is settling doesn't
    read as progress -- only a real move changes the signature.
    """
    res = execute_tool(tool_url, "get_swarm_state", {}, timeout)
    payload = res.get("result") if isinstance(res, dict) else None
    if not isinstance(payload, dict):
        return "unknown"
    entries = payload.get("huskies") or payload.get("swarm") or payload
    parts: List[str] = []
    if isinstance(entries, dict):
        entries = list(entries.values())
    if isinstance(entries, list):
        for item in entries:
            if not isinstance(item, dict):
                continue
            name = item.get("husky") or item.get("id") or "?"
            try:
                parts.append(f"{name}:{float(item.get('x', 0)):.1f},{float(item.get('y', 0)):.1f}")
            except (TypeError, ValueError):
                continue
    return "|".join(sorted(parts)) or "unknown"


def _swarm_busy(tool_url: str, timeout: float) -> bool:
    """True if any Husky is still executing a motion.

    The model reliably answers "they're on their way, I'll confirm when they
    arrive" and then stops -- which would end the loop mid-manoeuvre and leave
    the operator's question unanswered. Checking real motion state lets us
    nudge it exactly once instead of trusting the narration.
    """
    res = execute_tool(tool_url, "get_swarm_state", {}, timeout)
    payload = res.get("result") if isinstance(res, dict) else None
    if not isinstance(payload, dict):
        return False
    entries: Any = payload.get("state") or payload.get("huskies") or payload
    if isinstance(entries, dict):
        entries = list(entries.values())
    if not isinstance(entries, list):
        return False
    for item in entries:
        if isinstance(item, dict) and item.get("mode") not in (None, "idle"):
            return True
    return False


def _print_usage(delta: Any, label: str, engine: str = "g1-engine",
                 client: Any = None, started_iso: str = "",
                 elapsed_s: float = 0.0) -> None:
    if delta is None:
        return
    try:
        print(f"[usage] {label}: {delta.report()}")
    except Exception:  # noqa: BLE001 - telemetry must never break a run
        pass
    # UsageMeter labels everything "tokens"; for Gemini/GPT the server meters
    # CHARACTERS, so that label overstates by ~4x. Print the money instead --
    # OmniLink's credits_24h/7d/30d are hardcoded 0, so the platform cannot
    # answer "what did this cost" for us.
    try:
        sys.path.insert(0, str(_REPO_ROOT / "agents" / "production"))
        from _lib.cost import run_report  # type: ignore
        if client is not None and started_iso:
            print(run_report(client, started_iso, elapsed_s, engine))
    except Exception:  # noqa: BLE001
        pass


def main(argv: List[str]) -> int:
    ap = argparse.ArgumentParser(description="Drive the HuskySwarm coordinator.")
    ap.add_argument("prompt", help="what the swarm should do, in plain English")
    ap.add_argument("--engine", default="g1-engine",
                    help="OmniLink engine (default g1-engine / Gemini). Engines "
                         "other than the ones your account has a BYOK key for "
                         "return 402 BYOK_REQUIRED.")
    ap.add_argument("--max-turns", type=int, default=14)
    ap.add_argument("--no-fallback", action="store_true",
                    help="Fail loudly instead of letting OmniLink silently serve the "
                         "request from a different engine. ALWAYS use this when "
                         "benchmarking -- otherwise g5's results are really g1's.")
    ap.add_argument("--model", default="",
                    help="Override the provider model (e.g. llama3.1:8b for g5).")
    ap.add_argument("--tool-url", default=os.environ.get("HUSKY_SWARM_TOOL_URL", DEFAULT_TOOL_URL))
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--tool-timeout", type=float, default=120.0)
    args = ap.parse_args(argv)

    omni_key = os.environ.get("OMNI_KEY", "").strip()
    if not omni_key:
        print("ERROR: OMNI_KEY not set. Get one at https://www.omnilink-agents.com/key\n"
              "       or run: python -m omnisim key")
        return 2

    try:
        from omnilink.client import OmniLinkClient
    except ImportError:
        print("ERROR: omnilink-lib is not installed.\n    pip install omnilink")
        return 2

    # Fail fast and clearly if the coordinator isn't up -- otherwise every
    # tool call returns a connection error and the model flails.
    try:
        with urllib.request.urlopen(DEFAULT_HEALTH_URL, timeout=10) as resp:
            health = json.loads(resp.read().decode("utf-8"))
        print(f"[swarm] coordinator up: {health.get('tool_count')} tools, "
              f"{len(health.get('bridges') or {})} bridges")
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: HuskySwarm coordinator not reachable at {DEFAULT_HEALTH_URL} "
              f"({type(exc).__name__}). Start it first:\n"
              f"    python agents/production/husky_swarm/swarm_agent.py")
        return 3

    client = OmniLinkClient(omni_key=omni_key, timeout=args.timeout)

    profiles = [p for p in client.list_profiles() if p.get("name") == AGENT_NAME]
    if not profiles:
        print(f"ERROR: profile {AGENT_NAME!r} not found on OmniLink. "
              f"Start swarm_agent.py first -- it pushes the profile on boot.")
        return 3
    settings = profiles[0].get("settings") or {}

    meter = None
    try:
        from omnilink.usage_meter import UsageMeter
        meter = UsageMeter(client)
        meter.start()
    except Exception as exc:  # noqa: BLE001
        print(f"[usage] unavailable ({type(exc).__name__}) -- continuing without telemetry")

    messages: List[Dict[str, Any]] = [{"role": "user", "content": args.prompt}]
    recent: List[str] = []
    last_pose = ""
    total_tools = 0
    settle_nudges = 0
    run_t0 = time.time()
    run_started_iso = _utc_now_iso()

    for turn in range(1, args.max_turns + 1):
        try:
            chat_kwargs = dict(
                agent_name=AGENT_NAME,
                engine=args.engine,
                messages=messages,
                system_instruction=settings,
            )
            if args.no_fallback:
                chat_kwargs["no_fallback"] = True
            if args.model:
                # `extra` on OmniLinkClient.chat is a **kwargs collector, not a
                # dict parameter: passing extra={"model": X} sets body["extra"],
                # never body["model"], so the override is silently dropped.
                # Pass it as a top-level kwarg instead.
                chat_kwargs["model"] = args.model
            resp = client.chat(**chat_kwargs)
        except Exception as exc:  # noqa: BLE001
            print(f"[chat] turn {turn} failed: {type(exc).__name__}: {exc}")
            return 4

        # OmniLink silently substitutes another engine when one is unavailable:
        # a g5 request that hits "overloaded" comes back served by g1, with the
        # SAME response shape. Benchmarking that would attribute Gemini's
        # latency, quality and cost to Ollama, so refuse to continue.
        served = resp.get("engine")
        if resp.get("fallbackFrom") or (served and served != args.engine):
            print(f"[chat] ENGINE SUBSTITUTED: asked for {args.engine}, served by "
                  f"{served} (attempts={resp.get('attempts')}). Results would be "
                  f"attributed to the wrong engine -- aborting. Re-run with "
                  f"--no-fallback to see the underlying error.")
            return 6

        text = (resp.get("text") or "").strip()
        tool_calls = resp.get("toolCalls") or []

        if text:
            print(f"\n[{turn}] {text}")
        if not tool_calls:
            # Don't take "they're on their way" for done -- if the swarm is
            # still in motion, let it settle and give the model one chance to
            # actually answer. Bounded so a stuck robot can't loop forever.
            if settle_nudges < MAX_SETTLE_NUDGES and _swarm_busy(args.tool_url, args.tool_timeout):
                settle_nudges += 1
                print(f"[swarm] still moving -- settling before accepting an answer "
                      f"(nudge {settle_nudges}/{MAX_SETTLE_NUDGES})")
                deadline = time.time() + SETTLE_TIMEOUT_S
                while time.time() < deadline and _swarm_busy(args.tool_url, args.tool_timeout):
                    time.sleep(1.0)
                messages.append({"role": "assistant", "content": text or "(working)"})
                messages.append({"role": "user", "content":
                    "The Huskies have finished moving. Call get_swarm_state now and "
                    "report each robot's final x,y as you said you would."})
                continue
            print(f"\n[swarm] done in {turn} turn(s), {total_tools} tool call(s), "
                  f"{time.time() - run_t0:.1f}s")
            break

        messages.append({"role": "assistant", "content": text or "(working)"})

        signature = "|".join(
            f"{tc.get('name', '')}({json.dumps(tc.get('arguments') or {}, sort_keys=True)})"
            for tc in tool_calls
        )
        pose = _swarm_pose_signature(args.tool_url, args.tool_timeout)
        is_repeat = bool(recent and recent[-1] == signature and pose == last_pose)
        recent = recent + [signature] if is_repeat else [signature]
        last_pose = pose
        repeats = len(recent)
        if repeats >= LOOP_ABORT_AT:
            print(f"[swarm] LOOP ABORT: {signature[:90]} issued {repeats}x with no "
                  f"change in swarm pose. Stopping to save tokens.")
            _print_usage(meter.snapshot() if meter else None, "run", args.engine,
                 client, run_started_iso, time.time() - run_t0)
            return 5

        lines = []
        for tc in tool_calls:
            name = tc.get("name", "")
            tc_args = tc.get("arguments") or {}
            result = execute_tool(args.tool_url, name, tc_args, args.tool_timeout)
            total_tools += 1
            print(f"    -> {name}({json.dumps(tc_args, default=str)[:70]}) "
                  f"= {json.dumps(result, default=str)[:110]}")
            lines.append(f"`{name}` returned:\n```json\n{_summarise(result)}\n```")

        feedback = "\n\n".join(lines) + (
            "\n\nContinue. Call the next tool if the task isn't finished, or "
            "reply with a short text summary if it is."
        )
        if repeats >= LOOP_NUDGE_AT:
            feedback += (
                f"\n\n[LOOP DETECTED] You made the same call {repeats} turns in a "
                f"row and the swarm has not moved. Repeating it will not change "
                f"the result. Call get_swarm_state to see where the Huskies "
                f"actually are, then choose a DIFFERENT action."
            )
        messages.append({"role": "user", "content": feedback})
    else:
        print(f"\n[swarm] hit --max-turns ({args.max_turns})")

    _print_usage(meter.snapshot() if meter else None, "run", args.engine,
                 client, run_started_iso, time.time() - run_t0)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
