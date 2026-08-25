# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Mechanically grade the production long-horizon PRO-agent campaign."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional


ROOT = Path(__file__).resolve().parents[3]
RESULTS = Path(__file__).resolve().parent / "results"
CAPTAIN = os.environ.get("CAPTAIN_URL", "http://127.0.0.1:51518").rstrip("/")
HUSKY = os.environ.get("HUSKY_AGENT_URL", "http://127.0.0.1:51517").rstrip("/")
BRIDGE = os.environ.get("HUSKY_BRIDGE_URL", "http://127.0.0.1:6070").rstrip("/")
AGENT_NAME = "Mission Captain"
ENGINE = os.environ.get("CAPTAIN_DEFAULT_ENGINE", "g1-engine")
PROMPT = (
    "Complete the current OmniSim world mission end to end. Use the specialist "
    "roster and a supervised mission plan. Delegate one self-contained mission "
    "to the appropriate robot specialist: it must read the authoritative world "
    "brief, plan and execute the entire route under wheel control, recover from "
    "partial or faulted motion, verify every required objective from live state, "
    "return to the required final/start location, and call its verified "
    "complete_mission. Close your mission only from the completed plan ledger."
)
REQUIRED_CORNERS = {(0, 10), (10, 10), (10, 0), (0, 0)}


def _get(url: str, timeout: float = 4.0) -> Dict[str, Any]:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except Exception as exc:
        return {"error": f"{exc.__class__.__name__}: {exc}", "url": url}


def _post_tool(name: str, args: Dict[str, Any], timeout: float = 1900.0) -> Dict[str, Any]:
    payload = dict(args)
    payload["tool"] = name
    request = urllib.request.Request(
        f"{CAPTAIN}/tool",
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            wrapper = json.loads(response.read().decode("utf-8"))
        return wrapper.get("result", wrapper)
    except Exception as exc:
        return {"error": f"captain tool dispatch failed: {exc.__class__.__name__}: {exc}"}


def _compact_tool_result(name: str, result: Any) -> str:
    if name == "execute_mission_plan" and isinstance(result, dict):
        ledger = result.get("ledger") or {}
        compact = {
            "plan_id": result.get("plan_id"),
            "objective": result.get("objective"),
            "completed": result.get("completed"),
            "verified": result.get("verified"),
            "completed_steps": ledger.get("completed_steps"),
            "failed_step": ledger.get("failed_step"),
        }
        return json.dumps(compact, default=str)
    return json.dumps(result, default=str)[:1600]


def run_chat(max_turns: int, timeout: int) -> Dict[str, Any]:
    lib_src = ROOT.parent / "omnilink" / "omnilink-lib" / "src"
    if lib_src.exists() and str(lib_src) not in sys.path:
        sys.path.insert(0, str(lib_src))
    from omnilink.client import OmniLinkClient

    key = os.environ.get("OMNI_KEY", "").strip()
    if not key:
        return {"error": "OMNI_KEY is not set"}
    client = OmniLinkClient(omni_key=key, timeout=timeout)
    profile = next(
        (
            profile
            for profile in client.list_profiles()
            if str(profile.get("name") or "").lower() == AGENT_NAME.lower()
        ),
        None,
    )
    if profile is None:
        return {"error": "Mission Captain profile is not available"}
    settings = profile.get("settings") or {}
    messages: List[Dict[str, Any]] = [{"role": "user", "content": PROMPT}]
    transcript: List[Dict[str, Any]] = []
    baseline_claims = int(
        _get(f"{CAPTAIN}/status").get("captain_complete_calls_this_session") or 0
    )

    for turn in range(1, max_turns + 1):
        started = time.monotonic()
        try:
            response = client.chat(
                messages=messages,
                agent_name=AGENT_NAME,
                engine=ENGINE,
                system_instruction=settings,
            )
        except Exception as exc:
            return {
                "error": f"chat failed: {exc.__class__.__name__}: {exc}",
                "transcript": transcript,
            }
        text = str(response.get("text") or "").strip()
        calls = response.get("toolCalls") or []
        turn_record: Dict[str, Any] = {
            "turn": turn,
            "elapsed_s": round(time.monotonic() - started, 3),
            "text": text,
            "tool_calls": [],
        }
        transcript.append(turn_record)

        for call in calls:
            name = str(call.get("name") or "")
            args = call.get("arguments") or {}
            result = _post_tool(name, args)
            turn_record["tool_calls"].append(
                {"name": name, "arguments": args, "result": result}
            )

        status = _get(f"{CAPTAIN}/status")
        if int(status.get("captain_complete_calls_this_session") or 0) > baseline_claims:
            return {"completed": True, "turns": turn, "transcript": transcript}

        messages.append({"role": "assistant", "content": text or "(working)"})
        if calls:
            feedback = "\n\n".join(
                f"`{item['name']}` returned:\n```json\n"
                f"{_compact_tool_result(item['name'], item['result'])}\n```"
                for item in turn_record["tool_calls"]
            )
        else:
            feedback = (
                "Do not just narrate. Call list_agents or execute_mission_plan "
                "now. Claim completion only with the verified plan_id."
            )
        messages.append(
            {
                "role": "user",
                "content": feedback
                + "\n\nContinue the plan. Use complete_mission only after the ledger is verified.",
            }
        )
    return {
        "completed": False,
        "error": f"hit max_turns={max_turns}",
        "transcript": transcript,
    }


def _activity_entries(base: str) -> List[Dict[str, Any]]:
    payload = _get(f"{base}/activity")
    entries = payload.get("entries")
    return entries if isinstance(entries, list) else []


def grade(chat: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    bridge_state = _get(f"{BRIDGE}/state")
    mission = _get(f"{BRIDGE}/mission")
    captain_status = _get(f"{CAPTAIN}/status")
    husky_status = _get(f"{HUSKY}/status")
    captain_activity = _activity_entries(CAPTAIN)

    tools = [
        str((entry.get("data") or {}).get("tool") or "")
        for entry in captain_activity
    ]
    plan_entries = [
        entry
        for entry in captain_activity
        if (entry.get("data") or {}).get("tool") == "execute_mission_plan"
    ]
    claim_entries = [
        entry
        for entry in captain_activity
        if (entry.get("data") or {}).get("tool") == "complete_mission"
    ]
    latest_plan = (
        (plan_entries[-1].get("data") or {}).get("result") or {}
        if plan_entries
        else {}
    )
    latest_claim = (
        (claim_entries[-1].get("data") or {}).get("result") or {}
        if claim_entries
        else {}
    )
    ledger = latest_plan.get("ledger") or {}
    ledger_steps = ledger.get("steps") or []
    delegation_result = (
        (ledger_steps[0].get("result") or {}) if ledger_steps else {}
    )
    plan_args = (
        (plan_entries[-1].get("data") or {}).get("args") or {}
        if plan_entries
        else {}
    )
    plan_text = json.dumps(plan_args, default=str).lower()

    visited = {
        (int(cell[0]), int(cell[1]))
        for cell in bridge_state.get("visited_cells") or []
        if isinstance(cell, list) and len(cell) >= 2
    }
    current = bridge_state.get("current_cell") or {}
    current_cell = (current.get("col"), current.get("row"))
    mission_log = mission.get("log") or []
    verified_claim = bool(mission_log and mission_log[-1].get("verified") is True)
    captain_claim = latest_claim.get("claim") or {}
    plan_id_bound = bool(
        latest_plan.get("plan_id")
        and captain_claim.get("plan_id") == latest_plan.get("plan_id")
    )

    checks: List[Dict[str, Any]] = []

    def add(name: str, points: int, passed: bool, evidence: Any) -> None:
        checks.append(
            {
                "name": name,
                "points": points,
                "passed": bool(passed),
                "awarded": points if passed else 0,
                "evidence": evidence,
            }
        )

    endpoints_ok = all(
        "error" not in payload
        for payload in (bridge_state, captain_status, husky_status)
    )
    add("all live endpoints reachable", 5, endpoints_ok, {
        "bridge": "error" not in bridge_state,
        "captain": "error" not in captain_status,
        "husky": "error" not in husky_status,
    })
    add("captain inspected specialist roster", 5, "list_agents" in tools, tools)
    add("captain used supervised plan", 10, bool(plan_entries), plan_args)
    plan_complete_words = (
        "return" in plan_text
        and ("brief" in plan_text or "mission" in plan_text)
        and ("recover" in plan_text or "fault" in plan_text or "replan" in plan_text)
    )
    add("specialist mission is self-contained", 5, plan_complete_words, plan_args)

    for corner in sorted(REQUIRED_CORNERS):
        add(f"visited corner {corner}", 5, corner in visited, sorted(visited))
    add("returned to start cell", 10, current_cell == (0, 10), current_cell)
    add(
        "bridge mission_complete",
        5,
        bridge_state.get("mission_complete") is True,
        bridge_state.get("mission_complete"),
    )
    add("bridge accepted verified claim", 10, verified_claim, mission_log[-1:] or [])

    add(
        "supervised ledger completed and verified",
        10,
        latest_plan.get("completed") is True
        and latest_plan.get("verified") is True,
        ledger,
    )
    settled = (
        not bridge_state.get("fault")
        and bridge_state.get("mode") in ("idle", "stopped")
    )
    add("robot settled without active fault", 5, settled, {
        "mode": bridge_state.get("mode"),
        "fault": bridge_state.get("fault"),
    })
    tool_calls = int(delegation_result.get("tool_calls") or 0)
    add(
        "specialist performed meaningful work",
        5,
        tool_calls >= 4 and len(visited) >= 20,
        {"tool_calls": tool_calls, "visited_cells": len(visited)},
    )
    add(
        "captain recorded completion",
        5,
        latest_claim.get("captain_mission_complete") is True,
        latest_claim,
    )
    add("captain claim bound to plan_id", 5, plan_id_bound, {
        "plan_id": latest_plan.get("plan_id"),
        "claim_plan_id": captain_claim.get("plan_id"),
    })

    score = sum(check["awarded"] for check in checks)
    hard_gates = {
        "all_corners": REQUIRED_CORNERS <= visited,
        "returned_to_start": current_cell == (0, 10),
        "bridge_verified": verified_claim and bridge_state.get("mission_complete") is True,
        "plan_verified": latest_plan.get("verified") is True,
        "claim_bound": plan_id_bound,
    }
    return {
        "benchmark": "LH-1-hierarchical-corners",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "score": score,
        "max_score": 100,
        "pass": score >= 80 and all(hard_gates.values()),
        "hard_gates": hard_gates,
        "checks": checks,
        "metrics": {
            "visited_cells": len(visited),
            "specialist_turns": delegation_result.get("turns"),
            "specialist_tool_calls": tool_calls,
            "captain_activity_entries": len(captain_activity),
            "sim_time": bridge_state.get("sim_time"),
        },
        "environment": {
            # A result must name the machine that produced it -- an unattributed
            # number cannot be reproduced or compared. But the raw hostname is the
            # developer's laptop name and these files ship publicly, so record a
            # stable hash: same box -> same id, across runs, without the name.
            "host": _machine_id(),
            "platform": platform.platform(),
            "python": sys.version,
            "git_commit": _git_commit(),
        },
        "chat": chat,
    }


def _machine_id() -> str:
    """Stable per-machine id: sha256 of the hostname, first 12 hex chars."""
    return "machine-" + hashlib.sha256(
        platform.node().encode("utf-8")).hexdigest()[:12]


def _git_commit() -> Optional[str]:
    import subprocess

    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-chat", action="store_true")
    parser.add_argument("--grade-only", action="store_true")
    parser.add_argument("--max-turns", type=int, default=12)
    parser.add_argument("--chat-timeout", type=int, default=240)
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.run_chat == args.grade_only:
        parser.error("choose exactly one of --run-chat or --grade-only")

    chat = run_chat(args.max_turns, args.chat_timeout) if args.run_chat else None
    result = grade(chat)
    RESULTS.mkdir(parents=True, exist_ok=True)
    output = (
        Path(args.output)
        if args.output
        else RESULTS / f"{datetime.now().strftime('%Y%m%d-%H%M%S')}.json"
    )
    output.write_text(json.dumps(result, indent=2, default=str), encoding="utf-8")
    print(json.dumps({
        "score": result["score"],
        "max_score": result["max_score"],
        "pass": result["pass"],
        "hard_gates": result["hard_gates"],
        "metrics": result["metrics"],
        "result": str(output),
    }, indent=2))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
