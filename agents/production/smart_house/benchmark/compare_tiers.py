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

"""compare_tiers — normal vs persistent smart-house agent benchmark.

Two arms, same scenarios, same tools (the 19 Haven-shaped specs from
``../tools``), same model:

* **NORMAL** (free-tier behaviour): the agent gets a turn ONLY at scripted
  user-present moments.
* **PERSISTENT** (paid behaviour): the same, PLUS a wake turn every 60
  HOUSE-minutes while the occupant is unavailable (away or asleep — each
  scenario declares its unattended windows explicitly).

Between agent turns the benchmark advances house time via
``POST /scenario/advance``. Every measured output comes from
``POST /scenario/metrics`` ONLY — never from what the agent says about
itself.

Modes:

* ``--mock --fake-llm``   — offline CI mode: in-process ``mock_hub.py`` +
  a scripted deterministic policy. No network, no engine, no key.
* ``--mock``              — live LLM against the mock hub (needs OMNI_KEY).
* default (live)          — live LLM against a real hub (``--hub-url``),
  i.e. the Lane-A bridge under the simulator. Refuses without a key.

Examples::

    python compare_tiers.py --mock --fake-llm
    python compare_tiers.py --mock --fake-llm --scenarios s2_oven_left_on
    python compare_tiers.py --hub-url http://127.0.0.1:8766 --arms persistent

Outputs land in ``results/<UTC timestamp>/``: ``results.json``,
``report.md``, per-run transcripts (``transcript_<arm>_<scenario>.jsonl``)
and, in live mode, a redacted ``api_chat.log.jsonl``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_THIS = Path(__file__).resolve().parent
_PKG = _THIS.parent
sys.path.insert(0, str(_PKG))  # agents/production/smart_house -> `tools` package

from tools import load_all  # noqa: E402
from tools._base import CONFIRM_REQUIRED  # noqa: E402

DEFAULT_BASE_URL = "https://www.omnilink-agents.com"
DEFAULT_ENGINE = "g1-engine"
# Key-file fallbacks, tried in order, and only when OMNI_KEY is unset and
# OMNI_KEY_FILE names nothing. Only a portable, machine-independent location is
# listed: a path on one developer's drive is dead weight in a public repository
# and discloses their local layout for no benefit. Point OMNI_KEY_FILE at a key
# store elsewhere if yours does not live here.
DEFAULT_KEY_FILES = (
    Path.home() / ".omnilink" / "omni_key.txt",
)
AGENT_NAME = "SmartHouse-Bench"
WAKE_EVERY_MIN = 60            # Builder cadence — honest
MAX_TOOL_ROUNDS = 5
SYSTEM_PROMPT_PATH = _PKG / "prompts" / "system.md"

REGISTRY = load_all()
TOOL_DETAILS = [s.to_query_tool(mode="full") for s in REGISTRY.values()]


# ---------------------------------------------------------------------------
# Hub client (the benchmark's own — tools use tools._base.hub_call)
# ---------------------------------------------------------------------------

class HubClient:
    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")

    def post(self, endpoint: str, payload: Optional[Dict[str, Any]] = None) -> Any:
        url = f"{self.base_url}/{endpoint.lstrip('/')}"
        body = json.dumps(payload or {}).encode("utf-8")
        req = urllib.request.Request(url, data=body, method="POST")
        req.add_header("Content-Type", "application/json")
        with urllib.request.urlopen(req, timeout=300) as r:
            raw = r.read().decode("utf-8", errors="replace")
        return json.loads(raw) if raw else {}


# ---------------------------------------------------------------------------
# Tool execution (shared by live + fake engines)
# ---------------------------------------------------------------------------

def execute_tool(name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    """Dispatch one tool call through the registry (client-side gates apply).

    CONFIRM_REQUIRED impls refuse without an ``authorization`` arg exactly
    as they do in the interactive agent — the benchmark adds nothing.
    """
    spec = REGISTRY.get(name)
    if spec is None:
        return {
            "error": f"unknown tool: {name}",
            "known_tools": sorted(REGISTRY.keys()),
        }
    try:
        return spec.impl(**(args or {}))
    except TypeError as exc:
        return {"error": f"bad arguments for {name}: {exc}"}


# ---------------------------------------------------------------------------
# Live /api/chat client
# ---------------------------------------------------------------------------

class ChatClient:
    """Minimal /api/chat tool-loop client per the frozen contract."""

    def __init__(self, base_url: str, key: str, engine: str,
                 model: Optional[str], main_task: str,
                 log_path: Optional[Path]) -> None:
        self.base_url = base_url.rstrip("/")
        self.key = key
        self.engine = engine
        self.model = model
        self.main_task = main_task
        self.log_path = log_path

    def _log(self, record: Dict[str, Any]) -> None:
        if self.log_path is None:
            return
        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _post_chat(self, messages: List[Dict[str, Any]]) -> Dict[str, Any]:
        body: Dict[str, Any] = {
            "agentName": AGENT_NAME,
            "messages": messages,
            "engine": self.engine,
            "temperature": 0.2,
            "skipMemory": True,
            "noFallback": True,
            "systemInstructionRequest": {
                "mainTask": self.main_task,
                "availableToolDetails": TOOL_DETAILS,
                "allowToolUse": True,
            },
        }
        if self.model:
            body["model"] = self.model
        data = json.dumps(body).encode("utf-8")
        url = f"{self.base_url}/api/chat"
        for attempt in range(6):
            req = urllib.request.Request(url, data=data, method="POST")
            req.add_header("Content-Type", "application/json")
            req.add_header("Authorization", f"Bearer {self.key}")
            try:
                with urllib.request.urlopen(req, timeout=180) as r:
                    resp = json.loads(r.read().decode("utf-8", errors="replace"))
                self._log({"request": _redact(body), "response": resp,
                           "utc": _utc_now()})
                return resp
            except urllib.error.HTTPError as e:
                detail = e.read().decode("utf-8", errors="replace")
                self._log({"request": _redact(body),
                           "http_error": e.code, "detail": detail[:2000],
                           "utc": _utc_now()})
                if e.code == 429:
                    retry_s = 5.0
                    try:
                        d = json.loads(detail)
                        # PLAN_RATE_LIMITED nests the hint inside "error";
                        # the older engine window puts retry_after top-level.
                        err = d.get("error") if isinstance(d.get("error"), dict) else {}
                        retry_s = float(err.get("retryAfterSec")
                                        or d.get("retryAfterSec")
                                        or d.get("retry_after") or 5)
                    except Exception:
                        pass
                    print(f"    [429] rate limited; sleeping {retry_s:.0f}s "
                          f"(attempt {attempt + 1})")
                    time.sleep(retry_s)
                    continue
                raise RuntimeError(f"/api/chat HTTP {e.code}: {detail[:500]}") from e
        raise RuntimeError("/api/chat: rate-limit retries exhausted")

    def run_turn(self, messages: List[Dict[str, Any]],
                 prompt: str) -> Tuple[str, List[Dict[str, Any]]]:
        """One user turn incl. the tool loop. Mutates ``messages`` in place.

        Returns (assistant_text, executed_tool_calls).
        """
        messages.append({"role": "user", "content": prompt})
        executed: List[Dict[str, Any]] = []
        text = ""
        for _round in range(MAX_TOOL_ROUNDS + 1):
            resp = self._post_chat(messages)
            tool_calls = resp.get("toolCalls") or []
            text = (resp.get("message") or resp.get("content")
                    or resp.get("text") or resp.get("response") or "")
            if isinstance(text, dict):
                text = text.get("content", "")
            if not tool_calls or _round == MAX_TOOL_ROUNDS:
                break
            messages.append({"role": "assistant", "content": "",
                             "tool_calls": tool_calls})
            for tc in tool_calls:
                name = tc.get("name", "")
                args = tc.get("arguments") or {}
                if isinstance(args, str):
                    try:
                        args = json.loads(args) if args.strip() else {}
                    except json.JSONDecodeError:
                        args = {}
                result = execute_tool(name, args)
                executed.append({"name": name, "args": args, "result": result})
                messages.append({
                    "role": "tool",
                    "tool_call_id": tc.get("id", ""),
                    "name": name,
                    "content": json.dumps(result, ensure_ascii=False),
                })
        if text:
            messages.append({"role": "assistant", "content": text})
        return str(text), executed


def _redact(body: Dict[str, Any]) -> Dict[str, Any]:
    """The key travels only in the Authorization header (never logged)."""
    return body


def _utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

@dataclass
class Scenario:
    name: str
    start_time: str
    duration_min: int
    timeline: List[Dict[str, Any]]
    wake_windows: List[Tuple[int, int]] = field(default_factory=list)
    incident: Optional[Dict[str, Any]] = None
    return_t: Optional[int] = None
    notes: str = ""


SCENARIOS: Dict[str, Scenario] = {
    "s1_movie_night": Scenario(
        name="s1_movie_night",
        start_time="19:00",
        duration_min=180,
        timeline=[
            {"t": 0, "kind": "user",
             "prompt": "Movie time — set us up in the living room, please."},
            {"t": 150, "kind": "user",
             "prompt": "Movie's over, we're heading to bed. Goodnight!"},
        ],
        notes="Occupant present throughout; baseline parity — the arms "
              "should behave (and cost) the same.",
    ),
    "s2_oven_left_on": Scenario(
        name="s2_oven_left_on",
        start_time="08:00",
        duration_min=500,
        timeline=[
            {"t": 0, "kind": "event",
             "event": {"type": "device_set", "id": "appliance.oven", "state": "on"}},
            {"t": 5, "kind": "user",
             "prompt": "I'm heading to work — back around 16:05. "
                       "Put the house in away mode."},
            {"t": 6, "kind": "resident", "room": "away"},
            {"t": 485, "kind": "resident", "room": "living_room"},
            {"t": 486, "kind": "user", "prompt": "I'm home!"},
        ],
        wake_windows=[(6, 485)],
        incident={
            "t": 6,
            "match_device": {"id": "appliance.oven", "state": "off"},
            "notify_keywords": ["oven"],
            "description": "2.4 kW oven left on in an empty house",
        },
        return_t=485,
        notes="The oven was on at breakfast and never turned off. The away "
              "scene does not touch it. check_anomalies will NOT report it — "
              "the agent must infer it from device state / kitchen temp.",
    ),
    "s3_night_door": Scenario(
        name="s3_night_door",
        start_time="20:00",
        duration_min=660,
        timeline=[
            {"t": 0, "kind": "user",
             "prompt": "We're calling it a night — goodnight, SmartHouse."},
            {"t": 1, "kind": "resident", "room": "bedroom"},
            {"t": 370, "kind": "event",
             "event": {"type": "door", "id": "door.front", "state": "open"}},
            {"t": 655, "kind": "user", "prompt": "Good morning."},
        ],
        wake_windows=[(60, 655)],
        incident={
            "t": 370,
            "match_device": {"id": "door.front", "state": "closed"},
            "notify_keywords": ["door"],
            "description": "front door opens at 02:10 while armed",
        },
        notes="Occupant is home but ASLEEP 21:00-06:55, so the persistent "
              "arm's wake window covers the night (contract deviation noted "
              "in the README: 'away' generalised to 'unavailable').",
    ),
    "s4_morning_prep": Scenario(
        name="s4_morning_prep",
        start_time="21:00",
        duration_min=640,
        timeline=[
            {"t": 0, "kind": "user",
             "prompt": "Heading to the airport — I'll be back tomorrow at "
                       "07:30. Away mode please, and have the place ready "
                       "when I'm back."},
            {"t": 1, "kind": "resident", "room": "away"},
            {"t": 630, "kind": "resident", "room": "living_room"},
            {"t": 631, "kind": "user", "prompt": "I'm back!"},
        ],
        wake_windows=[(1, 630)],
        return_t=630,
        notes="No incident — the measure is comfort at the announced return "
              "(living-room temperature, coffee readiness) vs energy spent.",
    ),
}


# ---------------------------------------------------------------------------
# Turn engines
# ---------------------------------------------------------------------------

class LiveTurnEngine:
    """One continuing /api/chat conversation per (arm, scenario)."""

    def __init__(self, chat: ChatClient) -> None:
        self.chat = chat
        self.messages: List[Dict[str, Any]] = []

    def run_turn(self, prompt: str, kind: str,
                 ctx: Dict[str, Any]) -> Dict[str, Any]:
        text, executed = self.chat.run_turn(self.messages, prompt)
        return {"kind": kind, "prompt": prompt, "text": text,
                "tool_calls": executed}


class FakeTurnEngine:
    """Deterministic scripted policy — proves the harness offline.

    Persistent wake: sweep (check_anomalies + read_sensors + list_devices),
    turn the oven off if on while away, close the door if open while armed,
    prep shortly before a known return, notify on every fix. Normal user
    turns: do what was asked; on arrival/morning, sweep and fix.
    """

    def __init__(self) -> None:
        self.prepped = False

    # -- helpers over live hub state (via the same tool impls) --------

    def _sweep_and_fix(self, ctx: Dict[str, Any],
                       calls: List[Dict[str, Any]],
                       arrival: bool) -> None:
        for name in ("check_anomalies", "read_sensors", "list_devices"):
            calls.append({"name": name, "args": {},
                          "result": execute_tool(name, {})})
        devices = {d["id"]: d for d in calls[-1]["result"].get("items", [])}

        oven = devices.get("appliance.oven", {})
        away = ctx.get("occupant") == "away"
        if oven.get("state") == "on" and (away or arrival):
            calls.append({"name": "set_device",
                          "args": {"id": "appliance.oven", "state": "off"},
                          "result": execute_tool("set_device",
                                                 {"id": "appliance.oven",
                                                  "state": "off"})})
            msg = ("Safety: the oven was running with nobody home. "
                   "I turned it off.")
            calls.append({"name": "notify_occupant",
                          "args": {"message": msg, "severity": "high"},
                          "result": execute_tool("notify_occupant",
                                                 {"message": msg,
                                                  "severity": "high"})})

        door = devices.get("door.front", {})
        alarm = devices.get("security.system", {})
        if door.get("state") == "open" and alarm.get("state") == "armed":
            calls.append({"name": "set_device",
                          "args": {"id": "door.front", "state": "closed"},
                          "result": execute_tool("set_device",
                                                 {"id": "door.front",
                                                  "state": "closed"})})
            msg = ("SECURITY: the front door opened while the system was "
                   "armed. I closed it; the system remains armed.")
            calls.append({"name": "notify_occupant",
                          "args": {"message": msg, "severity": "critical"},
                          "result": execute_tool("notify_occupant",
                                                 {"message": msg,
                                                  "severity": "critical"})})

        if arrival:
            thermo = devices.get("thermostat.main", {})
            if isinstance(thermo.get("state"), dict) and \
                    thermo["state"].get("mode") == "eco":
                calls.append({"name": "adjust_thermostat",
                              "args": {"id": "thermostat.main",
                                       "target": 21, "mode": "heat"},
                              "result": execute_tool(
                                  "adjust_thermostat",
                                  {"id": "thermostat.main",
                                   "target": 21, "mode": "heat"})})
            if ctx.get("scenario") == "s4_morning_prep":
                coffee = devices.get("appliance.coffee_maker", {})
                if coffee.get("state") != "on" and not coffee.get("coffee_ready"):
                    calls.append({"name": "set_device",
                                  "args": {"id": "appliance.coffee_maker",
                                           "state": "on"},
                                  "result": execute_tool(
                                      "set_device",
                                      {"id": "appliance.coffee_maker",
                                       "state": "on"})})

    def _prep_for_return(self, calls: List[Dict[str, Any]]) -> None:
        calls.append({"name": "adjust_thermostat",
                      "args": {"id": "thermostat.main", "target": 21,
                               "mode": "heat"},
                      "result": execute_tool("adjust_thermostat",
                                             {"id": "thermostat.main",
                                              "target": 21, "mode": "heat"})})
        calls.append({"name": "set_device",
                      "args": {"id": "appliance.coffee_maker", "state": "on"},
                      "result": execute_tool("set_device",
                                             {"id": "appliance.coffee_maker",
                                              "state": "on"})})
        msg = "Warming the house and starting coffee ahead of your return."
        calls.append({"name": "notify_occupant",
                      "args": {"message": msg, "severity": "low"},
                      "result": execute_tool("notify_occupant",
                                             {"message": msg,
                                              "severity": "low"})})
        self.prepped = True

    def run_turn(self, prompt: str, kind: str,
                 ctx: Dict[str, Any]) -> Dict[str, Any]:
        calls: List[Dict[str, Any]] = []
        p = prompt.lower()
        if kind == "wake":
            self._sweep_and_fix(ctx, calls, arrival=False)
            rt, now = ctx.get("return_t"), ctx.get("t", 0)
            if (not self.prepped and rt is not None
                    and 0 < rt - now <= WAKE_EVERY_MIN):
                self._prep_for_return(calls)
            text = "Wake sweep complete." if calls else "Nothing to do."
        elif "goodnight" in p:
            calls.append({"name": "set_scene", "args": {"scene": "goodnight"},
                          "result": execute_tool("set_scene",
                                                 {"scene": "goodnight"})})
            text = "Goodnight scene applied. Sleep well."
        elif "movie" in p:
            calls.append({"name": "set_scene", "args": {"scene": "movie"},
                          "result": execute_tool("set_scene",
                                                 {"scene": "movie"})})
            text = "Movie scene applied. Enjoy."
        elif "away" in p:
            calls.append({"name": "set_scene", "args": {"scene": "away"},
                          "result": execute_tool("set_scene",
                                                 {"scene": "away"})})
            text = "Away mode set: eco heating, lights off, armed, locked."
        elif any(k in p for k in ("home", "back", "morning")):
            self._sweep_and_fix(ctx, calls, arrival=True)
            text = "Welcome back — arrival sweep done."
        else:
            text = "Acknowledged."
        return {"kind": kind, "prompt": prompt, "text": text,
                "tool_calls": calls}


# ---------------------------------------------------------------------------
# Arm driver
# ---------------------------------------------------------------------------

def _wake_times(sc: Scenario) -> List[int]:
    out: List[int] = []
    for a, b in sc.wake_windows:
        t = a + WAKE_EVERY_MIN
        while t < b:
            out.append(t)
            t += WAKE_EVERY_MIN
    return sorted(out)


# The real bridge caps /scenario/advance at 480 house-minutes per call
# (documented in its README); chunk long jumps so the NORMAL arm's
# turn-free stretches (630 min in s4) don't get rejected.
_ADVANCE_CHUNK_MIN = 480


def _advance(hub: "HubClient", house_minutes: int) -> None:
    remaining = int(house_minutes)
    while remaining > 0:
        step = min(remaining, _ADVANCE_CHUNK_MIN)
        hub.post("scenario/advance", {"house_minutes": step})
        remaining -= step


def run_arm(arm: str, sc: Scenario, hub: HubClient,
            make_engine, transcript_path: Path) -> Dict[str, Any]:
    hub.post("scenario/reset")
    start = hub.post("scenario/start", {
        "name": sc.name, "time_scale": 60, "start_time": sc.start_time,
    })
    start_iso = start.get("house_time")
    start_dt = datetime.fromisoformat(start_iso)

    agenda: List[Tuple[int, int, Dict[str, Any]]] = []
    for i, item in enumerate(sc.timeline):
        agenda.append((int(item["t"]), i, item))
    if arm == "persistent":
        for k, wt in enumerate(_wake_times(sc), start=1):
            agenda.append((wt, 10000 + k, {"kind": "wake", "wake_no": k}))
    agenda.sort(key=lambda x: (x[0], x[1]))

    engine = make_engine()
    occupant = "home"
    now = 0
    turns = 0
    tool_calls = 0
    records: List[Dict[str, Any]] = []

    def record(rec: Dict[str, Any]) -> None:
        records.append(rec)
        with open(transcript_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    for t, _order, item in agenda:
        if t > now:
            _advance(hub, t - now)
            now = t
        kind = item["kind"]
        if kind == "resident":
            res = hub.post("scenario/resident", {"room": item["room"]})
            occupant = "away" if item["room"] == "away" else "home"
            record({"t": t, "kind": "resident", "room": item["room"],
                    "result": res})
        elif kind == "event":
            res = hub.post("scenario/event", item["event"])
            record({"t": t, "kind": "event", "event": item["event"],
                    "result": res})
        elif kind in ("user", "wake"):
            status = hub.post("scenario/status")
            house_time = status.get("house_time", "?")
            if kind == "wake":
                k = item["wake_no"]
                prompt = (f"[Wake #{k} — house heartbeat] It is {house_time}. "
                          f"Occupant is {occupant}. Review the house and act "
                          f"per your mandate. Be economical.")
            else:
                prompt = item["prompt"]
            ctx = {"scenario": sc.name, "arm": arm, "t": t,
                   "occupant": occupant, "return_t": sc.return_t,
                   "house_time": house_time}
            rec = engine.run_turn(prompt, kind, ctx)
            turns += 1
            tool_calls += len(rec.get("tool_calls", []))
            rec.update({"t": t, "house_time": house_time})
            record(rec)
    if sc.duration_min > now:
        _advance(hub, sc.duration_min - now)

    metrics = hub.post("scenario/metrics")
    return analyze(arm, sc, start_dt, metrics, turns, tool_calls)


# ---------------------------------------------------------------------------
# Metrics analysis — from /scenario/metrics ONLY
# ---------------------------------------------------------------------------

def _min_from_start(start_dt: datetime, iso: str) -> float:
    return (datetime.fromisoformat(iso) - start_dt).total_seconds() / 60.0


def analyze(arm: str, sc: Scenario, start_dt: datetime,
            metrics: Dict[str, Any], turns: int,
            tool_calls: int) -> Dict[str, Any]:
    device_log = metrics.get("device_log", [])
    notifications = metrics.get("notifications", [])
    timeline = metrics.get("temp_timeline", [])

    detection: Optional[float] = None
    if sc.incident:
        inc_t = sc.incident["t"]
        md = sc.incident["match_device"]
        for entry in device_log:
            if entry.get("changed_by") != "agent":
                continue
            if entry.get("id") != md["id"] or entry.get("state") != md["state"]:
                continue
            t = _min_from_start(start_dt, entry["house_time"])
            if t >= inc_t:
                detection = round(t - inc_t, 1)
                break
        if detection is None:
            kws = [k.lower() for k in sc.incident.get("notify_keywords", [])]
            for n in notifications:
                t = _min_from_start(start_dt, n["house_time"])
                if t >= inc_t and any(k in n.get("message", "").lower()
                                      for k in kws):
                    detection = round(t - inc_t, 1)
                    break

    peaks: Dict[str, float] = {}
    for sample in timeline:
        for room, temp in sample.get("temps", {}).items():
            peaks[room] = max(peaks.get(room, -999.0), temp)

    agent_actions = [e for e in device_log if e.get("changed_by") == "agent"]

    out: Dict[str, Any] = {
        "arm": arm,
        "scenario": sc.name,
        "detection_latency_house_min": detection,
        "energy_wh": metrics.get("energy_wh_total"),
        "peak_temps": {r: round(v, 2) for r, v in peaks.items()},
        "final_temps": metrics.get("room_temps"),
        "agent_actions": agent_actions,
        "agent_action_count": len(agent_actions),
        "notifications": notifications,
        "turns_used": turns,
        "tool_calls_used": tool_calls,
    }

    if sc.return_t is not None and timeline:
        best = min(timeline,
                   key=lambda s: abs(_min_from_start(start_dt, s["house_time"])
                                     - sc.return_t))
        out["temps_at_return"] = best.get("temps")
        # coffee readiness derived from the device_log (metrics-only rule):
        # last coffee state change before return is "on", >=5 min before it.
        coffee_on_t: Optional[float] = None
        for e in device_log:
            if e.get("id") != "appliance.coffee_maker":
                continue
            t = _min_from_start(start_dt, e["house_time"])
            if t <= sc.return_t:
                coffee_on_t = t if e.get("state") == "on" else None
        out["coffee_ready_at_return"] = (
            coffee_on_t is not None and sc.return_t - coffee_on_t >= 5.0)
    return out


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def _fmt(v: Any) -> str:
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:g}"
    return str(v)


def write_report(out_dir: Path, results: Dict[str, Dict[str, Any]],
                 meta: Dict[str, Any]) -> str:
    lines: List[str] = []
    lines.append("# Smart-house tier comparison — NORMAL vs PERSISTENT")
    lines.append("")
    lines.append(f"Generated: {meta['generated_at']} · mode: "
                 f"{'mock hub' if meta['mock'] else 'live hub'} + "
                 f"{'scripted fake LLM' if meta['fake_llm'] else meta.get('model') or 'platform default model'} "
                 f"· wake cadence: every {WAKE_EVERY_MIN} house-min while unattended")
    lines.append("")
    lines.append("All numbers below are measured from `/scenario/metrics` "
                 "(simulator-side device log, energy integral, temperature "
                 "timeline) — never from agent self-reports.")
    lines.append("")
    lines.append("## Headline")
    lines.append("")
    lines.append("| scenario | metric | NORMAL | PERSISTENT |")
    lines.append("|---|---|---|---|")
    for sname, arms in results.items():
        n, p = arms.get("normal"), arms.get("persistent")
        rows: List[Tuple[str, Any, Any]] = []
        if (n or p or {}).get("detection_latency_house_min") is not None or \
                SCENARIOS[sname].incident:
            rows.append(("detection latency (house-min)",
                         (n or {}).get("detection_latency_house_min"),
                         (p or {}).get("detection_latency_house_min")))
        rows.append(("energy (Wh)", (n or {}).get("energy_wh"),
                     (p or {}).get("energy_wh")))
        if sname == "s2_oven_left_on":
            rows.append(("peak kitchen temp (C)",
                         ((n or {}).get("peak_temps") or {}).get("kitchen"),
                         ((p or {}).get("peak_temps") or {}).get("kitchen")))
        if SCENARIOS[sname].return_t is not None:
            rows.append(("living room at return (C)",
                         ((n or {}).get("temps_at_return") or {}).get("living_room"),
                         ((p or {}).get("temps_at_return") or {}).get("living_room")))
            rows.append(("coffee ready at return",
                         (n or {}).get("coffee_ready_at_return"),
                         (p or {}).get("coffee_ready_at_return")))
        rows.append(("turns used", (n or {}).get("turns_used"),
                     (p or {}).get("turns_used")))
        rows.append(("tool calls used", (n or {}).get("tool_calls_used"),
                     (p or {}).get("tool_calls_used")))
        for label, nv, pv in rows:
            lines.append(f"| {sname} | {label} | {_fmt(nv)} | {_fmt(pv)} |")
    lines.append("")
    for sname, arms in results.items():
        sc = SCENARIOS[sname]
        lines.append(f"## {sname}")
        lines.append("")
        if sc.notes:
            lines.append(f"> {sc.notes}")
            lines.append("")
        lines.append("| metric | NORMAL | PERSISTENT |")
        lines.append("|---|---|---|")
        n, p = arms.get("normal") or {}, arms.get("persistent") or {}
        keys = ["detection_latency_house_min", "energy_wh",
                "agent_action_count", "turns_used", "tool_calls_used"]
        for k in keys:
            lines.append(f"| {k} | {_fmt(n.get(k))} | {_fmt(p.get(k))} |")
        for room in ("living_room", "kitchen", "bedroom", "hallway"):
            lines.append(f"| peak temp {room} (C) "
                         f"| {_fmt((n.get('peak_temps') or {}).get(room))} "
                         f"| {_fmt((p.get('peak_temps') or {}).get(room))} |")
        lines.append(f"| notifications | {len(n.get('notifications') or [])} "
                     f"| {len(p.get('notifications') or [])} |")
        lines.append("")
        for arm_name, r in (("NORMAL", n), ("PERSISTENT", p)):
            acts = r.get("agent_actions") or []
            if acts:
                lines.append(f"**{arm_name} agent actions** (from device_log, "
                             "changed_by=agent):")
                lines.append("")
                for a in acts[:30]:
                    lines.append(f"- `{a['house_time']}` {a['id']} -> "
                                 f"`{json.dumps(a['state'])}`")
                lines.append("")
    text = "\n".join(lines) + "\n"
    (out_dir / "report.md").write_text(text, encoding="utf-8")
    return text


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

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


def main(argv: Optional[List[str]] = None) -> int:
    # Windows consoles default to cp1252; the report uses em-dashes.
    for _stream in (sys.stdout, sys.stderr):
        try:
            _stream.reconfigure(encoding="utf-8", errors="backslashreplace")
        except Exception:
            pass
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--mock", action="store_true",
                    help="run against an in-process mock_hub (no simulator)")
    ap.add_argument("--fake-llm", action="store_true",
                    help="scripted deterministic policy instead of /api/chat")
    ap.add_argument("--scenarios", default=",".join(SCENARIOS),
                    help="comma-separated scenario names")
    ap.add_argument("--arms", default="normal,persistent")
    ap.add_argument("--model", default=None)
    ap.add_argument("--engine", default=DEFAULT_ENGINE)
    ap.add_argument("--base-url", default=DEFAULT_BASE_URL)
    ap.add_argument("--hub-url", default=None,
                    help="live hub URL (default http://127.0.0.1:8766)")
    ap.add_argument("--out", default=None,
                    help="output dir (default results/<timestamp>)")
    ap.add_argument("--key-file", default=None,
                    help="read the OmniLink key from this file (overrides "
                         "OMNI_KEY_FILE and the default search)")
    args = ap.parse_args(argv)

    scenario_names = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    for s in scenario_names:
        if s not in SCENARIOS:
            print(f"unknown scenario {s!r}; known: {', '.join(SCENARIOS)}")
            return 2
    arms = [a.strip() for a in args.arms.split(",") if a.strip()]
    for a in arms:
        if a not in ("normal", "persistent"):
            print(f"unknown arm {a!r}; use normal / persistent")
            return 2

    key: Optional[str] = None
    if not args.fake_llm:
        key = resolve_key(args.key_file)
        if not key:
            print("Refusing to run a live-LLM benchmark without OMNI_KEY "
                  "(env), --key-file, OMNI_KEY_FILE, or a readable "
                  "~/.omnilink/omni_key.txt. Use --fake-llm for offline mode.")
            return 2

    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%SZ")
    out_dir = Path(args.out) if args.out else _THIS / "results" / ts
    out_dir.mkdir(parents=True, exist_ok=True)

    mock = None
    if args.mock:
        from mock_hub import MockHub  # local sibling import
        mock = MockHub(port=0).start()
        hub_url = mock.url
    else:
        hub_url = args.hub_url or "http://127.0.0.1:8766"
    # Point the tool registry (tools._base.hub_call) at the same hub.
    os.environ["SMART_HOUSE_HUB_URL"] = hub_url
    hub = HubClient(hub_url)

    main_task = SYSTEM_PROMPT_PATH.read_text(encoding="utf-8").strip()

    def make_engine():
        if args.fake_llm:
            return FakeTurnEngine()
        chat = ChatClient(args.base_url, key, args.engine, args.model,
                          main_task, out_dir / "api_chat.log.jsonl")
        return LiveTurnEngine(chat)

    results: Dict[str, Dict[str, Any]] = {}
    try:
        for sname in scenario_names:
            sc = SCENARIOS[sname]
            results[sname] = {}
            for arm in arms:
                print(f"[{sname}] arm={arm} ...")
                transcript = out_dir / f"transcript_{arm}_{sname}.jsonl"
                r = run_arm(arm, sc, hub, make_engine, transcript)
                results[sname][arm] = r
                print(f"  detection={_fmt(r['detection_latency_house_min'])} "
                      f"house-min, energy={_fmt(r['energy_wh'])} Wh, "
                      f"turns={r['turns_used']}, tools={r['tool_calls_used']}")
    finally:
        if mock is not None:
            mock.stop()

    meta = {
        "generated_at": _utc_now(),
        "mock": bool(args.mock),
        "fake_llm": bool(args.fake_llm),
        "model": args.model,
        "engine": args.engine,
        "hub_url": hub_url,
        "wake_every_house_min": WAKE_EVERY_MIN,
        "scenarios": scenario_names,
        "arms": arms,
    }
    (out_dir / "results.json").write_text(
        json.dumps({"meta": meta, "results": results}, indent=2,
                   ensure_ascii=False),
        encoding="utf-8")
    report = write_report(out_dir, results, meta)
    print()
    print(report)
    print(f"Results: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
