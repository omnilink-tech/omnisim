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
"""Competitor arms: an agent framework driving the same robot, same model.

Every arm here talks to the SAME bridge OmniLink does, through its gated
`POST /tool` (the operator's words ride along as `utterance`, so the shared
safety gate vets every call exactly as it vets OmniLink's). The model is the
same engine and model id OmniLink's relay uses, through the same OmniLink
chat transport. What differs is only the agent runtime:

  framework  plain  -- a hand-written plan/execute loop (the parser-off
                       ablation of the earlier campaigns);
             langgraph -- a real LangGraph StateGraph (plan -> execute);
             lobster   -- OpenClaw's real Lobster SDK pipeline.

  tier       basic -- "out of the box": the four motion tools a robot SDK
                       exposes (drive_forward, turn, stop_robot,
                       get_robot_state), and one operator message handled at a
                       time, in order. What a developer gets by following the
                       framework's own agent tutorial against a robot API.
             full  -- the SAME tool surface OmniLink's model gets (GET /tools:
                       boundaries, scheduled actions, the action journal, ...),
                       plus interruption: a new message while busy stops the
                       robot (the /stop_robot escape hatch) and abandons the
                       rest of the plan before the new message is handled.

The full tier is the fairness control: if OmniLink only beats `basic`, the
honest claim is "out of the box", and the size of the adapter that closes the
gap is reported (see adapter_loc). Nothing here reads expected actions, fault
schedules or thresholds.
"""
from __future__ import annotations

import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import secrets
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional, TypedDict

from omnisim.paths import REPO_ROOT
from omnisim.control_bench.engine import InfrastructureError

FRAMEWORKS = ("plain", "langgraph", "lobster")
# "lc" (long context, 2026-10-01): the full tier's tools and brief PLUS the
# memory practice a competent team builds for hours-long agents -- a bounded
# recent window, a rolling model-written summary of everything older, and both
# persisted to disk so they survive a restart. Without it a multi-hour win
# would be a win over a naive loop.
TIERS = ("basic", "full", "lc")
LC_KEEP = 40          # recent messages kept verbatim
LC_FOLD = 20          # fold this many more than LC_KEEP at a time
LC_SUMMARY_SYSTEM = (
    "You maintain the working memory of a robot's control agent on a long shift. "
    "Merge the existing summary with the older messages into ONE updated summary "
    "of everything still relevant: people and their roles and authority, named "
    "places and their coordinates, standing rules and keep-out zones (and who set "
    "or lifted them), jobs given and their status, scheduled or pending orders, "
    "counts the operator may ask about, and anything promised. Drop chit-chat. "
    "Plain text, at most 400 words.")
COMPETITOR_ARMS = tuple(f"{f}_{t}" for f in FRAMEWORKS for t in TIERS)

BASIC_TOOLS = [
    {"name": "drive_forward", "description": "Drive straight a signed distance in metres (negative = backwards). Blocks until done and returns the measured result.",
     "parameters": {"type": "object", "properties": {"distance": {"type": "number"}}, "required": ["distance"]}},
    {"name": "turn", "description": "Turn in place by a signed angle in radians (positive = left). Blocks until done and returns the measured result.",
     "parameters": {"type": "object", "properties": {"angle_rad": {"type": "number"}}, "required": ["angle_rad"]}},
    {"name": "stop_robot", "description": "Stop all motion now.", "parameters": {"type": "object", "properties": {}}},
    {"name": "get_robot_state", "description": "Read the measured pose (x, y, yaw in world metres/radians) and status.",
     "parameters": {"type": "object", "properties": {}}},
]
MOTION = {"drive_forward", "turn", "drive_to", "set_velocity", "reset_to_home"}
MAX_ROUNDS = 6
MAX_ACTIONS = 12

SYSTEM = """You are the control agent for a mobile robot. An operator sends you
messages; you act through tools and answer the operator. Reply with JSON only:
{"actions": [{"tool": "<name>", "args": {...}}], "say": "<what you tell the
operator>", "done": true|false}
- actions run in order; each result (measured) comes back to you. Set done
  false to see results and continue; set done true when the operator's message
  is fully handled, and put your answer to the operator in "say".
- Use only the listed tools and their exact argument names. Distances are
  metres, angles radians (positive = left). The world frame is x forward from
  the start, y left.
- Never invent a distance or angle the operator did not give; ask instead.
- Report only what the tool results measured.
Tools:
"""


HISTORY_CAP = 600


class _Transient(InfrastructureError):
    """A provider status the relay also retries (429 / 5xx)."""

    def __init__(self, message, retry_after_s=None):
        super().__init__(message)
        self.retry_after_s = retry_after_s


def _retry_after(response, data):
    """The server's retry hint in seconds: Retry-After, or the body's
    retryAfterSec / retryAfterMs (the fields the OmniLink SDK reads)."""
    for value in (response.headers.get("Retry-After"),
                  (data or {}).get("retryAfterSec") if isinstance(data, dict) else None):
        try:
            if value is not None: return float(value)
        except (TypeError, ValueError):
            pass
    ms = (data or {}).get("retryAfterMs") if isinstance(data, dict) else None
    try:
        return float(ms) / 1000.0 if ms is not None else None
    except (TypeError, ValueError):
        return None


class ModelClient:
    """The OmniLink chat transport, as OmniLink's relay calls it. Same engine,
    same model id, same agent profile -- only the runtime around it differs."""

    def __init__(self, key: str, engine: str, model: str, agent_name: str,
                 max_requests: int, log: Optional[List[dict]] = None):
        import requests
        self.key, self.engine, self.model, self.agent_name = key, engine, model, agent_name
        self.max_requests = max_requests
        self.records: List[dict] = log if log is not None else []
        self.session = requests.Session()
        self.lock = threading.Lock()

    # The SAME transient-retry policy OmniLink's relay applies to its own chat
    # calls (relay.CHAT_RETRIES = 2, RETRY_BACKOFF_S = 1.5, on 429/500/502/503/
    # 504): an arm that gave up on a first 503 while OmniLink retried would be
    # measuring the provider's weather, not the runtime.
    RETRIES = 2
    BACKOFF_S = 1.5
    HINT_CAP_S = 15.0
    # The relay's own chat timeout (relay.REQUEST_TIMEOUT, OMNILINK_TIMEOUT
    # default 120 s). This was 90 s: pilot-03's langgraph_full lost a reply to
    # a ReadTimeout the relay would have waited out.
    REQUEST_TIMEOUT_S = 120

    def complete(self, system: str, messages: List[dict]) -> str:
        for attempt in range(self.RETRIES + 1):
            try:
                return self._complete_once(system, messages)
            except _Transient as exc:
                if attempt == self.RETRIES:
                    raise InfrastructureError("model transient error, retries exhausted")
                # The relay's rule exactly (relay.retry_wait_s): a 429's hint
                # is honoured up to HINT_CAP_S, otherwise linear backoff.
                backoff = self.BACKOFF_S * (attempt + 1)
                hint = exc.retry_after_s
                time.sleep(min(max(hint, backoff), self.HINT_CAP_S) if hint and hint > 0 else backoff)
        raise InfrastructureError("unreachable")

    def _complete_once(self, system: str, messages: List[dict]) -> str:
        with self.lock:
            if len(self.records) >= self.max_requests:
                raise InfrastructureError("competitor model request cap reached")
            rec: Dict[str, Any] = {"t": time.time(), "engine": self.engine, "model": self.model}
            self.records.append(rec)
        body = {"agentName": self.agent_name, "engine": self.engine, "temperature": 0,
                "skipMemory": True, "usePromptPipeline": True,
                "systemInstructionRequest": {"mainTask": system, "availableTools": [],
                                             "availableToolDetails": [], "allowToolUse": False},
                "messages": messages}
        if self.model:
            body["model"] = self.model
        rec["request_sha256"] = hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()
        t0 = time.monotonic()
        try:
            r = self.session.post("https://www.omnilink-agents.com/api/chat",
                                  headers={"Authorization": "Bearer " + self.key}, json=body,
                                  timeout=self.REQUEST_TIMEOUT_S)
            rec["http"] = r.status_code
            data = r.json()
        except Exception as exc:
            rec["error"] = f"{type(exc).__name__}"
            raise InfrastructureError(f"model transport: {exc}") from exc
        finally:
            rec["elapsed_s"] = time.monotonic() - t0
        raw = data.get("raw") if isinstance(data, dict) else None
        rec["usage"] = (raw or {}).get("usageMetadata") or (raw or {}).get("usage") or data.get("usage")
        rec["model_returned"] = (raw or {}).get("modelVersion") or (raw or {}).get("model")
        if r.status_code != 200 or not data.get("text"):
            rec["error"] = str(data.get("error") or data.get("code") or r.status_code)[:200]
            if r.status_code in (429, 500, 502, 503, 504):
                raise _Transient(rec["error"],
                                 _retry_after(r, data) if r.status_code == 429 else None)
            raise InfrastructureError(f"model HTTP {r.status_code}: {rec['error']}")
        return str(data["text"])


def parse_plan(text: str) -> dict:
    t = text.strip()
    if t.startswith("```"):
        t = t.split("\n", 1)[-1].rsplit("```", 1)[0]
    start, end = t.find("{"), t.rfind("}")
    plan = json.loads(t[start:end + 1] if start >= 0 else t)
    if not isinstance(plan, dict) or not isinstance(plan.get("actions", []), list):
        raise ValueError("plan must be an object with an actions list")
    plan.setdefault("actions", []); plan.setdefault("say", ""); plan.setdefault("done", True)
    return plan


class State(TypedDict):
    messages: list
    round: int
    plan: dict
    done: bool
    reply: str


def _bridge_route():
    """omnisim_bridges.route from this checkout's source -- the SAME code the
    bridge runs -- for the full tier's interruption decision."""
    import sys
    src = str(Path(__file__).resolve().parents[2] / "packages" / "omnisim-bridges" / "src")
    if src not in sys.path:
        sys.path.insert(0, src)
    from omnisim_bridges import route
    return route


FULL_BRIEF = """
ROBOT BRIEF -- the same system instructions the OmniLink agent on this robot is
given, verbatim. Follow them; the JSON reply format above still applies.
---
"""


class CompetitorAgent:
    """One framework x tier. `say()` never blocks; messages go to a worker."""

    def __init__(self, session, clock, arm: str, model: ModelClient):
        self.framework, self.tier = arm.rsplit("_", 1)
        if self.framework not in FRAMEWORKS or self.tier not in TIERS:
            raise ValueError(f"unknown competitor arm {arm}")
        self.name = arm
        self.session, self.clock, self.model = session, clock, model
        self.tools = self._tool_surface()
        self.allowed = {t["name"] for t in self.tools}
        self.system = SYSTEM + json.dumps(self.tools)
        if self.tier in ("full", "lc"):
            # Same tools AND the same brief as OmniLink's model (v2).
            brief = self._main_task()
            if brief:
                self.system = SYSTEM + json.dumps(self.tools) + "\n" + FULL_BRIEF + brief
        self.history: List[dict] = []
        self.summary = ""
        self._memory_path = (Path(getattr(session, "directory", ".")) / f"{arm}_memory.json"
                             if self.tier == "lc" else None)
        self.inbox: "queue.Queue" = queue.Queue()
        self.cancel = threading.Event()
        self.busy = threading.Event()
        self.current_text = ""
        self.graph = self.worker_proc = self.server = None
        if self.framework == "langgraph":
            self._build_langgraph()
        elif self.framework == "lobster":
            self._start_lobster()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ── the tool surface ─────────────────────────────────────────────
    def _tool_surface(self) -> List[dict]:
        if self.tier == "basic":
            return list(BASIC_TOOLS)
        try:
            from omnisim.control_bench.engine import request
            listed = request(self.session.url + "/tools", timeout=10).get("tools") or []
        except InfrastructureError:
            listed = []
        if not listed:
            raise InfrastructureError("bridge published no tools for the full tier")
        return [{"name": t.get("name"), "description": t.get("description", ""),
                 "parameters": t.get("parameters", {})} for t in listed if t.get("name")]

    def _main_task(self) -> str:
        try:
            from omnisim.control_bench.engine import request
            return str(request(self.session.url + "/main_task", timeout=10).get("main_task") or "")
        except InfrastructureError:
            return ""

    def _interrupts(self, text: str) -> bool:
        """The full tier interrupts with the SAME classifier OmniLink's bridge
        uses: a real order, a correction or a stop -- not a remark or a
        question. It used to stop on every message while busy, the naive rule
        OmniLink itself dropped after shift_dev_v1 (chit-chat aborted work)."""
        try:
            route = _bridge_route()
            return bool(route.is_halt_order(text, "mobile") or route.interrupts_motion(text, "mobile"))
        except Exception:
            return True

    # ── operator messages ────────────────────────────────────────────
    def say(self, step, text, ctx):
        from .agents import Reply
        r = Reply(step["id"], text, self.clock())
        if self.tier in ("full", "lc") and self.busy.is_set() and self._interrupts(text):
            # Interruption, the full tier's piece of runtime engineering:
            # stop the robot now, abandon the rest of the current plan.
            self.cancel.set()
            try:
                self.session.post("/stop_robot", {}, timeout=15)
            except InfrastructureError:
                pass
        self.inbox.put((r, text))
        return r

    def _loop(self):
        while True:
            item = self.inbox.get()
            if item is None:
                return
            r, text = item
            self.cancel.clear()
            self.busy.set()
            self.current_text = text
            try:
                r.text = self._handle(text)
            except InfrastructureError as exc:
                r.error = f"transport:{exc}"
            except Exception as exc:  # recorded, never swallowed
                r.error = f"{type(exc).__name__}:{exc}"
            finally:
                self.busy.clear()
                r.t_done = self.clock(); r.done.set()

    # ── one message: plan -> execute, until done ─────────────────────
    def _handle(self, text: str) -> str:
        st = self.session.state()
        memory = ([{"role": "user", "content": "WORKING MEMORY (summary of the shift so far):\n" + self.summary}]
                  if self.tier == "lc" and self.summary else [])
        state: State = {"messages": [*memory, *self.history, {"role": "user", "content": json.dumps(
            {"operator": text, "measured_state": _pose(st)})}], "round": 0, "plan": {},
            "done": False, "reply": ""}
        if self.framework == "langgraph":
            state = self.graph.invoke(state, {"recursion_limit": 4 * MAX_ROUNDS})
        elif self.framework == "lobster":
            state = self._run_lobster(state)
        else:
            while not state["done"]:
                state = self.execute(self.plan(state))
        # The whole conversation, as each framework keeps it by default (a
        # LangGraph MessagesState accumulates; a plain loop appends). This was
        # the last 24 messages -- an adapter choice, not the frameworks' --
        # and on a 30-minute shift it would have erased the stations named in
        # minute one for the competitors alone. The cap only bounds a runaway.
        self.history = state["messages"][len(memory):][-HISTORY_CAP:]
        if self.tier == "lc":
            self._fold_and_persist()
        return state["reply"]

    def _fold_and_persist(self) -> None:
        """Keep LC_KEEP recent messages; fold older ones into the summary."""
        if len(self.history) > LC_KEEP + LC_FOLD:
            older, self.history = self.history[:-LC_KEEP], self.history[-LC_KEEP:]
            try:
                self.summary = self.model.complete(LC_SUMMARY_SYSTEM, [{"role": "user", "content": json.dumps(
                    {"summary_so_far": self.summary, "older_messages": older})[:60000]}]).strip()
            except InfrastructureError:
                # A failed fold keeps the messages rather than losing them.
                self.history = older + self.history
        if self._memory_path is not None:
            try:
                self._memory_path.write_text(json.dumps({"summary": self.summary, "history": self.history}),
                                             encoding="utf-8")
            except OSError:
                pass

    def on_restart(self) -> None:
        """The robot's software restarted. What only lived in this process is
        gone; the long-context tier reloads what it persisted (its checkpoint),
        the full tier -- a framework's default, in-memory -- starts empty."""
        self.history, self.summary = [], ""
        if self._memory_path is not None and self._memory_path.exists():
            try:
                saved = json.loads(self._memory_path.read_text(encoding="utf-8"))
                self.summary, self.history = saved.get("summary", ""), saved.get("history", [])
            except (OSError, ValueError):
                pass

    def plan(self, state: State) -> State:
        if self.cancel.is_set():
            return {**state, "plan": {"actions": [], "say": "Interrupted by a newer message.", "done": True}}
        text = self.model.complete(self.system, state["messages"])
        try:
            plan = parse_plan(text)
        except (ValueError, json.JSONDecodeError) as exc:
            plan = {"actions": [], "say": "", "done": False, "_protocol_error": str(exc)}
        return {**state, "round": state["round"] + 1, "plan": plan,
                "messages": [*state["messages"], {"role": "assistant", "content": text}]}

    def execute(self, state: State) -> State:
        plan, results = state["plan"], []
        for a in plan.get("actions", [])[:MAX_ACTIONS]:
            if self.cancel.is_set():
                results.append({"skipped": "interrupted by a newer operator message"}); break
            name, args = str(a.get("tool") or ""), dict(a.get("args") or {})
            if name not in self.allowed:
                results.append({"tool": name, "error": "not an available tool"}); break
            body = {"tool": name, **args, "utterance": self.current_text, "surface": "mobile"}
            if name in MOTION:
                body["wait"] = True
            try:
                out = self.session.post("/tool", body, timeout=120)
            except InfrastructureError as exc:
                out = {"error": str(exc)}
            results.append({"tool": name, "args": args, "result": out})
            res = out.get("result", out) if isinstance(out, dict) else {}
            if (isinstance(out, dict) and (out.get("status") in ("err", "refused") or isinstance(out.get("error"), str)
                                           or res.get("accepted") is False)):
                break
        done = bool(plan.get("done")) and "_protocol_error" not in plan
        done = done or state["round"] >= MAX_ROUNDS or self.cancel.is_set()
        feedback = {"tool_results": results, "measured_state": _pose(self.session.state())}
        if "_protocol_error" in plan:
            feedback["protocol_error"] = plan["_protocol_error"] + " -- answer with the JSON object only."
        return {**state, "done": done, "reply": str(plan.get("say") or ""),
                "messages": [*state["messages"], {"role": "user", "content": json.dumps(feedback)}]}

    # ── LangGraph ────────────────────────────────────────────────────
    def _build_langgraph(self):
        from langgraph.graph import StateGraph, START, END
        g = StateGraph(State)
        g.add_node("plan", self.plan)
        g.add_node("execute", self.execute)
        g.add_edge(START, "plan"); g.add_edge("plan", "execute")
        g.add_conditional_edges("execute", lambda s: "end" if s["done"] else "plan",
                                {"end": END, "plan": "plan"})
        self.graph = g.compile()

    # ── Lobster ──────────────────────────────────────────────────────
    def _start_lobster(self):
        owner = self
        self.secret = secrets.token_hex(24)
        self.callback_error = None

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_POST(self):
                if self.headers.get("Authorization") != owner.secret:
                    self.send_error(403); return
                try:
                    st = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
                    out = {"/plan": owner.plan, "/execute": owner.execute}[self.path](st); code = 200
                except Exception as exc:
                    owner.callback_error = exc
                    out, code = {"error": f"{type(exc).__name__}: {exc}"}, 500
                raw = json.dumps(out).encode()
                self.send_response(code); self.send_header("Content-Length", str(len(raw)))
                self.end_headers(); self.wfile.write(raw)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.callback_url = f"http://127.0.0.1:{self.server.server_port}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        vendor = REPO_ROOT / "tests/benchmarks/harness_comparison/vendor/lobster/dist/src/sdk/index.js"
        worker = REPO_ROOT / "omnisim/control_bench/lobster.mjs"
        self.worker_proc = subprocess.Popen(["node", str(worker), str(vendor)], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                            encoding="utf-8",
                                            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))

    def _run_lobster(self, state: State) -> State:
        self.callback_error = None
        self.worker_proc.stdin.write(json.dumps({"state": state, "url": self.callback_url,
                                                 "token": self.secret}) + "\n")
        self.worker_proc.stdin.flush()
        line = self.worker_proc.stdout.readline()
        if not line:
            raise InfrastructureError("Lobster worker exited")
        reply = json.loads(line)
        if not reply.get("ok"):
            if isinstance(self.callback_error, InfrastructureError):
                raise self.callback_error
            raise InfrastructureError(f"Lobster: {reply.get('error')}")
        return reply["state"]

    def close(self):
        self.inbox.put(None)
        if self.worker_proc:
            try:
                self.worker_proc.stdin.close(); self.worker_proc.wait(timeout=3)
            except Exception:
                self.worker_proc.terminate()
        if self.server:
            self.server.shutdown(); self.server.server_close()


def _pose(st: dict) -> dict:
    return {k: st.get(k) for k in ("x", "y", "yaw", "mode", "sim_time")}


def adapter_loc() -> Dict[str, int]:
    """Non-blank, non-comment lines: the integration effort each tier cost."""
    src = Path(__file__).read_text(encoding="utf-8").splitlines()
    code = [l for l in src if l.strip() and not l.strip().startswith("#")]
    return {"competitors_py_total": len(code)}
