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
"""Native Codex app-server arm for the robot-operations benchmark.

Codex owns reasoning, history and tool iteration. This adapter only transports
operator messages and the same gated bridge tools exposed to the full tier.
No benchmark scripts, calibration actions, grading data or filesystem tools
are exposed to the participant. Codex authentication uses its normal login.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import threading
import time

from omnisim.control_bench.engine import InfrastructureError
from .competitors import CompetitorAgent, MOTION, _pose


class AppServer:
    """Small JSON-RPC transport; server tool requests run off the reader."""

    def __init__(self, cwd, on_request=None, on_event=None):
        binary = shutil.which("codex")
        if not binary:
            raise InfrastructureError("Codex CLI is not installed")
        args = [binary, "app-server", "--stdio"]
        # Disable unrelated tools and capability discovery. Keep the native
        # Codex agent loop, context management and tool-result feedback.
        for feature in ("shell_tool", "unified_exec", "plugins", "apps",
                        "browser_use", "computer_use", "image_generation",
                        "view_image", "multi_agent", "sleep_tool", "hooks"):
            args += ["--disable", feature]
        args += ["-c", "mcp_servers.node_repl.enabled=false",
                 "-c", "project_doc_max_bytes=0", "-c", 'web_search="disabled"']
        self.on_request, self.on_event = on_request, on_event
        self.pending, self.lock, self.next_id = {}, threading.Lock(), 0
        self.closed = False
        self.stderr = []
        self.proc = subprocess.Popen(args, cwd=cwd, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, encoding="utf-8", bufsize=1,
                                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._read_stderr, daemon=True).start()
        try:
            self.version = self.request("initialize", {
                "clientInfo": {"name": "omnisim_codex_benchmark", "version": "0.1.0"},
                "capabilities": {"experimentalApi": True}}, timeout=45)
            self.send({"method": "initialized"})
        except BaseException:
            self.close()
            raise

    def send(self, message):
        with self.lock:
            if self.closed:
                raise InfrastructureError("Codex app-server is closed")
            self.proc.stdin.write(json.dumps(message, ensure_ascii=False) + "\n")
            self.proc.stdin.flush()

    def request(self, method, params=None, timeout=45):
        with self.lock:
            self.next_id += 1
            ident = self.next_id
            waiter = self.pending[ident] = queue.Queue()
        self.send({"id": ident, "method": method, "params": params or {}})
        try:
            result = waiter.get(timeout=timeout)
        except queue.Empty as exc:
            raise InfrastructureError(f"Codex RPC timeout: {method}") from exc
        finally:
            self.pending.pop(ident, None)
        if "error" in result:
            raise InfrastructureError(f"Codex RPC {method}: {result['error']}")
        return result.get("result", {})

    def _read(self):
        try:
            for line in self.proc.stdout:
                try:
                    message = json.loads(line)
                except ValueError:
                    continue
                if "method" not in message:
                    waiter = self.pending.get(message.get("id"))
                    if waiter:
                        waiter.put(message)
                elif "id" in message:
                    threading.Thread(target=self._respond, args=(message,), daemon=True).start()
                elif self.on_event:
                    self.on_event(message)
        finally:
            for waiter in list(self.pending.values()):
                waiter.put({"error": "Codex app-server exited"})
            if self.on_event:
                self.on_event({"method": "process/exited", "params": {}})

    def _respond(self, message):
        try:
            result = self.on_request(message) if self.on_request else {
                "contentItems": [{"type": "inputText", "text": "Tool unavailable"}],
                "success": False}
            self.send({"id": message["id"], "result": result})
        except Exception as exc:
            if not self.closed:
                self.send({"id": message["id"], "error": {
                    "code": -32603, "message": f"{type(exc).__name__}: {exc}"}})

    def _read_stderr(self):
        for line in self.proc.stderr:
            # Local diagnostics only. Never copy auth/config files to evidence.
            self.stderr.append(line.rstrip())
            self.stderr[:] = self.stderr[-30:]

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=5)


class CodexAgent:
    name = "codex_full"
    env = {}
    _tool_surface = CompetitorAgent._tool_surface
    _main_task = CompetitorAgent._main_task
    _interrupts = CompetitorAgent._interrupts

    def __init__(self, session, clock, cfg):
        self.session, self.clock, self.tier = session, clock, "full"
        self.records = cfg.setdefault("records", [])
        self.tools = self._tool_surface()
        self.allowed = {t["name"] for t in self.tools}
        self.inbox, self.events = queue.Queue(), queue.Queue()
        self.busy, self.cancel = threading.Event(), threading.Event()
        self.turn_id, self.current_text = None, ""
        self.stopping = False
        self.model = cfg.get("codex_model", "gpt-6.1-sol")
        self.effort = cfg.get("codex_reasoning", "high")
        self.max_turns = int(cfg.get("max_requests", 1000))
        # This empty workspace is outside the repository; environment access is
        # disabled too. Only dynamic bridge tools are available to Codex.
        workspace = Path(cfg["codex_workspace"]).resolve() / session.directory.name
        workspace.mkdir(parents=True, exist_ok=False)
        self.server = AppServer(workspace, self._tool_request, self._event)
        try:
            started = self.server.request("thread/start", {
                "model": self.model, "allowProviderModelFallback": False,
                "cwd": str(workspace), "approvalPolicy": "never", "sandbox": "read-only",
                "environments": [], "selectedCapabilityRoots": [],
                "config": {"model_reasoning_effort": self.effort},
                "developerInstructions": (
                    "You are the control agent for a mobile robot. Handle operator messages "
                    "using only the supplied robot tools. Distances are metres, angles radians. "
                    "Never invent a distance or angle the operator did not give; ask instead. "
                    "Report only measured results. Respond to the operator in plain text. "
                    "Tools execute in the live world; preserve standing instructions across turns.\n"
                    "ROBOT BRIEF â€” verbatim instructions also provided to OmniLink:\n" + self._main_task()),
                "dynamicTools": [{"type": "function", "name": t["name"],
                                  "description": t["description"], "inputSchema": t["parameters"]}
                                 for t in self.tools]})
            self.thread_id = started["thread"]["id"]
            self.metadata = {"runtime": self.server.version, "model": started.get("model", self.model),
                             "reasoning_effort": self.effort, "tools": self.tools,
                             "instruction_sources": started.get("instructionSources", []),
                             "thread_id": self.thread_id, "environment_access": False}
            if self.metadata["instruction_sources"]:
                raise InfrastructureError("Unexpected instruction files in isolated Codex thread")
            (session.directory / "codex-config.json").write_text(
                json.dumps(self.metadata, indent=2), encoding="utf-8")
        except BaseException:
            self.server.close()
            raise
        self.log = (session.directory / "codex-events.jsonl").open("w", encoding="utf-8")
        self.log_lock = threading.Lock()
        self.worker = threading.Thread(target=self._loop, daemon=True)
        self.worker.start()

    def _record(self, event):
        if not hasattr(self, "log"):
            return
        with self.log_lock:
            if self.log.closed:
                return
            self.log.write(json.dumps({"t": self.clock(), **event}, ensure_ascii=False) + "\n")
            self.log.flush()

    def _event(self, message):
        self._record(message)
        self.events.put(message)

    def _tool_request(self, message):
        p = message.get("params", {})
        if message["method"] != "item/tool/call":
            raise InfrastructureError("Unexpected Codex tool/approval request")
        name, args = p.get("tool"), p.get("arguments", {})
        if isinstance(args, str):
            args = json.loads(args)
        if name not in self.allowed or self.cancel.is_set() or p.get("turnId") != self.turn_id:
            out = {"error": "Tool unavailable or turn interrupted"}
        elif name == "stop_robot":
            # Same emergency-stop escape hatch as every full-tier adapter.
            out = self.session.post("/stop_robot", {}, timeout=15)
        else:
            body = {**args, "tool": name, "utterance": self.current_text, "surface": "mobile"}
            if name in MOTION:
                body["wait"] = True
            out = self.session.post("/tool", body, timeout=120)
        self._record({"kind": "bridge_tool", "turn_id": p.get("turnId"),
                      "tool": name, "arguments": args, "result": out})
        return {"contentItems": [{"type": "inputText", "text": json.dumps(out)}],
                "success": not (isinstance(out, dict) and bool(out.get("error")))}

    def say(self, step, text, ctx):
        from .agents import Reply
        reply = Reply(step["id"], text, self.clock())
        if self.busy.is_set() and self._interrupts(text):
            self.cancel.set()
            self.session.post("/stop_robot", {}, timeout=15)
            if self.turn_id:
                self._interrupt_turn(self.turn_id)
        self.inbox.put((reply, text))
        return reply

    def _interrupt_turn(self, turn_id):
        try:
            self.server.request("turn/interrupt", {"threadId": self.thread_id,
                                                   "turnId": turn_id})
        except InfrastructureError as exc:
            # Completion/start can race an operator interruption. The native
            # server rejects an ID that ceased to be active; cancellation still
            # blocks bridge calls, and the worker consumes the completion event.
            if "expected active turn id" not in str(exc):
                raise
            self._record({"kind": "interrupt_race", "turn_id": turn_id,
                          "error": str(exc)})

    def _loop(self):
        while True:
            item = self.inbox.get()
            if item is None:
                return
            reply, text = item
            self.cancel.clear()
            self.busy.set()
            self.current_text = text
            try:
                reply.text = self._handle(text)
            except InfrastructureError as exc:
                reply.error = f"transport:{exc}"
            except Exception as exc:
                reply.error = f"{type(exc).__name__}:{exc}"
            finally:
                self.turn_id = None
                self.busy.clear()
                reply.t_done = self.clock()
                reply.done.set()

    def _handle(self, text):
        if len(self.records) >= self.max_turns:
            raise InfrastructureError("Codex operator-turn cap reached")
        rec = {"engine": "codex", "model": self.model, "t": time.time(), "reasoning": self.effort}
        rec["model_requests"] = 0
        self.records.append(rec)
        t0 = time.monotonic()
        started = self.server.request("turn/start", {"threadId": self.thread_id,
            "input": [{"type": "text", "text": json.dumps({"operator": text,
                       "measured_state": _pose(self.session.state())})}]})
        self.turn_id = started["turn"]["id"]
        rec["turn_id"] = self.turn_id
        if self.cancel.is_set():
            self._interrupt_turn(self.turn_id)
        messages = []
        end = time.monotonic() + 180
        while time.monotonic() < end and not self.stopping:
            try:
                event = self.events.get(timeout=.2)
            except queue.Empty:
                continue
            method, p = event["method"], event.get("params", {})
            if method == "process/exited":
                raise InfrastructureError("Codex app-server exited during a turn")
            if p.get("turnId") != self.turn_id and method != "turn/completed":
                continue
            if method == "thread/tokenUsage/updated":
                rec["model_requests"] += 1
                rec["usage"] = p.get("tokenUsage", {})
            elif method == "item/completed" and p.get("item", {}).get("type") == "agentMessage":
                messages.append(p["item"])
            elif method == "turn/completed" and p.get("turn", {}).get("id") == self.turn_id:
                turn = p["turn"]
                rec["status"] = turn.get("status")
                rec["elapsed_s"] = time.monotonic() - t0
                if turn.get("status") == "failed":
                    raise InfrastructureError(f"Codex turn failed: {turn.get('error')}")
                final = [m.get("text", "") for m in messages if m.get("phase") == "final_answer"]
                answer = "\n".join(final) if final else (messages[-1].get("text", "") if messages else "")
                return answer or ("Interrupted by a newer message." if self.cancel.is_set() else "")
        if self.turn_id and not self.stopping:
            self._interrupt_turn(self.turn_id)
        raise InfrastructureError("Codex turn timed out")

    def close(self):
        self.stopping = True
        self.cancel.set()
        self.inbox.put(None)
        self.server.close()
        self.worker.join(timeout=5)
        with self.log_lock:
            self.log.close()
