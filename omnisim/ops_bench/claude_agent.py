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
"""Native Claude Code arm for the robot-operations benchmark.

The counterpart of the native Codex arm (codex_agent.py). Claude Code owns
reasoning, history, context management and tool iteration. This adapter only
transports operator messages and the same gated bridge tools exposed to the
full tier, through a stdio MCP server (claude_mcp_stdio.py). Every built-in
Claude Code tool, setting source, skill, slash command and unrelated MCP
server is disabled. Authentication uses the normal signed-in Claude login.
"""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import secrets
import shutil
import subprocess
import sys
import tempfile
import threading
import time

from omnisim.control_bench.engine import InfrastructureError
from .competitors import CompetitorAgent, MOTION, _pose

MCP_SERVER = "robot"
MCP_PREFIX = f"mcp__{MCP_SERVER}__"
# Host-session variables of whatever launched the benchmark (for example a
# Claude Code session). The participant must be an independent session.
_HOST_ENV_PREFIXES = ("CLAUDE", "AI_AGENT", "BAGGAGE")

INSTRUCTIONS = (
    "You are the control agent for a mobile robot. Handle operator messages "
    "using only the supplied robot tools. Distances are metres, angles radians. "
    "Never invent a distance or angle the operator did not give; ask instead. "
    "Report only measured results. Respond to the operator in plain text. "
    "Tools execute in the live world; preserve standing instructions across turns.\n"
    "ROBOT BRIEF — verbatim instructions also provided to OmniLink:\n")


class ClaudeCode:
    """Stream-JSON transport to one headless Claude Code process."""

    def __init__(self, binary, args, cwd, env, on_event=None):
        self.on_event = on_event
        self.pending, self.lock, self.next_id = {}, threading.Lock(), 0
        self.closed = False
        self.stderr = []
        self.proc = subprocess.Popen([binary, *args], cwd=cwd, env=env, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, encoding="utf-8", bufsize=1,
                                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._read_stderr, daemon=True).start()
        try:
            # Same handshake the Claude Agent SDK performs in streaming mode.
            self.version = self.control({"subtype": "initialize", "hooks": None}, timeout=60)
        except BaseException:
            self.close()
            raise

    def send(self, message):
        with self.lock:
            if self.closed:
                raise InfrastructureError("Claude Code is closed")
            try:
                self.proc.stdin.write(json.dumps(message, ensure_ascii=False) + "\n")
                self.proc.stdin.flush()
            except OSError as exc:
                raise InfrastructureError(f"Claude Code stdin: {exc}") from exc

    def control(self, request, timeout=30):
        with self.lock:
            self.next_id += 1
            ident = f"req_{self.next_id}_{secrets.token_hex(4)}"
            waiter = self.pending[ident] = queue.Queue()
        self.send({"type": "control_request", "request_id": ident, "request": request})
        try:
            response = waiter.get(timeout=timeout)
        except queue.Empty as exc:
            raise InfrastructureError(f"Claude control timeout: {request['subtype']}") from exc
        finally:
            self.pending.pop(ident, None)
        if response.get("subtype") != "success":
            raise InfrastructureError(f"Claude control {request['subtype']}: {response.get('error')}")
        return response.get("response") or {}

    def user(self, text):
        self.send({"type": "user", "message": {"role": "user", "content": text},
                   "parent_tool_use_id": None, "session_id": ""})

    def _read(self):
        try:
            for line in self.proc.stdout:
                try:
                    message = json.loads(line)
                except ValueError:
                    continue
                if message.get("type") == "control_response":
                    response = message.get("response", {})
                    waiter = self.pending.get(response.get("request_id"))
                    if waiter:
                        waiter.put(response)
                    continue
                if message.get("type") == "control_request":
                    # No hooks, permission prompts or SDK MCP servers are
                    # registered, so the CLI has nothing legitimate to ask.
                    self.send({"type": "control_response", "response": {
                        "subtype": "error", "request_id": message.get("request_id"),
                        "error": "Not supported by the benchmark adapter"}})
                if self.on_event:
                    self.on_event(message)
        finally:
            for waiter in list(self.pending.values()):
                waiter.put({"subtype": "error", "error": "Claude Code exited"})
            if self.on_event:
                self.on_event({"type": "process/exited"})

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
            self.proc.wait(timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=5)


def _instruction_files(workspace):
    """Instruction files Claude Code could discover from the workspace upward."""
    names = ("CLAUDE.md", "CLAUDE.local.md", "AGENTS.md", ".claude/CLAUDE.md")
    found = [str(d / n) for d in (workspace, *workspace.parents) for n in names if (d / n).exists()]
    user = Path.home() / ".claude" / "CLAUDE.md"
    return found + ([str(user)] if user.exists() else [])


class _ToolEndpoint(ThreadingHTTPServer):
    daemon_threads = True


class ClaudeAgent:
    name = "claude_full"
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
        self.turn, self.turns_started, self.current_text = None, 0, ""
        self.stopping = False
        self.metadata = None
        self.seen_requests, self.usage_by_request = set(), {}
        self.model = cfg.get("claude_model", "claude-opus-5-5")
        self.effort = cfg.get("claude_effort", "high")
        self.max_turns = int(cfg.get("max_requests", 1000))
        binary = cfg.get("claude_binary") or shutil.which("claude")
        if not binary or not Path(binary).exists():
            raise InfrastructureError("Claude Code CLI is not installed")
        # This empty workspace is outside the repository; every built-in tool
        # is disabled too. Only the bridge tools are available to Claude.
        workspace = Path(cfg["claude_workspace"]).resolve() / session.directory.name
        workspace.mkdir(parents=True, exist_ok=False)
        self.binary, self.workspace = binary, workspace
        if _instruction_files(workspace):
            raise InfrastructureError("Instruction files are discoverable from the Claude workspace")
        self.private = Path(tempfile.mkdtemp(prefix="ops-bench-claude-"))
        self.instructions = INSTRUCTIONS + self._main_task()
        (self.private / "tools.json").write_text(json.dumps(self.tools), encoding="utf-8")
        (self.private / "instructions.txt").write_text(self.instructions, encoding="utf-8")
        token = secrets.token_hex(16)
        agent = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                if body.get("token") != token:
                    self.send_response(403)
                    self.end_headers()
                    return
                result = agent._tool_call(body.get("name"), body.get("arguments") or {})
                data = json.dumps(result).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *_):
                pass

        self.endpoint = _ToolEndpoint(("127.0.0.1", 0), Handler)
        threading.Thread(target=self.endpoint.serve_forever, daemon=True).start()
        mcp = {"mcpServers": {MCP_SERVER: {
            "type": "stdio", "command": sys.executable,
            "args": ["-I", str(Path(__file__).with_name("claude_mcp_stdio.py"))],
            "env": {"OPS_BENCH_TOOLS_FILE": str(self.private / "tools.json"),
                    "OPS_BENCH_TOOL_ENDPOINT": f"http://127.0.0.1:{self.endpoint.server_address[1]}/call",
                    "OPS_BENCH_TOOL_TOKEN": token}}}}
        (self.private / "mcp.json").write_text(json.dumps(mcp), encoding="utf-8")
        env = {k: v for k, v in os.environ.items() if not k.upper().startswith(_HOST_ENV_PREFIXES)}
        env.update(MCP_TOOL_TIMEOUT="150000", MCP_TIMEOUT="60000", DISABLE_AUTOUPDATER="1")
        self.args = ["-p", "--input-format", "stream-json", "--output-format", "stream-json",
                     "--verbose", "--model", self.model, "--effort", self.effort,
                     "--tools", "", "--strict-mcp-config", "--mcp-config", str(self.private / "mcp.json"),
                     "--allowedTools", f"mcp__{MCP_SERVER}", "--setting-sources", "",
                     "--disable-slash-commands", "--no-session-persistence",
                     "--append-system-prompt-file", str(self.private / "instructions.txt")]
        self.log = (session.directory / "claude-events.jsonl").open("w", encoding="utf-8")
        self.log_lock = threading.Lock()
        try:
            self.server = ClaudeCode(binary, self.args, str(workspace), env, self._event)
        except BaseException:
            self._shutdown_endpoint()
            self.log.close()
            raise
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

    def _check_init(self, init):
        """The participant must see the robot tools and nothing else."""
        expected = sorted(MCP_PREFIX + n for n in self.allowed)
        servers = {s.get("name"): s.get("status") for s in init.get("mcp_servers", [])}
        memory = Path((init.get("memory_paths") or {}).get("auto") or "")
        # The initialize reply also names the signed-in account; keep only what
        # describes the session, never account details, in the evidence.
        handshake = {k: self.server.version.get(k) for k in (
            "commands", "output_style", "current_permission_mode", "session_state")}
        self.metadata = {"runtime": {"claude_code_version": init.get("claude_code_version"),
                                     "binary": Path(self.binary).name, "initialize": handshake},
                         "model": init.get("model"), "effort": self.effort, "tools": self.tools,
                         "native_tools": init.get("tools"), "mcp_servers": servers,
                         "skills": init.get("skills"), "slash_commands": init.get("slash_commands"),
                         "plugins": init.get("plugins"), "api_key_source": init.get("apiKeySource"),
                         "instruction_sources": _instruction_files(self.workspace),
                         "memory_files": sorted(
                             p.name for p in memory.glob("*")) if str(memory) != "." and memory.is_dir() else [],
                         "appended_system_prompt": self.instructions, "cli_args": self.args,
                         "session_id": init.get("session_id"), "environment_access": False}
        (self.session.directory / "claude-config.json").write_text(
            json.dumps(self.metadata, indent=2), encoding="utf-8")
        if init.get("model") != self.model:
            raise InfrastructureError(f"Claude Code resolved model {init.get('model')!r}, not {self.model!r}")
        if sorted(init.get("tools") or []) != expected or servers != {MCP_SERVER: "connected"}:
            raise InfrastructureError("Claude session tools differ from the bridge tool surface")
        if (self.metadata["instruction_sources"] or self.metadata["memory_files"]
                or init.get("skills") or init.get("slash_commands")):
            raise InfrastructureError("Unexpected memory, skills or commands in isolated Claude session")

    def _tool_call(self, name, args):
        turn = self.turn
        if name not in self.allowed or self.cancel.is_set() or turn is None:
            out = {"error": "Tool unavailable or turn interrupted"}
        else:
            try:
                if name == "stop_robot":
                    # Same emergency-stop escape hatch as every full-tier adapter.
                    out = self.session.post("/stop_robot", {}, timeout=15)
                else:
                    body = {**args, "tool": name, "utterance": self.current_text, "surface": "mobile"}
                    if name in MOTION:
                        body["wait"] = True
                    out = self.session.post("/tool", body, timeout=120)
            except Exception as exc:
                out = {"error": f"{type(exc).__name__}: {exc}"}
        self._record({"kind": "bridge_tool", "turn": turn, "tool": name, "arguments": args, "result": out})
        return {"text": json.dumps(out), "success": not (isinstance(out, dict) and bool(out.get("error")))}

    def say(self, step, text, ctx):
        from .agents import Reply
        reply = Reply(step["id"], text, self.clock())
        if self.busy.is_set() and self._interrupts(text):
            self.cancel.set()
            self.session.post("/stop_robot", {}, timeout=15)
            if self.turn is not None:
                self._interrupt_turn(self.turn)
        self.inbox.put((reply, text))
        return reply

    def _interrupt_turn(self, turn):
        self._record({"kind": "interrupt_sent", "turn": turn})
        try:
            self.server.control({"subtype": "interrupt"}, timeout=15)
        except InfrastructureError as exc:
            # A turn can finish while an operator interruption is in flight.
            # Cancellation still blocks bridge calls; only a dead process is fatal.
            if self.server.closed or self.server.proc.poll() is not None:
                raise
            self._record({"kind": "interrupt_race", "turn": turn, "error": str(exc)})

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
                self.turn = None
                self.busy.clear()
                reply.t_done = self.clock()
                reply.done.set()

    def _usage_total(self):
        total = {"input_tokens": 0, "cache_creation_input_tokens": 0,
                 "cache_read_input_tokens": 0, "output_tokens": 0}
        for usage in self.usage_by_request.values():
            for key in total:
                total[key] += int(usage.get(key) or 0)
        return total

    def _handle(self, text):
        if len(self.records) >= self.max_turns:
            raise InfrastructureError("Claude operator-turn cap reached")
        rec = {"engine": "claude-code", "model": self.model, "t": time.time(), "effort": self.effort}
        rec["model_requests"] = 0
        self.records.append(rec)
        t0 = time.monotonic()
        while not self.events.empty():
            stale = self.events.get_nowait()
            if stale.get("type") == "process/exited":
                raise InfrastructureError("Claude Code exited between turns")
        self.turns_started += 1
        self.turn = rec["turn"] = self.turns_started
        self._record({"kind": "operator_turn", "turn": self.turn, "text": text})
        self.server.user(json.dumps({"operator": text, "measured_state": _pose(self.session.state())}))
        if self.cancel.is_set():
            self._interrupt_turn(self.turn)
        texts = []
        deadline, timed_out = time.monotonic() + 180, False
        while not self.stopping:
            if time.monotonic() > deadline and not timed_out:
                timed_out, deadline = True, time.monotonic() + 30
                self._interrupt_turn(self.turn)
            elif time.monotonic() > deadline:
                break
            try:
                event = self.events.get(timeout=.2)
            except queue.Empty:
                continue
            kind = event.get("type")
            if kind == "process/exited":
                raise InfrastructureError("Claude Code exited during a turn")
            if kind == "system" and event.get("subtype") == "init" and self.metadata is None:
                self._check_init(event)
            elif kind == "assistant":
                message = event.get("message", {})
                ident = message.get("id")
                if ident and ident not in self.seen_requests:
                    self.seen_requests.add(ident)
                    rec["model_requests"] += 1
                if ident and message.get("usage"):
                    self.usage_by_request[ident] = message["usage"]
                texts += [c.get("text", "") for c in message.get("content", []) if c.get("type") == "text"]
            elif kind == "rate_limit_event":
                rec["rate_limit"] = event.get("rate_limit_info")
            elif kind == "result":
                rec["status"] = event.get("subtype")
                rec["elapsed_s"] = time.monotonic() - t0
                rec["num_turns"] = event.get("num_turns")
                rec["usage_total"] = self._usage_total()
                rec["cli_reported_cost_usd"] = event.get("total_cost_usd")
                if timed_out:
                    raise InfrastructureError("Claude turn timed out")
                if event.get("is_error") and not self.cancel.is_set():
                    raise InfrastructureError(f"Claude turn failed: {event.get('subtype')}: "
                                              f"{str(event.get('result') or event.get('errors'))[:300]}")
                answer = event.get("result") if event.get("subtype") == "success" else ""
                answer = answer or (texts[-1] if texts else "")
                return answer or ("Interrupted by a newer message." if self.cancel.is_set() else "")
        raise InfrastructureError("Claude turn timed out" if not self.stopping else "Claude arm closed")

    def _shutdown_endpoint(self):
        self.endpoint.shutdown()
        self.endpoint.server_close()
        shutil.rmtree(self.private, ignore_errors=True)

    def close(self):
        self.stopping = True
        self.cancel.set()
        self.inbox.put(None)
        self.server.close()
        self.worker.join(timeout=5)
        self._shutdown_endpoint()
        with self.log_lock:
            self.log.close()
