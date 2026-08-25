# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Drivers: turn a prompt into an ``Episode`` (a structured record of a run).

Three drivers, one interface:

``OmniLinkDriver``   OmniLink is the model + profile registry; the tool loop
                     runs HERE and dispatches every returned tool call to the
                     agent's local callback server. That indirection is not a
                     choice — OmniLink's hosted API cannot reach 127.0.0.1,
                     which is where every robot tool lives.
``LocalRouterDriver``the no-LLM control: the prompt goes straight to a
                     bridge's ``/prompt``, which is a regex intent router.
                     It produces no tool calls and no delegations, and that is
                     exactly what the control is for — it shows what the
                     harness scores with no model in the loop.
``StubDriver``       scripted episodes for the offline harness test. Never
                     touches the network.

WHAT AN EPISODE RECORDS, AND WHY
--------------------------------
* ``tool_calls`` — the model's own tool-call trace, with arguments and
  results. The TOOL-SELECTION lane grades on this, because outcome alone
  cannot tell ``stop_husky`` from ``set_husky_velocity(0,0)``.
* ``delegations`` — taken from the chat response's ``delegations`` array,
  which the platform fills in server-side. Delegation is NOT dispatched by
  this loop: ``api/chat.ts`` runs it and strips ``delegate_to_agent`` out of
  the tool calls it hands back. So this is the authoritative record, and it
  carries ``ok`` / ``errorCode`` per delegation.
* ``final_text`` — the last assistant message only. The honesty classifier
  must never see the transcript (tool errors would satisfy any failure
  keyword for free).
"""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

MAX_RESULT_CHARS = 1800


class CredentialMissing(Exception):
    """402 BYOK_REQUIRED — this engine has no provider credential on this
    account. Not a model result and not a harness bug; it is a SKIP."""

    def __init__(self, engine: str, detail: str = "") -> None:
        super().__init__(f"{engine}: BYOK_REQUIRED {detail}".strip())
        self.engine = engine
        self.detail = detail


class EngineSubstituted(Exception):
    """OmniLink served the request from a different engine than requested.

    Fatal on purpose: a substituted engine's latency, quality and cost would
    be attributed to the engine we asked for.
    """


class DriverError(Exception):
    """The harness or the transport failed. Not a model result."""


@dataclass
class ToolCall:
    name: str
    args: Dict[str, Any] = field(default_factory=dict)
    result: Any = None
    ok: bool = True
    turn: int = 0

    def as_dict(self) -> Dict[str, Any]:
        return {"name": self.name, "args": self.args, "ok": self.ok,
                "turn": self.turn,
                "result": _clip(self.result, 600)}


@dataclass
class Delegation:
    agent: str
    task: str = ""
    result: str = ""
    ok: bool = False
    error_code: Optional[str] = None
    engine: Optional[str] = None
    duration_ms: Optional[int] = None

    def as_dict(self) -> Dict[str, Any]:
        return {"agent": self.agent, "task": self.task[:200],
                "ok": self.ok, "error_code": self.error_code,
                "engine": self.engine, "duration_ms": self.duration_ms,
                "result": (self.result or "")[:800]}


@dataclass
class Episode:
    prompt: str
    engine: str
    model: Optional[str] = None
    final_text: str = ""
    transcript: str = ""
    tool_calls: List[ToolCall] = field(default_factory=list)
    delegations: List[Delegation] = field(default_factory=list)
    turns: int = 0
    wall_s: float = 0.0
    stop_reason: str = ""
    error: Optional[str] = None

    # -- convenience for graders ---------------------------------------

    def tool_names(self) -> List[str]:
        return [t.name for t in self.tool_calls]

    def called(self, *names: str) -> bool:
        wanted = set(names)
        return any(t.name in wanted for t in self.tool_calls)

    def count(self, *names: str) -> int:
        wanted = set(names)
        return sum(1 for t in self.tool_calls if t.name in wanted)

    def calls_of(self, *names: str) -> List[ToolCall]:
        wanted = set(names)
        return [t for t in self.tool_calls if t.name in wanted]

    def delegated_agents(self) -> List[str]:
        return [d.agent for d in self.delegations]

    def as_dict(self) -> Dict[str, Any]:
        return {
            "tool_calls": [t.as_dict() for t in self.tool_calls],
            "delegations": [d.as_dict() for d in self.delegations],
            "turns": self.turns,
            "stop_reason": self.stop_reason,
            "final_text": self.final_text[:2000],
            "error": self.error,
        }


def _clip(value: Any, n: int) -> Any:
    try:
        s = json.dumps(value, default=str)
    except Exception:
        s = str(value)
    return s if len(s) <= n else s[: n - 3] + "..."


# Tool names that MOVE a robot. Used by graders ("did it move anything?") and
# by the safety lane ("did it salami-slice an unsafe order into legal hops?").
MOTION_TOOLS = frozenset({
    "drive_husky", "turn_husky", "drive_to_xy", "drive_radial",
    "set_husky_velocity", "move_swarm_to", "drive_to_waypoint",
    "reset_husky_to_home", "run_routine", "execute_parallel",
    "turn_then_drive", "find_and_drive", "visit_and_return",
    "drive_with_fallback", "repeat_turn_then_drive",
})
READ_TOOLS = frozenset({
    "get_swarm_state", "get_husky_status", "list_huskies", "find_husky",
    "count_huskies", "wait_for_husky_idle", "get_swarm_metrics",
    "get_activity_log", "list_waypoints", "recall_waypoint",
})
STOP_TOOLS = frozenset({"stop_husky", "halt_all"})


# ── OmniLink ─────────────────────────────────────────────────────────


class OmniLinkDriver:
    """Local tool loop against a live coordinator, model served by OmniLink."""

    def __init__(self, *, agent_name: str, engine: str,
                 tool_url: str, omni_key: str,
                 model: str = "", max_turns: int = 12,
                 chat_timeout_s: float = 180.0,
                 tool_timeout_s: float = 180.0,
                 skip_memory: bool = True,
                 client: Any = None) -> None:
        self.agent_name = agent_name
        self.engine = engine
        self.model = model or None
        self.tool_url = tool_url
        self.omni_key = omni_key
        self.max_turns = max_turns
        self.chat_timeout_s = chat_timeout_s
        self.tool_timeout_s = tool_timeout_s
        # Every SuiteTask is an independent episode. Its in-episode history is
        # carried explicitly in `messages`; agent-scoped platform memory would
        # leak a previous fault or command into the next task. The README has
        # always specified "no memory across sessions", so isolation is the
        # default rather than a benchmark-specific prompt modification.
        self.skip_memory = bool(skip_memory)
        self._client = client
        self._settings: Optional[Dict[str, Any]] = None

    # -- platform plumbing ---------------------------------------------

    def client(self) -> Any:
        if self._client is None:
            try:
                from omnilink.client import OmniLinkClient
            except ImportError as exc:  # pragma: no cover - env dependent
                raise DriverError(
                    "omnilink-lib is not installed (pip install omnilink)") from exc
            self._client = OmniLinkClient(omni_key=self.omni_key,
                                          timeout=self.chat_timeout_s)
        return self._client

    def settings(self) -> Dict[str, Any]:
        """The agent's registered profile settings, pushed by the agent itself
        on boot. We read it rather than composing one so the benchmark scores
        the SHIPPED agent, not a benchmark-special variant."""
        if self._settings is None:
            profiles = [p for p in self.client().list_profiles()
                        if (p.get("name") or "") == self.agent_name]
            if not profiles:
                raise DriverError(
                    f"profile {self.agent_name!r} is not registered on OmniLink. "
                    f"Start the agent first — it pushes its profile on boot.")
            self._settings = profiles[0].get("settings") or {}
        return self._settings

    def dispatch(self, name: str, args: Dict[str, Any]) -> Any:
        body = dict(args or {})
        body["tool"] = name
        req = urllib.request.Request(
            self.tool_url, data=json.dumps(body, default=str).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.tool_timeout_s) as r:
                return json.loads(r.read().decode("utf-8") or "{}")
        except urllib.error.HTTPError as exc:
            return {"error": f"HTTP {exc.code}",
                    "detail": exc.read().decode("utf-8", "replace")[:400]}
        except Exception as exc:  # noqa: BLE001 — the model must see this
            return {"error": f"{type(exc).__name__}: {exc}"}

    # -- the loop -------------------------------------------------------

    def run(self, prompt: str, *, timeout_s: float = 600.0) -> Episode:
        ep = Episode(prompt=prompt, engine=self.engine, model=self.model)
        settings = self.settings()
        messages: List[Dict[str, Any]] = [{"role": "user", "content": prompt}]
        transcript: List[str] = []
        t0 = time.time()

        for turn in range(1, self.max_turns + 1):
            if time.time() - t0 > timeout_s:
                ep.stop_reason = "harness timeout"
                break
            kwargs: Dict[str, Any] = dict(
                agent_name=self.agent_name, engine=self.engine,
                messages=messages, system_instruction=settings,
                no_fallback=True,   # never let another engine answer for this one
                skipMemory=self.skip_memory,
            )
            if self.model:
                kwargs["model"] = self.model
            try:
                resp = self.client().chat(**kwargs)
            except Exception as exc:  # noqa: BLE001
                if _is_byok(exc):
                    raise CredentialMissing(self.engine, str(exc)[:200]) from exc
                ep.error = f"{type(exc).__name__}: {exc}"
                ep.stop_reason = "chat error"
                break

            served = resp.get("engine")
            if resp.get("fallbackFrom") or (served and served != self.engine):
                raise EngineSubstituted(
                    f"asked for {self.engine}, served by {served} "
                    f"(attempts={resp.get('attempts')})")

            text = (resp.get("text") or "").strip()
            if text:
                transcript.append(f"[{turn}] {text}")

            for d in (resp.get("delegations") or []):
                ep.delegations.append(Delegation(
                    agent=str(d.get("agent") or "?"),
                    task=str(d.get("task") or ""),
                    result=str(d.get("result") or ""),
                    ok=bool(d.get("ok")),
                    error_code=d.get("errorCode"),
                    engine=d.get("engine"),
                    duration_ms=d.get("durationMs"),
                ))

            tool_calls = resp.get("toolCalls") or []
            ep.turns = turn
            if not tool_calls:
                ep.final_text = text
                ep.stop_reason = "answered"
                break

            messages.append({"role": "assistant", "content": text or "(working)"})
            blocks: List[str] = []
            for tc in tool_calls:
                name = tc.get("name", "")
                args = tc.get("arguments") or {}
                result = self.dispatch(name, args)
                inner = result.get("result") if isinstance(result, dict) else result
                ok = not (isinstance(inner, dict) and "error" in inner) and \
                     not (isinstance(result, dict) and "error" in result)
                ep.tool_calls.append(ToolCall(name=name, args=args,
                                              result=inner, ok=ok, turn=turn))
                transcript.append(f"    -> {name}({_clip(args, 90)}) = "
                                  f"{_clip(inner, 140)}")
                blocks.append(f"`{name}` returned:\n```json\n"
                              f"{_clip(inner, MAX_RESULT_CHARS)}\n```")
            messages.append({"role": "user", "content": "\n\n".join(blocks) + (
                "\n\nContinue. Call the next tool if the task isn't finished, "
                "or reply with a short text summary if it is.")})
            # A turn that produced text AND tools: keep the newest text as the
            # provisional final answer so a max-turns stop still has something.
            if text:
                ep.final_text = text
        else:
            ep.stop_reason = f"max turns ({self.max_turns})"

        ep.wall_s = time.time() - t0
        ep.transcript = "\n".join(transcript)
        return ep


def _is_byok(exc: Exception) -> bool:
    code = getattr(exc, "code", None)
    status = getattr(exc, "status_code", None)
    if code == "BYOK_REQUIRED":
        return True
    if status == 402:
        return True
    return "BYOK_REQUIRED" in str(exc)


# ── The no-LLM control ───────────────────────────────────────────────


class LocalRouterDriver:
    """Send the prompt to a bridge's regex intent router. No model involved.

    This is the control arm. It cannot call tools, cannot delegate, and cannot
    reason — so most tasks fail. That is the measurement: it shows how much of
    a suite score is the harness and how much is the model.
    """

    def __init__(self, *, bridge_url: str, timeout_s: float = 60.0) -> None:
        self.bridge_url = bridge_url.rstrip("/")
        self.timeout_s = timeout_s

    def run(self, prompt: str, *, timeout_s: float = 120.0) -> Episode:
        ep = Episode(prompt=prompt, engine="local", model=None)
        t0 = time.time()
        req = urllib.request.Request(
            f"{self.bridge_url}/prompt",
            data=json.dumps({"text": prompt}).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=min(self.timeout_s,
                                                         timeout_s)) as r:
                payload = json.loads(r.read().decode("utf-8") or "{}")
        except Exception as exc:  # noqa: BLE001
            ep.error = f"{type(exc).__name__}: {exc}"
            ep.stop_reason = "router error"
            ep.wall_s = time.time() - t0
            return ep
        text = ""
        if isinstance(payload, dict):
            for key in ("reply", "text", "message", "response", "say", "result"):
                val = payload.get(key)
                if isinstance(val, str) and val.strip():
                    text = val.strip()
                    break
            if not text:
                text = json.dumps(payload, default=str)[:800]
        ep.final_text = text
        ep.transcript = text
        ep.turns = 1
        ep.stop_reason = "router answered"
        ep.wall_s = time.time() - t0
        return ep


# ── Offline stub ─────────────────────────────────────────────────────


class StubDriver:
    """Replays a canned Episode. For the offline harness test only.

    ``script`` maps prompt-substring -> Episode (or a callable returning one).
    A ``raises`` entry makes ``run`` raise, which is how the offline test
    exercises the 402 SKIPPED path without an account.
    """

    def __init__(self, script: Dict[str, Any], *, engine: str = "stub",
                 default: Optional[Callable[[str], Episode]] = None) -> None:
        self.script = script
        self.engine = engine
        self.default = default
        self.calls: List[str] = []

    def run(self, prompt: str, *, timeout_s: float = 600.0) -> Episode:
        self.calls.append(prompt)
        for key, val in self.script.items():
            if key in prompt:
                if isinstance(val, Exception):
                    raise val
                if callable(val):
                    return val(prompt)
                return val
        if self.default is not None:
            return self.default(prompt)
        return Episode(prompt=prompt, engine=self.engine,
                       final_text="", stop_reason="stub: no script entry")
