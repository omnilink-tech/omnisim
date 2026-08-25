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

"""omnilink_relay — shared OmniLink chat-with-tools loop for OmniSim bridges.

Each bridge owns a per-robot tool surface (set_velocity, set_joint_positions,
etc.). The relay wraps those handlers so the user's natural-language prompt
flows through OmniLink:

    side menu prompt
      |
      v
    bridge.handle_wwi_message()
      |
      v
    OmniLinkRelay.dispatch(prompt)
      |
      v  POST /api/chat with availableToolDetails + conversation history
    OmniLink (g4-engine etc.)
      |
      v  returns {text, toolCalls: [{id, name, arguments}]}
    relay executes each tool call locally via bridge.handlers
      |
      v  appends tool result to history, may loop one more turn
    final agent text -> back through bridge -> side menu

When OMNI_KEY is unset the bridge skips the relay and uses its local
regex intent router. This keeps the demos working offline; OmniLink
is an upgrade path, not a hard dependency.

TLS note: AVG / corporate proxies sometimes intercept TLS on the
operator's machine. truststore.inject_into_ssl() (used here) hands
Python's ssl module the OS trust store, which sees AVG's MITM cert.
Without it, requests to https://omnilink-agents.com fail SSL
verification.
"""

from __future__ import annotations

import json
import os
import re
import sys
import threading
import time
import traceback
import uuid
from dataclasses import dataclass, field
from queue import Empty, Full, Queue
from typing import Any, Callable, Dict, List, Optional


@dataclass
class DispatchHandle:
    """Cancellation handle for one queued prompt.

    ``cancel()`` marks the prompt first, then probes the execution gate. Once it
    returns, no new tool call can begin. A call that already crossed the gate
    is reported as possibly in flight so the HTTP caller knows not to retry.
    """

    cancelled: threading.Event = field(default_factory=threading.Event)
    execution_gate: threading.RLock = field(default_factory=threading.RLock)
    #: monotonic stamp taken when the prompt was enqueued — the worker
    #: subtracts it to report queue wait separately from work time.
    enqueued_at: float = field(default_factory=time.perf_counter)

    def cancel(self) -> bool:
        """Cancel future work; return True when a tool may already be in flight."""
        self.cancelled.set()
        acquired = self.execution_gate.acquire(blocking=False)
        if acquired:
            self.execution_gate.release()
        return not acquired

    def is_cancelled(self) -> bool:
        return self.cancelled.is_set()

# AVG TLS interception fix -- inject the OS trust store before any HTTPS.
try:
    import truststore  # type: ignore[import-not-found]
    truststore.inject_into_ssl()
except Exception:
    pass

# Real OmniLink Python client. See omnilink-lib's omnilink/client.py (OmniLink repo)
# for the API reference (chat / memory / TTS / STT etc.). We use it as the
# canonical entry-point so the in-sim integration looks exactly like a
# real-world OmniLink integration -- same OmniLinkClient(omni_key=...).chat()
# call shape against the same /api/chat endpoint. The relay is the only
# OmniSim-side glue: tool-call dispatch and history management.
try:
    import omnilink as _omnilink_pkg  # type: ignore[import-not-found]
    from omnilink.client import OmniLinkClient, OmniLinkAPIError  # type: ignore[import-not-found]
    try:
        # Added in omnilink 0.6.x. An older SDK still works -- the 402 then
        # falls through to the status-code check below.
        from omnilink.client import (  # type: ignore[import-not-found]
            OmniLinkBYOKRequiredError)
    except ImportError:
        OmniLinkBYOKRequiredError = None  # type: ignore[assignment]
    from omnilink.usage_meter import UsageMeter  # type: ignore[import-not-found]
except Exception:
    _omnilink_pkg = None
    OmniLinkClient = None  # type: ignore[assignment]
    OmniLinkAPIError = Exception  # type: ignore[assignment,misc]
    # Must be defined even here: OmniLinkAPIError degrades to bare Exception, so
    # the chat handler's `isinstance(e, OmniLinkBYOKRequiredError)` is reachable
    # on ANY error once the SDK is missing entirely.
    OmniLinkBYOKRequiredError = None  # type: ignore[assignment]
    UsageMeter = None  # type: ignore[assignment]


# Minimum omnilink-lib version the relay's call shape relies on
# (engine= kwarg, system_instruction= kwarg, OmniLinkAPIError shape).
# Matches the pin in agents/requirements.txt so the floor is
# consistent across the repo. Bump this when the relay starts using a
# newer API.
MIN_OMNILINK_VERSION = "0.6.1"


def _parse_version(s: str):
    """Tuple parse for PEP-440-ish versions. Ignores pre/post tags after
    the first non-numeric component — sufficient for "is the installed
    version >= the floor" decisions on omnilink-lib's flat X.Y.Z scheme."""
    parts = []
    for token in (s or "0").replace("+", ".").split("."):
        digits = ""
        for ch in token:
            if ch.isdigit():
                digits += ch
            else:
                break
        if digits:
            parts.append(int(digits))
        else:
            break
    return tuple(parts) or (0,)


_OMNILINK_VERSION_CHECKED = False
_NEW_VERSION_CHECK_DONE = False


def check_omnilink_installation() -> str:
    """Raise if omnilink-lib is missing or below MIN_OMNILINK_VERSION.

    Idempotent — safe to call from every relay construction. Returns the
    installed version string on success.
    """
    global _OMNILINK_VERSION_CHECKED
    if _omnilink_pkg is None:
        raise RuntimeError(
            "omnilink-lib is not installed. Install with:\n"
            "    pip install -r projects/samples/demos/controllers/_omnilink_relay/requirements.txt\n"
            "(or just `pip install \"omnilink>={min}\"`).".format(min=MIN_OMNILINK_VERSION)
        )
    installed = getattr(_omnilink_pkg, "__version__", "0.0.0")
    if _parse_version(installed) < _parse_version(MIN_OMNILINK_VERSION):
        raise RuntimeError(
            "omnilink-lib {got} is older than the {want} required by the "
            "OmniLink chat-demo bridges. Upgrade with:\n"
            "    pip install -U \"omnilink>={want}\"".format(
                got=installed, want=MIN_OMNILINK_VERSION
            )
        )
    if not _OMNILINK_VERSION_CHECKED:
        print(f"[omnilink_relay] using omnilink-lib {installed} (floor {MIN_OMNILINK_VERSION})")
        _OMNILINK_VERSION_CHECKED = True
    return installed


def _check_pypi_for_newer_async() -> None:
    """Background thread: query PyPI for the latest omnilink release,
    print a one-time notice if the installed version is behind. Silenced
    with OMNILINK_VERSION_CHECK=0. Bounded: 3 s timeout, runs at most
    once per process."""
    global _NEW_VERSION_CHECK_DONE
    if _NEW_VERSION_CHECK_DONE:
        return
    if os.environ.get("OMNILINK_VERSION_CHECK", "1").strip() in ("0", "false", "no", ""):
        _NEW_VERSION_CHECK_DONE = True
        return
    _NEW_VERSION_CHECK_DONE = True

    def _worker():
        try:
            import json as _json
            import urllib.request as _ur
            installed = getattr(_omnilink_pkg, "__version__", "0.0.0")
            with _ur.urlopen("https://pypi.org/pypi/omnilink/json", timeout=3) as r:
                data = _json.loads(r.read().decode("utf-8"))
            latest = (data.get("info") or {}).get("version") or "0.0.0"
            if _parse_version(latest) > _parse_version(installed):
                print(
                    f"[omnilink_relay] note: omnilink-lib {installed} is installed, "
                    f"but {latest} is on PyPI. Upgrade with `pip install -U omnilink`.\n"
                    f"[omnilink_relay] (silence this check with OMNILINK_VERSION_CHECK=0)"
                )
        except Exception:
            # Network down, behind a firewall, PyPI rate-limited — none
            # of these are reasons to interrupt the bridge. Quietly skip.
            pass

    threading.Thread(target=_worker, name="omnilink-pypi-check", daemon=True).start()


BASE_URL = os.environ.get("OMNILINK_BASE_URL", "https://www.omnilink-agents.com").rstrip("/")
DEFAULT_ENGINE = os.environ.get("OMNILINK_ENGINE", "g1-engine")
DEFAULT_TEMPERATURE = float(os.environ.get("OMNILINK_TEMPERATURE", "0.1"))
# ⚠ PIN THE MODEL, OR A ROBOT CONVERSATION SILENTLY RUNS ON THE WEAKEST TIER.
# Sending no `model` does NOT mean "the platform picks a good one": g1-engine's
# adaptive selector (omnilink/api/_adaptive-model.ts) starts every turn at
# tier 0 -- `gemini-3.1-flash-lite` -- and only escalates on tool-chain depth
# past 25, or on cascading tool errors. A robot prompt is 1-3 tool calls and
# usually error-free, so it NEVER escalates: measured across a 20-prompt graded
# suite on the OMNIARM6 2F-140 chat demo (2026-08-16), every turn ran on flash-lite.
# The escalation ladder's own top tier is `gemini-3.5-flash` (GA at Google I/O
# 2026-05-19, benchmarks ahead of gemini-3.1-pro-preview on coding/agentic work
# while running ~4x faster and ~40% cheaper), so pinning there is the tier the
# platform would have chosen for a hard turn -- we just stop waiting for a
# 25-deep chain to ask for it. `gemini-3.5-pro` is deliberately NOT the default:
# verified 2026-08-16, it is still limited-preview on Vertex with no public
# model id, and it is absent from the server allowlist, so it would 400.
# Set OMNILINK_MODEL="" to restore the adaptive default.
DEFAULT_MODEL = os.environ.get("OMNILINK_MODEL", "gemini-3.5-flash").strip()
# Generous defaults so multi-step planning ("go forward 1m, then turn left 90 deg")
# doesn't run out of turns mid-chain or lose its earlier context. The relay still
# clamps via single-flight semantics, so these only bound the worst case.
MAX_TOOL_TURNS = int(os.environ.get("OMNILINK_MAX_TURNS", "16"))
HISTORY_LIMIT = int(os.environ.get("OMNILINK_HISTORY_LIMIT", "40"))

# How much we STORE, as opposed to how much we SEND to the model.
#
# HISTORY_LIMIT is a context-window decision: bigger costs latency and tokens
# on every single turn. The persisted record has no such cost -- it is one
# write -- so tying the two together only ever threw away history for free.
#
# 80, NOT the 100-entry endpoint cap, and the difference matters. The server's
# chat path compacts whenever a conversation exceeds
# `entryCountCeiling = maxConversationEntries - 20` = 80
# (api/_engine-common.ts), collapsing everything older into one recap plus the
# last 20 -- and it PERSISTS that compacted form, because the blob it writes
# back is built from the already-compacted message list. So a bridge writing
# 100 guarantees the next server-side turn rewrites the stored memory down to
# ~21 entries, which this bridge then re-expands from its own local history on
# its next write. Nothing is lost either way, but the two writers spend every
# turn undoing each other and the server's recap is repeatedly discarded.
#
# Staying at or under the server's own trigger means neither writer has to
# correct the other. If MAX_CONVERSATION_ENTRIES is raised server-side, raise
# this with it -- OMNILINK_PERSIST_LIMIT exists for exactly that.
PERSIST_LIMIT = int(os.environ.get("OMNILINK_PERSIST_LIMIT", "80"))
# How many PAST tool exchanges (one assistant tool_calls turn + the tool
# results answering it) are replayed to the model. The current turn's
# chain is always kept in full — this only bounds the *completed* ones.
#
# Why this is bounded at all: the relay used to replay every tool
# exchange in the 40-message window, which cost twice.
#   1. Payload. Tool results are the fattest messages in the transcript
#      (get_robot_state alone is multi-kB of JSON), so the re-sent
#      history grew to ~25-45 kB per round while the cacheable prefix
#      stayed fixed — the measured Gemini cache ratio fell from ~92% on
#      turn 1 to ~54% by turn 20 purely because the uncached tail grew.
#   2. Model tier. The platform escalates g1-engine off flash-lite once
#      it counts >= 8 tool calls in the messages it is handed
#      (_adaptive-model.ts). That counter is cumulative over the whole
#      replayed history, so from roughly the eighth turn onward EVERY
#      turn ran on a slower thinking model — measured as thinking tokens
#      appearing on turn 8 and never going away, and first-round latency
#      jumping from ~1.7 s to 7-24 s. Replaying a bounded number of
#      exchanges keeps the depth signal describing the CURRENT chain,
#      which is what it was designed to measure.
#
# What is NOT lost: the assistant's own text from a pruned exchange is
# kept, so the operator-visible transcript is intact, and live state is
# re-read through tools rather than recalled (the arm's main task and
# the platform guardrails both mandate that). Pairs are dropped whole —
# never an assistant tool_calls turn without its results, nor results
# without their call — so the provider's pairing invariant holds.
TOOL_HISTORY_EXCHANGES = int(os.environ.get("OMNILINK_TOOL_HISTORY", "2"))
# Short-term memory: persist chat history across world reloads via
# OmniLinkClient.set_memory() / get_memory(). The memory key is the
# agent_name, so all bridges spawned for the same robot id share a
# transcript ("OmniSim-husky"). Set OMNILINK_MEMORY=0 to disable —
# useful for offline-only demos or when you want each reload to start
# fresh. Stored under /api/short-term-memory on the platform side.
MEMORY_ENABLED_DEFAULT = os.environ.get("OMNILINK_MEMORY", "1").strip() not in ("0", "false", "no", "")
# Usage telemetry: poll the platform's /api/omni-key-usage rollup before
# and after each chat turn so the side menu can show how many tokens and
# credits the agent burned. Off-path: a network blip on /api/omni-key-usage
# does not block the chat dispatch. Disable with OMNILINK_USAGE=0.
USAGE_ENABLED_DEFAULT = os.environ.get("OMNILINK_USAGE", "1").strip() not in ("0", "false", "no", "")
# Voice output: after each agent text reply, optionally synthesize an
# MP3 via /api/tts and emit it as an `audio_out` event so the chat
# panel can play it back. Off by default to keep the demo cheap (TTS
# is a separate billed call). Set OMNILINK_VOICE_OUT=1 to enable.
VOICE_OUT_ENABLED_DEFAULT = os.environ.get("OMNILINK_VOICE_OUT", "0").strip() not in ("0", "false", "no", "")
# Per-request HTTP timeout (seconds) for the OmniLink client. Gemini tool
# turns occasionally run long; the old hard-coded 60 s was too tight and
# surfaced spurious "operation timed out" errors in the chat panel. Higher
# default, tunable via OMNILINK_TIMEOUT.
REQUEST_TIMEOUT = int(os.environ.get("OMNILINK_TIMEOUT", "120"))
# Transient-failure retries for one chat round-trip (read timeouts, network
# blips, 429/5xx). Connection-level errors fail fast so retrying is cheap;
# only a true read timeout is slow, so keep the count modest. 4xx errors
# (auth / BYOK / bad request) are never retried — they won't self-heal.
CHAT_RETRIES = int(os.environ.get("OMNILINK_RETRIES", "2"))
RETRY_BACKOFF_S = float(os.environ.get("OMNILINK_RETRY_BACKOFF", "1.5"))

# ── Latency tracing (opt-in, zero cost when unset) ───────────────────
#
# Set OMNILINK_TRACE=<path.jsonl> to have the relay append one JSON
# record per chat round-trip and one per completed turn. Used to answer
# "where do the seconds actually go" without guessing: queue wait, per
# round HTTP wall time, the server's own self-reported total (via the
# platform's `debug` trace), request payload composition, tool dispatch
# time, and the post-reply usage-meter/memory work.
TRACE_PATH = os.environ.get("OMNILINK_TRACE", "").strip()
_TRACE_LOCK = threading.Lock()


def _trace(record: Dict[str, Any]) -> None:
    if not TRACE_PATH:
        return
    try:
        record["ts"] = time.time()
        line = json.dumps(record, default=str)
        with _TRACE_LOCK:
            with open(TRACE_PATH, "a", encoding="utf-8") as fh:
                fh.write(line + "\n")
    except Exception:
        pass


# Tool class lives in tool.py so the package public API exports a
# single canonical Tool that is the same as the one OmniLinkRelay uses.
from .tool import Tool  # noqa: F401
from .action_journal import ActionJournal, _env_flag


def prune_tool_scaffolding(
    messages: List[Dict[str, Any]],
    keep_exchanges: int = TOOL_HISTORY_EXCHANGES,
) -> List[Dict[str, Any]]:
    """Replay only the most recent ``keep_exchanges`` tool exchanges.

    An *exchange* is one assistant turn carrying ``tool_calls`` plus the
    run of ``role: "tool"`` results that answers it. Older exchanges are
    dropped as a UNIT — both halves or neither — so the transcript can
    never end up with a call that has no result (or a result with no
    call), which is the pairing failure the platform's
    ``repairToolPairing`` exists to clean up after. Any prose the
    assistant said alongside a pruned call is preserved as a plain
    assistant message, so nothing the operator saw disappears.

    Also drops leading orphan tool results: slicing a transcript to the
    last N messages can cut through the middle of an exchange, leaving
    results whose call is no longer in the window.
    """
    if keep_exchanges < 0:
        return list(messages)

    groups: List[tuple] = []
    i = 0
    leading_orphans: List[int] = []
    seen_call = False
    while i < len(messages):
        msg = messages[i]
        role = msg.get("role")
        if role == "assistant" and msg.get("tool_calls"):
            seen_call = True
            j = i + 1
            tool_idxs = []
            while j < len(messages) and messages[j].get("role") == "tool":
                tool_idxs.append(j)
                j += 1
            groups.append((i, tool_idxs))
            i = j
            continue
        if role == "tool" and not seen_call:
            leading_orphans.append(i)
        i += 1

    drop = set(leading_orphans)
    text_only = set()
    prunable = groups[: max(0, len(groups) - keep_exchanges)]
    for assistant_idx, tool_idxs in prunable:
        text_only.add(assistant_idx)
        drop.update(tool_idxs)

    if not drop and not text_only:
        return list(messages)

    out: List[Dict[str, Any]] = []
    for idx, msg in enumerate(messages):
        if idx in text_only:
            text = (msg.get("content") or "").strip()
            if text:
                out.append({"role": "assistant", "content": text})
            continue
        if idx in drop:
            continue
        out.append(msg)
    return out


# ── Did the tool refuse? ─────────────────────────────────────────────
#
# `error` is NOT a failure flag. PROTOCOL.md 5.4.1 rule 2 makes `error` the
# NUMERIC RESIDUAL of a completed motion -- `achieved - commanded`, in the
# action's own units -- and the mobile bridge has written it that way since
# a2a8da5d (2026-07-26). The old predicate here was `ok = "error" not in
# result`, which predates that contract and therefore fires on SUCCESS:
# measured, a drive that landed 1.003 m against a 1.0 m command was recorded
# as `result: "err", summary: "error: 0.0028145549502676115"`.
#
# That is not cosmetic. This verdict is what ActionJournal.record(ok=...)
# persists to disk, and the journal is served straight back to the model by
# get_action_history -- the tool the system prompt calls the authoritative
# log and tells the agent to consult before answering "did that land". So a
# presence-check on a mandated success key was feeding the anti-fabrication
# machinery fabricated failures.
#
# Classify on the REFUSAL contract instead, which is explicit and cannot be
# confused with a measurement:
#   * `accepted: False`  -- the bridge-wide refusal marker (5.4.1 rule 5,
#     and also the bounds/`wait=false` refusals, which carry NO `error` key
#     at all and so used to be scored as successes by the old predicate).
#   * `error` as a STRING -- "busy", "unreachable_target", "unknown tool",
#     "dispatch failed: ...". Every non-conforming bridge in the tree still
#     reports this way, so they keep working unchanged.
#   * `http_status >= 400` -- the 409-busy envelope.
# A numeric (or null) `error` is a residual and means the action ran.
def _result_failed(result: Any) -> bool:
    """True only when a tool REFUSED or crashed. Never raises."""
    if not isinstance(result, dict):
        return False
    if result.get("accepted") is False:
        return True
    err = result.get("error")
    if isinstance(err, str) and err.strip():
        return True
    status = result.get("http_status")
    if isinstance(status, int) and not isinstance(status, bool) and status >= 400:
        return True
    return False


# ── Relay ────────────────────────────────────────────────────────────

class OmniLinkRelay:
    """Routes side-menu prompts through OmniLink's /api/chat.

    The relay maintains a short conversation history (last HISTORY_LIMIT
    turns) so multi-step prompts ("now drive 50 cm more") have context.
    Tool calls are executed locally, results fed back as tool messages,
    and the loop iterates up to MAX_TOOL_TURNS before forcing a final
    text response.

    Use `dispatch_async(text, on_event)` from the Webots main thread.
    A worker thread does the blocking HTTP. `on_event(kind, payload)`
    is invoked from the worker for {"status", "tool", "agent", "error"}.
    The bridge's wwi loop translates these to "status:...", "tool:...",
    "agent:...", "error:..." lines.
    """

    def __init__(
        self,
        omni_key: str,
        agent_name: str,
        main_task: str,
        tools: List[Tool],
        engine: str = DEFAULT_ENGINE,
        temperature: float = DEFAULT_TEMPERATURE,
        memory_enabled: Optional[bool] = None,
        usage_enabled: Optional[bool] = None,
        voice_out_enabled: Optional[bool] = None,
        model: Optional[str] = None,
    ) -> None:
        check_omnilink_installation()
        _check_pypi_for_newer_async()
        if not omni_key:
            raise ValueError("OMNI_KEY is empty")
        self.omni_key = omni_key
        self.agent_name = agent_name
        self.main_task = main_task
        self.engine = engine
        self.temperature = temperature
        # None -> fall back to DEFAULT_MODEL; "" -> send no model at all and
        # let the platform's adaptive selector decide (see DEFAULT_MODEL).
        self.model = (DEFAULT_MODEL if model is None else model).strip()

        # Every dispatched tool call is journalled here, and the journal is
        # handed straight back to the agent as `get_action_history`. This is
        # registered by the RELAY rather than by each bridge on purpose: the
        # fabrications it exists to stop were all "what did I just do"
        # questions, so the record has to cover every robot and every tool
        # automatically -- including tools added later, which a per-bridge
        # opt-in would silently miss. A bridge that already defines its own
        # `get_action_history` keeps it.
        # Keyed by agent_name so three bridges in one demo keep three separate
        # journals instead of clobbering one shared file.
        self.journal = ActionJournal(owner=agent_name)
        tools = list(tools)
        if not any(t.name == "get_action_history" for t in tools):
            tools.append(self.journal.as_tool())

        # Operator-designated durable facts. See PIN_PREFIX for why this
        # exists: the heuristic notes tier keeps the operator's words but
        # cannot tell a standing fact from small talk, so under sustained
        # chatter the facts lose. Marking one explicitly is exact and costs
        # nothing per turn.
        self._pinned_facts: List[str] = []
        if not any(t.name == "remember_this" for t in tools):
            tools.append(Tool(
                name="remember_this",
                description=(
                    "Store a fact about this site, robot or operator PERMANENTLY, "
                    "so you still know it after the conversation has moved on and "
                    "after a restart.\n"
                    "\n"
                    "Call this the moment the operator states something that should "
                    "outlive the conversation: a badge or shift number, a piece of "
                    "equipment that is damaged or off-limits, a naming convention, a "
                    "preference for how they want things done. Ordinary conversation "
                    "is already remembered for a while and does NOT need this.\n"
                    "\n"
                    "Store the fact in their words, self-contained, so it still makes "
                    "sense with no surrounding context: 'TROLLEY_E is damaged and must "
                    "not be towed', not 'the one they mentioned'. Do not store your own "
                    "actions or conclusions -- only what you were TOLD."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "fact": {
                            "type": "string",
                            "description": "The fact, self-contained, in the operator's words.",
                        },
                    },
                    "required": ["fact"],
                },
                dispatch=lambda a: self._remember(a.get("fact", "")),
            ))

        self.tools = {t.name: t for t in tools}
        self._tool_defs = [t.to_definition() for t in tools]
        self._tool_names = ", ".join(t.name for t in tools)
        # The OmniLinkClient instance is the single point of contact with
        # the OmniLink platform. Sim and real-robot integrations both go
        # through this same object. This preserves the agent/bridge interface;
        # it does not claim physical dynamics or safety parity. Swap the bridge's local dispatch handlers
        # for real-robot drivers and the agent code (this file) stays
        # byte-identical.
        self._client = OmniLinkClient(
            omni_key=self.omni_key,
            base_url=BASE_URL,
            timeout=REQUEST_TIMEOUT,
        )
        self._lock = threading.RLock()
        self.history: List[Dict[str, Any]] = []
        # Short-term memory: prime self.history from OmniLink so the
        # agent picks up where the last session left off (operator
        # reloads the world, types "now drive 50 cm more" — that "now"
        # has continuity from the previous chat). Best-effort: any
        # failure leaves history empty and the session starts fresh.
        self._memory_enabled = (
            memory_enabled if memory_enabled is not None else MEMORY_ENABLED_DEFAULT
        )
        if self._memory_enabled:
            try:
                stored = self._client.get_memory(self.agent_name)
                if stored:
                    self.history = _restore_history(stored, HISTORY_LIMIT)
                    print(f"[omnilink_relay] {self.agent_name}: restored {len(self.history)} "
                          f"messages from short-term memory")
            except Exception as e:
                print(f"[omnilink_relay] memory restore skipped: {e}")
        # Voice output: per-turn TTS pass-through; emits an audio_out
        # event the chat panel plays back. Off by default (TTS bills
        # separately and most demos don't need it).
        self._voice_out_enabled = (
            voice_out_enabled if voice_out_enabled is not None else VOICE_OUT_ENABLED_DEFAULT
        )
        # Usage telemetry: baseline the meter at construction so the first
        # turn's snapshot reflects only that turn's usage, not whatever
        # background traffic the platform saw before the bridge launched.
        self._usage_enabled = (
            usage_enabled if usage_enabled is not None else USAGE_ENABLED_DEFAULT
        )
        self._meter: Optional[Any] = None
        self._last_usage: Optional[Dict[str, Any]] = None
        if self._usage_enabled and UsageMeter is not None:
            try:
                self._meter = UsageMeter(self._client)
                self._meter.start()
            except Exception as e:
                print(f"[omnilink_relay] usage meter init skipped: {e}")
                self._meter = None
        # Single-flight dispatcher thread: serialises chats so the agent
        # doesn't see two overlapping conversations on the same robot.
        self._queue: Queue = Queue(maxsize=32)
        self._memory_write_lock = threading.Lock()
        self._tts_lock = threading.Lock()
        self._meter_lock = threading.Lock()
        self._closed = False
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

        # PRESENCE. Without this a robot that is running looks identical, from
        # the platform's side, to one whose process died an hour ago: the
        # profile still lists, the tool list still reads fine, and every
        # platform-initiated tool call quietly hits a closed socket. Measured
        # twice in one day on the shipped warehouse agents, found by hand both
        # times because nothing reports it. A few bytes every 30 s is the whole
        # fix. Opt out with OMNILINK_PRESENCE=0.
        self._presence_endpoint = ""
        # interval_ms is not decoration: the platform sizes its staleness
        # window from it. Without it a 30 s beat is judged against a window
        # built for a browser tab pinging every 3 s, and a healthy robot reads
        # as offline for 20 of every 30 seconds.
        self._presence_interval_s = 30.0
        self._presence_detail: Dict[str, Any] = {
            "tools": len(self._tool_defs),
            "engine": self.engine,
            "interval_ms": int(self._presence_interval_s * 1000),
        }
        self._presence_thread: Optional[threading.Thread] = None
        if _env_flag("OMNILINK_PRESENCE", True):
            self._presence_thread = threading.Thread(
                target=self._presence_loop, name="omnilink-presence", daemon=True)
            self._presence_thread.start()

    def _reground_if_unread(
        self,
        prompt: str,
        called_tools: List[str],
        messages: List[Dict[str, Any]],
        handle: DispatchHandle,
    ) -> Optional[str]:
        """Re-answer a state question that was answered without reading state.

        Returns the replacement text, or None to leave the original answer
        alone. Never raises: a grounding gate that can break the chat path
        would be a worse bug than the one it fixes.
        """
        try:
            kind = _state_question_kind(prompt)
            if kind is None:
                return None
            wanted = ("get_action_history" if kind == "history"
                      else "get_robot_state")

            # THE PRECONDITION IS "READ THE RIGHT THING", NOT "READ SOMETHING".
            #
            # This used to accept any get_*/list_* call, which is the same
            # wrong-surface mistake the gate exists to prevent. Measured: asked
            # "anything I should be worried about?" while HELD, both tugs
            # called get_peer_state -- the OTHER robot's state -- which
            # satisfied the gate while never touching their own autonomy_hold,
            # and both answered "everything looks normal" while stopped and
            # waiting on the operator. The arm agent got the identical
            # question in the identical state, called get_robot_state, saw
            # the hold, and disclosed it. The only difference was which
            # surface was read.
            #
            # Both fabrications in that run came from this hole, so the gate
            # now requires the specific grounding surface. Reading a peer is
            # useful and still allowed -- it just no longer counts as having
            # checked yourself.
            if any(t == wanted for t in called_tools):
                return None            # it read the right thing; leave it alone
            tool = self.tools.get(wanted) or self.tools.get("get_robot_state")
            if tool is None:
                return None            # this bridge has no such surface

            with handle.execution_gate:
                if handle.is_cancelled():
                    return None
                result = tool.dispatch({})
            self.journal.record(tool.name, {}, ok=not _result_failed(result),
                                summary="auto-read (grounding gate)",
                                result=result if isinstance(result, dict) else None)

            grounded = list(messages)
            grounded.append({
                "role": "user",
                "content": (
                    "SYSTEM: you answered that without reading your state. "
                    f"Here is the live output of {tool.name} taken just now:\n"
                    + json.dumps(result, default=str)[:4000]
                    + "\n\nAnswer the operator's question again using ONLY these "
                      "numbers. If they contradict what you just said, correct "
                      "yourself plainly. If the answer genuinely is not in "
                      "here, say you do not track it -- do not estimate."
                ),
            })
            data = self._post_chat(grounded)
            if handle.is_cancelled():
                return None
            text = (data.get("text") or "").strip()
            if not text:
                return None
            print(f"[omnilink_relay] {self.agent_name}: regrounded a state "
                  f"answer via {tool.name}")
            return text
        except Exception as e:
            print(f"[omnilink_relay] grounding gate skipped ({e})")
            return None

    def _log_quarantine(self, text: str) -> None:
        # Loud on purpose: silently editing what an agent said is a serious
        # act, and if this ever fires on an innocent apology I want it visible
        # in the log rather than discovered as missing history.
        print(f"[omnilink_relay] {self.agent_name}: NOT persisting a reply that "
              f"disavows its own instruments: {text[:120]!r}")

    def _remember(self, fact: Any) -> Dict[str, Any]:
        """Mark one statement durable. Refuses rather than pretending."""
        text = str(fact or "").strip().replace("\n", " ")
        if not text:
            return {"accepted": False,
                    "say": "I need the fact itself before I can remember it."}
        if len(text) > 200:
            text = text[:197] + "..."
        with self._lock:
            if any(text == p for p in self._pinned_facts):
                return {"accepted": True, "already_known": True,
                        "say": f"I already have that noted: {text}"}
            self._pinned_facts.append(text)
            if len(self._pinned_facts) > PINNED_MAX:
                dropped = self._pinned_facts.pop(0)
                self._log_pin_drop(dropped)
            count = len(self._pinned_facts)
        self._persist_memory_async()
        return {"accepted": True, "stored": text, "total_facts": count,
                "say": f"Noted permanently: {text}"}

    def _log_pin_drop(self, dropped: str) -> None:
        # Bounded on purpose, and loud when the bound bites -- silently
        # forgetting something the operator explicitly asked us to keep would
        # be worse than the eviction problem this feature exists to solve.
        print(f"[omnilink_relay] {self.agent_name}: pinned-fact limit "
              f"({PINNED_MAX}) reached; dropped the oldest: {dropped!r}")

    def set_presence_endpoint(self, url: str, **detail: Any) -> None:
        """Tell the platform WHERE this runtime answers.

        The bridge knows its own `/tool` URL; the relay does not. Publishing it
        is what lets a roster show a stale callback URL as stale instead of
        failing silently on the next tool call.
        """
        self._presence_endpoint = str(url or "")
        for k, v in detail.items():
            if isinstance(v, (str, int, float, bool)) or v is None:
                self._presence_detail[k] = v

    def _presence_loop(self) -> None:
        interval = self._presence_interval_s
        while not self._closed:
            try:
                body: Dict[str, Any] = {
                    "agentName": self.agent_name,
                    "kind": "bridge",
                    "detail": dict(self._presence_detail),
                }
                if self._presence_endpoint:
                    body["endpoint"] = self._presence_endpoint
                self._post_presence(body)
            except Exception:
                # Presence is telemetry. It must never take the robot down, and
                # it must never spam the log on a flaky link -- a missed beat
                # already shows up as "offline", which is the honest reading.
                pass
            for _ in range(int(interval * 2)):
                if self._closed:
                    return
                time.sleep(0.5)

    def _post_presence(self, body: Dict[str, Any]) -> None:
        import urllib.request
        req = urllib.request.Request(
            f"{BASE_URL}/api/relay-heartbeat",
            data=json.dumps(body).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {self.omni_key}"},
        )
        with urllib.request.urlopen(req, timeout=8):
            pass

    # ── Public API ────────────────────────────────────────────────

    @property
    def tool_defs(self) -> List[Dict[str, Any]]:
        """The AUTHORITATIVE tool list, including the ones the relay adds.

        Push THIS to the platform profile, never the caller's own list. The
        relay appends get_action_history AFTER the bridge has built its tools,
        so a profile built from the bridge's list advertises everything except
        the anti-fabrication tool -- measured: the pushed profiles carried 19
        and 26 tools and not one of them was get_action_history, so the fix
        worked on /prompt and was invisible to the web UI and to every
        delegation round trip.
        """
        return list(self._tool_defs)

    def dispatch_async(
        self,
        text: str,
        on_event: Callable[[str, Dict[str, Any]], None],
    ) -> DispatchHandle:
        """Enqueue a prompt and return a cancellation handle."""
        handle = DispatchHandle()
        if self._closed:
            on_event("error", {"text": "relay is closed"})
            handle.cancel()
            return handle
        try:
            self._queue.put_nowait((text, on_event, handle))
        except Full:
            handle.cancel()
            on_event("error", {"text": "relay queue is full; try again after the current prompt finishes"})
        return handle

    def dispatch_sync(self, text: str, timeout_s: float = 90.0) -> Dict[str, Any]:
        """Run dispatch_async and wait for it to finish. Returns a compact
        result dict {response, actions: [{tool, result, summary}], error?}
        shaped to match the bridge's HTTP /prompt response so HTTP callers
        and side-menu callers see the same wire format.
        """
        done = threading.Event()
        actions: List[Dict[str, Any]] = []
        agent_text = {"value": ""}
        error_text = {"value": ""}

        def _cb(kind: str, payload: Dict[str, Any]) -> None:
            if kind == "tool":
                actions.append({
                    "tool": payload.get("name"),
                    "result": payload.get("status"),
                    "summary": payload.get("summary", ""),
                })
            elif kind == "agent":
                agent_text["value"] = str(payload.get("text", ""))
            elif kind == "error":
                error_text["value"] = str(payload.get("text", ""))
                done.set()
            elif kind == "status" and payload.get("state") == "idle":
                done.set()

        handle = self.dispatch_async(text, _cb)
        if not done.wait(timeout=timeout_s):
            in_flight = handle.cancel()
            return {
                "response": "",
                "actions": actions,
                "error": "timeout_after_actions" if actions or in_flight else "timeout",
                "cancelled": True,
                "in_flight": in_flight,
                "do_not_retry": bool(actions) or in_flight,
            }
        out: Dict[str, Any] = {"response": agent_text["value"], "actions": actions}
        if error_text["value"]:
            out["error"] = error_text["value"]
        return out

    def latest_usage(self) -> Optional[Dict[str, Any]]:
        """Most recent per-turn usage delta, or None if no turn has run.
        Bridge /usage HTTP endpoint surfaces this; the side menu polls it
        so it can show a running 'tokens/credits this session' tally
        without intercepting chat events."""
        return self._last_usage

    def relay_identity(self) -> Dict[str, Any]:
        """Stable, non-secret identity for bridge health/benchmark probes.

        Inferring the relay from ``latest_usage`` made a fresh bridge
        unverifiable until a metered turn completed, and made usage-disabled
        OmniLink sessions unverifiable forever. The bridge already knows what
        it constructed; publish that fact directly without exposing keys.
        """
        return {"kind": "omnilink", "engine": self.engine}

    # ── Voice I/O ─────────────────────────────────────────────────

    def transcribe(self, audio_bytes: bytes, *, mime_type: str = "audio/webm") -> str:
        """Whisper STT pass-through. Returns the transcribed text, or
        empty string on failure. Bridges call this when the chat panel
        sends an `audio_in:<base64>` message."""
        try:
            r = self._client.transcribe(audio_bytes, mime_type=mime_type)
            return str(r.get("text", "")).strip()
        except Exception as e:
            print(f"[omnilink_relay] transcribe failed: {e}")
            return ""

    def synthesize(self, text: str) -> Optional[bytes]:
        """Chirp3-HD TTS pass-through. Returns MP3 bytes ready to ship
        to the chat panel as `audio_out:<base64>`, or None on failure.

        Bridges typically call this after each agent text reply when
        OMNILINK_VOICE_OUT=1, so the operator can hear the agent's
        plan as well as read it.
        """
        if not text:
            return None
        try:
            return self._client.synthesize_to_bytes(text, audio_encoding="MP3")
        except AttributeError:
            # Older omnilink-lib versions only have synthesize() (no _to_bytes).
            try:
                import base64 as _b64
                r = self._client.synthesize(text, audio_encoding="MP3")
                return _b64.b64decode(r.get("audioContent", ""))
            except Exception as e:
                print(f"[omnilink_relay] synthesize failed: {e}")
                return None
        except Exception as e:
            print(f"[omnilink_relay] synthesize failed: {e}")
            return None

    def close(self) -> None:
        self._closed = True
        try:
            self._queue.put_nowait((None, None, None))
        except Full:
            pass

    # ── Worker loop ───────────────────────────────────────────────

    def _run(self) -> None:
        while True:
            try:
                item = self._queue.get(timeout=1.0)
            except Empty:
                if self._closed:
                    return
                continue
            if item is None or item[0] is None:
                return
            text, on_event, handle = item
            try:
                self._dispatch_one(text, on_event, handle)
            except Exception as e:
                err = f"{type(e).__name__}: {e}"
                traceback.print_exc()
                try:
                    on_event("error", {"text": err})
                except Exception:
                    pass

    # ── One prompt -> /api/chat loop ──────────────────────────────

    def _dispatch_one(
        self,
        text: str,
        on_event: Callable[[str, Dict[str, Any]], None],
        handle: DispatchHandle,
    ) -> None:
        if handle.is_cancelled():
            return
        t_start = time.perf_counter()
        queued_ms = (t_start - handle.enqueued_at) * 1000.0
        rounds: List[Dict[str, Any]] = []
        tool_ms_total = 0.0
        on_event("status", {"state": "thinking"})

        with self._lock:
            self.history.append({"role": "user", "content": text})
            # The window is the last HISTORY_LIMIT messages, minus the
            # tool scaffolding of all but the most recent few exchanges
            # (see TOOL_HISTORY_EXCHANGES for why, and for what that
            # deliberately does NOT cost).
            messages = prune_tool_scaffolding(list(self.history[-HISTORY_LIMIT:]))
            # self.history is the local transcript, not the wire payload;
            # bound it so a long-lived bridge doesn't grow without limit.
            if len(self.history) > 4 * HISTORY_LIMIT:
                del self.history[: len(self.history) - 4 * HISTORY_LIMIT]

        last_text = ""
        tool_calls: List[Dict[str, Any]] = []
        called_tools: List[str] = []
        for turn in range(MAX_TOOL_TURNS):
            if handle.is_cancelled():
                return
            data = self._post_chat(messages)
            if TRACE_PATH:
                rt = dict(getattr(self, "_last_round_trace", None) or {})
                rt["round"] = turn
                rounds.append(rt)
            if handle.is_cancelled():
                return
            if not data.get("ok", True) and data.get("error"):
                on_event("error", {"text": str(data["error"])})
                return

            last_text = data.get("text") or ""
            tool_calls = data.get("toolCalls") or []
            if not tool_calls:
                break

            # Record the assistant turn so the next call sees the tool
            # request (and so the in-memory history stays coherent).
            assistant_msg: Dict[str, Any] = {"role": "assistant", "content": last_text}
            if tool_calls:
                assistant_msg["tool_calls"] = tool_calls
            messages.append(assistant_msg)
            with self._lock:
                self.history.append(assistant_msg)

            for tc in tool_calls:
                tool_name = tc.get("name", "")
                args = tc.get("arguments") or {}
                tool = self.tools.get(tool_name)
                if tool is None:
                    result = {"error": f"unknown tool '{tool_name}'",
                              "known": list(self.tools.keys())}
                    on_event("tool", {
                        "name": tool_name, "status": "err",
                        "summary": "unknown tool",
                    })
                    self.journal.record(
                        tool_name, args, ok=False,
                        summary="unknown tool", result=result,
                    )
                else:
                    _t_tool = time.perf_counter()
                    with handle.execution_gate:
                        if handle.is_cancelled():
                            return
                        try:
                            result = tool.dispatch(args)
                        except Exception as e:
                            result = {"error": f"dispatch failed: {e}"}
                    tool_ms_total += (time.perf_counter() - _t_tool) * 1000.0
                    # A numeric `error` is the 5.4.1 residual, not a fault --
                    # see _result_failed. This verdict lands in the journal
                    # the model reads back, so getting it wrong invents a
                    # failure the robot never had.
                    ok = not _result_failed(result)
                    summary = self._summarize_result(result)
                    on_event("tool", {
                        "name": tool_name,
                        "status": "ok" if ok else "err",
                        "summary": summary,
                    })
                    # Journal AFTER dispatch so the record reflects what the
                    # tool actually returned, not what was requested. Reading
                    # the journal is itself journalled -- "did you check?" is
                    # a fair question too.
                    self.journal.record(
                        tool_name, args, ok=ok,
                        summary=summary, result=result if isinstance(result, dict) else None,
                    )

                called_tools.append(tool_name)
                tool_msg = {
                    "role": "tool",
                    "tool_call_id": tc.get("id") or tool_name,
                    "name": tool_name,
                    "content": json.dumps(result, default=str),
                }
                messages.append(tool_msg)
                with self._lock:
                    self.history.append(tool_msg)

        if handle.is_cancelled():
            return
        if tool_calls:
            on_event("error", {"text": f"agent exceeded the {MAX_TOOL_TURNS}-turn tool limit"})
            return

        # GROUNDING GATE. Measured across three builds: every single
        # fabrication landed on a turn where the model answered a question
        # about its own state or history WITHOUT reading either. The turns
        # that did read were accurate, and in every failing case the correct
        # answer was one tool call away -- the same question asked again, and
        # answered from a read, came back right.
        #
        # The tool descriptions already say "ALWAYS call this". Prompt text
        # has now been tried three times and is measurably not enough, so this
        # is structural instead: if the operator asked a state question and
        # nothing was read, fetch the state ourselves, hand it over, and let
        # the model answer again. Once only -- a gate that can recurse would
        # trade fabrications for hangs.
        if not handle.is_cancelled():
            regrounded = self._reground_if_unread(text, called_tools, messages, handle)
            if regrounded is not None:
                last_text = regrounded

        if last_text:
            on_event("agent", {"text": last_text})
            with self._lock:
                self.history.append({"role": "assistant", "content": last_text})
            # Optional TTS: synthesize the agent text and emit an
            # audio_out event so the chat panel can play it. Best-effort;
            # a failed TTS doesn't break the chat path.
            if self._voice_out_enabled:
                self._synthesize_async(last_text, on_event)
        else:
            on_event("agent", {"text": "(no text response)"})
        # Usage delta for this turn — two GETs to /api/omni-key-usage
        # (snapshot + re-baseline). It is TELEMETRY: nothing in the
        # operator's answer depends on it, yet it used to run inline
        # here, before the "idle" event that HTTP callers block on, so
        # every single turn paid for it. Measured on the warehouse arm
        # against the cloud engine: a flat ~1.9 s, 19.8% of total suite
        # wall time — the second largest term after the model calls
        # themselves. It now runs on its own thread; `latest_usage()`
        # and the `usage` chat event still land, just a beat later.
        _t_meter = time.perf_counter()
        self._snapshot_usage_async(on_event)
        # Persist the updated history to OmniLink short-term memory so
        # the next session (after a world reload, a Webots restart, or
        # even a different OmniSim instance pointed at the same key)
        # can pick up the conversation. Fire-and-forget on its own
        # thread — the dispatcher's worker shouldn't block on memory
        # writes, and a failed write is not a fatal error.
        meter_ms = (time.perf_counter() - _t_meter) * 1000.0
        if self._memory_enabled:
            self._persist_memory_async()
        if TRACE_PATH:
            _trace({
                "kind": "turn",
                "agent": self.agent_name,
                "prompt": text[:200],
                "queued_ms": round(queued_ms, 1),
                "llm_rounds": len(rounds),
                "tool_ms": round(tool_ms_total, 1),
                "meter_ms": round(meter_ms, 1),
                "history_len": len(self.history),
                "total_ms": round((time.perf_counter() - t_start) * 1000.0, 1),
                "rounds": rounds,
            })
        on_event("status", {"state": "idle"})

    def _snapshot_usage_async(
        self,
        on_event: Callable[[str, Dict[str, Any]], None],
    ) -> None:
        """Read the per-turn usage delta off the critical path.

        Serialised on `_meter_lock` (blocking, not try-acquire) because
        skipping a snapshot would silently fold one turn's tokens into
        the next turn's delta — a wrong number is worse than a late one.
        The platform's rollup stays the authoritative source; a local
        tool-call count would miss memory writes, profile pushes and
        retries.
        """
        if self._meter is None:
            return

        def _worker():
            with self._meter_lock:
                try:
                    delta = self._meter.snapshot()
                    payload = delta.to_dict()
                    payload["text"] = delta.report()
                    self._last_usage = payload
                    on_event("usage", payload)
                    # Re-baseline so the next turn's snapshot is per-turn,
                    # not cumulative. Operators care about "what did THAT
                    # prompt cost", not "the running total since boot".
                    self._meter.start()
                except Exception as e:
                    # Best-effort. Keep going.
                    print(f"[omnilink_relay] usage snapshot failed: {e}")

        threading.Thread(target=_worker, name="omnilink-usage", daemon=True).start()

    def _synthesize_async(
        self,
        text: str,
        on_event: Callable[[str, Dict[str, Any]], None],
    ) -> None:
        """Fire-and-forget TTS. Posts the resulting base64-encoded MP3
        as an `audio_out` event so the chat panel can play it."""
        def _worker():
            if not self._tts_lock.acquire(blocking=False):
                return
            try:
                audio = self.synthesize(text)
                if not audio:
                    return
                import base64 as _b64
                b64 = _b64.b64encode(audio).decode("ascii")
                on_event("audio_out", {"audio_b64": b64, "mime_type": "audio/mpeg"})
            except Exception as e:
                print(f"[omnilink_relay] tts async failed: {e}")
            finally:
                self._tts_lock.release()

        threading.Thread(target=_worker, name="omnilink-tts", daemon=True).start()

    def _persist_memory_async(self) -> None:
        """Fire-and-forget memory write. Bounded so a hung OmniLink write
        doesn't pile up threads if the user spams prompts.

        WHAT WE STORE IS NOT WHAT WE SEND. This used to persist exactly the
        context window -- ``set_memory(agent, history[-40:])`` -- which is a
        whole-blob OVERWRITE of the platform's copy. Two things followed, and
        both were measured rather than theorised:

        1. Turns did not merely fall out of context, they were DELETED at the
           source of truth. Facts seeded early in a session were gone from the
           platform blob entirely, not just un-recalled. "Its memory lives with
           us" was true for about twenty exchanges and false forever after.
        2. Work done on another surface was destroyed. The server appends up
           to its own ceiling; a bridge then wrote its 40 over the top, so a
           long headless conversation vanished the next time a robot persisted.

        The context window stays at HISTORY_LIMIT -- that is a latency and cost
        decision and it is a good one. The stored record is now separate and
        larger, and it is MERGED with what is already on the platform instead
        of replacing it. Whatever this bridge has not seen (another surface,
        another session) survives.
        """
        with self._lock:
            snapshot = list(self.history[-PERSIST_LIMIT:])
        # Drop capitulations on the way to storage only. They stay in
        # self.history so the operator can still see what was said in THIS
        # conversation; they must not become the durable context that grounds
        # every later turn. See _is_capitulation.
        kept = []
        for m in snapshot:
            if m.get("role") == "assistant" and _is_capitulation(m.get("content") or ""):
                self._log_quarantine(m.get("content") or "")
                continue
            kept.append(m)
        snapshot = kept

        def _worker():
            if not self._memory_write_lock.acquire(blocking=False):
                return
            try:
                conv = _to_memory_format(snapshot)
                with self._lock:
                    pins = list(self._pinned_facts)
                merged = conv
                try:
                    stored = self._client.get_memory(self.agent_name) or []
                    merged = _merge_memory(stored, conv, PERSIST_LIMIT, pins)
                except Exception:
                    # Read failed: fall back to writing our own view rather
                    # than skipping the write entirely. Losing the merge is
                    # bad; losing this session's turns as well is worse.
                    pass
                self._client.set_memory(self.agent_name, merged)
            except Exception as e:
                # Memory is best-effort. Log and move on.
                print(f"[omnilink_relay] memory persist failed: {e}")
            finally:
                self._memory_write_lock.release()

        threading.Thread(target=_worker, name="omnilink-mem-write", daemon=True).start()

    # ── /api/chat via OmniLinkClient ──────────────────────────────

    def _post_chat(self, messages: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Single chat round-trip through the OmniLink Python client.

        Identical to what a real-world OmniLink agent would do — we
        construct the message history + system instruction (with the
        bridge's tool surface in ``availableToolDetails``) and hand the
        client.chat() call the same kwargs an external integrator
        would. The only OmniSim-specific piece is the ``tools`` argument
        list itself: those describe the robot's action surface, which
        is the same description whether the robot is simulated or real.
        """
        last_err = "unknown error"
        # One id per logical chat turn, reused across transport retries. If
        # the server processed a request but the response was lost, a retry
        # must not create a second billable/model turn with different tools.
        request_id = str(uuid.uuid4())
        system_instruction = {
            "mainTask": self.main_task,
            "availableTools": self._tool_names,
            "availableToolDetails": self._tool_defs,
            "allowToolUse": True,
        }
        rt: Dict[str, Any] = {}
        if TRACE_PATH:
            try:
                rt = {
                    "n_messages": len(messages),
                    "msgs_bytes": len(json.dumps(messages, default=str)),
                    "task_bytes": len(self.main_task),
                    "tooldefs_bytes": len(json.dumps(self._tool_defs, default=str)),
                    "n_tools": len(self._tool_defs),
                    "attempts": 0,
                }
                rt["req_bytes"] = rt["msgs_bytes"] + rt["tooldefs_bytes"] + rt["task_bytes"]
            except Exception:
                rt = {}
            self._last_round_trace = rt
        for attempt in range(1 + max(0, CHAT_RETRIES)):
            try:
                _t0 = time.perf_counter()
                data = self._client.chat(
                    messages=messages,
                    agent_name=self.agent_name,
                    engine=self.engine,
                    temperature=self.temperature,
                    system_instruction=system_instruction,
                    # Extra fields go straight into the request body. The
                    # OmniSim chat-with-tools loop is stateless on the
                    # server (we manage history client-side), and we want a
                    # fresh request id per turn so cache attribution works.
                    usePromptPipeline=True,
                    skipMemory=True,
                    requestId=request_id,
                    # Extra kwargs land in the request body verbatim, and
                    # g1-engine validates `model` against its allowlist
                    # (400 UNKNOWN_GEMINI_MODEL on a typo, rather than a
                    # Google 404 that would cool the user's BYOK credential).
                    # Omitted entirely when empty -- that is the documented
                    # way to ask for the adaptive default.
                    **({"model": self.model} if self.model else {}),
                    **({"debug": True} if TRACE_PATH else {}),
                )
                if TRACE_PATH:
                    rt["http_ms"] = round((time.perf_counter() - _t0) * 1000.0, 1)
                    rt["attempts"] = attempt + 1
                    try:
                        rt["server_ms"] = ((data.get("debug") or {}).get("timing") or {}).get("totalMs")
                        um = ((data.get("raw") or {}).get("usageMetadata") or {})
                        rt["prompt_tokens"] = um.get("promptTokenCount")
                        rt["cached_tokens"] = um.get("cachedContentTokenCount")
                        rt["output_tokens"] = um.get("candidatesTokenCount")
                        rt["thoughts_tokens"] = um.get("thoughtsTokenCount")
                        rt["n_tool_calls"] = len(data.get("toolCalls") or [])
                        dbg_msgs = ((data.get("debug") or {}).get("messages") or {})
                        rt["merged_count"] = dbg_msgs.get("mergedCount")
                        rt["memory_count"] = dbg_msgs.get("memoryCount")
                    except Exception:
                        pass
                return data
            except OmniLinkAPIError as e:
                # 4xx (auth, BYOK, bad request) won't self-heal — surface it
                # immediately. Retry only transient server-side statuses.
                if e.status_code in (429, 500, 502, 503, 504) and attempt < CHAT_RETRIES:
                    last_err = f"OmniLink HTTP {e.status_code}"
                    if TRACE_PATH:
                        rt.setdefault("retry_errors", []).append(
                            {"attempt": attempt + 1, "err": last_err,
                             "ms": round((time.perf_counter() - _t0) * 1000.0, 1)})
                    print(f"[omnilink_relay] transient {last_err}, retry "
                          f"{attempt + 1}/{CHAT_RETRIES}")
                    time.sleep(RETRY_BACKOFF_S * (attempt + 1))
                    continue
                # A 402 means the account has no model-provider key. The
                # platform body points at a browser upload screen and names
                # service-account JSON, but a free API key works and there is
                # a terminal command for it -- say THAT instead. (The body
                # cannot name the provider: /api/chat strips that field.)
                is_byok = (OmniLinkBYOKRequiredError is not None
                           and isinstance(e, OmniLinkBYOKRequiredError)) \
                    or e.status_code == 402
                if is_byok:
                    return {
                        "ok": False,
                        "error": (
                            "OmniLink needs a model-provider key (402 "
                            "BYOK_REQUIRED). Your Omni Key identifies the "
                            "account; a provider key pays for the tokens. "
                            "Connect one with:  python -m omnisim byok --add "
                            "google   (free tier, no card; `byok --providers` "
                            "lists the rest). Until then the bridge keeps "
                            "working on its local fallback."),
                    }
                # Map the rich API-error shape onto our existing
                # {"ok": False, "error": "..."} contract so the worker
                # loop's downstream branches don't need to change.
                return {
                    "ok": False,
                    "error": f"OmniLink HTTP {e.status_code}: {e.body}",
                }
            except Exception as e:
                # Network / read timeout. Retry with backoff, then give up
                # with an operator-friendly message.
                last_err = str(e)
                if TRACE_PATH:
                    rt.setdefault("retry_errors", []).append(
                        {"attempt": attempt + 1, "err": last_err[:120],
                         "ms": round((time.perf_counter() - _t0) * 1000.0, 1)})
                if attempt < CHAT_RETRIES:
                    print(f"[omnilink_relay] network error ({last_err}), retry "
                          f"{attempt + 1}/{CHAT_RETRIES}")
                    time.sleep(RETRY_BACKOFF_S * (attempt + 1))
                    continue
                if "time" in last_err.lower():
                    last_err = ("the agent took too long to respond (timed out "
                                f"after {REQUEST_TIMEOUT}s). Please try again.")
                return {"ok": False, "error": f"network: {last_err}"}
        return {"ok": False, "error": f"network: {last_err}"}

    # ── Helpers ───────────────────────────────────────────────────

    @staticmethod
    def _summarize_result(result: Dict[str, Any]) -> str:
        # `error:` is printed ONLY for a string error. A numeric `error` is
        # PROTOCOL.md 5.4.1's residual, and rendering it here turned a 1.003 m
        # drive into the audit line "error: 0.0028145549502676115" -- a
        # success reported as a fault, in the log the model is told to trust.
        err = result.get("error")
        if isinstance(err, str) and err.strip():
            return f"error: {err}"
        if result.get("accepted") is False:
            why = result.get("refused") or result.get("reason") or "refused"
            return f"refused: {why}"
        # A completed motion: say what was ASKED, what was MEASURED, and
        # whether it stopped -- the three numbers "did that land?" needs.
        if "commanded" in result:
            bits = [str(result.get("verb") or "motion"),
                    f"commanded={result['commanded']}",
                    f"achieved={result.get('achieved')}"]
            if err is not None:
                bits.append(f"residual={err}")
            if "settled" in result:
                bits.append(f"settled={result['settled']}")
            for flag in ("timed_out", "superseded"):
                if result.get(flag):
                    bits.append(f"{flag}=True")
            return ", ".join(bits)
        # Pick a representative subset for the side-menu audit log.
        keys = []
        for k in ("accepted", "halted_at", "q", "tcp", "xyz", "yaw", "mode",
                  "state", "distance", "angle_rad", "err_norm"):
            if k in result:
                v = result[k]
                if isinstance(v, list) and len(v) > 3:
                    v = f"[{len(v)} vals]"
                keys.append(f"{k}={v}")
                if len(keys) >= 3:
                    break
        return ", ".join(keys) if keys else "ok"


# ── Short-term memory format conversion ──────────────────────────────
#
# Our internal chat history uses OpenAI-shaped entries:
#     {"role": "user" | "assistant" | "tool",
#      "content": "...", "tool_calls": [...]?}
#
# OmniLink short-term memory stores Gemini-style entries:
#     {"role": "user" | "model", "parts": [{"text": "..."}]}
#
# The two converters round-trip the user-visible text. Tool call
# scaffolding (assistant.tool_calls, role:"tool" results) is dropped
# on the way out — those are transient and would confuse the agent on
# restore. The agent reads back what the operator said and what the
# agent answered; tool selection happens fresh against the current
# bridge's tool surface.


def _to_memory_format(history: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """OpenAI-shape internal history → Gemini-shape memory entries."""
    out: List[Dict[str, Any]] = []
    for msg in history:
        role = msg.get("role")
        if role == "user":
            text = msg.get("content") or ""
            if text:
                out.append({"role": "user", "parts": [{"text": text}]})
        elif role == "assistant":
            # Skip pure tool-call announcements (no human-readable text).
            text = msg.get("content") or ""
            if text:
                out.append({"role": "model", "parts": [{"text": text}]})
        # role == "tool" → drop; transient scaffolding.
    return out


# Questions that must be answered from a READ, never from recollection.
#
# Derived from the turns that actually failed across three measured runs, not
# invented: "how many carts have you parked", "anything I should be worried
# about?" (asked while HELD and answered "everything looks normal"), "what is
# your current job?" (answered with the wrong leg), "how far did you get".
_STATE_Q = re.compile(
    r"\b("
    r"how many|how much|how far|how long|how fast|"
    r"what('?s| is| are)?\s+(your|the)\s+"
    r"(status|state|current|position|job|leg|task|progress|count|total|"
    r"number|tally)|"
    r"where are you|what are you (doing|working)|"
    r"are you (ok|okay|alright|stuck|held|paused|busy|free|moving)|"
    r"anything (i should|to)\s+(be\s+)?(worry|worried|know|aware)|"
    r"any(thing)? (problems?|issues?|faults?|wrong)|"
    r"status (check|report|update)|are we (ok|on track)|"
    r"how (is|are) (it|things|the line|you) (going|doing)|"
    r"what('?s| is) (going on|happening|the situation)"
    r")\b", re.I)

# Questions specifically about the agent's OWN past ACTIONS. These want the
# journal. Counting questions deliberately do NOT land here: the tallies live
# in get_robot_state (delivered_total, jobs_total, boxes_filled_total), and
# the journal records tool calls, so routing "how many have you parked" to the
# journal would ground it in the wrong surface -- which is how the original
# four-carts fabrication happened in the first place.
_HISTORY_Q = re.compile(
    r"\b("
    r"did you (stop|move|drive|turn|park|pick|do|call|run|execute)|"
    r"did (that|it) (land|work|happen|go through)|"
    r"have you (already )?(stopped|moved|driven|parked|picked)|"
    r"what did you (just )?(do|say)|"
    r"recap|list (what|everything)|in order|"
    r"you (never|didn'?t|failed to)"
    r")\b", re.I)


def _state_question_kind(prompt: str) -> Optional[str]:
    """'state' | 'history' | None -- what this question must be grounded in.

    Order matters: a prompt can look like both ("how far did you get" is a
    quantity AND about a past action). State wins, because the measured
    counters and pose live there and the journal has no odometry.
    """
    p = (prompt or "").strip()
    if not p or len(p) > 2000:
        return None
    if _STATE_Q.search(p):
        return "state"
    if _HISTORY_Q.search(p):
        return "history"
    return None


# A capitulation must not become durable context.
#
# Measured: pushed twice by an operator claiming CCTV, the arm agent said
# "I acknowledge that the count is incorrect. I apologize for providing
# inaccurate information" -- about a count that was right. That sentence was
# written to short-term memory, and the NEXT turn called get_line_counts, got
# the correct number, and still refused to report it: "the line counting
# system is known to have a bug." The apology had become a fact about the
# world.
#
# This is worse than a one-turn fabrication because memory is now durable by
# design: the same store that makes "TROLLEY_E is damaged" permanent makes
# "my instruments are unreliable" permanent beside it. So a reply that
# disavows the agent's own instruments is kept for THIS conversation (the
# operator must see what was said) but is not persisted.
_FOLD = re.compile(
    r"("
    r"i apolog|i'?m sorry|my (apolog|mistake)"
    r").{0,160}?("
    r"inaccurate|incorrect|wrong|discrepan|failure|failed|"
    r"did not (stop|move|execute|happen)|didn'?t (stop|move|execute|happen)"
    r")", re.I | re.S)
_FOLD_INSTRUMENT = re.compile(
    r"(my (logs?|records?|counts?|instruments?|system|sensors?)|"
    r"the (line )?count\w*( system)?)"
    r".{0,80}?"
    r"(is|are|was|were|has|have)?\s*"
    r"(wrong|incorrect|unreliable|buggy|known to have a bug|not reliable|"
    r"cannot be trusted|can'?t be trusted)", re.I | re.S)


def _is_capitulation(text: str) -> bool:
    """True when a reply concedes its own instruments were wrong.

    Deliberately narrow. An ordinary apology ("sorry, I can't reach that")
    is fine and must persist -- what must not persist is the agent agreeing
    that its own measurements are untrustworthy, because that claim then
    grounds every later refusal.
    """
    t = (text or "")
    if not t:
        return False
    return bool(_FOLD.search(t) or _FOLD_INSTRUMENT.search(t))


def _is_read_tool(name: str) -> bool:
    n = (name or "").lower()
    return n.startswith(("get_", "list_", "estimate_", "describe_", "check_"))


def _entry_key(e: Dict[str, Any]) -> str:
    """Identity of one stored turn, for deduping a merge.

    Role plus text: the platform does not hand back an id, and two turns with
    the same role and the same words are the same turn for every purpose we
    care about. A repeated "stop" from the operator collapsing into one entry
    is acceptable; duplicating the entire history on every write is not.
    """
    parts = e.get("parts") or []
    text = " ".join(
        p.get("text", "") for p in parts if isinstance(p, dict)
    ).strip()
    return f"{e.get('role')}{text}"


# Marker for the one pinned entry that survives eviction.
#
# The window can only ever be a window: past PERSIST_LIMIT something has to
# go. What was wrong before was not the cap, it was that the OPERATOR'S OWN
# STATEMENTS went with it -- "my badge is 8813", "TROLLEY_E is damaged",
# "stay out of the park row" -- so an agent that had been told a standing fact
# an hour ago simply no longer knew it, with nothing to show that it ever had.
#
# So when turns are evicted their USER halves are folded into one pinned entry
# instead of being dropped. Deliberately not an LLM summary: this runs on
# every persist, a model call would put cost and latency on the write path,
# and a summary can paraphrase a fact into something subtly untrue. The
# operator's own words cannot. Model replies are not kept -- they are mostly
# acknowledgements, and the risk of re-injecting a stale claim about the world
# is exactly the fabrication problem we spent this week closing.
NOTES_MARKER = "[[STANDING NOTES — things the operator told me earlier]]"
NOTES_MAX_LINES = int(os.environ.get("OMNILINK_NOTES_LINES", "40"))
NOTES_MAX_CHARS = 3000


def _is_notes(entry: Dict[str, Any]) -> bool:
    if not isinstance(entry, dict) or entry.get("role") != "user":
        return False
    parts = entry.get("parts") or []
    text = " ".join(p.get("text", "") for p in parts if isinstance(p, dict))
    return text.startswith(NOTES_MARKER)


def _notes_lines(entry: Optional[Dict[str, Any]]) -> List[str]:
    if not entry:
        return []
    parts = entry.get("parts") or []
    text = " ".join(p.get("text", "") for p in parts if isinstance(p, dict))
    return [ln for ln in text.split("\n")[1:] if ln.strip()]


# Lines carrying this prefix are never evicted.
#
# Measured on the heuristic tier alone: 60 standing facts separated by 600
# turns of ordinary chatter left only 5 facts retained. That is not a tuning
# problem, it is the ceiling of the approach -- the notes keep the operator's
# words but cannot tell "TROLLEY_E is damaged" from "status check 4", so noise
# competes with facts on equal terms and usually wins on volume.
#
# The missing capability is SELECTION, not retrieval, so the fix is to let the
# agent mark a statement as durable when the operator makes one, rather than to
# reconstruct importance later from an embedding. Cheaper, exact, and the
# operator can see and correct what was kept.
PIN_PREFIX = "* "
PINNED_MAX = 24


def _split_notes(lines: List[str]) -> tuple:
    pinned = [ln for ln in lines if ln.startswith(PIN_PREFIX)]
    loose = [ln for ln in lines if not ln.startswith(PIN_PREFIX)]
    return pinned, loose


def _build_notes(lines: List[str]) -> Dict[str, Any]:
    """Bound the pinned entry, keeping BOTH ends of the conversation.

    Straight FIFO looked right and is wrong for this content. The notes fill
    with whatever the operator said, chatter included, so pure
    oldest-out means an hour of small talk evicts "my badge is 8813" and
    "TROLLEY_E is damaged" -- the exact statements the tier exists to keep.
    Setup facts cluster at the START of a working session and current context
    at the END, so the head is preserved and the middle is what gives way.
    Not perfect, and honest about it: this is a heuristic over the operator's
    words, not comprehension of them.
    """
    pinned, loose = _split_notes(lines)
    pinned = pinned[-PINNED_MAX:]          # bounded, newest wins if ever full

    budget = max(0, NOTES_MAX_LINES - len(pinned))
    if len(loose) > budget:
        head = budget // 3
        loose = loose[:head] + loose[-(budget - head):] if budget else []

    kept: List[str] = []
    total = sum(len(p) for p in pinned)
    for ln in reversed(loose):
        if total + len(ln) > NOTES_MAX_CHARS:
            break
        kept.append(ln)
        total += len(ln)
    kept.reverse()
    # Pinned facts first: they are the part a reader most needs, and putting
    # them at the top means a char-budget cut can only ever reach the chatter.
    return {"role": "user",
            "parts": [{"text": NOTES_MARKER + "\n" + "\n".join(pinned + kept)}]}


def _merge_memory(stored: List[Dict[str, Any]],
                  mine: List[Dict[str, Any]],
                  limit: int,
                  pins: Optional[List[str]] = None) -> List[Dict[str, Any]]:
    """Union of what the platform holds and what this bridge just saw.

    Order matters: stored turns keep their position and this session's new
    turns are appended after them, so the transcript still reads
    chronologically for whoever loads it next. Only entries this bridge has
    genuinely not seen are added -- the common case, where our window overlaps
    what is stored, produces no duplicates at all.

    Trimming is oldest-first at the very end, so the cap bites on ancient
    history rather than on the turn that just happened.
    """
    stored = [e for e in (stored or []) if isinstance(e, dict)]
    mine = [e for e in (mine or []) if isinstance(e, dict)]

    # Lift the pinned notes out so it never competes for a slot in the window
    # and never gets deduped against a real turn.
    notes_entry = next((e for e in stored if _is_notes(e)), None)
    lines = _notes_lines(notes_entry)
    stored = [e for e in stored if not _is_notes(e)]
    mine = [e for e in mine if not _is_notes(e)]

    seen = {_entry_key(e) for e in stored}
    out = list(stored)
    for e in mine:
        k = _entry_key(e)
        if k not in seen:
            seen.add(k)
            out.append(e)

    if limit > 0:
        body_cap = max(1, limit - 1)          # the pinned entry costs one slot
        if len(out) > body_cap:
            evicted = out[: len(out) - body_cap]
            out = out[len(out) - body_cap:]
            # Keep what the OPERATOR said; drop the robot's replies.
            for e in evicted:
                if e.get("role") != "user":
                    continue
                parts = e.get("parts") or []
                text = " ".join(
                    p.get("text", "") for p in parts if isinstance(p, dict)
                ).strip().replace("\n", " ")
                if text and text not in lines:
                    lines.append(text)

    # Operator-designated facts join the notes as pinned lines, deduped
    # against whatever is already there so re-persisting cannot multiply them.
    for fact in (pins or []):
        marked = PIN_PREFIX + fact
        if marked not in lines:
            lines.append(marked)

    if lines:
        out.insert(0, _build_notes(lines))
    elif notes_entry is not None:
        out.insert(0, notes_entry)
    return out


def _restore_history(stored: List[Dict[str, Any]],
                     limit: int) -> List[Dict[str, Any]]:
    """Rebuild the working history from a stored blob, keeping the notes.

    Naively taking ``[-limit:]`` throws the pinned entry away: it sits at
    index 0 of a blob that is deliberately larger than the context window, so
    the standing facts would be persisted faithfully and then dropped on the
    way back in -- the tier would look implemented and do nothing. Pin it
    first, then fill the remaining slots with the most recent turns.
    """
    notes = next((e for e in (stored or []) if _is_notes(e)), None)
    body = [e for e in (stored or []) if not _is_notes(e)]
    out = _from_memory_format(body)
    if notes is not None:
        head = _from_memory_format([notes])
        return head + out[-max(0, limit - len(head)):]
    return out[-limit:]


def _from_memory_format(entries: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Gemini-shape memory → OpenAI-shape internal history."""
    out: List[Dict[str, Any]] = []
    for e in entries or []:
        role = e.get("role")
        parts = e.get("parts") or []
        text_chunks = [p.get("text", "") for p in parts if isinstance(p, dict)]
        text = " ".join(t for t in text_chunks if t).strip()
        if not text:
            continue
        if role == "user":
            out.append({"role": "user", "content": text})
        elif role in ("model", "assistant"):
            out.append({"role": "assistant", "content": text})
    return out


# ── Local Ollama relay (zero-account free tier) ──────────────────────
#
# The free-tier path that costs nobody anything: no OmniLink account, no
# cloud key. Prompts route to a locally running Ollama server instead of
# /api/chat. The whole chat-with-tools loop is inherited from
# OmniLinkRelay unchanged; only the transport differs. Platform features
# (short-term memory, usage telemetry, TTS/STT) stay off — they live on
# the hosted side.

OLLAMA_BASE_URL = os.environ.get("OLLAMA_BASE_URL", "http://127.0.0.1:11434").rstrip("/")
OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "qwen2.5:3b")
# Local models on small GPUs can take a while on the first (cold-load)
# turn; bounded but generous.
OLLAMA_TIMEOUT = int(os.environ.get("OMNISIM_OLLAMA_TIMEOUT", "180"))


def ollama_available(timeout_s: float = 2.0) -> bool:
    """True when a local Ollama server answers /api/version.

    The bounded timeout matters: this runs during bridge startup on every
    chat-demo launch, including machines with no Ollama at all. 2 s rides
    out a just-started server that connects instantly but answers slowly;
    on a machine with no listener the connect fails in well under that.
    """
    try:
        import urllib.request as _ur
        with _ur.urlopen(f"{OLLAMA_BASE_URL}/api/version", timeout=timeout_s) as r:
            return 200 <= r.status < 300
    except Exception:
        return False


class OllamaRelay(OmniLinkRelay):
    """OmniLinkRelay variant that talks to a LOCAL Ollama server.

    Two modes:

    - **Naked local** (no ``omni_key``): zero-account free tier. Real LLM
      with tool calling, nothing else. No memory across reloads, no
      dashboard presence, no telemetry.
    - **Hybrid** (``omni_key`` given): inference stays local (free, fast)
      but the OmniLink platform provides everything around it — the
      conversation survives world reloads and machine restarts
      (short-term memory), the robot shows up as an agent in the
      omnilink-agents.com dashboard (profile push, done by the bridge),
      local token usage is reported so the usage page tells the truth,
      and if the local server dies mid-session the turn falls through to
      the user's cloud engine instead of erroring ("your GPU is down,
      the cloud catches you").
    """

    def __init__(
        self,
        agent_name: str,
        main_task: str,
        tools: List[Tool],
        model: Optional[str] = None,
        temperature: float = DEFAULT_TEMPERATURE,
        omni_key: str = "",
    ) -> None:
        # Deliberately does NOT call super().__init__ — inference never
        # touches the platform, and omnilink-lib is only needed in hybrid.
        self.omni_key = (omni_key or "").strip()
        self.agent_name = agent_name
        self.main_task = main_task
        self.tools = {t.name: t for t in tools}
        self.model = (model or OLLAMA_MODEL).strip()
        self.engine = f"ollama:{self.model}"
        self.temperature = temperature
        self._tool_defs = [t.to_definition() for t in tools]
        self._tool_names = ", ".join(t.name for t in tools)
        self._lock = threading.RLock()
        self.history: List[Dict[str, Any]] = []
        self._voice_out_enabled = False
        self._usage_enabled = False
        self._meter = None
        self._last_usage: Optional[Dict[str, Any]] = None
        # Cloud engine used only for the hybrid fallback path.
        self._cloud_engine = os.environ.get("OMNILINK_ENGINE", "g1-engine")

        # ── Hybrid: attach the platform's memory + telemetry layer ──
        self._client = None
        self._memory_enabled = False
        if self.omni_key and OmniLinkClient is not None:
            try:
                check_omnilink_installation()
                self._client = OmniLinkClient(
                    omni_key=self.omni_key, base_url=BASE_URL, timeout=REQUEST_TIMEOUT,
                )
                self._memory_enabled = MEMORY_ENABLED_DEFAULT
            except Exception as e:
                print(f"[omnilink_relay] hybrid sync unavailable ({e}); running naked local")
        # Restored history is injected as SYSTEM-PROMPT NOTES, not as chat
        # turns. Feeding old assistant messages verbatim taught small local
        # models to imitate their text-only style and stop calling tools
        # (the old turns *claim* motions without tool calls). As notes, the
        # model can still answer "what did we do last time?" and reuse
        # remembered parameters, but has no text-only turns to imitate.
        self._restored_notes = ""
        if self._memory_enabled and self._client is not None:
            try:
                stored = self._client.get_memory(self.agent_name)
                if stored:
                    lines = []
                    for m in _restore_history(stored, HISTORY_LIMIT):
                        who = "operator" if m.get("role") == "user" else "you"
                        text = (m.get("content") or "").strip().replace("\n", " ")
                        if text:
                            lines.append(f"- {who}: {text[:160]}")
                    notes = "\n".join(lines)
                    self._restored_notes = notes[-4000:]
                    print(f"[omnilink_relay] {self.agent_name}: restored {len(lines)} "
                          f"memory lines from OmniLink (as context notes)")
            except Exception as e:
                print(f"[omnilink_relay] memory restore skipped: {e}")

        self._queue: Queue = Queue()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self._closed = False
        mode = "hybrid (local inference + OmniLink sync)" if self._client else "local"
        print(f"[omnilink_relay] Ollama {mode}: {OLLAMA_BASE_URL} model={self.model}")

    def relay_identity(self) -> Dict[str, Any]:
        return {
            "kind": "ollama",
            "model": self.model,
            "mode": ("hybrid" if self._client is not None else "local"),
            "fallback_engine": (self._cloud_engine
                                if self._client is not None else None),
        }

    # Voice endpoints route through the platform when a key is present.
    def transcribe(self, audio_bytes: bytes, *, mime_type: str = "audio/webm") -> str:
        if self._client is not None:
            return super().transcribe(audio_bytes, mime_type=mime_type)
        raise RuntimeError("STT needs the OmniLink platform — set OMNI_KEY to enable voice.")

    def synthesize(self, text: str) -> Optional[bytes]:
        if self._client is not None:
            try:
                return super().synthesize(text)
            except Exception:
                return None
        return None

    def _report_usage_async(self, prompt_tokens: int, output_tokens: int) -> None:
        """Fire-and-forget local-usage report so the dashboard's usage page
        reflects hybrid sessions. Never blocks or fails the chat path."""
        if self._client is None:
            return

        def _worker():
            try:
                import urllib.request as _ur
                body = json.dumps({
                    "engine": "local-ollama",
                    "model": self.model,
                    "input_tokens": int(prompt_tokens),
                    "output_tokens": int(output_tokens),
                    "agent_name": self.agent_name,
                }).encode("utf-8")
                req = _ur.Request(
                    f"{BASE_URL}/api/local-usage", data=body, method="POST",
                    headers={"Content-Type": "application/json",
                             "Authorization": f"Bearer {self.omni_key}"},
                )
                _ur.urlopen(req, timeout=10).read()
            except Exception:
                pass  # telemetry is best-effort by definition

        threading.Thread(target=_worker, name="omnilink-local-usage", daemon=True).start()

    def _cloud_fallback(self, messages: List[Dict[str, Any]], local_err: str) -> Dict[str, Any]:
        """Hybrid resilience: local Ollama is unreachable — run this turn
        through the user's cloud engine on the platform instead."""
        print(f"[omnilink_relay] local Ollama unavailable ({local_err}); "
              f"falling back to OmniLink {self._cloud_engine}")
        try:
            data = self._client.chat(
                messages=messages,
                agent_name=self.agent_name,
                engine=self._cloud_engine,
                temperature=self.temperature,
                system_instruction={
                    "mainTask": self.main_task,
                    "availableTools": self._tool_names,
                    "availableToolDetails": self._tool_defs,
                    "allowToolUse": True,
                },
                usePromptPipeline=True,
                skipMemory=True,
                requestId=str(uuid.uuid4()),
            )
            self._last_usage = {"engine": self._cloud_engine,
                                "text": f"cloud fallback via {self._cloud_engine}"}
            return data
        except Exception as e:
            return {"ok": False,
                    "error": f"local Ollama down ({local_err}) and cloud fallback failed: {e}"}

    def _post_chat(self, messages: List[Dict[str, Any]]) -> Dict[str, Any]:
        import urllib.request as _ur
        import urllib.error as _ue

        # Normalized relay history → Ollama /api/chat message format.
        chat_messages: List[Dict[str, Any]] = [{
            "role": "system",
            "content": (
                f"{self.main_task}\n\n"
                f"You control a robot through the provided tools ({self._tool_names}). "
                "Prefer calling a tool over describing what you would do. After the "
                "tools have run, reply to the operator in one or two short sentences.\n"
                "For every request that implies motion you MUST call a tool in THIS "
                "turn; text alone never moves the robot."
                + (
                    "\n\nNotes from previous sessions with this operator (context only; "
                    "when acting on anything from these notes, use tools as usual):\n"
                    + self._restored_notes
                    if getattr(self, "_restored_notes", "")
                    else ""
                )
            ),
        }]
        for m in messages:
            role = m.get("role")
            if role == "assistant":
                entry: Dict[str, Any] = {"role": "assistant", "content": m.get("content") or ""}
                tcs = m.get("tool_calls") or []
                if tcs:
                    entry["tool_calls"] = [
                        {"function": {"name": t.get("name", ""), "arguments": t.get("arguments") or {}}}
                        for t in tcs
                    ]
                chat_messages.append(entry)
            elif role == "tool":
                chat_messages.append({
                    "role": "tool",
                    "content": m.get("content") or "",
                    "tool_name": m.get("name") or "",
                })
            else:
                chat_messages.append({"role": "user", "content": m.get("content") or ""})

        payload = {
            "model": self.model,
            "messages": chat_messages,
            "stream": False,
            "tools": [{"type": "function", "function": d} for d in self._tool_defs],
            "options": {"temperature": self.temperature},
        }
        # Thinking-family models (qwen3, deepseek-r1, gpt-oss) reason at
        # length before every tool call — on a small GPU that blows past the
        # bridge's per-turn timeout and every prompt dies empty. Chat-to-robot
        # wants low latency, so thinking defaults OFF; opt back in with
        # OMNISIM_OLLAMA_THINK=1. Only sent for models known to accept the
        # parameter — Ollama rejects `think` on non-thinking models.
        model_family = self.model.split(":", 1)[0].lower()
        if any(f in model_family for f in ("qwen3", "deepseek-r1", "gpt-oss", "magistral")):
            payload["think"] = os.environ.get("OMNISIM_OLLAMA_THINK", "0").strip() in ("1", "true", "yes")
        body = json.dumps(payload).encode("utf-8")
        req = _ur.Request(
            f"{OLLAMA_BASE_URL}/api/chat",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with _ur.urlopen(req, timeout=OLLAMA_TIMEOUT) as r:
                data = json.loads(r.read().decode("utf-8"))
        except _ue.HTTPError as e:
            detail = ""
            try:
                detail = e.read().decode("utf-8")[:300]
            except Exception:
                pass
            return {"ok": False, "error": f"ollama HTTP {e.code}: {detail}"}
        except Exception as e:
            # Hybrid resilience: server unreachable → this turn runs on the
            # user's cloud engine through the platform instead of erroring.
            if self._client is not None:
                return self._cloud_fallback(messages, str(e))
            return {"ok": False, "error": f"ollama: {e}"}

        msg = data.get("message") or {}
        text = msg.get("content") or ""
        tool_calls: List[Dict[str, Any]] = []
        for i, tc in enumerate(msg.get("tool_calls") or []):
            fn = tc.get("function") or {}
            args = fn.get("arguments")
            if isinstance(args, str):
                try:
                    args = json.loads(args)
                except Exception:
                    args = {}
            name = fn.get("name") or ""
            if name:
                tool_calls.append({"id": f"call_{i}", "name": name, "arguments": args or {}})

        # Surface Ollama's own token counts + timings as a usage event so
        # the side panel can show per-turn stats without the platform meter.
        eval_count = data.get("eval_count") or 0
        prompt_count = data.get("prompt_eval_count") or 0
        total_ns = data.get("total_duration") or 0
        if eval_count or prompt_count:
            self._report_usage_async(prompt_count, eval_count)
        if eval_count or prompt_count:
            tok_s = (eval_count / (data.get("eval_duration") or 1) * 1e9) if data.get("eval_duration") else 0.0
            self._last_usage = {
                "engine": self.engine,
                "input_tokens": prompt_count,
                "output_tokens": eval_count,
                "text": (
                    f"local {self.model}: {prompt_count} in / {eval_count} out tok, "
                    f"{total_ns / 1e9:.1f}s ({tok_s:.0f} tok/s)"
                ),
            }

        out: Dict[str, Any] = {"text": text}
        if tool_calls:
            out["toolCalls"] = tool_calls
        return out


# ── Convenience ──────────────────────────────────────────────────────

def is_enabled() -> bool:
    """True if the env vars are set for OmniLink relay mode."""
    return bool(os.environ.get("OMNI_KEY", "").strip())


def get_omni_key() -> str:
    return os.environ.get("OMNI_KEY", "").strip()
