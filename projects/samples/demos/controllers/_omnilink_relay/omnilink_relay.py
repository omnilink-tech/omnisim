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
import sys
import threading
import time
import traceback
import uuid
from queue import Queue, Empty
from typing import Any, Callable, Dict, List, Optional

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
    from omnilink.usage_meter import UsageMeter  # type: ignore[import-not-found]
except Exception:
    _omnilink_pkg = None
    OmniLinkClient = None  # type: ignore[assignment]
    OmniLinkAPIError = Exception  # type: ignore[assignment,misc]
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
# Generous defaults so multi-step planning ("go forward 1m, then turn left 90 deg")
# doesn't run out of turns mid-chain or lose its earlier context. The relay still
# clamps via single-flight semantics, so these only bound the worst case.
MAX_TOOL_TURNS = int(os.environ.get("OMNILINK_MAX_TURNS", "16"))
HISTORY_LIMIT = int(os.environ.get("OMNILINK_HISTORY_LIMIT", "40"))
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


# ── Tool spec dataclass ──────────────────────────────────────────────

class Tool:
    """One tool the agent can call.

    `dispatch(args: dict) -> dict` is the bridge-side action. The dict it
    returns becomes the tool result the LLM sees on the next turn, so it
    should be small and informative (e.g. `{"q": [...], "err": 0.001}`,
    not raw motor objects).
    """

    def __init__(
        self,
        name: str,
        description: str,
        parameters: Dict[str, Any],
        dispatch: Callable[[Dict[str, Any]], Dict[str, Any]],
    ) -> None:
        self.name = name
        self.description = description
        self.parameters = parameters
        self.dispatch = dispatch

    def to_definition(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "parameters": self.parameters,
        }


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
    ) -> None:
        check_omnilink_installation()
        _check_pypi_for_newer_async()
        if not omni_key:
            raise ValueError("OMNI_KEY is empty")
        self.omni_key = omni_key
        self.agent_name = agent_name
        self.main_task = main_task
        self.tools = {t.name: t for t in tools}
        self.engine = engine
        self.temperature = temperature
        self._tool_defs = [t.to_definition() for t in tools]
        self._tool_names = ", ".join(t.name for t in tools)
        # The OmniLinkClient instance is the single point of contact with
        # the OmniLink platform. Sim and real-robot integrations both go
        # through this same object — that's the no-sim-to-real-gap claim
        # in concrete code form. Swap the bridge's local dispatch handlers
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
                    self.history = _from_memory_format(stored)[-HISTORY_LIMIT:]
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
        self._queue: Queue = Queue()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self._closed = False

    # ── Public API ────────────────────────────────────────────────

    def dispatch_async(
        self,
        text: str,
        on_event: Callable[[str, Dict[str, Any]], None],
    ) -> None:
        """Enqueue a prompt. on_event is called from the worker thread."""
        if self._closed:
            on_event("error", {"text": "relay is closed"})
            return
        self._queue.put((text, on_event))

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

        self.dispatch_async(text, _cb)
        if not done.wait(timeout=timeout_s):
            return {"response": "", "actions": actions, "error": "timeout"}
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
        self._queue.put((None, None))

    # ── Worker loop ───────────────────────────────────────────────

    def _run(self) -> None:
        while True:
            try:
                item = self._queue.get(timeout=1.0)
            except Empty:
                continue
            if item is None or item[0] is None:
                return
            text, on_event = item
            try:
                self._dispatch_one(text, on_event)
            except Exception as e:
                err = f"{type(e).__name__}: {e}"
                traceback.print_exc()
                try:
                    on_event("error", {"text": err})
                except Exception:
                    pass

    # ── One prompt -> /api/chat loop ──────────────────────────────

    def _dispatch_one(self, text: str, on_event: Callable[[str, Dict[str, Any]], None]) -> None:
        on_event("status", {"state": "thinking"})

        with self._lock:
            self.history.append({"role": "user", "content": text})
            messages = list(self.history[-HISTORY_LIMIT:])

        last_text = ""
        for turn in range(MAX_TOOL_TURNS):
            data = self._post_chat(messages)
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
                else:
                    try:
                        result = tool.dispatch(args)
                    except Exception as e:
                        result = {"error": f"dispatch failed: {e}"}
                    ok = "error" not in result
                    on_event("tool", {
                        "name": tool_name,
                        "status": "ok" if ok else "err",
                        "summary": self._summarize_result(result),
                    })

                tool_msg = {
                    "role": "tool",
                    "tool_call_id": tc.get("id") or tool_name,
                    "name": tool_name,
                    "content": json.dumps(result, default=str),
                }
                messages.append(tool_msg)
                with self._lock:
                    self.history.append(tool_msg)

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
        # Usage delta for this turn. Cheap GET to /api/omni-key-usage;
        # failures don't affect the chat path. The platform's rollup is
        # the authoritative source -- our local tool-call count would
        # miss memory writes, profile pushes, retries, etc.
        if self._meter is not None:
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
        # Persist the updated history to OmniLink short-term memory so
        # the next session (after a world reload, a Webots restart, or
        # even a different OmniSim instance pointed at the same key)
        # can pick up the conversation. Fire-and-forget on its own
        # thread — the dispatcher's worker shouldn't block on memory
        # writes, and a failed write is not a fatal error.
        if self._memory_enabled:
            self._persist_memory_async()
        on_event("status", {"state": "idle"})

    def _synthesize_async(
        self,
        text: str,
        on_event: Callable[[str, Dict[str, Any]], None],
    ) -> None:
        """Fire-and-forget TTS. Posts the resulting base64-encoded MP3
        as an `audio_out` event so the chat panel can play it."""
        def _worker():
            try:
                audio = self.synthesize(text)
                if not audio:
                    return
                import base64 as _b64
                b64 = _b64.b64encode(audio).decode("ascii")
                on_event("audio_out", {"audio_b64": b64, "mime_type": "audio/mpeg"})
            except Exception as e:
                print(f"[omnilink_relay] tts async failed: {e}")

        threading.Thread(target=_worker, name="omnilink-tts", daemon=True).start()

    def _persist_memory_async(self) -> None:
        """Fire-and-forget set_memory call. Bounded so a hung OmniLink
        write doesn't pile up threads if the user spams prompts."""
        with self._lock:
            snapshot = list(self.history[-HISTORY_LIMIT:])

        def _worker():
            try:
                conv = _to_memory_format(snapshot)
                self._client.set_memory(self.agent_name, conv)
            except Exception as e:
                # Memory is best-effort. Log and move on.
                print(f"[omnilink_relay] memory persist failed: {e}")

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
        for attempt in range(1 + max(0, CHAT_RETRIES)):
            try:
                return self._client.chat(
                    messages=messages,
                    agent_name=self.agent_name,
                    engine=self.engine,
                    temperature=self.temperature,
                    system_instruction={
                        "mainTask": self.main_task,
                        "availableTools": self._tool_names,
                        "availableToolDetails": self._tool_defs,
                        "allowToolUse": True,
                    },
                    # Extra fields go straight into the request body. The
                    # OmniSim chat-with-tools loop is stateless on the
                    # server (we manage history client-side), and we want a
                    # fresh request id per turn so cache attribution works.
                    usePromptPipeline=True,
                    skipMemory=True,
                    requestId=str(uuid.uuid4()),
                )
            except OmniLinkAPIError as e:
                # 4xx (auth, BYOK, bad request) won't self-heal — surface it
                # immediately. Retry only transient server-side statuses.
                if e.status_code in (429, 500, 502, 503, 504) and attempt < CHAT_RETRIES:
                    last_err = f"OmniLink HTTP {e.status_code}"
                    print(f"[omnilink_relay] transient {last_err}, retry "
                          f"{attempt + 1}/{CHAT_RETRIES}")
                    time.sleep(RETRY_BACKOFF_S * (attempt + 1))
                    continue
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
        if "error" in result:
            return f"error: {result['error']}"
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


# ── Convenience ──────────────────────────────────────────────────────

def is_enabled() -> bool:
    """True if the env vars are set for OmniLink relay mode."""
    return bool(os.environ.get("OMNI_KEY", "").strip())


def get_omni_key() -> str:
    return os.environ.get("OMNI_KEY", "").strip()
