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

OmniLink chat requires an OmniKey. When no relay is configured, prompts
return a connection error. Direct simulator controls remain independent.

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
MIN_OMNILINK_VERSION = "0.6.3"


# The safety veto, applied to a MODEL's tool call: `vet_toolcall(tool,
# args, utterance, surface=...)` returns None to allow and a reason string
# when the frame must not run.
#
# ⚠️ IT LIVES IN bridge_base, AND THERE IS ONLY ONE OF IT. This module used
# to carry its own copy of the fail-closed wrapper, complete with its own
# `_PHYSICAL` hint list -- one of six such copies, and one that omitted
# `grasp`, `release`, `joint`, `tcp` and `trolley`, so on an import failure
# the arm's gripper and every joint command would have been waved through
# by the very branch whose job is to refuse them. All six were deleted on
# 2026-09-22.
#
# It still FAILS CLOSED on the physical tools, which is the whole reason a
# wrapper exists at all: a safety check that disappears when its import
# fails is not a safety check, and this module is imported by a bundled
# interpreter that has surprised us before -- omnilink-lib was invisible to
# it for a whole session, the failure was swallowed, and the fallback
# reported the wrong cause.
from .bridge_base import vet_toolcall as _gate_reject_toolcall


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
#
# ⚠️ THE PIN IS GOOGLE-SPECIFIC, SO IT ONLY APPLIES TO THE GOOGLE ENGINE.
# Everything above is reasoning about Gemini tiers, and `gemini-3.5-flash` is a
# Google model id. Until 2026-09-22 it was sent to WHATEVER engine was
# selected, so picking another provider asked that provider for a Gemini model
# and got a 404. Measured live that day on an account with google AND xai
# connected: `OMNILINK_ENGINE=g3-engine` returned
#   G3-engine request failed. {"code":"not-found",
#    "error":"The model gemini-3.5-flash does not exist ..."}
# which made the xai credential unusable through OmniSim and left the account
# with exactly one reachable engine -- the one whose Google grant had lapsed.
#
# So the pin now follows the ENGINE. An explicit OMNILINK_MODEL still wins for
# any engine. For an engine with no entry here we send NO model and let the
# platform's adaptive selector pick one it actually serves, which is strictly
# better than naming another vendor's model: the worst case is the weakest tier
# (the thing the pin exists to avoid), and the alternative is a hard 404.
# Add an entry here when a tier is measured on that provider, not before.
MODEL_PIN_BY_ENGINE = {
    "g1-engine": "gemini-3.5-flash",   # Google; the reasoning above
}
_OMNILINK_MODEL_ENV = os.environ.get("OMNILINK_MODEL")
DEFAULT_MODEL = (
    _OMNILINK_MODEL_ENV if _OMNILINK_MODEL_ENV is not None
    else MODEL_PIN_BY_ENGINE.get(DEFAULT_ENGINE, "")
).strip()


def model_for_engine(engine: str) -> str:
    """The model to pin for `engine`, honouring an explicit OMNILINK_MODEL.

    A relay may be constructed with an engine other than DEFAULT_ENGINE, so the
    module-level DEFAULT_MODEL is not enough on its own -- it is computed from
    the environment's engine, not this instance's.
    """
    if _OMNILINK_MODEL_ENV is not None:
        return _OMNILINK_MODEL_ENV.strip()
    return MODEL_PIN_BY_ENGINE.get((engine or "").strip(), "")
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



def _platform_message(err: Any) -> str:
    """The platform's own human message from an API error, or ''.

    The server nests it as ``{"error": {"code", "message"}}``; older bodies put
    ``message`` at the top level. Never raises -- this runs on an error path.
    """
    try:
        body = getattr(err, "body", None)
        if isinstance(body, dict):
            inner = body.get("error")
            if isinstance(inner, dict) and inner.get("message"):
                return str(inner["message"])[:300]
            if body.get("message"):
                return str(body["message"])[:300]
    except Exception:                                   # pragma: no cover
        pass
    return ""

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


# ── The event ring, read defensively ─────────────────────────────────
#
# `events.py` belongs to the bridge side of this package and may be absent
# from an older checkout or a partial install. IMPORT FAILURE MUST NOT STOP
# A BRIDGE FROM BOOTING -- the relay is the process that keeps a robot
# answering, and a robot that will not start because a telemetry module is
# missing is a worse bug than the one this wiring fixes. Same reasoning as
# the `gate.register_tools` wrapper below: the failure mode of "no events"
# is the status quo ante, which is survivable.
try:                                              # pragma: no cover - trivial
    from .events import WAKE_POLICY as _WAKE_POLICY  # type: ignore
    from .events import may_wake as _may_wake        # type: ignore
except Exception:
    _WAKE_POLICY = None                           # type: ignore[assignment]
    _may_wake = None                              # type: ignore[assignment]

# Which event types may cost a model turn (decision L4 of the v9 loop plan):
# PHYSICAL OR SAFETY ONLY. Consulted when `events.WAKE_POLICY` has no entry
# for a type -- and for an unknown type the answer is NO, because a wake is
# roughly 5.6k prompt tokens and an unbounded wake on, say, a contact stream
# would spend the operator's budget on the floor.
_DEFAULT_WAKE_TYPES = frozenset({
    "joint.limit_hit",
    "motion.timed_out",
    "gate.refused",
    "break.hit",
    "contact.began",
})
_DEFAULT_WAKE_PREFIXES = ("fault.", "damage.")

# The two bounds of the token bucket. Module constants rather than env vars
# ON PURPOSE: §8 of the loop plan says that if idle-scene wakes come out
# above 6/hour the default POLICY is narrowed, not the limit raised, so
# there is deliberately no field knob for raising them. Tests patch these.
WAKE_MIN_INTERVAL_S = 10.0
WAKE_MAX_PER_SESSION = 30
# How often the presence thread looks at the ring. It is an in-process read
# of a bounded deque, so this is cheap; it bounds "event -> wake dispatched".
WAKE_POLL_S = 0.5
# A wake message carries a compacted `detail`; this bounds it so one noisy
# event cannot write a kilobyte into the conversation history.
WAKE_DETAIL_CHARS = 300

# How often the background beat offers to push the action journal to the
# platform. It is the PRESENCE cadence deliberately: a memory write is an
# HTTP round trip, and writing one per `record()` would put the platform on
# the robot's critical path -- the same mistake the usage meter made when it
# sat inline and cost a flat ~1.9 s on every single turn. A beat that finds
# nothing new costs one integer comparison (ActionJournal.revision).
JOURNAL_SYNC_S = 30.0
# What a clean shutdown will wait for the final flush. A daemon write thread
# does not survive the interpreter exiting behind it, so the close flush is
# synchronous -- and therefore bounded, because a dead link must not hold a
# world shutdown open.
JOURNAL_CLOSE_FLUSH_S = 5.0

# Floor-ish bodies, for the one wake rule that needs to look at the detail.
# L4 wakes on `contact.began` only "with a non-floor body"; the detector is
# specified to filter, and this is the belt to its braces. Only a NAMED
# floor suppresses -- a detail with no body name still wakes, because the
# detector's contract is what decides, not this guess.
_FLOOR_WORDS = ("floor", "ground", "plane", "terrain", "arena", "carpet")


def _wake_allowed(event: Any) -> bool:
    """May this event cost a model turn?

    `events.may_wake` decides when it is importable -- it owns the policy
    table AND the two per-event refinements L4 asks for (a floor contact is
    the world working normally; a refusal the agent caused on its own turn
    is already in front of it, and waking on it would be a loop with a
    bill). The table below is the fallback for a checkout where `events.py`
    is absent, and it answers NO for a type it does not recognise, because
    a wake is roughly 5.6k prompt tokens.
    """
    if not isinstance(event, dict):
        return False
    if _may_wake is not None:
        try:
            return bool(_may_wake(event))
        except Exception:                     # pragma: no cover - defensive
            pass
    etype = str(event.get("type") or "")
    if not etype:
        return False
    if isinstance(_WAKE_POLICY, dict) and etype in _WAKE_POLICY:
        if not _WAKE_POLICY[etype]:
            return False
    elif not (etype in _DEFAULT_WAKE_TYPES
              or etype.startswith(_DEFAULT_WAKE_PREFIXES)):
        return False
    if etype == "contact.began" and _names_a_floor(event.get("detail")):
        return False
    return True


def _names_a_floor(detail: Any) -> bool:
    if not isinstance(detail, dict):
        return False
    for key in ("body", "other", "with", "b", "name", "second", "against"):
        value = detail.get(key)
        if isinstance(value, str) and any(w in value.lower() for w in _FLOOR_WORDS):
            return True
    return False


def _compact_detail(detail: Any, limit: int = WAKE_DETAIL_CHARS) -> str:
    try:
        if isinstance(detail, str):
            text = detail
        elif detail is None:
            text = ""
        else:
            text = json.dumps(detail, default=str, sort_keys=True)
    except Exception:
        text = str(detail)
    text = " ".join(text.split())
    return text if len(text) <= limit else text[: limit - 3] + "..."


def _safe_get(obj: Any, name: str, default: Any = None) -> Any:
    """`getattr` that survives a PROPERTY which raises.

    Every telemetry name on a bridge is documented as read with
    `getattr(bridge, name, default)` -- but `getattr`'s default only covers
    a MISSING attribute, and several of these are properties that compute
    something (`held` walks the hold lease, `events` builds a ring lazily).
    A bridge mid-teardown whose property raises would have taken the
    heartbeat thread down with it, which is the one thread that reports the
    robot is alive.
    """
    try:
        return getattr(obj, name, default)
    except Exception:
        return default


def _round_or_none(value: Any, places: int = 3) -> Optional[float]:
    """A presence number, or None when the bridge does not publish one.

    None is not zero. A bridge with no clock reporting `sim_time: 0.0` would
    read on the roster as a robot frozen at t=0, which is a different fault
    from "this bridge does not report sim time".
    """
    if value is None or isinstance(value, bool):
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return round(out, places) if out == out else None


def _bool_or_none(value: Any) -> Optional[bool]:
    return bool(value) if isinstance(value, bool) else None


def _str_or_none(value: Any) -> Optional[str]:
    if value is None or isinstance(value, bool):
        return None
    text = str(value).strip()
    return text or None


def wake_text(event: Dict[str, Any]) -> str:
    """The user turn a waking event becomes. One line, fixed shape.

    `[Event] <type> at sim t=<s>: <detail>` -- the exact wording D4 step 5
    specifies, so a transcript reader (and the sweep) can tell a wake from
    an operator sentence without a side channel.
    """
    etype = str(event.get("type") or "event")
    raw_t = event.get("sim_time")
    try:
        stamp = f"{float(raw_t):.3f}" if raw_t is not None else "unknown"
    except (TypeError, ValueError):
        stamp = "unknown"
    return f"[Event] {etype} at sim t={stamp}: {_compact_detail(event.get('detail'))}"


# ── The edge connector: one per MACHINE, started beside the bridge ───
#
# A standing order fired by the platform's cron tick with no browser open
# dispatches its tool calls down a websocket to `omnilink.edge_connector`.
# The connector has always lived in omnilink-lib and NO bridge started it,
# so an unattended order failed with "edge not connected" unless the
# operator happened to be running a second process by hand. The relay
# starts it, because the relay is the thing that is already running
# whenever a robot can be driven at all.
#
# ONE PER MACHINE, and that is not an optimisation: the platform registry
# is `userId -> newest socket`, so a second connector REPLACES the first
# rather than joining it. Any connector can serve any local bridge (the
# tool frame carries the target's own `toolCallbackUrl`, which is how an
# unattended run ends up on the identical `/tool` an attended one uses), so
# one is sufficient and a replacement is merely wasteful.
#
# The claim is a lock FILE with a heartbeat rather than an OS file lock or
# a pid check. `os.kill(pid, 0)` is not a liveness probe on Windows -- it
# TERMINATES the process -- and an OS lock would need a different
# implementation per platform for a guarantee we do not need: two
# connectors racing at boot is harmless (see above), a stale lock from a
# killed engine is not, and a heartbeat distinguishes them with one mtime.
EDGE_LOCK_NAME = "edge.lock"
# Three missed beats. The owner rewrites the file every presence interval
# (30 s), so a lock older than this belongs to a process that is gone.
EDGE_LOCK_STALE_S = 95.0
_EDGE_LOCK_HELD = threading.Lock()
_edge_lock_owner: Optional[str] = None


def edge_state_dir() -> str:
    """Where cross-process bridge state lives on this machine.

    ⚠️ The same convention as `action_journal._default_journal_path` and as
    `omnisim/doctor.py::_edge_lock_path`. Three readers, one rule: if it
    moves, move all three. `tests/.../test_edge_start.py` pins doctor's copy
    against this one.
    """
    import tempfile
    return (os.environ.get("OMNILINK_INTENT_STATE_DIR")
            or os.path.join(tempfile.gettempdir(), "omnisim_intents"))


def edge_lock_path() -> str:
    return os.path.join(edge_state_dir(), EDGE_LOCK_NAME)


def read_edge_lock(path: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """The current lock record, or None when there is none. Never raises."""
    try:
        with open(path or edge_lock_path(), "r", encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def edge_lock_is_live(record: Optional[Dict[str, Any]],
                      now: Optional[float] = None) -> bool:
    """True when a record was written by a process that is still beating."""
    if not isinstance(record, dict):
        return False
    try:
        beat = float(record.get("beat") or record.get("started_at") or 0.0)
    except (TypeError, ValueError):
        return False
    return (now if now is not None else time.time()) - beat < EDGE_LOCK_STALE_S


def _write_edge_lock(agent: str, started_at: float) -> bool:
    path = edge_lock_path()
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = f"{path}.{os.getpid()}.tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump({"pid": os.getpid(), "agent": agent,
                       "started_at": started_at, "beat": time.time()}, fh)
        os.replace(tmp, path)
        return True
    except Exception:
        return False


def claim_edge_lock(agent: str) -> bool:
    """Take the machine's edge slot, unless a live process already holds it.

    Returns True when this process now owns it. Re-claiming from the owner
    is idempotent, so a second relay in the SAME process (three bridges in
    one demo share one) does not fight itself.
    """
    global _edge_lock_owner
    with _EDGE_LOCK_HELD:
        if _edge_lock_owner is not None:
            return False                      # this process already runs one
        held = read_edge_lock()
        if (edge_lock_is_live(held)
                and int(held.get("pid") or 0) != os.getpid()):
            return False
        if not _write_edge_lock(agent, time.time()):
            return False
        _edge_lock_owner = agent
        return True


def beat_edge_lock(agent: str) -> None:
    """Refresh our claim. Silent on failure -- a disk hiccup must not take
    the robot down, and the worst case is another bridge taking the slot."""
    if _edge_lock_owner != agent:
        return
    record = read_edge_lock()
    if isinstance(record, dict) and int(record.get("pid") or 0) != os.getpid():
        return                                # someone took it; stop claiming
    _write_edge_lock(agent, float((record or {}).get("started_at")
                                  or time.time()))


def release_edge_lock(agent: str) -> None:
    global _edge_lock_owner
    with _EDGE_LOCK_HELD:
        if _edge_lock_owner != agent:
            return
        _edge_lock_owner = None
    try:
        record = read_edge_lock()
        if isinstance(record, dict) and int(record.get("pid") or 0) == os.getpid():
            os.remove(edge_lock_path())
    except Exception:
        pass


def start_edge_connector(agent_name: str, omni_key: str) -> Dict[str, Any]:
    """Run omnilink-lib's edge connector in a daemon thread, tools-only.

    Returns a status record; it never raises and never blocks. States:

      ``off``              OMNILINK_EDGE=0
      ``no_key``           no OMNI_KEY in the environment, or this relay was
                           built with a different one (see below)
      ``unavailable``      omnilink-lib or websocket-client is missing
      ``held_elsewhere``   another live process on this machine owns the slot
      ``running``          the connector thread is up
      ``error``            it refused to start; the reason is in ``detail``

    ⚠️ TOOLS-ONLY. `EDGE_CAPABILITIES` is pinned to ``["tool"]`` before the
    loop starts, so this process never advertises local inference: an
    OmniKey is required for every OmniLink AI turn (access policy,
    2026-09-22) and a connector declaring ``chat`` would be offering a way
    around that. The declaration is load-bearing in the other direction
    too -- the server default-DENIES a connector that does not declare
    ``tool``, so dropping it silently breaks unattended runs.

    ⚠️ THE KEY MUST BE THE PROCESS'S OWN. `edge_connector.main()` reads
    OMNI_KEY from the environment, so a relay built with some other key
    would connect a socket that speaks for a different account. When they
    disagree we do not start -- which also keeps offline tests, which build
    relays with a literal placeholder key, from opening a real socket.
    """
    if not _env_flag("OMNILINK_EDGE", True):
        return {"state": "off", "detail": "OMNILINK_EDGE=0"}
    env_key = os.environ.get("OMNI_KEY", "").strip()
    if not env_key:
        return {"state": "no_key",
                "detail": "OMNI_KEY is not set in this process's environment"}
    if omni_key and omni_key.strip() != env_key:
        return {"state": "no_key",
                "detail": "this relay's key is not the process OMNI_KEY"}
    try:
        import websocket  # noqa: F401  (websocket-client)
    except Exception as exc:
        return {"state": "unavailable",
                "detail": f'websocket-client is not installed ({exc}); '
                          f'pip install "omnilink[bridges]"'}
    try:
        from omnilink import edge_connector as _edge  # type: ignore
    except Exception as exc:
        return {"state": "unavailable",
                "detail": f"omnilink.edge_connector is not importable ({exc})"}
    if not claim_edge_lock(agent_name):
        held = read_edge_lock() or {}
        return {"state": "held_elsewhere",
                "detail": f"pid {held.get('pid')} ({held.get('agent')}) "
                          f"already runs the edge connector on this machine"}
    try:
        _edge.EDGE_CAPABILITIES = ["tool"]

        def _worker() -> None:
            try:
                rc = _edge.main()
                print(f"[omnilink_relay] edge connector exited (rc={rc})")
            except Exception as exc:          # pragma: no cover - network path
                print(f"[omnilink_relay] edge connector stopped: {exc}")
            finally:
                release_edge_lock(agent_name)

        threading.Thread(target=_worker, name="omnilink-edge",
                         daemon=True).start()
        print(f"[omnilink_relay] edge connector starting (tools-only) for "
              f"{agent_name}")
        return {"state": "running", "detail": "tools-only, one per machine"}
    except Exception as exc:                  # pragma: no cover - defensive
        release_edge_lock(agent_name)
        return {"state": "error", "detail": f"{type(exc).__name__}: {exc}"}


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


# ── Refusals are a THIRD outcome (PROTOCOL §5.7.2) ───────────────────
#
# `ok` and `error` cannot express what the gate does. A refusal is not a
# success -- nothing moved. It is not an execution error either -- nothing
# failed; the bridge understood the request and declined it on purpose.
# Reporting it as an error tells a caller to RETRY, which is the one thing
# it must not do; reporting it as ok tells a caller the robot moved.
#
# The cost of not having the distinction is measured: a two-hour endurance
# run lost ~5% of its orders to refusals visible only in prose, logged zero
# errors from start to finish, and the robot drove off the floor.
_RULE_NAME = re.compile(r"^[a-z][a-z0-9_]*$")


def _refusal_rule(reason: Any) -> str:
    """The machine-readable rule behind a gate refusal.

    `gate.reject_toolcall` formats its verdict as `"<rule>: <detail>"`
    (§5.8.3's enumeration), so the rule is the head. The fail-CLOSED
    wrapper in `bridge_base` does not -- it answers "safety gate
    unavailable (ImportError)" when the gate will not import at all, and
    that is a real refusal with no rule name. It gets one here rather than
    a guessed member of the enumeration: the set is open and a client must
    tolerate an unknown name, but it must never be told `interrogative`
    when the truth is that nothing was checked.
    """
    head, sep, _detail = str(reason or "").partition(": ")
    if sep and _RULE_NAME.match(head):
        return head
    return "gate_unavailable"


def _result_refused(result: Any) -> bool:
    """The BRIDGE's own refusal marker: `accepted: False` (§5.4.1 rule 5).

    Distinct from `_result_failed`, which is the superset. A bounds refusal
    and a crashed dispatch are both failures; only the first is a refusal,
    and only the first means "do not retry these frames".
    """
    return isinstance(result, dict) and result.get("accepted") is False


def _action_error_line(actions: List[Dict[str, Any]]) -> str:
    """The top-level `error` string §5.7.2 REQUIRES when any entry refused.

    `<tool>: <rule>` for a refusal, `<tool>: <summary>` for a failure. A
    machine client reads fields, not sentences.

    ⚠️ Driven by `result`, never by a tool's own `error` key -- that key is
    the CONTROL error, a float, and a perfect stop carries 4.3e-11. A
    truthiness test here would report every successful motion as a failed
    prompt.
    """
    parts: List[str] = []
    for entry in actions:
        verdict = str(entry.get("result") or "").lower()
        if verdict not in ("refused", "err", "error"):
            continue
        tool = entry.get("tool") or "tool"
        why = (entry.get("rule") if verdict == "refused" else None) \
            or entry.get("summary") or verdict
        parts.append(f"{tool}: {str(why)[:160]}")
    return "; ".join(parts[:4])


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
        surface: Optional[str] = None,
        bridge: Optional[Any] = None,
    ) -> None:
        if not omni_key or not omni_key.strip():
            raise ValueError("An OmniKey is required to connect OmniLink.")
        check_omnilink_installation()
        _check_pypi_for_newer_async()
        self.omni_key = omni_key.strip()
        # ⚠️ SET BEFORE register_tools() BELOW. `surface` is the robot class
        # this bridge serves -- "mobile", "arm", "quadruped", "drone" (the
        # names in interpret.py). It was never assigned at all until
        # 2026-09-22, so the `getattr(self, "surface", None)` that feeds
        # gate.register_tools() read None for EVERY bridge and every tool in
        # the process was registered surface-less. A tool the gate has no
        # recorded surface for, called with no caller surface either, takes
        # the ground / body-shift rail -- which is the safe guess and the
        # wrong answer here: the drone's move_body{vertical} was judged
        # against a quadruped's 1.0 m body-shift rail instead of its own
        # 120 m climb.
        #
        # ⚠️ Do NOT restate that as "no surface means the strictest rail".
        # It is false as a general rule and this package has already had to
        # unpick three copies of it. What `gate.check` does: the caller's
        # surface wins; with none it uses the spec's recorded surface when
        # exactly ONE class registered the tool, and only discards it -- for
        # the ground rail -- when two or more did.
        self.surface = surface
        self.agent_name = agent_name
        self.main_task = main_task
        # The bridge, if it handed itself over. EVERY read of it is a
        # `getattr(..., default)` on a cached scalar -- `sim_time`, `held`,
        # `fault`, `world`, `events` -- and never a call into the controller
        # API, which is not thread-safe and drags the sim to ~0.2x realtime
        # when touched from a second thread. A bridge that never attaches
        # loses the presence detail and the wake, and nothing else.
        self._bridge: Optional[Any] = bridge
        # Where a wake turn's events go (window, chat card). A bridge
        # registers the real one with `set_event_sink`; the default logs.
        self._event_sink: Optional[Callable[[str, Dict[str, Any]], None]] = None
        # How a wake REQUEST from the platform raises the robot window. The
        # relay must not call the Robot API itself, so this is a hook the
        # bridge registers and which it marshals onto the sim thread.
        self._window_raiser: Optional[Callable[[], None]] = None
        self._wake_enabled = _env_flag("OMNILINK_EVENT_WAKE", True)
        self._wake_cursor: Optional[int] = None
        self._wake_count = 0
        self._wake_suppressed = 0
        self._wake_last_at = 0.0
        self._last_event: Optional[Dict[str, Any]] = None
        self._last_wake_requested_at: Optional[str] = None
        self.engine = engine
        self.temperature = temperature
        # None -> the pin for THIS instance's engine; "" -> send no model at
        # all and let the platform's adaptive selector decide. The pin is
        # per-engine because `gemini-3.5-flash` is a Google model id and a
        # relay may be built on another engine (see MODEL_PIN_BY_ENGINE).
        self.model = (model_for_engine(engine) if model is None else model).strip()

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

        # WHAT HAPPENED IN THE SIM, registered HERE for the same reason the
        # journal is: a per-bridge opt-in silently misses the bridges that
        # forget and every tool added later. The ring itself belongs to the
        # bridge; with no bridge attached this answers an explicit "not
        # available on this robot" rather than an empty list, because an
        # empty list reads as "nothing happened" and that is a claim.
        if not any(t.name == "get_events" for t in tools):
            tools.append(Tool(
                name="get_events",
                description=(
                    "What the SIMULATOR reported: contacts, joint limit hits, "
                    "faults, motions that timed out, refusals, damage. "
                    "READ-ONLY and instant.\n"
                    "\n"
                    "CALL THIS when the operator asks what went wrong, what "
                    "you noticed, or why something stopped -- and whenever you "
                    "were woken by an event and need its context. These are "
                    "MEASUREMENTS, not recollection: prefer them to your "
                    "memory of the run.\n"
                    "\n"
                    "Events are numbered. Pass the `next_since` you got last "
                    "time to see only what is new. `dropped` is how many fell "
                    "off the end before you read them -- if it is not zero, "
                    "say so rather than implying you saw everything."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "since": {
                            "type": "integer",
                            "description": (
                                "Cursor: return events after this number. "
                                "Omit for the most recent ones."),
                        },
                        "limit": {
                            "type": "integer",
                            "description": "How many to return (default 20).",
                        },
                        "types": {
                            "type": "string",
                            "description": (
                                "Comma-separated event types to keep, e.g. "
                                "'joint.limit_hit,contact.began'."),
                        },
                    },
                    "required": [],
                },
                dispatch=lambda a: self._read_events(a),
            ))

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
        # TEACH THE GATE WHAT THIS BRIDGE ACTUALLY SERVES.
        #
        # This is the one seam where every bridge's real tool set is known,
        # so it is the only place registration belongs. Before it, the gate
        # vetted frames against a hand-written table that had drifted from
        # the bridges -- ten of the arm's nineteen tools were missing from
        # it, and a missing tool is an UNGATED tool, because `unknown_tool`
        # is filtered at every call site. `grasp` and `set_tcp_target`
        # reached a motor unchecked; `place{xyz}` was refused outright.
        #
        # Wrapped because a gate that cannot register must not stop a bridge
        # from starting: the failure mode of an unregistered tool is the
        # status quo ante, which is survivable, and a bridge that refuses to
        # boot is not.
        try:
            from . import gate as _gate
            _gate.register_tools(tools, surface=self.surface)
        except Exception as exc:          # pragma: no cover - defensive
            print(f"[omnilink_relay] gate tool registration skipped: {exc}")
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
                    # THE JOURNAL RIDES IN HERE, and is split back out before
                    # the first turn -- see _restore_history. Without this the
                    # record of what the robot did dies with the machine that
                    # did it, and "what did you do before the restart?" goes
                    # back to being answered from narrative memory, which is
                    # where the 26% fabrication rate came from.
                    self.history = _restore_history(stored, HISTORY_LIMIT,
                                                    journal=self.journal)
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
        # The journal revision this process has got ONTO THE PLATFORM. 0 is
        # "nothing", which is the honest starting point even when the boot
        # restore has just filled the journal from disk: that file is the
        # copy that never left this machine.
        self._journal_synced_rev = 0
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
        self._presence_enabled = _env_flag("OMNILINK_PRESENCE", True)
        # THE ACTION JOURNAL RIDES THE SAME THREAD (see _sync_journal_if_dirty
        # for why it needs one at all). It is a second passenger, not a second
        # thread -- the event poll below is the first -- so the thread starts
        # when EITHER job wants it, and each job is gated on its own flag so
        # turning presence off does not silently turn the others on.
        # OMNILINK_JOURNAL_SYNC=0 opts out, value-parsed; with memory off
        # there is no memory write to ride and the answer is already no.
        self._journal_sync_enabled = bool(self._memory_enabled) and _env_flag(
            "OMNILINK_JOURNAL_SYNC", True)
        self._journal_sync_interval_s = JOURNAL_SYNC_S
        # One warning per outage, not one per beat: see the quiet branch of
        # _persist_memory_async.
        self._journal_sync_warned = False
        self._presence_thread: Optional[threading.Thread] = None
        if self._presence_enabled or self._journal_sync_enabled:
            self._presence_thread = threading.Thread(
                target=self._presence_loop, name="omnilink-presence", daemon=True)
            self._presence_thread.start()

        # UNATTENDED RUNS. One connector per machine; the first relay to
        # boot wins the slot and every other bridge's tool frames arrive
        # over it anyway, because the frame carries the target's own
        # callback URL. Never fatal: `start_edge_connector` reports its
        # reason and returns.
        self.edge_status: Dict[str, Any] = {"state": "unknown", "detail": ""}
        try:
            self.edge_status = start_edge_connector(self.agent_name, self.omni_key)
        except Exception as exc:              # pragma: no cover - defensive
            self.edge_status = {"state": "error",
                                "detail": f"{type(exc).__name__}: {exc}"}
        if self.edge_status.get("state") not in ("running", "off", "no_key"):
            print(f"[omnilink_relay] edge connector not started: "
                  f"{self.edge_status.get('state')} -- "
                  f"{self.edge_status.get('detail')}")

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

    # ── Attachments the bridge makes (all optional) ───────────────

    def attach_bridge(self, bridge: Any) -> None:
        """Hand the relay the bridge object it reports on.

        Everything the relay reads off it is a cached scalar or the event
        ring -- `sim_time`, `sim_step`, `held`, `fault`, `world`, `events`,
        `robot_id` -- read with `getattr(..., default)` so a bridge that
        publishes none of them still works and simply reports less. The
        relay NEVER calls the controller API through this handle: that API
        is not thread-safe and a couple of threaded reads per second have
        already been measured dragging the sim to ~0.2x realtime.
        """
        self._bridge = bridge

    def set_event_sink(self, sink: Callable[[str, Dict[str, Any]], None]) -> None:
        """Where a WAKE turn's events go (window, chat card, transcript).

        Same `(kind, payload)` callback shape the bridge already passes to
        `dispatch_async` for an operator turn, because a wake IS an ordinary
        turn -- it runs the same cascade, the same gate and the same memory
        write, and the only thing that differs is who wrote the sentence.
        """
        self._event_sink = sink

    def set_window_raiser(self, raiser: Callable[[], None]) -> None:
        """How a platform wake REQUEST raises the robot window.

        ⚠️ The callable is invoked from the PRESENCE thread, so the bridge's
        implementation must marshal onto the sim thread (`MainThreadCalls`)
        before it touches anything Webots-side. The relay cannot do that for
        it -- it has no sim thread of its own.
        """
        self._window_raiser = raiser

    # ── Events ────────────────────────────────────────────────────

    def _events_ring(self) -> Optional[Any]:
        """The bridge's ring, or None. Defensive on purpose: `events.py` is
        the other half of D4 and may be absent while this half exists."""
        ring = _safe_get(self._bridge, "events")
        return ring if ring is not None and hasattr(ring, "since") else None

    def _read_events(self, args: Dict[str, Any]) -> Dict[str, Any]:
        """`get_events` dispatch. Always a dict, never None, never raises.

        Abstaining by returning None is not a safe non-answer -- it reaches
        the model as a null tool result and gets narrated as "no events".
        An explicit `available: false` with a reason cannot be.
        """
        ring = self._events_ring()
        if ring is None:
            return {
                "available": False,
                "events": [],
                "total": 0,
                "reason": "this robot is not reporting simulator events",
                "note": ("No event stream is attached to this bridge, so this "
                         "is NOT evidence that nothing happened. Say that the "
                         "events surface is unavailable, never that the sim "
                         "was quiet."),
            }
        try:
            since = args.get("since")
            since = int(since) if since is not None else 0
        except (TypeError, ValueError):
            since = 0
        try:
            limit = int(args.get("limit") or 20)
        except (TypeError, ValueError):
            limit = 20
        limit = max(1, min(50, limit))
        raw_types = args.get("types")
        types: Optional[List[str]] = None
        if isinstance(raw_types, str) and raw_types.strip():
            types = [t.strip() for t in raw_types.split(",") if t.strip()]
        elif isinstance(raw_types, (list, tuple)):
            types = [str(t) for t in raw_types if str(t).strip()]
        try:
            batch = ring.since(since, limit=limit, types=types)
        except Exception as exc:
            return {"available": False, "events": [], "total": 0,
                    "reason": f"event read failed: {type(exc).__name__}",
                    "note": "The events surface errored; treat it as unknown, "
                            "not as quiet."}
        out = dict(batch) if isinstance(batch, dict) else {"events": list(batch or [])}
        out.setdefault("events", [])
        out["available"] = True
        out["note"] = (
            "Measured simulator events. `dropped` counts events that fell off "
            "the ring before they were read -- if it is non-zero, say the "
            "record is incomplete rather than implying you saw everything."
        )
        return out

    def event_wake_stats(self) -> Dict[str, Any]:
        """What the wake bucket has spent. Read by tests and by `/state`."""
        return {
            "enabled": bool(self._wake_enabled),
            "wakes": self._wake_count,
            "suppressed": self._wake_suppressed,
            "max_per_session": WAKE_MAX_PER_SESSION,
            "min_interval_s": WAKE_MIN_INTERVAL_S,
            "cursor": self._wake_cursor,
        }

    def poll_events_once(self, now: Optional[float] = None) -> int:
        """Drain new events, wake at most once. Returns the wakes dispatched.

        Called from the presence thread every WAKE_POLL_S. The FIRST call
        only primes the cursor: a bridge that restarts with a full ring must
        not wake on a backlog of events that were already answered (or that
        happened before anyone was listening).

        Never raises -- a telemetry poll that can throw would take down the
        heartbeat that runs beside it.
        """
        ring = self._events_ring()
        if ring is None:
            return 0
        if self._wake_cursor is None:
            # PRIMING. Jump to the newest sequence number the ring has ever
            # filed -- NOT to the end of a page. Draining the backlog through
            # `since()` would be limit-capped, so a ring holding 500 events
            # would leave ~300 of them unread and the SECOND poll would wake
            # on history: exactly the failure this pass exists to prevent.
            try:
                self._wake_cursor = int(_safe_get(ring, "total", 0) or 0)
                self._last_event = ring.last()
            except Exception:
                self._wake_cursor = 0
            return 0
        try:
            cursor = self._wake_cursor
            batch = ring.since(cursor, limit=200)
            if not isinstance(batch, dict):
                return 0
            events = list(batch.get("events") or [])
            nxt = batch.get("next_since")
            if isinstance(nxt, int):
                self._wake_cursor = nxt
            elif events:
                self._wake_cursor = cursor + len(events)
            if events:
                self._last_event = events[-1]
        except Exception:
            return 0
        if not self._wake_enabled or not events:
            return 0
        woken = 0
        for event in events:
            if not isinstance(event, dict):
                continue
            if not _wake_allowed(event):
                continue
            if woken or not self._wake_budget_ok(now):
                # The burst case: everything past the first qualifying event
                # is still READABLE through get_events, it just does not buy
                # a model turn. Counted so the cost of the policy is visible.
                self._wake_suppressed += 1
                continue
            if self._dispatch_wake(event, now):
                woken += 1
            else:
                self._wake_suppressed += 1
        return woken

    def _wake_budget_ok(self, now: Optional[float] = None) -> bool:
        stamp = time.monotonic() if now is None else now
        if self._wake_count >= WAKE_MAX_PER_SESSION:
            return False
        if self._wake_last_at and (stamp - self._wake_last_at) < WAKE_MIN_INTERVAL_S:
            return False
        return True

    def _dispatch_wake(self, event: Dict[str, Any],
                       now: Optional[float] = None) -> bool:
        """Turn one event into ONE relay turn. True when it was enqueued.

        The turn goes through `dispatch_async`, which means the model sees it
        as an ordinary user sentence and every tool it then calls is vetted
        by the same gate an operator turn is vetted by. Nothing here reaches
        an actuator on its own.
        """
        text = wake_text(event)
        stamp = time.monotonic() if now is None else now
        self._wake_count += 1
        self._wake_last_at = stamp
        try:
            handle = self.dispatch_async(text, self._wake_event_sink)
        except Exception as exc:              # pragma: no cover - defensive
            print(f"[omnilink_relay] wake dispatch failed: {exc}")
            handle = None
        if handle is None or handle.is_cancelled():
            # Never enqueued (closed relay, full queue): refund the budget
            # so a queue that is briefly busy does not burn the session's
            # thirty wakes on turns that never ran.
            self._wake_count -= 1
            self._wake_last_at = 0.0
            return False
        print(f"[omnilink_relay] {self.agent_name}: woken by "
              f"{event.get('type')} ({self._wake_count}/{WAKE_MAX_PER_SESSION})")
        return True

    def _wake_event_sink(self, kind: str, payload: Dict[str, Any]) -> None:
        sink = self._event_sink
        if sink is None:
            if kind in ("agent", "error"):
                print(f"[omnilink_relay] {self.agent_name}: wake {kind}: "
                      f"{str(payload.get('text', ''))[:200]}")
            return
        try:
            sink(kind, payload)
        except Exception as exc:              # pragma: no cover - defensive
            print(f"[omnilink_relay] wake sink failed: {exc}")

    # ── Presence ──────────────────────────────────────────────────

    #: The order the platform sees. `api/relay-heartbeat.ts` keeps the FIRST
    #: TWELVE keys of `detail` and truncates strings at 200 chars, so this is
    #: a priority list, not decoration: `interval_ms` sizes the staleness
    #: window (without it a 30 s beat is judged against a browser's 10 s and
    #: a healthy robot reads offline for 20 of every 30 seconds), so it goes
    #: first and can never be the key that falls off the end.
    PRESENCE_KEY_ORDER = (
        "interval_ms", "tools", "engine", "robot", "world", "held", "fault",
        "sim_time", "events_total", "last_event", "journal_len", "lockstep",
    )
    PRESENCE_MAX_KEYS = 12
    PRESENCE_MAX_CHARS = 200

    def presence_detail(self) -> Dict[str, Any]:
        """The heartbeat `detail`, capped to what the platform will store.

        Every field is read off the bridge with a default, so a bridge that
        publishes none of them heartbeats exactly as it did before. A `None`
        is DROPPED rather than sent: a null would spend one of the twelve
        slots saying nothing, and the roster renders a missing field and a
        null identically.
        """
        detail = dict(self._presence_detail)
        bridge = self._bridge
        live: Dict[str, Any] = {
            "sim_time": _round_or_none(_safe_get(bridge, "sim_time")),
            "held": _bool_or_none(_safe_get(bridge, "held")),
            "fault": _str_or_none(_safe_get(bridge, "fault")),
            "world": _str_or_none(_safe_get(bridge, "world")),
            "robot": detail.get("robot") or _str_or_none(
                _safe_get(bridge, "robot_id")),
            # Lockstep lives on the hold lease (`hold.py`), which is where
            # D6 put it; a bridge may also publish it directly. Reported as
            # "is this bridge RUNNING in lockstep", not "is it held right
            # now" -- `held` next to it already answers that, and the roster
            # needs to tell a paused demo from a reproducible one.
            "lockstep": _bool_or_none(
                _safe_get(_safe_get(bridge, "hold"), "enabled")
                if _safe_get(bridge, "hold") is not None
                else _safe_get(bridge, "lockstep")),
        }
        ring = self._events_ring()
        if ring is not None:
            try:
                live["events_total"] = int(getattr(ring, "total", 0) or 0)
            except Exception:
                live["events_total"] = None
            last = self._last_event
            if last is None:
                try:
                    last = ring.last()
                except Exception:
                    last = None
            if isinstance(last, dict):
                live["last_event"] = _compact_detail(
                    f"{last.get('type')}@{last.get('sim_time')}", 120)
        try:
            live["journal_len"] = len(self.journal)
        except Exception:
            live["journal_len"] = None
        for key, value in live.items():
            if value is not None:
                detail[key] = value

        ordered: Dict[str, Any] = {}
        for key in self.PRESENCE_KEY_ORDER:
            if key in detail and detail[key] is not None:
                ordered[key] = detail[key]
        for key, value in detail.items():      # whatever a bridge added
            if key not in ordered and value is not None:
                ordered[key] = value
        out: Dict[str, Any] = {}
        for key, value in ordered.items():
            if len(out) >= self.PRESENCE_MAX_KEYS:
                break
            if isinstance(value, str) and len(value) > self.PRESENCE_MAX_CHARS:
                value = value[: self.PRESENCE_MAX_CHARS]
            out[key] = value
        return out

    def _presence_loop(self) -> None:
        interval = self._presence_interval_s
        next_beat = 0.0
        # Far enough back that the first pass checks immediately: a boot
        # restore is the one moment we KNOW the local file may hold entries
        # the platform has never seen.
        last_sync = -1e9
        while not self._closed:
            now = time.monotonic()
            if self._presence_enabled and now >= next_beat:
                next_beat = now + interval
                try:
                    body: Dict[str, Any] = {
                        "agentName": self.agent_name,
                        "kind": "bridge",
                        "detail": self.presence_detail(),
                    }
                    if self._presence_endpoint:
                        body["endpoint"] = self._presence_endpoint
                    self._handle_presence_reply(self._post_presence(body))
                except Exception:
                    # Presence is telemetry. It must never take the robot down,
                    # and it must never spam the log on a flaky link -- a missed
                    # beat already shows up as "offline", which is the honest
                    # reading.
                    pass
                try:
                    beat_edge_lock(self.agent_name)
                except Exception:
                    pass
            # THE JOURNAL, on its own timer on this same thread. The interval
            # is read fresh every pass rather than latched at the top, so a
            # caller that changes it does not wait out the old one.
            sync_every = max(0.05, float(self._journal_sync_interval_s
                                         or JOURNAL_SYNC_S))
            if now - last_sync >= sync_every:
                last_sync = now
                try:
                    self._sync_journal_if_dirty()
                except Exception:
                    # Telemetry never takes the robot down.
                    pass
            # The event poll rides this thread rather than a new one: the
            # relay already owns it, and a bounded in-process deque read is
            # cheaper than the thread it would otherwise cost. Gated on
            # presence because that is what it has always been: until the
            # journal sync above, this thread existed only when presence did.
            if self._presence_enabled:
                try:
                    self.poll_events_once()
                except Exception:
                    pass
            if self._closed:
                return
            time.sleep(max(0.05, WAKE_POLL_S))

    def _post_presence(self, body: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Beat once and RETURN THE REPLY.

        The reply was discarded until 2026-09-22, which quietly threw away
        the only downstream channel the platform has to this process:
        `{ok, wakeRequestedAt}`. A `POST /api/wake-agent` sets that stamp,
        and nothing here noticed.
        """
        import urllib.request
        req = urllib.request.Request(
            f"{BASE_URL}/api/relay-heartbeat",
            data=json.dumps(body).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {self.omni_key}"},
        )
        with urllib.request.urlopen(req, timeout=8) as response:
            raw = response.read()
        try:
            data = json.loads(raw.decode("utf-8"))
        except Exception:
            return None
        return data if isinstance(data, dict) else None

    def _handle_presence_reply(self, data: Optional[Dict[str, Any]]) -> None:
        """Raise the window when the platform asks for attention.

        CHANGE-DETECTED, not level-triggered: `wakeRequestedAt` is a stamp
        that stays set, so acting on its presence would raise the window
        every 30 s forever after one request. The first reply only baselines
        it -- a request made before this bridge booted is not a request for
        this bridge.
        """
        if not isinstance(data, dict):
            return
        stamp = data.get("wakeRequestedAt")
        stamp = str(stamp) if stamp is not None else None
        previous = self._last_wake_requested_at
        self._last_wake_requested_at = stamp
        if previous is None or stamp is None or stamp == previous:
            return
        raiser = self._window_raiser
        print(f"[omnilink_relay] {self.agent_name}: platform requested "
              f"attention ({stamp})")
        if raiser is None:
            return
        try:
            raiser()
        except Exception as exc:              # pragma: no cover - defensive
            print(f"[omnilink_relay] window raise failed: {exc}")

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

    #: What `via` this relay reports. PROTOCOL §5.7.1 puts the stage that
    #: ANSWERED at the top level of the 200 body, and the sweep reads it from
    #: exactly there -- it will not infer it from `actions[]`, from latency or
    #: from whether a key was set, because inferring the column is fabricating
    #: the measurement. A turn that reaches this relay was answered by the
    #: model, so it is "relay"; the parser's short-circuit never gets here and
    #: reports "parser" from `route.py`.
    VIA = "relay"

    def dispatch_sync(self, text: str, timeout_s: float = 90.0) -> Dict[str, Any]:
        """Run dispatch_async and wait for it to finish.

        Returns `{via, response, actions: [{tool, result, summary, rule?}],
        error?}` -- the §5.7 `/prompt` envelope, so HTTP callers and
        side-menu callers see one wire format.

        Three things the shape is load-bearing about:

        * `via` is always present (see VIA above).
        * a REFUSAL is `result: "refused"` with a `rule`, not an `"err"`
          with the rule buried in prose (§5.7.2).
        * the top-level `error` is set whenever any entry refused or failed,
          and is computed from `result`, never from a tool's numeric
          `error` residual.
        """
        done = threading.Event()
        actions: List[Dict[str, Any]] = []
        agent_text = {"value": ""}
        error_text = {"value": ""}

        def _cb(kind: str, payload: Dict[str, Any]) -> None:
            if kind == "tool":
                entry: Dict[str, Any] = {
                    "tool": payload.get("name"),
                    "result": payload.get("status"),
                    "summary": payload.get("summary", ""),
                }
                rule = payload.get("rule")
                if isinstance(rule, str) and rule:
                    entry["rule"] = rule
                actions.append(entry)
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
                "via": self.VIA,
                "response": "",
                "actions": actions,
                "error": "timeout_after_actions" if actions or in_flight else "timeout",
                "cancelled": True,
                "in_flight": in_flight,
                "do_not_retry": bool(actions) or in_flight,
            }
        out: Dict[str, Any] = {"via": self.VIA,
                               "response": agent_text["value"],
                               "actions": actions}
        # §5.7.2: a top-level `error` MUST be set whenever any entry refused
        # or failed. The relay's own transport error wins the slot when it
        # has one -- it is the reason there is no answer at all -- and a
        # refusal fills it otherwise, so a machine client never has to read
        # prose to find out that nothing happened.
        if error_text["value"]:
            out["error"] = error_text["value"]
        else:
            line = _action_error_line(actions)
            if line:
                out["error"] = line
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
        # FLUSH THE JOURNAL BEFORE ANYTHING ELSE. The last actions of a
        # session are the ones an operator asks about first, and the next beat
        # is never going to come. Synchronous and bounded: a daemon thread
        # started here would be killed by the interpreter exiting behind it,
        # and a dead link must not hold a world shutdown open. A journal that
        # nothing has touched since the last beat writes nothing at all.
        try:
            flush = self._sync_journal_if_dirty(
                wait_for_lock=JOURNAL_CLOSE_FLUSH_S / 2)
            if flush is not None:
                flush.join(timeout=JOURNAL_CLOSE_FLUSH_S)
        except Exception:                     # pragma: no cover - defensive
            pass
        try:
            self._queue.put_nowait((None, None, None))
        except Full:
            pass
        # Hand the machine's edge slot back immediately rather than making
        # the next bridge wait out the staleness window. (A hard kill skips
        # this, which is exactly what the staleness window is for.)
        try:
            release_edge_lock(self.agent_name)
        except Exception:
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
                    # ⚠️ THE SAFETY GATE, ON THE MODEL'S PATH.
                    #
                    # Until 2026-09-21 `gate.check` ran in exactly one place
                    # -- route.py, the deterministic parser path -- so the
                    # PARSER was vetted and the MODEL was not. That is
                    # backwards from where the risk is: measured on 64
                    # held-out sentences, the parser produced 0 false
                    # actuations and the model produced 6-8, including
                    # "drive forward 300 metres" and acting on an order
                    # quoted inside a statement.
                    #
                    # (`handle.execution_gate` below is a threading lock --
                    #  same word, unrelated. The name collision is why this
                    #  gap survived a reading of the file.)
                    #
                    # With the veto here, the combined path scored 100% on
                    # that set and 11/11 with zero false refusals on a
                    # second set written afterwards.
                    # See tests/benchmarks/interpbench/RESULTS_2026-09-21.md
                    _rej = _gate_reject_toolcall(tool_name, args, text,
                                                 surface=self.surface)
                    if _rej is not None:
                        # §5.7.2: a refusal is its OWN outcome, and it
                        # carries the rule as a field. It used to be
                        # `status: "err"` with the rule behind a "refused: "
                        # prefix in the prose summary, so a client that had
                        # to tell "declined on purpose" from "failed, retry"
                        # had to parse a sentence -- and the two producers in
                        # this tree spelled that sentence differently.
                        _rule = _refusal_rule(_rej)
                        result = {"error": _rej, "accepted": False,
                                  "rule": _rule}
                        on_event("tool", {"name": tool_name,
                                          "status": "refused",
                                          "rule": _rule,
                                          "summary": f"refused: {_rej}"})
                        self.journal.record(tool_name, args, ok=False,
                                            summary=f"refused: {_rej}",
                                            result=result)
                        called_tools.append(tool_name)
                        messages.append({
                            "role": "tool",
                            "tool_call_id": tc.get("id") or tool_name,
                            "name": tool_name,
                            "content": json.dumps(result, default=str)})
                        continue
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
                    # A BRIDGE's own refusal (`accepted: False`, §5.4.1 rule
                    # 5) is a refusal too: an out-of-bounds target or a
                    # `wait: false` decline did not fail, it declined, and a
                    # caller must not retry it either. Its `rule` is whatever
                    # the bridge named, or `bridge_refused` -- never a
                    # borrowed gate rule, which would say a sentence was
                    # judged when it was not.
                    _refused = _result_refused(result)
                    _event: Dict[str, Any] = {
                        "name": tool_name,
                        "status": "refused" if _refused else ("ok" if ok else "err"),
                        "summary": summary,
                    }
                    if _refused:
                        _rule = result.get("rule") if isinstance(result, dict) else None
                        _event["rule"] = (_rule if isinstance(_rule, str) and _rule
                                          else "bridge_refused")
                    on_event("tool", _event)
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

    def _journal_dirty(self) -> bool:
        """Is there journal content the platform has not been told about?

        A sequence number rather than a content hash, because this is read on
        a timer: hashing would have to serialise the whole export to discover
        that nothing happened. It errs towards a redundant write, never
        towards a skipped one (ActionJournal.revision says so).
        """
        if not self._journal_sync_enabled:
            return False
        try:
            return self.journal.revision() != self._journal_synced_rev
        except Exception:                     # pragma: no cover - defensive
            return False

    def _sync_journal_if_dirty(
        self,
        *,
        wait_for_lock: float = 0.0,
    ) -> Optional[threading.Thread]:
        """Get the journal onto the platform WITHOUT a model turn.

        WHY THIS EXISTS. The journal rides inside the memory write (loop plan
        L7), and until 2026-09-22 the memory write had exactly two triggers
        and both of them needed a COMPLETED model turn -- the `remember_this`
        tool, and the tail of `_dispatch_one`. Two whole classes of session
        therefore never synced at all:

        * one the deterministic parser answered by itself. It short-circuits
          in route.py and never enters the dispatch loop, which is the entire
          point of parser-first -- so the BETTER the parser worked, the less
          the record persisted. A failure that grows with the feature's
          success is not a corner case.
        * one whose model turns all failed. Measured live: every turn on the
          account answered `402 BYOK_REQUIRED`, and that branch of
          `_dispatch_one` returns BEFORE the persist call, so a whole session
          of driving stayed on one disk.

        The local file always held the record, so "survives a restart on this
        machine" was true throughout. "Follows the agent to another machine"
        was not, and that is the half the journal exists for.

        Cadence and thread are both deliberate: it fires on the presence
        beat, on the presence thread. Writing per `record()` would put an
        HTTP round trip on the robot's tool-call path, and a thread of its
        own would be a third one for a job that is two comparisons.
        """
        if not self._journal_dirty():
            return None
        return self._persist_memory_async(quiet=True,
                                          wait_for_lock=wait_for_lock)

    def _persist_memory_async(
        self,
        *,
        quiet: bool = False,
        wait_for_lock: float = 0.0,
    ) -> Optional[threading.Thread]:
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
            # A turn's write never queues behind another (the next turn will
            # carry the same content anyway); a CLOSE waits, because there is
            # no next turn to carry it.
            if wait_for_lock > 0:
                got = self._memory_write_lock.acquire(timeout=wait_for_lock)
            else:
                got = self._memory_write_lock.acquire(blocking=False)
            if not got:
                return
            try:
                conv = _to_memory_format(snapshot)
                # D7: the journal travels with the conversation. Built here,
                # on the write thread, so it is as fresh as the turn that
                # triggered the write; bounded by `export_entries`, so a
                # long-lived robot cannot grow the memory row without limit.
                #
                # The revision is read BEFORE the export and never after. An
                # entry recorded in between then rides in this write but
                # leaves the marker behind it, which costs one redundant
                # write later; the other order would mark an entry as synced
                # that the export missed, and lose it.
                try:
                    rev = self.journal.revision()
                except Exception:
                    rev = None
                try:
                    section = _journal_entry(self.journal.export_entries())
                except Exception:
                    section = None
                if section is not None:
                    conv = conv + [section]
                with self._lock:
                    pins = list(self._pinned_facts)
                merged = conv
                read_ok = True
                try:
                    stored = self._client.get_memory(self.agent_name) or []
                    merged = _merge_memory(stored, conv, PERSIST_LIMIT, pins)
                except Exception:
                    # Read failed: fall back to writing our own view rather
                    # than skipping the write entirely. Losing the merge is
                    # bad; losing this session's turns as well is worse.
                    read_ok = False
                if not read_ok and not any(not _is_journal(e) for e in conv):
                    # ...with one exception, and it arrived with the
                    # background sync. A JOURNAL-ONLY write is what a session
                    # with no model turns produces, and "our own view" is then
                    # the section ALONE -- so a blind write would replace the
                    # stored row, every turn of every earlier session on every
                    # surface, with a 3 kB JSON blob. Skip it, stay dirty, let
                    # the next beat try again with a working read.
                    return
                self._client.set_memory(self.agent_name, merged)
                if rev is not None:
                    self._journal_synced_rev = rev
                self._journal_sync_warned = False
            except Exception as e:
                # Memory is best-effort. Log and move on.
                if not quiet:
                    print(f"[omnilink_relay] memory persist failed: {e}")
                elif not self._journal_sync_warned:
                    # A beat on a flaky link must not fill the log -- the same
                    # discipline as the presence beat. Say it once, then stay
                    # quiet until a write succeeds.
                    self._journal_sync_warned = True
                    print(f"[omnilink_relay] memory persist failed: {e} "
                          f"(further background failures stay quiet)")
            finally:
                self._memory_write_lock.release()

        thread = threading.Thread(target=_worker, name="omnilink-mem-write",
                                  daemon=True)
        thread.start()
        return thread

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
                # ⚠️ A 402 is NOT always a missing provider key. Until
                # 2026-09-23 this branch treated EVERY 402 as BYOK_REQUIRED
                # (`or e.status_code == 402`). The platform also answers 402
                # AGENT_LIMIT_REACHED when the account holds more agent
                # profiles than its plan allows, and that refusal reached
                # operators as "connect a model-provider key" -- measured: a
                # day of diagnosis went the wrong way on exactly that, on an
                # account whose Google key was working all along. Anyone who
                # fills their agent allowance would be told to add a key they
                # already have. The SDK carries the server's code; branch on
                # it, and never assert a cause the platform did not state.
                code = getattr(e, "code", None)
                detail = _platform_message(e)
                is_byok = ((OmniLinkBYOKRequiredError is not None
                            and isinstance(e, OmniLinkBYOKRequiredError))
                           or code == "BYOK_REQUIRED"
                           or (code is None and e.status_code == 402
                               and "BYOK" in str(getattr(e, "body", "") or "")))
                if is_byok:
                    # The platform body points at a browser upload screen and
                    # names service-account JSON, but a free API key works and
                    # there is a terminal command for it -- say THAT instead.
                    return {
                        "ok": False,
                        "error": (
                            "OmniLink needs a model-provider key (402 "
                            "BYOK_REQUIRED). Your Omni Key identifies the "
                            "account; a provider key pays for the tokens. "
                            "Connect one with:  python -m omnisim byok --add "
                            "google   (free tier, no card; `byok --providers` "
                            "lists the rest). AI instructions are unavailable "
                            "until a model connection is configured."),
                    }
                if code == "AGENT_LIMIT_REACHED":
                    return {
                        "ok": False,
                        "error": (
                            "OmniLink refused to run this agent: the account "
                            "holds more agent profiles than its plan allows "
                            "(402 AGENT_LIMIT_REACHED). Remove unused agent "
                            "profiles or upgrade the plan. Your model-provider "
                            "key is not the problem."
                            + (f" Platform: {detail}" if detail else "")),
                    }
                if e.status_code == 402:
                    # A payment-class refusal we have no specific text for.
                    # Report the platform's own code and words; do not guess.
                    return {
                        "ok": False,
                        "error": (f"OmniLink refused this turn (402"
                                  f"{' ' + code if code else ''})"
                                  + (f": {detail}" if detail else ".")),
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


# ── The action journal, riding in the memory row (D7) ────────────────
#
# The journal removed a 26% fabrication rate, and until now it died with
# the machine: `action_journal` persists to a file in the temp dir, so a
# different OmniSim instance, a different machine or a cleaned temp dir
# left the robot answering "what did you do before the restart?" from
# narrative memory again -- the exact surface the journal exists to
# replace. The platform memory row is the one channel that already
# survives all three, so the journal rides inside it.
#
# It is a SECTION of that row, in the same notes-tier shape `_merge_memory`
# already lifts out and re-inserts, so it never competes with a turn for a
# slot in the window and never gets deduped against one. It is stripped out
# again on restore: the model reads the journal through
# `get_action_history`, and feeding it a raw JSON blob as a conversation
# turn would cost tokens on every round to say the same thing worse.
JOURNAL_MARKER = "[[ACTION JOURNAL — tool calls this robot dispatched]]"


def _is_journal(entry: Any) -> bool:
    if not isinstance(entry, dict) or entry.get("role") != "user":
        return False
    parts = entry.get("parts") or []
    text = " ".join(p.get("text", "") for p in parts if isinstance(p, dict))
    return text.startswith(JOURNAL_MARKER)


def _journal_entry(rows: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Build the memory entry that carries `rows`, or None for an empty one."""
    if not rows:
        return None
    try:
        body = json.dumps(rows, default=str, separators=(",", ":"))
    except Exception:
        return None
    return {"role": "user",
            "parts": [{"text": JOURNAL_MARKER + "\n" + body}]}


def _journal_rows(entry: Any) -> List[Dict[str, Any]]:
    """Parse a journal section back into entries. Never raises."""
    if not _is_journal(entry):
        return []
    parts = entry.get("parts") or []
    text = " ".join(p.get("text", "") for p in parts if isinstance(p, dict))
    _, _, body = text.partition("\n")
    try:
        rows = json.loads(body)
    except Exception:
        return []
    return [r for r in rows if isinstance(r, dict)] if isinstance(rows, list) else []


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

    # Same treatment for the action-journal section, and for the same two
    # reasons -- but ALSO because eviction folds a dropped user turn's text
    # into the standing notes, and folding a 3 kB JSON blob into the notes
    # would poison the one tier whose whole value is the operator's words.
    # OURS WINS when we have one: this process's journal already merged
    # whatever was restored from the platform, so it is a superset, while
    # the stored copy may be an older machine's view.
    journal_entry = (next((e for e in mine if _is_journal(e)), None)
                     or next((e for e in stored if _is_journal(e)), None))
    stored = [e for e in stored if not _is_journal(e)]
    mine = [e for e in mine if not _is_journal(e)]

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

    if journal_entry is not None:
        out.insert(0, journal_entry)
    if lines:
        out.insert(0, _build_notes(lines))
    elif notes_entry is not None:
        out.insert(0, notes_entry)
    return out


def _restore_history(stored: List[Dict[str, Any]],
                     limit: int,
                     journal: Optional[Any] = None) -> List[Dict[str, Any]]:
    """Rebuild the working history from a stored blob, keeping the notes.

    Naively taking ``[-limit:]`` throws the pinned entry away: it sits at
    index 0 of a blob that is deliberately larger than the context window, so
    the standing facts would be persisted faithfully and then dropped on the
    way back in -- the tier would look implemented and do nothing. Pin it
    first, then fill the remaining slots with the most recent turns.

    `journal`, when given, receives the `[[ACTION JOURNAL]]` section BEFORE
    the first turn runs, which is the whole point of carrying it: the robot
    has to be able to answer "what did you do before the restart?" from a
    record on its very first question, not after it has done something new.
    The section never enters the returned history -- the model reads it
    through `get_action_history`, not as conversation text.
    """
    entries = list(stored or [])
    if journal is not None:
        rows: List[Dict[str, Any]] = []
        for entry in entries:
            rows.extend(_journal_rows(entry))
        if rows:
            try:
                added = journal.import_entries(rows)
                print(f"[omnilink_relay] restored {added} action-journal "
                      f"entries from short-term memory")
            except Exception as exc:          # pragma: no cover - defensive
                print(f"[omnilink_relay] journal restore skipped: {exc}")
    entries = [e for e in entries if not _is_journal(e)]
    notes = next((e for e in entries if _is_notes(e)), None)
    body = [e for e in entries if not _is_notes(e)]
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


# ── Local model compatibility (OmniKey required) ──────────────────────
#
# ⚠️ NOT A KEYLESS PATH, AND NOT A FREE ONE. This comment used to read
# "the free-tier path that costs nobody anything: no OmniLink account, no
# cloud key ... prompts route to a locally running Ollama server instead
# of /api/chat", and the constructor eighteen lines below has refused
# exactly that since the 2026-09-22 access policy. A local model is
# connected THROUGH OmniLink: `OllamaRelay` requires an OmniKey, raises
# without one, and then delegates to `OmniLinkRelay` with engine
# "g5-engine" -- the same transport as every other engine. The bridge's
# access check runs BEFORE anything is parsed or interpreted, so a keyless
# /prompt is refused 401 `omnikey_required` whatever is listening on
# localhost.
#
# Nothing here is selected automatically. Nothing in this tree constructs
# `OllamaRelay` at all: a caller names it, or an operator names the engine
# (OMNILINK_ENGINE). `ollama_available()` below only REPORTS whether a
# local server answered; it selects nothing, and a local server that
# happens to be up is never picked up on its own -- not at start-up, and
# not as a fallback after a connection error.
#
# Platform features (short-term memory, usage telemetry, TTS/STT) follow
# whatever the hosted side grants that engine.

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
    """Compatibility entry point for local models connected through OmniLink."""

    def __init__(self, agent_name, main_task, tools, omni_key="", **kwargs):
        if not omni_key or not omni_key.strip():
            raise ValueError("An OmniKey is required. Connect your local model through OmniLink.")
        super().__init__(omni_key=omni_key.strip(), agent_name=agent_name,
                         main_task=main_task, tools=tools, engine="g5-engine",
                         model=kwargs.get("model", ""),
                         surface=kwargs.get("surface"),
                         bridge=kwargs.get("bridge"))


# ── Convenience ──────────────────────────────────────────────────────

def is_enabled() -> bool:
    """True if the env vars are set for OmniLink relay mode."""
    return bool(os.environ.get("OMNI_KEY", "").strip())


def get_omni_key() -> str:
    return os.environ.get("OMNI_KEY", "").strip()
