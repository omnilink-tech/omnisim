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

"""omnilink_mobile_bridge — generic OmniLink bridge for URDF wheeled bases.

Drop this in as the URDFRobot's controller (`supervisor TRUE`):

    controller "omnilink_mobile_bridge"
    controllerArgs ["--robot" "husky" "--port" "8765"]

Picks a config from `_mobile_configs.py` and drives the URDF's wheel
motors with a unified diff-drive / skid-steer surface. Adding a new
base: append a config and (if needed) a new wheel-motor layout to
WHEEL_MOTORS.

Surfaces
--------
HTTP on 127.0.0.1:<port> (default 8765), Axis-normalized:

    POST /list_robots         -> [{id, model, capabilities}]
    POST /get_robot_state     -> {x, y, yaw, v_linear, v_angular, mode, fault, ...}
    POST /set_velocity        -> {accepted, linear, angular, expires_in_s}
    POST /drive_forward       -> {accepted, distance, eta_s}
    POST /turn                -> {accepted, angle_rad, eta_s}
    POST /stop_robot          -> {halted_at, commanded, achieved, error,
                                  settled, stationary, measured, idle_loop}
    POST /reset_to_home       -> teleport to start pose
    POST /prompt              -> natural language

Robot window: same omnilink_chat plugin as the arm bridge. Wire protocol
is identical: "prompt:<text>", "stop", "configure" inbound;
"configure:<json>" / "status:..." / "agent:..." / "tool:..." outbound.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

from omnisim import Supervisor

import os as _os
import re as _re

# Process start, for the wall-clock stamp on idle-loop events (see _log).
_T0 = time.time()
import sys as _sys
_THIS_DIR = _os.path.dirname(_os.path.abspath(__file__))
if _THIS_DIR not in _sys.path:
    _sys.path.insert(0, _THIS_DIR)
_RELAY_PARENT = _os.path.abspath(_os.path.join(_THIS_DIR, ".."))
if _RELAY_PARENT not in _sys.path:
    _sys.path.insert(0, _RELAY_PARENT)

# Shared conversational intents (resume / status) + the honest /state
# describer -- see omnisim_bridges.intent_router. Optional: a bare clone
# without the package installed keeps working, minus those two intents.
try:  # noqa: E402
    from omnisim_bridges.intent_router import (
        describe_state as shared_describe_state,
        is_resume as shared_is_resume,
        is_status as shared_is_status,
    )
except Exception:  # pragma: no cover - optional dependency
    shared_describe_state = None
    shared_is_resume = None
    shared_is_status = None

# The DEFERRED-INTENT layer: the persistent store + tools that let a model
# say "stop AFTER you park this cart" / "don't go in there until I say so"
# and have it actually happen. Optional for the same reason as above -- a
# bare clone without the package keeps every immediate tool, and simply
# cannot schedule (which is the honest degradation: the tools are absent,
# so the model is never invited to promise something nothing records).
try:  # noqa: E402
    from omnisim_bridges.intents import (
        IntentStore,
        build_intent_tools,
        DEFERRED_TOOLS,
        MAIN_TASK_RULE as INTENT_TASK_RULE,
    )
except Exception:  # pragma: no cover - optional dependency
    IntentStore = None  # type: ignore[assignment]
    build_intent_tools = None  # type: ignore[assignment]
    DEFERRED_TOOLS = frozenset()
    INTENT_TASK_RULE = ""

from _mobile_configs import (MOBILE_CONFIGS, STATIC_BASE_OBSTACLES,  # noqa: E402
                             WHEEL_MOTORS, get_config)
from _omnilink_relay.http_security import (  # noqa: E402
    RequestError,
    RequestIdGuard,
    allowed_origins,
    check_authorization,
    check_protocol_version,
    checked_origin,
    configured_token,
    error_envelope,
    finite_number,
    nonempty_string,
    read_json,
    require_field,
    validate_request_id,
    WIRE_SERVICE,
    WIRE_VERSION,
)

try:
    from _omnilink_relay import (  # noqa: E402
        OmniLinkRelay,
        OllamaRelay,
        Tool,
        is_enabled as omnilink_enabled,
        get_omni_key,
        ollama_available,
    )
    # Module scope on purpose: agent_name_for() is called near the top of
    # setup_omnilink_relay(). Do NOT re-import this inside that function --
    # a function-local import binds the name as a local for the whole body
    # and turns that earlier call into an UnboundLocalError.
    from _omnilink_relay import profile_sync
    # Importing it here again would rebind it as a function-local for
    # the WHOLE function, making the earlier agent_name_for() call an
    # unbound local -- which is exactly the regression this replaces.
except Exception:
    OmniLinkRelay = None  # type: ignore[assignment]
    OllamaRelay = None  # type: ignore[assignment]
    Tool = None  # type: ignore[assignment]
    def omnilink_enabled() -> bool: return False
    def get_omni_key() -> str: return ""
    def ollama_available() -> bool: return False
    profile_sync = None  # type: ignore[assignment]


# ── Chat transcript (opt-in: OMNILINK_TRANSCRIPT=<path>) ─────────────
#
# WHY THIS EXISTS. After a chat turn there is nowhere an operator can read
# back what the robot was ASKED or what it ANSWERED. Controller stdout is
# discarded (omnisim-bin.exe is a Windows GUI-subsystem binary, and
# OmLog::appendStdout writes to std::cout + a Qt signal but never calls
# fileLog(), so even --stdout produces a 0-byte capture); the ActionJournal
# under %TEMP%/omnisim_intents/journal_*.json records TOOL CALLS ONLY -- no
# prompt, no reply; and GET /usage carries token counts and timing. So a bad
# answer cannot be reviewed, or improved, after the fact. This appends one
# JSON line per chat turn to $OMNILINK_TRANSCRIPT.
#
# WHY THE TOOL CALLS ARE IN THE RECORD. A reply cannot be judged on its own:
# "I moved it a metre" is either true or fabricated depending on what the
# tools MEASURED, and the distinction an operator needs is between "said 1 m,
# the tool measured 0.998" and "said 1 m, called nothing at all". So each
# record pairs the reply with THIS turn's tool calls -- their arguments and
# their measured summaries -- sliced out of the relay's own ActionJournal by
# its monotonic sequence number `n`, through the public listing() surface.
# That is the correlation the relay already maintains; this does not invent a
# second one. Turns are serialised by the relay's single worker thread, so
# the slice `n > n_at_prompt` is exactly this turn's calls (including any
# auto-read the grounding gate performed on the model's behalf).
#
# CONTRACT. Additive, exception-safe, and OFF unless OMNILINK_TRANSCRIPT is
# set: with it unset _tx_begin/_tx_end are no-ops and _tx_window_cb returns
# the caller's callback OBJECT UNCHANGED, so not one byte of the chat path
# differs. A logging fault must never break a chat turn or a control action,
# so every helper below swallows, and every call site is placed AFTER the
# functional work it observes.

_TX_TAG = "omnilink_mobile_bridge"
_TX_BRIDGE = "mobile"

_TX_PATH = _os.environ.get("OMNILINK_TRANSCRIPT", "").strip()
_TX_ENABLED = bool(_TX_PATH)


def _tx_env_float(name: str, default: float) -> float:
    try:
        return float(_os.environ.get(name, "") or default)
    except Exception:
        return default


# Seconds a record may wait, OFF the chat path, for this turn's usage delta.
# The relay snapshots usage on its own thread AFTER the "idle" event (~1.9 s
# measured), so emitting the instant the reply lands would drop the token
# counts from every single turn. Skipped entirely when the relay has no
# meter; set to 0 to always emit immediately.
_TX_USAGE_WAIT_S = _tx_env_float("OMNILINK_TRANSCRIPT_USAGE_WAIT", 4.0)

# The Omni Key must never reach a transcript. Match the key SHAPE, so a key
# pasted mid-sentence by the operator or echoed back by the model is caught
# as well as one in a tool argument; then also blank the literal value of any
# key this process was actually started with, which covers a key whose shape
# the pattern does not know.
_TX_KEY_RE = re.compile(r"olink_[A-Za-z0-9]+")
_TX_SECRETS = tuple(v for v in (_os.environ.get(n, "").strip()
                                for n in ("OMNI_KEY", "OMNILINK_KEY",
                                          "OMNILINK_API_KEY"))
                    if len(v) >= 8)


def _tx_scrub(value: Any) -> Any:
    """Recursively strip anything key-shaped out of a transcript value."""
    try:
        if isinstance(value, str):
            out = _TX_KEY_RE.sub("olink_<redacted>", value)
            for secret in _TX_SECRETS:
                if secret in out:
                    out = out.replace(secret, "<redacted>")
            return out
        if isinstance(value, dict):
            return {str(k): _tx_scrub(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [_tx_scrub(v) for v in value]
        return value
    except Exception:
        return "<unscrubbable>"


def _tx_iso(t: float) -> Optional[str]:
    try:
        return (time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t))
                + ".%03dZ" % int((t - int(t)) * 1000))
    except Exception:
        return None


class _TranscriptWriter:
    """Append-only JSONL sink. Line-buffered; never raises at the call site."""

    def __init__(self, path: str) -> None:
        self._path = path
        self._fh = None
        self._lock = threading.Lock()
        self._seq = 0
        self._failures = 0

    def write(self, record: Dict[str, Any]) -> None:
        if not _TX_ENABLED:
            return
        try:
            with self._lock:
                self._seq += 1
                record["turn"] = self._seq
                line = json.dumps(_tx_scrub(record), default=str,
                                  ensure_ascii=False)
                if self._fh is None:
                    parent = _os.path.dirname(_os.path.abspath(self._path))
                    if parent:
                        _os.makedirs(parent, exist_ok=True)
                    self._fh = open(self._path, "a", encoding="utf-8",
                                    buffering=1)
                self._fh.write(line + "\n")
        except Exception as e:
            # Drop the handle so the next turn re-opens: a transient disk or
            # lock error must not silently disable the transcript for the
            # life of the process. Report the first few, then stay quiet.
            try:
                if self._fh is not None:
                    self._fh.close()
            except Exception:
                pass
            self._fh = None
            self._failures += 1
            if self._failures <= 3:
                print("[%s] transcript write failed: %r" % (_TX_TAG, e))


_TX_WRITER = _TranscriptWriter(_TX_PATH) if _TX_ENABLED else None


def _tx_sim_t(bridge: Any) -> Optional[float]:
    """Sim clock, cached by the main loop -- never a cross-thread Robot call."""
    try:
        t = getattr(bridge, "last_sim_t", None)
        return round(float(t), 3) if t is not None else None
    except Exception:
        return None


def _tx_mode(relay: Any) -> Tuple[str, Optional[str]]:
    if relay is None:
        return ("offline-regex", None)
    try:
        ident = relay.relay_identity() or {}
    except Exception:
        ident = {}
    engine = ident.get("engine") or getattr(relay, "engine", None)
    kind = ident.get("kind")
    if kind == "ollama" or (isinstance(engine, str)
                            and engine.startswith("ollama:")):
        return ("ollama", engine)
    return ("omnilink", engine)


def _tx_journal_seq(relay: Any) -> Optional[int]:
    """The journal's monotonic counter right now -- the turn's low-water mark."""
    try:
        journal = getattr(relay, "journal", None)
        if journal is None:
            return None
        return int(journal.listing(limit=1).get("total_recorded") or 0)
    except Exception:
        return None


def _tx_journal_slice(relay: Any, n0: Optional[int]) -> Optional[List[Dict[str, Any]]]:
    """Tool calls journalled since `n0` -- this turn's, with their arguments."""
    if n0 is None:
        return None
    try:
        journal = getattr(relay, "journal", None)
        if journal is None:
            return None
        rows = journal.listing(limit=50).get("entries") or []
    except Exception:
        return None
    out: List[Dict[str, Any]] = []
    for row in rows:
        try:
            if int(row.get("n", 0)) <= n0:
                continue
            call = {"name": row.get("tool"), "args": row.get("args"),
                    "ok": bool(row.get("ok")), "summary": row.get("summary", "")}
            for extra in ("kind", "accepted", "moved", "error"):
                if extra in row:
                    call[extra] = row[extra]
            out.append(call)
        except Exception:
            continue
    return out


def _tx_latest_usage(relay: Any) -> Optional[Dict[str, Any]]:
    try:
        return relay.latest_usage() if relay is not None else None
    except Exception:
        return None


def _tx_usage_expected(relay: Any) -> bool:
    """False when the relay has no meter, so no usage event will ever land."""
    if relay is None:
        return False
    try:
        # Sentinel, not None: an absent attribute means a future relay we do
        # not know, and waiting a bounded few seconds is the safe guess.
        return getattr(relay, "_meter", "unknown") is not None
    except Exception:
        return True


class _TxTurn:
    """One chat turn, from the prompt to its terminal event."""

    def __init__(self, bridge: Any, relay: Any, prompt: str, source: str) -> None:
        self.bridge = bridge
        self.relay = relay
        self.prompt = prompt
        self.source = source
        self.t0 = time.time()
        self.p0 = time.perf_counter()
        self.sim_t = _tx_sim_t(bridge)
        self.journal_n0 = _tx_journal_seq(relay)
        self.usage_before = _tx_latest_usage(relay)
        self.reply = ""
        self.error = ""
        self.usage: Optional[Dict[str, Any]] = None
        self.ev_tools: List[Dict[str, Any]] = []
        self.tools_from = "events"
        self.latency_s: Optional[float] = None
        self.t_end: Optional[float] = None
        self._usage_seen = threading.Event()
        self._lock = threading.Lock()
        self._claimed = False

    # -- the window path: one relay event at a time -------------------- #

    def observe(self, kind: str, payload: Dict[str, Any]) -> None:
        if kind == "tool":
            self.ev_tools.append({
                "name": payload.get("name"),
                "args": None,          # the event carries none; the journal does
                "ok": payload.get("status") == "ok",
                "summary": payload.get("summary", ""),
            })
        elif kind == "agent":
            self.reply = str(payload.get("text", ""))
        elif kind == "usage":
            self.usage = payload
            self._usage_seen.set()
        elif kind == "error":
            # Terminal: _dispatch_one returns after an error without ever
            # emitting "idle" -- the same terminal rule dispatch_sync uses.
            self.error = str(payload.get("text", ""))
            self.finish()
        elif kind == "status" and payload.get("state") == "idle":
            self.finish()

    # -- the HTTP / offline paths: one completed turn ------------------ #

    def complete(self, reply: str = "", actions: Any = None,
                 router_tools: Any = None, error: str = "") -> None:
        if reply:
            self.reply = reply
        if error:
            self.error = error
        if actions:
            self.ev_tools = [{"name": a.get("tool"), "args": None,
                              "ok": a.get("result") == "ok",
                              "summary": a.get("summary", "")}
                             for a in actions if isinstance(a, dict)]
        if router_tools:
            self.tools_from = "router"
            self.ev_tools = [{"name": t[0], "args": None, "ok": t[1] == "ok",
                              "summary": t[2]}
                             for t in router_tools if len(t) >= 3]
        self.finish()

    # -- emit ---------------------------------------------------------- #

    def finish(self) -> None:
        with self._lock:
            if self._claimed:
                return                  # one record per turn, always
            self._claimed = True
        # Stamped HERE, not at emit time, so a usage wait never inflates the
        # latency the operator reads.
        self.latency_s = round(time.perf_counter() - self.p0, 3)
        self.t_end = time.time()
        if (self._usage_seen.is_set() or _TX_USAGE_WAIT_S <= 0
                or not _tx_usage_expected(self.relay)):
            self._emit()
            return
        threading.Thread(target=self._wait_then_emit,
                         name="omnilink-transcript", daemon=True).start()

    def _wait_then_emit(self) -> None:
        try:
            deadline = time.time() + _TX_USAGE_WAIT_S
            while time.time() < deadline:
                if self._usage_seen.wait(timeout=0.2):
                    break
                # The HTTP path never sees the usage EVENT (dispatch_sync
                # keeps its own callback), so also watch the relay's rollup.
                # Identity, not equality: _last_usage is a fresh dict per
                # snapshot, so `is not` cannot false-negative on equal counts.
                fresh = _tx_latest_usage(self.relay)
                if fresh is not None and fresh is not self.usage_before:
                    self.usage = fresh
                    break
        except Exception:
            pass
        self._emit()

    def _emit(self) -> None:
        try:
            mode, engine = _tx_mode(self.relay)
            tools, tools_from = self.ev_tools, self.tools_from
            journal = _tx_journal_slice(self.relay, self.journal_n0)
            # Prefer the journal (it has the ARGUMENTS), but never let it
            # shrink the record: if it somehow saw fewer calls than the event
            # stream did, the event stream is the more complete witness.
            if journal is not None and len(journal) >= len(self.ev_tools):
                tools, tools_from = journal, "journal"
            record = {
                "v": 1,
                "type": "turn",
                "ts": _tx_iso(self.t0),
                "ts_end": _tx_iso(self.t_end or time.time()),
                "sim_t": self.sim_t,
                "robot": getattr(self.bridge, "robot_id", None),
                "bridge": _TX_BRIDGE,
                "source": self.source,
                "prompt": self.prompt,
                "reply": self.reply,
                "mode": mode,
                "engine": engine,
                "latency_s": self.latency_s,
                "tools": tools,
                "tools_from": tools_from,
                "usage": self.usage,
                "status": "error" if self.error else "ok",
            }
            if self.error:
                record["error"] = self.error
            if _TX_WRITER is not None:
                _TX_WRITER.write(record)
        except Exception:
            pass


def _tx_begin(bridge: Any, relay: Any, prompt: str, source: str) -> Optional[Any]:
    """Open a turn. Returns None (and costs nothing) when disabled."""
    if not _TX_ENABLED:
        return None
    try:
        return _TxTurn(bridge, relay, prompt, source)
    except Exception:
        return None


def _tx_end(turn: Optional[Any], reply: str = "", actions: Any = None,
            router_tools: Any = None, error: str = "") -> None:
    """Close a turn opened by _tx_begin. Safe with None, safe on any fault."""
    if turn is None:
        return
    try:
        turn.complete(reply=reply, actions=actions,
                      router_tools=router_tools, error=error)
    except Exception:
        pass


def _tx_window_cb(bridge: Any, relay: Any, prompt: str, source: str,
                  base_cb: Any) -> Any:
    """Wrap the relay's per-turn event callback so the turn is transcribed.

    Returns `base_cb` ITSELF when the transcript is off, so the robot-window
    chat path is byte-identical to what it was before this existed.
    """
    if not _TX_ENABLED:
        return base_cb
    turn = _tx_begin(bridge, relay, prompt, source)
    if turn is None:
        return base_cb

    def _cb(kind: str, payload: Dict[str, Any]) -> None:
        try:
            base_cb(kind, payload)
        finally:
            # Runs even if the chat panel's handler raised, so a broken UI
            # still leaves an auditable record; and it cannot itself raise.
            try:
                turn.observe(kind, payload)
            except Exception:
                pass

    return _cb


# ── Surfacing the relay's AUTO-READS (the grounding gate) ────────────
#
# WHY THIS EXISTS -- MEASURED, 2026-07-29 QA sweep (_scratch/foot_redesign/
# qa_transcript.jsonl). The relay has a grounding gate: when the operator asks
# a state/history question and the model answered without reading the matching
# surface, the relay dispatches that read ITSELF, hands the payload back and
# has the model answer again (relay.py `_reground_if_unread`). It journals the
# call with summary "auto-read (grounding gate)" -- but emits NO "tool" event,
# and both `actions` in the /prompt response and the chat panel's tool lines
# are built from those events alone. So a turn that WAS grounded looks from
# outside exactly like a fabrication: numbers in the reply, no read behind it.
#
# That misreading is what commissioned this change: the QA report filed the
# arm's "how many boxes have you shipped so far? TOOLS: []" as a fabrication,
# while the journal for that turn holds one entry, {"tool":"get_robot_state",
# "summary":"auto-read (grounding gate)"}. The gate had fired. Nothing said so.
#
# The tugs run the same relay and the same gate (tug_a's journal has its own
# auto-read entry), so they need the same reporting. The read genuinely
# happened; this reports it, it does not manufacture it.
def _auto_reads_since(relay: Any, n0: Optional[int]) -> List[Dict[str, Any]]:
    """Journalled tool calls since `n0` that no "tool" event announced."""
    try:
        rows = _tx_journal_slice(relay, n0) or []
        return [r for r in rows
                if str(r.get("summary") or "").startswith("auto-read")]
    except Exception:
        return []


def _autoread_window_cb(bridge: Any, relay: Any, base_cb: Any) -> Any:
    """Wrap the window callback so an auto-read shows up as a tool line.

    Emitted just BEFORE the reply text, which is the order the operator's
    panel already uses for real tool calls. Returns `base_cb` itself when
    there is no journal to read, so the path is unchanged.
    """
    n0 = _tx_journal_seq(relay)
    if n0 is None:
        return base_cb
    state = {"done": False}

    def _cb(kind: str, payload: Dict[str, Any]) -> None:
        if kind in ("agent", "error") and not state["done"]:
            state["done"] = True
            try:
                for row in _auto_reads_since(relay, n0):
                    bridge.queue_window(
                        "tool:%s:%s:%s" % (row.get("name") or "?",
                                           "ok" if row.get("ok") else "err",
                                           row.get("summary") or ""))
            except Exception:
                pass
        base_cb(kind, payload)

    return _cb


def _merge_auto_reads(relay: Any, n0: Optional[int], out: Dict[str, Any]) -> None:
    """Add the turn's auto-reads to an HTTP /prompt `actions` list, in place.

    Called AFTER end_chat_turn so the pause bookkeeping sees exactly the
    action list it always saw (the auto-read tools are read-only anyway).

    THE TWO CASES STAY TELLABLE APART. An auto-read carries
    summary="auto-read (grounding gate)"; a call the MODEL chose to make
    never does. A caller that wants the stricter question -- "did the model
    itself read anything?" -- filters those out; a caller that wants "was
    this answer grounded in a live read at all?" counts them. Before this,
    the second question had no answer and every caller was silently
    answering the first.
    """
    try:
        rows = _auto_reads_since(relay, n0)
        if not rows:
            return
        actions = list(out.get("actions") or [])
        actions += [{"tool": r.get("name"),
                     "result": "ok" if r.get("ok") else "err",
                     "summary": r.get("summary") or ""} for r in rows]
        out["actions"] = actions
    except Exception:
        pass


# ── CLI ──────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--robot", default="husky")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--name", default=None,
                   help=("Override the agent id used in the OmniLink "
                         "profile name and Axis robot_id. Defaults to "
                         "--robot. Use when multiple bases of the same "
                         "kind share a world (e.g. 4 Huskies in a swarm)."))
    p.add_argument("--pallets", default=None,
                   help=("OPT-IN kinematic dock-and-carry: comma-separated "
                         "DEF names of pallet Solids this tug may grab "
                         "(e.g. PALLET_PAYLOAD). Requires supervisor TRUE "
                         "on the Robot node. Omit for zero behaviour "
                         "change (the default)."))
    p.add_argument("--idle-loop", default=None,
                   choices=["dispatch", "trolley_return"],
                   help=("OPT-IN ambient demo loop. Requires --pallets and at "
                         "least three carts. ANY prompt or tool call pauses it "
                         "instantly; it resumes after --idle-resume-s of quiet "
                         "or when the operator calls resume_autonomy. Off by "
                         "default.\n"
                         "'dispatch' + 'trolley_return' are the two HALVES of "
                         "a closed cart ring and are meant to run TOGETHER on "
                         "two tugs. 'dispatch' waits for the line master "
                         "(--idle-arm-port) to report a loaded cart standing "
                         "at DEF CONVEYOR_STATION, tows it to a FREE spot in "
                         "the DEF PARK_SPOT_* row, and collects the oldest "
                         "cart out through DEF CART_OUTBOUND_INFEED when the "
                         "row fills. 'trolley_return' runs the fill conveyor "
                         "both ways and keeps empties flowing in from DEF "
                         "CART_LANE_PICKUP via DEF CART_STAGE."))
    p.add_argument("--idle-arm-port", type=int, default=0,
                   help=("HTTP port of the pick arm's bridge. It is "
                         "the LINE MASTER and its /state 'line' block is the "
                         "authoritative fill/load/ship state both loops key "
                         "off. 0 disables the check."))
    p.add_argument("--idle-peer-port", type=int, default=0,
                   help=("HTTP port of the OTHER tug's bridge. The two line "
                         "loops read each other's /state to keep off the "
                         "shared east-west transit lane at the same time "
                         "(see MavIdleLoop._lane_acquire). 0 disables."))
    p.add_argument("--idle-period", type=float, default=10.0,
                   help="Quiet gap between idle tow cycles (seconds).")
    p.add_argument("--idle-resume-s", type=float, default=60.0,
                   help="Quiet seconds after the last operator command "
                        "before the idle loop resumes (default 60).")
    args, _ = p.parse_known_args()
    return args


# ── Helpers ──────────────────────────────────────────────────────────

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_from_orientation(o) -> float:
    # Webots orientation is a 9-element row-major rotation matrix.
    # Z-up convention: yaw = atan2(o[3], o[0]).
    return math.atan2(o[3], o[0])


# ── Main-thread call marshalling ─────────────────────────────────────

class MainThreadCalls:
    """Marshals supervisor API calls from worker threads onto the sim thread.

    The controller API is not thread-safe: a supervisor pipe call issued
    from a background thread while the main loop is stepping stalls the
    whole controller<->sim step exchange (measured on the warehouse idle
    loop: a couple of threaded reads per second dragged the sim to ~0.2x
    realtime; marshalled, the same reads are free). Workers enqueue a
    closure with ``call``; the bridge's tick() pumps it on the sim thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._q: List[tuple] = []

    def call(self, fn, timeout: float = 6.0):
        ev = threading.Event()
        box: Dict[str, Any] = {}
        with self._lock:
            self._q.append((fn, ev, box))
        ev.wait(timeout)
        return box.get("r")

    def pump(self) -> None:
        with self._lock:
            if not self._q:
                return
            q = self._q
            self._q = []
        for fn, ev, box in q:
            try:
                box["r"] = fn()
            except Exception:
                box["r"] = None
            ev.set()


# ── Sensor read-through (PROTOCOL.md §6.6) ───────────────────────────
#
# WHY THIS LIVES IN THE BRIDGE AND NOT THE HARNESS
# ------------------------------------------------
# `GET /robot/<def>/sensor/<name>` on the World Harness returns **501 by
# design**: OmniSim, like upstream Webots, restricts device APIs to the
# controller that OWNS the device, so a supervisor genuinely cannot read
# another robot's IMU or lidar. The robot's own controller is the only
# honest source, and this bridge is that controller.
#
# THREAD SAFETY IS NOT OPTIONAL HERE
# ----------------------------------
# The controller API is not thread-safe, and a device read issued straight
# from an HTTP worker thread stalls the whole controller<->sim step exchange
# (see MainThreadCalls). Every read below is therefore marshalled onto the
# sim thread with `bridge.mt.call(...)`; nothing in this section may be
# called directly from a request handler.
#
# WARM-UP IS REPORTED, NEVER FAKED
# --------------------------------
# A Webots sensor yields no data until it has been `enable()`d AND at least
# one `robot.step()` has completed since. We enable lazily on first read and
# answer `{"available": true, "value": null, "warming_up": true}` until real
# data arrives. That is deliberately NOT a zero: a fabricated 0.0 reads as
# "the robot is level and stationary", which is a measurement nobody made.

# Devices whose payload is an IMAGE. `/read_sensor` is the scalar/vector
# verb; images have their own verb in PROTOCOL.md §6.7 (`/image`), and
# inlining a frame here would blow the request budget documented in
# packages/omnisim-ros2 (images are by far the heaviest payload).
IMAGE_DEVICE_TYPES = ("Camera", "RangeFinder")

# Actuators and other non-readable devices, listed so the refusal can name
# what the thing actually is instead of a bare "unknown".
NON_SENSOR_DEVICE_TYPES = ("Motor", "RotationalMotor", "LinearMotor", "Brake",
                           "LED", "Display", "Emitter", "Speaker", "Pen",
                           "Connector", "Skin", "Propeller")


def _finite_list(values) -> Optional[List[float]]:
    """Coerce a device reading to a list of floats, or None if unusable.

    Returns None when the device has not produced data yet -- Webots hands
    back None, or a vector of NaN, in that window."""
    if values is None:
        return None
    try:
        out = [float(v) for v in values]
    except (TypeError, ValueError):
        return None
    if not out or not any(math.isfinite(v) for v in out):
        return None
    return out


def describe_sensor(name: str, dev: Any) -> dict:
    """Static description of one device: what it is and what a read returns.

    Published in `/capabilities` so a client discovers the sensor set instead
    of probing for it -- the same reason the action list is published."""
    kind = type(dev).__name__
    entry = {"name": name, "type": kind, "readable": True}
    if kind in IMAGE_DEVICE_TYPES:
        entry["readable"] = False
        entry["note"] = "image device; use PROTOCOL.md 6.7 /image, not /read_sensor"
    elif kind in NON_SENSOR_DEVICE_TYPES:
        entry["readable"] = False
        entry["note"] = "actuator or output device; nothing to read"
    spec = SENSOR_SPECS.get(kind)
    if spec:
        entry["unit"] = spec["unit"]
        entry["shape"] = spec["shape"]
    return entry


# unit/shape are published so a consumer never has to guess what `value`
# means. `shape` is the length of the returned vector, or "n" when it is
# device-configured (a lidar's horizontalResolution x numberOfLayers).
SENSOR_SPECS = {
    "InertialUnit":   {"unit": "quaternion_xyzw", "shape": 4},
    "Gyro":           {"unit": "rad/s", "shape": 3},
    "Accelerometer":  {"unit": "m/s^2", "shape": 3},
    "Compass":        {"unit": "unit_vector", "shape": 3},
    "GPS":            {"unit": "m", "shape": 3},
    "Lidar":          {"unit": "m", "shape": "n"},
    "DistanceSensor": {"unit": "lookup_table", "shape": 1},
    "LightSensor":    {"unit": "lookup_table", "shape": 1},
    "TouchSensor":    {"unit": "lookup_table", "shape": 1},
    "PositionSensor": {"unit": "rad_or_m", "shape": 1},
    "Altimeter":      {"unit": "m", "shape": 1},
    "Radar":          {"unit": "target_list", "shape": "n"},
}


def relative_mount(self_pos, self_rot, dev_pos, dev_rot) -> Optional[dict]:
    """Express a device's world pose in the robot's own frame.

    ROS needs a transform from the robot base to each sensor frame, and the
    only honest source is the scene graph -- a zero offset would be a claim
    that the lidar sits at the robot's origin, which it does not.

        R_rel = R_robot^T . R_dev
        t_rel = R_robot^T . (t_dev - t_robot)

    A rotation matrix is orthonormal, so the transpose IS the inverse.
    Returns None when either pose is unavailable or not yet finite.
    """
    try:
        sp = [float(v) for v in self_pos]
        dp = [float(v) for v in dev_pos]
        sr = [float(v) for v in self_rot]
        dr = [float(v) for v in dev_rot]
    except (TypeError, ValueError):
        return None
    if len(sp) != 3 or len(dp) != 3 or len(sr) != 9 or len(dr) != 9:
        return None
    if not all(math.isfinite(v) for v in sp + dp + sr + dr):
        return None
    d = [dp[i] - sp[i] for i in range(3)]
    t = [sr[0] * d[0] + sr[3] * d[1] + sr[6] * d[2],
         sr[1] * d[0] + sr[4] * d[1] + sr[7] * d[2],
         sr[2] * d[0] + sr[5] * d[1] + sr[8] * d[2]]
    rel = [0.0] * 9
    for i in range(3):
        for j in range(3):
            rel[i * 3 + j] = sum(sr[k * 3 + i] * dr[k * 3 + j] for k in range(3))
    return {"translation": t, "rotation_matrix": rel}


def read_one_sensor(dev: Any, name: str, timestep: int, sim_time: float) -> dict:
    """Read one device. MUST run on the sim thread (see module note above).

    Enables the device on first use; the reading itself only becomes valid on
    the following step, which is reported as `warming_up` rather than zeroed.
    """
    kind = type(dev).__name__
    out = {"available": True, "sensor": name, "type": kind,
           "sim_time": sim_time, "value": None}
    spec = SENSOR_SPECS.get(kind)
    if spec:
        out["unit"] = spec["unit"]

    if kind in IMAGE_DEVICE_TYPES:
        return {"available": False, "sensor": name, "type": kind,
                "note": "image device; use PROTOCOL.md 6.7 /image, not /read_sensor"}
    if kind in NON_SENSOR_DEVICE_TYPES or spec is None:
        return {"available": False, "sensor": name, "type": kind,
                "note": f"{kind} exposes no scalar or vector reading"}

    # Lazy enable. getSamplingPeriod() returns 0 when the device is disabled.
    try:
        if dev.getSamplingPeriod() <= 0:
            dev.enable(max(int(timestep), 1))
            out["warming_up"] = True
            out["note"] = ("sensor enabled on this call; a reading is available "
                           "from the next simulation step")
            return out
    except Exception as exc:
        return {"available": False, "sensor": name, "type": kind,
                "note": f"enable failed: {exc}"}

    try:
        if kind == "InertialUnit":
            vals = _finite_list(dev.getQuaternion())
            out["value"] = vals
            rpy = _finite_list(dev.getRollPitchYaw())
            if rpy:
                out["roll_pitch_yaw"] = rpy
        elif kind in ("Gyro", "Accelerometer", "Compass"):
            out["value"] = _finite_list(dev.getValues())
        elif kind == "GPS":
            out["value"] = _finite_list(dev.getValues())
            speed = dev.getSpeed()
            out["speed"] = float(speed) if speed is not None and math.isfinite(speed) else None
            # 0 == LOCAL (metres in world frame), 1 == WGS84 (lat/lon/alt).
            # A consumer that assumes the wrong one silently mislocates the
            # robot by the whole planet, so it is always stated.
            try:
                out["coordinate_system"] = ("WGS84" if int(dev.getCoordinateSystem()) == 1
                                            else "local")
            except Exception:
                out["coordinate_system"] = "local"
        elif kind == "Lidar":
            ranges = dev.getRangeImage()
            out["value"] = list(ranges) if ranges else None
            out["layout"] = {
                "horizontal_resolution": int(dev.getHorizontalResolution()),
                "number_of_layers": int(dev.getNumberOfLayers()),
                "fov": float(dev.getFov()),
                "vertical_fov": float(dev.getVerticalFov()),
                "min_range": float(dev.getMinRange()),
                "max_range": float(dev.getMaxRange()),
            }
            # ⚠ A no-hit ray reads +inf, which is not valid JSON. The
            # response sanitizer maps every non-finite float to null, so a
            # null entry here means "no return past max_range" -- NOT a
            # missing sample and certainly not a zero-range hit.
            out["no_return_encoding"] = "null"
        else:
            out["value"] = float(dev.getValue())
    except Exception as exc:
        return {"available": False, "sensor": name, "type": kind,
                "note": f"read failed: {exc}"}

    if out.get("value") is None:
        out["warming_up"] = True
        out["note"] = "device enabled but has not produced a sample yet"
    return out


# ── Bridge ───────────────────────────────────────────────────────────

class MobileBridge:
    """Owns wheel motors and pose for one URDF mobile base."""

    # Standing restrictions this robot's autonomy loop GENUINELY enforces.
    # Anything outside this dict is refused by set_constraint -- an agreed
    # rule nobody checks is exactly the failure this layer exists to kill.
    CONSTRAINT_RULES = {
        "no_pick_cell": ("I stay out of the pick-cell column — I will not "
                         "claim it and I will not route a leg through it."),
        "no_new_pickups": ("I start no new tow jobs — I finish what I am "
                           "holding and then stand by."),
        "no_park_row": ("I stay out of the cart park row at the back of the "
                        "building."),
    }
    # Legs an `at_leg:<leg>` trigger may name (the coarse phases the idle
    # loop publishes in /state).
    KNOWN_LEGS = ("idle", "docking", "to_park", "back_aisle", "to_collect",
                  "to_dock_e", "returning", "holding", "column_wait",
                  "shuttle_in", "shuttle_out", "stage_to_station",
                  "lane_fetch", "lane_conveyor")

    def __init__(self, robot: Supervisor, cfg: dict, robot_id: str,
                 pallet_defs: Optional[List[str]] = None) -> None:
        self.robot = robot
        self.cfg = cfg
        self.robot_id = robot_id
        self.timestep = int(robot.getBasicTimeStep())

        layout = cfg["layout"]
        if layout not in WHEEL_MOTORS:
            raise ValueError(f"Unknown wheel layout {layout!r}")
        # Kinematic bodies (single-link URDFs with no wheel joints, e.g. the
        # warehouse AMR tug): the bridge integrates (v, w) each tick and writes
        # the pose via supervisor fields -- the same way the robot's own
        # shipped controllers drive it. Guarded by the config flag so every
        # wheeled base keeps the motor path unchanged.
        self.kinematic = bool(cfg.get("kinematic"))
        self.yaw_offset = float(cfg.get("forward_yaw_offset", 0.0))
        self.floor_z = float(cfg.get("floor_z", 0.0))
        self._kin_cmd = (0.0, 0.0)
        wheel_names = WHEEL_MOTORS[layout]
        self.left_motors = [robot.getDevice(n) for n in wheel_names["left"]]
        self.right_motors = [robot.getDevice(n) for n in wheel_names["right"]]
        missing = [n for n, m in zip(wheel_names["left"] + wheel_names["right"],
                                     self.left_motors + self.right_motors) if m is None]
        if missing:
            print(f"[omnilink_mobile_bridge] WARNING: missing motors: {missing}")
        for m in self.left_motors + self.right_motors:
            if m is not None:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)

        # Optional init poses -- tuck arms, lower torso, etc. for
        # robots whose default URDF spawn pose looks like a failure
        # (mobile-manipulator robots with extended arms, etc.).
        for joint_name, q in (cfg.get("init_poses") or {}).items():
            m = robot.getDevice(joint_name + "_motor") or robot.getDevice(joint_name)
            if m is not None:
                try:
                    m.setPosition(q)
                except Exception as e:
                    print(f"[omnilink_mobile_bridge] init_pose {joint_name}={q} failed: {e}")

        self.r = cfg["wheel_radius_m"]
        self.ht = cfg["half_track_m"]
        self.v_max = cfg["max_wheel_speed_radps"]
        self.v_max_linear = self.v_max * self.r
        self.v_max_angular = self.v_max * self.r / self.ht

        self.cruise_linear = cfg["cruise_frac"] * self.v_max_linear
        self.spin_speed = cfg["spin_speed"]

        # Pose tracking.
        self.self_node = robot.getSelf()
        self.start_xyz = list(self.self_node.getPosition())
        self.start_yaw = yaw_from_orientation(self.self_node.getOrientation())
        self.last_xy = (self.start_xyz[0], self.start_xyz[1])
        self.last_yaw = wrap_pi(self.start_yaw - self.yaw_offset)
        self.v_linear = 0.0
        self.v_angular = 0.0

        # Motion state machine.
        self.lock = threading.RLock()
        # ("idle", {}) | ("velocity", {"l", "a"}) | ("drive", {...}) | ("turn", {...})
        self.motion = ("idle", {})
        # Adaptive stop-rollback compensation, learned online from each
        # settled drive (see the settle-and-verify constants block).
        self._drive_bias = 0.0    # m of in-motion pose lead to aim past
        # Learned yaw rate achieved / commanded. 0.55 is a SKID-STEER seed:
        # on the Husky under Newton a commanded pivot delivers a fraction of
        # the commanded rate, so starting low under-shoots safely.
        #
        # A KINEMATIC base is the opposite case. `_tick` integrates the
        # commanded w straight into the pose write (see `if self.kinematic`
        # below), so achieved == commanded EXACTLY and the true gain is 1.0.
        # Seeding 0.55 there tells the planner a pulse buys 55% of what it
        # really does, so the pulse is sized 1.8x too long -- and once a
        # single pulse passes pi, the settled measurement below
        # (wrap_pi(yaw - pulse_y0)) reads it back NEGATED and `remaining`
        # GROWS. The loop still converges; it walks the long way round.
        #
        # MEASURED LIVE on tug_a mid-dock, before this change: 1294.3 deg of
        # rotation in 30 s -- 3.60 revolutions -- for 73.9 deg of net
        # progress. Efficiency |net|/total = 0.057, i.e. 94% of the turning
        # was wasted, and it was still turning when the sample ended.
        self._turn_gain = 1.0 if self.kinematic else 0.55
        # SETTLE. The pulse-and-settle design exists because a skid-steer's
        # IN-MOTION yaw readback is wrong, so the loop commands zero and waits
        # for the chassis to unwind before believing a number. A kinematic
        # base has nothing to unwind -- this controller wrote the pose itself
        # and the readback is exact on the same tick. Holding zero for a full
        # second there measures nothing and is simply dead time the operator
        # watches. Two ticks is enough for the pose write to land.
        if self.kinematic:
            self.TURN_SETTLE_S = 0.064
            self.DRIVE_SETTLE_S = 0.064
        # ACTION RESULT CONTRACT (PROTOCOL.md 5.4.1). Every motion carries a
        # monotonic seq; the tick writes what ACTUALLY happened here when the
        # motion ends. `accepted: true` plus an echo of the caller's own
        # argument is not a result -- it is the caller's request handed back,
        # and an agent has no way to tell the difference.
        self.motion_seq = 0
        self.last_completion: Optional[dict] = None
        self.fault: Optional[str] = None
        self.last_tick_at = time.time()

        # Idle-loop bookkeeping (see MavIdleLoop). last_external_cmd is the
        # wall-clock time of the last operator command; the opt-in idle loop
        # pauses while it is recent. _carries holds the in-flight CART
        # CONVEYOR runs (see start_cart_conveyor) -- a list, not a slot,
        # because one bridge can own more than one conveyor.
        self.last_external_cmd = 0.0
        self.last_external_src = ""
        self._resume_exempt_until = 0.0
        # AN OPERATOR STOP HOLDS THE ROBOT FROM THE MOMENT IT EXECUTES.
        # The quiet window above is armed when the operator's PROMPT arrives
        # (HTTP _route_post / handle_wwi_message), not when the model finally
        # picks a tool -- so with --idle-resume-s 12 and an LLM turn longer
        # than that, the window had already lapsed by the time stop_robot ran
        # and the idle loop was free to drive on. MEASURED on tug_a: 1.5 s
        # after a "stop" that answered "I have halted immediately", the tug
        # had moved 0.384 m and turned 34.7 deg, mode=turn, v_angular=-0.90,
        # idle_loop.leg=to_park, paused=False. This deadline is armed by
        # act_stop ITSELF and read by MavIdleLoop._blocked.
        self.stop_hold_until = 0.0
        # Pose samples taken by tick() (wall_t, x, y, yaw). This is the ONLY
        # honest basis for "did it actually come to rest": the base is
        # kinematic, so _command_velocity just stores a command that tick()
        # integrates into the pose write -- a commanded velocity of zero says
        # nothing about whether the pose stopped changing.
        self._pose_hist: List[Tuple[float, float, float, float]] = []
        self.idle_mode: Optional[str] = None
        self.idle_loop: Optional["MavIdleLoop"] = None
        self._carries: List[dict] = []
        self.mt = MainThreadCalls()

        # ── SENSOR READ-THROUGH (PROTOCOL.md §6.6) ───────────────────
        # Discovered once, at construction, from the robot's own device
        # table. Nothing is enabled here: enabling a lidar costs a render
        # every tick whether or not anyone reads it, so devices warm up
        # lazily on their first /read_sensor (see read_one_sensor).
        #
        # ⚠ A URDF robot has NO devices unless the world was loaded with
        # OMNISIM_URDF_USE_SENSORS=1 -- the importer parses <gazebo> sensor
        # blocks always but DROPS them at emit time when that is unset.
        # Measured on the shipped Husky: 0 devices without it, 5 with it.
        # An empty list here is that, far more often than a robot that
        # genuinely carries nothing.
        self.sensor_devices: Dict[str, Any] = {}
        try:
            for i in range(robot.getNumberOfDevices()):
                dev = robot.getDeviceByIndex(i)
                if dev is None:
                    continue
                dev_name = dev.getName()
                if type(dev).__name__ in NON_SENSOR_DEVICE_TYPES:
                    continue
                self.sensor_devices[dev_name] = dev
        except Exception as e:
            print(f"[omnilink_mobile_bridge] sensor discovery failed: {e}")
        self.sensor_catalog = [describe_sensor(n, d)
                               for n, d in sorted(self.sensor_devices.items())]
        # Measured lazily on the sim thread: poses are NaN until the first
        # robot.step() completes, and the HTTP server is up before that.
        self._sensor_mounts: Optional[Dict[str, Any]] = None
        if self.sensor_catalog:
            print(f"[omnilink_mobile_bridge] sensors: "
                  f"{', '.join(s['name'] + ':' + s['type'] for s in self.sensor_catalog)}")
        else:
            print("[omnilink_mobile_bridge] no readable sensors on this robot "
                  "(a URDF robot needs OMNISIM_URDF_USE_SENSORS=1 at world load)")

        # ── DEFERRED INTENTS ─────────────────────────────────────────
        # Every other tool on this bridge is IMMEDIATE, so "stop after you
        # park this cart" / "don't enter the pick cell until I say so" had
        # nowhere to live: the model promised in prose, the turn ended, and
        # the idle loop resumed 60 s later as if nothing had been said. This
        # store is where a conditional order survives the turn. See
        # omnisim_bridges.intents.
        self.intents = (
            IntentStore(
                robot_id,
                task_noun="delivery", task_plural="deliveries",
                conditions=("after_current_task", "after_n_deliveries",
                            "on_next_pickup", "at_leg"),
                rules=dict(self.CONSTRAINT_RULES),
                legs=list(self.KNOWN_LEGS),
                on_pause=self._on_intent_pause,
                on_notify=self._on_intent_notify,
            ) if (IntentStore is not None
                 # A/B kill-switch, same shape as OMNILINK_AVOID=0:
                 # OMNILINK_INTENTS=0 reverts this bridge to the
                 # pre-deferred-intent behaviour EXACTLY -- no store,
                 # no scheduling tools, no prompt rule, no /state
                 # block, no hold. Used to A/B the baseline honestly.
                 and _os.environ.get("OMNILINK_INTENTS", "1")
                 .strip() not in ("0", "false", "no")) else None)

        # wwi outbox.
        self.window_outbox: List[str] = []
        self.window_configured = False

        self.capabilities = {
            "layout": layout,
            "wheel_radius_m": self.r,
            "half_track_m": self.ht,
            "max_linear_m_s": self.v_max_linear,
            "max_angular_rad_s": self.v_max_angular,
            "cruise_linear_m_s": self.cruise_linear,
            "set_velocity_max_s": self.VELOCITY_MAX_S,
            # PROTOCOL.md 5.3: the verb set, discoverable rather than guessed.
            # Without this an agent 404-probes the surface and each miss costs
            # it a turn.
            "actions": ["drive_to", "drive_forward", "turn", "set_velocity",
                        "stop_robot", "reset_to_home", "get_robot_state"],
            # PROTOCOL.md 5.4.1: which verbs finish before they answer, and
            # which hand back a promise. Stated, not implied.
            "blocking_actions": ["drive_to"],
            "waitable_actions": ["drive_forward", "turn"],
            # PROTOCOL.md 5.4.1 rule 5: which verbs REJECT a second command
            # while one is in flight, and which override it instead. Both
            # behaviours are deliberate -- a `stop` that answers 409 is
            # useless -- so neither should have to be guessed. An overriding
            # verb cancels the running motion, whose `achieved` then reports
            # null, because nobody measured where it got to.
            "busy_rejecting_actions": ["drive_to", "drive_forward", "turn"],
            "busy_overriding_actions": ["stop_robot", "set_velocity"],
            # The fence is ENFORCED (see _fence_guard) -- so it is published.
            # An agent cannot plan inside a bound it cannot read.
            "site_bounds_m": {"half_x": self.SITE_HALF_X,
                              "half_y": self.SITE_HALF_Y},
            # PROTOCOL.md §6.6: the sensor set, discoverable rather than
            # probed. An empty list is a real answer ("this robot carries
            # nothing readable"), which is why the key is always present.
            "sensors": self.sensor_catalog,
        }
        if self.sensor_catalog:
            self.capabilities["actions"] = (self.capabilities["actions"]
                                            + ["read_sensor"])

        # ── OPTIONAL kinematic dock-and-carry (pallet tug) ────────────
        # Active ONLY when (a) the world opts in with --pallets <DEF[,DEF]>
        # AND (b) the Robot node has supervisor powers (getFromDef resolves
        # the named pallet). Demos without --pallets see ZERO change: the
        # feature flag stays False, no tools registered, no state key, no
        # per-tick work.
        self.pallet_defs: List[str] = [d.strip() for d in (pallet_defs or [])
                                       if d and d.strip()]
        self.pallet_feature = False
        self.carrying: Optional[str] = None      # DEF currently towed, or None
        self._carry: Optional[dict] = None       # trailer state (under lock)
        if self.pallet_defs:
            resolved = []
            try:
                for d in self.pallet_defs:
                    n = robot.getFromDef(d)
                    if n is not None:
                        resolved.append(d)
            except Exception as e:
                print(f"[omnilink_mobile_bridge] --pallets disabled "
                      f"(supervisor lookup failed: {e})")
                resolved = []
            if resolved:
                self.pallet_defs = resolved
                self.pallet_feature = True
                self.capabilities["trolley_tow"] = {
                    "trolleys": list(resolved),
                    "dock_radius_m": 0.6,
                }
                print(f"[omnilink_mobile_bridge] trolley dock-and-tow ON "
                      f"for {resolved}")
            else:
                print("[omnilink_mobile_bridge] --pallets given but no DEF "
                      f"resolved from {self.pallet_defs} (missing node or "
                      "no supervisor) -- pallet tools stay OFF")

    # ── Pose / velocity readout ───────────────────────────────────

    def _read_pose(self) -> Tuple[float, float, float]:
        p = self.self_node.getPosition()
        yaw = yaw_from_orientation(self.self_node.getOrientation())
        # Heading = direction of travel. For bodies whose mesh forward axis
        # is not local +X (the AMR tug: forward is +Y, yaw_offset -pi/2), the
        # node yaw and the heading differ by the fixed offset.
        return p[0], p[1], wrap_pi(yaw - self.yaw_offset)

    # ── Wheel commanding ──────────────────────────────────────────

    def _command_velocity(self, linear: float, angular: float) -> Tuple[float, float]:
        """Convert (linear m/s, angular rad/s) to (left rad/s, right rad/s),
        apply, return clamped values. Kinematic bodies store the command;
        tick() integrates it into a supervisor pose write."""
        linear = clamp(linear, -self.v_max_linear, self.v_max_linear)
        angular = clamp(angular, -self.v_max_angular, self.v_max_angular)
        if self.kinematic:
            self._kin_cmd = (linear, angular)
            return 0.0, 0.0
        # Skid-steer / diff-drive kinematics.
        v_left = (linear - angular * self.ht) / self.r
        v_right = (linear + angular * self.ht) / self.r
        max_abs = max(abs(v_left), abs(v_right), 1e-9)
        if max_abs > self.v_max:
            scale = self.v_max / max_abs
            v_left *= scale
            v_right *= scale
        for m in self.left_motors:
            if m is not None:
                m.setVelocity(v_left)
        for m in self.right_motors:
            if m is not None:
                m.setVelocity(v_right)
        return v_left, v_right

    def queue_window(self, line: str) -> None:
        with self.lock:
            self.window_outbox.append(line)

    # ── deferred-intent hooks ─────────────────────────────────────

    def _on_intent_pause(self, hold: bool, intent: dict) -> None:
        """A scheduled pause reached its trigger. Called from the idle-loop
        thread at a clean task boundary (or from HTTP for hold_until_told).

        The HOLD itself lives in the store and is read by
        MavIdleLoop._blocked; this only has to make the robot stand still and
        -- for a soft pause -- arm the ordinary quiet window."""
        with self.lock:
            self.motion = ("idle", {})
        self._command_velocity(0.0, 0.0)
        if not hold:
            self.last_external_cmd = time.time()
            self.last_external_src = "intent:" + str(intent.get("id", "?"))
        self.queue_window(
            "agent:" + ("Holding as you asked — I will not resume on my own."
                        if hold else "Pausing as you asked."))

    def _on_intent_notify(self, text: str, intent: dict) -> None:
        """A notify intent fired. There is no push channel to the operator,
        so the honest surfaces are the robot window, the idle log, and
        /state.notifications (which get_robot_state tells the model to read
        back). Nothing here pretends a message was delivered."""
        self.queue_window(f"agent:[{intent.get('id', '?')}] {text}")

    def note_external_command(self, source: str) -> None:
        """Record an operator command (chat prompt / HTTP tool call). The
        opt-in idle loop pauses INSTANTLY while this is recent and resumes
        after its quiet window. Cheap no-op when no idle loop is running."""
        if time.time() < getattr(self, "_resume_exempt_until", 0.0):
            # A resume landed in this same turn. Anything that arrives after
            # it (the relay finishing the turn, a trailing status poll) must
            # not silently re-arm the pause the operator just lifted.
            # TIME-BOXED on purpose: this used to be a sticky boolean that
            # stayed armed until the next command whenever it came, so a
            # "stop" minutes after a "carry on" was swallowed -- the tug
            # halted, failed to arm the pause, and the idle loop drove it
            # away again. The exemption only has to outlive its own turn.
            self._resume_exempt_until = 0.0
            return
        self.prev_external_cmd = self.last_external_cmd
        self.last_external_cmd = time.time()
        self.last_external_src = source

    # Tools that only READ. A chat turn that used nothing else was a
    # question, not a command, and must not park the tug.
    READ_ONLY_TOOLS = frozenset({
        "get_robot_state", "list_robots", "capabilities", "get_state",
        # Asking what the OTHER tug is doing, or how much longer this one
        # will be, is a question. Parking the line for 60 s over it would be
        # the same bug as parking it for "what are you doing?".
        "get_peer_state", "estimate_time_remaining",
    })

    # SCHEDULING tools are deliberately grouped with the reads here. Arming
    # the 60 s pause on "stop AFTER you park this cart" would stop the tug
    # NOW -- which is precisely the deferral the operator did not ask for.
    # The intent's own trigger is what pauses it later. hold_until_told is
    # deliberately NOT in here: it means "stop now, and stay stopped".
    SCHEDULING_TOOLS = DEFERRED_TOOLS

    # Tools whose whole job is to LIFT the pause. Never re-armed by the
    # bookkeeping below.
    RESUME_TOOLS = frozenset({"resume_autonomy", "resume_idle_loop"})

    def end_chat_turn(self, prev: tuple, actions: list) -> None:
        """Undo the idle-loop pause when the turn only read state.

        "What are you doing?" used to park the tug for the full 60 s quiet
        window and then report itself as PAUSED -- an answer that was true
        only because asking made it true. Commands still pause; questions no
        longer do. resume_autonomy is deliberately NOT listed as read-only:
        it clears the marker itself, and rolling back would re-arm the very
        pause it just lifted."""
        names = set()
        for a in (actions or []):
            if isinstance(a, dict):
                names.add(str(a.get("tool") or ""))
            elif isinstance(a, (list, tuple)) and a:
                names.add(str(a[0]))
        if names & self.RESUME_TOOLS:
            # The resume already cleared the marker on purpose. Leave it
            # alone -- rolling back OR re-arming would both undo it.
            return
        if not (names - self.READ_ONLY_TOOLS - self.SCHEDULING_TOOLS):
            # Pure question, or a turn that only SCHEDULED something: roll
            # the pause back to where it was. A scheduling turn must leave
            # the robot working -- that is the entire point of deferring.
            self.last_external_cmd, self.last_external_src = prev
            return
        # Commanding turn: the pause MUST end up armed. Forcing it here
        # (rather than relying on the blanket arm upstream) closes a real
        # hole -- a command landing inside the post-resume exemption window
        # was silently exempted, so the tug executed the order while its
        # idle loop kept running and immediately fought it.
        self._resume_exempt_until = 0.0
        self.last_external_cmd = time.time()
        self.last_external_src = "cmd:" + ",".join(sorted(names))

    def get_state_for_query(self) -> dict:
        """/state as it was the instant the operator ASKED.

        The HTTP layer arms the idle-loop pause before the prompt is even
        routed, so a bare get_state() inside a status answer reports the
        tug as paused by the very question being asked -- true, but true
        only because you asked. This reports the pause as it stood before
        this turn; everything else is live."""
        st = self.get_state()
        loop_obj = getattr(self, "idle_loop", None)
        loop = st.get("idle_loop")
        if isinstance(loop, dict) and loop_obj is not None:
            prev = getattr(self, "prev_external_cmd", 0.0)
            # A HOLD is not an artefact of the question -- it is a standing
            # operator order, and rolling it back would let the model report
            # "running" while the robot is deliberately stopped.
            held = (self.intents is not None and self.intents.hold_active())
            # NEITHER IS A STOP THIS TURN ISSUED. The roll-back exists so
            # "what are you doing?" does not report a pause it caused by
            # being asked; applying it to a stop made the model read
            # paused=false immediately after halting the robot itself, which
            # is the reporting half of the same defect act_stop fixes.
            stopped = time.time() < getattr(self, "stop_hold_until", 0.0)
            loop["paused"] = (held or stopped
                              or (time.time() - prev) < loop_obj.resume_s)
            loop["paused_by"] = (
                "operator hold" if held else
                "operator stop" if stopped else
                ("recent command" if loop["paused"] else ""))
            if stopped:
                loop["resumes_in_s"] = round(
                    self.stop_hold_until - time.time(), 1)
        return st

    def describe_role(self) -> str:
        """Honest answer to "what is your job on this line?".

        Derived from the idle loop this tug was actually configured with
        and its live state, so the offline router never invents a role.
        """
        model = self.cfg.get("model", "mobile robot")
        who = self.robot_id
        loop = self.idle_loop
        if loop is None:
            return (f"I'm {who}, a {model} on manual control — no standing "
                    f"job on this line, I just run what you send me.")
        mode = getattr(loop, "mode", "?")
        cart = self.carrying
        holding = (f" I'm towing {cart} right now." if cart
                   else " I'm not towing anything at this moment.")
        if mode == "dispatch":
            job = ("I'm the dispatch tug: when the pick cell finishes kitting "
                   "a box onto a cart, I hook up to that loaded cart and haul "
                   "it from the fill stop out to the dispatch bay, then drop "
                   "it there")
        elif mode == "trolley_return":
            job = ("I'm the return tug: I collect empty carts from the park "
                   "row and bring them back to the fill stop so the pick cell "
                   "always has somewhere to put the next box")
        else:
            job = f"I run the '{mode}' loop on this line"
        # CARTS, NOT CYCLES. This sentence used to end "N cycles done so far",
        # and `cycles` is the count of UNINTERRUPTED loop closures: an
        # operator command aborts a cycle wherever it lands, so a tug that had
        # demonstrably parked a cart could still say "0 cycles done so far" to
        # the operator who had just watched it (measured, show2_idle.txt:
        # PARKED at t=201.7, cycles still 0 at t=247.0). Carts delivered is
        # both the number the operator asked for and one that only moves when
        # a cart physically arrived somewhere.
        n = 0
        try:
            n = loop._carts_delivered()
        except Exception:
            n = getattr(loop, "parks_total", 0)
        return (f"I'm {who}, a {model} tug. {job}. "
                f"{n} cart(s) delivered so far this session.{holding}")

    def act_resume_autonomy(self) -> dict:
        """Hand the robot back to its idle loop NOW.

        Without this there is no way to say "carry on" and be believed: the
        prompt that asks for it is itself an operator command, so it refreshes
        the pause window and the robot stays parked while the model reports
        that it has resumed. Clearing the marker (rather than merely not
        setting it) is the whole point -- and _resume_exempt_until tells the HTTP
        layer not to re-arm the pause on the way back out of this call."""
        self._resume_exempt_until = time.time() + 1.5
        with self.lock:
            self.motion = ("idle", {})
        self._command_velocity(0.0, 0.0)
        self.last_external_cmd = 0.0
        self.last_external_src = ""
        # ...and the only thing that lifts an operator STOP early. act_stop
        # arms stop_hold_until for a full minute, so "carry on" has to clear
        # it here or the robot would sit out the rest of the hold while the
        # model reported that it had resumed.
        self.stop_hold_until = 0.0
        # THE ONLY THING THAT LIFTS A HOLD. hold_until_told exists precisely
        # so the 60 s quiet window cannot silently override the operator, so
        # clearing the marker above is not enough -- the hold has to be
        # released explicitly, here.
        released = False
        if self.intents is not None:
            released = bool(self.intents.release_hold("resume_autonomy")
                            .get("released"))
        loop = self.idle_loop
        if loop is None:
            return {"accepted": True, "autonomy": "none",
                    "hold_released": released,
                    "detail": "this robot has no idle loop to resume"}
        return {"accepted": True, "autonomy": "resumed",
                "hold_released": released,
                "mode": getattr(loop, "mode", "?"),
                "leg": getattr(loop, "leg", "idle"),
                "cycles": getattr(loop, "cycles", 0)}

    # ── Site fence ────────────────────────────────────────────────
    # A chat turn must not be able to drive a robot out of the building. An
    # open-ended set_velocity is the dangerous one: it keeps running until
    # something else lands, and a model that says "head over there" with no
    # distance has produced exactly that. Measured failure: one turn put a tug
    # at x=-21.5 -- 6 m outside a +/-15 m arena, through a wall.
    SITE_HALF_X = 14.4
    SITE_HALF_Y = 8.4
    VELOCITY_MAX_S = 12.0     # s an open-ended velocity command may run

    # ── Closed-loop drive/turn: settle-and-verify ─────────────────
    # Under the Newton/MuJoCo solver the supervisor pose read while the
    # base is MOVING leads the settled truth by a large, speed-dependent
    # margin (~0.24 m at cruise, ~0.33 m at 0.1 m/s on the Husky): the
    # instant the wheels are zeroed the pose snaps back. A loop that
    # stops the moment the in-motion pose reaches the target therefore
    # lands short by that lead — measured −21% on a 1 m Husky drive,
    # −10.6% on 2 m, −18.7% on a 90° turn (constant absolute shortfall,
    # NOT proportional). Fix, all supervisor-pose closed-loop:
    #   1. APPROACH — drive toward target + bias with a proportional
    #      slow-down (never below APPROACH_MIN, where the lead is worst);
    #   2. SETTLE — command zero and wait; the settled pose is truth;
    #   3. VERIFY — re-measure, learn the stop rollback into the bias,
    #      and re-issue the signed residual (capped count) if outside
    #      tolerance.
    # The bias starts at ZERO so solvers/robots without the lead see no
    # behaviour change beyond the settle-verify pass; it is learned from
    # each settled error, so later commands land in one shot.
    DRIVE_APPROACH_GAIN = 1.5    # m/s commanded per m of remaining error
    DRIVE_APPROACH_MIN = 0.25    # m/s floor (pose-lead grows at low speed)
    DRIVE_BRAKE_M = 0.02         # stop when |remaining| is inside this
    DRIVE_TOL_MIN_M = 0.03       # settled-error tolerance floor …
    DRIVE_TOL_FRAC = 0.02        # … or 2% of the commanded distance
    DRIVE_SETTLE_S = 1.0         # sim-s of zero command before re-measuring
    DRIVE_MAX_CORRECTIONS = 4
    DRIVE_BIAS_MAX_M = 0.8
    # TURNS are settle-and-verified too, on the same three-phase shape.
    #
    # History, because the previous note here was wrong twice and it cost
    # the agent benchmark a whole lane. It claimed (a) the undershoot was
    # "~−19%" and (b) an aim-past corrector "limit-cycled" and so could
    # not converge. Re-measured 2026-07-26 on husky_swarm (Newton/MuJoCo,
    # RTX 3060 laptop), 4 trials per angle, reproducible to 4 decimals:
    #
    #     commanded  +90° → achieved +51.0°   (ratio 0.567)
    #     commanded  −90° → achieved −51.0°   (ratio 0.567)
    #     commanded  +45° → achieved +32.0°   (ratio 0.710)
    #
    # So the real figure is −43% on a 90° turn, not −19%. And the snap-back
    # is NOT erratic — it is deterministic. What actually limit-cycled was
    # the MINIMUM SPIN FLOOR: the old loop forced |w| ≥ 0.4 rad/s even for a
    # residual of a few degrees, against a 0.04 rad completion threshold, so
    # every small correction overshot and the loop oscillated
    # 51° → 77° → 109° → 77° → 109°. The floor was the bug, not the corrector.
    #
    # Two corrector shapes were built and measured before this one, and both
    # failed for the same underlying reason — they decided when to stop by
    # reading the yaw WHILE PIVOTING, which is the signal that is wrong:
    #
    #   error-based closed-loop spin, no aim-past  → 90° landed 66-82°,
    #       timed out on 180°, and OVERSHOT 30° by 18.8°;
    #   the same, plus an additive learned aim-past bias → small turns came
    #       back at ratio 1.03/1.12 while large ones still undershot, because
    #       the lead is PROPORTIONAL to the pivot, not a fixed offset.
    #
    # What converges is pulse-and-settle: never read yaw in motion at all.
    # Hold a constant rate open-loop for a computed duration, stop, let the
    # chassis unwind, and believe only the settled-to-settled delta. The rate
    # gain (achieved ÷ commanded) is learned from that delta and converges to
    # ~0.17 on the Husky. Measured client-side before porting: +89.75° /
    # −90.17° / +135.79° against goals of ±90° / +135°, in 4–5 pulses.
    TURN_TOL_RAD = 0.0175        # settled-error tolerance (1°)
    TURN_SETTLE_S = 1.0          # sim-s of zero command before re-measuring
    # LONGEST SINGLE PULSE IN RADIANS -- a correctness bound, not a tuning
    # knob. The settle phase measures what a pulse delivered with
    #     stepped = wrap_pi(yaw - pulse_y0)
    # and wrap_pi CANNOT represent a rotation larger than pi. A pulse that
    # over-delivers past 180 deg comes back NEGATED, so `remaining` grows
    # instead of shrinking and the same inverted sample poisons the learned
    # gain. Capping one pulse below pi keeps that measurement valid BY
    # CONSTRUCTION whatever the gain error: a half-turn simply costs two
    # pulses. Verified by replay against the real constants -- with the cap,
    # every commanded angle lands exact-to-tolerance; and even with a
    # deliberately WRONG 0.55 seed still in place the worst case degrades to
    # one extra pulse instead of an unbounded spin. So correctness here does
    # not depend on the gain seed being right.
    # 0.9*pi leaves room for the gain being under-estimated (which makes the
    # pulse LONGER than planned) plus the one-tick overshoot past
    # pulse_end_sim.
    TURN_PULSE_MAX_RAD = 0.9 * math.pi
    TURN_PULSE_MAX_S = 15.0      # longest single open-loop pulse
    TURN_SLOW_RAD = 0.25         # below this residual, pulse at TURN_SLOW_W
    TURN_SLOW_W = 0.25           # rad/s for the fine pulses
    TURN_MAX_CORRECTIONS = 10
    # Typical learned gain, used ONLY to size the time budget before the real
    # gain is known. A skid-steer pivot delivers ~0.17 of the commanded rate,
    # so a half-turn needs ~30 sim-s of actual spinning: budgeting from the
    # commanded rate instead is what made 180° and 270° both stop dead at
    # 138° -- the same number for both, which is the signature of a timeout
    # rather than a control failure.
    TURN_GAIN_TYPICAL = 0.17
    # drive_to: the compound verb. Exists so the model never has to compose a
    # rotation with a translation, or call atan2 -- the two things LLMs are
    # measurably worst at and the tool is exactly correct at.
    DRIVE_TO_TOL_M = 0.10
    DRIVE_TO_HEADING_TOL = 0.03    # rad; below this, skip the turn leg
    DRIVE_TO_MAX_LEGS = 3
    TURN_GAIN_MIN = 0.08
    TURN_GAIN_MAX = 1.5

    def _site_clamped(self, x: float, y: float) -> bool:
        return (abs(x) > self.SITE_HALF_X or abs(y) > self.SITE_HALF_Y)

    def _fence_guard(self) -> Optional[str]:
        """Stop and report if the base has left the site. Fail-safe, not
        fail-silent: the operator is told, because a clamped command means the
        plan was wrong, not that the robot did well."""
        x, y, _ = self._read_pose()
        if not self._site_clamped(x, y):
            return None
        with self.lock:
            self.motion = ("idle", {})
        self._command_velocity(0.0, 0.0)
        return (f"halted at ({x:+.1f},{y:+.1f}): outside the site bounds "
                f"(|x|<={self.SITE_HALF_X}, |y|<={self.SITE_HALF_Y})")

    # ── Tick loop ─────────────────────────────────────────────────

    def tick(self, dt_s: float) -> None:
        # Run any supervisor calls the idle loop marshalled to this thread.
        self.mt.pump()
        x, y, yaw = self._read_pose()
        dx = x - self.last_xy[0]
        dy = y - self.last_xy[1]
        if dt_s > 1e-4:
            self.v_linear = math.cos(yaw) * dx / dt_s + math.sin(yaw) * dy / dt_s
            self.v_angular = wrap_pi(yaw - self.last_yaw) / dt_s
        self.last_xy = (x, y)
        self.last_yaw = yaw
        self.last_tick_at = time.time()
        # POSE HISTORY for act_stop's rest measurement. Appended here, on the
        # sim thread, so a stop arriving on an HTTP or relay thread can prove
        # the robot stood still WITHOUT issuing a supervisor read of its own
        # (MainThreadCalls: threaded reads drag the sim to ~0.2x realtime).
        with self.lock:
            self._pose_hist.append((self.last_tick_at, x, y, yaw))
            if len(self._pose_hist) > self.POSE_HIST_MAX:
                del self._pose_hist[:-self.POSE_HIST_MAX]

        # SITE FENCE — checked before any motion is applied, so no command of
        # any origin (chat, HTTP, idle loop) can leave the building.
        if kind_fence := self._fence_guard():
            self.queue_window("error:" + kind_fence)
            print(f"[omnilink_mobile_bridge] {kind_fence}", flush=True)

        with self.lock:
            kind, p = self.motion
        if kind == "idle":
            self._command_velocity(0.0, 0.0)
        elif kind == "velocity":
            # Open-ended velocity commands EXPIRE. "Keeps moving until another
            # command lands" is fine for a teleop joystick and wrong for an
            # LLM turn that may never send a second command.
            if (self.robot.getTime() - p.get("t0_sim", 0.0)
                    > self.VELOCITY_MAX_S):
                with self.lock:
                    self.motion = ("idle", {})
                self._command_velocity(0.0, 0.0)
                self.queue_window(
                    f"system:velocity command expired after "
                    f"{self.VELOCITY_MAX_S:.0f}s")
            else:
                self._command_velocity(p["l"], p["a"])
        elif kind == "drive":
            # Drive forward / backward a target distance, CLOSED-LOOP with
            # settle-and-verify (see the constants block for the measured
            # in-motion pose lead this compensates). The timeout budget is
            # measured in SIM seconds (falling back to wall clock for plans
            # created before the field existed): a wall-clock timeout silently
            # truncated motions whenever the sim ran below realtime.
            travelled = math.sqrt((x - p["x0"]) ** 2 + (y - p["y0"]) ** 2)
            timed_out = ((self.robot.getTime() - p["t0_sim"]) > p["timeout_s"]
                         if "t0_sim" in p else
                         (time.time() - p["t0"]) > p["timeout_s"])
            direction = 1.0 if p["speed"] >= 0 else -1.0
            if p.get("phase") == "settle":
                self._command_velocity(0.0, 0.0)
                if ((self.robot.getTime() - p["settle_t0"])
                        >= self.DRIVE_SETTLE_S or timed_out):
                    err = p["distance"] - travelled   # settled truth; +: short
                    tol = max(self.DRIVE_TOL_MIN_M,
                              self.DRIVE_TOL_FRAC * p["distance"])
                    # Learn the stop rollback from the settled truth.
                    self._drive_bias = clamp(self._drive_bias + 0.8 * err,
                                             0.0, self.DRIVE_BIAS_MAX_M)
                    if (abs(err) > tol and not timed_out
                            and p.get("corrections", 0)
                            < self.DRIVE_MAX_CORRECTIONS):
                        with self.lock:
                            p2 = dict(p)
                            p2["phase"] = "approach"
                            p2["corrections"] = p.get("corrections", 0) + 1
                            p2["timeout_s"] = p["timeout_s"] + 3.0
                            self.motion = ("drive", p2)
                    else:
                        with self.lock:
                            self.motion = ("idle", {})
                        # ACHIEVED IS MEASURED, SIGN INCLUDED. `travelled`
                        # is a magnitude, so copysign(travelled, commanded)
                        # ASSERTED the direction instead of measuring it: a
                        # robot shoved backwards, or dragged sideways,
                        # reported positive forward progress, and a
                        # direction inversion was undetectable by
                        # construction. Project the displacement onto the
                        # heading the drive STARTED on -- the axis the
                        # command was expressed in. (The husky bridge has
                        # always done it this way.)
                        yaw0 = p.get("yaw0")
                        if yaw0 is None:
                            achieved = math.copysign(
                                travelled, p.get("commanded", 1.0) or 1.0)
                        else:
                            achieved = ((x - p["x0"]) * math.cos(yaw0)
                                        + (y - p["y0"]) * math.sin(yaw0))
                        self._record_completion(
                            p, achieved,
                            settled=not timed_out, timed_out=timed_out)
                        self.queue_window(
                            f"system:drive complete (advanced {achieved:+.2f} m "
                            f"along the start heading, path {travelled:.2f} m)")
            else:
                remaining = p["distance"] + self._drive_bias - travelled
                if abs(remaining) <= self.DRIVE_BRAKE_M or timed_out:
                    with self.lock:
                        p2 = dict(p)
                        p2["phase"] = "settle"
                        p2["settle_t0"] = self.robot.getTime()
                        self.motion = ("drive", p2)
                    self._command_velocity(0.0, 0.0)
                else:
                    v_mag = clamp(self.DRIVE_APPROACH_GAIN * abs(remaining),
                                  self.DRIVE_APPROACH_MIN, abs(p["speed"]))
                    self._command_velocity(
                        direction * math.copysign(v_mag, remaining), 0.0)
        elif kind == "turn":
            # Turn in place: PULSE-AND-SETTLE with an adaptive rate gain.
            #
            # Three phases, cycled until the settled error is inside tolerance:
            #   PLAN   — read the SETTLED yaw, compute the residual, and size an
            #            open-loop pulse from the learned gain;
            #   PULSE  — hold a constant yaw rate for that duration. Deliberately
            #            open-loop: the in-motion yaw readback is the thing that
            #            is wrong, so nothing decides when to stop by reading it;
            #   SETTLE — command zero, wait, then re-measure. Settled-to-settled
            #            is the only yaw this loop ever believes.
            now_sim = self.robot.getTime()
            timed_out = ((now_sim - p["t0_sim"]) > p["timeout_s"]
                         if "t0_sim" in p else
                         (time.time() - p["t0"]) > p["timeout_s"])
            phase = p.get("phase", "plan")

            if phase == "pulse":
                if now_sim >= p["pulse_end_sim"] or timed_out:
                    with self.lock:
                        p2 = dict(p)
                        p2["phase"] = "settle"
                        p2["settle_t0"] = now_sim
                        self.motion = ("turn", p2)
                    self._command_velocity(0.0, 0.0)
                else:
                    self._command_velocity(0.0, p["pulse_w"])

            elif phase == "settle":
                self._command_velocity(0.0, 0.0)
                if (now_sim - p["settle_t0"]) >= self.TURN_SETTLE_S or timed_out:
                    # Learn the rate gain from settled truth: how much yaw did a
                    # commanded (w x dt) actually buy? On the Husky under
                    # Newton/MuJoCo this converges to ~0.17 -- i.e. a skid-steer
                    # pivot delivers about a sixth of the commanded rate.
                    got = abs(wrap_pi(yaw - p["pulse_y0"]))
                    want = abs(p["pulse_want"])
                    if want > 1e-6 and got > 1e-4:
                        self._turn_gain = clamp(
                            self._turn_gain * (got / want),
                            self.TURN_GAIN_MIN, self.TURN_GAIN_MAX)
                    stepped = wrap_pi(yaw - p["pulse_y0"])   # settled truth
                    with self.lock:
                        p2 = dict(p)
                        p2["phase"] = "plan"
                        p2["remaining"] = p["remaining"] - stepped
                        p2["corrections"] = p.get("corrections", 0) + 1
                        self.motion = ("turn", p2)

            else:  # "plan"
                # SIGNED REMAINING ROTATION, decremented by each settled delta
                # -- not an absolute target yaw. wrap_pi(target - yaw) makes
                # +180° and -180° the same number, so a half-turn had no defined
                # direction and landed 42° short; accumulated remaining has no
                # antipode and also lets a turn exceed ±π.
                err = p["remaining"]
                done = (abs(err) <= self.TURN_TOL_RAD or timed_out
                        or p.get("corrections", 0) >= self.TURN_MAX_CORRECTIONS)
                if done:
                    with self.lock:
                        self.motion = ("idle", {})
                    self._command_velocity(0.0, 0.0)
                    # `err` is what is LEFT of the commanded rotation, so the
                    # achieved rotation is commanded - remaining. Measured,
                    # settled, and signed -- not the argument echoed back.
                    self._record_completion(
                        p, float(p.get("commanded", 0.0)) - err,
                        settled=not timed_out, timed_out=timed_out)
                    msg = (f"turn complete (yaw err {math.degrees(err):+.2f} deg, "
                           f"{p.get('corrections', 0)} pulses, "
                           f"gain {self._turn_gain:.3f}"
                           f"{', TIMED OUT' if timed_out else ''})")
                    self.queue_window("system:" + msg)
                    # Also to stdout: the window queue reaches the robot GUI
                    # only, so the turn corrector's own verdict was invisible
                    # to anything measuring it -- which is how a -43% actuator
                    # shipped documented as -19%.
                    print(f"[omnilink_mobile_bridge] {self.robot_id}: {msg}",
                          flush=True)
                else:
                    w_mag = (self.spin_speed if abs(err) > self.TURN_SLOW_RAD
                             else self.TURN_SLOW_W)
                    dur = min(abs(err) / max(w_mag * self._turn_gain, 1e-3),
                              self.TURN_PULSE_MAX_S,
                              # Never command a pulse that could rotate past
                              # pi -- the settle phase measures it with
                              # wrap_pi and would read it back negated. The
                              # bound uses the COMMANDED rate, not the learned
                              # gain, so it holds precisely when the gain
                              # estimate is wrong, which is exactly when
                              # over-delivery happens.
                              self.TURN_PULSE_MAX_RAD / max(w_mag, 1e-3))
                    with self.lock:
                        p2 = dict(p)
                        p2["phase"] = "pulse"
                        p2["pulse_w"] = math.copysign(w_mag, err)
                        p2["pulse_end_sim"] = now_sim + dur
                        p2["pulse_y0"] = yaw
                        p2["pulse_want"] = w_mag * self._turn_gain * dur
                        self.motion = ("turn", p2)
                    self._command_velocity(0.0, p2["pulse_w"])

        # Kinematic bodies: integrate the commanded (v, w) into a pose write
        # (the same supervisor-drive method the robot's own controllers use).
        if self.kinematic:
            v, w = self._kin_cmd
            if v != 0.0 or w != 0.0:
                heading = wrap_pi(yaw + w * dt_s)
                nx = x + v * math.cos(heading) * dt_s
                ny = y + v * math.sin(heading) * dt_s
                try:
                    self.self_node.getField("translation").setSFVec3f(
                        [nx, ny, self.floor_z])
                    self.self_node.getField("rotation").setSFRotation(
                        [0.0, 0.0, 1.0, heading + self.yaw_offset])
                except Exception:
                    pass
                x, y, yaw = nx, ny, heading

        # Optional trolley tow (no-op unless --pallets active AND docked).
        if self.pallet_feature:
            self._tick_pallet(x, y, yaw)

        # Optional cart-conveyor carries (armed by the idle loop after a
        # detach; no-op otherwise).
        if self._carries:
            self._tick_carries()

    # ── Kinematic trolley tow (guarded by self.pallet_feature) ────────
    # One-trailer model: the robot's REAR is the tractor coupling; the
    # trolley's hitch point (a fixed point in the trolley frame) tracks it
    # each tick, and the trolley heading converges toward the hitch-to-body
    # direction — so the cart TRAILS naturally through turns instead of
    # rigidly rotating with the robot.

    _DOCK_RADIUS_M = 0.6      # robot rear must be this close to the hitch
    _REAR_OFFSET_M = 0.5      # robot centre -> rear coupling point
    _HITCH_LOCAL = (0.70, 0.0)   # hitch point in the trolley frame (x fwd)
    # ── Footprints (for the jackknife guard + towed-clearance telemetry) ──
    # AMR body 1.259 (L, heading axis) x 0.716 (W); trolley deck collider
    # 0.70 x 0.70 (the boundingObject in the world). Used only when a cart is
    # actually docked, so non-tug bases pay nothing.
    _FOOT_L = 1.259
    _FOOT_W = 0.716
    _DECK_M = 0.70
    # ── Jackknife guard (the "cart penetrates the MAV when it rotates" fix) ──
    # The MAV is KINEMATIC and the trolley is POSE-WRITTEN, so nothing in the
    # solver stops the drawbar folding until the deck sits inside the body. In
    # the bare pursuit model an in-place spin winds the trolley heading toward
    # a full 180deg jackknife, at which the deck centre coincides with the tug
    # centre -- the reported overlap. We clamp the ARTICULATION ANGLE (trolley
    # heading - tug heading): past the stop, a rigid drawbar swings the cart
    # WITH the tug instead of folding through.
    #
    # Derivation (2D OBB deck-vs-body sweep over articulation, all measured):
    # with rear_offset 0.63 m and drawbar (hitch x) 0.70 m the deck clears the
    # body up to ~54deg at a 0.06 m margin (see the fix report). The original
    # 0.60 m drawbar cleared only to ~44deg / clamped to ~34deg at that margin
    # -- too tight, it clips a normal arc -- so the drawbar was lengthened to
    # 0.70 m (world hitch geometry moved to match). Clamp at 50deg keeps a
    # guaranteed >=0.076 m deck-to-body gap at the stop, while sitting well
    # above the natural trailing lag of any normal arc (measured <~40deg): so
    # gentle arcs still trail freely and only hard / in-place rotation
    # saturates the clamp. Applied for BOTH spin senses and while reversing.
    _ARTIC_MAX_RAD = 0.8727      # math.radians(50.0)
    _SNAP_S = 0.5             # magnet closes the dock gap over this long
    _RIDER_RADIUS_M = 0.5     # GRASP_* parts this close to the basket ride it
    # Hover epsilons while the ensemble is teleported (tow + glide): a body
    # teleported INTO steady contact (trolley base on the floor, riders on
    # the basket floor) makes the contact solver fight the teleport every
    # tick — measured as a progressive realtime collapse over a long tow
    # leg (1.0x -> 0.25x), snapping back at detach. Millimetres of air keep
    # the contacts open; everything settles on detach/park.
    _TOW_LIFT_Z = 0.004       # trolley base hover while towed/glided
    # Per-rider hover, applied CUMULATIVELY up the stack (see _basket_riders):
    # the deck lifts by _TOW_LIFT_Z, so a rider lifted by the same amount is
    # still in steady contact with it, and a part resting on a BOX that is
    # itself a rider is in contact with the box. Every such steady contact
    # makes the solver fight the teleport on every tick -- measured here as
    # 0.999x realtime idle collapsing to 0.166x for the whole tow leg, with a
    # loaded box + 3 parts. Giving each layer its own extra millimetres of
    # air keeps all the contacts open and holds realtime through the tow.
    _RIDER_LIFT_Z = 0.003     # clearance granted per stack layer
    # Supervisor WRITE CADENCE while towing. Every pose write + resetPhysics on
    # a Newton body costs real sim time, and a tow writes the trolley plus each
    # rider forever. Clearing the contact fight above got the loaded tow from
    # 0.166x back to ~0.9x instantaneous, but a long tow leg still averaged
    # ~0.6x -- and an EMPTY trolley tow cost the same, which rules out rider
    # count and points at the raw per-tick write. These decimate it; the trailer
    # lag they introduce (a few cm at tow speed) is invisible next to the cart's
    # own size, and the pose is exact again the moment it is set down.
    _TOW_WRITE_EVERY = 2      # ticks between trolley pose writes
    _RIDER_WRITE_EVERY = 4    # ticks between rider pose writes

    # DEF prefixes whose top-level Solids ride a towed trolley. GRASP_* are
    # loose manipulanda (the classic basket payload); BOX_* are conveyor-line
    # boxes resting on the deck -- WITHOUT them a tow would drive off and
    # leave the filled box standing at the fill station, because the trolley
    # is teleported kinematically rather than pushed through contact.
    _RIDER_DEF_PREFIXES = ("GRASP_", "BOX_")

    def _iter_grasp_parts(self):
        """Yield (def, node, tfield) for every top-level rideable Solid."""
        try:
            root = self.robot.getRoot()
            kids = root.getField("children")
            n = kids.getCount()
        except Exception:
            return
        for i in range(n):
            try:
                node = kids.getMFNode(i)
                d = node.getDef() or ""
                if not d.startswith(self._RIDER_DEF_PREFIXES):
                    continue
                tf = node.getField("translation")
                if tf is not None:
                    yield d, node, tf
            except Exception:
                continue

    def _tick_pallet(self, x: float, y: float, yaw: float) -> None:
        """Per-tick trailer update (kinematic-ensemble, the anypick-line
        bin-transport pattern: pose + resetPhysics = zero velocity).

        H = the robot's rear coupling point (blended from the dock-time gap
        over _SNAP_S so the magnet closes smoothly -- no teleport pop).
        The trolley heading turns toward (H - trolley centre) and the centre
        is placed |_HITCH_LOCAL| behind H along the new heading: the classic
        follow-the-leader trailer, so the cart trails through turns and
        never slides sideways. Deck stays at z=0 (casters on the floor).
        Because the trolley is a real physics body, its basket collider
        MOVES with it -- after detach the parts simply REST in the basket
        (no steady-state pinning, which dragged the sim below 0.2x)."""
        with self.lock:
            carry = self._carry
        if carry is None:
            return
        hx_off = math.hypot(*self._HITCH_LOCAL)
        rear_off = float(self.cfg.get("rear_offset_m", self._REAR_OFFSET_M))
        rx = x - rear_off * math.cos(yaw)
        ry = y - rear_off * math.sin(yaw)
        # magnet snap: blend out the initial rear->hitch gap
        a = clamp((self.robot.getTime() - carry["t0"]) / self._SNAP_S, 0.0, 1.0)
        a = a * a * (3.0 - 2.0 * a)
        hx = rx + carry["gap"][0] * (1.0 - a)
        hy = ry + carry["gap"][1] * (1.0 - a)
        # trailer pursuit: heading turns toward the hitch
        dx, dy = hx - carry["cx"], hy - carry["cy"]
        if math.hypot(dx, dy) > 1e-4:
            carry["psi"] = math.atan2(dy, dx)
        psi = carry["psi"]
        # JACKKNIFE GUARD: a rigid drawbar cannot fold past _ARTIC_MAX_RAD
        # relative to the tug. The bare pursuit above would wind the trolley
        # heading toward a 180deg fold during an in-place rotation and slide
        # the deck through the (kinematic) MAV body. Clamping the articulation
        # (trolley heading - tug heading) makes the cart SWING WITH the tug at
        # the stop instead -- exactly what the drawbar's mechanical limit does.
        # Same clamp for both turn senses and while reversing; smooth, because
        # the clamped psi tracks the tug heading continuously past the stop.
        artic = wrap_pi(psi - yaw)
        if artic > self._ARTIC_MAX_RAD:
            psi = wrap_pi(yaw + self._ARTIC_MAX_RAD)
            carry["psi"] = psi
        elif artic < -self._ARTIC_MAX_RAD:
            psi = wrap_pi(yaw - self._ARTIC_MAX_RAD)
            carry["psi"] = psi
        cx = hx - hx_off * math.cos(psi)
        cy = hy - hx_off * math.sin(psi)
        carry["cx"], carry["cy"] = cx, cy
        # The pursuit math above runs EVERY tick (it is pure float work and
        # keeps the trailer path exact); only the supervisor WRITES are
        # decimated, so the cart still follows the same curve.
        carry["wphase"] = (carry.get("wphase", 0) + 1) % self._TOW_WRITE_EVERY
        if carry["wphase"] == 0:
            try:
                carry["tfield"].setSFVec3f([cx, cy, self._TOW_LIFT_Z])
                if carry.get("rfield") is not None:
                    carry["rfield"].setSFRotation([0.0, 0.0, 1.0, psi])
                carry["node"].resetPhysics()
            except Exception:
                pass
        # Riders lag further still: they sit inside a box or basket whose walls
        # hide a few cm of alternate-tick lag entirely.
        carry["rphase"] = (carry.get("rphase", 0) + 1) % self._RIDER_WRITE_EVERY
        if carry["rphase"] == 0:
            c, s = math.cos(psi), math.sin(psi)
            for rider in carry.get("riders", []):
                lx, ly, lz = rider["local"]
                try:
                    rider["tfield"].setSFVec3f([
                        cx + lx * c - ly * s,
                        cy + lx * s + ly * c,
                        lz + rider.get("lift", self._RIDER_LIFT_Z),
                    ])
                    rider["node"].resetPhysics()
                except Exception:
                    pass

    # ── Towed-cart clearance telemetry (honest, self-verifying) ───────
    # A towed cart is POSE-WRITTEN, so an observer cannot tell from the deck
    # pose alone whether it is clear of the tug body. This exposes the actual
    # 2D OBB gap between the MAV footprint and the deck (negative == overlap)
    # plus the live articulation angle, so /state proves the jackknife guard
    # is holding. Only computed while a cart is docked.

    @staticmethod
    def _rect(cx, cy, hx, hy, ang):
        c, s = math.cos(ang), math.sin(ang)
        return [(cx + sx * c - sy * s, cy + sx * s + sy * c)
                for sx, sy in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))]

    @staticmethod
    def _obb_clearance(A, B):
        """Signed clearance between two convex quads: >=0 true gap,
        <0 -penetration depth (SAT)."""
        def axes(poly):
            out = []
            for i in range(len(poly)):
                x0, y0 = poly[i]
                x1, y1 = poly[(i + 1) % len(poly)]
                ex, ey = x1 - x0, y1 - y0
                L = math.hypot(ex, ey) or 1.0
                out.append((-ey / L, ex / L))
            return out

        def proj(poly, ax):
            ds = [p[0] * ax[0] + p[1] * ax[1] for p in poly]
            return min(ds), max(ds)

        separated = False
        min_overlap = 1e9
        for ax in axes(A) + axes(B):
            a0, a1 = proj(A, ax)
            b0, b1 = proj(B, ax)
            if max(b0 - a1, a0 - b1) > 0:
                separated = True
                break
            min_overlap = min(min_overlap, min(a1 - b0, b1 - a0))
        if not separated:
            return -min_overlap
        # disjoint: true min distance over all vertex->edge pairs
        def seg_pt(px, py, ax_, ay_, bx_, by_):
            dx, dy = bx_ - ax_, by_ - ay_
            l2 = dx * dx + dy * dy
            if l2 < 1e-12:
                return math.hypot(px - ax_, py - ay_)
            t = clamp(((px - ax_) * dx + (py - ay_) * dy) / l2, 0.0, 1.0)
            return math.hypot(px - (ax_ + t * dx), py - (ay_ + t * dy))

        d = 1e9
        for P, Q in ((A, B), (B, A)):
            for px, py in P:
                for i in range(len(Q)):
                    qx0, qy0 = Q[i]
                    qx1, qy1 = Q[(i + 1) % len(Q)]
                    d = min(d, seg_pt(px, py, qx0, qy0, qx1, qy1))
        return d

    def _towed_telemetry(self, x: float, y: float, yaw: float) -> Optional[dict]:
        with self.lock:
            carry = self._carry
        if carry is None:
            return None
        cx, cy, psi = carry["cx"], carry["cy"], carry["psi"]
        body = self._rect(x, y, self._FOOT_L / 2, self._FOOT_W / 2, yaw)
        deck = self._rect(cx, cy, self._DECK_M / 2, self._DECK_M / 2, psi)
        return {
            "def": self.carrying,
            "x": round(cx, 4), "y": round(cy, 4), "yaw": round(psi, 5),
            "artic_deg": round(math.degrees(wrap_pi(psi - yaw)), 2),
            "clearance_m": round(self._obb_clearance(body, deck), 4),
        }

    def act_attach_trolley(self, def_name: Optional[str] = None) -> dict:
        if not self.pallet_feature:
            return {"error": "trolley_tow_inactive",
                    "hint": "world must pass --pallets and supervisor TRUE"}
        target = (def_name or "").strip() or self.pallet_defs[0]
        if target not in self.pallet_defs:
            return {"error": "unknown_trolley", "def": target,
                    "known": list(self.pallet_defs)}
        with self.lock:
            # ATTACHING WHAT YOU ALREADY TOW IS A NO-OP, NOT A FAILURE.
            #
            # This refused unconditionally, and that turned an ordinary chat
            # message into a permanent line stall. Measured: a "stop" landed
            # 3 s after the return tug attached TROLLEY_C. On resume the idle
            # loop correctly saw `carrying` and called _tow_to() to stage it --
            # but _tow_to() starts with _dock(), _dock() got `already_towing`,
            # returned False, and _tow_to() bailed BEFORE the detach. So
            # `carrying` stayed set, the next cycle did exactly the same, and
            # the tug livelocked: 131 iterations over ~21 minutes, the fill
            # station starved, and the arm did zero picks for the whole window.
            #
            # It was invisible too -- fault null, mode "drive", and jobs_total
            # climbing because every iteration hit a task boundary. The robot
            # reported itself busy and healthy while achieving nothing.
            #
            # Idempotence is the honest contract: if you are already towing the
            # cart you were asked to take, you are docked. Asking for a
            # DIFFERENT cart while towing is still an error, because that one
            # really cannot be satisfied.
            if self.carrying is not None:
                if self.carrying == target:
                    return {"ok": True, "carrying": self.carrying,
                            "already_attached": True,
                            "say": f"I already have {target} in tow."}
                return {"error": "already_towing", "carrying": self.carrying,
                        "requested": target,
                        "say": (f"I can't pick up {target} — I'm already towing "
                                f"{self.carrying}. Tell me to drop that first.")}
        node = self.robot.getFromDef(target)
        if node is None:
            return {"error": "trolley_not_found", "def": target}
        tfield = node.getField("translation")
        rfield = node.getField("rotation")
        if tfield is None:
            return {"error": "trolley_has_no_translation_field", "def": target}
        pp = tfield.getSFVec3f()
        # trolley yaw (z-axis rotation assumed)
        psi = 0.0
        try:
            rot = rfield.getSFRotation()
            if abs(rot[2]) > 0.9:
                psi = rot[3] * (1 if rot[2] > 0 else -1)
        except Exception:
            pass
        # world hitch point + robot rear coupling point
        hlx, hly = self._HITCH_LOCAL
        cpsi, spsi = math.cos(psi), math.sin(psi)
        hx = pp[0] + hlx * cpsi - hly * spsi
        hy = pp[1] + hlx * spsi + hly * cpsi
        x, y, yaw = self._read_pose()
        rear_off = float(self.cfg.get("rear_offset_m", self._REAR_OFFSET_M))
        rx = x - rear_off * math.cos(yaw)
        ry = y - rear_off * math.sin(yaw)
        dist = math.hypot(hx - rx, hy - ry)
        if dist > self._DOCK_RADIUS_M:
            return {"error": "too_far", "def": target,
                    "rear_to_hitch_m": round(dist, 3),
                    "max_m": self._DOCK_RADIUS_M,
                    "hint": ("back the robot's REAR up to the trolley hitch "
                             "bar first")}
        # Capture riders: GRASP_* parts in/on the basket, stored in the
        # trolley frame so they ride it exactly.
        riders = self._basket_riders(pp, psi)
        with self.lock:
            # Docking wins over a conveyor: a tug that has physically coupled
            # to a cart must not have the chain still writing that cart's pose.
            self._carries = [c for c in self._carries if c["def"] != target]
            self._carry = {"def": target, "node": node,
                           "tfield": tfield, "rfield": rfield,
                           "riders": riders,
                           "psi": psi, "cx": pp[0], "cy": pp[1],
                           # dock-time hitch gap, blended out over _SNAP_S
                           "gap": (hx - rx, hy - ry),
                           "t0": self.robot.getTime()}
            self.carrying = target
        self.queue_window(f"system:docked to trolley {target} "
                          f"({len(riders)} part(s) aboard)")
        # THE `on_next_pickup` TRIGGER. Fires for an autonomous dock and for
        # an operator-commanded one alike -- "if you pick up another cart,
        # tell me before you move it" must not care which.
        if self.intents is not None:
            # safe=True on purpose: "tell me BEFORE you move it" has to hold
            # the tug at exactly the moment it is holding a cart.
            self.intents.note_event("pickup", target, safe=True)
        return {"accepted": True, "carrying": target,
                "rear_to_hitch_m": round(dist, 3),
                "riders": sorted(r["def"] for r in riders)}

    def _basket_riders(self, pp, psi) -> List[dict]:
        """GRASP_* parts riding in/on the basket at trolley pose (pp, psi),
        with their offsets stored in the trolley frame."""
        cpsi, spsi = math.cos(psi), math.sin(psi)
        riders = []
        for d, rnode, rtf in self._iter_grasp_parts():
            try:
                rp = rtf.getSFVec3f()
            except Exception:
                continue
            if math.hypot(rp[0] - pp[0], rp[1] - pp[1]) > self._RIDER_RADIUS_M:
                continue
            if rp[2] < 0.05:
                continue
            dx, dy, dz = rp[0] - pp[0], rp[1] - pp[1], rp[2] - pp[2]
            riders.append({
                "def": d, "node": rnode, "tfield": rtf,
                "local": (dx * cpsi + dy * spsi, -dx * spsi + dy * cpsi, dz),
            })
        # Stack order: lowest rider first, so each layer can be given strictly
        # more clearance than whatever it is resting on (deck < box < parts).
        riders.sort(key=lambda r: r["local"][2])
        for i, r in enumerate(riders):
            r["lift"] = self._TOW_LIFT_Z + self._RIDER_LIFT_Z * (i + 1)
        return riders

    # Back-compat alias (earlier tool name).
    def act_grab_pallet(self, def_name: Optional[str] = None) -> dict:
        return self.act_attach_trolley(def_name)

    def act_detach_trolley(self) -> dict:
        if not self.pallet_feature:
            return {"error": "trolley_tow_inactive",
                    "hint": "world must pass --pallets and supervisor TRUE"}
        with self.lock:
            carry = self._carry
        if carry is None:
            return {"error": "not_towing"}
        # The trailer already sits flat on its casters at its trailed pose --
        # just stop following and settle the riders once; they then rest
        # PHYSICALLY in the basket (the trolley body's collider travelled
        # with it), so no ongoing pinning is needed.
        cx, cy, psi = carry["cx"], carry["cy"], carry["psi"]
        c, s = math.cos(psi), math.sin(psi)
        settled = []
        for rider in carry.get("riders", []):
            lx, ly, lz = rider["local"]
            try:
                rider["tfield"].setSFVec3f([cx + lx * c - ly * s,
                                            cy + lx * s + ly * c, lz])
                rider["node"].resetPhysics()
                settled.append(rider["def"])
            except Exception:
                pass
        try:
            carry["node"].resetPhysics()
        except Exception:
            pass
        with self.lock:
            self._carry = None
            self.carrying = None
        self.queue_window(f"system:detached trolley {carry['def']} at "
                          f"({cx:+.2f}, {cy:+.2f})")
        return {"accepted": True, "released": carry["def"],
                "at": [round(cx, 3), round(cy, 3), 0.0],
                "yaw": round(psi, 3),
                "riders_settled": sorted(settled)}

    # Back-compat alias (earlier tool name).
    def act_release_pallet(self) -> dict:
        return self.act_detach_trolley()

    # ── CART CONVEYORS (the in-floor drag chains) ─────────────────────
    #
    # THE RULE THIS EXISTS TO ENFORCE: a cart may only move if something
    # visible is moving it. A tug tow is self-evidently caused -- you can see
    # the tug. A conveyor carry is NOT, unless the conveyor is really there
    # and really running. So this primitive drives an actual world body, the
    # chain DOG, alongside the cart for the whole carry and then recirculates
    # it to the head of the run. Take the dog away and you are straight back
    # to a 22 kg cart sliding across bare concrete on its own -- which is the
    # "magic glide" this replaced, and the single worst read in the demo.
    #
    # STRAIGHT LINE, CONSTANT SPEED, soft start and soft stop. Deliberately
    # not a spline: a chain conveyor has no reason to curve, and a curve is a
    # tell that what you are watching is an animation rather than a machine.
    # The ramp is what makes it read as a driven chain taking up load rather
    # than a position being lerped.
    #
    # Several carries can be in flight at once (one bridge owns two
    # conveyors), hence a list rather than the single slot the old glide had.
    # Riders (a box on the deck, the parts in the box) travel with the cart.
    # Runs on the SIM THREAD from tick(); docking the cart cancels its carry.

    CONVEY_SPEED = 0.26       # m/s chain speed -- walking pace, believable
    CONVEY_RAMP = 0.55        # s of soft start / soft stop
    DOG_TRAIL = 0.44          # m the pusher dog rides behind the cart centre
    DOG_RETURN_MULT = 2.2     # empty chain recirculates faster than it carries

    @staticmethod
    def _ramp_profile(dist: float, speed: float, ramp: float):
        """Trapezoidal distance-vs-time for a chain conveyor: accelerate over
        ``ramp``, cruise at ``speed``, decelerate over ``ramp``. Returns
        (duration, s_of_t). Short runs degrade to a single smoothstep so the
        profile never inverts on a sub-ramp distance."""
        speed = max(0.02, float(speed))
        ramp = max(0.05, float(ramp))
        if dist <= speed * ramp:
            dur = max(0.8, 2.0 * math.sqrt(max(dist, 1e-6) / speed))

            def s_short(t):
                a = clamp(t / dur, 0.0, 1.0)
                return dist * a * a * (3.0 - 2.0 * a)

            return dur, s_short
        dur = dist / speed + ramp

        def s_of_t(t):
            if t <= 0.0:
                return 0.0
            if t < ramp:
                return speed * t * t / (2.0 * ramp)
            if t <= dur - ramp:
                return speed * (t - ramp / 2.0)
            if t >= dur:
                return dist
            rem = dur - t
            return dist - speed * rem * rem / (2.0 * ramp)

        return dur, s_of_t

    def start_cart_conveyor(self, def_name: str, target,
                            final_yaw: Optional[float] = None, *,
                            dog_def: Optional[str] = None,
                            speed: Optional[float] = None,
                            label: str = "conveyor") -> dict:
        """Carry a DETACHED cart in a straight line to ``target`` (x, y),
        driving ``dog_def`` along with it. Idempotent per cart: asking for a
        carry of a cart already being carried is a no-op, not a second one."""
        if not self.pallet_feature:
            return {"error": "trolley_tow_inactive"}
        with self.lock:
            if self.carrying == def_name:
                return {"error": "cannot_convey_while_towing"}
            if any(c["def"] == def_name for c in self._carries):
                return {"error": "already_on_a_conveyor"}
        node = self.robot.getFromDef(def_name)
        if node is None:
            return {"error": "trolley_not_found", "def": def_name}
        tfield = node.getField("translation")
        rfield = node.getField("rotation")
        if tfield is None:
            return {"error": "trolley_has_no_translation_field"}
        pp = tfield.getSFVec3f()
        psi0 = 0.0
        try:
            rot = rfield.getSFRotation()
            if abs(rot[2]) > 0.9:
                psi0 = rot[3] * (1 if rot[2] > 0 else -1)
        except Exception:
            pass
        x1, y1 = float(target[0]), float(target[1])
        dist = math.hypot(x1 - pp[0], y1 - pp[1])
        if dist < 1e-4:
            return {"error": "already_there"}
        spd = float(speed or self.CONVEY_SPEED)
        dur, s_of_t = self._ramp_profile(dist, spd, self.CONVEY_RAMP)
        ux, uy = (x1 - pp[0]) / dist, (y1 - pp[1]) / dist

        dog = None
        if dog_def:
            dnode = self.robot.getFromDef(dog_def)
            if dnode is not None:
                dtf = dnode.getField("translation")
                if dtf is not None:
                    dog = {"node": dnode, "tf": dtf,
                           "home": list(dtf.getSFVec3f())}

        carry = {
            "def": def_name, "node": node, "tfield": tfield, "rfield": rfield,
            "x0": pp[0], "y0": pp[1], "z": pp[2], "ux": ux, "uy": uy,
            "dist": dist, "dur": dur, "s_of_t": s_of_t,
            "psi0": psi0,
            "psi1": psi0 if final_yaw is None else float(final_yaw),
            "t0": self.robot.getTime(), "riders": self._basket_riders(pp, psi0),
            "dog": dog, "phase": "carry", "rphase": 0, "label": label,
        }
        with self.lock:
            self._carries.append(carry)
        return {"accepted": True, "distance": round(dist, 3),
                "duration_s": round(dur, 2),
                "riders": len(carry["riders"]), "conveyor": label}

    def conveyor_busy(self, def_name: Optional[str] = None) -> bool:
        with self.lock:
            if def_name is None:
                return bool(self._carries)
            return any(c["def"] == def_name and c["phase"] == "carry"
                       for c in self._carries)

    def cancel_cart_conveyor(self, def_name: str) -> None:
        with self.lock:
            self._carries = [c for c in self._carries
                             if c["def"] != def_name]

    def transfer_cart(self, def_name: str, xy, yaw: float) -> dict:
        """Set a cart down at an exact pose in ONE step. This is the dock-E
        handoff and nothing else: it is only ever called on a cart that has
        already been conveyed out through a doorway, to place it inside the
        opposite doorway. It is the one non-continuous cart move in the world
        and it is deliberately confined to a single call site so it cannot
        quietly become a general-purpose teleport."""
        node = self.robot.getFromDef(def_name)
        if node is None:
            return {"error": "trolley_not_found", "def": def_name}
        tf = node.getField("translation")
        rf = node.getField("rotation")
        if tf is None:
            return {"error": "trolley_has_no_translation_field"}
        p = tf.getSFVec3f()
        try:
            tf.setSFVec3f([float(xy[0]), float(xy[1]), p[2]])
            if rf is not None:
                rf.setSFRotation([0.0, 0.0, 1.0, float(yaw)])
            node.resetPhysics()
            node.setVelocity([0.0] * 6)
        except Exception as e:
            return {"error": "transfer_failed", "detail": repr(e)}
        return {"accepted": True}

    def _tick_carries(self) -> None:
        with self.lock:
            carries = list(self._carries)
        done_defs = []
        for g in carries:
            t = self.robot.getTime() - g["t0"]
            if g["phase"] == "dog_return":
                self._tick_dog_return(g, t, done_defs)
                continue
            s = g["s_of_t"](t)
            x = g["x0"] + g["ux"] * s
            y = g["y0"] + g["uy"] * s
            a = clamp(t / g["dur"], 0.0, 1.0)
            psi = g["psi0"] + wrap_pi(g["psi1"] - g["psi0"]) * (
                a * a * (3.0 - 2.0 * a))
            done = t >= g["dur"]
            lift = 0.0 if done else self._TOW_LIFT_Z
            try:
                g["tfield"].setSFVec3f([x, y, g["z"] + lift])
                if g["rfield"] is not None:
                    g["rfield"].setSFRotation([0.0, 0.0, 1.0, psi])
                g["node"].resetPhysics()
                if g["dog"] is not None:
                    dh = g["dog"]["home"]
                    g["dog"]["tf"].setSFVec3f(
                        [x - g["ux"] * self.DOG_TRAIL,
                         y - g["uy"] * self.DOG_TRAIL, dh[2]])
                # Riders are re-pinned every other tick: they are inside a
                # box on the deck, so a half-rate correction is invisible and
                # halves the supervisor writes on a leg that already runs two
                # conveyors plus a tow.
                g["rphase"] = (g["rphase"] + 1) % 2
                if g["rphase"] == 0 or done:
                    c, si = math.cos(psi), math.sin(psi)
                    for rider in g["riders"]:
                        lx, ly, lz = rider["local"]
                        rlift = (0.0 if done
                                 else rider.get("lift", self._RIDER_LIFT_Z))
                        rider["tfield"].setSFVec3f([x + lx * c - ly * si,
                                                    y + lx * si + ly * c,
                                                    lz + rlift])
                        rider["node"].resetPhysics()
            except Exception:
                pass
            if done:
                print(f"[omnilink_mobile_bridge] {g['label']}: cart "
                      f"{g['def']} set down at ({x:+.2f}, {y:+.2f})",
                      flush=True)
                self.queue_window(
                    f"system:{g['label']} delivered {g['def']}")
                if g["dog"] is not None:
                    # Recirculate the chain: the dog runs back to the head of
                    # the conveyor so the next carry starts from a real
                    # standing position rather than wherever it was left.
                    dh = g["dog"]["home"]
                    dx = dh[0] - (x - g["ux"] * self.DOG_TRAIL)
                    dy = dh[1] - (y - g["uy"] * self.DOG_TRAIL)
                    rdist = math.hypot(dx, dy)
                    if rdist > 0.02:
                        g["ret_x0"] = x - g["ux"] * self.DOG_TRAIL
                        g["ret_y0"] = y - g["uy"] * self.DOG_TRAIL
                        g["ret_dx"], g["ret_dy"] = dx, dy
                        g["ret_dur"] = max(
                            0.6, rdist / (self.CONVEY_SPEED
                                          * self.DOG_RETURN_MULT))
                        g["ret_t0"] = self.robot.getTime()
                        g["phase"] = "dog_return"
                        continue
                done_defs.append(g["def"])
        if done_defs:
            # done_defs only ever holds carries that are FULLY finished: the
            # cart is down AND (if there is a dog) the chain has recirculated.
            # A carry that just handed off to its dog-return phase is not in
            # here, so it survives this filter and keeps ticking.
            with self.lock:
                self._carries = [c for c in self._carries
                                 if c["def"] not in done_defs]

    def _tick_dog_return(self, g: dict, _t: float, done_defs: list) -> None:
        rt = self.robot.getTime() - g["ret_t0"]
        a = clamp(rt / g["ret_dur"], 0.0, 1.0)
        a = a * a * (3.0 - 2.0 * a)
        dh = g["dog"]["home"]
        try:
            g["dog"]["tf"].setSFVec3f([g["ret_x0"] + g["ret_dx"] * a,
                                       g["ret_y0"] + g["ret_dy"] * a, dh[2]])
        except Exception:
            pass
        if a >= 1.0:
            done_defs.append(g["def"])

    # ── Actions (HTTP + intent share these) ───────────────────────
    #
    # act_stop lives further down, next to the result-contract machinery it
    # reuses (_supersede_inflight, the SOURCE_* constants and the pose
    # history): it is a MEASURED verb like the others, not a bare acceptance.

    # ── Action result contract (PROTOCOL.md §5.4.1) ───────────────

    WAIT_POLL_S = 0.02          # how often a waiting caller re-checks
    # Ceiling on any `wait: true` call, deliberately BELOW the edge
    # connector's 55 s per-tool timeout (omnilink-lib TOOL_TIMEOUT) and the
    # platform's 60 s PER_TOOL_TIMEOUT_MS. If we blocked past those the model
    # would get an opaque transport error instead of our structured "still
    # moving, go poll" note -- and a long turn (a 180 deg pivot is ~40 s of
    # sim time) is close enough to that edge to matter.
    # A caller may explicitly ask a synchronous endpoint to wait through a
    # cold/JIT-bound run. Keep a finite safety ceiling, but do not silently
    # reduce ordinary 60-120 s agent budgets to 50 s.
    WAIT_MAX_S = 300.0
    # Wall pause after an aborted drive_to leg so the commanded stop has
    # actually reached the wheels before the final pose is read.
    DRIVE_TO_ABORT_SETTLE_S = 0.35

    # ── stop_robot: measured rest + operator hold ─────────────────
    # How long a stop watches the POSE before it is willing to say the robot
    # is stationary. Long enough to span several basic timesteps (so there
    # are real samples to difference) and short enough that an operator's
    # "stop" still answers inside a chat turn.
    STOP_SETTLE_S = 0.30
    # The residual-speed measurement is taken over the TAIL of that window,
    # so a decelerating skid-steer's ramp is not averaged into the answer.
    STOP_TAIL_FRAC = 0.5
    # Minimum sample span before a speed is believable. Below this the answer
    # is `null` and says why -- never a number nobody measured.
    STOP_MIN_SPAN_S = 0.04
    # "Stationary" thresholds. A kinematic base parks at exactly zero (this
    # controller writes the pose); the margin is for the physical bases,
    # whose supervisor pose still jitters a little at rest.
    STOP_STILL_MPS = 0.02
    STOP_STILL_RADPS = 0.05
    # HOW LONG AN OPERATOR STOP HOLDS THE IDLE LOOP, armed at EXECUTION time.
    # This is deliberately NOT --idle-resume-s: that window is armed when the
    # operator's prompt arrives and is sized for chat turns in general (this
    # world runs it at 12 s so a question does not park the line). A STOP is
    # the one command whose entire meaning is "stay stopped", so it gets the
    # "quiet minute" the shipped stop_robot tool description already promises
    # the model. It still AUTO-RESUMES: hold_until_told is the tool for an
    # indefinite hold, and a stop that never resumes can strand the line.
    STOP_HOLD_S = 60.0
    # Pose samples kept for the measurement above (~8 s at a 32 ms timestep).
    POSE_HIST_MAX = 256

    # Who started the motion currently occupying the single motion slot.
    # A motion an operator or an agent started is INVIOLABLE until it
    # finishes -- that is precisely the guarantee the tool descriptions
    # make. A motion the autonomy loop started yields: to an operator
    # command (the loop pauses for a quiet minute on one anyway) and to the
    # loop's own next leg, which is its long-standing self-recovery path.
    # Nothing ever waits on an idle-loop motion's measurement.
    SOURCE_EXTERNAL = "external"
    SOURCE_IDLE_LOOP = "idle_loop"

    def _begin_motion(self, source: str = SOURCE_EXTERNAL):
        """Claim the single motion slot. Returns ``(seq, refusal)``, of which
        exactly one is None.

        THE BUSY CHECK LIVES HERE, not in the HTTP route. On the route it
        covered one of the four ways to reach a motion: POST /tool -- the
        path an OmniLink agent's tool calls actually take -- plus the
        offline intent router and the idle loop all called act_* directly
        and silently clobbered whatever was running, while the shipped tool
        description promised the caller a 409.
        """
        with self.lock:
            kind, p = self.motion
            if kind in ("drive", "turn"):
                holder = p.get("source", self.SOURCE_EXTERNAL)
                if holder == self.SOURCE_EXTERNAL:
                    return None, {
                        "accepted": False,
                        "ok": False,
                        "error": "busy",
                        "http_status": 409,
                        "verb": p.get("verb", kind),
                        "message": (f"a {kind} is already in flight; this "
                                    f"command was NOT applied and the robot "
                                    f"is still executing the previous one."),
                        "details": {
                            "current": kind,
                            "current_seq": p.get("seq"),
                            "hint": ("wait for get_robot_state.mode == "
                                     "'idle', or pass wait=true so each "
                                     "command returns only once it has "
                                     "finished."),
                        },
                    }
                superseded = p
            else:
                superseded = None
            self.motion_seq += 1
            seq = self.motion_seq
        if superseded is not None and superseded.get("seq"):
            self._record_superseded(superseded, seq)
        return seq, None

    def _record_superseded(self, p: dict, by_seq, reason: str = "") -> None:
        """A motion ended before it could be measured. Say exactly that.

        achieved is null -- never a number nobody measured, and never the
        commanded value standing in for one."""
        self.last_completion = {
            "seq": int(p.get("seq", 0)),
            "verb": p.get("verb", "?"),
            "commanded": float(p.get("commanded", 0.0)),
            "achieved": None,
            "error": None,
            "unit": p.get("unit", ""),
            "settled": False,
            "timed_out": False,
            "superseded": True,
            "superseded_by": by_seq,
            "note": (reason or ("a later command ended this motion before it "
                                "finished; how far it got was not measured")),
            "sim_time": self.robot.getTime(),
        }

    def _supersede_inflight(self, reason: str) -> None:
        """Cancel whatever is in the motion slot and release its waiter.

        stop_robot and set_velocity deliberately do NOT reject when busy --
        they are the escape hatches, and a 'stop' that answers 409 is
        useless. But they do end the running motion, so its waiter has to be
        told, or it blocks for the full wait budget and then reports a
        timeout that never happened."""
        with self.lock:
            kind, p = self.motion
            victim = p if (kind in ("drive", "turn") and p.get("seq")) else None
        if victim is not None:
            self._record_superseded(victim, None, reason)

    # ── stop_robot ────────────────────────────────────────────────

    def _stop_measure_rest(self, t_halt: float,
                           halt_pose: Tuple[float, float, float]) -> dict:
        """Did the robot ACTUALLY come to rest? Answered from POSE SAMPLES.

        Never from v_linear/v_angular and never from the commanded velocity:
        this base is kinematic, so `_command_velocity(0, 0)` only stores a
        command, and the idle loop is free to put a fresh motion in the slot
        on the very next tick. Stationary here means THE POSE STOPPED
        CHANGING over a real interval, which is the only claim the tool is
        entitled to make."""
        blank = {
            "speed_mps": None, "yaw_rate_radps": None,
            "over_s": None, "samples": 0,
            "moved_since_halt_m": None, "turned_since_halt_rad": None,
            "basis": "supervisor pose samples taken by the simulation tick",
        }
        # THE SIM THREAD CANNOT WAIT FOR ITSELF. tick() is what produces the
        # samples, so blocking here (the Robot Window's stop button runs on
        # this thread) would measure nothing and stall the world. Say so
        # rather than inventing a verdict.
        if threading.current_thread() is threading.main_thread():
            blank["reason"] = (
                "the stop was executed on the simulation thread, which "
                "cannot advance while it waits, so no settling interval "
                "could be observed")
            return blank
        if time.time() - self.last_tick_at > 2.0:
            blank["reason"] = ("the simulation is not stepping, so no pose "
                               "samples are being produced")
            return blank
        deadline = t_halt + self.STOP_SETTLE_S
        while time.time() < deadline:
            time.sleep(min(self.WAIT_POLL_S, max(0.0, deadline - time.time())))
        with self.lock:
            after = [s for s in self._pose_hist if s[0] >= t_halt]
        if len(after) < 2:
            blank["samples"] = len(after)
            blank["reason"] = ("fewer than two pose samples arrived in the "
                               f"{self.STOP_SETTLE_S:.2f}s settling window "
                               "(is the world stepping?)")
            return blank
        # Residual speed over the TAIL of the window: a base that was still
        # decelerating when zero was commanded should not have its ramp
        # averaged into the answer.
        cut = t_halt + self.STOP_SETTLE_S * self.STOP_TAIL_FRAC
        tail = [s for s in after if s[0] >= cut]
        if len(tail) < 2 or (tail[-1][0] - tail[0][0]) < self.STOP_MIN_SPAN_S:
            tail = after[-2:] if (after[-1][0] - after[-2][0]) >= 1e-6 else []
        out = dict(blank)
        out["samples"] = len(after)
        hx, hy, hyaw = halt_pose
        out["moved_since_halt_m"] = math.hypot(after[-1][1] - hx,
                                               after[-1][2] - hy)
        out["turned_since_halt_rad"] = abs(wrap_pi(after[-1][3] - hyaw))
        out["pose_after_settle"] = {"x": after[-1][1], "y": after[-1][2],
                                    "yaw": after[-1][3]}
        if not tail:
            out["reason"] = ("pose samples arrived but spanned no measurable "
                             "interval, so no speed could be differenced")
            return out
        span = tail[-1][0] - tail[0][0]
        if span < self.STOP_MIN_SPAN_S:
            out["reason"] = (f"pose samples spanned only {span:.3f}s, below "
                             f"the {self.STOP_MIN_SPAN_S:.2f}s needed to "
                             "difference a speed")
            return out
        out["speed_mps"] = math.hypot(tail[-1][1] - tail[0][1],
                                      tail[-1][2] - tail[0][2]) / span
        out["yaw_rate_radps"] = abs(wrap_pi(tail[-1][3] - tail[0][3])) / span
        out["over_s"] = span
        return out

    def _stop_arm_hold(self) -> dict:
        """Make an operator stop HOLD the robot, from the moment it executes.

        WHY THIS IS NOT note_external_command: that arms the quiet window,
        and it is called when the operator's PROMPT lands (HTTP _route_post /
        handle_wwi_message), seconds before the model chooses a tool. With
        --idle-resume-s 12 a slow LLM turn outlives its own pause, which is
        exactly what was measured on tug_a: paused=False and idle_loop.leg
        back on to_park 1.5 s after a "stop" the model reported as halted.
        A stop is also never a trailing artefact of a resume, so it clears
        the post-resume exemption instead of being swallowed by it."""
        loop = self.idle_loop
        now = time.time()
        self._resume_exempt_until = 0.0
        self.prev_external_cmd = self.last_external_cmd
        self.last_external_cmd = now
        self.last_external_src = "stop_robot"
        if loop is None:
            self.stop_hold_until = 0.0
            return {"present": False,
                    "detail": "this robot has no autonomy loop to hold"}
        hold_s = max(self.STOP_HOLD_S, float(getattr(loop, "resume_s", 0.0)))
        self.stop_hold_until = now + hold_s
        return {"present": True, "paused": True, "hold_s": hold_s,
                "auto_resume": True, "auto_resume_at": self.stop_hold_until,
                "mode": getattr(loop, "mode", "?"),
                "leg_when_stopped": getattr(loop, "leg", "idle")}

    def act_stop(self, source: str = "external") -> dict:
        """Stop the wheels, then MEASURE and report whether they stopped.

        THE DEFECT THIS REPLACES, captured live: this returned
        `{accepted: True, halted_at: <ts>}` -- an acceptance and a timestamp,
        with no velocity, no pose and no confirmation of rest. The model
        relayed the only thing it was given ("I have halted immediately")
        while, measured 1.5 s later, the tug had moved 0.384 m and turned
        34.7 deg with mode=turn and v_angular=-0.90. PROTOCOL.md 5.4.1 rule 1
        and docs/developer/tool-design-for-agents.md: a result carries
        {commanded, achieved, error, settled}, never an acceptance dressed
        as an outcome.

        `source` mirrors act_drive_forward/act_turn. Anything other than
        SOURCE_EXTERNAL is an INTERNAL stop -- the idle loop's own
        obstacle-hold decel, or drive_to's abort -- and must neither hold the
        idle loop against itself nor spend a settling window inside the
        control path.
        """
        operator = (source == self.SOURCE_EXTERNAL)
        # Snapshot the victim BEFORE superseding it, so the result can name
        # what the stop actually cut short.
        with self.lock:
            kind, p = self.motion
            cut = ({"verb": p.get("verb", kind), "seq": p.get("seq"),
                    "commanded": p.get("commanded"), "unit": p.get("unit", "")}
                   if kind in ("drive", "turn") and p.get("seq") else None)
        self._supersede_inflight("stopped by stop_robot before it finished")
        try:
            halt_pose = self._read_pose()
        except Exception:
            halt_pose = (self.last_xy[0], self.last_xy[1], self.last_yaw)
        with self.lock:
            self.motion = ("idle", {})
        self._command_velocity(0.0, 0.0)
        halted_at = time.time()
        hold = self._stop_arm_hold() if operator else None
        out: Dict[str, Any] = {
            "accepted": True,
            "verb": "stop_robot",
            "halted_at": halted_at,
            # A stop commands one thing: zero speed. Reported in the same
            # {commanded, achieved, error, unit} shape as every other verb,
            # which is also what the relay's audit summariser renders.
            "commanded": 0.0,
            "unit": "m/s",
            "pose_at_halt": {"x": halt_pose[0], "y": halt_pose[1],
                             "yaw": halt_pose[2]},
            "stopped_motion": cut,
        }
        if not operator:
            out.update({"achieved": None, "error": None, "settled": None,
                        "stationary": None, "source": source,
                        "measured": {"basis": None,
                                     "reason": "internal stop (idle-loop hold "
                                               "or aborted leg): not measured, "
                                               "and it does not hold the "
                                               "autonomy loop"}})
            return out
        m = self._stop_measure_rest(halted_at, halt_pose)
        speed = m.get("speed_mps")
        stationary = (None if speed is None else
                      (speed <= self.STOP_STILL_MPS
                       and (m.get("yaw_rate_radps") or 0.0)
                       <= self.STOP_STILL_RADPS))
        out.update({
            "achieved": speed,
            "error": (None if speed is None else speed - 0.0),
            # `settled` == it is measurably at rest. Null, never True, when
            # nothing was measured.
            "settled": stationary,
            "stationary": stationary,
            "measured": m,
            "idle_loop": hold,
        })
        if stationary is None:
            out["note"] = (
                "the wheels were commanded to zero and the motion slot was "
                "cleared, but this bridge could NOT confirm the robot came "
                "to rest: " + str(m.get("reason", "no measurement")) + ". Do "
                "not tell the operator it has halted -- say it was commanded "
                "to stop and read get_robot_state to confirm.")
        elif stationary is False:
            out["note"] = (
                f"STILL MOVING {m['over_s']:.2f}s after the halt: "
                f"{speed:.3f} m/s and {m['yaw_rate_radps']:.3f} rad/s, "
                f"{m['moved_since_halt_m']:.3f} m travelled since the stop "
                "was issued. Tell the operator it has NOT stopped yet, and "
                "re-read get_robot_state before claiming otherwise.")
        else:
            out["note"] = (
                f"measured at rest: {speed:.3f} m/s over {m['over_s']:.2f}s, "
                f"{m['moved_since_halt_m']:.3f} m of coast after the halt.")
        if hold and hold.get("present"):
            out["note"] += (
                f" The autonomy loop is held for {hold['hold_s']:.0f}s and "
                "then RESUMES ON ITS OWN -- say that, do not promise it will "
                "stay stopped indefinitely. Call resume_autonomy to hand it "
                "back sooner, or hold_until_told for a stop with no "
                "auto-resume.")
        elif hold is not None:
            out["note"] += (" This robot has no autonomy loop, so nothing "
                            "will move it until you command it again.")
        return out

    def _record_completion(self, p: dict, achieved: float,
                           settled: bool, timed_out: bool) -> None:
        """Called from the tick when a motion ends. This is the ONLY place a
        motion result is written, and it writes what was MEASURED."""
        commanded = float(p.get("commanded", 0.0))
        self.last_completion = {
            "seq": int(p.get("seq", 0)),
            "verb": p.get("verb", "?"),
            "commanded": commanded,
            "achieved": achieved,
            "error": (None if achieved is None else achieved - commanded),
            "unit": p.get("unit", ""),
            "settled": bool(settled),
            "timed_out": bool(timed_out),
            "corrections": int(p.get("corrections", 0)),
            "sim_time": self.robot.getTime(),
        }

    def _await_completion(self, seq: int, budget_s: float) -> dict:
        """Block until motion `seq` reports in, or the budget expires.

        Polling rather than a condition variable: the tick already holds
        `self.lock` on every step, and an HTTP thread waiting on that same
        lock is a deadlock waiting for a slow world to find it."""
        deadline = time.time() + min(max(budget_s, 0.5), self.WAIT_MAX_S)
        while time.time() < deadline:
            done = self.last_completion
            # EXACTLY this motion's measurement, never a later one's. The
            # old `>= seq` handed a clobbered motion's waiter the CLOBBERING
            # motion's result: measured, internally consistent, and about a
            # completely different command. Proven -- an act_turn(1.5708)
            # clobbered by a drive returned {'verb': 'drive_forward',
            # 'commanded': 2.0, 'unit': 'm', 'achieved': 1.98,
            # 'settled': True}: wrong verb, wrong unit, wrong number, shaped
            # exactly like a valid answer to the turn.
            if done is not None and done["seq"] == seq:
                return dict(done)
            with self.lock:
                latest = self.motion_seq
            if latest > seq:
                return {"seq": seq, "achieved": None, "error": None,
                        "settled": False, "timed_out": False,
                        "superseded": True,
                        "note": ("a later command superseded this motion "
                                 "before it reported; how far it got was "
                                 "not measured -- reissue it, or read "
                                 "get_robot_state for the live pose")}
            time.sleep(self.WAIT_POLL_S)
        # The caller asked us to wait and we could not confirm. Say exactly
        # that -- never fall back to reporting the commanded value.
        return {"seq": seq, "achieved": None, "error": None,
                "settled": False, "timed_out": True,
                "note": "wait budget expired before the motion reported; "
                        "the robot may still be moving -- poll get_robot_state"}

    def act_set_velocity(self, linear: float, angular: float) -> dict:
        # Clamp to the platform's own limits and stamp an expiry (see tick).
        lin = clamp(float(linear), -self.v_max_linear, self.v_max_linear)
        ang = clamp(float(angular), -self.v_max_angular, self.v_max_angular)
        # Teleop overrides an in-flight drive/turn rather than being
        # refused -- but it does not get to do so silently.
        self._supersede_inflight(
            "overridden by set_velocity before it finished")
        with self.lock:
            self.motion = ("velocity", {"l": lin, "a": ang,
                                        "t0_sim": self.robot.getTime()})
        out = {"accepted": True, "linear": lin, "angular": ang,
               "expires_in_s": self.VELOCITY_MAX_S}
        if abs(lin - float(linear)) > 1e-6 or abs(ang - float(angular)) > 1e-6:
            out["clamped_from"] = {"linear": float(linear),
                                   "angular": float(angular)}
        return out

    def act_drive_forward(self, distance: float, speed: Optional[float] = None,
                          wait: bool = False,
                          timeout_s: Optional[float] = None,
                          source: str = SOURCE_EXTERNAL) -> dict:
        x, y, yaw0 = self._read_pose()
        actual_speed = speed if speed is not None else self.cruise_linear
        signed_speed = actual_speed if distance >= 0 else -actual_speed
        target = abs(distance)
        eta = target / max(abs(actual_speed), 1e-6)
        seq, refusal = self._begin_motion(source)
        if refusal is not None:
            return refusal
        with self.lock:
            self.motion = ("drive", {
                # yaw0 is the axis the command was EXPRESSED in, and the
                # axis the achieved distance is measured along (see the
                # drive branch of the tick).
                "x0": x, "y0": y, "yaw0": yaw0,
                "seq": seq, "verb": "drive_forward", "source": source,
                "commanded": float(distance), "unit": "m",
                "distance": target,
                "speed": signed_speed,
                "t0": time.time(),
                "t0_sim": self.robot.getTime(),
                # +6 s base: the settle-and-verify pass adds >=1 sim-s of
                # settle plus the slow final approach (see tick).
                "timeout_s": eta * 3.0 + 6.0,
                "phase": "approach",
                "corrections": 0,
            })
        if wait:
            wait_budget_s = eta * 3.0 + 12.0 if timeout_s is None else float(timeout_s)
            return {"accepted": True, "commanded": float(distance), "unit": "m",
                    **self._await_completion(seq, wait_budget_s)}
        return {"accepted": True, "seq": seq, "commanded": float(distance),
                "unit": "m", "eta_s": eta,
                "note": "NOT complete -- this returns on acceptance. Pass "
                        "wait=true, or poll get_robot_state until "
                        "last_command.seq == this seq, for the achieved "
                        "value. Match the seq EXACTLY: a later command's "
                        "record is a measurement of a different motion, and "
                        "a superseded motion reports achieved: null."}

    def act_turn(self, angle_rad: float, wait: bool = False,
                 source: str = SOURCE_EXTERNAL) -> dict:
        _, _, yaw = self._read_pose()
        target = wrap_pi(yaw + angle_rad)
        eta = abs(angle_rad) / max(self.spin_speed, 1e-6)
        seq, refusal = self._begin_motion(source)
        if refusal is not None:
            return refusal
        with self.lock:
            self.motion = ("turn", {
                "seq": seq, "verb": "turn", "source": source,
                "commanded": float(angle_rad), "unit": "rad",
                "target_yaw": target,   # reported only; the loop uses `remaining`
                "remaining": float(angle_rad),
                "t0": time.time(),
                "t0_sim": self.robot.getTime(),
                "phase": "plan",
                "corrections": 0,
                # Budget in SIM seconds. The skid-steer pivot saturates well
                # below the commanded spin under the MuJoCo solvers (measured
                # ~0.3-0.4 rad/s apparent on the Husky), and the settle-verify
                # pass adds >=1 sim-s per correction, so budget generously: the
                # turn converges inside TURN_TOL_RAD long before this fires.
                "timeout_s": (20.0 + 3.0 * abs(angle_rad)
                              / max(self.spin_speed * self.TURN_GAIN_TYPICAL,
                                    1e-3)),
            })
        if wait:
            return {"accepted": True, "commanded": float(angle_rad),
                    "unit": "rad",
                    **self._await_completion(seq, eta * 6.0 + 30.0)}
        return {"accepted": True, "seq": seq, "commanded": float(angle_rad),
                "unit": "rad", "eta_s": eta,
                "note": "NOT complete -- this returns on acceptance. Pass "
                        "wait=true, or poll get_robot_state until "
                        "last_command.seq == this seq, for the achieved "
                        "value. Match the seq EXACTLY: a later command's "
                        "record is a measurement of a different motion, and "
                        "a superseded motion reports achieved: null."}

    def act_drive_to(self, tx: float, ty: float,
                     wait: bool = True) -> dict:
        """Drive to an absolute (x, y) on the site. ALWAYS BLOCKING.

        This is a compound of turn + drive, and its entire value is that it
        completes and reports where the robot actually ended up. A
        non-blocking form would hand the caller back its own target, which is
        the failure this verb exists to remove -- so `wait=false` is refused
        rather than silently honoured."""
        if not wait:
            return {"accepted": False, "refused": "wait_false_unsupported",
                    "message": ("drive_to is always blocking -- it reports the "
                                "pose it actually reached. Use drive_forward "
                                "or set_velocity if you need a call that "
                                "returns before the motion finishes.")}
        if self._site_clamped(tx, ty):
            # Refuse and say why, with the bound named. A clamped command is a
            # wrong plan, not a successful drive to somewhere else.
            return {"accepted": False, "refused": "outside_site_bounds",
                    "message": (f"({tx:+.2f},{ty:+.2f}) is outside the site: "
                                f"|x| <= {self.SITE_HALF_X}, "
                                f"|y| <= {self.SITE_HALF_Y} metres."),
                    "bounds": {"half_x_m": self.SITE_HALF_X,
                               "half_y_m": self.SITE_HALF_Y}}
        x0, y0, _ = self._read_pose()
        legs = []
        # `settled` is EARNED, not asserted. Every leg here waits on
        # _await_completion, which honestly reports {achieved: None,
        # settled: False, timed_out: True} when its budget expires -- and
        # this loop used to answer that by breaking, reading the pose while
        # the robot was still driving, leaving the wheels turning, and
        # hardcoding "settled": True over the top of it.
        aborted = None          # why we stopped early, if we did
        timed_out = False
        for _ in range(self.DRIVE_TO_MAX_LEGS):
            x, y, yaw = self._read_pose()
            if math.hypot(tx - x, ty - y) <= self.DRIVE_TO_TOL_M:
                break
            heading_err = wrap_pi(math.atan2(ty - y, tx - x) - yaw)
            if abs(heading_err) > self.DRIVE_TO_HEADING_TOL:
                r = self.act_turn(heading_err, wait=True)
                legs.append({"turn": {"commanded_rad": heading_err,
                                      "achieved_rad": r.get("achieved")}})
                if r.get("accepted") is False:
                    aborted = r.get("error") or "turn_refused"
                    break
                if r.get("timed_out") or r.get("superseded"):
                    timed_out = bool(r.get("timed_out"))
                    aborted = "turn_timed_out" if timed_out else "turn_superseded"
                    break
            # Re-read AFTER the turn: the pivot moves the base a little, so a
            # distance computed before it is already stale.
            x, y, _ = self._read_pose()
            dist = math.hypot(tx - x, ty - y)
            r = self.act_drive_forward(dist, wait=True)
            legs.append({"drive": {"commanded_m": dist,
                                   "achieved_m": r.get("achieved")}})
            if r.get("accepted") is False:
                aborted = r.get("error") or "drive_refused"
                break
            if r.get("timed_out") or r.get("superseded"):
                timed_out = bool(r.get("timed_out"))
                aborted = "drive_timed_out" if timed_out else "drive_superseded"
                break
        if aborted is not None:
            # STOP BEFORE MEASURING. A pose sampled off a still-moving base
            # is not the pose the robot ended at, and every number below is
            # derived from it.
            #
            # INTERNAL: this is drive_to's own abort, not an operator "stop".
            # It must not spend act_stop's settling window (this method does
            # its own, below) and must not put a minute-long operator hold on
            # the idle loop for what is one verb giving up on a leg.
            self.act_stop(source=self.SOURCE_IDLE_LOOP)
            time.sleep(self.DRIVE_TO_ABORT_SETTLE_S)
        xf, yf, yawf = self._read_pose()
        err = math.hypot(tx - xf, ty - yf)
        out = {"accepted": True, "unit": "m",
               "commanded_xy": [tx, ty],
               "achieved_xy": [xf, yf],
               "achieved_yaw": yawf,
               "error_m": err,
               "arrived": err <= self.DRIVE_TO_TOL_M,
               "tolerance_m": self.DRIVE_TO_TOL_M,
               "start_xy": [x0, y0],
               "legs": legs,
               "timed_out": timed_out,
               "settled": aborted is None}
        if aborted is not None:
            out["aborted"] = aborted
            out["note"] = ("a leg did not complete, so the robot was stopped "
                           "before this pose was read: it is where the robot "
                           "gave up, not where it was asked to go")
        return out

    def act_reset_to_home(self) -> dict:
        try:
            self.self_node.getField("translation").setSFVec3f(list(self.start_xyz))
            # Reset orientation to identity-yaw.
            self.self_node.getField("rotation").setSFRotation([0, 0, 1, self.start_yaw])
            self.self_node.resetPhysics()
        except Exception as e:
            return {"error": f"reset_failed: {e}"}
        with self.lock:
            self.motion = ("idle", {})
        self._command_velocity(0.0, 0.0)
        return {"accepted": True, "start_xyz": self.start_xyz}

    def peer_report(self) -> dict:
        """What the OTHER robots on this site are doing, from the polling
        this bridge already does. See MavIdleLoop.peer_report."""
        loop = self.idle_loop
        if loop is None or not hasattr(loop, "peer_report"):
            return {
                "peer": {
                    "known": False,
                    "reason": "this robot runs no site loop, so it watches "
                              "nobody",
                    "say": ("I don't keep tabs on any other robot — I only "
                            "move when you tell me to."),
                },
            }
        return loop.peer_report()

    def _paused_by(self) -> str:
        """Which of the three pause sources is holding the idle loop, in the
        same precedence MavIdleLoop._blocked applies."""
        loop = self.idle_loop
        if loop is None:
            return ""
        if self.intents is not None and self.intents.hold_active():
            return "operator hold"
        if time.time() < self.stop_hold_until:
            return "operator stop"
        if (time.time() - self.last_external_cmd) < loop.resume_s:
            return "recent command"
        return ""

    # ── SENSOR READ-THROUGH (PROTOCOL.md §6.6) ───────────────────────

    def _measure_mounts(self) -> Dict[str, Any]:
        """Mount pose of every sensor in the robot's frame. Sim thread only.

        Measured through the supervisor rather than assumed, so a consumer can
        build a real TF tree. Cached after the first successful read: these are
        fixed joints on the robot, so they do not move."""
        mounts: Dict[str, Any] = {}
        try:
            self_pos = self.self_node.getPosition()
            self_rot = self.self_node.getOrientation()
        except Exception:
            return {}
        for name, dev in self.sensor_devices.items():
            try:
                # ⚠ getFromDevice takes the integer DEVICE TAG, not the Device
                # object -- passing the object builds Node(tag=<Device>) and
                # silently yields no reference, which reads as "this robot has
                # no measurable mounts" rather than as an error.
                tag = getattr(dev, "_tag", None)
                if tag is None:
                    continue
                node = self.robot.getFromDevice(tag)
                if node is None:
                    continue
                m = relative_mount(self_pos, self_rot,
                                   node.getPosition(), node.getOrientation())
                if m is not None:
                    mounts[name] = m
            except Exception:
                continue
        return mounts

    def list_sensors(self) -> dict:
        """Catalog: what this robot carries, what a read returns, and where
        each device is mounted relative to the robot."""
        if self._sensor_mounts is None and self.sensor_devices:
            measured = self.mt.call(self._measure_mounts)
            # Only cache a real answer; a timeout must not freeze an empty map
            # in place for the life of the process.
            if measured:
                self._sensor_mounts = measured
        mounts = self._sensor_mounts or {}
        out = []
        for entry in self.sensor_catalog:
            item = dict(entry)
            mount = mounts.get(entry["name"])
            # Explicit null, not a fabricated identity: "we could not measure
            # where this is" and "it sits exactly at the robot origin" are very
            # different claims.
            item["mount"] = mount
            out.append(item)
        return {"sensors": out, "count": len(out),
                "mount_frame": "robot_base",
                "mounts_measured": bool(mounts)}

    def read_sensor(self, name: str) -> dict:
        """Read one named device.

        The actual device call is marshalled onto the sim thread -- issuing it
        from this HTTP worker thread would stall the controller<->sim step
        exchange (see MainThreadCalls)."""
        dev = self.sensor_devices.get(name)
        if dev is None:
            # PROTOCOL.md §6.6's `available: false` is the structurally
            # correct answer for "this bridge cannot give you that", and it
            # is a 200, not an error. Naming the sensors that DO exist turns
            # a dead end into one more call.
            known = [s["name"] for s in self.sensor_catalog]
            note = (f"no sensor named {name!r} on this robot; "
                    f"available: {', '.join(known)}" if known else
                    "this robot exposes no readable sensors. A URDF robot "
                    "needs the world loaded with OMNISIM_URDF_USE_SENSORS=1 "
                    "for its <gazebo> sensor blocks to become devices.")
            return {"available": False, "sensor": name, "note": note,
                    "known_sensors": known}
        result = self.mt.call(
            lambda: read_one_sensor(dev, name, self.timestep, self.robot.getTime())
        )
        if result is None:
            # mt.call timed out: the sim thread is not pumping. Say so rather
            # than returning a shape that looks like a reading.
            return {"available": False, "sensor": name,
                    "note": "the simulation is not stepping, so no sample "
                            "could be taken"}
        return result

    def get_state(self) -> dict:
        x, y, yaw = self._read_pose()
        with self.lock:
            kind = self.motion[0]
        out = {
            "id": self.robot_id,
            "model": self.cfg["model"],
            "x": x, "y": y, "yaw": yaw,
            "v_linear": self.v_linear,
            "v_angular": self.v_angular,
            "mode": kind,
            "fault": self.fault,
            "last_tick_at": self.last_tick_at,
            "sim_time": self.robot.getTime(),
            # What the last finished motion ACTUALLY did (PROTOCOL.md 5.4.1).
            # None until one completes. This is how a caller that did not pass
            # wait=true learns the achieved value instead of assuming it.
            "last_command": self.last_completion,
        }
        # DEFERRED INTENTS, top level and always present. A pending intent
        # has to be VISIBLE, not implied: this is what get_robot_state's
        # description points the model at so it reads its own standing
        # orders back instead of inventing them.
        if self.intents is not None:
            out.update(self.intents.state())
        if self.pallet_feature:
            out["carrying"] = self.carrying
            tow = self._towed_telemetry(x, y, yaw)
            if tow is not None:
                out["towed"] = tow
        if self.idle_loop is not None:
            # on_lane / leg are the peer-visible half of the lane lock: the
            # other tug polls this block before entering the transit lane.
            out["idle_loop"] = {
                "on_lane": bool(getattr(self.idle_loop, "on_lane", False)),
                # The pick-cell column mutex, peer-visible: this is the whole
                # protocol surface -- the other tug reads want/want_t/hold/
                # lease_until out of here and nothing else.
                "col": (self.idle_loop._col_state()
                        if hasattr(self.idle_loop, "_col_state") else None),
                "leg": getattr(self.idle_loop, "leg", "idle"),
                "mode": getattr(self.idle_loop, "mode", "tow"),
                "cycles": self.idle_loop.cycles,
                "paused": self.idle_loop._blocked(),
                # WHY it is paused, and for how long. `paused` alone made an
                # operator stop indistinguishable from a stale quiet window,
                # so the model could not tell the operator whether the robot
                # would set off again on its own.
                "paused_by": self._paused_by(),
                "resumes_in_s": (
                    round(self.stop_hold_until - time.time(), 1)
                    if time.time() < self.stop_hold_until else None),
                "conveying": [c["def"] for c in self._carries
                              if c["phase"] == "carry"],
                # NAMED FOR WHAT THEY ARE. `parked`/`delivered` read as "carts
                # I parked" and a model duly reported them as its own work --
                # measured: "I have parked a total of 4 carts this shift,
                # specifically TROLLEY_D/E/F/G" from a robot whose
                # delivered_total was 0, because those four were placed at
                # world init. They are a census of the park row, not a tally
                # of this robot's labour. The old keys stay for compatibility
                # with existing consumers; the park_row_* names are the ones
                # that cannot be misread.
                "park_row_occupants": list(getattr(self.idle_loop, "park_order", [])),
                "park_row_count": len(getattr(self.idle_loop, "park_order", [])),
                "parked": list(getattr(self.idle_loop, "park_order", [])),
                "delivered": len(getattr(self.idle_loop, "park_order", [])),
                # MONOTONIC tally of carts parked this session. `delivered`
                # above is the row's current occupancy and goes DOWN when a
                # cart is collected out, so it is not a running total --
                # this is.
                #
                # ⚠️ THIS IS A PARKS-ONLY TALLY AND ITS NAME DOES NOT SAY SO.
                # _park_in_spot is the only thing that bumps it and that is
                # dispatch-only code, so on the RETURN tug it is structurally
                # 0 however many carts that tug has moved. MEASURED: asked
                # "how many carts have you delivered?" after two EMPTY CART
                # DELIVERED events and two loaded-cart shuttles, tug_b
                # answered 0 -- it read this field, and this field could not
                # have said anything else. Kept at its exact old value and
                # meaning because consumers depend on it (the goals suite
                # reads idle_loop.delivered_total, and the QA probes score
                # "how many did YOU park in that row" against it); the honest
                # names are the three keys below, and
                # carts_delivered_total is the one that answers the
                # operator's question on EITHER tug.
                "delivered_total": getattr(self.idle_loop, "parks_total", 0),
                # NAMED FOR WHAT THEY COUNT. carts_parked_total is
                # delivered_total under a name that cannot be misread;
                # carts_returned_total is the return tug's equivalent (empty
                # carts put on the fill spot for the arm); carts_delivered_
                # total is their sum, which is the role-correct answer to
                # "how many carts have you delivered this session" because a
                # tug only ever runs one of the two loops. The _means string
                # carries the definition next to the number, so a reader never
                # has to infer it from the key.
                "carts_parked_total": getattr(self.idle_loop, "parks_total", 0),
                "carts_returned_total": getattr(self.idle_loop,
                                                "returns_total", 0),
                "carts_delivered_total": self.idle_loop._carts_delivered(),
                "carts_delivered_means": self.idle_loop._delivery_meaning(),
                # Every completed job, deliveries + collections. This is what
                # a deferred "after the current task" waits for.
                "jobs_total": getattr(self.idle_loop, "jobs_total", 0),
                # Obstacle-awareness telemetry: whether this tug is currently
                # yielding/holding for an obstacle, why, and a cached snapshot
                # of every cart's (x, y) so an observer can check clearances
                # without its own supervisor read.
                "holding": bool(getattr(self.idle_loop, "holding", False)),
                "hold_reason": getattr(self.idle_loop, "hold_reason", ""),
                "holds_total": getattr(self.idle_loop, "holds_total", 0),
                "hold_secs_max": round(
                    getattr(self.idle_loop, "hold_secs_max", 0.0), 2),
                "cart_xy": {
                    d: [round(p[0], 3), round(p[1], 3)]
                    for d, p in getattr(
                        self.idle_loop, "_cart_cache", (0.0, {}))[1].items()
                },
            }
        return out


# ── Intent router ────────────────────────────────────────────────────

def _busy_reply(res: dict, verb: str) -> dict:
    """Turn a busy refusal into an offline-router answer that says so.

    The router is one of the entry points that used to clobber a running
    motion outright. Now that _begin_motion refuses, the router must not
    narrate a move that never started -- and must not touch res['eta_s'],
    which a refusal does not carry."""
    return {
        "agent": ("I'm still finishing the last move, so I did NOT start "
                  "that one. Ask again once I've stopped."),
        "tools": [(verb, "busy", res.get("message", "busy"))],
    }


class IntentRouter:
    NUMBER = r"(-?\d+\.?\d*)"

    def __init__(self, bridge: MobileBridge):
        self.bridge = bridge

    def dispatch(self, text: str) -> dict:
        s = text.strip().lower()
        if not s:
            return {"agent": "(empty prompt)", "tools": []}

        # ── resume / carry on ───────────────────────────────────
        # MUST be first: "back to work" contains "back" (which the
        # drive-backwards rule below would eat) and "keep going" reads
        # as a motion verb. This is the counterpart to "stop" -- without
        # it the operator can halt the tug from chat and has no way to
        # restart it short of waiting out the quiet-window timer.
        if shared_is_resume is not None and shared_is_resume(s):
            res = self.bridge.act_resume_autonomy()
            if res.get("autonomy") == "none":
                return {
                    "agent": "I have no autonomous loop to resume — "
                             "I only move when you tell me to.",
                    "tools": [("resume_autonomy", "ok", "no idle loop")],
                }
            return {
                "agent": f"Back on it — rejoining the line now "
                         f"({res.get('cycles', 0)} cycles done so far).",
                "tools": [("resume_autonomy", "ok",
                           f"autonomy={res.get('autonomy')}")],
            }

        # ── "what are you doing right now?" ─────────────────────
        # Answered from the real /state dict (which cart, which leg,
        # whether the loop is paused), not from raw odometry -- the
        # legacy `status|state|where|pose` intent below still answers
        # with x/y/yaw for operators who ask for pose specifically.
        if shared_is_status is not None and shared_is_status(s):
            st = self.bridge.get_state_for_query()
            return {
                "agent": shared_describe_state(st),
                "tools": [("get_robot_state", "ok",
                           str((st.get("idle_loop") or {}).get("leg")
                               or st.get("mode")))],
            }

        # ── "what's your job / which cart are you moving?" ──────
        if re.search(r"\b(your (job|role|task|assignment|purpose)|"
                     r"what do you do|what are you for|"
                     r"which (cart|trolley|pallet)|what cart|"
                     r"where (do|are) you tak(e|ing) (it|them|the cart)|"
                     r"what'?s your (job|role|purpose))\b", s):
            return {
                "agent": self.bridge.describe_role(),
                "tools": [("get_robot_state", "ok", "role brief")],
            }

        if re.search(r"\b(stop|halt|freeze|brake)\b", s):
            # OFFLINE ROUTER, SAME CONTRACT. "Stopping wheels." was the
            # regex router's version of the defect the LLM path had: a
            # sentence written before anything was measured. Say what the
            # stop MEASURED, or say plainly that it could not be confirmed.
            res = self.bridge.act_stop()
            still = res.get("stationary")
            m = res.get("measured") or {}
            if still is True:
                agent = (f"Stopped — measured {m['speed_mps']:.3f} m/s over "
                         f"{m['over_s']:.2f} s, so it is standing still.")
                summary = f"stationary, {m['speed_mps']:.3f} m/s"
            elif still is False:
                agent = (f"Wheels commanded to zero, but it is STILL MOVING: "
                         f"{m['speed_mps']:.3f} m/s "
                         f"{m['over_s']:.2f} s after the halt.")
                summary = f"NOT stationary, {m['speed_mps']:.3f} m/s"
            else:
                agent = ("Wheels commanded to zero. I could not confirm it "
                         "came to rest — " + str(m.get("reason", "not "
                                                       "measured")) + ".")
                summary = "rest unconfirmed"
            hold = res.get("idle_loop") or {}
            if hold.get("present"):
                agent += (f" Holding for {hold['hold_s']:.0f} s, then it "
                          f"resumes its own job unless you say otherwise.")
            return {
                "agent": agent,
                "tools": [("stop_robot", "ok", summary)],
            }

        if re.search(r"\b(reset|home|teleport.*start)\b", s):
            res = self.bridge.act_reset_to_home()
            ok = "error" not in res
            return {
                "agent": "Teleporting back to start." if ok else res["error"],
                "tools": [("reset_to_home", "ok" if ok else "err",
                          f"start=({res.get('start_xyz', ['?', '?'])[0]:.2f}, {res.get('start_xyz', ['?', '?'])[1]:.2f})" if ok else res["error"])],
            }

        # Spin in place.
        if re.search(r"\b(spin|rotate)\b", s) and not re.search(r"degree|rad|left|right", s):
            self.bridge.act_set_velocity(0.0, self.bridge.spin_speed)
            return {
                "agent": (f"Spinning in place for at most "
                          f"{self.bridge.VELOCITY_MAX_S:.0f} seconds."),
                "tools": [("set_velocity", "ok", f"a={self.bridge.spin_speed:+.2f} rad/s")],
            }

        if re.search(r"\b(circle|loop)\b", s):
            self.bridge.act_set_velocity(self.bridge.cruise_linear * 0.6, self.bridge.spin_speed * 0.6)
            return {
                "agent": (f"Driving in a circle for at most "
                          f"{self.bridge.VELOCITY_MAX_S:.0f} seconds."),
                "tools": [("set_velocity", "ok", "circle")],
            }

        # Turn N degrees / radians, with optional direction.
        m = re.search(r"turn(?:\s+(left|right))?\s+(?:about\s+)?" + self.NUMBER + r"\s*(deg|degree|rad|radian)?", s)
        if m:
            direction = m.group(1)
            val = float(m.group(2))
            unit = (m.group(3) or "deg").lower()
            angle = val if unit.startswith("rad") else math.radians(val)
            if direction == "right":
                angle = -abs(angle)
            elif direction == "left":
                angle = abs(angle)
            res = self.bridge.act_turn(angle)
            if res.get("accepted") is False:
                return _busy_reply(res, "turn")
            return {
                "agent": f"Turning {math.degrees(angle):+.0f}° (~{res['eta_s']:.1f}s).",
                "tools": [("turn", "ok", f"angle={math.degrees(angle):+.0f}°")],
            }
        if re.search(r"\bturn (left|right)\b", s):
            direction = re.search(r"\bturn (left|right)\b", s).group(1)
            sign = 1 if direction == "left" else -1
            res = self.bridge.act_turn(sign * math.pi / 2)
            if res.get("accepted") is False:
                return _busy_reply(res, "turn")
            return {
                "agent": f"Turning {direction} 90°.",
                "tools": [("turn", "ok", f"angle={sign*90}°")],
            }
        if re.search(r"\b(turn around|u[- ]?turn|about face)\b", s):
            res = self.bridge.act_turn(math.pi)
            if res.get("accepted") is False:
                return _busy_reply(res, "turn")
            return {
                "agent": "Turning around (180°).",
                "tools": [("turn", "ok", "angle=180°")],
            }

        # Drive forward / back N (meters|m|cm).
        m = re.search(r"\b(forward|forwards|ahead|back|backward|backwards|reverse)\b[^-\d]*" + self.NUMBER + r"\s*(m|meter|metre|metres|meters|cm|centi)?", s)
        if m:
            direction = m.group(1)
            val = float(m.group(2))
            unit = (m.group(3) or "m").lower()
            d = val / 100.0 if unit.startswith("cm") or unit.startswith("centi") else val
            if direction.startswith("back") or direction == "reverse":
                d = -d
            res = self.bridge.act_drive_forward(d)
            if res.get("accepted") is False:
                return _busy_reply(res, "drive_forward")
            return {
                "agent": f"Driving {direction} {abs(d):.2f} m (~{res['eta_s']:.1f}s).",
                "tools": [("drive_forward", "ok", f"distance={d:+.2f} m")],
            }
        m = re.search(r"\b(forward|forwards|ahead)\b", s)
        if m:
            res = self.bridge.act_drive_forward(1.0)
            if res.get("accepted") is False:
                return _busy_reply(res, "drive_forward")
            return {
                "agent": "Driving forward 1 m by default.",
                "tools": [("drive_forward", "ok", "1.00 m")],
            }
        m = re.search(r"\b(back|reverse)\b", s)
        if m:
            res = self.bridge.act_drive_forward(-1.0)
            if res.get("accepted") is False:
                return _busy_reply(res, "drive_forward")
            return {
                "agent": "Backing up 1 m by default.",
                "tools": [("drive_forward", "ok", "-1.00 m")],
            }

        # Set velocity directly: "set velocity 0.3 0.0"
        m = re.search(r"(?:set\s+)?velocity\s+" + self.NUMBER + r"[,\s]+" + self.NUMBER, s)
        if m:
            lin = float(m.group(1))
            ang = float(m.group(2))
            self.bridge.act_set_velocity(lin, ang)
            return {
                "agent": f"Setting velocity to ({lin:.2f} m/s, {ang:.2f} rad/s).",
                "tools": [("set_velocity", "ok", f"({lin:+.2f}, {ang:+.2f})")],
            }

        # State query.
        if re.search(r"\b(status|state|where|pose|telemetry|odometry)\b", s):
            st = self.bridge.get_state()
            return {
                "agent": f"x={st['x']:+.2f}, y={st['y']:+.2f}, yaw={math.degrees(st['yaw']):+.0f}°, v={st['v_linear']:+.2f} m/s, mode={st['mode']}.",
                "tools": [("get_robot_state", "ok", st['mode'])],
            }

        return {
            "agent": ("I don't recognise that. Try: "
                      "\"forward 1 m\", \"back 50 cm\", "
                      "\"turn left 90 degrees\", \"turn around\", "
                      "\"spin\", \"stop\", \"reset\"."),
            "tools": [],
        }


# ── Idle demo loop (opt-in, --idle-loop tow) ─────────────────────────

class MavIdleLoop(threading.Thread):
    """Ambient tug loop that keeps the warehouse ALIVE at idle. Opt-in, and
    always as one of a PAIR: tug_a runs mode 'dispatch', tug_b runs mode
    'trolley_return', and between them they close the cart ring.

    THE INVARIANT THE WHOLE LOOP SERVES: a cart only ever moves because a
    tug is towing it or a conveyor is running it. There is no auto-park, no
    recall glide, no "restore the canonical pose" — those all existed here
    once and every one of them read as the world cheating. If a cart needs to
    cross a gap, either a tug goes and gets it or the world has a machine for
    it (see MobileBridge.start_cart_conveyor).

    mode 'dispatch' (tug_a) — owns everything EAST of the pick-cell gate:
        A loaded cart appears at CART_PICKUP (the fill conveyor runs it right
        through the station and sets it down there, so this tug is never on
        the arm's critical path -- see step A of _cycle_return for the
        measurements that forced that). Dock it, tow it north onto the transit
        lane and east along it, pick a FREE
        spot in the PARK ROW by reading every cart's live pose, push it in
        from the south, detach, and come home the long way round the back
        aisle — a one-way loop, so the tug never reverses past parked carts.
        When the row is FULL the next job is a COLLECTION instead: dock the
        oldest parked cart, tow it to CART_OUTBOUND_INFEED, and let the
        outbound conveyor take it out through dock E. At the door it is
        handed to the inbound side, which is what makes the ring endless.

    mode 'trolley_return' (tug_b) — owns everything WEST of the gate:
        Runs the fill conveyor in BOTH directions (empty in when the fill
        spot clears, loaded out when the line master says the box is
        aboard), tows empties from CART_LANE_PICKUP to CART_STAGE while
        tug_a is still hauling (this is the overlap that keeps the arm from
        waiting), moves a staged cart onto the station the moment it frees,
        and runs the cart-lane conveyor to pull the next empty in off dock 2.

    Roles are partitioned BY PLACE, not by negotiation: the two sets of
    positions are disjoint, so the tugs cannot select the same cart. The only
    shared resource is the east-west transit lane, and that alone gets a lock
    (``on_lane``, published in /state and polled by the peer).

    Interruption contract + crash safety are unchanged: any operator command
    pauses the loop instantly (their motion plan simply replaces ours), it
    resumes after ``resume_s`` of quiet, and every step is fail-soft. On
    resume the loop re-reads the world rather than trusting saved state — if
    it is still towing something it finishes that job, otherwise it just
    starts a fresh pass. Nothing here votes, elects, or hands off a token.

    All geometry is read live from world DEFs (see the *_DEF constants), so
    moving a painted marker moves the choreography.
    """

    Y_LANE = 0.7        # east-west transit lane y (clear of fences + staging)
    Y_RETURN = -3.0     # southern lane empty tugs go home on (keeps the
                        # return clear of a tow running the transit lane)
    Y_OUTBOUND = -1.0   # the dock-E approach lane (south of the transit lane)
    PARK_AISLE_Y = 6.6  # back aisle behind the park row (the return half of
                        # tug_a's one-way loop)
    PARK_LINK_X = 4.5   # north-south link joining the back aisle to the lane
    # tug_b's between-jobs hold. Its whole beat is at the west end, so
    # trailing all the way back to the staging zone after every short leg
    # added ~16 m of pointless driving to each pass -- and that time lands
    # squarely on the arm, which is waiting for the next empty cart. Holding
    # here instead keeps it two moves from any of its jobs while staying off
    # the transit lane (y=0.7), off the cart lane (y=1.0) and out of tug_a's
    # dock approach to the station (the x=-8 column).
    # ...and now ALSO tug_b's column-mutex queue point, which is why it moved
    # 0.6 m further west (was -11.0): it has to sit clear OUTSIDE the COL_*
    # rect, or a tug waiting for the column would be standing in it.
    WEST_HOLD = (-11.6, -1.2)
    PRE_DOCK = 1.55     # stand-off from the hitch before backing on
    DOCK_REVERSE = 1.05  # nominal reverse distance onto the hitch
    # Stand-off used by a 'south' approach tow before the final push. 0.75 m,
    # not the old 0.25: at 0.25 the pivot onto the southward heading happened
    # at (station_x, 1.30), where the tug's swept circle reaches the fill
    # conveyor's west guide kerb AND its drive head. Turning 0.5 m further
    # south clears both by >0.09 m and costs nothing -- the final push
    # distance is computed from the live pose, so the cart still lands on the
    # station to the same tolerance.
    #
    # 1.05, not 0.75: at 0.75 the tug ARRIVES at the stand-off on a NE
    # diagonal (it comes off the stage dock 0.5 m to the south-west), and a
    # diagonal tug is a wide tug -- its front-right corner passed 0.03 m from
    # the fill conveyor's east kerb. Standing off another 0.3 m makes that
    # last approach leg run almost due east, so the corner clears by 0.49 m
    # and the guard has no reason to fire on the way in.
    SOUTH_APPROACH = 1.05
    # A hold on a STATIC structure is not worth MAX_HOLD_S: a kerb will never
    # move, so waiting on one is pure dead time. Abort the leg quickly and let
    # the route planner (which is static-aware, see _static_path_hit) pick a
    # different line on the retry.
    STATIC_HOLD_S = 4.0
    # How close a guarded leg has to get before it counts as ARRIVED. This is
    # the accuracy EVERY _goto delivers, so it is also the honest yardstick
    # for "would issuing this leg achieve anything?" -- _dock uses it to
    # refuse a sub-tolerance reposition. It was a local inside _drive_guarded
    # AND a literal default on _goto; both now read it from here, so the
    # arrival band is one number rather than three that can drift apart.
    ARRIVE_TOL = 0.12
    # How much of a leg may be left UNDRIVEN and still count as done. The
    # band that decides "re-aim at the remainder" versus "accept it".
    #
    # A guarded leg whose commanded drive completes while still outside
    # ARRIVE_TOL used to be re-aimed unconditionally: _reissue_drive faces
    # atan2 of the REMAINING vector and drives it. For a remainder of a few
    # centimetres that vector is not a direction, it is noise -- and
    # specifically it is PERPENDICULAR noise, because the residual of a
    # distance-accurate drive down a heading that was off by a fraction of a
    # degree is cross-track by construction.
    #
    # MEASURED LIVE, tug_a arriving at the park row, 3 of 3 dispatch cycles
    # (236 / 266 / 260 deg of turning). The t=733 cycle, traced in full:
    #   * _goto's _face returns satisfied at up to its 0.06 rad (3.4 deg)
    #     tolerance; a 0.6 deg residual over the 13.4 m east run leaves
    #     0.139 m of error at the target -- 0.021 m of it along x (the
    #     second-order 13.4*(1-cos) term), 0.138 m of it cross-track in y;
    #   * 0.139 > ARRIVE_TOL, so the idle branch re-aimed at atan2 of that
    #     14 cm residual. Bearing -91.2 deg: square across the leg. Pulse 1
    #     delivered -92.5 deg;
    #   * it then crawled 5.6 cm (0.139 -> 0.083 m) and the leg arrived;
    #   * _park_in_spot's next _face(pi/2) therefore started from yaw
    #     -91.6 deg -- wrap_pi(181.6) = -178.4 deg, the worst-case near-180
    #     turn, split by TURN_PULSE_MAX_RAD into 162 + 16.
    # Net: -270.9 deg of rotation to achieve +89.1 deg, over 6.3 s, with a
    # 22 kg cart whipping round behind it, to gain 5.6 cm.
    #
    # WHY THE OLD GUARD NEVER FIRED. _reissue_drive already had a skip band
    # -- a literal `rem <= 0.10`. That is NARROWER than the 0.12 arrival
    # band the loop tests one branch earlier, so it is unreachable by
    # construction: the loop returns ARRIVED at 0.12 before rem can ever be
    # seen at 0.10. Two numbers for one band, in the wrong order, is the
    # whole defect; this is that band, and _reissue_drive now reads it too
    # rather than keeping a second opinion.
    #
    # WHY 0.20 AND NOT MORE. It must sit strictly ABOVE ARRIVE_TOL or it is
    # unreachable in exactly the way 0.10 was. Its ceiling is set by the one
    # axis of the park nothing corrects:
    #   y -- CLOSED. _park_in_spot re-measures the CART after the push north
    #        and nudges until |dy| <= PARK_PLACE_TOL (4 tries), so
    #        cross-track error at the lane -- which is where essentially all
    #        of this residual lands -- is corrected out.
    #   x -- OPEN. Nothing re-measures it; it carries into the cart's
    #        resting x.
    # PARK_SPOT_n paints its side lines at +-0.55 with a 0.07 m stripe, so
    # the inner channel is 2*(0.55 - 0.035) = 1.030 m for a 0.70 m cart:
    # 0.165 m of slack per side. Measured placement error today is x 0.056 m
    # worst case, systematic to 3 mm -- and that was produced with the
    # residual capped at ARRIVE_TOL, so raising the cap to here adds at most
    # (0.20 - 0.12) = 0.08 m to it: 0.136 m worst case against 0.165 m of
    # budget. Overrunning it would be cosmetic in any case (the lines are
    # paint with no boundingObject; neighbours sit 1.40 m apart for 0.70 m
    # carts, and SPOT_TOL / OCCUPY_TOL are 0.55 / 1.05, both far larger) --
    # but 0.029 m of margin is why this is 0.20 rather than something
    # rounder and larger.
    #
    # NOT a general arrival tolerance. A leg that genuinely ended short --
    # a remainder a hold ate into, an avoidance detour, a peer yield -- is
    # metres, not centimetres, and is still re-faced and re-driven.
    REAIM_MIN_M = 0.20
    LONG_SPEED = 1.0    # m/s on the long east-west legs (kinematic AMR)
    TOW_SPEED = 0.6     # m/s while towing through the tighter west end

    # World DEFs this loop reads. Every one is a painted marker or a machine
    # that is visible in the world -- there are no invisible waypoints.
    STATION_DEF = "CONVEYOR_STATION"      # the EMPTY-cart end of the shuttle
    PICKUP_DEF = "CART_PICKUP"            # the LOADED-cart end (see below)
    STAGE_DEF = "CART_STAGE"              # tug_b's pre-fetch holding spot
    FILL_DOG_DEF = "FILL_CONVEYOR_DOG"
    LANE_PICKUP_DEF = "CART_LANE_PICKUP"
    LANE_ENTRY_DEF = "CART_INBOUND_ENTRY"
    LANE_DOG_DEF = "CART_LANE_DOG"
    OUT_INFEED_DEF = "CART_OUTBOUND_INFEED"
    OUT_EXIT_DEF = "CART_OUTBOUND_EXIT"
    OUT_DOG_DEF = "OUTBOUND_DOG"
    PARK_SPOT_PREFIX = "PARK_SPOT_"
    MAX_PARK_SPOTS = 24                   # scan ceiling; the world sets the
                                          # real count by how many it defines

    SPOT_TOL = 0.55     # a trolley within this xy of a spot is "at" it
    # A park spot counts as TAKEN from further away than a cart counts as
    # "at" it. The asymmetry is deliberate and load-bearing: a cart that
    # parked sloppily (measured: 0.66 m short, because a one-trailer tow does
    # not stop where the arithmetic says it will) would otherwise leave its
    # own spot reading FREE, and the next dispatch would drive a second cart
    # into the same square. Overlapping carts is the one thing the row must
    # never do, so occupancy is tested generously.
    OCCUPY_TOL = 1.05
    PARK_PLACE_TOL = 0.12   # closed-loop stop band when pushing a cart in
    # Keep this many spots free, and collect during DEAD TIME to maintain it.
    #
    # Why a reserve rather than "collect when full": the cart ring is
    # conserved, so in steady state every park must be matched by a
    # collection -- tug_a genuinely has to do both every cycle. The question
    # is only WHEN. Collecting on a full row means collecting while a loaded
    # cart is already waiting at the station, which blocks the station, which
    # blocks the next empty, which stalls the ARM for the whole ~110 s round
    # trip (measured: "box BOX_3 filled -- waiting for an empty trolley"
    # repeating). Collecting one cart early instead moves that same work into
    # the window where tug_a would otherwise be driving home empty and the arm
    # is busy filling the next box. Same total work, far less dead air.
    PARK_RESERVE = 1
    LANE_WAIT_S = 50.0  # bounded wait for the peer to clear the transit lane
    CARRY_WAIT_S = 90.0  # bounded wait for a conveyor carry to finish

    # ══ OBSTACLE AVOIDANCE ════════════════════════════════════════════
    # The AMR tug is KINEMATIC (its URDF has no collision shapes -- it phases
    # through everything), so nothing in the solver stops one tug driving into
    # the other or into a parked cart. Avoidance is therefore BEHAVIOURAL,
    # computed here in the nav, in two layers:
    #   1. PEER avoidance  -- read the other tug's live (x,y) and yield.
    #   2. OBSTACLE guard  -- never step a footprint into a wall, the pick-cell
    #      fence, or a parked cart (the park row grows over time; poses live).
    #
    # SAFETY RADIUS basis (measured, not guessed): the AMR tug footprint is
    # 0.716 m (W) x 1.259 m (L) (measured from its URDF), so its
    # circumscribing-circle radius (half-diagonal) is
    #   hypot(0.716/2, 1.259/2) = 0.724 m.
    # Two such circles just touch at 2*0.724 = 1.449 m centre-to-centre; add a
    # 0.15 m margin -> SAFETY_RADIUS = 1.6 m is the hard keep-apart distance
    # below which the two bodies could overlap. The reactive guard holds BOTH
    # tugs before their predicted centre crosses it, so it is also the floor
    # the min-inter-tug-distance gate asserts.
    FOOT_L = 1.259
    FOOT_W = 0.716
    FOOT_HDIAG = 0.724            # circumscribing-circle radius
    SAFETY_RADIUS = 1.6          # 2*FOOT_HDIAG + 0.15 margin (hard floor)
    YIELD_RADIUS = 2.4           # lower-priority tug yields early within this
    CONVERGE_CONE = 1.30         # rad (~75 deg): "peer roughly ahead of me"
    CART_HDIAG = 0.495           # 0.70 m cart circumscribing radius
    CART_CLEAR = 1.34            # FOOT_HDIAG + CART_HDIAG + 0.12 margin
    STATIC_INFLATE = 0.48        # FOOT_W/2 + 0.12: keep the body off walls/fence
    LOOKAHEAD_M = 0.55           # how far ahead the reactive guard predicts
    MAX_HOLD_S = 15.0            # hold longer than this -> abort leg + re-plan
    PEER_POLL_S = 0.25           # peer /state cache TTL (throttles HTTP)
    CART_POLL_S = 0.40           # cart-pose cache TTL (throttles supervisor)
    # A STATIONARY blocker (a parked cart, or the peer sitting idle in the
    # path) is routed AROUND with one perpendicular detour waypoint rather than
    # nosed-up-to and waited on -- yielding is for a MOVING peer that will
    # clear. PEER_STATIONARY_V is the speed below which the peer counts as
    # parked; REROUTE_* are the keep-clear radii the detour must satisfy.
    PEER_STATIONARY_V = 0.08     # m/s
    REROUTE_PEER_CLEAR = 1.90    # bow keeps centres >=~1.8 m apart (> safety)
    REROUTE_CART_CLEAR = 1.45    # CART_CLEAR + 0.11
    # Static-footprint margin. The guard tests the tug's ORIENTED footprint
    # (see _foot_span), not a circle, so this is a true edge clearance and can
    # be small: the tightest legitimate pass in the world -- the station dock,
    # nose-south between the fill conveyor's two guide kerbs -- has 0.167 m of
    # real clearance, and a circumscribing-circle guard (0.724 m) would refuse
    # it outright even though the tug fits with room to spare.
    STATIC_MARGIN = 0.08
    # PICK-CELL + WEST-CONVEYOR WORK ZONE. The pickup, conveyor station and the
    # fill/cart-lane conveyors all cluster on the west side, so BOTH tugs' cart
    # work is co-located here on purpose -- tug_a tows the loaded cart UP from
    # the pickup (x approx -8) while tug_b docks/shuttles at the lane, stage and
    # station (x -8..-11.6).
    #
    # THIS BOX NOW MEANS ONE THING ONLY: the carts standing in it are the tugs'
    # own docking targets, not obstacles, so the parked-cart guard stands down
    # here (the row of carts that guard exists for is out east, x 5.9..12.9).
    # It used to ALSO stand peer avoidance down, which is what let the two tugs
    # meet nose-to-nose at 0.13 m (measured). Peer exclusion here is now a TIME
    # mutex over the narrower COL_* rect below -- see "PICK-CELL COLUMN".
    WORK_ZONE_X0, WORK_ZONE_X1 = -12.6, -6.8
    WORK_ZONE_Y0, WORK_ZONE_Y1 = -2.8, 2.1

    # ══ PICK-CELL COLUMN — A TIME MUTEX, NOT A SPACE ONE ══════════════
    #
    # The column is the one place in the world where both tugs' routes
    # genuinely overlap: tug_a comes up the x=-8 conveyor column from the
    # CART_PICKUP, tug_b works the stage -> station tow and the lane fetch
    # across the same 2 m of floor. Spatial repulsion CANNOT solve that -- a
    # corridor narrower than two safety radii has no "around", so a symmetric
    # hold is a deadlock and an asymmetric one is a collision. It was tried,
    # and it deadlocked the line down to a single cycle.
    #
    # So the column is a MUTEX: exactly one tug may be inside it at a time.
    # The claim rides the existing peer /state channel (the same transport as
    # the on_lane transit-lane lock) and is published under idle_loop.col:
    #     want / want_t   this tug is queueing, and since when
    #     hold            this tug owns the column
    #     lease_until     wall clock; renewed while the holder is WORKING
    # Priority is deterministic: dispatch outranks trolley_return, tie-broken
    # by robot id -- with an anti-starvation override so a tug that has waited
    # COL_STARVE_S longer than the other wins regardless. Both tugs evaluate
    # the identical predicate over the identical published state, so exactly
    # one of a simultaneous pair ever grants itself (and a settle re-check
    # closes the poll-interval race).
    #
    # The LEASE is what makes a crash or a chat pause survivable: the holder
    # renews only from loops that run while it is actually working, so a tug
    # frozen by an operator command stops renewing, the lease expires, and the
    # peer takes the column rather than queueing behind a ghost. The waiter
    # stands at a defined point OUTSIDE the column (COL_WAIT_DISPATCH / the
    # west hold) rather than nosing up to the boundary.
    COL_X0, COL_X1 = -10.6, -6.9
    COL_Y0, COL_Y1 = -3.9, 2.2
    COL_LEASE_S = 45.0        # a holder that stops renewing frees it in 45 s
    COL_SETTLE_S = 0.8        # > 2 peer-poll TTLs: closes the both-claim race
    COL_WAIT_S = 150.0        # bounded queue; then abandon the leg + re-plan
    COL_MAX_HOLD_S = 150.0    # bounded ownership; force-release past this
    COL_STARVE_S = 40.0       # waited this much longer than the peer -> I win
    COL_RELEASE_DWELL_S = 6.0  # out of the column this long -> auto-release
    COL_WAIT_DISPATCH = (-6.2, -4.6)   # tug_a queues here (tug_b uses WEST_HOLD)

    # ══ STATIC OBSTACLES — DERIVED, NOT TYPED OUT ═════════════════════
    # The list this replaced was eight hand-written boxes (five walls + three
    # fence segments). Every conveyor, the outfeed spur, the parts feeder and
    # the arm pedestal were simply ABSENT, so the guard was honest about the
    # walls and blind to every machine on the site -- and a "zero incursions"
    # gate run against that list measured its own blind spot (measured at
    # 055d7c82 over 780 ticks: tug_b's footprint inside the FILL_CONVEYOR
    # kerbs/drive head on 111 of them, tug_a inside the OUTBOUND_CONVEYOR
    # kerbs on 36).
    #
    # The set is now read out of the LIVE scene graph at startup, and the rule
    # that separates driveable from solid is HEIGHT, not a name:
    #   top <= DRIVE_OVER_Z (0.06 m)  -> FLOOR. Every painted decal (2 mm) and
    #       every in-floor drag-chain deck plate, chain slot, steel strand and
    #       hatch stripe (<= 18 mm) lands here -- which is exactly the set the
    #       world says a tug crosses without a ramp.
    #   bottom >= TUG_TOP_Z (0.90 m)  -> OVERHEAD (dock-door lintels).
    #   anything else                 -> SOLID. Conveyor guide kerbs (0.10 m),
    #       drive heads (0.34 m), belt frames and legs, the feeder table, the
    #       outfeed spur, the belt strip, walls, fences, door leaves.
    # Height is the right discriminator because it is what a real AMR's bumper
    # scanner measures, and because it cannot rot: add a conveyor to the world
    # and its kerbs are guarded the next time the demo starts.
    DRIVE_OVER_Z = 0.06
    TUG_TOP_Z = 0.90
    # Bodies that are not scene structure and must never enter the static set:
    # the tugs, the carts (tracked live -- they move), the manipulanda on the
    # line, and the three conveyor pusher DOGS (visual bodies riding in the
    # floor slots; one of them parks ON the station a tug has to dock at).
    OBSTACLE_SKIP_RE = _re.compile(
        r"^(MAV_|TROLLEY_|BOX_|GRASP_|PART_|SUN)|_DOG$")
    OBSTACLE_SKIP_TYPES = ("Viewpoint", "WorldInfo", "Floor", "OmniSimSky",
                           "OmniSimSun", "OmniSimSunMarker", "URDFRobot",
                           "Background", "DirectionalLight", "PointLight",
                           "SpotLight")
    # Structures that MUST be present. If the world stops producing one of
    # these the derivation has silently regressed, so say so loudly rather
    # than run with a short list again.
    OBSTACLE_EXPECTED = ("CONVEYOR_LINE", "BELT_STRIP", "OUTFEED_SPUR",
                         "FEEDER", "FILL_CONVEYOR", "CART_LANE_CONVEYOR",
                         "OUTBOUND_CONVEYOR")
    # Static-base machines contribute one keep-out box each, named "<DEF>_BASE";
    # which ones exist is world-specific (see the STATIC_BASE_OBSTACLES registry
    # in _mobile_configs), so they are not asserted here.

    def __init__(self, bridge: MobileBridge,
                 resume_s: float = 60.0, arm_port: int = 0,
                 period: float = 10.0, mode: str = "dispatch",
                 peer_port: int = 0) -> None:
        super().__init__(name="omnilink-idle-" + mode, daemon=True)
        self.bridge = bridge
        self.mode = mode
        self.resume_s = max(5.0, float(resume_s))
        self.arm_port = int(arm_port or 0)
        self.peer_port = int(peer_port or 0)
        self.period = max(0.0, float(period))
        self.trolley_def: Optional[str] = None
        self.staging_xy = (0.0, 0.0)
        self.staging_heading = 0.0
        # Site geometry, all resolved from world DEFs in run().
        self.fill_xy = (0.0, 0.0)     # the fill spot, under the arm
        self.fill_yaw = 0.0
        self.station_xy = (0.0, 0.0)  # mid-span infeed of the fill conveyor
        self.pickup_xy = (0.0, 0.0)   # south end: where loaded carts wait
        self.has_pickup = False
        self.stage_xy = (0.0, 0.0)
        self.lane_pickup_xy = (0.0, 0.0)
        self.lane_entry_xy = (0.0, 0.0)
        self.out_infeed_xy = (0.0, 0.0)
        self.out_exit_xy = (0.0, 0.0)
        self.park_spots: List[tuple] = []     # [(def, (x, y))], scan order
        # park_order is the row's CENSUS: which carts are standing in it, in
        # the order they went in. Rebuilt from the world on startup
        # (pre-parked carts count), appended to on every park, pruned on
        # every collection -- and published as /state park_row_occupants.
        #
        # It is deliberately NOT the collection order any more. It used to be
        # ("collect the oldest, not the westmost"), and that cost 4.90 m of
        # towing per cycle for a distinction no one can see: every cart in
        # the ring is an interchangeable empty. See _collect_from_row.
        self.park_order: List[str] = []
        self.on_lane = False        # published in /state for peer exclusion
        self.leg = "idle"           # coarse phase, for /state + narration
        # Straight-tow trail = rear_offset (0.63) + drawbar (0.70). run()
        # recomputes this from the live constants; the default only covers the
        # window before run() arms, and is kept in sync with _HITCH_LOCAL.
        self._trail = 1.33
        self._paused_logged = False
        self.cycles = 0
        # How many times the CURRENT cycle number has been announced. A cycle
        # that is interrupted (see _cycle_dispatch) re-announces the same n on
        # the next pass, and three identical "dispatch #1" lines read as a tug
        # that is getting nowhere. MEASURED, show2_idle.txt: "dispatch #1" at
        # t=77.2, t=106.6 and t=247.0 -- with a completed park in between.
        self._announced_n = 0
        self._announce_attempt = 0
        self.parks_total = 0     # carts PARKED in the row; see _park_in_spot
        # Carts this tug delivered onto the FILL SPOT for the arm -- the return
        # tug's equivalent of a park, and the event it logs as "return #N:
        # EMPTY CART DELIVERED". It exists because parks_total is structurally
        # zero on this role (_park_in_spot is dispatch-only), so the return tug
        # could not answer "how many carts have you delivered?" with anything
        # but 0 however much work it had done. MEASURED, show2_idle.txt: two
        # EMPTY CART DELIVERED events (t=103.7, t=214.5) plus two loaded carts
        # shuttled out, against a published delivered_total of 0.
        self.returns_total = 0
        # Monotonic count of EVERY completed job, parks AND collections. A
        # full park row turns this tug into a pure collector, and counting
        # only parks then left "after you place this cart" waiting for a
        # delivery that was not coming (measured: 210 s, zero progress).
        self.jobs_total = 0
        # ── obstacle-awareness state (see the AVOIDANCE section) ───────
        # Published in /state so the peer and any observer can see a tug
        # that is yielding, and so the gate harness can prove the holds.
        self.holding = False          # True while decelerated-and-held
        self.hold_reason = ""         # human-readable cause of the hold
        self._avoid_ready = False     # geometry armed (set in run())
        self._leg_break = False       # per-leg deadlock-break latch
        self._static_boxes: List[tuple] = []
        self._peer_cache = (0.0, None)    # (wall_t, peer /state dict)
        self._peer_last_good = (0.0, None)  # last non-empty peer read (fail-safe)
        self._cart_cache = (0.0, {})      # (wall_t, {def: (x, y)})
        self.holds_total = 0          # count of distinct holds this session
        self.hold_secs_max = 0.0      # longest single hold (wall s)
        # ── pick-cell column mutex (see the COL_* block) ──────────────
        self.col_hold = False         # I own the column
        self.col_want = False         # I am queueing for it
        self.col_want_t = 0.0         # wall clock when I started queueing
        self.col_lease_until = 0.0    # wall clock; renewed while working
        self.col_wait_xy = (0.0, 0.0)  # where I stand while queueing
        self.col_waits = 0            # times I had to queue
        self.col_wait_max_s = 0.0     # longest single queue (wall s)
        self.col_wait_last_s = 0.0
        self.col_expiries = 0         # times I took it on an EXPIRED peer lease
        self.col_timeouts = 0         # times the queue timed out -> re-plan
        self._col_since = 0.0
        self._col_outside_since = 0.0
        self._col_acquiring = False
        # Intent events belong next to the work they gate, so route the
        # store's log through this loop's sink (OMNILINK_IDLE_LOG).
        self._intent_obs_t = 0.0
        if bridge.intents is not None:
            bridge.intents._log_fn = self._log

    # ── helpers ───────────────────────────────────────────────────

    def _log(self, msg: str) -> None:
        try:
            t = self.bridge.robot.getTime()
        except Exception:
            t = -1.0
        # SIM time AND wall time on every event, so the realtime factor of an
        # individual leg can be derived offline from one run (see the arm
        # bridge's matching note) instead of by a poll loop that perturbs it.
        line = f"[idle-{self.mode}] t={t:.1f}s w={time.time() - _T0:.1f}s {msg}"
        print(line, flush=True)
        # Optional file sink (OMNILINK_IDLE_LOG) — the windowed sim binary
        # has no capturable stdout, so headless gates read events here.
        path = _os.environ.get("OMNILINK_IDLE_LOG")
        if path:
            try:
                with open(path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except OSError:
                pass

    def _blocked(self) -> bool:
        # THREE ways to be paused, and they are NOT the same thing:
        #  * the quiet window after any operator command (auto-resumes),
        #    armed when the operator's PROMPT lands;
        #  * a STOP hold, armed by act_stop when the stop actually EXECUTES
        #    and lasting MobileBridge.STOP_HOLD_S. Needed because the quiet
        #    window above is armed at prompt time and this world runs it at
        #    12 s: an LLM turn longer than that outlived its own pause, so a
        #    "stop" landed on a robot with nothing holding it. MEASURED on
        #    tug_a -- 1.5 s after the stop the tug had moved 0.384 m and
        #    turned 34.7 deg with paused=False and leg back on to_park;
        #  * a HOLD placed by hold_until_told / a fired "until_told" intent,
        #    which does NOT auto-resume. Without the hold arm, "stop until I
        #    tell you to continue" was silently overridden by the quiet
        #    window sixty seconds later -- measured, and the reason this
        #    layer exists.
        it = self.bridge.intents
        if it is not None and it.hold_active():
            return True
        if time.time() < getattr(self.bridge, "stop_hold_until", 0.0):
            return True
        return (time.time() - self.bridge.last_external_cmd) < self.resume_s

    def _pause_gate(self) -> bool:
        self._intents_observe()
        if self._blocked():
            if not self._paused_logged:
                it = self.bridge.intents
                held = (it is not None and it.hold_active())
                stop_left = (getattr(self.bridge, "stop_hold_until", 0.0)
                             - time.time())
                self._log(
                    ("HELD by the operator (no auto-resume; only "
                     "resume_autonomy clears it)") if held else
                    (f"STOPPED by the operator; resumes in "
                     f"{stop_left:.0f}s unless resume_autonomy comes first")
                    if stop_left > 0 else
                    ("paused (external command: "
                     f"{self.bridge.last_external_src or '?'}); resumes "
                     f"after {self.resume_s:.0f}s quiet"))
                self._paused_logged = True
            return True
        if self._paused_logged:
            self._paused_logged = False
            self._log(f"resuming idle {self.mode} loop")
        return False

    # ── deferred intents ──────────────────────────────────────────

    def _intents_observe(self) -> None:
        """Feed the intent store the two things it can only learn by
        watching: the current leg, and the passage of time (expiry).
        Cheap and idempotent -- called from the cycle top and the two wait
        loops, throttled so a 0.08 s poll does not hammer the lock."""
        it = self.bridge.intents
        if it is None:
            return
        now = time.time()
        if now - getattr(self, "_intent_obs_t", 0.0) < 0.5:
            return
        self._intent_obs_t = now
        try:
            it.note_leg(self.leg)
            # AUTHORITATIVE COUNTER PUSH, every poll. This is what makes a
            # deferred "after you place this cart" impossible to miss: the
            # trigger is evaluated against the same monotonic number /state
            # publishes, not against whether one particular line at the tail
            # of the cycle was reached. See IntentStore.sync_tasks for the
            # measured failure this replaced.
            # ROLE-AWARE delivery count, not parks_total. On the return tug
            # parks_total can never leave 0, so "stop after two more
            # deliveries" said to tug_b could never fire -- it sat pending
            # until it expired. The dispatch tug is unaffected: its
            # returns_total is structurally 0, so this is parks_total exactly.
            it.sync_tasks(self._task_count(), "", self._safe_to_stop(),
                          deliveries=self._carts_delivered())
            it.tick()
        except Exception as e:
            self._log(f"intent observe failed: {e!r}")

    def _task_count(self) -> int:
        """This tug's MONOTONIC count of ANY completed job.

        Both modes count jobs_total. Dispatch increments it from the two
        un-missable primitives (a completed park, a completed collection);
        the return tug increments it in _task_boundary, whose call sites ARE
        its completed legs. The delivery-only tally (parks_total, published
        as delivered_total) is pushed separately so `after_n_deliveries`
        still means deliveries -- but `after_current_task` must not stall
        just because the park row is full and every job this hour happens to
        be a collection, nor because the return tug spent the last five
        minutes running conveyors rather than closing a ring."""
        return self.jobs_total

    # ── "how many carts have you delivered?" ──────────────────────
    #
    # THE OPERATOR'S QUESTION, and until now only one of the two tugs could
    # answer it. `parks_total` is bumped in _park_in_spot ONLY, which is
    # dispatch-only code, so the return tug's tally was structurally 0 -- and
    # it was published as `delivered_total` and described to the model as
    # "carts THIS robot has delivered". Asked how many carts it had delivered
    # after visibly delivering two, tug_b answered 0, correctly reading a
    # field that could not have said anything else. A number under a name
    # that does not match what it counts is the same defect class as an
    # accepted command reported as a completed one.
    #
    # So count what EACH ROLE actually delivers, and keep the two components
    # separately published so the total is auditable rather than asserted:
    #   * dispatch      -- a loaded cart placed in a park spot (parks_total)
    #   * trolley_return -- an empty cart placed on the fill spot for the arm
    #                       (returns_total)
    # A tug only ever runs one of the two loops, so exactly one term is ever
    # non-zero and the sum is that role's honest cart count.
    #
    # COLLECTIONS ARE DELIBERATELY EXCLUDED. Towing a parked cart out through
    # dock E is a completed JOB, not a cart delivered to a station, and it is
    # already counted in jobs_total -- so `jobs_total - carts_delivered_total`
    # stays the collection count it has always been.
    def _carts_delivered(self) -> int:
        return self.parks_total + self.returns_total

    def _delivery_meaning(self) -> str:
        """One sentence naming exactly what _carts_delivered() counted, so the
        model reads the definition off the same payload as the number instead
        of inferring it from a field name."""
        if self.mode == "dispatch":
            return ("loaded carts THIS robot towed from the cart pickup and "
                    "parked in the dispatch row; collections out through "
                    "dock E are NOT included here (they are in jobs_total)")
        if self.mode == "trolley_return":
            return ("empty carts THIS robot delivered onto the fill spot for "
                    "the arm; the staging and lane moves that feed them there "
                    "are NOT included here (they are in jobs_total), and this "
                    "robot never parks carts in the dispatch row")
        return "carts this robot delivered to a station this session"

    # Legs whose whole purpose is to go GET something. `carrying` is still
    # None on the approach, so these read as "empty-handed" for a few
    # seconds right before the hitch closes -- and a pause that landed in
    # that window stopped the tug holding a cart it had just picked up, one
    # poll after firing (measured: fired at leg=column_wait, /state 2 s later
    # showed carrying set). Excluding them closes the race: the tug always
    # passes through back_aisle/returning at the end of a cycle, so this
    # defers the pause, it does not starve it.
    ACQUIRING_LEGS = ("docking", "to_collect", "column_wait", "lane_fetch",
                      "stage_to_station")

    def _safe_to_stop(self) -> bool:
        """Could this tug stop dead right now without stranding anything?

        NOT a "has it finished" test -- that is the counter's job. This only
        answers "may a pause take effect at this instant", so a trigger met
        mid-tow is DEFERRED to the next safe moment rather than discarded.
        Towing strands a cart; the lane lock and the column lease both block
        the peer if their holder freezes; and an approach leg is about to
        become a tow."""
        return (not self.bridge.carrying
                and not self.on_lane
                and not self.col_hold
                and self.leg not in self.ACQUIRING_LEGS)

    def _task_boundary(self, detail: str) -> None:
        """An explicit, high-quality boundary: the cycle finished cleanly.

        Now only a NUDGE -- it labels the event in the log and pushes the
        counter one poll earlier than the observer would. Correctness no
        longer depends on reaching it (it once did, and that was the bug)."""
        it = self.bridge.intents
        if it is None:
            return
        if self.mode != "dispatch":
            # The return tug has no single un-missable primitive the way the
            # dispatch tug has "a cart went into a spot" -- its jobs ARE the
            # legs these calls sit at, so this is where its counter moves.
            self.jobs_total += 1
        try:
            it.sync_tasks(self._task_count(), detail, self._safe_to_stop(),
                          deliveries=self._carts_delivered())
        except Exception as e:
            self._log(f"intent boundary failed: {e!r}")

    def _forbidden(self, rule: str, detail: str) -> bool:
        """True when a standing constraint forbids the action about to be
        taken. Checked BEFORE acting -- a rule that only stops the robot
        after the fact is the bug, not the fix."""
        it = self.bridge.intents
        if it is None:
            return False
        if it.constraint(rule) is None:
            return False
        it.note_block(rule, detail)
        return True

    def _job_forbidden(self, xy, label: str) -> bool:
        """Would starting this JOB break a standing rule? Declining the whole
        job (rather than aborting a leg halfway through) is what lets a
        constrained tug carry on with its OTHER work instead of retrying a
        forbidden dock every pass and looking wedged."""
        if self._forbidden("no_new_pickups", f"declining to {label}"):
            return True
        if xy is not None and self._col_hit(xy[0], xy[1]) and self._forbidden(
                "no_pick_cell",
                f"declining to {label} — it is inside the pick-cell column"):
            return True
        if xy is not None and self._in_park_row(xy[0], xy[1]) and self._forbidden(
                "no_park_row",
                f"declining to {label} — it is in the cart park row"):
            return True
        return False

    def _sleep_gated(self, secs: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < secs:
            if self._blocked():
                return False
            time.sleep(0.25)
        return True

    def _wait_motion_idle(self, timeout: float) -> bool:
        """Wait for the current turn/drive to finish. ``timeout`` is a
        SIM-time budget (motion advances in sim seconds; a wall budget
        silently truncates legs whenever the sim runs below realtime), with
        a wall cap as a dead-sim escape hatch."""
        b = self.bridge
        t0_sim = b.robot.getTime()
        t0_wall = time.time()
        wall_cap = timeout * 3.0 + 30.0
        while time.time() - t0_wall < wall_cap:
            self._intents_observe()
            if self._blocked():
                return False
            self._col_tick()       # the column lease is renewed while WORKING
            with b.lock:
                kind = b.motion[0]
            if kind == "idle":
                return True
            if b.robot.getTime() - t0_sim > timeout:
                break
            time.sleep(0.08)
        self._log(f"leg timed out after {timeout:.0f} sim-s (continuing)")
        return False

    # ALL supervisor pipe access below goes through self._call — a
    # threaded supervisor call stalls the step exchange (see
    # MainThreadCalls). Waits and pure math stay on this thread.

    def _call(self, fn):
        return self.bridge.mt.call(fn)

    def _pose(self):
        return self._call(self.bridge._read_pose)

    @staticmethod
    def _applied(res):
        """Pass through a motion result only if it was actually APPLIED.

        A busy refusal is not a started motion. Treating one as started made
        the loop wait out somebody else's leg and then call its own leg
        done -- which is the same class of lie the busy check exists to
        stop, just told to the autonomy loop instead of to a model."""
        return res if isinstance(res, dict) and res.get("accepted") else None

    def _face(self, target_yaw: float, tol: float = 0.06) -> bool:
        b = self.bridge

        def impl():
            # Runs on the sim thread; the operator's own command always
            # notes last_external_cmd before it lands, so this check makes
            # a mid-cycle command win without a stomping race.
            if self._blocked():
                return None
            _, _, yaw = b._read_pose()
            err = wrap_pi(target_yaw - yaw)
            if abs(err) >= tol:
                if self._applied(b.act_turn(
                        err, source=b.SOURCE_IDLE_LOOP)) is None:
                    return None      # refused: report no turn we never began
            return err

        err = self._call(impl)
        if err is None:
            return False
        if abs(err) < tol:
            return True
        # Budget from the ACHIEVED pivot rate, not the commanded one. A
        # skid-steer turn delivers ~TURN_GAIN_TYPICAL of what is commanded, so
        # a budget sized on spin_speed alone abandons the leg about two thirds
        # of the way through -- which used to be masked by the turn stopping
        # short anyway.
        return self._wait_motion_idle(
            abs(err) / max(0.2, b.spin_speed * b.TURN_GAIN_TYPICAL) * 3.0
            + 4.0)

    def _drive(self, dist: float, speed: Optional[float] = None) -> bool:
        b = self.bridge
        if self._call(lambda: (None if self._blocked()
                               else self._applied(b.act_drive_forward(
                                   dist, speed,
                                   source=b.SOURCE_IDLE_LOOP)))) is None:
            return False
        v = abs(speed) if speed else b.cruise_linear
        return self._wait_motion_idle(abs(dist) / max(0.1, v) * 2.5 + 5.0)

    def _goto(self, x: float, y: float, speed: Optional[float] = None,
              tol: Optional[float] = None, near_cart: Optional[str] = None,
              _depth: int = 0, _col_bypass: bool = False) -> bool:
        # Default resolved here rather than in the signature so it stays tied
        # to ARRIVE_TOL -- the band the guarded drive actually stops in. A
        # literal default drifting away from that band is what let a 0.15 m
        # "approach" pass the skip test and then move only 3 cm (see _dock).
        if tol is None:
            tol = self.ARRIVE_TOL
        pose = self._pose()
        if pose is None:
            return False
        x0, y0, _yaw = pose
        d = math.hypot(x - x0, y - y0)
        if d < tol:
            return True
        # CONSTRAINT GATE — checked BEFORE the claim and before a wheel
        # turns. "Don't enter the pick cell until I say so" has to make this
        # leg IMPOSSIBLE, not merely stop the tug once: the same choke point
        # the column mutex uses is the only place that can promise that,
        # because every leg that could enter the column passes through here.
        # A ZONE RULE FORBIDS ENTERING, NEVER LEAVING. The first cut
        # refused any leg that touched the column, so setting "stay out of
        # the pick cell" while the tug was ALREADY in it (mid-tow, at
        # -8.12,+0.70) refused the escape leg too and trapped it inside the
        # very zone it had been told to leave -- 150 s, 1103 refusals, zero
        # work. Measured 2026-07-22. Only a leg that STARTS outside can be
        # an entry, so that is the only leg gated here; the cycle-level
        # _job_forbidden check is what stops it choosing in-zone work again.
        if not _col_bypass and not self._col_hit(x0, y0):
            if ((self._col_seg_enters(x0, y0, x, y) or self._col_hit(x, y))
                    and self._forbidden(
                        "no_pick_cell",
                        f"leg to ({x:+.2f},{y:+.2f}) would enter the "
                        "pick-cell column")):
                return False
        if (not self._in_park_row(x0, y0) and self._in_park_row(x, y)
                and self._forbidden("no_park_row",
                                    f"leg to ({x:+.2f},{y:+.2f}) would enter "
                                    "the cart park row")):
            return False
        # COLUMN MUTEX. Any leg that would put this tug in the pick-cell
        # column claims it first and queues OUTSIDE if the peer has it. This
        # is the single choke point -- putting it here rather than in each
        # cycle means no leg can ever forget to claim, and the claim is held
        # across the whole dock/tow/exit sequence because _col_tick only
        # drops it once the tug and its cart are demonstrably clear.
        if (self._avoid_ready and not _col_bypass and _depth == 0
                and not self._col_acquiring and self.peer_port
                and not self.col_hold
                and self._col_seg_enters(x0, y0, x, y)):
            self._col_acquiring = True
            try:
                if not self._col_acquire():
                    return False
            finally:
                self._col_acquiring = False
            pose = self._pose()          # queueing may have moved us
            if pose is None:
                return False
            x0, y0, _yaw = pose
            d = math.hypot(x - x0, y - y0)
            if d < tol:
                return True
        # LAYER 2 rerouting: if the straight path clips a STATIONARY blocker
        # (a parked cart, or the peer sitting idle in the way), bow around it
        # via one intermediate waypoint rather than nosing up and stalling.
        # A moving peer is handled by the yield/guard in _drive_guarded.
        if self._avoid_ready and _depth < 1 and d > 0.8:
            wp = self._reroute(x0, y0, x, y, near_cart)
            if wp is not None:
                # Throttle the log: a stationary blocker re-triggers the
                # reroute every cycle, which would flood OMNILINK_IDLE_LOG.
                lt, lwx, lwy = getattr(self, "_reroute_log", (0.0, 9e9, 9e9))
                if (time.time() - lt > 6.0
                        or math.hypot(wp[0] - lwx, wp[1] - lwy) > 0.6):
                    self._log(f"routing around {wp[2]} via "
                              f"({wp[0]:+.2f},{wp[1]:+.2f})")
                    self._reroute_log = (time.time(), wp[0], wp[1])
                if not self._goto(wp[0], wp[1], speed, tol, near_cart,
                                  _depth + 1):
                    return False
                pose = self._pose()
                if pose is None:
                    return False
                x0, y0, _yaw = pose
                d = math.hypot(x - x0, y - y0)
                if d < tol:
                    return True
        # LAYER 3: STRUCTURE-AWARE ROUTING. If the straight line would drag
        # the footprint through a machine, bow around it BEFORE setting off.
        # Doing this here rather than reactively is what turns "hold at the
        # kerb, time out, abandon the leg, try the same line again" into a
        # route that simply goes round.
        if self._avoid_ready and _depth < 1 and d > 0.6:
            hit = self._static_path_hit(x0, y0, x, y)
            if hit is not None:
                wp = self._static_detour(x0, y0, x, y)
                if wp is None:
                    self._log(f"[route] no way past {hit} to "
                              f"({x:+.2f},{y:+.2f}) — driving the direct line "
                              "under the reactive guard")
                else:
                    lt, lwx, lwy = getattr(self, "_detour_log",
                                           (0.0, 9e9, 9e9))
                    if (time.time() - lt > 6.0
                            or math.hypot(wp[0] - lwx, wp[1] - lwy) > 0.6):
                        self._log(f"[route] {hit} blocks the direct line; "
                                  f"going via ({wp[0]:+.2f},{wp[1]:+.2f})")
                        self._detour_log = (time.time(), wp[0], wp[1])
                    if not self._goto(wp[0], wp[1], speed, tol, near_cart,
                                      _depth + 1, True):
                        return False
                    pose = self._pose()
                    if pose is None:
                        return False
                    x0, y0, _yaw = pose
                    d = math.hypot(x - x0, y - y0)
                    if d < tol:
                        return True
        if not self._face(math.atan2(y - y0, x - x0)):
            return False
        # Travel legs are obstacle-guarded (peer + static + parked carts);
        # ``near_cart`` exempts the cart this leg is deliberately approaching
        # to dock, so the guard never blocks a legitimate dock.
        return self._drive_guarded(d, speed, x, y, near_cart)

    def _trolley_pose(self):
        def impl():
            node = self.bridge.robot.getFromDef(self.trolley_def)
            if node is None:
                return None
            p = node.getPosition()
            psi = 0.0
            rf = node.getField("rotation")
            rot = rf.getSFRotation() if rf is not None else None
            if rot and abs(rot[2]) > 0.9:
                psi = rot[3] * (1 if rot[2] > 0 else -1)
            return (p[0], p[1], psi)

        return self._call(impl)

    def _run_conveyor(self, trolley: str, target, yaw: Optional[float],
                      dog_def: str, label: str) -> bool:
        """Start a cart conveyor and wait for it to set the cart down.

        This is the ONLY way a detached cart moves in this loop. There is no
        sibling that quietly repositions one -- if a cart is in the wrong
        place, the answer is a tug, not a write."""
        res = self._call(lambda: self.bridge.start_cart_conveyor(
            trolley, target, yaw, dog_def=dog_def, label=label))
        if res is None or "error" in res:
            self._log(f"{label}: refused to carry {trolley} to "
                      f"({target[0]:+.2f},{target[1]:+.2f}): {res}")
            return False
        self._log(f"{label} RUNNING: carrying {trolley} "
                  f"{res.get('distance')} m to "
                  f"({target[0]:+.2f},{target[1]:+.2f}) "
                  f"[{res.get('duration_s')}s, {res.get('riders')} rider(s)]")
        if not self._wait_carry(trolley, self.CARRY_WAIT_S):
            self._log(f"{label}: carry of {trolley} did not finish in "
                      f"{self.CARRY_WAIT_S:.0f}s (continuing)")
            return False
        return True

    def _wait_carry(self, trolley: Optional[str], timeout: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if not self.bridge.conveyor_busy(trolley):
                return True
            # A conveyor run is WORK, and the tug is usually still standing in
            # the column while it happens -- renew, or the lease would expire
            # under a tug that is legitimately busy.
            self._col_tick()
            time.sleep(0.2)
        return False

    def _all_trolley_xy(self) -> dict:
        """Every cart's xy in ONE supervisor round trip.

        The per-cart version of this used to be fine at two carts; at eight
        it was eight MainThreadCalls hops per poll, on a loop that polls
        continuously, and it showed up as the tugs going sluggish. One call,
        one dict."""
        defs = list(self.bridge.pallet_defs)

        def impl():
            out = {}
            for d in defs:
                node = self.bridge.robot.getFromDef(d)
                if node is None:
                    continue
                p = node.getPosition()
                out[d] = (p[0], p[1])
            return out

        res = self._call(impl) or {}
        # Cache for the /state cart telemetry + the avoidance guard, so
        # neither has to make its own supervisor round trip.
        self._cart_cache = (time.time(), res)
        return res

    # ── obstacle avoidance (peer + static) ────────────────────────
    #
    # Two layers, both feeding one decision: "must I hold before advancing?"
    #   * PEER (robot-vs-robot): the lower-priority mover yields when the peer is
    #     inside YIELD_RADIUS AND roughly ahead on this tug's heading
    #     (converging). Priority is the existing dispatch>return ordering,
    #     tie-broken by name so the pair can never both-yield (deadlock) or
    #     both-proceed (collision). On top of that a HARD reactive check on
    #     BOTH tugs never lets a predicted footprint cross SAFETY_RADIUS of the
    #     peer body or its towed cart.
    #   * STATIC (robot-vs-scene): the same predicted footprint must stay out of
    #     every wall, the pick-cell fence, and every parked cart (live poses).
    # A hold is time-bounded (MAX_HOLD_S); a stuck peer therefore cannot freeze
    # the line -- the tug logs it and takes the deadlock-break priority.

    def _peer_state_cached(self) -> Optional[dict]:
        now = time.time()
        t, st = self._peer_cache
        if t > 0.0 and now - t < self.PEER_POLL_S:
            return st
        st = self._http_state(self.peer_port)
        self._peer_cache = (now, st)
        if st is not None and st.get("x") is not None:
            self._peer_last_good = (now, st)
        elif st is None:
            # FAIL-SAFE: a transient HTTP miss (the peer bridge is busy under
            # load) must NOT read as "no peer" and drop the guard -- that let a
            # tug creep into a parked peer 0.08 m at a time (observed). Reuse
            # the last good pose for a few seconds; the peer cannot have moved
            # far, and staleness only makes the guard more conservative.
            lt, lst = self._peer_last_good
            if lst is not None and now - lt < 3.0:
                return lst
        return st

    # ── what the OTHER robots are doing (exposed, not inferred) ────
    #
    # MEASURED FAILURE THIS FIXES: asked "what is the other tug doing right
    # now?" the model said "I don't have direct access to the other tug's
    # live state" and then GUESSED from an indirect clue. Both halves were
    # wrong. This tug polls the peer's /state several times a second for the
    # lane lock and the column mutex (_peer_state_cached), and it polls the
    # LINE MASTER's /state for the whole line picture (_line_state). The data
    # was there; it was simply never handed to the model.
    #
    # NO NEW POLLING. peer_report reuses the same cache the avoidance code
    # reads, and stamps the answer with the cache's AGE so a stale read is
    # reported as stale instead of as fact.

    def peer_report(self) -> dict:
        now = time.time()
        out: Dict[str, Any] = {
            "me": {
                "id": self.bridge.robot_id,
                "mode": self.mode,
                "leg": self.leg,
                "carrying": self.bridge.carrying,
                "on_lane": bool(self.on_lane),
                "holds_column": bool(self.col_hold),
                "wants_column": bool(self.col_want),
                "paused": self._blocked(),
                "delivered_total": self.parks_total,
                # Role-aware, so the two tugs are comparable in the same
                # report. delivered_total above is parks only and reads as 0
                # forever on the return tug -- see get_state.
                "carts_delivered_total": self._carts_delivered(),
                "carts_delivered_means": self._delivery_meaning(),
                "jobs_total": self.jobs_total,
            },
        }
        if not self.peer_port:
            out["peer"] = {
                "known": False,
                "reason": "this robot has no peer wired up",
                "say": ("There is no other tug configured for me to watch, so "
                        "I genuinely can't tell you what one is doing."),
            }
        else:
            st = self._peer_state_cached()
            age = max(0.0, now - (self._peer_cache[0] or now))
            if not st:
                lt, lst = self._peer_last_good
                out["peer"] = {
                    "known": False,
                    "reason": "the other tug did not answer its status port",
                    "last_seen_s_ago": (round(now - lt, 1) if lst else None),
                    "say": ("I can't reach the other tug's status right now — "
                            "it isn't answering me, so I won't guess at what "
                            "it's doing."),
                }
            else:
                il = st.get("idle_loop") or {}
                col = il.get("col") or {}
                peer = {
                    "known": True,
                    "id": st.get("id"),
                    "leg": il.get("leg"),
                    "mode": il.get("mode"),
                    "role": _ROLE_LABEL.get(il.get("mode") or "", None),
                    "carrying": st.get("carrying"),
                    "paused": il.get("paused"),
                    "held": bool((st.get("autonomy_hold") or {}).get("active")),
                    "delivered": il.get("delivered"),
                    "delivered_total": il.get("delivered_total"),
                    # The peer's ROLE-AWARE cart count, with the definition it
                    # published alongside it. Reporting a return tug's
                    # parks-only delivered_total as "it has delivered N carts"
                    # is the same misreading from the outside that the
                    # operator hit from the inside.
                    "carts_delivered_total": il.get("carts_delivered_total"),
                    "carts_delivered_means": il.get("carts_delivered_means"),
                    "jobs_total": il.get("jobs_total"),
                    "on_lane": il.get("on_lane"),
                    "holding_for_obstacle": il.get("holding"),
                    "hold_reason": il.get("hold_reason"),
                    "xy": ([round(st.get("x") or 0.0, 2),
                            round(st.get("y") or 0.0, 2)]
                           if st.get("x") is not None else None),
                    "column": {"holds": bool(col.get("hold")),
                               "wants": bool(col.get("want")),
                               "lease_until": col.get("lease_until")},
                    # HOW OLD THE ANSWER IS. The lock cache refreshes several
                    # times a second, so this is normally a fraction of a
                    # second -- but if it is not, the model must say so.
                    "data_age_s": round(age, 2),
                    "stale": age > 3.0,
                }
                peer["say"] = self._peer_say(peer)
                out["peer"] = peer
        # THE SHARED RESOURCES, from both sides. "Who is waiting for whom" is
        # a real question about this pair and it now has a real answer.
        out["shared"] = {
            "transit_lane": {
                "i_hold_it": bool(self.on_lane),
                "peer_holds_it": bool(
                    ((out.get("peer") or {}).get("on_lane"))),
            },
            "pick_cell_column": {
                "i_hold_it": bool(self.col_hold),
                "i_want_it": bool(self.col_want),
                "peer_holds_it": bool(
                    ((out.get("peer") or {}).get("column") or {}).get("holds")),
                "my_waits": self.col_waits,
                "my_longest_wait_s": round(self.col_wait_max_s, 1),
            },
        }
        # THE LINE MASTER'S published counts, attributed. This tug already
        # reads them every cycle to decide when a cart is ready.
        line = self._line_state()
        if line:
            out["line"] = dict(line)
            out["line"]["source"] = ("published by the pick cell (line "
                                     "master), not measured by me")
        else:
            out["line"] = {"active": False,
                           "reason": "the pick cell did not answer"}
        if self.bridge.intents is not None:
            out["progress"] = self.bridge.intents.progress()
        return out

    def _peer_say(self, p: dict) -> str:
        who = p.get("id") or "the other tug"
        role = p.get("role")
        leg = p.get("leg") or "idle"
        bits = [f"{who}"]
        if role:
            bits.append(f"({role})")
        if p.get("held"):
            bits.append("is stopped and holding for the operator")
        elif p.get("paused"):
            bits.append("is paused by a recent command")
        else:
            bits.append(f"is on its {leg} leg")
        if p.get("carrying"):
            bits.append(f"towing {p['carrying']}")
        if p.get("holding_for_obstacle"):
            bits.append(f"and currently holding for {p.get('hold_reason') or 'an obstacle'}")
        if p.get("on_lane"):
            bits.append("— it has the transit lane")
        if (p.get("column") or {}).get("holds"):
            bits.append("— it holds the pick-cell column")
        tail = ""
        # PREFER THE ROLE-AWARE COUNT. delivered_total is parks only, so this
        # sentence used to say "it has delivered 0 cart(s) so far" about a
        # return tug that had delivered several. Fall back to the old field
        # only for a peer running an older bridge that does not publish the
        # new one.
        nd = p.get("carts_delivered_total")
        if nd is None:
            nd = p.get("delivered_total")
        if nd is not None:
            tail = f" It has delivered {nd} cart(s) so far."
        if p.get("stale"):
            tail += (f" (That reading is {p['data_age_s']:.0f} s old — the "
                     "newest I have.)")
        return " ".join(bits) + "." + tail

    def _cart_xy_cached(self) -> dict:
        now = time.time()
        t, d = self._cart_cache
        if t > 0.0 and now - t < self.CART_POLL_S:
            return d
        return self._all_trolley_xy()      # refreshes self._cart_cache

    def _has_priority(self, peer: dict) -> bool:
        """True when THIS tug outranks the peer, so it is the peer that must
        yield. dispatch outranks trolley_return; a tie (same mode) breaks by
        robot id so exactly one of the pair ever yields."""
        peer_mode = (peer.get("idle_loop") or {}).get("mode")
        if peer_mode == self.mode:
            return self.bridge.robot_id <= str(peer.get("id", "~"))
        return self.mode == "dispatch"

    # ── deriving the static set from the live scene graph ─────────

    @staticmethod
    def _node_pose(node):
        """(tx, ty, tz, dyaw) of a node relative to its parent."""
        tx = ty = tz = 0.0
        dyaw = 0.0
        f = node.getField("translation")
        if f is not None:
            try:
                v = f.getSFVec3f()
                tx, ty, tz = float(v[0]), float(v[1]), float(v[2])
            except Exception:
                pass
        f = node.getField("rotation")
        if f is not None:
            try:
                r = f.getSFRotation()
                if r and len(r) >= 4 and abs(r[2]) > 0.9:
                    dyaw = float(r[3]) * (1.0 if r[2] > 0 else -1.0)
            except Exception:
                pass
        return tx, ty, tz, dyaw

    @staticmethod
    def _sub_nodes(node, fname):
        """Children of an SF- or MF-node field, or [] if it has none."""
        f = node.getField(fname)
        if f is None:
            return []
        try:
            cnt = f.getCount()
        except Exception:
            cnt = -1
        out = []
        if cnt is None or cnt < 0:
            try:
                n = f.getSFNode()
            except Exception:
                n = None
            return [n] if n is not None else []
        for i in range(cnt):
            try:
                n = f.getMFNode(i)
            except Exception:
                n = None
            if n is not None:
                out.append(n)
        return out

    def _walk_geometry(self, node, ox, oy, oz, oyaw, label, out, depth=0):
        """Accumulate (label, x0, y0, x1, y1, z0, z1) world AABBs for every
        Box/Cylinder under ``node``. Z is kept because HEIGHT is what decides
        driveable-vs-solid (see the DRIVE_OVER_Z block)."""
        if depth > 8:
            return
        try:
            typ = node.getTypeName()
        except Exception:
            return
        tx, ty, tz, dyaw = self._node_pose(node)
        cx = ox + tx * math.cos(oyaw) - ty * math.sin(oyaw)
        cy = oy + tx * math.sin(oyaw) + ty * math.cos(oyaw)
        cz = oz + tz
        yaw = oyaw + dyaw
        if typ == "Box":
            f = node.getField("size")
            if f is None:
                return
            try:
                s = f.getSFVec3f()
            except Exception:
                return
            c, sn = abs(math.cos(yaw)), abs(math.sin(yaw))
            ex = abs(s[0]) / 2.0 * c + abs(s[1]) / 2.0 * sn
            ey = abs(s[0]) / 2.0 * sn + abs(s[1]) / 2.0 * c
            out.append((label, cx - ex, cy - ey, cx + ex, cy + ey,
                        cz - abs(s[2]) / 2.0, cz + abs(s[2]) / 2.0))
            return
        if typ in ("Cylinder", "Capsule"):
            fr, fh = node.getField("radius"), node.getField("height")
            try:
                r = float(fr.getSFFloat()) if fr is not None else 0.0
                h = float(fh.getSFFloat()) if fh is not None else 0.0
            except Exception:
                return
            out.append((label, cx - r, cy - r, cx + r, cy + r,
                        cz - h / 2.0, cz + h / 2.0))
            return
        for fname in ("children", "geometry", "boundingObject"):
            for sub in self._sub_nodes(node, fname):
                self._walk_geometry(sub, cx, cy, cz, yaw, label, out,
                                    depth + 1)

    def _scan_scene_node(self, node) -> List[tuple]:
        """One top-level scene child -> its raw (label, aabb, z-span) tuples."""
        out: List[tuple] = []
        try:
            typ = node.getTypeName()
            dn = node.getDef() or ""
        except Exception:
            return out
        try:
            fn = node.getField("name")
            nm = fn.getSFString() if fn is not None else ""
        except Exception:
            nm = ""
        # A static-base machine (an arm on a pedestal) is a URDFRobot, so its
        # base is not in the scene tree as a Box the walk below can find -- it
        # contributes a declared keep-out box instead. The registry is empty
        # unless a world that has one is present.
        if dn in STATIC_BASE_OBSTACLES:
            hx, hy, top = STATIC_BASE_OBSTACLES[dn]
            tx, ty, _tz, _y = self._node_pose(node)
            out.append((f"{dn}_BASE", tx - hx, ty - hy, tx + hx,
                        ty + hy, 0.0, top))
            return out
        if typ in self.OBSTACLE_SKIP_TYPES:
            return out
        if dn and self.OBSTACLE_SKIP_RE.search(dn):
            return out
        if typ == "Wall":
            tx, ty, tz, _y = self._node_pose(node)
            f = node.getField("size")
            try:
                s = f.getSFVec3f() if f is not None else [1.0, 1.0, 1.0]
            except Exception:
                s = [1.0, 1.0, 1.0]
            out.append((f"WALL:{nm or 'wall'}", tx - s[0] / 2.0,
                        ty - s[1] / 2.0, tx + s[0] / 2.0, ty + s[1] / 2.0,
                        tz, tz + s[2]))
            return out
        if typ == "Fence":
            f = node.getField("path")
            pts = []
            try:
                for i in range(f.getCount()):
                    v = f.getMFVec3f(i)
                    pts.append((float(v[0]), float(v[1])))
            except Exception:
                pts = []
            if len(pts) >= 2:
                fh = node.getField("height")
                try:
                    h = float(fh.getSFFloat()) if fh is not None else 1.1
                except Exception:
                    h = 1.1
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                out.append((f"FENCE:{nm or 'fence'}", min(xs) - 0.05,
                            min(ys) - 0.05, max(xs) + 0.05, max(ys) + 0.05,
                            0.0, h))
            return out
        self._walk_geometry(node, 0.0, 0.0, 0.0, 0.0, dn or typ, out)
        return out

    # Fallback: the pre-derivation hand-written list. Used ONLY if the scene
    # walk comes back empty (an engine that will not expose scene fields), and
    # loudly, because it is the short list that hid the incursions.
    _FALLBACK_BOXES = [
        ("WALL:wall_N",         (-15.1,  8.9,  15.1,  9.1)),
        ("WALL:wall_S_west",    (-15.1, -9.1, -13.0, -8.9)),
        ("WALL:wall_S_east",    (-9.0,  -9.1,  15.1, -8.9)),
        ("WALL:wall_E",         (14.9,  -9.1,  15.1,  9.1)),
        ("WALL:wall_W",         (-15.1, -9.1, -14.9,  9.1)),
        ("FENCE:pick_s_west",   (-9.85,  2.55, -8.75, 2.65)),
        ("FENCE:pick_s_east",   (-7.25,  2.55, -6.15, 2.65)),
        ("FENCE:pick_e",        (-6.25,  2.55, -6.15, 6.25)),
    ]

    def _derive_static_boxes(self) -> List[tuple]:
        """Walk the scene ONCE at startup and build the solid-footprint set.

        Done in small chunks: every field read is a supervisor round trip and
        the whole walk is a few thousand of them, so one monolithic call would
        stall a sim step outright. Eight top-level nodes per hop keeps each
        hop well inside a 32 ms tick."""
        b = self.bridge

        def n_children():
            root = b.robot.getRoot()
            f = root.getField("children") if root is not None else None
            return f.getCount() if f is not None else 0

        total = self._call(n_children) or 0
        raw: List[tuple] = []
        i = 0
        while i < total:
            lo, hi = i, min(i + 8, total)

            def chunk(lo=lo, hi=hi):
                root = b.robot.getRoot()
                f = root.getField("children")
                acc: List[tuple] = []
                for k in range(lo, hi):
                    try:
                        acc.extend(self._scan_scene_node(f.getMFNode(k)))
                    except Exception:
                        continue
                return acc

            raw.extend(self._call(chunk) or [])
            i = hi

        solid: List[tuple] = []
        n_floor = n_over = 0
        per_label: Dict[str, int] = {}
        for (lab, x0, y0, x1, y1, z0, z1) in raw:
            if z1 <= self.DRIVE_OVER_Z:
                n_floor += 1
                continue
            if z0 >= self.TUG_TOP_Z:
                n_over += 1
                continue
            solid.append((f"{lab}#{per_label.get(lab, 0)}",
                          (min(x0, x1), min(y0, y1), max(x0, x1),
                           max(y0, y1))))
            per_label[lab] = per_label.get(lab, 0) + 1

        if len(solid) < 5:
            self._log("[obstacles] scene walk produced only "
                      f"{len(solid)} footprint(s) from {total} top-level "
                      "node(s) — FALLING BACK to the hand-written walls+fence "
                      "list. The conveyors are NOT guarded in this run.")
            return list(self._FALLBACK_BOXES)

        for name in self.OBSTACLE_EXPECTED:
            if name not in per_label:
                self._log(f"[obstacles] WARNING: expected structure {name} "
                          "produced no solid footprint — either the world "
                          "dropped it or it became flush. Tugs will not "
                          "avoid it.")
        n_wall = sum(1 for k in per_label if k.startswith("WALL:"))
        n_fence = sum(1 for k in per_label if k.startswith("FENCE:"))
        if n_wall < 5 or n_fence < 3:
            self._log(f"[obstacles] WARNING: found {n_wall} wall(s) and "
                      f"{n_fence} fence(s); the warehouse shell has 5 and 3.")
        self._log(f"[obstacles] derived {len(solid)} solid footprint(s) from "
                  f"{total} top-level node(s): "
                  + ", ".join(f"{k}x{v}" for k, v in sorted(per_label.items()))
                  + f" | dropped {n_floor} flush/driveable (decals, in-floor "
                  f"conveyor decks/slots/chains/hatching) and {n_over} "
                  f"overhead (top<= {self.DRIVE_OVER_Z} m / bottom>= "
                  f"{self.TUG_TOP_Z} m)")
        if _os.environ.get("OMNILINK_AVOID_DUMP"):
            for nm, (x0, y0, x1, y1) in solid:
                self._log(f"[obstacles]   {nm:<26s} "
                          f"x[{x0:+8.3f},{x1:+8.3f}] y[{y0:+8.3f},{y1:+8.3f}]")
        return solid

    def _foot_span(self, heading: Optional[float]) -> Tuple[float, float]:
        """Half-extents of the tug's axis-aligned bounding box at ``heading``,
        plus STATIC_MARGIN. ``None`` -> the circumscribing disc (any yaw).

        This is why the guard can be both accurate and tight. The tug is
        1.259 x 0.716 m, so its footprint is 0.36 m wide across the beam and
        0.63 m long fore-and-aft; testing it as a 0.72 m circle -- the old
        behaviour in everything but name -- overstates the beam by a factor of
        two and would refuse the station dock, which really does fit between
        the fill conveyor's kerbs with 0.167 m to spare."""
        if heading is None:
            m = self.FOOT_HDIAG + self.STATIC_MARGIN
            return (m, m)
        c, s = abs(math.cos(heading)), abs(math.sin(heading))
        hl, hw = self.FOOT_L / 2.0, self.FOOT_W / 2.0
        return (hl * c + hw * s + self.STATIC_MARGIN,
                hl * s + hw * c + self.STATIC_MARGIN)

    @staticmethod
    def _obb_vs_aabb(px, py, heading, hl, hw, box) -> bool:
        """Separating-axis test: the tug's ORIENTED footprint vs a world-axis
        obstacle box.

        The AABB-of-the-OBB shortcut is not good enough here and it cost a
        whole measurement run to learn why: at a 27 deg heading the bounding
        box of a 1.26 x 0.72 m tug is 1.61 x 1.29 m, so the guard reported the
        tug inside the fill conveyor's kerb while its true footprint was
        0.45 m clear -- and the leg aborted on a phantom. Four axes, sixteen
        dot products, once per obstacle per check; the broad phase above keeps
        it to the two or three boxes that could possibly matter."""
        x0, y0, x1, y1 = box
        c, s = math.cos(heading), math.sin(heading)
        pts = [(px + dx * c - dy * s, py + dx * s + dy * c)
               for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]
        if (max(p[0] for p in pts) <= x0 or min(p[0] for p in pts) >= x1
                or max(p[1] for p in pts) <= y0
                or min(p[1] for p in pts) >= y1):
            return False
        bpts = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
        for axx, axy in ((c, s), (-s, c)):
            pa = [p[0] * axx + p[1] * axy for p in pts]
            pb = [q[0] * axx + q[1] * axy for q in bpts]
            if max(pa) <= min(pb) or min(pa) >= max(pb):
                return False
        return True

    def _static_incursion(self, px: float, py: float,
                          heading: Optional[float] = None) -> Optional[str]:
        """Name of the first solid structure this tug's footprint would enter
        at (px, py) on ``heading``, or None. ``heading`` None means "any yaw"
        (used for waypoint vetting, where the tug will pivot) and falls back
        to the circumscribing disc."""
        hx, hy = self._foot_span(heading)
        hl = self.FOOT_L / 2.0 + self.STATIC_MARGIN
        hw = self.FOOT_W / 2.0 + self.STATIC_MARGIN
        for name, box in self._static_boxes:
            x0, y0, x1, y1 = box
            # broad phase: the oriented footprint cannot reach outside this
            if not (x0 - hx <= px <= x1 + hx and y0 - hy <= py <= y1 + hy):
                continue
            if heading is None:
                return name          # any-yaw check: the disc test is the test
            if self._obb_vs_aabb(px, py, heading, hl, hw, box):
                return name
        return None

    def _static_path_hit(self, x0, y0, x1, y1) -> Optional[str]:
        """Would driving straight from (x0,y0) to (x1,y1) put the tug's
        footprint inside a solid structure at any point along the way?

        Tested on the LEG'S OWN HEADING, not on a circumscribing disc. That
        distinction is load-bearing: the fill conveyor's guide kerbs leave a
        1.05 m lane and the tug is 0.72 m across the beam, so it fits nose-on
        with 0.167 m either side -- a disc test (1.61 m across) would declare
        the world's central cart column impassable and there would be nowhere
        left to drive."""
        d = math.hypot(x1 - x0, y1 - y0)
        if d < 1e-6:
            return self._static_incursion(x0, y0)
        hd = math.atan2(y1 - y0, x1 - x0)
        # BROAD PHASE ONCE PER LEG, not once per sample. Without it a 19 m
        # transit leg is ~100 samples x 62 boxes of arithmetic inside a
        # controller process that is sharing a CPU with the engine, and it
        # showed up as realtime factor, not as a bug: 0.65x against 0.99x.
        hx, hy = self._foot_span(hd)
        lx0, lx1 = min(x0, x1) - hx, max(x0, x1) + hx
        ly0, ly1 = min(y0, y1) - hy, max(y0, y1) + hy
        near = [(nm, bx) for nm, bx in self._static_boxes
                if not (bx[2] < lx0 or bx[0] > lx1
                        or bx[3] < ly0 or bx[1] > ly1)]
        if not near:
            return None
        hl = self.FOOT_L / 2.0 + self.STATIC_MARGIN
        hw = self.FOOT_W / 2.0 + self.STATIC_MARGIN
        n = max(1, int(d / 0.20))
        for i in range(n + 1):
            t = i / float(n)
            sx, sy = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            for nm, bx in near:
                if (bx[0] - hx <= sx <= bx[2] + hx
                        and bx[1] - hy <= sy <= bx[3] + hy
                        and self._obb_vs_aabb(sx, sy, hd, hl, hw, bx)):
                    return nm
        return None

    def _static_detour(self, x0, y0, x1, y1):
        """One perpendicular waypoint that gets a blocked straight leg past a
        STRUCTURE, or None if the widest offset tried still does not clear.

        Deliberately the same shape as the parked-cart bow (_reroute): a
        single legible sideways move, tried nearest-first on both sides, with
        every sub-leg re-tested. Static structures used to be handled purely
        reactively -- drive at the kerb, hold for MAX_HOLD_S, abandon the leg,
        re-plan the identical line -- which measured as 15 s of dead air per
        occurrence and, on one baseline run, eight of them."""
        L = math.hypot(x1 - x0, y1 - y0)
        if L < 0.6:
            return None
        px, py = -(y1 - y0) / L, (x1 - x0) / L
        mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        for off in (0.5, 1.0, 1.5, 2.0, 2.6):
            for sgn in (1.0, -1.0):
                wx = clamp(mx + sgn * px * off,
                           -self.bridge.SITE_HALF_X + 0.5,
                           self.bridge.SITE_HALF_X - 0.5)
                wy = clamp(my + sgn * py * off,
                           -self.bridge.SITE_HALF_Y + 0.5,
                           self.bridge.SITE_HALF_Y - 0.5)
                if self._static_incursion(wx, wy) is not None:
                    continue          # cannot stand (and pivot) there
                if (self._static_path_hit(x0, y0, wx, wy) is None
                        and self._static_path_hit(wx, wy, x1, y1) is None):
                    return (wx, wy)
        return None

    def _in_work_zone(self, x: float, y: float) -> bool:
        """True inside the choreographed pick-cell + west-conveyor work zone,
        where every cart standing about is a tug's own docking target rather
        than an obstacle (see the WORK_ZONE_* block). Peer exclusion here is
        the column mutex, NOT this box."""
        return (self.WORK_ZONE_X0 <= x <= self.WORK_ZONE_X1
                and self.WORK_ZONE_Y0 <= y <= self.WORK_ZONE_Y1)

    # ── the pick-cell column mutex ────────────────────────────────

    def _col_hit(self, x: float, y: float) -> bool:
        return (self.COL_X0 <= x <= self.COL_X1
                and self.COL_Y0 <= y <= self.COL_Y1)

    def _in_park_row(self, x: float, y: float) -> bool:
        """Bounding box of the painted PARK_SPOT_* row, grown by a tug
        half-length. Derived from the live spots so a re-painted world moves
        the zone with the decals."""
        if not self.park_spots:
            return False
        xs = [p[1][0] for p in self.park_spots]
        ys = [p[1][1] for p in self.park_spots]
        m = 1.2
        return (min(xs) - m <= x <= max(xs) + m
                and min(ys) - m <= y <= max(ys) + m)

    def _col_seg_enters(self, x0, y0, x1, y1) -> bool:
        """Would the straight leg (x0,y0)->(x1,y1) put this tug in the column?"""
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.25) + 1)
        for i in range(n + 1):
            t = i / float(n)
            if self._col_hit(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t):
                return True
        return False

    def _col_peer(self) -> dict:
        st = self._peer_state_cached() or {}
        return (st.get("idle_loop") or {}).get("col") or {}

    def _col_grant(self, peer: dict, now: float) -> Tuple[bool, str]:
        """May I take the column? Evaluated identically by BOTH tugs over the
        identical published state, so exactly one of a simultaneous pair says
        yes."""
        if peer.get("hold"):
            lease = float(peer.get("lease_until") or 0.0)
            if lease > now:
                return (False, f"{peer.get('id', 'peer')} holds the lease "
                               f"({lease - now:.0f}s left)")
            self.col_expiries += 1
            self._log(f"[column] peer lease EXPIRED {now - lease:.0f}s ago "
                      "(crashed, paused or wedged) — taking the column")
            return (True, "")
        if peer.get("want"):
            pw = float(peer.get("want_t") or now)
            # Anti-starvation first: a tug that has queued COL_STARVE_S longer
            # than the other wins whatever its mode, so a busy dispatch loop
            # cannot lock trolley_return out of its own work forever.
            if self.col_want_t < pw - self.COL_STARVE_S:
                return (True, "")
            if pw < self.col_want_t - self.COL_STARVE_S:
                return (False, "peer has queued longer")
            if self.mode != (peer.get("mode") or ""):
                if self.mode == "dispatch":
                    return (True, "")
                return (False, "dispatch outranks trolley_return")
            # Same mode (never happens in the shipped pair): break by id.
            if self.bridge.robot_id <= str(peer.get("id", "~")):
                return (True, "")
            return (False, "peer wins the id tie-break")
        return (True, "")

    def _col_take(self) -> None:
        self.col_hold = True
        self.col_want = False
        self._col_since = time.time()
        self._col_outside_since = 0.0
        self.col_lease_until = self._col_since + self.COL_LEASE_S

    def _col_renew(self) -> None:
        if not self.col_hold:
            return
        now = time.time()
        if now - self._col_since > self.COL_MAX_HOLD_S:
            self._log(f"[column] held {now - self._col_since:.0f}s (over the "
                      f"{self.COL_MAX_HOLD_S:.0f}s cap) — force-releasing so "
                      "the peer can work")
            self._col_release()
            return
        self.col_lease_until = now + self.COL_LEASE_S

    def _col_release(self) -> None:
        if self.col_hold:
            self._log(f"[column] released after "
                      f"{time.time() - self._col_since:.1f}s")
        self.col_hold = False
        self.col_want = False
        self.col_lease_until = 0.0
        self._col_outside_since = 0.0

    def _col_tick(self) -> None:
        """Lease heartbeat. Renews while the column is genuinely in use and
        drops it once this tug -- and anything it tows -- has been out of the
        column for COL_RELEASE_DWELL_S. Called from the loops that run while
        the tug is WORKING (guarded drive, motion wait, conveyor wait) and
        deliberately NOT from the chat-pause gate, so an operator-paused
        holder stops renewing and the peer inherits the column."""
        if not self.col_hold:
            return
        # THROTTLED. Every tick costs a supervisor pose round trip, and this
        # is called from wait loops that spin at up to 12 Hz; against a 45 s
        # lease twice a second is ample, and the unthrottled version cost
        # measurable realtime factor on a box already running four processes.
        now0 = time.time()
        if now0 - getattr(self, "_col_tick_t", 0.0) < 0.5:
            return
        self._col_tick_t = now0
        # Did I lose it while I was not looking? The peer only ever takes the
        # column on an EXPIRED lease, so a peer advertising a live hold while
        # I also think I hold means my lease lapsed (chat pause, stalled leg)
        # and it inherited. Stand down immediately -- from here the in-column
        # guard treats me as the intruder, which is exactly right.
        peer = self._col_peer()
        if peer.get("hold") and float(peer.get("lease_until") or 0.0) > time.time():
            self._log(f"[column] lease LOST to {peer.get('id', 'peer')} "
                      "(mine lapsed while this tug was paused or stalled) — "
                      "standing down")
            self.col_hold = False
            self.col_want = False
            self.col_lease_until = 0.0
            self._col_outside_since = 0.0
            return
        pose = self._pose()
        if pose is None:
            self._col_renew()
            return
        x, y, yaw = pose
        inside = self._col_hit(x, y)
        if not inside and self.bridge.carrying:
            inside = self._col_hit(x - self._trail * math.cos(yaw),
                                   y - self._trail * math.sin(yaw))
        now = time.time()
        if inside:
            self._col_outside_since = 0.0
        elif self._col_outside_since == 0.0:
            self._col_outside_since = now
        elif now - self._col_outside_since > self.COL_RELEASE_DWELL_S:
            self._col_release()
            return
        self._col_renew()

    def _col_acquire(self) -> bool:
        """Claim the column, queueing OUTSIDE it until the peer frees it.

        Returns False if an operator command arrives (chat wins, instantly)
        or the bounded queue expires -- in both cases the caller abandons the
        leg and the cycle re-plans on its next pass. It never proceeds
        without the claim."""
        if self.col_hold:
            self._col_renew()
            return True
        if not self.peer_port:
            self._col_take()
            return True
        self.col_want = True
        self.col_want_t = time.time()
        t0 = self.col_want_t
        moved = queued = False
        while time.time() - t0 < self.COL_WAIT_S:
            if self._blocked():
                self.col_want = False
                return False
            ok, why = self._col_grant(self._col_peer(), time.time())
            if ok:
                # SETTLE. Both tugs publish `want` before they poll, so a
                # dead-heat needs one more look after more than two peer-poll
                # TTLs: whichever took the lease is now advertising `hold` and
                # the other backs off before either has moved a wheel.
                time.sleep(self.COL_SETTLE_S)
                if self._blocked():
                    self.col_want = False
                    return False
                ok, why = self._col_grant(self._col_peer(), time.time())
                if ok:
                    self._col_take()
                    if queued:
                        w = time.time() - t0
                        self.col_wait_last_s = w
                        self.col_wait_max_s = max(self.col_wait_max_s, w)
                        self._log(f"[column] granted after {w:.1f}s queued")
                    return True
            if not queued:
                queued = True
                self.col_waits += 1
                self._log(f"[column] QUEUEING for the pick-cell column: {why}")
            if not moved:
                moved = True
                pose = self._pose()
                wx, wy = self.col_wait_xy
                if pose is not None and math.hypot(pose[0] - wx,
                                                   pose[1] - wy) > 0.5:
                    prev, self.leg = self.leg, "column_wait"
                    self._goto(wx, wy, speed=0.9, _col_bypass=True)
                    self.leg = prev
            time.sleep(0.3)
        self.col_want = False
        w = time.time() - t0
        self.col_timeouts += 1
        self.col_wait_last_s = w
        self.col_wait_max_s = max(self.col_wait_max_s, w)
        self._log(f"[column] queue TIMED OUT after {w:.0f}s — abandoning this "
                  "leg; the cycle re-plans on the next pass")
        return False

    def _col_state(self) -> dict:
        return {
            "hold": bool(self.col_hold),
            "want": bool(self.col_want),
            "want_t": round(self.col_want_t, 3),
            "lease_until": round(self.col_lease_until, 3),
            "mode": self.mode,
            "id": self.bridge.robot_id,
            "waits": self.col_waits,
            "wait_max_s": round(self.col_wait_max_s, 2),
            "wait_last_s": round(self.col_wait_last_s, 2),
            "expiries": self.col_expiries,
            "timeouts": self.col_timeouts,
        }

    @staticmethod
    def _seg_point_dist(px, py, x0, y0, x1, y1):
        """Distance from point (px,py) to segment (x0,y0)-(x1,y1); returns
        (dist, t in [0,1], closest_x, closest_y)."""
        dx, dy = x1 - x0, y1 - y0
        l2 = dx * dx + dy * dy
        if l2 < 1e-9:
            return math.hypot(px - x0, py - y0), 0.0, x0, y0
        t = clamp(((px - x0) * dx + (py - y0) * dy) / l2, 0.0, 1.0)
        cx, cy = x0 + t * dx, y0 + t * dy
        return math.hypot(px - cx, py - cy), t, cx, cy

    def _stationary_blockers(self, near_cart) -> List[tuple]:
        """(name, x, y, keep_clear) point-obstacles that are STANDING STILL --
        a parked cart, or the peer sitting idle in the path. These are routed
        AROUND (a moving peer is yielded to instead)."""
        obs = []
        peer = self._peer_state_cached()
        # A peer standing INSIDE the column while I hold the lease is not a
        # thing to bow around -- it is an exception (chat-paused or an aborted
        # leg), handled by the bounded hold in _should_hold. Everywhere else a
        # stopped peer is routed around exactly as a parked cart is.
        if (peer is not None and peer.get("x") is not None
                and not (self.col_hold
                         and self._col_hit(float(peer["x"]),
                                           float(peer["y"])))):
            pv = abs(float(peer.get("v_linear") or 0.0))
            p_holding = bool((peer.get("idle_loop") or {}).get("holding"))
            if pv < self.PEER_STATIONARY_V or p_holding:
                pid = peer.get("id", "peer")
                obs.append((f"peer {pid}", float(peer["x"]), float(peer["y"]),
                            self.REROUTE_PEER_CLEAR))
                if peer.get("carrying"):
                    pyaw = float(peer.get("yaw", 0.0))
                    obs.append((f"peer {pid} cart",
                                float(peer["x"]) - self._trail * math.cos(pyaw),
                                float(peer["y"]) - self._trail * math.sin(pyaw),
                                self.REROUTE_CART_CLEAR))
        mine = self.bridge.carrying
        for d, p in self._cart_xy_cached().items():
            if d == mine or d == near_cart or p is None:
                continue
            if self._in_work_zone(p[0], p[1]):
                continue      # work-zone carts are the tugs' own docking targets
            obs.append((f"cart {d}", p[0], p[1], self.REROUTE_CART_CLEAR))
        return obs

    def _reroute(self, x0, y0, x1, y1, near_cart):
        """If the straight path (x0,y0)->(x1,y1) passes within a stationary
        blocker's keep-clear radius, return ONE perpendicular detour waypoint
        (wx, wy, name) that clears it and both sub-legs, bounds-checked; else
        None. Keeps routes legible (a single sideways bow, not a random walk)."""
        blk = None
        for name, ox, oy, clr in self._stationary_blockers(near_cart):
            dist, t, cx, cy = self._seg_point_dist(ox, oy, x0, y0, x1, y1)
            # ignore blockers hugging an endpoint (start = here, end = target)
            if 0.06 < t < 0.94 and dist < clr:
                if blk is None or t < blk[0]:
                    blk = (t, name, ox, oy, clr, dist, cx, cy)
        if blk is None:
            return None
        _t, name, ox, oy, clr, dist, cx, cy = blk
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy) or 1.0
        perp = (-dy / L, dx / L)
        need = clr - dist + 0.30
        side = -1.0 if (perp[0] * (ox - cx) + perp[1] * (oy - cy)) > 0 else 1.0
        for s in (side, -side):
            wx = clamp(cx + s * perp[0] * need,
                       -self.bridge.SITE_HALF_X + 0.5,
                       self.bridge.SITE_HALF_X - 0.5)
            wy = clamp(cy + s * perp[1] * need,
                       -self.bridge.SITE_HALF_Y + 0.5,
                       self.bridge.SITE_HALF_Y - 0.5)
            ok = math.hypot(ox - wx, oy - wy) >= clr - 0.05
            # ...and never bow INTO a machine. The detour used to be
            # bounds-checked against the site walls only, which is fine when
            # the static set is five walls and meaningless now that it knows
            # about every conveyor -- a bow around a parked cart could have
            # put the tug on the outfeed spur. Tested at any yaw (the tug
            # arrives at a waypoint on one heading and leaves on another).
            if ok and self._static_incursion(wx, wy) is not None:
                ok = False
            for (ax, ay, bx, by) in ((x0, y0, wx, wy), (wx, wy, x1, y1)):
                dd, _tt, _cx, _cy = self._seg_point_dist(ox, oy, ax, ay, bx, by)
                if dd < clr - 0.10:
                    ok = False
                    break
                if self._static_path_hit(ax, ay, bx, by) is not None:
                    ok = False
                    break
            if ok:
                return (wx, wy, name)
        return None

    def _should_hold(self, tx: float, ty: float,
                     near_cart: Optional[str]) -> Tuple[bool, str]:
        """Decide whether to hold before continuing toward (tx, ty)."""
        pose = self._pose()
        if pose is None:
            return (False, "")
        x, y, _ = pose
        dgo = math.hypot(tx - x, ty - y)
        if dgo < 1e-3:
            return (False, "")
        hd = math.atan2(ty - y, tx - x)          # intended travel direction
        la = min(self.LOOKAHEAD_M, dgo)
        ax = x + la * math.cos(hd)               # predicted body centre
        ay = y + la * math.sin(hd)

        # 1) PEER -- ONLY THE LOWER-PRIORITY TUG HOLDS FOR THE PEER.
        # The higher-priority tug must NEVER freeze for the peer: a symmetric
        # reactive hold made both tugs stop at the shared west column, each
        # waiting on the other, until the deadlock-break drove them through
        # each other (measured min 0.14 m, throughput collapsed). Instead the
        # higher tug ROUTES AROUND a stopped/holding peer (see _reroute), so it
        # keeps moving and exactly one tug ever gives way -- no mutual hold.
        peer = self._peer_state_cached()
        in_col = self._col_hit(x, y) or self._col_hit(ax, ay)
        if peer is not None and peer.get("x") is not None:
            pxc, pyc = float(peer["x"]), float(peer["y"])
            dctr = math.hypot(pxc - x, pyc - y)
            dpred = math.hypot(pxc - ax, pyc - ay)
            pid = peer.get("id", "peer")
            pv = abs(float(peer.get("v_linear") or 0.0))
            p_moving = (pv >= self.PEER_STATIONARY_V and not bool(
                (peer.get("idle_loop") or {}).get("holding")))
            if in_col:
                # INSIDE THE COLUMN the tie is broken by the LEASE, never by
                # geometry -- that is the whole point of the mutex, and a
                # spatial tie-break in a corridor this narrow is what
                # deadlocked the line last time. The holder does not yield;
                # it only refuses to drive THROUGH a body, which can only
                # happen if the peer was chat-paused or aborted in here, and
                # then MAX_HOLD_S turns the hold into a re-plan rather than a
                # collision. A tug WITHOUT the lease has no business here at
                # all and stops for anything close.
                if self.col_hold:
                    if dpred < self.SAFETY_RADIUS:
                        return (True, f"peer {pid} left inside the column "
                                      f"({dctr:.2f} m) — holding, not passing")
                elif min(dctr, dpred) < self.YIELD_RADIUS:
                    return (True, f"no column lease; peer {pid} at "
                                  f"{dctr:.2f} m")
            elif not self._has_priority(peer):
                # predictive yield: give way to a MOVING peer that is roughly
                # ahead on my heading (it will pass and clear). A STATIONARY
                # peer is routed AROUND by _reroute instead -- yielding to
                # something that will not move just sits until the
                # hold-timeout (measured a 40 s startup stall while the peer
                # idled at its home).
                if p_moving and dctr < self.YIELD_RADIUS:
                    brg = math.atan2(pyc - y, pxc - x)
                    if (abs(wrap_pi(brg - hd)) < self.CONVERGE_CONE
                            or dctr < self.SAFETY_RADIUS + 0.4):
                        return (True, f"yield to {pid} ({dctr:.2f} m ahead)")
                # reactive hard guard: never let my predicted centre overlap it
                if dpred < self.SAFETY_RADIUS:
                    return (True, f"peer {pid} in path ({dctr:.2f} m)")
                # ...nor its towed cart (trails ~_trail behind the peer heading)
                if peer.get("carrying"):
                    pyaw = float(peer.get("yaw", 0.0))
                    cxp = pxc - self._trail * math.cos(pyaw)
                    cyp = pyc - self._trail * math.sin(pyaw)
                    if math.hypot(cxp - ax, cyp - ay) < self.CART_CLEAR:
                        return (True, f"peer {pid} cart in path")

        # 2) PARKED / STANDING CARTS (live poses; skip the one we tow or dock,
        # and stand down inside the west work zone where every cart is the
        # tug's own choreographed work -- the row of parked carts this guards
        # against is out east, never in the zone).
        mine = self.bridge.carrying
        if not self._in_work_zone(x, y):
            for d, p in self._cart_xy_cached().items():
                if d == mine or d == near_cart or p is None:
                    continue
                if math.hypot(p[0] - ax, p[1] - ay) < self.CART_CLEAR:
                    return (True, f"cart {d} in path")

        # 3) STATIC scene — every solid structure the world derivation found
        # (walls, fences, conveyor kerbs + drive heads, belt frames, the
        # outfeed spur, the feeder, the arm pedestal), tested against the
        # ORIENTED footprint on this leg's heading.
        hit = self._static_incursion(ax, ay, hd)
        if hit is not None:
            return (True, f"{hit} in path")
        return (False, "")

    def _enter_hold(self, reason: str) -> None:
        if not self.holding:
            self.holds_total += 1
        self.holding = True
        self.hold_reason = reason
        self._log(f"HOLD ({self.leg}): {reason}")

    def _exit_hold(self) -> None:
        if self.holding:
            self._log(f"clear ({self.leg}); resuming")
        self.holding = False
        self.hold_reason = ""

    def _exit_hold_if_any(self) -> None:
        if self.holding:
            self._exit_hold()

    def _reissue_drive(self, tx: float, ty: float,
                       speed: Optional[float]) -> bool:
        """Re-face + re-issue the remaining distance to (tx, ty) after a hold.
        Returns False if an operator command has arrived (leg should abort)."""
        pose = self._pose()
        if pose is None:
            return False
        rem = math.hypot(tx - pose[0], ty - pose[1])
        # One band, read from the class (see REAIM_MIN_M). This was a literal
        # 0.10 -- NARROWER than the 0.12 arrival band both call sites test
        # first, so it could never fire and every sub-tolerance remainder got
        # re-aimed at a bearing that was pure cross-track noise. Kept here as
        # well as in _drive_guarded because the post-hold call site reaches
        # this function directly, and a 15 cm remainder is just as meaningless
        # after a hold as it is after a completed drive.
        if rem <= self.REAIM_MIN_M:
            return True
        if not self._face(math.atan2(ty - pose[1], tx - pose[0])):
            return False
        return self._call(lambda: (None if self._blocked()
                                   else self._applied(
                                       self.bridge.act_drive_forward(
                                           rem, speed,
                                           source=self.bridge.SOURCE_IDLE_LOOP)
                                   ))) is not None

    def _drive_guarded(self, dist: float, speed: Optional[float],
                       tx: float, ty: float,
                       near_cart: Optional[str] = None) -> bool:
        """Drive ``dist`` m toward (tx, ty), holding (a visible decel to zero)
        whenever the peer tug or a static/parked footprint would be entered,
        and resuming when clear. An operator command aborts instantly (returns
        False, so chat wins). A hold is bounded by MAX_HOLD_S -> deadlock-break
        and proceed, so a stuck peer can never freeze the line."""
        b = self.bridge
        if abs(dist) < 0.02:
            return True
        v = abs(speed) if speed else b.cruise_linear
        v_eff = max(0.1, v)
        self._leg_break = False
        if self._call(lambda: (None if self._blocked()
                               else self._applied(b.act_drive_forward(
                                   dist, speed,
                                   source=b.SOURCE_IDLE_LOOP)))) is None:
            return False
        cap = abs(dist) / v_eff * 3.0 + 8.0
        t0 = time.time()
        extra = 0.0                      # wall time spent holding, refunds cap
        holding = False
        hstart = 0.0
        hcap = self.MAX_HOLD_S
        ARRIVE_TOL = self.ARRIVE_TOL   # same 0.12; see the class constant
        while time.time() - t0 < cap + extra:
            if self._blocked():
                self._exit_hold_if_any()
                return False
            pose = self._pose()
            if pose is None:
                time.sleep(0.1)
                continue
            # ARRIVAL is measured by DISTANCE to the target, never by motion
            # going idle -- because a hold calls act_stop, which also makes
            # motion idle, and misreading that as "arrived" would end the leg
            # at the hold point (observed as a creep toward the obstacle).
            rem = math.hypot(tx - pose[0], ty - pose[1])
            if rem <= ARRIVE_TOL:
                self._exit_hold_if_any()
                self._col_tick()
                return True
            self._col_tick()
            hold, reason = (False, "")
            if self._avoid_ready and not self._leg_break:
                hold, reason = self._should_hold(tx, ty, near_cart)
            if hold:
                if not holding:
                    holding = True
                    hstart = time.time()
                    # A structure will not move; a peer will. Wait seconds for
                    # the first and up to MAX_HOLD_S for the second.
                    hcap = (self.STATIC_HOLD_S if "peer" not in reason
                            else self.MAX_HOLD_S)
                    self._enter_hold(reason)
                    # INTERNAL: the loop stopping ITSELF for an obstacle.
                    # source=SOURCE_IDLE_LOOP keeps act_stop from arming the
                    # operator hold (the loop would freeze itself for a
                    # minute every time it yielded to the peer) and from
                    # spending its settling window -- this runs on the sim
                    # thread via _call, which cannot advance while it waits.
                    self._call(lambda: b.act_stop(source=b.SOURCE_IDLE_LOOP))
                elif time.time() - hstart > hcap:
                    held = time.time() - hstart
                    self.hold_secs_max = max(self.hold_secs_max, held)
                    # ABORT the leg rather than drive straight through the
                    # obstacle (driving through would breach the safety radius).
                    # The cycle re-plans on its next pass; _goto then routes
                    # around the now-confirmed-stationary blocker. Bounded, so
                    # a stuck peer can never permanently freeze the line, and
                    # the tug never collides to break a deadlock.
                    self._log(f"[hold-timeout] held {held:.0f}s for {reason}; "
                              f"abandoning leg to re-plan (reroute)")
                    self._exit_hold()
                    return False
                extra += 0.15
                time.sleep(0.15)
                continue
            if holding:
                # peer/obstacle cleared -> resume the remaining distance
                self.hold_secs_max = max(self.hold_secs_max,
                                         time.time() - hstart)
                holding = False
                self._exit_hold()
                if not self._reissue_drive(tx, ty, speed):
                    return False
            else:
                # Not holding and not arrived: keep the motor running. If the
                # commanded distance completed short of the target (earlier
                # holds ate into it) re-issue the remainder -- UNLESS the
                # remainder is too small to carry a bearing, in which case
                # accept the leg. Re-aiming at atan2 of a 14 cm residual is a
                # 271 deg pirouette, cart in tow, to gain 5.6 cm; the numbers
                # and the 0.165 m ceiling on the bound are on REAIM_MIN_M.
                with b.lock:
                    kind = b.motion[0]
                if kind == "idle":
                    if rem <= self.REAIM_MIN_M:
                        # RETURN True -- do not merely skip the re-issue.
                        # Nothing below this point moves the tug, so kind
                        # stays "idle" and rem stays put: a bare `continue`
                        # spins here at ~6.7 Hz until `cap + extra` runs out
                        # (only the HOLD branch refunds `extra`), then falls
                        # out the bottom and FAILS the leg with "guarded
                        # drive timed out". The leg is done; say so.
                        self._exit_hold_if_any()
                        self._col_tick()
                        return True
                    if not self._reissue_drive(tx, ty, speed):
                        return False
            # ~6 obstacle checks/s: responsive next to the 0.55 m lookahead
            # (the tug moves <=0.15 m between checks at 1 m/s) while keeping the
            # extra supervisor traffic off the sim thread's realtime budget.
            time.sleep(0.15)
        if holding:
            self.hold_secs_max = max(self.hold_secs_max, time.time() - hstart)
        self._exit_hold_if_any()
        self._log("guarded drive timed out (continuing)")
        return False

    # ── mission legs ──────────────────────────────────────────────

    def _dock(self) -> bool:
        b = self.bridge
        # Already holding it? Then we are docked, and the approach would be
        # absurd: the cart is rigidly attached, so driving to its hitch means
        # chasing a target that moves with us. Short-circuit before any
        # motion. (The bridge's attach is idempotent for the same reason --
        # this is the belt to that braces.)
        if b.carrying and b.carrying == self.trolley_def:
            return True
        tp = self._trolley_pose()
        if tp is None:
            return False
        tx, ty, psi = tp
        ox, oy = math.cos(psi), math.sin(psi)      # hitch outward dir (+x)
        hx, hy = tx + b._HITCH_LOCAL[0] * ox, ty + b._HITCH_LOCAL[0] * oy
        px, py = hx + self.PRE_DOCK * ox, hy + self.PRE_DOCK * oy
        # ── ALREADY AT THE STAND-OFF? THEN ISSUE NO TRAVEL LEG. ──────────
        # Measured on the COLLECT dock, and it is pure arithmetic: a parked
        # cart stands at y=4.20, its hitch is _HITCH_LOCAL (0.70) north of
        # that, and the pre-dock point is another PRE_DOCK (1.55) north --
        # y=6.45. But the tug is already standing on the back aisle at
        # PARK_AISLE_Y=6.60. The "approach" is therefore 6.60-6.45 = 0.15 m,
        # which is JUST outside the 0.12 skip band, so _goto fired: a 90 deg
        # turn to face south, a 3 cm crawl (the guarded drive stops the
        # moment it is within ARRIVE_TOL of the target, so 0.15-0.12 is all
        # it ever travelled), then a 180 deg turn back north onto the hitch
        # heading. 270 deg of spinning and two settles to move 15 cm -- and
        # on roughly HALF of all collections, because the tug's own y is
        # 6.60 +- ARRIVE_TOL, so the residual falls either side of the band.
        #
        # The residual is ALONG the hitch axis -- the same axis the dock
        # reverse already runs on -- so it needs no travel leg at all: fold
        # it into the reverse and the tug finishes in exactly the same place
        # having turned ONCE instead of three times. Only a LATERAL miss
        # genuinely needs driving out, and that is what the _goto is kept
        # for: the leg is skipped only when the perpendicular offset is
        # already inside the band _goto itself would have delivered
        # (ARRIVE_TOL), so this can never dock less accurately than today --
        # worst case it declines to skip and behaves exactly as before.
        #
        # Bounds, both from constants already on this class: the fold-in is
        # taken only when the tug is AT OR BEYOND the designed stand-off
        # (reverse >= DOCK_REVERSE), and the resulting blind reverse is
        # capped at PRE_DOCK, so the tug never backs up further than the
        # stand-off distance the dock was designed around. On the measured
        # geometry that is a 1.20 m reverse instead of 1.05 m, leaving the
        # rear coupling 0.13 m past the hitch -- identical to the nominal
        # dock, and well inside the bridge's 0.60 m _DOCK_RADIUS_M.
        at_standoff = False
        reverse = self.DOCK_REVERSE
        pose = self._pose()
        if pose is not None:
            dx, dy = pose[0] - hx, pose[1] - hy
            along = dx * ox + dy * oy       # + = out along the hitch axis
            lat = abs(dy * ox - dx * oy)    # perpendicular miss
            fold = along - (self.PRE_DOCK - self.DOCK_REVERSE)
            if (lat <= self.ARRIVE_TOL
                    and self.DOCK_REVERSE <= fold <= self.PRE_DOCK):
                at_standoff = True
                reverse = fold
                self._log(f"already at the {self.trolley_def} stand-off "
                          f"({along:.2f} m out, {lat:.2f} m off-axis) — "
                          f"no approach leg, reversing {reverse:.2f} m")
        # near_cart exempts THIS cart from the obstacle guard: docking is a
        # deliberate close approach to its hitch, not an incursion.
        if not at_standoff and not self._goto(px, py, speed=0.9,
                                              near_cart=self.trolley_def):
            return False
        if not self._face(math.atan2(oy, ox)):
            return False
        if not self._drive(-reverse, speed=0.35):
            return False
        res = self._call(lambda: b.act_attach_trolley(self.trolley_def)) or {}
        tries = 0
        while res.get("error") == "too_far" and tries < 3:
            if not self._drive(-0.18, speed=0.3):
                return False
            res = self._call(
                lambda: b.act_attach_trolley(self.trolley_def)) or {}
            tries += 1
        if not res or "error" in res:
            self._log(f"dock failed: {res}")
            return False
        self._log(f"attached to {self.trolley_def} "
                  f"({len(res.get('riders') or [])} part(s) aboard)")
        return True

    # ══ CLOSED-LINE MODES ('dispatch' + 'trolley_return') ═════════════
    #
    # COORDINATION — deliberately the simplest thing that is robust:
    #
    #  1. ONE MASTER, NO CONSENSUS. The arm bridge owns the line state and
    #     publishes it at /state["line"]; both tugs are pure followers that
    #     poll it. Nothing here votes, elects or hands off a token, so a
    #     tug that is paused, restarted or interrupted mid-cycle simply
    #     re-reads the world on its next pass and rejoins.
    #  2. ROLES ARE PARTITIONED BY PLACE, NOT BY NEGOTIATION. tug_a only
    #     ever touches the trolley standing at the CART PICKUP; tug_b only
    #     ever touches trolleys at the DISPATCH BAY or the BUFFER. Two tugs
    #     therefore cannot select the same cart -- not because they agreed
    #     not to, but because the sets are disjoint by construction. This
    #     is what makes the swap look intentional without a scheduler.
    #  3. A trolley is FREE only when the master says its load has shipped
    #     (gone from line["in_transit"]) and it is not the one being loaded.
    #  4. The single shared resource left is the east-west transit LANE, so
    #     that -- and only that -- gets a lock: each tug publishes on_lane
    #     in its /state and yields to the peer (--idle-peer-port) before
    #     entering. Dispatch has priority; the return leg waits.

    def _http_state(self, port: int) -> Optional[dict]:
        if not port:
            return None
        import urllib.request
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/state",
                                        timeout=0.8) as r:
                return json.loads(r.read().decode("utf-8"))
        except Exception:
            return None

    def _line_state(self) -> Optional[dict]:
        """The line master's published state, or None when the arm bridge is
        unreachable (then the loop just idles -- never guesses)."""
        st = self._http_state(self.arm_port)
        if not st:
            return None
        ln = st.get("line")
        return ln if isinstance(ln, dict) and ln.get("active") else None

    def _peer_on_lane(self) -> bool:
        st = self._http_state(self.peer_port)
        if not st:
            return False
        return bool((st.get("idle_loop") or {}).get("on_lane"))

    def _lane_acquire(self, priority: bool) -> bool:
        """Claim the shared transit lane. The dispatch leg has priority and
        only glances; the return leg genuinely waits for a clear lane so the
        two tugs never meet head-on on it."""
        if self.peer_port:
            budget = 4.0 if priority else self.LANE_WAIT_S
            t0 = time.time()
            waited = False
            while time.time() - t0 < budget and self._peer_on_lane():
                if self._blocked():
                    return False
                if not waited:
                    waited = True
                    self._log("holding for the peer tug to clear the lane")
                time.sleep(0.5)
            if waited and not self._peer_on_lane():
                self._log("lane clear")
        self.on_lane = True
        return True

    def _lane_release(self) -> None:
        self.on_lane = False

    def _at_spot(self, xy, spot) -> bool:
        return (xy is not None
                and math.hypot(xy[0] - spot[0], xy[1] - spot[1])
                <= self.SPOT_TOL)

    def _busy_trolleys(self, line: dict) -> set:
        """Trolleys the master still owns: the one being loaded plus every
        one whose delivered load has not shipped yet."""
        busy = set()
        loaded = line.get("loaded") or {}
        if loaded.get("trolley"):
            busy.add(loaded["trolley"])
        for e in line.get("in_transit") or []:
            if e.get("trolley"):
                busy.add(e["trolley"])
        return busy

    def _spot_free_xy(self, xy: dict, spot) -> bool:
        return not any(self._at_spot(p, spot) for p in xy.values())

    def _outfeed_leg_clear(self, xy: dict) -> bool:
        """No cart is anywhere on the conveyor's SOUTH leg (station ->
        pickup), the stretch a loaded cart crosses on its way out.

        Staging the next empty means driving this tug DOWN that leg: the push
        onto the station finishes with the tug about 1.2 m south of it, so a
        cart still travelling out would be run into. Step A normally makes
        that impossible by blocking on the carry, but a chat pause can drop
        this loop out mid-run and it re-reads the world rather than trusting
        saved state -- so the invariant is asserted on live poses here rather
        than assumed from control flow. Two spots on one conveyor is exactly
        the kind of thing that only bites once the demo is being watched.

        A cart parked AT the pickup is fine and does not count: it is 0.62 m
        clear of where this tug comes to rest (see DEF CART_PICKUP)."""
        if not self.has_pickup:
            return True
        sx, sy = self.station_xy
        py = self.pickup_xy[1]
        lo, hi = min(sy, py), max(sy, py)
        for p in xy.values():
            if p is None or abs(p[0] - sx) > 0.9:
                continue
            if lo + self.SPOT_TOL < p[1] < hi - self.SPOT_TOL:
                return False
        return True

    def _find_at_xy(self, xy: dict, spot, exclude: set) -> Optional[str]:
        for d, p in xy.items():
            if d in exclude:
                continue
            if self._at_spot(p, spot):
                return d
        return None

    def _one_trolley_xy(self, def_name: str):
        def impl():
            node = self.bridge.robot.getFromDef(def_name)
            if node is None:
                return None
            p = node.getPosition()
            return (p[0], p[1])

        return self._call(impl)

    def _spot_of(self, xy, tol: Optional[float] = None):
        """Which park spot (if any) a cart is standing on. Defaults to the
        generous OCCUPY_TOL -- see the constant for why."""
        if xy is None:
            return None
        t = self.OCCUPY_TOL if tol is None else tol
        best = None
        for sd, s in self.park_spots:
            d = math.hypot(xy[0] - s[0], xy[1] - s[1])
            if d <= t and (best is None or d < best[0]):
                best = (d, sd, s)
        return (best[1], best[2]) if best else None

    def _free_park_spot(self, xy: dict):
        """First park spot with no cart on it, scanned in world order.

        Scanning in order (rather than nearest-first) is what makes the row
        fill left to right and stay looking deliberate.

        ORDER IS NOT FREE, and this docstring used to claim it was ("because
        every spot is entered straight off the lane, order costs nothing in
        travel"). It is entered off the lane, but the tow has to RUN ALONG
        that lane to get there: the spots are 1.40 m apart (5.90 .. 12.90 in
        this world), so every index further east costs 1.40 m of towing on
        the way in and 1.40 m on the way back out to PARK_LINK_X -- 2.80 m
        per index, per cycle. Scanning from index 1 is therefore also the
        CHEAPEST choice, which is why this stays as it is; the matching
        choice on the way out is in _collect_from_row."""
        taken = set()
        for p in xy.values():
            hit = self._spot_of(p)
            if hit is not None:
                taken.add(hit[0])
        for sd, s in self.park_spots:
            if sd not in taken:
                return (sd, s)
        return None

    def _free_spot_count(self, xy: dict) -> int:
        taken = set()
        for p in xy.values():
            hit = self._spot_of(p)
            if hit is not None:
                taken.add(hit[0])
        return max(0, len(self.park_spots) - len(taken))

    # ── shared legs ───────────────────────────────────────────────

    def _park_at_staging(self) -> bool:
        """Drive home empty-handed via the southern return lane, so an empty
        tug never runs nose-to-nose with a tow on the transit lane."""
        self.leg = "returning"
        sx, sy = self.staging_xy
        pose = self._pose()
        if pose is None:
            return False
        # If we are up behind the park row, leave along the back aisle and
        # come down the link -- driving straight south from here would cut
        # through the row.
        if pose[1] > 2.0 and pose[0] > self.PARK_LINK_X + 0.3:
            if not self._goto(pose[0], self.PARK_AISLE_Y, speed=0.8):
                return False
            if not self._goto(self.PARK_LINK_X, self.PARK_AISLE_Y,
                              speed=self.LONG_SPEED):
                return False
            pose = self._pose()
            if pose is None:
                return False
        self._lane_release()          # off the transit lane now; hand it over
        if abs(pose[1] - self.Y_RETURN) > 0.4:
            if not self._goto(pose[0], self.Y_RETURN, speed=0.8):
                return False
        if not self._goto(sx, self.Y_RETURN, speed=self.LONG_SPEED):
            return False
        if not self._goto(sx, sy, speed=0.9):
            return False
        self._face(self.staging_heading)
        self.leg = "idle"
        self._col_release()   # home and empty-handed: the column is free
        return True

    def _tow_to(self, trolley: str, target, label: str,
                approach: str = "east", speed: Optional[float] = None) -> bool:
        """Dock a cart where it stands and tow it onto ``target``, finishing
        on a heading that leaves its hitch pointing somewhere useful.

        ``approach`` is the whole trick. A one-trailer tow leaves the cart
        directly behind the tug, so the tug's final heading IS the cart's
        final hitch direction -- and the hitch direction decides who can dock
        that cart next and from which side. 'south' leaves it hitch-south for
        a tug coming up off the lane; 'east' leaves it hitch-east for a tug
        coming along the lane. Getting this wrong does not fail loudly, it
        just makes the next dock reverse into a wall."""
        b = self.bridge
        self.trolley_def = trolley
        if not self._dock():
            return False
        tx, ty = target
        spd = speed or self.TOW_SPEED
        if approach == "south":
            if not self._goto(tx, ty - self.SOUTH_APPROACH, speed=spd):
                return False
            if not self._face(-math.pi / 2.0):
                return False
            pose = self._pose()
            if pose is None:
                return False
            d = pose[1] - (ty - self._trail)
            if d > 0.05 and not self._drive(d, speed=0.35):
                return False
        else:
            if not self._goto(tx + self._trail, ty, speed=spd):
                return False
        self._call(b.act_detach_trolley)
        self._log(f"{trolley} set down at the {label} "
                  f"({tx:+.2f},{ty:+.2f})")
        return True

    # ── mode 'dispatch' (tug_a): loaded cart -> a free park spot ───

    def _park_in_spot(self, trolley: str, spot_def: str, spot) -> bool:
        b = self.bridge
        sx, sy = spot
        self.leg = "to_park"
        pose = self._pose()
        if pose is None:
            return False
        # Off the conveyor column onto the transit lane, then the long east
        # run. This is a plain _goto rather than a signed drive precisely so
        # it does not care which side of the lane the dock left us on: from
        # the old station pickup it was a 0.25 m nudge north, from CART_PICKUP
        # it is a 2.8 m run north, and the trailer pursuit handles both.
        if not self._goto(pose[0], self.Y_LANE, speed=self.TOW_SPEED):
            return False
        if not self._goto(sx, self.Y_LANE, speed=self.LONG_SPEED):
            return False
        # Push it straight north into the spot. The tug finishes _trail north
        # of the cart, which leaves the cart hitch-NORTH -- the side this tug
        # has to come back to when it collects this cart later.
        if not self._face(math.pi / 2.0):
            return False
        if not self._drive(sy + self._trail - self.Y_LANE, speed=0.4):
            return False
        # CLOSE THE LOOP ON THE CART, not on the tug. The open-loop drive above
        # assumes the tug started exactly on the lane and that the trailer sits
        # exactly _trail behind it. Neither is true: measured error was 0.66 m
        # short, which is more than SPOT_TOL, so the spot still read FREE and
        # the NEXT dispatch drove a second cart into the same square (observed
        # live as "7/6 spots taken"). A couple of short corrective nudges cost
        # a few seconds and make the row actually mean something.
        for _ in range(4):
            cp = self._one_trolley_xy(trolley)
            if cp is None:
                break
            dy = sy - cp[1]
            if abs(dy) <= self.PARK_PLACE_TOL:
                break
            if not self._drive(dy, speed=0.28):
                break
        self._call(b.act_detach_trolley)
        final = self._one_trolley_xy(trolley)
        if trolley in self.park_order:
            self.park_order.remove(trolley)
        self.park_order.append(trolley)
        # MONOTONIC delivery count. park_order is the row's CURRENT
        # occupancy, not a tally: a collection out through dock E removes an
        # entry, so len(park_order) oscillates (measured 4->5->4->5 over one
        # 7-minute run). A deferred "do one more delivery, then wait" keyed
        # off that length can never fire -- it expired instead. Count parks.
        self.parks_total += 1
        self.jobs_total += 1
        off = ""
        if final is not None:
            err = math.hypot(final[0] - sx, final[1] - sy)
            off = f", cart at ({final[0]:+.2f},{final[1]:+.2f}) err={err:.2f}m"
            if err > self.SPOT_TOL:
                off += " [WARN: off-spot]"
        self._log(f"PARKED {trolley} at {spot_def} ({sx:+.2f},{sy:+.2f})"
                  f"{off} — {len(self.park_order)}/{len(self.park_spots)} "
                  f"spots taken")
        # Home the long way: north to the back aisle, west along it. One-way,
        # so the tug never reverses past the row it just added to.
        self.leg = "back_aisle"
        if not self._goto(sx, self.PARK_AISLE_Y, speed=0.7):
            return False
        return True

    def _collect_from_row(self, busy: Optional[set] = None) -> bool:
        """Take one parked cart out through dock E -- the WEST-MOST eligible
        one, i.e. the first in the same scan order _free_park_spot fills. This
        is what makes a FULL park row a pause rather than a deadlock.

        WHY WEST-MOST, NOT OLDEST (it was "collect the oldest" until now).
        The spot-dependent travel of a whole dispatch cycle collapses to a
        single term. The park tows the cart east along Y_LANE to its spot
        (x_P - x_pickup). The collection then runs the back aisle from the
        spot just filled to the cart being taken (|x_C - x_P|), and from there
        west to PARK_LINK_X (x_C - PARK_LINK_X). Everything else -- the push
        north into the spot, the dock, the run south to Y_OUTBOUND, dock E,
        the trip home -- is the same length wherever in the row those two
        carts are. So

            travel(x_P, x_C) = x_P + |x_C - x_P| + x_C + const
                             = 2 * max(x_P, x_C) + const

        Two things follow, and neither needs a stopwatch. (1) For THIS cycle
        every candidate at or west of the spot just filled costs exactly the
        same, because the tug drives back past them anyway -- so taking the
        west-most is never dearer. (2) For the NEXT cycle it is strictly
        better: a collection is what frees a spot, _free_park_spot then takes
        the west-most free spot, and 2*max() is monotone in x_P -- so freeing
        the west-most spot weakly lowers every future cycle as well. Greedy
        west is therefore optimal at every step, not merely cheaper now.

        Measured against this world's six spots (5.90..12.90, 1.40 m apart,
        four occupied at spawn, PARK_RESERVE=1): oldest-first marches the free
        spot across the whole row -- collecting from spots 1,2,3,4,5,1,2,...
        -- and averages 19.64 m/cycle of spot-dependent travel. West-most
        settles into a two-spot churn at 14.74 m/cycle. That is 4.90 m, and
        about 5 s at TOW_SPEED, saved every cycle -- off the one tug that is
        on the line's takt.

        WHAT IT COSTS: carts at the east end stop rotating and become
        long-stay residents while spots 1 and 2 do the churn. That is
        deliberate, and it is safe, because the rule only changes WHICH cart
        is taken, never HOW MANY: the row's occupancy -- and so the number of
        carts circulating in the ring, and when the reserve fires -- is set by
        the park/collect pairing and PARK_RESERVE, and is identical either
        way. Nor can it strand a BOX: the master ships a delivered load on a
        timer once its cart is at rest away from the fill spot (arm bridge
        SHIP_AFTER_S = 12 s), not when the cart is collected. Every cart in
        the ring is an interchangeable empty, so "oldest" was cosmetic --
        2.80 m per index of cosmetic.

        ``busy`` is the master's set of carts whose load has not shipped yet.
        Those are skipped: sending a cart out of the building with its box
        still on it would silently delete a box from the line, and the line
        only has three."""
        b = self.bridge
        busy = busy or set()
        xy = self._all_trolley_xy()
        # OCCUPANCY MAP, built with _spot_of -- the SAME reach (OCCUPY_TOL,
        # 1.05 m) the free-spot scan uses. It has to be the same: _find_at_xy
        # matches on SPOT_TOL (0.55 m), so a cart that parked sloppily
        # (measured 0.66 m short -- see OCCUPY_TOL) would read TAKEN to the
        # parker and INVISIBLE to the collector, and the row would quietly
        # lose that spot for the rest of the run. One tolerance, both
        # directions.
        at = {}
        for d, p in xy.items():
            hit = self._spot_of(p)
            if hit is not None:
                at.setdefault(hit[0], (d, hit))
        # PRUNE THE CENSUS. This used to happen as a side effect of the FIFO
        # walk that also picked the candidate; the pick no longer walks
        # park_order, so a cart that has left the row (collected, or moved by
        # an operator) must be dropped explicitly -- otherwise park_order, and
        # the park_row_occupants it publishes in /state, would drift.
        for d in list(self.park_order):
            if self._spot_of(xy.get(d)) is None:
                self.park_order.remove(d)
        cand = None
        for sd, s in self.park_spots:
            e = at.get(sd)
            if e is not None and e[0] not in busy:
                cand = e
                break
        if cand is None:
            return False
        trolley, (spot_def, (sx, sy)) = cand
        # Standing rules are checked BEFORE the job starts, same as the
        # loaded-cart collection above.
        if self._job_forbidden((sx, sy),
                               f"collect {trolley} from {spot_def}"):
            return False
        # `at` is keyed by spot DEF, so len(at) IS the occupied count -- the
        # very number _free_spot_count would rebuild from the same dict, so
        # the old `free = self._free_spot_count(xy)` here was a third walk of
        # the carts for an answer already in hand. `held` is counted over the
        # ROW, not over the master's whole busy set, so the line says how many
        # of THESE carts are still loaded rather than how many exist.
        held = sum(1 for e in at.values() if e[0] in busy)
        self._log(f"park row at {len(at)}/{len(self.park_spots)} — collecting "
                  f"{trolley} from {spot_def} (west-most of the "
                  f"{len(at) - held} eligible; {held} still loaded) for the "
                  f"outbound truck")
        if not self._lane_acquire(priority=False):
            return False
        self.leg = "to_collect"
        # Parked carts face NORTH, so the approach is down the back aisle.
        pose = self._pose()
        if pose is None or pose[1] < self.PARK_AISLE_Y - 0.5:
            if not self._goto(self.PARK_LINK_X, self.Y_LANE,
                              speed=self.LONG_SPEED):
                self._lane_release()
                return False
            if not self._goto(self.PARK_LINK_X, self.PARK_AISLE_Y, speed=0.9):
                self._lane_release()
                return False
        if not self._goto(sx, self.PARK_AISLE_Y, speed=self.LONG_SPEED):
            self._lane_release()
            return False
        self.trolley_def = trolley
        if not self._dock():
            self._lane_release()
            return False
        self.leg = "to_dock_e"
        if not self._goto(sx, self.PARK_AISLE_Y, speed=0.45):
            self._lane_release()
            return False
        if not self._goto(self.PARK_LINK_X, self.PARK_AISLE_Y,
                          speed=self.LONG_SPEED):
            self._lane_release()
            return False
        if not self._goto(self.PARK_LINK_X, self.Y_OUTBOUND, speed=0.8):
            self._lane_release()
            return False
        ox, oy = self.out_infeed_xy
        # RETRY THE LAST LEG ONCE, SLOWER. This is the one leg whose failure
        # is expensive out of all proportion to its length: giving up here
        # leaves the tug holding a cart with the row full, and the resume
        # branch then parks it straight back -- a park/collect churn that
        # never gets a cart out of the building and starves tug_b of empties
        # (observed as 6/6 spots taken for the whole back half of a run). One
        # retry at half speed costs a few seconds and breaks the cycle.
        if not self._goto(ox + self._trail, oy, speed=self.LONG_SPEED):
            self._log("dock E approach aborted — one more try at half speed")
            if not self._goto(ox + self._trail, oy, speed=0.5):
                self._lane_release()
                return False
        self._call(b.act_detach_trolley)
        if trolley in self.park_order:
            self.park_order.remove(trolley)
        self._log(f"{trolley} set down at the dock E infeed "
                  f"({ox:+.2f},{oy:+.2f})")
        # CLEAR THE DECK BEFORE STARTING THE CHAIN. The tug is standing east
        # of the cart, which is exactly where the conveyor is about to take
        # it -- start the chain first and the cart runs through the tug.
        if not self._face(-math.pi / 2.0):
            return False
        if not self._drive(2.0, speed=0.8):
            return False
        self._lane_release()
        self._run_conveyor(trolley, self.out_exit_xy, 0.0, self.OUT_DOG_DEF,
                           "outbound conveyor")
        # A collection is a completed JOB even though it is not a delivery.
        self.jobs_total += 1
        return True

    def _try_dock_handoff(self) -> bool:
        """Hand a cart that has reached the outbound door back to the inbound
        door: same cart, both ends inside a doorway, and only when the inbound
        side is genuinely clear -- so a busy lane produces back-pressure at
        the exit instead of a pile-up at the entry.

        This is the single non-continuous cart move in the demo. It is what
        closes the ring, and confining it to one call site is what keeps it
        from quietly becoming a general-purpose teleport."""
        xy = self._all_trolley_xy()
        at_exit = [d for d, p in xy.items()
                   if self._at_spot(p, self.out_exit_xy)]
        if not at_exit:
            return False
        if not self._spot_free_xy(xy, self.lane_entry_xy):
            return False
        if self.bridge.conveyor_busy():
            return False
        d = at_exit[0]
        res = self._call(lambda: self.bridge.transfer_cart(
            d, self.lane_entry_xy, 0.0))
        if res and "error" not in res:
            self._log(f"{d} went out on the outbound truck and came back as "
                      f"an empty at the dock 2 door — the cart ring is "
                      f"conserved ({len(self.bridge.pallet_defs)} carts)")
            return True
        self._log(f"dock handoff of {d} refused: {res}")
        return False

    def _hold_west(self) -> bool:
        """Stand off at the west hold between jobs instead of trailing all the
        way back to the staging zone. tug_b's four jobs are all within a few
        metres of each other; the round trip to staging after every one of
        them was ~16 m of driving that the ARM pays for, because it is what
        the arm waits behind. Going home is for having nothing to do."""
        self.leg = "holding"
        hx, hy = self.WEST_HOLD
        pose = self._pose()
        if pose is not None and math.hypot(pose[0] - hx, pose[1] - hy) < 0.4:
            self.leg = "idle"
            self._col_release()   # standing off the job: hand the column back
            return True
        ok = self._goto(hx, hy, speed=0.9)
        self.leg = "idle"
        # The hold is OUTSIDE the column by construction, so arriving here
        # means the job is done and the peer may have it. Explicit, so the
        # peer does not have to wait out the COL_RELEASE_DWELL_S grace.
        self._col_release()
        return ok

    def _cycle_dispatch(self) -> None:
        b = self.bridge
        if self._pause_gate():
            time.sleep(0.4)
            return
        if b.carrying:
            # Interrupted mid-tow: finish the job the world can see us doing.
            self.trolley_def = b.carrying
            xy = self._all_trolley_xy()
            spot = self._free_park_spot(xy)
            done = False
            if spot is not None:
                self._log(f"resuming with {b.carrying} in tow — completing the "
                          f"park at {spot[0]}")
                if self._park_in_spot(b.carrying, spot[0], spot[1]):
                    self._park_at_staging()
                    done = True
                    # THE RING CLOSED HERE, so count it here. `cycles` used to
                    # be assigned ONLY at the tail of an uninterrupted cycle,
                    # and an operator command aborts the cycle wherever it
                    # lands (by design -- chat wins instantly), leaving this
                    # branch to finish the job on the next pass. So a cycle
                    # that was interrupted and then COMPLETED still read as
                    # never having happened, and the next pass announced the
                    # same number again. MEASURED, show2_idle.txt: paused
                    # mid-park at t=143.3, resumed t=179.5, "PARKED
                    # TROLLEY_PAYLOAD at PARK_SPOT_5" t=201.7 -- and t=247.0
                    # still announced "dispatch #1", its third time. The work
                    # counters (parks_total/jobs_total, bumped inside
                    # _park_in_spot) were right throughout; only this one was
                    # wrong, which is why the cart really did get parked.
                    self.cycles += 1
            else:
                # No free spot and we cannot collect while towing. Send this
                # one out instead of stalling: a cart leaving through dock E
                # is always a legal move, and it re-enters as an empty.
                self._log(f"resuming with {b.carrying} in tow and no free "
                          "spot — taking it straight out through dock E")
                ox, oy = self.out_infeed_xy
                if self._goto(self.PARK_LINK_X, self.Y_OUTBOUND, speed=0.8) \
                        and self._goto(ox + self._trail, oy,
                                       speed=self.LONG_SPEED):
                    self._call(b.act_detach_trolley)
                    self._face(-math.pi / 2.0)
                    self._drive(2.0, speed=0.8)
                    self._run_conveyor(b.carrying, self.out_exit_xy, 0.0,
                                       self.OUT_DOG_DEF, "outbound conveyor")
                    done = True
                    # A COMPLETED JOB THAT NOTHING COUNTED. This is the same
                    # move _collect_from_row makes -- a cart towed to the dock
                    # E infeed and sent out of the building -- and that one
                    # bumps jobs_total on its way out. This copy of it bumped
                    # nothing, so a tug resuming into a full row could take
                    # cart after cart out of the building with every published
                    # counter flat. NOT a delivery (no cart reached a
                    # station), so carts_delivered_total is untouched and
                    # jobs_total - carts_delivered_total stays the collection
                    # count.
                    self.jobs_total += 1
                self._park_at_staging()
            # ONLY a COMPLETED job is a clean boundary. Calling this on the
            # way out regardless (the first cut did) fired a deferred pause
            # mid-tow the first time an obstacle hold made the park leg time
            # out -- the tug stopped in the pick-cell column still holding a
            # cart, which is exactly the state a deferred stop exists to
            # avoid. Measured 2026-07-22.
            if done:
                self._task_boundary("finished the cart that was already in "
                                    "tow")
            return
        line = self._line_state()
        if not line:
            time.sleep(2.0)
            return
        self._try_dock_handoff()
        xy = self._all_trolley_xy()
        # WHICH CART IS OURS. Scan the master's in_transit list, NOT its
        # "loaded" field: loaded is cleared the moment the box leaves the fill
        # station, and the fill conveyor moving the cart out to the station is
        # exactly that departure -- so keying off "loaded" meant the cart went
        # invisible to us at the precise moment it became ours to collect.
        # in_transit entries survive until the load ships, which is the window
        # we actually care about.
        loaded = line.get("loaded") or {}
        candidates = []
        for e in (line.get("in_transit") or []):
            if e.get("trolley"):
                candidates.append((e["trolley"], e.get("box"), None))
        if loaded.get("trolley"):
            candidates.append((loaded["trolley"], loaded.get("box"),
                               loaded.get("parts")))
        # DO NOT TOUCH A CART THAT IS STILL ON THE CONVEYOR. The station's
        # 0.55 m tolerance is entered before the carry finishes, so without
        # this check tug_a would dock a cart mid-ride and truncate the carry
        # it is supposed to be waiting for. tug_b publishes what it is
        # carrying; believe it. (Docking cancels the carry safely either way
        # -- this is about the demo reading right, not about correctness.)
        peer = self._http_state(self.peer_port) or {}
        conveying = set((peer.get("idle_loop") or {}).get("conveying") or [])
        trolley = box = parts = None
        for t, bx, pc in candidates:
            if t in conveying:
                continue
            if self._at_spot(xy.get(t), self.pickup_xy):
                trolley, box, parts = t, bx, pc
                break
        # The loaded cart is ours to take only once the FILL CONVEYOR has run
        # it all the way out to the CART PICKUP. While it is still up at the
        # fill spot -- or in transit past the station -- it belongs to tug_b
        # and the arm. Collecting from the pickup rather than the station is
        # what takes this tug off the arm's critical path: the station is free
        # for the next empty long before we get here, so however long we are
        # gone, nothing upstream is waiting on us.
        busy = self._busy_trolleys(line)
        ready = trolley is not None
        # CONSTRAINT PRE-CHECK, before any wheel turns. "Don't enter the pick
        # cell" must make this tug DECLINE the loaded-cart collection and
        # fall through to the park-row work at the far end of the building --
        # not stop dead, and not retry a forbidden dock every pass.
        if ready and self._job_forbidden(
                self.pickup_xy, "collect the loaded cart from the pick cell"):
            ready = False
        spot = self._free_park_spot(xy)
        if ready:
            if spot is None:
                if not self._collect_from_row(busy):
                    time.sleep(2.0)
                else:
                    self._park_at_staging()
                    self._task_boundary("collection out through dock E")
                return
            n = self.cycles + 1
            t0 = time.time()
            # SAY WHEN THIS IS A RETRY. Every early return below abandons the
            # cycle without closing it, so the next pass legitimately
            # re-announces the same n -- and an unqualified repeat reads as a
            # tug that is making no progress. MEASURED, show2_idle.txt:
            # "dispatch #1" three times (t=77.2, t=106.6, t=247.0), each after
            # an operator command aborted the previous attempt, with a
            # completed park at t=201.7 in between. Naming the attempt makes
            # the difference between "retrying" and "stuck" readable.
            self._announce_attempt = (
                self._announce_attempt + 1 if self._announced_n == n else 1)
            self._announced_n = n
            retry = ("" if self._announce_attempt == 1 else
                     f" [attempt {self._announce_attempt}; the previous one "
                     f"was interrupted before it finished]")
            self._log(f"dispatch #{n}: {trolley} is loaded with box "
                      f"{box or '?'}"
                      + (f" ({parts} part(s))" if parts is not None else "")
                      + f" and waiting at the cart pickup — hauling to "
                        f"{spot[0]}{retry}")
            if not self._lane_acquire(priority=True):
                return
            self.leg = "docking"
            self.trolley_def = trolley
            if not self._dock():
                self._lane_release()
                return
            if not self._park_in_spot(trolley, spot[0], spot[1]):
                self._lane_release()
                return
            # COLLECT ON THE SAME TRIP — this is the line's takt, and the takt
            # is what the arm's dead air is made of.
            #
            # The cart ring is CONSERVED, so in steady state every park has to
            # be matched by a collection: this tug genuinely moves two carts
            # per box, and the arm cannot start a box until the second of them
            # has come all the way round. Doing them as two separate round
            # trips meant crossing the site FOUR times per box, and it
            # measured exactly that way on the baseline: park run 86 s, then a
            # SEPARATE collect run of 105 s, for a takt of 191 s per box
            # against a 26-40 s fill. Nearly all of the arm's idle time is
            # that arithmetic, not any handoff.
            #
            # But _park_in_spot leaves this tug standing in the back aisle a
            # few metres from the oldest cart in the row -- which is precisely
            # where a collection starts. Doing it here skips the whole
            # get-to-the-aisle half of the collect run (measured 34 s from
            # staging to the hitch) and one _park_at_staging, and _collect_
            # oldest already no-ops its approach when it finds itself in the
            # aisle, so this needs no new route.
            #
            # Gated on the reserve rather than on the row being FULL: a full
            # row means collecting with a loaded cart already waiting, which
            # is the state that produced the 181 s stalls in the baseline.
            collected = False
            if self._free_spot_count(self._all_trolley_xy()) <= self.PARK_RESERVE:
                collected = self._collect_from_row(busy)
            if not self._park_at_staging():
                self._lane_release()
                return
            self.cycles = n
            self._log(f"dispatch #{n} complete (pickup -> {spot[0]}"
                      + (" -> dock E" if collected else "")
                      + f" -> staging, {time.time() - t0:.1f}s)")
            # CLEAN TASK BOUNDARY: the cart is parked, this tug is back at
            # staging and empty-handed. The only safe place for a deferred
            # "stop after you place this cart" to take effect.
            self._task_boundary(f"dispatch #{n}")
            # NO cool-down here. This loop used to sleep --idle-period (10 s)
            # after every dispatch, which is fine for an ambient demo and
            # actively harmful for a production line: the line is CART-limited
            # (tug_a's round trip is ~100 s against a ~45 s fill), so those
            # 10 s land directly on the arm as "waiting for an empty trolley".
            # The natural poll interval already paces the loop.
            return
        # Nothing loaded to move -- this is the dead time the reserve policy
        # exists to use. Top the row back up to PARK_RESERVE free spots now,
        # while the arm is busy filling the next box, instead of doing it
        # later with a loaded cart already blocking the station.
        if self._free_spot_count(xy) <= self.PARK_RESERVE:
            if self._collect_from_row(busy):
                self._park_at_staging()
                self._task_boundary("collection out through dock E")
                return
        time.sleep(1.5)

    # ── mode 'trolley_return' (tug_b): keep the fill station fed ───

    def _cycle_return(self) -> None:
        b = self.bridge
        if self._pause_gate():
            time.sleep(0.4)
            return
        if b.carrying:
            self._log(f"resuming with {b.carrying} in tow — staging it")
            self._tow_to(b.carrying, self.stage_xy, "cart stage",
                         approach="east")
            self._hold_west()
            self._task_boundary("staged the cart that was already in tow")
            return
        line = self._line_state()
        if not line:
            time.sleep(2.0)
            return
        busy = self._busy_trolleys(line)
        xy = self._all_trolley_xy()

        # A. A LOADED cart is standing at the fill spot -> shuttle it OUT, all
        #    the way THROUGH the station to the CART PICKUP at the south end
        #    of the conveyor. Highest-priority move, and the one that removed
        #    the demo's dead air: the loaded cart used to stop AT the station
        #    and wait there for tug_a, which blocked the only route an empty
        #    cart has to the arm, which stalled the ARM for the whole of tug_a's
        #    dock-and-drag. Running it 2.55 m further costs ~10 s of chain
        #    time and takes tug_a off the arm's critical path entirely: the
        #    station is clear the moment the cart passes it, the next empty
        #    follows straight in behind, and tug_a collects from the pickup
        #    whenever it gets back.
        #
        #    BOTH stops must be clear, because the cart travels THROUGH the
        #    station to reach the pickup. The station being busy can only mean
        #    an empty is standing on it, and that is exactly the state the
        #    shuttle invariant (step C) forbids while a cart is at the fill
        #    spot -- so in practice this only ever waits on the PICKUP, i.e.
        #    on tug_a being genuinely behind. That is honest back-pressure,
        #    and it is strictly better than the old version, where the same
        #    back-pressure landed one stop earlier and blocked the empty too.
        loaded = line.get("loaded") or {}
        lt = loaded.get("trolley")
        if (lt and self._at_spot(xy.get(lt), self.fill_xy)
                and self._spot_free_xy(xy, self.station_xy)
                and self._spot_free_xy(xy, self.pickup_xy)):
            self.leg = "shuttle_out"
            if self._run_conveyor(lt, self.pickup_xy, self.fill_yaw,
                                  self.FILL_DOG_DEF, "fill conveyor (out)"):
                self._log(f"LOADED CART OUT: {lt} is down at the cart pickup "
                          "— tug_a can collect it, the station is already "
                          "free for the next empty")
            self.leg = "idle"
            self._task_boundary("loaded cart out to the pickup")
            return

        # B. Fill spot vacant + an empty waiting at the station -> shuttle IN.
        if self._spot_free_xy(xy, self.fill_xy):
            cand = self._find_at_xy(xy, self.station_xy, busy)
            if cand:
                n = self.cycles + 1
                self.leg = "shuttle_in"
                if self._run_conveyor(cand, self.fill_xy, self.fill_yaw,
                                      self.FILL_DOG_DEF,
                                      "fill conveyor (in)"):
                    self.cycles = n
                    # THE RETURN TUG'S DELIVERY, counted where it completes.
                    # This is the un-missable primitive for this role -- the
                    # cart is physically on the fill spot and the arm can use
                    # it -- and it is exactly the event this line has always
                    # narrated as "EMPTY CART DELIVERED" while no counter
                    # recorded it. MEASURED, show2_idle.txt: this fired at
                    # t=103.7 and t=214.5 and delivered_total stayed 0.
                    self.returns_total += 1
                    self._log(f"return #{n}: EMPTY CART DELIVERED — {cand} is "
                              "on the fill spot, the arm can resume filling")
                self.leg = "idle"
                # CLEAR THE COLUMN, and do it HERE rather than in step C.
                # Staging a cart on the station leaves this tug standing at
                # (station_x, station_y - 1.23), which is on the conveyor's
                # south leg -- exactly where the next LOADED cart will be run
                # out to the pickup. It has to move before then, but the two
                # possible moments are not equal: leaving in step C costs the
                # arm ~6 s, because the very next thing the line needs is this
                # conveyor run and nothing can start it while the tug is
                # driving away. Leaving HERE, after the empty is already on
                # its way in, spends the same seconds off the critical path.
                self._hold_west()
                self._task_boundary(f"return #{n}: empty delivered")
                return

        # C. Station free AND fill spot free -> move a staged cart onto the
        #    station.
        #
        #    THE SHUTTLE INVARIANT: at most ONE cart may be inside the
        #    {station, fill spot} pair at a time. The fill conveyor is a single
        #    lane and the station is both its ends, so an empty parked at the
        #    station while a cart is up at the fill spot is a hard deadlock --
        #    the loaded cart has nowhere to come out to, and the empty has
        #    nowhere to go in. (Measured: the demo wedged at t=53s the first
        #    time the world spawned a cart at each end.) Waiting on the fill
        #    spot being clear costs nothing: the OVERLAP that keeps the arm fed
        #    is step D pre-fetching to the stage, two metres away, not stacking
        #    carts in the shuttle.
        #
        #    The invariant now spans THREE stops rather than two, because the
        #    conveyor runs past the station to the pickup -- so the OUTFEED LEG
        #    has to be clear as well, not just the two ends. See
        #    _outfeed_leg_clear.
        if (self._spot_free_xy(xy, self.station_xy)
                and self._spot_free_xy(xy, self.fill_xy)
                and self._outfeed_leg_clear(xy)):
            cand = self._find_at_xy(xy, self.stage_xy, busy)
            if cand and not self._job_forbidden(
                    self.station_xy, f"move {cand} onto the station"):
                self.leg = "stage_to_station"
                # NO hold-west here. The instant this cart is down on the
                # station the line needs step B to run the conveyor, and the
                # arm is standing over a filled box waiting for exactly that.
                # Driving 4.5 m back to the hold first put those ~6 s straight
                # onto the arm's wait (measured on the baseline: cart down at
                # t=97.2, conveyor-in not started until t=102.7). Returning
                # bare lets the very next pass start the carry; step B then
                # does the hold-west once the empty is already moving.
                self._tow_to(cand, self.station_xy, "conveyor station",
                             approach="south")
                self._task_boundary("staged cart moved onto the station")
                return

        # D. Stage free + a cart at the lane pickup -> fetch it.
        #    THIS IS THE OVERLAP. It runs while tug_a is still out hauling, so
        #    when the station finally clears the next empty is two metres away
        #    instead of fifteen, and the arm's wait collapses.
        if self._spot_free_xy(xy, self.stage_xy):
            cand = self._find_at_xy(xy, self.lane_pickup_xy, busy)
            if cand and not self._job_forbidden(
                    self.stage_xy, f"fetch {cand} to the cart stage"):
                self.leg = "lane_fetch"
                if self._tow_to(cand, self.stage_xy, "cart stage",
                                approach="east"):
                    self._hold_west()
                self._task_boundary("empty cart fetched to the stage")
                return

        # E. Lane pickup free + a cart at the inbound door -> run the lane in.
        if self._spot_free_xy(xy, self.lane_pickup_xy):
            cand = self._find_at_xy(xy, self.lane_entry_xy, busy)
            if cand:
                self.leg = "lane_conveyor"
                if self._run_conveyor(cand, self.lane_pickup_xy, 0.0,
                                      self.LANE_DOG_DEF,
                                      "cart lane conveyor"):
                    self._log(f"empty cart {cand} rolled in through dock 2 "
                              "and is ready at the lane pickup")
                self.leg = "idle"
                self._task_boundary("empty cart rolled in through dock 2")
                return
        # Genuinely nothing to do: wait where the work is.
        self._hold_west()
        time.sleep(1.5)

    # ── cycle + thread body ───────────────────────────────────────

    def run(self) -> None:
        b = self.bridge
        try:
            while b.robot.getTime() < 3.0:
                time.sleep(0.25)
        except Exception as e:
            print(f"[idle-{self.mode}] disabled (startup failed: {e!r})")
            return
        if self.mode not in ("dispatch", "trolley_return"):
            print(f"[idle-{self.mode}] disabled: unknown mode "
                  f"{self.mode!r} (expected 'dispatch' or 'trolley_return')")
            return
        if not b.pallet_feature:
            print(f"[idle-{self.mode}] disabled: --idle-loop {self.mode} "
                  "needs --pallets + supervisor TRUE")
            return
        if len(b.pallet_defs) < 3:
            print(f"[idle-{self.mode}] disabled: the cart ring needs at least "
                  "THREE carts in --pallets (one filling, one staged, one "
                  "circulating)")
            return
        if not self.arm_port:
            print(f"[idle-{self.mode}] disabled: --idle-arm-port is the line "
                  "master; without it there is no line state to follow")
            return

        def marker(def_name):
            return self._call(lambda: (lambda n: list(n.getPosition())
                                       if n is not None else None)(
                                           b.robot.getFromDef(def_name)))

        # The FILL SPOT is the spawn pose of the first --pallets cart (the
        # world parks one there); everything else is a painted marker read
        # live, so moving a decal moves the choreography.
        self.trolley_def = b.pallet_defs[0]
        tp = self._trolley_pose()
        pose = self._pose()
        if tp is None or pose is None:
            print(f"[idle-{self.mode}] disabled: cart "
                  f"{self.trolley_def!r} or own pose not found")
            return
        self.fill_xy, self.fill_yaw = (tp[0], tp[1]), tp[2]
        x, y, yaw = pose
        self.staging_xy = (x, y)
        self.staging_heading = yaw
        self._trail = (float(b.cfg.get("rear_offset_m", b._REAR_OFFSET_M))
                       + math.hypot(*b._HITCH_LOCAL))

        need = {
            "station": self.STATION_DEF, "stage": self.STAGE_DEF,
            "lane_pickup": self.LANE_PICKUP_DEF,
            "lane_entry": self.LANE_ENTRY_DEF,
            "out_infeed": self.OUT_INFEED_DEF, "out_exit": self.OUT_EXIT_DEF,
        }
        got = {}
        for key, dn in need.items():
            p = marker(dn)
            if p is None:
                print(f"[idle-{self.mode}] disabled: DEF {dn} not found "
                      "(the closed cart ring needs every marker)")
                return
            got[key] = (p[0], p[1])
        self.station_xy = got["station"]
        self.stage_xy = got["stage"]
        # The loaded-cart pickup is OPTIONAL, so a world that predates it
        # still runs: without the marker the loaded cart simply stops at the
        # station the way it used to (and the arm waits for tug_a again).
        pk = marker(self.PICKUP_DEF)
        self.pickup_xy = (pk[0], pk[1]) if pk else self.station_xy
        self.has_pickup = pk is not None
        self.lane_pickup_xy = got["lane_pickup"]
        self.lane_entry_xy = got["lane_entry"]
        self.out_infeed_xy = got["out_infeed"]
        self.out_exit_xy = got["out_exit"]

        # PARK_SPOT_1.. are scanned until the first gap, so the world decides
        # how many spots there are just by defining them.
        self.park_spots = []
        for i in range(1, self.MAX_PARK_SPOTS + 1):
            dn = f"{self.PARK_SPOT_PREFIX}{i}"
            p = marker(dn)
            if p is None:
                break
            self.park_spots.append((dn, (p[0], p[1])))
        if self.mode == "dispatch" and not self.park_spots:
            print(f"[idle-{self.mode}] disabled: no DEF "
                  f"{self.PARK_SPOT_PREFIX}1 (nowhere to park a cart)")
            return

        # Seed park_order from the world: carts the world spawned already
        # parked are the OLDEST, in spot order, so the first collection takes
        # a genuinely long-standing cart rather than the one just dropped.
        if self.mode == "dispatch":
            xy0 = self._all_trolley_xy()
            for sd, sp in self.park_spots:
                who = self._find_at_xy(xy0, sp, set())
                if who and who not in self.park_order:
                    self.park_order.append(who)

        if self.mode == "dispatch":
            self._log(
                f"armed [dispatch]: pickup "
                f"({self.pickup_xy[0]:+.2f},{self.pickup_xy[1]:+.2f})"
                f"{'' if self.has_pickup else ' [= station, no CART_PICKUP]'}, "
                f"station "
                f"({self.station_xy[0]:+.2f},{self.station_xy[1]:+.2f}), "
                f"{len(self.park_spots)} park spot(s) "
                f"{self.park_spots[0][1][0]:+.2f}..{self.park_spots[-1][1][0]:+.2f}"
                f" @ y={self.park_spots[0][1][1]:+.2f}, "
                f"{len(self.park_order)} already parked "
                f"({','.join(self.park_order) or '-'}), dock E infeed "
                f"({self.out_infeed_xy[0]:+.2f},{self.out_infeed_xy[1]:+.2f}), "
                f"staging ({x:+.2f},{y:+.2f}), master :{self.arm_port}, peer "
                f":{self.peer_port or 0}, {len(b.pallet_defs)} carts in the "
                f"ring, resume after {self.resume_s:.0f}s quiet")
        else:
            self._log(
                f"armed [trolley_return]: fill spot "
                f"({self.fill_xy[0]:+.2f},{self.fill_xy[1]:+.2f}), station "
                f"({self.station_xy[0]:+.2f},{self.station_xy[1]:+.2f}), "
                f"pickup ({self.pickup_xy[0]:+.2f},{self.pickup_xy[1]:+.2f})"
                f"{'' if self.has_pickup else ' [= station, no CART_PICKUP]'}, "
                f"stage "
                f"({self.stage_xy[0]:+.2f},{self.stage_xy[1]:+.2f}), lane "
                f"pickup ({self.lane_pickup_xy[0]:+.2f},"
                f"{self.lane_pickup_xy[1]:+.2f}), dock 2 entry "
                f"({self.lane_entry_xy[0]:+.2f},{self.lane_entry_xy[1]:+.2f}), "
                f"staging ({x:+.2f},{y:+.2f}), master :{self.arm_port}, peer "
                f":{self.peer_port or 0}, {len(b.pallet_defs)} carts in the "
                f"ring, resume after {self.resume_s:.0f}s quiet")

        # STATIC obstacle footprints, DERIVED from the live scene graph (see
        # the DRIVE_OVER_Z block). Parked/standing carts are not in here --
        # they are read live, because the park row grows over time.
        self._static_boxes = self._derive_static_boxes()
        # Where this tug queues when the peer owns the pick-cell column. Both
        # points are OUTSIDE the COL_* rect and off the transit lane, so a
        # waiting tug is parked out of the way rather than nosed up to the
        # boundary of the thing it is waiting for.
        self.col_wait_xy = (self.COL_WAIT_DISPATCH if self.mode == "dispatch"
                            else self.WEST_HOLD)
        # OMNILINK_AVOID=0 disables the behavioural guard (the guarded drive
        # then degenerates to a plain drive) -- used only to A/B the baseline.
        self._avoid_ready = _os.environ.get("OMNILINK_AVOID", "1") != "0"
        self._log("avoidance %s: safety_radius=%.2f m (2x foot half-diag "
                  "%.3f + 0.15 margin), yield_radius=%.2f m, %d derived "
                  "static footprints (height rule: solid above %.2f m, "
                  "driveable at or below), oriented-footprint margin %.2f m, "
                  "parked carts live, peer :%d"
                  % ("armed" if self._avoid_ready else "OFF (OMNILINK_AVOID=0)",
                     self.SAFETY_RADIUS, self.FOOT_HDIAG, self.YIELD_RADIUS,
                     len(self._static_boxes), self.DRIVE_OVER_Z,
                     self.STATIC_MARGIN, self.peer_port or 0))
        self._log("column mutex %s: rect x[%.2f,%.2f] y[%.2f,%.2f], lease "
                  "%.0fs, queue point (%.2f,%.2f), priority "
                  "dispatch>trolley_return with a %.0fs anti-starvation "
                  "override"
                  % ("armed" if self.peer_port else "OFF (no peer port)",
                     self.COL_X0, self.COL_X1, self.COL_Y0, self.COL_Y1,
                     self.COL_LEASE_S, self.col_wait_xy[0],
                     self.col_wait_xy[1], self.COL_STARVE_S))

        step = {"dispatch": self._cycle_dispatch,
                "trolley_return": self._cycle_return}[self.mode]
        while True:
            try:
                step()
            except Exception as e:
                # Log-and-continue: a bad pass must never end the line.
                # Both shared-resource claims are dropped, or a raise inside a
                # claimed leg would wedge the peer until the lease timed out.
                self.on_lane = False
                self._col_release()
                self._log(f"cycle error (continuing): {e!r}")
                time.sleep(2.0)


# ── HTTP ─────────────────────────────────────────────────────────────

def _json_finite(obj: Any) -> Any:
    """Recursively replace non-finite floats (NaN/Inf) with None.

    Pose / velocity reads can be NaN before the first robot.step()
    completes (the HTTP server is up BEFORE the main loop starts), and
    json.dumps happily emits bare `NaN` -- which is NOT valid JSON, so
    clients (jq, JSON.parse, json.loads) reject the whole body and the
    endpoint LOOKS empty/broken."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _json_finite(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_finite(v) for v in obj]
    return obj


def _intent_route(bridge: "MobileBridge", body: dict) -> dict:
    """POST /intents -- the relay-free door onto the deferred-intent store.

    Same dispatch the LLM tools use, so a headless gate can prove the
    MECHANISM (does the pause really land at the boundary? does the
    constraint really gate the column?) without spending model tokens, and
    an integrator can drive it from any HTTP client."""
    st = bridge.intents
    if st is None:
        return {"accepted": False, "reason": "deferred intents unavailable"}
    action = str(body.get("action") or body.get("tool") or "").strip()
    words = str(body.get("words") or "")
    # Integration/test hook, deliberately NOT exposed to the model: shorten
    # the horizon after which an unsatisfied intent expires. The default
    # (omnisim_bridges.intents.DEFAULT_TTL_S) is what a chat turn gets.
    ttl = body.get("ttl_s")
    ttl = float(ttl) if ttl is not None else None
    if action in ("pause_after_current_task", "stop_after_current_task",
                  "stop_after_task"):
        return st.schedule("pause", "after_current_task",
                           until_told=bool(body.get("until_told", True)),
                           words=words, ttl_s=ttl)
    if action in ("hold_until_told", "hold"):
        # force=True over HTTP: an integrator calling this route IS the
        # operator, so there are no "operator's own words" to verify. The
        # words check exists to stop a MODEL turning a bare "stop" into an
        # indefinite hold.
        return st.hold_now(words=words, ttl_s=ttl,
                           force=bool(body.get("force", True)))
    if action == "pause_when":
        return st.schedule("pause", body.get("condition"),
                           count=body.get("count"), leg=body.get("leg"),
                           until_told=bool(body.get("until_told", True)),
                           words=words, ttl_s=ttl)
    if action == "notify_when":
        return st.schedule("notify", body.get("condition"),
                           count=body.get("count"), leg=body.get("leg"),
                           until_told=bool(body.get("pause_first", False)),
                           message=str(body.get("message") or ""), words=words,
                           ttl_s=ttl)
    if action == "set_constraint":
        return st.set_constraint(body.get("rule"), words=words, ttl_s=ttl)
    if action == "clear_constraint":
        return st.clear_constraint(body.get("id"))
    if action in ("cancel", "cancel_pending_intent"):
        return st.cancel(body.get("id"))
    if action in ("list", "list_pending_intents", ""):
        return st.listing()
    return {"accepted": False, "reason": f"unknown action {action!r}",
            "actions": ["pause_after_current_task", "hold_until_told",
                        "pause_when", "notify_when", "set_constraint",
                        "clear_constraint", "cancel_pending_intent", "list"]}


def make_handler(bridge: MobileBridge, router: IntentRouter, relay: Any = None):
    action_lock = threading.RLock()
    request_ids = RequestIdGuard()
    trusted_origins = allowed_origins()
    bridge_token = configured_token()

    class _H(BaseHTTPRequestHandler):
        # ── HTTP/1.1 KEEP-ALIVE ──────────────────────────────────────
        # BaseHTTPRequestHandler defaults to HTTP/1.0, which closes the
        # connection after EVERY response. That made each poll cost a fresh
        # TCP connection, and a socket then sits in TIME_WAIT for 120 s on
        # Windows against a 16,384-port ephemeral range. Measured 2026-08-17
        # BEFORE this change: a ~50 Hz ROS 2 bringup drove 17,487 sockets
        # into TIME_WAIT and connect() started failing with WinError 10048 --
        # which reads like a bind conflict but is port exhaustion.
        #
        # Keep-alive is safe here only because every response already carries
        # an accurate Content-Length (see _json) and do_OPTIONS answers 204
        # with no body. If you add a response path, it MUST set
        # Content-Length or the client will hang waiting for a body.
        protocol_version = "HTTP/1.1"
        # An idle persistent connection parks a thread in ThreadingHTTPServer,
        # so reap the ones nobody is using.
        timeout = 30

        def log_message(self, fmt, *args):
            return

        def _motion_json(self, result):
            """Answer a motion verb with the status its own result implies.

            The busy check now lives in MobileBridge._begin_motion so that
            every entry point is covered, which means the refusal comes back
            as a dict rather than being raised by the route. Preserve the
            409 the tool descriptions promise."""
            code = 200
            if isinstance(result, dict):
                code = int(result.get("http_status", 200))
            return self._json(code, result)

        def _json(self, code, obj):
            # allow_nan=False guarantees strictly valid JSON on the wire;
            # the sanitizer maps any NaN/Inf field to null instead.
            try:
                data = json.dumps(obj, default=str, allow_nan=False).encode("utf-8")
            except ValueError:
                data = json.dumps(_json_finite(obj), default=str,
                                  allow_nan=False).encode("utf-8")
            # ⚠ KEEP-ALIVE CORRECTNESS. Several rejections fire BEFORE the
            # request body is read -- _guard()'s token/origin/version checks
            # run ahead of _read_json, and read_json itself rejects a bad
            # Content-Type and an oversized body before touching rfile. The
            # unread body would then be parsed as the next request on a
            # persistent connection, desyncing the stream. Errors are rare, so
            # closing on any of them is the cheap and always-correct fix.
            if code >= 400:
                self.close_connection = True
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("X-OmniSim-Wire", WIRE_VERSION)
            self.send_header("X-OmniSim-Service", WIRE_SERVICE)
            origin = getattr(self, "_response_origin", None)
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self):
            return read_json(self, allow_empty=True)

        def _guard(self) -> None:
            check_protocol_version(self.headers)
            self._response_origin = checked_origin(self.headers, trusted_origins)
            check_authorization(self.headers, bridge_token)

        def do_OPTIONS(self):
            try:
                self._response_origin = checked_origin(self.headers, trusted_origins)
                self.send_response(204)
                if self._response_origin:
                    self.send_header("Access-Control-Allow-Origin", self._response_origin)
                    self.send_header("Vary", "Origin")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header(
                    "Access-Control-Allow-Headers", "Content-Type, Authorization, X-OmniSim-Token"
                )
                self.end_headers()
            except RequestError as exc:
                self._json(exc.status, error_envelope(exc.code, exc.message, exc.details))

        # Top-level guards: an uncaught exception in a route used to
        # propagate into socketserver, which logs it server-side and closes
        # the connection with ZERO bytes sent -- the client sees an empty
        # reply (curl error 52) with no clue why. Every route now answers
        # with a JSON body: data, or a clear {"error": ...}.
        def do_GET(self):
            try:
                self._guard()
                self._route_get()
            except RequestError as e:
                self._json(e.status, error_envelope(e.code, e.message, e.details))
            except Exception as e:
                self._error_response(e)

        def do_POST(self):
            try:
                self._guard()
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                body = self._read_json()
                request_id = validate_request_id(body.pop("id", None))
                # Idempotent reads never claim a request id: replaying one is
                # meaningless, and the duplicate-id guard exists to stop a
                # retried COMMAND from executing twice.
                if path not in ("/state", "/get_robot_state", "/list_robots",
                                "/capabilities", "/read_sensor", "/list_sensors"):
                    request_ids.claim(path, request_id)
                # /stop_robot is the escape hatch and must never queue behind a
                # running motion. Sensor reads are exempt for a different
                # reason: they mutate nothing, and a 20 Hz ROS 2 poll holding
                # the action lock would serialise every motion command behind
                # it.
                if path in ("/stop_robot", "/read_sensor", "/list_sensors"):
                    self._route_post(body)
                else:
                    with action_lock:
                        self._route_post(body)
            except RequestError as e:
                self._json(e.status, error_envelope(e.code, e.message, e.details))
            except Exception as e:
                self._error_response(e)

        def _error_response(self, e: Exception) -> None:
            import traceback
            print(f"[omnilink_mobile_bridge] HTTP {self.command} {self.path} "
                  f"failed: {e!r}\n{traceback.format_exc()}")
            try:
                self._json(500, error_envelope("internal_error", "The bridge could not complete the request."))
            except Exception:
                pass  # headers already sent / socket gone -- nothing to add

        def _route_get(self):
            if self.path in ("/", "/help"):
                return self._json(200, {
                    "ok": True,
                    "service": WIRE_SERVICE,
                    "robot_id": bridge.robot_id,
                    "discovery": {
                        "protocol": "GET /protocol",
                        "capabilities": "GET /capabilities",
                        "state": "GET /state",
                        "sensors": "GET /list_sensors",
                        "natural_language": "POST /prompt {\"text\": \"...\"}",
                        "drive": "POST /drive_forward {\"distance\": 1.0, \"wait\": true}",
                        "turn": "POST /turn {\"angle\": 1.5708, \"wait\": true}",
                        "stop": "POST /stop_robot {}",
                    },
                })
            if self.path == "/protocol":
                return self._json(200, {
                    "ok": True, "omnisim_wire": WIRE_VERSION,
                    "service": WIRE_SERVICE,
                    "service_versions": {WIRE_SERVICE: WIRE_VERSION},
                    "instance": {"name": "omnilink_mobile_bridge", "robot_id": bridge.robot_id},
                    "extensions": [],
                })
            if self.path in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if self.path == "/peer_state":
                # Read-only view of what this tug knows about the OTHER
                # robots, from the polling it already does for its locks.
                # First-class endpoint on purpose: peer awareness has to be
                # inspectable without an LLM in the loop.
                return self._json(200, bridge.peer_report())
            if self.path == "/intents":
                # Read-only view of the deferred-intent store. Deliberately a
                # first-class endpoint rather than relay-only: a pending
                # intent must be inspectable without an LLM in the loop.
                if bridge.intents is None:
                    return self._json(501, error_envelope(
                        "not_supported", "deferred intents unavailable"))
                return self._json(200, bridge.intents.listing())
            if self.path in ("/capabilities", "/list_robots"):
                return self._json(200, [{
                    "id": bridge.robot_id, "model": bridge.cfg["model"],
                    "capabilities": bridge.capabilities,
                }])
            if self.path == "/list_sensors":
                return self._json(200, bridge.list_sensors())
            if self.path == "/usage":
                if relay is None:
                    return self._json(200, {"enabled": False})
                return self._json(200, {
                    "enabled": True,
                    "relay": relay.relay_identity(),
                    "latest": relay.latest_usage(),
                })
            return self._json(404, error_envelope("not_found", "Endpoint not found."))

        def _route_post(self, body):
            p = self.path.rstrip("/")
            # Any COMMAND (not a pure read) pauses the opt-in idle loop --
            # EXCEPT the one whose entire job is to un-pause it. Without that
            # exemption "carry on" is unimplementable: the request that asks
            # for autonomy back would re-arm the pause on its way in, the
            # robot would stay parked, and the model would report success.
            # Snapshot taken BEFORE the arm below, so a /prompt turn that
            # turns out to be a pure question can roll the pause back.
            prev_pause_marker = (bridge.last_external_cmd,
                                 bridge.last_external_src)
            if p not in ("/state", "/get_robot_state", "/list_robots",
                         "/capabilities", "/resume_autonomy",
                         "/resume_idle_loop", "/intents",
                         # Pure reads: a sensor poll is not a command and must
                         # never pause the idle loop. A ROS 2 sensor node
                         # polling at 20 Hz would otherwise hold the robot
                         # permanently parked.
                         "/read_sensor", "/list_sensors"):
                bridge.note_external_command("http:" + p)
            if p == "/read_sensor":
                sensor = require_field(body, "sensor")
                if not isinstance(sensor, str) or not sensor.strip():
                    raise RequestError(400, "invalid_sensor",
                                       "'sensor' must be a non-empty string.")
                return self._json(200, bridge.read_sensor(sensor.strip()))
            if p == "/list_sensors":
                return self._json(200, bridge.list_sensors())
            if p == "/intents":
                # SCHEDULE, don't command: this route must never arm the
                # pause (see the allowlist above). "Stop after you park this
                # cart" that stops the cart now is not a deferral.
                return self._json(200, _intent_route(bridge, body))
            if p in ("/resume_autonomy", "/resume_idle_loop"):
                return self._json(200, bridge.act_resume_autonomy())
            if p in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if p in ("/list_robots", "/capabilities"):
                return self._json(200, [{
                    "id": bridge.robot_id, "model": bridge.cfg["model"],
                    "capabilities": bridge.capabilities,
                }])
            if p == "/stop_robot":
                return self._json(200, bridge.act_stop())
            if p == "/reset_to_home":
                return self._json(200, bridge.act_reset_to_home())
            if p == "/set_velocity":
                linear = finite_number(require_field(body, "linear"), "linear")
                angular = finite_number(require_field(body, "angular"), "angular")
                return self._json(200, bridge.act_set_velocity(
                    linear, angular))
            # BUSY REJECTS, it does not clobber (PROTOCOL.md 5.4.1).
            # `self.motion` is a single slot: a second command used to
            # overwrite the first silently, so `turn` then `drive` -- which a
            # model emits as two tool calls in ONE turn, dispatched back to
            # back with no delay -- aborted the turn milliseconds in and drove
            # on a barely-rotated heading, with BOTH calls answering
            # `accepted: true`.
            #
            # The check used to live HERE, on these three routes, and so
            # covered one of the four ways to start a motion: POST /tool (the
            # path an OmniLink agent's tool calls actually take), the offline
            # intent router and the idle loop all bypassed it. It now lives in
            # MobileBridge._begin_motion; _motion_json carries the same 409
            # out to the wire.
            if p == "/drive_forward":
                distance = finite_number(require_field(body, "distance"), "distance")
                speed = body.get("speed")
                if speed is not None:
                    speed = finite_number(speed, "speed")
                timeout_s = body.get("timeout_s")
                if timeout_s is not None:
                    timeout_s = finite_number(timeout_s, "timeout_s")
                    if timeout_s <= 0:
                        raise RequestError(400, "invalid_argument",
                                           "timeout_s must be greater than zero")
                return self._motion_json(bridge.act_drive_forward(
                    distance, speed, wait=bool(body.get("wait", False)),
                    timeout_s=timeout_s))
            if p == "/turn":
                angle = finite_number(require_field(body, "angle"), "angle")
                return self._motion_json(bridge.act_turn(
                    angle, wait=bool(body.get("wait", False))))
            if p in ("/drive_to", "/drive_to_waypoint"):
                tx = finite_number(require_field(body, "x"), "x")
                ty = finite_number(require_field(body, "y"), "y")
                # drive_to is always blocking, so `wait` defaults TRUE here.
                # An explicit wait=false is still refused, with the reason.
                return self._motion_json(bridge.act_drive_to(
                    tx, ty, wait=bool(body.get("wait", True))))
            if p in ("/attach_trolley", "/grab_pallet"):
                return self._json(200, bridge.act_attach_trolley(body.get("def")))
            if p in ("/detach_trolley", "/release_pallet"):
                return self._json(200, bridge.act_detach_trolley())
            if p == "/prompt":
                text = nonempty_string(require_field(body, "text"), "text")
                # Give the intent store the operator's VERBATIM words for the
                # turn: hold_until_told checks them so a model cannot turn a
                # bare "stop" into an indefinite hold by inventing an
                # "until" in its own tool arguments (measured).
                if bridge.intents is not None:
                    bridge.intents.set_turn_text(text)
                # ENTRY PATH 1 of 2 (HTTP). The /chat full-page browser UI
                # fetches this same route, so it is covered here too.
                _tx = _tx_begin(bridge, relay, text, "http")
                try:
                    if relay is not None:
                        # Low-water mark for the turn's journal slice, so the
                        # relay's own auto-reads (which emit no "tool" event)
                        # can be reported below. See _auto_reads_since.
                        _n0 = _tx_journal_seq(relay)
                        out = relay.dispatch_sync(text)
                        bridge.end_chat_turn(prev_pause_marker,
                                             out.get("actions"))
                        # AFTER end_chat_turn on purpose: the pause decision
                        # keeps seeing exactly the list it always saw.
                        _merge_auto_reads(relay, _n0, out)
                        _tx_end(_tx, reply=out.get("response", ""),
                                actions=out.get("actions"),
                                error=out.get("error") or "")
                        return self._json(200, out)
                    result = router.dispatch(text)
                finally:
                    if bridge.intents is not None:
                        bridge.intents.set_turn_text("")
                bridge.end_chat_turn(prev_pause_marker, result["tools"])
                _tx_end(_tx, reply=result["agent"],
                        router_tools=result["tools"])
                return self._json(200, {
                    "response": result["agent"],
                    "actions": [{"tool": t[0], "result": t[1], "summary": t[2]}
                                for t in result["tools"]],
                })
            if p == "/tool":
                # Platform-side tool callback. omnilink-agents.com web UI
                # POSTs {"tool": "<name>", ...args} after producing tool
                # calls on its side; we dispatch the same registered Tool
                # the relay loop dispatches and return its result.
                tool_name = nonempty_string(require_field(body, "tool"), "tool")
                body.pop("tool", None)
                # Pause on INTENT, not on route. /tool is a single path that
                # carries both reads and commands, so a path allowlist scores a
                # platform-side "get_robot_state" poll as an operator command and
                # freezes the line for 60 s -- which is exactly what a status
                # check from the web UI does. Judge it by the tool name instead.
                # (A read_only flag on the Tool spec would be cleaner still, but
                # Tool lives in the omnisim-bridges relay module; this bridge-side
                # allowlist is the same rule without reaching across that seam.)
                bridge.end_chat_turn(prev_pause_marker, [{"tool": tool_name}])
                if relay is None or tool_name not in getattr(relay, "tools", {}):
                    return self._json(503, {
                        "status": "err",
                        "tool": tool_name,
                        "error": "tool_not_registered",
                    })
                try:
                    result = relay.tools[tool_name].dispatch(body)
                    return self._json(200, {
                        "status": "ok",
                        "tool": tool_name,
                        "result": result,
                    })
                except Exception as e:
                    # The detail used to be swallowed whole: `e` was bound and
                    # never read, nothing was logged, and the caller got the
                    # bare string "tool_execution_failed". A tool that raised
                    # was indistinguishable from a tool that raised something
                    # else, from either end of the wire.
                    tb = traceback.format_exc()
                    print(f"[omnilink_mobile_bridge] tool {tool_name!r} raised:"
                          f"\n{tb}", flush=True)
                    return self._json(500, {
                        "status": "err",
                        "tool": tool_name,
                        "error": "tool_execution_failed",
                        "detail": f"{type(e).__name__}: {e}",
                    })
            return self._json(404, error_envelope("not_found", "Endpoint not found.", {"path": p}))
    return _H


def start_http(bridge: MobileBridge, router: IntentRouter, port: int, relay: Any = None) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(bridge, router, relay))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[omnilink_mobile_bridge] HTTP on http://127.0.0.1:{port}")
    return server


# ── OmniLink tool builders ───────────────────────────────────────────

def _honest_no_basis(intent_tools: List[Any]) -> List[Any]:
    """Make estimate_time_remaining's AUDIT LINE say what its payload says.

    MEASURED in the same transcript as the stop_robot defect: the tool
    reported status ok / summary "ok" on a robot that had finished nothing
    this session, so it had no sample to estimate from. The PAYLOAD was
    already honest (`known: false` plus a `say` that admits it) -- the
    one-line summary was not, because the relay's summariser renders a fixed
    set of keys (accepted / halted_at / q / tcp / xyz / yaw / mode / state /
    ...) and falls back to the literal word "ok" when a result contains none
    of them. A tool that answers "ok" with nothing is the quiet version of
    the same defect this file exists to fix, so put the verdict on the key
    the summariser DOES render rather than leaving the audit line to invent
    reassurance.
    """
    for t in intent_tools:
        if getattr(t, "name", "") != "estimate_time_remaining":
            continue
        inner = t.dispatch

        def _wrapped(args, _inner=inner):
            out = _inner(args)
            if not isinstance(out, dict):
                return out
            if out.get("known"):
                out["state"] = (
                    f"estimate from {out.get('task_samples')} measured "
                    f"{out.get('task_noun', 'task')}(s): "
                    f"{out.get('task_remaining_estimate_s')}s left")
            else:
                out["state"] = (
                    "NO BASIS YET -- this robot has not finished a "
                    f"{out.get('task_noun', 'task')} this session, so there "
                    "is nothing measured to estimate from. Say that; do not "
                    "produce a number.")
            return out

        t.dispatch = _wrapped
    return intent_tools


# WHAT THE PROGRESS CLOCKS ACTUALLY MEASURE -- stated in the payload that
# carries them, because a number with no stated meaning gets narrated as
# whatever the sentence needs.
#
# MEASURED on the sibling arm bridge, 2026-07-29 sweep (_scratch/foot_redesign/
# qa_transcript.jsonl, 12:24:10Z): asked what it was doing, it said "I'm about
# two seconds into this pick, and ... about another 42 seconds to finish."
# Both quantities are PRESENT in the payload it read -- progress.leg_elapsed_s
# and progress.task_remaining_estimate_s, the latter a mean over that session's
# own timed jobs -- and still read as invented, because nothing in the payload said
# what the clocks measure. They are elapsed times: leg_elapsed_s is time since
# the leg LABEL last changed, and the idle loop keeps feeding note_leg while
# the loop is PAUSED, so the clock runs on a frozen leg. Ask during an operator
# hold and the same sentence becomes false. The tugs share the IntentStore that
# produces this block, so they share the defect.
_PROGRESS_KEYS = ("leg", "leg_elapsed_s", "task_noun", "is_estimate")

# The READ tools that can carry a progress block. Named explicitly so no
# motion verb gains a wrapper.
_PROGRESS_TOOLS = ("get_robot_state", "estimate_time_remaining")


def _annotate_progress(p: Dict[str, Any], *, loop_paused: Optional[bool]) -> None:
    noun = str(p.get("task_noun") or "task")
    p["clock_meaning"] = (
        f"leg_elapsed_s is seconds since the leg LABEL last changed; "
        f"current_task_elapsed_s is seconds since this {noun} started. "
        f"Neither senses how far through the {noun} the robot is, and both "
        f"keep counting while the loop is paused.")
    p["not_measured"] = [
        f"how far through the current {noun} the robot is (no per-{noun} "
        f"progress sensor exists -- do not state a fraction or a phase from "
        f"these clocks)",
    ]
    if loop_paused is not None:
        p["loop_running"] = not loop_paused
        if loop_paused:
            p["leg_frozen"] = True
            p["not_measured"].append(
                f"anything about work in progress right now: the loop is "
                f"paused, so '{p.get('leg')}' is the leg it stopped on, not "
                f"what it is doing")


def _honest_progress(tools: List[Any]) -> List[Any]:
    """Label the progress clocks wherever a read hands them to the model.

    Wraps dispatch (same bridge-side pattern as _honest_no_basis above), so it
    covers get_robot_state's nested block and estimate_time_remaining, whose
    payload IS the progress dict. Additive keys only, and only on the TOOL
    surface -- HTTP /state is untouched, so the peer polling that drives the
    lane lock and the column mutex sees byte-identical data.
    """
    for t in tools:
        if getattr(t, "name", "") not in _PROGRESS_TOOLS:
            continue
        inner = getattr(t, "dispatch", None)
        if inner is None:
            continue

        def _wrapped(args, _inner=inner):
            out = _inner(args)
            if not isinstance(out, dict):
                return out
            try:
                loop = out.get("idle_loop")
                paused = (bool(loop.get("paused"))
                          if isinstance(loop, dict) and "paused" in loop
                          else None)
                if all(k in out for k in _PROGRESS_KEYS):
                    _annotate_progress(out, loop_paused=paused)
                prog = out.get("progress")
                if isinstance(prog, dict) and "leg" in prog:
                    _annotate_progress(prog, loop_paused=paused)
            except Exception:
                pass          # a label must never break a read
            return out

        t.dispatch = _wrapped
    return tools


# WHERE A CLAIM IS ALLOWED TO COME FROM -- stated in the payload the model is
# reading at the moment it decides how to attribute it.
#
# MEASURED on the sibling arm, same sweep, 12:24:49Z: "I have shipped 0 boxes
# so far this session. My records show that I haven't completed any full boxes
# yet." The 0 came from a live get_robot_state the relay's grounding gate had
# just dispatched; "my records" names get_action_history, which was never
# called. A right number under a wrong source is unauditable -- the operator
# cannot tell a reading from a recollection, which is the one distinction this
# surface exists to make. The tugs answer the same class of question
# ("how many carts have you parked") off the same tool.
def _state_reading_note(tools: List[Any]) -> List[Any]:
    for t in tools:
        if getattr(t, "name", "") != "get_robot_state":
            continue
        inner = getattr(t, "dispatch", None)
        if inner is None:
            continue

        def _wrapped(args, _inner=inner):
            out = _inner(args)
            if isinstance(out, dict):
                out["reading_note"] = (
                    "Live STATE read. Your own totals here are "
                    "carts_delivered_total (carts, counted the way YOUR job "
                    "delivers them -- carts_delivered_means says exactly "
                    "what that included) and jobs_total (legs, so it is "
                    "larger); the record of what YOU "
                    "dispatched is get_action_history -- say 'my records "
                    "show' only when quoting that one. If you already "
                    "answered this from a measured read, that answer was "
                    "grounded: correct it only where these numbers differ, "
                    "and do not apologise for a check you did make.")
            return out

        t.dispatch = _wrapped
    return tools


def build_mobile_tools(bridge: MobileBridge) -> List[Any]:
    if Tool is None:
        return []
    tools = _build_base_tools(bridge)
    # PEER AWARENESS. Registered whenever this robot actually watches another
    # one -- i.e. when a peer port was wired up. The data has been flowing
    # since the lane lock was built; until now it stopped at the avoidance
    # code and never reached the model, which then said "I don't have direct
    # access to the other tug's live state" and guessed.
    # NOTE the gate: build_mobile_tools runs from setup_omnilink_relay, which
    # main() calls BEFORE it constructs bridge.idle_loop -- so testing the
    # loop's peer_port here would always be 0. main() copies it onto the
    # bridge first for exactly this reason.
    if getattr(bridge, "idle_peer_port", 0):
        tools.append(Tool(
            name="get_peer_state",
            description=(
                "WHAT THE OTHER ROBOTS ARE DOING, live. Call this for 'what "
                "is the other tug doing?', 'where is the other robot?', 'is "
                "it waiting for you?', 'who has the lane?', 'how is the line "
                "going?' — anything about a robot that is not you. You DO "
                "have this: your own collision-avoidance and lock code polls "
                "the other tug's status several times a second, and this "
                "hands you that same reading. Returns the peer's id, role, "
                "current leg, what it is towing, whether it is paused or "
                "held, how many carts it has delivered, its position, and "
                "who holds the shared transit lane and the pick-cell column; "
                "plus the pick cell's published line counts. Every field is "
                "stamped with 'data_age_s' and a 'stale' flag — if stale is "
                "true, say the reading is a few seconds old. If 'known' is "
                "false the peer did not answer: say THAT, and do not infer "
                "what it is doing from anything else. Relay the 'say' "
                "sentence rather than speculating. "
                # Reading a peer is NOT reading yourself -- relay.py says so
                # in as many words, and enforces it: the grounding gate
                # rejects get_peer_state on purpose (its comment records two
                # tugs answering "everything looks normal" off a peer read
                # while both were themselves HELD). "How is the line going?"
                # is listed above AND matches the gate's own trigger regex,
                # so on that phrasing this tool alone always earns the
                # "you answered that without reading your state" preface.
                "This is a read of SOMEONE ELSE. It never counts as having "
                "checked yourself: call get_robot_state in the same turn "
                "whenever the answer also touches what YOU are doing."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.peer_report(),
        ))
    # DEFERRED-INTENT tools. Registered whenever the store exists, on every
    # mobile base -- a robot that cannot express "after", "until" or "don't
    # ... until" answers those orders in prose and forgets them.
    if bridge.intents is not None and build_intent_tools is not None:
        tools += _honest_no_basis(build_intent_tools(Tool, bridge.intents))
    # Optional dock-and-carry tools -- registered ONLY when the world opted
    # in with --pallets and the supervisor lookup succeeded, so demos
    # without the feature expose the exact same tool set as before.
    if getattr(bridge, "pallet_feature", False):
        tools += [
            Tool(
                name="attach_trolley",
                description=(
                    "Magnetically dock to a tug trolley's hitch bar and "
                    "start towing it like a trailer. Only succeeds when the "
                    "robot's REAR is within "
                    f"{bridge._DOCK_RADIUS_M:.1f} m of the hitch, so drive "
                    "close to the trolley and turn so your tail faces its "
                    "hitch bar first. While towing, the trolley (and any "
                    "parts in its basket) trails naturally behind the "
                    "robot through turns. Known trolleys: "
                    f"{', '.join(bridge.pallet_defs)}. Typical mission: "
                    "drive to the cart, back the tail up to its hitch, "
                    "attach_trolley, drive to the destination, "
                    "detach_trolley. Check get_robot_state first -- "
                    "idle_loop tells you which cart is yours to move."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "def": {"type": "string",
                                "description": ("Trolley DEF name. Omit to "
                                                "dock the default "
                                                f"({bridge.pallet_defs[0]}).")},
                    },
                },
                dispatch=lambda args: bridge.act_attach_trolley(args.get("def")),
            ),
            Tool(
                name="detach_trolley",
                description=(
                    "Release the towed cart: it stays parked on its casters "
                    "at its current trailed spot. Fails if nothing is "
                    "being towed."
                ),
                parameters={"type": "object", "properties": {}},
                dispatch=lambda args: bridge.act_detach_trolley(),
            ),
        ]
    # LAST, so both wrappers compose over whatever dispatch is current (the
    # estimate_time_remaining one above included). Reads only -- see
    # _PROGRESS_TOOLS; no motion verb is wrapped.
    return _state_reading_note(_honest_progress(tools))


def _build_base_tools(bridge: MobileBridge) -> List[Any]:
    return [
        Tool(
            name="drive_to",
            description=(
                "GO TO AN ABSOLUTE POSITION on the site. PREFER THIS over "
                "turn+drive_forward whenever you know where you want to end "
                "up — it does the heading maths for you and it is the only "
                "motion verb that reports where the robot actually stopped. "
                "x/y are world-frame metres, the same frame get_robot_state "
                "reports. BLOCKING: it returns only once the robot has "
                "settled, and gives you 'achieved_xy', 'error_m' and "
                "'arrived'. Report those numbers, not the ones you asked for. "
                # THE FENCE, IN NUMBERS, BEFORE THE CALL RATHER THAN AFTER IT.
                #
                # MEASURED 2026-07-29, QA probe d02_offsite (results/
                # 2026-07-29_robot_qa.json): "drive to x 200, y 0" -> the model
                # called drive_to, the precheck refused
                # ("refused: outside_site_bounds"), measured motion was
                # 0.0000 m / 0.0000 deg, and the reply correctly said it could
                # not go there AND quoted 14.4 / 8.4. It could only quote them
                # because the REFUSAL carried them -- the description said only
                # "refuses ... if the target is off-site" and named no bound,
                # and `capabilities.site_bounds_m` is HTTP-only, not a tool. So
                # the sole way to learn the fence was to spend a call being
                # refused. The three refusals the same run got right (the moon,
                # tug-asked-to-pick, arm-asked-to-drive) all needed no tool
                # because the limit was knowable from the description alone.
                # Publishing the numbers here is what makes this one knowable
                # too. AGENTS.md: "an agent cannot plan inside a bound it
                # cannot read."
                f"THE SITE FENCE IS |x| <= {bridge.SITE_HALF_X} m and "
                f"|y| <= {bridge.SITE_HALF_Y} m, world frame. Check the target "
                "against it BEFORE calling: an off-site target is refused "
                "('outside_site_bounds', nothing moves), so say plainly that "
                "it is off site instead of spending a call to be told."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "Target X in world-frame metres."},
                    "y": {"type": "number", "description": "Target Y in world-frame metres."},
                },
                "required": ["x", "y"],
            },
            dispatch=lambda args: bridge.act_drive_to(
                float(args.get("x", 0.0)), float(args.get("y", 0.0)), wait=True),
        ),
        Tool(
            name="drive_forward",
            description=(
                "Drive a distance along the CURRENT heading (body frame). "
                "Positive is forward, negative is reverse. If you know the "
                "destination rather than the distance, use drive_to instead. "
                "Set wait=true (recommended) and the call returns only once "
                "the robot has settled, with 'achieved' in metres and "
                "'error' = achieved - commanded; report those, not the "
                "distance you asked for. With wait=false it returns "
                "IMMEDIATELY, before the robot has moved, and 'achieved' is "
                "unknown until you read get_robot_state.last_command. "
                "'achieved' is the displacement measured ALONG THE HEADING "
                "THE DRIVE STARTED ON, so it goes negative if the robot ends "
                "up behind where it began, and it under-reads if the robot "
                "was pushed sideways. A second motion command while one is "
                "running is REJECTED with 409 busy — it is not queued and it "
                "does not interrupt. stop_robot and set_velocity are the "
                "exceptions: they cancel a running motion instead, and its "
                "achieved value then comes back null."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "distance": {"type": "number", "description": "Distance in metres. Positive=forward."},
                    "speed": {"type": "number", "description": "Optional speed override (m/s). Omit to use cruise."},
                    "wait": {"type": "boolean", "description": "Block until the move finishes and return the ACHIEVED distance. Recommended."},
                },
                "required": ["distance"],
            },
            dispatch=lambda args: bridge.act_drive_forward(
                float(args.get("distance", 0.0)), args.get("speed"),
                wait=bool(args.get("wait", True))),
        ),
        Tool(
            name="turn",
            description=(
                "Turn in place by a signed angle in RADIANS. Positive = "
                "counter-clockwise (left). Set wait=true (recommended) and "
                "the call returns only once the yaw has settled, with "
                "'achieved' in radians and 'error' = achieved - commanded. "
                "A skid-steer pivot is slow: a 90 degree turn takes roughly "
                "15-25 seconds of simulated time, so do not read a delay as "
                "a failure. A second motion command while one is running is "
                "REJECTED with 409 busy, so turn THEN drive as two waited "
                "calls — do not issue both at once. (stop_robot and "
                "set_velocity are the exceptions: they cancel a running "
                "turn, whose achieved angle then comes back null.)"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "angle_rad": {"type": "number", "description": "Turn angle in radians. Positive=left."},
                    "wait": {"type": "boolean", "description": "Block until the turn finishes and return the ACHIEVED angle. Recommended."},
                },
                "required": ["angle_rad"],
            },
            dispatch=lambda args: bridge.act_turn(
                float(args.get("angle_rad", 0.0)),
                wait=bool(args.get("wait", True))),
        ),
        Tool(
            name="set_velocity",
            description=(
                "Set a SAFETY-LIMITED teleoperation (linear, angular) "
                "velocity. The robot moves for AT MOST 12 simulated seconds "
                "and then automatically stops; another command may stop it "
                "sooner. Never promise continuous motion 'until told' from "
                "this call. If the operator requests indefinite motion, "
                "state this safety limit rather than pretending the command "
                "is durable. The result's expires_in_s is authoritative. "
                "UNLIKE drive_forward/turn/drive_to this verb does NOT "
                "return 409 when a motion is already running -- it is the "
                "teleop override, so it CANCELS that motion (whose achieved "
                "value then reports as null, because it was never measured). "
                "It reports no achieved distance or angle of its own: read "
                "get_robot_state for the pose."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "linear": {"type": "number", "description": "Forward velocity (m/s)."},
                    "angular": {"type": "number", "description": "Yaw rate (rad/s, +=left)."},
                },
                "required": ["linear", "angular"],
            },
            dispatch=lambda args: bridge.act_set_velocity(
                float(args.get("linear", 0.0)), float(args.get("angular", 0.0))),
        ),
        Tool(
            name="stop_robot",
            # Terse descriptions lose tool-selection races. With the
            # scheduling tools added, a bare "stop" started routing to
            # hold_until_told on a small local model because THIS tool said
            # almost nothing while that one said a lot. Own the word.
            description=(
                "STOP THE ROBOT NOW. This is the tool for a bare 'stop', "
                "'halt', 'stop now', 'freeze', 'emergency stop' — any order "
                "to cease motion immediately, with no 'after', 'once' or "
                "'until' in it. Zeroes both wheel velocities on the spot, "
                "then WATCHES THE POSE for a moment and tells you whether "
                "the robot actually came to rest. REPORT WHAT IT MEASURED, "
                "not the fact that the call succeeded: 'stationary': true "
                "means it is standing still (with 'achieved' m/s and "
                "'moved_since_halt_m' as the evidence); false means it is "
                "STILL MOVING and you must say so; null means rest could not "
                "be confirmed — say it was commanded to stop and that you "
                "cannot yet confirm it halted. Never say 'I have halted' off "
                "a null or a false. It also HOLDS the robot's own job loop "
                "for about a minute ('idle_loop.hold_s') and then lets it "
                "resume on its own — tell the operator that, and use "
                "resume_autonomy to hand it back sooner or hold_until_told "
                "for a stop with no auto-resume. Only reach for a scheduling "
                "tool when the operator named a LATER moment or a condition "
                "for resuming."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_stop(),
        ),
        Tool(
            name="reset_to_home",
            description="Teleport the robot back to its spawn pose (supervisor reset).",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_reset_to_home(),
        ),
        Tool(
            name="get_robot_state",
            # The description IS the model's map of this tool. When it said
            # only "pose and mode", models stopped reading the reply after the
            # pose and answered questions about the production line from
            # imagination -- while the answer was sitting in the same payload.
            # Advertise everything the call actually returns.
            description=(
                "Read this robot's FULL live state. Returns: pose (x, y, yaw), "
                "measured linear/angular velocity, motion mode, fault; "
                "'carrying' (the cart being towed, if any); and 'idle_loop' "
                "with the autonomy state -- mode (this robot's assigned job), "
                "leg (what it is doing RIGHT NOW), paused (true while an "
                "operator command holds it); "
                "'carts_delivered_total' (carts THIS robot delivered this "
                "session, counted the way THIS robot's job delivers them -- "
                "USE THIS to answer 'how many carts have you delivered/done', "
                "whichever tug you are, and read 'carts_delivered_means' in "
                "the same payload for the one-sentence definition of what it "
                "did and did not count); its two components "
                "'carts_parked_total' (loaded carts parked in the dispatch "
                "row -- the dispatch tug's work, and 0 on the return tug "
                "because that tug does not park carts) and "
                "'carts_returned_total' (empty carts delivered onto the fill "
                "spot for the arm -- the return tug's work, and 0 on the "
                "dispatch tug); 'delivered_total' (the OLD name for "
                "carts_parked_total and identical to it -- parks ONLY, so on "
                "the return tug it reads 0 forever and is NOT the answer to "
                "'how many carts have you delivered'; it IS the right field "
                "for 'how many of the carts in that row did you park'); and "
                "'jobs_total' (every completed job by THIS robot -- "
                "deliveries plus collections plus, on the return tug, the "
                "staging and lane moves, so it counts LEGS and is larger than "
                "the number of carts); cycles (full uninterrupted "
                "pickup-to-return ring closures -- any operator command "
                "aborts the cycle in progress, so this UNDERCOUNTS finished "
                "work and must NOT be reported as your total accomplishment); "
                "conveying (carts currently on a conveyor). "
                "CAREFUL -- 'parked' and 'delivered' are NOT your work: they "
                "are the park row's CURRENT OCCUPANCY (which carts are "
                "sitting there, and how many), a fact about the world that "
                "includes carts already parked before you started and drops "
                "when any cart is collected out. Never report them as things "
                "you did; if carts_parked_total is 0 then you have parked "
                "nothing, however full the row looks -- but on the RETURN tug "
                "that is its normal reading and says nothing about how much "
                "work it has done, so answer 'how many carts have you "
                "delivered' from carts_delivered_total, never from a parks "
                "field. It ALSO returns the "
                "robot's STANDING ORDERS, which are facts, not guesses: "
                "'pending_intents' (deferred things it is waiting to do — "
                "each with the trigger, what it will do, the operator's own "
                "words and when it expires), 'constraints' (standing "
                "restrictions it is enforcing right now and how many times "
                "they have already made it decline work), 'autonomy_hold' "
                "(whether it is stopped and refusing to auto-resume until "
                "told), 'notifications' (things that fired and have not been "
                "reported yet) and 'intents_summary' (all of that as one "
                "sentence). READ THOSE BACK rather than recalling what you "
                "said earlier in the chat — the store is the truth and your "
                "memory of the turn is not. ALWAYS call this before "
                "answering any question about what the robot is doing, what it "
                "is carrying, where a cart is, whether it is running, or what "
                "it is waiting for. This call is about THIS robot only: for "
                "the OTHER tug, the shared lane/column locks, or the pick "
                "cell's line counts, call get_peer_state — you do have that "
                "data. For 'how much longer', call estimate_time_remaining. "
                # ROUTE *THROUGH* THIS CALL, NEVER PAST IT -- see the mismatch
                # table in the commit message. relay._reground_if_unread
                # accepts ONLY get_robot_state as proof that a "how many / how
                # much / how far / how is the line going" question was
                # grounded (get_peer_state is explicitly rejected there, by
                # name, and estimate_time_remaining is not in its accepted
                # set either). When the model follows the two pointers above
                # INSTEAD of reading here, the gate re-reads state itself and
                # prefaces the retry with "SYSTEM: you answered that without
                # reading your state" -- and the model then apologises to the
                # operator for a check it did make (MEASURED on the sibling
                # arm, probe a02, 2026-07-29: "My apologies for the confusion
                # earlier. I just checked my live state..." with no earlier
                # confusion anywhere in the turn). Fixing the accepted set
                # needs relay.py, which is a shared package; making the
                # pointers ADDITIVE is the bridge-side half.
                "Both of those are EXTRA reads, not substitutes: call this "
                "one in the same turn as well, always."
            ),
            parameters={"type": "object", "properties": {}},
            # get_state_for_query, not get_state: the chat turn asking the
            # question already armed the idle-loop pause, so a raw read
            # would tell the model the robot is paused BY the question.
            dispatch=lambda args: bridge.get_state_for_query(),
        ),
        Tool(
            name="resume_autonomy",
            description=(
                "Hand the robot back to its own job loop immediately. Call "
                "this whenever the operator says to carry on, resume, go back "
                "to work, or that they are done. It is the ONLY way to resume "
                "early -- simply replying 'resuming now' leaves the robot "
                "parked, because your own turn is what is holding the pause. "
                "Otherwise autonomy returns by itself after about a minute of "
                "quiet."
            ),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_resume_autonomy(),
        ),
    ]


# Role briefings for the warehouse tugs. A tug that does not know its own
# job answers "what are you doing?" with a plausible invention -- and the
# site Foreman agent delegates exactly those questions to it, so a generic
# mobile-base prompt leaves the delegation contract unbacked. These strings
# are the grounding that makes the delegation honest.
# Short human labels for the same roles, used when this tug describes the
# OTHER one (get_peer_state). A peer's mode string on its own ("dispatch")
# is not an answer an operator can use.
_ROLE_LABEL = {
    "dispatch": "the dispatch tug — loaded carts out to the cart park",
    "trolley_return": "the empty-cart return tug — keeps the pick cell supplied",
}

_ROLE_BRIEF = {
    "dispatch": (
        "YOUR ROLE ON THIS SITE: you are the DISPATCH tug. The pick cell "
        "fills a box into a cart; the in-floor fill conveyor sets that loaded "
        "cart down at the CONVEYOR STATION just south of the pick-cell gate. "
        "Your job is to dock it there, tow it east along the transit lane, "
        "and park it in a FREE spot in the CART PARK -- the single row of six "
        "painted spots across the back of the building. You choose the spot "
        "yourself by reading where every cart currently is; carts accumulate "
        "there, one per spot. When the row is FULL your next job is a "
        "COLLECTION instead: you dock the longest-parked cart and tow it to "
        "the dock E infeed, where the outbound conveyor takes it out of the "
        "building. You do NOT work the west end of the site -- the cart lane, "
        "the cart stage and the fill conveyor belong to the other tug."),
    "trolley_return": (
        "YOUR ROLE ON THIS SITE: you are the EMPTY-CART RETURN tug. You keep "
        "the pick cell supplied. Empty carts roll in through dock 2 on the "
        "cart-lane conveyor and wait at the CART LANE PICKUP; you tow one to "
        "the CART STAGE while the other tug is still out hauling, then move "
        "it onto the CONVEYOR STATION as soon as that clears. You also run "
        "the in-floor FILL CONVEYOR in both directions: it carries an empty "
        "cart north to the fill spot in front of the arm, and carries the "
        "loaded cart back out to the station for the other tug to collect. "
        "You do NOT work the back of the building -- the cart park and dock E "
        "belong to the other tug."),
}


def build_mobile_main_task(bridge: MobileBridge) -> str:
    role = getattr(bridge, "idle_mode", None)
    brief = _ROLE_BRIEF.get(role or "", "")
    tow = ""
    if getattr(bridge, "pallet_feature", False):
        tow = (
            "\n- You tow CARTS, you do not lift them. Back your REAR to within "
            f"{bridge._DOCK_RADIUS_M:.1f} m of a cart's hitch, call "
            "attach_trolley, drive to the destination, then detach_trolley. "
            "get_robot_state reports 'carrying' with the cart you hold.")
    auto = ""
    if role:
        auto = (
            "\n\nAUTONOMY: you normally run this job on a loop by yourself. "
            "ANY operator command pauses that loop instantly and it resumes "
            "after about a minute of quiet, or immediately if the operator "
            "asks you to carry on -- in which case call resume_autonomy. "
            "NEVER claim you have resumed without calling it. get_robot_state "
            "reports idle_loop.paused, idle_loop.leg (what you are doing right "
            "now) and idle_loop.carts_delivered_total (how many carts you "
            "have delivered this session, counted the way YOUR job delivers "
            "them; idle_loop.carts_delivered_means spells out what that "
            # DRIFT FIXED: this line used to point at `idle_loop.parked` and
            # call it "the carts you have parked". It is the park row's
            # CURRENT OCCUPANCY -- it includes carts placed at world init and
            # drops when any cart is collected out -- and get_state's own
            # comment warns against exactly this reading. The prompt was
            # teaching the model the conflation the state dict is written to
            # prevent. `parked`/`park_row_occupants` is a fact about the
            # world; it is never a tally of this robot's labour.
            "covered) -- READ IT before answering any question "
            "about what you are doing or where a cart is. idle_loop.parked / "
            "park_row_occupants is the park row's CURRENT OCCUPANCY, not your "
            "work: it counts carts that were there before you started."
            # MEASURED: "what is the other tug doing right now?" got "I don't
            # have direct access to the other tug's live state", followed by
            # a guess from an indirect clue. Both halves were avoidable: the
            # data is polled continuously for the lane lock and the column
            # mutex, and a guess is never the honest fallback.
            "\n\nYOU ARE NOT ALONE, AND YOU ARE NOT BLIND TO THE OTHERS. You "
            "share this site with another tug and with the pick cell, and "
            "your own code polls both of them several times a second. "
            "get_peer_state hands you that reading: what the other tug is "
            "doing right now, what it is towing, whether it is paused, who "
            "holds the shared transit lane and the pick-cell column, and the "
            "pick cell's line counts. So: for ANY question about another "
            "robot, CALL get_peer_state and answer from it. Saying 'I don't "
            "have access to the other tug's state' is false, and inferring "
            "what it is doing from something else is worse -- if the tool "
            "reports known=false or stale, say exactly that instead.")
    # THE DEFERRED-ORDER RULE. Appended for every robot that has the tools,
    # idle loop or not: the measured failure was a model that had no way to
    # schedule and so promised in prose. Giving it the tools without telling
    # it that prose is a LIE here reproduces the same bug.
    if getattr(bridge, "intents", None) is not None:
        auto += INTENT_TASK_RULE
        # NARROWING, appended here rather than edited into the shared
        # MAIN_TASK_RULE so no other consumer's deferred-intent handling
        # changes. That rule lists 'then' among the words that MUST route to
        # a scheduling tool, which is right for "park it, then wait for my
        # signal" and wrong for "back up half a metre, then turn left 45
        # degrees" -- there the 'then' is only ordering two things the robot
        # can do right now, and sending it to a scheduling tool defers a
        # motion that should simply be the second call. The deferred case is
        # restated verbatim so the exception cannot be read as a licence to
        # skip scheduling.
        auto += (
            "\n- 'A then B' where BOTH are things you can do right now is "
            "not a deferred order -- it is those two calls, in that order, "
            "now. The scheduling tools are for an order naming a LATER "
            "moment or an outside condition ('once the cart arrives', "
            "'after this delivery', 'until I say so'); those still MUST be "
            "scheduled, never answered in prose.\n")
    return (
        f"You drive a {bridge.cfg['model']} mobile base in OmniSim through the "
        f"OmniLink-OmniSim bridge. Max speed ~{bridge.v_max_linear:.2f} m/s, "
        f"max yaw rate ~{bridge.v_max_angular:.2f} rad/s.\n\n"
        + (brief + "\n\n" if brief else "")
        + "Rules:\n"
        # THE ONE-CALL RULE IS GONE, AND IT WAS COSTING MOTIONS. It shipped
        # with c938ce12 (2026-05-14), when motion verbs returned instantly
        # and a second command silently clobbered the first -- back then
        # "one call" was the only safe advice there was. Everything that
        # made ordering safe landed 2026-07-26: motion tools now BLOCK until
        # settled (a2a8da5d), a second motion arriving mid-flight is
        # REJECTED with 409 instead of overwriting, and every reply carries
        # {commanded, achieved, error, settled} measured against pose
        # (52f3f6ca). Nobody revisited the rule. MEASURED consequence: asked
        # to "back up half a metre, then turn left 45 degrees" a live model
        # issued drive_forward (commanded -0.5, achieved -0.501, settled
        # true) and then answered "I have backed up 0.5 meters. Now I will
        # turn left 45 degrees." The turn was never issued. Across all ten
        # probes of that run every motion-bearing turn had exactly ONE
        # action, and the only two-action turn was one implying no motion --
        # the model chains calls fine, and stopped precisely where this
        # rule told it to.
        "- EVERY MOTION THE OPERATOR ASKS FOR GETS ITS OWN CALL, IN ORDER. "
        "Two moves in one sentence are two calls, not one call plus a "
        "sentence about the other. drive_forward / turn / drive_to BLOCK "
        "until the robot has settled, so issue the next one as soon as the "
        "previous returns -- 'settled': true in the reply means it has "
        "stopped moving and the way is clear for the next command.\n"
        "- NARRATING A MOTION IS NOT PERFORMING IT. 'Now I will turn left' "
        "with no tool call behind it is a false report: your turn ends, "
        "nothing is queued, and the robot never turns. Either call the tool "
        "in this same turn or say plainly that you are not going to.\n"
        "- For 'forward N m' / 'back N m' use drive_forward(distance=N or -N).\n"
        "- For 'turn left/right N degrees' use turn(angle_rad=±N*pi/180).\n"
        "- For 'spin' or 'circle' use set_velocity, but tell the operator it "
        f"auto-stops after at most {bridge.VELOCITY_MAX_S:.0f} simulated "
        "seconds. Never claim it will continue until told.\n"
        "- 'stop'/'halt' -> stop_robot. 'reset' -> reset_to_home.\n"
        # MEASURED 2026-07-29, QA probes e01/e02. e01 ("move the cart", eight
        # trolleys present) PASSED on this bridge -- the model asked "did you
        # mean TROLLEY_C, which is currently at the conveyor station, or a
        # different cart?" -- but nothing in the prompt asked for that, so it
        # was the model's own judgement and not a property of the surface. The
        # sibling arm got the same class of prompt ("put it over there") and
        # produced a question that named NOTHING ("could you point to a spot
        # or give me a location?"), which is why the behaviour is written down
        # here rather than left to luck. NAMING the candidates is the load-
        # bearing half: a question that lists nothing is barely more useful
        # than a guess, and the operator cannot answer it in one word.
        # The exclusion is deliberate and is the more dangerous half to get
        # wrong: an operator who says 'stop' must never be asked 'which stop?'.
        "- IF THE REQUEST NAMES NO OBJECT AND MORE THAN ONE FITS, ASK WHICH "
        "-- and NAME the candidates you can actually see, so the operator can "
        "answer in one word ('did you mean TROLLEY_C at the conveyor, or the "
        "one in the park row?'). Read state first if you need the names. Do "
        "not silently pick one and drive. This applies ONLY when the referent "
        "is genuinely missing: 'stop', 'halt', 'carry on', 'reset', and any "
        "request that names or uniquely implies its target, are NOT ambiguous "
        "-- obey those immediately and never answer them with a question.\n"
        "- You are INSIDE a building. Never issue an open-ended set_velocity "
        "to 'go somewhere' -- use drive_forward with a bounded distance. The "
        "bridge clamps commands that would leave the site, and a clamped "
        "command is a bug in your plan, not a success.\n"
        # THE NUMBERS, not just "the site". MEASURED 2026-07-29, QA probe
        # d02_offsite: the fence was only discoverable by being refused by it
        # (see drive_to's description). Same fix, second surface: a rule that
        # says "would leave the site" without saying where the site ends
        # cannot be applied before the call.
        f"- THE SITE IS |x| <= {bridge.SITE_HALF_X} m by "
        f"|y| <= {bridge.SITE_HALF_Y} m in the world frame get_robot_state "
        "reports. A target outside that is not a long drive, it is off site: "
        "say so and do not call a motion verb to find out.\n"
        "- hold_until_told is a STATIONARY autonomy hold. Never combine it "
        "with set_velocity in the same turn: the first asks the robot to "
        "remain stopped while the second asks it to move.\n"
        # THE THREE CLAUSES BELOW ARE MEASURED ON THE SIBLING ARM, 2026-07-29
        # (_scratch/foot_redesign/qa_transcript.jsonl). They are here because
        # the tugs run the identical relay, the identical grounding gate and
        # the identical IntentStore progress block, so all three failure modes
        # are reachable from this prompt too.
        #  - 12:24:49Z "My records show ..." over a live get_robot_state the
        #    gate had dispatched; get_action_history was never called.
        #  - 12:24:10Z "about another 42 seconds" -- a REAL measured-mean
        #    estimate that read as invented because its basis went unsaid.
        #  - 12:24:29Z "I apologize for the oversight in my previous reply" --
        #    about a reply that was correct, triggered by the gate's
        #    regrounding preface on a turn that HAD read the right surface.
        "- CITE THE SOURCE YOU ACTUALLY READ. 'My records show' and 'my logs' "
        "belong to get_action_history alone. A number from get_robot_state or "
        "get_peer_state is a live reading -- say you just read it.\n"
        "- A TIME ESTIMATE COMES WITH ITS BASIS, in the same sentence ('about "
        "40 seconds, going by the two runs I have timed'). The clocks in "
        "'progress' are elapsed times, not a measured fraction of the job, "
        "and they keep running while the loop is paused -- read "
        "clock_meaning / not_measured in the payload before quoting them.\n"
        "- DO NOT APOLOGISE FOR A MISTAKE YOU HAVE NOT VERIFIED. Before 'I "
        "apologize for the oversight', check get_action_history; if the "
        "record shows no error, there was none -- just answer.\n"
        "- Keep the final text response short -- one sentence."
        + tow + auto
    )


def setup_omnilink_relay(bridge: MobileBridge, http_port: int = 8765) -> Optional[Any]:
    if OmniLinkRelay is None:
        return None
    local_ok = (
        OllamaRelay is not None
        and _os.environ.get("OMNISIM_OLLAMA", "1").strip() not in ("0", "false", "no")
        and ollama_available()
    )
    # Explicitly choosing a cloud engine (OMNILINK_ENGINE=g4-engine etc.)
    # means the user wants cloud inference; otherwise local wins when
    # available because it's free and faster.
    explicit_cloud = bool(_os.environ.get("OMNILINK_ENGINE", "").strip())

    # ── Hybrid: OMNI_KEY + local Ollama → free local inference, platform
    # memory/profile/telemetry/fallback on top. The best of both.
    if omnilink_enabled() and local_ok and not explicit_cloud:
        try:
            agent_name = profile_sync.agent_name_for(bridge.robot_id)
            tools = build_mobile_tools(bridge)
            main_task = build_mobile_main_task(bridge)
            relay = OllamaRelay(
                agent_name=agent_name,
                main_task=main_task,
                tools=tools,
                omni_key=get_omni_key(),
            )
            # Dashboard presence: push the profile as g5-engine so the web
            # UI can drive this robot too (through the platform's g5 path /
            # the user's edge connector when one is running).
            if relay._client is not None:
                # profile_sync is imported at MODULE scope (see top of file).
                # Importing it here again would rebind it as a function-local for
                # the WHOLE function, making the earlier agent_name_for() call an
                # unbound local -- which is exactly the regression this replaces.
                if profile_sync.is_enabled():
                    profile_sync.ensure_profile(
                        client=relay._client,
                        agent_name=agent_name,
                        main_task=main_task,
                        tool_defs=relay.tool_defs,
                        engine="g5-engine",
                        tool_callback_url=f"http://127.0.0.1:{http_port}/tool",
                    )
                    relay.set_presence_endpoint(
                        f"http://127.0.0.1:{http_port}/tool",
                        robot=str(bridge.robot_id),
                    )
            print(f"[omnilink_mobile_bridge] HYBRID relay ON (local {relay.model} + OmniLink sync)")
            return relay
        except Exception as e:
            print(f"[omnilink_mobile_bridge] hybrid relay setup failed: {e}")
            # fall through to the plain cloud path below

    # ── Zero-account free tier: local Ollama only.
    if not omnilink_enabled():
        if local_ok:
            try:
                relay = OllamaRelay(
                    agent_name=profile_sync.agent_name_for(bridge.robot_id),
                    main_task=build_mobile_main_task(bridge),
                    tools=build_mobile_tools(bridge),
                )
                print(f"[omnilink_mobile_bridge] local Ollama relay ON (model={relay.model})")
                return relay
            except Exception as e:
                print(f"[omnilink_mobile_bridge] Ollama relay setup failed: {e}")
        # Bottom of the ladder. This used to return silently, so the first
        # clue was the chat panel reading "local intent (regex)" and taking
        # everything literally -- which reads as a broken agent rather than
        # as no agent. Name the state, and name the one command that fixes
        # it; the failure path below is loud for the same reason.
        print("[omnilink_mobile_bridge] no OMNI_KEY and no local Ollama -- "
              "driving on the offline regex intent router. Chat works, but "
              "it only understands set phrases: there is NO LLM in the loop.",
              flush=True)
        print("[omnilink_mobile_bridge]   upgrade with a free key: "
              "python -m omnisim key", flush=True)
        return None
    try:
        agent_name = profile_sync.agent_name_for(bridge.robot_id)
        tools = build_mobile_tools(bridge)
        main_task = build_mobile_main_task(bridge)
        relay = OmniLinkRelay(
            omni_key=get_omni_key(),
            agent_name=agent_name,
            main_task=main_task,
            tools=tools,
        )
        # Push a per-robot profile so operators can pick this base in
        # the omnilink-agents.com web UI and chat to the sim from
        # there. Tool calls round-trip back via /tool below.
        # profile_sync is imported at MODULE scope (see top of file).
        # Importing it here again would rebind it as a function-local for
        # the WHOLE function, making the earlier agent_name_for() call an
        # unbound local -- which is exactly the regression this replaces.
        if profile_sync.is_enabled():
            profile_sync.ensure_profile(
                client=relay._client,
                agent_name=agent_name,
                main_task=main_task,
                tool_defs=relay.tool_defs,
                engine=relay.engine,
                tool_callback_url=f"http://127.0.0.1:{http_port}/tool",
            )
            relay.set_presence_endpoint(
                f"http://127.0.0.1:{http_port}/tool",
                robot=str(bridge.robot_id),
            )
        print(f"[omnilink_mobile_bridge] OmniLink relay ON (agent='{agent_name}')")
        return relay
    except Exception as e:
        # LOUD, with a traceback. A silent downgrade here is how both tugs
        # once ran a whole demo on the regex router with nobody noticing --
        # the symptom (chat "works", just stupidly) looks nothing like the
        # cause (one AttributeError while building the main task). Still
        # non-fatal: the bridge must come up either way.
        import traceback
        print(f"[omnilink_mobile_bridge] !! OmniLink relay setup FAILED "
              f"({type(e).__name__}: {e}) -- falling back to the local regex "
              f"intent router. Chat will work but there is NO LLM in the loop.",
              flush=True)
        traceback.print_exc()
        return None


# ── wwi plumbing ─────────────────────────────────────────────────────

def push_configure(bridge: MobileBridge, relay: Any) -> None:
    if relay is None:
        agent_label = "local intent (regex)"
    else:
        engine = getattr(relay, "engine", None) or _os.environ.get("OMNILINK_ENGINE", "g1-engine")
        agent_label = (
            f"local Ollama ({engine.split(':', 1)[1]})" if str(engine).startswith("ollama:")
            else f"OmniLink relay ({engine})"
        )
    cfg = {
        "robot": bridge.cfg["model"],
        "robot_class": "mobile base",
        "agent": agent_label,
        "suggestions": [
            "forward 1 m",
            "turn left 90 degrees",
            "back 50 cm",
            "spin",
            "stop",
        ],
    }
    bridge.queue_window("configure:" + json.dumps(cfg))
    bridge.queue_window("status:connected")
    bridge.window_configured = True


def _on_relay_event(bridge: MobileBridge, kind: str, payload: Dict[str, Any]) -> None:
    if kind == "status":
        bridge.queue_window(f"status:{payload.get('state', 'idle')}")
    elif kind == "tool":
        bridge.queue_window(
            f"tool:{payload.get('name', '?')}:{payload.get('status', 'ok')}:{payload.get('summary', '')}")
    elif kind == "agent":
        bridge.queue_window("agent:" + str(payload.get("text", "")))
    elif kind == "usage":
        bridge.queue_window("usage:" + json.dumps(payload, default=str))
    elif kind == "audio_out":
        bridge.queue_window("audio_out:" + json.dumps(payload, default=str))
    elif kind == "error":
        bridge.queue_window("error:" + str(payload.get("text", "")))


def handle_wwi_message(bridge: MobileBridge, router: IntentRouter, relay: Any, msg: str) -> None:
    if not msg:
        return
    if msg.startswith("configure"):
        push_configure(bridge, relay)
        return
    if msg.startswith("stop"):
        bridge.note_external_command("chat:stop")
        # THIS RUNS ON THE SIM THREAD (main() pumps wwi messages), so
        # act_stop cannot spend a settling window here -- it returns
        # stationary: None with the reason, and says so rather than claiming
        # "halted". The HOLD, which is the part that actually keeps the robot
        # still, is armed either way.
        res = bridge.act_stop()
        hold = res.get("idle_loop") or {}
        held = (f" Holding for {hold['hold_s']:.0f}s, then it resumes on its "
                f"own." if hold.get("present") else "")
        bridge.queue_window(
            "agent:Wheels commanded to zero." + held
            + " (Rest not measured from this button — read the state panel.)")
        bridge.queue_window("tool:stop_robot:ok:commanded v=0, rest unmeasured")
        bridge.queue_window("status:idle")
        return
    if msg.startswith("prompt:"):
        bridge.note_external_command("chat:prompt")
        text = msg[len("prompt:"):]
        if relay is not None:
            # ENTRY PATH 2 of 2 (the Robot Window chat panel) -- the one the
            # operator actually types into. Transcribed by wrapping the
            # per-turn event callback; _tx_window_cb hands the lambda back
            # untouched when OMNILINK_TRANSCRIPT is unset.
            # _autoread_window_cb is the inner wrap: it makes a relay-side
            # auto-read visible as a tool line instead of leaving the operator
            # with a numeric answer and no read behind it.
            relay.dispatch_async(text, _tx_window_cb(
                bridge, relay, text, "window",
                _autoread_window_cb(
                    bridge, relay,
                    lambda k, p: _on_relay_event(bridge, k, p))))
            return
        bridge.queue_window("status:thinking")
        _tx = _tx_begin(bridge, relay, text, "window")
        result = router.dispatch(text)
        for (tool, status, summary) in result["tools"]:
            bridge.queue_window(f"tool:{tool}:{status}:{summary}")
        bridge.queue_window("agent:" + result["agent"])
        bridge.queue_window("status:idle")
        _tx_end(_tx, reply=result["agent"], router_tools=result["tools"])
        return
    if msg.startswith("audio_in:"):
        if relay is None:
            bridge.queue_window("error:audio_in requires OMNI_KEY (no relay attached)")
            return
        import base64 as _b64
        payload = msg[len("audio_in:"):]
        try:
            info = json.loads(payload)
            audio = _b64.b64decode(info.get("audio_b64", ""))
            mime = info.get("mime_type", "audio/webm")
        except Exception as e:
            bridge.queue_window(f"error:audio_in decode failed: {e}")
            return
        bridge.queue_window("status:transcribing")

        def _stt_worker():
            text = relay.transcribe(audio, mime_type=mime)
            if not text:
                bridge.queue_window("error:could not transcribe audio")
                bridge.queue_window("status:idle")
                return
            bridge.queue_window("transcript:" + text)
            # Same window path, reached by voice: the prompt is the STT
            # transcription, so it is tagged apart from a typed one.
            relay.dispatch_async(text, _tx_window_cb(
                bridge, relay, text, "window_voice",
                _autoread_window_cb(
                    bridge, relay,
                    lambda k, p: _on_relay_event(bridge, k, p))))

        threading.Thread(target=_stt_worker, name="omnilink-stt", daemon=True).start()
        return
    bridge.queue_window("system:Unknown window message: " + msg[:200])


# ── Main ─────────────────────────────────────────────────────────────

def main() -> int:
    args = _parse_args()
    cfg = get_config(args.robot)
    robot = Supervisor()
    # --name overrides the agent id so multiple bases of the same kind
    # (e.g. 4 Huskies in a swarm) get distinct OmniLink profiles + Axis ids.
    robot_id = args.name or args.robot
    pallet_defs = (args.pallets.split(",") if args.pallets else None)
    bridge = MobileBridge(robot, cfg, robot_id, pallet_defs=pallet_defs)
    # The relay's main task is built during setup_omnilink_relay below, so
    # the tug's ROLE has to be on the bridge before that call -- otherwise
    # the platform gets the generic mobile-base prompt and the Foreman's
    # delegation to this robot has nothing behind it.
    bridge.idle_mode = args.idle_loop
    # Same reason, for the same call: get_peer_state is only registered when
    # this tug actually watches another one, and the idle loop that holds
    # peer_port is not constructed until further down.
    bridge.idle_peer_port = int(args.idle_peer_port or 0)
    bridge.idle_arm_port = int(args.idle_arm_port or 0)
    router = IntentRouter(bridge)
    relay = setup_omnilink_relay(bridge, http_port=args.port)
    start_http(bridge, router, args.port, relay)

    # Opt-in ambient idle loop (keeps the tug demo alive when nobody is
    # chatting; any operator command pauses it instantly).
    if args.idle_loop:
        bridge.idle_loop = MavIdleLoop(bridge,
                                       resume_s=args.idle_resume_s,
                                       arm_port=args.idle_arm_port,
                                       period=args.idle_period,
                                       mode=args.idle_loop,
                                       peer_port=args.idle_peer_port)
        bridge.idle_loop.start()
        print(f"[omnilink_mobile_bridge] idle loop {args.idle_loop!r} enabled "
              f"(pauses on any operator command; "
              f"resumes after {args.idle_resume_s:.0f}s quiet)")

    print(f"[omnilink_mobile_bridge] {cfg['model']} ready as '{robot_id}' "
          f"layout={cfg['layout']} r={bridge.r:.3f} ht={bridge.ht:.3f} "
          f"({'OmniLink' if relay else 'local'})")

    timestep = bridge.timestep
    dt_s = timestep / 1000.0
    while robot.step(timestep) != -1:
        if _TX_ENABLED:
            # Cache the sim clock for the transcript: a chat turn is stamped
            # from the main thread, never by calling the Robot API from the
            # HTTP or relay-worker thread.
            bridge.last_sim_t = robot.getTime()
        while True:
            msg = robot.wwiReceiveText()
            if msg is None or msg == "":
                break
            try:
                handle_wwi_message(bridge, router, relay, msg)
            except Exception as e:
                bridge.queue_window(f"error:bridge_exception: {e!r}")
        with bridge.lock:
            outbox = bridge.window_outbox
            bridge.window_outbox = []
        for line in outbox:
            try:
                robot.wwiSendText(line)
            except Exception as e:
                print(f"[omnilink_mobile_bridge] wwiSendText failed: {e}")
        bridge.tick(dt_s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
