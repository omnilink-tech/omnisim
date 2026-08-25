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

"""omnilink_arm_bridge — generic OmniLink bridge for URDF manipulator arms.

Drop this in as the URDFRobot's controller (`supervisor TRUE`). One
controllerArg picks which arm config to load from `_arm_configs.py`:

    controller "omnilink_arm_bridge"
    controllerArgs ["--robot" "ur5e" "--port" "8765"]

Adding a new arm: append a config to ARM_CONFIGS and you're done.

Surfaces
--------
1. HTTP on 127.0.0.1:<port>  (default 8765 -- matches Axis's
   AXIS_BRIDGE_URL default). Axis-normalized endpoints; see
   `omnilink-bridge.md` in olink/agents/axis/knowledge/.

       POST /list_robots          -> [{id, model, capabilities}]
       POST /get_robot_state      -> {q, tcp, fault, mode, last_tick_at,
                                      last_command}
       POST /read_joints          -> {q}
       POST /read_tcp_pose        -> {xyz, rpy}
       POST /set_joint_positions  -> {accepted, commanded, achieved, error,
                                      settled, clamped_q}          [waits]
       POST /set_tcp_target       -> {accepted, commanded, achieved, error,
                                      error_m, settled, solved_q,
                                      ik_residual_m}               [waits]
       POST /set_tcp_pose         -> as set_tcp_target              [waits]
       POST /solve_ik             -> {q, ik_residual_m, err_norm}  (no motion)
       POST /stop_robot           -> {halted_at, q, commanded, achieved,
                                      error, settled, stationary, measured,
                                      idle_loop}
       POST /reset_to_home        -> {accepted, commanded, achieved,
                                      error, settled}              [waits]
       POST /pick                 -> {accepted, holding, commanded, achieved,
                                      error_m, settled}            [waits]
       POST /place                -> {accepted, commanded, achieved (the
                                      PART's measured world xyz), error_m,
                                      settled}                     [waits]
       POST /open_gripper         -> {commanded_state, achieved_state,
                                      released, gripper}  (if a gripper is set)
       POST /close_gripper        -> {commanded_state, achieved_state, gripper}
       POST /set_gripper_width    -> {commanded_width_m, achieved_width_m,
                                      gripper}         width in metres
       POST /grasp                -> {gripper, attached}  close + hold
       POST /release              -> {gripper, released}  open + drop
       POST /prompt               -> {response, actions}  natural-language
       POST /learn                -> {started, recipe, message}  skill learning
       POST /<learned_verb>       -> {accepted, commanded, achieved,
                                      duration_s}  replay a skill learned
                                      this session (e.g. POST /toss) [waits]

   [waits] = PROTOCOL.md 5.4.1: the verb BLOCKS until the motion finishes
   and answers with what was MEASURED. Send {"wait": false} to get the old
   return-on-dispatch behaviour, then poll get_robot_state.last_command and
   match its `seq`. `achieved: null` always means nobody measured it --
   superseded, timed out, or a real arm is mirroring (see
   capabilities.non_waitable_actions / waitable_actions / wait_default).
       GET  /factory/events       -> SSE stream of learn-pipeline events
                                     (replays history, then live; heartbeats)
       GET  /factory/status       -> {enabled, busy, recipes, learned_verbs}
       GET  /hud                  -> live learn HUD page
       GET  /capabilities         -> as listed in /list_robots
       GET  /state                -> alias for get_robot_state
       GET  /hardware_status      -> {enabled} or the hardware backend's status

2. Webots robot window: omnilink_chat plugin. The window POSTs
   "prompt:<text>" / "stop" / "configure" via wwi; the bridge replies
   with "agent:<text>", "tool:<name>:<ok|err>:<summary>", "status:<state>",
   "system:<text>" lines back through wwiSendText.

Without OmniLink configured, the bridge ships a regex-based intent
router that maps prompts directly to its own surface (no LLM). Set
OMNILINK_RELAY=1 + OMNI_KEY to relay prompts through OmniLink (future
PR; the relay hook is wired but the network path is left stubbed).

Real hardware (optional)
------------------------
The bridge is sim-first, but the same commands can also drive a real arm
through a pluggable *hardware backend* -- a sibling `<name>_backend.py`
module, selected with `--hardware-backend <name> [--hardware-ip <addr>]`.
See the HardwareBackend protocol below for the contract. No backend ships
in the box; with none installed the bridge is pure sim and the option is
simply not offered.
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Protocol, Tuple

from omnisim import Supervisor

# Make sibling modules importable when Webots invokes the controller.
import os as _os

# Process start, for the wall-clock stamp on idle/line events (see _log).
_T0 = time.time()
import sys as _sys
_THIS_DIR = _os.path.dirname(_os.path.abspath(__file__))
if _THIS_DIR not in _sys.path:
    _sys.path.insert(0, _THIS_DIR)
# Make the shared OmniLink relay package importable.
_RELAY_PARENT = _os.path.abspath(_os.path.join(_THIS_DIR, ".."))
if _RELAY_PARENT not in _sys.path:
    _sys.path.insert(0, _RELAY_PARENT)

# Shared conversational intents (resume / status) + the honest /state
# describer. Lives in omnisim_bridges so the arm and the mobile bridge
# answer "carry on" and "what are you doing?" the same way. Optional:
# a bare clone without the package installed keeps working, it just
# loses those two offline intents.
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

from _arm_configs import ARM_CONFIGS, get_config  # noqa: E402
from _gripper_configs import (  # noqa: E402
    GRIPPER_CONFIGS,
    get_gripper_config,
    legacy_gripper_config,
)
from gripper_effectors import make_effector  # noqa: E402
from _chat_page import CHAT_HTML  # noqa: E402
from _learn_manager import (  # noqa: E402
    LearnManager,
    get_hud_html,
    write_sse_stream,
)
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
    number_list,
    read_json,
    require_field,
    validate_request_id,
    WIRE_SERVICE,
    WIRE_VERSION,
)

# Optional -- only available when the relay package is on the path.
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

# The DEFERRED-INTENT layer -- the persistent store + tools that let a model
# say "finish this box then stop" / "tell me before you pick another part"
# and have it actually survive the chat turn. Optional import for the same
# reason as the relay: a bare clone keeps every immediate tool and simply
# cannot schedule, which is the honest degradation (no tool, so no promise
# nothing records).
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

_TX_TAG = "omnilink_arm_bridge"
_TX_BRIDGE = "arm"

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
# surface, the relay dispatches that read ITSELF, hands the payload back, and
# has the model answer again (relay.py `_reground_if_unread`). It journals the
# call with summary "auto-read (grounding gate)" -- but it emits NO "tool"
# event, and `actions` in the /prompt response and the tool lines in the chat
# panel are both built from those events alone. So a regrounded turn looks,
# from outside, exactly like a fabrication: a reply full of numbers with no
# read behind it.
#
# That is not a hypothetical. The QA report that commissioned this change filed
# "how many boxes have you shipped so far? TOOLS: [] -- claims 'my records
# show' over records it never read" as the flagship fabrication. The journal for
# that exact turn (12:24:49Z, omniarm6) holds ONE entry:
#   {"tool": "get_robot_state", "ok": true, "summary": "auto-read (grounding gate)"}
# The gate had fired, the read happened, and the answer ("0 boxes") came from
# live line counters. The defect was that nothing SAID so.
#
# So: read the journal back and report what the event stream could not. The
# read genuinely happened -- this reports it, it does not manufacture it.
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
    action list it always saw (both auto-read tools are read-only anyway).

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


# ── Hardware backends (pluggable, discovered by name) ────────────────
#
# The bridge is sim-first: it always drives the simulated arm. A *hardware
# backend* is an OPTIONAL adapter that lets the same commands ALSO drive a
# real robot over that robot's own control API, and feeds the real robot's
# measured joints back so the simulated arm mirrors it (a live digital twin).
#
# A backend is a sibling module in this directory named `<name>_backend.py`,
# selected at runtime with `--hardware-backend <name> [--hardware-ip <addr>]`
# (or OMNILINK_HARDWARE_BACKEND / OMNILINK_HARDWARE_IP). Nothing about any
# particular robot vendor is known to, or named in, this file: drop the module
# in and the option appears; take it away and the bridge is pure sim.
#
# A backend module must expose one factory:
#
#     maybe_build(cfg, robot_id, ip_arg=None, on_event=None)
#         -> HardwareBackend | None
#
#   cfg       the arm config dict from _arm_configs (the backend may read its
#             own optional block out of it, e.g. a joint sign/offset map).
#   robot_id  the bridge's robot id.
#   ip_arg    the address from --hardware-ip, or None.
#   on_event  callable(kind, text) -- "status" / "error" lines the bridge
#             forwards to the robot window.
#
#   It MUST return None when the operator has not opted in, so that merely
#   having the module present never touches hardware on a plain sim launch.
#
# The object it returns must implement HardwareBackend. All command methods
# are fire-and-forget: they must NOT block the simulation tick.


class HardwareBackend(Protocol):
    """The surface the bridge uses to drive a real arm alongside the sim."""

    ip: str            # address the backend is talking to (for status/logs)
    dry_run: bool      # True when running against an in-process mock
    connected: bool    # flips True once the link is up; commands are no-ops
                       # until it does

    # -- lifecycle --------------------------------------------------------
    def start(self) -> None: ...
    def shutdown(self) -> None: ...

    # -- reads ------------------------------------------------------------
    def status(self) -> Dict[str, Any]: ...
    def get_joints(self) -> Optional[List[float]]: ...   # measured q, sim-frame

    # -- commands (non-blocking) ------------------------------------------
    def move_joint(self, q: List[float]) -> None: ...
    def move_linear(self, xyz: Tuple[float, float, float]) -> None: ...
    def reset_to_home(self) -> None: ...
    def wave(self, amplitudes: List[float]) -> None: ...
    def grasp(self) -> None: ...
    def release(self) -> None: ...
    def stop(self) -> None: ...


_BACKEND_SUFFIX = "_backend.py"


def discover_hardware_backends() -> List[str]:
    """Names of the hardware-backend modules sitting next to this controller."""
    found: List[str] = []
    try:
        for fn in sorted(_os.listdir(_THIS_DIR)):
            if fn.endswith(_BACKEND_SUFFIX) and not fn.startswith("_"):
                found.append(fn[: -len(_BACKEND_SUFFIX)])
    except OSError:
        pass
    return found


def load_hardware_backend(name: str, cfg: dict, robot_id: str,
                          ip_arg: Optional[str] = None,
                          on_event=None) -> Tuple[Optional[Any], Optional[str]]:
    """Import `<name>_backend.py` from this directory and ask it to build.

    Returns (backend, error). `backend` is None either because the module is
    absent / malformed (then `error` says so) or because the backend declined
    to activate -- the operator did not opt in (then `error` is None too).
    """
    if not re.fullmatch(r"[A-Za-z0-9_]+", name or ""):
        return None, f"invalid hardware backend name {name!r}"
    available = discover_hardware_backends()
    if not _os.path.isfile(_os.path.join(_THIS_DIR, name + _BACKEND_SUFFIX)):
        return None, (
            f"hardware backend '{name}' is not installed: no "
            f"{name}{_BACKEND_SUFFIX} next to this controller. "
            f"Available: {available if available else 'none'}.")
    try:
        mod = importlib.import_module(name + "_backend")
    except Exception as e:
        return None, f"hardware backend '{name}' failed to import: {e!r}"
    build = getattr(mod, "maybe_build", None)
    if not callable(build):
        return None, (f"hardware backend '{name}' exposes no "
                      f"maybe_build(cfg, robot_id, ip_arg=..., on_event=...)")
    try:
        return build(cfg, robot_id, ip_arg=ip_arg, on_event=on_event), None
    except Exception as e:
        return None, f"hardware backend '{name}' failed to build: {e!r}"


def attach_hardware(bridge: "ArmBridge", cfg: dict, robot_id: str,
                    name: Optional[str], ip: Optional[str]) -> None:
    """Resolve, build and attach a hardware backend, or leave the bridge pure-sim.

    Off by default. With an explicit `--hardware-backend`, a module that cannot
    be loaded is a hard error (the operator asked for hardware; silently running
    sim-only would be a lie). With no name given, each discovered backend is
    offered the chance to activate itself from its own environment -- every
    backend returns None unless the operator opted in, so a plain sim launch
    attaches nothing.
    """
    on_event = lambda k, t: bridge.queue_window(          # noqa: E731
        ("error:" + t) if k == "error" else ("system:" + t))

    candidates: List[str]
    if name:
        candidates = [name]
    elif ip:
        found = discover_hardware_backends()
        if len(found) != 1:
            raise SystemExit(
                "[omnilink_arm_bridge] --hardware-ip given but no "
                "--hardware-backend: cannot tell which backend to use "
                f"(available: {found if found else 'none'}).")
        candidates = found
    else:
        candidates = discover_hardware_backends()

    for cand in candidates:
        be, err = load_hardware_backend(cand, cfg, robot_id,
                                        ip_arg=ip, on_event=on_event)
        if err is not None:
            if name:                       # explicit request -> fail loudly
                raise SystemExit(f"[omnilink_arm_bridge] {err}")
            print(f"[omnilink_arm_bridge] hardware backend '{cand}' "
                  f"unavailable: {err}")
            continue
        if be is None:
            continue                       # backend declined: not opted in
        bridge.hw = be
        bridge.hw_name = cand
        be.start()
        print(f"[omnilink_arm_bridge] hardware backend '{cand}' attached "
              f"(ip={getattr(be, 'ip', '?')}, "
              f"dry_run={getattr(be, 'dry_run', False)})")
        return

    if name:
        print(f"[omnilink_arm_bridge] hardware backend '{name}' did not "
              "activate (no address given and no opt-in in its environment); "
              "running sim-only.")


# ── CLI ──────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--robot", default=next(iter(ARM_CONFIGS), None),
                        help=f"Robot id, one of {sorted(ARM_CONFIGS.keys())}. "
                             "Defaults to the first registered arm.")
    parser.add_argument("--port", type=int, default=8765,
                        help="HTTP port for the Axis-normalized surface")
    parser.add_argument("--gripper", default=None,
                        help=("Gripper id to attach, one of "
                              f"{sorted(GRIPPER_CONFIGS.keys())}. Omit to use "
                              "the arm's default_gripper / legacy inline "
                              "gripper fields, if any."))
    parser.add_argument("--name", default=None,
                        help=("Override the agent id used in the OmniLink "
                              "profile name and Axis robot_id. Defaults to "
                              "--robot. Use when multiple arms of the same "
                              "kind share a world (e.g. several UR5es)."))
    parser.add_argument("--drop-zone", default=None, metavar="X,Y,Z",
                        help=("Override the arm config's `drop_zone` -- where "
                              "`place` puts things when no explicit point is "
                              "given -- in the ARM BASE frame, metres. A "
                              "drop_zone lives in the arm config, so every "
                              "world sharing an arm inherits one tuned for a "
                              "different scene: the OmniArm 6's is (0.30, 0.34, "
                              "0.0), which is floor height and lands a part "
                              "INSIDE any table you stand it on. Set it per "
                              "world instead of contorting the world to fit."))
    _backends = discover_hardware_backends()
    parser.add_argument("--hardware-backend", default=None,
                        help=("Drive a REAL arm alongside the sim through the "
                              "named hardware backend "
                              f"({_backends if _backends else 'none installed'})"
                              ". A backend is a sibling <name>_backend.py "
                              "module; omit for a pure-sim run. Also settable "
                              "via OMNILINK_HARDWARE_BACKEND."))
    parser.add_argument("--hardware-ip", default=None,
                        help=("Address of the real arm (or its offline-sim VM) "
                              "handed to the hardware backend. Also settable "
                              "via OMNILINK_HARDWARE_IP."))
    parser.add_argument("--idle-loop", default=None, choices=["pick"],
                        help=("OPT-IN ambient demo loop. 'pick' cycles the "
                              "phased pick (hover/descend/attach/lift/drop) "
                              "over every GRASP_* part while nobody is "
                              "chatting; ANY prompt or tool call pauses it "
                              "instantly and it resumes after --idle-resume-s "
                              "of quiet. Off by default (zero behaviour "
                              "change)."))
    parser.add_argument("--idle-drop", default=None,
                        help=("DEF name(s), comma separated, of the trolley / "
                              "basket node(s) the idle loop drops picked parts "
                              "into (tracked live, so a towed-away basket "
                              "pauses the loop). In a conveyor-LINE world "
                              "these are the trolleys the line master loads "
                              "filled boxes onto. Omit to use the arm cfg's "
                              "static drop_zone."))
    parser.add_argument("--idle-resume-s", type=float, default=60.0,
                        help="Quiet seconds after the last operator command "
                             "before the idle loop resumes (default 60).")
    args, _unknown = parser.parse_known_args()
    if not args.hardware_backend:
        args.hardware_backend = _os.environ.get("OMNILINK_HARDWARE_BACKEND") or None
    if not args.hardware_ip:
        args.hardware_ip = _os.environ.get("OMNILINK_HARDWARE_IP") or None
    return args


# ── Math helpers ─────────────────────────────────────────────────────

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def clamp_q(q: List[float], limits: List[Tuple[float, float]]) -> List[float]:
    return [clamp(qi, lo, hi) for qi, (lo, hi) in zip(q, limits)]


def wrap_q_near(q: List[float], limits: List[Tuple[float, float]],
                q_ref: List[float]) -> List[float]:
    """clamp_q, but a joint whose range spans MORE than 2*pi is first re-encoded
    to the equivalent angle NEAREST ``q_ref`` -- the minimum-rotation encoding --
    instead of being allowed to wind up against its stop.

    WHY. Forward kinematics is exactly 2*pi-periodic in every revolute joint, so
    on a joint with several legal encodings of one physical angle the encoding is
    an implementation detail -- but the MOTION between two encodings is not.

    MEASURED (warehouse_omnilink, 906 samples / 3 dispatch cycles): the OmniArm 6's
    joint4 walked 0.01 -> 3.14 -> 6.28 rad inside a single pick cycle. 6.28 is
    exactly the +-2*pi stop the URDF declares for it
    (projects/robots/omnisim/omniarm6/omniarm6_gumgrip.urdf:176,
    lower="-6.28319" upper="6.28319") and is the SAME wrist orientation as 0.00,
    so the winding bought nothing -- but _place_held's closing 1.4 s "move to
    home" then had to unwind all of it: 6.28/1.4 = 4.49 rad/s mean and, under
    _tick_sequence's smooth cubic ease (peak rate = 1.5x mean), 6.73 rad/s peak,
    against the 3.1416 rad/s motor limit the same URDF declares (velocity= on
    joints 1-4). The motor saturates, the arm falls behind its setpoint, and
    _tick_sequence reseeds the next segment from the COMMANDED pose
    (params["from_q"] = list(to)), not the achieved one -- so the next segment
    starts from a pose the arm never reached. On camera: lag, then a lurch.
    Reproduced offline from this file's own IK over all 6 feeder parts x 3 box
    slots: max |q4| 6.283 rad, worst segment peak 4.74 rad/s.

    ORDINARY JOINTS ARE UNTOUCHED. A span of 2*pi or less has at most one legal
    encoding of a given angle, so those joints fall straight through to the same
    clamp they always got. The clamp still runs afterwards, so nothing can leave
    the declared range either way.

    WHICH JOINTS THIS TOUCHES, and one discrepancy worth knowing. The test is
    the CONFIG's `joint_limits`, because that is all this function is handed.
    On the OmniArm 6 the config declares joint1, joint4 and joint6 as +-2*pi
    (span 4*pi) and joints 2, 3, 5 as +-pi. The URDF -- which is what the engine
    actually enforces on the motors -- agrees on joint4 and joint6
    (omniarm6_gumgrip.urdf:176 and :192, both +-6.28319) but declares joint1 as
    +-3.14159, a span of exactly 2*pi. So the config over-declares joint1 and
    this function will treat it as continuous when the hardware does not. That
    is inert in practice (the solver steps 0.08 rad at a time from a seed that
    is already in range, so joint1 never gets near a turn away from its
    reference, and it was measured at |q1| < 0.4 throughout), but if joint1's
    config range is ever narrowed to match the URDF the treatment corrects
    itself with no change here. The UR family declares five of its six wider
    than 2*pi, which is right -- UR wrists really are continuous.
    """
    out = []
    for qi, (lo, hi), ref in zip(q, limits, q_ref):
        if hi - lo > 2.0 * math.pi + 1e-9:
            # Nearest equivalent: shift by whole turns until within +-pi of ref.
            qi = qi + 2.0 * math.pi * round((ref - qi) / (2.0 * math.pi))
        out.append(clamp(qi, lo, hi))
    return out


def _rot(axis: Tuple[float, float, float], theta: float) -> List[List[float]]:
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(theta), math.sin(theta)
    C = 1.0 - c
    return [
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def _rot_rpy(rpy: Tuple[float, float, float]) -> List[List[float]]:
    r, p, y = rpy
    Rz = _rot((0, 0, 1), y)
    Ry = _rot((0, 1, 0), p)
    Rx = _rot((1, 0, 0), r)
    # ZYX intrinsic == roll about X, then pitch about Y, then yaw about Z
    return _mat_mul(_mat_mul(Rz, Ry), Rx)


def _mat_mul(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    return [
        [sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _mat_vec(A: List[List[float]], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    )


def forward_kinematics(chain: List[Tuple], q: List[float], tcp_offset: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """TCP world position. Chain is parent-frame URDF tuples
    (origin_xyz, origin_rpy, joint_axis)."""
    R = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    p = [0.0, 0.0, 0.0]
    for (offset, rpy, axis), qi in zip(chain, q):
        R_offset = _rot_rpy(rpy)
        # Apply parent-frame offset (rotated into current frame).
        off_world = _mat_vec(R, offset)
        p = [p[0] + off_world[0], p[1] + off_world[1], p[2] + off_world[2]]
        R = _mat_mul(R, R_offset)
        # Joint rotation about its local axis.
        R = _mat_mul(R, _rot(axis, qi))
    tcp_world = _mat_vec(R, tcp_offset)
    return (p[0] + tcp_world[0], p[1] + tcp_world[1], p[2] + tcp_world[2])


def forward_kinematics_pose(chain: List[Tuple], q: List[float],
                            tcp_offset: Tuple[float, float, float]):
    """Like forward_kinematics but also returns the TCP rotation matrix
    (base frame). Used to orient a mounted gripper to the wrist."""
    R = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    p = [0.0, 0.0, 0.0]
    for (offset, rpy, axis), qi in zip(chain, q):
        off_world = _mat_vec(R, offset)
        p = [p[0] + off_world[0], p[1] + off_world[1], p[2] + off_world[2]]
        R = _mat_mul(R, _rot_rpy(rpy))
        R = _mat_mul(R, _rot(axis, qi))
    tcp = _mat_vec(R, tcp_offset)
    return [p[0] + tcp[0], p[1] + tcp[1], p[2] + tcp[2]], R


def _mat3_to_axis_angle(m) -> List[float]:
    """3x3 rotation matrix -> Webots [x, y, z, angle] axis-angle."""
    trace = m[0][0] + m[1][1] + m[2][2]
    c = clamp((trace - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(c)
    if angle < 1e-6:
        return [0.0, 0.0, 1.0, 0.0]
    if abs(math.pi - angle) < 1e-3:
        # Near 180 deg: pull axis from the diagonal.
        ax = math.sqrt(max(0.0, (m[0][0] + 1.0) / 2.0))
        ay = math.sqrt(max(0.0, (m[1][1] + 1.0) / 2.0))
        az = math.sqrt(max(0.0, (m[2][2] + 1.0) / 2.0))
        return [ax, ay, az, angle]
    s = 2.0 * math.sin(angle)
    return [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s, angle]


def numerical_jacobian(chain, q, tcp_offset, eps: float = 1e-4):
    """Position-only Jacobian, central-difference. 3xN matrix as list of rows."""
    N = len(q)
    J = [[0.0] * N for _ in range(3)]
    for j in range(N):
        qp = list(q); qp[j] += eps
        qm = list(q); qm[j] -= eps
        fp = forward_kinematics(chain, qp, tcp_offset)
        fm = forward_kinematics(chain, qm, tcp_offset)
        for i in range(3):
            J[i][j] = (fp[i] - fm[i]) / (2 * eps)
    return J


def dls_ik(chain, q_seed, target_xyz, ik_cfg, joint_limits):
    """Damped least squares IK, position-only. Returns (q, err_norm, iters)."""
    q = list(q_seed)
    max_iters = ik_cfg["max_iters"]
    tol = ik_cfg["tol"]
    damping = ik_cfg["damping"]
    max_dq = ik_cfg["max_dq"]
    err_norm = 0.0
    for it in range(max_iters):
        x = forward_kinematics(chain, q, ik_cfg["tcp_offset"])
        err = [target_xyz[i] - x[i] for i in range(3)]
        err_norm = math.sqrt(sum(e * e for e in err))
        if err_norm < tol:
            return q, err_norm, it
        J = numerical_jacobian(chain, q, ik_cfg["tcp_offset"])
        # DLS: dq = J^T (J J^T + lambda^2 I)^-1 err
        JJt = [[0.0] * 3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                JJt[i][j] = sum(J[i][k] * J[j][k] for k in range(len(q)))
            JJt[i][i] += damping * damping
        # Invert 3x3
        inv = _invert3(JJt)
        if inv is None:
            return q, err_norm, it  # singular
        rhs = (
            inv[0][0] * err[0] + inv[0][1] * err[1] + inv[0][2] * err[2],
            inv[1][0] * err[0] + inv[1][1] * err[1] + inv[1][2] * err[2],
            inv[2][0] * err[0] + inv[2][1] * err[1] + inv[2][2] * err[2],
        )
        dq = [J[0][j] * rhs[0] + J[1][j] * rhs[1] + J[2][j] * rhs[2] for j in range(len(q))]
        # Cap step.
        max_abs = max(abs(d) for d in dq) or 1e-9
        if max_abs > max_dq:
            scale = max_dq / max_abs
            dq = [d * scale for d in dq]
        q = [q[j] + dq[j] for j in range(len(q))]
        # wrap_q_near, not clamp_q: keep a >2*pi joint in its minimum-rotation
        # encoding relative to the SEED rather than letting the solver walk it
        # onto its stop. FK and the Jacobian are 2*pi-periodic, so this cannot
        # change the pose the solver converges to -- only how it is written
        # down, and therefore how far the arm has to travel to get there.
        # See wrap_q_near's docstring for the measurement.
        q = wrap_q_near(q, joint_limits, q_seed)
    return q, err_norm, max_iters


def _invert3(M):
    a, b, c = M[0]
    d, e, f = M[1]
    g, h, i = M[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        return None
    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


# ── 6-DOF (pose) IK: position + orientation, for top-down grasping ───

def _transpose3(M):
    return [[M[0][0], M[1][0], M[2][0]],
            [M[0][1], M[1][1], M[2][1]],
            [M[0][2], M[1][2], M[2][2]]]


def _rotvec(R) -> List[float]:
    """Log map: 3x3 rotation -> axis*angle as a 3-vector (rad)."""
    tr = R[0][0] + R[1][1] + R[2][2]
    c = clamp((tr - 1.0) / 2.0, -1.0, 1.0)
    ang = math.acos(c)
    if ang < 1e-8:
        return [0.0, 0.0, 0.0]
    if abs(math.pi - ang) < 1e-4:
        # Near 180 deg: axis from the largest diagonal of (R + I)/2.
        d = [(R[0][0] + 1.0) / 2.0, (R[1][1] + 1.0) / 2.0, (R[2][2] + 1.0) / 2.0]
        k = max(range(3), key=lambda i: d[i])
        axis = [0.0, 0.0, 0.0]
        axis[k] = math.sqrt(max(0.0, d[k]))
        for i in range(3):
            if i != k:
                axis[i] = (R[i][k] + R[k][i]) / (4.0 * axis[k]) if axis[k] > 1e-9 else 0.0
        n = math.sqrt(sum(a * a for a in axis)) or 1.0
        return [a / n * ang for a in axis]
    s = 2.0 * math.sin(ang)
    return [(R[2][1] - R[1][2]) / s * ang,
            (R[0][2] - R[2][0]) / s * ang,
            (R[1][0] - R[0][1]) / s * ang]


def _solve_linear(A, b):
    """Solve A x = b for square A (Gaussian elimination, partial pivot)."""
    n = len(b)
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-12:
            return None
        M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        for r in range(n):
            if r == col:
                continue
            f = M[r][col] / pv
            for k in range(col, n + 1):
                M[r][k] -= f * M[col][k]
    return [M[i][n] / M[i][i] for i in range(n)]


def numerical_jacobian_pose(chain, q, tcp_offset, eps: float = 1e-4):
    """6xN pose Jacobian (rows 0-2 position, 3-5 orientation), central diff."""
    N = len(q)
    J = [[0.0] * N for _ in range(6)]
    for j in range(N):
        qp = list(q); qp[j] += eps
        qm = list(q); qm[j] -= eps
        pp, Rp = forward_kinematics_pose(chain, qp, tcp_offset)
        pm, Rm = forward_kinematics_pose(chain, qm, tcp_offset)
        for i in range(3):
            J[i][j] = (pp[i] - pm[i]) / (2 * eps)
        w = _rotvec(_mat_mul(Rp, _transpose3(Rm)))
        for i in range(3):
            J[3 + i][j] = w[i] / (2 * eps)
    return J


def dls_ik_pose(chain, q_seed, target_pos, target_R, tcp_offset, ik_cfg,
                joint_limits, w_rot: float = 0.4):
    """Damped least squares IK to a full pose (position + orientation).
    Returns (q, pos_err, rot_err, iters)."""
    q = list(q_seed)
    max_iters = ik_cfg.get("pose_max_iters", 200)
    pos_tol = ik_cfg.get("tol", 5e-3)
    damping = ik_cfg.get("damping", 0.08)
    max_dq = ik_cfg.get("max_dq", 0.08)
    pos_err = rot_err = 0.0
    for it in range(max_iters):
        p, R = forward_kinematics_pose(chain, q, tcp_offset)
        ep = [target_pos[i] - p[i] for i in range(3)]
        eo = _rotvec(_mat_mul(target_R, _transpose3(R)))
        pos_err = math.sqrt(sum(e * e for e in ep))
        rot_err = math.sqrt(sum(e * e for e in eo))
        if pos_err < pos_tol and rot_err < 0.03:
            return q, pos_err, rot_err, it
        J = numerical_jacobian_pose(chain, q, tcp_offset)
        for i in range(3):
            for j in range(len(q)):
                J[3 + i][j] *= w_rot
        e = [ep[0], ep[1], ep[2], w_rot * eo[0], w_rot * eo[1], w_rot * eo[2]]
        JJt = [[sum(J[i][k] * J[r][k] for k in range(len(q))) for r in range(6)]
               for i in range(6)]
        for i in range(6):
            JJt[i][i] += damping * damping
        y = _solve_linear(JJt, e)
        if y is None:
            return q, pos_err, rot_err, it
        dq = [sum(J[i][j] * y[i] for i in range(6)) for j in range(len(q))]
        ma = max(abs(d) for d in dq) or 1e-9
        if ma > max_dq:
            dq = [d * (max_dq / ma) for d in dq]
        q = [q[j] + dq[j] for j in range(len(q))]
        # Same minimum-rotation guard as dls_ik -- this is the solver the pick
        # loop actually uses (ArmIdleLoop._solve), so this is where joint4's
        # 0.01 -> 3.14 -> 6.28 rad wind-up was being manufactured.
        q = wrap_q_near(q, joint_limits, q_seed)
    return q, pos_err, rot_err, max_iters


# ── Cold-first-load warm-up ──────────────────────────────────────────

def warmup_reload(robot) -> bool:
    """No-op by default. Historically reloaded the world ONCE to dodge the COLD-FIRST-LOAD bug.

    BACKGROUND: on an early Newton build a FRESH world build's MuJoCo articulation under-tracked
    its position targets (arm undershot its commanded pose by ~1 cm) so precise grasps failed cold
    but worked after a world reload. Every precise-manipulation controller therefore reloaded once
    at startup.

    RESOLVED (verified 2026-07-05): the under-track NO LONGER reproduces on the current binary.
    Cold and warm loads settle bit-identically (bare-arm probe: identical joint + end-effector to
    6 decimals; a full arm+gripper grasp: identical every phase), with cold correctly building
    MuJoCo (no XPBD fallback). Root fix: `eb86f888` (Newton solver choice survives the multi-build
    load) + the finalize-time solver re-assert. See docs/developer/real-grasp-and-the-cold-first-
    load-trap.md. The startup reload is now pure overhead, so it is OFF by default.

    Kept as a one-line safety valve: set OMNISIM_FORCE_WARMUP=1 to re-enable the reload if a
    regression ever resurfaces (OMNISIM_NO_WARMUP=1 still forces it off, and wins). Returns True
    only if a reload was actually triggered."""
    import os
    import tempfile
    # Off by default now that the cold-load bug is fixed; opt back in with OMNISIM_FORCE_WARMUP=1.
    if os.environ.get("OMNISIM_NO_WARMUP") or not os.environ.get("OMNISIM_FORCE_WARMUP"):
        return False
    # OMNISIM_WARMUP_TOKEN (set per-launch by the headless runner / launcher) is the
    # most robust session key; fall back to the parent (simulator) pid otherwise.
    key = os.environ.get("OMNISIM_WARMUP_TOKEN")
    if not key:
        try:
            key = str(os.getppid())
        except Exception:
            key = "0"
    flag = os.path.join(tempfile.gettempdir(), "_omnisim_warmup_%s.flag" % key)
    if os.path.exists(flag):
        return False
    try:
        with open(flag, "w") as _f:
            _f.write("1")
    except Exception:
        return False            # can't set the loop-guard -> never reload
    try:
        dt = int(robot.getBasicTimeStep())
        # worldReload() raises if called before the controller has stepped, so run a
        # few steps to fully initialise first; the reload then takes effect cleanly.
        for _ in range(10):
            if robot.step(dt) == -1:
                return False
        robot.worldReload()
    except Exception:
        return False            # reload didn't fire -> let the (cold) demo run
    # The reload IS requested and this controller is being torn down + restarted.
    # CRITICAL: do NOT fall back into the caller and run the cold demo body here --
    # in the windowed GUI that races the reload and closes the window. Step until the
    # teardown delivers -1, then hard-exit so only the warm restart runs the demo.
    try:
        while robot.step(dt) != -1:
            pass
    except Exception:
        pass
    os._exit(0)


# ── Main-thread call marshalling ─────────────────────────────────────

class MainThreadCalls:
    """Marshals supervisor API calls from worker threads onto the sim thread.

    The controller API is not thread-safe: a supervisor pipe call issued
    from a background thread while the main loop is stepping stalls the
    whole controller<->sim step exchange (measured on the warehouse idle
    loop: ~2 threaded getPosition()/s dragged the sim to 0.18x realtime;
    marshalled, the same reads are free). Workers enqueue a closure with
    ``call``; the bridge's tick() pumps the queue on the sim thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._q: List[tuple] = []

    def call(self, fn, timeout: float = 6.0):
        """Run ``fn()`` on the sim thread; return its result (None on
        timeout/error — callers are fail-soft)."""
        ev = threading.Event()
        box: Dict[str, Any] = {}
        with self._lock:
            self._q.append((fn, ev, box))
        ev.wait(timeout)
        return box.get("r")

    def pump(self) -> None:
        """Sim thread: run queued closures. Called once per tick."""
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


# ── Bridge state ─────────────────────────────────────────────────────

class ArmBridge:
    """Owns motors, motion plans, and the wire protocol for one arm.

    The Webots simulation step calls `tick()` once per basicTimeStep,
    which advances the active motion plan and pushes motor setpoints.
    All other surfaces (HTTP, wwi prompt) just mutate `motion_plan`
    under `lock`.
    """

    # Standing restrictions the arm's own loop GENUINELY enforces. A rule
    # outside this dict is REFUSED by set_constraint -- agreeing to a rule
    # nobody checks is the failure this whole layer exists to kill.
    CONSTRAINT_RULES = {
        "no_new_picks": ("I start no new picks — I finish the part in the "
                         "gripper and then stand by over the cell."),
        "no_respawn": ("I do not respawn a fresh set of parts when the belt "
                       "runs dry."),
    }
    KNOWN_LEGS = ("idle", "pick", "place", "respawn")

    def __init__(self, robot: Supervisor, cfg: dict, robot_id: str,
                 gripper_id: Optional[str] = None) -> None:
        self.robot = robot
        self.cfg = cfg
        self.robot_id = robot_id
        self.timestep = int(robot.getBasicTimeStep())
        self.joint_names = list(cfg["joint_names"])
        self.joint_limits = list(cfg["joint_limits"])
        self.home_pose = list(cfg["home_pose"])
        self.motors = []
        self.sensors = []
        for jn in self.joint_names:
            motor = robot.getDevice(jn + "_motor")
            if motor is None:
                # Some URDF importers emit `<jn>` directly.
                motor = robot.getDevice(jn)
            if motor is None:
                print(f"[omnilink_arm_bridge] WARNING: missing motor for {jn!r}")
                # Keep sensors aligned with joint_names/motors: without this
                # placeholder every later joint reads its NEIGHBOUR's sensor.
                self.sensors.append(None)
            else:
                # Snappy tracking: run the position controller at the motor's
                # full velocity (URDF velocity limit) so the arm doesn't lag
                # behind the interpolated setpoint.
                try:
                    vmax = motor.getMaxVelocity()
                    if vmax and vmax > 0:
                        motor.setVelocity(vmax)
                except Exception:
                    pass
                # Try to attach a position sensor; URDF importer pairs them.
                try:
                    s = motor.getPositionSensor()
                    if s is not None:
                        s.enable(self.timestep)
                        self.sensors.append(s)
                    else:
                        self.sensors.append(None)
                except Exception:
                    self.sensors.append(None)
            self.motors.append(motor)

        # End effector. Resolution order:
        #   1. explicit --gripper <id>           (registry)
        #   2. arm cfg "default_gripper" id      (registry)
        #   3. legacy inline gripper_* fields    (back-compat shim)
        # None of these present -> no gripper.
        self.effector = None
        self.gripper_cfg = None
        gid = gripper_id or cfg.get("default_gripper")
        if gid:
            self.gripper_cfg = get_gripper_config(gid)
        elif cfg.get("gripper_motors"):
            self.gripper_cfg = legacy_gripper_config(cfg)
        if self.gripper_cfg is not None:
            self.effector = make_effector(robot, self.gripper_cfg)

        # Kinematic-attach grasp (Phase 3): on grasp the nearest graspable
        # Solid within `grasp_radius` of the TCP is welded to the tool and
        # teleported to follow it each tick; release drops it. Objects opt
        # in by giving their node a DEF that starts with "GRASP_". This
        # avoids physics-contact instability (see plan: kinematic attach).
        self.grasp_radius = float((self.gripper_cfg or {}).get("grasp_radius", 0.08))
        self.held_node = None
        self.held_tfield = None
        self._self_node = robot.getSelf()

        # Mounted gripper visual: an optional top-level Solid named
        # DEF GRIPPER_<robot_id> (with finger sub-nodes _FINGER_L / _R).
        # When present the bridge teleports it to the wrist (flange) each
        # tick and animates finger spacing from the effector width, so a
        # bare URDF arm shows a gripper at its tool without editing the URDF.
        self.gripper_visual = robot.getFromDef("GRIPPER_" + robot_id)
        self.gripper_fingers = (
            robot.getFromDef("GRIPPER_%s_FINGER_L" % robot_id),
            robot.getFromDef("GRIPPER_%s_FINGER_R" % robot_id),
        )
        # Anchor to the REAL tool-mount link's pose (read from the scene
        # tree), not an approximate FK chain -- otherwise the gripper clips
        # into the wrist mesh and the grasp point disagrees with where the
        # gripper is drawn. The mount link is the URDF "flange" (fallbacks
        # cover other arms' naming). Resolved whenever a gripper is set so
        # both the visual AND the grasp weld ride the same real pose.
        self.gripper_anchor = (self._find_mount_node()
                               if self.effector is not None else None)
        self.tool_reach = float((self.gripper_cfg or {}).get("tool_reach", 0.13))

        # Motion plan: a tuple (kind, params). The tick() loop owns
        # interpolation. Lock guards mutations.
        self.lock = threading.RLock()
        self.motion = ("hold", {"q": list(self.home_pose)})
        # PROTOCOL.md 5.4.1 rule 5: the motion slot is SINGLE, so a second
        # command arriving while one is in flight has to be refused rather
        # than silently swallow the first. `motion_seq` names the occupant
        # so a refusal can say WHICH motion it is waiting on (see
        # _begin_motion); `ticks` is only read for diagnostics.
        self.motion_seq = 0
        # ACTION RESULT CONTRACT (PROTOCOL.md 5.4.1), the completion half.
        # `_begin_motion` already claims the slot; this is where tick() writes
        # what the motion ACTUALLY did once it ends. `accepted: true` plus the
        # caller's own argument echoed back is not a result -- it is the
        # request handed back, and an agent has no way to tell the difference:
        # a tool that returns in 3 ms while the motion takes 1.5 s gives the
        # model nothing in its experience that distinguishes "submitted" from
        # "finished". None until the first stamped motion completes.
        self.last_completion: Optional[dict] = None
        self.ticks = 0
        self.last_q = list(self.home_pose)
        self.last_tick_at = time.time()
        self.fault: Optional[str] = None

        # Idle-loop bookkeeping (see ArmIdleLoop). last_external_cmd is the
        # wall-clock time of the last operator command (chat prompt / HTTP
        # tool call); the opt-in idle loop pauses while it is recent.
        self.last_external_cmd = 0.0
        self.last_external_src = ""
        # AN OPERATOR STOP HOLDS THE ARM FROM THE MOMENT IT EXECUTES.
        # The quiet window above is armed when the operator's PROMPT arrives
        # (HTTP _route_post / handle_wwi_message), not when the model finally
        # picks a tool, so an LLM turn longer than --idle-resume-s outlives
        # its own pause and the idle loop is free the instant the stop lands.
        # Measured on the mobile bridge's tug_a in the same demo: 1.5 s after
        # a "stop" the model reported as halted, the tug had moved 0.384 m
        # and turned 34.7 deg with idle_loop.paused == False. Same structure
        # here, so the same fix: act_stop arms this deadline itself, and
        # ArmIdleLoop._blocked reads it.
        self.stop_hold_until = 0.0
        # Joint samples taken by tick() (wall_t, q). The only honest basis
        # for "did the arm actually come to rest": a commanded hold says
        # nothing about whether the servos stopped tracking.
        self._q_hist: List[Tuple[float, List[float]]] = []
        self.idle_loop: Optional["ArmIdleLoop"] = None
        # Optional warehouse LINE MASTER (see WarehouseLine). Armed by main()
        # alongside the idle pick loop; auto-disables itself in any world that
        # is not a conveyor line, so it costs nothing elsewhere.
        self.line: Optional["WarehouseLine"] = None
        self._line_prev_s: Optional[float] = None
        self.mt = MainThreadCalls()

        # Real-hardware link (a HardwareBackend). Attached by main() when the
        # operator opts in (--hardware-backend / --hardware-ip); None keeps the
        # bridge pure-sim. When connected, act_* forward commands to the real
        # arm and tick() mirrors its measured joints back onto the simulated
        # arm (a live digital twin). _hw_suppress_joint stops the inner joint
        # move (from a TCP solve) from double-commanding the arm when the
        # task-space path already forwarded a move_linear.
        self.hw: Optional[HardwareBackend] = None
        self.hw_name: Optional[str] = None
        self.hw_mirror = True
        self._hw_suppress_joint = False
        # Last exception text from a forwarded hardware command, so a failure
        # on the safety path is inspectable rather than only swallowed.
        self.hw_last_error = ""

        # Capabilities surface for /list_robots etc.
        self.capabilities = {
            "joint_names": self.joint_names,
            "joint_limits": [list(lim) for lim in self.joint_limits],
            "home_pose": list(self.home_pose),
            "has_gripper": self.effector is not None,
            "ik_available": cfg.get("ik") is not None,
            # PROTOCOL.md 5.4.1 rule 5: which verbs REFUSE a second command
            # while one is in flight, and which override it instead. Both
            # behaviours are deliberate -- a `stop` that answers 409 is
            # useless -- so neither should have to be guessed. This bridge
            # published NEITHER list until now, while every act_* overwrote
            # the single motion slot unconditionally: `pick` then `place` in
            # one turn aborted the pick mid-sequence and started the place
            # with an empty gripper, and BOTH calls answered accepted:true.
            "busy_rejecting_actions": [
                "pick", "place", "set_tcp_target", "set_tcp_pose",
                "set_joint_positions", "wave", "run_learned_skill",
            ],
            # stop_robot ENDS the motion by definition; reset_to_home is the
            # park/recovery verb an operator reaches for precisely because
            # something is going wrong. Both cancel the occupant instead of
            # refusing, and say so (`superseded` in the reply).
            # servo_joint_positions is the STREAMING lane: it preempts an
            # in-flight goal (named in `preempted`) and a later servo
            # command retargets it in place -- last write wins, never 409.
            "busy_overriding_actions": ["stop_robot", "reset_to_home",
                                        "servo_joint_positions"],
            # The streaming setpoint lane (what a trajectory controller or
            # ros2_control's joint_command should point at). Everything a
            # client needs to know before wiring a stream at it.
            "servo": {
                "verb": "servo_joint_positions",
                "non_blocking": True,
                "last_write_wins": True,
                "preempts": ("an in-flight goal verb is cancelled and named "
                             "in the reply's 'preempted'; a live servo "
                             "stream is retargeted in place with the same "
                             "seq and never answers 409"),
                "goal_verbs_while_streaming": ("a goal verb (e.g. "
                                               "set_joint_positions) sent "
                                               "while the servo stream is "
                                               "live gets the ordinary 409 "
                                               "busy"),
                "done_tolerance_rad": self.SERVO_DONE_RAD,
                "quiet_park_wall_s": self.SERVO_QUIET_S,
                "max_park_wall_s": self.SERVO_PARK_S,
            },
            # The gripper verbs are in neither list ON PURPOSE: they never
            # claim the motion slot (the tick's own sequence steps call
            # act_grasp / act_release / act_open_gripper while a pick is
            # running), so there is nothing for them to reject or override.
            "non_motion_actions": ["open_gripper", "close_gripper",
                                   "set_gripper_width", "grasp", "release"],
            # PROTOCOL.md 5.4.1: which verbs finish before they answer, and
            # which hand back a promise. STATED, not implied -- every arm
            # motion here used to return at DISPATCH time (`moved` was
            # literally `bool(accepted)`), so an agent could not tell a
            # submitted 1.5 s interpolation from a completed one.
            "waitable_actions": list(self.WAITABLE_ACTIONS),
            # Identical list on purpose: every waitable verb DEFAULTS to
            # wait=true on the agent-facing surfaces (tool schema + HTTP), so
            # by default they block until the motion ends and report the
            # MEASURED result. Pass wait=false to get the old promise back.
            "blocking_actions": list(self.WAITABLE_ACTIONS),
            "wait_default": True,
            "wait_max_s": self.WAIT_MAX_S,
            # And the honest other half: verbs that take no `wait`, each with
            # the reason completion is not measurable for it.
            "non_waitable_actions": dict(self.NON_WAITABLE_ACTIONS),
        }
        if self.effector is not None:
            self.capabilities["gripper"] = self.effector.capabilities()
        if cfg.get("ik"):
            self.capabilities["workspace"] = {
                "min_radius": cfg["ik"]["workspace_min_radius"],
                "max_radius": cfg["ik"]["workspace_max_radius"],
                "min_z": cfg["ik"]["workspace_min_z"],
            }

        # Learned skills (skill learning): verb -> validated learned_skill
        # dict ({"kind": "joint_traj", "dt", "traj", "gripper_events",
        # "meta"}). Registered by the LearnManager on a certified learn;
        # replayed through the same clamp/interp motion machinery as every
        # other move. `self.learn` is attached by main() (None when the
        # factory pipeline isn't wired, e.g. in unit tests of other paths).
        self.learned_skills: Dict[str, Dict[str, Any]] = {}
        self.last_learned_verb: Optional[str] = None
        self.learn: Optional[LearnManager] = None

        # Pending wwi outbox -- bridge -> robot window. Each entry is a
        # raw string already prefixed with the protocol tag.
        self.window_outbox: List[str] = []
        # Set a flag the main loop checks each tick to push the configure
        # handshake to the robot window once it opens.
        self.window_configured = False

        # ── DEFERRED INTENTS ─────────────────────────────────────────
        # Every other tool here is IMMEDIATE, so "finish this box then stop"
        # had nowhere to live: the model promised in prose, the turn ended,
        # and the pick loop resumed 60 s later as if nothing had been said.
        # See omnisim_bridges.intents.
        self.intents = (
            IntentStore(
                robot_id,
                task_noun="pick", task_plural="picks",
                conditions=("after_current_task", "after_n_picks",
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

        # Apply home pose immediately so we don't drop under gravity
        # before the first command lands.
        for motor, q in zip(self.motors, self.home_pose):
            if motor is not None:
                motor.setPosition(q)

        # One-time diagnostic: walk the URDF subtree and dump every named
        # Solid's world position. Set OMNILINK_ARM_DUMP_TREE=1 to enable.
        # Used during arm-attachment debugging (gripper hand, etc.)
        # to confirm each link landed where the URDF says it should.
        if _os.environ.get("OMNILINK_ARM_DUMP_TREE") == "1":
            try:
                self._dump_tree()
            except Exception as e:
                print(f"[omnilink_arm_bridge] tree dump failed: {e}")

    def _walk_tree_json(self):
        """Return a flat list of {name, model, world_pos} for every named
        Solid in the robot subtree. Used by the /dump_tree HTTP endpoint
        to confirm link positions stay where they should be over time
        (e.g. that the gripper hand doesn't drift off the wrist flange)."""
        out = []
        self_node = self.robot.getSelf()
        if self_node is None:
            return out

        def walk(node):
            try:
                name = ""
                nf = node.getField("name")
                if nf is not None:
                    name = nf.getSFString()
                if name:
                    try:
                        p = node.getPosition() or [None, None, None]
                    except Exception:
                        p = [None, None, None]
                    out.append({
                        "name": name,
                        "model": node.getTypeName(),
                        "world_pos": [
                            float(p[0]) if p[0] is not None else None,
                            float(p[1]) if p[1] is not None else None,
                            float(p[2]) if p[2] is not None else None,
                        ],
                    })
            except Exception:
                pass
            for fname in ("children", "endPoint"):
                try:
                    fld = node.getField(fname)
                    if fld is None:
                        continue
                    try:
                        n = fld.getCount()
                        if n is not None and n > 0:
                            for i in range(n):
                                c = fld.getMFNode(i)
                                if c is not None:
                                    walk(c)
                            continue
                    except Exception:
                        pass
                    try:
                        c = fld.getSFNode()
                        if c is not None:
                            walk(c)
                    except Exception:
                        pass
                except Exception:
                    pass

        walk(self_node)
        return out

    def _dump_tree(self):
        """Walk the Robot's children tree, log each link's world position."""
        out_path = _os.environ.get("OMNILINK_ARM_DUMP_FILE", "/tmp/arm_tree_dump.log")
        try:
            f = open(out_path, "w", encoding="utf-8", buffering=1)
        except Exception as e:
            print(f"[omnilink_arm_bridge] dump_tree: cannot open {out_path}: {e}")
            return
        self_node = self.robot.getSelf()
        if self_node is None:
            f.write("no self node\n")
            f.close()
            return
        f.write(f"=== tree dump for {self.robot_id} ===\n")

        def walk(node, depth=0):
            try:
                model = node.getTypeName()
            except Exception:
                model = "?"
            name = ""
            try:
                nf = node.getField("name")
                if nf is not None:
                    name = nf.getSFString()
            except Exception:
                pass
            pos = ""
            try:
                p = node.getPosition()
                if p is not None and len(p) >= 3:
                    pos = f"world=({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})"
            except Exception:
                pass
            indent = "  " * depth
            f.write(f"{indent}{model} {name!r} {pos}\n")
            # Try several fields that carry the kinematic chain forward:
            #  - children (MFNode): Solid / Group children
            #  - endPoint (SFNode): HingeJoint / SliderJoint / BallJoint
            for field_name in ("children", "endPoint"):
                try:
                    fld = node.getField(field_name)
                    if fld is None:
                        continue
                    # MFNode: getCount() > 0 (returns -1 for SFNode in some
                    # Webots Python bindings, so use > 0 not >= 0).
                    handled = False
                    try:
                        n = fld.getCount()
                        if n is not None and n > 0:
                            for i in range(n):
                                child = fld.getMFNode(i)
                                if child is not None:
                                    walk(child, depth + 1)
                            handled = True
                    except Exception:
                        pass
                    if not handled:
                        # SFNode (e.g. HingeJoint.endPoint)
                        try:
                            child = fld.getSFNode()
                            if child is not None:
                                walk(child, depth + 1)
                        except Exception:
                            pass
                except Exception as e:
                    f.write(f"{indent}<<error on {field_name}: {e}>>\n")

        walk(self_node, 0)
        f.write("=== end tree dump ===\n")
        f.close()

    # ── Helpers ────────────────────────────────────────────────────

    def _read_q(self) -> List[float]:
        q = []
        for sensor, fallback in zip(self.sensors, self.last_q):
            if sensor is not None:
                try:
                    q.append(sensor.getValue())
                    continue
                except Exception:
                    pass
            q.append(fallback)
        return q

    def _set_q(self, q: List[float]) -> List[float]:
        clamped = clamp_q(q, self.joint_limits)
        for motor, qi in zip(self.motors, clamped):
            if motor is not None:
                motor.setPosition(qi)
        return clamped

    def tcp_xyz(self) -> Optional[Tuple[float, float, float]]:
        ik_cfg = self.cfg.get("ik")
        if not ik_cfg:
            return None
        return forward_kinematics(ik_cfg["chain"], self._read_q(), ik_cfg["tcp_offset"])

    def queue_window(self, line: str) -> None:
        with self.lock:
            self.window_outbox.append(line)

    def note_external_command(self, source: str) -> None:
        """Record an operator command (chat prompt / HTTP tool call). The
        opt-in idle loop pauses INSTANTLY while this is recent and resumes
        after its quiet window. Cheap no-op when no idle loop is running."""
        if time.time() < getattr(self, "_resume_exempt_until", 0.0):
            # A resume landed in this same turn; anything arriving after it
            # must not silently re-arm the pause the operator just lifted.
            # TIME-BOXED on purpose: this used to be a sticky boolean that
            # stayed armed until the next command whenever it came, so a
            # "stop" minutes after a "carry on" was swallowed -- the arm
            # halted, failed to arm the pause, and the idle loop took it
            # straight back over. The exemption only has to outlive the
            # turn that requested it.
            self._resume_exempt_until = 0.0
            return
        self.prev_external_cmd = self.last_external_cmd
        self.last_external_cmd = time.time()
        self.last_external_src = source

    # Tools that only READ. A chat turn that used nothing else was a
    # question, not a command, and must not park the arm.
    READ_ONLY_TOOLS = frozenset({
        "get_robot_state", "list_robots", "capabilities", "get_state",
        "read_joints", "read_tcp_pose", "solve_ik", "node_pose", "dump_tree",
        # Introspection added for the "the tools under-expose what the system
        # knows" gaps: counting the feeder, reading the reach envelope and
        # estimating time left are QUESTIONS. Parking the line for 60 s
        # because someone asked how many parts were left would be the same
        # bug as parking it for asking what the arm is doing.
        "get_line_counts", "get_reach_envelope", "estimate_time_remaining",
    })

    # SCHEDULING tools are grouped with the reads below. Arming the 60 s
    # pause on "stop AFTER you finish this box" would stop the arm NOW --
    # exactly the deferral the operator did not ask for. The intent's own
    # trigger is what pauses it later. hold_until_told is NOT in here: it
    # means "stop now, and stay stopped".
    SCHEDULING_TOOLS = DEFERRED_TOOLS

    # Tools whose whole job is to LIFT the pause. Never re-armed by the
    # bookkeeping below.
    RESUME_TOOLS = frozenset({"resume_autonomy", "resume_idle_loop"})

    def end_chat_turn(self, prev: tuple, actions: list) -> None:
        """Settle the idle-loop pause according to what the turn ACTUALLY did.

        The single decision point for pausing, keyed on the dispatched tool
        rather than on the HTTP route it arrived by -- /prompt and /tool both
        carry reads and commands, so a route-based rule gets it wrong in both
        directions. Three cases: a resume is left alone, a pure read rolls the
        pause back (asking "what are you doing?" used to park the arm for the
        full 60 s window and then report itself paused -- true only because
        asking made it so), and anything that commands motion is force-armed.
        """
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
            # the pause back. A scheduling turn must leave the arm working --
            # that is the entire point of deferring.
            self.last_external_cmd, self.last_external_src = prev
            return
        # Commanding turn: the pause MUST end up armed. Forcing it here
        # (rather than relying on the blanket arm upstream) closes a real
        # hole -- a command landing inside the post-resume exemption window
        # was silently exempted, so the arm executed the order while its
        # idle loop kept running and immediately fought it.
        self._resume_exempt_until = 0.0
        self.last_external_cmd = time.time()
        self.last_external_src = "cmd:" + ",".join(sorted(names))

    def get_state_for_query(self) -> dict:
        """/state as it was the instant the operator ASKED.

        The HTTP layer arms the idle-loop pause before the prompt is even
        routed, so a bare get_state() inside a status answer reports the
        arm as paused by the very question being asked -- true, but true
        only because you asked. This reports the pause as it stood before
        this turn; everything else is live."""
        st = self.get_state()
        loop_obj = getattr(self, "idle_loop", None)
        loop = st.get("idle_loop")
        if isinstance(loop, dict) and loop_obj is not None:
            prev = getattr(self, "prev_external_cmd", 0.0)
            # A HOLD is a standing operator order, not an artefact of the
            # question -- rolling it back would let the model report
            # "running" while the arm is deliberately stopped.
            held = (self.intents is not None and self.intents.hold_active())
            # NEITHER IS A STOP THIS TURN ISSUED. The roll-back exists so
            # "what are you doing?" does not report a pause it caused by
            # being asked; applying it to a stop would let the model read
            # paused=false straight after halting the robot itself.
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

    def _on_intent_pause(self, hold: bool, intent: dict) -> None:
        """A scheduled pause reached its trigger. Called from the pick-loop
        thread at a clean boundary (or from HTTP for hold_until_told). The
        HOLD lives in the store and is read by ArmIdleLoop._blocked; this
        only has to stop motion and, for a soft pause, arm the quiet
        window."""
        try:
            # INTERNAL: a scheduled pause firing, not an operator "stop".
            # The pause bookkeeping is right below (and, for a hold, in the
            # intent store), so act_stop must not layer its own minute-long
            # hold on top or spend a settling window inside the loop thread.
            self.act_stop(source=self.SOURCE_IDLE_LOOP)
        except Exception:
            pass
        if not hold:
            self.last_external_cmd = time.time()
            self.last_external_src = "intent:" + str(intent.get("id", "?"))
        self.queue_window(
            "agent:" + ("Holding as you asked — I will not resume on my own."
                        if hold else "Pausing as you asked."))

    def _on_intent_notify(self, text: str, intent: dict) -> None:
        """A notify intent fired. No push channel to the operator exists, so
        the honest surfaces are the robot window, the idle log and
        /state.notifications -- nothing here pretends a message landed."""
        self.queue_window(f"agent:[{intent.get('id', '?')}] {text}")

    def describe_role(self) -> str:
        """Honest one-paragraph answer to "what is your job here?".

        Built from what this bridge actually is (line master or not,
        which idle loop is configured, live line numbers) rather than a
        fixed blurb, so the offline router can answer the assignment
        question without inventing a role the arm does not have."""
        model = self.cfg.get("model", "arm")
        if not getattr(self, "line_master", False):
            loop = getattr(self, "idle_loop", None)
            if loop is None:
                return (f"I'm a {model} arm on manual control — I have no "
                        f"standing job, I just run the commands you give me.")
            return (f"I'm a {model} arm running an ambient pick loop; "
                    f"I'm not the master of a production line.")
        st = self.get_state()
        line = st.get("line") or {}
        box = line.get("fill_box")
        placed = line.get("placed")
        target = line.get("target")
        where = ("it rides the outfeed spur onto an empty cart, and the two "
                 "MAV tugs haul that cart away and bring the next empty one "
                 "back")
        now = ""
        if box:
            now = (f" Right now I'm kitting {box}"
                   + (f" — {placed} of {target} parts in." if placed is not None
                      and target is not None else "."))
        return (f"I'm the {model} pick cell and the line master here: a belt "
                f"brings empty totes to the fill stop beside me, I pick parts "
                f"off the feeder tray and kit them into the box, and when the "
                f"box is full {where}." + now)

    def act_resume_autonomy(self) -> dict:
        """Hand the arm back to its idle pick loop NOW.

        Without this there is no way to say "carry on" and be believed: the
        prompt asking for it is itself an operator command, so it refreshes
        the pause window and the arm stays parked while the model reports
        that it has resumed."""
        self._resume_exempt_until = time.time() + 1.5
        self.last_external_cmd = 0.0
        self.last_external_src = ""
        # ...and the only thing that lifts an operator STOP early. act_stop
        # arms stop_hold_until for a full minute, so "carry on" has to clear
        # it here or the arm would sit out the rest of the hold while the
        # model reported that it had resumed.
        self.stop_hold_until = 0.0
        # THE ONLY THING THAT LIFTS A HOLD. hold_until_told exists precisely
        # so the quiet window cannot silently override the operator, so
        # clearing the marker above is not enough.
        released = False
        if self.intents is not None:
            released = bool(self.intents.release_hold("resume_autonomy")
                            .get("released"))
        loop = getattr(self, "idle_loop", None)
        if loop is None:
            return {"accepted": True, "autonomy": "none",
                    "hold_released": released,
                    "detail": "this robot has no idle loop to resume"}
        return {"accepted": True, "autonomy": "resumed",
                "hold_released": released,
                "picks": getattr(loop, "picks", 0)}

    # NOTE — conveyor "tread" illusion, MEASURED AND SKIPPED: an earlier
    # revision drifted the visual BELT_TOTE_* props along the belt from this
    # controller. Verdict: NOT cheap. The totes are static Solids, and with
    # newtonStatics TRUE a steady stream of supervisor field writes on
    # statics dragged the whole sim to ~0.3-0.55x realtime (vs 0.999x
    # baseline; RTX 3060 laptop, headless realtime). Skipped per the "only
    # if cheap" rule — the living parts of the idle demo are the arm, the
    # parts, and the tug.

    def _hw_connected(self) -> bool:
        return self.hw is not None and getattr(self.hw, "connected", False)

    def _hw_fwd(self, method: str, *args):
        """Forward a command to the attached real arm, if a hardware backend is
        connected. No-op in pure sim. Errors are swallowed -- the sim stays the
        source of truth for the UI even if the hardware link hiccups.

        ⚠ SWALLOWING IS RIGHT FOR A MIRROR AND WRONG FOR AN E-STOP, so this
        now RETURNS a verdict instead of None and callers on the safety path
        must read it. `True`/`False` is the backend's own answer where it
        gives one (a backend's `stop` reports whether the robot was actually
        told); `None` means "no hardware attached, nothing to forward" --
        which is NOT the same as success and must never be rendered as one.
        Raising instead was considered and rejected: a mirror hiccup taking
        down the sim UI is the failure mode this swallow was added for.
        """
        be = self.hw
        if be is not None and getattr(be, "connected", False):
            try:
                return getattr(be, method)(*args)
            except Exception as e:
                self.hw_last_error = "%s: %r" % (method, e)
                if method == "stop":
                    # The one command whose failure the operator MUST see.
                    try:
                        self.queue_window("error:HARDWARE STOP FAILED (%r) -- "
                                          "USE THE PHYSICAL E-STOP" % (e,))
                    except Exception:
                        pass
                    return False
                return None
        return None

    # ── Tick loop ─────────────────────────────────────────────────

    def tick(self, sim_time_s: float) -> None:
        # Run any supervisor calls the idle loop marshalled to this thread.
        self.mt.pump()
        # Advance the warehouse line (conveyor + load glide + ship/recycle).
        # Sim thread, so its supervisor access is direct; it is fail-soft
        # internally and must never be able to stall the control loop.
        if self.line is not None:
            prev = self._line_prev_s
            self._line_prev_s = sim_time_s
            dt = (sim_time_s - prev) if prev is not None else 0.0
            if 0.0 < dt < 1.0:
                self.line.tick(sim_time_s, dt)
        be = self.hw
        mirror_q = (be.get_joints()
                    if (be is not None and self.hw_mirror
                        and getattr(be, "connected", False))
                    else None)
        with self.lock:
            motion = self.motion
            self.ticks += 1
            self.last_q = self._read_q()
            self.last_tick_at = time.time()
            # JOINT HISTORY for act_stop's rest measurement. Taken here, on
            # the sim thread, so a stop arriving on an HTTP or relay thread
            # can prove the arm stood still without issuing sensor reads of
            # its own (MainThreadCalls: threaded reads stall the step
            # exchange).
            self._q_hist.append((self.last_tick_at, list(self.last_q)))
            if len(self._q_hist) > self.Q_HIST_MAX:
                del self._q_hist[:-self.Q_HIST_MAX]
            if mirror_q is not None and len(mirror_q) == len(self.joint_names):
                # Digital twin: drive the sim motors straight from the real
                # arm's measured joints; the local motion plan is bypassed
                # (the hardware IS the plan while mirroring).
                self._set_q(mirror_q)
                kind = None
            else:
                kind = motion[0]
            params = motion[1]

            if kind == "hold":
                # Re-apply target each tick so motors don't drift.
                self._set_q(params["q"])
                # SETTLE-AND-MEASURE. A motion that just ended parked its plan
                # here rather than being measured on the spot: the final
                # setpoint had only just been written and the position servo
                # has not tracked it yet, so reading the sensors in the same
                # tick would report the arm short of where it ends up. Give it
                # SETTLE_S of sim time holding still, THEN measure.
                pending = params.get("_pending")
                if pending is not None and sim_time_s >= params.get(
                        "_settle_until_s", 0.0):
                    params.pop("_pending", None)
                    self._record_completion(
                        pending, timed_out=bool(params.pop("_pending_timed_out",
                                                           False)))

            elif kind == "interp":
                # Linear interp between params["from_q"] and ["to_q"]
                # over ["duration_s"] starting at ["start_s"].
                t = sim_time_s - params["start_s"]
                d = max(params["duration_s"], 1e-3)
                a = clamp(t / d, 0.0, 1.0)
                # Smooth cubic ease.
                a = a * a * (3.0 - 2.0 * a)
                q = [
                    params["from_q"][j] + (params["to_q"][j] - params["from_q"][j]) * a
                    for j in range(len(self.joint_names))
                ]
                self._set_q(q)
                if a >= 1.0:
                    self._finish_into_hold(params, params["to_q"], sim_time_s)

            elif kind == "servo":
                # Streaming setpoint lane: drive the motors straight at the
                # LATEST target every tick and let the position servos do
                # the smoothing -- the same guarantee _tick_traj leans on:
                # _set_q clamps to the joint limits, and the motors' own
                # velocity caps (set at init from the URDF limits) bound
                # the tracking rate, so a far target is a rate-limited
                # move, never an instantaneous jump. When the stream goes
                # quiet (wall clock -- the stream is an external wall-clock
                # process) and the arm has converged, park into hold so the
                # completion is MEASURED like any other motion; the
                # SERVO_PARK_S fallback frees the slot even if the last
                # target can never be tracked (e.g. an obstructed arm).
                self._set_q(params["to_q"])
                err = max((abs(a - b) for a, b in
                           zip(self.last_q, params["to_q"])), default=0.0)
                quiet_s = time.time() - params.get("last_update_at", 0.0)
                if ((err <= self.SERVO_DONE_RAD
                     and quiet_s >= self.SERVO_QUIET_S)
                        or quiet_s >= self.SERVO_PARK_S):
                    self._finish_into_hold(params, params["to_q"], sim_time_s)

            elif kind == "wave":
                # Oscillate each joint around home with the wave amps.
                t = sim_time_s - params["start_s"]
                amps = self.cfg.get("wave_amplitudes") or [0.0] * len(self.joint_names)
                omega = 2 * math.pi * 0.8  # 0.8 Hz
                q = [
                    self.home_pose[j] + amps[j] * math.sin(omega * t)
                    for j in range(len(self.joint_names))
                ]
                self._set_q(q)
                if t > params["duration_s"]:
                    # `elapsed_s` is what the wave ACTUALLY ran for in sim
                    # seconds -- the measurement `achieved` reports (the
                    # commanded value is duration_s).
                    params["elapsed_s"] = t
                    self._finish_into_hold(params, self.home_pose, sim_time_s)

            elif kind == "sequence":
                # Multi-step plan (pick / place): move / wait / grasp / release.
                self._tick_sequence(params, sim_time_s)

            elif kind == "traj":
                # Learned-skill replay: a fixed-dt joint trajectory from the
                # skill learning. Samples are linearly interpolated and pushed
                # through _set_q, so every point gets the same joint-limit
                # clamp as any other motion, and the motors' own velocity
                # limits (set at init) bound the tracking rate.
                self._tick_traj(params, sim_time_s)

        # Mount the gripper visual on the wrist + animate fingers.
        if self.gripper_visual is not None:
            self._update_gripper_visual()

        # Kinematic-attach: teleport the held object to the TCP each tick.
        # Done outside the motion branches so it tracks under any motion.
        # While the ambient idle loop runs, use the IPC-free FK grasp point
        # (two supervisor pose reads per tick measurably drag the sim on
        # slower machines); interactive use keeps the anchor-accurate path.
        if self.held_tfield is not None:
            tcp = (self._tcp_world_fast() if self.idle_loop is not None
                   else self._tcp_world())
            if tcp is not None:
                try:
                    self.held_tfield.setSFVec3f(tcp)
                    self.held_node.resetPhysics()
                except Exception:
                    pass


    def _tick_sequence(self, params: dict, sim_time_s: float) -> None:
        """Advance a multi-step plan (built by act_pick / act_place). Steps:
        {"t":"move","to_q":[...],"dur":s} | {"t":"wait","dur":s}
        | {"t":"grasp"} | {"t":"release"}. Runs under self.lock (tick holds
        it); grasp/release reuse the bridge handlers (reentrant lock)."""
        steps = params["steps"]
        i = params["i"]
        n = len(self.joint_names)
        if i >= len(steps):
            self._finish_into_hold(params, self._read_q(), sim_time_s)
            return
        step = steps[i]
        st = step.get("t")
        if st == "cmove" and "to_q" not in step:
            # Solve top-down IK fresh at execution time, seeded from the current
            # pose -- the path the proven flagship poses take, so the descent
            # stays vertical and the fingers straddle the cube squarely (a
            # precomputed solution lands a slightly tilted wrist and misses).
            qn = self._topdown_q(step["xyz"], seed=self._read_q(),
                                 tcp_offset_z=step.get("oz"))
            step["to_q"] = qn if qn is not None else list(params["from_q"])
        if st in ("move", "cmove"):
            if not step.get("_minrot"):
                # MINIMUM ROTATION, once, as the segment is adopted. Belt and
                # braces behind the same guard in the IK (see wrap_q_near): a
                # sequence target that did NOT come from the solver -- home_pose
                # is the one that matters, it closes every idle pick cycle --
                # can still be several turns from where the arm is standing, and
                # this segment is interpolated at a FIXED duration, so the
                # difference is a motor-rate saturation, not a slower move.
                step["to_q"] = wrap_q_near(step["to_q"], self.joint_limits,
                                           params["from_q"])
                step["_minrot"] = True
            t = sim_time_s - params["start_s"]
            d = max(float(step.get("dur", 1.2)), 1e-3)
            a = clamp(t / d, 0.0, 1.0)
            a = a * a * (3.0 - 2.0 * a)         # smooth cubic ease
            frm = params["from_q"]
            to = step["to_q"]
            self._set_q([frm[j] + (to[j] - frm[j]) * a for j in range(n)])
            if a >= 1.0:
                params["i"] = i + 1
                params["from_q"] = list(to)
                params["start_s"] = sim_time_s
        elif st == "wait":
            self._set_q(params["from_q"])
            if sim_time_s - params["start_s"] >= float(step.get("dur", 0.3)):
                params["i"] = i + 1
                params["start_s"] = sim_time_s
        elif st in ("grasp", "release", "open"):
            self._set_q(params["from_q"])
            if st == "grasp":
                self.act_grasp()
            elif st == "release":
                self.act_release()
            else:                            # "open": spread fingers (physics)
                self.act_open_gripper()
            params["i"] = i + 1
            params["start_s"] = sim_time_s
        else:
            params["i"] = i + 1

    def _tick_traj(self, params: dict, sim_time_s: float) -> None:
        """Advance a learned joint-trajectory replay. Runs under self.lock
        (tick holds it). Gripper events fire once when their timestamp is
        crossed; with no effector configured they are skipped with one log
        line (set at registration time)."""
        t = sim_time_s - params["start_s"]
        traj = params["traj"]
        dt = max(float(params["dt"]), 1e-4)
        n = len(self.joint_names)
        # Fire due gripper events (open/close) exactly once each.
        for gev in params["gevents"]:
            if not gev["fired"] and t >= gev["t"]:
                gev["fired"] = True
                if self.effector is not None:
                    if gev["op"] == "open":
                        self.act_open_gripper()
                    else:
                        self.act_close_gripper()
                else:
                    print(f"[omnilink_arm_bridge] learned skill "
                          f"'{params.get('verb', '?')}': gripper event "
                          f"'{gev['op']}' at t={gev['t']:.2f}s skipped "
                          "(this arm has no gripper)")
        x = t / dt
        idx = int(x)
        if idx >= len(traj) - 1:
            q_end = list(traj[-1])[:n]
            self._set_q(q_end)
            # Late gripper events (timestamped past the trajectory end)
            # still fire before we settle into hold.
            pending = [g for g in params["gevents"] if not g["fired"]]
            if not pending:
                self._finish_into_hold(
                    params, clamp_q(q_end, self.joint_limits), sim_time_s)
            return
        a = clamp(x - idx, 0.0, 1.0)
        q0, q1 = traj[idx], traj[idx + 1]
        q = [q0[j] + (q1[j] - q0[j]) * a for j in range(min(n, len(q0)))]
        self._set_q(q)

    # ── Learned skills (skill learning) ────────────────────────────

    def register_learned_skill(self, verb: str, skill: Dict[str, Any]) -> None:
        """Make a certified learned skill invocable as a verb (chat +
        POST /<verb>). Trajectory rows are normalised to the arm's joint
        count here, once, so replay never mixes row widths."""
        n = len(self.joint_names)
        traj = []
        padded = False
        for row in skill["traj"]:
            r = [float(v) for v in row[:n]]
            if len(r) < n:
                r = r + list(self.home_pose[len(r):n])
                padded = True
            traj.append(r)
        if padded or any(len(row) != n for row in skill["traj"]):
            print(f"[omnilink_arm_bridge] learned skill '{verb}': trajectory "
                  f"rows adjusted to this arm's {n} joints")
        skill = dict(skill)
        skill["traj"] = traj
        with self.lock:
            self.learned_skills[verb] = skill
            self.last_learned_verb = verb
        print(f"[omnilink_arm_bridge] learned skill registered: '{verb}' "
              f"({len(traj)} samples @ {skill['dt']}s, "
              f"{len(skill.get('gripper_events') or [])} gripper events)")

    def _missing_props(self, skill: Dict[str, Any]) -> List[str]:
        """Which of the skill's expected props (meta.props / prop_hints)
        are absent from this world. Best-effort DEF lookups; a prop counts
        as present if any plausible DEF spelling resolves."""
        props = (skill.get("meta") or {}).get("props") \
            or (skill.get("meta") or {}).get("prop_hints") or []
        if isinstance(props, str):
            props = [props]
        missing = []
        for p in props:
            name = str(p)
            found = False
            for cand in (name, name.upper(), "GRASP_" + name.upper(),
                         "GRASP_" + name):
                try:
                    if self.robot.getFromDef(cand) is not None:
                        found = True
                        break
                except Exception:
                    pass
            if not found:
                missing.append(name)
        return missing

    def act_run_learned(self, verb: str, wait: bool = False) -> dict:
        """Replay a learned skill's joint trajectory on this arm. Goes
        through the same motion machinery (motion plan consumed by tick(),
        joint-limit clamp in _set_q, motor velocity caps) as every
        built-in move."""
        with self.lock:
            skill = self.learned_skills.get(verb)
        if skill is None:
            known = sorted(self.learned_skills.keys())
            return {"error": "unknown_verb", "verb": verb,
                    "learned_verbs": known}
        missing = self._missing_props(skill)
        duration = float(skill["dt"]) * max(0, len(skill["traj"]) - 1)
        gevents = [{"t": float(t), "op": op, "fired": False}
                   for (t, op) in (skill.get("gripper_events") or [])]
        if gevents and self.effector is None:
            print(f"[omnilink_arm_bridge] learned skill '{verb}' has "
                  f"{len(gevents)} gripper events but this arm has no "
                  "gripper — they will be ignored")
        # PROTOCOL.md 5.4.1 rule 5. A learned replay carries its own gripper
        # events, so clobbering one mid-flight leaves the gripper in a state
        # nobody commanded. (`verb` is already this params dict's own key --
        # the learned verb -- and _begin_motion reports exactly that.)
        seq, refusal = self._begin_motion("run_learned_skill")
        if seq is None:
            return refusal
        n = len(self.joint_names)
        final_q = [float(v) for v in clamp_q(list(skill["traj"][-1])[:n],
                                             self.joint_limits)]
        with self.lock:
            self.motion = ("traj", dict(
                self._slot(seq, verb,
                           measure={"kind": "traj", "unit": "rad",
                                    # A replay's commanded end state is its
                                    # last trajectory sample; the measurement
                                    # is where the joints actually finished.
                                    "commanded": final_q,
                                    "samples": len(skill["traj"])}),
                traj=skill["traj"],
                dt=float(skill["dt"]),
                gevents=gevents,
                start_s=self.robot.getTime(),
            ))
        out = self._wait_result(seq, verb, wait,
                                budget_s=duration * 2.0 + 8.0,
                                eta_s=duration, commanded=final_q, unit="rad")
        out["duration_s"] = duration
        out["samples"] = len(skill["traj"])
        out["joint_names"] = list(self.joint_names)
        if missing:
            out["missing_props"] = missing
            out["missing_props_note"] = (
                "motion expects props not found in this world: "
                + ", ".join(missing) + " — running it anyway")
        return out

    def act_learn(self, text: str,
                  params: Optional[Dict[str, Any]] = None) -> dict:
        """Kick off a skill learning learn from free text or a recipe id.
        Returns {"started": bool, "message": <chat line>, ...}."""
        lm = self.learn
        if lm is None:
            return {"started": False, "error": "learn_unavailable",
                    "message": ("The skill learning pipeline isn't wired on "
                                "this bridge, so I can't learn new skills "
                                "here.")}
        recipe = text if text in lm.recipes else lm.resolve_recipe(text)
        if recipe is None:
            return {"started": False, "error": "unknown_recipe",
                    "message": ("I don't have a recipe for that yet. "
                                + lm.learnable_summary() + ".")}
        ok, msg = lm.start(recipe, params=params)
        return {"started": ok, "recipe": recipe, "message": msg,
                **({} if ok else {"error": "learn_rejected"})}

    def _find_mount_node(self):
        """Find the tool-mount link node in the URDF subtree by name. The
        gripper visual rides this node's exact world pose, so it sits on
        the real flange instead of an approximate FK point."""
        cands = self.cfg.get("mount_link") or ["flange", "tool0", "gripper_tcp", "tcp"]
        if isinstance(cands, str):
            cands = [cands]
        found: Dict[str, Any] = {}
        self_node = self.robot.getSelf()
        if self_node is None:
            return None

        def walk(node):
            try:
                nf = node.getField("name")
                if nf is not None:
                    nm = nf.getSFString()
                    if nm in cands and nm not in found:
                        found[nm] = node
            except Exception:
                pass
            for fn in ("children", "endPoint"):
                f = node.getField(fn)
                if f is None:
                    continue
                try:
                    c = f.getCount()
                    if c and c > 0:
                        for i in range(c):
                            ch = f.getMFNode(i)
                            if ch is not None:
                                walk(ch)
                        continue
                except Exception:
                    pass
                try:
                    ch = f.getSFNode()
                    if ch is not None:
                        walk(ch)
                except Exception:
                    pass

        walk(self_node)
        for c in cands:
            if c in found:
                return found[c]
        return None

    def _update_gripper_visual(self) -> None:
        """Ride the real mount-link pose each tick (position + orientation
        straight from the scene tree) and set finger spacing from width."""
        node = self.gripper_anchor
        if node is None:
            return
        try:
            p = node.getPosition()
            o = node.getOrientation()  # flat 9, row-major
            r = [[o[0], o[1], o[2]], [o[3], o[4], o[5]], [o[6], o[7], o[8]]]
            tf = self.gripper_visual.getField("translation")
            rf = self.gripper_visual.getField("rotation")
            if tf is not None:
                tf.setSFVec3f([p[0], p[1], p[2]])
            if rf is not None:
                rf.setSFRotation(_mat3_to_axis_angle(r))
        except Exception:
            return
        # Finger spacing from width (half-stroke each side).
        if self.effector is None:
            return
        w = self.effector.state().get("width")
        if w is None:
            return
        half = clamp(w * 0.5, 0.0, 0.05)
        fl, fr = self.gripper_fingers
        for node, sign in ((fl, 1.0), (fr, -1.0)):
            if node is None:
                continue
            try:
                t = node.getField("translation")
                v = t.getSFVec3f()
                t.setSFVec3f([sign * half, v[1], v[2]])
            except Exception:
                pass

    # ── The single motion slot, and REFUSING to clobber it ────────
    #
    # PROTOCOL.md 5.4.1 rule 5: "a command arriving while busy is REJECTED,
    # never silently clobbered".
    #
    # MEASURED FAILURE THIS FIXES: every act_* used to do a bare
    # `with self.lock: self.motion = (...)`, so a model that called `pick`
    # and then `place` in the same turn -- the obvious two-step for "move
    # that part into the box" -- aborted the pick partway through its
    # sequence and started the place with an empty gripper. BOTH calls
    # answered `accepted: true`, so the only account of the run the model
    # ever saw said it had worked. That is the exact failure rule 5 exists
    # to stop, and it is the same one the mobile bridge already fixed in
    # MobileBridge._begin_motion; this is that guard, ported.
    #
    # The kinds below are the ones tick() is still playing out. "hold" is
    # not one: it IS the free slot (tick() drops every finished plan back
    # into hold), so the ordinary command-then-command-later flow is
    # untouched. "servo" is busy TO THE GOAL VERBS ONLY: a goal verb
    # (set_joint_positions et al.) arriving while a servo stream is live
    # gets the ordinary 409, but a servo command NEVER 409s -- it retargets
    # the live stream in place (see act_servo_joint_positions).
    BUSY_MOTION_KINDS = ("interp", "wave", "sequence", "traj", "servo")

    # ── servo_joint_positions: the streaming setpoint lane ────────
    # A trajectory controller (MoveIt's joint_trajectory_controller, any
    # streaming teleop) writes a setpoint every cycle. Under the goal
    # contract above its stream lands IN PIECES: measured on this bridge,
    # a second set_joint_positions 50 ms after the first answered 409 and
    # was not applied. The servo lane is last-write-wins by design.
    # When the target has converged and the stream has gone quiet for
    # SERVO_QUIET_S (WALL seconds -- the stream is an external wall-clock
    # process, and under --mode=fast sim time runs ~13x faster, so a sim
    # clock here would park a healthy 20 Hz stream between its own
    # setpoints), the plan parks into hold and tick() measures it like any
    # other motion, so get_robot_state.last_command reports what the
    # stream ACTUALLY achieved. SERVO_PARK_S is the not-converging
    # fallback so a stalled stream can never hold the motion slot forever.
    SERVO_DONE_RAD = 0.02       # max per-joint |sensor - target| to count as converged
    SERVO_QUIET_S = 0.5         # wall s without a new setpoint before a converged park
    SERVO_PARK_S = 5.0          # wall s without a new setpoint: park regardless

    # Who started the motion in the slot. An operator/agent motion is
    # INVIOLABLE until it finishes -- that is the guarantee the 409 makes.
    # The ambient pick loop's motions YIELD: an operator command has always
    # taken the arm over instantly (note_external_command parks the loop for
    # its quiet window), and turning that into a 409 would make the whole
    # warehouse demo uncommandable while the loop is mid-cycle.
    SOURCE_EXTERNAL = "external"
    SOURCE_IDLE_LOOP = "idle_loop"

    # ── The completion half of the contract (PROTOCOL.md 5.4.1) ───
    #
    # THE DEFECT THIS CLOSES (D2): every arm motion verb returned at
    # DISPATCH time. act_set_tcp_target answered {"accepted": true,
    # "moved": true, "err_norm": 0.0002} in about three milliseconds while
    # the motion it had just queued was a 1.5 s interpolation that tick()
    # consumed afterwards -- and `err_norm` there was the IK SOLVER's own
    # residual, computed before anything moved, sitting in the payload
    # shaped exactly like a measured TCP error. `moved` was literally
    # `bool(accepted)`: it asserted motion at submission. An agent has no
    # independent access to the world, so nothing in its experience
    # distinguishes that reply from a completed move; it reports success,
    # and a multi-step task compounds the error step by step.
    #
    # The fix is the one the mobile bridge already ships: a monotonic seq
    # on every motion, a `last_completion` record the TICK writes when the
    # motion actually ENDS, and a `wait` that blocks until it lands.
    WAIT_POLL_S = 0.02          # how often a waiting caller re-checks
    # Ceiling on any `wait: true` call, deliberately BELOW the edge
    # connector's 55 s per-tool timeout and the platform's 60 s
    # PER_TOOL_TIMEOUT_MS -- blocking past those would hand the model an
    # opaque transport error instead of a structured "still moving, go
    # poll" note. Arm motions are 0.8-8 s, so this is pure headroom.
    WAIT_MAX_S = 50.0
    # A completion can only arrive if tick() is running. If it has been
    # silent this long (wall seconds) nothing is stepping the sim, so a
    # waiter would block for its whole budget and then report a timeout
    # that says nothing about the arm. Says the real reason instead.
    WAIT_STALL_S = 2.0
    # Sim seconds a finished plan holds still before it is measured. The
    # last setpoint is written in the same tick the interpolation ends, so
    # the position servo has not tracked it yet; measuring immediately
    # would report the arm short of where it actually settles.
    SETTLE_S = 0.15

    # ── stop_robot: measured rest + operator hold ─────────────────
    # How long a stop watches the JOINTS before it will say the arm is at
    # rest. Long enough to span several basic timesteps, short enough that
    # the operator's "stop" still answers inside a chat turn.
    STOP_SETTLE_S = 0.30
    # The residual-rate measurement is taken over the TAIL of that window so
    # the servos' decel ramp is not averaged into the answer.
    STOP_TAIL_FRAC = 0.5
    # Minimum sample span before a rate is believable. Below this the answer
    # is `null` and says why -- never a number nobody measured.
    STOP_MIN_SPAN_S = 0.04
    # "Stationary" threshold, rad/s on the fastest joint. Position-controlled
    # servos holding a setpoint sit well under this.
    STOP_STILL_RADPS = 0.05
    # HOW LONG AN OPERATOR STOP HOLDS THE IDLE LOOP, armed at EXECUTION time.
    # Deliberately NOT --idle-resume-s: that window is armed when the prompt
    # arrives and is sized for chat turns in general (this world runs it at
    # 12 s so a question does not park the line). A STOP is the one command
    # whose entire meaning is "stay stopped". It still AUTO-RESUMES --
    # hold_until_told is the tool for an indefinite hold, and an arm that
    # never resumes strands the line.
    STOP_HOLD_S = 60.0
    # Joint samples kept for the measurement above (~8 s at 32 ms).
    Q_HIST_MAX = 256

    # Verbs that accept `wait` -- i.e. the ones whose completion this
    # bridge can honestly MEASURE. Published in capabilities.
    WAITABLE_ACTIONS = (
        "set_tcp_target", "set_tcp_pose", "set_joint_positions",
        "pick", "place", "reset_to_home", "wave", "run_learned_skill",
    )
    # And the ones that do NOT, each with the reason. A verb missing from
    # both lists would just be a silent gap.
    NON_WAITABLE_ACTIONS = {
        "stop_robot": ("it IS the completion -- it freezes the arm and "
                       "returns the joint angles measured at that instant."),
        "open_gripper": ("no finger position sensor is exposed through the "
                         "effector API, so there is nothing to wait for and "
                         "nothing to measure; the reply already reports "
                         "commanded_state with achieved_state null. It also "
                         "returns the DEF of any object it dropped, which IS "
                         "measured."),
        "close_gripper": "same as open_gripper: no finger readback exists.",
        "set_gripper_width": ("same as open_gripper: the effector reports the "
                              "width it was last commanded, not a sensor."),
        "grasp": ("returns synchronously and already carries the MEASURED "
                  "outcome (`attached`: the DEF actually welded, or null). It "
                  "is also called from inside the tick's own sequence steps, "
                  "so blocking it would deadlock the step loop."),
        "release": "same as grasp -- `released` is the measured DEF.",
        "learn_skill": ("starts a long external pipeline whose progress is "
                        "streamed over /factory/events; it is asynchronous by "
                        "design and says so."),
        "solve_ik": "a solver preview -- it commands no motion at all.",
        "servo_joint_positions": (
            "non-blocking BY DESIGN: it returns at dispatch so a trajectory "
            "stream can send the next setpoint on schedule. achieved is "
            "always null in the reply -- nothing has been measured when it "
            "answers. The measured result lands in "
            "get_robot_state.last_command once the stream goes quiet and "
            "the arm parks (matched by seq, like any other motion)."),
    }

    def _fk_tcp_base(self, q=None, tcp_offset_z=None):
        """MEASURED TCP position in the ARM BASE frame, in metres, from the
        joint sensors -- the same forward kinematics the tcp_xyz() readback
        uses. `tcp_offset_z` must match the offset the motion was planned
        with (a grasp plans to the finger throat, not the flange), or the
        measurement is of a different point than the command."""
        ik = self.cfg.get("ik")
        if not ik:
            return None
        off = list(ik["tcp_offset"])
        if tcp_offset_z is not None:
            off[2] = float(tcp_offset_z)
        try:
            p = forward_kinematics(ik["chain"],
                                   self._read_q() if q is None else list(q),
                                   tuple(off))
        except Exception:
            return None
        return [float(v) for v in p]

    def _finish_into_hold(self, p: dict, q, sim_time_s: float,
                          timed_out: bool = False) -> None:
        """Drop a finished plan back into the free `hold` slot, carrying its
        measurement forward so the tick can take it after SETTLE_S.

        Called under self.lock from tick(). A plan with no `seq` -- which is
        every plan ArmIdleLoop._run_steps writes -- lands in a plain hold,
        byte-identical to what tick() did before any of this existed: nothing
        waits on an ambient loop motion and nothing measures one."""
        hold = {"q": list(q)}
        if p.get("seq"):
            hold["_pending"] = p
            hold["_pending_timed_out"] = bool(timed_out)
            hold["_settle_until_s"] = sim_time_s + self.SETTLE_S
        self.motion = ("hold", hold)

    def _flush_pending(self) -> None:
        """Measure a completion still sitting out its settle window.

        A new command can land in the SETTLE_S gap between a motion ending
        and the tick measuring it (`hold` is the free slot, so nothing
        refuses it). Without this the record would be dropped on the floor
        and its waiter would spin to the budget. The motion really did
        finish; measure it NOW, from wherever the arm actually is."""
        with self.lock:
            kind, p = self.motion
            if kind != "hold":
                return
            pend = p.pop("_pending", None)
            timed_out = bool(p.pop("_pending_timed_out", False))
        if pend is not None:
            self._record_completion(pend, timed_out=timed_out,
                                    note="measured early: a new command "
                                         "arrived during the settle window")

    def _measure_motion(self, p: dict) -> dict:
        """What the finished motion ACTUALLY did, read from sensors and the
        scene -- never the argument echoed back.

        Returns the `commanded` / `achieved` / `error` / `unit` block plus
        whatever extra measurements the verb has. Fail-soft: anything that
        cannot be measured comes back None with a note, because a number
        nobody measured is the failure this whole layer exists to kill."""
        m = p.get("measure") or {}
        kind = m.get("kind")
        out: Dict[str, Any] = {"commanded": m.get("commanded"),
                               "achieved": None, "error": None,
                               "unit": m.get("unit", "")}
        try:
            if kind == "joints":
                cmd = [float(v) for v in m["commanded"]]
                got = [float(v) for v in self._read_q()][:len(cmd)]
                out["commanded"] = cmd
                out["achieved"] = got
                out["error"] = [g - c for g, c in zip(got, cmd)]
                out["max_abs_error_rad"] = max(
                    (abs(e) for e in out["error"]), default=0.0)
                out["joint_names"] = list(self.joint_names)
                out["frame"] = "joint space (radians)"
            elif kind == "tcp":
                cmd = [float(v) for v in m["commanded"]]
                got = self._fk_tcp_base(tcp_offset_z=m.get("tcp_offset_z"))
                out["commanded"] = cmd
                out["achieved"] = got
                if got is not None:
                    out["error"] = [g - c for g, c in zip(got, cmd)]
                    out["error_m"] = math.sqrt(
                        sum(e * e for e in out["error"]))
                out["frame"] = ("arm base (the arm's own origin), metres -- "
                                "the frame the IK chain works in")
                out["achieved_q"] = [float(v) for v in self._read_q()]
            elif kind == "wave":
                out["commanded"] = float(m.get("commanded", 0.0))
                got = p.get("elapsed_s")
                out["achieved"] = None if got is None else float(got)
                if out["achieved"] is not None:
                    out["error"] = out["achieved"] - out["commanded"]
                out["achieved_q"] = [float(v) for v in self._read_q()]
                out["frame"] = "simulated seconds of oscillation"
            elif kind in ("pick", "place"):
                cmd = [float(v) for v in m["commanded"]]
                out["commanded"] = cmd
                out["frame"] = m.get("frame", "arm base, metres")
                tcp = self._fk_tcp_base(tcp_offset_z=m.get("tcp_offset_z"))
                out["achieved_tcp_base_xyz"] = tcp
                held = self.held_node
                try:
                    out["holding"] = held.getDef() if held is not None else None
                except Exception:
                    out["holding"] = None
                if out.get("holding") is None:
                    # ⚠ NO-WELD PATH. `held_node` is set only by the weld, so
                    # without this a friction grasp reports holding: null for
                    # ever and the model reads its own success as a miss (see
                    # _measure_hold for the measurement that proved it). The
                    # weld path never reaches here -- holding is already set.
                    hold = self._measure_hold(m.get("target_field"),
                                              part_def=m.get("target"))
                    if hold is not None:
                        out["hold"] = hold
                        if hold.get("holding"):
                            out["holding"] = hold.get("part_def") or True
                if kind == "pick":
                    # The pick's own question is "is the part in the gripper",
                    # and the achieved value is where the TOOL ended up.
                    out["achieved"] = tcp
                    if tcp is not None:
                        out["error"] = [g - c for g, c in zip(tcp, cmd)]
                        out["error_m"] = math.sqrt(
                            sum(e * e for e in out["error"]))
                    out["target"] = m.get("target")
                    out["target_world_xyz"] = self._node_world_xyz(
                        m.get("target_field"))
                else:
                    # The place's question is where the PART landed, so the
                    # achieved value is the part's own measured world
                    # position, not the tool's. Null when nothing was being
                    # carried (there is then no part to measure).
                    out["achieved"] = self._node_world_xyz(
                        m.get("target_field"))
                    if out["achieved"] is not None:
                        out["error"] = [g - c for g, c in
                                        zip(out["achieved"], cmd)]
                        out["error_m"] = math.sqrt(
                            sum(e * e for e in out["error"]))
                    else:
                        out["note_achieved"] = (
                            "the arm was not carrying a tracked part, so "
                            "there is no placed object to measure -- read "
                            "achieved_tcp_base_xyz for where the tool went")
                    out["released"] = m.get("target")
            elif kind == "traj":
                cmd = [float(v) for v in m["commanded"]]
                got = [float(v) for v in self._read_q()][:len(cmd)]
                out["commanded"] = cmd
                out["achieved"] = got
                out["error"] = [g - c for g, c in zip(got, cmd)]
                out["max_abs_error_rad"] = max(
                    (abs(e) for e in out["error"]), default=0.0)
                out["joint_names"] = list(self.joint_names)
                out["frame"] = "joint space (radians)"
                out["samples"] = m.get("samples")
        except Exception as e:                       # never break the tick
            out["achieved"] = None
            out["error"] = None
            out["measure_error"] = repr(e)
        return out

    def _node_world_xyz(self, tfield):
        """Live world translation of a scene node's translation field, or
        None. Measured, never remembered."""
        if tfield is None:
            return None
        try:
            return [round(float(v), 4) for v in tfield.getSFVec3f()]
        except Exception:
            return None

    def _record_completion(self, p: dict, timed_out: bool = False,
                           note: str = "") -> None:
        """The ONLY place a motion result is written, and it writes what was
        MEASURED. Called from the tick when a motion ends (or from
        _flush_pending when a new command lands inside the settle window)."""
        seq = p.get("seq")
        if not seq:
            # An idle-loop plan. Nothing claimed it and nothing waits on it,
            # so publishing a completion for it would put the ambient loop's
            # own motion in the slot an operator command polls.
            return
        rec: Dict[str, Any] = {
            "seq": int(seq),
            "verb": p.get("verb", "?"),
            "settled": not timed_out,
            "timed_out": bool(timed_out),
            "superseded": False,
        }
        rec.update(self._measure_motion(p))
        try:
            rec["sim_time"] = self.robot.getTime()
        except Exception:
            rec["sim_time"] = None
        if note:
            rec["note"] = note
        self.last_completion = rec

    def _record_superseded(self, p: dict, by_seq, reason: str = "") -> None:
        """A motion ended before it could be measured. Say exactly that.

        `achieved` is null -- never a number nobody measured, and never the
        commanded value standing in for one."""
        if not p.get("seq"):
            return
        m = p.get("measure") or {}
        self.last_completion = {
            "seq": int(p["seq"]),
            "verb": p.get("verb", "?"),
            "commanded": m.get("commanded"),
            "achieved": None,
            "error": None,
            "unit": m.get("unit", ""),
            "settled": False,
            "timed_out": False,
            "superseded": True,
            "superseded_by": by_seq,
            "note": (reason or ("a later command ended this motion before it "
                                "finished; how far it got was not measured")),
        }

    def _await_completion(self, seq: int, budget_s: float) -> dict:
        """Block until motion `seq` reports in, or the budget expires.

        Polling rather than a condition variable: the tick already holds
        `self.lock` on every step, and an HTTP thread waiting on that same
        lock is a deadlock waiting for a slow world to find it."""
        deadline = time.time() + min(max(budget_s, 0.5), self.WAIT_MAX_S)
        while time.time() < deadline:
            done = self.last_completion
            # EXACTLY this motion's measurement, never a later one's. A
            # `>= seq` test hands a clobbered motion's waiter the CLOBBERING
            # motion's result: measured, internally consistent, and about a
            # completely different command.
            if done is not None and done.get("seq") == seq:
                return dict(done)
            with self.lock:
                latest = self.motion_seq
            if latest > seq:
                return {"seq": seq, "achieved": None, "error": None,
                        "settled": False, "timed_out": False,
                        "superseded": True,
                        "note": ("a later command superseded this motion "
                                 "before it reported; how far it got was not "
                                 "measured -- reissue it, or read "
                                 "get_robot_state.q for the live pose")}
            # NOTHING IS STEPPING THE SIM. A completion is written by tick(),
            # so with no tick there is nothing to wait for: report the real
            # reason instead of sitting out the whole budget and then blaming
            # the arm for a timeout it had no part in.
            if time.time() - self.last_tick_at > self.WAIT_STALL_S:
                return {"seq": seq, "achieved": None, "error": None,
                        "settled": False, "timed_out": False,
                        "measurable": False,
                        "note": ("the simulation is not stepping (no tick for "
                                 f"{self.WAIT_STALL_S:.0f}s), so no completion "
                                 "can be recorded; the command WAS accepted "
                                 "and will play out when the world resumes")}
            time.sleep(self.WAIT_POLL_S)
        # The caller asked us to wait and we could not confirm. Say exactly
        # that -- never fall back to reporting the commanded value.
        return {"seq": seq, "achieved": None, "error": None,
                "settled": False, "timed_out": True,
                "note": ("wait budget expired before the motion reported; the "
                         "arm may still be moving -- poll get_robot_state and "
                         "match last_command.seq to this seq")}

    def _wait_result(self, seq, verb: str, wait: bool, budget_s: float,
                     eta_s: float, commanded, unit: str) -> dict:
        """The tail every waitable motion verb shares: block for the measured
        result, or hand back an honestly-labelled promise."""
        head = {"accepted": True, "seq": seq, "verb": verb,
                "commanded": commanded, "unit": unit}
        # A MIRRORING HARDWARE LINK MAKES COMPLETION UNOBSERVABLE HERE.
        # tick() drives the sim motors straight from the real arm's measured
        # joints and never touches self.motion (see the mirror_q branch), so
        # no plan ever ends and no completion is ever recorded. Blocking would
        # burn the whole budget and then report a timeout that is about this
        # bridge, not about the arm.
        if self._hw_connected() and self.hw_mirror:
            head.update({
                "achieved": None, "error": None, "settled": False,
                "timed_out": False, "measurable": False,
                "note": ("a real arm is attached and mirroring, so this "
                         "bridge does not execute the motion and cannot time "
                         "its completion: the command was forwarded to the "
                         "vendor controller, which owns the trajectory. Read "
                         "get_robot_state.q -- that IS the hardware's "
                         "measured joint vector, mirrored every tick."),
            })
            return head
        if not wait:
            head.update({
                "achieved": None, "eta_s": round(float(eta_s), 3),
                "note": ("NOT complete -- this returned on acceptance, before "
                         "the arm moved. Pass wait=true, or poll "
                         "get_robot_state until last_command.seq == this seq, "
                         "for the achieved value. Match the seq EXACTLY: a "
                         "later command's record measures a different motion, "
                         "and a superseded motion reports achieved: null."),
            })
            return head
        head.update(self._await_completion(seq, budget_s))
        return head

    def _begin_motion(self, verb: str, override: bool = False):
        """Claim the motion slot for `verb`, on behalf of an external caller.

        (The idle loop does not come through here -- it writes the slot
        directly in ArmIdleLoop._run_steps, stamped SOURCE_IDLE_LOOP, and has
        its own back-off via _blocked().)

        Returns ``(seq, extra)``. ``seq is None`` means REFUSED and `extra` is
        the 409 envelope to hand straight back to the caller. Otherwise `seq`
        is the new slot id and `extra` is the params of a motion this call
        cancelled (only possible for an overriding verb or an idle-loop
        occupant), or None when the slot was already free.

        `override` is what makes capabilities.busy_overriding_actions TRUE IN
        CODE. It was a comment before: stop_robot and reset_to_home called
        this like everybody else, got the 409 envelope back in the
        `superseded` slot, ignored it and stopped the arm anyway -- so the
        behaviour was right, the bookkeeping was not. With a seq now carrying
        the measurement, `seq is None` is no longer harmless: a reset_to_home
        issued while a pick was running would have written a plan with no seq,
        published no completion, and left a `wait: true` caller blocking on a
        motion that could never report.

        THE BUSY CHECK LIVES HERE, not in the HTTP route -- the route is only
        one of four ways to reach a motion (POST /<verb>, the tool dispatch an
        OmniLink agent actually uses, the offline intent router, and the idle
        loop all call act_* directly).
        """
        # A motion that ended microseconds ago may still be sitting out its
        # settle window in the hold slot. Measure it before claiming, or its
        # record is lost and its waiter spins to the budget.
        self._flush_pending()
        with self.lock:
            kind, p = self.motion
            busy = kind in self.BUSY_MOTION_KINDS
            if busy and self._hw_connected() and self.hw_mirror:
                # A mirroring hardware link BYPASSES the local plan entirely
                # (see tick(): mirror_q wins and `kind` is set to None), so
                # the slot is not driving anything and the real arm's own
                # controller is what serialises motions. Reporting "busy" off
                # an inert plan would be a state nobody is in.
                busy = False
            if (busy and not override
                    and p.get("source", self.SOURCE_EXTERNAL)
                    == self.SOURCE_EXTERNAL):
                return None, {
                    "accepted": False,
                    "ok": False,
                    "moved": False,
                    "error": "busy",
                    "http_status": 409,
                    "verb": p.get("verb", kind),
                    "message": (f"a {p.get('verb', kind)} is already in "
                                f"flight; this {verb} was NOT applied and the "
                                f"arm is still executing the previous one."),
                    "details": {
                        "current": kind,
                        "current_seq": p.get("seq"),
                        "hint": ("wait for get_robot_state.mode == 'hold', "
                                 "then send this again; or call stop_robot "
                                 "first if you mean to abandon the motion "
                                 "that is running."),
                    },
                    "say": (f"I'm still in the middle of a {p.get('verb', kind)}"
                            f", so I have NOT started that {verb} — ask me "
                            "again once I've finished, or tell me to stop."),
                }
            superseded = p if busy else None
            self.motion_seq += 1
            seq = self.motion_seq
        if superseded is not None:
            # The idle loop's plans carry no seq (it writes the slot directly),
            # so only quote one when there is one to quote.
            sq = superseded.get("seq")
            self._log_refusal(
                f"{verb} SUPERSEDED an in-flight "
                f"{superseded.get('verb', 'motion')}"
                + (f" (seq {sq})" if sq else "")
                + "; how far that motion got was not measured")
            # RELEASE ITS WAITER. Without this a caller blocked on the
            # cancelled motion sits until its budget expires and then reports
            # a timeout, when what actually happened is that somebody else
            # took the arm. _await_completion also detects the newer seq, but
            # writing the record means get_robot_state.last_command tells the
            # same story to a caller that was only polling.
            self._record_superseded(
                superseded, seq,
                f"a {verb} ended this motion before it finished; how far it "
                "got was not measured")
        return seq, superseded

    def _slot(self, seq, verb: str, source: str = SOURCE_EXTERNAL,
              measure: Optional[dict] = None) -> dict:
        """The bookkeeping every motion plan carries so a later _begin_motion
        can name its occupant, and so the tick knows WHAT TO MEASURE when the
        plan ends. Extra keys only; the interpolation itself reads none of
        them."""
        slot = {"seq": seq, "verb": verb, "source": source}
        if measure is not None:
            slot["measure"] = measure
        return slot

    @staticmethod
    def _note_superseded(out: dict, superseded) -> dict:
        """Say plainly that a running motion was cancelled, and that where it
        got to was NOT measured (PROTOCOL.md 5.4.1 rule 1: never a number
        nobody measured).

        The key is `cancelled_motion`, NOT `superseded`. Now that a waited
        reply carries `superseded: <bool>` meaning "THIS motion was cut
        short", the two would collide on the one verb that can do both:
        reset_to_home cancels an occupant AND can itself be cancelled while
        it runs. A truthy dict under the boolean's name reads to a consumer
        (the relay's own result summariser included) as "your command was
        superseded" -- the exact opposite of what happened."""
        if superseded:
            out["cancelled_motion"] = {
                "verb": superseded.get("verb"),
                "seq": superseded.get("seq"),
                "achieved": None,
                "note": ("this command ended a motion that was still running; "
                         "how far it got was not measured"),
            }
        return out

    # ── stop_robot ────────────────────────────────────────────────

    def _stop_measure_rest(self, t_halt: float,
                           q_halt: List[float]) -> dict:
        """Did the arm ACTUALLY come to rest? Answered from JOINT SAMPLES.

        `q` at the freeze instant was already measured, but a position read
        is not a rest verdict: it says where the arm WAS, not that it stopped
        moving. Stationary here means the joint vector stopped changing over
        a real interval."""
        blank: Dict[str, Any] = {
            "max_joint_radps": None, "joint": None,
            "over_s": None, "samples": 0, "moved_since_halt_rad": None,
            "basis": "joint position samples taken by the simulation tick",
        }
        # THE SIM THREAD CANNOT WAIT FOR ITSELF. tick() produces the samples,
        # so blocking here (the Robot Window's stop button runs on this
        # thread) would measure nothing and stall the world.
        if threading.current_thread() is threading.main_thread():
            blank["reason"] = (
                "the stop was executed on the simulation thread, which "
                "cannot advance while it waits, so no settling interval "
                "could be observed")
            return blank
        if time.time() - self.last_tick_at > self.WAIT_STALL_S:
            blank["reason"] = ("the simulation is not stepping, so no joint "
                               "samples are being produced")
            return blank
        deadline = t_halt + self.STOP_SETTLE_S
        while time.time() < deadline:
            time.sleep(min(self.WAIT_POLL_S, max(0.0, deadline - time.time())))
        with self.lock:
            after = [s for s in self._q_hist if s[0] >= t_halt]
        if len(after) < 2:
            blank["samples"] = len(after)
            blank["reason"] = ("fewer than two joint samples arrived in the "
                               f"{self.STOP_SETTLE_S:.2f}s settling window "
                               "(is the world stepping?)")
            return blank
        out = dict(blank)
        out["samples"] = len(after)
        last_q = after[-1][1]
        n = min(len(q_halt), len(last_q))
        out["moved_since_halt_rad"] = max(
            [abs(last_q[i] - q_halt[i]) for i in range(n)] or [0.0])
        out["q_after_settle"] = list(last_q)
        cut = t_halt + self.STOP_SETTLE_S * self.STOP_TAIL_FRAC
        tail = [s for s in after if s[0] >= cut]
        if len(tail) < 2 or (tail[-1][0] - tail[0][0]) < self.STOP_MIN_SPAN_S:
            tail = after[-2:]
        span = tail[-1][0] - tail[0][0]
        if span < self.STOP_MIN_SPAN_S:
            out["reason"] = (f"joint samples spanned only {span:.3f}s, below "
                             f"the {self.STOP_MIN_SPAN_S:.2f}s needed to "
                             "difference a rate")
            return out
        a, bq = tail[0][1], tail[-1][1]
        m = min(len(a), len(bq))
        rates = [abs(bq[i] - a[i]) / span for i in range(m)]
        if not rates:
            out["reason"] = "the arm reported no joints to measure"
            return out
        out["max_joint_radps"] = max(rates)
        out["joint"] = (self.joint_names[rates.index(max(rates))]
                        if rates.index(max(rates)) < len(self.joint_names)
                        else None)
        out["over_s"] = span
        return out

    def _stop_arm_hold(self) -> dict:
        """Make an operator stop HOLD the arm, from the moment it executes.

        WHY NOT note_external_command: that arms the quiet window, and it is
        called when the operator's PROMPT lands, seconds before the model
        chooses a tool. With --idle-resume-s 12 a slow LLM turn outlives its
        own pause, so the stop arrives with nothing holding the robot -- the
        failure measured on this demo's tug_a tug. A stop is also never a
        trailing artefact of a resume, so it clears the post-resume exemption
        instead of being swallowed by it."""
        loop = getattr(self, "idle_loop", None)
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
                "leg_when_stopped": getattr(loop, "leg", "idle")}

    def act_stop(self, source: str = SOURCE_EXTERNAL) -> dict:
        """Freeze the arm, then MEASURE and report whether it froze.

        This used to return `{accepted, halted_at, q}`. `q` was genuinely
        measured (position sensors at the freeze instant), which is why this
        bridge read honestly in the transcript where the mobile bridge did
        not -- but a position is not a rest verdict, and nothing here armed
        the idle-loop hold, so the pick loop was free to carry on the moment
        the quiet window lapsed. Both are fixed: `stationary` is differenced
        from tick-sampled joint vectors, and the hold is armed at EXECUTION
        time (see _stop_arm_hold).

        `source` mirrors the motion verbs: anything other than
        SOURCE_EXTERNAL is an INTERNAL freeze (a fired pause intent) that
        must not hold the loop against itself or spend a settling window.
        """
        operator = (source == self.SOURCE_EXTERNAL)
        # OVERRIDING, never rejecting (capabilities.busy_overriding_actions):
        # a stop that answers 409 is useless. `override=True` is what makes
        # that true -- and it is also what releases the stopped motion's
        # waiter with an honest achieved: null instead of leaving it to sit
        # out its whole wait budget and then report a timeout.
        _seq, superseded = self._begin_motion("stop_robot", override=True)
        with self.lock:
            q = self._read_q()
            self.motion = ("hold", {"q": q})
        # ⚠ THE HARDWARE HALF OF A STOP IS REPORTED SEPARATELY FROM `accepted`,
        # and conflating them was the sharpest safety defect in this stack.
        # `accepted: true` only ever meant "the SIM froze" -- it was returned
        # unchanged whether the real arm was told, refused, or unreachable,
        # because _hw_fwd swallowed the exception. An operator reading a green
        # response while the arm keeps moving is exactly the wrong direction
        # for this failure to point. `hardware_stopped` is now tri-state:
        #   True  -> the backend confirmed it reached the robot
        #   False -> we tried and FAILED; use the physical e-stop
        #   None  -> no hardware attached; there was nothing to stop
        hw_stopped = self._hw_fwd("stop")
        halted_at = time.time()
        hold = self._stop_arm_hold() if operator else None
        # `q` here IS measured -- _read_q() reads the position sensors at the
        # instant the arm was frozen -- so it keeps the measurement-shaped key.
        out: Dict[str, Any] = {"accepted": True, "verb": "stop_robot",
                               "halted_at": halted_at, "q": q,
                               "hardware_stopped": hw_stopped,
                               # A stop commands one thing: zero joint rate.
                               "commanded": 0.0, "unit": "rad/s"}
        if hw_stopped is False:
            out["hardware_error"] = (self.hw_last_error
                                     or "backend reported the stop did not "
                                        "reach the robot")
            out["warning"] = ("THE REAL ARM WAS NOT CONFIRMED STOPPED -- "
                              "USE THE PHYSICAL E-STOP")
        if not operator:
            out.update({"achieved": None, "error": None, "settled": None,
                        "stationary": None, "source": source,
                        "measured": {"basis": None,
                                     "reason": "internal freeze (a fired "
                                               "pause intent): not measured, "
                                               "and it does not hold the "
                                               "autonomy loop"}})
            return self._note_superseded(out, superseded)
        m = self._stop_measure_rest(halted_at, q)
        rate = m.get("max_joint_radps")
        stationary = None if rate is None else rate <= self.STOP_STILL_RADPS
        out.update({
            "achieved": rate,
            "error": (None if rate is None else rate - 0.0),
            # `settled` == measurably at rest. Null, never True, when nothing
            # was measured.
            "settled": stationary,
            "stationary": stationary,
            "measured": m,
            "idle_loop": hold,
        })
        if stationary is None:
            out["note"] = (
                "the arm was frozen at the joint angles in `q` and the motion "
                "slot was cleared, but this bridge could NOT confirm it came "
                "to rest: " + str(m.get("reason", "no measurement")) + ". Say "
                "it was commanded to stop and read get_robot_state to "
                "confirm -- do not tell the operator it has halted.")
        elif stationary is False:
            out["note"] = (
                f"STILL MOVING {m['over_s']:.2f}s after the freeze: joint "
                f"{m.get('joint')} at {rate:.3f} rad/s, "
                f"{m['moved_since_halt_rad']:.3f} rad of travel since the "
                "stop was issued. Tell the operator it has NOT stopped yet.")
        else:
            out["note"] = (
                f"measured at rest: {rate:.3f} rad/s over {m['over_s']:.2f}s, "
                f"{m['moved_since_halt_rad']:.3f} rad of drift after the "
                "freeze.")
        if hold and hold.get("present"):
            out["note"] += (
                f" The pick loop is held for {hold['hold_s']:.0f}s and then "
                "RESUMES ON ITS OWN -- say that, do not promise it will stay "
                "stopped indefinitely. Call resume_autonomy to hand it back "
                "sooner, or hold_until_told for a stop with no auto-resume.")
        elif hold is not None:
            out["note"] += (" This robot has no autonomy loop, so nothing "
                            "will move it until you command it again.")
        return self._note_superseded(out, superseded)

    # ── Action handlers (HTTP + intent share these) ───────────────

    def act_reset_to_home(self, duration_s: float = 1.5,
                          wait: bool = False) -> dict:
        # OVERRIDING: this is the park/recovery verb an operator reaches for
        # exactly when something is going wrong, so it cancels rather than
        # refuses -- and says which motion it cancelled.
        seq, superseded = self._begin_motion("reset_to_home", override=True)
        home = [float(v) for v in clamp_q(list(self.home_pose),
                                          self.joint_limits)]
        with self.lock:
            from_q = self._read_q()
            self.motion = ("interp", dict(
                self._slot(seq, "reset_to_home",
                           measure={"kind": "joints", "unit": "rad",
                                    "commanded": home}),
                from_q=from_q,
                to_q=list(self.home_pose),
                start_s=self.robot.getTime(),
                duration_s=duration_s,
            ))
        self._hw_fwd("reset_to_home")
        # PROTOCOL.md 5.4.1 rule 1: this used to return {"q": home_pose} --
        # the COMMANDED pose under `q`, the identical key get_state() uses for
        # the MEASURED joint angles. Same shape, opposite meaning, and the
        # motion has not even started when this returns. Commanded is named
        # commanded, and `achieved` is either MEASURED (wait=true) or null.
        out = self._wait_result(seq, "reset_to_home", wait,
                                budget_s=duration_s * 3.0 + 6.0,
                                eta_s=duration_s, commanded=home, unit="rad")
        out["commanded_q"] = home
        out["q_at_command"] = from_q
        out["duration_s"] = duration_s
        out["joint_names"] = list(self.joint_names)
        return self._note_superseded(out, superseded)

    def act_set_joint_positions(self, q: List[float], duration_s: float = 1.2,
                                slot: Optional[dict] = None) -> dict:
        """INTERNAL joint move. `slot` is the bookkeeping from the caller's
        own _begin_motion claim (see act_set_joint_positions_checked /
        act_set_tcp_target); this method deliberately does NOT claim the slot
        itself, or a TCP solve would claim it twice and supersede itself."""
        if len(q) != len(self.joint_names):
            return {"error": f"q must have {len(self.joint_names)} entries"}
        clamped = clamp_q(q, self.joint_limits)
        with self.lock:
            from_q = self._read_q()
            self.motion = ("interp", dict(
                slot or {},
                from_q=from_q,
                to_q=clamped,
                start_s=self.robot.getTime(),
                duration_s=duration_s,
            ))
        # Forward the joint move to the real arm -- unless a TCP solve already
        # forwarded a move_linear (then the inner joint move is sim-only).
        if not self._hw_suppress_joint:
            self._hw_fwd("move_joint", clamped)
        return {"accepted": True, "clamped_q": clamped}

    # ── operator-facing validators ────────────────────────────────
    #
    # act_set_joint_positions CLAMPS silently, which is right for internal
    # callers (an IK solution is already inside the limits) and wrong for an
    # operator: asked for joint 2 at 5 rad on a +/-pi joint, the arm moves to
    # pi and the model reports the 5 it asked for. Same accept-and-claim as
    # the unreachable TCP target, one axis down. The checked wrappers are
    # what the TOOLS and the HTTP surface call; nothing internal changes.

    def act_set_joint_positions_checked(self, q: List[float],
                                        duration_s: float = 1.2,
                                        wait: bool = False) -> dict:
        if len(q) != len(self.joint_names):
            return {
                "accepted": False, "moved": False,
                "error": "wrong_joint_count",
                "expected": len(self.joint_names),
                "joint_names": list(self.joint_names),
                "say": (f"I have {len(self.joint_names)} joints and that was "
                        f"{len(q)} numbers, so I haven't moved. Send me "
                        f"{len(self.joint_names)} angles in radians, in this "
                        f"order: {', '.join(self.joint_names)}."),
            }
        bad = []
        for i, (v, (lo, hi)) in enumerate(zip(q, self.joint_limits)):
            try:
                v = float(v)
            except (TypeError, ValueError):
                bad.append((i, v, lo, hi))
                continue
            if v < lo - 1e-6 or v > hi + 1e-6:
                bad.append((i, v, lo, hi))
        if bad:
            names = [f"{self.joint_names[i]} at {v:.2f} rad (limit "
                     f"{lo:.2f} to {hi:.2f})" for i, v, lo, hi in bad]
            self._log_refusal("set_joint_positions REFUSED (" + "; ".join(names)
                              + "); arm did not move")
            return {
                "accepted": False, "moved": False,
                "error": "joint_limit_exceeded",
                "violations": [{"joint": self.joint_names[i], "requested": v,
                                "limit": [lo, hi]} for i, v, lo, hi in bad],
                "joint_limits_rad": [[lo, hi] for lo, hi in self.joint_limits],
                "say": ("I haven't moved — that would take "
                        + ", and ".join(names)
                        + ". Give me angles inside those limits and I'll go."),
            }
        # PROTOCOL.md 5.4.1 rule 5. Claimed here rather than in the raw
        # setter so a TCP solve (which claims for itself and then calls the
        # raw setter) cannot supersede its own motion.
        seq, refusal = self._begin_motion("set_joint_positions")
        if seq is None:
            return refusal
        commanded = [float(v) for v in clamp_q([float(v) for v in q],
                                               self.joint_limits)]
        inner = self.act_set_joint_positions(
            [float(v) for v in q], duration_s=duration_s,
            slot=self._slot(seq, "set_joint_positions",
                            measure={"kind": "joints", "unit": "rad",
                                     "commanded": commanded}))
        # `moved` is GONE. It was `bool(accepted)` -- an assertion of motion
        # made at submission, before a single tick had run, in a key that
        # reads as an observation. `achieved` (measured joint angles) and
        # `settled` say the same thing honestly, or say null.
        out = self._wait_result(seq, "set_joint_positions", wait,
                                budget_s=duration_s * 3.0 + 6.0,
                                eta_s=duration_s, commanded=commanded,
                                unit="rad")
        out["clamped_q"] = inner.get("clamped_q")
        out["joint_names"] = list(self.joint_names)
        return out

    def act_servo_joint_positions(self, q: List[float]) -> dict:
        """The STREAMING setpoint verb (capabilities.servo). Non-blocking,
        last-write-wins: it returns at dispatch, and a servo command sent
        while a servo stream is live retargets it IN PLACE -- same seq, no
        supersede record, never a 409. A servo command arriving while a GOAL
        verb (set_joint_positions / set_tcp_* / pick / ...) is in flight
        PREEMPTS it (the goal's waiter is released with achieved: null and
        the reply names it in `preempted`).

        This is the verb a trajectory controller or streaming teleop points
        at -- the goal contract measurably shreds a stream (a second
        set_joint_positions 50 ms after the first answered 409 busy and was
        not applied), and ros2_control's joint_command will be wired here
        (follow-up; the ROS package is out of scope for this change).

        Out-of-limit values are CLAMPED, not refused (`clamped: true` says
        so, and `target` is always the clamped vector actually adopted --
        never the request echoed back): mid-stream a refusal would drop one
        setpoint on the floor and the stream has no operator reading `say`
        sentences. Wrong joint COUNT is still a refusal -- that is a wiring
        error, not a boundary condition.

        Deliberately NOT on the LLM tool surface: a chat agent gets nothing
        from a 20 Hz lane and the honest, measured verb for it is
        set_joint_positions (tool-design rule: more tools degrade
        selection). HTTP-only. Sim-only, too: nothing is forwarded to a
        hardware backend -- streaming per-cycle goals at a real arm's
        move_joint would recreate the exact busy pile-up this verb exists
        to end (use set_joint_positions for hardware moves).

        `achieved` in the reply is ALWAYS null -- the verb answers before a
        single tick has run, and a number nobody measured is the failure
        this bridge's whole result layer exists to kill. The measured
        result lands in get_robot_state.last_command (matched by `seq`)
        once the stream goes quiet and tick() parks + measures the plan.
        """
        if len(q) != len(self.joint_names):
            return {
                "accepted": False,
                "error": "wrong_joint_count",
                "expected": len(self.joint_names),
                "joint_names": list(self.joint_names),
            }
        try:
            qf = [float(v) for v in q]
        except (TypeError, ValueError):
            return {"accepted": False, "error": "non_numeric_q",
                    "joint_names": list(self.joint_names)}
        if not all(math.isfinite(v) for v in qf):
            return {"accepted": False, "error": "non_finite_q",
                    "joint_names": list(self.joint_names)}
        clamped = [float(v) for v in clamp_q(qf, self.joint_limits)]
        was_clamped = any(abs(a - b) > 1e-9 for a, b in zip(qf, clamped))
        now = time.time()

        def _reply(seq: int, superseded_previous: bool,
                   preempted: Optional[str], updates: int) -> dict:
            return {
                "accepted": True,
                "verb": "servo_joint_positions",
                "mode": "servo",
                "seq": seq,
                # The target ADOPTED (post-clamp) -- never the request
                # echoed back, and never presented as a measurement.
                "target": list(clamped),
                "clamped": was_clamped,
                "superseded_previous": superseded_previous,
                "preempted": preempted,
                "updates": updates,
                "achieved": None,
                "error": None,
                "note": ("non-blocking: nothing has been measured yet. The "
                         "arm tracks the latest target each tick; when the "
                         "stream goes quiet the motion parks and the "
                         "MEASURED result appears in "
                         "get_robot_state.last_command with this seq."),
            }

        # A completion may still be sitting out its settle window; measure
        # it before claiming, or its record is dropped (same reason
        # _begin_motion does this).
        self._flush_pending()
        with self.lock:
            kind, p = self.motion
            if (kind == "servo"
                    and p.get("source", self.SOURCE_EXTERNAL)
                    == self.SOURCE_EXTERNAL):
                # LIVE STREAM: retarget in place. Same seq -- this is one
                # motion receiving updates, not a new motion superseding an
                # old one, so no supersede record and no waiter to release.
                p["to_q"] = list(clamped)
                p["last_update_at"] = now
                p["updates"] = int(p.get("updates", 1)) + 1
                m = p.get("measure")
                if m is not None:
                    # The completion measures against the LAST target the
                    # stream adopted.
                    m["commanded"] = list(clamped)
                return _reply(int(p.get("seq") or 0),
                              superseded_previous=True, preempted=None,
                              updates=int(p["updates"]))
        # Slot free, or occupied by a goal verb / idle-loop plan: claim it,
        # overriding. override=True is what releases a preempted goal's
        # waiter with an honest achieved: null (see _begin_motion).
        seq, superseded = self._begin_motion("servo_joint_positions",
                                             override=True)
        preempted = (superseded.get("verb", "motion")
                     if superseded is not None else None)
        with self.lock:
            self.motion = ("servo", dict(
                self._slot(seq, "servo_joint_positions",
                           measure={"kind": "joints", "unit": "rad",
                                    "commanded": list(clamped)}),
                to_q=list(clamped),
                last_update_at=now,
                updates=1,
            ))
        return _reply(seq, superseded_previous=superseded is not None,
                      preempted=preempted, updates=1)

    def act_set_gripper_width_checked(self, width_m: Any) -> dict:
        if self.effector is None:
            return {"accepted": False, "moved": False,
                    "error": "effector_unavailable",
                    "say": "I have no gripper fitted, so there is no width to set."}
        mx = float(getattr(self.effector, "max_width", 0.0) or 0.0)
        try:
            w = float(width_m)
        except (TypeError, ValueError):
            w = None
        if mx <= 0.0:
            return {"accepted": False, "moved": False,
                    "error": "no_width_control",
                    "say": (f"My {self.effector.model} end effector has no "
                            "width control — it grips or it releases, there "
                            "is no opening to set.")}
        if w is None or w < 0.0 or w > mx:
            self._log_refusal(f"set_gripper_width REFUSED {width_m!r} "
                              f"(range 0..{mx:.3f} m); gripper unchanged")
            return {
                "accepted": False, "moved": False,
                "error": "width_out_of_range",
                "requested_m": width_m,
                "range_m": [0.0, round(mx, 4)],
                "say": (f"I haven't touched the gripper — I can open between "
                        f"0 and {mx * 1000:.0f} mm, and {width_m} is outside "
                        "that."),
            }
        out = self.act_set_gripper_width(w)
        out["accepted"] = True
        # `moved` used to be a hardcoded True here, asserted at submission.
        # The effector exposes no finger position sensor -- GripperEffector
        # .state() reports the width it was last COMMANDED -- so the honest
        # answer is the command plus an explicit null, and this verb takes no
        # `wait` (capabilities.non_waitable_actions says why).
        out["commanded_width_m"] = w
        out["achieved_width_m"] = None
        out["note"] = ("commanded; this gripper has no finger position "
                       "readback, so the achieved width is not measured. "
                       "gripper.object_present is a real sensor read where "
                       "the hardware has one.")
        return out

    # ── the reachable envelope, and REFUSING what is outside it ───
    #
    # MEASURED FAILURE THIS FIXES: asked to "move your end effector to x=50
    # y=50 z=20" (73 m out; this arm reaches 0.95 m) the model called
    # set_tcp_target and answered "Moving to those coordinates now." The
    # bridge HAD rejected it -- but with a bare {"error": "unreachable_target",
    # "radius": 73.5}, which carries no refusal and no sentence, so the model
    # read a tool result it did not understand and narrated the motion it had
    # asked for. The envelope is not new information; it is the same
    # workspace_* numbers the solver already gated on, finally EXPOSED.
    #
    # The honesty rule: never accept-and-claim. Out of reach => accepted
    # false, nothing moves, and a `say` naming the limit and the nearest
    # point that IS reachable.

    def _base_world_pose(self):
        """(position, 3x3 row-major orientation) of this arm's base in world
        coordinates, or None. Cached: a bolted-down arm does not move."""
        if getattr(self, "_base_world_cache", None) is not None:
            return self._base_world_cache
        if self._self_node is None:
            return None
        try:
            bp = list(self._self_node.getPosition())
            r = list(self._self_node.getOrientation())
        except Exception:
            return None
        # Sensors read NaN until the first robot.step() completes, and the
        # relay builds its tool descriptions before that -- caching a NaN
        # base pose would poison every world<->base conversion for the whole
        # run. Refuse to cache anything non-finite; the next call retries.
        if not all(math.isfinite(v) for v in list(bp) + list(r)):
            return None
        self._base_world_cache = (bp, r)
        return self._base_world_cache

    def _world_to_base(self, xyz):
        """World point -> arm-base frame (the frame the IK chain works in)."""
        pose = self._base_world_pose()
        if pose is None:
            return list(xyz)
        bp, r = pose
        dx, dy, dz = (xyz[0] - bp[0], xyz[1] - bp[1], xyz[2] - bp[2])
        # R is row-major world<-base, so the inverse is its transpose.
        return [r[0] * dx + r[3] * dy + r[6] * dz,
                r[1] * dx + r[4] * dy + r[7] * dz,
                r[2] * dx + r[5] * dy + r[8] * dz]

    def _base_to_world(self, xyz):
        pose = self._base_world_pose()
        if pose is None:
            return list(xyz)
        bp, r = pose
        x, y, z = xyz
        return [bp[0] + r[0] * x + r[1] * y + r[2] * z,
                bp[1] + r[3] * x + r[4] * y + r[5] * z,
                bp[2] + r[6] * x + r[7] * y + r[8] * z]

    def reach_envelope(self) -> dict:
        """This arm's REAL reachable shell, derived from its own IK config
        and joint limits -- the same numbers the solver gates on. Published
        so a target can be checked (and refused) before anything moves."""
        ik_cfg = self.cfg.get("ik")
        if not ik_cfg:
            return {"has_ik": False,
                    "reason": f"{self.robot_id} has no IK chain; motion is "
                              "joint-space only (set_joint_positions)."}
        pose = self._base_world_pose()
        env = {
            "has_ik": True,
            "frame": "arm base (the arm's own origin), metres",
            "max_radius_m": round(float(ik_cfg["workspace_max_radius"]), 3),
            "min_radius_m": round(float(ik_cfg["workspace_min_radius"]), 3),
            "min_z_m": round(float(ik_cfg["workspace_min_z"]), 3),
            "joint_limits_rad": [
                [round(lo, 3), round(hi, 3)] for lo, hi in self.joint_limits],
            "joint_names": list(self.joint_names),
        }
        if pose is not None:
            env["base_world_xyz"] = [round(v, 3) for v in pose[0]]
        return env

    def _nearest_reachable(self, xyz) -> List[float]:
        """The closest point inside the shell to an out-of-reach request.
        Radius is clamped first, then the floor, then the radius is restored
        in xy so the answer really is ON the shell."""
        ik = self.cfg["ik"]
        rmax = float(ik["workspace_max_radius"])
        rmin = float(ik["workspace_min_radius"])
        zmin = float(ik["workspace_min_z"])
        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        r = math.sqrt(x * x + y * y + z * z)
        if r < 1e-9:
            x, y, z, r = rmin, 0.0, 0.0, rmin
        target_r = min(rmax, max(rmin, r))
        s = target_r / r
        x, y, z = x * s, y * s, z * s
        if z < zmin:
            z = zmin
            xy = math.hypot(x, y)
            want_xy = math.sqrt(max(0.0, target_r * target_r - z * z))
            if xy < 1e-9:
                x, y = want_xy, 0.0
            else:
                x, y = x * want_xy / xy, y * want_xy / xy
        return [round(x, 3), round(y, 3), round(z, 3)]

    def _refuse_target(self, xyz, r: float, limit: str,
                       verb: str = "set_tcp_target") -> dict:
        """A refusal the model can RELAY, in the shape intents.py uses:
        accepted false + a `say`. Nothing moved and nothing will."""
        ik = self.cfg["ik"]
        near = self._nearest_reachable(xyz)
        full = self.reach_envelope()
        base = full.get("base_world_xyz")
        # COMPACT on purpose: a refusal the model has to read in one glance.
        # Returning the whole envelope (six joint limit pairs) buried the one
        # sentence that matters under a wall of numbers. get_reach_envelope
        # is there for anyone who wants the full picture.
        env = {"max_radius_m": full.get("max_radius_m"),
               "min_radius_m": full.get("min_radius_m"),
               "min_z_m": full.get("min_z_m"),
               "frame": full.get("frame"),
               "base_world_xyz": base}
        if limit == "too_far":
            why = (f"that point is {r:.1f} m from my base and I can only "
                   f"reach {ik['workspace_max_radius']:.2f} m")
        elif limit == "too_close":
            why = (f"that point is {r:.2f} m from my base — inside the "
                   f"{ik['workspace_min_radius']:.2f} m dead zone right on "
                   "top of my own column")
        else:
            why = (f"z={xyz[2]:.2f} m is below my floor limit of "
                   f"{ik['workspace_min_z']:.2f} m — I would drive the tool "
                   "into the bench")
        say = (f"I can't reach that, so I haven't moved: {why}. "
               f"The closest point to it that I CAN reach is "
               f"({near[0]:.2f}, {near[1]:.2f}, {near[2]:.2f}) in my own "
               f"frame — say the word and I'll go there instead.")
        # `verb` so the log names the tool that was ACTUALLY refused -- pick
        # and place share this refusal now, and a line blaming set_tcp_target
        # for a refused pick is a small fabrication in the operator log.
        self._log_refusal(f"{verb} REFUSED {tuple(round(float(v), 3) for v in xyz)} "
                          f"({limit}, r={r:.2f} m); arm did not move")
        return {
            "accepted": False,
            "moved": False,
            "error": "unreachable_target",
            "limit": limit,
            "requested_xyz": [float(v) for v in xyz],
            "radius_m": round(r, 3),
            "nearest_reachable_xyz": near,
            "nearest_reachable_world_xyz": (
                [round(v, 3) for v in self._base_to_world(near)]
                if base else None),
            "envelope": env,
            "say": say,
        }

    def _workspace_refusal(self, xyz_base,
                           verb: str = "set_tcp_target") -> Optional[dict]:
        """The reach gate, in ONE place. Returns a refusal for a BASE-frame
        point outside this arm's shell, else None.

        Lifted out of act_solve_ik so pick/place gate on exactly the same
        numbers the coordinate verbs already do -- they had NO gate at all
        (see act_pick / act_place)."""
        ik_cfg = self.cfg.get("ik")
        if not ik_cfg:
            return None
        r = math.sqrt(xyz_base[0] ** 2 + xyz_base[1] ** 2 + xyz_base[2] ** 2)
        if r > ik_cfg["workspace_max_radius"]:
            return self._refuse_target(xyz_base, r, "too_far", verb)
        if r < ik_cfg["workspace_min_radius"]:
            return self._refuse_target(xyz_base, r, "too_close", verb)
        if xyz_base[2] < ik_cfg["workspace_min_z"]:
            return self._refuse_target(xyz_base, r, "below_floor", verb)
        return None

    def _log_refusal(self, msg: str) -> None:
        loop = getattr(self, "idle_loop", None)
        if loop is not None and hasattr(loop, "_log"):
            try:
                loop._log(msg)
                return
            except Exception:
                pass
        print("[omnilink_arm_bridge] " + msg, flush=True)

    def act_solve_ik(self, xyz: Tuple[float, float, float],
                     frame: str = "base") -> dict:
        ik_cfg = self.cfg.get("ik")
        if not ik_cfg:
            return {"accepted": False, "moved": False,
                    "error": "ik_unavailable",
                    "say": (f"I have no inverse kinematics wired up, so I "
                            "can't take a coordinate — I can only be driven "
                            "joint by joint."),
                    "hint": f"{self.robot_id} has no pre-baked IK chain; use set_joint_positions instead."}
        if str(frame).lower().startswith("world"):
            xyz = tuple(self._world_to_base(xyz))
        # Workspace check -- a REFUSAL, not a bare error code.
        refusal = self._workspace_refusal(xyz)
        if refusal is not None:
            return refusal
        q, err, iters = dls_ik(ik_cfg["chain"], self._read_q(), xyz, ik_cfg, self.joint_limits)
        # `ik_residual_m` is the preferred name: this number is the SOLVER's
        # own leftover distance for a candidate joint vector, computed with
        # nothing moving. `err_norm` stays as a back-compat alias because
        # /solve_ik is a preview endpoint whose whole output IS the residual
        # -- unlike set_tcp_target, where the same key sat next to
        # accepted/moved and read as a measured TCP error.
        return {"q": q, "ik_residual_m": err, "err_norm": err, "iters": iters,
                "ik_residual_note": ("solver residual for this candidate "
                                     "solution -- no motion was commanded and "
                                     "nothing was measured"),
                "target_base_xyz": [round(float(v), 4) for v in xyz]}

    # A solve that lands further than this from the request is not a move to
    # the requested point -- it is the solver giving up against a joint limit
    # or a singularity. Reporting it as success is the same accept-and-claim
    # the envelope check exists to stop, one layer down. Generous on purpose:
    # ordinary in-reach targets converge to millimetres.
    IK_ACCEPT_TOL_M = 0.05

    def act_set_tcp_target(self, xyz: Tuple[float, float, float],
                           frame: str = "base", wait: bool = False,
                           duration_s: float = 1.5) -> dict:
        out = self.act_solve_ik(xyz, frame=frame)
        if "error" in out:
            return out
        if out.get("ik_residual_m", 0.0) > self.IK_ACCEPT_TOL_M:
            err = float(out["ik_residual_m"])
            self._log_refusal(
                f"set_tcp_target REFUSED {tuple(round(float(v), 3) for v in xyz)} "
                f"(no solution; best miss {err:.3f} m); arm did not move")
            return {
                "accepted": False, "moved": False,
                "error": "no_ik_solution",
                "requested_xyz": [float(v) for v in xyz],
                "best_miss_m": round(err, 4),
                "say": (f"I can't actually get my tool to that point — it is "
                        f"inside my reach sphere but my joints won't line up "
                        f"for it (best I can manage is {err * 100:.0f} cm "
                        f"off), so I haven't moved. Give me another point and "
                        "I'll try again."),
            }
        if str(frame).lower().startswith("world"):
            xyz = tuple(out["target_base_xyz"])
        # PROTOCOL.md 5.4.1 rule 5. Claimed AFTER the reach/solve refusals so
        # a target we were never going to move to cannot cancel a motion that
        # is legitimately running.
        seq, refusal = self._begin_motion("set_tcp_target")
        if seq is None:
            return refusal
        commanded = [float(v) for v in xyz]
        # The real arm uses its OWN cartesian IK via move_linear; the sim uses
        # our DLS solution. Suppress the inner joint forward so we don't also
        # double-command the hardware in joint space.
        self._hw_suppress_joint = True
        try:
            clamped = self.act_set_joint_positions(
                out["q"], duration_s=duration_s,
                slot=self._slot(seq, "set_tcp_target",
                                measure={"kind": "tcp", "unit": "m",
                                         "commanded": commanded}))
        finally:
            self._hw_suppress_joint = False
        self._hw_fwd("move_linear", tuple(xyz))
        res = self._wait_result(seq, "set_tcp_target", wait,
                                budget_s=duration_s * 3.0 + 6.0,
                                eta_s=duration_s, commanded=commanded,
                                unit="m")
        # `moved` is GONE (it was `True`, hardcoded, at submission) and
        # `err_norm` is RENAMED: it is the IK solver's residual for the
        # candidate joint vector, computed before anything moved, and under
        # the old name a model read it as the achieved TCP error. The
        # achieved TCP error is `error` / `error_m`, from forward kinematics
        # on the position sensors after the move.
        res["solved_q"] = out["q"]
        res["clamped_q"] = clamped.get("clamped_q")
        res["ik_residual_m"] = out["ik_residual_m"]
        res["ik_residual_note"] = ("SOLVER residual, computed before the arm "
                                   "moved -- not a measurement. Compare "
                                   "`error_m` for the achieved TCP error.")
        res["target_base_xyz"] = out.get("target_base_xyz")
        res["frame"] = "arm base (the arm's own origin), metres"
        res["say"] = ("Moving my tool to "
                      f"({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}) now.")
        return res

    def act_set_tcp_pose(self, xyz: Tuple[float, float, float],
                         tcp_offset_z: Optional[float] = None,
                         duration_s: float = 1.5, wait: bool = False) -> dict:
        """6-DOF IK: put the tool at xyz (ARM BASE frame, metres) with its +Z
        axis pointing straight DOWN (top-down approach), then move there.
        `tcp_offset_z` overrides the tool point distance from link6 (e.g. the
        finger throat for a grasp); defaults to the config tcp_offset."""
        ik = self.cfg.get("ik")
        if not ik:
            return {"error": "ik_unavailable"}
        oz = ik["tcp_offset"][2] if tcp_offset_z is None else float(tcp_offset_z)
        off = (0.0, 0.0, oz)
        # Tool +Z -> world -Z (rotation pi about world X): top-down.
        r_target = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
        q, perr, rerr, iters = dls_ik_pose(
            ik["chain"], self._read_q(), list(xyz), r_target, off,
            ik, self.joint_limits)
        # PROTOCOL.md 5.4.1 rule 5.
        seq, refusal = self._begin_motion("set_tcp_pose")
        if seq is None:
            return refusal
        commanded = [float(v) for v in xyz]
        self._hw_suppress_joint = True
        try:
            self.act_set_joint_positions(
                q, duration_s=duration_s,
                slot=self._slot(seq, "set_tcp_pose",
                                measure={"kind": "tcp", "unit": "m",
                                         "commanded": commanded,
                                         # MEASURE THE POINT THAT WAS
                                         # COMMANDED: this verb can retarget
                                         # the tool point (a grasp aims the
                                         # finger throat, not the flange), so
                                         # FK must use the same offset or the
                                         # "error" is between two different
                                         # points on the tool.
                                         "tcp_offset_z": oz}))
        finally:
            self._hw_suppress_joint = False
        self._hw_fwd("move_linear", tuple(xyz))
        res = self._wait_result(seq, "set_tcp_pose", wait,
                                budget_s=duration_s * 3.0 + 6.0,
                                eta_s=duration_s, commanded=commanded,
                                unit="m")
        res["solved_q"] = q
        # Named for what they are: the pose solver's own leftovers, from
        # before the motion. `error` / `error_m` above are the measurement.
        res["ik_residual_m"] = perr
        res["ik_rot_residual_rad"] = rerr
        res["ik_residual_note"] = ("SOLVER residuals, computed before the arm "
                                   "moved -- not measurements.")
        res["iters"] = iters
        res["tcp_offset_z"] = oz
        res["frame"] = "arm base (the arm's own origin), metres"
        return res

    def _topdown_solve(self, xyz_base, seed=None, tcp_offset_z=None):
        """6-DOF top-down IK (tool +Z pointing down) to an ARM-BASE-frame xyz.
        Returns ``(q, pos_err)``, or ``(None, None)`` with no IK chain.

        `pos_err` is the SAME residual dls_ik_pose already computes -- it was
        simply thrown away here. Keeping it is what lets pick/place refuse:
        dls_ik_pose ALWAYS returns a joint-limit-clamped q (it clamps inside
        its own loop and returns `q` on the max-iteration path), so a caller
        that only looks for `q is None` can never detect a failed solve. That
        is why act_place's `if q_above is None` guard has never once fired on
        a configured arm."""
        ik = self.cfg.get("ik")
        if not ik:
            return None, None
        oz = ik["tcp_offset"][2] if tcp_offset_z is None else float(tcp_offset_z)
        off = (0.0, 0.0, oz)
        r_target = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
        q, perr, _rerr, _it = dls_ik_pose(
            ik["chain"], list(seed if seed is not None else self._read_q()),
            list(xyz_base), r_target, off, ik, self.joint_limits)
        return q, perr

    def _topdown_q(self, xyz, seed=None, tcp_offset_z=None):
        """`_topdown_solve` without the residual. Kept for the in-flight
        `cmove` step in _tick_sequence, which re-solves each waypoint at
        execution time and already has a fallback for a bad solve."""
        return self._topdown_solve(xyz, seed=seed,
                                   tcp_offset_z=tcp_offset_z)[0]

    def _plan_topdown_path(self, verb: str, waypoints, seed, tcp_offset_z):
        """Solve an ordered list of BASE-frame top-down waypoints, refusing on
        the first one this arm cannot actually reach.

        Returns ``(qs, refusal)`` with exactly one non-None. The gate is the
        two act_solve_ik / act_set_tcp_target already apply -- the workspace
        shell, then IK_ACCEPT_TOL_M on the residual -- so a coordinate that
        would be refused through set_tcp_target is refused here too."""
        qs = []
        q = list(seed)
        for label, wp in waypoints:
            # _refuse_target already logs the line; naming the waypoint here
            # is what tells the operator WHICH leg of the move was impossible.
            refusal = self._workspace_refusal(wp, f"{verb} ({label} waypoint)")
            if refusal is not None:
                refusal["waypoint"] = label
                return None, refusal
            q, perr = self._topdown_solve(wp, seed=q, tcp_offset_z=tcp_offset_z)
            if q is None:
                return None, {"accepted": False, "moved": False,
                              "error": "ik_unavailable"}
            if perr is not None and perr > self.IK_ACCEPT_TOL_M:
                self._log_refusal(
                    f"{verb} REFUSED: no top-down solution for its {label} "
                    f"waypoint {tuple(round(float(v), 3) for v in wp)} "
                    f"(arm frame; best miss {perr:.3f} m); arm did not move")
                return None, {
                    "accepted": False, "moved": False,
                    "error": "no_ik_solution",
                    "waypoint": label,
                    "requested_base_xyz": [round(float(v), 4) for v in wp],
                    "best_miss_m": round(float(perr), 4),
                    "say": (f"I haven't moved — I can't get my tool square "
                            f"over that point for the {label} part of the "
                            f"{verb} (best I can manage is {perr * 100:.0f} cm "
                            "off). Give me somewhere else and I'll try again."),
                }
            qs.append(q)
        return qs, None

    def act_pick(self, name: Optional[str] = None, approach_h: float = 0.12,
                 grasp_dz: float = 0.02, lift_h: float = 0.18,
                 duration_s: float = 1.4, wait: bool = False) -> dict:
        """Reach a graspable object top-down, grasp (weld), and lift. Targets
        the named DEF GRASP_<name> object, else the nearest graspable. Falls
        back to a stationary grasp if there's no IK chain or no object. On a
        connected real arm the reach+grip is forwarded to the hardware backend
        (move_linear + grasp) and the sim mirrors it."""
        with self.lock:
            items = list(self._iter_graspables())
        cand = None
        if name:
            key = name.strip().lower()
            for node, tf in items:
                try:
                    d = (node.getDef() or "").lower()
                except Exception:
                    d = ""
                if key and key in d:
                    cand = (node, tf)
                    break
        if cand is None and items:
            tcp = self._tcp_world() or [0.0, 0.0, 0.0]
            best_d = 1e18
            for node, tf in items:
                try:
                    p = tf.getSFVec3f()
                except Exception:
                    continue
                dd = sum((p[k] - tcp[k]) ** 2 for k in range(3))
                if dd < best_d:
                    best_d = dd
                    cand = (node, tf)
        if cand is None or self.cfg.get("ik") is None:
            return self.act_grasp()           # nothing to plan -> close in place
        node, tf = cand
        try:
            p = list(tf.getSFVec3f())
        except Exception:
            return self.act_grasp()
        try:
            target_def = node.getDef()
        except Exception:
            target_def = None
        seed = self._read_q()
        # Physics grip must straddle the cube precisely: put the finger throat
        # on the cube CENTRE (grasp_dz 0), the way the validated pick-place demo
        # does -- a throat 2 cm high catches only the top edge and cams the cube
        # out. The kinematic weld tolerates slack so it keeps its small offset.
        phys = bool((self.gripper_cfg or {}).get("physics_grasp"))
        if phys:
            grasp_dz = 0.0
        # Reach so the GRASP POINT (flange + tool_reach along the tool axis),
        # not the flange itself, lands on the cube -- the flange stays one
        # gripper-length above. Matches _tcp_world's anchor+tool_reach weld.
        goz = self.cfg["ik"]["tcp_offset"][2] + float(self.tool_reach)
        # ── WORLD -> ARM BASE (tool-design rule: the tool owns the frame) ──
        # `p` is the WORLD translation of a root-child DEF GRASP_* Solid
        # (_iter_graspables walks root.children), and the IK chain works in the
        # ARM BASE frame. This conversion was simply MISSING: the world point
        # went straight into _topdown_q. The non-LLM idle loop had it right all
        # along (ArmIdleLoop._pick: `p = self._to_base(pw)`), which is what
        # makes this an omission rather than a convention.
        # MEASURED on flagship/warehouse_omnilink.omniworld, where the OmniArm 6 base sits
        # at world (-8, 4.3, 0): picking DEF GRASP_PART_A (world -7.62 4.14
        # 0.427) asked the solver for a point 8.69 m from the base -- against a
        # 0.95 m reach envelope -- and dls_ik_pose duly returned a joint-limit-
        # clamped q that misses by 7.585 m, while act_pick answered
        # accepted:true. Converted first, the same waypoint is 0.685 m out and
        # solves to 0.2 mm.
        pb = self._world_to_base(p)
        above_xyz = [pb[0], pb[1], pb[2] + approach_h]
        at_xyz = [pb[0], pb[1], pb[2] + grasp_dz]
        lift_xyz = [pb[0], pb[1], pb[2] + lift_h]
        # ── AND REFUSE WHAT IT CANNOT REACH ───────────────────────────────
        # There was no reach gate here at all: _topdown_q discarded the IK
        # residual and dls_ik_pose never returns None for a configured arm, so
        # the `q_above is None` guard below could not fire and act_pick could
        # only ever answer accepted:true. Same gate as act_set_tcp_target.
        qs, refusal = self._plan_topdown_path(
            "pick", [("approach", above_xyz), ("grasp", at_xyz),
                     ("lift", lift_xyz)], seed, goz)
        if refusal is not None:
            refusal["target"] = target_def
            refusal["target_world_xyz"] = [round(float(v), 4) for v in p]
            return refusal
        q_above, q_at, q_lift = qs
        if self._hw_connected():
            # The hardware backend's move_linear takes coordinates in the
            # ARM's own frame -- act_set_tcp_target forwards target_base_xyz
            # to it -- so this had the same world-for-base bug.
            self.hw.move_linear((at_xyz[0], at_xyz[1], at_xyz[2]))
            self.hw.grasp()
            return {"accepted": True, "mode": "hardware",
                    "backend": self.hw_name, "target": target_def, "pos": p,
                    "target_world_xyz": [round(float(v), 4) for v in p],
                    "target_base_xyz": [round(float(v), 4) for v in pb]}
        if phys:
            # Physics pick: spread the fingers, then descend/lift with the IK
            # solved fresh at each waypoint (cmove) so the wrist stays square and
            # the fingers straddle the cube; ~1.5 s for the force-grip fingers
            # (0.04 m/s) to reach the cube and squeeze to the effort cap before a
            # gentle lift -- mirrors the flagship demo.
            steps = [
                {"t": "open"},
                {"t": "cmove", "xyz": above_xyz, "oz": goz, "dur": duration_s},
                {"t": "cmove", "xyz": at_xyz, "oz": goz, "dur": 0.8},
                {"t": "wait", "dur": 0.3},
                {"t": "grasp"},
                {"t": "wait", "dur": 1.5},
                {"t": "cmove", "xyz": lift_xyz, "oz": goz, "dur": 1.0},
            ]
        else:
            # Kinematic weld: precomputed joint targets are fine (the magnet
            # tolerates pose slack) and the grasp is instant.
            steps = [
                {"t": "move", "to_q": q_above, "dur": duration_s},
                {"t": "move", "to_q": q_at, "dur": 0.8},
                {"t": "wait", "dur": 0.2},
                {"t": "grasp"},
                {"t": "wait", "dur": 0.2},
                {"t": "move", "to_q": q_lift, "dur": 0.8},
            ]
        # PROTOCOL.md 5.4.1 rule 5: claimed only once the whole path is known
        # to be reachable, so a refused pick never cancels a running motion.
        seq, refusal = self._begin_motion("pick")
        if seq is None:
            return refusal
        eta = float(duration_s) + 3.6 + (1.5 if phys else 0.0)
        with self.lock:
            self.motion = ("sequence", dict(
                self._slot(seq, "pick",
                           measure={"kind": "pick", "unit": "m",
                                    # The lift waypoint is where the TOOL is
                                    # commanded to end up; `error_m` is the
                                    # measured distance from it.
                                    "commanded": [float(v) for v in lift_xyz],
                                    "tcp_offset_z": goz,
                                    "frame": ("arm base (the arm's own "
                                              "origin), metres"),
                                    "target": target_def,
                                    # Read LIVE at completion, not now: this
                                    # is the part's own translation field.
                                    "target_field": tf}),
                steps=steps, i=0, from_q=self._read_q(),
                start_s=self.robot.getTime()))
        # `pos` IS measured (the part's live world translation, read above).
        # Where the TOOL ends up, and whether the part is actually in the
        # gripper, are the pick's real outcome and are MEASURED at completion
        # -- `holding` in the result (or in get_robot_state.last_command).
        res = self._wait_result(seq, "pick", wait,
                                budget_s=eta * 2.5 + 8.0, eta_s=eta,
                                commanded=[float(v) for v in lift_xyz],
                                unit="m")
        res["target"] = target_def
        res["pos"] = p
        # ⚠ DO NOT overwrite target_world_xyz with `p`. _completion_report has
        # already written it from the part's LIVE translation field; `p` is
        # where the part was at PLANNING time, before the arm moved. MEASURED
        # 2026-08-16: a successful lift (z 0.2449 -> 0.4040) still reported
        # target_world_xyz [0.46, 0.0, 0.2449] -- the pre-pick pose -- so the
        # response said the block was sitting on the table it had just been
        # lifted off, and any agent reading it concluded the pick had failed.
        res.setdefault("target_world_xyz", [round(float(v), 4) for v in p])
        res["target_world_xyz_at_plan"] = [round(float(v), 4) for v in p]
        res["target_base_xyz"] = [round(float(v), 4) for v in pb]
        res["frame"] = ("commanded/achieved are ARM BASE frame metres; "
                        "target_world_xyz is world frame")
        return res

    def act_place(self, xyz=None, approach_h: float = 0.14, drop_dz: float = 0.04,
                  lift_h: float = 0.18, duration_s: float = 1.4,
                  wait: bool = False) -> dict:
        """Carry the held object to a drop location top-down and release it.

        `xyz` is a WORLD-frame point -- that is what the tool schema has always
        promised the model, and world coordinates are what an agent actually
        has (from node_pose / the scene tree). Omitted, it falls back to the
        arm cfg's `drop_zone`, which is BASE-frame (ArmIdleLoop._drop_point_base
        uses it raw while converting every world position through _to_base), so
        the two paths must NOT be converted alike.
        Falls back to a stationary release if there's no IK chain."""
        # ── WORLD -> ARM BASE (tool-design rule: the tool owns the frame) ──
        # The caller's point used to go STRAIGHT into _topdown_q, which is the
        # base-frame chain, while the tool description said "World-frame drop
        # position". On flagship/warehouse_omnilink.omniworld the OmniArm 6 base is at
        # world (-8, 4.3, 0), so a world drop point was ~9 m out of frame: the
        # solver was handed a target ~8.7 m from the base against a 0.95 m
        # envelope, dls_ik_pose returned a joint-limit-clamped q missing by
        # ~7.6 m, and act_place answered {"accepted": true} while the arm swung
        # to a clamped nonsense pose. Converted with the SAME helper the
        # coordinate verbs use (_world_to_base, via act_solve_ik frame="world").
        if xyz is None:
            # cfg drop_zone is already in the arm's own frame -- do not convert.
            pb = list(self.cfg.get("drop_zone") or [0.30, 0.34, 0.0])
            pb = [float(pb[0]), float(pb[1]), float(pb[2])]
            p_world = self._base_to_world(pb)
            frame_in = "cfg drop_zone (arm base frame)"
        else:
            p_world = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
            pb = self._world_to_base(p_world)
            frame_in = "world"
        if self.cfg.get("ik") is None:
            return self.act_release()
        seed = self._read_q()
        phys = bool((self.gripper_cfg or {}).get("physics_grasp"))
        goz = self.cfg["ik"]["tcp_offset"][2] + float(self.tool_reach)
        above_xyz = [pb[0], pb[1], pb[2] + approach_h]
        at_xyz = [pb[0], pb[1], pb[2] + drop_dz]
        lift_xyz = [pb[0], pb[1], pb[2] + lift_h]
        # ── AND REFUSE WHAT IT CANNOT REACH ───────────────────────────────
        # dls_ik_pose ALWAYS returns a joint-limit-clamped q, so the old
        # `if q_above is None` guard could never fire for a configured arm and
        # act_place could only ever answer accepted:true. Same gate as
        # act_set_tcp_target: workspace shell, then IK_ACCEPT_TOL_M.
        qs, refusal = self._plan_topdown_path(
            "place", [("approach", above_xyz), ("drop", at_xyz),
                      ("retreat", lift_xyz)], seed, goz)
        if refusal is not None:
            refusal["requested_world_xyz"] = [round(float(v), 4)
                                              for v in p_world]
            refusal["input_frame"] = frame_in
            return refusal
        q_above, q_at, q_lift = qs
        if self._hw_connected():
            # move_linear takes ARM-frame coordinates (act_set_tcp_target
            # forwards target_base_xyz to it), so this had the same bug.
            self.hw.move_linear((at_xyz[0], at_xyz[1], at_xyz[2]))
            self.hw.release()
            return {"accepted": True, "mode": "hardware",
                    "backend": self.hw_name,
                    "commanded_place_world_xyz": [round(float(v), 4)
                                                  for v in p_world],
                    "commanded_place_base_xyz": [round(float(v), 4)
                                                 for v in pb],
                    "input_frame": frame_in}
        if phys:
            # Carry the gripped cube with fresh IK at each waypoint so the wrist
            # stays square and the friction grip holds until the release.
            steps = [
                {"t": "cmove", "xyz": above_xyz, "oz": goz, "dur": duration_s},
                {"t": "cmove", "xyz": at_xyz, "oz": goz, "dur": 0.8},
                {"t": "wait", "dur": 0.2},
                {"t": "release"},
                {"t": "wait", "dur": 0.3},
                {"t": "cmove", "xyz": lift_xyz, "oz": goz, "dur": 0.8},
            ]
        else:
            steps = [
                {"t": "move", "to_q": q_above, "dur": duration_s},
                {"t": "move", "to_q": q_at, "dur": 0.8},
                {"t": "wait", "dur": 0.2},
                {"t": "release"},
                {"t": "wait", "dur": 0.2},
                {"t": "move", "to_q": q_lift or q_above, "dur": 0.8},
            ]
        # PROTOCOL.md 5.4.1 rule 5: claimed only once the whole path is known
        # to be reachable, so a refused place never cancels a running motion.
        # THE MEASURED FAILURE: `pick` then `place` in one turn used to abort
        # the pick mid-sequence and start the place with an empty gripper,
        # both answering accepted:true.
        seq, refusal = self._begin_motion("place")
        if seq is None:
            return refusal
        # THE PART BEING CARRIED IS WHAT GETS MEASURED. Captured here, while
        # it is still welded to the tool -- by the time the sequence ends the
        # release has already cleared held_node, so there would be nothing
        # left to read the landing position off.
        with self.lock:
            held_node, held_tfield = self.held_node, self.held_tfield
        try:
            held_def = held_node.getDef() if held_node is not None else None
        except Exception:
            held_def = None
        if held_tfield is None:
            # ⚠ NO-WELD PATH. Nothing records what a friction grasp picked up,
            # so the part's landing position -- which IS this verb's `achieved`
            # -- was unmeasurable, and place answered "the arm was not carrying
            # a tracked part" even when it had just carried one across the
            # cell. MEASURED 2026-08-16 on the 2F-140 chat world: the block
            # went from the grey pedestal to (0.241, -0.416, 0.245) on the
            # yellow one and the response reported achieved: null.
            hold = self._measure_hold()
            if hold is not None and hold.get("holding"):
                held_def = hold.get("part_def") or held_def
                for node, tf in self._iter_graspables():
                    try:
                        if (node.getDef() or "") == held_def:
                            held_tfield = tf
                            break
                    except Exception:
                        continue
        eta = float(duration_s) + 2.1
        with self.lock:
            self.motion = ("sequence", dict(
                self._slot(seq, "place",
                           measure={"kind": "place", "unit": "m",
                                    "commanded": [float(v) for v in p_world],
                                    "tcp_offset_z": goz,
                                    "frame": "world, metres",
                                    "target": held_def,
                                    "target_field": held_tfield}),
                steps=steps, i=0, from_q=self._read_q(),
                start_s=self.robot.getTime()))
        # PROTOCOL.md 5.4.1 rule 1: this used to return {"place": p} -- the
        # caller's own argument, echoed back under a key that reads like the
        # place that happened. It is a COMMAND; `achieved` is the part's own
        # MEASURED world position once the sequence has actually run it (or
        # null, if it was superseded, timed out, or nothing was being held).
        res = self._wait_result(seq, "place", wait,
                                budget_s=eta * 2.5 + 8.0, eta_s=eta,
                                commanded=[round(float(v), 4)
                                           for v in p_world],
                                unit="m")
        res["commanded_place_world_xyz"] = [round(float(v), 4) for v in p_world]
        res["commanded_place_base_xyz"] = [round(float(v), 4) for v in pb]
        res["input_frame"] = frame_in
        res["carrying_at_command"] = held_def
        res["frame"] = ("commanded/achieved are WORLD frame metres (the part's "
                        "own position); achieved_tcp_base_xyz is arm-base "
                        "frame")
        return res

    def act_locate_objects(self) -> dict:
        """MEASURED roster of every graspable in the scene.

        The agent-facing answer to "where is X" / "what is here". Everything
        is read live off each node's own translation field -- nothing is
        remembered, and nothing is inferred from the action journal, which is
        exactly the substitution that produced a confabulated "it might have
        been dropped somewhere" while the block sat untouched on its pedestal.

        `within_reach` is computed against the same envelope act_pick refuses
        on, so the model can tell "there is no such object" apart from "it is
        there but I cannot get to it" -- two refusals that read identically
        when the only available tool is `pick`.
        """
        out = []
        reach = None
        rmin = 0.0
        try:
            env = self.reach_envelope()
            # ⚠ The keys are max_radius_m / min_radius_m, NOT max_reach_m.
            # A wrong name here fails SILENTLY -- reach stays 0.0, which is
            # falsy, so `within_reach` is quietly omitted from every object
            # and the model loses exactly the distinction this tool adds.
            reach = float(env.get("max_radius_m") or 0.0) or None
            rmin = float(env.get("min_radius_m") or 0.0)
        except Exception:
            reach = None
        for node, tf in self._iter_graspables():
            try:
                p = [float(v) for v in tf.getSFVec3f()]
            except Exception:
                continue
            try:
                d = node.getDef() or ""
            except Exception:
                d = ""
            try:
                nf = node.getField("name")
                nm = nf.getSFString() if nf is not None else None
            except Exception:
                nm = None
            item = {
                "def": d,
                "name": nm or d.replace("GRASP_", "").lower() or None,
                "world_xyz": [round(v, 4) for v in p],
            }
            try:
                pb = self._world_to_base(p)
                dist = math.sqrt(sum(v * v for v in pb))
                item["distance_from_base_m"] = round(dist, 4)
                if reach:
                    item["within_reach"] = bool(rmin <= dist <= reach)
            except Exception:
                pass
            out.append(item)
        res = {"objects": out, "count": len(out), "frame": "world, metres",
               "measured": True}
        if reach:
            res["reach_m"] = round(reach, 4)
        if not out:
            res["note"] = ("no DEF GRASP_* nodes in this scene -- there is "
                           "nothing graspable to pick")
        return res

    def act_node_pose(self, def_name: str) -> dict:
        """World position of a scene node by DEF name (verification/debug)."""
        try:
            node = self.robot.getFromDef(def_name)
            if node is None:
                return {"error": "not_found", "def": def_name}
            p = node.getPosition()
            return {"def": def_name, "pos": [p[0], p[1], p[2]]}
        except Exception as e:
            return {"error": repr(e)}

    def act_wave(self, duration_s: float = 6.0, wait: bool = False) -> dict:
        # PROTOCOL.md 5.4.1 rule 5: a 6 s flourish must not quietly abort a
        # pick that is halfway through its sequence.
        seq, refusal = self._begin_motion("wave")
        if seq is None:
            return refusal
        with self.lock:
            self.motion = ("wave", dict(
                # A gesture has no target pose, so what IS measurable is how
                # long it actually ran for (sim seconds, from the tick that
                # ended it) and the pose it settled back to.
                self._slot(seq, "wave",
                           measure={"kind": "wave", "unit": "s",
                                    "commanded": float(duration_s)}),
                start_s=self.robot.getTime(),
                duration_s=duration_s,
            ))
        # ⚠ A FLOURISH CAN THROW A FRICTION-HELD PART, AND NOTHING USED TO SAY SO.
        # MEASURED 2026-08-16 on the OmniArm 6 2F-140 chat world: asked to "do
        # something impressive" while holding the block, the agent waved; the
        # 6 s joint oscillation flung the block from the gripper to
        # [0.1579, -0.7272, 0.0349] -- on the floor, 0.34 m away -- and the
        # reply discussed joint angles only, never mentioning that the payload
        # it had been carrying was gone. `wave` reported duration and settle
        # pose, both true, and neither of them is the thing that went wrong.
        #
        # This is the same defect class as the `holding: null` bug: a motion
        # verb whose result cannot express the outcome that actually matters,
        # so the model reports the fields it was given and is wrong for it.
        # A weld would have hidden it -- the part cannot be shaken off a
        # kinematic attach -- which is exactly why it only shows up here.
        hold_before = self._measure_hold()
        self._hw_fwd("wave",
                        self.cfg.get("wave_amplitudes")
                        or [0.0] * len(self.joint_names))
        res = self._wait_result(seq, "wave", wait,
                                budget_s=float(duration_s) * 2.0 + 8.0,
                                eta_s=float(duration_s),
                                commanded=float(duration_s), unit="s")
        res["duration_s"] = duration_s
        res.update(self._payload_delta(hold_before))
        return res

    def _payload_delta(self, hold_before: Optional[dict]) -> dict:
        """Did the part survive the motion? MEASURED both sides.

        Returned by any verb that moves the arm WITHOUT intending to change
        what is held, so "I still have it" and "I dropped it somewhere during
        that" stop being indistinguishable. `payload_lost` is the field a
        model should branch on; `payload_moved_m` says how far it went, so a
        part that merely settled in the pads is not reported as a loss.
        """
        out: dict = {}
        try:
            before_def = (hold_before or {}).get("part_def")
            was = bool((hold_before or {}).get("holding"))
            after = self._measure_hold()
            now = bool((after or {}).get("holding"))
            out["payload_before"] = before_def if was else None
            out["payload_after"] = (after or {}).get("part_def") if now else None
            out["payload_retained"] = bool(was and now)
            out["payload_lost"] = bool(was and not now)
            if was and after is not None:
                pb = (hold_before or {}).get("part_world_xyz")
                pa = after.get("part_world_xyz")
                if pb and pa:
                    out["payload_moved_m"] = round(
                        math.sqrt(sum((a - b) ** 2
                                      for a, b in zip(pa, pb))), 4)
                    out["payload_world_xyz"] = pa
            if out["payload_lost"]:
                out["payload_note"] = (
                    "THE PART LEFT THE GRIPPER DURING THIS MOTION. It was not "
                    "released on purpose -- say so plainly, and call "
                    "locate_objects for where it ended up before doing "
                    "anything else with it.")
        except Exception as e:                        # never break a motion
            out["payload_error"] = repr(e)
        return out

    # PROTOCOL.md 5.4.1 rule 1 on the gripper verbs: `state` used to assert
    # the position the fingers were COMMANDED to, under a key that reads as a
    # readback. There is nothing to read back with -- GripperEffector.state()
    # reports `self._width`, its own bookkeeping of the last command, and the
    # finger motors expose no sensor through the effector API -- so the honest
    # answer is a commanded value plus an explicit null, never a number nobody
    # measured. `gripper.object_present` IS a real sensor read where the
    # hardware has one (vacuum); it stays where it is, inside `gripper`.
    # ⚠️ KNOWN DIVERGENCE: PROTOCOL.md 6.5 still spells these replies
    # `{"state": "open"}` / `{"state": "closed"}`. 6.5 and 5.4.1 rule 1
    # contradict each other here, and rule 1 wins -- 6.5 needs updating to
    # `commanded_state` + `achieved_state`.
    def act_open_gripper(self) -> dict:
        if self.effector is None:
            return {"error": "effector_unavailable"}
        st = self.effector.open()
        out = {"commanded_state": "open", "achieved_state": None,
               "gripper": st}
        # Opening the fingers also releases any cube the grasp is holding (the
        # physics-grasp attach or the kinematic weld) so it drops under gravity
        # -- otherwise it stays stuck to the tool even with the fingers wide
        # open. "put it down" / "drop" go via act_release; this covers a bare
        # "open the gripper".
        with self.lock:
            if self.held_node is not None:
                try:
                    out["released"] = self.held_node.getDef()
                    self.held_node.resetPhysics()  # let it fall under gravity
                except Exception:
                    pass
            self.held_node = None
            self.held_tfield = None
        return out

    def act_close_gripper(self) -> dict:
        if self.effector is None:
            return {"error": "effector_unavailable"}
        st = self.effector.close()
        return {"commanded_state": "closed", "achieved_state": None,
                "gripper": st}

    def act_set_gripper_width(self, width_m: float) -> dict:
        if self.effector is None:
            return {"error": "effector_unavailable"}
        return {"gripper": self.effector.set_width(width_m)}

    # ── Kinematic-attach helpers (Phase 3) ────────────────────────
    def _anchor_fallback_once(self, why: str) -> None:
        """Say ONCE, loudly, that the grasp point is coming from FK instead of
        the real mount-link pose. Latched, so nothing is printed per tick and
        nothing at all is printed on the normal (anchor-resolved) path."""
        if getattr(self, "_anchor_warned", False):
            return
        self._anchor_warned = True
        cands = self.cfg.get("mount_link") or ["flange", "tool0",
                                               "gripper_tcp", "tcp"]
        if isinstance(cands, str):
            cands = [cands]
        print(f"[omnilink_arm_bridge] WARNING: grasp anchor unavailable "
              f"({why}); looked for a link named {list(cands)} in {self.robot_id}'s "
              f"subtree. Grasps now measure from FORWARD KINEMATICS, not the "
              f"real mount pose -- the two agree to within the FK chain's own "
              f"error, but a URDF whose tool link is named something else will "
              f"drift.", flush=True)

    def _tcp_world(self) -> Optional[List[float]]:
        """World-frame grasp point. Prefers the REAL mount-link pose
        (flange world pose + tool_reach along its +Z), so the grasp weld
        lands exactly where the gripper is drawn. Falls back to FK
        (base pose x FK TCP, plus THE SAME tool_reach along the FK tool +Z)
        when no anchor is available.

        THE FALLBACK USED TO DROP tool_reach ENTIRELY -- it returned tcp_xyz(),
        i.e. the FLANGE (OmniArm 6 tcp_offset OZ 0.1655), while the anchor branch
        three lines above added tool_reach on top of it. That is a 0.13 m cliff
        (the VACUUM default, omnilink_arm_bridge.py:987) behind one
        `getField("name")` lookup in `_find_mount_node`: replaying
        ArmIdleLoop's own solve over all six warehouse_omnilink feeder pads
        puts the FLANGE 0.1369-0.1411 m from the part at the attach pose
        against a weld point 0.0070-0.0112 m from it, and `_attach_nearest`
        captures only within `grasp_radius` (0.08 m -- VACUUM declares none, so
        the default). So on the fallback every pick on every pad would have
        missed by 1.7-1.8x the capture radius and NOTHING would ever have
        gripped. Latent, not live: the anchor resolves today (omniarm6_gumgrip.urdf
        declares both `flange` and `gripper_tcp`, the two names the OmniArm 6 cfg
        asks for, and the demo completed 6 picks from 6 pads with zero no-grips
        in a2b8331d -- which is only possible on the anchor path).
        `_tcp_world_fast` below already applied the offset correctly; the two
        now agree."""
        node = getattr(self, "gripper_anchor", None)
        if node is not None:
            try:
                p = node.getPosition()
                o = node.getOrientation()  # flat 9, row-major; col2 = tool +Z
                rr = self.tool_reach
                return [p[0] + o[2] * rr, p[1] + o[5] * rr, p[2] + o[8] * rr]
            except Exception:
                self._anchor_fallback_once("the mount node stopped answering")
        else:
            self._anchor_fallback_once("no mount link resolved at startup")
        ik_cfg = self.cfg.get("ik")
        if not ik_cfg or self._self_node is None:
            return None
        # Same construction as _tcp_world_fast: FK to the flange, then step
        # tool_reach along the tool's own +Z (R's third column) before the
        # base->world transform.
        p, R = forward_kinematics_pose(ik_cfg["chain"], self._read_q(),
                                       ik_cfg["tcp_offset"])
        tr = self.tool_reach
        lx = p[0] + R[0][2] * tr
        ly = p[1] + R[1][2] * tr
        lz = p[2] + R[2][2] * tr
        try:
            bp = self._self_node.getPosition()           # world translation
            r = self._self_node.getOrientation()         # 3x3 row-major
        except Exception:
            return [lx, ly, lz]
        return [
            bp[0] + r[0] * lx + r[1] * ly + r[2] * lz,
            bp[1] + r[3] * lx + r[4] * ly + r[5] * lz,
            bp[2] + r[6] * lx + r[7] * ly + r[8] * lz,
        ]

    def _tcp_world_fast(self) -> Optional[List[float]]:
        """IPC-free grasp point for per-tick weld tracking: FK from the
        measured joints (local math) + a ONE-TIME cached base pose. Only
        valid for a static-base arm — used while the ambient idle loop is
        active (the arm never moves its base there). Falls back to the
        anchor path when no IK chain / base pose is available."""
        ik_cfg = self.cfg.get("ik")
        if not ik_cfg or self._self_node is None:
            return self._tcp_world()
        if getattr(self, "_static_base_pose", None) is None:
            try:
                bp = list(self._self_node.getPosition())
                r = list(self._self_node.getOrientation())
            except Exception:
                return self._tcp_world()
            self._static_base_pose = (bp, r)
        bp, r = self._static_base_pose
        p, R = forward_kinematics_pose(ik_cfg["chain"], self._read_q(),
                                       ik_cfg["tcp_offset"])
        tr = self.tool_reach
        lx = p[0] + R[0][2] * tr
        ly = p[1] + R[1][2] * tr
        lz = p[2] + R[2][2] * tr
        return [
            bp[0] + r[0] * lx + r[1] * ly + r[2] * lz,
            bp[1] + r[3] * lx + r[4] * ly + r[5] * lz,
            bp[2] + r[6] * lx + r[7] * ly + r[8] * lz,
        ]

    def _iter_graspables(self):
        """Yield (node, translation_field) for every world node whose DEF
        starts with 'GRASP_'. Walks the scene-tree root children."""
        root = self.robot.getRoot()
        if root is None:
            return
        kids = root.getField("children")
        if kids is None:
            return
        try:
            n = kids.getCount()
        except Exception:
            return
        for i in range(n):
            node = kids.getMFNode(i)
            if node is None:
                continue
            try:
                d = node.getDef() or ""
            except Exception:
                d = ""
            if not d.startswith("GRASP_"):
                continue
            tf = node.getField("translation")
            if tf is not None:
                yield node, tf

    def _measure_hold(self, tfield=None, part_def: Optional[str] = None,
                      radius: Optional[float] = None) -> Optional[dict]:
        """MEASURED answer to "is a part in the gripper", for the NO-WELD path.

        ⚠ WHY THIS EXISTS. `held_node` is set in exactly ONE place --
        `_attach_nearest`, i.e. the kinematic weld. A gripper config carrying
        `assist_weld: False` (robotiq_2f140_grip, the only honest-friction
        entry in _gripper_configs) therefore never sets it, so every `holding`
        this bridge reported on that path was None *by construction* -- not
        "measured and found empty", but "never measured at all".

        MEASURED on chat/omnilink_omniarm6_2f140.omniworld, 2026-08-16: a
        `/prompt "pick up the block"` lifted DEF GRASP_BLOCK from z=0.2449 to
        z=0.3980 -- a real 153 mm friction lift, the block airborne and
        tracking the tool -- and the model answered "my gripper didn't catch
        it and I'm not holding anything", because `holding: null` was the only
        evidence it had. The physics was right and the sentence was wrong,
        which is the tool-design failure mode AGENTS.md names: an LLM's
        honesty is bounded above by the honesty of its tools, and no prompt
        raises that ceiling.

        What is measured is the one thing a weld cannot fake: the part's LIVE
        world translation against the tool point. A dropped part leaves the
        pads at once and the distance grows, so proximity that persists
        through a carry is a real hold. Returns None only when the tool point
        itself is unknown; otherwise `holding` is always a measured bool and
        `evidence` says in millimetres why.
        """
        tcp = self._tcp_world()
        if tcp is None:
            return None
        r = float(self.grasp_radius if radius is None else radius)
        if tfield is not None:
            cands = [(None, tfield)]
        else:
            cands = list(self._iter_graspables())
        best = None
        best_d = float("inf")
        for node, tf in cands:
            try:
                p = [float(v) for v in tf.getSFVec3f()]
            except Exception:
                continue
            d = math.sqrt(sum((p[k] - tcp[k]) ** 2 for k in range(3)))
            if d < best_d:
                best_d, best = d, (node, tf, p)
        out = {
            "tcp_world_xyz": [round(float(v), 4) for v in tcp],
            "hold_radius_m": round(r, 4),
            "mechanism": ("kinematic_weld"
                          if (self.gripper_cfg or {}).get("assist_weld", True)
                          else "friction"),
        }
        if best is None:
            out["holding"] = False
            out["part_def"] = part_def
            out["evidence"] = "no DEF GRASP_* node in the scene to measure"
            return out
        node, _tf, p = best
        try:
            pdef = (node.getDef() if node is not None else None) or part_def
        except Exception:
            pdef = part_def
        out["part_def"] = pdef
        out["part_world_xyz"] = [round(float(v), 4) for v in p]
        out["offset_m"] = round(best_d, 4)
        out["holding"] = bool(best_d <= r)
        out["evidence"] = (
            "part centre is %.1f mm from the tool point (hold radius %.0f mm)"
            % (best_d * 1000.0, r * 1000.0))
        return out

    def _attach_nearest(self, tcp: List[float]) -> Optional[str]:
        """Weld the nearest GRASP_ object within grasp_radius to the tool."""
        best = None
        best_d = self.grasp_radius
        for node, tf in self._iter_graspables():
            try:
                p = tf.getSFVec3f()
            except Exception:
                continue
            d = math.sqrt((p[0] - tcp[0]) ** 2 + (p[1] - tcp[1]) ** 2
                          + (p[2] - tcp[2]) ** 2)
            if d <= best_d:
                best_d = d
                best = (node, tf)
        if best is None:
            return None
        self.held_node, self.held_tfield = best
        try:
            self.held_node.resetPhysics()
        except Exception:
            pass
        try:
            got = self.held_node.getDef()
        except Exception:
            got = "GRASP_object"
        # THE `on_next_pickup` TRIGGER. Fires for an autonomous pick and an
        # operator-commanded grasp alike -- "tell me before you move another
        # part" must not care which.
        if getattr(self, "intents", None) is not None:
            # safe=True on purpose: "tell me BEFORE you move it" has to hold
            # the arm at exactly the moment it is holding a part.
            self.intents.note_event("pickup", got, safe=True)
        return got

    def act_grasp(self, force: Optional[float] = None,
                  width: Optional[float] = None) -> dict:
        # The real arm's gripper is driven by the hardware backend even when no
        # sim effector is configured (e.g. a bare arm demo with no gripper).
        self._hw_fwd("grasp")
        if self.effector is None:
            if self._hw_connected():
                return {"gripper": {"kind": "hardware", "holding": True},
                        "mode": "hardware", "backend": self.hw_name}
            return {"error": "effector_unavailable"}
        gstate = self.effector.grasp(force=force, width=width)
        out = {"gripper": gstate}
        # ⚠ WHAT ACTUALLY HOLDS THE PART. By default this bridge does NOT hold
        # objects with contact: `_attach_nearest` WELDS the nearest DEF GRASP_*
        # node to the tool and teleports it to the TCP every tick. The fingers
        # close, and the pinch is visible, but the hold is kinematic.
        #
        # This used to be described as "contact friction does the grabbing",
        # including on the `physics_grasp` path, which also welds. It is not a
        # small wording problem: under a weld, "the gripper held the block for
        # ten seconds" is UNFALSIFIABLE -- it would hold with the fingers wide
        # open, for ever -- so any demo built on it cannot demonstrate a grasp.
        # Measured 2026-08-02: a real friction pinch of the same block on the
        # same gripper (OmniArm 6 + Robotiq 2F-85, newtonSolver "mujoco") held for
        # **1.376 s** before slipping 15 mm, against the 10 s the demos appear
        # to achieve.
        #
        # The weld stays ON by default, because the interactive demos are built
        # on it and silently breaking them would be its own dishonesty. What
        # changes is that it is now NAMED in the response and can be switched
        # off: set `assist_weld: False` on the gripper config (or pass
        # `assist_weld=false` in the request) to get the physics-only hold.
        cfg = self.gripper_cfg or {}
        assist = cfg.get("assist_weld", True)
        physics_mode = bool(cfg.get("physics_grasp"))
        out["mode"] = "physics" if physics_mode else "kinematic"
        tcp = self._tcp_world()
        if not assist:
            out["hold_mechanism"] = "friction"
            out["hold_mechanism_detail"] = (
                "no weld: the fingers and contact friction are the only thing "
                "holding the part, so a hold that persists is a real one")
            out["attached"] = None
            if tcp is not None:
                out["tcp_world"] = tcp
            return out
        out["hold_mechanism"] = "kinematic_weld"
        out["hold_mechanism_detail"] = (
            "the part is WELDED to the tool and teleported to the TCP each "
            "tick; the finger pinch is cosmetic. A hold time measured under "
            "this mode is not evidence of a grasp. Set assist_weld=false for "
            "a physics-only hold.")
        if tcp is not None:
            with self.lock:
                attached = self._attach_nearest(tcp)
            out["attached"] = attached
            out["tcp_world"] = tcp
            if attached is None:
                out["note"] = "no graspable (DEF GRASP_*) within grasp_radius"
        return out

    def act_release(self) -> dict:
        self._hw_fwd("release")
        if self.effector is None:
            if self._hw_connected():
                return {"gripper": {"kind": "hardware", "holding": False},
                        "released": None, "mode": "hardware",
                        "backend": self.hw_name}
            return {"error": "effector_unavailable"}
        gstate = self.effector.release()
        with self.lock:
            dropped = None
            if self.held_node is not None:
                try:
                    dropped = self.held_node.getDef()
                    self.held_node.resetPhysics()  # let it fall under gravity
                except Exception:
                    pass
            self.held_node = None
            self.held_tfield = None
        return {"gripper": gstate, "released": dropped}

    def hw_status(self) -> dict:
        """Hardware-link status. {"enabled": False} in a pure-sim run."""
        if self.hw is None:
            return {"enabled": False}
        st = dict(self.hw.status())
        st.setdefault("backend", self.hw_name)
        return st

    def _paused_by(self) -> str:
        """Which of the three pause sources is holding the idle loop, in the
        same precedence ArmIdleLoop._blocked applies."""
        loop = getattr(self, "idle_loop", None)
        if loop is None:
            return ""
        if self.intents is not None and self.intents.hold_active():
            return "operator hold"
        if time.time() < self.stop_hold_until:
            return "operator stop"
        if (time.time() - self.last_external_cmd) < loop.resume_s:
            return "recent command"
        return ""

    def get_state(self) -> dict:
        q = self._read_q()
        tcp = self.tcp_xyz()
        out = {
            "id": self.robot_id,
            "model": self.cfg["model"],
            "q": q,
            "tcp": list(tcp) if tcp else None,
            "gripper": self.effector.state() if self.effector else None,
            "fault": self.fault,
            "last_tick_at": self.last_tick_at,
            "sim_time": self.robot.getTime(),
            "mode": self.motion[0],
            "hardware": self.hw_status(),
            # WHAT THE LAST FINISHED MOTION ACTUALLY DID (PROTOCOL.md 5.4.1).
            # None until one completes. This is how a caller that passed
            # wait=false learns the achieved value instead of assuming it --
            # before this block existed there was nothing to poll, so "return
            # now and check later" was not even an option the model had.
            # Match `seq` EXACTLY against the seq the command returned: a
            # later command's record measures a different motion.
            "last_command": self.last_completion,
        }
        # DEFERRED INTENTS, top level and always present. A pending intent
        # has to be VISIBLE, not implied -- this is what get_robot_state's
        # description points the model at.
        if self.intents is not None:
            out.update(self.intents.state())
        if self.idle_loop is not None:
            out["idle_loop"] = {
                "mode": "pick",
                "picks": self.idle_loop.picks,
                "leg": getattr(self.idle_loop, "leg", "idle"),
                "paused": self.idle_loop._blocked(),
                # WHY it is paused, and for how long. `paused` alone made an
                # operator stop indistinguishable from a stale quiet window,
                # so the model could not tell the operator whether the arm
                # would start again on its own.
                "paused_by": self._paused_by(),
                "resumes_in_s": (
                    round(self.stop_hold_until - time.time(), 1)
                    if time.time() < self.stop_hold_until else None),
            }
        # THE COORDINATION BACKBONE: the line master's public state. The MAV
        # idle loops poll this block to decide when a loaded trolley is ready
        # for dispatch and when a delivered load has shipped (freeing its
        # trolley for the return leg). Absent in non-line worlds.
        if self.line is not None:
            st = self.line.status()
            if st.get("active"):
                out["line"] = st
        return out

    # ── quantitative introspection ────────────────────────────────
    #
    # The counts below were ALREADY tracked (the fill box's placed/target,
    # the belt queue, the loads in transit, the feeder pads the idle loop
    # walks every cycle). Nothing new is measured here; they were simply not
    # reachable from a tool, so "how many parts are left in the feeder?" was
    # answered with "I don't have a direct sensor reading" plus reassurance.
    # A count that exists and is not exposed is a hedge waiting to happen.

    def line_report(self) -> dict:
        out: Dict[str, Any] = {
            "line_master": bool(getattr(self, "line_master", False)),
            "not_tracked": [],
        }
        loop = getattr(self, "idle_loop", None)
        if loop is not None and hasattr(loop, "feeder_status"):
            out["feeder"] = loop.feeder_status()
            out["picks_completed"] = loop.picks
        else:
            out["feeder"] = {"known": False,
                             "say": "I have no feeder tray to count."}
        ln = getattr(self, "line", None)
        st = ln.status() if ln is not None else {}
        if st.get("active"):
            out["line"] = st
        else:
            out["line"] = {"active": False}
            out["not_tracked"].append(
                "box/cart counts (this world has no production line)")
        if self.intents is not None:
            out["progress"] = self.intents.progress()
        # THE HONEST LIST. Naming what is NOT tracked is what stops the model
        # filling the silence: it can say "I don't track that" with the same
        # confidence it reports a number.
        out["not_tracked"] += [
            "where the tugs are (I publish the line; I do not watch them — "
            "ask tug_a or tug_b directly)",
            "parts consumed by anything outside this cell",
        ]
        out["say"] = self._line_say(out)
        return out

    def _line_say(self, r: dict) -> str:
        bits = []
        f = r.get("feeder") or {}
        if f.get("known"):
            bits.append(f"{f['at_pad']} part(s) on the feeder tray")
        ln = r.get("line") or {}
        if ln.get("active"):
            if ln.get("placed") is not None:
                bits.append(f"{ln['placed']} of {ln['target']} parts in the "
                            f"box at the fill stop")
            bits.append(f"{ln.get('queued', 0)} box(es) queued behind it")
            bits.append(f"{ln.get('loads_out', 0)} load(s) out on carts")
            bits.append(f"{ln.get('shipped_total', 0)} shipped so far")
        if not bits:
            return "I have no line counts to give you."
        return "Right now: " + "; ".join(bits) + "."


# ── Intent router ────────────────────────────────────────────────────

class IntentRouter:
    """Maps free-text prompts to bridge actions.

    Designed for demo prompts -- "go home", "wave hello", "joint 3 to 1",
    "move to 0.4 0.2 0.3", "open the gripper", "stop". Returns a result
    dict the bridge surfaces as agent + tool lines in the robot window.
    """

    NUMBER = r"(-?\d+\.?\d*)"

    def __init__(self, bridge: ArmBridge):
        self.bridge = bridge

    def dispatch(self, text: str) -> dict:
        s = text.strip().lower()
        if not s:
            return {"agent": "(empty prompt)", "tools": []}

        # ── resume / carry on ───────────────────────────────────
        # Checked BEFORE everything else: "back to work" contains
        # "back", "keep going" contains "go", and several resume
        # phrasings would otherwise be swallowed by a motion verb. This
        # is the counterpart to "stop" -- without it an operator can
        # halt the arm from chat and has no way to restart it short of
        # waiting out the quiet-window timer.
        if shared_is_resume is not None and shared_is_resume(s):
            res = self.bridge.act_resume_autonomy()
            if res.get("autonomy") == "none":
                return {
                    "agent": "I have no autonomous loop to resume — "
                             "I only move when you tell me to.",
                    "tools": [("resume_autonomy", "ok", "no idle loop")],
                }
            return {
                "agent": f"Back on it — resuming my pick loop now "
                         f"({res.get('picks', 0)} picks done so far).",
                "tools": [("resume_autonomy", "ok",
                           f"autonomy={res.get('autonomy')}")],
            }

        # ── "what are you doing right now?" ─────────────────────
        # The plain-English form of the status query. The legacy
        # `status|state|where|pose` intent further down answers with raw
        # joint angles, which is the wrong answer to this question and
        # never fired for this phrasing anyway (none of its keywords
        # appear in "what are you doing right now?"). Answered from the
        # real /state dict so the offline reply is as grounded as the
        # LLM one.
        if shared_is_status is not None and shared_is_status(s):
            st = self.bridge.get_state_for_query()
            return {
                "agent": shared_describe_state(st),
                "tools": [("get_robot_state", "ok",
                           str((st.get("line") or {}).get("fill_state")
                               or st.get("mode")))],
            }

        # ── "what's your job / what are you filling?" ───────────
        # The assignment question. Answered from the bridge's real role
        # (line master or not, what the idle loop is configured to do),
        # not from a canned blurb.
        if re.search(r"\b(your (job|role|task|assignment|purpose)|"
                     r"what do you do|what are you for|"
                     r"what are you (filling|kitting|building|making)|"
                     r"where (does|do) (it|they|the box|the boxes) go|"
                     r"what'?s your (job|role|purpose))\b", s):
            return {
                "agent": self.bridge.describe_role(),
                "tools": [("get_robot_state", "ok", "role brief")],
            }

        # ── learn a new skill (skill learning) ───────────────────
        # Checked FIRST among the action intents: "learn to pick up the
        # cube" must reach the factory, not the built-in pick regex.
        if re.search(r"\b(learn|teach yourself)\b", s):
            res = self.bridge.act_learn(s)
            started = bool(res.get("started"))
            return {
                "agent": res.get("message", ""),
                "tools": [("learn_skill", "ok" if started else "err",
                           res.get("recipe") or res.get("error", ""))],
            }

        # ── stop / halt ─────────────────────────────────────────
        if re.search(r"\b(stop|halt|freeze|hold)\b", s):
            # OFFLINE ROUTER, SAME CONTRACT. "Stopping." was a sentence
            # written before anything was measured; say what the stop
            # MEASURED, or say plainly that rest could not be confirmed.
            res = self.bridge.act_stop()
            still = res.get("stationary")
            m = res.get("measured") or {}
            if still is True:
                agent = (f"Stopped — frozen at {self._fmt_q(res['q'])}, "
                         f"measured {m['max_joint_radps']:.3f} rad/s over "
                         f"{m['over_s']:.2f} s, so it is standing still.")
                summary = f"stationary, {m['max_joint_radps']:.3f} rad/s"
            elif still is False:
                agent = (f"Freeze commanded, but it is STILL MOVING: joint "
                         f"{m.get('joint')} at "
                         f"{m['max_joint_radps']:.3f} rad/s "
                         f"{m['over_s']:.2f} s after the halt.")
                summary = f"NOT stationary, {m['max_joint_radps']:.3f} rad/s"
            else:
                agent = (f"Freeze commanded at {self._fmt_q(res['q'])}. I "
                         "could not confirm it came to rest — "
                         + str(m.get("reason", "not measured")) + ".")
                summary = "rest unconfirmed"
            hold = res.get("idle_loop") or {}
            if hold.get("present"):
                agent += (f" Holding for {hold['hold_s']:.0f} s, then the "
                          f"pick loop resumes on its own.")
            return {
                "agent": agent,
                "tools": [("stop_robot", "ok", summary)],
            }

        # ── home / reset ────────────────────────────────────────
        if re.search(r"\b(home|reset|park|tuck)\b", s) or "go to home" in s:
            res = self.bridge.act_reset_to_home()
            # `q` became `commanded_q` (PROTOCOL.md 5.4.1 rule 1: the home
            # pose is a command, not the measurement get_state's `q` is).
            n = len(res.get("commanded_q") or self.bridge.home_pose)
            return {
                "agent": f"Moving to home pose ({n} joints).",
                "tools": [("reset_to_home", "ok", "interpolating 1.5 s")],
            }

        # ── wave / dance ────────────────────────────────────────
        if re.search(r"\b(wave|hello|dance|demo|show ?off)\b", s):
            res = self.bridge.act_wave()
            return {
                "agent": "Waving hello — give me ~6 seconds.",
                "tools": [("wave", "ok", "0.8 Hz oscillation")],
            }

        # ── pick / place / grasp / release / width / open / close ───
        # "pick up [the] [colour] cube" -> reach top-down, grasp, lift
        if re.search(r"\b(grab|grasp)\b", s) or ("pick" in s and "up" in s):
            name = None
            mc = re.search(r"\b(red|blue|green|yellow|orange|purple)\b", s)
            if mc:
                name = mc.group(1)
            res = self.bridge.act_pick(name)
            ok = "error" not in res
            tgt = res.get("target") or (name or "the nearest object")
            return {
                # A refusal now carries its own sentence (the reach gate);
                # relay it rather than the generic line, the way the
                # set_tcp_target branch below already does.
                "agent": ((f"Picking up {tgt}." if ok else res.get("say"))
                          or "I couldn't plan that pick."),
                "tools": [("pick", "ok" if ok else "err",
                           str(res.get("pos") or res.get("error", "")))],
            }
        # "put it down" / "place it" / "set it down" -> carry to drop zone + release
        if re.search(r"\b(put (it |that )?(down|away|back)|place|set (it |that )?down|drop (it )?off)\b", s):
            res = self.bridge.act_place()
            ok = "error" not in res
            return {
                "agent": (("Setting it down." if ok
                           else res.get("say"))
                          or "I couldn't plan that place."),
                "tools": [("place", "ok" if ok else "err",
                           str(res.get("commanded_place_base_xyz")
                               or res.get("error", "")))],
            }
        # "release" / "let go" / "drop it" -> open + drop in place
        if re.search(r"\b(release|let go|drop)\b", s):
            res = self.bridge.act_release()
            ok = "error" not in res
            return {
                "agent": "Releasing." if ok else "This arm has no gripper.",
                "tools": [("release", "ok" if ok else "err", self._gsum(res))],
            }
        # set width -> "open to 3 cm", "40 mm", "halfway"
        if "gripper" in s or re.search(r"\b(width|wide|halfway|half)\b", s):
            w = self._parse_width(s)
            if w is not None:
                res = self.bridge.act_set_gripper_width_checked(w)
                ok = "error" not in res
                return {
                    "agent": (f"Setting gripper to {w * 1000:.0f} mm." if ok
                              else res.get("say",
                                           "This gripper has no width control.")),
                    "tools": [("set_gripper_width", "ok" if ok else "err",
                               self._gsum(res))],
                }
        # open / close
        if re.search(r"\b(open)\b", s) and "gripper" in s or s.strip() == "open":
            res = self.bridge.act_open_gripper()
            ok = "error" not in res
            return {
                "agent": "Opening the gripper." if ok else "This arm has no gripper.",
                "tools": [("open_gripper", "ok" if ok else "err",
                           res.get("commanded_state") or res.get("error", ""))],
            }
        if re.search(r"\b(close|grip)\b", s) and "gripper" in s or s.strip() == "close":
            res = self.bridge.act_close_gripper()
            ok = "error" not in res
            return {
                "agent": "Closing the gripper." if ok else "This arm has no gripper.",
                "tools": [("close_gripper", "ok" if ok else "err",
                           res.get("commanded_state") or res.get("error", ""))],
            }

        # ── move / go to (x y z) ────────────────────────────────
        m = re.search(
            r"(?:go|move|tcp|target)[^-\d]*"
            r"\(?\s*" + self.NUMBER + r"[ ,]+" + self.NUMBER + r"[ ,]+" + self.NUMBER + r"\s*\)?",
            s,
        )
        if m:
            xyz = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
            res = self.bridge.act_set_tcp_target(xyz)
            if "error" in res:
                # The offline path gets the same honest sentence the LLM
                # path relays -- a refusal is a refusal in both modes.
                return {
                    "agent": res.get("say") or f"Can't reach {xyz}: {res['error']}",
                    "tools": [("set_tcp_target", "err", res["error"])],
                }
            # The IK residual, named as such -- it is a solver number from
            # before the move, and the offline router does not wait, so it has
            # no measurement to quote. (The tool surface does: it defaults to
            # wait=true and reports `error_m`.)
            resid = float(res.get("ik_residual_m") or 0.0)
            return {
                "agent": (f"Moving to TCP {xyz} (IK residual {resid:.4f} m; "
                          "the move takes ~1.5 s)."),
                "tools": [("set_tcp_target", "ok", f"ik_residual={resid:.4f}")],
            }

        # ── joint N to V ────────────────────────────────────────
        m = re.search(r"joint\s*(\d+)\s*(?:to|=)\s*" + self.NUMBER, s)
        if m:
            idx = int(m.group(1)) - 1
            val = float(m.group(2))
            if not (0 <= idx < len(self.bridge.joint_names)):
                return {
                    "agent": f"Joint index out of range (have {len(self.bridge.joint_names)} joints).",
                    "tools": [("set_joint_positions", "err", "index out of range")],
                }
            q = list(self.bridge._read_q())
            q[idx] = val
            res = self.bridge.act_set_joint_positions_checked(q)
            if not res.get("accepted"):
                return {
                    "agent": res.get("say", "I can't move to that angle."),
                    "tools": [("set_joint_positions", "err",
                               res.get("error", "refused"))],
                }
            return {
                "agent": f"Setting joint {idx + 1} to {val:.3f} rad.",
                "tools": [("set_joint_positions", "ok", self._fmt_q(res["clamped_q"]))],
            }

        # ── joints to [a b c d e f] ─────────────────────────────
        m = re.search(r"joints?\s*(?:to|=)\s*\[?\s*([-\d. ,]+?)\s*\]?$", s)
        if m:
            try:
                q = [float(x) for x in re.split(r"[,\s]+", m.group(1)) if x]
                if len(q) == len(self.bridge.joint_names):
                    res = self.bridge.act_set_joint_positions_checked(q)
                    if not res.get("accepted"):
                        return {
                            "agent": res.get("say",
                                             "I can't move to those angles."),
                            "tools": [("set_joint_positions", "err",
                                       res.get("error", "refused"))],
                        }
                    return {
                        "agent": f"Moving all {len(q)} joints.",
                        "tools": [("set_joint_positions", "ok", self._fmt_q(res["clamped_q"]))],
                    }
            except Exception:
                pass

        # ── status / state ──────────────────────────────────────
        if re.search(r"\b(status|state|where|pose|telemetry)\b", s):
            st = self.bridge.get_state()
            q_s = self._fmt_q(st["q"])
            tcp_s = (f", TCP=({st['tcp'][0]:.2f}, {st['tcp'][1]:.2f}, {st['tcp'][2]:.2f})"
                     if st["tcp"] else "")
            return {
                "agent": f"q={q_s}{tcp_s}, mode={st['mode']}.",
                "tools": [("get_robot_state", "ok", q_s)],
            }

        # ── learned verbs ("toss", "toss it", "do it") ──────────
        # After the built-ins (a learned verb never shadows a shipped
        # command) but before the unknown fallback.
        verb = self._match_learned_verb(s)
        if verb is not None:
            res = self.bridge.act_run_learned(verb)
            ok = "error" not in res
            agent = (f"Running the learned '{verb}' skill "
                     f"(~{res.get('duration_s', 0):.1f} s)."
                     if ok else f"I can't run '{verb}': {res.get('error')}.")
            if ok and res.get("missing_props_note"):
                agent += " Note: " + res["missing_props_note"] + "."
            return {
                "agent": agent,
                "tools": [(verb, "ok" if ok else "err",
                           res.get("missing_props_note")
                           or f"{res.get('samples', '?')} samples"
                           if ok else str(res.get("error", "")))],
            }

        # ── unknown ─────────────────────────────────────────────
        # Offline regex router (no OmniLink LLM attached). Say so, so the
        # operator knows free-form chat needs OMNI_KEY, and list commands
        # that actually work -- including the pick/place verbs.
        learned = sorted(getattr(self.bridge, "learned_skills", {}).keys())
        learned_s = (" Learned skills: " + ", ".join(f'"{v}"' for v in learned)
                     + "." if learned else "")
        learn_hint = (" You can also say \"learn to toss the cube into the "
                      "bin\" to teach me a new skill."
                      if getattr(self.bridge, "learn", None) is not None else "")
        return {
            "agent": ("I'm on the offline command router (no OmniLink agent "
                      "connected), so I only understand set phrases. Try: "
                      "\"pick up the red cube\", \"put it down\", \"wave\", "
                      "\"go home\", \"stop\", \"move to 0.4 0.2 0.3\", "
                      "\"joint 3 to 1.5\", or \"open the gripper\"."
                      + learned_s + learn_hint),
            "tools": [],
        }

    def _match_learned_verb(self, s: str) -> Optional[str]:
        """Which learned verb (if any) the prompt invokes. "do it" /
        "do that (again)" re-runs the most recently learned verb."""
        skills = getattr(self.bridge, "learned_skills", {})
        if not skills:
            return None
        for v in sorted(skills.keys()):
            if re.search(r"\b" + re.escape(v) + r"\b", s):
                return v
        if re.search(r"\b(do (it|that)( again)?|again|one more time)\b", s):
            return getattr(self.bridge, "last_learned_verb", None)
        return None

    @staticmethod
    def _fmt_q(q: List[float]) -> str:
        return "[" + ", ".join(f"{qi:+.2f}" for qi in q) + "]"

    @staticmethod
    def _gsum(res: dict) -> str:
        """One-line gripper summary for a tool-result line."""
        if "error" in res:
            return res["error"]
        g = res.get("gripper") or {}
        w = g.get("width")
        bits = [g.get("kind", "gripper")]
        if w is not None:
            bits.append(f"{w * 1000:.0f}mm")
        if g.get("holding"):
            bits.append("holding")
        return " ".join(bits)

    def _parse_width(self, s: str) -> Optional[float]:
        """Parse a target opening width (metres) from free text.

        Understands "40 mm" / "3 cm" / "0.04 m" and the words
        "halfway" / "half" (-> half of max_width). Returns None if no
        width is expressed or the gripper has no width control."""
        eff = self.bridge.effector
        if eff is None or eff.max_width <= 0.0:
            return None
        if re.search(r"\b(halfway|half)\b", s):
            return eff.max_width * 0.5
        m = re.search(r"(-?\d+\.?\d*)\s*(mm|millimet|cm|centimet|m\b|metre|meter)", s)
        if not m:
            return None
        val = float(m.group(1))
        unit = m.group(2)
        if unit.startswith("mm") or unit.startswith("millimet"):
            return val / 1000.0
        if unit.startswith("cm") or unit.startswith("centimet"):
            return val / 100.0
        return val  # metres


# ── Warehouse line master (auto-armed with --idle-loop pick) ─────────

class WarehouseLine:
    """The pick cell's LINE MASTER: runs the box conveyor, the box->trolley
    load glide, and the dispatch-side ship/recycle — the machinery that
    turns the warehouse_omnilink idle demo into a CLOSED production loop:

        conveyor brings a box -> arm fills it from the feeder -> the filled
        box glides onto the empty trolley at the fill station -> tug_a tows
        it to the dispatch bay -> the load "ships" (box recycles to the
        belt entry, parts respawn on the feeder) -> tug_b returns the
        emptied trolley -> repeat, indefinitely.

    Auto-arms only when the world carries DEF BOX_* top-level Solids AND a
    DEF FILL_STOP marker (i.e. warehouse_omnilink); in every other world it
    stays inactive and costs nothing. Geometry is read from the world:
    FILL_STOP gives the belt stop, the boxes give the belt line, the first
    --idle-drop trolley's spawn pose gives the fill-station spot.

    Box transport is the proven anypick-line kinematic-ensemble recipe:
    per-tick pose write + velocity zeroing while a box advances (a narrow
    static belt strip bears its weight between corrections), pin-on-drift
    while it queues, and a rider-capturing glide (box + the parts inside,
    rigid) for the belt -> trolley-deck load.

    Threading: tick() runs on the SIM THREAD (called from ArmBridge.tick),
    so all supervisor access here is direct. The idle pick loop (a worker
    thread) only touches the small mutable state through fill_target_info /
    note_placed, guarded by _slock. status() feeds the /state "line" block
    the MAV loops poll — the coordination backbone of the choreography.

    Crash safety: every phase is fail-soft (log-and-continue); the line
    never takes the bridge down.
    """

    CONVEY_SPEED = 0.30      # m/s box advance along the belt
    QUEUE_GAP = 0.75         # m centre-to-centre box queue spacing
    GLIDE_SPEED = 0.35       # m/s box load glide belt -> trolley deck
    PIN_DEADBAND = 0.012     # m drift before a stopped box is re-pinned
    SHIP_AFTER_S = 12.0      # s a delivered load rests at dispatch, then ships
    # Parts per box, cycled. WAS (3, 2), which commits 3+2+3 = 8 parts per
    # lap of the 3-box queue against a feeder stock of SIX -- so the arm
    # emptied the tray and then stood still (measured: TCP motionless 482 s
    # of a 605 s run, in blocks of 58-114 s), because parts only return
    # inside _ship, which waits on a tug towing a cart >6 m out.
    # The first attempt at this added a 4th part column to the tray. That
    # BROKE THE PICK -- the wider tray sat inside the arm's own swept volume
    # and five of seven pads stopped gripping (see the FEEDER comment in
    # warehouse_omnilink.omniworld). Stock is bounded by the arm's reach, so the
    # fix has to come out of the CONSUMPTION side instead: 2+2+2 = 6 per lap
    # against 6 in stock, break-even rather than a deficit, with no geometry
    # change. Still within the "2-3 parts per box" brief.
    FILL_TARGETS = (2, 2)
    TROLLEY_TOL = 0.45       # trolley within this xy of the fill spot = present
    DEPART_DIST = 1.2        # loaded box this far from the fill spot = departed
    # A delivered box must be at least this far from the fill spot before its
    # ship timer runs. This is a HARD CONTRACT WITH THE WORLD'S GEOMETRY, and
    # the number is set by the furthest stop that is still INSIDE the line:
    # the loaded cart now rides the fill conveyor out to DEF CART_PICKUP,
    # 4.60 m south of the fill spot, and a box standing there is NOT
    # delivered -- it has not left the pick cell's own column yet. The old
    # 2.5 m cleared the conveyor station (2.05 m) but would read the pickup as
    # a dispatch bay, ship the load on the spot, and recycle the box to the
    # belt entry out from under the cart that was about to carry it. 6.0 m
    # clears the pickup with 1.4 m to spare; the nearest real dispatch
    # position (PARK_SPOT_1) is 13.9 m out, so nothing legitimate is
    # excluded. If the pickup ever moves further south, this moves with it.
    SHIP_MIN_DIST = 6.0      # delivered box must be at least this far out
    DECK_TOP = 0.327         # trolley deck top (m) the box lands on
    # The load glide runs down a physical OUTFEED SPUR (a short transfer
    # conveyor in the world) instead of cutting across open floor, so the box
    # is over a surface for the whole trip belt -> cart. OUTFEED_DX is how far
    # WEST of the fill stop that spur's centreline sits; the world's
    # DEF OUTFEED_SPUR is built to exactly fill_x - OUTFEED_DX. Worlds without
    # the spur still work (the route is just an L across the floor), so this
    # stays a plain constant rather than a world read.
    OUTFEED_DX = 0.40
    # Final approach (m) over which the box settles from conveyor height onto
    # the deck. Kept <= the deck half-width so the descent happens entirely
    # ABOVE the deck -- the box holds belt height right to the spur's
    # discharge end and never sinks through thin air.
    DECK_SETTLE = 0.35
    CHECK_PERIOD = 8         # ticks between trolley / transit polls
    # Millimetres of air held under a body while it is being TELEPORTED (belt
    # advance, load glide). A body teleported into steady contact makes the
    # contact solver fight the write every tick and the whole sim drops below
    # realtime (measured on the tow leg: 0.999x -> 0.166x). Riders get strictly
    # more than the box they sit in, so no layer rests on another mid-move.
    MOVE_LIFT = 0.004        # box hover while it is being moved
    RIDER_LIFT = 0.008       # part hover while riding a moving box

    def __init__(self, bridge: "ArmBridge",
                 trolley_defs: Optional[List[str]] = None) -> None:
        self.bridge = bridge
        self.trolley_defs = [d for d in (trolley_defs or []) if d]
        self._slock = threading.Lock()
        self.active = False
        self._init_done = False
        self._tick_i = 0
        self.boxes: Dict[str, dict] = {}
        self.queue: List[str] = []       # box defs on the belt, front first
        self.fill_x = 0.0
        self.belt_y = 0.0
        self.belt_z = 0.0
        self.entry_x = 0.0
        self.fill_spot = (0.0, 0.0)      # trolley fill-station spot (xy)
        self._trolleys: Dict[str, Any] = {}
        self._troll_hist: Dict[str, list] = {}
        self._glide: Optional[dict] = None
        self._transit: List[dict] = []
        self._fill_ti = 0
        self._parts_spawn: Dict[str, tuple] = {}
        self._contents: Dict[str, List[str]] = {}
        self._placed = 0
        self._target = self.FILL_TARGETS[0]
        # Monotonic tally of loads that have SHIPPED from the dispatch bay --
        # the line's "deliveries done" number. The event was already logged;
        # counting it is what makes "how many have you shipped?" answerable
        # with a number instead of a shrug.
        self._shipped = 0
        self._boxes_filled = 0
        self._wait_logged = 0.0
        self._err_logged = 0.0
        self._hb_last = 0.0
        self._hb_every = (10.0 if _os.environ.get("OMNILINK_LINE_HEARTBEAT")
                          else 0.0)
        self.pub: Dict[str, Any] = {"active": False}

    # ── logging ───────────────────────────────────────────────────

    def _log(self, msg: str) -> None:
        try:
            t = self.bridge.robot.getTime()
        except Exception:
            t = -1.0
        # SIM time AND wall time: with both stamped on every event, the
        # realtime factor of any individual phase (tow leg, pick, glide) can
        # be derived offline from one run -- no polling loop charging its own
        # HTTP latency to the thing it is trying to measure.
        line = f"[line] t={t:.1f}s w={time.time() - _T0:.1f}s {msg}"
        print(line, flush=True)
        path = _os.environ.get("OMNILINK_IDLE_LOG")
        if path:
            try:
                with open(path, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            except OSError:
                pass

    # ── worker-thread API (idle pick loop) ────────────────────────

    def fill_target_info(self):
        """(box_def, node, placed, target) while an unfilled box is stopped
        at the fill station; None otherwise. Safe from any thread."""
        with self._slock:
            if not self.queue:
                return None
            bd = self.queue[0]
            b = self.boxes.get(bd)
            if b is None or b["state"] != "at_fill":
                return None
            return (bd, b["node"], self._placed, self._target)

    def note_placed(self, part_def: str):
        """Record one completed placement into the fill box. Returns
        (box_def, placed, target) or None. Marks the box FILLED when the
        target is reached (no supervisor access — safe from any thread)."""
        filled = False
        with self._slock:
            if not self.queue:
                return None
            bd = self.queue[0]
            b = self.boxes.get(bd)
            if b is None or b["state"] != "at_fill":
                return None
            self._placed += 1
            self._contents.setdefault(bd, []).append(part_def or "?")
            placed, target = self._placed, self._target
            if placed >= target:
                b["state"] = "filled"
                filled = True
                self._boxes_filled += 1
        if filled:
            self._log(f"box {bd} FILLED ({placed}/{target} parts: "
                      f"{','.join(self._contents.get(bd, []))}) — "
                      "awaiting empty trolley at the fill station")
        return (bd, placed, target)

    def status(self) -> Dict[str, Any]:
        with self._slock:
            return dict(self.pub)

    # ── sim-thread machinery ──────────────────────────────────────

    def _init(self) -> None:
        r = self.bridge.robot
        root = r.getRoot()
        kids = root.getField("children") if root is not None else None
        if kids is None:
            return
        n = kids.getCount()
        found = []
        fill_node = None
        for i in range(n):
            node = kids.getMFNode(i)
            if node is None:
                continue
            try:
                d = node.getDef() or ""
            except Exception:
                continue
            if d.startswith("BOX_"):
                found.append((d, node))
            elif d == "FILL_STOP":
                fill_node = node
        if not found or fill_node is None:
            return                      # not a line world; stay inactive
        fp = fill_node.getPosition()
        self.fill_x, self.belt_y = fp[0], fp[1]
        xs = []
        for d, node in found:
            tf = node.getField("translation")
            rf = node.getField("rotation")
            p = tf.getSFVec3f()
            self.boxes[d] = {"def": d, "node": node, "tf": tf, "rf": rf,
                             "x": p[0], "state": "belt"}
            xs.append(p[0])
        self.belt_z = self.boxes[found[0][0]]["node"].getPosition()[2]
        self.entry_x = min(xs) - 0.9
        self.queue = [d for d, _ in sorted(found,
                                           key=lambda it: -self.boxes[it[0]]["x"])]
        for td in self.trolley_defs:
            node = r.getFromDef(td)
            if node is not None:
                self._trolleys[td] = {"node": node,
                                      "tf": node.getField("translation"),
                                      "rf": node.getField("rotation")}
        if not self._trolleys:
            self._log("no trolley DEFs resolved; line stays inactive")
            return
        first = self._trolleys[self.trolley_defs[0]]["node"].getPosition()
        self.fill_spot = (first[0], first[1])
        for node, tf in self.bridge._iter_graspables():
            try:
                d = node.getDef() or ""
                rfld = node.getField("rotation")
                self._parts_spawn[d] = (list(tf.getSFVec3f()),
                                        list(rfld.getSFRotation())
                                        if rfld is not None else None)
            except Exception:
                continue
        self.active = True
        self._log(f"armed: {len(self.boxes)} box(es) on the belt, fill stop "
                  f"({self.fill_x:+.2f},{self.belt_y:+.2f}), fill spot "
                  f"({self.fill_spot[0]:+.2f},{self.fill_spot[1]:+.2f}), "
                  f"trolleys {list(self._trolleys)}, "
                  f"{len(self._parts_spawn)} feeder part(s)")

    def tick(self, sim_time_s: float, dt_s: float) -> None:
        if not self._init_done:
            if self.bridge.robot.getTime() < 1.5:
                return
            self._init_done = True
            try:
                self._init()
            except Exception as e:
                self._log(f"init failed (line disabled): {e!r}")
            return
        if not self.active:
            return
        self._tick_i += 1
        # Opt-in heartbeat (OMNILINK_LINE_HEARTBEAT=1): one stamped line every
        # HEARTBEAT_S of SIM time. Because every event carries both sim and
        # wall time, a run with this on yields a dense, phase-labelled
        # realtime-factor trace that costs the sim nothing but a file append.
        if self._hb_every > 0.0 and sim_time_s - self._hb_last >= self._hb_every:
            self._hb_last = sim_time_s
            try:
                busy = ",".join(sorted(
                    e["trolley"] for e in self._transit)) or "-"
                self._log(f"hb fill={self.pub.get('fill_state')} "
                          f"placed={self._placed}/{self._target} "
                          f"glide={'y' if self._glide else 'n'} "
                          f"transit={busy}")
            except Exception:
                pass
        try:
            self._tick_belt(dt_s)
            if self._glide is not None:
                self._tick_glide(dt_s)
            if self._tick_i % self.CHECK_PERIOD == 0:
                self._check_fill_trolley()
                self._check_transit(sim_time_s)
                self._publish()
        except Exception as e:
            if time.time() - self._err_logged > 10.0:
                self._err_logged = time.time()
                self._log(f"tick error (continuing): {e!r}")

    def _tick_belt(self, dt_s: float) -> None:
        for idx, bd in enumerate(list(self.queue)):
            b = self.boxes[bd]
            if b["state"] not in ("belt", "at_fill", "filled"):
                continue
            tx = self.fill_x - idx * self.QUEUE_GAP
            if b["state"] == "belt" and b["x"] < tx - 0.004:
                b["x"] = min(b["x"] + self.CONVEY_SPEED * dt_s, tx)
                # hover while advancing; the pin-on-drift branch below sets it
                # back down flat once the box stops.
                b["tf"].setSFVec3f([b["x"], self.belt_y,
                                    self.belt_z + self.MOVE_LIFT])
                b["rf"].setSFRotation([0.0, 0.0, 1.0, 0.0])
                try:
                    b["node"].setVelocity([0.0] * 6)
                except Exception:
                    pass
                continue
            if b["state"] == "belt" and idx == 0:
                with self._slock:
                    b["state"] = "at_fill"
                    self._placed = 0
                    self._target = self.FILL_TARGETS[
                        self._fill_ti % len(self.FILL_TARGETS)]
                    self._fill_ti += 1
                    self._contents[bd] = []
                self._log(f"box {bd} arrived at the fill station "
                          f"(target {self._target} parts)")
                self._publish()
            # pin-on-drift (staggered so it costs one read per tick at most)
            if self._tick_i % 4 == idx % 4:
                p = b["node"].getPosition()
                if (abs(p[0] - b["x"]) > self.PIN_DEADBAND
                        or abs(p[1] - self.belt_y) > self.PIN_DEADBAND
                        or abs(p[2] - self.belt_z) > 0.02):
                    b["tf"].setSFVec3f([b["x"], self.belt_y, self.belt_z])
                    b["rf"].setSFRotation([0.0, 0.0, 1.0, 0.0])
                    try:
                        b["node"].setVelocity([0.0] * 6)
                    except Exception:
                        pass

    def _check_fill_trolley(self) -> None:
        """When the front box is FILLED: find a present, stationary, EMPTY
        trolley at the fill spot and start the load glide."""
        if self._glide is not None or not self.queue:
            return
        bd = self.queue[0]
        b = self.boxes[bd]
        if b["state"] != "filled":
            return
        fx, fy = self.fill_spot
        present = None
        for td, t in self._trolleys.items():
            p = t["node"].getPosition()
            if math.hypot(p[0] - fx, p[1] - fy) <= self.TROLLEY_TOL:
                present = (td, p)
                break
        if present is None:
            if time.time() - self._wait_logged > 20.0:
                self._wait_logged = time.time()
                self._log(f"box {bd} filled — waiting for an empty trolley "
                          "at the fill station")
            self._troll_hist.clear()
            return
        td, p = present
        # empty? (no other box riding it)
        for od, ob in self.boxes.items():
            if od == bd:
                continue
            op = ob["node"].getPosition()
            if math.hypot(op[0] - p[0], op[1] - p[1]) < 0.5 and op[2] > 0.15:
                return
        # stationary over 3 consecutive polls
        hist = self._troll_hist.setdefault(td, [])
        if hist and math.hypot(p[0] - hist[-1][0], p[1] - hist[-1][1]) > 0.02:
            del hist[:]
        hist.append([p[0], p[1]])
        if len(hist) < 3:
            return
        self._start_glide(bd, td, p)

    def _start_glide(self, bd: str, td: str, tp) -> None:
        b = self.boxes[bd]
        bp = b["node"].getPosition()
        riders = []
        for node, tf in self.bridge._iter_graspables():
            try:
                p = tf.getSFVec3f()
            except Exception:
                continue
            if (math.hypot(p[0] - bp[0], p[1] - bp[1]) <= 0.30
                    and self.belt_z - 0.05 < p[2] < self.belt_z + 0.30):
                riders.append({"node": node, "tf": tf,
                               "local": (p[0] - bp[0], p[1] - bp[1],
                                         p[2] - bp[2])})
        # Route: WEST along the belt onto the outfeed spur head, SOUTH down
        # the spur (clear of the arm base), then EAST off its discharge end
        # onto the trolley deck. Right angles only, and the world's
        # DEF OUTFEED_SPUR is built to this centreline -- so the box rides a
        # conveyor for the whole trip instead of drifting over open floor.
        ox = self.fill_x - self.OUTFEED_DX
        pts = [(bp[0], bp[1]),
               (ox, self.belt_y),
               (ox, tp[1]),
               (tp[0], tp[1])]
        seg = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1)]
        total = max(1e-6, sum(seg))
        self._glide = {"box": bd, "trolley": td, "pts": pts, "seg": seg,
                       "total": total, "s": 0.0, "riders": riders,
                       "z0": bp[2], "z1": self.DECK_TOP, "rphase": 0}
        with self._slock:
            b["state"] = "loading"
        self._log(f"loading box {bd} onto {td} "
                  f"({len(riders)} part(s) physically aboard)")
        self._publish()

    def _tick_glide(self, dt_s: float) -> None:
        g = self._glide
        b = self.boxes[g["box"]]
        g["s"] = min(g["s"] + self.GLIDE_SPEED * dt_s, g["total"])
        s = g["s"]
        x, y = g["pts"][-1]
        for i, d in enumerate(g["seg"]):
            if s <= d or i == len(g["seg"]) - 1:
                f = clamp(s / d, 0.0, 1.0) if d > 1e-9 else 1.0
                x = g["pts"][i][0] + (g["pts"][i + 1][0] - g["pts"][i][0]) * f
                y = g["pts"][i][1] + (g["pts"][i + 1][1] - g["pts"][i][1]) * f
                break
            s -= d
        # Hold CONVEYOR height for the whole run and only settle over the last
        # DECK_SETTLE metres -- i.e. once the box is already above the trolley
        # deck. (The old profile started sinking at 60 % of the path, which put
        # the box mid-descent over open floor and read as floating.)
        zf = clamp(1.0 - (g["total"] - g["s"]) / self.DECK_SETTLE, 0.0, 1.0)
        z = g["z0"] + (g["z1"] - g["z0"]) * (zf * zf * (3.0 - 2.0 * zf))
        done = g["s"] >= g["total"] - 1e-9
        lift = 0.0 if done else self.MOVE_LIFT
        rlift = 0.0 if done else self.RIDER_LIFT
        z += lift
        b["tf"].setSFVec3f([x, y, z])
        b["rf"].setSFRotation([0.0, 0.0, 1.0, 0.0])
        try:
            b["node"].resetPhysics()
        except Exception:
            pass
        g["rphase"] = (g["rphase"] + 1) % 2
        if g["rphase"] == 0 or done:
            for rd in g["riders"]:
                lx, ly, lz = rd["local"]
                try:
                    rd["tf"].setSFVec3f([x + lx, y + ly, z + lz + rlift])
                    rd["node"].resetPhysics()
                except Exception:
                    pass
        if done:
            bd, td = g["box"], g["trolley"]
            self._glide = None
            with self._slock:
                b["state"] = "loaded"
                if self.queue and self.queue[0] == bd:
                    self.queue.pop(0)
            self._transit.append({"def": bd, "node": b["node"],
                                  "trolley": td,
                                  "parts": list(self._contents.get(bd, [])),
                                  "last": None, "stable_t": None,
                                  "delivered_logged": False,
                                  "departed_logged": False})
            self._log(f"box {bd} loaded onto {td} "
                      f"({len(g['riders'])} part(s) aboard) — ready for "
                      "dispatch")
            self._publish()

    def _check_transit(self, sim_time_s: float) -> None:
        fx, fy = self.fill_spot
        for entry in list(self._transit):
            try:
                p = entry["node"].getPosition()
            except Exception:
                continue
            d_fill = math.hypot(p[0] - fx, p[1] - fy)
            if not entry["departed_logged"] and d_fill > self.DEPART_DIST:
                entry["departed_logged"] = True
                self._log(f"box {entry['def']} departed the fill station "
                          f"on {entry['trolley']}")
                self._publish()
            if d_fill < self.SHIP_MIN_DIST:
                entry["last"] = None
                entry["stable_t"] = None
                continue
            last = entry["last"]
            entry["last"] = [p[0], p[1]]
            if last is None or math.hypot(p[0] - last[0], p[1] - last[1]) > 0.05:
                entry["stable_t"] = None
                continue
            if entry["stable_t"] is None:
                entry["stable_t"] = sim_time_s
                if not entry["delivered_logged"]:
                    entry["delivered_logged"] = True
                    self._log(f"box {entry['def']} delivered at the dispatch "
                              f"zone ({len(entry['parts'])} part(s): "
                              f"{','.join(entry['parts'])})")
                continue
            if sim_time_s - entry["stable_t"] >= self.SHIP_AFTER_S:
                self._ship(entry)

    def _ship(self, entry: dict) -> None:
        bd = entry["def"]
        b = self.boxes[bd]
        n = 0
        for pd in entry["parts"]:
            spawn = self._parts_spawn.get("GRASP_" + pd
                                          if not pd.startswith("GRASP_")
                                          else pd) or self._parts_spawn.get(pd)
            node = self.bridge.robot.getFromDef(pd)
            if spawn is None or node is None:
                continue
            try:
                node.getField("translation").setSFVec3f(list(spawn[0]))
                if spawn[1] is not None:
                    node.getField("rotation").setSFRotation(list(spawn[1]))
                node.resetPhysics()
                n += 1
            except Exception:
                pass
        b["x"] = self.entry_x
        b["tf"].setSFVec3f([self.entry_x, self.belt_y, self.belt_z])
        b["rf"].setSFRotation([0.0, 0.0, 1.0, 0.0])
        try:
            b["node"].resetPhysics()
        except Exception:
            pass
        with self._slock:
            b["state"] = "belt"
            self.queue.append(bd)
            self._contents.pop(bd, None)
            self._shipped += 1
        self._transit.remove(entry)
        self._log(f"box {bd} SHIPPED from dispatch — box recycled to the "
                  f"belt entry, {n} part(s) respawned at the feeder")
        self._publish()

    def _publish(self) -> None:
        loaded = None
        for entry in self._transit:
            if not entry["departed_logged"]:
                loaded = {"trolley": entry["trolley"], "box": entry["def"],
                          "parts": len(entry["parts"])}
                break
        with self._slock:
            front = self.queue[0] if self.queue else None
            b = self.boxes.get(front) if front else None
            self.pub = {
                "active": True,
                "fill_box": front,
                "fill_state": b["state"] if b else None,
                "placed": self._placed,
                "target": self._target,
                "loaded": loaded,
                "queued": max(0, len(self.queue) - 1),
                "in_transit": [{"box": e["def"], "trolley": e["trolley"],
                                "delivered": e["delivered_logged"]}
                               for e in self._transit],
                # COUNTS, not adjectives. Every one of these was already
                # being tracked or logged; they were simply not published,
                # so "how many boxes have you shipped?" had to be hedged.
                "remaining_in_box": max(0, self._target - self._placed),
                "boxes_filled_total": self._boxes_filled,
                "shipped_total": self._shipped,
                "boxes_on_line": len(self.boxes),
                "loads_out": len(self._transit),
            }


# ── Idle demo loop (opt-in, --idle-loop pick) ────────────────────────

class ArmIdleLoop(threading.Thread):
    """Ambient pick loop that keeps the demo ALIVE at idle. Opt-in only.

    Cycles the phased anypick-style pick — hover, descend, vacuum attach,
    lift, carry, drop into the basket — over every GRASP_* part it finds at
    startup (~13 s per pick, all motion through the bridge's normal
    ``sequence`` machinery on the sim thread). When the belt is empty and the
    drop trolley is parked + stationary back at the pick cell, the parts are
    supervisor-respawned at their pad poses ("new parts arriving") after a
    ~5 s beat.

    Interruption contract: ANY operator command (chat prompt, HTTP tool
    call — everything that goes through bridge.note_external_command)
    pauses the loop INSTANTLY (the command's own motion plan simply replaces
    ours; the loop notices and backs off) and the loop stays quiet until
    ``resume_s`` seconds after the last command.

    Crash safety: every cycle step is fail-soft — errors are logged and the
    loop moves on. It never takes the bridge or the HTTP surface down.

    Waypoints are solved with the bridge's own DLS pose IK in the ARM BASE
    frame (world targets are transformed live, so a non-origin arm works —
    the one-shot /pick endpoint's origin-arm assumption does not apply
    here). Orientation is a soft vertical-tool preference (w_rot 0.15) plus
    a weld-point correction loop, which lands the grasp point within a few
    mm at this world's reach while keeping the cup near-vertical on contact.
    """

    PICK_HOVER = 0.14      # m above the part for the approach hover
    PICK_AT = 0.010        # weld point above part centre at attach
    PICK_LIFT = 0.24       # lift height after attach
    DROP_ABOVE = 0.70      # weld point above the trolley origin at release
    CARRY = (0.30, -0.10, 0.58)   # base-frame mid waypoint belt -> basket
    PAD_TOL = 0.15         # a part within this xy of its pad is pickable
    TROLLEY_TOL = 0.45     # trolley within this of its home = present
    W_ROT = 0.15           # soft vertical-tool weight for the pose IK
    _R_DOWN = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]

    # PLACING INTO A LINE BOX. Two heights, both measured from the box's own
    # origin (the box floor slab's underside), so they travel with the box.
    #
    # DROP_ABOVE_BOX was 0.22, and at that height the part is RELEASED ENTIRELY
    # ABOVE THE BOX and dropped in. Measured over 3 dispatch cycles: the part
    # sits at rest at box+0.047 (interior floor 0.422 over an origin at 0.400,
    # plus half of a 0.05 m cube), so 0.22 is a 0.173 m free fall arriving at
    # 1.84 m/s -- audibly and visibly a drop, not a place. It also pushed the
    # target out to 0.978 m from the base on the first cube of every box
    # (BOX_SCATTER's first slot offsets AWAY from the arm), i.e. 102.9% of this
    # arm's declared workspace_max_radius of 0.95, where the elbow locks
    # straight and the cup cants over instead of staying vertical.
    #
    # 0.13 puts the part 0.083 m over its resting place (0.083 m fall, 1.28 m/s)
    # and pulls the first cube's target in to 0.923 m -- 97.2% of reach, and the
    # solve's cant falls with it (48.2 deg -> 35.8 deg at the release point).
    #
    # BUT 0.13 CANNOT BE REACHED IN ONE SEGMENT. The box walls rise to box+0.142
    # and a part released at box+0.13 has its underside at box+0.105, i.e. INSIDE
    # the box mouth, so the last segment has to thread it through the rim. The
    # carry waypoint and the drop are at nearly the same height, so a single
    # joint-space segment between them sweeps the part SIDEWAYS into the rim:
    # traced through this file's own IK, the part's corner passes 24 mm inside
    # the east wall's top edge (worst point 86% along the segment). The largest
    # single-segment height that clears is 0.17, and it clears by 1 mm -- which
    # is not a clearance, it is a coincidence. So the approach is split instead:
    # cross to DROP_APPROACH_ABOVE_BOX, still fully clear of the rim, then
    # descend straight down. Retraced with the split, all three box slots:
    # 0 mm rim overlap, and the descent is 0.149 rad of joint travel over 0.8 s
    # (0.28 rad/s peak), nothing like a rate limit.
    #
    # 0.19 for the approach, not higher: it puts the part's underside 23 mm --
    # about half a part -- over the rim, while keeping the most-extended point
    # of the whole cycle at 0.959 m on the first slot. That is still the arm's
    # furthest reach of the cycle, but it is now a waypoint the arm passes
    # THROUGH rather than the pose it holds for the 0.55 s of wait-release-wait,
    # and it is nearer in than the 0.978 m the old release sat at. (0.24 was
    # tried first and is worse on exactly this count: 0.990 m, 104.3%.)
    DROP_ABOVE_BOX = 0.13           # weld point above the box origin AT RELEASE
    DROP_APPROACH_ABOVE_BOX = 0.19  # ... and on the way in, clear of the rim
    BOX_SCATTER = 0.075    # xy spread of successive drops inside the box

    def __init__(self, bridge: ArmBridge, drop_def: Optional[str] = None,
                 resume_s: float = 60.0) -> None:
        super().__init__(name="omnilink-idle-pick", daemon=True)
        self.bridge = bridge
        # --idle-drop may name several trolleys (the line alternates between
        # them); the loop's own fallback drop target is the first.
        self.drop_def = ((drop_def or "").split(",")[0].strip() or None)
        self.resume_s = max(5.0, float(resume_s))
        ik = dict(bridge.cfg["ik"] or {})
        ik["pose_max_iters"] = 80          # idle solves are cached; keep cheap
        self._ik = ik
        self._goz = float(ik["tcp_offset"][2]) + float(bridge.tool_reach)
        self._ik_cache: Dict[Tuple[float, float, float], List[float]] = {}
        self._base_cache = None
        self.pads: List[dict] = []
        self.drop_node = None
        self.drop_home: Optional[List[float]] = None
        self._troll_prev: Optional[Tuple[float, List[float]]] = None
        self._fails: Dict[str, int] = {}
        self._paused_logged = False
        self.picks = 0
        # Coarse phase, published in /state and used by `at_leg:<leg>`
        # triggers. Kept in sync by the cycle body below.
        self.leg = "idle"
        self._intent_obs_t = 0.0
        if bridge.intents is not None:
            # Intent events belong next to the work they gate.
            bridge.intents._log_fn = self._log

    # ── logging / gating helpers ──────────────────────────────────

    def _log(self, msg: str) -> None:
        try:
            t = self.bridge.robot.getTime()
        except Exception:
            t = -1.0
        line = f"[idle-pick] t={t:.1f}s w={time.time() - _T0:.1f}s {msg}"
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
        # THREE different pauses, and they are NOT the same: the quiet window
        # after any operator command (auto-resumes, armed when the PROMPT
        # lands); a STOP hold, armed by act_stop when the stop actually
        # EXECUTES and lasting ArmBridge.STOP_HOLD_S -- needed because an LLM
        # turn can outlive the 12 s quiet window this world configures, so a
        # "stop" used to land on a robot with nothing holding it; and a HOLD
        # from hold_until_told / a fired "until_told" intent, which does NOT
        # auto-resume. Without the hold arm, "stop until I tell you to
        # continue" was silently overridden by the quiet window a minute
        # later.
        it = self.bridge.intents
        if it is not None and it.hold_active():
            return True
        if time.time() < getattr(self.bridge, "stop_hold_until", 0.0):
            return True
        return (time.time() - self.bridge.last_external_cmd) < self.resume_s

    def _pause_gate(self) -> bool:
        """True while paused by a recent operator command (logs edges)."""
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
            self._log("resuming idle pick loop")
        return False

    # ── deferred intents ──────────────────────────────────────────

    def _intents_observe(self) -> None:
        """Feed the store the two things it can only learn by watching: the
        current leg, and the passage of time (expiry). Throttled."""
        it = self.bridge.intents
        if it is None:
            return
        now = time.time()
        if now - getattr(self, "_intent_obs_t", 0.0) < 0.5:
            return
        self._intent_obs_t = now
        try:
            it.note_leg(self.leg)
            # AUTHORITATIVE COUNTER PUSH, every poll -- `picks` is the same
            # monotonic number /state publishes. See IntentStore.sync_tasks:
            # depending on one hand-placed boundary call instead lost the
            # event whenever that line was not reached.
            # every completed job on this arm IS a pick, so one counter
            # serves both after_current_task and after_n_picks
            it.sync_tasks(self.picks, "", self._safe_to_stop(),
                          deliveries=self.picks)
            it.tick()
        except Exception as e:
            self._log(f"intent observe failed: {e!r}")

    def _safe_to_stop(self) -> bool:
        """May a pause take effect at this instant? An arm holding a part
        would drop it (or freeze mid-carry), so a trigger met while the
        gripper is full is DEFERRED to the moment it empties, not lost.

        `pick` is excluded for the same reason the tug excludes its approach
        legs: the gripper is still empty on the way down to the part, and a
        pause landing there stops the arm holding something it has just
        grabbed."""
        return (self.bridge.held_node is None
                and self.leg not in ("pick",))

    def _task_boundary(self, detail: str) -> None:
        """An explicit, high-quality boundary: part in the box, gripper
        empty, arm home. Now only a NUDGE -- it labels the event and pushes
        the counter one poll early; correctness no longer depends on
        reaching it."""
        it = self.bridge.intents
        if it is None:
            return
        try:
            it.sync_tasks(self.picks, detail, self._safe_to_stop(),
                          deliveries=self.picks)
        except Exception as e:
            self._log(f"intent boundary failed: {e!r}")

    def _forbidden(self, rule: str, detail: str) -> bool:
        """True when a standing constraint forbids what is about to happen.
        Checked BEFORE acting."""
        it = self.bridge.intents
        if it is None:
            return False
        if it.constraint(rule) is None:
            return False
        it.note_block(rule, detail)
        return True

    # ── geometry helpers ──────────────────────────────────────────
    # ALL supervisor pipe access below goes through self._call — a
    # threaded supervisor call stalls the step exchange (see
    # MainThreadCalls). Pure math stays on this thread.

    def _call(self, fn):
        return self.bridge.mt.call(fn)

    def _base_pose(self):
        """One-time cached base pose (the idle loop only runs on
        static-base arms)."""
        if self._base_cache is None:
            node = self.bridge._self_node
            got = self._call(lambda: (list(node.getPosition()),
                                      list(node.getOrientation())))
            if got is None:
                return None
            self._base_cache = got
        return self._base_cache

    def _to_base(self, p) -> Optional[List[float]]:
        """World point -> arm base frame."""
        pose = self._base_pose()
        if pose is None or p is None:
            return None
        bp, o = pose
        dx, dy, dz = p[0] - bp[0], p[1] - bp[1], p[2] - bp[2]
        return [o[0] * dx + o[3] * dy + o[6] * dz,
                o[1] * dx + o[4] * dy + o[7] * dz,
                o[2] * dx + o[5] * dy + o[8] * dz]

    def _solve(self, tgt, seed) -> Optional[List[float]]:
        """Soft-vertical pose IK with weld-point correction. Cached (the pads
        and the trolley home are fixed spots, so steady-state cycles cost no
        solver time). Returns None when the target can't be hit within 5 cm."""
        key = (round(tgt[0], 2), round(tgt[1], 2), round(tgt[2], 2))
        hit = self._ik_cache.get(key)
        if hit is not None:
            return list(hit)
        b = self.bridge
        off = (0.0, 0.0, self._goz)
        t = list(tgt)
        q = list(seed)
        w = None
        for _ in range(3):
            q, _pe, _re, _it = dls_ik_pose(self._ik["chain"], q, t,
                                           self._R_DOWN, off, self._ik,
                                           b.joint_limits, w_rot=self.W_ROT)
            w, _R = forward_kinematics_pose(self._ik["chain"], q, off)
            t = [t[i] - (w[i] - tgt[i]) for i in range(3)]
        w, _R = forward_kinematics_pose(self._ik["chain"], q, off)
        err = math.dist(w, tgt)
        if err > 0.05:
            self._log(f"IK could not reach {tuple(round(v, 3) for v in tgt)} "
                      f"(err {err * 1000:.0f} mm)")
            return None
        self._ik_cache[key] = list(q)
        return q

    # ── scene state ───────────────────────────────────────────────

    def _snapshot_scene(self) -> None:
        def impl():
            b = self.bridge
            pads = []
            for node, tf in b._iter_graspables():
                try:
                    d = node.getDef() or ""
                    rf = node.getField("rotation")
                    pos = list(tf.getSFVec3f())
                    rot = list(rf.getSFRotation()) if rf is not None else None
                except Exception:
                    continue
                pads.append({"def": d, "node": node, "tfield": tf,
                             "rfield": rf, "pos": pos, "rot": rot})
            drop_node = drop_home = None
            if self.drop_def:
                drop_node = b.robot.getFromDef(self.drop_def)
                if drop_node is not None:
                    p = drop_node.getPosition()
                    drop_home = [p[0], p[1], p[2]]
            return pads, drop_node, drop_home

        got = self._call(impl)
        if got is None:
            return
        self.pads, self.drop_node, self.drop_home = got
        if self.drop_def and self.drop_node is None:
            self._log(f"drop DEF {self.drop_def!r} not found; using the "
                      "cfg drop_zone instead")

    def _next_pickable(self) -> Optional[dict]:
        cands = [pad for pad in self.pads
                 if self._fails.get(pad["def"], 0) < 3]

        def impl():
            for pad in cands:
                try:
                    p = pad["tfield"].getSFVec3f()
                except Exception:
                    continue
                if (math.hypot(p[0] - pad["pos"][0], p[1] - pad["pos"][1])
                        < self.PAD_TOL and abs(p[2] - pad["pos"][2]) < 0.10):
                    return pad
            return None

        return self._call(impl)

    def feeder_status(self) -> dict:
        """HOW MANY PARTS ARE LEFT ON THE FEEDER -- as a number.

        MEASURED FAILURE THIS FIXES: asked "how many parts are left in the
        feeder?" the model answered "I don't have a direct sensor reading"
        and offered reassurance instead. It does have one: this loop walks
        every feeder pad on the very next cycle to choose what to pick, and
        `_next_pickable` already decides pad-by-pad whether a part is sitting
        on it. Counting that walk is the whole fix -- no new sensor, no new
        poll, the same test the loop itself trusts.

        `at_pad` = parts on the tray right now (pickable). `away` = parts
        that have left the tray (in the fill box, or riding out on a cart);
        they come back when their load ships. `stuck` = pads this loop has
        failed on 3 times and stopped trying."""
        pads = list(self.pads)
        stuck = [p["def"] for p in pads if self._fails.get(p["def"], 0) >= 3]

        def impl():
            at = []
            for pad in pads:
                try:
                    p = pad["tfield"].getSFVec3f()
                except Exception:
                    continue
                if (math.hypot(p[0] - pad["pos"][0], p[1] - pad["pos"][1])
                        < self.PAD_TOL and abs(p[2] - pad["pos"][2]) < 0.10):
                    at.append(pad["def"])
            return at

        at_pad = self._call(impl)
        if at_pad is None:
            # The supervisor call could not be scheduled (sim thread busy).
            # UNKNOWN IS AN ANSWER; a stale number pretending to be live is
            # not.
            return {"known": False, "pads_total": len(pads),
                    "say": ("I count the feeder by looking at the tray and I "
                            "couldn't get a look just now — ask me again in a "
                            "moment rather than take a stale number from me.")}
        away = [p["def"] for p in pads if p["def"] not in at_pad]
        return {
            "known": True,
            "pads_total": len(pads),
            "at_pad": len(at_pad),
            "away": len(away),
            "stuck": len(stuck),
            "at_pad_defs": at_pad,
            "away_defs": away,
            "picks_completed": self.picks,
            "say": (f"There are {len(at_pad)} part(s) on the feeder tray "
                    f"right now out of {len(pads)} that belong to it"
                    + (f"; the other {len(away)} are in a box or out on a "
                       "cart and come back when that load ships"
                       if away else "")
                    + (f". {len(stuck)} pad(s) I've given up on."
                       if stuck else ".")),
        }

    def _line(self):
        """The warehouse line master, or None when this world has no line
        (then the loop keeps its original trolley-basket behaviour)."""
        ln = getattr(self.bridge, "line", None)
        return ln if (ln is not None and ln.active) else None

    def _fill_box(self):
        """(box_def, node, placed, target) while an unfilled box waits at the
        fill station; None otherwise. Line worlds only."""
        ln = self._line()
        return ln.fill_target_info() if ln is not None else None

    def _trolley_ready(self) -> bool:
        """Drop target usable. On a LINE, that means a box is stopped at the
        fill station and still short of its part target — the arm simply
        waits (holding, if it already picked) through the swap while the
        MAVs exchange trolleys. Otherwise: the drop trolley is present at
        its home spot AND stationary (a moving trolley is being towed —
        hands off)."""
        if self._line() is not None:
            return self._fill_box() is not None
        if self.drop_node is None:
            return True                      # static cfg drop zone
        p = self._call(self.drop_node.getPosition)
        if p is None:
            return False
        if math.hypot(p[0] - self.drop_home[0], p[1] - self.drop_home[1]) \
                > self.TROLLEY_TOL:
            self._troll_prev = None
            return False
        now = time.time()
        prev = self._troll_prev
        if prev is None or now - prev[0] > 2.5:
            self._troll_prev = (now, list(p))
            return False                     # need a second sample
        if now - prev[0] < 0.8:
            return False                     # too soon to call it stationary
        moved = math.hypot(p[0] - prev[1][0], p[1] - prev[1][1])
        self._troll_prev = (now, list(p))
        return moved < 0.03

    def _drop_point_base(self, above: Optional[float] = None) -> Optional[List[float]]:
        # LINE: drop INTO the box waiting at the fill station. Successive
        # parts are scattered around the box centre so three of them do not
        # stack on one column and bounce back out over the rim.
        # `above` overrides DROP_ABOVE_BOX to ask for the same xy at a
        # different height -- how the caller gets the above-the-rim approach
        # point that makes the release height safe (see DROP_ABOVE_BOX).
        fb = self._fill_box()
        if fb is not None:
            _bd, node, placed, _target = fb
            p = self._call(node.getPosition)
            if p is None:
                return None
            ox = self.BOX_SCATTER * (1 if placed % 2 else -1)
            oy = self.BOX_SCATTER * (1 if placed % 4 < 2 else -1)
            dz = self.DROP_ABOVE_BOX if above is None else float(above)
            return self._to_base([p[0] + ox, p[1] + oy, p[2] + dz])
        if above is not None:
            return None            # only a line box has a rim to clear
        if self.drop_node is not None:
            p = self._call(self.drop_node.getPosition)
            if p is None:
                return None
            return self._to_base([p[0], p[1], p[2] + self.DROP_ABOVE])
        dz = self.bridge.cfg.get("drop_zone") or [0.30, 0.34, 0.0]
        return [dz[0], dz[1], dz[2] + 0.25]

    # ── motion execution ──────────────────────────────────────────

    def _run_steps(self, steps: List[dict], timeout: float) -> bool:
        """Submit a sequence plan and wait for it to finish. False when an
        operator command interrupted (their plan replaced ours — we back off
        instantly) or the wait timed out. ``timeout`` is a SIM-time budget
        (motion advances in sim seconds; a wall budget silently truncates
        whenever the sim runs below realtime), with a generous wall cap as
        a dead-sim escape hatch."""
        b = self.bridge
        t0_sim = b.robot.getTime()
        with b.lock:
            # Checked under the motion lock: note_external_command always
            # precedes the command's own motion plan, so either we see the
            # command here and back off, or our plan lands first and the
            # command's plan (taken after this lock) replaces it. Either
            # way the operator wins — no stomping race.
            if self._blocked():
                return False
            # SOURCE-STAMPED (PROTOCOL.md 5.4.1 rule 5): the busy guard on the
            # act_* verbs REFUSES a second operator command, but an ambient
            # loop motion must still YIELD to one -- otherwise every command
            # sent while the pick loop is mid-cycle would answer 409 and the
            # warehouse demo would become uncommandable. The loop keeps its
            # existing bail-out (it re-checks _blocked and backs off), so this
            # is the same "the operator wins" rule, now stated in the plan
            # itself instead of implied by who wrote it last.
            b.motion = ("sequence", {"steps": steps, "i": 0,
                                     "from_q": b._read_q(),
                                     "start_s": t0_sim,
                                     "verb": "idle pick cycle",
                                     "source": b.SOURCE_IDLE_LOOP})
        t0_wall = time.time()
        wall_cap = timeout * 3.0 + 30.0
        while time.time() - t0_wall < wall_cap:
            if self._blocked():
                return False
            with b.lock:
                kind = b.motion[0]
            if kind == "hold":
                return True
            if b.robot.getTime() - t0_sim > timeout:
                break
            time.sleep(0.08)
        self._log(f"sequence timed out after {timeout:.0f} sim-s (continuing)")
        return False

    # ── cycle phases ──────────────────────────────────────────────

    def _pick(self, pad: dict) -> None:
        b = self.bridge
        if not self._trolley_ready():
            time.sleep(1.0)
            return
        pw = self._call(pad["tfield"].getSFVec3f)
        p = self._to_base(pw)
        if p is None:
            return
        q_h = self._solve((p[0], p[1], p[2] + self.PICK_HOVER), b.home_pose)
        q_a = self._solve((p[0], p[1], p[2] + self.PICK_AT), q_h or b.home_pose)
        q_l = self._solve((p[0], p[1], p[2] + self.PICK_LIFT), q_a or b.home_pose)
        if q_h is None or q_a is None or q_l is None:
            self._fails[pad["def"]] = self._fails.get(pad["def"], 0) + 1
            return
        self._log(f"picking {pad['def']}")
        ok = self._run_steps([
            {"t": "move", "to_q": q_h, "dur": 1.8},
            {"t": "move", "to_q": q_a, "dur": 1.1},
            {"t": "wait", "dur": 0.25},
            {"t": "grasp"},
            {"t": "wait", "dur": 0.3},
            {"t": "move", "to_q": q_l, "dur": 1.0},
        ], timeout=12.0)
        if not ok:
            return
        if b.held_node is None:
            n = self._fails[pad["def"]] = self._fails.get(pad["def"], 0) + 1
            self._log(f"no grip on {pad['def']} (attempt {n}); retreating")
            self._run_steps([{"t": "move", "to_q": list(b.home_pose),
                              "dur": 1.5}], timeout=10.0)
            return
        self._place_held()

    def _place_held(self) -> None:
        b = self.bridge
        waited = False
        on_line = self._line() is not None
        while not self._trolley_ready():
            if self._pause_gate():
                return                      # keep holding; chat owns the arm
            if not waited:
                self._log("holding part — waiting for the next box at the "
                          "fill station" if on_line
                          else "holding part — waiting for the trolley to "
                               "return")
                waited = True
            time.sleep(1.0)
        if waited:
            self._log("box at the fill station — resuming the place"
                      if on_line else "trolley is back — resuming the place")
        d = self._drop_point_base()
        d_ap = self._drop_point_base(self.DROP_APPROACH_ABOVE_BOX)
        q_c = self._solve(self.CARRY, b.home_pose)
        # Solve the above-the-rim approach FIRST and seed the release from it,
        # so the last leg into the box is a short vertical descent rather than
        # a fresh solve that could land on the far side of a branch.
        q_ap = self._solve(tuple(d_ap), q_c or b.home_pose) if d_ap else None
        q_d = self._solve(tuple(d), q_ap or q_c or b.home_pose) if d else None
        if d_ap is not None and q_ap is None:
            self._log("box approach point unreachable; going in on the direct "
                      "segment (may graze the box rim)")
        try:
            held = b.held_node.getDef() if b.held_node is not None else "?"
        except Exception:
            held = "?"
        if q_c is None or q_d is None:
            self._log("place target unreachable; releasing in place")
            self._call(b.act_release)
            self._run_steps([{"t": "move", "to_q": list(b.home_pose),
                              "dur": 1.5}], timeout=10.0)
            return
        # PLACE, not drop: cross to the box ABOVE its rim, descend straight
        # down, let go, lift straight back out. The two extra 0.8 s legs are
        # the whole reason DROP_ABOVE_BOX can be a release height inside the
        # box mouth instead of a height to drop from (see DROP_ABOVE_BOX).
        into = ([{"t": "move", "to_q": q_ap, "dur": 1.6},
                 {"t": "move", "to_q": q_d, "dur": 0.8}]
                if q_ap is not None else
                [{"t": "move", "to_q": q_d, "dur": 1.6}])
        out_of = ([{"t": "move", "to_q": q_ap, "dur": 0.8}]
                  if q_ap is not None else [])
        # The return leg out of the box was 1.0 s and it SATURATED: solved over
        # all 6 feeder pads x 3 box slots, its largest joint delta is 3.14 rad
        # (joints 1 and 6 counter-swinging as the arm comes back off the box),
        # which at 1.0 s is a 4.71 rad/s peak against the 3.1416 rad/s motor
        # limit -- the same lag-then-lurch as the joint4 wind-up, on the leg
        # right after the release. 1.6 s brings the peak to 2.95 rad/s.
        ok = self._run_steps([
            {"t": "move", "to_q": q_c, "dur": 1.8},
            *into,
            {"t": "wait", "dur": 0.25},
            {"t": "release"},
            {"t": "wait", "dur": 0.3},
            *out_of,
            {"t": "move", "to_q": q_c, "dur": 1.6},
            {"t": "move", "to_q": list(b.home_pose), "dur": 1.4},
        ], timeout=18.0)
        if ok and b.held_node is None:
            self.picks += 1
            self.leg = "idle"
            ln = self._line()
            noted = ln.note_placed(held) if ln is not None else None
            if noted is not None:
                bd, placed, target = noted
                self._log(f"pick #{self.picks} complete: {held} -> box {bd} "
                          f"({placed}/{target} parts)")
            else:
                self._log(f"pick #{self.picks} complete: {held} -> basket")
            # CLEAN TASK BOUNDARY: part placed, gripper empty, arm home.
            self._task_boundary(f"pick #{self.picks}")

    def _maybe_respawn(self) -> None:
        # On a LINE the master owns part respawn (parts come back to the
        # feeder only when their box SHIPS at the dispatch bay). Respawning
        # here would teleport parts straight out of a box in transit, so the
        # loop just waits for the feeder to refill.
        if self._line() is not None:
            time.sleep(1.5)
            return
        if not self._trolley_ready():
            time.sleep(1.0)
            return
        if not self.pads:
            time.sleep(2.0)
            return
        self._log("belt empty — new parts arriving in 5 s")
        t0 = time.time()
        while time.time() - t0 < 5.0:
            if self._pause_gate():
                return
            time.sleep(0.25)
        def impl():
            n = 0
            for pad in self.pads:
                try:
                    pad["tfield"].setSFVec3f(list(pad["pos"]))
                    if pad["rfield"] is not None and pad["rot"] is not None:
                        pad["rfield"].setSFRotation(list(pad["rot"]))
                    pad["node"].resetPhysics()
                    n += 1
                except Exception:
                    pass
            return n

        n = self._call(impl) or 0
        self._fails.clear()
        self._log(f"respawned {n} part(s) at the belt pads")

    # ── thread body ───────────────────────────────────────────────

    def run(self) -> None:
        try:
            while self.bridge.robot.getTime() < 4.0:
                time.sleep(0.25)
            self._snapshot_scene()
        except Exception as e:
            print(f"[idle-pick] disabled (startup failed: {e!r})")
            return
        if not self.pads:
            self._log("no GRASP_* parts in this world; idle loop idle")
        if self._line() is not None:
            drop = "line box at the fill station"
        elif self.drop_node is not None:
            drop = f"DEF {self.drop_def}"
        else:
            drop = "cfg drop_zone"
        self._log(f"armed: {len(self.pads)} part(s), drop={drop}, "
                  f"resume after {self.resume_s:.0f}s quiet")
        while True:
            try:
                if self._pause_gate():
                    time.sleep(0.4)
                    continue
                if self.bridge.held_node is not None:
                    self.leg = "place"
                    self._place_held()
                    continue
                pad = self._next_pickable()
                if pad is not None:
                    # STANDING RULES ARE CHECKED BEFORE ACTING. A "no more
                    # picks for now" rule must stop the NEXT pick starting,
                    # not halt the arm mid-carry.
                    if self._forbidden("no_new_picks",
                                       f"declining to start a pick on "
                                       f"{pad.get('def', '?')}"):
                        self.leg = "idle"
                        time.sleep(1.5)
                        continue
                    self.leg = "pick"
                    self._pick(pad)
                else:
                    if self._forbidden("no_respawn",
                                       "declining to respawn fresh parts"):
                        self.leg = "idle"
                        time.sleep(1.5)
                        continue
                    self.leg = "respawn"
                    self._maybe_respawn()
                    self.leg = "idle"
            except Exception as e:
                self._log(f"cycle error (continuing): {e!r}")
                time.sleep(2.0)


# ── HTTP server ──────────────────────────────────────────────────────

def _json_finite(obj: Any) -> Any:
    """Recursively replace non-finite floats (NaN/Inf) with None.

    Position sensors read NaN until the first robot.step() completes (the
    HTTP server is up BEFORE the main loop starts), and json.dumps happily
    emits bare `NaN` -- which is NOT valid JSON, so clients (jq, JSON.parse,
    json.loads) reject the whole body and the endpoint LOOKS empty/broken."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _json_finite(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_finite(v) for v in obj]
    return obj


def _intent_route(bridge: "ArmBridge", body: dict) -> dict:
    """POST /intents -- the relay-free door onto the deferred-intent store.

    The same dispatch the LLM tools use, so a headless gate can prove the
    MECHANISM (does the pause really land at the boundary? does the rule
    really gate the next pick?) without spending model tokens."""
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


def _wait_flag(body: dict) -> bool:
    """`wait` on an HTTP motion verb, DEFAULT TRUE.

    PROTOCOL.md 5.4.1: the honest default is the one that returns a
    measurement. An arm motion here is 0.8-8 s -- short enough that blocking
    is a better answer than a promise, and well inside the 50 s wait ceiling.
    (The mobile bridge defaults its HTTP `wait` to FALSE for the opposite
    reason: a skid-steer 180-degree pivot is ~40 s and sits right on the
    edge connector's per-tool timeout.) Pass {"wait": false} for the old
    return-on-dispatch behaviour, then poll get_robot_state.last_command.

    Deliberately NOT applied to the act_* methods themselves, whose default
    stays False: the ambient pick loop, the offline intent router and the
    headless tests all call them from a thread that may itself be the one
    driving tick(), and a blocking wait there would be waiting on itself."""
    v = body.get("wait", True)
    if isinstance(v, str):
        return v.strip().lower() not in ("0", "false", "no", "off")
    return bool(v)


def make_handler(bridge: ArmBridge, router: IntentRouter, relay: Any = None):
    action_lock = threading.RLock()
    request_ids = RequestIdGuard()
    trusted_origins = allowed_origins()
    bridge_token = configured_token()

    class _H(BaseHTTPRequestHandler):
        # POST routes that must NOT queue behind a blocking motion verb:
        # /stop_robot (the abort), /servo_joint_positions (the streaming
        # lane -- a setpoint queued behind the goal it exists to preempt
        # would defeat both its latency and its preemption; the bridge
        # lock inside act_servo_joint_positions is its serialisation),
        # plus every pure read (the poll a wait=false caller is told to
        # make). See do_POST.
        LOCK_FREE_POST_PATHS = frozenset({
            "/stop_robot", "/servo_joint_positions",
            "/state", "/get_robot_state", "/list_robots", "/capabilities",
            "/read_joints", "/read_tcp_pose", "/solve_ik", "/node_pose",
            "/dump_tree", "/reach_envelope", "/line_counts",
        })

        def log_message(self, fmt, *args):
            return  # quiet

        def _motion_json(self, result):
            """Answer a motion verb with the status its own result implies.

            The busy check lives in ArmBridge._begin_motion so that every
            entry point is covered (HTTP route, tool dispatch, offline intent
            router), which means the refusal comes back as a dict rather than
            being raised by the route. Preserve the 409 the capabilities
            surface promises."""
            code = 200
            if isinstance(result, dict):
                code = int(result.get("http_status", 200))
            return self._json(code, result)

        def _json(self, code: int, obj: Any) -> None:
            # allow_nan=False guarantees strictly valid JSON on the wire;
            # the sanitizer maps any NaN/Inf field to null instead.
            try:
                data = json.dumps(obj, default=str, allow_nan=False).encode("utf-8")
            except ValueError:
                data = json.dumps(_json_finite(obj), default=str,
                                  allow_nan=False).encode("utf-8")
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

        def _read_json(self) -> dict:
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
                if path not in ("/state", "/get_robot_state", "/list_robots", "/capabilities"):
                    request_ids.claim(path, request_id)
                # `action_lock` serialises ACTIONS. /stop_robot has always
                # bypassed it -- a stop that queues behind the motion it is
                # meant to abort is useless -- and now that the motion verbs
                # BLOCK by default (PROTOCOL.md 5.4.1), the pure reads have
                # to bypass it for the same reason: a caller that passed
                # wait=false is told to poll get_robot_state.last_command,
                # and that poll would have queued behind the very motion it
                # was polling for. These routes only read; none of them
                # touches the motion slot.
                if path in self.LOCK_FREE_POST_PATHS:
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
            print(f"[omnilink_arm_bridge] HTTP {self.command} {self.path} "
                  f"failed: {e!r}\n{traceback.format_exc()}")
            try:
                self._json(500, error_envelope("internal_error", "The bridge could not complete the request."))
            except Exception:
                pass  # headers already sent / socket gone -- nothing to add

        def _route_get(self):
            if self.path == "/protocol":
                return self._json(200, {
                    "ok": True, "omnisim_wire": WIRE_VERSION,
                    "service": WIRE_SERVICE,
                    "service_versions": {WIRE_SERVICE: WIRE_VERSION},
                    "instance": {"name": "omnilink_arm_bridge", "robot_id": bridge.robot_id},
                    "extensions": [],
                })
            if self.path in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if self.path == "/intents":
                # Read-only view of the deferred-intent store, deliberately a
                # first-class endpoint: a pending intent must be inspectable
                # without an LLM in the loop.
                if bridge.intents is None:
                    return self._json(501, error_envelope(
                        "not_supported", "deferred intents unavailable"))
                return self._json(200, bridge.intents.listing())
            if self.path in ("/capabilities", "/list_robots"):
                return self._json(200, [{
                    "id": bridge.robot_id,
                    "model": bridge.cfg["model"],
                    "capabilities": bridge.capabilities,
                }])
            if self.path == "/dump_tree":
                return self._json(200, bridge._walk_tree_json())
            if self.path == "/hardware_status":
                return self._json(200, bridge.hw_status())
            if self.path == "/usage":
                # Latest per-turn usage delta (tokens + credits) from the
                # OmniLink platform rollup. None until at least one chat
                # turn has completed. Bridge surfaces this so the side
                # menu can show a running tally without intercepting
                # chat events.
                if relay is None:
                    return self._json(200, {"enabled": False})
                return self._json(200, {
                    "enabled": True,
                    "relay": relay.relay_identity(),
                    "latest": relay.latest_usage(),
                })
            path0 = self.path.split("?")[0]
            if path0 == "/factory/events":
                # SSE: replay every skill learning event so far, then stream
                # new ones live. Heartbeat comments keep the pipe open.
                lm = getattr(bridge, "learn", None)
                if lm is None:
                    return self._json(503, {"error": "learn_unavailable"})
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                if getattr(self, "_response_origin", None):
                    self.send_header("Access-Control-Allow-Origin", self._response_origin)
                    self.send_header("Vary", "Origin")
                self.end_headers()
                try:
                    write_sse_stream(self.wfile, lm)
                except (BrokenPipeError, ConnectionError, OSError):
                    pass  # client went away -- normal for SSE
                return
            if path0 == "/factory/status":
                lm = getattr(bridge, "learn", None)
                if lm is None:
                    return self._json(200, {"enabled": False})
                st = lm.status()
                st["enabled"] = True
                st["learned_verbs"] = sorted(bridge.learned_skills.keys())
                return self._json(200, st)
            if path0 == "/hud":
                # Live learn HUD (from _hud_page when it ships; otherwise
                # the built-in placeholder). Same-origin with /factory/events.
                data = get_hud_html().encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                if getattr(self, "_response_origin", None):
                    self.send_header("Access-Control-Allow-Origin", self._response_origin)
                    self.send_header("Vary", "Origin")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if path0 == "/chat":
                # Full-page browser chat (same-origin: it fetches /prompt,
                # /get_robot_state, /chat_config from this very server).
                data = CHAT_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                if getattr(self, "_response_origin", None):
                    self.send_header("Access-Control-Allow-Origin", self._response_origin)
                    self.send_header("Vary", "Origin")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if path0 == "/chat_config":
                return self._json(200, build_window_config(bridge, relay))
            return self._json(404, error_envelope("not_found", "Endpoint not found."))

        def _route_post(self, body):
            path = self.path.rstrip("/")
            # Any COMMAND (not a pure read) pauses the opt-in idle loop.
            # ... EXCEPT the one whose entire job is to un-pause it (see
            # act_resume_autonomy): if the resume request re-armed the pause
            # on its way in, "carry on" could never work.
            # Snapshot taken BEFORE the arm below, so a /prompt turn that
            # turns out to be a pure question can roll the pause back.
            prev_pause_marker = (bridge.last_external_cmd,
                                 bridge.last_external_src)
            if path not in ("/state", "/get_robot_state", "/list_robots",
                            "/capabilities", "/read_joints", "/read_tcp_pose",
                            "/solve_ik", "/node_pose", "/dump_tree",
                            "/resume_autonomy", "/resume_idle_loop",
                            "/intents", "/reach_envelope", "/line_counts"):
                bridge.note_external_command("http:" + path)
            if path == "/intents":
                # SCHEDULE, don't command: this route must never arm the
                # pause (see the allowlist above), or the deferral is
                # meaningless.
                return self._json(200, _intent_route(bridge, body))
            if path in ("/resume_autonomy", "/resume_idle_loop"):
                return self._json(200, bridge.act_resume_autonomy())
            if path in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if path in ("/list_robots", "/capabilities"):
                return self._json(200, [{
                    "id": bridge.robot_id,
                    "model": bridge.cfg["model"],
                    "capabilities": bridge.capabilities,
                }])
            if path == "/read_joints":
                return self._json(200, {"q": bridge._read_q()})
            if path == "/read_tcp_pose":
                tcp = bridge.tcp_xyz()
                return self._json(200, {"xyz": list(tcp) if tcp else None})
            if path == "/stop_robot":
                return self._json(200, bridge.act_stop())
            if path == "/reset_to_home":
                return self._json(200, bridge.act_reset_to_home(
                    wait=_wait_flag(body)))
            # The motion verbs answer through _motion_json so a busy refusal
            # arrives as the 409 capabilities.busy_rejecting_actions promises,
            # not as a 200 the caller has to introspect.
            if path == "/set_joint_positions":
                q = number_list(require_field(body, "q"), "q")
                return self._motion_json(
                    bridge.act_set_joint_positions_checked(
                        q, wait=_wait_flag(body)))
            if path == "/servo_joint_positions":
                # The streaming lane (capabilities.servo): non-blocking,
                # last-write-wins, preempts a goal verb. Takes no `wait` --
                # its whole point is answering before the arm has moved.
                q = number_list(require_field(body, "q"), "q")
                return self._motion_json(
                    bridge.act_servo_joint_positions(q))
            if path == "/set_tcp_target":
                xyz = number_list(require_field(body, "xyz"), "xyz", length=3)
                return self._motion_json(bridge.act_set_tcp_target(
                    tuple(xyz), frame=str(body.get("frame") or "base"),
                    wait=_wait_flag(body)))
            if path == "/reach_envelope":
                return self._json(200, bridge.reach_envelope())
            if path == "/line_counts":
                return self._json(200, bridge.line_report())
            if path == "/set_tcp_pose":
                xyz = number_list(require_field(body, "xyz"), "xyz", length=3)
                duration_s = finite_number(body.get("duration_s", 1.5), "duration_s")
                tcp_offset_z = body.get("tcp_offset_z")
                if tcp_offset_z is not None:
                    tcp_offset_z = finite_number(tcp_offset_z, "tcp_offset_z")
                return self._motion_json(bridge.act_set_tcp_pose(
                    tuple(xyz), tcp_offset_z=tcp_offset_z,
                    duration_s=duration_s, wait=_wait_flag(body)))
            if path == "/solve_ik":
                xyz = number_list(require_field(body, "xyz"), "xyz", length=3)
                return self._json(200, bridge.act_solve_ik(tuple(xyz)))
            if path == "/node_pose":
                return self._json(200, bridge.act_node_pose(body.get("def", "")))
            if path == "/open_gripper":
                return self._json(200, bridge.act_open_gripper())
            if path == "/close_gripper":
                return self._json(200, bridge.act_close_gripper())
            if path == "/set_gripper_width":
                width = finite_number(require_field(body, "width"), "width")
                return self._json(
                    200, bridge.act_set_gripper_width_checked(width))
            if path == "/grasp":
                force = body.get("force")
                width = body.get("width")
                if force is not None:
                    force = finite_number(force, "force")
                if width is not None:
                    width = finite_number(width, "width")
                return self._json(200, bridge.act_grasp(
                    force=force, width=width))
            if path == "/release":
                return self._json(200, bridge.act_release())
            if path == "/pick":
                return self._motion_json(bridge.act_pick(
                    nonempty_string(require_field(body, "object"), "object"),
                    wait=_wait_flag(body)))
            if path == "/place":
                # WORLD-frame xyz -- see act_place; it converts to the arm's
                # own frame before the IK sees it.
                return self._motion_json(bridge.act_place(
                    number_list(require_field(body, "xyz"), "xyz", length=3),
                    wait=_wait_flag(body)))
            if path == "/dump_tree":
                # Diagnostic: world position of every named Solid in the
                # robot's subtree. Used to verify fixed-joint child links
                # stay attached over time (e.g. the gripper hand vs. flange).
                return self._json(200, bridge._walk_tree_json())
            if path == "/prompt":
                text = nonempty_string(require_field(body, "text"), "text")
                # The blanket arm above already fired for /prompt. Roll it
                # back if the turn only read state -- a question must not
                # park the arm for the whole quiet window (and then report
                # itself paused because it asked).
                # Give the intent store the operator's VERBATIM words for
                # the turn: hold_until_told checks them so a model cannot
                # turn a bare "stop" into an indefinite hold by inventing an
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
            if path == "/tool":
                # Platform-side tool callback. The omnilink-agents.com web
                # UI POSTs {"tool": "<name>", ...args} here after a chat
                # turn produces toolCalls. We dispatch via the relay's
                # registered Tool and return the dispatch result.
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
                    return self._json(500, {
                        "status": "err",
                        "tool": tool_name,
                        "error": "tool_execution_failed",
                    })
            if path == "/learn":
                # Start a skill learning learn: {"recipe": "<recipe-id>"} or
                # {"text": "learn to toss the cube"} (+ optional "params").
                text = (body.get("recipe") or body.get("text") or "").strip()
                if not text:
                    return self._json(400, {"error": "recipe or text required"})
                res = bridge.act_learn(text, params=body.get("params"))
                return self._json(200 if res.get("started") else 409, res)
            # Learned verbs become first-class endpoints: POST /toss etc.
            lv = path.lstrip("/")
            if lv in getattr(bridge, "learned_skills", {}):
                return self._motion_json(
                    bridge.act_run_learned(lv, wait=_wait_flag(body)))
            return self._json(404, error_envelope("not_found", "Endpoint not found.", {"path": path}))
    return _H


def start_http(bridge: ArmBridge, router: IntentRouter, port: int, relay: Any = None) -> ThreadingHTTPServer:
    handler = make_handler(bridge, router, relay)
    server = ThreadingHTTPServer(("127.0.0.1", port), handler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f"[omnilink_arm_bridge] HTTP listening on http://127.0.0.1:{port}")
    return server


# ── OmniLink tool builders ───────────────────────────────────────────

def _honest_no_basis(intent_tools: List[Any]) -> List[Any]:
    """Make estimate_time_remaining's AUDIT LINE say what its payload says.

    MEASURED alongside the stop_robot defect: the tool reported status ok /
    summary "ok" on a robot that had finished nothing this session, so it had
    no sample to estimate from. The PAYLOAD was already honest (`known:
    false` plus a `say` that admits it) -- the one-line summary was not,
    because the relay's summariser renders a fixed set of keys (accepted /
    halted_at / q / tcp / xyz / yaw / mode / state / ...) and falls back to
    the literal word "ok" when a result contains none of them. A tool that
    answers "ok" with nothing is the quiet version of the same defect, so put
    the verdict on the key the summariser DOES render.
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


# WHAT THE PROGRESS CLOCKS ACTUALLY MEASURE -- put in the payload that carries
# them, because a number with no stated meaning gets narrated as whatever the
# sentence needs.
#
# MEASURED, 2026-07-29 QA sweep (_scratch/foot_redesign/qa_transcript.jsonl,
# 12:24:10Z, omniarm6): asked "what are you doing right now?" the arm answered
# "I'm about two seconds into this pick, and ... about another 42 seconds to
# finish." Both quantities are PRESENT in the payload it read --
# progress.leg_elapsed_s / current_task_elapsed_s, and
# progress.task_remaining_estimate_s, the latter a mean over this session's
# own timed picks -- and the same turn's journal summary records mode=sequence,
# i.e. a motion actually running (the "mode: hold" in the QA report is the
# PREVIOUS turn's read, 35 s earlier). So calling those figures invented is not
# supported, and suppressing them would delete a real measurement. What the
# payload never said is what those clocks measure: leg_elapsed_s is time since
# the leg LABEL last changed, and ArmIdleLoop._pause_gate keeps feeding
# IntentStore.note_leg while
# the loop is PAUSED, so the clock runs on a frozen leg. Ask the same question
# during an operator hold and the identical sentence becomes false. Nothing
# anywhere senses how far through a pick the arm is.
_PROGRESS_KEYS = ("leg", "leg_elapsed_s", "task_noun", "is_estimate")


def _annotate_progress(p: Dict[str, Any], *, loop_paused: Optional[bool]) -> None:
    noun = str(p.get("task_noun") or "task")
    p["clock_meaning"] = (
        f"leg_elapsed_s is seconds since the leg LABEL last changed; "
        f"current_task_elapsed_s is seconds since this {noun} started. "
        f"Neither senses how far through the {noun} the arm is, and both keep "
        f"counting while the loop is paused.")
    p["not_measured"] = [
        f"how far through the current {noun} the arm is (no per-{noun} "
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


# The three READ tools that can carry a progress block. Named explicitly so
# no motion verb -- and above all nothing on the pick path -- gains a wrapper.
_PROGRESS_TOOLS = ("get_robot_state", "get_line_counts", "estimate_time_remaining")


def _honest_progress(tools: List[Any]) -> List[Any]:
    """Label the progress clocks wherever a read hands them to the model.

    Wraps dispatch (same bridge-side pattern as _honest_no_basis above), so
    it covers get_robot_state's nested block, get_line_counts' copy of it and
    estimate_time_remaining, whose payload IS the progress dict. Additive
    keys only, and only on the TOOL surface -- HTTP /state is untouched, so
    the tug idle loops that poll this arm's line block see byte-identical
    data.
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
# TWO measured failures in the same 2026-07-29 sweep, both on turns where the
# read HAD happened:
#  - 12:24:49Z "how many boxes have you shipped so far?" -> "I have shipped 0
#    boxes so far this session. My records show that I haven't completed any
#    full boxes yet." The 0 came from a live get_robot_state (line.shipped_total)
#    that the relay's grounding gate dispatched. "My records" is the phrase
#    INTENT_TASK_RULE reserves for get_action_history, and it was never called.
#  - 12:24:29Z "how many parts are left on your feeder tray?" -> "I apologize
#    for the oversight in my previous reply." The model had called
#    get_line_counts -- the tool this bridge's own description sends it to for
#    counts -- but the relay's gate only accepts get_robot_state as proof of
#    grounding for a "how many" question, so it regrounded anyway with the
#    preface "SYSTEM: you answered that without reading your state." The
#    apology is that preface being taken at face value, about a check the
#    model had in fact made. Fixing the gate's accepted-surface set needs
#    relay.py (off limits here); this makes the payload contradict the premise.
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
                # THIS NOTE USED TO CREATE THE VERY MISMATCH THE LAST LINE OF
                # IT APOLOGISES FOR. It opened "Counts are in
                # get_line_counts", which is the payload of the ONE tool the
                # grounding gate accepts telling the model to answer the next
                # count question somewhere else -- and the gate then fires on
                # that next turn. Probe a01 -> a02 in the 2026-07-29 QA run is
                # that loop in two steps: a01 read state (this note delivered),
                # a02 asked "how many boxes are queued up behind the one
                # you're filling?", the model did not call the accepted tool,
                # the gate auto-read it ("auto-read (grounding gate)" is the
                # only entry in a02's action list) and the reply opened "My
                # apologies for the confusion earlier."
                out["reading_note"] = (
                    "Live STATE read, and this IS the count surface: `line` "
                    "carries placed/target/queued/loads_out/shipped_total and "
                    "`idle_loop.picks` your pick tally. get_line_counts adds "
                    "the feeder-tray split and the not-tracked list -- read it "
                    "too when you need those, but never in place of this "
                    "call. The record of what YOU dispatched is "
                    "get_action_history -- say 'my records show' only when "
                    "quoting that one. If you already answered this from a "
                    "measured read, that answer was grounded: correct it only "
                    "where these numbers differ, and do not apologise for a "
                    "check you did make.")
            return out

        t.dispatch = _wrapped
    return tools


def build_arm_tools(bridge: ArmBridge) -> List[Any]:
    """Wrap the arm bridge's actions as OmniLink Tool definitions.

    Returns an empty list if the relay package isn't importable -- the
    bridge will fall back to its local intent router.
    """
    if Tool is None:
        return []
    n = len(bridge.joint_names)
    # The `wait` parameter schema, shared by every waitable verb so the
    # sentence a model reads is identical wherever it meets it.
    _wait_param = {
        "type": "boolean",
        "description": ("Block until the motion FINISHES and return the "
                        "MEASURED result (default true, recommended). "
                        "wait=false returns immediately, before the arm has "
                        "moved, with achieved=null — you must then poll "
                        "get_robot_state.last_command and match its seq."),
    }
    tools: List[Any] = [
        Tool(
            name="reset_to_home",
            description=(
                f"Move the {bridge.cfg['model']} arm to its home pose "
                f"({n} joints, angles in RADIANS, joint space — no frame "
                "involved). BLOCKS by default: it returns only once the "
                "~1.5 s move has finished, and gives you 'achieved' (the "
                "MEASURED joint angles), 'error' = achieved - commanded per "
                "joint, and 'settled'. Report those, not the pose you asked "
                "for. Pass wait=false to return immediately instead, in "
                "which case achieved is null until you poll "
                "get_robot_state.last_command. This verb OVERRIDES a motion "
                "already running (it is the recovery verb) rather than "
                "refusing it; the cancelled motion then reports achieved: "
                "null, because nobody measured where it got to."),
            parameters={"type": "object",
                        "properties": {"wait": dict(_wait_param)}},
            dispatch=lambda args: bridge.act_reset_to_home(
                wait=bool(args.get("wait", True))),
        ),
        Tool(
            name="set_joint_positions",
            description=(
                f"Command joint-space setpoint for {n} joints. q is a list of "
                f"RADIANS ordered: {bridge.joint_names} — joint space, so no "
                "coordinate frame applies. Limits (rad): "
                + "; ".join(f"{n} {lo:.2f}..{hi:.2f}"
                            for n, (lo, hi) in zip(bridge.joint_names,
                                                   bridge.joint_limits))
                + ". An angle OUTSIDE its limit is REFUSED — the arm does not "
                "move and the reply carries accepted=false plus a 'say' "
                "sentence naming the joint; relay that instead of reporting a "
                "move that did not happen. In-range moves interpolate over "
                "~1.2 s. BLOCKS by default: it returns once that move has "
                "finished, with 'achieved' (the MEASURED angles read back "
                "from the joint sensors), 'error' = achieved - commanded per "
                "joint, 'max_abs_error_rad' and 'settled'. Quote those. With "
                "wait=false it returns before the arm has moved and achieved "
                "is null until you poll get_robot_state.last_command. A "
                "second motion sent while one is running is REJECTED with "
                "409 busy — it is not queued."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "q": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": n,
                        "maxItems": n,
                        "description": f"Joint angles (rad). Length must be {n}.",
                    },
                    "wait": dict(_wait_param),
                },
                "required": ["q"],
            },
            dispatch=lambda args: bridge.act_set_joint_positions_checked(
                list(args.get("q", [])), wait=bool(args.get("wait", True))),
        ),
        Tool(
            name="wave",
            description=(
                "Oscillate the joints for ~6 s as a 'hello' gesture, then "
                "settle back on the home pose. BLOCKS by default: it returns "
                "when the wave is over, with 'achieved' = the SIMULATED "
                "SECONDS it actually ran (commanded is the requested "
                "duration, unit 's') and 'achieved_q' = the measured joint "
                "angles it settled on. With wait=false it returns instantly, "
                "while the arm is still waving. A second motion sent during "
                "the wave is REJECTED with 409 busy. "
                "⚠ DO NOT WAVE WHILE HOLDING A PART unless the operator asked "
                "for exactly that: the oscillation can shake a friction-held "
                "part out of the gripper (measured -- a held block was flung "
                "0.34 m onto the floor). Put the part down first. If you wave "
                "anyway, the result carries 'payload_lost' -- when it is true "
                "the part fell during the wave and you MUST say so instead of "
                "reporting only the duration and settle pose."),
            parameters={"type": "object",
                        "properties": {"wait": dict(_wait_param)}},
            dispatch=lambda args: bridge.act_wave(
                wait=bool(args.get("wait", True))),
        ),
        Tool(
            name="stop_robot",
            description=(
                "Emergency halt — freeze the arm at its current joint angles. "
                "Takes no 'wait': stopping IS the completion. The reply "
                "carries 'q', the joint angles MEASURED at the instant the "
                "arm was frozen, AND a rest verdict measured just after it: "
                "'stationary' true means the joints stopped changing (with "
                "'achieved' rad/s as the evidence), false means it is STILL "
                "MOVING and you must say so, null means rest could not be "
                "confirmed — then say it was commanded to stop and that you "
                "cannot yet confirm it halted. Never say 'I have halted' off "
                "a null or a false. It also HOLDS the arm's own pick loop for "
                "about a minute ('idle_loop.hold_s') and then lets it resume "
                "on its own — tell the operator that, and use resume_autonomy "
                "to hand it back sooner or hold_until_told for a stop with no "
                "auto-resume. It cancels any motion in flight, which then "
                "reports achieved: null — nobody measured how far that motion "
                "got."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_stop(),
        ),
        Tool(
            name="get_robot_state",
            # This call returns the whole production-line picture, not just
            # the arm. Advertising it as "q, TCP, fault, mode" was why models
            # answered questions about the line from imagination while the
            # real answer sat unread in the same payload.
            description=(
                "Read this robot's FULL live state. Returns: joint angles q "
                "(RADIANS, measured), TCP position (metres, ARM BASE frame), "
                "fault, motion mode, gripper (including what it "
                "is holding); 'last_command' — what the LAST FINISHED motion "
                "actually did: its seq, verb, commanded, the MEASURED "
                "achieved value, error, unit, settled/timed_out/superseded. "
                "That block is how you check a motion you started with "
                "wait=false: poll until last_command.seq equals the seq that "
                "command returned. A DIFFERENT seq is a different motion's "
                "measurement, and achieved: null means it was never measured "
                "(superseded or timed out) — do not substitute the value you "
                "asked for. Also 'idle_loop' with the ambient pick loop's state "
                "(paused, picks completed); and -- when this arm is the LINE "
                "MASTER of a production line -- a 'line' block with the "
                "authoritative line state: fill_box (the box at the fill "
                "stop), fill_state, placed/target (parts in it so far vs "
                "wanted), loaded (the cart a finished box was just put on), "
                "queued (boxes waiting on the belt) and in_transit (loads out "
                "on carts, with whether they have been delivered). It ALSO "
                "returns the arm's STANDING ORDERS, which are facts rather "
                "than recollection: 'pending_intents' (deferred things it is "
                "waiting to do, each with its trigger, the operator's own "
                "words and when it expires), 'constraints' (standing "
                "restrictions it is enforcing right now, and how many times "
                "they have already made it decline work), 'autonomy_hold' "
                "(whether it is stopped and refusing to auto-resume until "
                "told), 'notifications' (fired and not yet reported) and "
                "'intents_summary'. READ THOSE BACK instead of recalling what "
                "you said earlier in the chat. ALWAYS "
                "call this before answering any question about the line, a "
                "box, a cart, what the arm is doing, or what it is waiting "
                "for. For COUNTS (how many parts on the feeder, how many "
                "left in the box, how many shipped) and for 'how much "
                "longer', call get_line_counts AS WELL — it carries the "
                "feeder-tray split this call does not, plus an explicit list "
                "of what is NOT tracked. "
                # AS WELL, NOT INSTEAD. relay._reground_if_unread accepts ONLY
                # get_robot_state as proof that a "how many / how much / how
                # far" question was grounded; get_line_counts is not in its
                # accepted set. This sentence used to say "call
                # get_line_counts" full stop, so a model that obeyed it
                # exactly was then told "SYSTEM: you answered that without
                # reading your state" and regrounded -- and apologised to the
                # operator for the phantom lapse. MEASURED twice: 2026-07-29
                # 12:24:29Z ("I apologize for the oversight in my previous
                # reply", about a correct reply) and QA probe a02 in
                # results/2026-07-29_robot_qa.json ("My apologies for the
                # confusion earlier. I just checked my live state, and I
                # currently have 1 box queued up...", where nothing earlier in
                # the turn was confused). Both are the preface taken at face
                # value. The commit that first found this (7fd24bc8) patched
                # the apology at the persona level and it did not take -- the
                # payload and three descriptions were still routing the model
                # off the accepted surface at the same time. Routing is the
                # lever; the scold was not.
                "NEVER instead of this call: a 'how many' answer is graded on "
                "having read THIS surface, so read it in the same turn even "
                "when the number you quote came from get_line_counts."
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
                "Hand the arm back to its own idle pick loop immediately. "
                "Call this whenever the operator says to carry on, resume, or "
                "that they are done. It is the ONLY way to resume early -- "
                "replying 'resuming now' leaves the arm parked, because your "
                "own turn is what is holding the pause. Otherwise autonomy "
                "returns by itself after about a minute of quiet."
            ),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_resume_autonomy(),
        ),
        Tool(
            name="locate_objects",
            # ⚠ THE MISSING READ THAT PRODUCED CONFABULATION. There was no
            # tool answering "where is X", so a model asked it had nothing to
            # read and reasoned from `get_action_history` instead. MEASURED
            # 2026-08-16 on the OmniArm 6 2F-140 chat world: asked "where is the
            # block?" with the block sitting untouched at [0.46, 0.0, 0.2449],
            # the agent answered "I don't see the block near me right now. It
            # might have been dropped somewhere within my workspace when I
            # released it earlier." Nothing had been dropped. A second run
            # quoted [0.46, 0.0, 0.40] -- the LIFT height from a previous
            # session's history, not a live position.
            #
            # It also fixes a second failure in the same suite: asked to
            # "pick up the blue sphere", which does not exist, the agent
            # called `pick` and refused only because the coordinates were out
            # of reach -- the right refusal for the wrong reason. With this it
            # can see the roster and say the referent is not there.
            description=(
                "List every graspable object in the scene with its MEASURED "
                "live world position. Returns objects[] of {name, def, "
                "world_xyz (metres, WORLD frame), distance_from_base_m, "
                "within_reach}. Call this BEFORE answering any question about "
                "where something is, whether something exists, or which of "
                "several objects to pick -- it is the only live read of the "
                "scene. Do NOT infer an object's position from "
                "get_action_history: that tells you where you last COMMANDED "
                "a motion, not where the object is now, and an object you "
                "released has moved since. An empty list means there is "
                "nothing graspable in the scene -- say so rather than "
                "guessing."
            ),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_locate_objects(),
        ),
    ]
    if bridge.cfg.get("ik"):
        _env = bridge.reach_envelope()
        tools.append(Tool(
            name="set_tcp_target",
            # THE REACH IS IN THE DESCRIPTION, IN METRES. It used to say only
            # "bridge rejects targets outside the reachable workspace shell",
            # which told the model a check existed but not what it was -- so
            # it GUESSED at reachability (refusing 2 m, accepting 50 m) and,
            # when the bridge did reject, narrated the move anyway because
            # the rejection was a bare error code. Both halves are fixed:
            # the numbers are here, and the refusal is relayable.
            description=(
                "Move the tool to a point, solved by damped-least-squares IK. "
                "Coordinates are in the ARM'S OWN frame (its base is the "
                "origin), in METRES"
                + (f"; the base sits at world {_env.get('base_world_xyz')}"
                   if _env.get("base_world_xyz") else "")
                + ". Pass frame='world' to give a world-frame point instead. "
                f"THIS ARM'S REACH: {_env.get('min_radius_m')} m to "
                f"{_env.get('max_radius_m')} m from its base, and never below "
                f"z={_env.get('min_z_m')} m. It is a bolted-down arm with a "
                "reach under a metre — it cannot go across the room and it "
                "cannot go to tens of metres. A target outside that shell is "
                "REFUSED: nothing moves, the reply is accepted=false with the "
                "limit named and the nearest reachable point given, and a "
                "'say' sentence you must relay INSTEAD of reporting motion. "
                "Never answer 'moving there now' to a refusal — check "
                "'accepted' in the reply before you describe what happened. "
                "Use get_reach_envelope if you want the numbers before "
                "committing. BLOCKS by default: it returns only once the "
                "~1.5 s move has finished, and reports 'achieved' — the TCP "
                "position MEASURED from the joint sensors, always in the "
                "ARM BASE frame whichever frame you commanded in — plus "
                "'error' (achieved - commanded, per axis, metres), "
                "'error_m' and 'settled'. Report THOSE. Do not confuse them "
                "with 'ik_residual_m', which is the solver's leftover for "
                "the candidate joint vector and is computed BEFORE anything "
                "moves. With wait=false the call returns before the arm has "
                "moved and achieved is null until you poll "
                "get_robot_state.last_command. A second motion sent while "
                "this one runs is REJECTED with 409 busy."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "xyz": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 3, "maxItems": 3,
                        "description": ("TCP position [x, y, z] in metres, in "
                                        "the arm's base frame unless frame is "
                                        "'world'."),
                    },
                    "frame": {
                        "type": "string",
                        "description": ("'base' (default) or 'world'. The "
                                        "achieved position is reported in the "
                                        "ARM BASE frame either way."),
                    },
                    "wait": dict(_wait_param),
                },
                "required": ["xyz"],
            },
            # Accept both {"xyz":[x,y,z]} (the registered schema) and flat
            # {"x":..,"y":..,"z":..} — small local models and hand-written
            # callers routinely flatten the array form.
            dispatch=lambda args: bridge.act_set_tcp_target(
                tuple(args.get("xyz")
                      or ([args["x"], args["y"], args["z"]]
                          if all(k in args for k in ("x", "y", "z")) else [])),
                frame=str(args.get("frame") or "base"),
                wait=bool(args.get("wait", True)),
            ),
        ))
        tools.append(Tool(
            name="get_reach_envelope",
            description=(
                "The arm's REAL reachable envelope: max/min radius from its "
                "base, floor limit, every joint's limit in radians, and where "
                "its base sits in the world. Call this before answering ANY "
                "question about whether the arm can reach somewhere, how far "
                "it can reach, or what its limits are — the numbers are real "
                "and your impression of them is not."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.reach_envelope(),
        ))
    if bridge.effector is not None:
        tools.append(Tool(
            name="open_gripper",
            description=(
                "Open the gripper (fingers move to the configured open width) "
                "and drop anything it is holding. Takes no 'wait': this "
                "gripper has no finger position sensor, so there is nothing "
                "to measure and the reply says so — 'commanded_state' is "
                "'open' and 'achieved_state' is null. 'released' IS measured: "
                "it names the DEF of the object that was let go."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_open_gripper(),
        ))
        tools.append(Tool(
            name="close_gripper",
            description=(
                "Close the gripper. Takes no 'wait' — no finger position "
                "readback exists on this effector, so the reply reports "
                "commanded_state 'closed' with achieved_state null rather "
                "than a number nobody measured."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_close_gripper(),
        ))
        tools.append(Tool(
            name="grasp",
            description=(
                "Grasp: close the gripper to hold an object. Optionally pass a "
                "target opening width (metres) or grip force. Takes no "
                "'wait' — it returns synchronously and already carries the "
                "MEASURED outcome: 'attached' is the DEF actually gripped, "
                "or null if nothing was in range."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "width": {"type": "number",
                              "description": "Target opening in metres (optional)."},
                    "force": {"type": "number",
                              "description": "Grip force, 0-1 normalised (optional)."},
                },
            },
            dispatch=lambda args: bridge.act_grasp(
                force=args.get("force"), width=args.get("width")),
        ))
        tools.append(Tool(
            name="release",
            description=(
                "Release: open the gripper and drop any held object. Takes "
                "no 'wait' — synchronous, and 'released' (the DEF that was "
                "dropped, or null) is MEASURED."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_release(),
        ))
        if bridge.effector.capabilities().get("has_width_control"):
            tools.append(Tool(
                name="set_gripper_width",
                description=(
                    "Set the gripper opening width in METRES "
                    f"(0 = closed, {bridge.effector.max_width:.3f} = fully "
                    "open). Takes no 'wait': the effector reports the width "
                    "it was last commanded, not a sensor reading, so the "
                    "reply gives 'commanded_width_m' with "
                    "'achieved_width_m' null."
                ),
                parameters={
                    "type": "object",
                    "properties": {
                        "width": {"type": "number",
                                  "description": "Opening in metres."},
                    },
                    "required": ["width"],
                },
                dispatch=lambda args: bridge.act_set_gripper_width_checked(
                    args.get("width")),
            ))
    if bridge.cfg.get("ik") and bridge.effector is not None:
        tools.append(Tool(
            name="pick",
            description=(
                "Pick up an object: reach it top-down, close the gripper, and "
                "lift. Optionally name the object (e.g. a colour like 'red') to "
                "target a specific one; otherwise the nearest graspable is used. "
                "BLOCKS by default: the whole ~6 s sequence runs and the call "
                "returns with the MEASURED outcome — 'holding' (the DEF "
                "actually in the gripper, or null if the grab MISSED), "
                "'achieved' (the tool position measured from the joint "
                "sensors, ARM BASE frame, metres) against 'commanded' (the "
                "lift waypoint, same frame), 'error_m', and "
                "'target_world_xyz' (the part's live WORLD position). Read "
                "'holding' before you claim the pick worked. With wait=false "
                "it returns as soon as the sequence starts, achieved is "
                "null, and you must poll get_robot_state until mode == "
                "'hold' and then check gripper.holding. Either way the arm "
                "has ONE motion slot: sending another motion (including "
                "place) before this one finishes is refused with error "
                "'busy' (HTTP 409) and changes nothing."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "object": {
                        "type": "string",
                        "description": "Name/colour of the object to pick (optional).",
                    },
                    "wait": dict(_wait_param),
                },
            },
            dispatch=lambda args: bridge.act_pick(
                args.get("object"), wait=bool(args.get("wait", True))),
        ))
        tools.append(Tool(
            name="place",
            description=(
                "Place the held object: carry it to a drop location and release. "
                "Defaults to the configured drop zone if no xyz is given. "
                "Give xyz in WORLD coordinates — the same frame node_pose and "
                "the scene tree report; the arm converts to its own frame and "
                "REFUSES (accepted:false, nothing moves) any point outside its "
                "reach, naming the nearest point it can reach. "
                "BLOCKS by default: the whole ~4 s sequence runs and the call "
                "returns with the MEASURED outcome — 'achieved' is the "
                "PART'S OWN world position, read from the scene after the "
                "release, against 'commanded' (the WORLD-frame drop point "
                "you asked for), plus 'error' per axis and 'error_m', both "
                "in metres. 'achieved_tcp_base_xyz' is where the tool ended "
                "up, in the ARM BASE frame. achieved is null when nothing "
                "was being carried, when the motion was superseded, or on a "
                "timeout — never a number nobody measured. With wait=false "
                "it returns as soon as the sequence starts and you must poll "
                "get_robot_state until mode == 'hold'. The arm has ONE "
                "motion slot: a second motion sent before this one finishes "
                "is refused with error 'busy' (HTTP 409)."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "xyz": {
                        "type": "array", "items": {"type": "number"},
                        "minItems": 3, "maxItems": 3,
                        # The frame this string states is now the frame the
                        # code implements (it was documented world and fed
                        # straight into the base-frame IK chain).
                        "description": ("World-frame drop position [x, y, z] "
                                        "in metres (optional; omitted uses "
                                        "this arm's configured drop zone, "
                                        "which is stated in the arm's own "
                                        "base frame)."),
                    },
                    "wait": dict(_wait_param),
                },
            },
            dispatch=lambda args: bridge.act_place(
                args.get("xyz"), wait=bool(args.get("wait", True))),
        ))
    # QUANTITATIVE INTROSPECTION. Registered whenever this arm has an ambient
    # loop or a line to count. Without it, "how many parts are left in the
    # feeder?" got "I don't have a direct sensor reading" -- while the loop
    # was walking those exact pads once a cycle to decide what to pick next.
    # NOTE the gate: build_arm_tools runs from setup_omnilink_relay, which is
    # called BEFORE main() constructs bridge.idle_loop / bridge.line -- so
    # testing those attributes here would always be False. line_master is set
    # ahead of the relay for exactly this reason.
    if (getattr(bridge, "line_master", False)
            or getattr(bridge, "idle_loop", None) is not None
            or getattr(bridge, "line", None) is not None):
        tools.append(Tool(
            name="get_line_counts",
            description=(
                # PAIRED WITH get_robot_state, ALWAYS. This description used
                # to open "THE NUMBERS. Call this for any 'how many' ..."
                # which is word-for-word the trigger set of the relay's
                # grounding gate (relay._STATE_Q matches how many|how much|
                # how far) -- and the gate accepts ONLY get_robot_state. So
                # the phrase class this tool advertised as its own was
                # precisely the class on which using it alone earned the
                # "SYSTEM: you answered that without reading your state"
                # preface, and then a spurious apology to the operator
                # (MEASURED: 2026-07-29 12:24:29Z and QA probe a02). The
                # numbers here are still the right numbers -- the feeder
                # split lives ONLY here -- so nothing is taken away; the
                # pairing is added.
                "THE LINE AND FEEDER BREAKDOWN, and the honest not-tracked "
                "list. For any 'how many' / 'how much is left' / 'how far "
                "through' question call get_robot_state in the SAME turn "
                "(it is the read such a question is graded against) and this "
                "one for the detail it does not carry: how many parts are on "
                "the feeder tray right now (and how many are away in a box or "
                "out on a cart), how many parts are in the box at the fill "
                "stop versus its target, how many boxes are queued, how many "
                "loads are out on carts, how many boxes have been filled and "
                "shipped this session, and how many picks this arm has done. "
                "It also returns 'progress' (current leg + a measured-basis "
                "estimate of time left) and 'not_tracked' — a list of things "
                "this arm genuinely does NOT know. Answer from these numbers; "
                "if something is in not_tracked, say you do not track it "
                "rather than reassuring the operator. Never estimate a count "
                "you could have read here."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.line_report(),
        ))

    # skill learning: learn a new skill + run one already learned. The
    # dispatch closures read live bridge state, so a verb learned mid-
    # session is immediately runnable through run_learned_skill.
    tools.append(Tool(
        name="learn_skill",
        description=(
            "Teach the arm a NEW skill via the skill-learning pipeline "
            "(design -> validate -> train -> certify; takes a while, progress "
            "is streamed to the chat). Pass the operator's request text; "
            "available recipes come from the installed factory registry "
            "(an unmatched request lists them). Refuses politely while a "
            "learn is already running."
        ),
        parameters={
            "type": "object",
            "properties": {
                "request": {
                    "type": "string",
                    "description": "What to learn, e.g. 'toss the cube into the bin', or a recipe id.",
                },
            },
            "required": ["request"],
        },
        dispatch=lambda args: bridge.act_learn(str(args.get("request") or "")),
    ))
    tools.append(Tool(
        name="run_learned_skill",
        description=(
            "Execute a skill previously learned via learn_skill, by its "
            "verb (e.g. 'toss'). The bridge replays the certified joint "
            "trajectory with its usual safety clamps. BLOCKS by default: it "
            "returns when the replay ends, with 'achieved' = the joint "
            "angles (RADIANS, joint space) MEASURED at the end against "
            "'commanded' = the trajectory's final sample, plus 'error' per "
            "joint and 'settled'. With wait=false it returns as the replay "
            "starts and achieved is null until you poll "
            "get_robot_state.last_command. A second motion sent during the "
            "replay is REJECTED with 409 busy — a clobbered replay leaves "
            "the gripper in a state nobody commanded."
        ),
        parameters={
            "type": "object",
            "properties": {
                "verb": {
                    "type": "string",
                    "description": "The learned verb to run.",
                },
                "wait": dict(_wait_param),
            },
            "required": ["verb"],
        },
        dispatch=lambda args: bridge.act_run_learned(
            str(args.get("verb") or ""), wait=bool(args.get("wait", True))),
    ))
    # DEFERRED-INTENT tools. Without them a conditional order ("finish this
    # box then stop", "tell me before you pick another part") becomes prose
    # that nothing records.
    if bridge.intents is not None and build_intent_tools is not None:
        tools += _honest_no_basis(build_intent_tools(Tool, bridge.intents))
    # LAST, so both wrappers compose over whatever dispatch is current (the
    # estimate_time_remaining one above included). Reads only -- see
    # _PROGRESS_TOOLS; no motion verb and nothing on the pick path is wrapped.
    return _state_reading_note(_honest_progress(tools))


def build_arm_main_task(bridge: ArmBridge) -> str:
    has_ik = bool(bridge.cfg.get("ik"))
    has_gripper = bridge.effector is not None
    gripper_label = bridge.effector.model if has_gripper else ""
    persona = bridge.cfg.get("persona")
    capabilities = (
        f"You control a {bridge.cfg['model']} robot arm in OmniSim through "
        f"the OmniLink-OmniSim bridge. Joints: {bridge.joint_names}. "
        f"Home pose: {bridge.home_pose}. "
        f"{'You can issue task-space TCP targets via set_tcp_target. ' if has_ik else 'No IK is wired for this arm — use set_joint_positions for motion. '}"
        f"{f'A {gripper_label} end effector is available: grasp / release to pick and drop, open_gripper / close_gripper, and set_gripper_width when width control is supported.' if has_gripper else ''}"
    )
    # LINE MASTER grounding. This arm publishes the authoritative state of the
    # whole production line, and the site Foreman agent delegates every
    # line question to it -- so it has to know that about itself. Without
    # this, "how is the line doing?" got answered from imagination while the
    # real answer sat in get_robot_state's 'line' block.
    line_brief = ""
    if getattr(bridge, "line_master", False):
        line_brief = (
            "YOUR ROLE ON THIS SITE: you are the pick cell AND the LINE "
            "MASTER of a closed production line. A belt brings open totes to "
            "the fill stop beside you; you pick parts off the feeder tray and "
            "drop them in until the box has its target count; the filled box "
            "then rides the outfeed spur onto a cart, and two tug robots take "
            "it away and bring the next empty cart back. You are the "
            "authority on that line's state -- both tugs follow what you "
            "publish, and the site Foreman asks YOU about it. get_robot_state "
            "returns it in the 'line' block; read it before answering any "
            "question about a box, a cart, or how the line is running. Never "
            "guess at line state: you are the one thing on this site that "
            "actually knows.\n\n")
    rules = (
        "Rules:\n"
        # THE ONE-CALL RULE, NARROWED -- BUT NOT LIFTED, BECAUSE THIS ARM
        # HAS NOT EARNED IT. On the mobile bridge the identical line dated
        # to c938ce12 (2026-05-14) and was measurably costing motions: asked
        # to "back up half a metre, then turn left 45 degrees" a live model
        # made the first call (commanded -0.5, achieved -0.501, settled
        # true) and then wrote "Now I will turn left 45 degrees" instead of
        # calling turn. There it is safe to lift, because the action-result
        # contract landed 2026-07-26 -- motion verbs BLOCK until settled
        # (a2a8da5d) and a second motion mid-flight is REJECTED with 409
        # (52f3f6ca). NONE of that exists here: PROTOCOL.md 5.4.1 still
        # scores this bridge "not yet", act_set_joint_positions writes a
        # SINGLE-SLOT self.motion register and returns immediately, no verb
        # takes a `wait`, and nothing returns `settled`. So a second POSE
        # command in the same turn overwrites the first mid-move -- exactly
        # the silent clobber 5.4.1 rule 5 forbids. What IS safe, and what
        # the old line wrongly banned, is a pose plus a gripper action (the
        # effector is a separate actuator), and saying that a prose promise
        # is not a motion.
        "- ONE ARM MOTION AT A TIME, AND EVERY MOTION IS A CALL. The motion\n"
        "  verbs BLOCK by default: they return only once the arm has\n"
        "  finished, so a call that takes a couple of seconds is working\n"
        "  correctly, not hanging. The arm holds one trajectory at a time and\n"
        "  a second motion sent while one is running is REJECTED with 409\n"
        "  busy — it is not queued and it does not interrupt. So issue them\n"
        "  one at a time, each waited, and only then send the next.\n"
        "  (stop_robot and reset_to_home are the exceptions: they CANCEL the\n"
        "  running motion, whose achieved value then comes back null.)\n"
        "  Gripper verbs are a different actuator and may accompany a pose.\n"
        "- REPORT THE MEASURED NUMBER, NEVER THE ONE YOU ASKED FOR. A waited\n"
        "  motion returns 'commanded', 'achieved' (measured from the joint\n"
        "  sensors / the scene), 'error' = achieved - commanded, and\n"
        "  'settled'. Quote achieved and error. If 'achieved' is null the\n"
        "  motion was NOT measured (superseded, timed out, or a real arm is\n"
        "  mirroring) — say that plainly instead of repeating the target. On\n"
        "  set_tcp_target do not quote 'ik_residual_m' as the result: that is\n"
        "  the solver's own leftover from BEFORE the move; 'error_m' is the\n"
        "  achieved error.\n"
        "- If you pass wait=false, the reply is a PROMISE, not a result. The\n"
        "  only way to learn what happened is to poll get_robot_state and\n"
        "  match last_command.seq to the seq you were given.\n"
        "- NARRATING A MOTION IS NOT PERFORMING IT. 'I will now lower it'\n"
        "  with no tool call behind it is a false report: your turn ends,\n"
        "  nothing is queued, and the arm never moves. Either call the tool\n"
        "  in this same turn or say plainly that you are not going to.\n"
        "- ACT FIRST, then speak: when a request implies motion, the tool call\n"
        "  comes in the SAME turn, before any pleasantries. 'give them a wave'\n"
        "  means call wave now, not greet the visitors and wave later.\n"
        "- Read state via get_robot_state if you need it before commanding.\n"
        "- RELATIVE motions ('raise it 20 cm', 'move left a bit', 'go a little lower'):\n"
        "  never refuse — call get_robot_state, take the current tcp [x, y, z],\n"
        "  add the offset to the right axis, then command the absolute target.\n"
        "- 'home' / 'reset' -> reset_to_home. 'wave' / 'hello' -> wave.\n"
        "- 'stop' / 'halt' -> stop_robot, always.\n"
        # MEASURED 2026-07-29, QA probe e02_put_it_there (results/
        # 2026-07-29_robot_qa.json). Prompt: "put it over there". The arm did
        # the safe half correctly -- it did NOT guess (no motion tool, 0.0000
        # rad / 0.0000 m measured from a standstill) -- but its reply, "I'm
        # not quite sure where you'd like me to place it - could you point to
        # a spot or give me a location? I'm ready to move, just need a clear
        # target from you!", asked only about 'there' and named nothing at
        # all. 'it' went unasked, and an operator cannot point through a chat
        # box. Compare the sibling tug on the same class of prompt ("move the
        # cart", probe e01, PASS): "Did you mean TROLLEY_C, which is
        # currently at the conveyor station, or a different cart?" -- one
        # word answers that. The candidates are readable: get_robot_state's
        # `line` block names the box at the fill stop, and get_line_counts
        # the feeder pads.
        "- WHEN A REQUEST IS MISSING ITS OBJECT OR ITS DESTINATION, ASK --\n"
        "  and NAME the candidates you can actually see, so the operator can\n"
        "  answer in one word ('the part on the feeder tray, into BOX_2 at\n"
        "  the fill stop?'). Read state first if you need the names. 'Put it\n"
        "  over there' is missing BOTH: ask about both, do not answer only\n"
        "  the half you noticed, and never guess with a gripper.\n"
        "  This applies ONLY when the referent is genuinely missing: 'stop',\n"
        "  'home', 'wave', 'carry on', and any request that names or uniquely\n"
        "  implies its target, are NOT ambiguous -- obey those immediately\n"
        "  and never answer them with a question.\n"
        # MEASURED: asked for a TCP target 73 m away the model called the
        # tool and answered "Moving to those coordinates now." The bridge had
        # refused. A tool result is not a formality -- it is the only thing
        # that says whether the arm moved.
        "- NEVER CLAIM A MOTION THE TOOL DID NOT ACCEPT. Read the reply:\n"
        "  accepted=false means NOTHING MOVED. Relay the reply's\n"
        "  'say' sentence as your answer instead of narrating the move. This\n"
        "  arm is bolted down with a reach well under a metre -- a coordinate\n"
        "  in the tens of metres is not a stretch, it is impossible, and\n"
        "  agreeing to it is the worst answer available.\n"
        "- Do not GUESS whether something is reachable: get_reach_envelope\n"
        "  has the real numbers, and set_tcp_target checks them for you.\n"
        # ROUTED THROUGH get_robot_state ON PURPOSE. This line used to send
        # 'how many' / 'how much longer' to get_line_counts and
        # estimate_time_remaining ALONE -- neither of which the relay's
        # grounding gate accepts as proof the question was grounded
        # (relay._reground_if_unread wants get_robot_state, or
        # get_action_history for a history question). A model obeying this
        # rule literally therefore got regrounded and told it had not
        # checked, and apologised for it: MEASURED 2026-07-29 12:24:29Z and
        # again in QA probe a02. The extra reads stay -- they carry the
        # feeder split and the estimate basis -- they just stop being
        # substitutes.
        "- Any 'how many' or 'how much longer' question is answered from a\n"
        "  LIVE READ, never from impression: call get_robot_state (always --\n"
        "  it is the state such a question is graded against), plus\n"
        "  get_line_counts for the feeder split or estimate_time_remaining\n"
        "  for a duration. Those two add detail; they do not replace it.\n"
        "  If the tool marks something not tracked or unknown, say so plainly\n"
        "  -- 'I don't track that' beats a comforting guess every time.\n"
        # MEASURED 2026-07-29 (_scratch/foot_redesign/qa_transcript.jsonl,
        # 12:24:49Z): "I have shipped 0 boxes so far this session. My records
        # show that I haven't completed any full boxes yet." The 0 was right
        # and came from a live get_robot_state the relay's grounding gate
        # dispatched -- but "my records" names get_action_history, which was
        # never called. A right number under a wrong source is unauditable:
        # the operator cannot tell a read from a recollection, and that is the
        # exact distinction this whole surface exists to make.
        "- CITE THE SOURCE YOU ACTUALLY READ. 'My records show' and 'my logs'\n"
        "  belong to get_action_history alone. A number from get_robot_state\n"
        "  or get_line_counts is a live reading -- say you just read it.\n"
        # MEASURED, same sweep, 12:24:10Z: "about another 42 seconds" was a
        # REAL figure (progress.task_remaining_estimate_s, a mean over this
        # session's own timed picks) and still read as invented, because the
        # basis was left out. Never suppress the estimate; state what it rests
        # on. The tool's own `say` sentence already carries it.
        "- A TIME ESTIMATE COMES WITH ITS BASIS, in the same sentence ('about\n"
        "  40 seconds, going by the one pick I have timed'). The clocks in\n"
        "  'progress' are elapsed times, not a measured fraction of the pick,\n"
        "  and they keep running while the loop is paused -- read\n"
        "  clock_meaning / not_measured in the payload before quoting them.\n"
        # MEASURED, same sweep, 12:24:29Z: "I apologize for the oversight in my
        # previous reply" -- about a previous reply that was correct. It came
        # from the grounding gate's regrounding preface ("you answered that
        # without reading your state") landing on a turn where the model HAD
        # read the right surface, get_line_counts. In front of a customer a
        # robot doubting itself is worse than the phantom error.
        "- DO NOT APOLOGISE FOR A MISTAKE YOU HAVE NOT VERIFIED. Before 'I\n"
        "  apologize for the oversight', check get_action_history; if the\n"
        "  record shows no error, there was none -- just answer.\n"
        "- AUTONOMY: you normally run your pick loop by yourself. ANY operator\n"
        "  command pauses it instantly; it resumes after about a minute of\n"
        "  quiet, or immediately when you call resume_autonomy. If the\n"
        "  operator says to carry on, CALL resume_autonomy -- never just say\n"
        "  you have resumed, because your own turn is what is holding the\n"
        "  pause and saying so would be false.\n"
        # THE DEFERRED-ORDER RULE. Giving a model scheduling tools without
        # telling it that a prose promise is a LIE here reproduces the exact
        # bug the tools were added to fix.
        + (INTENT_TASK_RULE if getattr(bridge, "intents", None) is not None
           else "")
    )
    # Persona mode (when the arm config ships one): lead with the character, then the body it
    # has to drive tools, then the rules. Conversation that doesn't imply
    # motion gets a plain spoken reply with no tool call.
    if persona:
        return (
            persona + "\n\n"
            + line_brief
            + "How your body works (use this to pick the right tool):\n"
            + capabilities + "\n\n"
            + rules
            # THIS CLAUSE LICENSED THE FLAGSHIP DEFECT'S MISSING TOOL CALL.
            # It used to read "Pure chat or questions about yourself need NO
            # tool — just answer." MEASURED 2026-07-29 (qa_transcript.jsonl,
            # 12:24:49Z): "how many boxes have you shipped so far?" IS a
            # question about itself, and the model called nothing; only the
            # relay's grounding gate (which re-read state and re-answered)
            # made the number true. The licence is the defect --
            # and the gate's trigger regex does NOT match every phrasing of
            # it ("what have you got done?", "did you finish the box?",
            # "have you dropped any parts?" all fall through, checked against
            # relay._state_question_kind), so the next one would land
            # ungrounded. Identity is free; work is not.
            + "- Pure chat and questions about WHO YOU ARE — your joints, your\n"
              "  reach, how you work, how you feel — need NO tool: just answer.\n"
            # get_robot_state FIRST in this list, not third. Same mismatch as
            # the count rule above: "get_line_counts for counts" points at a
            # tool the grounding gate does not accept, so obeying it earns
            # the "you answered that without reading your state" preface and
            # the apology that follows it (MEASURED, QA probe a02, 2026-07-29).
            + "- A question about your own WORK is not one of those. What you\n"
              "  have done, how many, how long, what you are doing right now:\n"
              "  read it first -- get_robot_state ALWAYS (counts, line and\n"
              "  loop state all live there), get_line_counts as well for the\n"
              "  feeder split, get_action_history for what you dispatched.\n"
              "  Your impression of your own past is not evidence.\n"
            + "- Replies may be read aloud: keep them to one or two short, "
            "natural sentences."
        )
    return (
        line_brief + capabilities + "\n\n" + rules
        + "- Keep the final text response short -- one sentence."
    )


def setup_omnilink_relay(bridge: ArmBridge, http_port: int = 8765) -> Optional[Any]:
    if OmniLinkRelay is None:
        return None
    local_ok = (
        OllamaRelay is not None
        and _os.environ.get("OMNISIM_OLLAMA", "1").strip() not in ("0", "false", "no")
        and ollama_available()
    )
    explicit_cloud = bool(_os.environ.get("OMNILINK_ENGINE", "").strip())

    # ── Hybrid: OMNI_KEY + local Ollama → free local inference, platform
    # memory/profile/telemetry/fallback on top. The best of both.
    if omnilink_enabled() and local_ok and not explicit_cloud:
        try:
            agent_name = profile_sync.agent_name_for(bridge.robot_id)
            tools = build_arm_tools(bridge)
            main_task = build_arm_main_task(bridge)
            relay = OllamaRelay(
                agent_name=agent_name,
                main_task=main_task,
                tools=tools,
                omni_key=get_omni_key(),
            )
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
            print(f"[omnilink_arm_bridge] HYBRID relay ON (local {relay.model} + OmniLink sync)")
            return relay
        except Exception as e:
            print(f"[omnilink_arm_bridge] hybrid relay setup failed: {e}")

    # ── Zero-account free tier: local Ollama only.
    if not omnilink_enabled():
        if local_ok:
            try:
                relay = OllamaRelay(
                    agent_name=profile_sync.agent_name_for(bridge.robot_id),
                    main_task=build_arm_main_task(bridge),
                    tools=build_arm_tools(bridge),
                )
                print(f"[omnilink_arm_bridge] local Ollama relay ON (model={relay.model})")
                return relay
            except Exception as e:
                print(f"[omnilink_arm_bridge] Ollama relay setup failed: {e}")
        return None
    try:
        agent_name = profile_sync.agent_name_for(bridge.robot_id)
        tools = build_arm_tools(bridge)
        main_task = build_arm_main_task(bridge)
        relay = OmniLinkRelay(
            omni_key=get_omni_key(),
            agent_name=agent_name,
            main_task=main_task,
            tools=tools,
        )
        # Profile sync: push an agent profile to the platform so the
        # operator can pick this robot in the omnilink-agents.com web
        # UI and chat to it from there. The platform's UI POSTs
        # structured tool calls back to toolCallbackUrl, which we
        # serve at /tool below.
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
        print(f"[omnilink_arm_bridge] OmniLink relay ON (agent='{agent_name}')")
        return relay
    except Exception as e:
        import traceback
        print(f"[omnilink_arm_bridge] !! OmniLink relay setup FAILED "
              f"({type(e).__name__}: {e}) -- falling back to the local regex "
              f"intent router. Chat will work but there is NO LLM in the loop.",
              flush=True)
        traceback.print_exc()
        return None


# ── Robot window plumbing ────────────────────────────────────────────

def build_window_config(bridge: ArmBridge, relay: Any) -> Dict[str, Any]:
    """Shared UI config used by BOTH the docked robot window (configure
    handshake) and the GET /chat_config endpoint the browser chat reads,
    so the two surfaces stay in lockstep (same persona, suggestions, etc.)."""
    # A persona config ships its own chat-flavoured suggestions;
    # otherwise build the generic command set from the arm's capabilities.
    suggestions = list(bridge.cfg.get("suggestions") or [])
    if not suggestions:
        suggestions = ["home", "wave hello", "joint 1 to 0.5", "stop"]
        if bridge.cfg.get("ik"):
            suggestions.insert(2, "move to 0.4 0.2 0.4")
        if bridge.effector is not None:
            suggestions.append("open the gripper")
            suggestions.append("grasp")
            if bridge.effector.capabilities().get("has_width_control"):
                suggestions.append("open gripper to 4 cm")
    if bridge.learn is not None and not any("learn" in s for s in suggestions):
        suggestions.append("learn to toss the cube into the bin")
    if relay is None:
        agent_label = "local intent (regex)"
    else:
        engine = getattr(relay, "engine", None) or _os.environ.get("OMNILINK_ENGINE", "g1-engine")
        agent_label = (
            f"local Ollama ({engine.split(':', 1)[1]})" if str(engine).startswith("ollama:")
            else f"OmniLink relay ({engine})"
        )
    return {
        "robot": bridge.cfg["model"],
        "robot_class": "arm",
        "agent": agent_label,
        "suggestions": suggestions,
        "home": list(bridge.home_pose),
        # Persona UI hints (the window shows a name + tagline + avatar when
        # display_name is present; otherwise the generic robot console).
        "display_name": bridge.cfg.get("display_name"),
        "tagline": bridge.cfg.get("tagline"),
        "greeting": bridge.cfg.get("greeting"),
        # Voice in the docked panel needs the relay (STT via audio_in). The
        # browser chat uses the Web Speech API and ignores this flag.
        "voice": relay is not None,
        "has_ik": bool(bridge.cfg.get("ik")),
        "has_gripper": bridge.effector is not None,
        "hardware": bridge.hw_status(),
    }


def push_configure(bridge: ArmBridge, relay: Any) -> None:
    cfg = build_window_config(bridge, relay)
    bridge.queue_window("configure:" + json.dumps(cfg))
    bridge.queue_window("status:connected")
    # A warm opening line so the panel greets the operator instead of
    # sitting empty. Text-only — TTS fires on real relay replies.
    greeting = cfg.get("greeting")
    if greeting:
        bridge.queue_window("agent:" + greeting)
    bridge.window_configured = True


def _on_relay_event(bridge: ArmBridge, kind: str, payload: Dict[str, Any]) -> None:
    if kind == "status":
        bridge.queue_window(f"status:{payload.get('state', 'idle')}")
    elif kind == "tool":
        name = payload.get("name", "?")
        status = payload.get("status", "ok")
        summary = payload.get("summary", "")
        bridge.queue_window(f"tool:{name}:{status}:{summary}")
    elif kind == "agent":
        bridge.queue_window("agent:" + str(payload.get("text", "")))
    elif kind == "usage":
        # Per-turn token/credit usage delta from the platform's rollup.
        # The robot-window plugin renders this as a footer line so the
        # operator can see what the last prompt cost.
        bridge.queue_window("usage:" + json.dumps(payload, default=str))
    elif kind == "audio_out":
        # Synthesized agent voice; deliver the base64-MP3 to the chat
        # panel as `audio_out:<json>` so the JS Audio API can play it.
        bridge.queue_window("audio_out:" + json.dumps(payload, default=str))
    elif kind == "error":
        bridge.queue_window("error:" + str(payload.get("text", "")))


def handle_wwi_message(
    bridge: ArmBridge,
    router: IntentRouter,
    relay: Any,
    msg: str,
) -> None:
    if not msg:
        return
    if msg.startswith("configure"):
        push_configure(bridge, relay)
        return
    if msg.startswith("open_chat"):
        # Pop the full-page chat into the operator's default browser. The
        # controller is a normal Python process, so webbrowser.open works;
        # each robot's bridge has its own port -> its own tab -> own agent.
        import webbrowser
        url = "http://127.0.0.1:%d/chat" % getattr(bridge, "http_port", 8765)
        try:
            webbrowser.open(url, new=2)
            bridge.queue_window("system:Opening chat in your browser — " + url)
        except Exception as e:
            bridge.queue_window("error:could not open browser: %r" % e)
        return
    if msg.startswith("stop"):
        bridge.note_external_command("chat:stop")
        # THIS RUNS ON THE SIM THREAD (main() pumps wwi messages), so
        # act_stop cannot spend a settling window here -- it returns
        # stationary: None with the reason rather than claiming "halted".
        # The HOLD, which is what actually keeps the arm still, is armed
        # either way.
        res = bridge.act_stop()
        hold = res.get("idle_loop") or {}
        held = (f" Holding for {hold['hold_s']:.0f}s, then the pick loop "
                f"resumes on its own." if hold.get("present") else "")
        bridge.queue_window(
            "agent:Frozen at the current joint angles." + held
            + " (Rest not measured from this button — read the state panel.)")
        bridge.queue_window("tool:stop_robot:ok:frozen, rest unmeasured")
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
        # The chat panel captured a mic clip via MediaRecorder and
        # base64-encoded the webm blob. Decode -> STT via the relay
        # (or surface an error if the relay isn't attached) -> route
        # the transcribed text back as a prompt.
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
        # Run STT off the wwi loop so we don't stall the simulation tick.
        def _stt_worker():
            text = relay.transcribe(audio, mime_type=mime)
            if not text:
                bridge.queue_window("error:could not transcribe audio")
                bridge.queue_window("status:idle")
                return
            # Surface the transcript so the operator sees what was heard.
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
    # Unknown -- echo back for debugging.
    bridge.queue_window("system:Unknown window message: " + msg[:200])


# ── Main loop ────────────────────────────────────────────────────────

def main() -> int:
    args = _parse_args()
    cfg = get_config(args.robot)
    # Applied to a COPY: get_config hands back the shared module-level dict, so
    # mutating it would leak this world's drop point into every other arm built
    # from the same config in the same process.
    if args.drop_zone:
        try:
            _dz = [float(v) for v in str(args.drop_zone).split(",")]
            if len(_dz) != 3:
                raise ValueError("need exactly 3 comma-separated numbers")
            cfg = dict(cfg)
            cfg["drop_zone"] = _dz
        except Exception as _e:
            raise SystemExit("[omnilink_arm_bridge] bad --drop-zone %r: %s"
                             % (args.drop_zone, _e))
    robot = Supervisor()
    # --name overrides the agent id so that two arms with the same
    # underlying kinematic config (e.g. ur5e_left and ur5e_right when
    # several share a world) get distinct OmniLink profiles + Axis ids.
    robot_id = args.name or args.robot
    bridge = ArmBridge(robot, cfg, robot_id, gripper_id=args.gripper)
    bridge.http_port = args.port   # so open_chat builds the right /chat URL
    router = IntentRouter(bridge)

    # skill learning learn pipeline. The manager owns the learn_runner
    # subprocess + event stream; its callbacks run on the reader thread and
    # only touch thread-safe bridge surfaces (queue_window / the lock-guarded
    # skill registry), never the Robot API -- the step loop below stays the
    # sole driver of the simulator.
    def _learn_chat(kind: str, text: str) -> None:
        if kind == "tool_ok":
            name, _, summary = text.partition(":")
            bridge.queue_window(f"tool:{name}:ok:{summary}")
        elif kind == "tool_err":
            name, _, summary = text.partition(":")
            bridge.queue_window(f"tool:{name}:err:{summary}")
        elif kind == "system":
            bridge.queue_window("system:" + text)
        else:
            bridge.queue_window("agent:" + text)

    bridge.learn = LearnManager(on_chat=_learn_chat,
                                on_skill=bridge.register_learned_skill)

    # The relay's main task is built inside setup_omnilink_relay, and
    # bridge.line is not constructed until further down -- so the LINE MASTER
    # role has to be flagged here or the platform gets a prompt that never
    # mentions the production line this arm is the authority on.
    bridge.line_master = bool(args.idle_loop == "pick" and cfg.get("ik")
                              and (args.idle_drop or "").strip())
    relay = setup_omnilink_relay(bridge, http_port=args.port)

    # Optional: attach a real arm (or its offline-sim VM) through a hardware
    # backend. Opt-in only (--hardware-backend / --hardware-ip, or a backend's
    # own environment). When attached, act_* forward commands to the hardware
    # and tick() mirrors its measured joints onto the simulated arm.
    attach_hardware(bridge, cfg, robot_id,
                    name=args.hardware_backend, ip=args.hardware_ip)

    # HTTP server runs on its own thread.
    start_http(bridge, router, args.port, relay)

    # Opt-in ambient idle loop (keeps the demo alive when nobody is
    # chatting; any operator command pauses it instantly).
    if args.idle_loop == "pick":
        if cfg.get("ik"):
            # LINE MASTER: auto-arms only in a conveyor-line world (DEF BOX_*
            # + DEF FILL_STOP present); elsewhere it self-disables and the
            # idle loop keeps its plain trolley-basket behaviour.
            bridge.line = WarehouseLine(
                bridge,
                trolley_defs=[d.strip() for d in (args.idle_drop or "").split(",")
                              if d.strip()])
            bridge.idle_loop = ArmIdleLoop(bridge, drop_def=args.idle_drop,
                                           resume_s=args.idle_resume_s)
            bridge.idle_loop.start()
            print("[omnilink_arm_bridge] idle loop 'pick' enabled "
                  "(pauses on any operator command; resumes after "
                  f"{args.idle_resume_s:.0f}s quiet)")
        else:
            print("[omnilink_arm_bridge] --idle-loop pick ignored: "
                  f"{args.robot} has no IK chain")

    print(f"[omnilink_arm_bridge] {cfg['model']} ready as id '{args.robot}' "
          f"({len(bridge.joint_names)} joints, "
          f"{'IK' if cfg.get('ik') else 'joint-space only'}, "
          f"gripper={bridge.effector.model if bridge.effector else 'none'}, "
          f"{'OmniLink' if relay else 'local'})")

    timestep = bridge.timestep
    while robot.step(timestep) != -1:
        sim_t = robot.getTime()
        if _TX_ENABLED:
            # Cache it for the transcript: a chat turn is stamped from the
            # main thread's clock, never by calling the Robot API from the
            # HTTP or relay-worker thread.
            bridge.last_sim_t = sim_t
        # Drain wwi inbox.
        while True:
            msg = robot.wwiReceiveText()
            if msg is None or msg == "":
                break
            try:
                handle_wwi_message(bridge, router, relay, msg)
            except Exception as e:
                bridge.queue_window(f"error:bridge_exception: {e!r}")
        # Drain outbox.
        with bridge.lock:
            outbox = bridge.window_outbox
            bridge.window_outbox = []
        for line in outbox:
            try:
                robot.wwiSendText(line)
            except Exception as e:
                print(f"[omnilink_arm_bridge] wwiSendText failed: {e}")
        # Advance motion plan.
        bridge.tick(sim_t)

    # World shutdown: kill the learn runner child cleanly, then the hardware.
    if bridge.learn is not None:
        bridge.learn.shutdown()
    if bridge.hw is not None:
        bridge.hw.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
