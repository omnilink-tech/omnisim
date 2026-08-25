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

"""What does OmniLink add, and is it more than generic LLM tool calling?

The question, framed honestly
-----------------------------
The warehouse_omnilink 5-stage production line is FULLY AUTONOMOUS
deterministic Python living inside the three bridge controllers. It needs no
LLM, no OMNI_KEY and no network. **No chat layer can make the line faster.**
The only two things an LLM chat layer can do in this suite are:

  1. add OPERATOR CAPABILITY  -- understand instructions the regex router
     cannot, and carry them out;
  2. COST THROUGHPUT          -- because any operator command instantly
     pauses that robot's idle loop.

So this harness measures exactly those two axes and nothing else. A lower
boxes/min under an interactive mode is the EXPECTED price of interacting with
a running line, not a defect. The local-Ollama condition is the control that
separates generic LLM tool-calling value from OmniLink-specific value. Read
BENCH_OMNILINK.md before quoting a number.

The one rule that makes this a measurement instead of a vibe
------------------------------------------------------------
**EVERY VERDICT COMES FROM MEASURED ROBOT STATE. NEVER FROM THE REPLY TEXT.**

For each prompt the harness snapshots pose / joints / `idle_loop.paused`
BEFORE, issues the prompt, polls state for a bounded window, and decides
pass/fail from the DELTA against an explicit per-prompt predicate. The reply
text is recorded as evidence and is never the verdict. A model that says
"driving forward one metre now" while the tug does not move scores 0 -- which
is the whole point, because that failure mode has been measured in this repo
(see docs/developer/tool-design-for-agents.md).

Two secondary, clearly-separated signals are also recorded and are NOT part of
the state score:
  * `text_check` -- for pure questions there is no robot-state consequence, so
    state-scoring alone cannot discriminate. The harness reads the GROUND TRUTH
    out of the bridge's own /state (never from the model) and checks whether
    the reply contains it. Reported under `answer_accuracy`, separately.
  * `latency_s`  -- wall time of the /prompt round trip.

What it does to the world
-------------------------
Unlike `measure_line.py` (GET-only, by design) this harness MUST issue POSTs --
that is the thing under test. It issues exactly three kinds:

    POST /prompt            the probe itself (identical text in every mode)
    POST /stop_robot        harness-owned SETUP, to park a robot before a
                            resume probe. Mode-independent, never scored.
    POST /resume_autonomy   harness-owned RESET between probes, so every probe
                            starts from the same precondition in every mode.
                            Mode-independent, never scored.

Nothing else. It writes exactly one file: `--out`.

Usage
-----
    # A control: no interaction at all (the throughput reference)
    python tests/benchmarks/warehouse/bench_omnilink.py --mode none \\
        --duration 900 --out results/none.json

    # B: offline regex router (start the world with nothing set)
    python tests/benchmarks/warehouse/bench_omnilink.py --mode offline \\
        --duration 900 --out results/offline.json

    # C: local Ollama (start the world with OMNI_KEY unset and Ollama live)
    python tests/benchmarks/warehouse/bench_omnilink.py --mode local \\
        --duration 900 --out results/local.json

    # D: OmniLink platform (start the world with OMNI_KEY set)
    python tests/benchmarks/warehouse/bench_omnilink.py --mode omnilink \\
        --duration 900 --out results/omnilink.json

    # side by side
    python tests/benchmarks/warehouse/bench_omnilink.py --compare \\
        results/none.json results/offline.json results/local.json \\
        results/omnilink.json

`--mode` is a LABEL you assert, not a switch. The chat mode is chosen when the
controllers START (env: OMNI_KEY / OMNISIM_OLLAMA / OMNILINK_ENGINE); this
harness cannot change it. It DOES probe `GET /usage` per bridge, classifies
which engine actually answered, and REFUSES to run when that contradicts
`--mode` (see the integrity gates below).

Integrity gates -- the measurement measuring itself
--------------------------------------------------
A 200 OK from a bridge port does NOT prove you are talking to the simulator
you think you are. `scripts/dev/headless_runner.py` stops the engine with
`proc.terminate()`, which on Windows is `TerminateProcess`: the engine never
tells its controllers to quit, so each bridge CONTROLLER is orphaned with its
main thread blocked forever in `robot.step()` on a dead IPC pipe -- while its
daemon HTTP thread KEEPS SERVING. `ThreadingHTTPServer` sets
`allow_reuse_address = 1`, and on Windows SO_REUSEADDR lets a NEW process bind
an ALREADY-BOUND 127.0.0.1 port. Measured here: two processes both LISTENING
on one loopback port, and the EARLIER binder answered all ten probe requests.
A fresh bridge can therefore be completely shadowed by last week's corpse, and
every number this harness prints would be about a robot that does not exist.

Three gates make that impossible rather than merely detectable afterwards:

  1. LIVENESS   `GET /state` twice ~1.2 s apart; `last_tick_at` MUST advance.
                It is stamped from the controller's tick loop, so a corpse
                whose Supervisor is dead reports a FROZEN value while still
                answering 200. Frozen -> exit 7, run refused.
  2. IDENTITY   The process behind each port is fingerprinted at preflight and
                re-verified at the end: OS listener PIDs (Windows, best
                effort) plus id/model, the sim clock and the bridges' own
                monotone session counters. A swap or a duplicate listener ->
                exit 8.
  3. ENGINE     `enabled: true` is necessary but NOT sufficient for
                `--mode omnilink`: a local OllamaRelay reports it too. The
                engine actually behind the relay is classified from
                `/usage.latest` after the first prompt. Positively not-cloud
                -> exit 9; indeterminate -> the result is MARKED
                `engine_verified: unverified` rather than claimed.

Exit codes
----------
    0  ran to completion (probe FAILURES are results, not errors -- still 0)
    2  a bridge did not answer preflight (or this is not the warehouse world)
    3  aborted mid-run (partial JSON still written)
    4  bad arguments
    6  --compare could not read an input file
    7  LIVENESS GATE: a bridge answered but its tick loop is frozen -- you are
       talking to an orphaned controller (or a paused simulation)
    8  IDENTITY GATE: the process behind a bridge port is ambiguous or changed
       during the run (duplicate listener / restarted counters / clock reset)
    9  ENGINE GATE: the engine that answered contradicts --mode
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import statistics
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

SCHEMA = "omnisim.warehouse.bench_omnilink/1"

# Exit codes. Named because three of them are new integrity gates and a
# caller branching on a bare integer literal cannot say which.
EXIT_OK = 0
EXIT_PREFLIGHT = 2
EXIT_ABORTED = 3
EXIT_ARGS = 4
EXIT_COMPARE_READ = 6
EXIT_LIVENESS = 7          # a bridge answered 200 with a FROZEN tick loop
EXIT_IDENTITY = 8          # the process behind a port is ambiguous or changed
EXIT_ENGINE = 9            # the engine that answered contradicts --mode

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

try:
    import measure_line as ml                                    # noqa: E402
except ImportError as e:                                  # pragma: no cover
    print(f"error: cannot import measure_line.py from {_HERE}: {e}",
          file=sys.stderr)
    raise

# Logical role -> which bridge. Remappable from the CLI so the suite text can
# stay fixed and auditable while the operator picks which tug to disturb.
ROLES = ("tug", "arm")


# ══════════════════════════════════════════════════════════════════════
# THE INTERVENTION SUITE.  Ordered, fixed, identical across modes.
#
# ONE module-level list on purpose: this is the experiment, and it has to be
# auditable and editable without reading the rest of the file. Difficulty
# rises down the list. `offline_expectation` is what the SOURCE of the regex
# router predicts (projects/samples/demos/controllers/omnilink_mobile_bridge
# /omnilink_mobile_bridge.py IntentRouter.dispatch, plus RESUME_RE / STATUS_RE
# in packages/omnisim-bridges/src/omnisim_bridges/intent_router.py) -- it is a
# PREDICTION recorded for falsification, never an input to any verdict.
# ══════════════════════════════════════════════════════════════════════

SUITE: List[Dict[str, Any]] = [

    # ── Tier 1: literal. The regex router documents this one. ──────────
    {
        "key": "t1_stop_literal",
        "tier": "1-literal",
        "target": "tug",
        "text": "stop",
        "predicate": "at_rest",
        "params": {"grace_s": 2.5, "rest_speed_m_s": 0.03,
                   "rest_disp_m": 0.08},
        "verify_s": 12.0,
        "setup": "resume",
        "why": "The floor. A single literal verb the offline router matches "
               "with \\b(stop|halt|freeze|brake)\\b. If a mode fails HERE the "
               "run is broken, not the mode.",
        "offline_expectation": "pass",
    },

    # ── Tier 2: parametric. Number + unit must survive into the tool. ──
    {
        "key": "t2_drive_1m",
        "tier": "2-parametric",
        "target": "tug",
        "text": "drive forward 1 meter",
        "predicate": "net_translation",
        "params": {"target_m": 1.0, "dist_tol_m": 0.15,
                   "yaw_tol_deg": 12.0},
        "verify_s": 30.0,
        "setup": "resume",
        "why": "One argument to extract and one unit to honour. The offline "
               "router has a regex for exactly this shape.",
        "offline_expectation": "pass",
    },

    # ── Tier 3: state query. Must NOT park a working robot. ───────────
    {
        "key": "t3_where_are_you",
        "tier": "3-query",
        "target": "tug",
        "text": "where are you?",
        "predicate": "not_parked",
        "params": {},
        "verify_s": 12.0,
        "setup": "resume",
        "text_check": {"truth": "tug_x_rounded", "kind": "number",
                       "tol": 0.75},
        "why": "A question is not a command. The bridges roll the idle-loop "
               "pause back when a turn used only read-only tools, so the "
               "MEASURED consequence of a good answer is: the robot is still "
               "working afterwards.",
        "offline_expectation": "pass",
    },

    # ── Tier 4: compositional + arithmetic. Two ordered sub-goals. ────
    {
        "key": "t4_compose_back_then_turn",
        "tier": "4-compositional",
        "target": "tug",
        "text": "back up half a metre, then turn left 45 degrees",
        "predicate": "translate_then_rotate",
        "params": {"target_m": -0.5, "target_deg": 45.0,
                   "dist_tol_m": 0.18, "yaw_tol_deg": 12.0},
        "verify_s": 45.0,
        "setup": "resume",
        "why": "THE FIRST REAL DISCRIMINATOR. Two goals in one sentence, one "
               "of them spelled ('half a metre' -> 0.5, no digits to match), "
               "and an order between them. Reading the offline router's "
               "source: its turn-regex fires on 'turn left 45 degrees' and "
               "returns immediately, so the back-up never happens and only "
               "the rotation sub-goal can pass.",
        "offline_expectation": "partial (rotation only)",
    },

    # ── Tier 5: world-state reasoning. ────────────────────────────────
    {
        "key": "t5_count_parked_carts",
        "tier": "5-world-state",
        "target": "tug",
        "text": "how many carts are parked in the row right now?",
        "predicate": "not_parked",
        "params": {},
        "verify_s": 15.0,
        "setup": "resume",
        "text_check": {"truth": "park_row_count", "kind": "count"},
        "why": "Requires reading live world state and reporting a number. "
               "State-wise all a good answer must do is leave the robot "
               "working -- so the discriminating signal here is the SECONDARY "
               "answer_accuracy check, whose ground truth is read out of the "
               "tug's own /state.idle_loop.park_row_count, never from the "
               "model. The offline router has no matching intent and falls "
               "through to its 'I don't recognise that' reply.",
        "offline_expectation": "state pass / answer fail",
    },

    # ── Tier 6: RESUME. The discriminator this whole harness exists for.
    {
        "key": "t6_resume_literal",
        "tier": "6-resume",
        "target": "tug",
        "text": "carry on, back to work",
        "predicate": "resumed",
        "params": {},
        "verify_s": None,            # -> resume_s + margin, see resolve_verify_s
        "setup": "pause",
        "requires_paused": True,
        "why": "Saying 'resuming now' is NOT resuming: the robot stays parked "
               "until the ~60 s quiet timer expires unless something actually "
               "calls resume_autonomy. Measured on idle_loop.paused, and the "
               "cause (agent tool vs the timer running out) is attributed "
               "explicitly. NOTE: this phrasing matches RESUME_RE in "
               "omnisim_bridges.intent_router, so the offline router IS "
               "expected to pass it -- see t7 for the version that is not "
               "handed to it.",
        "offline_expectation": "pass (RESUME_RE matches 'carry on')",
    },
    {
        "key": "t7_resume_oblique",
        "tier": "6-resume",
        "target": "tug",
        "text": ("I'm finished with you for now -- the line needs you more "
                 "than I do, so go finish the job you were on."),
        "predicate": "resumed",
        "params": {},
        "verify_s": None,
        "setup": "pause",
        "requires_paused": True,
        "why": "THE HEADLINE DISCRIMINATOR. Same intent as t6, phrased so "
               "that RESUME_RE does not match it (no resume / carry on / keep "
               "going / continue / as you were / proceed / back to work / "
               "restart / unpause, and no bare 'back' for the reverse-drive "
               "rule to eat). A regex router cannot get the robot moving "
               "again; an LLM with resume_autonomy in its toolbox can.",
        "offline_expectation": "fail (resumes only when the timer expires)",
    },

    # ── The arm. Same ladder, compressed: it is the line master, so
    #    parking it is the single most expensive intervention available.
    {
        "key": "a1_arm_stop_literal",
        "tier": "1-literal",
        "target": "arm",
        "text": "stop",
        "predicate": "joints_at_rest",
        "params": {"grace_s": 3.0, "rest_rad": 0.03},
        "verify_s": 14.0,
        "setup": "resume",
        "why": "Literal floor for the arm; verdict from the joint vector `q`, "
               "not from the reply.",
        "offline_expectation": "pass",
    },
    {
        "key": "a2_arm_shipped_count",
        "tier": "5-world-state",
        "target": "arm",
        "text": "how many boxes have you shipped so far on this shift?",
        "predicate": "not_parked",
        "params": {},
        "verify_s": 15.0,
        "setup": "resume",
        "text_check": {"truth": "shipped_total", "kind": "count"},
        "why": "The line master publishes line.shipped_total, so there is a "
               "hard ground truth for the answer check. State-wise: asking "
               "must not stall the line.",
        "offline_expectation": "state pass / answer uncertain",
    },
    {
        "key": "a3_arm_resume_oblique",
        "tier": "6-resume",
        "target": "arm",
        "text": ("that's everything I needed -- the belt is backing up, so "
                 "get yourself kitting again."),
        "predicate": "resumed",
        "params": {},
        "verify_s": None,
        "setup": "pause",
        "requires_paused": True,
        "why": "The costliest pause on the line is the arm's. Same oblique "
               "resume as t7, aimed at the line master.",
        "offline_expectation": "fail (resumes only when the timer expires)",
    },
]


# ══════════════════════════════════════════════════════════════════════
# Secrets.  The key is read from the environment and never leaves it.
# ══════════════════════════════════════════════════════════════════════

_SECRET_PATTERNS = [
    re.compile(r"olink_[A-Za-z0-9_\-]{4,}"),
    re.compile(r"sk-[A-Za-z0-9_\-]{8,}"),
    re.compile(r"(?i)bearer\s+[A-Za-z0-9_\-\.]{8,}"),
    re.compile(r"AIza[0-9A-Za-z_\-]{20,}"),
]


class Redactor:
    """Scrubs secrets out of everything on its way to disk or the terminal.

    Two layers, because either alone is insufficient: the LITERAL values of
    the sensitive env vars (so the exact key can never survive), and generic
    token SHAPES (so a key echoed back by a bridge, or one belonging to a
    provider we do not know the var name of, is caught too).
    """

    ENV_VARS = ("OMNI_KEY", "OMNISIM_BRIDGE_TOKEN", "OPENAI_API_KEY",
                "ANTHROPIC_API_KEY", "GOOGLE_API_KEY", "GEMINI_API_KEY",
                "XAI_API_KEY", "OMNILINK_TOKEN")

    def __init__(self, extra: Sequence[str] = ()) -> None:
        vals = [os.environ.get(v, "") for v in self.ENV_VARS]
        vals.extend(extra)
        # Longest first so a key that contains a shorter one is fully masked.
        self.literals = sorted({v for v in vals if v and len(v) >= 6},
                               key=len, reverse=True)

    def __call__(self, obj: Any) -> Any:
        if isinstance(obj, str):
            s = obj
            for lit in self.literals:
                s = s.replace(lit, "[REDACTED]")
            for pat in _SECRET_PATTERNS:
                s = pat.sub("[REDACTED]", s)
            return s
        if isinstance(obj, dict):
            return {k: self(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [self(v) for v in obj]
        return obj


# ══════════════════════════════════════════════════════════════════════
# Pure helpers + PREDICATES.  No I/O, no globals, no clock reads.
# Unit-tested in test_bench_omnilink.py -- including the adversarial case
# where the reply claims success and the pose did not change.
# ══════════════════════════════════════════════════════════════════════

def pose_of(state: Optional[dict]) -> Optional[Tuple[float, float, float]]:
    """(x, y, yaw) from a mobile-bridge /state dict, or None."""
    if not isinstance(state, dict):
        return None
    x, y, yaw = state.get("x"), state.get("y"), state.get("yaw")
    for v in (x, y, yaw):
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            return None
        if not math.isfinite(float(v)):
            return None
    return float(x), float(y), float(yaw)


def joints_of(state: Optional[dict]) -> Optional[List[float]]:
    """The arm's joint vector `q`, or None."""
    if not isinstance(state, dict):
        return None
    q = state.get("q")
    if not isinstance(q, (list, tuple)) or not q:
        return None
    out = []
    for v in q:
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            return None
        if not math.isfinite(float(v)):
            return None
        out.append(float(v))
    return out


def paused_of(state: Optional[dict]) -> Optional[bool]:
    """`idle_loop.paused`, or None when the robot has no idle loop."""
    if not isinstance(state, dict):
        return None
    loop = state.get("idle_loop")
    if not isinstance(loop, dict) or "paused" not in loop:
        return None
    return bool(loop.get("paused"))


def frame_delta(before: Tuple[float, float, float],
                after: Tuple[float, float, float]) -> Dict[str, float]:
    """Displacement resolved in the robot's INITIAL body frame.

    `forward_m` is what "drive forward 1 m" is actually asking for -- the
    straight-line distance is not, because a tug that drove 1 m sideways
    would satisfy it. `bearing_err_deg` is the angle between the displacement
    and the initial heading, which is what tells forward from backward when
    the magnitude alone cannot.
    """
    dx = after[0] - before[0]
    dy = after[1] - before[1]
    c, s = math.cos(before[2]), math.sin(before[2])
    forward = dx * c + dy * s
    lateral = -dx * s + dy * c
    dist = math.hypot(dx, dy)
    return {
        "dx_m": dx, "dy_m": dy,
        "dist_m": dist,
        "forward_m": forward,
        "lateral_m": lateral,
        "dyaw_deg": math.degrees(ml.wrap_pi(after[2] - before[2])),
        "bearing_err_deg": (math.degrees(abs(math.atan2(lateral, forward)))
                            if dist > 1e-6 else 0.0),
    }


def series_speeds(series: Sequence[Dict[str, Any]]) -> List[float]:
    """Per-step linear speed (m/s) derived from POSES, never from the
    bridge's reported v_linear -- that is the COMMAND, which is exactly the
    number that can look right while the robot does not move."""
    out: List[float] = []
    for i in range(1, len(series)):
        p0 = pose_of(series[i - 1].get("state"))
        p1 = pose_of(series[i].get("state"))
        dt = float(series[i]["t"]) - float(series[i - 1]["t"])
        if p0 is None or p1 is None or dt <= 0:
            continue
        out.append(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / dt)
    return out


def after_grace(series: Sequence[Dict[str, Any]], grace_s: float
                ) -> List[Dict[str, Any]]:
    return [s for s in series if float(s["t"]) >= grace_s]


def _res(verdict: str, reason: str, measured: Dict[str, Any],
         subgoals: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
    return {"verdict": verdict, "reason": reason, "measured": measured,
            "subgoals": subgoals or []}


def _sub(name: str, ok: Optional[bool], measured: Any, expected: Any,
         tol: Any, info: bool = False) -> Dict[str, Any]:
    return {"name": name, "ok": ok, "measured": measured,
            "expected": expected, "tol": tol, "informational": info}


# ── predicate: at_rest ────────────────────────────────────────────────

def pred_at_rest(probe: Dict[str, Any], ev: Dict[str, Any]) -> Dict[str, Any]:
    """"stop" succeeded iff the base is DEMONSTRABLY not moving afterwards.

    Judged on pose deltas over the window after a grace period (a robot
    already in motion needs a moment to brake), never on `mode` or `v_linear`
    -- both are commanded values that a bridge can report while the wheels
    keep turning.
    """
    p = probe.get("params") or {}
    grace = float(p.get("grace_s", 2.5))
    eps_v = float(p.get("rest_speed_m_s", 0.03))
    eps_d = float(p.get("rest_disp_m", 0.08))
    tail = after_grace(ev.get("series") or [], grace)
    if len(tail) < 2:
        return _res("inconclusive",
                    f"fewer than 2 pose samples after the {grace:.1f}s grace "
                    f"period; window too short or the bridge stopped answering",
                    {"samples_after_grace": len(tail)})
    speeds = series_speeds(tail)
    p0 = pose_of(tail[0].get("state"))
    p1 = pose_of(tail[-1].get("state"))
    if p0 is None or p1 is None or not speeds:
        return _res("inconclusive", "no usable pose samples", {})
    drift = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    vmax = max(speeds)
    subs = [
        _sub("max_speed", vmax <= eps_v, round(vmax, 4), 0.0, eps_v),
        _sub("net_drift", drift <= eps_d, round(drift, 4), 0.0, eps_d),
    ]
    measured = {
        "max_speed_m_s": round(vmax, 4),
        "drift_after_grace_m": round(drift, 4),
        "pre_speed_m_s": ev.get("pre_speed_m_s"),
        # A "stop" issued to an already-stationary robot proves nothing. Not a
        # failure -- but it must not be quietly counted as a success either.
        "trivially_satisfied": (ev.get("pre_speed_m_s") is not None
                                and float(ev["pre_speed_m_s"]) < eps_v),
        "samples_after_grace": len(tail),
    }
    ok = all(s["ok"] for s in subs)
    return _res("pass" if ok else "fail",
                "at rest after the command" if ok else
                f"still moving: max {vmax:.3f} m/s, drift {drift:.3f} m",
                measured, subs)


# ── predicate: net_translation ────────────────────────────────────────

def pred_net_translation(probe: Dict[str, Any], ev: Dict[str, Any]
                         ) -> Dict[str, Any]:
    """"drive forward 1 m" succeeded iff the base moved 1 m FORWARD.

    Signed along the initial heading, so 1 m backwards fails and 1 m sideways
    fails. Heading is a second sub-goal: a tug that arced a metre round a
    corner did not do what was asked.
    """
    p = probe.get("params") or {}
    target = float(p.get("target_m", 1.0))
    dtol = float(p.get("dist_tol_m", 0.15))
    ytol = float(p.get("yaw_tol_deg", 12.0))
    before = pose_of(ev.get("before"))
    after = pose_of(ev.get("after"))
    if before is None or after is None:
        return _res("inconclusive", "missing pose snapshot", {})
    d = frame_delta(before, after)
    err = d["forward_m"] - target
    subs = [
        _sub("forward_distance", abs(err) <= dtol,
             round(d["forward_m"], 4), target, dtol),
        _sub("heading_held", abs(d["dyaw_deg"]) <= ytol,
             round(d["dyaw_deg"], 2), 0.0, ytol),
        _sub("lateral_drift", None, round(d["lateral_m"], 4), 0.0, None,
             info=True),
    ]
    ok = all(s["ok"] for s in subs if s["ok"] is not None)
    measured = dict(d)
    measured["error_m"] = round(err, 4)
    measured["commanded_m"] = target
    for k in ("dx_m", "dy_m", "dist_m", "forward_m", "lateral_m",
              "dyaw_deg", "bearing_err_deg"):
        measured[k] = round(measured[k], 4)
    return _res("pass" if ok else "fail",
                f"moved {d['forward_m']:+.3f} m forward "
                f"(asked {target:+.2f} m, error {err:+.3f} m)",
                measured, subs)


# ── predicate: translate_then_rotate ──────────────────────────────────

def pred_translate_then_rotate(probe: Dict[str, Any], ev: Dict[str, Any]
                               ) -> Dict[str, Any]:
    """Both halves of a compound instruction, scored independently.

    Partial credit is RECORDED (each sub-goal carries its own ok) but the
    verdict is binary: a robot that did half of what it was told did not do
    what it was told. The inferred ORDER is reported as evidence only --
    displacement resolved in the initial frame points backwards along the
    old heading if the robot reversed first, and along the NEW heading if it
    turned first -- because scoring order would over-constrain a request whose
    end state is what the operator actually cares about.
    """
    p = probe.get("params") or {}
    target_m = float(p.get("target_m", -0.5))
    target_deg = float(p.get("target_deg", 45.0))
    dtol = float(p.get("dist_tol_m", 0.18))
    ytol = float(p.get("yaw_tol_deg", 12.0))
    before = pose_of(ev.get("before"))
    after = pose_of(ev.get("after"))
    if before is None or after is None:
        return _res("inconclusive", "missing pose snapshot", {})
    d = frame_delta(before, after)
    dist_err = d["dist_m"] - abs(target_m)
    yaw_err = ml.wrap_pi(math.radians(d["dyaw_deg"] - target_deg))
    yaw_err_deg = math.degrees(yaw_err)

    # Order evidence. If the robot translated first, the displacement lies
    # along the OLD heading; if it turned first, along the NEW one.
    want = math.pi if target_m < 0 else 0.0
    bearing = math.atan2(d["dy_m"], d["dx_m"]) if d["dist_m"] > 1e-6 else None
    if bearing is None:
        order = "no-displacement"
        e_move_first = e_turn_first = None
    else:
        e_move_first = abs(math.degrees(ml.wrap_pi(bearing - (before[2] + want))))
        e_turn_first = abs(math.degrees(ml.wrap_pi(bearing - (after[2] + want))))
        order = ("translate_then_rotate" if e_move_first <= e_turn_first
                 else "rotate_then_translate")

    subs = [
        _sub("translation", abs(dist_err) <= dtol,
             round(d["dist_m"], 4), abs(target_m), dtol),
        _sub("rotation", abs(yaw_err_deg) <= ytol,
             round(d["dyaw_deg"], 2), target_deg, ytol),
        _sub("inferred_order", None, order, "translate_then_rotate", None,
             info=True),
    ]
    ok = all(s["ok"] for s in subs if s["ok"] is not None)
    n_ok = sum(1 for s in subs if s["ok"] is True)
    n_scored = sum(1 for s in subs if s["ok"] is not None)
    measured = {
        "dist_m": round(d["dist_m"], 4),
        "forward_m": round(d["forward_m"], 4),
        "dyaw_deg": round(d["dyaw_deg"], 2),
        "commanded_m": target_m,
        "commanded_deg": target_deg,
        "dist_error_m": round(dist_err, 4),
        "yaw_error_deg": round(yaw_err_deg, 2),
        "subgoals_met": f"{n_ok}/{n_scored}",
        "inferred_order": order,
        "order_bearing_err_move_first_deg": (round(e_move_first, 1)
                                             if e_move_first is not None else None),
        "order_bearing_err_turn_first_deg": (round(e_turn_first, 1)
                                             if e_turn_first is not None else None),
    }
    return _res("pass" if ok else "fail",
                f"{n_ok}/{n_scored} sub-goals met "
                f"(moved {d['dist_m']:.3f} m, turned {d['dyaw_deg']:+.1f} deg)",
                measured, subs)


# ── predicate: not_parked ─────────────────────────────────────────────

def pred_not_parked(probe: Dict[str, Any], ev: Dict[str, Any]
                    ) -> Dict[str, Any]:
    """A QUESTION must not park a working robot.

    Deliberately NOT "the robot did not move": the idle loop is supposed to
    keep driving through a question, so demanding stillness would score the
    correct behaviour as a failure. The measured consequence of a well-handled
    question is that `idle_loop.paused` is still False afterwards -- the
    bridges implement this by rolling the pause back when a chat turn used
    only read-only tools.

    Precondition: the robot must be RUNNING before the question. If a previous
    probe left it parked there is nothing to preserve, and the verdict is
    inconclusive rather than a free pass.
    """
    before_p = paused_of(ev.get("before"))
    after_p = paused_of(ev.get("after"))
    if before_p is None or after_p is None:
        return _res("inconclusive",
                    "this robot publishes no idle_loop.paused, so 'did the "
                    "question park it' is unanswerable", {})
    if before_p:
        return _res("inconclusive",
                    "robot was ALREADY paused before the question -- nothing "
                    "to preserve; check the inter-probe reset",
                    {"paused_before": True, "paused_after": after_p})
    subs = [_sub("still_running", not after_p, after_p, False, None)]
    measured = {"paused_before": before_p, "paused_after": after_p}
    bp, ap = pose_of(ev.get("before")), pose_of(ev.get("after"))
    if bp and ap:
        measured["moved_m"] = round(math.hypot(ap[0] - bp[0], ap[1] - bp[1]), 4)
    ok = not after_p
    return _res("pass" if ok else "fail",
                "the robot was still working after the question" if ok else
                "asking a question PARKED the robot -- it was running before "
                "and paused after",
                measured, subs)


# ── predicate: joints_at_rest ─────────────────────────────────────────

def pred_joints_at_rest(probe: Dict[str, Any], ev: Dict[str, Any]
                        ) -> Dict[str, Any]:
    """The arm's version of at_rest, on the joint vector."""
    p = probe.get("params") or {}
    grace = float(p.get("grace_s", 3.0))
    tol = float(p.get("rest_rad", 0.03))
    tail = after_grace(ev.get("series") or [], grace)
    qs = [joints_of(s.get("state")) for s in tail]
    qs = [q for q in qs if q]
    if len(qs) < 2:
        return _res("inconclusive",
                    f"fewer than 2 joint samples after the {grace:.1f}s grace "
                    f"period", {"samples_after_grace": len(qs)})
    n = min(len(q) for q in qs)
    spread = max(max(abs(q[i] - qs[0][i]) for i in range(n)) for q in qs)
    step = 0.0
    for a, b in zip(qs, qs[1:]):
        step = max(step, max(abs(b[i] - a[i]) for i in range(n)))
    subs = [_sub("joint_spread_rad", spread <= tol, round(spread, 5), 0.0, tol)]
    ok = spread <= tol
    return _res("pass" if ok else "fail",
                "joints held still after the command" if ok else
                f"joints still moving: {spread:.4f} rad spread",
                {"joint_spread_rad": round(spread, 5),
                 "max_step_rad": round(step, 5),
                 "samples_after_grace": len(qs),
                 "pre_joint_step_rad": ev.get("pre_joint_step_rad")},
                subs)


# ── predicate: resumed  (THE DISCRIMINATOR) ───────────────────────────

def classify_resume(t_resume_s: Optional[float], latency_s: float,
                    resume_s: float, t_arm_before_send_s: float,
                    tool_grace_s: float = 5.0,
                    timer_tol_s: float = 8.0) -> Dict[str, Any]:
    """Did the AGENT resume the robot, or did the ~60 s timer just expire?

    This is the single number the whole harness exists to produce, so the
    attribution is spelled out rather than assumed:

      * The idle loop un-pauses on its own `resume_s` after the last
        pause-arming command. Two candidates for "the last one": the harness's
        setup /stop_robot (which fired `t_arm_before_send_s` before the
        prompt), and the prompt itself (POST /prompt arms the pause on its way
        in, unless the turn was a pure question and got rolled back). The
        EARLIER of the two expiries is the first moment the timer could
        explain an un-pause, so that is the boundary used.
      * A tool call happens BEFORE the reply comes back. So an un-pause seen
        at or before the reply (plus a small grace for poll quantisation) was
        caused by something the agent called -- not by a clock.
      * Everything in between is `unclear` and is NOT scored as a success.
      * If the model was so slow that its reply lands after the timer would
        have fired anyway, the two causes are not separable at all. That is
        reported as `unseparable`, and the probe is inconclusive. Silently
        crediting the agent there would be the exact failure this harness is
        built to prevent.

    All times are seconds measured from the moment the prompt was SENT.
    """
    timer_due_s = min(resume_s, resume_s - t_arm_before_send_s)
    tool_bound_s = latency_s + tool_grace_s
    timer_bound_s = timer_due_s - timer_tol_s
    out = {
        "resume_s_assumed": resume_s,
        "timer_due_s": round(timer_due_s, 2),
        "tool_bound_s": round(tool_bound_s, 2),
        "timer_bound_s": round(timer_bound_s, 2),
        "reply_latency_s": round(latency_s, 3),
        "time_to_resume_s": (None if t_resume_s is None
                             else round(t_resume_s, 3)),
        "time_to_resume_after_reply_s": (None if t_resume_s is None
                                         else round(t_resume_s - latency_s, 3)),
    }
    if tool_bound_s >= timer_bound_s:
        out["cause"] = "unseparable"
        out["detail"] = (f"the reply took {latency_s:.1f}s, which is not "
                         f"safely inside the {timer_due_s:.1f}s auto-resume "
                         f"timer; a tool call and the timer cannot be told "
                         f"apart on this turn")
        return out
    if t_resume_s is None:
        out["cause"] = "none"
        out["detail"] = ("the robot never un-paused inside the observation "
                         "window")
        return out
    if t_resume_s <= tool_bound_s:
        out["cause"] = "tool"
        out["detail"] = (f"un-paused {t_resume_s:.1f}s after the prompt was "
                         f"sent, at or before the reply returned "
                         f"({latency_s:.1f}s) -- something CALLED "
                         f"resume_autonomy")
    elif t_resume_s >= timer_bound_s:
        out["cause"] = "auto_timer"
        out["detail"] = (f"un-paused {t_resume_s:.1f}s after the prompt, at "
                         f"the ~{timer_due_s:.0f}s quiet-window expiry -- the "
                         f"TIMER released it, the agent did not")
    else:
        out["cause"] = "unclear"
        out["detail"] = (f"un-paused {t_resume_s:.1f}s after the prompt: too "
                         f"late to pin on the reply ({tool_bound_s:.1f}s "
                         f"bound), too early for the timer "
                         f"({timer_bound_s:.1f}s bound)")
    return out


def pred_resumed(probe: Dict[str, Any], ev: Dict[str, Any]) -> Dict[str, Any]:
    """PASS only when the AGENT put the robot back to work.

    "Resuming now" with the robot still parked is a FAIL. So is waiting out
    the timer -- the robot did come back, but nothing the agent did caused it.
    """
    before_p = paused_of(ev.get("before"))
    if before_p is None:
        return _res("inconclusive",
                    "this robot publishes no idle_loop.paused", {})
    if not before_p:
        return _res("inconclusive",
                    "PRECONDITION NOT MET: the robot was not paused when the "
                    "resume prompt was sent, so there was nothing to resume",
                    {"paused_before": False})

    t_resume: Optional[float] = None
    for s in ev.get("series") or []:
        if paused_of(s.get("state")) is False:
            t_resume = float(s["t"])
            break

    cls = classify_resume(
        t_resume_s=t_resume,
        latency_s=float(ev.get("latency_s") or 0.0),
        resume_s=float(ev.get("resume_s") or 60.0),
        t_arm_before_send_s=float(ev.get("t_arm_before_send_s") or 0.0),
        tool_grace_s=float(ev.get("tool_grace_s") or 5.0),
        timer_tol_s=float(ev.get("timer_tol_s") or 8.0),
    )
    cause = cls["cause"]
    subs = [_sub("resumed_at_all", t_resume is not None,
                 cls["time_to_resume_s"], "not None", None),
            _sub("caused_by_agent", cause == "tool", cause, "tool", None)]
    measured = dict(cls)
    measured["paused_before"] = True
    if cause == "unseparable":
        return _res("inconclusive", cls["detail"], measured, subs)
    if cause == "tool":
        return _res("pass", cls["detail"], measured, subs)
    return _res("fail", cls["detail"], measured, subs)


PREDICATES: Dict[str, Callable[[Dict[str, Any], Dict[str, Any]],
                               Dict[str, Any]]] = {
    "at_rest": pred_at_rest,
    "net_translation": pred_net_translation,
    "translate_then_rotate": pred_translate_then_rotate,
    "not_parked": pred_not_parked,
    "joints_at_rest": pred_joints_at_rest,
    "resumed": pred_resumed,
}


# ── SECONDARY: answer accuracy.  Never a verdict, always separate. ────

_WORD_NUMBERS = {
    "zero": 0, "no": 0, "none": 0, "one": 1, "two": 2, "three": 3, "four": 4,
    "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10,
    "eleven": 11, "twelve": 12,
}


def extract_numbers(text: str) -> List[float]:
    """Every number a reply mentions, digits and small words alike."""
    if not text:
        return []
    out: List[float] = []
    for m in re.finditer(r"-?\d+(?:\.\d+)?", text):
        try:
            out.append(float(m.group(0)))
        except ValueError:
            pass
    for w, v in _WORD_NUMBERS.items():
        if re.search(rf"\b{w}\b", text, re.IGNORECASE):
            out.append(float(v))
    return out


def check_number_in_text(reply: str, truths: Sequence[Optional[float]],
                         tol: float = 0.0) -> Dict[str, Any]:
    """Does the reply state a ground truth that was MEASURED from /state?

    SECONDARY EVIDENCE ONLY. It never contributes to a probe verdict, for the
    reason this whole file exists: a sentence is not a robot. It is recorded
    because for a pure question there IS no robot-state consequence, and
    refusing to look at the answer at all would score "I don't recognise that"
    identically to a correct one.

    `truths` may hold several acceptable values (e.g. the counter read at ask
    time AND at reply time, since a live line can tick between the two).
    """
    vals = [float(t) for t in truths if isinstance(t, (int, float))
            and not isinstance(t, bool)]
    found = extract_numbers(reply or "")
    if not vals:
        return {"match": None, "reason": "ground truth unavailable",
                "truths": [], "found": found}
    if not found:
        return {"match": False, "reason": "reply states no number",
                "truths": vals, "found": []}
    hit = any(any(abs(f - t) <= tol for t in vals) for f in found)
    return {"match": bool(hit),
            "reason": ("a stated number matches the measured ground truth"
                       if hit else "no stated number matches the measured "
                                   "ground truth"),
            "truths": vals, "found": found, "tol": tol}


def resolve_truth(name: str, snaps: Dict[str, Optional[dict]]
                  ) -> Tuple[Optional[float], str]:
    """Ground truth for an answer check, read from the bridges' own /state.

    Never from the model, and never hard-coded: a constant here would go
    stale the first time the world changes.
    """
    def dig(bridge: str, *path: str) -> Any:
        cur: Any = snaps.get(bridge)
        for k in path:
            if not isinstance(cur, dict):
                return None
            cur = cur.get(k)
        return cur

    if name == "park_row_count":
        # tug_a OWNS the park row (it is the only loop that appends on a
        # park), so it is the authority. tug_b seeds the same list at startup
        # and can drift; a disagreement makes the truth unusable rather than
        # arbitrary.
        a = dig("tug_a", "idle_loop", "park_row_count")
        b = dig("tug_b", "idle_loop", "park_row_count")
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and a != b:
            return None, (f"tug_a says {a} carts parked, tug_b says {b} -- the "
                          f"two tugs disagree, so there is no usable ground "
                          f"truth for this turn")
        if isinstance(a, (int, float)):
            return float(a), "tug_a /state.idle_loop.park_row_count"
        return None, "no tug published park_row_count"
    if name == "shipped_total":
        v = dig("omniarm6", "line", "shipped_total")
        if isinstance(v, (int, float)):
            return float(v), "omniarm6 /state.line.shipped_total"
        return None, "the arm published no line.shipped_total"
    if name == "tug_x_rounded":
        p = pose_of(snaps.get("_probe_target"))
        if p is None:
            return None, "no pose for the probe target"
        return round(p[0], 1), "probe target /state.x"
    return None, f"unknown truth source {name!r}"


# ══════════════════════════════════════════════════════════════════════
# HTTP.  GET via measure_line; POST is ours (and is the thing under test).
# ══════════════════════════════════════════════════════════════════════

def post_json(url: str, payload: dict, timeout: float, token: str = ""
              ) -> Tuple[int, Any, str]:
    """One POST. Returns (http_status, parsed_body_or_None, error_string).

    A 409 is NOT an exception here: the bridges answer 409 by design when a
    motion is already in flight (PROTOCOL.md 5.4.1), and treating that as a
    failure of the mode under test would be measuring the harness's own
    impatience. The caller retries.
    """
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Content-Type", "application/json")
    req.add_header("Accept", "application/json")
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read()
            try:
                obj = json.loads(body.decode("utf-8"))
            except ValueError:
                obj = None
            return int(resp.status), obj, ""
    except urllib.error.HTTPError as e:
        try:
            obj = json.loads(e.read().decode("utf-8"))
        except Exception:                                    # noqa: BLE001
            obj = None
        return int(e.code), obj, f"HTTP {e.code} {e.reason}"
    except urllib.error.URLError as e:
        return 0, None, f"not answering ({e.reason})"
    except (OSError, ValueError) as e:
        return 0, None, repr(e)


def is_busy(status: int, body: Any) -> bool:
    """True when the bridge refused because a motion is already in flight.

    Two shapes, because there are two doors: a direct tool route answers HTTP
    409, while POST /prompt answers 200 and reports the refusal INSIDE the
    actions list (the offline router's `_busy_reply`). Missing the second
    shape would score a refusal as a failure of the model.
    """
    if status == 409:
        return True
    if isinstance(body, dict):
        if str(body.get("error") or "").lower() == "busy":
            return True
        for a in (body.get("actions") or []):
            if isinstance(a, dict) and str(a.get("result") or "").lower() == "busy":
                return True
    return False


def actions_of(body: Any) -> List[Dict[str, Any]]:
    if isinstance(body, dict) and isinstance(body.get("actions"), list):
        return [a for a in body["actions"] if isinstance(a, dict)]
    return []


def reply_of(body: Any) -> str:
    if isinstance(body, dict):
        return str(body.get("response") or body.get("agent") or "")
    return ""


# ══════════════════════════════════════════════════════════════════════
# Background sampler -- the throughput lane, GET only, reuses measure_line.
# ══════════════════════════════════════════════════════════════════════

class Sampler(threading.Thread):
    """Polls all three bridges into measure_line Recorders while the probes
    run, so throughput is measured over the SAME wall clock as the
    intervention rather than inferred from it."""

    def __init__(self, recs: Dict[str, "ml.Recorder"], hz: float,
                 timeout: float, token: str) -> None:
        super().__init__(daemon=True, name="bench-sampler")
        self.recs = recs
        self.period = 1.0 / max(hz, 0.05)
        self.timeout = timeout
        self.token = token
        self.t0 = time.monotonic()
        # NOT `_stop`: threading.Thread._stop is a private METHOD that join()
        # calls, and shadowing it with an Event makes every join() raise
        # "'Event' object is not callable" at the end of an otherwise good run.
        self._halt = threading.Event()
        self.error: Optional[str] = None

    def stop(self) -> None:
        self._halt.set()

    def run(self) -> None:
        names = list(self.recs)
        next_tick = time.monotonic()

        def one(name: str):
            rec = self.recs[name]
            a = time.monotonic()
            try:
                st = ml.get_json(rec.url + "/state", self.timeout, self.token)
                b = time.monotonic()
                if not isinstance(st, dict):
                    return name, a - self.t0, None, b - a, "non-object /state"
                return name, a - self.t0, st, b - a, ""
            except ml.BridgeError as e:
                return name, a - self.t0, None, time.monotonic() - a, str(e)

        with ThreadPoolExecutor(max_workers=max(3, len(names))) as pool:
            while not self._halt.is_set():
                now = time.monotonic()
                if now < next_tick:
                    self._halt.wait(min(next_tick - now, 0.05))
                    continue
                next_tick += self.period
                if next_tick < time.monotonic():
                    next_tick = time.monotonic() + self.period
                try:
                    for name, t, st, lat, err in pool.map(one, names):
                        rec = self.recs[name]
                        if st is None:
                            rec.fail(t, err)
                        else:
                            rec.add(t, st, lat)
                except Exception as e:                       # noqa: BLE001
                    self.error = repr(e)
                    return


def slice_recorder(rec: "ml.Recorder", t0: float, t1: float) -> "ml.Recorder":
    """A Recorder holding only the samples inside [t0, t1]."""
    out = ml.Recorder(rec.name, rec.url)
    out.meta = dict(rec.meta)
    for i, t in enumerate(rec.t):
        if t0 <= t <= t1:
            out.t.append(t)
            out.sim.append(rec.sim[i])
            out.states.append(rec.states[i])
            out.latencies.append(rec.latencies[i])
    return out


def window_metrics(recs: Dict[str, "ml.Recorder"], t0: float, t1: float,
                   hz: float) -> Dict[str, Any]:
    """Throughput + operator-pause cost over one window, via measure_line's
    own analysis so the numbers are defined identically to the baseline
    harness (see its README, "What each metric means")."""
    out: Dict[str, Any] = {"t_start_s": round(t0, 2), "t_end_s": round(t1, 2),
                           "window_s": round(t1 - t0, 2)}
    arm = slice_recorder(recs["omniarm6"], t0, t1)
    if len(arm.t) < 2:
        out["error"] = "not enough OMNIARM6 samples in this window"
        return out
    line = ml.analyse_line(arm)
    th = line["throughput"]
    aw = line["arm_wait"]
    rt = ml.realtime_factor(arm.t, arm.sim)
    out.update({
        "samples": len(arm.t),
        "realtime_factor": rt.get("factor"),
        "boxes_shipped": th["boxes_shipped"],
        "boxes_per_minute": th["boxes_per_minute"],
        "fills_per_minute": th["fills_per_minute"],
        "picks_per_minute": th["picks_per_minute"],
        "arm_not_working_s": aw["arm_not_working_s"],
        "arm_paused_by_operator_s": aw["arm_paused_by_operator_s"],
        "arm_paused_by_operator_frac": (
            aw["arm_paused_by_operator_s"] / (t1 - t0) if t1 > t0 else None),
        "fill_blocked_total_s": aw["fill_blocked_total_s"],
        "complete_ship_cycles": aw["complete_cycles"],
    })
    tugs: Dict[str, Any] = {}
    for name in ("tug_a", "tug_b"):
        sub = slice_recorder(recs[name], t0, t1)
        if len(sub.t) < 2:
            tugs[name] = {"error": "not enough samples"}
            continue
        a = ml.analyse_tug(sub, 0.001, math.radians(0.02), 0.02,
                           math.radians(1.0), hz)
        if "error" in a:
            tugs[name] = a
            continue
        tugs[name] = {
            "paused_by_operator_frac": a["paused_by_operator_frac"],
            "path_length_m": a["motion"]["path_length_m"],
            "rotation_deg": a["motion"]["rotation_deg"],
            "stationary_frac": a["time"]["stationary_frac"],
            "jobs_delta": a["counters"]["jobs_total"]["delta"],
            "cycles_delta": a["counters"]["cycles"]["delta"],
        }
    out["tugs"] = tugs
    return out


# ══════════════════════════════════════════════════════════════════════
# Run mechanics
# ══════════════════════════════════════════════════════════════════════

class Runner:
    def __init__(self, args: argparse.Namespace, recs: Dict[str, "ml.Recorder"],
                 sampler: Sampler, redact: Redactor) -> None:
        self.args = args
        self.recs = recs
        self.sampler = sampler
        self.redact = redact
        self.urls = {name: rec.url for name, rec in recs.items()}
        self.role_url = {"tug": self.urls[args.tug_robot],
                         "arm": self.urls[args.arm_robot]}
        self.role_bridge = {"tug": args.tug_robot, "arm": args.arm_robot}
        self.notes: List[str] = []

    # ── low-level ────────────────────────────────────────────────────
    def t(self) -> float:
        return time.monotonic() - self.sampler.t0

    def get_state(self, bridge: str) -> Optional[dict]:
        try:
            st = ml.get_json(self.urls[bridge] + "/state", self.args.timeout,
                             self.args.token)
            return st if isinstance(st, dict) else None
        except ml.BridgeError:
            return None

    def all_states(self) -> Dict[str, Optional[dict]]:
        return {name: self.get_state(name) for name in self.urls}

    def post(self, role_or_bridge: str, path: str, payload: dict,
             timeout: Optional[float] = None) -> Tuple[int, Any, str]:
        url = self.role_url.get(role_or_bridge) or self.urls[role_or_bridge]
        return post_json(url + path, payload,
                         timeout if timeout is not None else self.args.timeout,
                         self.args.token)

    # ── setup / reset between probes ─────────────────────────────────
    def do_setup(self, probe: Dict[str, Any]) -> Dict[str, Any]:
        """Put the target robot into the probe's required precondition.

        Uses the DIRECT endpoints, never chat, so the precondition is
        established identically in every mode and the thing under test stays
        the thing under test.

        The quiet wait afterwards is not cosmetic: `act_resume_autonomy` sets
        a ~1.5 s exemption window during which the NEXT command's idle-loop
        pause is deliberately swallowed. A probe fired inside that window
        would silently fail to arm the pause and every downstream measurement
        would be wrong.
        """
        kind = probe.get("setup")
        role = probe["target"]
        rec: Dict[str, Any] = {"kind": kind, "t_s": round(self.t(), 2)}
        if kind == "pause":
            status, body, err = self.post(role, "/stop_robot", {})
            rec.update({"endpoint": "/stop_robot", "status": status,
                        "error": self.redact(err)})
            rec["t_armed_s"] = round(self.t(), 3)
        elif kind == "resume":
            status, body, err = self.post(role, "/resume_autonomy", {})
            rec.update({"endpoint": "/resume_autonomy", "status": status,
                        "error": self.redact(err)})
        else:
            rec["endpoint"] = None
            return rec
        wait = (self.args.post_setup_quiet_s if kind == "pause"
                else self.args.post_reset_quiet_s)
        time.sleep(wait)
        rec["quiet_wait_s"] = wait
        st = self.get_state(self.role_bridge[role])
        rec["paused_after_setup"] = paused_of(st)
        return rec

    # ── the probe itself ─────────────────────────────────────────────
    def measure_pre_motion(self, bridge: str) -> Dict[str, Any]:
        """Two GETs a short interval apart, to know whether the robot was
        ALREADY still before a 'stop'. A stop issued to a parked robot proves
        nothing, and that has to be visible rather than counted as a win."""
        s0 = self.get_state(bridge)
        time.sleep(self.args.pre_probe_gap_s)
        s1 = self.get_state(bridge)
        out: Dict[str, Any] = {"pre_speed_m_s": None,
                               "pre_joint_step_rad": None}
        p0, p1 = pose_of(s0), pose_of(s1)
        if p0 and p1 and self.args.pre_probe_gap_s > 0:
            out["pre_speed_m_s"] = round(
                math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                / self.args.pre_probe_gap_s, 4)
        q0, q1 = joints_of(s0), joints_of(s1)
        if q0 and q1:
            n = min(len(q0), len(q1))
            out["pre_joint_step_rad"] = round(
                max(abs(q1[i] - q0[i]) for i in range(n)), 5)
        out["_last_state"] = s1
        return out

    def run_probe(self, probe: Dict[str, Any]) -> Dict[str, Any]:
        role = probe["target"]
        bridge = self.role_bridge[role]
        verify_s = resolve_verify_s(probe, self.args.resume_s)
        out: Dict[str, Any] = {
            "key": probe["key"], "tier": probe["tier"], "target_role": role,
            "target_bridge": bridge, "text": probe["text"],
            "predicate": probe["predicate"], "params": probe.get("params") or {},
            "verify_s": verify_s,
            "offline_expectation": probe.get("offline_expectation"),
            "t_start_s": round(self.t(), 2),
        }

        out["setup"] = self.do_setup(probe)
        t_armed = out["setup"].get("t_armed_s")

        pre = self.measure_pre_motion(bridge)
        before = pre.pop("_last_state", None) or self.get_state(bridge)
        out["pre"] = pre
        if before is None:
            out["verdict"] = "inconclusive"
            out["reason"] = "no BEFORE state snapshot (bridge not answering)"
            return out
        truth_snaps = self.all_states()
        truth_snaps["_probe_target"] = before
        out["paused_before"] = paused_of(before)

        # ── issue the prompt, retrying only on a BUSY refusal ──────────
        attempts: List[Dict[str, Any]] = []
        status = 0
        body: Any = None
        err = ""
        t_send = 0.0
        latency = 0.0
        for attempt in range(self.args.retry_busy + 1):
            t_send = self.t()
            a = time.monotonic()
            status, body, err = self.post(role, "/prompt",
                                          {"text": probe["text"]},
                                          timeout=self.args.prompt_timeout)
            latency = time.monotonic() - a
            attempts.append({"attempt": attempt + 1, "status": status,
                             "latency_s": round(latency, 3),
                             "busy": is_busy(status, body),
                             "error": self.redact(err)})
            if not is_busy(status, body):
                break
            if attempt < self.args.retry_busy:
                time.sleep(self.args.busy_backoff_s)
        out["attempts"] = attempts
        out["http_status"] = status
        out["latency_s"] = round(latency, 3)
        out["reply"] = self.redact(reply_of(body))
        out["actions"] = self.redact(actions_of(body))
        out["http_error"] = self.redact(err)
        if isinstance(body, dict) and body.get("error"):
            out["bridge_error"] = self.redact(str(body["error"]))

        # ── bounded observation window ────────────────────────────────
        series: List[Dict[str, Any]] = []
        period = 1.0 / max(self.args.verify_hz, 0.5)
        deadline = time.monotonic() + verify_s
        while time.monotonic() < deadline:
            st = self.get_state(bridge)
            series.append({"t": round(self.t() - t_send, 3), "state": st})
            time.sleep(period)
        out["samples"] = len(series)
        after = None
        for s in reversed(series):
            if s.get("state"):
                after = s["state"]
                break
        out["t_end_s"] = round(self.t(), 2)

        # ── verdict, from state alone ─────────────────────────────────
        if is_busy(status, body):
            out["verdict"] = "inconclusive"
            out["reason"] = (f"bridge stayed BUSY across "
                             f"{self.args.retry_busy + 1} attempts -- a 409 "
                             f"is a protocol refusal, not a failure of the "
                             f"mode under test")
            out["measured"] = {}
            out["subgoals"] = []
        elif status != 200 or body is None:
            out["verdict"] = "error"
            out["reason"] = f"HTTP {status}: {self.redact(err) or 'no body'}"
            out["measured"] = {}
            out["subgoals"] = []
        else:
            ev = {
                "before": before, "after": after, "series": series,
                "latency_s": latency,
                "resume_s": self.args.resume_s,
                "t_arm_before_send_s": (max(0.0, t_send - t_armed)
                                        if t_armed is not None else 0.0),
                "tool_grace_s": self.args.tool_grace_s,
                "timer_tol_s": self.args.timer_tol_s,
                "pre_speed_m_s": pre.get("pre_speed_m_s"),
                "pre_joint_step_rad": pre.get("pre_joint_step_rad"),
            }
            fn = PREDICATES[probe["predicate"]]
            verdict = fn(probe, ev)
            out.update(verdict)

        out["paused_after"] = paused_of(after)

        # ── SECONDARY: answer accuracy.  Never touches the verdict. ────
        tc = probe.get("text_check")
        if tc:
            t_ask, note_ask = resolve_truth(tc["truth"], truth_snaps)
            reply_snaps = self.all_states()
            reply_snaps["_probe_target"] = after or before
            t_reply, note_reply = resolve_truth(tc["truth"], reply_snaps)
            chk = check_number_in_text(out.get("reply") or "",
                                       [t_ask, t_reply],
                                       float(tc.get("tol", 0.0)))
            chk.update({
                "truth_source": tc["truth"],
                "truth_at_ask": t_ask, "truth_at_reply": t_reply,
                "truth_note": note_ask if t_ask is not None else note_reply,
                "IS_NOT_A_VERDICT": ("secondary evidence only -- the probe "
                                     "verdict above comes from measured robot "
                                     "state, never from this"),
            })
            out["text_check"] = chk

        # ── the discriminator, hoisted for the summary ────────────────
        if probe["predicate"] == "resumed":
            m = out.get("measured") or {}
            out["discriminator"] = {
                "time_to_resume_s": m.get("time_to_resume_s"),
                "time_to_resume_after_reply_s":
                    m.get("time_to_resume_after_reply_s"),
                "cause": m.get("cause"),
                "detail": m.get("detail"),
                "timer_due_s": m.get("timer_due_s"),
                "reply_latency_s": m.get("reply_latency_s"),
            }
        return out


def resolve_verify_s(probe: Dict[str, Any], resume_s: float) -> float:
    """A resume probe must be watched PAST the auto-timer, or 'the agent did
    it' and 'the clock did it' cannot be told apart."""
    v = probe.get("verify_s")
    if v is None:
        return float(resume_s) + 25.0
    return float(v)


# ── resume_s calibration (opt-in) ─────────────────────────────────────

def calibrate_resume_s(runner: Runner, bridge_role: str, budget_s: float
                       ) -> Dict[str, Any]:
    """MEASURE the auto-resume timer instead of asserting it.

    The whole resume attribution rests on knowing when the quiet window
    expires, and the bridges do not publish `--idle-resume-s` anywhere
    readable. So: park the robot with a direct /stop_robot, then watch
    idle_loop.paused with GETs until it clears. The elapsed time IS the timer.
    Costs one pause-length of throughput, which is why it is opt-in.
    """
    out: Dict[str, Any] = {"role": bridge_role, "method": "stop_robot then "
                           "poll idle_loop.paused until it clears"}
    bridge = runner.role_bridge[bridge_role]
    status, _body, err = runner.post(bridge_role, "/stop_robot", {})
    t0 = time.monotonic()
    out["stop_status"] = status
    if err:
        out["error"] = runner.redact(err)
        return out
    deadline = t0 + budget_s
    while time.monotonic() < deadline:
        st = runner.get_state(bridge)
        p = paused_of(st)
        if p is False and time.monotonic() - t0 > 1.0:
            out["measured_resume_s"] = round(time.monotonic() - t0, 2)
            return out
        time.sleep(0.5)
    out["error"] = (f"idle_loop.paused never cleared within {budget_s:.0f}s -- "
                    f"is the idle loop running at all?")
    return out


# ══════════════════════════════════════════════════════════════════════
# INTEGRITY GATE 1 -- LIVENESS.  Is anything actually driving this port?
#
# THE FAILURE THIS EXISTS FOR.  scripts/dev/headless_runner.py stops the
# engine with proc.terminate() -- TerminateProcess on Windows. The engine
# never tells its controllers to quit, so every bridge CONTROLLER survives as
# an orphan: its main thread blocks forever inside robot.step() on a dead IPC
# pipe, and its daemon HTTP thread keeps serving 200s from a MobileBridge
# whose Supervisor is gone. ThreadingHTTPServer sets allow_reuse_address = 1
# (HTTPServer sets it; TCPServer does not), and on Windows SO_REUSEADDR lets a
# NEW process bind an ALREADY-BOUND loopback port. Measured on this box: two
# processes both LISTENING on one 127.0.0.1 port, and the EARLIER binder
# answered all ten requests -- i.e. a freshly launched bridge can be shadowed
# in full by a corpse from a previous run.
#
# `last_tick_at` is the tell. Both bridges stamp it with time.time() from
# inside their tick loop (omnilink_mobile_bridge.py MobileBridge.tick,
# omnilink_arm_bridge.py ArmBridge.tick) and publish it in /state. A corpse
# answers 200 with a value that never moves. So: read it twice, and REFUSE if
# it did not advance. This is a hard gate, not a warning, because every single
# number downstream is meaningless when it trips.
# ══════════════════════════════════════════════════════════════════════

def _finite(v: Any) -> Optional[float]:
    if isinstance(v, bool) or v is None:
        return None
    if isinstance(v, (int, float)) and math.isfinite(float(v)):
        return float(v)
    return None


def classify_liveness(name: str, url: str, first: Optional[dict],
                      second: Optional[dict], gap_s: float,
                      min_advance_s: float = 1e-6) -> Dict[str, Any]:
    """Did this bridge's TICK LOOP advance between two /state reads?

    Pure: takes the two state dicts, returns a verdict. `alive` is the only
    verdict that lets a run proceed; `frozen` is the orphaned-bridge hijack
    and is fatal; `unverifiable` means the bridge publishes no `last_tick_at`
    at all, which no bridge in this world does but a future one might.
    """
    out: Dict[str, Any] = {"bridge": name, "url": url,
                           "gap_s": round(float(gap_s), 3)}
    if not isinstance(first, dict) or not isinstance(second, dict):
        out["verdict"] = "no_answer"
        out["detail"] = ("the bridge did not return a /state object for both "
                         "reads")
        return out
    t0 = _finite(first.get("last_tick_at"))
    t1 = _finite(second.get("last_tick_at"))
    s0 = _finite(first.get("sim_time"))
    s1 = _finite(second.get("sim_time"))
    out.update({
        "last_tick_at_first": t0, "last_tick_at_second": t1,
        "sim_time_first": s0, "sim_time_second": s1,
        "tick_advance_s": (None if (t0 is None or t1 is None)
                           else round(t1 - t0, 6)),
        "sim_advance_s": (None if (s0 is None or s1 is None)
                          else round(s1 - s0, 6)),
    })
    if t0 is None or t1 is None:
        out["verdict"] = "unverifiable"
        out["detail"] = ("this bridge publishes no numeric /state.last_tick_at, "
                         "so 'is anything still stepping it?' cannot be "
                         "answered over HTTP")
        return out
    if (t1 - t0) <= min_advance_s:
        out["verdict"] = "frozen"
        out["detail"] = (
            f"last_tick_at did NOT advance across a {gap_s:.2f}s gap "
            f"({t0!r} -> {t1!r}). The HTTP thread is answering but nothing is "
            f"stepping this robot.")
        return out
    out["verdict"] = "alive"
    out["detail"] = (f"tick loop advanced {t1 - t0:.3f}s over a {gap_s:.2f}s "
                     f"wall gap")
    return out


def liveness_remedy(bad: Sequence[Dict[str, Any]]) -> str:
    """The operator-facing message for a failed liveness gate.

    Names the port, both timestamps and the fix. Two causes produce the same
    HTTP symptom and both are stated, because guessing between them from
    outside the process is not possible: an ORPHANED controller (its engine
    was terminated and never told it to quit) and a PAUSED simulation.
    """
    lines = ["LIVENESS GATE FAILED -- refusing to measure a robot that is not "
             "being stepped.", ""]
    for b in bad:
        lines.append(
            f"  {b.get('bridge')}  {b.get('url')}\n"
            f"      last_tick_at  first={b.get('last_tick_at_first')!r}  "
            f"second={b.get('last_tick_at_second')!r}  "
            f"(advance {b.get('tick_advance_s')!r}s over "
            f"{b.get('gap_s')!r}s)\n"
            f"      sim_time      first={b.get('sim_time_first')!r}  "
            f"second={b.get('sim_time_second')!r}\n"
            f"      {b.get('detail')}")
    lines += [
        "",
        "  A 200 OK with a frozen tick loop means ONE of:",
        "    (a) you are talking to an ORPHANED bridge controller. The engine "
        "was stopped",
        "        with terminate()/TerminateProcess, so its controllers were "
        "never told to",
        "        quit: the main thread is blocked in robot.step() on a dead "
        "IPC pipe while",
        "        the daemon HTTP thread keeps serving. On Windows "
        "SO_REUSEADDR lets a NEW",
        "        bridge bind the SAME port, and the corpse can win the "
        "connection.",
        "    (b) the simulation is PAUSED (nothing is stepping, so nothing "
        "ticks).",
        "",
        "  Remedy:",
        "    1. Stop every simulator and controller left over from a previous "
        "run:",
        "         Windows:  taskkill /IM omnisim-bin.exe /F",
        "                   (then kill any python.exe still holding 8765-8767 "
        "-- find them",
        "                    with:  netstat -ano -p TCP | findstr \":876\")",
        "         POSIX:    pkill -f omnisim-bin ; pkill -f omnilink_.*_bridge",
        "    2. Confirm the ports are FREE before relaunching (no LISTENING "
        "row at all).",
        "    3. Relaunch the world -- see BENCH_OMNILINK.md section 2 (use the "
        "headless",
        "       runner, NOT launch.bat).",
        "    4. If the sim is merely paused, press play / relaunch with "
        "--realtime.",
    ]
    return "\n".join(lines)


def probe_liveness(recs: Dict[str, "ml.Recorder"], timeout: float, token: str,
                   gap_s: float) -> Dict[str, Any]:
    """Two /state reads per bridge, `gap_s` apart. One gap total, not N."""
    def read_all() -> Dict[str, Optional[dict]]:
        out: Dict[str, Optional[dict]] = {}
        for name, rec in recs.items():
            try:
                st = ml.get_json(rec.url + "/state", timeout, token)
                out[name] = st if isinstance(st, dict) else None
            except ml.BridgeError:
                out[name] = None
        return out

    t_a = time.monotonic()
    first = read_all()
    time.sleep(max(0.0, gap_s))
    second = read_all()
    actual_gap = time.monotonic() - t_a
    per: Dict[str, Any] = {}
    for name, rec in recs.items():
        per[name] = classify_liveness(name, rec.url, first.get(name),
                                      second.get(name), actual_gap)
    per["_failed"] = [n for n in recs
                      if per[n].get("verdict") == "frozen"]
    per["_unverifiable"] = [n for n in recs
                            if per[n].get("verdict") in ("unverifiable",
                                                         "no_answer")]
    per["_states"] = second
    return per


# ══════════════════════════════════════════════════════════════════════
# INTEGRITY GATE 2 -- IDENTITY.  Is it the SAME process at the end?
#
# There is no process identity on any bridge endpoint: no pid, no boot id, no
# start time. /protocol carries only a static {name, robot_id}. So this gate
# is built from the strongest signals that DO exist, and its limits are stated
# rather than papered over:
#
#   * OS listener PIDs (Windows only, best effort via netstat -ano). This is
#     the only REAL fingerprint available, and it is also the only way to see
#     the duplicate-bind state directly: TWO PIDs LISTENING on one port means
#     which process answers is not determined by anything you control.
#   * /state.id and /state.model -- a different robot behind the port.
#   * sim_time -- the engine clock. It cannot run backwards within one run, so
#     a decrease proves a different process.
#   * the bridges' own MONOTONE session counters (idle_loop.cycles,
#     idle_loop.jobs_total, idle_loop.delivered_total, line.shipped_total).
#     They reset to 0 on controller start, so a decrease proves a restart.
#
# LIMIT, stated plainly: monotonicity is a NECESSARY condition, not a
# fingerprint. Two live processes whose clocks and counters happen to be
# consistent are indistinguishable this way. The PID set is a true
# fingerprint; where netstat is unavailable (non-Windows, locale mismatch,
# --no-port-identity) the gate degrades to the monotonicity checks and SAYS SO
# in the report.
# ══════════════════════════════════════════════════════════════════════

# (bridge-state path) -> label. Every one of these is monotone non-decreasing
# for the lifetime of a controller process and starts at 0.
_MONOTONE_PATHS: Tuple[Tuple[str, ...], ...] = (
    ("idle_loop", "cycles"),
    ("idle_loop", "jobs_total"),
    ("idle_loop", "delivered_total"),
    ("idle_loop", "holds_total"),
    ("line", "shipped_total"),
)


def identity_of(state: Optional[dict]) -> Dict[str, Any]:
    """The identity fingerprint this bridge is capable of publishing."""
    out: Dict[str, Any] = {"id": None, "model": None, "sim_time": None,
                           "counters": {}}
    if not isinstance(state, dict):
        return out
    out["id"] = state.get("id")
    out["model"] = state.get("model")
    out["sim_time"] = _finite(state.get("sim_time"))
    for path in _MONOTONE_PATHS:
        cur: Any = state
        for k in path:
            if not isinstance(cur, dict):
                cur = None
                break
            cur = cur.get(k)
        v = _finite(cur)
        if v is not None:
            out["counters"][".".join(path)] = v
    return out


def compare_identity(name: str, first: Dict[str, Any], last: Dict[str, Any],
                     wall_elapsed_s: float, pids_first: Optional[List[int]],
                     pids_last: Optional[List[int]],
                     max_realtime_factor: float = 20.0,
                     sim_scan: Optional[Dict[str, Any]] = None
                     ) -> Dict[str, Any]:
    """Same process at the end as at the start?  Pure.

    `swapped` is positive evidence that it is NOT; `suspect` is a soft flag
    that never fails a run on its own; `stable` means nothing contradicted it,
    which is weaker than "proven identical" and is labelled as such.
    """
    findings: List[str] = []
    swapped = False
    suspect = False

    for key in ("id", "model"):
        a, b = first.get(key), last.get(key)
        if a is not None and b is not None and a != b:
            swapped = True
            findings.append(f"/state.{key} changed {a!r} -> {b!r}: a DIFFERENT "
                            f"robot is behind this port")

    sa, sb = first.get("sim_time"), last.get("sim_time")
    sim_advance = None
    implied_rt = None
    if sa is not None and sb is not None:
        sim_advance = sb - sa
        if sim_advance < -1e-6:
            swapped = True
            findings.append(f"sim_time went BACKWARDS {sa:.3f} -> {sb:.3f}: "
                            f"the engine clock cannot do that within one run, "
                            f"so this is a different process")
        elif wall_elapsed_s > 1.0:
            implied_rt = sim_advance / wall_elapsed_s
            if implied_rt > max_realtime_factor:
                suspect = True
                findings.append(
                    f"sim_time advanced {sim_advance:.1f}s over "
                    f"{wall_elapsed_s:.1f}s wall ({implied_rt:.1f}x realtime, "
                    f"ceiling {max_realtime_factor:.0f}x) -- plausible under "
                    f"--mode=fast, but also what a swap to an older process "
                    f"looks like")

    # Full-run coverage, not just the endpoints. See scan_sim_monotonicity:
    # a swap that happens halfway through has time to climb the endpoint
    # comparison back past its starting value, and measurably does.
    if isinstance(sim_scan, dict) and sim_scan.get("backward_jumps"):
        swapped = True
        w = sim_scan.get("worst") or {}
        findings.append(
            f"the background sim_time series went BACKWARDS "
            f"{sim_scan['backward_jumps']} time(s) during the run (worst: "
            f"{w.get('from_sim_s')} -> {w.get('to_sim_s')} at run t="
            f"{w.get('at_run_t_s')}s). The engine clock is monotone within one "
            f"process, so a different process answered mid-run")

    ca, cb = first.get("counters") or {}, last.get("counters") or {}
    for key, av in ca.items():
        bv = cb.get(key)
        if bv is None:
            suspect = True
            findings.append(f"counter {key} was published at preflight "
                            f"({av:g}) and is GONE at the end")
            continue
        if bv < av - 1e-9:
            swapped = True
            findings.append(
                f"counter {key} DECREASED {av:g} -> {bv:g}. These reset to "
                f"zero when a controller starts, so this is a restarted or "
                f"different process")

    pid_note = "not probed"
    if pids_first is not None and pids_last is not None:
        if len(pids_first) > 1 or len(pids_last) > 1:
            swapped = True
            findings.append(
                f"MORE THAN ONE process is LISTENING on this port "
                f"(preflight {pids_first}, end {pids_last}). With "
                f"SO_REUSEADDR both are bound and which one answers a given "
                f"connection is not something this harness controls")
            pid_note = "duplicate listener"
        elif set(pids_first) != set(pids_last):
            swapped = True
            findings.append(f"the LISTENING pid changed {pids_first} -> "
                            f"{pids_last}: the process behind this port was "
                            f"replaced during the run")
            pid_note = "pid changed"
        elif pids_first:
            pid_note = f"same pid {pids_first[0]} throughout"
        else:
            pid_note = ("no LISTENING row parsed -- treated as inconclusive, "
                        "not as zero listeners")

    verdict = "swapped" if swapped else ("suspect" if suspect else "stable")
    return {
        "bridge": name,
        "verdict": verdict,
        "findings": findings,
        "pid_note": pid_note,
        "pids_first": pids_first,
        "pids_last": pids_last,
        "sim_advance_s": (None if sim_advance is None
                          else round(sim_advance, 3)),
        "wall_elapsed_s": round(wall_elapsed_s, 2),
        "implied_realtime_factor": (None if implied_rt is None
                                    else round(implied_rt, 3)),
        "sim_scan": sim_scan,
        "first": first,
        "last": last,
        "STRENGTH": ("'stable' means nothing CONTRADICTED continuity, which is "
                     "weaker than proof. Only the pid comparison is a true "
                     "fingerprint; the clock and counter checks are necessary "
                     "conditions that two consistent live processes could both "
                     "satisfy. MEASURED LIMIT: with the pid probe off, a "
                     "deliberate mid-run process swap on a SHORT run was scored "
                     "`stable` by the endpoint comparison alone -- which is why "
                     "the full-run sim_time scan exists."),
    }


def scan_sim_monotonicity(times: Sequence[float],
                          sims: Sequence[Optional[float]],
                          tol_s: float = 1e-6) -> Dict[str, Any]:
    """Scan the FULL background sim_time series for a backward jump.

    Comparing only the endpoints of a run is a weak swap detector: a process
    that took the port over halfway through has plenty of wall time to climb
    back past the first reading (measured -- an endpoint-only check scored a
    deliberate mid-run process swap as `stable`). The background sampler
    already records `sim_time` at --hz for the whole run, and the engine clock
    is monotone non-decreasing within one process, so ANY decrease in that
    series is positive evidence that a different process answered.

    Pure. Returns the worst drop found and where.
    """
    worst: Optional[Dict[str, Any]] = None
    drops = 0
    prev_t: Optional[float] = None
    prev_s: Optional[float] = None
    for t, s in zip(times, sims):
        v = _finite(s)
        if v is None:
            continue
        if prev_s is not None and v < prev_s - tol_s:
            drops += 1
            drop = prev_s - v
            if worst is None or drop > worst["drop_s"]:
                worst = {"drop_s": round(drop, 6),
                         "from_sim_s": round(prev_s, 6),
                         "to_sim_s": round(v, 6),
                         "at_run_t_s": round(float(t), 3),
                         "prev_run_t_s": (None if prev_t is None
                                          else round(float(prev_t), 3))}
        prev_t, prev_s = t, v
    return {"samples": sum(1 for s in sims if _finite(s) is not None),
            "backward_jumps": drops, "worst": worst,
            "monotone": drops == 0}


def parse_netstat_listeners(text: str, port: int) -> List[int]:
    """PIDs LISTENING on `port`, parsed from Windows `netstat -ano` output.

    Pure so it can be unit tested without a socket. Deliberately strict: rows
    that do not parse are skipped rather than guessed at, and an empty result
    is reported by the caller as INCONCLUSIVE (a locale whose state column is
    not "LISTENING" must not read as "nothing is bound").
    """
    pids: List[int] = []
    for line in (text or "").splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        if parts[0].upper() != "TCP":
            continue
        if parts[3].upper() != "LISTENING":
            continue
        local = parts[1]
        if ":" not in local:
            continue
        if local.rsplit(":", 1)[-1] != str(port):
            continue
        try:
            pid = int(parts[4])
        except ValueError:
            continue
        if pid not in pids:
            pids.append(pid)
    return sorted(pids)


def listener_pids(ports: Sequence[int], timeout_s: float = 10.0
                  ) -> Dict[str, Any]:
    """Best-effort OS-level listener census. Never raises; degrades loudly.

    Windows only, on purpose: the duplicate-bind hijack this defends against
    was MEASURED on Windows (two processes both LISTENING on one 127.0.0.1
    port with allow_reuse_address = 1). No claim is made here about other
    platforms, and on them the gate simply reports `supported: false`.
    """
    if sys.platform != "win32":
        return {"supported": False,
                "reason": f"OS listener probe is implemented for Windows only "
                          f"(this is {sys.platform!r})",
                "pids": {}}
    try:
        proc = subprocess.run(["netstat", "-ano", "-p", "TCP"],
                              capture_output=True, text=True,
                              timeout=timeout_s)
    except (OSError, subprocess.SubprocessError) as e:
        return {"supported": False, "reason": f"netstat unavailable: {e!r}",
                "pids": {}}
    if proc.returncode != 0:
        return {"supported": False,
                "reason": f"netstat exited {proc.returncode}", "pids": {}}
    return {"supported": True, "reason": "netstat -ano -p TCP",
            "pids": {str(p): parse_netstat_listeners(proc.stdout, p)
                     for p in ports}}


# ══════════════════════════════════════════════════════════════════════
# INTEGRITY GATE 3 -- ENGINE.  `enabled: true` is NOT "the cloud answered".
#
# GET /usage answers {"enabled": false} exactly when the bridge has NO relay
# (omnilink_mobile_bridge.py _route_get: `if relay is None`), i.e. the offline
# regex router is what answers /prompt. That much has always been checked.
#
# What was NOT checked, and is the reason a run can be mislabelled: an
# OllamaRelay reports `enabled: true` too. The two write DIFFERENT shapes into
# `latest`, and that is the whole discriminator:
#
#   OllamaRelay        relay.py:1981-1989 -> {"engine": "ollama:<model>",
#                      "text": "local <model>: N in / M out tok, ..."}
#   OllamaRelay        relay.py:1867-1868 -> {"engine": "<cloud engine>",
#     (cloud fallback)   "text": "cloud fallback via <cloud engine>"}
#   OmniLinkRelay      relay.py:1082-1085 -> UsageDelta.to_dict() + "text":
#                      UsageDelta.report(), i.e. the platform rollup
#                      "window=..s tokens=.. -> .. credits/hour" and NO
#                      "engine" key at all.
#
# Two asymmetries that matter and are easy to get backwards:
#   * `latest` is None until a chat turn COMPLETES. New bridges publish a
#     separate construction-time `relay` identity, so preflight can verify
#     the configured relay without pretending a metered turn already ran.
#     Old bridges without that field remain unverified.
#   * OMNILINK_USAGE=0 sets USAGE_ENABLED_DEFAULT False (relay.py:277), which
#     leaves OmniLinkRelay._meter None and therefore _last_usage None FOREVER
#     -- while OllamaRelay writes _last_usage directly and is unaffected.
#     Relay identity closes that observability gap without exposing a key.
# ══════════════════════════════════════════════════════════════════════

ENGINE_NONE = "none"                        # offline regex router
ENGINE_LOCAL_OLLAMA = "local_ollama"
ENGINE_CLOUD = "cloud_omnilink"
ENGINE_CLOUD_FALLBACK = "cloud_via_ollama_fallback"
ENGINE_UNVERIFIED = "unverified"

# The platform rollup, as produced by omnilink.usage_meter.UsageDelta.report().
_CLOUD_ROLLUP_RE = re.compile(
    r"(window=\s*\d|usage:\s*window too short|credits/hour|tokens/hour)",
    re.IGNORECASE)


def classify_engine(usage: Any) -> Dict[str, Any]:
    """Which chat engine actually answered, from one GET /usage body.

    Pure. Returns {engine_class, engine, evidence, cloud: bool|None}. `cloud`
    is None whenever the answer is genuinely unknown -- never False-by-default,
    because "we could not tell" and "it was not the cloud" are different
    claims and only one of them is provable here.
    """
    if not isinstance(usage, dict):
        return {"engine_class": ENGINE_UNVERIFIED, "engine": None,
                "cloud": None,
                "evidence": "no GET /usage object came back from this bridge"}
    if not usage.get("enabled"):
        return {"engine_class": ENGINE_NONE, "engine": None, "cloud": False,
                "evidence": "GET /usage reports enabled=false -- this bridge "
                            "has NO relay, so the offline regex router is "
                            "what answers /prompt"}
    identity = usage.get("relay")
    identity = identity if isinstance(identity, dict) else {}
    relay_kind = str(identity.get("kind") or "").lower()
    relay_engine = identity.get("engine")
    relay_model = identity.get("model")

    def identity_verdict() -> Optional[Dict[str, Any]]:
        # This is construction-time evidence, not an inference from a metered
        # turn. It closes the preflight/OMNILINK_USAGE=0 observability gap
        # without confusing "relay exists" with "latest turn used cloud".
        if relay_kind == "omnilink":
            return {
                "engine_class": ENGINE_CLOUD,
                "engine": str(relay_engine) if relay_engine is not None else None,
                "cloud": True,
                "evidence": "GET /usage.relay explicitly identifies the "
                            "constructed relay as OmniLinkRelay",
            }
        if relay_kind == "ollama":
            engine_name = (f"ollama:{relay_model}"
                           if relay_model is not None else None)
            return {
                "engine_class": ENGINE_LOCAL_OLLAMA,
                "engine": engine_name,
                "cloud": False,
                "evidence": "GET /usage.relay explicitly identifies the "
                            "constructed relay as OllamaRelay",
            }
        return None

    latest = usage.get("latest")
    if not isinstance(latest, dict):
        explicit = identity_verdict()
        if explicit is not None:
            return explicit
        return {"engine_class": ENGINE_UNVERIFIED, "engine": None,
                "cloud": None,
                "evidence": "a relay is live (enabled=true) but /usage.latest "
                            "is null: either no chat turn has completed yet, "
                            "or OMNILINK_USAGE=0 suppressed the cloud relay's "
                            "meter entirely. Null does NOT mean 'no relay' and "
                            "does NOT mean 'cloud'."}
    engine = latest.get("engine")
    engine_s = str(engine) if engine is not None else ""
    text_s = str(latest.get("text") or "")
    if engine_s.lower().startswith("ollama:"):
        return {"engine_class": ENGINE_LOCAL_OLLAMA, "engine": engine_s,
                "cloud": False,
                "evidence": f"/usage.latest.engine = {engine_s!r} -- that "
                            f"prefix is written only by OllamaRelay, so this "
                            f"turn ran on a LOCAL model"}
    if text_s.lower().startswith("cloud fallback via "):
        return {"engine_class": ENGINE_CLOUD_FALLBACK, "engine": engine_s or None,
                "cloud": True,
                "evidence": f"/usage.latest.text = {text_s!r} -- the platform "
                            f"WAS reached, but by an OllamaRelay falling back; "
                            f"this bridge's ordinary turns run on a local model"}
    if text_s.lower().startswith("local "):
        return {"engine_class": ENGINE_LOCAL_OLLAMA, "engine": engine_s or None,
                "cloud": False,
                "evidence": f"/usage.latest.text = {text_s!r} -- the "
                            f"'local <model>: ...' shape is written only by "
                            f"OllamaRelay"}
    if not engine_s and _CLOUD_ROLLUP_RE.search(text_s):
        return {"engine_class": ENGINE_CLOUD, "engine": None, "cloud": True,
                "evidence": f"/usage.latest carries the platform usage rollup "
                            f"and no engine key ({text_s!r}) -- that shape is "
                            f"written only by OmniLinkRelay via the platform "
                            f"UsageMeter"}
    explicit = identity_verdict()
    if explicit is not None:
        return explicit
    return {"engine_class": ENGINE_UNVERIFIED, "engine": engine_s or None,
            "cloud": None,
            "evidence": f"a relay is live and published usage, but the shape "
                        f"matches neither the OllamaRelay nor the OmniLink "
                        f"platform signature (engine={engine!r}, "
                        f"text={text_s[:120]!r})"}


def engine_gate(mode: str, per_bridge: Dict[str, Dict[str, Any]],
                stage: str) -> Dict[str, Any]:
    """Does the classified engine contradict the asserted --mode?

    Pure. `fatal` messages refuse the run; `warnings` do not. The split is the
    point: a POSITIVE identification of the wrong engine is provable and is
    refused, while "could not tell" is recorded as `unverified` and never
    dressed up as either answer.
    """
    classes = {n: e.get("engine_class") for n, e in per_bridge.items()
               if not n.startswith("_")}
    fatal: List[str] = []
    warnings: List[str] = []
    if not classes:
        return {"stage": stage, "mode": mode, "verdict": ENGINE_UNVERIFIED,
                "fatal": [], "per_bridge": per_bridge,
                "warnings": ["no bridge answered GET /usage -- the declared "
                             "--mode is entirely unverified"]}

    def named(*want: str) -> List[str]:
        return sorted(n for n, c in classes.items() if c in want)

    if mode == "omnilink":
        offline = named(ENGINE_NONE)
        if offline:
            fatal.append(
                f"--mode omnilink, but {', '.join(offline)} report "
                f"GET /usage enabled=false: those prompts were answered by "
                f"the OFFLINE REGEX ROUTER, not by an LLM. The usual cause is "
                f"launching the world in a way that puts an interpreter "
                f"without `omnilink` installed on PATH -- see "
                f"BENCH_OMNILINK.md section 2 ('why not launch.bat').")
        local = named(ENGINE_LOCAL_OLLAMA)
        if local:
            fatal.append(
                f"--mode omnilink, but {', '.join(local)} published a LOCAL "
                f"OLLAMA usage signature. `enabled: true` is necessary but not "
                f"sufficient -- an OllamaRelay reports it too. Set "
                f"OMNISIM_OLLAMA=0 (or OMNILINK_ENGINE=<cloud engine>) before "
                f"launching the world, or relabel this run.")
        fb = named(ENGINE_CLOUD_FALLBACK)
        if fb:
            warnings.append(
                f"{', '.join(fb)} reached the platform via an OllamaRelay "
                f"CLOUD FALLBACK. The turn did go to OmniLink, but this bridge "
                f"is running the hybrid relay, so its ordinary turns are "
                f"local. Do not report this as a clean --mode omnilink run.")
        unk = named(ENGINE_UNVERIFIED)
        if unk and not fatal:
            warnings.append(
                f"could not confirm the CLOUD relay on {', '.join(unk)}: "
                f"/usage.latest is null or an unrecognised shape. Recorded as "
                f"engine_verified=unverified -- do not quote this run as a "
                f"verified OmniLink result.")
    elif mode == "local":
        offline = named(ENGINE_NONE)
        if offline:
            fatal.append(
                f"--mode local, but {', '.join(offline)} report "
                f"GET /usage enabled=false: those prompts were answered by "
                f"the OFFLINE REGEX ROUTER, not by a local LLM. Start Ollama "
                f"before launching the world and leave OMNI_KEY unset, or "
                f"relabel this run.")
        cloud = named(ENGINE_CLOUD, ENGINE_CLOUD_FALLBACK)
        if cloud:
            fatal.append(
                f"--mode local, but {', '.join(cloud)} published an OmniLink "
                f"CLOUD usage signature. This condition exists to isolate "
                f"generic local-LLM tool-calling value from OmniLink-specific "
                f"value. Leave OMNI_KEY unset and restart the world, or "
                f"relabel this run.")
        unk = named(ENGINE_UNVERIFIED)
        if unk and not fatal:
            warnings.append(
                f"could not confirm the LOCAL Ollama relay on "
                f"{', '.join(unk)}: /usage.latest is null or an unrecognised "
                f"shape. Recorded as engine_verified=unverified -- do not "
                f"quote this run as a verified local-LLM control.")
    elif mode == "offline":
        live = named(ENGINE_LOCAL_OLLAMA, ENGINE_CLOUD, ENGINE_CLOUD_FALLBACK)
        if live:
            fatal.append(
                f"--mode offline, but {', '.join(live)} published an LLM usage "
                f"signature: an RELAY answered these prompts, not the regex "
                f"router. Restart the world with OMNI_KEY unset AND "
                f"OMNISIM_OLLAMA=0, or relabel this run.")
        maybe = named(ENGINE_UNVERIFIED)
        if maybe and not fatal:
            warnings.append(
                f"--mode offline, but {', '.join(maybe)} report a LIVE relay "
                f"(enabled=true) whose engine could not be classified. If any "
                f"prompt reached an LLM this run is mislabelled.")
    else:  # none -- no prompts are issued, so the chat layer is not exercised
        live = named(ENGINE_LOCAL_OLLAMA, ENGINE_CLOUD, ENGINE_CLOUD_FALLBACK,
                     ENGINE_UNVERIFIED)
        if live:
            warnings.append(
                f"--mode none with a live relay on {', '.join(live)}. Not an "
                f"error: mode none issues NO prompts, so the chat layer never "
                f"runs. Noted so the label is not read as 'no relay present'.")

    if mode == "omnilink" and not os.environ.get("OMNI_KEY"):
        warnings.append(
            "OMNI_KEY is not set in THIS shell. That does not prove the "
            "controllers lack it (they inherit the environment of whatever "
            "launched the simulator), but it is worth checking.")

    if fatal:
        verdict = "contradicted"
    elif mode == "omnilink":
        verdict = ("verified"
                   if classes and all(c == ENGINE_CLOUD
                                      for c in classes.values())
                   else ENGINE_UNVERIFIED)
    elif mode == "local":
        verdict = ("verified"
                   if classes and all(c == ENGINE_LOCAL_OLLAMA
                                      for c in classes.values())
                   else ENGINE_UNVERIFIED)
    elif mode == "offline":
        verdict = ("verified"
                   if classes and all(c == ENGINE_NONE
                                      for c in classes.values())
                   else ENGINE_UNVERIFIED)
    else:
        verdict = "not_applicable"
    return {"stage": stage, "mode": mode, "verdict": verdict, "fatal": fatal,
            "warnings": warnings, "per_bridge": per_bridge,
            "classes": classes}


# ══════════════════════════════════════════════════════════════════════
# Mode probing
# ══════════════════════════════════════════════════════════════════════

def probe_modes(recs: Dict[str, "ml.Recorder"], timeout: float, token: str
                ) -> Dict[str, Any]:
    """What the bridges say about their own chat layer, read-only.

    `GET /usage` answers `{"enabled": false}` exactly when the bridge has NO
    relay -- i.e. the offline regex router is what answers /prompt. On top of
    that flag every bridge's `latest` is classified (see classify_engine) so
    "a relay is live" and "the CLOUD relay is live" stop being the same claim.
    """
    out: Dict[str, Any] = {}
    for name, rec in recs.items():
        entry: Dict[str, Any] = {}
        try:
            u = ml.get_json(rec.url + "/usage", timeout, token)
            if isinstance(u, dict):
                entry["relay_enabled"] = bool(u.get("enabled"))
                entry["usage_seen"] = u.get("latest") is not None
            entry.update(classify_engine(u))
        except ml.BridgeError as e:
            entry["relay_enabled"] = None
            entry["error"] = str(e)
            entry.update(classify_engine(None))
        out[name] = entry
    vals = [v.get("relay_enabled") for v in out.values()
            if v.get("relay_enabled") is not None]
    out["_consensus"] = (all(vals) if vals and all(v is not None for v in vals)
                         else None) if vals else None
    out["_any_relay"] = any(vals) if vals else None
    out["_all_relay"] = all(vals) if vals else None
    return out


def mode_warnings(mode: str, probed: Dict[str, Any]) -> List[str]:
    """Non-fatal half of the engine gate, kept for callers that only want the
    advisory text. The refusing half lives in engine_gate()."""
    per = {k: v for k, v in probed.items() if not k.startswith("_")}
    gate = engine_gate(mode, per, stage="advisory")
    return list(gate["fatal"]) + list(gate["warnings"])


# ══════════════════════════════════════════════════════════════════════
# Report
# ══════════════════════════════════════════════════════════════════════

def score_probes(probes: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    counts: Dict[str, int] = {}
    for p in probes:
        counts[str(p.get("verdict"))] = counts.get(str(p.get("verdict")), 0) + 1
    scored = [p for p in probes if p.get("verdict") in ("pass", "fail")]
    passed = [p for p in scored if p["verdict"] == "pass"]
    ans = [p for p in probes if isinstance(p.get("text_check"), dict)
           and p["text_check"].get("match") is not None]
    ans_ok = [p for p in ans if p["text_check"]["match"]]
    lat = [float(p["latency_s"]) for p in probes
           if isinstance(p.get("latency_s"), (int, float))]
    by_tier: Dict[str, Dict[str, int]] = {}
    for p in probes:
        b = by_tier.setdefault(str(p.get("tier")), {"pass": 0, "fail": 0,
                                                    "other": 0})
        v = str(p.get("verdict"))
        b[v if v in ("pass", "fail") else "other"] += 1
    return {
        "probes_run": len(probes),
        "verdicts": counts,
        "state_score": {
            "scored": len(scored), "passed": len(passed),
            "rate": (len(passed) / len(scored)) if scored else None,
            "note": "state-measured verdicts only; inconclusive/error excluded",
        },
        "answer_accuracy_SECONDARY": {
            "checked": len(ans), "correct": len(ans_ok),
            "rate": (len(ans_ok) / len(ans)) if ans else None,
            "note": "NOT part of the state score. Ground truth read from the "
                    "bridges' own /state; recorded because a pure question "
                    "has no robot-state consequence to measure.",
        },
        "by_tier": by_tier,
        "prompt_latency_s": ml.describe(lat),
    }


def build_report(args: argparse.Namespace, recs: Dict[str, "ml.Recorder"],
                 probes: List[Dict[str, Any]], phases: Dict[str, Any],
                 probed_modes: Dict[str, Any], started_utc: str,
                 warnings: List[str], notes: List[str],
                 calibration: Optional[Dict[str, Any]],
                 aborted: Optional[str], hz: float,
                 integrity: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    windows: Dict[str, Any] = {}
    for name, (t0, t1) in phases.items():
        if t1 > t0:
            windows[name] = window_metrics(recs, t0, t1, hz)

    disc = [{"key": p["key"], "target": p["target_bridge"],
             "text": p["text"], "verdict": p.get("verdict"),
             **(p.get("discriminator") or {})}
            for p in probes if p.get("predicate") == "resumed"]

    cost = {}
    b = windows.get("baseline") or {}
    i = windows.get("intervention") or {}
    if b.get("boxes_per_minute") is not None and i.get("boxes_per_minute") is not None:
        cost["boxes_per_minute_baseline"] = b["boxes_per_minute"]
        cost["boxes_per_minute_intervention"] = i["boxes_per_minute"]
        if b["boxes_per_minute"]:
            cost["throughput_ratio"] = (i["boxes_per_minute"]
                                        / b["boxes_per_minute"])
    if i.get("arm_paused_by_operator_s") is not None:
        cost["arm_paused_by_operator_s_intervention"] = \
            i["arm_paused_by_operator_s"]
        cost["arm_paused_by_operator_frac_intervention"] = \
            i.get("arm_paused_by_operator_frac")
    cost["EXPECTED"] = ("A lower boxes/min during the intervention window is "
                        "the PRICE of talking to a running line, not a defect: "
                        "every operator command pauses that robot's idle loop "
                        "for the quiet window. The autonomous line needs no "
                        "LLM, so no mode can make it faster.")

    return {
        "schema": SCHEMA,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "run": {
            "started_at_utc": started_utc,
            "mode_declared": args.mode,
            "label": args.label,
            "duration_requested_s": args.duration,
            "poll_hz": args.hz,
            "verify_hz": args.verify_hz,
            "resume_s_assumed": args.resume_s,
            "resume_s_source": ("measured" if (calibration or {}).get(
                "measured_resume_s") else "asserted via --resume-s"),
            "tool_grace_s": args.tool_grace_s,
            "timer_tol_s": args.timer_tol_s,
            "tug_robot": args.tug_robot,
            "arm_robot": args.arm_robot,
            "aborted": aborted,
            "omni_key_present_in_harness_env": bool(os.environ.get("OMNI_KEY")),
            "secrets": "OMNI_KEY is read from the environment only. It is "
                       "never sent, logged or written by this harness, and "
                       "every recorded string is passed through a redactor.",
        },
        "mode_probe": probed_modes,
        # The measurement measuring itself. Read this BEFORE any number below
        # it: a run whose bridges were orphaned, swapped or answered by the
        # wrong engine produces perfectly formatted nonsense.
        "integrity": integrity or {
            "note": "integrity gates did not run (older invocation path)"},
        "calibration": calibration,
        "suite_sha256": suite_fingerprint(),
        "probes": probes,
        "scores": score_probes(probes),
        "discriminator": disc,
        "throughput": windows,
        "throughput_cost": cost,
        "phases": {k: {"t_start_s": round(v[0], 2), "t_end_s": round(v[1], 2)}
                   for k, v in phases.items()},
        "bridges": {name: {"url": rec.url, "id": rec.meta.get("id"),
                           "model": rec.meta.get("model"),
                           "polls_ok": len(rec.t),
                           "polls_failed": len(rec.failures)}
                    for name, rec in recs.items()},
        "warnings": warnings,
        "notes": notes,
    }


def suite_fingerprint() -> str:
    """So two result files can be shown to have run the SAME experiment."""
    payload = json.dumps(
        [{k: p.get(k) for k in ("key", "target", "text", "predicate",
                                "params", "verify_s", "setup")}
         for p in SUITE], sort_keys=True).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()[:16]


# ══════════════════════════════════════════════════════════════════════
# Human summary
# ══════════════════════════════════════════════════════════════════════

_VERDICT_MARK = {"pass": "PASS", "fail": "FAIL", "inconclusive": "INCON",
                 "error": "ERROR"}


def print_summary(rep: Dict[str, Any], out: Any = sys.stdout) -> None:
    try:
        out.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    p = lambda *a: print(*a, file=out)                        # noqa: E731
    f = ml._f
    run = rep["run"]
    p("")
    p("=" * 78)
    p(f"  WAREHOUSE CHAT-LAYER BENCH -- mode declared: {run['mode_declared'].upper()}"
      f"{'  [' + run['label'] + ']' if run.get('label') else ''}")
    p("=" * 78)
    p(f"  suite {rep['suite_sha256']}   resume_s {f(run['resume_s_assumed'])}"
      f" ({run['resume_s_source']})")

    mp = rep.get("mode_probe") or {}
    relay = ", ".join(f"{k}={v.get('relay_enabled')}"
                      for k, v in mp.items() if not k.startswith("_"))
    p(f"  relay live (GET /usage): {relay}")

    # INTEGRITY FIRST. Everything below it is conditional on these three.
    integ = rep.get("integrity") or {}
    live = integ.get("liveness") or {}
    ident = integ.get("identity") or {}
    eng = integ.get("engine") or {}
    if live or ident or eng:
        p("")
        p("  INTEGRITY GATES (is this measurement about the running world?)")
        lv = ", ".join(f"{k}={v.get('verdict')}"
                       for k, v in (live.get("preflight") or {}).items()
                       if not k.startswith("_"))
        p(f"    liveness (tick loop advancing) : {lv or 'not run'}")
        idv = ", ".join(f"{k}={v.get('verdict')}"
                        for k, v in (ident.get("per_bridge") or {}).items())
        p(f"    identity (same process, start->end): {idv or 'not run'}")
        p(f"    port listeners: {ident.get('pid_summary', 'not probed')}")
        if not ((ident.get("pid_probe_first") or {}).get("supported")):
            p("      (no OS pid fingerprint on this run: "
              f"{(ident.get('pid_probe_first') or {}).get('reason')}. "
              "'stable' below means")
            p("       nothing contradicted continuity -- NOT that continuity "
              "was proven.)")
        p(f"    engine verified vs --mode      : {eng.get('verdict', 'n/a')}")
        for k, v in (eng.get("per_bridge") or {}).items():
            if k.startswith("_"):
                continue
            p(f"      {k:6s} {str(v.get('engine_class')):26s} "
              f"{str(v.get('evidence'))[:96]}")
        for f_ in (eng.get("fatal") or []):
            p(f"    !! {f_}")
        for b in (ident.get("per_bridge") or {}).values():
            for note in (b.get("findings") or []):
                p(f"    !! {b.get('bridge')}: {note}")

    sc = rep.get("scores") or {}
    ss = sc.get("state_score") or {}
    aa = sc.get("answer_accuracy_SECONDARY") or {}
    p("")
    p("  SCORES")
    p(f"    state score (MEASURED ROBOT STATE)   "
      f"{ss.get('passed')}/{ss.get('scored')}"
      f"   {f(100.0 * (ss.get('rate') or 0), '.0f')}%")
    p(f"    answer accuracy (SECONDARY, not the score)  "
      f"{aa.get('correct')}/{aa.get('checked')}")
    lat = sc.get("prompt_latency_s") or {}
    p(f"    prompt latency  n={lat.get('n')}  mean {f(lat.get('mean'), '.2f')}s"
      f"  median {f(lat.get('median'), '.2f')}s  max {f(lat.get('max'), '.2f')}s")

    p("")
    p("  PER-PROMPT (verdict from state; reply shown as evidence only)")
    for pr in rep.get("probes") or []:
        mark = _VERDICT_MARK.get(str(pr.get("verdict")), "?")
        p(f"    [{mark:5s}] {pr['key']:<26s} {pr['tier']:<16s} "
          f"{pr['target_bridge']:<6s} {f(pr.get('latency_s'), '.1f')}s")
        p(f"             prompt : \"{pr['text']}\"")
        p(f"             measured: {pr.get('reason', '')}")
        subs = [s for s in (pr.get("subgoals") or [])
                if not s.get("informational")]
        if subs:
            p("             subgoals: " + "  ".join(
                f"{s['name']}={'ok' if s['ok'] else 'NO'}"
                f"({s['measured']} vs {s['expected']}+-{s['tol']})"
                for s in subs))
        tc = pr.get("text_check")
        if tc:
            p(f"             answer  : match={tc.get('match')} "
              f"truth={tc.get('truth_at_ask')} found={tc.get('found')} "
              f"(SECONDARY)")
        rep_txt = (pr.get("reply") or "").replace("\n", " ")
        if rep_txt:
            p(f"             said    : \"{rep_txt[:96]}"
              f"{'...' if len(rep_txt) > 96 else ''}\"")

    disc = rep.get("discriminator") or []
    p("")
    p("  DISCRIMINATOR -- time to resume, and WHAT caused it")
    if not disc:
        p("    (no resume probes ran)")
    for d in disc:
        p(f"    {d['key']:<26s} {_VERDICT_MARK.get(str(d.get('verdict')), '?'):<5s} "
          f"t_resume {f(d.get('time_to_resume_s'), '.1f'):>6s}s  "
          f"cause={str(d.get('cause')):<12s} "
          f"(timer would fire at ~{f(d.get('timer_due_s'), '.0f')}s)")
        if d.get("detail"):
            p(f"      {d['detail']}")

    p("")
    p("  THROUGHPUT COST (boxes/min -- lower under interaction is EXPECTED)")
    p(f"    {'window':<14s} {'s':>7s} {'boxes/min':>10s} {'fills/min':>10s} "
      f"{'picks/min':>10s} {'arm paused s':>13s} {'rt':>6s}")
    for name, w in (rep.get("throughput") or {}).items():
        if w.get("error"):
            p(f"    {name:<14s} {w.get('error')}")
            continue
        p(f"    {name:<14s} {f(w.get('window_s'), '.0f'):>7s} "
          f"{f(w.get('boxes_per_minute'), '.3f'):>10s} "
          f"{f(w.get('fills_per_minute'), '.3f'):>10s} "
          f"{f(w.get('picks_per_minute'), '.3f'):>10s} "
          f"{f(w.get('arm_paused_by_operator_s'), '.1f'):>13s} "
          f"{f(w.get('realtime_factor'), '.2f'):>6s}")
    tc = rep.get("throughput_cost") or {}
    if tc.get("throughput_ratio") is not None:
        p(f"    intervention / baseline = {f(tc['throughput_ratio'], '.3f')}x")

    warn = rep.get("warnings") or []
    p("")
    if warn:
        p("  WARNINGS")
        for w in warn:
            p(f"    ! {w}")
    else:
        p("  no warnings")
    p("=" * 78)
    p("")


# ══════════════════════════════════════════════════════════════════════
# --compare
# ══════════════════════════════════════════════════════════════════════

def do_compare(paths: Sequence[str], out: Any = sys.stdout) -> int:
    try:
        out.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    reps: List[Dict[str, Any]] = []
    for path in paths:
        try:
            with open(path, "r", encoding="utf-8") as fh:
                r = json.load(fh)
        except (OSError, ValueError) as e:
            print(f"error: cannot read {path}: {e}", file=sys.stderr)
            return EXIT_COMPARE_READ
        if r.get("schema") != SCHEMA:
            print(f"error: {path} is not a {SCHEMA} file "
                  f"(found {r.get('schema')!r})", file=sys.stderr)
            return EXIT_COMPARE_READ
        r["_path"] = path
        reps.append(r)

    p = lambda *a: print(*a, file=out)                        # noqa: E731
    f = ml._f
    cols = [str((r.get("run") or {}).get("mode_declared", "?")) for r in reps]
    w = max(12, max(len(c) for c in cols) + 2)
    p("")
    p("=" * (26 + w * len(reps)))
    p("  BENCH_OMNILINK COMPARISON")
    p("=" * (26 + w * len(reps)))
    p(f"  {'mode declared':<24s}" + "".join(f"{c:>{w}s}" for c in cols))

    fps = {r.get("suite_sha256") for r in reps}
    if len(fps) > 1:
        p("  !! the runs did NOT use the same suite -- these columns are not "
          "comparable")
    p(f"  {'suite sha':<24s}" + "".join(
        f"{str(r.get('suite_sha256', '?'))[:10]:>{w}s}" for r in reps))

    def row(label: str, fn, spec: str = ".3f") -> None:
        p(f"  {label:<24s}" + "".join(f"{f(fn(r), spec):>{w}s}" for r in reps))

    def rowstr(label: str, fn) -> None:
        p(f"  {label:<24s}" + "".join(f"{str(fn(r)):>{w}s}" for r in reps))

    def win(r, name, key):
        return ((r.get("throughput") or {}).get(name) or {}).get(key)

    p("")
    p("  CONDITIONS")
    rowstr("relay live (all)", lambda r: (r.get("mode_probe") or {}).get("_all_relay"))

    def _integ(r, *path):
        cur: Any = r.get("integrity") or {}
        for k in path:
            if not isinstance(cur, dict):
                return None
            cur = cur.get(k)
        return cur

    def _gate(r, *path):
        v = _integ(r, *path)
        # "?" is NOT "fine": a result file with no integrity block predates the
        # gates and was never checked at all.
        return {None: "?", "not_applicable": "n/a"}.get(v, str(v))[:11]

    rowstr("engine verified", lambda r: _gate(r, "engine", "verdict"))
    rowstr("liveness gate", lambda r: _gate(r, "liveness", "verdict"))
    rowstr("identity gate", lambda r: _gate(r, "identity", "verdict"))
    rowstr("aborted", lambda r: bool((r.get("run") or {}).get("aborted")))
    row("resume_s assumed", lambda r: (r.get("run") or {}).get("resume_s_assumed"), ".0f")

    p("")
    p("  CAPABILITY (verdicts from measured robot state)")
    rowstr("state score",
           lambda r: (f"{((r.get('scores') or {}).get('state_score') or {}).get('passed')}"
                      f"/{((r.get('scores') or {}).get('state_score') or {}).get('scored')}"))
    row("state pass rate %",
        lambda r: 100.0 * ((((r.get("scores") or {}).get("state_score") or {})
                            .get("rate")) or 0.0), ".0f")
    rowstr("answer acc (SECONDARY)",
           lambda r: (f"{((r.get('scores') or {}).get('answer_accuracy_SECONDARY') or {}).get('correct')}"
                      f"/{((r.get('scores') or {}).get('answer_accuracy_SECONDARY') or {}).get('checked')}"))
    row("prompt latency mean s",
        lambda r: (((r.get("scores") or {}).get("prompt_latency_s") or {})
                   .get("mean")), ".2f")

    p("")
    p("  PER-PROMPT VERDICT")
    keys: List[str] = []
    for r in reps:
        for pr in r.get("probes") or []:
            if pr["key"] not in keys:
                keys.append(pr["key"])
    for k in keys:
        cells = []
        for r in reps:
            hit = next((pr for pr in (r.get("probes") or [])
                        if pr["key"] == k), None)
            cells.append(_VERDICT_MARK.get(str(hit.get("verdict")), "-")
                         if hit else "-")
        p(f"  {k:<24s}" + "".join(f"{c:>{w}s}" for c in cells))

    p("")
    p("  DISCRIMINATOR: time to resume (s) / cause")
    dkeys: List[str] = []
    for r in reps:
        for d in r.get("discriminator") or []:
            if d["key"] not in dkeys:
                dkeys.append(d["key"])
    for k in dkeys:
        cells = []
        for r in reps:
            hit = next((d for d in (r.get("discriminator") or [])
                        if d["key"] == k), None)
            if not hit:
                cells.append("-")
            else:
                t = hit.get("time_to_resume_s")
                cells.append(f"{'--' if t is None else format(t, '.0f')}"
                             f"/{str(hit.get('cause'))[:9]}")
        p(f"  {k:<24s}" + "".join(f"{c:>{w}s}" for c in cells))

    p("")
    p("  THROUGHPUT COST")
    for wname in ("baseline", "intervention", "recovery", "whole_run"):
        if not any((r.get("throughput") or {}).get(wname) for r in reps):
            continue
        row(f"{wname} boxes/min", lambda r, n=wname: win(r, n, "boxes_per_minute"))
        row(f"{wname} window s", lambda r, n=wname: win(r, n, "window_s"), ".0f")
        row(f"{wname} realtime x", lambda r, n=wname: win(r, n, "realtime_factor"), ".2f")
    row("interv/baseline ratio",
        lambda r: (r.get("throughput_cost") or {}).get("throughput_ratio"))
    row("arm paused by op (s)",
        lambda r: win(r, "intervention", "arm_paused_by_operator_s"), ".1f")

    p("")
    p("  READ THIS BEFORE QUOTING THE TABLE")
    p("    * A lower boxes/min under interaction is EXPECTED, not a defect:")
    p("      the line is autonomous deterministic Python, so no chat mode can")
    p("      speed it up, and every operator command parks a robot for the")
    p("      quiet window.")
    p("    * Window lengths differ between modes (an LLM turn is slower than a")
    p("      regex match). Compare RATES, and check the window row above.")
    p("    * n is tiny and an LLM is nondeterministic. One run of this suite is")
    p("      an anecdote; repeat it before believing a difference.")
    p("    * CHECK THE INTEGRITY ROWS FIRST. A column whose engine/liveness/")
    p("      identity gate is not 'verified'/'alive'/'stable' may be about a")
    p("      different process than the one you launched -- see")
    p("      BENCH_OMNILINK.md, 'Known failure modes of the MEASUREMENT'.")
    p("=" * (26 + w * len(reps)))
    p("")
    return 0


# ══════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="bench_omnilink.py",
        description="Measure what each chat layer adds on the OmniLink "
                    "warehouse demo: offline regex, local Ollama, and the "
                    "OmniLink platform. Verdicts come from measured robot "
                    "state, never from reply text.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--mode", choices=("none", "offline", "local", "omnilink"),
                    default="offline",
                    help="the condition LABEL for this run. 'none' runs no "
                         "prompts at all (the throughput control); 'local' is "
                         "the Ollama LLM control needed to distinguish generic "
                         "LLM tool calling from OmniLink-specific value. This "
                         "does NOT switch the bridges' chat mode -- that is "
                         "chosen when the controllers start.")
    ap.add_argument("--duration", type=float, default=900.0,
                    help="total wall-clock seconds; the intervention window "
                         "gets whatever is left after --baseline-s and "
                         "--settle-s")
    ap.add_argument("--baseline-s", type=float, default=180.0,
                    help="quiet observation BEFORE any prompt (the "
                         "within-run throughput reference)")
    ap.add_argument("--settle-s", type=float, default=120.0,
                    help="quiet observation AFTER the last prompt. Should "
                         "exceed --resume-s so every pause has expired.")
    ap.add_argument("--out", default=None, help="write machine JSON here")
    ap.add_argument("--label", default="", help="free-text tag stored in JSON")

    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--arm-port", type=int, default=8765)
    ap.add_argument("--tug-a-port", type=int, default=8766)
    ap.add_argument("--tug-b-port", type=int, default=8767)
    ap.add_argument("--tug-robot", choices=("tug_a", "tug_b"), default="tug_b",
                    help="which tug the suite's 'tug' probes target")
    ap.add_argument("--arm-robot", choices=("omniarm6",), default="omniarm6")

    ap.add_argument("--hz", type=float, default=2.0,
                    help="background throughput sampling rate per bridge")
    ap.add_argument("--verify-hz", type=float, default=5.0,
                    help="state polling rate inside a probe's verify window")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="per-request HTTP timeout for GETs and setup POSTs")
    ap.add_argument("--prompt-timeout", type=float, default=150.0,
                    help="HTTP timeout for POST /prompt (an LLM turn is slow; "
                         "the relay's own sync dispatch gives up at 90 s)")
    ap.add_argument("--token", default=os.environ.get("OMNISIM_BRIDGE_TOKEN", ""),
                    help="bearer token, if the bridges were started with one")

    ap.add_argument("--resume-s", type=float, default=60.0,
                    help="the controllers' --idle-resume-s. MUST match, or "
                         "the resume attribution is wrong. Use "
                         "--calibrate-resume to measure it instead.")
    ap.add_argument("--calibrate-resume", action="store_true",
                    help="measure the auto-resume timer before the run "
                         "(costs one pause-length of throughput)")
    ap.add_argument("--tool-grace-s", type=float, default=5.0,
                    help="an un-pause seen within this long after the reply "
                         "returned is attributed to a tool call")
    ap.add_argument("--timer-tol-s", type=float, default=8.0,
                    help="an un-pause within this of the quiet-window expiry "
                         "is attributed to the timer, not the agent")

    ap.add_argument("--post-reset-quiet-s", type=float, default=2.5,
                    help="wait after a setup /resume_autonomy. MUST exceed "
                         "the bridges' ~1.5 s post-resume exemption window or "
                         "the next prompt's pause is silently swallowed.")
    ap.add_argument("--post-setup-quiet-s", type=float, default=3.0,
                    help="wait after a setup /stop_robot")
    ap.add_argument("--pre-probe-gap-s", type=float, default=1.0,
                    help="gap between the two pre-probe pose reads used to "
                         "tell 'was it already stationary?'")
    ap.add_argument("--retry-busy", type=int, default=2,
                    help="retries when the bridge answers BUSY (409, or a "
                         "busy action inside a 200 /prompt reply)")
    ap.add_argument("--busy-backoff-s", type=float, default=6.0)
    ap.add_argument("--no-reset-between", dest="reset_between",
                    action="store_false", default=True,
                    help="skip the harness-owned /resume_autonomy between "
                         "probes (preconditions then drift between modes)")

    # ── integrity gates ──────────────────────────────────────────────
    ap.add_argument("--liveness-gap-s", type=float, default=1.2,
                    help="seconds between the two preflight /state reads used "
                         "to prove each bridge's tick loop is ADVANCING. A "
                         "frozen last_tick_at means you are talking to an "
                         "orphaned controller; the run is refused (exit 7).")
    ap.add_argument("--no-port-identity", dest="port_identity",
                    action="store_false", default=True,
                    help="skip the OS-level `netstat -ano` listener census. "
                         "That census is the only TRUE process fingerprint "
                         "available and the only way to see a duplicate bind "
                         "directly; without it the identity gate degrades to "
                         "clock/counter monotonicity.")
    ap.add_argument("--identity-max-rt", type=float, default=20.0,
                    help="sim-clock advance over wall time above this ratio is "
                         "FLAGGED (never fatal): normal under --mode=fast, but "
                         "also what a swap to an older process looks like")
    ap.add_argument("--allow-mode-mismatch", action="store_true",
                    help="downgrade the ENGINE gate (exit 9) to a warning. The "
                         "liveness and identity gates have no bypass -- they "
                         "are about whether the numbers describe the running "
                         "world at all.")

    ap.add_argument("--compare", nargs="+", metavar="JSON",
                    help="print a side-by-side table of result files and exit")
    ap.add_argument("--selftest", action="store_true",
                    help="run the pure predicate unit tests and exit "
                         "(no simulator, no network)")
    ap.add_argument("--print-suite", action="store_true",
                    help="print the intervention suite and exit")
    ap.add_argument("--quiet", action="store_true")
    return ap


def print_suite(out: Any = sys.stdout) -> None:
    p = lambda *a: print(*a, file=out)                        # noqa: E731
    p(f"\nINTERVENTION SUITE  (sha256 {suite_fingerprint()})  "
      f"{len(SUITE)} prompts, run in this order\n")
    for i, s in enumerate(SUITE, 1):
        p(f"{i:2d}. {s['key']}   [{s['tier']}]  -> {s['target']}")
        p(f"    prompt   : \"{s['text']}\"")
        p(f"    predicate: {s['predicate']}  {json.dumps(s.get('params') or {})}")
        p(f"    setup    : {s.get('setup')}   verify_s: "
          f"{s.get('verify_s') if s.get('verify_s') is not None else 'resume_s + 25'}")
        if s.get("text_check"):
            p(f"    answer   : secondary check vs "
              f"{s['text_check']['truth']} (never a verdict)")
        p(f"    offline  : {s.get('offline_expectation')}")
        p(f"    why      : {s['why']}")
        p("")


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.selftest:
        sys.path.insert(0, _HERE)
        import test_bench_omnilink
        return test_bench_omnilink.run_all()
    if args.print_suite:
        print_suite()
        return 0
    if args.compare:
        return do_compare(args.compare)

    if args.duration <= 0 or args.hz <= 0 or args.verify_hz <= 0:
        print("error: --duration, --hz and --verify-hz must be positive",
              file=sys.stderr)
        return EXIT_ARGS
    if args.post_reset_quiet_s < 1.6:
        print("error: --post-reset-quiet-s must exceed the bridges' ~1.5 s "
              "post-resume exemption window, or the next prompt's idle-loop "
              "pause is silently swallowed and every measurement after it is "
              "wrong.", file=sys.stderr)
        return EXIT_ARGS
    if args.resume_s <= args.tool_grace_s + 2 * args.timer_tol_s:
        print(f"error: --resume-s {args.resume_s} leaves no separable gap "
              f"between 'the agent resumed it' and 'the timer expired' "
              f"(needs > tool_grace {args.tool_grace_s} + 2*timer_tol "
              f"{args.timer_tol_s}).", file=sys.stderr)
        return EXIT_ARGS

    redact = Redactor()
    recs = {
        "omniarm6": ml.Recorder("omniarm6", f"http://{args.host}:{args.arm_port}"),
        "tug_a": ml.Recorder("tug_a", f"http://{args.host}:{args.tug_a_port}"),
        "tug_b": ml.Recorder("tug_b", f"http://{args.host}:{args.tug_b_port}"),
    }

    ports = {"omniarm6": args.arm_port, "tug_a": args.tug_a_port,
             "tug_b": args.tug_b_port}

    print("bench_omnilink: probing bridges ...", file=sys.stderr)
    try:
        pre_warn = ml.preflight(recs, args.timeout, args.token)
    except ml.BridgeError as e:
        print(f"\nPREFLIGHT FAILED\n{e}\n", file=sys.stderr)
        return EXIT_PREFLIGHT
    for name, rec in recs.items():
        print(f"  {name:6s} ok  {rec.meta.get('model')}  ({rec.url})",
              file=sys.stderr)

    # ── INTEGRITY GATE 1: LIVENESS.  Hard refusal, before anything else.
    # A 200 OK is not evidence that anything is stepping this robot. See the
    # gate's own section header for why an orphaned bridge can answer at all.
    print(f"bench_omnilink: liveness gate ({args.liveness_gap_s:.1f}s tick "
          f"check) ...", file=sys.stderr)
    liveness_pre = probe_liveness(recs, args.timeout, args.token,
                                  args.liveness_gap_s)
    for name in recs:
        v = liveness_pre[name]
        print(f"  {name:6s} {str(v.get('verdict')).upper():13s} "
              f"{v.get('detail')}", file=sys.stderr)
    frozen = [liveness_pre[n] for n in liveness_pre.get("_failed") or []]
    if frozen:
        print("\n" + liveness_remedy(frozen) + "\n", file=sys.stderr)
        return EXIT_LIVENESS
    # NOT fatal, but it must not pass unremarked: a bridge the gate could not
    # read is a bridge whose liveness was never established, and reporting
    # that as "alive" would be exactly the fabrication this gate prevents.
    liveness_unverified = list(liveness_pre.get("_unverifiable") or [])

    # ── INTEGRITY GATE 2 (open): fingerprint the process behind each port.
    ident_states = liveness_pre.get("_states") or {}
    ident_first = {n: identity_of(ident_states.get(n)) for n in recs}
    pid_probe_first = (listener_pids([ports[n] for n in recs])
                       if args.port_identity
                       else {"supported": False,
                             "reason": "disabled with --no-port-identity",
                             "pids": {}})
    pids_first = {n: (pid_probe_first.get("pids") or {}).get(str(ports[n]))
                  for n in recs} if pid_probe_first.get("supported") else \
                 {n: None for n in recs}
    t_identity_open = time.monotonic()
    if pid_probe_first.get("supported"):
        dup = {n: pids_first[n] for n in recs
               if pids_first[n] and len(pids_first[n]) > 1}
        if dup:
            print("\nIDENTITY GATE FAILED at preflight -- MORE THAN ONE "
                  "PROCESS IS LISTENING ON A BRIDGE PORT.\n", file=sys.stderr)
            for n, pl in dup.items():
                print(f"  {n:6s} port {ports[n]}  LISTENING pids {pl}",
                      file=sys.stderr)
            print("\n  ThreadingHTTPServer sets allow_reuse_address = 1, so on "
                  "Windows a NEW bridge\n  can bind a port an ORPHANED one "
                  "still holds. Which process answers a given\n  connection is "
                  "not something this harness controls, so the run would be "
                  "about\n  an unknown mixture of live and dead robots.\n\n"
                  "  Remedy: kill the stale processes (taskkill /PID <pid> /F "
                  "for each pid above,\n  and `taskkill /IM omnisim-bin.exe /F` "
                  "for any leftover simulator), confirm the\n  ports are free, "
                  "then relaunch -- see BENCH_OMNILINK.md section 2.\n",
                  file=sys.stderr)
            return EXIT_IDENTITY
    else:
        print(f"  port identity: {pid_probe_first.get('reason')} -- the "
              f"identity gate degrades to clock/counter monotonicity",
              file=sys.stderr)

    # ── INTEGRITY GATE 3 (opening half): what the /usage flag alone proves.
    # Legacy bridges expose only `latest`, which is null until a chat turn
    # completes. Current bridges also expose construction-time relay identity,
    # so they can be verified here; the post-prompt gate still checks the
    # turn-specific usage signature when one exists.
    probed_modes = probe_modes(recs, args.timeout, args.token)
    engine_pre = engine_gate(args.mode,
                             {n: probed_modes[n] for n in recs},
                             stage="preflight")
    warnings = list(pre_warn) + list(engine_pre.get("warnings") or [])
    if liveness_unverified:
        warnings.append(
            f"LIVENESS could not be established on "
            f"{', '.join(liveness_unverified)}: "
            + "; ".join(f"{n}={liveness_pre[n].get('verdict')} "
                        f"({liveness_pre[n].get('detail')})"
                        for n in liveness_unverified)
            + ". Those bridges are NOT known to be alive -- treat their "
              "numbers as unverified.")
    for w in warnings:
        print(f"  ! {w}", file=sys.stderr)
    if engine_pre.get("fatal"):
        for f_ in engine_pre["fatal"]:
            print(f"  !! {f_}", file=sys.stderr)
        if not args.allow_mode_mismatch:
            print("\nENGINE GATE FAILED at preflight -- refusing to produce a "
                  "mislabelled result.\nRe-run with --allow-mode-mismatch to "
                  "downgrade this to a warning.\n", file=sys.stderr)
            return EXIT_ENGINE
        warnings.append("ENGINE GATE was CONTRADICTED at preflight and only "
                        "bypassed by --allow-mode-mismatch: " +
                        " | ".join(engine_pre["fatal"]))

    started_utc = datetime.now(timezone.utc).isoformat()
    sampler = Sampler(recs, args.hz, args.timeout, args.token)
    runner = Runner(args, recs, sampler, redact)
    sampler.start()

    calibration: Optional[Dict[str, Any]] = None
    probes: List[Dict[str, Any]] = []
    phases: Dict[str, Tuple[float, float]] = {}
    aborted: Optional[str] = None
    notes: List[str] = []
    engine_post: Optional[Dict[str, Any]] = None
    engine_refused = False

    try:
        if args.calibrate_resume:
            print("bench_omnilink: calibrating the auto-resume timer ...",
                  file=sys.stderr)
            calibration = calibrate_resume_s(runner, "tug",
                                             args.resume_s * 2.5 + 20.0)
            m = calibration.get("measured_resume_s")
            if m:
                print(f"  measured resume_s = {m:.1f}s "
                      f"(was assuming {args.resume_s:.1f}s)", file=sys.stderr)
                args.resume_s = float(m)
            else:
                warnings.append(f"resume calibration failed: "
                                f"{calibration.get('error')} -- falling back "
                                f"to the asserted --resume-s")
            # Leave the line clean before the baseline window.
            for role in ROLES:
                runner.post(role, "/resume_autonomy", {})
            time.sleep(args.post_reset_quiet_s)

        # ── phase 1: quiet baseline ───────────────────────────────────
        t_base0 = runner.t()
        print(f"bench_omnilink: baseline {args.baseline_s:.0f}s (no prompts) "
              f"...", file=sys.stderr)
        time.sleep(args.baseline_s)
        phases["baseline"] = (t_base0, runner.t())

        # ── phase 2: the intervention suite ───────────────────────────
        t_int0 = runner.t()
        if args.mode == "none":
            hold = max(0.0, args.duration - args.baseline_s - args.settle_s)
            notes.append(f"--mode none: NO prompts issued. The intervention "
                         f"window is a {hold:.0f}s quiet hold so its "
                         f"throughput is directly comparable with the other "
                         f"modes' intervention windows.")
            print(f"bench_omnilink: mode=none, holding quiet for {hold:.0f}s "
                  f"...", file=sys.stderr)
            time.sleep(hold)
        else:
            for i, probe in enumerate(SUITE, 1):
                if not args.reset_between and probe.get("setup") == "resume":
                    probe = dict(probe, setup=None)
                print(f"  [{i}/{len(SUITE)}] {probe['key']} -> "
                      f"{probe['target']}: \"{probe['text'][:52]}\"",
                      file=sys.stderr)
                res = runner.run_probe(probe)
                probes.append(res)
                if not args.quiet:
                    print(f"        {str(res.get('verdict')).upper():<12s} "
                          f"{res.get('reason', '')[:110]}", file=sys.stderr)

                # ── ENGINE GATE, decisive half. `latest` is populated only
                # once a chat turn has COMPLETED, so this is the first moment
                # local-Ollama and the OmniLink platform are distinguishable.
                # Checked here rather than at the end so a mislabelled run is
                # abandoned after one probe instead of after fifteen minutes.
                if i == 1:
                    probed_after = probe_modes(recs, args.timeout, args.token)
                    probed_modes["_after_first_prompt"] = probed_after
                    engine_post = engine_gate(
                        args.mode, {n: probed_after[n] for n in recs},
                        stage="after_first_prompt")
                    for w in engine_post.get("warnings") or []:
                        if w in warnings:
                            continue   # the preflight gate already said it
                        warnings.append(w)
                        print(f"  ! {w}", file=sys.stderr)
                    if engine_post.get("fatal"):
                        for f_ in engine_post["fatal"]:
                            print(f"  !! {f_}", file=sys.stderr)
                        if args.allow_mode_mismatch:
                            warnings.append(
                                "ENGINE GATE was CONTRADICTED after the first "
                                "prompt and only bypassed by "
                                "--allow-mode-mismatch: " +
                                " | ".join(engine_post["fatal"]))
                        else:
                            engine_refused = True
                            aborted = ("ENGINE GATE: " +
                                       " | ".join(engine_post["fatal"]))
                            print("\nENGINE GATE FAILED after the first "
                                  "prompt -- abandoning the run rather than "
                                  "producing a mislabelled result. Partial "
                                  "JSON is still written.\n", file=sys.stderr)
                            break
        phases["intervention"] = (t_int0, runner.t())

        # ── cleanup: hand every robot back before measuring recovery ──
        if args.mode != "none":
            for role in ROLES:
                st, _b, err = runner.post(role, "/resume_autonomy", {})
                notes.append(f"cleanup /resume_autonomy -> {role}: HTTP {st}"
                             f"{' ' + redact(err) if err else ''}")
            time.sleep(args.post_reset_quiet_s)

        # ── phase 3: recovery ─────────────────────────────────────────
        # Skipped when the engine gate already refused the run: there is
        # nothing to recover from and nothing that a recovery window could
        # make quotable.
        if not engine_refused:
            t_rec0 = runner.t()
            print(f"bench_omnilink: settling {args.settle_s:.0f}s ...",
                  file=sys.stderr)
            time.sleep(args.settle_s)
            phases["recovery"] = (t_rec0, runner.t())

    except KeyboardInterrupt:
        aborted = "interrupted by the operator (Ctrl-C)"
        print("\nbench_omnilink: interrupted -- reporting what was collected",
              file=sys.stderr)
    except ml.BridgeError as e:
        aborted = f"bridge failure: {e}"
    finally:
        sampler.stop()
        sampler.join(timeout=5.0)

    if sampler.error:
        warnings.append(f"background sampler died: {sampler.error}")
    phases["whole_run"] = (0.0, runner.t())

    # ── INTEGRITY GATES, closing half.  Everything above was measured on
    # the assumption that the process behind each port stayed the same and
    # kept ticking. Verify it now instead of assuming it, so a mid-run
    # swap or a bridge that died at minute 3 cannot pass silently.
    print("bench_omnilink: closing integrity gates ...", file=sys.stderr)
    liveness_post = probe_liveness(recs, args.timeout, args.token,
                                   args.liveness_gap_s)
    post_states = liveness_post.get("_states") or {}
    ident_last = {n: identity_of(post_states.get(n)) for n in recs}
    pid_probe_last = (listener_pids([ports[n] for n in recs])
                      if args.port_identity
                      else {"supported": False,
                            "reason": "disabled with --no-port-identity",
                            "pids": {}})
    pids_last = ({n: (pid_probe_last.get("pids") or {}).get(str(ports[n]))
                  for n in recs} if pid_probe_last.get("supported")
                 else {n: None for n in recs})
    wall_elapsed = time.monotonic() - t_identity_open
    sim_scans = {n: scan_sim_monotonicity(recs[n].t, recs[n].sim)
                 for n in recs}
    identity_per = {
        n: compare_identity(n, ident_first[n], ident_last[n], wall_elapsed,
                            pids_first.get(n), pids_last.get(n),
                            args.identity_max_rt, sim_scans[n])
        for n in recs
    }
    identity_verdict = ("swapped"
                        if any(v["verdict"] == "swapped"
                               for v in identity_per.values())
                        else ("suspect"
                              if any(v["verdict"] == "suspect"
                                     for v in identity_per.values())
                              else "stable"))
    for v in identity_per.values():
        for note in v.get("findings") or []:
            warnings.append(f"IDENTITY {v['bridge']}: {note}")
    died = [liveness_post[n] for n in liveness_post.get("_failed") or []]
    for d in died:
        warnings.append(
            f"LIVENESS {d.get('bridge')}: the tick loop was advancing at "
            f"preflight and is FROZEN at the end ({d.get('detail')}). "
            f"Everything measured after it stopped is about a robot that was "
            f"no longer being stepped.")
    if died or (liveness_pre.get("_failed") or []):
        liveness_verdict = "frozen"
    elif liveness_unverified or (liveness_post.get("_unverifiable") or []):
        liveness_verdict = "unverified"
    else:
        liveness_verdict = "alive"

    engine_final = engine_post or engine_pre
    integrity = {
        "WHY": ("A 200 OK from a bridge port is not evidence that the "
                "simulator you launched is behind it. These three gates are "
                "the difference between a measurement and a coincidence -- "
                "read them before any number in this file."),
        "liveness": {
            "verdict": liveness_verdict,
            "gap_s": args.liveness_gap_s,
            "preflight": {n: liveness_pre[n] for n in recs},
            "closing": {n: liveness_post[n] for n in recs},
            "method": ("GET /state twice; /state.last_tick_at is stamped from "
                       "the controller's own tick loop, so a frozen value with "
                       "a 200 OK is an orphaned bridge or a paused sim"),
        },
        "identity": {
            "verdict": identity_verdict,
            "per_bridge": identity_per,
            "pid_probe_first": pid_probe_first,
            "pid_probe_last": pid_probe_last,
            "pid_summary": "; ".join(
                f"{n}:{identity_per[n].get('pid_note')}" for n in recs),
            "LIMITS": ("Only the OS pid comparison is a true process "
                       "fingerprint, and it is Windows-only and best effort. "
                       "The clock and counter checks are NECESSARY conditions "
                       "-- 'stable' means nothing contradicted continuity, not "
                       "that continuity was proven."),
        },
        "engine": engine_final,
        "engine_verified": (engine_final or {}).get("verdict"),
    }

    if len(recs["omniarm6"].t) < 2:
        print("\nerror: fewer than 2 OMNIARM6 samples collected; nothing to "
              "report.", file=sys.stderr)
        return EXIT_ABORTED

    for name in ("tug_a", "tug_b", "omniarm6"):
        if recs[name].failures:
            warnings.append(f"{name}: {len(recs[name].failures)} failed "
                            f"background poll(s)")
    incon = [p["key"] for p in probes
             if p.get("verdict") in ("inconclusive", "error")]
    if incon:
        warnings.append(f"{len(incon)} probe(s) produced no state verdict "
                        f"({', '.join(incon)}) -- they are excluded from the "
                        f"state score, not counted as failures")

    report = build_report(args, recs, probes, phases, probed_modes,
                          started_utc, warnings, notes + runner.notes,
                          calibration, aborted, args.hz, integrity)
    report = redact(report)
    print_summary(report)

    if args.out:
        path = os.path.abspath(args.out)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=2, default=str)
        print(f"wrote {path}", file=sys.stderr)

    # Exit-code precedence. An integrity failure invalidates the WHOLE run, so
    # it outranks an abort, which outranks a clean finish. Probe failures are
    # results and never change the code.
    if identity_verdict == "swapped":
        print("\nIDENTITY GATE FAILED: the process behind at least one bridge "
              "port is not the one this run started against. Do not quote any "
              "number in this file.", file=sys.stderr)
        return EXIT_IDENTITY
    if died:
        print("\nLIVENESS GATE FAILED at close: a bridge stopped ticking "
              "during the run.", file=sys.stderr)
        return EXIT_LIVENESS
    if engine_refused:
        print(f"\nRUN ABORTED: {aborted}", file=sys.stderr)
        return EXIT_ENGINE
    if aborted:
        print(f"\nRUN ABORTED: {aborted}", file=sys.stderr)
        return EXIT_ABORTED
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
