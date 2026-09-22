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

"""Execute what `interpret` parsed, on whichever bridge is asking.

One function, `route(bridge, text, surface)`, so a bridge adopts the new
interpreter in a single line instead of reimplementing a dispatch ladder. It
returns the bridges' usual reply shape --

    {"agent": "<what to say>", "tools": [(name, status, detail[, rule]), ...]}

-- or ``None``, meaning *this was not something to execute*: hand it to the
model. Returning None rather than guessing is the whole point; the keyword
ladder it replaces had no way to abstain.

THREE HONESTY RULES, ENFORCED HERE RATHER THAN TRUSTED TO PROMPTS
-----------------------------------------------------------------
1. **Say what the action RETURNED, never what it was asked to do.** Every
   reply is built from the bridge's own result dict -- `accepted`, `error`,
   `measured` -- so "Stopped" is only ever said when the bridge measured a
   stop. This mirrors the note in the mobile bridge's own stop handler: the
   regex router used to print "Stopping wheels." before anything was known.

2. **Name the slots that were dropped.** Bridges differ: a quadruped's
   `act_walk()` takes no distance. Rather than silently discarding "2
   metres", the reply says it was ignored.

3. **Never invent a referent.** AMBIGUOUS comes back as a question, not as a
   guess, and nothing is actuated.
"""
from __future__ import annotations

import inspect
import re
from typing import Any, Dict, List, Optional, Tuple

from . import gate as _gate
from . import interpret as _i

__all__ = ["route", "execute", "describe_state", "short_circuit",
           "parser_first_plan", "parser_first_run", "parser_first_window",
           "window_lines", "parser_stats", "reply_payload", "stamp_via"]

# tool -> (bridge method, kwargs builder). Names match the act_* API the
# bridges already expose; anything missing degrades to a spoken refusal.
_ADAPTERS: Dict[str, Tuple[str, Any]] = {
    "stop":            ("act_stop", lambda a: {}),
    "resume_autonomy": ("act_resume_autonomy", lambda a: {}),
    "reset_to_home":   ("act_reset_to_home", lambda a: {}),
    "drive_forward":   ("act_drive_forward", lambda a: {"distance": a.get("distance", 1.0)}),
    "turn":            ("act_turn", lambda a: {"angle_rad": a["angle_rad"]}),
    "drive_to":        ("act_drive_to", lambda a: {"tx": a["x"], "ty": a["y"]}),
    "set_velocity":    ("act_set_velocity", lambda a: {"linear": a["v"], "angular": a["w"]}),
    "attach_trolley":  ("act_attach_trolley", lambda a: {"def_name": a.get("trolley")}),
    "detach_trolley":  ("act_detach_trolley", lambda a: {}),
    "pick":            ("act_pick", lambda a: {"name": a.get("object")}),
    "open_gripper":    ("act_open_gripper", lambda a: {}),
    "close_gripper":   ("act_close_gripper", lambda a: {}),
    "wave":            ("act_wave", lambda a: {}),
    "sit":             ("act_sit", lambda a: {}),
    "stand":           ("act_stand", lambda a: {}),
    "walk":            ("act_walk", lambda a: {"distance": a.get("distance")}),
    # drone
    "takeoff":         ("act_takeoff", lambda a: {"altitude": a.get("altitude")}),
    "land":            ("act_land", lambda a: {}),
    "hover":           ("act_hover", lambda a: {}),
    "move_body":       ("act_move_body", lambda a: {
        "forward": a.get("forward"), "vertical": a.get("vertical")}),
}


def _call(bridge: Any, method: str, kwargs: Dict[str, Any],
          block: bool = False):
    """Call act_*, passing only the arguments it declares. -> (res, dropped).

    `block` asks the action to finish before returning. It matters for
    compound orders: "turn left 90 then drive forward 1 m" issued both at
    once and the drive came back `busy`, because the turn was still running.
    It is injected only when the action declares `wait`, and never counted
    as a dropped slot -- it is our sequencing, not the operator's words.
    """
    fn = getattr(bridge, method, None)
    if fn is None or not callable(fn):
        return None, []
    try:
        params = inspect.signature(fn).parameters
    except (TypeError, ValueError):  # pragma: no cover - builtins
        params = {}
    wanted = {k: v for k, v in kwargs.items() if v is not None}
    passed = {k: v for k, v in wanted.items() if k in params}
    dropped = [k for k in wanted if k not in params]
    if block and "wait" in params:
        passed["wait"] = True
    return fn(**passed), dropped


def _emit_gate_refusal(bridge: Any, rejection: Any, utterance: str) -> None:
    """File the gate's refusal on the bridge's event ring (plan D4).

    ORIGIN `prompt`, deliberately: this is the PARSER path, so the refusal
    is already in the reply the operator and the model are reading. The
    wake policy uses that to avoid buying a second model turn to tell the
    agent about a refusal it is already looking at (`events.may_wake`).

    Defensive on every read: an external bridge built against the v8
    package has no ring, and a refusal must not become a 500 because the
    telemetry it wanted to record was absent.
    """
    try:
        from .events import emit_gate_refusal as _emit
        _emit(getattr(bridge, "events", None),
              getattr(rejection, "tool", ""), getattr(rejection, "detail", ""),
              sim_time=float(getattr(bridge, "sim_time", 0.0) or 0.0),
              step=int(getattr(bridge, "sim_step", 0) or 0),
              robot=str(getattr(bridge, "robot_id", "") or ""),
              origin="prompt", rule=getattr(rejection, "rule", ""),
              utterance=utterance)
    except Exception:                                  # pragma: no cover
        pass


def _describe(tool: str, args: Dict[str, Any], res: Any,
              dropped: List[str]) -> Tuple[str, str, str, str]:
    """One honest sentence + a (status, detail, rule) triple, from the RESULT.

    The fourth element is PROTOCOL.md §5.7.2's `rule`: empty for anything
    that is not a refusal, and the machine-readable reason when it is.
    """
    if not isinstance(res, dict):
        return f"{tool} done.", "ok", "", ""

    # ⚠️ `error` is NOT a failure here. The bridges follow the measured-result
    # convention -- {commanded, achieved, error, settled} -- where `error` is
    # achieved minus commanded, a float, and 4.3e-11 means a perfect stop.
    # Reading it as a failure reported "I could not stop: 4.3e-11" after a
    # textbook stop. Only a non-empty STRING is a failure.
    err = res.get("error")
    if isinstance(err, str) and err.strip():
        return (f"I could not {tool.replace('_', ' ')}: {err}", "err", err,
                "")
    if res.get("accepted") is False:
        why = res.get("say") or res.get("reason") or "the bridge refused it"
        # PROTOCOL.md §5.7.2: the machine-readable half. The bridge's own
        # code when it gives one, never a slug cut out of the prose -- a
        # rule name derived from a sentence changes when the sentence does,
        # which is the exact property `summary` has and `rule` must not.
        rule = str(res.get("rule") or res.get("code") or "bridge_refused")
        return (f"I did not {tool.replace('_', ' ')}: {why}", "refused",
                str(why), rule)

    # A measured stop is the reference case: say the measurement, or say
    # plainly that it could not be confirmed. Never "Stopping wheels."
    measured = res.get("measured") or {}
    if tool == "stop":
        still = res.get("stationary")
        if still is True and "speed_mps" in measured:
            said = (f"Stopped - measured {measured['speed_mps']:.3f} m/s over "
                    f"{measured.get('over_s', 0):.2f} s, so it is standing still.")
            detail = f"stationary, {measured['speed_mps']:.3f} m/s"
        elif still is False and "speed_mps" in measured:
            said = (f"Motion commanded to zero, but it is STILL MOVING: "
                    f"{measured['speed_mps']:.3f} m/s.")
            detail = f"NOT stationary, {measured['speed_mps']:.3f} m/s"
        else:
            # Deliberately not "wheels": the same executor drives arms and
            # quadrupeds, and an arm reporting stopped wheels is a small lie.
            said = ("Motion commanded to zero. I could not confirm it came to "
                    "rest - " + str(measured.get("reason", "not measured")) + ".")
            detail = "rest unconfirmed"
        return said, "ok", detail, ""

    bits = ", ".join(f"{k}={v}" for k, v in args.items() if v is not None)
    said = f"{tool.replace('_', ' ').capitalize()}" + (f" ({bits})" if bits else "") + "."
    if dropped:
        said += (" I ignored " + ", ".join(dropped)
                 + " - this robot's " + tool + " does not take it.")
    return said, "ok", bits or "ok", ""


def _match_constraint_rule(intents: Any, text: str) -> Optional[str]:
    """Map "don't go into the pick-cell column" onto the store's own rule.

    IntentStore refuses any rule outside its closed set, and answers with the
    set. That refusal is honest but needlessly lossy when the operator named
    a rule that IS enforceable in different words. The vocabulary comes from
    the store, never from a list kept here, so a new rule needs no change.
    """
    import re
    known = getattr(intents, "rules", None)
    if not known:
        return None
    tokens = set(re.findall(r"[a-z]+", text.lower()))
    best, best_score = None, 0
    for name in known:
        parts = [p for p in re.split(r"[_\-]+", str(name).lower())
                 if p and p != "no"]
        if not parts:
            continue
        hit = sum(1 for p in parts
                  if p in tokens or p.rstrip("s") in tokens
                  or any(t.startswith(p.rstrip("s")) for t in tokens))
        if hit > best_score:
            best, best_score = str(name), hit
    # Two matching words, so a single incidental overlap cannot bind a rule.
    return best if best_score >= 2 else None


def describe_state(state: Optional[Dict[str, Any]]) -> str:
    """One honest English sentence describing a bridge /state dict.

    Reads ONLY what the dict actually contains -- every clause below is
    gated on its key being present -- so a parser-answered reply can
    never claim a job the robot is not doing. Understands the two shapes
    the OmniSim bridges publish (arm: gripper/line; mobile: carrying/tow
    legs) plus the `idle_loop` block they share.

    It lived in `intent_router.py` until 2026-09-22, alongside the
    keyword ladder that module existed for. The ladder is deleted; this
    is the part that answered a QUESTION rather than actuating anything,
    so it moved here, next to its one caller.
    """
    if not isinstance(state, dict):
        return "I have no state to report."

    who = str(state.get("id") or "this robot")
    bits: List[str] = []

    loop = state.get("idle_loop") or {}
    paused = bool(loop.get("paused"))
    line = state.get("line") or {}

    # ── What am I doing this instant ─────────────────────────────
    if line.get("active"):
        # Arm running as the line master: the box is the real answer.
        box = line.get("fill_box") or "no box"
        fill_state = str(line.get("fill_state") or "?")
        placed = line.get("placed")
        target = line.get("target")
        if placed is not None and target is not None:
            bits.append(f"filling {box} ({placed} of {target} parts in, {fill_state})")
        else:
            bits.append(f"working {box} ({fill_state})")
        if line.get("loaded"):
            bits.append(f"cart {line['loaded']} is loaded")
        if line.get("queued"):
            bits.append(f"{line['queued']} queued on the outfeed")
    elif "carrying" in state:
        # Mobile tug.
        leg = str(loop.get("leg") or state.get("mode") or "idle")
        cart = state.get("carrying")
        if cart:
            bits.append(f"towing {cart} ({leg} leg)")
        else:
            bits.append(f"not towing anything right now ({leg} leg)")
    elif state.get("mode"):
        bits.append(f"in {state['mode']} mode")

    grip = state.get("gripper") or {}
    if grip.get("holding"):
        bits.append("holding a part in the gripper")

    # ── Is my autonomy running ───────────────────────────────────
    if loop:
        loop_mode = str(loop.get("mode") or "idle")
        counter = loop.get("picks")
        unit = "picks"
        if counter is None:
            counter = loop.get("cycles")
            unit = "cycles"
        done = f", {counter} {unit} done" if counter is not None else ""
        if paused:
            bits.append(f"my {loop_mode} loop is PAUSED by an operator command{done}")
        else:
            bits.append(f"my {loop_mode} loop is running{done}")
    else:
        bits.append("I have no autonomous loop running")

    return f"I'm {who}: " + "; ".join(bits) + "."


def _intent_reply(tool: str, res: Dict[str, Any], ok_text: str
                  ) -> Dict[str, Any]:
    """Relay an IntentStore result, refusal included, without softening it."""
    refused = res.get("accepted") is False
    if refused:
        said = res.get("say") or res.get("reason") or "I cannot express that."
        # §5.7.2: `rule` is the field a program branches on. The store's
        # own `reason` is already a short code (`unknown_rule`,
        # `unsupported_condition`); prose, if any, is in `say` and stays in
        # `summary`.
        _reason = str(res.get("reason", "") or "")
        _rule = (_reason if _reason and " " not in _reason
                 else "intent_refused")
        return {"agent": said,
                "tools": [(tool, "refused", _reason[:80] or said[:80],
                           _rule)]}
    return {"agent": res.get("commitment") or ok_text,
            "tools": [(tool, "ok", str(res.get("id", "")))]}


def execute(bridge: Any, r: "_i.Interpretation",
            surface: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """Run an interpretation. None = not ours; give it to the model."""
    intents = getattr(bridge, "intents", None)

    if r.intent == _i.QUERY:
        kind = (r.frames[0].args.get("kind") if r.frames else "state")
        if kind == "state":
            # The residual bucket: a question this parser could not tie to
            # anything the robot knows. It still must NOT fall through --
            # returning None hands a question to a keyword ladder -- so it
            # declines with no action rather than reciting pose at someone
            # who asked about the weather.
            return {"agent": "I'm not sure that's something I can answer. I "
                             "can tell you where I am, what I'm doing, what "
                             "I'm carrying, or what I've done this session.",
                    "tools": [("decline", "no_action", "unrecognised question")]}
        # Answered from the bridge's own /state, never from a template.
        # A bridge may supply its OWN sentence: describe_state() below
        # understands arms and tugs, and would answer a drone's altitude
        # question with "in idle mode", which is true but useless.
        own = getattr(bridge, "describe_state", None)
        state = getattr(bridge, "get_state_for_query", None)
        if own is not None or state is not None:
            try:
                if own is not None:
                    said = own()
                else:
                    said = describe_state(state())
            except Exception:                      # pragma: no cover
                said = "I could not read my own state just now."
            return {"agent": said,
                    "tools": [("get_robot_state", "ok", str(kind))]}
        # No state reader on this bridge. Still not a fallthrough: a question
        # must never reach a keyword ladder with a motor behind it.
        return {"agent": f"I can't answer that - this robot doesn't expose "
                         f"its {kind} to me.",
                "tools": [("decline", "no_action", str(kind))]}

    if r.intent == _i.AMBIGUOUS:
        # A question, not a guess, and nothing moves.
        return {"agent": f"I need one more thing before I move: {r.reason}. "
                         f"Which one do you mean?",
                "tools": [("clarify", "ask", r.residue[:80])]}

    # IntentStore refuses what it cannot express, and the refusal carries the
    # sentence to relay verbatim in `say`. Reporting those as "ok" is exactly
    # the "sure, I'll do that" lie its docstring exists to prevent.
    if r.intent == _i.CONSTRAINT:
        if intents is not None and hasattr(intents, "set_constraint"):
            words = r.residue or r.trigger or ""
            rule = _match_constraint_rule(intents, words) or words
            res = intents.set_constraint(rule, words=words)
            return _intent_reply("set_constraint", res,
                                 "Noted as a standing restriction.")
        return None

    if r.intent == _i.DEFERRED:
        if intents is not None and hasattr(intents, "schedule"):
            # The store schedules pauses/notifications, not arbitrary motion.
            # Never silently turn a deferred drive (or an unparsed action)
            # into a pause or notification.
            if not r.frames or any(f.tool != "stop" for f in r.frames):
                return {"agent": "I can schedule a stop after the current task, "
                                 "but I can't schedule that action.",
                        "tools": [("schedule_intent", "refused",
                                   "unsupported action",
                                   "unsupported_action")]}
            condition = r.trigger or ""
            if re.fullmatch(
                    r"after (?:this|the current|current) (?:delivery|task|pick|cart)",
                    condition, re.IGNORECASE) or re.fullmatch(
                    r"(?:when|once) you (?:have finished|finish|complete) "
                    r"(?:this|the current) (?:delivery|task|pick|cart)",
                    condition, re.IGNORECASE):
                condition = "after_current_task"
            res = intents.schedule("pause", condition, words=r.text or r.trigger or "")
            return _intent_reply("schedule_intent", res,
                                 "I will do that when the condition is met.")
        return None

    if r.intent == _i.MEMORY:
        # Nothing here persists a fact, and saying "noted" would be the lie
        # this whole layer exists to avoid.
        return {"agent": "I can't store that - I have no memory beyond this "
                         "session's standing restrictions. Tell me as a rule "
                         "(\"never tow TROLLEY_E\") and I will hold it.",
                "tools": [("remember", "unsupported", r.residue[:60])]}

    if r.intent == _i.CONVERSATION and r.confidence >= 0.8:
        # ⚠️ Do NOT fall through here. Returning None hands the utterance to
        # the bridge's own keyword ladder, and that is how a quadruped
        # answered "you never stood up, I was watching - admit it" by
        # actuating reset_to_home. A challenge to the robot's account must
        # reach a model or nothing -- never a regex with a motor behind it.
        return {"agent": "I can't confirm or deny that from here. What I can "
                         "do is tell you exactly what I did - ask me what my "
                         "last actions were.",
                "tools": [("decline", "no_action", r.reason[:60])]}

    if r.intent != _i.COMMAND:
        return None                    # low-confidence CONVERSATION / EMPTY

    # ── The gate. Last thing between a frame and a motor. ────────────
    # It judges (utterance, frames) and does not care who produced the
    # frames, so it protects against this parser, against a model
    # interpreting the messy cases, and against whatever comes next. If
    # the safety rules lived only in `interpret`, swapping the interpreter
    # would silently remove them.
    # `surface` picks the rail where two robot classes share a tool -- a
    # drone's move_body{vertical} is a climb, a quadruped's is a body shift.
    rejections = _gate.check(r.text or "", r.frames, surface=surface)
    if rejections:
        first = rejections[0]
        # ⚠️ PROSE IN `summary`, THE RULE NAME IN `rule` (§5.7.2). This used
        # to put the rule name in `summary` and drop `first.detail`
        # entirely, so the only human-readable account of the refusal --
        # "distance=300.0 exceeds the 50 m rail" -- existed nowhere in the
        # envelope, and a client had exactly one field carrying two jobs
        # badly.
        _emit_gate_refusal(bridge, first, r.text or "")
        return {"agent": f"I won't do that: {first.detail}.",
                "tools": [(first.tool, "refused", first.detail, first.rule)]}

    said: List[str] = []
    tools: List[Tuple[str, str, str]] = []
    motion = [i for i, f in enumerate(r.frames) if f.tool in _ADAPTERS]
    last_motion = motion[-1] if motion else -1
    for idx, f in enumerate(r.frames):
        if f.tool == "hold":
            if intents is not None and hasattr(intents, "hold_now"):
                res = intents.hold_now(words=f.source)
                said.append("I will stay put until you tell me to carry on - "
                            "no auto-resume.")
                tools.append(("hold", "ok", str(res.get("id", "held"))))
            else:
                said.append("I have stopped, but this robot cannot hold "
                            "indefinitely - it may resume on its own.")
                tools.append(("hold", "unsupported", ""))
            continue

        if f.tool == "place":
            # Placing needs a grounded target pose; the parser gives a NAME.
            said.append(f"I can place it on the {f.args.get('target', '?')} "
                        f"once I know where that is - say the coordinates, or "
                        f"ask me to look first.")
            tools.append(("place", "needs_grounding", str(f.args)))
            continue

        entry = _ADAPTERS.get(f.tool)
        if entry is None:
            said.append(f"I understood '{f.tool}' but I have no way to do it.")
            tools.append((f.tool, "unsupported", ""))
            continue
        method, build = entry
        # Every motion but the last blocks, so the next one is not refused
        # as `busy`. The last returns immediately and keeps chat responsive.
        res, dropped = _call(bridge, method, build(f.args),
                             block=idx != last_motion)
        if res is None and not hasattr(bridge, method):
            said.append(f"This robot cannot {f.tool.replace('_', ' ')}.")
            tools.append((f.tool, "unsupported", ""))
            continue
        text, status, detail, rule = _describe(f.tool, f.args, res, dropped)
        said.append(text)
        tools.append((f.tool, status, detail, rule))

    if r.residue:
        said.append(f"I did not act on '{r.residue[:60]}' - I did not "
                    f"understand that part.")
    return {"agent": " ".join(said), "tools": tools}


def route(bridge: Any, text: str, surface: str = _i.MOBILE
          ) -> Optional[Dict[str, Any]]:
    """Interpret `text` and execute it. None = hand this to the model."""
    return execute(bridge, _i.interpret(text, surface), surface)


# ⚠️ `legacy_router_requested()` / OMNISIM_BRIDGE_LEGACY_ROUTER ARE DELETED
# (2026-09-22). The variable existed to put each bridge's keyword ladder back
# in front of the parser. Those ladders are gone and none may return, so the
# lever restored nothing -- but it was a DOCUMENTED environment variable whose
# generated reference entry read "puts the old keyword ladder back in front of
# the parser", which is an advertisement for exactly the fallback the access
# policy forbids. A reader would have tried it. Do not reintroduce it under
# any name.
#
# `tests/benchmarks/commandbench` still names it as an experiment arm; that
# arm selected nothing once the ladders went and now sets an unread variable.


def route_if_enabled(bridge: Any, text: str, surface: str = _i.MOBILE
                     ) -> Optional[Dict[str, Any]]:
    """`route`, plus a hard guarantee of not breaking a demo.

    A bridge adopts the interpreter with two lines and cannot be taken down
    by it: any exception returns None instead of propagating.

    ⚠️ RETURNING None IS NOT A FALLBACK ANY MORE. It used to mean "the
    bridge's keyword ladder takes this one"; those ladders were deleted on
    2026-09-22, so None now means the turn goes to the relay's model, and a
    caller with no relay must refuse it. `short_circuit` is what the shipped
    bridges want -- it records the turn and is the parser-first path.
    """
    try:
        return route(bridge, text, surface)
    except Exception as exc:                       # pragma: no cover
        print(f"[interpret] parser error, deferring to the model: {exc}",
              flush=True)
        return None


# ── Parser-first: the shipped default ───────────────────────────────
# THE PARSER INTERPRETS FIRST; THE MODEL IS CALLED ONLY ON WHAT IT
# DECLINES. That is what every guide, the README and the chat-demo pages
# say the product does, and since 2026-09-22 it is what the code does too.
# It was SHADOW by default until then -- the parser ran, recorded what it
# would have handled, and the relay answered every turn anyway -- which
# made the documentation describe an unmeasured mode nobody shipped.
#
# The default acts on COMMAND only: a confident, exactly-parsed order with
# every magnitude present in the sentence. Everything else -- questions,
# challenges, constraints, deferred intents, anything under the confidence
# floor -- still goes to the relay, and `short_circuit` returns None so the
# caller's model path runs unchanged.
#
# ⚠️ THIS IS NOT AN ACCESS DECISION. The OmniKey check sits in FRONT of the
# parser, in each bridge's `/prompt` handler: with no relay attached the
# request is refused 401 `omnikey_required` and the parser never sees the
# sentence. Parser-first makes a CONNECTED bridge cheaper; it does not make
# an unconnected one answer. The 2026-09-22 access policy stands.
#
#   unset / 1 / true   act on COMMAND -- the deterministic, exact cases.
#   0 / false / off    shadow. count only. the relay answers every turn.
#   command,query      act on the named intents (comma-separated).
#   all                act on everything the parser does not defer.
#
# ⚠️ Widening past `command` is a PRODUCT decision, not a tuning one. A query
# answered from /state is a terse factual line; the relay would have written
# a sentence. Cheaper is not automatically better for someone paying.
_STATS: Dict[str, Any] = {
    "turns": 0,
    "by_intent": {},
    "eligible": 0,        # a confident parse the flag WOULD/DID act on
    "short_circuited": 0,  # actually answered without the relay
    "relay_calls": 0,
}

_SHORT_CIRCUIT_DEFAULT = (_i.COMMAND,)
_MIN_CONFIDENCE = 0.8


def _parser_first_set() -> Tuple[str, ...]:
    # OMNISIM_BRIDGE_PARSER_FIRST selects which intents the deterministic
    # parser ANSWERS instead of paying a model for them. Unset is ON: the
    # parser answers confident COMMAND turns and the relay gets everything
    # else, which is the behaviour the guides describe. Value-parsed, and
    # `=0` is the opt-OUT rather than the default -- it restores SHADOW
    # mode, where the parser still runs and still records what it would
    # have done (`_STATS`) but the relay answers every turn. A
    # comma-separated list names exactly the intents to answer, and `all`
    # answers everything the parser does not defer. It is never an access
    # control: the OmniKey check refuses a keyless `/prompt` before the
    # parser is reached, whatever this is set to.
    import os
    raw = os.environ.get("OMNISIM_BRIDGE_PARSER_FIRST", "").strip().lower()
    if raw in ("0", "false", "no", "off"):
        return ()
    if raw in ("", "1", "true", "yes", "on"):
        return _SHORT_CIRCUIT_DEFAULT
    if raw == "all":
        return (_i.COMMAND, _i.QUERY, _i.CONSTRAINT, _i.DEFERRED, _i.AMBIGUOUS)
    return tuple(p.strip() for p in raw.split(",") if p.strip())


def parser_stats() -> Dict[str, Any]:
    """What the parser saw, and what it would have handled without the LLM."""
    s = dict(_STATS)
    s["by_intent"] = dict(_STATS["by_intent"])
    turns = max(1, s["turns"])
    s["eligible_pct"] = round(100.0 * s["eligible"] / turns, 1)
    s["short_circuited_pct"] = round(100.0 * s["short_circuited"] / turns, 1)
    s["acting_on"] = list(_parser_first_set()) or ["(shadow: counting only)"]
    return s


def parser_first_plan(text: str, surface: str = _i.MOBILE
                      ) -> Optional["_i.Interpretation"]:
    """The parser-first DECISION, with no bridge call and no actuation.

    Split out of `short_circuit` so a caller on a thread that must not
    block can decide cheaply here (this is pure regex) and execute
    elsewhere. `handle_wwi_message` is exactly that caller: it runs on the
    SIM thread, and a compound order ("turn left 90 then drive forward
    1 m") asks every motion but the last to BLOCK until it completes --
    which on the sim thread is a deadlock, because the motion only
    advances when that same thread ticks.

    None means "not the parser's": the caller's model path runs unchanged.
    Records the turn either way, so shadow mode stays a free measurement.
    """
    try:
        r = _i.interpret(text, surface)
    except Exception:                                  # pragma: no cover
        return None

    _STATS["turns"] += 1
    _STATS["by_intent"][r.intent] = _STATS["by_intent"].get(r.intent, 0) + 1

    confident = r.confidence >= _MIN_CONFIDENCE
    acting = _parser_first_set()
    # Eligibility is judged against the DEFAULT set in shadow mode, so the
    # recorded number answers "what would turning this on have saved?"
    judged = acting or _SHORT_CIRCUIT_DEFAULT
    if confident and r.intent in judged:
        _STATS["eligible"] += 1

    if not acting or not confident or r.intent not in acting:
        _STATS["relay_calls"] += 1
        return None
    return r


def _only_unsupported(tools: Any) -> bool:
    """True when the parse produced ONLY `unsupported` outcomes.

    ⚠️ `refused` is deliberately not in here. A gate rejection must be the
    parser's final word: handing a gate-refused sentence on to a model is
    an invitation to find another way to do the thing the gate just
    vetoed.
    """
    rows = [t for t in (tools or ()) if len(t) >= 3]
    return bool(rows) and all(t[1] == "unsupported" for t in rows)


def parser_first_run(bridge: Any, r: "_i.Interpretation",
                     surface: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """Execute a `parser_first_plan` result. None = hand it to the model."""
    try:
        out = execute(bridge, r, surface)
    except Exception:                                  # pragma: no cover
        out = None
    if out is not None and _only_unsupported(out.get("tools")):
        # The parser understood the words and this robot has no way to do
        # it. That is a DECLINE, not an answer. A model attached to this
        # bridge may well serve the request another way -- the OmniTug
        # courier has stations and a route queue, not a `drive_forward` --
        # and closing the turn with "this robot cannot drive forward"
        # would make parser-first strictly worse than the relay it
        # replaced. Nothing was actuated, so handing it on is free.
        out = None
    if out is None:
        _STATS["relay_calls"] += 1
        return None

    _STATS["short_circuited"] += 1
    out = dict(out)
    out["via"] = "parser"
    return out


def short_circuit(bridge: Any, text: str, surface: str = _i.MOBILE
                  ) -> Optional[Dict[str, Any]]:
    """Called on the RELAY path, before the LLM.

    Returns a reply only when the flag says to act on this intent AND the
    parse is confident. Otherwise returns None and the relay runs as usual.
    Always records what it saw, so shadow mode is a free measurement.

    This is the SYNCHRONOUS form, for a caller already off the sim thread:
    every bridge's HTTP `/prompt` handler, which runs on a
    ThreadingHTTPServer worker. The robot-window path wants
    `parser_first_window` instead.
    """
    r = parser_first_plan(text, surface)
    if r is None:
        return None
    return parser_first_run(bridge, r, surface)


def window_lines(out: Dict[str, Any]) -> List[str]:
    """Render a parser-answered reply as robot-window protocol lines.

    Same three shapes the relay's event callback queues -- `agent:`,
    `tool:<name>:<status>:<detail>`, and a terminal `status:idle` -- so the
    chat panel cannot tell a parser-answered turn from a model-answered
    one except by looking at what it says.
    """
    lines: List[str] = []
    said = str(out.get("agent") or "").strip()
    if said:
        lines.append("agent:" + said)
    for t in out.get("tools") or ():
        if len(t) >= 3:
            lines.append(f"tool:{t[0]}:{t[1]}:{t[2]}")
    lines.append("status:idle")
    return lines


def parser_first_window(bridge: Any, text: str, surface: str,
                        emit: Any, to_model: Any,
                        transcribe: Any = None, spawn: Any = None) -> bool:
    """Parser-first for a bridge's ROBOT-WINDOW prompt path.

    The window path is the one a human actually types into, so a fix that
    only covered HTTP would demo wrongly -- but it is also the one that
    runs on the SIM THREAD (`main()` pumps wwi messages), where a blocking
    act_* deadlocks the simulation. So the DECISION is taken here, on the
    caller's thread, and the EXECUTION is handed to a worker exactly as
    `relay.dispatch_async` already does.

      emit(line)        the bridge's queue_window (one protocol line).
      to_model()        zero-arg; hands the turn to the relay as before.
      transcribe(out)   optional; records a parser-answered turn.
      spawn(fn)         optional; how to run the worker. Defaults to a
                        daemon thread; tests pass `lambda fn: fn()`.

    Returns True when this call has TAKEN OVER the turn -- answered it, or
    handed it to `to_model` from the worker. The caller must not dispatch
    again. False means nothing happened and the caller's own model path
    runs, unchanged.

    ⚠️ NOT AN ACCESS DECISION. The caller checks the OmniKey first and
    never reaches this with `relay is None`; see each bridge's
    `handle_wwi_message`.
    """
    try:
        r = parser_first_plan(text, surface)
    except Exception:                                  # pragma: no cover
        return False
    if r is None:
        return False

    def _worker() -> None:
        try:
            out = parser_first_run(bridge, r, surface)
        except Exception as exc:                       # pragma: no cover
            print(f"[interpret] parser error, deferring to the model: {exc}",
                  flush=True)
            out = None
        if out is None:
            # Planned, then declined (an intent store this bridge has not
            # got, a tool this robot does not serve). Same rule as
            # everywhere else: what the parser declines, the model gets.
            to_model()
            return
        try:
            for line in window_lines(out):
                emit(line)
        finally:
            if transcribe is not None:
                try:
                    transcribe(out)
                except Exception:                      # pragma: no cover
                    pass

    if spawn is None:
        import threading
        threading.Thread(target=_worker, name="omnilink-parser",
                         daemon=True).start()
    else:
        spawn(_worker)
    return True


def reply_payload(reply: str, tools: Any, **extra: Any) -> Dict[str, Any]:
    """Build an HTTP reply body from (tool, status, detail) tuples.

    ALSO SETS A TOP-LEVEL `error` WHEN AN ACTION FAILED, and that is the
    whole reason this is shared rather than written out at each site.

    A bridge refuses a command that arrives while it is still moving. It is
    honest about it: the reply reads "I could not turn: busy" and the action
    carries result="err", summary="busy". But a machine client reads fields,
    not sentences, and the one obvious field to check -- `error` -- was
    absent, so a refused order was indistinguishable from a completed one.

    A two-hour endurance run lost about 5% of its orders to exactly that.
    The closed square it was driving stopped closing, the robot walked off
    the 12 m floor, and the run logged zero errors from start to finish.
    The commands themselves were never inaccurate; the last turn before the
    escape missed by 0.00042 rad. What failed was the reporting.

    Whatever drives this surface is a program. A failure it cannot see is a
    failure that did not happen.
    """
    actions = []
    for t in tools:
        if len(t) < 3:
            continue
        entry: Dict[str, Any] = {"tool": t[0], "result": t[1],
                                 "summary": t[2]}
        # PROTOCOL.md §5.7.2: `rule` is REQUIRED on a refusal and is the
        # field a PROGRAM branches on; `summary` is prose and may change
        # between releases. It rides as an optional fourth element of the
        # tuple, so every existing 3-tuple call site keeps working and only
        # the sites that know a rule name have to say one.
        if len(t) >= 4 and t[3]:
            entry["rule"] = str(t[3])
        elif t[1] == "refused":
            # A refusal with no rule is exactly the hole this section
            # exists to close, so it is named rather than left absent: a
            # client can branch on "refused, reason not classified" and a
            # missing key looks like a client bug instead.
            entry["rule"] = "unclassified"
        actions.append(entry)
    out: Dict[str, Any] = {"response": reply, "actions": actions}
    out.update(extra)
    # ⚠️ "err" AND "error" ARE BOTH IN THE WILD and §5.7.2 says a client must
    # accept either; this producer is a client of its own tuples, so it
    # accepts both too. Missing "error" from that set is how a relay-spelled
    # failure could have set no top-level error at all.
    failed = [a for a in actions
              if a["result"] in ("err", "error", "refused")]
    if failed:
        out["error"] = "; ".join(
            # For a refusal the RULE is the useful half -- §5.7.2's own
            # example reads "drive_forward: interrogative". `unclassified`
            # is NOT that: it is the placeholder for a producer that did not
            # give one, and printing it would replace a caller's usable
            # sentence with a word meaning "we do not know", which is a
            # strictly worse error string than the one it had before.
            f"{a['tool']}: "
            f"{_error_reason(a)}"
            for a in failed)
    return out


def _error_reason(action: Dict[str, Any]) -> str:
    """The most useful half of one failed action, for the top-level `error`."""
    rule = action.get("rule") if action.get("result") == "refused" else ""
    if rule and rule != "unclassified":
        return str(rule)
    return str(action.get("summary") or action.get("rule")
               or action.get("result") or "")


def stamp_via(payload: Any, default: str = "relay") -> Any:
    """Guarantee a top-level `via` on a 200 from `/prompt` (plan D3 step 2).

    `via` names WHICH STAGE answered the turn -- `"parser"` or `"relay"` --
    and the sweep's whole two-door comparison is built on it: a missing
    `via` is recorded as `null`, the comparison becomes `undetermined`, and
    the run fails by design rather than guessing. So the field cannot be
    best-effort.

    The default is `"relay"` because the parser is the only stage that can
    answer without the model and it always stamps itself
    (`parser_first_run`). A bridge whose `/prompt` answers from neither --
    a canned reply, a queue, a second interpreter -- MUST stamp its own
    value; this function will not invent a third name for it.
    """
    if not isinstance(payload, dict):
        return payload
    via = payload.get("via")
    if isinstance(via, str) and via.strip():
        return payload
    payload["via"] = default
    return payload
