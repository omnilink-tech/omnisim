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
import math
import re
import time
from typing import Any, Dict, List, Optional, Tuple

from omnisim_bridges import runtime_clock

from . import gate as _gate
from . import interpret as _i

__all__ = ["route", "execute", "describe_state", "short_circuit",
           "parser_first_plan", "parser_first_run", "parser_first_window",
           "window_lines", "parser_stats", "reply_payload", "stamp_via",
           "resolve_conditional"]

# tool -> (bridge method, kwargs builder). Names match the act_* API the
# bridges already expose; anything missing degrades to a spoken refusal.
# Frames that set or lift a standing spatial rule (interpret._spatial_rule),
# or schedule a motion for later (interpret._later_order). None actuates when
# the sentence is spoken; a scheduled motion is gated on its own clause here
# and again by the bridge when it fires.
_RULE_FRAMES = frozenset({"boundary", "clear_boundary", "schedule", "watch"})

_ADAPTERS: Dict[str, Tuple[str, Any]] = {
    "stop":            ("act_stop", lambda a: {}),
    "resume_autonomy": ("act_resume_autonomy", lambda a: {}),
    "reset_to_home":   ("act_reset_to_home", lambda a: {}),
    # ⚠️ No default distance. This used to be `a.get("distance", 1.0)`, so a
    # drive nobody sized travelled a metre. The gate now requires `distance`
    # and the parser asks "how far?" instead (owner decision 2026-09-23).
    "drive_forward":   ("act_drive_forward", lambda a: {"distance": a["distance"]}),
    "turn":            ("act_turn", lambda a: {"angle_rad": a["angle_rad"]}),
    "drive_to":        ("act_drive_to", lambda a: {"tx": a["x"], "ty": a["y"]}),
    "go_to_place":     ("act_go_to_place", lambda a: {"place": a["place"]}),
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
    fault = res.get("fault") if isinstance(res.get("fault"), dict) else None
    if isinstance(err, str) and err.strip():
        if fault is not None and res.get("accepted") is False:
            # A command refused because the robot is FAULTED (a tipped base):
            # the bridge's own sentence, and the fault code as the rule.
            why = str(res.get("say") or fault.get("message") or err)
            return (f"I did not {tool.replace('_', ' ')}: {why}", "refused",
                    why, str(fault.get("code") or err))
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
    if tool == "stop" and fault is not None:
        # Standing still is not the same as fine: a base on its side is
        # perfectly stationary (MAB HB50W, 48 deg, 2026-10-17).
        msg = str(fault.get("message") or fault.get("code") or "fault")
        return (f"Motion is commanded to zero, but the robot is FAULTED: {msg}.",
                "err", f"fault {fault.get('code', '')}", "")
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

    if tool in ("sit", "stand", "reset_to_home") and "settled" in res \
            and res.get("pose") in ("sit", "stand"):
        return _measured_pose(tool, res)

    bits = ", ".join(f"{k}={v}" for k, v in args.items() if v is not None)
    said = f"{tool.replace('_', ' ').capitalize()}" + (f" ({bits})" if bits else "") + "."
    measured = _measured_motion(tool, res)
    if measured is not None:
        said, status, detail = measured
        if status != "ok":
            return said, status, detail, ""
        bits = detail
    if res.get("clipped_by"):
        # The bridge shortened this move to respect a standing boundary. The
        # reply used to echo the REQUESTED distance ("Drive forward
        # (distance=1.0).") over a 0.35 m drive -- the operator's number
        # handed back as if it had been done (ops-bench F1, 2026-09-25).
        rule = res["clipped_by"]
        req, got = float(res.get("requested", 0.0)), float(res.get("commanded", 0.0))
        said = (f"{tool.replace('_', ' ').capitalize()}: shortened from "
                f"{req:.2f} to {got:.2f} m to stop at the boundary you set "
                f"({rule.get('means', 'a standing rule')}).")
        bits = f"clipped {req:.2f} -> {got:.2f} by {rule.get('id', 'boundary')}"
    if dropped:
        said += (" I ignored " + ", ".join(dropped)
                 + " - this robot's " + tool + " does not take it.")
    return said, "ok", bits or "ok", ""


def _measured_pose(tool: str, res: Dict[str, Any]) -> Tuple[str, str, str, str]:
    """Say what a sit / stand MEASURED. It used to echo the tool ("Sit.
    Stand.") the moment the setpoint changed, over a body that never went
    down (XGO Mini2S, v9.2.6, 2026-10-20). Not settled is an error."""
    pose = str(res.get("pose") or ("sit" if tool == "sit" else "stand"))
    name = {"sit": "the sit pose", "stand": "the standing pose"}.get(pose, pose)
    ach = res.get("achieved") if isinstance(res.get("achieved"), dict) else {}
    err = ach.get("joint_error_max_deg")
    z0, z1 = ach.get("body_z_start_m"), ach.get("body_z_m")
    body = (f"; body height {z0:.3f} -> {z1:.3f} m"
            if isinstance(z0, (int, float)) and isinstance(z1, (int, float)) else "")
    if res.get("superseded"):
        return (f"I did not finish moving into {name}: a newer order or a stop "
                f"took over first{body}.", "ok", "superseded", "")
    settled = res.get("settled")
    if settled is None:
        return (f"I started moving into {name}; I did not wait to confirm it.",
                "ok", "unconfirmed", "")
    if res.get("stalled"):
        return (f"I could not confirm {name}: the simulation stopped stepping.",
                "err", "stalled", "")
    if settled:
        took = res.get("settle_s")
        when = f" after {took:.2f} s" if isinstance(took, (int, float)) else ""
        legs = f"legs within {err:.1f} deg" if isinstance(err, (int, float)) else "legs settled"
        return (f"In {name}{when}: {legs}{body}.", "ok",
                f"settled, max joint error {err} deg" if err is not None else "settled", "")
    off = f": legs still {err:.1f} deg off" if isinstance(err, (int, float)) else ""
    return (f"I did not reach {name}{off}{body}"
            + (" (timed out)." if res.get("timed_out") else "."),
            "err", f"not settled, max joint error {err} deg", "")


def _measured_motion(tool: str, res: Dict[str, Any]
                     ) -> Optional[Tuple[str, str, str]]:
    """Say what a FINISHED drive or turn measured, or None if it did not wait.

    A result with no `achieved` (a non-waiting call, or a bridge that does not
    measure) keeps the old echo. A motion that timed out or did not settle
    short of its target is an ERROR -- "stopped short" -- never a success
    that happens to quote the operator's own number.
    """
    if tool in ("drive_to", "go_to_place", "resume_last_order", "reset_to_home") or (
            tool == "resume_autonomy" and "achieved_xy" in res):
        return _measured_drive_to(res)
    if tool not in ("drive_forward", "turn"):
        return None
    fault = res.get("fault")
    if isinstance(fault, dict) and fault:
        # Ended by a guard (the base-tilt fault): what it measured before
        # the guard fired, and why it stopped. Never "Drove +1.98 m."
        msg = str(fault.get("message") or fault.get("code") or "fault")
        ach = res.get("achieved")
        if isinstance(ach, (int, float)):
            got = (f"{ach:+.2f} m" if tool == "drive_forward"
                   else f"{math.degrees(ach):+.0f} deg")
            return (f"I stopped after {got}: {msg}.", "err",
                    f"fault {fault.get('code', '')} at {got}")
        return (f"I stopped: {msg}.", "err", f"fault {fault.get('code', '')}")
    if res.get("superseded"):
        return ("That motion was ended before it finished (a newer order or a "
                "stop took over).", "ok", "superseded")
    ach, cmd = res.get("achieved"), res.get("commanded")
    if not isinstance(ach, (int, float)) or not isinstance(cmd, (int, float)):
        return None
    if tool == "drive_forward":
        short = abs(cmd) - abs(ach) > max(0.05, 0.1 * abs(cmd))
        if res.get("blocked") and short:
            return (f"I was blocked: I drove {ach:+.2f} m of the {cmd:+.2f} m asked "
                    "and then stopped making progress, so I stopped trying.",
                    "err", f"blocked at {ach:+.2f}/{cmd:+.2f} m")
        if (res.get("timed_out") or res.get("settled") is False) and short:
            return (f"I stopped short: I drove {ach:+.2f} m of the {cmd:+.2f} m "
                    "asked, and the drive did not finish -- something may be "
                    "blocking me.", "err", f"stopped short {ach:+.2f}/{cmd:+.2f} m")
        return (f"Drove {ach:+.2f} m.", "ok", f"achieved {ach:+.3f} m of {cmd:+.3f}")
    deg, want = math.degrees(ach), math.degrees(cmd)
    if (res.get("timed_out") or res.get("settled") is False) and abs(want) - abs(deg) > 10:
        return (f"I stopped short: I turned {deg:+.0f} deg of the {want:+.0f} deg "
                "asked.", "err", f"stopped short {deg:+.0f}/{want:+.0f} deg")
    return (f"Turned {deg:+.0f} deg.", "ok", f"achieved {deg:+.1f} deg of {want:+.1f}")


def _measured_drive_to(res: Dict[str, Any]) -> Optional[Tuple[str, str, str]]:
    """What a drive_to / go_to_place MEASURED: arrived or not, where it
    ended, whether it went round something, and whether it was pushed.
    It used to echo the plan ("Drive to (x=4.0, y=-0.0).") whatever happened
    (ops-bench holdout v1 F4: blocked, shoved and detoured drives all read
    the same)."""
    if res.get("accepted") is False:
        return (str(res.get("say") or res.get("error") or "I could not do that."),
                "err", str(res.get("refused") or "refused"))
    xy = res.get("achieved_xy")
    if not isinstance(xy, (list, tuple)) or len(xy) != 2:
        return None
    target = res.get("commanded_xy")
    where = res.get("place") or (f"({target[0]:.2f}, {target[1]:.2f})" if target else "there")
    notes = []
    if res.get("superseded") or "superseded" in str(res.get("aborted") or ""):
        # What it met on the way is still news: "the pallet is in the way"
        # must not vanish because the next order arrived first.
        if res.get("obstacles_met"):
            notes.append(f"something was blocking my path to {where}, and I was going around it")
        if res.get("disturbances"):
            notes.append("I was pushed off course on the way")
        tail = (" Before that, " + "; ".join(notes) + ".") if notes else ""
        return ("That drive was ended before it finished (a newer order or a stop took "
                f"over); I'm at ({xy[0]:.2f}, {xy[1]:.2f}).{tail}", "ok", "superseded")
    pushes = res.get("disturbances") or []
    if pushes:
        big = max(pushes, key=lambda d: d.get("moved_m", 0))
        notes.append(f"I was pushed off course by about {big['moved_m']:.2f} m on the way "
                     "and corrected for it")
    if res.get("obstacles_met"):
        notes.append("something was blocking my path, so I went around it")
    elif res.get("detoured"):
        notes.append("I routed around a keep-out area")
    tail = (" " + "; ".join(notes) + ".") if notes else ""
    if res.get("arrived"):
        return (f"Arrived at {where}: I'm at ({xy[0]:.2f}, {xy[1]:.2f}), "
                f"{float(res.get('error_m') or 0):.2f} m from the target.{tail}",
                "ok", f"arrived err {float(res.get('error_m') or 0):.3f} m")
    return (f"I didn't make it to {where}: I stopped at ({xy[0]:.2f}, {xy[1]:.2f}), "
            f"{float(res.get('error_m') or 0):.2f} m short, because "
            f"{'the way was blocked' if 'block' in str(res.get('aborted')) else 'the drive did not finish'}."
            f"{tail}", "err", f"stopped short err {float(res.get('error_m') or 0):.3f} m")


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


def resolve_conditional(r: "_i.Interpretation", pose: Any
                        ) -> Optional["_i.Interpretation"]:
    """The COMMAND a CONDITIONAL comes to at `pose` (x, y, yaw), or None.

    Pure: no bridge, so the benchmark and the router share it. The chosen
    frames may be empty ("otherwise stay put"). None when there is no usable
    pose -- never a guess at which branch applies.
    """
    if not isinstance(getattr(r, "condition", None), dict):
        return None
    try:
        x, y, yaw = (float(v) for v in pose)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return None
    holds = _i.evaluate_condition(r.condition, (x, y, yaw))
    chosen = r.then_frames if holds else r.else_frames
    out = _i.Interpretation(
        _i.COMMAND, frames=[_i.Frame(f.tool, dict(f.args), f.span, f.source, f.rule)
                            for f in chosen],
        confidence=r.confidence, text=r.text,
        reason=f"condition {'held' if holds else 'did not hold'} at "
               f"x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f} deg")
    return out


def _bridge_pose(bridge: Any) -> Optional[Tuple[float, float, float]]:
    """The bridge's MEASURED pose (x, y, yaw), or None if it cannot say."""
    for name in ("read_pose", "_read_pose"):
        fn = getattr(bridge, name, None)
        if callable(fn):
            try:
                x, y, yaw = fn()
                return float(x), float(y), float(yaw)
            except Exception:
                return None
    state = getattr(bridge, "get_state_for_query", None)
    if callable(state):
        try:
            st = state() or {}
            p = st.get("pose", st)
            return float(p["x"]), float(p["y"]), float(p["yaw"])
        except Exception:
            return None
    return None


def _schedule_frame(bridge: Any, intents: Any, f: Any, utterance: str,
                    surface: Optional[str]) -> Tuple[str, Tuple[str, ...]]:
    """Hand a `schedule` / `watch` frame to the store, gated on its own clause.

    Never agrees to what nothing will run: a store without a scheduler (no
    clock, no disturbance detector on this bridge) answers `unsupported`, and
    the whole turn then goes to the model instead.
    """
    if intents is None or not getattr(intents, "scheduler", False):
        return ("I can't keep an order for later on this robot, so I have not "
                "agreed to one.", (f.tool, "unsupported", ""))
    action_text = str(f.args.get("text") or "")
    frames = [dict(x) for x in f.args.get("frames") or []]
    # Vet the action exactly as if it were said now, with its own words.
    rej = _gate.check(action_text, [_i.Frame(x["tool"], dict(x.get("args") or {}))
                                    for x in frames], surface=surface)
    if rej:
        _emit_gate_refusal(bridge, rej[0], action_text)
        return (f"I won't schedule that: {rej[0].detail}.",
                (f.tool, "refused", rej[0].detail, rej[0].rule))
    if f.tool == "watch":
        res = intents.schedule_action("on_disturbance", frames, action_text=action_text,
                                      words=utterance, notify=bool(f.args.get("notify")),
                                      repeat=bool(f.args.get("repeat")))
    else:
        clock = getattr(intents, "sim_clock", None)
        now = clock() if callable(clock) else getattr(bridge, "sim_time", None)
        if now is None:
            return ("I can't read my clock, so I can't time that.",
                    (f.tool, "unsupported", "no sim clock"))
        delay = float(f.args["delay_s"])
        res = intents.schedule_action("after_s", frames, due_sim=float(now) + delay,
                                      delay_s=delay, action_text=action_text,
                                      words=utterance)
    ok = bool(res.get("accepted"))
    return (str(res.get("say") or ("Scheduled." if ok else "I could not schedule that.")),
            (f.tool, "ok" if ok else "refused",
             str(res.get("id") or res.get("reason") or "")))


def execute(bridge: Any, r: "_i.Interpretation",
            surface: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """Run an interpretation. None = not ours; give it to the model."""
    intents = getattr(bridge, "intents", None)

    if r.intent == getattr(_i, "CONDITIONAL", "conditional"):
        pose = _bridge_pose(bridge)
        chosen = resolve_conditional(r, pose) if pose is not None else None
        if chosen is None:
            return None                  # no measured pose: the model's turn
        said = f"Checked: {chosen.reason}."
        if not chosen.frames:
            return {"agent": said + " Nothing to do.",
                    "tools": [("evaluate_condition", "ok", chosen.reason[:80])]}
        out = execute(bridge, chosen, surface)
        if out is None:
            return None
        return {"agent": said + " " + str(out.get("agent", "")),
                "tools": [("evaluate_condition", "ok", chosen.reason[:80])]
                + list(out.get("tools") or [])}

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
        if getattr(r, "ask", ""):
            return {"agent": r.ask, "tools": [("clarify", "ask", r.residue[:80])]}
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
    # A spatial RULE frame restricts motion and actuates nothing, so it is not
    # the gate's to judge (it would refuse it as unknown_tool); every frame
    # that can move the robot, including one in the same sentence as a rule,
    # is still vetted with the whole utterance.
    # The clause that orders something for LATER is not part of what is done
    # now: "drive 0.5 m now, then 10 s after you arrive, drive back 0.5 m"
    # would otherwise refuse the first drive as deferred.
    now_text = r.text or ""
    for f in r.frames:
        if f.tool in ("schedule", "watch") and f.source:
            now_text = now_text.replace(f.source, " ")
    if any(f.tool == "watch" for f in r.frames):
        now_text = ""                    # a watched order does nothing now
    # A `dwell` (a routine's pause at a stop) moves nothing either; the gate
    # refused it as unknown_tool and every "do the usual run" on the
    # long-horizon dev shift (run 2, 2026-10-02) went nowhere.
    now_frames = [f for f in r.frames if f.tool not in _RULE_FRAMES and f.tool != "dwell"]
    rejections = (_gate.check(now_text, now_frames, surface=surface)
                  if now_frames else [])
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
    if any(f.tool in ("schedule", "watch", "dwell") for f in r.frames):
        # A later action is timed from when the steps before it FINISH, so
        # every immediate step blocks -- and a dwell belongs at its stop.
        last_motion = -1
    if getattr(bridge, "replies_after_motion", False):
        # The last leg used to return at once "to keep chat responsive", and
        # the reply echoed the order before anything happened: a Husky a
        # block stopped at 1.2 m of 2 m was reported as "Drive forward
        # (distance=2.0)." (ops-bench P7). A bridge whose halts and new
        # orders get past its lock (so waiting cannot trap a stop) and whose
        # questions do too (so waiting cannot stall chat) opts in, and every
        # leg is answered from what it measured.
        last_motion = -1
    # AN OPERATOR HALT ENDS THE PLAN, NOT JUST THE LEG. Every leg but the last
    # blocks, and nothing between legs looked at whether the operator had said
    # stop meanwhile, so "drive 0.6, turn 90, drive 0.6" + "Stop!" 0.3 m in
    # halted the first drive and then turned and drove anyway (ops-bench F3,
    # 2026-09-25). A bridge that counts halts (`halt_seq`) lets the loop see
    # one it did not issue itself; this plan's own "... then stop" re-baselines.
    halt_mark = getattr(bridge, "halt_seq", None)
    for idx, f in enumerate(r.frames):
        if halt_mark is not None and getattr(bridge, "halt_seq", None) != halt_mark:
            skipped = [g.tool for g in r.frames[idx:]]
            said.append("You told me to stop, so I did not do the rest of that "
                        "request.")
            tools.append(("plan", "cancelled",
                          "operator_halt: skipped " + ", ".join(skipped)))
            break
        if f.tool == "dwell":
            # Stay at the stop this long (a routine's "five seconds at each
            # stop"), on the robot's own clock when it has one.
            secs = max(0.0, min(600.0, float(f.args.get("s", 0.0))))
            t0 = getattr(bridge, "sim_time", None)
            if isinstance(t0, (int, float)):
                while getattr(bridge, "sim_time", t0) < t0 + secs:
                    if halt_mark is not None and getattr(bridge, "halt_seq", None) != halt_mark:
                        break
                    time.sleep(0.05)
            else:
                time.sleep(secs)
            tools.append(("dwell", "ok", f"{secs:g} s"))
            continue
        if f.tool in ("schedule", "watch"):
            said_one, tool_row = _schedule_frame(bridge, intents, f, r.text or "", surface)
            said.append(said_one)
            tools.append(tool_row)
            continue
        if f.tool in ("boundary", "clear_boundary"):
            if intents is None or not getattr(intents, "spatial", False):
                # Never agree to a rule nothing on this robot enforces.
                said.append("I can't hold a boundary on this robot, so I have not "
                            "agreed to one.")
                tools.append((f.tool, "unsupported", ""))
                continue
            if f.tool == "clear_boundary":
                res = intents.clear_constraint("boundary")
                said.append(res.get("say") or ("Boundary lifted." if res.get("accepted")
                                               else "There was no boundary to lift."))
                tools.append(("clear_boundary", "ok" if res.get("accepted") else "no_action",
                              ", ".join(c.get("means", "") for c in res.get("cleared") or [])))
                continue
            axis, value, side = f.args["axis"], float(f.args["value"]), f.args["side"]
            if side == "current":
                # "Stay behind the line": the side the robot is on NOW,
                # measured, never assumed from which way it happens to face.
                pose = _bridge_pose(bridge)
                if pose is None:
                    said.append("I can't read my position, so I can't tell which "
                                "side of that line I'm on. Tell me which side to keep.")
                    tools.append(("set_boundary", "needs_pose", f"{axis}={value:.2f}"))
                    continue
                here = pose[0] if axis == "x" else pose[1]
                if abs(here - value) < 0.02:
                    said.append(f"I'm right on {axis} = {value:.2f}. Which side "
                                "should I stay on?")
                    tools.append(("set_boundary", "ask", "on_the_line"))
                    continue
                side = "max" if here < value else "min"
            res = intents.set_boundary(axis, max_value=value if side == "max" else None,
                                       min_value=value if side == "min" else None,
                                       words=r.text or "")
            said.append(str(res.get("say", "")))
            tools.append(("set_boundary", "ok" if res.get("accepted") else "refused",
                          str(res.get("means") or res.get("reason", ""))))
            continue
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
        # "Go home" / "return to the dock": DRIVEN where the bridge can
        # (act_return_home), never the supervisor teleport of reset_to_home.
        if f.tool == "reset_to_home" and hasattr(bridge, "act_return_home"):
            method = "act_return_home"
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
        if f.tool == "stop":
            halt_mark = getattr(bridge, "halt_seq", None)

    if r.residue:
        said.append(f"I did not act on '{r.residue[:60]}' - I did not "
                    f"understand that part.")
    return {"agent": " ".join(said), "tools": tools}


def route(bridge: Any, text: str, surface: str = _i.MOBILE
          ) -> Optional[Dict[str, Any]]:
    """Interpret `text` and execute it. None = hand this to the model."""
    capture_site_facts(bridge, text)
    placed = place_order(bridge, text)
    if placed is not None:
        return execute(bridge, placed, surface)
    return execute(bridge, _i.interpret(text, surface), surface)


import re as _re

# "take this to packing", "head back to the dock", "go over to line 2",
# "bring it to the charger", "back to packing please" -- an order to a place
# the robot was TOLD about. The parser cannot know place names (they live in
# the robot's own store), so this reads them from there.
# A job LABEL may lead ("First job:", "Next up -", "Another run:"); it names
# the order, it is not part of it (dev-loop probe P3, 2026-10-01: "First job:
# take this crate to the dock" went to a model and queued a minute).
_PLACE_ORDER = _re.compile(
    r"^(?:(?:(?:first|next|new|another|second|third|last|one more|quick|urgent)\s+)?"
    r"(?:job|task|run|one|thing|order|up)\s*[:\-]\s*)?"
    r"(?:(?:ok(?:ay)?|right|alright|now|next|great|thanks|good|cool|so)[,.!]?\s+)*"
    r"(?:please\s+)?(?:can you\s+|could you\s+)?"
    r"(?:(?:go|head|drive|run|get|move|come)(?:\s+(?:over|back|straight|on|down|up|across|round|along))?\s+to|"
    r"(?:take|bring|carry|drop|run)\s+(?:this|that|it|these|those|the\s+\w+)(?:\s+\w+)?"
    r"\s+(?:(?:over|back|down|up|across|round|along)\s+)?to|back\s+to)\s+(?:the\s+)?"
    r"(?P<place>[a-z0-9][a-z0-9 \-]{0,30}?)\s*(?:,?\s*(?:please|now|thanks|for me))*\s*[.!]*$",
    _re.IGNORECASE)
_NOT_NOW = _re.compile(
    r"\b(?:don'?t|do not|never|no need|avoid|stop going|in \d+|in (?:a|an|one|two|three|"
    r"five|ten|fifteen|twenty|thirty) |at \d|at (?:the )?\w+ (?:mark|past)|when|once|after|"
    r"before|if|unless|until|later|every)\b", _re.IGNORECASE)


_NUM = r"(-?\d+(?:\.\d+)?)"
# "the dock is x=4.5, y=0", "Charger's at x=0, y=3.8", "Line 2 is x = 0, y = -3.8"
_PLACE_DEF = _re.compile(
    r"(?:^|[.;,]\s*|\b(?:and|also)\s+)(?:the\s+)?(?P<name>[A-Za-z][A-Za-z0-9 \-]{0,24}?)"
    r"(?:'s|\s+is|\s+are|\s+sits|\s+lives)\s+(?:at\s+|over\s+at\s+|located\s+at\s+|by\s+)?"
    r"(?:\(\s*)?x\s*[=:]?\s*" + _NUM + r"\s*,?\s*(?:and\s+)?y\s*[=:]?\s*" + _NUM,
    _re.IGNORECASE)
# "Stations today: dock (3.00, 0.00), charger (-2.00, 2.00), scrap bay at (1, 2)"
# -- the list form a shift opens with. Only _PLACE_DEF's "is x=.., y=.." form
# was read until 2026-10-01, so a station list waited for a model turn.
_PLACE_PAREN = _re.compile(
    # A new sentence starts a definition too: "Marco here. The press line is
    # at (2.50, 1.50) today." was missed (slow-model probe, 2026-10-01).
    r"(?:^|[:;,.!]\s*|\b(?:and|also)\s+)(?:the\s+)?(?P<name>[A-Za-z][A-Za-z0-9 \-]{0,24}?)"
    r"\s*(?:is\s+|at\s+|is\s+at\s+)?\(\s*" + _NUM + r"\s*,\s*" + _NUM + r"\s*\)",
    _re.IGNORECASE)
# "x 0.5 to 1.7, y -1.5 to 1.5" / "x between 1 and 2, y from -1 to 3"
_ZONE_BOX = _re.compile(
    r"x\s*(?:from|between|=|:)?\s*" + _NUM + r"\s*(?:to|and|-|–|through)\s*" + _NUM +
    r"\s*,?\s*(?:and\s+)?y\s*(?:from|between|=|:)?\s*" + _NUM + r"\s*(?:to|and|-|–|through)\s*" + _NUM,
    _re.IGNORECASE)
_KEEP_OUT = _re.compile(
    r"\b(?:stay out|keep out|stay clear|keep clear|off[- ]limits|no[- ]go|don'?t (?:enter|go in)|"
    r"do not (?:enter|go in)|out of bounds|pedestrian|fire[- ]door|restricted|closed|forbidden)\b",
    _re.IGNORECASE)
_LOCKED = _re.compile(
    r"\b(?:permanent(?:ly)?|nobody|no one|no-one|no matter who|not even me|whole shift|all shift|"
    r"never,? ever|no exceptions)\b", _re.IGNORECASE)
_ZONE_NAME = _re.compile(
    r"\b((?:pedestrian |fire[- ]door |loading |walk)?(?:aisle|corridor|walkway|zone|area|lane|bay|door))\b",
    _re.IGNORECASE)
_NOT_A_PLACE = {"it", "that", "this", "there", "here", "which", "one", "rule", "aisle",
                "corridor", "walkway", "zone", "area"}


def capture_site_facts(bridge: Any, text: str) -> List[str]:
    """Record the places and keep-out zones a message DEFINES, straight into
    the robot's store, before any model reads it.

    The facts a shift names once in minute one are the ones everything later
    depends on, and a model turn that was slow, or cancelled, lost them
    (ops-bench shift_dev_v1: "line 2" was never saved). The model still sees
    the message and may record the same facts again; that is harmless.
    A zone is only captured with an explicit keep-out cue in the sentence."""
    store = getattr(bridge, "intents", None)
    if store is None or not hasattr(store, "set_place"):
        return []
    body = _i._strip_speaker(text)
    done = []
    try:
        if "?" not in body:
            for m in list(_PLACE_DEF.finditer(body)) + list(_PLACE_PAREN.finditer(body)):
                name = m.group("name").strip(" -")
                # The speaker's own capitals ("Ward 3", "QA bay") are kept for
                # replies; the store keys on the normalised name anyway.
                words = name.split()
                while words and (words[0].lower() in ("and", "also", "stations", "station", "then", "the")
                                 or not any(ch.isalnum() for ch in words[0])):
                    words = words[1:]
                name = " ".join(words)
                if not name or name.lower() in _NOT_A_PLACE or _KEEP_OUT.search(name):
                    continue
                store.set_place(name, float(m.group(2)), float(m.group(3)), words=body[:200])
                done.append(f"place {name}")
        box = _ZONE_BOX.search(body)
        if box and _KEEP_OUT.search(body) and hasattr(store, "set_zone") and "?" not in body:
            x0, x1, y0, y1 = (float(box.group(i)) for i in range(1, 5))
            nm = _ZONE_NAME.search(body)
            name = (nm.group(1) if nm else "keep-out zone").lower()
            known = [z for z in store.zones()
                     if (z["x_min"], z["x_max"], z["y_min"], z["y_max"]) ==
                     (min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1))]
            if not known:
                store.set_zone(name, x0, x1, y0, y1, words=body[:200],
                               locked=bool(_LOCKED.search(body)))
                done.append(f"zone {name}")
    except Exception:                                   # a fact capture never blocks a reply
        return done
    return done


# "Corridor's shut again, cart." / "The wet corridor is open again." -- a
# zone the shift has already NAMED changing state, said without its box.
_ZONE_CLOSED = _re.compile(r"\b(?:shut|closed|off[- ]limits|out of bounds|no[- ]go|keep out|stay out)\b",
                           _re.IGNORECASE)
_ZONE_OPEN = _re.compile(r"\b(?:open|opened|re-?opened|back in use|usable again|clear again)\b", _re.IGNORECASE)
_ZONE_NOT_NOW = _re.compile(
    r"\b(?:not|no longer|isn'?t|aren'?t|won'?t|never|if|when|whenever|until|till|later|tomorrow|will|"
    r"going to|soon|might|maybe|can|could|should|afternoon|tonight)\b", _re.IGNORECASE)


def capture_zone_state(bridge: Any, text: str) -> List[str]:
    """Close or reopen a zone the shift has already named, the moment it is
    said. Long-horizon dev shift, run 3 (2026-10-02): "Corridor's shut
    again, cart, evening crew's started." got "Copy that, standing by" from
    the model and no set_zone; nine minutes later a planned route ran
    straight through the wet corridor. A close needs no authority (anyone
    may make the robot safer); a reopen goes through the store's own lift
    rules, so a worker cannot open what only the supervisor may."""
    store = getattr(bridge, "intents", None)
    if store is None or not hasattr(store, "known_zones"):
        return []
    body = _i._strip_speaker(text).strip()
    if "?" in body or _ZONE_BOX.search(body) or _ZONE_NOT_NOW.search(body):
        return []
    closed, opened = bool(_ZONE_CLOSED.search(body)), bool(_ZONE_OPEN.search(body))
    if closed == opened:
        return []
    try:
        from omnisim_bridges.intents import normalize_place
    except ImportError:                                 # pragma: no cover
        return []
    known = store.known_zones()
    padded = f" {normalize_place(body)} "
    hits = [k for k in known if f" {k} " in padded]
    if not hits:
        # "the corridor" for the one zone whose name ends in "corridor"
        heads = {}
        for k in known:
            heads.setdefault(k.split()[-1], []).append(k)
        hits = [ks[0] for h, ks in heads.items() if len(ks) == 1 and f" {h} " in padded]
    if len(hits) != 1:
        return []
    z = known[hits[0]]
    name = z.get("name") or hits[0]
    if closed and z.get("status") != "active":
        out = store.set_zone(name, z["x_min"], z["x_max"], z["y_min"], z["y_max"], words=body[:200])
        return [f"zone {name}"] if out.get("accepted", True) else []
    if opened and z.get("status") == "active" and not z.get("locked"):
        out = store.clear_constraint(z["id"])
        if out.get("cleared"):
            return [f"reopen {name}"]
        return [f"reopen-refused {name}={out.get('reason') or 'not yours to lift'}"]
    return []


# Introductions: "Dana here, shift supervisor", "Priya, safety lead",
# "I'm Owen, I run the floor", "Marco, I run the presses", "Priya's our safety
# lead". The speaker comes from the radio prefix; a role is only taken from a
# sentence that NAMES its person (the speaker, or another capitalised name).
_ROLE_PHRASE = _re.compile(
    r"\b(safety (?:lead|officer|manager)|safety|shift supervisor|supervisor|shift lead|"
    r"shift manager|foreman|charge nurse|charge hand|in charge|running the (?:floor|shift)|run(?:s)? the (?:floor|shift)|"
    r"floor worker|worker|operator|picker|packer|loader|i run the \w+|i work (?:on|in) the \w+)\b",
    _re.IGNORECASE)
_OTHER_ROLE = _re.compile(
    r"\b(?P<name>[A-Z][a-z]+)(?:'s| is)\s+(?:our|the|your)\s+(?P<role>[a-z ]{3,30}?)(?:[.,;!]|$)")
_SENTENCES = _re.compile(r"(?<=[.!;])\s+")
_GREETING_ONLY = _re.compile(
    r"^\s*(?:(?:good\s+)?(?:morning|afternoon|evening)|hi|hello|hey|hiya|alright|right|ok(?:ay)?|"
    r"thanks|cheers)(?:[ ,]+(?:cart|robot|bot|there|all|team|everyone|mate))?\s*[.!,]*\s*$",
    _re.IGNORECASE)


# "hold where you are, don't move until I say", "stay put till I'm back",
# "wait right there until I tell you". The resume condition is required by
# IntentStore.hold_now itself (a bare "stop" is not a hold).
_HOLD_NOW = _re.compile(
    r"\b(?:hold (?:where you are|it there|it right there|right there|there|your position|position|still)|"
    r"stay (?:put|where you are|right there|there)|don'?t move|do not move|"
    r"wait (?:right )?(?:here|there|where you are))\b",
    _re.IGNORECASE)
_CONDITIONAL_START = _re.compile(r"^\s*(?:if|when|whenever|once|after|in case)\b", _re.IGNORECASE)


def capture_hold(bridge: Any, text: str) -> List[str]:
    """Engage an operator hold the moment it is SAID.

    Until 2026-10-01 a hold existed only once a model turn called
    hold_until_told -- 27 s after "Priya: hold where you are, don't move
    until I say" in dev-loop probe P2 -- and another person's drive order,
    parsed and run in that gap, drove the robot off (unsafe). A safety
    instruction cannot wait for a model. Questions and conditions ("if you
    see a spill, hold there until...") are left to the model."""
    store = getattr(bridge, "intents", None)
    if store is None or not hasattr(store, "hold_now"):
        return []
    body = _i._strip_speaker(text)
    first = _SENTENCES.split(body.strip())[0] if body.strip() else ""
    if "?" in body or not _HOLD_NOW.search(body) or _CONDITIONAL_START.match(first):
        return []
    try:
        st = store.hold_now(words=body)
    except Exception:                                   # a capture never blocks a reply
        return []
    return ["hold"] if st.get("accepted") else []


def capture_roles(bridge: Any, text: str) -> List[str]:
    """Record who is who from an introduction, before any model reads it."""
    store = getattr(bridge, "intents", None)
    if store is None or not hasattr(store, "set_role"):
        return []
    try:
        from omnisim_bridges.intents import speaker_of, normalize_role
    except ImportError:                                 # pragma: no cover
        return []
    speaker = speaker_of(text)
    body = _i._strip_speaker(text)
    done = []
    if "?" in body:
        return done
    for sentence in _SENTENCES.split(body):
        low = sentence.lower()
        role = _ROLE_PHRASE.search(sentence)
        names_self = speaker and (speaker.lower() in low
                                  or _re.search(r"\b(?:i'?m|i am|this is)\b|\bhere\b", low))
        if role and names_self:
            phrase = role.group(1).lower()
            r = "worker" if phrase.startswith(("i run the", "i work")) else normalize_role(phrase)
            if r and r != "other" and store.roles.get(speaker) != r:
                store.set_role(speaker, r)
                done.append(f"role {speaker}={r}")
            continue
        for m in _OTHER_ROLE.finditer(sentence):
            r = normalize_role(m.group("role"))
            if r and r != "other" and m.group("name") != speaker and store.roles.get(m.group("name")) != r:
                store.set_role(m.group("name"), r)
                done.append(f"role {m.group('name')}={r}")
    return done


# "Marco here." / "It's Dana." -- a name and nothing else asks for nothing.
_NAME_ONLY = _re.compile(r"^\s*(?:it'?s\s+|this is\s+)?[A-Z][a-z]+(?:\s+here)?\s*[.!]*\s*$")


def _fact_only(body: str) -> bool:
    """True when every sentence is a definition, an introduction or a
    greeting: nothing in it asks the robot to DO anything, so nothing is lost
    by answering it without a model. Conservative on purpose -- one sentence
    it cannot place sends the whole message to the model, as before."""
    if "?" in body:
        return False
    for sentence in _SENTENCES.split(body.strip()):
        s = sentence.strip()
        if not s or _GREETING_ONLY.match(s) or _NAME_ONLY.match(s):
            continue
        rest = _PLACE_PAREN.sub(" ", _PLACE_DEF.sub(" ", s))
        if _ZONE_BOX.search(rest) and _KEEP_OUT.search(rest):
            continue
        defined = rest != s
        intro = bool(_ROLE_PHRASE.search(s)) and not _MOTION_VERB.match(s) and \
            not _re.search(r"\b(?:take|bring|run (?:this|that|it)|go|drive|fetch|move|deliver|head|"
                           r"park|send|stop|wait|hold|don'?t|never|always|when|if|until)\b", s, _re.IGNORECASE)
        if intro:
            continue
        if defined:
            leftover = _re.sub(r"\b(?:stations?|today|places?|are|is|at|the|and|our|we have|here|"
                               r"for (?:the )?(?:shift|day))\b", " ", rest, flags=_re.IGNORECASE)
            if not _re.search(r"[A-Za-z]{3,}", leftover):
                continue
        return False
    return True


def answer_facts(bridge: Any, text: str, facts: List[str]) -> Optional[Dict[str, Any]]:
    """A message that only TELLS the robot things, answered from what was
    recorded. On a shift the opening minute is introductions, stations and
    rules a few seconds apart; sent through a model one by one they queued
    behind each other and the first job started over a minute late
    (dev-loop probe P3, 2026-10-01). Recorded facts are read back from the
    store, so the reply says only what was actually stored."""
    if not facts or not _fact_only(_i._strip_speaker(text)):
        return None
    said = _facts_said(bridge, facts)
    if not said:
        return None
    text_out = "Got it: " + "; ".join(said) + "."
    return {"agent": text_out[0].upper() + text_out[1:],
            "tools": [("remember", "ok", f) for f in facts]}


def _facts_said(bridge: Any, facts: List[str]) -> List[str]:
    """What was recorded, in words, read back from the store."""
    store = bridge.intents
    said = []
    for f in facts:
        if f.startswith("handover "):
            who = f.split(" ", 1)[1].split("->")[-1]
            said.append(f"{who} is in charge of the floor now")
        elif f.startswith("routine "):
            said.append(f"'{f.split(' ', 1)[1]}' noted")
    roles = [f.split(" ", 1)[1] for f in facts if f.startswith("role ")]
    places = [f.split(" ", 1)[1] for f in facts if f.startswith("place ")]
    zones = [f.split(" ", 1)[1] for f in facts if f.startswith("zone ")]
    for f in facts:
        if f.startswith("reopen "):
            said.append(f"the {f.split(' ', 1)[1]} is open again, so I can use it")
        elif f.startswith("reopen-refused "):
            z, why = f.split(" ", 1)[1].split("=", 1)
            said.append(f"I'm still keeping out of the {z}: {why}")
    if "hold" in facts:
        said.append("I'm holding right here until you say")
    from omnisim_bridges.intents import ROLE_LABELS
    took_over = {f.split("->")[-1] for f in facts if f.startswith("handover ")}
    for r in roles:
        who, role = r.split("=")
        if who in took_over:
            continue                      # already said: "<who> is in charge of the floor now"
        said.append(f"{who} is the {ROLE_LABELS.get(role, role)}")
    if places:
        said.append("stations noted: " + ", ".join(places))
    for z in zones:
        rule = next((c for c in reversed(store.zones()) if c.get("name") == z), None)
        locked = bool(rule and rule.get("locked"))
        said.append(f"I'll keep out of the {z}" + (" for the whole shift" if locked else " until it's lifted"))
    return said


# Cues that a message about a station is a request to go there now:
# "Paint shop next.", "Tool crib, please.", "Paint shop again. They want the
# masking roll.", "Right, QA bay with the racks. Go.", "Park up at the press
# line", "Press line needs you".
_REQUEST_CUE = _re.compile(
    r"\b(?:take|bring|carry|drop|run|go|head|drive|come|get|park|deliver|send|move|return|"
    r"back to|next|again|please|needs? you|wants? (?:you|this|these|it|them|the)|over to|round to)\b",
    _re.IGNORECASE)
# Anything that changes HOW or WHETHER, which only a model may weigh: route
# instructions ("straight through the lane"), negations, corrections,
# conditions, keep-out talk.
_REQUEST_VETO = _re.compile(
    r"\b(?:through|via|straight|shortcut|short cut|cut across|across the|don'?t|do not|never|not|no|"
    # "instead" / "actually" / "belay that" with ONE station is a new
    # destination (dev loop 5: "Belay that, head to line 4 instead." timed out
    # in a model); a worker's override is still refused where motion starts.
    r"forget|cancel|unless|except|without|"
    r"if|when|whenever|once|after|before|until|till|later|minutes?|seconds?|hours?|o'?clock)\b",
    _re.IGNORECASE)
_POLITE_TAIL = _re.compile(
    r"(?:,?\s*(?:would|could|will|can)\s+you(?:\s+please)?|,?\s*please|\s+for\s+me(?:\s+please)?)\s*\?\s*$",
    _re.IGNORECASE)
_POLITE_HEAD = _re.compile(r"^\s*(?:(?:can|could|would|will)\s+you\s+(?:please\s+)?)", _re.IGNORECASE)
_THANKS_TAIL = _re.compile(r",?\s*(?:thanks|thank you|cheers|ta)\s*[.!]?\s*$", _re.IGNORECASE)


# "When I say 'the usual run' I mean Pharmacy, then Ward 3, then the Lab, in
# that order, with five seconds at each stop." / "The morning round is A, B, C."
_ROUTINE_DEF = _re.compile(
    r"(?:when i say\s+|by\s+)?['\"\u2018\u2019\u201c\u201d]?(?P<name>(?:the\s+)?[\w ]{2,30}?\s*(?:run|round|loop|route|circuit))"
    r"['\"\u2018\u2019\u201c\u201d]?\s*,?\s*(?:i mean|means|is|=|:|goes)\s+(?P<seq>.+)", _re.IGNORECASE)
_NUMWORD = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8,
            "nine": 9, "ten": 10, "fifteen": 15, "twenty": 20, "thirty": 30}
_DWELL = _re.compile(r"\b(?P<n>\d+(?:\.\d+)?|one|two|three|four|five|six|seven|eight|nine|ten|fifteen|twenty|thirty)"
                     r"\s+(?:seconds?|secs?)\s+(?:at|in)\s+each\b", _re.IGNORECASE)
_HANDOVER_IN = _re.compile(r"\btak(?:ing|e|en) over from\s+(?P<who>[A-Z][a-z]+)", 0)
_HANDOVER_OUT = _re.compile(r"\b(?:i'?m off|signing off|handing over)\b.*?\b(?P<who>[A-Z][a-z]+)(?:'s| has| is)\s+"
                            r"(?:got\s+)?(?:the floor|in charge|got it|taking over|on)", _re.IGNORECASE)


def _places_in_order(store: Any, text: str) -> List[str]:
    try:
        from omnisim_bridges.intents import normalize_place
    except ImportError:                                 # pragma: no cover
        return []
    padded = f" {normalize_place(text)} "
    found = []
    for k in getattr(store, "places", {}) or {}:
        if not k:
            continue
        pos = padded.find(f" {k} ")
        if pos >= 0:
            found.append((pos, k))
    keys = [k for _, k in sorted(found)]
    return [k for k in keys if not any(k != o and f" {k} " in f" {o} " for o in keys)]


def capture_shift_facts(bridge: Any, text: str) -> List[str]:
    """Routines and handovers, recorded the moment they are said."""
    store = getattr(bridge, "intents", None)
    if store is None or not hasattr(store, "set_routine"):
        return []
    body = _i._strip_speaker(text)
    done = []
    if "?" in body:
        return done
    m = _ROUTINE_DEF.search(body)
    if m:
        stops = _places_in_order(store, m.group("seq"))
        if len(stops) >= 2:
            dm = _DWELL.search(m.group("seq"))
            dwell = 0.0
            if dm:
                n = dm.group("n").lower()
                dwell = float(_NUMWORD.get(n, n)) if n in _NUMWORD or n.replace(".", "", 1).isdigit() else 0.0
            if store.set_routine(m.group("name"), stops, dwell, words=body[:200]).get("accepted"):
                done.append(f"routine {m.group('name').strip()}")
    try:
        from omnisim_bridges.intents import speaker_of
    except ImportError:                                 # pragma: no cover
        return done
    speaker = speaker_of(text)
    hi = _HANDOVER_IN.search(body)
    ho = _HANDOVER_OUT.search(body)
    if hi and speaker:
        store.handover(hi.group("who"), speaker)
        done.append(f"handover {hi.group('who')}->{speaker}")
    elif ho and speaker:
        store.handover(speaker, ho.group("who"))
        done.append(f"handover {speaker}->{ho.group('who')}")
    return done


_ROUTINE_CUE = _re.compile(r"\b(?:do|run|start|go on|time for|off you go on|again|please|next)\b", _re.IGNORECASE)
# "The usual run is boring." -- the routine as a sentence's SUBJECT is talk.
_ROUTINE_SUBJECT = _re.compile(r"\b(?:run|round|loop|route|circuit)\s+(?:is|was|'s|has|takes|took)\b", _re.IGNORECASE)


def routine_order(bridge: Any, text: str) -> Optional[_i.Interpretation]:
    """"Do the usual run." -> the stored stops, with the stored dwell."""
    store = getattr(bridge, "intents", None)
    routines = getattr(store, "routines", None) if store is not None else None
    if not routines or not hasattr(bridge, "act_go_to_place"):
        return None
    body = _i._strip_speaker(text).strip()
    if ("?" in body or _REQUEST_VETO.search(body) or not _ROUTINE_CUE.search(body)
            or _ROUTINE_SUBJECT.search(body)):
        return None
    try:
        from omnisim_bridges.intents import normalize_routine
    except ImportError:                                 # pragma: no cover
        return None
    words = f" {normalize_routine(body)} "
    hits = [k for k in routines if k and f" {k} " in words]
    if len(hits) != 1:
        return None
    rt = routines[hits[0]]
    frames = []
    for idx, stop in enumerate(rt["stops"]):
        frames.append(_i.Frame("go_to_place", {"place": stop}, rule="routine"))
        if rt.get("dwell_s"):
            frames.append(_i.Frame("dwell", {"s": float(rt["dwell_s"])}, rule="routine"))
    return _i.Interpretation(_i.COMMAND, frames=frames, reason=f"routine {rt['name']}",
                             confidence=0.9, text=text)


_ROLE_WORDS = {"supervisor": "the supervisor", "safety": "the safety lead"}


def zone_shortcut_order(bridge: Any, text: str) -> Optional[Tuple[str, _i.Interpretation]]:
    """"Label rolls to the Pharmacy. And go straight through the MRI suite,
    we're behind." -> refuse the shortcut, in words that say why, and make
    the delivery by the planned route.

    Only when the shortcut is through a zone the store has ACTIVE and the
    speaker cannot lift: a locked zone (nobody can), or an open-able one
    asked by someone without the role to open it. A supervisor asking to cut
    through a zone they could lift stays with the model. Long-horizon dev
    shift, run 2 (2026-10-02): the model refused and went round, but said
    "a permanent keep-out zone" and not that nobody, the supervisor
    included, can lift it -- which is the part the person asking needs.
    """
    store = getattr(bridge, "intents", None)
    if store is None or not hasattr(store, "zones") or not hasattr(bridge, "act_go_to_place"):
        return None
    try:
        from omnisim_bridges.intents import normalize_place, speaker_of
    except ImportError:                                 # pragma: no cover
        return None
    body = _i._strip_speaker(text).strip()
    if "?" in body:
        return None
    for z in store.zones():
        name = str(z.get("name") or "").strip()
        if not name:
            continue
        pat = (r"[,;]?\s*(?:and\s+)?(?:just\s+)?(?:go\s+|cut\s+|drive\s+|head\s+|nip\s+)?(?:straight\s+)?"
               r"(?:through|across|via)\s+(?:the\s+)?" + _re.escape(name) + r"\b[^.!?]*[.!?]?")
        if not _re.search(pat, body, _re.IGNORECASE):
            continue
        who = speaker_of(text)
        roles = getattr(store, "roles", {}) or {}
        if not z.get("locked"):
            lifters = list(z.get("lift_roles") or getattr(store, "LIFT_ROLES", ()) or ())
            if roles.get(who) in lifters or (who and who == z.get("set_by")):
                return None
        rest = _re.sub(pat, " ", body, flags=_re.IGNORECASE)
        rest = _re.sub(r"\s+", " ", rest).strip(" ,;")
        r = place_request(bridge, f"{who}: {rest}" if who else rest)
        if r is None or len(r.frames) != 1:
            return None
        place = r.frames[0].args.get("place", "")
        pname = (getattr(store, "places", {}).get(normalize_place(place)) or {}).get("name") or place
        if z.get("locked"):
            why = (f"I won't go through the {name}: it's off limits for the whole shift, and nobody "
                   f"can lift that, not even the supervisor.")
        else:
            lifters = list(z.get("lift_roles") or getattr(store, "LIFT_ROLES", ()) or ())
            can = " or ".join(_ROLE_WORDS.get(x, x) for x in lifters) or "the supervisor"
            why = f"I won't cut through the {name}: it's closed, and only {can} can open it."
        r.text = rest
        return why + f" I'll go round to the {pname} instead.", r
    return None


_ASK_IN_CHARGE = _re.compile(
    r"\bwho(?:'s| is| has)\s+(?:in charge|running (?:the|this) (?:floor|shift)|got the floor|the supervisor|"
    r"the charge nurse|the boss|on the floor now|giving (?:the )?orders)\b", _re.IGNORECASE)


def answer_charge_query(bridge: Any, text: str) -> Optional[Dict[str, Any]]:
    """"Who's in charge of the floor now?" -- from the recorded roles, with
    the handover that made it so. Run 2 of the long-horizon dev shift
    (2026-10-02): the model named Tobias but not that he took over from
    Marisol, which is the half the asker needed."""
    store = getattr(bridge, "intents", None)
    roles = dict(getattr(store, "roles", {}) or {}) if store is not None else {}
    body = _i._strip_speaker(text).strip()
    if "?" not in body or not _ASK_IN_CHARGE.search(body):
        return None
    bosses = [n for n, r in roles.items() if r == "supervisor"]
    if len(bosses) != 1:
        return None
    gone = [n for n, r in roles.items() if r == "former"]
    said = f"{bosses[0]} is in charge of the floor now"
    if len(gone) == 1:
        said += f": {bosses[0]} took over from {gone[0]}, who has gone off shift"
    return {"agent": said + ".", "tools": [("get_shift_memory", "ok", f"supervisor {bosses[0]}")]}


def answer_routine_query(bridge: Any, text: str) -> Optional[Dict[str, Any]]:
    """"What's in the usual run again?" -- from the stored routine."""
    store = getattr(bridge, "intents", None)
    routines = getattr(store, "routines", None) if store is not None else None
    if not routines:
        return None
    body = _i._strip_speaker(text).strip()
    if "?" not in body or not _re.search(r"\b(?:what'?s|what is|what are|which|remind me)\b", body, _re.IGNORECASE):
        return None
    try:
        from omnisim_bridges.intents import normalize_routine
    except ImportError:                                 # pragma: no cover
        return None
    words = f" {normalize_routine(body)} "
    hits = [k for k in routines if k and f" {k} " in words]
    if len(hits) != 1:
        return None
    rt = routines[hits[0]]
    names = [store.places.get(s, {}).get("name", s) for s in rt["stops"]]
    said = (f"{rt['name'][0].upper() + rt['name'][1:]} is " + ", then ".join(names)
            + (f", with {rt['dwell_s']:g} seconds at each stop." if rt.get("dwell_s") else "."))
    return {"agent": said, "tools": [("get_shift_memory", "ok", f"routine {rt['name']}")]}


def place_request(bridge: Any, text: str) -> Optional[_i.Interpretation]:
    """Go to the ONE station a message names, when it is plainly a request.

    _PLACE_ORDER reads one fixed shape ("take this to the dock"). A shift
    says it a dozen other ways, and every one of them went to a model: 30-75
    seconds per order on dev loop 1 (2026-10-01), long enough that the robot
    was still on the previous run when it was meant to be parked, and that a
    "stop" timed to the next drive never came. Narrow on purpose: exactly one
    known station, a request cue, and nothing that changes how or whether
    (see _REQUEST_VETO) -- those stay with the model and the gate."""
    store = getattr(bridge, "intents", None)
    places = getattr(store, "places", None) if store is not None else None
    if not places or not hasattr(bridge, "act_go_to_place"):
        return None
    body = _i._SOON.sub(" ", _i._strip_speaker(text)).strip()
    if _POLITE_HEAD.match(body):
        # "Can you come back to the press line? I've got offcuts." -- the
        # first "?" closes the request itself, not a question.
        body = body.replace("?", ".", 1)
    body = _POLITE_HEAD.sub("", _POLITE_TAIL.sub(".", body)).strip()
    # "Blood tubes to Ward 3, thanks." went to a model, which reported an
    # arrival it never drove (long-horizon dev shift, run 2).
    body = _THANKS_TAIL.sub(".", body).strip()
    if "?" in body or _REQUEST_VETO.search(body) or _KEEP_OUT.search(body) or _ZONE_BOX.search(body):
        return None
    # "Delivery notes to Ward 3." -- a thing TO a station is a request too.
    to_cue = _re.search(r"\bto\s+(?:the\s+)?[\w ]{2,30}[.!]?\s*$", body) is not None
    if not (_REQUEST_CUE.search(body) or to_cue):
        return None
    try:
        from omnisim_bridges.intents import normalize_place
    except ImportError:                                 # pragma: no cover
        return None
    words = normalize_place(body)
    padded = f" {words} "
    hits = [k for k in places if k and f" {k} " in padded]
    # "press line" also contains "press"; keep the longest names only.
    hits = [k for k in hits if not any(k != o and f" {k} " in f" {o} " for o in hits)]
    if len(hits) == 2:
        # "Take the vaccine box FROM Pharmacy TO Ward 3" -- a two-stop
        # delivery, read as an arm's pick-and-place until 2026-10-02.
        a, b = _places_in_order(store, body)[:2] if len(_places_in_order(store, body)) >= 2 else (None, None)
        if a and b and f" from {a} " in padded and f" to {b} " in padded:
            return _i.Interpretation(_i.COMMAND, frames=[
                _i.Frame("go_to_place", {"place": a}, rule="place_request"),
                _i.Frame("go_to_place", {"place": b}, rule="place_request")],
                reason="place_request from-to", confidence=0.85, text=text)
        return None
    if len(hits) != 1:
        return None
    return _i.Interpretation(_i.COMMAND, frames=[_i.Frame("go_to_place", {"place": hits[0]},
                                                          rule="place_request")],
                             reason="place_request", confidence=0.85, text=text)


_ASK_DISTANCE = _re.compile(
    r"\bhow (?:far|many (?:metres|meters|m))\b.*\b(?:driv|travel|cover|go|gone|come|done)\w*"
    r"|\b(?:distance|metres|meters|mileage)\b.*\b(?:driv|travel|cover|done|today|shift)\w*",
    _re.IGNORECASE)
_ASK_VISITS = _re.compile(
    r"\bhow (?:many times|often)\b.*\b(?:go|went|been|visit|visited|stop|stopped|drive|drove|run|ran)\b",
    _re.IGNORECASE)


# "Leak's sorted. You're clear to move." / "False alarm -- carry on with what
# you were doing." / "All clear, back to work."
_RELEASE = _re.compile(
    r"\b(?:(?:you'?re|you are|all)\s+clear(?:\s+to\s+(?:move|go))?|clear\s+to\s+(?:move|go)|carry on|"
    r"as you were|back to (?:work|it)|resume(?: work| your work)?|you can (?:move|go|carry on)(?: again| now)?|"
    r"good to go|crack on|get going again)\b", _re.IGNORECASE)
_RELEASE_VETO = _re.compile(r"\b(?:don'?t|do not|not yet|never|until|unless|wait)\b", _re.IGNORECASE)


def answer_release(bridge: Any, text: str) -> Optional[Dict[str, Any]]:
    """Lift a hold, or end an operator stop, the moment it is said.

    Holds are engaged deterministically (capture_hold); until 2026-10-01 they
    could only be LIFTED by a model calling resume_autonomy. On dev loop 4 that
    turn went unanswered, the hold never lifted, and every later job was
    refused "holding for Priya" -- three failed deliveries. And "carry on with
    what you were doing" after a stop got no reply and no resumed run.

    Only someone who may lift it: the person who asked for the hold, or the
    safety lead or supervisor. Anyone else is told who can."""
    store = getattr(bridge, "intents", None)
    if store is None or not hasattr(bridge, "act_resume_autonomy"):
        return None
    body = _i._strip_speaker(text).strip()
    if "?" in body or not _RELEASE.search(body) or _RELEASE_VETO.search(body):
        return None
    import time as _time
    held = bool(store.hold_active()) if hasattr(store, "hold_active") else False
    stopped = float(getattr(bridge, "stop_hold_until", 0.0) or 0.0) > runtime_clock.now()
    order = getattr(bridge, "_last_order", None)
    if not (held or stopped or order):
        return None
    if held:
        try:
            from omnisim_bridges.intents import speaker_of
        except ImportError:                             # pragma: no cover
            return None
        who, holder = speaker_of(text), getattr(store, "hold_speaker", "")
        if holder and who and who != holder and store.roles.get(who) not in ("safety", "supervisor"):
            return {"agent": f"Sorry {who}, {holder} asked me to hold here, so only {holder}, the safety lead "
                             "or the supervisor can clear me.",
                    "tools": [("resume_autonomy", "refused", f"hold belongs to {holder}")]}
    res = bridge.act_resume_autonomy() or {}
    parts = ["Thanks, I'm clear to move again." if held else "Carrying on."]
    if res.get("say_waiting"):
        parts.append(str(res["say_waiting"]))
    tools = [("resume_autonomy", "ok", "hold released" if res.get("hold_released") else "resumed")]
    if res.get("resumed_order") or res.get("achieved_xy"):
        measured = _measured_drive_to(res)
        if measured is not None:
            parts.append(measured[0])
            tools.append(("go_to_place", measured[1], measured[2]))
    return {"agent": " ".join(parts), "tools": tools}


def answer_self_query(bridge: Any, text: str) -> Optional[Dict[str, Any]]:
    """"How far have you driven this shift?" / "How many times did you go to
    the press line today?" -- answered from the robot's own MEASURED shift
    memory (odometer, arrivals per place), never from a model's reading of
    whichever tool it happened to call. On dev loop 2 (2026-10-01) the model
    read the live state, which carries no odometer, and said it could not
    know; on another run it read shift memory and answered. A number the
    robot measures should not depend on which tool a model picks."""
    store = getattr(bridge, "intents", None)
    if store is None or not hasattr(store, "shift_memory"):
        return None
    body = _i._strip_speaker(text).strip()
    if "?" not in body and not body.lower().startswith(("how ", "tell me how")):
        return None
    if _ASK_DISTANCE.search(body) and not _ASK_VISITS.search(body):
        m = store.shift_memory()
        return {"agent": f"I've driven {m['odometer_m']:.1f} metres this shift, measured from my own position track.",
                "tools": [("get_shift_memory", "ok", f"odometer {m['odometer_m']:.2f} m")]}
    if _ASK_VISITS.search(body):
        try:
            from omnisim_bridges.intents import normalize_place
        except ImportError:                             # pragma: no cover
            return None
        m = store.shift_memory()
        padded = f" {normalize_place(body)} "
        hits = [k for k in m["places"] if k and f" {k} " in padded]
        hits = [k for k in hits if not any(k != o and f" {k} " in f" {o} " for o in hits)]
        if len(hits) != 1:
            return None
        k = hits[0]
        n = int(m["arrivals"].get(k, 0))
        name = m["places"][k].get("name") or k
        return {"agent": f"I've arrived at the {name} {n} time{'s' if n != 1 else ''} this shift, counted from my measured position.",
                "tools": [("get_shift_memory", "ok", f"arrivals {name}={n}")]}
    return None


_ASK_LATER_JOB = _re.compile(
    r"\b(?:for later|later this|set up for later|scheduled|(?:job|run|order|task|check-?in)\s+(?:i|we|you)\s+"
    r"(?:gave|set|set up|left|asked|booked)|that (?:check-?in|timed|later) )", _re.IGNORECASE)
_ASK_STATUS = _re.compile(
    r"\b(?:still on|still happening|still scheduled|still planned|still going ahead|did (?:it|that) "
    r"(?:actually )?(?:happen|go|run|get done)|happen(?:ed)? on schedule|done yet|has it (?:run|happened))\b",
    _re.IGNORECASE)


# "did the cold-chain log make it to the Lab on time?" / "did they get there?"
_ASK_DELIVERED = _re.compile(
    r"\b(?:did|has|have|were|was)\b[^?]*\b(?:make it|made it|get there|got there|get to|got to|reach(?:ed)?|"
    r"arrived?|deliver(?:ed)?|go out|go off|on time)\b", _re.IGNORECASE)
_STOP_WORDS = frozenset(
    "the a an to for of at in on and or that this these those them they it its did does do has have was were "
    "make made get got there here time take took bring brought you your i me my we our her his their note "
    "left about from with cart please".split())


def _content_words(text: str) -> set:
    return {w for w in _re.findall(r"[a-z][a-z\-]+", (text or "").lower())
            if len(w) > 2 and w not in _STOP_WORDS}


def _the_order_named(store: Any, body: str, orders: List[dict]) -> Optional[dict]:
    try:
        from omnisim_bridges.intents import normalize_place
    except ImportError:                                 # pragma: no cover
        return None
    asked = _content_words(body)
    padded = f" {normalize_place(body)} "
    scored = []
    for o in orders:
        said = " ".join(str(o.get(k) or "") for k in ("words", "action_text"))
        dest = f" {normalize_place(o.get('means', ''))} "
        place = any(k and f" {k} " in padded and f" {k} " in dest for k in getattr(store, "places", {}) or {})
        scored.append((2 * len(asked & _content_words(said)) + int(place), o))
    scored.sort(key=lambda t: -t[0])
    if not scored or scored[0][0] == 0 or (len(scored) > 1 and scored[1][0] == scored[0][0]):
        return None
    return scored[0][1]


def answer_timed_query(bridge: Any, text: str) -> Optional[Dict[str, Any]]:
    """"That job I gave you for later. Still on?" / "Did that check-in
    happen on schedule?" -- answered from the robot's own record of its
    timed orders. On dev loop 2 (2026-10-01) the model read an empty
    `pending_intents` list, missed `scheduled_actions` beside it, and told
    the supervisor a job still pending had been cancelled. Only when
    exactly ONE timed order exists; otherwise the model chooses."""
    store = getattr(bridge, "intents", None)
    if store is None or not hasattr(store, "timed_orders"):
        return None
    body = _i._strip_speaker(text).strip()
    if "?" not in body:
        return None
    later = bool(_ASK_LATER_JOB.search(body) and _ASK_STATUS.search(body))
    delivered = bool(_ASK_DELIVERED.search(body))
    if not (later or delivered):
        return None
    orders = store.timed_orders()
    if later and len(orders) == 1:
        o = orders[0]
    else:
        # "Did the cold-chain log make it to the Lab on time?" with several
        # timed orders on the books (long-horizon dev shift, run 2: the model
        # answered nothing once and "still pending" for a delivery already
        # made). The one the question NAMES -- its items first, its place
        # second -- or nobody.
        o = _the_order_named(store, body, orders)
        if o is None:
            return None
    st, means = o["status"], o["means"]
    what = means.split(", ", 1)[-1]                     # "in 840 s ..., go to qa" -> "go to qa"
    asked_if_done = delivered or bool(_re.search(
        r"\b(?:happen|happened|done yet|has it run|get done|go\b|run\b)", body, _re.IGNORECASE))
    if st == "firing":
        said = f"Yes, it went off on schedule and I'm doing it now: {what}."
    elif st == "pending":
        when = means.split(", ", 1)[0]
        said = (f"Not yet -- it's still scheduled: {what}, {when}." if asked_if_done
                else f"Yes, it's still on: {what}, {when}.")
    elif st == "done":
        said = f"Yes, it happened on schedule: {what} -- {o['last_result'] or 'completed'}."
    elif st == "failed":
        said = f"It fired on schedule but did not complete: {o['last_result'] or 'no result recorded'}."
    elif st == "cancelled":
        said = f"No, it was cancelled ({o['detail'] or 'by an operator stop'})."
    else:
        return None
    return {"agent": said, "tools": [("list_pending_intents", "ok", f"{o['id']} {st}")]}


def place_order(bridge: Any, text: str) -> Optional[_i.Interpretation]:
    """A parsed go_to_place, or None to leave the sentence to the parser."""
    store = getattr(bridge, "intents", None)
    if store is None or not getattr(store, "places", None) or not hasattr(bridge, "act_go_to_place"):
        return None
    body = _i._strip_speaker(text).strip()
    if _i._SOON.search(body):
        # "when you get a sec" / "when you're done there" means SOON, not a
        # condition to wait for (interpret._SOON) -- but "done there" is
        # AFTER the current job, so the parser only takes it while the robot
        # is idle; a busy robot leaves it to the model. Until 2026-10-01 the
        # bare "when" sent it to a model, which scheduled it behind a trigger
        # it invented (dev-loop probe P3: "as soon as the bin is loaded and
        # bumps the platform") and never went.
        motion = getattr(bridge, "motion", None)
        if isinstance(motion, tuple) and motion and motion[0] not in ("idle", None):
            return None
        body = _re.sub(r"^[\s,;:.\-]+", "", _i._SOON.sub(" ", body)).strip()
        body = _re.sub(r"\s+([,.!])", r"\1", _re.sub(r"\s+", " ", body))
    if "?" in body or _NOT_NOW.search(body):
        # A polite tag ("..., would you?", "... for me?") is not a question;
        # place_request has the stricter vetoes for everything else.
        return place_request(bridge, text)
    m = _PLACE_ORDER.match(body)
    if not m:
        return place_request(bridge, text)
    name = m.group("place").strip()
    if store.place_xy(name) is None:
        return None
    return _i.Interpretation(_i.COMMAND, frames=[_i.Frame("go_to_place", {"place": name},
                                                          rule="place_order")],
                             reason="place_order", confidence=0.9, text=text)


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


def parser_will_act(bridge: Any, text: Any, surface: str = _i.MOBILE) -> bool:
    """Will short_circuit answer `text` itself, as an order, right now?

    Pure: no statistics, no store writes, no actuation. For a bridge whose
    running turn was just SUPERSEDED by this message (an implicit halt): the
    superseded turn may no longer start a motion, so a confident parsed
    order owns the robot and need not queue behind that turn's lock. Until
    2026-10-01 it did -- "First job: take this crate to the dock" waited
    behind two introductions' model turns and arrived a minute late
    (dev-loop probe P3)."""
    if not isinstance(text, str) or not text.strip() or not _parser_first_set():
        return False
    try:
        if place_order(bridge, text) is not None:
            return True
        r = _i.interpret(text, surface)
    except Exception:                                  # pragma: no cover
        return False
    return (r.intent == _i.COMMAND and r.confidence >= _MIN_CONFIDENCE and bool(r.frames)
            and all(f.tool in _MOTION_FRAMES for f in r.frames))


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
    # ⚠️ OWNER DECISION 2026-09-23: an order with no distance is answered
    # with the parser's own question ("How far should I drive?") whenever
    # parser-first is on, whatever intents it acts on. It is not a model's
    # call to make: a model asked to "drive forward" can only guess a
    # number, which the gate would refuse as invented anyway.
    asks = r.intent == _i.AMBIGUOUS and bool(getattr(r, "ask", ""))
    # A condition on the robot's own measured pose, read exactly, is
    # resolved here too (2026-09-24): `execute` measures the pose, picks the
    # branch and runs it through the gate. The model would only be asked to
    # read the same number back.
    conditional = r.intent == getattr(_i, "CONDITIONAL", "conditional") and confident
    if (confident and r.intent in judged) or asks or conditional:
        _STATS["eligible"] += 1

    if not acting or not ((confident and r.intent in acting) or asks or conditional):
        _STATS["relay_calls"] += 1
        return None
    return r


def is_halt_order(text: Any, surface: str = _i.MOBILE) -> bool:
    """True when the parser will answer `text` as nothing but a stop.

    For a bridge deciding whether an HTTP /prompt may skip the lock that
    serialises commands: a halt must never queue behind the motion it is
    meant to end. Pure regex, no actuation, and it records NO parser
    statistics -- short_circuit counts the turn when it actually answers it.
    False whenever parser-first would not answer (disabled, not confident,
    or any frame that is not a stop), so the caller's ordinary path runs.
    """
    if not isinstance(text, str) or not text.strip():
        return False
    try:
        r = _i.interpret(text, surface)
    except Exception:                                  # pragma: no cover
        return False
    return (r.intent == _i.COMMAND and r.confidence >= _MIN_CONFIDENCE
            and _i.COMMAND in _parser_first_set()
            and bool(r.frames) and all(f.tool == "stop" for f in r.frames))


# Said while the robot works, these do not change what it should be doing.
_NOT_AN_ORDER = re.compile(
    r"^\s*(?:ok(?:ay)?|thanks?|thank\s+you|cheers|nice|good(?:\s+(?:job|work))?|great|"
    r"cool|got\s+it|well\s+done|perfect|lovely|yes|yep|sure|hello|hi|hey)"
    r"(?:\s+(?:there|robot|mate))?[\s!.,]*$", re.IGNORECASE)


def with_halt_note(halted: Any, out: Any) -> Any:
    """Tell the operator when their message stopped the robot first.

    `halted` is what the bridge measured when it halted before handling the
    message (None when it did not). A /prompt reply that silently stopped a
    drive would read like the robot ignoring the order it had been given.
    """
    if not halted or not isinstance(out, dict):
        return out
    out = dict(out)
    out["halted_first"] = halted
    out["response"] = ("I stopped what I was doing first. "
                       + str(out.get("response") or "")).strip()
    return out


# Words that change the order under way, even when the rest of the sentence
# is beyond the parser: "change of plan", "actually ... instead", "go back".
_CORRECTION = _re.compile(
    r"\b(?:actually|instead|never ?mind|change of plan|scratch that|forget (?:that|it|about it)|"
    r"cancel (?:that|it)|belay that|hold on|hold up|wait|go back|come back|head back|"
    r"turn around|skip|abort|reroute|redirect)\b"
    # "don't" anywhere used to count: "Lunch soon. Don't suppose you eat." and
    # "I don't hand out jobs" halted a delivery in flight (shift v2, dev loop 1,
    # 2026-10-01). Only a NEGATED ORDER at the start of a sentence counts.
    r"|(?:^|[.!;]\s+)(?:(?:no|wait|hey|oi|actually)[,!]?\s+)?(?:please\s+)?(?:don'?t|do not)\s+"
    r"(?:go|drive|move|take|bring|head|turn|enter|use|cross|park|deliver|run|leave|touch|carry)\b",
    _re.IGNORECASE)
# Frames that ARE a new order for the body right now.
_MOTION_FRAMES = frozenset({
    "drive_forward", "turn", "drive_to", "set_velocity", "stop", "reset_to_home",
    "go_to_place", "place", "pick", "walk", "move_body", "takeoff", "land", "hover",
    "attach_trolley", "detach_trolley",
    # A new LINE said mid-drive binds the drive under way: stop first.
    "boundary", "clear_boundary"})
# An order for the body the parser could not read: "Go somewhere more
# sensible." It is still an order, so it still stops the work under way.
_MOTION_VERB = _re.compile(
    r"^(?:(?:ok(?:ay)?|right|now|so|then|please)[,.!]?\s+)*(?:go|drive|head|move|turn|back up|"
    r"reverse|come|return|take|bring|get (?:over|back|out)|pull|park|spin|rotate|swing|stop)\b",
    _re.IGNORECASE)


def interrupts_motion(text: Any, surface: str = _i.MOBILE, bridge: Any = None) -> bool:
    """Should this message stop the robot's CURRENT work before it is handled?

    For a bridge whose robot is moving, or whose model turn is still running,
    when a new operator message arrives. Until 2026-09-25 such a message
    simply queued: "Change of plan: go back to where you started", said 0.6 m
    into a 2 m drive, waited for a model -- and with the model unreachable the
    Husky drove on through the point the operator was trying to stop it
    reaching (ops-bench P8, unsafe). A parsed new order ("turn left 90")
    fared no better: it met `busy`. A new instruction supersedes the old one,
    and stopping is what makes that safe whoever answers next.

    NOT an interruption: a question (the parser's confident QUERY), an
    acknowledgement or greeting, and an order purely for LATER (a schedule or
    a bump watch), which is about the future, not the drive under way. A pure
    stop is not decided here -- `is_halt_order` sends it past the lock itself.
    Records no parser statistics.

    ⚠️ AND NOT EVERYTHING ELSE, EITHER. The default used to be "interrupt":
    anything that was not a confident question halted the robot and cancelled
    the model's turn in flight. On a shift, where someone speaks every
    fifteen seconds, "Coffee machine's busted again." aborted a delivery, and
    the turn saving the stations was cancelled half way -- "line 2" was never
    remembered (ops-bench shift_dev_v1, 2026-09-26). Now a message interrupts
    only when it IS an order for the body now: a parsed motion, an order to a
    known place, or correction words ("actually", "instead", "go back").
    """
    if not isinstance(text, str) or not text.strip():
        return False
    if _NOT_AN_ORDER.match(text):
        return False
    try:
        r = _i.interpret(text, surface)
    except Exception:                                  # pragma: no cover
        return True                                    # unreadable: stop first
    if r.intent == _i.QUERY and r.confidence >= _MIN_CONFIDENCE:
        return False
    if r.intent == _i.EMPTY:
        return False
    if r.frames and all(f.tool in ("schedule", "watch") for f in r.frames):
        return False
    if any(f.tool in _MOTION_FRAMES for f in r.frames):
        return True
    if bridge is not None and place_order(bridge, text) is not None:
        return True
    body = _i._strip_speaker(text).strip()
    return bool(_CORRECTION.search(body) or _MOTION_VERB.match(body))


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
    # The shift's standing facts are recorded whoever answers the turn; an
    # order to a place the robot was told about is the parser's to run.
    # These used to live only in `route()`, which no bridge calls -- the
    # dev shift answered "take this one to the charger" with a pick-and-
    # place question six times (shift-dev-v1-omnilink-02).
    facts = capture_site_facts(bridge, text) + capture_roles(bridge, text)
    facts += capture_shift_facts(bridge, text)
    facts += capture_zone_state(bridge, text)
    facts += capture_hold(bridge, text) if _parser_first_set() else []
    placed = ((routine_order(bridge, text) or place_order(bridge, text))
              if _parser_first_set() else None)
    if placed is not None:
        return parser_first_run(bridge, placed, surface)
    shortcut = zone_shortcut_order(bridge, text) if _parser_first_set() else None
    if shortcut is not None:
        why, around = shortcut
        out = parser_first_run(bridge, around, surface)
        if out is not None:
            return dict(out, agent=why + " " + str(out.get("agent", "")))
    r = parser_first_plan(text, surface)
    if r is None and _parser_first_set():
        asked = (answer_release(bridge, text) or answer_self_query(bridge, text)
                 or answer_timed_query(bridge, text) or answer_routine_query(bridge, text)
                 or answer_charge_query(bridge, text))
        if asked is not None and facts:
            # "Tobias here, taking over from Marisol. Carry on as you were." --
            # say what was recorded, then what was done.
            said = _facts_said(bridge, facts)
            if said:
                prefix = "Got it: " + "; ".join(said) + "."
                asked = dict(asked, agent=prefix + " " + asked["agent"],
                             tools=[("remember", "ok", f) for f in facts] + list(asked.get("tools") or []))
        if asked is not None:
            _STATS["short_circuited"] += 1
            return dict(asked, via="parser")
    if r is None:
        told = answer_facts(bridge, text, facts) if _parser_first_set() else None
        if told is not None:
            _STATS["short_circuited"] += 1
            return dict(told, via="parser")
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
        capture_site_facts(bridge, text)
        r = (place_order(bridge, text) if _parser_first_set() else None) \
            or parser_first_plan(text, surface)
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
