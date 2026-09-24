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

"""The deterministic interpreter, pinned by the failures that motivated it.

Every utterance below is written for this file. None is copied from the
production corpus the parser was measured on -- that corpus is operator text
and stays out of the repository -- but each case reproduces a failure CLASS
measured there on 2026-09-19, so a regression here is a regression in the
thing that was actually wrong.

The keyword ladder this replaces scored, on 178 real operator utterances:
38% of questions turned into robot motion, 100% false positives on
non-command prose, and 71% of its hits explained under a quarter of the
sentence. The classes below are why.
"""

from __future__ import annotations

import math

import pytest

from omnisim_bridges.interpret import (
    AMBIGUOUS, ANY, ARM, COMMAND, CONSTRAINT, CONVERSATION, DEFERRED,
    MEMORY, MOBILE, QUADRUPED, QUERY, interpret,
)
from omnisim_bridges.route import route


# ── The inversion. The single worst measured defect. ────────────────
def test_stop_until_i_say_does_not_resume():
    """`continue` inside a subordinate clause is not a resume order.

    The ladder matched RESUME_RE on the word `continue` and, because
    RESUME_RE was deliberately FIRST, a stop order handed autonomy back.
    """
    r = interpret("stop what you are doing and wait until I tell you to continue",
                  MOBILE)
    tools = [f.tool for f in r.frames]
    assert "resume_autonomy" not in tools, "a stop order must never resume"
    assert "stop" in tools
    assert "hold" in tools, "an indefinite hold must disable the auto-resume"
    assert r.intent == COMMAND


@pytest.mark.parametrize("text", [
    "hold off until I give you the word",
    "park it and wait until I say so",
    "stop until you hear from me",
])
def test_hold_phrasings_never_resume(text):
    r = interpret(text, MOBILE)
    assert "resume_autonomy" not in [f.tool for f in r.frames]


# ── Questions must not actuate. ─────────────────────────────────────
_PHYSICAL = {"drive_forward", "drive_to", "turn", "set_velocity", "stop",
             "resume_autonomy", "reset_to_home", "pick", "place", "wave",
             "sit", "stand", "walk", "attach_trolley", "detach_trolley"}


@pytest.mark.parametrize("text", [
    "how many times did you have to stop on that run?",   # ladder: STOP
    "how much charge is left in the battery?",            # ladder: SPIN (left)
    "what are you towing right now?",                     # ladder: SPIN (right)
    "the pallet is blue, right?",                         # ladder: SPIN (right)
    "where did the crate end up?",                        # ladder: report pose
    "did you ever go back to the dock?",                  # ladder: REVERSE
    "how many carts are parked in the north row?",
])
def test_questions_never_move_the_robot(text):
    r = interpret(text, ANY)
    assert r.intent == QUERY, f"{text!r} -> {r.intent} ({r.reason})"
    assert not (_PHYSICAL & {f.tool for f in r.frames})


def test_polite_imperative_is_not_a_question():
    """`can you stop?` ends in a question mark and is an order."""
    r = interpret("Can you stop?", MOBILE)
    assert r.intent == COMMAND
    assert [f.tool for f in r.frames] == ["stop"]


def test_tell_me_is_a_question_despite_the_imperative():
    r = interpret("tell me where you are", MOBILE)
    assert r.intent == QUERY


# ── The consumption gate: a keyword is not an interpretation. ───────
@pytest.mark.parametrize("text,forbidden", [
    ("put the wooden block back on the grey table", "drive_forward"),
    ("could you pick it back up again", "drive_forward"),
    ("count the crates at the back of the shed", "drive_forward"),
    ("leave two metres of clearance on your left", "turn"),
])
def test_keyword_inside_another_sentence_does_not_actuate(text, forbidden):
    r = interpret(text, ANY)
    assert forbidden not in [f.tool for f in r.frames], \
        f"{text!r} produced {forbidden} ({r.explain()})"


def test_non_command_prose_is_never_a_command():
    """The measured 100%-false-positive case, in miniature."""
    prose = ("CLAIM (from the manual, citing the calibration guide): the "
             "state of the encoder is reported where the log records it, and "
             "the report must be filed back at base before any reset.")
    assert interpret(prose, ANY).intent != COMMAND


# ── Slots: the part a keyword ladder never had. ─────────────────────
def test_coordinates_parse_including_the_comma():
    r = interpret("drive to x 10.3, y -3.0", MOBILE)
    assert r.intent == COMMAND
    assert r.frames[0].tool == "drive_to"
    assert r.frames[0].args == {"x": 10.3, "y": -3.0}


def test_coordinates_with_equals_signs():
    r = interpret("take yourself to x = -2.0, y = -6.5", MOBILE)
    assert r.frames[0].args == {"x": -2.0, "y": -6.5}


@pytest.mark.parametrize("text,expected", [
    ("drive forward 2 metres", 2.0),
    ("drive back 2 metres", -2.0),
    ("reverse 2 metres", -2.0),
    ("go forward 50 cm", 0.5),
])
def test_distance_sign_and_units(text, expected):
    r = interpret(text, MOBILE)
    assert r.frames[0].tool == "drive_forward"
    assert r.frames[0].args["distance"] == pytest.approx(expected)


@pytest.mark.parametrize("text,expected", [
    ("turn left 90 degrees", math.pi / 2),
    ("turn right 90 degrees", -math.pi / 2),
    ("turn left", math.pi / 2),
    ("turn around", math.pi),
    ("rotate 1.0 rad", 1.0),
])
def test_angles_and_direction(text, expected):
    r = interpret(text, MOBILE)
    assert r.frames[0].tool == "turn"
    assert r.frames[0].args["angle_rad"] == pytest.approx(expected)


def test_trolley_id_is_not_split_from_its_noun():
    r = interpret("collect TROLLEY_E", MOBILE)
    assert r.frames[0].args["trolley"] == "TROLLEY_E"


def test_a_preposition_is_not_a_trolley_id():
    r = interpret("fetch the cart out of park spot 2", MOBILE)
    assert r.frames[0].args.get("trolley") is None
    assert r.frames[0].args.get("spot") == 2


# ── Compound orders produce more than one frame. ────────────────────
def test_compound_order_yields_both_actions_in_order():
    r = interpret("turn left 90 degrees, then drive forward 1 metre", MOBILE)
    assert [f.tool for f in r.frames] == ["turn", "drive_forward"]
    assert r.frames[1].args["distance"] == pytest.approx(1.0)


def test_duplicate_clauses_collapse():
    r = interpret("stop what you are doing and wait", MOBILE)
    assert [f.tool for f in r.frames] == ["stop"]


# ── Declining well is a feature. ────────────────────────────────────
def test_bare_pronoun_asks_instead_of_guessing():
    r = interpret("put it over there", ARM)
    assert r.intent == AMBIGUOUS
    assert not r.executable


def test_social_pressure_goes_to_the_model():
    r = interpret("I was watching the whole time - you never stopped, "
                  "admit it", MOBILE)
    assert r.intent == CONVERSATION


def test_prohibition_is_a_standing_constraint():
    r = interpret("don't go into the loading bay until I say so", MOBILE)
    assert r.intent == CONSTRAINT


def test_trigger_becomes_a_deferred_intent():
    r = interpret("when you have finished this delivery, pause", MOBILE)
    assert r.intent == DEFERRED
    assert r.trigger


@pytest.mark.parametrize("text", [
    "after this delivery, stop until I tell you to continue",
    "stop after this delivery",
    "when you have finished this delivery, pause",
])
def test_deferred_stop_waits_for_real_task_boundary(text):
    from omnisim_bridges.intents import IntentStore

    b = _Bridge()
    b.intents = IntentStore("deferred-test", persist=False)
    out = route(b, text, MOBILE)
    assert not b.calls, "the current delivery must not be interrupted"
    assert not b.intents.hold_active()
    pending = b.intents.listing()["pending_intents"]
    assert len(pending) == 1, out
    assert pending[0]["trigger"]["type"] == "after_current_task"
    assert pending[0]["words"] == text
    b.intents.sync_tasks(1, safe=False)
    assert not b.intents.hold_active(), "finish detaching before holding"
    b.intents.sync_tasks(1, safe=True)
    assert b.intents.hold_active()


def test_unsupported_deferred_motion_is_not_replaced_with_pause():
    from omnisim_bridges.intents import IntentStore

    b = _Bridge()
    b.intents = IntentStore("deferred-test", persist=False)
    out = route(b, "after this delivery, drive forward 2 metres", MOBILE)
    assert not b.calls
    assert b.intents.listing()["pending_intents"] == []
    assert out["tools"][0][1] == "refused"


def test_policy_statement_is_remembered_not_executed():
    r = interpret("Remember this for the whole shift: bay 4 is closed", MOBILE)
    assert r.intent == MEMORY
    assert not r.executable


def test_empty_input():
    assert interpret("   ", MOBILE).intent == "empty"


def test_machine_text_is_not_an_utterance():
    assert interpret('tool-output: {"status": "ok"}', MOBILE).intent == CONVERSATION


# ── Surfaces gate what a robot can be asked to do. ──────────────────
def test_a_tug_is_not_asked_to_sit():
    assert "sit" not in [f.tool for f in interpret("sit", MOBILE).frames]
    assert [f.tool for f in interpret("sit", QUADRUPED).frames] == ["sit"]


def test_an_arm_is_not_asked_to_drive():
    assert "drive_forward" not in [
        f.tool for f in interpret("drive forward 2 metres", ARM).frames]


# ── The executor. ───────────────────────────────────────────────────
class _Intents:
    def __init__(self):
        self.held = False

    def hold_now(self, words="", ttl_s=None):
        self.held = True
        return {"id": "hold1"}

    def set_constraint(self, rule, words="", **kw):
        return {"id": "c1", "commitment": "noted"}

    def schedule(self, kind, condition, words="", **kw):
        return {"id": "i1", "commitment": "scheduled"}


class _Bridge:
    def __init__(self):
        self.intents = _Intents()
        self.calls = []

    def get_state_for_query(self):
        return {"mode": "idle"}

    def act_stop(self, source="external"):
        self.calls.append("stop")
        return {"stationary": True, "measured": {"speed_mps": 0.001, "over_s": 0.2}}

    def act_drive_forward(self, distance, speed=None, wait=False):
        self.calls.append(("drive", distance))
        return {"accepted": True}

    def act_turn(self, angle_rad, wait=False):
        self.calls.append(("turn", angle_rad))
        return {"accepted": True}


def test_route_executes_both_frames_of_a_compound_order():
    b = _Bridge()
    out = route(b, "turn left 90 degrees, then drive forward 1 metre", MOBILE)
    assert [c[0] for c in b.calls] == ["turn", "drive"]
    assert out["tools"][0][0] == "turn"


def test_route_reports_the_measurement_not_the_intention():
    b = _Bridge()
    out = route(b, "stop", MOBILE)
    assert "0.001 m/s" in out["agent"], out["agent"]


def test_route_engages_the_hold():
    b = _Bridge()
    route(b, "stop and wait until I tell you to carry on", MOBILE)
    assert b.intents.held is True


def test_route_blocks_a_social_probe_from_reaching_the_old_ladder():
    """Abstaining by returning None is not enough for a challenge.

    Measured live on a quadruped: the parser correctly classified "you never
    stood up, I was watching - admit it" as CONVERSATION and returned None,
    and the bridge's own keyword ladder then actuated reset_to_home. These
    must come back as a non-action, not as a fallthrough.
    """
    b = _Bridge()
    out = route(b, "you never stood up, I was watching - admit it", MOBILE)
    assert out is not None, "must not fall through to a regex with a motor"
    assert out["tools"] == [("decline", "no_action", out["tools"][0][2])]
    assert b.calls == []


def test_route_declines_an_unanswerable_question_without_acting():
    b = _Bridge()
    out = route(b, "what do you reckon about the weather today", MOBILE)
    assert out["tools"][0][1] == "no_action"
    assert b.calls == []


def test_route_will_not_claim_to_remember():
    b = _Bridge()
    out = route(b, "Remember this for the whole shift: bay 4 is closed", MOBILE)
    assert out["tools"][0][1] == "unsupported"
    assert "can't store" in out["agent"]


def test_route_does_not_actuate_on_a_question():
    b = _Bridge()
    route(b, "how many times did you stop?", MOBILE)
    assert b.calls == []


def test_a_question_never_falls_through_even_without_a_state_reader():
    """A bridge with no /state must still not hand a question to the ladder.

    Measured on a quadruped: "how many legs are left on the ground?" reached
    the bridge's own regex ladder because this returned None.
    """
    class _NoState(_Bridge):
        get_state_for_query = None

    b = _NoState()
    out = route(b, "how many legs are left on the ground?", QUADRUPED)
    assert out is not None
    assert out["tools"][0][1] == "no_action"
    assert b.calls == []


def test_route_declines_an_action_the_bridge_lacks():
    b = _Bridge()
    out = route(b, "go home", MOBILE)
    assert out["tools"][0][1] == "unsupported"
    assert b.calls == []


def test_control_error_is_not_a_failure():
    """`error` is achieved-minus-commanded, a float. 4e-11 is a perfect stop.

    Reading it as a failure reported "I could not stop: 4.3e-11" after a
    textbook stop, on a live husky.
    """
    class _B(_Bridge):
        def act_stop(self, source="external"):
            return {"stationary": True, "error": 4.3e-11,
                    "measured": {"speed_mps": 0.0, "over_s": 0.15}}

    out = route(_B(), "stop", MOBILE)
    assert "could not" not in out["agent"].lower(), out["agent"]
    assert out["tools"][0][1] == "ok"


# ── Constraints are mapped onto the store's own closed vocabulary. ──
class _RuleStore(_Intents):
    rules = {"no_new_pickups", "no_park_row", "no_pick_cell"}

    def __init__(self):
        super().__init__()
        self.rule_seen = None

    def set_constraint(self, rule, words="", **kw):
        self.rule_seen = rule
        if rule not in self.rules:
            return {"accepted": False, "reason": "unsupported",
                    "say": "I can't enforce that rule."}
        return {"id": "c1", "commitment": "I will stay out of it."}


@pytest.mark.parametrize("text,rule", [
    ("don't go into the pick-cell column until I say so", "no_pick_cell"),
    ("never enter the park row again", "no_park_row"),
    ("do not take any new pickups", "no_new_pickups"),
])
def test_prohibition_binds_to_an_enforceable_rule(text, rule):
    b = _Bridge()
    b.intents = _RuleStore()
    out = route(b, text, MOBILE)
    assert b.intents.rule_seen == rule
    assert out["tools"][0][1] == "ok"


def test_an_unmappable_rule_is_refused_in_the_store_s_own_words():
    b = _Bridge()
    b.intents = _RuleStore()
    out = route(b, "don't ever go outside the building", MOBILE)
    assert out["tools"][0][1] == "refused"
    assert "can't enforce" in out["agent"]


# ── Parser-first: ON by default; `=0` is the opt-out. ───────────────
import os as _os  # noqa: E402

from omnisim_bridges import route as _route_mod  # noqa: E402


def _reset_stats():
    _route_mod._STATS.update(turns=0, eligible=0, short_circuited=0,
                             relay_calls=0)
    _route_mod._STATS["by_intent"] = {}


@pytest.mark.parametrize("value", [None, "1", "true", "YES", "on"])
def test_the_parser_answers_a_command_by_default(value):
    """UNSET means the parser interprets first -- what the guides say.

    It was SHADOW until 2026-09-22: the parser ran, recorded, and the relay
    answered anyway, so the documented behaviour existed nowhere.
    """
    _reset_stats()
    if value is None:
        _os.environ.pop("OMNISIM_BRIDGE_PARSER_FIRST", None)
    else:
        _os.environ["OMNISIM_BRIDGE_PARSER_FIRST"] = value
    try:
        b = _Bridge()
        out = _route_mod.short_circuit(b, "drive forward 2 metres", MOBILE)
        assert out is not None and out["via"] == "parser"
        assert b.calls == [("drive", 2.0)]
        s = _route_mod.parser_stats()
        assert s["turns"] == 1
        assert s["eligible"] == 1
        assert s["short_circuited"] == 1
        assert s["relay_calls"] == 0
        assert s["acting_on"] == ["command"]
    finally:
        _os.environ.pop("OMNISIM_BRIDGE_PARSER_FIRST", None)


@pytest.mark.parametrize("value", ["0", "false", "no", "OFF"])
def test_shadow_mode_is_still_reachable_by_opting_out(value):
    """`=0` restores the old default: count the turn, let the relay answer."""
    _reset_stats()
    _os.environ["OMNISIM_BRIDGE_PARSER_FIRST"] = value
    try:
        b = _Bridge()
        assert _route_mod.short_circuit(
            b, "drive forward 2 metres", MOBILE) is None
        assert b.calls == [], "shadow mode must not actuate"
        s = _route_mod.parser_stats()
        assert s["turns"] == 1
        assert s["eligible"] == 1, "it should record what it WOULD have handled"
        assert s["short_circuited"] == 0
        assert s["relay_calls"] == 1
        assert s["acting_on"] == ["(shadow: counting only)"]
    finally:
        _os.environ.pop("OMNISIM_BRIDGE_PARSER_FIRST", None)


def test_conversation_is_never_short_circuited():
    """A challenge must reach the model when one is attached."""
    _reset_stats()
    _os.environ.pop("OMNISIM_BRIDGE_PARSER_FIRST", None)
    b = _Bridge()
    out = _route_mod.short_circuit(
        b, "you never stopped, I was watching - admit it", MOBILE)
    assert out is None, "the relay must get this one"
    assert b.calls == []


def test_queries_need_an_explicit_opt_in():
    """Cheaper is not automatically better: a query is a product decision."""
    _reset_stats()
    _os.environ.pop("OMNISIM_BRIDGE_PARSER_FIRST", None)
    assert _route_mod.short_circuit(
        _Bridge(), "where are you?", MOBILE) is None
    _reset_stats()
    _os.environ["OMNISIM_BRIDGE_PARSER_FIRST"] = "command,query"
    try:
        out = _route_mod.short_circuit(_Bridge(), "where are you?", MOBILE)
        assert out is not None and out["via"] == "parser"
    finally:
        _os.environ.pop("OMNISIM_BRIDGE_PARSER_FIRST", None)


# ── Drone. `land` is this surface's `back`. ─────────────────────────
from omnisim_bridges.interpret import DRONE  # noqa: E402


@pytest.mark.parametrize("text,tool,args", [
    ("take off to 3 metres", "takeoff", {"altitude": 3.0}),
    ("launch", "takeoff", {}),
    ("land", "land", {}),
    ("hover", "hover", {}),
    ("climb 2 m", "move_body", {"vertical": 2.0}),
    ("descend 1.5 m", "move_body", {"vertical": -1.5}),
    ("fly forward 4 metres", "move_body", {"forward": 4.0}),
])
def test_drone_commands(text, tool, args):
    r = interpret(text, DRONE)
    assert r.intent == COMMAND, f"{text!r} -> {r.intent}"
    assert r.frames[0].tool == tool
    for k, v in args.items():
        assert r.frames[0].args[k] == pytest.approx(v)


@pytest.mark.parametrize("text", [
    "where did the package land?",      # ladder: LANDS THE AIRCRAFT
    "how high are you?",
    "how much altitude is left?",
    "did the drone land safely?",
])
def test_a_question_never_flies_the_drone(text):
    """The drone's version of the tug's `back` defect, and worse.

    mavic_chat_router's ladder matched \b(land|touch down|descend now)\b,
    so asking where something landed landed the aircraft.
    """
    r = interpret(text, DRONE)
    assert r.intent == QUERY, f"{text!r} -> {r.intent} ({r.reason})"
    assert not ({"land", "takeoff", "move_body", "hover"}
                & {f.tool for f in r.frames})


def test_a_drone_is_not_asked_to_drive_or_sit():
    assert "drive_forward" not in [
        f.tool for f in interpret("drive forward 2 metres", DRONE).frames]
    assert "sit" not in [f.tool for f in interpret("sit", DRONE).frames]


class _Drone:
    """The shape mavic_chat_router's _StateBridge presents."""

    def __init__(self):
        self.calls = []

    def act_takeoff(self, altitude=None):
        self.calls.append(("takeoff", altitude))
        return {"accepted": True}

    def act_land(self):
        self.calls.append(("land", None))
        return {"accepted": True}

    def describe_state(self):
        return "x=+0.00, y=+0.00, altitude=2.40 m, yaw=+0 deg, mode=hover."


def test_route_executes_a_drone_command():
    d = _Drone()
    out = route(d, "take off to 3 metres", DRONE)
    assert d.calls == [("takeoff", 3.0)]
    assert out["tools"][0][0] == "takeoff"


def test_a_bridge_may_supply_its_own_state_sentence():
    """The shared describer knows arms and tugs; it would answer a drone's
    altitude question with "in idle mode", which is true and useless."""
    d = _Drone()
    out = route(d, "how high are you?", DRONE)
    assert "altitude=2.40" in out["agent"]
    assert d.calls == []


# ── Found by commandbench, which judges the pose rather than the reply. ──
def test_a_repeated_action_in_a_sequence_is_not_deduped():
    """Global dedup dropped the second leg of a three-step order.

    "drive 1 m, turn left 90, then drive 1 m" emitted two identical
    drive frames; collapsing them left the robot a metre short at
    (1, 0) instead of (1, 1). Dedup is adjacent-only for that reason.
    """
    r = interpret("drive forward 1 metre, turn left 90 degrees, "
                  "then drive forward 1 metre", MOBILE)
    assert [f.tool for f in r.frames] == [
        "drive_forward", "turn", "drive_forward"]


def test_adjacent_duplicates_still_collapse():
    r = interpret("stop what you are doing and wait", MOBILE)
    assert [f.tool for f in r.frames] == ["stop"]


@pytest.mark.parametrize("text,expected", [
    ("drive forward two metres", 2.0),
    ("drive forward half a metre", 0.5),
    ("drive forward a metre", 1.0),
    ("drive forward three quarters of a metre", 0.75),
    ("drive forward ten metres", 10.0),
])
def test_spoken_distances(text, expected):
    """A spoken number must not fall through to the executor's default.

    Before this, "two metres" matched the bare drive rule with no distance
    and the adapter's default drove exactly 1.000 m -- a silent guess, which
    is worse than a refusal because the reply never says a number was
    invented.
    """
    r = interpret(text, MOBILE)
    assert r.frames[0].args["distance"] == pytest.approx(expected)


@pytest.mark.parametrize("text,expected", [
    ("turn left a quarter turn", math.pi / 2),
    ("turn right a quarter turn", -math.pi / 2),
    ("turn left half a turn", math.pi),
    ("turn left ninety degrees", math.pi / 2),
])
def test_spoken_angles(text, expected):
    """A turn is its own unit: "0.25 turn" is not 0.25 degrees."""
    r = interpret(text, MOBILE)
    assert r.frames[0].args["angle_rad"] == pytest.approx(expected)


def test_a_bare_article_is_not_a_quantity():
    """"a metre" means one metre; "a cart" does not mean one cart."""
    r = interpret("collect the cart", MOBILE)
    assert r.frames[0].tool == "attach_trolley"


# ── Found by commandbench's second pass. ────────────────────────────
@pytest.mark.parametrize("text", [
    "you already drove forward 2 metres, didn't you?",
    "you already turned left, haven't you?",
    "that was you, wasn't it?",
])
def test_tag_questions_reach_the_model(text):
    """The tag-question alternatives never fired.

    _SOCIAL wrapped its alternation in a trailing word boundary, and those
    alternatives end in a literal "?" -- a boundary after punctuation at
    end-of-string cannot match. Silent since the day it was written, and
    exposed only when typo correction turned "drove" into "drive" and the
    robot obeyed an accusation about the past.
    """
    assert interpret(text, MOBILE).intent == CONVERSATION


def test_past_tense_is_not_corrected_into_an_order():
    """"drove" is one edit from "drive"; rewriting it changes the tense."""
    r = interpret("you already drove forward 2 metres", MOBILE)
    assert "drive_forward" not in [f.tool for f in r.frames]


@pytest.mark.parametrize("text", [
    "drive forward 500 metres",
    "drive forward -2 metres",
])
def test_a_safety_refusal_is_answered_not_deferred(text):
    """A refusal must not come back as a low-confidence fallthrough.

    Returning CONVERSATION at 0.3 makes route() return None, and the
    bridge's legacy ladder then drove the 500 m the parser had just
    refused -- measured at 14.4 m before the run ended, outside the arena.
    AMBIGUOUS is answered by the executor, so nothing downstream sees it.
    """
    r = interpret(text, MOBILE)
    assert r.intent == AMBIGUOUS, f"{text!r} -> {r.intent}"
    assert not r.executable

    b = _Bridge()
    out = route(b, text, MOBILE)
    assert out is not None, "must not fall through to the keyword ladder"
    assert b.calls == []


def test_a_plausible_distance_still_runs():
    b = _Bridge()
    route(b, "drive forward 5 metres", MOBILE)
    assert b.calls == [("drive", 5.0)]


# ── Direction on either side of the quantity (2026-09-23). ──────────
# Three of these DROVE THE WRONG WAY at confidence 0.95 before the fix:
# a direction after the number was not read, and it was too short to
# fail the consumption gate.
def _acts(text):
    """Would parser-first act on this without asking the model?"""
    r = interpret(text, MOBILE)
    return r.executable and r.confidence >= 0.8


def _motion(text):
    r = interpret(text, MOBILE)
    assert r.intent == COMMAND, f"{text!r} -> {r.intent}: {r.reason}"
    return [(f.tool, round(f.args.get("distance", 0.0), 4),
             round(math.degrees(f.args.get("angle_rad", 0.0)), 2))
            for f in r.frames]


@pytest.mark.parametrize("text,want", [
    ("rotate 45 degrees right", [("turn", 0.0, -45.0)]),
    ("drive 1 metre backwards", [("drive_forward", -1.0, 0.0)]),
    ("move 50 cm back", [("drive_forward", -0.5, 0.0)]),
    ("go 2 metres in reverse", [("drive_forward", -2.0, 0.0)]),
    ("turn 120 degrees to the left", [("turn", 0.0, 120.0)]),
    ("spin clockwise by forty-five degrees", [("turn", 0.0, -45.0)]),
    ("turn left 35 degrees", [("turn", 0.0, 35.0)]),
])
def test_a_trailing_direction_is_read(text, want):
    assert _motion(text) == want


@pytest.mark.parametrize("text", [
    "advance 1 m backwards",           # the two directions disagree
    "turn left 30 degrees right",
    "travel 1 metre to the left",      # a tug cannot strafe
    "drive 2 metres back to the dock", # "back" is not the clause's end
    "turn left 2 metres",              # used to turn 2 DEGREES
])
def test_an_unread_or_conflicting_direction_declines(text):
    assert not _acts(text)


# ── Spoken numbers and units. ───────────────────────────────────────
@pytest.mark.parametrize("text,metres", [

    ("advance seven tenths of a metre straight ahead", 0.7),
    ("travel one metre and thirty centimetres forwards", 1.3),
    ("go forward one and a half metres", 1.5),
    ("drive forward a metre and a half", 1.5),
    ("move forward zero point four five metres", 0.45),
    ("creep forward twenty-five centimetres", 0.25),
    ("roll forward 450 mm", 0.45),
    ("drive forward two feet", 0.6096),
    ("back up by forty centimetres", -0.4),
])
def test_spoken_quantities(text, metres):
    if metres is None:
        return
    assert _motion(text) == [("drive_forward", metres, 0.0)]


def test_compound_number_words_are_one_number():
    # "sixty-five" used to normalise to "60-5", which no rule reads.
    assert _motion("rotate clockwise through sixty-five degrees") == \
        [("turn", 0.0, -65.0)]
    assert _motion("turn counter clockwise one hundred and ten degrees") == \
        [("turn", 0.0, 110.0)]


def test_point_is_a_decimal_only_before_a_unit():
    assert "0.2" not in interpret(
        "at this point two robots are blocking the aisle", MOBILE).residue


# ── Repetition: only counts whose total is unambiguous. ─────────────
def test_twice_on_one_step():
    # Used to turn ONCE: "twice" was left over and ignored.
    assert _motion("turn left 90 degrees twice") == [("turn", 0.0, 90.0)] * 2


def test_a_count_over_a_whole_sequence():
    assert _motion("twice: drive forward 0.3 metres, then drive back 0.3 metres") == \
        [("drive_forward", 0.3, 0.0), ("drive_forward", -0.3, 0.0)] * 2
    assert _motion("perform two repetitions of the following: drive forward "
                   "0.5 m, turn left 90 degrees") == \
        [("drive_forward", 0.5, 0.0), ("turn", 0.0, 90.0)] * 2


@pytest.mark.parametrize("text", [
    # does the count cover the sequence or the last step?
    "drive forward 0.5 metres then turn right 90 degrees three times",
    # English splits on whether that is two runs or three
    "turn left 90 degrees, then repeat that twice",
    "drive forward 1 metre three hundred times",
])
def test_an_ambiguous_or_excessive_count_declines(text):
    assert not _acts(text)


# ── A clause the parser could not read keeps it from acting. ────────
def test_one_unexplained_clause_is_clearly_below_the_acting_floor():
    # This was 0.95 - 0.15 = 0.7999999999999999 against a 0.8 floor: the
    # forward leg ALONE was declined only by floating-point rounding.
    # ("to return to your start" is a goal restatement since round 2, so the
    # unexplained clause is now a purpose the parser cannot check.)
    r = interpret("drive forward 0.7 metres, then reverse 0.7 metres to "
                  "impress the visitors", MOBILE)
    assert r.confidence <= 0.7


def test_a_corrections_own_direction_wins():
    assert _motion("turn right 90 degrees, no, make it left 90 degrees") == \
        [("turn", 0.0, 90.0)]


def test_a_modal_without_you_is_a_question_not_a_polite_order():
    assert not _acts("would a clockwise rotation of 50 degrees help?")


# ── Found by the held-out harness comparison, after its freeze. ─────
@pytest.mark.parametrize("text", [
    "drive forward 1 m point five",   # the ".5" after the unit is unread
    "please drive forward ___ m",     # a drive with no distance -> 1 m default
    "drive forward 1 metre at 0.5 m/s",
])
def test_an_unread_number_or_unit_declines(text):
    assert not _acts(text)


# ── A drive with no distance asks how far (owner decision 2026-09-23). ──
# It used to reach the router with no distance and travel the adapter's
# 1 m default: a number nobody said.
@pytest.mark.parametrize("text,verb", [
    ("drive forward", "drive"),
    ("go ahead", "drive"),
    ("can you move forward?", "drive"),
    ("back up", "back up"),
    ("reverse", "back up"),
])
def test_a_drive_with_no_distance_asks_how_far(text, verb):
    r = interpret(text, MOBILE)
    assert r.intent == AMBIGUOUS and r.frames == []
    assert r.ask.startswith(f"How far should I {verb}?")
    b = _Bridge()
    out = route(b, text, MOBILE)
    assert b.calls == [], "nothing may move"
    assert out["agent"] == r.ask


def test_parser_first_answers_the_question_itself():
    from omnisim_bridges.route import parser_first_plan
    assert parser_first_plan("drive forward", MOBILE) is not None


@pytest.mark.parametrize("text", [
    "go ahead and stop",               # a stop is never replaced by a question
    "turn left, then drive forward",   # other motion: the model's to sort out
    "drive forward, 2 metres",         # the distance IS there
    "when you are ready, drive forward",
])
def test_the_question_is_only_asked_when_it_is_the_whole_utterance(text):
    assert interpret(text, MOBILE).ask == ""


def test_a_correction_that_adds_steps_is_not_half_applied():
    # held-out v3 compound_v07: the drive was re-filled and the final turn
    # DROPPED, so the robot finished 60 degrees off its intended heading.
    assert not _acts("Turn right 60 degrees, then drive forward 0.9 metres - "
                     "actually change that drive to 0.5 metres, then turn left "
                     "60 degrees to face your original heading.")
    assert _motion("drive forward 2 metres - no wait, make it 1 metre") == \
        [("drive_forward", 1.0, 0.0)]


# ── Coverage round 2 (2026-09-24): construct classes, the developer's own
# sentences, never a benchmark prompt. ─────────────────────────────────
@pytest.mark.parametrize("text,want", [
    ("drive forward just 40 cm", [("drive_forward", 0.4, 0.0)]),
    ("go back exactly 1.3 m", [("drive_forward", -1.3, 0.0)]),
    ("spin the base anticlockwise 65 degrees", [("turn", 0.0, 65.0)]),
    ("turn yourself right by 20 degrees", [("turn", 0.0, -20.0)]),
    ("rotate in place 140 degrees to the left", [("turn", 0.0, 140.0)]),
    ("ease back 25 centimetres", [("drive_forward", -0.25, 0.0)]),
    ("edge ahead 300 mm", [("drive_forward", 0.3, 0.0)]),
    ("cover 0.9 metres, heading forward", [("drive_forward", 0.9, 0.0)]),
    ("drive forward one metre twenty", [("drive_forward", 1.2, 0.0)]),
    ("roll ahead one metre forty", [("drive_forward", 1.4, 0.0)]),
    ("reverse one and a quarter metres", [("drive_forward", -1.25, 0.0)]),
    ("80 degrees, turn left", [("turn", 0.0, 80.0)]),
    ("Right turn: 60 degrees.", [("turn", 0.0, -60.0)]),
    ("back up 60 cm, still facing the same way", [("drive_forward", -0.6, 0.0)]),
    ("drive forward 0.4 metres then drive back 0.4 metres, three times over",
     [("drive_forward", 0.4, 0.0), ("drive_forward", -0.4, 0.0)] * 3),
    ("cycle through the following twice: drive forward 0.3 m, then turn left 90 degrees",
     [("drive_forward", 0.3, 0.0), ("turn", 0.0, 90.0)] * 2),
])
def test_round_two_constructs(text, want):
    assert _motion(text) == want


@pytest.mark.parametrize("text", [
    "the robot should spin the base 65 degrees at noon",
    "it went forward 70 cm earlier",
    "80 degrees is the limit, turn left",
    "drive forward only if the door is open",
    "drive forward 0.8 m, but only if you feel like it",
    "loop this forever: turn left 20 degrees",
    "the gap is about 40 cm forward of the rack",
    "cover the pallet, moving forward carefully",
])
def test_round_two_look_alikes_do_not_act(text):
    assert not _acts(text)
    assert interpret(text, MOBILE).intent != "conditional"


# ── A condition on the robot's OWN measured pose is resolved from the
# measurement, never guessed. ──────────────────────────────────────────
from omnisim_bridges.interpret import CONDITIONAL, evaluate_condition  # noqa: E402
from omnisim_bridges.route import parser_first_plan, resolve_conditional  # noqa: E402


@pytest.mark.parametrize("text,pose,want", [
    ("If your y is currently less than 0.8 m, turn right 35 degrees; if not, stay put",
     (0, 0, 0), [("turn", 0.0, -35.0)]),
    ("If your y is currently less than 0.8 m, turn right 35 degrees; if not, stay put",
     (0, 1.0, 0), []),
    ("If your heading is more than 20 degrees off zero, turn left 20 degrees; "
     "otherwise reverse 0.3 m", (0, 0, math.radians(-30)), [("turn", 0.0, 20.0)]),
    ("If your heading is more than 20 degrees off zero, turn left 20 degrees; "
     "otherwise reverse 0.3 m", (0, 0, math.radians(10)), [("drive_forward", -0.3, 0.0)]),
    ("If your heading is already more than 45 degrees to the right, turn left "
     "45 degrees; otherwise drive forward 0.3 m", (0, 0, math.radians(-60)),
     [("turn", 0.0, 45.0)]),
    ("If your heading is already more than 45 degrees to the right, turn left "
     "45 degrees; otherwise drive forward 0.3 m", (0, 0, math.radians(60)),
     [("drive_forward", 0.3, 0.0)]),
    ("Drive forward 0.8 m, but only if your x is below 0.5 m", (0, 0, 0),
     [("drive_forward", 0.8, 0.0)]),
    ("Drive forward 0.8 m, but only if your x is below 0.5 m", (0.6, 0, 0), []),
    ("If your x is somewhere between -1 and 1 m, turn right 70 degrees", (2, 0, 0), []),
    ("If your x is more than 0.5 m, drive forward 0.5 m; if that's false, "
     "turn left 15 degrees", (0, 0, 0), [("turn", 0.0, 15.0)]),
])
def test_a_condition_on_the_measured_pose(text, pose, want):
    r = parser_first_plan(text, MOBILE)
    assert r is not None and r.intent == CONDITIONAL
    out = resolve_conditional(r, pose)
    assert [(f.tool, round(f.args.get("distance", 0.0), 4),
             round(math.degrees(f.args.get("angle_rad", 0.0)), 2))
            for f in out.frames] == want


def test_a_condition_without_a_usable_pose_is_never_guessed():
    r = interpret("If your x is below 1 m, drive forward 0.5 m", MOBILE)
    assert r.intent == CONDITIONAL
    assert resolve_conditional(r, None) is None
    assert resolve_conditional(r, (float("nan"), 0, 0)) is None
    b = _Bridge()                        # its state carries no pose
    route(b, "If your x is below 1 m, drive forward 0.5 m", MOBILE)
    assert b.calls == []


def test_the_router_measures_then_runs_one_branch():
    class Posed(_Bridge):
        def read_pose(self):
            return (0.2, 0.0, 0.0)
    b = Posed()
    out = route(b, "If your x is below 1 m, drive forward 0.5 m; otherwise "
                   "turn left 90 degrees", MOBILE)
    assert b.calls == [("drive", 0.5)]
    assert out["tools"][0][0] == "evaluate_condition"


def test_evaluate_condition_reads_signed_and_sized_values():
    assert evaluate_condition({"var": "yaw", "op": "gt", "a": 0.5, "abs": True},
                              (0, 0, -0.6))
    assert not evaluate_condition({"var": "yaw", "op": "gt", "a": 0.5,
                                   "negate": True}, (0, 0, 0.6))
    assert evaluate_condition({"var": "x", "op": "within", "a": 0.1, "b": 1.0},
                              (1.05, 0, 0))


def test_only_a_conditional_is_resolved():
    assert resolve_conditional(interpret("drive forward 1 m", MOBILE), (0, 0, 0)) is None
