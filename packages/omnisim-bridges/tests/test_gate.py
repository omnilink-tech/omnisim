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

"""The gate, tested against frames the parser would never produce.

That is the whole point of it. `test_interpret.py` checks that the parser
declines the dangerous cases; these check that the dangerous cases are
STILL refused when something else produced the frame -- a model
interpreting the messy 70%, a bridge client, a future tier-1.

Every frame below is hand-written to be exactly what a plausible
interpreter gets wrong. None of them comes from `interpret`.
"""
from __future__ import annotations

import math

import pytest

from omnisim_bridges.gate import MAX_DISTANCE_M, check


def F(tool, **args):
    """A frame as a bare dict -- the shape a non-parser producer emits."""
    return {"tool": tool, "args": args}


def rules(rejections):
    return {r.rule for r in rejections}


# ── The class of mistake that motivated the whole gate. ─────────────
@pytest.mark.parametrize("utterance,frame", [
    ("how many times have you had to stop?", F("stop")),
    ("how much charge is left?", F("turn", angle_rad=1.57)),
    ("did you drive forward earlier?", F("drive_forward", distance=1.0)),
    ("where did the package land?", F("land")),
    ("what happens if you turn left?", F("turn", angle_rad=1.57)),
    ("could you drive 2 metres, in principle?", F("drive_forward", distance=2.0)),
])
def test_a_question_may_never_produce_motion(utterance, frame):
    """A model asked a question can still emit an action. This is the net.

    The shipped keyword ladder did exactly this on real traffic -- 38% of
    questions actuated the robot. A language model does it less often and
    less predictably, and cannot be read to find out why.
    """
    assert "interrogative" in rules(check(utterance, [frame]))


def test_a_polite_imperative_is_not_blocked():
    """"can you stop?" ends in a question mark and is an order."""
    assert check("Can you stop?", [F("stop")]) == []


def test_a_question_may_still_be_answered():
    """Read-only tools are what a question is FOR."""
    assert check("where are you?", [F("get_robot_state", kind="pose")]) == []
    assert check("how many carts?", [F("clarify")]) == []


# ── The other utterance-versus-frame rules. ─────────────────────────
def test_a_prohibition_may_not_produce_the_thing_it_forbids():
    assert "prohibition" in rules(
        check("do not drive forward 2 metres", [F("drive_forward", distance=2.0)]))


def test_a_self_negating_order_produces_nothing():
    assert "self_negating" in rules(
        check("drive forward 1 metre without moving", [F("drive_forward", distance=1.0)]))


def test_an_implausible_magnitude_is_refused():
    r = rules(check("drive forward 500 metres", [F("drive_forward", distance=500.0)]))
    assert "implausible" in r


def test_a_plausible_magnitude_is_not():
    assert check("drive forward 5 metres", [F("drive_forward", distance=5.0)]) == []


def test_direction_and_sign_must_agree():
    assert "sign_conflict" in rules(
        check("drive forward -2 metres", [F("drive_forward", distance=-2.0)]))


def test_reverse_is_still_allowed():
    assert check("back up 2 metres", [F("drive_forward", distance=-2.0)]) == []


# ── Schema. A model will invent arguments and tools. ────────────────
def test_an_invented_tool_is_refused():
    assert "unknown_tool" in rules(check("engage the warp drive", [F("warp", factor=9)]))


def test_an_invented_argument_is_refused():
    # ⚠️ This used `speed=99` as its invented argument, and `speed` turned
    # out to be a REAL parameter of the bridge's drive_forward. The test
    # passed for the wrong reason, and declaring the parameter honestly
    # broke it. Use an argument nobody ships.
    assert "unknown_arg" in rules(
        check("drive forward 1 metre", [F("drive_forward", distance=1.0, turbo=True)]))


def test_a_real_argument_with_an_absurd_value_is_refused_on_magnitude():
    """`speed` is legitimate; 99 m/s is not.

    The distinction matters because it is the one a model will find. An
    argument that is refused only because the schema forgot it is refused
    by luck, and the same luck rejects valid calls -- measured in
    tests/benchmarks/interpbench, where 4 of 8 apparent catches were
    schema drift rather than judgement.
    """
    out = rules(check("drive forward 1 metre",
                      [F("drive_forward", distance=1.0, speed=99)]))
    assert "implausible" in out
    assert "unknown_arg" not in out


def test_a_sane_speed_is_allowed():
    assert check("drive forward 1 metre",
                 [F("drive_forward", distance=1.0, speed=0.5)]) == []


def test_a_missing_required_argument_is_refused():
    assert "missing_arg" in rules(check("drive to the bay", [F("drive_to", x=1.0)]))


@pytest.mark.parametrize("value", ["fast", None, True])
def test_a_wrong_type_is_refused(value):
    assert "bad_type" in rules(
        check("drive forward", [F("drive_forward", distance=value)]))


@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_a_non_finite_number_is_refused(value):
    """NaN reaches a motor as a silent no-op or a solver explosion."""
    assert "not_finite" in rules(
        check("drive forward", [F("drive_forward", distance=value)]))


def test_an_unresolved_referent_never_reaches_an_actuator():
    assert "unresolved_referent" in rules(
        check("put it over there", [F("place", object="it", target="there")]))


# ── The clean path still runs. ──────────────────────────────────────
@pytest.mark.parametrize("utterance,frame", [
    ("drive forward 2 metres", F("drive_forward", distance=2.0)),
    ("turn left 90 degrees", F("turn", angle_rad=math.pi / 2)),
    ("drive to x 3, y -1", F("drive_to", x=3.0, y=-1.0)),
    ("stop", F("stop")),
    ("wave hello", F("wave")),
    ("take off to 2 metres", F("takeoff", altitude=2.0)),
    ("pick up the cube", F("pick", object="cube")),
])
def test_a_good_frame_passes(utterance, frame):
    assert check(utterance, [frame]) == []


def test_a_compound_order_passes_as_a_whole():
    assert check("turn left 90 degrees, then drive forward 1 metre",
                 [F("turn", angle_rad=math.pi / 2),
                  F("drive_forward", distance=1.0)]) == []


def test_one_bad_frame_in_a_batch_is_caught():
    bad = check("turn left 90 degrees, then drive forward 1 metre",
                [F("turn", angle_rad=math.pi / 2),
                 F("drive_forward", distance=MAX_DISTANCE_M * 10)])
    assert "implausible" in rules(bad)


# ── The gate is wired into the executor, not just importable. ───────
class _B:
    def __init__(self):
        self.calls = []

    def act_drive_forward(self, distance, speed=None, wait=False):
        self.calls.append(distance)
        return {"accepted": True}


def test_defence_in_depth_the_parser_catches_it_first():
    """Both layers hold, and the parser is the one that fires here.

    The parser's own rail turns "500 metres" into a question before a frame
    exists, so the gate never sees it. That is the desired order -- asking
    is a better answer than refusing -- and the bridge is untouched either
    way, which is the assertion that matters.
    """
    from omnisim_bridges.route import route

    b = _B()
    out = route(b, "drive forward 500 metres", "mobile")
    assert out is not None
    assert b.calls == [], "nothing may reach the bridge"
    assert out["tools"][0][1] in ("ask", "refused")


def test_the_gate_catches_a_frame_the_parser_never_saw():
    """The case the gate exists for: a frame from somewhere else.

    Built by hand and pushed straight through the executor, exactly as a
    model-produced frame would arrive once interpretation moves off the
    parser. No parse guard can help here, and the bridge must still not
    move.
    """
    from omnisim_bridges.interpret import COMMAND, Frame, Interpretation
    from omnisim_bridges.route import execute

    hostile = Interpretation(
        COMMAND,
        frames=[Frame("drive_forward", {"distance": 500.0})],
        confidence=0.99,
        text="how far could you drive if I asked you to?",
    )
    b = _B()
    out = execute(b, hostile)
    assert b.calls == [], "the gate must stop this before the bridge sees it"
    assert out["tools"][0][1] == "refused"
    # PROTOCOL.md §5.7.2, as of the v9 envelope work: the tuple is
    # (tool, result, summary, rule). The RULE NAME moved out of `summary`
    # into its own field -- `summary` is prose a client must not branch on,
    # `rule` is the machine-readable name it must. Before this, the rule
    # name sat in `summary` and `first.detail` ("distance=500.0 exceeds the
    # 50 m rail") was dropped on the floor, so the envelope carried no
    # human-readable account of the refusal at all.
    assert out["tools"][0][3] in ("interrogative", "implausible")
    assert out["tools"][0][2] and " " in out["tools"][0][2]   # prose
