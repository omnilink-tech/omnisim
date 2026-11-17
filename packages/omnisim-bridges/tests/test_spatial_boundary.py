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
"""A standing spatial rule is set in one turn, lifted in another, and holds
on every move in between.

WHY THIS EXISTS. ops-bench F1 (2026-09-25) found three gaps at once:

1. The intent store's only rules were named warehouse zones; a coordinate
   limit ("keep your x at or below 0.4") had no representation at all.
2. The parser read "Until I say so, keep your x ... at or below 0.40" as
   FREEZE until told (hold), and "don't let your x go above 0.5" as an
   unenforceable named constraint.
3. No model-called drive was ever checked against any rule: constraints
   were consulted by the autonomy loop only.

The mobile bridge cannot be imported off an engine, so the clipping maths
lives in `intents.clip_drive` and is tested here; the bridge's call to it is
checked in source.
"""
from __future__ import annotations

import math
import pathlib

import pytest

from omnisim_bridges import interpret as I
from omnisim_bridges import route
from omnisim_bridges.intents import IntentStore, clip_drive

ROOT = pathlib.Path(__file__).resolve().parents[3]
MOBILE = ROOT / "projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py"


def frames(text):
    r = I.interpret(text, "mobile")
    return r.intent, [(f.tool, f.args) for f in r.frames]


@pytest.mark.parametrize("text,axis,value,side", [
    ("Until I say so, keep your x coordinate at or below 0.40.", "x", 0.4, "max"),
    ("New safety rule: do not let your x coordinate go above 0.50 until I lift the rule.", "x", 0.5, "max"),
    ("Keep your y above -1.5.", "y", -1.5, "min"),
    ("Your x must not exceed 3.", "x", 3.0, "max"),
    ("Don't let y drop below -0.5 m.", "y", -0.5, "min"),
    ("Stay behind the line x = 0.60: your x coordinate must not go above that until I tell you otherwise.",
     "x", 0.6, "current"),
    ("From now on, never go past x = 2.", "x", 2.0, "current"),
])
def test_rules_parse_as_a_boundary(text, axis, value, side):
    intent, fr = frames(text)
    assert intent == I.COMMAND
    assert fr == [("boundary", {"axis": axis, "value": value, "side": side})]


def test_a_rule_then_an_order_keeps_both_in_order():
    intent, fr = frames("Stay behind the line x = 0.6, then turn left 90 degrees")
    assert [f[0] for f in fr] == ["boundary", "turn"]


@pytest.mark.parametrize("text", ["That line no longer applies.", "You can cross the line now.",
                                  "Lift the boundary.", "The restriction is lifted."])
def test_lifts_parse(text):
    assert frames(text)[1] == [("clear_boundary", {})]


def test_a_lift_then_an_order():
    assert [f[0] for f in frames("That line no longer applies. Drive forward 0.5 metres.")[1]] == \
        ["clear_boundary", "drive_forward"]


@pytest.mark.parametrize("text", [
    "If a supervisor ever told you to stay behind x = 0.30, could you do that? Just answer, do not move.",
    "Is the rule lifted?", "What is your x?", "Don't go past the kitchen.",
    "keep the box at x 0.5", "x is 0.5 now, right",
    # A rule plus motion the guard cannot read exactly goes to the model whole.
    "Stay behind x = 0.6 and drive forward 2 m",
])
def test_not_a_parsed_rule(text):
    intent, fr = frames(text)
    assert not any(f[0] in ("boundary", "clear_boundary") for f in fr)
    assert intent != I.COMMAND or not fr or fr[0][0] != "boundary"


def test_a_plain_hold_is_still_a_hold():
    assert frames("Stop until I say so.")[1] == [("stop", {}), ("hold", {"release": "operator"})]


# ── the store ───────────────────────────────────────────────────────────

def store(tmp_path, **kw):
    return IntentStore("probe", state_path=str(tmp_path / "s.json"), spatial=True, **kw)


def test_set_and_list_boundary(tmp_path):
    s = store(tmp_path)
    res = s.set_boundary("x", max_value=0.6, words="stay behind x = 0.6")
    assert res["accepted"] and s.boundaries()[0]["max"] == 0.6
    assert "x <= 0.60" in s.state()["intents_summary"]


@pytest.mark.parametrize("axis,kw", [("z", {"max_value": 1}), ("x", {}),
                                     ("x", {"max_value": 1, "min_value": 0}),
                                     ("x", {"max_value": "nan"})])
def test_malformed_boundaries_are_refused_with_a_sentence(tmp_path, axis, kw):
    res = store(tmp_path).set_boundary(axis, **kw)
    assert res["accepted"] is False and res["say"]


def test_same_turn_cannot_lift_but_a_later_turn_can(tmp_path):
    s = store(tmp_path)
    s.set_turn_text("keep x below 0.6")
    s.set_boundary("x", max_value=0.6)
    assert s.clear_constraint("boundary")["accepted"] is False
    s.set_turn_text("the line no longer applies")
    assert s.clear_constraint("boundary")["accepted"] is True
    assert s.boundaries() == []


def test_a_boundary_survives_a_reload(tmp_path):
    s = store(tmp_path)
    s.set_boundary("y", min_value=-1.0)
    again = store(tmp_path)
    assert again.boundaries() and again.boundaries()[0]["min"] == -1.0


def test_boundary_tools_only_where_enforced(tmp_path):
    from omnisim_bridges.intents import build_intent_tools
    from omnisim_bridges.tool import Tool
    names = lambda st: {t.name for t in build_intent_tools(Tool, st)}
    assert {"set_boundary", "clear_boundary"} <= names(store(tmp_path))
    plain = IntentStore("p2", state_path=str(tmp_path / "p.json"))
    assert not ({"set_boundary", "clear_boundary"} & names(plain))


# ── the clip ────────────────────────────────────────────────────────────

B = [{"id": "rule-1", "axis": "x", "max": 0.6, "means": "keep x <= 0.60"}]


def test_a_drive_toward_the_line_stops_short_of_it():
    hit = clip_drive(B, 0.0, 0.0, 0.0, 1.5, margin=0.05)
    assert hit["allowed"] == pytest.approx(0.55)


def test_a_drive_away_from_the_line_is_not_limited():
    assert clip_drive(B, 0.0, 0.0, 0.0, -1.5) is None
    assert clip_drive(B, 0.0, 0.0, math.pi, 1.5) is None


def test_a_drive_that_stays_inside_is_not_limited():
    assert clip_drive(B, 0.0, 0.0, 0.0, 0.3) is None


def test_at_the_line_there_is_no_usable_room():
    # 1 cm short of the line at most -- under the bridge's 2 cm minimum move,
    # so the drive is refused, with the rule named.
    assert clip_drive(B, 0.58, 0.0, 0.0, 1.0)["allowed"] <= 0.02


def test_an_oblique_drive_is_limited_by_its_component():
    hit = clip_drive(B, 0.0, 0.0, math.pi / 3, 2.0, margin=0.0)   # cos 60 = 0.5
    assert hit["allowed"] == pytest.approx(1.2)


def test_a_reverse_drive_meets_a_min_boundary():
    hit = clip_drive([{"axis": "x", "min": -0.5}], 0.0, 0.0, 0.0, -2.0, margin=0.0)
    assert hit["allowed"] == pytest.approx(0.5)


# ── execution through the real gate ─────────────────────────────────────

class _Bridge:
    def __init__(self, tmp_path, x=0.0, spatial=True):
        self.intents = IntentStore("b", state_path=str(tmp_path / "b.json"), spatial=spatial)
        self.x = x
        self.calls = []

    def _read_pose(self):
        return (self.x, 0.0, 0.0)

    def act_drive_forward(self, distance, wait=False):
        self.calls.append(distance)
        return {"accepted": True, "commanded": distance, "achieved": distance,
                "error": 0.0, "settled": True}


@pytest.mark.parametrize("x,side", [(0.0, "max"), (1.0, "min")])
def test_the_line_side_is_the_side_the_robot_is_on(tmp_path, x, side):
    b = _Bridge(tmp_path, x=x)
    out = route.short_circuit(b, "Stay behind the line x = 0.6 until I say so.", "mobile")
    assert out is not None and out["tools"][0][1] == "ok"
    assert b.intents.boundaries()[0][side] == 0.6


def test_on_the_line_it_asks_which_side(tmp_path):
    b = _Bridge(tmp_path, x=0.6)
    out = route.short_circuit(b, "Stay behind the line x = 0.6.", "mobile")
    assert out["tools"][0][1] == "ask" and b.intents.boundaries() == []


def test_a_robot_that_cannot_enforce_never_agrees(tmp_path):
    # Nothing but `unsupported` -> the parser declines the turn (the model
    # gets it, and has no boundary tool on this store), never "noted".
    b = _Bridge(tmp_path, spatial=False)
    out = route.short_circuit(b, "Keep your x at or below 0.4.", "mobile")
    assert out is None and b.intents.boundaries() == []


def test_lift_then_drive_executes_both(tmp_path):
    b = _Bridge(tmp_path)
    b.intents.set_turn_text("keep x below 0.6")
    b.intents.set_boundary("x", max_value=0.6)
    b.intents.set_turn_text("That line no longer applies. Drive forward 0.5 metres.")
    route.short_circuit(b, "That line no longer applies. Drive forward 0.5 metres.", "mobile")
    assert b.intents.boundaries() == [] and b.calls == [0.5]


def test_mobile_bridge_clips_every_drive_through_clip_drive():
    src = MOBILE.read_text(encoding="utf-8")
    body = src[src.index("    def act_drive_forward(self"):]
    body = body[:body.index("\n    def ", 10)]
    assert "self._boundary_clip(" in body
    assert "clip_drive(store.boundaries()" in src
    assert "spatial=True" in src


def test_a_clipped_drive_is_reported_as_clipped_not_as_asked(tmp_path):
    # The parser used to answer "Drive forward (distance=1.0)." over a drive
    # the bridge had shortened to 0.35 m (ops-bench F1, 2026-09-25).
    class Clipping(_Bridge):
        def act_drive_forward(self, distance, wait=False):
            self.calls.append(distance)
            return {"accepted": True, "commanded": 0.35, "requested": distance,
                    "clipped_by": {"id": "rule-1", "means": "keep x <= 0.40 m"}}
    b = Clipping(tmp_path)
    out = route.short_circuit(b, "Drive forward 1 metre.", "mobile")
    assert "shortened from 1.00 to 0.35" in out["agent"] and "x <= 0.40" in out["agent"]


def test_a_drive_along_the_line_from_inside_the_margin_is_not_refused():
    # Clipped at y = -0.15 against y >= -0.2, then turned to drive along the
    # line with a 0.02 rad heading error: the old margin arithmetic left zero
    # room and refused the whole drive (ops-bench shift_tb3_dev).
    b = [{"axis": "y", "min": -0.2}]
    assert clip_drive(b, 0.5, -0.15, -0.02, 1.0) is None
    # Straight at the line from inside the band: only to 1 cm short of it.
    hit = clip_drive(b, 0.5, -0.15, -math.pi / 2, 1.0)
    assert hit["allowed"] == pytest.approx(0.04)


@pytest.mark.parametrize("text", [
    "Morning. Rule for today: don't let your x go above 1.5 until I say otherwise.",
    "Hi! For now, keep your x at or below 1.5.",
])
def test_a_greeting_does_not_hide_a_rule(text):
    intent, fr = frames(text)
    assert intent == I.COMMAND and fr and fr[0][0] == "boundary"
