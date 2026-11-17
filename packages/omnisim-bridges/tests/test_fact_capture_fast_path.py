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

"""Facts a shift opens with are recorded and answered without a model.

Dev-loop probe P3 (2026-10-01): introductions, a station list and a keep-out
rule four seconds apart each went to a model one after another; the first
job waited behind them and reached the dock over a minute late, so the
second job never started. The station list ("dock (3.00, 0.00)") was not
even captured -- only the "dock is x=3, y=0" form was -- and "safety lead"
was stored as SUPERVISOR because "lead" is a supervisor word checked first.
"""

from __future__ import annotations

import types

import pytest

from omnisim_bridges import route
from omnisim_bridges.intents import IntentStore, normalize_role


def _bridge():
    return types.SimpleNamespace(intents=IntentStore("facts-test", persist=False))


def _facts(b, text):
    return route.capture_site_facts(b, text) + route.capture_roles(b, text)


def test_safety_lead_is_the_safety_role_not_the_supervisor():
    assert normalize_role("safety lead") == "safety"
    assert normalize_role("shift supervisor") == "supervisor"
    assert normalize_role("shift lead") == "supervisor"


def test_a_station_list_is_captured():
    b = _bridge()
    f = _facts(b, "Dana: Stations today: dock (3.00, 0.00), charger (-2.00, 2.00), scrap bay (-2.00, -2.50).")
    assert f == ["place dock", "place charger", "place scrap bay"]
    assert b.intents.place_xy("dock") == pytest.approx((3.0, 0.0))
    assert b.intents.place_xy("charger") == pytest.approx((-2.0, 2.0))


@pytest.mark.parametrize("text,who,role", [
    ("Dana: Morning, cart. Dana here, shift supervisor.", "Dana", "supervisor"),
    ("Priya: Safety lead here. Keep it tidy.", "Priya", "safety"),
    ("Marco: Marco, I run the presses.", "Marco", "worker"),
    ("Dana: Dana, shift supervisor. Priya's our safety lead.", "Priya", "safety"),
])
def test_introductions_record_roles(text, who, role):
    b = _bridge()
    _facts(b, text)
    assert b.intents.roles.get(who) == role


@pytest.mark.parametrize("text", [
    "Dana: Stations today: dock (3.00, 0.00), charger (-2.00, 2.00).",
    "Priya: Safety lead here. The aisle x 0.80 to 1.60, y 1.00 to 3.00 is off limits all shift.",
    "Dana: Morning, cart. Dana, shift supervisor. Priya's our safety lead.",
])
def test_a_message_that_only_tells_is_answered_without_a_model(text):
    b = _bridge()
    out = route.answer_facts(b, text, _facts(b, text))
    assert out is not None and out["agent"].startswith("Got it")


def test_the_answer_says_only_what_was_stored():
    b = _bridge()
    text = "Priya: Safety lead here. The aisle x 0.80 to 1.60, y 1.00 to 3.00 is off limits all shift."
    out = route.answer_facts(b, text, _facts(b, text))
    assert "Priya is the safety lead" in out["agent"]
    assert "keep out of the aisle for the whole shift" in out["agent"]
    assert [z["locked"] for z in b.intents.zones()] == [True]


@pytest.mark.parametrize("text", [
    "Dana: Stations today: dock (3.00, 0.00). Take this crate to the dock.",   # an order too
    "Dana: Morning, cart. Dana here, shift supervisor. Jobs come from me, and Marco can send you on runs too.",
    "Marco: Marco, I run the presses. I'll shout when I need stuff moved.",
    "Priya: Is the dock at (3, 0)?",
    "Dana: First job: take this crate to the dock.",
])
def test_anything_more_than_facts_still_goes_to_the_model(text):
    b = _bridge()
    assert route.answer_facts(b, text, _facts(b, text)) is None


# -- a parsed order that supersedes a running turn does not queue behind it --

def _with_stations():
    b = _bridge()
    b.act_go_to_place = lambda place: None
    route.capture_site_facts(b, "Dana: Stations today: dock (3.00, 0.00), charger (-2.00, 2.00).")
    return b


@pytest.mark.parametrize("text,place", [
    ("Dana: First job: take this crate to the dock.", "dock"),
    ("Next up: run this bin to the charger.", "charger"),
    ("Another run - take this to the dock, please.", "dock"),
])
def test_a_job_label_does_not_hide_a_place_order(text, place):
    r = route.place_order(_with_stations(), text)
    assert r is not None and r.frames[0].args == {"place": place}


@pytest.mark.parametrize("text,acts", [
    ("Dana: First job: take this crate to the dock.", True),
    ("Dana: drive forward 2 metres.", True),
    # an idle robot: "when you're done there" is soon, not a condition (2026-10-01)
    ("Marco: When you're done there, run this bin to the charger.", True),
    ("Dana: How far is the dock?", False),
    ("Dana: Morning, cart.", False),
])
def test_parser_will_act_is_a_pure_decision(text, acts):
    b = _with_stations()
    before = dict(route._STATS["by_intent"])
    assert route.parser_will_act(b, text, "mobile") is acts
    assert route._STATS["by_intent"] == before        # records nothing


def test_the_bridge_lets_a_superseding_parsed_order_skip_the_lock():
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[3] / "projects" / "samples" / "demos"
           / "controllers" / "omnilink_mobile_bridge" / "omnilink_mobile_bridge.py"
           ).read_text(encoding="utf-8")
    post = src[src.index("        def do_POST(self):"):]
    post = post[:post.index("            except RequestError as e:")]
    assert "takes_over = bool(shared_parser_will_act is not None" in post
    assert "or halt or aside or takes_over):" in post
    # decided only after the implicit halt, never for an ordinary message
    assert post.index("bridge.act_stop(keep_scheduled=True)") < post.index("takes_over = bool(")


@pytest.mark.parametrize("text,place", [
    ("Marco: When you're done there, run this bin to the charger.", "charger"),
    ("Marco: run this bin to the charger when you can.", "charger"),
])
def test_a_soon_idiom_does_not_block_a_place_order_on_an_idle_robot(text, place):
    b = _with_stations()
    b.motion = ("idle", {})
    r = route.place_order(b, text)
    assert r is not None and r.frames[0].args == {"place": place}
    b.motion = ("drive", {})                      # busy: "done there" is after the job
    assert route.place_order(b, text) is None


def test_a_real_condition_still_blocks_a_place_order():
    b = _with_stations()
    b.motion = ("idle", {})
    assert route.place_order(b, "Marco: Take this to the dock when the forklift has gone.") is None


# -- a hold is engaged when it is said, not when a model gets to it ----------

@pytest.mark.parametrize("text,held", [
    ("Priya: Cart, hold where you are, don't move until I say. I'm checking a leak.", True),
    ("Dana: Stay put till I'm back.", True),
    ("Priya: Stop.", False),                                       # a stop is not a hold
    ("Priya: If you see a spill, hold there until I come.", False),  # a rule for later
    ("Priya: Can you hold there until I say?", False),               # a question
])
def test_a_hold_is_engaged_the_moment_it_is_said(text, held):
    b = _bridge()
    b.intents.set_turn_text(text)
    route.capture_hold(b, text)
    assert b.intents.hold_active() is held
    if held:
        assert b.intents.hold_speaker in ("Priya", "Dana")


def test_the_gate_reads_availability_idioms_as_now():
    from omnisim_bridges import gate
    assert not gate._is_deferred("When you're done there, run this bin to the charger.")
    assert not gate._is_deferred("Back to the press line when you get a sec.")
    assert gate._is_deferred("Drive to the dock when the forklift has gone.")
