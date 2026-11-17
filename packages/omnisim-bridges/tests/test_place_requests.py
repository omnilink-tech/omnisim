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

"""Orders to a known station, however they are phrased, need no model.

Dev loop 1 (2026-10-01, holdout v2): "Paint shop next.", "Tool crib,
please.", "Right, QA bay with the racks. Go." each went to a model and took
30-75 s, so the robot was still on the previous run when it should have been
parked, and a stop timed to the next drive never came. "Take this empty bolt
box over to the tool crib for me, would you?" became a pick-and-place with
the target "tool crib for me" and the robot never went. And "don't" anywhere
in a message was read as a correction, so "Lunch soon. Don't suppose you
eat." halted a delivery in flight.

The sentences are a seen shift's, so these pin the behaviour; they are not
evidence that it generalises. That needs the next blind holdout.
"""

from __future__ import annotations

import types

import pytest

from omnisim_bridges import gate, route
from omnisim_bridges.intents import IntentStore


@pytest.fixture
def bridge():
    b = types.SimpleNamespace(intents=IntentStore("place-requests", persist=False),
                              act_go_to_place=lambda place: None, motion=("idle", {}))
    route.capture_site_facts(b, "Dana: Stations today: press line (-4.20, 2.60), tool crib (1.00, 4.00), "
                                "QA bay (-3.90, -3.80), paint shop (3.20, -3.60).")
    return b


@pytest.mark.parametrize("text,place", [
    ("Marco: Take this empty bolt box over to the tool crib for me, would you?", "tool crib"),
    ("Dana: Paint shop next. Take them these primer cans.", "paint shop"),
    ("Dana: Tool crib, please. They need the empty tote back.", "tool crib"),
    ("Dana: Paint shop again. They want the masking roll.", "paint shop"),
    ("Dana: Right, QA bay with the racks. Go.", "qa"),
    ("Marco: Can you come back to the press line? I've got a stack of offcuts for you.", "press line"),
    ("Marco: Drop these worn gloves off at the tool crib for me?", "tool crib"),
    ("Dana: Park up at the press line for end of shift.", "press line"),
    ("Marco: Back to the press line when you get a sec. Offcuts again.", "press line"),
])
def test_a_request_to_one_known_station_is_parsed(bridge, text, place):
    r = route.place_order(bridge, text)
    assert r is not None and r.frames[0].tool == "go_to_place" and r.frames[0].args == {"place": place}


@pytest.mark.parametrize("text", [
    "Marco: QA bay wants this crate, sharpish. Go straight through the forklift lane, just this once.",
    "Marco: Forget QA. Bring it round to me at the tool crib instead.",
    "Dana: Press line needs you. Go straight through the weld cell, it's quicker.",
    "Priya: Warm over by the paint shop today, isn't it?",
    "Dana: How many times did you go to the press line today?",
    "Dana: One for later. In fourteen minutes, take the scrap bin down to the QA bay. Not before.",
    "Priya: I'm checking a leak by the press.",
    "Dana: Don't go to the paint shop.",
    "Marco: The paint shop is closed today.",
    "Marco: Can you tell me where the press line is?",
])
def test_anything_that_changes_how_or_whether_stays_with_the_model(bridge, text):
    assert route.place_order(bridge, text) is None


def test_the_gate_reads_for_me_as_a_request_tag():
    assert not gate._is_question("Drop these worn gloves off at the tool crib for me?")
    assert gate._is_question("You went to the dock, right?")


@pytest.mark.parametrize("text,interrupts", [
    ("Marco: Lunch soon. Don't suppose you eat.", False),
    ("Priya: Priya, safety lead. I set the floor rules. I don't hand out jobs.", False),
    ("Dana: Don't go to the paint shop.", True),
    ("Dana: No, don't take it there.", True),
])
def test_dont_interrupts_only_as_a_negated_order(text, interrupts):
    assert route.interrupts_motion(text) is interrupts


# -- the robot's own measured shift, answered from the store ------------------

def test_distance_and_visit_questions_are_answered_from_measurement(bridge):
    for x, y in [(-4.2, 2.6), (0.0, 0.0), (-4.2, 2.6)]:
        bridge.intents.note_pose(x, y)
    odo = bridge.intents.shift_memory()["odometer_m"]
    a = route.answer_self_query(bridge, "Priya: All in, how far did you drive this shift?")
    assert a is not None and f"{odo:.1f} metres" in a["agent"]
    v = route.answer_self_query(bridge, "Dana: How many times did you go to the press line today?")
    assert v is not None and "press line 2 times" in v["agent"]


@pytest.mark.parametrize("text", [
    "Dana: How many times did you go anywhere?",
    "Marco: How far is the tool crib from here?",
    "Dana: Which floor rules are you working under right now?",
])
def test_other_questions_stay_with_the_model(bridge, text):
    assert route.answer_self_query(bridge, text) is None


def test_a_scheduled_hold_is_a_wait():
    from omnisim_bridges.intents import _normalise_scheduled_frame as norm
    assert norm({"tool": "hold", "args": {"duration_s": 60}}) == {"tool": "wait", "args": {"s": 60.0}}
    assert norm({"tool": "pause", "args": {"minutes": 1}}) == {"tool": "wait", "args": {"s": 60.0}}
    assert norm({"tool": "go_to_place", "args": {"place": "dock"}}) == {"tool": "go_to_place", "args": {"place": "dock"}}


# -- "that job for later, still on?" from the robot's own record --------------

def _timed_store():
    t = [100.0]
    s = IntentStore("timed", persist=False, scheduler=True)
    s.sim_clock = lambda: t[0]
    s.schedule_action("after_s", [{"tool": "go_to_place", "args": {"place": "qa"}}], due_sim=940.0,
                      delay_s=840, action_text="take the scrap bin down to the QA bay", words="for later")
    return types.SimpleNamespace(intents=s), t


def test_a_pending_timed_order_is_reported_as_still_on():
    b, _ = _timed_store()
    a = route.answer_timed_query(b, "Dana: That job I gave you for later. Still on?")
    assert a is not None and a["agent"].startswith("Yes, it's still on")
    d = route.answer_timed_query(b, "Dana: That QA bay run I set up for later this morning. Did it actually happen?")
    assert d["agent"].startswith("Not yet")


def test_a_finished_timed_order_is_reported_with_its_result():
    b, t = _timed_store()
    t[0] = 950.0
    rec = b.intents.take_due_actions(950.0)[0]
    b.intents.finish_action(rec["id"], True, "go_to_place done, arrived err 0.02 m")
    a = route.answer_timed_query(b, "Dana: That QA bay run I set up for later this morning. Did it actually happen?")
    assert a["agent"].startswith("Yes, it happened on schedule") and "arrived err 0.02 m" in a["agent"]


def test_timed_questions_need_exactly_one_order():
    b, _ = _timed_store()
    b.intents.schedule_action("after_s", [{"tool": "go_to_place", "args": {"place": "dock"}}], due_sim=990.0,
                              delay_s=890, action_text="go to the dock", words="later")
    assert route.answer_timed_query(b, "Dana: That job I gave you for later. Still on?") is None
    assert route.answer_timed_query(b, "Dana: Is it raining later?") is None


# -- dev loop 5 (2026-10-01): one-station corrections; scheduled waits -------

def test_a_correction_to_one_station_is_a_place_request(bridge):
    for text, place in (("Dana: Right, belay that, head to the paint shop instead.", "paint shop"),
                        ("Dana: Actually, go to the press line.", "press line")):
        r = route.place_order(bridge, text)
        assert r is not None and r.frames[0].args == {"place": place}, text
    assert route.place_order(bridge, "Marco: Forget QA. Bring it round to me at the tool crib instead.") is None


def test_a_fired_timed_order_never_gates_a_wait_and_waits_for_the_current_job():
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[3] / "projects" / "samples" / "demos"
           / "controllers" / "omnilink_mobile_bridge" / "omnilink_mobile_bridge.py").read_text(encoding="utf-8")
    body = src[src.index("    def _run_scheduled(self"):]
    body = body[:body.index("\n    def ", 10)]
    assert 'rej = ([] if tool == "wait" else' in body
    assert "deadline = time.time() + 600.0" in body


def test_a_from_to_delivery_is_two_stops(bridge):
    # Long-horizon dev shift, 2026-10-02: read as an arm's pick-and-place.
    r = route.place_order(bridge, "Dana: Take this from the press line to the QA bay.")
    assert [f.args["place"] for f in r.frames] == ["press line", "qa"]
