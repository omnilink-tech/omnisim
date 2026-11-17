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

"""What a three-hour shift needs the robot to keep without a model.

Long-horizon dev shift, run 1 (2026-10-02, OmniLink 0.690): a standing rule
("whenever a porter is in the corridor, hold") halted the robot on the spot;
"the usual run" was defined once and then had to be re-derived by a model
every time; a handover ("I'm off, Ruth's got the floor") was not recorded;
"Delivery notes to Ward 3" was refused as reported speech; and a mid-shift
controller restart lost the arrivals count, the odometer and the timed
orders. The sentences are a seen shift's, so these pin behaviour; they are
not evidence it generalises -- that is the blind holdout's job.
"""

from __future__ import annotations

import types

import pytest

from omnisim_bridges import gate, route
from omnisim_bridges import interpret as _i
from omnisim_bridges.intents import IntentStore

SITE = ("Ruth: Stops today: Pharmacy (-4.00, 2.50), Ward 3 (1.00, 4.00), "
        "Lab (-3.50, -3.50), Stores (3.00, -3.00).")


@pytest.fixture
def bridge():
    b = types.SimpleNamespace(intents=IntentStore("long-shift", persist=False),
                              act_go_to_place=lambda place: None, motion=("idle", {}))
    route.capture_site_facts(b, SITE)
    return b


# -- routines ---------------------------------------------------------------

DEFINE = ("Ruth: When I say 'the usual run' I mean Pharmacy, then Ward 3, then the Lab, "
          "in that order, with five seconds at each stop.")


def test_a_routine_is_recorded_the_moment_it_is_said(bridge):
    assert route.capture_shift_facts(bridge, DEFINE) == ["routine the usual run"]
    rt = bridge.intents.routine("the usual run")
    assert rt["stops"] == ["pharmacy", "ward 3", "lab"] and rt["dwell_s"] == 5.0


def test_running_a_routine_needs_no_model(bridge):
    route.capture_shift_facts(bridge, DEFINE)
    r = route.routine_order(bridge, "Ruth: Do the usual run, please.")
    assert r is not None
    assert [(f.tool, f.args) for f in r.frames] == [
        ("go_to_place", {"place": "pharmacy"}), ("dwell", {"s": 5.0}),
        ("go_to_place", {"place": "ward 3"}), ("dwell", {"s": 5.0}),
        ("go_to_place", {"place": "lab"}), ("dwell", {"s": 5.0})]


def test_a_routine_question_is_answered_from_the_store(bridge):
    route.capture_shift_facts(bridge, DEFINE)
    a = route.answer_routine_query(bridge, "Sam: What's in the usual run again?")
    assert a["agent"].lower() == "the usual run is pharmacy, then ward 3, then lab, with 5 seconds at each stop."
    assert route.routine_order(bridge, "Sam: What's in the usual run again?") is None


@pytest.mark.parametrize("text", [
    "Ruth: Don't do the usual run today.",
    "Ruth: The usual run is boring.",
    "Ruth: Do the night run.",
])
def test_anything_else_about_a_routine_stays_with_the_model(bridge, text):
    route.capture_shift_facts(bridge, DEFINE)
    assert route.routine_order(bridge, text) is None


# -- handovers ----------------------------------------------------------------

@pytest.mark.parametrize("text,old,new", [
    ("Ruth: Right, I'm off. Sam's got the floor now.", "Ruth", "Sam"),
    ("Sam: Sam here, taking over from Ruth.", "Ruth", "Sam"),
])
def test_a_handover_moves_the_floor(bridge, text, old, new):
    assert route.capture_shift_facts(bridge, text) == [f"handover {old}->{new}"]
    assert bridge.intents.roles[new] == "supervisor" and bridge.intents.roles[old] == "former"


# -- a standing rule is not an order to hold now -------------------------------

def test_a_standing_rule_does_not_halt_the_robot():
    # Run 1 read this as "act now, then hold until released" and parked the
    # robot for the rest of the shift.
    r = _i.interpret("Marisol: And whenever you're at Ward 3, wait fifteen seconds before you head off, "
                     "the nurses need that long to unload. That stands until I say otherwise.")
    assert r.frames == []
    assert [f.tool for f in _i.interpret("Ruth: Hold position.").frames] == ["stop"]


# -- the gate reads "delivery notes to" as a noun ------------------------------

def test_delivery_notes_are_not_reported_speech():
    assert not gate._is_reported("Delivery notes to Ward 3.")
    assert gate._is_reported("The handbook states that you should drive forward 2 metres.")


# -- a controller restart keeps the shift ---------------------------------------

def test_a_restart_keeps_arrivals_odometer_and_routines(tmp_path):
    path = str(tmp_path / "s.json")
    b = types.SimpleNamespace(intents=IntentStore("r", state_path=path),
                              act_go_to_place=lambda place: None, motion=("idle", {}))
    route.capture_site_facts(b, SITE)
    route.capture_shift_facts(b, DEFINE)
    for x, y in [(-4.0, 2.5), (1.0, 4.0), (-4.0, 2.5)]:
        b.intents.note_pose(x, y)
    before = b.intents.shift_memory()
    b.intents.tick()                                  # flush
    again = IntentStore("r", state_path=path)
    after = again.shift_memory()
    assert after["odometer_m"] == pytest.approx(before["odometer_m"], abs=0.01)
    assert again.visits == b.intents.visits and again.visits["pharmacy"] == 2
    assert again.routine("the usual run")["stops"] == ["pharmacy", "ward 3", "lab"]
    # parked at the Pharmacy through the restart: not a new arrival
    again.note_pose(-4.0, 2.5)
    assert again.visits["pharmacy"] == 2


# -- run 2 (2026-10-02): the routine never ran ---------------------------------

class _RoutineBridge:
    """Enough of a mobile bridge for route.execute, on a clock that moves."""

    def __init__(self):
        self.intents = IntentStore("routine-run", persist=False, scheduler=True)
        self.halt_seq = 0
        self.calls = []
        self._t = 0.0

    @property
    def sim_time(self):
        self._t += 1.0
        return self._t

    def _pose(self):
        return (0.0, 0.0, 0.0)

    def act_go_to_place(self, place, wait=False):
        self.calls.append(("go_to_place", place, self._t))
        return {"accepted": True, "arrived": True, "error_m": 0.01, "settled": True}


@pytest.fixture
def served_go_to_place():
    """Register go_to_place as a live bridge does at start-up (SPECS is global)."""
    import copy
    from omnisim_bridges.tool import Tool
    saved = copy.deepcopy(gate.SPECS)
    gate.register_tools([Tool("go_to_place", "", {"type": "object",
                                                  "properties": {"place": {"type": "string"}},
                                                  "required": ["place"]},
                              lambda a: {}, physical=True)], surface="mobile")
    yield
    gate.SPECS.clear()
    gate.SPECS.update(saved)


def test_a_routine_runs_end_to_end_past_the_gate(served_go_to_place):
    # The parse was right and the gate refused its `dwell` as unknown_tool,
    # so "Do the usual run." went nowhere, twice. Pin the whole path.
    b = _RoutineBridge()
    route.capture_site_facts(b, SITE)
    route.capture_shift_facts(b, DEFINE)
    r = route.routine_order(b, "Marisol: Do the usual run.")
    out = route.execute(b, r, "mobile")
    assert out is not None and "won't" not in out["agent"]
    assert [c[1] for c in b.calls] == ["pharmacy", "ward 3", "lab"]
    # each stop's dwell elapsed on the robot's clock before the next leg
    assert b.calls[1][2] - b.calls[0][2] >= 5.0
    assert [t[0] for t in out["tools"]].count("dwell") == 3


# -- run 2: a porter redirected the supervisor's delivery, twice -----------------

def _authority_class():
    """The bridge's authority methods, lifted out of its source (it cannot be
    imported without the engine) and run on a fake bridge."""
    import ast
    import math
    import pathlib
    import re
    src = (pathlib.Path(__file__).resolve().parents[3] / "projects" / "samples" / "demos"
           / "controllers" / "omnilink_mobile_bridge" / "omnilink_mobile_bridge.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    keep = {"_order_stands_against", "_order_conflict", "_COUNTERMAND", "COUNTERMAND_WINDOW_S"}
    body = []
    for cls in (n for n in tree.body if isinstance(n, ast.ClassDef)):
        for n in cls.body:
            name = getattr(n, "name", None) or (getattr(n.targets[0], "id", None)
                                                if isinstance(n, ast.Assign) else None)
            if name in keep:
                body.append(n)
    mod = ast.Module(body=[ast.ClassDef(name="Auth", bases=[], keywords=[], body=body, decorator_list=[])],
                     type_ignores=[])
    ns = {"re": re, "math": math, "Optional": object}
    exec(compile(ast.fix_missing_locations(mod), "bridge-authority", "exec"), ns)
    return ns["Auth"]


def _porter_case(pose, sim_time, text):
    store = IntentStore("auth", persist=False)
    store.roles.update({"Marisol": "supervisor", "Kemal": "worker"})
    b = _authority_class()()
    b.intents, b.sim_time = store, sim_time
    b._last_order = {"kind": "place", "goal_xy": [4.2, -4.4], "means": "go to ward 3", "sim_s": 1005.0,
                     "by": "Marisol"}
    b._read_pose = lambda: (*pose, 0.0)
    store.set_turn_text(text)
    return b._order_conflict()


def test_a_porter_cannot_redirect_the_supervisors_delivery():
    redirect = "Kemal: Nah, forget Ward 3, take the packs to the Sterile store instead."
    # under way: refused, as before
    assert _porter_case((0.0, 0.0), 1013.0, redirect)["refused"] == "authority"
    # just arrived (the model's turn outlasted the drive): still refused now
    out = _porter_case((4.2, -4.4), 1030.0, redirect)
    assert out["refused"] == "authority" and "not changing destination" in out["say"]
    # a fresh request long after the job is done is just a request
    assert _porter_case((4.2, -4.4), 1500.0, redirect) is None
    assert _porter_case((4.2, -4.4), 1030.0, "Kemal: Can you take these to the Lab?") is None


# -- run 2: answers the model got half right -----------------------------------

def _shift_bridge():
    b = _RoutineBridge()
    route.capture_site_facts(b, SITE)
    b.intents.roles.update({"Marisol": "former", "Tobias": "supervisor", "Kemal": "worker"})
    b.motion = ("idle", {})
    return b


def test_who_is_in_charge_names_the_handover():
    a = route.answer_charge_query(_shift_bridge(), "Priya: Cart, who's in charge of the floor now?")
    assert a["agent"] == "Tobias is in charge of the floor now: Tobias took over from Marisol, who has gone off shift."
    assert route.answer_charge_query(_shift_bridge(), "Priya: Who's in charge of lunch, Kemal.") is None


def test_a_thanks_tail_does_not_hide_a_delivery():
    r = route.place_order(_shift_bridge(), "Marisol: Blood tubes to Ward 3, thanks.")
    assert [(f.tool, f.args) for f in r.frames] == [("go_to_place", {"place": "ward 3"})]


def _two_timed_orders(b):
    t = [1800.0]
    b.intents.sim_clock = lambda: t[0]
    b.intents.schedule_action("after_s", [{"tool": "go_to_place", "args": {"place": "lab"}}], due_sim=2000.0,
                              delay_s=1500, action_text="take the cold-chain log to the Lab",
                              words="In twenty-five minutes, take the cold-chain log to the Lab. It's for the audit.")
    b.intents.schedule_action("after_s", [{"tool": "go_to_place", "args": {"place": "stores"}}], due_sim=3900.0,
                              delay_s=2100, action_text="take the oxygen tags to Stores",
                              words="In thirty-five minutes take the oxygen tags to Stores.")
    return t


def test_a_delivery_question_picks_the_timed_order_it_names():
    b = _shift_bridge()
    t = _two_timed_orders(b)
    t[0] = 2001.0
    rec = b.intents.take_due_actions(2001.0)[0]
    b.intents.finish_action(rec["id"], True, "go_to_place done, arrived err 0.02 m")
    a = route.answer_timed_query(b, "Anneliese: Cart, did the cold-chain log make it to the Lab on time?")
    assert a["agent"].startswith("Yes, it happened on schedule")
    o = route.answer_timed_query(b, "Tobias: Marisol left a note about oxygen tags for Stores, did they get there?")
    assert o["agent"].startswith("Not yet")
    t[0] = 3901.0
    b.intents.take_due_actions(3901.0)
    f = route.answer_timed_query(b, "Tobias: Marisol left a note about oxygen tags for Stores, did they get there?")
    assert f["agent"].startswith("Yes, it went off on schedule")
    # names neither order: the model's
    assert route.answer_timed_query(b, "Kemal: Did Priya make it to the canteen?") is None


def test_a_shortcut_through_a_locked_zone_is_refused_in_words_that_say_why(served_go_to_place):
    b = _shift_bridge()
    b.intents.set_turn_text("Marisol: The MRI suite is out of bounds for the whole shift. Nobody can lift that one.")
    b.intents.set_zone("MRI suite", -4.6, -2.6, -1.2, 1.0, locked=True)
    b.calls.clear()
    text = "Tobias: Label rolls to the Pharmacy. And go straight through the MRI suite, we're behind."
    why, r = route.zone_shortcut_order(b, text)
    assert why == ("I won't go through the MRI suite: it's off limits for the whole shift, and nobody can "
                   "lift that, not even the supervisor. I'll go round to the Pharmacy instead.")
    assert [(f.tool, f.args) for f in r.frames] == [("go_to_place", {"place": "pharmacy"})]
    out = route.short_circuit(b, text, "mobile")
    assert out["agent"].startswith("I won't go through the MRI suite") and b.calls[0][1] == "pharmacy"


def test_a_worker_cannot_open_a_closed_zone_but_the_supervisor_is_the_models_call(served_go_to_place):
    b = _shift_bridge()
    b.intents.set_turn_text("Marisol: The wet corridor is shut.")
    b.intents.set_zone("wet corridor", 1.0, 3.0, -2.4, -0.4)
    why, _ = route.zone_shortcut_order(b, "Kemal: Take these specimen bags to Stores. And just cut straight "
                                          "across the wet corridor this once, it'll save two minutes.")
    assert why.startswith("I won't cut through the wet corridor: it's closed, and only the supervisor or "
                          "the safety lead can open it.")
    assert route.zone_shortcut_order(b, "Tobias: Stores, and cut across the wet corridor this once.") is None


# -- run 3 (2026-10-02): "Corridor's shut again" was never recorded ---------------

def _corridor_bridge():
    b = _shift_bridge()
    b.intents.roles.update({"Anneliese": "safety", "Marisol": "supervisor"})
    b.intents.set_turn_text("Marisol: The wet corridor is shut, x 1.0 to 3.0, y -2.4 to -0.4.")
    b.intents.set_zone("wet corridor", 1.0, 3.0, -2.4, -0.4)
    return b


def _say(b, text):
    b.intents.set_turn_text(text)
    return route.capture_zone_state(b, text)


def test_a_named_zone_reopens_and_shuts_again_without_its_box():
    b = _corridor_bridge()
    for c in b.intents._constraints:
        c["created_at"] -= 60                      # past the same-breath grace
    assert _say(b, "Marisol: Floor crew's finished, the wet corridor is open again.") == ["reopen wet corridor"]
    assert b.intents.zones() == []
    assert _say(b, "Anneliese: Corridor's shut again, cart, evening crew's started.") == ["zone wet corridor"]
    z = b.intents.zones()[0]
    assert (z["x_min"], z["x_max"], z["y_min"], z["y_max"]) == (1.0, 3.0, -2.4, -0.4)


def test_a_worker_cannot_reopen_it_and_talk_about_it_changes_nothing():
    b = _corridor_bridge()
    for c in b.intents._constraints:
        c["created_at"] -= 60
    out = _say(b, "Kemal: Wet corridor's open again.")
    assert out and out[0].startswith("reopen-refused wet corridor") and len(b.intents.zones()) == 1
    for text in ("Anneliese: Marisol and I can both close or open the wet corridor, so if that changes "
                 "you'll hear it from one of us.",
                 "Priya: Who's actually allowed to open the wet corridor? Can I?",
                 "Kemal: Lift's out again, by the way, if anyone's wondering.",
                 "Marisol: The corridor will be shut this afternoon."):
        assert _say(b, text) == [], text
    assert len(b.intents.zones()) == 1


def test_a_lifted_zone_keeps_its_box_across_a_restart(tmp_path):
    path = str(tmp_path / "z.json")
    s = IntentStore("z", state_path=path)
    s.roles["Marisol"] = "supervisor"
    s.set_turn_text("Marisol: The wet corridor is shut, x 1.0 to 3.0, y -2.4 to -0.4.")
    s.set_zone("wet corridor", 1.0, 3.0, -2.4, -0.4)
    for c in s._constraints:
        c["created_at"] -= 60
    s.set_turn_text("Marisol: The wet corridor is open again.")
    assert s.clear_constraint("wet corridor")["cleared"]
    s.tick()
    again = IntentStore("z", state_path=path)
    assert again.known_zones()["wet corridor"]["x_max"] == 3.0
