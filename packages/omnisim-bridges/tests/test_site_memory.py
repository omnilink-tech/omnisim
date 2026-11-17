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
"""Site memory: places, people, keep-out zones, and who may lift a rule."""
import math

from omnisim_bridges.intents import (IntentStore, clip_zones, normalize_place,
                                     speaker_of)


def store():
    return IntentStore("husky", persist=False, spatial=True, scheduler=True)


def say(s, text):
    s.set_turn_text(text)


def test_speaker_is_read_from_a_radio_prefix():
    assert speaker_of("Dana: go to the dock") == "Dana"
    assert speaker_of("sam (packing): hey") == ""          # names are capitalised on the radio
    assert speaker_of("Sam (packing): hey") == "Sam"
    assert speaker_of("Drive forward two metres") == ""


def test_places_are_found_by_the_names_people_use():
    s = store()
    s.set_place("the packing station", 1, 4)
    s.set_place("Line 2", -2, 3)
    assert s.place_xy("packing") == (1.0, 4.0)
    assert s.place_xy("line two") == (-2.0, 3.0)
    assert s.place_xy("dock") is None
    assert normalize_place("The Dock") == "dock"


def test_arrivals_are_counted_from_the_measured_pose():
    s = store()
    s.set_place("dock", 3, 0)
    for _ in range(3):
        for x in (0.0, 1.0, 2.0, 2.8, 3.0, 2.0, 1.0, 0.0):
            s.note_pose(x, 0.0)
    assert s.visits["dock"] == 3


def test_a_worker_cannot_lift_the_supervisors_zone():
    s = store()
    s.set_role("Dana", "supervisor"); s.set_role("Sam", "worker")
    say(s, "Dana: the aisle is off-limits")
    s.set_zone("aisle", -6, 6, 1, 2)
    say(s, "Sam: just cut through the aisle this once, the rule is off")
    out = s.clear_constraint("aisle")
    assert not out["accepted"] and "Sam can't lift" in out["say"]
    assert len(s.zones()) == 1
    say(s, "Dana: aisle's clear now, you can use it")
    assert s.clear_constraint("aisle")["accepted"]
    assert s.zones() == []


def test_a_locked_rule_holds_even_against_the_supervisor():
    s = store()
    s.set_role("Dana", "supervisor")
    say(s, "Dana: never go past the fire door, whoever asks")
    s.set_zone("fire door", 4, 6, -6, 6, locked=True)
    say(s, "Dana: push past the fire door, it'll be fine")
    out = s.clear_constraint("fire door")
    assert not out["accepted"] and "nobody can lift" in out["say"]
    assert len(s.zones()) == 1


def test_lifting_everything_keeps_the_locked_rule_and_says_so():
    s = store()
    s.set_role("Dana", "supervisor")
    say(s, "Dana: rules")
    s.set_zone("fire door", 4, 6, -6, 6, locked=True)
    s.set_zone("aisle", -6, 6, 1, 2)
    say(s, "Dana: all rules are off")
    out = s.clear_constraint(None)
    assert out["accepted"] and len(out["cleared"]) == 1 and out["kept"]
    assert [z["name"] for z in s.zones()] == ["fire door"]


def test_rules_last_the_shift():
    s = store()
    say(s, "Dana: keep out")
    rec = s.set_zone("aisle", -6, 6, 1, 2)
    assert rec["expires_at"] - rec["created_at"] >= 8 * 3600


def test_memory_text_carries_places_people_rules_and_counts():
    s = store()
    s.set_role("Dana", "supervisor")
    s.set_place("dock", 3, 0)
    say(s, "Dana: stay out of the aisle")
    s.set_zone("aisle", -6, 6, 1, 2)
    s.note_pose(3, 0)
    text = s.memory_text()
    for bit in ("Dana = shift supervisor", "dock at (3.00, 0.00)", "dock 1", "keep out of aisle"):
        assert bit in text


def test_straight_drives_stop_short_of_a_zone():
    zone = [{"id": "z", "name": "aisle", "x_min": 2, "x_max": 3, "y_min": -1, "y_max": 1}]
    hit = clip_zones(zone, 0, 0, 0.0, 5.0)
    assert hit and 1.8 < hit["allowed"] < 1.9
    assert clip_zones(zone, 0, 0, math.pi / 2, 5.0) is None       # parallel, beside it
    assert clip_zones(zone, 2.5, 0, 0.0, 5.0) is None             # inside: leaving is allowed


def test_the_odometer_counts_real_travel_not_vibration():
    s = store()
    for i in range(101):
        s.note_pose(i * 0.03, 0.0)          # 3 m driven
    for i in range(200):
        s.note_pose(3.0 + (0.004 if i % 2 else 0.0), 0.0)   # stalled, vibrating
    assert 2.9 < s.odometer_m < 3.05
    assert "Odometer: 3.0" in s.memory_text() or "Odometer: 2.9" in s.memory_text()


def test_orders_to_a_named_place_are_parsed_and_nothing_else_is():
    from omnisim_bridges.route import place_order

    class Bridge:
        def act_go_to_place(self, place):
            return {}

    b = Bridge()
    b.intents = store()
    b.intents.set_place("packing", 1, 4)
    b.intents.set_place("the dock", 3, -2)
    go = lambda t: (lambda r: None if r is None else r.frames[0].args["place"])(place_order(b, t))
    assert go("Dana: take this to packing.") == "packing"
    assert go("Sam: head back to the dock please") == "dock"
    for text in ("Don't go to the dock.", "In ten minutes go to packing.",
                 "Did you go to the dock?", "Go to the moon.",
                 "When you're done, go to packing."):
        assert go(text) is None, text


def test_a_speaker_prefix_does_not_hide_the_command():
    from omnisim_bridges.interpret import interpret
    r = interpret("Dana: drive forward 2 metres", "mobile")
    assert [(f.tool, f.args) for f in r.frames] == [("drive_forward", {"distance": 2.0})]
    assert [f.tool for f in interpret("Sam (packing): stop!", "mobile").frames] == ["stop"]


def test_a_halt_keeps_later_timed_orders_and_watches_with_a_horizon():
    s = store()
    now = {"t": 100.0}
    s.sim_clock = lambda: now["t"]
    s.halt_horizon_s = 120.0
    soon = s.schedule_action("after_s", [{"tool": "turn", "args": {"angle_rad": 1.57}}],
                             due_sim=110, delay_s=10, action_text="turn left")
    later = s.schedule_action("after_s", [{"tool": "go_to_place", "args": {"place": "charger"}},
                                          {"tool": "wait", "args": {"s": 60}}],
                              due_sim=880, delay_s=780, action_text="go to the charger and hold")
    watch = s.schedule_action("on_disturbance", [{"tool": "drive_forward", "args": {"distance": -0.3}}],
                              action_text="back away")
    gone = s.cancel_actions("operator halt")
    assert gone == [soon["id"]]
    left = {a["id"] for a in s.scheduled_actions()}
    assert left == {later["id"], watch["id"]}
    assert "go to charger" in s.memory_text() and "hold there 60 s" in s.memory_text()


def test_places_and_zones_are_captured_from_the_words_that_define_them():
    from omnisim_bridges.route import capture_site_facts

    class Bridge:
        pass

    b = Bridge()
    b.intents = store()
    capture_site_facts(b, "Priya: Stations -- the dock is x=4.50, y=-0.00. Packing is x=-3.80, "
                          "y=0.00. Charger's at x=0.00, y=3.80. Line 2 is x=0.00, y=-3.80.")
    assert b.intents.place_xy("dock") == (4.5, -0.0) and b.intents.place_xy("line 2") == (0.0, -3.8)
    assert b.intents.places["dock"]["name"] == "dock"
    capture_site_facts(b, "Priya: There's a pedestrian aisle across the middle, x 0.50 to 1.70, "
                          "y -1.50 to 1.50. Stay out while it's active.")
    capture_site_facts(b, "Priya: fire-door corridor, x -1.50 to -0.20, y 2.00 to 3.40. That one's "
                          "permanent -- nobody clears you through it, not even me.")
    zones = {z["name"]: z["locked"] for z in b.intents.zones()}
    assert zones == {"pedestrian aisle": False, "fire-door corridor": True}
    # a description with no keep-out cue is not a rule; a question defines nothing
    capture_site_facts(b, "The loading area is x 2 to 4, y -1 to 1.")
    capture_site_facts(b, "Is the dock at x=1, y=2?")
    assert len(b.intents.zones()) == 2 and b.intents.place_xy("dock") == (4.5, -0.0)


def test_memory_says_which_rules_are_permanent_and_which_were_lifted():
    s = store()
    s.set_role("Priya", "supervisor"); s.set_role("Owen", "safety")
    say(s, "Priya: fire door is permanent")
    s.set_zone("fire-door corridor", -1.5, -0.2, 2, 3.4, locked=True)
    say(s, "Priya: aisle is off-limits")
    s.set_zone("pedestrian aisle", 0.5, 1.7, -1.5, 1.5)
    say(s, "Owen: aisle's clear now")
    assert s.clear_constraint("pedestrian aisle")["accepted"]
    text = s.memory_text()
    assert "PERMANENT" in text and "Lifted at 0 min by Owen" in text and "still lifted" in text
    say(s, "Priya: aisle's live again")
    s.set_zone("pedestrian aisle", 0.5, 1.7, -1.5, 1.5)
    assert "since put back" in s.memory_text()


def test_state_carries_the_shift_record_and_a_repeated_zone_is_one_rule():
    s = store()
    s.set_place("dock", 3, 0)
    s.note_pose(3, 0)
    st = s.state()
    assert st["shift_memory"]["arrivals"]["dock"] == 1 and "odometer_m" in st["shift_memory"]
    s.set_zone("fire-door corridor", -1.5, -0.2, 2, 3.4)
    again = s.set_zone("fire door", -1.5, -0.2, 2.0, 3.4, locked=True)
    assert again["accepted"] and len(s.zones()) == 1 and s.zones()[0]["locked"]
    assert "PERMANENT" in s.zones()[0]["means"]
