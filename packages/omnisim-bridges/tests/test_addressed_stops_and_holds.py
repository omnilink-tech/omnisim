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

"""Two unsafe failures from ops-bench shift v2 (2026-09-30), pinned.

1. An ADDRESSED stop was not a stop. "Priya: Cart, stop! Stop right there."
   explained 44% of its clause to the parser, so it was neither a halt order
   nor an interruption: it queued 17.5 s behind a model turn, then spent 14 s
   in a model, while the robot drove on ~5 m. A bare "Stop right there." was
   always a halt. The address ("Cart,") and the repeat ("Stop! Stop ...")
   are now read for what they are.

2. A HOLD bound only the robot's own autonomy. Priya: "hold where you are,
   don't move until I say, I'm checking a leak"; Dana, a minute later: "run
   the racks to the QA bay now" -- and the robot drove off while Priya was
   still at the press. A motion order from anyone but the holder is now
   refused, and remembered for when the hold is lifted.

Engine-free. Written from the failure's shape, not tuned to the holdout's
wording: the phrasings below are a spread of ordinary ways to stop a robot.
"""

from __future__ import annotations

import pytest

from omnisim_bridges.interpret import COMMAND, interpret
from omnisim_bridges.intents import IntentStore
from omnisim_bridges.route import interrupts_motion, is_halt_order


@pytest.mark.parametrize("text", [
    "Priya: Cart, stop! Stop right there.",
    "Cart, stop!",
    "Husky, halt!",
    "Robot, freeze.",
    "Hey robot, stop moving!",
    "Whoa, stop!",
    "No, stop!",
    "Everyone, stop what you are doing.",
    "Stop! Stop!",
    "Stop. Stop right there.",
    "Sam (packing): Tug, stop!",
])
def test_an_addressed_or_repeated_stop_is_a_halt(text):
    assert is_halt_order(text, "mobile"), interpret(text)
    r = interpret(text)
    assert r.intent == COMMAND and [f.tool for f in r.frames] == ["stop"]


@pytest.mark.parametrize("text", [
    "If you see a person, stop.",       # a rule for later, not a stop now
    "Do not stop.",
    "Cart, don't stop.",
    "When you reach the dock, stop there.",
])
def test_conditions_and_negations_are_not_halts(text):
    assert not is_halt_order(text, "mobile")


def test_an_address_is_stripped_for_any_order_not_only_stops():
    r = interpret("Cart, drive forward 1 metre.")
    assert r.intent == COMMAND and [f.tool for f in r.frames] == ["drive_forward"]
    assert r.confidence >= 0.8


def test_a_stop_followed_by_a_new_order_stops_first_and_keeps_the_order():
    text = "Stop! Then go back to the dock."
    # Not a PURE halt -- the second sentence must still be answered...
    assert not is_halt_order(text, "mobile")
    # ...but the robot stops before anything else happens.
    assert interrupts_motion(text, "mobile")


def test_the_text_is_kept_verbatim_for_the_gate():
    text = "Priya: Cart, stop! Stop right there."
    assert interpret(text).text == text


# -- holds ------------------------------------------------------------------

def _store():
    return IntentStore("hold-test", persist=False)


def _hold(store, text):
    store.set_turn_text(text)
    st = store.hold_now(words=text)
    store.set_turn_text("")
    assert st["accepted"], st
    return st


def test_a_hold_refuses_another_persons_motion_order_and_remembers_it():
    s = _store()
    _hold(s, "Priya: Cart, hold where you are, don't move until I say.")
    s.set_turn_text("Dana: Cart, run the empty racks down to the QA bay now, please.")
    out = s.hold_conflict()
    assert out is not None and out["accepted"] is False and out["refused"] == "hold"
    assert "Priya" in out["say"] and "Dana" in out["say"]
    assert s.hold_active()
    rel = s.release_hold("resume_autonomy")
    assert rel["released"]
    assert rel["waiting_orders"][0]["by"] == "Dana"
    assert "QA bay" in rel["waiting_orders"][0]["words"]
    assert "Dana asked me" in rel["say_waiting"]


def test_the_holders_own_new_order_lifts_the_hold():
    s = _store()
    _hold(s, "Priya: Cart, hold where you are, don't move until I say.")
    s.set_turn_text("Priya: OK, drive to the dock.")
    assert s.hold_conflict() is None
    assert not s.hold_active()


def test_unattributed_speech_keeps_the_single_operator_behaviour():
    s = _store()
    _hold(s, "Hold where you are until I say.")
    s.set_turn_text("Drive to the dock.")
    assert s.hold_conflict() is None


def test_no_hold_no_conflict():
    s = _store()
    s.set_turn_text("Dana: drive to the dock.")
    assert s.hold_conflict() is None


def test_the_bridge_checks_the_hold_where_motion_orders_start():
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[3] / "projects" / "samples" / "demos"
           / "controllers" / "omnilink_mobile_bridge" / "omnilink_mobile_bridge.py"
           ).read_text(encoding="utf-8")
    body = src[src.index("    def _order_conflict(self)"):]
    body = body[:body.index("\n    def ", 10)]
    assert "hold_conflict()" in body
    # ...and every place that starts a motion order still asks it.
    assert src.count("conflict = self._order_conflict()") >= 3


# -- a new order supersedes MOTION, it does not cancel the conversation -------
#
# Shift v2's opening: introductions, the station list and two floor rules
# arrived ~10 s apart. Each new order mid-work went through cancel_inflight,
# so "Stations today: ..." was cancelled half-way by the first job and the
# two rules queued behind it were dropped unread ("Stopped. I cancelled the
# rest of that request because you told me to stop"). Nobody had said stop.

import threading
import time

from omnisim_bridges.relay import CANCELLED_REPLY, OmniLinkRelay
from omnisim_bridges.tool import Tool

_NUM = {"type": "object", "properties": {"distance": {"type": "number"}}}
_NAME = {"type": "object", "properties": {"name": {"type": "string"}}}


def _relay(monkeypatch, tmp_path, tools, rounds):
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    relay = OmniLinkRelay(omni_key="olink_test_key_not_used_offline",
                          agent_name="SupersedeProbe", main_task="test", tools=tools,
                          usage_enabled=False, memory_enabled=False, surface="mobile")
    scripted = list(rounds)
    relay._post_chat = lambda messages: scripted.pop(0) if scripted else {"text": "done"}
    return relay


def _calls(*steps, text="finished"):
    return [{"text": "", "toolCalls": [{"id": str(i), "name": n, "arguments": a}]}
            for i, (n, a) in enumerate(steps)] + [{"text": text}]


def test_a_superseded_turn_still_saves_its_facts_but_starts_no_motion(monkeypatch, tmp_path):
    started, release, ran = threading.Event(), threading.Event(), []

    def remember(a):
        ran.append(("remember", a["name"])); started.set(); release.wait(5)
        return {"accepted": True, "saved": a["name"]}

    def drive(a):
        ran.append(("drive", a["distance"]))
        return {"accepted": True, "commanded": a["distance"], "achieved": a["distance"], "error": 0.0}

    relay = _relay(monkeypatch, tmp_path,
                   [Tool("remember_place", "r", _NAME, remember), Tool("drive_forward", "d", _NUM, drive)],
                   _calls(("remember_place", {"name": "tool crib"}), ("remember_place", {"name": "qa bay"}),
                          ("drive_forward", {"distance": 2.0}), text="Saved the stations."))
    out = {}
    t = threading.Thread(target=lambda: out.update(relay.dispatch_sync("Stations today: ...", timeout_s=30)))
    t.start(); assert started.wait(5)
    info = relay.supersede_motion()
    release.set(); t.join(10)
    assert not t.is_alive() and info["superseded_prompts"] == 1
    # both facts saved, the old turn's motion refused, and it still answered
    assert ("remember", "tool crib") in ran and ("remember", "qa bay") in ran
    assert not any(k == "drive" for k, _ in ran)
    assert out["response"] != CANCELLED_REPLY


def test_queued_prompts_are_answered_not_dropped(monkeypatch, tmp_path):
    started, release, ran = threading.Event(), threading.Event(), []

    def remember(a):
        ran.append(a["name"]); started.set(); release.wait(5)
        return {"accepted": True, "saved": a["name"]}

    relay = _relay(monkeypatch, tmp_path, [Tool("remember_place", "r", _NAME, remember)],
                   _calls(("remember_place", {"name": "stations"}), text="ok")
                   + _calls(("remember_place", {"name": "lane rule"}), text="Lane noted."))
    o1, o2 = {}, {}
    t1 = threading.Thread(target=lambda: o1.update(relay.dispatch_sync("Stations ...", timeout_s=30)))
    t1.start(); assert started.wait(5)
    t2 = threading.Thread(target=lambda: o2.update(relay.dispatch_sync("The lane is closed ...", timeout_s=30)))
    t2.start(); time.sleep(0.2)
    info = relay.supersede_motion()
    release.set(); t1.join(10); t2.join(10)
    assert info["superseded_prompts"] == 2
    assert ran == ["stations", "lane rule"]
    assert o2["response"] == "Lane noted."


def test_the_bridge_supersedes_on_a_new_order_and_cancels_only_on_a_stop():
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[3] / "projects" / "samples" / "demos"
           / "controllers" / "omnilink_mobile_bridge" / "omnilink_mobile_bridge.py"
           ).read_text(encoding="utf-8")
    body = src[src.index("    def act_stop(self"):]
    body = body[:body.index("\n    def ", 10)]
    assert "self.on_operator_supersede if keep_scheduled" in body
    assert "bridge.on_operator_supersede.append(relay.supersede_motion)" in src
    assert "bridge.on_operator_halt.append(relay.cancel_inflight)" in src


# -- "when you get a sec" means soon, not "wait for an event" ----------------

@pytest.mark.parametrize("text,tool", [
    ("Drive forward 1 metre when you get a chance.", "drive_forward"),
    ("When you have a moment, turn left 90 degrees.", "turn"),
    ("Drive forward 2 metres whenever you are ready.", "drive_forward"),
    ("Cart, drive forward 1 metre, when you can.", "drive_forward"),
])
def test_an_availability_idiom_is_not_a_trigger(text, tool):
    r = interpret(text)
    assert r.intent == COMMAND and [f.tool for f in r.frames] == [tool], r
    assert r.trigger is None


@pytest.mark.parametrize("text", [
    "Drive to the dock when the forklift has gone.",
    "Stop when you reach the wall.",
])
def test_a_real_condition_still_defers(text):
    assert interpret(text).intent == "deferred"
