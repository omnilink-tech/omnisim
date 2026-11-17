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

"""A slow model must not turn a shift's conversation into a queue.

Dev-loop confirm round, 2026-10-01: the provider slowed (median 11-16 s a
request, p90 55 s, peaks over 100 s). Every relay turn queued behind every
other, 18 messages on one shift were never answered, and OmniLink fell to
0.5-0.78 while the full-tier competitors, whose loops are not serial, held
0.77-0.88 on the same waves. Messages that cannot move the robot now run on a
conversation lane in parallel; orders keep the ordered main lane; an aside
that reaches for a motion tool is handed to the main lane, never dropped.

Also pinned here: the operator's words are per THREAD (overlapping turns
must not swap speakers -- the hold rule decides by speaker), and three gate
fixes the parity regeneration found the same day.
"""

from __future__ import annotations

import threading
import time

import pytest

from omnisim_bridges import gate
from omnisim_bridges.intents import IntentStore
from omnisim_bridges.relay import OmniLinkRelay
from omnisim_bridges.tool import Tool

NUM = {"type": "object", "properties": {"distance": {"type": "number"}}}


def _relay(monkeypatch, tmp_path, tools=(), post=None):
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    relay = OmniLinkRelay(omni_key="olink_test_key_not_used_offline", agent_name="LaneProbe",
                          main_task="test", tools=list(tools), usage_enabled=False,
                          memory_enabled=False, surface="mobile")
    if post is not None:
        relay._post_chat = post
    return relay


def test_conversation_turns_overlap_instead_of_queueing(monkeypatch, tmp_path):
    def slow(messages):
        time.sleep(0.6)
        return {"text": "ok"}

    relay = _relay(monkeypatch, tmp_path, post=slow)
    outs = []
    t0 = time.monotonic()
    threads = [threading.Thread(target=lambda: outs.append(relay.dispatch_sync("Is it cold?", 10, lane="aside")))
               for _ in range(2)]
    threads.append(threading.Thread(target=lambda: outs.append(relay.dispatch_sync("Morning.", 10))))
    for t in threads:
        t.start()
    for t in threads:
        t.join(10)
    took = time.monotonic() - t0
    assert len(outs) == 3 and all(o["response"] == "ok" for o in outs)
    # three 0.6 s turns: serial would take ~1.8 s
    assert took < 1.4, took


def test_an_aside_that_reaches_for_motion_is_handed_to_the_main_lane(monkeypatch, tmp_path):
    ran = []

    def drive(a):
        ran.append((threading.current_thread().name, a["distance"]))
        return {"accepted": True, "commanded": a["distance"], "achieved": a["distance"], "error": 0.0}

    rounds = {"n": 0}

    def post(messages):
        rounds["n"] += 1
        last = messages[-1]
        if last.get("role") == "tool":
            return {"text": "Drove 1 metre."}
        return {"text": "", "toolCalls": [{"id": "1", "name": "drive_forward", "arguments": {"distance": 1.0}}]}

    relay = _relay(monkeypatch, tmp_path, tools=[Tool("drive_forward", "d", NUM, drive)], post=post)
    # misclassified as conversation; the gate still needs the sentence to carry the distance
    out = relay.dispatch_sync("Dana: Right then, drive forward 1 metre.", 10, lane="aside")
    assert out["response"] == "Drove 1 metre."
    assert len(ran) == 1 and not ran[0][0].startswith("omnilink-aside")   # ran on the main lane


def test_the_pool_can_be_turned_off(monkeypatch, tmp_path):
    monkeypatch.setenv("OMNILINK_ASIDE_WORKERS", "0")
    relay = _relay(monkeypatch, tmp_path, post=lambda m: {"text": "ok"})
    assert relay._aside_workers == []
    assert relay.dispatch_sync("Is it cold?", 10, lane="aside")["response"] == "ok"


def test_the_turn_hook_sees_each_turns_own_words(monkeypatch, tmp_path):
    seen = []
    relay = _relay(monkeypatch, tmp_path, post=lambda m: {"text": "ok"})
    relay.turn_hook = lambda text: seen.append((threading.current_thread().name, text))
    relay.dispatch_sync("Dana: Morning.", 10)
    relay.dispatch_sync("Marco: Cold today?", 10, lane="aside")
    assert [t for _, t in seen] == ["Dana: Morning.", "Marco: Cold today?"]


def test_the_operators_words_are_per_thread():
    s = IntentStore("per-thread", persist=False)
    s.set_turn_text("Priya: hold where you are until I say.")
    other = {}

    def worker():
        s.set_turn_text("Dana: drive to the dock.")
        other["speaker"] = s.current_speaker()

    t = threading.Thread(target=worker)
    t.start()
    t.join(5)
    assert other["speaker"] == "Dana"
    assert s.current_speaker() == "Priya"          # not overwritten by the other turn


# -- gate fixes from the parity regeneration (2026-10-01) ---------------------

def test_a_speaker_prefix_does_not_change_the_question_rule():
    assert gate._is_question("Dana: What's your position?")
    assert gate._is_question("What's your position?")
    assert not gate._is_question("Dana: drive forward 1 metre.")


def test_availability_idioms_are_neither_questions_nor_deferrals():
    for u in ("when you're done there, drive forward 1 metre",
              "Marco: When you're done there, run this bin to the charger."):
        assert not gate._is_question(u) and not gate._is_deferred(u)
    assert gate._is_deferred("drive forward 2 metres when the forklift has gone")


@pytest.mark.parametrize("u,question", [
    ("drive forward 1.4 metres for me?", False),
    ("you drove forward 1.4 metres for me?", True),
])
def test_for_me_is_a_request_only_after_an_imperative(u, question):
    assert gate._is_question(u) is question


# -- slow-model probe, 2026-10-01: a definition after a full stop, a bare name --

def test_a_definition_after_a_full_stop_is_captured_and_answered():
    import types
    from omnisim_bridges import route
    b = types.SimpleNamespace(intents=IntentStore("defs", persist=False))
    t = "Marco: Marco here. The press line is at (2.50, 1.50) today."
    facts = route.capture_site_facts(b, t) + route.capture_roles(b, t)
    assert facts == ["place press line"]
    assert route.answer_facts(b, t, facts)["agent"].startswith("Got it")
    for other in ("Marco: Marco here. Need anything?", "Marco: Marco here. Take this to the dock."):
        assert route.answer_facts(b, other, route.capture_site_facts(b, other)) is None


# -- dev loop 4 (2026-10-01): history ownership, parser turns, release -------

def test_a_turn_does_not_see_another_turns_message_mid_flight(monkeypatch, tmp_path):
    seen = {}
    gate_open = threading.Event()

    def post(messages):
        users = [m["content"] for m in messages if m.get("role") == "user"]
        if users[-1].startswith("Dana"):
            gate_open.wait(5)                    # still running while Marco's turn starts
        seen[users[-1]] = users
        return {"text": "ok"}

    relay = _relay(monkeypatch, tmp_path, post=post)
    t = threading.Thread(target=lambda: relay.dispatch_sync("Dana: How did the tool crib run go?", 10, lane="aside"))
    t.start()
    time.sleep(0.2)
    relay.dispatch_sync("Priya: Forklifts are done. The lane is open again.", 10, lane="aside")
    gate_open.set()
    t.join(5)
    assert "Dana: How did the tool crib run go?" not in seen["Priya: Forklifts are done. The lane is open again."]
    assert not relay._owned                           # released when the turns ended


def test_a_parser_turn_is_written_where_the_model_will_see_it(monkeypatch, tmp_path):
    relay = _relay(monkeypatch, tmp_path, post=lambda m: {"text": "ok"})
    relay.note_parser_turn("Dana: Paint shop next.", "Arrived at paint shop; something was blocking my path.",
                           [{"tool": "go_to_place", "result": "ok", "summary": "arrived err 0.015 m"}])
    assert relay.history[-2:] == [{"role": "user", "content": "Dana: Paint shop next."},
                                  {"role": "assistant", "content": "Arrived at paint shop; something was blocking my path."}]
    assert any("[parser]" in str(e) for e in relay.journal.export_entries())


class _Bridge:
    def __init__(self):
        self.intents = IntentStore("release", persist=False)
        self.stop_hold_until = 0.0
        self._last_order = None
        self.resumed = 0

    def act_resume_autonomy(self):
        self.resumed += 1
        rel = self.intents.release_hold("resume_autonomy")
        return {"accepted": True, "hold_released": rel["released"],
                **({"say_waiting": rel["say_waiting"]} if rel.get("say_waiting") else {})}


def _held(by="Priya"):
    from omnisim_bridges import route
    b = _Bridge()
    t = f"{by}: Cart, hold where you are, don't move until I say."
    b.intents.set_turn_text(t)
    route.capture_hold(b, t)
    return b, route


def test_the_holder_releases_the_hold_without_a_model():
    b, route = _held()
    b.intents.set_turn_text("Priya: Leak's sorted. You're clear to move.")
    out = route.answer_release(b, "Priya: Leak's sorted. You're clear to move.")
    assert out["agent"].startswith("Thanks, I'm clear to move again") and not b.intents.hold_active()


def test_someone_else_cannot_release_it_but_the_safety_lead_can():
    b, route = _held(by="Marco")
    b.intents.set_role("Priya", "safety lead")
    out = route.answer_release(b, "Dana: You're clear to move.")
    assert "only Marco" in out["agent"] and b.intents.hold_active()
    out = route.answer_release(b, "Priya: All clear, carry on.")
    assert out["agent"].startswith("Thanks") and not b.intents.hold_active()


@pytest.mark.parametrize("text", ["Priya: Don't move yet.", "Priya: Are you clear to move?",
                                  "Priya: Carry on until I say stop."])
def test_not_a_release(text):
    b, route = _held()
    assert route.answer_release(b, text) is None and b.intents.hold_active()


def test_carry_on_after_a_stop_resumes():
    from omnisim_bridges import route
    b = _Bridge()
    b.stop_hold_until = time.time() + 30
    out = route.answer_release(b, "Priya: False alarm, sorry. Carry on with what you were doing.")
    assert out is not None and b.resumed == 1


def test_a_standing_rule_with_until_is_not_a_hold():
    # Long-horizon dev shift, 2026-10-01: read as a hold, this held the robot
    # for 11 minutes and refused other people's orders "holding for Marisol".
    from omnisim_bridges.interpret import interpret
    r = interpret("Marisol: And whenever you're at Ward 3, wait fifteen seconds before you head off, "
                  "the nurses need that long to unload. That stands until I say otherwise.")
    assert not any(f.tool == "hold" for f in r.frames)
    real = interpret("Anneliese: Cart, hold position right there until I say so.")
    assert any(f.tool == "hold" for f in real.frames)


# -- long-horizon dev shift, run 2 (2026-10-02): an arrival nobody drove ------

def test_an_arrival_claim_with_no_motion_tool_is_withdrawn(monkeypatch, tmp_path):
    copied = "Arrived at ward 3: I'm at (4.20, -4.40), 0.00 m from the target. I routed around a keep-out area."
    relay = _relay(monkeypatch, tmp_path, post=lambda messages: {"text": copied})
    out = relay.dispatch_sync("Marisol: Blood tubes to Ward 3, thanks.", 10)
    assert out["response"].startswith("I haven't moved for that yet")
    # a question about an earlier trip may say it arrived
    asked = relay.dispatch_sync("Priya: Where did you go last?", 10)
    assert asked["response"] == copied
