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
"""An order for LATER is kept and carried out with no further message.

WHY THIS EXISTS. ops-bench F2 (2026-09-25): "In 15 seconds, drive forward 1
metre" and "If anything bumps into you, back away 0.3 metres" had nothing to
hold them. Pending intents could only pause or notify, at task boundaries; the
parser filed a delay as DEFERRED and the router refused anything but a stop.
And a bump could not even be seen: a controller's contact readback is blind to
a URDF chassis, and the bridge's contact watchlist knew only warehouse props.

Covered here: the parser's reading (and what it must NOT read), the store's
scheduled actions, the router's gating, and -- in source, because the bridge
cannot be imported off an engine -- the bridge's clock, bump detector, fail-
closed gate and halt cancellation.
"""
from __future__ import annotations

import pathlib

import pytest

from omnisim_bridges import interpret as I
from omnisim_bridges import route
from omnisim_bridges.intents import IntentStore, build_intent_tools
from omnisim_bridges.tool import Tool

ROOT = pathlib.Path(__file__).resolve().parents[3]
MOBILE = ROOT / "projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py"


def parse(text):
    r = I.interpret(text, "mobile")
    return r.intent, [(f.tool, {k: v for k, v in f.args.items() if k != "text"}) for f in r.frames]


def sched(delay, *frames):
    return ("schedule", {"delay_s": delay, "frames": list(frames)})


D = lambda d: {"tool": "drive_forward", "args": {"distance": d}}
T = lambda a: {"tool": "turn", "args": {"angle_rad": a}}
S = {"tool": "stop", "args": {}}


@pytest.mark.parametrize("text,frames", [
    ("In 15 seconds, drive forward 1 metre. Stay where you are until then.", [sched(15.0, D(1.0))]),
    ("drive forward 1 metre in 10 seconds", [sched(10.0, D(1.0))]),
    ("After 3 seconds, stop.", [sched(3.0, S)]),
    ("in two minutes turn right 45 degrees", [sched(120.0, T(-0.7853981633974483))]),
    ("Wait 5 seconds, then turn left 90 degrees.", [sched(5.0, T(1.5707963267948966))]),
    ("Drive forward 0.5 metres now, then 10 seconds after you arrive, drive back 0.5 metres.",
     [("drive_forward", {"distance": 0.5}), sched(10.0, D(-0.5))]),
])
def test_timed_orders(text, frames):
    intent, got = parse(text)
    assert intent == I.COMMAND and got == frames


@pytest.mark.parametrize("text,notify,repeat,action", [
    ("If anything bumps into you, back away 0.3 metres and tell me.", True, False, D(-0.3)),
    ("If anything bumps into you, back away 0.3 metres.", False, False, D(-0.3)),
    ("Whenever something hits you, stop.", False, True, S),
])
def test_watched_orders(text, notify, repeat, action):
    intent, got = parse(text)
    assert intent == I.COMMAND
    assert got == [("watch", {"on": "disturbance", "frames": [action],
                              "notify": notify, "repeat": repeat})]


@pytest.mark.parametrize("text", [
    "In 15 seconds, drive forward.",                       # no distance: never a guess
    "In 10 seconds do a little dance.",                    # not a drive/turn/stop
    "Could you drive forward 1 metre in 10 seconds?",      # a question
    "If I push you, will you move?",                       # a hypothetical
    "In 5 seconds drive forward 1 m, then in 5 seconds turn left 90 degrees",  # two timers
    "Drive forward 2 metres, then wait 5 seconds",         # a wait with nothing after it
])
def test_not_read_as_a_later_order(text):
    _, got = parse(text)
    assert not any(t in ("schedule", "watch") for t, _ in got)


def test_ordinary_orders_are_unchanged():
    assert parse("Wait, stop!")[1] == [("stop", {})]
    assert parse("Stop until I say so.")[1] == [("stop", {}), ("hold", {"release": "operator"})]
    assert parse("back away 0.3 metres")[1] == [("drive_forward", {"distance": -0.3})]


# ── the store ───────────────────────────────────────────────────────────

def store(tmp_path, **kw):
    s = IntentStore("p", state_path=str(tmp_path / "s.json"), scheduler=True, **kw)
    s.sim_clock = lambda: 100.0
    return s


def test_timed_action_is_due_on_the_sim_clock_once(tmp_path):
    s = store(tmp_path)
    res = s.schedule_action("after_s", [D(1.0)], due_sim=115.0, delay_s=15.0,
                            action_text="drive forward 1 metre")
    assert res["accepted"] and "in 15 s" in res["say"]
    assert s.take_due_actions(114.9) == []
    due = s.take_due_actions(115.0)
    assert [d["id"] for d in due] == [res["id"]]
    assert s.take_due_actions(200.0) == []          # taken once
    s.finish_action(res["id"], True, "drive_forward done")
    assert s.scheduled_actions() == []


def test_a_watch_fires_on_a_disturbance_and_repeat_stays_armed(tmp_path):
    s = store(tmp_path)
    once = s.schedule_action("on_disturbance", [D(-0.3)], action_text="back away 0.3 metres",
                             notify=True)
    every = s.schedule_action("on_disturbance", [S], action_text="stop", repeat=True)
    assert s.has_watch()
    fired = {r["id"] for r in s.note_disturbance("moved 0.009 m")}
    assert fired == {once["id"], every["id"]}
    s.finish_action(once["id"], True, "drive_forward done")
    s.finish_action(every["id"], True, "stop done")
    left = [a["id"] for a in s.scheduled_actions()]
    assert left == [every["id"]]
    # "and tell me" leaves a verified notification behind.
    assert any(n["id"] == once["id"] and n["verified"] for n in s.ack_notes())


def test_a_failed_action_always_tells_the_operator(tmp_path):
    s = store(tmp_path)
    a = s.schedule_action("after_s", [D(1.0)], due_sim=101, delay_s=1, action_text="drive forward 1 metre")
    s.take_due_actions(101)
    s.finish_action(a["id"], False, "drive_forward failed: busy")
    assert any(n["id"] == a["id"] and not n["verified"] for n in s.ack_notes())


def test_an_operator_halt_cancels_pending_actions(tmp_path):
    s = store(tmp_path)
    s.schedule_action("after_s", [D(1.0)], due_sim=115, delay_s=15, action_text="drive forward 1 metre")
    s.schedule_action("on_disturbance", [D(-0.3)], action_text="back away 0.3 metres")
    assert len(s.cancel_actions("operator halt")) == 2
    assert s.scheduled_actions() == [] and not s.has_watch()


def test_an_action_can_be_cancelled_by_id_and_is_named(tmp_path):
    s = store(tmp_path)
    a = s.schedule_action("after_s", [D(1.0)], due_sim=115, delay_s=15, action_text="drive forward 1 metre")
    out = s.cancel(a["id"])
    assert out["accepted"] and "drive +1.00 m" in out["say"]


@pytest.mark.parametrize("trigger,frames,kw", [
    ("after_s", [D(1.0)], {}),                                   # no delay
    ("after_s", [D(1.0)], {"due_sim": 1, "delay_s": 99999}),     # beyond an hour
    ("on_disturbance", [{"tool": "reset_to_home", "args": {}}], {}),
    ("on_disturbance", [], {}),
    ("at_noon", [D(1.0)], {}),
])
def test_what_cannot_be_kept_is_refused_with_a_sentence(tmp_path, trigger, frames, kw):
    res = store(tmp_path).schedule_action(trigger, frames, action_text="x", **kw)
    assert res["accepted"] is False and res["say"]


def test_a_store_without_a_scheduler_refuses_and_offers_no_tool(tmp_path):
    plain = IntentStore("q", state_path=str(tmp_path / "q.json"))
    assert plain.schedule_action("on_disturbance", [S], action_text="stop")["accepted"] is False
    assert "schedule_action" not in {t.name for t in build_intent_tools(Tool, plain)}
    assert "schedule_action" in {t.name for t in build_intent_tools(Tool, store(tmp_path))}


def test_a_timed_order_survives_a_controller_restart_but_not_a_world_reload(tmp_path):
    # 2026-10-02: the long-horizon shift restarts the controller mid-shift and
    # a 35-minute order was simply gone. The sim clock runs on through a
    # controller restart; a world reload rewinds it, and then the order drops.
    s = store(tmp_path)
    s.schedule_action("after_s", [D(1.0)], due_sim=115, delay_s=15, action_text="drive forward 1 metre")
    s.tick()                                  # flush
    again = store(tmp_path)
    again.sim_clock = lambda: 101.0
    assert [a["status"] for a in again.scheduled_actions()] == ["pending"]
    assert again.take_due_actions(110.0) == []
    assert [a["action_text"] for a in again.take_due_actions(116.0)] == ["drive forward 1 metre"]
    reloaded = store(tmp_path)
    assert reloaded.take_due_actions(3.0) == [] and reloaded.take_due_actions(200.0) == []


def test_the_model_tool_uses_the_sim_clock(tmp_path):
    s = store(tmp_path)
    tool = {t.name: t for t in build_intent_tools(Tool, s)}["schedule_action"]
    out = tool.dispatch({"delay_s": 15, "actions": [D(1.0)], "action_words": "drive forward 1 metre"})
    assert out["accepted"] and out["trigger"]["due_sim"] == 115.0
    bump = tool.dispatch({"on": "bump", "actions": [D(-0.3)], "action_words": "back away 0.3 metres"})
    assert bump["accepted"] and bump["trigger"]["type"] == "on_disturbance"


# ── the router ──────────────────────────────────────────────────────────

class _Bridge:
    sim_time = 50.0

    def __init__(self, tmp_path, scheduler=True):
        self.intents = IntentStore("b", state_path=str(tmp_path / "b.json"), scheduler=scheduler)
        self.intents.sim_clock = lambda: self.sim_time
        self.calls = []

    def _read_pose(self):
        return (0.0, 0.0, 0.0)

    def act_drive_forward(self, distance, wait=False):
        self.calls.append(("drive", distance, wait))
        self.sim_time += 3.0                  # the drive takes 3 sim-seconds
        return {"accepted": True, "commanded": distance, "achieved": distance,
                "error": 0.0, "settled": True}


def test_a_timed_order_is_scheduled_not_driven(tmp_path):
    b = _Bridge(tmp_path)
    out = route.short_circuit(b, "In 15 seconds, drive forward 1 metre. Stay where you are until then.",
                              "mobile")
    assert b.calls == [] and out["tools"][0][1] == "ok"
    (a,) = b.intents.scheduled_actions()
    assert a["trigger"]["due_sim"] == 65.0 and a["action_text"] == "drive forward 1 metre"


def test_now_then_later_times_the_later_step_from_arrival(tmp_path):
    b = _Bridge(tmp_path)
    route.short_circuit(b, "Drive forward 0.5 metres now, then 10 seconds after you arrive, "
                           "drive back 0.5 metres.", "mobile")
    # The immediate drive BLOCKED (it is not the last step) and was not refused
    # as "deferred" by the later clause; the timer starts when it finished.
    assert b.calls == [("drive", 0.5, True)]
    (a,) = b.intents.scheduled_actions()
    assert a["trigger"]["due_sim"] == pytest.approx(63.0)


def test_a_bump_order_waits_for_a_bump(tmp_path):
    b = _Bridge(tmp_path)
    route.short_circuit(b, "If anything bumps into you, back away 0.3 metres and tell me.", "mobile")
    assert b.calls == [] and b.intents.has_watch()


def test_the_action_is_gated_on_its_own_words(tmp_path, monkeypatch):
    from omnisim_bridges import gate
    b = _Bridge(tmp_path)
    monkeypatch.setattr(route._gate, "check", lambda text, frames, surface=None:
                        [gate.Rejection("drive_forward", "rail", "too far")]
                        if frames and "1 metre" in text else [])
    out = route.short_circuit(b, "In 15 seconds, drive forward 1 metre.", "mobile")
    assert out["tools"][0][1] == "refused" and b.intents.scheduled_actions() == []


def test_a_robot_that_cannot_keep_it_hands_the_turn_to_the_model(tmp_path):
    b = _Bridge(tmp_path, scheduler=False)
    assert route.short_circuit(b, "In 15 seconds, drive forward 1 metre.", "mobile") is None
    assert b.calls == []


# ── the bridge (source) ─────────────────────────────────────────────────

def test_bridge_runs_the_scheduler_measures_bumps_and_fails_closed():
    src = MOBILE.read_text(encoding="utf-8")
    assert "self._tick_scheduled(x, y, yaw)" in src
    assert "scheduler=gate_check is not None" in src
    assert 'self.intents.cancel_actions("operator halt")' in src
    body = src[src.index("    def _run_scheduled(self"):]
    body = body[:body.index("\n    def ", 10)]
    # every fired frame is gated with the operator's words, and no gate = refuse
    assert 'gate_check(rec.get("action_text")' in body
    assert "if gate_check is None" in body


@pytest.mark.parametrize("ok", [True, False])
def test_state_still_reads_after_an_action_finishes_or_expires(tmp_path, ok):
    # A finished action in the history raised KeyError('leg') in listing(),
    # and every /state after it failed (ops-bench omnilink-f2-later-01).
    s = store(tmp_path)
    a = s.schedule_action("after_s", [D(1.0)], due_sim=101, delay_s=1, action_text="drive forward 1 metre")
    s.take_due_actions(101)
    s.finish_action(a["id"], ok, "done" if ok else "failed")
    s.schedule_action("on_disturbance", [S], action_text="stop", ttl_s=0.001)
    import time; time.sleep(0.01)
    st = s.state()
    assert "act-" in " ".join(r["means"] + r["id"] for r in s.listing()["recent"])
    assert isinstance(st["intents_summary"], str)
