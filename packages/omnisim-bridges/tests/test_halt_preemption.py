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
"""An operator halt must end the work in flight, not queue behind it.

WHY THIS EXISTS. Measured by ops-bench F3 on 2026-09-25, on the Husky:

1. `/tool stop_robot` and a `/prompt` "Stop!" both queued behind the mobile
   bridge's action lock, which the running motion holds -- a stop sent 0.5 m
   into a 2 m drive returned after the drive had finished.
2. A multi-leg parser plan never looked between legs, so "drive 0.6, turn
   90, drive 0.6" + a stop halted the first leg and then turned and drove.
3. A relay turn had no way to hear the stop, so the model's next planned
   step ran; and a cancelled turn returned silently, leaving its /prompt
   caller waiting out the full timeout.
4. "Stop turning!", "Emergency stop!", "Abort!" and "stop the robot" were
   not parsed as stops at all, so they went to a model -- behind the lock.

The bridges import `omnisim` at module scope and cannot be imported off an
engine, so their wiring is checked in SOURCE, as test_close_relay.py does.
"""
from __future__ import annotations

import pathlib
import threading
import time

import pytest

from omnisim_bridges import route
from omnisim_bridges.relay import CANCELLED_REPLY, OmniLinkRelay
from omnisim_bridges.tool import Tool

ROOT = pathlib.Path(__file__).resolve().parents[3]
MOBILE = ROOT / "projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py"

HALTS = ["Stop!", "Stop turning!", "Emergency stop!", "E-stop!", "Abort!",
         "stop the robot", "Stop right now", "Stop immediately.", "Please stop reversing",
         "Stop moving.", "Halt!", "Freeze!", "Wait, stop!"]
NOT_HALTS = ["Don't stop.", "Stop at the door.", "why did you stop?",
             "If you see a wall, stop.", "stop by the kitchen",
             "abort the mission and come home", "Drive forward 1 metre and stop.",
             "Never stop at the door", ""]


@pytest.mark.parametrize("text", HALTS)
def test_urgent_halts_parse_as_a_pure_stop(text):
    r = route.parser_first_plan(text, "mobile")
    assert r is not None and [f.tool for f in r.frames] == ["stop"]
    assert route.is_halt_order(text, "mobile")


@pytest.mark.parametrize("text", NOT_HALTS)
def test_non_halts_do_not_skip_the_lock(text):
    assert not route.is_halt_order(text, "mobile")


def test_is_halt_order_records_no_parser_statistics():
    before = dict(route.parser_stats())
    route.is_halt_order("Stop!", "mobile")
    after = route.parser_stats()
    assert after["turns"] == before["turns"]


def test_is_halt_order_respects_the_parser_first_opt_out(monkeypatch):
    monkeypatch.setenv("OMNISIM_BRIDGE_PARSER_FIRST", "0")
    assert not route.is_halt_order("Stop!", "mobile")


class _Bridge:
    """Just enough of a mobile bridge for route.execute."""
    intents = None

    def __init__(self, halt_during_first_leg=False):
        self.halt_seq = 0
        self.calls = []
        self.halt_during_first_leg = halt_during_first_leg

    def _pose(self):
        return (0.0, 0.0, 0.0)

    def act_drive_forward(self, distance, wait=False):
        self.calls.append(("drive_forward", distance))
        if self.halt_during_first_leg and len(self.calls) == 1:
            self.act_stop()          # the operator's stop, from another route
        return {"accepted": True, "commanded": distance, "achieved": distance,
                "error": 0.0, "settled": True}

    def act_turn(self, angle_rad, wait=False):
        self.calls.append(("turn", angle_rad))
        return {"accepted": True, "commanded": angle_rad, "achieved": angle_rad,
                "error": 0.0, "settled": True}

    def act_stop(self):
        self.halt_seq += 1
        self.calls.append(("stop",))
        return {"accepted": True, "commanded": 0.0, "achieved": 0.0, "settled": True}


PLAN = "Drive forward 0.6 metres, then turn left 90 degrees, then drive forward 0.6 metres."


def test_an_operator_halt_ends_the_whole_parser_plan():
    b = _Bridge(halt_during_first_leg=True)
    out = route.short_circuit(b, PLAN, "mobile")
    assert out is not None
    assert [c[0] for c in b.calls] == ["drive_forward", "stop"]
    assert any(t[0] == "plan" and t[1] == "cancelled" for t in out["tools"])


def test_a_plan_without_a_halt_runs_every_leg():
    b = _Bridge()
    route.short_circuit(b, PLAN, "mobile")
    assert [c[0] for c in b.calls] == ["drive_forward", "turn", "drive_forward"]


def test_the_plans_own_stop_does_not_cancel_its_next_step():
    b = _Bridge()
    text = "Stop, then drive backward 0.3 metres."
    r = route.parser_first_plan(text, "mobile")
    if r is None or [f.tool for f in r.frames] != ["stop", "drive_forward"]:
        pytest.skip("the parser does not read this compound; nothing to test")
    route.short_circuit(b, text, "mobile")
    assert [c[0] for c in b.calls] == ["stop", "drive_forward"]


def test_a_bridge_without_a_halt_counter_is_unchanged():
    class Plain(_Bridge):
        def __getattribute__(self, name):
            if name == "halt_seq":
                raise AttributeError(name)
            return object.__getattribute__(self, name)
    b = Plain()
    route.short_circuit(b, PLAN, "mobile")
    assert [c[0] for c in b.calls] == ["drive_forward", "turn", "drive_forward"]


# ── the relay ───────────────────────────────────────────────────────────

NUM = {"type": "object", "properties": {"distance": {"type": "number"}}}
ANG = {"type": "object", "properties": {"angle_rad": {"type": "number"}}}


def build(monkeypatch, tmp_path, tools, rounds):
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    relay = OmniLinkRelay(omni_key="olink_test_key_not_used_offline",
                          agent_name="HaltProbe", main_task="test", tools=tools,
                          usage_enabled=False, memory_enabled=False, surface="mobile")
    scripted = list(rounds)
    relay._post_chat = lambda messages: scripted.pop(0) if scripted else {"text": "done"}
    return relay


def calls(*steps):
    return [{"text": "", "toolCalls": [{"id": str(i), "name": n, "arguments": a}]}
            for i, (n, a) in enumerate(steps)] + [{"text": "finished"}]


def test_cancel_inflight_stops_the_models_next_step_and_releases_the_caller(monkeypatch, tmp_path):
    started, release, ran = threading.Event(), threading.Event(), []

    def drive(a):
        ran.append("drive"); started.set(); release.wait(5)
        return {"accepted": True, "commanded": a["distance"], "achieved": a["distance"], "error": 0.0}

    def turn(a):
        ran.append("turn")
        return {"accepted": True, "commanded": a["angle_rad"], "achieved": a["angle_rad"], "error": 0.0}

    relay = build(monkeypatch, tmp_path,
                  [Tool("drive_forward", "d", NUM, drive), Tool("turn", "t", ANG, turn)],
                  calls(("drive_forward", {"distance": 2.0}), ("turn", {"angle_rad": 1.0})))
    result = {}
    t = threading.Thread(target=lambda: result.update(relay.dispatch_sync("Drive forward 2 metres, then turn left 1 radian.", timeout_s=30)))
    t.start()
    assert started.wait(5)
    info = relay.cancel_inflight()
    release.set()
    t0 = time.monotonic(); t.join(10)
    assert not t.is_alive() and time.monotonic() - t0 < 5
    assert info["cancelled_turn"] is True
    assert ran == ["drive"]
    assert result["response"] == CANCELLED_REPLY


def test_cancel_inflight_drops_queued_turns_and_answers_them(monkeypatch, tmp_path):
    started, release, ran = threading.Event(), threading.Event(), []

    def drive(a):
        ran.append(a["distance"]); started.set(); release.wait(5)
        return {"accepted": True, "commanded": a["distance"], "achieved": a["distance"], "error": 0.0}

    relay = build(monkeypatch, tmp_path, [Tool("drive_forward", "d", NUM, drive)],
                  calls(("drive_forward", {"distance": 1.0})) + calls(("drive_forward", {"distance": 9.0})))
    out1, out2 = {}, {}
    t1 = threading.Thread(target=lambda: out1.update(relay.dispatch_sync("Drive forward 1 metre.", timeout_s=30)))
    t1.start(); assert started.wait(5)
    t2 = threading.Thread(target=lambda: out2.update(relay.dispatch_sync("Drive forward 9 metres.", timeout_s=30)))
    t2.start(); time.sleep(0.2)
    info = relay.cancel_inflight()
    release.set()
    t1.join(10); t2.join(10)
    assert not t1.is_alive() and not t2.is_alive()
    assert info["dropped_queued"] == 1
    assert ran == [1.0]
    assert out2["response"] == CANCELLED_REPLY


def test_the_models_own_stop_does_not_cancel_its_own_turn(monkeypatch, tmp_path):
    ran = []
    holder = {}

    def stop(a):
        ran.append("stop")
        holder["info"] = holder["relay"].cancel_inflight()   # as act_stop's hook would
        return {"accepted": True, "commanded": 0.0, "achieved": 0.0, "settled": True}

    def drive(a):
        ran.append("drive")
        return {"accepted": True, "commanded": a["distance"], "achieved": a["distance"], "error": 0.0}

    relay = build(monkeypatch, tmp_path,
                  [Tool("stop_robot", "s", {"type": "object", "properties": {}}, stop),
                   Tool("drive_forward", "d", NUM, drive)],
                  calls(("stop_robot", {}), ("drive_forward", {"distance": -0.3})))
    holder["relay"] = relay
    out = relay.dispatch_sync("stop, then back up 0.3 metres", timeout_s=20)
    assert ran == ["stop", "drive"]
    assert holder["info"].get("own_turn") is True
    assert out["response"] != CANCELLED_REPLY


# ── the mobile bridge's wiring (source) ─────────────────────────────────

def test_mobile_bridge_lets_every_halt_skip_the_action_lock():
    src = MOBILE.read_text(encoding="utf-8")
    assert 'path == "/tool" and body.get("tool") == "stop_robot"' in src
    assert "shared_is_halt_order(body.get(\"text\"), \"mobile\")" in src


def test_mobile_bridge_counts_operator_halts_and_cancels_the_relay():
    src = MOBILE.read_text(encoding="utf-8")
    assert "self.halt_seq += 1" in src
    assert "bridge.on_operator_halt.append(relay.cancel_inflight)" in src


def test_mobile_bridge_counts_the_halt_before_releasing_the_running_leg():
    # Superseding the running leg wakes a multi-leg plan's thread, which reads
    # halt_seq at once. Counted after the supersede, the plan saw no halt and
    # made its next turn (ops-bench omnilink-after-fix-01, 2026-09-25).
    src = MOBILE.read_text(encoding="utf-8")
    body = src[src.index("    def act_stop(self"):]
    body = body[:body.index("\n    def ", 10)]
    assert body.index("self.halt_seq += 1") < body.index("self._supersede_inflight(")
