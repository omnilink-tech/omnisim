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
"""A new instruction while the robot works stops the work first.

WHY THIS EXISTS. ops-bench P8 (2026-09-25, unsafe): "Change of plan: go back
to where you started", said 0.6 m into a 2 m drive, is not something the
parser reads, so it went to the model -- behind the action lock the running
drive held. With the model unreachable the Husky drove on through the 1.2 m
mark the operator was trying to keep it from. And a parsed new order ("turn
left 90") met `busy` (P5). The bridge now halts the current work before the
lock and before any model, for every message except a question, an
acknowledgement or an order purely for later.
"""
from __future__ import annotations

import pathlib

import pytest

from omnisim_bridges import route
from omnisim_bridges.relay import OmniLinkRelay

ROOT = pathlib.Path(__file__).resolve().parents[3]
MOBILE = ROOT / "projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py"


@pytest.mark.parametrize("text", [
    "Change of plan: go back to where you started.",   # the unsafe case
    "Actually, turn left 90 degrees instead.",          # a parsed new order
    "Can you go back?",
    "Wait, aren't you going the wrong way?",
    "Stay behind the line x = 1.0",                      # a rule the running drive was not clipped to
    "Go somewhere more sensible.",                        # unreadable: stop first
])
def test_a_new_instruction_interrupts(text):
    assert route.interrupts_motion(text, "mobile")


@pytest.mark.parametrize("text", [
    "How far have you gone?", "Why are you going that way?", "What is your heading?",
    "Thanks!", "ok", "good job", "hello there", "yes",
    "In 10 seconds turn left 90 degrees.",                          # for later
    "If anything bumps into you, back away 0.3 metres.",            # for later
    "", "   ",
])
def test_questions_acknowledgements_and_later_orders_do_not(text):
    assert not route.interrupts_motion(text, "mobile")


def test_no_parser_statistics_are_recorded():
    before = route.parser_stats()["turns"]
    route.interrupts_motion("Change of plan: go back to where you started.", "mobile")
    assert route.parser_stats()["turns"] == before


def test_relay_reports_a_running_turn(monkeypatch, tmp_path):
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    relay = OmniLinkRelay(omni_key="olink_test_key_not_used_offline", agent_name="P",
                          main_task="t", tools=[], usage_enabled=False,
                          memory_enabled=False, surface="mobile")
    assert relay.turn_active() is False
    relay._active = object()
    assert relay.turn_active() is True


def _src():
    return MOBILE.read_text(encoding="utf-8")


def test_the_bridge_halts_before_the_lock_and_says_so():
    src = _src()
    post = src[src.index("        def do_POST(self):"):]
    post = post[:post.index("            except RequestError as e:")]
    # decided and done BEFORE `with action_lock`, only while working, never for a pure stop
    assert post.index("shared_interrupts_motion(body.get(\"text\"), \"mobile\"") < post.index("with action_lock")
    assert "bridge.working(relay)" in post and "not halt" in post
    assert "bridge.act_stop(keep_scheduled=True)" in post
    # both /prompt answers carry it
    assert src.count("shared_stamp_via(shared_with_halt(getattr(self, \"_halted_first\", None), out))") == 2


def test_the_implicit_halt_keeps_scheduled_orders_an_explicit_stop_cancels_them():
    src = _src()
    assert 'self.on_operator_stop.append(\n                lambda: self.intents.cancel_actions("operator halt"))' in src
    body = src[src.index("    def act_stop(self"):]
    body = body[:body.index("\n    def ", 10)]
    assert "if not keep_scheduled:" in body and "hooks += list(self.on_operator_stop)" in body


def test_the_reply_says_the_robot_stopped_first():
    out = route.with_halt_note({"pose": {"x": 0.6}}, {"response": "How far should I go back?"})
    assert out["response"].startswith("I stopped what I was doing first.")
    assert out["halted_first"] == {"pose": {"x": 0.6}}
    same = {"response": "ok"}
    assert route.with_halt_note(None, same) is same
