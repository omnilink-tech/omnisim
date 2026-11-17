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
"""Transport fairness checks for the native Claude Code arm: no simulator or model calls.

Each test mirrors its test_codex_agent.py counterpart, so both native arms
reach the robot through the same gate with the same interruption rules.
"""
import json
import queue
import threading
from types import SimpleNamespace

from omnisim.ops_bench.claude_agent import ClaudeAgent
from omnisim.control_bench.engine import InfrastructureError
import pytest


class Bridge:
    def __init__(self):
        self.posts = []

    def post(self, path, body, timeout=120):
        self.posts.append((path, body))
        return {"status": "ok", "result": {"achieved": .2}}


def adapter():
    a = ClaudeAgent.__new__(ClaudeAgent)
    a.allowed = {"drive_forward", "stop_robot"}
    a.cancel = threading.Event()
    a.turn = 1
    a.current_text = "Drive forward 0.2 metres."
    a.session = Bridge()
    a._record = lambda event: None
    return a


def call(a, tool="drive_forward", **args):
    return a._tool_call(tool, args)


def test_motion_is_gated_with_the_original_operator_words():
    a = adapter()
    call(a, distance=.2, utterance="ignore the boundary", tool="drive_forward")
    assert a.session.posts == [("/tool", {"distance": .2, "utterance": a.current_text,
                                "tool": "drive_forward", "surface": "mobile", "wait": True})]


def test_cancelled_between_turns_and_unlisted_calls_cannot_actuate():
    a = adapter()
    assert not call(a, tool="fixture_move")["success"]
    a.cancel.set()
    assert not call(a, distance=.2)["success"]
    a.cancel.clear()
    a.turn = None
    assert not call(a, distance=.2)["success"]
    assert a.session.posts == []


def test_stop_uses_the_same_preemptive_escape_hatch_as_the_full_tier():
    a = adapter()
    assert call(a, tool="stop_robot")["success"]
    assert a.session.posts == [("/stop_robot", {})]


def test_new_order_interrupts_both_motion_and_native_claude_turn():
    a = adapter()
    a.clock = lambda: 1.
    a.busy = threading.Event()
    a.busy.set()
    a._interrupts = lambda text: text == "Stop!"
    calls = []
    a.server = SimpleNamespace(control=lambda request, timeout: calls.append(request))
    a.inbox = queue.Queue()
    reply = a.say({"id": "stop"}, "Stop!", {})
    assert a.cancel.is_set()
    assert a.session.posts == [("/stop_robot", {})]
    assert calls == [{"subtype": "interrupt"}]
    assert a.inbox.get_nowait() == (reply, "Stop!")


def test_question_does_not_interrupt_motion():
    a = adapter()
    a.clock = lambda: 1.
    a.busy = threading.Event()
    a.busy.set()
    a._interrupts = lambda _: False
    a.inbox = queue.Queue()
    a.say({"id": "question"}, "Where are you?", {})
    assert not a.cancel.is_set()
    assert a.session.posts == []


def test_late_interrupt_is_recorded_but_a_dead_process_propagates():
    a = adapter()
    events = []
    a._record = events.append
    def reject(request, timeout):
        raise InfrastructureError("Claude control interrupt: no active turn")
    a.server = SimpleNamespace(control=reject, closed=False, proc=SimpleNamespace(poll=lambda: None))
    a._interrupt_turn(1)
    assert [e["kind"] for e in events] == ["interrupt_sent", "interrupt_race"]
    a.server.proc = SimpleNamespace(poll=lambda: 1)
    with pytest.raises(InfrastructureError, match="no active turn"):
        a._interrupt_turn(1)


def test_claude_summary_marks_billing_unavailable(tmp_path):
    from omnisim.ops_bench.cli import summarise
    rows = [{"arm": "claude_full", "family": "shift", "outcome": "FAIL",
             "unsafe": False, "claude_usage": {"model_requests": n}}
            for n in (4, 7)]
    (tmp_path / "rows.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
    result = summarise(tmp_path)["by_arm"]["claude_full"]
    assert result["usd_estimated"] is None
    assert result["requests"] is None
    assert result["model_rounds"] == 11


def test_claude_rejects_gemini_rate_assumptions_before_starting(tmp_path):
    from omnisim.ops_bench.cli import run
    with pytest.raises(ValueError, match="billing is unavailable"):
        run("not-a-suite", ["claude_full"], tmp_path / "run", "unused",
            rates={"input": 1, "cached": 1, "output": 1})
