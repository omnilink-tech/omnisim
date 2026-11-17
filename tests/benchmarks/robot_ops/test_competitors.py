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
"""Offline contract tests for the competitor arms: no engine, no model.

A scripted model and a fake bridge prove the plumbing -- tool surface per
tier, the operator's words on every gated call, queueing (basic) versus
interruption (full), protocol repair -- before any paid request is made.
"""
import json
import shutil
import threading
import time

import pytest

from omnisim.ops_bench import competitors as C


class FakeSession:
    robot = "husky"
    url = "http://fake"

    def __init__(self, drive_s=0.0):
        self.posts = []
        self.drive_s = drive_s
        self.x = 0.0
        self.stopped = threading.Event()

    def state(self):
        return {"x": self.x, "y": 0.0, "yaw": 0.0, "mode": "idle", "sim_time": 1.0}

    def post(self, path, body, timeout=90):
        self.posts.append((path, dict(body)))
        if path == "/stop_robot":
            self.stopped.set()
            return {"accepted": True}
        if body.get("tool") == "drive_forward":
            end = time.monotonic() + self.drive_s
            while time.monotonic() < end and not self.stopped.is_set():
                time.sleep(0.01)
            self.x += body["distance"]
            return {"status": "ok", "result": {"accepted": True, "achieved": body["distance"]}}
        return {"status": "ok", "result": {"accepted": True}}


class ScriptedModel:
    def __init__(self, plans):
        self.plans = list(plans)
        self.calls = []

    def complete(self, system, messages):
        self.calls.append((system, messages))
        return self.plans.pop(0) if self.plans else json.dumps({"actions": [], "say": "idle", "done": True})


def clock():
    return time.monotonic()


def agent(arm, model, session=None, monkeypatch=None, tools=None):
    if tools is not None and monkeypatch is not None:
        monkeypatch.setattr(C.CompetitorAgent, "_tool_surface", lambda self: tools)
    return C.CompetitorAgent(session or FakeSession(), clock, arm, model)


def wait(r, s=5):
    assert r.done.wait(s)
    return r


def plan(actions, say="", done=True):
    return json.dumps({"actions": actions, "say": say, "done": done})


def test_basic_tier_offers_only_the_four_motion_tools():
    a = agent("plain_basic", ScriptedModel([]))
    assert {t["name"] for t in a.tools} == {"drive_forward", "turn", "stop_robot", "get_robot_state"}
    assert "set_boundary" not in a.system
    a.close()


def test_full_tier_offers_exactly_what_the_bridge_publishes(monkeypatch):
    listed = [{"name": "drive_forward", "description": "d", "parameters": {}},
              {"name": "schedule_action", "description": "s", "parameters": {}}]
    monkeypatch.setattr("omnisim.control_bench.engine.request", lambda url, timeout=10: {"tools": listed})
    a = C.CompetitorAgent(FakeSession(), clock, "plain_full", ScriptedModel([]))
    assert {t["name"] for t in a.tools} == {"drive_forward", "schedule_action"}
    a.close()


def test_every_call_is_gated_with_the_operators_words_and_the_reply_is_the_models():
    s = FakeSession()
    m = ScriptedModel([plan([{"tool": "drive_forward", "args": {"distance": 1.0}}], "Drove 1 m.")])
    a = agent("plain_basic", m, s)
    r = wait(a.say({"id": "a"}, "Drive forward 1 metre.", {}))
    assert r.text == "Drove 1 m." and r.error is None
    path, body = s.posts[0]
    assert path == "/tool" and body["utterance"] == "Drive forward 1 metre." and body["wait"] is True
    a.close()


def test_an_unlisted_tool_is_refused_not_called():
    s = FakeSession()
    m = ScriptedModel([plan([{"tool": "teleport", "args": {}}], "", done=False),
                       plan([], "I cannot do that.")])
    a = agent("plain_basic", m, s)
    r = wait(a.say({"id": "a"}, "Teleport.", {}))
    assert s.posts == [] and r.text == "I cannot do that."
    a.close()


def test_a_malformed_plan_is_repaired_within_the_round_budget():
    m = ScriptedModel(["sure thing!", plan([], "Done.")])
    a = agent("plain_basic", m)
    r = wait(a.say({"id": "a"}, "hello", {}))
    assert r.text == "Done." and len(m.calls) == 2
    assert "protocol_error" in m.calls[1][1][-1]["content"]
    a.close()


def test_basic_tier_handles_messages_in_order_and_never_interrupts():
    s = FakeSession(drive_s=0.5)
    m = ScriptedModel([plan([{"tool": "drive_forward", "args": {"distance": 2.0}}], "Drove."),
                       plan([{"tool": "stop_robot", "args": {}}], "Stopped.")])
    a = agent("plain_basic", m, s)
    r1 = a.say({"id": "a"}, "Drive forward 2 metres.", {})
    time.sleep(0.1)
    r2 = a.say({"id": "b"}, "Stop!", {})
    wait(r1); wait(r2)
    assert [p for p, _ in s.posts] == ["/tool", "/tool"]      # no escape-hatch stop
    assert s.x == 2.0                                          # the drive ran to the end
    a.close()


def test_full_tier_interrupts_the_running_plan_on_a_new_message(monkeypatch):
    s = FakeSession(drive_s=2.0)
    m = ScriptedModel([plan([{"tool": "drive_forward", "args": {"distance": 2.0}},
                             {"tool": "turn", "args": {"angle_rad": 1.0}}], "Done."),
                       plan([], "Stopped as asked.")])
    a = agent("plain_full", m, s, monkeypatch, tools=list(C.BASIC_TOOLS))
    r1 = a.say({"id": "a"}, "Drive forward 2 metres, then turn left.", {})
    time.sleep(0.2)
    r2 = a.say({"id": "b"}, "Stop!", {})
    wait(r1); wait(r2)
    paths = [(p, b.get("tool")) for p, b in s.posts]
    assert ("/stop_robot", None) in paths                        # the escape hatch
    assert ("/tool", "turn") not in paths                        # the rest was abandoned
    assert r2.text == "Stopped as asked."
    a.close()


@pytest.mark.skipif(pytest.importorskip("importlib").util.find_spec("langgraph") is None,
                    reason="LangGraph not installed in this interpreter")
def test_langgraph_arm_runs_the_same_loop_in_a_real_stategraph():
    s = FakeSession()
    m = ScriptedModel([plan([{"tool": "drive_forward", "args": {"distance": 0.5}}], "", done=False),
                       plan([], "Drove 0.5 m.")])
    a = agent("langgraph_basic", m, s)
    r = wait(a.say({"id": "a"}, "Drive forward 0.5 metres.", {}))
    assert r.text == "Drove 0.5 m." and s.x == 0.5 and a.graph is not None
    a.close()


@pytest.mark.skipif(shutil.which("node") is None or not (
    C.REPO_ROOT / "tests/benchmarks/harness_comparison/vendor/lobster/dist/src/sdk/index.js").exists(),
    reason="Node or the pinned Lobster SDK is not present")
def test_lobster_arm_runs_the_same_loop_in_the_real_sdk():
    s = FakeSession()
    m = ScriptedModel([plan([{"tool": "drive_forward", "args": {"distance": 0.5}}], "Drove 0.5 m.")])
    a = agent("lobster_basic", m, s)
    r = wait(a.say({"id": "a"}, "Drive forward 0.5 metres.", {}), s=20)
    assert r.error is None and r.text == "Drove 0.5 m." and s.x == 0.5
    a.close()


def test_the_model_client_refuses_past_its_cap():
    client = C.ModelClient("k", "g3-engine", "", "OmniSim-husky", max_requests=0)
    with pytest.raises(C.InfrastructureError):
        client.complete("s", [])


# ── the long-context tier (2026-10-01, long-horizon benchmark) ──────────────

class _LCModel:
    """Plans: say 'ok' and finish. Summaries: a fixed marker."""
    def __init__(self):
        self.summaries = 0

    def complete(self, system, messages):
        if system == C.LC_SUMMARY_SYSTEM:
            self.summaries += 1
            return f"SUMMARY-{self.summaries}"
        return json.dumps({"actions": [], "say": "ok", "done": True})


def _lc(tmp_path, monkeypatch, arm="plain_lc"):
    session = FakeSession()
    session.directory = tmp_path
    monkeypatch.setattr(C.CompetitorAgent, "_tool_surface", lambda self: list(C.BASIC_TOOLS))
    monkeypatch.setattr(C.CompetitorAgent, "_main_task", lambda self: "")
    model = _LCModel()
    return C.CompetitorAgent(session, clock, arm, model), model


def test_long_context_tier_folds_old_messages_into_a_summary(tmp_path, monkeypatch):
    a, model = _lc(tmp_path, monkeypatch)
    for i in range(40):
        a._handle(f"message {i}")
    assert model.summaries >= 1 and a.summary.startswith("SUMMARY-")
    assert len(a.history) <= C.LC_KEEP + C.LC_FOLD
    # the summary rides at the front of every later model call
    a._handle("one more")
    assert (tmp_path / "plain_lc_memory.json").exists()
    a.close()


def test_long_context_tier_survives_a_restart_and_full_tier_does_not(tmp_path, monkeypatch):
    a, _ = _lc(tmp_path, monkeypatch)
    for i in range(40):
        a._handle(f"message {i}")
    kept = (a.summary, len(a.history))
    a.on_restart()
    assert (a.summary, len(a.history)) == kept          # reloaded from its checkpoint
    a.close()
    f = C.CompetitorAgent.__new__(C.CompetitorAgent)
    f.tier, f.history, f.summary, f._memory_path = "full", [{"role": "user", "content": "x"}], "", None
    f.on_restart()
    assert f.history == [] and f.summary == ""          # a framework default keeps nothing
