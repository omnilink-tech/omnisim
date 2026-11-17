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

"""A `/prompt` turn's budget is counted on the SLOWER clock (2026-10-17).

THE DEFECT. The arm bridge called `relay.dispatch_sync(text)` with the default
90 s WALL budget and ignored the body's `timeout_s`; the other three bridges
honoured `timeout_s` but still counted it on the wall clock. MEASURED
(WIRobotics ALLEX on installed v9.2.5, 0.09-0.18x real time): a correct
"red, blue, yellow, then home" turn needed 92.7 s of wall for 16.9 s of sim,
and the 90 s budget cancelled it after two hovers -- `timeout_after_actions`,
an empty reply, no home.

THE RULE NOW. With a sim clock the turn is cancelled only once BOTH clocks
have run `timeout_s`, so a slow world gets `timeout_s` of simulation and a
fast world still gets `timeout_s` of wall for the model to think. A sim that
stops stepping never spends its budget, so `wall_timeout_s` (default 600 s)
caps the wall clock alone. Every case below runs the shipped
`OmniLinkRelay.dispatch_sync` against a scripted turn, offline.
"""

from __future__ import annotations

import ast
import pathlib
import threading
import time

import pytest

from omnisim_bridges.http_security import RequestError, prompt_budget
from omnisim_bridges.relay import OmniLinkRelay
from omnisim_bridges.tool import Tool

ROOT = pathlib.Path(__file__).resolve().parents[3]
CONTROLLERS = ROOT / "projects/samples/demos/controllers"
BRIDGES = {
    "arm": CONTROLLERS / "omnilink_arm_bridge/omnilink_arm_bridge.py",
    "mobile": CONTROLLERS / "omnilink_mobile_bridge/omnilink_mobile_bridge.py",
    "quadruped": CONTROLLERS / "omnilink_quadruped_bridge/omnilink_quadruped_bridge.py",
    "drone": CONTROLLERS / "mavic_omnilink_bridge/mavic_omnilink_bridge.py",
}


class _Handle:
    def __init__(self):
        self.cancelled = False

    def cancel(self) -> bool:
        self.cancelled = True
        return True


def _relay(monkeypatch, tmp_path, finish_after_s):
    """A relay whose turn reports idle after `finish_after_s` WALL seconds
    (None: never)."""
    monkeypatch.setenv("OMNILINK_INTENT_STATE_DIR", str(tmp_path))
    monkeypatch.setenv("OMNILINK_PRESENCE", "0")
    monkeypatch.setenv("OMNILINK_EDGE", "0")
    relay = OmniLinkRelay(
        omni_key="olink_test_key_not_used_offline",
        agent_name="BudgetProbe", main_task="test",
        tools=[Tool("noop", "no-op", {"type": "object", "properties": {}},
                    lambda a: {"ok": True})],
        usage_enabled=False, memory_enabled=False, surface="arm")
    handle = _Handle()

    def _fake_async(text, cb, lane="main"):
        if finish_after_s is not None:
            def _finish():
                time.sleep(finish_after_s)
                cb("agent", {"text": "all three hovers done"})
                cb("status", {"state": "idle"})
            threading.Thread(target=_finish, daemon=True).start()
        return handle

    relay.dispatch_async = _fake_async          # type: ignore[assignment]
    return relay, handle


class _SimClock:
    """A sim clock advancing `rate` sim-seconds per wall-second."""

    def __init__(self, rate: float):
        self.rate = rate
        self.t0 = time.monotonic()

    def __call__(self) -> float:
        return (time.monotonic() - self.t0) * self.rate


def test_a_slow_world_is_given_its_budget_in_sim_time(monkeypatch, tmp_path):
    # 0.1x real time: 0.5 s of wall is only 0.05 s of sim, far inside a
    # 0.2 s budget. The old wall rule cancelled this turn at 0.2 s.
    relay, handle = _relay(monkeypatch, tmp_path, finish_after_s=0.5)
    out = relay.dispatch_sync("hover over the three tags", timeout_s=0.2,
                              sim_clock=_SimClock(0.1))
    assert handle.cancelled is False
    assert out.get("error") is None, out
    assert out["response"] == "all three hovers done"


def test_a_fast_world_still_gets_its_budget_in_wall_time(monkeypatch, tmp_path):
    # 50x real time: the sim budget is spent in milliseconds, but the model
    # is thinking in WALL time -- a sim-only rule would cut it off.
    relay, handle = _relay(monkeypatch, tmp_path, finish_after_s=0.3)
    out = relay.dispatch_sync("hover", timeout_s=0.6, sim_clock=_SimClock(50.0))
    assert handle.cancelled is False
    assert out.get("error") is None, out


def test_a_turn_that_never_ends_is_cancelled_once_both_clocks_ran(monkeypatch, tmp_path):
    relay, handle = _relay(monkeypatch, tmp_path, finish_after_s=None)
    t0 = time.monotonic()
    out = relay.dispatch_sync("hover", timeout_s=0.3, sim_clock=_SimClock(5.0))
    took = time.monotonic() - t0
    assert handle.cancelled is True
    # in flight when cancelled (the fake handle says so), hence *_after_actions
    assert out["error"] == "timeout_after_actions" and out["cancelled"] is True
    assert out["budget"]["expired_on"] == "sim"
    assert out["budget"]["clock"] == "sim+wall"
    assert out["budget"]["elapsed_sim_s"] >= 0.3
    assert 0.3 <= took < 1.5


def test_a_stalled_sim_is_bounded_by_the_wall_cap(monkeypatch, tmp_path):
    # The sim never advances (a paused world): the sim budget can never be
    # spent, so only `wall_timeout_s` ends the wait -- and says so.
    relay, handle = _relay(monkeypatch, tmp_path, finish_after_s=None)
    t0 = time.monotonic()
    out = relay.dispatch_sync("hover", timeout_s=0.1, sim_clock=lambda: 12.0,
                              wall_timeout_s=0.4)
    took = time.monotonic() - t0
    assert handle.cancelled is True
    assert out["budget"]["expired_on"] == "wall_cap"
    assert out["budget"]["wall_timeout_s"] == 0.4
    assert out["budget"]["elapsed_sim_s"] == 0.0
    assert 0.4 <= took < 1.5


def test_an_unreadable_sim_clock_falls_back_to_the_wall_rule(monkeypatch, tmp_path):
    relay, _h = _relay(monkeypatch, tmp_path, finish_after_s=None)

    def _broken():
        raise RuntimeError("no clock")

    t0 = time.monotonic()
    out = relay.dispatch_sync("hover", timeout_s=0.2, sim_clock=_broken)
    assert out["budget"]["expired_on"] == "wall"
    assert time.monotonic() - t0 < 1.5


def test_no_sim_clock_keeps_the_wall_budget(monkeypatch, tmp_path):
    relay, handle = _relay(monkeypatch, tmp_path, finish_after_s=None)
    out = relay.dispatch_sync("hover", timeout_s=0.2)
    assert handle.cancelled is True
    assert out["budget"]["clock"] == "wall"
    assert out["budget"]["expired_on"] == "wall"
    assert out["budget"]["timeout_s"] == 0.2


def test_prompt_budget_parses_and_bounds_the_body():
    assert prompt_budget({}) == (90.0, None)
    assert prompt_budget({"timeout_s": 300}) == (300.0, None)
    assert prompt_budget({"timeout_s": 60, "wall_timeout_s": 900}) == (60.0, 900.0)
    for bad in ({"timeout_s": 0}, {"timeout_s": 601}, {"timeout_s": "90"},
                {"timeout_s": 90, "wall_timeout_s": 30},
                {"timeout_s": 90, "wall_timeout_s": 4000}):
        with pytest.raises(RequestError) as exc:
            prompt_budget(bad)
        assert exc.value.status == 400


def _dispatch_calls(path: pathlib.Path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return [n for n in ast.walk(tree)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == "dispatch_sync"]


@pytest.mark.parametrize("name", sorted(BRIDGES))
def test_every_bridge_passes_its_sim_clock_and_the_body_budget(name):
    """The arm's bare `dispatch_sync(text)` is what cancelled ALLEX. Each
    shipped bridge's /prompt must pass the parsed budget AND its sim clock;
    a bridge that drops either is back on a fixed or wall-only budget."""
    path = BRIDGES[name]
    calls = _dispatch_calls(path)
    assert calls, f"{name}: no dispatch_sync call found"
    for call in calls:
        kws = {k.arg for k in call.keywords}
        assert {"timeout_s", "sim_clock", "wall_timeout_s"} <= kws, (
            f"{name}: dispatch_sync called with {sorted(kws)} -- the /prompt "
            f"budget would be wall-only or fixed")
    assert "prompt_budget(body)" in path.read_text(encoding="utf-8"), name
