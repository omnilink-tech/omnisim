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

"""The safety veto must be on the MODEL's path, and must fail closed.

Until 2026-09-21 `gate.check` ran in exactly one place -- route.py, the
deterministic parser path. So the parser was vetted and the model was not,
which is backwards from where the risk is. Measured on 64 held-out
sentences: the parser produced 0 false actuations, the model produced 6-8,
including "drive forward 300 metres" and acting on an order quoted inside
a statement.

The gap survived a reading of relay.py because the dispatch site already
had something called `execution_gate` -- a threading lock, unrelated.
"""
from __future__ import annotations

import builtins

import pytest

# The veto lives in bridge_base now -- ONE copy for the relay, the four
# demo bridges and the reference handler. `relay._gate_reject` was one of
# six near-identical wrappers and was deleted on 2026-09-22; the argument
# order changed with it (tool first, then args, then the utterance).
from omnisim_bridges.bridge_base import vet_toolcall


def _gate_reject(utterance, tool, args, surface=None):
    """The relay's old call shape, so these cases read unchanged."""
    return vet_toolcall(tool, args, utterance, surface=surface)


# ── It refuses what the gate refuses ────────────────────────────────
@pytest.mark.parametrize("utterance,tool,args,rule", [
    ("drive forward 300 metres", "drive_forward", {"distance": 300},
     "implausible"),
    ("turn left 3000 degrees", "turn", {"angle_rad": 52.36}, "implausible"),
    ("drive forward negative 2 metres", "drive_forward", {"distance": -2},
     "sign_conflict"),
    ("how far can you drive forward?", "drive_forward", {"distance": 1},
     "interrogative"),
    ("under no circumstances drive forward", "drive_forward",
     {"distance": 1}, "prohibition"),
    ('the manual says "drive forward 2 metres"', "drive_forward",
     {"distance": 2}, "reported_speech"),
    ("reverse 2 metres - sorry, ignore that", "drive_forward",
     {"distance": -2}, "retracted"),
])
def test_the_model_is_refused_what_the_parser_would_be(utterance, tool, args, rule):
    """Refused, and for the stated reason -- but not necessarily ONLY it.

    ⚠️ This asserted `out.startswith(rule)`, i.e. that the named rule was
    the FIRST rejection. That held only while each sentence tripped one
    rule. "under no circumstances drive forward" now trips two --
    prohibition AND invented_magnitude, since it names no distance -- and
    the order between them is an implementation detail of the rule list.
    Both are correct, so the assertion is membership, not ordering.
    """
    from omnisim_bridges import gate

    out = _gate_reject(utterance, tool, args)
    assert out is not None, f"{utterance!r} reached an actuator ungated"
    fired = {r.rule for r in gate.check(utterance, [{"tool": tool, "args": args}])}
    assert rule in fired, f"{rule} not among {sorted(fired)}"


# ── It does not refuse what it should not ───────────────────────────
@pytest.mark.parametrize("utterance,tool,args", [
    ("drive forward 2 metres", "drive_forward", {"distance": 2, "wait": True}),
    ("turn left 90 degrees", "turn", {"angle_rad": 1.5708, "wait": True}),
    ("stop", "stop_robot", {}),
    ("what is your position?", "get_robot_state", {"kind": "pose"}),
    ("set your speed to 0.4 and drive forward 2 metres", "drive_forward",
     {"distance": 2, "speed": 0.4}),
    ("drive forward 5 metres, scratch that, make it 2 metres",
     "drive_forward", {"distance": 2}),
    ("reverse 3 metres. never mind, go forward 1 metre", "drive_forward",
     {"distance": 1}),
])
def test_legitimate_orders_still_run(utterance, tool, args):
    """A false refusal is an order a person gave and a robot ignored.

    These are the cases that make an over-broad rule expensive, including
    the `wait`/`speed` arguments the bridge really sends and the
    retractions that REPLACE rather than withdraw.
    """
    assert _gate_reject(utterance, tool, args) is None


# ── Fail closed ─────────────────────────────────────────────────────
def test_it_fails_CLOSED_when_the_gate_cannot_be_imported(monkeypatch):
    """A safety check that vanishes with its import is not a safety check.

    This module is loaded by a bundled interpreter that has surprised us:
    omnilink-lib was invisible to it for an entire session, the failure was
    swallowed, and the fallback reported the wrong cause. If that happens
    to the gate, physical tools must stop, not sail through.
    """
    real_import = builtins.__import__

    def boom(name, globals=None, locals=None, fromlist=(), level=0):
        if "gate" in (fromlist or ()) or name.endswith("gate"):
            raise ImportError("simulated: gate module missing")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", boom)
    out = _gate_reject("drive forward 2 metres", "drive_forward",
                       {"distance": 2})
    assert out is not None and "unavailable" in out


def test_a_read_only_tool_still_answers_when_the_gate_is_gone(monkeypatch):
    """Failing closed must not take the read-only surface down with it.

    Refusing to ANSWER because a motion check is unavailable would be a
    self-inflicted outage; `get_robot_state` cannot move anything.
    """
    real_import = builtins.__import__

    def boom(name, globals=None, locals=None, fromlist=(), level=0):
        if "gate" in (fromlist or ()) or name.endswith("gate"):
            raise ImportError("simulated: gate module missing")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", boom)
    assert _gate_reject("where are you?", "get_robot_state",
                        {"kind": "pose"}) is None


def test_it_fails_closed_when_the_gate_itself_raises(monkeypatch):
    import omnisim_bridges.gate as g

    def explode(*a, **k):
        raise RuntimeError("simulated gate fault")

    monkeypatch.setattr(g, "check", explode)
    out = _gate_reject("drive forward 2 metres", "drive_forward",
                       {"distance": 2})
    assert out is not None and "errored" in out


# ── The bare /tool path: a tool call with no sentence ───────────────
def test_a_bare_tool_call_is_not_refused_for_having_a_number():
    """⚠️ THIS BRICKED A ROBOT SURFACE. Do not let it regress.

    `invented_magnitude` asks whether the utterance mentioned the number
    in the frame. On the platform's `/tool` callback there IS no
    utterance -- just a tool and its arguments -- so the question is
    vacuous and the answer was "no", refusing EVERY drive, turn and
    takeoff over that surface.

    The e2e test that should have caught it had run before the rule
    existed and stayed green. It was found by a second implementation of
    the gate reading the same code.
    """
    from omnisim_bridges.bridge_base import vet_toolcall as toolcall

    for tool, args in (("drive_forward", {"distance": 2.0}),
                       ("turn", {"angle_rad": 1.5708}),
                       ("takeoff", {"altitude": 2.0}),
                       ("walk", {"distance": 3.0}),
                       ("stop_robot", {})):
        assert toolcall(tool, args) is None, (
            f"/tool refused a legitimate {tool}: {toolcall(tool, args)}")


def test_the_bare_path_still_refuses_what_needs_no_sentence():
    """Dropping the rule must not drop the rails that need no utterance."""
    from omnisim_bridges.bridge_base import vet_toolcall as toolcall

    assert "implausible" in (toolcall("drive_forward", {"distance": 300.0}) or "")
    assert "implausible" in (toolcall("set_velocity", {"v": 40.0, "w": 0.0}) or "")
    assert "implausible" in (toolcall("turn", {"angle_rad": 52.36}) or "")


def test_invented_magnitude_still_fires_when_there_IS_an_utterance():
    from omnisim_bridges import gate

    fired = {r.rule for r in gate.check(
        "forward", [{"tool": "drive_forward", "args": {"distance": 1.0}}])}
    assert "invented_magnitude" in fired
    assert gate.check(
        "drive forward 2 metres",
        [{"tool": "drive_forward", "args": {"distance": 2.0}}]) == []
