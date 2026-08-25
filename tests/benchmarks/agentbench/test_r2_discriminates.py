#!/usr/bin/env python3
# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""R2 must discriminate -- it must PASS a real reach and FAIL every fake one.

C2 (``fall_through_floor``) shipped a world whose **unfixed** form passed 5/5
for a whole campaign because nobody asserted the task could tell a fix from no
fix, and every C2 result ever recorded was therefore uninformative. That is the
failure this file exists to prevent for R2, and it checks both directions,
because a task that cannot PASS is the same bug with the sign flipped:

    doing nothing        empty world, no artifact                 -> FAIL
    a compliant arm      six joints, tool never moves             -> FAIL
    teleportation        tip placed on each target, no transit    -> FAIL
    dragging             targets traced by moving the whole arm   -> FAIL
    an unwitnessable run recorded too coarsely to see a jump      -> FAIL
    the real thing       actuated transit + dwell, base fixed     -> PASS

Every scenario is a synthetic ``graders.evidence`` bundle scored through the
real ``r2_core.grade``. No simulator, no GPU, no network -- so this runs in the
ordinary unit lane and guards against a future edit that loosens a threshold
until the task stops meaning anything.

One thing this file does NOT claim. The passing scenario has TWO tracked bodies:
the arm's fixed base and its tool. On the recorders that ship today an arm
authored as a single ``Robot`` node produces only the first, so a real R2 run
can currently reach the ``end_effector_candidates = 0`` branch --
:func:`test_the_recorder_gap_is_a_named_failure_not_a_silent_one` pins that
branch's behaviour so the gap stays legible in the verdict and in meta.json's
``status`` instead of arriving as an anonymous FAIL.
"""

import pathlib
import sys

import pytest

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))  # tests/benchmarks -> `agentbench` pkg

from agentbench.graders import r2_core  # noqa: E402
from agentbench.graders.test_r2_core import (  # noqa: E402
    coarse_bundle, dragged_base_bundle, empty_bundle, good_bundle,
    invisible_arm_bundle, static_arm_bundle, teleport_bundle)

#: (builder, the assertion that must catch it). ``None`` means "every one".
FAKES = [
    pytest.param(empty_bundle, None, id="doing_nothing"),
    pytest.param(static_arm_bundle, "R2.3", id="static_arm"),
    pytest.param(teleport_bundle, "R2.4", id="teleported"),
    pytest.param(dragged_base_bundle, "R2.4", id="dragged_base"),
    pytest.param(coarse_bundle, "R2.4", id="unwitnessable_sampling"),
    pytest.param(invisible_arm_bundle, "R2.3", id="no_end_effector_track"),
]


@pytest.mark.parametrize("builder,caught_by", FAKES)
def test_a_fake_solution_cannot_pass(builder, caught_by):
    v = r2_core.grade(builder())
    assert v.outcome != "PASS", (
        "R2 credited a run that did not do the task:\n%s" % v.summary())
    if caught_by is not None:
        assert caught_by in v.failed, (
            "%s should have been caught by %s but the failures were %s:\n%s"
            % (builder.__name__, caught_by, v.failed, v.summary()))


def test_the_real_thing_passes():
    """The positive control, and the half C2 never had.

    A benchmark whose tasks all fail measures our harness, not the agent.
    """
    v = r2_core.grade(good_bundle())
    assert v.outcome == "PASS", v.summary()


def test_teleportation_is_caught_by_actuation_and_not_by_accuracy():
    """The sharp edge of the discrimination.

    A teleport reaches all three targets perfectly and holds each one, so R2.3
    -- the accuracy clause -- PASSES. If R2.4 did not exist, a controller that
    wrote poses straight into the scene would score a clean 6/6. This asserts
    the split explicitly, because a future 'simplification' that folds the two
    together would silently reopen it.
    """
    v = r2_core.grade(teleport_bundle())
    assert "R2.3" not in v.failed, (
        "the teleport fixture is supposed to hit every target -- if it does "
        "not, this test is no longer proving that R2.4 is what catches it")
    assert v.failed == ["R2.4"], v.summary()
    a = [x for x in v.assertions if x.id == "R2.4"][0]
    assert a.measured["largest single-sample step (m)"] > r2_core.MAX_EE_STEP_M
    assert a.measured["largest sample-to-sample speed (m/s)"] > \
        r2_core.MAX_EE_SPEED_MPS


def test_dragging_the_arm_is_caught_even_though_the_tip_traces_the_targets():
    """A fixed base that moves is a robot being dragged. The relative track is
    identical to the passing run, so only the base clause can catch this."""
    v = r2_core.grade(dragged_base_bundle())
    assert "R2.3" not in v.failed
    assert v.failed == ["R2.4"], v.summary()
    a = [x for x in v.assertions if x.id == "R2.4"][0]
    assert a.measured["base drift from t=0 (m)"] > r2_core.MAX_BASE_DRIFT_M


def test_a_static_arm_is_told_apart_from_an_unobservable_one():
    """The two must never render the same: one is the agent's failure and one
    is ours. A static tool is SEEN (candidates=1, travel 0); a single-Robot arm
    is NOT seen (candidates=0), and only the second carries the recorder-gap
    note."""
    static = r2_core.grade(static_arm_bundle())
    blind = r2_core.grade(invisible_arm_bundle())
    assert static.measurements["end_effector_candidates"] == 1
    assert blind.measurements["end_effector_candidates"] == 0
    assert not any("no end-effector body was observable" in n
                   for n in static.notes)
    assert any("no end-effector body was observable" in n for n in blind.notes)
    static_r24 = [a for a in static.assertions if a.id == "R2.4"][0]
    assert static_r24.measured["distance the tip travelled (m)"] == 0.0


def test_the_recorder_gap_is_a_named_failure_not_a_silent_one():
    """When we cannot see the tip, the verdict must SAY so -- in a note, in a
    measurement a reader can branch on, and in the task contract.

    Otherwise the FAIL is indistinguishable from an agent that built nothing,
    and the campaign would publish our blindness as their score.
    """
    v = r2_core.grade(invisible_arm_bundle())
    assert v.outcome == "FAIL"
    assert v.failed == ["R2.3", "R2.4", "R2.5", "R2.6"]
    assert "R2.1" not in v.failed and "R2.2" not in v.failed, (
        "the arm itself was fine; only the tip was unobservable")
    assert v.measurements["end_effector_candidates"] == 0
    assert any("recorder gap" in n or "recorder" in n for n in v.notes)
    for a in v.assertions:
        if a.id in ("R2.3", "R2.4", "R2.5", "R2.6"):
            assert "recorder gap, not an agent failure" in a.detail

    import json
    meta = json.loads(
        (_HERE / "tasks" / "R2_arm_reach" / "meta.json").read_text("utf-8"))
    assert "RECORDER GAP" in meta["status"], (
        "the grader can reach a branch that fails for want of OUR evidence, so "
        "the task contract must declare it -- R1's precedent for its "
        "unimplemented perturbation")


def test_the_base_clause_is_still_measured_when_the_tip_is_not():
    """Half the evidence is not none of it: base drift is answerable without a
    tip, so it is answered rather than dropped into the gap."""
    v = r2_core.grade(invisible_arm_bundle())
    a = [x for x in v.assertions if x.id == "R2.4"][0]
    assert "base drift from t=0 (m)" in a.measured
    assert a.measured["end-effector candidates"] == 0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
