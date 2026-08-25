#!/usr/bin/env python3
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

"""The MuJoCo arm's remaining bring-up is scoped PER TASK -- and honestly.

R1 on this arm is publishable and A1/R2 are not, and those three facts have to
hold together or the scoping was just a way of turning a light green.

The change under test moved ``sims.SIMS["mujoco"].pending`` from one free-text
string to three :class:`sims.Pending` items, each declaring which tasks it
blocks. The string made the arm's publication bar the WORST of its tasks: R1's
oracle/null gate was green, on record, and pinned by
``test_r1_discriminates_mujoco.py``, while two sentences about A1's missing
gate and A1's missing Husky analogue kept every MuJoCo cell unpublishable.

That is the mirror image of the C2 defect rather than a safe version of it. C2
shipped a green nobody could make red; this was a red nobody could make green
for any reason connected to the row it was printed on -- and the only way to
clear such a red is to delete the text, which is how a real gap gets lost. So
this file pins BOTH directions:

* R1 is clear of pending items **and** its gate is on record. Descoping alone
  must never be enough to publish a cell (:func:`test_r1s_green_is_backed_by_a
  _recorded_gate_and_not_by_the_scoping`).
* A1 and R2 are still blocked, by name, and still fail their gate check.
* An item that narrows its scope must say why the other tasks are immune, and
  the constructor refuses it otherwise.
* The prose fallback that used to satisfy ``readiness._discriminating`` from
  ``pending`` text is gone, and a pending sentence saying a gate is MISSING can
  no longer be read as a gate being present.

Pure registry logic: no MuJoCo, no simulator, no network. It runs everywhere.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench import readiness, sims  # noqa: E402

ARM = "mujoco"
R1 = "R1_lidar_nav"
A1 = "A1_husky_swarm_10"
R2 = "R2_arm_reach"

VERDICTS = (Path(__file__).resolve().parents[2] / "preregister"
            / "oracle_verdicts.json")


def _mujoco():
    return sims.get(ARM)


def _gates(task, arm):
    """The recorded oracle/null verdicts for one (task, arm), if any."""
    doc = json.loads(VERDICTS.read_text(encoding="utf-8"))
    cells = list(doc.get("cells") or [])
    cells += list((doc.get("driver_gates") or {}).get("cells") or [])
    return {c.get("agent"): c.get("outcome") for c in cells
            if c.get("task") == task and c.get("sim") == arm}


# --- R1: green, and green for a reason that is not the scoping --------------

def test_nothing_pending_on_this_arm_blocks_R1():
    """The bring-up debt this arm carries is A1's and R2's, not R1's."""
    sim = _mujoco()
    assert sim.pending_for(R1) == (), (
        "these items block R1: %s" % [it.id for it in sim.pending_for(R1)])
    assert sim.publishable_for(R1) is True


def test_r1s_green_is_backed_by_a_recorded_gate_and_not_by_the_scoping():
    """The anti-C2 invariant, stated as a test rather than as a convention.

    ``publishable_for`` is a NECESSARY condition and this asserts it is not
    treated as a sufficient one: R1 may publish on this arm only because the
    oracle/null gate was actually RUN here and its verdicts are on record.
    Delete the record and this fails even though ``pending`` stays empty --
    which is the whole point. A cell that goes green because someone narrowed
    a scope, with no gate behind it, is exactly the defect that let an unfixed
    C2 world pass 5/5 for a campaign.
    """
    got = _gates(R1, ARM)
    assert got.get("oracle") == "PASS" and got.get("null") == "FAIL", got
    state, why = readiness._discriminating(R1, ARM)
    assert state == readiness.OK, why


def test_readiness_publishes_R1_on_this_arm_only_with_both_halves():
    """End to end through the table an operator actually reads."""
    _task, rows = readiness.check(R1)
    gates = dict(rows)[ARM]
    assert gates["expressible"][0] == readiness.OK
    assert gates["deliverable"][0] == readiness.OK
    assert gates["discriminating"][0] == readiness.OK, gates["discriminating"]
    assert gates["publishable"][0] == readiness.OK, gates["publishable"]


# --- A1 and R2: still blocked, by name --------------------------------------

@pytest.mark.parametrize("task,expect", [
    (A1, {"A1_gate", "A1_husky_analogue"}),
    (R2, {"R2_gate"}),
])
def test_the_other_two_tasks_are_still_blocked_by_the_items_that_name_them(
        task, expect):
    """Scoping narrowed these items; it did not clear them.

    If a later change closes one of these for real, this test should be
    edited alongside the closure -- which is the intended friction. Silently
    emptying ``pending`` to widen the green would fail here.
    """
    sim = _mujoco()
    assert {it.id for it in sim.pending_for(task)} == expect
    assert sim.publishable_for(task) is False


@pytest.mark.parametrize("task", [A1, R2])
def test_the_other_two_tasks_still_have_no_gate_on_record(task):
    """...and the second half is missing too, so neither could publish even
    if its pending list were emptied. The two checks are independent on
    purpose."""
    assert _gates(task, ARM) == {}
    state, _why = readiness._discriminating(task, ARM)
    assert state == readiness.NO


def test_a_pending_sentence_about_a_MISSING_gate_is_not_read_as_a_gate():
    """The false green this change closed, pinned so it cannot come back.

    ``readiness._discriminating`` used to fall back to the arm's free-text
    ``pending`` and accept the cell when the text contained "gate" and the
    task id. The MuJoCo arm's own sentence was "A1_husky_swarm_10 and
    R2_arm_reach are still UNGATED" -- which contains "gate" -- so both cells
    reported as DISCRIMINATING while saying in prose that they were not.
    Measured over the 9x3 grid the fallback produced two greens, (A1, mujoco)
    and (R2, mujoco), and no true positives.

    The text below is the shape that used to fool it. Whatever an arm writes
    in ``pending``, only a recorded verdict may satisfy this gate.
    """
    sim = _mujoco()
    prose = sim.pending or ""
    assert "gate" in prose.lower() and A1 in prose, (
        "this test is only meaningful while the pending text still mentions "
        "a gate and this task; it does not: %r" % prose)
    state, why = readiness._discriminating(A1, ARM)
    assert state == readiness.NO, why
    assert "sims.pending" not in why


# --- the scoping mechanism itself -------------------------------------------

def test_narrowing_a_scope_without_a_reason_is_refused():
    """A scope is an argument that other tasks are immune. Unstated, it is
    just an assertion that something is fine."""
    with pytest.raises(ValueError) as exc:
        sims.Pending("x", blocks=(A1,), detail="a gap")
    assert "why_scoped" in str(exc.value)
    # ...and with one it is accepted.
    ok = sims.Pending("x", blocks=(A1,), detail="a gap",
                      why_scoped="A1 only, because ...")
    assert ok.blocks_task(A1) and not ok.blocks_task(R1)


def test_an_item_that_blocks_nothing_is_refused():
    """``blocks=()`` would be a pending item with no effect -- a gap recorded
    in a field that no longer reads it. If it truly blocks nothing it should
    be closed, not emptied."""
    with pytest.raises(ValueError):
        sims.Pending("x", blocks=(), detail="a gap", why_scoped="...")


def test_an_unscoped_item_still_blocks_everything():
    """The default and the compatibility path both stay conservative: an item
    nobody scoped blocks every task, and a bare string (which is what this
    field held before) is read as exactly that."""
    wide = sims.Pending("x", detail="a gap")
    assert wide.blocks_task(R1) and wide.blocks_task(A1)

    legacy = sims.Sim("t", name="T", role="r", tier="extended",
                      status="implemented", surface="s",
                      pending="something is outstanding")
    assert legacy.pending_for(R1) and legacy.publishable_for(R1) is False
    assert legacy.publishable is False


def test_a_task_this_arm_cannot_express_is_not_publishable_by_default():
    """An unexpressible cell has no number, so 'nothing pending' must not read
    as 'ready to publish'. B1 ships a .wbt fixture with no MJCF equivalent."""
    sim = _mujoco()
    assert sim.expresses("B1_overlap_audit") is False
    assert sim.publishable_for("B1_overlap_audit") is False


def test_the_whole_arm_claim_was_not_weakened():
    """``publishable`` (no task) still means "nothing outstanding ANYWHERE".

    The per-task split must not have quietly upgraded the arm-level claim:
    this arm has three open items and is not a finished arm.
    """
    sim = _mujoco()
    assert sim.publishable is False
    assert len(sim.pending_items) == 3
    assert sim.pending and "A1_gate" in sim.pending


def test_every_pending_item_names_its_evidence():
    """A gap the reader cannot go and check is a claim, not a record."""
    for it in _mujoco().pending_items:
        assert it.evidence, it.id
        assert it.detail, it.id


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
