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

"""Is a (task, arm) actually ready to be SCORED? Answer from the tree, not opinion.

    python tests/benchmarks/agentbench/readiness.py --task R1_lidar_nav

Every gate below has a story behind it from a day where the answer was assumed
and turned out wrong:

* **expressible** -- MuJoCo can express three of the nine tasks; the rest ship a
  ``.wbt`` fixture with no MJCF equivalent. A missing FIXTURE scored as a
  failing run blames a simulator for our asset gap (SPEC 6.4).
* **deliverable** -- a world collected WITHOUT the thing that drives it grades a
  robot that cannot move. Measured twice: OmniSim's R1 collected no controller
  (path 0.0 m), and MJCF is inert data so an ``.xml`` without its driver is the
  same bug in a new spelling.
* **decidable** -- an assertion that CANNOT PASS is as broken as one that cannot
  fail. R1.3 was scored against two arms all day while their name-keyed bounds
  scan made it unpassable whatever the agent built.
* **discriminating** -- the oracle/null gate (SPEC 7.1). Without it a pass rate
  is unfalsifiable: C2 shipped a world whose UNFIXED form passed 5/5 for a whole
  campaign because nobody had asserted it could fail. Read ONLY from the
  recorded verdicts: a prose fallback that sniffed the arm's ``pending`` text
  used to satisfy this, and it read the sign backwards -- "still UNGATED"
  contains "gate", so it reported two cells as gated on the strength of a
  sentence saying they were not.
* **publishable** -- the arm's own remaining bring-up, FOR THIS TASK
  (``sims.Pending``). Per task because the debt is: the MuJoCo arm's two open
  items are about A1 and R2, and while this asked the arm-wide flag they made
  R1 unpublishable on an arm whose R1 gate was green and pinned by a test.
  Never on its own a licence to publish -- ``main`` ANDs it with
  **discriminating**, so scoping an item cannot green a cell that has no gate.
* **bounded** -- the cell wall ceiling, enforced and REGRESSION-TESTED rather
  than asserted. Three separate budget bugs shipped in one day, each because the
  stated guarantee was stronger than the enforced one.

A gate that cannot be checked mechanically reports UNKNOWN, never OK. "We could
not tell" and "we checked and it was fine" are different claims and only one of
them supports spending money.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))

from agentbench import sims, tasks                      # noqa: E402
from agentbench.agents import external as external_agent  # noqa: E402

OK, NO, UNKNOWN = "OK", "NO", "??"


def _deliverable(task_id, sim):
    """Does this arm have a deliverable convention for this task?"""
    # B1/B3 are answer tasks: the agent's final message IS the artifact.
    # ``artifact_name`` correctly returns None for them because no world is
    # collected, but treating that as "not deliverable" made the readiness
    # table say two implemented tasks were unrunnable on every simulator.
    if task_id in external_agent.ANSWER_TASKS:
        return OK, "final answer text"
    try:
        name = external_agent.artifact_name(task_id, sim)
    except Exception as exc:                              # noqa: BLE001
        return UNKNOWN, repr(exc)
    if not name:
        return NO, "no artifact name for (%s, %s)" % (task_id, sim)
    return OK, name


def _discriminating(task_id, sim):
    """Has the oracle/null gate been shown green for THIS (task, arm)?

    Read from the recorded verdicts, never inferred from an arm-level flag.
    My first version of this asked ``sims.publishable`` -- an ARM property --
    and cheerfully reported R1 as gated on OmniSim and Webots when the gate has
    never been run for R1 on either. The gate is per (task, arm): an arm whose
    older tasks discriminate says nothing about a task added later.
    """
    import json
    vp = HERE / "preregister" / "oracle_verdicts.json"
    if not vp.is_file():
        return UNKNOWN, "no oracle_verdicts.json"
    try:
        doc = json.loads(vp.read_text(encoding="utf-8"))
        # Both instruments count: scripted-LLM cells in `cells`, and
        # driver gates in `driver_gates` (kept apart because the former
        # is regenerated wholesale and would delete the latter).
        cells = list(doc.get("cells") or [])
        cells += list((doc.get("driver_gates") or {}).get("cells") or [])
    except (OSError, ValueError) as exc:
        return UNKNOWN, "verdicts unreadable: %r" % (exc,)
    mine = [c for c in cells
            if c.get("task") == task_id and c.get("sim") == sim]
    if not mine:
        # There used to be a fallback here: if the arm's free-text `pending`
        # contained "gate" and the task id, the gate was taken as recorded
        # elsewhere. It read the wrong sign. `pending` is where an arm says
        # what it is MISSING, so the MuJoCo arm's own sentence "A1_husky_
        # swarm_10 and R2_arm_reach are still UNGATED" matched ("ungated"
        # contains "gate") and reported both cells as DISCRIMINATING -- the
        # exact claim the text was making the opposite of. Measured over the
        # 9x3 grid it produced two greens, (A1, mujoco) and (R2, mujoco), and
        # not one true positive: every arm whose gate has actually been run
        # has it in the verdicts file, MuJoCo's R1 included. A prose sniff
        # cannot tell "we ran it" from "we have not run it", so it is gone
        # and the record is the only evidence this gate accepts.
        return NO, ("no oracle/null gate on record for (%s, %s) -- the task "
                    "has never been shown to discriminate on this arm"
                    % (task_id, sim))
    outcomes = {c.get("agent"): c.get("outcome") for c in mine}
    if outcomes.get("oracle") == "PASS" and outcomes.get("null") == "FAIL":
        return OK, "oracle PASS + null FAIL on record"
    return NO, "gate cells present but not oracle=PASS/null=FAIL: %s" % outcomes


def _red_evidence(task_id):
    """Has every assertion for this task been observed failing honestly?

    The generated coverage table is a publication gate, not merely a report.
    A task can have an oracle PASS and a null FAIL while one of its individual
    assertions is still vacuous.  That is exactly why the red-evidence rule
    exists (validation plan 5.5), so readiness must not omit it.
    """
    import json
    path = HERE / "phase0_validation" / "coverage.json"
    if not path.is_file():
        return UNKNOWN, "no generated red-evidence coverage.json"
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return UNKNOWN, "coverage unreadable: %r" % (exc,)
    rows = [r for r in (doc.get("rows") or [])
            if r.get("task") == task_id]
    if not rows:
        return NO, "no assertion coverage rows for %s" % task_id
    bad = [r for r in rows if not r.get("validated")]
    if not bad:
        return OK, "%d/%d assertions have non-null red evidence" % (
            len(rows), len(rows))
    names = ", ".join(str(r.get("assertion") or "?") for r in bad)
    return NO, "%d/%d assertions unvalidated: %s" % (
        len(bad), len(rows), names)


def _publishable(task_id, s):
    """Is this arm's remaining bring-up clear FOR THIS TASK (SPEC 6.2)?

    Per task, because the debt is. This asked ``s.publishable`` -- "nothing
    outstanding anywhere on this arm" -- and printed it in a table whose header
    names one task, so the MuJoCo arm read NO for R1 on account of two items
    about A1 and R2 while R1's own gate was green and pinned by a test. A
    column that cannot go green for a reason that has nothing to do with its
    row teaches readers to ignore it, and the fix people reach for is to delete
    the blocking text, which loses a real gap.

    It is deliberately NOT the whole bar. This says the registry knows of
    nothing outstanding; ``discriminating`` says the oracle/null gate has been
    shown green on this cell, and ``main`` ANDs them. Neither alone may
    publish a number: without the gate a pass rate is unfalsifiable (C2), and
    without this an arm publishes over its own declared bring-up.
    """
    if s.publishable_for(task_id):
        return OK, ""
    if not s.expresses(task_id):
        return NO, "arm cannot express this task, so there is nothing to publish"
    blocking = s.pending_for(task_id)
    return NO, "; ".join("%s: %s" % (it.id, it.detail) for it in blocking)


def check(task_id):
    task = tasks.get(task_id)
    rows = []
    for sim_id in sims.IMPLEMENTED:
        s = sims.get(sim_id)
        gates = {}
        gates["expressible"] = ((OK, "") if s.expresses(task_id)
                                else (NO, "fixture missing for this arm"))
        gates["deliverable"] = _deliverable(task_id, sim_id)
        gates["red_evidence"] = _red_evidence(task_id)
        gates["discriminating"] = _discriminating(task_id, sim_id)
        gates["publishable"] = _publishable(task_id, s)
        rows.append((sim_id, gates))
    return task, rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--task", default="R1_lidar_nav")
    args = ap.parse_args(argv)

    task, rows = check(args.task)
    ceiling = tasks.TASK_HARD_CEILING_S
    try:
        from agentbench.cc_lane.run_cc_cell import cell_wall_bound_s as bound_f
    except Exception:                                     # noqa: BLE001
        bound_f = None

    print("task %s -- par %ss, budget %ss, cell wall bound %s"
          % (task.id, task.par_s, task.timeout_s,
             ("%ss" % int(bound_f(task.timeout_s))) if bound_f
             else "UNBOUNDED"))
    # The protocol is ONE run per (task, arm) under that ceiling, so the line
    # above is the whole cost of scoring this task on one arm -- not a fifth
    # of it. It is also the whole EVIDENCE: one cell estimates no variance,
    # and this script's verdicts say a task is ready to be scored, never that
    # a score from it is a rate (SPEC 3.5).
    print("        ceiling %ss, ONE run per (task, arm) -- an outcome, "
          "not a rate (variance unmeasured)" % ceiling)
    print()
    names = ("expressible", "deliverable", "red_evidence",
             "discriminating", "publishable")
    print("%-10s %s" % ("arm", "  ".join("%-14s" % n for n in names)))
    print("-" * 74)
    runnable, publishable = [], []
    for sim_id, gates in rows:
        cells = "  ".join("%-14s" % gates[n][0] for n in names)
        print("%-10s %s" % (sim_id, cells))
        for n in names:
            state, why = gates[n]
            if state != OK and why:
                print("%-10s   %s: %s" % ("", n, why))
        if all(gates[n][0] == OK for n in ("expressible", "deliverable")):
            runnable.append(sim_id)
        # PUBLISHABLE for a TASK needs both: the arm has nothing pending AND
        # this task's gate is green on it. `publishable` alone is an ARM
        # property -- reporting it per task listed OmniSim as publishable for
        # R1 while R1's oracle half was blocked by an engine defect, which is
        # exactly the conflation this file was written to stop making.
        if (gates["publishable"][0] == OK
                and gates["discriminating"][0] == OK
                and gates["red_evidence"][0] == OK):
            publishable.append(sim_id)
    print()
    print("RUNNABLE for %s     : %s" % (task.id, ", ".join(runnable) or "none"))
    print("PUBLISHABLE for %s  : %s"
          % (task.id, ", ".join(publishable) or "none"))
    if bound_f is None:
        print()
        print("WARNING: no cell wall bound found -- a cell's total cost is "
              "unbounded, and 'cells x ceiling' is not a real limit.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
