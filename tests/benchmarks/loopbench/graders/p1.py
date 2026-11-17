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

"""P1's shim: re-run the deliverable on its column, read one body, grade it.

The agent BUILT the robot, so unlike every earlier rung there is no declared
body name to look for. The rule is therefore *"the body that moved the
furthest"*, which is stated in the verdict so a reader can see what was graded
and disagree with it.

That rule is deliberate rather than lazy. Picking "the first body" would grade
whatever the agent happened to author first; picking by name would require the
task to dictate names, which is a constraint on how the robot is built and
would make the columns' conventions part of the score.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from loopbench.graders import p1_core                       # noqa: E402

#: A cell whose run produced nothing is an INSTRUMENT outcome, never a column
#: result. The engine has a known intermittent cold-launch failure that drops
#: roughly one run in three with an empty log and exit code 1
#: (docs/developer/cold-launch-failure-2026-08-02.md), and it is indis-
#: tinguishable from the deliverable failing. Scoring it would attribute our
#: own defect to whatever was being measured.
NO_DATA = "NO_DATA"


class Verdict:
    def __init__(self, outcome, clauses, numbers, notes=None, sim=None):
        self.outcome = outcome
        self.clauses = clauses
        self.numbers = numbers
        self.notes = list(notes or [])
        self.sim = sim
        self.artifacts = {}

    # --- the interface ladder.cell.classify() reads ------------------------
    #
    # It asks a verdict for `.failed`, `.progress` and `.measurements` as well
    # as `.outcome`. A grader that answers only its own questions crashes the
    # CELL RUNNER after the agent has already run -- measured 2026-08-05, when
    # a finished webots session was lost at classification to
    # `AttributeError: 'Verdict' object has no attribute 'failed'`. The
    # deliverable survived and was re-graded by hand, but the cell did not.

    @property
    def failed(self):
        """Ids of the clauses that went red, for the tier machinery."""
        return [k for k, c in self.clauses.items() if not c.ok]

    @property
    def measurements(self):
        return dict(self.numbers)

    @property
    def progress(self):
        """0 nothing ran / 2 it ran and was graded / 3 every clause green.

        Deliberately coarse: the ladder reads `<= 1` as "the deliverable did
        not load" and `>= 3` as "it ran". A graded FAIL is neither, so it sits
        at 2 and is classified on its assertions rather than on this number.
        """
        if self.outcome in ("ERROR", "NO_DATA"):
            return 0
        return 3 if self.outcome == "PASS" else 2

    @property
    def failed_assertion(self):
        return p1_core.first_failure(self.clauses)

    def as_dict(self):
        return {"outcome": self.outcome,
                "assertions": {k: v.as_dict() for k, v in self.clauses.items()},
                "failed_assertion": self.failed_assertion,
                "measurements": dict(self.numbers),
                "notes": list(self.notes), "sim": self.sim}

    def summary(self):
        rows = ["  %-5s %-4s %s" % (k, "PASS" if v.ok else "FAIL", v.detail)
                for k, v in sorted(self.clauses.items())]
        return "\n".join(["P1 %s" % self.outcome] + rows)


def _wide_csv(phase_b_dir):
    """Every body's series from a wide-format ``t, r<N>_x/_y/_z`` motion file.

    Returns ``{index: (t, xy, z)}``. Both columns that write this format do so
    with the same header, which is why the same reader serves both.
    """
    p = Path(phase_b_dir) / "phaseB.csv"
    if not p.is_file():
        return {}, "no motion file: the run produced nothing"
    rows = []
    with open(p, newline="", encoding="utf-8", errors="replace") as fh:
        reader = csv.DictReader(fh)
        cols = reader.fieldnames or []
        idx = sorted({int(c.split("_")[0][1:]) for c in cols
                      if c.startswith("r") and "_" in c and c[1:].split("_")[0].isdigit()})
        for row in reader:
            rows.append(row)
    if not rows or not idx:
        return {}, "the motion file carries no body series"
    out = {}
    for i in idx:
        t, xy, z = [], [], []
        for row in rows:
            try:
                t.append(float(row["t"]))
                xy.append((float(row["r%d_x" % i]), float(row["r%d_y" % i])))
                z.append(float(row["r%d_z" % i]))
            except (KeyError, TypeError, ValueError):
                continue
        if len(t) > 2:
            out[i] = (t, xy, z)
    return out, (None if out else "no body series could be parsed")


def select_body(series):
    """``(index, why)`` for the body that travelled furthest."""
    best, best_len = None, -1.0
    for i, (_t, xy, _z) in series.items():
        d = p1_core.path_length(xy)
        if d > best_len:
            best, best_len = i, d
    return best, ("the body that travelled furthest (%.2f m); the agent built "
                  "the robot, so no name was prescribed" % best_len)


def grade(run_dir, *, task, artifact=None, phase_b=None, sim=None, **kw):
    series, err = _wide_csv(run_dir)
    if err:
        # NOT a FAIL. See NO_DATA above.
        clauses = {k: p1_core.Clause(k, False, err, vacuous=True)
                   for k, _ in p1_core._ALL}
        v = Verdict(NO_DATA, clauses, {"run_error": err},
                    notes=["the run produced no usable motion series, so this "
                           "cell measures the INSTRUMENT and not the column; "
                           "it must be re-run, never scored"], sim=sim)
        return v

    idx, why = select_body(series)
    t, xy, z = series[idx]
    outcome, clauses, numbers = p1_core.grade(t=t, xy=xy, z=z,
                                              constants=task.constants)
    numbers["graded_body"] = {"index": idx, "selection": why,
                              "bodies_in_run": len(series)}
    numbers["column"] = sim
    return Verdict(outcome, clauses, numbers,
                   notes=["graded from a cold standalone re-run with the "
                          "grader's own sampler and no agent present"], sim=sim)
