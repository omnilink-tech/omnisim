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

"""L2's shim: re-run a deliverable on its column, then grade it neutrally.

The physical half of L2 asks the same question T1 does -- where did the base
end up, and when did it stop -- of the same robot on the same floor. So phase B
here IS the ladder's T1 phase B, per column, and the only new work is reading
two different numbers out of the same series and adding the loop clauses.

Reusing it is not a shortcut. It means an L2 row and a T1 row on the same
column were produced by the same sampler, so a difference between them is the
target and never the instrument.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from loopbench.graders import l2_core, loop_trace            # noqa: E402


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
        return l2_core.first_failure(self.clauses)

    def as_dict(self):
        return {"outcome": self.outcome,
                "assertions": {k: v.as_dict() for k, v in self.clauses.items()},
                "failed_assertion": self.failed_assertion,
                "measurements": dict(self.numbers),
                "notes": list(self.notes), "sim": self.sim}

    def summary(self):
        head = "L2 %s" % self.outcome
        rows = ["  %-5s %-4s %s" % (k, "PASS" if v.ok else "FAIL", v.detail)
                for k, v in sorted(self.clauses.items())]
        return "\n".join([head] + rows)


# --- per-column series readers ----------------------------------------------


def _series_omnisim(phase_b_dir):
    """(t, xy) from the ladder recorder's wide-format motion file."""
    p = Path(phase_b_dir) / "phaseB.csv"
    if not p.is_file():
        return [], [], "no motion file: the sampler never attached"
    t, xy = [], []
    with open(p, newline="", encoding="utf-8", errors="replace") as fh:
        for row in csv.DictReader(fh):
            try:
                t.append(float(row["t"]))
                xy.append((float(row["r0_x"]), float(row["r0_y"])))
            except (KeyError, TypeError, ValueError):
                continue
    return t, xy, None


def _series_mujoco(phase_b_dir):
    """(t, xy) from the MuJoCo column's own recording."""
    try:
        from ladder.adapters.mujoco import recording
    except ImportError as exc:
        return [], [], "the column's recorder is not importable: %r" % (exc,)
    try:
        run = recording.read_run(str(phase_b_dir))
    except (OSError, ValueError, KeyError) as exc:
        return [], [], "the run directory could not be read: %r" % (exc,)
    traj = getattr(run, "trajectory", None) or getattr(run, "traj", None)
    if traj is None:
        return [], [], "the run carries no trajectory"
    ts = list(getattr(traj, "t", None) or getattr(traj, "times", None) or [])
    xyz = getattr(traj, "xyz", None)
    if xyz is None or not len(ts):
        return [], [], "the run's trajectory has no series"
    xy = [(float(row[0][0]), float(row[0][1])) for row in xyz]
    return [float(v) for v in ts], xy, None


_SERIES = {"omnisim": _series_omnisim, "mujoco": _series_mujoco}


def run_and_grade(deliverable, run_dir, *, task, sim, transcript=None,
                  phase_b=None):
    """Re-run ``deliverable`` cold on ``sim``, then grade it."""
    from ladder.graders import t1 as t1_shim          # phase B, per column

    notes = []
    reader = _SERIES.get(sim)
    if reader is None:
        return Verdict("ERROR", {}, {},
                       notes=["no L2 series reader for column %r" % sim],
                       sim=sim), None

    res = phase_b
    if res is None:
        runner, attr, refusal = _phase_b_runner(sim, task.rung)
        if refusal:
            return Verdict("ERROR", {}, {}, notes=[refusal], sim=sim), None
        phase = task.standalone
        res = runner(str(deliverable), str(run_dir),
                     duration=float(phase.get("duration_s", 30.0)),
                     settle=float(phase.get("settle_s", 0.5)))
        notes.append("phase B through %r (a cold standalone re-run with the "
                     "grader's own sampler and no agent)" % attr)
    del t1_shim

    t, xy, err = reader(run_dir)
    goal = None
    if t and xy:
        off = task.waypoint_spec.get("offset_m", [0.0, 5.0])
        goal = (xy[0][0] + float(off[0]), xy[0][1] + float(off[1]))

    trace = {"cycles": 0, "measurements": []}
    if transcript:
        trace = loop_trace.analyse(transcript)
        notes.append(trace["rule"])
    else:
        notes.append("no transcript supplied, so the loop clauses (L2.4, "
                     "L2.5) are graded against zero cycles: correct for a "
                     "SCRIPTED control, which does not iterate")

    outcome, clauses, numbers = l2_core.grade(
        t=t, xy=xy, goal_xy=goal or (0.0, 0.0),
        cycles=trace["cycles"], measurements_seen=trace["measurements"],
        constants=task.constants, run_error=err)
    numbers["column"] = sim
    return Verdict(outcome, clauses, numbers, notes=notes, sim=sim), res


def _phase_b_runner(sim, rung="L2"):
    """The column's L2 hook, through the tier machinery's own preference loop.

    Asked for by RUNG rather than pinned to T1's name: each column exposes
    ``l2_run_standalone`` explicitly -- an alias of T1's sampler on one, a
    purpose-built re-runner on the other -- so the lookup stays the same
    while what it finds is the column's own business.
    """
    from ladder.cell import run_ladder_cell as cell
    return cell.phase_b_runner(sim, rung)


def find_transcript(run_dir):
    """The session record beside a cell's run directory, or None.

    Looked up rather than passed in: the cell runner's grader call carries the
    deliverable and the phase-B directory and has no transcript argument, and
    threading one through every tier's signature to serve one rung would make
    the loop clauses look like the tier machinery's business. They are not --
    they are L2's, so L2 goes and finds its own evidence.

    ``run_agent_session`` writes it as ``<run_dir>/transcript.jsonl``, and the
    grade call receives ``<run_dir>/phase_b``, so the parent is checked too.
    """
    d = Path(run_dir)
    for cand in (d / "transcript.jsonl", d.parent / "transcript.jsonl",
                 d.parent / "forensics" / "session_transcript.jsonl"):
        if cand.is_file():
            return cand
    return None


def grade(run_dir, *, task, artifact=None, phase_b=None, sim=None, **kw):
    """The signature the cell runner calls."""
    tx = kw.get("transcript") or find_transcript(run_dir)
    v, _ = run_and_grade(artifact, run_dir, task=task, sim=sim,
                         transcript=tx, phase_b=phase_b)
    v.artifacts["transcript"] = str(tx) if tx else None
    return v
