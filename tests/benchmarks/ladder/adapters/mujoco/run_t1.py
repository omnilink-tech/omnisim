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

"""End to end: the T1 container -> a MuJoCo run -> a graded verdict.

One command, no agent, no network, no GPU::

    python tests/benchmarks/ladder/adapters/mujoco/run_t1.py --out <dir>

It exists to prove the column's plumbing, and it proves exactly three things:

1. the raw URDF the T1 container ships can be brought into MuJoCo and driven
   (:mod:`ladder.adapters.mujoco.model_build`, :mod:`~.runner`);
2. what comes back parses into the neutral
   :class:`~agentbench.graders.evidence.EvidenceBundle` **and passes
   ``agentbench.adapters.check_bundle``** plus the ladder's own
   ``check_t1_evidence``;
3. the sim-neutral T1 core grades it without a single MuJoCo-shaped field
   reaching it.

**It is not a ladder cell and its verdict is not a result.** A cell is an
autonomous agent given one sentence (``capability-ladder-plan.md`` §2); this is
a scripted control that a human wrote knowing the thresholds. Its only claim is
"the instrument works on this column".

Where the commanded point comes from
------------------------------------

From the task file, never from the run. ``ladder/tasks/T1_arrive/meta.json``
declares a **start-relative** waypoint (``offset_m: [0, 5]``, north = +y) and
the resolution rule that "where it starts" means the graded body's position at
the *first recorded sample*. This driver resolves it exactly that way and cites
the task file, because an adapter that could report the goal could report the
goal it observed the robot at (``ladder.graders.ladder_evidence``).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench import adapters as ab_adapters                    # noqa: E402
from ladder import tasks as ladder_tasks                          # noqa: E402
from ladder.adapters.mujoco import evidence as mj_evidence        # noqa: E402
from ladder.adapters.mujoco import recording, runner              # noqa: E402
from ladder.graders import ladder_evidence as lev                 # noqa: E402

TASK_ID = "T1_arrive"


def build_t1_evidence(run_dir, *, task=None, task_id=TASK_ID):
    """One MuJoCo run directory -> :class:`~ladder.graders.ladder_evidence.T1Evidence`.

    Self-contained on purpose. ``ladder.adapters.build_t1_evidence`` is the
    shared front door and this column should route through it, but that
    function resolves the bundle builder through
    ``agentbench.adapters.resolve()``, whose registry points ``mujoco`` at
    ``agentbench.adapters.mujoco.evidence`` -- a module that does not exist in
    this tree, because this column lives entirely in the ladder namespace. See
    ``BRINGUP.md`` §5: that seam is one three-line forwarder away from closing
    and it belongs to whoever owns the agentbench registry, not to this file.
    """
    task = task or ladder_tasks.get(task_id)
    run = recording.read_run(run_dir)
    bundle = mj_evidence.build_bundle(task.id, robot_identity="any_robot",
                                      live_expected=False, run=run)
    support = mj_evidence.support_observation(run=run)
    ev = lev.T1Evidence(bundle=bundle, support=support,
                        robot_name=task.robot_name)
    ev.waypoint = _waypoint(task, ev)
    return ev


def _waypoint(task, ev):
    """The commanded point, resolved against the run's own first sample."""
    from ladder.graders import t1_core
    cite = ladder_tasks.waypoint_citation(task)
    spec = task.waypoint_spec
    if spec.get("frame") != "start_relative":
        return lev.Waypoint(xy=ladder_tasks.resolve_waypoint(task, (0.0, 0.0)),
                            label="the commanded point", source=cite)
    index, body, _ = t1_core.select_robot(ev)
    row, _why = t1_core.series_row_for(ev, index, body)
    traj = ev.bundle.trajectory
    if row is None or traj is None or traj.xyz is None:
        return lev.Waypoint(
            source="", label="unresolved: the run produced no pose series for "
                             "the graded body")
    start = traj.xyz[row][0][:2]
    return lev.Waypoint(xy=ladder_tasks.resolve_waypoint(task, start),
                        label="the commanded point", source=cite)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, help="the run directory to write")
    ap.add_argument("--description",
                    help="override the robot package (default: the task's "
                         "own container/)")
    ap.add_argument("--skip-run", action="store_true",
                    help="grade an existing run directory, do not re-run")
    ap.add_argument("--json", help="write the verdict as JSON to this path")
    a = ap.parse_args(argv)

    task = ladder_tasks.get(TASK_ID)
    out = Path(a.out)
    phase = task.standalone
    desc = a.description or (task.container_dir
                             / Path(task.meta["container"]["description_dir"]
                                    ).name)

    if not a.skip_run:
        proc = runner.launch(
            out, goal_offset=tuple(task.waypoint_spec.get("offset_m",
                                                          [0.0, 5.0])),
            description=str(desc),
            extra_args=["--settle-s", str(phase.get("settle_s", 0.5)),
                        "--drive-timeout-s", str(phase.get("duration_s", 60.0)),
                        "--contact-stride",
                        str(phase.get("contact_stride", 10))])
        print("process: exit=%s wall=%.2fs timed_out=%s"
              % (proc.get("exit_code"), proc.get("wall_s") or 0.0,
                 proc.get("timed_out")))

    ev = build_t1_evidence(out, task=task)

    problems = (["bundle: %s" % p
                 for p in ab_adapters.check_bundle(ev.bundle)]
                + lev.check_t1_evidence(ev))
    print("\n=== contract checks ===")
    if problems:
        for p in problems:
            print("  ! %s" % p)
    else:
        print("  clean: agentbench.check_bundle + ladder.check_t1_evidence "
              "report nothing")

    from ladder.graders import t1_core
    verdict = t1_core.grade(ev)
    print("\n=== T1 verdict (a SCRIPTED control, not a ladder cell) ===")
    print(verdict.summary())

    if a.json:
        Path(a.json).write_text(
            json.dumps({"verdict": verdict.as_dict(),
                        "contract_problems": problems,
                        "waypoint": ev.waypoint.as_dict(),
                        "provenance": ev.provenance()}, indent=2, default=str),
            encoding="utf-8")
    return 0 if not problems else 0


if __name__ == "__main__":
    sys.exit(main())
