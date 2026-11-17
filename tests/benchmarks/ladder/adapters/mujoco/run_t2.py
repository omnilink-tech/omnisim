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

"""**Is T2 achievable at all?** Three descriptions -> a graded verdict, no agent.

One command, no network, no GPU, no tokens::

    python tests/benchmarks/ladder/adapters/mujoco/run_t2.py --out <dir>

``tasks/T2_transfer/container/PROVENANCE.txt`` says it plainly: *"The arm has
never been demonstrated completing this task on any simulator … if it turns out
to be unachievable everywhere then the grid measured the asset and not the
agents."* ``meta.json`` → ``container.authored_here.before_the_freeze`` makes
settling it a precondition of the freeze. This file settles it, the same way
T1's credibility was established (``BRINGUP.md`` §4.1: a scripted run graded
PASS 5/5 before any agent touched the tier).

It does four things and claims exactly those four:

1. assembles the three shipped descriptions into a MuJoCo scene
   (:mod:`ladder.adapters.mujoco.t2_scene`) -- the deliverable;
2. re-runs that deliverable **cold and standalone** with the grader's own
   sampler and no agent (:mod:`ladder.adapters.mujoco.runner_t2`) -- phase B;
3. runs both contract checks (``agentbench.adapters.check_bundle`` and
   ``ladder.graders.check_t2_evidence``) against what comes back;
4. grades it through **the real T2 path** -- ``ladder.graders.t2.grade``, which
   resolves this column through ``ladder.adapters.build_t2_evidence``, so
   nothing MuJoCo-shaped reaches the sim-neutral core.

**It is not a ladder cell and its verdict is not a result.** A cell is an
autonomous agent given one sentence and no help (``capability-ladder-plan.md``
§2); this is a scripted control that a human wrote knowing the thresholds. Its
only claims are *"the task is achievable"* and *"the instrument works on this
column"* -- and the first of those is the one the freeze was waiting on.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder import adapters as ladder_adapters                    # noqa: E402
from ladder import tasks as ladder_tasks                          # noqa: E402
from ladder.adapters.mujoco import runner_t2, t2_scene            # noqa: E402
from ladder.graders import t2 as t2_grader                        # noqa: E402

TASK_ID = "T2_transfer"
HERE = Path(__file__).resolve().parent


def build_deliverable(out_dir, *, task=None, radius=t2_scene.WORK_RADIUS):
    """Assemble the scene and its driver into ``<out>/deliverable/``.

    The driver is **copied**, not imported: what phase B re-runs must be the
    files on disk, so a deliverable that only works because the benchmark's own
    package happened to be importable would be caught here rather than passing.
    """
    task = task or ladder_tasks.get(TASK_ID)
    out = Path(out_dir)
    deliverable = out / "deliverable"
    deliverable.mkdir(parents=True, exist_ok=True)

    res = t2_scene.build_t2_scene(task.container_dir, out / "workspace",
                                  radius=radius)
    t2_scene.write_build_record(res, out / "build.json")
    for p in res.problems:
        print("build problem: %s" % p, file=sys.stderr)
    if not res.scene:
        return None, res
    scene = deliverable / runner_t2.SCENE_NAME
    shutil.copyfile(res.scene, scene)
    shutil.copyfile(HERE / "t2_drive.py", deliverable / runner_t2.DRIVER_NAME)
    return deliverable, res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, help="the run directory to write")
    ap.add_argument("--deliverable", default=None,
                    help="grade an existing deliverable instead of building "
                         "one")
    ap.add_argument("--skip-run", action="store_true",
                    help="grade an existing run directory, do not re-run")
    ap.add_argument("--radius", type=float, default=t2_scene.WORK_RADIUS)
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--json", help="write the verdict as JSON to this path")
    a = ap.parse_args(argv)

    task = ladder_tasks.get(TASK_ID)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    phase = task.standalone

    deliverable = Path(a.deliverable) if a.deliverable else None
    if deliverable is None and not a.skip_run:
        deliverable, build = build_deliverable(out, task=task,
                                               radius=a.radius)
        if deliverable is None:
            print("the deliverable was not built; see build.json",
                  file=sys.stderr)
            return 2
        print("built: %d bodies, %.3f kg compiled, %d actuators, %d build "
              "problems" % (build.bodies, build.mass_kg_compiled,
                            len(build.actuators), len(build.problems)))
    if deliverable is None:
        deliverable = out / "deliverable"

    res = None
    if not a.skip_run:
        res = runner_t2.run_standalone(
            deliverable, out,
            duration=float(a.duration if a.duration is not None
                           else phase.get("duration_s", 60.0)),
            settle=float(phase.get("settle_s", 0.5)),
            stride=int(phase.get("contact_stride", 10)),
            surfaces=task.surfaces)
        print("phase B: exit=%s wall=%.2fs timed_out=%s"
              % (res.rc, res.wall_s or 0.0, res.timed_out))

    # --- contract checks, against what the run actually produced ---------
    ev = ladder_adapters.build_t2_evidence(
        task, sim="mujoco", artifact=str(deliverable / runner_t2.SCENE_NAME),
        run_dir=out, phase_b=res)
    # Both contracts at once -- the shim already prefixes agentbench's own
    # findings with "bundle:" and appends the ladder's.
    problems = ladder_adapters.check_t2_evidence(ev)
    print("\n=== contract checks ===")
    if problems:
        for p in problems:
            print("  ! %s" % p)
    else:
        print("  clean: agentbench.check_bundle + ladder.check_t2_evidence "
              "report nothing")

    # --- the real grading path -------------------------------------------
    verdict = t2_grader.grade(out, task=task,
                              artifact=str(deliverable / runner_t2.SCENE_NAME),
                              phase_b=res, sim="mujoco")
    print("\n=== T2 verdict (a SCRIPTED control, not a ladder cell) ===")
    print(verdict.summary())
    print("")
    # Never omitted: no prose sentence about a T2 cell may leave it out.
    print("hold_mechanism: %s" % verdict.measurements.get("hold_mechanism"))
    gaps = verdict.measurements.get("unanswered_channels")
    print("unanswered channels: %s" % (gaps if gaps else "none"))

    if a.json:
        Path(a.json).write_text(
            json.dumps({"verdict": verdict.as_dict(),
                        "contract_problems": problems,
                        "phase_b": (res.as_dict() if res else None),
                        "provenance": ev.provenance()}, indent=2, default=str),
            encoding="utf-8")
    return 0 if verdict.outcome == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
