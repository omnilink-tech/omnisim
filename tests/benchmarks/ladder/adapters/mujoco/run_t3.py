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

"""**Is T3 achievable at all?** One description -> a graded verdict, no agent.

One command, no network, no GPU, no tokens::

    python tests/benchmarks/ladder/adapters/mujoco/run_t3.py --out <dir>

``tasks/T3_quadruped/meta.json`` -> ``container.authored_here.before_the_freeze``
makes settling it a precondition of the freeze: *"Demonstrate the walk once, by
hand, on at least one column, and publish the recipe alongside the scaffolding.
If it cannot be demonstrated anywhere, the tier is unachievable-as-shipped and
that is a finding about this file, not about any simulator."*

**Why this file exists rather than the scratch script that came first.** The
walk *was* demonstrated on 2026-08-02 -- and from a harness outside this tree,
so by the standing *"no row, no result"* rule
(``agent-edge-validation-plan.md`` §0.2) the numbers were a claim about this
container that a reader could not re-derive from the repository. That is the
weaker of the two states and the task file said so. This file is that gap
closed.

It does four things and claims exactly those four:

1. assembles the shipped description into a MuJoCo scene
   (:mod:`ladder.adapters.mujoco.t3_scene`) -- the deliverable;
2. re-runs that deliverable **cold and standalone** with the grader's own
   sampler and no agent (:mod:`ladder.adapters.mujoco.runner_t3`) -- phase B;
3. runs both contract checks (``agentbench.adapters.check_bundle`` and
   ``ladder.graders.t3.check_evidence``) against what comes back;
4. grades it through **the real T3 path** -- ``ladder.graders.t3.grade``, which
   hands the sim-neutral core a :class:`T3Evidence` and nothing MuJoCo-shaped.

**It is not a ladder cell and its verdict is not a result.** A cell is an
autonomous agent given one sentence and no help
(``capability-ladder-plan.md`` §2); this is a scripted control that a human
wrote knowing the thresholds. Its only claims are *"the task is achievable"*,
*"the outcome is reachable with no learning at all"* and *"the instrument works
on this column"*.

``--skip-run`` re-grades an existing run directory from the artifacts on disk,
which is how a repaired grader is re-applied to a finished walk **without
re-simulating it** -- the same discipline the T2.3 repair was re-checked under.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder import tasks as ladder_tasks                          # noqa: E402
from ladder.adapters.mujoco import runner_t3, t3_drive, t3_scene  # noqa: E402
from ladder.graders import t3 as t3_grader                        # noqa: E402

TASK_ID = "T3_quadruped"
HERE = Path(__file__).resolve().parent


def build_deliverable(out_dir, *, task=None, ground="box", force_limits=True):
    """Assemble the scene and its driver into ``<out>/deliverable/``.

    The driver is **copied**, not imported: what phase B re-runs must be the
    files on disk, so a deliverable that only works because the benchmark's own
    package happened to be importable would be caught here rather than passing.
    """
    task = task or ladder_tasks.get(TASK_ID)
    out = Path(out_dir)
    deliverable = out / "deliverable"
    deliverable.mkdir(parents=True, exist_ok=True)

    res = t3_scene.build_t3_scene(task.container_dir, out / "workspace",
                                  ground=ground, force_limits=force_limits)
    t3_scene.write_build_record(res, out / "build.json")
    for p in res.problems:
        print("build problem: %s" % p, file=sys.stderr)
    if not res.scene:
        return None, res
    shutil.copyfile(res.scene, deliverable / runner_t3.SCENE_NAME)
    shutil.copyfile(HERE / "t3_drive.py", deliverable / runner_t3.DRIVER_NAME)
    return deliverable, res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, help="the run directory to write")
    ap.add_argument("--deliverable", default=None,
                    help="grade an existing deliverable instead of building "
                         "one")
    ap.add_argument("--skip-run", action="store_true",
                    help="grade an existing run directory, do not re-run")
    ap.add_argument("--ground", default="box", choices=("box", "plane"),
                    help="the floor's geometry. 'box' is the default and is "
                         "what makes the arena channel a readable number; "
                         "'plane' is the recorded recipe's own choice and is "
                         "kept so the two can be compared")
    ap.add_argument("--unlimited-actuators", action="store_true",
                    help="do NOT clamp the actuators to the URDF's declared "
                         "efforts. This reproduces the recorded recipe, under "
                         "which the gait peaks at 95.4 N.m on a 30 N.m thigh "
                         "joint; the default is the stricter setting")
    ap.add_argument("--speed", type=float, default=None,
                    help="the commanded body speed in m/s, passed to the "
                         "deliverable's driver. --speed 0.27 is the slow "
                         "statically-stable walk whose 0.0018 m bob retired "
                         "T3.3's vertical-bob pass condition")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--json", help="write the verdict as JSON to this path")
    a = ap.parse_args(argv)

    task = ladder_tasks.get(TASK_ID)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    phase = task.standalone
    if a.speed is not None:
        # The child process inherits this; the driver reads it and records the
        # value it used in the run, so a re-grade can see which speed produced
        # the row rather than having to be told.
        os.environ[t3_drive.SPEED_ENV] = "%g" % float(a.speed)
        print("commanded body speed: %g m/s (%s)"
              % (float(a.speed), t3_drive.SPEED_ENV))

    deliverable = Path(a.deliverable) if a.deliverable else None
    if deliverable is None and not a.skip_run:
        deliverable, build = build_deliverable(
            out, task=task, ground=a.ground,
            force_limits=not a.unlimited_actuators)
        if deliverable is None:
            print("the deliverable was not built; see build.json",
                  file=sys.stderr)
            return 2
        print("built: %d bodies, %.3f kg compiled robot, %d actuators, "
              "%d build problems, ground=%s"
              % (build.bodies, build.mass_kg_compiled, len(build.actuators),
                 len(build.problems), build.ground.get("kind")))
    if deliverable is None:
        deliverable = out / "deliverable"

    res = None
    if not a.skip_run:
        res = runner_t3.run_standalone(
            deliverable, out,
            duration=float(a.duration if a.duration is not None
                           else phase.get("duration_s", 180.0)),
            settle=float(phase.get("settle_s", 1.0)),
            stride=int(phase.get("contact_stride", 2)),
            surfaces=task.surfaces, base=task.robot_name)
        print("phase B: exit=%s wall=%.2fs timed_out=%s"
              % (res.rc, res.wall_s or 0.0, res.timed_out))

    # --- contract checks, against what the run actually produced ---------
    ev = t3_grader.build_evidence(
        task, sim="mujoco", artifact=str(deliverable / runner_t3.SCENE_NAME),
        run_dir=out, phase_b=res)
    problems = t3_grader.check_evidence(ev)
    print("\n=== contract checks ===")
    if problems:
        for p in problems:
            print("  ! %s" % p)
    else:
        print("  clean: agentbench.check_bundle + ladder.check_t3_evidence "
              "report nothing")

    # --- the real grading path -------------------------------------------
    verdict = t3_grader.grade(out, task=task,
                              artifact=str(deliverable / runner_t3.SCENE_NAME),
                              phase_b=res, sim="mujoco")
    print("\n=== T3 verdict (a SCRIPTED control, not a ladder cell) ===")
    print(verdict.summary())
    print("")
    # Never omitted: capability-ladder-plan.md §8 forbids quoting a T3 cell
    # without these, so neither may the tool that prints the cell.
    print("support_attestation:  %s"
          % verdict.measurements.get("support_attestation"))
    print("external_support:     %s"
          % json.dumps(verdict.measurements.get("external_support"),
                       default=str))
    print("method:               %s" % verdict.measurements.get("method"))
    print("reuse_class:          %s -- NOT decidable by a grader (§9 Q3: the "
          "reviewer arbitrates). A cell published with this null is not "
          "publishable." % verdict.measurements.get("reuse_class"))
    print("termination:          %s"
          % json.dumps(verdict.measurements.get("termination"), default=str))
    print("distance_to_termination_m: %s"
          % verdict.measurements.get("distance_to_termination_m"))
    gaps = verdict.measurements.get("unanswered_channels")
    print("unanswered channels:  %s" % (gaps if gaps else "none"))
    vac = {a_.id: [f.clause for f in a_.vacuous_clauses]
           for a_ in verdict.assertions if a_.vacuous}
    print("vacuous clauses:      %s" % (vac if vac else "none"))
    _print_the_effort_the_walk_actually_cost(out)

    if a.json:
        Path(a.json).write_text(
            json.dumps({"verdict": verdict.as_dict(),
                        "contract_problems": problems,
                        "phase_b": (res.as_dict() if res else None),
                        "provenance": ev.provenance()}, indent=2, default=str),
            encoding="utf-8")
    return 0 if verdict.outcome == "PASS" else 1


def _print_the_effort_the_walk_actually_cost(run_dir):
    """Peak actuator force per joint against the effort the URDF declares.

    Printed rather than asserted: the recorded recipe left the actuators
    unlimited, so *"did the walk stay inside the robot's own declared limits"*
    is a measurement about the run and it belongs beside the verdict rather
    than in a footnote. A joint over its declared effort is a real finding and
    it should be visible without opening a JSON file.
    """
    try:
        doc = json.loads((Path(run_dir) / "t3.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return
    peaks = doc.get("actuator_peak_force") or {}
    efforts = doc.get("declared_efforts_nm") or {}
    if not peaks:
        return
    rows, over = [], []
    for act, peak in sorted(peaks.items()):
        joint = act[4:] if act.startswith("pos_") else act
        eff = efforts.get(joint)
        rows.append("%s %.2f/%s" % (joint, peak,
                                    ("%.0f" % eff) if eff else "?"))
        if eff and peak > float(eff):
            over.append("%s (%.2f > %.0f N.m)" % (joint, peak, eff))
    print("peak |actuator force| vs the URDF's declared effort, N.m:")
    print("  " + "  ".join(rows))
    print("  %s" % ("ALL INSIDE the declared efforts" if not over else
                    "OVER the declared effort: " + ", ".join(over)))


if __name__ == "__main__":
    sys.exit(main())
