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

"""**Is T4 achievable at all?** One description -> a graded verdict, no agent.

One command, no network, no GPU, no tokens::

    python tests/benchmarks/ladder/adapters/mujoco/run_t4.py --out <dir>

``tasks/T4_humanoid/meta.json`` -> ``container.authored_here.before_the_freeze``
makes settling it a precondition of the freeze: *"Demonstrate the walk once, by
hand, on at least one column, and publish the recipe alongside the scaffolding.
If it cannot be demonstrated anywhere, the tier is unachievable-as-shipped and
that is a finding about this file, not about any simulator."*

**Why this file exists rather than the scratch script that came first.** The
walk *was* demonstrated on 2026-08-02 -- and from a harness outside this tree,
so by the standing *"no row, no result"* rule
(``agent-edge-validation-plan.md`` §0.2) the numbers were a claim about this
container that a reader could not re-derive from the repository. The task file
said so itself, in as many words: *"this tier's achievability is DEMONSTRATED
BUT NOT REPRODUCIBLE, and it should be read as the weaker of the two."* This
file is that gap closed.

It does four things and claims exactly those four:

1. assembles the shipped description into a MuJoCo scene
   (:mod:`ladder.adapters.mujoco.t4_scene`) -- the deliverable;
2. re-runs that deliverable **cold and standalone** with the grader's own
   sampler and no agent (:mod:`ladder.adapters.mujoco.runner_t4`) -- phase B;
3. runs both contract checks (``agentbench.adapters.check_bundle`` and
   ``ladder.graders.t4.check_evidence``) against what comes back;
4. grades it through **the real T4 path** -- ``ladder.graders.t4.grade``, which
   hands the sim-neutral core a :class:`T4Evidence` and nothing MuJoCo-shaped.

**It is not a ladder cell and its verdict is not a result.** A cell is an
autonomous agent given one sentence and no help
(``capability-ladder-plan.md`` §2); this is a scripted control that a human
wrote knowing the thresholds. Its only claims are *"the task is achievable"*,
*"the outcome is reachable in the supported cell with no learning at all"* and
*"the instrument works on this column"*.

⚠ **And it is a SUPPORTED cell.** The default run applies an attitude-and-
lateral rig to the base, the grader measures it, and the cell it publishes says
so with its own numbers. ``AGENTS.md``'s humanoid disclosure rule binds every
sentence about it: this is **not** a free-standing walk.

Three runs, and the second and third are the point
--------------------------------------------------

``--rig wrench`` (default)
    the recorded recipe: an attitude PD plus a lateral catch, **no vertical
    carry**, applied through ``mjData.xfrc_applied``.
``--rig none``
    the identical script with the wrench switched off. Two cells from one
    script, and the number that separates them printed in both.
``--rig weld``
    ⚠ **the tier's own open question, made executable.** The rig is a weld
    equality to a mocap body instead of an applied wrench. It holds the robot
    exactly as firmly and contributes **nothing** to ``xfrc_applied``. When the
    two readings differ this command grades the *same recording* twice and
    prints both cells, so *"a constraint rig would be published as
    unsupported"* is a measurement rather than an argument.

``--skip-run`` re-grades an existing run directory from the artifacts on disk,
which is how a repaired grader is re-applied to a finished walk **without
re-simulating it**.
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
from ladder.adapters.mujoco import (evidence, runner_t4,          # noqa: E402
                                    t4_drive, t4_scene)
from ladder.graders import t4 as t4_grader                        # noqa: E402

TASK_ID = "T4_humanoid"
HERE = Path(__file__).resolve().parent


def build_deliverable(out_dir, *, task=None, ground="box", rig="wrench",
                      force_limits=True):
    """Assemble the scene and its driver into ``<out>/deliverable/``.

    The driver is **copied**, not imported: what phase B re-runs must be the
    files on disk, so a deliverable that only works because the benchmark's own
    package happened to be importable would be caught here rather than passing.
    """
    task = task or ladder_tasks.get(TASK_ID)
    out = Path(out_dir)
    deliverable = out / "deliverable"
    deliverable.mkdir(parents=True, exist_ok=True)

    res = t4_scene.build_t4_scene(task.container_dir, out / "workspace",
                                  ground=ground, rig=rig,
                                  force_limits=force_limits)
    t4_scene.write_build_record(res, out / "build.json")
    for p in res.problems:
        print("build problem: %s" % p, file=sys.stderr)
    if not res.scene:
        return None, res
    shutil.copyfile(res.scene, deliverable / runner_t4.SCENE_NAME)
    shutil.copyfile(HERE / "t4_drive.py", deliverable / runner_t4.DRIVER_NAME)
    return deliverable, res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, help="the run directory to write")
    ap.add_argument("--deliverable", default=None,
                    help="grade an existing deliverable instead of building "
                         "one")
    ap.add_argument("--skip-run", action="store_true",
                    help="grade an existing run directory, do not re-run")
    ap.add_argument("--rig", default="wrench",
                    choices=("wrench", "none", "weld"),
                    help="what holds the trunk up. 'wrench' is the recorded "
                         "recipe (attitude PD + lateral catch, NO vertical "
                         "carry); 'none' is the same script with it switched "
                         "off; 'weld' is the constraint rig the task file's "
                         "open question is about and is NOT a recommended "
                         "technique")
    ap.add_argument("--ground", default="box", choices=("box", "plane"),
                    help="the floor's geometry. 'box' is the default and is "
                         "what makes the arena channel a readable number; "
                         "'plane' is the recorded recipe's own choice and is "
                         "kept so the two can be compared")
    ap.add_argument("--unlimited-actuators", action="store_true",
                    help="do NOT clamp the actuators to the URDF's declared "
                         "efforts. This reproduces the recorded recipe, which "
                         "stated 'no force limit'; the default is stricter")
    ap.add_argument("--duration", type=float, default=None,
                    help="override the task's own standalone window (300 s)")
    ap.add_argument("--json", help="write the verdict as JSON to this path")
    a = ap.parse_args(argv)

    task = ladder_tasks.get(TASK_ID)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    phase = task.standalone
    # The child process inherits this; the driver reads it and records the
    # value it used, so a re-grade can see which rig produced the row rather
    # than having to be told. A welded scene overrides it -- a deliverable that
    # carries a weld equality IS welded whatever an env var says.
    os.environ[t4_drive.RIG_ENV] = ("none" if a.rig == "none" else "wrench")
    print("support rig: %s (%s=%s%s)"
          % (a.rig, t4_drive.RIG_ENV, os.environ[t4_drive.RIG_ENV],
             ", overridden by the scene: a deliverable carrying a weld "
             "equality IS welded whatever this says" if a.rig == "weld"
             else ""))

    deliverable = Path(a.deliverable) if a.deliverable else None
    if deliverable is None and not a.skip_run:
        deliverable, build = build_deliverable(
            out, task=task, ground=a.ground, rig=a.rig,
            force_limits=not a.unlimited_actuators)
        if deliverable is None:
            print("the deliverable was not built; see build.json",
                  file=sys.stderr)
            return 2
        print("built: %d bodies, %.3f kg compiled robot, %d actuators, "
              "%d build problems, ground=%s, scene-side rig=%s"
              % (build.bodies, build.mass_kg_compiled, len(build.actuators),
                 len(build.problems), build.ground.get("kind"),
                 build.rig.get("kind")))
    if deliverable is None:
        deliverable = out / "deliverable"

    res = None
    if not a.skip_run:
        res = runner_t4.run_standalone(
            deliverable, out,
            duration=float(a.duration if a.duration is not None
                           else phase.get("duration_s", 300.0)),
            settle=float(phase.get("settle_s", 1.5)),
            stride=int(phase.get("contact_stride", 2)),
            surfaces=task.surfaces, base=task.robot_name)
        print("phase B: exit=%s wall=%.2fs timed_out=%s"
              % (res.rc, res.wall_s or 0.0, res.timed_out))

    # --- contract checks, against what the run actually produced ---------
    ev = t4_grader.build_evidence(
        task, sim="mujoco", artifact=str(deliverable / runner_t4.SCENE_NAME),
        run_dir=out, phase_b=res)
    problems = t4_grader.check_evidence(ev)
    print("\n=== contract checks ===")
    if problems:
        for p in problems:
            print("  ! %s" % p)
    else:
        print("  clean: agentbench.check_bundle + ladder.check_t4_evidence "
              "report nothing")

    # --- the real grading path -------------------------------------------
    verdict = t4_grader.grade(out, task=task,
                              artifact=str(deliverable / runner_t4.SCENE_NAME),
                              phase_b=res, sim="mujoco")
    _print_verdict(verdict, "T4 verdict (a SCRIPTED control, not a ladder cell)")
    _print_the_effort_the_walk_actually_cost(out)
    naive = _grade_the_xfrc_only_reading(out, task, deliverable, res)

    if a.json:
        Path(a.json).write_text(
            json.dumps({"verdict": verdict.as_dict(),
                        "contract_problems": problems,
                        "phase_b": (res.as_dict() if res else None),
                        "xfrc_only_verdict": (naive.as_dict() if naive
                                              else None),
                        "provenance": ev.provenance()}, indent=2, default=str),
            encoding="utf-8")
    return 0 if verdict.outcome == "PASS" else 1


def _print_verdict(verdict, title):
    print("\n=== %s ===" % title)
    print(verdict.summary())
    print("")
    m = verdict.measurements
    # Never omitted: capability-ladder-plan.md §8 forbids quoting a T4 cell
    # without these, so neither may the tool that prints the cell.
    print("cell:                 %s" % m.get("cell"))
    print("  published as:       %s" % m.get("cell_text"))
    print("support_attestation:  %s" % m.get("support_attestation"))
    print("external_support:     %s"
          % json.dumps(m.get("external_support"), default=str))
    print("arena_attestation:    %s" % m.get("arena_attestation"))
    print("excluded from comparison: %s %s"
          % (m.get("excluded_from_comparison"),
             json.dumps(m.get("comparison_exclusions"), default=str)))
    print("method:               %s" % m.get("method"))
    print("reuse_class:          %s -- NOT decidable by a grader "
          "(capability-ladder-plan.md section 9 Q3: the "
          "reviewer arbitrates). A cell published with this null is not "
          "publishable." % m.get("reuse_class"))
    print("termination:          %s"
          % json.dumps(m.get("termination"), default=str))
    print("distance_to_termination_m: %s" % m.get("distance_to_termination_m"))
    gaps = m.get("unanswered_channels")
    print("unanswered channels:  %s" % (gaps if gaps else "none"))
    vac = {a_.id: [f.clause for f in a_.vacuous_clauses]
           for a_ in verdict.assertions if a_.vacuous}
    print("vacuous clauses:      %s" % (vac if vac else "none"))


def _grade_the_xfrc_only_reading(run_dir, task, deliverable, phase_b):
    """Grade the SAME recording as a column that ignored constraint forces.

    ⚠ This is the tier's own open question, executed. ``meta.json`` ->
    ``container.authored_here.an_open_question_the_demonstration_exposed``
    says a rig built as a weld *"would hold the robot just as firmly and the
    wrench channel would read ZERO -- and the run would be published in
    T4-unsupported, which is the cell the plan says must be 'numerically
    nothing'"*, and that no grader can close it because it reads what the
    column attests.

    So this prints what this column WOULD have published had it attested only
    ``mjData.xfrc_applied``. It runs only when the two readings actually differ
    -- i.e. when something in the scene really is holding the robot through a
    constraint -- because on every other run it would be the same verdict
    twice.
    """
    try:
        doc = json.loads((Path(run_dir) / "t4.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    sup = doc.get("support") or {}
    if sup.get("xfrc_only_identical_to_total", True):
        return None
    chans = evidence.t4_channels(run_dir=run_dir, support_reading="xfrc_only")
    if not chans:
        return None
    naive = t4_grader.grade(run_dir, task=task,
                            artifact=str(Path(deliverable)
                                         / runner_t4.SCENE_NAME),
                            phase_b=phase_b, sim="mujoco", channels=chans)
    print("\n" + "=" * 72)
    print("!! THE SAME RECORDING, READ BY A COLUMN THAT ATTESTED ONLY")
    print("   mjData.xfrc_applied -- i.e. one that did not count constraint")
    print("   forces on the base. This is NOT this column's attestation; it is")
    print("   the task file's open question executed. The equality reaction was")
    print("   live in %s of %s samples and peaked at %s constraint rows."
          % (sup.get("equality_reaction_was_live_in_samples"),
             len(sup.get("t") or []), sup.get("peak_equality_rows_observed")))
    print("=" * 72)
    _print_verdict(naive, "T4 verdict under the xfrc-only reading")
    return naive


def _print_the_effort_the_walk_actually_cost(run_dir):
    """Peak actuator force per joint against the effort the URDF declares.

    Printed rather than asserted: the recorded recipe left the actuators
    unlimited, so *"did the walk stay inside the robot's own declared limits"*
    is a measurement about the run and it belongs beside the verdict rather
    than in a footnote. A joint over its declared effort is a real finding and
    it should be visible without opening a JSON file.
    """
    try:
        doc = json.loads((Path(run_dir) / "t4.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return
    peaks = doc.get("actuator_peak_force") or {}
    efforts = doc.get("declared_efforts_nm") or {}
    if not peaks:
        return
    rows, over, at = [], [], []
    for act, peak in sorted(peaks.items()):
        joint = act[4:] if act.startswith("pos_") else act
        eff = efforts.get(joint)
        rows.append("%s %.2f/%s" % (joint, peak,
                                    ("%.0f" % eff) if eff else "?"))
        if eff and peak > float(eff) + 1e-6:
            over.append("%s (%.2f > %.0f N.m)" % (joint, peak, eff))
        elif eff and peak >= float(eff) - 1e-6:
            at.append("%s (%.2f N.m)" % (joint, peak))
    print("peak |actuator force| vs the URDF's declared effort, N.m:")
    print("  " + "  ".join(rows))
    if over:
        print("  OVER the declared effort: " + ", ".join(over))
    elif at:
        print("  ALL INSIDE the declared efforts, and SATURATING at the clamp "
              "on: " + ", ".join(at))
    else:
        print("  ALL INSIDE the declared efforts")
    rig = ((doc.get("driver") or {}).get("describe") or {}).get("rig")
    cr = doc.get("constraint_rig") or {}
    print("rig in the run record: %s | neq=%s equalities=%s nmocap=%s "
          "base has a free joint=%s kinematic base=%s actuators on it=%s"
          % (rig, cr.get("neq"),
             [e.get("name") for e in (cr.get("equalities") or [])],
             cr.get("nmocap"), cr.get("base_has_a_free_joint"),
             cr.get("base_is_kinematic"),
             cr.get("actuators_on_the_base_free_joint")))


if __name__ == "__main__":
    sys.exit(main())
