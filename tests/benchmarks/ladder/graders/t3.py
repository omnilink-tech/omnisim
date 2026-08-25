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

"""T3 ``quadruped`` -- the task entry point.

Thin by design, exactly as :mod:`ladder.graders.t1` and
:mod:`ladder.graders.t2` are: resolve the adapter for the simulator under test,
ask it for neutral evidence, hand that to :mod:`ladder.graders.t3_core`, where
the five assertions and every threshold live. Nothing here decides anything.

T3 declares ``any_robot`` as its identity predicate, for the reason T2 gives:
*"is this a Go2"* is not the question the tier asks. The container ships one
four-legged robot description; which body in the run is its base is answered by
name (``meta.json`` -> ``robot.declared_name``), which bodies are the ground is
answered by name too (``meta.json`` -> ``ground.names``), and body **class** is
not asked at all.

Where the evidence shim lives, and why it is here
-------------------------------------------------

T1 and T2 build their evidence through ``ladder.adapters.build_t1_evidence`` /
``build_t2_evidence``. T3's equivalent is :func:`build_evidence` **in this
file**, and that is a deliberate, reversible choice rather than a preference:
``ladder/adapters/__init__.py`` is shared, live scaffolding, and a rung that
has not been frozen has no business editing it. Everything the shim needs from
that module -- ``resolve``, ``resolve_ladder_channels``, the reuse of
``agentbench.adapters`` -- it *calls*; nothing is forked. Moving this function
into ``ladder.adapters`` alongside its two siblings is a two-line change and
should happen when T3 freezes.

A column supplies T3's channels through **one** optional hook on its
ladder-side module::

    t3_channels(phase_b_or_run_dir, surface=<WalkingSurface>) -> dict

whose keys are a subset of :data:`T3_CHANNELS`. Every key it does not supply
falls back to what the frozen bundle can derive -- which for two of them is a
real answer (the settled standing height, and the walking region's extent
wherever the ground is a bounded body), for two is half an answer (position
with no orientation; ``dynamic`` with no kilograms) and for the rest is nothing
at all. Each fallback puts :data:`T3_GAP_NOTE` in the verdict's notes naming
the channel and the assertions it breaks, so a red that belongs to our
scaffolding is never published as a finding about somebody else's product.

Two ways in
-----------

:func:`grade` grades evidence that already exists -- a phase-B run somebody
else performed, or a fixture. :func:`run_and_grade` performs phase B first: it
re-runs the deliverable **cold and standalone** with the grader's own sampler
injected and no agent present, which is what SPEC §2.3 means by phase B and the
only thing a ladder cell may be scored from.

    python -m ladder.graders.t3 <deliverable> [--run-dir DIR] [--sim ...]

**What a cell of this shape may not be quoted without.** Every T3 verdict
carries ``measurements["support_attestation"]``, ``["external_support"]``,
``["method"]`` and ``["reuse_class"]``, and ``capability-ladder-plan.md`` §8
forbids quoting a T3 cell without its ``reuse_class`` -- which no grader can
decide (§9 Q3: *"the reviewer arbitrates"*), so it is printed as ``null`` and
the CLI says out loud that the cell is not publishable until a human fills it.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench import adapters as ab_adapters  # noqa: E402
from ladder import adapters, tasks  # noqa: E402
from ladder.graders import t3_core, t3_evidence as t3ev  # noqa: E402
from ladder.graders.t3_core import (  # noqa: E402,F401  (re-exported)
    ARENA_MARGIN_M, FALL_Z_FRACTION, MAX_SAMPLE_DT_S, MAX_STEP_JUMP_M,
    MAX_SUPPORT_FORCE_FRACTION, MAX_SUPPORT_TORQUE_NM, MAX_TILT_RAD,
    MIN_BOB_RMS_M, MIN_CONTACT_TRANSITIONS, MIN_MEAN_SPEED_MPS,
    MIN_RUN_UP_FACTOR, NET_DISPLACEMENT_M, PLATEAU_TOL_M, PLATEAU_WINDOW_S,
    TASK)

ROBOT_IDENTITY = "any_robot"

T3_CHANNELS = ("base_pose", "standing", "gait", "support", "arena",
               "base_physics", "world", "controller")

T3_GAP_NOTE = (
    "no ladder-side %s channel exists for the %r column, so what depends on "
    "it is unmeasured rather than measured-and-absent. The blocker for a cell "
    "whose only reds are those assertions is scaffolding_defect_ours -- it "
    "may not be published as a capability finding about the simulator.")


# --- building the evidence ---------------------------------------------------


def _resolve_surface(task):
    """Which bodies are the ground, from the task file. Never from an adapter.

    Task data, for the reason T1's waypoint and T2's roles are, and with more
    at stake than either: T3.4 asserts that *nothing but the ground* touched
    the robot, so an adapter free to decide what counts as the ground could
    label the thing holding the robot up as the floor and pass the assertion
    outright.
    """
    spec = task.meta.get("ground", {}) or {}
    names = tuple(spec.get("names", []) or [])
    return t3ev.WalkingSurface(
        names=names,
        source=("%s -> ground.names (%s); %s"
                % (task.citation(), ", ".join(names) or "none declared",
                   spec.get("resolution_rule", "no resolution rule stated"))))


def build_evidence(task, *, sim=None, artifact=None, run_dir=None,
                   phase_b=None, surface=None, channels=None, **kw):
    """One T3 run -> :class:`T3Evidence`. Never raises for a broken simulator.

    See the module docstring for why this lives here rather than in
    ``ladder.adapters`` beside its two siblings.
    """
    if not hasattr(task, "meta"):
        task = tasks.get(str(task))
    sim = adapters._sim_id(sim)          # noqa: SLF001  (its own shim)
    notes = []

    adapter = adapters.resolve(sim)
    bundle = adapter.build_bundle(
        task.id, robot_identity=ROBOT_IDENTITY, live_expected=False,
        artifact=artifact, phase_b=phase_b, run_dir=run_dir, **kw)

    if surface is None:
        surface = _resolve_surface(task)
    robot = task.robot_name

    supplied = dict(channels or {})
    if not supplied:
        mod = adapters.resolve_ladder_channels(sim)
        hook = getattr(mod, "t3_channels", None) if mod is not None else None
        if hook is not None:
            try:
                # ``run_dir`` stands in when there is no phase-B result object,
                # exactly as the T2 shim does it: grading an EXISTING run
                # directory passes ``run_dir`` and no ``phase_b``, and a hook
                # handed ``None`` would report every channel unanswered on a
                # run that has all of them on disk.
                supplied = dict(hook(phase_b if phase_b is not None
                                     else run_dir, surface=surface) or {})
            except Exception as exc:  # noqa: BLE001  (adapter rule 1)
                notes.append("the %r column's T3 channel builder raised %r; "
                             "every channel falls back" % (sim, exc))
                supplied = {}

    def take(key, fallback, gap_for=None):
        got = supplied.get(key)
        if got is None:
            notes.append(T3_GAP_NOTE % (gap_for or key, sim))
            return fallback
        return got

    return t3ev.T3Evidence(
        bundle=bundle, surface=surface, robot_name=robot, notes=notes,
        robot_mass_kg_declared=getattr(task, "robot_mass_kg_declared", None),
        base_pose=take("base_pose",
                       t3ev.pose_series_from_bundle(bundle, name=robot)),
        standing=take("standing",
                      t3ev.standing_height_from_bundle(bundle, name=robot),
                      gap_for="standing_height"),
        gait=take("gait", t3ev.EMPTY_GAIT_CONTACTS, gap_for="gait_contacts"),
        support=take("support", t3ev.EMPTY_APPLIED_SUPPORT,
                     gap_for="applied_support"),
        arena=take("arena", t3ev.arena_from_bundle(bundle, surface)),
        base_physics=take("base_physics",
                          t3ev.body_physics_from_bundle(bundle, name=robot),
                          gap_for="base_mass"),
        world=take("world", t3ev.EMPTY_WORLD_PHYSICS, gap_for="gravity"),
        controller=take("controller", t3ev.EMPTY_CONTROLLER_LOAD,
                        gap_for="controller_load"))


def check_evidence(ev):
    """Both contracts at once: agentbench's bundle check plus the ladder's."""
    return (["bundle: %s" % p for p in ab_adapters.check_bundle(ev.bundle)]
            + t3ev.check_t3_evidence(ev))


# --- grading -----------------------------------------------------------------


def grade(run_dir, *, task=TASK, artifact=None, phase_b=None, sim=None,
          surface=None, channels=None, self_verified=False, **kw):
    """Grade one T3 attempt from evidence that already exists."""
    ev = build_evidence(task, sim=sim, artifact=artifact, run_dir=run_dir,
                        phase_b=phase_b, surface=surface, channels=channels,
                        **kw)
    verdict = t3_core.grade(ev, self_verified=self_verified)
    gaps = t3ev.unanswered_channels(ev)
    if gaps:
        verdict.measurements["unanswered_channels"] = {
            k: list(v) for k, v in gaps.items()}
        verdict.note(
            "channels this column could not answer: %s. A red assertion that "
            "depends only on those is blocker=scaffolding_defect_ours and is "
            "not a finding about the simulator."
            % "; ".join("%s (breaks %s)" % (k, ", ".join(v) or "no assertion")
                        for k, v in sorted(gaps.items())))
    return verdict


def run_and_grade(deliverable, run_dir, *, task=TASK, sim=None, backend=None,
                  duration=None, settle=None, stride=None, timeout_s=7200.0,
                  self_verified=False):
    """Run phase B on ``deliverable``, then grade it. Never raises.

    The phase config comes from the task's own ``meta.json`` unless overridden
    here, so the numbers a cell was graded under are in a file a reader can
    open rather than in a call site.
    """
    if not hasattr(task, "meta"):
        task = tasks.get(str(task))
    channels = adapters.resolve_ladder_channels(sim)
    # A column may ship a tier-specific sampler. Prefer it: the generic
    # ``run_standalone`` on a column that also serves T2 is *that* tier's
    # sampler, and it records none of T3's channels -- a run that grades as
    # "unanswered" for a reason that has nothing to do with the simulator.
    runner = None
    for attr in ("t3_run_standalone", "run_standalone"):
        if channels is not None and hasattr(channels, attr):
            runner = getattr(channels, attr)
            break
    if runner is None:
        raise NotImplementedError(
            "no ladder-side phase-B runner for the %r column; grade an "
            "existing run with grade(...) instead"
            % adapters._sim_id(sim))      # noqa: SLF001  (its own shim)

    cfg = task.standalone
    res = runner(
        deliverable, run_dir, backend=backend,
        duration=float(duration if duration is not None
                       else cfg.get("duration_s", 120.0)),
        settle=float(settle if settle is not None
                     else cfg.get("settle_s", 1.0)),
        stride=int(stride if stride is not None
                   else cfg.get("contact_stride", 2)),
        surfaces=task.surfaces, timeout_s=timeout_s)
    verdict = grade(run_dir, task=task, artifact=str(deliverable),
                    phase_b=res, sim=sim, self_verified=self_verified)
    verdict.artifacts["phase_b_run_dir"] = str(run_dir)
    return verdict, res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("deliverable", help="the scene the attempt produced")
    ap.add_argument("--run-dir", default=None,
                    help="where phase B writes (default: alongside it)")
    ap.add_argument("--sim", default=None)
    ap.add_argument("--task", default=TASK)
    ap.add_argument("--backend", default=None, choices=(None, "ode", "newton"))
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--settle", type=float, default=None)
    ap.add_argument("--stride", type=int, default=None)
    ap.add_argument("--timeout", type=float, default=7200.0)
    ap.add_argument("--json", default=None, help="also write the row here")
    a = ap.parse_args(argv)

    deliverable = Path(a.deliverable).resolve()
    run_dir = Path(a.run_dir) if a.run_dir else deliverable.parent / "_phase_b"
    verdict, res = run_and_grade(
        deliverable, run_dir, task=a.task, sim=a.sim, backend=a.backend,
        duration=a.duration, settle=a.settle, stride=a.stride,
        timeout_s=a.timeout)

    print(verdict.summary())
    print("")
    # Never omitted: §8 forbids quoting a T3 cell without these, so neither may
    # the tool that prints the cell.
    print("support_attestation: %s"
          % verdict.measurements.get("support_attestation"))
    print("external_support:    %s"
          % json.dumps(verdict.measurements.get("external_support"),
                       default=str))
    print("method:              %s" % verdict.measurements.get("method"))
    print("reuse_class:         %s -- NOT decidable by a grader "
          "(capability-ladder-plan.md section 9 Q3: the reviewer arbitrates). "
          "A cell published with this null is not publishable."
          % verdict.measurements.get("reuse_class"))
    print("termination:         %s"
          % json.dumps(verdict.measurements.get("termination"), default=str))
    print("")
    print("phase B: %s" % json.dumps(res.as_dict(), indent=2, default=str))
    if a.json:
        Path(a.json).write_text(json.dumps(verdict.as_dict(), indent=2,
                                           default=str), encoding="utf-8")
    return 0 if verdict.outcome == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
