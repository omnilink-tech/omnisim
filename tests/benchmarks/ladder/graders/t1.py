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

"""T1 ``arrive`` -- the task entry point.

Thin by design, exactly as the AgentBench task entry points are: resolve the
adapter for the simulator under test, ask it for neutral evidence, hand that
to :mod:`ladder.graders.t1_core`, where the five assertions and every
threshold live. Nothing here decides anything.

T1 declares ``any_robot`` as its identity predicate. "Is this a Husky" is not
the question the tier asks -- the container ships one robot description and the
agent builds one scene from it, so the question is *which body in the run is
the one the description became*, and that is the core's selection rule
(:func:`ladder.graders.t1_core.select_robot`), not an identity count.

Two ways in
-----------

:func:`grade` grades evidence that already exists -- a phase-B run somebody
else performed, or a fixture. :func:`run_and_grade` performs phase B first:
it re-runs the deliverable **cold and standalone** with the grader's own
sampler injected and no agent present, which is what SPEC 2.3 means by phase
B and the only thing a ladder cell may be scored from.

    python -m ladder.graders.t1 <deliverable> [--run-dir DIR] [--sim omnisim]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ladder import adapters, tasks  # noqa: E402
from ladder.graders import t1_core  # noqa: E402
from ladder.graders.t1_core import (  # noqa: E402,F401  (re-exported)
    ARRIVE_TOL_M, DWELL_S, HOLD_TOL_M, MAX_ABS_DZ_M, MAX_PATH_RATIO,
    MAX_SAMPLE_DT_S, MAX_STEP_JUMP_M, MIN_PATH_RATIO, MIN_Z_M,
    MOVING_SPEED_EPS_MPS, TASK)

ROBOT_IDENTITY = "any_robot"


def grade(run_dir, *, task=TASK, artifact=None, phase_b=None, sim=None,
          support=None, waypoint=None, self_verified=False, **kw):
    """Grade one T1 attempt from evidence that already exists."""
    ev = adapters.build_t1_evidence(
        task, sim=sim, artifact=artifact, run_dir=run_dir, phase_b=phase_b,
        support=support, waypoint=waypoint, **kw)
    return t1_core.grade(ev, self_verified=self_verified)


def run_and_grade(deliverable, run_dir, *, task=TASK, sim=None, backend=None,
                  duration=None, settle=None, stride=None, timeout_s=900.0,
                  self_verified=False):
    """Run phase B on ``deliverable``, then grade it. Never raises.

    The phase config comes from the task's own ``meta.json`` unless overridden
    here, so the numbers a cell was graded under are in a file a reader can
    open rather than in a call site.
    """
    if not hasattr(task, "meta"):
        task = tasks.get(str(task))
    channels = adapters.resolve_ladder_channels(sim)
    # Prefer a tier-specific phase-B sampler where the column ships one. A
    # column's generic ``run_standalone`` is whichever tier was written first
    # on it, and grading tier N through tier M's sampler records none of tier
    # N's channels -- a cell that reads "unanswered" for a reason that has
    # nothing to do with the simulator.
    runner = None
    for _attr in ("t1_run_standalone", "run_standalone"):
        if channels is not None and hasattr(channels, _attr):
            runner = getattr(channels, _attr)
            break
    if runner is None:
        raise NotImplementedError(
            "no ladder-side phase-B runner for the %r column; grade an "
            "existing run with grade(...) instead"
            % adapters._sim_id(sim))          # noqa: SLF001  (its own shim)

    cfg = task.standalone
    res = runner(
        deliverable, run_dir, backend=backend,
        duration=float(duration if duration is not None
                       else cfg.get("duration_s", 60.0)),
        settle=float(settle if settle is not None
                     else cfg.get("settle_s", 0.5)),
        stride=int(stride if stride is not None
                   else cfg.get("contact_stride", 10)),
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
    ap.add_argument("--timeout", type=float, default=900.0)
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
    print("phase B: %s" % json.dumps(res.as_dict(), indent=2, default=str))
    if a.json:
        Path(a.json).write_text(json.dumps(verdict.as_dict(), indent=2,
                                           default=str), encoding="utf-8")
    return 0 if verdict.outcome == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
