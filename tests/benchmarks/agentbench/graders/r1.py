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

"""R1 ``lidar_nav`` -- the task entry point (robotics tier).

Thin by design: it resolves the adapter for the simulator under test, asks it
for a neutral evidence bundle, and hands that to
:mod:`agentbench.graders.r1_core`, where the six assertions and their
thresholds live.

R1 declares ``any_robot`` as its identity predicate. The task does NOT ship a
world -- the agent authors the robot -- so "is this our Husky" is the wrong
question; "is there exactly one drivable robot, and did it get to the goal
around the obstacles without touching anything" is the whole question, and
R1.2 asks it in the core.

The obstacle bodies must be in the frozen t=0 inventory, because R1.3
re-derives from their MEASURED geometry that the straight-line path is
blocked -- the fact the sensing argument rests on. Trusting the asset file for
that would make the grader score a claim instead of a measurement.

**Which layout is this run scored against?** R1's anti-hardcode mechanism is
grade-time placement: the obstacle positions are drawn from the benchmark seed
and the world is placed with them (``r1_core.sample_layout``). This entry point
resolves that layout for the core, in the order
:func:`r1_core.resolve_graded_layout` declares -- an explicit ``graded_layout``
argument, then an ``r1_graded_layout.json`` sidecar written next to the run by
whoever placed the obstacles (beside the run, in its phase-B output, in the
grader's scratch, or in the directory ``$AGENTBENCH_R1_LAYOUT_DIR`` names when
the grader runs in a subprocess), then a ``seed`` to draw from. When none of
those is present no placement happened, the core scores the published layout,
and the verdict says out loud that the row is not evidence of perception.

The placement step itself is ``common/r1_placement.py``, driven from
``cc_lane/run_cc_cell.run_cell`` between the agent's session and the graded
run.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench import adapters  # noqa: E402
from agentbench.graders import r1_core  # noqa: E402
from agentbench.graders.r1_core import (  # noqa: E402,F401  (re-exported)
    ARENA_HALF_M, GOAL_TOL_M, GOAL_XY, LAYOUT_DIR_ENV, LAYOUT_SIDECAR,
    MAX_STEP_DISPLACEMENT_M, MIN_ROBOT_JOINTS, N_OBSTACLES,
    OBSTACLE_POSE_TOL_M, START_XY, TASK)

ROBOT_IDENTITY = "any_robot"


def grade(run_dir, *, artifact=None, phase_b=None, self_verified=False,
          scratch_dir=None, sim=None, seed=None, graded_layout=None, **kw):
    adapter = adapters.resolve(sim)
    bundle = adapter.build_bundle(
        TASK, robot_identity=ROBOT_IDENTITY, live_expected=False,
        artifact=artifact, phase_b=phase_b, scratch_dir=scratch_dir,
        run_dir=run_dir,
        # R1.3 matches obstacles by GEOMETRY and never by name, so it needs
        # every non-robot body the run contains -- not the five the task
        # happened to publish names for. Without this the geometric matcher
        # has no candidates at all and R1.3 is UNDECIDABLE whatever the agent
        # builds: measured 2026-08-09 on both arms, while the same assertion
        # passed on the MuJoCo arm, whose t=0 scan has never needed a name
        # list. This flag is what asks each adapter for its name-free scan.
        scene_inventory=True, **kw)
    layout, source = r1_core.resolve_graded_layout(
        layout=graded_layout, seed=seed,
        # The sidecar is looked for beside the run first, then in the phase-B
        # output, then in the agent's scratch: the placement step writes it
        # wherever it does its work, and a grader that only looked in one place
        # would silently fall back to the published layout -- which is the
        # memoriser's best case and the failure this mechanism exists to close.
        # ...and, last, the directory a placement step names in the
        # environment. The OmniSim arm's graded run happens inside a
        # `run_agentbench.py --agent external` SUBPROCESS whose per-cell run
        # dir is created (and any existing one deleted) after the placer has
        # finished, so there is no path the placer could have written to that
        # this search would otherwise reach. Same one file, same one name --
        # only the directory is passed.
        directories=(run_dir, Path(run_dir) / "phaseB" if run_dir else None,
                     scratch_dir, os.environ.get(LAYOUT_DIR_ENV)))
    return r1_core.grade(bundle, self_verified=self_verified,
                         graded_layout=layout, layout_source=source)
