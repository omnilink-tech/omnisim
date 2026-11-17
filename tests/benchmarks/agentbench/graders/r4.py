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

"""R4 ``mobile_manipulation`` -- the task entry point (robotics tier).

Thin by design, exactly as R1 and R3: it resolves the adapter for the
simulator under test, asks it for a neutral evidence bundle, and hands that to
:mod:`agentbench.graders.r4_core`, where the nine assertions and every
threshold live.

R4 declares ``any_robot`` as its identity predicate. The task ships no world --
the agent authors the arena, the robot, the gripper and the controller -- so
"is this our Husky" is the wrong question. "Is there one articulated mobile
manipulator, did it get to the table around the obstacles without touching
anything, did the payload leave the table and stay attached to it for the whole
transport, and is it on the pad at the end" is the whole question.

Every body the grader measures reaches it through the adapters' **name-free
t=0 scene scan** (``scene_inventory=True``), because R4.2 and R4.4 match the
obstacles, the table, the pad and the payload by GEOMETRY and never by name --
R1 shipped a grader keyed on the published ``OBSTACLE_n`` names and the first
real agent called its boxes "crate A".."crate E".

**Which layout is this run scored against?** The same mechanism R1 uses:
:func:`r4_core.resolve_graded_layout` takes an explicit ``graded_layout``, then
an ``r4_graded_layout.json`` sidecar written next to the run by whoever placed
the obstacles, then a ``seed`` to draw from. When none is present no placement
happened, the core scores the published layout, and **the verdict says out loud
that the row is not evidence of perception**. R4 has no injection step wired
into ``cc_lane`` yet (``meta.json`` ``anti_hardcode.injection_step`` records
that plainly), so that is the state a cell run today is in.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench import adapters  # noqa: E402
from agentbench.graders import r4_core  # noqa: E402
from agentbench.graders.r4_core import (  # noqa: E402,F401  (re-exported)
    ARENA_HALF_M, CARRY_CLEARANCE_M, CARRY_RADIUS_TOL_M, DELIVERY_TOL_M,
    LAYOUT_DIR_ENV, LAYOUT_SIDECAR, MAX_BASE_STEP_M, MIN_ROBOT_JOINTS,
    N_OBSTACLES, OBSTACLE_POSE_TOL_M, PAD_XY, PAYLOAD_SIZE_M, START_XY,
    TABLE_TOP_Z_M, TABLE_XY, TASK, TRANSPORT_MIN_M, Z_CARRIED_M, Z_LIFTED_M)

ROBOT_IDENTITY = "any_robot"


def grade(run_dir, *, artifact=None, phase_b=None, self_verified=False,
          scratch_dir=None, sim=None, seed=None, graded_layout=None, **kw):
    adapter = adapters.resolve(sim)
    bundle = adapter.build_bundle(
        TASK, robot_identity=ROBOT_IDENTITY, live_expected=False,
        artifact=artifact, phase_b=phase_b, scratch_dir=scratch_dir,
        run_dir=run_dir,
        # Every geometric matcher in this task needs candidates it was not
        # handed a name for: the obstacles (the agent names them), the table
        # and the pad (the agent authors them), and the payload. Without the
        # name-free scan there are no candidates at all and R4.2/R4.4 are
        # UNDECIDABLE whatever the agent builds -- measured on both .wbt arms
        # for R1 on 2026-08-09.
        scene_inventory=True, **kw)
    layout, source = r4_core.resolve_graded_layout(
        layout=graded_layout, seed=seed,
        directories=(run_dir, Path(run_dir) / "phaseB" if run_dir else None,
                     scratch_dir, os.environ.get(LAYOUT_DIR_ENV)))
    return r4_core.grade(bundle, self_verified=self_verified,
                         graded_layout=layout, layout_source=source)
