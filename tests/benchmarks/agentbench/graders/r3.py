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

"""R3 ``pick_and_place`` -- the task entry point (robotics tier).

Thin by design, exactly as R1: it resolves the adapter for the simulator under
test, asks it for a neutral evidence bundle, and hands that to
:mod:`agentbench.graders.r3_core`, where the eight assertions and every
threshold live.

R3 declares ``any_robot`` as its identity predicate. The task ships the SCENE
(floor, table, cube, bin) and the agent supplies the manipulator, so "is this
our arm" is the wrong question -- and the cube, which is what the grader
actually measures, is found by its t=0 geometry rather than by any name the
adapter could attest to.

The cube, the table and the bin must all be in the frozen t=0 inventory: R3.2
re-derives the bin volume and the table surface from their MEASURED geometry,
because every later assertion is stated against them and trusting the file for
that would make the grader score a claim instead of a measurement. The table
and the bin reach that inventory through ``meta.json``'s
``phases.standalone.solids`` list; the cube is authored as a Robot node so that
both arms' recorders give it a pose series at all.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench import adapters  # noqa: E402
from agentbench.graders import r3_core  # noqa: E402
from agentbench.graders.r3_core import (  # noqa: E402,F401  (re-exported)
    AIRBORNE_CLEARANCE_M, AIRBORNE_MIN_S, BIN_CENTER_XYZ, BIN_INSET_M,
    CARRY_MIN_XY_M, CUBE_REST_Z_M, CUBE_SIZE_M, CUBE_START_XYZ, HOLD_S,
    MAX_SPEED_MPS, MAX_STEP_DISPLACEMENT_M, MIN_ARM_JOINTS, RELEASE_DESCENT_M,
    REST_BAND_M, SCENE_TOL_M, START_TOL_M, TABLE_TOP_Z_M, TASK, Z_AIRBORNE_M)

ROBOT_IDENTITY = "any_robot"


def grade(run_dir, *, artifact=None, phase_b=None, self_verified=False,
          scratch_dir=None, sim=None, **kw):
    adapter = adapters.resolve(sim)
    bundle = adapter.build_bundle(
        TASK, robot_identity=ROBOT_IDENTITY, live_expected=False,
        artifact=artifact, phase_b=phase_b, scratch_dir=scratch_dir,
        run_dir=run_dir, **kw)
    return r3_core.grade(bundle, self_verified=self_verified)
