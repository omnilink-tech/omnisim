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

"""R2 ``arm_reach`` -- the task entry point (robotics tier).

Thin by design, exactly like R1: resolve the adapter for the simulator under
test, ask it for a neutral evidence bundle, hand that to
:mod:`agentbench.graders.r2_core`, where the six assertions and their thresholds
live -- and where every threshold is read from the task's own ``meta.json``.

R2 declares ``any_robot`` as its identity predicate. The task ships NO world at
all (the prompt is "create or import"), so "is this our arm" is the wrong
question; "is there exactly one body with at least six joints, and did the thing
on the end of it hold three Cartesian targets in order without being dragged or
teleported" is the whole question, and R2.2-R2.6 ask it in the core.

There is no Phase A here. The harness pass reads a live scene through OmniSim's
own HTTP surface, which has no counterpart on the other arm; every R2 assertion
is answered from the grader-owned recorder instead, so the two simulators are
measured by the same instrument.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench import adapters  # noqa: E402
from agentbench.graders import r2_core  # noqa: E402
from agentbench.graders.r2_core import (  # noqa: E402,F401  (re-exported)
    DEADLINE_S, DWELL_S, GROUND_CLEARANCE_TOL_M, MAX_BASE_DRIFT_M,
    MAX_EE_SPEED_MPS, MAX_EE_STEP_M, MAX_REACH_M, MAX_SAMPLE_DT_S,
    MIN_ARM_JOINTS, MIN_EE_TRAVEL_M, POSITION_TOL_M, RECORD_DURATION_S,
    TARGETS_BASE_FRAME, TASK)

ROBOT_IDENTITY = "any_robot"


def grade(run_dir, *, artifact=None, phase_b=None, self_verified=False,
          scratch_dir=None, sim=None, **kw):
    adapter = adapters.resolve(sim)
    bundle = adapter.build_bundle(
        TASK, robot_identity=ROBOT_IDENTITY, live_expected=False,
        artifact=artifact, phase_b=phase_b, scratch_dir=scratch_dir,
        run_dir=run_dir, **kw)
    return r2_core.grade(bundle, self_verified=self_verified)
