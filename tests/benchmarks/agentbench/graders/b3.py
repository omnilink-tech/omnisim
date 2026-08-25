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

"""B3 ``measure_and_report`` -- the task entry point (inspect tier).

There is no artifact: the deliverable is the agent's ANSWER. The adapter's only
job here is the ground truth -- the two named bodies' positions and world-space
AABBs at a frozen t=0 -- and :mod:`agentbench.graders.b3_core` does the
comparison in metres.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench import adapters  # noqa: E402
from agentbench.graders import b3_core  # noqa: E402
from agentbench.graders.b3_core import (  # noqa: E402,F401  (re-exported)
    DISTANCE_TOL_M, GROUND_NAME, PLINTH_NAME, TASK, measure_ground_truth)

ROBOT_IDENTITY = "any_robot"


def grade(run_dir, *, answer="", phase_b=None, self_verified=False,
          artifact=None, scratch_dir=None, sim=None, **kw):
    adapter = adapters.resolve(sim)
    bundle = adapter.build_bundle(
        TASK, robot_identity=ROBOT_IDENTITY, live_expected=False,
        artifact=artifact, phase_b=phase_b, scratch_dir=scratch_dir,
        run_dir=run_dir, **kw)
    return b3_core.grade(bundle, answer=answer, self_verified=self_verified)
