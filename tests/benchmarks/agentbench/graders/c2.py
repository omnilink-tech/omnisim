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

"""C2 ``fall_through_floor`` -- the task entry point (debug tier).

Thin by design: it resolves the adapter for the simulator under test, asks it
for a neutral evidence bundle, and hands that to
:mod:`agentbench.graders.c2_core`, where the five assertions and their
thresholds live.

C2 declares ``any_robot`` as its identity predicate: the task ships the world,
so "is this the right robot" is not the question -- "is the body the task named
still a dynamic body resting on the floor it built" is. It also asks the adapter
to include the named floor body in the frozen t=0 inventory, so C2.5 measures
the floor the agent actually built rather than the one the fixture shipped.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench import adapters  # noqa: E402
from agentbench.graders import c2_core  # noqa: E402
from agentbench.graders.c2_core import (  # noqa: E402,F401  (re-exported)
    FLOOR_NAME, MIN_DESCENT_M, MIN_Z_M, REST_BAND_M, REST_TOL_M,
    REST_WINDOW_S, ROBOT_NAME, TASK)

ROBOT_IDENTITY = "any_robot"


def grade(run_dir, *, artifact=None, phase_b=None, self_verified=False,
          scratch_dir=None, sim=None, **kw):
    adapter = adapters.resolve(sim)
    bundle = adapter.build_bundle(
        TASK, robot_identity=ROBOT_IDENTITY, live_expected=False,
        artifact=artifact, phase_b=phase_b, scratch_dir=scratch_dir,
        run_dir=run_dir, **kw)
    return c2_core.grade(bundle, self_verified=self_verified)
