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

"""A1 ``husky_swarm_10`` -- the task entry point (SPEC 2.3).

This module is deliberately thin. It is the seam between the runner (which
speaks whatever the simulator under test produced) and the grader core (which
speaks only metres and seconds):

    run evidence --> adapters.resolve(sim).build_bundle(...) --> a1_core.grade()

The ten assertions, their thresholds and the vacuity declarations live in
:mod:`agentbench.graders.a1_core`; the OmniSim-shaped reading lives in the
adapter. Nothing task-specific about *this simulator* is in either -- what A1
declares here is the robot-identity predicate (``"husky"``) and the fact that it
expects a live-session pass as well as a standalone one.

Two implementation notes that are deviations from the letter of the SPEC and
are recorded on every verdict as ``notes``:

  * **A1.3's t=0 half is measured by the grader-owned recorder, not by the live
    session.** A free-running session cannot freeze t=0 (measured on OmniSim's
    harness: ``GET /robots`` on this world class costs 22 s of simulated
    driving). The assertion itself -- world-space AABBs must not overlap, with
    >= 0.05 m clearance on at least one axis, and no robot-robot contact in the
    first 10 steps -- is graded exactly as written, just from a source that can
    honestly claim "t=0".
  * **A1.9's "the log shows the world reached finalize"** has no cross-backend
    marker (OmniSim's Newton backend logs one; ODE logs nothing equivalent, and
    another simulator logs something else entirely). The engine-agnostic
    evidence is the recorder's own completion record: it reached SETTLE_S + N of
    *simulated* time and asked the simulator to quit, which is impossible unless
    the world was built, finalized and stepped. Whichever evidence the adapter
    had is named in the verdict.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench import adapters  # noqa: E402
from agentbench.graders import a1_core  # noqa: E402
from agentbench.graders.a1_core import (  # noqa: E402,F401  (re-exported)
    BEARING_EPS_M, CONTACT_STEPS, MAX_ABS_DZ_M, MAX_R, MIN_AABB_CLEARANCE_M,
    MIN_NET_DISP_M, MIN_PATH_M, MIN_Z_M, N_ROBOTS, SETTLE_S, TASK, WINDOW_S)

# The task's robot-identity predicate. Each adapter answers it in its own terms
# and publishes the rule it used (SPEC 6.2.6); the count and the distinctness
# stay in the core.
ROBOT_IDENTITY = "husky"


def grade(run_dir, *, artifact=None, harness=None, phase_b=None,
          self_verified=False, scratch_dir=None, sim=None, **kw):
    """Grade one A1 run. Pure function of measured state, never raises."""
    adapter = adapters.resolve(sim)
    bundle = adapter.build_bundle(
        TASK, robot_identity=ROBOT_IDENTITY, live_expected=True,
        artifact=artifact, harness=harness, phase_b=phase_b,
        scratch_dir=scratch_dir, run_dir=run_dir, **kw)
    return a1_core.grade(bundle, self_verified=self_verified)
