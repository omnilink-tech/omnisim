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

"""C1 ``parse_error_fix`` -- the task entry point (debug tier).

Thin by design: it resolves the adapter for the simulator under test, asks it
for a neutral evidence bundle, and hands that to
:mod:`agentbench.graders.c1_core`, where the three assertions and their
thresholds live.

Like C2, C1 declares ``any_robot`` as its identity predicate: the task ships
the world, so "is this the right robot" is not the question -- "is the scene
the task shipped still whole, loaded, and stepping" is. The intended
inventory comes from the task's own ``reference_roster.json`` (kept outside
``initial/`` so it is never copied into the agent's scratch dir), and the
unit tests assert it agrees with ``c1_core.EXPECTED_BODIES`` so the file and
the core constant cannot drift apart.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench import adapters  # noqa: E402
from agentbench.graders import c1_core  # noqa: E402
from agentbench.graders.c1_core import (  # noqa: E402,F401  (re-exported)
    EXPECTED_BODIES, MAX_ABS_COORD_M, MIN_RECORDED_S, ROBOT_NAME, TASK)

ROBOT_IDENTITY = "any_robot"

TASK_DIR = Path(__file__).resolve().parents[1] / "tasks" / TASK
ROSTER_FILE = TASK_DIR / "reference_roster.json"


def load_reference_roster(path=ROSTER_FILE):
    """The intended body inventory, from the task's own reference file.

    Falls back to the core constant if the file is unreadable -- the grader
    must never award a PASS because its own roster went missing.
    """
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        bodies = tuple(data["intended_bodies"])
        return bodies if bodies else EXPECTED_BODIES
    except (OSError, KeyError, ValueError):
        return EXPECTED_BODIES


def grade(run_dir, *, artifact=None, phase_b=None, self_verified=False,
          scratch_dir=None, sim=None, **kw):
    adapter = adapters.resolve(sim)
    bundle = adapter.build_bundle(
        TASK, robot_identity=ROBOT_IDENTITY, live_expected=False,
        artifact=artifact, phase_b=phase_b, scratch_dir=scratch_dir,
        run_dir=run_dir, **kw)
    return c1_core.grade(bundle, self_verified=self_verified,
                         expected_bodies=load_reference_roster())
