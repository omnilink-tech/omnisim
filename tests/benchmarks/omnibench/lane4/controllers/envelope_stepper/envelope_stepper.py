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

"""envelope_stepper — lane-4b step driver.

Advances the world a fixed number of basic timesteps and quits. It reads
NOTHING: every pose, sensor or contact read is an FFI round trip that would
land in the engine's `postPhysics` bucket and contaminate the per-step cost
this lane is measuring (the same rule step_cost's runner follows).

    --steps=N   basic timesteps to advance before simulationQuit(0)
"""

import sys

from omnisim import Supervisor


def main():
    sv = Supervisor()
    step_ms = int(round(sv.getBasicTimeStep())) or 1
    steps = 1200
    for a in sys.argv[1:]:
        if a.startswith("--steps="):
            steps = int(a.split("=", 1)[1])
    for _ in range(steps):
        if sv.step(step_ms) == -1:
            break
    # Without this the engine free-runs to the runner's timeout instead of
    # exiting, which turns a 15 s measurement into a 240 s one.
    sv.simulationQuit(0)
    sv.step(step_ms)


main()
