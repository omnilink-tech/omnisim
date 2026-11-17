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

"""omnibench_emitter — lane-4 radio transmitter for the Emitter/Receiver probe.

A SECOND robot exists solely to hold the Emitter, because the engine filters
same-robot delivery by design:

    // robot cannot send message to self
    if (eRobot != rRobot) { ... }        -- src/omnisim/nodes/OmReceiver.cpp

The first version of this probe put both devices on the prober and scored the
capability `broken` when nothing arrived. It was measuring a documented rule,
not a defect. Reading the dispatch code was what settled it.

Sends one packet per step on the configured channel, for the whole run.
"""

import sys

from omnisim import Robot


def main():
    robot = Robot()
    step_ms = int(round(robot.getBasicTimeStep())) or 1
    name = "emit"
    for a in sys.argv[1:]:
        if a.startswith("--device="):
            name = a.split("=", 1)[1]
    emitter = robot.getDevice(name)
    if emitter is None:
        print("[omnibench_emitter] no Emitter named %r" % name, flush=True)
    while robot.step(step_ms) != -1:
        if emitter is not None:
            emitter.send(b"omnibench")


main()
