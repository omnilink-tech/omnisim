#!/usr/bin/env python3
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

"""Step the world exactly N times, then quit. Nothing else.

This controller exists to give `--log-performance` a bounded, IDENTICAL run to
measure on both backends. It deliberately does NOT read poses, sensors or
contacts: every such read is an FFI round trip that would land in the engine's
`postPhysics` bucket and contaminate the very number this bench reports.

It must also be the same cost on both backends, which is why it is a plain
Robot and not a Supervisor -- `simulationQuit` needs a Supervisor, so the
world grants it, but the step loop touches no supervisor API.

    --steps N   how many basic timesteps to run before quitting (required)
"""
import sys

try:  # `omnisim` is the only module name at HEAD (the `controller`
    from omnisim import Supervisor  # alias was deleted 2026-08-16)
except ImportError:  # older trees (e.g. a RunPod volume) still ship it
    from controller import Supervisor


def main():
    steps = 0
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == "--steps" and i + 1 < len(argv):
            steps = int(argv[i + 1])
    if steps <= 0:
        sys.stderr.write("step_cost_runner: --steps N (N>0) is required\n")
        sys.stderr.flush()
        steps = 1000

    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())

    done = 0
    while done < steps:
        if robot.step(dt) == -1:
            break
        done += 1

    sys.stdout.write("step_cost_runner: stepped %d\n" % done)
    sys.stdout.flush()
    robot.simulationQuit(0)


if __name__ == "__main__":
    main()
