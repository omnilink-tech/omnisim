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

"""Step the world exactly N times, then quit -- for ANY world.

Same contract as `tests/benchmarks/omnibench/step_cost/controllers/
step_cost_runner`, but it lives in `projects/default/controllers/` so it is
resolvable from a world in ANY project directory. That is what lets the
two-window `--log-performance` differencing method (the one step_cost uses to
cancel finalize and warm-up costs exactly) be applied to a SHIPPED world --
a Husky swarm, a G1 deploy -- instead of only to the generated box stacks.

Closing that gap matters because box worlds are the least representative
robotics workload in the tree: they have no motors, so they never exercise
the per-tick control-drain path, which is where a motorised world spends its
per-joint solver-API calls.

Like its sibling it deliberately reads NOTHING -- no poses, sensors or
contacts. Every such read is an FFI round trip that would land in the
engine's `postPhysics` bucket and contaminate the number being measured.

    --steps N   basic timesteps to run before quitting (default 1000)
"""
import sys

from omnisim import Supervisor


def main():
    steps = 1000
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == "--steps" and i + 1 < len(argv):
            try:
                steps = int(argv[i + 1])
            except ValueError:
                pass
    if steps <= 0:
        steps = 1000

    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())

    done = 0
    while done < steps:
        if robot.step(dt) == -1:
            break
        done += 1

    sys.stdout.write("perf_window_runner: stepped %d\n" % done)
    sys.stdout.flush()
    robot.simulationQuit(0)


if __name__ == "__main__":
    main()
