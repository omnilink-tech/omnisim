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

"""apply_wrench probe for tests/test_wrench_api.py.

Two arms on two identical free bodies in a gravity-0 world, so the only thing
that can move either one is the wrench:

    SUSTAINED   apply_wrench(..., duration_s=T) + tick() every step
                -> delta-v = F*T/m         (the documented behaviour)

    ONE_SHOT    apply_wrench(...) with no duration, then the same number of
                idle steps
                -> delta-v = F*dt/m        (one step of impulse, then coasting)

The one-shot arm is the DEGENERATE arm: it is what the sustained arm collapses
to if tick() ever stops re-applying. Their ratio is the step count, so a green
run means the re-application is really reaching the solver rather than the test
merely agreeing with itself.

The values ARE the verdict; assertions live in the test.
Output: OMNISIM_WRENCH_PROBE_OUT.
"""
import json
import os
import sys

from omnisim import Supervisor
from omnisim.wrench import apply_wrench

FORCE_N = 10.0
DURATION_S = 0.4

robot = Supervisor()
dt_ms = int(robot.getBasicTimeStep())
dt_s = dt_ms / 1000.0

sustained = robot.getFromDef("SUBJECT_SUSTAINED")
one_shot = robot.getFromDef("SUBJECT_ONE_SHOT")

out = {
    "dt_s": dt_s,
    "force_n": FORCE_N,
    "duration_s": DURATION_S,
}

# -- sustained arm ----------------------------------------------------------
# One step BEFORE the first application, deliberately. The engine free-runs
# until the controller's first step(), so a wrench applied ahead of that lands
# in that window and is never integrated -- measured, it costs exactly one of
# the N applications (49 impulses from 50 calls). Nothing about apply_wrench
# causes it and nothing in it can detect it; it is a property of when the
# controller joins the run. Stepping once first makes the arm measure the
# helper rather than that startup window.
robot.step(dt_ms)

# tick() is called AFTER the step the previous application acted on, which is
# the usage the docstring documents.
w = apply_wrench(sustained, force=[0.0, 0.0, FORCE_N], frame="world",
                 duration_s=DURATION_S)
steps = 0
while robot.step(dt_ms) != -1 and w.tick():
    steps += 1
out["sustained_applications"] = w.applications
out["sustained_ticked_steps"] = steps
out["sustained_velocity"] = list(sustained.getVelocity()[:3])

# -- one-shot (degenerate) arm ---------------------------------------------
one = apply_wrench(one_shot, force=[0.0, 0.0, FORCE_N], frame="world")
out["one_shot_first_tick"] = bool(one.tick())
for _ in range(steps):
    if robot.step(dt_ms) == -1:
        break
out["one_shot_applications"] = one.applications
out["one_shot_velocity"] = list(one_shot.getVelocity()[:3])

path = os.environ.get("OMNISIM_WRENCH_PROBE_OUT", "wrench_probe_out.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, indent=1)
sys.stdout.write("wrench_probe: wrote %s\n" % path)
sys.stdout.flush()
