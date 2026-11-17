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

"""Drive ONE motor for the wgpu render-gate worlds (P2 of the WREN-deletion runbook).

WHY THIS EXISTS RATHER THAN REUSING THE SAMPLE CONTROLLERS. The Track gate world wants
`projects/samples/devices/controllers/track_conveyor_belt` and the Muscle gate world wants
`projects/samples/geometries/controllers/muscle`, but controller lookup is PROJECT-relative
and both gate worlds live under `projects/samples/demos/`. The first attempt used those
names and the engine logged `Could not find the controller directory` -- the belt then sat
still and the gate's ANIMATION arm failed for a reason that had nothing to do with the
renderer. A Python controller also needs no per-project compile, which the C originals do.

Usage (controllerArgs):
    ["--motor" "linear motor" "--mode" "ramp"  "--step" "0.0008"]
    ["--motor" "muscle"       "--mode" "swing" "--max" "2.0" "--step" "0.4"]

ramp   monotonically increases the position target -- a conveyor belt at constant surface
       velocity, which is what makes a Track's belt elements circulate.
swing  bounces the target between 0 and --max off the joint's own PositionSensor -- the
       schedule projects/samples/geometries/controllers/muscle.c uses, so the Muscle
       stretches and (constant volume) thins.
"""

import sys

from omnisim import Robot


def arg(name, default=None):
    if name in sys.argv:
        i = sys.argv.index(name)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return default


robot = Robot()
ts = int(robot.getBasicTimeStep())

motor_name = arg("--motor", "linear motor")
mode = arg("--mode", "ramp")
step = float(arg("--step", "0.0008"))
maximum = float(arg("--max", "2.0"))

motor = robot.getDevice(motor_name)
if motor is None:
    sys.stderr.write("wgpu_gate_driver: no motor named %r\n" % motor_name)
    while robot.step(ts) != -1:
        pass
    sys.exit(0)

sensor = None
try:
    sensor = motor.getPositionSensor()
except Exception:
    sensor = None
if sensor is not None:
    sensor.enable(ts)

p = 0.0
dp = step
while robot.step(ts) != -1:
    if mode == "swing" and sensor is not None:
        pos = sensor.getValue()
        if pos != pos:  # NaN before the first sample
            pos = p
        if pos <= 0.0:
            dp = -step
        elif pos >= maximum:
            dp = step
        p = pos - dp
    else:
        p += step
    motor.setPosition(p)
