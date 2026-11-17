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

"""Minimal skid-steer drive for the Pioneer 3-AT bring-up world.

Deliberately sensor-free (webots-control-baseline.md sec. 5 trap 2: lidar is
GPU-bound and dominates a careless comparison) and deterministic per robot
name, so the two robots in the bring-up world trace two different gentle arcs
and the recorder has real displacement to measure. This is bring-up
scaffolding, not a benchmark behaviour.
"""

from controller import Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())

wheels = [robot.getDevice(n) for n in
          ("front left wheel", "front right wheel",
           "back left wheel", "back right wheel")]
vmax = min(m.getMaxVelocity() for m in wheels)
if vmax != vmax or vmax == float("inf") or vmax > 60.0:
    vmax = 6.0   # guard: an unset maxVelocity would send NaN into ODE

# Per-robot arc: robot "p3at0" drives a slight left arc, "p3at1" a slight
# right arc, anything else straight.
name = robot.getName()
bias = {"p3at0": 0.10, "p3at1": -0.10}.get(name, 0.0)
v = 0.6 * vmax
vl = max(-vmax, min(vmax, v * (1.0 - bias)))
vr = max(-vmax, min(vmax, v * (1.0 + bias)))

for m in wheels:
    m.setPosition(float("inf"))
for m in (wheels[0], wheels[2]):
    m.setVelocity(vl)
for m in (wheels[1], wheels[3]):
    m.setVelocity(vr)

while robot.step(dt) != -1:
    pass
