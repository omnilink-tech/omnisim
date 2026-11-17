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

"""R1 BLIND driver -- drives at the goal and never reads the LiDAR.

The negative control that makes R1.5 mean something. The null proves a task
cannot be passed by doing nothing; this proves the COLLISION assertion can
actually fire, which is a different claim and the one that was false on this
arm until 2026-08-09: the recorder queried only Robot nodes for contacts, so a
robot/obstacle contact had no second participant, could not be named, and R1.5
reported "0 collisions" for every run including the ones that ploughed straight
into a box.

It is a full-speed heading controller onto (4, 4) using the same GPS and
InertialUnit the oracle uses -- so the ONLY difference between this and the
oracle is that this one never calls ``getRangeImage()``. The straight line
from (-4, -4) to (4, 4) passes through OBSTACLE_2 at about (-1.8, -1.8) and
then OBSTACLE_1, so it arrives nowhere and hits something on the way.
"""

import math

from controller import Robot

GOAL_XY = (4.0, 4.0)
WHEELS = ("front left wheel", "front right wheel",
          "back left wheel", "back right wheel")
WHEEL_RADIUS_M = 0.11
HALF_TRACK_M = 0.197
MAX_WHEEL_RAD_S = 6.4
V_MAX_MPS = 0.62
K_HEADING = 2.2

robot = Robot()
dt = int(robot.getBasicTimeStep())
gps = robot.getDevice("gps")
gps.enable(dt)
imu = robot.getDevice("inertial unit")
imu.enable(dt)
wheels = [robot.getDevice(w) for w in WHEELS]
for m in wheels:
    m.setPosition(float("inf"))
    m.setVelocity(0.0)

print("[blind] driving at the goal; the LiDAR is never read", flush=True)

while robot.step(dt) != -1:
    p = gps.getValues()
    x, y = float(p[0]), float(p[1])
    yaw = float(imu.getRollPitchYaw()[2])
    err = math.atan2(GOAL_XY[1] - y, GOAL_XY[0] - x) - yaw
    err = (err + math.pi) % (2 * math.pi) - math.pi
    w = max(-1.4, min(1.4, K_HEADING * err))
    v = 0.0 if abs(err) > 0.7 else V_MAX_MPS * math.cos(err)
    wl = (v - w * HALF_TRACK_M) / WHEEL_RADIUS_M
    wr = (v + w * HALF_TRACK_M) / WHEEL_RADIUS_M
    scale = max(1.0, abs(wl) / MAX_WHEEL_RAD_S, abs(wr) / MAX_WHEEL_RAD_S)
    wheels[0].setVelocity(wl / scale)
    wheels[2].setVelocity(wl / scale)
    wheels[1].setVelocity(wr / scale)
    wheels[3].setVelocity(wr / scale)
