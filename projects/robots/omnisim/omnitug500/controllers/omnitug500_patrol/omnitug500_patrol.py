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

"""OMNITUG500 patrol controller.

The OMNITUG500 is a low 4-wheeled rover. This supervisor drives the rover
itself around a smooth elliptical loop on the floor by writing its own
translation/rotation fields every tick (getSelf -> setSFVec3f /
setSFRotation). The mesh is in its true orientation -- wheels on the
underside touch the floor at mesh z=0 -- so we drive at world z=0 and the
wheels stay on the ground.

This is scripted kinematic motion (no wheel-torque physics), but it is the
rover body itself moving, oriented correctly, with the wheels on the floor.

The rover's forward axis is its length, local +Y. With yaw=0 the rover's
+Y points to world +Y (angle pi/2), so to face a world heading ``h`` we
rotate about +Z by ``h - pi/2``.
"""

import math

from omnisim import Supervisor

# --- path / motion tuning --------------------------------------------------
A = 3.6                               # ellipse semi-axis along X (m)
B = 2.3                               # ellipse semi-axis along Y (m)
FLOOR_Z = 0.0                         # wheels touch floor at mesh z=0 -> drive at z=0
PERIOD = 18.0                         # seconds for one full loop
TURN_RATE_LIMIT = math.radians(150)   # max yaw slew (rad/s) for a smooth turn
FORWARD_OFFSET = -math.pi / 2.0       # local +Y forward -> world heading offset


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def main() -> None:
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    me = robot.getSelf()
    tf = me.getField("translation")
    rf = me.getField("rotation")

    w = 2.0 * math.pi / PERIOD   # phase rate along the ellipse
    phi = -math.pi / 2.0         # start on the near side (y = -B)
    yaw = None

    while robot.step(dt_ms) != -1:
        phi += w * dt
        x = A * math.cos(phi)
        y = B * math.sin(phi)

        # tangent (velocity) direction -> desired heading
        vx = -A * math.sin(phi)
        vy = B * math.cos(phi)
        target_yaw = math.atan2(vy, vx) + FORWARD_OFFSET

        if yaw is None:
            yaw = target_yaw
        else:
            step = max(-TURN_RATE_LIMIT * dt,
                       min(TURN_RATE_LIMIT * dt, wrap(target_yaw - yaw)))
            yaw = wrap(yaw + step)

        tf.setSFVec3f([x, y, FLOOR_Z])
        rf.setSFRotation([0.0, 0.0, 1.0, yaw])


if __name__ == "__main__":
    main()
