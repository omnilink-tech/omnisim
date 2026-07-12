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

"""UR5e slave — applies joint angle commands sent via customData.

The supervisor writes JSON {"angles": [6 floats]} into this robot's
customData and the slave commands each motor at its per-joint URDF
velocity limit. Real joint angles are read directly from the scene
tree by the supervisor (via the HingeJoint jointParameters nodes),
so no echo channel is needed here.
"""

import json

from controller import Robot

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Per-joint velocity caps from ur5e_reference.urdf. The first three
# joints are limited to 3.14 rad/s, the wrists to 6.28 rad/s.
JOINT_VELOCITIES = [3.14, 3.14, 3.14, 6.28, 6.28, 6.28]


def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    motors = []
    for name, v_max in zip(JOINT_NAMES, JOINT_VELOCITIES):
        motor = robot.getDevice(f"{name}_motor")
        motor.setVelocity(v_max)
        motors.append(motor)

    last_payload = ""
    while robot.step(time_step) != -1:
        payload = robot.getCustomData()
        if payload == last_payload or not payload:
            continue
        last_payload = payload
        try:
            command = json.loads(payload)
        except ValueError:
            continue
        angles = command.get("angles")
        if not angles or len(angles) != 6:
            continue
        for motor, target in zip(motors, angles):
            motor.setPosition(float(target))


if __name__ == "__main__":
    main()
