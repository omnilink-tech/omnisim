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

"""OmniBot keyboard-controlled differential drive.

Controls:
  W / Up Arrow      : forward
  S / Down Arrow    : backward
  A / Left Arrow    : turn left
  D / Right Arrow   : turn right
  Space             : brake
  +/-               : change cruise speed
"""

from omnisim import Robot, Keyboard

MAX_SPEED = 6.28
DEFAULT_CRUISE = 4.0
TURN_GAIN = 0.7  # how much spin overrides forward when turning


def clamp(value, low, high):
    return max(low, min(high, value))


def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    # URDF importer generates "<joint_name>_motor" device names.
    left_motor = robot.getDevice("left_wheel_joint_motor")
    right_motor = robot.getDevice("right_wheel_joint_motor")
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    keyboard = robot.getKeyboard()
    keyboard.enable(time_step)

    cruise = DEFAULT_CRUISE
    last_help_print = 0

    print("=" * 56)
    print("OmniBot keyboard control")
    print("  W / Up Arrow    : forward")
    print("  S / Down Arrow  : backward")
    print("  A / Left Arrow  : turn left")
    print("  D / Right Arrow : turn right")
    print("  Space           : brake")
    print("  +/-             : adjust cruise speed")
    print("=" * 56)
    print(f"Cruise speed: {cruise:.1f}")
    print("Click the 3D view first, then press keys.")

    while robot.step(time_step) != -1:
        forward = 0.0
        spin = 0.0

        # Read all currently pressed keys
        keys = []
        k = keyboard.getKey()
        while k != -1:
            keys.append(k)
            k = keyboard.getKey()

        for key in keys:
            ch = key & 0xFFFF  # strip modifier bits
            if ch == ord('W') or ch == Keyboard.UP:
                forward += 1.0
            elif ch == ord('S') or ch == Keyboard.DOWN:
                forward -= 1.0
            elif ch == ord('A') or ch == Keyboard.LEFT:
                spin -= 1.0
            elif ch == ord('D') or ch == Keyboard.RIGHT:
                spin += 1.0
            elif ch == ord(' '):
                forward = 0.0
                spin = 0.0
            elif ch == ord('+') or ch == ord('='):
                cruise = clamp(cruise + 0.5, 0.5, MAX_SPEED)
                print(f"Cruise speed: {cruise:.1f}")
            elif ch == ord('-') or ch == ord('_'):
                cruise = clamp(cruise - 0.5, 0.5, MAX_SPEED)
                print(f"Cruise speed: {cruise:.1f}")

        # Differential drive mixing
        forward_speed = forward * cruise
        turn_speed = spin * cruise * TURN_GAIN

        left_speed = clamp(forward_speed - turn_speed, -MAX_SPEED, MAX_SPEED)
        right_speed = clamp(forward_speed + turn_speed, -MAX_SPEED, MAX_SPEED)

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    main()
