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

"""Newton motor smoke-test controller.

Drives the chassis-mounted wheel_motor at a constant 5 rad/s. Used by
projects/samples/demos/worlds/physics/newton_smoke_test.wbt to verify
WbRotationalMotor -> WbNewtonBackend::setJointTargetVelocity end-to-end.

Expected behaviour: with the runtime brought up (NEWTON=ON build +
warp/newton wheels installed), the wheel's body_q quaternion should
visibly rotate in the per-step log entries from WbNewtonBackend.

If you're seeing the assembly fall but the wheel not rotate, check
the omnisim_log.txt for either:
  - "[motorized: target_ke=100, target_kd=2]" on the joint (wireup fired)
  - or "[free-spinning]" (the joint was registered without a motor)
"""

from omnisim import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor = robot.getDevice("wheel_motor")
# setPosition(inf) puts the motor in velocity-control mode -- the
# documented Webots idiom for "spin at a target velocity, no PID
# position control".
motor.setPosition(float("inf"))

# Wait ~1 sec (60 ticks at 16ms) before driving. Without this delay
# Newton's XPBD actuator yanks the wheel up to 5 rad/s on the first
# tick while the assembly is still mid-air; the hinge's reaction
# torque flings the light chassis upward before gravity recovers.
# Letting it land first gives clean visible rolling instead of an
# apparent "explosion".
SETTLE_TICKS = 60
TARGET_VEL = 5.0

motor.setVelocity(0.0)
ticks = 0
while robot.step(timestep) != -1:
    ticks += 1
    if ticks == SETTLE_TICKS:
        motor.setVelocity(TARGET_VEL)
