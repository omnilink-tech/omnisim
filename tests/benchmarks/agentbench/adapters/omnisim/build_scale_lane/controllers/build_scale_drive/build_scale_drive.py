"""BuildScale oracle controller: ordinary wheel actuation, no Supervisor API."""

from omnisim import Robot


MOTORS = ("left front motor", "left rear motor",
          "right front motor", "right rear motor")


robot = Robot()
step_ms = int(robot.getBasicTimeStep())
motors = [robot.getDevice(name) for name in MOTORS]
for motor in motors:
    motor.setPosition(float("inf"))
    motor.setVelocity(4.0)

while robot.step(step_ms) != -1:
    pass
