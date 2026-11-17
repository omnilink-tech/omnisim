"""Straight physical drive for an upstream Webots Pioneer 3-AT."""

from controller import Robot


robot = Robot()
step_ms = int(robot.getBasicTimeStep())
wheels = [robot.getDevice(name) for name in
          ("front left wheel", "front right wheel",
           "back left wheel", "back right wheel")]
for wheel in wheels:
    wheel.setPosition(float("inf"))
    wheel.setVelocity(4.0)
while robot.step(step_ms) != -1:
    pass
