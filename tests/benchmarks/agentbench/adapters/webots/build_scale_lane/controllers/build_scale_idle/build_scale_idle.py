"""Matched upstream Webots null: controller runs but commands no motion."""

from controller import Robot


robot = Robot()
step_ms = int(robot.getBasicTimeStep())
while robot.step(step_ms) != -1:
    pass
