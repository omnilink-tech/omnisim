# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0 (the "License");

"""Null driver: every controller instance runs, but commands no actuator."""

from controller import Robot


robot = Robot()
step_ms = int(robot.getBasicTimeStep())
while robot.step(step_ms) != -1:
    pass
