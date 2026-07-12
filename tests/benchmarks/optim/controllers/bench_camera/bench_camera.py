"""Minimal robot controller used by camera-heavy bench worlds.

Enables the `front_camera` device (and optionally a `front_depth`
RangeFinder) at every step. The point is to force per-step sensor
rendering without doing any non-trivial controller-side work, so the
benchmark isolates simulator render + IPC cost.
"""

from __future__ import annotations

from controller import Robot


def main() -> None:
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())

    cam = robot.getDevice("front_camera")
    if cam is not None:
        cam.enable(time_step)

    depth = robot.getDevice("front_depth")
    if depth is not None:
        depth.enable(time_step)

    while robot.step(time_step) != -1:
        pass


if __name__ == "__main__":
    main()
