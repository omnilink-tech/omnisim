"""Real-engine motion programmes for the AgenticSimBench R2 red fixtures."""

from __future__ import annotations

import math
import sys

from omnisim import Supervisor


TARGETS = (
    (0.45, 0.0, 0.45),
    (0.35, 0.25, 0.35),
    (0.35, -0.25, 0.50),
)
START = (0.20, 0.0, 0.35)


def _lerp(a, b, u):
    return [a[i] + (b[i] - a[i]) * u for i in range(3)]


def _late_position(t):
    """A continuous sequence whose third dwell completes after 30 seconds."""
    segments = (
        (28.0, 28.7, START, TARGETS[0]),
        (29.3, 30.0, TARGETS[0], TARGETS[1]),
        (30.6, 31.3, TARGETS[1], TARGETS[2]),
    )
    current = START
    for begin, end, source, target in segments:
        if t < begin:
            return list(current)
        if t <= end:
            return _lerp(source, target, (t - begin) / (end - begin))
        current = target
    return list(current)


def _jump_position(mode, t):
    if mode == "teleport":
        if t < 1.0:
            return START
        if t < 2.0:
            return TARGETS[0]
        if t < 3.0:
            return TARGETS[1]
        return TARGETS[2]
    if mode == "wrong_order":
        if t < 1.0:
            return START
        if t < 2.0:
            return TARGETS[1]
        if t < 3.0:
            return TARGETS[0]
        return TARGETS[2]
    return START


def main():
    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())
    mode = sys.argv[1] if len(sys.argv) > 1 else "static"
    tip = robot.getFromDef("TIP")
    if tip is None:
        raise RuntimeError("R2 fixture TIP is missing")
    translation = tip.getField("translation")
    while robot.step(dt) != -1:
        t = robot.getTime()
        if mode == "late":
            position = _late_position(t)
        else:
            position = _jump_position(mode, t)
        if not all(math.isfinite(v) for v in position):
            raise RuntimeError("non-finite R2 fixture position")
        translation.setSFVec3f(list(position))


if __name__ == "__main__":
    main()
