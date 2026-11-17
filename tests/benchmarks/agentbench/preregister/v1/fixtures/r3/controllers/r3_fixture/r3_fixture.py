"""Measured cube trajectories for AgenticSimBench v1 R3 negative controls."""

from __future__ import annotations

import math
import sys

from omnisim import Supervisor


AUTHORED = (0.45, 0.0, 0.8)
REST = (0.45, 0.0, 0.775)
AIR_START = (0.45, 0.0, 1.0)
AIR_BIN = (0.30, 0.35, 1.0)
BIN_REST = (0.30, 0.35, 0.785)
WRONG_REST = (0.15, 0.20, 0.775)
BAD_START = (0.65, 0.0, 0.775)


def _lerp(a, b, u):
    return [a[i] + (b[i] - a[i]) * u for i in range(3)]


def _timeline(t, segments, initial):
    current = initial
    for begin, end, source, target in segments:
        if t < begin:
            return list(current)
        if t <= end:
            return _lerp(source, target, (t - begin) / (end - begin))
        current = target
    return list(current)


def position(mode, t):
    if mode == "bad_start":
        return _timeline(
            t,
            ((0.2, 0.8, AUTHORED, BAD_START),),
            AUTHORED,
        )
    if mode == "never_released":
        return _timeline(
            t,
            (
                (2.0, 3.0, REST, AIR_START),
                (3.0, 4.0, AIR_START, AIR_BIN),
            ),
            REST,
        )
    if mode == "wrong_destination":
        return _timeline(
            t,
            (
                (2.0, 3.0, REST, AIR_START),
                (3.0, 4.0, AIR_START, (0.15, 0.20, 1.0)),
                (4.0, 5.0, (0.15, 0.20, 1.0), WRONG_REST),
            ),
            REST,
        )
    if mode == "teleport":
        if t < 2.0:
            return REST
        if t < 2.7:
            return AIR_START
        if t < 3.4:
            return AIR_BIN
        return BIN_REST
    return None


def main():
    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())
    mode = sys.argv[1] if len(sys.argv) > 1 else "no_lift"
    cube = robot.getFromDef("CUBE")
    if cube is None:
        raise RuntimeError("R3 fixture CUBE is missing")
    translation = cube.getField("translation")
    while robot.step(dt) != -1:
        target = position(mode, robot.getTime())
        if target is None:
            continue
        if not all(math.isfinite(v) for v in target):
            raise RuntimeError("non-finite R3 fixture position")
        translation.setSFVec3f(list(target))
        cube.resetPhysics()


if __name__ == "__main__":
    main()
