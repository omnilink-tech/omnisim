"""Real-engine trajectories for AgenticSimBench v1 R4 negative controls."""

from __future__ import annotations

import math
import sys

from omnisim import Supervisor


START = (-4.0, -4.0, 0.0)
TABLE_BASE = (2.5, 3.0, 0.0)
PAD_BASE = (-3.5, 3.5, 0.0)
PAYLOAD_START = (3.0, 3.0, 0.625)
PAYLOAD_AIR = (3.0, 3.0, 0.90)
PAYLOAD_PAD_AIR = (-3.0, 3.5, 0.90)
PAYLOAD_PAD = (-3.0, 3.5, 0.125)
WRONG_PAD = (-2.45, 3.5, 0.125)

TO_TABLE = (
    START,
    (-3.5, -1.0, 0.0),
    (-3.0, 2.2, 0.0),
    (1.4, 2.5, 0.0),
    TABLE_BASE,
)
TO_PAD = (
    TABLE_BASE,
    (1.5, 2.5, 0.0),
    (-2.0, 2.4, 0.0),
    PAD_BASE,
)
COLLISION_TO_TABLE = (
    START,
    (-0.152, -1.8888, 0.0),
    (-3.0, 2.2, 0.0),
    (1.4, 2.5, 0.0),
    TABLE_BASE,
)


def lerp(a, b, u):
    return tuple(a[i] + (b[i] - a[i]) * u for i in range(3))


def path_position(path, u):
    lengths = [
        math.dist(path[i], path[i + 1]) for i in range(len(path) - 1)
    ]
    total = sum(lengths)
    d = max(0.0, min(1.0, u)) * total
    for i, length in enumerate(lengths):
        if d <= length:
            return lerp(path[i], path[i + 1], d / length)
        d -= length
    return path[-1]


def smooth_sequence(mode, t):
    base = START
    payload = PAYLOAD_START
    if t < 15.0:
        route = COLLISION_TO_TABLE if mode == "collision" else TO_TABLE
        base = path_position(route, t / 15.0)
    elif t < 18.0:
        base = TABLE_BASE
        payload = lerp(PAYLOAD_START, PAYLOAD_AIR, (t - 15.0) / 3.0)
    elif t < 30.0:
        u = (t - 18.0) / 12.0
        base = path_position(TO_PAD, u)
        # A real carried body is rigid in the moving base frame even while the
        # base follows a bent route; interpolating both world poses separately
        # only agrees at the endpoints and correctly fails the grasp grader.
        payload = (base[0] + 0.5, base[1], base[2] + 0.9)
    elif t < 33.0:
        base = PAD_BASE
        payload = lerp(PAYLOAD_PAD_AIR, PAYLOAD_PAD, (t - 30.0) / 3.0)
    else:
        base, payload = PAD_BASE, PAYLOAD_PAD

    if mode == "bad_start" and t < 15.0:
        # Start outside R4.4's 4 cm tolerance, then return continuously before
        # pickup so this fixture does not manufacture a teleport failure.
        offset = 0.10 if t < 12.0 else 0.10 * (15.0 - t) / 3.0
        payload = (PAYLOAD_START[0] + offset, PAYLOAD_START[1],
                   PAYLOAD_START[2])
    if mode == "no_carry":
        payload = PAYLOAD_START
    if mode == "wrong_delivery" and t >= 30.0:
        u = min(1.0, max(0.0, (t - 30.0) / 3.0))
        payload = lerp(PAYLOAD_PAD_AIR, WRONG_PAD, u)
    if mode == "reacquire" and 24.0 <= t < 25.6:
        # A continuous, rate-bounded drop to the floor, a >=0.5 s rest, then
        # a second lift.  The first carry segment is still long enough to
        # prove R4.6 independently.
        xy = (base[0] + 0.5, base[1])
        if t < 24.5:
            z = 0.9 + (0.025 - 0.9) * ((t - 24.0) / 0.5)
        elif t < 25.1:
            z = 0.025
        else:
            z = 0.025 + (0.9 - 0.025) * ((t - 25.1) / 0.5)
        payload = (xy[0], xy[1], z)
    return base, payload


def teleport_sequence(t):
    if t < 2.0:
        return START, PAYLOAD_START
    if t < 5.0:
        return TABLE_BASE, PAYLOAD_AIR
    if t < 8.0:
        return PAD_BASE, PAYLOAD_PAD_AIR
    return PAD_BASE, PAYLOAD_PAD


def main():
    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())
    mode = sys.argv[1] if len(sys.argv) > 1 else "oracle"
    base_node = robot.getSelf()
    payload_node = robot.getFromDef("PAYLOAD")
    links = [robot.getFromDef("R4_LINK%d" % i) for i in (1, 2, 3)]
    motors = [robot.getDevice("r4_joint_%d" % i) for i in range(1, 7)]
    if (payload_node is None or any(link is None for link in links)
            or any(motor is None for motor in motors)):
        raise RuntimeError("R4 fixture nodes are missing")
    base_field = base_node.getField("translation")
    payload_field = payload_node.getField("translation")

    while robot.step(dt) != -1:
        t = robot.getTime()
        if mode == "teleport":
            base, payload = teleport_sequence(t)
        else:
            base, payload = smooth_sequence(mode, t)
        # These are genuine joint transforms, not field-authored link poses.
        # Joint 3 returns to zero before pickup, placing its 0.5/0/0.9-m
        # endpoint exactly at the payload's rigid carry offset.
        angles = [
            0.55 * math.sin(0.45 * t),
            0.50 * math.sin(0.37 * t + 0.4),
            0.35 * math.sin(0.7 * t) if t < 10.0 else 0.0,
            0.45 * math.sin(0.31 * t + 0.8),
            0.40 * math.sin(0.29 * t + 1.1),
            0.35 * math.sin(0.41 * t + 1.5),
        ]
        for motor, angle in zip(motors, angles):
            motor.setPosition(angle)
        base_field.setSFVec3f(list(base))
        payload_field.setSFVec3f(list(payload))
        base_node.resetPhysics()
        payload_node.resetPhysics()


if __name__ == "__main__":
    main()
