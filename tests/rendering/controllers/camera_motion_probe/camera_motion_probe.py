# Copyright 2026 OmniLink. Licensed under the Apache License, Version 2.0.
"""Photograph a scene three times while things in it move, and save the raw frames.

Frame 0 at rest; frame 1 after physics has dropped BALL a few centimetres; frame 2 one
step after the supervisor moved BLOCK's Pose. The test compares these frames with each
other and with the same run under OMNISIM_WGPU_SENSOR_DRAW_CACHE=0.
argv: <result.json> <frame directory>
"""
import json
import sys
from pathlib import Path

from omnisim import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
camera = robot.getDevice("camera")
camera.enable(step)
block = robot.getFromDef("BLOCK").getField("translation")
ball = robot.getFromDef("BALL")
out = Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)


def advance(steps):
    for _ in range(steps):
        if robot.step(step) == -1:
            raise RuntimeError("simulation ended before the capture")


def capture(name):
    data = camera.getImage()
    w, h = camera.getWidth(), camera.getHeight()
    if not data or len(data) != w * h * 4:
        raise RuntimeError("camera did not return a complete image")
    rgb = bytearray(w * h * 3)  # BGRA -> RGB
    rgb[0::3], rgb[1::3], rgb[2::3] = data[2::4], data[1::4], data[0::4]
    (out / (name + ".ppm")).write_bytes(b"P6 %d %d 255\n" % (w, h) + bytes(rgb))
    return {"name": name, "time": robot.getTime(), "ball": ball.getPosition(), "block": block.getSFVec3f()}


try:
    frames = []
    advance(4)  # BALL is held by nothing: it starts falling on the first step
    frames.append(capture("f0_rest"))
    advance(6)
    frames.append(capture("f1_ball_fell"))
    p = block.getSFVec3f()
    block.setSFVec3f([p[0], p[1] + 0.35, p[2]])
    advance(1)
    frames.append(capture("f2_block_moved"))
    Path(sys.argv[1]).write_text(json.dumps({"frames": frames}, indent=2), encoding="utf-8")
    robot.simulationQuit(0)
except Exception as exc:
    Path(sys.argv[1]).write_text(json.dumps({"error": str(exc)}), encoding="utf-8")
    robot.simulationQuit(1)
