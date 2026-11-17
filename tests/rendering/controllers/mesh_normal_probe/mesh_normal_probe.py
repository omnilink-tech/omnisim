# Copyright 2026 OmniLink. Licensed under the Apache License, Version 2.0.
"""Photograph mesh A, then mesh B at the same spot, and compare the two frames.

A is a sphere on an untranslated node; B is the same sphere instanced under a node
translated far away, its Pose bringing it back to A's spot. Same surface, same
light, same camera: the frames match only if B's normals ignore the translation.
argv: <result.json> [<frame directory>]
"""
import json
import sys
from pathlib import Path

from omnisim import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
camera = robot.getDevice("camera")
camera.enable(step)
SPOT = [0.0, 0.0, 1.0]
AWAY = [0.0, 50.0, 1.0]
mesh_a = robot.getFromDef("MESH_A").getField("translation")
mesh_b = robot.getFromDef("MESH_B").getField("translation")
offset_b = mesh_b.getSFVec3f()  # as authored: brings B's instance to SPOT


def capture(name):
    for _ in range(8):
        if robot.step(step) == -1:
            raise RuntimeError("simulation ended before the capture")
    data = camera.getImage()
    w, h = camera.getWidth(), camera.getHeight()
    if not data or len(data) != w * h * 4:
        raise RuntimeError("camera did not return a complete image")
    bgr = bytes(v for i, v in enumerate(data) if i % 4 != 3)  # BGRA, alpha dropped
    if len(sys.argv) > 2:
        out = Path(sys.argv[2])
        out.mkdir(parents=True, exist_ok=True)
        ppm = bytearray(len(bgr))  # PPM wants RGB
        ppm[0::3], ppm[1::3], ppm[2::3] = bgr[2::3], bgr[1::3], bgr[0::3]
        (out / (name + ".ppm")).write_bytes(b"P6 %d %d 255\n" % (w, h) + bytes(ppm))
    return bgr


try:
    mesh_b.setSFVec3f([offset_b[0] + AWAY[0] - SPOT[0], offset_b[1] + AWAY[1] - SPOT[1], offset_b[2] + AWAY[2] - SPOT[2]])
    mesh_a.setSFVec3f(SPOT)
    frame_a = capture("a_untranslated_node")
    mesh_a.setSFVec3f(AWAY)
    mesh_b.setSFVec3f(offset_b)
    frame_b = capture("b_translated_node")
    diff = [abs(x - y) for x, y in zip(frame_a, frame_b)]
    result = {
        "mean_abs_diff": sum(diff) / len(diff),
        "max_abs_diff": max(diff),
        "frame_a_range": max(frame_a) - min(frame_a),
        "frame_b_range": max(frame_b) - min(frame_b),
        "frame_a_mean": sum(frame_a) / len(frame_a),
        "frame_b_mean": sum(frame_b) / len(frame_b),
    }
    Path(sys.argv[1]).write_text(json.dumps(result, indent=2), encoding="utf-8")
    robot.simulationQuit(0)
except Exception as exc:
    Path(sys.argv[1]).write_text(json.dumps({"error": str(exc)}), encoding="utf-8")
    robot.simulationQuit(1)
