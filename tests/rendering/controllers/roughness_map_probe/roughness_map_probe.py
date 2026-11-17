# Copyright 2026 OmniLink. Licensed under the Apache License, Version 2.0.
"""Compare roughness-map overrides against rendered scalar references."""
import json
import sys
from pathlib import Path

from omnisim import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
camera = robot.getDevice("camera")
camera.enable(step)
surface = robot.getFromDef("SURFACE")
roughness = surface.getField("roughness")
roughness_map = surface.getField("roughnessMap")


def capture(value):
    roughness.setSFFloat(value)
    for _ in range(8):
        if robot.step(step) == -1:
            raise RuntimeError("simulation ended before the material capture")
    data = camera.getImage()
    if not data or len(data) != camera.getWidth() * camera.getHeight() * 4:
        raise RuntimeError("camera did not return a complete image")
    # Ignore alpha: this test concerns reflected light, not the image container.
    return bytes(v for i, v in enumerate(data) if i % 4 != 3)


def difference(a, b):
    values = [abs(x - y) for x, y in zip(a, b)]
    return {"mean": sum(values) / len(values), "max": max(values)}


try:
    mapped_zero = capture(0.0)
    mapped_one = capture(1.0)
    roughness_map.removeSF()
    scalar_one = capture(1.0)
    scalar_zero = capture(0.0)
    result = {
        "map_ignores_scalar": difference(mapped_zero, mapped_one),
        "map_matches_scalar_reference": difference(mapped_zero, scalar_one),
        "scalar_still_matters": difference(scalar_zero, scalar_one),
        "image_range": max(scalar_one) - min(scalar_one),
    }
    Path(sys.argv[1]).write_text(json.dumps(result, indent=2), encoding="utf-8")
    robot.simulationQuit(0)
except Exception as exc:
    Path(sys.argv[1]).write_text(json.dumps({"error": str(exc)}), encoding="utf-8")
    robot.simulationQuit(1)
