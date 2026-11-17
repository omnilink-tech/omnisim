"""Dump the live Viewpoint fields for the v1 B2 negative controls."""

from __future__ import annotations

import json
import sys

from omnisim import Supervisor


def main():
    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())
    out = sys.argv[1]
    if robot.step(dt) == -1:
        raise RuntimeError("simulation ended before the viewpoint probe")
    children = robot.getRoot().getField("children")
    viewpoint = None
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        if node.getTypeName() == "Viewpoint":
            viewpoint = node
            break
    if viewpoint is None:
        raise RuntimeError("world has no Viewpoint")
    payload = {
        "position": viewpoint.getField("position").getSFVec3f(),
        "orientation": viewpoint.getField("orientation").getSFRotation(),
        "fieldOfView": viewpoint.getField("fieldOfView").getSFFloat(),
        "source": "live Supervisor fields after the engine loaded the world",
    }
    with open(out, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)
        stream.write("\n")
    robot.simulationQuit(0)


if __name__ == "__main__":
    main()
