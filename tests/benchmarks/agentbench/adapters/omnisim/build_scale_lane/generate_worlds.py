#!/usr/bin/env python3
"""Generate the large, mechanically repeated BuildScale oracle/null worlds."""

from __future__ import annotations

import argparse
from pathlib import Path


HERE = Path(__file__).resolve().parent


def world_text(level: int, *, null: bool) -> str:
    dt = 32 if level >= 50 else 8
    y_size = 2 * level + 10
    controller = ' controller "build_scale_idle"' if null else ""
    role = "null" if null else "oracle"
    lines = [
        "#OMNISIM R2025a utf8", "",
        'EXTERNPROTO "../protos/ScaleBot.proto"', "",
        "WorldInfo {",
        f'  title "AgenticSimBench BuildScale {level} {role}"',
        f"  basicTimeStep {dt}",
        '  newtonSolver "mujoco"',
        "  newtonSubsteps 4", "  newtonRobotColliders TRUE",
        f"  newtonNjmax {max(8192, level * 256)}",
        f"  newtonNconmax {max(4096, level * 128)}", "}",
        f"Viewpoint {{ position 0 0 {max(55, level * 2)} }}", "",
        "DEF FLOOR Solid {", "  translation 0 0 -0.05", '  name "floor"',
        f"  children [ DEF FLOOR_GEOMETRY Shape {{ geometry Box {{ size 60 {y_size} 0.1 }} }} ]",
        "  boundingObject USE FLOOR_GEOMETRY", "}",
        # Keep the falsifier pair off the fleet floor: querying the main floor
        # would return every wheel contact and double a large evidence stream.
        # This tiny dedicated support contributes exactly the calibration pair.
        "DEF CONTACT_WITNESS_SUPPORT Solid {",
        f"  translation 20 {level + 2} 0.05", '  name "contact witness support"',
        "  children [ DEF CONTACT_WITNESS_SUPPORT_SHAPE Shape { geometry Box { size 2 2 0.1 } } ]",
        "  boundingObject USE CONTACT_WITNESS_SUPPORT_SHAPE",
        "  physics Physics { density -1 mass 1 }", "}",
        "DEF CONTACT_WITNESS Solid {",
        # One centimetre of initial overlap makes contact explicit; the broad
        # support keeps the dynamic witness on its private surface after settle.
        f"  translation 20 {level + 2} 0.19", '  name "contact witness"',
        "  children [ DEF CONTACT_WITNESS_SHAPE Shape { geometry Box { size 0.2 0.2 0.2 } } ]",
        "  boundingObject USE CONTACT_WITNESS_SHAPE",
        "  physics Physics { density -1 mass 0.1 }", "}", "",
    ]
    for i in range(level):
        y = -(level - 1) + 2 * i
        lines.append(
            f'ScaleBot {{ translation -10 {y} 0 name "scale_{i:03d}"{controller} }}')
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("levels", nargs="+", type=int)
    args = parser.parse_args()
    worlds = HERE / "worlds"
    worlds.mkdir(exist_ok=True)
    for level in args.levels:
        for null in (False, True):
            suffix = "_null" if null else ""
            path = worlds / f"build_scale_{level}{suffix}.wbt"
            path.write_text(world_text(level, null=null), encoding="utf-8")
            print(path)


if __name__ == "__main__":
    main()
