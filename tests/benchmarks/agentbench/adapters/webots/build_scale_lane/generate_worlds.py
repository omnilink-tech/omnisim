#!/usr/bin/env python3
"""Generate mechanically repeated upstream-Webots BuildScale worlds."""

from __future__ import annotations

import argparse
from pathlib import Path


HERE = Path(__file__).resolve().parent
PIONEER = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/adept/pioneer3/protos/Pioneer3at.proto"


def world_text(level: int, *, null: bool) -> str:
    controller = "build_scale_idle" if null else "build_scale_drive"
    role = "null" if null else "oracle"
    y_size = 2 * level + 10
    lines = [
        "#VRML_SIM R2025a utf8", "", f'EXTERNPROTO "{PIONEER}"', "",
        f'WorldInfo {{ title "AgenticSimBench BuildScale {level} Webots {role}" basicTimeStep 32 }}',
        f"Viewpoint {{ position 0 0 {max(55, level * 2)} }}",
        "DEF FLOOR Solid {", "  translation 0 0 -0.05", '  name "floor"',
        f"  children [ DEF FLOOR_SHAPE Shape {{ geometry Box {{ size 60 {y_size} 0.1 }} }} ]",
        "  boundingObject USE FLOOR_SHAPE", "}",
        "DEF CONTACT_WITNESS Solid {",
        f"  translation 20 {level + 2} 0.1", '  name "contact witness"',
        "  children [ DEF WITNESS_SHAPE Shape { geometry Box { size 0.2 0.2 0.2 } } ]",
        "  boundingObject USE WITNESS_SHAPE",
        "  physics Physics { density -1 mass 0.1 }", "}", "",
    ]
    for i in range(level):
        y = -(level - 1) + 2 * i
        lines.append(
            f'Pioneer3at {{ translation -10 {y} 0.11 name "scale_{i:03d}" controller "{controller}" }}')
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("levels", nargs="+", type=int)
    args = parser.parse_args()
    worlds = HERE / "worlds"
    for level in args.levels:
        for null in (False, True):
            suffix = "_null" if null else ""
            path = worlds / f"build_scale_{level}{suffix}.wbt"
            path.write_text(world_text(level, null=null), encoding="utf-8")
            print(path)


if __name__ == "__main__":
    main()
