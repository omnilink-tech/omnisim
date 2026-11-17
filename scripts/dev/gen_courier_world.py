#!/usr/bin/env python3
# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generator for the OMNITUG500 Warehouse Courier OmniLink demo.

Emits TWO co-located, never-drifting artifacts from one layout definition:

  * projects/robots/omnisim/omnitug500/worlds/omnitug500_courier.omniworld
        the simulated transportation warehouse (Newton physics) — racks,
        walls, loading docks, named pickup bays, staged packages, the
        OMNITUG500 rover + its laser-scanner sidecar running the courier bridge.

  * projects/robots/omnisim/omnitug500/worlds/omnitug500_courier_layout.json
        the machine-readable layout the `omnitug500_courier` controller loads
        at startup: the static-obstacle footprints it seeds its occupancy
        grid with (so A* routing to any station is instant and reliable),
        the named station anchors (where the rover parks), each pickup
        bay's package staging point, the dock drop points, and the deck
        slot offsets it carries packages on.

The world is a realistic AGV warehouse: a wide central N-S aisle, two E-W
cross aisles between the rack rows, and E/W perimeter aisles — a connected
aisle grid the rover routes through. Pickup bays sit at the rack ends that
face the central aisle; dropoff docks line the east wall.

Run:  python scripts/dev/gen_courier_world.py
Then: python scripts/dev/headless_runner.py \
        projects/robots/omnisim/omnitug500/worlds/omnitug500_courier.omniworld --gui --realtime
"""

from __future__ import annotations

import json
import math
import os

# ── Repo paths ────────────────────────────────────────────────────────
_THIS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(_THIS, "..", ".."))
WORLD_DIR = os.path.join(REPO, "projects", "robots", "omnisim", "omnitug500", "worlds")
WBT = os.path.join(WORLD_DIR, "omnitug500_courier.omniworld")
LAYOUT = os.path.join(WORLD_DIR, "omnitug500_courier_layout.json")

# ── Warehouse geometry (metres) ───────────────────────────────────────
# Floor 26.4 x 17.4; walls inner faces ~ x=+/-12.9, y=+/-8.4.
FLOOR_X, FLOOR_Y = 26.4, 17.4
WALL_T = 0.2
WALL_HALF_X = 13.0   # wall centreline x
WALL_HALF_Y = 8.5    # wall centreline y

# 6 selective pallet racks: 8.2 (X) x 1.4 (Y) x 2.6 (Z), centres at:
RACK_HALF_X = 4.1
RACK_HALF_Y = 0.7
RACK_X = 5.7
RACK_Y = (-4.6, 0.0, 4.6)   # three rack rows
RACK_DEFS = []   # (def, name, cx, cy) — filled below
for _row, _cy in enumerate(RACK_Y):
    RACK_DEFS.append((f"RACK_{_row}_L", f"rack_{_row}_l", -RACK_X, _cy))
    RACK_DEFS.append((f"RACK_{_row}_R", f"rack_{_row}_r", +RACK_X, _cy))

# Loading docks on the east wall.
DOCK_X = 12.88
DOCK_Y = (-4.5, 0.0, 4.5)

# Rover spawn — south perimeter aisle, on the central-aisle centreline,
# facing north (+Y) up the main thoroughfare. heading is the world
# direction the rover's FORWARD (+local-Y) points.
SPAWN = {"x": 0.0, "y": -6.85, "heading": math.pi / 2.0}

# Grid the controller seeds (a hair larger than the walls so the perimeter
# aisles are inside the grid).
GRID_BOUNDS = [-13.6, -9.1, 13.6, 9.1]
GRID_RES = 0.12
# Inflate past the rover circumscribed radius (sqrt(0.36^2+0.63^2)=0.725) with a
# margin, so the A* route keeps the rover centre clear of every rack/wall and the
# whole oriented footprint clears at any heading, even rounding a corner. 0.78 ->
# ceil(0.78/0.12)=7 grid cells = 0.84 m effective; larger over-shrinks the 3.2 m
# central aisle and blocks the bay anchors.
GRID_INFLATE = 0.78

# Deck slots (rover-local: +Y forward, +X right, Z up). Packages ride here.
DECK_SLOTS = [[0.0, 0.40, 0.50], [0.0, 0.0, 0.50], [0.0, -0.40, 0.50]]

PKG_SIZE = 0.36

# ── Station + package palette ─────────────────────────────────────────
# Pickup bays sit at the rack ends that face the central aisle. West bays
# are the L-rack east ends (x=-1.6); east bays are the R-rack west ends
# (x=+1.6). The rover parks just inside the aisle and the package stages at
# the rack end. Colours double as a spoken handle ("the red package").
RACK_END_W = -RACK_X + RACK_HALF_X   # -1.6  (L-rack east end)
RACK_END_E = +RACK_X - RACK_HALF_X   # +1.6  (R-rack west end)

# (name, side, rack_row_y, label, rgb, package def/name, package colour, colour name)
BAYS = [
    ("bay-a", "W", 4.6,  "Bay A", (0.85, 0.15, 0.15), "PKG_A", "pkg_a", (0.85, 0.13, 0.13), "red"),
    ("bay-b", "W", 0.0,  "Bay B", (0.15, 0.45, 0.85), "PKG_B", "pkg_b", (0.13, 0.40, 0.85), "blue"),
    ("bay-c", "W", -4.6, "Bay C", (0.20, 0.70, 0.30), "PKG_C", "pkg_c", (0.16, 0.68, 0.28), "green"),
    ("bay-d", "E", 4.6,  "Bay D", (0.90, 0.55, 0.10), "PKG_D", "pkg_d", (0.92, 0.56, 0.10), "orange"),
    ("bay-e", "E", 0.0,  "Bay E", (0.60, 0.25, 0.75), "PKG_E", "pkg_e", (0.62, 0.26, 0.78), "purple"),
    ("bay-f", "E", -4.6, "Bay F", (0.10, 0.65, 0.70), "PKG_F", "pkg_f", (0.10, 0.66, 0.72), "teal"),
]
# (name, dock_y, label) — dropoff docks on the east wall.
DOCKS = [
    ("dock-1", -4.5, "Dock 1"),
    ("dock-2",  0.0, "Dock 2"),
    ("dock-3",  4.5, "Dock 3"),
]

# Pickup geometry: the rover pulls ALONGSIDE the bay in the open central
# aisle facing north (narrow 0.36 m side toward the rack), and the package
# stages beside it at the rack-end face. Nose-in would clip the rack (the
# 1.26 m chassis is longer than the half-aisle); alongside keeps the rover
# in free, routable space and the pick is a deck-load teleport regardless.
BAY_ANCHOR_X = 0.55   # |x| where the rover parks in the central aisle
BAY_PKG_X = 1.30      # |x| where the package stages (clear of the rack face)
BAY_HEADING = math.pi / 2.0   # forward points north (+Y), along the aisle
DOCK_ANCHOR_X = 11.0  # rover park x in the east perimeter aisle
DOCK_DROP_X = 12.15   # where delivered packages are set down (in front of dock)


# ── Layout (single source of truth) ───────────────────────────────────
def build_layout() -> dict:
    obstacles = []
    # walls as thin rects (inner-facing footprint)
    obstacles.append({"name": "wall_n", "x0": -WALL_HALF_X, "y0": WALL_HALF_Y - WALL_T,
                      "x1": WALL_HALF_X, "y1": WALL_HALF_Y + WALL_T})
    obstacles.append({"name": "wall_s", "x0": -WALL_HALF_X, "y0": -WALL_HALF_Y - WALL_T,
                      "x1": WALL_HALF_X, "y1": -WALL_HALF_Y + WALL_T})
    obstacles.append({"name": "wall_w", "x0": -WALL_HALF_X - WALL_T, "y0": -WALL_HALF_Y,
                      "x1": -WALL_HALF_X + WALL_T, "y1": WALL_HALF_Y})
    obstacles.append({"name": "wall_e", "x0": WALL_HALF_X - WALL_T, "y0": -WALL_HALF_Y,
                      "x1": WALL_HALF_X + WALL_T, "y1": WALL_HALF_Y})
    for _def, name, cx, cy in RACK_DEFS:
        obstacles.append({"name": name,
                          "x0": cx - RACK_HALF_X, "y0": cy - RACK_HALF_Y,
                          "x1": cx + RACK_HALF_X, "y1": cy + RACK_HALF_Y})

    stations = []
    packages = []
    for name, side, ry, label, rgb, pdef, pname, pcol, cname in BAYS:
        sgn = -1.0 if side == "W" else 1.0
        anchor_x = sgn * BAY_ANCHOR_X      # park in the open central aisle
        pkg_x = sgn * BAY_PKG_X            # package beside it, at the rack face
        stations.append({
            "name": name, "kind": "pickup", "label": label,
            "anchor": [round(anchor_x, 3), ry], "heading": round(BAY_HEADING, 4),
            "package_point": [round(pkg_x, 3), ry],
            "color": [round(c, 3) for c in rgb], "color_name": cname,
        })
        packages.append({
            "name": pname, "def": pdef, "station": name,
            "spawn": [round(pkg_x, 3), ry, round(PKG_SIZE / 2.0, 3)],
            "color": [round(c, 3) for c in pcol], "color_name": cname,
        })
    for name, dy, label in DOCKS:
        stations.append({
            "name": name, "kind": "dropoff", "label": label,
            "anchor": [DOCK_ANCHOR_X, dy], "heading": 0.0,   # forward points east
            "drop_point": [DOCK_DROP_X, dy],
        })
    stations.append({
        "name": "home", "kind": "home", "label": "Charging Dock",
        "anchor": [SPAWN["x"], SPAWN["y"]], "heading": round(SPAWN["heading"], 4),
    })

    return {
        "world": "omnitug500_courier.omniworld",
        "bounds": GRID_BOUNDS, "res": GRID_RES, "inflate": GRID_INFLATE,
        "spawn": SPAWN,
        "rover": {"half_len": 0.63, "half_wid": 0.36, "deck_z": 0.30},
        "deck_slots": DECK_SLOTS,
        "obstacles": obstacles,
        "stations": stations,
        "packages": packages,
    }


# ── VRML emission ─────────────────────────────────────────────────────
def _rack_children() -> str:
    """The identical interior of every selective pallet rack (verticals,
    cross-braces, deck, layered cardboard pallets). Copied from the proven
    omnitug500_warehouse.omniworld rack so the look matches the rest of the repo."""
    blue = "PBRAppearance { baseColor 0.13 0.27 0.50 roughness 0.5 metalness 0.3 }"
    orange = "PBRAppearance { baseColor 0.85 0.40 0.10 roughness 0.5 metalness 0.0 }"
    rows = []
    rows.append('    Pose { translation 0 0 0.3 children [ Shape { appearance Cardboard { colorOverride 1 1 1 textureTransform TextureTransform { scale 8 1 } } geometry Box { size 8.2 1.4 0.6 } } ] }')
    rows.append('    Pose { translation 0 0 0.06 children [ Shape { appearance RoughPine { colorOverride 1 1 1 textureTransform TextureTransform { scale 8 1 } } geometry Box { size 8.2 1.4 0.12 } } ] }')
    for vx in (-4.1, -1.3667, 1.3667, 4.1):
        for vy in (-0.7, 0.7):
            rows.append(f'    Pose {{ translation {vx} {vy} 1.3 children [ Shape {{ appearance {blue} geometry Box {{ size 0.12 0.12 2.6 }} }} ] }}')
    for vy in (-0.7, 0.7):
        for vz in (0.95, 1.75):
            rows.append(f'    Pose {{ translation 0 {vy} {vz} children [ Shape {{ appearance {orange} geometry Box {{ size 8.15 0.1 0.12 }} }} ] }}')
    # layered cardboard pallets on the shelves
    pallets = [
        (-3.28, 1.4, 0.7, "1 1 1"), (-3.28, 2.1, 0.5, "0.9 0.87 0.82"),
        (-1.64, 1.325, 0.55, "0.86 0.82 0.76"), (0.0, 1.325, 0.55, "1 1 1"),
        (0.0, 2.1, 0.5, "0.9 0.87 0.82"), (1.64, 1.4, 0.7, "0.86 0.82 0.76"),
        (3.28, 1.325, 0.55, "1 1 1"), (3.28, 2.1, 0.5, "0.9 0.87 0.82"),
    ]
    for px, pz, ph, col in pallets:
        sx = 1.46 if ph >= 0.55 else 1.39
        sy = 1.15 if ph >= 0.55 else 1.10
        rows.append(f'    Pose {{ translation {px} 0 {pz} children [ Shape {{ appearance Cardboard {{ colorOverride {col} }} geometry Box {{ size {sx} {sy} {ph} }} }} ] }}')
    return "\n".join(rows)


def _wall(defname, name, tx, ty, sx, sy) -> str:
    sh = f"{defname}_SH"
    return (f"DEF {defname} Solid {{\n"
            f"  translation {tx} {ty} 2.0\n"
            f"  physicsBackend \"newton\"\n"
            f"  children [ DEF {sh} Shape {{ appearance Roughcast {{ colorOverride 0.9 0.89 0.86 "
            f"textureTransform TextureTransform {{ scale 12 2 }} }} geometry Box {{ size {sx} {sy} 4.0 }} }} ]\n"
            f"  name \"{name}\"\n"
            f"  boundingObject USE {sh}\n}}\n"
            f"DEF {defname}_DADO Solid {{\n"
            f"  translation {tx} {ty} 0.6\n"
            f"  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.34 0.37 0.42 roughness 0.85 metalness 0.0 }} "
            f"geometry Box {{ size {sx if sx > sy else sx + 0.02} {sy if sy > sx else sy + 0.02} 1.2 }} }} ]\n"
            f"  name \"{name}_dado\"\n}}\n")


def emit_world(layout: dict) -> str:
    out = []
    out.append("#OMNISIM R2025a utf8\n")
    out.append("# OmniTug 500 - WAREHOUSE COURIER (OmniLink natural-language pick-and-deliver demo).")
    out.append("# An operator (or the OmniLink agent) tells the rover, in plain language, which")
    out.append("# named bay to pick a package from and which dock to deliver it to. The rover")
    out.append("# A*-routes through the aisle grid (known facility map), loads the package onto its")
    out.append("# deck, drives to the dock, and sets it down. GENERATED by scripts/dev/gen_courier_world.py")
    out.append("# -- edit the generator, not this file.")
    out.append("")
    for p in ("backgrounds/protos/OmniSimSky", "lights/protos/OmniSimSun",
              "lights/protos/OmniSimSunMarker"):
        out.append(f'EXTERNPROTO "omnisim://projects/objects/{p}.proto"')
    for p in ("RoughConcrete", "Roughcast", "Cardboard", "RoughPine"):
        out.append(f'EXTERNPROTO "omnisim://projects/appearances/protos/{p}.proto"')
    out.append("")
    out.append("WorldInfo {")
    out.append("  basicTimeStep 16")
    out.append('  title "OmniTug 500 - Warehouse Courier (OmniLink)"')
    out.append('  newtonSolver "mujoco"')
    out.append("  newtonStatics TRUE")
    out.append("  newtonRobotColliders TRUE")
    out.append("  newtonSubsteps 4")
    out.append("}")
    out.append("")
    out.append("Viewpoint {")
    out.append("  orientation 0.27 0.30 0.92 1.66")
    out.append("  position 19 -19 15")
    out.append("  exposure 1.05")
    out.append("}")
    out.append("")
    out.append("OmniSimSky { }")
    out.append("DEF SUN OmniSimSun { }")
    out.append("DEF SUN_MARKER OmniSimSunMarker { }")
    out.append("")
    # Floor
    out.append("DEF FLOOR Solid {")
    out.append("  translation 0 0 -0.02")
    out.append('  physicsBackend "newton"')
    out.append('  children [ DEF FLOOR_SH Shape { appearance RoughConcrete { colorOverride 0.75 0.75 0.75 '
               'textureTransform TextureTransform { scale 14 9 } } geometry Box { size 26.4 17.4 0.04 } } ]')
    out.append('  name "floor"')
    out.append("  boundingObject USE FLOOR_SH")
    out.append("}")
    out.append("")
    # Walls
    out.append(_wall("WALL_N", "wall_n", 0, WALL_HALF_Y, 26.2, WALL_T))
    out.append(_wall("WALL_S", "wall_s", 0, -WALL_HALF_Y, 26.2, WALL_T))
    out.append(_wall("WALL_W", "wall_w", -WALL_HALF_X, 0, WALL_T, 17.0))
    out.append(_wall("WALL_E", "wall_e", WALL_HALF_X, 0, WALL_T, 17.0))
    # Racks
    rc = _rack_children()
    for _def, name, cx, cy in RACK_DEFS:
        out.append(f"DEF {_def} Solid {{")
        out.append(f"  translation {cx} {cy} 0")
        out.append('  physicsBackend "newton"')
        out.append("  children [")
        out.append(rc)
        out.append("  ]")
        out.append(f'  name "{name}"')
        out.append("  boundingObject Pose { translation 0 0 1.3 children [ Box { size 8.2 1.4 2.6 } ] }")
        out.append("}")
    out.append("")
    # Docks (visual targets on the east wall)
    for i, (name, dy, label) in enumerate(DOCKS):
        out.append(f"DEF DOCK_{i} Solid {{")
        out.append(f"  translation {DOCK_X} {dy} 1.5")
        out.append('  children [ Shape { appearance PBRAppearance { baseColor 0.50 0.52 0.56 roughness 0.6 metalness 0.2 } '
                   'geometry Box { size 0.06 2.6 3.0 } } ]')
        out.append(f'  name "{name.replace("-", "_")}"')
        out.append("}")
    out.append("")
    # Station floor pads + sign posts (visual only).
    for st in layout["stations"]:
        ax, ay = st["anchor"]
        if st["kind"] == "pickup":
            col = st["color"]
        elif st["kind"] == "dropoff":
            col = [0.50, 0.52, 0.56]
        else:
            col = [0.20, 0.80, 0.45]
        cstr = f"{col[0]} {col[1]} {col[2]}"
        estr = f"{col[0]*0.12:.3f} {col[1]*0.12:.3f} {col[2]*0.12:.3f}"
        pad_def = "PAD_" + st["name"].upper().replace("-", "_")
        out.append(f"DEF {pad_def} Solid {{")
        out.append(f"  translation {ax} {ay} 0.011")
        out.append(f'  children [ Shape {{ appearance PBRAppearance {{ baseColor {cstr} roughness 0.7 metalness 0.0 '
                   f'emissiveColor {estr} }} geometry Box {{ size 1.1 1.1 0.012 }} }} ]')
        out.append(f'  name "pad_{st["name"].replace("-", "_")}"')
        out.append("}")
    out.append("")
    # Packages — staged dynamic boxes at the pickup bays.
    for pk in layout["packages"]:
        sx, sy, sz = pk["spawn"]
        col = pk["color"]
        cstr = f"{col[0]} {col[1]} {col[2]}"
        out.append(f"DEF {pk['def']} Solid {{")
        out.append(f"  translation {sx} {sy} {sz}")
        out.append('  physicsBackend "newton"')
        out.append(f'  children [ Shape {{ appearance PBRAppearance {{ baseColor {cstr} roughness 0.55 metalness 0.0 }} '
                   f'geometry Box {{ size {PKG_SIZE} {PKG_SIZE} {PKG_SIZE} }} }} ]')
        out.append(f'  name "{pk["name"]}"')
        out.append(f"  boundingObject Box {{ size {PKG_SIZE} {PKG_SIZE} {PKG_SIZE} }}")
        out.append("  physics Physics { density -1 mass 2.0 }")
        out.append("}")
    out.append("")
    # Rover: visual URDFRobot (controller none) + scanner sidecar (the physics
    # body that runs the courier bridge and drives both to the same pose).
    yaw = SPAWN["heading"] - math.pi / 2.0   # rover rotation about Z
    out.append("DEF OMNITUG500 URDFRobot {")
    out.append('  url "../urdf/omnitug500.urdf"')
    out.append(f"  translation {SPAWN['x']} {SPAWN['y']} 0")
    out.append(f"  rotation 0 0 1 {round(yaw, 4)}")
    out.append('  name "OMNITUG500"')
    out.append('  controller "<none>"')
    out.append("}")
    out.append("")
    out.append("DEF SCANNERS Robot {")
    out.append(f"  translation {SPAWN['x']} {SPAWN['y']} 0.02")
    out.append(f"  rotation 0 0 1 {round(yaw, 4)}")
    out.append('  name "OMNITUG500_scanners"')
    out.append('  controller "omnitug500_courier"')
    out.append('  controllerArgs [ "--layout" "../worlds/omnitug500_courier_layout.json" "--port" "8765" ]')
    out.append("  supervisor TRUE")
    out.append('  physicsBackend "newton"')
    out.append("  boundingObject Pose { translation 0 0 0.15 children [ Box { size 0.70 1.24 0.27 } ] }")
    out.append("  physics Physics { density -1 mass 40 centerOfMass [ 0 0 0.10 ] }")
    out.append('  window "omnilink_chat"')
    out.append("  children [")
    out.append('    DEF SCAN_FR Lidar { translation 0.29 0.537 0.131 rotation 0 0 1 0.7854 '
               'name "scanner_front_right" horizontalResolution 512 fieldOfView 3.1 verticalFieldOfView 0.02 '
               'numberOfLayers 1 near 0.05 minRange 0.06 maxRange 6.0 type "fixed" }')
    out.append('    DEF SCAN_RL Lidar { translation -0.29 -0.536 0.131 rotation 0 0 1 3.927 '
               'name "scanner_rear_left" horizontalResolution 512 fieldOfView 3.1 verticalFieldOfView 0.02 '
               'numberOfLayers 1 near 0.05 minRange 0.06 maxRange 6.0 type "fixed" }')
    out.append("  ]")
    out.append("}")
    out.append("")
    return "\n".join(out)


def main() -> None:
    layout = build_layout()
    os.makedirs(WORLD_DIR, exist_ok=True)
    with open(LAYOUT, "w", encoding="utf-8") as f:
        json.dump(layout, f, indent=2)
    with open(WBT, "w", encoding="utf-8", newline="\n") as f:
        f.write(emit_world(layout))
    print(f"wrote {os.path.relpath(WBT, REPO)}")
    print(f"wrote {os.path.relpath(LAYOUT, REPO)}")
    print(f"  {len(layout['obstacles'])} obstacles, "
          f"{sum(1 for s in layout['stations'] if s['kind']=='pickup')} pickup bays, "
          f"{sum(1 for s in layout['stations'] if s['kind']=='dropoff')} docks, "
          f"{len(layout['packages'])} packages")


if __name__ == "__main__":
    main()
