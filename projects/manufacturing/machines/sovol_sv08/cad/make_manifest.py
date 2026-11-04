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

"""Write the cad_pack.py manifest for Sovol's SV08 assembly, from the assembly itself.

Sovol's pre-assembled STEP names every occurrence anonymously (NAUO1, NAUO2, ...), so
the moving groups cannot be cut by name the way the LongMill's are. They are cut by
where the parts are instead, read from the assembly once:

- **toolhead**: subassembly NAUO8, the hotend assembly (it holds the lowest point of
  the toolhead, the nozzle), without its two tall flexible parts (the umbilical and
  the filament path), which would swing rigidly with the toolhead;
- **beam**: the parts of NAUO6 (the AB gantry) that lie in the X beam's band across
  the machine, so they travel in Y with it: the extrusion, rail, end blocks, idlers;
- **gantry**: the rest of NAUO6, which rides up and down on the four Z drives;
- **frame**: everything else (base, Z columns, top, bed, spool holder, screen).

Fasteners and other parts smaller than SMALL_PART_MM across are dropped (hundreds of
screws are a third of the triangles and cannot be seen at a machine's scale).

The CAD pose comes from the CAD too: the nozzle tip against the bed sheet under it.
Assumed: the 350 mm printable area is centred on the sheet (Sovol publishes no bed
origin), so machine X/Y is measured from the sheet's centre.

    python make_manifest.py <the SV08 pre-assembled .step> --glb <cache.glb>
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SMALL_PART_MM = 9.0
TOOLHEAD, GANTRY, BED = "NAUO8", "NAUO6", "NAUO10"
UMBILICAL = ("NAUO195", "NAUO148")       # in NAUO8: tall flexible parts that follow the toolhead
BEAM_BAND_Z = (0.175, 0.232)              # CAD z (front-back, m) of the X beam where the CAD has it
BED_SHEET = "NAUO42"                      # in NAUO10: the PEI sheet, the print surface
PRINTABLE_MM = 350.0
BASE = "NAUO1"                            # the base assembly (electronics enclosure)
BASE_TOP_PANEL_Y = 0.020                  # CAD y (m) under the base's top panel
BASE_PANEL_MIN_M = 0.25                   # a part this wide is a shell panel, seen from outside
WALL_CLEAR_M = 0.005                      # and a part this close to the outer walls may show
COLOUR_SUFFIX = re.compile(r".+_[0-9a-f]{6}")


def load(step: Path, glb: Path):
    import trimesh
    if not glb.exists():
        import cascadio
        cascadio.step_to_glb(str(step), str(glb), 0.3, 0.5)
    scene = trimesh.load(str(glb))
    graph = scene.graph
    parents = graph.transforms.parents
    parts = []
    for node in graph.nodes_geometry:
        transform, name = graph[node]
        mesh = scene.geometry[name].copy()
        mesh.apply_transform(transform)
        names, n = [], node
        while n != graph.base_frame:
            names.append(n)
            n = parents.get(n, graph.base_frame)
        parts.append((list(reversed(names)), mesh))
    return parts


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("step")
    ap.add_argument("--glb", required=True, help="cache of the tessellated assembly")
    ap.add_argument("--out", default=str(HERE / "manifest.json"))
    args = ap.parse_args(argv)
    parts = load(Path(args.step), Path(args.glb))

    def under(top):
        return [(p, m) for p, m in parts if len(p) > 1 and p[1] == top]

    # The nozzle: the lowest point of the toolhead, its centre where several vertices share it.
    tool = [(p, m) for p, m in under(TOOLHEAD) if p[2] not in UMBILICAL]
    v = np.vstack([m.vertices for _, m in tool])
    low = v[v[:, 1] <= v[:, 1].min() + 0.0003]
    tip = np.array([low[:, 0].mean(), v[:, 1].min(), low[:, 2].mean()])
    sheet = [m for p, m in under(BED) if p[2] == BED_SHEET]
    lo = np.min([m.bounds[0] for m in sheet], axis=0)
    hi = np.max([m.bounds[1] for m in sheet], axis=0)
    centre = (lo + hi) / 2
    # Machine X runs along CAD +x, machine Y toward the back (CAD -z), machine Z up (CAD +y).
    cad_pose = [PRINTABLE_MM / 2 + (tip[0] - centre[0]) * 1000.0,
                PRINTABLE_MM / 2 - (tip[2] - centre[2]) * 1000.0,
                (tip[1] - hi[1]) * 1000.0]

    # Part by part, not by subassembly: a Y rail and the carriage block riding on it are
    # one subassembly, and only the block moves with the beam. A part drawn in several
    # colours comes out of the tessellator as one mesh per colour, named after the part
    # plus a RANDOM suffix (NAUO239_a41dce) that changes on every run, so a part is its
    # path without that suffix, and its bounds are those of all its colours.
    by_part = {}
    for p, m in parts:
        key = tuple(p[:-1]) if COLOUR_SUFFIX.fullmatch(p[-1]) and p[-1].startswith(p[-2] + "_") else tuple(p)
        lo_hi = by_part.get(key)
        by_part[key] = (m.bounds.copy() if lo_hi is None else
                        np.array([np.minimum(lo_hi[0], m.bounds[0]), np.maximum(lo_hi[1], m.bounds[1])]))
    beam = [k for k, b in by_part.items() if len(k) > 1 and k[1] == GANTRY
            and BEAM_BAND_Z[0] <= b[0][2] and b[1][2] <= BEAM_BAND_Z[1]]
    small = [k for k, b in by_part.items() if np.linalg.norm(b[1] - b[0]) * 1000.0 < SMALL_PART_MM]
    # Inside the closed base: below its top panel and clear of its outer walls (the
    # control board, power supply and wiring; the 10 cm board alone is 97 k triangles).
    shell = np.array([np.min([b[0] for k, b in by_part.items() if k[1] == BASE], axis=0),
                      np.max([b[1] for k, b in by_part.items() if k[1] == BASE], axis=0)])
    hidden = [k for k, b in by_part.items() if k[1] == BASE and b[1][1] < BASE_TOP_PANEL_Y
              and max(b[1][0] - b[0][0], b[1][2] - b[0][2]) < BASE_PANEL_MIN_M
              and all(b[0][i] > shell[0][i] + WALL_CLEAR_M and b[1][i] < shell[1][i] - WALL_CLEAR_M for i in (0, 2))]

    def sub(top, children=None):
        inner = f" / ({'|'.join(children)})" if children else ""
        return f"^[^/]+ / {top}{inner}( / |$)"

    def leaf(path):
        # Below the root (its name carries the vendor's Chinese title, which tools
        # re-encode), and any of the part's colour meshes.
        return "^[^/]+ / " + re.escape(" / ".join(path[1:])) + "( / [^/]+)?$"

    manifest = {
        "name": "sovol_sv08",
        "source": {
            "url": "https://github.com/Sovol3d/SV08/tree/main/STEP",
            "file": "0JXHSV08-00000 Frame 3d printer pre-assembled.zip",
            "licence": "GPL-3.0 (Sovol3d/SV08 LICENSE)",
            "attribution": "Sovol SV08 CAD, Sovol3d",
        },
        "units": "mm", "up": "y", "tolerance_mm": 0.6,
        "parts": "frame.step",
        "groups": {"toolhead": [sub(TOOLHEAD)], "beam": [leaf(p) for p in beam], "gantry": [sub(GANTRY)]},
        "exclude": [sub(TOOLHEAD, list(UMBILICAL))] + [leaf(p) for p in small + hidden],
        "bodies": [{"group": "gantry", "axis": "z", "parent": "frame"},
                   {"group": "beam", "axis": "y", "parent": "gantry"},
                   {"group": "toolhead", "axis": "x", "parent": "beam"}],
        "axis_dirs": {"x": [1, 0, 0], "y": [0, 0, -1], "z": [0, 1, 0]},
        "cad_pose_s": [round(float(v), 3) for v in cad_pose],
        "tool_tip": "lowest", "tool_group": "toolhead",
        "notes": {
            "beam_parts": len(beam), "small_parts_dropped": len(small),
            "hidden_parts_dropped": len(hidden),
            "nozzle_cad_m": [round(float(c), 5) for c in tip],
            "sheet_top_cad_m": round(float(hi[1]), 5),
            "assumed": "the 350 mm printable area is centred on the PEI sheet",
        },
    }
    # The frame and gantry tessellated coarser; the toolhead and beam, which the camera sits
    # next to, keep the full detail. With the hidden base interior gone this is 0.79 M
    # triangles instead of 1.37 M, and side by side the views are the same (2026-11-02).
    manifest["coarse"] = {"tolerance_mm": 1.0, "angular_rad": 1.0, "groups": ["frame", "gantry"]}
    Path(args.out).write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8")
    print(f"wrote {args.out}: cad_pose_s {manifest['cad_pose_s']}, {len(beam)} beam parts, "
          f"{len(small)} small parts dropped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
