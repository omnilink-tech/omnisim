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

"""The SparkRover's parts as separate meshes, for the stills that pull it apart: the exploded
view, the wiring and the assembly sequence.

Every part is the design's own solid (spark_rover/robot.py part_table: the solids integrate.py
checks, in the robot frame, the arms at zero), written as one STL per part into the showcase
scratch folder. Every wire is harness.py's routed tube (spark_rover/meshes/wires/W*.stl) in
the colour wires_routed.json gives it. The worlds built here are static scenery: no physics,
no controller but the photographer.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import showcase_lib as lib  # noqa: E402

ROOT = lib.ROOT
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"
PART_MESHES = lib.SCRATCH / "meshes" / "parts"
WIRE_MESHES = PRODUCT / "meshes" / "wires"
WIRES_JSON = PRODUCT / "release/electrical/wires_routed.json"
ASSEMBLY_LOG = PRODUCT / "release/assembly/assembly_log.json"

WIRE_RGB = {"red": (0.85, 0.12, 0.10), "black": (0.08, 0.08, 0.09), "white": (0.92, 0.92, 0.90),
            "yellow": (0.95, 0.80, 0.10), "blue": (0.12, 0.35, 0.85), "green": (0.10, 0.65, 0.25),
            "orange": (0.95, 0.50, 0.10), "brown": (0.45, 0.28, 0.12), "grey": (0.55, 0.56, 0.58),
            "gray": (0.55, 0.56, 0.58), "purple": (0.50, 0.25, 0.70), "violet": (0.50, 0.25, 0.70), "pink": (0.95, 0.55, 0.75)}


def export_parts() -> List[dict]:
    """Write one STL per part (metres, robot frame); return the index."""
    sys.path.insert(0, str(PRODUCT))
    import robot  # noqa: E402
    d = robot.design()
    index = []
    PART_MESHES.mkdir(parents=True, exist_ok=True)
    for p in robot.part_table(d):
        if p["tris"] is None:
            continue
        f = PART_MESHES / f"{p['label']}.stl"
        if not f.exists():
            robot.write_stl(f, p["tris"] * 1e-3, p["label"])
        index.append({"label": p["label"], "owner": p["owner"], "body": p["body"], "rgb": list(p["rgb"]),
                      "transparency": p["transparency"], "file": str(f), "centre_mm": list(p["centre_mm"])})
    (PART_MESHES / "index.json").write_text(json.dumps(index, indent=1), encoding="utf-8")
    return index


def wires() -> List[dict]:
    out = []
    for w in json.loads(WIRES_JSON.read_text(encoding="utf-8")):
        f = WIRE_MESHES / f"{w['id']}.stl"
        if f.exists():
            out.append({"id": w["id"], "file": str(f), "rgb": WIRE_RGB.get(str(w.get("colour", "")).lower(), (0.6, 0.6, 0.6)),
                        "colour": w.get("colour"), "kind": w.get("kind")})
    return out


def steps() -> List[dict]:
    """assemble.py's steps: [{step, title, parts: [...labels]}]."""
    log = json.loads(ASSEMBLY_LOG.read_text(encoding="utf-8"))
    return [{"step": s["step"], "title": s["title"],
             "parts": [lab for u in s.get("units", []) for lab in u.get("parts", [])]} for s in log["steps"]]


def _url(f: str) -> str:
    return os.path.relpath(f, lib.WORLDS).replace("\\", "/")


def shape(f: str, rgb: Sequence[float], transparency: float = 0.0, roughness: float = 0.45,
          metalness: float = 0.0) -> str:
    tr = f" transparency {transparency:g}" if transparency else ""
    return (f'Shape {{ appearance PBRAppearance {{ baseColor {rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} '
            f'roughness {roughness:g} metalness {metalness:g}{tr} }} geometry Mesh {{ url "{_url(f)}" }} }}')


def finish(p: dict) -> Tuple[float, float, float]:
    """(roughness, metalness, transparency) by look: glossy clear acrylic, satin metal, matte plastic."""
    if p["transparency"]:
        return 0.08, 0.0, 0.92            # cast acrylic passes ~92 % (see make_images.clear_acrylic)
    lab = p["label"]
    if any(k in lab for k in ("bracket", "spine_foot", "screw", "insert", "standoff")):
        return 0.35, 0.6, 0.0
    return 0.5, 0.0, 0.0


def group_node(parts: Iterable[dict], offset=(0.0, 0.0, 0.0), extra: str = "", def_name: Optional[str] = None,
               recolour: Optional[Sequence[float]] = None, fade: float = 0.0) -> str:
    kids = []
    for p in parts:
        r, m, t = finish(p)
        rgb = recolour or p["rgb"]
        kids.append(shape(p["file"], rgb, max(t, fade), r, m))
    d = f"DEF {def_name} " if def_name else ""
    return (f'{d}Pose {{ translation {offset[0]:g} {offset[1]:g} {offset[2]:g} children [ '
            + " ".join(kids) + f" {extra} ] }}\n")
