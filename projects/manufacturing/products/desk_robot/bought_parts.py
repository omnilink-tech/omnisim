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

"""The bought parts as solids: each in a local frame fixed from its vendor's own CAD.

Every part has two models in the SAME local frame:

- DRAWN: ours, built here from the vendor's dimension drawing (the numbers below cite
  it). Apache-2.0 like the rest, committed under meshes/bought/, so a fresh clone works.
- VENDOR: the manufacturer's STEP, fetched from its official URL at build time, checked
  against its SHA-256, tessellated and moved into the local frame. Pololu publishes no
  licence for its models and Seeed's is ambiguous for a STEP, so none of it is committed
  (meshes/vendor/ is git-ignored); `python bought_parts.py fetch` makes it.

robot.py draws the vendor model when it is there and the drawn one otherwise, and
`clearance()` checks every part against every other in the assembled robot with FCL.

Local frames (mm):
  gearmotor  origin on the shaft axis in the gearbox's output face, +y out along the
             shaft, x across the 12 mm, z across the 10 mm
  caster     origin at the centre of the mounting face (against the spacer), -z toward
             the ball, the 19.05 mm length (and the two holes) along y
  xiao       origin under the board's centre, on its lowest face; USB-C toward -x
  drv8833    origin under the board's centre; the 20.32 mm length along x
  battery    origin under the holder's centre; cells along x
  standoff   origin at its centre; axis z
  screw      origin under the head (the bearing face), shank toward -z
  nut        origin at its bearing face, body toward +z
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from shapely.geometry import Point, Polygon, box

import design
from design import BOUGHT, Robot

PRODUCT = design.PRODUCT
MESHES = PRODUCT / "meshes"
DRAWN = MESHES / "bought"
VENDOR = MESHES / "vendor"
CACHE = design.ROOT / ".tmp/vendor_cad"

# ---- the vendors' files ------------------------------------------------------------------
# Fetched, never committed (no licence grants redistribution: Pololu states none; Seeed's
# CC BY-SA 4.0 names documents, not models). `member` picks one file out of a zip.
SOURCES = {
    "gearmotor": {"url": "https://www.pololu.com/file/0J2042/Pololu%20Micro%20Metal%20Gearmotor%203D%20(STEP)%20Models.zip",
                  "sha256": "eeafd5682d7a2c7deed7c7c7e24719eeb36761706aca4a1c5c815c07c36cbb55",
                  "member": "No Encoder/mmgm-cb.step", "vendor": "Pololu", "licence": "none stated (all rights reserved)"},
    "caster": {"url": "https://www.pololu.com/file/0J1637/pololu-ball-caster-with-0-375in-ball.step",
               "sha256": "859916355cb72f3a018573e6979bdb4892d103816774da0d846ffe501fe42fd8",
               "vendor": "Pololu", "licence": "none stated (all rights reserved)"},
    "drv8833": {"url": "https://www.pololu.com/file/0J1616/drv8833-dual-motor-driver-carrier-step.step",
                "sha256": "90960797d0f972d87bd07ec43d6adeec8bd4145686464896cd7f93a45d6bd70a",
                "vendor": "Pololu", "licence": "none stated (all rights reserved)"},
    "xiao": {"url": "https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/seeed-studio-xiao-esp32s3-3d_model.zip",
             "sha256": "0d552ed2cc7359ea132601f4f7fe2c9d816e2dfe4f58f6463e83f84a901d3650",
             "member": "XIAO-ESP32S3 v2.step", "vendor": "Seeed Studio",
             "licence": "Seeed wiki: CC BY-SA 4.0 for documents; models not named"},
}


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


# CAD frame -> local frame, read off each tessellated STEP (bounding boxes, mm):
#  gearmotor: shaft along +z_cad, gearbox z_cad 0..9 (face at 9), bushing to 9.72, shaft tip
#             19.0; 12 mm along x_cad, 10 mm along y_cad; terminals z_cad -17.2..-15.4.
#  caster:    flange z_cad -1.52..0 (its far face, -1.52, is the mounting face), ball up to
#             8.55 (10.07 overall: Pololu's 0.397 in); 19.05 along x_cad.
#  xiao:      board in the x_cad/z_cad plane, centred (1.805, -6.11), lowest face y_cad -0.25;
#             USB-C toward +x_cad, overhanging the board edge (12.28) to 13.81.
#  drv8833:   board 0..12.7 x 0..20.32 in x_cad/y_cad, z_cad up from 0.
CAD_TO_LOCAL = {
    "gearmotor": (np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float), np.array([0.0, -9.0, 0.0])),
    "caster": (_rz(math.pi / 2) @ _rx(math.pi), None),     # translation set below
    "xiao": (np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]], float), None),
    "drv8833": (np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float), None),
}
_CAD_POINT = {"caster": np.array([0.0, 0.0, -1.52]), "xiao": np.array([1.805, -0.25, -6.11]),
              "drv8833": np.array([6.35, 10.16, 0.0])}
for _k, _p in _CAD_POINT.items():
    R_, _ = CAD_TO_LOCAL[_k]
    CAD_TO_LOCAL[_k] = (R_, -R_ @ _p)           # that CAD point lands on the local origin


# ---- fasteners ----------------------------------------------------------------------------
# ISO 4762 socket head cap screws and ISO 4032 nuts (fasteners.eu tables): head d / k, nut s / m.
SHCS = {"M1.6": {"head_d": 3.0, "head_k": 1.6, "pitch": 0.35, "d": 1.6},
        "M2": {"head_d": 3.8, "head_k": 2.0, "pitch": 0.4, "d": 2.0},
        "M3": {"head_d": 5.5, "head_k": 3.0, "pitch": 0.5, "d": 3.0}}
NUT = {"M1.6": {"s": 3.2, "m": 1.3}, "M2": {"s": 4.0, "m": 1.6}, "M3": {"s": 5.5, "m": 2.4}}
SHCS_LENGTHS = (3, 4, 5, 6, 8, 10, 12, 16, 20, 25)          # ISO 4762 preferred lengths, M1.6-M3


def screw_length(size: str, grip_mm: float, nut=True, thread_in_mm: Optional[float] = None) -> dict:
    """The shortest standard screw that clamps `grip_mm` of parts: through a nut with at
    least one full pitch standing proud (the usual rule), or `thread_in_mm` into a tapped
    hole (into a standoff: at least 1.5 d, but no deeper than the hole)."""
    s = SHCS[size]
    nut_key = (size if nut is True else nut) if nut else None
    need = grip_mm + (NUT[nut_key]["m"] + s["pitch"] if nut_key else (thread_in_mm or 1.5 * s["d"]))
    length = next(L for L in SHCS_LENGTHS if L >= need - 1e-9)
    return {"size": size, "length": length, "grip_mm": round(grip_mm, 3), "needs_mm": round(need, 3), "nut": nut_key,
            "proud_mm": round(length - grip_mm - NUT[nut_key]["m"], 3) if nut_key else None,
            "thread_in_mm": round(length - grip_mm, 3) if not nut_key else None}


def joints(r: Robot) -> List[dict]:
    """Every screwed joint: where, which way the screw points, what it clamps, the screw it needs.
    Robot frame, mm. `head_z` is the bearing face of the head; `dir` the shank's direction."""
    c = BOUGHT["caster"]
    under = r.plate_top_z - r.plate_t
    top0 = r.plate_top_z + r.standoff_len
    out = []
    sx, sy = r.standoff_xy
    standoff_tap = 8.0                       # Wurth 970250321: M3 thread depth each end (datasheet)
    for i in (-1, 1):
        for j in (-1, 1):
            out.append({"joint": "standoff_bottom", "step": 4, "at": (i * sx, j * sy), "head_z": under, "dir": 1,
                        **screw_length("M3", r.plate_t, nut=False, thread_in_mm=min(standoff_tap, 4.5))})
            out.append({"joint": "standoff_top", "step": 6, "at": (i * sx, j * sy), "head_z": top0 + r.plate_t, "dir": -1,
                        **screw_length("M3", r.plate_t, nut=False, thread_in_mm=min(standoff_tap, 4.5))})
    for i in (-1, 1):
        for j in (-1, 1):
            # From below: plate, then the clamp's tab; the nut on the tab.
            out.append({"joint": "motor_clamp", "step": 2, "at": (r.axle_x + i * r.clamp_hole_spacing / 2, j * r.clamp_y),
                        "head_z": under, "dir": 1, **screw_length("M2", r.plate_t + r.clamp_tab_t)})
    s = c["hole_spacing_mm"] / 2
    for j in (-1, 1):
        # From the ball's side: Pololu's housing is notched beside each hole, 3.0 mm wide
        # (it pulls in to |y| 3.0-4.6 within 1.5 mm of the hole line, measured on its STEP;
        # elsewhere it stands at 5.7, 0.9 mm from the hole, so no nut fits on that side and an
        # M2 cap head, 3.8 mm, does not fit the notch). An M1.6 cap head (3.0 mm) does. Flange,
        # spacer, plate; the nut on the plate's top face, behind the tray.
        grip = r.plate_t + c["spacer_mm"] + c["flange_mm"]
        out.append({"joint": "caster", "step": 1, "at": (r.caster_x, j * s), "head_z": r.plate_top_z - grip, "dir": 1,
                    **screw_length("M1.6", grip)})
    for j in (-1, 1):
        out.append({"joint": "tray", "step": 3, "at": (r.tray_x, j * design.TRAY["hole_y"]), "head_z": under, "dir": 1,
                    **screw_length("M2", r.plate_t + design.TRAY["base"])})
    return out


# ---- drawn models ---------------------------------------------------------------------------

def _prism(poly: Polygon, z0: float, z1: float) -> np.ndarray:
    return np.vstack([design._prism(g, z0, z1) for g in getattr(poly, "geoms", [poly])])


def _move(tris: np.ndarray, R=None, t=(0.0, 0.0, 0.0)) -> np.ndarray:
    out = tris if R is None else tris @ np.asarray(R).T
    return out + np.asarray(t, float)


def _cyl(d: float, z0: float, z1: float, n: int = 32) -> np.ndarray:
    return _prism(Point(0, 0).buffer(d / 2, quad_segs=max(4, n // 4)), z0, z1)


def _hexagon(s: float) -> Polygon:
    r = s / math.sqrt(3)                     # across flats s -> corner radius
    return Polygon([(r * math.cos(math.pi / 3 * k), r * math.sin(math.pi / 3 * k)) for k in range(6)])


def _sphere(d: float, centre) -> np.ndarray:
    import trimesh
    return trimesh.creation.icosphere(subdivisions=3, radius=d / 2).triangles + np.asarray(centre, float)


def drawn_models() -> Dict[str, List[Tuple[str, np.ndarray]]]:
    """name -> [(colour 'r g b', triangles mm in the local frame)]."""
    Y = _rx(-math.pi / 2)                    # prism axis z -> +y
    can = Point(0, 0).buffer(6.0, quad_segs=16).intersection(box(-6, -5, 6, 5))
    shaft = design.shaft_section(Robot())
    # Pololu micro metal gearmotor, HPCB, no encoder (drawing 0J949 p3; the STEP's extents).
    gear = [(".78 .62 .30", _move(_prism(box(-6, -5, 6, 5), -9.0, 0.0), Y)),
            (".70 .70 .72", _move(_cyl(4.0, 0.0, 0.7), Y)),
            (".70 .70 .72", _move(_prism(shaft, 0.7, 10.0), Y)),
            (".72 .72 .74", _move(_prism(can, -21.5, -9.0), Y)),
            (".08 .08 .08", _move(_prism(box(-5.92, -5, 5.92, 5), -25.2, -21.5), Y)),
            (".72 .45 .20", _move(_prism(box(-3.2, -1.0, -2.2, 1.0).union(box(2.2, -1.0, 3.2, 1.0)), -26.2, -25.2), Y))]
    # Pololu #951 (drawing 0J1636): flange 19.05 x 12.06 x 1.52, housing, 3/8 in ball.
    c = BOUGHT["caster"]
    flange = box(-6.03, -9.525, 6.03, 9.525).difference(Point(0, 6.75).buffer(1.15)).difference(Point(0, -6.75).buffer(1.15))
    caster = [(".10 .10 .11", _prism(flange, -c["flange_mm"], 0.0)),
              (".10 .10 .11", _prism(Point(0, 0).buffer(5.84, quad_segs=12).difference(box(-1.5, 3.0, 1.5, 9.0))
                                     .difference(box(-1.5, -9.0, 1.5, -3.0)), -5.8 - c["flange_mm"] + 1.52, -c["flange_mm"])),
              (".85 .85 .87", _sphere(9.525, (0, 0, -c["height_mm"] + 9.525 / 2)))]
    # Seeed XIAO ESP32-S3 (wiki dimensions; the STEP's extents): PCB, RF shield, USB-C.
    xiao = [(".03 .03 .03", _prism(box(-10.5, -8.9, 10.5, 8.9), 0.0, 1.25)),
            (".55 .55 .57", _prism(box(-5.3, -6.1, 5.3, 6.1), 1.25, 3.25)),
            (".60 .60 .62", _prism(box(-12.0, -4.5, -4.7, 4.5), 0.26, 4.46))]
    # Pololu #2130 (dimension drawing 0J1615): 0.5 x 0.8 in PCB, the DRV8833.
    drv = [(".02 .27 .04", _prism(box(-10.16, -6.35, 10.16, 6.35), 0.0, 1.57)),
           (".05 .05 .05", _prism(box(-2.0, -2.5, 2.0, 2.5), 1.57, 2.57))]
    # Keystone 2478, 4 x AA side by side, open (drawing M65p28): floor, end walls, the cells.
    b = BOUGHT["battery_holder"]
    L, W, H = b["outline_mm"]
    hold = [(".08 .08 .08", _prism(box(-L / 2, -W / 2, L / 2, W / 2), 0.0, 1.2)),
            (".08 .08 .08", _prism(box(-L / 2, -W / 2, -L / 2 + 2.5, W / 2).union(box(L / 2 - 2.5, -W / 2, L / 2, W / 2)), 1.2, H)),
            (".08 .08 .08", _prism(box(-L / 2, -W / 2, L / 2, -W / 2 + 0.8).union(box(-L / 2, W / 2 - 0.8, L / 2, W / 2)), 1.2, 6.0))]
    X = _rx(0) @ np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)      # prism axis z -> +x
    for k in range(4):
        yc = -W / 2 + 0.8 + 14.1 / 2 + k * 14.1
        hold.append((".15 .25 .70", _move(_cyl(14.1, -25.25, 25.25), X, (0, yc, 1.2 + 14.1 / 2))))
    # Wurth 970250321: hex 5.5 AF x 25, M3 female both ends.
    so = BOUGHT["standoff"]
    standoff = [(".75 .76 .78", _prism(_hexagon(5.5).difference(Point(0, 0).buffer(1.25)), -so["len_mm"] / 2, so["len_mm"] / 2))]
    # The electrical design's mounted boards (electrical.MOUNTS; Pololu's dimensions), on foam
    # tape: SW1 the #2810 MOSFET switch (0.6 x 0.6 in, slide toward +y), U1 the #2119 regulator.
    tape = (".85 .85 .80", _prism(box(-6.5, -6.5, 6.5, 6.5), 0.0, 1.0))            # 1 mm foam tape
    sw1 = [tape, (".15 .35 .15", _prism(box(-7.62, -7.62, 7.62, 7.62), 1.0, 2.57)),
           (".10 .10 .10", _prism(box(-2.0, 2.5, 2.0, 7.0), 2.57, 4.07)),           # the slide switch body
           (".85 .85 .85", _prism(box(-0.8, 5.0, 0.8, 8.5), 4.07, 5.57))]           # its actuator, over the edge
    u1 = [(".85 .85 .80", _prism(box(-5.0, -3.5, 5.0, 3.5), 0.0, 1.0)),
          (".15 .35 .15", _prism(box(-6.0, -4.5, 6.0, 4.5), 1.0, 2.57)),
          (".20 .20 .22", _prism(box(-1.5, -1.5, 1.5, 1.5), 2.57, 4.07))]          # its inductor
    # Hung on the harness (not solids the wires route round): fuse, bulk capacitor, diode, connectors.
    f1 = [(".85 .65 .10", _move(_prism(box(-6.0, -1.5, 6.0, 1.5), -9.15, 9.15), _rx(-math.pi / 2)))]
    c1 = [(".10 .20 .55", _move(_cyl(6.3, -5.5, 5.5), np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)))]
    d1 = [(".08 .08 .08", _move(_cyl(2.7, -2.6, 2.6), np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)))]
    jst = [(".95 .95 .92", _prism(box(-5.5, -3.0, 5.5, 3.0), -2.0, 2.0))]
    out = {"gearmotor": gear, "caster": caster, "xiao": xiao, "drv8833": drv, "battery": hold, "standoff": standoff,
           "sw1": sw1, "u1": u1, "fuse": f1, "cap_bulk": c1, "diode": d1, "jst_sm": jst}
    for size in SHCS:
        s = SHCS[size]
        for length in SHCS_LENGTHS:
            out[f"shcs_{size}x{length}"] = [(".12 .12 .13", _cyl(s["head_d"], 0.0, s["head_k"], 24)),
                                            (".12 .12 .13", _cyl(s["d"], -length, 0.0, 16))]
    for key, n in NUT.items():
        d = SHCS[key.split("_")[0]]["d"]
        out[f"nut_{key}"] = [(".70 .70 .72", _prism(_hexagon(n["s"]).difference(Point(0, 0).buffer(d / 2 * 0.98)), 0.0, n["m"]))]
    return out


# ---- the vendors' models ----------------------------------------------------------------------

def _sha256(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(names: Sequence[str] = tuple(SOURCES), tol_mm: float = 0.02) -> Dict[str, dict]:
    """Download each vendor STEP (once, into .tmp/vendor_cad/dl/<name>/), verify it,
    tessellate it and write meshes/vendor/<name>_<k>.stl in the local frame plus
    meshes/vendor/index.json (colours). A file whose hash does not match is refused."""
    sys.path.insert(0, str(design.MANUFACTURING / "core"))
    import cad_pack
    import trimesh

    VENDOR.mkdir(parents=True, exist_ok=True)
    (VENDOR / ".gitignore").write_text("# vendor CAD: fetched by bought_parts.py fetch, never committed\n*\n", encoding="utf-8")
    index = json.loads((VENDOR / "index.json").read_text(encoding="utf-8")) if (VENDOR / "index.json").exists() else {}
    scratch = CACHE / "scratch"
    scratch.mkdir(parents=True, exist_ok=True)
    for name in names:
        src = SOURCES[name]
        d = CACHE / "dl" / name
        d.mkdir(parents=True, exist_ok=True)
        raw = d / Path(urllib.request.url2pathname(src["url"].rsplit("/", 1)[1])).name
        if not raw.exists() or _sha256(raw) != src["sha256"]:
            with urllib.request.urlopen(src["url"], timeout=300) as resp:
                raw.write_bytes(resp.read())
        got = _sha256(raw)
        if got != src["sha256"]:
            raise RuntimeError(f"{name}: {raw.name} has SHA-256 {got}, expected {src['sha256']}; the vendor changed the "
                               "file: check it, then update SOURCES")
        step = raw
        if "member" in src:
            ex = CACHE / "extracted" / name
            ex.mkdir(parents=True, exist_ok=True)
            step = ex / Path(src["member"]).name
            with zipfile.ZipFile(raw) as z:
                step.write_bytes(z.read(src["member"]))
        R, t = CAD_TO_LOCAL[name]
        by_colour: Dict[tuple, list] = {}
        for _, m in cad_pack._load_placed(step, tol_mm, scratch, 0.001):
            v = m.vertices * 1000.0 @ R.T + t
            by_colour.setdefault(tuple(cad_pack._rgba(m)[:3]), []).append(trimesh.Trimesh(v, m.faces, process=False))
        entries = []
        for old in VENDOR.glob(f"{name}_*.stl"):
            old.unlink()
        for k, (rgb, ms) in enumerate(sorted(by_colour.items())):
            for j, chunk in enumerate(cad_pack._chunks(ms)):
                f = f"{name}_{k}_{j}.stl"
                design.write_stl(VENDOR / f, chunk.vertices[chunk.faces] * 0.001, f"{src['vendor']} {name}")
                entries.append({"file": f, "rgb": list(rgb), "faces": int(len(chunk.faces))})
        index[name] = {"source": src["url"], "sha256": src["sha256"], "licence": src["licence"], "meshes": entries}
        print(f"{name}: {sum(e['faces'] for e in entries)} faces in {len(entries)} meshes", flush=True)
    (VENDOR / "index.json").write_text(json.dumps(index, indent=1), encoding="utf-8")
    return index


def vendor_index() -> dict:
    p = VENDOR / "index.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}


def write_drawn() -> dict:
    """meshes/bought/<name>_<k>.stl (metres) and meshes/bought/index.json."""
    DRAWN.mkdir(parents=True, exist_ok=True)
    index = {}
    for name, items in drawn_models().items():
        entries = []
        for k, (rgb, tris) in enumerate(items):
            f = f"{name}_{k}.stl"
            design.write_stl(DRAWN / f, tris * 0.001, f"desk robot bought {name}")
            entries.append({"file": f, "rgb": [float(v) for v in rgb.split()], "faces": int(len(tris))})
        index[name] = {"meshes": entries}
    (DRAWN / "index.json").write_text(json.dumps(index, indent=1), encoding="utf-8")
    return index


# ---- where they go ------------------------------------------------------------------------------

@dataclass
class Placed:
    label: str          # unique, e.g. "gearmotor_left"
    model: str          # a key of drawn_models() / SOURCES
    step: int           # assembly step (robot.STEPS index)
    R: np.ndarray       # local -> robot rotation
    t: np.ndarray       # local origin in the robot frame, mm


def placements(r: Robot) -> List[Placed]:
    c = BOUGHT["caster"]
    out = []
    for side, j in (("left", 1), ("right", -1)):
        out.append(Placed(f"gearmotor_{side}", "gearmotor", 2, _rz(0.0 if j > 0 else math.pi),
                          np.array([r.axle_x, j * r.faceplate_y, r.axle_z])))
    out.append(Placed("caster", "caster", 1, np.eye(3),
                      np.array([r.caster_x, 0.0, r.plate_top_z - r.plate_t - c["spacer_mm"]])))
    floor = r.plate_top_z + design.TRAY["base"]
    (xx, _, _), (dx, _, _) = design.TRAY["pockets"]
    out.append(Placed("xiao", "xiao", 3, np.eye(3), np.array([r.tray_x + xx, 0.0, floor])))
    out.append(Placed("drv8833", "drv8833", 3, np.eye(3), np.array([r.tray_x + dx, 0.0, floor])))
    top0 = r.plate_top_z + r.standoff_len
    out.append(Placed("battery", "battery", 6, np.eye(3), np.array([0.0, 0.0, top0 + r.plate_t])))
    import electrical
    for ref, model in (("SW1", "sw1"), ("U1", "u1")):
        x, y = electrical.MOUNTS[ref]["at"]
        out.append(Placed(ref.lower(), model, 3, np.eye(3), np.array([x, y, r.plate_top_z])))
    sx, sy = r.standoff_xy
    for i in (-1, 1):
        for j in (-1, 1):
            out.append(Placed(f"standoff_{i:+d}{j:+d}", "standoff", 4, np.eye(3),
                              np.array([i * sx, j * sy, r.plate_top_z + r.standoff_len / 2])))
    flip = _rx(math.pi)
    for n, jt in enumerate(joints(r)):
        x, y = jt["at"]
        R = np.eye(3) if jt["dir"] < 0 else flip           # local shank is -z
        out.append(Placed(f"screw_{jt['joint']}_{n}", f"shcs_{jt['size']}x{jt['length']}", jt["step"], R,
                          np.array([x, y, jt["head_z"]])))
        if jt["proud_mm"] is not None:
            # The nut sits on the far face of the grip, bearing face against it.
            nz = jt["head_z"] + jt["dir"] * jt["grip_mm"]
            out.append(Placed(f"nut_{jt['joint']}_{n}", f"nut_{jt['nut']}", jt["step"],
                              np.eye(3) if jt["dir"] > 0 else flip, np.array([x, y, nz])))
    return out


def model_meshes(model: str, prefer_vendor: bool = True) -> Tuple[str, List[dict]]:
    """('vendor'|'drawn', [{file, rgb}]) for one model; files relative to meshes/."""
    v = vendor_index().get(model) if prefer_vendor else None
    if v and all((VENDOR / e["file"]).exists() for e in v["meshes"]):
        return "vendor", [dict(e, file=f"vendor/{e['file']}") for e in v["meshes"]]
    d = json.loads((DRAWN / "index.json").read_text(encoding="utf-8"))[model]
    return "drawn", [dict(e, file=f"bought/{e['file']}") for e in d["meshes"]]


def axis_angle(R: np.ndarray) -> Tuple[float, float, float, float]:
    a = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    if a < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)
    if abs(a - math.pi) < 1e-6:
        k = int(np.argmax(np.diag(R)))
        v = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0))
        v[[i for i in range(3) if i != k]] *= np.sign(R[k, [i for i in range(3) if i != k]] + 1e-12)
        v /= np.linalg.norm(v)
        return (*v, a)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(a))
    return (*v, a)


# ---- clearance: every part against every other, assembled --------------------------------------

def _load_tris(model: str, prefer_vendor: bool) -> np.ndarray:
    import trimesh
    kind, entries = model_meshes(model, prefer_vendor)
    return np.vstack([trimesh.load(str(MESHES / e["file"])).triangles for e in entries]) * 1000.0


def assembled_solids(r: Robot, prefer_vendor: bool = True) -> Dict[str, np.ndarray]:
    """label -> triangles (mm, robot frame) for the chassis: made parts and bought ones."""
    import robot as robot_mod
    import trimesh
    robot_mod.write_meshes(r)
    top0 = r.plate_top_z + r.standoff_len
    made = {"bottom_plate": (0, 0, r.plate_top_z - r.plate_t), "top_plate": (0, 0, top0), "tray": (r.tray_x, 0, r.plate_top_z)}
    out = {}
    for name, t in made.items():
        out[name] = trimesh.load(str(robot_mod.MESHES / f"{name}.stl")).triangles * 1000.0 + np.asarray(t, float)
    for side, j in (("left", 1), ("right", -1)):
        out[f"motor_clamp_{side}"] = (trimesh.load(str(robot_mod.MESHES / "motor_clamp.stl")).triangles * 1000.0
                                      + np.array([r.axle_x, j * r.clamp_y, r.plate_top_z]))
    for p in placements(r):
        out[p.label] = _load_tris(p.model, prefer_vendor) @ p.R.T + p.t
    out.update(wheel_solids(r))
    return out


def wheel_solids(r: Robot) -> Dict[str, np.ndarray]:
    """Each wheel as robot.py assembles it (rim, hub, tyre) and the hub's three M2 cap screws:
    heads on the hub's gearbox side, shanks through the hub into the rim's 1.7 mm holes."""
    import robot as robot_mod
    import trimesh
    load = lambda n: trimesh.load(str(robot_mod.MESHES / f"{n}.stl")).triangles * 1000.0
    rim, hub, tyre = load("rim"), load("hub"), load("tyre")
    screw = _load_tris(f"shcs_M2x{BOUGHT_HUB_SCREW_LEN}", False) @ _rx(math.pi).T      # shank toward +z
    out = {}
    for side, j in (("left", 1), ("right", -1)):
        c = np.array([r.axle_x, j * r.wheel_y, r.axle_z])
        # The part's +z out of the chassis; the right wheel turned half a turn about its axle, so
        # its D-bore meets the flat of a motor mounted the other way round (WHEEL_TURN).
        R = _ry(WHEEL_TURN[side]) @ _rx(-j * math.pi / 2)
        out[f"rim_{side}"] = rim @ R.T + c + [0, -j * r.rim_width / 2, 0]
        hub_at = c + [0, -j * (r.rim_width / 2 + r.hub_t), 0]
        out[f"hub_{side}"] = hub @ R.T + hub_at
        out[f"tyre_{side}"] = tyre @ _rx(math.pi / 2).T + c
        for k, a in enumerate((30, 150, 270)):
            pt = np.array([r.hub_screw_pcd / 2 * math.cos(math.radians(a)), r.hub_screw_pcd / 2 * math.sin(math.radians(a)), 0.0])
            out[f"screw_hub_{side}_{k}"] = (screw + pt) @ R.T + hub_at
    return out


WHEEL_TURN = {"left": 0.0, "right": math.pi}   # measured: at 0 the right D-bore overlaps the shaft 1.57 mm, at pi 0
BOUGHT_HUB_SCREW_LEN = 8          # design.BOUGHT["screw_m2_hub"]: M2 x 8, 4 mm of hub then 4 into the rim


# Overlaps that ARE the joint: a screw's thread in a nut or a tapped standoff (the drawn
# bores are the minor diameter, so the thread overlaps them by design).
THREADED = [("screw_standoff", "standoff"), ("screw_", "nut_"), ("screw_hub_left", "rim_left"),
            ("screw_hub_right", "rim_right")]
# Overlaps that are a designed press: (a, b, the overlap the design intends, mm).
PRESSED = [("gearmotor", "motor_clamp", "clamp_squeeze")]
# Stretch fits measured by FCL's depth against an allowance: the O-ring tyre pulled into its
# groove (the rim's mesh steps its flanks in 0.25 mm bands, so its corners poke the torus).
STRETCHED = [("rim_", "tyre_", 0.1)]
NUDGE_MM = 0.05


def clearance(r: Robot, prefer_vendor: bool = True) -> dict:
    """Every pair of chassis parts whose solids meet, sorted into: TOUCHING (a 0.05 mm
    nudge along some axis frees them: a face bearing on its seat), THREADED (a screw in
    its nut or standoff) and INTERFERENCE (no nudge frees them: they occupy the same
    space). FCL's depth is reported for interference only: on touching faces it is not
    a meaningful number."""
    import trimesh
    solids = {k: trimesh.Trimesh(**trimesh.triangles.to_kwargs(t)) for k, t in assembled_solids(r, prefer_vendor).items()}
    cm = trimesh.collision.CollisionManager()
    for label, m in solids.items():
        cm.add_object(label, m)
    _, _, data = cm.in_collision_internal(return_names=True, return_data=True)
    depth: Dict[tuple, float] = {}
    for c in data:
        pair = tuple(sorted(c.names))
        depth[pair] = max(depth.get(pair, 0.0), float(c.depth))
    rows = {"touching": [], "threaded": [], "pressed": [], "interference": []}
    for (a, b), d in sorted(depth.items()):
        if any((a.startswith(x) and b.startswith(y)) or (b.startswith(x) and a.startswith(y)) for x, y in THREADED):
            rows["threaded"].append({"a": a, "b": b})
            continue
        stretch = next((allow for x, y, allow in STRETCHED
                        if (a.startswith(x) and b.startswith(y)) or (b.startswith(x) and a.startswith(y))), None)
        if stretch is not None:
            rows["pressed"].append({"a": a, "b": b, "designed_mm": stretch, "measured_mm": round(d, 3), "ok": d <= stretch})
            continue
        press = next((getattr(r, k) for x, y, k in PRESSED
                      if (a.startswith(x) and b.startswith(y)) or (b.startswith(x) and a.startswith(y))), None)
        if press is not None:
            # FCL's depth spans the contact patch here; the press is the gearbox's height
            # over the clamp roof, measured on the solids inside the clamp's length.
            side = 1 if "left" in a + b else -1
            gm = solids[a if a.startswith("gearmotor") else b]
            v = gm.vertices[np.abs(gm.vertices[:, 1] - side * r.clamp_y) < r.clamp_depth / 2]
            roof = r.plate_top_z + r.motor["body_mm"][1] - r.clamp_squeeze
            got = float(v[:, 2].max() - roof)
            rows["pressed"].append({"a": a, "b": b, "designed_mm": press, "measured_mm": round(got, 3),
                                    "ok": abs(got - press) <= 0.02})
            continue
        one = trimesh.collision.CollisionManager()
        one.add_object(b, solids[b])
        freed = None
        for axis in range(3):
            for sgn in (1, -1):
                T = np.eye(4)
                T[axis, 3] = sgn * NUDGE_MM
                if not one.in_collision_single(solids[a], transform=T):
                    freed = "+-"[sgn < 0] + "xyz"[axis]
                    break
            if freed:
                break
        if freed:
            rows["touching"].append({"a": a, "b": b, "freed_by": f"{a} {freed} {NUDGE_MM} mm"})
        else:
            rows["interference"].append({"a": a, "b": b, "depth_mm": round(d, 3)})
    return {"solids": len(solids), "vendor": prefer_vendor and bool(vendor_index()), **rows}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="The desk robot's bought parts: drawn models, vendor CAD, clearance.")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("drawn", help="write meshes/bought/ (our models from the vendors' drawings)")
    f = sub.add_parser("fetch", help="download, verify and tessellate the vendors' STEP models (not committed)")
    f.add_argument("names", nargs="*", default=list(SOURCES))
    c = sub.add_parser("clearance", help="check every part against every other, assembled")
    c.add_argument("--drawn", action="store_true", help="use the drawn models even where vendor CAD is present")
    a = p.parse_args(argv)
    r = Robot()
    if a.cmd == "drawn":
        print(json.dumps({k: len(v["meshes"]) for k, v in write_drawn().items()}))
    elif a.cmd == "fetch":
        fetch(a.names)
    else:
        write_drawn()
        print(json.dumps(clearance(r, prefer_vendor=not a.drawn), indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
