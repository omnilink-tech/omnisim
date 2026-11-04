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

"""Measure what a laser job made, from the engine's sheet (OMNISIM_MILLING_STOCK_DUMP).

The sheet is a MillingStock height map with the laser's absorbed fluence per point
("OMZMAP2", OmMillingStock::dumpHeights). From it:

- PIECES: the material left, split where the beam cut THROUGH. Cells join along an EDGE
  (4-connected): the bottom of a V kerf is a gap one cell wide, and on a slanted cut the
  cells either side of it meet only at their corners, which is no material at all. A
  piece not touching the sheet's edge fell out.
- The PART's size against the drawing, its hole, and the kerf: the through gap (the
  bottom of the cut) and the width deeper than 0.1 mm (its top).
- The ENGRAVING: depth and char level over a rectangle, and how even they are.
- ENERGY: what the sheet absorbed against what the controller's laser emitted.

Coordinates are work coordinates: the sheet's top centre is X0 Y0 (laser/world.py
powers the focus up there and the job zeroes on it).

    python projects/manufacturing/laser/inspect_cut.py .tmp/laser-run/sheet.zmap --trace .tmp/laser-run/trace.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
from scipy import ndimage


def load(path: Path) -> dict:
    raw = Path(path).read_bytes()
    magic = raw[:8]
    if magic not in (b"OMZMAP1\0", b"OMZMAP2\0"):
        raise ValueError(f"{path}: not a MillingStock dump")
    nx, ny = struct.unpack_from("<2i", raw, 8)
    res, top, ox, oy, oz, removed = struct.unpack_from("<6d", raw, 16)
    off = 64
    deposited = char_fluence = None
    if magic == b"OMZMAP2\0":
        deposited, char_fluence = struct.unpack_from("<2d", raw, off)
        off += 16
    h = np.frombuffer(raw, dtype="<f4", count=nx * ny, offset=off).reshape(ny, nx)
    off += 4 * nx * ny
    fluence = np.frombuffer(raw, dtype="<f4", count=nx * ny, offset=off).reshape(ny, nx) if deposited is not None else None
    return {"nx": nx, "ny": ny, "res": res, "top": top, "origin": (ox, oy, oz), "removed_m3": removed,
            "deposited_j": deposited, "char_fluence": char_fluence, "h": h.astype(np.float64),
            "fluence": None if fluence is None else fluence.astype(np.float64)}


def _work(z: dict, i: float, j: float):
    """Cell indices -> work mm (the sheet's centre is 0, 0)."""
    return ((i - (z["nx"] - 1) / 2) * z["res"] * 1000.0, (j - (z["ny"] - 1) / 2) * z["res"] * 1000.0)


def pieces(z: dict) -> list:
    material = z["h"] > 1e-7
    labels, n = ndimage.label(material)    # 4-connected
    cell_mm2 = (z["res"] * 1000.0) ** 2
    out = []
    for k in range(1, n + 1):
        mask = labels == k
        js, is_ = np.nonzero(mask)
        edge = bool(js.min() == 0 or is_.min() == 0 or js.max() == z["ny"] - 1 or is_.max() == z["nx"] - 1)
        x0, y0 = _work(z, is_.min(), js.min())
        x1, y1 = _work(z, is_.max(), js.max())
        cx, cy = _work(z, is_.mean(), js.mean())
        out.append({"label": k, "area_mm2": round(float(mask.sum() * cell_mm2), 3), "fell_out": not edge,
                    "size_mm": [round(x1 - x0, 3), round(y1 - y0, 3)], "centre_mm": [round(cx, 3), round(cy, 3)],
                    "_mask": mask})
    out.sort(key=lambda p: -p["area_mm2"])
    return out


def part_report(z: dict, drawn_mm: Sequence[float], hole_d_mm: Optional[float], perimeter_mm: float) -> dict:
    ps = pieces(z)
    cell_mm2 = (z["res"] * 1000.0) ** 2
    loose = [p for p in ps if p["fell_out"]]
    rep = {"pieces": [{k: v for k, v in p.items() if k != "_mask"} for p in ps]}
    if not loose:
        rep["part"] = None
        return rep
    part = loose[0]
    filled = ndimage.binary_fill_holes(part["_mask"])
    hole_area = float((filled & ~part["_mask"]).sum() * cell_mm2)
    sheet = next((p for p in ps if not p["fell_out"]), None)
    gap = None
    if sheet is not None:
        opening = ndimage.binary_fill_holes(sheet["_mask"]) & ~sheet["_mask"]
        gap = float((opening & ~filled).sum() * cell_mm2)
    w, h = part["size_mm"]
    rep["part"] = {
        "size_mm": part["size_mm"], "drawn_mm": list(drawn_mm),
        "undersize_mm": [round(drawn_mm[0] - w, 3), round(drawn_mm[1] - h, 3)],
        "hole_d_mm": round(math.sqrt(4 * hole_area / math.pi), 3) if hole_area > 0 else None, "hole_drawn_mm": hole_d_mm,
        "kerf_bottom_mm": round(gap / perimeter_mm, 3) if gap else None,
    }
    # Top kerf: the band around the outline cut deeper than 0.1 mm, by area over perimeter.
    deep = (z["top"] - z["h"]) > 1e-4
    ring = ndimage.binary_dilation(filled, iterations=int(math.ceil(1.0 / (z["res"] * 1000.0)))) & \
        ~ndimage.binary_erosion(filled, iterations=int(math.ceil(1.0 / (z["res"] * 1000.0))))
    rep["part"]["kerf_top_mm"] = round(float((deep & ring).sum() * cell_mm2) / perimeter_mm, 3)
    return rep


def engrave_report(z: dict, rect_mm: Sequence[float]) -> dict:
    """rect_mm: x0, y0, x1, y1 in work mm; the middle 80 % is judged (the edges ramp)."""
    x0, y0, x1, y1 = rect_mm
    mx, my = 0.1 * (x1 - x0), 0.1 * (y1 - y0)
    r = z["res"] * 1000.0
    ci = lambda x: int(round(x / r + (z["nx"] - 1) / 2))   # noqa: E731
    cj = lambda y: int(round(y / r + (z["ny"] - 1) / 2))   # noqa: E731
    sl = (slice(cj(y0 + my), cj(y1 - my) + 1), slice(ci(x0 + mx), ci(x1 - mx) + 1))
    depth = (z["top"] - z["h"][sl]) * 1000.0
    out = {"depth_mm_mean": round(float(depth.mean()), 4), "depth_mm_std": round(float(depth.std()), 4)}
    if z["fluence"] is not None:
        c = 1.0 - np.exp(-z["fluence"][sl] / z["char_fluence"])
        out["char_mean"] = round(float(c.mean()), 3)
        out["char_std"] = round(float(c.std()), 3)
        # Ends of the raster lines against the middle: M4 keeps them even, M3 overburns them.
        cols = c.mean(axis=0)
        k = max(1, len(cols) // 10)
        out["char_ends_vs_middle"] = round(float(np.r_[cols[:k], cols[-k:]].mean() - cols[len(cols) // 2 - k:len(cols) // 2 + k].mean()), 3)
    return out


def emitted_j(trace: Path) -> Optional[float]:
    last = None
    for line in Path(trace).read_text(encoding="utf-8").splitlines():
        if '"laser_j"' in line:
            last = json.loads(line).get("laser_j")
    return last


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Measure a laser-cut sheet from the engine's dump.")
    p.add_argument("zmap")
    p.add_argument("--trace", help="grbl_twin trace (for the energy the laser emitted)")
    p.add_argument("--drawn", default="30,20", help="the part's drawn size x,y in mm")
    p.add_argument("--hole", type=float, default=6.0, help="the drawn hole diameter, mm")
    p.add_argument("--perimeter", type=float, default=2 * (24 + 14) + 2 * math.pi * 3,
                   help="the outline's drawn length, mm (default: the tag)")
    p.add_argument("--engrave", default="0,-2,10,2", help="engraved rectangle x0,y0,x1,y1 in work mm")
    a = p.parse_args(argv)
    z = load(Path(a.zmap))
    rep = {"grid": {"nx": z["nx"], "ny": z["ny"], "res_mm": z["res"] * 1000.0, "thickness_mm": z["top"] * 1000.0},
           "removed_mm3": round(z["removed_m3"] * 1e9, 3)}
    rep.update(part_report(z, [float(v) for v in a.drawn.split(",")], a.hole, a.perimeter))
    rep["engrave"] = engrave_report(z, [float(v) for v in a.engrave.split(",")])
    if z["deposited_j"] is not None:
        rep["energy"] = {"absorbed_j": round(z["deposited_j"], 3)}
        if a.trace:
            e = emitted_j(Path(a.trace))
            rep["energy"]["emitted_j"] = e
            if e:
                rep["energy"]["absorbed_fraction"] = round(z["deposited_j"] / e, 4)
    print(json.dumps(rep, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
