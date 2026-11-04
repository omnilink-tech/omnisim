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

"""Measure a milled part: the engine's height map against the drawing.

The engine writes its MillingStock height map when OMNISIM_MILLING_STOCK_DUMP=<file>
is set (format in OmMillingStock.cpp). This compares it with two references:

- the DRAWING: the region the job was asked to pocket;
- the REACHABLE part: the drawing opened by the cutter radius, which is what a
  perfect machine with this end mill can cut. A round tool cannot make a sharp
  inside corner, so the drawing's inside corners stay as material of the tool's
  radius on any machine. The machine is judged against this one.

A height map samples the surface at its grid points, so a wall is only located
to within one grid spacing. Errors are reported as distances from the
reachable part's boundary, measured at the grid points that disagree with it.

    python inspect_part.py star.zmap --part star_in_circle --tool-d 6.35 --depth 3 \\
        --work-origin-world 84.227,-61.818

A 3D part (`--stl`) is compared point by point with the STL's top surface, and
with the IDEAL cut: the program's exact path cut into the same grid by
`cut_program`, which is what a perfect machine would leave. The difference
between the engine's part and the ideal cut is the twin's error; the difference
between the ideal cut and the STL is the CAM's (scallops, places the tool cannot
reach, and any gouge).

    python inspect_part.py mould.zmap --stl mould.stl --program mould.nc \\
        --tools ../machines/longmill_mk2_30x30/longmill_mk2_tools.json --work-origin-world 84.227,-61.818
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path
from typing import Dict, Optional, Sequence

import numpy as np
import shapely
from shapely.geometry.base import BaseGeometry

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
from gcode import Interpreter  # noqa: E402
import toolpath  # noqa: E402
import toolpath3d  # noqa: E402
import tool_table as tools_mod  # noqa: E402

MAGIC = b"OMZMAP1\0"
# Points this close to a reference boundary are wall points, not floor or top.
WALL_BAND_MM = 0.5


def load_zmap(path: Path) -> dict:
    data = Path(path).read_bytes()
    if data[:8] != MAGIC:
        raise ValueError(f"{path}: not a MillingStock height map")
    nx, ny = struct.unpack_from("<2i", data, 8)
    res, top, ox, oy, oz, removed = struct.unpack_from("<6d", data, 16)
    heights = np.frombuffer(data, dtype="<f4", offset=64, count=nx * ny).reshape(ny, nx)
    return {"nx": nx, "ny": ny, "res_mm": res * 1000.0, "top_mm": top * 1000.0,
            "origin_mm": [ox * 1000.0, oy * 1000.0, oz * 1000.0], "removed_mm3": removed * 1e9,
            "depth_mm": top * 1000.0 - heights.astype(np.float64) * 1000.0}


def _stats(v: np.ndarray) -> Optional[dict]:
    if v.size == 0:
        return None
    def mm(x):
        return round(float(x), 4) + 0.0  # + 0.0: no "-0.0"
    return {"n": int(v.size), "mean": mm(v.mean()), "min": mm(v.min()), "max": mm(v.max()),
            "p99_abs": mm(np.percentile(np.abs(v), 99))}


def inspect(zmap: dict, drawing: BaseGeometry, depth: float, tool_d: float,
            work_origin_local: Sequence[float]) -> dict:
    """Compare the cut with `drawing` (work XY, mm) pocketed `depth` mm deep by a flat end mill."""
    r = tool_d / 2.0
    reachable = drawing.buffer(-r, quad_segs=64).buffer(r, quad_segs=64)
    res = zmap["res_mm"]
    d = zmap["depth_mm"]
    ny, nx = d.shape
    xs = np.arange(nx) * res - work_origin_local[0]
    ys = np.arange(ny) * res - work_origin_local[1]
    X, Y = np.meshgrid(xs, ys)
    shapely.prepare(reachable)
    inside = shapely.contains_xy(reachable, X, Y)
    boundary = reachable.boundary
    shapely.prepare(boundary)
    near = shapely.dwithin(boundary, shapely.points(X.ravel(), Y.ravel()), WALL_BAND_MM).reshape(X.shape)

    cut = d > depth / 2.0
    over = cut & ~inside     # material removed outside the part: the wall moved out
    under = ~cut & inside    # material left inside the part: the wall moved in
    def wall_error(mask):
        if not mask.any():
            return {"points": 0, "max_mm": 0.0}
        dist = shapely.distance(boundary, shapely.points(X[mask], Y[mask]))
        return {"points": int(mask.sum()), "max_mm": round(float(dist.max()), 4),
                "p99_mm": round(float(np.percentile(dist, 99)), 4)}

    floor = inside & ~near
    untouched = ~inside & ~near
    area = reachable.area
    return {
        "grid": {"points": f"{nx} x {ny}", "spacing_mm": round(res, 4)},
        "drawing_area_mm2": round(drawing.area, 2),
        "reachable_area_mm2": round(area, 2),
        "inside_corner_material_mm2": round(drawing.area - area, 2),
        "floor_error_mm": _stats(d[floor] - depth),  # + is too deep
        "untouched_depth_mm": _stats(d[untouched]),  # should be 0 everywhere
        "wall_out": wall_error(over),
        "wall_in": wall_error(under),
        "volume_mm3": {"engine": round(zmap["removed_mm3"], 1), "reachable": round(area * depth, 1),
                       "drawing": round(drawing.area * depth, 1)},
    }


PARTS = {"star_in_circle": toolpath.star_in_circle}


def _profile(tool: tools_mod.Tool, d: np.ndarray) -> np.ndarray:
    """Height of the cutter's bottom above its tip at radius d (inf outside the cutter)."""
    r = tool.radius_mm
    if tool.shape == "ball":
        h = r - np.sqrt(np.maximum(r * r - d * d, 0.0))
    elif tool.shape == "vbit":
        h = d / np.tan(np.radians(min(max(tool.angle_deg, 1.0), 179.0)) / 2.0)
    else:
        h = np.zeros_like(d)
    return np.where(d <= r, h, np.inf)


def cut_program(program: str, x0: float, y0: float, nx: int, ny: int, res: float,
                table: Dict[int, tools_mod.Tool], start=(0.0, 0.0, 5.0)) -> np.ndarray:
    """The part a perfect machine leaves: every move of `program` swept into a height map.

    Heights are work Z (stock top 0) at (x0 + i res, y0 + j res). Moves are stamped every
    half grid spacing, as the engine's MillingStock does. M6 loads the T tool, as the sender
    does; work Z is the tip, so the length offset does not enter.
    """
    z = np.zeros((ny, nx))
    it = Interpreter()
    it.position = list(start)
    tool = table[1]
    for raw in program.splitlines():
        send, change, _ = tools_mod.tool_change_in(raw)
        a = list(it.position)
        for b in it.execute(send):
            if b.kind != "line":
                continue
            _sweep(z, np.array(a), np.array(b.target), x0, y0, res, tool)
            a = list(b.target)
        if change:
            tool = table[it.tool]
    return z


def _sweep(z: np.ndarray, a: np.ndarray, b: np.ndarray, x0: float, y0: float, res: float,
           tool: tools_mod.Tool) -> None:
    if min(a[2], b[2]) >= 0.0:
        return
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / (0.5 * res))))
    pts = a + np.outer(np.arange(1, n + 1) / n, b - a)
    r = tool.radius_mm
    for c in range(0, n, 128):
        chunk = pts[c:c + 128]
        chunk = chunk[chunk[:, 2] < 0.0]
        if not len(chunk):
            continue
        i0 = max(0, int(np.floor((chunk[:, 0].min() - r - x0) / res)))
        i1 = min(z.shape[1] - 1, int(np.ceil((chunk[:, 0].max() + r - x0) / res)))
        j0 = max(0, int(np.floor((chunk[:, 1].min() - r - y0) / res)))
        j1 = min(z.shape[0] - 1, int(np.ceil((chunk[:, 1].max() + r - y0) / res)))
        if i0 > i1 or j0 > j1:
            continue
        gx = x0 + np.arange(i0, i1 + 1) * res
        gy = y0 + np.arange(j0, j1 + 1) * res
        d = np.hypot(gx[None, None, :] - chunk[:, 0, None, None], gy[None, :, None] - chunk[:, 1, None, None])
        bottom = (chunk[:, 2, None, None] + _profile(tool, d)).min(axis=0)
        block = z[j0:j1 + 1, i0:i1 + 1]
        np.minimum(block, bottom, out=block)


def _signed_stats(v: np.ndarray) -> dict:
    return {"mean": round(float(v.mean()), 4) + 0.0, "min": round(float(v.min()), 4) + 0.0,
            "max": round(float(v.max()), 4) + 0.0, "p01": round(float(np.percentile(v, 1)), 4) + 0.0,
            "p99": round(float(np.percentile(v, 99)), 4) + 0.0}


def inspect_3d(zmap: dict, target: np.ndarray, ideal: np.ndarray, region: np.ndarray) -> dict:
    """Engine part vs ideal cut vs STL surface, all work Z on the zmap's grid; + is material left.

    Heights differ vertically; on a steep face a tiny sideways error is a large vertical one.
    So each comparison is also given ALONG THE NORMAL of the surface it is measured from:
    the ideal cut's for the twin (a ball's fillet is steep where the STL is flat floor), the
    STL's for the part.
    """
    part = -zmap["depth_mm"]
    res = zmap["res_mm"]

    def to_normal(surface):
        gy, gx = np.gradient(surface, res)
        return 1.0 / np.sqrt(1.0 + gx * gx + gy * gy)

    twin = part - ideal
    cam = ideal - target
    off = part - target
    k_ideal, k_stl = to_normal(ideal), to_normal(target)
    return {
        "grid": {"points": f"{part.shape[1]} x {part.shape[0]}", "spacing_mm": round(res, 4),
                 "compared": int(region.sum())},
        # The twin's error: what it cut against what a perfect machine would have.
        "twin_vs_ideal_mm": _signed_stats(twin[region]),
        "twin_vs_ideal_normal_mm": _signed_stats((twin * k_ideal)[region]),
        # The program's own result: scallops and unreachable corners (+), gouges (-).
        "ideal_vs_stl_mm": _signed_stats(cam[region]),
        "part_vs_stl_mm": _signed_stats(off[region]),
        "part_vs_stl_normal_mm": _signed_stats((off * k_stl)[region]),
        "gouged_points": int(((off * k_stl) < -0.01)[region].sum()),
        "volume_mm3": {"engine": round(zmap["removed_mm3"], 1),
                       "ideal": round(float(-ideal.sum()) * zmap["res_mm"] ** 2, 1),
                       "stl": round(float(-np.minimum(target, 0).sum()) * zmap["res_mm"] ** 2, 1)},
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("zmap", type=Path)
    ap.add_argument("--part", choices=sorted(PARTS), help="a 2.5D pocket by name")
    ap.add_argument("--stl", type=Path, help="a 3D part: its STL")
    ap.add_argument("--program", type=Path, help="3D: the G-code that cut it (for the ideal cut)")
    ap.add_argument("--tools", type=Path, help="3D: the tool table the program used")
    ap.add_argument("--tool-d", type=float, help="2.5D: flat end mill diameter, mm")
    ap.add_argument("--depth", type=float, help="2.5D: pocket depth, mm")
    ap.add_argument("--work-origin-world", help="work X0 Y0 in world mm (default: the block's centre)")
    ap.add_argument("--edge-mm", type=float, default=1.0, help="3D: leave this much of the block's edge out")
    args = ap.parse_args(argv)
    z = load_zmap(args.zmap)
    if args.work_origin_world:
        wx, wy = (float(v) for v in args.work_origin_world.split(","))
        local = [wx - z["origin_mm"][0], wy - z["origin_mm"][1]]
    else:
        local = [(z["nx"] - 1) * z["res_mm"] / 2.0, (z["ny"] - 1) * z["res_mm"] / 2.0]
    if args.stl is None:
        if not (args.part and args.tool_d and args.depth):
            ap.error("a 2.5D check needs --part, --tool-d and --depth (or give --stl)")
        print(json.dumps(inspect(z, PARTS[args.part](), args.depth, args.tool_d, local), indent=2))
        return 0
    if args.program is None:
        ap.error("a 3D check needs --program (the ideal cut is the program's)")
    res = z["res_mm"]
    x0, y0 = -local[0], -local[1]
    target = toolpath3d.surface_grid(toolpath3d.load_stl(args.stl), x0, y0, z["nx"], z["ny"], res).z
    target = np.minimum(target, 0.0)           # nothing stands above the stock top
    table = tools_mod.load(args.tools) if args.tools else tools_mod.default_tools()
    ideal = cut_program(args.program.read_text(encoding="utf-8"), x0, y0, z["nx"], z["ny"], res, table)
    edge = max(1, int(round(args.edge_mm / res)))
    region = np.zeros_like(target, dtype=bool)
    region[edge:-edge, edge:-edge] = True
    print(json.dumps(inspect_3d(z, target, ideal, region), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
