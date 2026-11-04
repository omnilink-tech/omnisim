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

"""3D CAM for a 3-axis router: rough a part from an STL, then finish it.

The part is the STL's top surface seen from +Z, which is everything a 3-axis
machine can reach from above. Work coordinates: X/Y as in the STL, Z0 = stock
top, so the part's surface is at Z <= 0.

1. The surface is rasterised onto a grid (`surface_grid`).
2. Drop-cutter (`tip_heights`): at each grid point, the lowest the tool tip can go
   without the cutter entering the part. For a cutter whose bottom rises h(d)
   above its tip at radius d, that is max over the disc of surface(p + u) - h(|u|):
   a greyscale dilation, done by scipy. To stay clear of walls that fall between
   grid points, h is evaluated half a grid diagonal closer than the point is.
3. Roughing (`rough`): a flat end mill clears Z levels, riding at the larger of the
   level and the tip height for the surface plus an allowance.
4. Finishing (`finish`): a ball nose follows the tip height along a zig-zag raster.

Each pass is a continuous path, simplified within `tolerance_mm` of the computed
heights. `job()` puts them in one program, with the tool changes between them.

5. Holes (`bore`): round holes by helical interpolation (G2 helices, chip-clearing
   pecks), cut with the roughing tool; `job(holes=...)`.

Not a production CAM: one raster direction, no rest machining, no ramping (it
plunges at the plunge feed). Steep faces along the raster get a second raster
across them (`finish(cross="steep")`).
"""
from __future__ import annotations

import math
from pathlib import Path
import struct
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage

from tool_table import Tool

Polyline = np.ndarray      # (n, 3) work mm


# ---- STL ------------------------------------------------------------------------------
def load_stl(path) -> np.ndarray:
    """Triangles of a binary or ASCII STL, (n, 3, 3) in the file's units (mm here)."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * n:
            rec = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
            return np.frombuffer(data, dtype=rec, count=n, offset=84)["v"].astype(np.float64)
    verts = [line.split()[1:4] for line in data.decode("ascii", "replace").splitlines()
             if line.strip().startswith("vertex")]
    if not verts or len(verts) % 3:
        raise ValueError(f"{path}: not an STL file")
    return np.array(verts, dtype=np.float64).reshape(-1, 3, 3)


def write_stl(path, tris: np.ndarray, name: bytes = b"OmniSim CNC part") -> None:
    tris = np.asarray(tris, dtype=np.float64)
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    nrm = np.cross(e1, e2)
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=[("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    rec["normal"], rec["v"] = nrm, tris
    Path(path).write_bytes(name.ljust(80, b" ")[:80] + struct.pack("<I", len(tris)) + rec.tobytes())


def heightfield_solid(z: np.ndarray, x0: float, y0: float, res: float, bottom: float) -> np.ndarray:
    """A closed triangle mesh of a height field z[j, i] at (x0 + i res, y0 + j res) over `bottom`."""
    ny, nx = z.shape
    xs, ys = x0 + np.arange(nx) * res, y0 + np.arange(ny) * res
    X, Y = np.meshgrid(xs, ys)
    P = np.stack([X, Y, z], axis=-1)
    a, b, c, d = P[:-1, :-1], P[:-1, 1:], P[1:, 1:], P[1:, :-1]
    top = np.concatenate([np.stack([a, b, c], -2).reshape(-1, 3, 3), np.stack([a, c, d], -2).reshape(-1, 3, 3)])
    sides = []
    for edge in (P[0, :], P[:, -1], P[-1, ::-1], P[::-1, 0]):      # counter-clockwise from +Z
        lo = edge.copy()
        lo[:, 2] = bottom
        p0, p1, q0, q1 = edge[:-1], edge[1:], lo[:-1], lo[1:]
        sides += [np.stack([p0, q0, q1], -2), np.stack([p0, q1, p1], -2)]
    corners = np.array([[xs[0], ys[0], bottom], [xs[-1], ys[0], bottom], [xs[-1], ys[-1], bottom],
                        [xs[0], ys[-1], bottom]])
    base = np.array([[corners[0], corners[2], corners[1]], [corners[0], corners[3], corners[2]]])
    return np.concatenate([top] + sides + [base])


# ---- the surface ----------------------------------------------------------------------
class Grid:
    """Heights z[j, i] at (x0 + i res, y0 + j res), work mm."""

    def __init__(self, z: np.ndarray, x0: float, y0: float, res: float):
        self.z, self.x0, self.y0, self.res = z, float(x0), float(y0), float(res)

    @property
    def xs(self) -> np.ndarray:
        return self.x0 + np.arange(self.z.shape[1]) * self.res

    @property
    def ys(self) -> np.ndarray:
        return self.y0 + np.arange(self.z.shape[0]) * self.res

    def at(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """The highest of the four grid points around (x, y): a tip height that is safe anywhere
        in between (a link moves through points off the grid)."""
        fx = (np.asarray(x, dtype=np.float64) - self.x0) / self.res
        fy = (np.asarray(y, dtype=np.float64) - self.y0) / self.res
        nx, ny = self.z.shape[1], self.z.shape[0]
        i0 = np.clip(np.floor(fx).astype(int), 0, nx - 1)
        j0 = np.clip(np.floor(fy).astype(int), 0, ny - 1)
        i1, j1 = np.minimum(i0 + 1, nx - 1), np.minimum(j0 + 1, ny - 1)
        return np.maximum(np.maximum(self.z[j0, i0], self.z[j0, i1]), np.maximum(self.z[j1, i0], self.z[j1, i1]))


def surface_grid(tris: np.ndarray, x0: float, y0: float, nx: int, ny: int, res: float,
                 empty: float = 0.0) -> Grid:
    """The highest surface over each grid point; `empty` where no triangle covers it."""
    z = np.full((ny, nx), -np.inf)
    v = tris
    lo = np.floor((v[:, :, :2].min(axis=1) - [x0, y0]) / res).astype(int)
    hi = np.ceil((v[:, :, :2].max(axis=1) - [x0, y0]) / res).astype(int)
    lo = np.maximum(lo, 0)
    hi = np.minimum(hi, [nx - 1, ny - 1])
    for t in range(len(v)):
        (i0, j0), (i1, j1) = lo[t], hi[t]
        if i0 > i1 or j0 > j1:
            continue
        a, b, c = v[t]
        det = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(det) < 1e-12:
            continue                     # a vertical face: its edges belong to its neighbours
        X, Y = np.meshgrid(x0 + np.arange(i0, i1 + 1) * res, y0 + np.arange(j0, j1 + 1) * res)
        w0 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / det
        w1 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / det
        w2 = 1.0 - w0 - w1
        inside = (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9)
        if inside.any():
            zt = w0 * a[2] + w1 * b[2] + w2 * c[2]
            block = z[j0:j1 + 1, i0:i1 + 1]
            np.maximum(block, np.where(inside, zt, -np.inf), out=block)
    z[np.isneginf(z)] = empty
    return Grid(z, x0, y0, res)


def tip_heights(surface: Grid, tool: Tool, allowance: float = 0.0) -> Grid:
    """Drop-cutter: the lowest tip height at each grid point that leaves `allowance` on the part."""
    res = surface.res
    n = int(math.ceil(tool.radius_mm / res))
    u = np.arange(-n, n + 1) * res
    U, V = np.meshgrid(u, u)
    d = np.hypot(U, V)
    footprint = d <= tool.radius_mm + 1e-9
    closer = np.clip(d - res * math.sqrt(0.5), 0.0, tool.radius_mm)   # walls between grid points
    h = np.vectorize(tool.surface_mm)(closer)
    structure = np.where(footprint, -h, 0.0)
    z = ndimage.grey_dilation(surface.z + allowance, footprint=footprint, structure=structure,
                              mode="nearest")
    return Grid(z, surface.x0, surface.y0, res)


# ---- paths ----------------------------------------------------------------------------
def staircase(points: np.ndarray, rise: float) -> np.ndarray:
    """Make a path of tip heights safe BETWEEN its points where it is steep.

    A straight move between two tip heights that differ by more than `rise` cuts the
    corner off the climb (a ball nose rolling over a wall edge gouged the wall 1.7 mm,
    measured 2026-11-01). There the tool rises first and then moves across, or moves
    across first and then descends: never below the higher of the two.
    """
    if len(points) < 2:
        return points
    dz = np.diff(points[:, 2])
    steep = np.flatnonzero(np.abs(dz) > rise)
    if steep.size == 0:
        return points
    out, last = [], 0
    for k in steep:
        out.append(points[last:k + 1])
        a, b = points[k], points[k + 1]
        out.append(np.array([[a[0], a[1], b[2]]] if b[2] > a[2] else [[b[0], b[1], a[2]]]))
        last = k + 1
    out.append(points[last:])
    return np.concatenate(out)


def simplify(points: np.ndarray, tolerance: float) -> np.ndarray:
    """Douglas-Peucker in 3D: drop points within `tolerance` of the chord between kept ones."""
    if len(points) <= 2:
        return points
    keep = np.zeros(len(points), dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        seg = points[b] - points[a]
        rel = points[a + 1:b] - points[a]
        L2 = float(seg @ seg)
        if L2 < 1e-18:
            dist = np.linalg.norm(rel, axis=1)
        else:
            t = np.clip(rel @ seg / L2, 0.0, 1.0)
            dist = np.linalg.norm(rel - np.outer(t, seg), axis=1)
        k = int(np.argmax(dist))
        if dist[k] > tolerance:
            m = a + 1 + k
            keep[m] = True
            stack += [(a, m), (m, b)]
    return points[keep]


def _link(tip: Grid, p: np.ndarray, q: np.ndarray, floor: float) -> np.ndarray:
    """A move from p to q that rides over the part (no retract): the tip height along it."""
    n = max(2, int(math.ceil(np.linalg.norm(q[:2] - p[:2]) / tip.res)) + 1)
    t = np.linspace(0.0, 1.0, n)
    x, y = p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])
    z = np.maximum(floor, tip.at(x, y))
    z[0], z[-1] = p[2], q[2]
    return staircase(np.column_stack([x, y, z]), tip.res)[1:-1]


# Two stretches of a row farther apart than this are joined by a retract and a rapid,
# not by riding the surface between them at the cutting feed.
LINK_MAX_MM = 10.0


def _raster(tip: Grid, floor: float, need: np.ndarray, stepover: float, axis: int, tolerance: float,
            margin_cells: int) -> List[np.ndarray]:
    """Zig-zag paths along `axis` (0: rows along X, 1: along Y) over the points in `need`.

    Each row cuts only the stretches that need it, each run `margin_cells` past its ends.
    Consecutive stretches closer than LINK_MAX_MM are linked over the surface; farther
    ones start a new path (the G-code retracts between paths).
    """
    z = tip.z if axis == 0 else tip.z.T
    nd = need if axis == 0 else need.T
    along = tip.xs if axis == 0 else tip.ys
    across = tip.ys if axis == 0 else tip.xs
    rows = np.flatnonzero(nd.any(axis=1))
    if rows.size == 0:
        return []
    step = max(1, int(round(stepover / tip.res)))
    picks = list(range(rows[0], rows[-1] + 1, step))
    if picks[-1] != rows[-1]:
        picks.append(rows[-1])
    paths: List[List[np.ndarray]] = []
    forward = True
    last = len(along) - 1
    for r in picks:
        cols = np.flatnonzero(nd[r])
        if cols.size == 0:
            continue
        breaks = np.flatnonzero(np.diff(cols) > 2 * margin_cells + 1)
        runs = [(max(0, a - margin_cells), min(last, b + margin_cells))
                for a, b in zip(np.r_[cols[0], cols[breaks + 1]], np.r_[cols[breaks], cols[-1]])]
        if not forward:
            runs = [(b, a) for a, b in reversed(runs)]
        for c0, c1 in runs:
            idx = np.arange(c0, c1 + 1) if c1 >= c0 else np.arange(c0, c1 - 1, -1)
            zz = np.maximum(floor, z[r, idx])
            if axis == 0:
                row = np.column_stack([along[idx], np.full(idx.size, across[r]), zz])
            else:
                row = np.column_stack([np.full(idx.size, across[r]), along[idx], zz])
            row = simplify(staircase(row, tip.res), tolerance)
            if paths and np.linalg.norm(paths[-1][-1][-1][:2] - row[0][:2]) <= LINK_MAX_MM:
                paths[-1] += [_link(tip, paths[-1][-1][-1], row[0], floor), row]
            else:
                paths.append([row])
        forward = not forward
    return [np.concatenate(p) for p in paths]


def rough(surface: Grid, tool: Tool, step_down: float, stepover: float, allowance: float = 0.5,
          tolerance: float = 0.01) -> List[np.ndarray]:
    """Z-level roughing: one path per level, each riding max(level, tip height + allowance)."""
    tip = tip_heights(surface, tool, allowance)
    bottom = float(tip.z.min())
    if bottom >= -1e-6:
        return []
    levels, level = [], 0.0
    while level > bottom + 1e-6:
        level = max(level - step_down, bottom)
        levels.append(level)
    paths, previous = [], 0.0
    # Rows run a tool radius past the material, so points between two rows get cut too.
    margin = int(math.ceil(tool.radius_mm / tip.res))
    for level in levels:
        need = np.maximum(tip.z, level) < previous - 1e-6       # material between the two levels
        paths += _raster(tip, level, need, stepover, 0, tolerance, margin)
        previous = level
    return paths


def finish(surface: Grid, tool: Tool, stepover: float, tolerance: float = 0.002,
           cross: str = "steep", steep_deg: float = 60.0) -> List[np.ndarray]:
    """Raster finishing on the tip heights (ball nose, usually), rows along X.

    Rows along X leave a coarse finish on steep faces that run along X: between two rows
    the ball climbs them in big steps: on a slope t across the rows the stepover along the
    surface is stepover / cos(t). `cross="steep"` adds rows along Y over the faces steeper
    than `steep_deg` across the rows; "all" a full second raster; "none" none.
    """
    tip = tip_heights(surface, tool)
    need = tip.z < -1e-6
    margin = int(math.ceil(tool.radius_mm / tip.res))
    paths = _raster(tip, -np.inf, need, stepover, 0, tolerance, margin)
    if cross != "none":
        second = need
        if cross == "steep":
            slope = np.abs(np.gradient(surface.z, surface.res, axis=0))
            second = need & (slope > math.tan(math.radians(steep_deg)))
        paths += _raster(tip, -np.inf, second, stepover, 1, tolerance, margin)
    return paths


# ---- G-code ---------------------------------------------------------------------------
def _f(v: float) -> str:
    return f"{v:.3f}".rstrip("0").rstrip(".") if abs(v) >= 5e-4 else "0"


def _emit(out: List[str], paths: Sequence[np.ndarray], feed: float, plunge: float, safe_z: float) -> None:
    for p in paths:
        out.append(f"G0 Z{_f(safe_z)}")
        out.append(f"G0 X{_f(p[0, 0])} Y{_f(p[0, 1])}")
        out.append(f"G1 Z{_f(p[0, 2])} F{_f(plunge)}")
        out.append(f"F{_f(feed)}")
        last = p[0]
        for q in p[1:]:
            words = [f"{a}{_f(q[k])}" for k, a in enumerate("XYZ") if abs(q[k] - last[k]) >= 5e-4]
            if words:
                out.append("G1 " + " ".join(words))
                last = q
        out.append(f"G0 Z{_f(safe_z)}")


def bore(x: float, y: float, hole_d: float, tool: Tool, bottom: float, feed: float, *, pitch: float = 0.3,
         peck: float = 5.0, safe_z: float = 2.0) -> List[str]:
    """A round hole by helical interpolation: G2 helices down `pitch` mm a turn, a full circle
    at `bottom` to square the floor. Every `peck` mm it rises out of the hole to clear the
    chips and drops back (a deep, narrow hole packs chips in, aluminium worst of all).
    """
    rh = 0.5 * (hole_d - tool.diameter_mm)
    if rh <= 1e-3:
        raise ValueError(f"a {hole_d:g} mm hole needs a cutter smaller than T{tool.number}'s {tool.diameter_mm:g} mm")
    out = [f"G0 Z{_f(safe_z)}", f"G0 X{_f(x + rh)} Y{_f(y)}", f"G1 Z0.2 F{_f(feed)}"]
    z, since = 0.2, 0.0
    while z > bottom + 1e-9:
        dz = min(pitch, z - bottom)
        z -= dz
        since += dz
        out.append(f"G2 X{_f(x + rh)} Y{_f(y)} Z{_f(z)} I{_f(-rh)} J0")
        if since >= peck - 1e-9 and z > bottom + 1e-9:
            # Back down the cut hole at a quick feed, not a rapid: never G0 below Z0.
            out += [f"G0 Z{_f(safe_z)}", f"G1 Z{_f(z + 0.2)} F{_f(5 * feed)}", f"G1 Z{_f(z)} F{_f(feed)}"]
            since = 0.0
    out += [f"G2 X{_f(x + rh)} Y{_f(y)} I{_f(-rh)} J0", f"G1 X{_f(x)} Y{_f(y)}", f"G0 Z{_f(safe_z)}"]
    return out


def job(surface: Grid, rough_tool: Tool, finish_tool: Tool, *, step_down: float = 2.5,
        rough_stepover: Optional[float] = None, allowance: float = 0.4, finish_stepover: float = 0.5,
        rough_feed: float = 1800.0, finish_feed: float = 2000.0, plunge: float = 400.0,
        finish_plunge: float = 1200.0, rpm: float = 18000.0, safe_z: float = 2.0, cross: str = "steep",
        holes: Sequence[Tuple[float, float, float, float]] = (), hole_feed: Optional[float] = None,
        title: str = "3D part") -> Tuple[str, dict]:
    """The whole part: rough with `rough_tool`, bore `holes` (x, y, diameter, bottom Z) with it,
    change, finish with `finish_tool`. Leave the holes out of `surface`: the raster passes
    would ride into them with no idea of the shank above the cutter."""
    rough_paths = rough(surface, rough_tool, step_down, rough_stepover or 0.6 * rough_tool.diameter_mm,
                        allowance)
    finish_paths = finish(surface, finish_tool, finish_stepover, cross=cross)
    # Every line, comments too, must fit GRBL's 127-byte line buffer (the sender refuses longer).
    out = [f"({title[:120]})",
           f"(rough T{rough_tool.number} {rough_tool.diameter_mm:g} mm {rough_tool.shape}, "
           f"{step_down:g} mm levels, {allowance:g} mm left)",
           f"(finish T{finish_tool.number} {finish_tool.diameter_mm:g} mm {finish_tool.shape}, "
           f"{finish_stepover:g} mm stepover)",
           "G21 G90 G17 G94"]
    # The finishing plunges re-enter a roughed surface (`allowance` left), so they can be faster.
    for tool, paths, feed, down, bores in ((rough_tool, rough_paths, rough_feed, plunge, holes),
                                           (finish_tool, finish_paths, finish_feed, finish_plunge, ())):
        if not paths and not bores:
            continue
        out += [f"T{tool.number} M6", f"M3 S{_f(rpm)}", "G4 P1"]
        _emit(out, paths, feed, down, safe_z)
        for x, y, d, bottom in bores:
            out += bore(x, y, d, tool, bottom, hole_feed or feed, safe_z=safe_z)
        out.append("M5")
    out += [f"G0 Z{_f(safe_z)}", "M2"]
    info = {"rough_paths": len(rough_paths), "finish_paths": len(finish_paths), "rough_points": int(sum(len(p) for p in rough_paths)),
            "finish_points": int(sum(len(p) for p in finish_paths)), "holes": len(holes), "lines": len(out)}
    return "\n".join(out) + "\n", info
