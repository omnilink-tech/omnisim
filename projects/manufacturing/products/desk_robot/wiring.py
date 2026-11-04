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

"""Route the desk robot's wires through the assembled chassis, in 3D.

Each wire runs from one pin to another through the space the parts leave free: the
chassis (made and bought parts, bought_parts.assembled_solids) is voxelised, grown by
the wire's radius plus a clearance, and A* finds the shortest path that keeps the
insulation off every solid. A path that hugs a surface costs a little less than one in
open air, as a harness tied down along the plates would. The path is then shortened by
line-of-sight (string pulling) and its length, plus a service loop at each end, is the
length to cut.

    python wiring.py              # routes electrical.wire_list(), prints the cut lengths
"""
from __future__ import annotations

import heapq
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

import bought_parts
import design
from design import Robot

GRID_MM = 0.5
CLEAR_MM = 0.3           # insulation to any part
FANOUT_MM = 3.5          # wire-to-wire clearance is measured beyond this from each end: where
                         # wires fan out of a 2.54 mm-pitch row (or share a crimp) they touch and
                         # are bent by hand
SERVICE_LOOP_MM = 10.0   # each end: strip, solder and a little slack to rework
HUG_MM = 1.5             # within this of a surface a step costs less ...
HUG_DISCOUNT = 0.3       # ... by this fraction
EDT_WINDOW_MM = 25.0     # occupy() updates the distance field this far round a new wire


class Space:
    """The chassis as a boolean occupancy grid (True = solid), mm, robot frame."""

    def __init__(self, solids: Dict[str, np.ndarray], lo=(-50.0, -50.0, 0.0), hi=(50.0, 50.0, 70.0), pitch=GRID_MM):
        import trimesh
        self.lo, self.pitch = np.asarray(lo, float), pitch
        self.shape = tuple(int(math.ceil((h - l) / pitch)) + 1 for l, h in zip(lo, hi))
        occ = np.zeros(self.shape, bool)
        for tris in solids.values():
            m = trimesh.Trimesh(**trimesh.triangles.to_kwargs(tris))
            m.merge_vertices()
            v = m.voxelized(pitch)
            try:
                v = v.fill()
            except Exception:
                pass
            pts = v.points
            idx = np.round((pts - self.lo) / pitch).astype(int)
            ok = np.all((idx >= 0) & (idx < np.array(self.shape)), axis=1)
            occ[tuple(idx[ok].T)] = True
        occ[:, :, 0] = True                       # the desk
        self.occ = occ
        from scipy import ndimage
        self.dist = ndimage.distance_transform_edt(~occ) * pitch   # mm to the nearest solid cell

    def cell(self, p) -> Tuple[int, int, int]:
        return tuple(int(round(v)) for v in (np.asarray(p, float) - self.lo) / self.pitch)

    def point(self, c) -> np.ndarray:
        return self.lo + np.asarray(c, float) * self.pitch


_NB = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1) if (dx, dy, dz) != (0, 0, 0)]

try:
    import numba
except ImportError:      # the pure-Python search below is the reference; numba only makes it fast
    numba = None
JIT = numba is not None


if numba is not None:
    import heapq as _hq

    @numba.njit(cache=True)
    def _astar_jit(free, dist, start, goal, hug_mm, disc, max_expand):
        """_astar_py, compiled: the same search step for step. A cell is its C-order index,
        which sorts as its (i, j, k) tuple does, so ties on the queue break the same way."""
        nx, ny, nz = free.shape
        g = np.full(nx * ny * nz, 1e18)
        came = np.full(nx * ny * nz, -1, np.int64)
        done = np.zeros(nx * ny * nz, np.bool_)
        s = (start[0] * ny + start[1]) * nz + start[2]
        t = (goal[0] * ny + goal[1]) * nz + goal[2]
        g[s] = 0.0
        q = [(0.0, s)]
        n = 0
        while len(q) > 0:
            cur = _hq.heappop(q)[1]
            if cur == t:
                path = [cur]
                while came[cur] >= 0:
                    cur = came[cur]
                    path.append(cur)
                return n, path[::-1]
            if done[cur]:
                continue
            done[cur] = True
            n += 1
            if n > max_expand:
                break
            ci, cj, ck = cur // (ny * nz), (cur // nz) % ny, cur % nz
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    for dz in range(-1, 2):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        i, j, k = ci + dx, cj + dy, ck + dz
                        if not (0 <= i < nx and 0 <= j < ny and 0 <= k < nz):
                            continue
                        if not free[i, j, k]:
                            continue
                        step = math.sqrt(dx * dx + dy * dy + dz * dz)
                        if dist[i, j, k] <= hug_mm:
                            step *= 1.0 - disc
                        ng = g[cur] + step
                        nb = (i * ny + j) * nz + k
                        if ng < g[nb]:
                            g[nb] = ng
                            came[nb] = cur
                            h = math.sqrt((i - goal[0]) ** 2 + (j - goal[1]) ** 2 + (k - goal[2]) ** 2) * (1.0 - disc)
                            _hq.heappush(q, (ng + h, nb))
        return n, [np.int64(-1)][:0]

    @numba.njit(cache=True)
    def _line_free_jit(free, a, b, dist, hug_mm, short):
        """_line_free, compiled, sampling as np.linspace and np.round do."""
        n = int(np.ceil(np.abs(b - a).max() * 2)) + 1
        step = 1.0 / (n - 1) if n > 1 else 0.0
        for s in range(n):
            t = 1.0 if (s == n - 1 and n > 1) else s * step
            i = int(np.rint(a[0] + (b[0] - a[0]) * t))
            j = int(np.rint(a[1] + (b[1] - a[1]) * t))
            k = int(np.rint(a[2] + (b[2] - a[2]) * t))
            if not free[i, j, k] or (not short and dist[i, j, k] > hug_mm):
                return False
        return True


def _astar(space: Space, start, goal, free: np.ndarray, hug_mm: float, max_expand: int = 3_000_000) -> List[tuple]:
    if not JIT:
        return _astar_py(space, start, goal, free, hug_mm, max_expand)
    n, path = _astar_jit(free, space.dist, np.asarray(start, np.int64), np.asarray(goal, np.int64),
                         float(hug_mm), HUG_DISCOUNT, max_expand)
    if not path:
        raise RuntimeError(f"no route from {start} to {goal} ({n} cells expanded)")
    _, ny, nz = space.shape
    return [(int(c // (ny * nz)), int((c // nz) % ny), int(c % nz)) for c in path]


def _astar_py(space: Space, start, goal, free: np.ndarray, hug_mm: float, max_expand: int = 3_000_000) -> List[tuple]:
    g = {start: 0.0}
    came = {}
    goal_p = np.asarray(goal)
    openq = [(0.0, start)]
    done = set()
    n = 0
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        if cur in done:
            continue
        done.add(cur)
        n += 1
        if n > max_expand:
            break
        for d in _NB:
            nb = (cur[0] + d[0], cur[1] + d[1], cur[2] + d[2])
            if not (0 <= nb[0] < space.shape[0] and 0 <= nb[1] < space.shape[1] and 0 <= nb[2] < space.shape[2]):
                continue
            if not free[nb]:
                continue
            step = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
            if space.dist[nb] <= hug_mm:
                step *= 1.0 - HUG_DISCOUNT
            ng = g[cur] + step
            if ng < g.get(nb, 1e18):
                g[nb] = ng
                came[nb] = cur
                h = float(np.linalg.norm(np.asarray(nb) - goal_p)) * (1.0 - HUG_DISCOUNT)
                heapq.heappush(openq, (ng + h, nb))
    raise RuntimeError(f"no route from {start} to {goal} ({n} cells expanded)")


FREE_SPAN_MM = 4.0       # a shortcut may leave the surfaces for this long; longer, it must hug them


def _line_free(free: np.ndarray, a, b, dist: np.ndarray = None, hug_mm: float = None, pitch: float = 1.0) -> bool:
    """A straight line between two cells is free; and, unless it is short, it stays within
    `hug_mm` of a surface all the way (a wire is laid along the parts and tied, it does not
    hang across open space)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = int(np.ceil(np.abs(b - a).max() * 2)) + 1
    short = dist is None or np.linalg.norm(b - a) * pitch <= FREE_SPAN_MM
    if JIT:
        return _line_free_jit(free, a, b, free if dist is None else dist, float(hug_mm or 0.0), short)
    for t in np.linspace(0, 1, n):
        c = tuple(np.round(a + (b - a) * t).astype(int))
        if not free[c] or (not short and dist[c] > hug_mm):
            return False
    return True


def _pull(free: np.ndarray, path: List[tuple], dist: np.ndarray = None, hug_mm: float = None, pitch: float = 1.0) -> List[tuple]:
    """String-pull: drop every corner a straight line can skip (see _line_free)."""
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_free(free, path[i], path[j], dist, hug_mm, pitch):
            j -= 1
        out.append(path[j])
        i = j
    return out


def route(space: Space, a_xyz, b_xyz, dia_mm: float, lead_a=(0, 0, 1), lead_b=(0, 0, 1), lead_mm=2.0,
          max_expand: int = 3_000_000, guard_mm: float = 0.0) -> dict:
    """One wire from pin a to pin b. Each end leaves its pin along `lead_*` (the pin's
    axis, out of the board) for `lead_mm` (one length, or a pair: a, b), as a soldered
    wire does, then routes. `guard_mm` widens the search only (a coarse grid's straight
    runs between cell centres pass up to about half a cell nearer a solid); the clearances
    reported are the wire's own."""
    r = dia_mm / 2 + CLEAR_MM + guard_mm
    free = space.dist > r
    la, lb = (lead_mm, lead_mm) if np.isscalar(lead_mm) else lead_mm
    a0, b0 = np.asarray(a_xyz, float), np.asarray(b_xyz, float)
    a1 = a0 + la * np.asarray(lead_a, float) / np.linalg.norm(lead_a)
    b1 = b0 + lb * np.asarray(lead_b, float) / np.linalg.norm(lead_b)
    ca, cb = space.cell(a1), space.cell(b1)
    for c in (ca, cb):             # the lead's end may sit in the grown margin of its own board
        sl = tuple(slice(max(0, v - 2), v + 3) for v in c)
        free[sl] = free[sl] | ~space.occ[sl]
    path = _pull(free, _astar(space, ca, cb, free, r + HUG_MM, max_expand), space.dist, r + HUG_MM, space.pitch)
    pts = [a0] + [space.point(c) for c in path] + [b0]
    pts[1], pts[-2] = a1, b1
    length = float(sum(np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(pts) - 1)))
    return {"points_mm": [list(map(float, p)) for p in pts], "routed_mm": round(length, 1),
            "cut_mm": round(length + 2 * SERVICE_LOOP_MM),
            "clear_of_parts_mm": _clearance(space, pts[1:-1], dia_mm, space.dist0),
            "clear_of_wires_mm": _clearance(space, _trim(pts[1:-1], FANOUT_MM), dia_mm, space.dist)}


def _trim(pts, mm: float):
    """The polyline without its first and last `mm` of length."""
    P = np.asarray(pts, float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 2 * mm:
        return P[len(P) // 2: len(P) // 2 + 1]
    t = np.linspace(mm, s[-1] - mm, max(2, int((s[-1] - 2 * mm) / 0.1)))
    return np.stack([np.interp(t, s, P[:, k]) for k in range(3)], axis=1)


def _clearance(space: Space, pts, dia_mm: float, dist: np.ndarray) -> float:
    """The smallest gap from the insulation to any solid, sampled every quarter grid cell
    along the routed part of the wire (between the two pin leads)."""
    from scipy.ndimage import map_coordinates
    P = np.asarray(pts, float)
    samples = [P[0]]
    for a, b in zip(P[:-1], P[1:]):
        n = max(1, int(np.linalg.norm(b - a) / (space.pitch / 4)))
        samples += [a + (b - a) * t for t in np.linspace(0, 1, n + 1)[1:]]
    idx = ((np.asarray(samples) - space.lo) / space.pitch).T
    d = map_coordinates(dist, idx, order=1, mode="nearest")
    return round(float(d.min()) - dia_mm / 2, 2)


def occupy(space: Space, pts, dia_mm: float) -> None:
    """Make a routed wire solid, so the next wire lies beside it, not through it."""
    from scipy import ndimage
    P = np.asarray(pts, float)
    rad = dia_mm / 2
    w_lo, w_hi = np.array(space.shape), np.zeros(3, int)
    for a, b in zip(P[:-1], P[1:]):
        n = max(1, int(np.linalg.norm(b - a) / (space.pitch / 2)))
        for t in np.linspace(0, 1, n + 1):
            c = (a + (b - a) * t - space.lo) / space.pitch
            k = int(math.ceil(rad / space.pitch))
            lo = np.maximum(np.floor(c - k).astype(int), 0)
            hi = np.minimum(np.ceil(c + k).astype(int) + 1, space.shape)
            g = np.stack(np.meshgrid(*[np.arange(l, h) for l, h in zip(lo, hi)], indexing="ij"), -1)
            inside = np.linalg.norm(g - c, axis=-1) * space.pitch <= rad
            sub = space.occ[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]]
            sub |= inside
            w_lo, w_hi = np.minimum(w_lo, lo), np.maximum(w_hi, hi)
    if not (w_hi > w_lo).all():
        return
    # Only cells near the new wire get closer to a solid. Within a window round it the
    # distance is min(old, the window's own transform), exact (every new solid cell is in
    # the window; an old one outside it is already in `old`); outside, a distance above
    # EDT_WINDOW_MM may stay too large, and nothing reads one that large.
    pad = int(math.ceil(EDT_WINDOW_MM / space.pitch))
    w = tuple(slice(max(0, l - pad), min(s, h + pad)) for l, h, s in zip(w_lo, w_hi, space.shape))
    space.dist[w] = np.minimum(space.dist[w], ndimage.distance_transform_edt(~space.occ[w]) * space.pitch)


def tube_triangles(points: Sequence[Sequence[float]], dia_mm: float, n: int = 10) -> np.ndarray:
    """A round wire along a polyline (mm), as triangles: a ring at every vertex, the rings
    turned with the polyline (parallel transport), the ends capped."""
    P = np.asarray(points, float)
    keep = [0] + [i for i in range(1, len(P)) if np.linalg.norm(P[i] - P[i - 1]) > 1e-6]
    P = P[keep]
    T = np.gradient(P, axis=0)
    T /= np.linalg.norm(T, axis=1, keepdims=True)
    ref = np.array([0, 0, 1.0]) if abs(T[0][2]) < 0.9 else np.array([1.0, 0, 0])
    N = np.cross(T[0], ref)
    N /= np.linalg.norm(N)
    rings = []
    for i, t in enumerate(T):
        if i:
            N = N - np.dot(N, t) * t
            N /= np.linalg.norm(N)
        B = np.cross(t, N)
        ang = np.linspace(0, 2 * math.pi, n, endpoint=False)
        rings.append(P[i] + dia_mm / 2 * (np.outer(np.cos(ang), N) + np.outer(np.sin(ang), B)))
    tris = []
    for i in range(len(rings) - 1):
        a, b = rings[i], rings[i + 1]
        for k in range(n):
            k2 = (k + 1) % n
            tris += [[a[k], b[k], b[k2]], [a[k], b[k2], a[k2]]]
    for ring, c, flip in ((rings[0], P[0], True), (rings[-1], P[-1], False)):
        for k in range(n):
            t = [c, ring[(k + 1) % n], ring[k]]
            tris.append(t if flip else t[::-1])
    return np.asarray(tris)


# Insulated outside diameter by AWG: UL 1007 / 1061 hook-up wire, PVC (Alpha Wire 1550/1551
# series datasheets; a typical figure, not one maker's tolerance).
INSULATED_OD_MM = {22: 1.60, 24: 1.40, 26: 1.25, 28: 1.10}


def space(r: Robot) -> Space:
    """The chassis grid, cached in .tmp by a hash of every solid (voxelising takes minutes)."""
    import hashlib
    solids = bought_parts.assembled_solids(r, prefer_vendor=False)
    h = hashlib.sha256()
    for k in sorted(solids):
        h.update(k.encode())
        h.update(np.ascontiguousarray(solids[k], dtype=np.float32).tobytes())
    cache = bought_parts.CACHE / f"wiring_space_{h.hexdigest()[:16]}_{GRID_MM}.npz"
    if cache.exists():
        d = np.load(cache)
        sp = Space.__new__(Space)
        sp.lo, sp.pitch, sp.occ, sp.dist = d["lo"], float(d["pitch"]), d["occ"], d["dist"]
        sp.shape = sp.occ.shape
        sp.dist0 = sp.dist.copy()
        return sp
    sp = Space(solids)
    sp.dist0 = sp.dist.copy()             # the parts alone; .dist gains each routed wire
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(cache, lo=sp.lo, pitch=sp.pitch, occ=sp.occ, dist=sp.dist)
    return sp


def _lead(r: Robot, ref: str, pin: str, connector: str = "") -> Tuple[float, float, float]:
    """Which way a wire leaves its pin: up out of a board, back off a motor's tab, and out of
    its own face of an in-line JST SM pair (the plug's wires one way, the receptacle's the other)."""
    if ref in ("M1", "M2"):
        return (0.0, -1.0, 0.0) if ref == "M1" else (0.0, 1.0, 0.0)
    if ref.startswith("J"):
        return (0.0, -1.0, 0.0) if "SMP" in connector else (0.0, 1.0, 0.0)
    return (0.0, 0.0, 1.0)


def route_all(r: Robot) -> List[dict]:
    """Every wire in electrical.wire_list(), routed; each entry gains its path and lengths."""
    import electrical
    sp = space(r)
    pos = electrical.positions(r)
    out = []
    # The battery's leads first (they share the top plate's 8 mm hole), then thick wires
    # before thin: thick ones have the fewest ways through, thin signal wires fit round them.
    order = lambda w: ("BT1" not in (w["from"][0], w["to"][0]), w["awg"], int(w["id"][1:]))
    for w in sorted(electrical.wire_list(r), key=order):
        dia = INSULATED_OD_MM[w["awg"]]
        a, b = pos[tuple(w["from"])], pos[tuple(w["to"])]
        try:
            res = route(sp, a, b, dia, _lead(r, *w["from"], w["connector_from"]), _lead(r, *w["to"], w["connector_to"]))
            occupy(sp, res["points_mm"][2:-2] if len(res["points_mm"]) > 4 else [], dia)
        except RuntimeError as e:
            res = {"error": str(e)}
        out.append({**w, "dia_mm": dia, **res})
    return sorted(out, key=lambda w: int(w["id"][1:]))


def main(argv=None) -> int:
    import argparse
    import json
    p = argparse.ArgumentParser(description="Route the desk robot's wires; print cut lengths.")
    p.add_argument("--json", help="write the routed wires here")
    a = p.parse_args(argv)
    wires = route_all(Robot())
    for w in wires:
        print(f"{w['id']:4s} {w['net']:9s} {w['awg']} AWG {w['colour']:7s} "
              + (f"routed {w['routed_mm']:6.1f} mm, cut {w['cut_mm']:4d} mm (estimate {w['length_mm']:.0f}), "
                 f"clear of parts {w['clear_of_parts_mm']:5.2f}, of wires {w['clear_of_wires_mm']:5.2f}"
                 if "error" not in w else "NO ROUTE: " + w["error"]))
    if a.json:
        Path(a.json).write_text(json.dumps(wires, indent=1), encoding="utf-8")
    write(wires)
    return 0


COLOURS = {"red": ".80 .08 .06", "black": ".05 .05 .05", "orange": ".95 .45 .05", "yellow": ".95 .85 .10",
           "green": ".10 .60 .15", "blue": ".10 .25 .85", "violet": ".50 .15 .70", "brown": ".40 .22 .08",
           "grey": ".55 .55 .55", "white": ".92 .92 .90"}
WIRE_MESHES = bought_parts.MESHES / "wires"
ROUTED = design.PRODUCT / "release" / "electrical" / "wires_routed.json"


def write(wires: List[dict]) -> None:
    """meshes/wires/<id>.stl (metres) and release/electrical/wires_routed.json: the routes
    robot.py draws and the cut list the build uses."""
    import json
    WIRE_MESHES.mkdir(parents=True, exist_ok=True)
    for w in wires:
        if "points_mm" in w:
            design.write_stl(WIRE_MESHES / f"{w['id']}.stl", tube_triangles(w["points_mm"], w["dia_mm"]) * 0.001,
                             f"desk robot wire {w['id']}")
    ROUTED.parent.mkdir(parents=True, exist_ok=True)
    ROUTED.write_text(json.dumps(wires, indent=1), encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
