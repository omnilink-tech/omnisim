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

"""Measure what the twin printed against the part it was asked for.

Reads the beads the engine laid (`OMNISIM_PRINTED_PART_DUMP`, written by PrintedPart)
and the STL the job was sliced from, placed as the slicer placed it (cura_slice.py
centres the footprint on the bed, or on --at). Layer by layer, at the middle of each
layer, it cuts the STL and unions the beads of that layer, then reports:

- the outer surface error: how far the printed outline sits outside (+) or inside (-)
  the drawing, sampled along the printed outline (outer surfaces only; holes are not
  measured yet);
- the footprint's size against the drawing's (X and Y);
- how much of the drawing's section the layer fills (solid layers ~1, sparse infill less);
- the printed height against the drawing's.

Blobs (plastic left where the nozzle stood, or that would have spread wider than the
nozzle's face) are part of the layer they sit in: each is a disc of its width in the
outline, so a blob on an outer wall reads as the bump it is.

Brim, skirt, purge lines and support are not the part: a layer's beads count only if
their centre line's midpoint lies inside the drawing's section (grown by 0.05 mm). The
dump does not say which feature a bead was, so support is left out by where it is: the
slicer keeps it its X/Y distance (Cura's support_xy_distance, 0.7 mm) clear of the
section.

Layers are measured in parallel (--jobs, default half the CPUs); the numbers are the
same as in one process.

    python inspect_print.py cube.beads jobs/cube20.stl
    python inspect_print.py cube.beads part.stl --at 120,80 --json
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np
import shapely
import trimesh
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import printer as printer_mod  # noqa: E402

LAYER_GAP_MM = 0.02
MIN_LAYER_PATH_MM = 2.0   # less nozzle path than this in a cluster of bead tops is not a layer
QUAD_SEGS = 16      # a bead's round ends: shapely's geometry.buffer default (shapely.buffer's is 8)


def load_beads(path) -> dict:
    data = Path(path).read_bytes()
    if data[:8] != b"OMBEADS1":
        raise SystemExit(f"{path}: not a PrintedPart dump (OMBEADS1)")
    (count,) = struct.unpack_from("<i", data, 8)
    ox, oy, oz, deposited, res = struct.unpack_from("<5d", data, 12)
    beads = np.frombuffer(data, dtype="<f4", count=count * 9, offset=52).reshape(count, 9).astype(float)
    beads[:, :8] *= 1000.0                      # metres -> mm; the blob volume stays in m^3
    return {"origin_m": [ox, oy, oz], "deposited_mm3": deposited * 1e9, "resolution_mm": res * 1000.0,
            "beads": beads}


def place_drawing(stl, bed_size, at: Optional[List[float]] = None) -> trimesh.Trimesh:
    """The STL where the slicer put it: footprint centred on `at` (default the bed centre), bottom on z = 0."""
    mesh = trimesh.load(stl, force="mesh")
    lo, hi = mesh.bounds
    target = at if at is not None else [bed_size[0] / 2.0, bed_size[1] / 2.0]
    mesh.apply_translation([target[0] - (lo[0] + hi[0]) / 2.0, target[1] - (lo[1] + hi[1]) / 2.0, -lo[2]])
    return mesh


def section(mesh: trimesh.Trimesh, z: float):
    s = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if s is None:
        return None
    planar, to3d = s.to_2D()
    if not len(planar.polygons_full):
        return None
    # to_planar re-centres; map back to the bed frame with the returned transform.
    out = []
    for p in planar.polygons_full:
        ext = np.c_[np.asarray(p.exterior.coords), np.zeros(len(p.exterior.coords)), np.ones(len(p.exterior.coords))]
        ext = (to3d @ ext.T).T[:, :2]
        holes = []
        for ring in p.interiors:
            r = np.c_[np.asarray(ring.coords), np.zeros(len(ring.coords)), np.ones(len(ring.coords))]
            holes.append((to3d @ r.T).T[:, :2])
        out.append(Polygon(ext, holes))
    return unary_union(out)


def bead_shape(b) -> Polygon:
    a, e, w = b[0:2], b[3:5], max(b[6], 1e-4)
    if np.hypot(*(e - a)) < 1e-6:
        return Point(a).buffer(w / 2.0)
    return LineString([a, e]).buffer(w / 2.0, cap_style="round")


def bead_shapes(beads: np.ndarray) -> np.ndarray:
    """bead_shape for many beads at once (the same polygons, built by shapely's array API)."""
    a, e = beads[:, 0:2], beads[:, 3:5]
    r = np.maximum(beads[:, 6], 1e-4) / 2.0
    dot = np.hypot(e[:, 0] - a[:, 0], e[:, 1] - a[:, 1]) < 1e-6
    out = np.empty(len(beads), dtype=object)
    if dot.any():
        out[dot] = shapely.buffer(shapely.points(a[dot]), r[dot], quad_segs=QUAD_SEGS)
    line = ~dot
    if line.any():
        coords = np.stack([a[line], e[line]], axis=1)
        out[line] = shapely.buffer(shapely.linestrings(coords), r[line], quad_segs=QUAD_SEGS, cap_style="round")
    return out


def bead_volume_mm3(beads: np.ndarray) -> float:
    """The plastic the beads hold, as PrintedPart laid it: a flattened stadium of width w and
    height h is (w - h) h + pi h^2 / 4, a round strand pi w^2 / 4; a blob keeps its volume."""
    line = beads[:, 8] == 0.0
    b = beads[line]
    length = np.hypot(b[:, 3] - b[:, 0], b[:, 4] - b[:, 1])
    w, h = b[:, 6], b[:, 7]
    area = np.where(w > h * 1.0001, (w - h) * h + 0.25 * np.pi * h * h, 0.25 * np.pi * w * w)
    return float((length * area).sum() + beads[~line, 8].sum() * 1e9)


# A dump whose beads hold less than this share of the plastic the part counted is missing
# beads (an engine that stopped recording them: before 2026-11-03 PrintedPart kept only
# 523,776, and a 145 g PETG shell section's dump held 52 % of its plastic).
UNRECORDED_WARN = 0.01


# The drawing, set in each worker process (inspect's `jobs`).
_MESH: Optional[trimesh.Trimesh] = None


def _init_worker(mesh) -> None:
    global _MESH
    _MESH = mesh


def blob_discs(blobs: np.ndarray) -> np.ndarray:
    """Blobs as their footprint: the same rows with both ends at the blob's centre, so
    bead_shapes draws each as a disc of its width."""
    out = np.array(blobs, dtype=float, copy=True)
    out[:, 0:3] = out[:, 3:6] = 0.5 * (blobs[:, 0:3] + blobs[:, 3:6])
    return out


def _measure_layer(args):
    """One layer: (z, its line beads, its blobs, the drawing's top) -> its row and surface
    samples, or None."""
    z, mine, blobs, zmax = args
    mesh = _MESH
    h = float(np.median(mine[:, 7]))
    zc = z - h / 2.0
    if zc <= 0.0 or zc >= zmax:
        return None
    drawing = section(mesh, zc)
    if drawing is None or drawing.is_empty:
        return None
    # A bead is the part's if its centre line runs inside the drawing: a slicer puts the
    # outer wall's centre half a line inside the outline, and the brim's first loop
    # half a line outside it. Support stands clear of the drawing (the slicer's X/Y
    # distance), so its beads are left out the same way.
    inside = drawing.buffer(0.05)
    keep = shapely.contains_xy(inside, 0.5 * (mine[:, 0] + mine[:, 3]), 0.5 * (mine[:, 1] + mine[:, 4]))
    if not keep.any():
        return None
    discs = blob_discs(blobs)
    keep_blobs = shapely.contains_xy(inside, discs[:, 0], discs[:, 1]) if len(discs) else np.zeros(0, bool)
    shapes = bead_shapes(np.r_[mine[keep], discs[keep_blobs]])
    printed = shapely.union_all(shapes)
    # The outer surface: for each island of the drawing, the outline of the printed
    # piece that covers most of it (loose infill islands inside are not a surface).
    pieces = np.array(list(getattr(printed, "geoms", [printed])), dtype=object)
    rings = []
    for island in getattr(drawing, "geoms", [drawing]):
        best = pieces[int(np.argmax(shapely.area(shapely.intersection(pieces, island))))]
        rings.append(best.exterior)
    boundary = shapely.get_exterior_ring(drawing) if isinstance(drawing, Polygon) else \
        unary_union([g.exterior for g in drawing.geoms])
    layer_err = []
    for ring in rings:
        n = max(8, int(ring.length / 0.1))
        pts = shapely.line_interpolate_point(ring, np.linspace(0.0, ring.length, n, endpoint=False))
        dist = shapely.distance(boundary, pts)
        coords = shapely.get_coordinates(pts)
        outside = ~shapely.contains_xy(drawing, coords[:, 0], coords[:, 1])
        layer_err.append(np.where(outside, dist, -dist))
    layer_err = np.concatenate(layer_err)
    bx = printed.bounds
    dx = drawing.bounds
    size = [bx[2] - bx[0] - (dx[2] - dx[0]), bx[3] - bx[1] - (dx[3] - dx[1])]
    fill = printed.intersection(drawing).area / drawing.area
    row = {"z": round(float(z), 3), "height": round(h, 3), "beads": int(keep.sum()), "blobs": int(keep_blobs.sum()),
           "outer_err_mean": round(float(np.mean(layer_err)), 4),
           "size_err_xy": [round(v, 4) for v in size], "fill": round(fill, 3)}
    return row, layer_err, size, fill


def inspect(dump: dict, mesh: trimesh.Trimesh, nozzle: float = 0.4, jobs: int = 1) -> dict:
    """`jobs` > 1 measures the layers in that many processes (the same numbers, sooner)."""
    beads = dump["beads"]
    lines = beads[beads[:, 8] == 0.0]
    # Layers: bead tops within LAYER_GAP_MM of each other (the Z servo settles a layer a
    # micron or two off; a layer is never thinner than a few hundredths of a millimetre).
    order = np.argsort(lines[:, 2], kind="stable")
    tops = np.empty(len(lines))
    zs = lines[order, 2]
    cuts = np.flatnonzero(np.diff(zs) > LAYER_GAP_MM) + 1
    bounds = np.r_[0, cuts, len(zs)]
    path = np.hypot(lines[order, 3] - lines[order, 0], lines[order, 4] - lines[order, 1])
    clusters = [(s, t, round(float(np.median(zs[s:t])), 3), float(path[s:t].sum()))
                for s, t in zip(bounds[:-1], bounds[1:]) if t > s]
    # A cluster with almost no path is not a layer: it is the first bead of a layer laid
    # while Z still rises into it (the servo trails a layer change by ~20 um). Measured
    # alone it compared one bead with a whole section (the shell section: six such
    # "layers", outer error -1 mm, footprint -170 mm). Its beads join the nearest layer.
    real = [c for c in clusters if c[3] >= MIN_LAYER_PATH_MM] or clusters
    real_z = np.array([c[2] for c in real])
    for s, t, top, _ in clusters:
        if real_z.size and top not in real_z:
            top = float(real_z[np.argmin(np.abs(real_z - top))])
        tops[order[s:t]] = top
    layer_z = np.unique(tops) if len(tops) else np.zeros(0)
    # each layer's beads, in dump order (as the old per-layer mask gave them)
    by_layer = np.argsort(tops, kind="stable")
    starts = np.searchsorted(tops[by_layer], layer_z)
    ends = np.r_[starts[1:], len(by_layer)]
    zmax = float(mesh.bounds[1][2])
    # A blob belongs to the highest layer whose top it reaches: one the face held down tops
    # out at the nozzle, its own layer's top; a taller one welled up around the nozzle.
    blobs = beads[beads[:, 8] > 0.0]
    blob_layer = np.searchsorted(layer_z, blobs[:, 2] + LAYER_GAP_MM, side="right") - 1 if len(blobs)         else np.zeros(0, int)
    groups = [(float(z), lines[by_layer[s:t]], blobs[blob_layer == k], zmax)
              for k, (z, s, t) in enumerate(zip(layer_z, starts, ends))]
    if jobs > 1 and len(groups) > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=jobs, initializer=_init_worker, initargs=(mesh,)) as pool:
            results = list(pool.map(_measure_layer, groups, chunksize=max(1, len(groups) // (8 * jobs))))
    else:
        _init_worker(mesh)
        results = [_measure_layer(g) for g in groups]
    errors, sizes, fills, per_layer = [], [], [], []
    for res in results:
        if res is None:
            continue
        row, layer_err, size, fill = res
        errors.append(layer_err)
        sizes.append(size)
        fills.append(fill)
        per_layer.append(row)
    errors = np.concatenate(errors) if errors else []
    e = np.array(errors) if len(errors) else np.zeros(1)
    s = np.array(sizes) if sizes else np.zeros((1, 2))
    part_tops = [r["z"] for r in per_layer]
    recorded = bead_volume_mm3(beads) if len(beads) else 0.0
    deposited = dump["deposited_mm3"]
    return {
        "beads": int(len(beads)), "blobs": int((beads[:, 8] > 0).sum()),
        "deposited_mm3": round(deposited, 1),
        # the plastic the dump's beads hold: short of deposited_mm3 means beads were lost
        "recorded_mm3": round(recorded, 1),
        "unrecorded_fraction": round(max(0.0, 1.0 - recorded / deposited), 4) if deposited > 0 else 0.0,
        "layers_measured": len(per_layer),
        "outer_surface_error_mm": {"mean": round(float(e.mean()), 4), "p05": round(float(np.percentile(e, 5)), 4),
                                   "p95": round(float(np.percentile(e, 95)), 4),
                                   "min": round(float(e.min()), 4), "max": round(float(e.max()), 4)},
        "size_error_mm": {"x_mean": round(float(s[:, 0].mean()), 4), "y_mean": round(float(s[:, 1].mean()), 4),
                          "x_range": [round(float(s[:, 0].min()), 4), round(float(s[:, 0].max()), 4)],
                          "y_range": [round(float(s[:, 1].min()), 4), round(float(s[:, 1].max()), 4)]},
        "height_mm": {"printed": max(part_tops) if part_tops else None, "drawing": round(zmax, 3)},
        "fill_min": round(float(min(fills)), 3) if fills else None,
        "layers": per_layer,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("beads")
    ap.add_argument("stl")
    ap.add_argument("--printer", default=str(printer_mod.DEFAULT_PROFILE))
    ap.add_argument("--at", help="x,y where the slicer centred the part (default the bed centre)")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2),
                    help="processes measuring layers (default half the CPUs; 1 = in this process)")
    args = ap.parse_args(argv)
    p = printer_mod.load(args.printer)
    bed = p.profile.get("bed", {}).get("size_mm", [p.axes["x"].travel, p.axes["y"].travel])
    at = [float(v) for v in args.at.split(",")] if args.at else None
    report = inspect(load_beads(args.beads), place_drawing(args.stl, bed, at), p.extruder.nozzle_diameter, args.jobs)
    if report["unrecorded_fraction"] > UNRECORDED_WARN:
        print(f"inspect_print: WARNING the beads hold {report['recorded_mm3']} of the {report['deposited_mm3']} mm^3 "
              f"the part laid ({100 * report['unrecorded_fraction']:.1f} % missing): the dump is not the whole part",
              file=sys.stderr)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        for k, v in report.items():
            if k != "layers":
                print(f"{k:>24}: {v}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
