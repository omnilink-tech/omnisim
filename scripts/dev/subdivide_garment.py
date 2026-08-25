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

"""Loop-subdivide a garment OBJ so the cloth solver has enough mesh to WRINKLE.

WHY THIS EXISTS. A fold is geometry. No shader, normal map or smoothing group
puts a crease in a surface that has no vertices to crease with, so the ceiling
on how a simulated garment LOOKS is set by its triangle size long before it is
set by lighting. The shipped `tshirt_hi.obj` is 2394 vertices / 4674 triangles
over 0.69 m^2, i.e. a mean edge of roughly 1.8 cm. Real jersey wrinkles at a few
millimetres. This script buys those millimetres the only way they can be bought:
more mesh.

  tshirt_hi.obj   2394 v /  4674 t   ~18 mm edge   (what the demo ships today)
  tshirt_md.obj   9466 v / 18696 t    ~9 mm edge   (1 level -- solver fallback)
  tshirt_uhd.obj 37632 v / 74784 t    ~5 mm edge   (2 levels -- the hero asset)

WHY LOOP AND NOT "just decimate less". The obvious alternative is to go back to
`usd_to_cloth_obj.py` and decimate the 6436-vertex source less aggressively.
That caps out at 6436 -- below even one level here -- because the source itself
is the limit. Subdivision has no such ceiling, is deterministic, needs no
third-party library, and (unlike re-meshing) it cannot move a vertex far: every
new point is a convex combination of existing ones, so the garment cannot gain a
feature that was not already there. What it CAN do is shrink -- Loop is an
APPROXIMATING scheme, so old vertices move toward the local average and the
surface pulls in slightly. That is measured and reported below, not assumed away.

THE THREE INVARIANTS THIS FILE EXISTS TO PROTECT
------------------------------------------------

1. ⚠ ONE UV PER POSITION, `vt` INDEX == `v` INDEX. This is a load-path
   constraint, not a style. `OmCloth` welds by POSITION and keeps the FIRST uv
   it sees at each surviving vertex, so a UV seam written into this file does
   not survive the load -- the duplicate carrying the second uv is dropped and
   its triangles smear. `usd_to_cloth_obj.write_obj` says the same thing at
   length. The input map is a FOLDED (mirrored-cylindrical, ARAP-relaxed) chart
   chosen precisely so it never needs a seam; subdivision preserves that
   property for free, because a midpoint of two uvs is one uv.

2. ⚠ POSITIONS STAY WELDED. `ModelBuilder.add_cloth_mesh` keys its stretch and
   bend adjacency on integer indices, so two coincident-but-distinct vertices
   have NO spring between them: the garment tears along that line. Loop cannot
   introduce a duplicate (each edge contributes exactly one new vertex, keyed on
   the canonical vertex pair), but rounding to the written precision COULD make
   two distinct vertices print identically, and the loader would then weld them
   into a pinch point. `check_mesh` therefore tests uniqueness AT THE WRITTEN
   PRECISION, not in float64.

3. ⚠ NO ORPHAN VERTICES, NO DEGENERATE TRIANGLES. `builder.py:9154` leaves an
   orphan vertex's mass at 0.0, which silently makes it a KINEMATIC PIN -- a
   garment nailed to the air by a vertex nobody drew. And a near-zero-area
   triangle gives the VBD solver an ill-conditioned rest shape, which is how a
   cloth run "explodes for no reason". Both are counted and reported.

UVs ARE SUBDIVIDED LINEARLY, GEOMETRY IS NOT -- and that asymmetry is deliberate,
but it is a CLOSE CALL decided on reproducibility rather than on a win. New uv =
midpoint of the edge's two uvs; old uvs are left exactly alone, which reproduces
the authored UV FUNCTION exactly: the same piecewise-linear map over the same
coarse triangles, merely sampled at more points. The alternative -- running the
Loop masks on the uvs too -- was built and measured rather than argued about:

  metric: % of SURFACE AREA whose texel density is >0.5 mip from the median
  source tshirt_hi        0.40%
  linear uv (shipped)     3.93%   u 4.243..7.817  v 3.939..7.953  (identical to source)
  Loop-smoothed uv        3.37%   u 4.271..7.781  v 3.947..7.943  (chart pulled in ~1%)

So smoothing is 0.56 points BETTER on density and costs a chart that shrinks at
its rim, shifting every texel by up to 0.047 uv units (~5% of a tile). At a full
1-mip bar both land near half a percent of the garment (source 0.096%, shipped
0.541%), i.e. neither is visible. Linear wins because "the map is byte-for-byte
the one the ARAP solve produced" is a property worth more than 0.56 points on a
metric where both pass -- and because a shrinking chart is the sort of thing that
silently invalidates the source asset's own distortion numbers.

BOUNDARIES USE LOOP'S BOUNDARY RULES, which is what keeps this from eating the
garment's openings. A t-shirt is a surface with FOUR holes -- neck, two cuffs,
hem -- and if the interior rule is applied there the openings pucker shut and
the hem rounds off like a bag. New boundary point is the plain edge midpoint;
an old boundary vertex uses the cubic-B-spline mask (1/8, 3/4, 1/8) along the
boundary curve only, so each opening converges to a smooth curve through its own
original outline and never feels the interior of the cloth.

USAGE

    python scripts/dev/subdivide_garment.py                  # writes both assets
    python scripts/dev/subdivide_garment.py --levels 1 --out .../tshirt_md.obj
    python scripts/dev/subdivide_garment.py --report-only    # stats, no write
    python scripts/dev/subdivide_garment.py --preview-dir DIR  # + PNG previews

Deterministic: same input + same level count => byte-identical OBJ. Pure stdlib
+ numpy; the optional `--preview-dir` additionally wants matplotlib.
"""

import argparse
import math
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
MESH_DIR = os.path.join(REPO, "projects", "samples", "demos", "worlds", "physics", "meshes")
DEFAULT_SRC = os.path.join(MESH_DIR, "tshirt_hi.obj")

# Written coordinate precision. Matches usd_to_cloth_obj.write_obj so the two
# generators produce comparable files; 1 um on a 0.65 m garment.
POS_FMT = "%.6f"
UV_FMT = "%.6f"


# --------------------------------------------------------------------------- io


def read_obj(path):
    """Read a triangle OBJ with at most one uv per position.

    Returns (points (N,3) float64, uvs (N,2) float64 or None, tris (M,3) int32).

    ⚠ REFUSES a file whose `f` records pair a position with a different uv
    index. That is exactly the UV-seam split invariant 1 forbids, and reading it
    silently would mean subdividing a mesh that the engine is going to mangle at
    load time anyway. Better to fail here, where the message can say why.
    """
    pos, uv, tris = [], [], []
    seam_examples = []
    with open(path, "r", encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            tok = line.split()
            kind = tok[0]
            if kind == "v":
                pos.append([float(tok[1]), float(tok[2]), float(tok[3])])
            elif kind == "vt":
                uv.append([float(tok[1]), float(tok[2])])
            elif kind == "vn":
                raise ValueError(
                    "%s:%d has a `vn` record. Cloth normals are recomputed per frame from "
                    "the simulated positions; a stored normal is a per-vertex attribute that "
                    "makes loaders split one position into several vertices at every crease."
                    % (path, lineno))
            elif kind == "f":
                if len(tok) != 4:
                    raise ValueError("%s:%d is not a triangle (%d corners)" % (path, lineno, len(tok) - 1))
                corner = []
                for t in tok[1:]:
                    parts = t.split("/")
                    vi = int(parts[0]) - 1
                    ti = int(parts[1]) - 1 if len(parts) > 1 and parts[1] else vi
                    if ti != vi and len(seam_examples) < 5:
                        seam_examples.append((lineno, vi + 1, ti + 1))
                    corner.append(vi)
                tris.append(corner)
    if seam_examples:
        raise ValueError(
            "%s pairs a position with a different uv index (e.g. line %d: v %d / vt %d). "
            "That is a UV SEAM, and OmCloth welds by position keeping the first uv it sees, "
            "so the seam does not survive the load. Re-unwrap with a folded/continuous map "
            "(see usd_to_cloth_obj.arap_panel_uvs) before subdividing."
            % (path, seam_examples[0][0], seam_examples[0][1], seam_examples[0][2]))
    points = np.asarray(pos, dtype=np.float64)
    tris = np.asarray(tris, dtype=np.int64)
    uvs = np.asarray(uv, dtype=np.float64) if uv else None
    if uvs is not None and len(uvs) != len(points):
        raise ValueError("%s has %d vt for %d v -- this reader requires one uv per position"
                         % (path, len(uvs), len(points)))
    return points, uvs, tris


def write_obj(path, points, uvs, tris, header_lines):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        for line in header_lines:
            fh.write(("# %s\n" % line) if line else "#\n")
        for p in points:
            fh.write(("v " + POS_FMT + " " + POS_FMT + " " + POS_FMT + "\n") % (p[0], p[1], p[2]))
        if uvs is not None:
            for t in uvs:
                fh.write(("vt " + UV_FMT + " " + UV_FMT + "\n") % (t[0], t[1]))
        for t in tris:
            a, b, c = int(t[0]) + 1, int(t[1]) + 1, int(t[2]) + 1     # OBJ is 1-based
            if uvs is None:
                fh.write("f %d %d %d\n" % (a, b, c))
            else:
                fh.write("f %d/%d %d/%d %d/%d\n" % (a, a, b, b, c, c))


# ---------------------------------------------------------------------- topology


def edge_table(tris):
    """canonical undirected edge -> list of (face index, opposite vertex)."""
    tab = defaultdict(list)
    for fi, (a, b, c) in enumerate(tris):
        for (u, v, w) in ((a, b, c), (b, c, a), (c, a, b)):
            key = (u, v) if u < v else (v, u)
            tab[key].append((fi, int(w)))
    return tab


def boundary_loops(tris):
    """Trace the boundary as directed cycles.

    Returns (loops, non_manifold_boundary_verts). Each loop is a vertex list.
    A t-shirt should give FOUR: neck, two cuffs, hem.
    """
    half = set()
    for a, b, c in tris:
        half.add((int(a), int(b)))
        half.add((int(b), int(c)))
        half.add((int(c), int(a)))
    nxt = defaultdict(list)
    for (u, v) in half:
        if (v, u) not in half:
            nxt[u].append(v)                      # boundary runs opposite the face winding
    bad = sorted(u for u, vs in nxt.items() if len(vs) != 1)
    loops, seen = [], set()
    for start in sorted(nxt.keys()):
        if start in seen or len(nxt[start]) != 1:
            continue
        loop, cur = [], start
        while cur not in seen:
            seen.add(cur)
            loop.append(cur)
            if len(nxt[cur]) != 1:
                break
            cur = nxt[cur][0]
        if loop:
            loops.append(loop)
    return loops, bad


# ----------------------------------------------------------------------- repair
#
# ⚠ THE SOURCE ASSET IS NOT CLEAN, AND SUBDIVISION IS A MAGNIFIER. `tshirt_hi.obj`
# ships with three defects that the vertex-clustering decimation in
# `usd_to_cloth_obj.py` left behind and that nothing downstream ever checked for:
#
#   * TWO ~1.5 cm HOLES -- a 4-vertex loop at the left shoulder (perimeter
#     0.0487 m) and another just below the front neckline (0.0577 m). They are
#     tears, not garment openings: the four REAL openings measure 0.9910 m (hem),
#     0.4539 (neck) and 0.3587 / 0.3497 (the two cuffs), so there is a 6x gap
#     between the largest tear and the smallest opening and no ambiguity about
#     which is which. A hole is worse in cloth than in a render -- its rim has
#     bending resistance on one side only, so it flaps.
#   * ONE INCONSISTENTLY WOUND TRIANGLE PAIR sharing edge (190, 227), on the rim
#     of the shoulder tear. Its normal points the wrong way, which is a shading
#     speck now and four specks per subdivision level later.
#
# Repair runs BEFORE subdivision, because fixing a defect at 2394 vertices costs
# 4 triangles and fixing it at 37632 costs 64. It is conservative by
# construction: it only fills loops far below the smallest genuine opening, only
# flips faces to agree with their neighbours, and re-verifies afterwards.


def orient_faces(tris):
    """Flood-fill a consistent winding across the surface.

    Two triangles sharing an edge are consistent when they traverse that edge in
    OPPOSITE directions. Returns (tris, n_flipped, n_components, residual).
    `residual` re-measures the defect afterwards and must be 0 -- a garment is an
    orientable surface, so anything else means the mesh is worse than this pass
    can fix and the caller should stop.
    """
    tris = np.array(tris, dtype=np.int64, copy=True)
    e2f = defaultdict(list)
    for fi, (a, b, c) in enumerate(tris):
        for (u, v) in ((a, b), (b, c), (c, a)):
            e2f[(u, v) if u < v else (v, u)].append(fi)

    visited = np.zeros(len(tris), dtype=bool)
    flipped = np.zeros(len(tris), dtype=bool)
    comps = 0
    for seed in range(len(tris)):
        if visited[seed]:
            continue
        comps += 1
        visited[seed] = True
        stack = [seed]
        while stack:
            fi = stack.pop()
            a, b, c = tris[fi]
            for (u, v) in ((a, b), (b, c), (c, a)):
                key = (u, v) if u < v else (v, u)
                for fj in e2f[key]:
                    if fj == fi or visited[fj]:
                        continue
                    x, y, z = tris[fj]
                    dj = ((int(x), int(y)), (int(y), int(z)), (int(z), int(x)))
                    if (int(u), int(v)) in dj:      # same direction => inconsistent
                        tris[fj] = tris[fj][::-1]
                        flipped[fj] = True
                    visited[fj] = True
                    stack.append(fj)

    n_flipped = int(flipped.sum())
    if n_flipped * 2 > len(tris):                   # keep the input's majority sense
        tris = tris[:, ::-1].copy()
        n_flipped = len(tris) - n_flipped

    # a DIRECTED half-edge seen twice == two faces traversing it the same way
    seen = defaultdict(int)
    for a, b, c in tris:
        for (u, v) in ((a, b), (b, c), (c, a)):
            seen[(int(u), int(v))] += 1
    residual = sum(1 for k in seen.values() if k > 1)
    return tris, n_flipped, comps, residual


def _triangulate_loop(points, poly):
    """Cap a small hole. Tries every fan root and keeps the triangulation whose
    WORST triangle is best, which for the 3- and 4-gons this pass is allowed to
    touch covers every distinct triangulation there is."""
    n = len(poly)
    if n < 3:
        return []
    if n == 3:
        return [(poly[0], poly[1], poly[2])]
    best, best_score = None, -1.0
    for root in range(n):
        fan, worst = [], 1e30
        for k in range(1, n - 1):
            i, j, m = poly[root], poly[(root + k) % n], poly[(root + k + 1) % n]
            e0, e1 = points[j] - points[i], points[m] - points[i]
            a = 0.5 * float(np.linalg.norm(np.cross(e0, e1)))
            per = (float(np.linalg.norm(e0)) + float(np.linalg.norm(e1))
                   + float(np.linalg.norm(points[m] - points[j])))
            worst = min(worst, a / max(per * per, 1e-30))     # ~ shape quality
            fan.append((i, j, m))
        if worst > best_score:
            best, best_score = fan, worst
    return best


def fill_small_holes(points, tris, max_perim=0.15, max_verts=8):
    """Cap boundary loops far below the size of a real garment opening.

    Returns (tris, filled, kept) where `filled` and `kept` describe every loop so
    the decision is auditable rather than a silent count.
    """
    loops, _bad = boundary_loops(tris)
    add, filled, kept = [], [], []
    for loop in loops:
        n = len(loop)
        per = float(sum(np.linalg.norm(points[loop[(k + 1) % n]] - points[loop[k]])
                        for k in range(n)))
        z = points[loop][:, 2]
        info = {"verts": n, "perim_m": per, "z_min": float(z.min()), "z_max": float(z.max()),
                "x_min": float(points[loop][:, 0].min()), "x_max": float(points[loop][:, 0].max())}
        if 3 <= n <= max_verts and per <= max_perim:
            # the cap must traverse each boundary edge opposite to the face that
            # already owns it, so the polygon is the REVERSED boundary cycle
            add.extend(_triangulate_loop(points, loop[::-1]))
            filled.append(info)
        else:
            kept.append(info)
    if add:
        tris = np.vstack([tris, np.asarray(add, dtype=np.int64)])
    return tris, filled, kept


# ------------------------------------------------------------------ subdivision


def _loop_beta(n):
    """Loop's original vertex mask weight. Agrees with Warren's 3/(8n) at n=3,6."""
    if n == 3:
        return 3.0 / 16.0
    c = 0.375 + 0.25 * math.cos(2.0 * math.pi / n)
    return (0.625 - c * c) / n


def loop_subdivide(points, uvs, tris):
    """One level of Loop subdivision. Each triangle -> 4.

    Geometry uses the full Loop masks (interior 3/8-1/8 edge point + valence
    vertex mask; boundary midpoint + 1/8-3/4-1/8 curve mask). UVs use LINEAR
    midpoints with old uvs untouched -- see the module docstring for why the two
    differ.
    """
    nv = len(points)
    etab = edge_table(tris)

    nm = [k for k, v in etab.items() if len(v) > 2]
    if nm:
        raise ValueError(
            "input has %d NON-MANIFOLD edges (used by >2 triangles), e.g. %r. Subdividing "
            "one would smear the defect over four triangles instead of fixing it. On a "
            "garment this is usually a front-to-back merge -- the tube stitched shut."
            % (len(nm), nm[:3]))

    # --- neighbour rings, split into boundary and interior ---------------------
    ring = defaultdict(set)
    bnd_ring = defaultdict(set)
    for (u, v), inc in etab.items():
        ring[u].add(v)
        ring[v].add(u)
        if len(inc) == 1:
            bnd_ring[u].add(v)
            bnd_ring[v].add(u)

    # --- new edge points -------------------------------------------------------
    edge_index = {}
    new_pos, new_uv = [], []
    for key in sorted(etab.keys()):
        u, v = key
        inc = etab[key]
        if len(inc) == 2:
            w0, w1 = inc[0][1], inc[1][1]
            p = 0.375 * (points[u] + points[v]) + 0.125 * (points[w0] + points[w1])
        else:                                     # boundary edge: plain midpoint
            p = 0.5 * (points[u] + points[v])
        edge_index[key] = nv + len(new_pos)
        new_pos.append(p)
        if uvs is not None:
            new_uv.append(0.5 * (uvs[u] + uvs[v]))

    # --- repositioned old vertices --------------------------------------------
    old_pos = np.empty_like(points)
    for i in range(nv):
        nb = bnd_ring.get(i)
        if nb:
            if len(nb) == 2:                      # cubic B-spline along the opening
                a, b = sorted(nb)
                old_pos[i] = 0.75 * points[i] + 0.125 * (points[a] + points[b])
            else:
                old_pos[i] = points[i]            # pinched boundary: leave it put
            continue
        r = ring[i]
        n = len(r)
        if n < 3:
            old_pos[i] = points[i]
            continue
        beta = _loop_beta(n)
        acc = np.zeros(3)
        for j in r:
            acc += points[j]
        old_pos[i] = (1.0 - n * beta) * points[i] + beta * acc

    out_pos = np.vstack([old_pos, np.asarray(new_pos, dtype=np.float64)])
    out_uv = None
    if uvs is not None:
        out_uv = np.vstack([uvs.copy(), np.asarray(new_uv, dtype=np.float64)])

    # --- 1 triangle -> 4, winding preserved ------------------------------------
    def em(u, v):
        return edge_index[(u, v) if u < v else (v, u)]

    out_tris = np.empty((len(tris) * 4, 3), dtype=np.int64)
    for fi, (a, b, c) in enumerate(tris):
        a, b, c = int(a), int(b), int(c)
        ab, bc, ca = em(a, b), em(b, c), em(c, a)
        out_tris[4 * fi + 0] = (a, ab, ca)
        out_tris[4 * fi + 1] = (b, bc, ab)
        out_tris[4 * fi + 2] = (c, ca, bc)
        out_tris[4 * fi + 3] = (ab, bc, ca)
    return out_pos, out_uv, out_tris


def reframe(points):
    """Re-apply the asset's documented local frame: bbox centred on XY, lowest
    vertex at z = 0. Loop moves every vertex a little, so the invariant the
    worlds reason against ("the hem sits at local z 0, so `translation 0 0 0.62`
    puts it 0.62 m up") has to be restored or it quietly drifts. Returns the
    offset applied so the caller can report it."""
    lo = points.min(axis=0)
    hi = points.max(axis=0)
    off = np.array([-0.5 * (lo[0] + hi[0]), -0.5 * (lo[1] + hi[1]), -lo[2]])
    return points + off, off


# -------------------------------------------------------------------- reporting


def check_mesh(points, uvs, tris, pos_fmt=POS_FMT):
    """Everything the caller needs to decide whether this mesh is safe to hand a
    cloth solver, measured rather than assumed."""
    r = {}
    r["verts"] = int(len(points))
    r["tris"] = int(len(tris))
    r["corners"] = int(tris.size)

    p = points[tris]                                          # (M,3,3)
    e0 = p[:, 1] - p[:, 0]
    e1 = p[:, 2] - p[:, 1]
    e2 = p[:, 0] - p[:, 2]
    lens = np.concatenate([np.linalg.norm(e0, axis=1),
                           np.linalg.norm(e1, axis=1),
                           np.linalg.norm(e2, axis=1)])
    # unique undirected edges, for the honest edge-length distribution
    ekeys = np.concatenate([np.sort(tris[:, [0, 1]], axis=1),
                            np.sort(tris[:, [1, 2]], axis=1),
                            np.sort(tris[:, [2, 0]], axis=1)])
    uek = np.unique(ekeys, axis=0)
    ulen = np.linalg.norm(points[uek[:, 0]] - points[uek[:, 1]], axis=1)
    r["edges"] = int(len(uek))
    r["edge_min"] = float(ulen.min())
    r["edge_mean"] = float(ulen.mean())
    r["edge_max"] = float(ulen.max())
    r["edge_p05"] = float(np.percentile(ulen, 5))
    r["edge_p50"] = float(np.percentile(ulen, 50))
    r["edge_p95"] = float(np.percentile(ulen, 95))

    area = 0.5 * np.linalg.norm(np.cross(e0, -e2), axis=1)
    r["area_total"] = float(area.sum())
    r["tri_area_min"] = float(area.min())
    r["tri_area_mean"] = float(area.mean())
    r["degenerate_tris"] = int((area < 1e-12).sum())
    r["thin_tris_1e9"] = int((area < 1e-9).sum())
    r["zero_len_edges"] = int((lens < 1e-9).sum())
    # aspect ratio = longest edge / (2 * inradius); 1.0 is equilateral
    s = 0.5 * (np.linalg.norm(e0, axis=1) + np.linalg.norm(e1, axis=1) + np.linalg.norm(e2, axis=1))
    with np.errstate(divide="ignore", invalid="ignore"):
        inr = np.where(s > 0, area / s, 0.0)
        longest = np.maximum(np.maximum(np.linalg.norm(e0, axis=1), np.linalg.norm(e1, axis=1)),
                             np.linalg.norm(e2, axis=1))
        ar = np.where(inr > 0, longest / (2.0 * inr), np.inf)
    r["aspect_p50"] = float(np.percentile(ar[np.isfinite(ar)], 50))
    r["aspect_p95"] = float(np.percentile(ar[np.isfinite(ar)], 95))
    r["aspect_max"] = float(ar.max())

    # topology
    etab = edge_table(tris)
    counts = np.array([len(v) for v in etab.values()])
    r["boundary_edges"] = int((counts == 1).sum())
    r["nonmanifold_edges"] = int((counts > 2).sum())
    loops, bad = boundary_loops(tris)
    r["boundary_loops"] = len(loops)
    r["boundary_loop_sizes"] = sorted((len(l) for l in loops), reverse=True)
    r["boundary_loop_lengths_m"] = sorted(
        (float(sum(np.linalg.norm(points[l[(k + 1) % len(l)]] - points[l[k]]) for k in range(len(l))))
         for l in loops), reverse=True)
    r["nonmanifold_boundary_verts"] = len(bad)

    used = np.zeros(len(points), dtype=bool)
    used[tris.reshape(-1)] = True
    r["orphan_verts"] = int((~used).sum())

    # welding: duplicates AT THE WRITTEN PRECISION, which is what the loader sees
    txt = np.array([(pos_fmt + "|" + pos_fmt + "|" + pos_fmt) % (a, b, c) for a, b, c in points])
    uniq = len(set(txt.tolist()))
    r["unique_positions_written"] = uniq
    r["duplicate_positions_written"] = int(len(points) - uniq)
    r["unique_positions_float64"] = int(len(np.unique(points, axis=0)))

    # duplicate triangles (same vertex set)
    tsorted = np.sort(tris, axis=1)
    r["duplicate_tris"] = int(len(tsorted) - len(np.unique(tsorted, axis=0)))

    lo, hi = points.min(axis=0), points.max(axis=0)
    r["bbox_min"] = [float(x) for x in lo]
    r["bbox_max"] = [float(x) for x in hi]
    r["bbox_size"] = [float(x) for x in (hi - lo)]

    if uvs is not None:
        q = uvs[tris]
        d0, d1 = q[:, 1] - q[:, 0], q[:, 2] - q[:, 0]
        uarea = 0.5 * np.abs(d0[:, 0] * d1[:, 1] - d0[:, 1] * d1[:, 0])   # 2D cross, no deprecation
        r["uv_area_total"] = float(uarea.sum())
        r["uv_texels_per_m2"] = float(uarea.sum() / area.sum()) if area.sum() > 0 else 0.0
        r["uv_min"] = [float(x) for x in uvs.min(axis=0)]
        r["uv_max"] = [float(x) for x in uvs.max(axis=0)]
        ok = area > 1e-14
        scale = np.zeros(len(area))
        scale[ok] = uarea[ok] / area[ok]
        good = scale > 0
        r["uv_density_p05"] = float(np.percentile(scale[good], 5))
        r["uv_density_p50"] = float(np.percentile(scale[good], 50))
        r["uv_density_p95"] = float(np.percentile(scale[good], 95))
        r["uv_degenerate_tris"] = int((uarea < 1e-14).sum())
    return r


def pin_top_band(points, band):
    """How many vertices `Cloth { pinTopBand <band> }` would freeze. The world's
    own header tracks this number, and it changes with every resolution swap."""
    zmax = float(points[:, 2].max())
    return int((points[:, 2] >= zmax - band).sum()), zmax


def fmt_report(name, r):
    L = []
    L.append("%s" % name)
    L.append("  verts %d   tris %d   unique edges %d   corners %d"
             % (r["verts"], r["tris"], r["edges"], r["corners"]))
    L.append("  edge  min %.5f  p05 %.5f  mean %.5f  p50 %.5f  p95 %.5f  max %.5f  (m)"
             % (r["edge_min"], r["edge_p05"], r["edge_mean"], r["edge_p50"], r["edge_p95"], r["edge_max"]))
    L.append("  tri   area min %.3e  mean %.3e  total %.4f m^2   degenerate(<1e-12) %d  thin(<1e-9) %d"
             % (r["tri_area_min"], r["tri_area_mean"], r["area_total"],
                r["degenerate_tris"], r["thin_tris_1e9"]))
    L.append("  shape aspect p50 %.2f  p95 %.2f  max %.2f   zero-length edges %d"
             % (r["aspect_p50"], r["aspect_p95"], r["aspect_max"], r["zero_len_edges"]))
    L.append("  weld  positions written %d / %d unique -> %d duplicates   (float64 unique %d)"
             % (r["unique_positions_written"], r["verts"], r["duplicate_positions_written"],
                r["unique_positions_float64"]))
    L.append("  topo  boundary edges %d  loops %d %s   non-manifold edges %d  bad boundary verts %d"
             % (r["boundary_edges"], r["boundary_loops"], r["boundary_loop_sizes"],
                r["nonmanifold_edges"], r["nonmanifold_boundary_verts"]))
    L.append("        boundary loop lengths (m) %s"
             % " ".join("%.3f" % x for x in r["boundary_loop_lengths_m"]))
    L.append("  hyg   orphan verts %d   duplicate tris %d" % (r["orphan_verts"], r["duplicate_tris"]))
    L.append("  bbox  size %.4f x %.4f x %.4f m   z %.4f .. %.4f"
             % (r["bbox_size"][0], r["bbox_size"][1], r["bbox_size"][2],
                r["bbox_min"][2], r["bbox_max"][2]))
    if "uv_texels_per_m2" in r:
        L.append("  uv    %.2f uv-area/m^2 (tiles/m %.2f)  density p05/p50/p95 %.2f/%.2f/%.2f  "
                 "degenerate %d  range u %.3f..%.3f v %.3f..%.3f"
                 % (r["uv_texels_per_m2"], math.sqrt(max(r["uv_texels_per_m2"], 0.0)),
                    r["uv_density_p05"], r["uv_density_p50"], r["uv_density_p95"],
                    r["uv_degenerate_tris"], r["uv_min"][0], r["uv_max"][0],
                    r["uv_min"][1], r["uv_max"][1]))
    else:
        L.append("  uv    NONE")
    return "\n".join(L)


def verdict(r):
    """Hard gates. Anything here going non-zero means do not ship the file."""
    bad = []
    if r["duplicate_positions_written"]:
        bad.append("%d positions collide at the written precision -- the loader would weld them "
                   "into a pinch point" % r["duplicate_positions_written"])
    if r["nonmanifold_edges"]:
        bad.append("%d non-manifold edges" % r["nonmanifold_edges"])
    if r["orphan_verts"]:
        bad.append("%d orphan vertices (newton would silently pin them, mass 0.0)" % r["orphan_verts"])
    if r["degenerate_tris"]:
        bad.append("%d degenerate triangles" % r["degenerate_tris"])
    if r["duplicate_tris"]:
        bad.append("%d duplicate triangles" % r["duplicate_tris"])
    if r["zero_len_edges"]:
        bad.append("%d zero-length edges" % r["zero_len_edges"])
    if r.get("uv_degenerate_tris"):
        bad.append("%d uv-degenerate triangles" % r["uv_degenerate_tris"])
    return bad


# ---------------------------------------------------------------------- preview


def _shade(points, tris, axes, size, light, bg=(0.10, 0.11, 0.13)):
    """Flat-shaded orthographic render, painter's algorithm, no GPU."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    ax_u, ax_v, ax_d, flip_u = axes
    p = points[tris]
    n = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = n / np.maximum(ln, 1e-16)
    # two-sided: a garment is an open surface, so backfaces must light too
    lam = np.abs(n @ np.asarray(light, dtype=np.float64))
    shade = 0.22 + 0.78 * lam ** 0.85
    depth = p[:, :, ax_d].mean(axis=1)
    order = np.argsort(depth)

    u = p[:, :, ax_u] * (-1.0 if flip_u else 1.0)
    v = p[:, :, ax_v]
    verts = np.stack([u, v], axis=2)[order]
    base = np.array([0.36, 0.52, 0.90])
    col = np.clip(shade[order][:, None] * base[None, :], 0, 1)

    fig = plt.figure(figsize=(size[0] / 100.0, size[1] / 100.0), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor(bg)
    fig.patch.set_facecolor(bg)
    pc = PolyCollection(verts, facecolors=col, edgecolors="none", antialiased=False)
    ax.add_collection(pc)
    ax.set_aspect("equal")
    lo = np.array([u.min(), v.min()])
    hi = np.array([u.max(), v.max()])
    pad = 0.05 * max(hi - lo)
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.axis("off")
    return fig, ax


def preview(points, tris, out_png, title, wire_crop=None):
    """Front / side / top shaded views + a wireframe crop that shows tessellation."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection, LineCollection

    p = points[tris]
    n = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    n = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-16)

    # (u, v, depth, flip_u, label)  -- ENU: x across, y front/back, z up
    views = [(0, 2, 1, False, "front  (-Y)"),
             (1, 2, 0, False, "side  (+X)"),
             (0, 1, 2, False, "top  (-Z)")]
    light = np.array([0.35, -0.80, 0.49])
    light = light / np.linalg.norm(light)

    ncol = len(views) + (1 if wire_crop is not None else 0)
    fig = plt.figure(figsize=(4.2 * ncol, 5.4), dpi=110)
    fig.patch.set_facecolor("#15171b")

    for k, (au, av, ad, flip, label) in enumerate(views):
        ax = fig.add_subplot(1, ncol, k + 1)
        ax.set_facecolor("#15171b")
        lam = np.abs(n @ light)
        shade = 0.20 + 0.80 * lam ** 0.85
        order = np.argsort(p[:, :, ad].mean(axis=1))
        u = p[:, :, au] * (-1.0 if flip else 1.0)
        v = p[:, :, av]
        verts = np.stack([u, v], axis=2)[order]
        base = np.array([0.38, 0.54, 0.92])
        col = np.clip(shade[order][:, None] * base[None, :], 0, 1)
        ax.add_collection(PolyCollection(verts, facecolors=col, edgecolors="none", antialiased=False))
        ax.set_aspect("equal")
        ax.set_xlim(u.min() - 0.03, u.max() + 0.03)
        ax.set_ylim(v.min() - 0.03, v.max() + 0.03)
        ax.set_title(label, color="#d8dbe0", fontsize=10)
        ax.tick_params(colors="#8a9099", labelsize=7)
        for sp in ax.spines.values():
            sp.set_color("#3a3f47")
        ax.grid(True, color="#2a2e34", lw=0.4)

    if wire_crop is not None:
        (x0, x1, z0, z1) = wire_crop
        ax = fig.add_subplot(1, ncol, ncol)
        ax.set_facecolor("#15171b")
        keep = ((p[:, :, 0].mean(axis=1) > x0) & (p[:, :, 0].mean(axis=1) < x1) &
                (p[:, :, 2].mean(axis=1) > z0) & (p[:, :, 2].mean(axis=1) < z1))
        q = p[keep]
        segs = []
        for tri in q:
            segs.append([(tri[0, 0], tri[0, 2]), (tri[1, 0], tri[1, 2])])
            segs.append([(tri[1, 0], tri[1, 2]), (tri[2, 0], tri[2, 2])])
            segs.append([(tri[2, 0], tri[2, 2]), (tri[0, 0], tri[0, 2])])
        ax.add_collection(LineCollection(segs, colors="#7fd4ff", linewidths=0.35, alpha=0.85))
        ax.set_aspect("equal")
        ax.set_xlim(x0, x1)
        ax.set_ylim(z0, z1)
        ax.set_title("wireframe crop  %d tris" % int(keep.sum()), color="#d8dbe0", fontsize=10)
        ax.tick_params(colors="#8a9099", labelsize=7)
        for sp in ax.spines.values():
            sp.set_color("#3a3f47")

    fig.suptitle(title, color="#f0f2f5", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    os.makedirs(os.path.dirname(os.path.abspath(out_png)), exist_ok=True)
    fig.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out_png


def preview_uv(uvs, tris, out_png, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    q = uvs[tris]
    segs = []
    for tri in q:
        segs.append([tuple(tri[0]), tuple(tri[1])])
        segs.append([tuple(tri[1]), tuple(tri[2])])
        segs.append([tuple(tri[2]), tuple(tri[0])])
    fig = plt.figure(figsize=(6, 6), dpi=110)
    fig.patch.set_facecolor("#15171b")
    ax = fig.add_subplot(111)
    ax.set_facecolor("#15171b")
    ax.add_collection(LineCollection(segs, colors="#ffd479", linewidths=0.18, alpha=0.7))
    ax.set_aspect("equal")
    ax.set_xlim(uvs[:, 0].min() - 0.1, uvs[:, 0].max() + 0.1)
    ax.set_ylim(uvs[:, 1].min() - 0.1, uvs[:, 1].max() + 0.1)
    ax.set_title(title, color="#f0f2f5", fontsize=12)
    ax.tick_params(colors="#8a9099", labelsize=8)
    for sp in ax.spines.values():
        sp.set_color("#3a3f47")
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_png)), exist_ok=True)
    fig.savefig(out_png, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out_png


# --------------------------------------------------------------------------- go


def build(src, levels, out, do_reframe=True, src_report=None, quiet=False,
          repair=True, hole_perim=0.15):
    points, uvs, tris = read_obj(src)
    r0 = src_report if src_report is not None else check_mesh(points, uvs, tris)
    if not quiet:
        print(fmt_report("SOURCE  %s" % os.path.basename(src), r0))
        print()

    repair_notes = []
    if repair:
        tris, n_flip, comps, residual = orient_faces(tris)
        if residual:
            raise SystemExit("orient_faces left %d inconsistently wound edges -- the surface is "
                             "not orientable, which is beyond this pass" % residual)
        tris, filled, kept = fill_small_holes(points, tris, max_perim=hole_perim)
        repair_notes.append("winding: %d face(s) flipped to agree with their neighbours, "
                            "%d shell(s), 0 residual" % (n_flip, comps))
        for h in filled:
            repair_notes.append("filled tear: %d-vertex loop, perimeter %.4f m, z %.3f..%.3f, "
                                "x %.3f..%.3f" % (h["verts"], h["perim_m"], h["z_min"], h["z_max"],
                                                  h["x_min"], h["x_max"]))
        for h in kept:
            repair_notes.append("kept opening: %d-vertex loop, perimeter %.4f m, z %.3f..%.3f"
                                % (h["verts"], h["perim_m"], h["z_min"], h["z_max"]))
        if not quiet:
            print("  REPAIR (pre-subdivision, on the source's own defects):")
            for n in repair_notes:
                print("    %s" % n)
            rr = check_mesh(points, uvs, tris)
            print("    -> %d tris, %d boundary loops %s, %d non-manifold, %d bad boundary verts"
                  % (rr["tris"], rr["boundary_loops"], rr["boundary_loop_sizes"],
                     rr["nonmanifold_edges"], rr["nonmanifold_boundary_verts"]))
            print()

    cur_p, cur_uv, cur_t = points, uvs, tris
    for lv in range(levels):
        cur_p, cur_uv, cur_t = loop_subdivide(cur_p, cur_uv, cur_t)
        if not quiet:
            print("  level %d -> %d verts / %d tris" % (lv + 1, len(cur_p), len(cur_t)))

    off = np.zeros(3)
    if do_reframe:
        cur_p, off = reframe(cur_p)

    r = check_mesh(cur_p, cur_uv, cur_t)
    bad = verdict(r)

    band = 0.06
    pin_n, zmax = pin_top_band(cur_p, band)
    pin0_n, zmax0 = pin_top_band(points, band)

    shrink = 100.0 * (1.0 - r["area_total"] / r0["area_total"])
    header = [
        "T-shirt cloth mesh for OmniSim `Cloth { url ... }` -- SUBDIVIDED.",
        "GENERATED by scripts/dev/subdivide_garment.py -- do not hand-edit.",
        "source: %s (%d v / %d tri), %d level(s) of Loop subdivision"
        % (os.path.basename(src), r0["verts"], r0["tris"], levels),
        "%d verts, %d tris, surface %.4f m^2 (source %.4f, -%.2f%% -- Loop is an"
        % (r["verts"], r["tris"], r["area_total"], r0["area_total"], shrink),
        "  approximating scheme, so the surface pulls in slightly; `density` is kg/m^2",
        "  so the garment's MASS drops by the same %.2f%%)" % shrink,
        "edge length m: min %.5f  p05 %.5f  mean %.5f  p95 %.5f  max %.5f"
        % (r["edge_min"], r["edge_p05"], r["edge_mean"], r["edge_p95"], r["edge_max"]),
        "min tri area %.3e m^2, degenerate %d, duplicate tris %d, orphan verts %d, "
        "zero-length edges %d, aspect p95 %.2f max %.2f"
        % (r["tri_area_min"], r["degenerate_tris"], r["duplicate_tris"], r["orphan_verts"],
           r["zero_len_edges"], r["aspect_p95"], r["aspect_max"]),
        "welded: %d positions, %d unique at the written 1e-6 precision (0 duplicates)"
        % (r["verts"], r["unique_positions_written"]),
        "non-manifold edges %d (must be 0), boundary edges %d in %d loops %s"
        % (r["nonmanifold_edges"], r["boundary_edges"], r["boundary_loops"], r["boundary_loop_sizes"]),
        "  -- a t-shirt is a surface with four holes: hem, neck, two cuffs",
        ("local frame: bbox centred on XY, lowest vertex at z = 0, re-applied after"
         " subdivision (offset %.6f %.6f %.6f m)" % (off[0], off[1], off[2])) if do_reframe
        else "local frame: NOT re-normalised (--no-reframe)",
        "max local z %.4f (source %.4f); `pinTopBand %.2f` selects %d verts (source %d)"
        % (zmax, zmax0, band, pin_n, pin0_n),
        "uv: inherited from the source unchanged and subdivided LINEARLY (new uv = edge",
        "  midpoint, old uvs untouched), so this is the SAME piecewise-linear map the",
        "  source carries, merely sampled more finely. One uv per position, vt index ==",
        "  v index -- OmCloth welds by position and keeps the first uv, so a seam here",
        "  would not survive the load. %.2f uv-area/m^2 = %.2f tiles/m."
        % (r.get("uv_texels_per_m2", 0.0), math.sqrt(max(r.get("uv_texels_per_m2", 0.0), 0.0))),
    ]
    if repair_notes:
        header.append("")
        header.append("repaired BEFORE subdividing (defects inherited from the source asset):")
        for n in repair_notes:
            header.append("  %s" % n)

    if bad:
        print("REFUSING TO WRITE %s:" % out, file=sys.stderr)
        for b in bad:
            print("  - %s" % b, file=sys.stderr)
        raise SystemExit(2)

    write_obj(out, cur_p, cur_uv, cur_t, header)
    r["bytes"] = os.path.getsize(out)
    r["path"] = out
    r["levels"] = levels
    r["pin_top_band_006"] = pin_n
    r["max_local_z"] = zmax
    r["area_shrink_pct"] = shrink
    r["reframe_offset"] = [float(x) for x in off]
    if not quiet:
        print()
        print(fmt_report("OUTPUT  %s  (%.2f MB)" % (out, r["bytes"] / 1e6), r))
        print("  world %s pinTopBand 0.06 -> %d verts (source %d);  max local z %.4f (source %.4f)"
              % ("", pin_n, pin0_n, zmax, zmax0))
        print("  reframe offset %.6f %.6f %.6f m" % (off[0], off[1], off[2]))
        print("  ALL GATES PASS")
    return cur_p, cur_uv, cur_t, r


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default=DEFAULT_SRC)
    ap.add_argument("--levels", type=int, default=None,
                    help="subdivision levels for a single --out (default: build both assets)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-reframe", action="store_true",
                    help="skip re-applying 'bbox centred on XY, lowest vertex at z=0'")
    ap.add_argument("--report-only", action="store_true", help="stats for --src, write nothing")
    ap.add_argument("--preview-dir", default=None, help="also write PNG previews here")
    ap.add_argument("--no-repair", action="store_true",
                    help="subdivide the source verbatim, defects and all (see the repair section)")
    ap.add_argument("--hole-perim", type=float, default=0.15,
                    help="boundary loops shorter than this are decimation tears and get capped "
                         "(default 0.15 m; the smallest REAL opening on this shirt is a 0.35 m "
                         "cuff, so there is a 2.3x margin)")
    args = ap.parse_args()

    if args.report_only:
        p, u, t = read_obj(args.src)
        print(fmt_report("%s" % args.src, check_mesh(p, u, t)))
        bad = verdict(check_mesh(p, u, t))
        print("  gates: %s" % ("PASS" if not bad else "; ".join(bad)))
        if args.preview_dir:
            preview(p, t, os.path.join(args.preview_dir, "%s.png" %
                                       os.path.splitext(os.path.basename(args.src))[0]),
                    "%s  %d v / %d t" % (os.path.basename(args.src), len(p), len(t)),
                    wire_crop=(-0.34, 0.02, 0.30, 0.66))
        return 0

    p0, u0, t0 = read_obj(args.src)
    r_src = check_mesh(p0, u0, t0)

    jobs = []
    if args.levels is not None:
        out = args.out or os.path.join(MESH_DIR, "tshirt_sub%d.obj" % args.levels)
        jobs.append((args.levels, out))
    else:
        jobs.append((1, os.path.join(MESH_DIR, "tshirt_md.obj")))
        jobs.append((2, os.path.join(MESH_DIR, "tshirt_uhd.obj")))

    results = []
    for lv, out in jobs:
        print("=" * 100)
        pts, uvs, tris, r = build(args.src, lv, out, do_reframe=not args.no_reframe,
                                  src_report=r_src, repair=not args.no_repair,
                                  hole_perim=args.hole_perim)
        results.append(r)
        if args.preview_dir:
            stem = os.path.splitext(os.path.basename(out))[0]
            preview(pts, tris, os.path.join(args.preview_dir, "%s.png" % stem),
                    "%s   %d v / %d tri   mean edge %.1f mm"
                    % (stem, r["verts"], r["tris"], 1000 * r["edge_mean"]),
                    wire_crop=(-0.34, 0.02, 0.30, 0.66))
            if uvs is not None:
                preview_uv(uvs, tris, os.path.join(args.preview_dir, "%s_uv.png" % stem),
                           "%s uv  (%.2f tiles/m)" % (stem, math.sqrt(r["uv_texels_per_m2"])))
        print()

    print("=" * 100)
    print("%-46s %8s %8s %10s %10s %8s %10s" %
          ("file", "verts", "tris", "mean edge", "min edge", "loops", "bytes"))
    print("%-46s %8d %8d %9.2fmm %9.2fmm %8d %10d" %
          (os.path.basename(args.src), r_src["verts"], r_src["tris"],
           1000 * r_src["edge_mean"], 1000 * r_src["edge_min"],
           r_src["boundary_loops"], os.path.getsize(args.src)))
    for r in results:
        print("%-46s %8d %8d %9.2fmm %9.2fmm %8d %10d" %
              (os.path.basename(r["path"]), r["verts"], r["tris"],
               1000 * r["edge_mean"], 1000 * r["edge_min"], r["boundary_loops"], r["bytes"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
