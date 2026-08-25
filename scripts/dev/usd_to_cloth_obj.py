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

"""Turn a USD garment into an OBJ a `Cloth { url ... }` node can simulate.

WHY THIS IS AN OFFLINE STEP AND NOT A LOADER. Three separate conversions have
to happen between "an artist's USD garment" and "a mesh SolverVBD can step",
and every one of them is a judgement call that belongs in a reviewable artefact
rather than in the engine's load path:

  1. UNITS AND UP AXIS. USD carries both as stage metadata; OmniSim's `.wbt`
     carries neither on a mesh. The vendored shirt is Y-up in CENTIMETRES, so a
     verbatim load would produce a 65-metre shirt lying on its side.
  2. DECIMATION. The shirt is 6436 vertices. Cloth is a per-particle GPU
     workload, so a demo that has to stay cheap wants a few hundred. Which
     vertices to drop is a mesh-processing decision, not a physics one.
  3. THE THREE NEWTON TRAPS. `ModelBuilder.add_cloth_mesh` silently turns an
     orphan vertex into a KINEMATIC PIN (builder.py:9154 leaves its mass at
     0.0), never welds a duplicate (adjacency keys on integer indices, so a
     UV-split seam gets no stretch and no bending across it), and drops a
     degenerate triangle with a bare `print`. Cleaning here means the shipped
     asset is known-good; the runtime cleans again on load because a
     user-supplied mesh is not.

The decimation is VERTEX CLUSTERING, not quadric edge collapse: it needs no
third-party library, it is deterministic, and welding is not a separate pass
because merging vertices IS the algorithm. The cost is that it does not respect
sharp features -- irrelevant for fabric, which has none.

⚠ THE FAILURE MODE TO WATCH IS FRONT-TO-BACK MERGE. A T-shirt is a flattened
tube; cluster it too coarsely and a cell swallows one vertex from the front
panel and one from the back, stitching the garment shut so it can never open.
That shows up as NON-MANIFOLD EDGES (an edge used by more than two triangles),
which this script counts and reports rather than leaving to be discovered as
"the shirt drapes like a board". Keep it at 0.

⚠ THE UV MAP IS THE FOURTH JUDGEMENT CALL, and it is the one that shows. The
garment carries NO authored UVs (the vendored `unisex_shirt.usd` has no
`primvars:st` / `texCoord2f` / `st` / `uv` of any kind), so there is nothing to
recover and the map must be generated. It is generated HERE rather than at load
for the same reason as the rest: it is a choice, and a bad one is only visible
three worlds away as "the fabric looks blotchy". `--uv arap-panel` is the
default and `uv_distortion` scores whatever gets written, so a regeneration that
makes it worse says so in the report. See `arap_panel_uvs` for why the obvious
front/back planar split measured WORSE than the map it would have replaced.

Run it with the interpreter that has `pxr` -- in this tree that is the bundled
Newton runtime's own Python, which vendors usd-core:

  msys64/mingw64/bin/newton-runtime/python.exe scripts/dev/usd_to_cloth_obj.py

Defaults reproduce the committed asset exactly (it is deterministic: same input
+ same target => byte-identical OBJ).
"""

import argparse
import json
import os
import sys

BUNDLE_SP = os.path.join("msys64", "mingw64", "bin", "newton-runtime", "site-packages")
DEFAULT_USD = os.path.join(BUNDLE_SP, "newton", "examples", "assets", "unisex_shirt.usd")
# Next to the world that uses it, in a `meshes/` subfolder -- the tree's
# convention for a world-local asset (cf. projects/samples/devices/worlds/meshes)
# and what a relative `url` in that world resolves against.
DEFAULT_OUT = os.path.join("projects", "samples", "demos", "worlds", "physics", "meshes", "tshirt.obj")


def _bootstrap_bundle_path():
    # The bundled runtime's python.exe does NOT put its own site-packages on
    # sys.path (the engine's embedded interpreter injects it), so numpy and pxr
    # are both invisible until we add it. Resolved relative to this file so the
    # script works from any cwd, and only prepended when it exists -- on a clone
    # with the runtime elsewhere, the interpreter's own path is left alone.
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    sp = os.path.join(here, BUNDLE_SP)
    if os.path.isdir(sp) and sp not in sys.path:
        sys.path.insert(0, sp)


_bootstrap_bundle_path()


def load_usd_mesh(path, prim_path=None):
    """Return (points Nx3 float64 in METRES Z-up, tris Mx3 int64, info dict)."""
    try:
        from pxr import Usd, UsdGeom
    except ImportError:
        sys.exit("pxr (usd-core) is not importable by %s.\n"
                 "Use the bundled Newton runtime's interpreter, which vendors it:\n"
                 "  msys64/mingw64/bin/newton-runtime/python.exe %s"
                 % (sys.executable, " ".join(sys.argv)))
    import numpy as np

    stage = Usd.Stage.Open(path)
    if stage is None:
        sys.exit("could not open USD stage: %s" % path)

    prim = None
    if prim_path:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsA(UsdGeom.Mesh):
            sys.exit("%s is not a UsdGeom.Mesh in %s" % (prim_path, path))
    else:
        for p in stage.Traverse():
            if p.IsA(UsdGeom.Mesh):
                prim = p
                break
        if prim is None:
            sys.exit("no UsdGeom.Mesh found in %s" % path)

    mesh = UsdGeom.Mesh(prim)
    pts = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)

    # Fan-triangulate anything that is not already a triangle. The vendored
    # shirt is all-triangles, but a garment exported from a DCC tool routinely
    # is not, and a quad silently read as a triangle is a corrupted mesh.
    if counts.size and int(counts.min()) == 3 and int(counts.max()) == 3:
        tris = idx.reshape(-1, 3)
    else:
        out, off = [], 0
        for c in counts:
            c = int(c)
            for k in range(1, c - 1):
                out.append((idx[off], idx[off + k], idx[off + k + 1]))
            off += c
        tris = np.asarray(out, dtype=np.int64)

    mpu = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    up = str(UsdGeom.GetStageUpAxis(stage) or "Y")
    pts = pts * mpu
    if up.upper().startswith("Y"):
        # Y-up -> Z-up (OmniSim ENU). A right-handed turn of -90 deg about X:
        # (x, y, z) -> (x, -z, y). NOT a bare axis swap, which would mirror the
        # garment and reverse every triangle's winding with it.
        pts = np.column_stack([pts[:, 0], -pts[:, 2], pts[:, 1]])
    return pts, tris, {"prim": str(prim.GetPath()), "meters_per_unit": mpu, "up_axis": up}


def clean(points, tris):
    """Weld exact duplicates, drop degenerate + duplicate tris, drop orphans.

    Returns (points, tris, stats). This is the same contract the runtime's
    loader applies -- kept in both places on purpose: here so the SHIPPED asset
    is known-good and the numbers appear in review, there because a user's mesh
    has never been through this script.
    """
    import numpy as np
    stats = {}

    uniq, inv = np.unique(points, axis=0, return_inverse=True)
    stats["welded_verts"] = int(len(points) - len(uniq))
    points, tris = uniq, inv[tris]

    deg = (tris[:, 0] == tris[:, 1]) | (tris[:, 1] == tris[:, 2]) | (tris[:, 0] == tris[:, 2])
    stats["degenerate_tris"] = int(deg.sum())
    tris = tris[~deg]

    # A duplicate face is two triangles over the same three vertices in any
    # order. Harmless to render, but it doubles that patch's stretch stiffness
    # and its mass contribution, so the fabric is locally stiff and heavy for
    # no reason an author can see.
    _, keep = np.unique(np.sort(tris, axis=1), axis=0, return_index=True)
    stats["duplicate_tris"] = int(len(tris) - len(keep))
    tris = tris[np.sort(keep)]

    used = np.zeros(len(points), dtype=bool)
    used[tris.reshape(-1)] = True
    stats["orphan_verts"] = int((~used).sum())
    if stats["orphan_verts"]:
        remap = np.full(len(points), -1, dtype=np.int64)
        remap[used] = np.arange(int(used.sum()))
        points, tris = points[used], remap[tris]
    return points, tris, stats


def cluster(points, tris, cell):
    """One CONNECTIVITY-AWARE vertex-clustering pass at the given cell size (m).

    ⚠ THE `connected` PART IS THE WHOLE POINT, AND PLAIN CLUSTERING IS A TRAP
    HERE. Textbook vertex clustering merges every vertex sharing a grid cell,
    which is fine for a solid but wrong for a garment: a T-shirt is a flattened
    tube whose front and back panels pass within a couple of centimetres of each
    other, so any cell coarse enough to decimate also swallows one vertex from
    each panel and FUSES THE SHIRT SHUT. MEASURED on the vendored shirt: 16 to 46
    non-manifold edges at every target from 620 to 1500 vertices, i.e. the defect
    does not decimate away -- it is structural.

    So a cell is not a cluster. Within each cell we take the connected components
    of the MESH graph restricted to that cell, and each component is a cluster.
    Two vertices therefore merge only when they are close in space AND joined by
    a path of real fabric that never leaves the cell -- which the front and back
    of a shirt never are.
    """
    import numpy as np
    key = np.floor((points - points.min(axis=0)) / cell).astype(np.int64)
    _, cell_id = np.unique(key, axis=0, return_inverse=True)

    # Union-find over mesh edges whose two ends share a cell. Path-halving find,
    # union by size: linear enough for the ~60 passes the binary search makes.
    parent = np.arange(len(points), dtype=np.int64)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    e = np.vstack([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]])
    e = e[cell_id[e[:, 0]] == cell_id[e[:, 1]]]
    for u, v in e:
        ru, rv = find(int(u)), find(int(v))
        if ru != rv:
            parent[ru] = rv

    roots = np.array([find(int(i)) for i in range(len(points))], dtype=np.int64)
    _, inv = np.unique(roots, return_inverse=True)
    n = int(inv.max()) + 1
    # Representative = the centroid of each cluster's members, not the cell
    # centre: a garment is a thin surface threading through a coarse grid, and
    # snapping to cell centres visibly inflates it and shifts the hems.
    rep = np.zeros((n, 3))
    np.add.at(rep, inv, points)
    cnt = np.bincount(inv, minlength=n).reshape(-1, 1)
    return rep / np.maximum(cnt, 1), inv[tris]


def nonmanifold_and_boundary(points, tris):
    import numpy as np
    e = np.sort(np.vstack([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]]), axis=1)
    _, c = np.unique(e, axis=0, return_counts=True)
    return int((c > 2).sum()), int((c == 1).sum())


def repair_nonmanifold(points, tris, max_removed=200):
    """Delete the fewest triangles that make every edge two-manifold.

    Connectivity-aware clustering removes the BULK of the non-manifoldness but
    not all of it -- collapsing a connected patch to a single vertex can still
    fold that patch onto itself. MEASURED on the vendored shirt across nine
    decimation targets from 400 to 2500 vertices: plain clustering left 16-46
    bad edges, connectivity-aware clustering left 4-14, and no target reached 0.
    It is a residue, so it has to be repaired rather than tuned away.

    Greedy and one-at-a-time: score each triangle by how many over-used edges it
    touches, drop the worst, recount. Removing a handful of triangles from ~1200
    opens at most a few tiny holes, and a hole in cloth is a hem -- newton gives
    boundary edges zero bending, which is the correct behaviour for a garment's
    edge anyway. The alternative (keeping two arbitrary faces per bad edge) would
    leave the fabric fused front-to-back at that point, which is the defect.
    """
    import numpy as np
    removed = 0
    for _ in range(max_removed):
        e = np.sort(np.vstack([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]]), axis=1)
        uniq, inv, cnt = np.unique(e, axis=0, return_inverse=True, return_counts=True)
        bad = cnt > 2
        if not bad.any():
            break
        # inv is laid out as 3 blocks of len(tris) (edge 01, 12, 20), so folding
        # it back to (3, ntris) and summing down the columns scores each triangle.
        score = bad[inv.reshape(3, -1)].sum(axis=0)
        tris = np.delete(tris, int(np.argmax(score)), axis=0)
        removed += 1
    return points, tris, removed


def decimate_to(points, tris, target):
    """Binary-search the clustering cell size for ~target vertices."""
    import numpy as np
    span = float(np.max(points.max(axis=0) - points.min(axis=0)))
    lo, hi = span / 4000.0, span          # hi collapses to a handful, lo keeps ~everything
    best = None
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        p, t = cluster(points, tris, mid)
        p, t, _s = clean(p, t)
        if best is None or abs(len(p) - target) < abs(best[0] - target):
            best = (len(p), mid, p, t)
        if len(p) > target:
            lo = mid                       # bigger cell => fewer vertices
        else:
            hi = mid
        if hi - lo < span * 1e-6:
            break
    return best[2], best[3], best[1]


def mirrored_cylindrical_uvs(points, tiles_per_m):
    """A SEAMLESS, per-vertex UV set for a garment authored as worn (+Z = torso).

    ⚠ WHY NOT A NORMAL CYLINDRICAL MAP. The obvious `u = atan2(y, x) / 2pi` has a
    branch cut, and the one column of triangles that straddles it interpolates u
    from ~1 back to ~0, smearing the whole texture across those faces. The usual
    fix is to DUPLICATE the seam vertices and give each copy its own u -- and
    that fix is unavailable here, twice over: `add_cloth_mesh` treats an unwelded
    duplicate as two unrelated particles (no stretch, no bending, the garment
    splits open along the seam), and OmCloth welds by position on load anyway, so
    the duplicate would not survive to the renderer.

    So the map is MIRRORED instead: u = |atan2(y, x)| / pi. That is continuous at
    theta = 0 AND at theta = +/-pi, i.e. it has no cut at all. The cost is that
    the left and right halves of the garment sample the same texels -- invisible
    for the tiling weave a fabric material wants, and the reason this is offered
    as a fabric parameterisation rather than as a place to put a chest print.
    v is (z - zmin) / height, which needs no trick: a shirt is open at both ends.

    Both axes are scaled to `tiles_per_m` so texel density is roughly isotropic
    and physically sized -- a fabric texture has a real-world scale, and getting
    it wrong is what makes a knit read as gift wrap.
    """
    import numpy as np
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    theta = np.abs(np.arctan2(y, x))                  # [0, pi], continuous at both ends
    # Half-circumference of the bbox-fitted ellipse, i.e. the surface distance u
    # actually has to cover; Ramanujan's approximation, so a flat garment (b -> 0)
    # does not report a wildly optimistic perimeter.
    a = float(max(np.abs(x).max(), 1e-6))
    b = float(max(np.abs(y).max(), 1e-6))
    h = ((a - b) ** 2) / ((a + b) ** 2)
    half_circumference = 0.5 * np.pi * (a + b) * (1.0 + 3.0 * h / (10.0 + np.sqrt(4.0 - 3.0 * h)))
    height = float(max(z.max() - z.min(), 1e-6))
    u = (theta / np.pi) * (half_circumference * tiles_per_m)
    v = ((z - z.min()) / height) * (height * tiles_per_m)
    return np.stack([u, v], axis=1)


def _arap_setup(points, tris, wmin=1e-3):
    """Per-triangle isometric flattening + cotangent weights, both in METRES."""
    import numpy as np
    p0, p1, p2 = points[tris[:, 0]], points[tris[:, 1]], points[tris[:, 2]]
    e1, e2 = p1 - p0, p2 - p0
    l1 = np.maximum(np.linalg.norm(e1, axis=1), 1e-15)
    b1 = e1 / l1[:, None]
    q = np.stack([np.zeros((len(tris), 2)),
                  np.stack([l1, np.zeros_like(l1)], axis=1),
                  np.stack([(e2 * b1).sum(1),
                            np.linalg.norm(np.cross(e1, e2), axis=1) / l1], axis=1)], axis=1)
    cot = np.empty((len(tris), 3))
    for t in range(3):
        u_, v_ = q[:, (t + 1) % 3] - q[:, t], q[:, (t + 2) % 3] - q[:, t]
        cross = np.abs(u_[:, 0] * v_[:, 1] - u_[:, 1] * v_[:, 0])
        cot[:, t] = (u_ * v_).sum(1) / np.maximum(cross, 1e-15)
    # Clamping keeps the assembled Laplacian positive semi-definite on the obtuse
    # triangles decimation leaves behind; without it the dense inverse below is
    # solving an indefinite system and the relaxation can walk away.
    return q, np.maximum(cot, wmin)


def arap_panel_uvs(points, tris, tiles_per_m, iters=40, seed=None, report=None):
    """A NEAR-ISOMETRIC map, found by relaxation, that folds into garment panels.

    ⚠ WHY NOT THE OBVIOUS FRONT/BACK PLANAR SPLIT. Projecting each panel along
    its own normal (u from x, v from z) is the intuitive fix and it MEASURED
    WORSE than the mirrored-cylindrical map it would replace: whole-mesh mip
    spread 1.58 vs 1.18, and the torso -- which is most of what you see -- went
    from 0.2% to 10.9% of its area more than a mip level off. The reason is in
    the geometry: this shirt's torso is a nearly ROUND tube (0.32 wide x 0.27
    deep), so a planar projection compresses it hard at the sides, and a
    cylindrical map is already close to right there. Measured per region, the
    old map's torso is fine (mip spread 0.52) and its SLEEVES are the defect
    (1.75, anisotropy up to 1805) -- they are tubes lying along X, so `u` from
    an angle about Z and `v` from z are BOTH nearly constant along a sleeve and
    the whole thing collapses to a blob of UV space. No single closed-form
    projection fixes a garment with two axes at right angles to each other.

    So this does not guess a projection -- it MINIMISES DISTORTION DIRECTLY.
    Standard local/global ARAP (Liu et al. 2008): per triangle, find the closest
    orthogonal 2x2 to the current map's Jacobian, then solve one cotangent-
    Laplacian Poisson system for the UVs that best match those targets, and
    repeat. The per-triangle flattening is in metres, so the relaxed map is in
    metres of SURFACE and `tiles_per_m` finally means exactly what it says.

    ⚠ REFLECTIONS ARE ALLOWED (`R = U @ Vt`, no det correction) AND THAT IS THE
    WHOLE TRICK. One uv per position is forced on us -- OmCloth welds by
    position and keeps the first uv it sees, so a UV seam cannot survive the
    load -- and a continuous non-degenerate map of a tube is topologically
    impossible. Letting a triangle match a MIRRORED target makes a folded,
    2-to-1 map cost nothing, which is exactly the freedom a garment needs and
    is invisible on a tiling weave.

    ⚠ THE FOLD WAS ALREADY IN THE RIGHT PLACE AND THIS DOES NOT MOVE IT -- worth
    stating plainly, because the tempting claim is that the relaxation discovers
    the side seam, and MEASURED it does not: it INHERITS it. `|atan2(y, x)|`
    already folds at theta = 0 and theta = pi, i.e. on the two sides, and the
    fold statistics come out the same before and after (85 fold vertices, 95% of
    them on the side/sleeve silhouette, median |y| = 0.006 m -- the side plane,
    running hem -> underarm -> sleeve cap -> shoulder -- and none on the chest).
    So SEAM PLACEMENT was never the defect. What the relaxation fixes is the
    METRIC between the seams, which is where every measured artefact lived.

    The seed is the mirrored-cylindrical map, so this is a refinement of it
    rather than a different idea, and the result is insensitive to the seed
    (an arc-length cylinder seed converges to the same numbers).
    """
    import numpy as np
    n = len(points)
    q, cot = _arap_setup(points, tris)
    ea = np.stack([tris[:, (t + 1) % 3] for t in range(3)], axis=1)
    eb = np.stack([tris[:, (t + 2) % 3] for t in range(3)], axis=1)
    dq = np.stack([q[:, (t + 1) % 3] - q[:, (t + 2) % 3] for t in range(3)], axis=1)

    M = np.zeros((n, n))
    for t in range(3):
        np.add.at(M, (ea[:, t], ea[:, t]), cot[:, t])
        np.add.at(M, (eb[:, t], eb[:, t]), cot[:, t])
        np.add.at(M, (ea[:, t], eb[:, t]), -cot[:, t])
        np.add.at(M, (eb[:, t], ea[:, t]), -cot[:, t])
    # ⚠ DO NOT KILL THE NULL SPACE BY PINNING A VERTEX. The Laplacian's null
    # space is the constants (one translation per axis) and the textbook fix is
    # to pin one vertex -- but that is only safe if the vertex is well connected.
    # MEASURED: pinning vertex 0, which on this shirt is a valence-3 boundary ear
    # at a cuff tip with only two incident triangles, held it still while the map
    # relaxed away from it and stretched those two triangles into a 1.13 m needle
    # (anisotropy 96 against a whole-mesh p95 of 1.4). This rank-1 term selects
    # the mean-zero solution instead and anchors no vertex at all -- exact here
    # because every RHS column already sums to zero by construction, each edge
    # contributing +w to one end and -w to the other.
    M += 1.0 / n
    Minv = np.linalg.inv(M)

    uv = (mirrored_cylindrical_uvs(points, 1.0) if seed is None else seed).astype(np.float64)
    for _ in range(iters):
        dx = np.stack([uv[ea[:, t]] - uv[eb[:, t]] for t in range(3)], axis=1)
        S = np.einsum("fti,ftj,ft->fij", dx, dq, cot)
        U, _s, Vt = np.linalg.svd(S)
        tgt = np.einsum("fij,ftj->fti", U @ Vt, dq)
        b = np.zeros((n, 2))
        for t in range(3):
            w = cot[:, t][:, None] * tgt[:, t]
            np.add.at(b, ea[:, t], w)
            np.add.at(b, eb[:, t], -w)
        new = Minv @ b
        delta = float(np.abs(new - uv).max())
        uv = new
        if delta < 1e-12:
            break

    # ⚠ GAUGE-FIX THE ROTATION, OR THE FILE NEVER STOPS CHANGING. The ARAP energy
    # is invariant to a global rotation of the uv plane, so that direction is a
    # free mode the relaxation drifts along for ever at no cost. MEASURED on the
    # shipped shirt: every distortion number is converged to four decimals by
    # sweep 10, but the UVs themselves keep creeping -- iterating 320 -> 640
    # sweeps still moved them 9.95e-07, i.e. right at the 6-decimal write
    # precision, so the OBJ's bytes depended on the sweep count and nothing else.
    # Rotating to a canonical frame collapses every drifted state onto the same
    # answer. It is also the chance to CHOOSE the orientation rather than inherit
    # whatever the seed happened to leave: align uv +v with world +Z, so the
    # weave's wale direction runs up the garment the way a real knit does.
    # (Only the ROTATION drifts. A global reflection is energy-free too, but it
    # is a discrete symmetry the iteration cannot wander into from a fixed seed.)
    zc = points[:, 2] - points[:, 2].mean()
    w = (zc[:, None] * (uv - uv.mean(axis=0))).sum(axis=0)
    nw = float(np.hypot(w[0], w[1]))
    if nw > 1e-12:
        nx, ny = w[0] / nw, w[1] / nw
        uv = uv @ np.array([[ny, nx], [-nx, ny]])          # sends w to +v
    # Anchor the translation on the MEAN, not on the extreme vertex. `uv -=
    # uv.min()` looks equivalent and is the same class of mistake as pinning v0
    # above: it hands the whole map's origin to whichever vertex happens to be
    # extreme, so that one vertex's residual drift shifts every other one. The
    # integer shift afterwards keeps the written values non-negative and is free
    # -- a whole-repeat offset is invisible to a tiling texture.
    uv -= uv.mean(axis=0)
    uv -= np.floor(uv.min(axis=0))
    if report is not None:
        f1, f2 = uv[tris[:, 1]] - uv[tris[:, 0]], uv[tris[:, 2]] - uv[tris[:, 0]]
        sgn = f1[:, 0] * f2[:, 1] - f1[:, 1] * f2[:, 0] > 0
        report.update({"iters": iters, "final_delta_m": round(delta, 12),
                       "mirrored_tris": int((~sgn).sum()), "upright_tris": int(sgn.sum())})
    return uv * tiles_per_m


def uv_distortion(points, tris, uvs):
    """Score a UV set for the two artefacts a fabric material actually shows.

    MOTTLING is texel-density variation: the renderer picks a mip level from the
    uv-area / world-area ratio, so a map whose density wanders makes light and
    dark patches that travel with the SURFACE instead of with the light. Scored
    as the area-weighted p05..p95 spread of that ratio in MIP LEVELS.

    STREAKING is anisotropy, the ratio of the map Jacobian's singular values. It
    matters twice over here because these worlds use a `normalMap`, and a normal
    map is sampled in a tangent frame DERIVED FROM THE UVs -- where the map is
    near-degenerate the tangent frame is near-singular and the perturbed normal
    points somewhere arbitrary, which is the same mottle by a second route.
    """
    import numpy as np
    p0, p1, p2 = points[tris[:, 0]], points[tris[:, 1]], points[tris[:, 2]]
    q0, q1, q2 = uvs[tris[:, 0]], uvs[tris[:, 1]], uvs[tris[:, 2]]
    e1, e2, f1, f2 = p1 - p0, p2 - p0, q1 - q0, q2 - q0
    nrm = np.cross(e1, e2)
    A3 = np.maximum(0.5 * np.linalg.norm(nrm, axis=1), 1e-12)
    A2 = 0.5 * np.abs(f1[:, 0] * f2[:, 1] - f1[:, 1] * f2[:, 0])

    b1 = e1 / np.maximum(np.linalg.norm(e1, axis=1, keepdims=True), 1e-15)
    b2 = np.cross(nrm / np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-15), b1)
    E = np.stack([np.stack([(e1 * b1).sum(1), (e2 * b1).sum(1)], axis=1),
                  np.stack([(e1 * b2).sum(1), (e2 * b2).sum(1)], axis=1)], axis=1)
    Fm = np.stack([np.stack([f1[:, 0], f2[:, 0]], axis=1),
                   np.stack([f1[:, 1], f2[:, 1]], axis=1)], axis=1)
    det = E[:, 0, 0] * E[:, 1, 1] - E[:, 0, 1] * E[:, 1, 0]
    ok = np.abs(det) > 1e-14
    Einv = np.zeros_like(E)
    Einv[ok, 0, 0], Einv[ok, 1, 1] = E[ok, 1, 1] / det[ok], E[ok, 0, 0] / det[ok]
    Einv[ok, 0, 1], Einv[ok, 1, 0] = -E[ok, 0, 1] / det[ok], -E[ok, 1, 0] / det[ok]
    J = np.zeros_like(Fm)
    J[ok] = Fm[ok] @ Einv[ok]
    sv = np.linalg.svd(J, compute_uv=False)
    aniso = sv[:, 0] / np.maximum(sv[:, 1], 1e-12)

    def wq(vals, qs):
        o = np.argsort(vals)
        c = np.cumsum(A3[o]) / A3.sum()
        return [float(np.interp(t, c, vals[o])) for t in qs]

    ld = np.log2(np.maximum(A2 / A3, 1e-12))
    p05, p50, p95 = wq(ld, [0.05, 0.50, 0.95])
    a50, a95 = wq(aniso, [0.50, 0.95])
    dev = np.abs(ld - p50)                       # log2 of an AREA: 2 == one mip
    return {"mip_spread_p05_p95": round((p95 - p05) / 2.0, 3),
            "area_pct_over_1_mip": round(float(100.0 * A3[dev > 2].sum() / A3.sum()), 2),
            "area_pct_over_2_mip": round(float(100.0 * A3[dev > 4].sum() / A3.sum()), 2),
            "aniso_p50": round(a50, 3), "aniso_p95": round(a95, 3),
            "aniso_max": round(float(aniso.max()), 2),
            "median_texels2_per_m2": round(2.0 ** p50, 1)}


def write_obj(path, points, tris, header_lines, uvs=None):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        for line in header_lines:
            fh.write("# %s\n" % line)
        # ⚠ NO `vn`, EVER. Cloth normals are recomputed per frame from the
        # simulated positions (OmCloth::uploadPositions), and a stored normal is
        # a per-vertex attribute that would make a loader split one position into
        # several vertices at every crease -- exactly the un-welding that costs a
        # garment its stretch and bending across that seam.
        #
        # `vt` used to be excluded for the same reason and is now WRITTEN, with
        # ONE uv per position and vt index == v index. That keeps the split from
        # ever arising (there is nothing to split on) while giving the fabric a
        # real parameterisation, which is what `Cloth { appearance PBRAppearance
        # { baseColorMap ... } }` samples. Pass --uv none to go back.
        #
        # ⚠ ONE UV PER POSITION IS A CONSTRAINT, NOT A STYLE, and it is why the
        # unwrap has to be a folded map rather than a cut-and-lay-flat atlas: a
        # UV seam written here does not survive the load. OmCloth welds by
        # POSITION and keeps the first uv it sees at each surviving one, so the
        # duplicate carrying the second uv is dropped and its triangles smear.
        # Anything that wants to unwrap this mesh must therefore stay CONTINUOUS
        # across the whole surface -- see arap_panel_uvs for how a fold buys that
        # without the degeneracy a continuous map would otherwise force.
        for p in points:
            fh.write("v %.6f %.6f %.6f\n" % (p[0], p[1], p[2]))
        if uvs is not None:
            for t in uvs:
                fh.write("vt %.6f %.6f\n" % (t[0], t[1]))
        for t in tris:
            if uvs is None:
                fh.write("f %d %d %d\n" % (t[0] + 1, t[1] + 1, t[2] + 1))   # OBJ is 1-based
            else:
                fh.write("f %d/%d %d/%d %d/%d\n"
                         % (t[0] + 1, t[0] + 1, t[1] + 1, t[1] + 1, t[2] + 1, t[2] + 1))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd", default=DEFAULT_USD)
    ap.add_argument("--prim", default=None, help="mesh prim path (default: first UsdGeom.Mesh)")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--target-verts", type=int, default=620,
                    help="approximate vertex count after decimation (0 = none)")
    ap.add_argument("--center-xy", action="store_true", default=True,
                    help="centre the bbox on local X/Y (default on)")
    ap.add_argument("--floor-z", action="store_true", default=True,
                    help="put the lowest vertex at local z = 0 (default on)")
    ap.add_argument("--uv", choices=("arap-panel", "mirrored-cylindrical", "none"),
                    default="arap-panel",
                    help="per-vertex UV set to bake into the OBJ (default: arap-panel, which is "
                         "mirrored-cylindrical relaxed to near-isometry -- see arap_panel_uvs)")
    ap.add_argument("--uv-arap-iters", type=int, default=40,
                    help="local/global relaxation sweeps for --uv arap-panel (default 40; every "
                         "distortion number on the shipped shirt is stable to four decimals by "
                         "sweep 10, and a fixed count is bit-reproducible run to run)")
    ap.add_argument("--uv-tiles-per-m", type=float, default=12.0,
                    help="fabric texture repeats per metre of surface (default 12, i.e. ~2.6 mm per knit "
                         "stitch on a 32-stitch texture). Under arap-panel this is exact; under "
                         "mirrored-cylindrical it is approximate and was measured 1.6x off on the torso")
    ap.add_argument("--json", action="store_true", help="emit the report as JSON")
    args = ap.parse_args()

    import numpy as np
    points, tris, info = load_usd_mesh(args.usd, args.prim)
    report = {"source": args.usd, "out": args.out}
    report.update(info)
    report["raw"] = {"verts": int(len(points)), "tris": int(len(tris))}

    points, tris, s0 = clean(points, tris)
    report["clean_source"] = s0
    report["clean_source"].update({"verts": int(len(points)), "tris": int(len(tris))})

    if args.target_verts and args.target_verts < len(points):
        points, tris, cell = decimate_to(points, tris, args.target_verts)
        report["decimation"] = {"method": "vertex clustering", "cell_m": round(cell, 6),
                                "verts": int(len(points)), "tris": int(len(tris))}

    nm_before, _ = nonmanifold_and_boundary(points, tris)
    points, tris, removed = repair_nonmanifold(points, tris)
    points, tris, s1 = clean(points, tris)          # the repair can orphan a vertex
    report["repair"] = {"nonmanifold_before": nm_before, "tris_removed": removed,
                        "orphans_after": s1["orphan_verts"],
                        "verts": int(len(points)), "tris": int(len(tris))}
    nm, bnd = nonmanifold_and_boundary(points, tris)
    report["nonmanifold_edges"] = nm
    report["boundary_edges"] = bnd

    if args.center_xy:
        c = 0.5 * (points.max(axis=0) + points.min(axis=0))
        points[:, 0] -= c[0]
        points[:, 1] -= c[1]
    if args.floor_z:
        points[:, 2] -= points[:, 2].min()

    report["bbox_min_m"] = [round(float(v), 6) for v in points.min(axis=0)]
    report["bbox_max_m"] = [round(float(v), 6) for v in points.max(axis=0)]
    e = np.linalg.norm(points[tris[:, 0]] - points[tris[:, 1]], axis=1)
    report["edge_len_m"] = {"min": round(float(e.min()), 6), "mean": round(float(e.mean()), 6),
                            "max": round(float(e.max()), 6)}
    a = 0.5 * np.linalg.norm(np.cross(points[tris[:, 1]] - points[tris[:, 0]],
                                      points[tris[:, 2]] - points[tris[:, 0]]), axis=1)
    report["area_m2"] = round(float(a.sum()), 6)

    uvs = None
    if args.uv != "none":
        uv_extra = {}
        if args.uv == "arap-panel":
            uvs = arap_panel_uvs(points, tris, args.uv_tiles_per_m,
                                 iters=args.uv_arap_iters, report=uv_extra)
        else:
            uvs = mirrored_cylindrical_uvs(points, args.uv_tiles_per_m)
        report["uv"] = {"mode": args.uv, "tiles_per_m": args.uv_tiles_per_m,
                        "u_range": [round(float(uvs[:, 0].min()), 4), round(float(uvs[:, 0].max()), 4)],
                        "v_range": [round(float(uvs[:, 1].min()), 4), round(float(uvs[:, 1].max()), 4)]}
        report["uv"].update(uv_extra)
        # Reported, not just computed: the whole point of the 2026-08-15 rework
        # was that "the shirt looks mottled" is a MEASURABLE property of the map,
        # so a regeneration that quietly makes it worse should be visible in
        # review rather than in a screenshot three worlds later.
        report["uv_distortion"] = uv_distortion(points, tris, uvs)
        report["uv_distortion"]["ideal_texels2_per_m2"] = round(args.uv_tiles_per_m ** 2, 1)

    write_obj(args.out, points, tris, [
        "T-shirt cloth mesh for OmniSim `Cloth { url ... }`.",
        "GENERATED by scripts/dev/usd_to_cloth_obj.py -- do not hand-edit.",
        "source: %s (%s)" % (args.usd, info["prim"]),
        "%s-up %g m/unit -> ENU Z-up metres; %d verts, %d tris, surface %.4f m^2"
        % (info["up_axis"], info["meters_per_unit"], len(points), len(tris), report["area_m2"]),
        "welded %d dup verts, dropped %d degenerate / %d duplicate tris, %d orphan verts"
        % (s0["welded_verts"], s0["degenerate_tris"], s0["duplicate_tris"], s0["orphan_verts"]),
        "non-manifold edges %d (must be 0), boundary edges %d (the hems)" % (nm, bnd),
        "local frame: bbox centred on XY, lowest vertex at z = 0",
        ("uv: %s, %g tiles/m, one uv per position (no seam split -- see the note in "
         "write_obj)" % (args.uv, args.uv_tiles_per_m)) if uvs is not None else "uv: none",
    ] + ([
        "uv distortion: mip spread %.2f, %.2f%% of area over 1 mip, anisotropy p50 %.2f "
        "p95 %.2f max %.2f, %.1f texels^2/m^2 (ideal %.1f)"
        % (report["uv_distortion"]["mip_spread_p05_p95"],
           report["uv_distortion"]["area_pct_over_1_mip"],
           report["uv_distortion"]["aniso_p50"], report["uv_distortion"]["aniso_p95"],
           report["uv_distortion"]["aniso_max"],
           report["uv_distortion"]["median_texels2_per_m2"],
           report["uv_distortion"]["ideal_texels2_per_m2"]),
    ] if uvs is not None else []), uvs=uvs)

    if args.json:
        print(json.dumps(report, indent=2))
    else:
        for k, v in report.items():
            print("%-16s %s" % (k, v))
    if nm:
        print("\nWARNING: %d non-manifold edges -- the decimation stitched the garment's "
              "front to its back. Raise --target-verts." % nm, file=sys.stderr)


if __name__ == "__main__":
    main()
