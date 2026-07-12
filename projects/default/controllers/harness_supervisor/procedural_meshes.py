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

"""procedural_meshes — Phase 14/15 runtime mesh generators.

Pure-Python helpers that emit VRML stanzas for procedurally-deformed
geometry. The damage tracker uses these to swap a part's Shape geometry
to a "crumpled" mesh on state transitions and to apply impact-localized
dents on contact events without pre-authored `.dae` assets.

Two models are supported:

1. One-shot generator (Phase 14a): `generate_crumpled_box(size,
   crumple, seed)` returns a fully-formed IndexedFaceSet stanza with
   per-vertex random displacement along outward normals. Stateless;
   deterministic per seed.

2. Persistent vertex buffer (Phase 15a): `make_baseline_box_buffer(size)`
   returns a flat list of (N+1)*(N+1)*6 × 3 floats representing each
   face's grid vertices. `apply_local_dent(...)` mutates the buffer
   to push vertices near a contact point along an inward normal.
   `vertex_buffer_to_ifs_stanza(...)` emits the current buffer as
   an IndexedFaceSet for `Field.importSFNodeFromString`. Same
   topology as the one-shot generator so the two layer cleanly.
"""

from __future__ import annotations

import random


def generate_crumpled_box(
    size: tuple[float, float, float],
    crumple: float = 0.0,
    seed: int = 0,
    subdivision: int = 4,
    edge_dampen: float = 0.3,
) -> str:
    """Emit a VRML `IndexedFaceSet` stanza for a subdivided rectangular
    box with per-vertex random displacement along its outward normal.

    Args:
        size: (sx, sy, sz) box extents in meters.
        crumple: 0.0 = perfect box, 0.5 = heavily dented. Magnitude of
            random displacement is `crumple * min(size) * 0.5`.
        seed: deterministic RNG seed; same seed → same mesh.
        subdivision: quads per face edge. 4 = 4×4 quads = 50 verts/face,
            32 triangles/face, 192 triangles total. Cheap.
        edge_dampen: multiplier on displacement at face boundaries.
            Lower = tighter seams between adjacent faces. Set to 0.0
            for pixel-perfect edges; 1.0 for uniform crumple.

    Returns:
        A VRML stanza string suitable for `Field.importSFNodeFromString`.
    """
    rng = random.Random(seed)
    n = max(1, int(subdivision))

    sx, sy, sz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    disp_mag = float(crumple) * min(size) * 0.5

    # Each face: (outward_normal, tangent, bitangent, half_extent_along_normal,
    #             half_extent_along_tangent, half_extent_along_bitangent).
    # Tangent×bitangent = outward_normal (right-hand rule), so the
    # quad winding (A, B, C) below is CCW from outside.
    faces_def = [
        ((1.0, 0.0, 0.0),  (0.0, 1.0, 0.0),  (0.0, 0.0, 1.0),  sx, sy, sz),  # +X
        ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0),  sx, sy, sz),  # -X
        ((0.0, 1.0, 0.0),  (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0),  sy, sx, sz),  # +Y
        ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0),  (0.0, 0.0, 1.0),  sy, sx, sz),  # -Y
        ((0.0, 0.0, 1.0),  (1.0, 0.0, 0.0),  (0.0, 1.0, 0.0),  sz, sx, sy),  # +Z
        ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0),  sz, sx, sy),  # -Z
    ]

    points: list[str] = []
    triangles: list[tuple[int, int, int]] = []

    for normal, tangent, bitangent, e_n, e_t, e_b in faces_def:
        base = len(points)
        # Generate (n+1) × (n+1) vertex grid for this face.
        for i in range(n + 1):
            for j in range(n + 1):
                u = (i / n) * 2.0 - 1.0  # in [-1, 1]
                v = (j / n) * 2.0 - 1.0
                px = e_n * normal[0] + u * e_t * tangent[0] + v * e_b * bitangent[0]
                py = e_n * normal[1] + u * e_t * tangent[1] + v * e_b * bitangent[1]
                pz = e_n * normal[2] + u * e_t * tangent[2] + v * e_b * bitangent[2]

                # Per-vertex displacement along outward normal. Boundary
                # verts (touching adjacent faces) get dampened so the
                # seam isn't a gaping crack — adjacent faces use their
                # own RNG draw, so without damping the edges fly apart.
                d = rng.uniform(-1.0, 1.0) * disp_mag
                if i in (0, n) or j in (0, n):
                    d *= edge_dampen
                px += d * normal[0]
                py += d * normal[1]
                pz += d * normal[2]
                points.append(f"{px:.4f} {py:.4f} {pz:.4f}")

        # Triangulate each quad in the (i, j) grid.
        for i in range(n):
            for j in range(n):
                a = base + i * (n + 1) + j
                b = base + (i + 1) * (n + 1) + j
                c = base + (i + 1) * (n + 1) + (j + 1)
                d = base + i * (n + 1) + (j + 1)
                # CCW winding from outside: (a, b, c) and (a, c, d).
                triangles.append((a, b, c))
                triangles.append((a, c, d))

    point_str = ", ".join(points)
    index_parts: list[str] = []
    for (a, b, c) in triangles:
        index_parts.extend((str(a), str(b), str(c), "-1"))
    index_str = " ".join(index_parts)

    return (
        "IndexedFaceSet { "
        f"coord Coordinate {{ point [ {point_str} ] }} "
        f"coordIndex [ {index_str} ] "
        "}"
    )


# ---------------------------------------------------------------------------
# Phase 15 — persistent vertex buffer + impact-localized deformation
# ---------------------------------------------------------------------------


def _box_face_table() -> list[tuple[tuple[float, float, float],
                                     tuple[float, float, float],
                                     tuple[float, float, float]]]:
    """The same 6-face definition the one-shot generator uses, returned
    as (normal, tangent, bitangent) triples. Tangent×bitangent =
    outward normal, so triangle winding stays CCW from outside.
    """
    return [
        ((1.0, 0.0, 0.0),  (0.0, 1.0, 0.0),  (0.0, 0.0, 1.0)),   # +X
        ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)),   # -X
        ((0.0, 1.0, 0.0),  (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),   # +Y
        ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0),  (0.0, 0.0, 1.0)),   # -Y
        ((0.0, 0.0, 1.0),  (1.0, 0.0, 0.0),  (0.0, 1.0, 0.0)),   # +Z
        ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),   # -Z
    ]


def make_baseline_box_buffer(size: tuple[float, float, float],
                             subdivision: int = 4) -> list[float]:
    """Return a flat list of (N+1)² × 6 × 3 floats — one (x, y, z)
    triple per vertex of an undeformed subdivided box of `size`.

    Layout matches the one-shot generator face order so vertex indices
    stay consistent for both rendering and dent-application.
    """
    n = max(1, int(subdivision))
    sx, sy, sz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    # Half extent the face spans along its normal axis
    extent_n = {(1, 0, 0): sx, (-1, 0, 0): sx,
                (0, 1, 0): sy, (0, -1, 0): sy,
                (0, 0, 1): sz, (0, 0, -1): sz}
    # Half extents along tangent / bitangent depend on which two axes
    # the face spans.
    def _half_along(unit):
        ax, ay, az = unit
        return abs(ax) * sx + abs(ay) * sy + abs(az) * sz

    out: list[float] = []
    for normal, tangent, bitangent in _box_face_table():
        e_n = extent_n[normal]
        e_t = _half_along(tangent)
        e_b = _half_along(bitangent)
        for i in range(n + 1):
            for j in range(n + 1):
                u = (i / n) * 2.0 - 1.0
                v = (j / n) * 2.0 - 1.0
                px = e_n * normal[0] + u * e_t * tangent[0] + v * e_b * bitangent[0]
                py = e_n * normal[1] + u * e_t * tangent[1] + v * e_b * bitangent[1]
                pz = e_n * normal[2] + u * e_t * tangent[2] + v * e_b * bitangent[2]
                out.append(px)
                out.append(py)
                out.append(pz)
    return out


def build_neighbor_table(subdivision: int = 4,
                          pin_boundary: bool = False) -> list[list[int]]:
    """Build a per-vertex adjacency table for the subdivided box layout
    used by `make_baseline_box_buffer`. Returns a list with one entry
    per vertex; each entry is a list of vertex indices that are direct
    grid neighbors of that vertex within the same face.

    Edges are *intra-face only* — vertices on opposing face boundaries
    aren't unified, matching the per-face independent grid the buffer
    uses. This is sufficient for single-hop spring coupling
    (Phase 16b); the occasional seam mismatch between adjacent faces
    is dampened by the edge_dampen factor in apply_uniform_random_dents.

    Total: 6 faces × (subdivision+1)² vertices, each with up to 4
    grid neighbors (fewer at face edges/corners).

    When `pin_boundary` is True, vertices on a face's boundary
    (i==0, i==n, j==0, j==n) get an empty neighbor list. Relaxation
    callers should use a boundary-pinned table; spring-coupling
    callers want the full table so dent rims at edges still propagate.
    """
    n = max(1, int(subdivision))
    verts_per_face = (n + 1) * (n + 1)
    neighbors: list[list[int]] = []
    for face_idx in range(6):
        base = face_idx * verts_per_face
        for i in range(n + 1):
            for j in range(n + 1):
                if pin_boundary and (i in (0, n) or j in (0, n)):
                    neighbors.append([])
                    continue
                nbrs: list[int] = []
                if i > 0:
                    nbrs.append(base + (i - 1) * (n + 1) + j)
                if i < n:
                    nbrs.append(base + (i + 1) * (n + 1) + j)
                if j > 0:
                    nbrs.append(base + i * (n + 1) + (j - 1))
                if j < n:
                    nbrs.append(base + i * (n + 1) + (j + 1))
                neighbors.append(nbrs)
    return neighbors


def vertex_buffer_to_ifs_stanza(vertices: list[float],
                                subdivision: int = 4) -> str:
    """Build a VRML IndexedFaceSet stanza from a vertex buffer
    produced by `make_baseline_box_buffer` (and possibly mutated by
    `apply_local_dent`). Topology is fixed: 6 faces × subdivision²
    quads × 2 triangles each.
    """
    n = max(1, int(subdivision))
    verts_per_face = (n + 1) * (n + 1)
    point_parts: list[str] = []
    for i in range(0, len(vertices), 3):
        point_parts.append(
            f"{vertices[i]:.4f} {vertices[i + 1]:.4f} {vertices[i + 2]:.4f}"
        )
    point_str = ", ".join(point_parts)

    index_parts: list[str] = []
    for face_idx in range(6):
        base = face_idx * verts_per_face
        for i in range(n):
            for j in range(n):
                a = base + i * (n + 1) + j
                b = base + (i + 1) * (n + 1) + j
                c = base + (i + 1) * (n + 1) + (j + 1)
                d = base + i * (n + 1) + (j + 1)
                index_parts.extend((str(a), str(b), str(c), "-1",
                                    str(a), str(c), str(d), "-1"))
    index_str = " ".join(index_parts)
    return (
        "IndexedFaceSet { "
        f"coord Coordinate {{ point [ {point_str} ] }} "
        f"coordIndex [ {index_str} ] "
        "}"
    )


def apply_local_dent(vertices: list[float],
                     local_point: tuple[float, float, float],
                     normal: tuple[float, float, float],
                     magnitude: float,
                     radius: float,
                     neighbors: list[list[int]] | None = None,
                     coupling: float = 0.0) -> int:
    """Push every vertex within `radius` of `local_point` along
    `normal` (in part-local frame) by `magnitude * falloff(d)` where
    `falloff(d) = max(0, 1 - d/radius)²` is a smooth radial dropoff.

    Mutates `vertices` in place. Returns the count of vertices moved.

    `normal` should be a unit vector pointing in the direction the
    surface should be pushed (typically *into* the body, i.e. from the
    contact point toward the part center).

    Phase 16b spring coupling:
        If `neighbors` is provided (one list of vertex indices per
        vertex) and `coupling > 0`, the dent first applies the radial
        falloff displacement, then propagates `coupling * push` to each
        neighbor of every directly-displaced vertex. This produces
        organic dimple shapes with rolled rims rather than sharp
        radial cones — the dent zone "drags" surrounding metal.
        Single propagation pass; deeper coupling can be obtained by
        running per-step relaxation (Phase 16d).
    """
    if magnitude == 0.0 or radius <= 0.0:
        return 0
    px, py, pz = local_point
    nx, ny, nz = normal
    r_sq = radius * radius
    moved = 0
    # First pass: radial falloff. Track which vertices got pushed and
    # by how much so the coupling pass below can propagate proportionally.
    primary_push: dict[int, float] = {}
    n = len(vertices)
    for i in range(0, n, 3):
        dx = vertices[i] - px
        dy = vertices[i + 1] - py
        dz = vertices[i + 2] - pz
        d_sq = dx * dx + dy * dy + dz * dz
        if d_sq >= r_sq:
            continue
        d = d_sq ** 0.5
        f = (1.0 - d / radius) ** 2
        push = magnitude * f
        vertices[i] += nx * push
        vertices[i + 1] += ny * push
        vertices[i + 2] += nz * push
        primary_push[i // 3] = push
        moved += 1

    # Second pass: spring coupling — neighbors of directly-displaced
    # vertices get a fraction of their neighbour's displacement. This
    # is single-hop only (no recursive propagation); Phase 16d's
    # per-step relaxation handles deeper smoothing.
    if neighbors is not None and coupling > 0.0 and primary_push:
        secondary: dict[int, float] = {}
        for v_idx, push in primary_push.items():
            for nbr in neighbors[v_idx]:
                if nbr in primary_push:
                    continue  # already pushed by primary pass
                secondary[nbr] = max(secondary.get(nbr, 0.0), push * coupling)
        for nbr, push in secondary.items():
            vi = nbr * 3
            vertices[vi] += nx * push
            vertices[vi + 1] += ny * push
            vertices[vi + 2] += nz * push
            moved += 1

    return moved


def fragment_ifs_stanza(vertices: list[float],
                         island: list[int],
                         subdivision: int = 4) -> tuple[str, list[float]]:
    """Phase 17c: build an IndexedFaceSet stanza for a fracture island
    of vertex indices. The fragment Solid will be spawned at the
    centroid of the island's vertices, so output points are relative
    to that centroid (range ±half-island-size).

    Returns (stanza, centroid_local) where centroid_local is the
    centroid of the island vertices in the source part's local frame.

    Triangulation: for each (i,j) of the island's face grid where all
    4 quad corners are island members, emit the two corner triangles.
    Topology preserved → the fragment looks like a torn-off piece of
    the chassis surface, not a generic Box.
    """
    n = max(1, int(subdivision))
    verts_per_face = (n + 1) * (n + 1)
    island_set = set(island)

    # Compute centroid in local frame.
    cx = cy = cz = 0.0
    for v_idx in island:
        vi = v_idx * 3
        cx += vertices[vi]
        cy += vertices[vi + 1]
        cz += vertices[vi + 2]
    inv = 1.0 / len(island)
    cx *= inv
    cy *= inv
    cz *= inv

    # Re-index island vertices to a dense local mesh: src global idx ->
    # dense fragment idx.
    dense_index: dict[int, int] = {}
    point_parts: list[str] = []
    for v_idx in island:
        dense_index[v_idx] = len(point_parts)
        vi = v_idx * 3
        # Position relative to the centroid so the spawned Solid's
        # translation field carries the world position.
        point_parts.append(
            f"{vertices[vi] - cx:.4f} "
            f"{vertices[vi + 1] - cy:.4f} "
            f"{vertices[vi + 2] - cz:.4f}"
        )

    # Group island indices by (face, i, j) so we can identify quads.
    triangles: list[tuple[int, int, int]] = []
    on_face: dict[int, dict[tuple[int, int], int]] = {}
    for v_idx in island:
        face_idx = v_idx // verts_per_face
        local = v_idx % verts_per_face
        i = local // (n + 1)
        j = local % (n + 1)
        on_face.setdefault(face_idx, {})[(i, j)] = v_idx

    for face_idx, ij_map in on_face.items():
        for (i, j), a_global in ij_map.items():
            b_global = ij_map.get((i + 1, j))
            c_global = ij_map.get((i + 1, j + 1))
            d_global = ij_map.get((i, j + 1))
            if b_global is None or c_global is None or d_global is None:
                continue  # quad not fully in the island
            a, b, c, d = (dense_index[a_global], dense_index[b_global],
                          dense_index[c_global], dense_index[d_global])
            triangles.append((a, b, c))
            triangles.append((a, c, d))

    # Edge case: an island that's just a strip (e.g. one row of
    # strained verts) won't produce any complete quads. Fall back to
    # a fan triangulation from the centroid so we still get something
    # visible.
    if not triangles and len(island) >= 3:
        # Add a synthetic centroid vertex (already at origin in
        # relative frame) and fan to it.
        center_idx = len(point_parts)
        point_parts.append("0.0000 0.0000 0.0000")
        for k in range(len(island)):
            kn = (k + 1) % len(island)
            triangles.append((center_idx, k, kn))

    point_str = ", ".join(point_parts)
    index_parts: list[str] = []
    for (a, b, c) in triangles:
        index_parts.extend((str(a), str(b), str(c), "-1"))
    index_str = " ".join(index_parts)
    # Webots' IndexedFaceSet doesn't accept the VRML `solid` field --
    # passing it floods the log with "Skipped unknown 'solid' field"
    # errors that under heavy regen rate (Phase 14b/15) overwhelmed
    # the parser and crashed the binary. Webots renders both sides
    # of an IFS by default, which is what we want for fragments
    # anyway, so the field is purely cosmetic.
    stanza = (
        "IndexedFaceSet { "
        f"coord Coordinate {{ point [ {point_str} ] }} "
        f"coordIndex [ {index_str} ] "
        "}"
    )
    return stanza, [cx, cy, cz]


def collapse_island_to_neighbors(vertices: list[float],
                                  island: list[int],
                                  neighbors: list[list[int]]) -> int:
    """Phase 17c chassis-hole: for each vertex in `island`, snap it to
    the centroid of its non-island neighbors. Effect: the chassis IFS
    keeps topology (no triangle removal needed) but the island region
    visually collapses to its boundary, reading as a torn opening.

    Returns the count of vertices snapped (only those with at least
    one non-island neighbor; isolated vertices are left in place).
    """
    if not island:
        return 0
    island_set = set(island)
    moved = 0
    for v_idx in island:
        sx = sy = sz = 0.0
        count = 0
        for nbr in neighbors[v_idx]:
            if nbr in island_set:
                continue
            ni = nbr * 3
            sx += vertices[ni]
            sy += vertices[ni + 1]
            sz += vertices[ni + 2]
            count += 1
        if count == 0:
            continue
        inv = 1.0 / count
        vi = v_idx * 3
        vertices[vi] = sx * inv
        vertices[vi + 1] = sy * inv
        vertices[vi + 2] = sz * inv
        moved += 1
    return moved


def find_fracture_islands(strained: list[int],
                          neighbors: list[list[int]],
                          min_size: int = 4) -> list[list[int]]:
    """Phase 17b: group strained vertex indices into connected islands
    via the shared neighbor table. Single-pass union-find. Returns a
    list of islands, where each island is a list of vertex indices,
    only including islands of size >= `min_size`.

    Used by Phase 17c to decide which strained regions should tear off
    as fragments. Smaller groups (a single vertex past yield, no
    neighbors over the threshold) are ignored — they're isolated stress
    points, not structural failures.
    """
    if not strained:
        return []
    strained_set = set(strained)
    parent: dict[int, int] = {v: v for v in strained}

    def find(v: int) -> int:
        while parent[v] != v:
            parent[v] = parent[parent[v]]
            v = parent[v]
        return v

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for v in strained:
        for nbr in neighbors[v]:
            if nbr in strained_set:
                union(v, nbr)

    groups: dict[int, list[int]] = {}
    for v in strained:
        r = find(v)
        groups.setdefault(r, []).append(v)

    return [members for members in groups.values() if len(members) >= min_size]


def find_strained_vertices(current: list[float],
                           baseline: list[float],
                           strain_threshold_m: float) -> tuple[list[int], float]:
    """Phase 17a: scan each vertex's distance from its baseline
    position. Return:
      - list of vertex indices whose distance exceeds
        `strain_threshold_m` (i.e. "yielded" past plastic limits)
      - the max distance observed across all vertices

    Pure measurement; no mutation. Cost is O(N) where N=verts/3.
    """
    if len(current) != len(baseline):
        return [], 0.0
    strained: list[int] = []
    max_d = 0.0
    n = len(current)
    t_sq = strain_threshold_m * strain_threshold_m
    for i in range(0, n, 3):
        dx = current[i] - baseline[i]
        dy = current[i + 1] - baseline[i + 1]
        dz = current[i + 2] - baseline[i + 2]
        d_sq = dx * dx + dy * dy + dz * dz
        if d_sq > max_d:
            max_d = d_sq
        if d_sq >= t_sq:
            strained.append(i // 3)
    return strained, max_d ** 0.5


def relax_vertices(vertices: list[float],
                   neighbors: list[list[int]],
                   rate: float = 0.05,
                   iterations: int = 2) -> float:
    """Phase 16d Laplacian smoothing pass. For each vertex, move it
    `rate` of the way toward the centroid of its grid neighbors. Run
    `iterations` passes (per call). Returns the maximum per-vertex
    delta seen during the call (so the caller can short-circuit when
    the mesh has converged).

    Mutates `vertices` in place. Vertex topology is preserved — we're
    only smoothing positions, not changing connectivity.
    """
    if rate <= 0.0 or iterations <= 0 or not neighbors:
        return 0.0
    max_delta_sq = 0.0
    n = len(vertices)
    # Use a single per-iteration scratch buffer for new positions so
    # the smoothing is "Jacobi" (each iteration uses the previous
    # iteration's positions, not in-flight updates). Keeps the pass
    # symmetric and stable.
    for _ in range(iterations):
        new_buf = vertices[:]  # shallow copy
        for v_idx in range(n // 3):
            nbrs = neighbors[v_idx]
            if not nbrs:
                continue
            sx = sy = sz = 0.0
            for nbr in nbrs:
                ni = nbr * 3
                sx += vertices[ni]
                sy += vertices[ni + 1]
                sz += vertices[ni + 2]
            inv = 1.0 / len(nbrs)
            cx = sx * inv
            cy = sy * inv
            cz = sz * inv
            vi = v_idx * 3
            dx = (cx - vertices[vi]) * rate
            dy = (cy - vertices[vi + 1]) * rate
            dz = (cz - vertices[vi + 2]) * rate
            new_buf[vi] = vertices[vi] + dx
            new_buf[vi + 1] = vertices[vi + 1] + dy
            new_buf[vi + 2] = vertices[vi + 2] + dz
            d_sq = dx * dx + dy * dy + dz * dz
            if d_sq > max_delta_sq:
                max_delta_sq = d_sq
        # Swap buffers for next iteration. Mutate `vertices` in place
        # so the caller sees the smoothed positions.
        for i in range(n):
            vertices[i] = new_buf[i]
    return max_delta_sq ** 0.5


def apply_uniform_random_dents(vertices: list[float],
                               magnitude: float,
                               seed: int,
                               edge_dampen: float = 0.3,
                               subdivision: int = 4) -> None:
    """Phase 14b layered behavior: instead of regenerating from
    scratch on each state transition (which would wipe out per-strike
    dents from Phase 15), add a uniform-random displacement along each
    vertex's outward normal *on top of* the current vertex buffer.

    `magnitude` is the *delta* to apply for this transition (e.g. the
    increase in CRUMPLE_PER_STATE between old and new state). Mutates
    in place.
    """
    if magnitude == 0.0:
        return
    rng = random.Random(seed)
    n = max(1, int(subdivision))
    verts_per_face = (n + 1) * (n + 1)
    face_idx = 0
    for normal, _t, _b in _box_face_table():
        base = face_idx * verts_per_face
        for i in range(n + 1):
            for j in range(n + 1):
                idx = base + i * (n + 1) + j
                vi = idx * 3
                d = rng.uniform(-1.0, 1.0) * magnitude
                if i in (0, n) or j in (0, n):
                    d *= edge_dampen
                vertices[vi] += d * normal[0]
                vertices[vi + 1] += d * normal[1]
                vertices[vi + 2] += d * normal[2]
        face_idx += 1
