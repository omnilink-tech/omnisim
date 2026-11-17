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

"""Procedural rock mesh generator.

Replaces the clean low-poly ``Rock.proto`` stand-in with a per-instance
displaced icosphere, so every placed rock has a unique silhouette
instead of N copies of the same mesh.

Pipeline:

1. Start with a regular icosahedron (12 vertices, 20 triangular faces).
2. Subdivide N times — each triangle becomes 4. ``subdivisions=2``
   produces 162 verts / 320 faces, which is enough silhouette detail
   to read as "real rock" at mid-range without exploding the WBT file
   size for hundreds of instances.
3. Displace each vertex outward along its own direction by a
   multi-octave 3D value-noise field keyed on the rock's ``seed``, so
   bumps look physically correlated across the surface.
4. Apply a per-axis elongation so rocks aren't all spherical.

All deterministic given ``seed``. Output is a tuple of vertex list
(unit-ish sphere with per-vertex radius) and an inclusive flat face
index list suitable for a OmniSim ``IndexedFaceSet.coordIndex``
(triangles, ``-1``-terminated).
"""

from __future__ import annotations

import hashlib
import math
import random
from dataclasses import dataclass


Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class RockMesh:
    """Triangulated closed mesh in a rock-sized bounding box."""

    vertices: tuple[Vec3, ...]
    face_indices: tuple[int, ...]    # flat, -1-terminated per face
    bounding_radius: float            # for cheap boundingObject spheres


# ---------------------------------------------------------------------------
# Icosahedron base
# ---------------------------------------------------------------------------


def _normalise(v: Vec3) -> Vec3:
    x, y, z = v
    length = math.sqrt(x * x + y * y + z * z)
    if length == 0.0:
        return (0.0, 0.0, 0.0)
    return (x / length, y / length, z / length)


def _icosahedron() -> tuple[list[Vec3], list[tuple[int, int, int]]]:
    phi = (1.0 + math.sqrt(5.0)) / 2.0
    raw_verts = [
        (-1,  phi,  0), ( 1,  phi,  0), (-1, -phi,  0), ( 1, -phi,  0),
        ( 0, -1,  phi), ( 0,  1,  phi), ( 0, -1, -phi), ( 0,  1, -phi),
        ( phi,  0, -1), ( phi,  0,  1), (-phi,  0, -1), (-phi,  0,  1),
    ]
    verts = [_normalise((float(x), float(y), float(z))) for x, y, z in raw_verts]
    faces = [
        (0, 11,  5), (0,  5,  1), (0,  1,  7), (0,  7, 10), (0, 10, 11),
        (1,  5,  9), (5, 11,  4), (11, 10, 2), (10, 7,  6), (7,  1,  8),
        (3,  9,  4), (3,  4,  2), (3,  2,  6), (3,  6,  8), (3,  8,  9),
        (4,  9,  5), (2,  4, 11), (6,  2, 10), (8,  6,  7), (9,  8,  1),
    ]
    return verts, faces


def _subdivide(
    verts: list[Vec3], faces: list[tuple[int, int, int]]
) -> tuple[list[Vec3], list[tuple[int, int, int]]]:
    """One Loop-style subdivision step that projects midpoints back to
    the unit sphere. Returns new vertex + face lists."""
    midpoint_cache: dict[tuple[int, int], int] = {}
    new_verts = list(verts)

    def midpoint(a: int, b: int) -> int:
        key = (a, b) if a < b else (b, a)
        if key in midpoint_cache:
            return midpoint_cache[key]
        ax, ay, az = verts[a]
        bx, by, bz = verts[b]
        m = _normalise((
            (ax + bx) * 0.5,
            (ay + by) * 0.5,
            (az + bz) * 0.5,
        ))
        idx = len(new_verts)
        new_verts.append(m)
        midpoint_cache[key] = idx
        return idx

    new_faces: list[tuple[int, int, int]] = []
    for a, b, c in faces:
        ab = midpoint(a, b)
        bc = midpoint(b, c)
        ca = midpoint(c, a)
        new_faces.extend([
            (a,  ab, ca),
            (b,  bc, ab),
            (c,  ca, bc),
            (ab, bc, ca),
        ])
    return new_verts, new_faces


def icosphere(subdivisions: int = 2) -> tuple[list[Vec3], list[tuple[int, int, int]]]:
    """Unit icosphere at the requested subdivision level.

    - 0: 12 verts, 20 faces
    - 1: 42 verts, 80 faces
    - 2: 162 verts, 320 faces
    - 3: 642 verts, 1280 faces

    Anything past 3 is probably overkill for a rock.
    """
    if subdivisions < 0 or subdivisions > 4:
        raise ValueError("icosphere: subdivisions must be in [0, 4]")
    verts, faces = _icosahedron()
    for _ in range(subdivisions):
        verts, faces = _subdivide(verts, faces)
    return verts, faces


# ---------------------------------------------------------------------------
# 3D value noise (for vertex displacement)
# ---------------------------------------------------------------------------


def _hash_value_3d(seed: int, i: int, j: int, k: int, salt: int = 0) -> float:
    """Deterministic uniform sample in [0, 1) keyed by (seed, i, j, k, salt)."""
    key = (seed & 0xFFFFFFFFFFFFFFFF).to_bytes(8, "little", signed=False)
    msg = (
        i.to_bytes(8, "little", signed=True)
        + j.to_bytes(8, "little", signed=True)
        + k.to_bytes(8, "little", signed=True)
        + salt.to_bytes(4, "little", signed=False)
    )
    digest = hashlib.blake2b(msg, digest_size=8, key=key).digest()
    n = int.from_bytes(digest, "little", signed=False)
    return (n >> 11) * (1.0 / (1 << 53))


def _cell_value_3d(seed: int, i: int, j: int, k: int, salt: int = 0) -> float:
    """Signed noise value in [-1, 1)."""
    return _hash_value_3d(seed, i, j, k, salt) * 2.0 - 1.0


def _smoothstep(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def _value_noise_3d(x: float, y: float, z: float, seed: int, salt: int = 0) -> float:
    """Trilinear-interpolated 3D value noise."""
    i0 = math.floor(x)
    j0 = math.floor(y)
    k0 = math.floor(z)
    tx = _smoothstep(x - i0)
    ty = _smoothstep(y - j0)
    tz = _smoothstep(z - k0)

    # Eight corner samples.
    c000 = _cell_value_3d(seed, i0,     j0,     k0,     salt)
    c100 = _cell_value_3d(seed, i0 + 1, j0,     k0,     salt)
    c010 = _cell_value_3d(seed, i0,     j0 + 1, k0,     salt)
    c110 = _cell_value_3d(seed, i0 + 1, j0 + 1, k0,     salt)
    c001 = _cell_value_3d(seed, i0,     j0,     k0 + 1, salt)
    c101 = _cell_value_3d(seed, i0 + 1, j0,     k0 + 1, salt)
    c011 = _cell_value_3d(seed, i0,     j0 + 1, k0 + 1, salt)
    c111 = _cell_value_3d(seed, i0 + 1, j0 + 1, k0 + 1, salt)

    # Lerp along x.
    a00 = c000 * (1.0 - tx) + c100 * tx
    a10 = c010 * (1.0 - tx) + c110 * tx
    a01 = c001 * (1.0 - tx) + c101 * tx
    a11 = c011 * (1.0 - tx) + c111 * tx
    # Lerp along y.
    b0 = a00 * (1.0 - ty) + a10 * ty
    b1 = a01 * (1.0 - ty) + a11 * ty
    # Lerp along z.
    return b0 * (1.0 - tz) + b1 * tz


def _fbm_3d(
    x: float, y: float, z: float,
    seed: int,
    *,
    octaves: int = 3,
    lacunarity: float = 2.0,
    gain: float = 0.5,
) -> float:
    """Multi-octave 3D value noise in approximately [-1, 1]."""
    total = 0.0
    norm = 0.0
    amplitude = 1.0
    frequency = 1.0
    for octave in range(octaves):
        total += amplitude * _value_noise_3d(
            x * frequency, y * frequency, z * frequency, seed, salt=octave,
        )
        norm += amplitude
        amplitude *= gain
        frequency *= lacunarity
    return total / norm if norm != 0.0 else 0.0


# ---------------------------------------------------------------------------
# Rock construction
# ---------------------------------------------------------------------------


def _deterministic_rng(seed: int, *name_parts: str) -> random.Random:
    """Local RNG seeded by (seed, name_parts). Isolates mesh shaping
    decisions (elongation, base frequency) from per-vertex displacement
    so changing one does not perturb the other."""
    key = (seed & 0xFFFFFFFFFFFFFFFF).to_bytes(8, "little", signed=False)
    msg = ".".join(name_parts).encode("utf-8")
    digest = hashlib.blake2b(msg, digest_size=8, key=key).digest()
    return random.Random(int.from_bytes(digest, "little", signed=False))


def generate_rock(
    seed: int,
    *,
    subdivisions: int = 2,
    displacement: float = 0.35,
    frequency: float | None = None,
    elongation_max: float = 0.5,
    octaves: int = 3,
) -> RockMesh:
    """Produce one unique rock mesh.

    Arguments:
        seed: controls every random choice. Two calls with the same
            seed produce byte-identical meshes.
        subdivisions: icosphere subdivision depth (0..4).
        displacement: maximum outward displacement as a fraction of
            radius. 0 gives a sphere; 0.35 gives a recognisable rock.
        frequency: noise frequency. Higher = more, finer bumps. When
            None, picks a seed-dependent value in [1.8, 3.5].
        elongation_max: maximum per-axis stretch/squash as a fraction
            of radius. 0 gives a sphere; 0.5 gives visibly elongated
            rocks.
        octaves: number of fbm octaves for displacement.
    """
    if displacement < 0.0:
        raise ValueError("generate_rock: displacement must be >= 0")
    if elongation_max < 0.0 or elongation_max > 1.0:
        raise ValueError("generate_rock: elongation_max must be in [0, 1]")

    verts_unit, faces = icosphere(subdivisions)

    shape_rng = _deterministic_rng(seed, "rock", "shape")
    if frequency is None:
        frequency = shape_rng.uniform(1.8, 3.5)
    # Per-axis scale. Pick three values around 1 with spread controlled
    # by elongation_max.
    ex = 1.0 + shape_rng.uniform(-elongation_max, elongation_max)
    ey = 1.0 + shape_rng.uniform(-elongation_max, elongation_max)
    ez = 1.0 + shape_rng.uniform(-elongation_max * 0.5, elongation_max * 0.2)
    # Clamp z so rocks don't turn into pancakes on the top.
    ex = max(0.5, ex)
    ey = max(0.5, ey)
    ez = max(0.5, ez)

    # Vertex displacement. Sample fbm at vertex position * frequency,
    # then scale outward + apply axis elongation.
    displaced: list[Vec3] = []
    max_r = 0.0
    for vx, vy, vz in verts_unit:
        n = _fbm_3d(vx * frequency, vy * frequency, vz * frequency,
                    seed, octaves=octaves)
        # n in ~[-1, 1]; map to [-displacement, +displacement].
        d = 1.0 + n * displacement
        x = vx * d * ex
        y = vy * d * ey
        z = vz * d * ez
        displaced.append((x, y, z))
        r = math.sqrt(x * x + y * y + z * z)
        if r > max_r:
            max_r = r

    # Flat face index list, -1-terminated per face.
    face_indices: list[int] = []
    for a, b, c in faces:
        face_indices.extend([a, b, c, -1])

    return RockMesh(
        vertices=tuple(displaced),
        face_indices=tuple(face_indices),
        bounding_radius=max_r,
    )


__all__ = ["RockMesh", "generate_rock", "icosphere"]
