"""Procedural rock mesh primitive tests."""

from __future__ import annotations

import math

import pytest

from omniworld.primitives.rock_mesh import (
    RockMesh,
    generate_rock,
    icosphere,
)


# -------------------------------------------------------------------
# Icosphere
# -------------------------------------------------------------------


def test_icosphere_vertex_counts():
    # 12 * 4^n + 2 * (2^n - 1) for n subdivisions — let's just pin the
    # expected counts we rely on elsewhere.
    verts, faces = icosphere(0)
    assert len(verts) == 12
    assert len(faces) == 20

    verts, faces = icosphere(1)
    assert len(verts) == 42
    assert len(faces) == 80

    verts, faces = icosphere(2)
    assert len(verts) == 162
    assert len(faces) == 320


def test_icosphere_vertices_on_unit_sphere():
    verts, _ = icosphere(2)
    for x, y, z in verts:
        r = math.sqrt(x * x + y * y + z * z)
        assert abs(r - 1.0) < 1e-9, f"vertex off unit sphere: r={r}"


def test_icosphere_rejects_bad_subdivisions():
    with pytest.raises(ValueError):
        icosphere(-1)
    with pytest.raises(ValueError):
        icosphere(5)


# -------------------------------------------------------------------
# generate_rock
# -------------------------------------------------------------------


def test_generate_rock_deterministic_same_seed():
    a = generate_rock(42, subdivisions=2)
    b = generate_rock(42, subdivisions=2)
    assert a == b


def test_generate_rock_different_seeds_differ():
    a = generate_rock(1, subdivisions=2)
    b = generate_rock(2, subdivisions=2)
    assert a.vertices != b.vertices


def test_generate_rock_has_correct_face_count():
    rock = generate_rock(0, subdivisions=2)
    # 320 faces * 4 (a, b, c, -1) = 1280 indices.
    assert len(rock.face_indices) == 320 * 4
    # Every 4th index is a -1 face terminator.
    for i in range(3, len(rock.face_indices), 4):
        assert rock.face_indices[i] == -1


def test_generate_rock_bounding_radius_reasonable():
    """With displacement=0.35 and elongation_max=0.5, the bounding
    radius should be somewhere around 1.0 * (1 + displacement) * (1 + elongation).
    We just check it's in a plausible range."""
    rock = generate_rock(7, subdivisions=2, displacement=0.35, elongation_max=0.5)
    assert 0.8 < rock.bounding_radius < 3.0


def test_generate_rock_zero_displacement_is_ellipsoid():
    """With displacement=0 and elongation_max=0, every vertex should be
    on the unit sphere."""
    rock = generate_rock(11, subdivisions=1,
                         displacement=0.0, elongation_max=0.0)
    for x, y, z in rock.vertices:
        r = math.sqrt(x * x + y * y + z * z)
        assert abs(r - 1.0) < 1e-9


def test_generate_rock_displacement_moves_vertices_off_sphere():
    baseline = generate_rock(3, subdivisions=1,
                             displacement=0.0, elongation_max=0.0)
    displaced = generate_rock(3, subdivisions=1,
                              displacement=0.3, elongation_max=0.0)
    # At least one vertex must differ materially.
    assert any(
        abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2]) > 0.01
        for a, b in zip(baseline.vertices, displaced.vertices)
    )


def test_generate_rock_face_indices_in_range():
    rock = generate_rock(0, subdivisions=2)
    vcount = len(rock.vertices)
    for idx in rock.face_indices:
        if idx == -1:
            continue
        assert 0 <= idx < vcount, f"face index {idx} out of range"


def test_generate_rock_rejects_bad_params():
    with pytest.raises(ValueError):
        generate_rock(0, displacement=-0.1)
    with pytest.raises(ValueError):
        generate_rock(0, elongation_max=1.5)
