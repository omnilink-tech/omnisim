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
"""Author the generic mesh fixtures under ``projects/default/worlds/meshes``.

WHY THIS SCRIPT EXISTS
----------------------
These meshes are demonstration and test payloads: something for the ``Mesh``
node to load in the geometry sample world, something for a ``DistanceSensor``
rake to hit and miss, something for a ``Pen`` to draw on. Nothing about those
jobs requires a *particular* shape -- but everything about shipping in an
Apache-2.0 tree requires knowing where the coordinates came from.

The file this replaced (``suzanne.obj``) was a verbatim export of Blender's
built-in monkey primitive, whose coordinate table is a data array inside a
``GPL-2.0-or-later`` source file. Rather than ship geometry whose terms could
not be established, the geometry is now *generated here*: the script is
committed, so the asset is reproducible from source and provably this
project's own work.

WHAT IT EMITS
-------------
A (p, q) torus knot swept with a circular tube. Chosen deliberately:

  * closed and watertight, so it behaves like a solid to sensors;
  * strongly non-convex with real see-through gaps, so a ray rake genuinely
    produces both hits and misses (a convex blob would make that test vacuous);
  * exact analytic normals and a seamless UV chart, so textured and
    Pen-painted materials work the way the old mesh's did;
  * quad topology, matching what the sample worlds previously loaded.

Determinism: fixed-count arithmetic over stdlib ``math``, rounded to a fixed
precision before formatting, so every run is byte-identical.

Usage::

    python scripts/dev/gen_mesh_fixtures.py            # write the fixtures
    python scripts/dev/gen_mesh_fixtures.py --check    # verify on-disk match
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
MESH_DIR = REPO / 'projects' / 'default' / 'worlds' / 'meshes'

# Coordinates are rounded to this many decimals before writing, which is what
# makes the output byte-stable across platforms and Python versions.
PRECISION = 6


def _round(value):
    """Round to PRECISION, collapsing negative zero so the text form is stable."""
    result = round(value, PRECISION)
    return 0.0 if result == 0.0 else result


def _normalise(vector):
    length = math.sqrt(sum(component * component for component in vector))
    if length == 0.0:
        return (0.0, 0.0, 0.0)
    return tuple(component / length for component in vector)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _knot_point(t, major_radius, wave, depth, p, q):
    """A point on the (p, q) torus knot centre curve at parameter ``t``."""
    radius = major_radius + wave * math.cos(q * t)
    return (radius * math.cos(p * t),
            radius * math.sin(p * t),
            depth * math.sin(q * t))


def _centre_curve(segments, major_radius, wave, depth, p, q):
    step = 2.0 * math.pi / segments
    return [_knot_point(index * step, major_radius, wave, depth, p, q)
            for index in range(segments)]


def _parallel_transport_frames(curve):
    """A rotation-minimising frame per curve sample.

    Frenet frames flip at inflection points, which would twist the tube and
    tear the UV chart. Parallel transport carries one arbitrary starting
    normal along the curve instead, so the frame varies smoothly.
    """
    count = len(curve)
    tangents = []
    for index in range(count):
        after = curve[(index + 1) % count]
        before = curve[index - 1]
        tangents.append(_normalise((after[0] - before[0],
                                    after[1] - before[1],
                                    after[2] - before[2])))

    # Seed a normal that is not parallel to the first tangent.
    seed = (0.0, 0.0, 1.0)
    if abs(tangents[0][2]) > 0.9:
        seed = (1.0, 0.0, 0.0)
    normal = _normalise(_cross(tangents[0], seed))

    frames = []
    for index in range(count):
        tangent = tangents[index]
        # Re-project the carried normal into the plane perpendicular to the
        # current tangent -- that is the parallel-transport step.
        dot = sum(normal[axis] * tangent[axis] for axis in range(3))
        normal = _normalise((normal[0] - dot * tangent[0],
                             normal[1] - dot * tangent[1],
                             normal[2] - dot * tangent[2]))
        binormal = _normalise(_cross(tangent, normal))
        frames.append((normal, binormal))
    return frames


def _rotate_x90(vector):
    """Rotate +90 degrees about X: (x, y, z) -> (x, -z, y).

    Applied to the finished knot so its ring plane is X-Z rather than X-Y.
    That is not cosmetic: it is what puts material where the DistanceSensor
    rake in tests/api/worlds/distance_sensor_infra-red_vs_mesh samples, so
    that test keeps producing both hits and misses instead of degenerating
    into "every ray passes through the hole".
    """
    return (vector[0], -vector[2], vector[1])


def build_torus_knot(segments=96, sides=16, major_radius=1.0, wave=0.32,
                     depth=0.44, tube=0.26, p=2, q=3, centre=(0.0, 0.0, 0.0),
                     scale=1.0, upright=True):
    """Return (positions, uvs, normals, faces) for a tubed torus knot.

    ``faces`` are quads of (position, uv, normal) index triples, 1-based,
    ready for OBJ. Positions and normals are shared around the seam; UVs are
    duplicated across it so the chart does not wrap.
    """
    curve = _centre_curve(segments, major_radius, wave, depth, p, q)
    frames = _parallel_transport_frames(curve)

    positions, normals = [], []
    for index in range(segments):
        origin = curve[index]
        normal_axis, binormal_axis = frames[index]
        for side in range(sides):
            angle = 2.0 * math.pi * side / sides
            cos_a, sin_a = math.cos(angle), math.sin(angle)
            radial = (cos_a * normal_axis[0] + sin_a * binormal_axis[0],
                      cos_a * normal_axis[1] + sin_a * binormal_axis[1],
                      cos_a * normal_axis[2] + sin_a * binormal_axis[2])
            positions.append((origin[0] + tube * radial[0],
                              origin[1] + tube * radial[1],
                              origin[2] + tube * radial[2]))
            normals.append(radial)

    # Scale about the origin, optionally stand the ring up, then recentre the
    # bounding box on `centre` so worlds framed for the previous mesh still
    # frame this one.
    positions = [tuple(component * scale for component in point) for point in positions]
    if upright:
        positions = [_rotate_x90(point) for point in positions]
        normals = [_rotate_x90(normal) for normal in normals]
    lows = [min(point[axis] for point in positions) for axis in range(3)]
    highs = [max(point[axis] for point in positions) for axis in range(3)]
    shift = [centre[axis] - 0.5 * (lows[axis] + highs[axis]) for axis in range(3)]
    positions = [(point[0] + shift[0], point[1] + shift[1], point[2] + shift[2])
                 for point in positions]

    # Seam-duplicated UV grid: (segments + 1) x (sides + 1).
    uvs = []
    for index in range(segments + 1):
        for side in range(sides + 1):
            uvs.append((index / segments, side / sides))

    def position_index(index, side):
        return (index % segments) * sides + (side % sides) + 1

    def uv_index(index, side):
        return index * (sides + 1) + side + 1

    faces = []
    for index in range(segments):
        for side in range(sides):
            corners = ((index, side), (index + 1, side),
                       (index + 1, side + 1), (index, side + 1))
            faces.append([(position_index(i, s), uv_index(i, s), position_index(i, s))
                          for i, s in corners])
    return positions, uvs, normals, faces


def render_obj(positions, uvs, normals, faces, title, description):
    lines = ['# %s' % title]
    lines.extend('# %s' % line for line in description)
    lines.append('o %s' % title)
    for point in positions:
        lines.append('v %f %f %f' % tuple(_round(c) for c in point))
    for uv in uvs:
        lines.append('vt %f %f' % tuple(_round(c) for c in uv))
    for normal in normals:
        lines.append('vn %f %f %f' % tuple(_round(c) for c in normal))
    lines.append('s off')
    for face in faces:
        lines.append('f ' + ' '.join('%d/%d/%d' % corner for corner in face))
    return '\n'.join(lines) + '\n'


# The previous occupant of this slot spanned roughly 2.73 x 1.97 x 1.70 with
# its bounding box centred at (-0.28, 0.83, 0.02). Matching that envelope means
# the sample and test worlds keep their existing camera framing, poses and
# scales, so nothing but the file name has to change in them.
FIXTURES = {
    'torus_knot.obj': dict(
        title='TorusKnot',
        description=(
            'Generated by scripts/dev/gen_mesh_fixtures.py -- OmniLink own work,',
            'Apache-2.0 with the rest of this repository. Regenerate with:',
            '  python scripts/dev/gen_mesh_fixtures.py',
        ),
        # tube/scale/orientation are not free choices: they were selected so
        # the DistanceSensor rake test keeps a non-degenerate hit/miss pattern
        # with a wide margin either side of the detection threshold (measured
        # hit distances 0.002-0.028 m against a 0.1 m threshold, miss distances
        # 0.198-0.697 m). See the note on _rotate_x90.
        params=dict(segments=96, sides=16, p=2, q=3, major_radius=1.0,
                    wave=0.32, depth=0.44, tube=0.38, scale=0.90,
                    centre=(-0.28, 0.83, 0.02)),
    ),
}


def emit(name):
    spec = FIXTURES[name]
    positions, uvs, normals, faces = build_torus_knot(**spec['params'])
    return render_obj(positions, uvs, normals, faces, spec['title'], spec['description'])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--check', action='store_true',
                        help='verify the files on disk match what this script emits')
    args = parser.parse_args()

    failures = 0
    for name in sorted(FIXTURES):
        target = MESH_DIR / name
        text = emit(name)
        if args.check:
            on_disk = target.read_text(encoding='utf-8') if target.exists() else None
            if on_disk == text:
                print('OK       %s' % target.relative_to(REPO))
            else:
                print('MISMATCH %s' % target.relative_to(REPO))
                failures += 1
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            with open(target, 'w', encoding='utf-8', newline='\n') as handle:
                handle.write(text)
            print('wrote    %s  (%d vertices, %d faces)'
                  % (target.relative_to(REPO), text.count('\nv '), text.count('\nf ')))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
