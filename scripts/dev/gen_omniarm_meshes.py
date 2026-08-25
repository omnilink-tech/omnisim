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
"""Author OmniArm's visual shells as smooth surfaces of revolution.

The arm's first pass was bare URDF cylinders: hard rims, no taper, no fillets.
Correct, provably ours, and it read as a toy. A real cobot's shape is almost
entirely a SURFACE OF REVOLUTION -- tube, fillet, shoulder, collar, flange --
so that is what this emits: profiles revolved about an axis, with analytic
per-vertex normals, so shading is smooth where the surface is smooth and
creased only where it genuinely creases.

Everything here is generated. No mesh, texture or CAD file from any third party
is involved, and the output is reproducible: same script, same bytes. That is
the whole point -- the arm has to look professional AND stay provably ours.

    python scripts/dev/gen_omniarm_meshes.py            # write
    python scripts/dev/gen_omniarm_meshes.py --check    # CI drift gate

WARNING: VISUAL ONLY. This never touches joints, limits, inertials or collision
geometry. Those stay exactly as they are, which is what keeps worlds,
controllers and trained policies valid across the change.
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[2]
OUT_DIR = REPO / "projects/robots/omnisim/omniarm6/meshes"
SEGMENTS = 56          # radial subdivisions; 56 keeps a 90 mm tube visually round
CREASE_COS = math.cos(math.radians(35.0))


# --- profile helpers -----------------------------------------------------
# A profile is a list of (r, z) in the revolve frame: r >= 0, z along the axis.

def arc(cr, cz, rad, a0, a1, n=10):
    """Points along a circular arc; angles in degrees measured from +r."""
    out = []
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n)
        out.append((cr + rad * math.cos(a), cz + rad * math.sin(a)))
    return out


def tube(r, z0, z1, fillet_bottom=0.0, fillet_top=0.0):
    """A cylinder z0..z1 with rounded rims, closed by flat caps."""
    p = [(0.0, z0)]
    fb, ft = fillet_bottom, fillet_top
    p.append((max(r - fb, 0.0), z0))
    if fb > 0:
        p += arc(r - fb, z0 + fb, fb, -90, 0, 8)[1:]
    else:
        p.append((r, z0))
    p.append((r, z1 - ft))
    if ft > 0:
        p += arc(r - ft, z1 - ft, ft, 0, 90, 8)[1:]
    else:
        p.append((r, z1))
    p.append((max(r - ft, 0.0), z1))
    p.append((0.0, z1))
    return p


def taper(r0, r1, z0, z1, fillet=0.0):
    """A conical section -- the thing a stack of equal cylinders lacks."""
    p = [(0.0, z0), (r0, z0), (r1, z1 - fillet)]
    if fillet > 0:
        p += arc(r1 - fillet, z1 - fillet, fillet, 0, 90, 8)[1:]
        p.append((max(r1 - fillet, 0.0), z1))
    else:
        p.append((r1, z1))
    p.append((0.0, z1))
    return p


# --- revolve -------------------------------------------------------------

def revolve(profile, axis="z", offset=(0.0, 0.0, 0.0), segs=SEGMENTS):
    """Surface of revolution with analytic normals; creases split vertices."""
    n = len(profile)
    tang = []
    for i in range(n):
        a = profile[max(i - 1, 0)]
        b = profile[min(i + 1, n - 1)]
        dr, dz = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dr, dz) or 1.0
        tang.append((dr / L, dz / L))

    creased = [False] * n
    for i in range(1, n - 1):
        a, b, c = profile[i - 1], profile[i], profile[i + 1]
        t0 = (b[0] - a[0], b[1] - a[1])
        t1 = (c[0] - b[0], c[1] - b[1])
        l0 = math.hypot(*t0) or 1.0
        l1 = math.hypot(*t1) or 1.0
        if (t0[0] * t1[0] + t0[1] * t1[1]) / (l0 * l1) < CREASE_COS:
            creased[i] = True

    rows = []          # (r, z, nr, nz)
    for i, (r, z) in enumerate(profile):
        if creased[i]:
            a, b, c = profile[i - 1], profile[i], profile[i + 1]
            for t in ((b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])):
                L = math.hypot(*t) or 1.0
                rows.append((r, z, t[1] / L, -t[0] / L))
        else:
            tr, tz = tang[i]
            rows.append((r, z, tz, -tr))

    verts, norms, faces = [], [], []
    for (r, z, nr, nz) in rows:
        for s in range(segs):
            a = 2.0 * math.pi * s / segs
            ca, sa = math.cos(a), math.sin(a)
            p = (r * ca, r * sa, z)
            q = (nr * ca, nr * sa, nz)
            if axis == "y":
                p = (p[0], -p[2], p[1])
                q = (q[0], -q[2], q[1])
            elif axis == "x":
                p = (p[2], p[0], p[1])
                q = (q[2], q[0], q[1])
            verts.append((p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]))
            L = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2) or 1.0
            norms.append((q[0] / L, q[1] / L, q[2] / L))

    for i in range(len(rows) - 1):
        for s in range(segs):
            s2 = (s + 1) % segs
            faces.append((i * segs + s, i * segs + s2,
                          (i + 1) * segs + s2, (i + 1) * segs + s))
    return verts, norms, faces


def build_obj(parts):
    """parts: [(group_name, (verts, norms, faces))] -> one OBJ text."""
    V, N, F, base, groups = [], [], [], 0, []
    for name, (v, n, f) in parts:
        groups.append((name, len(F), len(f)))
        V += v
        N += n
        F += [tuple(i + base for i in q) for q in f]
        base += len(v)
    out = ["# Generated by scripts/dev/gen_omniarm_meshes.py -- do not hand-edit.",
           "# Surfaces of revolution authored in this repository. Apache-2.0, (c) OmniLink.",
           "# %d vertices, %d quads, %d radial segments." % (len(V), len(F), SEGMENTS)]
    for x, y, z in V:
        out.append("v %.6f %.6f %.6f" % (x, y, z))
    for x, y, z in N:
        out.append("vn %.6f %.6f %.6f" % (x, y, z))
    for name, start, count in groups:
        out.append("g " + name)
        for q in F[start:start + count]:
            out.append("f " + " ".join("%d//%d" % (i + 1, i + 1) for i in q))
    return "\n".join(out) + "\n"


# --- the arm -------------------------------------------------------------
# One OBJ per (link, material) so the palette survives: SHELL is the cream
# body, JOINT the dark housings, ACCENT the mimosa collars. The geometry
# follows the SAME skeleton the URDF already declares; nothing here moves a
# joint or changes a collider.

def omniarm6():
    return {
        # link0 -- pedestal: chamfered foot tapering into the shoulder column
        "link0_shell": [
            ("foot", revolve(tube(0.115, 0.000, 0.030, 0.006, 0.010))),
            ("body", revolve(taper(0.104, 0.094, 0.030, 0.072, 0.010))),
        ],
        "link0_accent": [("collar", revolve(tube(0.089, 0.072, 0.086, 0.003, 0.003)))],
        # link1 -- shoulder riser, and the j2 housing lying along Y
        "link1_shell": [("riser", revolve(taper(0.088, 0.082, 0.000, 0.176, 0.014)))],
        "link1_joint": [("j2", revolve(tube(0.092, -0.092, 0.092, 0.030, 0.030),
                                       axis="y", offset=(0.0, 0.0, 0.220)))],
        # link2 -- upper arm: housing, tapered tube, accent ring, housing
        "link2_shell": [("tube", revolve(taper(0.076, 0.068, 0.030, 0.352, 0.012)))],
        "link2_accent": [("ring", revolve(tube(0.078, 0.322, 0.338, 0.004, 0.004)))],
        "link2_joint": [
            ("j2b", revolve(tube(0.088, -0.088, 0.088, 0.028, 0.028), axis="y")),
            ("j3", revolve(tube(0.086, -0.084, 0.084, 0.026, 0.026), axis="y",
                           offset=(0.0, 0.0, 0.380))),
        ],
        # link3 -- elbow rotor
        "link3_shell": [("rotor", revolve(taper(0.082, 0.076, -0.020, 0.126, 0.012)))],
        "link3_accent": [("collar", revolve(tube(0.074, 0.126, 0.140, 0.003, 0.003)))],
        # link4 -- forearm
        "link4_shell": [("tube", revolve(taper(0.068, 0.060, 0.030, 0.392, 0.012)))],
        "link4_accent": [("ring", revolve(tube(0.070, 0.352, 0.366, 0.004, 0.004)))],
        "link4_joint": [
            ("j4b", revolve(tube(0.078, -0.076, 0.076, 0.024, 0.024), axis="y")),
            ("j5", revolve(tube(0.078, -0.076, 0.076, 0.024, 0.024), axis="y",
                           offset=(0.0, 0.0, 0.420))),
        ],
        # link5 -- wrist rotor
        "link5_shell": [("rotor", revolve(taper(0.070, 0.064, -0.018, 0.112, 0.010)))],
        "link5_accent": [("collar", revolve(tube(0.062, 0.112, 0.124, 0.003, 0.003)))],
        # link6 -- wrist and tool flange
        "link6_shell": [("wrist", revolve(taper(0.058, 0.052, 0.000, 0.140, 0.010)))],
        "link6_joint": [("flange", revolve(tube(0.0475, 0.140, 0.1655, 0.004, 0.004)))],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="fail if files on disk differ from what this emits")
    args = ap.parse_args()
    rc, tris = 0, 0
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for stem, parts in omniarm6().items():
        path = OUT_DIR / (stem + ".obj")
        text = build_obj(parts)
        n = sum(len(f) for _, (_, _, f) in parts) * 2
        tris += n
        if args.check:
            cur = path.read_text(encoding="utf-8") if path.exists() else None
            if cur != text:
                print("DRIFT %s" % path.relative_to(REPO))
                rc = 1
            else:
                print("ok    %s" % path.relative_to(REPO))
        else:
            path.write_text(text, encoding="utf-8", newline="\n")
            print("wrote %-52s %6d tris" % (path.relative_to(REPO), n))
    if not args.check:
        print("total %d triangles" % tris)
    return rc


if __name__ == "__main__":
    sys.exit(main())
