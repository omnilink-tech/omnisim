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

"""Task-world-support prober for **upstream Webots R2025a**: world-space AABBs
plus the Viewpoint pose, measured from the LIVE loaded scene at a frozen t=0.

Why it exists (BRINGUP.md sec. 4 gap 1): upstream's Supervisor API has no
bounds query, so a world-space AABB only exists if a controller computes one
itself from each body's ``boundingObject`` geometry and pose. The Lane B
graders (B1 pairwise overlap, B3 taller-by-AABB-top, C2 rests-on-the-floor,
B2 subject bounds) all consume AABBs, so without this measurement the Webots
arm cannot be graded on those tasks at all.

It is deliberately a SEPARATE controller from the grader's recorder, run in a
SEPARATE measurement launch by ``adapters.webots.task_support`` (the graded
run's world is never touched): a second Robot node in the graded world would
pollute the recorder's roster. The world is byte-identical across the two
launches and both scans are synchronization-TRUE t=0 scans, so the geometry
is the same by construction; task_support records that provenance on the
bundle.

Scope, stated plainly (all limits are published, none silently ignored):

* only **top-level** bodies are measured, from their own (or, for PROTO
  instances, their base node's) ``boundingObject`` -- for a robot PROTO like
  the Pioneer 3-AT this is the body hull and excludes the wheels, which
  under-states the AABB slightly. The task worlds place their decisive
  geometry far outside that error;
* geometry primitives handled: Box, Cylinder (z-axis, the R2022b+
  convention), Sphere, Capsule, wrapped arbitrarily in Pose/Transform/Group
  (and Shape); a Plane or Mesh contributes nothing and is counted in
  ``skipped_geometry``;
* the world must be z-up ENU (R2022b+ default); the doc records the world's
  declared coordinate system so the reader can check.

Uses only upstream's own ``controller`` API + stdlib; must not import
anything from the OmniSim tree.
"""

import json
import math
import os
import sys

from controller import Supervisor  # upstream Webots controller API

_SKIP_NAMES = ("agentbench_recorder", "agentbench_aabb_prober")


def _args():
    out = {"out_dir": "."}
    for a in sys.argv[1:]:
        if a.startswith("--out-dir="):
            out["out_dir"] = a.split("=", 1)[1]
    return out


def _field(node, field_name):
    """Same lookup rule as the grader's recorder (own field first, then --
    for PROTO instances only -- the base node's hidden field)."""
    try:
        f = node.getField(field_name)
    except Exception:  # noqa: BLE001
        f = None
    if f is not None:
        return f
    try:
        if node.getProto() is not None:
            return node.getBaseNodeField(field_name)
    except Exception:  # noqa: BLE001
        pass
    return None


def _sf_string(node, field_name):
    try:
        f = _field(node, field_name)
        return None if f is None else f.getSFString()
    except Exception:  # noqa: BLE001
        return None


def _sf_float(node, field_name):
    try:
        f = _field(node, field_name)
        return None if f is None else float(f.getSFFloat())
    except Exception:  # noqa: BLE001
        return None


def _sf_vec3(node, field_name):
    try:
        f = _field(node, field_name)
        if f is None:
            return None
        v = f.getSFVec3f()
        return [float(v[0]), float(v[1]), float(v[2])]
    except Exception:  # noqa: BLE001
        return None


def _sf_rot(node, field_name):
    try:
        f = _field(node, field_name)
        if f is None:
            return None
        v = f.getSFRotation()
        return [float(v[0]), float(v[1]), float(v[2]), float(v[3])]
    except Exception:  # noqa: BLE001
        return None


def _axis_angle_matrix(rot):
    """3x3 rotation matrix (row lists) from [x, y, z, angle]."""
    x, y, z, a = rot
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(a), math.sin(a)
    C = 1.0 - c
    return [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def _mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def _vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


_HALF_EXTENT_GEOMS = ("Box", "Cylinder", "Sphere", "Capsule")


def _local_half_extents(node, base):
    """Half extents of a leaf geometry in its own frame, or None.

    Cylinder/Capsule axes are z (the R2022b-and-later convention this
    release uses).
    """
    if base == "Box":
        try:
            f = _field(node, "size")
            s = f.getSFVec3f()
            return [abs(s[0]) / 2.0, abs(s[1]) / 2.0, abs(s[2]) / 2.0]
        except Exception:  # noqa: BLE001
            return None
    if base == "Cylinder":
        r = _sf_float(node, "radius")
        h = _sf_float(node, "height")
        if r is None or h is None:
            return None
        return [abs(r), abs(r), abs(h) / 2.0]
    if base == "Sphere":
        r = _sf_float(node, "radius")
        return None if r is None else [abs(r)] * 3
    if base == "Capsule":
        r = _sf_float(node, "radius")
        h = _sf_float(node, "height")
        if r is None or h is None:
            return None
        return [abs(r), abs(r), abs(h) / 2.0 + abs(r)]
    return None


def _walk_bounding(node, R, p, corners, skipped, depth=0):
    """Accumulate world-space corner points of every primitive under a
    boundingObject subtree. (R, p) is the world transform of the subtree
    root's frame."""
    if node is None or depth > 16:
        return
    try:
        base = node.getBaseTypeName()
    except Exception:  # noqa: BLE001
        return
    if base in ("Pose", "Transform"):
        t = _sf_vec3(node, "translation") or [0.0, 0.0, 0.0]
        r = _sf_rot(node, "rotation")
        Rl = _axis_angle_matrix(r) if r else [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        Rn = _mat_mul(R, Rl)
        pn = _vec_add(p, _mat_vec(R, t))
        _walk_children(node, Rn, pn, corners, skipped, depth)
        return
    if base == "Group":
        _walk_children(node, R, p, corners, skipped, depth)
        return
    if base == "Shape":
        f = _field(node, "geometry")
        try:
            geom = f.getSFNode() if f is not None else None
        except Exception:  # noqa: BLE001
            geom = None
        if geom is not None:
            _walk_bounding(geom, R, p, corners, skipped, depth + 1)
        return
    if base in _HALF_EXTENT_GEOMS:
        he = _local_half_extents(node, base)
        if he is None:
            skipped.append(base + " (unreadable)")
            return
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    local = [sx * he[0], sy * he[1], sz * he[2]]
                    corners.append(_vec_add(p, _mat_vec(R, local)))
        return
    # Plane (infinite), Mesh/IndexedFaceSet (vertex data not walked),
    # ElevationGrid, anything unrecognised: recorded, never guessed.
    skipped.append(base)


def _walk_children(node, R, p, corners, skipped, depth):
    f = _field(node, "children")
    if f is None:
        return
    try:
        count = f.getCount()
    except Exception:  # noqa: BLE001
        return
    for i in range(max(0, count)):
        try:
            child = f.getMFNode(i)
        except Exception:  # noqa: BLE001
            child = None
        if child is not None:
            _walk_bounding(child, R, p, corners, skipped, depth + 1)


def _aabb_for(node):
    """(bbox_min, bbox_max, skipped) from the node's boundingObject, in the
    world frame -- or (None, None, skipped)."""
    skipped = []
    f = _field(node, "boundingObject")
    try:
        bo = f.getSFNode() if f is not None else None
    except Exception:  # noqa: BLE001
        bo = None
    if bo is None:
        return None, None, ["no boundingObject"]
    try:
        p = list(node.getPosition())
        o = list(node.getOrientation())
        R = [o[0:3], o[3:6], o[6:9]]
    except Exception as exc:  # noqa: BLE001
        return None, None, ["world pose unreadable: %r" % (exc,)]
    corners = []
    _walk_bounding(bo, R, p, corners, skipped)
    if not corners:
        return None, None, skipped
    lo = [min(c[i] for c in corners) for i in range(3)]
    hi = [max(c[i] for c in corners) for i in range(3)]
    return lo, hi, skipped


def main():
    cfg = _args()
    out_dir = cfg["out_dir"]
    os.makedirs(out_dir, exist_ok=True)

    sup = Supervisor()
    dt_ms = int(sup.getBasicTimeStep())
    self_id = sup.getSelf().getId() if sup.getSelf() is not None else None

    coordinate_system = "ENU"
    viewpoint = None
    bodies = {}

    root_children = sup.getRoot().getField("children")
    for i in range(root_children.getCount()):
        node = root_children.getMFNode(i)
        if node is None:
            continue
        try:
            base = node.getBaseTypeName()
        except Exception:  # noqa: BLE001
            continue
        if base == "WorldInfo":
            cs = _sf_string(node, "coordinateSystem")
            if cs:
                coordinate_system = cs
            continue
        if base == "Viewpoint":
            viewpoint = {
                "position": _sf_vec3(node, "position"),
                "orientation": _sf_rot(node, "orientation"),
                "field_of_view": _sf_float(node, "fieldOfView"),
            }
            continue
        if node.getId() == self_id:
            continue
        name = _sf_string(node, "name")
        if not name or name in _SKIP_NAMES:
            continue
        lo, hi, skipped = _aabb_for(node)
        rec = {"skipped_geometry": skipped}
        if lo is not None:
            rec["bbox_min"] = lo
            rec["bbox_max"] = hi
        bodies[name] = rec

    doc = {
        "t_s": 0.0,
        "frozen": True,
        "synchronization": True,
        "coordinate_system": coordinate_system,
        "world": sup.getWorldPath(),
        "bodies": bodies,
        "viewpoint": viewpoint,
        "source": ("agentbench_aabb_prober: synchronization-TRUE Supervisor "
                   "scan before the first step; AABBs computed from each "
                   "top-level body's boundingObject geometry and world pose "
                   "(upstream has no bounds API)"),
    }
    tmp = os.path.join(out_dir, "aabbs.json.tmp")
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(doc, fh)
    os.replace(tmp, os.path.join(out_dir, "aabbs.json"))

    # One step so the grader's co-injected recorder finishes its own t=0
    # write before this controller ends the run.
    sup.step(dt_ms)
    sup.simulationQuit(0)
    sup.step(dt_ms)


if __name__ == "__main__":
    main()
