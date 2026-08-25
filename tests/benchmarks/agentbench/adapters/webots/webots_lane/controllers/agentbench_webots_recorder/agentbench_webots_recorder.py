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

"""The grader-owned Supervisor recorder for **upstream Webots R2025a**.

This controller is injected into a world by
``agentbench.adapters.webots.launcher`` and is the *only* measurement channel
an upstream run has: R2025a exposes no HTTP surface, no scene-tree query, no
``--duration`` and no auto-exit (``docs/developer/webots-control-baseline.md``
sec. 3, sec. 6). It therefore does three jobs:

1. **scan** every top-level body at t=0, before its first ``step()`` -- the
   recorder Robot is ``synchronization TRUE``, so the world is genuinely frozen
   while it walks the scene (``roster.json``, ``frozen: true``);
2. **sample** every tracked body's world position once per basic timestep
   (``trajectory.json``), and pair ``Node.getContactPoints()`` world points
   across bodies for ``--contact-steps`` steps -- or, with
   ``--contact-steps=-1``, for the whole run (``contacts.json``);
3. **terminate** the run: upstream only ends cleanly through
   ``simulationQuit()``, so after ``--duration`` simulated seconds it writes
   ``completion.json`` (the clean-quit attestation) and quits -- which is also
   what makes ``--log-performance`` flush (upstream writes it only in
   ``worldClosed()``).

The artifact set written into ``--out-dir`` is exactly what
``agentbench.adapters.webots.recording.read_run`` parses:
``trajectory.json``, ``roster.json``, ``contacts.json``, ``completion.json``.
(``process.json`` and the console/perf captures belong to the launcher -- they
are facts about the *process*, which an in-world controller cannot honestly
attest.)

Artifacts are written incrementally (roster at t=0, contacts after the scan
window, trajectory + completion at the end) so a killed run still leaves its
partial evidence -- and, deliberately, NO ``completion.json``, because a run
that was killed did not complete.

**What "tracked body" means, and the symmetry rule.** Until 2026-08-09 it meant
"a top-level ``root.children`` node that is a Robot or has a Physics node" --
the upstream twin of the OmniSim recorder's Robot-only walk, and the same gap:
an arm authored as ONE Robot exposed only its fixed base, so R2 had no end
effector to measure, and a graspable part nested inside a group had no track at
all. There are now three track kinds, in this ``trajectory.json`` order:

    ``robot``  top-level Robot-derived bodies, exactly as before
    ``solid``  top-level dynamic bodies (as before) plus any body the grader
               NAMED via ``--solids=``, wherever it sits in the scene
    ``link``   moving bodies INSIDE a robot (``--links=N``, default OFF)

The order and the opt-in defaults are chosen so an already-recorded task's
artifact set is unchanged, and so that BOTH arms answer the same question --
a cross-simulator comparison where one arm can see an end effector and the
other cannot is not a comparison.

**Contacts name BOTH participants, including scenery (2026-08-09).** Upstream
gives a contact point and the *queried* subtree, never the other party, so
naming both sides means querying both and pairing by world point -- and until
this date only Robot nodes were queried. A robot striking an obstacle or a wall
therefore had no second participant and could not be NAMED, which made R1.5
("nothing was hit") structurally unfailable on this arm: every R1 run, honest
or not, reported zero collisions. The pass now queries the name-free scan's
non-robot bodies too (capped at :data:`_CONTACT_SCENERY_CAP`, truncation
reported) and emits ``{a: robot, b: body, b_robot: False}`` for a matched
point. It rests on a fact that had to be measured rather than assumed -- a
Solid with **no Physics node** has no ODE rigid body, yet still answers
``getContactPoints()``: a Pioneer 3-AT driven into a static 1.6 m box produced
four points on the BOX's own query, at its -x face, on the same step the
robot's query listed the same four (``controllers/r1_probe_contacts``). The two
robot-side counters are untouched, and every scenery pair carries
``b_robot: False``, so A1.3 -- which reads ``robot_robot_pairs`` and
``distinct_named`` -- measures exactly what it measured before.

**World-space AABBs: computed here, in a channel of their own.** Upstream's
Supervisor API has **no bounds query** (``BRINGUP.md`` §4.1), so an AABB on this
arm only exists if a controller derives one from the body's own geometry and
world pose. This recorder now does, for the same reason the OmniSim recorder
grew a name-free scan on the same day: the suite's geometric assertions match by
world-space box and never by name (an agent called R1's obstacles ``crate
A``..``crate E``), so a bounds channel that has to be handed a name list cannot
answer them at all. See ``scene_bodies`` below.

Two rules keep that from disturbing anything already measured:

* the ``bodies`` list -- the roster the cores count robots in -- is **left
  exactly as it was, with no AABB on any record**. The separate
  ``agentbench_aabb_prober`` merges its own measurements into that list
  (``task_support.augment_run``), and every task already graded through it
  keeps being graded on the prober's numbers, from the prober's launch;
* everything this file measures goes in ``scene_bodies``, keyed by node id, and
  a reader that does not know the key sees the pre-2026-08-09 document.

**What upstream still cannot express, stated rather than approximated.** The
walk resolves ``Box`` / ``Sphere`` / ``Cylinder`` / ``Capsule`` / ``Cone`` /
``Plane`` / ``ElevationGrid`` and explicit coordinate sets (``IndexedFaceSet``
/ ``IndexedLineSet`` / ``PointSet``), wrapped arbitrarily in ``Pose`` /
``Transform`` / ``Group`` / ``Shape``. It does **not** resolve ``Mesh { url
... }``: that needs the STL/OBJ/PLY/glTF file readers OmniSim's
``geometry.bounds_for_subtree`` has, and a controller running under upstream's
own ``python3`` is not the place to grow them. A body bounded only by a mesh
file therefore reports **no AABB and the reason**, never a guessed box -- so it
is visibly unmeasured rather than invisibly wrong. That is a real asymmetry
with the OmniSim arm and it is published on every run rather than left to be
discovered.

This file uses only upstream's own ``controller`` API and runs under the
system ``python3`` that upstream Webots spawns. It must not import anything
from the OmniSim tree.
"""

import json
import math
import os
import sys

from controller import Supervisor  # upstream Webots controller API

# Base node type names that anchor a joint, counted for the roster's
# ``n_joints``. Matches what upstream's getBaseTypeName() reports.
_JOINT_BASE_TYPES = ("HingeJoint", "Hinge2Joint", "SliderJoint",
                     "BallJoint")

# Two contact points from two different bodies' queries closer than this (m)
# are the same physical contact. Generous against solver jitter, far tighter
# than any body separation.
_PAIR_TOL = 1e-4

# Cap on recorded contact pairs -- the counters keep counting past it.
_MAX_PAIRS = 200

# Cap on NON-ROBOT bodies queried for contacts each contact step. THE COST
# BOUND for the robot<->scenery channel: one IPC round-trip per body per step,
# so a whole-run window on a busy scene is bounded by this, not by the scene
# size. Truncation is reported, never silent.
_CONTACT_SCENERY_CAP = 48

# Hard per-robot ceiling on link tracks, whatever --links= asks for. THE COST
# BOUND: this controller samples every track once per basic timestep, and an
# imported robot can carry dozens of link bodies, so the walk's RESULT is
# capped as well as its predicate being narrow. Kept identical to the OmniSim
# recorder's LINK_CAP_CEILING so neither arm can measure more of an arm than
# the other. Truncation is reported, never silent.
_LINK_CAP_CEILING = 64

# Which named --solids= get a per-step track. Same three modes, same default,
# same reasoning as the OmniSim recorder: a body with no mass model cannot move
# under the solver, so a track for it records a constant.
_SOLID_TRACK_MODES = ("dynamic", "all", "none")
_DEFAULT_SOLID_TRACKS = "dynamic"

# Cap on the NAME-FREE t=0 scene scan, kept identical to the OmniSim
# recorder's SCENE_SCAN_DEFAULT_CAP so neither arm can offer a geometric
# matcher more candidates than the other. THE COST BOUND: the scan runs ONCE,
# at t=0, and adds NO sampled row, so a long recording is not one sample per
# body more expensive for having run it -- what it costs is one geometry walk
# per body found, over an API where every field read is an IPC round-trip.
# Three bounds apply together (predicate, depth limit, this ceiling on the
# result) and truncation is reported in `scene_bodies`, never silent.
_SCENE_SCAN_DEFAULT_CAP = 128
_SCENE_SCAN_CEILING = 512
_SCENE_SCAN_DEPTH_LIMIT = 32


def _args():
    out = {"out_dir": ".", "duration": 10.0, "contact_steps": 10,
           "solids": [], "solid_tracks": _DEFAULT_SOLID_TRACKS, "links": 0,
           "scan_solids": _SCENE_SCAN_DEFAULT_CAP}
    for a in sys.argv[1:]:
        if a.startswith("--out-dir="):
            out["out_dir"] = a.split("=", 1)[1]
        elif a.startswith("--duration="):
            out["duration"] = float(a.split("=", 1)[1])
        elif a.startswith("--contact-steps="):
            # < 0 means "the WHOLE run" -- the window R1.5 ("nothing was hit
            # over the whole run") needs and the one the MuJoCo arm always
            # takes. A fixed first-N-steps window can only ever witness a
            # collision that happens in the first N steps, which for a 60 s
            # navigation task is none of them.
            try:
                out["contact_steps"] = max(-1, int(a.split("=", 1)[1]))
            except ValueError:
                out["contact_steps"] = 10
        elif a.startswith("--solids="):
            out["solids"] = [s.strip() for s in a.split("=", 1)[1].split(",")
                             if s.strip()]
        elif a.startswith("--solid-tracks="):
            mode = a.split("=", 1)[1].strip()
            out["solid_tracks"] = (mode if mode in _SOLID_TRACK_MODES
                                   else _DEFAULT_SOLID_TRACKS)
        elif a.startswith("--links="):
            try:
                out["links"] = max(0, min(int(a.split("=", 1)[1]),
                                          _LINK_CAP_CEILING))
            except ValueError:
                out["links"] = 0
        elif a.startswith("--scan-solids="):
            try:
                out["scan_solids"] = max(0, min(int(a.split("=", 1)[1]),
                                                _SCENE_SCAN_CEILING))
            except ValueError:
                out["scan_solids"] = _SCENE_SCAN_DEFAULT_CAP
    return out


def _write(out_dir, name, doc):
    """Atomic-enough JSON write: tmp + rename, so a kill mid-write cannot
    leave a half-file that parses as garbage."""
    path = os.path.join(out_dir, name)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(doc, fh)
    os.replace(tmp, path)


def _field(node, field_name):
    """The named field: the node's own (or PROTO-interface) field first, else
    -- for PROTO instances only -- the base node's hidden field.

    The order and the ``getProto()`` gate both matter: calling
    ``getBaseNodeField`` on a NON-proto node makes the engine print
    ``Error: ... 'node' is not a PROTO node`` into the console this adapter
    later reads, and ``getId()`` on internal PROTO nodes is similarly refused
    -- both verified live against R2025a's Pioneer3at.
    """
    try:
        f = node.getField(field_name)
    except Exception:  # noqa: BLE001 -- a broken field read is not a crash
        f = None
    if f is not None:
        return f
    try:
        if node.getProto() is not None:
            return node.getBaseNodeField(field_name)
    except Exception:  # noqa: BLE001
        pass
    return None


def _sf_node_present(node, field_name):
    """Does ``field_name`` (an SFNode) hold a node? None when unreadable."""
    try:
        f = _field(node, field_name)
        if f is None:
            return None
        return f.getSFNode() is not None
    except Exception:  # noqa: BLE001
        return None


def _sf_string(node, field_name):
    try:
        f = _field(node, field_name)
        if f is None:
            return None
        return f.getSFString()
    except Exception:  # noqa: BLE001
        return None


def _count_joints(node, depth=0):
    """Joints in ``node``'s subtree, PROTO internals included.

    No id-based dedupe: ``getId()`` returns -1 for every internal PROTO node
    (upstream refuses the call), so a visited-set keyed on it collapses the
    whole subtree into one entry and counts zero -- the bug the first
    bring-up run hit. The scene tree is acyclic; a ``USE`` subtree may be
    counted once per use, which over-counts rather than zeroes.
    """
    if depth > 32:
        return 0
    total = 0
    try:
        if node.getBaseTypeName() in _JOINT_BASE_TYPES:
            total += 1
    except Exception:  # noqa: BLE001
        pass
    for fname in ("children", "endPoint", "device", "device2", "device3"):
        f = _field(node, fname)
        if f is None:
            continue
        try:
            count = f.getCount()
        except Exception:  # noqa: BLE001
            count = None
        if count is not None and count >= 0:      # MF field
            for i in range(count):
                try:
                    child = f.getMFNode(i)
                except Exception:  # noqa: BLE001
                    child = None
                if child is not None:
                    total += _count_joints(child, depth + 1)
        else:                                     # SF field
            try:
                child = f.getSFNode()
            except Exception:  # noqa: BLE001
                child = None
            if child is not None:
                total += _count_joints(child, depth + 1)
    return total


def _finite3(p):
    return (isinstance(p, (list, tuple)) and len(p) >= 3
            and all(isinstance(v, float) or isinstance(v, int) for v in p[:3]))


def _base_type(node):
    try:
        return node.getBaseTypeName()
    except Exception:  # noqa: BLE001
        return ""


def _mf_children(node, fname):
    """The MF-node children of ``fname``, or ``[]``. Never raises."""
    f = _field(node, fname)
    if f is None:
        return []
    try:
        n = f.getCount()
    except Exception:  # noqa: BLE001
        return []
    if n is None or n < 0:
        return []
    out = []
    for i in range(n):
        try:
            c = f.getMFNode(i)
        except Exception:  # noqa: BLE001
            continue
        if c is not None:
            out.append(c)
    return out


def _sf_child(node, fname):
    f = _field(node, fname)
    if f is None:
        return None
    try:
        return f.getSFNode()
    except Exception:  # noqa: BLE001
        return None


def _link_bodies(robot, depth_limit=32):
    """Moving BODIES inside one Robot, depth-first from the base outwards.

    Returns ``[(node, via_endpoint), ...]``. Same predicate as the OmniSim
    recorder's, deliberately -- a Solid-derived descendant counts when it is a
    joint ``endPoint`` (so a KINEMATIC arm, authored without Physics nodes,
    still exposes its chain) or when it carries a ``Physics`` node (so a tool
    body rigidly parented to the last link is not missed). Nested ``Robot``
    nodes are not entered: they are tracked in their own right, and recursing
    would give one body two rows.

    No id-based visited-set: ``getId()`` returns -1 for every internal PROTO
    node and upstream refuses the call, so keying a seen-set on it collapses a
    whole PROTO subtree into one entry -- the same trap ``_count_joints``
    documents. The tree is acyclic; ``depth_limit`` is the runaway guard.
    """
    out = []

    def visit(node, via_endpoint, depth):
        if node is None or depth > depth_limit:
            return
        if _base_type(node) == "Robot":
            return
        if _base_type(node) == "Solid":
            if via_endpoint or _sf_node_present(node, "physics") is True:
                out.append((node, via_endpoint))
        for fname in ("children", "device", "device2", "device3"):
            for c in _mf_children(node, fname):
                visit(c, False, depth + 1)
        ep = _sf_child(node, "endPoint")
        if ep is not None:
            visit(ep, True, depth + 1)

    for c in _mf_children(robot, "children"):
        visit(c, False, 1)
    return out


def _named_solids(root, wanted, depth_limit=32):
    """Solid-derived bodies ANYWHERE in the scene whose ``name`` was asked for.

    Scene-wide rather than top-level-only: the OmniSim recorder's ``--solids=``
    walks the whole tree, and a named body that one arm can find and the other
    cannot is exactly the asymmetry this pass exists to remove.
    """
    if not wanted:
        return []
    out = []

    def visit(node, depth):
        if node is None or depth > depth_limit:
            return
        if _base_type(node) == "Solid":
            nm = _sf_string(node, "name")
            if nm in wanted:
                out.append((nm, node))
        for c in _mf_children(node, "children"):
            visit(c, depth + 1)
        ep = _sf_child(node, "endPoint")
        if ep is not None:
            visit(ep, depth + 1)

    for c in _mf_children(root, "children"):
        visit(c, 1)
    return out


# --- world-space AABBs, computed (upstream has no bounds query) --------------
#
# The primitive conventions are R2022b-and-later's, and they are the ones the
# OmniSim harness's geometry.py uses, transcribed rather than guessed:
# Cylinder / Capsule / Cone run along local +z with the radius spanning xy;
# Box.size is the FULL extent; Plane.size lies in xy; ElevationGrid spans x/y
# with heights along z. Getting the cylinder axis wrong is not academic -- it
# inflated every Husky wheel hull by (radius - halfHeight) on the wrong axis
# and made the robot measure 0.885 m wide instead of 0.685 m.


def _sf_float(node, field_name, default=None):
    try:
        f = _field(node, field_name)
        return default if f is None else float(f.getSFFloat())
    except Exception:  # noqa: BLE001
        return default


def _sf_int(node, field_name, default=0):
    try:
        f = _field(node, field_name)
        return default if f is None else int(f.getSFInt32())
    except Exception:  # noqa: BLE001
        return default


def _sf_vec3(node, field_name, default=None):
    try:
        f = _field(node, field_name)
        if f is None:
            return default
        v = f.getSFVec3f()
        return [float(v[0]), float(v[1]), float(v[2])]
    except Exception:  # noqa: BLE001
        return default


def _sf_vec2(node, field_name, default=None):
    try:
        f = _field(node, field_name)
        if f is None:
            return default
        v = f.getSFVec2f()
        return [float(v[0]), float(v[1])]
    except Exception:  # noqa: BLE001
        return default


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
    """3x3 rotation matrix (row lists) from ``[x, y, z, angle]``."""
    x, y, z, a = rot
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
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


def _local_corners(node, base):
    """Local-frame corner points of one leaf geometry, or ``None``.

    Corners rather than half-extents because an explicit coordinate set has no
    half-extent: the same return shape then carries a Box and an
    ``IndexedFaceSet`` through the same transform.
    """
    if base == "Box":
        s = _sf_vec3(node, "size", [2.0, 2.0, 2.0])
        h = [abs(s[0]) / 2.0, abs(s[1]) / 2.0, abs(s[2]) / 2.0]
    elif base == "Sphere":
        r = abs(_sf_float(node, "radius", 1.0) or 1.0)
        h = [r, r, r]
    elif base in ("Cylinder", "Capsule"):
        r = abs(_sf_float(node, "radius", 1.0) or 1.0)
        hz = abs(_sf_float(node, "height", 2.0) or 2.0) / 2.0
        if base == "Capsule":
            hz += r          # hemispherical caps sit beyond the barrel
        h = [r, r, hz]
    elif base == "Cone":
        r = abs(_sf_float(node, "bottomRadius", 1.0) or 1.0)
        h = [r, r, abs(_sf_float(node, "height", 2.0) or 2.0) / 2.0]
    elif base == "Plane":
        # A Plane is INFINITE for collision; this is its DRAWN extent, which
        # under-states the surface a body can rest on. Flagged by the caller.
        s = _sf_vec2(node, "size", [1.0, 1.0])
        h = [abs(s[0]) / 2.0, abs(s[1]) / 2.0, 0.0]
    elif base == "ElevationGrid":
        nx = _sf_int(node, "xDimension", 0) or 0
        ny = _sf_int(node, "yDimension", 0) or 0
        sx = _sf_float(node, "xSpacing", 1.0) or 1.0
        sy = _sf_float(node, "ySpacing", 1.0) or 1.0
        zs = _mf_floats(node, "height")
        thick = _sf_float(node, "thickness", 0.0) or 0.0
        lo = [0.0, 0.0, (min(zs) if zs else 0.0) - thick]
        hi = [max(0, nx - 1) * sx, max(0, ny - 1) * sy,
              (max(zs) if zs else 0.0)]
        return [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                for z in (lo[2], hi[2])]
    elif base in ("IndexedFaceSet", "IndexedLineSet", "PointSet"):
        pts = _coord_points(node)
        return pts or None
    else:
        return None
    return [[sx * h[0], sy * h[1], sz * h[2]]
            for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)]


def _mf_floats(node, field_name, limit=200000):
    try:
        f = _field(node, field_name)
        n = f.getCount() if f is not None else 0
    except Exception:  # noqa: BLE001
        return []
    out = []
    for i in range(min(max(0, n), limit)):
        try:
            out.append(float(f.getMFFloat(i)))
        except Exception:  # noqa: BLE001
            break
    return out


def _coord_points(node, limit=200000):
    """``coord Coordinate { point [...] }`` as local xyz triples."""
    coord = _sf_child(node, "coord")
    if coord is None:
        return []
    try:
        f = _field(coord, "point")
        n = f.getCount() if f is not None else 0
    except Exception:  # noqa: BLE001
        return []
    out = []
    for i in range(min(max(0, n), limit)):
        try:
            v = f.getMFVec3f(i)
        except Exception:  # noqa: BLE001
            break
        if v is not None and len(v) >= 3:
            out.append([float(v[0]), float(v[1]), float(v[2])])
    return out


_INFINITE_SURFACE = "Plane"


def _walk_geometry(node, R, p, corners, skipped, depth=0):
    """Accumulate world-space corner points under one geometry subtree.

    ``(R, p)`` is the world transform of the subtree root's frame. Everything
    unrecognised is APPENDED TO ``skipped`` and contributes nothing -- a
    ``Mesh { url ... }`` in particular, which needs a file reader this arm does
    not have. An unmeasured body must be visibly unmeasured.
    """
    if node is None or depth > 16:
        return
    base = _base_type(node)
    if not base:
        return
    if base in ("Pose", "Transform"):
        t = _sf_vec3(node, "translation", [0.0, 0.0, 0.0])
        r = _sf_rot(node, "rotation")
        Rl = (_axis_angle_matrix(r) if r
              else [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        _walk_geometry_children(node, _mat_mul(R, Rl),
                                _vec_add(p, _mat_vec(R, t)), corners, skipped,
                                depth)
        return
    if base == "Group":
        _walk_geometry_children(node, R, p, corners, skipped, depth)
        return
    if base == "Shape":
        geom = _sf_child(node, "geometry")
        if geom is not None:
            _walk_geometry(geom, R, p, corners, skipped, depth + 1)
        return
    local = _local_corners(node, base)
    if local is None:
        # Mesh / IndexedFaceSet with no coord / anything unrecognised.
        skipped.append(base)
        return
    if base == _INFINITE_SURFACE:
        skipped.append("Plane (INFINITE for collision; the box below is its "
                       "drawn extent and under-states the surface)")
    for c in local:
        corners.append(_vec_add(p, _mat_vec(R, c)))


def _walk_geometry_children(node, R, p, corners, skipped, depth):
    for c in _mf_children(node, "children"):
        _walk_geometry(c, R, p, corners, skipped, depth + 1)


def _world_aabb(node):
    """``(bbox_min, bbox_max, source, skipped)`` for one body, or Nones.

    ``boundingObject`` first -- it is what the body actually collides with and
    it is the channel ``agentbench_aabb_prober`` uses, so a body that has one
    is measured the same way in both places. A body with NO ``boundingObject``
    falls back to its VISUAL ``children`` geometry, because the OmniSim arm's
    helper unions both and a body that is invisible to one arm's bounds and
    not the other's is an instrument asymmetry wearing a capability's clothes.
    ``source`` says which was used, on every body.
    """
    try:
        pos = list(node.getPosition())
        o = list(node.getOrientation())
        R = [o[0:3], o[3:6], o[6:9]]
    except Exception as exc:  # noqa: BLE001
        return None, None, None, ["world pose unreadable: %r" % (exc,)]

    bo = _sf_child(node, "boundingObject")
    attempts = [("boundingObject", [bo] if bo is not None else []),
                ("children geometry", _mf_children(node, "children"))]
    all_skipped = []
    for source, roots in attempts:
        roots = [r for r in roots if r is not None]
        if not roots:
            continue
        corners, skipped = [], []
        for r in roots:
            _walk_geometry(r, R, pos, corners, skipped)
        all_skipped.extend(skipped)
        if corners:
            lo = [min(c[i] for c in corners) for i in range(3)]
            hi = [max(c[i] for c in corners) for i in range(3)]
            return lo, hi, source, sorted(set(skipped))
        # Nothing measurable under this root -- a mesh hull, say. Fall through
        # to the next channel rather than reporting nothing, and carry the
        # reason so the body says WHY it is unmeasured.
    return None, None, None, sorted(set(all_skipped)) or [
        "no measurable geometry"]


# --- the name-free t=0 scene scan --------------------------------------------


def _node_key(node):
    """A stable per-run key: the DEF, else ``#<node id>``, else the name.

    The SAME convention ``evidence._body_id`` uses for the roster, so a scene
    entry and a roster entry for one body carry one identity and the adapter
    can merge them without a name match. ``getId()`` is -1 for internal PROTO
    nodes, which is why the name is the last resort rather than the first.
    """
    try:
        d = node.getDef()
    except Exception:  # noqa: BLE001
        d = None
    if d:
        return str(d)
    nid = _safe_id(node)
    if nid is not None and int(nid) >= 0:
        return "#%d" % int(nid)
    return _sf_string(node, "name") or ""


def _scene_bodies(root, claimed_keys, cap, depth_limit=_SCENE_SCAN_DEPTH_LIMIT):
    """Every non-robot Solid in the scene, found WITHOUT a name list.

    Returns ``(entries, found_total)``. Same predicate as the OmniSim
    recorder's ``_scene_bodies``, deliberately: Solid-derived bodies only, a
    Robot's subtree is never entered (its links are ``--links=``' business and
    the recorder's own Robot is excluded for free), a body already offered
    through another channel is not offered twice, and a Solid nested inside
    another Solid IS recorded with ``nested_in`` naming its container.

    No id-keyed visited set: ``getId()`` returns -1 for every internal PROTO
    node and upstream refuses the call, so a seen-set keyed on it collapses a
    whole PROTO subtree into one entry -- the trap ``_count_joints`` documents.
    The scene tree is acyclic; ``depth_limit`` is the runaway guard.
    """
    out = []
    found = [0]

    def visit(node, parent_key, depth):
        if node is None or depth > depth_limit:
            return
        base = _base_type(node)
        if base == "Robot":
            return
        child_parent = parent_key
        if base == "Solid":
            child_parent = _node_key(node)
            if child_parent not in claimed_keys:
                found[0] += 1
                if len(out) < cap:
                    out.append((node, child_parent, parent_key, depth))
        for fname in ("children", "device", "device2", "device3"):
            for c in _mf_children(node, fname):
                visit(c, child_parent, depth + 1)
        ep = _sf_child(node, "endPoint")
        if ep is not None:
            visit(ep, child_parent, depth + 1)

    for c in _mf_children(root, "children"):
        visit(c, None, 1)
    return out, found[0]


def _scene_scan(root, claimed_keys, cap, self_id=None):
    """``(scene_bodies document, [(key, node), ...])``.

    The second return is the same bodies as node handles, in scan order, so the
    contact pass can QUERY them without a second walk. It exists because a
    contact this recorder cannot name a second participant for is a contact no
    collision assertion can count -- see ``scan_contacts``.
    """
    doc = {
        "supported": True,
        "cap": int(cap), "found": 0, "bounded": 0, "truncated": False,
        "source": (
            "name-free t=0 scan by the grader-owned Supervisor recorder "
            "(synchronization TRUE, before the first step): every non-robot "
            "Solid-derived body outside a Robot subtree, with a world-space "
            "AABB COMPUTED from its own geometry and world pose. Upstream "
            "exposes no bounds query, so nothing here is read from the "
            "engine's own bounding volume. Mesh { url ... } geometry is NOT "
            "resolved and such a body reports no AABB with the reason"),
        "bodies": [],
    }
    if cap <= 0:
        doc["supported"] = False
        doc["source"] = "disabled: --scan-solids=0"
        return doc, []
    entries, found = _scene_bodies(root, claimed_keys, cap)
    doc["found"] = found
    doc["truncated"] = found > len(entries)
    nodes = []
    for node, key, parent_key, depth in entries:
        if self_id is not None:
            try:
                if node.getId() == self_id:
                    continue     # the recorder never measures itself
            except Exception:  # noqa: BLE001
                pass
        nodes.append((key, node))
        lo, hi, source, skipped = _world_aabb(node)
        rec = {
            "key": key, "name": _sf_string(node, "name"),
            "def": node.getDef() or None,
            "id": _safe_id(node),
            "type": _safe_type(node), "base_type": _base_type(node),
            "has_physics": _sf_node_present(node, "physics"),
            "nested_in": parent_key, "depth": depth,
            "position": (list(node.getPosition())
                         if _finite3(node.getPosition()) else None),
            "bounds_source": source,
            "skipped_geometry": skipped,
            # The nested layout the OmniSim recorder's t0_solids uses and
            # ``evidence._aabb`` already reads, so one reader serves both arms.
            # Absent (never {"bbox_min": null}) when nothing was measurable:
            # a key that exists with a null inside is how an unmeasured box
            # starts looking like a measured one.
            "bounds": None,
        }
        if lo is not None and hi is not None:
            rec["bounds"] = {"bbox_min": lo, "bbox_max": hi}
            doc["bounded"] += 1
        doc["bodies"].append(rec)
    return doc, nodes


def _safe_id(node):
    try:
        return node.getId()
    except Exception:  # noqa: BLE001
        return None


def _safe_type(node):
    try:
        return node.getTypeName()
    except Exception:  # noqa: BLE001
        return ""


def main():
    cfg = _args()
    out_dir = cfg["out_dir"]
    os.makedirs(out_dir, exist_ok=True)

    sup = Supervisor()
    dt_ms = int(sup.getBasicTimeStep())
    dt_s = dt_ms / 1000.0
    self_id = sup.getSelf().getId() if sup.getSelf() is not None else None

    # ---- t=0 scan, BEFORE the first step(). This Robot is synchronization
    # TRUE (the injected stanza never sets it FALSE; upstream's default is
    # TRUE), so the world is frozen right now.
    world_path = sup.getWorldPath()
    coordinate_system = "ENU"      # R2022b+ default
    root_children = sup.getRoot().getField("children")
    n_top = root_children.getCount()

    tracked = []                   # (key, Node, kind, parent) per sampled row
    roster_bodies = []
    robot_nodes = []               # Robot-derived, for the contact scan
    for i in range(n_top):
        node = root_children.getMFNode(i)
        if node is None:
            continue
        base = node.getBaseTypeName()
        if base == "WorldInfo":
            cs = _sf_string(node, "coordinateSystem")
            if cs:
                coordinate_system = cs
            continue
        if node.getId() == self_id:
            continue               # the recorder does not measure itself
        if base not in ("Robot", "Solid"):
            # Only Solid-derived top-level nodes have a pose worth sampling;
            # Viewpoint / Background / lights and arena PROTOs derived from
            # Solid still pass through the branch below.
            try:
                if node.getField("translation") is None:
                    continue
            except Exception:  # noqa: BLE001
                continue
        name = _sf_string(node, "name") or node.getDef() or (
            "%s#%d" % (node.getTypeName(), node.getId()))
        has_physics = _sf_node_present(node, "physics")
        n_joints = _count_joints(node)
        rec = {
            "name": name,
            "def": node.getDef() or None,
            "id": node.getId(),
            "type": node.getTypeName(),
            "base_type": base,
            "n_joints": n_joints,
            "controller": _sf_string(node, "controller"),
            "has_physics": has_physics,
            "position": list(node.getPosition()),
        }
        roster_bodies.append(rec)
        # Sample only bodies that can move; static scenery stays in the
        # roster but not in the trajectory.
        if has_physics or base == "Robot":
            tracked.append((name, node, "robot" if base == "Robot"
                            else "solid", None))
        if base == "Robot":
            robot_nodes.append((name, node))

    # ---- the two OPT-IN track kinds, appended AFTER every top-level row so
    # every existing row keeps its index (a core that resolves a body through
    # the roster and indexes the pose series with that position must not move).
    tracked_names = set(t[0] for t in tracked)

    link_records = []
    links_truncated = False
    if cfg["links"]:
        for rname, rnode in robot_nodes:
            found = _link_bodies(rnode)
            kept = found[:cfg["links"]]
            if len(found) > len(kept):
                links_truncated = True
            for k, (node, via_endpoint) in enumerate(kept):
                # A STRUCTURAL id, not a name: "link k of robot X", read off
                # the scene's own topology. getId() is -1 for internal PROTO
                # nodes so it cannot carry uniqueness here, and an agent had no
                # reason to name a forearm anything in particular.
                key = "%s/link%d" % (rname, k)
                link_records.append({
                    "key": key, "node": node, "parent": rname,
                    "link_index": k,
                    "name": _sf_string(node, "name"),
                    "def": node.getDef() or None,
                    "type": node.getTypeName(),
                    "base_type": _base_type(node),
                    "joint_endpoint": bool(via_endpoint),
                    "has_physics": _sf_node_present(node, "physics"),
                })
        for rec in link_records:
            tracked.append((rec["key"], rec["node"], "link", rec["parent"]))
            tracked_names.add(rec["key"])

    solid_records = []
    for nm, node in _named_solids(sup.getRoot(), set(cfg["solids"])):
        has_physics = _sf_node_present(node, "physics")
        already = nm in tracked_names
        if cfg["solid_tracks"] == "none":
            why = "--solid-tracks=none: no pose series requested"
        elif already:
            why = "already tracked as a top-level body"
        elif cfg["solid_tracks"] == "all" or has_physics is True:
            why = ("carries a mass model" if has_physics is True
                   else "--solid-tracks=all")
            tracked.append((nm, node, "solid", None))
            tracked_names.add(nm)
        else:
            why = ("no mass model (physics is NULL), so it cannot move under "
                   "the solver; pass --solid-tracks=all if a supervisor "
                   "drives it kinematically")
        solid_records.append({
            "name": nm, "def": node.getDef() or None,
            "key": _node_key(node),
            "type": node.getTypeName(), "has_physics": has_physics,
            "tracked": nm in tracked_names, "track_reason": why,
            "position": (list(node.getPosition())
                         if _finite3(node.getPosition()) else None),
        })

    # ---- the NAME-FREE t=0 scan. Still frozen: nothing below has stepped.
    #
    # NOTHING is claimed away from it, and that is deliberate on this arm.
    # A top-level body and a named ``--solids=`` body are both re-offered
    # here, because ``bodies`` and ``named_solids`` carry NO AABB (that is the
    # prober's channel and it is left byte-for-byte alone) and this scan is the
    # only place on this arm a world-space box exists. Every entry is keyed by
    # ``_node_key``, the same identity ``evidence._body_id`` builds, so the
    # adapter MERGES a scene entry into the record it already has instead of
    # adding a second body -- one body, one identity, bounds attached.
    # A robot's links are excluded structurally: the walk never enters a Robot.
    scene, scene_nodes = _scene_scan(sup.getRoot(), set(), cfg["scan_solids"],
                                     self_id=self_id)

    _write(out_dir, "roster.json", {
        "t_s": 0.0,
        "frozen": True,
        "synchronization": True,
        "coordinate_system": coordinate_system,
        "world": world_path,
        "bodies": roster_bodies,
        # The two opt-in kinds are reported SEPARATELY and are NOT folded into
        # "bodies": that list is what the adapter turns into the roster the
        # cores count robots in, and a link or a nested prop is not a
        # top-level body of this scene.
        "links": [{k: v for k, v in r.items() if k != "node"}
                  for r in link_records],
        "links_truncated": links_truncated,
        "link_cap": cfg["links"],
        "named_solids": solid_records,
        "solid_tracks_mode": cfg["solid_tracks"],
        # Upstream's Supervisor API has no bounds query (BRINGUP.md 4.1), so
        # every AABB on this arm is COMPUTED. Two channels, deliberately kept
        # apart:
        #
        #   "bodies"       carries NONE, exactly as before. The separate
        #                  agentbench_aabb_prober merges its own measurements
        #                  into that list (task_support.augment_run) and every
        #                  task already graded that way keeps being graded on
        #                  the prober's numbers, from the prober's launch.
        #   "scene_bodies" carries the ones computed here, by the NAME-FREE
        #                  scan, keyed by node key -- the channel a grader that
        #                  matches by geometry needs, because a bounds channel
        #                  that must be handed a name list cannot answer it.
        "bounds_supported": False,
        "bounds_note": ("upstream Webots exposes no Supervisor bounds query, "
                        "so no body in the 'bodies' list carries a "
                        "world-space AABB -- see the agentbench_aabb_prober, "
                        "which measures them in its own launch. World-space "
                        "AABBs COMPUTED by this recorder's name-free scan are "
                        "in 'scene_bodies' and are never merged into "
                        "'bodies'"),
        "scene_bodies": scene,
    })

    # ---- the stepping loop -------------------------------------------------
    duration = float(cfg["duration"])
    contact_steps = int(cfg["contact_steps"])
    times = []
    xyz = {name: [] for name, _, _, _ in tracked}

    contacts_supported = True
    total_observed = 0
    named_pairs = []               # recorded pairs (capped)
    distinct_named = 0
    contact_err = None
    # The robot<->SCENERY channel, in counters of its own so that every number
    # A1 reads keeps meaning exactly what it meant: ``total_observed`` and
    # ``distinct_named`` stay the ROBOT-side witness, and the pairs this
    # channel emits carry ``b_robot: False`` so ``robot_robot_pairs`` -- A1.3's
    # input -- cannot pick them up.
    scenery = list(scene_nodes[:_CONTACT_SCENERY_CAP])
    scenery_truncated = len(scene_nodes) > len(scenery)
    scenery_observed = 0
    scenery_pair_keys = set()      # deduped: one record per (robot, body)
    scenery_err = None

    def sample():
        t = sup.getTime()
        times.append(t)
        for name, node, _kind, _parent in tracked:
            # NaN, never a substituted 0.0: a pose that could not be read is
            # unmeasured, and the reader keeps it non-finite so the cores'
            # floors can see it.
            try:
                p = node.getPosition()
            except Exception:  # noqa: BLE001
                p = None
            xyz[name].append([p[0], p[1], p[2]]
                             if _finite3(p) else [float("nan")] * 3)

    def _same_point(pa, pb):
        return (abs(pa[0] - pb[0]) < _PAIR_TOL
                and abs(pa[1] - pb[1]) < _PAIR_TOL
                and abs(pa[2] - pb[2]) < _PAIR_TOL)

    def scan_contacts(step_idx):
        nonlocal total_observed, distinct_named, contact_err, \
            contacts_supported, scenery_observed, scenery_err
        per_robot = []
        for name, node in robot_nodes:
            try:
                pts = node.getContactPoints(True) or []
            except Exception as exc:  # noqa: BLE001
                contact_err = "getContactPoints failed: %r" % (exc,)
                contacts_supported = False
                return
            per_robot.append((name, [tuple(cp.getPoint()) for cp in pts]))
            total_observed += len(pts)
        # Pair by world point ACROSS distinct robot subtrees. A robot-floor
        # contact appears in one robot's query only and never pairs.
        for ai in range(len(per_robot)):
            a_name, a_pts = per_robot[ai]
            for bi in range(ai + 1, len(per_robot)):
                b_name, b_pts = per_robot[bi]
                for pa in a_pts:
                    for pb in b_pts:
                        if _same_point(pa, pb):
                            distinct_named += 1
                            if len(named_pairs) < _MAX_PAIRS:
                                named_pairs.append({
                                    "a": a_name, "b": b_name,
                                    "a_robot": True, "b_robot": True,
                                    "point": [pa[0], pa[1], pa[2]],
                                    "step": step_idx,
                                })
                            break

        # --- robot <-> SCENERY, the same pairing across a wider participant
        # set. Until 2026-08-09 only Robot nodes were queried, so a contact
        # between a robot and an obstacle or a wall had no second participant
        # and could not be NAMED -- which meant R1.5 ("nothing was hit") could
        # not count one, on an arm where it is the whole point of the task.
        # Whether this works at all rests on something undocumented: a Solid
        # with NO Physics node has no ODE rigid body, so it was an open
        # question whether it answers the query. It does -- measured, R2025a:
        # a Pioneer 3-AT driven into a static 1.6 m box produced 4 contact
        # points on the BOX's own query, at its -x face, on the same step the
        # robot's query listed the same four points
        # (controllers/r1_probe_contacts).
        if not per_robot or not scenery:
            return
        for key, node in scenery:
            try:
                pts = node.getContactPoints(True) or []
            except Exception as exc:  # noqa: BLE001
                # One body refusing the query is not the whole channel
                # failing: record it and carry on, so a single Mesh-hulled
                # prop cannot blind the collision witness for the scene.
                if scenery_err is None:
                    scenery_err = "%s: getContactPoints failed: %r" % (key,
                                                                       exc)
                continue
            if not pts:
                continue
            scenery_observed += len(pts)
            spts = [tuple(cp.getPoint()) for cp in pts]
            for r_name, r_pts in per_robot:
                if (r_name, key) in scenery_pair_keys:
                    continue        # one record per distinct pair, first step
                for pa in r_pts:
                    hit = None
                    for pb in spts:
                        if _same_point(pa, pb):
                            hit = pb
                            break
                    if hit is None:
                        continue
                    scenery_pair_keys.add((r_name, key))
                    if len(named_pairs) < _MAX_PAIRS:
                        named_pairs.append({
                            "a": r_name, "b": key,
                            "a_robot": True, "b_robot": False,
                            "point": [hit[0], hit[1], hit[2]],
                            "step": step_idx,
                        })
                    break

    # First sample is at t=0, while still frozen.
    sample()
    step = 0
    scanned = 0
    whole_run = contact_steps < 0
    while sup.getTime() < duration - 0.5 * dt_s:
        if sup.step(dt_ms) == -1:
            break                  # the world is being killed under us
        step += 1
        sample()
        if whole_run or step <= contact_steps:
            scan_contacts(step)
            scanned += 1

    # ---- artifacts ---------------------------------------------------------
    #
    # ``supported`` answers ONE question for every consumer: can an empty pair
    # list be read as "nothing was hit"? A run that sampled ZERO contact steps
    # never looked, so the answer is no, and saying True there is the C2 defect
    # in miniature -- an assertion passing on evidence that was never
    # collected. It is reported with the reason attached so a reader can tell
    # "this simulator has no contact query" from "this RUN was not asked for
    # one", which are different facts and only one of them is about Webots.
    zero_window = scanned == 0
    _write(out_dir, "contacts.json", {
        "supported": contacts_supported and not zero_window,
        "steps": scanned,
        "window_s": scanned * dt_s,
        "whole_run": bool(whole_run),
        "requested_steps": int(contact_steps),
        "total_observed": total_observed if contacts_supported else None,
        "distinct_named": distinct_named if contacts_supported else None,
        # The robot<->scenery channel's own witness. Separate keys so the two
        # counters above keep meaning "the robot-robot pipeline saw this much",
        # which is what A1.3's falsifier reads.
        "scenery_participants": len(scenery),
        "scenery_truncated": bool(scenery_truncated),
        "scenery_observed": scenery_observed if contacts_supported else None,
        "robot_scenery_pairs": len(scenery_pair_keys),
        "pairs": named_pairs,
        # The parentheses around the conditional are load-bearing: a
        # conditional expression binds LOOSER than ``or``, so without them
        # ``a or b or X if cond else None`` is ``(a or b or X) if cond else
        # None`` and every error is swallowed whenever ``cond`` is false.
        "error": (contact_err or scenery_err
                  or (("no contact window was sampled (--contact-steps=%d), "
                       "so this run contains NO contact evidence: an empty "
                       "pair list here means 'never looked', not 'nothing was "
                       "hit'. This is coverage, not capability -- upstream's "
                       "Node.getContactPoints() works on this arm, including "
                       "on static Solids. Pass --contact-steps=-1 for the "
                       "whole run." % contact_steps)
                      if zero_window else None)),
    })

    _write(out_dir, "trajectory.json", {
        "dt_s": dt_s,
        "basic_time_step_ms": float(dt_ms),
        "world": world_path,
        "recorded_s": (times[-1] - times[0]) if times else 0.0,
        "complete": True,
        # "kind" and "parent" are additive and optional -- a reader that does
        # not know them sees exactly the pre-2026-08-09 document. Together they
        # ARE the sim-neutral identity of a link row: "link of robot X".
        "bodies": [{"name": name, "kind": kind, "parent": parent,
                    "t": times, "xyz": xyz[name]}
                   for name, _node, kind, parent in tracked],
    })

    reached = sup.getTime() >= duration - 0.5 * dt_s
    _write(out_dir, "completion.json", {
        "complete": bool(reached),
        "quit_called": True,
        "recorded_s": (times[-1] - times[0]) if times else 0.0,
        "steps": step,
        "dt_s": dt_s,
        "world": world_path,
        "coordinate_system": coordinate_system,
    })

    # The ONLY clean way an upstream headless run ends. This also makes
    # --log-performance flush (worldClosed()).
    sup.simulationQuit(0)
    sup.step(dt_ms)


if __name__ == "__main__":
    main()
