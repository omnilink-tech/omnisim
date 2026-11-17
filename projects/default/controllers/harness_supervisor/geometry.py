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

"""geometry — per-node world-space bounds for the harness supervisor.

Why this exists
---------------
An agent driving OmniSim over HTTP has to answer one question before it can
aim a camera at anything: *how big is this thing and where is its middle?*
Until now nothing in the harness answered it, so agents guessed a camera pose,
screenshotted, looked, and guessed again. This module computes, for every node
in the scene, an axis-aligned bounding box and a bounding sphere in **world
coordinates**, from the geometry the scene actually contains.

Strategy (and its honesty)
--------------------------
This is a **Python-side geometry walk** over the supervisor's node/field API.
It is not the engine's internal ``OmBoundingSphere`` — that class is not
exposed to controllers — so the numbers here are computed independently:

* **Primitives are exact.** ``Box``, ``Sphere``, ``Cylinder``, ``Capsule``,
  ``Cone``, ``Plane`` and ``ElevationGrid`` produce an exact local AABB, which
  is then transformed by the node's world pose. Rotating an AABB inflates it
  (we take the AABB of the 8 rotated corners), so a rotated primitive's world
  box is conservative — it always *contains* the geometry, never clips it.
* **Explicit coordinate sets are exact** (``IndexedFaceSet`` / ``PointSet`` /
  ``IndexedLineSet`` via their ``coord Coordinate { point [...] }``).
* **``Mesh { url ... }`` is read off disk** (STL binary + ASCII, OBJ, ASCII
  PLY, Collada ``.dae``, glTF ``.gltf`` / ``.glb``) and cached by (path,
  mtime). If a mesh file cannot be parsed it is *skipped* and the node is
  flagged ``exact: false`` with the reason, rather than silently
  under-reporting.

Cost
----
Every supervisor field read is a round-trip to the engine (~10 ms on a busy
one), so a whole-scene walk of a manipulation world is tens of seconds. Node
*shape* — type, which fields exist, geometry extents, child handles — is
cached per node id for the life of the world (the supervisor process is
restarted by every world load, so the cache cannot outlive its world); only
the world transforms are re-read. Callers that know what they want should ask
for it by DEF: the ``scene_bounds`` command walks only the named subtrees.

Both the **visual** geometry (``children`` → ``Shape`` → ``geometry``) and the
**collision** geometry (``boundingObject``) are collected and unioned, so a
robot whose visuals are meshes still gets sane bounds from its collision
primitives.

Bounds are computed **subtree-inclusive** in a single post-order pass, so
``/scene/tree?bounds=1`` costs one walk rather than one walk per node — the
same containment semantics the engine's own bounding sphere has.

Output shape (per node)::

    {"center": [x, y, z],            # AABB centre, world
     "radius": r,                    # half the AABB diagonal (>= true sphere)
     "bbox_min": [x, y, z],
     "bbox_max": [x, y, z],
     "size": [dx, dy, dz],
     "exact": bool,                  # false if any geometry was skipped
     "sources": ["boundingObject", "shape"],
     "skipped": ["Mesh: unreadable url '...'"]}

``radius`` is the half-diagonal of the AABB: always >= the true bounding-sphere
radius, so framing on it never crops the subject.
"""

from __future__ import annotations

import math
import os
import struct
import sys

# --- geometry type tables ---------------------------------------------------

# Node types that carry their own world pose (derived from OmTransform), i.e.
# ones where getPosition()/getOrientation() are authoritative. Anything else
# (Shape, Group, geometry nodes) inherits its parent's frame. Shared with
# observe.py's posed-node classification (POSED_BY_BASE_TYPE) — extend HERE,
# not there, so the two walks cannot drift.
_POSE_TYPES = frozenset({
    "Transform", "Pose", "Solid", "Robot", "Supervisor", "URDFRobot",
    "Charger", "TrackWheel", "Accelerometer", "Altimeter", "Camera",
    "Compass", "Connector", "Display", "DistanceSensor", "Emitter",
    "GPS", "Gyro", "InertialUnit", "LED", "Lidar", "LightSensor",
    "Pen", "RangeFinder", "Radar", "Receiver", "Speaker", "TouchSensor",
    "Propeller", "Track", "Skin", "Fluid", "VacuumGripper",
})

_GEOMETRY_TYPES = frozenset({
    "Box", "Sphere", "Cylinder", "Capsule", "Cone", "Plane",
    "ElevationGrid", "IndexedFaceSet", "IndexedLineSet", "PointSet", "Mesh",
})

# Types that can carry a `scale` field. Probing for `scale` on every node cost
# one supervisor round-trip each and always came back empty for Shapes,
# Groups and geometry.
_SCALABLE_TYPES = _POSE_TYPES | {"Group", "Shape"}

# Fields recursed into, and what they contribute to `sources`.
_CHILD_FIELDS = (
    ("children", "shape"),
    ("endPoint", "shape"),
    ("geometry", "shape"),
    ("boundingObject", "boundingObject"),
    ("appearance", None),  # never contains geometry; listed so we skip it fast
)

_IDENTITY = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

# Field type constants come from the controller library. Resolved once — the
# walk touches several fields per node and re-importing per field was measurable
# on robot-heavy scenes. None means "fall back to the field name" (see the walk).
try:  # pragma: no cover - depends on the controller runtime being present
    from omnisim import Field as _Field
    _MF_NODE = _Field.MF_NODE
except Exception:  # noqa: BLE001
    _MF_NODE = None


# --- small matrix helpers (row-major 3x3 as a flat 9-tuple) -----------------

def mat_mul(a, b):
    """Row-major 3x3 matrix product ``a @ b``."""
    return (
        a[0] * b[0] + a[1] * b[3] + a[2] * b[6],
        a[0] * b[1] + a[1] * b[4] + a[2] * b[7],
        a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
        a[3] * b[0] + a[4] * b[3] + a[5] * b[6],
        a[3] * b[1] + a[4] * b[4] + a[5] * b[7],
        a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
        a[6] * b[0] + a[7] * b[3] + a[8] * b[6],
        a[6] * b[1] + a[7] * b[4] + a[8] * b[7],
        a[6] * b[2] + a[7] * b[5] + a[8] * b[8],
    )


def mat_apply(m, v):
    """Row-major 3x3 matrix times a column vector."""
    return (
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    )


def axis_angle_to_matrix(rot):
    """Axis-angle ``[x, y, z, angle]`` -> row-major 3x3 rotation matrix."""
    ax, ay, az, angle = (float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3]))
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-12 or abs(angle) < 1e-15:
        return _IDENTITY
    ax, ay, az = ax / n, ay / n, az / n
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return (
        t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay,
        t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax,
        t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c,
    )


class Frame:
    """A world transform: origin, rotation (row-major 3x3), non-uniform scale."""

    __slots__ = ("origin", "rot", "scale")

    def __init__(self, origin=(0.0, 0.0, 0.0), rot=_IDENTITY, scale=(1.0, 1.0, 1.0)):
        self.origin = origin
        self.rot = rot
        self.scale = scale

    def to_world(self, p):
        """Map a point expressed in this frame's local coordinates to world."""
        s = (p[0] * self.scale[0], p[1] * self.scale[1], p[2] * self.scale[2])
        r = mat_apply(self.rot, s)
        return (self.origin[0] + r[0], self.origin[1] + r[1], self.origin[2] + r[2])

    def compose(self, translation, rotation, scale):
        """Return the child frame for a local translation/rotation/scale."""
        t = (translation[0] * self.scale[0],
             translation[1] * self.scale[1],
             translation[2] * self.scale[2])
        rt = mat_apply(self.rot, t)
        return Frame(
            (self.origin[0] + rt[0], self.origin[1] + rt[1], self.origin[2] + rt[2]),
            mat_mul(self.rot, axis_angle_to_matrix(rotation)),
            (self.scale[0] * scale[0], self.scale[1] * scale[1], self.scale[2] * scale[2]),
        )


# --- field readers (defensive: the supervisor API raises on odd fields) -----

def _sf_float(node, name, default=None):
    try:
        f = node.getField(name)
        if f is None:
            return default
        v = f.getSFFloat()
        return default if v is None else float(v)
    except Exception:  # noqa: BLE001
        return default


def _sf_int(node, name, default=None):
    try:
        f = node.getField(name)
        if f is None:
            return default
        v = f.getSFInt32()
        return default if v is None else int(v)
    except Exception:  # noqa: BLE001
        return default


def _sf_vec3(node, name, default=None):
    try:
        f = node.getField(name)
        if f is None:
            return default
        v = f.getSFVec3f()
        if v is None or len(v) != 3:
            return default
        return (float(v[0]), float(v[1]), float(v[2]))
    except Exception:  # noqa: BLE001
        return default


def _sf_vec2(node, name, default=None):
    try:
        f = node.getField(name)
        if f is None:
            return default
        v = f.getSFVec2f()
        if v is None or len(v) != 2:
            return default
        return (float(v[0]), float(v[1]))
    except Exception:  # noqa: BLE001
        return default


def _read_vec3(field, default):
    """Read an SFVec3f from an already-resolved field handle."""
    if field is None:
        return default
    try:
        v = field.getSFVec3f()
    except Exception:  # noqa: BLE001
        return default
    if not v or len(v) != 3:
        return default
    return (float(v[0]), float(v[1]), float(v[2]))


def _read_rotation(field, default=(0.0, 0.0, 1.0, 0.0)):
    """Read an SFRotation from an already-resolved field handle."""
    if field is None:
        return default
    try:
        v = field.getSFRotation()
    except Exception:  # noqa: BLE001
        return default
    if not v or len(v) != 4:
        return default
    return (float(v[0]), float(v[1]), float(v[2]), float(v[3]))


def _sf_rotation(node, name, default=(0.0, 0.0, 1.0, 0.0)):
    try:
        f = node.getField(name)
        if f is None:
            return default
        v = f.getSFRotation()
        if v is None or len(v) != 4:
            return default
        return (float(v[0]), float(v[1]), float(v[2]), float(v[3]))
    except Exception:  # noqa: BLE001
        return default


def _sf_string(node, name, default=""):
    try:
        f = node.getField(name)
        if f is None:
            return default
        v = f.getSFString()
        return default if v is None else str(v)
    except Exception:  # noqa: BLE001
        return default


def _mf_strings(node, name):
    out = []
    try:
        f = node.getField(name)
        if f is None:
            return out
        for i in range(f.getCount()):
            v = f.getMFString(i)
            if v:
                out.append(str(v))
    except Exception:  # noqa: BLE001
        pass
    return out


def _mf_floats(node, name, limit=200000):
    out = []
    try:
        f = node.getField(name)
        if f is None:
            return out
        n = min(f.getCount(), limit)
        for i in range(n):
            out.append(float(f.getMFFloat(i)))
    except Exception:  # noqa: BLE001
        pass
    return out


# type name -> "does this type carry a world pose?". PROTO instances report
# their PROTO name from getTypeName(), so the answer needs a getBaseTypeName()
# lookup. That lookup is answered from libController's client-side node struct
# (src/controller/c/supervisor.c, wb_supervisor_node_get_base_type_name) —
# NO engine round-trip; the memo only skips repeated string set-lookups.
_POSE_TYPE_MEMO: dict = {}


def _is_pose_type(node, type_name) -> bool:
    if type_name in _POSE_TYPES:
        return True
    if type_name in _GEOMETRY_TYPES:
        return False
    known = _POSE_TYPE_MEMO.get(type_name)
    if known is not None:
        return known
    try:
        known = node.getBaseTypeName() in _POSE_TYPES
    except Exception:  # noqa: BLE001
        known = False
    _POSE_TYPE_MEMO[type_name] = known
    return known


def _world_pose(node, type_name=None):
    """Return ``(position, rotation_matrix)`` if the node carries a real world
    pose, else ``None``. Non-transform nodes (Shape, Group, geometry) do not,
    and the engine either warns or returns garbage for them — so gate on the
    type name first and still validate what comes back.
    """
    try:
        if type_name is None:
            type_name = node.getTypeName()
    except Exception:  # noqa: BLE001
        return None
    if not _is_pose_type(node, type_name):
        return None
    try:
        pos = node.getPosition()
        rot = node.getOrientation()
    except Exception:  # noqa: BLE001
        return None
    if not pos or len(pos) != 3 or not rot or len(rot) != 9:
        return None
    if not all(math.isfinite(v) for v in pos) or not all(math.isfinite(v) for v in rot):
        return None
    return (float(pos[0]), float(pos[1]), float(pos[2])), tuple(float(v) for v in rot)


# --- mesh files -------------------------------------------------------------

_MESH_CACHE: dict = {}
# Refuse to slurp pathological assets; a bbox is not worth a 500 MB read.
_MESH_MAX_BYTES = 128 * 1024 * 1024


def _mesh_file_bounds(path: str):
    """Local-space AABB of a mesh file, or None. Cached by (path, mtime, size)."""
    try:
        st = os.stat(path)
    except OSError:
        return None
    key = (path, st.st_mtime_ns, st.st_size)
    if key in _MESH_CACHE:
        return _MESH_CACHE[key]
    result = None
    if st.st_size <= _MESH_MAX_BYTES:
        ext = os.path.splitext(path)[1].lower()
        try:
            with open(path, "rb") as fh:
                raw = fh.read()
            if ext == ".stl":
                result = _bounds_stl(raw)
            elif ext == ".obj":
                result = _bounds_obj(raw)
            elif ext == ".ply":
                result = _bounds_ply(raw)
            elif ext == ".dae":
                result = _bounds_dae(raw)
            elif ext in (".glb", ".gltf"):
                result = _bounds_gltf(raw)
        except Exception:  # noqa: BLE001
            result = None
    _MESH_CACHE[key] = result
    return result


def _accumulate(points):
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    n = 0
    for p in points:
        for i in range(3):
            v = p[i]
            if v < lo[i]:
                lo[i] = v
            if v > hi[i]:
                hi[i] = v
        n += 1
    if n == 0 or not all(math.isfinite(v) for v in lo + hi):
        return None
    return (tuple(lo), tuple(hi))


def _bounds_stl(raw: bytes):
    head = raw[:5].lower()
    if head == b"solid" and b"facet" in raw[:2048].lower():
        pts = []
        for line in raw.decode("ascii", errors="replace").splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[0] == "vertex":
                try:
                    pts.append((float(parts[1]), float(parts[2]), float(parts[3])))
                except ValueError:
                    pass
        return _accumulate(pts)
    if len(raw) < 84:
        return None
    (count,) = struct.unpack_from("<I", raw, 80)
    if 84 + count * 50 > len(raw):
        count = max(0, (len(raw) - 84) // 50)

    def _tris():
        off = 84
        for _ in range(count):
            vals = struct.unpack_from("<12f", raw, off)
            yield vals[3:6]
            yield vals[6:9]
            yield vals[9:12]
            off += 50
    return _accumulate(_tris())


def _bounds_obj(raw: bytes):
    pts = []
    for line in raw.decode("utf-8", errors="replace").splitlines():
        if not line.startswith("v "):
            continue
        parts = line.split()
        if len(parts) >= 4:
            try:
                pts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError:
                pass
    return _accumulate(pts)


def _bounds_ply(raw: bytes):
    end = raw.find(b"end_header")
    if end < 0:
        return None
    header = raw[:end].decode("ascii", errors="replace")
    if "format ascii" not in header:
        return None  # binary PLY: not worth a full parser for a bbox
    body = raw[end:].split(b"\n", 1)
    if len(body) < 2:
        return None
    n_vertex = 0
    for line in header.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[0] == "element" and parts[1] == "vertex":
            try:
                n_vertex = int(parts[2])
            except ValueError:
                n_vertex = 0
    pts = []
    for line in body[1].decode("ascii", errors="replace").splitlines()[:n_vertex]:
        parts = line.split()
        if len(parts) >= 3:
            try:
                pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except ValueError:
                pass
    return _accumulate(pts)


def _bounds_dae(raw: bytes):
    """Collada: union the POSITION sources referenced by every <mesh>.

    Walks ``<mesh><vertices><input semantic="POSITION" source="#id">`` back to
    ``<source id="id"><float_array>``, which is the one path every exporter
    agrees on. Node-level transforms inside the ``<library_visual_scenes>`` are
    NOT applied — Webots loads the mesh geometry itself — so this is the same
    space the engine renders it in.
    """
    import xml.etree.ElementTree as ET
    root = ET.fromstring(raw.decode("utf-8", errors="replace"))

    def _tag(e):
        return e.tag.rsplit("}", 1)[-1]

    pts = []
    for mesh in root.iter():
        if _tag(mesh) != "mesh":
            continue
        sources = {}
        for src in mesh:
            if _tag(src) != "source":
                continue
            sid = src.get("id")
            for child in src:
                if _tag(child) == "float_array" and sid:
                    sources[sid] = child.text or ""
        wanted = []
        for verts in mesh:
            if _tag(verts) != "vertices":
                continue
            for inp in verts:
                if _tag(inp) == "input" and inp.get("semantic") == "POSITION":
                    ref = (inp.get("source") or "").lstrip("#")
                    if ref:
                        wanted.append(ref)
        if not wanted:
            wanted = list(sources)
        for ref in wanted:
            text = sources.get(ref)
            if not text:
                continue
            vals = text.split()
            for i in range(0, len(vals) - 2, 3):
                try:
                    pts.append((float(vals[i]), float(vals[i + 1]), float(vals[i + 2])))
                except ValueError:
                    break
    return _accumulate(pts)


def _bounds_gltf(raw: bytes):
    """glTF / GLB: read the POSITION accessors' declared ``min``/``max``.

    The spec REQUIRES ``min``/``max`` on the POSITION accessor, so the bbox is
    available without decoding a single vertex — the JSON chunk is enough.
    """
    if raw[:4] == b"glTF":
        # GLB container: 12-byte header, then chunks (length, type, data).
        if len(raw) < 20:
            return None
        offset = 12
        payload = None
        while offset + 8 <= len(raw):
            (chunk_len, chunk_type) = struct.unpack_from("<I4s", raw, offset)
            body = raw[offset + 8: offset + 8 + chunk_len]
            if chunk_type == b"JSON":
                payload = body
                break
            offset += 8 + chunk_len + ((4 - chunk_len % 4) % 4)
        if payload is None:
            return None
    else:
        payload = raw
    import json as _json
    doc = _json.loads(payload.decode("utf-8", errors="replace"))
    accessors = doc.get("accessors") or []
    wanted = set()
    for mesh in doc.get("meshes") or []:
        for prim in mesh.get("primitives") or []:
            idx = (prim.get("attributes") or {}).get("POSITION")
            if isinstance(idx, int):
                wanted.add(idx)
    if not wanted:
        wanted = set(range(len(accessors)))
    pts = []
    for idx in wanted:
        if not (0 <= idx < len(accessors)):
            continue
        acc = accessors[idx]
        lo, hi = acc.get("min"), acc.get("max")
        if isinstance(lo, list) and isinstance(hi, list) and \
                len(lo) >= 3 and len(hi) >= 3:
            pts.append(tuple(float(v) for v in lo[:3]))
            pts.append(tuple(float(v) for v in hi[:3]))
    return _accumulate(pts)


def _resolve_mesh_url(url: str, search_roots) -> str | None:
    if not url:
        return None
    u = url.replace("\\", "/")
    if u.startswith("omnisim://") or u.startswith("webots://"):
        rel = u.split("://", 1)[1]
        for root in search_roots:
            cand = os.path.join(root, rel)
            if os.path.isfile(cand):
                return cand
        return None
    if os.path.isabs(u) and os.path.isfile(u):
        return u
    for root in search_roots:
        cand = os.path.normpath(os.path.join(root, u))
        if os.path.isfile(cand):
            return cand
    return None


# --- local geometry AABBs ---------------------------------------------------

def local_geometry_bounds(node, type_name: str, search_roots):
    """Local-space AABB of one geometry node.

    Returns ``(lo, hi)`` or ``(None, reason)`` when the geometry cannot be
    measured. OmniSim inherits Webots' post-R2022b geometry conventions:

    * ``Box.size`` is the full x/y/z extent centred on the origin.
    * ``Cylinder`` / ``Capsule`` / ``Cone`` are **Z-axis aligned**: ``height``
      runs along local +Z and the radius spans XY. Do not be misled by
      ``OmCylinder::rescale`` / ``OmCone::scaledHeight``, which still multiply
      height by ``scale.y()`` — those are stale leftovers from the pre-R2022b
      Y-up convention. The authoritative code is ``scaledHeight()`` (uses
      ``absoluteScale().z()``), the ODE geom setup, ``computeFrictionDirection``
      (tests ``localNormal[2]`` for the caps) and ``recomputeBoundingSphere``
      (offsets the cap sphere along ``z``). Getting this wrong inflates every
      URDF wheel's collision hull by (radius - halfHeight) on the wrong axis —
      it made the Husky measure 0.885 m wide instead of 0.685 m.
    * ``Plane.size`` lies in XY; ``ElevationGrid`` spans x/y with heights
      along z.
    """
    if type_name == "Box":
        s = _sf_vec3(node, "size", (2.0, 2.0, 2.0))
        h = (abs(s[0]) / 2.0, abs(s[1]) / 2.0, abs(s[2]) / 2.0)
        return ((-h[0], -h[1], -h[2]), (h[0], h[1], h[2]))
    if type_name == "Sphere":
        r = abs(_sf_float(node, "radius", 1.0) or 1.0)
        return ((-r, -r, -r), (r, r, r))
    if type_name in ("Cylinder", "Capsule"):
        r = abs(_sf_float(node, "radius", 1.0) or 1.0)
        h = abs(_sf_float(node, "height", 2.0) or 2.0) / 2.0
        if type_name == "Capsule":
            h += r  # hemispherical caps sit beyond the cylindrical section
        return ((-r, -r, -h), (r, r, h))
    if type_name == "Cone":
        r = abs(_sf_float(node, "bottomRadius", 1.0) or 1.0)
        h = abs(_sf_float(node, "height", 2.0) or 2.0) / 2.0
        return ((-r, -r, -h), (r, r, h))
    if type_name == "Plane":
        s = _sf_vec2(node, "size", (1.0, 1.0))
        return ((-abs(s[0]) / 2.0, -abs(s[1]) / 2.0, 0.0),
                (abs(s[0]) / 2.0, abs(s[1]) / 2.0, 0.0))
    if type_name == "ElevationGrid":
        nx = _sf_int(node, "xDimension", 0) or 0
        ny = _sf_int(node, "yDimension", 0) or 0
        sx = _sf_float(node, "xSpacing", 1.0) or 1.0
        sy = _sf_float(node, "ySpacing", 1.0) or 1.0
        heights = _mf_floats(node, "height")
        zlo = min(heights) if heights else 0.0
        zhi = max(heights) if heights else 0.0
        thickness = _sf_float(node, "thickness", 0.0) or 0.0
        return ((0.0, 0.0, zlo - thickness),
                (max(0, nx - 1) * sx, max(0, ny - 1) * sy, zhi))
    if type_name in ("IndexedFaceSet", "IndexedLineSet", "PointSet"):
        try:
            coord_field = node.getField("coord")
            coord = coord_field.getSFNode() if coord_field is not None else None
        except Exception:  # noqa: BLE001
            coord = None
        if coord is None:
            return (None, f"{type_name}: no coord node")
        pts = []
        try:
            pf = coord.getField("point")
            n = pf.getCount() if pf is not None else 0
            for i in range(min(n, 500000)):
                v = pf.getMFVec3f(i)
                if v and len(v) == 3:
                    pts.append((float(v[0]), float(v[1]), float(v[2])))
        except Exception:  # noqa: BLE001
            pass
        box = _accumulate(pts)
        if box is None:
            return (None, f"{type_name}: empty coord point array")
        return box
    if type_name == "Mesh":
        urls = _mf_strings(node, "url") or ([_sf_string(node, "url")] if _sf_string(node, "url") else [])
        if not urls:
            return (None, "Mesh: no url")
        lo = [float("inf")] * 3
        hi = [float("-inf")] * 3
        found = False
        for url in urls:
            path = _resolve_mesh_url(url, search_roots)
            if path is None:
                continue
            box = _mesh_file_bounds(path)
            if box is None:
                continue
            found = True
            for i in range(3):
                lo[i] = min(lo[i], box[0][i])
                hi[i] = max(hi[i], box[1][i])
        if not found:
            return (None, f"Mesh: unreadable url {urls[0]!r}")
        return (tuple(lo), tuple(hi))
    return (None, f"{type_name}: unsupported geometry")


# --- the walk ---------------------------------------------------------------

class _Accum:
    """Mutable AABB accumulator for one node's subtree."""

    __slots__ = ("lo", "hi", "sources", "skipped", "count")

    def __init__(self):
        self.lo = [float("inf")] * 3
        self.hi = [float("-inf")] * 3
        self.sources = set()
        self.skipped = []
        self.count = 0

    def add_point(self, p):
        for i in range(3):
            if p[i] < self.lo[i]:
                self.lo[i] = p[i]
            if p[i] > self.hi[i]:
                self.hi[i] = p[i]
        self.count += 1

    def merge(self, other):
        if other.count:
            for i in range(3):
                self.lo[i] = min(self.lo[i], other.lo[i])
                self.hi[i] = max(self.hi[i], other.hi[i])
            self.count += other.count
        self.sources |= other.sources
        for s in other.skipped:
            if s not in self.skipped and len(self.skipped) < 16:
                self.skipped.append(s)

    def note_skip(self, reason):
        if reason not in self.skipped and len(self.skipped) < 16:
            self.skipped.append(reason)

    def result(self):
        if not self.count:
            return None
        lo, hi = self.lo, self.hi
        center = [(lo[i] + hi[i]) / 2.0 for i in range(3)]
        size = [hi[i] - lo[i] for i in range(3)]
        radius = 0.5 * math.sqrt(sum(s * s for s in size))
        return {
            "center": [round(c, 6) for c in center],
            "radius": round(radius, 6),
            "bbox_min": [round(v, 6) for v in lo],
            "bbox_max": [round(v, 6) for v in hi],
            "size": [round(s, 6) for s in size],
            "exact": not self.skipped,
            "sources": sorted(self.sources),
            "skipped": list(self.skipped),
        }


def default_search_roots():
    """Directories mesh urls are resolved against, most specific first."""
    roots = []
    home = os.environ.get("OMNISIM_HOME")
    if home:
        roots.append(home)
    roots.append(os.getcwd())
    return roots


# Per-world caches. Every supervisor field/type read is a TCP round-trip to
# the engine, and a full scene walk does thousands of them — a 149-node
# manipulation world measured 41 s uncached. Node *shape* (its type, which
# fields it has, and the local extent of its geometry) does not change while a
# world is loaded, so it is cached by node id; only the world transforms are
# re-read each walk, which is what actually moves. The supervisor process is
# restarted by every world load, so the caches cannot outlive their world.
_TYPE_CACHE: dict = {}
_FIELD_CACHE: dict = {}
_FIELD_TYPE_CACHE: dict = {}
_CHILDREN_CACHE: dict = {}
_LOCAL_GEOM_CACHE: dict = {}


def reset_caches() -> None:
    """Drop the per-world caches (exposed for tests and manual invalidation)."""
    _TYPE_CACHE.clear()
    _FIELD_CACHE.clear()
    _FIELD_TYPE_CACHE.clear()
    _CHILDREN_CACHE.clear()
    _LOCAL_GEOM_CACHE.clear()
    _POSE_TYPE_MEMO.clear()
    _MESH_CACHE.clear()


def _node_id(node):
    try:
        return node.getId()
    except Exception:  # noqa: BLE001
        return None


def _cached_type(node, node_id):
    if node_id is not None and node_id in _TYPE_CACHE:
        return _TYPE_CACHE[node_id]
    try:
        type_name = node.getTypeName()
    except Exception:  # noqa: BLE001
        return None
    if node_id is not None:
        _TYPE_CACHE[node_id] = type_name
    return type_name


def _cached_field(node, node_id, name):
    key = (node_id, name)
    if node_id is not None and key in _FIELD_CACHE:
        return _FIELD_CACHE[key]
    try:
        field = node.getField(name)
    except Exception:  # noqa: BLE001
        field = None
    if node_id is not None:
        _FIELD_CACHE[key] = field
    return field


class BoundsWalker:
    """One post-order walk of the scene producing subtree AABBs per node.

    ``bounds_by_id`` maps ``node.getId()`` to the result dict, so both
    ``scene_tree`` (every node) and ``scene_node`` (one DEF) read out of the
    same pass. The walk is O(nodes + geometry), not O(nodes^2).
    """

    def __init__(self, search_roots=None, max_depth=64):
        self.search_roots = list(search_roots or default_search_roots())
        self.max_depth = max_depth
        self.bounds_by_id: dict = {}
        self.errors: list[str] = []

    def walk(self, node, frame: Frame | None = None, depth: int = 0) -> _Accum:
        acc = _Accum()
        if node is None or depth > self.max_depth:
            return acc
        node_id = _node_id(node)
        type_name = _cached_type(node, node_id)
        if type_name is None:
            return acc

        # Establish this node's frame. A transform-derived node knows its own
        # world pose (physics included); everything else composes locally off
        # the parent frame.
        if frame is None:
            frame = Frame()
        pose = _world_pose(node, type_name)
        if type_name in _SCALABLE_TYPES:
            scale = _read_vec3(_cached_field(node, node_id, "scale"),
                               (1.0, 1.0, 1.0))
        else:
            scale = (1.0, 1.0, 1.0)
        if pose is not None:
            local = Frame(pose[0], pose[1],
                          (frame.scale[0] * scale[0],
                           frame.scale[1] * scale[1],
                           frame.scale[2] * scale[2]))
        else:
            translation = _read_vec3(_cached_field(node, node_id, "translation"),
                                     (0.0, 0.0, 0.0))
            rotation = _read_rotation(_cached_field(node, node_id, "rotation"))
            local = frame.compose(translation, rotation, scale)

        if type_name in _GEOMETRY_TYPES:
            # A geometry node's LOCAL extent is fixed for the life of the
            # world; only its world transform moves. Cache the expensive part
            # (field reads + mesh-file parsing) and re-transform each walk.
            if node_id is not None and node_id in _LOCAL_GEOM_CACHE:
                box = _LOCAL_GEOM_CACHE[node_id]
            else:
                box = local_geometry_bounds(node, type_name, self.search_roots)
                if node_id is not None:
                    _LOCAL_GEOM_CACHE[node_id] = box
            if box[0] is None:
                acc.note_skip(box[1])
            else:
                lo, hi = box
                for cx in (lo[0], hi[0]):
                    for cy in (lo[1], hi[1]):
                        for cz in (lo[2], hi[2]):
                            acc.add_point(local.to_world((cx, cy, cz)))
            # Geometry nodes never carry children/boundingObject; skipping the
            # probes saves four round-trips on the most numerous node type.
            result = acc.result()
            if result is not None and node_id is not None:
                self.bounds_by_id[node_id] = result
            return acc

        for field_name, source in _CHILD_FIELDS:
            if source is None:
                continue
            field = _cached_field(node, node_id, field_name)
            if field is None:
                continue
            children = []
            # A field's TYPE is immutable; its contents are not. Cache the
            # type, re-read the contents.
            key = (node_id, field_name)
            if key in _FIELD_TYPE_CACHE:
                ftype = _FIELD_TYPE_CACHE[key]
            else:
                try:
                    ftype = field.getType()
                except Exception:  # noqa: BLE001
                    continue
                _FIELD_TYPE_CACHE[key] = ftype
            is_mf = (ftype == _MF_NODE) if _MF_NODE is not None \
                else (field_name == "children")
            try:
                if is_mf:
                    # Node handles are stable while the scene is; the cheap
                    # `getCount()` is enough to detect an add/remove (debris,
                    # supervisor insertions) and re-fetch. This turns n+1
                    # round-trips per multi-child field into 1.
                    count = field.getCount()
                    cached = _CHILDREN_CACHE.get(key)
                    if cached is not None and cached[0] == count:
                        children = cached[1]
                    else:
                        children = [field.getMFNode(i) for i in range(count)]
                        if node_id is not None:
                            _CHILDREN_CACHE[key] = (count, children)
                else:
                    # SFNode contents are NOT cached: a boundingObject or
                    # geometry can be swapped wholesale (the damage tracker
                    # does exactly that) and a stale handle would silently
                    # report the old shape.
                    children = [field.getSFNode()]
            except Exception:  # noqa: BLE001
                continue
            for child in children:
                if child is None:
                    continue
                sub = self.walk(child, local, depth + 1)
                if sub.count:
                    sub.sources.add(source)
                acc.merge(sub)

        result = acc.result()
        if result is not None and node_id is not None:
            self.bounds_by_id[node_id] = result
        return acc


def bounds_for_subtree(node, search_roots=None) -> dict | None:
    """Subtree world bounds for a single node (used by ``scene_node``)."""
    walker = BoundsWalker(search_roots)
    walker.walk(node)
    try:
        return walker.bounds_by_id.get(node.getId())
    except Exception:  # noqa: BLE001
        return None


def bounds_index(root, search_roots=None) -> dict:
    """Map node id -> bounds for the whole scene, in one pass."""
    walker = BoundsWalker(search_roots)
    walker.walk(root)
    return walker.bounds_by_id


# --- exact cross-check: invert the engine's own moveViewpoint fit ------------

def probe_bounding_sphere(supervisor, viewpoint, node, aspect: float,
                          settle_steps: int = 90, basic_step_ms: int = 32):
    """Recover the engine's TRUE bounding sphere for ``node`` by inverting
    ``OmViewpoint::moveViewpointToObject``.

    The engine moves the camera to ``center - direction * distance`` with::

        distance = 1.05 * radius / (sin(fov/2) * min(aspect, 1/aspect))

    keeping the current orientation. One probe gives 3 equations for 4
    unknowns (centre + radius), so we probe from **two different
    orientations**: the distance is identical in both, so

        distance = |p1 - p2| / |d2 - d1|,   centre = p1 + d1 * distance

    and the radius falls straight out of the formula. The camera pose is
    saved and restored, so the probe is side-effect free apart from the sim
    steps needed to let the engine's eased camera animation finish
    (``ANIMATION_DURATION`` is 1 s of wall clock, hence ``settle_steps``).

    This is SLOW (~2-3 s) and exists as a correctness oracle for the Python
    geometry walk, not as the default path. Returns a dict with ``center``,
    ``radius``, ``distance`` and ``residual`` (how far the two probes'
    recovered centres disagree — a sanity check on the inversion), or a dict
    with ``error``.
    """
    import time as _time

    pos_field = viewpoint.getField("position")
    ori_field = viewpoint.getField("orientation")
    fov_field = viewpoint.getField("fieldOfView")
    if pos_field is None or ori_field is None or fov_field is None:
        return {"error": "Viewpoint is missing position/orientation/fieldOfView"}
    saved_pos = list(pos_field.getSFVec3f())
    saved_ori = list(ori_field.getSFRotation())
    fov = float(fov_field.getSFFloat())
    tight = aspect if aspect <= 1.0 else 1.0 / aspect
    denom = math.sin(fov / 2.0) * tight
    if denom <= 1e-9:
        return {"error": f"degenerate fov/aspect (fov={fov}, aspect={aspect})"}

    # Two probe orientations, deliberately far apart so |d2 - d1| is large.
    # The engine's view direction is `orientation.toQuaternion() * (1,0,0)`,
    # i.e. column 0 of the rotation matrix — derived here rather than
    # hard-coded, because getting it wrong silently poisons the inversion.
    probe_rotations = [
        [0.0, 0.0, 1.0, 0.0],            # identity -> looks along +X
        [0.0, 1.0, 0.0, -math.pi / 2.0],  # -90 deg about +Y -> looks along +Z
    ]
    observed = []

    def _flush(n=2):
        """Supervisor field writes are queued and only applied on the next
        sim step, so every set must be followed by a step before the engine
        acts on it. Skipping this is what made the first probe read back the
        PREVIOUS orientation's fit."""
        for _ in range(n):
            if supervisor.step(basic_step_ms) == -1:
                return False
        return True

    def _settle(timeout_s=6.0):
        """Step until the Viewpoint position stops changing — the engine
        animates the move over ANIMATION_DURATION (1 s of wall clock)."""
        deadline = _time.time() + timeout_s
        last = None
        stable = 0
        while _time.time() < deadline and stable < 6:
            if supervisor.step(basic_step_ms) == -1:
                break
            cur = tuple(round(v, 9) for v in pos_field.getSFVec3f())
            if cur == last:
                stable += 1
            else:
                stable = 0
                last = cur
        return list(pos_field.getSFVec3f())

    try:
        for rotation in probe_rotations:
            ori_field.setSFRotation([float(v) for v in rotation])
            if not _flush():
                break
            direction = mat_apply(axis_angle_to_matrix(rotation), (1.0, 0.0, 0.0))
            node.moveViewpoint()
            if not _flush(1):
                break
            observed.append((_settle(), direction))
    finally:
        ori_field.setSFRotation([float(v) for v in saved_ori])
        pos_field.setSFVec3f([float(v) for v in saved_pos])
        for _ in range(settle_steps):
            if supervisor.step(basic_step_ms) == -1:
                break
        # Re-assert after the settle window: the engine's eased camera
        # animation keeps writing to `position` until it finishes, and would
        # otherwise overwrite the restore.
        ori_field.setSFRotation([float(v) for v in saved_ori])
        pos_field.setSFVec3f([float(v) for v in saved_pos])

    if len(observed) != 2:
        return {"error": "probe did not complete"}
    (p1, d1), (p2, d2) = observed
    dd = [d2[i] - d1[i] for i in range(3)]
    dd_norm = math.sqrt(sum(v * v for v in dd))
    dp = [p1[i] - p2[i] for i in range(3)]
    dp_norm = math.sqrt(sum(v * v for v in dp))
    if dd_norm < 1e-6:
        return {"error": "probe directions were parallel"}
    # p1 - p2 = (d2 - d1) * distance  ->  distance = |p1-p2| / |d2-d1|
    distance = dp_norm / dd_norm
    c1 = [p1[i] + d1[i] * distance for i in range(3)]
    c2 = [p2[i] + d2[i] * distance for i in range(3)]
    residual = math.sqrt(sum((c1[i] - c2[i]) ** 2 for i in range(3)))
    center = [(c1[i] + c2[i]) / 2.0 for i in range(3)]
    radius = distance * denom / 1.05
    return {
        "center": [round(v, 6) for v in center],
        "radius": round(radius, 6),
        "distance": round(distance, 6),
        "residual": round(residual, 6),
        "fov": fov,
        "aspect": aspect,
        "probe_positions": [p1, p2],
        "method": "moveViewpoint inversion (engine OmBoundingSphere)",
    }


if __name__ == "__main__":  # pragma: no cover - manual smoke test
    print("geometry.py is a supervisor-side module; import it, don't run it.",
          file=sys.stderr)
