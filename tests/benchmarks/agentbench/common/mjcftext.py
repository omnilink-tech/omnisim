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

"""The MuJoCo arm's half of the same job ``worldtext`` does for ``.wbt``.

R1's grade-time placement has to move obstacles inside the deliverable the
agent authored, whatever simulator that deliverable is for. On MuJoCo the
deliverable is an MJCF ``.xml`` (plus the driver that steps it), the obstacles
are ``<geom type="box">`` elements, and the field that moves one is ``pos``.

**Why a tag scan and not ``ElementTree``.** The placer edits a file the agent
wrote and hands it straight to the grader. Re-serialising an ElementTree
rewrites attribute order, drops the XML declaration, collapses whitespace and
(by default) deletes every comment -- so the graded artifact would differ from
the delivered one in a hundred ways that have nothing to do with the five
numbers we changed, and a forensic reader could no longer diff them. This scan
touches exactly the ``pos`` attributes it moves and leaves every other byte
alone. It is not a general XML parser and does not pretend to be: MJCF is
attribute-only markup with no mixed content, which is the property that makes
the scan sufficient.

The geometry conventions that matter here, both of them the ones an author
gets wrong:

  * MJCF ``size`` on a box is a **half** extent; ``obstacles.json`` publishes
    **full** extents. Everything this module returns is a full extent, so the
    conversion happens once, here.
  * ``<compiler angle="...">`` defaults to **degrees**, so ``euler`` and
    ``axisangle`` are read through whatever the file declares rather than
    assumed to be radians.
"""

from __future__ import annotations

import math
import re

from agentbench.common.worldtext import (IDENTITY, Frame, abs_apply,  # noqa: F401
                                         axis_angle_matrix,
                                         bounds_of_boxes, mat_apply, mat_mul,
                                         mat_transpose)

_COMMENT = re.compile(r"<!--.*?-->", re.S)
_ATTR = re.compile(r"([\w.:-]+)\s*=\s*(\"[^\"]*\"|'[^']*')")
_TAG = re.compile(
    r"<\s*(?P<close>/?)\s*(?P<tag>[A-Za-z_][\w.:-]*)"
    r"(?P<attrs>(?:\s+[\w.:-]+\s*=\s*(?:\"[^\"]*\"|'[^']*'))*)"
    r"\s*(?P<empty>/?)\s*>")

#: Elements whose presence on a ``<body>`` means that body MOVES -- it is a
#: robot link (or a free body), never a static obstacle. Its geometry is
#: excluded from placement for the same reason ``Robot`` subtrees are on the
#: ``.wbt`` side.
_ARTICULATION = frozenset({"joint", "freejoint"})

#: Orientation attributes this module can read. ``zaxis`` is deliberately NOT
#: among them: rather than guess, an element carrying one is reported as
#: unmeasurable, and the placer refuses to move it (a wrongly-oriented AABB
#: would place an obstacle somewhere nobody asked for).
_ORIENT = ("quat", "axisangle", "euler", "xyaxes")


def mask_text(text):
    """A SAME-LENGTH copy with comment bodies blanked, so offsets survive."""
    return _COMMENT.sub(lambda m: " " * len(m.group(0)), text)


class Element:
    """One XML element, by offset in the source."""

    __slots__ = ("tag", "attrs", "attr_end", "start", "end", "parent",
                 "children")

    def __init__(self, tag, attrs, attr_end, start, end, parent):
        self.tag = tag
        self.attrs = attrs
        #: index just after the tag name / attribute list, where a new
        #: attribute can be inserted
        self.attr_end = attr_end
        self.start = start
        self.end = end
        self.parent = parent
        self.children = []

    def ancestors(self):
        node = self.parent
        while node is not None:
            yield node
            node = node.parent

    def __repr__(self):                                  # pragma: no cover
        return "<mjcf %s %s>" % (self.tag, self.attrs.get("name", ""))


def parse(text):
    """``(root_children, all_elements)`` -- a shallow element tree.

    Never raises on malformed markup: an unbalanced close tag is ignored and
    the scan continues, because the caller's contract is "find what you can
    measure and say what you could not", not "validate the agent's XML".
    """
    masked = mask_text(text)
    roots, everything, stack = [], [], []
    for m in _TAG.finditer(masked):
        tag = m.group("tag")
        if m.group("close"):
            while stack:
                done = stack.pop()
                if done.tag == tag:
                    break
            continue
        attrs = {}
        attr_end = m.start() + 1 + len(tag)
        for a in _ATTR.finditer(m.group("attrs") or ""):
            attrs[a.group(1)] = a.group(2)[1:-1]
            attr_end = m.start("attrs") + a.end()
        el = Element(tag, attrs, attr_end, m.start(), m.end(),
                     stack[-1] if stack else None)
        everything.append(el)
        if stack:
            stack[-1].children.append(el)
        else:
            roots.append(el)
        if not m.group("empty"):
            stack.append(el)
    return roots, everything


def _floats(el, key, count=None):
    raw = el.attrs.get(key)
    if raw is None:
        return None
    try:
        vals = [float(v) for v in raw.replace(",", " ").split()]
    except ValueError:
        return None
    if count is not None and len(vals) != count:
        return None
    return vals


def compiler_settings(elements):
    """``{"angle_scale": <to radians>, "eulerseq": "xyz"}`` from ``<compiler>``.

    MuJoCo's own defaults (degrees, ``xyz``) apply when the file is silent.
    """
    angle, seq = "degree", "xyz"
    for el in elements:
        if el.tag == "compiler":
            angle = el.attrs.get("angle", angle)
            seq = el.attrs.get("eulerseq", seq)
    scale = 1.0 if str(angle).lower().startswith("rad") else math.pi / 180.0
    return {"angle_scale": scale, "eulerseq": seq}


def _quat_matrix(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return IDENTITY
    w, x, y, z = w / n, x / n, y / n, z / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def _axis_matrix(axis, angle):
    return axis_angle_matrix(axis[0], axis[1], axis[2], angle)


def _euler_matrix(a, seq, scale):
    m = IDENTITY
    axes = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}
    for ch, ang in zip(seq.lower(), a):
        m = mat_mul(m, _axis_matrix(axes.get(ch, (0.0, 0.0, 1.0)),
                                    ang * scale))
    return m


def _normalise(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v) if n > 1e-12 else None


def orientation(el, cfg):
    """``(matrix, unmeasurable)`` for one element's own rotation."""
    q = _floats(el, "quat", 4)
    if q is not None:
        return _quat_matrix(*q), False
    aa = _floats(el, "axisangle", 4)
    if aa is not None:
        return _axis_matrix(aa[:3], aa[3] * cfg["angle_scale"]), False
    eu = _floats(el, "euler", 3)
    if eu is not None:
        return _euler_matrix(eu, cfg["eulerseq"], cfg["angle_scale"]), False
    xy = _floats(el, "xyaxes", 6)
    if xy is not None:
        x = _normalise(xy[:3])
        y = _normalise(xy[3:])
        if x and y:
            z = (x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2],
                 x[0] * y[1] - x[1] * y[0])
            return tuple((x[i], y[i], z[i]) for i in range(3)), False
    if "zaxis" in el.attrs or "xyaxes" in el.attrs:
        # Present but unreadable (a degenerate ``xyaxes``, or the one spelling
        # this module does not implement). Reported, never guessed.
        return IDENTITY, True
    return IDENTITY, False


class Body:
    """A movable box with world-space bounds; duck-types ``evidence.Body``.

    The same shape ``worldtext.Body`` has, so ``r1_core.match_spec_obstacles``
    matches an MJCF box exactly as it matches a ``.wbt`` Solid and a recorded
    scene body -- one matcher, three spellings of the same geometry.
    """

    __slots__ = ("name", "body_id", "kind", "element", "lo", "hi",
                 "parent_rot", "depth")

    has_aabb = True
    robot_class = False
    member_of = None

    def __init__(self, name, body_id, kind, element, lo, hi, parent_rot,
                 depth):
        self.name = name
        self.body_id = body_id
        self.kind = kind
        self.element = element
        self.lo = lo
        self.hi = hi
        self.parent_rot = parent_rot
        self.depth = depth

    @property
    def aabb(self):
        return (self.lo, self.hi)

    @property
    def top_z(self):
        return self.hi[2]

    @property
    def centre(self):
        return tuple((self.lo[i] + self.hi[i]) / 2.0 for i in range(3))

    @property
    def size(self):
        return tuple(self.hi[i] - self.lo[i] for i in range(3))

    def __repr__(self):                                  # pragma: no cover
        return ("<mjcf Body %s centre=(%.3f, %.3f) size=(%.3f, %.3f)>"
                % (self.name or "-", self.centre[0], self.centre[1],
                   self.size[0], self.size[1]))


def scan_bodies(text, *, max_bodies=400):
    """Every static box in an MJCF model, with WORLD-space bounds.

    "Static" means: not inside a ``<body>`` that carries a joint or a
    freejoint. That is the robot exclusion, and it is structural rather than
    name-based -- the R1 rover is excluded because it has a ``<freejoint/>``,
    not because it is called ``rover``.

    Boxes whose own (or whose ancestors') orientation this module cannot read
    are omitted, so a caller can never move one to the wrong place; the
    omission surfaces as a placement failure rather than as a silent success.
    """
    roots, everything = parse(text)
    cfg = compiler_settings(everything)
    out = []

    def walk(elements, frame, articulated, depth):
        for el in elements:
            if len(out) >= max_bodies:
                return
            rot, unknown = orientation(el, cfg)
            pos = _floats(el, "pos", 3) or [0.0, 0.0, 0.0]
            child = frame.compose(tuple(pos), rot)
            if el.tag == "body":
                moving = articulated or any(
                    c.tag in _ARTICULATION for c in el.children)
                walk(el.children, child, moving or unknown, depth + 1)
                continue
            if el.tag == "geom":
                if articulated or unknown:
                    continue
                if (el.attrs.get("type") or "").lower() != "box":
                    continue
                size = _floats(el, "size", 3)
                if size is None:
                    continue                  # unmeasurable without defaults
                lo, hi = bounds_of_boxes([(child, tuple(size))])
                out.append(Body(
                    name=el.attrs.get("name", ""),
                    body_id="geom@%d" % el.start, kind="geom", element=el,
                    lo=lo, hi=hi, parent_rot=frame.rot, depth=depth))
                continue
            if el.tag in ("mujoco", "worldbody"):
                # ...and ONLY these. ``<default>`` and ``<asset>`` also
                # contain ``<geom>`` elements -- class templates and mesh
                # declarations, not scene bodies -- and walking them would
                # invent an obstacle at the origin out of a default size.
                walk(el.children, child, articulated, depth + 1)

    walk(roots, Frame(), False, 0)
    return out


def _fmt(v):
    s = "%.6f" % (v + 0.0)
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s or "0"


def move_bodies(text, moves):
    """``(new_text, applied)`` -- shift each box by a WORLD-frame delta.

    Only the box's own ``pos`` attribute is written (inserted when the element
    relied on the implicit ``0 0 0``), converted into its parent body's frame
    first. Edits are applied back to front so earlier offsets stay valid.
    """
    edits, applied = [], []
    for body, delta in moves:
        local = mat_apply(mat_transpose(body.parent_rot), tuple(delta))
        el = body.element
        old = _floats(el, "pos", 3) or [0.0, 0.0, 0.0]
        new = tuple(old[i] + local[i] for i in range(3))
        value = "%s %s %s" % (_fmt(new[0]), _fmt(new[1]), _fmt(new[2]))
        raw = el.attrs.get("pos")
        if raw is None:
            edits.append((el.attr_end, el.attr_end, ' pos="%s"' % value))
            how = "inserted"
        else:
            # Locate THIS element's own pos="..." -- searched inside its tag
            # span only, so an identically-spelled attribute elsewhere in the
            # file can never be the one that moves.
            m = re.search(r'(?<![\w.:-])pos\s*=\s*"[^"]*"',
                          text[el.start:el.end])
            if m is None:
                edits.append((el.attr_end, el.attr_end, ' pos="%s"' % value))
                how = "inserted"
            else:
                edits.append((el.start + m.start(), el.start + m.end(),
                              'pos="%s"' % value))
                how = "rewritten"
        applied.append({"body_id": body.body_id,
                        "def": body.name or None, "kind": body.kind,
                        "field": how,
                        "translation_from": [round(v, 6) for v in old],
                        "translation_to": [round(v, 6) for v in new]})
    for start, end, repl in sorted(edits, key=lambda e: e[0], reverse=True):
        text = text[:start] + repl + text[end:]
    return text, applied
