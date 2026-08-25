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

"""Reading the artifact the agent produced, as TEXT.

Two assertions in the SPEC are explicitly about the artifact's source, not the
loaded scene:

  * A1.1 wants "a URDF/proto reference whose basename is ``husky.urdf`` (read
    from the artifact text)". The engine expands ``URDFRobot { url ... }``
    blocks into plain ``Robot { ... }`` at parse time
    (``WbUrdfImporter::expandUrdfRobotBlocks``), so the loaded scene has no
    ``url`` field left to inspect -- the reference only exists in the file.
  * discovery of *which* file is the artifact when the agent left several.

...and, since 2026-08-09, a third: R1's **grade-time obstacle placement**
(``common/r1_placement.py``) has to MOVE bodies inside a world the agent wrote,
between the agent's session and the graded run. That is a text edit by
construction -- there is no loaded scene yet, and re-emitting a parsed world
would hand the agent's deliverable back to it rewritten. :func:`scan_bodies`
and :func:`move_bodies` are that half: a name-free geometric read of every
non-robot body's world AABB, and a minimal in-place rewrite of the ONE field
that moves it.

Everything else is measured from the running simulation.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

# The prefix the grader stamps on the world copies it injects the recorder
# into, so artifact discovery never picks up the grader's own scratch.
INJECTED_PREFIX = "_agentbench_"

_COMMENT = re.compile(r"(?m)^([^#\"\n]*(?:\"[^\"\n]*\"[^#\"\n]*)*)#.*$")


def strip_comments(text: str) -> str:
    """Drop ``#`` comments, honouring quoted strings on the same line."""
    return _COMMENT.sub(r"\1", text)


_HARNESS_SIBLING = re.compile(r"^\.\.?harness_.*\.(?:wbt|wbproj)$")


def is_harness_sibling(name: str) -> bool:
    """Harness-injected sibling files (``.harness_<world>.wbt`` supervisor
    copies and ``..harness_<world>.wbproj`` project files) -- written by the
    validation harness next to any world it loads, gitignored, never task or
    agent content. Staging code skips them: a stale ``.wbproj`` copied beside
    a world made the phase-B engine launch flaky (measured 2026-08-01)."""
    return bool(_HARNESS_SIBLING.match(name))


def find_world_artifacts(scratch_dir) -> list[Path]:
    """Every candidate ``.wbt`` under ``scratch_dir``, newest first.

    Grader-injected copies are excluded by filename prefix so the discovery
    rule can never latch onto our own scaffolding.
    """
    scratch = Path(scratch_dir)
    if not scratch.exists():
        return []
    out = [p for p in [*scratch.rglob("*.omniworld"), *scratch.rglob("*.wbt")]
           if not p.name.startswith(INJECTED_PREFIX)
           and not is_harness_sibling(p.name)]
    out.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return out


def pick_artifact(scratch_dir):
    """The world the agent last worked on, or None.

    Rule, stated so it can be argued with: **the most recently modified
    non-injected ``.wbt`` under the scratch dir**. The prompts are fixed
    verbatim and cannot ask the agent to name its output, so the grader has to
    have a discovery rule; "the one it touched last" is the least prescriptive
    one available. Every candidate it saw is recorded in the row.
    """
    cands = find_world_artifacts(scratch_dir)
    return cands[0] if cands else None


# --- robot-block scanning ---------------------------------------------------

_BLOCK_HEAD = re.compile(r"(?<![A-Za-z0-9_])(URDFRobot|Robot)\s*\{")


def _block_body(text: str, open_brace: int) -> tuple[str, int]:
    """Return (inside, index_after_close) for the block whose ``{`` is at
    ``open_brace``. Brace counting, quote-aware."""
    depth = 1
    i = open_brace + 1
    in_str = False
    while i < len(text) and depth:
        c = text[i]
        if in_str:
            if c == "\\":
                i += 2
                continue
            if c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                break
        i += 1
    return text[open_brace + 1:i], i + 1


_SF_STRING = {
    k: re.compile(r'(?<![A-Za-z0-9_])%s\s+"([^"]*)"' % k)
    for k in ("url", "name", "controller", "model")
}


def scan_robot_blocks(world_path) -> list[dict]:
    """Top-level ``Robot``/``URDFRobot`` declarations in a ``.wbt``.

    Returns ``[{"kind", "name", "controller", "url", "urls", "offset"}]``.
    Nested Robot nodes (a Robot inside another Robot's children) are skipped:
    the scan only walks the file at brace depth 0.
    """
    text = strip_comments(Path(world_path).read_text(
        encoding="utf-8", errors="replace"))
    out: list[dict] = []
    i = 0
    while True:
        m = _BLOCK_HEAD.search(text, i)
        if not m:
            break
        inside, after = _block_body(text, m.end() - 1)
        urls = _SF_STRING["url"].findall(inside)

        def first(key):
            hit = _SF_STRING[key].search(inside)
            return hit.group(1) if hit else ""

        out.append({
            "kind": m.group(1),
            "name": first("name"),
            "controller": first("controller"),
            "url": urls[0] if urls else "",
            "urls": urls,
            "offset": m.start(),
        })
        i = after
    return out


def husky_robot_blocks(world_path) -> list[dict]:
    """Robot blocks whose ``url`` basename is ``husky.urdf`` (A1.1)."""
    return [b for b in scan_robot_blocks(world_path)
            if any(u.replace("\\", "/").rsplit("/", 1)[-1].lower()
                   == "husky.urdf" for u in b["urls"])]


def rebase_relative_urls(text: str, source_dir, *, only_existing=True):
    """Rewrite relative ``url "..."`` values to absolute, against ``source_dir``.

    OmniSim resolves ``URDFRobot { url ... }`` **relative to the world file**,
    so copying a world to a different directory silently breaks every relative
    asset reference in it. The benchmark does exactly that: the deliverable is
    relocated into the grader's scratch dir before Phase B, and a world the
    agent authored -- and verified -- in its workspace then loads with
    ``Cannot open URDF file`` for every robot.

    Measured on the first v0.3 pilot (A1/omnisim): the agent wrote ten correct
    ``URDFRobot`` blocks with a relative url, the copy re-based it four levels
    into the results tree, and the graded world reported ``n_robots: 0``.

    Re-basing restores the file to what it meant where it was written; it does
    not change which assets the world points at, and it grants the agent
    nothing it had not already earned. It is the same transformation the
    benchmark already applies to its OWN fixtures through the
    ``@HUSKY_URDF@`` token, which expands to an absolute path for this very
    reason -- so leaving the agent's worlds un-rebased holds them to a stricter
    standard than the benchmark holds itself to.

    ``only_existing`` (default) rewrites a url ONLY when it actually resolves
    to a file from ``source_dir``. A path that was already broken where it was
    authored stays broken, so a genuinely bad reference still fails the task.

    Returns ``(new_text, [{"from", "to"} ...])``.
    """
    base = Path(source_dir)
    changes = []

    def _sub(m):
        raw = m.group(1)
        if not raw:
            return m.group(0)
        p = Path(raw)
        # Absolute, or a URL scheme (omnisim://, http://): leave alone.
        if p.is_absolute() or "://" in raw:
            return m.group(0)
        resolved = (base / raw).resolve()
        if only_existing and not resolved.is_file():
            return m.group(0)
        new = resolved.as_posix()
        changes.append({"from": raw, "to": new})
        return m.group(0).replace('"%s"' % raw, '"%s"' % new)

    return _SF_STRING["url"].sub(_sub, text), changes


# --- name-free body scanning, and moving a body ------------------------------
#
# Used by R1's grade-time placement step. Two rules shape all of it:
#
#   * **Bodies are found by GEOMETRY, never by name.** Agents have called the
#     five R1 obstacles ``crate A``..``crate E`` and ``obstacle_1``..``_6``;
#     the grader already matches them by measured AABB
#     (``r1_core.match_spec_obstacles``) and the placer must use the same
#     handle or it would be placing OUR convention rather than the task's.
#   * **The rewrite is minimal.** Exactly one numeric field per moved body is
#     replaced (inserted, if the body relied on the ``0 0 0`` default); every
#     other byte of the agent's world survives, including its comments,
#     formatting and node order. A world that is re-emitted from a parse is a
#     different deliverable.

#: Node types whose subtree is skipped whole: a robot's own geometry is never
#: an obstacle, and R1.2 has already counted the robots.
ROBOT_KINDS = frozenset({"Robot", "URDFRobot"})

#: Geometry primitives -- they carry extents, not a pose, so they are read for
#: their size and never offered as something to move.
GEOMETRY_KINDS = frozenset({
    "Box", "Cylinder", "Sphere", "Capsule", "Cone", "Plane", "Mesh",
    "IndexedFaceSet", "IndexedLineSet", "PointSet", "ElevationGrid"})

#: ...plus the field-value nodes that can appear inside a body and can never
#: be one. A node OUTSIDE this set that encloses geometry is a candidate,
#: whatever it is called -- an unknown PROTO is a body until proven otherwise.
_NOT_A_BODY = GEOMETRY_KINDS | frozenset({
    "Shape", "Appearance", "PBRAppearance", "Material", "ImageTexture",
    "TextureTransform", "Color", "Coordinate", "Normal", "TextureCoordinate",
    "Physics", "Damping", "Fluid", "ImmersionProperties",
    "HingeJointParameters", "JointParameters", "BallJointParameters",
    "WorldInfo", "Viewpoint", "Background", "TexturedBackground", "Fog",
    "DirectionalLight", "PointLight", "SpotLight", "Recognition", "Lens",
    "Focus", "Zoom", "LensFlare", "ContactProperties"})

_NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
_NODE_HEAD = re.compile(
    r"(?:DEF\s+(?P<def>[^\s{}\[\]]+)\s+)?(?P<kind>[A-Za-z_][A-Za-z0-9_]*)\s*\{")
_USE_TOKEN = re.compile(r"(?<![A-Za-z0-9_])USE\s+([^\s{}\[\],]+)")
_STRING = re.compile(r'"(?:[^"\\]|\\.)*"')

#: Recursion guard for pathological (or hostile) nesting.
_MAX_DEPTH = 16


def mask_text(text: str) -> str:
    """A SAME-LENGTH copy with string contents and comments blanked.

    Index-preserving on purpose: every scan below runs on the mask -- so a
    ``#`` or a ``{`` inside a quoted string can never be mistaken for syntax --
    while every edit is applied to the ORIGINAL at the same offsets.
    """
    out = list(_STRING.sub(lambda m: '"' + " " * (len(m.group(0)) - 2) + '"',
                           text))
    i = 0
    while i < len(out):
        if out[i] == "#":
            while i < len(out) and out[i] != "\n":
                out[i] = " "
                i += 1
        i += 1
    return "".join(out)


class Node:
    """One ``[DEF <name>] <Kind> { ... }`` block, by offset in the source."""

    __slots__ = ("def_name", "kind", "head", "brace", "inner_start",
                 "inner_end")

    def __init__(self, def_name, kind, head, brace, inner_start, inner_end):
        self.def_name = def_name
        self.kind = kind
        self.head = head              # index of DEF, or of the type name
        self.brace = brace            # index of '{'
        self.inner_start = inner_start
        self.inner_end = inner_end    # index of the matching '}'


def iter_child_nodes(masked, start, end):
    """Node blocks at the TOP level of ``masked[start:end]``, in file order.

    Nested blocks are skipped over (the caller recurses when it wants them),
    so this walks one brace level at a time.
    """
    i = start
    while True:
        m = _NODE_HEAD.search(masked, i, end)
        if not m:
            return
        brace = m.end() - 1
        _inside, after = _block_body(masked, brace)
        inner_end = after - 1
        if inner_end <= brace:                       # unbalanced braces
            return
        yield Node(m.group("def"), m.group("kind"), m.start(), brace,
                   brace + 1, inner_end)
        i = after


def collect_defs(masked):
    """``{DEF name: Node}`` for every DEF'd node anywhere in the file."""
    out = {}

    def walk(start, end, depth):
        if depth > _MAX_DEPTH:
            return
        for node in iter_child_nodes(masked, start, end):
            if node.def_name and node.def_name not in out:
                out[node.def_name] = node
            walk(node.inner_start, node.inner_end, depth + 1)

    walk(0, len(masked), 0)
    return out


def _depth(masked, base, pos):
    d = 0
    for c in masked[base:pos]:
        if c == "{" or c == "[":
            d += 1
        elif c == "}" or c == "]":
            d -= 1
    return d


def numeric_field(masked, inner_start, inner_end, name, count):
    """The first ``<name> n1 .. n<count>`` at THIS node's own field level.

    Depth-checked, so a nested node's ``translation`` is never mistaken for
    the enclosing body's.
    """
    pat = re.compile(r"(?<![A-Za-z0-9_])" + re.escape(name) + r"\s+"
                     + r"\s+".join(["(" + _NUM + ")"] * count))
    for m in pat.finditer(masked, inner_start, inner_end):
        if _depth(masked, inner_start, m.start()) == 0:
            return m
    return None


def _vec(masked, node, name, count, default):
    m = numeric_field(masked, node.inner_start, node.inner_end, name, count)
    if m is None:
        return default
    return tuple(float(m.group(i + 1)) for i in range(count))


# --- the small amount of rotation algebra the AABBs need --------------------

IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def axis_angle_matrix(x, y, z, angle):
    """Rodrigues. A zero axis (Webots' ``rotation 0 0 0 0``) is identity."""
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12 or abs(angle) < 1e-12:
        return IDENTITY
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return ((t * x * x + c, t * x * y - s * z, t * x * z + s * y),
            (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
            (t * x * z - s * y, t * y * z + s * x, t * z * z + c))


def mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def mat_apply(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def mat_transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def abs_apply(m, v):
    """|M| . v -- the half-extent of an oriented box's axis-aligned bounds."""
    return tuple(sum(abs(m[i][k]) * v[k] for k in range(3)) for i in range(3))


class Frame:
    """A rigid placement: where a node's origin is, and how it is turned."""

    __slots__ = ("origin", "rot")

    def __init__(self, origin=(0.0, 0.0, 0.0), rot=IDENTITY):
        self.origin = origin
        self.rot = rot

    def compose(self, translation, rot):
        o = mat_apply(self.rot, translation)
        return Frame((self.origin[0] + o[0], self.origin[1] + o[1],
                      self.origin[2] + o[2]), mat_mul(self.rot, rot))


class Body:
    """A movable body with world-space bounds.

    Duck-types ``graders.evidence.Body`` closely enough for
    ``r1_core.match_spec_obstacles`` -- which is the point: the placer matches
    with the SAME function the grader matches with, so a body the placer moved
    is a body the grader will recognise.
    """

    __slots__ = ("name", "body_id", "kind", "node", "lo", "hi", "parent_rot",
                 "depth")

    has_aabb = True
    robot_class = False
    member_of = None

    def __init__(self, name, body_id, kind, node, lo, hi, parent_rot, depth):
        self.name = name
        self.body_id = body_id
        self.kind = kind
        self.node = node
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
        return ("<wbt Body %s %s centre=(%.3f, %.3f) size=(%.3f, %.3f)>"
                % (self.kind, self.name or "-", self.centre[0], self.centre[1],
                   self.size[0], self.size[1]))


def _geometry_boxes(masked, node, frame, defs, boxes, seen, depth):
    """Accumulate ``(centre, half_extent)`` for every primitive in a node."""
    if depth > _MAX_DEPTH:
        return
    kind = node.kind
    frame = frame.compose(_vec(masked, node, "translation", 3, (0.0, 0.0, 0.0)),
                          axis_angle_matrix(
                              *_vec(masked, node, "rotation", 4,
                                    (0.0, 0.0, 1.0, 0.0))))
    if kind == "Box":
        s = _vec(masked, node, "size", 3, (0.0, 0.0, 0.0))
        boxes.append((frame, (s[0] / 2.0, s[1] / 2.0, s[2] / 2.0)))
        return
    if kind in ("Cylinder", "Capsule"):
        h = _vec(masked, node, "height", 1, (0.0,))[0]
        r = _vec(masked, node, "radius", 1, (0.0,))[0]
        pad = r if kind == "Capsule" else 0.0
        boxes.append((frame, (r, r, h / 2.0 + pad)))
        return
    if kind == "Sphere":
        r = _vec(masked, node, "radius", 1, (0.0,))[0]
        boxes.append((frame, (r, r, r)))
        return
    if kind in GEOMETRY_KINDS:
        return                          # a hull we cannot measure from text
    before = len(boxes)
    _subtree_boxes(masked, node.inner_start, node.inner_end, frame, defs,
                   boxes, seen, depth + 1)
    if len(boxes) == before:
        # A PROTO that draws its own box and publishes the extents as a field
        # (``WoodenBox { size ... }`` and friends). Read rather than ignored:
        # an obstacle authored as one is still an obstacle.
        s = _vec(masked, node, "size", 3, None)
        if s:
            boxes.append((frame, (s[0] / 2.0, s[1] / 2.0, s[2] / 2.0)))


def _subtree_boxes(masked, start, end, frame, defs, boxes, seen, depth):
    if depth > _MAX_DEPTH:
        return
    spans = []
    for node in iter_child_nodes(masked, start, end):
        spans.append((node.head, node.inner_end + 1))
        if node.kind in ROBOT_KINDS:
            continue
        _geometry_boxes(masked, node, frame, defs, boxes, seen, depth)
    for m in _USE_TOKEN.finditer(masked, start, end):
        if any(a <= m.start() < b for a, b in spans):
            continue                       # belongs to a nested node
        name = m.group(1)
        target = defs.get(name)
        if target is None or name in seen or target.kind in ROBOT_KINDS:
            continue
        _geometry_boxes(masked, target, frame, defs, boxes,
                        seen | {name}, depth)


def bounds_of_boxes(boxes):
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for frame, half in boxes:
        h = abs_apply(frame.rot, half)
        for i in range(3):
            lo[i] = min(lo[i], frame.origin[i] - h[i])
            hi[i] = max(hi[i], frame.origin[i] + h[i])
    return tuple(lo), tuple(hi)


def scan_bodies(text, *, max_bodies=400):
    """Every non-robot body in a ``.wbt``, with WORLD-space bounds.

    Outermost first (pre-order), which is what makes a geometric match land on
    the node a caller can actually move: a ``Solid`` and a ``Pose`` inside it
    can have identical bounds, and the ``Solid`` is the one that carries the
    collision geometry with it.

    ``Robot``/``URDFRobot`` subtrees are skipped whole. Bodies with no
    measurable geometry (a ``Mesh`` hull, an empty group) are not returned at
    all -- an unmeasured body is never silently given a guessed box.
    """
    masked = mask_text(text)
    defs = collect_defs(masked)
    out = []

    def walk(start, end, frame, depth):
        if depth > _MAX_DEPTH or len(out) >= max_bodies:
            return
        for node in iter_child_nodes(masked, start, end):
            if node.kind in ROBOT_KINDS:
                continue
            child = frame.compose(
                _vec(masked, node, "translation", 3, (0.0, 0.0, 0.0)),
                axis_angle_matrix(*_vec(masked, node, "rotation", 4,
                                        (0.0, 0.0, 1.0, 0.0))))
            if node.kind not in _NOT_A_BODY:
                boxes = []
                _subtree_boxes(masked, node.inner_start, node.inner_end,
                               child, defs, boxes, set(), 0)
                if not boxes:
                    s = _vec(masked, node, "size", 3, None)
                    if s:
                        boxes = [(child, (s[0] / 2.0, s[1] / 2.0, s[2] / 2.0))]
                if boxes:
                    lo, hi = bounds_of_boxes(boxes)
                    out.append(Body(
                        name=node.def_name or "",
                        body_id="%s@%d" % (node.kind, node.head),
                        kind=node.kind, node=node, lo=lo, hi=hi,
                        parent_rot=frame.rot, depth=depth))
            walk(node.inner_start, node.inner_end, child, depth + 1)

    walk(0, len(masked), Frame(), 0)
    return out


def _fmt(v):
    s = "%.6f" % (v + 0.0)
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s or "0"


def _line_indent(text, idx):
    start = text.rfind("\n", 0, idx) + 1
    pad = []
    for ch in text[start:idx]:
        if ch in " \t":
            pad.append(ch)
        else:
            break
    return "".join(pad)


def move_bodies(text, moves):
    """``(new_text, applied)`` -- shift each body by a WORLD-frame delta.

    The delta is converted into the body's PARENT frame before it is written,
    so a body under a rotated ancestor still lands where the caller asked. The
    body's own ``translation`` is the only field touched; if it had none (the
    ``0 0 0`` default) one is inserted, indented to match its block.

    ``moves`` is ``[(body, (dx, dy, dz)), ...]``. Edits are applied back to
    front so earlier offsets stay valid.
    """
    masked = mask_text(text)
    edits, applied = [], []
    for body, delta in moves:
        local = mat_apply(mat_transpose(body.parent_rot), tuple(delta))
        node = body.node
        m = numeric_field(masked, node.inner_start, node.inner_end,
                          "translation", 3)
        if m is None:
            old = (0.0, 0.0, 0.0)
            new = tuple(old[i] + local[i] for i in range(3))
            indent = _line_indent(text, node.head) + "  "
            edits.append((node.brace + 1, node.brace + 1,
                          "\n%stranslation %s %s %s"
                          % (indent, _fmt(new[0]), _fmt(new[1]),
                             _fmt(new[2]))))
            how = "inserted"
        else:
            old = tuple(float(m.group(i + 1)) for i in range(3))
            new = tuple(old[i] + local[i] for i in range(3))
            edits.append((m.start(1), m.end(3),
                          "%s %s %s" % (_fmt(new[0]), _fmt(new[1]),
                                        _fmt(new[2]))))
            how = "rewritten"
        applied.append({"body_id": body.body_id, "def": body.name or None,
                        "kind": body.kind, "field": how,
                        "translation_from": [round(v, 6) for v in old],
                        "translation_to": [round(v, 6) for v in new]})
    for start, end, repl in sorted(edits, key=lambda e: e[0], reverse=True):
        text = text[:start] + repl + text[end:]
    return text, applied
