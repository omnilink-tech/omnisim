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

"""A small ``.wbt`` reader, and the wheel-detection rule the roll-check uses.

This exists so ``scripts/dev/roll_check.py`` can answer "is this a hand-authored
wheeled robot, and what is each wheel's radius / motor / axis?" WITHOUT starting
the engine. Static answers are what make a 1400-world sweep affordable: the
scan over the whole tree runs in a couple of seconds, and only the worlds it
flags ever get a (much more expensive) simulated run.

It is deliberately a *reader*, not a writer -- nothing here ever rewrites a
world in place. ``roll_check.py`` composes sibling copies by string append, the
same way ``run-headless --fail-on-runaway`` does.

SCOPE, stated up front because a partial parser that pretends to be complete is
worse than none: this understands the subset of VRML that ``.wbt`` files in this
repo actually use -- ``DEF``/``USE``, SFNode and MFNode fields, and scalar /
vector / string / boolean values. It does NOT expand ``PROTO`` bodies and it
does NOT expand ``URDFRobot { url ... }`` (that is a source-level expansion the
engine's tokenizer does when it reads the FILE -- see AGENTS.md). Both of those
are *deliberate*: the defect class this serves is hand-authored wheel stanzas,
and a PROTO/URDF robot's wheels are not authored in the world file at all.
``robots()`` therefore reports what it could not see, so a caller can say
"skipped, PROTO" rather than "no wheels found".
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field as dc_field
from pathlib import Path

__all__ = [
    "Node", "parse_wbt", "robots", "world_info", "WheeledRobot", "Wheel",
]

# A bare token: an identifier, a number, or a punctuation character we care
# about. Strings and comments are handled separately by the tokenizer.
_TOKEN_RE = re.compile(r'"(?:[^"\\]|\\.)*"|[{}\[\]]|[^\s{}\[\]]+')

_BOOLS = {"TRUE": True, "FALSE": False}


def _is_value_token(tok: str) -> bool:
    """True for a token that can only be part of a FIELD VALUE, never a name.

    Field values in .wbt are numbers, quoted strings and TRUE/FALSE. A bare
    identifier is either the next field's name or a node type, so it ends the
    current value -- that is the whole disambiguation rule this parser needs.
    """
    if tok.startswith('"') or tok in _BOOLS:
        return True
    try:
        float(tok)
    except ValueError:
        return False
    return True


def _tokenize(text: str):
    out = []
    for line in text.splitlines():
        # Comments run to end of line, but a '#' inside a quoted string is
        # data (texture URLs and the odd colour literal use them).
        in_string = False
        cut = len(line)
        i = 0
        while i < len(line):
            ch = line[i]
            if ch == '"' and (i == 0 or line[i - 1] != "\\"):
                in_string = not in_string
            elif ch == "#" and not in_string:
                cut = i
                break
            i += 1
        out.extend(_TOKEN_RE.findall(line[:cut]))
    return out


@dataclass
class Node:
    """One VRML node: its type, optional DEF name, and its fields."""

    type: str
    defname: str | None = None
    fields: dict = dc_field(default_factory=dict)
    #: Set when this node was written ``USE OTHER`` -- we keep the reference
    #: rather than resolving it, because a USE'd wheel is the SAME body and
    #: counting it twice would double a robot's wheel count.
    use: str | None = None

    # -- convenience readers, all None-tolerant ---------------------------
    def sf(self, name, default=None):
        """A scalar field's value list (already coerced to float/str/bool)."""
        v = self.fields.get(name)
        return default if v is None else v

    def num(self, name, default=None):
        v = self.fields.get(name)
        if isinstance(v, list) and v and isinstance(v[0], float):
            return v[0]
        return default

    def vec(self, name, default=None):
        v = self.fields.get(name)
        if isinstance(v, list) and all(isinstance(x, float) for x in v) and v:
            return v
        return default

    def text(self, name, default=None):
        v = self.fields.get(name)
        if isinstance(v, list) and v and isinstance(v[0], str):
            return v[0]
        return default

    def node(self, name):
        v = self.fields.get(name)
        return v if isinstance(v, Node) else None

    def nodes(self, name):
        v = self.fields.get(name)
        if isinstance(v, Node):
            return [v]
        if isinstance(v, list):
            return [x for x in v if isinstance(x, Node)]
        return []

    def descendants(self, _seen=None):
        """Every Node reachable through this node's fields, depth-first.

        Cycle-safe by identity. A DEF'd node USE'd inside its own subtree makes
        the graph cyclic once references are bound, and three worlds in this
        corpus do exactly that (asymmetric_friction, laser_pointer,
        interaction_with_solid_reference_model); without the guard they blow the
        recursion limit and drop out of the sweep silently.
        """
        if _seen is None:
            _seen = {id(self)}
        for value in self.fields.values():
            items = value if isinstance(value, list) else [value]
            for item in items:
                if isinstance(item, Node) and id(item) not in _seen:
                    _seen.add(id(item))
                    yield item
                    yield from item.descendants(_seen)


class _Parser:
    def __init__(self, tokens):
        self.t = tokens
        self.i = 0

    def peek(self, k=0):
        j = self.i + k
        return self.t[j] if j < len(self.t) else None

    def take(self):
        tok = self.peek()
        self.i += 1
        return tok

    def parse_top(self):
        out = []
        while self.peek() is not None:
            node = self.try_node()
            if node is None:
                self.take()  # EXTERNPROTO lines, stray tokens: skip
            else:
                out.append(node)
        return out

    def try_node(self):
        tok = self.peek()
        if tok is None:
            return None
        if tok == "DEF" and self.peek(3) == "{":
            self.take()
            name = self.take()
            type_name = self.take()
            self.take()  # '{'
            return self._body(Node(type_name, defname=name))
        if tok == "USE":
            self.take()
            return Node("USE", use=self.take())
        if self.peek(1) == "{" and not _is_value_token(tok) and tok not in ("[", "]", "{", "}"):
            type_name = self.take()
            self.take()  # '{'
            return self._body(Node(type_name))
        return None

    def _body(self, node):
        while True:
            tok = self.peek()
            if tok is None or tok == "}":
                self.take()
                return node
            name = self.take()
            self._field(node, name)

    def _field(self, node, name):
        tok = self.peek()
        if tok == "[":
            self.take()
            items = []
            while self.peek() not in (None, "]"):
                sub = self.try_node()
                if sub is not None:
                    items.append(sub)
                else:
                    items.append(_coerce(self.take()))
            self.take()  # ']'
            node.fields[name] = items
            return
        sub = self.try_node()
        if sub is not None:
            node.fields[name] = sub
            return
        values = []
        while self.peek() is not None and _is_value_token(self.peek()):
            values.append(_coerce(self.take()))
        node.fields[name] = values


def _coerce(tok):
    if tok in _BOOLS:
        return _BOOLS[tok]
    if tok.startswith('"'):
        return tok[1:-1]
    try:
        return float(tok)
    except ValueError:
        return tok


def _resolve_uses(top):
    """Replace ``USE NAME`` placeholders with the DEF'd node itself.

    VRML requires a DEF to precede its USEs, so one document-order pass is
    enough. This matters more than it sounds: ``projects/samples/devices`` and
    ``projects/languages`` author every wheel as ``boundingObject USE WHEEL``,
    and without resolution the wheel rule sees a collider of type "USE", finds
    no rolling geometry, and reports 45 wheeled worlds as having no wheels --
    i.e. it would silently under-report exactly the corpus it exists to sweep.
    """
    defs = {}
    # A DEF'd node that is USE'd inside its own subtree (the corpus has a few:
    # asymmetric_friction, laser_pointer, interaction_with_solid_reference_model)
    # would otherwise be walked into itself forever once the reference is bound
    # to the shared object. Identity-keyed, because equal-looking nodes at
    # different places in the tree are genuinely different nodes.
    seen = set()

    def walk(node):
        if id(node) in seen:
            return
        seen.add(id(node))
        if node.defname:
            defs.setdefault(node.defname, node)
        for key, value in list(node.fields.items()):
            if isinstance(value, Node):
                node.fields[key] = _sub(value)
            elif isinstance(value, list):
                node.fields[key] = [_sub(x) if isinstance(x, Node) else x
                                    for x in value]

    def _sub(child):
        if child.use is not None:
            return defs.get(child.use, child)
        walk(child)
        return child

    for node in top:
        walk(node)
    return top


def parse_wbt(path):
    """Parse a .wbt into a list of top-level Nodes, with USE references bound."""
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    return _resolve_uses(_Parser(_tokenize(text)).parse_top())


def world_info(top):
    for node in top:
        if node.type == "WorldInfo":
            return node
    return None


# ---------------------------------------------------------------------------
# The wheel rule
# ---------------------------------------------------------------------------

#: Geometry types whose boundingObject makes a rolling contact patch. A wheel
#: authored with a BOX collider is not excluded here (it is reported with
#: ``radius=None``), because a box "wheel" cannot roll by construction and the
#: caller should see that rather than have it filtered away silently.
_ROLLING_GEOMS = ("Cylinder", "Sphere")


@dataclass
class Wheel:
    motor: str | None
    sensor: str | None
    radius: float | None
    axis: tuple
    anchor: tuple
    max_torque: float | None
    max_velocity: float | None
    geom: str | None
    #: True when the collider is a bare Cylinder with no Pose wrapper AND the
    #: wheel Solid itself carries the 90-degree rotation. Recorded because it is
    #: a *suspected* second failure pattern (drive_test.omniworld's DEF FLAP), not
    #: because it is known-bad -- the roll-check measures, it does not assume.
    bare_cylinder: bool = False
    solid_def: str | None = None
    solid_name: str | None = None


@dataclass
class WheeledRobot:
    defname: str | None
    name: str | None
    controller: str | None
    supervisor: bool
    wheels: list
    translation: tuple
    node: Node = None

    @property
    def radii(self):
        return [w.radius for w in self.wheels if w.radius]

    @property
    def radius(self):
        """The modal wheel radius -- what ``omega * r`` should use."""
        rs = self.radii
        if not rs:
            return None
        rs = sorted(rs)
        return rs[len(rs) // 2]


def _geom_of(bounding):
    """(geom_type, radius, is_bare) for a boundingObject field value.

    Unwraps a ``Pose``/``Transform``/``Group`` wrapper, which is how the Husky
    convention authors a wheel collider (see drive_test.omniworld DEF ROLL).
    """
    if not isinstance(bounding, Node):
        return (None, None, False)
    bare = bounding.type in _ROLLING_GEOMS
    node = bounding
    depth = 0
    while node is not None and node.type not in _ROLLING_GEOMS and depth < 8:
        # Shape wraps its collider in `geometry`; Pose/Transform/Group/Solid in
        # `children`. Both spellings are in the corpus.
        nxt = node.node("geometry")
        if nxt is None:
            kids = node.nodes("children")
            nxt = kids[0] if kids else None
        node = nxt
        depth += 1
    if node is None or node.type not in _ROLLING_GEOMS:
        return (bounding.type, None, False)
    return (node.type, node.num("radius"), bare)


def _wheels_in(node):
    """Every HingeJoint under ``node`` that looks like a driven wheel."""
    out = []
    for joint in [node] + list(node.descendants()):
        if joint.type != "HingeJoint":
            continue
        motors = [d for d in joint.nodes("device") if d.type == "RotationalMotor"]
        sensors = [d for d in joint.nodes("device") if d.type == "PositionSensor"]
        if not motors:
            continue
        end = joint.node("endPoint")
        if end is None or end.type not in ("Solid", "Robot"):
            continue
        geom, radius, bare = _geom_of(end.fields.get("boundingObject"))
        if geom not in _ROLLING_GEOMS:
            continue
        params = joint.node("jointParameters")
        axis = tuple(params.vec("axis", [1.0, 0.0, 0.0])) if params else (1.0, 0.0, 0.0)
        anchor = tuple(params.vec("anchor", [0.0, 0.0, 0.0])) if params else (0.0, 0.0, 0.0)
        out.append(Wheel(
            motor=motors[0].text("name"),
            sensor=sensors[0].text("name") if sensors else None,
            radius=radius,
            axis=axis,
            anchor=anchor,
            max_torque=motors[0].num("maxTorque"),
            max_velocity=motors[0].num("maxVelocity"),
            geom=geom,
            bare_cylinder=bare,
            solid_def=end.defname,
            solid_name=end.text("name"),
        ))
    return out


def robots(top, min_wheels=2):
    """Hand-authored wheeled Robots in a parsed world.

    A robot qualifies when it owns >= ``min_wheels`` HingeJoints that each have
    a RotationalMotor and an endPoint Solid with a Cylinder/Sphere
    boundingObject. ``URDFRobot`` and PROTO-instanced robots never qualify --
    their wheels are not in the world file -- which is exactly the intended
    scope.
    """
    found = []
    for node in top:
        for candidate in ([node] + list(node.descendants())):
            if candidate.type != "Robot":
                continue
            wheels = _wheels_in(candidate)
            if len(wheels) < min_wheels:
                continue
            found.append(WheeledRobot(
                defname=candidate.defname,
                name=candidate.text("name"),
                controller=candidate.text("controller"),
                supervisor=bool(candidate.fields.get("supervisor") == [True]),
                wheels=wheels,
                translation=tuple(candidate.vec("translation", [0.0, 0.0, 0.0])),
                node=candidate,
            ))
    return found


def axis_dominant(axis):
    """Index of the dominant component of a joint axis (0=x, 1=y, 2=z)."""
    mags = [abs(a) for a in axis]
    return mags.index(max(mags)) if any(mags) else 1


def norm(v):
    return math.sqrt(sum(x * x for x in v))
