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

"""A small, brace-aware ``.wbt`` / ``.proto`` node scanner.

``_wbt_scan`` (the Tier-1 regex scanner) only sees flat, top-level
``URDFRobot``/``Transform`` blocks. The viewpoint check needs more: a robot
can be nested three ``Pose`` levels deep inside a ``children`` list, or hidden
behind a PROTO instance (``Husky { ... }``), and the camera check has to know
where it actually *is* in world space. That needs a real tree.

This is deliberately NOT a full VRML parser — it does not expand PROTOs,
evaluate ``IS`` bindings, or run the ``%< ... >%`` JavaScript templating. It
builds a node tree with raw field values, which is exactly enough to answer
"what nodes are in this world and where are they".

Lexical rules honoured (the ones that break naive regex scanners):

* ``#`` starts a comment that runs to end of line — but not inside a string.
* ``"..."`` strings may contain ``#``, ``{``, ``}`` and escaped quotes.
* ``%< ... >%`` template blocks (PROTO bodies) are skipped wholesale; they
  contain JavaScript whose braces would otherwise unbalance the scan.
* OmniSim convention: node types are ``UpperCamelCase``, fields are
  ``lowerCamelCase``. That is what disambiguates ``translation 0 0 1`` from
  the node that follows it.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field as dc_field
from pathlib import Path

__all__ = [
    "WbtNode",
    "parse_wbt",
    "iter_tree",
    "ProtoIndex",
    "proto_index_for",
    "axis_angle_to_matrix",
    "mat_mul",
    "mat_apply",
]

# --------------------------------------------------------------------------
# Tokeniser
# --------------------------------------------------------------------------

_TOKEN_RE = re.compile(
    r"""
      (?P<js>%<.*?>%)                       # PROTO template block (skipped)
    | (?P<comment>\#[^\n]*)                 # comment to end of line
    | (?P<string>"(?:\\.|[^"\\])*"?)        # string (tolerates unterminated)
    | (?P<punct>[{}\[\]])
    | (?P<number>[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)
    | (?P<ident>[A-Za-z_][A-Za-z0-9_]*)
    | (?P<other>\S)
    """,
    re.VERBOSE | re.DOTALL,
)

_KEYWORDS = {"DEF", "USE", "IS", "NULL", "TRUE", "FALSE", "PROTO", "EXTERNPROTO"}


def _tokenize(text: str) -> list[str]:
    out: list[str] = []
    for m in _TOKEN_RE.finditer(text):
        kind = m.lastgroup
        if kind in ("comment", "js"):
            continue
        out.append(m.group())
    return out


def _is_node_type(tok: str) -> bool:
    """OmniSim convention: node types start uppercase, fields start lowercase."""
    return bool(tok) and tok[0].isupper() and tok not in _KEYWORDS and tok.isidentifier()


def _is_field_name(tok: str) -> bool:
    return bool(tok) and (tok[0].islower() or tok[0] == "_") and tok.isidentifier()


# --------------------------------------------------------------------------
# Node tree
# --------------------------------------------------------------------------


@dataclass
class WbtNode:
    """One ``[DEF name] Type { ... }`` node.

    ``fields`` maps a field name to its value:

    * a ``list[str]`` of raw tokens for scalar / multi-scalar fields
      (``translation 0 0 1`` -> ``["0", "0", "1"]``),
    * a :class:`WbtNode` for an SFNode field,
    * a ``list`` (possibly mixed) for an MFNode / bracketed field.
    """

    type_name: str
    def_name: str | None = None
    use_name: str | None = None
    fields: dict[str, object] = dc_field(default_factory=dict)

    # ---- typed field access -------------------------------------------
    def raw(self, name: str) -> list[str] | None:
        v = self.fields.get(name)
        return v if isinstance(v, list) and all(isinstance(x, str) for x in v) else None

    def floats(self, name: str, count: int) -> tuple[float, ...] | None:
        toks = self.raw(name)
        if not toks or len(toks) < count:
            return None
        try:
            return tuple(float(t) for t in toks[:count])
        except ValueError:
            return None

    def string(self, name: str) -> str | None:
        toks = self.raw(name)
        if not toks or not toks[0].startswith('"'):
            return None
        return toks[0].strip('"')

    def nodes(self, name: str) -> list["WbtNode"]:
        v = self.fields.get(name)
        if isinstance(v, WbtNode):
            return [v]
        if isinstance(v, list):
            return [x for x in v if isinstance(x, WbtNode)]
        return []

    @property
    def children(self) -> list["WbtNode"]:
        return self.nodes("children")

    def child_nodes(self) -> list["WbtNode"]:
        """Every node reachable one level down, through any field."""
        out: list[WbtNode] = []
        for v in self.fields.values():
            if isinstance(v, WbtNode):
                out.append(v)
            elif isinstance(v, list):
                out.extend(x for x in v if isinstance(x, WbtNode))
        return out


class _Parser:
    def __init__(self, tokens: list[str]) -> None:
        self.t = tokens
        self.i = 0

    def peek(self, k: int = 0) -> str | None:
        j = self.i + k
        return self.t[j] if j < len(self.t) else None

    def next(self) -> str | None:
        tok = self.peek()
        if tok is not None:
            self.i += 1
        return tok

    # -- node ---------------------------------------------------------
    def try_node(self) -> WbtNode | None:
        """Parse ``[DEF n] Type { ... }`` or ``USE n`` at the cursor."""
        tok = self.peek()
        if tok is None:
            return None
        def_name = None
        if tok == "DEF":
            nxt = self.peek(1)
            if nxt is None:
                return None
            self.i += 2
            def_name = nxt
            tok = self.peek()
        elif tok == "USE":
            nxt = self.peek(1)
            self.i += 2
            return WbtNode(type_name="USE", use_name=nxt)
        if tok is None or not _is_node_type(tok):
            return None
        if self.peek(1) != "{":
            # A bare uppercase token that is not a node (rare); treat as scalar.
            if def_name is not None:
                self.i -= 2
            return None
        type_name = tok
        self.i += 2  # past Type and '{'
        node = WbtNode(type_name=type_name, def_name=def_name)
        self._parse_fields(node)
        return node

    def _parse_fields(self, node: WbtNode) -> None:
        while True:
            tok = self.peek()
            if tok is None:
                return
            if tok == "}":
                self.i += 1
                return
            if not _is_field_name(tok):
                # Stray token (malformed world, or an unhandled construct).
                self.i += 1
                continue
            name = tok
            self.i += 1
            node.fields[name] = self._parse_value()

    def _parse_value(self) -> object:
        tok = self.peek()
        if tok is None:
            return []
        if tok == "[":
            self.i += 1
            return self._parse_bracket()
        if tok == "IS":
            self.i += 1
            alias = self.next()
            return ["IS", alias or ""]
        sub = self.try_node()
        if sub is not None:
            return sub
        return self._parse_scalars()

    def _parse_bracket(self) -> list:
        out: list = []
        while True:
            tok = self.peek()
            if tok is None:
                return out
            if tok == "]":
                self.i += 1
                return out
            sub = self.try_node()
            if sub is not None:
                out.append(sub)
                continue
            out.append(self.next())

    def _parse_scalars(self) -> list[str]:
        out: list[str] = []
        while True:
            tok = self.peek()
            if tok is None or tok in ("}", "]", "["):
                return out
            if _is_field_name(tok):
                # Next field starts here — unless it is the value of an
                # enum-ish field, which OmniSim always quotes, so this is safe.
                return out
            if tok in ("DEF", "USE") or (_is_node_type(tok) and self.peek(1) == "{"):
                return out
            out.append(tok)
            self.i += 1


def parse_wbt(text: str) -> list[WbtNode]:
    """Parse a ``.wbt`` body into its list of top-level nodes."""
    p = _Parser(_tokenize(text))
    out: list[WbtNode] = []
    while p.peek() is not None:
        before = p.i
        node = p.try_node()
        if node is not None:
            out.append(node)
            continue
        if p.i == before:
            p.i += 1
    return out


def iter_tree(nodes: list[WbtNode]):
    """Depth-first walk yielding ``(node, depth)``."""
    stack = [(n, 0) for n in reversed(nodes)]
    while stack:
        node, depth = stack.pop()
        yield node, depth
        for c in reversed(node.child_nodes()):
            stack.append((c, depth + 1))


# --------------------------------------------------------------------------
# PROTO base-node resolution
# --------------------------------------------------------------------------

_PROTO_DECL_RE = re.compile(r"(?m)^\s*PROTO\s+([A-Za-z_][A-Za-z0-9_]*)\s*\[")


class ProtoIndex:
    """Maps a PROTO name to the built-in node type it ultimately derives from.

    ``Husky { ... }`` in a world is a ``Robot`` (or ``URDFRobot``); without
    resolving that, every PROTO-instanced robot is invisible to the checker.
    Derivation is transitive (a PROTO whose body is another PROTO) with a
    cycle guard.
    """

    def __init__(self, roots: list[Path]) -> None:
        self._base: dict[str, str] = {}
        self._resolved: dict[str, str] = {}
        for root in roots:
            self._scan(root)

    def _scan(self, root: Path) -> None:
        skip = {"msys64", ".claude", ".git", "node_modules", "build", "dependencies"}
        for path in root.rglob("*.proto"):
            if any(part in skip for part in path.parts):
                continue
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            m = _PROTO_DECL_RE.search(text)
            if not m:
                continue
            name = m.group(1)
            base = _proto_base_type(text, m.end())
            if base and name not in self._base:
                self._base[name] = base

    def base_of(self, name: str) -> str | None:
        """The immediate node type a PROTO's body starts with."""
        return self._base.get(name)

    def resolve(self, name: str) -> str:
        """Follow the derivation chain to a (probably) built-in node type."""
        if name in self._resolved:
            return self._resolved[name]
        seen: set[str] = set()
        cur = name
        while cur in self._base and cur not in seen:
            seen.add(cur)
            cur = self._base[cur]
        for n in seen:
            self._resolved[n] = cur
        self._resolved[name] = cur
        return cur

    def known(self) -> list[str]:
        return sorted(self._base)


def _proto_base_type(text: str, header_start: int) -> str | None:
    """The node type at the head of a PROTO body, given the offset of ``[``."""
    # Walk the header's bracket to its match, honouring strings/comments.
    i = header_start - 1
    depth = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c == "#":
            while i < n and text[i] != "\n":
                i += 1
        elif c == '"':
            i += 1
            while i < n and text[i] != '"':
                if text[i] == "\\":
                    i += 1
                i += 1
        elif c == "[":
            depth += 1
        elif c == "]":
            depth -= 1
            if depth == 0:
                i += 1
                break
        i += 1
    body = text[i:]
    # Skip whitespace/comments/'{'/template blocks to the first node type.
    m = re.search(
        r"\{(?:\s|\#[^\n]*|%<.*?>%)*(?:DEF\s+[A-Za-z_][A-Za-z0-9_]*\s+)?"
        r"([A-Z][A-Za-z0-9_]*)\s*\{",
        body,
        re.DOTALL,
    )
    return m.group(1) if m else None


_PROTO_INDEX_CACHE: dict[str, ProtoIndex] = {}


def proto_index_for(repo_root: Path | str) -> ProtoIndex:
    """Cached :class:`ProtoIndex` for a checkout (scan is ~0.3 s, once)."""
    key = str(Path(repo_root).resolve())
    idx = _PROTO_INDEX_CACHE.get(key)
    if idx is None:
        root = Path(key)
        roots = [p for p in (root / "projects", root / "resources", root / "tests",
                             root / "distribution") if p.is_dir()]
        idx = ProtoIndex(roots or [root])
        _PROTO_INDEX_CACHE[key] = idx
    return idx


# --------------------------------------------------------------------------
# Small 3x3 rotation helpers (transform accumulation)
# --------------------------------------------------------------------------

Mat3 = tuple[tuple[float, float, float], ...]
IDENTITY: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def axis_angle_to_matrix(x: float, y: float, z: float, angle: float) -> Mat3:
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12 or abs(angle) < 1e-12:
        return IDENTITY
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def mat_apply(m: Mat3, v: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )
