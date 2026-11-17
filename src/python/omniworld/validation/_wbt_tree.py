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
from collections import OrderedDict
from dataclasses import dataclass, field as dc_field
from pathlib import Path

__all__ = [
    "WbtNode",
    "parse_wbt",
    "parse_wbt_cached",
    "TextMemo",
    "MISS",
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

# Tokenising is two C-level passes, not one Python loop. The first pass blanks
# every comment and ``%< ... >%`` block (a string is matched by the same
# alternation, in the same order, and kept verbatim -- so a ``#`` or ``%<``
# inside a string still belongs to the string, exactly as before). The second
# pass is a bare ``findall``: no per-match ``lastgroup`` / ``group()`` calls,
# which on a 1 M-token generated world was ~1 µs per token of pure overhead
# (1.1 s -> 0.3 s, measured 2026-09-02 on the Mars world). A skipped span is
# replaced by a single space, never dropped, so the tokens on either side of
# a template block never fuse (``12%<..>%34`` stays ``12``, ``34``).
_SKIP_RE = re.compile(
    r"""
      (?P<js>%<.*?>%)                       # PROTO template block (skipped)
    | (?P<comment>\#[^\n]*)                 # comment to end of line
    | (?P<string>"(?:\\.|[^"\\])*"?)        # string: kept as is
    """,
    re.VERBOSE | re.DOTALL,
)

# Number first: on a generated world nearly every token is one. The four
# non-fallback alternatives start with disjoint characters (digit/sign/dot,
# letter/underscore, brace, quote), so at most one can match at a position
# and their order cannot change which token comes out.
_TOKEN_RE = re.compile(
    r"""
      [+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?   # number
    | [A-Za-z_][A-Za-z0-9_]*                # ident
    | [{}\[\]]                              # punct
    | "(?:\\.|[^"\\])*"?                    # string (tolerates unterminated)
    | \S                                    # other
    """,
    re.VERBOSE,
)

_KEYWORDS = {"DEF", "USE", "IS", "NULL", "TRUE", "FALSE", "PROTO", "EXTERNPROTO"}

# The three characters at which ``_SKIP_RE`` can match at all.
_SKIP_STARTS = ('"', "#", "%<")


def _blank_skips(text: str) -> str:
    """``text`` with every comment and ``%< ... >%`` block replaced by a space.

    Same result as ``_SKIP_RE.sub(...)``, but the scan jumps between the
    positions where the pattern can match with ``str.find`` (a C memchr)
    instead of trying the alternation at all 8 M positions of a generated
    world (0.27 s -> ~5 ms). Each needle is re-searched only once its cached
    position has been consumed, so the whole thing stays linear.
    """
    parts: list[str] = []
    pos = 0
    nxt = {needle: text.find(needle) for needle in _SKIP_STARTS}
    while True:
        p = -1
        for cand in nxt.values():
            if cand >= 0 and (p < 0 or cand < p):
                p = cand
        if p < 0:
            break
        m = _SKIP_RE.match(text, p)
        if m is None:
            # An unterminated ``%<``: ``sub`` would step past the ``%`` too.
            end = p + 1
        else:
            end = m.end()
            if m.lastgroup != "string":
                parts.append(text[pos:p])
                parts.append(" ")
                pos = end
        for needle, cand in nxt.items():
            if 0 <= cand < end:
                nxt[needle] = text.find(needle, end)
    parts.append(text[pos:])
    return "".join(parts)


def _tokenize(text: str) -> list[str]:
    if "#" in text or "%<" in text:
        text = _blank_skips(text)
    return _TOKEN_RE.findall(text)


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

    # The two loops below are the parser's hot path: on a generated world
    # nearly every token is a number inside a bracketed mesh / heightmap
    # field. They index the token list directly instead of going through
    # ``peek()`` / ``next()`` per token, and only fall back to ``try_node``
    # when the token could start one (``DEF``, ``USE`` and every node type
    # begin with an uppercase letter; ``try_node`` returns ``None`` without
    # moving the cursor for anything else). Same tokens in, same tree out.

    def _parse_bracket(self) -> list:
        out: list = []
        t = self.t
        n = len(t)
        while True:
            i = self.i
            if i >= n:
                return out
            tok = t[i]
            if tok == "]":
                self.i = i + 1
                return out
            if tok[0].isupper():
                # ``try_node`` may return None AFTER moving the cursor (a
                # ``DEF`` whose name is not followed by a node, e.g. the
                # template ``DEF %<= n >% Pose {`` once the block is
                # blanked), so re-read the cursor rather than reuse ``tok``.
                sub = self.try_node()
                out.append(sub if sub is not None else self.next())
                continue
            out.append(tok)
            self.i = i + 1

    def _parse_scalars(self) -> list[str]:
        out: list[str] = []
        t = self.t
        n = len(t)
        i = self.i
        while i < n:
            tok = t[i]
            c = tok[0]
            if c in "}][":
                break
            if c.islower() or c == "_":
                if tok.isidentifier():
                    # Next field starts here — unless it is the value of an
                    # enum-ish field, which OmniSim always quotes, so this
                    # is safe.
                    break
            elif c.isupper():
                if tok == "DEF" or tok == "USE":
                    break
                if (tok not in _KEYWORDS and tok.isidentifier()
                        and i + 1 < n and t[i + 1] == "{"):
                    break
            out.append(tok)
            i += 1
        self.i = i
        return out


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


class TextMemo:
    """A small, bounded, exact-match memo keyed on a text.

    Lookups hash the text (Python caches a ``str``'s hash on the object, and
    computing it is ~10 ms for 8 MB) and then compare the stored text for
    equality, so a hash collision is a miss, never a wrong answer. The memo
    is a pure cache: it only ever hands back what the same input produced.
    Least-recently-used entries are evicted past ``max_entries``.
    """

    def __init__(self, max_entries: int) -> None:
        self.max_entries = max_entries
        self._entries: OrderedDict[tuple[int, int], tuple[str, object]] = OrderedDict()
        self.hits = 0
        self.misses = 0

    def get(self, text: str):
        """The memoised value for ``text``, or :data:`MISS`."""
        key = (len(text), hash(text))
        hit = self._entries.get(key)
        if hit is not None and hit[0] == text:
            self._entries.move_to_end(key)
            self.hits += 1
            return hit[1]
        self.misses += 1
        return MISS

    def put(self, text: str, value: object) -> None:
        key = (len(text), hash(text))
        self._entries[key] = (text, value)
        self._entries.move_to_end(key)
        while len(self._entries) > self.max_entries:
            self._entries.popitem(last=False)

    def clear(self) -> None:
        self._entries.clear()
        self.hits = 0
        self.misses = 0

    def __len__(self) -> int:
        return len(self._entries)


MISS = object()

# The viewpoint check parses the whole world; a generated world is ~1 M
# tokens (0.65 s to parse after the tokeniser/parser work above), so a second
# analysis of the same text -- ``omniworld validate`` after ``generate`` in
# one process, or several checks on one world -- reuses the tree. Two
# entries: a parsed 8 MB Mars world is 50 MB live (tracemalloc, 2026-09-02),
# and the repeat callers this exists for analyse the same text back to back.
_PARSE_MEMO = TextMemo(max_entries=2)


def parse_wbt_cached(text: str) -> list[WbtNode]:
    """:func:`parse_wbt` through a process-local, bounded, exact-match memo.

    The returned tree is SHARED with every later caller that passes the same
    text: treat it as read-only. A caller that wants to mutate nodes must use
    :func:`parse_wbt`, which always builds a fresh tree.
    """
    nodes = _PARSE_MEMO.get(text)
    if nodes is MISS:
        nodes = parse_wbt(text)
        _PARSE_MEMO.put(text, nodes)
    return nodes


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
            # Match the skip list against the path BELOW the scan root, never the root's own
            # ancestors: an absolute root that merely lives under a directory with one of these
            # names (a checkout under .../build/, or a temp dir under C:/msys64/tmp) otherwise
            # skips every file and the index silently comes back empty.
            try:
                parts = path.relative_to(root).parts
            except ValueError:
                parts = path.parts
            if any(part in skip for part in parts):
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
