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

"""Read a world file far enough to NAME things.

The engine builds the physics; this reader never does. It parses the world
(and the PROTOs it instantiates) so the core can tell which joint is which
motor, which Solid is which DEF, and where each stood when the world was
written. PROTO templates are JavaScript and are run with `node`; without
`node` a templated PROTO cannot be expanded and `TemplateUnavailable` says so.
"""

from __future__ import annotations

import json
import math
import os
import re
import subprocess

from .runtime import HOME

_TOK = re.compile(r'[\s,]+|#[^\n]*|"(?:[^"\\]|\\.)*"|[\[\]{}]|[-+]?(?:\d+\.?\d*(?:[eE][-+]?\d+)?|\.\d+(?:[eE][-+]?\d+)?)(?![A-Za-z_])'
                  r'|[A-Za-z_][A-Za-z0-9_+\-.:#]*')


class TemplateUnavailable(RuntimeError):
    """A PROTO template could not be run (no `node`, or the template failed)."""


class Str(str):
    """A quoted string of the file (an identifier is a tuple token, a number a float)."""


class Ref:
    def __init__(self, kind, name):
        self.kind, self.name = kind, name          # kind: USE | IS

    def __repr__(self):
        return "%s %s" % (self.kind, self.name)


class Node:
    def __init__(self, type_, defname=None):
        self.type, self.defname, self.fields = type_, defname, {}

    def get(self, name, default=None):
        v = self.fields.get(name)
        return default if v is None or len(v) == 0 else v

    def __repr__(self):
        return "<%s%s>" % (self.type, " DEF " + self.defname if self.defname else "")


def tokenize(text):
    out, i = [], 0
    while i < len(text):
        m = _TOK.match(text, i)
        if not m:
            raise ValueError("cannot tokenize at %r" % text[i:i + 40])
        t = m.group(0)
        i = m.end()
        if t[0].isspace() or t[0] in "#,":
            continue
        if t[0] == '"':
            out.append(Str(json.loads(t.replace("\n", "\\n"))))
        elif t in "[]{}":
            out.append(("p", t))
        elif t[0] in "+-.0123456789":
            out.append(float(t))
        else:
            out.append(("id", t))
    return out


_KW = {"TRUE": True, "FALSE": False, "NULL": None}


def _is_field_name(tok, nxt=None):
    return (isinstance(tok, tuple) and tok[0] == "id" and tok[1] not in ("TRUE", "FALSE", "NULL", "DEF", "USE", "IS")
            and nxt != ("p", "{"))


def parse_item(toks, i):
    t = toks[i]
    if isinstance(t, (float, Str)):
        return t, i + 1
    if t == ("p", "["):
        items, i = [], i + 1
        while toks[i] != ("p", "]"):
            v, i = parse_item(toks, i)
            items.append(v)
        return items, i + 1
    if t[0] == "id":
        if t[1] in _KW:
            return _KW[t[1]], i + 1
        if t[1] in ("USE", "IS"):
            return Ref(t[1], toks[i + 1][1]), i + 2
        return parse_node(toks, i)
    raise ValueError("unexpected token %r" % (t,))


def parse_node(toks, i):
    defname = None
    if toks[i] == ("id", "DEF"):
        defname = toks[i + 1][1]
        i += 2
    n = Node(toks[i][1], defname)
    i += 1
    if toks[i] != ("p", "{"):
        raise ValueError("expected { after %s" % n.type)
    i += 1
    while toks[i] != ("p", "}"):
        if not _is_field_name(toks[i], toks[i + 1]):
            raise ValueError("expected a field name in %s, got %r" % (n.type, toks[i]))
        name = toks[i][1]
        i += 1
        vals = []
        while toks[i] != ("p", "}") and not _is_field_name(toks[i], toks[i + 1]):
            v, i = parse_item(toks, i)
            vals.append(v)
        n.fields[name] = vals[0] if len(vals) == 1 and isinstance(vals[0], list) else vals
    return n, i + 1


def parse_file(text):
    """-> (externprotos {Name: url}, [top-level nodes])"""
    toks = tokenize(text)
    ext, nodes, i = {}, [], 0
    while i < len(toks):
        if toks[i] in (("id", "EXTERNPROTO"), ("id", "IMPORTABLE")):
            if toks[i] == ("id", "IMPORTABLE"):
                i += 1
            url = toks[i + 1]
            ext[os.path.basename(url)[:-6]] = url
            i += 2
        else:
            n, i = parse_node(toks, i)
            nodes.append(n)
    return ext, nodes


# ---------------------------------------------------------------- PROTO expansion
def resolve_url(url, base_dir):
    if url.startswith("omnisim://"):
        return str(HOME) + "/" + url[len("omnisim://"):]
    return os.path.normpath(os.path.join(base_dir, url))


def _js_value(ftype, v):
    if isinstance(v, Ref) or v is None:
        return None
    seq = v if isinstance(v, list) else [v]

    def one(t, xs):
        if t in ("SFFloat", "SFInt32"):
            return xs[0]
        if t == "SFBool":
            return bool(xs[0])
        if t == "SFString":
            return str(xs[0])
        if t == "SFVec2f":
            return {"x": xs[0], "y": xs[1]}
        if t == "SFVec3f":
            return {"x": xs[0], "y": xs[1], "z": xs[2]}
        if t == "SFColor":
            return {"r": xs[0], "g": xs[1], "b": xs[2]}
        if t == "SFRotation":
            return {"x": xs[0], "y": xs[1], "z": xs[2], "a": xs[3]}
        if t == "SFNode":
            return {"node_name": xs[0].type} if xs and isinstance(xs[0], Node) else None
        return None
    if ftype.startswith("MF"):
        st = "SF" + ftype[2:]
        w = {"SFVec2f": 2, "SFVec3f": 3, "SFColor": 3, "SFRotation": 4}.get(st, 1)
        return [one(st, seq[k:k + w]) for k in range(0, len(seq), w)]
    return one(ftype, seq)


_PROTO_CACHE = {}
_FIELD_KW = ("field", "hiddenField", "unconnectedField", "deprecatedField", "vrmlField")


def load_proto(path):
    if path in _PROTO_CACHE:
        return _PROTO_CACHE[path]
    with open(path, encoding="utf-8") as f:
        text = f.read()
    m = re.search(r'^\s*PROTO\s+(\w+)\s*\[', text, re.M)
    if not m:
        raise ValueError("no PROTO header in %s" % path)
    depth, j = 1, m.end()
    while depth:                                   # the interface's closing ] (strings may hold brackets)
        c = text[j]
        if c == '"':
            j = text.index('"', j + 1)
        elif c == "#":
            j = text.index("\n", j)
        elif c == "[":
            depth += 1
        elif c == "]":
            depth -= 1
        j += 1
    iface_txt, rest = text[m.end():j - 1], text[j:]
    body = rest[rest.index("{") + 1:rest.rindex("}")]
    toks = tokenize(iface_txt)
    fields, i = [], 0
    while i < len(toks):
        if toks[i][1] not in _FIELD_KW:
            raise ValueError("unexpected %r in the interface of %s" % (toks[i], path))
        ftype = toks[i + 1][1]
        i += 2
        if toks[i] == ("p", "{"):                  # allowed-values restriction
            while toks[i] != ("p", "}"):
                i += 1
            i += 1
        name = toks[i][1]
        i += 1
        vals = []
        while i < len(toks) and not (isinstance(toks[i], tuple) and toks[i][0] == "id" and toks[i][1] in _FIELD_KW):
            v, i = parse_item(toks, i)
            vals.append(v)
        fields.append((ftype, name, vals[0] if len(vals) == 1 and isinstance(vals[0], list) else vals))
    ext = {os.path.basename(u)[:-6]: u for u in re.findall(r'^\s*EXTERNPROTO\s+"([^"]+)"', text, re.M)}
    p = {"name": m.group(1), "fields": fields, "body": body, "ext": ext, "dir": os.path.dirname(path), "path": path}
    _PROTO_CACHE[path] = p
    return p


def run_template(body, js_fields):
    if "%<" not in body:
        return body
    parts = re.split(r'(%<=?.*?>%)', body, flags=re.S)
    js = ["let __out='';", "const fields=" + json.dumps(js_fields) + ";",
          "const context={os:'windows',world:'',proto:'',project_path:'',webots_version:{major:'R2025a'}};"]
    for p in parts:
        if p.startswith("%<="):
            js.append("__out+=(" + p[3:-2].strip() + ");")
        elif p.startswith("%<"):
            js.append(p[2:-2])
        elif p:
            js.append("__out+=" + json.dumps(p) + ";")
    js.append("process.stdout.write(__out);")
    try:
        r = subprocess.run(["node", "-"], input="\n".join(js), capture_output=True, text=True, encoding="utf-8")
    except OSError as exc:
        raise TemplateUnavailable("PROTO templates need node on PATH: %r" % (exc,)) from exc
    if r.returncode != 0:
        raise TemplateUnavailable("PROTO template failed: " + r.stderr[-400:])
    return r.stdout


def expand(node, ext, base_dir):
    """Expand PROTO instances until `node` is a base node. -> (node, ext, base_dir) of the result."""
    while node.type in ext:
        proto = load_proto(resolve_url(ext[node.type], base_dir))
        vals, jsf = {}, {}
        for ftype, name, default in proto["fields"]:
            v = node.fields.get(name, default)
            vals[name] = v
            jsf[name] = {"value": _js_value(ftype, v), "defaultValue": _js_value(ftype, default)}
        toks = tokenize(run_template(proto["body"], jsf))
        root, _ = parse_node(toks, 0)
        _bind_is(root, vals)
        if node.defname and not root.defname:
            root.defname = node.defname
        node, ext, base_dir = root, proto["ext"], proto["dir"]
    return node, ext, base_dir


def _bind_is(n, vals):
    for k, v in list(n.fields.items()):
        items = v if isinstance(v, list) else [v]
        if len(items) == 1 and isinstance(items[0], Ref) and items[0].kind == "IS":
            n.fields[k] = vals.get(items[0].name, [])
            continue
        for it in items:
            if isinstance(it, Node):
                _bind_is(it, vals)
            elif isinstance(it, list):
                for x in it:
                    if isinstance(x, Node):
                        _bind_is(x, vals)


def child_nodes(node):
    """Every Node held directly in any field of `node`."""
    for v in node.fields.values():
        for it in (v if isinstance(v, list) else [v]):
            if isinstance(it, Node):
                yield it
            elif isinstance(it, list):
                for x in it:
                    if isinstance(x, Node):
                        yield x


# ---------------------------------------------------------------- small maths (quaternions are x, y, z, w)
IDQ = (0.0, 0.0, 0.0, 1.0)


def q_axis_angle(x, y, z, a):
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    s = math.sin(a / 2) / n
    return (x * s, y * s, z * s, math.cos(a / 2))


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def q_rot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def q_rpy(r, p, y):
    return q_mul(q_mul(q_axis_angle(0, 0, 1, y), q_axis_angle(0, 1, 0, p)), q_axis_angle(1, 0, 0, r))


def add3(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
