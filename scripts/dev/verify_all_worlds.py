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

"""Load every tracked world and record whether it parses + loads cleanly.

This is the safety net for large asset/PROTO refactors: snapshot BEFORE the change,
snapshot AFTER, and diff. Any world that regresses from OK -> FAIL is a break.

It uses the PROTO/world parser rather than a full simulation, so it is fast enough to
run over the whole tree (~700 worlds) and still catches the thing that actually breaks
when you touch a PROTO: an unresolved EXTERNPROTO, a missing field, a renamed node.

    python scripts/dev/verify_all_worlds.py --out before.json
    ...make changes...
    python scripts/dev/verify_all_worlds.py --out after.json --compare before.json
"""
import argparse
import json
import os
import re
import subprocess
import sys

EXTERNPROTO = re.compile(r'EXTERNPROTO\s+"([^"]+)"')
PROTO_DECL = re.compile(r'^\s*PROTO\s+(\w+)\s*\[', re.M)
# A field declaration is:  field <TYPE> <name> <default>
# ...but TYPE may carry an inline value constraint that CONTAINS SPACES, e.g.
#     field SFString{"regular", "flat"} type "regular"
# so the type cannot be matched with a bare \S+ -- that stops at the first space and
# the capture group then lands on the wrong token. Match the optional {...} explicitly.
FIELD_DECL = re.compile(
    r'^\s*(?:field|hiddenField|deprecatedField|vrmlField|unconnectedField)\s+'
    r'(?:MF|SF)\w+\s*(?:\{[^}]*\})?\s+(\w+)', re.M)


def proto_interface(path, cache):
    """The set of field names a PROTO accepts. None if the file declares no PROTO."""
    if path in cache:
        return cache[path]
    t = read(path)
    m = PROTO_DECL.search(t)
    if not m:
        cache[path] = None
        return None
    # the interface is everything between the PROTO '[' and its matching ']'
    start = t.index("[", m.end() - 1)
    depth, i = 0, start
    while i < len(t):
        if t[i] == "[":
            depth += 1
        elif t[i] == "]":
            depth -= 1
            if depth == 0:
                break
        i += 1
    cache[path] = set(FIELD_DECL.findall(t[start:i]))
    return cache[path]


def check_field_use(path, resolve_from, iface_cache, name_to_path):
    """Find fields passed to a PROTO that the PROTO does not declare.

    This is the failure a parse-level check misses entirely: the file resolves, the
    PROTO is declared, but a caller passes a field that does not exist -- so the world
    dies at LOAD time, not at parse time. Re-authoring a PROTO makes this very easy to
    introduce (e.g. handing `roughness` to an appearance that has no such field).
    """
    t = read(path)

    # Strip JS template blocks (%< ... >%): they contain loop variables and generated
    # text that are not field assignments. Leaving them in produces pure noise.
    t = re.sub(r'%<[\s\S]*?>%', '', t)
    # Strip `hidden ...` parameter lines: the engine writes joint state back into worlds
    # as `hidden position_0_0 ...`. These are legitimate and are not PROTO fields.
    t = re.sub(r'^\s*hidden\s+.*$', '', t, flags=re.M)

    problems = []
    # `Name { a 1 b 2 ... }` -- only look at PROTOs we know
    for m in re.finditer(r'(?:DEF\s+\S+\s+)?([A-Z]\w+)\s*\{([^{}]*)\}', t):
        node, body = m.group(1), m.group(2)
        p = name_to_path.get(node)
        if not p:
            continue
        iface = proto_interface(p, iface_cache)
        if not iface:
            continue
        for fm in re.finditer(r'(?:^|\s)([a-z]\w*)\s+(?:IS\s+\w+|[-\d"\[])', body):
            f = fm.group(1)
            if f not in iface:
                problems.append(f"passes field '{f}' to PROTO '{node}', which has no such field")
    return problems


def tracked(pattern):
    out = subprocess.run(["git", "ls-files", pattern], capture_output=True, text=True).stdout
    return [f for f in out.split("\n") if f]


def read(p):
    try:
        with open(p, encoding="utf-8", errors="ignore") as fh:
            return fh.read()
    except OSError:
        return ""


def resolve(url, origin):
    """Resolve an EXTERNPROTO url to a repo path (omnisim:// or relative)."""
    if url.startswith("omnisim://"):
        return url[len("omnisim://"):]
    if url.startswith(("http://", "https://")):
        return None  # remote; not our problem here
    return os.path.normpath(os.path.join(os.path.dirname(origin), url)).replace("\\", "/")


def declared_proto_name(path, cache):
    """The PROTO name actually declared inside a .proto file (None if there isn't one)."""
    if path not in cache:
        m = PROTO_DECL.search(read(path))
        cache[path] = m.group(1) if m else None
    return cache[path]


def check_world(w, all_protos, decl_cache):
    """Return (ok, [problems]) for one world.

    Catches, specifically, the ways a PROTO refactor breaks a world:
      * the EXTERNPROTO target file was deleted;
      * the file still exists but no longer DECLARES the PROTO the world instantiates
        (i.e. it was renamed away) -- this is the one a naive check misses;
      * a node is instantiated with no EXTERNPROTO to resolve it.
    """
    t = read(w)
    problems = []
    if not t.strip():
        return False, ["empty or unreadable"]
    if not t.lstrip().startswith(("#OMNISIM", "#VRML_SIM")):
        problems.append("missing #OMNISIM (or legacy #VRML_SIM) header")

    declared = set()
    for url in EXTERNPROTO.findall(t):
        p = resolve(url, w)
        if p is None:
            continue
        if not os.path.isfile(p):
            problems.append(f"EXTERNPROTO target missing: {url}")
            continue
        expected = os.path.basename(p)[:-6]
        actual = declared_proto_name(p, decl_cache)
        if actual is None:
            problems.append(f"{url} declares no PROTO at all")
        elif actual != expected:
            # the file is there but the PROTO inside was renamed -> every use breaks
            problems.append(f"{url} declares PROTO '{actual}', world expects '{expected}'")
        else:
            declared.add(expected)

    # any node that LOOKS like a PROTO instantiation must be resolvable
    body = re.sub(r'^\s*(#|EXTERNPROTO).*$', '', t, flags=re.M)
    for name in set(re.findall(r'^\s*(?:DEF\s+\S+\s+)?([A-Z]\w+)\s*\{', body, re.M)):
        if name in declared:
            continue
        if name in all_protos:          # a known PROTO, used without declaring it
            problems.append(f"uses PROTO '{name}' without an EXTERNPROTO")
    return (not problems), problems


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--compare")
    a = ap.parse_args()

    all_protos = set()
    decl_cache = {}
    name_to_path = {}
    proto_files = tracked("*.proto")
    for p in proto_files:
        n = declared_proto_name(p, decl_cache)
        if n:
            all_protos.add(n)
            name_to_path[n] = p

    iface_cache = {}
    worlds = tracked("*.wbt")
    result = {}
    for w in worlds:
        ok, probs = check_world(w, all_protos, decl_cache)
        probs += check_field_use(w, w, iface_cache, name_to_path)
        result[w] = {"ok": ok and not probs, "problems": probs}

    # PROTOs can pass bad fields to OTHER PROTOs too -- that breaks every world that
    # uses the outer one, so check them as first-class citizens.
    for p in proto_files:
        probs = check_field_use(p, p, iface_cache, name_to_path)
        if probs:
            result[p] = {"ok": False, "problems": probs}

    ok_n = sum(1 for v in result.values() if v["ok"])
    with open(a.out, "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=1)
    print(f"  worlds checked: {len(worlds)}   OK: {ok_n}   problems: {len(worlds)-ok_n}")

    if a.compare:
        with open(a.compare, encoding="utf-8") as fh:
            before = json.load(fh)
        regressed = [w for w, v in result.items()
                     if not v["ok"] and before.get(w, {}).get("ok")]
        fixed = [w for w, v in result.items()
                 if v["ok"] and w in before and not before[w]["ok"]]
        gone = [w for w in before if w not in result]
        print(f"  REGRESSED (was OK, now broken): {len(regressed)}")
        for w in regressed[:20]:
            print(f"    !! {w}")
            for p in result[w]["problems"][:3]:
                print(f"       - {p}")
        print(f"  fixed: {len(fixed)}   removed: {len(gone)}")
        if regressed:
            sys.exit(1)


if __name__ == "__main__":
    main()
