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

"""Minimal WBT scanner used by the Tier 1 in-process validators.

Parsing a OmniSim world file is out of scope for Tier 1 — the full PROTO
expansion is an entire subsystem. For validators that only care about
placement (spawn reachability, prop overlap), a cheap regex scan of
``URDFRobot`` and ``DEF ... Transform`` blocks is enough.

This scanner is deliberately narrow: it recognises axis-aligned
``translation`` fields on URDFRobot and DEF Transform nodes at the top
level of a world. Anything more structural is a later-tier concern.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from ._wbt_tree import MISS, TextMemo

# Match e.g.:
#   URDFRobot {
#     url "..."
#     translation 1.0 -2.5 0.1
#     ...
#   }
# We capture the translation triple, and the DEF name if any.
_NODE_RE = re.compile(
    r"(?:DEF\s+(?P<def>[A-Za-z_][\w\-]*)\s+)?"
    r"(?P<node>URDFRobot|Transform)\s*\{"
    r"(?P<body>[^{}]*)\}",
    re.MULTILINE,
)

_TRANSLATION_RE = re.compile(
    r"translation\s+"
    r"(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s+"
    r"(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s+"
    r"(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)"
)

_NAME_RE = re.compile(r"name\s+\"([^\"]+)\"")


@dataclass(frozen=True)
class PlacedNode:
    node_type: str             # "URDFRobot" or "Transform"
    def_name: str | None       # DEF identifier, if any
    name: str | None           # ``name "..."`` field value, if any
    translation: tuple[float, float, float]


# Two validators (spawn, overlap) scan the same text back to back, and the
# scan is a regex pass over the whole world (0.35 s on an 8 MB generated
# Mars world). Memoised per text, exact-match; see ``TextMemo``.
_SCAN_MEMO = TextMemo(max_entries=4)


def scan_placements(text: str) -> list[PlacedNode]:
    """Return every placed URDFRobot / DEF Transform with a translation.

    Memoised on ``text``; each call returns a fresh list (the entries are
    frozen), so callers may reorder or filter it freely.
    """
    found = _SCAN_MEMO.get(text)
    if found is MISS:
        found = _scan_placements(text)
        _SCAN_MEMO.put(text, found)
    return list(found)


# ``_NODE_RE`` split in two for the ``str.find``-driven walk below: the
# optional DEF prefix, anchored to END at the node keyword (``\Z`` with an
# ``endpos``), and the tail after it.
_DEF_PREFIX_RE = re.compile(r"DEF\s+(?P<def>[A-Za-z_][\w\-]*)\s+\Z")
_NODE_TAIL_RE = re.compile(r"\s*\{(?P<body>[^{}]*)\}")
_NODE_KEYWORDS = ("URDFRobot", "Transform")


def _iter_nodes(text: str):
    """Yield ``(def_name, node_type, body)`` for every ``_NODE_RE`` match, in
    order, WITHOUT trying the pattern at all ~8 M positions of a generated
    world (0.36 s -> ~5 ms on the Mars world, 2026-09-02).

    A match always contains one of the node keywords, so the walk jumps
    between keyword occurrences with ``str.find``; at each one the tail is
    matched forward and the optional ``DEF name`` prefix backward, on the
    span since the previous match (a prefix can never start inside an
    earlier match, exactly as ``finditer`` never revisits consumed text).
    An occurrence whose tail does not match is stepped over, and every
    keyword inside a consumed body is skipped, both as ``finditer`` does.
    """
    pos = 0
    nxt = {kw: text.find(kw) for kw in _NODE_KEYWORDS}
    while True:
        q = -1
        kw = ""
        for cand_kw, cand in nxt.items():
            if cand >= 0 and (q < 0 or cand < q):
                q, kw = cand, cand_kw
        if q < 0:
            return
        tail = _NODE_TAIL_RE.match(text, q + len(kw))
        if tail is None:
            end = q + 1
        else:
            prefix = _DEF_PREFIX_RE.search(text, pos, q)
            yield (prefix.group("def") if prefix else None, kw, tail.group("body"))
            end = tail.end()
            pos = end
        for cand_kw, cand in nxt.items():
            if 0 <= cand < end:
                nxt[cand_kw] = text.find(cand_kw, end)


def _scan_placements(text: str) -> list[PlacedNode]:
    out: list[PlacedNode] = []
    for def_name, node_type, body in _iter_nodes(text):
        tm = _TRANSLATION_RE.search(body)
        if not tm:
            continue
        nm = _NAME_RE.search(body)
        out.append(
            PlacedNode(
                node_type=node_type,
                def_name=def_name,
                name=nm.group(1) if nm else None,
                translation=(
                    float(tm.group(1)),
                    float(tm.group(2)),
                    float(tm.group(3)),
                ),
            )
        )
    return out
