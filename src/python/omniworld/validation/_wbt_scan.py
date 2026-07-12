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

Parsing a Webots world file is out of scope for Tier 1 — the full PROTO
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


def scan_placements(text: str) -> list[PlacedNode]:
    """Return every placed URDFRobot / DEF Transform with a translation."""
    out: list[PlacedNode] = []
    for m in _NODE_RE.finditer(text):
        body = m.group("body")
        tm = _TRANSLATION_RE.search(body)
        if not tm:
            continue
        nm = _NAME_RE.search(body)
        out.append(
            PlacedNode(
                node_type=m.group("node"),
                def_name=m.group("def"),
                name=nm.group(1) if nm else None,
                translation=(
                    float(tm.group(1)),
                    float(tm.group(2)),
                    float(tm.group(3)),
                ),
            )
        )
    return out
