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

"""The **declaration side** of the Webots identity rule: reading the ``.wbt``.

``IdentityRule.declared_count`` asks the *artifact* the same question the roster
asks the *scene*, and on upstream Webots the artifact answers it differently than
it does on OmniSim:

* **OmniSim** expands ``URDFRobot { url "husky.urdf" }`` into a plain ``Robot``
  at parse time (``WbUrdfImporter::expandUrdfRobotBlocks``), so the model name
  survives **only** in the file. The declaration count is a count of URDF url
  references, and it is the *only* place the name exists.
* **Upstream Webots** instantiates a PROTO: ``DEF UGV0 Moose { ... }``. The
  PROTO name is in the file **and** in the live scene (``getTypeName()`` on the
  instance returns it), so here the declaration count is a corroboration of
  something the scene can also answer, not a substitute for it.

That asymmetry is the reason ``IdentityRule`` publishes both rules as strings:
"A1.1 passed on both" would otherwise hide the fact that one side counted URDF
references and the other counted PROTO instantiations.

Two more things are read from the world text because they are load-bearing for a
cross-sim comparison and cannot be read anywhere else:

``WorldInfo.coordinateSystem``
    R2022b and later default to ``"ENU"`` (z up, which is what the neutral
    schema's ``UNITS`` demands) but ``"NUE"`` (y up) is still accepted for
    backward compatibility. A NUE world's ``xyz`` is not in the bundle's frame,
    and a grader that ignores this measures a robot's *height* as its northing.

``WorldInfo.physics``
    The path to an **ODE physics plugin**. It does not change which engine ran
    (there is only one upstream), but it can change the forces, so it is
    recorded on the attribution rather than thrown away.
"""

from __future__ import annotations

import re
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from agentbench.common import worldtext as common_wt  # noqa: E402

# A node instantiation at brace depth 0: an optional DEF, then a node type
# (VRML node types are capitalised; every field name is not, which is what makes
# depth-0 scanning unambiguous once block interiors are skipped).
_NODE_HEAD = re.compile(
    r"(?:(?<![A-Za-z0-9_])DEF\s+([A-Za-z0-9_+\-]+)\s+)?"
    r"(?<![A-Za-z0-9_])([A-Z][A-Za-z0-9_]*)\s*\{")

_USE = re.compile(r"(?<![A-Za-z0-9_])USE\s+([A-Za-z0-9_+\-]+)")
_EXTERNPROTO = re.compile(r'(?m)^\s*(?:IMPORTABLE\s+)?EXTERNPROTO\s+"([^"]+)"')
_SF_STRING = r'(?<![A-Za-z0-9_])%s\s+"([^"]*)"'

# Keywords that can start a capitalised token at depth 0 and are NOT nodes.
_NOT_A_NODE = ("PROTO", "EXTERNPROTO", "IMPORTABLE", "DEF", "USE", "IS")


def scan(world_path):
    """Every depth-0 node instantiation in a ``.wbt``, plus the world's frame.

    Returns a dict::

        {"nodes": [{"type", "def", "name", "controller", "offset"}],
         "uses": [str],                  # USE <DEF> references, counted apart
         "externprotos": [str],          # the URLs the world imports
         "coordinate_system": "ENU"|"NUE"|None,
         "physics_plugin": str|None,
         "basic_time_step_ms": float|None,
         "error": str|None}

    Nested nodes are skipped: only the top level of the file is walked, which is
    where scene instances live. ``error`` is set instead of raising.
    """
    out = {"nodes": [], "uses": [], "externprotos": [],
           "coordinate_system": None, "physics_plugin": None,
           "basic_time_step_ms": None, "error": None}
    try:
        raw = Path(world_path).read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        out["error"] = "unreadable: %r" % (exc,)
        return out

    # EXTERNPROTO lines are read BEFORE comment stripping only in the sense that
    # they are ordinary lines; strip first so a commented-out import is not
    # counted as one.
    text = common_wt.strip_comments(raw)
    out["externprotos"] = _EXTERNPROTO.findall(text)
    out["uses"] = _USE.findall(text)

    i = 0
    while True:
        m = _NODE_HEAD.search(text, i)
        if not m:
            break
        # Reuse the harness-side quote-aware brace walker rather than write a
        # second one: two scanners that disagree about where a block ends is a
        # class of bug this suite cannot afford.
        inside, after = common_wt._block_body(text, m.end() - 1)
        kind = m.group(2)
        if kind not in _NOT_A_NODE:
            out["nodes"].append({
                "type": kind,
                "def": m.group(1) or "",
                "name": _field(inside, "name"),
                "controller": _field(inside, "controller"),
                "offset": m.start(),
            })
            if kind == "WorldInfo":
                cs = _field(inside, "coordinateSystem")
                out["coordinate_system"] = cs.upper() if cs else None
                out["physics_plugin"] = _field(inside, "physics") or None
                bts = re.search(r"(?<![A-Za-z0-9_])basicTimeStep\s+"
                                r"([0-9]*\.?[0-9]+)", inside)
                if bts:
                    out["basic_time_step_ms"] = float(bts.group(1))
        i = max(after, m.end())
    return out


def _field(block_text, key):
    """One SFString field's value inside a block body, or ``""``."""
    hit = re.search(_SF_STRING % key, block_text)
    return hit.group(1) if hit else ""


def proto_instances(scanned, accepted):
    """Depth-0 instantiations whose PROTO/node type is in ``accepted``.

    Case-insensitive on the type name, because a PROTO's file name and its node
    name are the same token and the accepted set is written by hand.
    """
    want = {str(a).lower() for a in accepted}
    return [n for n in scanned.get("nodes", [])
            if n["type"].lower() in want]


def uses_of(scanned, accepted):
    """``USE <DEF>`` references pointing at a DEF that instantiated ``accepted``.

    Counted **separately** from instantiations and never folded into
    ``declared_count``: a ``USE`` is a second reference to one definition, and
    whether that is "another robot" is not a question this adapter should answer
    silently. It is published so a world written that way is visibly
    under-counted rather than invisibly so.
    """
    defs = {n["def"] for n in proto_instances(scanned, accepted) if n["def"]}
    return [u for u in scanned.get("uses", []) if u in defs]


def network_protos(scanned):
    """EXTERNPROTO imports that are fetched over the network on first load.

    ``webots-control-baseline.md`` §5.1: the R2025a distribution ships **zero**
    ``.proto`` files, so upstream worlds ``EXTERNPROTO`` from
    ``raw.githubusercontent.com`` into ``~/.cache/Cyberbotics``. That makes a
    world network-dependent on first load, against this suite's own
    local-asset-only rule -- so any run of such a world carries the caveat that
    a network hiccup could have become a benchmark result.
    """
    return [u for u in scanned.get("externprotos", [])
            if u.startswith("http://") or u.startswith("https://")]


__all__ = ["network_protos", "proto_instances", "scan", "uses_of"]
