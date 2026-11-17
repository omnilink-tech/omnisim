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

"""Mapping between OmniSim scene nodes and ``simulation_interfaces`` entities.

Entity identity
---------------
A ``simulation_interfaces`` entity is addressed by a unique string name. OmniSim
addresses scene nodes by **DEF**, which is the key every harness write endpoint
(``/scene/set_pose``, ``/scene/delete``, ``/scene/node/<def>``) already takes. So
the entity name *is* the DEF — no side table, no invented namespace, and an
entity name a ROS caller obtained from :class:`GetEntities` can be pasted
straight into a harness call.

Nodes with no DEF are reported by the harness as ``"#<id>"``. Those are exposed
too (a scene is allowed to contain anonymous nodes) but they cannot be moved or
deleted, because the harness resolves writes by DEF only.

Which nodes are entities
------------------------
``/scene/tree`` is a *flat, pre-order* list of every node in the scene, joint
endpoints included — several hundred entries for one URDF robot. Publishing all
of them as entities would be useless, and publishing the wrong ones is worse.
Two rules select the set:

1. **Root-level only** (``parent_def is None``). That is the same set OmniSim's
   own ``/world/sync`` treats as live-editable, and it yields one entity per
   robot/prop/floor rather than one per link.
2. **Posed only.** A ``simulation_interfaces`` entity is defined by having an
   ``EntityState`` — a pose. OmniSim reports ``position: [null, null, null]``
   for nodes that have no pose at all, which is exactly the set a ROS caller
   does not want: ``WorldInfo``, ``Viewpoint``, ``Group``, the sky and the
   directional-light nodes. Filtering on "has a pose" removes them on a
   principle rather than a hand-maintained blocklist of type names.

The harness-injected supervisor Robot is excluded too. ⚠ ``/scene/tree``'s own
``harness_injected`` list is **empty even when the supervisor is present**
(measured 2026-08-17 on ``omnilink_husky.omniworld``: the tree reported ``[]``
while ``/robots?include_harness=1`` correctly reported
``['harness_supervisor']`` and tagged node ``#185``). So the tree's own tagging
cannot be relied on, and the caller passes the DEFs to exclude, sourced from
``/robots?include_harness=1``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any, Iterable

# simulation_interfaces/msg/EntityCategory constants, mirrored so this module
# stays importable without a ROS environment (the tests rely on that).
CATEGORY_OBJECT = 0
CATEGORY_ROBOT = 1
CATEGORY_HUMAN = 2
CATEGORY_DYNAMIC_OBJECT = 4
CATEGORY_STATIC_OBJECT = 5

# Node type names that are robots in OmniSim's schema.
_ROBOT_TYPES = {"Robot", "URDFRobot", "Supervisor"}


@dataclass
class Entity:
    """One OmniSim scene node presented as a simulation_interfaces entity."""

    name: str
    node_type: str
    position: list[float]
    orientation: list[float]
    node_id: int | None = None
    addressable: bool = True
    """False for DEF-less nodes, which the harness cannot resolve for writes."""

    @property
    def tags(self) -> list[str]:
        """Tags exposed through ``GetEntityInfo``.

        Only facts read from the scene are published — the VRML type name, and
        ``robot`` for robot types. Nothing is inferred beyond that.
        """
        out = [f"type:{self.node_type}"]
        if self.node_type in _ROBOT_TYPES:
            out.append("robot")
        return out


def is_harness_injected(node: dict[str, Any], injected_names: Iterable[str]) -> bool:
    """True when this node is the harness's own supervisor scaffolding."""
    if node.get("harness_injected"):
        return True
    names = set(injected_names or ())
    return bool(names) and node.get("def") in names


def has_pose(node: dict[str, Any]) -> bool:
    """True when the harness reported a real position for this node.

    Pose-less nodes come back as ``[null, null, null]``; the harness uses that
    for every node that is not spatial (``WorldInfo``, ``Viewpoint``, ``Group``,
    backgrounds and lights).
    """
    pos = node.get("position")
    if not isinstance(pos, (list, tuple)) or len(pos) != 3:
        return False
    return all(v is not None for v in pos)


def entities_from_scene_tree(
    tree: dict[str, Any],
    *,
    root_only: bool = True,
    exclude_defs: Iterable[str] = (),
) -> list[Entity]:
    """Select entities from a ``GET /scene/tree`` body.

    ``exclude_defs`` should carry the harness-injected robot DEFs, which the
    tree's own ``harness_injected`` list does not reliably report.
    """
    injected = tree.get("harness_injected") or []
    excluded = {str(d) for d in exclude_defs}
    out: list[Entity] = []
    seen: set[str] = set()
    for node in tree.get("nodes") or []:
        if not isinstance(node, dict):
            continue
        if is_harness_injected(node, injected):
            continue
        if root_only and node.get("parent_def") is not None:
            continue
        if not has_pose(node):
            continue
        def_name = node.get("def")
        if def_name in excluded:
            continue
        if def_name is None and node.get("id") is not None:
            if f"#{node.get('id')}" in excluded:
                continue
        node_id = node.get("id")
        addressable = bool(def_name)
        if not def_name:
            if node_id is None:
                continue
            def_name = f"#{node_id}"
        if def_name in seen:
            # DEFs are unique in a well-formed world; if one repeats, the first
            # occurrence is the one every harness write will resolve to.
            continue
        seen.add(def_name)
        out.append(
            Entity(
                name=str(def_name),
                node_type=str(node.get("type") or ""),
                position=list(node.get("position") or []),
                orientation=list(node.get("orientation") or []),
                node_id=node_id if isinstance(node_id, int) else None,
                addressable=addressable,
            )
        )
    return out


def category_for(node_type: str, node_detail: dict[str, Any] | None) -> int:
    """Classify a node into a ``simulation_interfaces`` EntityCategory.

    ``node_detail`` is a ``GET /scene/node/<def>`` body. The distinction between
    a dynamic and a static object is whether the node carries a ``Physics`` node,
    which is exactly what OmniSim uses to decide whether a body is simulated, so
    this is a read rather than a guess. Without a detail body we can only answer
    ROBOT (from the type) or the neutral OBJECT.
    """
    if node_type in _ROBOT_TYPES:
        return CATEGORY_ROBOT
    if not node_detail:
        return CATEGORY_OBJECT
    fields = node_detail.get("fields") or {}
    physics = fields.get("physics") or {}
    if physics.get("present"):
        return CATEGORY_DYNAMIC_OBJECT
    if physics.get("field_exists"):
        return CATEGORY_STATIC_OBJECT
    return CATEGORY_OBJECT


def matches_filter(name: str, pattern: str) -> bool:
    """Apply a ``EntityFilters.filter`` regular expression.

    The standard specifies POSIX Extended regular expressions. Python's ``re`` is
    a superset for the constructs POSIX ERE supports, so ordinary filters behave
    identically; an invalid pattern matches nothing rather than raising, so one
    bad filter cannot take down the service.
    """
    if not pattern:
        return True
    try:
        return re.search(pattern, name) is not None
    except re.error:
        return False


def compose_spawn_body(
    name: str,
    resource_uri: str,
    resource_string: str,
    position: tuple[float, float, float],
    rotation_axis_angle: list[float],
) -> dict[str, Any]:
    """Build a ``POST /scene/spawn`` body from a ``SpawnEntity`` request.

    OmniSim's spawn endpoint accepts a raw VRML node string (``vrml``) or a URDF
    path (``urdf``). ``resource_string`` maps to ``vrml``; a ``uri`` ending in
    ``.urdf`` maps to ``urdf``, and any other URI is passed as ``urdf`` only when
    it looks like one — otherwise the caller gets an explicit unsupported-format
    error rather than a confusing parse failure deep in the engine.
    """
    body: dict[str, Any] = {
        "translation": [float(v) for v in position],
        "rotation": [float(v) for v in rotation_axis_angle],
    }
    if name:
        body["def"] = name
        body["name"] = name
    if resource_string:
        body["vrml"] = resource_string
    elif resource_uri:
        body["urdf"] = _strip_file_scheme(resource_uri)
    return body


def _strip_file_scheme(uri: str) -> str:
    """Reduce a ``file://`` URI to a plain path the harness can open."""
    if uri.startswith("file://"):
        return uri[len("file://") :]
    return uri


def spawn_format_for(uri: str, resource_string: str) -> str:
    """Name the spawn format implied by a ``Resource``, or '' when unsupported."""
    if resource_string:
        return "vrml"
    lowered = (uri or "").lower().split("?")[0]
    if lowered.endswith(".urdf") or lowered.endswith(".xacro"):
        return "urdf"
    if lowered.endswith(".wbo") or lowered.endswith(".vrml") or lowered.endswith(".proto"):
        return "vrml"
    return ""
