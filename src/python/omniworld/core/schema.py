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

"""Layout serialisation.

A Layout is diffable, inspectable, and agent-editable. The JSON form
is the authoritative serialisation: recipes can be dumped, swapped,
and reloaded without touching Python code. YAML is out of scope at
Tier 1 — JSON subsumes it for our needs and adds no dependency.

Deliberately lossless: every field on every dataclass round-trips.
Frozen sets become sorted lists; tuples become lists; ``frozenset``
and ``tuple`` are reconstructed on parse. Unknown keys raise — catching
typos is cheaper than debugging silent drift.
"""

from __future__ import annotations

import json
from typing import Any

from .layout import Layout, Path, PropGroup, Stamp, Zone
from .recipe import Spawn


SCHEMA_VERSION = 1


# ---------------------------------------------------------------------------
# Encoding
# ---------------------------------------------------------------------------


def _zone_to_dict(z: Zone) -> dict[str, Any]:
    return {
        "name": z.name,
        "polygon": [[float(p[0]), float(p[1])] for p in z.polygon],
        "priority": z.priority,
        "exclusive": z.exclusive,
        "tags": sorted(z.tags),
        "params": [list(kv) for kv in z.params],
    }


def _path_to_dict(p: Path) -> dict[str, Any]:
    return {
        "name": p.name,
        "polyline": [[float(pt[0]), float(pt[1])] for pt in p.polyline],
        "width": float(p.width),
        "priority": p.priority,
        "tags": sorted(p.tags),
        "params": [list(kv) for kv in p.params],
    }


def _stamp_to_dict(s: Stamp) -> dict[str, Any]:
    return {
        "name": s.name,
        "zone": s.zone,
        "mode": s.mode,
        "height": float(s.height),
        "falloff": s.falloff,
    }


def _prop_group_to_dict(g: PropGroup) -> dict[str, Any]:
    return {
        "name": g.name,
        "proto_weights": [[n, float(w)] for n, w in g.proto_weights],
        "scatter": g.scatter,
        "include_zones": list(g.include_zones),
        "exclude_zones": list(g.exclude_zones),
        "exclude_paths": list(g.exclude_paths),
        "exclude_margin": float(g.exclude_margin),
        "along_path_ref": g.along_path_ref,
        "clustered_centres_zone": g.clustered_centres_zone,
        "params": [list(kv) for kv in g.params],
    }


def _spawn_to_dict(s: Spawn) -> dict[str, Any]:
    return {
        "name": s.name,
        "translation": [float(c) for c in s.translation],
        "rotation": [float(c) for c in s.rotation],
        "urdf_url": s.urdf_url,
        "controller": s.controller,
    }


def to_dict(layout: Layout) -> dict[str, Any]:
    """Serialise a Layout to a plain-JSON-compatible dict."""
    return {
        "schema_version": SCHEMA_VERSION,
        "world_size": list(layout.world_size),
        "terrain_params": dict(layout.terrain_params),
        "zones": [_zone_to_dict(z) for z in layout.zones],
        "paths": [_path_to_dict(p) for p in layout.paths],
        "stamps": [_stamp_to_dict(s) for s in layout.stamps],
        "prop_groups": [_prop_group_to_dict(g) for g in layout.prop_groups],
        "spawns": [_spawn_to_dict(s) for s in layout.spawns],
    }


def to_json(layout: Layout) -> str:
    return json.dumps(to_dict(layout), indent=2, sort_keys=True) + "\n"


# ---------------------------------------------------------------------------
# Decoding
# ---------------------------------------------------------------------------


def _check_keys(payload: dict[str, Any], allowed: set[str], where: str) -> None:
    extra = set(payload) - allowed
    if extra:
        raise ValueError(f"{where}: unknown keys {sorted(extra)!r}")


_ZONE_KEYS = {"name", "polygon", "priority", "exclusive", "tags", "params"}
_PATH_KEYS = {"name", "polyline", "width", "priority", "tags", "params"}
_STAMP_KEYS = {"name", "zone", "mode", "height", "falloff"}
_PROPGROUP_KEYS = {
    "name", "proto_weights", "scatter", "include_zones", "exclude_zones",
    "exclude_paths", "exclude_margin", "along_path_ref", "clustered_centres_zone",
    "params",
}
_SPAWN_KEYS = {"name", "translation", "rotation", "urdf_url", "controller"}
_LAYOUT_KEYS = {
    "schema_version", "world_size", "terrain_params",
    "zones", "paths", "stamps", "prop_groups", "spawns",
}


def _zone_from_dict(d: dict[str, Any]) -> Zone:
    _check_keys(d, _ZONE_KEYS, f"zone {d.get('name')!r}")
    return Zone(
        name=d["name"],
        polygon=tuple((float(x), float(y)) for x, y in d["polygon"]),
        priority=int(d.get("priority", 0)),
        exclusive=bool(d.get("exclusive", False)),
        tags=frozenset(d.get("tags", ())),
        params=tuple((k, v) for k, v in d.get("params", ())),
    )


def _path_from_dict(d: dict[str, Any]) -> Path:
    _check_keys(d, _PATH_KEYS, f"path {d.get('name')!r}")
    return Path(
        name=d["name"],
        polyline=tuple((float(x), float(y)) for x, y in d["polyline"]),
        width=float(d["width"]),
        priority=int(d.get("priority", 0)),
        tags=frozenset(d.get("tags", ())),
        params=tuple((k, v) for k, v in d.get("params", ())),
    )


def _stamp_from_dict(d: dict[str, Any]) -> Stamp:
    _check_keys(d, _STAMP_KEYS, f"stamp {d.get('name')!r}")
    return Stamp(
        name=d["name"],
        zone=d["zone"],
        mode=d.get("mode", "max"),
        height=float(d.get("height", 1.0)),
        falloff=d.get("falloff", "smoothstep"),
    )


def _prop_group_from_dict(d: dict[str, Any]) -> PropGroup:
    _check_keys(d, _PROPGROUP_KEYS, f"prop_group {d.get('name')!r}")
    return PropGroup(
        name=d["name"],
        proto_weights=tuple((n, float(w)) for n, w in d["proto_weights"]),
        scatter=d["scatter"],
        include_zones=tuple(d.get("include_zones", ())),
        exclude_zones=tuple(d.get("exclude_zones", ())),
        exclude_paths=tuple(d.get("exclude_paths", ())),
        exclude_margin=float(d.get("exclude_margin", 0.0)),
        along_path_ref=d.get("along_path_ref"),
        clustered_centres_zone=d.get("clustered_centres_zone"),
        params=tuple((k, v) for k, v in d.get("params", ())),
    )


def _spawn_from_dict(d: dict[str, Any]) -> Spawn:
    _check_keys(d, _SPAWN_KEYS, f"spawn {d.get('name')!r}")
    return Spawn(
        name=d["name"],
        translation=tuple(d["translation"]),
        rotation=tuple(d.get("rotation", (0.0, 0.0, 1.0, 0.0))),
        urdf_url=d.get("urdf_url"),
        controller=d.get("controller"),
    )


def from_dict(payload: dict[str, Any]) -> Layout:
    """Parse a Layout from a plain-JSON-compatible dict."""
    _check_keys(payload, _LAYOUT_KEYS, "layout")
    schema = int(payload.get("schema_version", SCHEMA_VERSION))
    if schema != SCHEMA_VERSION:
        raise ValueError(
            f"unsupported schema_version {schema}; this build expects {SCHEMA_VERSION}"
        )
    layout = Layout(
        world_size=tuple(payload["world_size"]),
        terrain_params=dict(payload.get("terrain_params", {})),
    )
    for zd in payload.get("zones", []):
        layout.zones.append(_zone_from_dict(zd))
    for pd in payload.get("paths", []):
        layout.paths.append(_path_from_dict(pd))
    for sd in payload.get("stamps", []):
        layout.stamps.append(_stamp_from_dict(sd))
    for gd in payload.get("prop_groups", []):
        layout.prop_groups.append(_prop_group_from_dict(gd))
    for sp in payload.get("spawns", []):
        layout.spawns.append(_spawn_from_dict(sp))
    return layout


def from_json(text: str) -> Layout:
    return from_dict(json.loads(text))


__all__ = ["SCHEMA_VERSION", "to_dict", "to_json", "from_dict", "from_json"]
