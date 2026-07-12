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

"""Layout DSL.

A Layout is a declarative description of a world: named ``Zone`` s
(polygons), ``Path`` s (polylines with width), ``Stamp`` s (heightmap
modifiers anchored to a zone), ``PropGroup`` s (scatter recipes
constrained to a zone set), and ``Spawn`` s (robot placements). The
solver in ``core.solver`` turns a Layout into concrete terrain and
prop placements; biome recipes build Layouts instead of emitting
``.wbt`` text directly.

Design rules:

- Layouts are **declarative**. No code runs at construction time — all
  randomness happens during ``solve``.
- Layouts are **comparable and serialisable**. Two Layouts with the
  same fields compare equal. JSON round-trip is lossless
  (``schema.to_dict`` / ``from_dict``).
- Zone ``priority`` resolves conflicts: when two zones overlap, the
  higher-priority zone wins. A ``PropGroup`` declares
  ``include_zones`` and ``exclude_zones`` explicitly; the solver uses
  priority only when the recipe did not spell the relationship out.

This module is pure data. The solver lives next door so primitives
like `point_in_polygon` can be reused across both layers without
creating a dependency from the data layer back to the solver.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Sequence

from .recipe import Spawn

Point2 = tuple[float, float]


@dataclass(frozen=True)
class Zone:
    """A named 2D polygon describing where some kind of content belongs.

    - ``polygon`` is a list of ``(x, y)`` vertices in world metres.
      Closed implicitly — the last edge goes back to the first vertex.
    - ``priority`` breaks overlaps between zones in the solver. Higher
      wins. Ties are broken by the order zones were added to the
      Layout, so recipes get a predictable outcome even without
      spelling out priority on every zone.
    - ``exclusive`` marks "nothing else may be placed inside me" —
      useful for spawn clearings and named anchors.
    - ``tags`` are a free-form set consumers can filter on. Typical
      tags: ``"road"``, ``"grass"``, ``"rock_field"``, ``"plaza"``.
    """

    name: str
    polygon: tuple[Point2, ...]
    priority: int = 0
    exclusive: bool = False
    tags: frozenset[str] = frozenset()
    params: tuple[tuple[str, Any], ...] = ()   # frozen dict as tuple-of-pairs

    def params_dict(self) -> dict[str, Any]:
        return dict(self.params)


@dataclass(frozen=True)
class Path:
    """A named polyline with a width.

    Paths are conceptually 2D corridors — the scatter solver treats a
    path as "excluded unless you declared you want to be along it".
    Used by roads, warehouse aisles, sidewalks, service drives.
    """

    name: str
    polyline: tuple[Point2, ...]
    width: float
    priority: int = 0
    tags: frozenset[str] = frozenset()
    params: tuple[tuple[str, Any], ...] = ()

    def params_dict(self) -> dict[str, Any]:
        return dict(self.params)


@dataclass(frozen=True)
class Stamp:
    """A heightmap modifier anchored to a zone.

    - ``zone`` is the name of the Zone this stamp applies to.
    - ``mode`` is passed through to ``primitives.heightmap.stamp``:
      ``add``, ``sub``, ``max``, ``min``, ``replace``.
    - ``height`` is the peak magnitude.
    - ``falloff`` blends the edge of the stamp back into the base
      heightmap. Currently ``"smoothstep"`` or ``"linear"``; more
      shapes land as later tiers need them.
    """

    name: str
    zone: str
    mode: str = "max"
    height: float = 1.0
    falloff: str = "smoothstep"


@dataclass(frozen=True)
class PropGroup:
    """A scatter operation constrained to a subset of zones.

    The solver picks a proto per placement by weighted-sampling
    ``proto_weights``. The asset catalog (T1.5) will eventually give
    the solver footprint + collision tier + rotation preferences for
    each proto; Tier 1 emits a raw proto name and expects the recipe
    to own the instance parameters.

    - ``scatter`` picks the scatter primitive: ``poisson``,
      ``grid_jittered``, ``clustered``, ``along_path``.
    - ``include_zones`` — names of zones that define the allowed region.
      At least one must contain the candidate.
    - ``exclude_zones`` — zone names the candidate must NOT be inside.
    - ``exclude_paths`` — path names the candidate must be at least
      ``exclude_margin`` metres from.
    - ``params`` — scatter-primitive-specific parameters (e.g.
      ``count``, ``min_dist``, ``sigma``, ``density``).
    """

    name: str
    proto_weights: tuple[tuple[str, float], ...]
    scatter: str
    include_zones: tuple[str, ...] = ()
    exclude_zones: tuple[str, ...] = ()
    exclude_paths: tuple[str, ...] = ()
    exclude_margin: float = 0.0
    along_path_ref: str | None = None        # for scatter="along_path"
    clustered_centres_zone: str | None = None  # for scatter="clustered"
    params: tuple[tuple[str, Any], ...] = ()

    def params_dict(self) -> dict[str, Any]:
        return dict(self.params)

    def weights_dict(self) -> dict[str, float]:
        return dict(self.proto_weights)


@dataclass
class Layout:
    """Top-level container. The declarative description of one world.

    Mutable at construction time so recipes can build it progressively
    with helper methods. Once frozen (by a recipe handing it to the
    solver), the solver treats it as read-only.
    """

    world_size: tuple[float, float]
    zones: list[Zone] = field(default_factory=list)
    paths: list[Path] = field(default_factory=list)
    stamps: list[Stamp] = field(default_factory=list)
    prop_groups: list[PropGroup] = field(default_factory=list)
    spawns: list[Spawn] = field(default_factory=list)
    # Global heightmap parameters the recipe wants solver to honour.
    # (e.g. ``grid_resolution``, ``height_scale``.) Kept loose on
    # purpose — a recipe may add its own keys without a schema bump.
    terrain_params: dict[str, Any] = field(default_factory=dict)

    # --- mutation helpers ---------------------------------------------

    def add_zone(self, zone: Zone) -> None:
        if any(z.name == zone.name for z in self.zones):
            raise ValueError(f"zone name already exists: {zone.name!r}")
        self.zones.append(zone)

    def add_path(self, path: Path) -> None:
        if any(p.name == path.name for p in self.paths):
            raise ValueError(f"path name already exists: {path.name!r}")
        self.paths.append(path)

    def add_stamp(self, stamp: Stamp) -> None:
        if not any(z.name == stamp.zone for z in self.zones):
            raise ValueError(
                f"stamp {stamp.name!r} references unknown zone {stamp.zone!r}"
            )
        self.stamps.append(stamp)

    def add_prop_group(self, group: PropGroup) -> None:
        if any(g.name == group.name for g in self.prop_groups):
            raise ValueError(f"prop group name already exists: {group.name!r}")
        known_zones = {z.name for z in self.zones}
        known_paths = {p.name for p in self.paths}
        for zn in group.include_zones + group.exclude_zones:
            if zn not in known_zones:
                raise ValueError(
                    f"prop group {group.name!r} references unknown zone {zn!r}"
                )
        for pn in group.exclude_paths:
            if pn not in known_paths:
                raise ValueError(
                    f"prop group {group.name!r} references unknown path {pn!r}"
                )
        if group.along_path_ref is not None and group.along_path_ref not in known_paths:
            raise ValueError(
                f"prop group {group.name!r} references unknown along-path {group.along_path_ref!r}"
            )
        if (
            group.clustered_centres_zone is not None
            and group.clustered_centres_zone not in known_zones
        ):
            raise ValueError(
                f"prop group {group.name!r} references unknown centres zone "
                f"{group.clustered_centres_zone!r}"
            )
        self.prop_groups.append(group)

    def add_spawn(self, spawn: Spawn) -> None:
        if any(s.name == spawn.name for s in self.spawns):
            raise ValueError(f"spawn name already exists: {spawn.name!r}")
        self.spawns.append(spawn)

    # --- lookups -------------------------------------------------------

    def zone(self, name: str) -> Zone:
        for z in self.zones:
            if z.name == name:
                return z
        raise KeyError(f"no such zone: {name!r}")

    def path(self, name: str) -> Path:
        for p in self.paths:
            if p.name == name:
                return p
        raise KeyError(f"no such path: {name!r}")


# ---------------------------------------------------------------------------
# Geometry helpers shared by the solver
# ---------------------------------------------------------------------------


def point_in_polygon(p: Point2, polygon: Sequence[Point2]) -> bool:
    """Even-odd rule. Duplicated here (vs primitives.scatter) so the
    data layer does not pull in the scatter module."""
    x, y = p
    inside = False
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        if (ay > y) != (by > y):
            t = (y - ay) / (by - ay)
            if x < ax + t * (bx - ax):
                inside = not inside
    return inside


def distance_point_segment(
    p: Point2, a: Point2, b: Point2
) -> float:
    """Minimum distance from a point to a line segment."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    seg_len2 = dx * dx + dy * dy
    if seg_len2 == 0.0:
        rx, ry = px - ax, py - ay
        return (rx * rx + ry * ry) ** 0.5
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len2
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    cx = ax + t * dx
    cy = ay + t * dy
    rx = px - cx
    ry = py - cy
    return (rx * rx + ry * ry) ** 0.5


def distance_point_polyline(p: Point2, polyline: Sequence[Point2]) -> float:
    """Minimum distance to any segment of the polyline. Empty / single-
    point inputs return infinity."""
    if len(polyline) < 2:
        return float("inf")
    best = float("inf")
    for i in range(len(polyline) - 1):
        d = distance_point_segment(p, polyline[i], polyline[i + 1])
        if d < best:
            best = d
    return best


__all__ = [
    "Layout",
    "Path",
    "PropGroup",
    "Stamp",
    "Zone",
    "distance_point_polyline",
    "distance_point_segment",
    "point_in_polygon",
]
