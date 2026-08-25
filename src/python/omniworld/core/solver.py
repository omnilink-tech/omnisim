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

"""Layout solver.

Given a ``Layout`` and a seed, produce a concrete ``SolvedWorld``:

- ``heightmap`` — optional, produced by applying all stamps on top of
  the base terrain declared via ``layout.terrain_params``.
- ``props`` — a list of ``PropPlacement`` records (proto name,
  position, rotation). Each ``PropGroup`` contributes some.
- ``spawns`` — passed through verbatim.

This is the T1.4 solver — naive but honest. It honors zone priority
(higher wins; ``exclusive`` zones clear everything at lower priority
inside them), explicit include/exclude relationships on prop groups,
and a perpendicular-distance check against paths. No polygon CSG yet.

Deterministic given ``(layout, seed)``. Sub-seeds for heightmap,
scatter, and per-placement rotation are derived by name from the
parent seed using ``core.rng.derive_seed``, so two solves with the
same inputs produce the same output byte-for-byte, and adding a new
named sub-system does not perturb existing ones.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Any, Sequence

from ..primitives.heightmap import (
    Heightmap,
    apply_mask,
    blend,
    fbm,
    mask_polygon,
    ridged,
    stamp as heightmap_stamp,
    worley,
)
from ..primitives.scatter import (
    along_path,
    clustered,
    grid_jittered,
    on_surface,
    pick_weighted,
    poisson_polygon,
)
from .layout import (
    Layout,
    Path,
    PropGroup,
    Zone,
    distance_point_polyline,
    point_in_polygon,
)
from .recipe import Spawn
from .rng import derive_seed, rng_from_seed

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]


# ---------------------------------------------------------------------------
# Solved output
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PropPlacement:
    """One concrete prop placement the solver decided on."""

    proto: str
    group: str                  # name of the PropGroup that produced it
    translation: Point3
    rotation: tuple[float, float, float, float]  # axis + angle

    def to_dict(self) -> dict[str, Any]:
        return {
            "proto": self.proto,
            "group": self.group,
            "translation": list(self.translation),
            "rotation": list(self.rotation),
        }


@dataclass
class SolvedWorld:
    """Result of solving a Layout at a specific seed."""

    heightmap: Heightmap | None
    terrain_extent: tuple[float, float]        # world-space width, depth (metres)
    props: list[PropPlacement] = field(default_factory=list)
    spawns: list[Spawn] = field(default_factory=list)

    def prop_count(self) -> int:
        return len(self.props)


# ---------------------------------------------------------------------------
# Allowed-region testing
# ---------------------------------------------------------------------------


def _effective_zones_for(
    group: PropGroup, layout: Layout
) -> tuple[list[Zone], list[Zone]]:
    """Resolve include/exclude zone names to Zone objects.

    When ``include_zones`` is empty the group is unconstrained in x/y
    (the whole world is fair game). When an exclusive higher-priority
    zone overlaps an include zone it contributes to the exclude set.
    """
    zones_by_name = {z.name: z for z in layout.zones}
    include = [zones_by_name[n] for n in group.include_zones]
    explicit_excludes = [zones_by_name[n] for n in group.exclude_zones]

    # Priority resolution: any exclusive zone whose priority is higher
    # than every include zone's priority acts as an implicit excluder.
    include_priority = max((z.priority for z in include), default=-(1 << 30))
    implicit_excludes = [
        z
        for z in layout.zones
        if z.exclusive
        and z.priority > include_priority
        and z not in include
        and z not in explicit_excludes
    ]
    return include, explicit_excludes + implicit_excludes


def _is_point_allowed(
    p: Point2,
    include: Sequence[Zone],
    exclude: Sequence[Zone],
    path_exclusions: Sequence[tuple[Path, float]],
) -> bool:
    """Decide whether a point may host a prop.

    - In at least one include zone (or unconstrained if include is empty).
    - Not in any exclude zone.
    - Not within its own half-width plus margin of any excluded path.
    """
    if include:
        if not any(point_in_polygon(p, z.polygon) for z in include):
            return False
    for z in exclude:
        if point_in_polygon(p, z.polygon):
            return False
    for path, margin in path_exclusions:
        if distance_point_polyline(p, path.polyline) < (path.width * 0.5 + margin):
            return False
    return True


# ---------------------------------------------------------------------------
# Terrain solve
# ---------------------------------------------------------------------------


_DEFAULT_GRID_RES = 128
_DEFAULT_BASE_HEIGHT = 1.0


def _build_terrain(layout: Layout, seed: int) -> Heightmap | None:
    """Produce the heightmap described by ``layout.terrain_params``.

    Supported ``kind`` values:
      - missing / ``"flat"`` — no heightmap; solver returns ``None``.
      - ``"fbm"`` — multi-octave value noise.

    Stamps anchored to zones are applied after the base is computed.
    """
    params = layout.terrain_params
    kind = params.get("kind")
    if kind is None or kind == "flat":
        return None

    grid_res = int(params.get("grid_resolution", _DEFAULT_GRID_RES))
    base_height = float(params.get("height_scale", _DEFAULT_BASE_HEIGHT))

    octaves = int(params.get("octaves", 5))
    gain = float(params.get("gain", 0.5))
    lacunarity = float(params.get("lacunarity", 2.0))
    base_period = params.get("base_period", None)

    if kind == "fbm":
        hm = fbm(
            grid_res, grid_res,
            seed=derive_seed(seed, "terrain.fbm"),
            octaves=octaves, lacunarity=lacunarity, gain=gain,
            base_period=base_period,
        )
    elif kind == "ridged":
        hm = ridged(
            grid_res, grid_res,
            seed=derive_seed(seed, "terrain.ridged"),
            octaves=octaves, lacunarity=lacunarity, gain=gain,
            base_period=base_period,
        )
    elif kind == "worley":
        points_per_cell = int(params.get("worley_points_per_cell", 1))
        cell_size = params.get("worley_cell_size", None)
        variant = str(params.get("worley_variant", "f1"))
        hm = worley(
            grid_res, grid_res,
            seed=derive_seed(seed, "terrain.worley"),
            points_per_cell=points_per_cell,
            cell_size=cell_size,
            variant=variant,
        )
    elif kind == "ridged_plus_fbm":
        # Mesa-and-roll: ridged peaks blended over a fbm base.
        mix = float(params.get("mix", 0.5))
        base = fbm(
            grid_res, grid_res,
            seed=derive_seed(seed, "terrain.mix.fbm"),
            octaves=octaves, lacunarity=lacunarity, gain=gain,
        )
        peaks = ridged(
            grid_res, grid_res,
            seed=derive_seed(seed, "terrain.mix.ridged"),
            octaves=max(1, octaves - 1),
            lacunarity=lacunarity, gain=gain,
        )
        mask = Heightmap.constant(grid_res, grid_res, mix)
        hm = blend(base, peaks, mask)
    else:
        raise ValueError(f"unsupported terrain kind: {kind!r}")

    # Scale the unit-range output by the base height.
    hm = Heightmap(hm.width, hm.height, [v * base_height for v in hm.data])

    # Apply stamps.
    world_w, world_h = layout.world_size
    for st in layout.stamps:
        zone = layout.zone(st.zone)
        # Convert zone polygon from world coords to grid coords.
        grid_poly = [
            ((x / world_w) * hm.width, (y / world_h) * hm.height)
            for x, y in zone.polygon
        ]
        m = mask_polygon(hm.width, hm.height, grid_poly)
        # Feather via the same mask, since mask_polygon has no falloff.
        if st.mode == "add":
            hm = apply_mask(hm, m, strength=st.height, mode="add")
        elif st.mode == "sub":
            # Mask drives a subtraction — invert sign in strength.
            hm = apply_mask(hm, m, strength=-st.height, mode="add")
        elif st.mode == "replace":
            hm = apply_mask(hm, m, strength=st.height, mode="replace")
        elif st.mode in {"max", "min"}:
            # "max"/"min" semantics against a constant: replace where
            # mask is 1.0 only if it would be a max/min. Cheap version.
            pat = Heightmap.constant(hm.width, hm.height, st.height)
            merged = heightmap_stamp(hm, pat, 0, 0, mode=st.mode)
            # Blend by mask so only inside the stamp region changes.
            hm = Heightmap(
                hm.width,
                hm.height,
                [
                    hm.data[idx] * (1 - m.data[idx]) + merged.data[idx] * m.data[idx]
                    for idx in range(len(hm.data))
                ],
            )
        else:
            raise ValueError(f"stamp {st.name!r}: unsupported mode {st.mode!r}")
    return hm


# ---------------------------------------------------------------------------
# Prop generation
# ---------------------------------------------------------------------------


def _generate_group_points(
    group: PropGroup,
    layout: Layout,
    seed: int,
) -> list[Point2]:
    """Run the group's scatter primitive and return 2D candidates.

    Further filtering (include / exclude / path exclusion) happens in
    the caller so all groups share the same gate.
    """
    rng = rng_from_seed(derive_seed(seed, f"propgroup.{group.name}"))
    params = group.params_dict()

    if group.scatter == "poisson":
        # Pick a polygon to scatter inside. We use the union of include
        # zones' bounding boxes and rely on the caller's gate to reject
        # points outside the zones.
        if group.include_zones:
            verts: list[Point2] = []
            for name in group.include_zones:
                verts.extend(layout.zone(name).polygon)
            xs = [v[0] for v in verts]
            ys = [v[1] for v in verts]
            bbox = [
                (min(xs), min(ys)),
                (max(xs), min(ys)),
                (max(xs), max(ys)),
                (min(xs), max(ys)),
            ]
        else:
            wx, wy = layout.world_size
            bbox = [(0.0, 0.0), (wx, 0.0), (wx, wy), (0.0, wy)]
        return poisson_polygon(
            rng,
            int(params.get("count", 0)),
            polygon=bbox,
            min_dist=float(params.get("min_dist", 0.0)),
        )
    if group.scatter == "grid_jittered":
        if not group.include_zones:
            raise ValueError(
                f"grid_jittered prop group {group.name!r} needs include_zones"
            )
        # Use the first include zone as the polygon.
        zone = layout.zone(group.include_zones[0])
        return grid_jittered(
            rng,
            list(zone.polygon),
            cell_size=float(params["cell_size"]),
            jitter=float(params.get("jitter", 0.25)),
        )
    if group.scatter == "clustered":
        if group.clustered_centres_zone is None:
            raise ValueError(
                f"clustered prop group {group.name!r} needs clustered_centres_zone"
            )
        centres_zone = layout.zone(group.clustered_centres_zone)
        # Use the zone's vertices as cluster centres.
        centres = list(centres_zone.polygon)
        return clustered(
            rng,
            centres,
            count_per=int(params.get("count_per", 0)),
            sigma=float(params.get("sigma", 1.0)),
            min_dist=float(params.get("min_dist", 0.0)),
        )
    if group.scatter == "along_path":
        if group.along_path_ref is None:
            raise ValueError(
                f"along_path prop group {group.name!r} needs along_path_ref"
            )
        path = layout.path(group.along_path_ref)
        return along_path(
            rng,
            list(path.polyline),
            density=float(params["density"]),
            offset=float(params.get("offset", 0.0)),
            offset_jitter=float(params.get("offset_jitter", 0.0)),
            along_jitter=float(params.get("along_jitter", 0.0)),
            side=str(params.get("side", "both")),
        )
    raise ValueError(
        f"prop group {group.name!r}: unknown scatter {group.scatter!r}"
    )


def _place_group(
    group: PropGroup,
    layout: Layout,
    hm: Heightmap | None,
    seed: int,
) -> list[PropPlacement]:
    candidates = _generate_group_points(group, layout, seed)

    include, exclude = _effective_zones_for(group, layout)
    path_exclusions = [
        (layout.path(n), group.exclude_margin) for n in group.exclude_paths
    ]

    rot_rng = rng_from_seed(derive_seed(seed, f"propgroup.{group.name}.rot"))
    pick_rng = rng_from_seed(derive_seed(seed, f"propgroup.{group.name}.pick"))

    # Catalog integration is T1.5, so the solver uses the declared
    # proto weights verbatim.
    names = [name for name, _ in group.proto_weights]
    weights = [w for _, w in group.proto_weights]

    # Height sampling requires converting world coords to grid coords.
    wx, wy = layout.world_size

    placements: list[PropPlacement] = []
    for p in candidates:
        if not _is_point_allowed(p, include, exclude, path_exclusions):
            continue
        if hm is not None:
            lifted = on_surface(
                [p],
                hm,
                grid_origin=(0.0, 0.0),
                x_spacing=wx / hm.width,
                y_spacing=wy / hm.height,
                height_scale=1.0,
            )
            z = lifted[0][2] if lifted else 0.0
        else:
            z = 0.0
        proto = names[pick_weighted(pick_rng, weights)]
        yaw = rot_rng.random() * 2.0 * math.pi
        placements.append(
            PropPlacement(
                proto=proto,
                group=group.name,
                translation=(float(p[0]), float(p[1]), float(z)),
                rotation=(0.0, 0.0, 1.0, yaw),
            )
        )
    return placements


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def solve(layout: Layout, *, seed: int) -> SolvedWorld:
    """Solve a Layout into concrete heightmap + prop placements.

    Given the same ``(layout, seed)`` the output is byte-identical.
    """
    hm = _build_terrain(layout, seed)

    props: list[PropPlacement] = []
    # Spawn-zone exclusivity: any zone with ``exclusive=True`` containing
    # the group's region implicitly excludes.
    for group in layout.prop_groups:
        props.extend(_place_group(group, layout, hm, seed))

    return SolvedWorld(
        heightmap=hm,
        terrain_extent=layout.world_size,
        props=props,
        spawns=list(layout.spawns),
    )


__all__ = ["PropPlacement", "SolvedWorld", "solve"]
