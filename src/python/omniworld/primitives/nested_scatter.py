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

"""Nested scatter: place children on a parent's surface.

Closes one of the plan's T1.11 perceptual lies: empty desks and
empty shelves. Every catalog entry can declare named ``Surface`` s
(half-extents and z_offset in the PROTO's local frame); the
primitive here takes a parent placement plus a surface reference and
produces ``count`` child placements in world coordinates.

Child placements honour the parent's yaw: rotating the parent rotates
the surface rectangle with it, so a pallet stack tipped 5° still
gets its clutter on the tipped surface, not on the idealized
untipped surface.

Children are emitted as ``PlacedProto`` records the recipe can feed
directly into ``WorldDescription.props``.
"""

from __future__ import annotations

import math
import random
from typing import Iterable

from ..catalog import Asset, Surface, get as catalog_get
from ..core.recipe import PlacedProto
from .scatter import poisson_polygon, pick_weighted


def _rotate(p: tuple[float, float], yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c)


def _parent_yaw(rotation: tuple[float, float, float, float]) -> float:
    """Read the z-axis rotation angle from a ``(ax, ay, az, angle)``
    Webots-style rotation. Surfaces are assumed to be axis-aligned,
    so only the z component of the axis matters for top surfaces."""
    ax, ay, az, angle = rotation
    if az >= 0.9:
        return angle
    if az <= -0.9:
        return -angle
    return 0.0


def scatter_on_surface(
    rng: random.Random,
    parent: PlacedProto,
    surface_name: str,
    *,
    count: int,
    min_dist: float,
    children: Iterable[tuple[str, float]],
    parent_asset: Asset | None = None,
    z_jitter: float = 0.0,
) -> list[PlacedProto]:
    """Place ``count`` children on the named surface of ``parent``.

    - ``children`` is an iterable of ``(catalog_name, weight)`` pairs,
      used for weighted asset selection.
    - ``parent_asset`` can be supplied directly; otherwise the catalog
      is queried by ``parent.proto_type``.
    - ``z_jitter`` is an optional uniform jitter added to child z so
      stacks of identical objects don't look robotically flat.

    Returns fewer than ``count`` if the surface cannot fit them at the
    given ``min_dist``.
    """
    if count <= 0:
        return []
    asset = parent_asset or catalog_get(parent.proto_type)
    surf: Surface = asset.surface(surface_name)

    hx, hy = surf.half_extents
    rect_poly = ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))

    # Generate candidates in local-frame coords.
    local_points = poisson_polygon(
        rng, count, polygon=rect_poly, min_dist=min_dist
    )

    yaw = _parent_yaw(parent.rotation)
    parent_x, parent_y, parent_z = parent.translation

    child_names = [n for n, _ in children]
    child_weights = [w for _, w in children]
    if not child_names:
        raise ValueError("scatter_on_surface: children must be non-empty")

    out: list[PlacedProto] = []
    for local in local_points:
        rotated = _rotate(local, yaw)
        world_x = parent_x + rotated[0]
        world_y = parent_y + rotated[1]
        z_jit = (rng.random() * 2.0 - 1.0) * z_jitter if z_jitter > 0.0 else 0.0
        world_z = parent_z + surf.z_offset + z_jit

        child_name = child_names[pick_weighted(rng, child_weights)]
        child_asset = catalog_get(child_name)
        child_yaw = rng.random() * 2.0 * math.pi
        out.append(PlacedProto(
            proto_url=child_asset.url,
            proto_type=child_name,
            translation=(world_x, world_y, world_z),
            rotation=(0.0, 0.0, 1.0, child_yaw),
        ))
    return out


__all__ = ["scatter_on_surface"]
