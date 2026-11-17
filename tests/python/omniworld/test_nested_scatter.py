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

"""T1.11 nested_scatter + surface_manifest tests."""

from __future__ import annotations

import math
import random

import pytest

from omniworld.catalog import Asset, Surface, get
from omniworld.core.recipe import PlacedProto
from omniworld.primitives.nested_scatter import scatter_on_surface


def _rock_parent(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return PlacedProto(
        proto_url="omnisim://dummy",
        proto_type="WoodenPalletStack",
        translation=(x, y, z),
        rotation=(0.0, 0.0, 1.0, yaw),
    )


def test_surface_loaded_from_catalog():
    asset = get("WoodenPalletStack")
    s = asset.surface("top")
    assert isinstance(s, Surface)
    assert s.half_extents == (0.55, 0.35)
    assert s.z_offset > 0.0


def test_unknown_surface_raises():
    asset = get("WoodenPalletStack")
    with pytest.raises(KeyError):
        asset.surface("bottom")


def test_scatter_on_surface_places_children_on_top():
    rng = random.Random(0)
    parent = _rock_parent(x=5.0, y=3.0, z=0.0)
    children = scatter_on_surface(
        rng, parent, "top",
        count=3, min_dist=0.2,
        children=[("CardboardBox", 1.0)],
    )
    assert len(children) > 0
    top_asset = get("WoodenPalletStack").surface("top")
    for c in children:
        cx, cy, cz = c.translation
        # Child must sit at parent z + surface z_offset (+ no jitter).
        assert cz == pytest.approx(top_asset.z_offset)
        # Child x/y must lie inside the surface rectangle (parent at 5, 3).
        assert abs(cx - 5.0) <= top_asset.half_extents[0] + 1e-6
        assert abs(cy - 3.0) <= top_asset.half_extents[1] + 1e-6


def test_scatter_on_surface_honours_parent_yaw():
    rng = random.Random(0)
    # Parent rotated 90°: the surface rectangle rotates with it, so a
    # child that was at local (hx, 0) lands at world (0, hx) (relative
    # to parent).
    yaw = math.pi / 2.0
    parent = _rock_parent(x=10.0, y=20.0, yaw=yaw)
    children = scatter_on_surface(
        rng, parent, "top",
        count=12, min_dist=0.1,
        children=[("CardboardBox", 1.0)],
    )
    # No child must exceed the swapped half-extent bound in either axis.
    top_asset = get("WoodenPalletStack").surface("top")
    hx, hy = top_asset.half_extents
    for c in children:
        cx, cy, _ = c.translation
        dx = cx - 10.0
        dy = cy - 20.0
        # After yaw=90° rotation, the local x rectangle maps to world y,
        # local y rectangle maps to world -x. The bound check tightens
        # to: |dy| <= hx, |dx| <= hy.
        assert abs(dy) <= hx + 1e-6
        assert abs(dx) <= hy + 1e-6


def test_scatter_on_surface_deterministic():
    rng1 = random.Random(5)
    rng2 = random.Random(5)
    parent = _rock_parent()
    a = scatter_on_surface(
        rng1, parent, "top",
        count=4, min_dist=0.15,
        children=[("CardboardBox", 1.0)],
    )
    b = scatter_on_surface(
        rng2, parent, "top",
        count=4, min_dist=0.15,
        children=[("CardboardBox", 1.0)],
    )
    assert a == b


def test_scatter_on_surface_requires_children():
    with pytest.raises(ValueError):
        scatter_on_surface(
            random.Random(0), _rock_parent(), "top",
            count=1, min_dist=0.0,
            children=[],
        )


def test_scatter_on_surface_zero_count_is_empty():
    result = scatter_on_surface(
        random.Random(0), _rock_parent(), "top",
        count=0, min_dist=0.0,
        children=[("CardboardBox", 1.0)],
    )
    assert result == []


# -------------------------------------------------------------------
# Warehouse integration: boxes actually land on top of some racks.
# -------------------------------------------------------------------


def test_warehouse_produces_top_clutter(tmp_path):
    from omniworld import generate

    out = tmp_path / "w.wbt"
    result = generate("warehouse", seed=42, out=out)
    # Cluttered boxes sit at z > 0 (stacked above the rack), while
    # floor boxes sit at z = 0 (floor level).
    boxes = [p for p in result.description.props
             if p.proto_type in ("CardboardBox", "PlasticCrate", "WoodenBox")]
    stacked = [b for b in boxes if b.translation[2] > 0.5]
    assert stacked, "expected at least one box on top of a rack"


def test_warehouse_top_clutter_disabled_by_zero_fraction(tmp_path):
    from omniworld import generate

    out = tmp_path / "w.wbt"
    result = generate(
        "warehouse", seed=42, out=out,
        params={"rack_top_clutter_fraction": 0.0},
    )
    boxes = [p for p in result.description.props
             if p.proto_type in ("CardboardBox", "PlasticCrate", "WoodenBox")]
    stacked = [b for b in boxes if b.translation[2] > 0.5]
    assert not stacked, "top clutter should be suppressed"
