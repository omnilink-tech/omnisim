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

"""Minimal 2.5D CAM: pocket a region with a flat end mill and emit GRBL G-code.

Clearing is a zig-zag raster of the region shrunk by the tool radius; the tool
stays down between rows only when the straight link stays inside the region,
otherwise it retracts. Each depth pass ends with a finishing contour along
every boundary (outer wall and islands). Work coordinates: X/Y as given,
Z0 = stock top, so depths are negative.

This is enough for demos and for an agent to turn "pocket a 100 mm circle,
3 mm deep" into a real job. It is not a production CAM: no climb/conventional
choice, no ramping or helical entry (it plunges straight at the plunge feed),
no rest machining, no tabs.
"""
from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

from shapely.geometry import LineString, MultiLineString, Point, Polygon
from shapely.geometry.base import BaseGeometry


def star(cx: float, cy: float, r_out: float, r_in: float, points: int = 5) -> Polygon:
    pts = []
    for k in range(points * 2):
        r = r_out if k % 2 == 0 else r_in
        a = math.pi / 2 + k * math.pi / points
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return Polygon(pts)


def circle(cx: float, cy: float, r: float) -> Polygon:
    return Point(cx, cy).buffer(r, quad_segs=64)


def _polys(geom: BaseGeometry) -> List[Polygon]:
    if geom.is_empty:
        return []
    return [geom] if geom.geom_type == "Polygon" else [g for g in geom.geoms if g.geom_type == "Polygon"]


def _segments(geom: BaseGeometry) -> List[List[Tuple[float, float]]]:
    if geom.is_empty:
        return []
    if isinstance(geom, LineString):
        return [list(geom.coords)]
    if isinstance(geom, MultiLineString) or hasattr(geom, "geoms"):
        return [list(g.coords) for g in geom.geoms if isinstance(g, LineString)]
    return []


class GcodeWriter:
    def __init__(self, safe_z: float, feed: float, plunge: float):
        self.lines: List[str] = []
        self.safe_z, self.feed, self.plunge = safe_z, feed, plunge
        self.down = False
        self.at = None

    def emit(self, line: str) -> None:
        self.lines.append(line)

    def retract(self) -> None:
        if self.down:
            self.emit(f"G0 Z{self.safe_z:.3f}")
            self.down = False

    def travel(self, x: float, y: float) -> None:
        self.retract()
        self.emit(f"G0 X{x:.3f} Y{y:.3f}")
        self.at = (x, y)

    def plunge_to(self, z: float) -> None:
        self.emit(f"G1 Z{z:.3f} F{self.plunge:.0f}")
        self.down = True

    def cut(self, x: float, y: float) -> None:
        self.emit(f"G1 X{x:.3f} Y{y:.3f} F{self.feed:.0f}")
        self.at = (x, y)


def pocket(region: Polygon, tool_d: float, depth: float, stepover: float, step_down: float,
           feed: float = 2500.0, plunge: float = 600.0, rpm: float = 18000.0, safe_z: float = 5.0,
           title: str = "pocket") -> str:
    """G-code that removes `region` (work XY, mm) to `depth` (mm below Z0)."""
    if depth <= 0 or tool_d <= 0 or not 0 < stepover <= tool_d or step_down <= 0:
        raise ValueError("depth, tool_d and step_down must be positive; 0 < stepover <= tool_d")
    if region.buffer(-tool_d / 2.0, quad_segs=32).is_empty:
        raise ValueError("the tool does not fit in the region")
    w = GcodeWriter(safe_z, feed, plunge)
    w.emit(f"({title}: {tool_d:g} mm flat end mill, {depth:g} mm deep, stepover {stepover:g}, "
           f"step-down {step_down:g})")
    w.emit("G21 G90 G54")
    w.emit(f"M3 S{rpm:.0f}")
    w.emit(f"G0 Z{safe_z:.3f}")
    clear(w, region, tool_d, depth, stepover, step_down)
    w.emit("M5")
    w.emit(f"G0 X0 Y0 Z{safe_z * 4:.3f}")
    w.emit("M30")
    return "\n".join(w.lines) + "\n"


def clear(w: GcodeWriter, region: Polygon, tool_d: float, depth: float, stepover: float, step_down: float) -> None:
    """Append the moves that remove `region` to `depth` with a flat end mill to `w`: per
    depth pass a zig-zag raster of the region shrunk by the tool radius, then a finishing
    contour along every boundary. Leaves the tool retracted."""
    if depth <= 0 or tool_d <= 0 or not 0 < stepover <= tool_d or step_down <= 0:
        raise ValueError("depth, tool_d and step_down must be positive; 0 < stepover <= tool_d")
    area = region.buffer(-tool_d / 2.0, quad_segs=32)
    if area.is_empty:
        raise ValueError("the tool does not fit in the region")
    keep_inside = area.buffer(1e-6)
    passes = max(1, math.ceil(depth / step_down - 1e-9))
    minx, miny, maxx, maxy = area.bounds
    for p in range(1, passes + 1):
        z = -min(depth, p * step_down)
        rows = []
        y = miny + 1e-3
        flip = False
        while y <= maxy - 1e-3:
            line = LineString([(minx - 1, y), (maxx + 1, y)])
            segs = sorted(_segments(area.intersection(line)), key=lambda s: min(s[0][0], s[-1][0]))
            for seg in (reversed(segs) if flip else segs):
                a, b = (seg[0], seg[-1]) if (seg[0][0] < seg[-1][0]) != flip else (seg[-1], seg[0])
                rows.append((a, b))
            flip = not flip
            y += stepover
        for a, b in rows:
            linked = w.down and w.at is not None and keep_inside.contains(LineString([w.at, a]))
            if not linked:
                w.travel(*a)
                w.plunge_to(z)
            else:
                w.cut(*a)
            w.cut(*b)
        # Finishing contour along every boundary of the shrunk region.
        for poly in _polys(area):
            for ring in [poly.exterior, *poly.interiors]:
                coords = list(ring.coords)
                w.travel(*coords[0])
                w.plunge_to(z)
                for x, y2 in coords[1:]:
                    w.cut(x, y2)
        w.retract()


def star_in_circle(radius: float = 70.0, star_out: float = 42.0, star_in: float = 17.0) -> Polygon:
    """The demo part: a round pocket that leaves a five-point star standing."""
    return circle(0.0, 0.0, radius).difference(star(0.0, 0.0, star_out, star_in))
