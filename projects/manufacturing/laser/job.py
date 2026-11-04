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

"""Laser jobs as GRBL G-code, in the shape LightBurn writes for a GRBL laser.

`tag()` is the test part: a rounded 30 x 20 mm tag with a 6 mm hole, cut from 3 mm
plywood with Sienci's settings (100 %, 2.2 mm/s, 3 passes, Z down 1 mm a pass), and a
10 x 4 mm block engraved beside the hole (100 %, 25.4 mm/s, 0.1 mm line interval). Work
zero is the sheet's top centre with the beam focused on it (the twin powers up there;
`G10 L20 P1 X0 Y0 Z0` makes it work zero). Inner shapes are cut first, as LightBurn does,
so the hole is cut while the part is still held by the sheet. No kerf offset: the part
comes out a kerf smaller than drawn and the hole a kerf larger, as with LightBurn's default.

    python projects/manufacturing/laser/job.py --out projects/manufacturing/laser/jobs/tag_3mm_plywood.gcode
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import List, Sequence

S_MAX = 255                      # the LaserBeam's $30 (Sienci's LightBurn "S-value max")


def _f(v: float) -> str:
    s = f"{v:.3f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def rounded_rect(cx: float, cy: float, w: float, h: float, r: float, feed: float, s: int) -> List[str]:
    """Counter-clockwise from the middle of the bottom edge, corners as G3 arcs."""
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [f"G0 X{_f(cx)} Y{_f(y0)}",
            f"G1 X{_f(x1 - r)} Y{_f(y0)} S{s} F{_f(feed)}",
            f"G3 X{_f(x1)} Y{_f(y0 + r)} I0 J{_f(r)}",
            f"G1 X{_f(x1)} Y{_f(y1 - r)}",
            f"G3 X{_f(x1 - r)} Y{_f(y1)} I{_f(-r)} J0",
            f"G1 X{_f(x0 + r)} Y{_f(y1)}",
            f"G3 X{_f(x0)} Y{_f(y1 - r)} I0 J{_f(-r)}",
            f"G1 X{_f(x0)} Y{_f(y0 + r)}",
            f"G3 X{_f(x0 + r)} Y{_f(y0)} I{_f(r)} J0",
            f"G1 X{_f(cx)} Y{_f(y0)}"]


def circle(cx: float, cy: float, r: float, feed: float, s: int) -> List[str]:
    return [f"G0 X{_f(cx + r)} Y{_f(cy)}", f"G3 X{_f(cx + r)} Y{_f(cy)} I{_f(-r)} J0 S{s} F{_f(feed)}"]


def raster(cx: float, cy: float, w: float, h: float, interval: float, feed: float, s: int) -> List[str]:
    """Bidirectional lines along X. Between lines the head steps in Y with the laser off
    (G0); in M4 the power follows the speed, so the ends of each line are not overburnt."""
    out, n = [], int(round(h / interval)) + 1
    for k in range(n):
        y = cy - h / 2 + k * interval
        a, b = (cx - w / 2, cx + w / 2) if k % 2 == 0 else (cx + w / 2, cx - w / 2)
        out += [f"G0 X{_f(a)} Y{_f(y)}", f"G1 X{_f(b)} S{s} F{_f(feed)}"]
    return out


def ring_path(coords, feed: float, s: int) -> List[str]:
    """One closed outline: rapid to its start, then cut all the way round."""
    pts = list(coords)
    out = [f"G0 X{_f(pts[0][0])} Y{_f(pts[0][1])}", f"G1 X{_f(pts[1][0])} Y{_f(pts[1][1])} S{s} F{_f(feed)}"]
    out += [f"G1 X{_f(x)} Y{_f(y)}" for x, y in pts[2:]]
    return out


def cut_shape(shape, feed: float, s: int, kerf_mm: float = 0.0) -> List[str]:
    """Cut a part (a shapely Polygon or MultiPolygon, work mm) out of the sheet.

    With `kerf_mm`, the beam runs half a kerf OUTSIDE the part's outline and inside its
    holes (LightBurn's kerf offset), so the part comes out at its drawn size. Holes are
    cut first, while the part is still held by the sheet."""
    path = shape.buffer(kerf_mm / 2.0, quad_segs=32, join_style="round") if kerf_mm else shape
    polys = list(path.geoms) if path.geom_type == "MultiPolygon" else [path]
    out: List[str] = []
    for p in polys:
        for hole in p.interiors:
            out += ring_path(hole.coords, feed, s)
    for p in polys:
        out += ring_path(p.exterior.coords, feed, s)
    return out


def engrave_shape(shape, interval: float, feed: float, s: int) -> List[str]:
    """Raster-fill a region (shapely, work mm) with bidirectional lines along X, laser off
    (G0) between strokes. Meant for M4, so the strokes' ends are not overburnt."""
    from shapely.geometry import LineString
    x0, y0, x1, y1 = shape.bounds
    out: List[str] = []
    n = int(math.floor((y1 - y0) / interval)) + 1
    for k in range(n):
        y = y0 + (k + 0.5) * interval
        hit = shape.intersection(LineString([(x0 - 1, y), (x1 + 1, y)]))
        segs = [hit] if hit.geom_type == "LineString" else [g for g in getattr(hit, "geoms", []) if g.geom_type == "LineString"]
        segs = sorted((sorted((c[0] for c in g.coords)) for g in segs if not g.is_empty), key=lambda xs: xs[0])
        if k % 2:
            segs = [list(reversed(xs)) for xs in reversed(segs)]
        for xs in segs:
            out += [f"G0 X{_f(xs[0])} Y{_f(y)}", f"G1 X{_f(xs[-1])} S{s} F{_f(feed)}"]
    return out


def program(cuts: Sequence, engrave=None, passes: int = 3, z_step: float = 1.0, cut_mm_s: float = 2.2,
            engrave_mm_s: float = 25.4, kerf_mm: float = 0.0, title: str = "laser job",
            interval: float = 0.1) -> str:
    """A whole job: engrave first (one pass, focus on top), then every part cut in
    `passes` passes, the focus lowered `z_step` mm a pass. Work zero: the sheet's top
    centre, focused (the twin powers up there)."""
    lines = [f"; {title}", "G21 G90 G94", "G10 L20 P1 X0 Y0 Z0", "G54", "M4 S0"]
    if engrave is not None and not engrave.is_empty:
        lines += ["; engrave"] + engrave_shape(engrave, interval, engrave_mm_s * 60.0, S_MAX)
    for k in range(passes):
        lines += [f"; cut pass {k + 1}: focus {_f(-k * z_step)} mm", f"G0 Z{_f(-k * z_step)}"]
        for shape in cuts:
            lines += cut_shape(shape, cut_mm_s * 60.0, S_MAX, kerf_mm)
    lines += ["M5", "G0 Z0", "G0 X0 Y0", "M2", ""]
    return "\n".join(lines)


def tag(passes: int = 3, z_step: float = 1.0, cut_mm_s: float = 2.2, engrave_mm_s: float = 25.4,
        cut_power: float = 1.0, engrave_power: float = 1.0) -> str:
    cut_f, engrave_f = cut_mm_s * 60.0, engrave_mm_s * 60.0
    cs, es = int(round(S_MAX * cut_power)), int(round(S_MAX * engrave_power))
    lines = ["; OmniSim laser test part: rounded 30 x 20 mm tag, 6 mm hole, 10 x 4 mm engraved block",
             "; 3 mm plywood, Sienci LaserBeam 7W settings (lb-material-guide)",
             "G21 G90 G94", "G10 L20 P1 X0 Y0 Z0", "G54", "M4 S0",
             "; engrave"]
    lines += raster(5.0, 0.0, 10.0, 4.0, 0.1, engrave_f, es)
    for k in range(passes):
        z = -k * z_step
        lines += [f"; cut pass {k + 1}: focus {_f(z)} mm", f"G0 Z{_f(z)}"]
        lines += circle(-8.0, 0.0, 3.0, cut_f, cs)
        lines += rounded_rect(0.0, 0.0, 30.0, 20.0, 3.0, cut_f, cs)
    lines += ["M5", "G0 Z0", "G0 X0 Y0", "M2", ""]
    return "\n".join(lines)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Write the laser test part's G-code.")
    p.add_argument("--out", required=True)
    p.add_argument("--passes", type=int, default=3)
    p.add_argument("--z-step", type=float, default=1.0)
    a = p.parse_args(argv)
    Path(a.out).write_text(tag(a.passes, a.z_step), encoding="utf-8", newline="\n")
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
