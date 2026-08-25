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

"""Emit OmniSim ``ElevationGrid`` geometry from a ``Heightmap``.

Two entry points:

- ``render_elevation_grid(hm, x_spacing=..., y_spacing=...)`` — emit just
  the ``ElevationGrid { ... }`` block. Callers wrap it in whatever
  ``Solid`` / ``Shape`` they need.
- ``render_terrain_solid(hm, ...)`` — convenience: wraps the grid in a
  ``Solid > Shape`` with a configurable appearance. Outdoor biomes call
  this directly.

Output is byte-stable: field order fixed, ``height`` list broken across
lines for diffability, no trailing whitespace.
"""

from __future__ import annotations

from ..primitives.heightmap import Heightmap


def _fmt_float(x: float) -> str:
    return format(x, ".6g")


def _fmt_height_list(hm: Heightmap, per_line: int = 16) -> list[str]:
    """Chunk the flat height array into lines so the output diffs cleanly."""
    out: list[str] = []
    data = hm.data
    for start in range(0, len(data), per_line):
        chunk = data[start : start + per_line]
        out.append(", ".join(_fmt_float(v) for v in chunk))
    return out


def render_elevation_grid(
    hm: Heightmap,
    *,
    x_spacing: float = 1.0,
    y_spacing: float = 1.0,
    indent: str = "  ",
) -> str:
    """Emit a bare ``ElevationGrid`` node.

    ``x_spacing`` / ``y_spacing`` are the world-space cell edge lengths in
    metres. Width × x_spacing is the terrain's world extent in x, same
    for height in y.
    """
    if x_spacing <= 0.0 or y_spacing <= 0.0:
        raise ValueError("render_elevation_grid: spacing must be positive")

    lines: list[str] = ["ElevationGrid {"]
    lines.append(f"{indent}height [")
    for chunk in _fmt_height_list(hm):
        lines.append(f"{indent}{indent}{chunk}")
    lines.append(f"{indent}]")
    lines.append(f"{indent}xDimension {hm.width}")
    lines.append(f"{indent}xSpacing {_fmt_float(x_spacing)}")
    lines.append(f"{indent}yDimension {hm.height}")
    lines.append(f"{indent}ySpacing {_fmt_float(y_spacing)}")
    lines.append("}")
    return "\n".join(lines)


def render_terrain_solid(
    hm: Heightmap,
    *,
    x_spacing: float = 1.0,
    y_spacing: float = 1.0,
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    name: str = "terrain",
    diffuse_color: tuple[float, float, float] = (0.45, 0.50, 0.35),
) -> str:
    """Convenience wrapper: ``Solid > Shape { appearance, geometry }``.

    The appearance is deliberately minimal. Later tiers hook up the
    splatmap / PBR material system ([rendering-roadmap.md] T3.1 extended
    BRDF); Tier 1 just needs a default green-brown for the outdoor demo.
    """
    dc = " ".join(_fmt_float(c) for c in diffuse_color)
    tx = " ".join(_fmt_float(c) for c in translation)
    grid_lines = render_elevation_grid(
        hm, x_spacing=x_spacing, y_spacing=y_spacing, indent="    "
    ).splitlines()

    lines: list[str] = [f"DEF {name.upper()} Solid {{"]
    lines.append(f"  translation {tx}")
    lines.append(f"  name \"{name}\"")
    lines.append("  children [")
    lines.append("    Shape {")
    lines.append("      appearance Appearance {")
    lines.append("        material Material {")
    lines.append(f"          diffuseColor {dc}")
    lines.append("        }")
    lines.append("      }")
    lines.append("      geometry " + grid_lines[0])
    # grid_lines[0] already starts "ElevationGrid {"; append the rest
    # verbatim, which are already indented by render_elevation_grid.
    for line in grid_lines[1:]:
        lines.append(line)
    lines.append("    }")
    lines.append("  ]")
    # boundingObject mirrors the geometry so collision works for free.
    lines.append("  boundingObject " + grid_lines[0])
    for line in grid_lines[1:]:
        lines.append(line)
    lines.append("}")
    return "\n".join(lines)


__all__ = ["render_elevation_grid", "render_terrain_solid"]
