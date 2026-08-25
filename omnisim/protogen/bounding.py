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

"""Typed boundingObject generators.

OmniSim's primitive ``boundingObject`` types (``Box``, ``Sphere``,
``Cylinder``, ``Capsule``, ``Plane``) can't express common shapes like a
hollow pipe or a torus, which is why the upstream
``PipeBoundingObject`` / ``TorusBoundingObject`` PROTOs exist. Those
helper PROTOs are runtime trampolines: they wrap a JavaScript template
that, on every instantiation, computes a ``Group`` of primitive boxes.

OmniSim's bounding generators do the same computation, but at PROTO
*author* time. The function emits a literal ``Group { children [ ... ] }``
VRML fragment that the author embeds directly in their PROTO body as a
``boundingObject`` value. The result:

* No EXTERNPROTO indirection.
* No JS template parsed at world load.
* The numbers in the resulting collision shape are visible in the
  ``.proto`` file (and survive ``diff``).

Example::

    from omnisim.protogen import emit
    from omnisim.protogen.bounding import pipe

    emit(
        name="MyPipeProp",
        fields=[
            ("SFVec3f", "translation", [0, 0, 0], "Pose."),
            ("SFRotation", "rotation", [0, 0, 1, 0], "Pose."),
        ],
        body=f'''
            Solid {{
              translation IS translation
              rotation IS rotation
              boundingObject {pipe(height=2.0, radius=0.5, thickness=0.05, subdivision=24)}
            }}
        ''',
    )
"""

from __future__ import annotations

import math

DEFAULT_SUBDIVISION = 24


def _fmt(v: float) -> str:
    if v == int(v):
        return str(int(v))
    return f"{v:.6f}".rstrip("0").rstrip(".") or "0"


def pipe(
    *,
    height: float,
    radius: float,
    thickness: float | None = None,
    subdivision: int = DEFAULT_SUBDIVISION,
    debug_shape: bool = False,
) -> str:
    """Emit a hollow-pipe boundingObject as a ``Group`` of primitive boxes.

    Args:
        height: Pipe length along the Z axis.
        radius: Outer radius.
        thickness: Wall thickness. Defaults to ``radius * 0.5`` if omitted.
        subdivision: Number of boxes around the circumference. Must be
            ``>= 8``.
        debug_shape: If True, wrap each Box in a ``Shape`` for visual
            inspection. Defaults to False (collision-only).

    The math mirrors the upstream ``PipeBoundingObject.proto`` template
    so the generated geometry is interchangeable with it.
    """
    if subdivision < 8:
        raise ValueError(f"subdivision must be >= 8, got {subdivision}")
    if height <= 0:
        raise ValueError(f"height must be > 0, got {height}")
    if radius <= 0:
        raise ValueError(f"radius must be > 0, got {radius}")
    if thickness is None:
        thickness = radius * 0.5
    if thickness <= 0 or thickness >= radius:
        raise ValueError(f"thickness must be in (0, radius); got {thickness} (radius={radius})")

    beta = 2.0 * math.pi / subdivision
    alpha = beta * 0.5
    inner = radius - thickness
    su = 2.0 * radius * math.sin(alpha)
    sv = max(radius * math.cos(alpha) - inner, abs(radius * math.cos(alpha) - inner))
    sw = height
    box_radius = inner + sv * 0.5

    children: list[str] = []
    for i in range(subdivision):
        gamma = beta * i + beta * 0.5 + (subdivision % 2) * math.pi / subdivision
        ax = box_radius * math.cos(gamma)
        ay = box_radius * math.sin(gamma)
        angle = gamma + 0.5 * math.pi
        inner_geom = f"Box {{ size {_fmt(su)} {_fmt(sv)} {_fmt(sw)} }}"
        if debug_shape:
            inner_geom = (
                "Shape {\n"
                "          appearance Appearance { material Material { } }\n"
                f"          geometry {inner_geom}\n"
                "        }"
            )
        children.append(
            "      Pose {\n"
            f"        translation {_fmt(ax)} {_fmt(ay)} 0\n"
            f"        rotation 0 0 1 {_fmt(angle)}\n"
            "        children [\n"
            f"          {inner_geom}\n"
            "        ]\n"
            "      }"
        )
    return "Group {\n    children [\n" + "\n".join(children) + "\n    ]\n  }"


def torus(
    *,
    major_radius: float,
    minor_radius: float,
    subdivision: int = DEFAULT_SUBDIVISION,
) -> str:
    """Emit a torus boundingObject as a ``Group`` of primitive spheres.

    Args:
        major_radius: Distance from the torus center to the tube center.
        minor_radius: Radius of the tube cross-section.
        subdivision: Number of spheres around the major circle.

    Mirrors the upstream ``TorusBoundingObject.proto`` template.
    """
    if subdivision < 8:
        raise ValueError(f"subdivision must be >= 8, got {subdivision}")
    if major_radius <= 0 or minor_radius <= 0:
        raise ValueError("radii must be positive")

    children: list[str] = []
    for i in range(subdivision):
        theta = 2.0 * math.pi * i / subdivision
        x = major_radius * math.cos(theta)
        y = major_radius * math.sin(theta)
        children.append(
            "      Pose {\n"
            f"        translation {_fmt(x)} {_fmt(y)} 0\n"
            "        children [\n"
            f"          Sphere {{ radius {_fmt(minor_radius)} subdivision 2 }}\n"
            "        ]\n"
            "      }"
        )
    return "Group {\n    children [\n" + "\n".join(children) + "\n    ]\n  }"
