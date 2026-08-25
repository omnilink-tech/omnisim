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

"""Emit a ``.wbt`` world file from a ``WorldDescription``.

Supports two shapes in parallel:

- **Flat arena.** ``heightmap is None`` — the emitter produces a
  ``RectangleArena`` over the declared ``floor_size``. Matches the
  flat_ground stub used by T1.1.
- **Terrain + props.** ``heightmap`` and ``props`` are set — the
  emitter produces an ``ElevationGrid``-backed terrain plus one
  node per prop, with de-duplicated ``EXTERNPROTO`` declarations at
  the top.

Output is byte-stable: no timestamps, fixed field order, deterministic
prop emission order (the solver's output order is honoured verbatim).
"""

from __future__ import annotations

from ..core.recipe import PlacedProto, PlacedSolid, Spawn, WorldDescription
from ..viewpoint import format_orientation, format_position, hero_view, top_down_view
from .elevation_grid import render_terrain_solid

_OMNISIM_HEADER = "#OMNISIM R2025a utf8\n"

_TEXTURED_BG_EXTERNPROTO = (
    "EXTERNPROTO \"omnisim://projects/objects/backgrounds/protos/TexturedBackground.proto\""
)
_TEXTURED_BG_LIGHT_EXTERNPROTO = (
    "EXTERNPROTO \"omnisim://projects/objects/backgrounds/protos/TexturedBackgroundLight.proto\""
)
_TEXTURED_SKY_EXTERNPROTOS = (
    _TEXTURED_BG_EXTERNPROTO,
    _TEXTURED_BG_LIGHT_EXTERNPROTO,
)
_ARENA_EXTERNPROTO = (
    "EXTERNPROTO \"omnisim://projects/objects/floors/protos/RectangleArena.proto\""
)


def _fmt_float(x: float) -> str:
    return format(x, ".6g")


def _fmt_vec3(v: tuple[float, float, float]) -> str:
    return " ".join(_fmt_float(c) for c in v)


def _fmt_rot4(v: tuple[float, float, float, float]) -> str:
    return " ".join(_fmt_float(c) for c in v)


def _emit_recipe_lights(out: list[str], world: WorldDescription) -> None:
    """Emit the recipe's hand-tuned sun and optional fill light.

    Used by both the flat-colour ``Background`` path and the named-
    cubemap ``TexturedBackground`` path, so a recipe's ``sun_*`` and
    ``fill_light_*`` settings are honoured either way.
    """
    cr, cg, cb = world.sun_color
    dx, dy, dz = world.sun_direction
    out.append("DirectionalLight {")
    out.append(f"  color {_fmt_float(cr)} {_fmt_float(cg)} {_fmt_float(cb)}")
    out.append(f"  direction {_fmt_float(dx)} {_fmt_float(dy)} {_fmt_float(dz)}")
    out.append(f"  intensity {_fmt_float(world.sun_intensity)}")
    out.append(f"  castShadows {'TRUE' if world.sun_cast_shadows else 'FALSE'}")
    out.append("}")
    if world.fill_light_enabled:
        fcr, fcg, fcb = world.fill_light_color
        fdx, fdy, fdz = world.fill_light_direction
        out.append("DirectionalLight {")
        out.append(f"  color {_fmt_float(fcr)} {_fmt_float(fcg)} {_fmt_float(fcb)}")
        out.append(f"  direction {_fmt_float(fdx)} {_fmt_float(fdy)} {_fmt_float(fdz)}")
        out.append(f"  intensity {_fmt_float(world.fill_light_intensity)}")
        out.append("  castShadows FALSE")
        out.append("}")


def _emit_spawn(spawn: Spawn) -> list[str]:
    lines: list[str] = []
    if spawn.urdf_url is None:
        # If the spawn declares a controller, emit a plain Robot
        # node (the OmniSim URDFRobot importer strips supervisor +
        # customData on URDFRobot blocks, but those fields *do*
        # work on a plain Robot). This is how the Mars biome adds
        # an observer Supervisor to the world.
        if spawn.controller:
            lines.append("Robot {")
            lines.append(f"  translation {_fmt_vec3(spawn.translation)}")
            lines.append(f"  rotation {_fmt_rot4(spawn.rotation)}")
            lines.append(f"  name \"{spawn.name}\"")
            lines.append(f"  controller \"{spawn.controller}\"")
            if spawn.supervisor:
                lines.append("  supervisor TRUE")
            if spawn.custom_data:
                escaped = spawn.custom_data.replace("\\", "\\\\").replace("\"", "\\\"")
                lines.append(f"  customData \"{escaped}\"")
            lines.append("  children []")
            lines.append("}")
            return lines
        lines.append(f"DEF {spawn.name} Transform {{")
        lines.append(f"  translation {_fmt_vec3(spawn.translation)}")
        lines.append(f"  rotation {_fmt_rot4(spawn.rotation)}")
        lines.append("}")
        return lines

    lines.append("URDFRobot {")
    lines.append(f"  url \"{spawn.urdf_url}\"")
    lines.append(f"  translation {_fmt_vec3(spawn.translation)}")
    lines.append(f"  rotation {_fmt_rot4(spawn.rotation)}")
    lines.append(f"  name \"{spawn.name}\"")
    if spawn.controller:
        lines.append(f"  controller \"{spawn.controller}\"")
    if spawn.supervisor:
        lines.append("  supervisor TRUE")
    if spawn.custom_data:
        # Quote and escape so VRML accepts the string. Backslashes and
        # double quotes are the two characters we have to handle.
        escaped = spawn.custom_data.replace("\\", "\\\\").replace("\"", "\\\"")
        lines.append(f"  customData \"{escaped}\"")
    lines.append("}")
    return lines


def _emit_solid(solid: PlacedSolid) -> list[str]:
    """Emit a complete ``Solid { ... }`` block with inline
    IndexedFaceSet geometry. The Solid is DEF'd so later controllers
    can grab it by name.

    ``Solid`` extends Pose, so it has neither a ``scale`` nor a
    ``castShadows`` field (emitting them used to flood the load log with
    "Skipped unknown field" errors). When ``solid.scale != 1.0`` we wrap
    the Shape in a child ``Transform { scale ... }`` rather than
    pre-multiplying the vertices. That lets WREN's StaticMesh cache
    (which keys on sipHash13c of the coord/index data) share a single
    GPU upload across every PlacedSolid that has byte-identical
    ``vertices``/``face_indices`` — the win the mars rock-template
    refactor expects. Pre-multiplying baked the per-instance scale
    into the vertex stream, defeating the cache.
    """
    safe_name = solid.name.replace(" ", "_").replace("\"", "")

    shape: list[str] = []
    shape.append("Shape {")
    if not solid.cast_shadows:
        # Skip stencil shadow-volume work for this Shape. Set to FALSE
        # on small props to avoid paying per-light shadow-volume
        # rasterization cost for objects whose ground shadows are
        # barely visible anyway.
        shape.append("  castShadows FALSE")
    shape.append("  appearance PBRAppearance {")
    r, g, b = solid.diffuse_color
    shape.append(
        f"    baseColor {_fmt_float(r)} {_fmt_float(g)} {_fmt_float(b)}"
    )
    shape.append("    roughness 0.9")
    shape.append("    metalness 0")
    shape.append("  }")
    shape.append("  geometry IndexedFaceSet {")
    shape.append("    coord Coordinate {")
    shape.append("      point [")
    # Pack coords 3 per line for diff-friendliness. Vertices are
    # written *unscaled* — the per-instance scale is applied via the
    # wrapper Transform below.
    for vx, vy, vz in solid.vertices:
        shape.append(
            f"        {_fmt_float(vx)} {_fmt_float(vy)} {_fmt_float(vz)}"
        )
    shape.append("      ]")
    shape.append("    }")
    shape.append("    coordIndex [")
    # Emit face indices, 4 ints per line (one triangle + -1 terminator).
    buf: list[str] = []
    for v in solid.face_indices:
        buf.append(str(v))
        if v == -1:
            shape.append("      " + " ".join(buf))
            buf = []
    if buf:
        shape.append("      " + " ".join(buf))
    shape.append("    ]")
    shape.append("    creaseAngle 0.8")
    shape.append("  }")
    shape.append("}")

    lines: list[str] = []
    lines.append(f"DEF {safe_name.upper()} Solid {{")
    lines.append(f"  translation {_fmt_vec3(solid.translation)}")
    lines.append(f"  rotation {_fmt_rot4(solid.rotation)}")
    lines.append(f"  name \"{safe_name}\"")
    lines.append("  children [")
    if solid.scale != 1.0:
        s = _fmt_float(solid.scale)
        lines.append("    Transform {")
        lines.append(f"      scale {s} {s} {s}")
        lines.append("      children [")
        lines.extend("        " + sl for sl in shape)
        lines.append("      ]")
        lines.append("    }")
    else:
        lines.extend("    " + sl for sl in shape)
    lines.append("  ]")
    # Cheap sphere boundingObject — much cheaper than mesh collision.
    # The wrapper Transform does NOT scale the boundingObject, so the
    # per-instance scale is baked into the sphere radius here.
    lines.append("  boundingObject Sphere {")
    lines.append(f"    radius {_fmt_float(solid.bounding_radius * solid.scale)}")
    lines.append("  }")
    lines.append("}")
    return lines


def _emit_prop(prop: PlacedProto, instance_index: int) -> list[str]:
    lines = [f"{prop.proto_type} {{"]
    lines.append(f"  translation {_fmt_vec3(prop.translation)}")
    lines.append(f"  rotation {_fmt_rot4(prop.rotation)}")
    # OmniSim warns on sibling Solids that share a ``name``. The recipe
    # may already have supplied one via extra_fields; otherwise we
    # synthesise a deterministic per-instance name from the PROTO
    # type and the global placement index.
    has_explicit_name = any(k == "name" for k, _ in prop.extra_fields)
    if not has_explicit_name:
        lines.append(f"  name \"{prop.proto_type}_{instance_index}\"")
    for key, value in prop.extra_fields:
        lines.append(f"  {key} {value}")
    lines.append("}")
    return lines


def _unique_prop_urls(props: list[PlacedProto]) -> list[str]:
    """Deterministic unique list preserving first-use order."""
    seen: set[str] = set()
    out: list[str] = []
    for p in props:
        if p.proto_url not in seen:
            seen.add(p.proto_url)
            out.append(p.proto_url)
    return out


def _frame_world(world: WorldDescription) -> tuple[tuple[float, float, float], float, str]:
    """Pick the default camera ``(center, radius, mode)`` for a generated world.

    The subject is the robot spawn(s) when present, otherwise the scene extent
    (terrain or arena). Distance/orientation are then derived by
    ``omniworld.viewpoint``. See docs/developer/viewpoint-convention.md.
    """
    if world.spawns:
        xs = [s.translation[0] for s in world.spawns]
        ys = [s.translation[1] for s in world.spawns]
        zs = [s.translation[2] for s in world.spawns]
        cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
        center = (cx, cy, max(zs) + 0.4)
        if len(world.spawns) == 1:
            # Single robot: a tight, body-centred hero shot.
            return center, 1.4, "hero"
        # A group of robots: frame them all, with a little breathing room.
        spread = max(max(xs) - min(xs), max(ys) - min(ys))
        return center, max(spread / 2.0 + 1.5, 2.0), "hero"
    if world.heightmap is not None:
        ex = (world.heightmap.width - 1) * world.heightmap_spacing[0]
        ey = (world.heightmap.height - 1) * world.heightmap_spacing[1]
        cx = world.heightmap_origin[0] + ex / 2.0
        cy = world.heightmap_origin[1] + ey / 2.0
        return (cx, cy, 0.0), max(ex, ey, 1.0) / 2.0, "hero"
    fx, fy = world.floor_size
    return (0.0, 0.0, 0.0), max(fx, fy, 1.0) / 2.0, "hero"


def render_wbt(world: WorldDescription) -> str:
    """Render the ``WorldDescription`` to a ``.wbt`` text string."""
    out: list[str] = [_OMNISIM_HEADER, ""]

    externprotos: list[str] = []
    if world.sky_atmosphere:
        # Atmospheric mode: emit a plain Background — no cubemap files
        # are loaded.  PBR irradiance is fed by the procedural sky
        # bake (T1.3 session 5).
        pass
    elif world.sky_texture is not None:
        # Recipe-tuned sun: skybox PROTO only, light is hand-emitted.
        externprotos.append(_TEXTURED_BG_EXTERNPROTO)
    elif world.sky_color is None:
        externprotos.extend(_TEXTURED_SKY_EXTERNPROTOS)
    if world.heightmap is None:
        externprotos.append(_ARENA_EXTERNPROTO)
    for url in _unique_prop_urls(world.props):
        externprotos.append(f"EXTERNPROTO \"{url}\"")
    out.extend(externprotos)
    out.append("")

    out.append("WorldInfo {")
    out.append(f"  basicTimeStep {world.basic_time_step_ms}")
    out.append(f"  title \"{world.title}\"")
    out.append("}")
    out.append("Viewpoint {")
    # Default camera: frame the subject (robot spawn if present, else the
    # scene) with the standard angled hero view so the world opens looking at
    # something, not at the engine's fixed -10 0 0 fallback. See
    # docs/developer/viewpoint-convention.md.
    cam_center, cam_radius, cam_mode = _frame_world(world)
    cam_eye, cam_orient = (
        top_down_view(cam_center, cam_radius)
        if cam_mode == "topdown"
        else hero_view(cam_center, cam_radius)
    )
    out.append(f"  orientation {format_orientation(cam_orient)}")
    out.append(f"  position {format_position(cam_eye)}")
    # Post-process effect controls. OmniSim defaults bloomThreshold=21
    # (bloom on) and ambientOcclusionRadius=2 (GTAO on); both cost real
    # GPU time on every frame. We default them off because robotics
    # simulation rarely needs cinematic polish and the savings are
    # significant — together they cut total render time on mars_big by
    # ~2.6 ms (5.0 → 2.4 ms forward+post). Set
    # bloom_enabled=True / ambient_occlusion_enabled=True at world-gen
    # time when you want the visual effects.
    if not world.bloom_enabled:
        out.append("  bloomThreshold -1.0")
    if not world.ambient_occlusion_enabled:
        out.append("  ambientOcclusionRadius 0")
    out.append("}")

    atmosphere = (world.sky_atmosphere or "").strip()
    if atmosphere:
        # Atmospheric mode: plain Background with skyColor as the
        # ``OMNISIM_RENDERER=compatibility`` fallback, plus the
        # ``atmosphericSky`` selector.  No cubemap URLs — the renderer
        # generates the entire sky procedurally and bakes its own
        # irradiance.
        out.append("Background {")
        if world.sky_color is not None:
            sr, sg, sb = world.sky_color
            out.append(
                f"  skyColor [ {_fmt_float(sr)} {_fmt_float(sg)} {_fmt_float(sb)} ]"
            )
        out.append(f"  atmosphericSky \"{atmosphere}\"")
        out.append("}")
        _emit_recipe_lights(out, world)
    elif world.sky_texture is not None:
        out.append("TexturedBackground {")
        out.append(f"  texture \"{world.sky_texture}\"")
        if world.sky_color is not None:
            # Fallback solid colour visible if the cubemap fails to load
            # AND drives the `skybox FALSE` deprecation path; kept here
            # so worlds that set both fields stay consistent.
            sr, sg, sb = world.sky_color
            out.append(
                f"  skyColor [ {_fmt_float(sr)} {_fmt_float(sg)} {_fmt_float(sb)} ]"
            )
        out.append("}")
        _emit_recipe_lights(out, world)
    elif world.sky_color is None:
        out.append("TexturedBackground {")
        out.append("}")
        out.append("TexturedBackgroundLight {")
        out.append("  luminosity 0.95")
        # Disable stencil shadow casting for the same reason as the
        # explicit DirectionalLight branch below: shadow-volume cost is
        # the dominant forward-GPU bucket on prop-heavy outdoor worlds,
        # and TexturedBackgroundLight defaults castShadows=TRUE which
        # routes the entire scene through the per-light stencil path.
        # Honours world.sun_cast_shadows so the world generator can
        # opt back in for cinematic captures.
        out.append(f"  castShadows {'TRUE' if world.sun_cast_shadows else 'FALSE'}")
        out.append("}")
    else:
        sr, sg, sb = world.sky_color
        out.append("Background {")
        out.append(f"  skyColor [ {_fmt_float(sr)} {_fmt_float(sg)} {_fmt_float(sb)} ]")
        out.append("}")
        _emit_recipe_lights(out, world)

    if world.fog_enabled:
        # Fog colour defaults to the sky colour for a unified horizon.
        fog_color = world.fog_color
        if fog_color is None and world.sky_color is not None:
            fog_color = world.sky_color
        if fog_color is None:
            fog_color = (0.8, 0.8, 0.85)
        fr, fg, fb = fog_color
        out.append("Fog {")
        out.append(f"  color {_fmt_float(fr)} {_fmt_float(fg)} {_fmt_float(fb)}")
        out.append(f"  visibilityRange {_fmt_float(world.fog_visibility_range)}")
        out.append(f"  fogType \"{world.fog_type}\"")
        out.append("}")

    if world.heightmap is None:
        fx, fy = world.floor_size
        out.append("RectangleArena {")
        out.append(f"  floorSize {_fmt_float(fx)} {_fmt_float(fy)}")
        out.append("  wallHeight 0.05")
        out.append("}")
    else:
        xs, ys = world.heightmap_spacing
        ox, oy = world.heightmap_origin
        kwargs = {
            "x_spacing": xs,
            "y_spacing": ys,
            "translation": (ox, oy, 0.0),
            "name": "terrain",
        }
        if world.terrain_color is not None:
            kwargs["diffuse_color"] = world.terrain_color
        terrain = render_terrain_solid(world.heightmap, **kwargs)
        out.append(terrain)

    for i, prop in enumerate(world.props):
        out.append("")
        out.extend(_emit_prop(prop, i))

    for solid in world.solids:
        out.append("")
        out.extend(_emit_solid(solid))

    for spawn in world.spawns:
        out.append("")
        out.extend(_emit_spawn(spawn))

    return "\n".join(out) + "\n"
