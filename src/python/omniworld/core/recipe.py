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

"""The Recipe protocol every biome implements.

A recipe takes a seed and a parameter dict, builds an internal description
of the world (heightmap, prop list, spawn) using the primitives layer, and
hands that description to the emitter. Nothing else.

Recipes do not touch the filesystem, do not read the clock, and do not use
any RNG they did not derive from the seed they were handed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Mapping, Protocol

if TYPE_CHECKING:
    from ..primitives.heightmap import Heightmap


@dataclass(frozen=True)
class PlacedProto:
    """A concrete PROTO instance at a world transform.

    - ``proto_url`` is a ``omnisim://`` URL or repo-relative path; the
      emitter will pull it into an ``EXTERNPROTO`` declaration.
    - ``proto_type`` is the PROTO type name as written in OmniSim
      (e.g. ``"Oak"`` for ``Oak.proto``). Emitted as the node name.
    - ``extra_fields`` carries per-instance overrides (rotation of the
      mesh within the PROTO, scale, name). Emitted as ``key value``
      lines inside the instance block.
    """

    proto_url: str
    proto_type: str
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 0.0)
    extra_fields: tuple[tuple[str, str], ...] = ()


@dataclass(frozen=True)
class PlacedSolid:
    """A ``Solid`` with inline geometry, bypassing the PROTO system.

    Used when the recipe wants unique per-instance geometry that no
    existing PROTO can express — e.g. the mars biome's displaced
    icosphere rocks. The emitter writes a full ``Solid { ... }`` block
    with an inline ``IndexedFaceSet``, so no EXTERNPROTO is needed.

    ``face_indices`` is the OmniSim ``coordIndex`` flat list, with ``-1``
    terminating each face. ``bounding_radius`` defines a cheap
    ``Sphere`` boundingObject — far cheaper than mesh collision while
    usually correct enough for small props.
    """

    name: str
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]
    vertices: tuple[tuple[float, float, float], ...]
    face_indices: tuple[int, ...]
    diffuse_color: tuple[float, float, float] = (0.5, 0.5, 0.5)
    bounding_radius: float = 0.5
    # If set, also emit a ``scale`` field on the ``Shape`` so the same
    # generated mesh can be reused at a different size without re-
    # emitting vertices. Unused by Tier 1 biomes today.
    scale: float = 1.0
    # When False, the emitted Solid sets ``castShadows FALSE``. Stencil
    # shadow volumes are surprisingly expensive — cost scales with the
    # projected screen area of the shadow volume, not its mesh size —
    # and disabling them on small props (debris, rocks, boulders) skips
    # the entire shadow-volume rasterization for those objects per
    # directional light. The visual difference is minor at typical
    # viewing distances since small props cast small ground shadows.
    cast_shadows: bool = True


@dataclass(frozen=True)
class Spawn:
    """A named robot spawn site written into the emitted world."""

    name: str
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 0.0)
    urdf_url: str | None = None
    controller: str | None = None
    # When True the URDFRobot is emitted with ``supervisor TRUE``,
    # which lets the controller call ``robot.getSelf().getPosition()``
    # to read its own world pose at runtime — required for any
    # behaviour that needs to know where the robot is.
    supervisor: bool = False
    # Free-form string passed via the URDFRobot ``customData`` field;
    # the controller reads it via ``robot.getCustomData()``. Used by
    # the Mars biome to ship the crater list to the Husky controllers
    # for runtime obstacle avoidance.
    custom_data: str = ""


@dataclass
class WorldDescription:
    """In-memory, pre-emission representation of a generated world.

    Default shape is a ``RectangleArena`` (flat ground) with no
    scattered props. Biomes with terrain and props fill in
    ``heightmap`` + ``props`` and the emitter swaps in an
    ``ElevationGrid`` terrain and EXTERNPROTO declarations
    automatically.
    """

    title: str
    # Axis-aligned floor size in metres (x, y). Ignored when ``heightmap`` is set.
    floor_size: tuple[float, float] = (10.0, 10.0)
    # World time step (ms). Matches the OmniSim WorldInfo.basicTimeStep field.
    basic_time_step_ms: int = 16
    # Optional terrain. When present the emitter produces an
    # ``ElevationGrid`` instead of a ``RectangleArena``.
    heightmap: "Heightmap | None" = None
    # World-space cell size (metres per heightmap cell) in (x, y).
    heightmap_spacing: tuple[float, float] = (1.0, 1.0)
    # World-space translation of the terrain's (0, 0) corner. Biomes
    # typically centre the world on the origin so the robot spawn at
    # (0, 0) sits mid-terrain.
    heightmap_origin: tuple[float, float] = (0.0, 0.0)
    # Optional diffuse color override for the terrain material. When
    # None the emitter picks a default green-brown.
    terrain_color: tuple[float, float, float] | None = None
    # Optional flat-color sky override. When set (and ``sky_texture``
    # is None), the emitter emits ``Background { skyColor [r g b] }``
    # plus a plain DirectionalLight instead of TexturedBackground /
    # TexturedBackgroundLight. Useful for planetary / space / night
    # scenes where no shipped cubemap fits.
    sky_color: tuple[float, float, float] | None = None
    # Optional named HDR skybox. When set, the emitter emits
    # ``TexturedBackground { texture <name> }`` for the visible skybox
    # and PBR irradiance, and emits the recipe's own
    # ``DirectionalLight`` (using ``sun_*`` and ``fill_light_*``)
    # instead of the matched ``TexturedBackgroundLight`` — so a recipe
    # can pick the cubemap that fits its scene while keeping its
    # hand-tuned sun direction, colour, and intensity. Must be one of
    # the values supported by TexturedBackground.proto (``mars``,
    # ``mountains``, ``dusk``, ``factory``, ...). Takes precedence over
    # ``sky_color`` for the visible sky; ``sky_color`` is still used
    # as the fog-colour fallback.
    sky_texture: str | None = None
    # Optional procedural atmospheric sky preset. When set, the
    # emitter writes ``atmosphericSky "<preset>"`` onto whichever
    # Background flavour gets emitted (flat ``Background``,
    # ``TexturedBackground``, or default) — the renderer then swaps
    # the cubemap skybox for the Hillaire 2020 procedural pipeline.
    # Recognised values: ``earth``, ``mars``. Empty / None leaves the
    # cubemap path unchanged. See T1.3 in
    # docs/developer/rendering-roadmap.md (§2 for the feature, §7 for
    # the session-by-session progress on the shader math itself) —
    # enabling this before the math lands produces a magenta sky by
    # design, as a "scaffold is wired but math is missing" sentinel.
    sky_atmosphere: str | None = None
    # Optional directional light color + direction override. Ignored
    # unless ``sky_color`` is also set.
    sun_color: tuple[float, float, float] = (1.0, 0.95, 0.85)
    sun_direction: tuple[float, float, float] = (0.3, -0.5, -0.8)
    sun_intensity: float = 2.0
    # Whether the sun directional light (or TexturedBackgroundLight)
    # casts stencil shadows. Default False because shadow-volume
    # rasterization is the dominant forward-GPU bucket on prop-heavy
    # outdoor worlds — disabling it cuts forward GPU by ~50% (mars_big:
    # 5.83 → 2.87 ms; outdoor_forest: 5.44 ms with shadows). Visual cost:
    # no ground shadows from any object. Override to True at world-gen
    # time for cinematic captures.
    sun_cast_shadows: bool = False
    # Whether the post-process bloom effect runs. Bloom is a multi-pass
    # downsample/upsample chain that adds ~1 ms post-process at typical
    # window resolutions; visually it makes bright pixels glow. Default
    # False because robotics simulation rarely benefits from cinematic
    # bloom and the perf cost is meaningful. Override at world-gen time
    # to True for visual demos.
    bloom_enabled: bool = False
    # Whether GTAO (ground-truth ambient occlusion) runs. GTAO has a
    # post-process compute pass (~0.5 ms) and an apply pass mixed into
    # the forward render (~0.5 ms). Total impact ~1 ms total render
    # time. Visually adds soft contact shadows in concave areas — nicer
    # for indoor scenes, marginal for outdoor. Default False; override
    # to True for visually rich indoor captures.
    ambient_occlusion_enabled: bool = False
    # Optional fill light. When ``sky_color`` is set, the emitter can
    # add a second DirectionalLight from roughly the opposite direction
    # to avoid the single-light "flat" look. Disabled by default; the
    # mars recipe turns it on with a cool tint.
    fill_light_enabled: bool = False
    fill_light_color: tuple[float, float, float] = (0.6, 0.7, 0.85)
    fill_light_direction: tuple[float, float, float] = (-0.3, 0.5, -0.3)
    fill_light_intensity: float = 0.6
    # Optional atmospheric fog. Softens the horizon, hides distant
    # prop popping, and is the single biggest perceptual cue for a
    # "planetary" atmosphere. ``fog_color`` defaults to the sky when
    # unset; ``fog_visibility_range`` is the OmniSim field of the same
    # name (distance in metres at which fog becomes fully opaque).
    fog_enabled: bool = False
    fog_color: tuple[float, float, float] | None = None
    fog_visibility_range: float = 60.0
    fog_type: str = "EXPONENTIAL"  # OmniSim: "LINEAR", "EXPONENTIAL", "EXPONENTIAL2"
    # Scattered PROTO placements.
    props: list[PlacedProto] = field(default_factory=list)
    # Inline-geometry Solid placements. The emitter processes these
    # after ``props`` and writes full Solid blocks (no EXTERNPROTO).
    solids: list[PlacedSolid] = field(default_factory=list)
    # Robot spawns. Tier 1 stub has zero or one; later tiers have N.
    spawns: list[Spawn] = field(default_factory=list)
    # Free-form structured metadata surfaced into the seed manifest.
    metadata: dict[str, Any] = field(default_factory=dict)


class Recipe(Protocol):
    """A biome recipe.

    Implementations declare a ``name`` (unique, kebab-case-lower) and a
    ``build`` method that consumes a seed and parameter mapping. They do
    not own emission — the ``generate`` entry point does.
    """

    name: str

    def default_params(self) -> dict[str, Any]:
        """Return the recipe's default parameter set."""
        ...

    def build(self, seed: int, params: Mapping[str, Any]) -> WorldDescription:
        """Build the WorldDescription. Deterministic given (seed, params)."""
        ...
