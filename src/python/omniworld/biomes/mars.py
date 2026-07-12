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

"""``mars`` — Mars-like rocky outdoor biome.

Dramatic ridged-plus-fbm relief with impact craters, three-layer
rock scatter (fine debris, regular rocks, sparse boulders), ground-
fitted rotation so nothing floats on slopes, atmospheric fog, an
ochre sky, and a warm-white sun low on the horizon.

Uses the shared realism machinery (per-instance scale, hue shift,
weathering grade) everywhere it can — no Mars-specific shaders,
just catalog entries and the existing primitive set.

No new PROTOs: rocks are the stock ``Rock`` PROTO. The photo-
realistic-geometry problem is called out in the plan as T3.8
(photoscanned asset library) and is out of scope here.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any, Mapping

from ..catalog import get_url
from ..core.layout import Layout, Stamp, Zone
from ..core.layout import PropGroup as _PropGroup
from ..core.recipe import PlacedProto, PlacedSolid, Spawn, WorldDescription
from ..core.registry import register_recipe
from ..core.rng import derive_seed, rng_from_seed
from ..core.solver import solve
from ..primitives.heightmap import (
    Heightmap,
    crater_pattern,
    stamp as heightmap_stamp,
)
from ..primitives.rock_mesh import generate_rock
from ..realism import compute_grade, grade_to_color_multiplier
from ..realism.ground_fit import ground_fit_on_heightmap
from ..realism.variation import jitter_hue_rgb, variation_for_instance


DEFAULT_PARAMS: dict[str, Any] = {
    # Terrain size + resolution.
    "size": 80.0,
    "grid_resolution": 256,
    "height_scale": 5.5,
    "octaves": 6,
    "gain": 0.55,
    "lacunarity": 2.1,
    "mesa_mix": 0.68,
    # Spawn clearing — also the level pad the huskies land on.
    # Wide enough that the husky spawn circle at 12 m radius is well
    # inside the levelled pad with margin on every side.
    "clearing_radius": 14.0,
    # Impact craters. Depth + radius + rim_width are picked so the
    # *steepest* possible wall stays under ~12° for any seed — well
    # within Husky climbing capability. Shallower than real Mars
    # craters but the visual silhouette still reads as one. If you
    # want deeper basins for non-driving demos, override
    # crater_max_depth_m.
    "crater_count": 7,
    "crater_min_radius_m": 4.0,
    "crater_max_radius_m": 9.0,
    "crater_min_depth_m": 0.10,
    "crater_max_depth_m": 0.30,
    # Minimum distance (in metres) from the clearing centre that a
    # crater's rim is allowed to touch. Keeps craters at the periphery
    # so the husky operating zone is crater-free — the controller's
    # active avoidance is then a belt-and-braces fallback rather than
    # a load-bearing safety mechanism.
    "crater_exclusion_radius_m": 28.0,
    # Three-layer rock scatter.
    # Three-layer rock scatter. Scale ranges widened relative to the
    # earlier defaults so each layer covers a wider size spectrum;
    # combined with log-uniform sampling (see scale_distribution
    # below), the field reads like a natural rock-strewn surface where
    # small pebbles vastly outnumber boulders. Real planetary rock
    # populations follow N(d > D) ∝ D^(-α) with α ≈ 2-3, which is
    # closer to log-uniform than to linear-uniform across each layer's
    # range.
    "debris_count": 280,
    "debris_min_dist": 0.45,
    "debris_scale_min": 0.10,
    "debris_scale_max": 0.50,
    "rock_count": 120,
    "rock_min_dist": 1.3,
    "rock_scale_min": 0.4,
    "rock_scale_max": 2.0,
    "boulder_count": 20,
    "boulder_min_dist": 3.5,
    "boulder_scale_min": 1.5,
    "boulder_scale_max": 4.5,
    # Sampling distribution for rock instance scales. ``log_uniform``
    # biases each draw toward the small end of [scale_min, scale_max]:
    # log(scale) is uniform between log(min) and log(max). Combined
    # with the wide ranges above, this yields the heavy-tailed size
    # mix that real rocky surfaces have. Set to ``uniform`` for the
    # earlier evenly-distributed look.
    "rock_scale_distribution": "log_uniform",
    # Rock colouring. All three layers draw from the same rust palette,
    # with per-instance hue jitter so no two rocks are pixel-identical.
    "rock_base_color": (0.52, 0.27, 0.17),
    "rock_hue_shift_deg": 12.0,
    # World-level age drives a wear/oxidation darkening per rock.
    "world_age": 0.45,
    # Atmosphere: Hillaire 2020 procedural sky tuned for Mars
    # (CO2 + dust, ~6 mbar surface pressure).  Sun direction is
    # supplied by the recipe via ``sun_direction``; the renderer
    # negates that sign internally because the shader convention is
    # TOWARD-sun, not direction-of-light-travel.
    #
    # ``sky_texture`` and ``sky_color`` are kept as fallbacks the
    # renderer falls back to if atmospheric mode is unavailable
    # (e.g. ``OMNISIM_RENDERER=compatibility``).  The cubemap also
    # still feeds PBR irradiance reflections — session 5 of the
    # implementation plan replaces that with a procedural irradiance
    # bake from the sky-view LUT.
    "sky_atmosphere": "mars",
    "sky_texture": "mars",
    "sky_color": (0.70, 0.40, 0.25),
    "sun_color": (1.0, 0.78, 0.62),
    "sun_direction": (0.35, -0.35, -0.55),
    "sun_intensity": 3.2,
    # Sun stencil shadow casting. Default False because mars is the
    # primary big-world perf benchmark — disabling shadow volumes
    # cuts forward GPU by ~47% (5.83 → 3.07 ms on mars_big). Visual
    # cost: no ground shadows from huskies, rocks, or terrain props.
    # If you want the shadowed look (smaller worlds, cinematic
    # captures), set this to True at world-gen time.
    "sun_cast_shadows": False,
    "fog_enabled": True,
    "fog_color": (0.78, 0.52, 0.35),
    "fog_visibility_range": 55.0,
    # Fill light to lift the shadow side — dusty Mars sky scatters some
    # light even without proper atmospheric rendering.
    "fill_light_enabled": True,
    "fill_light_color": (0.85, 0.55, 0.4),
    "fill_light_direction": (-0.3, 0.4, -0.3),
    "fill_light_intensity": 0.8,
    # Rock mesh detail. ``subdivisions=3`` = 642 verts / 1280 faces per
    # rock; ``=2`` = 162 verts / 320 faces (cheaper, still unique).
    # Defaults dropped to 2 across all layers because (a) since γ each
    # layer only generates ~5 mesh templates that get shared across
    # hundreds of placements via WREN's StaticMesh cache, so per-frame
    # draw counts no longer scale with rock count — but per-instance
    # *vertex* and *shadow-volume edge* costs do, and shadow-volume
    # geometry is recomputed per light from the mesh edge set; (b) at
    # typical scatter densities the silhouette difference between
    # subdiv=2 and subdiv=3 is imperceptible from the husky's working
    # height. Boulders kept at 3 because they're large enough that the
    # extra silhouette detail reads at viewing distance.
    "rock_mesh_subdivisions": 2,
    "debris_mesh_subdivisions": 2,
    "boulder_mesh_subdivisions": 3,
    # Number of distinct mesh templates generated per scatter layer.
    # Each placed rock picks one of N templates (round-robin by index)
    # and gets its visual diversity from per-instance scale, rotation,
    # ground-tilt, and hue jitter — not from a unique mesh per rock.
    # Identical-template instances share their GPU upload via WREN's
    # mesh cache (sipHash13c on coord/index data), which collapses
    # per-frame draw calls dramatically on shadow-cast scenes. Bigger
    # N = more silhouette variety but less draw-call sharing; 5 is
    # the smallest count where the repetition isn't obvious at typical
    # scatter densities.
    "rock_template_count": 5,
    "debris_template_count": 5,
    "boulder_template_count": 4,
    # Displacement strength as fraction of radius. Bigger = rockier
    # silhouette, smaller = smoother boulder.
    "rock_displacement": 0.35,
    "debris_displacement": 0.30,
    "boulder_displacement": 0.40,
    "rock_elongation_max": 0.45,
    # Dusty iron-oxide terrain colour.
    "terrain_color": (0.58, 0.33, 0.22),
    # Ground fit: cap the tilt so a rock on a ridge does not end up
    # nearly horizontal (beyond angle-of-repose it would tumble).
    "max_tilt_deg": 28.0,
    # Optional URDF rover spawn at the clearing. Mutually exclusive
    # with husky_count.
    "spawn_urdf": None,
    "spawn_controller": None,
    # Height above the clearing surface at which robots spawn. Real
    # Webots physics pulls them down onto the ground on the first
    # step, so 0.5-1.5 m is the usual range.
    "spawn_height": 0.5,
    # Husky fleet option. Drops N Clearpath Huskies on the clearing in
    # a wedge formation — e.g. a multi-agent exploration scenario.
    # Set to 0 to disable. The wedge packs tight enough to fit inside
    # the default 5.5 m clearing radius.
    "husky_count": 0,
    # Vertical offset added above the clearing pad surface for husky
    # spawns. Real Husky robots never free-fall onto a surface — they
    # are placed by hand or driven on. A 1.5 m drop produced ~5 m/s
    # impact velocity (Supervisor observer measurement: peak_speed_m_s
    # = 5.023 across all 4 huskies in the first 2-second window),
    # which dwarfs the robot's 1 m/s operational max and is the
    # source of the "crazy speeds" complaint. Set to 0.1 m so the
    # wheels settle onto the terrain mesh without intersecting it
    # (impact velocity = sqrt(2*9.81*0.1) ≈ 1.4 m/s, briefly, then
    # gravity-settle to zero — within operational range).
    "husky_drop_height": 0.1,
    # Layout of the husky fleet. ``circle`` = evenly spaced on
    # ``husky_spawn_radius`` around the clearing centre. ``corners`` =
    # one husky per map corner (plus the centre on a 5-count) so they
    # start far apart and drive inward when the controller picks
    # forward velocity.
    "husky_formation": "circle",
    "husky_spawn_radius": 12.0,
    # When formation == "corners", how far each spawn is inset from
    # the world edge. Determines the "safe corner" radius too — the
    # corner clearing pad has this radius so the husky lands flat.
    "husky_corner_margin": 12.0,
    # Distance from the world edge that triggers the controller's
    # boundary repulsion (huskies steer inward when this close to any
    # edge). Forwarded to each husky's customData.
    "husky_boundary_margin": 8.0,
    # Controller name Webots looks for in
    # ``<WEBOTS_HOME>/projects/samples/demos/controllers/<name>/``.
    # Default ``husky_random`` drives a random walk across the scene;
    # set to ``"<none>"`` if you want stationary huskies.
    "husky_controller": "husky_random",
    # Override if the world file lives somewhere other than
    # ``distribution/generated_worlds/``; must be a path Webots can
    # resolve relative to the output .wbt.
    "husky_urdf": None,
}


# Husky URDF path relative to the generated .wbt file when the world
# lives under ``distribution/generated_worlds/``. The URL is resolved
# by Webots at load time, so it must be correct for the output's
# directory layout. Users writing worlds to a different directory
# should pass ``husky_urdf`` to override.
_HUSKY_URDF_DEFAULT = "../../projects/robots/clearpath/husky_description/urdf/husky.urdf"


# Maximum number of huskies a single Mars run will place. Capped at
# 8 because beyond that the per-husky physics + controller load makes
# realtime rendering struggle on a typical desktop.
_HUSKY_MAX = 8


def _husky_circle_positions(count: int, radius_m: float):
    """Equally-spaced positions on a circle around (0, 0), each facing
    radially outward.

    Returns ``[(name, (dx, dy), yaw_rad), ...]`` where ``(dx, dy)`` is
    the offset from the clearing centre and ``yaw_rad`` is the
    rotation around +Z so the Husky's +X axis points outward — this
    way each husky immediately drives away from its neighbours when
    the random-walk controller picks any positive forward velocity.
    """
    if count <= 0:
        return []
    out = []
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        dx = radius_m * math.cos(angle)
        dy = radius_m * math.sin(angle)
        out.append((f"husky_{i}", (dx, dy), angle))
    return out


def _husky_corner_positions(count: int, world_size: float, margin: float):
    """Place huskies at the inset corners of the world, each facing
    inward toward the world centre.

    Up to 5: SW, SE, NW, NE, then centre. Counts above 5 fall back to
    the circle formation (caller's responsibility).
    """
    if count <= 0:
        return []
    half = world_size / 2.0
    inset = half - margin
    diag_yaw = math.pi / 4.0
    # Each corner faces the world centre. Yaw = atan2(-dy, -dx) since
    # we want the +x body axis to point at (-dx, -dy) -> centre.
    raw = [
        ("husky_sw",     (-inset, -inset), math.atan2( inset,  inset)),  # face NE
        ("husky_se",     ( inset, -inset), math.atan2( inset, -inset)),  # face NW
        ("husky_nw",     (-inset,  inset), math.atan2(-inset,  inset)),  # face SE
        ("husky_ne",     ( inset,  inset), math.atan2(-inset, -inset)),  # face SW
        ("husky_centre", (   0.0,    0.0), 0.0),
    ]
    # Reorder so 4 huskies populate the four corners cleanly, and the
    # 5th lands at centre.
    return raw[:min(count, len(raw))]


def _square_polygon(size: float):
    return ((0.0, 0.0), (size, 0.0), (size, size), (0.0, size))


def _disk_polygon(centre, radius, segments=24):
    cx, cy = centre
    return tuple(
        (
            cx + radius * math.cos(2 * math.pi * i / segments),
            cy + radius * math.sin(2 * math.pi * i / segments),
        )
        for i in range(segments)
    )


def _build_layout(params: Mapping[str, Any]) -> Layout:
    size = float(params["size"])
    centre = (size / 2.0, size / 2.0)

    layout = Layout(
        world_size=(size, size),
        terrain_params={
            "kind": "ridged_plus_fbm",
            "mix": float(params["mesa_mix"]),
            "grid_resolution": int(params["grid_resolution"]),
            "height_scale": float(params["height_scale"]),
            "octaves": int(params["octaves"]),
            "gain": float(params["gain"]),
            "lacunarity": float(params["lacunarity"]),
        },
    )
    layout.add_zone(Zone(
        name="surface",
        polygon=_square_polygon(size),
        tags=frozenset({"natural", "scatter_host", "arid", "mars"}),
    ))
    layout.add_zone(Zone(
        name="spawn_clearing",
        polygon=_disk_polygon(centre, float(params["clearing_radius"])),
        priority=10,
        exclusive=True,
        tags=frozenset({"spawn"}),
    ))
    layout.add_stamp(Stamp(
        name="clearing_pad",
        zone="spawn_clearing",
        mode="replace",
        height=float(params["height_scale"]) * 0.28,
    ))

    # Corner spawn pads — only contribute when formation == "corners"
    # AND husky_count > 0. Each pad is a small disk around the corner
    # spawn position, levelled to the same height as the centre pad
    # so a husky landing there doesn't slide off uneven terrain.
    husky_count = int(params.get("husky_count", 0))
    formation = str(params.get("husky_formation", "circle"))
    if husky_count > 0 and formation == "corners":
        margin = float(params["husky_corner_margin"])
        pad_radius = max(margin * 0.7, 5.0)  # generous pad
        for name, (dx, dy), _yaw in _husky_corner_positions(
            husky_count, size, margin
        ):
            zone_name = f"corner_pad_{name}"
            layout.add_zone(Zone(
                name=zone_name,
                polygon=_disk_polygon(
                    (centre[0] + dx, centre[1] + dy), pad_radius,
                ),
                priority=10,
                exclusive=True,
                tags=frozenset({"spawn"}),
            ))
            layout.add_stamp(Stamp(
                name=f"corner_stamp_{name}",
                zone=zone_name,
                mode="replace",
                height=float(params["height_scale"]) * 0.28,
            ))
        # Operating zone: a large central rectangle stamped flat at
        # the same height as the corner pads, so a husky driving off
        # a corner pad hits no cliff against the bumpy terrain
        # underneath. Validated offline: without this stamp huskies
        # wedge at the disk-edge of the corner pads where the
        # heightmap drop reaches ~1 m.
        op_inset = max(margin + 2.0, 8.0)
        layout.add_zone(Zone(
            name="operating_zone",
            polygon=(
                (op_inset,        op_inset),
                (size - op_inset, op_inset),
                (size - op_inset, size - op_inset),
                (op_inset,        size - op_inset),
            ),
            priority=8,
            exclusive=True,
            tags=frozenset({"clear"}),
        ))
        layout.add_stamp(Stamp(
            name="operating_zone_pad",
            zone="operating_zone",
            mode="replace",
            height=float(params["height_scale"]) * 0.28,
        ))

        # Boundary walls. Validated offline: without these, a random-
        # walking husky drove past x=80 and the supervisor logged its
        # z plummeting in physics free-fall. Four border bands at
        # priority 12 (above corner pads at 10 and operating_zone at 8)
        # stamped to a tall fixed height = uncrossable.
        wall_band_m = 3.0
        wall_height_m = 4.0
        s = size
        for wname, poly in (
            ("wall_s", ((0.0, 0.0),              (s, 0.0),              (s, wall_band_m),       (0.0, wall_band_m))),
            ("wall_n", ((0.0, s - wall_band_m),  (s, s - wall_band_m),  (s, s),                 (0.0, s))),
            ("wall_w", ((0.0, 0.0),              (wall_band_m, 0.0),    (wall_band_m, s),       (0.0, s))),
            ("wall_e", ((s - wall_band_m, 0.0),  (s, 0.0),              (s, s),                 (s - wall_band_m, s))),
        ):
            layout.add_zone(Zone(
                name=wname,
                polygon=poly,
                priority=12,
                exclusive=True,
                tags=frozenset({"wall"}),
            ))
            layout.add_stamp(Stamp(
                name=f"{wname}_stamp",
                zone=wname,
                mode="replace",
                height=wall_height_m,
            ))

    # Three rock layers, all Poisson-scattered inside the surface zone.
    # The solver gives each layer its own RNG (derive_seed is keyed on
    # group name), so increasing one group's count does not perturb
    # the others.
    layout.add_prop_group(_PropGroup(
        name="mars_debris",
        proto_weights=(("Rock", 1.0),),
        scatter="poisson",
        include_zones=("surface",),
        params=(
            ("count", int(params["debris_count"])),
            ("min_dist", float(params["debris_min_dist"])),
        ),
    ))
    layout.add_prop_group(_PropGroup(
        name="mars_rocks",
        proto_weights=(("Rock", 1.0),),
        scatter="poisson",
        include_zones=("surface",),
        params=(
            ("count", int(params["rock_count"])),
            ("min_dist", float(params["rock_min_dist"])),
        ),
    ))
    layout.add_prop_group(_PropGroup(
        name="mars_boulders",
        proto_weights=(("Rock", 1.0),),
        scatter="poisson",
        include_zones=("surface",),
        params=(
            ("count", int(params["boulder_count"])),
            ("min_dist", float(params["boulder_min_dist"])),
        ),
    ))

    husky_count = int(params.get("husky_count", 0))
    if husky_count > 0 and params.get("spawn_urdf") is not None:
        raise ValueError(
            "mars: husky_count and spawn_urdf are mutually exclusive "
            "(set one or the other, not both)"
        )
    if husky_count < 0 or husky_count > _HUSKY_MAX:
        raise ValueError(
            f"mars: husky_count must be in [0, {_HUSKY_MAX}]; got {husky_count}"
        )

    # Clearing pad is at z = height_scale * 0.28 in world space (the
    # stamp mode=replace above). Robots spawn relative to that so
    # they drop onto the pad, not through it.
    clearing_z = float(params["height_scale"]) * 0.28

    # Husky spawns are added in ``build()`` after crater positions are
    # known, since each husky's customData carries the crater list.
    if husky_count == 0 and params["spawn_urdf"] is not None:
        layout.add_spawn(Spawn(
            name="rover",
            translation=(
                centre[0], centre[1],
                clearing_z + float(params["spawn_height"]),
            ),
            rotation=(0.0, 0.0, 1.0, 0.0),
            urdf_url=str(params["spawn_urdf"]),
            controller=(
                str(params["spawn_controller"])
                if params["spawn_controller"] is not None
                else None
            ),
        ))
    return layout


def _fmt_color(c):
    return " ".join(format(float(x), ".4g") for x in c)


_LAYER_PARAMS = {
    # group -> (scale_min_key, scale_max_key, mesh_subdiv_key, displacement_key,
    #           template_count_key, footprint_radius_m, sink_offset_m, base_size_m)
    "mars_debris":   ("debris_scale_min",  "debris_scale_max",
                      "debris_mesh_subdivisions",  "debris_displacement",
                      "debris_template_count",
                      0.15, -0.03, 0.18),
    "mars_rocks":    ("rock_scale_min",    "rock_scale_max",
                      "rock_mesh_subdivisions",    "rock_displacement",
                      "rock_template_count",
                      0.30, -0.05, 0.35),
    "mars_boulders": ("boulder_scale_min", "boulder_scale_max",
                      "boulder_mesh_subdivisions", "boulder_displacement",
                      "boulder_template_count",
                      0.55, -0.08, 0.70),
}


def _stamp_craters(
    hm: Heightmap, size: float, params: Mapping[str, Any], seed: int,
    extra_exclusions: list[tuple[float, float, float]] | None = None,
) -> tuple[Heightmap, list[tuple[float, float, float]]]:
    """Return ``(heightmap, crater_centres_world)``.

    ``extra_exclusions`` is an optional list of
    ``(world_x_m, world_y_m, radius_m)`` zones the crater must stay
    out of in addition to the central one. Used to keep corner spawn
    pads safe.
    """
    count = int(params["crater_count"])
    if count <= 0:
        return hm, []

    rng = rng_from_seed(derive_seed(seed, "mars.craters"))
    pitch = size / hm.width
    rmin = float(params["crater_min_radius_m"]) / pitch
    rmax = float(params["crater_max_radius_m"]) / pitch
    dmin = float(params["crater_min_depth_m"])
    dmax = float(params["crater_max_depth_m"])
    exclusion_radius_cells = (
        float(params["crater_exclusion_radius_m"]) / pitch
    )
    centre_i = hm.width / 2.0
    centre_j = hm.height / 2.0

    # Pre-convert extra exclusions to grid coordinates.
    extra_zones_cells: list[tuple[float, float, float]] = []
    if extra_exclusions:
        for ex_x, ex_y, ex_r in extra_exclusions:
            extra_zones_cells.append((
                ex_x / pitch, ex_y / pitch, ex_r / pitch,
            ))

    out = hm
    attempts = count * 8
    placed = 0
    placed_centres: list[tuple[float, float, float]] = []
    while placed < count and attempts > 0:
        attempts -= 1
        r = rng.uniform(rmin, rmax)
        margin = r + 2.0
        i = rng.uniform(margin, hm.width - margin)
        j = rng.uniform(margin, hm.height - margin)
        # Skip if too close to the central spawn area.
        dx, dy = i - centre_i, j - centre_j
        if math.sqrt(dx * dx + dy * dy) < exclusion_radius_cells + r:
            continue
        # Skip if too close to any per-spawn pad (corner formation).
        bad = False
        for ex_i, ex_j, ex_r in extra_zones_cells:
            ddx, ddy = i - ex_i, j - ex_j
            if math.sqrt(ddx * ddx + ddy * ddy) < ex_r + r + 2.0:
                bad = True
                break
        if bad:
            continue
        depth = rng.uniform(dmin, dmax)
        pattern = crater_pattern(
            radius=int(math.ceil(r)),
            depth=depth,
            rim_height=depth * rng.uniform(0.25, 0.35),
            rim_width=rng.uniform(0.45, 0.60),
        )
        x0 = int(i) - pattern.width // 2
        y0 = int(j) - pattern.height // 2
        out = heightmap_stamp(out, pattern, x0, y0, mode="add")
        placed_centres.append((i * pitch, j * pitch, r * pitch))
        placed += 1
    return out, placed_centres


@dataclass
class Mars:
    name: str = "mars"

    def default_params(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for k, v in DEFAULT_PARAMS.items():
            if isinstance(v, tuple):
                out[k] = list(v)
            else:
                out[k] = v
        return out

    def build(self, seed: int, params: Mapping[str, Any]) -> WorldDescription:
        merged = self.default_params()
        for key, value in params.items():
            if key not in merged:
                raise ValueError(
                    f"unknown param for recipe {self.name!r}: {key!r}; "
                    f"known: {sorted(merged)}"
                )
            merged[key] = value

        # Apply husky-friendly defaults *before* the layout is built,
        # otherwise the original high rock counts get baked into the
        # PropGroups before we reduce them.
        #
        # Validated offline via the observer log: with the dramatic
        # default height_scale=5.5 m relief, the corner spawn pads
        # form ~3-4 m cliffs against the surrounding fbm+ridged
        # heightmap, and a husky driving off the pad edge tips over
        # or wedges. Cutting height_scale to 1.5 m (still visibly
        # rolling, just gentler) plus a sparser rock scatter lets the
        # huskies actually navigate.
        husky_count_now = int(merged.get("husky_count", 0))
        formation_now = str(merged.get("husky_formation", "circle"))
        if formation_now == "corners" and husky_count_now > 0:
            if "crater_count" not in params:
                merged["crater_count"] = 0
            for k, default in (
                ("debris_count", 20),
                ("rock_count", 8),
                ("boulder_count", 2),
                ("height_scale", 1.5),
            ):
                if k not in params:
                    merged[k] = default

        layout = _build_layout(merged)
        solved = solve(layout, seed=seed)
        hm = solved.heightmap
        assert hm is not None

        # Compute the spawn pad locations *before* stamping craters
        # so the crater placement skips any zone a husky will land on.
        size = float(merged["size"])
        husky_count = int(merged.get("husky_count", 0))
        formation = str(merged.get("husky_formation", "circle"))
        spawn_pad_exclusions: list[tuple[float, float, float]] = []
        if husky_count > 0:
            centre_xy_local = (size / 2.0, size / 2.0)
            if formation == "corners":
                margin = float(merged["husky_corner_margin"])
                pad_radius = max(margin * 0.7, 5.0)
                positions = _husky_corner_positions(
                    husky_count, size, margin,
                )
                for _, (dx, dy), _y in positions:
                    spawn_pad_exclusions.append((
                        centre_xy_local[0] + dx,
                        centre_xy_local[1] + dy,
                        pad_radius,
                    ))
        # Stamp craters with the per-spawn exclusion zones.
        hm, crater_centres = _stamp_craters(
            hm, size, merged, seed,
            extra_exclusions=spawn_pad_exclusions,
        )
        spacing = (size / hm.width, size / hm.height)
        height_scale_max = float(merged["height_scale"])

        world_age = float(merged["world_age"])
        base_rock_color = tuple(merged["rock_base_color"])
        hue_shift_max = float(merged["rock_hue_shift_deg"])
        max_tilt_rad = math.radians(float(merged["max_tilt_deg"]))

        # Per-layer indices so per-instance variation is stable even
        # if the user edits one layer's count.
        per_group_counters: dict[str, int] = {}
        elongation_max = float(merged["rock_elongation_max"])

        # Per-layer mesh templates. Each placed rock picks one by its
        # instance index modulo the layer's template count, so the
        # `vertices`/`face_indices` tuples match across instances and
        # WREN's StaticMesh cache (sipHash13c on coord/index data)
        # collapses them to a single GPU upload. Per-instance visual
        # variation comes from scale, rotation, ground-tilt, and hue
        # jitter — none of which require unique geometry.
        rock_templates: dict[str, list[Any]] = {}
        for group, layer_params in _LAYER_PARAMS.items():
            (_smin_k, _smax_k, mesh_subdiv_k, displacement_k,
             template_count_k, _fp, _sink, _bs) = layer_params
            n_templates = max(1, int(merged[template_count_k]))
            templates = []
            for t in range(n_templates):
                tpl_seed = derive_seed(seed, f"mars.template.{group}.{t}")
                templates.append(generate_rock(
                    tpl_seed,
                    subdivisions=int(merged[mesh_subdiv_k]),
                    displacement=float(merged[displacement_k]),
                    elongation_max=elongation_max,
                ))
            rock_templates[group] = templates

        solids: list[PlacedSolid] = []
        for placement in solved.props:
            group = placement.group
            idx = per_group_counters.get(group, 0)
            per_group_counters[group] = idx + 1

            (scale_min_k, scale_max_k,
             mesh_subdiv_k, displacement_k,
             template_count_k,
             footprint, sink, base_size) = _LAYER_PARAMS[group]
            scale_min = float(merged[scale_min_k])
            scale_max = float(merged[scale_max_k])

            variation = variation_for_instance(
                seed, group, idx,
                scale_min=scale_min,
                scale_max=scale_max,
                hue_shift_deg=hue_shift_max,
                scale_distribution=str(merged["rock_scale_distribution"]),
            )

            # Weathering grade: boulders weather more visibly than
            # debris because they have more surface area to darken.
            exposure = 0.9 if group != "mars_debris" else 0.7
            oxidation_base = 0.2 if group != "mars_boulders" else 0.3
            grade = compute_grade(
                seed, group, idx,
                world_age=world_age,
                exposure=exposure,
                oxidation_base=oxidation_base,
                moisture_base=0.0,
            )
            shifted = jitter_hue_rgb(base_rock_color, variation.hue_shift_deg)
            mult = grade_to_color_multiplier(grade)
            color = tuple(c * mult for c in shifted)

            # Ground-fit: sample terrain normal, tilt rotation to match.
            twist = placement.rotation[3]
            fit = ground_fit_on_heightmap(
                (placement.translation[0], placement.translation[1]),
                hm,
                origin=(0.0, 0.0),
                x_spacing=spacing[0],
                y_spacing=spacing[1],
                height_scale=1.0,
                footprint_radius=footprint * variation.scale,
                sample_count=6,
                sink_offset=sink * variation.scale,
                twist_rad=twist,
                max_tilt_rad=max_tilt_rad,
            )

            # Per-layer template lookup — each rock reuses one of N
            # template meshes (round-robin by instance index), so the
            # `vertices`/`face_indices` are byte-identical to siblings
            # at idx % N. WREN dedups them in StaticMesh::cCache, which
            # collapses per-frame draw calls in the ambient and per-light
            # passes when the renderer instances same-mesh runs.
            templates = rock_templates[group]
            mesh = templates[idx % len(templates)]

            world_scale = base_size * variation.scale

            solids.append(PlacedSolid(
                name=f"{group}_{idx}",
                translation=fit.translation,
                rotation=fit.rotation,
                # Skip stencil shadow casting on rocks/debris/boulders.
                # Shadow-volume rasterization scales with projected pixel
                # area, not mesh complexity, so even small props are
                # disproportionately expensive in the per-light pass —
                # disabling cast_shadows here cut mars_big perLight
                # significantly with imperceptible visual change at
                # typical viewing distances (their ground shadows were
                # tiny dots anyway).
                cast_shadows=False,
                vertices=mesh.vertices,
                face_indices=mesh.face_indices,
                diffuse_color=color,
                bounding_radius=mesh.bounding_radius,
                scale=world_scale,
            ))

        # Husky spawns are added here, after craters are placed, so
        # each one can carry the crater list as customData for the
        # controller's active avoidance behaviour.
        all_spawns: list[Spawn] = list(solved.spawns)
        if husky_count > 0:
            import json as _json
            centre_xy = (size / 2.0, size / 2.0)
            clearing_z = float(merged["height_scale"]) * 0.28
            drop = float(merged["husky_drop_height"])
            husky_url = merged.get("husky_urdf") or _HUSKY_URDF_DEFAULT
            husky_controller = str(merged.get("husky_controller") or "<none>")
            boundary_margin = float(merged["husky_boundary_margin"])
            crater_payload = _json.dumps(
                {
                    "world_centre": [float(centre_xy[0]), float(centre_xy[1])],
                    "world_size": float(size),
                    "boundary_margin": float(boundary_margin),
                    "craters": [
                        [float(cx), float(cy), float(cr)]
                        for cx, cy, cr in crater_centres
                    ],
                },
                separators=(",", ":"),
            )

            # Pick formation. ``corners`` for up to 5 huskies; anything
            # larger automatically falls back to the circle formation
            # because there are only 5 corner+centre slots.
            if formation == "corners" and husky_count <= 5:
                positions = _husky_corner_positions(
                    husky_count,
                    size,
                    float(merged["husky_corner_margin"]),
                )
            else:
                spawn_radius = float(merged["husky_spawn_radius"])
                positions = _husky_circle_positions(
                    husky_count, spawn_radius,
                )

            for name, (dx, dy), yaw in positions:
                all_spawns.append(Spawn(
                    name=name,
                    translation=(
                        centre_xy[0] + dx,
                        centre_xy[1] + dy,
                        clearing_z + drop,
                    ),
                    rotation=(0.0, 0.0, 1.0, yaw),
                    urdf_url=str(husky_url),
                    controller=husky_controller,
                    supervisor=True,
                    custom_data=crater_payload,
                ))

            # World observer: a plain Supervisor Robot that watches
            # husky positions and writes them to a log file. Useful
            # for offline verification of motion/avoidance. The
            # ``supervisor`` + ``customData`` fields work on plain
            # Robot nodes (only URDFRobot strips them).
            all_spawns.append(Spawn(
                name="mars_observer",
                translation=(centre_xy[0], centre_xy[1], 0.0),
                rotation=(0.0, 0.0, 1.0, 0.0),
                urdf_url=None,
                controller="mars_observer",
                supervisor=True,
            ))

        return WorldDescription(
            title="mars (omniworld)",
            heightmap=hm,
            heightmap_spacing=spacing,
            heightmap_origin=(0.0, 0.0),
            terrain_color=tuple(merged["terrain_color"]),
            sky_color=tuple(merged["sky_color"]),
            sky_texture=(
                str(merged["sky_texture"])
                if merged.get("sky_texture") else None
            ),
            sky_atmosphere=(
                str(merged["sky_atmosphere"])
                if merged.get("sky_atmosphere") else None
            ),
            sun_color=tuple(merged["sun_color"]),
            sun_direction=tuple(merged["sun_direction"]),
            sun_intensity=float(merged["sun_intensity"]),
            sun_cast_shadows=bool(merged["sun_cast_shadows"]),
            fog_enabled=bool(merged["fog_enabled"]),
            fog_color=tuple(merged["fog_color"]) if merged.get("fog_color") else None,
            fog_visibility_range=float(merged["fog_visibility_range"]),
            fill_light_enabled=bool(merged["fill_light_enabled"]),
            fill_light_color=tuple(merged["fill_light_color"]),
            fill_light_direction=tuple(merged["fill_light_direction"]),
            fill_light_intensity=float(merged["fill_light_intensity"]),
            props=[],              # mars uses inline solids only
            solids=solids,
            spawns=all_spawns,
            metadata={
                "recipe": self.name,
                "prop_count": len(solids),
                "world_age": world_age,
                "crater_count": int(merged["crater_count"]),
                "debris_count": sum(1 for p in solved.props if p.group == "mars_debris"),
                "rock_count": sum(1 for p in solved.props if p.group == "mars_rocks"),
                "boulder_count": sum(1 for p in solved.props if p.group == "mars_boulders"),
            },
        )


register_recipe(Mars())
