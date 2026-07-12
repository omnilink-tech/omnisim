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

"""damage_profiles — robot-specific configuration for the damage system.

Phase 12 of docs/developer/damage-system-plan.md. The original Phase 1-9
implementation hardcoded Husky URDF link names, per-part HP, wheel
geometry, and a wheel-mesh URL. This module factors all of that into a
`DamageProfile` dataclass so any URDF robot can opt into the damage
system by registering a profile (or falling back to the generic
4-wheeler profile that works on any robot whose link names roughly
follow the Husky convention).

Profile selection priority (handled by `select_profile` below):
  1. Explicit `damage_profile` key in the robot's customData JSON.
  2. `OMNISIM_DAMAGE_PROFILE` environment variable.
  3. Auto-detect: scan the robot's URDF link names; pick HUSKY if a
     critical mass match, else GENERIC_4_WHEEL.
  4. Fallback: GENERIC_4_WHEEL.
"""

from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass, field


@dataclass(frozen=True)
class DamageProfile:
    """Per-robot configuration for the damage tracker. All fields except
    `name` and `part_table` have sensible defaults so a partial profile
    still works.
    """

    name: str

    # URDF link name -> logical part. Multiple URDF links may map to the
    # same logical part. Anything not in this table maps to `chassis`.
    part_table: dict[str, str] = field(default_factory=dict)

    # Which logical-part labels are wheels (used by Phase 5 wheel-torque
    # gate, Phase 9 detachment, and the contact-point Z reattribution).
    wheel_parts: frozenset = field(default_factory=lambda: frozenset({
        "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"
    }))

    # Per-part HP and impulse thresholds. Missing keys fall back to the
    # *_default fields below.
    part_hp_max: dict[str, float] = field(default_factory=dict)
    part_hp_threshold_J: dict[str, float] = field(default_factory=dict)
    buffer_threshold_J: dict[str, float] = field(default_factory=dict)
    log_threshold_J: dict[str, float] = field(default_factory=dict)
    decal_threshold_J: dict[str, float] = field(default_factory=dict)

    default_hp_max: float = 200.0
    default_hp_threshold_J: float = 1.0
    default_buffer_threshold_J: float = 0.1
    default_log_threshold_J: float = 1.0
    default_decal_threshold_J: float = 3.0

    # Synthetic chassis impact detector (Phase 2): trigger when the
    # URDFRobot body's velocity changes by more than this in one step.
    synthetic_chassis_dv_threshold: float = 0.1
    # Phase 7 fallback chassis mass when the URDFRobot's Physics field
    # doesn't expose one.
    chassis_mass_default: float = 46.0

    # Phase 2 contact-point Z reattribution: contacts above this world
    # Z that node-id-attribute to a wheel get reattributed to the
    # closest non-wheel part. Husky wheel top is at world z ~0.33.
    reattribute_z_threshold: float = 0.4

    # Phase 9 — wheel detachment geometry. Used when spawning the free
    # wheel Solid that replaces the URDF wheel.
    wheel_height: float = 0.1143
    wheel_radius: float = 0.1651
    wheel_mass: float = 2.637
    # If set, the spawned wheel uses this Mesh URL for visuals (matches
    # the still-attached wheels). If None, the spawned wheel falls back
    # to a plain dark Cylinder Shape.
    wheel_mesh_url: str | None = None

    # Phase 19 — generalised part detachment. Logical parts in this
    # frozenset physically detach from the robot when they transition
    # to `broken`. Wheels in this set use the Phase 9 cylinder path;
    # other parts (bumpers, top_plate, arm links) detach as Box-shaped
    # slabs sized from `crumple_size`. If empty, `wheel_parts` is used
    # as the fallback so existing profiles keep working unchanged.
    detachable_parts: frozenset = field(default_factory=frozenset)
    # Per-part mass (kg) for the spawned slab Solid. Wheels keep
    # `wheel_mass`. Missing entries fall back to `default_detach_mass`.
    detach_mass: dict[str, float] = field(default_factory=dict)
    default_detach_mass: float = 3.0
    # Per-part Mesh URL for the spawned detached body. When set the
    # detach path renders the free body with that mesh + an explicit
    # PBRAppearance baseColor (from `detach_color`); without it, the
    # detach path falls back to a plain Box of `crumple_size`, which
    # reads as a foreign object rather than the part that just fell
    # off. Recommend matching the URDF link's visual mesh.
    detach_mesh_url: dict[str, str] = field(default_factory=dict)
    # Per-part baseColor [r, g, b] for the spawned detached body's
    # PBRAppearance. Matters because Webots's VRML importer doesn't
    # carry .dae-baked materials onto stanza-built Shapes -- the colour
    # has to be set explicitly or the spawned body renders white. Falls
    # back to a neutral dark tone if missing.
    detach_color: dict[str, tuple[float, float, float]] = field(default_factory=dict)

    # Phase 20 — chassis-to-slab reattribution. URDFRobot consolidates
    # chassis-area contacts onto the leaf rigid bodies; the existing
    # Phase 2 reattribution rerouted high-Z wheel contacts to the
    # nearest non-wheel part by horizontal distance. That doesn't
    # cover the case where a contact attributed (correctly, at the
    # rigid-body level) to `chassis` actually sits inside a slab's
    # geometry (top of top_plate, tip of front_bumper). Parts listed
    # here get a second attribution pass: any chassis-attributed
    # contact whose world position falls inside the slab's
    # axis-aligned bbox (centre from getPosition, extent from
    # `crumple_size`) is rebadged to that slab so HP accumulates on
    # the part actually being struck. Empty by default — robots
    # without slab-shaped accessories keep current behaviour.
    slab_attribute_parts: frozenset = field(default_factory=frozenset)

    # Phase 21 — crash impulse multiplier. When the synthetic-chassis
    # detector fires AND at least one contact above ground (z > 0.05)
    # is present, the impact came from outside the husky (something
    # hit the chassis or another vehicle ran into it) rather than
    # wheel-on-floor settling. ODE rigid-body simulation under-reports
    # the impulse-J·s in such impacts because it doesn't model crumple
    # energy absorption — the bodies bounce or briefly stick instead
    # of plastically absorbing the kinetic energy. Multiplying the
    # synthetic impulse by `crash_impulse_multiplier` when a real
    # external contact is visible compensates for that, so head-on
    # vehicle collisions read like a car crash instead of a tap.
    # 1.0 = behave as before (no boost). 5-10 typical for cars.
    crash_impulse_multiplier: float = 1.0

    # Phase 10 — per-state mesh variants. Maps logical_part -> state ->
    # mesh URL. On state transition the supervisor walks the part's
    # Shape nodes and rewrites their `geometry Mesh { url ... }` to the
    # state-specific variant, so e.g. the chassis swaps from a clean
    # base_link.dae to a dented version when state hits 'damaged'.
    # Default empty: when there's no variant for a (part, state) the
    # mechanism no-ops and Phase 7 appearance darkening carries the
    # visual story alone. Asset authoring (the actual damaged .dae
    # files) is a separate deliverable.
    mesh_variants: dict[str, dict[str, str]] = field(default_factory=dict)

    # Phase 14 — procedural deformation. Parts in this set have their
    # geometry replaced at runtime with a crumpled IndexedFaceSet
    # (intensity scaling with damage state) when no static
    # `mesh_variants` entry exists for the (part, state). Wheels and
    # similar geometry-critical parts should NOT be in this set — they
    # need their original cylindrical shape preserved.
    procedural_deformation_parts: frozenset = field(default_factory=frozenset)
    # Per-part rough bounding box for the procedural crumpler. The
    # generator emits a deformed *box* of these dimensions in the
    # part's local frame, replacing whatever the URDF imported. Tune
    # these to match the visual silhouette of the original .dae mesh.
    crumple_size: dict[str, tuple[float, float, float]] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Built-in profiles
# ---------------------------------------------------------------------------

HUSKY = DamageProfile(
    name="husky",
    part_table={
        "base_link": "chassis",
        "top_chassis_link": "chassis",
        "inertial_link": "chassis",
        "imu_link": "chassis",
        "front_left_wheel_link": "wheel_fl",
        "front_right_wheel_link": "wheel_fr",
        "rear_left_wheel_link": "wheel_rl",
        "rear_right_wheel_link": "wheel_rr",
        "front_bumper_link": "front_bumper",
        "rear_bumper_link": "rear_bumper",
        "top_plate_link": "top_plate",
        "top_plate_front_link": "top_plate",
        "top_plate_rear_link": "top_plate",
        "user_rail_link": "top_plate",
    },
    # Physical damage thresholds, all in J of plastic deformation work.
    # Calibrated against the synthetic detector now using ½·m·Δv² (so
    # a 2 m/s closing-Δv at chassis mass 46 produces ~92 J per side).
    part_hp_max={
        # Chassis is the bottleneck. It's the part that fails last,
        # and once it's "broken" we mark the robot disabled so the
        # other side wins. Five-ish 92 J impacts get it there.
        "chassis": 500.0,
        # Steel bumpers crumple at ~50 J onset, fail ~250 J. At our
        # combat energies the front bumper falls off after 2-3 hits.
        "front_bumper": 200.0,
        "rear_bumper": 200.0,
        # Top plate is on top of the chassis, well out of the head-on
        # impact path. It only takes damage from things landing on it
        # (debris, cubes). HP this high means it stays intact through
        # routine combat — realistic.
        "top_plate": 600.0,
        # Wheels are outboard. Head-on bumper-to-bumper impacts don't
        # contact them; they only see wheel-to-floor and (rarely)
        # wheel-to-wheel grazes. HP this high means they essentially
        # never detach in head-on combat — realistic.
        "wheel_fl": 300.0,
        "wheel_fr": 300.0,
        "wheel_rl": 300.0,
        "wheel_rr": 300.0,
    },
    part_hp_threshold_J={
        "chassis": 0.5,
        "front_bumper": 1.0,
        "rear_bumper": 1.0,
        "top_plate": 0.5,
        "wheel_fl": 3.0,
        "wheel_fr": 3.0,
        "wheel_rl": 3.0,
        "wheel_rr": 3.0,
    },
    buffer_threshold_J={
        "chassis": 0.1,
        "front_bumper": 0.1,
        "rear_bumper": 0.1,
        "top_plate": 0.05,
        "wheel_fl": 0.5,
        "wheel_fr": 0.5,
        "wheel_rl": 0.5,
        "wheel_rr": 0.5,
    },
    log_threshold_J={
        "chassis": 1.0,
        "front_bumper": 1.0,
        "rear_bumper": 1.0,
        "top_plate": 0.5,
        "wheel_fl": 5.0,
        "wheel_fr": 5.0,
        "wheel_rl": 5.0,
        "wheel_rr": 5.0,
    },
    decal_threshold_J={
        "chassis": 2.0,
        "front_bumper": 5.0,
        "rear_bumper": 5.0,
        "top_plate": 1.5,
        "wheel_fl": 8.0,
        "wheel_fr": 8.0,
        "wheel_rl": 8.0,
        "wheel_rr": 8.0,
    },
    # Real rams produce chassis Δv ≥ 2 m/s. Routine traction and
    # turning rarely exceed 1 m/s. 2.0 m/s is the floor for "this is
    # an actual collision, not driving micro-jitter".
    synthetic_chassis_dv_threshold=2.0,
    chassis_mass_default=46.0,
    reattribute_z_threshold=0.4,
    wheel_height=0.1143,
    wheel_radius=0.1651,
    wheel_mass=2.637,
    wheel_mesh_url=(
        "omnisim://projects/robots/clearpath/husky_description/meshes/wheel.dae"
    ),
    # mesh_variants intentionally empty — Phase 10 mechanism is in
    # place but pre-authored damaged Husky meshes don't ship with this
    # repo yet. Populate this dict with state -> .dae URL mappings as
    # damaged assets land:
    #   "chassis": {"scuffed": "...", "damaged": "...", "broken": "..."},
    # Without entries Phase 14's procedural deformation kicks in for
    # the parts listed below.
    procedural_deformation_parts=frozenset({
        "chassis", "front_bumper", "rear_bumper", "top_plate",
    }),
    # Rough bounding boxes per logical part, used by the procedural
    # crumpler to size the generated IndexedFaceSet. Approximates the
    # silhouette of the original URDF .dae meshes.
    crumple_size={
        "chassis":      (0.99, 0.57, 0.32),
        "front_bumper": (0.10, 0.50, 0.18),
        "rear_bumper":  (0.10, 0.50, 0.18),
        "top_plate":    (0.66, 0.45, 0.05),
    },
    # Phase 19 — wheels keep the Phase 9 cylinder path; bumpers and the
    # top plate fall off as slabs when they break.
    detachable_parts=frozenset({
        "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr",
        "front_bumper", "rear_bumper", "top_plate",
    }),
    detach_mass={
        "front_bumper": 4.0,
        "rear_bumper":  4.0,
        "top_plate":    2.5,
    },
    # Per-part .dae visuals so the detached free body matches what was
    # just on the chassis. Without these the slab path renders a plain
    # dark Box, which looks foreign. baseColor matches each .dae's
    # baked diffuse exactly (Webots does NOT carry .dae materials
    # through importMFNodeFromString, so we have to recreate them);
    # otherwise the part visibly changes colour when it detaches.
    detach_mesh_url={
        "front_bumper": "omnisim://projects/robots/clearpath/husky_description/meshes/bumper.dae",
        "rear_bumper":  "omnisim://projects/robots/clearpath/husky_description/meshes/bumper.dae",
        "top_plate":    "omnisim://projects/robots/clearpath/husky_description/meshes/top_plate.dae",
    },
    detach_color={
        # bumper.dae diffuse = 0.1 0.1 0.1
        "front_bumper": (0.10, 0.10, 0.10),
        "rear_bumper":  (0.10, 0.10, 0.10),
        # top_plate.dae diffuse = 0.08 0.08 0.08
        "top_plate":    (0.08, 0.08, 0.08),
    },
    # NOTE: these detach_color values are now only a *fallback*. The
    # detach paths clone each part's live appearance (the colour the URDF
    # importer actually assigned, e.g. the wheel's 0.65 0.55 0.5 grey)
    # via _live_appearance_field, so the free body matches the robot
    # regardless of what's listed here. The colours above are used only
    # if that live read fails.
    # Phase 20 — slab reattribution. URDFRobot consolidates contacts on
    # top_plate / bumpers onto the chassis rigid body; without this
    # pass, top_plate gets 0 HP loss from cubes landing on it and
    # bumpers get 0 HP loss from head-on collisions. Reattribution
    # routes chassis-bucket contacts that physically fall inside a
    # slab's bbox to the slab.
    slab_attribute_parts=frozenset({
        "top_plate", "front_bumper", "rear_bumper",
    }),
    # Phase 21 — crash multiplier. ODE rigid-body collisions transfer
    # only a fraction of kinetic energy into measurable Δv on the
    # chassis body. We multiply every fired synthetic impulse by this
    # factor. With closing speed 13 m/s a 3x boost is plenty for the
    # bumper to break, and avoids the visual-spawn explosion (decals,
    # dent overlays, smoke) that 10x produces when each step's already
    # large impulse gets amplified — that explosion was crashing
    # Webots under sustained collision.
    # Multiplier deprecated by switching the synthetic detector to
    # use ½·m·Δv² (energy) instead of m·Δv (momentum). The energy
    # form scales correctly with Δv on its own; no artificial boost
    # needed. Left at 1.0 = identity for backward compat.
    crash_impulse_multiplier=1.0,
)

# Generic 4-wheeled mobile robot. Maps anything that LOOKS like a wheel
# (any link with "wheel" in its name) by suffix into the wheel_fl/fr/rl/rr
# logical parts; everything else collapses to chassis. Conservative HP
# and thresholds chosen so behaviour reads sensibly across a wide range
# of robot sizes; tune via a custom profile if your robot is unusually
# light or heavy.
GENERIC_4_WHEEL = DamageProfile(
    name="generic_4_wheel",
    part_table={},  # populated by auto-detect (see select_profile)
    part_hp_max={
        "chassis": 400.0,
        "wheel_fl": 200.0,
        "wheel_fr": 200.0,
        "wheel_rl": 200.0,
        "wheel_rr": 200.0,
    },
    part_hp_threshold_J={
        "chassis": 0.5,
        "wheel_fl": 3.0,
        "wheel_fr": 3.0,
        "wheel_rl": 3.0,
        "wheel_rr": 3.0,
    },
    buffer_threshold_J={
        "chassis": 0.1,
        "wheel_fl": 0.5,
        "wheel_fr": 0.5,
        "wheel_rl": 0.5,
        "wheel_rr": 0.5,
    },
    log_threshold_J={
        "chassis": 1.0,
        "wheel_fl": 5.0,
        "wheel_fr": 5.0,
        "wheel_rl": 5.0,
        "wheel_rr": 5.0,
    },
    decal_threshold_J={
        "chassis": 2.0,
        "wheel_fl": 8.0,
        "wheel_fr": 8.0,
        "wheel_rl": 8.0,
        "wheel_rr": 8.0,
    },
    # Conservative defaults; specific robots should override.
    wheel_height=0.10,
    wheel_radius=0.15,
    wheel_mass=2.0,
    wheel_mesh_url=None,  # falls back to a plain cylinder visual
)

# Stub profiles for non-mobile robots. Provided so a future drone or
# arm demo can plug in without per-robot scaffolding; the per-part
# tables are intentionally minimal here and tunable via custom profiles.
GENERIC_DRONE = DamageProfile(
    name="generic_drone",
    part_table={},
    wheel_parts=frozenset(),  # no wheels on a drone
    part_hp_max={"chassis": 200.0, "rotor": 50.0, "leg": 100.0},
    part_hp_threshold_J={"chassis": 0.5, "rotor": 0.3, "leg": 0.5},
    buffer_threshold_J={"chassis": 0.1, "rotor": 0.05, "leg": 0.1},
    log_threshold_J={"chassis": 1.0, "rotor": 0.5, "leg": 1.0},
    decal_threshold_J={"chassis": 1.5, "rotor": 0.5, "leg": 2.0},
    chassis_mass_default=2.0,
)

GENERIC_ARM = DamageProfile(
    name="generic_arm",
    part_table={},
    wheel_parts=frozenset(),
    part_hp_max={"chassis": 600.0, "joint": 150.0, "end_effector": 200.0},
    part_hp_threshold_J={"chassis": 1.0, "joint": 0.8, "end_effector": 1.5},
    buffer_threshold_J={"chassis": 0.2, "joint": 0.2, "end_effector": 0.3},
    log_threshold_J={"chassis": 2.0, "joint": 1.5, "end_effector": 3.0},
    decal_threshold_J={"chassis": 3.0, "joint": 2.0, "end_effector": 4.0},
    chassis_mass_default=20.0,
)


# Demo profile for the box-drop arena (husky_damage_arena.wbt). The stock
# HUSKY HP values are tuned for vehicle ramming combat, where the chassis
# rigid body picks up a real multi-m/s Δv on impact and the momentum-Δv
# impulse model reports large per-contact impulses. A box dropped on a
# *parked* husky barely moves the body, so those impulses come out near
# zero and nothing ever reaches `broken` — which is exactly why the arena
# demo showed an intact robot. This profile makes the arena husky fragile
# enough that falling weights visibly tear parts off:
#   - much lower per-part HP,
#   - lower per-part damage thresholds so sub-Joule contacts still bite,
#   - a far lower synthetic-chassis Δv gate so the small jolt a heavy box
#     imparts to a grounded husky registers as chassis damage.
# Detachment geometry/colours, crumple sizes, and part_table are all
# inherited unchanged from HUSKY via dataclasses.replace.
from dataclasses import replace as _replace

HUSKY_ARENA = _replace(
    HUSKY,
    name="husky_arena",
    part_hp_max={
        "chassis": 80.0,
        "front_bumper": 25.0,
        "rear_bumper": 25.0,
        "top_plate": 30.0,
        "wheel_fl": 45.0,
        "wheel_fr": 45.0,
        "wheel_rl": 45.0,
        "wheel_rr": 45.0,
    },
    part_hp_threshold_J={
        "chassis": 0.2,
        "front_bumper": 0.2,
        "rear_bumper": 0.2,
        "top_plate": 0.2,
        "wheel_fl": 0.5,
        "wheel_fr": 0.5,
        "wheel_rl": 0.5,
        "wheel_rr": 0.5,
    },
    # Grounded-husky box impacts only nudge the chassis body a few tenths
    # of a m/s. Drop the gate well below that so they count.
    synthetic_chassis_dv_threshold=0.4,
)


_PROFILE_REGISTRY: dict[str, DamageProfile] = {
    "husky": HUSKY,
    "husky_arena": HUSKY_ARENA,
    "generic_4_wheel": GENERIC_4_WHEEL,
    "generic_drone": GENERIC_DRONE,
    "generic_arm": GENERIC_ARM,
}


def get_profile(name: str) -> DamageProfile | None:
    return _PROFILE_REGISTRY.get(name.lower())


# ---------------------------------------------------------------------------
# Auto-detect / selection helpers
# ---------------------------------------------------------------------------


def _link_names_from_part_table(profile: DamageProfile) -> set[str]:
    return set(profile.part_table.keys())


def _autodetect_from_link_names(observed_links: set[str]) -> DamageProfile:
    """Pick a profile by intersecting observed URDF link names with each
    built-in profile's part_table. Highest match-count wins; ties go to
    the most specific (HUSKY > GENERIC). Falls back to GENERIC_4_WHEEL
    augmented with a derived part_table built from any "*_wheel_link"
    or "wheel_*" names we find.
    """
    best_match = 0
    best_profile = GENERIC_4_WHEEL
    for prof in (HUSKY,):  # only specific profiles compete here
        match = len(observed_links & _link_names_from_part_table(prof))
        if match > best_match:
            best_match = match
            best_profile = prof
    if best_profile is HUSKY:
        return HUSKY

    # Fall back to a generic profile, but build a part_table on the fly
    # from observed link names so the generic case isn't completely
    # nameless. Heuristic: links containing "front_left", "front_right",
    # etc. + "wheel" map to wheel_fl/fr/rl/rr; anything else -> chassis.
    derived: dict[str, str] = {}
    for link in observed_links:
        l = link.lower()
        if "wheel" not in l:
            continue
        if "front" in l and "left" in l:
            derived[link] = "wheel_fl"
        elif "front" in l and "right" in l:
            derived[link] = "wheel_fr"
        elif "rear" in l and "left" in l:
            derived[link] = "wheel_rl"
        elif "rear" in l and "right" in l:
            derived[link] = "wheel_rr"
    # Use dataclasses.replace to override part_table without losing
    # the rest of GENERIC_4_WHEEL's defaults.
    from dataclasses import replace
    return replace(GENERIC_4_WHEEL, part_table=derived)


def _parse_custom_data_profile(raw: str) -> str | None:
    """Extract the `damage_profile` key from a customData JSON blob.
    Returns None on missing/invalid JSON; that's expected and benign.
    """
    if not raw:
        return None
    try:
        data = json.loads(raw)
    except (json.JSONDecodeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    name = data.get("damage_profile")
    return name if isinstance(name, str) and name else None


def select_profile(custom_data_raw: str = "",
                   observed_link_names: set[str] | None = None,
                   default: DamageProfile = GENERIC_4_WHEEL) -> DamageProfile:
    """Resolve which profile to use for a given robot.

    Priority order, first match wins:
      1. `damage_profile` key in the robot's customData JSON.
      2. OMNISIM_DAMAGE_PROFILE environment variable.
      3. Auto-detect from observed URDF link names.
      4. `default` (GENERIC_4_WHEEL).
    """
    name = _parse_custom_data_profile(custom_data_raw)
    if name is not None:
        prof = get_profile(name)
        if prof is not None:
            return prof
        sys.stderr.write(
            f"[damage_profiles] customData damage_profile={name!r} not registered; "
            "falling through to next selector\n"
        )

    env_name = os.environ.get("OMNISIM_DAMAGE_PROFILE")
    if env_name:
        prof = get_profile(env_name)
        if prof is not None:
            return prof
        sys.stderr.write(
            f"[damage_profiles] OMNISIM_DAMAGE_PROFILE={env_name!r} not registered; "
            "falling through to auto-detect\n"
        )

    if observed_link_names:
        return _autodetect_from_link_names(observed_link_names)

    return default
