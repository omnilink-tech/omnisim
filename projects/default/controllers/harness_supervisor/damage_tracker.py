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

"""damage_tracker — physics-based damage system (Phases 1–2).

Discovers a named robot in the scene, walks its Solid subtree to map
URDF link names to logical parts (chassis, wheel_fl, top_plate, ...),
and each step:

1. Polls contact points on the URDFRobot root (Phase 1).
2. Computes per-contact impulse via momentum conservation on the Husky-
   side body's velocity delta over the step (Phase 1).
3. Reattributes high-Z wheel contacts to the closest non-wheel part
   to work around URDFRobot's leaf-body contact consolidation (Phase 2).
4. Accumulates damage against per-part HP using `hp -= max(0, J - τ)`
   and emits state-transition events when crossing band boundaries
   (Phase 2 — pristine → scuffed → damaged → broken).

Phase 3 exposes the resulting state snapshot and event ring buffer
over the harness wire protocol via the `damage_state` / `damage_events`
/ `damage_reset` commands; HTTP plumbing in scripts/harness comes next.

The Webots/ODE quirks this code works around — getContactPoints needs
a top-level Solid, ContactPoint.node_id is the this-side body, and
URDFRobot fuses linked bodies — are documented in
docs/developer/physics-contact-and-collision-complexity.md
("Controller API caveats" section).

See docs/developer/damage-system-plan.md.
"""

from __future__ import annotations

import collections
import json
import math
import os
import random
import socket
import struct
import sys
import time

from damage_profiles import DamageProfile, GENERIC_4_WHEEL, HUSKY, select_profile
from procedural_meshes import (
    apply_local_dent,
    apply_uniform_random_dents,
    build_neighbor_table,
    collapse_island_to_neighbors,
    find_fracture_islands,
    find_strained_vertices,
    fragment_ifs_stanza,
    generate_crumpled_box,
    make_baseline_box_buffer,
    relax_vertices,
    vertex_buffer_to_ifs_stanza,
)


# Per-robot configuration (URDF part table, HP/threshold tables, wheel
# geometry, etc.) lives in damage_profiles.py. The damage tracker reads
# its profile at startup; everything below this point is universal —
# state bands, visual styling, debris/marker config — and applies the
# same way regardless of which robot is being damaged.

# State bands as (label, hp_fraction_floor). Walked top-to-bottom — first
# band whose floor is <= current fraction wins. The "broken" floor is 0.0
# which catches anything below "damaged".
STATE_BANDS: list[tuple[str, float]] = [
    ("pristine", 0.80),
    ("scuffed", 0.40),
    ("damaged", 0.10),
    ("broken", 0.0),
]

# Phase 14 — procedural deformation intensity per state. Fed into the
# crumpled-box generator's `crumple` parameter. Pristine = perfect
# (clean) mesh; broken = heavily deformed silhouette.
CRUMPLE_PER_STATE: dict[str, float] = {
    "pristine": 0.00,
    "scuffed":  0.05,
    "damaged":  0.15,
    "broken":   0.30,
}

# Phase 15 — impact-localized mesh deformation.
DEFORM_THRESHOLD_J = 3.0     # impulse below this doesn't dent the mesh
# Translates impulse into a vertex-displacement magnitude, capped so a
# single big hit can't punch entirely through the body.
DEFORM_MAG_PER_J = 0.0050
DEFORM_MAG_CAP = 0.10
DEFORM_RADIUS = 0.30         # dent influence radius in part-local meters
REEMIT_INTERVAL_MS = 200     # min ms between geometry re-imports per part
DEFORM_SUBDIVISION = 4       # quads per face — must match procedural_meshes
# Phase 16b: spring-coupling fraction. After the radial-falloff dent
# pass, each directly-pushed vertex propagates `coupling × push` to its
# 4 grid neighbors. Higher = wider organic dimples; 0.0 = original
# Phase 15 hard radial falloff. 0.35 is a visually convincing default
# without inflating the dent magnitude unrealistically.
DEFORM_COUPLING = 0.35

# Phase 16d: per-step Laplacian smoothing. Each step, for parts whose
# vertex buffer has been dented within RELAX_AFTER_MS, run RELAX_ITERS
# iterations of vertex-toward-neighbor-centroid smoothing at RELAX_RATE.
# Smooths the high-frequency rim of fresh dents into rolled silhouettes
# without erasing them. Skipped when convergence dx < RELAX_CONVERGED_M
# so we don't waste cycles smoothing a stable mesh.
RELAX_RATE = 0.06
RELAX_ITERS = 2
RELAX_AFTER_MS = 1500          # only relax for this long after a dent
RELAX_CONVERGED_M = 0.0008     # stop relaxing once max delta < 0.8mm

# Phase 17 — topology fracture. A vertex displaced more than
# FRACTURE_STRAIN_M from its baseline position is "yielded" — past
# plastic-strain limits. Connected groups of yielded vertices of size
# >= FRACTURE_MIN_VERTICES become candidate fracture islands that
# tear off as free-body fragments (Phase 17c).
#
# Calibration: with Phase 16d relaxation + Phase 16b coupling the
# accumulated strain plateaus around 3-5cm in normal play because
# relaxation erodes peak dents back toward neighbor positions each
# step. Threshold set just above that plateau so fracture only fires
# after sustained heavy damage. Headless box_drop_extreme regression
# (heavier schedule, longer runtime) reliably crosses it.
FRACTURE_STRAIN_M = 0.045
FRACTURE_MIN_VERTICES = 4
# Hard cap on alive fragments per part. Prevents ODE collision-pair
# explosion in long-running scenarios (e.g. repair tests where boxes
# keep impacting throughout). Once the cap is hit, no new fragments
# spawn — vertices stay yielded but don't tear off.
FRAGMENTS_PER_PART_CAP = 8

# Phase 18 — repair mechanics. Heal rates are zero by default (no
# passive regen). Set per-part via damage_set_heal_rate or implicitly
# by placing the chassis inside a repair-station volume (Phase 18c).
DEFAULT_HEAL_RATE_HP_PER_S = 0.0
DEFAULT_HEAL_RATE_MESH_M_PER_S = 0.0
# Vertex buffer regen migrates each non-baseline vertex toward its
# baseline position at HEAL_RATE_MESH_M_PER_S. Capped per step so we
# don't overshoot baseline with a long dt step.
HEAL_MESH_OVERSHOOT_GUARD_M = 0.001

# Phase 4 — visual damage markers. Spawned at the contact point of each
# state transition; cumulative across runs and cleared on damage_reset.
# Colors and sizes are tuned for visibility from the default top-down
# arena viewpoint.
DAMAGE_MARKER_COLORS: dict[str, list[float]] = {
    "scuffed": [0.95, 0.80, 0.10],   # yellow
    "damaged": [0.95, 0.45, 0.05],   # orange
    "broken":  [0.95, 0.10, 0.05],   # red
}
# Markers are flash-style indicators, not persistent decoration. Phase 7
# darkens the chassis itself which carries the damage state visually,
# so the marker only needs to flash long enough to register the moment
# of transition (the diagnostic signal) before fading.
DAMAGE_MARKER_SIZES: dict[str, float] = {
    "scuffed": 0.06,
    "damaged": 0.10,
    "broken":  0.16,
}
DEFAULT_MARKER_COLOR = [0.85, 0.85, 0.85]
DEFAULT_MARKER_SIZE = 0.05
# How long markers stick around before being removed, in milliseconds
# of sim time. Long enough for an agent polling damage_events at ~1Hz
# to consistently see the transition; short enough that the scene
# isn't perpetually decorated with floating spheres.
MARKER_LIFETIME_MS = 4000

# Markers are placed ABOVE the contact point so they're not hidden inside
# the chassis or below the wheel. Tuned so the marker floats just above
# the Husky's roof in the default top-down viewpoint.
MARKER_Z_OFFSET = 0.6

# Phase 5 — behavioral consequences. Per-state wheel torque multipliers
# the supervisor writes into the URDFRobot's customData; a driving
# controller reads its own customData and scales each wheel's commanded
# velocity by these multipliers. Pristine/scuffed leave the wheel fully
# functional; damaged halves it; broken zeroes it.
WHEEL_TORQUE_SCALE: dict[str, float] = {
    "pristine": 1.0,
    "scuffed":  1.0,
    "damaged":  0.5,
    "broken":   0.0,
}

# Phase 6 — debris bursts. On every "X -> broken" transition, spawn a
# handful of physics-enabled chunks that fly off the impact site,
# tumble through the air, and rest. These are tracked alongside markers
# and cleaned up by damage_reset.
#
# Refined: chunks are non-uniform thin Box "plates" (one dim small,
# the other two larger) so they read as torn metal/panel fragments
# rather than uniform dropped cubes. Count tightened to 3-5 — fewer
# but more meaningful pieces.
DEBRIS_BURST_COUNT_RANGE = (3, 5)
DEBRIS_PLATE_LONG_RANGE = (0.10, 0.22)   # the two longer Box dimensions
DEBRIS_PLATE_THIN_RANGE = (0.012, 0.035) # the one thin dimension (panel thickness)
DEBRIS_MASS_RANGE = (0.20, 0.60)
DEBRIS_VELOCITY_RANGE = (2.5, 4.5)       # m/s outward magnitude
DEBRIS_VERTICAL_BIAS = (0.3, 0.9)        # multiplier on outward z component
# Charred / scorched palette. Slight variation between chunks so a
# single burst doesn't look like one repeated mesh.
DEBRIS_COLORS: list[list[float]] = [
    [0.10, 0.10, 0.11],
    [0.18, 0.16, 0.12],
    [0.22, 0.14, 0.08],
    [0.08, 0.08, 0.08],
    [0.30, 0.24, 0.18],
]

# Phase 7 — per-part appearance darkening. On every state transition we
# replace each tracked Shape's `appearance` field with a state-specific
# PBRAppearance, so the robot itself visibly degrades alongside the
# floating markers and debris. Wheels are already very dark; non-wheel
# parts (chassis, bumpers, top plate) take the more visible darkening.
# Phase 8 — cumulative impact decals. Per-part thresholds live on the
# DamageProfile (decal_threshold_J + default_decal_threshold_J). The
# remaining knobs below — visual size, color, per-part cap — are
# universal and apply across all profiles.
DECAL_PER_PART_CAP = 30
DECAL_SIZE = 0.08
DECAL_COLOR = [0.08, 0.06, 0.04]

# Phase 14c — per-impact local dents. Bigger and more 3D than the
# Phase 8 decal scuffs; read as bent metal panels. Triggered on
# impacts above DENT_THRESHOLD_J (separate from the decal threshold,
# so a typical run gets ~5-10 cosmetic decals + a smaller number of
# more visible dents at heavier strikes). Random rotation per dent.
DENT_THRESHOLD_J = 5.0
DENT_PER_PART_CAP = 20
DENT_SIZE = (0.10, 0.08, 0.04)         # Box dims
DENT_COLOR = [0.06, 0.05, 0.04]        # very dark, scuff-like
DENT_TILT_RANGE = (-0.6, 0.6)          # radians of random tilt per axis

# Phase 11 — particle effects. Three effect types, all spawned as
# lifetime-tracked Solids alongside the Phase 4 markers. Cadence
# limits matter more than visual tuning: too many spawns and the
# scene clogs.
SMOKE_IMPULSE_THRESHOLD_J = 8.0
SMOKE_RATE_LIMIT_MS = 200          # min ms between consecutive smoke spawns
SMOKE_SIZE_RANGE = (0.10, 0.18)
SMOKE_LIFETIME_MS = 1500
SMOKE_COLORS: list[list[float]] = [
    [0.55, 0.55, 0.55],
    [0.40, 0.40, 0.40],
    [0.65, 0.62, 0.58],
]
SMOKE_RISE_VEL = 0.4               # m/s upward bias

SPARK_IMPULSE_THRESHOLD_J = 5.0
SPARK_RATE_LIMIT_MS = 80
SPARK_SIZE_RANGE = (0.015, 0.030)
SPARK_LIFETIME_MS = 350
SPARK_COLORS: list[list[float]] = [
    [1.00, 0.95, 0.40],   # bright yellow
    [1.00, 0.65, 0.10],   # orange
    [1.00, 0.85, 0.20],
]

# Fluid leaks: once per second while chassis is damaged/broken. Drops
# fall under gravity (so they pile up on the floor) and have a longer
# lifetime so the puddle reads as ongoing.
FLUID_LEAK_INTERVAL_MS = 1000
FLUID_DROP_RADIUS = 0.025
FLUID_LIFETIME_MS = 4000
FLUID_COLOR_DAMAGED = [0.12, 0.30, 0.10]   # dark coolant green
FLUID_COLOR_BROKEN  = [0.45, 0.06, 0.05]   # dark crimson

# Phase 9 wheel-detachment geometry now lives on the DamageProfile
# (wheel_height/radius/mass/mesh_url). See damage_profiles.py.


def _appearance_stanza(part: str, state: str, wheel_parts: frozenset) -> str:
    """Build a PBRAppearance VRML stanza for a given (part, state)."""
    if part in wheel_parts:
        # Wheels start near-black; just sliver darker on damage so the
        # state escalation reads.
        palette = {
            "pristine": (0.10, 0.10, 0.10, 0.7),
            "scuffed":  (0.09, 0.09, 0.09, 0.75),
            "damaged":  (0.07, 0.06, 0.05, 0.85),
            "broken":   (0.04, 0.03, 0.02, 0.95),
        }
    else:
        # Chassis/bumpers/top plate start a Husky-ish gray, darken+brown
        # toward charred on the way to broken.
        palette = {
            "pristine": (0.60, 0.62, 0.64, 0.55),
            "scuffed":  (0.45, 0.43, 0.40, 0.65),
            "damaged":  (0.30, 0.25, 0.20, 0.80),
            "broken":   (0.12, 0.10, 0.07, 0.95),
        }
    r, g, b, rough = palette.get(state, palette["pristine"])
    return (
        f'PBRAppearance {{'
        f'  baseColor {r} {g} {b}'
        f'  roughness {rough}'
        f'  metalness 0.0'
        f'}}'
    )


def _solid_name(node) -> str:
    """Return the SFString 'name' field of a node, or '' if unavailable."""
    if node is None:
        return ""
    f = node.getField("name")
    if f is None:
        return ""
    try:
        return f.getSFString() or ""
    except Exception:
        return ""


def find_robot(supervisor, robot_name: str):
    """Walk the root's children for a top-level Robot/URDFRobot whose
    'name' field matches. Returns the node or None.
    """
    root = supervisor.getRoot()
    if root is None:
        return None
    children = root.getField("children")
    if children is None:
        return None
    try:
        count = children.getCount()
    except Exception:
        return None
    for i in range(count):
        node = children.getMFNode(i)
        if node is None:
            continue
        try:
            type_name = node.getTypeName()
        except Exception:
            continue
        if type_name not in ("Robot", "URDFRobot"):
            continue
        if _solid_name(node) == robot_name:
            return node
    return None


def collect_descendant_solids(node) -> list:
    """DFS walk the Solid subtree rooted at `node` (inclusive). Yields
    every Solid encountered. URDFRobot expansion produces Solid nodes
    for each URDF link with the link name as the SFString 'name'.
    """
    out: list = []
    if node is None:
        return out
    stack = [node]
    while stack:
        n = stack.pop()
        try:
            t = n.getTypeName()
        except Exception:
            t = ""
        if t in ("Solid", "Robot", "URDFRobot"):
            out.append(n)
        # Walk children/endPoint fields.
        for fname in ("children", "endPoint"):
            f = n.getField(fname)
            if f is None:
                continue
            try:
                if fname == "endPoint":
                    child = f.getSFNode()
                    if child is not None:
                        stack.append(child)
                else:
                    cnt = f.getCount()
                    for i in range(cnt):
                        ch = f.getMFNode(i)
                        if ch is not None:
                            stack.append(ch)
            except Exception:
                continue
    return out


def _node_mass(node) -> float:
    """Best-effort mass lookup. Reads the SFFloat 'mass' field of the
    node's Physics child if present. Returns 0.0 when unavailable
    (massless/static or the node has no Physics).
    """
    if node is None:
        return 0.0
    pf = node.getField("physics")
    if pf is None:
        return 0.0
    try:
        physics = pf.getSFNode()
    except Exception:
        return 0.0
    if physics is None:
        return 0.0
    mf = physics.getField("mass")
    if mf is None:
        return 0.0
    try:
        m = float(mf.getSFFloat())
    except Exception:
        return 0.0
    # Webots sentinel: -1 means "compute from density × volume". We can't
    # cheaply derive that here, so return a conservative non-zero so the
    # impulse formula still produces a useful magnitude.
    if m <= 0:
        return 1.0
    return m


def _node_velocity(node) -> list[float]:
    """Linear velocity (vx, vy, vz). Webots returns 6 elements (linear +
    angular); we only need the linear part for impulse magnitude.
    """
    if node is None:
        return [0.0, 0.0, 0.0]
    try:
        v = node.getVelocity()
    except Exception:
        return [0.0, 0.0, 0.0]
    if not v or len(v) < 3:
        return [0.0, 0.0, 0.0]
    return [float(v[0]), float(v[1]), float(v[2])]


def _vmag(v: list[float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _contact_impulse_J(mass: float, dv_mag: float, depth: float,
                       use_depth: bool, depth_scale: float) -> tuple[float, bool]:
    """Per-contact damage magnitude, in the profile's Joule-proxy units.

    Default (``use_depth`` False): the historical momentum proxy
    ``mass*|Δv|`` — byte-identical to the pre-depth scoring, so every
    shipping demo and the ODE battlebot games are unchanged while the
    opt-in is off.

    Depth mode (``use_depth`` True): when a real per-contact penetration
    ``depth`` (m, from the engine's contact API — see
    physics-contact-impulse-api.md) is available (>0), score on
    ``mass * depth_scale * depth`` instead. Penetration is a backend-
    symmetric, jitter-free signal that does not inherit Newton's per-step
    ``body_qd`` write-back jitter (the 57k-spurious-event problem the
    velocity proxy suffers on Newton). ``depth_scale`` (1/s) maps
    penetration metres to an equivalent approach velocity so the product
    lands in the same range as ``mass*|Δv|`` and the existing per-part
    ``*_threshold_J`` tables stay valid — no threshold re-derivation, the
    whole change is one tunable constant.

    Graceful degradation: under the default **XPBD** Newton solver
    ``depth`` is reported 0 (a positional solve exposes no penetration
    witness), so depth mode transparently falls back to the velocity
    proxy there — it only "bites" where real depth exists (ODE,
    Newton+MuJoCo). Returns ``(impulse_J, used_depth)``.
    """
    if use_depth and depth is not None and depth > 0.0:
        return mass * depth_scale * depth, True
    return mass * dv_mag, False


def _matrix_to_axis_angle(m: list[float]) -> list[float]:
    """Convert a 3x3 row-major rotation matrix (Webots' getOrientation
    layout) to VRML axis-angle: [ax, ay, az, angle] with (ax,ay,az)
    a unit axis. Used by Phase 9 to spawn a detached wheel at the
    original wheel's orientation.
    """
    trace = m[0] + m[4] + m[8]
    cos_a = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    angle = math.acos(cos_a)
    if angle < 1e-6:
        return [0.0, 1.0, 0.0, 0.0]
    s = 2.0 * math.sin(angle)
    if abs(s) < 1e-9:
        # Angle near pi — fall back to a default axis; wheel detachment
        # tolerates approximate orientation.
        return [0.0, 1.0, 0.0, angle]
    ax = (m[7] - m[5]) / s
    ay = (m[2] - m[6]) / s
    az = (m[3] - m[1]) / s
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-9:
        return [0.0, 1.0, 0.0, angle]
    return [ax / n, ay / n, az / n, angle]


def _world_to_local(world_point: list[float], parent_pos: list[float],
                    parent_rot: list[float]) -> list[float]:
    """Convert a world-coords 3D point to the local frame of a parent
    Solid given the parent's world position and 3x3 orientation matrix
    (Webots row-major flattening). The rotation matrix is orthonormal
    so its inverse is its transpose.
    """
    dx = world_point[0] - parent_pos[0]
    dy = world_point[1] - parent_pos[1]
    dz = world_point[2] - parent_pos[2]
    R = parent_rot
    lx = R[0] * dx + R[3] * dy + R[6] * dz
    ly = R[1] * dx + R[4] * dy + R[7] * dz
    lz = R[2] * dx + R[5] * dy + R[8] * dz
    return [lx, ly, lz]


def _read_mesh_url(shape) -> str | None:
    """If `shape`'s geometry is a Mesh node with a non-empty url field,
    return the first URL string. Otherwise None. Used by Phase 14 to
    snapshot pristine geometry so reset() can restore it after
    procedural deformation has overwritten it.
    """
    if shape is None:
        return None
    geom_field = shape.getField("geometry")
    if geom_field is None:
        return None
    try:
        geom = geom_field.getSFNode()
    except Exception:
        return None
    if geom is None:
        return None
    try:
        if geom.getTypeName() != "Mesh":
            return None
    except Exception:
        return None
    url_field = geom.getField("url")
    if url_field is None:
        return None
    try:
        urls = url_field.getMFString()
    except Exception:
        return None
    if not urls:
        return None
    return str(urls[0])


def _collect_shapes(node) -> list:
    """DFS walk yielding every Shape node under `node`. Used by Phase 7
    to snapshot which Shapes belong to which logical part so we can
    rewrite their appearance fields on state transitions.
    """
    out: list = []
    if node is None:
        return out
    stack = [node]
    while stack:
        n = stack.pop()
        try:
            t = n.getTypeName()
        except Exception:
            continue
        if t == "Shape":
            out.append(n)
            continue  # Shape children (appearance, geometry) aren't worth recursing
        for fname in ("children", "endPoint"):
            f = n.getField(fname)
            if f is None:
                continue
            try:
                if fname == "endPoint":
                    child = f.getSFNode()
                    if child is not None:
                        stack.append(child)
                else:
                    cnt = f.getCount()
                    for i in range(cnt):
                        ch = f.getMFNode(i)
                        if ch is not None:
                            stack.append(ch)
            except Exception:
                continue
    return out


def _closest_part(point: list[float], centers: list[tuple[str, list[float]]]) -> str:
    """Return the logical-part label whose Solid center is closest to
    `point` in 3D Euclidean distance. Falls back to 'chassis' when
    `centers` is empty (shouldn't happen post-discovery but the fallback
    keeps a malformed Husky from breaking the poll).
    """
    best_label = "chassis"
    best_d2 = float("inf")
    for label, c in centers:
        dx = point[0] - c[0]
        dy = point[1] - c[1]
        dz = point[2] - c[2]
        d2 = dx * dx + dy * dy + dz * dz
        if d2 < best_d2:
            best_d2 = d2
            best_label = label
    return best_label


def _closest_non_wheel_part(point: list[float],
                            centers: list[tuple[str, list[float]]],
                            wheel_parts: frozenset) -> str:
    """Pick the closest non-wheel part by horizontal (xy) distance. Used
    to reattribute high-Z contacts that node_id-attributed to a wheel
    because of URDFRobot consolidation. Horizontal because boxes drop
    from above — vertical distance to the contact is dominated by the
    box's height, not by which body part is actually being hit.
    """
    best_label = "chassis"
    best_d2 = float("inf")
    for label, c in centers:
        if label in wheel_parts:
            continue
        dx = point[0] - c[0]
        dy = point[1] - c[1]
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best_label = label
    return best_label


def _slab_for_point(point: list[float],
                    centers: list[tuple[str, list[float]]],
                    slab_parts: frozenset,
                    slab_sizes: dict[str, tuple[float, float, float]]) -> str | None:
    """Phase 20 — chassis-to-slab reattribution. Returns the name of
    the slab (top_plate, front_bumper, etc.) whose world-axis-aligned
    bbox contains `point`, or None if the point doesn't fall inside
    any slab.

    World-axis-aligned approximation: assumes the slab's local frame
    is roughly upright (no roll/pitch) and that 180-deg yaw symmetry
    is fine because slabs have symmetric extents in xy. Works for
    the husky's flat ground driving; revisit for arms or robots that
    pitch/roll significantly during normal operation.

    Bbox padding: `crumple_size` is the *tight* geometric extent of
    the part (used by the procedural deformation pass to size its
    IndexedFaceSet). For attribution, the contact point usually sits
    on the part's surface — exactly at the bbox edge — so a strict
    `<` against `size/2` rejects it. Pad each axis by `SLAB_BBOX_PAD`
    (10 cm) so contacts on the top of the top_plate or the leading
    face of a bumper still land inside the bbox.

    Tie-breaking: when multiple slabs' padded bboxes overlap (e.g. a
    contact at z=0.26 in front of the chassis sits inside both
    top_plate's padded bbox AND front_bumper's bbox), pick the slab
    whose CENTRE is closest to the contact in 3D distance. That's the
    physical "which slab is the contact actually on" answer — a
    chassis-front contact is much closer to front_bumper's centre
    (offset along +X) than to top_plate's centre (above chassis).
    """
    if not slab_parts:
        return None
    best_label: str | None = None
    best_d2 = float("inf")
    for label, c in centers:
        if label not in slab_parts:
            continue
        size = slab_sizes.get(label)
        if size is None:
            continue
        sx, sy, sz = size
        hx = sx * 0.5 + SLAB_BBOX_PAD
        hy = sy * 0.5 + SLAB_BBOX_PAD
        hz = sz * 0.5 + SLAB_BBOX_PAD
        dx = point[0] - c[0]
        dy = point[1] - c[1]
        dz = point[2] - c[2]
        if abs(dx) <= hx and abs(dy) <= hy and abs(dz) <= hz:
            d2 = dx * dx + dy * dy + dz * dz
            if d2 < best_d2:
                best_d2 = d2
                best_label = label
    return best_label


SLAB_BBOX_PAD = 0.10  # m; 10 cm tolerance around each axis of the slab bbox

# Phase 21: chassis Δv magnitude (m/s per step) above which a synthetic
# event is treated as a real crash and gets the crash_impulse_multiplier
# boost. Set equal to the profile's synthetic_chassis_dv_threshold so
# every fired synth event gets the crash boost — driving noise alone
# rarely exceeds 0.1 m/s per step, so this is effectively a "boost
# every real chassis-impulse event" rule. The multiplier value is what
# tunes the magnitude.
CRASH_DV_THRESHOLD = 0.1


class DamageTracker:
    """Per-step contact poller and event ring buffer.

    Attribute model is intentionally flat — Phase 2 will add per-part HP
    on the same `parts` dict, and Phase 3 will read `events` directly
    when servicing the harness `damage_events` command.
    """

    def __init__(self, supervisor, robot_name: str = "husky",
                 profile: DamageProfile | None = None,
                 buffer_size: int = 1024,
                 log_summary_period_s: float = 5.0):
        self.supervisor = supervisor
        self.robot_name = robot_name
        # Profile resolution: if caller passes one explicitly we honour
        # it. Otherwise auto-select using the robot's customData (for
        # explicit advertise) and observed URDF link names (for sniff).
        # The robot lookup happens here so we can feed it into selection.
        self.robot_node = find_robot(supervisor, robot_name)
        if profile is not None:
            self.profile: DamageProfile = profile
        else:
            self.profile = self._auto_select_profile()
        # `part_table` retained as an attribute for the diagnostic snapshot;
        # it's just the profile's part_table mirror.
        self.part_table = self.profile.part_table

        # Logical part -> list of Solid nodes that map to it.
        self.parts: dict[str, list] = {}
        # Direct lookup for contact attribution. getContactPoints(True)
        # on the URDFRobot returns each ContactPoint with node_id set to
        # the descendant Solid that owns the colliding boundingObject —
        # so we can attribute each contact in O(1) via this map. Closest-
        # point fallback handles the rare case where the descendant is
        # nameless or unmapped.
        self.id_to_part: dict[int, str] = {}
        # Flat list of (logical_part, solid_node) for the closest-point
        # fallback. Positions are read per step in poll().
        self.attribution_table: list[tuple[str, object]] = []
        # Cached velocity from the previous step per node id. Used for
        # impulse-via-momentum-conservation: when a Husky part's velocity
        # changes over a step, that delta is the impulse it received.
        self.prev_velocity: dict[int, list[float]] = {}
        # Mass per node id, looked up once. Used to convert velocity
        # delta into momentum (impulse magnitude).
        self.mass_cache: dict[int, float] = {}

        self.events: collections.deque = collections.deque(maxlen=buffer_size)
        self.event_counter = 0
        self.dropped_events = 0  # incremented when buffer wraps
        # contacts_seen counts every contact point returned by the API.
        self.contacts_seen = 0
        # P6 contact-velocity smoothing (OMNISIM_DAMAGE_VEL_SMOOTH = EMA
        # weight in percent of the NEW sample, 0 = OFF/default → existing ODE
        # behaviour unchanged). The contact API exposes no per-contact
        # impulse/depth (only world point + colliding node_id), so the tracker
        # synthesises impulse as mass*|Δv| from per-body getVelocity() deltas.
        # On ODE a resting/rolling wheel has ~0 per-step Δv so its persistent
        # per-step contact stays under the buffer threshold (head-on debounces
        # to ~149 events). On Newton the body_qd write-back carries per-step
        # solver JITTER, so |Δv| is spuriously large every step and the same
        # persistent contacts clear the threshold and re-count to ~57k. We
        # low-pass each body's velocity with an EMA before differencing:
        # zero-mean jitter is attenuated (rolling/resting Δv → ~0, like ODE)
        # while a genuine sustained crash deceleration survives across steps
        # (so the multi-second collision still produces the per-step impact
        # stream ODE reports). Opt-in so the ODE battlebot damage games are
        # never altered. _emit's impulse gate then does the rest.
        try:
            _vs = int(os.environ.get("OMNISIM_DAMAGE_VEL_SMOOTH", "0"))
        except ValueError:
            _vs = 0
        # alpha in (0,1]; smaller = more smoothing. 1.0 (or 0 env) = no smoothing.
        self.vel_smooth_alpha = 1.0 if _vs <= 0 else max(0.05, min(1.0, _vs / 100.0))
        self.vel_ema: dict = {}  # nid -> smoothed velocity [vx,vy,vz]
        # L4: opt-in real-penetration damage magnitude. When
        # OMNISIM_DAMAGE_USE_DEPTH is set, the per-part contact path scores on
        # the engine's real per-contact penetration `cp.depth` (jitter-free,
        # backend-symmetric) instead of the synthetic mass*|Δv| momentum proxy
        # — the principled successor to OMNISIM_DAMAGE_VEL_SMOOTH (which
        # low-passes the proxy's Newton jitter rather than sidestepping it).
        # Default OFF → existing scoring byte-unchanged. Where depth is scored,
        # the per-contact path no longer depends on the velocity delta, so
        # vel-smooth becomes unnecessary for it (full vel-smooth retirement also
        # needs the synthetic-chassis path off velocity, which awaits an XPBD
        # penetration witness — L1). See physics-contact-impulse-api.md.
        self.use_depth = os.environ.get(
            "OMNISIM_DAMAGE_USE_DEPTH", "0") not in ("", "0", "false", "False")
        # 1/s: maps penetration metres to an equivalent approach velocity so
        # mass*depth_scale*depth lands in the same J-proxy range as mass*|Δv|,
        # keeping the existing per-part *_threshold_J tables valid. Calibrated
        # against the head-on capture; env-overridable per world.
        try:
            self.depth_scale = float(
                os.environ.get("OMNISIM_DAMAGE_DEPTH_SCALE", "100.0"))
        except ValueError:
            self.depth_scale = 100.0
        if self.depth_scale <= 0.0:
            self.depth_scale = 100.0
        # Telemetry (surfaced in damage_state): contacts scored via real depth
        # vs. depth-mode-on steps that fell back to the proxy (XPBD/old wire).
        self.depth_contacts_used = 0
        self.depth_contacts_fallback = 0
        # Phase 1.5 (CUDA particle effects): best-effort RPC channel to
        # a cuda_particle_pool supervisor on :6791. Falls back to the
        # legacy importMFNodeFromString path when the pool isn't in
        # the world. Connection retries with backoff: every failure
        # extends the cooldown so we don't hammer when the pool isn't
        # there, but we do try again periodically so a pool that boots
        # late (Webots starts controllers asynchronously) eventually
        # gets used.
        self._pool_sock: socket.socket | None = None
        self._pool_next_retry_ms = 0
        self._pool_retry_cooldown_ms = 1000  # starts at 1s, doubles up to 30s
        self._pool_spawn_count = 0  # exposed in damage_state for telemetry
        self._pool_fail_count = 0   # exposed for diagnostics
        # When the supervisor's customData has "use_particle_pool": true,
        # we know the world has the pool wired in. Pool failures stop
        # falling back to the legacy importMFNodeFromString path —
        # otherwise the legacy spawn flood under collision slows the
        # whole sim down so much that the pool's own main loop can't
        # keep up, leading to a deadlock-by-starvation.
        self._suppress_legacy_spawn = self._read_use_pool_flag(supervisor)
        # Phase 20 debug: count chassis-attributed contacts entering the
        # slab-reattribution check, and how many actually got rebadged.
        # Exposed in damage_state so we can verify the path is firing
        # without spamming stderr each step.
        self.slab_check_count = 0
        self.slab_reattrib_count = 0
        # Phase 20 debug, synthetic path: how many times the synthetic
        # chassis detector fired, how many of those rebadged to a slab,
        # how many contacts above z>0.05 the pre-scan saw on a fired
        # step, and the highest contact z observed (so we can sanity-
        # check that bumper-height contacts are actually visible).
        self.synth_fire_count = 0
        self.synth_to_slab_count = 0
        self.synth_high_z_contacts_total = 0
        self.synth_max_z_seen = 0.0
        # Phase 21: counts synth firings whose Δv exceeded
        # CRASH_DV_THRESHOLD and got the crash_impulse_multiplier boost.
        self.crash_boost_count = 0
        # Per-part HP/state record. Keyed by logical part name; populated
        # in _init_part_hp() once the discovery walk has built the parts
        # dict. Phase 3's `damage_state` command surfaces this directly.
        self.part_hp: dict[str, dict] = {}
        # Cached URDFRobot body mass and previous-step velocity for the
        # synthetic chassis impact detector. Mass is read once at
        # discovery (it's the sum of fused-body link masses; Webots
        # exposes it via the URDFRobot's Physics field).
        self.chassis_mass = 0.0
        self.prev_chassis_velocity: list[float] | None = None
        # Phase 4: spawned damage marker (def_name, sim_time_ms_created)
        # tuples. Tracked so reset() can remove them and so poll() can
        # expire stale markers after MARKER_LIFETIME_MS.
        self.damage_markers: list[tuple[str, int]] = []
        self._marker_counter = 0
        # Phase 6: spawned debris chunk DEF names. Same lifetime as
        # markers — cleared by damage_reset.
        self.debris_chunks: list[str] = []
        self._debris_counter = 0
        # Deterministic-but-varied bursts: seed the chunk RNG from the
        # robot name so re-runs of the same demo look the same, but
        # each chunk in a burst still varies.
        self._debris_rng = random.Random(robot_name)
        # Phase 7: Shape nodes per logical part, populated at discovery.
        # Maps logical_part -> [Shape node, ...]. Updated only at startup
        # (URDFRobot subtree is structurally fixed once loaded), used by
        # _apply_appearance to rewrite each Shape's appearance field on
        # state transitions.
        self.part_shapes: dict[str, list] = {}
        # Phase 14: per-Shape original Mesh URL captured at discovery so
        # damage_reset can swap procedural geometry back to the URDF's
        # imported mesh. Indexed parallel to self.part_shapes[part];
        # entry is None when the original geometry wasn't a Mesh (e.g.
        # already a primitive).
        self.original_geometry_urls: dict[str, list[str | None]] = {}
        # Phase 15: per-part deformable vertex buffer. Initialized on
        # first transition to a non-pristine state (we don't pay the
        # cost on robots that never take damage). Phase 14b's uniform
        # state-transition crumple and Phase 15's per-impact dents
        # both accumulate in this buffer.
        self.part_vertices: dict[str, list[float]] = {}
        # Phase 16b: shared vertex-neighbor table for spring coupling.
        # Topology is identical across all procedural-deformation parts
        # (same subdivision, same face layout) so one table suffices.
        # Built lazily on the first vertex-buffer init.
        self._neighbor_table: list[list[int]] | None = None
        # Phase 16d: separate table with boundary vertices pinned to
        # empty neighbor lists. Used by the relaxation pass so face-
        # edge/corner vertices don't get pulled inward into the face
        # interior (which would shrink the box on every iteration).
        self._relax_neighbor_table: list[list[int]] | None = None
        # Phase 16d: per-part timestamp of the last dent application.
        # Drives the relaxation pass — only relax recently-dented parts.
        self._last_dent_at_ms: dict[str, int] = {}
        # Phase 17c: fragment DEF names spawned by topology fracture.
        # Tracked so damage_reset can remove them and so geometry_stats
        # can report fragments_spawned_total.
        self.fragments_spawned: list[str] = []
        self._fragment_counter = 0
        # Phase 18: per-part heal rates. Zero default = no passive
        # regen. Phase 18c repair stations override these dynamically;
        # Phase 18d wire-protocol command lets agents set them
        # explicitly. heal_rate_hp_per_s drives Phase 18a HP regen;
        # heal_rate_mesh_m_per_s drives Phase 18b vertex buffer regen.
        self.heal_rate_hp_per_s: dict[str, float] = {}
        self.heal_rate_mesh_m_per_s: dict[str, float] = {}
        self._last_heal_tick_ms: int = -1
        # Per-part dirty flag + last-remit timestamp for the rate-limited
        # geometry re-import sweep in poll().
        self.dirty_meshes: dict[str, bool] = {}
        self.last_remit_ms: dict[str, int] = {}
        # Tracks the crumple intensity already baked into part_vertices,
        # so each transition applies only the *delta* of random uniform
        # dents on top of accumulated localized ones.
        self.current_crumple: dict[str, float] = {}
        # Phase 14c: per-part list of (def_name, parent_solid) for
        # cumulative local dents. Capped per part; oldest is removed
        # when a new dent pushes the count over the cap.
        self.part_dents: dict[str, list[tuple[str, object]]] = {}
        self._dent_counter = 0
        # Phase 8: per-part list of (def_name, parent_solid) for cumulative
        # impact decals. Capped per part; oldest is removed when a new
        # decal pushes the count over the cap.
        self.part_decals: dict[str, list[tuple[str, object]]] = {}
        self._decal_counter = 0
        # Phase 11: particles (smoke, sparks, fluid). Each entry is
        # (def_name, expire_at_sim_time_ms). Same expiry-sweep pattern
        # as markers. Rate-limit timestamps so cadence stays sane.
        self.particles: list[tuple[str, int]] = []
        self._particle_counter = 0
        self._last_smoke_at = -10**9
        self._last_spark_at = -10**9
        self._last_fluid_at = -10**9
        # Diagnostic: count attempts vs. successful spawns so we can tell
        # "the threshold never fires" from "spawn keeps short-circuiting".
        self._decal_attempts = 0
        # Phase 9 + 19: parts we've physically detached and the DEF
        # names of the corresponding free Solids spawned to replace
        # them. Tracked so damage_reset can clean up the free bodies
        # (the originals are gone for good — reset heals state but the
        # structural damage is permanent until world reload).
        self.detached_parts: set[str] = set()
        self.detached_part_defs: list[str] = []
        # Cache the world's root children field for marker spawning.
        root = supervisor.getRoot()
        self.root_children = root.getField("children") if root is not None else None

        # Periodic summary throttling.
        self._last_summary_t = time.monotonic()
        self._summary_period_s = float(log_summary_period_s)
        self._summary_counts: dict[str, int] = {}

        if self.robot_node is None:
            sys.stderr.write(
                f"[damage_tracker] no Robot/URDFRobot named {robot_name!r} found; "
                "tracker will idle\n"
            )
            return

        self._discover_parts()
        self._init_part_hp()
        self._write_behavior_gate()
        # Chassis body mass for the synthetic impact detector. We try
        # the URDFRobot node's own Physics first, then fall back to the
        # profile's default so the detector still works on robots whose
        # URDF doesn't expose a top-level Physics field.
        self.chassis_mass = _node_mass(self.robot_node)
        if self.chassis_mass <= 1.0:
            self.chassis_mass = self.profile.chassis_mass_default
        sys.stderr.write(
            f"[damage_tracker] tracking {robot_name!r} with profile "
            f"{self.profile.name!r}: "
            f"{sum(len(v) for v in self.parts.values())} solids across "
            f"{len(self.parts)} logical parts ({sorted(self.parts.keys())})\n"
        )
        sys.stderr.flush()

    def _auto_select_profile(self) -> DamageProfile:
        """Resolve a DamageProfile when the caller didn't pass one. See
        damage_profiles.select_profile for the priority order; we
        provide it with whatever signals are observable on this robot.
        """
        custom_data = ""
        link_names: set[str] = set()
        if self.robot_node is not None:
            f = self.robot_node.getField("customData")
            if f is not None:
                try:
                    custom_data = f.getSFString() or ""
                except Exception:
                    custom_data = ""
            for solid in collect_descendant_solids(self.robot_node):
                link_names.add(_solid_name(solid))
        return select_profile(custom_data_raw=custom_data,
                              observed_link_names=link_names)

    @property
    def attached(self) -> bool:
        return self.robot_node is not None

    def _discover_parts(self) -> None:
        """Walk the robot subtree, bucketing Solids by logical part and
        building a node_id -> logical_part lookup. Any Solid whose URDF
        link name isn't in part_table maps to `chassis` so unknown
        attached geometry still contributes. Also collects every Shape
        under each tracked Solid for Phase 7's appearance rewriting.
        """
        for solid in collect_descendant_solids(self.robot_node):
            try:
                node_id = solid.getId()
            except Exception:
                node_id = -1

            link_name = _solid_name(solid)
            if not link_name:
                continue
            logical = self.part_table.get(link_name, "chassis")
            self.parts.setdefault(logical, []).append(solid)
            self.attribution_table.append((logical, solid))
            if node_id >= 0:
                self.id_to_part[node_id] = logical
                # Cache mass once for impulse computation.
                self.mass_cache[node_id] = _node_mass(solid)
            # Phase 7: collect shapes under this solid (not includeDescendants
            # at the URDFRobot level, because we want to attribute shapes
            # to specific logical parts).
            new_shapes = _collect_shapes(solid)
            self.part_shapes.setdefault(logical, []).extend(new_shapes)
            # Phase 14: snapshot the original Mesh URL per shape so reset()
            # can swap procedural geometry back to the URDF's imported
            # mesh. Non-Mesh geometry (already a primitive) records None.
            url_bucket = self.original_geometry_urls.setdefault(logical, [])
            for shape in new_shapes:
                url_bucket.append(_read_mesh_url(shape))

    def _init_part_hp(self) -> None:
        """Initialise per-part HP records once parts have been discovered.
        Each record carries the state band, current HP, max HP, and a
        few diagnostic fields that Phase 3 surfaces verbatim.
        """
        for part in self.parts.keys():
            hp_max = self.profile.part_hp_max.get(part, self.profile.default_hp_max)
            self.part_hp[part] = {
                "state": "pristine",
                "hp": hp_max,
                "hp_max": hp_max,
                "last_impact_step": 0,
                "last_impact_J": 0.0,
                "total_impulse_J": 0.0,
            }

    @staticmethod
    def _state_for_hp(hp: float, hp_max: float) -> str:
        fraction = hp / hp_max if hp_max > 0 else 0.0
        for label, floor in STATE_BANDS:
            if fraction >= floor:
                return label
        return STATE_BANDS[-1][0]

    def _apply_damage(self, part: str, impulse_J: float, sim_time_ms: int,
                      point: list[float] | None = None) -> None:
        """Update the per-part HP record for one impact event. Crossing a
        STATE_BANDS boundary emits a state-transition event onto the
        same ring buffer as impact events, sharing the monotonic step_id.
        Point is the contact-point world position; used by Phase 4 to
        spawn the visual damage marker at the right place.
        """
        record = self.part_hp.get(part)
        if record is None:
            return
        record["last_impact_step"] = self.event_counter
        record["last_impact_J"] = impulse_J
        record["total_impulse_J"] += impulse_J

        threshold = self.profile.part_hp_threshold_J.get(
            part, self.profile.default_hp_threshold_J
        )
        damage = max(0.0, impulse_J - threshold)
        if damage <= 0.0:
            return

        new_hp = max(0.0, record["hp"] - damage)
        record["hp"] = new_hp
        new_state = self._state_for_hp(new_hp, record["hp_max"])
        if new_state != record["state"]:
            old_state = record["state"]
            record["state"] = new_state
            self._emit_transition(sim_time_ms, part, old_state, new_state,
                                  new_hp, impulse_J, point)

    def poll(self, sim_time_ms: int) -> int:
        """Once per simulation step. Calls getContactPoints(True) on the
        URDFRobot root (per Webots docs the API requires a top-level
        Solid with no Solid parent). Each ContactPoint's node_id is the
        Husky descendant Solid that owns the colliding boundingObject;
        we look up the logical part directly and use that part's own
        velocity delta over the step to compute the received impulse via
        momentum conservation. Returns the number of events emitted.
        """
        if not self.attached:
            return 0

        # Phase 4 refinement: expire markers older than MARKER_LIFETIME_MS
        # so the scene doesn't accumulate floating spheres. Phase 7's
        # chassis darkening already carries the persistent damage state,
        # markers are now flash indicators only.
        if self.damage_markers:
            cutoff = int(sim_time_ms) - MARKER_LIFETIME_MS
            keep: list[tuple[str, int]] = []
            for name, t_created in self.damage_markers:
                if t_created < cutoff:
                    node = self.supervisor.getFromDef(name)
                    if node is not None:
                        try:
                            node.remove()
                        except Exception:
                            pass
                else:
                    keep.append((name, t_created))
            self.damage_markers = keep

        # Phase 11: same lifetime sweep for particles. Each particle
        # carries its own absolute expire-at time so smoke/spark/fluid
        # can have very different lifetimes without per-class machinery.
        if self.particles:
            t_now = int(sim_time_ms)
            keep_p: list[tuple[str, int]] = []
            for name, expire_at in self.particles:
                if expire_at <= t_now:
                    node = self.supervisor.getFromDef(name)
                    if node is not None:
                        try:
                            node.remove()
                        except Exception:
                            pass
                else:
                    keep_p.append((name, expire_at))
            self.particles = keep_p

        # Phase 11: fluid leak ticks even on steps without contacts —
        # it's a state-driven effect, not impact-driven. The spawn
        # checks the chassis state internally and rate-limits to
        # FLUID_LEAK_INTERVAL_MS.
        self._spawn_fluid_drop(int(sim_time_ms))

        # Phase 18: time-based HP + mesh regen. Per-part heal rates
        # default to zero (no passive regen); set via wire-protocol
        # command or implicitly while inside a repair-station volume.
        if self.heal_rate_hp_per_s or self.heal_rate_mesh_m_per_s:
            self._apply_heal(int(sim_time_ms))

        # Phase 16d: per-step Laplacian relaxation on recently-dented
        # parts. Smooths high-frequency dent rims into rolled
        # silhouettes; runs for RELAX_AFTER_MS after each dent and
        # exits early when convergence is reached. Marks dirty when
        # vertices actually moved so the IFS re-emit picks up the
        # smoothed positions.
        if self._last_dent_at_ms and self._relax_neighbor_table is not None:
            t_now = int(sim_time_ms)
            for part, t_last in list(self._last_dent_at_ms.items()):
                if t_now - t_last > RELAX_AFTER_MS:
                    continue
                verts = self.part_vertices.get(part)
                if verts is None:
                    continue
                max_d = relax_vertices(
                    verts, self._relax_neighbor_table,
                    rate=RELAX_RATE, iterations=RELAX_ITERS,
                )
                if max_d > RELAX_CONVERGED_M:
                    self.dirty_meshes[part] = True

        # Phase 15c: rate-limited mesh re-emit. Per-impact dents
        # accumulate in self.part_vertices and mark the part dirty;
        # this sweep regenerates the IFS and swaps it into the part's
        # Shape geometries at most once every REEMIT_INTERVAL_MS so the
        # Webots VRML import cost is bounded.
        if self.dirty_meshes:
            self._remit_dirty_meshes(int(sim_time_ms))

        try:
            contacts = self.robot_node.getContactPoints(True) or []
        except Exception:
            return 0

        # Build part centres up front so both the synthetic chassis
        # detector AND the contact loop can use them. Cheap O(N_solids)
        # walk; the contact loop already needed this every step.
        part_centers: list[tuple[str, list[float]]] = []
        for logical_part, solid in self.attribution_table:
            try:
                pos = list(solid.getPosition())
            except Exception:
                continue
            part_centers.append((logical_part, pos))

        # Phase 20 pre-scan: when the synthetic-chassis detector fires
        # below, the impulse is whatever pushed the URDFRobot's fused
        # body. Without a hint, that always credits "chassis" — but if
        # this step has any contact whose world position falls inside
        # a slab's bbox (a cube on the top plate, a bumper on bumper),
        # the synthetic impulse should route there instead. Pick the
        # slab whose contact has the highest Z so cubes-on-top wins
        # over an incidental side-graze.
        synth_target_part = "chassis"
        synth_target_point: list[float] | None = None
        synth_high_z_count_this_step = 0
        if self.profile.slab_attribute_parts and contacts:
            best_z = -1.0
            for cp in contacts:
                pt = list(cp.point)
                if pt[2] < 0.05:  # ground contacts (wheels) — ignore
                    continue
                synth_high_z_count_this_step += 1
                if pt[2] > self.synth_max_z_seen:
                    self.synth_max_z_seen = pt[2]
                slab = _slab_for_point(
                    pt, part_centers,
                    self.profile.slab_attribute_parts,
                    self.profile.crumple_size,
                )
                if slab is not None and pt[2] > best_z:
                    synth_target_part = slab
                    synth_target_point = pt
                    best_z = pt[2]

        # Synthetic chassis impact: if the URDFRobot's body velocity has
        # changed by more than the threshold this step, something hit
        # the chassis (or it landed after a fall). We can't see this in
        # the contact stream because URDFRobot consolidates contact
        # reporting to the leaf bodies (wheels), so we read body
        # velocity directly. Robot-position contact point is used so the
        # event location is sensible for downstream consumers when no
        # slab pre-scan hint is available.
        chassis_v = _node_velocity(self.robot_node)
        if self.prev_chassis_velocity is not None:
            dv = [chassis_v[i] - self.prev_chassis_velocity[i] for i in range(3)]
            dv_mag = _vmag(dv)
            # Spawn-drop suppression: when the husky first appears at z=0.1
            # (or wherever the world placed it), gravity pulls it onto its
            # wheels in the first 1-2 sim steps. That produces a ~1.4 m/s
            # vertical Δv on the chassis -- which the synthetic detector
            # would otherwise read as `chassis_mass * dv_mag = ~64 J·s`,
            # well above SMOKE / decal thresholds, causing the husky to
            # "self-explode" with smoke/sparks/dents the moment the world
            # starts. We skip the synthetic path until the chassis has
            # settled (vertical velocity below 0.1 m/s for at least one
            # observed step). After settling, real impacts -- including
            # cubes from above onto a stationary husky -- still fire
            # normally because they produce dv much larger than the
            # vertical-only settling oscillations.
            # Hard gate: no real impact unless at least one contact this
            # step is above z=0.05 (i.e., not wheel-on-floor). At spawn
            # drop, during driving, during ordinary settling, every
            # contact is at z≈0 — chassis Δv is gravity / friction /
            # motor torque, NOT a real impact. Suppressing the synth
            # path in that case kills the "huskies explode at startup"
            # behaviour without breaking real cases: a cube from above
            # ALWAYS shows up as a high-Z contact (cube body height),
            # and a head-on bumper-to-bumper hit also shows up high-Z
            # (bumper z≈0.18). Both clear this gate.
            if synth_high_z_count_this_step == 0:
                # No external high-Z contact: this Δv is internal
                # (gravity, drive torque, settling). Don't fire.
                pass
            elif dv_mag >= self.profile.synthetic_chassis_dv_threshold:
                self.synth_fire_count += 1
                self.synth_high_z_contacts_total += synth_high_z_count_this_step
                # Phase 21b: when no contact-based slab hint was found
                # (very common — the collision step's contacts often
                # aren't visible to getContactPoints in the same step
                # the chassis Δv shows up), use the chassis dv direction
                # as the hint. The impact came from the opposite side
                # of where dv points: chassis pushed in -dv direction
                # = struck from +dv direction. Pick the slab whose
                # offset from chassis centre best aligns with -dv.
                if (synth_target_part == "chassis"
                        and self.profile.slab_attribute_parts
                        and dv_mag >= CRASH_DV_THRESHOLD):
                    chassis_center: list[float] | None = None
                    for plabel, ppos in part_centers:
                        if plabel == "chassis":
                            chassis_center = ppos
                            break
                    if chassis_center is not None:
                        # Negative dv = chassis decelerated; impact
                        # direction (toward chassis) = -dv normalized.
                        impact_dir = (-dv[0] / dv_mag,
                                      -dv[1] / dv_mag,
                                      -dv[2] / dv_mag)
                        best_align = 0.5  # require >50% alignment
                        best_slab: str | None = None
                        for plabel, ppos in part_centers:
                            if plabel not in self.profile.slab_attribute_parts:
                                continue
                            ox = ppos[0] - chassis_center[0]
                            oy = ppos[1] - chassis_center[1]
                            oz = ppos[2] - chassis_center[2]
                            omag = math.sqrt(ox * ox + oy * oy + oz * oz)
                            if omag < 1e-3:
                                continue
                            align = (ox * impact_dir[0] + oy * impact_dir[1]
                                     + oz * impact_dir[2]) / omag
                            if align > best_align:
                                best_align = align
                                best_slab = plabel
                        if best_slab is not None:
                            synth_target_part = best_slab
                if synth_target_part != "chassis":
                    self.synth_to_slab_count += 1
                # Damage is plastic deformation work, not momentum
                # transfer. For an (in)elastic collision the energy
                # absorbed by one side ≈ ½·m·Δv². Using m·Δv (units of
                # N·s) and treating it as J was a unit error; at Δv=2
                # the two formulas happen to converge numerically, but
                # the energy form scales correctly with Δv (real
                # collisions hurt more when faster) where momentum is
                # linear.
                impulse_J = 0.5 * self.chassis_mass * dv_mag * dv_mag
                # No multiplier: ODE under-reports impulses some, but
                # we no longer want a 1.5× hand of god dialed up to
                # blow parts off in one impact. Real-energy thresholds
                # in the profile carry the damage curve now.
                dv_xy_mag = math.sqrt(dv[0] * dv[0] + dv[1] * dv[1])
                _ = dv_xy_mag  # retained for future asymmetric checks
                try:
                    pos = list(self.robot_node.getPosition())
                except Exception:
                    pos = [0.0, 0.0, 0.0]
                # Use the slab pre-scan's contact point if we found one
                # so downstream effects (decals, dents) land where the
                # actual impact was, not at the URDFRobot centre.
                emit_point = synth_target_point if synth_target_point else pos
                self._emit(sim_time_ms, synth_target_part, impulse_J,
                           emit_point, "<synthetic>")
                self._apply_damage(synth_target_part, impulse_J, sim_time_ms,
                                   point=emit_point)
                # Phase 8: synthetic-path impacts dominate the chassis
                # impulse stream, so they need their own decal spawn —
                # the contact-path branch below only sees wheel-on-floor
                # rolls and misses the chassis hits.
                if impulse_J >= self.profile.decal_threshold_J.get(
                    "chassis", self.profile.default_decal_threshold_J
                ):
                    self._spawn_decal("chassis", pos)
                # Phase 14c: bigger dent overlays on heavier strikes.
                if impulse_J >= DENT_THRESHOLD_J:
                    self._spawn_dent("chassis", pos)
                # Phase 15b: localized vertex-level deformation of the
                # chassis mesh itself. Lower threshold than the dent
                # overlay so even subtle hits leave a small mark.
                # Synthetic chassis impacts feed the URDFRobot's center
                # as the contact point — so without jitter every dent
                # would land at the same spot in local frame. Spread
                # across the chassis dimensions (matches how decals +
                # 14c overlays get jittered) so dents speckle the body.
                if impulse_J >= DEFORM_THRESHOLD_J:
                    rng = self._debris_rng
                    jittered = [
                        pos[0] + rng.uniform(-0.40, 0.40),
                        pos[1] + rng.uniform(-0.22, 0.22),
                        pos[2] + rng.uniform(0.05, 0.20),
                    ]
                    # Phase 16c: dv is the chassis body's velocity
                    # change over this step — i.e. exactly the
                    # direction the impact pushed it. Use that as the
                    # inward normal instead of the radial approximation.
                    if dv_mag > 1e-6:
                        normal_hint = (dv[0] / dv_mag, dv[1] / dv_mag,
                                       dv[2] / dv_mag)
                    else:
                        normal_hint = None
                    self._apply_impact_dent("chassis", jittered, impulse_J,
                                            sim_time_ms=int(sim_time_ms),
                                            normal_hint=normal_hint)
                # Phase 11: smoke puff at chassis impact above threshold.
                # Rate-limited inside the spawner so a sustained beating
                # doesn't fill the scene.
                if impulse_J >= SMOKE_IMPULSE_THRESHOLD_J:
                    self._spawn_smoke_puff(pos, int(sim_time_ms))
        self.prev_chassis_velocity = chassis_v

        # part_centers was built up front (before the synthetic detector
        # so the slab pre-scan could use it); reused as-is here for the
        # contact-loop reattribution and closest-point fallback.

        # Compute |Δv| per contacting Husky body, once per step. The same
        # body can appear in many contacts (wheels often produce 2-4
        # points each); we only want one velocity-delta computation per.
        dv_per_id: dict[int, float] = {}
        for cp in contacts:
            try:
                nid = int(getattr(cp, "node_id", -1))
            except Exception:
                nid = -1
            if nid < 0 or nid in dv_per_id:
                continue
            node = self.supervisor.getFromId(nid)
            if node is None:
                continue
            v_raw = _node_velocity(node)
            # P6: low-pass the velocity before differencing so Newton's
            # per-step body_qd jitter doesn't manufacture a spurious |Δv|
            # on persistent rolling/resting contacts. alpha=1.0 (default /
            # env unset) = passthrough → ODE behaviour byte-unchanged.
            a = self.vel_smooth_alpha
            if a < 1.0:
                vs = self.vel_ema.get(nid)
                v = list(v_raw) if vs is None else [a * v_raw[i] + (1.0 - a) * vs[i] for i in range(3)]
                self.vel_ema[nid] = v
            else:
                v = v_raw
            prev = self.prev_velocity.get(nid)
            self.prev_velocity[nid] = v
            if prev is None:
                dv_per_id[nid] = 0.0  # first observation, no delta yet
                continue
            dv = [v[0] - prev[0], v[1] - prev[1], v[2] - prev[2]]
            dv_per_id[nid] = _vmag(dv)

        emitted = 0
        for cp in contacts:
            self.contacts_seen += 1
            point = list(cp.point)
            try:
                nid = int(getattr(cp, "node_id", -1))
            except Exception:
                nid = -1

            if nid >= 0 and nid in self.id_to_part:
                logical_part = self.id_to_part[nid]
            else:
                logical_part = _closest_part(point, part_centers)

            # Phase 2: URDFRobot consolidates chassis-area contacts onto
            # the wheel rigid bodies. If a "wheel" contact happens well
            # above the wheel's geometry, it's actually a chassis-or-
            # above hit transmitted through the joint. Reattribute by
            # horizontal distance to the closest non-wheel part.
            if (logical_part in self.profile.wheel_parts and
                    point[2] > self.profile.reattribute_z_threshold):
                logical_part = _closest_non_wheel_part(point, part_centers,
                                                       self.profile.wheel_parts)

            # Phase 20: chassis-to-slab reattribution. The wheel rule
            # above lands many contacts on `chassis`, but URDFRobot
            # also fuses top_plate / bumpers into the same chassis
            # rigid body, so contacts physically on those slabs come
            # in already labelled `chassis`. If the contact's world
            # position falls inside a slab's bbox, rebadge it so the
            # slab actually accumulates HP from being struck.
            if logical_part == "chassis" and self.profile.slab_attribute_parts:
                self.slab_check_count += 1
                slab = _slab_for_point(
                    point, part_centers,
                    self.profile.slab_attribute_parts,
                    self.profile.crumple_size,
                )
                if slab is not None:
                    logical_part = slab
                    self.slab_reattrib_count += 1

            mass = self.mass_cache.get(nid, 0.0)
            dv_mag = dv_per_id.get(nid, 0.0)
            # Default: mass · |Δv|, a momentum proxy (N·s ≈ J·s/m) treated as
            # the impulse magnitude. Opt-in (OMNISIM_DAMAGE_USE_DEPTH): score on
            # the real per-contact penetration `cp.depth` where it's available
            # — backend-symmetric and free of Newton's per-step body_qd jitter.
            # `depth` is 0.0 on the pre-depth wire and under XPBD, in which case
            # the helper transparently falls back to the proxy.
            #
            # Depth scoring is restricted to NON-load-bearing parts. A head-on
            # capture (physics-contact-impulse-api.md) showed husky wheels rest
            # ~0.03 m into the floor under the robot's weight — a large, steady
            # penetration, not the sub-mm a light box shows — so mass*scale*depth
            # on wheels fires every step, the exact resting-contact spam the
            # depth migration exists to kill. Wheels (the profile's load-bearing
            # rolling contacts) therefore stay on the velocity proxy, whose
            # rolling/resting Δv≈0 already suppresses them; depth scores bumpers /
            # slabs / top-plate, which only penetrate on a real collision.
            depth = getattr(cp, "depth", 0.0) or 0.0
            part_uses_depth = (self.use_depth
                               and logical_part not in self.profile.wheel_parts)
            impulse_J, used_depth = _contact_impulse_J(
                mass, dv_mag, depth, part_uses_depth, self.depth_scale)
            if used_depth:
                self.depth_contacts_used += 1
            elif part_uses_depth:
                # depth-eligible part but no penetration witness (XPBD / old
                # wire) -> fell back to the proxy.
                self.depth_contacts_fallback += 1
            self._emit(sim_time_ms, logical_part, impulse_J, point, f"<id:{nid}>")
            self._apply_damage(logical_part, impulse_J, sim_time_ms, point=point)
            # Phase 8: leave a permanent scratch on the impacted part if
            # the impulse cleared its decal threshold. Decals attach to
            # the part Solid so they move with the robot.
            if impulse_J >= self.profile.decal_threshold_J.get(
                logical_part, self.profile.default_decal_threshold_J
            ):
                self._spawn_decal(logical_part, point)
            # Phase 14c: bigger dent overlays on heavier strikes.
            # Limit dents to non-wheel parts: wheel "dents" don't
            # really make sense visually for cylindrical tires.
            if (impulse_J >= DENT_THRESHOLD_J
                    and logical_part not in self.profile.wheel_parts):
                self._spawn_dent(logical_part, point)
            # Phase 15b: localized vertex-level deformation. Same
            # non-wheel filter — wheels don't get vertex dents, just
            # the wheel-specific spark effect.
            if (impulse_J >= DEFORM_THRESHOLD_J
                    and logical_part not in self.profile.wheel_parts):
                self._apply_impact_dent(logical_part, point, impulse_J,
                                        sim_time_ms=int(sim_time_ms))
            # Phase 11: brief sparks at high-impulse wheel contacts
            # (metal-on-metal feel). Rate-limited inside the spawner.
            if (logical_part in self.profile.wheel_parts
                    and impulse_J >= SPARK_IMPULSE_THRESHOLD_J):
                self._spawn_spark(point, int(sim_time_ms))
            emitted += 1

        self._maybe_summary()
        return emitted

    def _emit(self, sim_time_ms: int, part: str, impulse_J: float,
              point: list[float], other_name: str) -> None:
        # Filter low-impulse events (wheel-on-floor steady state) before
        # they enter the ring buffer; otherwise the buffer fills with
        # noise and meaningful impacts get evicted before an agent can
        # observe them. event_counter still increments so caller-side
        # `since` cursors stay monotonic.
        self.event_counter += 1
        if impulse_J < self.profile.buffer_threshold_J.get(
            part, self.profile.default_buffer_threshold_J
        ):
            return

        if len(self.events) == self.events.maxlen:
            self.dropped_events += 1
        evt = {
            "type": "impact",
            "step_id": self.event_counter,
            "sim_time_ms": int(sim_time_ms),
            "part": part,
            "impulse_J": float(impulse_J),
            "point": [float(point[0]), float(point[1]), float(point[2])],
            "other": other_name,
        }
        self.events.append(evt)

        threshold = self.profile.log_threshold_J.get(
            part, self.profile.default_log_threshold_J
        )
        if impulse_J >= threshold:
            sys.stderr.write(
                f"[damage_tracker] t={sim_time_ms}ms part={part} "
                f"impulse={impulse_J:.2f} other={other_name} "
                f"point=({point[0]:.2f},{point[1]:.2f},{point[2]:.2f})\n"
            )
            self._summary_counts[part] = self._summary_counts.get(part, 0) + 1

    def _emit_transition(self, sim_time_ms: int, part: str, from_state: str,
                         to_state: str, hp: float, trigger_J: float,
                         point: list[float] | None = None) -> None:
        """Emit a state-transition event onto the same ring buffer as
        impacts. Distinguished by the `type` field. Always logs to stderr
        regardless of impulse — state changes are rare and load-bearing.
        Also spawns a Phase 4 damage marker at the contact point (or at
        the part's center if no point was supplied) so screenshots tell
        the damage story at a glance.
        """
        if len(self.events) == self.events.maxlen:
            self.dropped_events += 1
        self.event_counter += 1
        evt = {
            "type": "state_transition",
            "step_id": self.event_counter,
            "sim_time_ms": int(sim_time_ms),
            "part": part,
            "from_state": from_state,
            "to_state": to_state,
            "hp": float(hp),
            "trigger_impulse_J": float(trigger_J),
        }
        self.events.append(evt)
        sys.stderr.write(
            f"[damage_tracker] t={sim_time_ms}ms STATE {part}: "
            f"{from_state} -> {to_state} (hp={hp:.1f}, trigger={trigger_J:.2f})\n"
        )
        sys.stderr.flush()

        marker_point = point if point is not None else self._fallback_point_for(part)
        if marker_point is not None:
            self._spawn_damage_marker(marker_point, to_state, sim_time_ms)
            # Phase 6: physical debris burst on the dramatic transition.
            # Only on broken — earlier transitions get the marker only,
            # so the visual escalation reads cleanly.
            if to_state == "broken":
                self._spawn_debris_burst(marker_point)

        # Phase 7: re-skin the affected part's shapes so the robot
        # itself reflects the new state, not just the floating marker.
        self._apply_appearance(part, to_state)
        # Phase 10: swap geometry to the state-specific damaged mesh
        # variant if the profile declares one. No-ops without variants.
        self._apply_geometry(part, to_state)

        # Phase 9 + 19: dramatic finale — when a part configured as
        # detachable breaks, physically remove it from the URDFRobot
        # and spawn a free body that flies/rolls/tumbles away. Wheels
        # take the Phase 9 cylinder path; bumpers and other slab-shaped
        # parts take the Phase 19 box path. Profile decides which parts
        # are detachable; defaults to wheels for backwards compat.
        detachable = self.profile.detachable_parts or self.profile.wheel_parts
        if to_state == "broken" and part in detachable:
            self._detach_part(part)

        # Phase 5: re-publish the behaviour-gate customData so any
        # driving controller picks up the new wheel scale or game_over
        # flag on its next step. Cheap (one SFString write per
        # transition; transitions are rare).
        if part in self.profile.wheel_parts or part == "chassis":
            self._write_behavior_gate()

    def _fallback_point_for(self, part: str) -> list[float] | None:
        """Return a sensible world-space point for a marker when a
        contact point wasn't supplied (e.g. synthetic chassis events
        before we plumbed the point through).
        """
        solids = self.parts.get(part)
        if solids:
            try:
                return list(solids[0].getPosition())
            except Exception:
                return None
        if self.robot_node is not None:
            try:
                return list(self.robot_node.getPosition())
            except Exception:
                return None
        return None

    def _write_behavior_gate(self) -> None:
        """Compute the current wheel-torque scales and game_over flag,
        write them to the robot's customData as JSON. Driving
        controllers read this to know when to throttle their motors.

        Schema (just the keys we own):
            {"wheel_torque": {"fl": 1.0, ...}, "game_over": false}

        Merges with whatever existing customData JSON is on the robot
        so consumers like husky_random's crater payload aren't
        clobbered when the harness loads a world that uses customData
        for purposes other than the damage gate. If the existing
        customData isn't valid JSON, we replace it (we own the
        key namespace, the previous content is presumably legacy).
        """
        if self.robot_node is None:
            return
        f = self.robot_node.getField("customData")
        if f is None:
            return

        wheel_torque: dict[str, float] = {}
        for short, long_name in (("fl", "wheel_fl"), ("fr", "wheel_fr"),
                                  ("rl", "wheel_rl"), ("rr", "wheel_rr")):
            rec = self.part_hp.get(long_name)
            state = rec["state"] if rec else "pristine"
            wheel_torque[short] = WHEEL_TORQUE_SCALE.get(state, 1.0)

        chassis = self.part_hp.get("chassis")
        game_over = bool(chassis and chassis["state"] == "broken")

        # Read existing customData and merge our keys in, preserving
        # any other top-level keys (e.g. husky_random's crater payload).
        # NOTE: Webots' VRML SFString parser truncates initial customData
        # values containing `\"` escapes at the second character. So the
        # FIRST read after world load typically returns garbage; we still
        # accept the merge attempt because subsequent reads — after
        # we've called setSFString below — return the full string fine.
        existing: dict = {}
        try:
            raw = f.getSFString() or ""
        except Exception:
            raw = ""
        if raw:
            try:
                parsed = json.loads(raw)
                if isinstance(parsed, dict):
                    existing = parsed
            except (json.JSONDecodeError, ValueError):
                pass

        existing["wheel_torque"] = wheel_torque
        existing["game_over"] = game_over
        payload = json.dumps(existing)
        try:
            f.setSFString(payload)
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f"[damage_tracker] customData write failed: {exc}\n")

    def _spawn_damage_marker(self, point: list[float], to_state: str,
                              sim_time_ms: int = 0) -> None:
        """Import a small colored sphere into the world at `point`,
        sized and coloured by the new state. Tracks the DEF name so
        reset() can remove it.
        """
        if self.root_children is None:
            return
        if os.environ.get("OMNISIM_LITE_DAMAGE") == "1":
            return
        color = DAMAGE_MARKER_COLORS.get(to_state, DEFAULT_MARKER_COLOR)
        radius = DAMAGE_MARKER_SIZES.get(to_state, DEFAULT_MARKER_SIZE)
        self._marker_counter += 1
        name = f"damage_marker_{self._marker_counter:03d}"
        # Float the marker above the contact point so it's not hidden
        # inside the chassis. Slight per-marker counter offset prevents
        # overlapping markers from z-fighting when multiple transitions
        # land at the same spot.
        z = float(point[2]) + MARKER_Z_OFFSET + 0.05 * self._marker_counter
        stanza = (
            f'DEF {name} Solid {{'
            f'  translation {point[0]} {point[1]} {z}'
            f'  name "{name}"'
            f'  children ['
            f'    Shape {{'
            f'      appearance PBRAppearance {{'
            f'        baseColor {color[0]} {color[1]} {color[2]}'
            f'        emissiveColor {color[0]*0.6} {color[1]*0.6} {color[2]*0.6}'
            f'        roughness 0.4'
            f'        metalness 0.0'
            f'      }}'
            f'      geometry Sphere {{ radius {radius} subdivision 1 }}'
            f'    }}'
            f'  ]'
            f'}}'
        )
        try:
            self.root_children.importMFNodeFromString(-1, stanza)
            self.damage_markers.append((name, int(sim_time_ms)))
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f"[damage_tracker] marker spawn failed: {exc}\n")

    def _apply_geometry(self, part: str, state: str) -> int:
        """Phase 10/14: swap the geometry of every Shape under `part`
        on a state transition. Resolution order:

          1. Static `mesh_variants[part][state]` — Phase 10 path,
             pre-authored damaged .dae assets.
          2. Procedural deformation (Phase 14b) — if the part is in
             `procedural_deformation_parts` AND has a `crumple_size`
             entry, generate a crumpled IndexedFaceSet at the state's
             intensity and swap that in.
          3. Pristine restore — if state is "pristine" and we have a
             snapshotted original Mesh URL, swap back to that.
          4. Otherwise no-op.

        Returns the count of shapes actually rewritten.
        """
        if os.environ.get("OMNISIM_LITE_DAMAGE") == "1":
            return 0
        shapes = self.part_shapes.get(part) or []
        if not shapes:
            return 0

        stanzas = self._resolve_geometry_stanzas(part, state, shapes)
        if not stanzas:
            return 0

        updated = 0
        for shape, stanza in zip(shapes, stanzas):
            if stanza is None:
                continue
            f = shape.getField("geometry")
            if f is None:
                continue
            try:
                f.importSFNodeFromString(stanza)
                updated += 1
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[damage_tracker] geometry swap failed for {part} -> {state}: {exc}\n"
                )
        if updated:
            sys.stderr.write(
                f"[damage_tracker] {part} -> {state}: swapped geometry on "
                f"{updated} shape(s)\n"
            )
        return updated

    def _remit_dirty_meshes(self, sim_time_ms: int) -> None:
        """Phase 15c: walk dirty parts, re-emit IFS from current vertex
        buffer, swap into each Shape's geometry. Rate-limited per part
        so a torrent of impacts doesn't cause a torrent of VRML imports.
        """
        if os.environ.get("OMNISIM_LITE_DAMAGE") == "1":
            return
        for part, dirty in list(self.dirty_meshes.items()):
            if not dirty:
                continue
            last = self.last_remit_ms.get(part, 0)
            if sim_time_ms - last < REEMIT_INTERVAL_MS:
                continue
            verts = self.part_vertices.get(part)
            if not verts:
                self.dirty_meshes[part] = False
                continue
            stanza = vertex_buffer_to_ifs_stanza(
                verts, subdivision=DEFORM_SUBDIVISION
            )
            shapes = self.part_shapes.get(part) or []
            for shape in shapes:
                f = shape.getField("geometry")
                if f is None:
                    continue
                try:
                    f.importSFNodeFromString(stanza)
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(
                        f"[damage_tracker] mesh re-emit failed for {part}: {exc}\n"
                    )
            self.dirty_meshes[part] = False
            self.last_remit_ms[part] = sim_time_ms

    def _resolve_geometry_stanzas(self, part: str, state: str,
                                  shapes: list) -> list[str | None]:
        """Build a per-Shape geometry stanza list according to the
        Phase 10/14 resolution order. Returns one entry per shape,
        with None meaning "leave that shape's geometry unchanged."
        """
        # 1. Static mesh variant (Phase 10): same stanza for all shapes.
        variants = self.profile.mesh_variants.get(part) or {}
        url = variants.get(state)
        if url:
            return [f'Mesh {{ url [ "{url}" ] }}'] * len(shapes)

        # 2. Procedural deformation (Phase 14b + 15). Pulls from the
        # persistent vertex buffer; the buffer is mutated by Phase 14b's
        # additive state-transition crumple and Phase 15's per-impact
        # local dents. Only fires once damage has actually deformed the
        # part — pristine + zero accumulated displacement falls through
        # to the snapshot-restore branch.
        if (part in self.profile.procedural_deformation_parts
                and part in self.profile.crumple_size):
            target_crumple = CRUMPLE_PER_STATE.get(state, 0.0)
            current_crumple = self.current_crumple.get(part, 0.0)
            crumple_delta = target_crumple - current_crumple

            # Lazy-init buffer the first time we need to crumple this
            # part. Saves cost on parts that never take damage.
            if part not in self.part_vertices:
                self.part_vertices[part] = make_baseline_box_buffer(
                    self.profile.crumple_size[part],
                    subdivision=DEFORM_SUBDIVISION,
                )

            # Layer additive uniform crumple if state climbed (delta
            # negative on heal — handled by reset path, ignore here).
            if crumple_delta > 0.0:
                size = self.profile.crumple_size[part]
                dim_scale = min(size) * 0.5
                seed = abs(hash((part, state))) & 0xFFFF
                apply_uniform_random_dents(
                    self.part_vertices[part],
                    magnitude=crumple_delta * dim_scale,
                    seed=seed,
                    subdivision=DEFORM_SUBDIVISION,
                )
                self.current_crumple[part] = target_crumple

            # Re-emit immediately on transition (rate-limit is for
            # high-frequency per-impact dents, not state changes).
            if target_crumple > 0.0 or self.current_crumple.get(part, 0.0) > 0.0:
                stanza = vertex_buffer_to_ifs_stanza(
                    self.part_vertices[part], subdivision=DEFORM_SUBDIVISION
                )
                self.last_remit_ms[part] = 0  # force-clear rate limit
                self.dirty_meshes[part] = False
                return [stanza] * len(shapes)

        # 3. Pristine restore via captured Mesh URL (Phase 14b).
        if state == "pristine":
            urls = self.original_geometry_urls.get(part) or []
            if urls and any(u for u in urls):
                stanzas2: list[str | None] = []
                for u in urls:
                    if u:
                        stanzas2.append(f'Mesh {{ url [ "{u}" ] }}')
                    else:
                        stanzas2.append(None)
                return stanzas2

        # 4. No-op.
        return []

    def _apply_appearance(self, part: str, state: str) -> int:
        """Phase 7: rewrite every Shape under `part` to use the
        appearance variant for `state`. Returns the count of shapes
        actually updated. Failures are logged and counted as misses;
        we don't abort the whole transition over one stubborn Shape.
        """
        shapes = self.part_shapes.get(part) or []
        if not shapes:
            return 0
        stanza = _appearance_stanza(part, state, self.profile.wheel_parts)
        updated = 0
        for shape in shapes:
            f = shape.getField("appearance")
            if f is None:
                continue
            try:
                f.importSFNodeFromString(stanza)
                updated += 1
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[damage_tracker] appearance update failed for {part}: {exc}\n"
                )
        return updated

    def _live_appearance_field(self, part: str) -> str | None:
        """Return an `appearance <Node { ... }>` field string cloned from
        the part's first Shape, so a detached free body renders exactly
        like the part did while attached.

        The URDF importer assigns each Shape a real appearance node (e.g.
        the Husky wheel gets `PBRAppearance { baseColor 0.65 0.55 0.5
        roughness 0.5 ... }`, NOT the .dae's baked diffuse). A stanza-
        built Shape does not inherit that, so without cloning it the
        detached body uses whatever colour we hardcode — invariably
        darker than the importer's light-grey default, which reads as the
        part "turning black" on detach. exportString() on the live
        appearance node captures it verbatim, type and all.

        Returns None if the part has no readable appearance (caller then
        falls back to a profile colour).
        """
        for shape in self.part_shapes.get(part) or []:
            try:
                af = shape.getField("appearance")
                if af is None:
                    continue
                app = af.getSFNode()
                if app is None:
                    continue
                exported = app.exportString().strip()
            except Exception:
                continue
            if exported:
                return f"appearance {exported}"
        return None

    def _detach_part(self, part: str) -> None:
        """Phase 19 dispatcher: route a part-detach request to the
        wheel-specific cylinder path or the generic slab path based on
        whether the part is a wheel. Idempotent — second call for an
        already-detached part is a no-op.

        Disable hook: env var OMNISIM_DISABLE_DETACH=1 short-circuits
        all part detachment. Removing a node from a URDFRobot subtree
        mid-physics-step can leave dangling references in ODE's joint
        structure, occasionally triggering Webots binary access
        violations (0xC0000005) under sustained collision. Setting
        this env var keeps the chassis intact at the cost of the
        "wheels rolling away" / "bumper falls off" visual.
        """
        if os.environ.get("OMNISIM_DISABLE_DETACH") == "1":
            return
        if part in self.detached_parts:
            return
        if part in self.profile.wheel_parts:
            self._detach_wheel(part)
        else:
            self._detach_slab(part)

    def _detach_wheel(self, part: str) -> None:
        """Phase 9: physically remove the wheel from the URDFRobot tree
        and replace it with a free Solid at the same pose+velocity, so
        the broken wheel actually rolls/bounces away. Idempotent — safe
        to call multiple times if the same wheel re-enters broken state.

        Strategy:
          1. Capture the wheel Solid's world pose + linear/angular velocity.
          2. Spawn a free Solid (cylinder + Physics) at the captured state.
          3. Set its initial velocity to match.
          4. Remove the HingeJoint that holds the original wheel from the
             URDFRobot's children, deleting both the joint and its endPoint
             Solid in one node.remove() call.

        The remaining 3 wheels' joints and behaviours are untouched.
        """
        if part not in self.profile.wheel_parts or part in self.detached_parts:
            return
        solids = self.parts.get(part) or []
        if not solids:
            return
        wheel = solids[0]

        # 1. Capture pose + velocity. If any of these fail we abort the
        #    detachment rather than spawn a phantom wheel at the origin.
        try:
            pos = list(wheel.getPosition())
            rot = list(wheel.getOrientation())
            vel = list(wheel.getVelocity())
        except Exception as exc:
            sys.stderr.write(f"[damage_tracker] detach {part}: pose read failed: {exc}\n")
            return

        # Compute the outward direction (wheel relative to chassis), so
        # we can offset the spawn outboard of the hub and add a roll-
        # away kick. Without this the wheel reappears in-place and looks
        # static.
        try:
            chassis_pos = list(self.robot_node.getPosition())
        except Exception:
            chassis_pos = [pos[0], pos[1], pos[2]]
        out_x = pos[0] - chassis_pos[0]
        out_y = pos[1] - chassis_pos[1]
        out_mag = math.sqrt(out_x * out_x + out_y * out_y)
        if out_mag < 1e-6:
            # Wheel exactly at chassis center (shouldn't happen on a
            # Husky); fall back to +X so we don't divide by zero.
            out_x, out_y, out_mag = 1.0, 0.0, 1.0
        out_x /= out_mag
        out_y /= out_mag

        # Small spawn separation outboard so the wheel cylinder doesn't
        # interpenetrate the chassis hub at t=0; just enough clearance
        # for a clean physics start, not a dramatic offset.
        pos[0] += 0.06 * out_x
        pos[1] += 0.06 * out_y
        pos[2] += 0.02
        aa = _matrix_to_axis_angle(rot)

        # No kick. Real wheel hub fracture: the wheel inherits the
        # chassis's linear + angular velocity (already in `vel` from
        # getVelocity()) and is then governed by gravity, the floor, and
        # whatever residual spin the wheel motor had. If the husky was
        # driving when the wheel broke, that forward momentum carries
        # the wheel; if it was stopped, the wheel just falls off.

        # 2. Build the free wheel and import it into the world root.
        if self.root_children is None:
            return
        name = f"detached_{part}"
        # Visual: prefer the profile's wheel mesh URL so the detached
        # wheel matches the still-attached ones. Wrapped in a Pose with
        # a 90-deg X rotation so the .dae's native axis (Z, URDF
        # convention) aligns with Webots' Cylinder boundingObject axis
        # (Y). Profiles without a mesh URL fall back to a plain dark
        # Cylinder visual. Collision: keep the Cylinder boundingObject
        # in either case — mesh-based collisions are expensive.
        wh = self.profile.wheel_height
        wr = self.profile.wheel_radius
        wm = self.profile.wheel_mass
        # Clone the still-attached wheel's actual appearance so the free
        # wheel renders identically. Fall back to the profile colour (or
        # a rubber-grey default) only if the live appearance can't be
        # read — the importer's default is far lighter than any colour we
        # used to hardcode, which is why detached wheels looked black.
        wc = self.profile.detach_color.get(part, (0.2, 0.2, 0.2))
        app_field = self._live_appearance_field(part) or (
            f'appearance PBRAppearance {{ baseColor {wc[0]} {wc[1]} {wc[2]} '
            f'roughness 0.95 metalness 0.0 }}'
        )
        if self.profile.wheel_mesh_url:
            visual_block = (
                f'    Pose {{'
                f'      rotation 1 0 0 1.5707963'
                f'      children ['
                f'        Shape {{'
                f'          {app_field}'
                f'          geometry Mesh {{ url [ "{self.profile.wheel_mesh_url}" ] }}'
                f'        }}'
                f'      ]'
                f'    }}'
            )
        else:
            visual_block = (
                f'    Shape {{'
                f'      {app_field}'
                f'      geometry Cylinder {{ height {wh} radius {wr} }}'
                f'    }}'
            )
        stanza = (
            f'DEF {name} Solid {{'
            f'  translation {pos[0]} {pos[1]} {pos[2]}'
            f'  rotation {aa[0]} {aa[1]} {aa[2]} {aa[3]}'
            f'  name "{name}"'
            f'  children ['
            f'{visual_block}'
            f'  ]'
            f'  boundingObject Cylinder {{ height {wh} radius {wr} }}'
            f'  physics Physics {{ density -1 mass {wm} }}'
            f'}}'
        )
        try:
            self.root_children.importMFNodeFromString(-1, stanza)
        except Exception as exc:
            sys.stderr.write(f"[damage_tracker] detach {part}: spawn failed: {exc}\n")
            return

        # 3. Set the captured velocity on the new free body.
        new_node = self.supervisor.getFromDef(name)
        if new_node is not None:
            try:
                new_node.setVelocity(vel)
            except Exception as exc:
                sys.stderr.write(
                    f"[damage_tracker] detach {part}: setVelocity failed: {exc}\n"
                )

        # 4. Remove the HingeJoint that owned the original wheel. URDF
        #    `continuous` joints become Webots HingeJoints whose
        #    endPoint is the wheel Solid; removing the HingeJoint
        #    deletes the joint and its Solid endpoint in one call.
        try:
            parent = wheel.getParentNode()
        except Exception as exc:
            sys.stderr.write(
                f"[damage_tracker] detach {part}: getParentNode failed: {exc}\n"
            )
            parent = None
        if parent is not None:
            try:
                parent_type = parent.getTypeName()
            except Exception:
                parent_type = "?"
            try:
                parent.remove()
                sys.stderr.write(
                    f"[damage_tracker] detached {part}: removed parent {parent_type!r}\n"
                )
            except Exception as exc:
                sys.stderr.write(
                    f"[damage_tracker] detach {part}: parent.remove() failed: {exc}; "
                    f"free wheel still spawned\n"
                )

        self.detached_parts.add(part)
        self.detached_part_defs.append(name)
        sys.stderr.write(
            f"[damage_tracker] detached {part} as {name} at "
            f"({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})\n"
        )

    def _detach_slab(self, part: str) -> None:
        """Phase 19: physically detach a non-wheel slab-shaped part
        (bumper, top_plate, panel) from the URDFRobot. Spawns a free
        Box-collision Solid at the part's current pose+velocity, kicks
        it outward + upward + adds tumble, then attempts to remove the
        original Solid from the URDFRobot tree. If subtree removal
        fails (which is documented as risky), the spawned free body
        still ships and reads as "the part fell off"; the original
        stays in place but its appearance is already darkened by the
        Phase 7 broken-state pass.

        Idempotent — guarded by `detached_parts` membership.
        """
        if part in self.detached_parts:
            return
        solids = self.parts.get(part) or []
        if not solids:
            return
        slab = solids[0]

        # 1. Capture pose + velocity. Abort if any read fails so we
        #    don't spawn a phantom slab at the origin.
        try:
            pos = list(slab.getPosition())
            rot = list(slab.getOrientation())
            vel = list(slab.getVelocity())
        except Exception as exc:
            sys.stderr.write(f"[damage_tracker] detach {part}: pose read failed: {exc}\n")
            return

        # Outward direction = vector from chassis center to slab center.
        # For the front_bumper this points forward; for top_plate it
        # points up. We push the slab along this vector so it visibly
        # leaves the chassis instead of dropping in place.
        try:
            chassis_pos = list(self.robot_node.getPosition())
        except Exception:
            chassis_pos = [pos[0], pos[1], pos[2]]
        out_x = pos[0] - chassis_pos[0]
        out_y = pos[1] - chassis_pos[1]
        out_z = pos[2] - chassis_pos[2]
        out_mag = math.sqrt(out_x * out_x + out_y * out_y + out_z * out_z)
        if out_mag < 1e-6:
            # Slab co-located with chassis center — push straight up.
            out_x, out_y, out_z, out_mag = 0.0, 0.0, 1.0, 1.0
        out_x /= out_mag
        out_y /= out_mag
        out_z /= out_mag

        # Small spawn separation along the outward direction so the new
        # slab doesn't interpenetrate the chassis at t=0; otherwise ODE
        # immediately resolves the overlap with a violent ejection.
        # Just enough clearance for a clean physics start, no more.
        pos[0] += 0.04 * out_x
        pos[1] += 0.04 * out_y
        pos[2] += max(0.02, 0.04 * out_z)
        aa = _matrix_to_axis_angle(rot)

        # No kick. The realistic detach for a fastener-shear failure is
        # the part inheriting whatever velocity the chassis had and
        # falling under gravity. `vel` already carries the chassis-side
        # linear+angular velocity from getVelocity(), so we leave it
        # untouched. If the chassis was driving when it broke, the slab
        # will keep that forward momentum naturally.

        # Sizing: prefer profile.crumple_size (already authored for the
        # procedural deformation pass), fall back to a generic small
        # slab. Mass from profile.detach_mass with a profile default.
        size = self.profile.crumple_size.get(part) or (0.3, 0.3, 0.05)
        sx, sy, sz = size
        mass = (
            self.profile.detach_mass.get(part)
            or getattr(self.profile, "default_detach_mass", 3.0)
        )

        if self.root_children is None:
            return
        name = f"detached_{part}"

        # Visual: prefer the profile's per-part mesh + colour so the
        # detached body looks like the part that just fell off. Without
        # this configured the user sees a generic dark Box pop into
        # being where the bumper used to be — reads as a spawn, not a
        # detachment. Bounding object stays a coarse Box so collision
        # cost is bounded (mesh-based collision on detached pieces is
        # expensive and not visually necessary).
        mesh_url = self.profile.detach_mesh_url.get(part)
        color = self.profile.detach_color.get(part, (0.18, 0.12, 0.10))
        # Clone the part's live appearance so the free slab matches the
        # robot; the profile colour is only a fallback (see
        # _live_appearance_field — the importer's default is lighter than
        # the .dae-diffuse-derived colours we used to hardcode).
        app_field_mesh = self._live_appearance_field(part) or (
            f'appearance PBRAppearance {{ baseColor {color[0]} {color[1]} '
            f'{color[2]} roughness 0.7 metalness 0.05 }}'
        )
        app_field_box = self._live_appearance_field(part) or (
            f'appearance PBRAppearance {{ baseColor {color[0]} {color[1]} '
            f'{color[2]} roughness 0.92 metalness 0.05 }}'
        )
        if mesh_url:
            visual_block = (
                f'    Shape {{'
                f'      {app_field_mesh}'
                f'      geometry Mesh {{ url [ "{mesh_url}" ] }}'
                f'    }}'
            )
        else:
            visual_block = (
                f'    Shape {{'
                f'      {app_field_box}'
                f'      geometry Box {{ size {sx} {sy} {sz} }}'
                f'    }}'
            )
        stanza = (
            f'DEF {name} Solid {{'
            f'  translation {pos[0]} {pos[1]} {pos[2]}'
            f'  rotation {aa[0]} {aa[1]} {aa[2]} {aa[3]}'
            f'  name "{name}"'
            f'  children ['
            f'{visual_block}'
            f'  ]'
            f'  boundingObject Box {{ size {sx} {sy} {sz} }}'
            f'  physics Physics {{ density -1 mass {mass} }}'
            f'}}'
        )
        try:
            self.root_children.importMFNodeFromString(-1, stanza)
        except Exception as exc:
            sys.stderr.write(f"[damage_tracker] detach {part}: spawn failed: {exc}\n")
            return

        new_node = self.supervisor.getFromDef(name)
        if new_node is not None:
            try:
                new_node.setVelocity(vel)
            except Exception as exc:
                sys.stderr.write(
                    f"[damage_tracker] detach {part}: setVelocity failed: {exc}\n"
                )

        # Try to remove the original Solid. URDF fixed-joint children
        # sit directly inside the parent's children field, so a
        # `solid.remove()` should suffice. If it fails the spawned free
        # body still reads as the visible "what fell off" — log and
        # continue rather than abort.
        try:
            slab.remove()
            removed = True
        except Exception as exc:
            sys.stderr.write(
                f"[damage_tracker] detach {part}: solid.remove() failed: {exc}; "
                f"original stays attached, free body still spawned\n"
            )
            removed = False

        self.detached_parts.add(part)
        self.detached_part_defs.append(name)
        sys.stderr.write(
            f"[damage_tracker] detached {part} as {name} at "
            f"({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
            f"original_removed={removed}\n"
        )

    def _apply_heal(self, sim_time_ms: int) -> None:
        """Phase 18a/18b: apply per-part HP and mesh-vertex regen each
        poll() step. Healing crosses state bands upward (e.g.
        damaged → scuffed → pristine) and emits state_transition events
        on the same buffer as damage transitions, distinguishable by
        from_state having lower HP fraction than to_state.
        """
        if self._last_heal_tick_ms < 0:
            self._last_heal_tick_ms = sim_time_ms
            return
        dt_ms = sim_time_ms - self._last_heal_tick_ms
        if dt_ms <= 0:
            return
        self._last_heal_tick_ms = sim_time_ms
        dt_s = dt_ms / 1000.0

        # Pass 1: HP regen + state transitions
        for part, rate in list(self.heal_rate_hp_per_s.items()):
            if rate <= 0.0:
                continue
            record = self.part_hp.get(part)
            if record is None or record["hp"] >= record["hp_max"]:
                continue
            old_state = record["state"]
            new_hp = min(record["hp_max"], record["hp"] + rate * dt_s)
            record["hp"] = new_hp
            new_state = self._state_for_hp(new_hp, record["hp_max"])
            if new_state != old_state:
                record["state"] = new_state
                # Healing transitions feed the same event stream as
                # damage transitions; consumers distinguish by checking
                # the band order (heal: from worse to better).
                self._emit_transition(sim_time_ms, part, old_state, new_state,
                                      new_hp, 0.0)
                # Phase 5: re-publish the behaviour gate so a driving
                # controller picks up restored wheel torque.
                if part in self.profile.wheel_parts or part == "chassis":
                    self._write_behavior_gate()

        # Pass 2: vertex buffer regen
        for part, rate in list(self.heal_rate_mesh_m_per_s.items()):
            if rate <= 0.0:
                continue
            verts = self.part_vertices.get(part)
            size = self.profile.crumple_size.get(part)
            if verts is None or size is None:
                continue
            baseline = make_baseline_box_buffer(
                size, subdivision=DEFORM_SUBDIVISION
            )
            step_m = rate * dt_s
            moved = False
            n = len(verts)
            for i in range(0, n, 3):
                dx = baseline[i] - verts[i]
                dy = baseline[i + 1] - verts[i + 1]
                dz = baseline[i + 2] - verts[i + 2]
                d_sq = dx * dx + dy * dy + dz * dz
                if d_sq < HEAL_MESH_OVERSHOOT_GUARD_M ** 2:
                    # Already at baseline — snap to avoid drift.
                    if d_sq > 0:
                        verts[i] = baseline[i]
                        verts[i + 1] = baseline[i + 1]
                        verts[i + 2] = baseline[i + 2]
                        moved = True
                    continue
                d = d_sq ** 0.5
                # Migrate by min(step_m, d) so we don't overshoot.
                t = min(1.0, step_m / d)
                verts[i] += dx * t
                verts[i + 1] += dy * t
                verts[i + 2] += dz * t
                moved = True
            if moved:
                self.dirty_meshes[part] = True
                # Heal the current_crumple counter proportionally so
                # subsequent state transitions apply correct deltas.
                if self.current_crumple.get(part, 0.0) > 0.0:
                    self.current_crumple[part] = max(
                        0.0, self.current_crumple[part] - 0.05 * dt_s
                    )

    def _maybe_spawn_fragments(self, part: str, sim_time_ms: int) -> int:
        """Phase 17c entry point. Scan the part's vertex buffer for
        strained regions, group connected ones into fracture islands,
        spawn each as a free-body fragment, and collapse the island
        vertices in the chassis IFS so the rendered chassis develops a
        torn-open look. Returns the count of fragments spawned this
        call (typically 0; >0 means the impulse just past plastic
        yield somewhere on the part).

        Disable hook: env var OMNISIM_DISABLE_FRACTURE=1 short-circuits
        this method entirely. Useful when fragment spawning under
        sustained heavy collision (multi-robot head-on with both
        sides mutating their URDF subtrees on the same physics step)
        triggers Webots binary crashes — observed in Phase 21 testing.
        Less risky scene mutation = more stable demo at the cost of
        the "torn-open chassis" visual.
        """
        if os.environ.get("OMNISIM_DISABLE_FRACTURE") == "1":
            return 0
        # Sustained fracture spawning during head-on collisions has been
        # a Webots-crash signature (see _apply_geometry note); the same
        # gate that quiets the procedural deformation needs to quiet
        # this too, so --light truly is light.
        if os.environ.get("OMNISIM_LITE_DAMAGE") == "1":
            return 0
        if part not in self.profile.procedural_deformation_parts:
            return 0
        size = self.profile.crumple_size.get(part)
        if size is None:
            return 0
        verts = self.part_vertices.get(part)
        if verts is None or self._neighbor_table is None:
            return 0
        baseline = make_baseline_box_buffer(size, subdivision=DEFORM_SUBDIVISION)
        strained, _max = find_strained_vertices(
            verts, baseline, FRACTURE_STRAIN_M
        )
        if not strained:
            return 0
        islands = find_fracture_islands(
            strained, self._neighbor_table, min_size=FRACTURE_MIN_VERTICES
        )
        if not islands:
            return 0
        # Cap alive fragments per part: once we hit FRAGMENTS_PER_PART_CAP,
        # additional islands stay yielded but don't tear off. Prevents
        # ODE collision-pair growth in long heal/damage loops.
        alive_for_part = sum(
            1 for n in self.fragments_spawned
            if n.startswith(f"damage_fragment_")  # crude but matches pattern
        )
        if alive_for_part >= FRAGMENTS_PER_PART_CAP:
            return 0
        spawned = 0
        for island in islands:
            if alive_for_part + spawned >= FRAGMENTS_PER_PART_CAP:
                break
            if self._spawn_fragment(part, island, sim_time_ms):
                # Collapse vertices in the chassis buffer so the rendered
                # chassis shows the hole. Mark dirty so the IFS re-emit
                # picks up the change.
                collapse_island_to_neighbors(verts, island, self._neighbor_table)
                self.dirty_meshes[part] = True
                spawned += 1
        return spawned

    def _spawn_fragment(self, part: str, island: list[int],
                        sim_time_ms: int) -> bool:
        """Spawn a free-body Solid for one fracture island. Geometry
        is built from the island's current vertex positions in the
        part's local frame (via fragment_ifs_stanza), translated to
        the island's centroid in world coords. Initial velocity =
        chassis body velocity + outward kick + small random spin so
        the chunk reads as physically separating, not teleporting.

        Returns True on success; False if any prerequisite is missing.
        """
        if self.root_children is None:
            return False
        solids = self.parts.get(part) or []
        if not solids:
            return False
        parent = solids[0]
        try:
            ppos = list(parent.getPosition())
            prot = list(parent.getOrientation())
        except Exception:
            return False
        verts = self.part_vertices.get(part)
        if verts is None:
            return False

        ifs_stanza, centroid_local = fragment_ifs_stanza(
            verts, island, subdivision=DEFORM_SUBDIVISION
        )
        # Transform island centroid from part-local frame into world.
        R = prot
        cx, cy, cz = centroid_local
        wx = ppos[0] + R[0] * cx + R[1] * cy + R[2] * cz
        wy = ppos[1] + R[3] * cx + R[4] * cy + R[5] * cz
        wz = ppos[2] + R[6] * cx + R[7] * cy + R[8] * cz

        # Outward direction in world frame: from chassis center toward
        # the island centroid. Drives the separation kick.
        ox, oy, oz = wx - ppos[0], wy - ppos[1], wz - ppos[2]
        omag = math.sqrt(ox * ox + oy * oy + oz * oz)
        if omag > 1e-6:
            ox /= omag
            oy /= omag
            oz /= omag
        else:
            ox, oy, oz = 0.0, 0.0, 1.0

        self._fragment_counter += 1
        name = f"damage_fragment_{self._fragment_counter:04d}"
        # Mass scales with island size; per-vertex 30g is a rough
        # approximation that keeps fragments lightweight enough to fly
        # but heavy enough to interact with other rigid bodies.
        mass = max(0.05, 0.03 * len(island))
        # Bounding box estimate: pick a Box covering the island's
        # extent. Each face cell is ~size/subdivision wide; an island
        # of N verts spans roughly sqrt(N) cells.
        cell = min(self.profile.crumple_size[part]) / DEFORM_SUBDIVISION
        bbox = max(0.06, cell * (len(island) ** 0.5))
        rng = self._debris_rng
        full_stanza = (
            f'DEF {name} Solid {{'
            f'  translation {wx} {wy} {wz}'
            f'  name "{name}"'
            f'  children ['
            f'    Shape {{'
            f'      appearance PBRAppearance {{'
            f'        baseColor 0.32 0.28 0.22'
            f'        roughness 0.85'
            f'        metalness 0.45'
            f'      }}'
            f'      geometry {ifs_stanza}'
            f'    }}'
            f'  ]'
            f'  boundingObject Box {{ size {bbox} {bbox} {bbox} }}'
            f'  physics Physics {{ density -1 mass {mass} }}'
            f'}}'
        )
        try:
            self.root_children.importMFNodeFromString(-1, full_stanza)
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(
                f"[damage_tracker] fragment spawn failed for {part}: {exc}\n"
            )
            return False

        # Initial velocity = chassis velocity + outward kick + small
        # vertical lift so the fragment arcs visibly. Random tumbling
        # spin makes it read as a torn-off piece, not a static drop.
        new_node = self.supervisor.getFromDef(name)
        if new_node is not None:
            try:
                chassis_v = _node_velocity(self.robot_node)
                kick = 1.5
                lin = [chassis_v[0] + kick * ox,
                       chassis_v[1] + kick * oy,
                       chassis_v[2] + kick * oz + 0.4]
                ang = [rng.uniform(-3.5, 3.5) for _ in range(3)]
                new_node.setVelocity([*lin, *ang])
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[damage_tracker] fragment velocity set failed: {exc}\n"
                )

        self.fragments_spawned.append(name)
        sys.stderr.write(
            f"[damage_tracker] FRACTURE {part}: spawned {name} "
            f"from {len(island)} verts at "
            f"({wx:.2f},{wy:.2f},{wz:.2f})\n"
        )
        return True

    def _apply_impact_dent(self, part: str, world_point: list[float],
                           impulse_J: float, sim_time_ms: int = 0,
                           normal_hint: tuple[float, float, float] | None = None
                           ) -> None:
        """Phase 15b: push the chassis (or other procedural-deformation
        part) mesh inward at the local-frame projection of `world_point`.
        Magnitude scales with impulse, falloff with distance from the
        contact. Mutates the persistent vertex buffer; marks the part
        dirty so the rate-limited sweep in poll() re-emits the IFS.

        Phase 16c: caller can pass `normal_hint` (in part-local frame)
        to override the radial approximation. Synthetic chassis impacts
        use -normalize(chassis_dv) — a real momentum-conservation
        normal — instead of the radial heuristic. Contact-loop impacts
        keep the radial fallback since the other body's identity isn't
        exposed by Webots' getContactPoints.
        """
        if part not in self.profile.procedural_deformation_parts:
            return
        if part not in self.profile.crumple_size:
            return
        solids = self.parts.get(part)
        if not solids:
            return
        parent = solids[0]
        try:
            parent_pos = list(parent.getPosition())
            parent_rot = list(parent.getOrientation())
        except Exception:
            return
        local = _world_to_local(world_point, parent_pos, parent_rot)
        # Lazy-init buffer if Phase 14b transition hasn't run yet.
        if part not in self.part_vertices:
            self.part_vertices[part] = make_baseline_box_buffer(
                self.profile.crumple_size[part],
                subdivision=DEFORM_SUBDIVISION,
            )
        # Lazy-build the shared neighbor tables on first dent.
        if self._neighbor_table is None:
            self._neighbor_table = build_neighbor_table(
                subdivision=DEFORM_SUBDIVISION
            )
        if self._relax_neighbor_table is None:
            self._relax_neighbor_table = build_neighbor_table(
                subdivision=DEFORM_SUBDIVISION, pin_boundary=True
            )
        # Inward normal: prefer the caller's hint (Phase 16c — derived
        # from chassis velocity delta when available), fall back to the
        # radial approximation (Phase 15) when not.
        if normal_hint is not None:
            # Hint is given in WORLD frame; rotate into part-local using
            # the part Solid's orientation matrix transpose. Same trick
            # _world_to_local uses for translations, applied to a
            # direction vector (no centering subtraction).
            R = parent_rot
            hx, hy, hz = normal_hint
            nx = R[0] * hx + R[3] * hy + R[6] * hz
            ny = R[1] * hx + R[4] * hy + R[7] * hz
            nz = R[2] * hx + R[5] * hy + R[8] * hz
            mag = math.sqrt(nx * nx + ny * ny + nz * nz)
            if mag > 1e-6:
                normal = (nx / mag, ny / mag, nz / mag)
            else:
                normal = (0.0, 0.0, -1.0)
        else:
            d = math.sqrt(local[0] * local[0] + local[1] * local[1]
                          + local[2] * local[2])
            if d < 1e-6:
                normal = (0.0, 0.0, -1.0)
            else:
                normal = (-local[0] / d, -local[1] / d, -local[2] / d)
        magnitude = min(DEFORM_MAG_CAP, DEFORM_MAG_PER_J * float(impulse_J))
        apply_local_dent(
            self.part_vertices[part],
            (local[0], local[1], local[2]),
            normal, magnitude, DEFORM_RADIUS,
            neighbors=self._neighbor_table,
            coupling=DEFORM_COUPLING,
        )
        self.dirty_meshes[part] = True
        # Tracked separately from dirty_meshes (which clears as soon as
        # the IFS is re-emitted): the relaxation pass keeps running for
        # RELAX_AFTER_MS regardless of re-emit cadence.
        self._last_dent_at_ms[part] = int(sim_time_ms)
        # Phase 17c: after each dent, check if accumulated strain has
        # crossed the plastic-yield threshold anywhere on this part.
        # Most dents add a small amount of strain that doesn't fracture;
        # this only spawns a fragment when a connected island of
        # yielded vertices exists.
        self._maybe_spawn_fragments(part, int(sim_time_ms))

    def _spawn_dent(self, part: str, world_point: list[float]) -> None:
        """Phase 14c: attach a small dented-Box overlay to the impacted
        part at the local-frame projection of `world_point`. Visual-
        only (no boundingObject, no physics), random tilt per dent so
        a cluster of strikes builds a "covered in dings" look.
        Capped per-part; FIFO eviction.
        """
        if os.environ.get("OMNISIM_LITE_DAMAGE") == "1":
            return
        solids = self.parts.get(part)
        if not solids:
            return
        parent = solids[0]
        try:
            parent_pos = list(parent.getPosition())
            parent_rot = list(parent.getOrientation())
        except Exception:
            return
        local = _world_to_local(world_point, parent_pos, parent_rot)
        # Synthetic chassis impacts feed in the URDFRobot center, so
        # without jitter every dent stacks at the same spot. Spread
        # them across the chassis dimensions like decals.
        rng = self._debris_rng
        if part == "chassis":
            local[0] += rng.uniform(-0.40, 0.40)
            local[1] += rng.uniform(-0.22, 0.22)
            local[2] += rng.uniform(0.18, 0.30)

        children = parent.getField("children")
        if children is None:
            return
        self._dent_counter += 1
        name = f"damage_dent_{self._dent_counter:05d}"
        sx, sy, sz = DENT_SIZE
        c = DENT_COLOR
        # Random tilt: small rotation about a random axis. Tilts give
        # the dents an organic "bent panel" feel rather than rigid
        # blocks aligned to the chassis frame.
        ax = rng.uniform(-1.0, 1.0)
        ay = rng.uniform(-1.0, 1.0)
        az = rng.uniform(-1.0, 1.0)
        amag = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        ax, ay, az = ax / amag, ay / amag, az / amag
        angle = rng.uniform(*DENT_TILT_RANGE)
        stanza = (
            f'DEF {name} Solid {{'
            f'  translation {local[0]} {local[1]} {local[2]}'
            f'  rotation {ax} {ay} {az} {angle}'
            f'  name "{name}"'
            f'  children ['
            f'    Shape {{'
            f'      appearance PBRAppearance {{'
            f'        baseColor {c[0]} {c[1]} {c[2]}'
            f'        roughness 0.95'
            f'        metalness 0.6'
            f'      }}'
            f'      geometry Box {{ size {sx} {sy} {sz} }}'
            f'    }}'
            f'  ]'
            f'}}'
        )
        try:
            children.importMFNodeFromString(-1, stanza)
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f"[damage_tracker] dent spawn failed for {part}: {exc}\n")
            return
        bucket = self.part_dents.setdefault(part, [])
        bucket.append((name, parent))
        while len(bucket) > DENT_PER_PART_CAP:
            old_name, _old_parent = bucket.pop(0)
            old = self.supervisor.getFromDef(old_name)
            if old is not None:
                try:
                    old.remove()
                except Exception:  # noqa: BLE001
                    pass

    def _spawn_decal(self, part: str, world_point: list[float]) -> None:
        """Phase 8: attach a small dark mark to the impacted part at
        the local-frame projection of `world_point`. Decals are visual-
        only Solids (no boundingObject, no physics) imported into the
        parent Solid's children field, so they translate and rotate
        with the part. Capped per-part; oldest is removed when over cap.
        """
        if os.environ.get("OMNISIM_LITE_DAMAGE") == "1":
            return
        self._decal_attempts += 1
        solids = self.parts.get(part)
        if not solids:
            return
        # Attach to the first Solid bucketed under this part. For
        # multi-Solid parts (chassis has 5), this is good enough — they
        # all share the same fused rigid body and move together.
        parent = solids[0]
        try:
            parent_pos = list(parent.getPosition())
            parent_rot = list(parent.getOrientation())
        except Exception as exc:
            sys.stderr.write(f"[damage_tracker] decal: pose read failed for {part}: {exc}\n")
            return
        local = _world_to_local(world_point, parent_pos, parent_rot)
        # Synthetic chassis impacts feed in the URDFRobot's center as the
        # contact point, so every chassis decal would otherwise stack at
        # (0,0,0) in local frame. Spread them out across roughly the
        # chassis dimensions so the marks read as accumulating across
        # the body.
        if part == "chassis":
            jitter = self._debris_rng
            local[0] += jitter.uniform(-0.45, 0.45)
            local[1] += jitter.uniform(-0.25, 0.25)
            local[2] += jitter.uniform(0.15, 0.30)  # bias upward onto the deck

        children = parent.getField("children")
        if children is None:
            sys.stderr.write(
                f"[damage_tracker] decal: parent {_solid_name(parent)!r} has no children field "
                f"for {part}\n"
            )
            return

        self._decal_counter += 1
        name = f"damage_decal_{self._decal_counter:05d}"
        # Tiny dark sphere — symmetric so we don't have to compute a
        # surface-aligned rotation. Reads as a scuff/burn mark at the
        # default arena viewpoint.
        radius = DECAL_SIZE * 0.6
        c = DECAL_COLOR
        stanza = (
            f'DEF {name} Solid {{'
            f'  translation {local[0]} {local[1]} {local[2]}'
            f'  name "{name}"'
            f'  children ['
            f'    Shape {{'
            f'      appearance PBRAppearance {{'
            f'        baseColor {c[0]} {c[1]} {c[2]}'
            f'        roughness 0.95'
            f'        metalness 0.0'
            f'      }}'
            f'      geometry Sphere {{ radius {radius} subdivision 1 }}'
            f'    }}'
            f'  ]'
            f'}}'
        )
        try:
            children.importMFNodeFromString(-1, stanza)
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f"[damage_tracker] decal spawn failed for {part}: {exc}\n")
            return

        bucket = self.part_decals.setdefault(part, [])
        bucket.append((name, parent))
        # Enforce per-part cap: drop the oldest when over.
        while len(bucket) > DECAL_PER_PART_CAP:
            old_name, old_parent = bucket.pop(0)
            old = self.supervisor.getFromDef(old_name)
            if old is not None:
                try:
                    old.remove()
                except Exception:  # noqa: BLE001
                    pass

    def _spawn_particle(self, name: str, stanza: str, expire_at_ms: int) -> None:
        """Common particle spawn helper. Imports `stanza` into root
        children, registers it with an expiry time. Failures are
        logged once and swallowed; one bad particle shouldn't break
        the poll loop.
        """
        if self.root_children is None:
            return
        try:
            self.root_children.importMFNodeFromString(-1, stanza)
            self.particles.append((name, int(expire_at_ms)))
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(f"[damage_tracker] particle {name} spawn failed: {exc}\n")

    def _read_use_pool_flag(self, supervisor) -> bool:
        """Read the "use_particle_pool" boolean from the supervisor's
        own customData JSON. Returns False if the field is missing or
        the JSON is malformed (Webots' SFString parser is fragile).
        """
        try:
            self_node = supervisor.getSelf()
            if self_node is None:
                return False
            f = self_node.getField("customData")
            if f is None:
                return False
            raw = (f.getSFString() or "").strip()
            if not raw:
                return False
            cfg = json.loads(raw)
            return bool(cfg.get("use_particle_pool", False)) if isinstance(cfg, dict) else False
        except Exception:  # noqa: BLE001
            return False

    def _pool_send(self, cmd: str, args: dict, sim_time_ms: int = 0) -> bool:
        """Fire-and-forget send to the cuda_particle_pool on :6791.
        Returns True on successful TCP send (caller skips the legacy
        path), False on failure.

        Uses the pool's `spawn_oneway` command which is no-reply, so
        damage_tracker never blocks on the pool's main-loop response.
        That sidesteps the boot-order race where the pool's response
        latency under collision load (sim running at <5% real-time)
        exceeded the recv timeout.

        Backoff: on send failure we close the socket and refuse to
        reconnect until `_pool_next_retry_ms` has passed. Cooldown
        starts at 1 s, doubles per failure up to 30 s.
        """
        if sim_time_ms < self._pool_next_retry_ms:
            return False
        try:
            if self._pool_sock is None:
                self._pool_sock = socket.create_connection(
                    ("127.0.0.1", 6791), timeout=0.2)
                # Short send timeout so a wedged pool doesn't slow
                # damage_tracker. No recv path means receive timeout
                # is irrelevant.
                self._pool_sock.settimeout(0.1)
            payload = json.dumps({"id": 0, "cmd": cmd, "args": args}).encode()
            self._pool_sock.sendall(struct.pack(">I", len(payload)) + payload)
            self._pool_spawn_count += 1
            self._pool_retry_cooldown_ms = 1000  # reset on success
            return True
        except (ConnectionError, OSError, socket.timeout):
            try:
                if self._pool_sock is not None:
                    self._pool_sock.close()
            except Exception:  # noqa: BLE001
                pass
            self._pool_sock = None
            self._pool_fail_count += 1
            self._pool_next_retry_ms = sim_time_ms + self._pool_retry_cooldown_ms
            self._pool_retry_cooldown_ms = min(
                30000, self._pool_retry_cooldown_ms * 2)
            return False

    def _spawn_smoke_puff(self, point: list[float], sim_time_ms: int) -> None:
        """Phase 11: brief gray smoke sphere at a chassis impact site.
        Visual-only (no Physics) — applies a hard-coded upward velocity
        via Node.setVelocity after import so the puff drifts up rather
        than stays in place.
        """
        if os.environ.get("OMNISIM_LITE_DAMAGE") == "1":
            return
        if sim_time_ms - self._last_smoke_at < SMOKE_RATE_LIMIT_MS:
            return
        self._last_smoke_at = sim_time_ms
        # Phase 1.5: try the CUDA particle pool first. If the pool is
        # in the world it absorbs the spawn at ~310 us / particle (vs
        # ~265 ms / particle for the legacy importMFNodeFromString path
        # under collision load). _pool_send returns False if the pool
        # isn't running; we then fall through to the legacy path so
        # worlds without the pool keep working unchanged.
        rng = self._debris_rng
        if self._pool_send("spawn_oneway", {
            "pos": [float(point[0]), float(point[1]), float(point[2]) + 0.20],
            "vel": [rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3),
                    rng.uniform(0.4, 0.8)],
            "life": SMOKE_LIFETIME_MS / 1000.0,
            "size": rng.uniform(*SMOKE_SIZE_RANGE),
        }, sim_time_ms=sim_time_ms):
            return
        if self._suppress_legacy_spawn:
            # World wired the pool in; pool send failed for now (still
            # cooling down or the connect raced). Drop this spawn rather
            # than fall back to importMFNodeFromString, which would
            # flood the VRML parser and slow the sim enough to keep
            # the pool's main loop starved -- a deadlock-by-starvation.
            return
        self._particle_counter += 1
        name = f"damage_smoke_{self._particle_counter:05d}"
        radius = rng.uniform(*SMOKE_SIZE_RANGE)
        color = rng.choice(SMOKE_COLORS)
        # Spawn slightly above the contact point so it reads as rising.
        z = float(point[2]) + 0.20
        # Visual-only: no boundingObject, no Physics. Means setVelocity
        # has no rigid body to operate on, so we rely on simulation time
        # advancing the supervisor moving these things isn't an option
        # without physics. Instead the puff stays put and just fades by
        # being removed at expiry — which reads OK at the demo viewpoint.
        stanza = (
            f'DEF {name} Solid {{'
            f'  translation {point[0]} {point[1]} {z}'
            f'  name "{name}"'
            f'  children ['
            f'    Shape {{'
            f'      appearance PBRAppearance {{'
            f'        baseColor {color[0]} {color[1]} {color[2]}'
            f'        roughness 1.0'
            f'        metalness 0.0'
            f'        transparency 0.45'
            f'      }}'
            f'      geometry Sphere {{ radius {radius} subdivision 1 }}'
            f'    }}'
            f'  ]'
            f'}}'
        )
        self._spawn_particle(name, stanza, sim_time_ms + SMOKE_LIFETIME_MS)

    def _spawn_spark(self, point: list[float], sim_time_ms: int) -> None:
        """Phase 11: tiny bright sphere at a wheel impact site. Very
        short lifetime so cadence can be high without clogging the
        scene.
        """
        if os.environ.get("OMNISIM_LITE_DAMAGE") == "1":
            return
        if sim_time_ms - self._last_spark_at < SPARK_RATE_LIMIT_MS:
            return
        self._last_spark_at = sim_time_ms
        # Phase 1.5: try the CUDA particle pool first. Sparks burst
        # outward + upward with random horizontal velocity so they
        # arc visibly. Falls back to importMFNodeFromString if the
        # pool isn't in the world.
        rng = self._debris_rng
        if self._pool_send("spawn_oneway", {
            "pos": [float(point[0]), float(point[1]), float(point[2]) + 0.05],
            "vel": [rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0),
                    rng.uniform(1.5, 3.5)],
            "life": SPARK_LIFETIME_MS / 1000.0,
            "size": rng.uniform(*SPARK_SIZE_RANGE),
        }, sim_time_ms=sim_time_ms):
            return
        if self._suppress_legacy_spawn:
            return
        self._particle_counter += 1
        name = f"damage_spark_{self._particle_counter:05d}"
        radius = rng.uniform(*SPARK_SIZE_RANGE)
        color = rng.choice(SPARK_COLORS)
        z = float(point[2]) + 0.05
        stanza = (
            f'DEF {name} Solid {{'
            f'  translation {point[0]} {point[1]} {z}'
            f'  name "{name}"'
            f'  children ['
            f'    Shape {{'
            f'      appearance PBRAppearance {{'
            f'        baseColor {color[0]} {color[1]} {color[2]}'
            f'        emissiveColor {color[0]} {color[1]} {color[2]*0.6}'
            f'        roughness 0.2'
            f'        metalness 0.5'
            f'      }}'
            f'      geometry Sphere {{ radius {radius} subdivision 1 }}'
            f'    }}'
            f'  ]'
            f'}}'
        )
        self._spawn_particle(name, stanza, sim_time_ms + SPARK_LIFETIME_MS)

    def _spawn_fluid_drop(self, sim_time_ms: int) -> None:
        """Phase 11: small dark stain placed on the floor under the
        chassis. Visual-only (no Physics) so we don't blow ODE's
        contact-joint budget — earlier physics-enabled drops piled up
        and triggered "Contact joints will only be created for the 10
        deepest contact points" warnings within seconds.
        """
        if sim_time_ms - self._last_fluid_at < FLUID_LEAK_INTERVAL_MS:
            return
        chassis = self.part_hp.get("chassis")
        if chassis is None or chassis["state"] not in ("damaged", "broken"):
            return
        if self.robot_node is None:
            return
        try:
            cpos = list(self.robot_node.getPosition())
        except Exception:
            return
        self._last_fluid_at = sim_time_ms
        self._particle_counter += 1
        name = f"damage_fluid_{self._particle_counter:05d}"
        rng = self._debris_rng
        # Stain pinned just above the floor; jittered around the chassis
        # XY footprint so the puddle widens over time. No Physics, no
        # boundingObject.
        ox = cpos[0] + rng.uniform(-0.30, 0.30)
        oy = cpos[1] + rng.uniform(-0.22, 0.22)
        oz = 0.01  # right above the floor; visual-only
        color = (FLUID_COLOR_BROKEN if chassis["state"] == "broken"
                 else FLUID_COLOR_DAMAGED)
        # Render as a thin flat box (a "splat") rather than a sphere —
        # spheres half-buried under the floor look weird, splats read
        # as a stain.
        stanza = (
            f'DEF {name} Solid {{'
            f'  translation {ox} {oy} {oz}'
            f'  name "{name}"'
            f'  children ['
            f'    Shape {{'
            f'      appearance PBRAppearance {{'
            f'        baseColor {color[0]} {color[1]} {color[2]}'
            f'        roughness 0.6'
            f'        metalness 0.0'
            f'      }}'
            f'      geometry Box {{ size 0.18 0.14 0.012 }}'
            f'    }}'
            f'  ]'
            f'}}'
        )
        self._spawn_particle(name, stanza, sim_time_ms + FLUID_LIFETIME_MS)

    def _spawn_debris_burst(self, point: list[float]) -> None:
        """Phase 6: spawn a handful of small physics-enabled chunks at
        `point`, each with a random outward velocity. They fly off,
        tumble, and rest naturally as ODE handles their trajectories.

        The chunks are spawned slightly above the contact point so they
        don't immediately interpenetrate the surface that just produced
        them, and biased upward so the burst reads as an explosion
        rather than a drop.
        """
        if self.root_children is None:
            return
        if os.environ.get("OMNISIM_LITE_DAMAGE") == "1":
            return
        rng = self._debris_rng
        n = rng.randint(*DEBRIS_BURST_COUNT_RANGE)
        # Slight elevation prevents instant ground penetration; the
        # marker sits much higher (z+0.6) so debris is naturally below it.
        spawn_z = float(point[2]) + 0.25

        for _ in range(n):
            self._debris_counter += 1
            name = f"damage_debris_{self._debris_counter:04d}"
            mass = rng.uniform(*DEBRIS_MASS_RANGE)
            color = rng.choice(DEBRIS_COLORS)
            # Build a thin panel: pick which axis is the thin one, give
            # it a small dimension; the other two are longer. This reads
            # as a torn-off plate rather than a uniform cube of debris.
            thin_axis = rng.randint(0, 2)
            dims = [rng.uniform(*DEBRIS_PLATE_LONG_RANGE) for _ in range(3)]
            dims[thin_axis] = rng.uniform(*DEBRIS_PLATE_THIN_RANGE)
            # Outward unit vector in xy + a vertical bias. Small offset
            # along the outward direction so chunks from the same burst
            # don't stack at one xy.
            theta = rng.uniform(0.0, 2.0 * math.pi)
            vx_dir = math.cos(theta)
            vy_dir = math.sin(theta)
            vz_dir = rng.uniform(*DEBRIS_VERTICAL_BIAS)
            speed = rng.uniform(*DEBRIS_VELOCITY_RANGE)
            mag = math.sqrt(vx_dir * vx_dir + vy_dir * vy_dir + vz_dir * vz_dir)
            vx = speed * vx_dir / mag
            vy = speed * vy_dir / mag
            vz = speed * vz_dir / mag
            ox = float(point[0]) + 0.05 * vx_dir
            oy = float(point[1]) + 0.05 * vy_dir
            # Random initial rotation so plates don't all spawn axis-
            # aligned (they'd look like a stack of CD cases). Pick a
            # uniform axis-angle.
            r_axis = [rng.uniform(-1.0, 1.0) for _ in range(3)]
            r_norm = math.sqrt(sum(x * x for x in r_axis)) or 1.0
            r_axis = [x / r_norm for x in r_axis]
            r_angle = rng.uniform(0.0, math.pi)
            stanza = (
                f'DEF {name} Solid {{'
                f'  translation {ox} {oy} {spawn_z}'
                f'  rotation {r_axis[0]} {r_axis[1]} {r_axis[2]} {r_angle}'
                f'  name "{name}"'
                f'  children ['
                f'    Shape {{'
                f'      appearance PBRAppearance {{'
                f'        baseColor {color[0]} {color[1]} {color[2]}'
                f'        roughness 0.85'
                f'        metalness 0.4'
                f'      }}'
                f'      geometry Box {{ size {dims[0]} {dims[1]} {dims[2]} }}'
                f'    }}'
                f'  ]'
                f'  boundingObject Box {{ size {dims[0]} {dims[1]} {dims[2]} }}'
                f'  physics Physics {{ density -1 mass {mass} }}'
                f'}}'
            )
            try:
                self.root_children.importMFNodeFromString(-1, stanza)
                self.debris_chunks.append(name)
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(f"[damage_tracker] debris spawn failed: {exc}\n")
                continue
            # Set the chunk's initial velocity so it actually flies.
            # getFromDef + setVelocity is the standard idiom for giving
            # a freshly-imported body initial motion.
            node = self.supervisor.getFromDef(name)
            if node is not None:
                try:
                    # Webots setVelocity wants 6 elements (linear xyz +
                    # angular xyz). A small random spin makes the chunk
                    # tumble visibly mid-air.
                    spin = [rng.uniform(-6.0, 6.0) for _ in range(3)]
                    node.setVelocity([vx, vy, vz, *spin])
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(
                        f"[damage_tracker] debris {name} setVelocity failed: {exc}\n"
                    )

        sys.stderr.write(
            f"[damage_tracker] debris burst: {n} chunks at "
            f"({point[0]:.2f},{point[1]:.2f},{point[2]:.2f})\n"
        )

    def _maybe_summary(self) -> None:
        now = time.monotonic()
        if now - self._last_summary_t < self._summary_period_s:
            return
        self._last_summary_t = now
        if not self._summary_counts:
            return
        parts_str = ", ".join(
            f"{p}={c}" for p, c in sorted(self._summary_counts.items(),
                                          key=lambda kv: -kv[1])
        )
        sys.stderr.write(
            f"[damage_tracker] summary last {self._summary_period_s:.0f}s: {parts_str} "
            f"(total events buffered: {len(self.events)}, dropped: {self.dropped_events})\n"
        )
        sys.stderr.flush()
        self._summary_counts.clear()

    def geometry_stats(self) -> dict:
        """Phase 15 verification hook. For each procedural-deformation
        part with an active vertex buffer, compare its current vertex
        positions to the pristine baseline and report displacement
        statistics. This is the numerical ground-truth for "is the
        chassis actually deforming?" — visual inspection at default
        zoom is unreliable.

        Returns: {part: {vertex_count, displaced_count, max_displacement_m,
                          mean_displacement_m, rms_displacement_m,
                          current_crumple, dirty}, ...}
        """
        out: dict = {}
        for part, verts in self.part_vertices.items():
            size = self.profile.crumple_size.get(part)
            if size is None:
                continue
            baseline = make_baseline_box_buffer(
                size, subdivision=DEFORM_SUBDIVISION
            )
            if len(baseline) != len(verts):
                continue
            n = len(verts) // 3
            max_d = 0.0
            sum_d = 0.0
            sum_sq = 0.0
            displaced = 0
            for i in range(n):
                vi = i * 3
                dx = verts[vi] - baseline[vi]
                dy = verts[vi + 1] - baseline[vi + 1]
                dz = verts[vi + 2] - baseline[vi + 2]
                d = (dx * dx + dy * dy + dz * dz) ** 0.5
                if d > 0.001:  # 1mm threshold for "displaced"
                    displaced += 1
                if d > max_d:
                    max_d = d
                sum_d += d
                sum_sq += d * d
            # Phase 17a: count vertices past the plastic-yield strain
            # threshold. A non-zero count is necessary (but not yet
            # sufficient) for fracture; Phase 17b will require these
            # to also form a connected island.
            strained_idx, max_strain = find_strained_vertices(
                verts, baseline, FRACTURE_STRAIN_M
            )
            out[part] = {
                "vertex_count": n,
                "displaced_count": displaced,
                "max_displacement_m": max_d,
                "mean_displacement_m": sum_d / n if n else 0.0,
                "rms_displacement_m": (sum_sq / n) ** 0.5 if n else 0.0,
                "current_crumple": self.current_crumple.get(part, 0.0),
                "dirty": bool(self.dirty_meshes.get(part, False)),
                "max_strain_m": max_strain,
                "strained_vertex_count": len(strained_idx),
            }
        # Phase 17 — fragment counter spans the whole tracker, not
        # per-part. Surfaced once at the dict root so callers see total
        # fracture activity.
        out["__fragments_spawned_total"] = self._fragment_counter
        out["__fragments_alive"] = len(self.fragments_spawned)
        return out

    def state_snapshot(self) -> dict:
        """Phase 3 hook. Returns the full per-part HP/state record plus
        a small set of counters and the Phase 5 game_over flag. Stable
        shape; passes straight through to the harness `damage_state`
        HTTP endpoint.
        """
        chassis = self.part_hp.get("chassis")
        game_over = bool(chassis and chassis["state"] == "broken")
        return {
            "robot": self.robot_name,
            "attached": self.attached,
            "parts": {p: len(solids) for p, solids in self.parts.items()},
            "damage": {p: dict(record) for p, record in self.part_hp.items()},
            "game_over": game_over,
            "events_total": self.event_counter,
            "events_buffered": len(self.events),
            "events_dropped": self.dropped_events,
            "contacts_seen": self.contacts_seen,
            "use_depth": self.use_depth,
            "depth_scale": self.depth_scale,
            "depth_contacts_used": self.depth_contacts_used,
            "depth_contacts_fallback": self.depth_contacts_fallback,
            "slab_check_count": self.slab_check_count,
            "slab_reattrib_count": self.slab_reattrib_count,
            "synth_fire_count": self.synth_fire_count,
            "synth_to_slab_count": self.synth_to_slab_count,
            "synth_high_z_contacts_total": self.synth_high_z_contacts_total,
            "synth_max_z_seen": self.synth_max_z_seen,
            "crash_boost_count": self.crash_boost_count,
            "pool_spawn_count": self._pool_spawn_count,
            "pool_fail_count": self._pool_fail_count,
            "decals_per_part": {p: len(b) for p, b in self.part_decals.items()},
            "decals_attempted": self._decal_attempts,
            "decals_spawned_total": self._decal_counter,
            "detached_parts": sorted(self.detached_parts),
        }

    def events_since(self, since_step_id: int = 0, limit: int = 256) -> list[dict]:
        """Return events with step_id > since_step_id, oldest-first, up
        to `limit`. Phase 3 wire format passes through unchanged.
        """
        out: list[dict] = []
        for evt in self.events:
            if evt["step_id"] <= since_step_id:
                continue
            out.append(evt)
            if len(out) >= limit:
                break
        return out

    def set_heal_rate(self, rate_hp: float | None = None,
                      rate_mesh: float | None = None,
                      parts: list[str] | None = None) -> dict:
        """Phase 18d: set per-part heal rates explicitly. If `parts`
        is None, applies to all known parts. None values for rate_hp
        / rate_mesh leave that channel unchanged.

        Returns the resulting heal-rate maps.
        """
        target_parts = list(parts) if parts else list(self.part_hp.keys())
        for part in target_parts:
            if rate_hp is not None:
                self.heal_rate_hp_per_s[part] = float(rate_hp)
            if rate_mesh is not None:
                self.heal_rate_mesh_m_per_s[part] = float(rate_mesh)
        return {
            "heal_rate_hp_per_s": dict(self.heal_rate_hp_per_s),
            "heal_rate_mesh_m_per_s": dict(self.heal_rate_mesh_m_per_s),
            "parts_targeted": target_parts,
        }

    def heal_to_pristine(self, sim_time_ms: int = 0) -> dict:
        """Phase 18d: instant full repair — HP back to max, vertex
        buffers back to baseline, current_crumple cleared. Unlike
        damage_reset, this preserves the event ring buffer and emits
        the right state-transition events so SDK consumers see the
        full transition stream rather than a silent reset.
        """
        healed_parts: list[str] = []
        for part, record in self.part_hp.items():
            if record["state"] == "pristine" and record["hp"] >= record["hp_max"]:
                continue
            old_state = record["state"]
            record["hp"] = record["hp_max"]
            record["state"] = "pristine"
            self._emit_transition(sim_time_ms, part, old_state, "pristine",
                                  record["hp"], 0.0)
            healed_parts.append(part)
        # Vertex buffers back to baseline.
        for part in list(self.part_vertices.keys()):
            size = self.profile.crumple_size.get(part)
            if size is None:
                continue
            baseline = make_baseline_box_buffer(
                size, subdivision=DEFORM_SUBDIVISION
            )
            self.part_vertices[part] = baseline
            self.dirty_meshes[part] = True
        self.current_crumple.clear()
        # Phase 5: re-publish gate so wheels regain torque.
        self._write_behavior_gate()
        return {"healed_parts": healed_parts}

    def inject(self, part: str, hp_delta: float | None = None,
               state: str | None = None,
               sim_time_ms: int = 0) -> dict:
        """Phase 5 test/debug hook. Apply a direct damage delta or set a
        target state on a part, bypassing the contact pipeline. Used to
        verify behaviour gating end-to-end without relying on box-drop
        timing or contact attribution. Returns the part's record after
        the change, or an error dict for unknown parts.
        """
        record = self.part_hp.get(part)
        if record is None:
            return {"error": f"unknown part: {part}"}
        old_state = record["state"]
        if hp_delta is not None:
            record["hp"] = max(0.0, min(record["hp_max"], record["hp"] + float(hp_delta)))
            record["state"] = self._state_for_hp(record["hp"], record["hp_max"])
        if state is not None:
            valid = {label for label, _ in STATE_BANDS}
            if state not in valid:
                return {"error": f"unknown state: {state}; valid: {sorted(valid)}"}
            record["state"] = state
            for label, floor in STATE_BANDS:
                if label == state:
                    record["hp"] = record["hp_max"] * max(floor, 0.001)
                    break
        if record["state"] != old_state:
            self._emit_transition(sim_time_ms, part, old_state, record["state"],
                                  record["hp"], 0.0)
        return dict(record)

    def reset(self) -> None:
        """Zero accumulated state without resetting the simulation. Heals
        all parts back to full HP / pristine state, clears the event
        buffer, removes all spawned damage markers from the world, and
        forgets per-body velocity history.
        """
        self.events.clear()
        self.event_counter = 0
        self.dropped_events = 0
        self.prev_velocity.clear()
        self.prev_chassis_velocity = None
        self.vel_ema.clear()  # opt-in OMNISIM_DAMAGE_VEL_SMOOTH path; honor the
        # docstring's "forgets per-body velocity history" so a post-reset capture
        # doesn't carry a stale EMA into its first few synthesized impulses.
        self.contacts_seen = 0
        self.depth_contacts_used = 0
        self.depth_contacts_fallback = 0
        self._summary_counts.clear()
        for record in self.part_hp.values():
            record["state"] = "pristine"
            record["hp"] = record["hp_max"]
            record["last_impact_step"] = 0
            record["last_impact_J"] = 0.0
            record["total_impulse_J"] = 0.0
        # Remove damage markers and debris from the world. getFromDef +
        # node.remove is the standard runtime-removal idiom; misses
        # (e.g. node already gone) are benign.
        marker_names = [name for name, _t in self.damage_markers]
        for name in marker_names + self.debris_chunks:
            node = self.supervisor.getFromDef(name)
            if node is not None:
                try:
                    node.remove()
                except Exception:  # noqa: BLE001
                    pass
        self.damage_markers.clear()
        self.debris_chunks.clear()
        # Phase 11: clear particles too.
        for name, _expire in self.particles:
            node = self.supervisor.getFromDef(name)
            if node is not None:
                try:
                    node.remove()
                except Exception:  # noqa: BLE001
                    pass
        self.particles.clear()
        self._last_smoke_at = -10**9
        self._last_spark_at = -10**9
        self._last_fluid_at = -10**9
        # Phase 9 + 19: clean up free bodies we spawned (wheels,
        # bumpers, top_plate). The originals we removed from the
        # URDFRobot tree don't come back — reset heals state but the
        # structural damage is permanent until world reload. Document
        # this in the snapshot so callers know.
        for name in self.detached_part_defs:
            node = self.supervisor.getFromDef(name)
            if node is not None:
                try:
                    node.remove()
                except Exception:  # noqa: BLE001
                    pass
        self.detached_part_defs.clear()
        self.detached_parts.clear()
        # Phase 8: remove all decals attached to parts.
        for bucket in self.part_decals.values():
            for name, _parent in bucket:
                node = self.supervisor.getFromDef(name)
                if node is not None:
                    try:
                        node.remove()
                    except Exception:  # noqa: BLE001
                        pass
        self.part_decals.clear()
        # Phase 14c: remove all dents attached to parts.
        for bucket in self.part_dents.values():
            for name, _parent in bucket:
                node = self.supervisor.getFromDef(name)
                if node is not None:
                    try:
                        node.remove()
                    except Exception:  # noqa: BLE001
                        pass
        self.part_dents.clear()
        # Phase 7: restore each tracked part's shapes to their pristine
        # appearance variant. We don't perfectly recover the original
        # URDF mesh material; we set a clean PBRAppearance that reads
        # as undamaged.
        for part in self.part_shapes.keys():
            self._apply_appearance(part, "pristine")
        # Phase 15d: wipe the deformable vertex buffer so the next
        # damage cycle starts from a clean slate.
        self.part_vertices.clear()
        self.dirty_meshes.clear()
        self.last_remit_ms.clear()
        self.current_crumple.clear()
        self._last_dent_at_ms.clear()
        # Phase 17c: remove all spawned fragments. Counter resets too
        # so geometry_stats fragments_spawned_total starts fresh.
        for name in self.fragments_spawned:
            node = self.supervisor.getFromDef(name)
            if node is not None:
                try:
                    node.remove()
                except Exception:  # noqa: BLE001
                    pass
        self.fragments_spawned.clear()
        self._fragment_counter = 0
        # Phase 10: restore each tracked part's geometry to its
        # pristine mesh variant if one is declared. No-ops without
        # variants. Note: this means the original URDF-imported mesh
        # is REPLACED with whatever pristine variant is declared, so
        # profiles should declare the original .dae as the pristine
        # entry to preserve appearance.
        for part in self.part_shapes.keys():
            self._apply_geometry(part, "pristine")
        # Phase 5: re-publish the gate so a driving controller sees full
        # wheel torque + game_over=false on the next step.
        self._write_behavior_gate()
