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

"""Runtime observation helpers for the harness supervisor.

These functions are pulled out of harness_supervisor.py so they can be
unit-tested with a fake Supervisor stub and so the supervisor's main file
stays focused on the IPC loop. They walk the live scene tree using the
Supervisor read APIs (`getRoot`, `getField`, `getMFNode`, etc.) — they do
not own the robot's controller-side device APIs.
"""

from __future__ import annotations

import math
import sys
from contextlib import contextmanager
from typing import Any

import geometry


# Webots node typenames that mark a Robot. There is a single canonical
# typename ("Robot"), but URDF imports and PROTOs may resolve to
# user-named PROTOs whose base type is Robot. Falling back to a name list
# would miss PROTOs; instead we ask each candidate via
# `getNumberOfDevices()` (which only Robot nodes implement) — but that
# only works for Robots that own a device root, which the supervisor
# itself does. For sibling robots we settle for typename-suffix
# matching, which catches "Robot" and the common URDF base.
ROBOT_TYPENAMES = {"Robot"}

JOINT_TYPENAMES = {"HingeJoint", "SliderJoint", "Hinge2Joint", "BallJoint"}
SOLID_TYPENAMES = {"Solid", "Robot"}

# Motor device typenames. Used by the joint-write path to tell a motorised
# joint (the supervisor write re-pins the motor's PD target) from a passive
# one (the write sets the joint coordinate and nothing holds it).
MOTOR_TYPENAMES = {"RotationalMotor", "LinearMotor"}

# ---------------------------------------------------------------------------
# Posed-node classification (the /scene/tree round-trip killer)
# ---------------------------------------------------------------------------
#
# `wb_supervisor_node_get_position()` / `get_orientation()` cost one engine
# round-trip EACH, per node, per call — and for any node not derived from Pose
# the engine (a) warns into the world log and (b) returns NaN, which the
# harness sanitizes to null. Measured on the 298-node 10-Husky bench scene:
# 145 of 298 nodes are non-posed (100 Shape, 40 HingeJoint, WorldInfo, sky,
# ...), so every /scene/tree call paid 290 round-trips and pushed 145 warning
# lines into the engine log for values that are null by construction.
#
# Classification is by the node's BASE type, which mirrors the engine's OmPose
# class hierarchy and costs no round-trip (getTypeName / getBaseTypeName are
# answered from libController's client-side node struct — see
# src/controller/c/supervisor.c, wb_supervisor_node_get_base_type_name).
# The pose-carrying and geometry base-type sets are SHARED with geometry.py's
# frame walk so the two cannot drift; extend the sets there.
#
# A type in neither table is classified by MEASUREMENT on its first pose read
# (exactly what the old code paid every call): finite -> posed, NaN ->
# unposed; the verdict is memoized per TYPE NAME (posedness is a property of
# the type, and this keeps the memo bounded by the type vocabulary, not the
# node count). A wrong static entry is the only wrong-answer risk, which is
# why the tables carry only fixed engine base types, never PROTO names.
POSED_BY_BASE_TYPE: dict[str, bool] = {
    **{t: True for t in geometry._POSE_TYPES},
    **{t: False for t in geometry._GEOMETRY_TYPES},
    **{t: False for t in JOINT_TYPENAMES},
    **{t: False for t in (
        # scene scaffolding
        "Group", "Slot", "SolidReference", "WorldInfo", "Viewpoint",
        "Background", "Fog", "DirectionalLight", "PointLight", "SpotLight",
        # joint satellites (OmBasicJoint and its devices are not Pose-derived)
        "JointParameters", "HingeJointParameters", "BallJointParameters",
        "RotationalMotor", "LinearMotor", "PositionSensor", "Brake",
        # visual / physical satellites
        "Shape", "Appearance", "PBRAppearance", "Material", "ImageTexture",
        "TextureTransform", "TextureCoordinate", "Color", "Coordinate",
        "Normal", "Physics", "Damping", "ContactProperties",
        "ImmersionProperties", "Recognition", "Lens", "LensFlare", "Focus",
        "Zoom",
    )},
}

# Measured verdicts for types the static table does not know. Keyed by type
# name; cleared only for tests (a type's posedness cannot change, and the
# supervisor process — hence this dict — dies with its world).
_POSED_BY_TYPE_MEASURED: dict[str, bool] = {}


def reset_posed_memo() -> None:
    """Drop measured posedness verdicts (exposed for unit tests)."""
    _POSED_BY_TYPE_MEASURED.clear()


def node_is_posed(node, type_name: str) -> bool | None:
    """Does this node carry a real world pose? True / False / None (unknown —
    the caller should read the pose and report it via
    `record_pose_measurement`). No engine round-trips."""
    verdict = POSED_BY_BASE_TYPE.get(type_name)
    if verdict is not None:
        return verdict
    try:
        base = node.getBaseTypeName()
    except Exception:  # noqa: BLE001
        base = None
    if base is not None:
        verdict = POSED_BY_BASE_TYPE.get(base)
        if verdict is not None:
            return verdict
    return _POSED_BY_TYPE_MEASURED.get(type_name)


def record_pose_measurement(type_name: str, position) -> None:
    """Classify an unknown type from what the engine actually answered:
    a finite position is a real pose, NaN is the engine's "not a Pose node"
    marker. Never overrides the static tables (node_is_posed consults them
    first), so a measurement cannot null out a known-posed type."""
    if position is None or len(position) != 3:
        return
    _POSED_BY_TYPE_MEASURED[type_name] = all(
        math.isfinite(float(v)) for v in position)


# ---------------------------------------------------------------------------
# Paused read bursts
# ---------------------------------------------------------------------------


# wb enum value of WB_SUPERVISOR_SIMULATION_MODE_PAUSE (stable engine ABI;
# also exposed as Supervisor.SIMULATION_MODE_PAUSE by the binding).
SIMULATION_MODE_PAUSE = 0


@contextmanager
def paused_reads(supervisor):
    """Pause the engine for the duration of a read burst, then restore.

    Why: every supervisor read (getPosition, getMFNode, getSFFloat,
    getContactPoints, ...) is a synchronous round-trip serviced by the
    engine's event loop. Measured on the 298-node 10-Husky bench scene
    (Newton, light session): ~6 ms per round-trip free-running vs ~0.15 ms
    paused — 40x — with ~1 ms to enter the pause. Re-measured 2026-09-01
    after the engine's immediate-burst fast path (OMNISIM_IMMEDIATE_BURST,
    OmController::readRequest serves a request burst at pipe speed instead
    of one event-loop wakeup each): ~0.9 ms free-running vs ~0.6 ms paused
    on the 309-node fleet arena — the pause is no longer the dominant lever,
    but the CONSISTENT-snapshot rationale below still holds, so keep it.
    GET /debug/read_bench measures both numbers on any live session. The read-heavy RPCs pause for their walk (the whole-scene
    readers below do it themselves; the dispatch arms that compose geometry
    walks wrap their own blocks) and restore the caller's mode afterwards.
    Side benefit: the snapshot is CONSISTENT — the old walks smeared over
    hundreds of free-running sim steps.

    Boundary rule: a paused block starts at the command body's FIRST engine
    access and ends at its return; anything that steps the sim happens
    before the block. ⚠️ Never call supervisor.step() inside — a step
    against a paused engine blocks until someone unpauses, i.e. deadlock
    (tests/harness/test_read_paths.py has an AST tripwire for this).

    Re-entrant by construction (an already-paused engine is left alone) and
    exception-safe (the previous mode is restored no matter what the body
    raised). Yields True when a pause was actually taken, False when the
    supervisor does not support simulation modes (unit-test stubs) or is
    already paused.
    """
    get_mode = getattr(supervisor, "simulationGetMode", None)
    set_mode = getattr(supervisor, "simulationSetMode", None)
    took = False
    mode_before = None
    if get_mode is not None and set_mode is not None:
        try:
            mode_before = get_mode()
            if mode_before != SIMULATION_MODE_PAUSE:
                set_mode(SIMULATION_MODE_PAUSE)
                took = True
        except Exception:  # noqa: BLE001
            took = False
    try:
        yield took
    finally:
        if took:
            try:
                set_mode(mode_before)
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] paused_reads: failed to restore "
                    f"simulation mode {mode_before}: {exc}\n")


# ---------------------------------------------------------------------------
# Scene-walk helpers
# ---------------------------------------------------------------------------


def _children_of(node) -> list:
    """Return the list of child nodes via the standard `children` field.
    Returns [] for nodes with no children field or where the field is
    inaccessible.
    """
    if node is None:
        return []
    field = node.getField("children")
    if field is None:
        return []
    try:
        count = field.getCount()
    except Exception:
        return []
    out = []
    for i in range(count):
        try:
            child = field.getMFNode(i)
        except Exception:
            continue
        if child is not None:
            out.append(child)
    return out


def _endpoint_of(node):
    """Return the SF endPoint child of a joint node, or None."""
    if node is None:
        return None
    field = node.getField("endPoint")
    if field is None:
        return None
    try:
        return field.getSFNode()
    except Exception:
        return None


def _joint_parameters(node):
    """Return the JointParameters SF child of a joint node, or None.

    HingeJoint and SliderJoint expose `jointParameters`. Hinge2Joint and
    BallJoint additionally expose `jointParameters2`/`jointParameters3`;
    we treat the first axis as canonical for snapshot reads.
    """
    if node is None:
        return None
    field = node.getField("jointParameters")
    if field is None:
        return None
    try:
        return field.getSFNode()
    except Exception:
        return None


def _walk(root, predicate, accumulator: list) -> None:
    """Depth-first walk from `root`, appending nodes that satisfy
    `predicate(node)` to `accumulator`. Recurses through `children` fields
    and joint `endPoint` fields so robot subtrees are visited fully.
    """
    if root is None:
        return
    if predicate(root):
        accumulator.append(root)
    for child in _children_of(root):
        _walk(child, predicate, accumulator)
    endpoint = _endpoint_of(root)
    if endpoint is not None and endpoint is not root:
        _walk(endpoint, predicate, accumulator)


def _is_robot(node) -> bool:
    if node is None:
        return False
    try:
        if node.getTypeName() in ROBOT_TYPENAMES:
            return True
        # A portable world naturally packages a reusable robot as a PROTO.
        # Supervisor.getTypeName() then returns the PROTO name (ScaleBot,
        # Pioneer3at, ...), while getBaseTypeName() carries the fact that the
        # instance IS a Robot. Ignoring the base type made /robots and every
        # recorder built on this helper report zero robots in a running world
        # that had ten controller processes and fifty dynamic bodies.
        return node.getBaseTypeName() in ROBOT_TYPENAMES
    except Exception:
        return False


def _is_joint(node) -> bool:
    if node is None:
        return False
    try:
        return node.getTypeName() in JOINT_TYPENAMES
    except Exception:
        return False


def _is_solid(node) -> bool:
    if node is None:
        return False
    try:
        return (node.getTypeName() in SOLID_TYPENAMES
                or node.getBaseTypeName() in SOLID_TYPENAMES)
    except Exception:
        return False


def _sf_string(node, field_name: str) -> str | None:
    if node is None:
        return None
    f = node.getField(field_name)
    if f is None:
        return None
    try:
        return f.getSFString() or None
    except Exception:
        return None


def _sf_float(node, field_name: str) -> float | None:
    if node is None:
        return None
    f = node.getField(field_name)
    if f is None:
        return None
    try:
        return float(f.getSFFloat())
    except Exception:
        return None


def _node_def_or_id(node) -> str:
    """Return a stable identifier for a node: its DEF if any, else
    `"#<id>"` using the node's getId(). DEF is preferable for agent
    consumption; the id fallback ensures every node has a key.
    """
    if node is None:
        return "#null"
    try:
        d = node.getDef()
        if d:
            return d
    except Exception:
        pass
    try:
        return f"#{node.getId()}"
    except Exception:
        return "#?"


def _safe_position(node) -> list[float] | None:
    if node is None:
        return None
    try:
        return [float(x) for x in node.getPosition()]
    except Exception:
        return None


def _safe_orientation(node) -> list[float] | None:
    if node is None:
        return None
    try:
        return [float(x) for x in node.getOrientation()]
    except Exception:
        return None


# ---------------------------------------------------------------------------
# /robots
# ---------------------------------------------------------------------------


def list_robots(supervisor) -> list[dict]:
    """Enumerate every Robot node in the scene tree.

    For each robot, return identity (def, name, model, controller),
    pose (position, orientation), and a count of joints in its subtree
    (rough proxy for "how complex is this thing"). The supervisor's own
    Robot node is included — callers can filter by name if they want to
    skip it.
    """
    with paused_reads(supervisor):
        root = supervisor.getRoot()
        robots: list = []
        _walk(root, _is_robot, robots)
        out: list[dict] = []
        for node in robots:
            joints: list = []
            _walk(node, _is_joint, joints)
            out.append({
                "def": _node_def_or_id(node),
                "name": _sf_string(node, "name"),
                "model": _sf_string(node, "model"),
                "controller": _sf_string(node, "controller"),
                "type": node.getTypeName(),
                "position": _safe_position(node),
                "orientation": _safe_orientation(node),
                "num_joints": len(joints),
            })
        return out


# ---------------------------------------------------------------------------
# /robot/<def>/joints
# ---------------------------------------------------------------------------


def joint_display_name(joint_node) -> str | None:
    """The name GET /robot/<def>/joints reports for a joint.

    Joint name = the device[0] motor's name when present, else the
    endPoint solid's name. Better than nothing — gives the agent a
    human label to correlate with the URDF. Shared by the read path
    (joint_snapshot) and the write path (joint_write_index) so the names
    an agent reads are, by construction, the names it can command.
    """
    name = None
    devices = joint_node.getField("device")
    if devices is not None:
        try:
            if devices.getCount() > 0:
                first = devices.getMFNode(0)
                name = _sf_string(first, "name")
        except Exception:
            pass
    if name is None:
        endpoint = _endpoint_of(joint_node)
        name = _sf_string(endpoint, "name")
    return name


def _joint_motor(joint_node):
    """First Motor device (RotationalMotor/LinearMotor) on a joint, or None.

    device[0] is not necessarily the motor — a joint may list a
    PositionSensor or Brake first — so this scans the whole device list.
    """
    if joint_node is None:
        return None
    devices = joint_node.getField("device")
    if devices is None:
        return None
    try:
        count = devices.getCount()
    except Exception:
        return None
    for i in range(count):
        try:
            dev = devices.getMFNode(i)
            if dev is not None and dev.getTypeName() in MOTOR_TYPENAMES:
                return dev
        except Exception:
            continue
    return None


def joint_write_index(supervisor, robot_def: str) -> list[dict]:
    """Everything a joint WRITE needs to know before the first mutation.

    One paused walk of the robot's subtree, returning per joint: the node
    itself (for Node.setJointPosition), its JointParameters node (position
    read-back + hard stops, which are what the engine clamps against —
    OmJointParameters::clampPosition), and the motor's control limits
    (minPosition/maxPosition), which are what the Newton registration path
    uses to decide servo-vs-velocity-wheel (OmBasicJoint.cpp: a motor whose
    effective limits are equal is built with ke=0 and position targets are
    silently ignored). Raises KeyError for an unknown robot DEF, mirroring
    list_joints.
    """
    with paused_reads(supervisor):
        robot = supervisor.getFromDef(robot_def)
        if robot is None:
            raise KeyError(f"no node with DEF '{robot_def}'")
        joints: list = []
        _walk(robot, _is_joint, joints)
        out: list[dict] = []
        for j in joints:
            params = _joint_parameters(j)
            motor = _joint_motor(j)
            out.append({
                "node": j,
                "params": params,
                "name": joint_display_name(j),
                "type": j.getTypeName(),
                "position": _sf_float(params, "position"),
                "min_stop": _sf_float(params, "minStop"),
                "max_stop": _sf_float(params, "maxStop"),
                "has_motor": motor is not None,
                "motor_name": _sf_string(motor, "name"),
                "motor_min": _sf_float(motor, "minPosition"),
                "motor_max": _sf_float(motor, "maxPosition"),
            })
        return out


def read_joint_positions(supervisor, entries: list[dict]) -> list[float | None]:
    """Re-read `JointParameters.position` for joint_write_index entries.

    One paused burst; an entry whose joint has no JointParameters node reads
    None (unmeasured — never a number that was not measured).
    """
    with paused_reads(supervisor):
        return [_sf_float(e.get("params"), "position") for e in entries]


def joint_snapshot(joint_node, prev_position: float | None,
                   dt_s: float) -> dict:
    """Build a single joint dict.

    Joint position is read from `JointParameters.position`, which Webots
    keeps live during simulation. Velocity is computed by differencing
    against `prev_position`; if no previous sample exists or dt is 0,
    velocity is None.

    Limits come from `JointParameters.minStop` / `maxStop`. A joint hits
    a limit when `position - tol <= minStop` or `position + tol >= maxStop`
    (only meaningful when stops are set; if both stops are 0 the joint is
    unconstrained and `hit_limit` is None).
    """
    type_name = joint_node.getTypeName()
    params = _joint_parameters(joint_node)
    position = _sf_float(params, "position")
    min_stop = _sf_float(params, "minStop")
    max_stop = _sf_float(params, "maxStop")
    name = joint_display_name(joint_node)

    velocity = None
    if position is not None and prev_position is not None and dt_s > 0:
        velocity = (position - prev_position) / dt_s

    hit_limit: str | None = None
    if position is not None and min_stop is not None and max_stop is not None \
            and not (min_stop == 0.0 and max_stop == 0.0):
        tol = 1e-3
        if position <= min_stop + tol:
            hit_limit = "lower"
        elif position >= max_stop - tol:
            hit_limit = "upper"

    return {
        "name": name,
        "type": type_name,
        "position": position,
        "velocity": velocity,
        "lower": min_stop,
        "upper": max_stop,
        "hit_limit": hit_limit,
    }


def list_joints(supervisor, robot_def: str,
                joint_velocity_cache: dict[int, tuple[float, float]],
                sim_time_ms: float) -> dict:
    """Walk a robot's subtree and snapshot every joint.

    `joint_velocity_cache` maps joint node id -> (last_position, last_t_s)
    so we can compute velocity. Caller supplies and persists it across
    invocations.
    """
    with paused_reads(supervisor):
        robot = supervisor.getFromDef(robot_def)
        if robot is None:
            raise KeyError(f"no node with DEF '{robot_def}'")
        joints: list = []
        _walk(robot, _is_joint, joints)
        now_s = sim_time_ms / 1000.0
        out: list[dict] = []
        for j in joints:
            try:
                jid = j.getId()
            except Exception:
                jid = None
            prev_pos, prev_t = (None, None)
            if jid is not None and jid in joint_velocity_cache:
                prev_pos, prev_t = joint_velocity_cache[jid]
            dt_s = (now_s - prev_t) if prev_t is not None else 0.0
            snap = joint_snapshot(j, prev_pos, dt_s)
            out.append(snap)
            if jid is not None and snap["position"] is not None:
                joint_velocity_cache[jid] = (snap["position"], now_s)
        return {"robot": robot_def, "joints": out}


# ---------------------------------------------------------------------------
# /sim/contacts
# ---------------------------------------------------------------------------


# How close two solids' reported contact points must be to be treated as the
# same contact. Both sides of one contact are pushed from the same world point
# by the engine, so a match is normally exact; the tolerance only absorbs the
# float round-trip through the wire.
CONTACT_PAIR_TOL_M = 1e-6


def _quantize_point(point, tol: float = CONTACT_PAIR_TOL_M) -> tuple:
    return tuple(round(float(c) / tol) for c in point)


def collect_contacts(supervisor, light: bool = False) -> dict:
    """The global contact set, PAIRED, plus what was and wasn't measured.

    Two things about the engine's contact API drove this shape.

    1. **`node_id` is NOT the other body.** `OmSupervisorUtilities::
       pushContactPointsToStream` streams `solid->uniqueId()` for a shallow
       query and, for a deep one, the sub-solid *inside the queried subtree*.
       So the queried body's own id comes back either way -- FLOOR reported
       ids [9,9,9,9] and CRATE_BOT ids [14,14,14,14] for the same four
       contacts. The previous code read that field as "the other body" and
       therefore keyed every contact as the pair (id, id), i.e. it could only
       ever report a body in contact with ITSELF. Pairing is instead done on
       the contact POINT: both sides of one contact are pushed from the same
       world position, so the point is the join key. A point only one body
       reports (its partner is not a walkable Solid) is still reported, with
       `b_def: null` and `paired: false` -- an honest half-contact beats a
       fabricated pair.

    2. **An empty `contacts` list is still not proof of no contact -- but NOT
       for the reason this docstring used to give.**

       ⚠ THE OLD REASON WAS AN ODE BODY-SLEEP MODEL, AND IT IS FICTION. It
       claimed ODE auto-disables an idle body after
       `WorldInfo.physicsDisableTime` so a resting body generates no contact
       points, and told the agent to call `wake=true` to clear the sleep timer.
       There is no ODE (src/ode deleted, commit bdc02139) and there is no body
       sleep: `physicsDisableTime` is parsed into `OmWorldInfo::
       mPhysicsDisableTime` and **nothing in the engine reads it back** --
       `physicsDisableTime()` has zero call sites. So the surface was handing an
       agent a false, plausible, reassuring cause for the exact symptom a REAL
       defect produces (a Solid pinned `physicsBackend "ode"`, or an absent
       Newton runtime -- both of which make a body genuinely contact-free and
       genuinely never-falling), and the recommended fix was a no-op.

       What is true today: a Newton-registered Solid at rest DOES report its
       contacts. Native contact readback has been on by default since
       2026-08-07 (`OmSolid::extractContactPoints`; pinned by
       tests/test_newton_contacts_visible_by_default.py with a geometric
       rest-height assertion precisely so a zero cannot be excused as "really
       mid-air"). The real reasons an empty set can be empty are enumerated in
       `tracking.empty_set_reasons`, and the first of them is MEASURED here per
       Solid, not guessed: a Solid whose own (or whose ancestor's)
       `physicsBackend` field says `"ode"` has no physics at all.

       This response NEVER asserts the list is complete.
    """
    with paused_reads(supervisor):
        solids, contacts, queried = _walk_contacts(supervisor)
        tracking = _contact_tracking_scope(supervisor, solids, queried,
                                           light=light)
        tracking["contacts_paired"] = sum(1 for c in contacts if c["paired"])
        tracking["contacts_unpaired"] = sum(
            1 for c in contacts if not c["paired"])
        return {"contacts": contacts, "tracking": tracking}


# ---------------------------------------------------------------------------
# Solid-list cache
# ---------------------------------------------------------------------------
# WHY THIS EXISTS. _walk_contacts re-walked the whole scene graph from the root
# on EVERY basic step (ContactTracker.poll -> contact_pairs -> here), and that
# walk is not cheap in the way it looks cheap: every node costs several
# SEQUENTIAL supervisor round-trips (getTypeName, getBaseTypeName, the children
# field, the endPoint field), and a round-trip is serviced at an engine step
# boundary. So the walk costs (number of round-trips) x (engine step time), and
# NEITHER factor is the scene's node count alone.
#
# MEASURED 2026-08-14, machine 9722d23d12a3, heavy mode, GET /sim/contacts:
#
#     rigid    16 nodes, 3 solids, 4 contacts ->  126 ms
#     8-Husky 240 nodes                       -> 4710 ms
#     cloth    10 nodes, 2 solids, 0 contacts -> 4747 ms
#
# The cloth world has 24x FEWER nodes than the Husky world and reports ZERO
# contacts, yet costs the same -- because its engine step is slow, so each
# round-trip is expensive. That is why the old "light mode is for multi-robot
# scenes" framing was wrong: the per-step tracker made heavy mode cost
# 2222 ms/step on a two-static-body cloth world vs 1.9 ms/step on a rigid one.
#
# The scene graph does not change between steps. Cache it.
#
# CORRECTNESS. The supervisor process is restarted on every world load, so the
# cache is born fresh per world and only in-session mutation can stale it:
# invalidate_scene_cache() is called from the spawn and delete handlers. The
# poll backstop is the belt to that braces -- another supervisor-capable
# controller could import or remove a node without telling us, so the cache is
# rebuilt unconditionally every _SOLID_CACHE_MAX_POLLS polls regardless. A
# stale entry degrades safely in any case: getContactPoints on a removed node
# raises and is already swallowed by the per-solid try/except below.
_SOLID_CACHE: list | None = None
_SOLID_CACHE_POLLS = 0
_SOLID_CACHE_MAX_POLLS = 120


_IDENTITY_CACHE: dict[int, tuple] = {}
_ROBOT_INDEX_CACHE: dict[str, str] | None = None
_JOINT_CACHE: list[tuple] | None = None
# Which supervisor the cached views belong to. In production this never changes
# (the supervisor process is restarted per world load), but a module-global
# keyed on NOTHING quietly assumes that forever -- and the harness unit tests,
# which build a fresh fake supervisor per test in one process, proved the
# assumption wrong immediately: three observability tests passed alone and
# failed in-suite, reading the previous test's scene. Bind the cache to its
# owner so a new supervisor can never inherit a stale scene.
_CACHE_OWNER = None


def invalidate_scene_cache() -> None:
    """Drop every cached view of the scene graph. Call after ANY structural
    scene change (spawn, delete). The per-step trackers rebuild on next use."""
    global _SOLID_CACHE, _SOLID_CACHE_POLLS, _ROBOT_INDEX_CACHE, _JOINT_CACHE
    _SOLID_CACHE = None
    _SOLID_CACHE_POLLS = 0
    _ROBOT_INDEX_CACHE = None
    _JOINT_CACHE = None
    _IDENTITY_CACHE.clear()


def _bind_cache_owner(supervisor) -> None:
    """Reset every cached view when the supervisor instance changes."""
    global _CACHE_OWNER
    if _CACHE_OWNER is not supervisor:
        invalidate_scene_cache()
        _CACHE_OWNER = supervisor


def cached_joints(supervisor) -> list[tuple]:
    """[(joint_node, joint_id, jointParameters_node)] for every joint, cached.

    JointLimitTracker.poll ran the SAME root walk this module caches for solids
    -- and called `_walk` directly, so 3b952b61d's cache did not cover it. On a
    robot world the walk is the dominant per-step read path: `_walk` recurses
    through `children` AND `endPoint`, and a joint's endPoint is exactly what
    drags the recursion through the whole robot subtree, one `getTypeName()`
    round-trip per node, every basic step.

    Cached here: the walk, the joint id, and the jointParameters node handle --
    all immutable for the life of the node.
    NOT cached: `position`, obviously, and also `minStop`/`maxStop`, which a
    supervisor could legitimately retune at runtime. Those stay live reads, so
    this cannot make the tracker emit a limit event against a stale limit.
    """
    global _JOINT_CACHE
    _bind_cache_owner(supervisor)
    if _JOINT_CACHE is not None:
        return _JOINT_CACHE
    root = supervisor.getRoot()
    joints: list = []
    _walk(root, _is_joint, joints)
    out: list[tuple] = []
    for j in joints:
        try:
            jid = j.getId()
        except Exception:
            continue
        out.append((j, jid, _joint_parameters(j)))
    _JOINT_CACHE = out
    return out


def _cached_identity(solids: list) -> dict[int, tuple]:
    """{id(node): (def-or-id, name)} for the current cached walk.

    Both values are immutable for the life of a node, so this turns two
    per-solid round-trips per step into two per solid per CACHE REBUILD.
    """
    for s in solids:
        key = id(s)
        if key not in _IDENTITY_CACHE:
            _IDENTITY_CACHE[key] = (_node_def_or_id(s), _sf_string(s, "name"))
    return _IDENTITY_CACHE


def _cached_solids(supervisor) -> list:
    global _SOLID_CACHE, _SOLID_CACHE_POLLS, _ROBOT_INDEX_CACHE, _JOINT_CACHE
    _bind_cache_owner(supervisor)
    if _SOLID_CACHE is not None and _SOLID_CACHE_POLLS < _SOLID_CACHE_MAX_POLLS:
        _SOLID_CACHE_POLLS += 1
        return _SOLID_CACHE
    # Backstop rebuild: drop the derived views too, so a scene change made by
    # someone other than this supervisor cannot leave them permanently stale.
    _ROBOT_INDEX_CACHE = None
    _JOINT_CACHE = None
    _IDENTITY_CACHE.clear()
    root = supervisor.getRoot()
    solids: list = []
    _walk(root, _is_solid, solids)
    _SOLID_CACHE = solids
    _SOLID_CACHE_POLLS = 0
    return solids


def _walk_contacts(supervisor):
    """(solids, paired contacts, solids that answered). No velocity reads."""
    solids = _cached_solids(supervisor)

    # point -> list of (solid identifier, name, raw point). A contact reported by
    # two different bodies is one contact between them.
    by_point: dict[tuple, list] = {}
    queried = 0
    # A solid's DEF/id and its `name` field are FIXED for the life of the node,
    # but both were re-read from the engine on every step -- two more of the
    # sequential round-trips that the measurement above shows are the real
    # currency here (each one is serviced at a step boundary, so on a slow world
    # each costs milliseconds, not microseconds). Resolve them once per cached
    # walk. Keyed by the node's own unique id rather than the Python object, so
    # a re-walk that hands back equivalent wrappers still hits.
    idents = _cached_identity(solids)
    for s in solids:
        try:
            cps = s.getContactPoints(False) or []
        except Exception:
            continue
        queried += 1
        ident, name = idents.get(id(s)) or (_node_def_or_id(s), _sf_string(s, "name"))
        for cp in cps:
            try:
                point = [float(x) for x in cp.point]
            except Exception:
                continue
            entry = by_point.setdefault(_quantize_point(point), [])
            if ident not in [e[0] for e in entry]:
                entry.append((ident, name, point))

    out: list[dict] = []
    for entry in by_point.values():
        point = entry[0][2]
        if len(entry) == 1:
            out.append({
                "a_def": entry[0][0], "a_name": entry[0][1],
                "b_def": None, "b_name": None,
                "point": point, "paired": False,
                # Being explicit beats leaving a null to interpret: the usual
                # cause is a PROTO floor/terrain (`Floor {}`, `UnevenTerrain {}`)
                # whose internals a Supervisor cannot query, so only the robot
                # side of a real contact is visible.
                "note": ("the other body did not report this contact: it is not a "
                         "node this walk can query (typically a PROTO floor or "
                         "terrain). The contact is real; its partner is not "
                         "nameable from the supervisor API."),
            })
            continue
        # >2 is possible where several sub-solids of a compound body report the
        # same world point; emit each distinct pair once.
        for i in range(len(entry)):
            for j in range(i + 1, len(entry)):
                out.append({"a_def": entry[i][0], "a_name": entry[i][1],
                            "b_def": entry[j][0], "b_name": entry[j][1],
                            "point": point, "paired": True})
    return solids, out, queried


def contact_pairs(supervisor) -> list[dict]:
    """Paired contacts only — the per-step path (ContactTracker), so it must not
    pay for the `tracking` block's velocity reads."""
    return _walk_contacts(supervisor)[1]


def list_contacts(supervisor) -> list[dict]:
    """Back-compatible view: just the contact list (see `collect_contacts`)."""
    return contact_pairs(supervisor)


def list_robot_contacts(robots) -> list[dict]:
    """Pair contacts by querying only the supplied Robot subtrees, deeply.

    This is the scalable instrument for a robot-robot-only assertion. A deep
    query includes every colliding link below one Robot, while attributing the
    returned points to that Robot here avoids a separate query for each wheel
    and chassis Solid. A robot-floor point remains an honest unpaired contact;
    a robot-robot point is reported by both queried Robot subtrees and therefore
    pairs exactly as in :func:`list_contacts`.
    """
    by_point: dict[tuple, list] = {}
    for robot in robots:
        try:
            cps = robot.getContactPoints(True) or []
        except Exception:
            continue
        ident = _node_def_or_id(robot)
        name = _sf_string(robot, "name")
        for cp in cps:
            try:
                point = [float(x) for x in cp.point]
            except Exception:
                continue
            entry = by_point.setdefault(_quantize_point(point), [])
            if ident not in [e[0] for e in entry]:
                entry.append((ident, name, point))

    out: list[dict] = []
    for entry in by_point.values():
        point = entry[0][2]
        if len(entry) == 1:
            out.append({
                "a_def": entry[0][0], "a_name": entry[0][1],
                "b_def": None, "b_name": None, "point": point,
                "paired": False,
                "note": ("the other body was outside the queried Robot "
                         "subtrees; the contact is real but its partner is not "
                         "needed by this robot-robot-only instrument"),
            })
            continue
        for i in range(len(entry)):
            for j in range(i + 1, len(entry)):
                out.append({"a_def": entry[i][0], "a_name": entry[i][1],
                            "b_def": entry[j][0], "b_name": entry[j][1],
                            "point": point, "paired": True})
    return out


def world_info_node(supervisor):
    """The scene's WorldInfo node, or None."""
    root = supervisor.getRoot()
    for child in _children_of(root):
        try:
            if child.getTypeName() == "WorldInfo":
                return child
        except Exception:
            continue
    return None


# Velocity magnitudes under which a body is reported as "at rest". Fixed
# constants, NOT WorldInfo.physicsDisable*Threshold: those fields belong to a
# body-sleep mechanism that no longer exists (nothing in the engine reads them),
# so sourcing a threshold from them would imply they still govern something.
AT_REST_LINEAR_M_S = 0.01
AT_REST_ANGULAR_RAD_S = 0.01

# Depth to which a Solid's ancestors are consulted for an inherited
# `physicsBackend` pin. Mirrors OmSolid::effectivePhysicsBackendName, which
# walks to the outermost ancestor Solid/Robot (a URDFRobot-generated chassis
# inherits the outer Robot's choice), so a chassis pinned via its Robot is
# reported too.
_BACKEND_PIN_ANCESTOR_DEPTH = 12


def _effective_backend_pin(node) -> str | None:
    """The `physicsBackend` value governing this node, walking to ancestors.

    Returns the lower-cased field value ("ode" / "newton" / "auto" / ...), or
    None when neither the node nor any ancestor declares one. Read from the
    scene tree -- never inferred.
    """
    cur = node
    for _ in range(_BACKEND_PIN_ANCESTOR_DEPTH):
        if cur is None:
            return None
        value = _sf_string(cur, "physicsBackend")
        if value:
            value = value.strip().lower()
            if value and value != "auto":
                return value
        try:
            cur = cur.getParentNode()
        except Exception:
            return None
    return None


def _has_collision_or_mass(node) -> bool:
    """True when the node declares a boundingObject or a Physics node, i.e.
    when it is a body somebody expects to collide or move. Matches the engine's
    own gate on the ODE-pin warning (OmSolid.cpp ~3170), so a visual-only prop
    is not reported as a problem."""
    for field_name in ("boundingObject", "physics"):
        try:
            f = node.getField(field_name)
            if f is not None and f.getSFNode() is not None:
                return True
        except Exception:
            continue
    return False


def _contact_tracking_scope(supervisor, solids, queried: int,
                            light: bool = False) -> dict:
    """What the contact scan covered, and every real reason its result can be
    empty. Everything here is read from the live scene, never guessed.

    ⚠ Deliberately contains NO body-sleep model. See `collect_contacts` -- the
    previous version of this function invented one, and it pointed an agent
    away from the actual defect.
    """
    inert_pinned: list[dict] = []
    at_rest: list[str] = []
    for s in solids:
        pin = _effective_backend_pin(s)
        if pin == "ode" and _has_collision_or_mass(s):
            inert_pinned.append({"def": _node_def_or_id(s),
                                 "name": _sf_string(s, "name"),
                                 "physics_backend": pin})
        if s.getField("physics") is None:
            continue
        try:
            if s.getField("physics").getSFNode() is None:
                continue
            v = s.getVelocity()
        except Exception:
            continue
        if v is None or len(v) < 6:
            continue
        lin = (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5
        ang = (v[3] ** 2 + v[4] ** 2 + v[5] ** 2) ** 0.5
        if lin <= AT_REST_LINEAR_M_S and ang <= AT_REST_ANGULAR_RAD_S:
            at_rest.append(_node_def_or_id(s))

    # Mirror the shape the --light branch of sim_grips already gets right:
    # say what was measured and what an empty result does NOT mean, ALWAYS,
    # rather than attaching a conditional caveat that a caller can miss.
    reasons = [
        {"cause": "solid_pinned_physics_backend_ode",
         "observed_here": bool(inert_pinned),
         "detail": ("a Solid whose own or whose ancestor's `physicsBackend` field is "
                    "\"ode\" has NO physics at all -- no gravity, no contact -- because "
                    "ODE was deleted and the field no longer selects an engine. It can "
                    "never appear in this list. The Solids in `inert_pinned_solids` were "
                    "READ from the scene tree, not inferred."),
         "check": "GET /scene/node/<def> -> fields.physics_backend, and the SOLID_ODE_PIN_INERT "
                  "diagnostic on the load"},
        {"cause": "no_physics_backend_available",
         "observed_here": None,
         "detail": ("if the Newton runtime did not come up, NOTHING in the world is "
                    "simulated and no Solid can report a contact. The supervisor cannot "
                    "see this from the scene tree; the engine reports it on the load."),
         "check": "GET /capabilities -> physics (source must be \"sidecar\"), and the "
                  "NO_PHYSICS_BACKEND / NEWTON_RUNTIME_ABSENT / NEWTON_RUNTIME_BROKEN "
                  "diagnostics"},
        {"cause": "solid_never_registered_with_the_backend",
         "observed_here": None,
         "detail": ("a Solid the backend never registered (a capability-gated "
                    "articulation, an unregistered static collider) reads through a "
                    "permanently-empty bridge list and reports zero contacts for ever. "
                    "The engine's own census is the check."),
         "check": "the NEWTON_ZERO_DYNAMIC_BODIES / NEWTON_STATICS_NOT_REGISTERED "
                  "diagnostics on the load"},
        {"cause": "native_contact_readback_disabled",
         "observed_here": None,
         "detail": ("OMNISIM_NEWTON_NATIVE_CONTACTS=0 makes getContactPoints blind on "
                    "Newton-backed Solids: its empty answer then means \"cannot see\", "
                    "not \"nothing is touching\". On by default since 2026-08-07."),
         "check": "the CONTACT_QUERIES_BLIND diagnostic on the load"},
        {"cause": "partner_is_not_a_walkable_solid",
         "observed_here": None,
         "detail": ("a contact with the implicit ground plane the backend adds at z=0, or "
                    "with the internals of a PROTO floor/terrain, is reported as a "
                    "half-contact (paired=false) or not at all -- the partner is not a "
                    "node this walk can name."),
         "check": "contacts[].paired and contacts[].note"},
        {"cause": "genuinely_not_touching",
         "observed_here": None,
         "detail": "the ordinary case. It is indistinguishable from the above WITHOUT the "
                   "checks named alongside them, which is the whole point of this block."},
    ]

    scope = {
        "scope": "every Solid / Robot node in the scene, walked per call",
        "measured": True,
        "solids_walked": len(solids),
        "solids_answering": queried,
        "name_filter": None,
        "live_contacts_only": True,
        # NEVER "this list is complete". The scan can only report what the
        # engine answers for the nodes it can walk.
        "completeness": ("UNKNOWN. This is what the engine answered this step for the "
                         "nodes this walk can query. An empty list is NOT proof of no "
                         "contact and a non-empty list is not proof of the full set; "
                         "see empty_set_reasons."),
        "bodies_at_rest": at_rest[:64],
        "bodies_at_rest_total": len(at_rest),
        "bodies_at_rest_note": (
            "informational only. A body at rest DOES report its contacts -- native "
            "contact readback is on by default -- so this list is NOT a reason for an "
            "empty result. There is no body-sleep mechanism in this engine."),
        # Back-compat alias: clients written against the sleep-era response read
        # `idle_bodies`. Same data, and the note above says what it does not mean.
        "idle_bodies": at_rest[:64],
        "idle_bodies_total": len(at_rest),
        "inert_pinned_solids": inert_pinned[:64],
        "inert_pinned_solids_total": len(inert_pinned),
        "empty_set_is_proof_of_no_contact": False,
        "empty_set_reasons": reasons,
        "wake_parameter": (
            "?wake=1 is a NO-OP kept for compatibility. It used to write "
            "WorldInfo.physicsDisableTime and advance two steps to 'clear a sleep "
            "timer'; that field has no reader in the engine and there is no body "
            "sleep, so it measured nothing while mutating the world during a "
            "documented read. It now changes nothing and costs nothing."),
    }
    if light:
        # /sim/contacts itself is UNAFFECTED by --light (it walks the scene per
        # call and never reads a tracker). Only the derived streams are gone --
        # scope it precisely, because over-claiming this once recommended a
        # ~790x-cost reload for nothing.
        scope["light_mode"] = {
            "contacts_affected": False,
            "detail": ("the supervisor is running with --light. This contact WALK is "
                       "unaffected. What is missing is everything derived from the "
                       "per-step trackers: the contact.* / grip.* / joint.limit_hit "
                       "event types are not produced, and GET /sim/grips reports "
                       "tracking.enabled=false."),
        }
    return scope


# ---------------------------------------------------------------------------
# /robot/<def>/devices
# ---------------------------------------------------------------------------


def list_devices(supervisor, robot_def: str) -> dict:
    """Enumerate the devices of a robot via scene-tree introspection.

    The Supervisor cannot use `Robot.getDeviceByIndex()` for sibling
    robots — that API only works for the controller's own Robot. Instead
    we walk the subtree and report every node with a `name` field whose
    typename is a recognized device type. This is rougher than the
    runtime API but it works for any robot the supervisor can see.
    """
    with paused_reads(supervisor):
        return _list_devices_paused(supervisor, robot_def)


def _list_devices_paused(supervisor, robot_def: str) -> dict:
    robot = supervisor.getFromDef(robot_def)
    if robot is None:
        raise KeyError(f"no node with DEF '{robot_def}'")
    device_typenames = {
        "Camera", "RangeFinder", "Lidar", "DistanceSensor", "TouchSensor",
        "Accelerometer", "Gyro", "Compass", "GPS", "InertialUnit",
        "LightSensor", "Receiver", "Emitter", "Radar", "Speaker",
        "Microphone", "Display", "LED", "RotationalMotor", "LinearMotor",
        "Brake", "PositionSensor",
    }

    def _is_device(n):
        if n is None:
            return False
        try:
            return n.getTypeName() in device_typenames
        except Exception:
            return False

    devs: list = []
    _walk(robot, _is_device, devs)
    out: list[dict] = []
    for d in devs:
        out.append({
            "name": _sf_string(d, "name"),
            "type": d.getTypeName(),
        })
    return {"robot": robot_def, "devices": out}


# ---------------------------------------------------------------------------
# /sim/grips
# ---------------------------------------------------------------------------


def detect_grips(contact_pairs: list[tuple[str, str]],
                 robot_subtree_index: dict[str, str]) -> list[dict]:
    """Return grips inferred from a contact-pair list.

    `contact_pairs`: list of `(a_id, b_id)` tuples (DEF-or-id strings).
    `robot_subtree_index`: map node-id -> its containing robot def, for
    every solid the supervisor knows about. A grip is heuristically
    "object touched by ≥2 distinct solids that share a robot ancestor."

    This implementation is intentionally simple and does NOT try to
    distinguish a gripper from any other multi-finger contact; agents
    that need stricter grip semantics can layer a domain check on top.
    Phase 3 introduces a `since_t_ms` field tracked outside this pure
    function — see GripTracker in event_bus.py.
    """
    # object -> set of (robot_def, finger_def)
    fingers_per_object: dict[str, dict[str, set[str]]] = {}
    for a, b in contact_pairs:
        a_robot = robot_subtree_index.get(a)
        b_robot = robot_subtree_index.get(b)
        if a_robot and not b_robot:
            fingers_per_object.setdefault(b, {}).setdefault(a_robot, set()).add(a)
        if b_robot and not a_robot:
            fingers_per_object.setdefault(a, {}).setdefault(b_robot, set()).add(b)
    grips: list[dict] = []
    for obj, by_robot in fingers_per_object.items():
        for robot_def, fingers in by_robot.items():
            if len(fingers) >= 2:
                grips.append({
                    "gripper_def": robot_def,
                    "held_def": obj,
                    "fingers": sorted(fingers),
                })
    return grips


def build_robot_subtree_index(supervisor) -> dict[str, str]:
    """Map every solid's def-or-id to the def-or-id of its containing Robot.

    ⚠ The docstring used to say "built once per snapshot/event-poll", and that
    was true of the snapshot path but NOT of the caller that dominates cost:
    the main loop calls this through grip_tracker.poll() on EVERY basic step.
    It is the most expensive walk in the file -- a full _walk for robots, then
    another _walk per robot for solids, plus a _node_def_or_id round-trip per
    node -- and every one of those round-trips is serviced at an engine step
    boundary. It is cached for exactly the reasons given at _SOLID_CACHE, and
    shares that cache's invalidation, so a spawn or delete rebuilds both.
    """
    global _ROBOT_INDEX_CACHE
    _bind_cache_owner(supervisor)
    if _ROBOT_INDEX_CACHE is not None:
        return _ROBOT_INDEX_CACHE
    _ROBOT_INDEX_CACHE = _build_robot_subtree_index_uncached(supervisor)
    return _ROBOT_INDEX_CACHE


def _build_robot_subtree_index_uncached(supervisor) -> dict[str, str]:
    root = supervisor.getRoot()
    robots: list = []
    _walk(root, _is_robot, robots)
    index: dict[str, str] = {}
    for robot in robots:
        robot_id = _node_def_or_id(robot)
        solids: list = []
        _walk(robot, _is_solid, solids)
        for s in solids:
            sid = _node_def_or_id(s)
            # Don't overwrite if a solid is its own robot (self entries
            # break finger-vs-non-finger discrimination).
            if sid == robot_id:
                continue
            # First-writer-wins: an inner robot inside an outer robot
            # claims its own subtree first (DFS from outer).
            index.setdefault(sid, robot_id)
    return index
