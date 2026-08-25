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

"""harness_supervisor — generic supervisor used by the OmniSim agent-facing
validation harness.

Injected into any user world by the harness via a sibling file (see
scripts/harness/README.md). Listens on a local TCP socket for
length-prefixed JSON commands from the harness process and proxies them
onto the Supervisor API.

Wire protocol
-------------
Each frame is a 4-byte big-endian length followed by a UTF-8 JSON payload.

Request:   {"id": <int>, "cmd": "<name>", "args": {...}}
Response:  {"id": <int>, "ok": true, "result": {...}}
       or  {"id": <int>, "ok": false, "error": "<message>"}

Commands
--------
ping              -> {}
sim_state         -> {sim_time_ms, basic_time_step_ms, ...}
capabilities      -> {light, basic_time_step_ms, commands, event_types{...}, snapshots}
                     event_types is CROSS-CHECKED against the emit() call sites
                     (event_bus.verify_event_types), and marks which types the
                     running configuration actually produces (--light suppresses
                     the contact / joint-limit / grip ones).
step              -> {sim_time_ms, advanced_to_ms}  args: {steps?: int}
reset             -> {sim_time_ms, restored, verification}
                     args: {restore?: str|null, verify?: bool, settle_steps?: int}
                     Rewinds the clock AND restores '__init__', the engine's own
                     parse-time state (every node's authored pose), unless
                     restore is null — simulationReset() alone does not
                     restore node poses.
sim_snapshot      -> {name, sim_time_ms, sampled_nodes, names}   args: {name?: str}
                     Node.saveState on the scene root (recursive, engine-side).
sim_restore       -> {name, verification:{vs_snapshot, moved_by_restore}, ...}
                     args: {name?: str, settle_steps?: int}
                     Refuses a name this process never saved: OmPose's saved-pose
                     map default-constructs a ZERO vector on a miss, so restoring
                     an unknown name would teleport the scene to the origin.
sim_snapshots     -> {snapshots: [{name, sim_time_ms, sampled_nodes, age_s}]}
scene_spawn       -> {def, id, type, position, index, children_before/after, verification}
                     args: {vrml: str, parent?: str(DEF), index?: int, def?: str,
                            settle_steps?: int}
                     Field.importMFNodeFromString into the parent's children.
scene_delete      -> {removed: [...], missing: [...], verification}
                     args: {def?: str, defs?: [str], settle_steps?: int}
scene_set_pose    -> {def, position_before, position, verification}
                     args: {def: str, translation?: [x,y,z], rotation?: [ax,ay,az,a],
                            reset_physics?: bool, settle_steps?: int}
scene_set_poses   -> {changes: [...], verification}
                     args: {changes: [{def, translation?, rotation?}],
                            reset_physics?: bool, settle_steps?: int}
                     Validates the whole batch before mutating, resets every
                     moved body, settles once, and rolls back on a local error.
world_load        -> {path}                          args: {path: str}     hot reload
screenshot        -> {path}                          args: {path: str, quality?: int}
scene_tree        -> {nodes: [{def, type, position, ...}, ...]}   args: {bounds?: bool}
scene_node        -> {def, type, fields: {...}, position, orientation}   args: {def: str, bounds?: bool}
set_viewpoint     -> {position, orientation}         args: {position: [x,y,z], orientation: [ax,ay,az,angle]}
get_viewpoint     -> {position, orientation, orientation_matrix, fieldOfView, near, far,
                      follow, followType, followSmoothness, projectionMode, exposure}
scene_bounds      -> {bounds: {<def>: {center, radius, bbox_min, bbox_max, ...}}, count}
                     args: {defs?: [str]}     world-space geometric bounds per node
bounds_probe      -> {center, radius, distance, residual, ...}   args: {def: str, aspect?: float}
                     SLOW (~2-6 s) exactness oracle: inverts the engine's own
                     moveViewpoint bounding-sphere fit. Restores the camera pose.
damage_state      -> {robot, attached, parts, damage:{part:{state,hp,hp_max,...}}, game_over, ...}
damage_events     -> {events: [...], last_step_id, events_total}         args: {since?: int, limit?: int}
                     events have type:"impact" or type:"state_transition";
                     impact = {step_id, sim_time_ms, part, impulse_J, point, other};
                     state_transition = {step_id, sim_time_ms, part, from_state, to_state, hp, trigger_impulse_J}
damage_reset      -> {ok: true}                       heals all parts to pristine without resetting the sim
damage_inject     -> {state, hp, hp_max, ...}                          args: {part: str, hp_delta?: float, state?: str}
                     test/debug hook to set a part's state directly without the contact pipeline
robots_list       -> {robots: [{def, name, model, controller, type, position, orientation, num_joints}]}
robot_joints      -> {robot, joints: [{name, type, position, velocity, lower, upper, hit_limit}]}   args: {def: str}
robot_devices     -> {robot, devices: [{name, type}]}                  args: {def: str}
set_joint_positions -> {robot, joints: {name: {requested, commanded, clamped,
                     position_before, achieved, error, moved,
                     position_controllable, limits, note?}}, verification}
                     args: {def: str, joints: {name: rad_or_m}, settle_steps?: int}
                     Settle-and-verify joint position targets. NOT a teleport:
                     Node.setJointPosition also re-pins the motor's PD target
                     (OmJoint.cpp), so the joint converges over the settled
                     steps; targets beyond the joint's hard stops are clamped
                     and flagged, and a limit-less motor (velocity wheel,
                     ke=0 — position targets ignored by the physics) is
                     reported per joint as position_controllable: false.
solve_ik          -> {robot, effector, solved_joints: [{name, node_id, appliable}],
                     results: [{target, residual_m, joints: {name: angle}}],
                     solve_ms, verification}
                     args: {def: str, effector: str(DEF of the end-effector
                            Solid), targets: [[x,y,z], ...],
                            rotations?: [[qx,qy,qz,qw], ...],
                            tool_offset?: [x,y,z], iterations?: int}
                     Batched IK PREVIEW against the live Newton model
                     (World.solve_ik). PURE READ: nothing moves; angles come
                     back clamped to the authored limits, keyed by the same
                     joint names set_joint_positions accepts, with a
                     per-target FK-measured residual in metres. First call
                     per world compiles a warp kernel (seconds).
sim_contacts      -> {contacts: [{a_def, b_def, point}]}
sim_grips         -> {grips: [{gripper_def, held_def, since_t_ms}]}
events_drain      -> {events: [...], next_seq, total, dropped, buffered}
                     args: {since?: int, limit?: int, types?: [str]}

The bind host/port can be overridden via OMNISIM_HARNESS_SUPERVISOR_HOST
and OMNISIM_HARNESS_SUPERVISOR_PORT environment variables (defaults
127.0.0.1:6790). The robot name the damage tracker looks for can be set
via OMNISIM_HARNESS_DAMAGE_ROBOT (default 'husky'); if no robot of that
name is in the world, the tracker idles and damage_* commands return
the empty/idle state.
"""

from __future__ import annotations

import inspect
import json
import math
import os
import re
import select
import socket
import struct
import sys
import time
import traceback

from omnisim import Supervisor

from damage_tracker import DamageTracker
import event_bus
from event_bus import ContactTracker, EventBus, GripTracker, JointLimitTracker
import geometry
import observe

# Optional: mirror stderr to a file (OMNISIM_SUPERVISOR_STDERR_LOG=path).
# Useful when the OmniSim console isn't accessible -- e.g. debugging FPS
# issues remotely or post-mortem grep.
_STDERR_LOG_PATH = os.environ.get("OMNISIM_SUPERVISOR_STDERR_LOG", "")
if _STDERR_LOG_PATH:
    class _StderrTee:
        def __init__(self, stream, path):
            self._stream = stream
            try:
                self._file = open(path, "w", buffering=1, encoding="utf-8")
            except OSError:
                self._file = None
        def write(self, s):
            self._stream.write(s)
            if self._file is not None:
                self._file.write(s)
        def flush(self):
            self._stream.flush()
            if self._file is not None:
                self._file.flush()
    sys.stderr = _StderrTee(sys.stderr, _STDERR_LOG_PATH)

HOST = os.environ.get("OMNISIM_HARNESS_SUPERVISOR_HOST", "127.0.0.1")
PORT = int(os.environ.get("OMNISIM_HARNESS_SUPERVISOR_PORT", "6790"))
DAMAGE_ROBOT_NAME = os.environ.get("OMNISIM_HARNESS_DAMAGE_ROBOT", "husky")
# Optional secondary robots tracked alongside the primary. Each name in
# the comma-separated list gets its own DamageTracker with its own
# state, contact polling, scene mutations (Phase 7 appearance, Phase 9
# detach, etc.). Wire-protocol damage_* commands address the PRIMARY
# tracker only; secondaries run for visual symmetry (e.g. so a head-on
# crash damages BOTH huskies on screen, not just the tracked one).
DAMAGE_EXTRA_ROBOTS = [
    n.strip() for n in os.environ.get(
        "OMNISIM_HARNESS_DAMAGE_EXTRA_ROBOTS", "husky_b").split(",")
    if n.strip()
]

# --light (passed via controllerArgs by the harness's injection stanza) drops
# the per-step contact / joint-limit / grip trackers. Module-level so the
# `capabilities` command can report it honestly — an agent that filters
# /sim/events on a suppressed type otherwise sees an empty stream with no
# explanation.
LIGHT_MODE = "--light" in sys.argv

# Named engine-side state snapshots taken in THIS supervisor process, keyed by
# name. The value carries a pose fingerprint used only to *verify* a restore;
# the state itself lives in the engine (OmNode::save / ::reset, recursive over
# the scene via OmGroup::save).
#
# ⚠️ The registry is load-bearing, not bookkeeping. `Node.loadState(name)`
# restores `mSavedTranslations[name]`, a QMap lookup that DEFAULT-CONSTRUCTS a
# zero vector on a miss (src/omnisim/nodes/OmPose.hpp) — so restoring a name
# that was never saved would silently teleport the whole scene to the origin.
# `sim_restore` therefore refuses any name not in here.
_SNAPSHOTS: dict[str, dict] = {}

# The engine populates one state for free, and it is the one an agent actually
# wants: `OmNode`'s constructor sets `mCurrentStateId = "__init__"`
# (src/omnisim/vrml/OmNode.cpp:161) and `OmPose`'s constructor saves the
# node's translation/rotation under the current state id — so `"__init__"`
# holds every node's pose *as the .wbt authored it*, populated at parse time.
#
# This matters because a supervisor-taken "initial" snapshot is NOT the
# authored state: the engine free-runs (`--mode=fast`, `synchronization
# FALSE`), so by the time the injected controller's first step runs, a dropped
# body has already fallen. Measured on lane3_drive.wbt: BALL is authored at
# z = 1.0 and reads z = 0.1 on the supervisor's very first tick.
ENGINE_INIT_STATE = "__init__"


def recv_exact(sock: socket.socket, n: int) -> bytes | None:
    chunks: list[bytes] = []
    remaining = n
    while remaining > 0:
        try:
            chunk = sock.recv(remaining)
        except OSError:
            # Aborted/reset connections (e.g. the harness timing out an RPC
            # and closing its socket) are a disconnect, not a controller
            # crash — the caller drops this client and keeps serving.
            return None
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_frame(sock: socket.socket) -> dict | None:
    header = recv_exact(sock, 4)
    if header is None:
        return None
    (length,) = struct.unpack(">I", header)
    if length == 0 or length > 16 * 1024 * 1024:
        return None
    payload = recv_exact(sock, length)
    if payload is None:
        return None
    try:
        return json.loads(payload.decode("utf-8"))
    except json.JSONDecodeError:
        return None


def write_frame(sock: socket.socket, obj: dict) -> bool:
    body = json.dumps(obj).encode("utf-8")
    try:
        sock.sendall(struct.pack(">I", len(body)) + body)
        return True
    except OSError:
        return False


def node_summary(node) -> dict:
    """Identity + world pose of one node.

    Pose reads are skipped for nodes the engine cannot pose anyway (a
    non-Pose node's getPosition() costs a round-trip, warns into the world
    log, and returns NaN — sanitized to null downstream): the classification
    lives in observe.node_is_posed, and a type the tables don't know is
    classified from its first measured read, so a wrong static entry cannot
    fabricate a null for a node that really has a pose. Pose VALUES are
    re-read every call — only the type-level "has a pose" verdict is cached.
    """
    if node is None:
        return {}
    type_name = node.getTypeName()
    out: dict = {"type": type_name}
    def_name = node.getDef()
    if def_name:
        out["def"] = def_name
    try:
        # The node id is the key the bounds index is built on, and it is the
        # only stable handle for a DEF-less node.
        out["id"] = node.getId()
    except Exception:
        pass
    verdict = observe.node_is_posed(node, type_name)
    if verdict is False:
        # The engine would answer NaN (sanitized to null downstream) and warn.
        out["position"] = [None, None, None]
        out["orientation"] = [None] * 9
        return out
    position = None
    try:
        position = list(node.getPosition())
        out["position"] = position
    except Exception:
        pass
    try:
        # 3x3 rotation matrix flattened
        out["orientation"] = list(node.getOrientation())
    except Exception:
        pass
    if verdict is None and position is not None:
        observe.record_pose_measurement(type_name, position)
    return out


def walk_scene_tree(root, bounds_by_id: dict | None = None) -> list[dict]:
    """Flatten the scene into a list of node summaries.

    ``bounds_by_id`` (from ``geometry.bounds_index``) attaches each node's
    world-space geometric bounds. It is opt-in because computing it walks
    every geometry node (and reads mesh files) — see the ``bounds`` arg on the
    ``scene_tree`` command.
    """
    nodes: list[dict] = []

    def visit(node, parent_def):
        if node is None:
            return
        summary = node_summary(node)
        summary["parent_def"] = parent_def
        if bounds_by_id is not None:
            b = bounds_by_id.get(summary.get("id"))
            if b is not None:
                summary["bounds"] = b
        nodes.append(summary)
        own_def = summary.get("def")
        for field_name in ("children", "endPoint"):
            field = node.getField(field_name)
            if field is None:
                continue
            try:
                if field_name == "endPoint":
                    visit(field.getSFNode(), own_def or parent_def)
                else:
                    count = field.getCount()
                    for i in range(count):
                        visit(field.getMFNode(i), own_def or parent_def)
            except Exception:
                continue

    visit(root, None)
    return nodes


def find_node_by_def(supervisor: Supervisor, def_name: str):
    return supervisor.getFromDef(def_name)


def find_viewpoint(supervisor: Supervisor):
    """Walk the root group looking for the Viewpoint node. There is no
    direct Supervisor.getViewpoint() in the Python binding, so we scan the
    top-level children of the root.
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
        if node is not None and node.getTypeName() == "Viewpoint":
            return node
    return None


# Field-type -> reader map for field_value(), built once: importing Field and
# allocating eight lambdas per field read was measurable once the engine
# round-trips stopped dominating (paused reads are ~0.15 ms each).
_FIELD_VALUE_READERS: dict | None = None


def _field_value_readers() -> dict:
    global _FIELD_VALUE_READERS
    if _FIELD_VALUE_READERS is None:
        from omnisim import Field

        _FIELD_VALUE_READERS = {
            Field.SF_BOOL: lambda f: f.getSFBool(),
            Field.SF_INT32: lambda f: f.getSFInt32(),
            Field.SF_FLOAT: lambda f: f.getSFFloat(),
            Field.SF_VEC2F: lambda f: list(f.getSFVec2f()),
            Field.SF_VEC3F: lambda f: list(f.getSFVec3f()),
            Field.SF_ROTATION: lambda f: list(f.getSFRotation()),
            Field.SF_COLOR: lambda f: list(f.getSFColor()),
            Field.SF_STRING: lambda f: f.getSFString(),
        }
    return _FIELD_VALUE_READERS


def field_value(field) -> object:
    if field is None:
        return None
    try:
        ftype = field.getType()
    except Exception:
        return None
    try:
        reader = _field_value_readers().get(ftype)
        if reader is not None:
            return reader(field)
    except Exception:
        pass
    return None


def _geometry_summary(node, depth: int = 0) -> dict | None:
    """A shallow description of a boundingObject / geometry subtree.

    Not the full subtree: the type plus whatever size-ish fields that type
    carries, recursing through the wrappers a boundingObject is normally
    written with (`Transform`/`Pose` -> `children`, `Shape` -> `geometry`,
    `Group` -> children). Enough to answer "is the collision surface the right
    shape and size", which is the question a fall-through bug turns on.
    """
    if node is None or depth > 4:
        return None
    try:
        type_name = node.getTypeName()
    except Exception:  # noqa: BLE001
        return None
    out: dict = {"type": type_name}
    for fname in ("size", "radius", "height", "translation", "rotation", "scale"):
        v = field_value(node.getField(fname))
        if v is not None:
            out[fname] = v
    # Wrappers: descend one level so `boundingObject Transform { children [ Box ] }`
    # still reports the Box.
    inner = node.getField("geometry")
    if inner is not None:
        try:
            child = _geometry_summary(inner.getSFNode(), depth + 1)
        except Exception:  # noqa: BLE001
            child = None
        if child is not None:
            out["geometry"] = child
    kids = node.getField("children")
    if kids is not None:
        try:
            count = min(kids.getCount(), 4)
        except Exception:  # noqa: BLE001
            count = 0
        described = []
        for i in range(count):
            try:
                child = _geometry_summary(kids.getMFNode(i), depth + 1)
            except Exception:  # noqa: BLE001
                child = None
            if child is not None:
                described.append(child)
        if described:
            out["children"] = described
    return out


def _collision_and_mass(node) -> dict:
    """`boundingObject` / `physics` presence + summary for one node.

    These two fields decide whether a node collides and whether it moves, and
    they were the two the field dump did NOT report: an agent debugging a body
    that falls through the floor got `physics field present: False` for a floor
    whose real defect was a MISSING boundingObject, with no way to tell the two
    apart. `field_value()` cannot help -- both are SFNode fields, which it maps
    to None -- so they need their own reporting.
    """
    out: dict = {}
    bo_field = node.getField("boundingObject")
    bo_node = None
    if bo_field is not None:
        try:
            bo_node = bo_field.getSFNode()
        except Exception:  # noqa: BLE001
            bo_node = None
    out["boundingObject"] = {
        "field_exists": bo_field is not None,
        "present": bo_node is not None,
        "summary": _geometry_summary(bo_node),
        # The note is only meaningful for a node that COULD carry one: a
        # Viewpoint or a Shape has no boundingObject field and is not "missing"
        # a collision surface.
        "note": None if (bo_node is not None or bo_field is None) else (
            "no collision surface: a Solid collides ONLY through its "
            "boundingObject -- visual `children` geometry is never collidable"),
    }

    ph_field = node.getField("physics")
    ph_node = None
    if ph_field is not None:
        try:
            ph_node = ph_field.getSFNode()
        except Exception:  # noqa: BLE001
            ph_node = None
    physics: dict = {
        "field_exists": ph_field is not None,
        "present": ph_node is not None,
        "note": None if (ph_node is not None or ph_field is None) else (
            "no Physics node: this body is STATIC (immovable, but still "
            "collidable if it has a boundingObject)"),
    }
    if ph_node is not None:
        for fname in ("mass", "density"):
            v = field_value(ph_node.getField(fname))
            if v is not None:
                physics[fname] = v
        for fname in ("centerOfMass", "inertiaMatrix"):
            f = ph_node.getField(fname)
            if f is None:
                continue
            try:
                physics[fname] = [list(f.getMFVec3f(i)) for i in range(f.getCount())]
            except Exception:  # noqa: BLE001
                pass
    out["physics"] = physics

    # THE THIRD FIELD THAT DECIDES WHETHER A NODE IS SIMULATED, and it was not
    # reported either. `physicsBackend "ode"` parses fine and yields NO physics:
    # ODE was deleted (bdc02139) so the field selects an inert dispatcher whose
    # every verb returns -1 -- the node gets no gravity and no contact while the
    # world loads clean. A boundingObject + a Physics node are then both
    # "present" and the body still never moves, which is exactly the reading an
    # agent cannot make from the two fields above.
    backend = observe._effective_backend_pin(node)
    out["physics_backend"] = {
        "declared": _sf_string_or_none(node, "physicsBackend"),
        "effective": backend or "auto",
        "inert": backend == "ode",
        "note": ("this node has NO physics: `physicsBackend \"ode\"` (declared here or "
                 "inherited from an ancestor Solid/Robot) no longer selects an engine -- "
                 "ODE was removed and Newton is the only backend -- so the node gets no "
                 "gravity and no contact, and GET /sim/contacts can never report it. "
                 "Delete the field to simulate it.") if backend == "ode" else None,
    }
    return out


def _sf_string_or_none(node, field_name: str):
    try:
        f = node.getField(field_name)
        if f is None:
            return None
        return f.getSFString() or None
    except Exception:  # noqa: BLE001
        return None


def node_detail(node, with_bounds: bool = False) -> dict:
    summary = node_summary(node)
    if with_bounds:
        try:
            b = geometry.bounds_for_subtree(node)
        except Exception as exc:  # noqa: BLE001
            b = None
            summary["bounds_error"] = str(exc)
        if b is not None:
            summary["bounds"] = b
    fields: dict = {}
    # Best-effort field dump: walk known field names. The full field list isn't
    # cheaply enumerable from Python, so we expose what callers can see by
    # name. Future revisions can add an explicit field-introspection RPC.
    for fname in ("name", "controller", "translation", "rotation", "scale", "model"):
        f = node.getField(fname)
        v = field_value(f)
        if v is not None:
            fields[fname] = v
    # boundingObject / physics are SFNode fields, so field_value() returns None
    # for them and the dump above silently omitted BOTH -- the two fields that
    # decide whether a node collides and whether it moves. They get explicit
    # presence + summary reporting instead.
    fields.update(_collision_and_mass(node))
    summary["fields"] = fields
    contacts: list[list[float]] = []
    try:
        for cp in node.getContactPoints(False) or []:
            contacts.append(list(cp.point))
    except Exception:
        pass
    if contacts:
        summary["contact_points"] = contacts
    else:
        # An empty list here is ambiguous for the same reason /sim/contacts is
        # -- but NOT because of a body-sleep timer. That model was fiction (no
        # ODE, and WorldInfo.physicsDisableTime has no reader in the engine),
        # and it named a false cause for the symptom a real defect produces.
        summary["contact_points_note"] = (
            "no contact points reported for this node this step. NOT proof of no "
            "contact, and NOT a sleeping body -- this engine has no body-sleep "
            "mechanism. The real causes are enumerated in GET /sim/contacts -> "
            "tracking.empty_set_reasons; the first one to check is on this very "
            "response: fields.physics_backend == \"ode\" means the node has no "
            "physics at all and can never report a contact.")
    return summary


class CommandError(Exception):
    pass


# ---------------------------------------------------------------------------
# Scene mutation (spawn / delete / set_pose) and state snapshots
# ---------------------------------------------------------------------------
#
# Everything below is a thin wrapper over entry points that have shipped in the
# controller binding all along and were reachable from no HTTP verb:
# Field.importMFNodeFromString, Node.remove, Field.setSFVec3f/setSFRotation,
# Node.resetPhysics, Node.saveState/loadState.
#
# The one engine behaviour that shapes all of them: **a supervisor write is
# queued and applied by the engine on its next step**, so a read-back taken in
# the same RPC can legitimately still show the old value. Each mutation
# therefore takes `settle_steps` and reports how many it used, and the
# verification block says what was actually observed rather than asserting the
# write worked.


def _spawn_parent(supervisor: Supervisor, parent_def: str | None):
    """Return (parent_node, children_field) for a spawn target."""
    if parent_def:
        parent = find_node_by_def(supervisor, parent_def)
        if parent is None:
            raise CommandError(
                f"no node with DEF {parent_def!r} to spawn into "
                "(GET /scene/tree lists the DEFs that exist)")
    else:
        parent = supervisor.getRoot()
        if parent is None:
            raise CommandError("scene root is unavailable")
    field = parent.getField("children")
    if field is None:
        raise CommandError(
            f"{'DEF ' + parent_def if parent_def else 'the scene root'} has no "
            "'children' field, so nothing can be spawned into it")
    return parent, field


_DEF_PREFIX_RE = re.compile(r"^\s*DEF\s+([^\s{]+)")


def def_in_vrml(vrml: str) -> str | None:
    """The DEF name of a `DEF NAME Type { ... }` node string, if it has one."""
    m = _DEF_PREFIX_RE.match(vrml or "")
    return m.group(1) if m else None


def _skip_quoted(text: str, i: int) -> int:
    """Index just past the string literal starting at `text[i] == '\"'`."""
    i += 1
    n = len(text)
    while i < n and text[i] != '"':
        if text[i] == "\\":
            i += 1
        i += 1
    return i + 1


def _scalar_value_end(text: str, i: int) -> int | None:
    """End index of a simple field value at `text[i:]`.

    Handles exactly the value kinds the spawn overrides need: a quoted string,
    or up to four numeric / TRUE / FALSE tokens (SFVec3f, SFRotation, SFBool,
    SFFloat). Returns None for anything else (a node, an MF list) so callers
    fall back instead of corrupting the text.
    """
    n = len(text)
    while i < n and text[i] in " \t":
        i += 1
    if i >= n:
        return None
    if text[i] == '"':
        return _skip_quoted(text, i)
    if text[i] in "{[":
        return None
    end = i
    tokens = 0
    while end < n and tokens < 4:
        while end < n and text[end] in " \t":
            end += 1
        start = end
        while end < n and text[end] not in " \t\r\n":
            end += 1
        token = text[start:end]
        if not token:
            break
        if token not in ("TRUE", "FALSE"):
            try:
                float(token)
            except ValueError:
                return start if tokens else None
        tokens += 1
        # Stop at a line break: exportString emits one field per line.
        probe = end
        while probe < n and text[probe] in " \t":
            probe += 1
        if probe < n and text[probe] in "\r\n":
            break
    return end


def replace_top_level_field(vrml: str, field: str, value_text: str) -> tuple[str, bool]:
    """Rewrite `field <value>` at the TOP level of a node string.

    Used by the clone path, and it has to happen before the import rather than
    as a field write afterwards: the engine starts the imported Robot's
    controller immediately, and the controller's IPC channel is keyed by the
    robot's NAME — so two clones that arrive carrying the source's name collide
    ("refusing connection attempt from another extern controller", the second
    controller exits 1 and that robot never moves). Measured: 8 of 9 clones
    silently dead until the name was rewritten here.

    Depth-aware because a robot subtree is full of nested `name` fields; only
    the node's own field is touched. Returns (text, replaced?).
    """
    try:
        open_brace = vrml.index("{")
    except ValueError:
        return vrml, False
    depth = 1
    i = open_brace + 1
    n = len(vrml)
    while i < n:
        ch = vrml[i]
        if ch == '"':
            i = _skip_quoted(vrml, i)
            continue
        if ch in "{[":
            depth += 1
            i += 1
            continue
        if ch in "}]":
            depth -= 1
            if depth == 0:
                break
            i += 1
            continue
        if depth == 1 and (ch.isalpha() or ch == "_"):
            k = i
            while k < n and (vrml[k].isalnum() or vrml[k] == "_"):
                k += 1
            if vrml[i:k] == field:
                end = _scalar_value_end(vrml, k)
                if end is None:
                    return vrml, False
                return vrml[:i] + f"{field} {value_text}" + vrml[end:], True
            i = k
            continue
        i += 1
    return vrml, False


def delete_verification(removed: list, missing: list, still: list) -> dict:
    """Did scene_delete do what was asked? `all_removed` must mean exactly that.

    It used to be `not still`, and `still` is derived from `removed` -- so a
    request naming only DEFs that DO NOT EXIST removed nothing, had nothing to
    re-resolve, and came back `all_removed: true`. Measured: POST /scene/delete
    {"def":"TYPO"} -> 200, removed [], missing ["TYPO"], all_removed true. An
    agent branching on that flag got a confirmation for a typo.

    True now requires BOTH directions: something was removed, and nothing was
    left behind or unfound. `all_removed_reason` names which one failed so the
    caller does not have to re-derive it. Pure function.
    """
    out = {
        "all_removed": bool(removed) and not still and not missing,
        "still_resolves": still,
        "removed_count": len(removed),
        "missing_count": len(missing),
    }
    if missing:
        out["all_removed_reason"] = (
            f"{len(missing)} requested DEF(s) did not exist in the scene: "
            + ", ".join(str(m) for m in missing[:8]))
    elif still:
        out["all_removed_reason"] = (
            "removed, but still resolving after the settle: "
            + ", ".join(str(s) for s in still[:8]))
    elif not removed:
        out["all_removed_reason"] = "nothing was removed"
    return out


def rearm_after_reset(trackers, reason: str, log=None) -> dict:
    """Clear damage state and rearm the inject schedule after a sim reset.

    Called from BOTH reset paths, which is the whole point. The main loop used
    to detect a reset only by watching the engine clock jump BACKWARDS relative
    to its own counter -- and that stopped firing for the `reset` command once
    the command started reporting `advanced_to_ms` (the loop then pulls the
    rewound time in itself, so there is no backwards jump left to see). The
    visible symptom: a damage world reset over HTTP came back with the robot
    still destroyed, prior HP and debris intact, and scheduled injects never
    replaying. The backwards-jump detector is still there for resets triggered
    from outside the RPC (a controller or the GUI calling simulationReset).

    `log` is an optional line writer (the callers pass sys.stderr.write) so the
    function stays pure enough to unit-test. Returns what was actually done --
    never a claim, always the outcome.
    """
    cleared: list = []
    errors: list = []
    for tracker in trackers:
        if tracker is None:
            continue
        name = getattr(tracker, "robot_name", None) or repr(tracker)
        try:
            tracker.reset()
            cleared.append(name)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"{name}: {exc}")
    if log is not None:
        log(f"[harness_supervisor] {reason}: rearming inject schedule and "
            f"clearing damage state on {len(cleared)} tracker(s)"
            + (f"; {len(errors)} failed: {'; '.join(errors)}" if errors else "")
            + "\n")
    return {
        "inject_schedule_rearmed": True,
        "damage_trackers_reset": cleared,
        "errors": errors,
        "reason": reason,
    }


def _pose_of(node) -> list[float] | None:
    try:
        return [float(v) for v in node.getPosition()]
    except Exception:  # noqa: BLE001
        return None


def pose_fingerprint(supervisor: Supervisor) -> dict:
    """World positions of the scene's TOP-LEVEL posed nodes, keyed by
    `DEF` (or `#id` when DEF-less).

    A verification sample, not the state: the engine's saved state is the
    whole scene recursively, but reading every node's pose costs one IPC
    round-trip each (tens of seconds on a 298-node world). Root children are
    the bodies whose pose answers "did the restore land / did the ball stay
    where it fell", at ~10-20 round-trips.
    """
    root = supervisor.getRoot()
    if root is None:
        return {}
    children = root.getField("children")
    if children is None:
        return {}
    out: dict = {}
    try:
        count = children.getCount()
    except Exception:  # noqa: BLE001
        return {}
    for i in range(count):
        try:
            node = children.getMFNode(i)
        except Exception:  # noqa: BLE001
            continue
        if node is None:
            continue
        # Gate on the presence of a `translation` field instead of just
        # try/except-ing getPosition(): the engine WARNS on
        # wb_supervisor_node_get_position() for a non-Pose node (Viewpoint,
        # WorldInfo, Background, DirectionalLight...), and those warnings land
        # in the harness's own load diagnostics. Probing a field name is
        # silent — node_detail() already relies on that.
        try:
            if node.getField("translation") is None:
                continue
        except Exception:  # noqa: BLE001
            continue
        pos = _pose_of(node)
        if pos is None:
            continue
        try:
            key = node.getDef() or f"#{node.getId()}"
        except Exception:  # noqa: BLE001
            key = f"#{i}"
        out[key] = pos
    return out


def compare_fingerprints(before: dict, after: dict) -> dict:
    """Max/mean pose delta between two `pose_fingerprint()` samples."""
    common = [k for k in before if k in after]
    deltas: list[tuple[str, float]] = []
    for key in common:
        a, b = before[key], after[key]
        deltas.append((key, math.dist(a, b)))
    missing = [k for k in before if k not in after]
    added = [k for k in after if k not in before]
    if not deltas:
        return {"sampled_nodes": 0, "max_pose_delta_m": None,
                "exact": None, "missing": missing, "added": added}
    worst_key, worst = max(deltas, key=lambda kv: kv[1])
    return {
        "sampled_nodes": len(deltas),
        "max_pose_delta_m": round(worst, 6),
        "max_pose_delta_node": worst_key,
        "mean_pose_delta_m": round(sum(d for _, d in deltas) / len(deltas), 6),
        "exact": worst < 1e-6,
        "missing": missing,
        "added": added,
    }


def _advance(supervisor: Supervisor, basic_step_ms: int, sim_time_ms: float,
             steps: int) -> float:
    """Step `steps` basic steps, returning the new sim time. Used by the
    mutation verbs so a queued field write actually lands before read-back.
    """
    t = float(sim_time_ms)
    for _ in range(max(0, int(steps))):
        if supervisor.step(basic_step_ms) == -1:
            raise CommandError("simulator step returned -1 (terminating)")
        t += basic_step_ms
    return t


def dispatch_commands() -> list[str]:
    """Every command name `dispatch()` answers, scanned from its own source.

    Published by `capabilities` so the harness (and an agent behind it) can
    enumerate the RPC surface without a hand-maintained list going stale.
    """
    try:
        src = inspect.getsource(dispatch)
    except (OSError, TypeError):
        return []
    return sorted(set(re.findall(r'cmd == "(\w+)"', src)))


def dispatch(supervisor: Supervisor, basic_step_ms: int, sim_time_ms: float,
             cmd: str, args: dict, damage: DamageTracker | None = None,
             bus: EventBus | None = None,
             contact_tracker: ContactTracker | None = None,
             grip_tracker: GripTracker | None = None,
             joint_velocity_cache: dict | None = None):
    if cmd == "ping":
        return {}
    if cmd == "sim_state":
        return {
            "sim_time_ms": sim_time_ms,
            "basic_time_step_ms": basic_step_ms,
        }
    if cmd == "damage_state":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        return damage.state_snapshot()
    if cmd == "damage_events":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        since = int(args.get("since", 0))
        limit = int(args.get("limit", 256))
        if limit < 1:
            limit = 1
        if limit > 1024:
            limit = 1024
        evts = damage.events_since(since, limit)
        last_id = evts[-1]["step_id"] if evts else since
        return {"events": evts, "last_step_id": last_id,
                "events_total": damage.event_counter}
    if cmd == "damage_reset":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        damage.reset()
        return {"ok": True}
    if cmd == "damage_geometry_stats":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        return damage.geometry_stats()
    if cmd == "damage_set_heal_rate":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        rate_hp = args.get("rate_hp")
        rate_mesh = args.get("rate_mesh")
        parts = args.get("parts")
        if rate_hp is not None:
            try:
                rate_hp = float(rate_hp)
            except (TypeError, ValueError):
                raise CommandError("'rate_hp' must be a number")
        if rate_mesh is not None:
            try:
                rate_mesh = float(rate_mesh)
            except (TypeError, ValueError):
                raise CommandError("'rate_mesh' must be a number")
        if parts is not None and not isinstance(parts, list):
            raise CommandError("'parts' must be a list of strings")
        return damage.set_heal_rate(rate_hp=rate_hp, rate_mesh=rate_mesh,
                                     parts=parts)
    if cmd == "damage_heal_to_pristine":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        return damage.heal_to_pristine(sim_time_ms=int(sim_time_ms))
    if cmd == "damage_inject":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        part = args.get("part")
        if not isinstance(part, str) or not part:
            raise CommandError("damage_inject requires 'part'")
        result = damage.inject(
            part,
            hp_delta=args.get("hp_delta"),
            state=args.get("state"),
            sim_time_ms=int(sim_time_ms),
        )
        if "error" in result:
            raise CommandError(result["error"])
        return result
    if cmd == "step":
        steps = int(args.get("steps", 1))
        if steps < 1:
            steps = 1
        # Tick damage detection once per inner step so headless harness
        # callers (--mode=fast scripted runs) see contact-driven dent
        # accumulation. Without this, damage.poll() only fires on the
        # outer main-loop iterations between commands — which is roughly
        # one tick per RPC, not one per sim step.
        local_sim_ms = float(sim_time_ms)
        for _ in range(steps):
            if supervisor.step(basic_step_ms) == -1:
                raise CommandError("simulator step returned -1 (terminating)")
            local_sim_ms += basic_step_ms
            if damage is not None:
                damage.poll(int(local_sim_ms))
            if contact_tracker is not None:
                try:
                    contact_tracker.poll(local_sim_ms)
                except Exception:
                    pass
            if grip_tracker is not None and contact_tracker is not None:
                try:
                    grip_tracker.poll(
                        contact_tracker.current_pairs(),
                        observe.build_robot_subtree_index(supervisor),
                        local_sim_ms,
                    )
                except Exception:
                    pass
        return {"sim_time_ms": local_sim_ms, "advanced_to_ms": local_sim_ms}
    if cmd == "reset":
        # simulationReset() rewinds the clock. On its own it measurably does
        # NOT restore node poses (docs/developer/agent-native-api.md G2,
        # re-verified on both backends), so unless the caller opts out we also
        # load the authored snapshot the supervisor saved before its first
        # step — which is what "reset" is supposed to mean.
        restore = args.get("restore", ENGINE_INIT_STATE)
        if restore is not None and not isinstance(restore, str):
            raise CommandError("'restore' must be a snapshot name or null")
        before = pose_fingerprint(supervisor) if args.get("verify", True) else {}
        supervisor.simulationReset()
        restored = None
        restore_error = None
        if restore:
            snap = _SNAPSHOTS.get(restore)
            if snap is None:
                restore_error = (
                    f"no snapshot named {restore!r} in this world "
                    f"(have: {sorted(_SNAPSHOTS)}); the clock was rewound but "
                    "node state was NOT restored")
            else:
                root = supervisor.getRoot()
                if root is None:
                    restore_error = "scene root unavailable; state not restored"
                else:
                    root.loadState(restore)
                    restored = restore
        settle = int(args.get("settle_steps", 1 if args.get("verify", True) else 0))
        sim_after = _advance(supervisor, basic_step_ms, 0.0, settle)
        out: dict = {
            "sim_time_ms": sim_after,
            "advanced_to_ms": sim_after,
            "restored": restored,
            "settle_steps": settle,
            # The main loop owns the damage trackers and the inject cursor, so
            # the side effects of a reset are REQUESTED here and performed
            # there. This marker is what replaced the backwards-clock
            # heuristic: reporting `advanced_to_ms` (above) makes the loop pull
            # the rewound time in itself, which means there is no longer any
            # backwards jump for the heuristic to see. The loop pops this key
            # and swaps in the outcome under "reset_side_effects".
            "_rearm_after_reset": True,
        }
        if restore_error:
            out["warning"] = restore_error
        if args.get("verify", True):
            after = pose_fingerprint(supervisor)
            target = (_SNAPSHOTS.get(restored) or {}).get("poses") if restored else None
            verification = {
                "moved_by_reset": compare_fingerprints(before, after),
                "poses_after": after,
            }
            if target is not None:
                verification["vs_snapshot"] = compare_fingerprints(target, after)
            else:
                verification["vs_snapshot"] = None
                verification["vs_snapshot_note"] = (
                    f"'{restored}' is the engine's own parse-time state, so the "
                    "supervisor has no sampled poses to diff against; compare "
                    "poses_after with the .wbt instead")
            out["verification"] = verification
        return out
    if cmd == "sim_snapshot":
        name = args.get("name") or "default"
        if not isinstance(name, str):
            raise CommandError("'name' must be a string")
        if name.startswith("__"):
            raise CommandError(
                "snapshot names starting with '__' are reserved for "
                f"engine-provided states (currently {ENGINE_INIT_STATE!r})")
        root = supervisor.getRoot()
        if root is None:
            raise CommandError("scene root is unavailable")
        root.saveState(name)
        poses = pose_fingerprint(supervisor)
        _SNAPSHOTS[name] = {
            "name": name,
            "sim_time_ms": float(sim_time_ms),
            "poses": poses,
            "created_wall": time.time(),
        }
        return {
            "name": name,
            "sim_time_ms": float(sim_time_ms),
            "sampled_nodes": len(poses),
            "names": sorted(_SNAPSHOTS),
            "scope": "world (OmNode::save recurses the whole scene)",
            "sample_scope": "root children (used only to verify a later restore)",
        }
    if cmd == "sim_restore":
        name = args.get("name") or "default"
        snap = _SNAPSHOTS.get(name)
        if snap is None:
            raise CommandError(
                f"no snapshot named {name!r} in this world (have: "
                f"{sorted(_SNAPSHOTS)}). Snapshot names live in the supervisor "
                "process, so a world load clears them; POST /sim/snapshot "
                "first. Restoring an unsaved name would teleport the scene to "
                "the origin, so it is refused.")
        root = supervisor.getRoot()
        if root is None:
            raise CommandError("scene root is unavailable")
        before = pose_fingerprint(supervisor)
        root.loadState(name)
        settle = int(args.get("settle_steps", 1))
        sim_after = _advance(supervisor, basic_step_ms, sim_time_ms, settle)
        after = pose_fingerprint(supervisor)
        target = snap.get("poses")
        return {
            "name": name,
            "sim_time_ms": sim_after,
            "advanced_to_ms": sim_after,
            "snapshot_sim_time_ms": snap.get("sim_time_ms"),
            "engine_provided": bool(snap.get("engine_provided")),
            "settle_steps": settle,
            # The clock is NOT rewound: a restore puts the bodies back, it does
            # not pretend the run did not happen. Use `reset` for t=0.
            "clock_rewound": False,
            "verification": {
                "vs_snapshot": (compare_fingerprints(target, after)
                                if target is not None else None),
                "moved_by_restore": compare_fingerprints(before, after),
                "poses_after": after,
            },
        }
    if cmd == "sim_snapshots":
        return {"snapshots": [
            {"name": s["name"], "sim_time_ms": s["sim_time_ms"],
             "sampled_nodes": (None if s.get("poses") is None
                               else len(s["poses"])),
             "engine_provided": bool(s.get("engine_provided")),
             "note": s.get("note"),
             "age_s": round(time.time() - s["created_wall"], 3)}
            for s in sorted(_SNAPSHOTS.values(), key=lambda s: s["created_wall"])
        ]}
    if cmd == "scene_spawn":
        vrml = args.get("vrml")
        clone_of = args.get("clone")
        if clone_of is not None:
            # Clone an existing node: ask the ENGINE for its VRML
            # (Node.exportString) and re-import that. This is the only way to
            # spawn a URDF-derived robot over the wire: `URDFRobot { url ... }`
            # is a tokenizer-level SOURCE expansion applied in
            # OmTokenizer::tokenizeFile (src/omnisim/vrml/OmTokenizer.cpp:412),
            # and the supervisor import path goes through tokenizeString, which
            # never sees it — so the parser treats `URDFRobot` as an undeclared
            # PROTO and refuses. exportString hands back the *already expanded*
            # Robot node, straight from the engine, so there is no second URDF
            # importer to drift from.
            if not isinstance(clone_of, str) or not clone_of:
                raise CommandError("'clone' must be a DEF string")
            source = find_node_by_def(supervisor, clone_of)
            if source is None:
                raise CommandError(f"no node with DEF {clone_of!r} to clone")
            try:
                vrml = source.exportString()
            except Exception as exc:  # noqa: BLE001
                raise CommandError(f"exportString on DEF {clone_of!r} failed: {exc}")
            if not isinstance(vrml, str) or not vrml.strip():
                raise CommandError(
                    f"DEF {clone_of!r} exported an empty node string")
            # exportString emits `DEF <source> Type { ... }` when the source
            # carries a DEF; strip it so the caller's `def` is authoritative
            # and two clones never collide on one DEF.
            existing_def = def_in_vrml(vrml)
            if existing_def:
                vrml = vrml[vrml.index(existing_def) + len(existing_def):].lstrip()
            new_def = args.get("def")
            if new_def:
                vrml = f"DEF {new_def} {vrml}"
            # Rewrite name / pose in the TEXT, before the import: the name has
            # to be right at import time (the controller starts then and its
            # IPC channel is keyed by it), and putting the pose in the text
            # means the clone is created where it belongs instead of appearing
            # at the source's pose for one step.
            in_vrml: list[str] = []
            for fname, value in (("name", args.get("name")),
                                 ("translation", args.get("translation")),
                                 ("rotation", args.get("rotation"))):
                if value is None:
                    continue
                if fname == "name":
                    text = '"' + str(value).replace("\\", "\\\\").replace('"', '\\"') + '"'
                else:
                    text = " ".join(repr(round(float(v), 9)) for v in value)
                vrml, done = replace_top_level_field(vrml, fname, text)
                if done:
                    in_vrml.append(fname)
            args = dict(args)
            args["_overrides_in_vrml"] = in_vrml
        if not isinstance(vrml, str) or not vrml.strip():
            raise CommandError("scene_spawn requires a 'vrml' node string or 'clone'")
        parent_def = args.get("parent")
        parent, field = _spawn_parent(supervisor, parent_def)
        try:
            before_count = field.getCount()
        except Exception as exc:  # noqa: BLE001
            raise CommandError(f"cannot read the parent's children count: {exc}")
        index = args.get("index")
        at = before_count if index is None else int(index)
        if at < 0:
            at = max(0, before_count + 1 + at)
        def_name = args.get("def") or def_in_vrml(vrml)
        # A DEF must be free BEFORE the import, because nothing downstream can
        # recover from a collision: the engine does not rename a duplicate DEF
        # on import, and `find_node_by_def` is `getFromDef`, which answers with
        # the FIRST dictionary match. So a spawn onto a taken DEF imports the
        # new node and then reports the PRE-EXISTING one's def/id/type/position
        # -- with `verification.def_resolves` reading true and `pose_delta_m`
        # computed against the wrong body. Refusing is the only honest answer
        # available at this layer.
        if def_name:
            taken = find_node_by_def(supervisor, def_name)
            if taken is not None:
                existing = node_summary(taken)
                raise CommandError(
                    f"DEF {def_name!r} is already taken by an existing "
                    f"{existing.get('type') or 'node'} (id "
                    f"{existing.get('id')}, position {existing.get('position')}); "
                    "spawning onto it would report that node back to you instead "
                    "of the new one. Choose a free 'def', or delete the existing "
                    "node first (POST /scene/delete).")
        field.importMFNodeFromString(at, vrml)
        # The cached Solid list the per-step contact tracker walks is now stale.
        observe.invalidate_scene_cache()
        try:
            after_count = field.getCount()
        except Exception as exc:  # noqa: BLE001
            raise CommandError(f"cannot re-read the parent's children count: {exc}")
        if after_count <= before_count:
            # The engine rejected the node string. It logs the parse error, so
            # the actionable detail is in the event stream, not here.
            raise CommandError(
                "import added no node: the engine rejected the VRML "
                f"(children {before_count} -> {after_count}). The parse error "
                "is in the engine log — GET /sim/events?types=world.error")
        node = find_node_by_def(supervisor, def_name) if def_name else None
        if node is None:
            try:
                node = field.getMFNode(min(at, after_count - 1))
            except Exception:  # noqa: BLE001
                node = None
        # A clone arrives carrying the SOURCE's pose and name (exportString
        # exports what the source has), so those are overridden here by field
        # write. For the composed / raw-VRML forms the pose is already spliced
        # into the node text and this is a no-op.
        applied: dict = {}
        in_vrml = args.get("_overrides_in_vrml") or []
        if node is not None:
            for fname, setter in (("translation", "setSFVec3f"),
                                  ("rotation", "setSFRotation"),
                                  ("name", "setSFString")):
                value = args.get(fname)
                if value is None or fname in in_vrml:
                    continue
                if clone_of is None and fname != "name":
                    continue
                f = node.getField(fname)
                if f is None:
                    continue
                try:
                    getattr(f, setter)(
                        [float(v) for v in value] if fname != "name" else str(value))
                    applied[fname] = value
                except Exception as exc:  # noqa: BLE001
                    applied[fname] = f"failed: {exc}"
            if applied and args.get("reset_physics", True):
                try:
                    node.resetPhysics()
                except Exception:  # noqa: BLE001
                    pass
        # Settle AFTER the overrides: a field write is applied by the engine on
        # its next step, so a clone read back with settle_steps=0 still reports
        # the source's pose.
        settle = int(args.get("settle_steps", 1 if applied else 0))
        sim_after = _advance(supervisor, basic_step_ms, sim_time_ms, settle)
        summary = node_summary(node) if node is not None else {}
        verification = {
            "node_resolved": node is not None,
            "def_resolves": bool(def_name) and find_node_by_def(
                supervisor, def_name) is not None,
            "children_delta": after_count - before_count,
        }
        want_pos = args.get("translation")
        if want_pos and summary.get("position"):
            verification["pose_delta_m"] = round(
                math.dist(summary["position"], [float(v) for v in want_pos]), 6)
        return {
            "def": summary.get("def") or def_name,
            "id": summary.get("id"),
            "type": summary.get("type"),
            "position": summary.get("position"),
            "orientation": summary.get("orientation"),
            "index": at,
            "parent": parent_def or "root",
            "cloned_from": clone_of,
            "overrides_in_vrml": in_vrml,
            "overrides_by_field_write": applied,
            "children_before": before_count,
            "children_after": after_count,
            "settle_steps": settle,
            "sim_time_ms": sim_after,
            "advanced_to_ms": sim_after,
            "verification": verification,
        }
    if cmd == "scene_delete":
        defs = args.get("defs")
        if defs is None:
            one = args.get("def")
            defs = [one] if isinstance(one, str) and one else []
        if not isinstance(defs, list) or not defs:
            raise CommandError("scene_delete requires 'def' or a non-empty 'defs' list")
        removed: list[dict] = []
        missing: list[str] = []
        for def_name in defs:
            if not isinstance(def_name, str) or not def_name:
                continue
            node = find_node_by_def(supervisor, def_name)
            if node is None:
                missing.append(def_name)
                continue
            summary = node_summary(node)
            node.remove()
            # Same reason as the spawn path: the cached Solid list now holds a
            # node that no longer exists.
            observe.invalidate_scene_cache()
            removed.append({"def": def_name, "id": summary.get("id"),
                            "type": summary.get("type")})
        settle = int(args.get("settle_steps", 0))
        sim_after = _advance(supervisor, basic_step_ms, sim_time_ms, settle)
        still = [r["def"] for r in removed
                 if find_node_by_def(supervisor, r["def"]) is not None]
        return {
            "removed": removed,
            "missing": missing,
            "settle_steps": settle,
            "sim_time_ms": sim_after,
            "advanced_to_ms": sim_after,
            "verification": delete_verification(removed, missing, still),
        }
    if cmd == "scene_set_pose":
        def_name = args.get("def")
        if not isinstance(def_name, str) or not def_name:
            raise CommandError("scene_set_pose requires a 'def' string")
        node = find_node_by_def(supervisor, def_name)
        if node is None:
            raise CommandError(f"no node with DEF {def_name!r}")
        translation = args.get("translation")
        rotation = args.get("rotation")
        if translation is None and rotation is None:
            raise CommandError("scene_set_pose requires 'translation' and/or 'rotation'")
        if translation is not None and (not isinstance(translation, list)
                                        or len(translation) != 3):
            raise CommandError("'translation' must be a list of 3 numbers")
        if rotation is not None and (not isinstance(rotation, list)
                                     or len(rotation) != 4):
            raise CommandError("'rotation' must be [ax, ay, az, angle]")
        before = _pose_of(node)
        if translation is not None:
            field = node.getField("translation")
            if field is None:
                raise CommandError(
                    f"DEF {def_name!r} ({node.getTypeName()}) has no "
                    "'translation' field")
            field.setSFVec3f([float(v) for v in translation])
        if rotation is not None:
            field = node.getField("rotation")
            if field is None:
                raise CommandError(
                    f"DEF {def_name!r} ({node.getTypeName()}) has no "
                    "'rotation' field")
            field.setSFRotation([float(v) for v in rotation])
        # A teleported body keeps its velocity otherwise, which reads as the
        # pose "not sticking" on the next step.
        reset_physics = bool(args.get("reset_physics", True))
        if reset_physics:
            try:
                node.resetPhysics()
            except Exception as exc:  # noqa: BLE001
                raise CommandError(f"resetPhysics failed: {exc}")
        settle = int(args.get("settle_steps", 1))
        sim_after = _advance(supervisor, basic_step_ms, sim_time_ms, settle)
        after = _pose_of(node)
        verification: dict = {"settled_steps": settle,
                              "reset_physics": reset_physics}
        if translation is not None and after is not None:
            # World position vs the requested LOCAL translation: equal only
            # when the parent frame is the world (true for root children).
            verification["pose_delta_m"] = round(
                math.dist(after, [float(v) for v in translation]), 6)
            verification["frame"] = (
                "world position vs requested local translation; these differ "
                "when the node is not a root child")
        return {
            "def": def_name,
            "type": node.getTypeName(),
            "requested": {"translation": translation, "rotation": rotation},
            "position_before": before,
            "position": after,
            "sim_time_ms": sim_after,
            "advanced_to_ms": sim_after,
            "verification": verification,
        }
    if cmd == "scene_set_poses":
        requested = args.get("changes")
        if not isinstance(requested, list) or not requested:
            raise CommandError("scene_set_poses requires a non-empty 'changes' list")
        prepared: list[dict] = []
        seen: set[str] = set()
        # Validate and capture every authored/local pose before the first write.
        # That makes caller mistakes all-or-nothing and gives the error path an
        # exact rollback value rather than a world-position approximation.
        for change in requested:
            if not isinstance(change, dict):
                raise CommandError("every scene_set_poses change must be an object")
            def_name = change.get("def")
            if not isinstance(def_name, str) or not def_name:
                raise CommandError("every scene_set_poses change requires a 'def' string")
            if def_name in seen:
                raise CommandError(f"duplicate DEF {def_name!r} in scene_set_poses")
            seen.add(def_name)
            node = find_node_by_def(supervisor, def_name)
            if node is None:
                raise CommandError(f"no node with DEF {def_name!r}")
            translation = change.get("translation")
            rotation = change.get("rotation")
            if translation is None and rotation is None:
                raise CommandError(
                    f"scene_set_poses change for {def_name!r} requires "
                    "'translation' and/or 'rotation'")
            if translation is not None and (not isinstance(translation, list)
                                             or len(translation) != 3):
                raise CommandError(
                    f"translation for {def_name!r} must be a list of 3 numbers")
            if rotation is not None and (not isinstance(rotation, list)
                                          or len(rotation) != 4):
                raise CommandError(
                    f"rotation for {def_name!r} must be [ax, ay, az, angle]")
            item = {"def": def_name, "node": node,
                    "position_before": _pose_of(node), "fields": {}}
            for field_name, value, getter in (
                    ("translation", translation, "getSFVec3f"),
                    ("rotation", rotation, "getSFRotation")):
                if value is None:
                    continue
                field = node.getField(field_name)
                if field is None:
                    raise CommandError(
                        f"DEF {def_name!r} ({node.getTypeName()}) has no "
                        f"{field_name!r} field")
                try:
                    numeric = [float(v) for v in value]
                    if any(not math.isfinite(v) for v in numeric):
                        raise ValueError("non-finite value")
                    before = [float(v) for v in getattr(field, getter)()]
                except Exception as exc:  # noqa: BLE001
                    raise CommandError(
                        f"invalid {field_name} for {def_name!r}: {exc}")
                item["fields"][field_name] = {
                    "field": field, "before": before, "requested": numeric}
            prepared.append(item)

        reset_physics = bool(args.get("reset_physics", True))
        try:
            settle = int(args.get("settle_steps", 1))
        except (TypeError, ValueError):
            raise CommandError("settle_steps must be an integer")
        if settle < 0:
            raise CommandError("settle_steps must be >= 0")
        applied: list[dict] = []
        try:
            for item in prepared:
                # Include the current item before its first write so a failure
                # in a later field or resetPhysics still rolls that write back.
                applied.append(item)
                for field_name, spec in item["fields"].items():
                    if field_name == "translation":
                        spec["field"].setSFVec3f(spec["requested"])
                    else:
                        spec["field"].setSFRotation(spec["requested"])
                if reset_physics:
                    item["node"].resetPhysics()
        except Exception as exc:  # noqa: BLE001
            rollback_errors: list[str] = []
            for item in reversed(applied):
                try:
                    for field_name, spec in item["fields"].items():
                        if field_name == "translation":
                            spec["field"].setSFVec3f(spec["before"])
                        else:
                            spec["field"].setSFRotation(spec["before"])
                    if reset_physics:
                        item["node"].resetPhysics()
                except Exception as rollback_exc:  # noqa: BLE001
                    rollback_errors.append(f"{item['def']}: {rollback_exc}")
            suffix = (f"; rollback errors: {'; '.join(rollback_errors)}"
                      if rollback_errors else "; prior changes rolled back")
            raise CommandError(f"scene_set_poses failed: {exc}{suffix}")

        sim_after = _advance(supervisor, basic_step_ms, sim_time_ms, settle)
        results: list[dict] = []
        for item in prepared:
            results.append({
                "def": item["def"],
                "type": item["node"].getTypeName(),
                "requested": {name: spec["requested"]
                              for name, spec in item["fields"].items()},
                "runtime_local_before": {name: spec["before"]
                                         for name, spec in item["fields"].items()},
                "position_before": item["position_before"],
                "position": _pose_of(item["node"]),
            })
        return {
            "changes": results,
            "sim_time_ms": sim_after,
            "advanced_to_ms": sim_after,
            "verification": {
                "validated_before_mutation": True,
                "applied": len(results),
                "reset_physics": reset_physics,
                "settled_steps": settle,
                "single_settle_for_batch": True,
            },
        }
    if cmd == "capabilities":
        try:
            sources = [inspect.getsource(event_bus), inspect.getsource(sys.modules[__name__])]
        except (OSError, TypeError, KeyError):
            sources = []
        events = event_bus.verify_event_types(*sources)
        suppressed = [t for t, p in event_bus.EVENT_TYPE_PRODUCERS.items()
                      if LIGHT_MODE and p in event_bus.LIGHT_MODE_DISABLED_PRODUCERS]
        events["active"] = [t for t in events["types"] if t not in suppressed]
        events["suppressed"] = sorted(suppressed)
        if suppressed:
            events["suppressed_reason"] = (
                "the supervisor is running with --light, which skips the "
                "contact / joint-limit / grip trackers; /sim/events?types= is "
                "an exact-match allowlist, so filtering on a suppressed type "
                "returns an empty stream, not an error")
        events["producers"] = dict(event_bus.EVENT_TYPE_PRODUCERS)
        return {
            "light": LIGHT_MODE,
            "basic_time_step_ms": basic_step_ms,
            "sim_time_ms": float(sim_time_ms),
            "commands": dispatch_commands(),
            "commands_source": "scanned from dispatch() in harness_supervisor.py",
            "event_types": events,
            "snapshots": sorted(_SNAPSHOTS),
            "damage_robot": DAMAGE_ROBOT_NAME,
            "damage_attached": bool(damage.attached) if damage is not None
                               and hasattr(damage, "attached") else None,
        }
    if cmd == "world_load":
        path = args.get("path")
        if not isinstance(path, str) or not path:
            raise CommandError("world_load requires a 'path' string")
        # Webots's worldLoad() resolves relative paths against the
        # controller's CWD — which for our supervisor is its own
        # controller folder, NOT the repo root. That gave us a
        # `controllers/harness_supervisor/projects/samples/.../foo.wbt`
        # nonsense path that failed to open, leaving the simulator in
        # a half-reverted broken state. Accept relative paths from
        # callers but resolve them against OMNISIM_HOME or the current
        # working directory before forwarding.
        if not os.path.isabs(path):
            base = os.environ.get("OMNISIM_HOME") or os.getcwd()
            path = os.path.normpath(os.path.join(base, path))
        if not os.path.exists(path):
            raise CommandError(f"world file not found: {path!r}")
        # The reply must be sent BEFORE the simulator actually swaps worlds —
        # worldLoad terminates this controller process, dropping the socket.
        # We return early; the caller must handle the disconnect and reconnect
        # to the new supervisor instance after the world swap completes.
        supervisor.worldLoad(path)
        return {"path": path}
    if cmd == "screenshot":
        path = args.get("path")
        if not isinstance(path, str) or not path:
            raise CommandError("screenshot requires a 'path' string")
        quality = int(args.get("quality", 90))
        if not (1 <= quality <= 100):
            quality = 90
        ok = supervisor.exportImage(path, quality)
        if ok is False:  # Webots API returns None on success, False on failure
            raise CommandError(f"exportImage failed for path '{path}'")
        return {"path": path}
    if cmd == "scene_tree":
        with observe.paused_reads(supervisor):
            root = supervisor.getRoot()
            bounds_by_id = None
            if args.get("bounds"):
                try:
                    bounds_by_id = geometry.bounds_index(root)
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(
                        f"[harness_supervisor] bounds_index failed: {exc}\n"
                        f"{traceback.format_exc()}"
                    )
                    bounds_by_id = {}
            return {"nodes": walk_scene_tree(root, bounds_by_id),
                    "bounds_included": bounds_by_id is not None}
    if cmd == "scene_node":
        def_name = args.get("def")
        if not isinstance(def_name, str) or not def_name:
            raise CommandError("scene_node requires a 'def' string")
        with observe.paused_reads(supervisor):
            node = find_node_by_def(supervisor, def_name)
            if node is None:
                raise CommandError(f"no node with DEF '{def_name}'")
            return node_detail(node, with_bounds=bool(args.get("bounds")))
    if cmd == "scene_bounds":
        # World-space geometric bounds keyed by DEF. One post-order pass over
        # the whole scene, so asking for N nodes costs the same as asking for
        # one. Nodes without a DEF are addressable by their numeric id.
        wanted = args.get("defs")
        if wanted is not None and not isinstance(wanted, list):
            raise CommandError("'defs' must be a list of DEF strings")
        if wanted is not None and not wanted:
            # An explicit empty defs list asks for nothing: answer without
            # paying the whole-scene walk (which previously ran and then
            # filtered every node back out).
            return {"bounds": {}, "count": 0, "scope": "targeted"}

        def bounds_entry(b: dict, summary: dict) -> dict:
            entry = dict(b)
            entry["type"] = summary.get("type")
            entry["position"] = summary.get("position")
            entry["orientation"] = summary.get("orientation")
            entry["id"] = summary.get("id")
            return entry

        with observe.paused_reads(supervisor):
            out: dict = {}
            if wanted:
                # Targeted path: walk ONLY the named subtrees — the caller
                # that named its DEFs (e.g. /scene/frame, /scene/visible)
                # needs a fraction of a whole-scene walk.
                for def_name in wanted:
                    if not isinstance(def_name, str) or not def_name:
                        continue
                    node = find_node_by_def(supervisor, def_name)
                    if node is None:
                        continue
                    b = geometry.bounds_for_subtree(node)
                    if b is None:
                        continue
                    out[def_name] = bounds_entry(b, node_summary(node))
                return {"bounds": out, "count": len(out), "scope": "targeted"}
            root = supervisor.getRoot()
            index = geometry.bounds_index(root)
            for summary in walk_scene_tree(root, index):
                key = summary.get("def") or f"#{summary.get('id')}"
                b = summary.get("bounds")
                if b is None:
                    continue
                out[key] = bounds_entry(b, summary)
            return {"bounds": out, "count": len(out), "scope": "scene"}
    if cmd == "get_viewpoint":
        with observe.paused_reads(supervisor):
            viewpoint = find_viewpoint(supervisor)
            if viewpoint is None:
                raise CommandError("no Viewpoint node found in the scene")
            result: dict = {}
            for fname in ("position", "orientation", "fieldOfView", "near", "far",
                          "follow", "followType", "followSmoothness",
                          "exposure", "ambientOcclusionRadius", "bloomThreshold",
                          "orthographicHeight", "projectionMode"):
                v = field_value(viewpoint.getField(fname))
                if v is not None:
                    result[fname] = v
            return result
    if cmd == "bounds_probe":
        def_name = args.get("def")
        if not isinstance(def_name, str) or not def_name:
            raise CommandError("bounds_probe requires a 'def' string")
        node = find_node_by_def(supervisor, def_name)
        if node is None:
            raise CommandError(f"no node with DEF '{def_name}'")
        viewpoint = find_viewpoint(supervisor)
        if viewpoint is None:
            raise CommandError("no Viewpoint node found in the scene")
        try:
            aspect = float(args.get("aspect", 1.0))
        except (TypeError, ValueError):
            raise CommandError("'aspect' must be a number")
        return geometry.probe_bounding_sphere(
            supervisor, viewpoint, node, aspect,
            basic_step_ms=basic_step_ms)
    if cmd == "set_viewpoint":
        position = args.get("position")
        orientation = args.get("orientation")
        if position is not None and (not isinstance(position, list) or len(position) != 3):
            raise CommandError("position must be a list of 3 numbers")
        if orientation is not None and (not isinstance(orientation, list) or len(orientation) != 4):
            raise CommandError("orientation must be a list of 4 numbers")
        viewpoint = find_viewpoint(supervisor)
        if viewpoint is None:
            raise CommandError("no Viewpoint node found in the scene")
        if position is not None:
            field = viewpoint.getField("position")
            if field is None:
                raise CommandError("Viewpoint has no 'position' field")
            field.setSFVec3f([float(x) for x in position])
        if orientation is not None:
            field = viewpoint.getField("orientation")
            if field is None:
                raise CommandError("Viewpoint has no 'orientation' field")
            field.setSFRotation([float(x) for x in orientation])
        return {
            "position": position if position is not None else None,
            "orientation": orientation if orientation is not None else None,
        }
    if cmd == "robots_list":
        # observe.list_robots (like list_joints / list_devices /
        # collect_contacts) pauses the engine for its own walk — the pause is
        # a property of the reader, not of this dispatch arm.
        return {"robots": observe.list_robots(supervisor)}
    if cmd == "robot_joints":
        def_name = args.get("def")
        if not isinstance(def_name, str) or not def_name:
            raise CommandError("robot_joints requires a 'def' string")
        cache = joint_velocity_cache if joint_velocity_cache is not None else {}
        try:
            return observe.list_joints(supervisor, def_name, cache,
                                       sim_time_ms)
        except KeyError as exc:
            raise CommandError(str(exc))
    if cmd == "robot_devices":
        def_name = args.get("def")
        if not isinstance(def_name, str) or not def_name:
            raise CommandError("robot_devices requires a 'def' string")
        try:
            return observe.list_devices(supervisor, def_name)
        except KeyError as exc:
            raise CommandError(str(exc))
    if cmd == "set_joint_positions":
        # The harness's first robot-COMMANDING verb (internal parity plan, item W2.1).
        # Node.setJointPosition() is NOT a teleport under Newton: OmJoint::
        # setPosition also re-pins the motor's PD target (OmJoint.cpp:130-149),
        # so the joint CONVERGES over ticks — hence settle-and-verify: apply,
        # advance settle_steps, then MEASURE, and report achieved/error rather
        # than echoing the argument back (tool-design-for-agents.md).
        def_name = args.get("def")
        if not isinstance(def_name, str) or not def_name:
            raise CommandError("set_joint_positions requires a 'def' string")
        joints_arg = args.get("joints")
        if not isinstance(joints_arg, dict) or not joints_arg:
            raise CommandError(
                "set_joint_positions requires a non-empty 'joints' object of "
                "{joint_name: target_rad_or_m}")
        targets: dict[str, float] = {}
        for name, value in joints_arg.items():
            if not isinstance(name, str) or not name:
                raise CommandError(
                    "set_joint_positions requires non-empty string joint names")
            try:
                v = float(value)
                if not math.isfinite(v):
                    raise ValueError("non-finite")
            except (TypeError, ValueError):
                raise CommandError(
                    f"target for joint {name!r} must be a finite number "
                    f"(got {value!r})")
            targets[name] = v
        try:
            settle = int(args.get("settle_steps", 16))
        except (TypeError, ValueError):
            raise CommandError("settle_steps must be an integer")
        if settle < 0:
            raise CommandError("settle_steps must be >= 0")
        try:
            index = observe.joint_write_index(supervisor, def_name)
        except KeyError as exc:
            raise CommandError(str(exc))
        by_name: dict[str, list[dict]] = {}
        for entry in index:
            if entry["name"]:
                by_name.setdefault(entry["name"], []).append(entry)
        # All-or-nothing on caller mistakes: refuse the whole batch BEFORE the
        # first write, naming the offenders and what IS commandable.
        unknown = sorted(n for n in targets if n not in by_name)
        if unknown:
            raise CommandError(
                f"unknown joint name(s) {unknown} on DEF {def_name!r}; "
                f"available joints: {sorted(by_name)}")
        ambiguous = sorted(n for n in targets if len(by_name[n]) > 1)
        if ambiguous:
            raise CommandError(
                f"joint name(s) {ambiguous} match multiple joints on DEF "
                f"{def_name!r}; rename the devices so each joint is uniquely "
                "addressable")
        plans: list[dict] = []
        for name, requested in targets.items():
            entry = by_name[name][0]
            # Mirror the engine's own clamp (OmJointParameters::clampPosition):
            # hard stops only, and minStop == maxStop == 0 means unconstrained.
            commanded, clamped = requested, False
            min_stop, max_stop = entry["min_stop"], entry["max_stop"]
            if entry["params"] is not None and min_stop is not None \
                    and max_stop is not None \
                    and not (min_stop == 0.0 and max_stop == 0.0):
                if requested < min_stop:
                    commanded, clamped = min_stop, True
                elif requested > max_stop:
                    commanded, clamped = max_stop, True
            # Servo-vs-velocity-wheel classification, mirroring the Newton
            # registration rule (OmBasicJoint.cpp): effective limits are the
            # motor's minPosition/maxPosition when they differ, else the
            # joint's minStop/maxStop; a motorised hinge whose effective
            # limits are EQUAL is built as a velocity wheel with ke=0 and
            # setPosition() on it is silently ignored by the physics. A
            # slider is always position-controlled. Reported per joint so a
            # bare success can never paper over the W1.4 trap.
            limit_lower = limit_upper = 0.0
            limit_source = None
            if entry["motor_min"] is not None and entry["motor_max"] is not None \
                    and entry["motor_min"] != entry["motor_max"]:
                limit_lower, limit_upper = entry["motor_min"], entry["motor_max"]
                limit_source = "motor minPosition/maxPosition"
            elif min_stop is not None and max_stop is not None \
                    and min_stop != max_stop:
                limit_lower, limit_upper = min_stop, max_stop
                limit_source = "joint minStop/maxStop"
            note = None
            if not entry["has_motor"]:
                controllable = None
                note = ("passive joint (no Motor device): the write sets the "
                        "joint coordinate directly and nothing holds it — the "
                        "physics may pull it straight back; judge from "
                        "'achieved'")
            elif entry["type"] == "SliderJoint":
                controllable = True
            elif entry["type"] == "HingeJoint":
                controllable = limit_lower != limit_upper
                if not controllable:
                    note = ("motor declares no position limits, so the physics "
                            "backend built this joint as a VELOCITY wheel with "
                            "ke=0: position targets are silently IGNORED "
                            "(OmBasicJoint.cpp). Give the motor a minPosition/"
                            "maxPosition (or the joint a minStop/maxStop) to "
                            "make it a servo, or drive it by velocity from its "
                            "own controller")
            else:
                # Hinge2Joint / BallJoint: only axis 1 is written here, and
                # motorised BallJoint actuation is broken on this backend
                # (the sensor readback can move while the body does not).
                controllable = None
                note = (f"{entry['type']}: first-axis write only; motorised "
                        "BallJoint actuation does not work on this backend "
                        "(Hinge2Joint does) — trust 'achieved', not the "
                        "command")
            if entry["params"] is None:
                note = ((note + ". " if note else "")
                        + "joint has no JointParameters node, so its position "
                          "cannot be read back: the write is applied but "
                          "'achieved' is null (unverified)")
            plans.append({
                "name": name, "entry": entry, "requested": requested,
                "commanded": commanded, "clamped": clamped,
                "position_controllable": controllable, "note": note,
                "limit_lower": limit_lower, "limit_upper": limit_upper,
                "limit_source": limit_source,
            })
        for plan in plans:
            try:
                plan["entry"]["node"].setJointPosition(
                    float(plan["commanded"]), 1)
            except Exception as exc:  # noqa: BLE001
                raise CommandError(
                    f"setJointPosition failed for joint {plan['name']!r}: {exc}")
        sim_after = _advance(supervisor, basic_step_ms, sim_time_ms, settle)
        achieved_list = observe.read_joint_positions(
            supervisor, [p["entry"] for p in plans])
        results: dict[str, dict] = {}
        max_abs_error = None
        max_abs_error_joint = None
        for plan, achieved in zip(plans, achieved_list):
            before = plan["entry"]["position"]
            error = None if achieved is None else achieved - plan["commanded"]
            moved = None
            if achieved is not None and before is not None:
                # 1e-4 rad/m: measured PD dither on a live held arm is
                # ~8e-5, which a 1e-6 threshold reported as "moved".
                moved = abs(achieved - before) > 1e-4
            if error is not None and (max_abs_error is None
                                      or abs(error) > max_abs_error):
                max_abs_error = abs(error)
                max_abs_error_joint = plan["name"]
            row = {
                "requested": plan["requested"],
                "commanded": plan["commanded"],
                "clamped": plan["clamped"],
                "position_before": before,
                "achieved": achieved,
                "error": error,
                "moved": moved,
                "position_controllable": plan["position_controllable"],
                "limits": {"lower": plan["limit_lower"],
                           "upper": plan["limit_upper"],
                           "source": plan["limit_source"]},
            }
            if plan["note"]:
                row["note"] = plan["note"]
            results[plan["name"]] = row
        return {
            "robot": def_name,
            "joints": results,
            "sim_time_ms": sim_after,
            "advanced_to_ms": sim_after,
            "verification": {
                "applied": len(results),
                "settle_steps": settle,
                "sim_time_advanced_ms": sim_after - float(sim_time_ms),
                "max_abs_error": max_abs_error,
                "max_abs_error_joint": max_abs_error_joint,
                "semantics": ("PD setpoint, not a teleport: "
                              "Node.setJointPosition re-pins the motor "
                              "target and the joint converges over the "
                              "settled steps; 'achieved'/'error' are "
                              "measured after settling, and a still-large "
                              "error usually means more settle_steps (or a "
                              "controller re-asserting its own targets)"),
            },
        }
    if cmd == "solve_ik":
        # Batched IK PREVIEW (internal parity plan, item W2.1): World.solve_ik via the
        # supervisor's own controller API (Node.solveIk). A PURE READ — the
        # solver owns its buffers and nothing in the scene moves; angles are
        # applied, if at all, by a separate set_joint_positions call. The
        # engine solves for every Hinge/Slider joint of the END EFFECTOR'S
        # robot registered with the physics backend, clamps to the authored
        # limits, and measures each target's residual (METRES) by forward
        # kinematics on exactly the angles it returns — so a caller can
        # reject a target instead of driving to it (never claim "reached").
        def_name = args.get("def")
        if not isinstance(def_name, str) or not def_name:
            raise CommandError("solve_ik requires a 'def' string (the robot)")
        effector = args.get("effector")
        if not isinstance(effector, str) or not effector:
            raise CommandError(
                "solve_ik requires an 'effector' string (DEF of the "
                "end-effector Solid whose position the targets constrain)")
        targets_arg = args.get("targets")
        if not isinstance(targets_arg, list) or not targets_arg:
            raise CommandError(
                "solve_ik requires a non-empty 'targets' list of [x, y, z] "
                "world-frame positions")
        targets: list[list[float]] = []
        for i, t in enumerate(targets_arg):
            if not isinstance(t, (list, tuple)) or len(t) != 3:
                raise CommandError(
                    f"solve_ik target {i} must be a [x, y, z] triple "
                    f"(got {t!r})")
            row = []
            for value in t:
                try:
                    v = float(value)
                    if not math.isfinite(v):
                        raise ValueError("non-finite")
                except (TypeError, ValueError):
                    raise CommandError(
                        f"solve_ik target {i} must contain finite numbers "
                        f"(got {value!r})")
                row.append(v)
            targets.append(row)
        rotations_arg = args.get("rotations")
        rotations: list[list[float]] | None = None
        if rotations_arg is not None:
            if not isinstance(rotations_arg, list) \
                    or len(rotations_arg) != len(targets):
                raise CommandError(
                    "solve_ik 'rotations' must pair 1:1 with 'targets' "
                    "([qx, qy, qz, qw] each)")
            rotations = []
            for i, q in enumerate(rotations_arg):
                if not isinstance(q, (list, tuple)) or len(q) != 4:
                    raise CommandError(
                        f"solve_ik rotation {i} must be a [qx, qy, qz, qw] "
                        f"quaternion (got {q!r})")
                try:
                    rotations.append([float(c) for c in q])
                except (TypeError, ValueError):
                    raise CommandError(
                        f"solve_ik rotation {i} must contain numbers")
        tool_arg = args.get("tool_offset")
        tool_offset: list[float] | None = None
        if tool_arg is not None:
            if not isinstance(tool_arg, (list, tuple)) or len(tool_arg) != 3:
                raise CommandError(
                    "solve_ik 'tool_offset' must be a [x, y, z] offset in "
                    "the end effector's own frame")
            try:
                tool_offset = [float(c) for c in tool_arg]
            except (TypeError, ValueError):
                raise CommandError("solve_ik 'tool_offset' must contain numbers")
        try:
            iterations = int(args.get("iterations", 64))
        except (TypeError, ValueError):
            raise CommandError("iterations must be an integer")
        if iterations <= 0:
            raise CommandError("iterations must be >= 1")
        # The robot walk FIRST: it owns the slot→name mapping AND validates
        # the robot DEF (KeyError → the same 404 shape as the read paths).
        try:
            entries = observe.joint_write_index(supervisor, def_name)
        except KeyError as exc:
            raise CommandError(str(exc))
        node = supervisor.getFromDef(effector)
        if node is None:
            raise CommandError(
                f"no node with DEF '{effector}' (the end effector)")
        t0 = time.perf_counter()
        res = node.solveIk(targets, rotations, tool_offset, iterations)
        solve_ms = (time.perf_counter() - t0) * 1000.0
        status = int(res.get("status", -9))
        if status != 0:
            reasons = {
                -1: "IK unavailable: no Newton physics backend, or the world "
                    "is not finalised yet",
                -2: f"IK unavailable: end effector '{effector}' has no Newton "
                    "physics body (does the chain carry Physics nodes, and is "
                    "the world finalised?)",
                -3: f"no IK-solvable joints on robot DEF '{def_name}': no "
                    "Hinge/Slider joint is registered with the physics "
                    "backend (Hinge2/Ball joints are multi-coordinate and "
                    "excluded by design)",
                -4: "IK solver failed inside the engine — the engine log "
                    "(omnisim_log.txt) carries the solver's own error",
                -9: "IK unavailable: the engine did not answer the solve "
                    "request",
                -10: "solve_ik was called with invalid arguments",
            }
            raise CommandError(reasons.get(
                status, f"IK unavailable: engine status {status}"))
        by_id: dict[int, dict] = {}
        for entry in entries:
            try:
                by_id[int(entry["node"].id)] = entry
            except Exception:  # noqa: BLE001
                continue
        solved_joints: list[dict] = []
        keys: list[str] = []
        unmapped: list[int] = []
        for nid in res["joint_node_ids"]:
            entry = by_id.get(int(nid))
            name = entry["name"] if entry else None
            key = name if name else f"node_{int(nid)}"
            keys.append(key)
            if not entry:
                unmapped.append(int(nid))
            solved_joints.append({
                "name": name,
                "node_id": int(nid),
                # appliable == set_joint_positions can address it by name
                "appliable": bool(name),
            })
        results = []
        for i, target in enumerate(targets):
            results.append({
                "target": target,
                "residual_m": float(res["residuals"][i]),
                "joints": {k: float(a)
                           for k, a in zip(keys, res["angles"][i])},
            })
        out = {
            "robot": def_name,
            "effector": effector,
            "solved_joints": solved_joints,
            "results": results,
            "solve_ms": round(solve_ms, 1),
            "verification": {
                "semantics": (
                    "PURE PREVIEW: nothing moved. Angles are clamped to the "
                    "authored joint limits; residual_m is measured by forward "
                    "kinematics on exactly the returned angles, in metres — "
                    "reject a target on its residual instead of driving to "
                    "it, and never report 'reached' from this call alone. "
                    "Apply via set_joint_positions (POST "
                    "/robot/<def>/joints/set) using the same joint names."),
                "warmup": (
                    "the FIRST solve per world compiles a warp kernel "
                    "(seconds — 8.3 s measured cold on a 6R arm; ~150 ms "
                    "warm); solve_ms above is the measured cost of THIS "
                    "call"),
            },
        }
        if unmapped:
            out["verification"]["unmapped_node_ids"] = unmapped
            out["verification"]["unmapped_note"] = (
                "these solved joints are not on robot DEF "
                f"'{def_name}' or carry no addressable name — their angles "
                "are keyed node_<id> and cannot be applied via "
                "set_joint_positions (is 'effector' really on this robot?)")
        return out
    if cmd == "sim_contacts":
        # ⚠ `wake` IS A DOCUMENTED NO-OP AS OF 2026-08-08, and the parameter is
        # kept ONLY so existing callers do not 400.
        #
        # What it used to do: write WorldInfo.physicsDisableTime = 0 "to clear
        # ODE's sleep timer", advance `settle_steps` basic steps, re-read, and
        # then REPLACE the response's caveat with a guarantee -- "this list is
        # complete for the step it was taken on".
        #
        # Every clause of that was wrong:
        #   - There is no ODE (src/ode deleted, bdc02139) and no body sleep.
        #   - `physicsDisableTime` is parsed into OmWorldInfo and read back by
        #     NOTHING: `physicsDisableTime()` has zero call sites in the engine.
        #     So the write was inert.
        #   - It therefore MUTATED THE WORLD (2+ steps of sim time, on a
        #     multi-robot Newton world up to ~27 s of wall clock EACH) during a
        #     call documented as a read, measured nothing, and then asserted
        #     completeness -- while `idle_bodies` still listed the body.
        #
        # Chosen fix: make it change nothing at all, rather than keep the two
        # steps and merely re-word the reply. Justification: the steps bought no
        # information (nothing consumed the field they enabled), and they were
        # the reason `sim_contacts` had to be excluded from the harness's
        # transparent-retry set -- so deleting them makes a read idempotent
        # again, which is worth more than the parameter ever was. The reply says
        # plainly that it did nothing, so a caller cannot mistake silence for
        # success.
        result = observe.collect_contacts(supervisor, light=LIGHT_MODE)
        if args.get("wake"):
            result["tracking"]["woken"] = {
                "requested": True,
                "applied": False,
                "steps_advanced": 0,
                "reason": ("no-op. `wake` cleared a body-sleep timer that does not "
                           "exist: this engine has no body sleep, and the field it "
                           "wrote (WorldInfo.physicsDisableTime) has no reader in the "
                           "engine. It advanced the simulation and measured nothing, so "
                           "it now does nothing instead. A body at rest already reports "
                           "its contacts -- native contact readback is on by default."),
                "if_your_set_is_empty": ("see tracking.empty_set_reasons: the causes are a "
                                         "physicsBackend \"ode\" pin, a backend that never "
                                         "came up, a Solid the backend never registered, or "
                                         "genuinely no contact. None of them is fixed by "
                                         "stepping."),
            }
        return result
    if cmd == "sim_grips":
        # `{"grips": []}` from a tracker that DOES NOT EXIST is
        # indistinguishable from "nothing is gripped" -- which is how --light
        # turned a missing capability into a confidently wrong answer. Mirror
        # what sim_contacts already does and say which one this is.
        if grip_tracker is None:
            return {
                "grips": [],
                "tracking": {
                    "enabled": False,
                    "reason": "--light",
                    "detail": ("the supervisor was started with --light, so no GripTracker "
                               "was constructed (it re-reads the contact set every basic "
                               "step). This empty list means NOT MEASURED, not 'nothing "
                               "is gripped'."),
                    "workaround": ("reload the world with {\"light\": false}, or infer the grip "
                                   "from sim_contacts (HTTP: GET /sim/contacts) -- that read "
                                   "walks the scene per call and is unaffected by --light."),
                },
            }
        return {
            "grips": grip_tracker.active_grips(),
            "tracking": {
                "enabled": True,
                "reason": None,
                # ⚠ THIS USED TO SAY "an empty list here does mean nothing is
                # gripped". It cannot: this tracker is a pure FUNCTION of the
                # contact set, so it inherits every reason that set can be empty
                # (a physicsBackend "ode" pin, a backend that never came up, a
                # Solid never registered) and it upgraded them into a positive
                # claim. A running tracker proves the tracker ran, nothing more.
                "detail": ("inferred per basic step from the contact set: a gripper subtree "
                           "and a non-robot solid in sustained contact. The tracker is "
                           "running, so an empty list means it saw no qualifying contact "
                           "as of the last step -- which is NOT the same as 'nothing is "
                           "gripped'. This is derived data: it inherits every reason the "
                           "contact set itself can be empty."),
                "derived_from": ("the same contact set GET /sim/contacts reports; read its "
                                 "tracking.empty_set_reasons before concluding anything from "
                                 "an empty list here"),
                "proof_of_a_grasp": ("do not use an empty list as proof of no grasp, and do not "
                                     "use contacts as proof of one: prove a grasp geometrically "
                                     "(the part is airborne and tracks the gripper) via "
                                     "GET /scene/node/<def>"),
            },
        }
    if cmd == "events_drain":
        if bus is None:
            return {"events": [], "next_seq": 0, "total": 0,
                    "dropped": 0, "buffered": 0}
        since = int(args.get("since", 0))
        limit = int(args.get("limit", 256))
        if limit < 1:
            limit = 1
        if limit > 1024:
            limit = 1024
        types = args.get("types")
        if types is not None and not isinstance(types, list):
            raise CommandError("'types' must be a list of strings")
        events = bus.since(since, limit, types=types)
        next_seq = events[-1]["seq"] if events else since
        return {
            "events": events,
            "next_seq": next_seq,
            "total": bus.total,
            "dropped": bus.dropped,
            "buffered": bus.buffered,
        }
    raise CommandError(f"unknown cmd: {cmd}")


def _load_inject_schedule(supervisor) -> list[dict]:
    """Parse a `damage_inject_schedule` array from the supervisor robot's
    own customData. Returns a sorted list of {t_ms, part, state, hp_delta}
    dicts. Bad entries are silently skipped; malformed JSON yields []
    with a stderr note. World authors use this to script broken-state
    transitions without an external wire-protocol client.
    """
    self_node = supervisor.getSelf()
    if self_node is None:
        return []
    f = self_node.getField("customData")
    if f is None:
        return []
    raw = f.getSFString() or ""
    raw = raw.strip()
    if not raw:
        return []
    try:
        cfg = json.loads(raw)
    except (TypeError, ValueError) as exc:
        sys.stderr.write(
            f"[harness_supervisor] customData JSON invalid ({exc}); "
            "ignoring inject schedule\n"
        )
        return []
    sched = cfg.get("damage_inject_schedule") if isinstance(cfg, dict) else None
    if not isinstance(sched, list):
        return []
    out: list[dict] = []
    for raw_entry in sched:
        if not isinstance(raw_entry, dict):
            continue
        try:
            t_ms = int(raw_entry["t_ms"])
            part = str(raw_entry["part"])
        except (KeyError, TypeError, ValueError):
            continue
        out.append({
            "t_ms": t_ms,
            "part": part,
            "state": raw_entry.get("state"),
            "hp_delta": raw_entry.get("hp_delta"),
        })
    out.sort(key=lambda e: e["t_ms"])
    return out


def main() -> int:
    supervisor = Supervisor()
    basic_step_ms = int(supervisor.getBasicTimeStep())

    # Bind the control socket before the first sim step so the harness can
    # connect immediately after world load completes.
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((HOST, PORT))
    except OSError as exc:
        sys.stderr.write(f"[harness_supervisor] bind {HOST}:{PORT} failed: {exc}\n")
        return 1
    server.listen(4)
    server.setblocking(False)
    sys.stderr.write(f"[harness_supervisor] listening on {HOST}:{PORT}\n")
    sys.stderr.flush()

    # Damage tracker: discovers a robot by name on startup and polls per
    # step. Idles silently if the named robot isn't in the world (most
    # harnessed worlds have no Husky and no damage to report).
    damage = DamageTracker(supervisor, robot_name=DAMAGE_ROBOT_NAME)
    # Secondary trackers for additional robots (visual symmetry only;
    # the wire protocol still operates on `damage`). Each idles silently
    # if its robot isn't in the world. With OMNISIM_HARNESS_DAMAGE_EXTRA_ROBOTS
    # set to "husky_b" by default, a husky_head_on world tracks both
    # huskies — head-on collision damages both bumpers on screen.
    extra_damages = []
    for extra_name in DAMAGE_EXTRA_ROBOTS:
        if extra_name == DAMAGE_ROBOT_NAME:
            continue  # don't double-track the same robot
        try:
            extra_damages.append(
                DamageTracker(supervisor, robot_name=extra_name))
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(
                f"[harness_supervisor] failed to create tracker for "
                f"{extra_name!r}: {exc}\n")
    if extra_damages:
        sys.stderr.write(
            f"[harness_supervisor] tracking {1 + len(extra_damages)} robot(s): "
            f"{DAMAGE_ROBOT_NAME!r} + extras "
            f"{[t.robot_name for t in extra_damages]}\n")
        sys.stderr.flush()

    # Unified event bus + producers (Phase 2). Trackers run each step
    # alongside damage.poll(); their events feed the same /sim/events
    # endpoint that fans out damage events too. Walking the scene tree
    # every step is fine for the worlds the harness sees today
    # (typically <50 solids); bigger worlds will want a longer cadence.
    #
    # P6: --light flag (passed via controllerArgs in the world file)
    # skips contact_tracker, joint_limit_tracker, grip_tracker. They
    # each walk the scene tree per step, which on multi-husky worlds
    # (newton_husky_head_on_damage = 8 huskies, ~150 solids) drops
    # the rendered framerate to slow motion. damage.poll() does not
    # depend on those trackers, so disabling them keeps the visible
    # damage demo and gets the framerate back.
    bus = EventBus()
    if LIGHT_MODE:
        sys.stderr.write("[harness_supervisor] --light: skipping contact/joint-limit/grip trackers\n")
        sys.stderr.flush()
        contact_tracker = None
        joint_limit_tracker = None
        grip_tracker = None
        # --light also implies lite damage FX: no spawned-in-scene
        # markers, debris, smoke, sparks, decals, or mesh re-emit. The
        # robots still take damage internally (HP, state transitions);
        # only the visual "rash" on top of the chassis goes away.
        # Explicit OMNISIM_LITE_DAMAGE=0 in the environment opts back in.
        if os.environ.get("OMNISIM_LITE_DAMAGE") is None:
            os.environ["OMNISIM_LITE_DAMAGE"] = "1"
    else:
        contact_tracker = ContactTracker(supervisor, bus)
        joint_limit_tracker = JointLimitTracker(supervisor, bus)
        grip_tracker = GripTracker(supervisor, bus)
    # Per-joint velocity cache: id -> (last_position, last_t_s). Shared
    # across all dispatch("robot_joints") calls so velocities are
    # meaningful across snapshots.
    joint_velocity_cache: dict = {}
    # Hook the damage tracker so its events are also visible in the
    # unified bus. We monkey-patch the existing _emit / _emit_transition
    # methods to fan-out — keeps the existing /robot/damage/events
    # endpoint working byte-for-byte.
    _orig_emit = damage._emit
    _orig_emit_transition = damage._emit_transition

    def _emit_with_fanout(sim_time_ms_inner, part, impulse_J, point, other_name):
        _orig_emit(sim_time_ms_inner, part, impulse_J, point, other_name)
        bus.emit("damage.impact", {
            "part": part,
            "impulse_J": float(impulse_J),
            "point": [float(point[0]), float(point[1]), float(point[2])],
            "other": other_name,
        }, t_sim_ms=sim_time_ms_inner)

    def _emit_transition_with_fanout(sim_time_ms_inner, part, from_state,
                                     to_state, hp, trigger_J, point=None):
        _orig_emit_transition(sim_time_ms_inner, part, from_state, to_state,
                              hp, trigger_J, point)
        bus.emit("damage.state_transition", {
            "part": part,
            "from_state": from_state,
            "to_state": to_state,
            "hp": float(hp),
            "trigger_impulse_J": float(trigger_J),
        }, t_sim_ms=sim_time_ms_inner)

    damage._emit = _emit_with_fanout
    damage._emit_transition = _emit_transition_with_fanout

    # Parse an optional inject schedule from the supervisor's own
    # customData. Lets a world script "break this part at t=20s" without
    # any external client — the supervisor runs the schedule itself.
    # Schema: {"damage_inject_schedule": [{"t_ms": int, "part": str,
    # "state": str|null, "hp_delta": float|null}, ...]}. Sorted by t_ms;
    # entries fire once when sim_time_ms first crosses their threshold.
    inject_schedule = _load_inject_schedule(supervisor)
    inject_idx = 0
    if inject_schedule:
        sys.stderr.write(
            f"[harness_supervisor] loaded {len(inject_schedule)} scheduled "
            f"inject(s)\n"
        )

    # Register the engine's own load-time state so `reset` / `sim_restore` can
    # target it. Nothing to save: the engine populated it at parse time (see
    # ENGINE_INIT_STATE). `poses` is None because the supervisor cannot sample
    # the authored poses — it starts too late (the engine has already stepped),
    # which is exactly why this state, and not a supervisor snapshot, is what
    # "reset" has to mean.
    _SNAPSHOTS[ENGINE_INIT_STATE] = {
        "name": ENGINE_INIT_STATE,
        "sim_time_ms": 0.0,
        "poses": None,
        "engine_provided": True,
        "created_wall": time.time(),
        "note": ("the engine's parse-time state: every node's authored "
                 "translation/rotation, saved by OmPose's constructor under "
                 "the default state id"),
    }

    clients: list[socket.socket] = []
    sim_time_ms = 0.0

    # Damage poll sub-sampling. damage.poll() does ~15 supervisor IPC
    # calls per invocation (one per tracked solid for getPosition + the
    # robot_node.getContactPoints call). At 62 Hz that's ~1000 IPC
    # calls/sec from damage alone, which dominates the rendered FPS on
    # multi-husky worlds. In --light we sub-sample to ~8 Hz; damage
    # state is visual feedback, not a control loop. Sweep on the head-
    # on damage world (8 huskies) measured cumulative sim-speed of
    # 1.75x at every-step polling vs 3.54x at every-8th-step, and the
    # speed during the post-collision damage-effect burst stays above
    # real-time only at >=8. 16 and 32 didn't improve further.
    # OMNISIM_DAMAGE_POLL_EVERY overrides the default (1 = every step).
    _default_poll_every = 8 if LIGHT_MODE else 1
    damage_poll_every = int(os.environ.get(
        "OMNISIM_DAMAGE_POLL_EVERY", _default_poll_every))
    damage_poll_tick = 0

    # Optional FPS measurement: ratio of sim-time advance to wall-time
    # elapsed (1.0 = real-time, <1.0 = slow motion). Enabled by setting
    # OMNISIM_FPS_LOG to a writable path; one line per FPS_REPORT_S
    # carries the cumulative + windowed speed. The poll cadence is in
    # the header so logs across runs are self-identifying.
    import time as _stdtime
    _FPS_LOG_PATH = os.environ.get("OMNISIM_FPS_LOG", "")
    FPS_REPORT_S = 2.0
    fps_t0_wall = _stdtime.time()
    fps_t0_sim_ms = 0
    fps_last_report_wall = fps_t0_wall
    if _FPS_LOG_PATH:
        try:
            with open(_FPS_LOG_PATH, "w", buffering=1) as _f:
                _f.write(
                    f"# sim_fps.log poll_every={damage_poll_every} "
                    f"light={LIGHT_MODE} basic_step_ms={basic_step_ms}\n"
                )
        except OSError:
            _FPS_LOG_PATH = ""

    while supervisor.step(basic_step_ms) != -1:
        # Webots's simulationReset rewinds sim time to 0 without
        # restarting controller processes. Detect that by watching
        # supervisor.getTime() jump backwards relative to our local
        # counter; when it does, rewind sim_time_ms and rearm the
        # inject schedule so a reloaded demo replays the same
        # detachments.
        engine_time_ms = supervisor.getTime() * 1000.0
        if engine_time_ms + 1.0 < sim_time_ms:
            # Catches a reset triggered from OUTSIDE the RPC (an in-world
            # controller or the GUI calling simulationReset). The `reset`
            # COMMAND no longer reaches this branch -- it reports
            # advanced_to_ms, so the loop has already pulled the rewound clock
            # in and there is no backwards jump left for this test to see --
            # and asks for the same work explicitly instead
            # (_rearm_after_reset, handled where the RPC result is consumed).
            was_ms = sim_time_ms
            sim_time_ms = engine_time_ms
            inject_idx = 0
            # Reset accumulated damage state too — otherwise the user
            # sees one robot starting "already destroyed" because HP
            # values and spawned debris carry over from the prior run.
            rearm_after_reset(
                (damage, *extra_damages),
                f"sim reset detected (was {was_ms:.0f}ms, now "
                f"{engine_time_ms:.0f}ms)",
                log=sys.stderr.write)
        else:
            sim_time_ms += basic_step_ms
        damage_poll_tick += 1
        if damage_poll_tick >= damage_poll_every:
            damage_poll_tick = 0
            # Defensive: a crash inside damage.poll() would kill the
            # supervisor, leaving the harness HTTP service unable to
            # service any command until the world is reloaded. Worth
            # far more than the minor risk of masking a damage-tracker
            # bug — the bug log goes to stderr where it can be
            # inspected.
            try:
                damage.poll(int(sim_time_ms))
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] damage.poll crashed: {exc}\n"
                    f"{traceback.format_exc()}"
                )
            # Secondary trackers run in parallel for visual symmetry.
            # One secondary crashing doesn't take down the others,
            # doesn't take down the primary, and doesn't kill the
            # supervisor.
            for extra in extra_damages:
                try:
                    extra.poll(int(sim_time_ms))
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(
                        f"[harness_supervisor] extra damage.poll on "
                        f"{extra.robot_name!r} crashed: {exc}\n"
                        f"{traceback.format_exc()}"
                    )
        # Phase 2 producers: contact deltas, joint-limit transitions,
        # grip detection. Each is wrapped so a producer crash on one
        # step doesn't kill the whole supervisor — stderr captures the
        # bug for inspection. Skipped in --light mode (P6).
        if contact_tracker is not None:
            try:
                contact_tracker.poll(sim_time_ms)
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] contact_tracker.poll crashed: {exc}\n"
                    f"{traceback.format_exc()}"
                )
        if joint_limit_tracker is not None:
            try:
                joint_limit_tracker.poll(sim_time_ms)
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] joint_limit_tracker.poll crashed: {exc}\n"
                    f"{traceback.format_exc()}"
                )
        if grip_tracker is not None and contact_tracker is not None:
            try:
                grip_tracker.poll(
                    contact_tracker.current_pairs(),
                    observe.build_robot_subtree_index(supervisor),
                    sim_time_ms,
                )
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] grip_tracker.poll crashed: {exc}\n"
                    f"{traceback.format_exc()}"
                )

        # Periodic FPS measurement (sim time / wall time over the last
        # FPS_REPORT_S window). Only writes if OMNISIM_FPS_LOG is set.
        if _FPS_LOG_PATH:
            _wall_now = _stdtime.time()
            if _wall_now - fps_last_report_wall >= FPS_REPORT_S:
                window_wall = _wall_now - fps_last_report_wall
                window_sim_ms = sim_time_ms - fps_t0_sim_ms
                total_wall = _wall_now - fps_t0_wall
                window_speed = (window_sim_ms / 1000.0) / max(window_wall, 1e-6)
                cumulative_speed = (sim_time_ms / 1000.0) / max(total_wall, 1e-6)
                try:
                    with open(_FPS_LOG_PATH, "a", buffering=1) as _f:
                        _f.write(
                            f"sim={sim_time_ms:7.0f}ms wall={total_wall:6.2f}s "
                            f"speed_window={window_speed:5.2f}x "
                            f"speed_cum={cumulative_speed:5.2f}x "
                            f"effective_hz={window_speed*1000.0/basic_step_ms:5.1f}\n"
                        )
                except OSError:
                    pass
                fps_last_report_wall = _wall_now
                fps_t0_sim_ms = sim_time_ms

        # Fire any scheduled injects whose t_ms is now due. Schedule is
        # sorted, so a single forward index suffices.
        while inject_idx < len(inject_schedule) and \
                sim_time_ms >= inject_schedule[inject_idx]["t_ms"]:
            entry = inject_schedule[inject_idx]
            inject_idx += 1
            try:
                damage.inject(
                    entry["part"],
                    hp_delta=entry.get("hp_delta"),
                    state=entry.get("state"),
                    sim_time_ms=int(sim_time_ms),
                )
                sys.stderr.write(
                    f"[harness_supervisor] scheduled inject t={entry['t_ms']}ms "
                    f"part={entry['part']} state={entry.get('state')}\n"
                )
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] scheduled inject failed: {exc}\n"
                )

        # Accept any pending connections (non-blocking).
        try:
            while True:
                client, _ = server.accept()
                client.setblocking(True)
                clients.append(client)
        except BlockingIOError:
            pass

        # Drain ready clients. Each iteration handles at most one frame per
        # client per step so a chatty client can't starve the sim loop.
        if clients:
            ready, _, _ = select.select(clients, [], [], 0)
            for client in ready:
                request = read_frame(client)
                if request is None:
                    try:
                        client.close()
                    except OSError:
                        pass
                    clients.remove(client)
                    continue
                req_id = request.get("id", 0)
                cmd = request.get("cmd", "")
                args = request.get("args") or {}
                try:
                    result = dispatch(supervisor, basic_step_ms, sim_time_ms, cmd, args,
                                      damage=damage, bus=bus,
                                      contact_tracker=contact_tracker,
                                      grip_tracker=grip_tracker,
                                      joint_velocity_cache=joint_velocity_cache)
                    # Any command that advanced sim time inside its own loop
                    # (step, and the mutation / snapshot verbs that settle a
                    # queued field write) reports `advanced_to_ms`; pull the
                    # new counter back so the outer loop stays in sync.
                    if isinstance(result, dict) and "advanced_to_ms" in result:
                        sim_time_ms = float(result["advanced_to_ms"])
                    # A reset command asks for its own side effects rather than
                    # letting the backwards-clock heuristic above infer them:
                    # that heuristic CANNOT fire for this command any more (the
                    # line above already pulled the rewound clock in), and it
                    # is the only thing that ever cleared damage state or
                    # rearmed the inject cursor. Without this, a damage world
                    # reset over HTTP comes back with the robot still
                    # destroyed and the schedule never replaying.
                    if isinstance(result, dict) and result.pop("_rearm_after_reset", False):
                        inject_idx = 0
                        result["reset_side_effects"] = rearm_after_reset(
                            (damage, *extra_damages), "reset command",
                            log=sys.stderr.write)
                    write_frame(client, {"id": req_id, "ok": True, "result": result})
                except CommandError as exc:
                    write_frame(client, {"id": req_id, "ok": False, "error": str(exc)})
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(f"[harness_supervisor] {cmd} crashed: {exc}\n{traceback.format_exc()}")
                    write_frame(client, {"id": req_id, "ok": False, "error": f"internal: {exc}"})

    for client in clients:
        try:
            client.close()
        except OSError:
            pass
    server.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
