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
from typing import Any


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
        return node.getTypeName() in ROBOT_TYPENAMES
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
        return node.getTypeName() in SOLID_TYPENAMES
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
    # Joint name = the device[0] motor's name when present, else the
    # endPoint solid's name. Better than nothing — gives the agent a
    # human label to correlate with the URDF.
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


def list_contacts(supervisor) -> list[dict]:
    """Aggregate the global contact set.

    Walks every Solid in the scene and asks for its contact points. Each
    contact between A and B will surface twice (once from each side); we
    dedupe on the unordered pair (a_id, b_id). The contact-point object
    has `.point` (world-frame xyz) and `.node_id` (the node id of the
    OTHER body). We resolve that id back to a DEF when one is set.
    """
    root = supervisor.getRoot()
    solids: list = []
    _walk(root, _is_solid, solids)

    # Build id -> identifier table for resolving contact node_ids.
    id_to_def: dict[int, str] = {}
    for s in solids:
        try:
            id_to_def[s.getId()] = _node_def_or_id(s)
        except Exception:
            continue

    seen: set[tuple[int, int]] = set()
    out: list[dict] = []
    for s in solids:
        try:
            sid = s.getId()
        except Exception:
            continue
        try:
            cps = s.getContactPoints(False) or []
        except Exception:
            continue
        for cp in cps:
            other_id = getattr(cp, "node_id", None)
            if other_id is None:
                continue
            key = (min(sid, other_id), max(sid, other_id))
            if key in seen:
                continue
            seen.add(key)
            try:
                point = [float(x) for x in cp.point]
            except Exception:
                continue
            out.append({
                "a_def": id_to_def.get(sid, f"#{sid}"),
                "b_def": id_to_def.get(other_id, f"#{other_id}"),
                "point": point,
            })
    return out


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
    """Map every solid's def-or-id to the def-or-id of its containing
    Robot. Used by detect_grips. Built once per snapshot/event-poll.
    """
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
