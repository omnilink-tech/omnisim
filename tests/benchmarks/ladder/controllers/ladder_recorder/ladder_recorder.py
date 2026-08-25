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

"""ladder_recorder -- the ladder's own grader-owned pose + contact sampler.

Why this exists instead of reusing ``agentbench_recorder``
----------------------------------------------------------

The AgentBench sampler answers the questions A1 asks, and it answers them
well. T1.4 is not one of them, in three ways that no argument can paper over:

1. it filters contacts down to **robot-robot** pairs and discards the rest,
   because A1.3 is "no robot touched another robot" -- and a robot-to-ground
   contact is exactly what that filter throws away;
2. it samples contacts over the first N basic timesteps, **before** the settle
   phase, so the window is over before the robot has driven anywhere;
3. it is part of a **frozen** task set with published rows against it, so
   growing a channel there would silently change what a re-run of that
   campaign measures.

So the ladder owns this one. Same discipline, same file conventions, one extra
channel: contacts naming a robot body and the **non-robot body under it**,
timestamped on the same clock as the pose series so the grader can ask whether
the robot was moving when the contact was seen.

Everything else is unchanged on purpose -- robots are enumerated by node type
plus name and never by DEF, raw samples only (every metre and second is
computed by the grader from the CSV), and the meta file is written before the
run so a death mid-way is visible as ``"complete": false``.

Controller args (all optional):
    --out=PATH            CSV path (default $LADDER_OUT, else ladder.csv)
    --duration=S          sim seconds to record after settle (default 60)
    --settle=S            sim seconds to run before recording (default 0.5)
    --contact-stride=N    sample contacts every N recorded steps (default 10;
                          0 disables the contact channel entirely)
    --surfaces=A,B        names of non-robot bodies to bound at t=0, and the
                          names a contact's other side is matched against when
                          the tier channel asks "was that the ground?"
    --exclude=A,B         robot names never recorded
    --tier=t1|t2|t3|t4    which rung this run serves (default t1). ``t1`` is
                          byte-for-byte the historical behaviour; anything
                          else ADDITIONALLY writes the tier-channel document
                          described below. Nothing about the T1 outputs
                          changes on any setting.
    --record-stride=N     write a tier-channel pose sample every N recorded
                          steps (default 5; the T1 CSV is always every step)

Outputs, all next to --out:
    <out>                 CSV: t,r0_x,r0_y,r0_z,r1_x,...  (header row)
    <out>.meta.json       roster + dt + completion flags
    <out>.support.json    t=0 bodies with world AABBs, the robot-to-surface
                          contacts seen during the recorded window, and the
                          vacuity witness (how much contact evidence EXISTED,
                          not just how much survived the filter)
    <out>.channels.json   (``--tier`` != t1 only) the tier-channel document:
                          every NAMED body's pose AND orientation over the
                          window on one clock, the frozen t=0 inventory with
                          world AABBs / masses / static flags, the full
                          contact record with both sides named, the world's
                          gravity, the per-robot controller and joint-motion
                          record, and the structural support probe.

One document, three rungs -- and why
------------------------------------

T2, T3 and T4 read the SAME ``<out>.channels.json``. That is a property of
this column rather than a shortcut: a Supervisor sees the whole scene
generically, so "every named body's pose" already contains T2's object, T2's
end effector, T3's base and T4's base, and splitting it into three documents
would be three copies of one scan. The cross-simulator contract is the
dataclasses in ``ladder.graders.ladder_evidence`` / ``t3_evidence`` /
``t4_evidence``, never an artifact shape (``adapters/mujoco/BRINGUP_T2.md``
section 7.6 says so in as many words), so a column may write whatever it likes
as long as its ``t*_channels`` produces those.

**The sampler never selects a role.** It records every named body and every
contact with both sides named; *which* body is the object, the end effector,
the container, the base or the ground is resolved by the channel builder from
the task file, and by the grader core against the task file again. A sampler
free to decide which body counts as "the object" could record the one that
happens to be in the container.
"""

import json
import os
import sys

from controller import Supervisor

ROBOT_TYPENAMES = ("Robot",)
JOINT_TYPENAMES = ("HingeJoint", "SliderJoint", "Hinge2Joint", "BallJoint")
ALWAYS_EXCLUDE = ("ladder_recorder", "agentbench_recorder",
                  "harness_supervisor")

# geometry.py / observe.py are the harness supervisor's own scene-walk
# helpers. Reused rather than reimplemented: bounds_for_subtree is the same
# world-space AABB an agent gets from the scene-tree query, so the grader and
# the agent look at the same numbers.
_HELPERS = os.path.join(
    os.environ.get("OMNISIM_HOME", ""),
    "projects", "default", "controllers", "harness_supervisor")
if _HELPERS and os.path.isdir(_HELPERS) and _HELPERS not in sys.path:
    sys.path.insert(0, _HELPERS)
try:
    import geometry as _geometry
except Exception as _exc:  # noqa: BLE001
    _geometry = None
    _GEOM_ERR = repr(_exc)
else:
    _GEOM_ERR = None
try:
    import observe as _observe
except Exception as _exc:  # noqa: BLE001
    _observe = None
    _OBS_ERR = repr(_exc)
else:
    _OBS_ERR = None


def parse_args(argv):
    out = {}
    for a in argv:
        if a.startswith("--") and "=" in a:
            k, v = a[2:].split("=", 1)
            out[k] = v
    return out


# --- scene walking -----------------------------------------------------------


def _children(node):
    if node is None:
        return []
    f = node.getField("children")
    if f is None:
        return []
    try:
        n = f.getCount()
    except Exception:
        return []
    kids = []
    for i in range(n):
        try:
            c = f.getMFNode(i)
        except Exception:
            continue
        if c is not None:
            kids.append(c)
    return kids


def _endpoint(node):
    if node is None:
        return None
    f = node.getField("endPoint")
    if f is None:
        return None
    try:
        return f.getSFNode()
    except Exception:
        return None


def _walk(root, pred, acc):
    if root is None:
        return
    try:
        if pred(root):
            acc.append(root)
    except Exception:
        pass
    for c in _children(root):
        _walk(c, pred, acc)
    ep = _endpoint(root)
    if ep is not None and ep is not root:
        _walk(ep, pred, acc)


def _typename(node):
    try:
        return node.getTypeName()
    except Exception:
        return ""


def _sf_string(node, name):
    f = node.getField(name) if node is not None else None
    if f is None:
        return ""
    try:
        return f.getSFString() or ""
    except Exception:
        return ""


def _sf_bool(node, name):
    f = node.getField(name) if node is not None else None
    if f is None:
        return None
    try:
        return bool(f.getSFBool())
    except Exception:
        return None


def _def(node):
    try:
        return node.getDef() or ""
    except Exception:
        return ""


def _nid(node):
    try:
        return int(node.getId())
    except Exception:
        return -1


def _obs_id(node):
    """The identifier the contact helper gives this node.

    Byte-for-byte the helper's own convention (DEF if set, else ``#<id>``),
    because that is the key space contacts are expressed in. Reimplemented
    rather than imported so this still works when the helper failed to load.
    """
    if node is None:
        return "#null"
    d = _def(node)
    if d:
        return d
    try:
        return "#%d" % int(node.getId())
    except Exception:
        return "#?"


def _has_physics(node):
    f = node.getField("physics") if node is not None else None
    if f is None:
        return None
    try:
        return f.getSFNode() is not None
    except Exception:
        return None


# --- the support-contact channel --------------------------------------------


def _resolve_robot(ident, subtree_index, robot_ids):
    """Which Robot a contact participant belongs to, or ``None``.

    The subtree index deliberately SKIPS the entry where a solid IS its own
    Robot, so a contact on a robot's own base link -- the single likeliest
    place for a chassis to touch the floor -- arrives as the robot's own id and
    the index lookup misses. Without the second branch this whole channel
    would report nothing while still counting the contact in the witness: a
    check that says it is watching and is not.
    """
    hit = subtree_index.get(ident)
    if hit:
        return hit
    if ident in robot_ids:
        return ident
    return None


def _support_contacts(sv, subtree_index, robot_ids, t_s, step):
    """``(pairs, witness)`` for one instant.

    A support pair is a contact where **exactly one** side resolves to a robot
    subtree. Both sides robot is a robot-robot contact and is not support;
    neither side robot is scenery touching scenery.
    """
    witness = {"total_observed": None, "distinct_named": None,
               "supported": False, "error": None}
    if _observe is None:
        witness["error"] = "the contact helper is unavailable"
        return [], witness
    try:
        pairs = _observe.list_contacts(sv)
    except Exception as exc:  # noqa: BLE001
        witness["error"] = repr(exc)[:200]
        return [], witness
    witness["supported"] = True
    witness["total_observed"] = len(pairs)
    named = 0
    out = []
    for c in pairs:
        a_def, b_def = c.get("a_def"), c.get("b_def")
        if a_def and b_def and a_def != b_def:
            named += 1
        else:
            continue
        ra = _resolve_robot(a_def, subtree_index, robot_ids)
        rb = _resolve_robot(b_def, subtree_index, robot_ids)
        if bool(ra) == bool(rb):
            continue                       # robot-robot, or scenery-scenery
        robot_side = ra or rb
        surface_side = b_def if ra else a_def
        out.append({"robot_body": robot_side, "surface_body": surface_side,
                    "surface_is_robot": False, "point": c.get("point"),
                    "t_s": t_s, "step": step})
    witness["distinct_named"] = named
    return out, witness


def _t0_bodies(sv, tracked, wanted_surfaces):
    """The frozen t=0 inventory: every tracked robot plus the named surfaces,
    each with a world-space AABB where the geometry helper can produce one."""
    out = []
    for i, r in enumerate(tracked):
        entry = {k: v for k, v in r.items() if k != "node"}
        entry["index"] = i
        entry["robot_class"] = True
        try:
            entry["position"] = [float(v) for v in r["node"].getPosition()]
        except Exception:
            entry["position"] = None
        entry["bounds"] = None
        if _geometry is not None:
            try:
                entry["bounds"] = _geometry.bounds_for_subtree(r["node"])
            except Exception as exc:  # noqa: BLE001
                entry["bounds_error"] = repr(exc)
        out.append(entry)

    if wanted_surfaces:
        solids = []
        _walk(sv.getRoot(), lambda n: _typename(n) == "Solid", solids)
        for node in solids:
            nm = _sf_string(node, "name")
            if nm not in wanted_surfaces:
                continue
            entry = {"name": nm, "def": _def(node), "id": _nid(node),
                     "type": "Solid", "robot_class": False, "bounds": None,
                     "has_physics": _has_physics(node)}
            try:
                entry["position"] = [float(x) for x in node.getPosition()]
            except Exception:
                entry["position"] = None
            if _geometry is not None:
                try:
                    entry["bounds"] = _geometry.bounds_for_subtree(node)
                except Exception as exc:  # noqa: BLE001
                    entry["bounds_error"] = repr(exc)
            out.append(entry)
    return out


# --- the tier channels (T2/T3/T4) --------------------------------------------

#: ⚠ **The tier-channel pose series may never be sparser than this, and the
#: number is not a preference.** T3/T4's continuity clause requires a pose
#: series sampled at most every 0.05 s -- coarser and the clause reports its
#: witness absent and cannot fail. Measured on the first T3 probe: a fixed
#: stride of 5 on a 16 ms world gave 0.08 s and turned T3.1's
#: "one continuous run" clause vacuous while every other channel was fine.
#: So ``--record-stride`` is a CEILING that the sampler tightens against the
#: world's own basic timestep, never a rate it obeys blindly.
MAX_RECORD_DT_S = 0.04

POSE_SOURCE = (
    "the grader-owned sampler: wb_supervisor_node_get_position and "
    "wb_supervisor_node_get_orientation for the named body, every %d basic "
    "timesteps (%.4f s), in simulated seconds and metres, world frame. "
    "getOrientation returns a world-from-body 3x3 directly, so no quaternion "
    "or Euler convention is involved")

INVENTORY_SOURCE = (
    "the grader-owned sampler's frozen scan taken after the settle window and "
    "before the first recorded step, with the controller synchronized so "
    "nothing can advance the clock while it runs. World AABBs come from the "
    "harness supervisor's own bounds_for_subtree -- the SAME world-space box "
    "an agent gets from GET /scene/tree?bounds=1, so the grader and the agent "
    "look at the same numbers")

MASS_SOURCE = (
    "the Solid's own Physics node: its 'mass' field in kilograms where the "
    "scene states one, summed over the subtree for subtree_mass_kg. A Physics "
    "node whose mass is negative is DENSITY-based and the engine derives the "
    "kilograms from the boundingObject's volume at build time; the supervisor "
    "API has no read-back for that derived value (there is no getMass), so "
    "such a body reports mass_kg=null with the reason rather than a number "
    "this sampler multiplied out itself. 'dynamic' is 'a Physics node is "
    "attached', which is exactly what decides whether the body can move")

CONTACT_SOURCE = (
    "the grader-owned sampler's per-step contact scan (the harness "
    "supervisor's own paired query over every Solid's getContactPoints, "
    "joined on the world contact point), emitted every %d recorded steps with "
    "BOTH sides named and the simulated time the query ran. 'is the ground' "
    "is a NAME match against the names the task declared, never a guess; the "
    "core takes the stricter of that and its own list. A contact only one "
    "side reported (a PROTO floor a Supervisor cannot walk) is kept with the "
    "other side null rather than dropped. WARNING, MEASURED: this query is "
    "ODE-only -- under the Newton backend it runs cleanly and returns ZERO "
    "contacts for a scene that returns thousands on ODE")

GRAVITY_SOURCE = (
    "WorldInfo.gravity of the scene that ran, read by the grader-owned "
    "supervisor in BOTH declared shapes and never assumed: this engine "
    "declares the field as an SFFloat magnitude acting along -Z, upstream "
    "R2022+ declares it as an SFVec3f, and reading only the vector form "
    "reported gravity absent on a world that plainly had it")

ARENA_SOURCE = (
    "the union of the world AABBs of the bodies whose names the task declared "
    "as the walking surface, from the frozen t=0 inventory. boundary_bodies "
    "are the STATIC non-surface bodies whose vertical extent is at least "
    "0.20 m -- the walls that end a run -- named so a travel figure can be "
    "read against the room that allowed it")

CONTROLLER_SOURCE = (
    "the robot's own declared 'controller' and 'controllerArgs' fields plus a "
    "POSITIVE motion attestation: the number of the robot's joints whose "
    "position changed by more than 1e-4 between the first and last recorded "
    "sample. It is deliberately not an exit code -- the trap this exists for "
    "is a deploy whose model runtime was missing, which runs a zero-residual "
    "baseline and exits 0. A robot whose controller field is empty or "
    "'<none>' is reported as loaded=false; one that is declared and moved "
    "nothing is reported as loaded=false WITH the joint count, and one that "
    "is declared and moved joints is loaded=true")

SUPPORT_SOURCE_HEAD = (
    "a STRUCTURAL enumeration of every route by which a non-gravitational, "
    "non-contact wrench could reach a body in this scene, NOT a wrench "
    "read-back -- OmniSim has none. wb_supervisor_node_add_force / "
    "add_torque are write-only from a Supervisor controller and nothing in "
    "the supervisor API reports what another controller applied; contact "
    "points carry no force either (see the contact channel). So this column "
    "can attest the total is ZERO only by proving no route is open, and must "
    "report 'unverified' the moment one is. The routes: (1) any Robot in the "
    "scene declaring supervisor TRUE other than the grader's own sampler -- "
    "it can call add_force on any node; (2) a robot whose base carries no "
    "Physics node, which is held rigidly by the engine; (3) a robot that is "
    "not a child of the scene root, i.e. parented into another body's "
    "subtree; (4) a Connector device anywhere in a robot's subtree, which can "
    "lock to another body; (5) a physics plugin declared in WorldInfo.physics, "
    "which runs arbitrary ODE force calls every step")


def _walk_named(root, acc, depth=0, limit=4000):
    """Every Robot/Solid carrying a non-empty ``name``, with its tree depth."""
    if root is None or len(acc) >= limit:
        return
    try:
        tn = _typename(root)
    except Exception:
        tn = ""
    if tn in ROBOT_TYPENAMES or tn == "Solid":
        nm = _sf_string(root, "name")
        if nm:
            acc.append({"node": root, "name": nm, "def": _def(root),
                        "id": _nid(root), "type": tn, "depth": depth,
                        "robot_class": tn in ROBOT_TYPENAMES})
    for c in _children(root):
        _walk_named(c, acc, depth + 1, limit)
    ep = _endpoint(root)
    if ep is not None and ep is not root:
        _walk_named(ep, acc, depth + 1, limit)


def _physics_node(node):
    f = node.getField("physics") if node is not None else None
    if f is None:
        return None
    try:
        return f.getSFNode()
    except Exception:
        return None


def _own_mass(node):
    """``(mass_kg_or_None, reason_or_None)`` for one body's own Physics node."""
    p = _physics_node(node)
    if p is None:
        return None, "no Physics node is attached, so the body is static"
    f = p.getField("mass")
    if f is None:
        return None, "the Physics node has no mass field"
    try:
        m = float(f.getSFFloat())
    except Exception as exc:  # noqa: BLE001
        return None, repr(exc)[:120]
    if m < 0:
        return None, ("the Physics node states mass %.6g, i.e. DENSITY-based: "
                      "the engine derives kilograms from the boundingObject "
                      "volume and the supervisor API has no read-back for the "
                      "derived value" % m)
    return m, None


def _subtree_mass(node):
    """``(kg, n_bodies, n_unreadable)`` summed over the whole subtree."""
    solids = []
    _walk(node, lambda n: _typename(n) in ROBOT_TYPENAMES
          or _typename(n) == "Solid", solids)
    total, counted, unreadable = 0.0, 0, 0
    for s in solids:
        m, _why = _own_mass(s)
        if m is None:
            if _physics_node(s) is not None:
                unreadable += 1
            continue
        total += m
        counted += 1
    return (total if counted else None), counted, unreadable


def _orientation(node):
    try:
        return [float(v) for v in node.getOrientation()]
    except Exception:
        return None


def _parent_summary(node):
    try:
        p = node.getParentNode()
    except Exception:
        return None
    if p is None:
        return None
    return {"type": _typename(p), "name": _sf_string(p, "name"),
            "id": _nid(p), "is_scene_root": _typename(p) in ("", "Group")
            and _nid(p) <= 0}


def _joint_positions(node):
    """``{joint name or id: position}`` for every joint in a subtree."""
    joints = []
    _walk(node, lambda n: _typename(n) in JOINT_TYPENAMES, joints)
    out = {}
    for i, j in enumerate(joints):
        key = None
        try:
            params = j.getField("jointParameters")
            pn = params.getSFNode() if params is not None else None
        except Exception:
            pn = None
        if pn is not None:
            key = _sf_string(pn, "name")
            f = pn.getField("position")
            try:
                val = float(f.getSFFloat()) if f is not None else None
            except Exception:
                val = None
        else:
            val = None
        out[key or ("joint_%d" % i)] = val
    return out


def _has_connector(node):
    hits = []
    _walk(node, lambda n: _typename(n) == "Connector", hits)
    return len(hits)


def _world_info(sv):
    """The scene's WorldInfo node, or ``None``.

    Walked here rather than through the harness helper on purpose: gravity
    and the physics-plugin field decide T2.4's datum and a T3/T4 support
    route, and neither may go unanswered because an optional import failed.
    """
    try:
        root = sv.getRoot()
    except Exception:
        return None
    for child in _children(root):
        if _typename(child) == "WorldInfo":
            return child
    return None


def _world_gravity(sv):
    """``(vec, magnitude, error)`` from WorldInfo.gravity."""
    wi = _world_info(sv)
    if wi is None:
        return None, None, "the scene has no WorldInfo node"
    f = wi.getField("gravity")
    if f is None:
        return None, None, "WorldInfo has no gravity field"
    # Two shapes are live in this lineage and BOTH are read rather than
    # assumed: upstream R2022+ declares gravity as an SFVec3f, and this engine
    # declares it as an SFFloat magnitude acting along -Z. Guessing wrong
    # here would report gravity absent on a world that plainly has it.
    v = None
    try:
        v = [float(x) for x in f.getSFVec3f()]
    except Exception:
        v = None
    if v is None:
        try:
            g = float(f.getSFFloat())
        except Exception as exc:  # noqa: BLE001
            return None, None, repr(exc)[:120]
        return [0.0, 0.0, -abs(g)], abs(g), None
    mag = (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5
    return v, mag, None


#: What ``WorldInfo.physics`` reads as when no plugin is declared. The engine
#: writes the literal string ``<none>`` rather than leaving the field empty,
#: and treating that as a declared plugin would open a support route on every
#: scene ever authored -- measured on the T3 probe, where it alone turned an
#: otherwise-clean structural probe into ``unverified``.
NO_PLUGIN = ("", "<none>", "none")


def _physics_plugin(sv):
    wi = _world_info(sv)
    name = _sf_string(wi, "physics") if wi is not None else ""
    return None if (name or "").strip().lower() in NO_PLUGIN else name


class ChannelRecorder:
    """The T2/T3/T4 half of the sampler. Never raises into the record loop."""

    def __init__(self, sv, tracked, surfaces, stride, contact_stride, dt_ms):
        self.sv = sv
        self.surfaces = set(surfaces or ())
        self.stride = max(1, int(stride))
        self.contact_stride = int(contact_stride)
        self.dt_ms = dt_ms
        self.errors = []
        self.t_s = []
        self.bodies = []
        self.contacts = []
        self.contact_times = []
        self.contact_steps = 0
        self.contact_total = 0
        self.contact_named = 0
        self.contact_supported = False
        self.contact_error = None
        self.robot_ids = set()
        self.subtree_index = {}
        if _observe is not None:
            try:
                self.subtree_index = _observe.build_robot_subtree_index(sv)
            except Exception as exc:  # noqa: BLE001
                self.errors.append("subtree index failed: %r" % (exc,))
        self.tracked = tracked
        self.joints_start = {}
        self.joints_end = {}
        try:
            found = []
            _walk_named(sv.getRoot(), found)
            for b in found:
                b["xyz"] = []
                b["rot"] = []
                b["parent"] = _parent_summary(b["node"])
                self.bodies.append(b)
        except Exception as exc:  # noqa: BLE001
            self.errors.append("body scan failed: %r" % (exc,))

    # -- the frozen t=0 inventory ------------------------------------------
    def inventory(self):
        out = []
        for b in self.bodies:
            node = b["node"]
            mass, why = _own_mass(node)
            sub, counted, unreadable = _subtree_mass(node)
            ident = _obs_id(node)
            entry = {"name": b["name"], "def": b["def"], "id": b["id"],
                     "type": b["type"], "depth": b["depth"],
                     "robot_class": b["robot_class"],
                     "ident": ident,
                     "is_grader_sampler": b["name"] in ALWAYS_EXCLUDE,
                     "owner_robot": (ident if b["robot_class"]
                                     else self.subtree_index.get(ident)),
                     "parent": b["parent"],
                     "has_physics": _has_physics(node),
                     "static": _physics_node(node) is None,
                     "mass_kg": mass, "mass_error": why,
                     "subtree_mass_kg": sub,
                     "subtree_bodies_with_mass": counted,
                     "subtree_bodies_mass_unreadable": unreadable,
                     "connectors_in_subtree": _has_connector(node),
                     "aabb_min": None, "aabb_max": None, "bounds_error": None}
            try:
                entry["position"] = [float(v) for v in node.getPosition()]
            except Exception:
                entry["position"] = None
            if _geometry is not None:
                try:
                    bd = _geometry.bounds_for_subtree(node)
                except Exception as exc:  # noqa: BLE001
                    entry["bounds_error"] = repr(exc)[:160]
                    bd = None
                if bd:
                    entry["aabb_min"] = list(bd.get("bbox_min") or []) or None
                    entry["aabb_max"] = list(bd.get("bbox_max") or []) or None
                    entry["bounds_exact"] = bd.get("exact")
            else:
                entry["bounds_error"] = _GEOM_ERR
            out.append(entry)
        return out

    # -- the structural support probe --------------------------------------
    def structure(self):
        foreign = []
        for b in self.bodies:
            if not b["robot_class"]:
                continue
            if b["name"] in ALWAYS_EXCLUDE:
                continue
            if _sf_bool(b["node"], "supervisor"):
                foreign.append(b["name"])
        held = []
        parented = []
        connectors = []
        for r in self.tracked:
            node = r["node"]
            if _physics_node(node) is None:
                held.append(r["name"])
            par = _parent_summary(node)
            if par is not None and par.get("type") not in ("", "Group", None):
                parented.append({"robot": r["name"], "parent": par})
            n_conn = _has_connector(node)
            if n_conn:
                connectors.append({"robot": r["name"], "connectors": n_conn})
        plugin = _physics_plugin(self.sv)
        routes = []
        if foreign:
            routes.append("a Supervisor robot other than the grader's sampler "
                          "is present (%s): it can call add_force/add_torque "
                          "on any node and nothing reports what it applied"
                          % ", ".join(sorted(foreign)))
        if held:
            routes.append("a tracked robot carries no Physics node on its own "
                          "body (%s), so the engine holds it rigidly"
                          % ", ".join(sorted(held)))
        for p in parented:
            routes.append("robot %r is parented inside a %s, so its pose is "
                          "carried rather than simulated"
                          % (p["robot"], p["parent"].get("type")))
        for c in connectors:
            routes.append("robot %r carries %d Connector device(s), which can "
                          "lock it to another body"
                          % (c["robot"], c["connectors"]))
        if plugin:
            routes.append("WorldInfo.physics declares the plugin %r, which "
                          "runs arbitrary force calls every step" % plugin)
        return {"foreign_supervisors": sorted(foreign),
                "our_sampler": SAMPLER_NAME,
                "robots_without_physics": sorted(held),
                "robots_parented_into_a_body": parented,
                "robots_with_connectors": connectors,
                "physics_plugin": plugin or None,
                "routes_open": routes,
                "attested": not routes,
                "source": SUPPORT_SOURCE_HEAD}

    # -- per-sample --------------------------------------------------------
    def sample(self, t, k):
        if k % self.stride:
            return
        self.t_s.append(t)
        for b in self.bodies:
            node = b["node"]
            try:
                p = [float(v) for v in node.getPosition()]
            except Exception:
                p = [float("nan")] * 3
            b["xyz"].append(p)
            b["rot"].append(_orientation(node) or [float("nan")] * 9)

    def scan(self, t, k, subtree_index, robot_ids):
        if self.contact_stride <= 0 or (k % self.contact_stride):
            return
        self.contact_steps += 1
        self.contact_times.append(t)
        if _observe is None:
            self.contact_error = self.contact_error or (
                "the contact helper is unavailable")
            return
        try:
            pairs = _observe.list_contacts(self.sv)
        except Exception as exc:  # noqa: BLE001
            self.contact_error = self.contact_error or repr(exc)[:200]
            return
        self.contact_supported = True
        self.contact_total += len(pairs)
        for c in pairs:
            a_def, b_def = c.get("a_def"), c.get("b_def")
            a_name = c.get("a_name") or ""
            b_name = c.get("b_name") or ""
            if a_def and b_def and a_def != b_def:
                self.contact_named += 1
            ra = _resolve_robot(a_def, subtree_index, robot_ids)
            rb = _resolve_robot(b_def, subtree_index, robot_ids)
            self.contacts.append({
                "a": a_def, "b": b_def, "a_name": a_name, "b_name": b_name,
                "a_robot": ra, "b_robot": rb,
                "a_is_ground": (bool(a_name in self.surfaces)
                                if a_name else None),
                "b_is_ground": (bool(b_name in self.surfaces)
                                if b_name else None),
                "point": c.get("point"), "t_s": t, "step": k,
                "paired": bool(c.get("paired"))})

    def snapshot_joints(self, into):
        for r in self.tracked:
            try:
                into[r["name"]] = _joint_positions(r["node"])
            except Exception as exc:  # noqa: BLE001
                self.errors.append("joint read failed for %r: %r"
                                   % (r["name"], exc))

    # -- the document ------------------------------------------------------
    def controllers(self):
        out = []
        for r in self.tracked:
            start = self.joints_start.get(r["name"]) or {}
            end = self.joints_end.get(r["name"]) or {}
            moved = 0
            for key, v0 in start.items():
                v1 = end.get(key)
                if v0 is None or v1 is None:
                    continue
                if abs(float(v1) - float(v0)) > 1e-4:
                    moved += 1
            name = (r.get("controller") or "").strip()
            declared = bool(name) and name != "<none>"
            out.append({"robot": r["name"], "controller": name,
                        "declared": declared,
                        "joints": len(start), "joints_moved": moved,
                        "loaded": bool(declared and moved > 0)
                        if (declared or start) else None,
                        "evidence": ("controller %r declared; %d of %d joints "
                                     "moved by more than 1e-4 over the "
                                     "recorded window"
                                     % (name or "<none>", moved, len(start))),
                        "source": CONTROLLER_SOURCE})
        return out

    def document(self, tier, window_s, inventory):
        vec, mag, gerr = _world_gravity(self.sv)
        return {
            "tier": tier,
            "schema": "omnisim_ladder_channels/v1",
            "basic_timestep_ms": self.dt_ms,
            "record_stride_steps": self.stride,
            "contact_stride_steps": self.contact_stride,
            "window_s": window_s,
            "t_s": self.t_s,
            "pose_source": POSE_SOURCE % (self.stride,
                                          self.stride * self.dt_ms / 1000.0),
            "bodies": [{"name": b["name"], "def": b["def"], "id": b["id"],
                        "type": b["type"], "depth": b["depth"],
                        "robot_class": b["robot_class"],
                        "is_grader_sampler": b["name"] in ALWAYS_EXCLUDE,
                        "xyz": b["xyz"], "rot": b["rot"]}
                       for b in self.bodies],
            "inventory": inventory,
            "inventory_source": INVENTORY_SOURCE,
            "mass_source": MASS_SOURCE,
            "arena_source": ARENA_SOURCE,
            "declared_surfaces": sorted(self.surfaces),
            "world": {"gravity_vec_mps2": vec, "gravity_mps2": mag,
                      "source": GRAVITY_SOURCE, "error": gerr},
            "contacts": {"supported": self.contact_supported,
                         "steps": self.contact_steps,
                         "window_s": window_s,
                         "total_observed": (self.contact_total
                                            if self.contact_supported
                                            else None),
                         "distinct_named": (self.contact_named
                                            if self.contact_supported
                                            else None),
                         "sample_times": self.contact_times,
                         "emit_stride_steps": self.contact_stride,
                         "source": CONTACT_SOURCE % max(1,
                                                        self.contact_stride),
                         "error": self.contact_error,
                         "pairs": self.contacts},
            "structure": self.structure(),
            "controllers": self.controllers(),
            "errors": self.errors,
        }


SAMPLER_NAME = "ladder_recorder"


# --- main --------------------------------------------------------------------


def main():
    args = parse_args(sys.argv[1:])
    sv = Supervisor()

    out_path = args.get("out") or os.environ.get("LADDER_OUT", "ladder.csv")
    duration = float(args.get("duration", "60.0"))
    settle = float(args.get("settle", "0.5"))
    stride = int(args.get("contact-stride", "10"))
    tier = (args.get("tier", "t1") or "t1").strip().lower()
    record_stride = max(1, int(args.get("record-stride", "5")))
    surfaces = {s.strip() for s in args.get("surfaces", "").split(",")
                if s.strip()}
    excluded = set(ALWAYS_EXCLUDE)
    for name in args.get("exclude", "").split(","):
        if name.strip():
            excluded.add(name.strip())

    dt_ms = sv.getBasicTimeStep()
    step_ms = int(round(dt_ms))
    n_record = int(round(duration * 1000.0 / dt_ms))
    n_settle = int(round(settle * 1000.0 / dt_ms))

    robot_nodes = []
    _walk(sv.getRoot(), lambda n: _typename(n) in ROBOT_TYPENAMES, robot_nodes)
    tracked = []
    for node in robot_nodes:
        name = _sf_string(node, "name")
        if name in excluded:
            continue
        joints = []
        _walk(node, lambda n: _typename(n) in JOINT_TYPENAMES, joints)
        tracked.append({"node": node, "name": name, "def": _def(node),
                        "id": _nid(node),
                        "controller": _sf_string(node, "controller"),
                        "supervisor": _sf_bool(node, "supervisor"),
                        "has_physics": _has_physics(node),
                        "num_joints": len(joints), "type": _typename(node)})

    roster = [{k: v for k, v in r.items() if k != "node"} for r in tracked]
    meta = {"out": os.path.abspath(out_path), "world": sv.getWorldPath(),
            "dt_ms": dt_ms, "settle_s": settle, "duration_s": duration,
            "robots": roster, "n_robots": len(tracked), "rows": 0,
            "recorded_s": 0.0, "complete": False, "quit_called": False,
            "step_returned_minus1": False, "contact_stride": stride}

    out_dir = os.path.dirname(os.path.abspath(out_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    def write_meta():
        with open(out_path + ".meta.json", "w", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2)

    write_meta()   # exists even if the run dies mid-way ("complete": false)

    support = {"pairs": [], "t0_bodies": [], "stride": stride,
               "dt_ms": dt_ms, "bounds_error": _GEOM_ERR,
               "observe_error": _OBS_ERR,
               "witness": {"total_observed": None, "distinct_named": None,
                           "supported": False, "steps_sampled": 0,
                           "error": None},
               "window_s": 0.0}

    def write_support():
        with open(out_path + ".support.json", "w", encoding="utf-8") as fh:
            json.dump(support, fh, indent=2)

    # The tier channels. Built for every tier but T1 so a T1 run stays
    # byte-for-byte what it was; a construction failure is recorded and the
    # run continues, because a broken extra channel must never cost a T1 row.
    chan = None
    record_stride = max(1, min(record_stride,
                               int(MAX_RECORD_DT_S * 1000.0 / max(dt_ms,
                                                                  1e-6))))
    if tier != "t1":
        try:
            chan = ChannelRecorder(sv, tracked, surfaces, record_stride,
                                   stride, dt_ms)
        except Exception as exc:  # noqa: BLE001
            print("[ladder_recorder] tier-channel setup failed: %r" % (exc,),
                  flush=True)
            chan = None
    meta["tier"] = tier
    meta["record_stride"] = record_stride
    meta["tier_channels"] = chan is not None

    # The inventory is FROZEN once, after the settle and before the first
    # recorded step, and carried here -- not recomputed at exit, where the
    # container would have been moved and the "t=0 geometry" would be a lie.
    tier_state = {"inventory": [], "frozen": False}

    def write_channels():
        if chan is None:
            return
        try:
            doc = chan.document(tier, meta.get("recorded_s", 0.0),
                                tier_state["inventory"])
            doc["inventory_frozen"] = tier_state["frozen"]
            with open(out_path + ".channels.json", "w",
                      encoding="utf-8") as fh:
                json.dump(doc, fh)
        except Exception as exc:  # noqa: BLE001
            print("[ladder_recorder] tier-channel write failed: %r" % (exc,),
                  flush=True)

    def finish(complete):
        meta["complete"] = complete
        meta["quit_called"] = True
        write_meta()
        write_support()
        write_channels()
        sv.simulationQuit(0)
        sv.step(step_ms)

    if not tracked:
        print("[ladder_recorder] no robots to track; quitting", flush=True)
        with open(out_path, "w", encoding="utf-8", newline="") as fh:
            fh.write("t\n")
        support["t0_bodies"] = _t0_bodies(sv, [], surfaces)
        support["witness"]["error"] = ("no robot body in the scene, so no "
                                       "support contact is possible")
        finish(True)
        return

    print("[ladder_recorder] tracking %d robot(s): %s"
          % (len(tracked), ", ".join(r["name"] or r["def"] or "?"
                                     for r in tracked)), flush=True)

    # --- the frozen t=0 inventory (this controller IS synchronized) ---------
    support["t0_bodies"] = _t0_bodies(sv, tracked, surfaces)

    subtree_index = {}
    if _observe is not None:
        try:
            subtree_index = _observe.build_robot_subtree_index(sv)
        except Exception as exc:  # noqa: BLE001
            support["observe_error"] = repr(exc)
    robot_ids = set()
    for node in robot_nodes:
        ident = _obs_id(node)
        if ident:
            robot_ids.add(ident)
    support["robot_ids"] = sorted(robot_ids)

    cw = support["witness"]
    cw_totals = {"total_observed": 0, "distinct_named": 0}

    def accumulate(w):
        cw["steps_sampled"] += 1
        if w.get("supported"):
            cw["supported"] = True
        for key in ("total_observed", "distinct_named"):
            v = w.get(key)
            if v is not None:
                cw_totals[key] += v
        if w.get("error") and not cw["error"]:
            cw["error"] = w["error"]

    broke = False

    # --- settle --------------------------------------------------------------
    for _ in range(n_settle):
        if sv.step(step_ms) == -1:
            meta["step_returned_minus1"] = True
            broke = True
            break

    # --- the frozen tier inventory (after the settle, before step 1) ---------
    if chan is not None and not broke:
        try:
            tier_state["inventory"] = chan.inventory()
            tier_state["frozen"] = True
            chan.snapshot_joints(chan.joints_start)
        except Exception as exc:  # noqa: BLE001
            print("[ladder_recorder] tier inventory failed: %r" % (exc,),
                  flush=True)

    # --- record --------------------------------------------------------------
    header = ["t"]
    for i in range(len(tracked)):
        header += ["r%d_x" % i, "r%d_y" % i, "r%d_z" % i]

    rows = 0
    with open(out_path, "w", encoding="utf-8", newline="") as fh:
        fh.write(",".join(header) + "\n")

        def sample(t):
            vals = ["%.17g" % t]
            for r in tracked:
                try:
                    p = r["node"].getPosition()
                    vals += ["%.17g" % float(p[0]), "%.17g" % float(p[1]),
                             "%.17g" % float(p[2])]
                except Exception:
                    vals += ["nan", "nan", "nan"]
            fh.write(",".join(vals) + "\n")

        def scan(t, k):
            if chan is not None:
                chan.scan(t, k, subtree_index, robot_ids)
            if stride <= 0 or (k % stride):
                return
            pairs, w = _support_contacts(sv, subtree_index, robot_ids, t, k)
            accumulate(w)
            support["pairs"].extend(pairs)

        if not broke:
            sample(0.0)
            if chan is not None:
                chan.sample(0.0, 0)
            scan(0.0, 0)
            rows = 1
            for k in range(n_record):
                if sv.step(step_ms) == -1:
                    meta["step_returned_minus1"] = True
                    broke = True
                    break
                t = (k + 1) * dt_ms / 1000.0
                sample(t)
                if chan is not None:
                    chan.sample(t, k + 1)
                scan(t, k + 1)
                rows += 1

    if chan is not None:
        chan.snapshot_joints(chan.joints_end)

    meta["rows"] = rows
    meta["recorded_s"] = (rows - 1) * dt_ms / 1000.0 if rows else 0.0
    support["window_s"] = meta["recorded_s"]
    if cw["supported"]:
        cw["total_observed"] = cw_totals["total_observed"]
        cw["distinct_named"] = cw_totals["distinct_named"]

    complete = (not broke) and rows == n_record + 1
    print("[ladder_recorder] wrote %d rows (%.3f sim s) -> %s (complete=%s); "
          "support contacts: %d over %d sampled steps (witness: %s of any "
          "kind, %s naming two distinct bodies, supported=%s)"
          % (rows, meta["recorded_s"], out_path, complete,
             len(support["pairs"]), cw["steps_sampled"],
             cw["total_observed"], cw["distinct_named"], cw["supported"]),
          flush=True)
    finish(complete)


if __name__ == "__main__":
    main()
