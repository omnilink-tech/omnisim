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

"""agentbench_recorder — the grader-owned pose recorder (AgentBench SPEC 4.5.1).

Phase-B motion measurement does NOT poll the harness (the harness free-runs
between RPCs and /scene/tree costs seconds). Instead the grader copies the
agent's world, appends

    Robot { name "agentbench_recorder" controller "agentbench_recorder"
            supervisor TRUE controllerArgs [...] }

and runs it headless. This controller then samples the world pose of every
tracked body once per basic timestep in sim time, writes ``%.17g`` CSV, and
calls ``simulationQuit(0)`` at the target sim time.

**What "tracked body" means, and why it is three things and not one.** Until
2026-08-09 it meant *Robot node*, full stop -- and that single predicate was
the gap that blocked the whole robotics tier. R2 needs the arm's END EFFECTOR
over time, and an arm authored as one ``Robot`` exposes only its root; R3 needs
the CUBE's trajectory, and a cube is a ``Solid``. Neither is a Robot node, so
neither had a track at all and both tasks failed for want of evidence that the
*recorder* never collected. There are now three track kinds, in this CSV order:

    ``robot``  every Robot node, exactly as before -- SAME order, SAME columns,
               so every existing grader's roster-index-into-trajectory read is
               untouched. This is the backward-compatibility contract.
    ``link``   moving bodies INSIDE a robot (``--links=N``, default OFF).
    ``solid``  named non-robot Solids (``--solids=``), which now get a per-step
               pose track as well as their t=0 bounds.

Both additions are appended AFTER the robot rows and both are OPT-IN, because
row count is load-bearing: ``a1_core`` asserts ``n_bodies == 10``, so silently
handing A1 forty wheel tracks would turn a passing swarm into a failing one.

Deliberate differences from omnibench/lane1's recorder, per SPEC 4.5.1:

  * robots are enumerated by **node type + name**, never by DEF -- the agent
    has no reason to add DEFs and a grader that requires them grades our
    conventions instead of the task;
  * it records raw samples only. Path length, net displacement, bearing and
    the z-band test are computed by the grader from the CSV, so the physical
    assertions live in one sim-agnostic place.

**Why the t=0 structure pass lives here and not in the harness.** SPEC 2.3
puts A1.3 ("none interpenetrating at t=0") on the harness. It cannot go there:
the harness's injected supervisor is ``synchronization FALSE``, so the world
free-runs in ``--mode=fast`` between RPCs -- measured, a ``GET /robots`` on a
10-Husky world took 22 s, by which time every robot had driven >10 m. There is
no pause verb on the harness surface. This controller IS synchronized, so the
world is genuinely frozen at t=0 while it walks the geometry. The harness pass
still runs and still supplies the assertions it can answer honestly (roster,
node ids, controller fields) -- see adapters/omnisim/harness.py.

Controller args (all optional):
    --out=PATH          CSV path (default $AGENTBENCH_OUT, else agentbench.csv)
    --duration=S        sim seconds to record AFTER settle (default 30.0)
    --settle=S          sim seconds to run before recording starts (default 1.0)
    --exclude=A,B       robot names never recorded (the recorder excludes
                        itself and "harness_supervisor" unconditionally)
    --phase-a=1         do the t=0 structure/bounds scan + contact watch
    --contact-steps=10  basic timesteps to watch for robot-robot contact
                        AT t=0 (the A1.3 interpenetration question)
    --run-contacts=8    sample RUN-LONG contacts every N basic timesteps of
                        the RECORDING window, naming the robot's partner
                        whatever it is (the "did it hit anything while it
                        drove" question). 0 turns it off; ON by default
    --robot-contacts-only=1
                        query each Robot subtree deeply instead of every Solid;
                        exact for robot-robot assertions. The recorder enables
                        contact tracking on those Robot nodes, so the engine
                        batches the sampled contact sets into normal step
                        replies instead of forcing one IPC flush per Robot
    --contact-witness-defs=A,B
                        additional Solid DEFs queried through that same contact
                        channel. A non-colliding fleet can otherwise prove zero
                        pairs but not prove that the channel can name a pair;
                        adapter-owned calibration bodies close that vacuity
    --solids=A,B        named non-robot Solids: t=0 world bounds AND (subject
                        to --solid-tracks) a per-step pose track
    --solid-tracks=MODE which of those named Solids get a per-step track --
                        "dynamic" (default: only ones carrying a mass model),
                        "all", or "none" (the pre-2026-08-09 behaviour)
    --links=N           per-robot cap on LINK tracks; 0 (the default) is off
    --scan-solids=N     cap on the NAME-FREE t=0 scene scan (default
                        SCENE_SCAN_DEFAULT_CAP; 0 turns it off)

Outputs, all next to --out:
    <out>                CSV: t,r0_x,r0_y,r0_z,r1_x,...  (header row). Robot
                         columns keep the r<i> prefix and their exact original
                         order; link columns are l<i>, solid columns s<i>, and
                         both only exist when asked for.
    <out>.meta.json      roster + dt + completion flags, plus "tracks": the
                         CSV column map (one entry per xyz triple, in order),
                         plus "run_contacts": the run-long contact watch
                         (below). Its presence with "complete": true is the
                         recorder's own proof that the world built, stepped to
                         the target sim time, and quit.
    <out>.phase_a.json   t=0 per-robot world AABB + the robot-robot contacts
                         seen during the first --contact-steps steps, plus
                         t0_solids / t0_links for the other two track kinds,
                         plus t0_scene -- the NAME-FREE scan (below).

**The name-free t=0 scene scan (``t0_scene``), and why it had to exist.**
``--solids=`` is a NAME LIST, and every geometric assertion in the suite matches
by GEOMETRY -- deliberately, because an agent names things freely: R1 published
``OBSTACLE_1``..``OBSTACLE_5`` and the first real agent called its boxes ``crate
A``..``crate E``, a second called them ``obstacle_1``..``obstacle_6``. A grader
keyed on our published names scores a correct world zero, which is grading OUR
CONVENTION instead of the task -- so ``r1_core.match_spec_obstacles`` matches by
world-space AABB centre and footprint and never by name. But the only bounds
channel this recorder had was the name-keyed one, so the geometric matcher saw
NO CANDIDATES AT ALL and R1.3 could not pass whatever the agent built. That is
an instrument gap, not an agent failure, and this scan closes it: every
non-robot ``Solid`` in the scene is offered with its world-space AABB and its
identity, and the grader decides by geometry. The MuJoCo arm has bounded every
body and every world geom with no name list since it was written; this is the
same contract on ours.
"""

import json
import os
import sys

from controller import Supervisor

ROBOT_TYPENAMES = ("Robot",)
JOINT_TYPENAMES = ("HingeJoint", "SliderJoint", "Hinge2Joint", "BallJoint")
ALWAYS_EXCLUDE = ("agentbench_recorder", "harness_supervisor")

#: Sample the RUN-LONG contact watch every N basic timesteps. ON by default,
#: because the alternative is a collision assertion that reports 0 whatever
#: happened, and a physical assertion that cannot fail is not evidence (see
#: ``_run_contacts``). ``--run-contacts=0`` turns it off.
#:
#: THE CADENCE IS A COST/BLINDNESS TRADE AND IT IS STATED, NOT HIDDEN. Every
#: sample walks the scene and queries each Solid's contact points, so sampling
#: every basic step costs one full walk per 16 ms of sim. At 8 (128 ms) a
#: robot that is stuck against a wall, resting on an obstacle, or wedged --
#: the cases a navigation task cares about, which all persist for seconds --
#: is caught many times over, while a glancing sub-128 ms brush can be missed.
#: That direction of error is the safe one for an INSTRUMENT (it can miss a
#: hit, never invent one) but it is the unsafe one for a GRADE, so it is
#: reported: ``meta["run_contacts"]["every_n_steps"]`` says what was watched.
RUN_CONTACT_EVERY = 8

#: Distinct (robot, other) pairs kept. A run cannot produce many -- the pairs
#: are deduped -- but a pathological scene must not be able to grow the meta
#: file without bound.
RUN_CONTACT_PAIR_CAP = 256

#: Hard per-robot ceiling on link tracks, whatever ``--links=`` asks for.
#:
#: THE COST BOUND, stated rather than discovered. This controller samples every
#: track once per BASIC TIMESTEP -- 2,188 samples for R2's 35 s at 16 ms -- and
#: a URDF import can hand us dozens of link bodies per robot, so an unbounded
#: walk turns a 6-DOF arm and a 60-body humanoid into the same request with very
#: different costs. Two bounds apply together: the PREDICATE below keeps the set
#: to bodies that actually move, and this ceiling keeps a mis-set task meta from
#: costing a campaign. Truncation is never silent -- it lands in
#: ``meta["links_truncated"]`` and in the per-robot roster entry.
LINK_CAP_CEILING = 64

#: Which named ``--solids=`` get a per-step track.
#:
#: ``dynamic`` is the default and it is the one that keeps every already-frozen
#: task byte-identical: B2's five props, C1's floor and pallets, C2's floor and
#: R3's table and bin are all STATIC, so none of them gains a row, while a cube
#: with a Physics node gains one the moment it is named. It is also the honest
#: default -- a body with no mass model has no motion to record, so a track for
#: it is pure cost. ``all`` is there for a kinematic prop a supervisor drives
#: (which has no mass model but does move); ``none`` restores the pre-2026-08-09
#: bounds-only behaviour exactly.
SOLID_TRACK_MODES = ("dynamic", "all", "none")
DEFAULT_SOLID_TRACKS = "dynamic"

#: How many non-robot bodies the NAME-FREE t=0 scene scan will bound.
#:
#: THE COST BOUND FOR THAT SCAN, stated rather than discovered. Unlike the two
#: track kinds above, this scan costs nothing per step: it runs ONCE, at t=0,
#: and it adds NO CSV column, so a 60 s recording is not one sample per body
#: more expensive for having run it. What it does cost is one
#: ``bounds_for_subtree`` geometry walk per body found, and every supervisor
#: field read is an IPC round-trip -- so a procedurally generated forest could
#: hand us thousands of trunks. Three bounds apply together: the PREDICATE
#: (non-robot Solid-derived bodies only, and a Robot's subtree is never
#: entered -- its links are ``--links=``' business), a DEPTH LIMIT so a
#: malformed scene cannot make the walk run away, and this ceiling on the
#: RESULT. Truncation is never silent: it lands in ``t0_scene["truncated"]``
#: next to how many were found and how many were bounded, and the adapter turns
#: it into a note on the verdict.
SCENE_SCAN_DEFAULT_CAP = 128
SCENE_SCAN_CEILING = 512
SCENE_SCAN_DEPTH_LIMIT = 32

# geometry.py / observe.py are the harness supervisor's own scene-walk helpers.
# Reused rather than reimplemented: geometry.bounds_for_subtree is the same
# world-space AABB an agent gets from GET /scene/tree?bounds=1, so the grader
# and the agent are looking at the same numbers.
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


def _enable_pose_tracking(tracks, sampling_period):
    """Enable one batched pose stream for every trajectory row.

    ``Node.getPosition()`` is a synchronous Supervisor request: one call forces
    one controller/engine flush.  At BuildScale level 100, a 20 s recording at
    32 ms used to make 62,500 such round trips.  Pose tracking moves the exact
    same 4x4 world transforms into the ordinary ``step()`` reply; ``getPose()``
    then reads the same-sim-time cache without another flush.

    Tracking is an optimisation, never an evidence assumption.  A node whose
    binding does not expose it remains readable through ``getPosition()``, and
    that fallback is counted in the returned provenance document.
    """
    doc = {"mode": "tracked_pose_step_batch", "sampling_period_ms": int(sampling_period),
           "requested": len(tracks), "enabled": 0, "fallback": 0,
           "read_fallbacks": 0, "errors": []}
    for tr in tracks:
        tr["_pose_tracking"] = False
        try:
            tr["node"].enablePoseTracking(int(sampling_period))
            tr["_pose_tracking"] = True
            doc["enabled"] += 1
        except Exception as exc:  # noqa: BLE001 - exact fallback is intentional
            doc["fallback"] += 1
            if len(doc["errors"]) < 8:
                doc["errors"].append(repr(exc)[:200])
    if not doc["enabled"]:
        doc["mode"] = "synchronous_get_position"
    elif doc["fallback"]:
        doc["mode"] = "mixed_tracked_pose_and_synchronous_fallback"
    return doc


def _track_position(track, tracking_doc):
    """Return an exact world translation, preferring the tracked pose cache."""
    if track.get("_pose_tracking"):
        try:
            pose = track["node"].getPose()
            # Supervisor pose matrices are serialized row-major. Translation
            # is therefore the fourth column: m(0,3), m(1,3), m(2,3).
            if pose is None or len(pose) < 12:
                raise ValueError("tracked pose did not contain a 4x4 matrix")
            return [pose[3], pose[7], pose[11]]
        except Exception as exc:  # noqa: BLE001 - fall through to exact read
            track["_pose_tracking"] = False
            tracking_doc["read_fallbacks"] += 1
            tracking_doc["fallback"] += 1
            tracking_doc["enabled"] = max(0, tracking_doc["enabled"] - 1)
            tracking_doc["mode"] = (
                "mixed_tracked_pose_and_synchronous_fallback"
                if tracking_doc["enabled"] else "synchronous_get_position")
            if len(tracking_doc["errors"]) < 8:
                tracking_doc["errors"].append(repr(exc)[:200])
    return track["node"].getPosition()


def _enable_robot_contact_tracking(nodes, sampling_period):
    """Batch deep Robot contact sets into step replies; report every fallback."""
    doc = {"mode": "tracked_robot_subtrees_deep",
           "sampling_period_ms": int(sampling_period),
           "requested": len(nodes), "enabled": 0, "fallback": 0,
           "errors": []}
    for node in nodes:
        try:
            node.enableContactPointsTracking(int(sampling_period), True)
            doc["enabled"] += 1
        except Exception as exc:  # noqa: BLE001 - query remains exact, just slower
            doc["fallback"] += 1
            if len(doc["errors"]) < 8:
                doc["errors"].append(repr(exc)[:200])
    if not doc["enabled"]:
        doc["mode"] = "synchronous_robot_subtrees_deep"
    elif doc["fallback"]:
        doc["mode"] = "mixed_tracked_and_synchronous_robot_subtrees_deep"
    return doc


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


def _basetype(node):
    """``getBaseTypeName()`` -- the BASE node a PROTO instance derives from.

    Needed for links and not for the robot walk: a hand-authored arm's links are
    plain ``Solid`` nodes, but an imported or PROTO-wrapped one's are
    Solid-DERIVED instances whose ``getTypeName()`` is the PROTO's name. Reading
    the base type is what makes the link predicate work on both.
    """
    try:
        return node.getBaseTypeName()
    except Exception:
        return ""


def _is_robot(node):
    """Robot node or a PROTO instance whose declared base is Robot."""
    return (_typename(node) in ROBOT_TYPENAMES
            or _basetype(node) in ROBOT_TYPENAMES)


def _is_solid(node):
    return _basetype(node) == "Solid" or _typename(node) == "Solid"


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
    """The identifier ``observe._node_def_or_id`` gives this node.

    Byte-for-byte the same convention (DEF if set, else ``"#<id>"``, else
    ``"#?"``), because it is the key space contacts and the robot-subtree index
    are expressed in. Reimplemented rather than imported so the recorder still
    works when the observe helper failed to load.
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
    """True when the node carries a Physics node (i.e. is a dynamic body)."""
    f = node.getField("physics") if node is not None else None
    if f is None:
        return None
    try:
        return f.getSFNode() is not None
    except Exception:
        return None


def _link_bodies(robot, depth_limit=64):
    """The moving BODIES inside one Robot, depth-first from the base outwards.

    Returns ``[(node, via_endpoint), ...]``. A Solid-derived descendant counts
    as a link when EITHER:

      * it is the ``endPoint`` of a joint -- the articulated chain's own links.
        This clause is what makes the predicate work on a KINEMATIC arm, one
        authored without ``Physics`` nodes: its motors still drive it and its
        tip still reaches the targets, so requiring a mass model would have
        reported "no end effector" for an arm that demonstrably did the task;
      * or it carries a ``Physics`` node -- a mass model. This clause catches a
        tool or gripper body rigidly parented to the last link rather than hung
        off a joint, which is exactly where an end effector often lives.

    Everything else is deliberately NOT a link: ``Pose`` / ``Group`` / shape
    wrappers carry no pose of their own worth sampling, and a nested ``Robot``
    is tracked in its own right as a robot row -- recursing into it would give
    one body two tracks and make the roster-index invariant a lie.

    The walk is bounded by ``depth_limit`` (a malformed scene cannot make it
    run away) and its RESULT is bounded by the caller's ``--links=`` cap; see
    ``LINK_CAP_CEILING`` for why both bounds exist.
    """
    out = []
    seen = set()

    def visit(node, via_endpoint, depth):
        if node is None or depth > depth_limit:
            return
        if _is_robot(node):
            return
        key = _nid(node)
        if key != -1:
            if key in seen:
                return
            seen.add(key)
        if _is_solid(node) and (via_endpoint or _has_physics(node) is True):
            out.append((node, via_endpoint))
        for c in _children(node):
            visit(c, False, depth + 1)
        ep = _endpoint(node)
        if ep is not None and ep is not node:
            visit(ep, True, depth + 1)

    for c in _children(robot):
        visit(c, False, 1)
    return out


def _resolve_robot(ident, subtree_index, robot_ids):
    """Which Robot a contact participant belongs to, or None.

    ``build_robot_subtree_index`` deliberately SKIPS the entry where a solid IS
    its own Robot (``sid == robot_id``), so a contact on a robot's own base
    link -- the whole chassis of a Husky, i.e. the single likeliest place for
    two robots to touch -- arrives as the robot's own id and the index lookup
    misses. Before this fallback that pair was silently dropped while
    ``distinct_named`` still counted it, so A1.3's robot-robot clause reported
    itself NON-vacuous (the witness was there) and could still never fire: the
    worst combination, a check that says it is watching and is not.
    """
    hit = subtree_index.get(ident)
    if hit:
        return hit, False
    if ident in robot_ids:
        return ident, True
    return None, False


def _robot_robot_contacts(sv, subtree_index, robot_ids=(), robot_nodes=None):
    """Contacts whose BOTH participants resolve to a Robot subtree.

    Returns (robot_robot_pairs, witness) where `witness` records how much
    contact evidence EXISTED, not just how much survived the filter:

        total_observed   contacts the engine reported at all
        distinct_named   contacts naming two DIFFERENT bodies
        supported        whether the contact query worked at all

    Without those counters an empty result is indistinguishable from a broken
    contact pipeline -- which is exactly how this check hid for weeks: the
    engine reported ContactPoint.node_id as the QUERIED solid's own id, so
    every pair was (id, id) and `ra != rb` could never be true. The grader
    uses the witness to mark the assertion "vacuous: witness absent" rather
    than quietly passing it.
    """
    witness = {"total_observed": None, "distinct_named": None,
               "self_resolved": None, "supported": False, "error": None}
    if _observe is None:
        witness["error"] = "observe helper unavailable"
        return [], witness
    try:
        pairs = (_observe.list_robot_contacts(robot_nodes)
                 if robot_nodes is not None
                 else _observe.list_contacts(sv))
    except Exception as exc:  # noqa: BLE001
        witness["error"] = repr(exc)[:200]
        return [], witness
    witness["supported"] = True
    witness["total_observed"] = len(pairs)
    named = 0
    self_hits = 0
    out = []
    for c in pairs:
        a_def, b_def = c.get("a_def"), c.get("b_def")
        if a_def and b_def and a_def != b_def:
            named += 1
        ra, a_self = _resolve_robot(a_def, subtree_index, robot_ids)
        rb, b_self = _resolve_robot(b_def, subtree_index, robot_ids)
        self_hits += int(a_self) + int(b_self)
        if ra and rb and ra != rb:
            out.append({"a": a_def, "b": b_def,
                        "a_robot": ra, "b_robot": rb,
                        "a_is_robot_body": a_self, "b_is_robot_body": b_self,
                        "point": c.get("point")})
    witness["distinct_named"] = named
    witness["self_resolved"] = self_hits
    return out, witness


def _contact_identity_maps(sv, robot_nodes, subtree_index):
    """``(robot_of, name_of)`` in the CONTACT key space.

    ``robot_of[body_id]`` is the ``name`` of the Robot a contact participant
    belongs to (its own base link included -- ``build_robot_subtree_index``
    skips that self entry, and a chassis is the likeliest thing to hit a
    wall). ``name_of[body_id]`` is any other Solid's own ``name`` field.

    Names, not DEFs, because a grader's assertion is written against the same
    names its t=0 inventory carries, and an agent is free to leave DEFs off
    entirely.
    """
    robot_of, name_of = {}, {}
    by_id = {}
    for node in robot_nodes:
        ident = _obs_id(node)
        nm = _sf_string(node, "name")
        by_id[ident] = nm
        robot_of[ident] = nm
    for solid_id, robot_id in (subtree_index or {}).items():
        nm = by_id.get(robot_id)
        if nm is not None:
            robot_of[solid_id] = nm
    solids = []
    _walk(sv.getRoot(), _is_solid, solids)
    for s in solids:
        ident = _obs_id(s)
        if ident not in robot_of:
            name_of[ident] = _sf_string(s, "name")
    return robot_of, name_of


def _run_contacts(sv, robot_of, name_of, robot_nodes=None):
    """Every contact naming a ROBOT and something else, this step.

    Returns ``(records, witness)``. A record is
    ``{"a": <robot name>, "b": <other body's name>, "b_robot": bool, ...}``
    -- NAMES, because that is the space a sim-neutral grader's assertion is
    written in (``r1_core`` decides "was this a hit" by asking whether the
    other participant is one of the obstacles it matched by GEOMETRY, and it
    only has their names).

    ⚠ **WHY THIS EXISTS, and what it is NOT.** The phase-A watch next door
    reports ROBOT-ROBOT pairs over the first ``--contact-steps`` basic steps at
    t=0. That is the right instrument for A1.3 (are ten robots spawned
    interpenetrating?) and it is the WRONG one -- structurally, not by
    tuning -- for "did this robot hit an obstacle or a wall during a 60 s
    drive". Two reasons, and either alone is fatal:

      1. ``_robot_robot_contacts`` DROPS every pair whose other side is not a
         Robot subtree, so a robot-vs-box contact cannot even be represented;
      2. R1's own task meta asks for ``contact_steps: 0``, so the window is one
         sample at t=0, before the robot has moved.

    Measured consequence, on a recorded run rather than in theory: the
    ``r1_settled_omnisim`` cell's rover finished at (9.22, 17.36) -- OUTSIDE a
    walled 10 x 10 arena -- and R1.5 "nothing was hit" reported
    ``robot-obstacle/wall contacts: 0`` and PASSED. An assertion that cannot
    fail is not evidence, which is the exact defect C2 shipped with.

    This watch keeps that channel untouched and adds a second one: sampled
    across the RECORDING window, and reporting the robot's partner whatever it
    is. The pairs are deduped by (a, b) with first/last step and a count, so a
    robot resting against a wall for 3,000 steps is one record and not three
    thousand.
    """
    witness = {"supported": False, "total_observed": 0, "distinct_named": 0,
               "error": None}
    if _observe is None:
        witness["error"] = "observe helper unavailable"
        return [], witness
    try:
        pairs = (_observe.list_robot_contacts(robot_nodes)
                 if robot_nodes is not None
                 else _observe.list_contacts(sv))
    except Exception as exc:  # noqa: BLE001
        witness["error"] = repr(exc)[:200]
        return [], witness
    witness["supported"] = True
    witness["total_observed"] = len(pairs)
    out = []
    for c in pairs:
        a_def, b_def = c.get("a_def"), c.get("b_def")
        if not a_def or not b_def or a_def == b_def:
            # An UNPAIRED contact names one side only: the engine reported it,
            # but the partner is not a body a supervisor can query (a PROTO
            # arena's internal walls are the standard case). Recorded as such
            # -- never silently dropped and never guessed at -- so a world that
            # is contact-blind for that reason is visibly blind.
            witness["distinct_named"] += 0
            other = None
        else:
            witness["distinct_named"] += 1
            other = b_def
        a_robot = robot_of.get(a_def)
        b_robot = robot_of.get(b_def) if other else None
        if a_robot is None and b_robot is None:
            continue                      # no robot in this pair: not our
            #                               business, and not R1.5's either
        if a_robot is None:               # put the robot on side A
            a_def, b_def = b_def, a_def
            a_robot, b_robot = b_robot, a_robot
        out.append({
            "a": a_robot,
            "b": (b_robot if b_robot is not None
                  else (name_of.get(b_def) if b_def else None)),
            "a_body": a_def, "b_body": b_def,
            "a_robot": True, "b_robot": b_robot is not None,
            "paired": bool(c.get("paired")),
            "point": c.get("point"),
        })
    return out, witness


def _find_named_solids(sv, wanted):
    """The ``--solids=`` nodes, in scene order. ``[]`` when nothing was asked.

    Split out of the old ``_scan_solids`` so the SAME node list feeds both the
    t=0 bounds scan and the per-step track builder -- one walk, one identity per
    solid, so a grader that looks the cube up in the t=0 inventory and then in
    the pose series is guaranteed to get the same body.
    """
    if not wanted:
        return []
    solids = []
    _walk(sv.getRoot(), lambda n: _typename(n) == "Solid", solids)
    out = []
    for node in solids:
        nm = _sf_string(node, "name")
        if nm not in wanted:
            continue
        out.append({"node": node, "name": nm, "def": _def(node),
                    "id": _nid(node), "has_physics": _has_physics(node)})
    return out


def _scene_bodies(root, claimed_ids, cap, depth_limit=SCENE_SCAN_DEPTH_LIMIT):
    """Every non-robot Solid in the scene, found WITHOUT a name list.

    Returns ``(entries, found_total)`` where ``entries`` is capped at ``cap``
    and ``found_total`` is how many the predicate actually matched, so the
    caller can publish the truncation instead of hiding it.

    The predicate, and what each clause is for:

    * **Solid-derived** (``getBaseTypeName()``, so a PROTO-wrapped or imported
      box counts too). A ``Pose`` / ``Group`` / ``Shape`` wrapper carries no
      pose of its own worth bounding and is walked THROUGH, not recorded.
    * **not inside a Robot.** A Robot's subtree is never entered: its links are
      ``--links=``' business, they are tracked and bounded there with an honest
      ``robot_class=False`` and a ``member_of``, and offering them twice would
      give one body two identities. It also means the recorder's own Robot and
      the harness supervisor are excluded for free.
    * **not already claimed.** A body the caller has already bounded through
      ``--solids=`` keeps that entry and does not get a second one; two entries
      for one body would not break the geometric matcher (each spec entry is
      matched at most once) but it would give a reader two answers to one
      question.

    A nested Solid -- a part inside a crate -- IS recorded, with ``nested_in``
    naming the enclosing body, because the enclosing body's own bounds are
    subtree-inclusive and a grader asking "what is at this position with this
    footprint" must be able to see both. The walk continues INTO a recorded
    Solid for exactly that reason.
    """
    out = []
    found = [0]
    seen = set()

    def visit(node, parent_id, depth):
        if node is None or depth > depth_limit:
            return
        if _is_robot(node):
            return                      # a robot's insides are --links=' job
        nid = _nid(node)
        if nid != -1:
            if nid in seen:
                return
            seen.add(nid)
        child_parent = parent_id
        if _is_solid(node):
            child_parent = _obs_id(node)
            if nid not in claimed_ids:
                found[0] += 1
                if len(out) < cap:
                    out.append({"node": node, "body_id": _obs_id(node),
                                "name": _sf_string(node, "name"),
                                "def": _def(node), "id": nid,
                                "type": _typename(node),
                                "base_type": _basetype(node),
                                "has_physics": _has_physics(node),
                                "nested_in": parent_id, "depth": depth})
        for c in _children(node):
            visit(c, child_parent, depth + 1)
        ep = _endpoint(node)
        if ep is not None and ep is not node:
            visit(ep, child_parent, depth + 1)

    visit(root, None, 0)
    return out, found[0]


def _scan_scene(phase_a, sv, claimed_ids, cap):
    """The ``t0_scene`` block: name-free bodies, each with world bounds.

    Written from BOTH phase-A exits, like ``_scan_solids`` -- a robot-free
    world is exactly where a name-free inventory matters most.

    ``bodies: []`` with ``found: 0`` is a measurement ("the scene has no
    non-robot Solid"); ``supported: false`` is the absence of one (the bounds
    helper did not load, so nothing could be measured). The two must never
    read the same, which is why both keys are here.

    ⚠ **A PROTO instance's INTERNALS are not reached, and its own box may be
    absent.** Measured on ``B1``'s ``six_huskies.wbt``: its ``RectangleArena``
    is Solid-derived, so the walk finds the arena itself -- and then
    ``geometry.bounds_for_subtree`` returns ``None`` for it, because a PROTO
    instance's supervisor fields are its PROTO INTERFACE (``floorSize``,
    ``wallHeight``) and not a ``children`` list, so neither the bounds helper
    nor this walk can see the floor and four walls inside. That is a
    pre-existing property of ``bounds_for_subtree`` -- it is exactly what an
    agent gets from ``GET /scene/tree?bounds=1`` -- and it is deliberately not
    worked around here: this scan reuses that helper precisely so the grader
    and the agent are looking at the same numbers. The body arrives with
    ``bounds: null`` and the adapter says how many were unbounded, so it is
    visibly unmeasured rather than invisibly wrong. Note the DIRECTION: the
    upstream-Webots recorder resolves PROTO internals (its field lookup falls
    back to ``getBaseNodeField``), so on that one body class the control arm
    currently measures MORE than we do.
    """
    doc = {"supported": _geometry is not None,
           "cap": int(cap), "found": 0, "bounded": 0,
           "truncated": False, "bounds_error": _GEOM_ERR,
           "source": ("name-free t=0 scan: every non-robot Solid-derived body "
                      "outside a Robot subtree, bounded with "
                      "geometry.bounds_for_subtree -- the same world-space "
                      "AABB an agent gets from GET /scene/tree?bounds=1"),
           "bodies": []}
    phase_a["t0_scene"] = doc
    if cap <= 0:
        doc["supported"] = False
        doc["source"] = "disabled: --scan-solids=0"
        doc["bounds_error"] = ("the scan was turned OFF for this run "
                               "(--scan-solids=0), so no non-robot body was "
                               "offered -- this is a configuration choice, "
                               "not a property of the scene")
        return
    entries, found = _scene_bodies(sv.getRoot(), claimed_ids, cap)
    doc["found"] = found
    doc["truncated"] = found > len(entries)
    for e in entries:
        node = e.pop("node")
        try:
            e["position"] = [float(x) for x in node.getPosition()]
        except Exception:  # noqa: BLE001
            e["position"] = None
        e["bounds"] = None
        if _geometry is not None:
            try:
                e["bounds"] = _geometry.bounds_for_subtree(node)
            except Exception as exc:  # noqa: BLE001
                e["bounds_error"] = repr(exc)
        if (e["bounds"] or {}).get("bbox_min"):
            doc["bounded"] += 1
        doc["bodies"].append(e)


def _scan_solids(phase_a, solid_entries, wanted):
    """The ``--solids=`` t=0 scan: named non-robot Solids, with world bounds.

    Run from BOTH exits: the tracked-robots path and the no-robots early exit.
    It used to live only in the former, so a robot-free world (B2's is exactly
    that -- five scenery props, no Robot node) silently dropped the requested
    bounds and the grader read an empty t=0 inventory (measured 2026-08-01:
    every live B2 cell came back INVALID with 'red_cylinder not found with
    world-space bounds').

    Each entry now also says whether it got a per-step track and, when it did
    not, WHY -- so "the cube has no trajectory" is a readable finding rather
    than an absence a grader has to guess at.
    """
    if not wanted:
        return
    phase_a["t0_solids"] = []
    for e in solid_entries:
        node = e["node"]
        entry = {"name": e["name"], "def": e["def"], "id": e["id"],
                 "bounds": None, "has_physics": e["has_physics"],
                 "track_index": e.get("track_index"),
                 "tracked": e.get("track_index") is not None,
                 "track_reason": e.get("track_reason")}
        try:
            entry["position"] = [float(x) for x in node.getPosition()]
        except Exception:
            entry["position"] = None
        if _geometry is not None:
            try:
                entry["bounds"] = _geometry.bounds_for_subtree(node)
            except Exception as exc:  # noqa: BLE001
                entry["bounds_error"] = repr(exc)
        phase_a["t0_solids"].append(entry)


def _scan_links(phase_a, link_entries):
    """t=0 provenance for every LINK track: identity, pose and world bounds.

    The pose series alone would leave a link row anonymous -- a number with no
    account of which body it is or which robot it belongs to. This is where
    ``parent_body_id`` and the endPoint/mass-model reason are recorded, and it
    is what lets the adapter put a link into the neutral t=0 inventory with an
    honest ``robot_class=False``.
    """
    if not link_entries:
        return
    phase_a["t0_links"] = []
    for e in link_entries:
        node = e["node"]
        entry = {k: v for k, v in e.items() if k != "node"}
        entry["bounds"] = None
        try:
            entry["position"] = [float(x) for x in node.getPosition()]
        except Exception:
            entry["position"] = None
        if _geometry is not None:
            try:
                entry["bounds"] = _geometry.bounds_for_subtree(node)
            except Exception as exc:  # noqa: BLE001
                entry["bounds_error"] = repr(exc)
        phase_a["t0_links"].append(entry)


def main():
    args = parse_args(sys.argv[1:])
    sv = Supervisor()

    out_path = args.get("out") or os.environ.get(
        "AGENTBENCH_OUT", "agentbench.csv")
    duration = float(args.get("duration", "30.0"))
    settle = float(args.get("settle", "1.0"))
    want_phase_a = args.get("phase-a", "0") not in ("0", "", "false", "False")
    contact_steps = int(args.get("contact-steps", "10"))
    try:
        run_contact_every = max(0, int(args.get("run-contacts",
                                                str(RUN_CONTACT_EVERY))))
    except ValueError:
        run_contact_every = RUN_CONTACT_EVERY
    robot_contacts_only = args.get("robot-contacts-only", "0") not in (
        "0", "", "false", "False")
    contact_witness_defs = [s.strip() for s in
                            args.get("contact-witness-defs", "").split(",")
                            if s.strip()]
    excluded = set(ALWAYS_EXCLUDE)
    for name in args.get("exclude", "").split(","):
        if name.strip():
            excluded.add(name.strip())
    wanted_solids = [s.strip() for s in args.get("solids", "").split(",")
                     if s.strip()]
    solid_tracks = args.get("solid-tracks", DEFAULT_SOLID_TRACKS)
    if solid_tracks not in SOLID_TRACK_MODES:
        solid_tracks = DEFAULT_SOLID_TRACKS
    try:
        link_cap = max(0, min(int(args.get("links", "0")), LINK_CAP_CEILING))
    except ValueError:
        link_cap = 0
    try:
        scene_cap = max(0, min(int(args.get("scan-solids",
                                            str(SCENE_SCAN_DEFAULT_CAP))),
                               SCENE_SCAN_CEILING))
    except ValueError:
        scene_cap = SCENE_SCAN_DEFAULT_CAP

    dt_ms = sv.getBasicTimeStep()
    step_ms = int(round(dt_ms))
    n_record = int(round(duration * 1000.0 / dt_ms))
    n_settle_total = int(round(settle * 1000.0 / dt_ms))
    # A NEGATIVE --contact-steps means "the whole run" on the upstream-Webots
    # recorder, whose t=0 window is the only contact channel it has. Here it
    # is not needed and must not be a crash: this arm answers "did it hit
    # anything while it drove" from the run-long watch below, which is on by
    # default and independent of this window, so a negative value clamps to
    # the t=0 sample and the two arms can share one task meta.
    n_contact = max(0, contact_steps) if want_phase_a else 0

    # --- enumerate robots by node type + name (SPEC 4.5.1) -----------------
    robot_nodes = []
    _walk(sv.getRoot(), _is_robot, robot_nodes)
    tracked = []
    for node in robot_nodes:
        name = _sf_string(node, "name")
        if name in excluded:
            continue
        joints = []
        _walk(node, lambda n: _typename(n) in JOINT_TYPENAMES, joints)
        tracked.append({
            "node": node,
            "name": name,
            "def": _def(node),
            "id": _nid(node),
            "controller": _sf_string(node, "controller"),
            "supervisor": _sf_bool(node, "supervisor"),
            "has_physics": _has_physics(node),
            "num_joints": len(joints),
            "type": _typename(node),
            "base_type": _basetype(node),
        })

    # --- build the CSV column map ------------------------------------------
    #
    # ORDER IS THE BACKWARD-COMPATIBILITY CONTRACT: every robot row keeps its
    # original index and its r<i> column names, so a grader that resolves a
    # body through the roster and indexes the pose series with that position
    # (c2_core, r1_core do exactly this) is unaffected by anything below it.
    tracks = []
    for i, r in enumerate(tracked):
        tracks.append({"index": len(tracks), "kind": "robot",
                       "column": "r%d" % i,
                       "node": r["node"], "body_id": _obs_id(r["node"]),
                       "name": r["name"], "def": r["def"], "id": r["id"],
                       "parent_body_id": None, "link_index": None,
                       "has_physics": r["has_physics"],
                       "type": r["type"], "base_type": r["base_type"]})

    link_entries = []
    links_truncated = False
    if link_cap:
        for r in tracked:
            parent_id = _obs_id(r["node"])
            found = _link_bodies(r["node"])
            kept = found[:link_cap]
            if len(found) > len(kept):
                links_truncated = True
            r["num_link_tracks"] = len(kept)
            r["num_link_bodies_found"] = len(found)
            r["links_truncated"] = len(found) > len(kept)
            for k, (node, via_endpoint) in enumerate(kept):
                # A STRUCTURAL id, never a name. R1 shipped a grader keyed on
                # the published OBSTACLE_n names and the first real agent called
                # them "crate A"; nothing in R2's prompt suggests a name for a
                # link either, so the identity a grader gets is "link k of robot
                # X" -- derived from the scene's own topology, stable across
                # runs of the same world, and impossible for an agent's naming
                # choice to break.
                entry = {"index": None, "kind": "link",
                         "column": "l%d" % len(link_entries),
                         "node": node,
                         "body_id": "%s/link%d" % (parent_id, k),
                         "name": _sf_string(node, "name"),
                         "def": _def(node), "id": _nid(node),
                         "parent_body_id": parent_id,
                         "parent_name": r["name"], "link_index": k,
                         "has_physics": _has_physics(node),
                         "joint_endpoint": bool(via_endpoint),
                         "type": _typename(node),
                         "base_type": _basetype(node)}
                link_entries.append(entry)
        for e in link_entries:
            e["index"] = len(tracks)
            tracks.append(e)

    solid_entries = _find_named_solids(sv, wanted_solids)
    # Node ids already spoken for, so a named body that is ALSO a sampled link
    # of some robot does not get a second row. Two rows for one body would not
    # break the index invariant (the ids differ), but it would double the
    # per-step cost and give a grader two answers to one question.
    claimed = set()
    for tr in tracks:
        nid = tr.get("id")
        if nid is not None and nid != -1:
            claimed.add(nid)
    n_solid_tracks = 0
    for e in solid_entries:
        if solid_tracks == "none":
            e["track_reason"] = ("--solid-tracks=none: bounds only, the "
                                 "pre-2026-08-09 behaviour")
        elif e["id"] in claimed:
            e["track_reason"] = "already tracked as a robot or a link row"
        elif solid_tracks == "all" or e["has_physics"] is True:
            e["track_reason"] = ("carries a mass model"
                                 if e["has_physics"] is True
                                 else "--solid-tracks=all")
            e["index"] = len(tracks)
            e["track_index"] = len(tracks)
            e["column"] = "s%d" % n_solid_tracks
            n_solid_tracks += 1
            if e["id"] is not None and e["id"] != -1:
                claimed.add(e["id"])
            tracks.append({"index": e["index"], "kind": "solid",
                           "column": e["column"], "node": e["node"],
                           "body_id": _obs_id(e["node"]), "name": e["name"],
                           "def": e["def"], "id": e["id"],
                           "parent_body_id": None, "link_index": None,
                           "has_physics": e["has_physics"], "type": "Solid"})
        else:
            e["track_reason"] = (
                "no mass model (physics is NULL), so it cannot move under the "
                "solver and a per-step track would record a constant; pass "
                "--solid-tracks=all if a supervisor drives it kinematically")
        e.setdefault("track_index", None)

    # Ids the name-free scan must not offer a SECOND time: every track row
    # (robots, links, named-Solid tracks) plus every named Solid that got a
    # bounds entry without a track. The named list keeps its own entry -- it
    # carries the track bookkeeping the scan does not.
    scene_claimed = set(claimed)
    for e in solid_entries:
        if e.get("id") is not None and e["id"] != -1:
            scene_claimed.add(e["id"])

    roster = [{k: v for k, v in r.items() if k != "node"} for r in tracked]
    track_map = [{k: v for k, v in tr.items() if k != "node"}
                 for tr in tracks]
    meta = {
        "out": os.path.abspath(out_path),
        "world": sv.getWorldPath(),
        "dt_ms": dt_ms,
        "settle_s": settle,
        "duration_s": duration,
        "robots": roster,
        "n_robots": len(tracked),
        "tracks": track_map,
        "n_tracks": len(tracks),
        "n_link_tracks": len(link_entries),
        "n_solid_tracks": n_solid_tracks,
        "link_cap": link_cap,
        "link_cap_ceiling": LINK_CAP_CEILING,
        "links_truncated": links_truncated,
        "solid_tracks_mode": solid_tracks,
        "solids_requested": list(wanted_solids),
        "scene_scan_cap": scene_cap,
        "rows": 0,
        "recorded_s": 0.0,
        "run_contacts": None,
        "pose_sampling": None,
        "contact_sampling": None,
        "complete": False,
        "quit_called": False,
        "step_returned_minus1": False,
    }

    out_dir = os.path.dirname(os.path.abspath(out_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    def write_meta():
        with open(out_path + ".meta.json", "w", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2)

    write_meta()  # exists even if the run dies mid-way ("complete": false)

    def finish(complete):
        meta["complete"] = complete
        meta["quit_called"] = True
        write_meta()
        sv.simulationQuit(0)
        sv.step(step_ms)

    if not tracked:
        # No robots does not mean no phase A, and since 2026-08-09 it does not
        # necessarily mean no recording either: the --solids scan is about
        # NON-robot bodies and must still run (B2's world is robot-free
        # scenery; dropping the scan starved its grader of every world-space
        # bound), and a DYNAMIC named solid in such a world now earns a track.
        if want_phase_a:
            phase_a = {"t0_robots": [], "robot_robot_contacts": [],
                       "contact_steps": 0, "bounds_error": _GEOM_ERR,
                       "contact_witness": {
                           "total_observed": None, "distinct_named": None,
                           "supported": False, "steps_sampled": 0,
                           "error": "phase A contact scan skipped: no robots"}}
            _scan_solids(phase_a, solid_entries, wanted_solids)
            _scan_links(phase_a, link_entries)
            _scan_scene(phase_a, sv, scene_claimed, scene_cap)
            with open(out_path + ".phase_a.json", "w", encoding="utf-8") as fh:
                json.dump(phase_a, fh, indent=2)
        if not tracks:
            print("[agentbench_recorder] no robots to track; quitting",
                  flush=True)
            with open(out_path, "w", encoding="utf-8", newline="") as fh:
                fh.write("t\n")
            finish(True)
            return
        print("[agentbench_recorder] no robots, but %d non-robot track(s) to "
              "record" % len(tracks), flush=True)

    else:
        print("[agentbench_recorder] tracking %d robot(s): %s"
              % (len(tracked), ", ".join(r["name"] or r["def"] or "?"
                                         for r in tracked)), flush=True)
    if len(tracks) > len(tracked):
        print("[agentbench_recorder] plus %d link track(s) and %d solid "
              "track(s)" % (len(link_entries), n_solid_tracks), flush=True)

    broke = False
    #: Basic timesteps already consumed by phase A, so the settle window is the
    #: length the task asked for however phase A behaved. Without this a
    #: robot-free world (which runs no contact scan) would settle
    #: ``contact_steps`` steps LONGER than a world with robots.
    steps_done = 0

    # Phase A asks on consecutive timesteps, so a tracked contact stream at the
    # basic timestep is exact for that window.  It is re-aligned to the run-long
    # cadence after settle below.  No tracking is enabled for the all-Solids
    # compatibility path: only BuildScale opts into the bounded Robot query.
    contact_robot_nodes = ([r["node"] for r in tracked]
                           if robot_contacts_only else None)
    contact_witness_found = []
    if contact_robot_nodes is not None:
        for def_name in contact_witness_defs:
            try:
                node = sv.getFromDef(def_name)
            except Exception:
                node = None
            if node is not None:
                contact_robot_nodes.append(node)
                contact_witness_found.append(def_name)
    meta["contact_witness_defs"] = {
        "requested": list(contact_witness_defs),
        "found": contact_witness_found,
        "missing": [d for d in contact_witness_defs
                    if d not in contact_witness_found],
    }
    if contact_robot_nodes and want_phase_a:
        meta["contact_sampling"] = _enable_robot_contact_tracking(
            contact_robot_nodes, step_ms)
        write_meta()

    # --- Phase A: the t=0 structure pass, world frozen ---------------------
    if want_phase_a and tracked:
        phase_a = {
            "contact_steps": n_contact,
            "dt_ms": dt_ms,
            "bounds_error": _GEOM_ERR,
            "observe_error": _OBS_ERR,
            "t0_robots": [],
            "robot_robot_contacts": [],
        }
        for i, r in enumerate(tracked):
            entry = {k: v for k, v in r.items() if k != "node"}
            entry["index"] = i
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
            phase_a["t0_robots"].append(entry)

        # Named non-robot Solids the grader asked for (e.g. C2's floor slab),
        # so "where is the surface?" is measured rather than assumed -- and,
        # next to them, the t=0 identity of every LINK track, so a link row in
        # the CSV is an accountable body rather than an anonymous number.
        _scan_solids(phase_a, solid_entries, wanted_solids)
        _scan_links(phase_a, link_entries)
        # ...and every OTHER non-robot body, found without a name list, so a
        # grader that matches by geometry has candidates to match against
        # whatever the agent chose to call them.
        _scan_scene(phase_a, sv, scene_claimed, scene_cap)

        subtree_index = {}
        if _observe is not None:
            try:
                subtree_index = _observe.build_robot_subtree_index(sv)
            except Exception as exc:  # noqa: BLE001
                phase_a["observe_error"] = repr(exc)
        # Every Robot node's own identifier, in the contact key space. The
        # subtree index cannot contain these (it skips the self entry), so a
        # contact on a robot's base link is only resolvable through this set.
        robot_ids = set()
        for node in robot_nodes:
            ident = _obs_id(node)
            if ident:
                robot_ids.add(ident)
        phase_a["robot_ids"] = sorted(robot_ids)

        # Accumulate the witness across every sampled step: the grader needs
        # to know whether contact evidence EXISTED, not only whether any
        # robot-robot pair survived the filter (see _robot_robot_contacts).
        cw = {"total_observed": 0, "distinct_named": 0, "self_resolved": 0,
              "supported": False, "steps_sampled": 0, "error": None}

        def _accumulate(w):
            cw["steps_sampled"] += 1
            if w.get("supported"):
                cw["supported"] = True
            for key in ("total_observed", "distinct_named", "self_resolved"):
                v = w.get(key)
                if v is not None:
                    cw[key] += v
            if w.get("error") and not cw["error"]:
                cw["error"] = w["error"]

        for k in range(n_contact):
            pairs, w = _robot_robot_contacts(
                sv, subtree_index, robot_ids, contact_robot_nodes)
            _accumulate(w)
            for c in pairs:
                c["step"] = k
                phase_a["robot_robot_contacts"].append(c)
            if sv.step(step_ms) == -1:
                meta["step_returned_minus1"] = True
                broke = True
                break
            steps_done += 1
        if not broke:
            pairs, w = _robot_robot_contacts(
                sv, subtree_index, robot_ids, contact_robot_nodes)
            _accumulate(w)
            for c in pairs:
                c["step"] = n_contact
                phase_a["robot_robot_contacts"].append(c)
        if not cw["supported"]:
            cw["total_observed"] = None
            cw["distinct_named"] = None
            cw["self_resolved"] = None
        phase_a["contact_witness"] = cw

        with open(out_path + ".phase_a.json", "w", encoding="utf-8") as fh:
            json.dump(phase_a, fh, indent=2)
        print("[agentbench_recorder] phase A: %d robots, %d robot-robot "
              "contacts in the first %d steps (witness: %s observed, %s "
              "naming two bodies, supported=%s)"
              % (len(phase_a["t0_robots"]),
                 len(phase_a["robot_robot_contacts"]), n_contact,
                 phase_a["contact_witness"]["total_observed"],
                 phase_a["contact_witness"]["distinct_named"],
                 phase_a["contact_witness"]["supported"]), flush=True)
        _sc = phase_a.get("t0_scene") or {}
        print("[agentbench_recorder] phase A: name-free scene scan found %s "
              "non-robot bod(ies), bounded %s (cap %s%s)"
              % (_sc.get("found"), _sc.get("bounded"), _sc.get("cap"),
                 ", TRUNCATED" if _sc.get("truncated") else ""), flush=True)

    # --- settle -------------------------------------------------------------
    if not broke:
        for _ in range(max(0, n_settle_total - steps_done)):
            if sv.step(step_ms) == -1:
                meta["step_returned_minus1"] = True
                broke = True
                break

    # Align tracked state to t=0 of the RECORDED window. Enabling tracking
    # performs one setup flush per node and seeds the cache at the current sim
    # time; every sample after this arrives in the normal step response.
    if not broke:
        meta["pose_sampling"] = _enable_pose_tracking(tracks, step_ms)
        if contact_robot_nodes and run_contact_every:
            meta["contact_sampling"] = _enable_robot_contact_tracking(
                contact_robot_nodes, run_contact_every * step_ms)
        write_meta()

    # --- record -------------------------------------------------------------
    header = ["t"]
    for tr in tracks:
        c = tr["column"]
        header += ["%s_x" % c, "%s_y" % c, "%s_z" % c]

    # --- the RUN-LONG contact watch (see _run_contacts) --------------------
    #
    # Set up here rather than in phase A because it watches the RECORDING
    # window: the question it answers ("did this robot hit anything while it
    # drove") is not a question about t=0, and the t=0 channel next door stays
    # exactly as it was so every already-frozen A1 measurement is untouched.
    rc_doc = {"supported": False, "every_n_steps": run_contact_every,
              "query_mode": ((meta.get("contact_sampling") or {}).get("mode")
                             if robot_contacts_only else "all_solids_shallow"),
              "steps_sampled": 0, "total_observed": 0, "distinct_named": 0,
              "unpaired": 0, "pairs": [], "truncated": False, "error": None,
              "source": ("contacts sampled across the RECORDING window and "
                         "resolved to body NAMES: the robot's own name for a "
                         "participant inside a Robot subtree, the Solid's "
                         "name field otherwise")}
    rc_seen = {}
    rc_robot_of, rc_name_of = {}, {}
    if run_contact_every and tracked:
        rc_index = {}
        if _observe is not None:
            try:
                rc_index = _observe.build_robot_subtree_index(sv)
            except Exception as exc:  # noqa: BLE001
                rc_doc["error"] = repr(exc)[:200]
        rc_robot_of, rc_name_of = _contact_identity_maps(sv, robot_nodes,
                                                         rc_index)
    elif not run_contact_every:
        rc_doc["error"] = ("the run-long contact watch was turned OFF for "
                           "this run (--run-contacts=0), so an empty pair "
                           "list is a configuration choice and NOT evidence "
                           "that nothing was hit")

    def watch_contacts(step_index):
        if not run_contact_every or not tracked:
            return
        if step_index % run_contact_every:
            return
        recs, w = _run_contacts(
            sv, rc_robot_of, rc_name_of,
            [r["node"] for r in tracked] if robot_contacts_only else None)
        rc_doc["steps_sampled"] += 1
        if w.get("supported"):
            rc_doc["supported"] = True
        rc_doc["total_observed"] += int(w.get("total_observed") or 0)
        rc_doc["distinct_named"] += int(w.get("distinct_named") or 0)
        if w.get("error") and not rc_doc["error"]:
            rc_doc["error"] = w["error"]
        for r in recs:
            if r["b"] is None:
                rc_doc["unpaired"] += 1
            key = (r["a"], r["b"], r["b_body"])
            hit = rc_seen.get(key)
            if hit is None:
                if len(rc_seen) >= RUN_CONTACT_PAIR_CAP:
                    rc_doc["truncated"] = True
                    continue
                hit = dict(r)
                hit["first_step"] = step_index
                hit["count"] = 0
                rc_seen[key] = hit
                rc_doc["pairs"].append(hit)
            hit["last_step"] = step_index
            hit["count"] += 1

    rows = 0
    with open(out_path, "w", encoding="utf-8", newline="") as fh:
        fh.write(",".join(header) + "\n")

        def sample(t):
            vals = ["%.17g" % t]
            for tr in tracks:
                # ``nan``, never a substituted 0.0 or a carried-forward value:
                # a pose that could not be read is UNMEASURED, and the reader
                # side keeps it non-finite so a grader's floors can see it.
                try:
                    p = _track_position(tr, meta["pose_sampling"])
                    vals += ["%.17g" % float(p[0]), "%.17g" % float(p[1]),
                             "%.17g" % float(p[2])]
                except Exception:
                    vals += ["nan", "nan", "nan"]
            fh.write(",".join(vals) + "\n")

        if not broke:
            sample(0.0)   # t=0 of the RECORDED window (i.e. after settle)
            watch_contacts(0)
            rows = 1
            for k in range(n_record):
                if sv.step(step_ms) == -1:
                    meta["step_returned_minus1"] = True
                    broke = True
                    break
                sample((k + 1) * dt_ms / 1000.0)
                watch_contacts(k + 1)
                rows += 1

    meta["run_contacts"] = rc_doc
    print("[agentbench_recorder] run-long contact watch: supported=%s, "
          "%d sample(s) every %s step(s), %d contact(s) observed, %d distinct "
          "robot pair(s)%s"
          % (rc_doc["supported"], rc_doc["steps_sampled"],
             rc_doc["every_n_steps"], rc_doc["total_observed"],
             len(rc_doc["pairs"]),
             ", TRUNCATED" if rc_doc["truncated"] else ""), flush=True)
    meta["rows"] = rows
    meta["recorded_s"] = (rows - 1) * dt_ms / 1000.0 if rows else 0.0

    complete = (not broke) and rows == n_record + 1
    print("[agentbench_recorder] wrote %d rows (%.3f sim s) -> %s (complete=%s)"
          % (rows, meta["recorded_s"], out_path, complete), flush=True)
    finish(complete)


if __name__ == "__main__":
    main()
