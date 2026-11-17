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

"""One tier-channel document -> the ladder's neutral channel dataclasses.

**Pure.** Nothing here launches an engine, opens a socket or touches a file:
it takes the ``dict`` the grader-owned sampler wrote
(``<out>.channels.json``, schema ``omnisim_ladder_channels/v1``) and returns
the dataclasses in ``ladder.graders.ladder_evidence`` /
``ladder.graders.t3_evidence`` / ``ladder.graders.t4_evidence``. That split is
what makes the whole T2-T4 surface unit-testable with no simulator, no GPU and
no engine binary -- the same reason the MuJoCo column keeps ``recording.py``
separate from its runners.

Three rules, all inherited and none of them ours to relax
--------------------------------------------------------

1. **Return only what was measured.** A key this module omits means
   *unmeasured*, and the shim falls back and reports the gap as
   ``scaffolding_defect_ours``. A key it supplies **empty** means
   *measured and there was nothing there*, which is a far stronger claim.
   The two are never interchanged.
2. **``None`` is "I could not answer"; ``0`` and ``False`` are measurements.**
3. **The adapter never chooses which body is which.** Every name this module
   selects on -- the object, the end effector, the container, the base, the
   ground -- arrives from the task file (``roles``, ``ground.names``,
   ``robot.declared_name``), which is where the shim's own fallbacks read it
   from too. The sampler recorded *every* named body precisely so that the
   selection could be made here against task data rather than in the scene.

Where a name resolves to more than one body
-------------------------------------------

The task files anticipate this ("Each of the three is BOTH a robot name and a
link name in its own file"). This module picks the **shallowest** body
carrying the name -- the outermost, i.e. the robot wrapper rather than a link
inside it -- breaks a depth tie on the lowest node id, and names every
candidate it rejected in the channel's ``source``. It does not refuse: the
refusal rule is the grader core's and belongs there, and a channel that
withheld the measurement would be indistinguishable from an instrument we
never built.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder.graders import ladder_evidence as lev      # noqa: E402
from ladder.graders import t3_evidence as t3ev         # noqa: E402

SCHEMA = "omnisim_ladder_channels/v1"

#: The eight keys each rung's shim reads. Published so a test can assert the
#: builder never invents a key the shim will silently drop.
T2_CHANNEL_KEYS = ("object_pose", "end_effector", "container", "grip",
                   "object_physics", "world", "support_surfaces",
                   "object_aabb")
T3_CHANNEL_KEYS = ("base_pose", "standing", "gait", "support", "arena",
                   "base_physics", "world", "controller")
T4_CHANNEL_KEYS = T3_CHANNEL_KEYS

#: The vertical extent at or above which a static body is called a wall rather
#: than a floor, for the arena's ``boundary_bodies`` list. Recorded, never a
#: pass condition: T3/T4's arena reading is a finding about the room, not a
#: threshold (``t3_evidence.T3_CHANNEL_ASSERTIONS['arena'] == ()``).
WALL_MIN_Z_SPAN_M = 0.20

MASS_DISCLOSURE = (
    ". NOTE ON WHICH MASS THIS IS, because the tier's cell boundary is "
    "0.02 x m.g and its published figure is a multiple of body weight: this "
    "is the named body's OWN Physics.mass (%s), NOT the whole robot "
    "(the subtree sums to %s, recorded beside it as subtree_mass_kg). The "
    "body's own reading is the literal one and is the STRICTER of the two "
    "(a smaller m means a tighter unsupported bound); which one the tier "
    "means is an open question for its owner. The MuJoCo column reports "
    "mjModel.body_mass here for the same reason, so the two columns are like "
    "for like")

SUPPORT_ZERO_TAIL = (
    ". ON THIS RUN NO ROUTE WAS OPEN, so the total is zero over the whole "
    "window by construction rather than by measurement: there was no "
    "Supervisor robot other than the grader's own sampler (nothing could call "
    "add_force), every tracked robot carried its own Physics node (nothing "
    "was held rigidly), none was parented into another body, none carried a "
    "Connector, and WorldInfo declared no physics plugin. The force and "
    "torque series below are that argument written out per sample; they are "
    "NOT a wrench read back off the engine, and this column cannot read one")

SUPPORT_OPEN_TAIL = (
    ". ON THIS RUN AT LEAST ONE ROUTE WAS OPEN, so the total is NOT "
    "attested and this cell is support_attestation: unverified rather than "
    "supported or unsupported -- publishing a zero here would publish a "
    "possibly-held robot in the cell reserved for numerically nothing. Open "
    "routes: %s")

GRIP_SOURCE = (
    "contacts naming the object and a body belonging to the carrier's robot "
    "subtree, taken from the grader-owned sampler's full contact record and "
    "timed on the pose clock. The mechanism is an OBSERVATION, not a "
    "declaration: 'attachment' when a Connector device is present in a "
    "tracked robot's subtree (the only constraint in this engine that can "
    "bind two bodies at run time), 'friction' when contacts hold the object "
    "and no Connector exists, and 'unknown' when neither is true. Suction is "
    "not separately observable here -- a vacuum gripper in this engine is "
    "either a Connector (reported as attachment) or ordinary contact "
    "(reported as friction) -- and is never guessed")


# --- small readers ------------------------------------------------------------


def _triple(v):
    if not v:
        return None
    try:
        out = tuple(float(x) for x in v[:3])
    except (TypeError, ValueError):
        return None
    return out if len(out) == 3 else None


def _num(v):
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) \
        else None


def _tri_bool(v):
    return bool(v) if isinstance(v, bool) else None


def _bodies(doc):
    return [b for b in (doc.get("bodies") or []) if isinstance(b, dict)]


def _inventory(doc):
    return [b for b in (doc.get("inventory") or []) if isinstance(b, dict)]


def select(records, name):
    """``(record, rejected)`` -- the shallowest body carrying ``name``.

    ``rejected`` names every other candidate, so an ambiguity travels into the
    channel's citation instead of being silently resolved.
    """
    if not name:
        return None, []
    hits = [r for r in records if (r.get("name") or "") == name]
    if not hits:
        return None, []
    hits.sort(key=lambda r: (int(r.get("depth") or 0),
                             int(r.get("id") or 0)))
    return hits[0], hits[1:]


#: ⚠ **MEASURED, and it is a finding about this column rather than a bug in
#: it.** OmniSim's URDF importer does not preserve the URDF *root link's*
#: name: the root link becomes the ``URDFRobot`` node itself, which carries
#: the name the ``.wbt`` gave it. On the T3 probe the shipped
#: ``walker.urdf`` -- whose root link is ``base_link`` and whose robot name is
#: ``walker`` -- produced a scene whose named bodies were ``walker``,
#: ``hip_fl``, ``thigh_fl``, ``shank_fl``, ... and **no ``base_link`` at
#: all**, with the root's own mass (6.0 kg) and the subtree's (10.8 kg,
#: exactly ``robot.mass_kg_declared``) both correct on the ``walker`` node. So
#: T3/T4's declared base name matches nothing on this column through no fault
#: of the agent's, which is precisely the ``open_question_for_the_freeze``
#: both tiers already record.
#:
#: The substitution below is the one T1's own core already makes -- *"prefers
#: the body carrying this name and falls back to the single robot-class body;
#: it never picks the luckiest of several"* -- applied here, refusing on
#: ambiguity exactly as that rule does, and **stated in the channel's
#: citation** so it travels into the verdict instead of living in a comment.
BASE_SUBSTITUTION = (
    ". SUBSTITUTION, DISCLOSED: no body in this scene carries the declared "
    "base name %r -- OmniSim's URDF importer folds the root link into the "
    "URDFRobot node, which carries the name the scene gave it -- so the "
    "single robot-class body in the scene (%r) was measured instead. This is "
    "the rule T1's own core already applies to the same problem, and it "
    "refuses rather than choosing where more than one candidate exists")

BASE_AMBIGUOUS = (
    "no body carries the declared base name %r and this scene holds %d "
    "robot-class bodies (%s), so there is no single unambiguous substitute "
    "and none was chosen. OmniSim's URDF importer folds a description's root "
    "link into the URDFRobot node, which carries the name the scene gave it")


def _robot_candidates(records):
    return [r for r in records
            if r.get("robot_class") and not r.get("is_grader_sampler")]


def resolve_base(records, name):
    """``(record, note, error)`` for the tier's declared base body.

    Prefer the declared name; fall back to the single robot-class body that
    is not the grader's own sampler; refuse when several are candidates.
    """
    rec, rejected = select(records, name)
    if rec is not None:
        return rec, _ambiguity(name, rejected), None
    cands = _robot_candidates(records)
    if len(cands) == 1:
        return cands[0], BASE_SUBSTITUTION % (name, cands[0].get("name")), None
    if not cands:
        return None, "", ("no body carries the declared base name %r and the "
                          "scene holds no robot-class body at all" % name)
    return None, "", (BASE_AMBIGUOUS
                      % (name, len(cands),
                         ", ".join(sorted(str(c.get("name")) for c in cands))))


def _ambiguity(name, rejected):
    if not rejected:
        return ""
    return (". WARNING: %d other bod%s in this scene also carr%s the name %r "
            "(%s); the shallowest was taken and the rest are named here"
            % (len(rejected), "ies" if len(rejected) > 1 else "y",
               "y" if len(rejected) > 1 else "ies", name,
               ", ".join("%s#%s at depth %s" % (r.get("type"), r.get("id"),
                                                r.get("depth"))
                         for r in rejected)))


# --- pose ---------------------------------------------------------------------


def pose_series(doc, name, *, label=""):
    """A :class:`PoseSeries` for the named body, or ``None`` when absent.

    ``None`` is deliberate and different from an errored series: it means the
    document has no body of that name at all, which is a fact about the
    agent's scene the core should see through the shim's own fallback rather
    than through an empty channel of ours.
    """
    rec, rejected = select(_bodies(doc), name)
    src = str(doc.get("pose_source") or "") + _ambiguity(name, rejected)
    t = doc.get("t_s")
    if rec is None:
        return lev.PoseSeries(
            body=name, source=src,
            error=("no body in the recorded scene carries the %s name %r. "
                   "Names recorded: %s"
                   % (label or "declared", name,
                      ", ".join(sorted({str(b.get('name'))
                                        for b in _bodies(doc)})) or "none")))
    xyz = rec.get("xyz") or []
    rot = rec.get("rot") or []
    n = min(len(t or []), len(xyz))
    if n < 1:
        return lev.PoseSeries(body=name, source=src,
                              error="the run recorded no pose samples")
    arr_t = np.asarray(t[:n], dtype=float)
    arr_xyz = np.asarray(xyz[:n], dtype=float)
    arr_rot = None
    if len(rot) >= n:
        flat = np.asarray(rot[:n], dtype=float)
        if flat.ndim == 2 and flat.shape[1] == 9:
            arr_rot = flat.reshape(n, 3, 3)
    return lev.PoseSeries(body=name, t=arr_t, xyz=arr_xyz, rot=arr_rot,
                          source=src)


# --- t=0 geometry --------------------------------------------------------------


def container_geometry(doc, name):
    rec, rejected = select(_inventory(doc), name)
    src = str(doc.get("inventory_source") or "") + _ambiguity(name, rejected)
    if rec is None:
        return lev.ContainerGeometry(
            body=name, source=src,
            error="no body in the recorded scene carries the declared "
                  "container name %r" % name)
    lo, hi = _triple(rec.get("aabb_min")), _triple(rec.get("aabb_max"))
    rim = hi[2] if hi else None
    return lev.ContainerGeometry(
        body=name, aabb_min=lo, aabb_max=hi, rim_z=rim,
        rim_rule=("the top of the union of the container's WHOLE SUBTREE of "
                  "world AABBs, which on a bin whose walls are child bodies "
                  "is the lip of those walls and not the top of its floor "
                  "slab. Measured, not declared: bounds_for_subtree walks "
                  "every geometry under the node"),
        t_s=0.0,
        source=src + (". bounds are %s"
                      % ("exact" if rec.get("bounds_exact")
                         else "approximate (a mesh bound the walker could "
                              "not open exactly)")),
        error=rec.get("bounds_error"))


def object_aabb(doc, name):
    rec, _rej = select(_inventory(doc), name)
    if rec is None:
        return None
    lo, hi = _triple(rec.get("aabb_min")), _triple(rec.get("aabb_max"))
    if lo is None or hi is None:
        return None
    return (lo, hi)


def support_surfaces(doc):
    """Every STATIC non-robot body carrying a world box. Structural, not names.

    A surface the agent named something unexpected is still found, and a body
    that can move is still excluded -- which is the point: T2.2 asks whether
    the object came to rest on something, and a name list would answer a
    different question.
    """
    out = []
    src = (str(doc.get("inventory_source") or "")
           + ". A body counts as a support surface when it carries NO Physics "
             "node (so the engine can never move it) and is not itself a "
             "robot. That is structural: it does not consult the declared "
             "names at all")
    for rec in _inventory(doc):
        if rec.get("robot_class"):
            continue
        if not rec.get("static"):
            continue
        lo, hi = _triple(rec.get("aabb_min")), _triple(rec.get("aabb_max"))
        if lo is None or hi is None:
            continue
        out.append(lev.SupportSurface(body=str(rec.get("name") or ""),
                                      aabb_min=lo, aabb_max=hi, static=True,
                                      source=src))
    return out


def body_physics(doc, name, *, disclose_mass=False):
    rec, rejected = select(_inventory(doc), name)
    src = str(doc.get("mass_source") or "") + _ambiguity(name, rejected)
    if rec is None:
        return lev.BodyPhysics(
            body=name, source=src,
            error="no body in the recorded scene carries the name %r" % name)
    mass = _num(rec.get("mass_kg"))
    if disclose_mass:
        src += MASS_DISCLOSURE % (
            "%.4f kg" % mass if mass is not None else "unreadable",
            ("%.4f kg" % _num(rec.get("subtree_mass_kg"))
             if _num(rec.get("subtree_mass_kg")) is not None
             else "unreadable"))
    return lev.BodyPhysics(body=str(rec.get("name") or name), mass_kg=mass,
                           dynamic=_tri_bool(rec.get("has_physics")),
                           source=src, error=rec.get("mass_error"))


def world_physics(doc):
    w = doc.get("world") or {}
    return lev.WorldPhysics(gravity_mps2=_num(w.get("gravity_mps2")),
                            gravity_vec=_triple(w.get("gravity_vec_mps2")),
                            source=str(w.get("source") or ""),
                            error=w.get("error"))


# --- contacts ------------------------------------------------------------------


def _contact_doc(doc):
    return doc.get("contacts") or {}


def _owner_of(doc, name):
    """The robot identifier the named body belongs to, or ``None``."""
    rec, _rej = select(_inventory(doc), name)
    return (rec or {}).get("owner_robot")


def grip_observation(doc, roles, *, backend_note=""):
    """T2's grip. Recorded in every cell and graded in none."""
    c = _contact_doc(doc)
    obj = getattr(roles, "object_name", "") or ""
    carrier = _owner_of(doc, getattr(roles, "end_effector_name", "") or "")
    obj_owner = _owner_of(doc, obj)
    attachment = bool((doc.get("structure") or {}).get(
        "robots_with_connectors"))
    hits = []
    for p in (c.get("pairs") or []):
        if not isinstance(p, dict):
            continue
        names = (p.get("a_name") or "", p.get("b_name") or "")
        owners = (p.get("a_robot"), p.get("b_robot"))
        for i in (0, 1):
            j = 1 - i
            is_object = names[i] == obj or (obj_owner is not None
                                            and owners[i] == obj_owner)
            if not is_object:
                continue
            holds = (owners[j] is not None and owners[j] != owners[i]
                     and (carrier is None or owners[j] == carrier))
            if not holds:
                continue
            hits.append(lev.GripContact(holder_body=names[j] or owners[j],
                                        held_body=names[i] or obj,
                                        point=_triple(p.get("point")),
                                        t_s=_num(p.get("t_s")),
                                        step=p.get("step")))
            break
    if attachment:
        mechanism = "attachment"
    elif hits:
        mechanism = "friction"
    else:
        mechanism = "unknown"
    return lev.GripObservation(
        mechanism=mechanism, mechanism_source=GRIP_SOURCE, contacts=hits,
        attachment=attachment, supported=bool(c.get("supported")),
        total_observed=_opt_int(c.get("total_observed")),
        distinct_named=_opt_int(c.get("distinct_named")),
        steps_sampled=int(c.get("steps") or 0),
        window_s=_num(c.get("window_s")),
        source=GRIP_SOURCE + (". " + backend_note if backend_note else ""),
        error=(c.get("error")
               or (backend_note if (backend_note and not hits) else None)))


def _opt_int(v):
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def gait_contacts(doc, surface, robot_name, *, backend_note=""):
    """T3/T4's per-foot contact record, on the query clock."""
    c = _contact_doc(doc)
    ground = set(getattr(surface, "names", ()) or ())
    owner = _owner_of(doc, robot_name)
    out = []
    for p in (c.get("pairs") or []):
        if not isinstance(p, dict):
            continue
        owners = (p.get("a_robot"), p.get("b_robot"))
        names = (p.get("a_name") or "", p.get("b_name") or "")
        grounds = (p.get("a_is_ground"), p.get("b_is_ground"))
        idx = None
        if owner is not None:
            if owners[0] == owner and owners[1] != owner:
                idx = 0
            elif owners[1] == owner and owners[0] != owner:
                idx = 1
        elif bool(owners[0]) != bool(owners[1]):
            idx = 0 if owners[0] else 1
        if idx is None:
            continue
        j = 1 - idx
        other_name = names[j]
        is_ground = grounds[j]
        if is_ground is None and other_name:
            is_ground = other_name in ground
        elif is_ground is not None and other_name:
            is_ground = bool(is_ground) and (not ground
                                             or other_name in ground)
        out.append(t3ev.GroundContact(
            robot_body=names[idx] or owners[idx],
            other_body=other_name or p.get("b" if j else "a"),
            other_is_ground=_tri_bool(is_ground),
            other_is_robot=(owners[j] is not None) if p.get("paired")
            else None,
            point=_triple(p.get("point")), t_s=_num(p.get("t_s")),
            step=p.get("step")))
    times = c.get("sample_times") or []
    return t3ev.GaitContactObservation(
        contacts=out,
        sample_times=(np.asarray(times, dtype=float) if len(times) >= 1
                      else None),
        supported=bool(c.get("supported")),
        total_observed=_opt_int(c.get("total_observed")),
        distinct_named=_opt_int(c.get("distinct_named")),
        steps_sampled=int(c.get("steps") or 0),
        window_s=_num(c.get("window_s")),
        source=str(c.get("source") or "") + (
            ". The robot side of a pair is the body whose owning robot "
            "subtree is the graded robot%s; 'is the ground' is the other "
            "side's name matched against the names the task declared, and "
            "where the sampler and the task disagree the STRICTER reading "
            "wins here as well as in the core"
            % (" (%s)" % owner if owner else
               ", or -- when the declared base name matched no body -- any "
               "robot subtree, which is the same thing on a single-robot "
               "scene and is stated because it is not on a crowded one"))
        + (". " + backend_note if backend_note else ""),
        error=(c.get("error")
               or (backend_note if (backend_note and not out) else None)))


# --- T3/T4 structural channels -------------------------------------------------


def standing_height(doc, robot_name):
    series = pose_series(doc, robot_name, label="declared base")
    if series is None or series.xyz is None or len(series.xyz) < 1:
        return t3ev.StandingHeight(
            body=robot_name, source=str(doc.get("pose_source") or ""),
            error=(series.error if series is not None
                   else "no pose series for the declared base"))
    return t3ev.StandingHeight(
        z_m=float(series.xyz[0][2]), body=robot_name,
        source=str(doc.get("pose_source") or "") + (
            ". The settled standing height is the base's z at the FIRST "
            "recorded sample, which is taken after the tier's own settle "
            "window has already run -- so it is the height the robot came to "
            "rest at, not the height it was authored at"))


def arena_bounds(doc, surface):
    inv = _inventory(doc)
    names = set(getattr(surface, "names", ()) or ())
    floors = [r for r in inv
              if (r.get("name") or "") in names and _triple(r.get("aabb_min"))
              and _triple(r.get("aabb_max"))]
    if not floors:
        floors = [r for r in inv
                  if r.get("static") and not r.get("robot_class")
                  and _triple(r.get("aabb_min"))
                  and _triple(r.get("aabb_max"))
                  and (_triple(r["aabb_max"])[2]
                       - _triple(r["aabb_min"])[2]) < WALL_MIN_Z_SPAN_M]
    if not floors:
        return t3ev.ArenaBounds(
            source=str(doc.get("arena_source") or ""),
            error="no static body carrying a world box matched the declared "
                  "walking-surface names, and none was flat enough to be "
                  "taken for a floor")
    lo = [min(_triple(r["aabb_min"])[i] for r in floors) for i in range(3)]
    hi = [max(_triple(r["aabb_max"])[i] for r in floors) for i in range(3)]
    walls = sorted({str(r.get("name") or "") for r in inv
                    if r.get("static") and not r.get("robot_class")
                    and _triple(r.get("aabb_min")) and _triple(r.get("aabb_max"))
                    and (_triple(r["aabb_max"])[2]
                         - _triple(r["aabb_min"])[2]) >= WALL_MIN_Z_SPAN_M})
    return t3ev.ArenaBounds(aabb_min=tuple(lo), aabb_max=tuple(hi),
                            boundary_bodies=walls,
                            source=str(doc.get("arena_source") or ""))


def controller_load(doc, robot_name, *, declared_method="unknown"):
    owner = _owner_of(doc, robot_name)
    entries = [c for c in (doc.get("controllers") or []) if isinstance(c, dict)]
    rec = None
    for c in entries:
        if c.get("robot") == robot_name:
            rec = c
            break
    if rec is None and owner is not None:
        for r in _inventory(doc):
            if r.get("ident") == owner and r.get("robot_class"):
                for c in entries:
                    if c.get("robot") == r.get("name"):
                        rec = c
                        break
                break
    if rec is None and len(entries) == 1:
        rec = entries[0]
    if rec is None:
        return t3ev.ControllerLoad(
            declared_method=declared_method,
            source=str(doc.get("pose_source") or ""),
            error="no robot in the recorded scene could be tied to the "
                  "declared base name %r, so nothing attests a controller"
                  % robot_name)
    return t3ev.ControllerLoad(
        declared_method=declared_method, loaded=_tri_bool(rec.get("loaded")),
        evidence=str(rec.get("evidence") or ""),
        identity=str(rec.get("controller") or ""),
        source=str(rec.get("source") or ""))


def applied_support(doc):
    """The tier's own measurement -- as far as this engine can attest it.

    OmniSim has **no wrench read-back**. ``add_force`` / ``add_torque`` are
    write-only from a Supervisor and nothing reports what another controller
    applied; contact points carry no force either. So this channel attests a
    zero **only by proving no route is open**, and reports ``attested=None``
    (the tier's ``unverified`` cell) the moment one is -- which is not a
    failure and not a credit, exactly as ``T3_quadruped/meta.json`` ->
    ``support_attestation`` requires.
    """
    st = doc.get("structure") or {}
    head = str(st.get("source") or "")
    routes = list(st.get("routes_open") or [])
    times = (_contact_doc(doc).get("sample_times")
             or doc.get("t_s") or [])
    if routes or not isinstance(st.get("attested"), bool):
        return t3ev.AppliedSupport(
            attested=None,
            source=head + (SUPPORT_OPEN_TAIL % "; ".join(routes)
                           if routes else
                           ". The structural probe did not run, so no route "
                           "can be ruled out"),
            error=("; ".join(routes) if routes
                   else "the structural support probe produced no verdict"))
    n = max(1, len(times))
    zeros3 = np.zeros((n, 3), dtype=float)
    return t3ev.AppliedSupport(
        attested=True, t=np.asarray(times if times else [0.0], dtype=float),
        force=zeros3, torque=zeros3, peak_force_n=0.0, peak_torque_nm=0.0,
        fraction_nonzero=0.0, source=head + SUPPORT_ZERO_TAIL)


# --- the three hooks' bodies ---------------------------------------------------


def build_t2(doc, roles, *, backend_note=""):
    """``{key: channel}`` for T2. ``{}`` when there is no document."""
    if not doc:
        return {}
    obj = getattr(roles, "object_name", "") or ""
    eff = getattr(roles, "end_effector_name", "") or ""
    con = getattr(roles, "container_name", "") or ""
    out = {"world": world_physics(doc),
           "support_surfaces": support_surfaces(doc),
           "grip": grip_observation(doc, roles, backend_note=backend_note),
           "container": container_geometry(doc, con),
           "object_physics": body_physics(doc, obj)}
    p = pose_series(doc, obj, label="declared object")
    if p is not None:
        out["object_pose"] = p
    e = pose_series(doc, eff, label="declared end-effector")
    if e is not None:
        out["end_effector"] = e
    box = object_aabb(doc, obj)
    if box is not None:
        out["object_aabb"] = box
    return out


def build_t3(doc, surface, robot_name, *, backend_note=""):
    """``{key: channel}`` for T3 and T4 -- the eight are the same eight.

    The base is resolved ONCE, here, and every base-dependent channel is
    built against the body that resolution actually landed on -- with the
    substitution note appended to each of their citations, so a reader of any
    one of them learns that the declared name matched nothing.
    """
    if not doc:
        return {}
    rec, note, err = resolve_base(_inventory(doc), robot_name)
    name = str((rec or {}).get("name") or robot_name)
    out = {"support": applied_support(doc),
           "arena": arena_bounds(doc, surface),
           "world": world_physics(doc)}
    if rec is None:
        # The base could not be resolved AT ALL. Every base channel is
        # supplied carrying the reason rather than omitted: omitting would
        # read as "we never built the instrument", and the instrument ran.
        out["standing"] = t3ev.StandingHeight(body=robot_name, error=err,
                                              source=str(doc.get(
                                                  "inventory_source") or ""))
        out["base_physics"] = lev.BodyPhysics(body=robot_name, error=err,
                                              source=str(doc.get(
                                                  "mass_source") or ""))
        out["controller"] = t3ev.ControllerLoad(error=err, source=str(
            doc.get("inventory_source") or ""))
        out["base_pose"] = lev.PoseSeries(body=robot_name, error=err,
                                          source=str(doc.get("pose_source")
                                                     or ""))
        out["gait"] = gait_contacts(doc, surface, robot_name,
                                    backend_note=backend_note)
        return out
    out["standing"] = standing_height(doc, name)
    out["gait"] = gait_contacts(doc, surface, name,
                                backend_note=backend_note)
    out["base_physics"] = body_physics(doc, name, disclose_mass=True)
    out["controller"] = controller_load(doc, name)
    p = pose_series(doc, name, label="declared base")
    if p is not None:
        out["base_pose"] = p
    if note:
        for key in ("standing", "gait", "base_physics", "controller",
                    "base_pose"):
            ch = out.get(key)
            if ch is not None and hasattr(ch, "source"):
                ch.source = str(ch.source or "") + note
    return out


build_t4 = build_t3


def check_document(doc):
    """Problems with a tier document, ``[]`` when it is well formed.

    The executable half of the schema: a column runs this against its own
    output before anyone spends a token on a cell.
    """
    problems = []
    if not isinstance(doc, dict):
        return ["the document is not an object"]
    if doc.get("schema") != SCHEMA:
        problems.append("schema is %r, expected %r"
                        % (doc.get("schema"), SCHEMA))
    t = doc.get("t_s")
    if not isinstance(t, list) or len(t) < 2:
        problems.append("t_s is not a series of at least two samples")
    for b in _bodies(doc):
        if len(b.get("xyz") or []) != len(t or []):
            problems.append("body %r has %d pose samples for %d times"
                            % (b.get("name"), len(b.get("xyz") or []),
                               len(t or [])))
            break
    if not _inventory(doc):
        problems.append("the frozen inventory is empty")
    if not isinstance((doc.get("structure") or {}).get("attested"), bool):
        problems.append("the structural support probe produced no verdict")
    w = doc.get("world") or {}
    if w.get("gravity_mps2") is None and not w.get("error"):
        problems.append("gravity is absent with no reason given")
    c = _contact_doc(doc)
    if c.get("supported") and not c.get("sample_times"):
        problems.append("the contact scan ran but recorded no query times, "
                        "so the make-and-break clause cannot fire")
    if any(math.isnan(x) for x in (t or []) if isinstance(x, float)):
        problems.append("the time series carries NaN")
    return problems


__all__ = ["SCHEMA", "T2_CHANNEL_KEYS", "T3_CHANNEL_KEYS", "T4_CHANNEL_KEYS",
           "WALL_MIN_Z_SPAN_M", "applied_support", "arena_bounds",
           "body_physics", "build_t2", "build_t3", "build_t4",
           "check_document", "container_geometry", "controller_load",
           "gait_contacts", "grip_observation", "object_aabb", "pose_series",
           "select", "standing_height", "support_surfaces", "world_physics"]
