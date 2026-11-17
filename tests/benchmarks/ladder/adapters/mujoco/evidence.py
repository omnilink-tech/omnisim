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

"""**MuJoCo 3.8.1** -> :class:`EvidenceBundle` (+ the ladder's support channel).

The MuJoCo column's evidence builder. It reads only what a MuJoCo run can
actually produce (:mod:`ladder.adapters.mujoco.recording` is the artifact
contract) and maps it into the neutral schema, answering the two questions the
sim-agnostic cores are forbidden to ask -- *"is this body the robot the task
named"* and *"which engine drove this run"* -- and publishing the rule it used
for each.

What MuJoCo gives us, and what it does not
------------------------------------------

======================  ====================================================
neutral field           the MuJoCo channel, and its limit
======================  ====================================================
``identity``            **MuJoCo has no robot node type.** A model is one
                        kinematic tree of bodies; nothing in ``mjModel`` says
                        "this subtree is a robot". So the predicate is read
                        structurally -- a **direct child of the world body
                        whose subtree carries joints** -- and the rule is
                        published in full. The *declaration* half is counted
                        in the source URDF (``<robot name=...>``), which is
                        where the model's name survives: MuJoCo's own MJCF
                        output keeps it only as the ``<mujoco model="...">``
                        attribute.
``roster`` / ``t0``     an in-process scan between ``mj_forward`` and the
                        first ``mj_step``. **Frozen by construction** -- there
                        is no simulator process to race with, so this is the
                        one column where a t=0 inventory needs no arrangement
                        to be honest. World-space AABBs are computed by the
                        recorder from ``mjModel.geom_aabb`` carried through
                        ``mjData.geom_xpos``/``geom_xmat``; MuJoCo exposes no
                        per-body world AABB directly.
``contacts``            ``mjData.contact`` every physics step, each contact's
                        two geoms mapped to bodies via ``mjModel.geom_bodyid``.
                        Naming both sides is trivial here (unlike the pipeline
                        that produced OmniSim's ``(id, id)`` self-pair bug),
                        but the two vacuity counters are still mandatory and
                        still counted over EVERY step.
``trajectory``          the recorder's own per-physics-step sampling, in
                        simulated seconds and metres. Not polled, not
                        resampled.
``process``             exit code + captured stdout/stderr of the runner
                        process. ⚠ MuJoCo writes **no log file**; there is no
                        post-hoc channel at all.
``reached_finalize``    MuJoCo has **no build/finalize phase to witness**:
                        ``mj_loadXML`` returns a compiled model or raises, so
                        a model that did not compile cannot be stepped. The
                        witnesses used are the ENGINE-authored
                        ``mj_saveLastXML`` output and the driver's own record
                        that the clock advanced; both are cited separately
                        because they are not of equal strength.
``attribution``         a per-run **reading**, not an invariant: the solver,
                        integrator, cone, timestep and gravity are read off
                        the ``mjOption`` of the model that ran, together with
                        ``mj_versionString()``.
======================  ====================================================

Two places this adapter deliberately differs from the shipped columns
---------------------------------------------------------------------

``_error_lines`` counts MuJoCo's warning counters, not just console text
    The OmniSim and Webots adapters count ``ERROR:`` lines out of an engine
    log. MuJoCo has no log and no ``ERROR:`` line: its hard errors go through
    ``mju_error``, which **aborts the process** (so they arrive as a non-zero
    exit code, not as text), and everything short of that is a
    ``mjData.warning`` counter -- a filled contact buffer, a truncated
    constraint set, a non-finite ``qpos``. Counting only console text would
    make T1.5 **unfalsifiable on this column**: a run whose solver silently
    truncated would read "clean, zero error lines". So the physics warning
    counters are mapped to error-class lines and the mapping is stated on
    every row. This is a real asymmetry between the columns and it is
    published rather than smoothed over; note its direction, which is
    *stricter on MuJoCo*, and see :data:`WARNING_NOTE`.

``robot_class`` and ``ContactPair.a_robot`` mean different things
    ``Body.robot_class`` answers the task's identity predicate and is true for
    **one** body (the assembly's root). ``a_robot``/``b_robot`` on a contact ask
    whether that body is part of the robot's kinematic subtree, which is true
    for the wheels too. Conflating them would make every wheel-ground contact
    look like a contact with something that is not the robot -- or, worse,
    make the robot appear to be touching itself.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench.graders.evidence import (  # noqa: E402
    Body, BodyInventory, ContactObservation, ContactPair, EngineAttribution,
    EvidenceBundle, IdentityRule, ProcessFacts, Trajectory)
import numpy as np  # noqa: E402

from ladder.adapters.mujoco import recording  # noqa: E402
from ladder.graders.ladder_evidence import (  # noqa: E402
    BodyPhysics, ContainerGeometry, GripContact, GripObservation, PoseSeries,
    SupportContact, SupportContactObservation, SupportSurface, WorldPhysics)

SIM = "mujoco"
ADAPTER = "ladder.adapters.mujoco.evidence"

# The reference release every citation below is written against. MuJoCo's URDF
# compiler defaults have changed within living memory (``strippath`` was once
# hardcoded true for URDF and is now a plain false-defaulting attribute), so a
# different version is a different measurement, not the same one.
MUJOCO_RELEASE = "3.8.1"

# Kept identical to the OmniSim and Webots adapters' floor on purpose: the three
# identity rules must differ ONLY in the channel that carries the name, so a
# cross-column reader can see that the identity question is the same question.
MIN_JOINTS = 4

# ``mjData.warning`` counters that describe the PHYSICS going wrong. The two
# excluded ones are rendering-only (``mjWARN_VGEOMFULL``) or a texture/asset
# note; neither says anything about whether the simulation was sound.
PHYSICS_WARNINGS = ("mjWARN_INERTIA", "mjWARN_CONTACTFULL",
                    "mjWARN_CNSTRFULL", "mjWARN_BADQPOS", "mjWARN_BADQVEL",
                    "mjWARN_BADQACC", "mjWARN_BADCTRL")

# MuJoCo's compiler writes "Error: ..." and the python bindings raise; a run
# that got far enough to write artifacts will rarely have one, but a captured
# console is the only place it could appear.
_ERROR_LINE = re.compile(r"(?mi)^\s*(?:Error|FATAL|ERROR):")


# --- citations ---------------------------------------------------------------

_ATTRIBUTION_SOURCE = (
    "read off the mjOption of the model that drove THIS run -- "
    "mjModel.opt.solver / .integrator / .cone / .timestep / .gravity, plus "
    "mujoco.mj_versionString() and the resolved path of the mujoco module -- "
    "and written into completion.json by the runner at the end of the run. "
    "MuJoCo is a single library with no engine-selection field and no verdict "
    "sidecar, so 'which engine' is answered by naming the library and the "
    "version, and 'driven how' by the solver settings that were actually in "
    "force. WARNING ON A WORD: mjOption.solver == 'Newton' is MuJoCo's own "
    "constraint solver (Newton's method on the constraint problem). It is NOT "
    "NVIDIA Newton, the GPU physics engine this repository's default backend "
    "uses, and a cross-column table will contain both.")

_ROSTER_SOURCE = (
    "the grader-owned in-process scan of mjModel/mjData taken after the settle "
    "window and BEFORE the first recorded step. MuJoCo has no simulator "
    "process and no scene service: the recorder owns the stepping loop, so "
    "nothing can advance the clock while the scan runs")
_T0_SOURCE = (
    "the same scan, which is frozen BY CONSTRUCTION rather than by "
    "arrangement: MuJoCo is a library, the recorder calls mj_step itself, and "
    "a scan between mj_forward and the first step is at t=0 with no "
    "synchronization flag to trust. Each world-space AABB is computed by the "
    "recorder from mjModel.geom_aabb carried through mjData.geom_xpos/"
    "geom_xmat and re-min/maxed over the eight transformed corners -- MuJoCo "
    "exposes no per-body world AABB of its own")
_CONTACT_SOURCE = (
    "mjData.contact scanned at EVERY physics step of the recorded window; each "
    "contact's geom1/geom2 mapped to bodies through mjModel.geom_bodyid and "
    "named with mj_id2name. Both participants are always nameable on this "
    "column, so the vacuity counters measure whether contacts HAPPENED rather "
    "than whether the query works")
_TRAJ_SOURCE = (
    "the grader-owned recorder: every body's world position (mjData.xpos) "
    "sampled once per physics step in simulated seconds and metres")
_LOG_SOURCE = (
    "the captured stdout+stderr of the runner process plus its exit code, AND "
    "the mjData.warning counters recorded at the end of the run. WARNING: "
    "MuJoCo writes NO log file -- there is no post-hoc equivalent of "
    "$OMNISIM_LOG_PATH, so whatever was not captured at launch does not exist")
_NO_LOG_SOURCE = (
    "NOT AVAILABLE for this run: MuJoCo writes no log file and no "
    "stdout/stderr capture was found, so its compiler errors and mju_warning "
    "text could not be read at all")
_STARTS_SOURCE = (
    "the runner's own control-loop record (completion.json ctrl_writes). "
    "MuJoCo has no controller-process concept at all: a behaviour is a "
    "function called between mj_step calls, so 'a behaviour started' is "
    "evidenced by the number of actuator-command writes the loop made, and "
    "never by a process table")
_SAVED_MODEL_FINALIZE = (
    "a non-empty mj_saveLastXML output: MuJoCo itself serialised the compiled "
    "model that drove this run. mj_loadXML either returns a compiled mjModel "
    "or raises with the compiler's error string, so a model that did not "
    "compile cannot be saved and cannot be stepped")

# --- notes carried on every MuJoCo verdict -----------------------------------

_NO_APPLICATION_NOTE = (
    "MuJoCo is a LIBRARY, not an application: there is no simulator process to "
    "launch, no world to load, no controller to spawn, no log file, no "
    "scene-tree service and no hot reload. Every fact in this bundle is either "
    "something the grader-owned recorder computed inside its own stepping loop "
    "or something the engine serialised on request. That gives this column a "
    "genuinely frozen t=0 scan and a genuinely per-step pose series, and it "
    "takes away every independently-written engine artifact the other columns "
    "read.")

WARNING_NOTE = (
    "error-class lines on this column are MuJoCo's mjData.warning counters "
    "(%s), not text: MuJoCo's hard errors go through mju_error, which ABORTS "
    "the process and therefore arrives as a non-zero exit code rather than as "
    "a line, and everything short of that is a counter. Counting only console "
    "text would make 'the run is real' unfalsifiable here -- a run whose "
    "contact buffer filled and whose solver truncated would read clean. The "
    "asymmetry with the ERROR:-line columns is real and its direction is "
    "STRICTER on MuJoCo." % ", ".join(PHYSICS_WARNINGS))

_MASS_NOTE = (
    "the compiled model's total mass is %.3f kg against the %.3f kg the URDF "
    "declares. MuJoCo's compiler/inertiafromgeom defaults to 'auto': a link "
    "with no <inertial> gets mass and inertia INFERRED from its geoms at "
    "geom/density (1000 kg/m^3), where URDF treats such a link as massless. "
    "Nothing warns. See build.json for whether the build set "
    "inertiafromgeom=false to reproduce the declared mass.")


# --- presentation-only wording ----------------------------------------------

_COMMON_LABELS = {
    "exit_code": "exit code",
    "error_lines": "mjData.warning counters + console error lines",
    "timed_out": "timed out",
}

LABELS = {
    "T1_arrive": dict(
        _COMMON_LABELS,
        artifact_missing_detail="no MJCF scene was produced",
        motion_absent_default="no recorder samples",
        identity_scene_count="bodies with a free joint and >= %d subtree "
                             "joints" % MIN_JOINTS,
        identity_declared_count="<robot name=...> declarations in the source "
                                "description",
        driver_completed="the recorder reached the end of its schedule",
        finalize_evidence="finalize evidence",
        dynamic="non-zero subtree mass",
    ),
}


# --- the adapter -------------------------------------------------------------


def build_bundle(task_id, *, robot_identity="any_robot", live_expected=False,
                 artifact=None, run=None, run_dir=None, phase_b=None,
                 scratch_dir=None, mujoco_dir=None, **_):
    """Read one MuJoCo run into a neutral bundle. **Never raises.**

    ``run`` is a :class:`~ladder.adapters.mujoco.recording.MujocoRun`; pass it
    when the caller already parsed the run. Otherwise ``phase_b`` (a run
    directory or a ``MujocoRun``), ``mujoco_dir``, or ``run_dir`` is searched.

    The OmniSim-shaped ``harness`` keyword is accepted and ignored: MuJoCo has
    no HTTP surface and no live session to observe.
    """
    labels = dict(LABELS.get(task_id, _COMMON_LABELS))
    bundle = EvidenceBundle(task=task_id, sim=SIM, adapter=ADAPTER,
                            labels=labels)

    tried = []
    if run is None and isinstance(phase_b, recording.MujocoRun):
        run = phase_b
    if run is None:
        for cand in _run_dir_candidates(mujoco_dir, run_dir, phase_b):
            tried.append(str(cand))
            got = recording.read_run(cand)
            if got.any_evidence:
                run = got
                break
            run = run or got
        if run is None:
            run = recording.read_run(None)

    bundle.adapter_measurements["motion"] = {
        "mujoco_run": dict(run.as_dict(), dirs_tried=tried)}
    if not run.any_evidence:
        bundle.notes.append(
            "no MuJoCo run artifacts found (looked in: %s). Every field below "
            "is 'the adapter could not answer', which is not the same as a "
            "failing measurement."
            % (", ".join(tried) if tried else "nowhere -- no run_dir and no "
                                              "mujoco_dir"))
    bundle.notes.append(_NO_APPLICATION_NOTE)
    bundle.notes.append(WARNING_NOTE)

    # -- the deliverable ---------------------------------------------------
    if artifact is None:
        artifact = run.scene
    if artifact is None and scratch_dir is not None:
        cands = sorted(Path(scratch_dir).rglob("*.xml"))
        artifact = str(cands[0]) if cands else None
    bundle.artifact = str(artifact) if artifact else None

    # -- identity ----------------------------------------------------------
    bundle.identity = _identity(robot_identity, run, bundle)

    # -- roster + the frozen t=0 inventory ---------------------------------
    #
    # ONE scan feeds both. MuJoCo has no second channel and needs none: unlike
    # the OmniSim arm, whose roster comes from a free-running harness, this
    # scan is at t=0 by construction, so roster and t0 are the same bodies in
    # the same order -- which also satisfies the trajectory index invariant.
    rdoc = run.roster or {}
    if run.roster is None:
        err = (run.errors.get("roster")
               or run.missing.get("roster", "no roster recorded"))
        bundle.roster = BodyInventory(source=_ROSTER_SOURCE, error=err)
        bundle.t0 = BodyInventory(source=_T0_SOURCE, error=err)
    else:
        recs = rdoc.get("bodies")
        recs = recs if isinstance(recs, list) else []
        bodies = [_body(r, robot_identity) for r in recs
                  if isinstance(r, dict)]
        frozen, note = _frozen(rdoc)
        if note:
            bundle.notes.append(note)
        t_s = rdoc.get("t_s")
        t_s = float(t_s) if isinstance(t_s, (int, float)) else None
        bundle.roster = BodyInventory(bodies=bodies, frozen=frozen, t_s=t_s,
                                      source=_ROSTER_SOURCE,
                                      error=rdoc.get("error"))
        bundle.t0 = BodyInventory(bodies=bodies, frozen=frozen, t_s=t_s,
                                  source=_T0_SOURCE, error=rdoc.get("error"))
        if bodies and not any(b.has_aabb for b in bodies):
            bundle.notes.append(
                "no world-space AABB on any body: the recorder computes them "
                "from geom_aabb and the per-geom world transform, so an empty "
                "result means the scan ran before mj_forward or the bodies "
                "carry no geoms at all.")
    bundle.adapter_measurements["roster"] = {
        "mujoco": {"bodies": len(bundle.roster.bodies),
                   "frozen": bundle.roster.frozen,
                   "source_file": str(run.files.get("roster") or "")}}

    # -- contacts ----------------------------------------------------------
    # ABSENT MEANS None: an empty ContactObservation would claim "the query ran
    # and found nothing", which is a different and much stronger statement.
    if run.contacts is not None:
        bundle.contacts = _contacts(run.contacts)

    # -- trajectory --------------------------------------------------------
    if run.trajectory is None:
        bundle.motion_error = (
            run.errors.get("trajectory") or run.missing.get("trajectory")
            or labels.get("motion_absent_default", "no motion samples"))
    else:
        arr = recording.to_arrays(run.trajectory)
        for p in arr.problems:
            bundle.notes.append("trajectory: %s" % p)
        if arr.fatal:
            bundle.motion_error = arr.fatal
            bundle.trajectory = Trajectory(source=_TRAJ_SOURCE,
                                           error=arr.fatal)
        else:
            bundle.trajectory = _trajectory(arr, bundle, run)

    # -- process facts -----------------------------------------------------
    bundle.process = _process(run)

    # -- engine attribution ------------------------------------------------
    bundle.attribution = _attribution(run)
    if bundle.attribution is None:
        bundle.notes.append(
            "no engine attribution: nothing here evidences that a MuJoCo "
            "process ran at all (no completion record, no saved model, no "
            "captured console, no exit code). 'MuJoCo is MuJoCo' describes the "
            "library, not a directory of numbers, so the run stays "
            "unattributed -- which is INVALID rather than FAIL.")

    # -- 'the artifact loaded' ---------------------------------------------
    # There is no load RPC to have succeeded. The honest witness is that the
    # in-process scan enumerated a scene at all: a scene that did not compile
    # produces no roster, because mj_loadXML raised before the loop began.
    if run.roster is not None or run.trajectory is not None:
        bundle.live_load_ok = True
    elif run.any_evidence:
        bundle.live_load_ok = False
    else:
        bundle.live_load_ok = None

    # -- the build recipe, recorded and never graded -----------------------
    if run.build:
        bundle.adapter_measurements["build"] = run.build
        d = run.build.get("mass_kg_declared_by_urdf")
        c = run.build.get("mass_kg_in_the_compiled_model")
        if isinstance(d, (int, float)) and isinstance(c, (int, float)):
            bundle.adapter_measurements["build"]["mass_check"] = {
                "declared_kg": d, "compiled_kg": c,
                "ratio": (c / d) if d else None}
            if d and abs(c - d) > 0.02 * d:
                bundle.notes.append(_MASS_NOTE % (c, d))
        for p in run.build.get("problems") or []:
            bundle.notes.append("build: %s" % p)
    return bundle


# --- run location -----------------------------------------------------------


def _run_dir_candidates(mujoco_dir, run_dir, phase_b):
    out = []
    for c in (mujoco_dir, phase_b if isinstance(phase_b, (str, Path)) else None):
        if c:
            out.append(Path(c))
    if run_dir:
        rd = Path(run_dir)
        out += [rd / "mujoco", rd]
    seen, uniq = set(), []
    for p in out:
        if str(p) not in seen:
            seen.add(str(p))
            uniq.append(p)
    return uniq


# --- identity ---------------------------------------------------------------


def _identity(robot_identity, run, bundle):
    """The published :class:`IdentityRule` for this column.

    The structural predicate is the honest one here and it is worth being
    explicit about why. On upstream Webots a robot is a ``Robot``-derived PROTO
    instance; on OmniSim it is a ``Robot`` node the engine built from a URDF
    import. **MuJoCo has neither.** ``mjModel`` is a flat array of bodies,
    joints, geoms and actuators with no notion of a machine, so any predicate
    that claimed to find "the robot" by node type would be inventing a channel
    that does not exist. What MuJoCo *does* have is the structure the tier
    actually cares about: a body free to move relative to the world, carrying
    articulation. That is what is counted, and the count is stated.
    """
    floor = MIN_JOINTS if robot_identity != "any_robot" else 1
    scene_rule = (
        "a body that is a DIRECT CHILD OF THE WORLD BODY and whose subtree "
        "carries at least %d joints. MuJoCo has no robot node type -- an "
        "mjModel is one kinematic tree of bodies with no notion of a machine "
        "-- so 'a robot' is read structurally as an articulated assembly "
        "attached to the scene. The world body itself and every unarticulated "
        "static body (a floor, a plinth: zero joints in its subtree) are "
        "excluded by construction, not by a name filter. The joint floor is "
        "identical to the OmniSim and Webots adapters', so the three identity "
        "rules differ only in the channel carrying the name. LIMIT, stated: a "
        "scene that parents the robot under an intermediate frame body rather "
        "than under the world would not match, and the count would read 0 "
        "rather than 1. Note also that a FREE joint is deliberately NOT "
        "required -- an arm bolted to a bench is a robot, and a welded chassis "
        "is a robot that cannot move, which is a T1.1 failure and not an "
        "empty scene." % floor)

    declared = None
    declaration_rule = (
        "'<robot name=...>' declarations in the source robot description the "
        "run was built from (build.json -> urdf_source). MuJoCo's own MJCF "
        "output keeps the model's name only as the <mujoco model='...'> "
        "attribute, and the deliverable is machine-generated, so the URDF is "
        "where a declaration count means anything")
    if run.build:
        src = run.build.get("urdf_source") or run.build.get("urdf_edited")
        name = run.build.get("robot_name")
        if src and Path(str(src)).is_file():
            try:
                text = Path(str(src)).read_text(encoding="utf-8",
                                                errors="replace")
                declared = len(re.findall(r"<robot\b[^>]*\bname\s*=", text))
            except OSError:
                declared = None
        bundle.adapter_measurements["identity"] = {
            "urdf_source": src, "urdf_robot_name": name,
            "robot_declarations_in_urdf": declared,
        }
    else:
        bundle.notes.append(
            "no build record, so the declaration half of the identity rule is "
            "unanswered (None), not zero.")

    label = "robot" if robot_identity == "any_robot" else robot_identity
    return IdentityRule(
        label=label,
        requirement=("the robot body the task is about"
                     if robot_identity == "any_robot"
                     else "a distinct robot-class body that is the robot the "
                          "task named"),
        scene_rule=scene_rule, declaration_rule=declaration_rule,
        declared_count=declared)


def _robot_class(rec, robot_identity):
    """Is this recorded body the robot? True / False / None.

    ``None`` only when the recorder gave us neither a parent id nor a subtree
    joint count -- an unanswerable question, which the core reports as an
    absent identity witness. A body that IS answerable and simply is not the
    robot returns ``False``.

    **A free joint is not required, and that is load-bearing.** An earlier
    version of this rule keyed on ``mjJNT_FREE``, which made a welded chassis
    -- the exact thing a raw URDF load produces on this simulator -- report as
    *no robot in the scene at all*. That collapses the whole verdict instead of
    failing T1.1 and T1.3, and it turns a physical finding ("it could not
    move") into an instrument failure ("there was nothing to grade"). Caught by
    the ``welded`` negative fixture, which is what negative fixtures are for.
    """
    bid = rec.get("id")
    parent = rec.get("parent_id")
    sub = rec.get("n_joints_subtree")
    if parent is None or sub is None:
        return None
    if bid is not None and int(bid) == 0:
        return False                      # the world body is not a robot
    if int(parent) != 0:
        return False                      # not attached directly to the world
    floor = 1 if robot_identity == "any_robot" else MIN_JOINTS
    return bool(int(sub) >= floor)


# --- bodies -----------------------------------------------------------------


def _body(rec, robot_identity):
    lo = recording.triple(rec.get("aabb_min"))
    hi = recording.triple(rec.get("aabb_max"))
    mass = rec.get("mass_kg")
    sub_mass = rec.get("subtree_mass_kg")
    free = bool(rec.get("has_free_joint"))
    # "dynamic" in the neutral schema means "a mass model is attached and it
    # moves". A MuJoCo body is dynamic exactly when it has a joint chain to the
    # world AND carries mass somewhere in its subtree: a static body has no
    # joints, and a massless frame link (a URDF convention this Husky uses
    # heavily) is not a body a physics assertion can be made about.
    dynamic = None
    if mass is not None or sub_mass is not None:
        moving = bool(rec.get("n_joints") or free)
        dynamic = bool(moving and float(sub_mass or mass or 0.0) > 0.0)
    return Body(
        body_id=_body_id(rec),
        name=str(rec.get("name") or ""),
        kind=str(rec.get("type") or "mjBODY"),
        position=recording.triple(rec.get("position")),
        aabb_min=lo, aabb_max=hi,
        n_joints=_opt_int(rec.get("n_joints")),
        # MuJoCo has no controller process. The behaviour that drives a body is
        # the runner's own control loop, and only the actuated robot has one.
        behaviour=("mujoco_t1_drive (the runner's in-process control loop)"
                   if free else None),
        behaviour_declared=("in-process control loop" if free else "<none>"),
        dynamic=dynamic,
        robot_class=_robot_class(rec, robot_identity),
        identity_evidence="parent_id=%s has_free_joint=%s n_joints=%s "
                          "n_joints_subtree=%s mass=%s kg subtree_mass=%s kg "
                          "(in-process mjModel scan)"
                          % (rec.get("parent_id"), rec.get("has_free_joint"),
                             rec.get("n_joints"), rec.get("n_joints_subtree"),
                             mass, sub_mass))


def _body_id(rec):
    """A stable per-run body id: ``#<mjModel body index>``, else the name.

    MuJoCo body indices are stable for the life of a compiled model and unique
    within it, which is exactly what the core counts for distinctness. Names
    come from the URDF's link names and are unique in a well-formed model, but
    MuJoCo does not require them at all -- an unnamed body is legal, so the
    index is the primary key and the name is the fallback.
    """
    i = rec.get("id")
    if i is not None:
        return "#%s" % i
    return str(rec.get("name") or "")


def _opt_int(v):
    if isinstance(v, bool) or v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


# --- contacts ---------------------------------------------------------------


def _contacts(doc):
    """One :class:`ContactObservation`. Witness counters are never invented."""
    pairs = []
    for c in (doc.get("pairs") or []):
        if not isinstance(c, dict):
            continue
        pairs.append(ContactPair(
            a=c.get("a"), b=c.get("b"),
            a_robot=bool(c.get("a_robot")), b_robot=bool(c.get("b_robot")),
            point=recording.triple(c.get("point")), step=c.get("step")))
    return ContactObservation(
        pairs=pairs,
        steps=int(doc.get("steps") or 0),
        window_s=doc.get("window_s"),
        supported=bool(doc.get("supported")),
        # ``or None`` is deliberately NOT used: 0 is a real count and must
        # survive, while a missing key must stay None (adapter rule 2).
        total_observed=_opt_int(doc.get("total_observed")),
        distinct_named=_opt_int(doc.get("distinct_named")),
        source=doc.get("source") or _CONTACT_SOURCE,
        error=doc.get("error"))


# --- the ladder's support channel -------------------------------------------


def support_observation(phase_b=None, *, run=None, run_dir=None):
    """Robot-to-surface contacts over the motion window (ladder channel).

    This is the channel neither shipped column has (see
    ``ladder.graders.ladder_evidence``): contacts naming the robot **and the
    non-robot body it is riding on**, sampled across the whole travel window
    rather than only at t=0, and timestamped on the same clock as the pose
    series so the core can ask whether the robot was moving when the contact
    was seen.

    ``surface_is_robot`` is answered from the recorder's subtree membership
    flag and is never guessed: an adapter that cannot distinguish "resting on
    the ground" from "leaning on another robot" must supply ``None`` and let
    the clause report itself vacuous.
    """
    if run is None:
        if isinstance(phase_b, recording.MujocoRun):
            run = phase_b
        elif isinstance(phase_b, (str, Path)) or run_dir:
            run = recording.read_run(phase_b or run_dir)
        else:
            return SupportContactObservation(
                error="no MuJoCo run was supplied, so no support-contact "
                      "observation could be read")
    doc = run.contacts
    if doc is None:
        return SupportContactObservation(
            error=(run.errors.get("contacts")
                   or run.missing.get("contacts")
                   or "the run recorded no contact scan"))

    pairs = []
    for c in (doc.get("pairs") or []):
        if not isinstance(c, dict):
            continue
        a_robot, b_robot = bool(c.get("a_robot")), bool(c.get("b_robot"))
        if a_robot == b_robot:
            # robot-robot (self-contact) or surface-surface: neither is the
            # robot being carried by something that is not a robot.
            continue
        robot_side, surface_side = ((c.get("a"), c.get("b")) if a_robot
                                    else (c.get("b"), c.get("a")))
        pairs.append(SupportContact(
            robot_body=robot_side, surface_body=surface_side,
            surface_is_robot=False,
            point=recording.triple(c.get("point")),
            t_s=c.get("t_s"), step=c.get("step")))
    return SupportContactObservation(
        pairs=pairs,
        supported=bool(doc.get("supported")),
        total_observed=_opt_int(doc.get("total_observed")),
        distinct_named=_opt_int(doc.get("distinct_named")),
        steps_sampled=int(doc.get("steps") or 0),
        window_s=doc.get("window_s"),
        source=(_CONTACT_SOURCE + "; pair records emitted once per distinct "
                "body pair every %s steps, each carrying the simulated time it "
                "was seen. 'not a robot' is the recorder's own subtree "
                "membership flag (mjModel.body_parentid walked up to the "
                "free-jointed base), never a name heuristic"
                % doc.get("emit_stride_steps", "?")),
        error=doc.get("error"))


# --- T2's channels -----------------------------------------------------------
#
# ``ladder.graders.ladder_evidence`` gap 3: five questions T2 asks that the
# frozen bundle has no field for, plus the three the ladder derives beside
# them. Every one is answered here from what
# ``ladder.adapters.mujoco.runner_t2`` wrote, and every one comes back with the
# citation that says which MuJoCo call produced it -- because a channel with no
# provenance is a number somebody typed, and the shim reports an unanswered
# channel as OUR defect rather than the simulator's.


T2_CHANNEL_KEYS = ("object_pose", "end_effector", "container", "grip",
                   "object_physics", "world", "support_surfaces",
                   "object_aabb")


def t2_channels(phase_b=None, *, roles=None, run=None, run_dir=None):
    """T2's channels for one MuJoCo run, as ``{key: channel}``.

    The hook ``ladder.adapters.build_t2_evidence`` looks for. Returns ``{}``
    when there is no run to read -- which makes every channel fall back to
    what the frozen bundle can derive and puts the gap note in the verdict,
    rather than manufacturing an empty channel that would read as
    "measured, and there was nothing there".

    ``roles`` is accepted and **not used to choose bodies**: the sampler
    already recorded which body it sampled and writes that name down, and the
    core compares that name against the task file itself
    (``t2_core.series_naming_problems``). An adapter that re-resolved the role
    here could quietly hand the core a different body than the one the run
    recorded.
    """
    run = _t2_run(phase_b, run, run_dir)
    if run is None or not run.t2:
        return {}
    doc = run.t2
    out = {}

    t = doc.get("t_s")
    for key, blob in (("object_pose", doc.get("object")),
                      ("end_effector", doc.get("end_effector"))):
        series = _pose_series(t, blob)
        if series is not None:
            out[key] = series

    c = doc.get("container") or {}
    if c:
        out["container"] = ContainerGeometry(
            body=str(c.get("body") or ""),
            aabb_min=recording.triple(c.get("aabb_min")),
            aabb_max=recording.triple(c.get("aabb_max")),
            rim_z=(float(c["rim_z"]) if isinstance(c.get("rim_z"),
                                                   (int, float)) else None),
            rim_rule=str(c.get("rim_rule") or ""),
            t_s=c.get("t_s"),
            source=("the frozen t=0 scan of the container's whole KINEMATIC "
                    "SUBTREE (%s): the bin's four walls are child bodies, so "
                    "the named body's own box would be its floor slab and "
                    "nothing else. World AABBs are composed from "
                    "mjModel.geom_aabb carried through mjData.geom_xpos/"
                    "geom_xmat"
                    % ", ".join(c.get("subtree_bodies") or ["the body alone"])),
            error=c.get("error"))

    surfaces = []
    for s in (doc.get("support_surfaces") or []):
        lo, hi = recording.triple(s.get("aabb_min")), \
            recording.triple(s.get("aabb_max"))
        if lo is None or hi is None:
            continue
        surfaces.append(SupportSurface(
            body=str(s.get("body") or ""), aabb_min=lo, aabb_max=hi,
            static=(bool(s["static"]) if isinstance(s.get("static"), bool)
                    else None),
            source=str(doc.get("support_surfaces_source") or "")))
    if surfaces:
        out["support_surfaces"] = surfaces

    p = doc.get("object_physics") or {}
    if p:
        out["object_physics"] = BodyPhysics(
            body=str(p.get("body") or ""),
            mass_kg=(float(p["mass_kg"]) if isinstance(p.get("mass_kg"),
                                                       (int, float)) else None),
            dynamic=(bool(p["dynamic"]) if isinstance(p.get("dynamic"), bool)
                     else None),
            source=("mjModel.body_mass for the named body, read off the "
                    "compiled model that drove this run. 'dynamic' is "
                    "mjModel.body_weldid != 0 (the body is NOT in the world's "
                    "weld group, so it can move) AND mass > 0"),
            error=p.get("error"))

    w = doc.get("world") or {}
    if w:
        out["world"] = WorldPhysics(
            gravity_mps2=(float(w["gravity_mps2"])
                          if isinstance(w.get("gravity_mps2"), (int, float))
                          else None),
            gravity_vec=recording.triple(w.get("gravity_vec_mps2")),
            source=str(w.get("source") or ""), error=w.get("error"))

    aabb = doc.get("object_aabb")
    if aabb:
        lo, hi = recording.triple(aabb.get("min")), recording.triple(
            aabb.get("max"))
        if lo is not None and hi is not None:
            out["object_aabb"] = (lo, hi)

    g = doc.get("grip") or {}
    if g:
        out["grip"] = GripObservation(
            mechanism=str(g.get("mechanism") or "unknown"),
            mechanism_source=str(g.get("source") or ""),
            contacts=[GripContact(holder_body=x.get("holder_body"),
                                  held_body=x.get("held_body"),
                                  point=recording.triple(x.get("point")),
                                  t_s=x.get("t_s"), step=x.get("step"))
                      for x in (g.get("contacts") or [])
                      if isinstance(x, dict)],
            attachment=(bool(g["attachment"])
                        if isinstance(g.get("attachment"), bool) else None),
            supported=bool(g.get("supported")),
            total_observed=_opt_int(g.get("total_observed")),
            distinct_named=_opt_int(g.get("distinct_named")),
            steps_sampled=int(g.get("steps") or 0),
            window_s=g.get("window_s"),
            source=str(g.get("source") or ""), error=g.get("error"))
    _ = roles
    return out


def run_standalone(deliverable, run_dir, **kw):
    """Phase B for T2 on this column -- the hook ``t2.run_and_grade`` calls.

    A thin forwarder to :func:`ladder.adapters.mujoco.runner_t2.run_standalone`,
    imported lazily so reading an existing run never drags in a launcher.

    **This name is T2's; T3 has its own** (:func:`t3_run_standalone`), and
    since 2026-08-02 ``ladder.graders.t3`` prefers the tier-specific hook.
    outside this work's scope, so the gap is named here and in
    ``BRINGUP_T3.md`` §7 rather than papered over; ``run_t3.py`` calls the T3
    runner directly and is unaffected.
    """
    from ladder.adapters.mujoco import runner_t2
    return runner_t2.run_standalone(deliverable, run_dir, **kw)


def t3_run_standalone(deliverable, run_dir, **kw):
    """Phase B for **T3** here. See :func:`run_standalone`'s warning."""
    from ladder.adapters.mujoco import runner_t3
    return runner_t3.run_standalone(deliverable, run_dir, **kw)


# --- T3's channels -----------------------------------------------------------
#
# ``ladder.graders.t3_evidence`` lists eight questions T3 asks that the frozen
# bundle has no field for. Every one is answered here from what
# ``ladder.adapters.mujoco.runner_t3`` wrote, and every one comes back with the
# citation naming the MuJoCo call that produced it -- because a channel with no
# provenance is a number somebody typed, and the shim reports an unanswered
# channel as OUR defect rather than the simulator's.


T3_CHANNEL_KEYS = ("base_pose", "standing", "gait", "support", "arena",
                   "base_physics", "world", "controller")


def t3_channels(phase_b=None, *, surface=None, run=None, run_dir=None):
    """T3's channels for one MuJoCo run, as ``{key: channel}``.

    The hook ``ladder.graders.t3.build_evidence`` looks for. Returns ``{}``
    when there is no run to read -- which makes every channel fall back to what
    the frozen bundle can derive and puts the gap note in the verdict, rather
    than manufacturing an empty channel that would read as *"measured, and
    there was nothing there"*.

    ``surface`` is accepted and **not used to choose bodies**: the sampler
    already recorded which bodies it bounded and which contacts it saw, and the
    core classifies them against the task file itself
    (:func:`t3_evidence.classify_contacts`), taking the stricter of the two
    readings on every disagreement. An adapter that re-resolved the ground here
    could quietly hand the core its own opinion of what the floor is, which is
    the one thing T3.4 cannot survive.
    """
    from ladder.graders import t3_evidence as t3ev

    run = _t2_run(phase_b, run, run_dir)
    if run is None or not run.t3:
        return {}
    doc = run.t3
    out = {}

    t = doc.get("t_s")
    series = _pose_series(t, doc.get("base_pose"))
    if series is not None:
        out["base_pose"] = series

    s = doc.get("standing") or {}
    if s:
        z = s.get("z_m")
        out["standing"] = t3ev.StandingHeight(
            z_m=(float(z) if isinstance(z, (int, float)) else None),
            body=str(s.get("body") or ""), source=str(s.get("source") or ""),
            error=s.get("error"))

    g = doc.get("gait") or {}
    if g:
        out["gait"] = t3ev.GaitContactObservation(
            contacts=[t3ev.GroundContact(
                robot_body=c.get("robot_body"), other_body=c.get("other_body"),
                other_is_ground=(bool(c["other_is_ground"])
                                 if isinstance(c.get("other_is_ground"), bool)
                                 else None),
                other_is_robot=(bool(c["other_is_robot"])
                                if isinstance(c.get("other_is_robot"), bool)
                                else None),
                point=recording.triple(c.get("point")),
                t_s=c.get("t_s"), step=c.get("step"))
                for c in (g.get("contacts") or []) if isinstance(c, dict)],
            sample_times=(np.asarray(g["sample_times"], dtype=float)
                          if g.get("sample_times") else None),
            supported=bool(g.get("supported")),
            total_observed=_opt_int(g.get("total_observed")),
            distinct_named=_opt_int(g.get("distinct_named")),
            steps_sampled=int(g.get("steps") or 0),
            window_s=g.get("window_s"),
            source=str(g.get("source") or ""), error=g.get("error"))

    sup = doc.get("support") or {}
    if sup:
        out["support"] = t3ev.AppliedSupport(
            attested=(bool(sup["attested"])
                      if isinstance(sup.get("attested"), bool) else None),
            t=(np.asarray(sup["t"], dtype=float) if sup.get("t") else None),
            force=(np.asarray(sup["force"], dtype=float)
                   if sup.get("force") else None),
            torque=(np.asarray(sup["torque"], dtype=float)
                    if sup.get("torque") else None),
            source=str(sup.get("source") or ""), error=sup.get("error"))

    a = doc.get("arena") or {}
    if a:
        out["arena"] = t3ev.ArenaBounds(
            aabb_min=recording.triple(a.get("aabb_min")),
            aabb_max=recording.triple(a.get("aabb_max")),
            boundary_bodies=list(a.get("boundary_bodies") or []),
            source=str(a.get("source") or ""), error=a.get("error"))

    p = doc.get("base_physics") or {}
    if p:
        out["base_physics"] = BodyPhysics(
            body=str(p.get("body") or ""),
            mass_kg=(float(p["mass_kg"]) if isinstance(p.get("mass_kg"),
                                                       (int, float)) else None),
            dynamic=(bool(p["dynamic"]) if isinstance(p.get("dynamic"), bool)
                     else None),
            source=str(p.get("source") or ""), error=p.get("error"))

    w = doc.get("world") or {}
    if w:
        out["world"] = WorldPhysics(
            gravity_mps2=(float(w["gravity_mps2"])
                          if isinstance(w.get("gravity_mps2"), (int, float))
                          else None),
            gravity_vec=recording.triple(w.get("gravity_vec_mps2")),
            source=str(w.get("source") or ""), error=w.get("error"))

    c = doc.get("controller") or {}
    if c:
        out["controller"] = t3ev.ControllerLoad(
            declared_method=str(c.get("declared_method") or "unknown"),
            loaded=(bool(c["loaded"]) if isinstance(c.get("loaded"), bool)
                    else None),
            evidence=str(c.get("evidence") or ""),
            identity=str(c.get("identity") or ""),
            source=str(c.get("source") or ""), error=c.get("error"))
    _ = surface
    return out


# --- T4's channels -----------------------------------------------------------
#
# T4's evidence contract IS T3's -- ``ladder.graders.t4_evidence`` subclasses
# T3's evidence and adds no field, because the tier replaces an ASSERTION and
# not a channel list. So this hook exists for two reasons and neither of them
# is a new channel: it reads the file ``runner_t4`` wrote rather than the one
# ``runner_t3`` writes, and it can serve the applied-wrench channel in TWO
# readings -- the honest total, and the xfrc-only total a column that never
# thought about constraint rigs would have published. See
# ``support_reading`` below.


T4_CHANNEL_KEYS = T3_CHANNEL_KEYS

SUPPORT_READINGS = ("total", "xfrc_only")


def t4_run_standalone(deliverable, run_dir, **kw):
    """Phase B for **T4** here -- the hook ``t4.run_and_grade`` calls."""
    from ladder.adapters.mujoco import runner_t4
    return runner_t4.run_standalone(deliverable, run_dir, **kw)


def t4_channels(phase_b=None, *, surface=None, run=None, run_dir=None,
                support_reading="total"):
    """T4's channels for one MuJoCo run, as ``{key: channel}``.

    The hook ``ladder.graders.t4.build_evidence`` looks for first. Returns
    ``{}`` when there is no ``t4.json`` to read -- which makes every channel
    fall back to what the frozen bundle can derive and puts the gap note in the
    verdict, rather than manufacturing an empty channel that would read as
    *"measured, and there was nothing there"*.

    ``support_reading`` selects which applied-wrench reading is served, and it
    exists to make a **finding about the tier** reproducible rather than to
    give a caller a choice:

    ``"total"`` (the default, and this column's attestation)
        the explicitly applied wrench **plus the equality-constraint reaction
        on the base**, which is what ``LADDER_REQUIRED_EVIDENCE_T4`` requires
        of a column and what stops a welded robot being published as
        ``T4-unsupported``.
    ``"xfrc_only"``
        ``mjData.xfrc_applied`` alone. **Not an attestation** -- it is what a
        column that never counted constraint forces would have published, and
        grading the same recording under both is what turns the task file's
        open question into a printed pair of cells. See ``BRINGUP_T4.md`` §6.

    On a scene with no equality constraints the two are the same array and the
    run says so (``xfrc_only_identical_to_total``).
    """
    from ladder.graders import t4_evidence as t4ev

    run = _t2_run(phase_b, run, run_dir)
    if run is None or not getattr(run, "t4", None):
        return {}
    doc = run.t4
    out = _channels_from_doc(doc, t4ev)

    key = ("support" if str(support_reading) != "xfrc_only"
           else "support_xfrc_only")
    sup = doc.get(key) or {}
    if key == "support_xfrc_only" and sup.get("identical_to_the_attested_total"):
        # Nothing to serve separately: the two readings ARE the same series and
        # the run recorded that instead of a duplicate copy of it.
        sup = dict(doc.get("support") or {},
                   source=str(sup.get("source") or "")
                   + " -- identical to the attested total on this run, because "
                     "the scene carries no equality constraint on the base")
    if sup:
        out["support"] = t4ev.AppliedSupport(
            attested=(bool(sup["attested"])
                      if isinstance(sup.get("attested"), bool) else None),
            t=(np.asarray(sup["t"], dtype=float) if sup.get("t") else None),
            force=(np.asarray(sup["force"], dtype=float)
                   if sup.get("force") else None),
            torque=(np.asarray(sup["torque"], dtype=float)
                    if sup.get("torque") else None),
            source=str(sup.get("source") or ""), error=sup.get("error"))
    _ = surface
    return out


def _channels_from_doc(doc, ev):
    """The seven channels T3 and T4 share, from either rung's own document.

    The support channel is deliberately NOT here: it is the one T4 reads twice.
    """
    out = {}
    series = _pose_series(doc.get("t_s"), doc.get("base_pose"))
    if series is not None:
        out["base_pose"] = series

    s = doc.get("standing") or {}
    if s:
        z = s.get("z_m")
        out["standing"] = ev.StandingHeight(
            z_m=(float(z) if isinstance(z, (int, float)) else None),
            body=str(s.get("body") or ""), source=str(s.get("source") or ""),
            error=s.get("error"))

    g = doc.get("gait") or {}
    if g:
        out["gait"] = ev.GaitContactObservation(
            contacts=[ev.GroundContact(
                robot_body=c.get("robot_body"), other_body=c.get("other_body"),
                other_is_ground=(bool(c["other_is_ground"])
                                 if isinstance(c.get("other_is_ground"), bool)
                                 else None),
                other_is_robot=(bool(c["other_is_robot"])
                                if isinstance(c.get("other_is_robot"), bool)
                                else None),
                point=recording.triple(c.get("point")),
                t_s=c.get("t_s"), step=c.get("step"))
                for c in (g.get("contacts") or []) if isinstance(c, dict)],
            sample_times=(np.asarray(g["sample_times"], dtype=float)
                          if g.get("sample_times") else None),
            supported=bool(g.get("supported")),
            total_observed=_opt_int(g.get("total_observed")),
            distinct_named=_opt_int(g.get("distinct_named")),
            steps_sampled=int(g.get("steps") or 0),
            window_s=g.get("window_s"),
            source=str(g.get("source") or ""), error=g.get("error"))

    a = doc.get("arena") or {}
    if a:
        out["arena"] = ev.ArenaBounds(
            aabb_min=recording.triple(a.get("aabb_min")),
            aabb_max=recording.triple(a.get("aabb_max")),
            boundary_bodies=list(a.get("boundary_bodies") or []),
            source=str(a.get("source") or ""), error=a.get("error"))

    p = doc.get("base_physics") or {}
    if p:
        out["base_physics"] = BodyPhysics(
            body=str(p.get("body") or ""),
            mass_kg=(float(p["mass_kg"]) if isinstance(p.get("mass_kg"),
                                                       (int, float)) else None),
            dynamic=(bool(p["dynamic"]) if isinstance(p.get("dynamic"), bool)
                     else None),
            source=str(p.get("source") or ""), error=p.get("error"))

    w = doc.get("world") or {}
    if w:
        out["world"] = WorldPhysics(
            gravity_mps2=(float(w["gravity_mps2"])
                          if isinstance(w.get("gravity_mps2"), (int, float))
                          else None),
            gravity_vec=recording.triple(w.get("gravity_vec_mps2")),
            source=str(w.get("source") or ""), error=w.get("error"))

    c = doc.get("controller") or {}
    if c:
        out["controller"] = ev.ControllerLoad(
            declared_method=str(c.get("declared_method") or "unknown"),
            loaded=(bool(c["loaded"]) if isinstance(c.get("loaded"), bool)
                    else None),
            evidence=str(c.get("evidence") or ""),
            identity=str(c.get("identity") or ""),
            source=str(c.get("source") or ""), error=c.get("error"))
    return out


def _t2_run(phase_b, run, run_dir):
    """The :class:`MujocoRun` behind whatever the caller passed. Never raises.

    Shared by :func:`t2_channels`, :func:`t3_channels` and
    :func:`t4_channels`: locating the run directory is not rung-specific, and
    copies of the search order would be places for a re-grade to stop finding
    artifacts that are on disk.
    """
    if run is not None:
        return run
    if isinstance(phase_b, recording.MujocoRun):
        return phase_b
    for cand in (getattr(phase_b, "run_dir", None),
                 phase_b if isinstance(phase_b, (str, Path)) else None,
                 run_dir):
        if cand:
            got = recording.read_run(cand)
            if (got.t2 or got.t3 or getattr(got, "t4", None)
                    or got.any_evidence):
                return got
    return None


def _pose_series(t, blob):
    """One :class:`PoseSeries` from the sampler's arrays. ``None`` if absent.

    A ragged series is TRUNCATED to the shortest common length and the
    truncation published in the citation -- never padded, never interpolated,
    for the reason ``REQUIRED_EVIDENCE`` states about resampling.
    """
    if not isinstance(blob, dict):
        return None
    xyz = blob.get("xyz")
    if not isinstance(t, list) or not isinstance(xyz, list) or not xyz:
        return PoseSeries(body=str((blob or {}).get("body") or ""),
                          error="the sampler recorded no samples for this body")
    n = min(len(t), len(xyz))
    rot = blob.get("rot")
    note = ""
    if rot and len(rot) < n:
        n = min(n, len(rot))
    if n < min(len(t), len(xyz)):
        note = (" -- TRUNCATED to %d samples: the clock, the positions and the "
                "rotations disagreed on their length" % n)
    try:
        ts = np.asarray(t[:n], dtype=float)
        pos = np.asarray(xyz[:n], dtype=float)
        mats = (np.asarray(rot[:n], dtype=float).reshape(n, 3, 3)
                if rot else None)
    except (TypeError, ValueError) as exc:
        return PoseSeries(body=str(blob.get("body") or ""),
                          error="the recorded series would not parse: %r"
                                % (exc,))
    return PoseSeries(body=str(blob.get("body") or ""), t=ts, xyz=pos,
                      rot=mats, source=str(blob.get("source") or "") + note)


# --- trajectory -------------------------------------------------------------


def _trajectory(arr, bundle, run):
    """The pose series, with the index invariant honoured against the roster.

    ``Trajectory`` promises row *i* of ``xyz`` is ``body_ids[i]`` and that a
    core which looks a body up by name in a frozen inventory and indexes
    ``xyz`` with that position gets the same body. On this column the recorder
    writes both files from the same ``mjModel`` in body-index order, so they do
    agree -- but "they should agree" is not evidence, so the rows are permuted
    into roster order by name and the failure to do so is published.
    """
    xyz = arr.xyz
    names = list(arr.names)
    body_ids = list(names)
    roster = bundle.roster

    if roster.bodies:
        idx, ok = [], True
        for nm in names:
            j = roster.index_of_name(nm) if nm else None
            if j is None or j in idx:
                ok = False
                break
            idx.append(j)
        if ok and len(idx) == len(roster.bodies):
            order = sorted(range(len(idx)), key=lambda k: idx[k])
            xyz = xyz[order]
            names = [names[k] for k in order]
            body_ids = [roster.bodies[idx[k]].body_id for k in order]
        else:
            bundle.notes.append(
                "trajectory rows could not be aligned to the roster by name "
                "(%d trajectory bodies, %d roster bodies); the bundle's "
                "body_ids are the recorder's trajectory names and a per-body "
                "index is NOT valid across the two inventories."
                % (len(names), len(roster.bodies)))

    src = _TRAJ_SOURCE
    if arr.stride and arr.stride > 1:
        src += (" -- sampled every %d physics steps, not every step"
                % arr.stride)
        bundle.notes.append(
            "the pose series is DOWNSAMPLED: one sample every %d physics "
            "steps (%.4g ms against a %.4g ms timestep). It is not the "
            "one-sample-per-physics-step series REQUIRED_EVIDENCE asks for, "
            "and integrated path length over a stride is a LOWER bound -- so "
            "the path-length floor is being applied to a different quantity "
            "than on a per-step arm. Direction of the bias: against this run."
            % (arr.stride, (arr.dt_s or 0.0) * 1000.0,
               (arr.physics_dt_s or 0.0) * 1000.0))
    if run.files.get("trajectory"):
        src += " (%s)" % Path(run.files["trajectory"]).name
    complete = arr.complete
    if complete is None:
        complete = bool((run.completion or {}).get("complete"))
    return Trajectory(body_ids=body_ids, t=arr.t, xyz=xyz, vel=None,
                      dt_s=arr.dt_s, recorded_s=arr.recorded_s,
                      complete=bool(complete), source=src)


# --- process ----------------------------------------------------------------


def _error_lines(run):
    """MuJoCo's error-class evidence: warning counters, then console text.

    See the module docstring for why the counters are here. Each non-zero
    physics counter becomes one line naming the counter and its count, so a
    reader of the verdict sees *which* thing went wrong rather than a number.
    """
    out = []
    for name, n in sorted((run.completion or {}).get("warnings", {}).items()):
        if name in PHYSICS_WARNINGS and n:
            out.append("%s raised %s time(s) during the run "
                       "(mjData.warning counter)" % (name, n))
    for ln in (run.console_text or "").splitlines():
        if _ERROR_LINE.match(ln):
            out.append(ln.strip())
    return out


def _process(run):
    """:class:`ProcessFacts`, or None when nothing about the process is known."""
    pdoc = run.process or {}
    cdoc = run.completion or {}
    have_console = run.console_text is not None
    if not pdoc and not cdoc and not have_console:
        return None

    errs = _error_lines(run)
    steps = cdoc.get("steps")
    driver_completed = None
    if cdoc:
        driver_completed = bool(steps and float(cdoc.get("recorded_s") or 0) > 0)

    saved_ok = run.saved_model_nonempty
    if saved_ok:
        reached, why = True, _SAVED_MODEL_FINALIZE
    elif driver_completed:
        reached = True
        why = ("no engine-authored saved model, but the recorder advanced "
               "mjData.time to %s s over %s steps, which mj_step cannot do on "
               "anything but a compiled model. This is the WEAKER of the two "
               "witnesses and is named as such"
               % (_fmt(cdoc.get("sim_time_at_exit_s")), steps))
    elif driver_completed is False or saved_ok is False:
        reached = False
        why = ("no finalize evidence: %s%s"
               % ("the recorder advanced no simulated time"
                  if driver_completed is False else "no recorder record",
                  "; the saved-model file is empty"
                  if saved_ok is False else ""))
    else:
        reached, why = None, ("neither an engine-authored saved model nor a "
                              "recorder completion record: 'did a model "
                              "compile and step' is unanswered, not answered "
                              "no")

    writes = cdoc.get("ctrl_writes")
    starts = None
    if writes is not None:
        starts = ({"mujoco_t1_drive": 1} if int(writes) > 0 else {})

    return ProcessFacts(
        exit_code=_opt_int(pdoc.get("exit_code")),
        timed_out=bool(pdoc.get("timed_out")),
        error_lines=errs,
        log_available=(have_console or bool(cdoc)),
        log_source=(_LOG_SOURCE if (have_console or cdoc)
                    else _NO_LOG_SOURCE),
        behaviour_starts=starts,
        start_source=(_STARTS_SOURCE if starts is not None else ""),
        driver_completed=driver_completed,
        reached_finalize=reached, finalize_evidence=why,
        wall_s=pdoc.get("wall_s"),
        attempts_used=int(pdoc.get("attempts_used") or 1))


def _fmt(v):
    return "%g" % v if isinstance(v, (int, float)) else "?"


# --- attribution ------------------------------------------------------------


def _process_evidence(run):
    """Did a MuJoCo run demonstrably happen, as opposed to leaving numbers?

    A ``trajectory.json`` on its own is a bag of pose samples: it could have
    come from anywhere. An engine-authored saved model, a completion record
    carrying the compiled model's shape, an exit code or a captured console
    could not.
    """
    return bool(run.process or run.completion or run.saved_model_path
                or run.console_text is not None)


def _attribution(run):
    """Which MuJoCo drove this run, and how it was configured.

    Unlike upstream Webots -- where the attribution is a structural invariant
    about the product -- this is a genuine per-run *reading*: MuJoCo's solver,
    integrator, friction cone and timestep are all model fields, and two runs
    of the same library can differ in every one of them. So a run with no
    completion record has no attribution, and A1.10's rule applies: an
    unattributed physics result is INVALID rather than FAILED.
    """
    if not _process_evidence(run):
        return None
    eng = ((run.completion or {}).get("engine") or {})
    version = (eng.get("version")
               or (run.process or {}).get("mujoco_version") or "")
    if not eng and not version:
        return None
    source = _ATTRIBUTION_SOURCE
    if version and MUJOCO_RELEASE not in str(version):
        source += (" WARNING: this run reports MuJoCo %r while the citations in "
                   "this adapter are written against %s; the compiler defaults "
                   "this column depends on have changed between releases "
                   "before." % (version, MUJOCO_RELEASE))
    extra = {k: v for k, v in eng.items()
             if k in ("integrator", "cone", "timestep_s", "gravity",
                      "iterations", "version_string", "module", "executed_on",
                      "solver_note")}
    extra["mujoco_version"] = version or None
    return EngineAttribution(backend="mujoco",
                             solver=eng.get("solver"),
                             degraded=False, source=source, extra=extra)


# --- frozen -----------------------------------------------------------------


def _frozen(rdoc):
    """``frozen`` for the inventories -- never upgraded, only downgraded."""
    frozen = rdoc.get("frozen")
    frozen = None if frozen is None else bool(frozen)
    if frozen is None:
        return None, ("the recorder did not state whether its scan was taken "
                      "with the world frozen, so 'frozen' is unanswered rather "
                      "than assumed; no 'at t=0' assertion can rest on it.")
    return frozen, None


__all__ = ["ADAPTER", "LABELS", "MIN_JOINTS", "MUJOCO_RELEASE",
           "PHYSICS_WARNINGS", "SIM", "SUPPORT_READINGS", "T2_CHANNEL_KEYS",
           "T3_CHANNEL_KEYS", "T4_CHANNEL_KEYS", "WARNING_NOTE",
           "build_bundle", "run_standalone", "support_observation",
           "t2_channels", "t3_channels", "t3_run_standalone", "t4_channels",
           "t4_run_standalone"]


def l2_run_standalone(deliverable, run_dir, **kw):
    """L2's phase B on this column -- see ``ladder.adapters.mujoco.runner_l2``.

    Neither sibling fits: ``runner_t2`` re-runs a deliverable of the right
    shape but records a manipulation scene's channels, and ``run_t1`` records
    the right channel but drives with this column's own controller instead of
    re-running the agent's. L2 needs somebody else's driver and one body's
    pose, so it has its own runner, writing the same wide-format CSV the other
    column's recorder writes.
    """
    from ladder.adapters.mujoco import runner_l2
    return runner_l2.run_standalone(deliverable, run_dir, **kw)


def p1_run_standalone(deliverable, run_dir, **kw):
    """P1's phase B on this column -- see ``runner_l2``, which already
    re-runs somebody else's scene+driver and records one body's pose into the
    shared wide-format CSV. P1 needs precisely that."""
    from ladder.adapters.mujoco import runner_l2
    return runner_l2.run_standalone(deliverable, run_dir, **kw)
