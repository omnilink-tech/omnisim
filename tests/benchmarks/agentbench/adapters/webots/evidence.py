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

"""Upstream **Webots R2025a** -> :class:`EvidenceBundle`. The control-arm adapter.

Why this file exists: the graders were split into a simulator-neutral physical
core plus per-simulator adapters precisely so that *the same grader* can score a
Webots run and an OmniSim run. Without this module ``adapters.resolve("webots")``
raises and the cross-simulator comparison -- Phase W of
``docs/developer/agent-edge-validation-plan.md``, the cheapest decisive
experiment the programme has -- cannot be graded at all.

It reads only what an upstream run can actually produce (see
:mod:`agentbench.adapters.webots.recording` for the artifact contract) and maps
it into the neutral schema. It answers the two questions the core is forbidden to
ask -- *"is this body the robot the task named"* and *"which engine drove this
run"* -- and it publishes the rule it used for each.

What upstream gives us, and what it does not
--------------------------------------------

======================  ====================================================
neutral field           the upstream channel, and its limit
======================  ====================================================
``identity``            PROTO instantiation. Upstream does **not** expand
                        PROTOs, so the model name is in the live scene
                        (``getTypeName()``) *and* in the ``.wbt`` -- the
                        opposite of OmniSim, where a URDF import is expanded at
                        parse time and the name survives only in the file.
                        ⚠ R2025a ships **no Husky** (zero matches in all
                        15,404 paths of the tag), so the flagship predicate is
                        only answerable against a pre-converted asset or an
                        explicitly declared analogue -- see ``robot_proto``.
``roster`` / ``t0``     an in-world ``Supervisor`` recorder. Upstream has no
                        HTTP surface, no scene-tree query and no hot reload
                        (``webots-control-baseline.md`` §6), so a Supervisor is
                        the only scene reader there is. It is
                        ``synchronization TRUE``, which means it CAN honestly
                        freeze t=0 -- but it has no bounds API, so a world-space
                        AABB only exists if the recorder computed one itself.
``contacts``            ``Supervisor`` ``Node.getContactPoints()``. Naming
                        *both* participants needs world-point pairing across
                        queries, which is exactly the pipeline that produced
                        OmniSim's ``(id, id)`` self-pair bug -- so the two
                        vacuity counters are mandatory here and an absent
                        contact file stays ``None``.
``trajectory``          the recorder's own per-step sampling, in simulated
                        seconds and metres. NOT polled over a network and not
                        resampled.
``process``             exit code + captured stdout/stderr. ⚠ upstream writes
                        **no log file**; there is no ``$OMNISIM_LOG_PATH``
                        equivalent to read after the fact, so anything not
                        captured at launch is gone.
``reached_finalize``    two channels, either sufficient: the recorder's own
                        ``simulationQuit()`` record, or a **non-empty**
                        ``--log-performance`` file (upstream flushes it only in
                        ``worldClosed()``, ``WbPerformanceLog.cpp:123-150``, so
                        a killed run leaves it empty). Upstream has no
                        ``--duration`` and no auto-exit at all: a run ends when
                        a Supervisor quits it or when something kills it.
``attribution``         a documented structural invariant: there is exactly one
                        physics engine upstream. Asserted with a citation, and
                        only for a run that left evidence it happened.
======================  ====================================================

Two places where this adapter is deliberately NOT byte-identical to the OmniSim
one, and why each is the honest choice:

``_NO_BEHAVIOUR`` includes ``"void"``
    Required for correctness, in the direction that makes the control arm harder
    rather than easier: ``"void"`` is upstream's own no-controller sentinel, and
    counting it as a controller would report ten controlled robots in a scene
    where nothing moves. An asymmetry that flattered Webots would be just as
    dishonest as one that flattered us.

``_error_lines`` stays ``ERROR:``-only
    A free choice, and it is made **in the control arm's favour**: both engines
    descend from the same ``WbLog`` and both also write ``FATAL:``, which neither
    adapter counts. Widening it here alone would make A1.9 stricter on upstream
    than on us, and a comparison whose thresholds lean towards our own simulator
    is not evidence of anything. ``FATAL:`` lines are recorded and noted instead.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from agentbench.adapters.webots import recording, worldtext  # noqa: E402
from agentbench.common import worldtext as common_wt  # noqa: E402
from agentbench.graders.evidence import (  # noqa: E402
    Body, BodyInventory, ContactObservation, ContactPair, EngineAttribution,
    EvidenceBundle, IdentityRule, ProcessFacts, Trajectory)

SIM = "webots"
ADAPTER = "agentbench.adapters.webots.evidence"

# The reference release this adapter's citations are written against.
WEBOTS_RELEASE = "R2025a"

# Kept identical to the OmniSim adapter's floor on purpose: the two identity
# rules must differ ONLY in the channel that carries the model name, so that a
# cross-sim reader can see the identity question is the same question.
MIN_JOINTS = 4

# ``getBaseTypeName()`` on a Robot-derived PROTO instance returns this, which is
# how a scene body is recognised as a robot without knowing the PROTO.
ROBOT_BASE_TYPE = "Robot"

# PROTO type names that satisfy each identity predicate out of the box.
#
# ⚠ ``husky`` is EMPTY-BY-DEFAULT and that is the finding, not an omission:
# upstream R2025a contains no Husky (nor jackal / warthog / ridgeback) -- all
# 15,404 paths of the tag were checked (webots-control-baseline.md sec. 4). Our
# husky_description/ arrived with OUR URDF importer. A Webots run can therefore
# only satisfy the flagship predicate with (a) a Husky PROTO produced by
# upstream's own urdf2webots and published alongside the benchmark, which this
# set accepts by name, or (b) an analogue declared by the runner via
# ``robot_proto=`` -- which is recorded in the rule and in a note, never
# silently.
IDENTITY_PROTOS = {
    "husky": ("Husky",),
    "any_robot": (),
}

# A ``controller`` field value upstream reads as "no controller".
#
# ⚠ This set is WIDER than the OmniSim adapter's ``("", "<none>")`` and it has
# to be: ``"void"`` is upstream's own documented no-controller sentinel, so a
# scene of ten ``controller "void"`` robots declares ten controller fields and
# is not controlled at all. Note the direction of the asymmetry -- it can only
# make the control arm's A1.4 *harder* to fake, never easier.
_NO_BEHAVIOUR = ("", "void", "<none>")

# Upstream's frame options (WorldInfo.coordinateSystem). R2022b and later default
# to ENU, which is already the neutral schema's frame ("z is the axis the world's
# own gravity acts along"). NUE is y-up and MUST be remapped or every height
# assertion silently measures a northing.
#
# NUE is (x=North, y=Up, z=East); ENU is (x=East, y=North, z=Up).
_FRAME_PERMUTATION = {"ENU": (0, 1, 2), "NUE": (2, 0, 1)}
DEFAULT_FRAME = "ENU"

# --- citations ---------------------------------------------------------------

_ODE_INVARIANT = (
    "upstream Webots %s ships ODE as its ONLY physics engine -- a documented "
    "structural invariant of that simulator rather than a per-run reading, "
    "because it exposes no engine-selection field and writes no backend-verdict "
    "file to read. Its own %s WorldInfo reference documents the ODE solver "
    "constants directly (basicTimeStep, ERP, and the ERP/CFM-bearing "
    "contactProperties) and documents WorldInfo.physics as the path to an ODE "
    "*physics plugin*: https://cyberbotics.com/doc/reference/worldinfo and "
    "https://cyberbotics.com/doc/reference/physics-plugin. Install and "
    "invocation for this arm: docs/developer/webots-control-baseline.md."
    % (WEBOTS_RELEASE, WEBOTS_RELEASE))

_RECORDER_SOURCE = (
    "the grader-owned in-world Supervisor recorder's roster scan. Upstream has "
    "no HTTP/scene-tree surface (webots-control-baseline.md sec. 6), so a Supervisor "
    "controller is the only scene reader that exists")
_T0_SOURCE = (
    "the grader-owned Supervisor recorder's t=0 scan. The recorder is "
    "synchronization TRUE, so the world is genuinely frozen while it walks the "
    "scene -- but upstream exposes no bounds API, so each world-space AABB is "
    "one the recorder computed itself from the body's boundingObject geometry "
    "and pose")
_CONTACT_SOURCE = (
    "the recorder's per-step Supervisor Node.getContactPoints() scan, paired by "
    "world point to name both participants and filtered to pairs resolving to "
    "DISTINCT robot subtrees")
_TRAJ_SOURCE = (
    "the grader-owned Supervisor recorder: every tracked body's world position "
    "sampled once per basic timestep in simulated seconds, metres")
_LOG_SOURCE = (
    "the captured stdout+stderr of the webots process (launched --stdout "
    "--stderr) plus its exit code. WARNING: upstream Webots writes NO log file -- "
    "there is no post-hoc equivalent of $OMNISIM_LOG_PATH, so whatever was not "
    "captured at launch does not exist")
_NO_LOG_SOURCE = (
    "NOT AVAILABLE for this run: upstream Webots writes no log file, and no "
    "stdout/stderr capture was found, so the engine's own error stream and its "
    "controller-start lines could not be read at all")
_STARTS_SOURCE = (
    "'[INFO: ]<name>: Starting controller:' lines in the captured "
    "stdout/stderr of the webots process (upstream writes no log file)")
_PERF_FINALIZE = (
    "a non-empty --log-performance file: upstream flushes it only in "
    "worldClosed() (WbPerformanceLog.cpp:123-150), so its content is evidence "
    "the world closed cleanly and a killed run leaves it empty")

# --- notes carried on every Webots verdict -----------------------------------

_NO_HARNESS_NOTE = (
    "upstream Webots exposes no HTTP/JSON authoring surface, no scene-tree "
    "query, no hot reload and no structured load diagnostics "
    "(webots-control-baseline.md sec. 6), so there is no live-session pass on this "
    "arm. 'the artifact loaded' is evidenced instead by the in-world recorder "
    "having enumerated the scene at all.")
_NO_AUTOEXIT_NOTE = (
    "upstream Webots has no --duration and no auto-exit: a headless run "
    "terminates only when an in-world Supervisor calls simulationQuit(), or "
    "when something kills it. 'the run exited cleanly' therefore rests on the "
    "recorder's own quit record, not on a wall-clock timeout.")
_NO_HUSKY_NOTE = (
    "upstream Webots %s ships NO Husky: zero matches across all 15,404 paths "
    "of the tag (also none for jackal / warthog / ridgeback); "
    "projects/robots/clearpath/ holds heron, moose and pr2 only, and this "
    "repo's husky_description/ arrived with OUR URDF importer. The flagship "
    "identity predicate is therefore not portable as written -- it needs either "
    "an asset pre-converted with upstream's own urdf2webots or an explicitly "
    "declared analogue (webots-control-baseline.md sec. 4). This asymmetry cuts in "
    "OmniSim's favour, which is exactly why it is stated rather than buried."
    % WEBOTS_RELEASE)

_A1_T0_NOTE = (
    "A1.3 t=0 geometry measured by the grader-owned Supervisor recorder, which "
    "is synchronization TRUE and therefore freezes the world while it scans.")


# --- presentation-only wording ----------------------------------------------
#
# Labels carry NO logic (see graders/evidence.py). The identity keys are spelled
# in Webots' own words so a cross-sim table shows on its face that one column
# counted PROTO instantiations and the other counted URDF url references -- the
# thing "A1.1 passed on both" would otherwise hide.

_COMMON_LABELS = {
    "exit_code": "exit code",
    "error_lines": "ERROR: lines",
    "timed_out": "timed out",
}

LABELS = {
    "A1_husky_swarm_10": dict(
        _COMMON_LABELS,
        artifact_missing_detail="no .wbt artifact was produced",
        motion_absent_default="no recorder samples",
        identity_scene_count="Robot-derived PROTO instances >= %d joints"
                             % MIN_JOINTS,
        identity_declared_count="matching PROTO instantiations in the artifact",
        identity_scene_threshold="robot bodies",
        identity_declared_threshold="PROTO instantiations",
        distinct_ids="distinct node ids",
        behaviour_fields="non-empty controller fields",
        behaviour_starts="controller processes started",
        behaviour_fields_threshold="controller fields",
        behaviour_starts_threshold="starts",
        driver_completed="recorder called simulationQuit()",
        finalize_evidence="finalize evidence",
        attribution_threshold="an engine/solver attribution with a citation",
        attribution_missing_note=(
            "no physics-engine attribution: this run left no artifact showing "
            "that an upstream Webots process ran at all, so even the "
            "single-engine invariant cannot honestly be applied to it. An "
            "unattributed physics result is not a result (SPEC A1.10)."),
    ),
    "C2_fall_through_floor": dict(
        _COMMON_LABELS,
        artifact_missing_detail="no .wbt artifact",
        motion_absent_default="no samples",
        driver_completed="recorder called simulationQuit()",
        dynamic="Physics node attached",
        no_body_note="the recorder saw no Robot-derived node at all",
        attribution_missing_note=(
            "no physics-engine attribution (SPEC A1.10 rule, applied "
            "suite-wide): this run left no evidence an upstream Webots process "
            "ran."),
    ),
    "B3_measure_and_report": dict(_COMMON_LABELS),
}


# --- the adapter -------------------------------------------------------------


_SCENE_SCAN_SOURCE = (
    "the grader-owned Supervisor recorder's NAME-FREE t=0 scene scan "
    "(synchronization TRUE, before the first step): every non-robot "
    "Solid-derived body outside a Robot subtree, with a world-space AABB the "
    "recorder COMPUTED from that body's own geometry and world pose -- "
    "upstream exposes no bounds query, so nothing here is read from the "
    "engine's bounding volume. Mesh { url ... } geometry is not resolved and "
    "such a body arrives with bounds ABSENT and the reason attached")


def build_bundle(task_id, *, robot_identity="any_robot", live_expected=False,
                 artifact=None, run=None, run_dir=None, scratch_dir=None,
                 webots_dir=None, robot_proto=None, scene_inventory=False,
                 **_):
    """Read one upstream-Webots run into a neutral bundle. **Never raises.**

    ``run`` is a :class:`~agentbench.adapters.webots.recording.WebotsRun`; pass
    it when the Webots lane already parsed its own artifacts. Otherwise
    ``webots_dir`` (or ``$AGENTBENCH_WEBOTS_DIR``, or a ``webots/`` subdirectory
    of ``run_dir``) is searched for them.

    ``robot_proto`` names an explicitly declared **analogue** for the task's
    robot -- necessary because upstream ships no Husky. Passing it is recorded in
    the published identity rule and in a note; omitting it means the flagship
    predicate simply is not satisfied, which is the honest answer.

    ``scene_inventory`` opts this run's bundle into the recorder's NAME-FREE
    scene scan: the world-space AABBs it computed are merged into the bodies
    already listed (by node key, so one body keeps one identity) and any body
    it found that the roster does not carry -- a crate nested inside a group --
    is appended. It is **off by default and that is a contract**: on this arm
    the roster bodies currently carry bounds only when the separate
    ``agentbench_aabb_prober`` merged them in, and quietly supplying a second
    source would change what an already-frozen task measured. A grader that
    matches by GEOMETRY asks for it (``graders/r1.py``); one graded through the
    prober does not.

    The ``harness`` / ``phase_b`` keywords the OmniSim graders pass are accepted
    and ignored: both are OmniSim-shaped and neither has an upstream counterpart.
    """
    labels = dict(LABELS.get(task_id, _COMMON_LABELS))
    bundle = EvidenceBundle(task=task_id, sim=SIM, adapter=ADAPTER,
                            labels=labels)

    # -- locate the run ----------------------------------------------------
    tried = []
    if run is None:
        for cand in _run_dir_candidates(webots_dir, run_dir):
            tried.append(str(cand))
            got = recording.read_run(cand)
            if got.any_evidence:
                run = got
                break
            run = run or got
        if run is None:
            run = recording.read_run(None)
    # Filed under "motion" because that is the slot the cores fold into the
    # verdict's ``measurements`` (the OmniSim adapter files its PhaseBResult the
    # same way), so the whole run record travels with the row.
    bundle.adapter_measurements["motion"] = {
        "webots_run": dict(run.as_dict(), dirs_tried=tried)}
    if not run.any_evidence:
        bundle.notes.append(
            "no upstream-Webots run artifacts found (looked in: %s). Every "
            "field below is 'the adapter could not answer', which is not the "
            "same as a failing measurement."
            % (", ".join(tried) if tried else "nowhere -- no run_dir, no "
                                              "webots_dir, no "
                                              "$AGENTBENCH_WEBOTS_DIR"))

    bundle.notes.append(_NO_HARNESS_NOTE)
    bundle.notes.append(_NO_AUTOEXIT_NOTE)
    if live_expected:
        bundle.notes.append(_A1_T0_NOTE)

    # -- the artifact ------------------------------------------------------
    if artifact is None and scratch_dir is not None:
        artifact = common_wt.pick_artifact(scratch_dir)
    if artifact is None and run.world:
        artifact = run.world
    bundle.artifact = str(artifact) if artifact else None

    scanned = None
    if artifact is not None and Path(artifact).is_file():
        scanned = worldtext.scan(artifact)

    # -- frame -------------------------------------------------------------
    frame, frame_note = _frame(run, scanned)
    if frame_note:
        bundle.notes.append(frame_note)
    perm = _FRAME_PERMUTATION.get(frame, _FRAME_PERMUTATION[DEFAULT_FRAME])

    # -- identity ----------------------------------------------------------
    accepted, ident = _identity(robot_identity, robot_proto, scanned, bundle)
    bundle.identity = ident

    def robot_class(rec):
        return _robot_class(rec, robot_identity, accepted)

    # -- roster + the frozen t=0 inventory ---------------------------------
    #
    # ONE scan feeds both, unlike the OmniSim adapter (whose roster comes from
    # the free-running harness and whose t=0 comes from the recorder). Upstream
    # has no second channel, so roster and t0 are the same bodies in the same
    # order -- which also satisfies the trajectory index invariant below.
    rdoc = run.roster or {}
    rbodies = rdoc.get("bodies")
    if run.roster is None:
        err = (run.errors.get("roster")
               or run.missing.get("roster", "no roster recorded"))
        bundle.roster = BodyInventory(source=_RECORDER_SOURCE, error=err)
        bundle.t0 = BodyInventory(source=_T0_SOURCE, error=err)
    else:
        if not isinstance(rbodies, list):
            rbodies = []
        # The name-free scan is the only channel on this arm that carries a
        # world-space AABB for a body the prober's separate launch did not
        # measure. Resolved into a key -> box map FIRST, so a body that is
        # both in ``bodies`` and in the scan arrives once, with bounds.
        scan = rdoc.get("scene_bodies") if scene_inventory else None
        scan_boxes = _scan_boxes(scan, perm) if scan else {}
        bodies = [_body(rec, robot_class, perm, scan_boxes) for rec in rbodies
                  if isinstance(rec, dict)]
        frozen, frozen_note = _frozen(rdoc)
        if frozen_note:
            bundle.notes.append(frozen_note)
        t_s = rdoc.get("t_s")
        t_s = float(t_s) if isinstance(t_s, (int, float)) else None
        # The t=0 inventory gets the opt-in extra track kinds; the ROSTER does
        # not. That asymmetry is the whole point: the roster is what the cores
        # count robots in (R1.2, R2.2, R3.3), and a link of an arm is neither a
        # robot nor an independent body of the scene. Both lists are ordered
        # top-level-first, so a roster position still indexes the same body.
        extra = _extra_t0(rdoc, perm, scan_boxes)
        # Whatever the scan found that neither list already carries -- a crate
        # nested inside a group, an obstacle the agent named nothing in
        # particular. Appended LAST so every index already published stays put,
        # and never robot-class.
        seen_keys = {b.body_id for b in bodies} | {b.body_id for b in extra}
        fresh = ([b for b in _scene_extra(scan, perm)
                  if b.body_id and b.body_id not in seen_keys]
                 if scan else [])
        bundle.roster = BodyInventory(bodies=bodies + fresh, frozen=frozen,
                                      t_s=t_s, source=_RECORDER_SOURCE,
                                      error=rdoc.get("error"))
        bundle.t0 = BodyInventory(bodies=bodies + extra + fresh,
                                  frozen=frozen, t_s=t_s,
                                  source=_T0_SOURCE,
                                  error=rdoc.get("error"))
        if scene_inventory:
            _note_scene_scan(bundle, scan, len(fresh))
        if not any(b.has_aabb for b in bodies) and bodies:
            bundle.notes.append(
                "no world-space AABB on any body: upstream's Supervisor API has "
                "no bounds query, so the recorder must compute each AABB from "
                "the body's boundingObject geometry and pose. Without them the "
                "geometric half of the interpenetration assertion has no "
                "evidence either way.")
    bundle.adapter_measurements["roster"] = {
        "webots": {"bodies": len(bundle.roster.bodies),
                   "frozen": bundle.roster.frozen,
                   "source_file": str(run.files.get("roster") or "")}}

    # -- contacts ----------------------------------------------------------
    #
    # ABSENT MEANS None. An empty ContactObservation would tell the core "the
    # query ran and found nothing", which is a different claim and the one that
    # hid the (id, id) self-pair bug for weeks.
    if run.contacts is not None:
        bundle.contacts = _contacts(run.contacts, perm)

    # -- trajectory --------------------------------------------------------
    if run.trajectory is None:
        bundle.motion_error = (
            run.errors.get("trajectory")
            or run.missing.get("trajectory")
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
            bts_ms = ((run.trajectory or {}).get("basic_time_step_ms")
                      or (scanned or {}).get("basic_time_step_ms"))
            bundle.trajectory = _trajectory(arr, bundle, perm, frame, run,
                                            bts_ms)

    # -- process facts -----------------------------------------------------
    bundle.process = _process(run)
    fatals = _fatal_lines(run.console_text)
    if fatals:
        bundle.adapter_measurements["motion"]["fatal_lines"] = fatals[:5]
        bundle.notes.append(
            "%d FATAL: line(s) in the captured console. They are recorded but "
            "NOT counted as error lines, for exact parity with the OmniSim "
            "adapter's ERROR:-only rule -- the one direction a control-arm "
            "comparison must not lean is stricter on the control arm."
            % len(fatals))

    # -- engine attribution ------------------------------------------------
    bundle.attribution = _attribution(run, scanned, frame)
    if bundle.attribution is None:
        bundle.notes.append(
            "no engine attribution: nothing here evidences that an upstream "
            "Webots PROCESS ran (no exit code, no captured console, no "
            "--log-performance file, no recorder simulationQuit() record). "
            "Upstream ships exactly one physics engine, but that invariant "
            "describes the simulator, not a directory of numbers -- so the "
            "run stays unattributed, which is INVALID rather than FAIL "
            "(SPEC A1.10).")

    # -- 'the artifact loaded' ---------------------------------------------
    #
    # There is no load RPC to have succeeded, so the honest witness is that the
    # in-world recorder enumerated the scene: a world that failed to load has no
    # controllers and therefore no roster.
    if run.roster is not None or run.trajectory is not None:
        bundle.live_load_ok = True
    elif run.any_evidence:
        bundle.live_load_ok = False
    else:
        bundle.live_load_ok = None

    if scanned is not None:
        net = worldtext.network_protos(scanned)
        bundle.adapter_measurements["identity"] = dict(
            bundle.adapter_measurements.get("identity") or {},
            externproto_count=len(scanned.get("externprotos") or []),
            coordinate_system=frame)
        if net:
            bundle.notes.append(
                "%d EXTERNPROTO import(s) in this world are fetched over the "
                "network on first load (the R2025a distribution ships zero "
                ".proto files; webots-control-baseline.md sec. 5.1). That is "
                "against this suite's local-asset-only rule: a network hiccup "
                "on a cold cache becomes a benchmark result." % len(net))
    return bundle


# --- run location -----------------------------------------------------------


def _run_dir_candidates(webots_dir, run_dir):
    """Where to look for the Webots lane's artifacts, most explicit first."""
    out = []
    for c in (webots_dir, os.environ.get("AGENTBENCH_WEBOTS_DIR")):
        if c:
            out.append(Path(c))
    if run_dir:
        rd = Path(run_dir)
        out += [rd / "webots", rd / "phaseW", rd]
    seen, uniq = set(), []
    for p in out:
        key = str(p)
        if key not in seen:
            seen.add(key)
            uniq.append(p)
    return uniq


# --- identity ---------------------------------------------------------------


def _identity(robot_identity, robot_proto, scanned, bundle):
    """The accepted PROTO set and the published :class:`IdentityRule`."""
    accepted = tuple(IDENTITY_PROTOS.get(robot_identity, ()))
    substitute = str(robot_proto).strip() if robot_proto else ""
    if substitute:
        accepted = accepted + (substitute,)

    if robot_identity == "husky":
        bundle.notes.append(_NO_HUSKY_NOTE)
        scene_rule = (
            "a scene body whose PROTO/node type name is one of {%s} and which "
            "carries >= %d joints. Upstream does NOT expand PROTOs, so the "
            "model name IS present in the live scene: getTypeName() on the "
            "instance returns the PROTO name and getBaseTypeName() returns "
            "Robot. The joint floor is identical to the OmniSim adapter's, so "
            "the two identity rules differ only in the channel carrying the "
            "name." % (", ".join(accepted) or "-- nothing accepted --",
                       MIN_JOINTS))
        if substitute:
            scene_rule += (
                " DECLARED ANALOGUE: %r was accepted as a substitute for the "
                "task's robot because upstream ships no Husky. The task is "
                "being scored by OUTCOME ('ten four-wheeled UGVs moving "
                "randomly') with each simulator using its native robot, and "
                "this is the substitution that was made." % substitute)
        declaration_rule = (
            "depth-0 instantiations in the artifact text whose PROTO/node type "
            "is one of {%s} (a 'DEF <id> <Proto> { ... }' block). Unlike the "
            "OmniSim side -- where the engine expands URDFRobot { url ... } at "
            "parse time and the model name survives ONLY in the file -- this "
            "count corroborates a name the live scene can also report."
            % (", ".join(accepted) or "-- nothing accepted --"))
    else:
        scene_rule = (
            "a scene body whose base node type is %s (getBaseTypeName(), which "
            "reports Robot for any Robot-derived PROTO instance), enumerated by "
            "node type + name and never by DEF (SPEC 4.5.1: the agent has no "
            "reason to add DEFs)" % ROBOT_BASE_TYPE)
        declaration_rule = ("not counted: this task ships the world, so counting "
                            "declarations would grade our own fixture")

    declared = None
    if scanned is not None and robot_identity == "husky":
        matches = worldtext.proto_instances(scanned, accepted)
        declared = len(matches)
        uses = worldtext.uses_of(scanned, accepted)
        bundle.adapter_measurements["identity"] = {
            "proto_instantiations": declared,
            "proto_types_seen": sorted({n["type"] for n in
                                        scanned.get("nodes", [])}),
            "use_references_to_those_defs": len(uses),
            "accepted_protos": list(accepted),
            "declared_analogue": substitute or None,
        }
        if uses:
            bundle.notes.append(
                "%d 'USE <DEF>' reference(s) point at a matching robot "
                "definition. USE references are counted separately and are NOT "
                "folded into the declaration count, so a world written that way "
                "is visibly under-counted rather than invisibly so." % len(uses))
    elif robot_identity == "husky":
        bundle.notes.append(
            "the artifact text could not be read, so the declaration half of "
            "the identity rule is unanswered (None), not zero.")

    label = "Husky" if robot_identity == "husky" else "robot"
    if substitute:
        label = "%s (declared analogue: %s)" % (label, substitute)
    return accepted, IdentityRule(
        label=label,
        requirement=("a distinct robot-class body that is the robot the task "
                     "named" if robot_identity == "husky"
                     else "the robot body the task is about"),
        scene_rule=scene_rule, declaration_rule=declaration_rule,
        declared_count=declared)


def _robot_class(rec, robot_identity, accepted):
    """Is this recorded body the robot the task named? True / False / None.

    ``None`` is returned only when the recorder gave us neither type channel --
    an unanswerable question, which the core reports as an absent identity
    witness. A body that IS answerable and simply is not the robot returns
    ``False``: a Moose is not a Husky, and that is evidence, not ignorance.
    """
    kind = str(rec.get("type") or "")
    base = str(rec.get("base_type") or "")
    accepted_lower = {a.lower() for a in accepted}
    if not kind and not base:
        return None
    if base:
        is_robot_derived = (base == ROBOT_BASE_TYPE)
    elif kind == ROBOT_BASE_TYPE or kind.lower() in accepted_lower:
        is_robot_derived = True
    else:
        # No getBaseTypeName() channel and an unrecognised PROTO name: whether
        # this PROTO derives from Robot is genuinely unknowable from here. Saying
        # False would be evidence of absence we do not have.
        is_robot_derived = None
    if robot_identity != "husky":
        return None if is_robot_derived is None else bool(is_robot_derived)
    if not accepted:
        # Nothing can satisfy the predicate: there is no such robot upstream and
        # no analogue was declared. That is a definite negative, not an unknown.
        return False
    n_joints = rec.get("n_joints")
    if n_joints is None:
        n_joints = rec.get("num_joints")
    # For the flagship predicate the PROTO name is decisive either way: a body
    # whose type is not an accepted one is definitely not the robot, whether or
    # not we could tell that it derives from Robot.
    return bool(is_robot_derived is not False
                and kind.lower() in accepted_lower
                and (n_joints or 0) >= MIN_JOINTS)


# --- bodies -----------------------------------------------------------------


def _effective(controller):
    """The behaviour that will actually run, or None if the field means none."""
    return None if (controller or "") in _NO_BEHAVIOUR else controller


def _body(rec, robot_class, perm, scan_boxes=None):
    n_joints = rec.get("n_joints")
    if n_joints is None:
        n_joints = rec.get("num_joints")
    controller = rec.get("controller")
    lo, hi = _aabb(rec, perm)
    # ``bounds_from`` stays None on the prober's path so this citation is
    # BYTE-IDENTICAL to the one every already-graded row carries -- c2_core
    # prints ``identity_evidence`` into its ``attested`` list, and a frozen row
    # must not change because a channel it never used now exists.
    bounds_from = None
    if lo is None and scan_boxes:
        # The prober did not measure this body (or was never launched for this
        # task): take the recorder's own name-free scan, which keys on the
        # SAME identity ``_body_id`` builds. The prober WINS when both exist,
        # so a task already graded through it keeps being graded on its
        # numbers, from its launch.
        got = scan_boxes.get(_body_id(rec))
        if got:
            lo, hi, bounds_from = got[0], got[1], got[2]
    kind = str(rec.get("type") or "")
    base = str(rec.get("base_type") or "")
    return Body(
        body_id=_body_id(rec), name=str(rec.get("name") or ""), kind=kind,
        position=_point(rec.get("position"), perm),
        aabb_min=lo, aabb_max=hi,
        n_joints=int(n_joints) if isinstance(n_joints, (int, float)) else None,
        behaviour=_effective(controller), behaviour_declared=controller,
        dynamic=rec.get("has_physics"), robot_class=robot_class(rec),
        identity_evidence="getTypeName()=%s getBaseTypeName()=%s n_joints=%s "
                          "(Supervisor recorder scan)%s"
                          % (kind or "?", base or "?", n_joints,
                             "; world-space AABB from %s" % bounds_from
                             if bounds_from else ""))


def _scan_boxes(scan, perm):
    """``{body key: (aabb_min, aabb_max, provenance)}`` from the scene scan.

    Bodies the scan could not bound are deliberately ABSENT from the map
    rather than present with ``None``: a caller must not be able to overwrite
    a real box with a missing one.
    """
    out = {}
    for rec in ((scan or {}).get("bodies") or []):
        if not isinstance(rec, dict):
            continue
        key = str(rec.get("key") or "")
        lo, hi = _aabb(rec, perm)
        if not key or lo is None or hi is None:
            continue
        out[key] = (lo, hi, "the recorder's name-free t=0 scan (%s)"
                    % (rec.get("bounds_source") or "geometry walk"))
    return out


def _scene_extra(scan, perm):
    """Bodies the name-free scan found that no other channel reported.

    ``robot_class=False`` unconditionally, and that is the load-bearing half:
    a scanned box is scenery, and R1.2 ("exactly one drivable robot"), R2.2 and
    R3.3 all count ``inventory.robots``. The walk never enters a Robot subtree,
    so nothing here can be one -- but the field states it rather than relying
    on the walk to be remembered.
    """
    out = []
    for rec in ((scan or {}).get("bodies") or []):
        if not isinstance(rec, dict):
            continue
        key = str(rec.get("key") or "")
        if not key:
            continue
        lo, hi = _aabb(rec, perm)
        nested = rec.get("nested_in") or None
        skipped = rec.get("skipped_geometry") or []
        out.append(Body(
            body_id=key, name=str(rec.get("name") or ""),
            kind=str(rec.get("type") or "Solid"),
            position=_point(rec.get("position"), perm),
            aabb_min=lo, aabb_max=hi,
            dynamic=rec.get("has_physics"), robot_class=False,
            member_of=str(nested) if nested else None,
            identity_evidence=(
                "non-robot %s found by the recorder's NAME-FREE t=0 scene scan "
                "(depth %s%s); bounds: %s"
                % (rec.get("base_type") or rec.get("type") or "Solid",
                   rec.get("depth"),
                   ", inside %s" % nested if nested else ", top level",
                   ("computed from %s" % rec.get("bounds_source"))
                   if lo is not None else
                   ("NOT MEASURED: %s" % ", ".join(skipped[:3])
                    if skipped else "NOT MEASURED")))))
    return out


def _note_scene_scan(bundle, scan, n_fresh):
    """Publish what the name-free scan cost and what it could not answer."""
    if not scan:
        bundle.notes.append(
            "the recorder's NAME-FREE t=0 scene scan did not run on this "
            "recording, so no non-robot body was offered to the geometric "
            "matchers with world-space bounds. That is an INSTRUMENT GAP, not "
            "a statement about what the world contains.")
        return
    bodies = [b for b in (scan.get("bodies") or []) if isinstance(b, dict)]
    bundle.adapter_measurements.setdefault("roster", {})["scene_scan"] = {
        "supported": scan.get("supported"),
        "found": scan.get("found"), "bounded": scan.get("bounded"),
        "cap": scan.get("cap"), "truncated": scan.get("truncated"),
        "added_to_inventories": n_fresh,
        "merged_into_existing": len(bodies) - n_fresh,
        "source": _SCENE_SCAN_SOURCE,
    }
    if scan.get("truncated"):
        bundle.notes.append(
            "the name-free scene scan is TRUNCATED: %s non-robot bod(ies) "
            "matched the predicate and the first %s were bounded. A body past "
            "the cap has no entry, so an assertion that fails to find "
            "something may be looking at a truncated inventory."
            % (scan.get("found"), scan.get("cap")))
    unbounded = [b for b in bodies if not (b.get("bounds") or {}).get(
        "bbox_min")]
    if unbounded:
        why = sorted({s for b in unbounded
                      for s in (b.get("skipped_geometry") or [])})
        bundle.notes.append(
            "%d of the %d bodies in the name-free scene scan carry NO "
            "world-space AABB: upstream exposes no bounds query, so this arm "
            "computes each box from the body's own geometry, and %s cannot be "
            "resolved by a controller running under upstream's python3. They "
            "are in the inventory with bounds absent, never with a guessed "
            "box -- so a geometric assertion over them is unmeasured rather "
            "than failed, and the asymmetry with the OmniSim arm (whose "
            "geometry helper reads mesh files off disk) is visible here "
            "rather than buried."
            % (len(unbounded), len(bodies),
               ", ".join(why[:4]) or "their geometry"))


def _extra_t0(rdoc, perm, scan_boxes=None):
    """Bodies from the recorder's OPT-IN track kinds, for the t=0 inventory.

    Links first (each carrying ``member_of``, its owning robot), then any named
    Solid the recorder found nested rather than at the top level -- a top-level
    one is already in ``bodies`` and is not duplicated here.

    Every one of these is ``robot_class=False``. A LINK never carries a
    world-space AABB on this arm: the name-free scan does not enter a Robot
    subtree (its links belong to ``--links=``), and the prober measures
    top-level bodies only -- so a link's box is genuinely unmeasured here and
    is left absent. A nested named Solid DOES get one when the scan bounded it.
    """
    out = []
    for rec in (rdoc.get("links") or []):
        if not isinstance(rec, dict):
            continue
        why = ("joint endPoint" if rec.get("joint_endpoint")
               else "carries a Physics node")
        out.append(Body(
            body_id=str(rec.get("key") or ""),
            name=str(rec.get("name") or ""),
            kind=str(rec.get("type") or "Solid"),
            dynamic=rec.get("has_physics"), robot_class=False,
            member_of=(str(rec["parent"]) if rec.get("parent") else None),
            identity_evidence="link %s of %s (%s), Supervisor recorder t=0 scan"
                              % (rec.get("link_index"), rec.get("parent"), why)))
    seen = {str(b.get("name") or "") for b in (rdoc.get("bodies") or [])
            if isinstance(b, dict)}
    for rec in (rdoc.get("named_solids") or []):
        if not isinstance(rec, dict):
            continue
        nm = str(rec.get("name") or "")
        if not nm or nm in seen:
            continue
        # The published body_id convention is unchanged (DEF, else the name);
        # the scan's own key is only used to LOOK UP a box, never to rename a
        # body that a frozen row may already cite.
        body_id = str(rec.get("def")) if rec.get("def") else nm
        got = ((scan_boxes or {}).get(str(rec.get("key") or ""))
               or (scan_boxes or {}).get(body_id))
        out.append(Body(
            body_id=body_id, name=nm,
            kind=str(rec.get("type") or "Solid"),
            position=_point(rec.get("position"), perm),
            aabb_min=got[0] if got else None,
            aabb_max=got[1] if got else None,
            dynamic=rec.get("has_physics"), robot_class=False,
            identity_evidence="named Solid (nested), Supervisor recorder t=0 "
                              "scan; per-step track: %s%s"
                              % (rec.get("track_reason") or "not requested",
                                 "; world-space AABB from %s" % got[2]
                                 if got else "")))
    return out


def _body_id(rec):
    """A stable per-run body id: the DEF, else ``#<node id>``, else the name.

    The agent has no reason to add DEFs (SPEC 4.5.1), so ``getId()`` is the
    channel that actually carries uniqueness.
    """
    d = rec.get("def")
    if d:
        return str(d)
    nid = rec.get("id")
    if nid is not None:
        return "#%s" % nid
    name = rec.get("name")
    return str(name) if name else ""


def _point(seq, perm):
    p = recording.triple(seq)
    return None if p is None else tuple(p[i] for i in perm)


def _aabb(rec, perm):
    """``(min_xyz, max_xyz)`` in the neutral frame, or ``(None, None)``.

    Two shapes are accepted -- flat ``aabb_min``/``aabb_max`` and the nested
    ``bounds: {bbox_min, bbox_max}`` the OmniSim recorder writes -- so a Webots
    recorder can reuse either layout. After a frame permutation the corners are
    re-min/maxed componentwise, because permuting a box's corners does not keep
    them sorted.
    """
    lo = recording.triple(rec.get("aabb_min"))
    hi = recording.triple(rec.get("aabb_max"))
    if lo is None or hi is None:
        b = rec.get("bounds") or {}
        if isinstance(b, dict):
            lo = recording.triple(b.get("bbox_min"))
            hi = recording.triple(b.get("bbox_max"))
    if lo is None or hi is None:
        return None, None
    a = tuple(lo[i] for i in perm)
    c = tuple(hi[i] for i in perm)
    return (tuple(min(x, y) for x, y in zip(a, c)),
            tuple(max(x, y) for x, y in zip(a, c)))


# --- contacts ---------------------------------------------------------------


def _contacts(doc, perm):
    """One :class:`ContactObservation`. Witness counters are never invented."""
    pairs = []
    for c in (doc.get("pairs") or []):
        if not isinstance(c, dict):
            continue
        pairs.append(ContactPair(
            a=c.get("a") or c.get("a_def") or c.get("a_robot"),
            b=c.get("b") or c.get("b_def") or c.get("b_robot"),
            a_robot=bool(c.get("a_robot", True)),
            b_robot=bool(c.get("b_robot", True)),
            point=_point(c.get("point"), perm), step=c.get("step")))
    return ContactObservation(
        pairs=pairs,
        steps=int(doc.get("steps") or 0),
        window_s=doc.get("window_s"),
        supported=bool(doc.get("supported")),
        # ``or None`` is deliberately NOT used here: 0 is a real count and must
        # survive, while a missing key must stay None (adapter rule 2).
        total_observed=_opt_int(doc.get("total_observed")),
        distinct_named=_opt_int(doc.get("distinct_named")),
        source=doc.get("source") or _CONTACT_SOURCE,
        error=doc.get("error"))


def _opt_int(v):
    if isinstance(v, bool) or v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


# --- trajectory -------------------------------------------------------------


def _trajectory(arr, bundle, perm, frame, run, bts_ms=None):
    """The pose series, with the index invariant honoured against the roster.

    ``Trajectory`` promises that row *i* of ``xyz`` is ``body_ids[i]``, and that
    a core which looks a body up **by name** in a frozen inventory and indexes
    ``xyz`` with that position gets the same body (``c2_core`` does exactly
    that). The recorder writes both files, but nothing structurally guarantees
    the two orders agree -- so when every trajectory name maps to exactly one
    roster body, the rows are permuted into roster order. When they do not, the
    trajectory keeps its own order, the ids become the recorder's names, and the
    mismatch is published.
    """
    xyz = arr.xyz[:, :, perm]
    names = list(arr.names)
    body_ids = list(names)
    kinds = list(arr.kinds)
    parents = list(arr.parents)
    roster = bundle.roster

    if roster.bodies:
        idx = []
        ok = True
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
            # The descriptive vectors are PARALLEL to the rows, so they have to
            # travel with the permutation or row 3's kind would describe row 5.
            kinds = [kinds[k] for k in order] if kinds else []
            parents = [parents[k] for k in order] if parents else []
        else:
            bundle.notes.append(
                "trajectory rows could not be aligned to the roster by name "
                "(%d trajectory bodies, %d roster bodies); the bundle's "
                "body_ids are the recorder's trajectory names and a per-body "
                "index is NOT valid across the two inventories."
                % (len(names), len(roster.bodies)))
    # ``REQUIRED_EVIDENCE`` asks for a pose "once per physics step ... not
    # resampled". A recorder that strides is still usable -- but the difference
    # has to be on the row, because integrated path length over a coarse stride
    # is a LOWER bound (every sub-sample wiggle is cut off), so A1.6 is being
    # asked an easier question of a longer-strided arm than of a per-step one.
    if bts_ms and arr.dt_s and arr.dt_s * 1000.0 > 1.5 * float(bts_ms):
        bundle.notes.append(
            "the pose series is DOWNSAMPLED: sampled every %.4g ms while the "
            "world's basicTimeStep is %.4g ms (a stride of ~%d steps). It is "
            "not the one-sample-per-physics-step series REQUIRED_EVIDENCE asks "
            "for, and integrated path length over a stride is a LOWER bound -- "
            "so the path-length floor is being applied to a different quantity "
            "than it is on a per-step arm. Direction of the bias: against this "
            "run, not for it."
            % (arr.dt_s * 1000.0, float(bts_ms),
               max(1, int(round(arr.dt_s * 1000.0 / float(bts_ms))))))

    src = _TRAJ_SOURCE
    if frame != DEFAULT_FRAME:
        src += (" -- remapped from the world's %s frame to %s"
                % (frame, DEFAULT_FRAME))
    if run.files.get("trajectory"):
        src += " (%s)" % Path(run.files["trajectory"]).name
    complete = arr.complete
    if complete is None:
        complete = bool((run.completion or {}).get("complete"))
    return Trajectory(body_ids=body_ids, t=arr.t, xyz=xyz, vel=None,
                      dt_s=arr.dt_s, recorded_s=arr.recorded_s,
                      complete=bool(complete), source=src,
                      kinds=kinds, parents=parents, names=names)


# --- process ----------------------------------------------------------------

# The engine's error-class prefix. Kept to EXACTLY the OmniSim adapter's
# ``ERROR:`` on purpose: both engines descend from the same WbLog, which also
# writes ``FATAL:``, and neither adapter counts that as an error line. Widening
# it here alone would make A1.9 stricter on the control arm than on ours, which
# is the one direction a comparison must never lean. ``FATAL:`` lines are
# recorded separately and noted, so a fatal run is visible rather than counted.
_ERROR_LINE_PREFIX = "ERROR:"
_FATAL_LINE_PREFIX = "FATAL:"

# ``INFO: <name>: Starting controller: <cmd>`` (WbController::info prepends the
# controller name). The INFO prefix is optional because this arm's only channel
# is a captured console, which may have been filtered on the way to disk --
# missing a start that really happened would be a false FAIL.
_CONTROLLER_START = re.compile(
    r"(?m)^(?:INFO:\s+)?(\S+):\s+Starting controller:")


def _error_lines(text):
    return [ln for ln in (text or "").splitlines()
            if ln.startswith(_ERROR_LINE_PREFIX)]


def _fatal_lines(text):
    return [ln for ln in (text or "").splitlines()
            if ln.startswith(_FATAL_LINE_PREFIX)]


def _controller_starts(text):
    out = {}
    for name in _CONTROLLER_START.findall(text or ""):
        out[name] = out.get(name, 0) + 1
    return out


def _process(run):
    """:class:`ProcessFacts`, or None when nothing about the process is known."""
    pdoc = run.process or {}
    cdoc = run.completion or {}
    have_console = run.console_text is not None
    if not pdoc and not cdoc and not have_console:
        return None

    errs = _error_lines(run.console_text)
    fatals = _fatal_lines(run.console_text)

    driver_completed = None
    if cdoc:
        driver_completed = bool(cdoc.get("complete")
                                and cdoc.get("quit_called", True))

    perf_ok = run.perf_nonempty
    if perf_ok:
        reached, why = True, _PERF_FINALIZE
    elif driver_completed:
        reached = True
        why = ("the recorder reached %s s of simulated time and called "
               "simulationQuit(), which upstream cannot do unless the world was "
               "built, finalized and stepped"
               % _fmt(cdoc.get("recorded_s")))
    elif driver_completed is False or perf_ok is False:
        reached = False
        why = ("no finalize evidence: %s%s"
               % ("the recorder did not reach its target simulated time"
                  if driver_completed is False else "no recorder record",
                  "; the --log-performance file is empty, which upstream leaves "
                  "behind when a run is killed rather than quit"
                  if perf_ok is False else ""))
    else:
        reached, why = None, ("neither a recorder completion record nor a "
                              "--log-performance file: 'did the world "
                              "finalize' is unanswered, not answered no")

    if not have_console:
        log_source = _NO_LOG_SOURCE
    elif fatals:
        log_source = ("%s. WARNING: %d FATAL: line(s) were also seen; they are NOT "
                      "counted as error lines, for parity with the OmniSim "
                      "adapter -- see the _ERROR_LINE_PREFIX note."
                      % (_LOG_SOURCE, len(fatals)))
    else:
        log_source = _LOG_SOURCE

    return ProcessFacts(
        exit_code=_opt_int(pdoc.get("exit_code")),
        timed_out=bool(pdoc.get("timed_out")),
        error_lines=errs,
        log_available=have_console,
        log_source=log_source,
        # {} would claim "counted, found none"; None is "could not count".
        behaviour_starts=(_controller_starts(run.console_text)
                          if have_console else None),
        start_source=_STARTS_SOURCE if have_console else "",
        driver_completed=driver_completed,
        reached_finalize=reached, finalize_evidence=why,
        wall_s=pdoc.get("wall_s"),
        attempts_used=int(pdoc.get("attempts_used") or 1))


def _fmt(v):
    return "%g" % v if isinstance(v, (int, float)) else "?"


# --- attribution ------------------------------------------------------------


def _process_evidence(run):
    """Did an upstream *process* demonstrably run, as opposed to leaving numbers?

    A ``trajectory.json`` on its own is a bag of pose samples: it could have come
    from anywhere. An exit code, a console capture, a ``--log-performance`` file
    or a recorder ``simulationQuit()`` record could not.
    """
    return bool(run.process or run.completion or run.perf_path
                or run.console_text is not None)


def _attribution(run, scanned, frame):
    """SPEC A1.10 on upstream Webots: the single-engine invariant, with a cite.

    Asserted only for a run that left evidence a Webots **process** happened.
    "Webots is ODE" is a true statement about the *simulator*; applying it to a
    directory of numbers would be attributing a run that may never have taken
    place, and A1.10's rule is that an unattributed physics result is INVALID
    rather than FAILED.

    Note the consequence, which is a real asymmetry between the two arms: on
    OmniSim the attribution is a per-run *reading* (the ``.newton.json`` sidecar)
    and can genuinely be missing for a run that otherwise worked, so A1.10 is a
    live INVALID risk there. Upstream has one engine and no such file, so once a
    Webots process is evidenced at all the attribution always succeeds -- A1.10
    can only be INVALID here when the process itself is unevidenced.
    """
    if not _process_evidence(run):
        return None
    pdoc = run.process or {}
    version = str(pdoc.get("webots_version") or "").strip()
    source = _ODE_INVARIANT
    if version and WEBOTS_RELEASE.lower() not in version.lower():
        source += (" WARNING: this run reports version %r, while the citation above is "
                   "written against %s; the invariant is believed to hold for "
                   "every upstream release but it was verified for %s."
                   % (version, WEBOTS_RELEASE, WEBOTS_RELEASE))
    extra = {"engine_invariant": True,
             "webots_version": version or None,
             "coordinate_system": frame}
    if scanned is not None:
        plugin = scanned.get("physics_plugin")
        extra["ode_physics_plugin"] = plugin
        if plugin:
            source += (" This world also declares WorldInfo.physics %r: an ODE "
                       "physics plugin, which can change the forces applied "
                       "without changing the engine." % plugin)
        extra["basic_time_step_ms"] = scanned.get("basic_time_step_ms")
    return EngineAttribution(backend="ode", solver="ode", degraded=False,
                             source=source, extra=extra)


# --- frame ------------------------------------------------------------------


def _frame(run, scanned):
    """The world's coordinate system, and a note when it is not the default.

    Order of preference: what the recorder read off ``WorldInfo`` in the live
    scene, then the artifact text, then R2022b-and-later's documented ENU
    default. An unrecognised value is treated as ENU **and said out loud**.
    """
    for doc in (run.roster, run.completion, run.trajectory):
        cs = str((doc or {}).get("coordinate_system") or "").upper()
        if cs:
            break
    else:
        cs = ""
    if not cs and scanned is not None:
        cs = str(scanned.get("coordinate_system") or "").upper()
    if not cs:
        return DEFAULT_FRAME, None
    if cs == DEFAULT_FRAME:
        return cs, None
    if cs in _FRAME_PERMUTATION:
        return cs, (
            "the world declares WorldInfo.coordinateSystem %r (y-up); every "
            "position and AABB in this bundle was remapped to the neutral "
            "frame (%s, z along gravity) so the vertical assertions measure a "
            "height and not a northing." % (cs, DEFAULT_FRAME))
    return DEFAULT_FRAME, (
        "the world declares an unrecognised WorldInfo.coordinateSystem %r; the "
        "bundle assumes %s (the R2022b-and-later default) and applies NO "
        "remapping. If that world is not z-up, every vertical assertion in this "
        "row is measuring the wrong axis." % (cs, DEFAULT_FRAME))


def _frozen(rdoc):
    """``frozen`` for the inventories -- never upgraded, only downgraded.

    An in-world Supervisor with ``synchronization TRUE`` blocks the simulation
    between its ``step()`` calls, so a scan taken before the first step really is
    at t=0 and the recorder may claim it. If the recorder says it was
    unsynchronized, the claim is refused regardless: upstream free-runs in
    ``--mode=fast`` exactly like OmniSim's harness does, and a free-running read
    cannot answer any "at t=0" question.
    """
    frozen = rdoc.get("frozen")
    frozen = None if frozen is None else bool(frozen)
    if frozen and rdoc.get("synchronization") is False:
        return False, (
            "the recorder claimed a frozen t=0 scan but also reports "
            "synchronization FALSE, so the world was free-running while it "
            "scanned; the claim is refused and the t=0 inventory is marked "
            "un-frozen.")
    if frozen is None:
        return None, (
            "the recorder did not state whether its scan was taken with the "
            "world frozen, so 'frozen' is unanswered rather than assumed; no "
            "'at t=0' assertion can rest on it.")
    return frozen, None


__all__ = ["ADAPTER", "IDENTITY_PROTOS", "LABELS", "MIN_JOINTS", "SIM",
           "WEBOTS_RELEASE", "build_bundle"]
