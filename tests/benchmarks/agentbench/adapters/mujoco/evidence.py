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

"""**MuJoCo** -> :class:`EvidenceBundle`. The strongest authoring-tier adversary.

MuJoCo is on the primary leaderboard *because it may beat us* (SPEC 6.1): MJCF
is compact, heavily represented in training data, and an LLM writing ``<body>``
blocks plus a twenty-line Python driver may simply outperform an agent
authoring a ``.wbt`` and driving a harness. This adapter is therefore written
to the standard SPEC 6.2 sets for a competitor's scaffolding: where MuJoCo can
answer a question it is asked properly, and where it cannot the answer is
``None`` with the reason, never a value that quietly costs it an assertion.

What MuJoCo gives us, and what it does not
------------------------------------------

======================  ====================================================
neutral field           the MuJoCo channel, and its limit
======================  ====================================================
``identity``            **a decision, not a reading.** MJCF has no ``Robot``
                        node -- ``<body>`` describes a manipulator link, a
                        crate and a wall alike -- so "is this a robot" is
                        answered as *"a top-level body whose subtree something
                        can drive"*: an actuator transmission resolving into
                        it, or an applied force observed on it during the run.
                        Rule, evidence and failure modes: ``BRINGUP.md`` sec. 4.
``roster`` / ``t0``     ONE frozen scan, taken by the grader-owned recorder on
                        the first observed ``mj_step`` -- before any
                        integration, on a deep copy of ``MjData``, so it
                        cannot perturb the run. It carries a world-space AABB
                        for **every** body AND for every geom hanging directly
                        off ``<worldbody>``, which is how MJCF idiomatically
                        writes static scenery. No name list is needed to get
                        bounds here, which is a real instrumentation advantage
                        over both other arms.
``contacts``            ``data.ncon`` / ``data.contact`` after every step,
                        naming BOTH participants -- MuJoCo's contact record is
                        a geom pair, so there is no world-point re-pairing step
                        and no way to produce the ``(id, id)`` self-pair that
                        hid A1.3 for weeks. Deduped by participant pair with a
                        first witness; the two vacuity counters are totals.
``trajectory``          the recorder's own per-step sampling of ``data.xpos``
                        in simulated seconds and metres. Not polled over
                        anything, not resampled.
``process``             the child interpreter's exit code plus MuJoCo's own
                        error class. ⚠ MuJoCo writes **no log file** and has no
                        engine log format at all: it raises. See
                        :func:`_error_lines` for what is counted and the
                        direction the choice leans.
``reached_finalize``    the model COMPILED and at least one integration step
                        was observed. There is no world-finalize event in
                        MuJoCo -- ``mj_step`` on a compiled ``MjModel`` is the
                        whole of "the world was built and stepped".
``attribution``         read from the compiled model at run time:
                        ``mjOption.solver`` / ``integrator`` / ``cone`` plus
                        ``mj_versionString()``. A per-run READING, not a
                        structural invariant -- stronger evidence than the
                        Webots arm can offer.
``view``                ``<camera>`` elements, converted from MuJoCo's -z
                        forward / +y up frame and degrees of vertical FOV into
                        the neutral world-frame vectors and radians.
======================  ====================================================

Three places this adapter is deliberately NOT shaped like the other two, each
because the honest mapping differs rather than because it was convenient:

``behaviour_declared`` is always ``None``
    MJCF has no per-body ``controller`` field, and MuJoCo starts no per-robot
    processes: **one** external Python program drives the whole scene. So the
    "N controller fields vs N controller starts" pair that A1.4 compares has no
    per-body counterpart here, and inventing one -- writing the driver's name
    into every body's *declared* slot -- would manufacture an agreement the
    scene never stated. The effective ``behaviour`` is filled (the driver, when
    it was observed stepping), and the difference is published.

world-attached geoms are inventory entries in their own right
    Folding five obstacle geoms on ``<worldbody>`` into the single ``world``
    body would report them as one object with an AABB spanning the scene. That
    is not a conservative simplification, it is a wrong measurement.

no retries
    The other two arms retry a launch flake that comes from having a simulator
    process with a GUI stack in it. Here there is an interpreter and an import.
    A retry loop would only re-roll a genuine result.
"""

from __future__ import annotations

import math
import os
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from agentbench.adapters.mujoco import recording  # noqa: E402
from agentbench.graders.evidence import (  # noqa: E402
    Body, BodyInventory, CameraPose, ContactObservation, ContactPair,
    EngineAttribution, EvidenceBundle, IdentityRule, ProcessFacts, Trajectory,
    ViewEvidence)

SIM = "mujoco"
ADAPTER = "agentbench.adapters.mujoco.evidence"

#: The deliverable's extension on this arm. An MJCF scene is ``.xml``; the
#: program that steps it is a sibling ``.py`` (see ``launcher.find_driver``).
MODEL_SUFFIX = ".xml"

#: Kept identical to the OmniSim and Webots adapters on purpose: the three
#: identity rules must differ ONLY in the channel that carries the model's
#: identity, so a cross-sim reader can see it is the same question.
MIN_JOINTS = 4

#: Model names that satisfy the flagship identity predicate out of the box.
#:
#: ⚠ EMPTY BY DEFAULT, and that is the finding rather than an omission: neither
#: MuJoCo nor MuJoCo Menagerie ships a Clearpath Husky, so -- exactly as on the
#: Webots arm -- the flagship predicate can only be satisfied by an explicitly
#: declared analogue, passed as ``robot_model=`` and recorded in the published
#: rule and in a note. Never silently.
IDENTITY_MODELS = {"husky": (), "any_robot": ()}

#: Exception type names that are MuJoCo's ENGINE speaking rather than the
#: agent's program. ``FatalError`` / ``UnexpectedError`` are what the Python
#: bindings raise for a C-level ``mju_error``.
ENGINE_ERROR_TYPES = ("FatalError", "UnexpectedError")

_INJECTED_PREFIX = "_agentbench_"


# --- citations ---------------------------------------------------------------

_T0_SOURCE = (
    "the grader-owned recorder's frozen t=0 scan: taken inside the child "
    "process on the first observed mj_step, BEFORE that step integrates, on a "
    "deep copy of MjData so it cannot perturb the run. World-space AABBs are "
    "computed from model.geom_aabb rotated by data.geom_xmat and translated by "
    "data.geom_xpos -- MuJoCo has no bounds query, but it publishes every term "
    "the box needs, so the box is measured rather than absent")
_ROSTER_SOURCE = (
    "the same frozen t=0 scan, restricted to top-level bodies and to geoms "
    "attached directly to <worldbody> (MJCF's idiom for static scenery). "
    "MuJoCo has no scene-tree service and no live session: the recorder inside "
    "the run is the only scene reader that exists")
_CONTACT_SOURCE = (
    "the recorder's per-step read of data.ncon / data.contact. MuJoCo's "
    "contact record IS a geom pair, so both participants are named directly "
    "and no world-point re-pairing is involved. Each side is resolved to the "
    "TOP-LEVEL body of its kinematic subtree -- so a wheel striking a wall is "
    "reported as the robot striking the wall, as it is on the other two arms "
    "-- except for a geom attached directly to <worldbody>, which is reported "
    "under that GEOM's name because that is the only name such a wall or "
    "obstacle has. Pairs are deduped with their first witness; the two "
    "vacuity counters are totals over every scanned step")
_TRAJ_SOURCE = (
    "the grader-owned recorder: every body's world position (data.xpos) "
    "sampled once per completed integration step, in simulated seconds "
    "(data.time) and metres")
_LOG_SOURCE = (
    "the captured stdout+stderr of the child interpreter plus its exit code, "
    "and MuJoCo's own error class (a model-compile failure, or a "
    "FatalError/UnexpectedError raised out of the C library). WARNING: MuJoCo "
    "writes NO log file and has no engine log format -- it raises -- so there "
    "is no post-hoc equivalent of $OMNISIM_LOG_PATH to read")
_NO_LOG_SOURCE = (
    "NOT AVAILABLE for this run: MuJoCo writes no log file, and no "
    "stdout/stderr capture was found, so nothing the child printed can be read")
_STARTS_SOURCE = (
    "the recorder's own record of the ONE driver program the runner executed. "
    "MuJoCo starts no per-robot processes and MJCF declares no per-body "
    "behaviour, so a run has exactly one behaviour by construction and this "
    "count is 0 or 1 for the whole scene -- not a per-robot tally")
_VIEW_SOURCE = (
    "<camera> elements read from the compiled model at t=0 by the "
    "grader-owned recorder (data.cam_xpos, data.cam_xmat, model.cam_fovy), "
    "converted from MuJoCo's -z-forward / +y-up camera frame and degrees of "
    "VERTICAL field of view into the neutral world-frame unit vectors and "
    "radians")

_ATTRIB_SOURCE = (
    "read from the COMPILED MjModel inside the run: mjOption.solver, "
    ".integrator, .cone and .timestep plus mj_versionString(). This is a "
    "per-run reading of the actual configuration, not a structural claim about "
    "the product -- MuJoCo is its own physics engine, so there is no second "
    "backend a run could have silently landed on, and the solver it used is "
    "still recorded because 'MuJoCo' alone does not identify the numerics")


# --- notes carried on every MuJoCo verdict -----------------------------------

_NO_HARNESS_NOTE = (
    "MuJoCo exposes no authoring service, no scene-tree query, no hot reload "
    "and no structured load diagnostics: an MJCF file is inert data compiled by "
    "MjModel.from_xml_path, and everything else is a Python program. There is "
    "therefore no live-session pass on this arm; 'the artifact loaded' is "
    "evidenced by the model COMPILING in the child process, checked "
    "independently of whether the agent's driver worked.")
_NO_AUTOEXIT_NOTE = (
    "MuJoCo has no --duration, no auto-exit and no simulationQuit(): the "
    "agent's driver owns the stepping loop and ends when it decides to. The "
    "grader's recording window is enforced by raising out of mj_step (a "
    "BaseException, so an ordinary 'except Exception' in the driver cannot "
    "swallow it), which is this arm's equivalent of the in-world supervisor "
    "quitting the run.")
_ONE_DRIVER_NOTE = (
    "MJCF has no per-body controller field and MuJoCo starts no per-robot "
    "processes: ONE external Python program drives the whole scene. So the "
    "declared-behaviour channel is null on every body here -- not zero -- and "
    "any assertion that compares 'N behaviour fields' against 'N behaviour "
    "processes started' is not expressible per-body on this arm. What IS "
    "measured is whether that one program ran and whether it stepped the "
    "model.")
_NO_HUSKY_NOTE = (
    "neither MuJoCo nor MuJoCo Menagerie ships a Clearpath Husky, so the "
    "flagship identity predicate is not portable as written: it needs an "
    "explicitly declared analogue (robot_model=), which is recorded in the "
    "published rule and in this note, or it simply is not satisfied. The same "
    "asymmetry exists on the upstream-Webots arm and it cuts in OmniSim's "
    "favour, which is exactly why it is stated rather than buried.")


# --- presentation-only wording ----------------------------------------------
#
# Labels carry NO logic (graders/evidence.py). The identity and behaviour keys
# are spelled in MuJoCo's own words so a cross-sim table shows on its face that
# one column counted URDF url references, one counted PROTO instantiations and
# this one counted actuated subtrees.

_COMMON_LABELS = {
    "exit_code": "exit code",
    "error_lines": "MuJoCo error-class events",
    "timed_out": "timed out",
}

LABELS = {
    "A1_husky_swarm_10": dict(
        _COMMON_LABELS,
        artifact_missing_detail="no .xml model was produced",
        motion_absent_default="no recorder samples",
        identity_scene_count="actuated top-level bodies >= %d joints"
                             % MIN_JOINTS,
        identity_declared_count="matching <body> declarations in the model",
        identity_scene_threshold="robot bodies",
        identity_declared_threshold="<body> declarations",
        distinct_ids="distinct body keys",
        behaviour_fields="bodies with a declared behaviour (MJCF has no such "
                         "field)",
        behaviour_starts="driver programs started",
        behaviour_fields_threshold="behaviour fields",
        behaviour_starts_threshold="starts",
        driver_completed="the driver ran to the grader's window or returned",
        finalize_evidence="finalize evidence",
        attribution_threshold="an engine/solver attribution with a citation",
        attribution_missing_note=(
            "no physics-engine attribution: this run left no artifact showing "
            "that a MuJoCo model was ever compiled and stepped, so even "
            "'MuJoCo is its own engine' cannot honestly be applied to it. An "
            "unattributed physics result is not a result (SPEC A1.10)."),
    ),
    "C2_fall_through_floor": dict(
        _COMMON_LABELS,
        artifact_missing_detail="no .xml model",
        motion_absent_default="no samples",
        driver_completed="the driver ran to the grader's window or returned",
        dynamic="not welded to the world (body_weldid != 0)",
        no_body_note="the recorder saw no body at all",
        attribution_missing_note=(
            "no physics-engine attribution (SPEC A1.10 rule, applied "
            "suite-wide): this run left no evidence a MuJoCo model was "
            "compiled and stepped."),
    ),
    "B3_measure_and_report": dict(_COMMON_LABELS),
}


# --- artifact discovery ------------------------------------------------------


def find_model_artifacts(scratch_dir):
    """Every candidate ``.xml`` under ``scratch_dir``, newest first.

    The sibling of ``common.worldtext.find_world_artifacts``, which only knows
    about ``.wbt``. Grader-injected copies are excluded by the same filename
    prefix both other arms use.
    """
    scratch = Path(scratch_dir)
    if not scratch.exists():
        return []
    out = [p for p in scratch.rglob("*%s" % MODEL_SUFFIX)
           if not p.name.startswith(_INJECTED_PREFIX)]
    out.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return out


def pick_model(scratch_dir):
    """The MJCF the agent last worked on, or None. Same rule as the .wbt one."""
    cands = find_model_artifacts(scratch_dir)
    return cands[0] if cands else None


# --- MJCF text scanning (the DECLARATION half of identity) -------------------

_BODY_NAME = re.compile(r"<body\b[^>]*\bname\s*=\s*[\"']([^\"']+)[\"']")
_INCLUDE = re.compile(r"<include\b[^>]*\bfile\s*=\s*[\"']([^\"']+)[\"']")


def scan_mjcf(path):
    """``{body_names, includes, text_ok}`` from the artifact TEXT.

    Deliberately a text scan and not an XML parse: whether the file is
    well-formed is answered authoritatively elsewhere (the child compiles it
    with ``MjModel.from_xml_path`` and records the result), and a scan that
    threw on malformed XML would leave the declaration count unanswered exactly
    when the artifact is most interesting.
    """
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    return {"body_names": _BODY_NAME.findall(text),
            "includes": _INCLUDE.findall(text),
            "chars": len(text)}


# --- the adapter -------------------------------------------------------------


def build_bundle(task_id, *, robot_identity="any_robot", live_expected=False,
                 artifact=None, run=None, run_dir=None, scratch_dir=None,
                 mujoco_dir=None, robot_model=None, **_):
    """Read one MuJoCo run into a neutral bundle. **Never raises.**

    ``run`` is a :class:`~agentbench.adapters.mujoco.recording.MujocoRun`; pass
    it when the MuJoCo lane already parsed its own artifacts. Otherwise
    ``mujoco_dir`` (or ``$AGENTBENCH_MUJOCO_DIR``, or a ``mujoco/``
    subdirectory of ``run_dir``) is searched for them.

    ``robot_model`` names an explicitly declared **analogue** for the task's
    robot -- necessary because MuJoCo ships no Husky. Passing it is recorded in
    the published identity rule and in a note; omitting it means the flagship
    predicate simply is not satisfied, which is the honest answer.

    The ``harness`` / ``phase_b`` keywords the OmniSim graders pass are
    accepted and ignored: both are OmniSim-shaped and neither has a MuJoCo
    counterpart. ``scene_inventory`` is likewise accepted and ignored, and
    that is not a gap: it asks an adapter whose bounds scan is NAME-KEYED to
    bound every body instead of a handed list, which this arm's t=0 scan does
    unconditionally (BRINGUP.md §4.2). Ignoring it here means the same thing
    as honouring it; a future `False` would not mean the opposite, so do not
    read this as a flag being respected.
    """
    labels = dict(LABELS.get(task_id, _COMMON_LABELS))
    bundle = EvidenceBundle(task=task_id, sim=SIM, adapter=ADAPTER,
                            labels=labels)

    # -- locate the run ----------------------------------------------------
    tried = []
    if run is None:
        for cand in _run_dir_candidates(mujoco_dir, run_dir):
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
            % (", ".join(tried) if tried else "nowhere -- no run_dir, no "
                                              "mujoco_dir, no "
                                              "$AGENTBENCH_MUJOCO_DIR"))

    bundle.notes.append(_NO_HARNESS_NOTE)
    bundle.notes.append(_NO_AUTOEXIT_NOTE)
    bundle.notes.append(_ONE_DRIVER_NOTE)

    # -- the artifact ------------------------------------------------------
    if artifact is None and scratch_dir is not None:
        artifact = pick_model(scratch_dir)
    if artifact is None and run.model:
        artifact = run.model
    bundle.artifact = str(artifact) if artifact else None
    scanned = None
    if artifact is not None and Path(artifact).is_file():
        scanned = scan_mjcf(artifact)

    # -- frame -------------------------------------------------------------
    for note in _frame_notes(run.model_info):
        bundle.notes.append(note)

    rdoc = run.roster or {}
    robot_roots = set(int(r) for r in (rdoc.get("robot_roots") or []))
    driver_ran = bool((run.completion or {}).get("steps"))
    driver_name = ((run.completion or {}).get("driver")
                   or (run.process or {}).get("driver") or "")
    if driver_name:
        driver_name = Path(str(driver_name)).name

    # -- identity ----------------------------------------------------------
    accepted, ident = _identity(robot_identity, robot_model, scanned, bundle)
    bundle.identity = ident

    def robot_class(rec):
        return _robot_class(rec, robot_identity, accepted, robot_roots)

    # -- roster + the frozen t=0 inventory ---------------------------------
    #
    # ONE scan feeds both, as on the Webots arm: MuJoCo has no second channel.
    # The ROSTER is top-level bodies + world-attached geoms (what independent
    # objects the scene has); the t=0 inventory adds the LINKS, which are never
    # robot-class and would otherwise be counted as robots by every core that
    # counts inventory.robots.
    if run.roster is None:
        err = (run.errors.get("roster")
               or run.missing.get("roster", "no t=0 scan recorded"))
        bundle.roster = BodyInventory(source=_ROSTER_SOURCE, error=err)
        bundle.t0 = BodyInventory(source=_T0_SOURCE, error=err)
    else:
        behaviour = driver_name if driver_ran else None
        tops = [_body(rec, robot_class, behaviour)
                for rec in (rdoc.get("bodies") or [])
                if isinstance(rec, dict) and rec.get("top_level")]
        geoms = [_world_geom(rec) for rec in (rdoc.get("world_geoms") or [])
                 if isinstance(rec, dict)]
        links = [_link(rec) for rec in (rdoc.get("bodies") or [])
                 if isinstance(rec, dict) and not rec.get("top_level")]
        frozen, frozen_note = _frozen(rdoc)
        if frozen_note:
            bundle.notes.append(frozen_note)
        t_s = rdoc.get("t_s")
        t_s = float(t_s) if isinstance(t_s, (int, float)) else None
        bundle.roster = BodyInventory(bodies=tops + geoms, frozen=frozen,
                                      t_s=t_s, source=_ROSTER_SOURCE,
                                      error=rdoc.get("error"))
        bundle.t0 = BodyInventory(bodies=tops + geoms + links, frozen=frozen,
                                  t_s=t_s, source=_T0_SOURCE,
                                  error=rdoc.get("error"))
        for pn in (rdoc.get("plane_notes") or []):
            bundle.notes.append(
                "world-space bounds caveat -- %s. Every geometric assertion "
                "about that surface is measuring the drawn box, not the "
                "collision surface." % pn)
        if rdoc.get("forced_roots"):
            bundle.notes.append(
                "%d subtree(s) were classified robot-class from an OBSERVED "
                "applied force (qfrc_applied / xfrc_applied) rather than from "
                "a declared <actuator>: %s. The run showed them being driven; "
                "the file did not say so."
                % (len(rdoc["forced_roots"]),
                   "; ".join("body %s: %s" % (k, ", ".join(v))
                             for k, v in list(rdoc["forced_roots"].items())[:4])))
    bundle.adapter_measurements["roster"] = {
        "mujoco": {"top_level_bodies": len([b for b in bundle.roster.bodies
                                            if b.kind == "body"]),
                   "world_geoms": len([b for b in bundle.roster.bodies
                                       if b.kind == "world_geom"]),
                   "links": max(0, len(bundle.t0.bodies)
                                - len(bundle.roster.bodies)),
                   "robot_roots": sorted(robot_roots),
                   "frozen": bundle.roster.frozen,
                   "source_file": str(run.files.get("roster") or "")}}

    # -- contacts ----------------------------------------------------------
    #
    # ABSENT MEANS None. An empty ContactObservation would tell the core "the
    # query ran and found nothing", which is a different claim.
    if run.contacts is not None:
        bundle.contacts = _contacts(run.contacts, robot_roots)
        bundle.adapter_measurements["motion"]["contact_pairs"] = [
            {k: p.get(k) for k in ("a", "b", "a_detail", "b_detail",
                                   "self_pair", "step", "t_s")}
            for p in (run.contacts.get("pairs") or [])[:24]
            if isinstance(p, dict)]
        if run.contacts.get("pairs_truncated"):
            bundle.notes.append(
                "the distinct-contact-pair list hit its cap (%s) and is "
                "TRUNCATED; the two vacuity counters are totals and are not."
                % run.contacts.get("pair_cap"))
        stride = int(run.contacts.get("stride") or 1)
        if stride > 1:
            bundle.notes.append(
                "contacts were scanned every %d steps, not every step, so a "
                "contact that began and ended inside one stride window is not "
                "in this set. Direction of the bias: against detecting a hit."
                % stride)

    # -- trajectory --------------------------------------------------------
    if run.trajectory is None:
        bundle.motion_error = (
            run.errors.get("trajectory")
            or run.missing.get("trajectory")
            or labels.get("motion_absent_default", "no motion samples"))
    else:
        arr = recording.to_arrays(run.trajectory, run.csv_path)
        for p in arr.problems:
            bundle.notes.append("trajectory: %s" % p)
        if arr.fatal:
            bundle.motion_error = arr.fatal
            bundle.trajectory = Trajectory(source=_TRAJ_SOURCE,
                                           error=arr.fatal)
        else:
            bundle.trajectory = _trajectory(arr, bundle, run)

    # -- process facts -----------------------------------------------------
    bundle.process = _process(run, driver_name, driver_ran)
    warn = (run.completion or {}).get("warnings") or {}
    if warn:
        bundle.adapter_measurements["motion"]["mujoco_warnings"] = dict(warn)
        bundle.notes.append(
            "MuJoCo raised %d warning class(es) during this run: %s. They are "
            "RECORDED and NOT counted as error lines, for parity with the "
            "OmniSim and Webots adapters -- both of those count only their "
            "engine's ERROR: class and neither counts a divergence warning. "
            "Note that mjWARN_BADQACC additionally RESETS the state, so a run "
            "that hit it will show the discontinuity in its own pose series, "
            "where the physical assertions can see it."
            % (len(warn), ", ".join("%s x%s" % (k, v)
                                    for k, v in sorted(warn.items()))))
    n_resets = int((run.completion or {}).get("n_state_resets") or 0)
    if n_resets:
        bundle.notes.append(
            "MuJoCo RESET the simulation state %d time(s) during this run. "
            "That is what mj_step does when the solve diverges "
            "(mjWARN_BADQACC calls mj_resetData), and it has two consequences "
            "a reader must not miss: the pose series contains a physical "
            "DISCONTINUITY at %s, and the state after each one is the model's "
            "INITIAL state rather than a continuation. The series' time base "
            "is the recorder's accumulated clock, because data.time is set "
            "back to zero by the reset and a window keyed on it would never "
            "close." % (n_resets,
                        ", ".join("step %s" % r.get("step") for r in
                                  ((run.completion or {}).get("state_resets")
                                   or [])[:5])))

    tamper = (run.completion or {}).get("tamper") or []
    for t in tamper:
        bundle.notes.append("TAMPER: %s" % t)
    for n in ((run.completion or {}).get("notes") or []):
        bundle.notes.append("recorder: %s" % n)
    if (run.trajectory or {}).get("bodies_truncated"):
        bundle.notes.append(
            "the pose series is TRUNCATED to the first %s bodies of a larger "
            "scene; bodies beyond the cap have no row."
            % (run.trajectory or {}).get("body_cap"))

    # -- engine attribution ------------------------------------------------
    bundle.attribution = _attribution(run)
    if bundle.attribution is None:
        bundle.notes.append(
            "no engine attribution: nothing here evidences that a MuJoCo model "
            "was compiled and stepped (no model_info, no completion record, no "
            "process record). MuJoCo is its own physics engine, but that "
            "describes the product, not a directory of numbers -- so the run "
            "stays unattributed, which is INVALID rather than FAIL "
            "(SPEC A1.10).")

    # -- 'the artifact loaded' ---------------------------------------------
    #
    # The child compiles the agent's MJCF on its own, before the driver runs,
    # precisely so this question has an answer that does not depend on the
    # driver working.
    bundle.live_load_ok = run.compiled
    if run.compiled is False:
        bundle.notes.append(
            "the agent's MJCF did NOT compile: %s"
            % ((run.model_load or {}).get("error") or "reason not recorded"))

    if scanned is not None:
        bundle.adapter_measurements["identity"] = dict(
            bundle.adapter_measurements.get("identity") or {},
            body_declarations=len(scanned.get("body_names") or []),
            includes=list(scanned.get("includes") or [])[:8])
        if scanned.get("includes"):
            bundle.notes.append(
                "the model uses %d <include> file(s); MuJoCo resolves them "
                "relative to the including file, so this artifact is not "
                "self-contained and a collector that moves it must move them "
                "too." % len(scanned["includes"]))
    return bundle


# --- run location -----------------------------------------------------------


def _run_dir_candidates(mujoco_dir, run_dir):
    """Where to look for the MuJoCo lane's artifacts, most explicit first."""
    out = []
    for c in (mujoco_dir, os.environ.get("AGENTBENCH_MUJOCO_DIR")):
        if c:
            out.append(Path(c))
    if run_dir:
        rd = Path(run_dir)
        out += [rd / "mujoco", rd / "phaseM", rd]
    seen, uniq = set(), []
    for p in out:
        if str(p) not in seen:
            seen.add(str(p))
            uniq.append(p)
    return uniq


# --- identity ---------------------------------------------------------------


def _identity(robot_identity, robot_model, scanned, bundle):
    """The accepted model set and the published :class:`IdentityRule`."""
    accepted = tuple(IDENTITY_MODELS.get(robot_identity, ()))
    substitute = str(robot_model).strip() if robot_model else ""
    if substitute:
        accepted = accepted + (substitute,)

    base_rule = (
        "MJCF has no Robot node -- <body> describes a manipulator link, a "
        "falling crate and a wall alike -- so 'robot' is DECIDED here rather "
        "than read: a body is robot-class when it is TOP-LEVEL (its parent is "
        "the world body) and something can drive its kinematic subtree. Two "
        "channels count, and both are published per body: (a) an <actuator> "
        "whose transmission resolves into the subtree -- joint, "
        "jointinparent, tendon, site, slidercrank or adhesion-body, all "
        "followed; (b) a non-zero qfrc_applied / xfrc_applied OBSERVED on the "
        "subtree during the run, which catches a robot driven by applied "
        "forces that the file never declares. A body inside such a subtree is "
        "a LINK (member_of), never a robot; a free-floating body nothing "
        "drives is scenery. Known failure modes are published in BRINGUP.md "
        "sec. 4: an actuated conveyor counts as a robot, and a body moved only "
        "by direct qpos/qvel writes does not.")

    if robot_identity == "husky":
        bundle.notes.append(_NO_HUSKY_NOTE)
        scene_rule = (
            base_rule + " For the flagship predicate the body must ALSO carry "
            ">= %d joints (mjJNT_FREE excluded: a floating base is not an "
            "articulation and the other arms have no joint node for it) and "
            "its name must be one of {%s}."
            % (MIN_JOINTS, ", ".join(accepted) or "-- nothing accepted --"))
        if substitute:
            scene_rule += (
                " DECLARED ANALOGUE: %r was accepted as a substitute for the "
                "task's robot because MuJoCo ships no Husky. The task is being "
                "scored by OUTCOME with each simulator using its native robot, "
                "and this is the substitution that was made." % substitute)
        declaration_rule = (
            "<body> elements in the artifact text whose name attribute is one "
            "of {%s}. Unlike the OmniSim side -- where the engine expands "
            "URDFRobot { url ... } at parse time and the model name survives "
            "ONLY in the file -- MJCF carries no model identity at all beyond "
            "the names the author chose, so this count corroborates a NAME and "
            "nothing more."
            % (", ".join(accepted) or "-- nothing accepted --"))
    else:
        scene_rule = base_rule
        declaration_rule = ("not counted: this task ships the world, so "
                            "counting declarations would grade our own "
                            "fixture")

    declared = None
    if scanned is not None and robot_identity == "husky":
        low = {a.lower() for a in accepted}
        names = [n for n in (scanned.get("body_names") or [])
                 if n.lower() in low]
        declared = len(names)
        bundle.adapter_measurements["identity"] = {
            "matching_body_declarations": declared,
            "body_names_seen": sorted(set(scanned.get("body_names") or []))[:12],
            "accepted_models": list(accepted),
            "declared_analogue": substitute or None,
        }
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


def _robot_class(rec, robot_identity, accepted, robot_roots):
    """Is this top-level body a robot? True / False -- never None.

    MuJoCo answers both halves of this question completely: the model states
    every actuator's transmission and the recorder watched every applied force,
    so "nothing drives this body" is evidence of absence rather than absence of
    evidence. There is no unanswerable case to return ``None`` for, which is
    why this returns a bool where the Webots adapter cannot.
    """
    if not rec.get("top_level"):
        return False
    driven = int(rec.get("id") or -1) in robot_roots
    if robot_identity != "husky":
        return bool(driven)
    if not accepted:
        # Nothing can satisfy the predicate: MuJoCo ships no such robot and no
        # analogue was declared. A definite negative, not an unknown.
        return False
    name = str(rec.get("name") or "").lower()
    return bool(driven and name in {a.lower() for a in accepted}
                and (rec.get("n_joints") or 0) >= MIN_JOINTS)


# --- bodies -----------------------------------------------------------------


def _body(rec, robot_class, behaviour):
    """One top-level MuJoCo body."""
    rc = robot_class(rec)
    return Body(
        body_id=str(rec.get("key") or ""), name=str(rec.get("name") or ""),
        kind="body",
        position=recording.triple(rec.get("position")),
        aabb_min=recording.triple(rec.get("aabb_min")),
        aabb_max=recording.triple(rec.get("aabb_max")),
        n_joints=(int(rec["n_joints"])
                  if isinstance(rec.get("n_joints"), (int, float)) else None),
        # The EFFECTIVE behaviour is the one driver program, and only when it
        # was observed stepping. The DECLARED slot stays None because MJCF has
        # no field to declare one -- see _ONE_DRIVER_NOTE.
        behaviour=(behaviour if rc else None), behaviour_declared=None,
        dynamic=(bool(rec["movable"]) if rec.get("movable") is not None
                 else None),
        robot_class=rc,
        identity_evidence=(
            "top-level body #%s, %s joint(s) excluding a floating base, "
            "mass %.4g kg, %s; driven-by: %s (grader-owned t=0 scan)"
            % (rec.get("id"), rec.get("n_joints"),
               float(rec.get("mass_kg") or 0.0),
               "movable" if rec.get("movable") else "welded to the world",
               "; ".join(rec.get("actuated_reasons") or []) or "nothing "
               "declared in the model")))


def _link(rec):
    """One body INSIDE another body's kinematic subtree.

    ``robot_class`` is hard ``False``: the identity predicate asks "is this
    body the robot the task named", and a forearm is part of that robot without
    being it. Membership travels in ``member_of`` instead, so every core that
    counts ``inventory.robots`` keeps counting arms rather than arm segments.
    """
    return Body(
        body_id=str(rec.get("key") or ""), name=str(rec.get("name") or ""),
        kind="body",
        position=recording.triple(rec.get("position")),
        aabb_min=recording.triple(rec.get("aabb_min")),
        aabb_max=recording.triple(rec.get("aabb_max")),
        dynamic=(bool(rec["movable"]) if rec.get("movable") is not None
                 else None),
        robot_class=False, member_of=rec.get("parent_key"),
        identity_evidence=("body #%s inside the subtree rooted at body #%s "
                           "(grader-owned t=0 scan)"
                           % (rec.get("id"), rec.get("root"))))


def _world_geom(rec):
    """One geom attached directly to ``<worldbody>``.

    MJCF's idiom for static scenery -- a floor, a wall, an obstacle box -- is a
    bare ``<geom>`` on ``<worldbody>``, i.e. geometry on the *world* body
    rather than a body of its own. Each one is an independent object of the
    scene and gets its own inventory entry; folding them into one ``world``
    body would report five obstacles as a single object whose AABB spans the
    arena.
    """
    return Body(
        body_id=str(rec.get("key") or ""), name=str(rec.get("name") or ""),
        kind="world_geom",
        position=recording.triple(rec.get("position")),
        aabb_min=recording.triple(rec.get("aabb_min")),
        aabb_max=recording.triple(rec.get("aabb_max")),
        n_joints=0, behaviour=None, behaviour_declared=None,
        dynamic=False, robot_class=False,
        identity_evidence=("a %s geom attached directly to <worldbody>: "
                           "static scene geometry with no body of its own "
                           "(grader-owned t=0 scan)"
                           % (rec.get("geom_type") or "?")))


# --- contacts ---------------------------------------------------------------


def _contacts(doc, robot_roots):
    """One :class:`ContactObservation`. Witness counters are never invented."""
    pairs = []
    for c in (doc.get("pairs") or []):
        if not isinstance(c, dict):
            continue
        pairs.append(ContactPair(
            a=c.get("a") or None, b=c.get("b") or None,
            a_robot=int(c.get("a_root") or 0) in robot_roots,
            b_robot=int(c.get("b_root") or 0) in robot_roots,
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


def _opt_int(v):
    if isinstance(v, bool) or v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


# --- trajectory -------------------------------------------------------------


def _trajectory(arr, bundle, run):
    """The pose series. ``body_ids`` are the SAME keys the inventories use.

    The index invariant the neutral schema asks for is satisfied structurally
    rather than by a permutation: the recorder writes one key space
    (``body:<id>`` / ``geom:<id>``) for the inventory and the pose series
    alike, so a core that resolves a body through the roster and then calls
    ``trajectory.index_of(body.body_id)`` gets that body by construction. No
    name matching is involved, which matters because MJCF names are optional
    and may repeat across element types.
    """
    doc = run.trajectory or {}
    bts = doc.get("model_timestep_s")
    if bts and arr.dt_s and arr.dt_s > 1.5 * float(bts):
        bundle.notes.append(
            "the pose series is DOWNSAMPLED: one sample every %.4g ms while "
            "the model's own timestep is %.4g ms (a stride of ~%d steps). The "
            "driver called mj_step with nstep>1, so a sample is the end of a "
            "BATCH of integrations rather than of one. Integrated path length "
            "over a stride is a LOWER bound and the largest single-sample "
            "displacement is an UPPER one, so both of R1.6's floors are being "
            "applied to a different quantity than on a per-step arm. Direction "
            "of the bias on path length: against this run."
            % (arr.dt_s * 1000.0, float(bts) * 1000.0,
               max(1, int(round(arr.dt_s / float(bts))))))
    src = _TRAJ_SOURCE
    if run.csv_path:
        src += " (%s)" % Path(run.csv_path).name
    complete = arr.complete
    if complete is None:
        complete = bool((run.completion or {}).get("complete"))
    return Trajectory(body_ids=list(arr.keys), t=arr.t, xyz=arr.xyz, vel=None,
                      dt_s=arr.dt_s, recorded_s=arr.recorded_s,
                      complete=bool(complete), source=src,
                      kinds=list(arr.kinds), parents=list(arr.parents),
                      names=list(arr.names))


# --- process ----------------------------------------------------------------


def _error_lines(run):
    """MuJoCo's own error class for this run, as lines.

    **What is counted, and the direction the choice leans.** MuJoCo has no log
    file and no ``ERROR:`` prefix; it raises. Two things here are the ENGINE
    speaking rather than the agent's program, and both are counted:

    * a **model-compile failure** -- ``MjModel.from_xml_path`` refusing the
      artifact. This is the exact counterpart of the OmniSim / Webots
      "the world failed to load" ERROR: lines;
    * a ``FatalError`` / ``UnexpectedError``, which is what the Python bindings
      raise for a C-level ``mju_error``.

    What is deliberately **not** counted: MuJoCo's ``mjWARN_*`` counters --
    including ``mjWARN_BADQACC``, which fires when the solve diverges and which
    additionally RESETS the state. Counting a divergence warning as an error
    line would make this assertion stricter on MuJoCo than the same assertion
    is on OmniSim or upstream Webots, neither of which counts its engine's
    divergence warnings, and a comparison whose thresholds lean towards our own
    simulator is not evidence of anything. The warnings are recorded and noted
    on every verdict instead -- and a run that hit ``BADQACC`` shows the state
    reset as a discontinuity in its own pose series, where R1.6's teleport
    bound and every other physical floor can see it. It is visible; it is just
    not double-counted in the log channel.

    A traceback from the agent's own driver is also not counted here: that is
    the competitor's *controller* crashing, which on the other arms shows up as
    a non-zero exit rather than as an engine error line -- and it does here
    too.
    """
    out = []
    load = run.model_load or {}
    if load.get("error"):
        out.append("MJCF compile error: %s" % load["error"])
    err = (run.completion or {}).get("driver_error") or ""
    if any(err.startswith(t + ":") for t in ENGINE_ERROR_TYPES):
        out.append("MuJoCo engine error: %s" % err)
    return out


def _process(run, driver_name, driver_ran):
    """:class:`ProcessFacts`, or None when nothing about the process is known."""
    pdoc = run.process or {}
    cdoc = run.completion or {}
    have_console = run.console_text is not None
    if not pdoc and not cdoc and not run.model_load and not have_console:
        return None

    errs = _error_lines(run)
    driver_completed = bool(cdoc.get("complete")) if cdoc else None

    compiled = run.compiled
    stepped = bool(cdoc.get("steps"))
    if compiled and stepped:
        reached = True
        why = ("the agent's MJCF compiled into an MjModel and the recorder "
               "observed %s integration step(s) on it. MuJoCo has no "
               "world-finalize event -- compiling the model and stepping it IS "
               "the whole of 'the world was built and stepped'"
               % cdoc.get("steps"))
    elif compiled is False:
        reached = False
        why = ("the agent's MJCF did not compile, so no world was built: %s"
               % (run.model_load or {}).get("error"))
    elif compiled and not stepped:
        reached = False
        why = ("the model compiled but no integration step was ever observed: "
               "the driver %s" % (("raised: %s" % cdoc.get("driver_error"))
                                  if cdoc.get("driver_error")
                                  else "never called mj_step through a path "
                                       "this recorder wraps (MJX, "
                                       "mujoco.rollout and native callers are "
                                       "invisible to it)"))
    else:
        reached = None
        why = ("neither a model-compile record nor a recorder completion "
               "record: 'was a world built and stepped' is unanswered, not "
               "answered no")

    if driver_name and (cdoc or pdoc):
        starts = {driver_name: 1}
    elif cdoc or pdoc:
        starts = {}          # counted: this run executed no driver at all
    else:
        starts = None        # could not count

    return ProcessFacts(
        exit_code=_opt_int(pdoc.get("exit_code")),
        timed_out=bool(pdoc.get("timed_out")),
        error_lines=errs,
        log_available=have_console,
        log_source=_LOG_SOURCE if have_console else _NO_LOG_SOURCE,
        behaviour_starts=starts,
        start_source=_STARTS_SOURCE if starts is not None else "",
        driver_completed=driver_completed,
        reached_finalize=reached, finalize_evidence=why,
        wall_s=pdoc.get("wall_s"),
        attempts_used=int(pdoc.get("attempts_used") or 1))


# --- attribution ------------------------------------------------------------


def _attribution(run):
    """SPEC A1.10 on MuJoCo: the compiled model's own option block, with a cite.

    Asserted only for a run that left evidence a MuJoCo model was compiled and
    observed. "MuJoCo is its own engine" is a true statement about the
    *product*; applying it to a directory of numbers would be attributing a run
    that may never have taken place, and A1.10's rule is that an unattributed
    physics result is INVALID rather than FAILED.

    Note what this arm can say that the Webots arm cannot: the solver, the
    integrator and the friction cone are READ from the model that ran. MuJoCo
    has one engine but several numerics, and "MuJoCo" alone does not identify
    which -- an mjSOL_PGS / mjINT_EULER / pyramidal run and an mjSOL_NEWTON /
    mjINT_IMPLICITFAST / elliptic run are not the same physics, and a
    cross-release comparison that did not record which is not a comparison with
    itself.
    """
    info = run.model_info or {}
    observed = bool(info.get("observed"))
    if not observed and not (run.completion or {}).get("steps"):
        return None
    solver = info.get("solver")
    if not solver:
        return None
    extra = {k: info.get(k) for k in
             ("integrator", "cone", "jacobian", "timestep_s", "iterations",
              "gravity", "impratio", "mujoco_version",
              "mujoco_python_version", "model_name", "nbody", "ngeom", "njnt",
              "nu")}
    extra["engine_invariant"] = True
    return EngineAttribution(backend="mujoco", solver=solver, degraded=False,
                             source=_ATTRIB_SOURCE, extra=extra)


# --- frame ------------------------------------------------------------------


def _frame_notes(info):
    """Notes about the world's gravity axis. **No remapping is ever applied.**

    The neutral frame is "z is the axis the world's own gravity acts along".
    MuJoCo's gravity is a free 3-vector, not a named convention, so there is no
    permutation that generally makes an arbitrary one z-down: a world with
    gravity along y would need a rotation, and rotating a measurement to make
    it pass is not a translation, it is a rewrite. So a non-z gravity is stated
    loudly and left alone -- a reader can then see that the vertical assertions
    in that row measured the wrong axis, instead of trusting a silent fix.
    """
    g = (info or {}).get("gravity")
    if not isinstance(g, (list, tuple)) or len(g) != 3:
        return []
    gx, gy, gz = (float(v) for v in g)
    mag = math.sqrt(gx * gx + gy * gy + gz * gz)
    if mag < 1e-9:
        return ["the model declares ZERO gravity, so nothing falls in it. "
                "Every assertion about resting, landing or falling in this row "
                "is about a world where those words do not apply."]
    if abs(gz) >= max(abs(gx), abs(gy)):
        return []
    return ["the model's gravity is (%g, %g, %g) -- NOT dominated by the z "
            "axis. The neutral schema's frame is 'z is the axis gravity acts "
            "along', and no remapping was applied (a general rotation is not a "
            "relabelling). Every vertical assertion in this row is therefore "
            "measuring the wrong axis, and that is stated rather than silently "
            "corrected." % (gx, gy, gz)]


def _frozen(rdoc):
    """``frozen`` for the inventories -- never upgraded, only downgraded.

    The scan runs inside the child process on the first observed ``mj_step``,
    *before* that step integrates and on a copy of ``MjData``, so it genuinely
    is a t=0 read -- but only if the driver had not already stepped the model
    through a path the recorder does not wrap. The recorder reports what it
    saw; this refuses the claim whenever the recorded scan time is not zero.
    """
    frozen = rdoc.get("frozen")
    frozen = None if frozen is None else bool(frozen)
    t_s = rdoc.get("t_s")
    if frozen and isinstance(t_s, (int, float)) and float(t_s) != 0.0:
        return False, (
            "the recorder claimed a frozen t=0 scan but reports scan time "
            "%g s, so the model had already been advanced before the first "
            "step it could observe; the claim is refused and the inventory is "
            "marked un-frozen." % float(t_s))
    if frozen is None:
        return None, (
            "the recorder did not state whether its scan was taken before any "
            "integration, so 'frozen' is unanswered rather than assumed; no "
            "'at t=0' assertion can rest on it.")
    return frozen, None


# --- the camera channel (graders/b2.py's VIEW_HOOK) -------------------------


def build_view_evidence(task_id, *, bundle=None, artifact=None, run_dir=None,
                        scratch_dir=None, mujoco_dir=None, initial_run=None,
                        **_):
    """The B2 camera channel for a MuJoCo run. Never raises.

    ``final`` is the pose of the artifact's camera as the run compiled it.
    ``initial`` is the pose the TASK shipped -- and on this arm that requires a
    task-supplied MJCF fixture, because AgentBench's tasks ship ``.wbt``
    worlds. Until one exists, ``initial`` is absent with the reason stated;
    inventing a "default MuJoCo camera" would be fabricating the baseline the
    whole assertion is measured against.

    A model with several ``<camera>`` elements reports the first, and says so.
    A model with none reports no pose: MuJoCo's interactive free camera is not
    part of the model, is not in ``MjData`` and does not exist in a headless
    run, so "the scene's camera" is a scene element or it is nothing.
    """
    ev = ViewEvidence(artifact=str(artifact) if artifact else None,
                      source=_VIEW_SOURCE)
    run = None
    for cand in _run_dir_candidates(mujoco_dir, run_dir):
        got = recording.read_run(cand)
        if got.any_evidence:
            run = got
            break
    if run is None:
        ev.error = ("no MuJoCo run artifacts, so no camera pose could be read; "
                    "the pose comes from the compiled model inside the run")
        return ev
    if ev.artifact is None and run.model:
        ev.artifact = str(run.model)
    ev.artifact_parsed = run.compiled

    cams = (run.roster or {}).get("cameras") or []
    if not cams:
        ev.error = ("the model declares no <camera>. MuJoCo's interactive free "
                    "camera is not part of the model and does not exist in a "
                    "headless run, so this scene has no camera pose to read.")
        return ev
    src = "the agent's model under " + _VIEW_SOURCE
    if len(cams) > 1:
        named = ", ".join(str(c.get("name") or "#%s" % c.get("id"))
                          for c in cams[:4])
        src += (" -- the model declares %d cameras (%s); the FIRST is "
                "reported and the others are not" % (len(cams), named))
    ev.final = _camera(cams[0], src)
    if initial_run is not None:
        icams = (recording.read_run(initial_run).roster or {}).get("cameras") \
            or []
        if icams:
            ev.initial = _camera(icams[0],
                                 "the task's shipped model under "
                                 + _VIEW_SOURCE)
    if ev.initial is None:
        ev.error = ("no initial camera pose: AgentBench's tasks ship .wbt "
                    "worlds, so there is no MuJoCo fixture for this task to "
                    "read a shipped pose from. B2 is not gradeable on this arm "
                    "until one exists -- which is a missing FIXTURE, not a "
                    "missing MuJoCo capability.")
    return ev


def _camera(rec, source):
    """One recorded camera -> a neutral :class:`CameraPose`.

    ``cam_fovy`` is the FULL VERTICAL angle in degrees; MuJoCo states no
    horizontal angle, and the aspect ratio only exists when the camera declares
    a ``resolution``. Both are passed through as they are so
    ``CameraPose.half_angles`` can say how it resolved the missing axis rather
    than having a guess baked in here.
    """
    res = rec.get("resolution")
    aspect = None
    if isinstance(res, (list, tuple)) and len(res) == 2 and res[1]:
        aspect = float(res[0]) / float(res[1])
    fovy = rec.get("fovy_deg")
    return CameraPose(
        position=recording.triple(rec.get("position")),
        forward=recording.triple(rec.get("forward")),
        up=recording.triple(rec.get("up")),
        fov_h_rad=None,
        fov_v_rad=math.radians(float(fovy)) if fovy is not None else None,
        aspect=aspect, source=source)


__all__ = ["ADAPTER", "ENGINE_ERROR_TYPES", "IDENTITY_MODELS", "LABELS",
           "MIN_JOINTS", "MODEL_SUFFIX", "SIM", "build_bundle",
           "build_view_evidence", "find_model_artifacts", "pick_model",
           "scan_mjcf"]
