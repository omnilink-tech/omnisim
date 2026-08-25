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

"""OmniSim -> :class:`EvidenceBundle`. **This file is ADAPTER code.**

It is the only module in the suite that knows a ``URDFRobot`` from a hole in the
ground, that has heard of ``husky.urdf``, or that can find a ``.newton.json``
sidecar -- which is why it lives here and not under ``graders/``. (It was parked
in ``graders/evidence_omnisim.py`` when the grader split landed, because
``adapters/omnisim/**`` was owned by another lane at the time; the move here was
a file rename plus the registry line in ``adapters/__init__.py``, and nothing
else imported it.)

Everything OmniSim-shaped that the graders used to read is now on this side of
the line:

===============================  ==========================================
was, in the grader               is, here
===============================  ==========================================
``type in {URDFRobot, Robot}``   ``IdentityRule.scene_rule`` -> ``Body.robot_class``
``num_joints >= 4``              same
``husky.urdf`` refs in the text  ``IdentityRule.declared_count``
``<log>.newton.json`` sidecar    ``EngineAttribution`` + its citation
``ERROR:`` line regex            ``ProcessFacts.error_lines``
``Starting controller:`` regex   ``ProcessFacts.behaviour_starts``
``[WbNewtonBackend] world        ``ProcessFacts.reached_finalize`` /
finalised`` marker               ``.finalize_evidence``
``has_physics`` (Physics node)   ``Body.dynamic``
harness ``def`` / ``#id``        ``Body.body_id``
===============================  ==========================================

The ``LABELS`` maps below are **presentation only** and exist for one reason:
the v0 rows already published for OmniSim spell several measured keys in
OmniSim's own words, and this refactor must not silently rewrite a published
row's keys. They carry no logic -- delete them and the verdicts are identical
except for the wording of a few dict keys.
"""

from __future__ import annotations

import math
import re
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from agentbench.adapters.omnisim import headless  # noqa: E402
from agentbench.common import worldtext  # noqa: E402
from agentbench.graders.evidence import (  # noqa: E402
    Body, BodyInventory, CameraPose, ContactObservation, ContactPair,
    EngineAttribution, EvidenceBundle, IdentityRule, ProcessFacts, Trajectory,
    ViewEvidence)

SIM = "omnisim"
ADAPTER = "agentbench.adapters.omnisim.evidence"

ROBOT_TYPES = ("Robot", "URDFRobot")
MIN_JOINTS = 4

# --- the identity predicates, answered in OmniSim's own terms ---------------
#
# The task names the predicate ("husky"); this adapter says how it decided.
# Published with the scaffolding (SPEC 6.2.6), so a reader can see that the
# OmniSim and the Webots answers to "is this a Husky" are different questions
# with the same intent.

_IDENTITY = {
    "husky": dict(
        label="Husky",
        requirement="a distinct robot-class body that is the robot the task "
                    "named",
        scene_rule="a scene node whose type is Robot or URDFRobot carrying "
                   ">= %d joints. OmniSim expands URDFRobot { url ... } into a "
                   "plain Robot at parse time "
                   "(WbUrdfImporter::expandUrdfRobotBlocks), so the loaded "
                   "node cannot carry the model name and the joint count is "
                   "the only structural signal left in the live scene."
                   % MIN_JOINTS,
        declaration_rule="top-level Robot/URDFRobot blocks in the artifact "
                         "text whose url basename is husky.urdf (the "
                         "reference survives only in the file, because the "
                         "engine expands it away at parse time)"),
    "any_robot": dict(
        label="robot",
        requirement="the robot body the task is about",
        scene_rule="a scene node whose type is Robot, enumerated by node type "
                   "+ name and never by DEF (SPEC 4.5.1: the agent has no "
                   "reason to add DEFs)",
        declaration_rule="not counted: this task ships the world, so counting "
                         "declarations would grade our own fixture"),
}


# --- presentation-only wording (see the module docstring) -------------------

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
        identity_scene_count="robot_nodes_ge_%d_joints" % MIN_JOINTS,
        identity_declared_count="husky.urdf refs in the artifact",
        identity_scene_threshold="robot_nodes",
        identity_declared_threshold="husky.urdf refs",
        distinct_ids="distinct node ids",
        behaviour_fields="non-empty controller fields",
        behaviour_starts="controller processes started",
        behaviour_fields_threshold="controller fields",
        behaviour_starts_threshold="starts",
        driver_completed="recorder reached quit(0)",
        finalize_evidence="finalize evidence",
        attribution_threshold="a sidecar verdict or an explicit pin",
        attribution_missing_note=(
            "no physics-backend attribution: the .newton.json sidecar is "
            "absent and the world does not pin ODE. An unattributed physics "
            "result is not a result (SPEC A1.10)."),
    ),
    "C2_fall_through_floor": dict(
        _COMMON_LABELS,
        artifact_missing_detail="no .wbt artifact",
        motion_absent_default="no samples",
        driver_completed="recorder complete",
        dynamic="Physics node attached",
        no_body_note="the recorder saw no Robot node at all",
        attribution_missing_note=(
            "no physics-backend attribution (SPEC A1.10 rule, applied "
            "suite-wide): sidecar absent and no explicit ODE pin."),
    ),
    "B3_measure_and_report": dict(_COMMON_LABELS),
}

# The provenance note A1 carries on every verdict: WHY the t=0 half of A1.3 is
# not measured where the SPEC puts it. Measured on a 10-Husky world: GET
# /robots costs 22 s of free-running sim, by which time every robot has driven
# >10 m from spawn.
_A1_T0_NOTE = ("A1.3 t=0 geometry measured by the grader-owned recorder: the "
               "harness free-runs and cannot report t=0.")

_HARNESS_SOURCE = ("GET /robots on a per-run private harness instance "
                   "(free-running: static fields only, no t=0 question is "
                   "answerable here)")
_RECORDER_ROSTER_SOURCE = ("the grader-owned recorder's roster, enumerated by "
                           "node type + name at t=0")
_RECORDER_T0_SOURCE = ("the grader-owned recorder's frozen t=0 scan "
                       "(synchronized supervisor; geometry.bounds_for_subtree, "
                       "the same world-space AABB an agent gets from GET "
                       "/scene/tree?bounds=1)")
_SCENE_SCAN_SOURCE = ("the grader-owned recorder's NAME-FREE t=0 scene scan: "
                      "every non-robot Solid-derived body outside a Robot "
                      "subtree, bounded with geometry.bounds_for_subtree. No "
                      "name list is involved, so a world that calls its boxes "
                      "'crate A' is as visible as one using the published "
                      "names")
_CONTACT_SOURCE = ("the recorder's per-step contact scan "
                   "(harness_supervisor.observe.list_contacts, world-point "
                   "pairing), filtered to pairs whose two participants resolve "
                   "to DISTINCT Robot subtrees")
_STARTS_SOURCE = ("'INFO: <name>: Starting controller:' lines in the engine "
                  "log at $OMNISIM_LOG_PATH")
_LOG_SOURCE = ("the per-run engine log at $OMNISIM_LOG_PATH plus the process "
               "exit code from the headless launch")
# The bracketed tag is named after the emitting C++ class, and those classes are
# being renamed Wb* -> Om*. Accept BOTH prefixes permanently so this evidence
# read is correct on either side of the rename, and logs captured before it keep
# parsing forever.
_FINALIZE_MARKER_RE = re.compile(r"\[(?:Wb|Om)NewtonBackend\] world finalised")


def build_bundle(task_id, *, robot_identity="any_robot", live_expected=False,
                 artifact=None, harness=None, phase_b=None, scratch_dir=None,
                 run_dir=None, scene_inventory=False, **_):
    """Read one OmniSim run into a neutral bundle. Never raises.

    ``harness`` is a ``HarnessProbe`` (the live Phase-A pass) and ``phase_b`` a
    ``PhaseBResult`` (the standalone, cold, recorder-driven run). Both are
    OmniSim-shaped and neither escapes this module.

    ``scene_inventory`` opts this run's bundle into the recorder's NAME-FREE
    scene scan: every non-robot body it found, with world-space bounds, added
    to BOTH inventories. It is **off by default and that is a contract, not
    timidity** -- ``a1_core`` publishes ``robots_seen = len(roster.bodies)``
    and ``c1_core`` publishes "bodies beyond the intended roster" from
    ``t0.names``, so quietly handing either task a floor and four walls would
    rewrite a frozen row's measurements without changing a single physical
    fact. A grader that matches by GEOMETRY asks for it (``graders/r1.py``);
    one that counts a fixed roster does not.
    """
    labels = dict(LABELS.get(task_id, _COMMON_LABELS))
    if artifact is None and scratch_dir is not None:
        artifact = worldtext.pick_artifact(scratch_dir)

    bundle = EvidenceBundle(task=task_id, sim=SIM, adapter=ADAPTER,
                            artifact=str(artifact) if artifact else None,
                            labels=labels)
    if live_expected:
        bundle.notes.append(_A1_T0_NOTE)
    bundle.live_load_ok = bool(harness is not None and harness.ok)

    # --- identity -------------------------------------------------------
    spec = _IDENTITY.get(robot_identity) or _IDENTITY["any_robot"]
    declared = None
    if artifact is not None and robot_identity == "husky":
        try:
            declared = len(worldtext.husky_robot_blocks(artifact))
        except OSError:
            declared = None
        bundle.adapter_measurements["identity"] = {
            "husky_url_blocks": declared}
    bundle.identity = IdentityRule(declared_count=declared, **spec)

    def is_robot(kind, n_joints, base_type=None):
        # A Supervisor reports the PROTO name from getTypeName() (for example
        # ScaleBot), not the base node it instantiates.  Recorder evidence
        # carries getBaseTypeName() as well, so a Robot-derived PROTO remains
        # a robot without teaching this neutral adapter every PROTO name.
        if kind not in ROBOT_TYPES and base_type not in ROBOT_TYPES:
            return False
        if robot_identity == "husky":
            return (n_joints or 0) >= MIN_JOINTS
        return True

    # --- roster: the live harness pass, else the recorder ----------------
    hrobots = list(harness.robots) if (harness and harness.ok) else []
    if harness is not None:
        bundle.adapter_measurements["roster"] = {
            "harness": {"ok": harness.ok, "error": harness.error,
                        "timings_s": harness.timings,
                        "n_robots": len(hrobots)}}
    if harness is not None and harness.ok:
        bundle.roster = BodyInventory(
            bodies=[_body_from_harness(r, is_robot) for r in hrobots],
            frozen=False, source=_HARNESS_SOURCE)
    else:
        if live_expected and artifact is not None:
            bundle.notes.append(
                "harness pass unavailable (%s); the structural assertions "
                "fall back to the recorder roster"
                % (harness.error if harness else "not run"))
        bundle.roster = BodyInventory(
            bodies=[_body_from_recorder(r, is_robot)
                    for r in ((phase_b.roster if phase_b else []) or [])],
            frozen=True, t_s=0.0, source=_RECORDER_ROSTER_SOURCE)

    # --- the frozen t=0 inventory + contacts ----------------------------
    pa = (phase_b.phase_a if phase_b else None) or None
    t0_error = None
    if phase_b is None:
        t0_error = "phase B not run"
    elif phase_b.error:
        t0_error = phase_b.error
    elif pa is None:
        t0_error = "no t=0 scan"
    if pa is None:
        bundle.t0 = BodyInventory(source=_RECORDER_T0_SOURCE, error=t0_error)
    else:
        bodies = [_body_from_t0(e, is_robot) for e in pa.get("t0_robots", [])]
        bodies += [_solid_from_t0(e) for e in (pa.get("t0_solids") or [])]
        # Links come LAST and are never robot-class: the t=0 inventory is what
        # R3's cube search and B1's overlap audit walk, and a link that read as
        # a robot would be counted as one there.
        bodies += [_link_from_t0(e) for e in (pa.get("t0_links") or [])]
        bundle.t0 = BodyInventory(bodies=bodies, frozen=True, t_s=0.0,
                                  source=_RECORDER_T0_SOURCE, error=t0_error)
        _cw = pa.get("contact_witness") or None
        bundle.contacts = ContactObservation(
            pairs=[ContactPair(a=c.get("a_robot"), b=c.get("b_robot"),
                               a_robot=True, b_robot=True,
                               point=c.get("point"), step=c.get("step"))
                   for c in (pa.get("robot_robot_contacts") or [])],
            steps=int(pa.get("contact_steps") or 0),
            supported=(_cw.get("supported") is True
                       if _cw else pa.get("observe_error") is None),
            # The recorder now reports how much contact evidence EXISTED, not
            # just how much survived the robot-robot filter, so an empty pair
            # list can be told apart from a contact query that can never name
            # two distinct bodies (the (id,id) bug that hid this check for
            # weeks). Older recordings have no witness -> stays None -> the
            # dependent clause correctly reports itself vacuous.
            total_observed=_cw.get("total_observed") if _cw else None,
            distinct_named=_cw.get("distinct_named") if _cw else None,
            source=_CONTACT_SOURCE,
            error=pa.get("observe_error") or (_cw or {}).get("error"))

    # --- the RUN-LONG contact watch -------------------------------------
    _apply_run_contacts(
        bundle, phase_b.run_contacts if phase_b is not None else None,
        phase_a_steps=(int((phase_b.phase_a or {}).get("contact_steps") or 0)
                       if phase_b is not None else 0))

    # --- the name-free scene inventory ----------------------------------
    #
    # Appended LAST to both lists so every index already published stays put:
    # the robot rows keep their positions (c2_core resolves a body through the
    # roster and indexes the pose series with it), and nothing here is
    # robot-class, so every core that counts ``inventory.robots`` counts the
    # same robots it counted before.
    scan = phase_b.scene_scan if phase_b is not None else None
    if scene_inventory:
        # The NAMED solids first. The name-free scan deliberately SKIPS a body
        # the caller already claimed through ``--solids=``, so a world that
        # uses the task's own published obstacle names lands its obstacles in
        # ``t0_solids`` and NOWHERE ELSE -- and the geometric matcher, which
        # reads the roster, then sees only the floor and the walls. Measured
        # 2026-08-09 on a purpose-built R1 world whose five boxes sit at the
        # published centres to the millimetre: R1.3 reported "specified
        # obstacles found: 0". That is the exact mirror of the gap the scan
        # was built to close -- it fixed the world that renames its boxes and
        # left the world that does NOT rename them invisible.
        #
        # Gated on the same ``scene_inventory`` flag for the same reason it
        # is: a1_core publishes ``robots_seen = len(roster.bodies)`` and
        # c2_core resolves a body through the roster, so quietly handing
        # either a floor slab would rewrite a frozen row's measurements.
        _apply_named_solids(bundle, (pa or {}).get("t0_solids"))
        _apply_scene_inventory(bundle, scan)

    # --- motion ---------------------------------------------------------
    usable = (phase_b is not None and not phase_b.error
              and phase_b.xyz is not None)
    if usable:
        meta = phase_b.meta or {}
        # Row i of xyz is the recorder's i-th TRACK. The recorder publishes the
        # column map (``meta["tracks"]``) precisely so this is read rather than
        # assumed; a recording made before that map existed carried robot rows
        # only, and the fallback reconstructs exactly that.
        ids, kinds, parents, names, src = _rows_from_tracks(phase_b)
        bundle.trajectory = Trajectory(
            body_ids=ids, kinds=kinds, parents=parents, names=names,
            t=phase_b.t, xyz=phase_b.xyz, vel=None,
            dt_s=(float(meta["dt_ms"]) / 1000.0 if meta.get("dt_ms")
                  else None),
            recorded_s=meta.get("recorded_s"),
            complete=bool(meta.get("complete")),
            source=src)
    else:
        bundle.motion_error = (
            (phase_b.error if phase_b else "phase B not run")
            or labels.get("motion_absent_default", "no motion samples"))

    # --- process facts + engine attribution -----------------------------
    if phase_b is not None:
        meta = phase_b.meta or {}
        recorded_s = meta.get("recorded_s", 0.0)
        driver_done = bool(meta.get("complete") and meta.get("quit_called"))
        marker = _FINALIZE_MARKER_RE.search(phase_b.log_text or "") is not None
        bundle.process = ProcessFacts(
            exit_code=phase_b.rc, timed_out=bool(phase_b.timed_out),
            error_lines=_error_lines(phase_b.log_text),
            log_available=bool(phase_b.log_text),
            log_source=_LOG_SOURCE,
            behaviour_starts=_controller_starts(phase_b.log_text),
            start_source=_STARTS_SOURCE,
            driver_completed=driver_done,
            reached_finalize=bool(marker or driver_done),
            finalize_evidence=("newton log marker" if marker else
                               ("recorder completed %gs of sim" % recorded_s
                                if driver_done else "none")),
            wall_s=phase_b.wall_s, attempts_used=phase_b.attempts_used)
        bundle.adapter_measurements["motion"] = {
            "phase_b": phase_b.as_dict()}

        world_text = ""
        if artifact is not None:
            try:
                world_text = Path(artifact).read_text(encoding="utf-8",
                                                      errors="replace")
            except OSError:
                world_text = ""
        bundle.attribution = _attribution(phase_b.sidecar, world_text,
                                          phase_b.launch_env)

    return bundle


# --- the camera channel (graders/b2.py's VIEW_HOOK) -------------------------

_VIEWPOINT_SOURCE = ("GET /scene/viewpoint on a per-run private harness "
                     "instance (engine-resolved forward/up axes; per-axis "
                     "FOV resolved against the real viewport aspect, read "
                     "from a rendered frame's pixel size)")


def _camera_from_viewpoint(payload, source):
    """One harness ``/scene/viewpoint`` response -> a neutral CameraPose.

    The endpoint resolves everything the neutral pose needs: ``forward`` /
    ``up`` are world-frame unit vectors derived from the live Viewpoint's
    axis-angle by the engine-side supervisor, and ``fov_h_deg`` /
    ``fov_v_deg`` are the per-axis angles already narrowed by the REAL
    viewport aspect (the harness measures it from a screenshot's PNG header,
    falling back to a documented default). Nothing here interprets a VRML
    rotation -- that is exactly the per-simulator step this adapter exists to
    absorb.
    """
    fov_h = payload.get("fov_h_deg")
    fov_v = payload.get("fov_v_deg")
    return CameraPose(
        position=tuple(payload.get("position") or ()) or None,
        forward=tuple(payload.get("forward") or ()) or None,
        up=tuple(payload.get("up") or ()) or None,
        fov_h_rad=math.radians(float(fov_h)) if fov_h is not None else None,
        fov_v_rad=math.radians(float(fov_v)) if fov_v is not None else None,
        aspect=payload.get("aspect"),
        source=source)


def _task_initial_world(task_id, stage_dir):
    """A loadable copy of the task's shipped world, staged outside tasks/.

    Staged because loading a world makes the harness write its injected
    supervisor sibling into the world's own directory -- which must never be
    the tracked ``tasks/<id>/initial/`` tree. The same ``@HUSKY_URDF@``
    substitution the orchestrator's scratch materialiser applies is applied
    here, so the copy loads for any task that ships a URDF reference.
    """
    from agentbench import tasks as task_registry
    from agentbench.common.paths import HUSKY_URDF, as_wbt_path
    task = task_registry.get(task_id)
    worlds = sorted(task.initial_dir.glob("*.wbt"))
    if not worlds:
        return None
    src = worlds[0]
    stage_dir.mkdir(parents=True, exist_ok=True)
    dst = stage_dir / src.name
    text = src.read_text(encoding="utf-8").replace(
        task_registry.SUBSTITUTION_TOKEN, as_wbt_path(HUSKY_URDF))
    dst.write_text(text, encoding="utf-8")
    return dst


def build_view_evidence(task_id, *, bundle=None, artifact=None, run_dir=None,
                        scratch_dir=None, **_):
    """The B2 camera channel, measured from live scenes. Never raises.

    Two loads under one per-run private harness session:

      1. a staged copy of the task's shipped world -> the **initial** pose
         (the pose the task shipped, engine-resolved rather than re-parsed);
      2. the agent's artifact -> the **final** pose, "however the agent set
         it". An artifact the engine refuses to load comes back as
         ``artifact_parsed=False`` with no final pose -- the agent-broke-it
         case B2.1 fails on -- while a harness that never came up is an
         adapter ``error`` (a broken measurement, never a verdict).
    """
    if artifact is None and scratch_dir is not None:
        artifact = worldtext.pick_artifact(scratch_dir)
    if artifact is None:
        return ViewEvidence(source=ADAPTER,
                            error="no artifact world to read a camera from")
    base = Path(run_dir) if run_dir is not None else Path(artifact).parent
    view_dir = base / "view_evidence"
    ev = ViewEvidence(artifact=str(artifact), source=_VIEWPOINT_SOURCE)

    from agentbench.adapters.omnisim import harness as harness_mod
    sess = harness_mod.HarnessSession(view_dir)
    try:
        if not sess.start():
            ev.error = ("the view-evidence harness session never became "
                        "healthy, so no camera pose could be measured")
            return ev

        # -- the pose the task shipped ----------------------------------
        staged = _task_initial_world(task_id, view_dir / "initial_world")
        if staged is None:
            ev.error = ("task %s ships no initial world, so the initial "
                        "camera pose cannot be measured" % task_id)
            return ev
        _dt, code, payload = sess.post(
            "/world/load", {"path": str(staged.resolve()), "wait_s": 300.0})
        if code == 200 and payload.get("ok"):
            _dt, vcode, vp = sess.get("/scene/viewpoint")
            if vcode == 200:
                ev.initial = _camera_from_viewpoint(
                    vp, "the task's shipped world under " + _VIEWPOINT_SOURCE)
            else:
                ev.error = ("/scene/viewpoint on the shipped world returned "
                            "http %s" % vcode)
        else:
            ev.error = ("the task's shipped world failed to load (http %s), "
                        "so the initial pose is unmeasured" % code)

        # -- the pose the agent left ------------------------------------
        _dt, code, payload = sess.post(
            "/world/load", {"path": str(Path(artifact).resolve()),
                            "wait_s": 300.0})
        if code != 200 or not payload.get("ok"):
            # The engine itself refused the agent's edit: that is the
            # broke-the-scene case, an AGENT failure (a red B2.1), and it is
            # reported as such rather than as a broken instrument -- so no
            # ``error`` is set here.
            ev.artifact_parsed = False
            return ev
        ev.artifact_parsed = True
        _dt, vcode, vp = sess.get("/scene/viewpoint")
        if vcode == 200:
            ev.final = _camera_from_viewpoint(
                vp, "the agent's artifact under " + _VIEWPOINT_SOURCE)
        elif not ev.error:
            ev.error = ("/scene/viewpoint on the artifact returned http %s"
                        % vcode)
        return ev
    except Exception as exc:  # noqa: BLE001  (adapter rule 1: never raise)
        ev.error = "view-evidence pass failed: %r" % (exc,)
        return ev
    finally:
        sess.stop()
        shutil.rmtree(view_dir / "initial_world", ignore_errors=True)


# --- OmniSim-shaped readers -------------------------------------------------


# A ``controller`` field OmniSim reads as "no controller". The empty string is
# the field's own default; "<none>" is the spelling a world uses to say it out
# loud. Kept to exactly the v0 set on purpose -- widening it (Webots also
# accepts "void") would silently change what A1.4 counts.
_NO_BEHAVIOUR = ("", "<none>")


def _effective(controller):
    """The behaviour that will actually run, or None if the field means none."""
    return None if (controller or "") in _NO_BEHAVIOUR else controller


def _body_from_harness(r, is_robot):
    kind = r.get("type") or ""
    n_joints = r.get("num_joints")
    return Body(body_id=r.get("def") or "", name=r.get("name") or "",
                kind=kind, position=tuple(r["position"])
                if r.get("position") else None,
                n_joints=n_joints,
                behaviour=_effective(r.get("controller")),
                behaviour_declared=r.get("controller"),
                dynamic=None, robot_class=is_robot(kind, n_joints),
                identity_evidence="type=%s num_joints=%s (GET /robots)"
                % (kind, n_joints))


def _recorder_id(r):
    """The recorder's stable body id: its DEF, else ``#<node id>``.

    The agent has no reason to add DEFs (SPEC 4.5.1), so most worlds have
    none -- but a node id is unique per body, which is what distinctness needs.
    """
    return r.get("def") or "#%s" % r.get("id")


def _body_from_recorder(r, is_robot):
    kind = r.get("type") or ""
    base_type = r.get("base_type") or ""
    n_joints = r.get("num_joints")
    return Body(body_id=_recorder_id(r), name=r.get("name") or "", kind=kind,
                n_joints=n_joints,
                behaviour=_effective(r.get("controller")),
                behaviour_declared=r.get("controller"),
                dynamic=r.get("has_physics"),
                robot_class=is_robot(kind, n_joints, base_type),
                identity_evidence=("type=%s base_type=%s num_joints=%s "
                                   "(recorder roster)")
                % (kind, base_type, n_joints))


def _body_from_t0(e, is_robot):
    kind = e.get("type") or ""
    base_type = e.get("base_type") or ""
    n_joints = e.get("num_joints")
    b = e.get("bounds") or {}
    return Body(body_id=_recorder_id(e), name=e.get("name") or "", kind=kind,
                position=tuple(e["position"]) if e.get("position") else None,
                aabb_min=tuple(b["bbox_min"]) if b.get("bbox_min") else None,
                aabb_max=tuple(b["bbox_max"]) if b.get("bbox_max") else None,
                n_joints=n_joints,
                behaviour=_effective(e.get("controller")),
                behaviour_declared=e.get("controller"),
                dynamic=e.get("has_physics"),
                robot_class=is_robot(kind, n_joints, base_type),
                identity_evidence=("type=%s base_type=%s num_joints=%s "
                                   "(recorder t=0 scan)")
                % (kind, base_type, n_joints))


def _solid_from_t0(e):
    """A named non-robot body the grader asked the recorder to bound."""
    b = e.get("bounds") or {}
    return Body(body_id=_recorder_id(e), name=e.get("name") or "",
                kind="Solid",
                position=tuple(e["position"]) if e.get("position") else None,
                aabb_min=tuple(b["bbox_min"]) if b.get("bbox_min") else None,
                aabb_max=tuple(b["bbox_max"]) if b.get("bbox_max") else None,
                dynamic=e.get("has_physics"),
                robot_class=False,
                identity_evidence=("named Solid, recorder t=0 scan; per-step "
                                   "track: %s"
                                   % (("row %d" % e["track_index"])
                                      if e.get("track_index") is not None
                                      else (e.get("track_reason")
                                            or "not requested"))))


def _scene_body_from_t0(e):
    """One non-robot body the recorder found WITHOUT being told its name.

    ``robot_class=False`` unconditionally, and this is the load-bearing half of
    the whole change: a scanned box is scenery, and R1.2 ("exactly one drivable
    robot"), R2.2 and R3.3 all count ``inventory.robots``. A scan that let a
    crate read as a robot would fail a correct world with "six robot-class
    bodies" -- exactly the kind of instrument-authored verdict this scan exists
    to remove.
    """
    b = e.get("bounds") or {}
    nested = e.get("nested_in")
    return Body(body_id=e.get("body_id") or _recorder_id(e),
                name=e.get("name") or "",
                kind=e.get("type") or "Solid",
                position=tuple(e["position"]) if e.get("position") else None,
                aabb_min=tuple(b["bbox_min"]) if b.get("bbox_min") else None,
                aabb_max=tuple(b["bbox_max"]) if b.get("bbox_max") else None,
                dynamic=e.get("has_physics"),
                robot_class=False,
                # A Solid nested inside another Solid IS part of it; a
                # top-level one is an independent object of the scene. That is
                # what ``BodyInventory.independent`` is for, and it is why the
                # nesting is carried rather than flattened.
                member_of=nested,
                identity_evidence=(
                    "non-robot %s found by the recorder's NAME-FREE t=0 scene "
                    "scan (depth %s%s); bounds: %s"
                    % (e.get("base_type") or e.get("type") or "Solid",
                       e.get("depth"),
                       ", inside %s" % nested if nested else ", top level",
                       "world-space AABB from geometry.bounds_for_subtree"
                       if b.get("bbox_min") else
                       (e.get("bounds_error") or "NOT MEASURED"))))


#: What the run-long watch is, in the row.
_RUN_CONTACT_SOURCE = ("agentbench_recorder run-long contact watch: the "
                       "engine's own contact points, sampled across the "
                       "RECORDING window and resolved to body names")


def _apply_run_contacts(bundle, doc, phase_a_steps=0):
    """Merge the recorder's RUN-LONG contact pairs into the neutral channel.

    **Only the pairs that are NOT robot-robot are merged, and that is a
    correctness requirement rather than caution.** ``a1_core`` reads
    ``contacts.robot_robot_pairs`` and ``contacts.steps`` as the answer to a
    question about **t=0** ("were ten robots spawned interpenetrating?").
    Adding a second window's robot-robot pairs to the same list would silently
    re-scope a frozen assertion -- an A1 world whose robots touch at t=12 s
    would start failing an interpenetration check about t=0, with no physical
    fact having changed. ``steps`` is likewise left exactly as phase A wrote
    it, and the full run-long summary is published in
    ``adapter_measurements`` regardless, so nothing is hidden -- it is only
    kept out of a channel scoped to a different question.

    **THE VACUITY COUNTERS ARE THE ONE EXCEPTION, AND ONLY WHEN PHASE A HAD NO
    WINDOW.** ``total_observed`` / ``distinct_named`` are not measurements of
    the scene, they are the witness for "could this channel have reported a
    contact at all" (``graders/evidence.ContactObservation``), and
    ``r4_core``'s R4.5 gates on exactly that: ``supported and total_observed >
    0``. When a task asks for ``contact_steps`` 0 or -1 the phase-A window is
    a single PRE-STEP sample, which observes nothing because the engine has
    not stepped yet -- so the witness said 0 while the run-long watch was
    holding 25,209 contacts and had NAMED the collision. MEASURED on the first
    R4/omnisim cell (2026-08-11): R4.5 was structurally un-passable on this
    arm, whatever a robot did, because the assertion's own falsifier read a
    counter from a window that does not exist. That is the mirror image of the
    defect R1 shipped (a collision clause that could not go RED) and it is the
    same class of bug.

    So the counters are filled from the run-long doc **only when phase A's
    window was zero-width** (``phase_a_steps <= 0``). Scope, measured rather
    than asserted: A1 asks for ``contact_steps: 10``, so its counters are
    untouched under every recording and every A1 row stays byte-reproducible;
    the tasks with a zero-width window are R1, R2, C1 and R4, and of their
    graders only ``r4_core`` reads these fields at all.

    ``supported`` is OR-ed, because "can this adapter answer contacts at all?"
    is true if EITHER window answered. R1's task meta asks for
    ``contact_steps: 0``, so its phase-A window is one sample wide; without
    this, an arm that measured the whole run would still report itself unable
    to check, and "unmeasured is never a pass" would fail an honest run for
    our own instrument's silence.
    """
    if not doc:
        return
    bundle.adapter_measurements["contacts_run"] = {
        k: doc.get(k) for k in ("supported", "every_n_steps", "steps_sampled",
                                "total_observed", "distinct_named",
                                "unpaired", "truncated", "error")}
    bundle.adapter_measurements["contacts_run"]["pairs"] = [
        {"a": p.get("a"), "b": p.get("b"), "b_robot": p.get("b_robot"),
         "first_step": p.get("first_step"), "last_step": p.get("last_step"),
         "count": p.get("count")}
        for p in (doc.get("pairs") or [])][:16]
    con = bundle.contacts
    if con is None:
        con = ContactObservation(source=_RUN_CONTACT_SOURCE)
        bundle.contacts = con
    if doc.get("supported"):
        con.supported = True
    # The vacuity witness, and only where phase A has none to overwrite.
    if int(phase_a_steps or 0) <= 0 \
            and int(doc.get("steps_sampled") or 0) > 0:
        con.total_observed = doc.get("total_observed")
        con.distinct_named = doc.get("distinct_named")
    added = 0
    for p in (doc.get("pairs") or []):
        if p.get("b_robot"):
            continue                  # phase A owns the robot-robot question
        if not p.get("a") or not p.get("b"):
            continue                  # an unnameable partner is not a pair
        con.pairs.append(ContactPair(a=p.get("a"), b=p.get("b"),
                                     a_robot=True, b_robot=False,
                                     point=p.get("point"),
                                     step=p.get("first_step")))
        added += 1
    if added:
        con.source = "%s; plus %d run-long robot-vs-scene pair(s) from the %s" \
            % (con.source, added, _RUN_CONTACT_SOURCE)


def _apply_named_solids(bundle, entries):
    """Put the ``--solids=`` bodies in the ROSTER too. Never raises.

    They are already in ``bundle.t0`` (``_solid_from_t0``); this adds the same
    bodies, by ``body_id``, to the inventory a geometric matcher reads. See
    the call site for the measurement that made it necessary.
    """
    if not entries:
        return
    known = {b.body_id for b in bundle.roster.bodies}
    fresh = [_solid_from_t0(e) for e in entries if isinstance(e, dict)]
    fresh = [b for b in fresh if b.body_id and b.body_id not in known]
    bundle.roster.bodies.extend(fresh)
    bundle.adapter_measurements.setdefault("roster", {})["named_solids"] = {
        "added_to_roster": len(fresh),
        "source": ("the recorder's --solids= t=0 bounds scan, which the "
                   "name-free scan skips by design"),
    }


def _apply_scene_inventory(bundle, scan):
    """Fold the recorder's name-free scan into both inventories. Never raises.

    Absent stays absent: a recording with no scan block leaves a note saying
    the instrument did not run, which is what R1.3 must be able to tell apart
    from "the agent built no obstacles".
    """
    if not scan:
        bundle.notes.append(
            "the recorder's NAME-FREE t=0 scene scan did not run on this "
            "recording, so no non-robot body was offered to the geometric "
            "matchers. That is an INSTRUMENT GAP, not a statement about what "
            "the world contains.")
        return
    known = {b.body_id for b in bundle.roster.bodies}
    known |= {b.body_id for b in bundle.t0.bodies}
    scanned = [_scene_body_from_t0(e) for e in (scan.get("bodies") or [])
               if isinstance(e, dict)]
    fresh = [b for b in scanned if b.body_id and b.body_id not in known]
    bundle.roster.bodies.extend(fresh)
    bundle.t0.bodies.extend(fresh)
    bundle.adapter_measurements.setdefault("roster", {})["scene_scan"] = {
        "supported": scan.get("supported"),
        "found": scan.get("found"), "bounded": scan.get("bounded"),
        "cap": scan.get("cap"), "truncated": scan.get("truncated"),
        "added_to_inventories": len(fresh),
        "already_known": len(scanned) - len(fresh),
        "source": _SCENE_SCAN_SOURCE,
    }
    if scan.get("supported") is False:
        bundle.notes.append(
            "the recorder's name-free scene scan produced NO measurement: %s. "
            "Every geometric assertion over a non-robot body in this run is "
            "unmeasured rather than failed."
            % (scan.get("bounds_error") or "reason unrecorded"))
    if scan.get("truncated"):
        bundle.notes.append(
            "the name-free scene scan is TRUNCATED: %s non-robot bod(ies) "
            "matched the predicate and the first %s were bounded. A body past "
            "the cap has no entry here, so an assertion that fails to find "
            "something may be looking at a truncated inventory."
            % (scan.get("found"), scan.get("cap")))
    unbounded = len(scanned) - int(scan.get("bounded") or 0)
    if unbounded > 0:
        bundle.notes.append(
            "%d of the %d bodies in the name-free scene scan carry no "
            "world-space AABB (no measurable geometry, or the bounds walk "
            "failed on them); they are in the inventory with bounds absent, "
            "never with a made-up box." % (unbounded, len(scanned)))


def _link_from_t0(e):
    """One moving body INSIDE a robot: a link of the articulated chain.

    ``robot_class`` is hard-coded ``False`` and that is the honest answer, not
    a shortcut. The identity predicate asks "is this body the robot the task
    named"; a forearm is part of that robot and is not it. Membership is
    carried by ``member_of`` instead, so nothing has to lie to express it --
    and every core that counts ``inventory.robots`` (R1.2, R2.2, R3.3) keeps
    counting arms rather than arm segments.
    """
    b = e.get("bounds") or {}
    why = ("joint endPoint" if e.get("joint_endpoint")
           else "carries a Physics node")
    return Body(body_id=e.get("body_id") or "",
                name=e.get("name") or "",
                kind=e.get("type") or "Solid",
                position=tuple(e["position"]) if e.get("position") else None,
                aabb_min=tuple(b["bbox_min"]) if b.get("bbox_min") else None,
                aabb_max=tuple(b["bbox_max"]) if b.get("bbox_max") else None,
                dynamic=e.get("has_physics"),
                robot_class=False,
                member_of=e.get("parent_body_id"),
                identity_evidence=("link %s of %s (%s), recorder t=0 scan"
                                   % (e.get("link_index"),
                                      e.get("parent_name")
                                      or e.get("parent_body_id"), why)))


#: Wording for the trajectory's ``source``, per track composition.
_TRAJ_SOURCE = ("the grader-owned recorder: every tracked body's world pose "
                "sampled once per basic timestep, %.17g CSV")


def _rows_from_tracks(phase_b):
    """``(body_ids, kinds, parents, names, source)`` for the pose series.

    Reads the recorder's own column map, which is the only thing that knows
    what row 7 of a mixed robot/link/solid recording IS. Two invariants this
    upholds, both load-bearing:

      * ROBOT ROWS COME FIRST AND KEEP THEIR ORDER, so a core that resolves a
        body through the (robots-only) roster and indexes the pose series with
        that position -- ``c2_core`` and ``r1_core`` both do -- is unaffected by
        any link or solid row appended after it.
      * every row has an id, because ``adapters.check_bundle`` requires
        ``len(body_ids) == xyz.shape[0]`` and a row nobody can name is a row a
        grader cannot cite.

    A recording with no map is a pre-2026-08-09 one: robots only, ids from the
    roster, exactly as before.
    """
    tracks = phase_b.tracks
    n_rows = phase_b.n_tracks
    if not tracks:
        return ([_recorder_id(r) for r in (phase_b.roster or [])],
                [], [], [], _TRAJ_SOURCE)
    ids, kinds, parents, names = [], [], [], []
    for tr in tracks[:n_rows]:
        ids.append(tr.get("body_id") or _recorder_id(tr))
        kinds.append(tr.get("kind") or "")
        parents.append(tr.get("parent_body_id"))
        names.append(tr.get("name") or "")
    meta = phase_b.meta or {}
    while len(ids) < n_rows:
        # The map and the CSV disagree -- a truncated write, or a meta from a
        # different run. Name the row for what it is rather than dropping it:
        # a row with no id trips check_bundle, and a silently dropped row would
        # shift every index after it.
        i = len(ids)
        ids.append("#unmapped_row%d" % i)
        kinds.append("")
        parents.append(None)
        names.append("")
    extra = []
    if len(tracks) != n_rows:
        extra.append("WARNING: the recorder's column map has %d entr(ies) for "
                     "%d CSV row(s); unmapped rows are named "
                     "'#unmapped_row<i>' and carry no kind"
                     % (len(tracks), n_rows))
    if meta.get("n_link_tracks"):
        extra.append("%d link track(s), cap %s per robot%s"
                     % (meta["n_link_tracks"], meta.get("link_cap"),
                        " (TRUNCATED)" if meta.get("links_truncated") else ""))
    if meta.get("n_solid_tracks"):
        extra.append("%d named-Solid track(s), mode %r"
                     % (meta["n_solid_tracks"],
                        meta.get("solid_tracks_mode")))
    src = _TRAJ_SOURCE
    if extra:
        src += (" -- %d Robot row(s) first, then %s"
                % (meta.get("n_robots", 0), "; then ".join(extra)))
    return ids, kinds, parents, names, src


_ERROR_LINE_PREFIX = "ERROR:"
_CONTROLLER_START = re.compile(r"(?m)^INFO:\s+(\S+):\s+Starting controller:")


def _error_lines(log_text):
    """Every ``ERROR:``-class line the engine wrote.

    Lived in ``graders/physical.py`` until the sim-agnostic split: a log format
    is not a physical unit, and no other simulator writes this prefix.
    """
    return [ln for ln in (log_text or "").splitlines()
            if ln.startswith(_ERROR_LINE_PREFIX)]


def _controller_starts(log_text):
    """{controller name: processes observed to start}, from the engine log.

    This is the *second half* of A1.4 and it is deliberately not inferrable
    from the scene: a world can declare ten controllers and start none.
    """
    out = {}
    for name in _CONTROLLER_START.findall(log_text or ""):
        out[name] = out.get(name, 0) + 1
    return out


def _attribution(sidecar, world_text, launch_env=None):
    """SPEC A1.10 on OmniSim: the sidecar verdict, else an explicit ODE pin.

    Delegates to ``adapters/omnisim/headless.newton_attribution`` so there is
    exactly one implementation of "which backend drove this run" in the tree,
    and wraps its answer in the neutral :class:`EngineAttribution`.

    ``launch_env`` is the environment the engine subprocess was actually
    given (``PhaseBResult.launch_env``). It is threaded through rather than
    re-read from ``os.environ`` because the launcher strips the backend
    knobs from the child env, so the parent's copy of OMNISIM_FORCE_ODE
    describes a run that never happened.
    """
    d = headless.newton_attribution(sidecar or {"present": False},
                                    world_text, launch_env=launch_env)
    if d is None:
        # Say WHY when the run was asking for the deleted ODE backend, so the
        # INVALID row does not read as "the load was just slow".
        why = headless.ode_request_detected(world_text, launch_env=launch_env)
        if why:
            print("[evidence] UNATTRIBUTABLE, and not merely missing: %s" % why)
        return None
    extra = {k: v for k, v in d.items()
             if k not in ("backend", "solver", "degraded", "source")}
    return EngineAttribution(backend=d.get("backend"), solver=d.get("solver"),
                             degraded=bool(d.get("degraded")),
                             source=d.get("source", ""), extra=extra)
