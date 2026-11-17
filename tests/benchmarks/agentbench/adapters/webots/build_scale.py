"""Real upstream-Webots adapter for the frozen BuildScale v1 core."""

from __future__ import annotations

import copy
from pathlib import Path

from agentbench.adapters.omnisim import build_scale as common_scale
from agentbench.adapters.webots import evidence as webots_evidence
from agentbench.adapters.webots import launcher, recording, task_support
from agentbench.graders.evidence import ContactObservation
from agentbench.preregister.v1 import build_scale_core
from agentbench.preregister.v1.build_scale_core import (
    BuildScaleEvidence, ControlObservation, PortableObservation)


NO_CONTROLLER = {"", "<none>", "void"}


def _contacts(observation, *, first_steps=None):
    if observation is None:
        return ContactObservation(error="no upstream contact observation")
    pairs = list(observation.pairs)
    if first_steps is not None:
        pairs = [p for p in pairs if p.step is not None
                 and int(p.step) <= int(first_steps)]
    return ContactObservation(
        pairs=pairs,
        steps=(min(observation.steps, first_steps)
               if first_steps is not None else observation.steps),
        window_s=(first_steps * 0.016 if first_steps is not None
                  else build_scale_core.SAMPLE["motion_window_s"]),
        supported=observation.supported,
        total_observed=observation.total_observed,
        distinct_named=max(int(observation.distinct_named or 0),
                           sum(1 for pair in pairs if pair.distinct)),
        source=(observation.source + ("; filtered to first %d steps" % first_steps
                                      if first_steps is not None else "")),
        error=observation.error)


def _controls(bundle, audit):
    robots = bundle.roster.robots
    starts = dict(bundle.process.behaviour_starts or {}) if bundle.process else {}
    grouped = {}
    for body in robots:
        grouped.setdefault(body.behaviour or "", []).append(body)
    channels, started = {}, {}
    for behaviour, bodies in grouped.items():
        if behaviour in NO_CONTROLLER:
            continue
        enough = int(starts.get(behaviour, 0)) >= len(bodies)
        for body in bodies:
            channels[body.body_id] = "webots-controller-instance/%s" % body.body_id
            started[body.body_id] = enough
    return ControlObservation(
        channel_by_robot=channels, started_by_robot=started,
        forbidden_world_mutation_calls=list(audit["forbidden"]),
        source=("live Robot controller fields + captured per-name controller "
                "start counts; upstream launches an isolated controller "
                "instance per Robot; collected sources scanned for pose writes"),
        error=None if bundle.process else "no process facts")


def build_evidence(level, *, artifact, run, portability=None):
    artifact = Path(artifact)
    # BuildScale tracks independent robots only.  Upstream's generic recorder
    # also includes the static floor in its roster and the dynamic calibration
    # box in its trajectory; filtering both documents before neutral mapping
    # preserves the trajectory/roster row invariant without hiding any robot.
    run.roster = copy.deepcopy(run.roster)
    run.trajectory = copy.deepcopy(run.trajectory)
    if run.roster and isinstance(run.roster.get("bodies"), list):
        run.roster["bodies"] = [
            body for body in run.roster["bodies"]
            if body.get("base_type") == "Robot" or body.get("type") == "Robot"]
    if run.trajectory and isinstance(run.trajectory.get("bodies"), list):
        run.trajectory["bodies"] = [
            body for body in run.trajectory["bodies"]
            if body.get("kind") == "robot"]
    bundle = webots_evidence.build_bundle(
        "BuildScale_%d" % level, robot_identity="any_robot",
        live_expected=False, artifact=artifact, run=run,
        scene_inventory=False)
    audit = common_scale.source_audit(artifact)
    if portability is None:
        portability = PortableObservation(
            artifact_files=audit["artifact_files"],
            missing_dependencies=audit["missing"],
            absolute_host_dependencies=audit["absolute"],
            source="collected dependency scan; clean replay not run")
    initial_steps = int(build_scale_core.T["initial_contact_steps"])
    return BuildScaleEvidence(
        requested_robots=int(level), sim="webots",
        adapter="agentbench.adapters.webots.build_scale",
        artifact=str(artifact), live_load_ok=bundle.live_load_ok,
        roster=bundle.roster, t0=bundle.t0,
        trajectory=bundle.trajectory,
        contacts_initial=_contacts(bundle.contacts, first_steps=initial_steps),
        contacts_window=_contacts(bundle.contacts),
        controls=_controls(bundle, audit), portability=portability,
        process=bundle.process, attribution=bundle.attribution,
        notes=list(bundle.notes))


def _launch(world, run_dir, *, port, probe_port):
    world, run_dir = Path(world).resolve(), Path(run_dir).resolve()
    launcher.launch(
        world, run_dir, duration=build_scale_core.SAMPLE["motion_window_s"],
        contact_steps=-1, port=port, timeout_s=900.0, attempts=2)
    aabbs, _probe_facts = task_support.probe_world(
        world, run_dir / "aabb_probe", port=probe_port)
    run, _merged = task_support.augment_run(run_dir, aabbs)
    run.world = Path(world)
    return run


def run_level(world, run_dir, level):
    world, run_dir = Path(world).resolve(), Path(run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    primary = _launch(world, run_dir / "primary", port=1506, probe_port=1508)
    audit = common_scale.source_audit(world)
    portable = PortableObservation(
        artifact_files=audit["artifact_files"],
        missing_dependencies=audit["missing"],
        absolute_host_dependencies=audit["absolute"],
        clean_replay_attempts=1,
        source="dependency scan + grader-owned clean-directory project replay")
    replay = replay_verdict = None
    replay_error = None
    try:
        clean_world = common_scale._copy_clean_project(
            world, run_dir / "clean_project")
        replay = _launch(clean_world, run_dir / "clean_run", port=1507,
                         probe_port=1509)
        replay_evidence = build_evidence(
            level, artifact=clean_world, run=replay,
            portability=PortableObservation(
                artifact_files=audit["artifact_files"],
                clean_replay_attempts=1, clean_replay_passes=1,
                clean_replay_verdicts=["PASS"], source="replay self-check"))
        replay_verdict = build_scale_core.grade(replay_evidence)
        physical_pass = all(a.ok for a in replay_verdict.assertions
                            if a.id != "BS.9")
        portable.clean_replay_passes = int(physical_pass)
        portable.clean_replay_verdicts = ["PASS" if physical_pass else "FAIL"]
    except Exception as exc:
        replay_error = repr(exc)
        portable.error = replay_error
        portable.clean_replay_verdicts = ["ERROR"]

    verdict = build_scale_core.grade(build_evidence(
        level, artifact=world, run=primary, portability=portable))
    facts = {
        "primary": primary.as_dict(),
        "clean_replay": replay.as_dict() if replay else None,
        "clean_replay_failed_assertions": (
            replay_verdict.failed if replay_verdict else None),
        "clean_replay_error": replay_error,
        "source_audit": audit,
    }
    return verdict, facts


__all__ = ["build_evidence", "run_level"]
