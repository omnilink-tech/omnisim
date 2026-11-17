# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0 (the "License");

"""Real OmniSim evidence adapter for the frozen BuildScale v1 core."""

from __future__ import annotations

import re
import shutil
from pathlib import Path

from agentbench.adapters.omnisim import evidence as generic
from agentbench.adapters.omnisim import headless
from agentbench.common import worldtext
from agentbench.preregister.v1 import build_scale_core
from agentbench.preregister.v1.build_scale_core import (
    BuildScaleEvidence, ControlObservation, PortableObservation)
from agentbench.graders.evidence import ContactObservation, ContactPair


TEXT_SUFFIXES = {".wbt", ".proto", ".py", ".js", ".c", ".cc", ".cpp",
                 ".h", ".hpp", ".json", ".yaml", ".yml", ".xml", ".urdf"}
NO_CONTROLLER = {"", "<none>", "void"}
ABSOLUTE_LITERAL = re.compile(
    r'''["'](?:[A-Za-z]:[\\/]|/(?:home|mnt|Users|opt)/)[^"']*["']''')
FORBIDDEN_SOURCE = (
    re.compile(r"\.setSFVec[234]f\s*\("),
    re.compile(r"\.setSFRotation\s*\("),
    re.compile(r"wb_supervisor_(?:field_set_sf_vec|node_set_velocity)"),
    re.compile(r"\.getField\s*\(\s*[\"'](?:translation|rotation)[\"']\s*\)"),
)


def _controller_files(world: Path, controller_names: set[str]):
    root = headless.project_root_for_world(world)
    files, missing = [], []
    for name in sorted(controller_names - NO_CONTROLLER):
        directory = root / "controllers" / name
        if not directory.is_dir():
            missing.append(str(directory))
            continue
        files.extend(p for p in directory.rglob("*") if p.is_file())
    return root, files, missing


def _robot_blocks_with_text(world: Path):
    text = worldtext.strip_comments(world.read_text(encoding="utf-8",
                                                   errors="replace"))
    out = []
    head = re.compile(r"(?<![A-Za-z0-9_])(URDFRobot|Robot)\s*\{")
    i = 0
    while True:
        match = head.search(text, i)
        if not match:
            break
        inside, after = worldtext._block_body(text, match.end() - 1)
        name = re.search(r'(?<![A-Za-z0-9_])name\s+"([^"]*)"', inside)
        controller = re.search(
            r'(?<![A-Za-z0-9_])controller\s+"([^"]*)"', inside)
        out.append({
            "name": name.group(1) if name else "",
            "controller": controller.group(1) if controller else "",
            "supervisor": bool(re.search(
                r"(?<![A-Za-z0-9_])supervisor\s+TRUE(?![A-Za-z0-9_])",
                inside)),
        })
        i = after
    return out


def source_audit(world: Path):
    """Return artifact files, missing/absolute deps, and pose-mutation hits."""
    world = Path(world)
    blocks = _robot_blocks_with_text(world)
    controllers = {b["controller"] for b in blocks if b["controller"]}
    root = headless.project_root_for_world(world)
    missing = []

    # Follow local textual assets (PROTO, URDF, meshes/configs named by url)
    # from the artifact. Schemes are runtime assets, not portable project
    # dependencies. The queue is bounded by the project root: a relative path
    # escaping it is an absolute-host dependency in all but spelling.
    files, queue, seen = [world], [world], {world.resolve()}
    quoted_ref = re.compile(
        r'(?:(?:EXTERNPROTO)|(?:url))\s+(?:\[\s*)?"([^"]+)"')
    while queue:
        current = queue.pop(0)
        try:
            text = current.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        controllers.update(re.findall(
            r'field\s+SFString\s+controller\s+"([^"]+)"', text))
        controllers.update(re.findall(
            r'(?<![A-Za-z0-9_])controller\s+"([^"]+)"', text))
        for raw in quoted_ref.findall(worldtext.strip_comments(text)):
            if "://" in raw or Path(raw).is_absolute():
                continue
            candidate = (current.parent / raw).resolve()
            try:
                candidate.relative_to(root.resolve())
            except ValueError:
                missing.append(f"relative dependency escapes project: {raw}")
                continue
            if not candidate.is_file():
                missing.append(str(candidate))
                continue
            if candidate not in seen:
                seen.add(candidate)
                files.append(candidate)
                if candidate.suffix.lower() in TEXT_SUFFIXES:
                    queue.append(candidate)

    _root, controller_files, controller_missing = _controller_files(
        world, controllers)
    files.extend(controller_files)
    missing.extend(controller_missing)
    absolute, forbidden = [], []

    for block in blocks:
        if block["supervisor"] and block["controller"] not in NO_CONTROLLER:
            forbidden.append(
                f"{world}: robot {block['name'] or '<unnamed>'} grants its "
                "motion controller Supervisor world-mutation privileges")

    for path in files:
        if path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            missing.append(str(path))
            continue
        for match in ABSOLUTE_LITERAL.finditer(text):
            absolute.append(f"{path}:{match.group(0)}")
        for pattern in FORBIDDEN_SOURCE:
            for match in pattern.finditer(text):
                line = text.count("\n", 0, match.start()) + 1
                forbidden.append(f"{path}:{line}:{match.group(0)}")

    artifact_files = []
    for path in files:
        try:
            artifact_files.append(str(path.resolve().relative_to(root.resolve()))
                                  .replace("\\", "/"))
        except (OSError, ValueError):
            artifact_files.append(str(path))
    return {
        "root": root,
        "blocks": blocks,
        "artifact_files": sorted(set(artifact_files)),
        "missing": sorted(set(missing)),
        "absolute": sorted(set(absolute)),
        "forbidden": sorted(set(forbidden)),
    }


def _controls(bundle, audit) -> ControlObservation:
    robots = bundle.roster.robots
    process = bundle.process
    starts = dict(process.behaviour_starts if process else {})
    by_behaviour = {}
    for body in robots:
        by_behaviour.setdefault(body.behaviour or "", []).append(body)

    channel_by_robot, started_by_robot = {}, {}
    for behaviour, bodies in by_behaviour.items():
        if behaviour in NO_CONTROLLER:
            continue
        enough = int(starts.get(behaviour, 0)) >= len(bodies)
        for body in bodies:
            # OmniSim launches one isolated controller instance per Robot and
            # keys its IPC endpoint by robot identity.  Source programs may be
            # shared; runtime instances are not.
            channel_by_robot[body.body_id] = (
                f"omnisim-controller-instance/{body.body_id}")
            started_by_robot[body.body_id] = bool(enough)

    return ControlObservation(
        channel_by_robot=channel_by_robot,
        started_by_robot=started_by_robot,
        forbidden_world_mutation_calls=list(audit["forbidden"]),
        source=("live Robot controller fields + per-name 'Starting controller' "
                "engine-log counts; OmniSim controller instances use isolated "
                "robot-keyed IPC endpoints; collected controller source and "
                "Robot supervisor fields scanned for world-pose mutation"),
        error=None if process else "no process facts")


def _window_contacts(phase_b) -> ContactObservation:
    doc = (phase_b.run_contacts if phase_b is not None else None) or {}
    phase_doc = ((phase_b.phase_a or {}).get("contact_witness")
                 if phase_b is not None else None) or {}
    witness_defs = ((phase_b.meta or {}).get("contact_witness_defs")
                    if phase_b is not None else None) or {}
    pairs = []
    for rec in doc.get("pairs") or []:
        if not rec.get("b_robot"):
            continue
        pairs.append(ContactPair(
            a=rec.get("a"), b=rec.get("b"), a_robot=True, b_robot=True,
            step=rec.get("first_step")))
    recorded_s = ((phase_b.meta or {}).get("recorded_s")
                  if phase_b is not None else None)
    # The persistent watch and the short calibration use the SAME tracked
    # Robot-subtree query.  A non-colliding fleet can produce no named pair in
    # the run window by construction, so the adapter-owned dynamic witness pair
    # proves the instrument can name two bodies during phase A; the run watch
    # then proves it remained live by observing contacts on every step.  Carry
    # only the VACUITY COUNTER across that boundary, never a collision pair.
    # Missing calibration bodies or an unsupported phase-A query cannot make
    # the counter positive.
    calibrated_named = int(doc.get("distinct_named") or 0)
    calibration_ok = bool(
        doc.get("supported") and phase_doc.get("supported") and
        int(phase_doc.get("distinct_named") or 0) > 0 and
        witness_defs.get("requested") and not witness_defs.get("missing"))
    if calibration_ok:
        calibrated_named = max(
            calibrated_named, int(phase_doc.get("distinct_named") or 0))
    return ContactObservation(
        pairs=pairs,
        steps=int(doc.get("steps_sampled") or 0),
        window_s=recorded_s,
        supported=bool(doc.get("supported")),
        total_observed=doc.get("total_observed"),
        distinct_named=calibrated_named,
        source=("agentbench_recorder per-step run-long native contact watch; "
                "pairs resolved to distinct Robot subtrees; phase-A dynamic "
                "witness calibrates the same query's ability to name both "
                "sides without being counted as a run-window collision"),
        error=doc.get("error"))


def build_evidence(level: int, *, artifact, phase_b,
                   portability: PortableObservation | None = None):
    artifact = Path(artifact)
    bundle = generic.build_bundle(
        f"BuildScale_{level}", robot_identity="any_robot", live_expected=False,
        artifact=artifact, phase_b=phase_b, run_dir=phase_b.run_dir)
    audit = source_audit(artifact)
    if portability is None:
        portability = PortableObservation(
            artifact_files=audit["artifact_files"],
            missing_dependencies=audit["missing"],
            absolute_host_dependencies=audit["absolute"],
            source="collected project dependency scan; clean replay not run")
    return BuildScaleEvidence(
        requested_robots=int(level), sim="omnisim",
        adapter="agentbench.adapters.omnisim.build_scale",
        artifact=str(artifact),
        # BuildScale is standalone by contract; recorder completion is its
        # authoritative live-load witness. generic.build_bundle's
        # live_load_ok field is reserved for an optional harness Phase A and
        # is therefore False here even on a clean 20 s run.
        live_load_ok=bool(phase_b and headless.recorder_finished(phase_b)),
        roster=bundle.roster, t0=bundle.t0,
        trajectory=bundle.trajectory,
        contacts_initial=bundle.contacts,
        contacts_window=_window_contacts(phase_b),
        controls=_controls(bundle, audit), portability=portability,
        process=bundle.process, attribution=bundle.attribution,
        notes=list(bundle.notes))


def _copy_clean_project(world: Path, destination: Path):
    root = headless.project_root_for_world(world)
    destination = Path(destination)
    try:
        destination.resolve().relative_to(root.resolve())
    except ValueError:
        pass
    else:
        raise ValueError("clean replay destination must be outside source project")
    if destination.exists():
        shutil.rmtree(destination)
    shutil.copytree(root, destination, ignore=shutil.ignore_patterns(
        "_agentbench_*", ".pytest_cache", "__pycache__", "*.pyc"))
    relative = world.resolve().relative_to(root.resolve())
    return destination / relative


def run_level(world, run_dir, level: int, *, attempts=3, timeout_s=900.0):
    """Run original + clean-directory replay and return `(verdict, facts)`."""
    world, run_dir = Path(world), Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    primary = headless.run_standalone(
        world, run_dir / "primary", attempts=attempts,
        duration=build_scale_core.SAMPLE["motion_window_s"],
        settle=build_scale_core.SAMPLE["settle_s"],
        contact_steps=int(build_scale_core.T["initial_contact_steps"]),
        run_contacts=1, robot_contacts_only=True,
        contact_witness_defs=("CONTACT_WITNESS_SUPPORT", "CONTACT_WITNESS"),
        timeout_s=timeout_s, tag="buildscale_primary")
    audit = source_audit(world)
    portable = PortableObservation(
        artifact_files=audit["artifact_files"],
        missing_dependencies=audit["missing"],
        absolute_host_dependencies=audit["absolute"],
        clean_replay_attempts=1,
        source="dependency scan + grader-owned clean-directory project replay")

    replay_error = None
    replay_phase = None
    replay_verdict = None
    try:
        clean_world = _copy_clean_project(world, run_dir / "clean_project")
        replay_phase = headless.run_standalone(
            clean_world, run_dir / "clean_run", attempts=attempts,
            duration=build_scale_core.SAMPLE["motion_window_s"],
            settle=build_scale_core.SAMPLE["settle_s"],
            contact_steps=int(build_scale_core.T["initial_contact_steps"]),
            run_contacts=1, robot_contacts_only=True,
            contact_witness_defs=("CONTACT_WITNESS_SUPPORT",
                                  "CONTACT_WITNESS"),
            timeout_s=timeout_s, tag="buildscale_replay")
        replay_evidence = build_evidence(
            level, artifact=clean_world, phase_b=replay_phase,
            portability=PortableObservation(
                artifact_files=audit["artifact_files"],
                clean_replay_attempts=1, clean_replay_passes=1,
                clean_replay_verdicts=["PASS"], source="replay self-check"))
        replay_verdict = build_scale_core.grade(replay_evidence)
        physical_pass = all(a.ok for a in replay_verdict.assertions
                            if a.id != "BS.9")
        portable.clean_replay_passes = int(physical_pass)
        portable.clean_replay_verdicts = ["PASS" if physical_pass else "FAIL"]
    except Exception as exc:  # evidence, never a runner crash
        replay_error = repr(exc)
        portable.error = replay_error
        portable.clean_replay_passes = 0
        portable.clean_replay_verdicts = ["ERROR"]

    evidence = build_evidence(level, artifact=world, phase_b=primary,
                              portability=portable)
    verdict = build_scale_core.grade(evidence)
    facts = {
        "primary": primary.as_dict(),
        "clean_replay": replay_phase.as_dict() if replay_phase else None,
        "clean_replay_failed_assertions": (
            replay_verdict.failed if replay_verdict else None),
        "clean_replay_error": replay_error,
        "source_audit": audit,
    }
    return verdict, facts


__all__ = ["build_evidence", "run_level", "source_audit"]
