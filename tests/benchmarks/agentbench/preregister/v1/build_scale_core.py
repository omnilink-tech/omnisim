# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Frozen simulator-neutral BuildScale v1 grader.

BuildScale measures an agent's ability to author and package a functioning
multi-robot simulation.  It is not a solver-FPS test.  All simulator-specific
facts are translated by an adapter into the dataclasses below; this core sees
only counts, SI-unit trajectories, contacts, process facts, and dependency
replay evidence.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from agentbench.graders import physical as ph
from agentbench.graders.evidence import (BodyInventory, ContactObservation,
                                         EngineAttribution, ProcessFacts,
                                         Trajectory)
from agentbench.graders.verdict import (ARTIFACT_RUNS, CORE_PHYSICAL,
                                        CORE_STRUCTURAL, Falsifier, INVALID,
                                        MIXED, NO_ARTIFACT, Verdict)


SPEC_PATH = Path(__file__).resolve().parent / "build_scale_spec.json"
SPEC = json.loads(SPEC_PATH.read_text(encoding="utf-8"))
LEVELS = tuple(SPEC["levels"])
T = SPEC["thresholds"]
SAMPLE = SPEC["sampling"]


@dataclass
class ControlObservation:
    """Per-robot runtime-controller attribution from the adapter.

    A shared process may control a fleet and still pass: independence is about
    runtime controller *instances*, not unique source programs.  Each robot id
    must map to a distinct instance id and the adapter must establish that all
    instances started.  It must also scan the collected controller sources for
    simulator-specific world-pose mutation APIs: gradual pose writes are still
    teleportation even when every sampled step stays below BS.8's bound.
    """

    channel_by_robot: dict = field(default_factory=dict)
    started_by_robot: dict = field(default_factory=dict)
    forbidden_world_mutation_calls: list = field(default_factory=list)
    source: str = ""
    error: str | None = None


@dataclass
class PortableObservation:
    artifact_files: list = field(default_factory=list)
    missing_dependencies: list = field(default_factory=list)
    absolute_host_dependencies: list = field(default_factory=list)
    clean_replay_attempts: int = 0
    clean_replay_passes: int = 0
    clean_replay_verdicts: list = field(default_factory=list)
    source: str = ""
    error: str | None = None


@dataclass
class BuildScaleEvidence:
    requested_robots: int
    sim: str = ""
    adapter: str = ""
    artifact: str | None = None
    live_load_ok: bool | None = None
    roster: BodyInventory = field(default_factory=BodyInventory)
    t0: BodyInventory = field(default_factory=BodyInventory)
    trajectory: Trajectory | None = None
    contacts_initial: ContactObservation | None = None
    contacts_window: ContactObservation | None = None
    controls: ControlObservation | None = None
    portability: PortableObservation | None = None
    process: ProcessFacts | None = None
    attribution: EngineAttribution | None = None
    notes: list = field(default_factory=list)


def _all_fail(v: Verdict, detail: str) -> Verdict:
    for aid, what in SPEC["assertions"].items():
        v.add(aid, False, what, detail=detail, basis=CORE_STRUCTURAL)
    return v.finish()


def _pairs(contacts: ContactObservation | None) -> set[tuple[str, str]]:
    out = set()
    for pair in contacts.robot_robot_pairs if contacts else []:
        out.add(tuple(sorted((str(pair.a), str(pair.b)))))
    return out


def grade(e: BuildScaleEvidence, *, self_verified: bool = False) -> Verdict:
    task = "BuildScale_%d" % int(e.requested_robots)
    v = Verdict(task, self_verified=self_verified)
    for note in e.notes:
        v.note(note)

    if e.requested_robots not in LEVELS:
        v.outcome = INVALID
        return _all_fail(v, "requested level is outside the frozen BuildScale levels")
    if not e.artifact:
        v.progress = NO_ARTIFACT
        return _all_fail(v, "no portable artifact was produced")
    v.artifacts["artifact"] = str(e.artifact)

    process = e.process
    attribution_ok = bool(e.attribution and e.attribution.backend
                          and e.attribution.source)
    clean = bool(process and process.clean and process.driver_completed)
    bs1 = bool(e.live_load_ok and clean and attribution_ok)
    v.add(
        "BS.1", bs1, SPEC["assertions"]["BS.1"],
        measured={
            "live_load_ok": e.live_load_ok,
            "exit_code": process.exit_code if process else None,
            "timed_out": process.timed_out if process else None,
            "error_lines": len(process.error_lines) if process else None,
            "driver_completed": process.driver_completed if process else None,
            "backend": e.attribution.backend if e.attribution else None,
        },
        threshold="load true; clean exit; driver completed; attributed backend",
        basis=MIXED,
        attested=[e.attribution.source] if attribution_ok else [],
        falsifiers=[Falsifier(
            "clean attributable run", "load failure, simulator error, timeout, "
            "short run, or missing backend identity",
            "simulator log, recorder completion flag, and backend citation",
            bool(process and process.log_available is not False and attribution_ok),
            detail=(process.log_source if process else "no process facts"))])
    if attribution_ok:
        v.attribution = e.attribution.as_dict()

    robots = e.roster.robots
    ids = [str(b.body_id) for b in robots if b.body_id]
    names = [str(b.name) for b in robots if b.name]
    n = int(e.requested_robots)
    bs2 = (len(robots) == n and len(set(ids)) == n and len(set(names)) == n)
    v.add(
        "BS.2", bs2, SPEC["assertions"]["BS.2"],
        measured={"robot bodies": len(robots), "distinct ids": len(set(ids)),
                  "distinct names": len(set(names))},
        threshold=n, basis=CORE_STRUCTURAL,
        falsifiers=[Falsifier(
            "exact distinct count", "one robot missing, duplicated, or replaced "
            "by a non-robot body", "a frozen inventory of every live body",
            bool(e.roster.frozen and e.roster.error is None),
            detail=e.roster.source or e.roster.error or "")])

    controls = e.controls
    channels = controls.channel_by_robot if controls else {}
    starts = controls.started_by_robot if controls else {}
    controlled_ids = set(channels) & set(ids)
    distinct_channels = {channels[rid] for rid in controlled_ids if channels[rid]}
    started = {rid for rid in controlled_ids if starts.get(rid) is True}
    forbidden = list(controls.forbidden_world_mutation_calls if controls else [])
    bs3 = (len(controlled_ids) == n and len(distinct_channels) == n
           and len(started) == n and not forbidden)
    v.add(
        "BS.3", bs3, SPEC["assertions"]["BS.3"],
        measured={"robots with channel": len(controlled_ids),
                  "distinct channels": len(distinct_channels),
                  "runtime controller instances started": len(started),
                  "forbidden world-pose mutation calls": forbidden},
        threshold=n, basis=MIXED,
        attested=[controls.source] if controls and controls.source else [],
        falsifiers=[Falsifier(
            "independent addressability", "two robots share a channel or one "
            "robot's controller instance did not start, or a controller can "
            "write world poses", "per-robot runtime instance ids, process-start "
            "evidence, and an adapter-owned source scan",
            bool(controls and controls.source),
            detail=controls.error if controls else "no control observation")])

    t0robots = e.t0.robots
    boxes = [b.aabb for b in t0robots if b.has_aabb]
    missing_bounds = len(t0robots) - len(boxes)
    min_clearance, worst_pair = (ph.pairwise_min_clearance(boxes)
                                 if len(boxes) >= 2 else (float("nan"), None))
    initial_pairs = _pairs(e.contacts_initial)
    initial_witness = bool(e.contacts_initial
                           and e.contacts_initial.can_name_a_robot_pair)
    bs4 = (len(t0robots) == n and missing_bounds == 0
           and math.isfinite(min_clearance)
           and min_clearance >= T["minimum_initial_aabb_clearance_m"]
           and not initial_pairs and initial_witness)
    v.add(
        "BS.4", bs4, SPEC["assertions"]["BS.4"],
        measured={"minimum AABB clearance (m)": (
                      round(min_clearance, 6) if math.isfinite(min_clearance)
                      else None),
                  "worst pair": worst_pair, "missing bounds": missing_bounds,
                  "initial robot collision pairs": len(initial_pairs)},
        threshold={"clearance_m": T["minimum_initial_aabb_clearance_m"],
                   "collision_pairs": 0,
                   "steps": T["initial_contact_steps"]},
        basis=CORE_PHYSICAL,
        falsifiers=[
            Falsifier("AABB separation", "two initial robot AABBs overlap or "
                      "are closer than the margin", "frozen world-space AABBs "
                      "for all robots", bool(e.t0.frozen and len(boxes) == n),
                      detail=e.t0.source),
            Falsifier("initial contacts", "a named robot-robot contact occurs",
                      "a contact channel proven able to name two distinct bodies",
                      initial_witness,
                      detail=(e.contacts_initial.witness_detail
                              if e.contacts_initial else "no contact observation")),
        ])

    traj = e.trajectory
    usable = bool(traj and traj.xyz is not None and traj.t is not None
                  and traj.n_samples >= 2)
    indices = []
    if usable:
        indices = [traj.index_of(rid) for rid in ids]
        usable = len(indices) == n and all(i is not None for i in indices)
    if usable:
        xyz = np.asarray([traj.xyz[i] for i in indices], dtype=float)
        times = np.asarray(traj.t, dtype=float)
        delta = np.diff(xyz, axis=1)
        step = np.linalg.norm(delta, axis=2)
        dt = np.diff(times)
        max_period = float(np.max(dt))
        speed = step / dt.reshape(1, -1)
        net = np.linalg.norm(xyz[:, -1, :2] - xyz[:, 0, :2], axis=1)
        path = np.sum(np.linalg.norm(np.diff(xyz[:, :, :2], axis=1), axis=2), axis=1)
        mover = ((net >= T["minimum_net_displacement_m"])
                 & (path >= T["minimum_path_length_m"]))
        moving_count = int(np.count_nonzero(mover))
        finite = bool(np.isfinite(xyz).all() and np.isfinite(times).all())
        xy_from_start = np.linalg.norm(xyz[:, :, :2] - xyz[:, :1, :2], axis=2)
        max_xy = float(np.max(xy_from_start))
        z_drop = xyz[:, :1, 2] - xyz[:, :, 2]
        max_z_drop = float(np.max(z_drop))
        max_step = float(np.max(step))
        max_speed = float(np.max(speed))
        recorded_s = float(times[-1] - times[0])
    else:
        net = path = []
        moving_count = 0
        finite = False
        max_xy = max_z_drop = max_step = max_speed = max_period = float("nan")
        recorded_s = 0.0

    required_movers = int(math.ceil(T["minimum_moving_fraction"] * n))
    window_ok = recorded_s >= SAMPLE["motion_window_s"] - 1e-9
    bs5 = bool(usable and window_ok and moving_count >= required_movers)
    v.add(
        "BS.5", bs5, SPEC["assertions"]["BS.5"],
        measured={"moving robots": moving_count, "moving fraction": (
                      moving_count / n if n else None),
                  "minimum net displacement (m)": (
                      float(np.min(net)) if len(net) else None),
                  "minimum path length (m)": (
                      float(np.min(path)) if len(path) else None),
                  "recorded window (s)": recorded_s},
        threshold={"moving robots": required_movers,
                   "net displacement_m": T["minimum_net_displacement_m"],
                   "path_length_m": T["minimum_path_length_m"],
                   "window_s": SAMPLE["motion_window_s"]},
        basis=CORE_PHYSICAL,
        falsifiers=[Falsifier(
            "movement fraction", "more than 10% of robots remain stationary or "
            "only jitter in place", ">=2 pose samples for every robot over the "
            "full window", usable and window_ok,
            detail=traj.source if traj else "no trajectory")])

    bs6 = bool(usable and finite
               and max_xy <= T["maximum_xy_distance_from_start_m"]
               and max_z_drop <= T["maximum_z_drop_from_start_m"])
    v.add(
        "BS.6", bs6, SPEC["assertions"]["BS.6"],
        measured={"all poses finite": finite,
                  "maximum xy distance from start (m)": (
                      max_xy if math.isfinite(max_xy) else None),
                  "maximum z drop from start (m)": (
                      max_z_drop if math.isfinite(max_z_drop) else None)},
        threshold={"maximum_xy_m": T["maximum_xy_distance_from_start_m"],
                   "maximum_z_drop_m": T["maximum_z_drop_from_start_m"]},
        basis=CORE_PHYSICAL,
        falsifiers=[Falsifier(
            "runaway bound", "a NaN/Inf pose, a robot leaving the 25 m envelope, "
            "or a 0.5 m fall", "full-window pose samples for every robot",
            usable, detail=traj.source if traj else "no trajectory")])

    window_pairs = _pairs(e.contacts_window)
    allowed_pairs = int(math.floor(0.02 * n))
    contact_window_ok = bool(e.contacts_window
                             and e.contacts_window.can_name_a_robot_pair
                             and (e.contacts_window.window_s or 0.0)
                             >= SAMPLE["motion_window_s"] - 1e-9)
    bs7 = bool(contact_window_ok and len(window_pairs) <= allowed_pairs)
    v.add(
        "BS.7", bs7, SPEC["assertions"]["BS.7"],
        measured={"unique robot collision pairs": len(window_pairs),
                  "contact window (s)": (e.contacts_window.window_s
                                           if e.contacts_window else None)},
        threshold={"maximum pairs": allowed_pairs,
                   "window_s": SAMPLE["motion_window_s"]},
        basis=CORE_PHYSICAL,
        falsifiers=[Falsifier(
            "collision bound", "more unique robot pairs collide than the "
            "level permits", "full-window contacts able to name two distinct "
            "bodies", contact_window_ok,
            detail=(e.contacts_window.witness_detail if e.contacts_window
                    else "no contact observation"))])

    bs8 = bool(usable and max_period <= SAMPLE["maximum_sample_period_s"] + 1e-9
               and max_step <= T["maximum_single_sample_step_m"]
               and max_speed <= T["maximum_speed_m_s"])
    v.add(
        "BS.8", bs8, SPEC["assertions"]["BS.8"],
        measured={"maximum sample period (s)": (
                      max_period if math.isfinite(max_period) else None),
                  "maximum single-sample step (m)": (
                      max_step if math.isfinite(max_step) else None),
                  "maximum speed (m/s)": (
                      max_speed if math.isfinite(max_speed) else None)},
        threshold={"sample_period_s": SAMPLE["maximum_sample_period_s"],
                   "step_m": T["maximum_single_sample_step_m"],
                   "speed_m_s": T["maximum_speed_m_s"]},
        basis=CORE_PHYSICAL,
        falsifiers=[Falsifier(
            "teleport bound", "one robot jumps farther or faster than the "
            "physical bound", "pose samples no more than 0.1 s apart", usable
            and math.isfinite(max_period)
            and max_period <= SAMPLE["maximum_sample_period_s"] + 1e-9,
            detail=traj.source if traj else "no trajectory")])

    portable = e.portability
    bs9 = bool(portable and portable.artifact_files
               and not portable.missing_dependencies
               and not portable.absolute_host_dependencies
               and portable.clean_replay_attempts
               >= T["minimum_clean_replay_attempts"]
               and portable.clean_replay_passes
               >= T["minimum_clean_replay_passes"]
               and "PASS" in portable.clean_replay_verdicts)
    v.add(
        "BS.9", bs9, SPEC["assertions"]["BS.9"],
        measured={"artifact files": len(portable.artifact_files) if portable else 0,
                  "missing dependencies": (portable.missing_dependencies
                                            if portable else None),
                  "absolute host dependencies": (
                      portable.absolute_host_dependencies if portable else None),
                  "clean replay attempts": (portable.clean_replay_attempts
                                             if portable else None),
                  "clean replay passes": (portable.clean_replay_passes
                                           if portable else None),
                  "clean replay verdicts": (portable.clean_replay_verdicts
                                             if portable else None)},
        threshold={"missing": 0, "absolute": 0,
                   "replay_attempts": T["minimum_clean_replay_attempts"],
                   "replay_passes": T["minimum_clean_replay_passes"]},
        basis=MIXED,
        attested=[portable.source] if portable and portable.source else [],
        falsifiers=[Falsifier(
            "clean replay", "an asset is missing, resolves through an absolute "
            "authoring-machine path, or the clean replay fails", "dependency "
            "scan plus a grader-owned clean-directory replay",
            bool(portable and portable.source),
            detail=portable.error if portable else "no portability observation")])

    v.progress = ARTIFACT_RUNS
    v.measurements["requested_robots"] = n
    v.measurements["provenance"] = {
        "sim": e.sim, "adapter": e.adapter,
        "roster": e.roster.source, "t0": e.t0.source,
        "trajectory": traj.source if traj else None,
        "controls": controls.source if controls else None,
        "portability": portable.source if portable else None,
    }
    if not attribution_ok:
        v.outcome = INVALID
        v.note("missing simulator/backend attribution; numeric result is invalid")
    return v.finish()


__all__ = ["BuildScaleEvidence", "ControlObservation", "LEVELS",
           "PortableObservation", "SPEC", "grade"]
