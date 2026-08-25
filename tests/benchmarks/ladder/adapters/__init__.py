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

"""Ladder columns -- one package per simulator. **A shim, not a fork.**

A ladder column is **two** things, and the split matters:

1. a **runner** that produces one attempt's artifacts on that simulator, and
2. an **evidence builder** that reads those artifacts into the neutral
   :class:`~agentbench.graders.evidence.EvidenceBundle` the sim-agnostic cores
   consume, plus the ladder's own extra channels
   (``ladder.graders.ladder_evidence``).

The bundle contract is ``agentbench/adapters/__init__.py`` -- its six rules and
its ``REQUIRED_EVIDENCE`` list bind here verbatim, and
``agentbench.adapters.check_bundle`` is the executable form a new column runs
against its own output before anyone spends a token on a cell. Its
``resolve()`` already answers *"which module turns this simulator's run into a
neutral bundle"*, so copying that registry here would fork the cross-simulator
contract. This module delegates to it and adds only what the ladder's two
extra channels need::

    ladder.adapters.build_t1_evidence(task, sim=..., **run_evidence)
        -> ladder.graders.ladder_evidence.T1Evidence

which is::

    agentbench.adapters.resolve(sim).build_bundle(...)   # unchanged, reused
      + a support-contact observation                    # ladder channel
      + the commanded point, resolved from the task       # ladder channel

Columns present
---------------

``omnisim`` / ``webots``
    reuse the runners and evidence builders already published under
    ``agentbench/adapters/``; nothing is duplicated. ``omnisim`` additionally
    ships the ladder's own pose + contact sampler
    (``ladder/adapters/omnisim/evidence.py`` and
    ``ladder/controllers/ladder_recorder/``), because the frozen AgentBench
    sampler cannot answer T1.4 -- see below.

``mujoco``
    MuJoCo 3.8.1 via the official ``mujoco`` PyPI wheel; local, CPU, free.
    Bring-up record and effort ledger: ``mujoco/BRINGUP.md``.

The support-contact gap, stated once and reported in every row that hits it
---------------------------------------------------------------------------

T1.4 needs a contact naming a robot body and the surface under it, observed
while the robot is moving. Neither *shipped AgentBench* column populates that:
both filter contacts to robot-robot pairs over a window that ends before the
robot has driven anywhere (three reasons, in full, in
``ladder.graders.ladder_evidence``). A column with no ladder-side sampler
therefore falls back to ``support_from_bundle``, which returns an empty
observation, T1.4 goes red, and :data:`SUPPORT_GAP_NOTE` goes in the verdict's
notes naming the blocker as **ours** (``scaffolding_defect_ours``) rather than
the simulator's. A red assertion whose cause is our own missing scaffolding
must never be published as a capability finding about somebody else's product.
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench import adapters as ab_adapters  # noqa: E402
from ladder import tasks as ladder_tasks  # noqa: E402
from ladder.graders import ladder_evidence as lev  # noqa: E402

# sim id -> the module that supplies the LADDER's extra channels for it.
# Absent is fine and normal: the fallback is honest and the gap is reported.
LADDER_CHANNELS = {
    "omnisim": "ladder.adapters.omnisim.evidence",
    "mujoco": "ladder.adapters.mujoco.evidence",
    # Registered 2026-08-02 for the P-rungs. It is a FORMAT SHIM over
    # agentbench's Webots launcher (which already runs upstream Webots headless
    # inside WSL2), not a second integration, and it answers only the pose
    # series the P/L rungs read -- NOT the T2/T3/T4 channel surface. A rung
    # that needs more will find the attribute missing and refuse by name.
    "webots": "ladder.adapters.webots.evidence",
}

# What a column must be able to answer BEYOND agentbench's REQUIRED_EVIDENCE
# before a ladder cell on it can be published. **Per rung**, because the rungs
# ask for different things and a column can be ready for one and not the next.
LADDER_REQUIRED_EVIDENCE_T1 = (
    ("waypoint", "nothing -- the commanded point is task data and reaches the "
                 "grader from the task file. Listed here so it is visible "
                 "that no adapter may supply it."),
    ("support_contacts", "contacts naming a robot body and the non-robot body "
                         "it is riding on, sampled ACROSS the travel window "
                         "(not only at t=0), timestamped on the same clock as "
                         "the pose series, with the two vacuity "
                         "counters -- so "
                         "'it never touched the ground', 'it touched it only "
                         "at rest' and 'the query names nothing' are three "
                         "distinguishable readings"),
)

# The name T1 published, kept pointing at T1's list rather than growing into a
# union: a column can be ready for T1 and not for T2, and a single merged list
# would hide that.
LADDER_REQUIRED_EVIDENCE = LADDER_REQUIRED_EVIDENCE_T1

LADDER_REQUIRED_EVIDENCE_T2 = (
    ("roles", "nothing -- which body is the object, which is the container "
              "and which is the end effector is task data, for the same "
              "reason the waypoint is. An adapter that could choose which "
              "body counts as 'the object' could choose the one that happens "
              "to be in the container."),
    ("object_pose", "the OBJECT's world position over the run, on the "
                    "simulated clock. Both shipped samplers enumerate robots; "
                    "the object is a plain body and nothing records it."),
    ("end_effector_pose", "the carrier's world position AND ORIENTATION over "
                          "the run, on the same clock. The orientation is not "
                          "optional: T2.3 is stated in the end-effector's "
                          "frame, and a rigidly held object whose wrist "
                          "rotates reads as a 0.1 m slip without it."),
    ("container_geometry", "the container's world-space bounding box, and its "
                           "RIM height where the column can attest one. "
                           "Derivable from the frozen t=0 inventory for the "
                           "box; the rim is not in that contract."),
    ("object_mass", "the object's mass in kilograms and whether gravity is "
                    "acting. The neutral inventory answers 'a mass model is "
                    "attached' and carries no kilograms, and a massless link "
                    "is exactly the prop T2.4 exists to catch."),
    ("grip", "how the object was held -- friction, suction, attachment or "
             "unknown -- with a citation. RECORDED IN EVERY CELL AND GRADED "
             "IN NONE (capability-ladder-plan.md section 2 T2), and no prose "
             "sentence about a T2 cell may omit it."),
)

SUPPORT_GAP_NOTE = (
    "no ladder-side support-contact sampler exists for the %r column, so the "
    "contact half of this cell is unmeasured rather than measured-and-absent. "
    "If T1.4 is the only red, the blocker is scaffolding_defect_ours and the "
    "cell may not be published as a capability finding about the simulator.")

T2_GAP_NOTE = (
    "no ladder-side %s channel exists for the %r column, so what depends on "
    "it is unmeasured rather than measured-and-absent. The blocker for a cell "
    "whose only reds are those assertions is scaffolding_defect_ours -- it "
    "may not be published as a capability finding about the simulator.")

# The channel a T2 assertion dies without, so a gap can be reported against
# the assertion it actually breaks rather than as a general shrug.
T2_CHANNEL_ASSERTIONS = {
    "object_pose": ("T2.1", "T2.2", "T2.3", "T2.4"),
    "end_effector_pose": ("T2.3",),
    "container_geometry": ("T2.1",),
    "support_surfaces": ("T2.2",),
    "object_mass": ("T2.4",),
    "gravity": ("T2.4",),
    "grip": (),          # recorded, never graded: it breaks no assertion
}


def _sim_id(sim=None):
    return (sim or os.environ.get("LADDER_SIM")
            or os.environ.get("AGENTBENCH_SIM") or "omnisim")


def resolve(sim=None):
    """The module that turns a run on ``sim`` into a neutral bundle.

    Delegates to ``agentbench.adapters.resolve`` -- reused, never forked --
    unless the column ships its own ``build_bundle`` on the ladder side. A
    column that AgentBench has no adapter for (a roster member added for this
    programme) can therefore stand up here without an edit to the frozen
    registry, and a column that has one keeps using it.
    """
    sim = _sim_id(sim)
    channels = resolve_ladder_channels(sim)
    if channels is not None and hasattr(channels, "build_bundle"):
        return channels
    return ab_adapters.resolve(sim)


def resolve_ladder_channels(sim=None):
    """The ladder-side channel module for ``sim``, or ``None``."""
    path = LADDER_CHANNELS.get(_sim_id(sim))
    if not path:
        return None
    try:
        return importlib.import_module(path)
    except ImportError:
        return None


def build_t1_evidence(task, *, sim=None, artifact=None, run_dir=None,
                      phase_b=None, support=None, waypoint=None, **kw):
    """One T1 run -> :class:`T1Evidence`. Never raises for a broken simulator.

    ``task`` is a :class:`ladder.tasks.Task` or a task id. Everything in
    ``kw`` is passed straight through to the simulator's own
    ``build_bundle``, exactly as agentbench's task entry points do.
    """
    if not hasattr(task, "meta"):
        task = ladder_tasks.get(str(task))
    sim = _sim_id(sim)
    notes = []

    adapter = resolve(sim)
    bundle = adapter.build_bundle(
        task.id, robot_identity="any_robot", live_expected=False,
        artifact=artifact, phase_b=phase_b, run_dir=run_dir, **kw)

    # --- the support-contact channel -------------------------------------
    if support is None:
        channels = resolve_ladder_channels(sim)
        if channels is not None and hasattr(channels, "support_observation"):
            support = channels.support_observation(phase_b)
    if support is None:
        support = lev.support_from_bundle(bundle)
        notes.append(SUPPORT_GAP_NOTE % sim)

    ev = lev.T1Evidence(bundle=bundle, support=support,
                        robot_name=task.robot_name, notes=notes)

    # --- the commanded point ---------------------------------------------
    ev.waypoint = waypoint if waypoint is not None else _resolve_waypoint(
        task, ev)
    return ev


def _resolve_waypoint(task, ev):
    """The commanded point for this run, resolved against its own start.

    Imported inside the function so the shim does not depend on grader
    internals at import time; the body-selection rule itself is the core's and
    stays there.
    """
    from ladder.graders import t1_core
    cite = ladder_tasks.waypoint_citation(task)
    spec = task.waypoint_spec
    if spec.get("frame") != "start_relative":
        try:
            return lev.Waypoint(
                xy=ladder_tasks.resolve_waypoint(task, (0.0, 0.0)),
                label="the commanded point", source=cite)
        except ValueError as exc:
            return lev.Waypoint(source="", label=str(exc))
    index, body, _why = t1_core.select_robot(ev)
    row, _row_why = t1_core.series_row_for(ev, index, body)
    traj = ev.bundle.trajectory
    if row is None or traj is None or traj.xyz is None:
        return lev.Waypoint(
            source="", label="unresolved: the run produced no pose series for "
                             "the graded body, so a start-relative commanded "
                             "point cannot be resolved")
    start = traj.xyz[row][0][:2]
    return lev.Waypoint(xy=ladder_tasks.resolve_waypoint(task, start),
                        label="the commanded point", source=cite)


def check_t1_evidence(ev):
    """Both contracts at once: agentbench's bundle check plus the ladder's."""
    return (["bundle: %s" % p for p in ab_adapters.check_bundle(ev.bundle)]
            + lev.check_t1_evidence(ev))


# --- T2 ----------------------------------------------------------------------


def build_t2_evidence(task, *, sim=None, artifact=None, run_dir=None,
                      phase_b=None, roles=None, channels=None, **kw):
    """One T2 run -> :class:`T2Evidence`. Never raises for a broken simulator.

    A column supplies T2's channels through **one** optional hook on its
    ladder-side module::

        t2_channels(phase_b, roles=<RoleNames>) -> dict

    whose keys are a subset of :data:`T2_CHANNELS`. Every key it does not
    supply falls back to what the frozen bundle can derive -- which for three
    of them is a real answer (the container box, the static surfaces, the
    object's size at t=0), for one is half an answer (``dynamic`` without the
    kilograms) and for the rest is nothing at all. Each fallback puts
    :data:`T2_GAP_NOTE` in the verdict's notes naming the channel and the
    assertions it breaks, so a red that belongs to our scaffolding is never
    published as a finding about somebody else's product.
    """
    if not hasattr(task, "meta"):
        task = ladder_tasks.get(str(task))
    sim = _sim_id(sim)
    notes = []

    adapter = resolve(sim)
    bundle = adapter.build_bundle(
        task.id, robot_identity="any_robot", live_expected=False,
        artifact=artifact, phase_b=phase_b, run_dir=run_dir, **kw)

    if roles is None:
        roles = _resolve_roles(task)

    supplied = dict(channels or {})
    if not supplied:
        mod = resolve_ladder_channels(sim)
        hook = getattr(mod, "t2_channels", None) if mod is not None else None
        if hook is not None:
            try:
                # ``run_dir`` stands in when there is no phase-B result object:
                # grading an EXISTING run directory (a re-grade, a fixture, a
                # run somebody else performed) passes ``run_dir`` and no
                # ``phase_b``, and a hook handed ``None`` would report every
                # channel unanswered on a run that has all of them on disk.
                # This mirrors what ``build_bundle`` already does with its own
                # run-directory candidates.
                supplied = dict(hook(phase_b if phase_b is not None
                                     else run_dir, roles=roles) or {})
            except Exception as exc:  # noqa: BLE001  (adapter rule 1)
                notes.append("the %r column's T2 channel builder raised %r; "
                             "every channel falls back" % (sim, exc))
                supplied = {}

    def take(key, fallback, gap_for=None):
        got = supplied.get(key)
        if got is None:
            notes.append(T2_GAP_NOTE % (gap_for or key, sim))
            return fallback
        return got

    ev = lev.T2Evidence(
        bundle=bundle, roles=roles, notes=notes,
        object_pose=take("object_pose",
                         lev.pose_series_from_bundle(
                             bundle, name=roles.object_name)),
        end_effector=take("end_effector",
                          lev.pose_series_from_bundle(
                              bundle, name=roles.end_effector_name),
                          gap_for="end_effector_pose"),
        container=take("container",
                       lev.container_from_bundle(bundle,
                                                 roles.container_name),
                       gap_for="container_geometry"),
        grip=take("grip", lev.EMPTY_GRIP),
        object_physics=take("object_physics",
                            lev.body_physics_from_bundle(
                                bundle, name=roles.object_name),
                            gap_for="object_mass"),
        world=take("world", lev.EMPTY_WORLD_PHYSICS, gap_for="gravity"),
        support_surfaces=take("support_surfaces",
                              lev.support_surfaces_from_bundle(bundle)),
        object_aabb=(supplied.get("object_aabb")
                     or lev.object_aabb_from_bundle(bundle,
                                                    roles.object_name)))
    return ev


T2_CHANNELS = ("object_pose", "end_effector", "container", "grip",
               "object_physics", "world", "support_surfaces", "object_aabb")


def t2_unanswered_channels(ev):
    """``{channel: (assertion, ...)}`` for what this evidence cannot answer.

    Computed from the evidence itself rather than from which hook fired, on
    purpose: a column that supplies a channel and fills it with nothing has
    the same gap as one that supplies no channel at all, and the cell's
    blocker is ours either way.
    """
    out = {}
    if not ev.object_pose.usable:
        out["object_pose"] = T2_CHANNEL_ASSERTIONS["object_pose"]
    if not (ev.end_effector.usable and ev.end_effector.has_orientation):
        out["end_effector_pose"] = T2_CHANNEL_ASSERTIONS["end_effector_pose"]
    if not ev.container.usable:
        out["container_geometry"] = T2_CHANNEL_ASSERTIONS["container_geometry"]
    if not [s for s in ev.support_surfaces if s.usable]:
        out["support_surfaces"] = T2_CHANNEL_ASSERTIONS["support_surfaces"]
    if not ev.object_physics.has_mass:
        out["object_mass"] = T2_CHANNEL_ASSERTIONS["object_mass"]
    if ev.world.gravity_acting is None:
        out["gravity"] = T2_CHANNEL_ASSERTIONS["gravity"]
    if not ev.grip.supported:
        out["grip"] = T2_CHANNEL_ASSERTIONS["grip"]
    return out


def _resolve_roles(task):
    """Which body is which, from the task file. Never from an adapter."""
    spec = task.roles
    return lev.RoleNames(
        object_name=spec.get("object", ""),
        end_effector_name=spec.get("end_effector", ""),
        container_name=spec.get("container", ""),
        source=ladder_tasks.roles_citation(task))


def check_t2_evidence(ev):
    """Both contracts at once: agentbench's bundle check plus the ladder's."""
    return (["bundle: %s" % p for p in ab_adapters.check_bundle(ev.bundle)]
            + lev.check_t2_evidence(ev))


__all__ = ["LADDER_CHANNELS", "LADDER_REQUIRED_EVIDENCE",
           "LADDER_REQUIRED_EVIDENCE_T1", "LADDER_REQUIRED_EVIDENCE_T2",
           "SUPPORT_GAP_NOTE", "T2_GAP_NOTE", "T2_CHANNELS",
           "T2_CHANNEL_ASSERTIONS", "resolve", "resolve_ladder_channels",
           "build_t1_evidence", "check_t1_evidence", "build_t2_evidence",
           "check_t2_evidence", "t2_unanswered_channels"]
