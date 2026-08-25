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

"""The simulator registry -- the single source of truth for the comparator set.

AgenticSimBench compares a **primary leaderboard** of seven systems and tracks
an **extended research set** separately (SPEC 6.1, 19). This module is what the
runners, the campaign driver and the report generator all read, so "which
simulators does this benchmark cover" has exactly one answer in the tree.

The distinction that matters here is **declared** vs **implemented**:

* ``status="implemented"`` -- an adapter exists and cells can be SCORED.
  That is a weaker claim than it used to make, deliberately: the old wording
  folded "the oracle/null gate is green" into the same word, and the MuJoCo
  arm landed with a working adapter, a task graded end to end from measured
  evidence, and NO oracle gate -- honest in its ``pending`` text while the
  status field said otherwise. A word that code branches on must not carry a
  claim the code does not check.
* :attr:`Sim.publishable` -- implemented AND nothing left in ``pending``.
  THIS is the SPEC 6.2 bar: only a publishable arm may appear in a published
  comparison. Splitting the two lets an arm be exercised exploratorily (which
  is how its remaining gaps get found) without its number being quotable.
  Its per-task sibling :meth:`Sim.publishable_for` is the one a readiness
  table should ask -- see :class:`Pending`.
* ``status="declared"`` -- the comparator is part of the frozen design and its
  bring-up is specified, but no adapter exists yet. Asking for one is a clear
  ``NotImplementedError`` naming what is missing -- never a silent argparse
  rejection that makes the comparator look like it was never in the design.

That difference is the honesty property this module exists for. A benchmark
that supports two simulators while its specification claims seven should say so
in code, not only in prose.

There is a third fact between those two, and ``tasks`` / ``pending`` carry it.
An arm can have a working adapter and still be unable to express part of the
suite -- most often because a task ships its fixture in one simulator's file
format and no equivalent exists for the other. That is a missing FIXTURE, not
a failing run, and the two must never be scored the same way (SPEC 6.4). So
``tasks`` names the subset an arm can score today (``None`` = all of them) and
``require_implemented(sid, task)`` refuses the rest up front rather than
letting them become failures attributed to that simulator. ``pending`` records
what still has to land before a *published* number, which is a stricter bar
than "a cell runs".

``pending`` is a list of :class:`Pending` items and each one declares WHICH
TASKS it blocks. It used to be one free-text string, which made an arm's
publication bar the worst of its tasks: the MuJoCo arm's R1 gate was green and
pinned by a test while two items about A1 and R2 kept the whole arm
unpublishable. That is not caution, it is imprecision -- and imprecision in
this direction is not free, because the only way to clear it is to delete the
text, which is how a real gap gets lost. Scoping is therefore *declared*, and
a scoped item must say why the other tasks are immune (SPEC 6.2, 7.1).

``vendor_minimum`` is metadata for the platform ladder (SPEC 20.6), archived
from each vendor's own published requirements page. It is **never** used to
skip a cell: a machine below a vendor minimum is still attempted and its real
outcome recorded (``PASS-BELOW-MIN`` when it works). Asserting failure in
advance is exactly the thing SPEC 20.2 forbids.
"""

from __future__ import annotations


class Pending:
    """One thing that must land before a number from this arm is PUBLISHED.

    ``blocks`` is the load-bearing half. Bring-up debt is rarely arm-wide --
    "A1 has no oracle yet" says nothing about R1 -- so the scope is declared
    per item rather than inferred from prose:

    * ``blocks=None`` -- every task this arm can express. The DEFAULT, because
      an unscoped gap is the safe reading of an item nobody has thought about,
      and it is also exactly what a bare string used to mean.
    * ``blocks=("A1_husky_swarm_10",)`` -- those tasks only. ``why_scoped`` is
      then REQUIRED and must state what makes the remaining tasks immune. A
      narrowed scope with no argument behind it is a green nobody has to
      justify, which is the C2 failure mode wearing a different hat; the
      constructor refuses it rather than trusting the author to remember.

    Narrowing a scope is NOT the same as clearing the item: a task-scoped item
    still blocks its own tasks, still prints in ``pending``, and still has to
    be closed before those cells publish. What it stops doing is blocking
    unrelated tasks.
    """

    def __init__(self, pid, *, detail, blocks=None, why_scoped=None,
                 evidence=None):
        if not pid or not detail:
            raise ValueError("a pending item needs an id and a detail")
        if blocks is not None:
            blocks = tuple(blocks)
            if not blocks:
                raise ValueError(
                    "%s: blocks=() would block nothing at all; pass "
                    "blocks=None for an arm-wide item" % (pid,))
            if not (why_scoped or "").strip():
                raise ValueError(
                    "%s is scoped to %s but gives no why_scoped. An item that "
                    "stops blocking a task must say WHY that task is immune, "
                    "or the scoping is an unjustified green (SPEC 7.1)."
                    % (pid, ", ".join(blocks)))
        self.id = pid
        self.detail = detail
        self.blocks = blocks              # tuple of task ids, or None = all
        self.why_scoped = why_scoped
        self.evidence = evidence          # where the claim can be checked

    def blocks_task(self, task_id):
        """Does this item block ``task_id``? Unscoped items block everything."""
        return self.blocks is None or task_id in self.blocks

    @property
    def scope_label(self):
        return ("EVERY task this arm expresses" if self.blocks is None
                else "blocks " + ", ".join(self.blocks))

    def text(self):
        """The item as one human-readable line, scope first."""
        out = "[%s] %s" % (self.scope_label, self.detail)
        if self.why_scoped:
            out += " -- scoped because: %s" % self.why_scoped
        if self.evidence:
            out += " (evidence: %s)" % self.evidence
        return out

    def as_dict(self):
        return {"id": self.id, "detail": self.detail,
                "blocks": list(self.blocks) if self.blocks else None,
                "why_scoped": self.why_scoped, "evidence": self.evidence}

    def __repr__(self):
        return "<Pending %s (%s)>" % (self.id, self.scope_label)


class Sim:
    """One comparator's frozen identity."""

    def __init__(self, sid, *, name, role, tier, status, surface,
                 version=None, adapter=None, vendor_minimum=None,
                 blocked_by=None, notes=None, tasks=None, pending=None):
        self.id = sid
        self.name = name
        self.role = role
        self.tier = tier                  # "primary" | "extended"
        self.status = status              # "implemented" | "declared"
        self.surface = surface            # the shell+tools condition
        self.version = version            # pinned; None until bring-up
        self.adapter = adapter            # dotted module path, when built
        self.vendor_minimum = vendor_minimum or {}
        self.blocked_by = blocked_by      # what bring-up needs, if declared
        self.notes = notes
        # The task ids this arm can EXPRESS today. ``None`` means every task,
        # which is what both original arms are. A subset is not a soft warning
        # in prose: a task whose fixture does not exist for a simulator cannot
        # be scored on it, and an arm that answers "implemented" to every task
        # and then produces a bundle with no run is the exact failure mode the
        # implemented/declared split exists to prevent. See ``expressible``.
        self.tasks = tuple(tasks) if tasks else None
        # What still has to land before a PUBLISHED number from this arm --
        # recorded even when the arm is scoreable, because "it runs" and "its
        # number may be published" are different claims (SPEC 6.2). Accepts a
        # bare string for compatibility, which is read as ONE arm-wide item:
        # the conservative reading, and the one the string always had.
        if pending is None:
            items = ()
        elif isinstance(pending, str):
            items = (Pending("legacy", detail=pending),)
        elif isinstance(pending, Pending):
            items = (pending,)
        else:
            items = tuple(pending)
            for it in items:
                if not isinstance(it, Pending):
                    raise TypeError(
                        "pending takes Pending items (or one string); got %r"
                        % (it,))
        self.pending_items = items

    @property
    def implemented(self):
        """An adapter exists and cells can be scored. NOT a publication bar."""
        return self.status == "implemented"

    @property
    def pending(self):
        """Everything still outstanding on this arm, as one readable string.

        Kept as a string-shaped property so every existing reader (the
        ``require_implemented`` message, ``coverage()``) is unchanged, and so
        ``if sim.pending`` still means "something is outstanding somewhere on
        this arm". Per-task readers want :meth:`pending_for`.
        """
        if not self.pending_items:
            return None
        return "; ".join("(%d) %s: %s" % (n, it.id, it.text())
                         for n, it in enumerate(self.pending_items, 1))

    def pending_for(self, task_id):
        """The pending items that block THIS task on this arm."""
        return tuple(it for it in self.pending_items if it.blocks_task(task_id))

    @property
    def publishable(self):
        """May a number from this arm be PUBLISHED (SPEC 6.2)?

        Implemented is not enough. An arm whose oracle/null gate has never
        run has not shown its tasks discriminate on it, and a pass rate from
        such an arm is unfalsifiable -- exactly the defect C2 shipped with,
        where a task nobody had proven could FAIL passed 5/5 unfixed for a
        whole campaign. ``pending`` names what is missing; while it is
        non-empty the arm runs but does not publish.

        This is the WHOLE-ARM claim: nothing outstanding on any task. It is
        the right question for "is this arm finished" and the wrong one for
        "may this cell publish" -- use :meth:`publishable_for` for that.
        """
        return self.implemented and not self.pending_items

    def publishable_for(self, task_id):
        """May a number for THIS (task, arm) be published, as far as the
        registry knows?

        Three conditions, and the third is why this is not just
        ``not pending_for(task)``: an arm that cannot EXPRESS a task has no
        number to publish, so answering "publishable" there would be a green
        on an empty cell.

        This is a NECESSARY condition, never a sufficient one. The registry
        knows what bring-up is outstanding; it does not know whether the
        task's oracle/null gate has been shown green on this arm, which lives
        in the recorded verdicts and is checked separately (``readiness.py``
        ANDs the two). Do not let anything read this as the whole bar.
        """
        return (self.implemented and self.expresses(task_id)
                and not self.pending_for(task_id))

    def expresses(self, task_id):
        """Can this arm score ``task_id`` today?"""
        return self.tasks is None or task_id in self.tasks

    def __repr__(self):
        return "<Sim %s (%s, %s)>" % (self.id, self.tier, self.status)


#: The primary leaderboard (SPEC 19.1). Seven systems; the public headline
#: comparison is these and only these.
_PRIMARY = [
    Sim("omnisim",
        name="OmniSim",
        role="system under evaluation",
        tier="primary", status="implemented",
        version="repo sha (pinned per campaign)",
        adapter="agentbench.adapters.omnisim",
        surface="18 MCP tools generated from the shipped packages/omnisim-mcp "
                "registry (SPEC 4.2 asserts equality with what ships)",
        vendor_minimum={"declared": False,
                        "note": "no published quantitative minimum"},
        notes="Developed on an RTX 3060 laptop (P2). macOS is untested, so "
              "OmniSim expects UNSUPPORTED-OS on its own Apple Silicon row -- "
              "recorded, not hidden."),

    Sim("webots",
        name="Webots (upstream)",
        role="control experiment",
        tier="primary", status="implemented",
        version="R2025a",
        adapter="agentbench.adapters.webots",
        surface="operator-authored bridge over the published Supervisor / "
                "controller API reference (SPEC 4.1.1 -- upstream publishes no "
                "service definitions to generate from, so we write our "
                "competitor's tools and the 6.2 fidelity machinery binds)",
        vendor_minimum={"declared": True, "class": "low CPU/RAM baseline",
                        "source": "cyberbotics.com/doc/guide/"
                                  "system-requirements"},
        notes="THE load-bearing baseline: same file format, same base engine, "
              "no OmniSim harness -- so any delta is exactly the surface we "
              "added. If OmniSim does not clearly beat it, the agent-native "
              "claim is dead and that publishes as prominently as a win."),

    Sim("mujoco",
        name="MuJoCo",
        role="strongest authoring-tier adversary",
        tier="primary", status="implemented",
        version="3.8.1",
        adapter="agentbench.adapters.mujoco",
        surface="the mujoco Python API -- which on this simulator IS the "
                "surface: MJCF is inert data and every MuJoCo program is "
                "Python, so the shell condition's write_file + a POSIX shell "
                "already hands an agent the whole documented API. A separate "
                "shell+tools bridge is NOT built (adapters/mujoco/"
                "EXCLUSIONS.md sec. 1), so only the shell condition is "
                "runnable today",
        vendor_minimum={"declared": True, "cpu": "AVX-capable",
                        "os": "Windows / Linux / macOS",
                        "source": "mujoco.readthedocs.io/en/stable/"
                                  "programming/"},
        # Only the AUTHORING tasks. The other six ship a `.wbt` fixture and
        # there is no `initial_mujoco/` MJCF equivalent, so they are not
        # expressible on this arm yet -- a missing FIXTURE, not a missing
        # MuJoCo capability, and the difference is the point. A1/R1/R2 start
        # from an empty (or world-free) `initial/`, which stages correctly for
        # this arm with no task-side change at all.
        tasks=("A1_husky_swarm_10", "R1_lidar_nav", "R2_arm_reach"),
        # The gate is per (task, arm) -- BRINGUP.md sec. 5 says so in those
        # words -- so the debt is recorded per task too. R1's gate IS green on
        # this arm and is on record in preregister/oracle_verdicts.json
        # (driver_gates: oracle PASS, null FAIL), so nothing below blocks R1.
        # The .xml + driver deliverable wiring, once item (1) of a single
        # free-text string, landed in 385319148 and is verified end to end
        # through cc_lane.grade_mujoco -- it is closed, not descoped.
        pending=(
            Pending(
                "A1_gate",
                blocks=("A1_husky_swarm_10",),
                detail="the oracle/null gate (SPEC 7.1) has never been run "
                       "for A1_husky_swarm_10 on this arm, so an A1 number "
                       "from it is unfalsifiable",
                why_scoped="a gate is a property of a (task, arm) pair, not "
                           "of an arm: R1's gate on this same arm was run and "
                           "is on record, and it says nothing about A1. "
                           "Blocking R1 on A1's missing gate would be the "
                           "opposite error to C2's -- a red with no argument "
                           "behind it -- and it costs the same thing, because "
                           "the only way to clear it is to delete the item.",
                evidence="adapters/mujoco/BRINGUP.md sec. 5 'Still not run'"),
            Pending(
                "R2_gate",
                blocks=("R2_arm_reach",),
                detail="the oracle/null gate (SPEC 7.1) has never been run "
                       "for R2_arm_reach on this arm, so an R2 number from it "
                       "is unfalsifiable",
                why_scoped="same reason as A1_gate: the gate is per (task, "
                           "arm). R2 shares no fixture, driver or grader with "
                           "R1 -- R1 is graded by graders.r1_core against the "
                           "mujoco_lane scene, R2 has no lane fixture at all "
                           "-- so R1's green cannot have come from anything "
                           "R2 is missing, and vice versa.",
                evidence="adapters/mujoco/BRINGUP.md sec. 5 'Still not run'"),
            Pending(
                "A1_husky_analogue",
                blocks=("A1_husky_swarm_10",),
                detail="MuJoCo ships no Husky, so A1's flagship identity "
                       "predicate needs an explicitly declared analogue "
                       "(robot_model=) exactly as the Webots arm does; "
                       "without one the predicate is a definite NO",
                why_scoped="it is an A1 predicate. R1 is graded with "
                           "robot_identity='any_robot' (tasks/R1_lidar_nav "
                           "asks the agent to author its own robot and "
                           "names none), so no Husky, analogue or identity "
                           "predicate enters an R1 verdict -- verified in "
                           "adapters/mujoco/test_r1_discriminates_mujoco.py, "
                           "which builds every R1 bundle with 'any_robot'.",
                evidence="adapters/mujoco/evidence.py robot_model=; "
                         "test_mujoco_adapter.py::test_husky_identity_without"
                         "_a_declared_analogue_is_a_definite_no"),
        ),
        notes="Cheapest competitor to stand up and the most likely to beat us "
              "on authoring: MJCF is compact and heavily represented in "
              "training data. Included BECAUSE it may win. Bring-up "
              "2026-08-09 on machine 9722d23d12a3 (adapters/mujoco/"
              "BRINGUP.md): mujoco 3.8.1 was ALREADY present as a Newton "
              "dependency, so nothing was installed for it. Two capabilities "
              "R1's oracle/null gate (SPEC 7.1) IS green here and is the "
              "reason R1 carries no pending item: an mj_ray navigator PASSes "
              "6/6 (14.272 m path, 0.094 m from goal, zero contacts) and a "
              "do-nothing driver FAILs R1.4/R1.5/R1.6 on the same scene -- "
              "BRINGUP.md sec. 3.3, pinned by test_r1_discriminates_mujoco.py "
              "and recorded in preregister/oracle_verdicts.json. Two "
              "capabilities "
              "this arm has that neither other arm does, both stated because "
              "they cut against us: its t=0 scan bounds EVERY body and world "
              "geom without a name list (so R1.3's instrument gap does not "
              "exist here), and its engine attribution is a per-run READING "
              "of the compiled model's solver/integrator/cone rather than a "
              "structural claim."),

    Sim("gazebo",
        name="Gazebo Sim",
        role="ROS-ecosystem comparator",
        tier="primary", status="declared",
        version="Jetty 10.0.0 LTS",
        blocked_by="ros_gz bring-up + a simulation_interfaces client",
        surface="ROS 2 simulation_interfaces (SpawnEntity, DeleteEntity, "
                "Get/SetEntityState, GetEntityBounds, StepSimulation, "
                "LoadWorld, ...) plus the gz CLI",
        vendor_minimum={"declared": True, "os": "Ubuntu 24.04 amd64 official",
                        "note": "Windows/macOS/ARM are best-effort",
                        "source": "gazebosim.org/docs/latest/install/"},
        notes="Has per-entity spawn/delete/set-pose services that OmniSim's "
              "harness lacks entirely. If those beat file-and-reload we lose "
              "the authoring tier, and that is the finding."),

    Sim("isaac",
        name="NVIDIA Isaac Sim",
        role="GPU/RTX platform comparator",
        tier="primary", status="declared",
        version="6.0",
        blocked_by="RunPod 4090 pod (local 3060/6 GB is under the floor) + "
                   "isaacsim.ros2.sim_control bring-up",
        surface="isaacsim.ros2.sim_control (19 services + 1 action) plus the "
                "Kit Python API, because that is how Isaac users script it",
        vendor_minimum={"declared": True, "ram_gb": 32, "gpu": "RTX 4080",
                        "vram_gb": 16,
                        "os": "Ubuntu 22.04/24.04 or Windows 11",
                        "note": "container is Linux-only; no-RT-core GPUs "
                                "(A100/H100) unsupported",
                        "source": "docs.isaacsim.omniverse.nvidia.com/6.0.0/"
                                  "installation/requirements.html"},
        notes="Its declared minimum puts P0-P3 below floor. Those cells are "
              "still ATTEMPTED and their real outcome recorded (SPEC 20.2) -- "
              "they are the most informative platform cells in the matrix."),

    Sim("coppeliasim",
        name="CoppeliaSim",
        role="mature general-purpose comparator",
        tier="primary", status="declared",
        version=None,
        blocked_by="install + ZeroMQ remote-API client; physics backend must "
                   "be pinned and disclosed (it ships several)",
        surface="the ZeroMQ remote API / integrated scripting and the "
                "headless command-line path",
        vendor_minimum={"declared": False},
        notes="Pin ONE physics backend per release and disclose it, or the "
              "row is not a comparison with itself between releases."),

    Sim("genesis",
        name="Genesis",
        role="Python-first physical-AI comparator",
        tier="primary", status="declared",
        version="1.2.1",
        blocked_by="install + adapter; CPU backend needed for the P0/P1 rows",
        surface="the Pythonic simulation interface (CPU and GPU backends)",
        vendor_minimum={"declared": True,
                        "os": "Linux / macOS / Windows",
                        "note": "Linux + CUDA recommended for performance",
                        "source": "genesis-world.readthedocs.io/en/v1.2.1/"
                                  "user_guide/overview/installation.html"},
        notes=None),
]

#: The extended research set (SPEC 19.2) -- published as a SECOND table, never
#: merged into the primary leaderboard, because these are not all the same
#: product category.
_EXTENDED = [
    Sim("pybullet", name="PyBullet", role="low-resource baseline",
        tier="extended", status="declared", blocked_by="adapter",
        surface="the pybullet Python API"),
    Sim("sapien", name="SAPIEN", role="embodied-AI comparator",
        tier="extended", status="declared", blocked_by="adapter",
        surface="the SAPIEN Python API"),
    Sim("newton", name="Newton", role="emerging GPU physics engine",
        tier="extended", status="declared", blocked_by="adapter",
        surface="the Newton/Warp Python API",
        notes="Reported as its OWN row and never conflated with OmniSim's "
              "results: Newton is also OmniSim's only physics backend, so a "
              "reader could otherwise mistake one row for the other."),
]

SIMS = {s.id: s for s in (_PRIMARY + _EXTENDED)}

#: Ordered ids for the primary leaderboard, report columns and the platform
#: heatmap. Order is frozen so charts do not silently reshuffle.
PRIMARY = tuple(s.id for s in _PRIMARY)
EXTENDED = tuple(s.id for s in _EXTENDED)

#: The arms that can actually be scored today. Runners validate against this.
IMPLEMENTED = tuple(s.id for s in _PRIMARY + _EXTENDED if s.implemented)

#: Declared-but-unbuilt. Naming these in errors (rather than hiding them) is
#: the point: the gap between the design and the tree stays visible.
DECLARED = tuple(s.id for s in _PRIMARY + _EXTENDED if not s.implemented)


def get(sid):
    """Look up a comparator, or raise naming the full declared set."""
    if sid in SIMS:
        return SIMS[sid]
    raise KeyError(
        "unknown simulator %r; primary: %s; extended: %s"
        % (sid, ", ".join(PRIMARY), ", ".join(EXTENDED)))


def require_implemented(sid, task=None):
    """Resolve ``sid`` for a scored run, or explain precisely what is missing.

    A declared-but-unbuilt comparator raises ``NotImplementedError`` naming its
    blocker -- not ``KeyError`` -- so the caller can tell "this simulator is not
    part of the benchmark" from "this simulator is part of the benchmark and
    has not been built yet". Those are different facts and only one of them is
    a bug in the request.

    ``task`` adds the third fact: an implemented arm that cannot express THIS
    task. An arm whose fixture set covers only part of the suite is not a
    broken arm, and its unrunnable cells must not silently become failures
    attributed to that simulator -- so passing the task id here refuses them up
    front, naming what the arm does cover. Omitting it keeps the old
    behaviour exactly, so every existing caller is unchanged.
    """
    sim = get(sid)
    if not sim.implemented:
        raise NotImplementedError(
            "%s (%s) is a DECLARED %s comparator with no adapter yet: %s. "
            "Implemented arms: %s."
            % (sim.id, sim.name, sim.tier, sim.blocked_by or "bring-up needed",
               ", ".join(IMPLEMENTED)))
    if task is not None and not sim.expresses(task):
        raise NotImplementedError(
            "%s (%s) has an adapter but cannot express task %r yet; it covers "
            "%s. This is a missing FIXTURE for that task on that simulator, "
            "not a failing run -- scoring it would attribute our own gap to "
            "%s (SPEC 6.4). Remaining bring-up: %s"
            % (sim.id, sim.name, task, ", ".join(sim.tasks or ()), sim.name,
               sim.pending or "see the arm's BRINGUP.md"))
    return sim


def coverage():
    """Rows for the coverage table: what is claimed vs what can be run."""
    return [{"id": s.id, "name": s.name, "tier": s.tier, "role": s.role,
             "status": s.status, "version": s.version,
             "blocked_by": s.blocked_by,
             "tasks": list(s.tasks) if s.tasks else None,
             "pending": s.pending,
             # The scoped form as well as the rendered string: a reader that
             # wants "which cells does this block" should not have to parse
             # prose to find out.
             "pending_items": [it.as_dict() for it in s.pending_items]}
            for s in (_PRIMARY + _EXTENDED)]


__all__ = ["Pending", "Sim", "SIMS", "PRIMARY", "EXTENDED", "IMPLEMENTED",
           "DECLARED", "get", "require_implemented", "coverage"]
