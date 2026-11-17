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

"""The scripted-oracle stand-in that ``--dry-run`` puts where the agent goes.

``--dry-run`` exists to walk a whole schedule through **everything except the
Claude session** -- staging, the quarantine check, deliverable discovery,
phase B, the real tier grader, the §3 row, resume, publication -- without
spending one token of the owner's quota. The substitute is the tier's own
**committed scripted oracle**, so the plumbing is exercised on a deliverable a
real column really produced rather than on a fixture.

**A dry-run row is not a cell and can never become one.** Every row it writes
carries ``dry_run: true``, ``condition: "scripted_oracle"`` and
``metrics_source: "scripted_oracle_dry_run"``, and the agent-cost columns are
``null`` because there was no agent -- exactly the ``capability-ladder-plan.md``
§2 distinction the oracles' own docstrings already make ("*a scripted control
that a human wrote knowing the thresholds*").

Coverage is what is COMMITTED, and the gaps are the point rather than an
embarrassment: a (tier, column) with no committed oracle returns
``ok=False`` with the reason, the cell records
``blocker=scaffolding_defect_ours``, and the dry run has just demonstrated the
runner's own honesty path on a real hole.

============================  ==================================================
(tier, column)                 oracle
============================  ==================================================
``T1_arrive`` / ``mujoco``     ``ladder.adapters.mujoco.runner`` -- drives AND
                               records in one process, so there is **no
                               separable deliverable**: the oracle returns its
                               run directory as phase-B evidence and the cell
                               grades from it
``T2_transfer`` / ``mujoco``   ``run_t2.build_deliverable`` -> a
                               ``scene.xml`` + ``drive.py`` directory the cell
                               re-runs cold through the real phase B
``T3_quadruped`` / ``mujoco``  ``run_t3.build_deliverable`` -- same shape
``T4_humanoid`` / ``mujoco``   ``run_t4.build_deliverable`` -- same shape
everything else                no committed oracle (the omnisim column has
                               none at any tier)
============================  ==================================================

Every oracle is pointed at the **staged** ``container/`` in the workspace, not
at the repo's. That is deliberate: if the staging dropped a mesh or flattened
a package path, the oracle fails to build and the dry run says so -- which is
one more thing proven without an agent.
"""

from __future__ import annotations

import shutil
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
LADDER = HERE.parent
BENCHMARKS = LADDER.parent
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))


@dataclass
class OracleSession:
    """What a scripted oracle produced, in the shape the cell consumes."""

    ok: bool = False
    deliverable: str = None        # what phase B should re-run, if separable
    phase_b_dir: str = None        # set when the oracle ALREADY ran phase B
    wall_s: float = None
    error: str = None
    detail: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)

    def as_dict(self):
        return {"ok": self.ok, "deliverable": self.deliverable,
                "phase_b_dir": self.phase_b_dir, "wall_s": self.wall_s,
                "error": self.error, "detail": self.detail,
                "notes": list(self.notes)}


class _StagedContainer:
    """A task proxy whose ``container_dir`` is the STAGED one.

    The committed oracles read exactly one thing off the task -- where the
    container is -- so pointing them at the workspace's copy needs a proxy and
    not a fork of their builders. Everything else forwards.
    """

    def __init__(self, task, container_dir):
        self._task = task
        self.container_dir = Path(container_dir)

    def __getattr__(self, name):
        return getattr(self._task, name)


def _t1_mujoco(task, workspace, run_dir, container_root):
    from ladder.adapters.mujoco import runner
    phase = task.standalone
    desc = Path(container_root) / "husky_description"
    if not desc.is_dir():
        return OracleSession(
            ok=False, error="the staged container has no husky_description/ "
                            "at %s -- staging dropped the robot" % desc)
    out = Path(run_dir) / "oracle"
    t0 = time.monotonic()
    proc = runner.launch(
        out, goal_offset=tuple(task.waypoint_spec.get("offset_m", [0.0, 5.0])),
        description=str(desc),
        extra_args=["--settle-s", str(phase.get("settle_s", 0.5)),
                    "--drive-timeout-s", str(phase.get("duration_s", 60.0)),
                    "--contact-stride", str(phase.get("contact_stride", 10))])
    wall = round(time.monotonic() - t0, 2)
    ok = proc.get("exit_code") == 0 and not proc.get("timed_out")
    return OracleSession(
        ok=ok, deliverable=None, phase_b_dir=str(out), wall_s=wall,
        error=(None if ok else "the oracle process exited %s (timed_out=%s)"
               % (proc.get("exit_code"), proc.get("timed_out"))),
        detail={"process": proc},
        notes=["the T1 mujoco oracle drives AND records in one process, so "
               "there is no separable deliverable for phase B to re-run; its "
               "run directory IS the phase-B evidence and the cell grades "
               "from it through the real ladder.graders.t1 path"])


def _t2_mujoco(task, workspace, run_dir, container_root):
    from ladder.adapters.mujoco import run_t2
    out = Path(run_dir) / "oracle"
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.monotonic()
    deliverable, res = run_t2.build_deliverable(
        out, task=_StagedContainer(task, container_root))
    wall = round(time.monotonic() - t0, 2)
    if deliverable is None:
        return OracleSession(ok=False, wall_s=wall,
                             error="the oracle could not build a scene from "
                                   "the STAGED container: %s"
                                   % "; ".join(res.problems),
                             detail={"problems": list(res.problems)})
    return OracleSession(ok=True, deliverable=str(deliverable), wall_s=wall,
                         detail={"problems": list(res.problems)},
                         notes=["deliverable built from the staged "
                                "container/, so a staging defect would have "
                                "failed the build rather than passing"])


def _t3_mujoco(task, workspace, run_dir, container_root):
    from ladder.adapters.mujoco import run_t3
    out = Path(run_dir) / "oracle"
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.monotonic()
    deliverable, res = run_t3.build_deliverable(
        out, task=_StagedContainer(task, container_root))
    wall = round(time.monotonic() - t0, 2)
    if deliverable is None:
        return OracleSession(ok=False, wall_s=wall,
                             error="the oracle could not build a scene from "
                                   "the STAGED container: %s"
                                   % "; ".join(res.problems),
                             detail={"problems": list(res.problems)})
    return OracleSession(ok=True, deliverable=str(deliverable), wall_s=wall,
                         detail={"problems": list(res.problems)},
                         notes=["deliverable built from the staged "
                                "container/"])


def _t4_mujoco(task, workspace, run_dir, container_root):
    """T4's oracle. Imported lazily and guarded, like its siblings.

    Registered on the strength of the module existing and passing its own
    tests. If it is not importable or its API moves, ``run_oracle`` catches it
    and the cell reads ``blocker=scaffolding_defect_ours`` -- which is the
    correct reading of a broken oracle of ours, and never a finding about
    MuJoCo.
    """
    from ladder.adapters.mujoco import run_t4
    out = Path(run_dir) / "oracle"
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.monotonic()
    deliverable, res = run_t4.build_deliverable(
        out, task=_StagedContainer(task, container_root))
    wall = round(time.monotonic() - t0, 2)
    if deliverable is None:
        return OracleSession(ok=False, wall_s=wall,
                             error="the oracle could not build a scene from "
                                   "the STAGED container: %s"
                                   % "; ".join(res.problems),
                             detail={"problems": list(res.problems)})
    return OracleSession(ok=True, deliverable=str(deliverable), wall_s=wall,
                         detail={"problems": list(res.problems)},
                         notes=["deliverable built from the staged "
                                "container/; the tier's own file records "
                                "that its supported cell is reachable with a "
                                "scripted gait and its UNSUPPORTED cell is "
                                "not -- the oracle's cell is whatever the "
                                "wrench says, and the grader routes it"])


def _t1_omnisim(task, workspace, run_dir, container_root):
    """T1's scripted control on the OmniSim column.

    Builds a world holding the STAGED container's Husky and a hand-written
    drive controller (``ladder/controllers/ladder_t1_drive``), which reads its
    own pose through ``supervisor TRUE`` and stops on the commanded point. The
    offset comes from ``meta.json`` and reaches the controller as an argument;
    nothing observed at runtime supplies it.

    ⚠ **ODE, explicitly.** The same world under `defaultPhysicsBackend
    "newton"` records a 296 kB motion file whose body roster is empty, so the
    grader reports *"the run reported no bodies at all"* -- an open item on
    this column, recorded rather than worked around, and the reason this
    oracle names its backend instead of taking the engine default.
    """
    from ladder.adapters.omnisim import t1_scene
    staged = _StagedContainer(task, container_root)
    out = Path(run_dir) / "oracle"
    out.mkdir(parents=True, exist_ok=True)
    desc = (Path(container_root)
            / Path(task.meta["container"]["description_dir"]).name)
    t0 = time.monotonic()
    try:
        phase = task.standalone
        world = t1_scene.build_world(
            out, description_dir=desc,
            offset_m=tuple(task.waypoint_spec.get("offset_m", [0.0, 5.0])),
            settle_s=float(phase.get("settle_s", 0.5)),
            duration_s=float(phase.get("duration_s", 60.0)),
            backend="ode")
    except (OSError, KeyError, FileNotFoundError) as exc:
        return OracleSession(ok=False, wall_s=round(time.monotonic() - t0, 2),
                             error="the oracle could not build a scene from "
                                   "the STAGED container: %r" % (exc,))
    del staged
    return OracleSession(ok=True, deliverable=str(world),
                         wall_s=round(time.monotonic() - t0, 2),
                         notes=["deliverable built from the staged container/",
                                "backend pinned to ode; see the docstring for "
                                "the newton open item"])


#: ``(task_id, column) -> callable(task, workspace, run_dir, container_root)``
ORACLES = {
    ("T1_arrive", "mujoco"): _t1_mujoco,
    ("T1_arrive", "omnisim"): _t1_omnisim,
    ("T2_transfer", "mujoco"): _t2_mujoco,
    ("T3_quadruped", "mujoco"): _t3_mujoco,
    ("T4_humanoid", "mujoco"): _t4_mujoco,
}

NO_ORACLE = (
    "no committed scripted oracle for (%s, %s). The dry run cannot stand in "
    "for an agent here, and the blocker is OURS -- this is a hole in our "
    "scaffolding, never a finding about the simulator "
    "(capability-ladder-plan.md 4's binding rule).")


def has_oracle(task_id, column):
    return (task_id, column) in ORACLES


def run_oracle(task, column, workspace, run_dir, container_root):
    """Run the committed oracle for ``(task, column)``. Never raises."""
    fn = ORACLES.get((task.id, column))
    if fn is None:
        return OracleSession(ok=False, error=NO_ORACLE % (task.id, column))
    try:
        return fn(task, Path(workspace), Path(run_dir), Path(container_root))
    except Exception as exc:                              # noqa: BLE001
        return OracleSession(ok=False,
                             error="the oracle raised %r" % (exc,))


def copy_deliverable(deliverable, dest_dir):
    """Preserve the oracle's deliverable beside the row (evidence, per §3.1).

    Returns the copied path, or None when there was nothing separable to copy.
    """
    if not deliverable:
        return None
    src = Path(deliverable)
    dest = Path(dest_dir)
    dest.mkdir(parents=True, exist_ok=True)
    if src.is_dir():
        out = dest / src.name
        if out.exists():
            shutil.rmtree(out)
        shutil.copytree(src, out)
        return str(out)
    out = dest / src.name
    shutil.copy2(src, out)
    return str(out)


__all__ = ["ORACLES", "OracleSession", "has_oracle", "run_oracle",
           "copy_deliverable", "NO_ORACLE"]
