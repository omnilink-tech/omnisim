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

"""The ladder's task registry -- deliberately the same shape as agentbench's.

One directory per rung holding ``prompt.txt`` (the exact one sentence),
``meta.json`` (the rung, the grader entry point, the phase config, the
thresholds, the pre-registered expectation) and ``container/`` (the files the
agent is given, and nothing else).

``prompt.txt`` is byte-identical across simulators. There is no per-simulator
prompt tuning, ever.

Two differences from ``agentbench.tasks``, both from
``capability-ladder-plan.md``:

* ``container/`` rather than ``initial/``. The word matters: an ``initial/``
  ships a *starting scene*, and the whole point of T1 is that no column gets
  one (§2 T1).
* the waypoint. A rung may carry a **start-relative** goal, resolved against
  the run's own first recorded sample by :func:`resolve_waypoint`. It lives in
  ``meta.json`` -- grader-side -- and never in ``container/``.
"""

from __future__ import annotations

import importlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


class Task:
    def __init__(self, path):
        self.dir = Path(path)
        self.meta = json.loads((self.dir / "meta.json").read_text("utf-8"))
        self.id = self.meta["id"]
        self.prompt = (self.dir / self.meta.get("prompt_file", "prompt.txt")
                       ).read_text("utf-8").strip()

    # -- the rung ----------------------------------------------------------
    @property
    def rung(self):
        return self.meta.get("rung")

    @property
    def outcome(self):
        return self.meta.get("outcome")

    @property
    def repeats_default(self):
        return self.meta.get("repeats_default", 3)

    @property
    def par_s(self):
        return self.meta.get("par_s")

    @property
    def timeout_s(self):
        return self.meta.get("timeout_s", 3 * (self.meta.get("par_s") or 0))

    # -- the files -----------------------------------------------------------
    @property
    def container_dir(self):
        """This task's ``container/``, or the one it declares it shares.

        ``container.shared_from: "<task id>"`` makes a rung reuse another's
        assets instead of copying them. That is not tidiness -- two rungs
        posed on the same robot MUST be posed on the byte-identical robot, or
        a difference between their rows could be the asset.

        ⚠ **Declaring the reuse and not implementing it staged an EMPTY
        container.** Measured 2026-08-02 on L2's first cell: the agent found
        nothing in ``container/``, went and copied the description out of the
        benchmark's own tree, and solved the task in a different simulator
        than the column under test. The cell measured nothing. A meta file
        that names a description directory must therefore either ship it or
        say where it comes from -- and :func:`check` enforces that.
        """
        own = self.dir / "container"
        if own.is_dir():
            return own
        shared = (self.meta.get("container", {}) or {}).get("shared_from")
        if shared:
            return Path(__file__).resolve().parent / shared / "container"
        return own

    @property
    def container_files(self):
        names = (self.meta.get("container", {}) or {}).get("files", [])
        return [self.container_dir / n for n in names]

    # -- grading data ---------------------------------------------------------
    @property
    def standalone(self):
        return (self.meta.get("phases", {}) or {}).get("standalone", {}) or {}

    @property
    def wants_harness(self):
        return bool((self.meta.get("phases", {}) or {})
                    .get("harness", {}).get("enabled"))

    @property
    def constants(self):
        return self.meta.get("constants", {}) or {}

    @property
    def robot_name(self):
        return (self.meta.get("robot", {}) or {}).get("declared_name", "")

    @property
    def robot_mass_kg_declared(self):
        """The declared mass of the robot under test, kg, or None.

        Used to resolve the base body on a column whose importer does not
        reproduce the declared link name -- an identity check, never a graded
        quantity. See ``ladder.graders.t3_core.select_base_body``.
        """
        m = (self.meta.get("robot", {}) or {}).get("mass_kg_declared")
        return None if m is None else float(m)

    @property
    def waypoint_spec(self):
        return self.meta.get("waypoint", {}) or {}

    @property
    def roles(self):
        """Which body is which, for a rung that grades more than one.

        Task data, exactly like the waypoint and for the same reason: an
        adapter that could choose which body counts as "the object" could
        choose the one that happens to be in the container. It lives in
        ``meta.json`` -- grader-side -- and never in ``container/``.
        """
        return self.meta.get("roles", {}) or {}

    @property
    def surfaces(self):
        """Names the grader asks the sampler to bound, if the sim can."""
        return list(self.standalone.get("surfaces", []) or [])

    def grader(self):
        return importlib.import_module(self.meta["grader"])

    def citation(self):
        return "%s (%s)" % (self.dir.name + "/meta.json", self.id)

    def __repr__(self):
        return "<LadderTask %s (%s)>" % (self.id, self.rung)


def resolve_waypoint(task, start_xy):
    """``(x, y)`` of the commanded point for one run, in world metres.

    ``start_relative`` resolves the goal against the run's own first recorded
    sample, which is what the one sentence means by *"where it starts"*.
    ``absolute`` takes the number as written. Anything else raises, because a
    goal the grader guessed at is worse than no cell.
    """
    spec = task.waypoint_spec
    frame = spec.get("frame", "absolute")
    if frame == "start_relative":
        off = spec.get("offset_m") or [0.0, 0.0]
        return (float(start_xy[0]) + float(off[0]),
                float(start_xy[1]) + float(off[1]))
    if frame == "absolute":
        xy = spec.get("xy_m") or [0.0, 0.0]
        return (float(xy[0]), float(xy[1]))
    raise ValueError("task %s declares an unknown waypoint frame %r"
                     % (task.id, frame))


def waypoint_citation(task):
    spec = task.waypoint_spec
    return ("%s -> waypoint (%s); %s"
            % (task.citation(), spec.get("frame", "absolute"),
               spec.get("resolution_rule", "no resolution rule stated")))


def roles_citation(task):
    spec = task.roles
    return ("%s -> roles (%s); %s"
            % (task.citation(),
               ", ".join("%s=%s" % (k, v) for k, v in sorted(spec.items())
                         if k != "resolution_rule") or "none declared",
               spec.get("resolution_rule", "no resolution rule stated")))


#: Task directories searched in addition to this one. LOOPBENCH's rungs are a
#: DIFFERENT benchmark asking a different question, but they are run by the
#: same cell machinery -- staged workspace, one headless session, deliverable
#: discovery, a cold standalone re-run -- and duplicating that machinery to
#: keep the directories apart would mean two copies of the answer-key
#: quarantine and the launch-race retry. Each task names its own grader, so
#: nothing about a rung's SCORING is shared; only the plumbing is.
EXTRA_TASK_ROOTS = (HERE.parents[1] / "loopbench" / "tasks",)


def discover():
    out = {}
    for root in (HERE,) + tuple(EXTRA_TASK_ROOTS):
        if not root.is_dir():
            continue
        for d in sorted(root.iterdir()):
            if d.is_dir() and (d / "meta.json").exists():
                t = Task(d)
                out[t.id] = t
    return out


def get(task_id):
    tasks = discover()
    if task_id in tasks:
        return tasks[task_id]
    short = {tid.split("_")[0]: t for tid, t in tasks.items()}
    if task_id in short:
        return short[task_id]
    raise KeyError("unknown ladder task %r; known: %s"
                   % (task_id, ", ".join(sorted(tasks))))


__all__ = ["Task", "discover", "get", "resolve_waypoint", "waypoint_citation",
           "roles_citation"]
