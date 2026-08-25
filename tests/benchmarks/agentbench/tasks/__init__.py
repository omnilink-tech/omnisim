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

"""The task registry.

One directory per task holding ``prompt.txt`` (the exact one sentence),
``meta.json`` (tier, grader entry point, par time, phase config, the
thresholds the grader must use) and ``initial/`` (the starting files).

``prompt.txt`` is byte-identical across simulators except for a path -- there
is no per-simulator prompt tuning, ever (SPEC 4.4).
"""

from __future__ import annotations

import importlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent

# @HUSKY_URDF@ is the only substitution an initial/ file may carry: URDFRobot
# urls resolve relative to the world file, so an initial world copied into a
# per-run scratch dir cannot hold a relative path back into the repo.
SUBSTITUTION_TOKEN = "@HUSKY_URDF@"

#: The hard wall-clock ceiling on ONE cell, in seconds (SPEC 2.4).
#:
#: A cell that reaches it is killed and scored ``FAIL`` with
#: ``budget_exhausted`` -- running out of budget is a result, never
#: ``INVALID``. The ceiling exists so campaign cost is bounded a priori:
#: ``cells x TASK_HARD_CEILING_S`` is the worst case the campaign can bill,
#: whatever the agent does. No ``meta.json`` can buy itself a longer run --
#: :attr:`Task.timeout_s` clamps every declared value to it.
TASK_HARD_CEILING_S = 2700      # 45 minutes
#
# HISTORY -- append, never rewrite. The value is policy and it has moved
# twice; a reader comparing two campaigns needs to know which ceiling each
# ran under, because this constant is GLOBAL and a cell scored under one is
# not comparable with a cell scored under another (see NOT COMPARABLE below).
#
# 900 s (freeze v3, 2026-08-09) -- the original cost cap.
#
# 900 -> 1800 s (2026-08-09, owner's call) on measured evidence: EVERY
#   robotics-tier cell exhausted the 15-minute budget (R1: 2 of 2 on
#   omnisim, both with budget_exhausted=True and the goal unreached because
#   the agent was still building). A task that asks for an arena, a robot, a
#   compliant LiDAR AND a working controller from an empty project does not
#   fit in a quarter hour, so at 900 s the suite was measuring the CAP rather
#   than the simulator -- and a cap that truncates every cell on every arm
#   cannot discriminate between arms, which is the same disease as an
#   intersection-only task set.
#
# 1800 -> 2700 s (2026-08-10, owner's call) -- COST CONTROL, and it is a
#   trade, not a loosening. The protocol changed in the same decision: the
#   suite no longer runs n repeats per (task, arm), it runs ONE cell under a
#   wall-clock ceiling ("we are not doing n times anymore... we set a maximum
#   time that the agent should finish before... the maximum would be 45
#   minutes"). Repeats were the dominant token cost -- Lane B was n = 5 and
#   A1 was n = 10 -- so dropping to a single run buys back far more budget
#   than a 900 s ceiling rise spends: A1's worst case goes from
#   10 x 1800 = 18000 s of agent time per (task, arm) to 1 x 2160 = 2160 s.
#   A cell finishing well inside the ceiling is NOT a problem; the ceiling is
#   a cap on the worst case, not a target.
#
# ⚠ NOT COMPARABLE ACROSS CEILINGS. This constant applies to every task at
# once, so a cell scored at 900 s, one scored at 1800 s and one scored at
# 2700 s are three different measurements of three different experiments.
# They may never be pooled, averaged, or placed in the same column. The
# per-task effective budget is on every row (``timeout_s``) and the protocol
# a row ran under is on the row too (``protocol`` -- ``cc_lane/run_cc_cell``
# writes it); a row with NO ``protocol`` block predates the single-run
# protocol and must not be pooled with one that has it.
#
# ⚠ ONE RUN IS NOT A RATE. With a single cell per (task, arm) there is no
# variance estimate at all: ``pass@1``, a pass fraction and a confidence
# interval are undefined, not merely wide. A single observation is an
# OUTCOME. See SPEC 3.5.


class Task:
    def __init__(self, path):
        self.dir = Path(path)
        self.meta = json.loads((self.dir / "meta.json").read_text("utf-8"))
        self.id = self.meta["id"]
        self.prompt = (self.dir / self.meta.get("prompt_file", "prompt.txt")
                       ).read_text("utf-8").strip()

    @property
    def tier(self):
        return self.meta.get("tier")

    @property
    def par_s(self):
        return self.meta.get("par_s")

    @property
    def timeout_s(self):
        """Effective budget: ``min(declared or 3 x par, TASK_HARD_CEILING_S)``.

        The declared value is honoured when it is under the ceiling; the
        ceiling always wins above it. Runners must gate the child process on
        THIS number, not on a global default -- see SPEC 2.4.
        """
        declared = self.meta.get("timeout_s")
        if declared is None:
            declared = 3 * (self.meta.get("par_s") or 0)
        return min(declared, TASK_HARD_CEILING_S)

    @property
    def declared_timeout_s(self):
        """What ``meta.json`` asked for, before the ceiling. Audit only."""
        return self.meta.get("timeout_s",
                             3 * (self.meta.get("par_s") or 0))

    @property
    def initial_dir(self):
        return self.dir / "initial"

    @property
    def standalone(self):
        return (self.meta.get("phases", {}) or {}).get("standalone", {}) or {}

    @property
    def wants_harness(self):
        return bool((self.meta.get("phases", {}) or {})
                    .get("harness", {}).get("enabled"))

    def grader(self):
        return importlib.import_module(self.meta["grader"])

    def __repr__(self):
        return "<Task %s (%s)>" % (self.id, self.tier)


def discover():
    out = {}
    for d in sorted(HERE.iterdir()):
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
    raise KeyError("unknown task %r; known: %s"
                   % (task_id, ", ".join(sorted(tasks))))
