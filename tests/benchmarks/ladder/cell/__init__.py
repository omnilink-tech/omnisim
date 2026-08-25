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

"""**The agent-cell runner for the capability ladder.**

Everything the ladder has proven so far came from SCRIPTED oracles -- a human
wrote them knowing the thresholds, and ``capability-ladder-plan.md`` §2 is
explicit that a scripted control is *not* a cell. A **cell** is one autonomous
agent, given one sentence and the tier's ``container/``, with no help. This
package is that path::

    stage_ladder_workspace.py   one (tier, column) workspace, answer key OUT
    run_ladder_cell.py          one graded cell: stage -> session -> phase B
                                -> the REAL tier grader -> a §3 row -> publish
    run_ladder_campaign.py      the lane driver: (tier, column) groups, n=3,
                                resume, locks, deferral, per-group publish
    test_ladder_cell.py         the unit tests (no Claude, no engine, no GPU)

**Reused by import, never forked** (the ladder's own rule, and
``capability-ladder-plan.md`` §4.1's "staging reuses ``cc_lane`` unchanged"):

=================================================  ==========================
reused                                              from
=================================================  ==========================
workspace templates, junctions, link-safe           ``agentbench.cc_lane.
teardown, the redaction pass, the manifest,         stage_workspaces``
the pending-delete / process / port / repo-
artifact sweeps
the headless ``claude -p`` child, the transcript    ``agentbench.cc_lane.
tool-call counter, env scrubbing, the artifact      run_cc_cell``
discovery walk, ``metrics_source`` honesty
the engine semaphore, the same-task lock, the       ``agentbench.cc_lane.
resource guard, rate-limit recognition              concurrency``
"no row, no result" publication                     ``agentbench.
                                                    run_agentbench.
                                                    publish_run``
the tier graders, phase B, the task registry        ``ladder.graders.t1..t4``,
                                                    ``ladder.tasks``
=================================================  ==========================

Nothing in this package writes to ``agentbench/**`` or to the ladder's
graders, fixtures or task files. The coupling is one direction.

The three honesty rules this package is built under, restated because they
decide code paths rather than prose:

1. **No metric is ever fabricated.** Unmeasured is ``null`` with the reason
   recorded -- never ``0.0``, never the value that was asked for.
2. **A column that cannot answer a channel yields**
   ``blocker=scaffolding_defect_ours``, never a simulator failure
   (``capability-ladder-plan.md`` §4's binding rule).
3. **A cell whose workspace is shown to have contained answer-key material is
   INVALID** -- not failed, not quietly graded. The check runs before the
   session (on the staged tree) and after it (on the session transcript), and
   both readings ride in every row.
"""

from __future__ import annotations

__all__ = []
