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

"""**The capability ladder** -- scaffolding for the E2 reachability programme.

The contract this package implements is
``docs/developer/capability-ladder-plan.md``. Read it first: the tier
statements, the thresholds, the cell taxonomy (its 3) and the three binding
traps (its 5) are that file's, not this one's, and **nothing here is a
result**.

Why a separate package instead of new tasks inside ``agentbench``
----------------------------------------------------------------

``tests/benchmarks/agentbench/`` carries a **frozen, published task set** and
seventy published rows measured against it (Phase W, ``124ae7f3``). Adding a
tier to it would silently change what a re-run of that campaign measures. So
the ladder gets its own namespace and **imports agentbench read-only**:

===================================  =========================================
reused, unmodified, by import        why
===================================  =========================================
``agentbench.graders.evidence``      the neutral bundle IS the cross-sim
                                     contract; forking it would fork the
                                     contract
``agentbench.graders.verdict``       assertions, falsifiers, the vacuity
                                     detector, the progress ordinal
``agentbench.graders.physical``      path length, net displacement, z bands
``agentbench.adapters``              ``resolve()`` and the two columns that
                                     already produce evidence (omnisim,
                                     webots)
``agentbench.common.paths``          the engine launcher bridge
===================================  =========================================

Nothing in this package writes to ``agentbench/**``. The coupling is one
direction and it is listed above so a change on that side has a visible blast
radius.

What the ladder adds in its own namespace
-----------------------------------------

Two channels the frozen bundle has no field for, both recorded in
:mod:`ladder.graders.ladder_evidence` rather than by editing
``agentbench/graders/evidence.py``:

``Waypoint``
    the commanded point on the ground plane. It is **task data**, not
    simulator evidence -- the grader owns it, the agent is told it in one
    sentence, and no adapter may supply it.

``SupportContactObservation``
    contacts naming a robot body and the **non-robot surface it is riding
    on**, sampled *during the motion window*. The frozen bundle's
    ``ContactObservation`` can carry any pair, but the only populated channel
    on either shipped column is the robot-robot filter over the first N steps
    at t=0 -- which cannot answer T1.4. See that module's docstring for the
    gap in full.
"""

from __future__ import annotations

__all__ = []
