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

"""Oracle for B1 ``overlap_audit``: measure the scene, answer honestly.

The intended solution is *not* to read spawn translations out of the world
file -- it is to load the world and compare every robot's real world-space
bounds at a frozen t=0, exactly what an agent with the harness surface would
do with ``GET /scene/tree?bounds=1``. This oracle reads the same numbers
through the grader-owned recorder's t=0 scan (the same
``geometry.bounds_for_subtree`` helper the harness serves), computes every
pairwise AABB clearance itself, and commits to ONE verdict and -- when the
verdict is yes -- ONE pair.

Registered by the orchestrator in ``agents/__init__.py``: the b1_fixtures
module ships the negatives only and records that the oracle is the
orchestrator's job (SPEC 7.1's oracle-PASSes-every-task gate).
"""

from __future__ import annotations

import sys
from itertools import combinations
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench.agents.base import AgentResult  # noqa: E402
from agentbench.common import worldtext  # noqa: E402
from agentbench.graders import physical as ph  # noqa: E402
from agentbench.graders.b1_core import OVERLAP_MIN_M  # noqa: E402


def run(ctx):
    res = AgentResult()
    world = worldtext.pick_artifact(ctx.scratch_dir)
    if world is None:
        res.final_message = "no world found in the scratch directory"
        ctx.trace.final(res.final_message, False)
        return res
    res.artifacts["world"] = str(world)

    ctx.trace.turn("I will not eyeball this. I will load the world, freeze "
                   "t=0, read every robot's world-space bounding box, and "
                   "compare every pair's axis clearances in metres.")
    ctx.trace.tool("measure_scene", {"world": str(world),
                                     "what": "world-space AABB per robot"})
    if getattr(ctx, "fake_sim", False):
        res.final_message = "skipped (--fake-sim): nothing was measured"
        ctx.trace.final(res.final_message, False)
        return res

    from agentbench.adapters.omnisim import headless
    r = headless.run_standalone(world, ctx.run_dir / "agent_measure",
                                duration=1.0, settle=0.0, phase_a=True,
                                contact_steps=1, tag="agentmeasure",
                                timeout_s=300)
    entries = [(e.get("name"), e) for e in
               ((r.phase_a or {}).get("t0_robots", []) or [])
               if (e.get("bounds") or {}).get("bbox_min")]
    ctx.trace.event("measurement", robots=[n for n, _ in entries])
    if len(entries) < 2:
        res.final_message = ("could not measure the scene: %s"
                             % (r.error or "fewer than two measured robots"))
        ctx.trace.final(res.final_message, False)
        return res

    clearances = {}
    for (na, a), (nb, b) in combinations(entries, 2):
        box_a = (tuple(a["bounds"]["bbox_min"]), tuple(a["bounds"]["bbox_max"]))
        box_b = (tuple(b["bounds"]["bbox_min"]), tuple(b["bounds"]["bbox_max"]))
        clearances[tuple(sorted((na, nb)))] = ph.aabb_clearance(box_a, box_b)

    overlapping = sorted((pair, c) for pair, c in clearances.items()
                         if c <= -OVERLAP_MIN_M)
    res.self_verified = True
    if overlapping:
        (pa, pb), depth = overlapping[0]
        msg = ("Yes. Measured from the loaded scene at a frozen t=0, %s and "
               "%s are overlapping: their world-space bounding boxes "
               "interpenetrate by %.3f m on the shallowest axis. Every other "
               "pair is measurably clear (best-axis separation %.3f m or "
               "more)."
               % (pa, pb, -depth,
                  min((c for pair, c in clearances.items()
                       if (pa, pb) != pair), default=float("inf"))))
    else:
        best = min(clearances.values())
        msg = ("No. Measured from the loaded scene at a frozen t=0, no two "
               "robots' world-space bounding boxes interpenetrate; the "
               "closest pair is still %.3f m apart on its best axis." % best)
    ctx.trace.final(msg, True)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res
