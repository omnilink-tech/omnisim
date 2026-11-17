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

"""Oracle for B3: measure the scene, then report what you measured.

The intended solution is *not* to read the numbers out of the world file --
it is to ask the running scene for real geometry. This oracle does what an
agent with the harness tool surface would: load the world, read the bounds,
compute the two answers, and state them.

It reads the geometry through the same supervisor-side helper the harness's
``/scene/tree?bounds=1`` uses, so the oracle and a real agent see the same
numbers.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench.agents.base import AgentResult  # noqa: E402
from agentbench.common import worldtext  # noqa: E402


def run(ctx):
    res = AgentResult()
    world = worldtext.pick_artifact(ctx.scratch_dir)
    if world is None:
        res.final_message = "no world found in the scratch directory"
        ctx.trace.final(res.final_message, False)
        return res
    res.artifacts["world"] = str(world)

    ctx.trace.turn("I will not guess from the file. I will load the world and "
                   "read each Husky's real world-space bounds, then report "
                   "the centre-to-centre distance and which one is taller.")
    ctx.trace.tool("measure_scene", {"world": str(world),
                                     "what": "positions + world AABBs"})
    if getattr(ctx, "fake_sim", False):
        res.final_message = "skipped (--fake-sim): nothing was measured"
        ctx.trace.final(res.final_message, False)
        return res

    from agentbench.adapters.omnisim import headless
    r = headless.run_standalone(world, ctx.run_dir / "agent_measure",
                                duration=1.0, settle=0.0, phase_a=True,
                                contact_steps=1, tag="agentmeasure",
                                timeout_s=300)
    entries = {e.get("name"): e for e in
               ((r.phase_a or {}).get("t0_robots", []) or [])}
    ctx.trace.event("measurement", robots=list(entries))
    if len(entries) < 2:
        res.final_message = ("could not measure the scene: %s"
                             % (r.error or "fewer than two robots found"))
        ctx.trace.final(res.final_message, False)
        return res

    (na, a), (nb, b) = list(entries.items())[:2]
    dist = math.dist(a["position"], b["position"])
    tops = {n: float((e.get("bounds") or {}).get("bbox_max", [0, 0, 0])[2])
            for n, e in ((na, a), (nb, b))}
    taller = max(tops, key=tops.get)

    res.self_verified = True
    msg = ("Measured from the loaded scene, not from the file. The two "
           "Huskies are %.3f m apart centre to centre (%s at %s, %s at %s). "
           "%s is taller: the top of its bounding box is at z = %.3f m "
           "versus %.3f m for %s."
           % (dist, na, _fmt(a["position"]), nb, _fmt(b["position"]),
              taller, tops[taller], min(tops.values()),
              min(tops, key=tops.get)))
    ctx.trace.final(msg, True)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res


def _fmt(p):
    return "(%.2f, %.2f, %.2f)" % tuple(p)
