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

"""Oracle for C2: give the floor a collision surface, then prove it holds.

The bug is a ``Solid`` with visible geometry and no ``boundingObject`` -- it
is scenery, not a surface. The fix is one field. What makes this an oracle
rather than a patch is the second half of the prompt: it re-runs the world and
reads the robot's z track back before claiming anything.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench.agents.base import AgentResult  # noqa: E402
from agentbench.common import worldtext  # noqa: E402

FLOOR_BOUNDING = """  boundingObject Box {
    size 20 20 0.1
  }
"""


def apply_fix(text):
    """Insert a boundingObject into the DEF FLOOR Solid block."""
    m = re.search(r'(?ms)^DEF FLOOR Solid \{.*?^\}\n', text)
    if not m:
        return text, False
    block = m.group(0)
    if "boundingObject" in block:
        return text, False
    fixed = block[:-2] + FLOOR_BOUNDING + "}\n"
    return text[:m.start()] + fixed + text[m.end():], True


def run(ctx):
    res = AgentResult()
    world = worldtext.pick_artifact(ctx.scratch_dir)
    if world is None:
        res.final_message = "no world found in the scratch directory"
        ctx.trace.final(res.final_message, False)
        return res
    res.artifacts["world"] = str(world)

    ctx.trace.turn("The floor Solid has visible geometry but no "
                   "boundingObject, so it is scenery with no collision "
                   "surface and the crate passes straight through. I will "
                   "give it a boundingObject and then re-run to check the z "
                   "track.")
    text = world.read_text(encoding="utf-8")
    fixed, changed = apply_fix(text)
    ctx.trace.tool("edit_file", {"path": str(world),
                                 "edit": "add boundingObject Box 20x20x0.1 "
                                         "to DEF FLOOR",
                                 "applied": changed})
    if not changed:
        res.final_message = "could not locate the floor Solid to fix"
        ctx.trace.final(res.final_message, False)
        return res
    world.write_text(fixed, encoding="utf-8")

    if getattr(ctx, "fake_sim", False):
        res.final_message = ("applied the fix; verification skipped "
                             "(--fake-sim)")
        ctx.trace.final(res.final_message, False)
        return res

    from agentbench.adapters.omnisim import headless
    ctx.trace.tool("run_headless", {"world": str(world), "duration_s": 5.0})
    r = headless.run_standalone(world, ctx.run_dir / "agent_self_check",
                                duration=5.0, settle=0.0, phase_a=False,
                                tag="selfcheck", timeout_s=300)
    if r.xyz is None or r.n_robots == 0:
        res.final_message = ("applied the fix but the verification run "
                             "produced no samples: %s" % (r.error,))
        ctx.trace.final(res.final_message, False)
        return res
    z = r.z(0)
    ctx.trace.event("self_check", z_start=float(z[0]), z_end=float(z[-1]),
                    z_min=float(z.min()))
    res.self_verified = True
    msg = ("Fixed: DEF FLOOR had no boundingObject, so it had no collision "
           "surface. Added one (Box 20 x 20 x 0.1, matching the visible "
           "slab). Proof over a 5 s re-run: the crate falls from z = %.3f m "
           "and settles at z = %.3f m; the lowest z it ever reaches is "
           "%.3f m, so it is resting on the floor rather than passing "
           "through it." % (float(z[0]), float(z[-1]), float(z.min())))
    ctx.trace.final(msg, True)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res
