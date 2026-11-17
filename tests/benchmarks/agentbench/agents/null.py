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

"""The negative controls. This module is the load-bearing part of Phase 0.

Two families, and they answer different questions:

``null``
    Does nothing at all except declare success. It exists to enforce the
    invariant inherited from ``omnilink_tasks``: **no task may be passable by
    doing nothing** (SPEC 1.1, 7.1). If any task PASSes here, that task is
    broken, not the agent.

``wrong`` / ``parade``
    Do something *plausible* and wrong, aimed at exactly one assertion each.
    They are how we find out whether a grader measures what it claims:

      A1 ``parade``  ten Huskies, correctly authored, correctly solved,
                     genuinely moving -- all driving the same way. Must fail
                     A1.8 (circular resultant R < 0.70) and NOTHING else. This
                     is the single most informative check in Phase 0: it is
                     the "copy the 10-Husky combat world" failure the SPEC
                     names, and if it fails on some other assertion instead
                     then A1.8 is not measuring randomness.
      A1 ``wrong``   ten Huskies, correctly placed, with no controller. Must
                     fail A1.4/A1.5/A1.6 -- present but inert.
      B3 ``wrong``   a confident, well-formatted, WRONG answer -- the failure
                     ``omnilink_tasks`` was built around. Must fail B3.2/B3.4
                     on the numbers, not on the formatting.
      C2 ``wrong``   "fixes" the fall by deleting the robot's Physics node.
                     It then stays up forever, which is the whole point: must
                     fail C2.3/C2.4 and pass the naive z test.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench.agents import oracle_a1  # noqa: E402
from agentbench.agents.base import AgentResult  # noqa: E402
from agentbench.common import worldtext  # noqa: E402


# --- the do-nothing agent ---------------------------------------------------

def run(ctx):
    """Touches nothing, claims success. Must never PASS anything."""
    res = AgentResult()
    ctx.trace.turn("(null agent: performs no action)")
    msg = "Done -- everything is set up and working as requested."
    ctx.trace.final(msg, False)
    res.final_message = msg
    res.turns = ctx.trace.turns
    return res


# --- A1: plausible but wrong ------------------------------------------------

def run_a1_parade(ctx):
    """Ten real, moving Huskies -- all driving the same direction.

    Targets A1.8 alone. Everything else about this world is correct.
    """
    res = AgentResult()
    world = ctx.scratch_dir / "husky_swarm_10.wbt"
    ctx.trace.turn("Ten Huskies driving forward. I will reuse the existing "
                   "swarm pattern and the drive_forward controller.")
    ctx.trace.tool("write_file", {"path": str(world), "n_robots": 10,
                                  "controller": "drive_forward",
                                  "headings": "all identical"})
    oracle_a1.build_world(world, controller="drive_forward",
                          supervisor=False, uniform_heading=True)
    res.artifacts["world"] = str(world)
    msg = ("Built %s with ten Huskies all running drive_forward. They move."
           % world.name)
    ctx.trace.final(msg, False)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res


def run_a1_static(ctx):
    """Ten correctly placed Huskies that never move (no controller)."""
    res = AgentResult()
    world = ctx.scratch_dir / "husky_swarm_10.wbt"
    ctx.trace.turn("Ten Huskies placed in the scene.")
    ctx.trace.tool("write_file", {"path": str(world), "n_robots": 10,
                                  "controller": "<none>"})
    oracle_a1.build_world(world, controller="<none>", supervisor=False)
    res.artifacts["world"] = str(world)
    msg = "Built %s: ten Huskies, placed and ready." % world.name
    ctx.trace.final(msg, False)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res


# --- B3: a confident wrong answer ------------------------------------------

def run_b3_wrong(ctx):
    """Well-formatted, precise, plausible -- and never measured anything.

    This is the ``omnilink_tasks`` failure exactly: an agent reporting
    completed work with confident coordinates while nothing was read.
    """
    res = AgentResult()
    ctx.trace.turn("Both Huskies look like they are about four metres apart.")
    msg = ("The two Huskies are 4.20 m apart centre to centre. husky_ground "
           "is the taller of the two, by about 5 cm.")
    ctx.trace.final(msg, True)   # it CLAIMS to have verified
    res.final_message = msg
    res.self_verified = False
    res.turns = ctx.trace.turns
    return res


# --- C2: the freeze-it cheat ------------------------------------------------

_PHYSICS = re.compile(r"(?ms)^  physics Physics \{.*?^  \}\n")


def run_c2_freeze(ctx):
    """Delete the robot's Physics node. It "stays up" -- forever."""
    res = AgentResult()
    world = worldtext.pick_artifact(ctx.scratch_dir)
    if world is None:
        res.final_message = "no world found"
        ctx.trace.final(res.final_message, False)
        return res
    res.artifacts["world"] = str(world)
    text = world.read_text(encoding="utf-8")
    fixed, n = _PHYSICS.subn("", text)
    ctx.trace.turn("The robot is falling, so I will stop it falling.")
    ctx.trace.tool("edit_file", {"path": str(world),
                                 "edit": "remove the Physics node from the "
                                         "robot", "applied": bool(n)})
    world.write_text(fixed, encoding="utf-8")
    msg = ("Fixed. The robot no longer falls through the floor -- it stays "
           "exactly where it is spawned for the whole run.")
    ctx.trace.final(msg, False)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res
