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

"""Oracle for A1: the known-good solution, written the intended way.

Its only job is to prove A1 is PASSABLE and that the grader says so. It does
what a competent agent would:

  * authors ONE world with ten ``URDFRobot`` blocks pointing at
    ``husky.urdf`` -- no file in the tree is the answer (the shipped swarm
    world has 8, the combat one has 10 but drives them in straight lines);
  * reuses ``husky_random`` and sets ``supervisor TRUE``, which is what turns
    on the controller's position-based stuck detection;
  * sets ``newtonSolver "mujoco"`` -- the documented fix for XPBD's inability
    to bound lateral wheel slip on a skid-steer pivot. Without it the wheels
    jam and ten Huskies sit still;
  * spawns them on a ring, well clear of each other and facing outward, and
    then verifies its own work by reading the poses back.

**Measured caveat, reported rather than hidden.** The bearing spread that
satisfies A1.8 comes mostly from the spawn headings, not from the controller.
With all ten spawned on heading 0 in a grid, ``husky_random`` over 30 s
produced R = 0.953 -- a FAIL -- because the random walk's yaw authority moves
the net-displacement bearing by only ~+-37 deg in that window. This is direct
evidence for SPEC 9 open question 2.
"""

from __future__ import annotations

import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from agentbench.agents.base import AgentResult  # noqa: E402
from agentbench.common.paths import HUSKY_URDF, as_wbt_path  # noqa: E402

N = 10
RING_RADIUS_M = 12.0
ARENA_M = 60.0
SEED = 20260726

HEADER = """#VRML_SIM R2025a utf8

# Ten Huskies on a random walk. Built for the AgentBench A1 prompt:
# "Build me a scene with 10 Huskies in it and let them move randomly, and
#  show me proof they actually moved."

EXTERNPROTO "omnisim://projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "omnisim://projects/appearances/protos/Asphalt.proto"
EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"

WorldInfo {
  basicTimeStep 32
  title "Ten Huskies, random walk"
  # A skid-steer pivot scrubs all four wheels sideways at once. XPBD's soft
  # positional contact has no bounded lateral slip, so the wheels jam and the
  # Huskies never leave their spawn. SolverMuJoCo slips correctly.
  newtonSolver "mujoco"
  # Arena walls have to be Newton collision surfaces or the Huskies drive out
  # of the world.
  newtonStatics TRUE
  # Even sub-step count: the CUDA-graph replay is disabled on an odd one.
  newtonSubsteps 6
}
Viewpoint {
  orientation 0.000000 1.000000 0.000000 1.570796
  position 0.000 0.000 90.000
}
OmniSimSky { }
DEF SUN OmniSimSun { }
DEF SUN_MARKER OmniSimSunMarker { }

RectangleArena {
  floorSize %(arena)g %(arena)g
  wallHeight 0.6
  floorAppearance Asphalt {
    colorOverride 0.55 0.57 0.60
  }
}
"""

ROBOT = """
URDFRobot {
  url "%(url)s"
  translation %(x).4f %(y).4f 0.2
  rotation 0 0 1 %(yaw).5f
  name "husky_%(i)d"
  controller "husky_random"
  supervisor TRUE
}
"""


def build_world(path, url=None, n=N, radius=RING_RADIUS_M, seed=SEED,
                jitter=0.6, controller="husky_random", supervisor=True,
                uniform_heading=False):
    """Write the world. Kept parameterised so the null variants can reuse the
    same emitter and differ only in the one thing being tested."""
    url = url or as_wbt_path(HUSKY_URDF)
    rng = random.Random(seed)
    parts = [HEADER % {"arena": ARENA_M}]
    for i in range(n):
        a = 2.0 * math.pi * i / n
        yaw = 0.0 if uniform_heading else a + rng.uniform(-jitter, jitter)
        block = ROBOT % {"url": url, "x": radius * math.cos(a),
                         "y": radius * math.sin(a), "yaw": yaw, "i": i}
        if controller != "husky_random":
            block = block.replace('controller "husky_random"',
                                  'controller "%s"' % controller)
        if not supervisor:
            block = block.replace("  supervisor TRUE\n", "")
        parts.append(block)
    Path(path).write_text("".join(parts), encoding="utf-8")
    return Path(path)


def run(ctx):
    res = AgentResult()
    world = ctx.scratch_dir / "husky_swarm_10.wbt"

    ctx.trace.turn("Ten Huskies that move randomly. I will author one world "
                   "with ten URDFRobot blocks on husky.urdf, drive them with "
                   "the shipped husky_random controller, pin the skid-steer "
                   "solver, then run it and read the poses back as proof.")
    ctx.trace.tool("write_file", {"path": str(world),
                                  "n_robots": N,
                                  "controller": "husky_random",
                                  "newtonSolver": "mujoco"})
    build_world(world)
    res.artifacts["world"] = str(world)

    # Self-verification: the prompt asks for proof, so the agent measures its
    # own result before answering. The grader measures independently either
    # way -- this only sets `self_verified`.
    ctx.trace.tool("run_headless", {"world": str(world), "duration_s": 12.0,
                                    "why": "read the poses back as proof"})
    proof = _self_check(ctx, world)
    ctx.trace.event("self_check", **proof)
    res.self_verified = bool(proof.get("ok"))

    msg = ("Built %s: ten Huskies (husky.urdf) on a 12 m ring in a 60x60 m "
           "arena, each running the husky_random walk with supervisor TRUE, "
           "under newtonSolver \"mujoco\" so the skid-steer wheels do not "
           "jam. Proof: %s" % (world.name, proof.get("summary", "not run")))
    ctx.trace.final(msg, res.self_verified)
    res.final_message = msg
    res.turns = ctx.trace.turns
    res.tool_calls = ctx.trace.tool_calls
    return res


def _self_check(ctx, world, duration=12.0):
    """The agent's OWN measurement -- deliberately shorter and cheaper than
    the grader's, and read through the same public path an agent has."""
    if getattr(ctx, "fake_sim", False):
        return {"ok": False, "summary": "skipped (--fake-sim)"}
    from agentbench.adapters.omnisim import headless
    try:
        r = headless.run_standalone(
            world, ctx.run_dir / "agent_self_check", duration=duration,
            settle=1.0, phase_a=False, tag="selfcheck", timeout_s=300)
        if r.xyz is None:
            return {"ok": False, "summary": "self-check produced no samples"}
        import numpy as np
        d = [float(np.hypot(*(r.xy(i)[-1] - r.xy(i)[0])))
             for i in range(r.n_robots)]
        return {"ok": bool(d) and min(d) > 1.0,
                "n_robots": r.n_robots,
                "min_net_displacement_m": round(min(d), 3) if d else None,
                "summary": ("all %d Huskies moved, minimum net displacement "
                            "%.2f m over %.0f s"
                            % (r.n_robots, min(d), duration)) if d else "none"}
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "summary": "self-check failed: %r" % (exc,)}
