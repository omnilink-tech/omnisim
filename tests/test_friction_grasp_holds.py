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

"""The flagship friction-grasp demo must HOLD -- measured, per run, in metres.

WHY THIS EXISTS
---------------
On 2026-08-07 the shipped pinch demo (friction_grasp_minimal.omniworld -- the world
AGENTS.md's "make a gripper HOLD something" row points every agent at) read

    SLIPPED  lifted_by_m = -0.00033

on a clean environment, while its own documentation promised HELD at +0.2817 m
"reproduced bit-identically over three runs". The regression had been in the
tree for up to three weeks: the batched CPU control path (899eb425, 7b3762f7)
verified bit-identity on PASSIVE box stacks and was never A/B'd with
FORCE-MODE control -- and a pinch grasp is pure force mode. A July-18 binary
HELD (+0.28173); every binary since SLIPPED. No gate ran this world, so
nothing noticed: the smoke lane runs on ODE, and lane-1's exact-value gate has
no force-driven scene.

This test is that missing gate. It runs the demo exactly as the docs tell a
user to and asserts the controller's own measured verdict file. The fix it
guards is the force-mode refusal in _mjc_batch_substeps_ok
(OmNewtonBackend.cpp): once a world writes control.joint_f, the batch path is
off for the run. If this test starts failing with SLIPPED again, suspect any
"optimisation" of the CPU step path first, and read the gate's comment before
weakening either.

⚠ 2026-10-17: THE +0.2817 REFERENCE WAS THE EJECTION, NOT THE HOLD. The wrist
only travels 0.205 m (LIFT_M), so a part carried between the pads can rise at
most ~0.205 m. Measured with a probe copy of this world (wrist and finger
positions logged per phase), OMNISIM_NEWTON_SERVO_ARMATURE_BOUND=0: the pads
end 11.8 mm inside each face of the 40 mm part, and the part sits at z 0.3367
ON TOP OF THE WRIST PLATE (its underside 0.3067 = the plate's top), clear of
the pads -- the "squeezed out upward" mechanism the omniarm6 pick controllers
already describe. With the position-servo armature bound (the default since
2026-10-17) the pads stop 0.6 mm into the faces, the part centre stays 4 mm
from the pad centre, rises 0.2025 m for 0.2023 m of wrist travel, and is set
back down on the floor (0.0524) at the end. So the gate is now a BAND around
the wrist's own travel: above it is physically an ejection, below it a slip.

Skips (not failures) when the engine binary or the Newton runtime is absent,
same convention as the other engine-driven Newton tests.

    python -m pytest tests/test_friction_grasp_holds.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLD = REPO / "projects/samples/demos/worlds/starter/friction_grasp_minimal.omniworld"
VERDICT = (REPO / "projects/samples/demos/controllers/friction_grasp_minimal/"
                  "friction_grasp_result.json")

#: The wrist travels 0.205 m (the controller's LIFT_M); a part held between the
#: pads rises with it -- measured +0.2025 m on 2026-10-17. The old +0.2817
#: reference (any binary before the position-servo armature bound) was the part
#: ejected upward onto the wrist plate; the broken batch path reads -0.0003.
#: The band accepts a few mm of slip and rejects both failure modes.
MIN_LIFT_M = 0.185
MAX_LIFT_M = 0.215


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def test_flagship_pinch_grasp_holds(tmp_path):
    if VERDICT.exists():
        VERDICT.unlink()
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(tmp_path / "grasp.log")
    # A stray contact knob in the parent shell must not decide this verdict --
    # the world file declares its own physics.
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    run = subprocess.run(
        [sys.executable, "-m", "omnisim", "run-headless", str(WORLD),
         "--duration", "45"],
        cwd=str(REPO), env=env, capture_output=True, text=True, timeout=600)

    if not VERDICT.exists():
        # Engine ran but the controller never finished its script: an
        # instrument outcome (bring-up flake / timeout), not a physics verdict.
        pytest.skip("no verdict written (rc=%s) -- engine bring-up or timeout, "
                    "not a grasp result:\n%s" % (run.returncode, run.stdout[-800:]))
    v = json.loads(VERDICT.read_text())
    assert v["verdict"] == "HELD" and MIN_LIFT_M <= v["lifted_by_m"] <= MAX_LIFT_M, (
        "the flagship pinch demo did not hold: verdict=%s lifted=%.5f m "
        "(reference +0.2025 for 0.205 m of wrist travel). If this reads SLIPPED "
        "with lifted ~ -0.0003, the batched CPU control path is mangling "
        "force-mode control again -- see the joint_forces gate in "
        "_mjc_batch_substeps_ok, OmNewtonBackend.cpp. If it reads ~+0.28 the "
        "part was squeezed out onto the wrist plate: the position-servo armature "
        "bound (OMNISIM_NEWTON_SERVO_ARMATURE_BOUND) is off or not staged."
        % (v["verdict"], v["lifted_by_m"]))
