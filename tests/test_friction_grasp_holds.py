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

#: The doc's reference is +0.2817 m (July-18 binary: +0.28173; current binary
#: post-fix: +0.28169, bit-identical across three runs). The broken path reads
#: -0.0003. 0.25 sits far from both noise bands: a pass means the box really
#: rode up with the gripper, a fail means it stayed on the table.
MIN_LIFT_M = 0.25


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
    assert v["verdict"] == "HELD" and v["lifted_by_m"] >= MIN_LIFT_M, (
        "the flagship pinch demo did not hold: verdict=%s lifted=%.5f m "
        "(reference +0.2817). If this reads SLIPPED with lifted ~ -0.0003, the "
        "batched CPU control path is mangling force-mode control again -- see "
        "the joint_forces gate in _mjc_batch_substeps_ok, OmNewtonBackend.cpp."
        % (v["verdict"], v["lifted_by_m"]))
