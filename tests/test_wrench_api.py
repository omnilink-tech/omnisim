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

"""omnisim.wrench.apply_wrench must deliver the analytic impulse, per step.

WHY THIS EXISTS
---------------
A supervisor wrench is consumed into the solver's body-force accumulator and
cleared after the tick, so one call is a single-step impulse and anything
lasting has to be re-applied every step. `apply_wrench` hands back a handle
whose `tick()` does that re-application. This pins BOTH halves of that
contract: the physics it delivers, and the fact that it is really re-applying.

THE MEASUREMENT
---------------
Gravity 0, so the wrench is the only thing that can move either body, and the
acceleration is the applied force over the mass and nothing else:

    delta-v = F * T / m = 10 N * 0.4 s / 2 kg = 2.0 m/s

THE TEST IS RED-CAPABLE
-----------------------
A green run here has to mean something, so the probe runs a DEGENERATE second
arm beside the real one: an identical body given the same wrench with NO
duration, i.e. a single application and then the same number of idle steps.
That is exactly what the sustained arm decays into if `tick()` ever stops
re-applying, and it lands an impulse one engine step long:

    delta-v = F * dt / m = 10 N * 0.008 s / 2 kg = 0.04 m/s

so the two arms differ by the step count (50x here). If the re-application
silently died, the sustained arm would collapse onto the degenerate one and
both the ratio and the absolute assertion would fail. The count is asserted
against STEP BOUNDARIES rather than wall time, because the force lands on the
engine step after the call.

ONE STARTUP ARTEFACT THE PROBE STEPS PAST
-----------------------------------------
The engine free-runs until the controller's first step(), so a wrench applied
ahead of that lands in a window that is never integrated -- measured, it costs
exactly one of the N applications and reads as a clean 2% deficit (49 impulses
from 50 calls). It is a property of when the controller joins the run, not of
apply_wrench, so the probe takes one step before its first application and this
test then asserts the undiluted analytic value.

    python -m pytest tests/test_wrench_api.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"

MASS_KG = 2.0
FORCE_N = 10.0            # must match wrench_probe.py
DURATION_S = 0.4          # must match wrench_probe.py
BASIC_TIME_STEP_MS = 8

EXPECTED_DT_S = BASIC_TIME_STEP_MS / 1000.0
EXPECTED_STEPS = round(DURATION_S / EXPECTED_DT_S)          # 50
EXPECTED_SUSTAINED_DV = FORCE_N * DURATION_S / MASS_KG      # 2.0 m/s
EXPECTED_ONE_SHOT_DV = FORCE_N * EXPECTED_DT_S / MASS_KG    # 0.04 m/s

# Measured on CPU mj_step: 1.9999992847 vs an analytic 2.0, and 0.0399999991
# vs 0.04 -- agreement to seven significant figures, the residual being the
# float32 velocity readback. These budgets are deliberately three orders of
# magnitude tighter than "some slack": one lost application costs 2%, and a
# tolerance that absorbs that is a tolerance that hides it.
SUSTAINED_REL_TOL = 1e-3
ONE_SHOT_REL_TOL = 1e-3
RATIO_REL_TOL = 1e-3

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up")

WORLD_TEXT = """#OMNISIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_wrench_api.py -- do not commit.
# Gravity 0: the applied wrench is the only thing that can move either body.

WorldInfo {
  basicTimeStep %(dt)d
  gravity 0
  coordinateSystem "ENU"
}
Viewpoint { position -6 0 1 }
Background { skyColor [ 0.2 0.2 0.25 ] }
DEF SUBJECT_SUSTAINED Solid {
  translation 0 0 1
  name "subject_sustained"
  children [
    DEF SUBJECT_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.3 0.6 0.85 roughness 1 metalness 0 }
      geometry Box { size 0.2 0.2 0.2 }
    }
  ]
  boundingObject USE SUBJECT_SHAPE
  physics Physics { density -1 mass %(mass)g }
}
DEF SUBJECT_ONE_SHOT Solid {
  translation 3 0 1
  name "subject_one_shot"
  children [
    DEF SUBJECT_SHAPE_B Shape {
      appearance PBRAppearance { baseColor 0.85 0.5 0.2 roughness 1 metalness 0 }
      geometry Box { size 0.2 0.2 0.2 }
    }
  ]
  boundingObject USE SUBJECT_SHAPE_B
  physics Physics { density -1 mass %(mass)g }
}
DEF PROBE Robot {
  name "probe"
  controller "wrench_probe"
  supervisor TRUE
}
""" % {"dt": BASIC_TIME_STEP_MS, "mass": MASS_KG}


def _binary():
    """The engine to run. $OMNISIM_BIN first, so a source clone with no built
    engine can be tested against an installed one."""
    override = os.environ.get("OMNISIM_BIN")
    if override and Path(override).exists():
        return Path(override)
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone; set "
                                       "$OMNISIM_BIN to an installed engine")


def _run(tmp_path, attempt):
    """Run the probe world. -> probe dict, or None on a bring-up flake."""
    WORLDS.mkdir(parents=True, exist_ok=True)
    world = WORLDS / ".wrench_api.omniworld"
    world.write_text(WORLD_TEXT, encoding="utf-8")
    out = tmp_path / ("wrench_probe_%d.json" % attempt)
    log = tmp_path / ("engine_%d.log" % attempt)

    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_WRENCH_PROBE_OUT"] = str(out)
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_"):
            env.pop(k)

    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(180):
            if out.exists():
                break
            try:
                proc.wait(timeout=1)
                break
            except subprocess.TimeoutExpired:
                continue
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()

    if not out.exists():
        text = log.read_text(encoding="utf-8", errors="replace") if log.exists() else ""
        if any(m in text for m in _BRINGUP):
            return None
        pytest.fail("probe wrote no output; engine log:\n%s" % text[-4000:])
    return json.loads(out.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def probe(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("wrench")
    for attempt in range(3):
        data = _run(tmp_path, attempt)
        if data is not None:
            return data
    pytest.skip("engine did not come up in 3 attempts")


def test_timestep_is_what_the_analytic_values_assume(probe):
    assert probe["dt_s"] == pytest.approx(EXPECTED_DT_S, rel=1e-6)


def test_sustained_wrench_delivers_the_analytic_impulse(probe):
    """delta-v = F*T/m, on a free body with no gravity."""
    dv = probe["sustained_velocity"][2]
    assert dv == pytest.approx(EXPECTED_SUSTAINED_DV, rel=SUSTAINED_REL_TOL), (
        "sustained arm read %.4f m/s, analytic %.4f m/s"
        % (dv, EXPECTED_SUSTAINED_DV))


def test_reapplication_count_lands_on_step_boundaries(probe):
    """The span is measured in simulation time, so the count is the step count.

    Asserted exactly rather than with a step of slack: both dt and the duration
    are fixed constants here, so the boundary is not a tie-break and a count
    that moves is a behavioural change worth seeing.
    """
    n = probe["sustained_applications"]
    assert n == EXPECTED_STEPS, (
        "re-applied %d times over %.3f s at dt=%.4f s; expected %d"
        % (n, DURATION_S, EXPECTED_DT_S, EXPECTED_STEPS))


def test_one_shot_is_a_single_step_impulse(probe):
    """The degenerate arm: one application, then coasting."""
    assert probe["one_shot_applications"] == 1
    assert probe["one_shot_first_tick"] is False, (
        "tick() on a wrench with no duration_s must report False")
    dv = probe["one_shot_velocity"][2]
    assert dv == pytest.approx(EXPECTED_ONE_SHOT_DV, rel=ONE_SHOT_REL_TOL), (
        "one-shot arm read %.4f m/s, analytic %.4f m/s"
        % (dv, EXPECTED_ONE_SHOT_DV))


def test_sustained_and_degenerate_arms_differ_by_the_step_count(probe):
    """This is what makes a green run mean something.

    If tick() stopped re-applying, the sustained arm would BE the one-shot arm
    and this ratio would collapse to 1.
    """
    sustained = probe["sustained_velocity"][2]
    one_shot = probe["one_shot_velocity"][2]
    assert one_shot > 0.0, "degenerate arm did not move at all; nothing to compare"
    ratio = sustained / one_shot
    assert ratio == pytest.approx(EXPECTED_STEPS, rel=RATIO_REL_TOL), (
        "arms differ by %.1fx, expected the %d-step count"
        % (ratio, EXPECTED_STEPS))
