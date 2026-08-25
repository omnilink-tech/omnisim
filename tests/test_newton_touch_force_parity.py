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

"""Newton "force" TouchSensors must read the MOUNT force, like ODE did.

WHY THIS EXISTS
---------------
The "force"/"force-3d" TouchSensor was the second ODE mechanism blocking
src/ode deletion (_scratch/design_weld_touch.md): its value is the mount-joint
dJointFeedback, which on Newton-backed Solids measured artificially-disabled
proxy bodies and read eternal zeros. The replacement un-folds the sensor into
its OWN Newton body (fixed-welded to its parent -- newton keeps such a child
as a separate mjc body with its own geoms) and serves the mount wrench from
NEGATED mjData.cfrc_int after mj_rnePostConstraint, gated by the value-parsed
OMNISIM_NEWTON_TOUCH_FORCE.

THE MEASUREMENT
---------------
The canonical geometry (tests/api/worlds/touch_sensor_force.omniworld, rebuilt in
ENU): a 100 kg robot body 2 cm ABOVE its 100 kg force TouchSensor, which is
the only thing touching the floor, sensitive x-axis pointing DOWN. The number
that separates mount-force (right) from contact-sum (wrong) is:

    resting value = 981 N   (the robot's weight through the mount)
    NOT 1962 N              (the whole stack's floor contact)

and it lands in computeValue's kept-negative branch ONLY if the Newton read
carries the ODE f1 sign (force on the PARENT = negated cfrc_int) -- an
un-negated cfrc_int reads 0 here, which is exactly what this test pins.
Also asserted: ~0 in free fall (the stack spawns 5 cm up).

THE ODE ARM IS GONE
-------------------
This test used to run OMNISIM_FORCE_ODE=1 as a live oracle beside the Newton
arm and compare the two within 5%. src/ode is being deleted, so the ODE arm has
been DELETED from this file. Its answers were measured first and frozen into

    tests/goldens/ode_oracle_goldens.json  ->  families.touch_force

and the 5% parity budget now runs against those frozen values. The ABSOLUTE
981 N (+/- 3%) and the ~0 N free-fall reading are asserted as well: they are
backend-independent physics and outlive both the golden and ODE.

    python -m pytest tests/test_newton_touch_force_parity.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"
GOLDENS = REPO / "tests" / "goldens" / "ode_oracle_goldens.json"

EXPECTED_REST = 981.0     # 100 kg * 9.81 through the mount
ABS_REL_TOL = 0.03        # absolute budget (soft contacts settle)
GOLDEN_REL_TOL = 0.05     # vs the frozen ODE oracle (the task's original 5%)
FLIGHT_TOL = 5.0          # N; free fall reads ~0

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")

WORLD_TEXT = """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_touch_force_parity.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  newtonStatics TRUE
  coordinateSystem "ENU"
}
Viewpoint { position -3 0 1 }
Background { skyColor [ 0.2 0.2 0.25 ] }
DEF FLOOR Solid {
  translation 0 0 -0.05
  name "floor"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.3 0.35 roughness 1 metalness 0 }
      geometry Box { size 4 4 0.1 }
    }
  ]
  boundingObject Box { size 4 4 0.1 }
}
DEF STACK Robot {
  translation 0 0 0.47
  name "stack"
  controller "touch_force_probe"
  children [
    DEF ROBOT_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.7 0.7 0.75 roughness 1 metalness 0 }
      geometry Box { size 0.2 0.2 0.2 }
    }
    TouchSensor {
      name "ts"
      translation 0 0 -0.27
      rotation 0 1 0 1.5708
      type "force"
      children [
        DEF SENSOR_SHAPE Shape {
          appearance PBRAppearance { baseColor 0.8 0.3 0.2 roughness 1 metalness 0 }
          geometry Box { size 0.3 0.3 0.3 }
        }
      ]
      boundingObject USE SENSOR_SHAPE
      physics Physics { density -1 mass 100 }
      lookupTable [ ]
      resolution -1
    }
  ]
  boundingObject USE ROBOT_SHAPE
  physics Physics { density -1 mass 100 }
}
"""


def _goldens():
    return json.loads(GOLDENS.read_text(encoding="utf-8"))["families"]["touch_force"]["measurements"]


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run_newton(tmp_path, attempt):
    """Run the Newton arm. -> probe dict or None (bring-up flake)."""
    world = WORLDS / ".touch_force_parity.wbt"
    world.write_text(WORLD_TEXT, encoding="utf-8")
    out = tmp_path / ("probe_newton_%d.json" % attempt)
    log = tmp_path / ("engine_newton_%d.log" % attempt)
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_TOUCH_PROBE_OUT"] = str(out)
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    env["OMNISIM_NEWTON_TOUCH_FORCE"] = "1"
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(180):          # the probe runs ~2.6 s sim time; poll
            if out.exists():
                break
            try:
                proc.wait(timeout=1)
                break
            except subprocess.TimeoutExpired:
                continue
    finally:
        if proc.poll() is None:
            if os.name == "nt":
                subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                               capture_output=True)
            else:
                proc.kill()
            proc.wait()
        try:
            world.unlink()
        except OSError:
            pass
    blob = log.read_text(errors="replace") if log.exists() else ""
    if any(sig in blob for sig in _BRINGUP):
        return None
    if not out.exists():
        pytest.fail("the Newton arm produced no probe output\n%s" % blob[-1200:])
    return json.loads(out.read_text())


def _run_with_retry(tmp_path):
    """The Newton FFI bring-up flakes ~3% of launches; retry once before skipping."""
    for attempt in (1, 2):
        got = _run_newton(tmp_path, attempt)
        if got is not None:
            return got
    return None


# HISTORY -- this test carried an xfail(strict=False) from the ODE deletion until
# 2026-08-13, on the reading that Newton returned 0.00 N against the 980.998 N
# frozen oracle. Whatever caused that has since been fixed: measured 2026-08-13
# on machine 9722d23d12a3, three consecutive runs all read 981.0000419616694 N,
# i.e. the analytic 981 N to seven significant figures.
#
# THE MARKER IS REMOVED RATHER THAN FLIPPED TO strict, AND THE REASON MATTERS:
# under strict=False an XPASS is not a failure either, so for that whole period
# this test could not fail in EITHER direction -- it was reporting nothing while
# looking like coverage. That is exactly how the sibling bumper defect reached
# OmniBench lane 4 unnoticed. A test that cannot go red is not a gate.
def test_newton_touch_force_matches_frozen_ode_golden(tmp_path):
    newton = _run_with_retry(tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")
    g = _goldens()

    problems = []
    # (1) the absolute mount-force truth -- backend-independent
    if abs(newton["value_rest"] - EXPECTED_REST) > EXPECTED_REST * ABS_REL_TOL:
        problems.append("Newton rest value %.2f N vs the mount-force truth %.0f N. 0 means the "
                        "cfrc_int sign/negation is wrong or the sensor was never un-folded; "
                        "~1962 means it is reading the contact sum, not the mount."
                        % (newton["value_rest"], EXPECTED_REST))
    # (2) the frozen ODE oracle, same 5% budget the live comparison used
    rest_g = g["value_rest"]["ode_value"]
    if abs(newton["value_rest"] - rest_g) > abs(rest_g) * GOLDEN_REL_TOL:
        problems.append("Newton %.6f N vs the FROZEN ODE-oracle value %.6f N (>%g%%)"
                        % (newton["value_rest"], rest_g, GOLDEN_REL_TOL * 100))
    # (3) free fall reads ~0 on both the absolute and the frozen reading
    if abs(newton["value_flight"]) > FLIGHT_TOL:
        problems.append("Newton free-fall value %.2f N (should be ~0 -- the whole stack "
                        "accelerates at g, so the internal mount wrench vanishes)"
                        % newton["value_flight"])
    flight_g = g["value_flight"]["ode_value"]
    if abs(newton["value_flight"] - flight_g) > FLIGHT_TOL:
        problems.append("Newton free-fall value %.2f N vs the FROZEN ODE-oracle value %.2f N "
                        "(> %g N apart)" % (newton["value_flight"], flight_g, FLIGHT_TOL))

    assert not problems, (
        "touch-force parity against the frozen ODE goldens failed:\n  " + "\n  ".join(problems) +
        "\n\nThe reference numbers are FROZEN ODE-ORACLE VALUES -- the mount joint's own "
        "dJointFeedback f1 readings, measured before src/ode was removed and committed to "
        "tests/goldens/ode_oracle_goldens.json (families.touch_force). THE ODE ARM NO LONGER "
        "EXISTS, so they cannot be re-derived. Newton reads the NEGATED cfrc_int of the "
        "un-folded sensor body behind OMNISIM_NEWTON_TOUCH_FORCE=1; a mismatch is a Newton "
        "regression, not a stale golden. Do not retune the golden.")
