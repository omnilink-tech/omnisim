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

"""The Newton raycast service must answer LightSensor occlusion like ODE did.

WHY THIS EXISTS
---------------
A LightSensor with `occlusion TRUE` zeroes a light's direct contribution when
the sensor->light segment is blocked, answered under ODE by one ray geom per
light (OmLightSensor LightRay). The Newton replacement re-answers the same
segments in ONE OmNewtonBackend::raycastBatch call (mj_ray over the LIVE
mjModel), excluding the sensor's own robot bodies, behind the value-parsed
OMNISIM_NEWTON_RAYCAST gate.

THE MEASUREMENT
---------------
One generated world, one PointLight (intensity 1, constant attenuation,
ambient 0), two sensors with identity lookupTables on a body-less robot
(newtonStatics TRUE so the wall exists):

    ls_clear   at (0,0,0.5) aimed +x at the light (2,0,0.5)  -> reads 1.0
    ls_blocked at (4,0,0.5) aimed -x at the light, wall at x=3 -> reads 0.0

THE ODE ARM IS GONE
-------------------
This test used to run OMNISIM_FORCE_ODE=1 as a live oracle beside the Newton
arm and compare the two within 1e-6. src/ode is being deleted, so the ODE arm
has been DELETED from this file. Its answers were measured first and frozen into

    tests/goldens/ode_oracle_goldens.json  ->  families.lightsensor_occlusion

(measured: ls_clear 1.0, ls_blocked 0.0 exactly). The 1e-6 budget now runs
against those frozen values, and the ABSOLUTE expectations are asserted too --
they are the physics and outlive both the golden and ODE.

    python -m pytest tests/test_newton_lightsensor_occlusion_parity.py -v
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"
GOLDENS = REPO / "tests" / "goldens" / "ode_oracle_goldens.json"

#: sensor -> expected value (identity lookupTable; intensity 1, cos 0, atten 1)
EXPECTED = {"ls_clear": 1.0, "ls_blocked": 0.0}
#: the SAME budget the live ODE-vs-Newton comparison used, now applied against
#: the frozen ODE value instead of a live one.
GOLDEN_TOL = 1e-6
ABS_TOL = 1e-3


def _goldens():
    return json.loads(GOLDENS.read_text(encoding="utf-8"))["families"]["lightsensor_occlusion"]["measurements"]

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")


def _world_text():
    return """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_lightsensor_occlusion_parity.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  gravity 0
  newtonStatics TRUE
  coordinateSystem "ENU"
}
Viewpoint { position -3 0 1 }
Background { skyColor [ 0.2 0.2 0.25 ] }
PointLight {
  location 2 0 0.5
  intensity 1
  attenuation 1 0 0
}
DEF PROBE Robot {
  name "probe"
  controller "lightsensor_occlusion_probe"
  children [
    LightSensor {
      translation 0 0 0.5
      name "ls_clear"
      lookupTable [ 0 0 0, 10 10 0 ]
      occlusion TRUE
      resolution -1
    }
    LightSensor {
      translation 4 0 0.5
      rotation 0 0 1 3.14159265358979
      name "ls_blocked"
      lookupTable [ 0 0 0, 10 10 0 ]
      occlusion TRUE
      resolution -1
    }
  ]
}
DEF WALL Solid {
  translation 3 0 0.5
  name "wall"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.3 0.2 roughness 1 metalness 0 }
      geometry Box { size 0.2 1 1 }
    }
  ]
  boundingObject Box { size 0.2 1 1 }
}
"""


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run_newton(tmp_path, attempt):
    """Run the Newton arm. -> {sensor: value} or None (bring-up flake)."""
    world = WORLDS / ".lightsensor_occlusion_parity.wbt"
    world.write_text(_world_text(), encoding="utf-8")
    out = tmp_path / ("probe_newton_%d.json" % attempt)
    log = tmp_path / ("engine_newton_%d.log" % attempt)
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_LIGHTSENSOR_PROBE_OUT"] = str(out)
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    env["OMNISIM_NEWTON_RAYCAST"] = "1"
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(120):          # the probe writes after ~8 ticks; poll
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
    return {k: v["value"] for k, v in json.loads(out.read_text()).items()}


def _run_with_retry(tmp_path):
    """The Newton FFI bring-up flakes ~3% of launches; retry once before skipping."""
    for attempt in (1, 2):
        got = _run_newton(tmp_path, attempt)
        if got is not None:
            return got
    return None


def test_newton_lightsensor_occlusion_matches_frozen_ode_goldens(tmp_path):
    newton = _run_with_retry(tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")
    goldens = _goldens()

    problems = []
    for name, expected in EXPECTED.items():
        n = newton.get(name)
        if n is None:
            problems.append("%s: missing from the Newton probe output" % name)
            continue
        # (1) absolute occlusion physics -- backend-independent
        if not math.isclose(n, expected, abs_tol=ABS_TOL):
            problems.append("%s: Newton read %.6f, expected %.3f (|d|=%.2e > %g)"
                            % (name, n, expected, abs(n - expected), ABS_TOL))
        # (2) the frozen ODE oracle
        g = goldens[name]["ode_value"]
        if not math.isclose(n, g, abs_tol=GOLDEN_TOL):
            problems.append("%s: Newton %.9f vs the FROZEN ODE-oracle value %.9f "
                            "(|d|=%.2e > %g)" % (name, n, g, abs(n - g), GOLDEN_TOL))
    assert not problems, (
        "lightsensor occlusion parity against the frozen ODE goldens failed:\n  "
        + "\n  ".join(problems) +
        "\n\nThe reference numbers are FROZEN ODE-ORACLE VALUES, measured before src/ode was "
        "removed and committed to tests/goldens/ode_oracle_goldens.json "
        "(families.lightsensor_occlusion). THE ODE ARM NO LONGER EXISTS. Newton answers via "
        "mj_ray on the live mjModel behind OMNISIM_NEWTON_RAYCAST=1; a mismatch is a Newton "
        "raycast regression, not a stale golden. Do not retune the golden.")
