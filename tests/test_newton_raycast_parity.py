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

"""The Newton raycast service must answer DistanceSensors like ODE did.

WHY THIS EXISTS
---------------
Raycasting is kernel blocker #1 for deleting src/ode: DistanceSensor, camera
recognition, LightSensor, Receiver and Radar all read through ODE ray geoms,
which only work in Newton worlds because every Newton solid keeps a live ODE
geom (the keepalive). The replacement service answers rays with mujoco's
mj_ray over the LIVE mjModel -- the same physics the world steps -- reached via
World.raycast_batch and OmNewtonBackend::raycastBatch, consumed first by
OmDistanceSensor behind the value-parsed OMNISIM_NEWTON_RAYCAST gate.

THE MEASUREMENT
---------------
One generated world, three sensors on a static rig, each aimed at a box whose
near face is at an exactly-known distance (identity lookupTables, resolution
-1, newtonStatics TRUE):

    ds_laser    laser,   1 ray  -> target at 1.000 m
    ds_generic  generic, 1 ray  -> target at 1.500 m
    ds_sonar    sonar,   1 ray  -> target at 2.000 m (normal incidence)

plus ds_lasertr, a laser fired through a transparency-1 boundingObject wall at
1.0 m at an opaque wall at 2.0 m -- the recipe that sets
OmGeometry::isTransparent, which ODE's LASER callback skipped and the Newton
path re-casts past.

THE ODE ARM IS GONE
-------------------
This test used to run TWO arms in-process -- OMNISIM_FORCE_ODE=1 as the live
oracle vs Newton + OMNISIM_NEWTON_RAYCAST=1 -- and compare them. src/ode is
being deleted, so that oracle cannot be run any more and the ODE arm has been
DELETED from this file. Its answers were measured first and frozen into

    tests/goldens/ode_oracle_goldens.json  ->  families.raycast

and this test now runs the Newton arm ONLY, asserting against those frozen
numbers within the same tolerance the live comparison used. The ABSOLUTE
geometry (1.0 / 1.5 / 2.0 m, and "one of the two walls" for the transparency
case) is asserted too: it does not depend on either backend and is the
stronger claim, so it outlives the golden as well as ODE.

    python -m pytest tests/test_newton_raycast_parity.py -v
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

#: sensor -> (aimed distance in metres). Absolute geometry: backend-independent.
EXPECTED = {"ds_laser": 1.0, "ds_generic": 1.5, "ds_sonar": 2.0}
#: mj_ray and ODE dCollide agreed to float precision on primitive faces; the
#: budget covers the lookup-table interpolation and the one-tick settle. This is
#: the SAME number the live ODE-vs-Newton comparison used -- it is now the
#: budget against the frozen ODE value instead of a live one.
GOLDEN_TOL = 1e-4
ABS_TOL = 5e-3

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")


def _goldens():
    return json.loads(GOLDENS.read_text(encoding="utf-8"))["families"]["raycast"]["measurements"]


def _world_text():
    def sensor(name, stype, y):
        return """
    DistanceSensor {
      translation 0 %(y)s 0.5
      name "%(N)s"
      type "%(T)s"
      lookupTable [ 0 0 0, 10 10 0 ]
      numberOfRays 1
      resolution -1
    }""" % {"N": name, "T": stype, "y": y}

    def target(name, y, dist):
        return """
DEF %(N)s_TARGET Solid {
  translation %(d)s %(y)s 0.5
  name "%(N)s_target"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.3 0.2 roughness 1 metalness 0 }
      geometry Box { size 0.4 0.6 0.6 }
    }
  ]
  boundingObject Box { size 0.4 0.6 0.6 }
}
""" % {"N": name, "y": y, "d": dist + 0.2}  # +half box depth: near face at `dist`

    # ds_lasertr aims through a TRANSPARENT wall at 1.0 m (boundingObject
    # Shape with transparency-1 appearance -- the recipe that sets
    # OmGeometry::isTransparent) at an opaque wall at 2.0 m. The frozen ODE
    # verdict is 2.0, i.e. the transparent wall IS skipped; the assertion
    # checks that frozen verdict AND that the reading is one of the two walls.
    transparent_wall = """
DEF TRANSP_WALL Solid {
  translation 1.2 6 0.5
  name "transp_wall"
  boundingObject Shape {
    appearance PBRAppearance { baseColor 1 1 1 transparency 1 roughness 1 metalness 0 }
    geometry Box { size 0.4 0.6 0.6 }
  }
}
"""

    return """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_raycast_parity.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  gravity 0
  newtonStatics TRUE
  coordinateSystem "ENU"
}
Viewpoint { position -3 0 1 }
Background { skyColor [ 0.2 0.2 0.25 ] }
DEF PROBE Robot {
  name "probe"
  controller "raycast_parity_probe"
  supervisor TRUE
  children [%s%s%s%s
  ]
}
%s%s%s%s%s""" % (sensor("ds_laser", "laser", 0.0),
                 sensor("ds_generic", "generic", 2.0),
                 sensor("ds_sonar", "sonar", 4.0),
                 sensor("ds_lasertr", "laser", 6.0),
                 target("ds_laser", 0.0, EXPECTED["ds_laser"]),
                 target("ds_generic", 2.0, EXPECTED["ds_generic"]),
                 target("ds_sonar", 4.0, EXPECTED["ds_sonar"]),
                 target("ds_lasertr", 6.0, 2.0),
                 transparent_wall)


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run_newton(tmp_path, attempt):
    """Run the Newton arm. -> {sensor: value} or None (bring-up flake)."""
    world = WORLDS / ".raycast_parity.wbt"
    world.write_text(_world_text(), encoding="utf-8")
    out = tmp_path / ("probe_newton_%d.json" % attempt)
    log = tmp_path / ("engine_newton_%d.log" % attempt)
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_RAYCAST_PROBE_OUT"] = str(out)
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


def _run_newton_with_retry(tmp_path):
    """The Newton FFI bring-up flakes ~3% of launches; retry once before skipping."""
    for attempt in (1, 2):
        got = _run_newton(tmp_path, attempt)
        if got is not None:
            return got
    return None


def test_newton_raycast_matches_frozen_ode_goldens(tmp_path):
    newton = _run_newton_with_retry(tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")
    goldens = _goldens()

    problems = []
    for name, dist in EXPECTED.items():
        n = newton.get(name)
        if n is None:
            problems.append("%s: missing from the Newton probe output" % name)
            continue
        # (1) absolute geometry -- outlives both the golden and ODE
        if not math.isclose(n, dist, abs_tol=ABS_TOL):
            problems.append("%s: Newton read %.6f, the target's near face is at %.3f m "
                            "(|d|=%.2e > %g)" % (name, n, dist, abs(n - dist), ABS_TOL))
        # (2) the frozen ODE oracle
        g = goldens[name]["ode_value"]
        if not math.isclose(n, g, abs_tol=GOLDEN_TOL):
            problems.append("%s: Newton %.9f vs the FROZEN ODE-oracle value %.9f "
                            "(|d|=%.2e > %g)" % (name, n, g, abs(n - g), GOLDEN_TOL))

    # LASER transparency. The frozen ODE verdict is the FAR wall (2.0) -- the
    # transparent geom is skipped. Assert that frozen verdict, and separately
    # that the reading is one of the two walls at all.
    n = newton.get("ds_lasertr")
    if n is None:
        problems.append("ds_lasertr: missing from the Newton probe output")
    else:
        if not (math.isclose(n, 1.0, abs_tol=ABS_TOL) or math.isclose(n, 2.0, abs_tol=ABS_TOL)):
            problems.append("ds_lasertr: Newton read %.6f -- neither the transparent wall "
                            "(1.0) nor the opaque one behind it (2.0)" % n)
        g = goldens["ds_lasertr"]["ode_value"]
        if not math.isclose(n, g, abs_tol=GOLDEN_TOL):
            problems.append("ds_lasertr: transparency verdict CHANGED -- Newton %.9f vs the "
                            "FROZEN ODE-oracle value %.9f. ODE skipped the transparent geom and "
                            "reported the wall behind it; the Newton LASER re-cast must agree."
                            % (n, g))

    assert not problems, (
        "raycast parity against the frozen ODE goldens failed:\n  " + "\n  ".join(problems) +
        "\n\nThe reference numbers are FROZEN ODE-ORACLE VALUES, measured from the live ODE "
        "arm before it was removed and committed to tests/goldens/ode_oracle_goldens.json "
        "(families.raycast). THE ODE ARM NO LONGER EXISTS -- these cannot be re-derived by "
        "re-running ODE. Newton answers via mj_ray on the live mjModel behind "
        "OMNISIM_NEWTON_RAYCAST=1; a mismatch is a Newton raycast regression, not a stale "
        "golden. Do not retune the golden to make this pass.")
