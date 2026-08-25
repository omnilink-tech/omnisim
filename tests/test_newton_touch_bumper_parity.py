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

"""A "bumper" TouchSensor with NO Physics node must still sense, and collide.

WHY THIS EXISTS
---------------
OmniBench lane 4 measured the bumper type at max = 0 over 750 samples and the
docs claimed it worked natively. The mechanism was the un-fold gate in
OmSolid.cpp: it required a Physics node for BOTH TouchSensor types, and a
bumper is the one device whose canonical authoring has none -- upstream needs
no physics for a bumper, the shipped PROTOs are written that way, and
OmTouchSensor::updateType demands physics only for "force"/"force-3d". So the
normal spelling never un-folded; and because the fold DROPS a folded child's
boundingObject, the pad was not even a collider.

ONE MECHANISM, TWO SYMPTOMS, SO TWO ASSERTIONS. A fix that restored only the
READ would leave the scene's collision geometry wrong by the pad's protrusion,
and a value-only test would call that green. Hence rest_z is asserted as
hard as the value is.

THE MEASUREMENT
---------------
Lane 4's own geometry (that is the scene that found the defect):

    floor top                z = 0.55
    chassis box 0.2          underside = z_robot - 0.100
    bumper pad box 0.025     underside = z_robot - 0.110   <- protrudes 10 mm

    pad IS the collider      -> rest z = 0.660, bumper reads 1
    pad was dropped by fold  -> rest z = 0.650, bumper reads 0   (the defect)

Both arms run in ONE test, because "1" only means something next to a "0" the
same rig can produce: OMNISIM_NEWTON_BUMPER=0 is the documented exact-revert
hatch, and it is asserted to reproduce the defect's numbers exactly. A rig
that cannot go red is not evidence.

    python -m pytest tests/test_newton_touch_bumper_parity.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"

FLOOR_TOP = 0.55
REST_Z_PAD = 0.66          # pad underside on the floor -- the correct answer
REST_Z_CHASSIS = 0.65      # chassis underside -- what the defect produced
Z_TOL = 0.005              # soft contacts settle a fraction of a mm

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")

WORLD_TEXT = """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_touch_bumper_parity.py -- do not commit.

WorldInfo {
  gravity 9.81
  basicTimeStep 4
  coordinateSystem "ENU"
  newtonSolver "mujoco"
  newtonStatics TRUE
  newtonRobotColliders TRUE
}
Viewpoint { position -6 -6 3 }
Background { skyColor [ 0.15 0.17 0.22 ] }
DEF FLOOR Solid {
  translation 0 0 0.45
  name "floor"
  children [
    DEF FLOOR_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.45 0.47 0.5 roughness 1 metalness 0 }
      geometry Box { size 20 20 0.2 }
    }
  ]
  boundingObject USE FLOOR_SHAPE
}
DEF PROBER Robot {
  translation 0 0 1.2
  name "prober"
  controller "touch_bumper_probe"
  supervisor TRUE
  children [
    DEF PROBER_BODY Shape {
      appearance PBRAppearance { baseColor 0.9 0.7 0.2 roughness 1 metalness 0 }
      geometry Box { size 0.2 0.2 0.2 }
    }
    TouchSensor {
      translation 0 0 -0.0975
      name "ts"
      type "bumper"
      children [
        DEF PAD_SHAPE Shape {
          appearance PBRAppearance { baseColor 0.2 0.2 0.25 roughness 1 metalness 0 }
          geometry Box { size 0.2 0.2 0.025 }
        }
      ]
      boundingObject USE PAD_SHAPE
      physics NULL
    }
  ]
  boundingObject USE PROBER_BODY
  physics Physics { density -1 mass 1 }
}
"""


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run(tmp_path, attempt, bumper_gate=None):
    """One headless run. -> probe dict, or None on a Newton bring-up flake."""
    world = WORLDS / ".touch_bumper_parity.wbt"
    world.write_text(WORLD_TEXT, encoding="utf-8")
    tag = "hatch" if bumper_gate == "0" else "default"
    out = tmp_path / ("probe_%s_%d.json" % (tag, attempt))
    log = tmp_path / ("engine_%s_%d.log" % (tag, attempt))
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["WEBOTS_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_BUMPER_PROBE_OUT"] = str(out)
    env["OMNISIM_REQUIRE_NEWTON"] = "1"
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE",
                                                    "OMNISIM_LEGACY",
                                                    "OMNISIM_ALLOW_ODE_FALLBACK"):
            env.pop(k)
    env["OMNISIM_REQUIRE_NEWTON"] = "1"
    if bumper_gate is not None:
        env["OMNISIM_NEWTON_BUMPER"] = bumper_gate
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
        pytest.fail("the bumper probe produced no output\n%s" % blob[-1200:])
    return json.loads(out.read_text())


def _with_retry(tmp_path, bumper_gate=None):
    """The Newton FFI bring-up flakes ~3% of launches; retry once."""
    for attempt in (1, 2):
        got = _run(tmp_path, attempt, bumper_gate=bumper_gate)
        if got is not None:
            return got
    return None


def test_bumper_without_physics_senses_and_collides(tmp_path):
    got = _with_retry(tmp_path)
    if got is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")

    problems = []
    if got["value_rest_duty"] < 0.5:
        problems.append(
            "the bumper read 1 on only %.0f%% of a resting window (want > 50%%). "
            "0%% is the defect signature: the sensor was not un-folded into a "
            "Newton body of its own, so the native contact set cannot be asked "
            "about it." % (100.0 * got["value_rest_duty"]))
    if abs(got["rest_z"] - REST_Z_PAD) > Z_TOL:
        extra = ""
        if abs(got["rest_z"] - REST_Z_CHASSIS) <= Z_TOL:
            extra = ("  This is EXACTLY the chassis-underside height %.3f, i.e. "
                     "the pad's boundingObject was dropped by the fold and is "
                     "not a collider -- the other half of the same defect."
                     % REST_Z_CHASSIS)
        problems.append(
            "rest z = %.5f, want %.3f (floor top %.2f + the pad's 0.11 m "
            "protrusion).%s" % (got["rest_z"], REST_Z_PAD, FLOOR_TOP, extra))
    if got["value_flight"] != 0.0:
        problems.append("the bumper read %g while still in free fall with "
                        "nothing near it -- a sensor that always reads 1 is "
                        "not a sensor" % got["value_flight"])
    assert not problems, ("bumper TouchSensor regression:\n  "
                          + "\n  ".join(problems))


def test_bumper_revert_hatch_reproduces_the_defect(tmp_path):
    """OMNISIM_NEWTON_BUMPER=0 must put the OLD behaviour back, exactly.

    This is the half that makes the test above evidence rather than an
    assertion: the same rig, one env var apart, produces the defect's own two
    numbers. It also keeps the documented hatch honest -- an exact-revert
    switch that has quietly stopped reverting is worse than none.
    """
    got = _with_retry(tmp_path, bumper_gate="0")
    if got is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")

    problems = []
    if got["value_rest_duty"] != 0.0:
        problems.append("the bumper still read 1 on %.0f%% of the resting "
                        "window with the un-fold switched OFF"
                        % (100.0 * got["value_rest_duty"]))
    if abs(got["rest_z"] - REST_Z_CHASSIS) > Z_TOL:
        problems.append("rest z = %.5f with the un-fold OFF, want the "
                        "chassis-underside %.3f (the pad should not be a "
                        "collider at all here)"
                        % (got["rest_z"], REST_Z_CHASSIS))
    assert not problems, ("OMNISIM_NEWTON_BUMPER=0 no longer reproduces the "
                          "pre-fix behaviour:\n  " + "\n  ".join(problems))
