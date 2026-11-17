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

"""Every limit-less motor must be NAMED, not just the first one in the world.

WHY THIS EXISTS (internal parity plan, item W1.4)
--------------------------------------------
A motor whose joint declares no minPosition/maxPosition (and whose joint
declares no minStop/maxStop) is classified as a VELOCITY WHEEL by
OmBasicJoint: ke = 0, kd = 500. setPosition() on it then does nothing. For a
husky wheel that is correct. For a servo authored without limits -- the most
common robotics primitive there is -- it is a silent wrong answer: the motor
accepts the target, reports no error, and never moves.

The engine did warn. Once per PROCESS, via a `static bool`. On any world with
more than one limit-less motor -- i.e. essentially every wheeled-robot world --
the first wheel consumed the warning and every genuinely-affected servo after it
was configured in silence. Our own OmniBench author hit exactly this and
published a `broken` verdict against the engine.

THE MEASUREMENT
---------------
One world, TWO limit-less motorised hinges with distinct device names, on a
robot that is not a wheeled base. The warning must fire TWICE and must name
BOTH devices. Before the fix it fired once and named only `alpha_motor`, so
reverting the per-device set to a `static bool` turns this red.

The second arm is the control that keeps the first honest: a motor WITH
minPosition/maxPosition must produce no warning at all, so a rig that warned
unconditionally would fail here.

    python -m pytest tests/test_newton_limitless_servo_warning.py -v
"""
from __future__ import annotations

import os
import re
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"

#: The engine's own wording (OmBasicJoint.cpp). Matched loosely enough to
#: survive a copy edit, tightly enough that nothing else in the log matches.
WARN_RE = re.compile(r"Joint motor '([^']+)' declares no position limits")

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")


def _hinge(name, motor_name, y, limits):
    """One motorised hinge carrying a 1 kg link. `limits` -> a position servo."""
    lim = ("\n          minPosition -1.4\n          maxPosition 1.4" if limits else "")
    return """
    HingeJoint {
      jointParameters HingeJointParameters {
        anchor 0 %(Y)s 0.4
        axis 0 1 0
      }
      device [
        RotationalMotor {
          name "%(M)s"
          maxVelocity 6
          maxTorque 40%(LIM)s
        }
      ]
      endPoint DEF %(N)s Solid {
        translation 0 %(Y)s 0.25
        name "%(N)s"
        children [
          Shape {
            appearance PBRAppearance { baseColor 0.85 0.6 0.2 roughness 1 metalness 0 }
            geometry Box { size 0.1 0.1 0.3 }
          }
        ]
        boundingObject Box { size 0.1 0.1 0.3 }
        physics Physics { density -1 mass 1 }
      }
    }""" % {"N": name, "M": motor_name, "Y": y, "LIM": lim}


def _world_text(limits):
    return """#OMNISIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_limitless_servo_warning.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  newtonStatics TRUE
  coordinateSystem "ENU"
  newtonSolver "mujoco"
}
Viewpoint { position -3 0 1 }
Background { skyColor [ 0.15 0.18 0.24 ] }
DEF FLOOR Solid {
  translation 0 0 -0.05
  name "floor"
  children [
    DEF FLOOR_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.4 0.4 0.45 roughness 1 metalness 0 }
      geometry Box { size 8 8 0.1 }
    }
  ]
  boundingObject USE FLOOR_SHAPE
}
DEF RIG Robot {
  translation 0 0 0.05
  name "rig"
  controller "void"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.32 0.38 roughness 1 metalness 0 }
      geometry Box { size 1 1 0.1 }
    }%s%s
  ]
  boundingObject Box { size 1 1 0.1 }
  physics Physics { density -1 mass 200 }
}
""" % (_hinge("ALPHA", "alpha_motor", -0.3, limits),
       _hinge("BRAVO", "bravo_motor", 0.3, limits))


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run(tmp_path, limits):
    tag = "servo" if limits else "wheel"
    world = WORLDS / (".limitless_servo_%s.omniworld" % tag)
    world.write_text(_world_text(limits), encoding="utf-8")
    log = tmp_path / ("engine_%s.log" % tag)
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_REQUIRE_NEWTON"] = "1"
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    env["OMNISIM_REQUIRE_NEWTON"] = "1"
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(90):
            blob = log.read_text(errors="replace") if log.exists() else ""
            if "world finalised" in blob:
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
    if "world finalised" not in blob:
        pytest.fail("the world never finalised, so nothing registered:\n%s" % blob[-1500:])
    return blob


def test_every_limitless_motor_is_named(tmp_path):
    blob = _run(tmp_path, limits=False)
    if blob is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    named = WARN_RE.findall(blob)
    assert sorted(set(named)) == ["alpha_motor", "bravo_motor"], (
        "the limit-less-motor warning named %r. Both motors are limit-less and both were "
        "silently configured as velocity wheels, so both must be named. Naming only the "
        "first is the `static bool sWarnedLimitlessServo` defect: one warning per PROCESS, "
        "so on a world with any wheeled robot in it the wheel eats the warning and every "
        "real servo after it is degraded in silence." % (sorted(set(named)),))


def test_a_limited_motor_produces_no_warning(tmp_path):
    """The control. A warning that fires for everything names nothing."""
    blob = _run(tmp_path, limits=True)
    if blob is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    named = WARN_RE.findall(blob)
    assert not named, (
        "motors declaring minPosition/maxPosition were warned about as limit-less: %r. "
        "They are position servos and are configured correctly; warning here would train "
        "readers to ignore the message." % (named,))
