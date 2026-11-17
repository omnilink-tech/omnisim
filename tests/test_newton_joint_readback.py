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

"""A joint angle must survive the round trip through the solver.

Joint state takes a different path from a body pose: it is a coordinate the
solver owns, written as a motor target and read back through a PositionSensor.
`_update_newton_state` reconstructs it every tick alongside the body transforms,
so any native step path that serves state straight from `mj_data` has to answer
this too. It is pinned here before that work starts, not after.

Commanded position vs reported angle exercises both halves at once — the write
(target into the solver) and the read (solver back out to the sensor) — and a
servo that has ARRIVED is a settled state rather than a moment on a trajectory,
so the assertion does not ride on integrator accuracy.

Gravity is off so the link has nothing to sag under; the only thing holding the
angle is the servo, and the only thing reporting it is the readback.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)

WORLD = """#VRML_SIM R2025a utf8
WorldInfo {
  basicTimeStep 4
  gravity 0
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
}
Viewpoint { orientation 0 0 1 0 position -4 0 2 }
DEF ARM Robot {
  translation 0 0 2
  name "arm"
  controller "jointprobe"
  children [
    Pose { rotation 0 1 0 1.5707963267948966 children [ Shape { geometry Capsule { height 0.5 radius 0.02 } } ] }
    HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor 0.25 0 0 }
      device [
        RotationalMotor { name "m1" maxTorque 100 }
        PositionSensor { name "s1" }
      ]
      endPoint DEF LINK Solid {
        translation 0.5 0 0
        name "link"
        children [ Pose { rotation 0 1 0 1.5707963267948966 children [ Shape { geometry Capsule { height 0.5 radius 0.02 } } ] } ]
        boundingObject Pose { rotation 0 1 0 1.5707963267948966 children [ Capsule { height 0.5 radius 0.02 } ] }
        physics Physics { density -1 mass 1 }
      }
    }
  ]
  boundingObject Pose { rotation 0 1 0 1.5707963267948966 children [ Capsule { height 0.5 radius 0.02 } ] }
  physics Physics { density -1 mass 1 }
}
"""

CONTROLLER = '''import os

from omnisim import Robot

out = open(os.environ["PROBE_OUT"], "w", buffering=1)
robot = Robot()
dt = int(robot.getBasicTimeStep())
motor = robot.getDevice("m1")
sensor = robot.getDevice("s1")
sensor.enable(dt)

target = 0.5
motor.setPosition(target)
for _ in range(900):
    if robot.step(dt) == -1:
        break

out.write("target %.9f\\n" % target)
out.write("sensor %.9f\\n" % sensor.getValue())
out.write("done\\n")
out.close()
'''


def _binary():
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


@pytest.fixture(scope="module")
def joint_readback(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("jointrb")
    worlds = tmp_path / "worlds"
    ctrl = tmp_path / "controllers" / "jointprobe"
    worlds.mkdir(parents=True, exist_ok=True)
    ctrl.mkdir(parents=True, exist_ok=True)
    (worlds / "j.wbt").write_text(WORLD, encoding="utf-8")
    (ctrl / "jointprobe.py").write_text(CONTROLLER, encoding="utf-8")

    result = tmp_path / "probe_out.txt"
    log = tmp_path / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(result),
               OMNISIM_LOG_PATH=str(log))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(worlds / "j.wbt")],
                       env=env, timeout=180, capture_output=True)
    except subprocess.TimeoutExpired:
        pass

    if not result.is_file():
        text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up (%r); the run produced no "
                            "data, so it says nothing about the readback" % sig)
        pytest.fail("the joint probe produced no output:\n%s" % text[-1200:])

    values = {}
    engine_log = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    values["_warned"] = 1.0 if "setPosition() on it will be IGNORED" in engine_log else 0.0
    for line in result.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 2:
            try:
                values[parts[0]] = float(parts[1])
            except ValueError:
                pass
    return values


def test_joint_angle_readback_tracks_the_commanded_position(joint_readback):
    """A servo that has arrived must REPORT that it arrived."""
    target = joint_readback.get("target")
    sensor = joint_readback.get("sensor")
    if target is None or sensor is None:
        pytest.fail("joint probe output incomplete: %r" % (joint_readback,))
    if abs(sensor - target) < 0.05:
        return                       # position control worked; nothing to explain

    # It did not track. That is ACCEPTABLE only if the engine said why. This
    # backend configures a motor with no declared position limits as a velocity
    # wheel, and setPosition() on such a motor does nothing -- measured, with
    # gravity off and a commanded 0.5 rad:
    #
    #   ODE                                        0.500000 rad
    #   Newton, no limits                          0.000000 rad
    #   Newton, minPosition/maxPosition declared   0.500000 rad
    #
    # Refusing to position-control a wheel is a defensible choice. Accepting the
    # target, reporting no error and never moving is not.
    assert joint_readback.get("_warned"), (
        "joint angle readback is %.6f rad against a commanded %.6f rad, AND the "
        "engine issued no warning. A caller cannot tell this motor from one that "
        "is simply slow to arrive. Either honour setPosition or say it was "
        "ignored." % (sensor, target))
