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

"""setPosition() must move a motor that declares no position limits -- and the
wheel that the "no limits means wheel" heuristic exists for must stay a wheel.

The Newton path picks a joint's actuator gains at REGISTRATION time, from the
only thing available then: whether the joint declares finite position limits.
A limit-less motorized hinge is classified as a velocity wheel (target_ke=0,
target_kd=500), which is right for a `continuous` URDF wheel and wrong for a
servo -- and it fails in the worst possible way, because a zero position gain
means the target is accepted, applied, and produces exactly zero force:

    commanded 0.5 rad, gravity off
      ODE                                        0.500000 rad
      Newton, no limits                          0.000000 rad
      Newton, minPosition/maxPosition declared   0.500000 rad

In Webots the real discriminator is a RUNTIME fact -- setPosition(inf) means
velocity control, a finite setPosition means position control -- and it arrives
long after registration. So the classification has to be revisable, and the two
halves of "revisable" are what this file pins:

  * ``test_setposition_moves_a_motor_with_no_declared_limits`` -- the servo. It
    FAILS on the pre-fix build (the joint reports ~0.000 against a commanded
    0.5) and PASSES once the first finite position target re-configures the
    joint with position-spring gains.

  * ``test_a_velocity_driven_wheel_stays_a_wheel`` -- the guard. The SAME world,
    the SAME limit-less joint shape, driven the way a wheel is actually driven
    (setPosition(inf) then setVelocity). It PASSES on both builds, and it is
    there to fail loudly if a fix reaches the classification by simply giving
    every limit-less joint a position spring: a spring on a husky wheel holds it
    at its authored angle and the robot stops driving.

Both joints live in ONE world under ONE controller, so neither result can be
explained by a difference in the run.

Gravity is off: nothing sags, so the only thing that can hold the servo's angle
is the servo, and the only thing that can turn the wheel is the velocity drive.
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

# Neither RotationalMotor declares minPosition/maxPosition and neither
# HingeJointParameters declares minStop/maxStop -- that is the whole point.
# Both joints therefore reach the registration-time classification as
# "limit-less" and are configured as velocity wheels. The controller then tells
# them apart the way Webots does: a finite setPosition vs setPosition(inf).
WORLD = """#VRML_SIM R2025a utf8
WorldInfo {
  basicTimeStep 4
  gravity 0
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
}
Viewpoint { orientation 0 0 1 0 position -4 0 2 }
DEF RIG Robot {
  translation 0 0 2
  name "rig"
  controller "servoprobe"
  children [
    Pose { rotation 0 1 0 1.5707963267948966 children [ Shape { geometry Capsule { height 0.5 radius 0.02 } } ] }
    HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor 0.25 0 0 }
      device [
        RotationalMotor { name "servo" maxTorque 100 }
        PositionSensor { name "servo_sensor" }
      ]
      endPoint DEF SERVO_LINK Solid {
        translation 0.5 0 0
        name "servo_link"
        children [ Pose { rotation 0 1 0 1.5707963267948966 children [ Shape { geometry Capsule { height 0.5 radius 0.02 } } ] } ]
        boundingObject Pose { rotation 0 1 0 1.5707963267948966 children [ Capsule { height 0.5 radius 0.02 } ] }
        physics Physics { density -1 mass 1 }
      }
    }
    HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor -0.25 0 0 }
      device [
        RotationalMotor { name "wheel" maxTorque 100 maxVelocity 20 }
        PositionSensor { name "wheel_sensor" }
      ]
      endPoint DEF WHEEL_LINK Solid {
        translation -0.5 0 0
        name "wheel_link"
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

# dt is 4 ms. The wheel is sampled at step 125 (0.5 s) -- ~1.0 rad at the
# commanded 2.0 rad/s, comfortably short of any angle-wrap ambiguity. The servo
# is read at the end (3.6 s), long after a ke=1000/kd=50 spring on a ~0.06
# kg.m^2 link has settled (slow pole ~ke/kd = 20 rad/s, tau ~0.05 s).
CONTROLLER = '''import os

from omnisim import Robot

out = open(os.environ["PROBE_OUT"], "w", buffering=1)
robot = Robot()
dt = int(robot.getBasicTimeStep())

servo = robot.getDevice("servo")
servo_sensor = robot.getDevice("servo_sensor")
servo_sensor.enable(dt)

wheel = robot.getDevice("wheel")
wheel_sensor = robot.getDevice("wheel_sensor")
wheel_sensor.enable(dt)

servo_target = 0.5
wheel_target_vel = 2.0

# Position control: a FINITE target. Velocity control: the Webots idiom.
servo.setPosition(servo_target)
wheel.setPosition(float("inf"))
wheel.setVelocity(wheel_target_vel)

wheel_early = 0.0
for i in range(900):
    if robot.step(dt) == -1:
        break
    if i == 125:
        wheel_early = wheel_sensor.getValue()

out.write("servo_target %.9f\\n" % servo_target)
out.write("servo %.9f\\n" % servo_sensor.getValue())
out.write("wheel_target_vel %.9f\\n" % wheel_target_vel)
out.write("wheel_early %.9f\\n" % wheel_early)
out.write("wheel_final %.9f\\n" % wheel_sensor.getValue())
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
def probe(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("servonolimits")
    worlds = tmp_path / "worlds"
    ctrl = tmp_path / "controllers" / "servoprobe"
    worlds.mkdir(parents=True, exist_ok=True)
    ctrl.mkdir(parents=True, exist_ok=True)
    (worlds / "s.wbt").write_text(WORLD, encoding="utf-8")
    (ctrl / "servoprobe.py").write_text(CONTROLLER, encoding="utf-8")

    result = tmp_path / "probe_out.txt"
    log = tmp_path / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(result),
               OMNISIM_LOG_PATH=str(log))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(worlds / "s.wbt")],
                       env=env, timeout=180, capture_output=True)
    except subprocess.TimeoutExpired:
        pass

    engine_log = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not result.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in engine_log:
                pytest.skip("Newton did not come up (%r); the run produced no "
                            "data, so it says nothing about setPosition" % sig)
        pytest.fail("the servo probe produced no output:\n%s" % engine_log[-1200:])

    values = {}
    for line in result.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 2:
            try:
                values[parts[0]] = float(parts[1])
            except ValueError:
                pass
    # Diagnostic only -- never asserted on. It exists so a failure report says
    # whether the engine even considered re-configuring the joint.
    values["_promotion_lines"] = "\n".join(
        ln for ln in engine_log.splitlines()
        if "re-configuring it with position-servo gains" in ln
        or "velocity wheel" in ln)
    return values


def _need(probe, key):
    v = probe.get(key)
    if v is None:
        pytest.fail("servo probe output incomplete, %r missing: %r"
                    % (key, {k: v for k, v in probe.items() if not k.startswith("_")}))
    return v


def test_setposition_moves_a_motor_with_no_declared_limits(probe):
    """A finite setPosition must actually drive the joint.

    FAILS on the pre-fix build: the joint is registered with target_ke=0, so the
    position target reaches a MuJoCo position actuator whose gainprm[0] is 0 and
    the joint reports ~0.000 rad. PASSES once the first finite target
    re-configures the joint with position-spring gains.
    """
    target = _need(probe, "servo_target")
    angle = _need(probe, "servo")
    assert abs(angle - target) <= 0.02, (
        "commanded %.6f rad on a motor with no declared position limits and the "
        "joint reports %.6f rad. setPosition was accepted, no error was raised, "
        "and nothing moved -- the caller cannot tell this from a slow servo.\n"
        "engine lines about the classification:\n%s"
        % (target, angle, probe.get("_promotion_lines") or "  (none)"))


def test_a_velocity_driven_wheel_stays_a_wheel(probe):
    """The case the "no limits means wheel" heuristic exists for.

    PASSES on both builds. It is the guard on the fix: making setPosition work
    by giving every limit-less joint a position spring would hold a wheel at its
    authored angle, and this assertion is what notices.
    """
    vel = _need(probe, "wheel_target_vel")
    early = _need(probe, "wheel_early")
    expected = vel * 0.5           # 125 steps x 4 ms
    assert early >= expected * 0.5, (
        "a wheel commanded setPosition(inf) + setVelocity(%.3f rad/s) turned only "
        "%.6f rad in 0.5 s (expected about %.3f). The velocity-wheel gain config "
        "is what drives it, and something took it away.\n"
        "engine lines about the classification:\n%s"
        % (vel, early, expected, probe.get("_promotion_lines") or "  (none)"))
