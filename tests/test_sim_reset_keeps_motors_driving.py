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

"""A supervisor reset must not silently brake every wheel in the scene.

MEASURED SYMPTOM (harness `/sim/reset`, 2026-08-12): the call reports "authored
poses restored", 1250 subsequent `/sim/step`s advance 20.0 s of sim time at
normal per-step cost, and all 10 robots read 0.00 m net displacement AND 0.00 m
path. The same world drives 57.9-89.3 m/robot under `run-headless`.

MECHANISM (read from the source, then pinned here). A wheel is put into velocity
control by `setPosition(inf)`; on the Newton path the ONLY thing that records
that mode is the target being infinite -- `OmMotor::isPIDPositionControl()` is
literally `!isinf(mTargetPosition)`. The reset cascade overwrites it with a
finite number, twice:

  1. `OmMotor::reset`  -> `mTargetPosition = position()`   (the LIVE angle)
  2. `OmJoint::reset`  -> `setPosition(mSavedPositions[id])`
                       -> `motor->setTargetPosition(authored angle)`   <- survives

Either way the motor is now in "PID position control", so
`OmBasicJoint::pushNewtonMotorTargets` stops pushing the commanded wheel speed
and pushes **target velocity 0** instead. A limit-less wheel is registered with
the velocity-wheel actuator config (ke=0, kd=500), so a zero velocity target
with kd=500 is a hard brake -- and ke=0 means the position half contributes
nothing, which is why the wheel does not even unwind to its authored angle. It
just stops.

Nothing re-arms it: the supervisor path passes `restartControllers = false` by
design, so a controller that commanded its wheels once at start-up and then only
loops `robot.step()` -- the overwhelmingly common shape -- never speaks again.

THREE MEASUREMENTS PIN THAT MECHANISM RATHER THAN A PHYSICS FAULT (2026-08-12,
on this world):

  * the wheel's ANGULAR VELOCITY reads exactly 0.000000 after the reset, so the
    wheel is braked, not slipping;
  * having the controller re-issue `setPosition(inf)`/`setVelocity()` on the
    step it sees the clock rewind restores 6.048 m against 0.000 m -- same
    build, same world, one extra command;
  * `simulationReset()` ALONE and `loadState("__init__")` ALONE each reproduce
    it, which is why the fix has to sit under both entry points.

The two assertions below are a differential pair inside ONE run:

  * ``test_the_wheel_still_turns_after_a_supervisor_reset`` -- friction-free.
    The driver reads its own PositionSensor and reports the angle swept in the
    150 steps before the reset and in the 150 steps after it. Nothing about
    ground contact can explain a difference between the two windows.

  * ``test_the_robot_still_drives_after_a_supervisor_reset`` -- the headline,
    the user-visible claim: net chassis displacement, measured by the
    supervisor, in the same two windows.

`test_the_robot_drives_before_the_reset` is the control: it is what makes a
failure of the other two a reset bug rather than a broken world.
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

# A four-wheeled rover in the husky shape: motorised hinges with NO declared
# position limits, which is exactly the class that Newton registers as a
# velocity wheel (ke=0, kd=500). The floor's top face is at z=0 and each wheel
# has radius 0.1 with its centre at z=0.1, so the rover starts resting.
WORLD = """#VRML_SIM R2025a utf8
WorldInfo {
  basicTimeStep 8
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
  newtonGroundMu 1.5
}
Viewpoint {
  orientation 0 0 1 0
  position -6 0 3
}
DEF FLOOR Solid {
  translation 0 0 -0.25
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.4 0.4 0.4 roughness 1 metalness 0 }
      geometry Box { size 60 6 0.5 }
    }
  ]
  name "floor"
  boundingObject Box { size 60 6 0.5 }
}
DEF DRIVER Robot {
  translation 0 0 0.16
  name "driver"
  controller "driveprobe"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.6 0.1 roughness 1 metalness 0 }
      geometry Box { size 0.5 0.3 0.12 }
    }
    HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor 0.16 0.2 -0.06 }
      device [
        RotationalMotor { name "wfl" maxTorque 40 maxVelocity 40 }
        PositionSensor { name "wfl_s" }
      ]
      endPoint DEF WFL Solid {
        translation 0.16 0.2 -0.06
        name "wfl"
        children [ Shape { geometry Cylinder { radius 0.1 height 0.06 } } ]
        boundingObject Cylinder { radius 0.1 height 0.06 }
        physics Physics { density -1 mass 0.6 }
      }
    }
    HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor 0.16 -0.2 -0.06 }
      device [
        RotationalMotor { name "wfr" maxTorque 40 maxVelocity 40 }
        PositionSensor { name "wfr_s" }
      ]
      endPoint DEF WFR Solid {
        translation 0.16 -0.2 -0.06
        name "wfr"
        children [ Shape { geometry Cylinder { radius 0.1 height 0.06 } } ]
        boundingObject Cylinder { radius 0.1 height 0.06 }
        physics Physics { density -1 mass 0.6 }
      }
    }
    HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor -0.16 0.2 -0.06 }
      device [
        RotationalMotor { name "wrl" maxTorque 40 maxVelocity 40 }
        PositionSensor { name "wrl_s" }
      ]
      endPoint DEF WRL Solid {
        translation -0.16 0.2 -0.06
        name "wrl"
        children [ Shape { geometry Cylinder { radius 0.1 height 0.06 } } ]
        boundingObject Cylinder { radius 0.1 height 0.06 }
        physics Physics { density -1 mass 0.6 }
      }
    }
    HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor -0.16 -0.2 -0.06 }
      device [
        RotationalMotor { name "wrr" maxTorque 40 maxVelocity 40 }
        PositionSensor { name "wrr_s" }
      ]
      endPoint DEF WRR Solid {
        translation -0.16 -0.2 -0.06
        name "wrr"
        children [ Shape { geometry Cylinder { radius 0.1 height 0.06 } } ]
        boundingObject Cylinder { radius 0.1 height 0.06 }
        physics Physics { density -1 mass 0.6 }
      }
    }
  ]
  boundingObject Box { size 0.5 0.3 0.12 }
  physics Physics { density -1 mass 6 }
}
DEF SUP Robot {
  name "sup"
  controller "resetprobe"
  supervisor TRUE
  children []
}
"""

# THE SHAPE THAT MATTERS: command the wheels ONCE at start-up, then only step().
# Every stock drive controller in this repo is written this way, which is why
# the defect is silent -- there is no second command for the engine to overwrite.
DRIVER = '''import os

from omnisim import Robot

out = open(os.environ["PROBE_WHEEL_OUT"], "w", buffering=1)
robot = Robot()
dt = int(robot.getBasicTimeStep())

names = ["wfl", "wfr", "wrl", "wrr"]
motors = [robot.getDevice(n) for n in names]
sensor = robot.getDevice("wfl_s")
sensor.enable(dt)

TARGET_RAD_S = 8.0
for m in motors:
    m.setPosition(float("inf"))
    m.setVelocity(TARGET_RAD_S)

# SETTLE is not cosmetic. The reset makes the wheel angle JUMP once (measured:
# 0 -> 7.882409 rad within two steps, because the restore puts the bodies back
# but does not rewind the solver's joint_q), and a window that starts at the
# rewind reads that single jump as ~7.9 rad of "sweep" -- which is how an
# earlier draft of this test PASSED against a wheel whose angular velocity was
# exactly 0. Start the measurement window clear of the transient.
WINDOW = 150
SETTLE = 10
prev_t = None
a_start = None
before = 0.0
n_after = None
after_start = None
i = 0
written = False

while robot.step(dt) != -1:
    t = robot.getTime()
    a = sensor.getValue()
    if prev_t is not None and t < prev_t - 1e-9 and n_after is None:
        # The clock ran backwards: that is the reset landing.
        n_after = 0
    elif n_after is None:
        i += 1
        if i == SETTLE:
            a_start = a
        elif i == SETTLE + WINDOW:
            before = a - (a_start or 0.0)
    else:
        n_after += 1
        if n_after == SETTLE:
            after_start = a
        elif n_after == SETTLE + WINDOW and not written:
            written = True
            out.write("wheel_target_rad_s %.9f\\n" % TARGET_RAD_S)
            out.write("wheel_sweep_before %.9f\\n" % before)
            out.write("wheel_sweep_after %.9f\\n" % (a - (after_start or 0.0)))
            out.write("wheel_done 1\\n")
            out.flush()
    prev_t = t
'''

# The harness does exactly this pair: simulationReset() rewinds the clock, and
# loadState("__init__") restores the authored scene. Both funnel into the same
# node reset cascade, so either alone reproduces; issuing both is the faithful
# reproduction of `POST /sim/reset`.
SUPERVISOR = '''import os

from omnisim import Supervisor

out = open(os.environ["PROBE_OUT"], "w", buffering=1)
sup = Supervisor()
dt = int(sup.getBasicTimeStep())

driver = sup.getFromDef("DRIVER")
root = sup.getRoot()

WINDOW = 150


def x():
    return driver.getPosition()[0]


def run(n):
    """Advance n steps; return (net displacement, path length)."""
    start = x()
    prev = start
    path = 0.0
    for _ in range(n):
        if sup.step(dt) == -1:
            break
        cur = x()
        path += abs(cur - prev)
        prev = cur
    return x() - start, path


for _ in range(25):          # settle onto the floor
    sup.step(dt)

net_before, path_before = run(WINDOW)

sup.simulationReset()
if root is not None:
    root.loadState("__init__")
sup.step(dt)                 # the reset lands at the end of this step

x_after_reset = x()
net_after, path_after = run(WINDOW)

out.write("net_before %.9f\\n" % net_before)
out.write("path_before %.9f\\n" % path_before)
out.write("x_after_reset %.9f\\n" % x_after_reset)
out.write("net_after %.9f\\n" % net_after)
out.write("path_after %.9f\\n" % path_after)
out.write("done 1\\n")
out.flush()

for _ in range(90):          # let the driver finish its own window and write
    if sup.step(dt) == -1:
        break
sup.simulationQuit(0)
'''


def _binary():
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def _parse(path):
    values = {}
    if not path.is_file():
        return values
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 2:
            try:
                values[parts[0]] = float(parts[1])
            except ValueError:
                pass
    return values


@pytest.fixture(scope="module")
def probe(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("simreset")
    worlds = tmp_path / "worlds"
    worlds.mkdir(parents=True, exist_ok=True)
    for name, src in (("driveprobe", DRIVER), ("resetprobe", SUPERVISOR)):
        d = tmp_path / "controllers" / name
        d.mkdir(parents=True, exist_ok=True)
        (d / (name + ".py")).write_text(src, encoding="utf-8")
    (worlds / "r.wbt").write_text(WORLD, encoding="utf-8")

    sup_out = tmp_path / "sup_out.txt"
    wheel_out = tmp_path / "wheel_out.txt"
    log = tmp_path / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(sup_out),
               PROBE_WHEEL_OUT=str(wheel_out), OMNISIM_LOG_PATH=str(log))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(worlds / "r.wbt")],
                       env=env, timeout=300, capture_output=True)
    except subprocess.TimeoutExpired:
        pass

    engine_log = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    values = _parse(sup_out)
    values.update(_parse(wheel_out))
    if "done" not in values:
        for sig in _BRINGUP_SIGNATURES:
            if sig in engine_log:
                pytest.skip("Newton did not come up (%r); the run produced no "
                            "data, so it says nothing about /sim/reset" % sig)
        pytest.fail("the reset probe produced no supervisor output:\n%s"
                    % engine_log[-1500:])
    return values


def _need(probe, key):
    v = probe.get(key)
    if v is None:
        pytest.fail("reset probe output incomplete, %r missing: %r" % (key, probe))
    return v


def test_the_robot_drives_before_the_reset(probe):
    """CONTROL. Without this, a failure below could just be a broken world."""
    net = _need(probe, "net_before")
    assert net > 0.4, (
        "the rover did not drive even BEFORE any reset (net %.3f m in 150 steps); "
        "the world or the traction is wrong, so the reset assertions below say "
        "nothing" % net)


def test_the_wheel_still_turns_after_a_supervisor_reset(probe):
    """The friction-free half: a velocity-mode motor must survive a reset.

    RED on the pre-fix build -- the reset re-pins the wheel's target to a finite
    angle, `isPIDPositionControl()` flips true, and the joint is handed target
    velocity 0 against kd=500. The wheel stops dead and the sweep after the
    reset reads ~0 against ~9.6 rad before it.
    """
    before = _need(probe, "wheel_sweep_before")
    after = _need(probe, "wheel_sweep_after")
    assert before > 4.0, (
        "the wheel was not turning before the reset either (%.3f rad); this is a "
        "world/traction problem, not a reset problem" % before)
    assert after > 0.5 * before, (
        "THE RESET BRAKED THE WHEEL. Same motor, same command, same 150 steps: "
        "%.3f rad swept before the reset, %.3f rad after it. Nothing re-issued "
        "setPosition(inf)/setVelocity(), so the reset's finite position target "
        "is still standing on the joint." % (before, after))


def test_the_robot_still_drives_after_a_supervisor_reset(probe):
    """The headline: the user-visible claim, net chassis displacement."""
    net_before = _need(probe, "net_before")
    net_after = _need(probe, "net_after")
    path_after = _need(probe, "path_after")
    assert net_after > 0.5 * net_before, (
        "THE ROBOT IS FROZEN AFTER /sim/reset. It drove %.3f m in the 150 steps "
        "before the reset and %.3f m (path %.3f m) in the 150 steps after it, "
        "while the clock advanced normally." % (net_before, net_after, path_after))
