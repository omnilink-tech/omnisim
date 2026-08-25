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

"""gravity_rest_height -- the smoke lane's rigid-body dynamics gate.

WHY THIS EXISTS. tests/smoke is the pre-push gate. After src/ode was deleted
(bdc02139) the lane was down to two worlds -- an empty-world startup check and a
PROTO-determinism check -- and asserted NO DYNAMICS AT ALL: a physics regression
sailed straight through `git push`. The three candidate physics worlds that
could have filled the hole are skipped for measured, engine-side reasons
(see tests/smoke/smoke_worlds.json), and the four next-best candidates
(damping, hinge_joint_damping, rolling_friction, floating_point_precision) fail
here for an unrelated reason: their C controllers are not built in this clone,
which surfaces only as a 30 s "results file has not been written" timeout. This
controller is PYTHON precisely so that failure mode cannot recur -- there is
nothing to build.

WHAT IT ASSERTS. Three measured numbers, each against a closed-form expectation
derived from the world's own authored fields, never from "whatever the engine
currently does":

  1. FREE-FALL ACCELERATION. FALLER is released from rest with nothing under it
     for 0.9 m. g is recovered from the SECOND DIFFERENCE of the recorded z
     trajectory:

         z[k+1] - 2 z[k] + z[k-1] = -g dt^2

     which is exact for any constant-acceleration integrator (explicit Euler,
     semi-implicit Euler, leapfrog all satisfy it), so the estimate carries no
     integrator-convention bias -- unlike comparing z(t) against the analytic
     parabola, which is off by g*dt*t/2 (~4 mm here) purely from discretisation.
     Expectation: WorldInfo.gravity, read live from the world file.

     This is the assertion that the "gravity was never plumbed into Newton, so
     every world ran at -9.81 regardless of WorldInfo.gravity" defect would have
     tripped, and the one an inert no-physics stub trips (g_est == 0).

  2. ANALYTIC REST HEIGHT ON A STATIC COLLIDER. After settling, FALLER's centre
     must sit at

         floor_top + faller_half_height

     with floor_top and the half-heights read from the scene graph. FLOOR's top
     face is at z = 0.5, NOT z = 0, on purpose: a floor at z = 0 coincides with
     Newton's implicit ground plane, which is exactly how the statics-off defect
     hid for so long. With statics broken the box lands near z = 0.1 (the
     implicit plane) or never stops -- both miss by >= 0.1 m against a 6 mm
     tolerance.

  3. A MOTORISED HINGE REACHES ITS COMMANDED ANGLE, checked two independent
     ways: the PositionSensor reading, and the arm link's ABSOLUTE world
     position rotated back into an angle. A sensor that merely echoed the
     setpoint would pass (1) and fail (2).

TOLERANCES, why they are these numbers, and how much of each is actually used.
Every bound is derived from a physical model, never fitted to an observation;
the measured column is what this world produces today on LAPTOP-H61DJILS
(RTX 3060 laptop) under solver "MuJoCo (cpu/mj_step, default)", bit-identical
across repeat runs:

  g          1.0 %  of WorldInfo.gravity = 0.0981 m/s^2. The second-difference
                    estimate is analytically exact for constant acceleration, so
                    this is pure headroom for solver bookkeeping. Measured
                    9.8124 vs 9.8100, i.e. 2.4 % of the budget. Zero gravity
                    misses by 100 %; a halved, doubled or reprojected g by
                    >= 50 %.
  rest z     6 mm   Soft normal contact can only ever settle BELOW the rigid
                    expectation, by at most about m*g/ke = 1*9.81/2500 = 3.9 mm
                    at the engine's default contact stiffness; 6 mm is that
                    worst case plus a little. The solver does far better in
                    practice -- measured 0.599892 vs 0.600000, a 0.108 mm
                    penetration, 1.8 % of the budget -- but the bound stays
                    sized to the model, not to the observation, so a legitimate
                    newtonContactKe change does not read as a regression. Every
                    collision-failure mode misses by >= 100 mm: with
                    OMNISIM_NEWTON_STATICS=0 the box measures z = 0.099892, off
                    by 0.500 m (83x the tolerance).
  |v| at rest 2 cm/s Settled, not sampled mid-bounce.
  hinge angle 0.02 rad sensor / 0.03 rad geometric (~1.1 / 1.7 deg) on a 0.6 rad
                    command. Measured 0.6012 sensor (6 % of budget) and 0.6020
                    geometric (7 %). A dead motor misses by the full 0.6 rad;
                    the historical cold-load articulation under-tracking was
                    ~1 cm of tip travel, i.e. ~0.07 rad here, so this would
                    still catch it.

NEGATIVE CONTROLS ACTUALLY RUN (a gate that has only ever been seen to pass is
not evidence of anything):
  OMNISIM_NEWTON_STATICS=0        -> assertion 2 fires, naming the implicit
                                     z=0 ground plane as the surface it landed
                                     on instead of FLOOR.
  FALLER's Physics node removed   -> assertion 1 fires with g_est = -0.000000
                                     from 597 airborne samples, which is the
                                     inert-stub / dead-physics signature.
  RotationalMotor without
  minPosition/maxPosition         -> assertion 3 fires at 0.003532 rad. This was
                                     not hypothetical: it is how this world was
                                     first authored, and the backend configures
                                     a limitless motor as a VELOCITY wheel whose
                                     setPosition() is ignored (it says so in an
                                     explicit engine WARNING).

REPORTING. Mirrors tests/lib/ts_utils.h: announce on the ts_emitter, append
exactly one "OK: <world stem>" or "FAILURE with <world stem>: ..." line to
tests/output.txt, then announce termination so the TestSuiteSupervisor can
advance instead of waiting out its 30 s timeout. TEST_NAME must equal the world
file's stem -- tests/smoke/run_smoke.py greps for "OK: <world stem>".
"""

import math
import os
import sys

from omnisim import Supervisor

# Must equal the stem of the .wbt this controller is used by: the C harness
# derives it from argv[0] (= the controller name), and both run_smoke.py's
# verdict grep and the human reading tests/output.txt key on it.
TEST_NAME = "gravity_rest_height"

# Relative to the controller's cwd (tests/physics/controllers/<name>/), same
# path tests/lib/ts_utils.h uses.
RESULTS_FILE = os.path.join("..", "..", "..", "output.txt")

HINGE_TARGET = 0.6          # rad, commanded on arm_motor
SETTLE_STEPS = 600          # 600 * 4 ms = 2.4 s: fall is 0.43 s, rest is the rest
G_TOLERANCE_FRAC = 0.01     # 1 % of WorldInfo.gravity
REST_TOLERANCE = 0.006      # m
REST_SPEED_TOLERANCE = 0.02  # m/s
HINGE_TOLERANCE_SENSOR = 0.02   # rad
HINGE_TOLERANCE_GEOM = 0.03     # rad


class Reporter:
    """The tests/lib/ts_utils.h protocol, in Python."""

    def __init__(self, robot):
        self._robot = robot
        self._emitter = robot.getDevice("ts_emitter")
        self._notify(True)

    def _notify(self, running):
        if self._emitter is not None:
            self._emitter.send("ts %d %d" % (1 if running else 0, os.getpid()))

    def _append(self, line):
        # Append-only, and only ever at the very end of the run: a controller
        # that truncates its output file on its first line makes any
        # poll-for-content wait succeed instantly and the engine gets torn down
        # before the world has finalised.
        with open(RESULTS_FILE, "a", encoding="utf-8") as handle:
            handle.write(line)

    def _finish(self, code):
        self._notify(False)
        # One more step so the engine actually transmits the termination packet
        # before this process goes away; without it the supervisor waits out its
        # 30 s timeout and reports a spurious FAILURE.
        self._robot.step(int(self._robot.getBasicTimeStep()))
        sys.exit(code)

    def success(self, detail):
        print("OK: %s (%s)" % (TEST_NAME, detail))
        self._append("OK: %s\n" % TEST_NAME)
        self._finish(0)

    def failure(self, message):
        print("FAILURE with %s: %s" % (TEST_NAME, message))
        self._append("FAILURE with %s: %s\n" % (TEST_NAME, message))
        self._finish(1)


def world_info_gravity(supervisor):
    """WorldInfo.gravity (SFFloat, magnitude along the world's down axis)."""
    children = supervisor.getRoot().getField("children")
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        if node is not None and node.getTypeName() == "WorldInfo":
            field = node.getField("gravity")
            if field is not None:
                return field.getSFFloat()
    return None


def bounding_box_size(node):
    """The `size` of a Solid's Box boundingObject, or None if unreadable."""
    if node is None:
        return None
    field = node.getField("boundingObject")
    if field is None:
        return None
    box = field.getSFNode()
    if box is None or box.getTypeName() != "Box":
        return None
    size = box.getField("size")
    return None if size is None else size.getSFVec3f()


def median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def main():
    supervisor = Supervisor()
    step_ms = int(supervisor.getBasicTimeStep())
    dt = step_ms / 1000.0
    report = Reporter(supervisor)

    # ---- the world's own numbers: everything below is derived, not hardcoded --
    gravity = world_info_gravity(supervisor)
    if gravity is None or gravity <= 0.0:
        report.failure("could not read WorldInfo.gravity from the scene (got %r)" % (gravity,))

    floor = supervisor.getFromDef("FLOOR")
    faller = supervisor.getFromDef("FALLER")
    arm_link = supervisor.getFromDef("ARM_LINK")
    if floor is None or faller is None or arm_link is None:
        report.failure("scene is missing FLOOR / FALLER / ARM_LINK (got %r / %r / %r)"
                       % (floor, faller, arm_link))

    floor_size = bounding_box_size(floor)
    faller_size = bounding_box_size(faller)
    if floor_size is None or faller_size is None:
        report.failure("could not read the Box boundingObject size of FLOOR / FALLER "
                       "(got %r / %r) -- the rest-height expectation cannot be derived"
                       % (floor_size, faller_size))

    floor_top = floor.getPosition()[2] + 0.5 * floor_size[2]
    expected_rest_z = floor_top + 0.5 * faller_size[2]
    start_z = faller.getPosition()[2]
    if start_z <= expected_rest_z + 0.2:
        report.failure("FALLER starts at z=%.6f, only %.6f m above its rest height %.6f "
                       "-- not enough free fall to measure g"
                       % (start_z, start_z - expected_rest_z, expected_rest_z))

    # ---- command the hinge, then let the world run -------------------------
    motor = supervisor.getDevice("arm_motor")
    sensor = supervisor.getDevice("arm_sensor")
    if motor is None or sensor is None:
        report.failure("PROBE is missing arm_motor / arm_sensor (got %r / %r)" % (motor, sensor))
    sensor.enable(step_ms)
    motor.setPosition(HINGE_TARGET)

    trajectory = []
    for _ in range(SETTLE_STEPS):
        if supervisor.step(step_ms) == -1:
            report.failure("simulation ended after %d of %d steps, before the test could "
                           "conclude" % (len(trajectory), SETTLE_STEPS))
        trajectory.append(faller.getPosition()[2])

    # ---- 1. free-fall acceleration -----------------------------------------
    # Airborne samples only, with a margin above the rest height so no contact
    # step contaminates the window, and skipping the first few steps so any
    # start-up transient is excluded.
    airborne = [k for k in range(2, len(trajectory) - 1)
                if trajectory[k] > expected_rest_z + 0.15]
    if len(airborne) < 10:
        report.failure("only %d usable airborne samples out of %d steps: FALLER went from "
                       "z=%.6f to z=%.6f -- it is not falling (gravity dead, or the body "
                       "has no physics)"
                       % (len(airborne), len(trajectory), start_z, trajectory[-1]))
    second_differences = [trajectory[k + 1] - 2.0 * trajectory[k] + trajectory[k - 1]
                          for k in airborne]
    measured_g = -median(second_differences) / (dt * dt)
    g_tolerance = G_TOLERANCE_FRAC * gravity
    if abs(measured_g - gravity) > g_tolerance:
        report.failure("free-fall acceleration is %.6f m/s^2, expected WorldInfo.gravity "
                       "%.6f +/- %.6f (measured from %d airborne samples at dt=%g s)"
                       % (measured_g, gravity, g_tolerance, len(airborne), dt))

    # ---- 2. analytic rest height on the static collider --------------------
    rest_z = trajectory[-1]
    velocity = faller.getVelocity()
    speed = math.sqrt(sum(component * component for component in velocity[:3]))
    if speed > REST_SPEED_TOLERANCE:
        report.failure("FALLER is still moving at %.6f m/s after %.3f s (z=%.6f): it never "
                       "came to rest, so the rest height cannot be graded"
                       % (speed, SETTLE_STEPS * dt, rest_z))
    if abs(rest_z - expected_rest_z) > REST_TOLERANCE:
        report.failure("FALLER rests at z=%.6f, expected floor_top(%.6f) + half_box(%.6f) = "
                       "%.6f +/- %.6f (error %+.6f m). It fell %.6f m from z=%.6f -- if the "
                       "error is about %.3f m it landed on the implicit z=0 ground plane "
                       "instead of FLOOR"
                       % (rest_z, floor_top, 0.5 * faller_size[2], expected_rest_z,
                          REST_TOLERANCE, rest_z - expected_rest_z, start_z - rest_z,
                          start_z, expected_rest_z - 0.5 * faller_size[2]))

    # ---- 3. the motorised hinge reached its command ------------------------
    sensor_angle = sensor.getValue()
    if abs(sensor_angle - HINGE_TARGET) > HINGE_TOLERANCE_SENSOR:
        report.failure("arm_sensor reads %.6f rad after %.3f s, expected the commanded "
                       "%.6f +/- %.6f rad"
                       % (sensor_angle, SETTLE_STEPS * dt, HINGE_TARGET,
                          HINGE_TOLERANCE_SENSOR))

    # Independent of the sensor: recover the angle from where the link actually
    # is. The hinge axis is +Y with its anchor at the PROBE origin, so an
    # endpoint authored at local (+r, 0, 0) sits at (r cos0, 0, -r sin0).
    base_position = supervisor.getSelf().getPosition()
    link_position = arm_link.getPosition()
    delta_x = link_position[0] - base_position[0]
    delta_z = link_position[2] - base_position[2]
    radius = math.hypot(delta_x, delta_z)
    if radius < 1e-6:
        report.failure("ARM_LINK is coincident with the hinge anchor -- cannot recover an "
                       "angle from its position")
    geometric_angle = math.atan2(-delta_z, delta_x)
    if abs(geometric_angle - HINGE_TARGET) > HINGE_TOLERANCE_GEOM:
        report.failure("ARM_LINK is at world (%.6f, %.6f) relative to the anchor, i.e. "
                       "%.6f rad, but %.6f rad was commanded (tolerance %.6f). The sensor "
                       "reads %.6f rad -- if the sensor agrees with the command and the "
                       "geometry does not, the joint is not actually moving"
                       % (delta_x, delta_z, geometric_angle, HINGE_TARGET,
                          HINGE_TOLERANCE_GEOM, sensor_angle))

    report.success("g=%.4f m/s^2 (expected %.4f), rest z=%.6f (expected %.6f), hinge "
                   "%.4f rad sensor / %.4f rad geometric (commanded %.4f)"
                   % (measured_g, gravity, rest_z, expected_rest_z, sensor_angle,
                      geometric_angle, HINGE_TARGET))


main()
