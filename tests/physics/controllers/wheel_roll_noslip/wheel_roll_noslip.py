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

"""wheel_roll_noslip -- the smoke lane's "the wheels actually roll" gate.

WHY THIS EXISTS. Every other check in this repo asks whether the body MOVED. A
robot dragging itself along on four locked wheels passes a headless run, a
displacement assertion, a "drove to the goal" verdict and a benchmark grader
alike -- and one in this tree did exactly that for its whole life: a 4-wheel
rover crossing an arena at 1.0-1.6 m/s while its four wheel hinges turned at
~0.14 rad/s, which at the authored 0.08 m radius is 0.011 m/s of rolling.
99.3% of that motion was slip and nothing noticed, because nothing anywhere
asserted that a wheel turned.

⚠ THE CHECK IS BEHAVIOURAL, NOT A TORQUE FORMULA, AND THAT IS DELIBERATE. The
obvious stiction model says the failing rover should have been fine: 0.4 N.m on
each of four 0.08 m wheels against 3.6 kg is ~5.6 m/s^2 of nominally available
traction. It slid anyway, and `maxTorque 12` fixed it. The mechanism is not
established, so nothing here predicts -- it measures both sides of

    omega_wheel * r  ==  v_body

and grades the residual.

FOUR ASSERTIONS, in the order they fire:

  1. THE ROVER DRIVES AT ALL. |v_body| >= 0.15 m/s over the window. A world
     where nothing moves satisfies no-slip trivially (0 == 0), so this gate
     refuses to grade one -- a vacuous green is worse than a red.

  2. NO-SLIP ON VELOCITIES.   |omega*r - v_fwd| <= TOL * |v_fwd|.

  3. NO-SLIP ON THE INTEGRALS. (theta_end - theta_start) * r must match the
     chassis's net displacement over the same window. Same physical property,
     but computed with no differentiation anywhere, so a single noisy sample
     cannot move it. This is the assertion the defect misses by ~100x.

  4. THE SENSOR AND THE RIGID BODY AGREE. omega from differentiating the
     PositionSensor is cross-checked against omega read from the wheel Solid's
     own angular velocity through the supervisor, de-rotated by the chassis's
     angular velocity (a chassis yawing at 2 rad/s would otherwise contribute
     2 rad/s of phantom "spin"). Two independent routes to the same number: a
     PositionSensor that merely echoed its own setpoint would sail through 2
     and 3 and fail this.

TOLERANCES, and why they are these numbers.

  TOL = 0.35 (assertions 2 and 3). NOT a guess and NOT fitted to make this
        world pass. It sits in the empty gap between two populations measured
        by scripts/dev/roll_check.py across this tree's hand-authored wheeled
        worlds: healthy rovers, e-pucks and battlebots land at 0.00-0.18 slip
        ratio, while the sliding defect lands at 0.97-1.00. 0.35 is ~2x the
        worst healthy world and ~3x below the mildest failure, so the verdict
        does not depend on where in that gap the line is drawn. It is
        deliberately LOOSE -- a driven wheel legitimately spins slightly faster
        than ground speed (positive slip is how a tyre makes tractive force at
        all), and a 4-wheel skid-steer chassis scrubs. Neither comes near 0.35.
        This world measures ~0.01. If a change ever lands this between 0.10 and
        0.35, do not retune: investigate, because nothing measured so far lives
        there.

  MIN_SPEED = 0.15 m/s (assertion 1). The rover is commanded at 5 rad/s x
        0.09 m = 0.45 m/s, so this is a third of the command -- loose enough
        that a slower solver or a friction change does not trip it, tight
        enough that "barely twitched" cannot be graded as rolling.

  OMEGA_AGREE = 0.10 rad/s + 8% (assertion 4). The absolute term covers
        differentiating a sensor over one 8 ms step; the relative term covers
        the two readings being taken at slightly different points in the
        engine's step.

NEGATIVE CONTROL. scripts/dev/roll_check_assets/wheel_roll_slip_negative_control.omniworld
is this world with maxTorque 12 -> 0.4. `python scripts/dev/roll_check.py
--self-test` runs the pair and requires OPPOSITE verdicts; measured results are
in docs/developer/roll-check.md. A gate that has only ever been seen to pass is
not evidence of anything.

REPORTING. Mirrors tests/lib/ts_utils.h and tests/physics/controllers/
gravity_rest_height: announce on the ts_emitter, append exactly one
"OK: <world stem>" or "FAILURE with <world stem>: ..." line to tests/output.txt,
then announce termination so TestSuiteSupervisor can advance instead of waiting
out its 30 s timeout. TEST_NAME must equal the world file's stem --
tests/smoke/run_smoke.py greps for "OK: <world stem>".
"""

import math
import os
import sys

from omnisim import Supervisor

TEST_NAME = "wheel_roll_noslip"
RESULTS_FILE = os.path.join("..", "..", "..", "output.txt")

WHEELS = (
    ("front_left_wheel_motor", "front_left_wheel_sensor", "FL_WHEEL"),
    ("front_right_wheel_motor", "front_right_wheel_sensor", "FR_WHEEL"),
    ("rear_left_wheel_motor", "rear_left_wheel_sensor", "RL_WHEEL"),
    ("rear_right_wheel_motor", "rear_right_wheel_sensor", "RR_WHEEL"),
)
WHEEL_AXIS = (0.0, 1.0, 0.0)      # HingeJointParameters.axis, robot-local
WORLD_UP = (0.0, 0.0, 1.0)        # ENU

OMEGA_CMD = 5.0                   # rad/s -> 0.45 m/s at r = 0.09
RAMP_STEPS = 60                   # 0.48 s of ramp: a step command is a torque spike
SETTLE_STEPS = 90                 # discarded entirely (drop + spin-up transient)
WINDOW_STEPS = 250                # 2.0 s of graded driving

TOL = 0.35
MIN_SPEED = 0.15
OMEGA_AGREE_ABS = 0.10
OMEGA_AGREE_REL = 0.08


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
        with open(RESULTS_FILE, "a", encoding="utf-8") as handle:
            handle.write(line)

    def _finish(self, code):
        self._notify(False)
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


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def unit(a):
    n = math.sqrt(dot(a, a))
    return tuple(x / n for x in a) if n > 1e-12 else (0.0, 0.0, 0.0)


def rotate(orientation, v):
    """Rotate v by a row-major 3x3 orientation matrix (Node.getOrientation)."""
    return (orientation[0] * v[0] + orientation[1] * v[1] + orientation[2] * v[2],
            orientation[3] * v[0] + orientation[4] * v[1] + orientation[5] * v[2],
            orientation[6] * v[0] + orientation[7] * v[1] + orientation[8] * v[2])


def median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def wheel_radius(supervisor, wheel_def, report):
    """Read r from the wheel's own boundingObject -- never hardcode it here.

    A gate that hardcodes the radius stops being a check on the world and
    becomes a check on this file agreeing with itself.
    """
    node = supervisor.getFromDef(wheel_def)
    if node is None:
        report.failure("scene has no DEF %s -- the world does not match this "
                       "controller" % wheel_def)
    current = node.getField("boundingObject").getSFNode()
    for _ in range(6):
        if current is None:
            break
        if current.getTypeName() == "Cylinder":
            return node, current.getField("radius").getSFFloat()
        children = current.getField("children")
        current = children.getMFNode(0) if children is not None else None
    report.failure("could not read a Cylinder radius from DEF %s's boundingObject"
                   % wheel_def)
    return None, None


def main():
    supervisor = Supervisor()
    step_ms = int(supervisor.getBasicTimeStep())
    dt = step_ms / 1000.0
    report = Reporter(supervisor)

    body = supervisor.getSelf()
    motors, sensors, wheel_nodes, radii = [], [], [], []
    for motor_name, sensor_name, wheel_def in WHEELS:
        motor = supervisor.getDevice(motor_name)
        sensor = supervisor.getDevice(sensor_name)
        if motor is None or sensor is None:
            report.failure("ROVER is missing %s / %s (got %r / %r)"
                           % (motor_name, sensor_name, motor, sensor))
        sensor.enable(step_ms)
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        node, radius = wheel_radius(supervisor, wheel_def, report)
        motors.append(motor)
        sensors.append(sensor)
        wheel_nodes.append(node)
        radii.append(radius)

    radius = median(radii)
    if not radius or radius <= 0.0:
        report.failure("wheel radius read from the scene is %r" % (radius,))

    # ---- drive ----------------------------------------------------------
    v_samples, omega_sensor_samples, omega_body_samples = [], [], []
    start_theta = end_theta = None
    start_position = end_position = None
    previous_theta = None

    total = SETTLE_STEPS + WINDOW_STEPS
    for k in range(total):
        scale = min(1.0, (k + 1) / float(RAMP_STEPS)) if RAMP_STEPS else 1.0
        for motor in motors:
            motor.setVelocity(OMEGA_CMD * scale)
        if supervisor.step(step_ms) == -1:
            report.failure("simulation ended after %d of %d steps, before the test "
                           "could conclude" % (k, total))

        theta = median([sensor.getValue() for sensor in sensors])
        if k < SETTLE_STEPS:
            previous_theta = theta
            continue

        if start_theta is None:
            start_theta = theta
            start_position = body.getPosition()

        # (a) omega from the PositionSensor, differentiated.
        omega_sensor_samples.append((theta - previous_theta) / dt)
        previous_theta = theta

        # (b) omega from the rigid bodies, chassis rotation removed.
        orientation = body.getOrientation()
        axis_world = unit(rotate(orientation, WHEEL_AXIS))
        forward = unit(cross(axis_world, WORLD_UP))
        body_velocity = body.getVelocity()
        body_angular = body_velocity[3:6]
        spins = []
        for node in wheel_nodes:
            wheel_velocity = node.getVelocity()
            spins.append(dot((wheel_velocity[3] - body_angular[0],
                              wheel_velocity[4] - body_angular[1],
                              wheel_velocity[5] - body_angular[2]), axis_world))
        omega_body_samples.append(median(spins))
        v_samples.append(dot(body_velocity[0:3], forward))
        end_theta = theta
        end_position = body.getPosition()

    if not v_samples:
        report.failure("no samples were collected in the %d-step window" % WINDOW_STEPS)

    v_fwd = median(v_samples)
    omega_sensor = median(omega_sensor_samples)
    omega_body = median(omega_body_samples)
    v_roll = omega_body * radius

    # ---- 1. it drove at all ---------------------------------------------
    if abs(v_fwd) < MIN_SPEED:
        report.failure("the rover barely moved: %.4f m/s over %.2f s against a "
                       "commanded %.3f rad/s x r=%.4f = %.3f m/s. No-slip cannot be "
                       "graded on a stationary robot (0 == 0 passes trivially), so "
                       "this is a FAILURE, not a pass"
                       % (v_fwd, WINDOW_STEPS * dt, OMEGA_CMD, radius,
                          OMEGA_CMD * radius))

    # ---- 2. no-slip, on velocities --------------------------------------
    residual = abs(v_roll - v_fwd)
    ratio = residual / abs(v_fwd)
    if ratio > TOL:
        needed = v_fwd / radius
        report.failure(
            "SLIP: body moves %.4f m/s but the wheels turn %.4f rad/s, which at "
            "r=%.4f m is only %.4f m/s of rolling -- %.1f%% of the motion is "
            "unaccounted for (tolerance %.0f%%). Rolling that fast would need "
            "%.2f rad/s. The chassis is being dragged, not driven: motor torque "
            "is going into the joint reaction instead of into the ground"
            % (v_fwd, omega_body, radius, v_roll, 100.0 * ratio, 100.0 * TOL,
               needed))

    # ---- 3. no-slip, on the integrals (no differentiation anywhere) ------
    travelled = math.sqrt(sum((end_position[i] - start_position[i]) ** 2
                              for i in range(3)))
    rolled = abs(end_theta - start_theta) * radius
    if travelled < 1e-6:
        report.failure("the chassis did not move at all over the graded window")
    integral_ratio = abs(rolled - travelled) / travelled
    if integral_ratio > TOL:
        report.failure(
            "SLIP (integrated): the chassis travelled %.4f m while the wheels wound "
            "%.4f rad = %.4f m of rolling -- %.1f%% unaccounted for over %.2f s "
            "(tolerance %.0f%%). This is the same property as the velocity check "
            "but computed without differentiating anything, so it is not a "
            "sampling artefact"
            % (travelled, abs(end_theta - start_theta), rolled,
               100.0 * integral_ratio, WINDOW_STEPS * dt, 100.0 * TOL))

    # ---- 4. the sensor and the rigid body agree --------------------------
    agree_budget = OMEGA_AGREE_ABS + OMEGA_AGREE_REL * abs(omega_body)
    if abs(omega_sensor - omega_body) > agree_budget:
        report.failure(
            "the two independent readings of wheel speed disagree: the "
            "PositionSensor differentiates to %.4f rad/s, the wheel body's own "
            "angular velocity (chassis rotation removed) is %.4f rad/s, "
            "difference %.4f > %.4f. One of them is not reporting the real joint"
            % (omega_sensor, omega_body, abs(omega_sensor - omega_body),
               agree_budget))

    report.success(
        "v_body=%.4f m/s, omega=%.4f rad/s x r=%.4f = %.4f m/s rolling "
        "(slip %.2f%% of body motion); integrated %.4f m rolled vs %.4f m "
        "travelled (%.2f%%); sensor omega %.4f vs body omega %.4f"
        % (v_fwd, omega_body, radius, v_roll, 100.0 * ratio, rolled, travelled,
           100.0 * integral_ratio, omega_sensor, omega_body))


main()
