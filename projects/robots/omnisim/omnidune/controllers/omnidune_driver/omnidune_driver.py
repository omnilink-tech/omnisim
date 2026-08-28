#!/usr/bin/env python3
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

"""OmniDune driver -- scripted demo sequence + measured telemetry.

Everything this controller reports is MEASURED: chassis pose and velocity come
from the supervisor node, strut deflection from the struts' own PositionSensors,
steer angle from the steer PositionSensors, attitude from the InertialUnit. No
line echoes a command back as if it were an outcome; where a commanded value is
shown it is printed NEXT TO the measured one and labelled.

WHERE THE TELEMETRY GOES
------------------------
Both to stdout AND to a file, because stdout alone is not reliable here: on
Windows `omnisim-bin.exe` is a GUI-subsystem binary, so a controller's stdout is
DISCARDED -- a headless run's `<log>.stdout` sink comes back empty even though
the controller ran and drove the motors. The file sink is therefore the one that
always works:

    OMNIDUNE_TELEMETRY=<path>     explicit
    else, if OMNISIM_LOG_PATH is set:  <OMNISIM_LOG_PATH>.omnidune.txt
    else                          stdout only

THE THREE RULES THIS CONTROLLER EXISTS TO OBEY
----------------------------------------------
1. The struts are held at setPosition(0). That IS the spring: a SliderJoint's
   LinearMotor is always built position-controlled and Newton's PD servo gives
   k = maxForce * 10 [N/m] and c = maxForce * 0.5 [N.s/m]. Never command a
   strut anywhere else during normal driving -- doing so moves the ride height,
   it does not change the damping.
2. The drive motors are limit-less, so they are velocity wheels: they need
   setPosition(inf) + setVelocity(w). setPosition() on them is ignored.
3. The steer motors declare min/maxPosition, so they are position servos:
   they need setPosition(angle). setVelocity() on them is only a rate cap.

It is the `controller` field of the OmniDune PROTO -- there is nothing to launch.
"""

import json
import math
import os
import sys

from omnisim import Supervisor

# --------------------------------------------------------------------------
# Vehicle constants -- these MUST match OmniDune.proto. They are used only to
# turn a steering command into per-wheel Ackermann angles and to state the
# design predictions next to the measurements.
# --------------------------------------------------------------------------
WHEELBASE = 2.30          # m
TRACK = 1.70              # m
WHEEL_R = 0.34            # m
DESIGN_RIDE_Z = 0.58      # m -- Robot-origin height at zero strut deflection
STRUT_TRAVEL = 0.20       # m -- +/- stop position (400 mm total wheel travel)

# Predicted static sag, from  sag = g / (2*pi*f_n)^2  (see the PROTO header):
# front f_n 2.50 Hz, rear f_n 2.35 Hz.
PREDICTED_SAG = {"fl": 0.03976, "fr": 0.03976, "rl": 0.04500, "rr": 0.04500}
PREDICTED_RIDE_Z = DESIGN_RIDE_Z - 0.5 * (PREDICTED_SAG["fl"] + PREDICTED_SAG["rl"])

CORNERS = ("fl", "fr", "rl", "rr")
STEER_CORNERS = ("fl", "fr")

# --------------------------------------------------------------------------
# Scripted demo sequence: (t_end_seconds, label, wheel_rad_s, steer_rad)
# steer_rad is the AVERAGE (bicycle-model) steer angle; Ackermann splits it.
# --------------------------------------------------------------------------
SEQUENCE = (
    (2.50, "settle",      0.0, 0.00),   # static equilibrium -- this is where sag is measured
    (9.00, "bump-run",   10.0, 0.00),   # 3.4 m/s across the bumps at x = 5.0 / 6.5 / 8.0
    (13.50, "sprint",    25.0, 0.00),   # 8.5 m/s -- the drive check
    (17.00, "turn-left", 14.0, 0.32),   # ~6.9 m radius, Ackermann-split
    (18.50, "straight",  14.0, 0.00),
    (21.50, "brake",      0.0, 0.00),   # setVelocity(0) on a kd=500 velocity wheel IS the brake
)
# Throttle slew, rad/s^2. A step change of wheel target would spin the tyres and
# shock-load the driveline; real throttle and brake are not steps either.
THROTTLE_RATE = 12.0
BRAKE_RATE = 40.0


SEQUENCE_END = SEQUENCE[-1][0]

# --------------------------------------------------------------------------
# Lap mode. When the scripted verification finishes, the buggy stops being a
# test rig and starts driving the course: a pure-pursuit follower on the
# generator's own centerline, so the racing line the terrain was BUILT around
# is the one the car actually takes. Waypoints come from
# worlds/dune_course_centerline.json (exported from gen_dune_course.py, same
# seed), never from numbers retyped here -- if the course is regenerated the
# car follows the new one.
# --------------------------------------------------------------------------
STEER_MAX = 0.55          # rad, inside the PROTO's 0.60 limit
V_MAX = 15.0              # m/s on a straight
V_MIN = 5.5               # m/s through the tightest berm
LD_MIN, LD_MAX = 7.0, 20.0


class LapFollower:
    """Pure pursuit over the course centerline. Returns (label, wheel_w, steer)."""

    def __init__(self):
        self.pts = []
        self.lap = 0
        self.i = 0
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.normpath(os.path.join(here, "..", "..", "worlds",
                                             "dune_course_centerline.json"))
        try:
            with open(path) as fh:
                d = json.load(fh)
            self.pts = [(float(a), float(b)) for a, b, _ in d["points"]]
            emit("[omnidune] lap mode: %d waypoints, lap %.1f m, from %s"
                 % (len(self.pts), d.get("lap_length_m", 0.0), os.path.basename(path)))
        except Exception as exc:
            emit("[omnidune] lap mode UNAVAILABLE (%s: %s) -- holding station" %
                 (type(exc).__name__, exc))

    def _nearest(self, x, y):
        n = len(self.pts)
        # search a window around the last index so this stays O(1) per tick and
        # cannot jump across the course when two parts of the lap run close by
        best, bd = self.i, 1e30
        for k in range(-25, 60):
            j = (self.i + k) % n
            px, py = self.pts[j]
            d = (px - x) ** 2 + (py - y) ** 2
            if d < bd:
                bd, best = d, j
        return best

    def command(self, pose, rpy, speed):
        if not self.pts or pose is None or rpy is None:
            return "lap", 0.0, 0.0
        n = len(self.pts)
        x, y = pose[0], pose[1]
        yaw = rpy[2]
        v = speed or 0.0

        j = self._nearest(x, y)
        if j < self.i and (self.i - j) > n // 2:      # wrapped past start/finish
            self.lap += 1
            emit("[omnidune] lap %d complete" % self.lap)
        self.i = j

        ld = max(LD_MIN, min(LD_MAX, 0.9 * v + 6.0))
        acc, k = 0.0, j
        px, py = self.pts[j]
        while acc < ld:
            k = (k + 1) % n
            qx, qy = self.pts[k]
            acc += math.hypot(qx - px, qy - py)
            px, py = qx, qy
        tx, ty = self.pts[k]

        # target in vehicle frame -> pure-pursuit steer (bicycle model)
        dx, dy = tx - x, ty - y
        fx = math.cos(yaw) * dx + math.sin(yaw) * dy
        fy = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(fy, max(fx, 0.1))
        steer = math.atan2(2.0 * WHEELBASE * math.sin(alpha), max(acc, 1.0))
        steer = max(-STEER_MAX, min(STEER_MAX, steer))

        # slow for corners: the harder the wheel is turned, the lower the target
        v_t = V_MIN + (V_MAX - V_MIN) * max(0.0, 1.0 - abs(steer) / STEER_MAX) ** 1.5
        return "lap", v_t / WHEEL_R, steer


# --------------------------------------------------------------------------
def _open_telemetry():
    path = os.environ.get("OMNIDUNE_TELEMETRY")
    if not path:
        log = os.environ.get("OMNISIM_LOG_PATH")
        if log:
            path = log + ".omnidune.txt"
    if not path:
        return None
    try:
        return open(path, "w", buffering=1, encoding="utf-8")
    except OSError as exc:
        print("[omnidune] could not open telemetry file %r: %s" % (path, exc), flush=True)
        return None


_TEL = _open_telemetry()


def emit(msg):
    print(msg, flush=True)
    if _TEL is not None:
        try:
            _TEL.write(msg + "\n")
        except OSError:
            pass


def ackermann(avg_steer):
    """Per-wheel steer angles for a commanded average (bicycle) steer angle.

    Returns (left, right) in radians. Positive = left turn, so on a left turn
    the LEFT wheel is the inner one and takes the larger angle.
    """
    if abs(avg_steer) < 1e-4:
        return 0.0, 0.0
    radius = WHEELBASE / math.tan(avg_steer)          # signed turn radius
    inner = math.atan(WHEELBASE / (radius - math.copysign(TRACK / 2.0, radius)))
    outer = math.atan(WHEELBASE / (radius + math.copysign(TRACK / 2.0, radius)))
    return (inner, outer) if avg_steer > 0 else (outer, inner)


def phase_at(t):
    """(label, wheel_target_rad_s, steer_rad) for sim time t."""
    for t_end, label, w, steer in SEQUENCE:
        if t < t_end:
            return label, w, steer
    return "done", 0.0, 0.0


class OmniDuneDriver(object):
    def __init__(self):
        self.robot = Supervisor()
        self.dt_ms = int(self.robot.getBasicTimeStep())
        self.dt = self.dt_ms / 1000.0

        # ---- devices -----------------------------------------------------
        self.struts = {}
        self.strut_sensors = {}
        self.wheels = {}
        self.wheel_sensors = {}
        self.steers = {}
        self.steer_sensors = {}

        for c in CORNERS:
            self.struts[c] = self.robot.getDevice("%s_strut" % c)
            self.strut_sensors[c] = self.robot.getDevice("%s_strut_sensor" % c)
            self.wheels[c] = self.robot.getDevice("%s_wheel" % c)
            self.wheel_sensors[c] = self.robot.getDevice("%s_wheel_sensor" % c)
        for c in STEER_CORNERS:
            self.steers[c] = self.robot.getDevice("%s_steer" % c)
            self.steer_sensors[c] = self.robot.getDevice("%s_steer_sensor" % c)
        self.imu = self.robot.getDevice("imu")

        named = ([("%s_strut" % c, self.struts[c]) for c in CORNERS] +
                 [("%s_strut_sensor" % c, self.strut_sensors[c]) for c in CORNERS] +
                 [("%s_wheel" % c, self.wheels[c]) for c in CORNERS] +
                 [("%s_steer" % c, self.steers[c]) for c in STEER_CORNERS] +
                 [("imu", self.imu)])
        missing = [n for n, d in named if d is None]
        if missing:
            emit("[omnidune] MISSING DEVICES: %s" % ", ".join(missing))

        for c in CORNERS:
            if self.strut_sensors[c] is not None:
                self.strut_sensors[c].enable(self.dt_ms)
            if self.wheel_sensors[c] is not None:
                self.wheel_sensors[c].enable(self.dt_ms)
        for c in STEER_CORNERS:
            if self.steer_sensors[c] is not None:
                self.steer_sensors[c].enable(self.dt_ms)
        if self.imu is not None:
            self.imu.enable(self.dt_ms)

        # ---- the suspension: hold every strut at its design ride height.
        # This is the ONLY strut command of the whole run. The PD servo around
        # this setpoint IS the spring; there is no passive spring to declare.
        for c in CORNERS:
            if self.struts[c] is not None:
                self.struts[c].setVelocity(8.0)
                self.struts[c].setPosition(0.0)

        # ---- drive motors are limit-less => velocity wheels.
        for c in CORNERS:
            if self.wheels[c] is not None:
                self.wheels[c].setPosition(float("inf"))
                self.wheels[c].setVelocity(0.0)

        # ---- measurement state ------------------------------------------
        self.node = self.robot.getSelf()
        if self.node is None:
            emit("[omnidune] WARNING: not a supervisor -- chassis pose and speed "
                 "are UNAVAILABLE and will report as n/a, never as a guess.")
        self.start_xy = None
        self.settled_z = None
        self.settled_sag = {}
        self.strut_min = dict((c, 1e9) for c in CORNERS)
        self.strut_max = dict((c, -1e9) for c in CORNERS)
        self.max_speed = 0.0
        self.yaw_at_turn_start = None
        self.yaw_at_turn_end = None
        self.last_phase = None
        self.next_report_t = 0.0
        self.summary_done = False
        self.wheel_cmd = 0.0          # slew-limited throttle state

    # ---------------------------------------------------------------------
    # measurement helpers -- every one of these reads the simulator.
    # ---------------------------------------------------------------------
    def pose(self):
        return None if self.node is None else self.node.getPosition()

    def speed(self):
        """Horizontal speed magnitude, m/s, from the body's measured velocity."""
        if self.node is None:
            return None
        v = self.node.getVelocity()
        return math.sqrt(v[0] * v[0] + v[1] * v[1])

    def rpy(self):
        return None if self.imu is None else self.imu.getRollPitchYaw()

    def strut_deflections(self):
        return dict((c, None if self.strut_sensors[c] is None
                     else self.strut_sensors[c].getValue()) for c in CORNERS)

    def steer_measured(self):
        return dict((c, None if self.steer_sensors[c] is None
                     else self.steer_sensors[c].getValue()) for c in STEER_CORNERS)

    # ---------------------------------------------------------------------
    def run(self):
        emit("[omnidune] OmniDune driver up. dt = %d ms" % self.dt_ms)
        emit("[omnidune] design: zero-deflection ride z %.4f m | predicted settled z "
             "%.4f m | predicted sag front %.2f mm rear %.2f mm | strut travel +/-%.0f mm"
             % (DESIGN_RIDE_Z, PREDICTED_RIDE_Z,
                PREDICTED_SAG["fl"] * 1000.0, PREDICTED_SAG["rl"] * 1000.0,
                STRUT_TRAVEL * 1000.0))

        t = 0.0
        self._lap_started = False
        self.lap = None
        while self.robot.step(self.dt_ms) != -1:
            t += self.dt
            if t <= SEQUENCE_END:
                label, wheel_w, steer_cmd = phase_at(t)
            else:
                if not self._lap_started:
                    self._lap_started = True
                    self.report_summary()
                    self.lap = LapFollower()
                label, wheel_w, steer_cmd = self.lap.command(
                    self.pose(), self.rpy(), self.speed())

            if label != self.last_phase:
                emit("[omnidune] t=%6.2f s  PHASE -> %-10s  (wheel target %5.1f rad/s, "
                     "steer cmd %+.3f rad)" % (t, label, wheel_w, steer_cmd))
                if label == "turn-left":
                    r = self.rpy()
                    self.yaw_at_turn_start = None if r is None else r[2]
                if self.last_phase == "turn-left":
                    r = self.rpy()
                    self.yaw_at_turn_end = None if r is None else r[2]
                self.last_phase = label

            # ---- actuate ------------------------------------------------
            rate = THROTTLE_RATE if wheel_w >= self.wheel_cmd else BRAKE_RATE
            step = rate * self.dt
            if wheel_w > self.wheel_cmd:
                self.wheel_cmd = min(wheel_w, self.wheel_cmd + step)
            else:
                self.wheel_cmd = max(wheel_w, self.wheel_cmd - step)

            left, right = ackermann(steer_cmd)
            if self.steers["fl"] is not None:
                self.steers["fl"].setPosition(left)
            if self.steers["fr"] is not None:
                self.steers["fr"].setPosition(right)
            for c in CORNERS:
                if self.wheels[c] is not None:
                    self.wheels[c].setVelocity(self.wheel_cmd)

            # ---- measure ------------------------------------------------
            defl = self.strut_deflections()
            for c in CORNERS:
                if defl[c] is None:
                    continue
                if defl[c] < self.strut_min[c]:
                    self.strut_min[c] = defl[c]
                if defl[c] > self.strut_max[c]:
                    self.strut_max[c] = defl[c]

            sp = self.speed()
            if sp is not None and sp > self.max_speed:
                self.max_speed = sp

            p = self.pose()
            if p is not None and self.start_xy is None and t > 0.05:
                self.start_xy = (p[0], p[1])

            # End of the settle phase: capture the static equilibrium.
            if self.settled_z is None and t >= 2.40:
                if p is not None:
                    self.settled_z = p[2]
                self.settled_sag = dict((c, defl[c]) for c in CORNERS)
                self.report_static()

            # ---- 1 Hz telemetry -----------------------------------------
            if t >= self.next_report_t:
                self.next_report_t += 1.0
                self.report(t, label, steer_cmd)

            # The sequence is over: publish the verdict now rather than waiting
            # for the engine to quit (a headless run is usually killed, not
            # closed, so an exit-only summary is a summary you never see).
            if label == "done" and not self.summary_done:
                self.summary_done = True
                self.report_summary()

        if not self.summary_done:
            self.report_summary()

    # ---------------------------------------------------------------------
    def report(self, t, label, steer_cmd):
        p = self.pose()
        sp = self.speed()
        r = self.rpy()
        d = self.strut_deflections()
        sm = self.steer_measured()

        pos_s = ("x %8.3f y %7.3f z %6.4f" % (p[0], p[1], p[2])) if p else "pose UNAVAILABLE"
        spd_s = ("%5.2f" % sp) if sp is not None else "  n/a"
        yaw_s = ("%+7.4f" % r[2]) if r else "    n/a"
        pit_s = ("%+6.3f" % r[1]) if r else "   n/a"
        rol_s = ("%+6.3f" % r[0]) if r else "   n/a"
        defl_s = " ".join(("%s %+7.2f" % (c, d[c] * 1000.0)) if d[c] is not None
                          else ("%s     n/a" % c) for c in CORNERS)
        st_s = " ".join(("%s %+.3f" % (c, sm[c])) if sm[c] is not None
                        else ("%s n/a" % c) for c in STEER_CORNERS)

        emit("[omnidune] t=%6.2f %-9s | %s | v %s m/s | yaw %s pitch %s roll %s | "
             "strut mm: %s | steer meas %s (cmd %+.3f)"
             % (t, label, pos_s, spd_s, yaw_s, pit_s, rol_s, defl_s, st_s, steer_cmd))

    def report_static(self):
        emit("[omnidune] ---- STATIC EQUILIBRIUM (end of settle phase) ----")
        if self.settled_z is not None:
            emit("[omnidune]   chassis z : measured %.5f m | predicted %.5f m | "
                 "error %+.2f mm" % (self.settled_z, PREDICTED_RIDE_Z,
                                     (self.settled_z - PREDICTED_RIDE_Z) * 1000.0))
        for c in CORNERS:
            m = self.settled_sag.get(c)
            pred = PREDICTED_SAG[c]
            if m is None:
                emit("[omnidune]   %s sag   : UNAVAILABLE" % c)
                continue
            emit("[omnidune]   %s sag   : measured %6.2f mm | predicted %6.2f mm | "
                 "error %+6.1f %%" % (c, abs(m) * 1000.0, pred * 1000.0,
                                      100.0 * (abs(m) - pred) / pred))
        emit("[omnidune] --------------------------------------------------")

    def report_summary(self):
        emit("[omnidune] ================= RUN SUMMARY =================")
        p = self.pose()

        # 1. ride height / static sag
        ok_sag = bool(self.settled_sag)
        for c in CORNERS:
            m = self.settled_sag.get(c)
            if m is None or abs(abs(m) - PREDICTED_SAG[c]) > 0.15 * PREDICTED_SAG[c]:
                ok_sag = False
        emit("[omnidune] 1 RIDE HEIGHT : %s -- all four corners within 15%% of the "
             "design sag" % ("PASS" if ok_sag else "FAIL"))

        # 2. drive
        dist = None
        if p is not None and self.start_xy is not None:
            dist = math.hypot(p[0] - self.start_xy[0], p[1] - self.start_xy[1])
        emit("[omnidune] 2 DRIVE       : %s -- travelled %s m, peak measured speed "
             "%.2f m/s"
             % ("PASS" if (dist is not None and dist > 5.0) else "FAIL",
                ("%.3f" % dist) if dist is not None else "n/a", self.max_speed))

        # 3. steer -> yaw
        dyaw = None
        if self.yaw_at_turn_start is not None and self.yaw_at_turn_end is not None:
            dyaw = self.yaw_at_turn_end - self.yaw_at_turn_start
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi
        emit("[omnidune] 3 STEER       : %s -- yaw change over the turn phase %s rad"
             % ("PASS" if (dyaw is not None and abs(dyaw) > 0.20) else "FAIL",
                ("%+.4f" % dyaw) if dyaw is not None else "n/a"))

        # 4. suspension travel
        ok_travel = True
        for c in CORNERS:
            lo, hi = self.strut_min[c], self.strut_max[c]
            if lo > 1e8 or hi < -1e8:
                ok_travel = False
                emit("[omnidune]   %s strut: UNAVAILABLE" % c)
                continue
            stroke = hi - lo
            margin = min(STRUT_TRAVEL - abs(lo), STRUT_TRAVEL - abs(hi))
            pinned = margin < 0.001
            if stroke < 0.005 or pinned:
                ok_travel = False
            emit("[omnidune]   %s strut: min %+7.2f mm  max %+7.2f mm  stroke %6.2f mm"
                 "  nearest stop %6.2f mm away%s"
                 % (c, lo * 1000.0, hi * 1000.0, stroke * 1000.0, margin * 1000.0,
                    "   <-- PINNED" if pinned else ""))
        emit("[omnidune] 4 SUSPENSION  : %s -- every strut strokes > 5 mm and none "
             "reaches a stop" % ("PASS" if ok_travel else "FAIL"))
        emit("[omnidune] ==============================================")
        sys.stdout.flush()


if __name__ == "__main__":
    OmniDuneDriver().run()
