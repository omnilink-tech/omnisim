#!/usr/bin/env python3
# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""RoboLife robot program: one process per Husky, exactly like a real robot.

The robot owns its devices (the supervisor cannot drive a velocity wheel or
read a device it does not own) and talks to the supervisor over the modelled
radio link -- the Robot node's ``customData`` field: every BUS_EVERY ticks it
reads the supervisor's JSON with ``getCustomData()`` and answers with
``setCustomData()`` in DESIGN.md's shape. All decisions are made by the pure,
unit-tested ``rl/brain.py``; this file only measures, applies and logs.

MEASURED, never echoed. ``v`` / ``w`` in the status are differenced from the
GPS position and the IMU yaw -- the wheel command is never reported as the
velocity. The IMU->world yaw offset is CALIBRATED from GPS motion, not assumed
(the URDF's imu_link may carry a rotation), and the brain's yaw-rate gain is
learned from the measured rate. Wheel PositionSensors, when present, give an
independent odometry twist that is logged next to the GPS one so slip is
visible in the log.

Devices (names from DESIGN.md / the urdf_import expansion):
    GPS "navsat_fix", InertialUnit "imu_data", Lidar "lidar",
    Connector "socket_front" / "socket_rear",
    RotationalMotor front_left/front_right/rear_left/rear_right_wheel_motor
    (+ optional PositionSensor "*_wheel_sensor").
Args: ``--slot <i>`` (falls back to the trailing integer of the robot name).
"""
import json
import math
import os
import sys
import time

from omnisim import Robot

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOLIFE = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROBOLIFE)
from rl import brain as B  # noqa: E402

BUS_EVERY = 5                 # ticks between customData read + status write
LOG_EVERY_S = 10.0            # periodic commanded-vs-achieved line
YAW_CAL_SAMPLES = 150         # GPS-motion samples used to calibrate the IMU yaw offset
YAW_CAL_MIN_SPEED = 0.15      # m/s of GPS ground speed before a sample counts
VEL_ALPHA = 0.3               # EMA on the differenced GPS/IMU velocities
WHEEL_MOTORS = ("front_left_wheel_motor", "front_right_wheel_motor",
                "rear_left_wheel_motor", "rear_right_wheel_motor")
LEFT_MOTORS = ("front_left_wheel_motor", "rear_left_wheel_motor")
RIGHT_MOTORS = ("front_right_wheel_motor", "rear_right_wheel_motor")


def parse_slot(argv, fallback_name=""):
    for i, a in enumerate(argv):
        if a == "--slot" and i + 1 < len(argv):
            try:
                return int(argv[i + 1])
            except ValueError:
                pass
        if a.startswith("--slot="):
            try:
                return int(a.split("=", 1)[1])
            except ValueError:
                pass
    tail = fallback_name.rsplit("_", 1)[-1] if fallback_name else ""
    return int(tail) if tail.isdigit() else -1


robot = Robot()
DT = int(robot.getBasicTimeStep())
DT_S = DT / 1000.0
SLOT = parse_slot(sys.argv, getattr(robot, "getName", lambda: "")() or "")
TAG = "[robot %d]" % SLOT


def log(msg):
    print("%s %s" % (TAG, msg), flush=True)


# ------------------------------------------------------------------ devices
missing = []


def dev(name, required=True):
    d = robot.getDevice(name)
    if d is None:
        missing.append(name)
        log("%s device '%s' NOT FOUND" % ("FATAL" if required else "WARNING", name))
    return d


gps = dev("navsat_fix")
imu = dev("imu_data")
lidar = dev("lidar", required=False)
socket_front = dev("socket_front", required=False)
socket_rear = dev("socket_rear", required=False)
motors = {n: dev(n) for n in WHEEL_MOTORS}
wheel_sensors = {}
for n in WHEEL_MOTORS:
    s = robot.getDevice(n.replace("_motor", "_sensor"))
    if s is not None:
        wheel_sensors[n] = s

fatal = [n for n in missing if n in WHEEL_MOTORS or n in ("navsat_fix", "imu_data")]
if fatal:
    log("FATAL cannot run without %s -- check husky_base.txt device names; exiting 1"
        % ", ".join(fatal))
    sys.exit(1)

for m in motors.values():
    m.setPosition(float("inf"))
    m.setVelocity(0.0)
gps.enable(DT)
imu.enable(DT)
radio = robot.getDevice("radio")
if radio is not None:
    radio.enable(DT)
else:
    log("WARNING: no Receiver 'radio' -- this robot will never hear the supervisor")
for s in wheel_sensors.values():
    s.enable(DT)
LIDAR_FOV = None
if lidar is not None:
    lidar.enable(DT)
    LIDAR_FOV = float(lidar.getFov())
    log("lidar: %d columns, fov %.3f rad" % (lidar.getHorizontalResolution(), LIDAR_FOV))
else:
    log("WARNING no lidar -> driving without the obstacle guard")
for name, sock in (("socket_front", socket_front), ("socket_rear", socket_rear)):
    if sock is not None:
        sock.enablePresence(DT)
    else:
        log("WARNING no %s -> that socket can never dock" % name)
log("slot %d, dt %d ms, wheel sensors %d/4, devices missing: %s"
    % (SLOT, DT, len(wheel_sensors), missing or "none"))

brain = B.Brain(SLOT)

# ------------------------------------------------------------------ measurement state
prev_xy = None
prev_yaw_imu = None
prev_wheel_pos = {}
yaw_offset = 0.0              # world yaw = imu yaw + yaw_offset (calibrated)
yaw_cal_n = 0
yaw_cal_acc = [0.0, 0.0]      # unit-vector accumulator of the observed offset
v_meas = w_meas = 0.0         # EMA of the GPS / IMU differenced twist
v_meas_raw = w_meas_raw = 0.0
last_state = brain.state
last_phase = None
last_bus = B.parse_bus("")
tick = 0
next_log_t = LOG_EVERY_S
cmd_v = cmd_w = 0.0
stat = {"bus_reads": 0, "bus_bad": 0, "locks": 0, "unlocks": 0}


def set_wheels(v, w):
    left, right = B.saturate_wheels(*B.diff_drive(v, w))
    for n in LEFT_MOTORS:
        motors[n].setVelocity(left)
    for n in RIGHT_MOTORS:
        motors[n].setVelocity(right)
    return left, right


def wheel_twist():
    """Odometry twist from the wheel PositionSensors (None without them)."""
    global prev_wheel_pos
    if len(wheel_sensors) < 4:
        return None
    cur = {n: s.getValue() for n, s in wheel_sensors.items()}
    if not prev_wheel_pos:
        prev_wheel_pos = cur
        return None
    rates = {n: (cur[n] - prev_wheel_pos[n]) / DT_S for n in cur}
    prev_wheel_pos = cur
    left = 0.5 * (rates["front_left_wheel_motor"] + rates["rear_left_wheel_motor"])
    right = 0.5 * (rates["front_right_wheel_motor"] + rates["rear_right_wheel_motor"])
    return B.wheel_speeds_to_twist(left, right)


def connector_action(sock, name, want_lock):
    """lock()/unlock() and report what the device says afterwards."""
    if sock is None:
        log("WARNING brain asked to %s %s but the device is missing"
            % ("lock" if want_lock else "unlock", name))
        return
    presence = sock.getPresence()
    if want_lock:
        sock.lock()
        stat["locks"] += 1
    else:
        sock.unlock()
        stat["unlocks"] += 1
    locked = sock.isLocked()
    log("%s %s: presence=%d isLocked=%s (measured after the call)"
        % ("LOCK" if want_lock else "UNLOCK", name, presence, locked))
    if want_lock and not locked:
        log("WARNING lock() on %s did not take (presence was %d)" % (name, presence))


# ------------------------------------------------------------------ main loop
log("running: state %s" % brain.state)
t_wall0 = time.perf_counter()
while robot.step(DT) != -1:
    tick += 1
    t = robot.getTime()

    # -- sense --------------------------------------------------------
    gx, gy = gps.getValues()[0:2]
    rpy = imu.getRollPitchYaw()
    yaw_imu = float(rpy[2])
    if any(math.isnan(x) for x in (gx, gy, yaw_imu)):
        if tick % 125 == 1:
            log("WARNING GPS/IMU returned NaN (gps %r yaw %r) -- holding" % ((gx, gy), yaw_imu))
        set_wheels(0.0, 0.0)
        continue

    if prev_xy is not None:
        dx, dy = gx - prev_xy[0], gy - prev_xy[1]
        speed = math.hypot(dx, dy) / DT_S
        # calibrate the IMU->world yaw offset from forward GPS motion
        if (yaw_cal_n < YAW_CAL_SAMPLES and speed > YAW_CAL_MIN_SPEED and cmd_v > 0.1
                and brain.state in ("explore", "seek_module", "seek_charge")):
            obs = B.wrap_pi(math.atan2(dy, dx) - yaw_imu)
            yaw_cal_acc[0] += math.cos(obs)
            yaw_cal_acc[1] += math.sin(obs)
            yaw_cal_n += 1
            new_off = math.atan2(yaw_cal_acc[1], yaw_cal_acc[0])
            if yaw_cal_n in (10, YAW_CAL_SAMPLES):
                log("yaw calibration: imu->world offset %.3f rad after %d samples"
                    % (new_off, yaw_cal_n))
            yaw_offset = new_off
        yaw = B.wrap_pi(yaw_imu + yaw_offset)
        v_meas_raw = (dx * math.cos(yaw) + dy * math.sin(yaw)) / DT_S
        w_meas_raw = B.wrap_pi(yaw_imu - prev_yaw_imu) / DT_S
        v_meas += VEL_ALPHA * (v_meas_raw - v_meas)
        w_meas += VEL_ALPHA * (w_meas_raw - w_meas)
    else:
        yaw = B.wrap_pi(yaw_imu + yaw_offset)
    prev_xy = (gx, gy)
    prev_yaw_imu = yaw_imu
    pose = (gx, gy, yaw)

    ranges = None
    if lidar is not None:
        img = lidar.getRangeImage()
        if img:
            ranges = list(img)
    lidar_arg = (ranges, LIDAR_FOV) if ranges else None
    pf = socket_front.getPresence() if socket_front is not None else 0
    pr = socket_rear.getPresence() if socket_rear is not None else 0

    # -- bus in -------------------------------------------------------
    if tick % BUS_EVERY == 1:
        # Radio in: drain the queue, keep the newest packet addressed to us.
        # (customData is the reply channel only -- sharing it both ways made
        # the robot read its own status back instead of the bus.)
        raw = ""
        if radio is not None:
            while radio.getQueueLength() > 0:
                pkt = radio.getString()
                radio.nextPacket()
                if pkt.startswith('{"slot":%d,' % SLOT) or ('"slot": %d,' % SLOT) in pkt[:16]:
                    raw = pkt
        stat["bus_reads"] += 1
        if raw:
            parsed = B.parse_bus(raw)
            if parsed["t"] == 0.0 and parsed["modules"] == [] and raw.strip()[:1] != "{":
                stat["bus_bad"] += 1
                if stat["bus_bad"] in (1, 100):
                    log("WARNING customData is not a JSON object (%r...)" % raw[:60])
            last_bus = parsed

    # -- think --------------------------------------------------------
    out = brain.step(last_bus, pose, (v_meas, w_meas), lidar_arg, pf, pr, t)

    # -- act ----------------------------------------------------------
    cmd_v, cmd_w = out["v"], out["w"]
    set_wheels(cmd_v, cmd_w)
    if out["unlock_front"]:
        connector_action(socket_front, "socket_front", False)
    if out["unlock_rear"]:
        connector_action(socket_rear, "socket_rear", False)
    if out["lock_front"]:
        connector_action(socket_front, "socket_front", True)
    if out["lock_rear"]:
        connector_action(socket_rear, "socket_rear", True)

    # -- log ----------------------------------------------------------
    if out["note"]:
        log("t=%.2f %s | pose (%.2f, %.2f, %.2f) batt %.2f"
            % (t, out["note"], gx, gy, yaw, last_bus["batt"]))
    if out["state"] != last_state or out.get("phase") != last_phase:
        log("t=%.2f state %s -> %s%s target=%s docked=%s"
            % (t, last_state, out["state"],
               (" [%s]" % out["phase"]) if out.get("phase") else "",
               out["target"], brain.docked))
        last_state, last_phase = out["state"], out.get("phase")
    if t >= next_log_t:
        next_log_t += LOG_EVERY_S
        odo = wheel_twist()
        wall = (time.perf_counter() - t_wall0) / max(tick, 1) * 1000.0
        log("t=%.1f %s cmd v %.2f w %.2f | gps/imu v %.2f w %.2f | wheel-odo %s | "
            "w_gain %.2f yaw_off %.3f | lidar_min %s | %.3f ms/tick wall | bus %d reads %d bad"
            % (t, out["state"], cmd_v, cmd_w, v_meas, w_meas,
               "n/a" if odo is None else "v %.2f w %.2f" % odo,
               brain.w_gain, yaw_offset,
               "%.2f" % B.lidar_min(ranges) if B.lidar_min(ranges) is not None else "none",
               wall, stat["bus_reads"], stat["bus_bad"]))
    elif wheel_sensors:
        wheel_twist()   # keep the odometry difference fresh even when not logging

    # -- bus out ------------------------------------------------------
    if tick % BUS_EVERY == 0:
        status = brain.status(v_meas, w_meas, B.lidar_min(ranges))
        robot.setCustomData(json.dumps(status, separators=(",", ":")))

log("step returned -1 -> exiting (%d ticks, %d locks, %d unlocks)"
    % (tick, stat["locks"], stat["unlocks"]))
