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

"""The desk robot's drive test: full speed ahead, a stop, a spin in place, measured.

Each motor is the Pololu #3065's DC characteristic at full voltage: every step the
available torque is set to stall x |1 - |speed| / no-load speed|, and the velocity is
commanded at the no-load speed, so the wheel settles where the motor's torque meets the
load, as a DC motor on a battery does -- driving below no-load speed, braking above it. The supervisor reads the robot's own pose.

controllerArgs: [--result <json>] [--quit]  (default .tmp/desk_robot/drive.json)
"""
import argparse
import json
import math
from pathlib import Path

from omnisim import Supervisor

ROOT = Path(__file__).resolve().parents[5]
NO_LOAD_RAD_S = 330 * 2 * math.pi / 60          # Pololu #3065, 6 V
STALL_NM = 1.6 * 9.81 / 100                      # 1.6 kg cm
PHASES = [("settle", 0.5, 0.0, 0.0), ("straight", 1.2, 1.0, 1.0), ("stop", 1.0, 0.0, 0.0),
          ("spin", 3.0, -1.0, 1.0), ("stop2", 1.0, 0.0, 0.0)]


def available_torque(u, w):
    """The torque the motor can apply at wheel speed w (rad/s) when commanded u (-1..1).

    Both branches of the DC line: below no-load the motor drives, above it the back-EMF
    exceeds the battery and it BRAKES; the servo targets no-load speed, so the torque it
    is allowed acts toward it either way. Clamped at zero (as this was until 2026-11-03)
    an over-speed wheel goes FREE: setAvailableTorque(0) is a passive joint. In the spin
    one wheel crossed no-load, went limp, and the other drove the robot round it until
    the free wheel was back-driven to 58 rad/s, 1.7x a speed its motor can reach.
    """
    return STALL_NM * abs(1.0 - abs(w) / NO_LOAD_RAD_S) if u else STALL_NM


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--result", default=str(ROOT / ".tmp/desk_robot/drive.json"))
    p.add_argument("--quit", action="store_true", help="end the simulation once the result is written")
    p.add_argument("--loop", action="store_true",
                   help="for watching: after each run, put the robot back where it started and drive again "
                        "(the result is written after the first run only)")
    a = p.parse_args()
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0
    motors = [robot.getDevice("left_motor"), robot.getDevice("right_motor")]
    sensors = [robot.getDevice("left_sensor"), robot.getDevice("right_sensor")]
    for m, s in zip(motors, sensors):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        s.enable(dt_ms)
    me = robot.getSelf()
    start_pose = (list(me.getField("translation").getSFVec3f()), list(me.getField("rotation").getSFRotation()))
    first = True
    while True:
        t = 0.0
        last = [None, None]
        samples = []
        phase_i, phase_t = 0, 0.0
        while robot.step(dt_ms) != -1 and phase_i < len(PHASES):
            t += dt
            name, length, ul, ur = PHASES[phase_i]
            angles = [s.getValue() for s in sensors]
            omega = [0.0 if last[i] is None else (angles[i] - last[i]) / dt for i in range(2)]
            last = angles
            for m, u, w in zip(motors, (ul, ur), omega):
                # capped at the motor's own limits: the world stores them rounded, so the
                # datasheet value can read as just over them
                m.setAvailableTorque(min(available_torque(u, w), m.getMaxTorque()))
                m.setVelocity(max(-m.getMaxVelocity(), min(u * NO_LOAD_RAD_S, m.getMaxVelocity())))
            pos = me.getPosition()
            R = me.getOrientation()
            yaw = math.atan2(R[3], R[0])
            samples.append((t, name, pos[0], pos[1], pos[2], yaw, omega[0], omega[1]))
            phase_t += dt
            if phase_t >= length - 1e-9:
                phase_i, phase_t = phase_i + 1, 0.0

        def window(name, tail=1.0):
            rows = [s for s in samples if s[1] == name]
            end = rows[-1][0]
            return [s for s in rows if s[0] >= end - tail]

        st = window("straight")
        v = math.hypot(st[-1][2] - st[0][2], st[-1][3] - st[0][3]) / (st[-1][0] - st[0][0])
        w_wheel = sum(abs(s[6]) + abs(s[7]) for s in st) / (2 * len(st))
        sp = window("spin")
        yaw = [s[5] for s in sp]
        turned = sum(math.remainder(b - a, 2 * math.pi) for a, b in zip(yaw[:-1], yaw[1:]))
        yaw_rate = turned / (sp[-1][0] - sp[0][0])
        w_spin = sum(abs(s[6]) + abs(s[7]) for s in sp) / (2 * len(sp))
        z = [s[4] for s in samples if s[1] == "straight"]
        out = {
            "straight_speed_m_s": round(v, 4), "straight_wheel_rad_s": round(w_wheel, 3),
            "straight_effective_radius_mm": round(1000 * v / w_wheel, 3) if w_wheel else None,
            "spin_yaw_rate_rad_s": round(yaw_rate, 4), "spin_wheel_rad_s": round(w_spin, 3),
            "spin_wheel_peak_rad_s": round(max(max(abs(s[6]), abs(s[7])) for s in sp), 3),
            "spin_effective_track_mm": round(2000 * (v / w_wheel if w_wheel else 0) * w_spin / abs(yaw_rate), 2) if yaw_rate else None,
            "chassis_z_range_mm": [round(1000 * min(z), 3), round(1000 * max(z), 3)],
            "heading_drift_deg_straight": round(math.degrees(math.remainder(st[-1][5] - st[0][5], 2 * math.pi)), 3),
            "dt_ms": dt_ms,
        }
        if first:
            path = Path(a.result)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(out, indent=1), encoding="utf-8")
            print("[desk_robot_drive] " + json.dumps(out), flush=True)
        for m in motors:
            m.setVelocity(0.0)
        if not a.loop:
            break
        first = False
        for _ in range(int(1000 / dt_ms)):                 # a second's pause, then from the start again
            if robot.step(dt_ms) == -1:
                return
        me.getField("translation").setSFVec3f(start_pose[0])
        me.getField("rotation").setSFRotation(start_pose[1])
        me.resetPhysics()
    if a.quit:
        robot.simulationQuit(0)
    while robot.step(dt_ms) != -1:
        pass


if __name__ == "__main__":
    main()
