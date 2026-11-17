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

"""OPEN-LOOP replay of the DDBot `examples/Square/Square.ino` command sequence.

THE LOOP IS OPEN. There is no feedback of any kind in this controller:

  * no supervisor-driven steering correction,
  * no odometry-driven correction,
  * no "drive until distance reached" primitive,
  * no per-corner re-aiming.

The wheels are commanded to a fixed velocity for a fixed number of simulation
ticks and then re-commanded, exactly as an Arduino sketch calling
`bot.forward(100); delay(2000); bot.right(100); delay(1000);` would. The
supervisor handle is used ONLY to READ ground-truth pose for the measurement,
never to write a correction.

DDBot's `right(speed)` is `writeDirections(LOW, LOW, LOW, HIGH, speed)`:
  directionPins[0] left-forward  = LOW
  directionPins[1] left-backward = LOW
  directionPins[2] right-forward = LOW
  directionPins[3] right-backward= HIGH
i.e. the LEFT motor is not driven and the RIGHT motor is driven BACKWARD.
That is a pivot about the left wheel, not a spin in place (`clockwise()` is
the spin). We model the undriven left motor as HELD AT ZERO VELOCITY -- see
--left-mode, which also offers a free-rolling ("coast") variant.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

from omnisim import Supervisor


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--wheel-rad-s", type=float, required=True,
                    help="commanded wheel angular velocity for speed=100")
    ap.add_argument("--forward-s", type=float, default=2.0)
    ap.add_argument("--turn-s", type=float, default=1.0)
    ap.add_argument("--laps", type=int, default=4)
    ap.add_argument("--settle-s", type=float, default=1.5)
    ap.add_argument("--tail-s", type=float, default=2.0)
    ap.add_argument("--left-mode", default="hold", choices=["hold", "coast"])
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    L = robot.getDevice("wheel_left_joint_motor")
    R = robot.getDevice("wheel_right_joint_motor")
    Ls = robot.getDevice("wheel_left_joint_sensor")
    Rs = robot.getDevice("wheel_right_joint_sensor")
    if L is None or R is None:
        print("[ddbot] FATAL: wheel motors not found", flush=True)
        return 2
    have_enc = Ls is not None and Rs is not None
    if have_enc:
        Ls.enable(dt_ms)
        Rs.enable(dt_ms)

    # continuous joints -> velocity mode
    for m in (L, R):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)

    devinfo = {}
    for nm, m in (("left", L), ("right", R)):
        d = {}
        for attr in ("getMaxVelocity", "getMaxTorque", "getAvailableTorque"):
            try:
                d[attr] = float(getattr(m, attr)())
            except Exception:
                d[attr] = None
        devinfo[nm] = d

    self_node = robot.getSelf()

    def truth():
        p = self_node.getPosition()
        o = self_node.getOrientation()   # row-major 3x3, Z-up
        return (p[0], p[1], p[2],
                math.atan2(o[3], o[0]),
                math.acos(max(-1.0, min(1.0, o[8]))))

    def enc():
        if not have_enc:
            return (None, None)
        a, b = Ls.getValue(), Rs.getValue()
        if a != a or b != b:   # NaN before first enabled read
            return (None, None)
        return (a, b)

    W = args.wheel_rad_s
    n_fwd = int(round(args.forward_s / dt))
    n_turn = int(round(args.turn_s / dt))
    n_settle = int(round(args.settle_s / dt))
    n_tail = int(round(args.tail_s / dt))

    trace = []
    marks = []
    yaw_acc = 0.0
    prev_yaw = None
    t = 0.0
    started = False
    start_state = None

    def record(phase, idx):
        nonlocal yaw_acc, prev_yaw
        x, y, z, yaw, tilt = truth()
        if prev_yaw is None:
            prev_yaw = yaw
        yaw_acc += wrap_pi(yaw - prev_yaw)
        prev_yaw = yaw
        el, er = enc()
        trace.append({"t": round(t, 6), "phase": phase, "i": idx,
                      "x": x, "y": y, "z": z, "yaw": yaw,
                      "yaw_unwrapped": yaw_acc, "tilt": tilt,
                      "enc_l": el, "enc_r": er})

    def run(n, vl, vr, phase, idx):
        nonlocal t
        for _ in range(n):
            L.setVelocity(vl)
            if args.left_mode == "coast" and vl == 0.0:
                # best-effort free-roll: zero the available torque so the
                # velocity servo cannot hold the wheel
                try:
                    L.setAvailableTorque(0.0)
                except Exception:
                    pass
            else:
                try:
                    L.setAvailableTorque(devinfo["left"]["getMaxTorque"] or 10.0)
                except Exception:
                    pass
            R.setVelocity(vr)
            if robot.step(dt_ms) == -1:
                return False
            t += dt
            record(phase, idx)
        return True

    # ---- settle: wheels commanded zero, robot drops onto the floor --------
    if not run(n_settle, 0.0, 0.0, "settle", 0):
        return 1
    x, y, z, yaw, tilt = truth()
    el, er = enc()
    start_state = {"x": x, "y": y, "z": z, "yaw": yaw,
                   "yaw_unwrapped": yaw_acc, "tilt": tilt,
                   "enc_l": el, "enc_r": er, "t": t}
    started = True
    marks.append({"event": "square_start", "t": t, "x": x, "y": y, "yaw": yaw,
                  "yaw_unwrapped": yaw_acc, "enc_l": el, "enc_r": er})

    # ---- the DDBot Square loop, played open loop --------------------------
    for lap in range(1, args.laps + 1):
        if not run(n_fwd, W, W, "forward", lap):
            break
        x, y, z, yaw, tilt = truth()
        el, er = enc()
        marks.append({"event": "forward_end", "lap": lap, "t": t,
                      "x": x, "y": y, "yaw": yaw, "yaw_unwrapped": yaw_acc,
                      "enc_l": el, "enc_r": er})
        # right(): left motor undriven, right motor backward
        if not run(n_turn, 0.0, -W, "right", lap):
            break
        x, y, z, yaw, tilt = truth()
        el, er = enc()
        marks.append({"event": "turn_end", "lap": lap, "t": t,
                      "x": x, "y": y, "yaw": yaw, "yaw_unwrapped": yaw_acc,
                      "enc_l": el, "enc_r": er})

    # ---- stop() and let the base come to rest -----------------------------
    run(n_tail, 0.0, 0.0, "stop", 0)
    x, y, z, yaw, tilt = truth()
    el, er = enc()
    end_state = {"x": x, "y": y, "z": z, "yaw": yaw,
                 "yaw_unwrapped": yaw_acc, "tilt": tilt,
                 "enc_l": el, "enc_r": er, "t": t}
    marks.append({"event": "square_end", "t": t, "x": x, "y": y, "yaw": yaw,
                  "yaw_unwrapped": yaw_acc, "enc_l": el, "enc_r": er})

    out = {
        "tag": args.tag,
        "open_loop": True,
        "feedback_used": "none",
        "basic_time_step_ms": dt_ms,
        "commanded_wheel_rad_s": W,
        "forward_s": args.forward_s,
        "turn_s": args.turn_s,
        "laps": args.laps,
        "left_mode": args.left_mode,
        "ticks": {"forward": n_fwd, "turn": n_turn,
                  "settle": n_settle, "tail": n_tail},
        "devices": devinfo,
        "encoders_available": have_enc,
        "start": start_state,
        "end": end_state,
        "marks": marks,
        "trace": trace,
    }
    tmp = args.out + ".part"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(out, f)
    os.replace(tmp, args.out)
    print("[ddbot] WROTE %s  n_trace=%d" % (args.out, len(trace)), flush=True)
    sys.stdout.flush()

    # idle out the rest of the run without touching the wheels
    while robot.step(dt_ms) != -1:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
