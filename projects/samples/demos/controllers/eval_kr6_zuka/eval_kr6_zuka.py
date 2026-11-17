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

"""Replay zuka's own PTP sequence on a KR6 R900 sixx and measure where each
axis actually ended up.

The sequence is codes/krl-driver/example2.py verbatim -- four joint-space
PTP waypoints in degrees, A1 through A6:

    PTP(AXIS,  0, -90, 90, 0, 0, 0)     OV_PRO 30
    PTP(AXIS, 90, -90, 90, 0, 0, 0)     OV_PRO 30
    PTP(AXIS,-90, -90, 90, 0, 0, 0)     OV_PRO 50
    PTP(AXIS,  0, -90, 90, 0, 0, 0)     OV_PRO 50

plus the home pose the KRL driver itself commands on entry, which is the
same vector as waypoint 1:

    KRLDRIVER.src:  PTP {A1 0, A2 -90, A3 90, A4 0, A5 0, A6 0}

The axis limits the driver declares are checked against the ones the
description carries. What is reported per waypoint and per axis is
commanded, achieved, error, and whether it settled.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = os.path.join("O:", os.sep, "omnisim", ".tmp", "reply-evals-20260919",
                   "kr6_runs")

AXES = ["joint_a%d" % i for i in range(1, 7)]

# zuka codes/krl-driver/krldriver.py line 8, degrees
ZUKA_LIMITS = [(-170, 170), (-190, 45), (-120, 156), (-185, 185),
               (-120, 120), (-350, 350)]

# zuka codes/krl-driver/example2.py, degrees
SEQUENCE = [
    ("home  (KRLDRIVER.src)", [0, -90, 90, 0, 0, 0]),
    ("wp1   OV_PRO 30", [0, -90, 90, 0, 0, 0]),
    ("wp2   OV_PRO 30", [90, -90, 90, 0, 0, 0]),
    ("wp3   OV_PRO 50", [-90, -90, 90, 0, 0, 0]),
    ("wp4   OV_PRO 50", [0, -90, 90, 0, 0, 0]),
]

DWELL_S = 3.0            # per waypoint
SETTLE_EPS = 1e-4        # rad of motion over the last 10 samples


def main():
    robot = Supervisor()
    ms = int(robot.getBasicTimeStep())
    dt = ms / 1000.0

    motors, sensors = {}, {}
    for jn in AXES:
        m = robot.getDevice(jn + "_motor")
        s = robot.getDevice(jn + "_sensor")
        if m is None:
            print("[kr6] FATAL missing motor %s_motor" % jn, flush=True)
            sys.exit(1)
        if s is not None:
            s.enable(ms)
        motors[jn] = m
        sensors[jn] = s

    # What the description declares, so it can be compared with zuka's table.
    declared = {}
    for jn in AXES:
        m = motors[jn]
        try:
            declared[jn] = {
                "min_deg": round(math.degrees(m.getMinPosition()), 3),
                "max_deg": round(math.degrees(m.getMaxPosition()), 3),
                "max_velocity": round(m.getMaxVelocity(), 5),
                "max_torque": round(m.getMaxTorque(), 5),
            }
        except Exception as e:
            declared[jn] = {"error": repr(e)}

    rec = {"robot": "kuka_kr6r900sixx (ROS-Industrial description)",
           "sequence_source": "zuka codes/krl-driver/example2.py",
           "basic_time_step_ms": ms,
           "zuka_declared_limits_deg": {
               AXES[i]: list(ZUKA_LIMITS[i]) for i in range(6)},
           "description_declared": declared,
           "waypoints": []}

    def read():
        out = []
        for jn in AXES:
            s = sensors[jn]
            out.append(float(s.getValue()) if s is not None else float("nan"))
        return out

    robot.step(ms)
    for label, target_deg in SEQUENCE:
        tgt = [math.radians(v) for v in target_deg]
        for jn, q in zip(AXES, tgt):
            motors[jn].setPosition(q)
        hist = []
        t = 0.0
        while robot.step(ms) != -1 and t < DWELL_S:
            hist.append(read())
            t += dt
        ach = read()
        settled = []
        for i in range(6):
            tail = [h[i] for h in hist[-10:]] if len(hist) >= 10 else [ach[i]]
            settled.append(bool(max(tail) - min(tail) < SETTLE_EPS))
        wp = {"label": label, "axes": []}
        for i, jn in enumerate(AXES):
            err = ach[i] - tgt[i]
            lo, hi = ZUKA_LIMITS[i]
            wp["axes"].append({
                "joint": jn,
                "commanded_deg": round(target_deg[i], 4),
                "achieved_deg": round(math.degrees(ach[i]), 4),
                "error_deg": round(math.degrees(err), 4),
                "settled": settled[i],
                "inside_zuka_limits": bool(lo <= target_deg[i] <= hi),
            })
        rec["waypoints"].append(wp)
        print("[kr6] %s" % label, flush=True)
        for a in wp["axes"]:
            print("[kr6]   %-10s cmd %+8.2f  ach %+9.4f  err %+8.4f  "
                  "settled %s" % (a["joint"], a["commanded_deg"],
                                  a["achieved_deg"], a["error_deg"],
                                  a["settled"]), flush=True)

    # ---- OVER-LIMIT PROBE -------------------------------------------------
    # zuka's driver declares the axis envelope and then does not enforce it:
    # the range check in krldriver.py PTP() is commented out. So the question
    # is whether anything downstream clamps. Each axis is commanded 30 deg
    # past its own declared limit, one at a time, from the home pose.
    probe = []
    home = [0, -90, 90, 0, 0, 0]
    for i, jn in enumerate(AXES):
        lo, hi = ZUKA_LIMITS[i]
        over = hi + 30.0
        for j, q in zip(AXES, home):
            motors[j].setPosition(math.radians(q))
        t = 0.0
        while robot.step(ms) != -1 and t < 1.5:
            t += dt
        motors[jn].setPosition(math.radians(over))
        t = 0.0
        while robot.step(ms) != -1 and t < 3.0:
            t += dt
        ach = math.degrees(read()[i])
        probe.append({"joint": jn, "declared_max_deg": hi,
                      "commanded_deg": over, "achieved_deg": round(ach, 4),
                      "clamped_at_limit": bool(abs(ach - hi) < 1.0),
                      "went_past_limit": bool(ach > hi + 1.0)})
        print("[kr6] over-limit %-10s declared max %+7.1f  commanded %+7.1f  "
              "achieved %+9.3f  %s"
              % (jn, hi, over, ach,
                 "CLAMPED at the limit" if abs(ach - hi) < 1.0 else
                 ("WENT PAST the limit" if ach > hi + 1.0 else "neither")),
              flush=True)
    # and the same on the LOWER side: A2's -190 deg is real KUKA travel and
    # exceeds -pi, which is where a clamp would bite.
    for i, jn in enumerate(AXES):
        lo, hi = ZUKA_LIMITS[i]
        under = lo - 30.0
        for j, q in zip(AXES, home):
            motors[j].setPosition(math.radians(q))
        t = 0.0
        while robot.step(ms) != -1 and t < 1.5:
            t += dt
        motors[jn].setPosition(math.radians(under))
        t = 0.0
        while robot.step(ms) != -1 and t < 3.0:
            t += dt
        ach = math.degrees(read()[i])
        probe.append({"joint": jn, "declared_min_deg": lo,
                      "commanded_deg": under, "achieved_deg": round(ach, 4),
                      "stopped_at_declared": bool(abs(ach - lo) < 1.0),
                      "stopped_short_by_deg": round(ach - lo, 4)})
        print("[kr6] under-limit %-10s declared min %+7.1f  commanded %+7.1f  "
              "achieved %+9.3f  short by %+7.3f deg"
              % (jn, lo, under, ach, ach - lo), flush=True)
    rec["over_limit_probe"] = probe

    # worst case across the whole replay
    worst = max((abs(a["error_deg"]), a["joint"], w["label"])
                for w in rec["waypoints"] for a in w["axes"])
    rec["worst_error_deg"] = {"value": round(worst[0], 4), "joint": worst[1],
                              "at": worst[2]}
    rec["all_settled"] = all(a["settled"] for w in rec["waypoints"]
                             for a in w["axes"])

    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "replay.json"), "w") as fh:
        json.dump(rec, fh, indent=1)
    print("[kr6] worst |error| %.4f deg on %s at %s ; all settled %s"
          % (worst[0], worst[1], worst[2], rec["all_settled"]), flush=True)
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        os.makedirs(OUT, exist_ok=True)
        tb = traceback.format_exc()
        with open(os.path.join(OUT, "tb.txt"), "a") as fh:
            fh.write(tb)
        print("[kr6] FAILED " + tb, flush=True)
        raise
