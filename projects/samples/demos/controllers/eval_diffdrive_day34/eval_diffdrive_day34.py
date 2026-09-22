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

"""Does the Day 34 wall-clock odometry patch actually remove the cost of a
dropped packet?

Sibling of eval_diffdrive_odom.py, same world geometry, same frozen command
trace, same 20 Hz encoder stream. What changes is that four dead-reckoning
estimators now run side by side on that one stream, so the only differences
between them are the ones under test.

  pre_patch_L020   as published before the patch: hardcoded dt, L = 0.20 m
  pre_patch_L022   the same integrator with the kinematics file's L = 0.22 m
  day34_rate       the Day 34 patch as written: dt from the wall clock,
                   L = 0.22 m, theta advanced before x and y, theta wrapped
                   to (-pi, pi], and the wheel RATE carried by the packet
                   that arrives is held across the whole measured interval
  day34_counts     the Day 34 patch with one thing changed: the packet
                   carries cumulative encoder position instead of a rate, so
                   the interval's velocity is differenced across the gap

PACKET MODEL. Every 50 ms a telemetry packet is produced carrying both the
instantaneous wheel rate and the cumulative encoder position. A dropped packet
is simply never delivered; the estimators see the next one 100 ms after the
last, which is what the Day 34 jitter test is looking for.

DROP PLACEMENT is the second factor, and the interesting one:
  none    nothing dropped
  3.00    inside the steady gentle-curve segment, wheel rates constant
  4.00    straddling a command change -- the gap spans the switch from the
          gentle curve to the in-place pivot

Ground truth is the supervisor's own read of the body pose.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = r"O:\omnisim\.tmp\reply-evals-20260913\charan_day34"

# ---- published robot parameters (unchanged) --------------------------------
WHEEL_RADIUS = 0.05
TRACK_KIN = 0.22
TRACK_ODO = 0.20
MAX_WHEEL_RAD_S = 25.0

ODOM_PERIOD_TICKS = 10   # 10 x 5 ms = 50 ms = 20 Hz
NOMINAL_DT = 0.05        # the hardcoded dt of the pre-patch integrator
JITTER_THRESHOLD_S = 0.08  # the Day 34 warning threshold, verbatim

TRACE = [
    (0.0, 2.0, 0.5, 0.0, "pure forward (published test case 1)"),
    (2.0, 4.0, 0.3, 0.8, "gentle left curve"),
    (4.0, 5.0, 0.0, 2.0, "in-place pivot (published test case 2)"),
    (5.0, 7.0, 1.2, 4.0, "sharp curve, saturating (published test case 3)"),
    (7.0, 8.0, 0.0, 0.0, "stop"),
]
T_END = 8.0


def twist_to_wheels(v, w):
    """Verbatim reproduction of the published inverse kinematics."""
    v_left = v - (w * TRACK_KIN / 2.0)
    v_right = v + (w * TRACK_KIN / 2.0)
    w_left = v_left / WHEEL_RADIUS
    w_right = v_right / WHEEL_RADIUS
    saturated = False
    scale = 1.0
    max_requested = max(abs(w_left), abs(w_right))
    if max_requested > MAX_WHEEL_RAD_S:
        scale = MAX_WHEEL_RAD_S / max_requested
        w_left *= scale
        w_right *= scale
        saturated = True
    return w_left, w_right, saturated, scale


class PrePatchOdometry:
    """The integrator as published before Day 34.

    dt is an argument the caller hands in and the class holds no notion of
    wall time, so a packet that never arrives is simply never integrated.
    x and y advance on the OLD heading, then theta advances; theta is never
    wrapped.
    """

    def __init__(self, track_width):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.track_width = track_width
        self.updates = 0

    def update(self, v_left, v_right, dt):
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.track_width
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        self.updates += 1


class Day34Odometry:
    """The Day 34 RobustOdometry, reproduced as written.

    dt comes from a clock read at the moment the packet is handled, theta is
    advanced BEFORE x and y and is wrapped to (-pi, pi], and the track width
    is a constructor parameter set to the kinematics value.

    `midpoint=True` changes exactly one thing: x and y advance along the
    heading halfway through the interval rather than the heading at its end.
    """

    def __init__(self, wheel_radius, track_width, midpoint=False):
        self.midpoint = midpoint
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_radius = wheel_radius
        self.track_width = track_width
        self.first_update = True
        self.last_update_t = 0.0
        self.updates = 0
        self.jitter_warnings = []

    def update_pose(self, left_rad_s, right_rad_s, now_s):
        if self.first_update:
            self.last_update_t = now_s
            self.first_update = False
            return None
        dt = now_s - self.last_update_t
        self.last_update_t = now_s
        if dt > JITTER_THRESHOLD_S:
            self.jitter_warnings.append(
                {"t_s": round(now_s, 4), "dt_ms": round(dt * 1000.0, 3)})

        v_left = left_rad_s * self.wheel_radius
        v_right = right_rad_s * self.wheel_radius
        v_center = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.track_width

        heading_before = self.theta
        self.theta += omega * dt
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta <= -math.pi:
            self.theta += 2.0 * math.pi

        drive_heading = (heading_before + 0.5 * omega * dt
                         if self.midpoint else self.theta)
        self.x += v_center * math.cos(drive_heading) * dt
        self.y += v_center * math.sin(drive_heading) * dt
        self.updates += 1
        return dt


def rpy_of(node):
    r = node.getOrientation()
    pitch = math.asin(max(-1.0, min(1.0, -r[6])))
    roll = math.atan2(r[7], r[8])
    yaw = math.atan2(r[3], r[0])
    return roll, pitch, yaw


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def main():
    drop_at = os.environ.get("EVAL_DROP_AT", "none")   # "none" | "3.00" | "4.00"
    tag = os.environ.get("EVAL_TAG", "run1")
    drop_t = None if drop_at == "none" else float(drop_at)

    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    left = robot.getDevice("left_wheel_motor")
    right = robot.getDevice("right_wheel_motor")
    lsens = robot.getDevice("left_wheel_sensor")
    rsens = robot.getDevice("right_wheel_sensor")
    lsens.enable(dt_ms)
    rsens.enable(dt_ms)
    for m in (left, right):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)

    self_node = robot.getSelf()

    pre020 = PrePatchOdometry(TRACK_ODO)
    pre022 = PrePatchOdometry(TRACK_KIN)
    d34_rate = Day34Odometry(WHEEL_RADIUS, TRACK_KIN)
    d34_counts = Day34Odometry(WHEEL_RADIUS, TRACK_KIN)
    d34_mid = Day34Odometry(WHEEL_RADIUS, TRACK_KIN, midpoint=True)
    ESTIMATORS = ("pre020", "pre022", "d34_rate", "d34_counts", "d34_mid")

    samples = []
    dropped = []
    tick = 0
    t = 0.0
    seg_idx = -1
    commands_received = []

    robot.step(dt_ms)
    prev_tick_l = lsens.getValue()
    prev_tick_r = rsens.getValue()
    # encoder position at the last DELIVERED packet, for the counts variant
    last_delivered_l = prev_tick_l
    last_delivered_r = prev_tick_r
    # encoder position at the last nominal 50 ms slot boundary, dropped
    # or not, which is what the pre-patch integrator differenced
    prev_slot_l = prev_tick_l
    prev_slot_r = prev_tick_r
    last_odom_tick = 0

    x0 = self_node.getPosition()
    _, _, yaw0 = rpy_of(self_node)
    truth_yaw_cont = yaw0
    prev_raw_yaw = yaw0

    # prime the Day 34 clocks exactly as main() does with its init call
    for est in (d34_rate, d34_counts, d34_mid):
        est.update_pose(0.0, 0.0, 0.0)

    while robot.step(dt_ms) != -1 and t < T_END:
        t += dt
        tick += 1

        _, _, raw_yaw = rpy_of(self_node)
        truth_yaw_cont += wrap(raw_yaw - prev_raw_yaw)
        prev_raw_yaw = raw_yaw

        cur_l = lsens.getValue()
        cur_r = rsens.getValue()
        # instantaneous wheel rate over ONE tick: what a rate packet carries
        inst_l = (cur_l - prev_tick_l) / dt
        inst_r = (cur_r - prev_tick_r) / dt
        prev_tick_l, prev_tick_r = cur_l, cur_r

        v = w = 0.0
        seg = None
        for i, (ts, te, vv, ww, note) in enumerate(TRACE):
            if ts <= t < te:
                v, w, seg = vv, ww, i
                break
        if seg is not None and seg != seg_idx:
            seg_idx = seg
            wl_c, wr_c, sat, sc = twist_to_wheels(v, w)
            commands_received.append({
                "t_s": round(t, 3), "segment": seg, "note": TRACE[seg][4],
                "v_mps": v, "w_radps": w,
                "wheel_left_radps_cmd": round(wl_c, 4),
                "wheel_right_radps_cmd": round(wr_c, 4),
                "saturated": sat, "saturation_scale": round(sc, 4),
            })

        wl, wr, sat, sc = twist_to_wheels(v, w)
        left.setVelocity(wl)
        right.setVelocity(wr)

        if tick - last_odom_tick >= ODOM_PERIOD_TICKS:
            last_odom_tick = tick
            is_drop = (drop_t is not None
                       and not dropped
                       and t >= drop_t - 1e-9)

            # mean wheel rate over the nominal 50 ms slot, which is what the
            # original evaluation fed the pre-patch integrator
            slot_l = (cur_l - prev_slot_l) / NOMINAL_DT
            slot_r = (cur_r - prev_slot_r) / NOMINAL_DT
            prev_slot_l, prev_slot_r = cur_l, cur_r

            if is_drop:
                dropped.append({
                    "t_s": round(t, 4),
                    "inst_left_radps": round(inst_l, 4),
                    "inst_right_radps": round(inst_r, 4),
                    "slot_left_radps": round(slot_l, 4),
                    "slot_right_radps": round(slot_r, 4),
                    "cmd_v": v, "cmd_w": w,
                })
                # nothing is delivered: no estimator advances, and the count
                # variants keep their reference where it was
            else:
                # --- pre-patch pair: handed its hardcoded dt, so a slot that
                #     never arrives is never accounted for anywhere
                pre020.update(slot_l * WHEEL_RADIUS, slot_r * WHEEL_RADIUS,
                              NOMINAL_DT)
                pre022.update(slot_l * WHEEL_RADIUS, slot_r * WHEEL_RADIUS,
                              NOMINAL_DT)

                # --- Day 34 as written, deployed with RATE telemetry: the
                #     rate in the packet that arrives is held across the whole
                #     measured interval
                used_dt = d34_rate.update_pose(inst_l, inst_r, t)

                # --- Day 34 fed cumulative encoder COUNTS instead of a rate
                gap = t - d34_counts.last_update_t
                if gap > 1e-9:
                    mean_l = (cur_l - last_delivered_l) / gap
                    mean_r = (cur_r - last_delivered_r) / gap
                else:
                    mean_l, mean_r = inst_l, inst_r
                d34_counts.update_pose(mean_l, mean_r, t)
                d34_mid.update_pose(mean_l, mean_r, t)

                last_delivered_l, last_delivered_r = cur_l, cur_r

                p = self_node.getPosition()
                samples.append({
                    "t_s": round(t, 4),
                    "cmd_v": v, "cmd_w": w,
                    "dt_used_s": round(used_dt, 5) if used_dt else None,
                    "inst_wheel": [round(inst_l, 4), round(inst_r, 4)],
                    "slot_wheel": [round(slot_l, 4), round(slot_r, 4)],
                    "mean_wheel": [round(mean_l, 4), round(mean_r, 4)],
                    "truth": [round(p[0], 5), round(p[1], 5),
                              round(truth_yaw_cont, 5)],
                    "pre020": [round(pre020.x, 5), round(pre020.y, 5),
                               round(pre020.theta, 5)],
                    "pre022": [round(pre022.x, 5), round(pre022.y, 5),
                               round(pre022.theta, 5)],
                    "d34_rate": [round(d34_rate.x, 5), round(d34_rate.y, 5),
                                 round(d34_rate.theta, 5)],
                    "d34_counts": [round(d34_counts.x, 5),
                                   round(d34_counts.y, 5),
                                   round(d34_counts.theta, 5)],
                    "d34_mid": [round(d34_mid.x, 5), round(d34_mid.y, 5),
                                round(d34_mid.theta, 5)],
                })

    p = self_node.getPosition()
    truth = (p[0], p[1], truth_yaw_cont)

    def err(o, wrapped):
        th = o.theta
        ref = wrap(truth[2]) if wrapped else truth[2]
        dth = wrap(th - ref) if wrapped else (th - ref)
        return {
            "x": round(o.x - truth[0], 5),
            "y": round(o.y - truth[1], 5),
            "euclidean_xy": round(math.hypot(o.x - truth[0],
                                             o.y - truth[1]), 5),
            "theta_rad": round(dth, 5),
            "theta_deg": round(math.degrees(dth), 3),
            "heading_compared": "wrapped" if wrapped else "continuous",
        }

    OBJ = {"pre020": pre020, "pre022": pre022, "d34_rate": d34_rate,
           "d34_counts": d34_counts, "d34_mid": d34_mid}

    result = {
        "drop_at": drop_at,
        "tag": tag,
        "basic_time_step_ms": dt_ms,
        "odom_rate_hz": round(1.0 / (ODOM_PERIOD_TICKS * dt), 3),
        "jitter_threshold_s": JITTER_THRESHOLD_S,
        "commands_received": commands_received,
        "dropped_packets": dropped,
        "updates_applied": {k: o.updates for k, o in OBJ.items()},
        "final_truth": {
            "x": round(truth[0], 5), "y": round(truth[1], 5),
            "theta_rad_continuous": round(truth[2], 5),
            "theta_deg_continuous": round(math.degrees(truth[2]), 3),
            "theta_deg_wrapped": round(math.degrees(wrap(truth[2])), 3),
        },
        "final": {k: [round(o.x, 5), round(o.y, 5),
                      round(math.degrees(o.theta), 3)]
                  for k, o in OBJ.items()},
        "error": {k: err(o, k.startswith("d34")) for k, o in OBJ.items()},
        "jitter_warnings": {"d34_rate": d34_rate.jitter_warnings,
                            "d34_counts": d34_counts.jitter_warnings,
                            "d34_mid": d34_mid.jitter_warnings},
        "samples": samples,
    }

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"day34_drop{drop_at}_{tag}.json")
    with open(path, "w") as f:
        json.dump(result, f, indent=1)
    print(f"[eval] wrote {path}", flush=True)
    e = result["error"]
    print(f"[eval] drop_at={drop_at} truth=({truth[0]:.4f},{truth[1]:.4f},"
          f"{math.degrees(truth[2]):.2f}deg)", flush=True)
    for k in ESTIMATORS:
        print(f"[eval]   {k:11s} xy_err={e[k]['euclidean_xy']:.5f} m  "
              f"theta_err={e[k]['theta_deg']:.3f} deg", flush=True)
    print(f"[eval] jitter warnings fired: "
          f"{len(d34_rate.jitter_warnings)}", flush=True)
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        os.makedirs(OUT, exist_ok=True)
        tb = traceback.format_exc()
        with open(os.path.join(OUT, "controller_traceback.txt"), "a") as f:
            f.write(tb)
            f.write("=" * 60)
        print("[eval] CONTROLLER FAILED: " + tb, flush=True)
        raise
