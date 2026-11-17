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

"""Velocity-command + odometry loop evaluation.

Reproduces, unmodified, three pieces published in a 100-day robotics log:

  1. twist -> wheel inverse kinematics   (r=0.05, L=0.22, cap 25 rad/s,
     proportional saturation scaling that preserves curvature)
  2. differential-drive dead-reckoning   (track_width = 0.20, Euler integration)
  3. a heartbeat failsafe state machine  (warn 150 ms, critical 350 ms)

A frozen command trace is replayed on real physics. Wheel encoders feed the
dead-reckoning. Exactly one odometry update is dropped in the DROP run and none
in the BASELINE run, so the dropped sample is the only factor that changes.

Four dead-reckoning estimators run on the same encoder stream:
  odo_L020        his odometry file's track width
  odo_L022        the kinematics file's track width
  (each is also run in the drop and no-drop condition, via two engine runs)

Ground truth comes from the supervisor, which is what the published code has no
way to obtain on its own.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = r"O:\omnisim\social\launch\pilots\replies_2026_09_12\hundred_days_robotics"

# ---- published robot parameters -------------------------------------------
WHEEL_RADIUS = 0.05      # m   (kinematics file)
TRACK_KIN = 0.22         # m   (kinematics file)
TRACK_ODO = 0.20         # m   (odometry file)
MAX_WHEEL_RAD_S = 25.0   # rad/s (kinematics file)

# ---- heartbeat thresholds as constructed in the published main() ----------
WARN_MS = 150
CRIT_MS = 350

ODOM_PERIOD_TICKS = 10   # 10 x 5 ms = 50 ms = 20 Hz

# frozen command trace: (t_start, t_end, v_mps, w_radps, note)
TRACE = [
    (0.0, 2.0, 0.5, 0.0, "pure forward (published test case 1)"),
    (2.0, 4.0, 0.3, 0.8, "gentle left curve"),
    (4.0, 5.0, 0.0, 2.0, "in-place pivot (published test case 2)"),
    (5.0, 7.0, 1.2, 4.0, "sharp curve, saturating (published test case 3)"),
    (7.0, 8.0, 0.0, 0.0, "stop"),
]
T_END = 8.0
DROP_AT_S = 3.0          # drop one odometry update here (mid-curve)


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


class Odometry:
    """Verbatim reproduction of the published dead-reckoning integrator."""

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


class Heartbeat:
    """Verbatim reproduction of the published failsafe state machine."""

    OPTIMAL = "OPTIMAL_OPERATION"
    WARNING = "WARNING_HEARTBEAT_DELAY"
    CRITICAL = "EMERGENCY_FAILSAFE_PARK"

    def __init__(self, warn_ms, crit_ms):
        self.warn_ms = warn_ms
        self.crit_ms = crit_ms
        self.state = self.OPTIMAL
        self.last_kick_ms = 0.0
        self.transitions = []

    def kick(self, now_ms):
        restored = self.state != self.OPTIMAL
        self.last_kick_ms = now_ms
        if restored:
            self.transitions.append(
                {"t_ms": now_ms, "to": self.OPTIMAL, "reason": "heartbeat restored"})
        self.state = self.OPTIMAL

    def evaluate(self, now_ms):
        elapsed = now_ms - self.last_kick_ms
        prev = self.state
        if elapsed >= self.crit_ms:
            self.state = self.CRITICAL
        elif elapsed >= self.warn_ms:
            self.state = self.WARNING
        else:
            self.state = self.OPTIMAL
        if self.state != prev:
            self.transitions.append(
                {"t_ms": now_ms, "to": self.state, "elapsed_ms": elapsed})
        return elapsed


def yaw_of(node):
    r = node.getOrientation()
    return math.atan2(r[3], r[0])


def rpy_of(node):
    """roll, pitch, yaw from the row-major 3x3 orientation matrix."""
    r = node.getOrientation()
    pitch = math.asin(max(-1.0, min(1.0, -r[6])))
    roll = math.atan2(r[7], r[8])
    yaw = math.atan2(r[3], r[0])
    return roll, pitch, yaw


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    mode = os.environ.get("EVAL_DROP_MODE", "drop")   # "drop" or "baseline"
    tag = os.environ.get("EVAL_TAG", "run1")

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

    odo020 = Odometry(TRACK_ODO)
    odo022 = Odometry(TRACK_KIN)
    hb = Heartbeat(WARN_MS, CRIT_MS)

    samples = []
    commands_received = []
    dropped = []
    prev_l = None
    prev_r = None
    last_odom_tick = 0
    tick = 0
    t = 0.0
    seg_idx = -1

    # prime one step so sensors read
    robot.step(dt_ms)
    prev_l = lsens.getValue()
    prev_r = rsens.getValue()
    hb.kick(0.0)
    x0 = self_node.getPosition()
    yaw0 = yaw_of(self_node)
    truth_yaw_cont = yaw0
    prev_raw_yaw = yaw0
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    min_z = x0[2]
    max_z = x0[2]

    while robot.step(dt_ms) != -1 and t < T_END:
        t += dt
        tick += 1
        now_ms = t * 1000.0

        # unwrap the true heading so it can be compared with the odometry's
        # freely accumulating theta over more than one full turn
        roll_t, pitch_t, raw_yaw = rpy_of(self_node)
        max_abs_roll = max(max_abs_roll, abs(roll_t))
        max_abs_pitch = max(max_abs_pitch, abs(pitch_t))
        pz = self_node.getPosition()[2]
        min_z = min(min_z, pz)
        max_z = max(max_z, pz)
        truth_yaw_cont += wrap(raw_yaw - prev_raw_yaw)
        prev_raw_yaw = raw_yaw

        # ---- command from the frozen trace ----
        v = w = 0.0
        seg = None
        for i, (ts, te, vv, ww, note) in enumerate(TRACE):
            if ts <= t < te:
                v, w, seg = vv, ww, i
                break
        if seg is not None and seg != seg_idx:
            seg_idx = seg
            wl, wr, sat, sc = twist_to_wheels(v, w)
            commands_received.append({
                "t_s": round(t, 3), "segment": seg, "note": TRACE[seg][4],
                "v_mps": v, "w_radps": w,
                "wheel_left_radps_cmd": round(wl, 4),
                "wheel_right_radps_cmd": round(wr, 4),
                "saturated": sat, "saturation_scale": round(sc, 4),
            })

        wl, wr, sat, sc = twist_to_wheels(v, w)
        left.setVelocity(wl)
        right.setVelocity(wr)

        # ---- encoder-driven odometry at 20 Hz ----
        if tick - last_odom_tick >= ODOM_PERIOD_TICKS:
            odom_dt = (tick - last_odom_tick) * dt
            cur_l = lsens.getValue()
            cur_r = rsens.getValue()
            wl_meas = (cur_l - prev_l) / odom_dt
            wr_meas = (cur_r - prev_r) / odom_dt
            vl_meas = wl_meas * WHEEL_RADIUS
            vr_meas = wr_meas * WHEEL_RADIUS

            is_drop = (mode == "drop"
                       and not dropped
                       and t >= DROP_AT_S)
            if is_drop:
                # The published odometry class is only advanced when update()
                # is called; a sample that never arrives is simply never
                # integrated, and the class has no notion of wall time with
                # which to notice.
                dropped.append({
                    "t_s": round(t, 3),
                    "odom_dt_s": round(odom_dt, 4),
                    "wheel_left_radps_meas": round(wl_meas, 4),
                    "wheel_right_radps_meas": round(wr_meas, 4),
                })
            else:
                odo020.update(vl_meas, vr_meas, odom_dt)
                odo022.update(vl_meas, vr_meas, odom_dt)
                hb.kick(now_ms)

            prev_l, prev_r = cur_l, cur_r
            last_odom_tick = tick

            p = self_node.getPosition()
            roll_s, pitch_s, _ = rpy_of(self_node)
            samples.append({
                "roll_deg": round(math.degrees(roll_s), 3),
                "pitch_deg": round(math.degrees(pitch_s), 3),
                "z": round(p[2], 5),
                "t_s": round(t, 4),
                "cmd_v": v, "cmd_w": w,
                "wheel_cmd": [round(wl, 4), round(wr, 4)],
                "wheel_meas": [round(wl_meas, 4), round(wr_meas, 4)],
                "truth": [round(p[0], 5), round(p[1], 5), round(truth_yaw_cont, 5)],
                "odo020": [round(odo020.x, 5), round(odo020.y, 5), round(odo020.theta, 5)],
                "odo022": [round(odo022.x, 5), round(odo022.y, 5), round(odo022.theta, 5)],
                "hb_state": hb.state,
                "dropped_here": bool(is_drop),
            })

        hb.evaluate(now_ms)

    p = self_node.getPosition()
    truth = [p[0], p[1], truth_yaw_cont]

    def err(o):
        dth = o.theta - truth[2]
        return {
            "x": round(o.x - truth[0], 5),
            "y": round(o.y - truth[1], 5),
            "theta_rad": round(dth, 5),
            "theta_deg": round(math.degrees(dth), 3),
            "euclidean_xy": round(math.hypot(o.x - truth[0], o.y - truth[1]), 5),
        }

    result = {
        "mode": mode,
        "tag": tag,
        "basic_time_step_ms": dt_ms,
        "odom_rate_hz": round(1.0 / (ODOM_PERIOD_TICKS * dt), 3),
        "published_params": {
            "wheel_radius_m": WHEEL_RADIUS,
            "track_width_kinematics_m": TRACK_KIN,
            "track_width_odometry_m": TRACK_ODO,
            "max_wheel_rad_s": MAX_WHEEL_RAD_S,
            "heartbeat_warn_ms": WARN_MS,
            "heartbeat_critical_ms": CRIT_MS,
        },
        "start_pose": {"x": x0[0], "y": x0[1], "z": x0[2], "yaw": yaw0},
        "attitude_check": {
            "max_abs_roll_deg": round(math.degrees(max_abs_roll), 3),
            "max_abs_pitch_deg": round(math.degrees(max_abs_pitch), 3),
            "z_min_m": round(min_z, 5),
            "z_max_m": round(max_z, 5),
            "upright": bool(max_abs_roll < 0.35 and max_abs_pitch < 0.35),
        },
        "commands_received": commands_received,
        "dropped_updates": dropped,
        "odom_updates_applied": {"odo020": odo020.updates, "odo022": odo022.updates},
        "final_truth": {"x": round(truth[0], 5), "y": round(truth[1], 5),
                        "theta_rad": round(truth[2], 5),
                        "theta_deg": round(math.degrees(truth[2]), 3)},
        "final_odo020": {"x": round(odo020.x, 5), "y": round(odo020.y, 5),
                         "theta_rad": round(odo020.theta, 5),
                         "theta_deg": round(math.degrees(odo020.theta), 3)},
        "final_odo022": {"x": round(odo022.x, 5), "y": round(odo022.y, 5),
                         "theta_rad": round(odo022.theta, 5),
                         "theta_deg": round(math.degrees(odo022.theta), 3)},
        "heading_convention": ("theta is CONTINUOUS (unwrapped) for both the "
                               "truth and the estimates; the frozen trace "
                               "commands more than a full turn"),
        "error_odo020_vs_truth": err(odo020),
        "error_odo022_vs_truth": err(odo022),
        "heartbeat_transitions": hb.transitions,
        "heartbeat_final_state": hb.state,
        "samples": samples,
    }

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"odom_{mode}_{tag}.json")
    with open(path, "w") as f:
        json.dump(result, f, indent=1)
    print(f"[eval] wrote {path}", flush=True)
    print(f"[eval] mode={mode} truth=({truth[0]:.4f},{truth[1]:.4f},"
          f"{math.degrees(truth[2]):.2f}deg) "
          f"odo020_err_xy={result['error_odo020_vs_truth']['euclidean_xy']} "
          f"odo020_err_theta_deg={result['error_odo020_vs_truth']['theta_deg']} "
          f"hb={hb.state} drops={len(dropped)}", flush=True)
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
