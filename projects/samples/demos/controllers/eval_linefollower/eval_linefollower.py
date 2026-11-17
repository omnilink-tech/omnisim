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

"""Five-channel IR line-follower evaluation.

The control law is transcribed verbatim from the published Arduino sketch,
including C operator precedence, the 20 ms drive pulse and the 50 ms brake that
follows every motion command.

Two variants, identical in every other respect:

  published  the five IR channels are sampled ONCE per loop() iteration and the
             while-loops spin on those cached values, exactly as written.
  resampled  one change only: the five channels are re-sampled at the top of
             each pulse, so a satisfied condition can stop being satisfied.

The course is three straights joined by two 90-degree corners of radius R.
Tightening R is the single factor changed between the two course runs.

The painted line is modelled geometrically: each sensor reports TRUE when its
ground point lies within half a stripe width of the analytic course centreline.
Optical reflectance, ambient light, battery, motor and tyre behaviour are NOT
reproduced -- that was stated up front and it still holds.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = r"O:\omnisim\social\launch\pilots\replies_2026_09_12\linefollower"

# ---- rover geometry -------------------------------------------------------
SENSOR_AHEAD = 0.055        # m ahead of the wheel axis
SENSOR_PITCH = 0.012        # m between adjacent channels
SENSOR_OFFSETS = [2 * SENSOR_PITCH, SENSOR_PITCH, 0.0,
                  -SENSOR_PITCH, -2 * SENSOR_PITCH]   # ir1 (left) .. ir5 (right)
STRIPE_WIDTH = float(os.environ.get("EVAL_STRIPE", "0.025"))   # m
STRIPE_HALF = STRIPE_WIDTH / 2.0

WHEEL_SPEED = 12.0          # rad/s at full drive

# ---- published timing -----------------------------------------------------
DRIVE_MS = 20               # DRIVE_THRESH_ACCURACY
BRAKE_MS = 50               # the brake every motion command appends

TIME_BUDGET_S = 100.0
OFFLINE_M = 0.036           # beyond the reach of the outermost channel
ONLINE_M = STRIPE_HALF      # back on the stripe


# ---------------------------------------------------------------- course ---
def build_course(R):
    """Three straights joined by two 90-degree corners of radius R."""
    segs = []
    segs.append({"kind": "line", "a": (0.0, 0.0), "b": (1.0, 0.0)})
    segs.append({"kind": "arc", "c": (1.0, R), "r": R,
                 "a0": -math.pi / 2, "a1": 0.0, "ccw": True})
    segs.append({"kind": "line", "a": (1.0 + R, R), "b": (1.0 + R, R + 0.8)})
    segs.append({"kind": "arc", "c": (1.0 + 2 * R, R + 0.8), "r": R,
                 "a0": math.pi, "a1": math.pi / 2, "ccw": False})
    segs.append({"kind": "line", "a": (1.0 + 2 * R, 2 * R + 0.8),
                 "b": (2.0 + 2 * R, 2 * R + 0.8)})
    total = 0.0
    for s in segs:
        if s["kind"] == "line":
            s["len"] = math.hypot(s["b"][0] - s["a"][0], s["b"][1] - s["a"][1])
        else:
            s["len"] = abs(s["a1"] - s["a0"]) * s["r"]
        s["s0"] = total
        total += s["len"]
    return segs, total


def nearest(segs, px, py):
    """Return (distance, arclength_progress) of the nearest centreline point."""
    best_d = float("inf")
    best_s = 0.0
    for s in segs:
        if s["kind"] == "line":
            ax, ay = s["a"]
            bx, by = s["b"]
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            t = 0.0 if L2 == 0 else ((px - ax) * vx + (py - ay) * vy) / L2
            t = max(0.0, min(1.0, t))
            cx, cy = ax + t * vx, ay + t * vy
            d = math.hypot(px - cx, py - cy)
            sl = s["s0"] + t * s["len"]
        else:
            cx0, cy0 = s["c"]
            ang = math.atan2(py - cy0, px - cx0)
            a0, a1 = s["a0"], s["a1"]
            lo, hi = (a0, a1) if a0 <= a1 else (a1, a0)
            # bring ang into the neighbourhood of the span
            while ang < lo - math.pi:
                ang += 2 * math.pi
            while ang > hi + math.pi:
                ang -= 2 * math.pi
            ca = max(lo, min(hi, ang))
            cx = cx0 + s["r"] * math.cos(ca)
            cy = cy0 + s["r"] * math.sin(ca)
            d = math.hypot(px - cx, py - cy)
            frac = abs(ca - a0) / abs(a1 - a0) if a1 != a0 else 0.0
            sl = s["s0"] + frac * s["len"]
        if d < best_d:
            best_d, best_s = d, sl
    return best_d, best_s


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


def main():
    variant = os.environ.get("EVAL_VARIANT", "published")   # published | resampled
    R = float(os.environ.get("EVAL_RADIUS", "0.40"))
    tag = os.environ.get("EVAL_TAG", "run1")

    segs, total_len = build_course(R)

    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    left = robot.getDevice("left_wheel_motor")
    right = robot.getDevice("right_wheel_motor")
    for m in (left, right):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    self_node = robot.getSelf()

    def read_ir():
        p = self_node.getPosition()
        yaw = yaw_of(self_node)
        cy, sy = math.cos(yaw), math.sin(yaw)
        vals = []
        pts = []
        for off in SENSOR_OFFSETS:
            sx = p[0] + SENSOR_AHEAD * cy - off * sy
            sy_ = p[1] + SENSOR_AHEAD * sy + off * cy
            d, _ = nearest(segs, sx, sy_)
            vals.append(d <= STRIPE_HALF)
            pts.append((round(sx, 4), round(sy_, 4), round(d, 5)))
        return vals, pts

    def decide(ir):
        ir1, ir2, ir3, ir4, ir5 = ir
        # verbatim, with C precedence: (!(ir1&&ir5) && (ir2||ir4)) || (ir3&&ir4)
        if (not (ir1 and ir5)) and (ir2 or ir4) or (ir3 and ir4):
            return "FWD"
        if ir1 and ir2 and not (ir4 and ir5):
            return "LEFT"
        if ir4 and ir5 and ((not ir1) or (not ir2)):
            return "RIGHT"
        return None

    def apply(action, braking):
        if braking or action is None:
            left.setVelocity(0.0)
            right.setVelocity(0.0)
            return
        if action == "FWD":
            left.setVelocity(WHEEL_SPEED)
            right.setVelocity(WHEEL_SPEED)
        elif action == "LEFT":
            left.setVelocity(-WHEEL_SPEED)
            right.setVelocity(WHEEL_SPEED)
        elif action == "RIGHT":
            left.setVelocity(WHEEL_SPEED)
            right.setVelocity(-WHEEL_SPEED)

    t = 0.0
    latched = None
    latched_at = None
    phase = "idle"          # idle | drive | brake
    phase_left_ms = 0.0
    pulses = {"FWD": 0, "LEFT": 0, "RIGHT": 0}
    reads = 0
    samples = []
    max_dev = 0.0
    sum_dev = 0.0
    n_dev = 0
    max_progress = 0.0
    offline_events = []
    cur_offline = None
    completed = False
    completed_t = None
    moving_ticks = 0
    total_ticks = 0
    max_abs_roll = 0.0
    max_abs_pitch = 0.0

    robot.step(dt_ms)

    while robot.step(dt_ms) != -1 and t < TIME_BUDGET_S:
        t += dt
        total_ticks += 1

        # ---- pulse state machine (20 ms drive, 50 ms brake) ----
        if phase == "idle" or phase_left_ms <= 0.0:
            if phase == "drive":
                phase = "brake"
                phase_left_ms = BRAKE_MS
            else:
                # start of a new decision point
                if variant == "resampled" or latched is None:
                    ir, pts = read_ir()
                    reads += 1
                    action = decide(ir)
                    if action is not None and latched is None:
                        latched_at = round(t, 4)
                    latched = action
                    last_ir = ir
                    last_pts = pts
                action = latched
                if action is None:
                    phase = "idle"
                    phase_left_ms = dt_ms
                else:
                    pulses[action] += 1
                    phase = "drive"
                    phase_left_ms = DRIVE_MS

        apply(latched, phase == "brake")
        if phase == "drive" and latched is not None:
            moving_ticks += 1
        phase_left_ms -= dt_ms

        # ---- metrics ----
        roll_t, pitch_t, _ = rpy_of(self_node)
        max_abs_roll = max(max_abs_roll, abs(roll_t))
        max_abs_pitch = max(max_abs_pitch, abs(pitch_t))
        p = self_node.getPosition()
        d, prog = nearest(segs, p[0], p[1])
        max_dev = max(max_dev, d)
        sum_dev += d
        n_dev += 1
        max_progress = max(max_progress, prog)

        if cur_offline is None and d > OFFLINE_M:
            cur_offline = {"t_start": round(t, 3), "dev_at_start": round(d, 5),
                           "peak_dev": round(d, 5)}
        elif cur_offline is not None:
            cur_offline["peak_dev"] = max(cur_offline["peak_dev"], round(d, 5))
            if d <= ONLINE_M:
                cur_offline["t_end"] = round(t, 3)
                cur_offline["recovery_s"] = round(t - cur_offline["t_start"], 3)
                offline_events.append(cur_offline)
                cur_offline = None

        if not completed and max_progress >= total_len - 0.05 and d <= 0.05:
            completed = True
            completed_t = round(t, 3)

        if total_ticks % 20 == 0:
            samples.append({
                "t_s": round(t, 3),
                "pose": [round(p[0], 4), round(p[1], 4),
                         round(math.degrees(yaw_of(self_node)), 2)],
                "dev_m": round(d, 5),
                "progress_m": round(prog, 4),
                "action": latched,
                "phase": phase,
            })

        if completed:
            break

    if cur_offline is not None:
        cur_offline["t_end"] = None
        cur_offline["recovery_s"] = None
        cur_offline["note"] = "never returned to the stripe within the budget"
        offline_events.append(cur_offline)

    total_pulses = sum(pulses.values())
    turn_pulses = pulses["LEFT"] + pulses["RIGHT"]
    result = {
        "variant": variant,
        "corner_radius_m": R,
        "tag": tag,
        "course_length_m": round(total_len, 4),
        "basic_time_step_ms": dt_ms,
        "drive_pulse_ms": DRIVE_MS,
        "brake_ms": BRAKE_MS,
        "wheel_speed_rad_s": WHEEL_SPEED,
        "stripe_width_m": STRIPE_WIDTH,
        "sensor_pitch_m": SENSOR_PITCH,
        "attitude_check": {
            "max_abs_roll_deg": round(math.degrees(max_abs_roll), 3),
            "max_abs_pitch_deg": round(math.degrees(max_abs_pitch), 3),
            "upright": bool(max_abs_roll < 0.35 and max_abs_pitch < 0.35),
        },
        "completed": completed,
        "completion_time_s": completed_t,
        "sim_time_s": round(t, 3),
        "max_progress_m": round(max_progress, 4),
        "progress_fraction": round(max_progress / total_len, 4),
        "max_lateral_deviation_m": round(max_dev, 5),
        "mean_lateral_deviation_m": round(sum_dev / max(1, n_dev), 5),
        "sensor_reads": reads,
        "first_latch_time_s": latched_at,
        "final_action": latched,
        "pulses": pulses,
        "total_pulses": total_pulses,
        "steering_saturation_fraction": (round(turn_pulses / total_pulses, 4)
                                         if total_pulses else None),
        "duty_cycle_moving": round(moving_ticks / max(1, total_ticks), 4),
        "offline_events": offline_events,
        "n_offline_events": len(offline_events),
        "max_recovery_s": max([e["recovery_s"] for e in offline_events
                               if e.get("recovery_s") is not None], default=None),
        "samples": samples,
    }

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"line_{variant}_R{R:.2f}_S{STRIPE_WIDTH:.3f}_{tag}.json")
    with open(path, "w") as f:
        json.dump(result, f, indent=1)
    print(f"[eval] wrote {path}", flush=True)
    print(f"[eval] variant={variant} R={R} completed={completed} "
          f"progress={result['progress_fraction']} "
          f"max_dev={result['max_lateral_deviation_m']} "
          f"reads={reads} latch_t={latched_at} "
          f"sat={result['steering_saturation_fraction']} "
          f"pulses={pulses}", flush=True)
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
