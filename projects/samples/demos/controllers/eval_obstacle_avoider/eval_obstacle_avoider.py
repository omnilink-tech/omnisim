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

"""A published Arduino obstacle-avoider, executed on real physics.

The control logic is a faithful port of the published sketch: the same 15 cm
trigger, the same stop / reverse / look-right / look-left / turn recovery with
its exact delays, the same 160 ms pivot, the same servo angles, and the same
readPing() rule that turns a zero reading into 250 cm.  Nothing was copied --
the repository publishes no licence -- the sequence was read and rebuilt.

The blocking delay() calls are reproduced as tick counts, and the motors hold
their last command throughout, which is what the Arduino does.

One factor changes across runs: the obstacle's lateral offset (EVAL_OBS_Y).
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = r"O:\omnisim\.tmp\reply-evals-20260913\sainadh_rig"

# ---- published constants -------------------------------------------------
TRIGGER_CM = 15           # if (distance <= 15)
PWM = 130                 # analogWrite(enA/enB, 130) out of 255
SERVO_CENTRE_DEG = 115    # servo_motor.write(115)
SERVO_RIGHT_DEG = 50      # lookRight()
SERVO_LEFT_DEG = 170      # lookLeft()
TURN_MS = 160             # delay(160) inside turnRight/turnLeft
BACK_MS = 300
STOP_MS = 300
SETTLE_MS = 300           # delay(300) after each look
LOOK_SETTLE_MS = 300      # servo settling inside lookRight/lookLeft
PING_DELAY_MS = 70        # delay(70) at the top of readPing
LOOP_DELAY_MS = 50        # delay(50) at the top of loop
PING_NOTHING_CM = 250     # readPing maps a 0 reading to 250

GOAL = (1.9, 0.0)
GOAL_RADIUS = 0.14
T_END = 60.0


def deg_to_servo_rad(d):
    """The sketch writes servo degrees where 115 is straight ahead."""
    return math.radians(d - SERVO_CENTRE_DEG)


def yaw_of(node):
    r = node.getOrientation()
    return math.atan2(r[3], r[0])


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def main():
    obs_y = float(os.environ.get("EVAL_OBS_Y", "0.0"))
    wheel_rad_s = float(os.environ.get("EVAL_WHEEL_RAD_S", "9.0"))
    tag = os.environ.get("EVAL_TAG", "run1")

    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    left = robot.getDevice("left_wheel_motor")
    right = robot.getDevice("right_wheel_motor")
    lsens = robot.getDevice("left_wheel_sensor")
    rsens = robot.getDevice("right_wheel_sensor")
    scan = robot.getDevice("scan_motor")
    sonar = robot.getDevice("sonar")
    for s in (lsens, rsens):
        s.enable(dt_ms)
    sonar.enable(dt_ms)
    for m in (left, right):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    scan.setPosition(deg_to_servo_rad(SERVO_CENTRE_DEG))

    self_node = robot.getSelf()
    obstacle = robot.getFromDef("OBSTACLE")
    if obstacle is not None:
        f = obstacle.getField("translation")
        t = f.getSFVec3f()
        f.setSFVec3f([t[0], obs_y, t[2]])

    # PWM 130/255 of the free-running wheel speed
    duty = PWM / 255.0
    v_cmd = wheel_rad_s * duty

    def drive(l, r):
        left.setVelocity(l)
        right.setVelocity(r)

    def read_ping():
        """readPing(): a zero reading becomes 250 cm."""
        raw = sonar.getValue()          # metres, lookupTable is identity
        cm = 0 if raw <= 0.0 else raw * 100.0
        if cm >= 199.0:                 # beyond maximum_distance, NewPing gives 0
            cm = 0
        return PING_NOTHING_CM if cm == 0 else cm

    # cooperative reproduction of the blocking sketch
    script = []            # list of (duration_ms, action)
    log = []
    events = []
    tick = 0
    t = 0.0
    distance = PING_NOTHING_CM
    goes_forward = False
    min_clearance_m = 9.9
    collisions = 0
    in_contact = False
    path_len = 0.0
    prev_p = self_node.getPosition()
    start_p = list(prev_p)
    obstacle_encounters = 0
    reached = False
    t_reached = None

    state = ("loop_delay", LOOP_DELAY_MS)
    phase_end = LOOP_DELAY_MS
    queue = []
    dist_right = dist_left = 0

    robot.step(dt_ms)

    while robot.step(dt_ms) != -1 and t < T_END:
        t += dt
        tick += 1
        now_ms = t * 1000.0

        p = self_node.getPosition()
        path_len += math.dist(p[:2], prev_p[:2])
        prev_p = p

        raw = sonar.getValue()
        if 0.0 < raw < min_clearance_m:
            min_clearance_m = raw
        # contact with the obstacle: chassis half-length 0.10 + obstacle half 0.12
        if obstacle is not None:
            op = obstacle.getPosition()
            dx = abs(p[0] - op[0]) - (0.10 + 0.12)
            dy = abs(p[1] - op[1]) - (0.07 + 0.25)
            touching = dx < 0.0 and dy < 0.0
            if touching and not in_contact:
                collisions += 1
                events.append({"t_s": round(t, 3), "type": "collision",
                               "pose": [round(p[0], 4), round(p[1], 4)]})
            in_contact = touching

        if not reached and math.dist(p[:2], GOAL) <= GOAL_RADIUS:
            reached = True
            t_reached = round(t, 3)
            events.append({"t_s": t_reached, "type": "goal_reached"})

        # ---- the sketch's control flow, tick-accurate ----
        if now_ms >= phase_end:
            if queue:
                name, dur, act = queue.pop(0)
                act()
                state = (name, dur)
                phase_end = now_ms + dur
            else:
                # top of loop(): delay(50), then decide on the last distance
                if distance <= TRIGGER_CM:
                    obstacle_encounters += 1
                    events.append({"t_s": round(t, 3), "type": "obstacle_detected",
                                   "distance_cm": round(distance, 2),
                                   "pose": [round(p[0], 4), round(p[1], 4)]})

                    def a_stop():
                        drive(0.0, 0.0)

                    def a_back():
                        drive(-v_cmd, -v_cmd)

                    def a_look_right():
                        scan.setPosition(deg_to_servo_rad(SERVO_RIGHT_DEG))
                        drive(0.0, 0.0)

                    def a_read_right():
                        nonlocal dist_right
                        dist_right = read_ping()
                        scan.setPosition(deg_to_servo_rad(SERVO_CENTRE_DEG))

                    def a_look_left():
                        scan.setPosition(deg_to_servo_rad(SERVO_LEFT_DEG))

                    def a_read_left():
                        nonlocal dist_left
                        dist_left = read_ping()
                        scan.setPosition(deg_to_servo_rad(SERVO_CENTRE_DEG))

                    def a_turn():
                        nonlocal goes_forward
                        goes_forward = False
                        if dist_right >= dist_left:
                            drive(v_cmd, -v_cmd)      # turnRight
                            events.append({"t_s": round(t, 3), "type": "turn",
                                           "dir": "right",
                                           "right_cm": round(dist_right, 1),
                                           "left_cm": round(dist_left, 1)})
                        else:
                            drive(-v_cmd, v_cmd)      # turnLeft
                            events.append({"t_s": round(t, 3), "type": "turn",
                                           "dir": "left",
                                           "right_cm": round(dist_right, 1),
                                           "left_cm": round(dist_left, 1)})

                    queue = [
                        ("moveStop", STOP_MS, a_stop),
                        ("moveBackward", BACK_MS, a_back),
                        ("moveStop", STOP_MS, a_stop),
                        ("lookRight_settle", LOOK_SETTLE_MS, a_look_right),
                        ("lookRight_ping", PING_DELAY_MS + 100, a_read_right),
                        ("after_right", SETTLE_MS, a_stop),
                        ("lookLeft_settle", LOOK_SETTLE_MS, a_look_left),
                        ("lookLeft_ping", PING_DELAY_MS + 100, a_read_left),
                        ("after_left", SETTLE_MS, a_stop),
                        ("turn", TURN_MS, a_turn),
                        ("moveStop", 1, a_stop),
                        ("readPing", PING_DELAY_MS, lambda: None),
                        ("loop_delay", LOOP_DELAY_MS, lambda: None),
                    ]
                else:
                    if not goes_forward:
                        goes_forward = True
                        drive(v_cmd, v_cmd)
                    queue = [
                        ("readPing", PING_DELAY_MS, lambda: None),
                        ("loop_delay", LOOP_DELAY_MS, lambda: None),
                    ]
                name, dur, act = queue.pop(0)
                act()
                state = (name, dur)
                phase_end = now_ms + dur

            if state[0] == "readPing":
                distance = read_ping()

        if tick % 10 == 0:
            log.append({
                "t_s": round(t, 3), "phase": state[0],
                "distance_cm": round(distance, 2),
                "sonar_m": round(raw, 5),
                "pose": [round(p[0], 4), round(p[1], 4),
                         round(yaw_of(self_node), 4)],
            })

    p = self_node.getPosition()
    op = obstacle.getPosition() if obstacle is not None else [None, None, None]
    result = {
        "tag": tag,
        "obstacle_y": obs_y,
        "obstacle_pos": [round(op[0], 4), round(op[1], 4)] if op[0] is not None else None,
        "wheel_rad_s_free": wheel_rad_s,
        "pwm": PWM,
        "commanded_wheel_rad_s": round(v_cmd, 4),
        "published_constants": {
            "trigger_cm": TRIGGER_CM, "turn_ms": TURN_MS,
            "back_ms": BACK_MS, "stop_ms": STOP_MS,
            "servo_deg": [SERVO_RIGHT_DEG, SERVO_CENTRE_DEG, SERVO_LEFT_DEG],
            "ping_nothing_cm": PING_NOTHING_CM,
        },
        "start": [round(start_p[0], 4), round(start_p[1], 4)],
        "goal": list(GOAL), "goal_radius": GOAL_RADIUS,
        "final_pose": [round(p[0], 4), round(p[1], 4),
                       round(yaw_of(self_node), 4)],
        "reached_goal": reached,
        "t_reached_s": t_reached,
        "final_distance_to_goal_m": round(math.dist(p[:2], GOAL), 4),
        "path_length_m": round(path_len, 4),
        "min_sonar_clearance_m": round(min_clearance_m, 4),
        "collisions": collisions,
        "obstacle_encounters": obstacle_encounters,
        "events": events,
        "trace": log,
    }
    os.makedirs(OUT, exist_ok=True)
    name = f"avoid_y{obs_y:+.2f}_w{wheel_rad_s}_{tag}.json"
    with open(os.path.join(OUT, name), "w") as f:
        json.dump(result, f, indent=1)
    print(f"[eval] wrote {name}", flush=True)
    print(f"[eval] obs_y={obs_y:+.2f} reached={reached} t={t_reached} "
          f"path={path_len:.3f}m encounters={obstacle_encounters} "
          f"collisions={collisions} min_clear={min_clearance_m:.3f}m",
          flush=True)
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        os.makedirs(OUT, exist_ok=True)
        tb = traceback.format_exc()
        with open(os.path.join(OUT, "controller_traceback.txt"), "a") as f:
            f.write(tb + "=" * 60)
        print("[eval] CONTROLLER FAILED: " + tb, flush=True)
        raise
