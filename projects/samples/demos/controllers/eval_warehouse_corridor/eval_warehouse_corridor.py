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

"""Does the FIFO corridor order survive contact, and how far off-line do
robots drift when the floor changes?

The token rule is his, transcribed from Multirobot_Warehouse_Dynamic_2D.m:
one robot at a time inside the approach zone, the token released when the
owner leaves it, the queue served in order. The difference is that here the
robots have wheels and a floor, so holding the token does not guarantee
arriving anywhere in particular.

Measured per robot:
    max |x - 34.0|   lateral deviation from the corridor centreline
    entry / exit     sim time crossing CORR_Y_LO and CORR_Y_HI
    clear order      the order they physically leave the zone
    wall contact     whether the chassis ever touched a shelf face
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = os.path.join("O:", os.sep, "omnisim", ".tmp", "reply-evals-20260915",
                   "aaron")

NUM_R = 4
CORR_X_LO, CORR_X_HI = 32.5, 35.5
CORR_Y_LO, CORR_Y_HI = 9.0, 36.0
CORR_APPROACH = 1.5
CENTRE_X = 34.0
V = 0.6                 # his vfhP.v
MAX_OMEGA = 1.2         # his vfhP.maxOmega
WHEEL_R = 0.12
TRACK = 0.72            # wheel separation, from the generator
GOAL_Y = CORR_Y_HI + 2.0
RUN_S = 400.0


def main():
    robot = Supervisor()
    ms = int(robot.getBasicTimeStep())
    dt = ms / 1000.0

    # Every vehicle runs THIS controller. A supervisor can only getDevice()
    # its own robot's motors, so each instance drives only its own wheels --
    # and because each is a supervisor it can read all four poses and apply
    # his token rule itself. The rule is a pure function of those poses, so
    # all four instances derive the same owner without any messaging.
    me_name = robot.getName()
    my_id = int(me_name.lstrip("r"))

    bots = []
    for i in range(NUM_R):
        n = robot.getFromDef("R%d" % i)
        if n is None:
            print("[wh] FATAL missing node R%d" % i, flush=True)
            sys.exit(1)
        bots.append({"node": n, "ml": None, "mr": None, "id": i,
                     "max_dev": 0.0, "entry": None, "exit": None,
                     "done": False, "touched": False})
    ml = robot.getDevice("%s_left" % me_name)
    mr = robot.getDevice("%s_right" % me_name)
    if ml is None or mr is None:
        print("[wh] FATAL missing motors for %s" % me_name, flush=True)
        sys.exit(1)
    for m in (ml, mr):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    bots[my_id]["ml"], bots[my_id]["mr"] = ml, mr

    def pose(b):
        p = b["node"].getPosition()
        o = b["node"].getOrientation()
        return p[0], p[1], math.atan2(o[3], o[0])

    def in_zone(x, y):
        return (CORR_X_LO - CORR_APPROACH <= x <= CORR_X_HI + CORR_APPROACH
                and CORR_Y_LO <= y <= CORR_Y_HI)

    def touching(b):
        try:
            pts = b["node"].getContactPoints(True)
        except TypeError:
            pts = b["node"].getContactPoints()
        except Exception:
            return False
        return len(pts or []) > 2      # wheels+caster are the floor contacts

    owner = None
    order = []
    t = 0.0
    robot.step(ms)
    while robot.step(ms) != -1 and t < RUN_S:
        t += dt

        # ── his token rule ────────────────────────────────────────────────
        if owner is not None:
            b = bots[owner]
            x, y, _ = pose(b)
            if not in_zone(x, y) and y > CORR_Y_LO:
                order.append({"robot": owner, "t_clear": round(t, 3)})
                b["exit"] = round(t, 3)
                owner = None
        if owner is None:
            for b in bots:
                if b["done"]:
                    continue
                x, y, _ = pose(b)
                if y < CORR_Y_HI:
                    owner = b["id"]
                    break

        for b in bots:
            x, y, th = pose(b)
            if b["entry"] is None and y >= CORR_Y_LO:
                b["entry"] = round(t, 3)
            if in_zone(x, y):
                b["max_dev"] = max(b["max_dev"], abs(x - CENTRE_X))
            if touching(b):
                b["touched"] = True
            if y >= GOAL_Y:
                b["done"] = True
                if b["id"] == my_id:
                    ml.setVelocity(0.0)
                    mr.setVelocity(0.0)
                continue
            if b["id"] != my_id:
                continue

            # His rule: a robot is held only while it is inside the approach
            # zone and does not own the token. Outside the zone it drives on.
            # Queueing therefore happens at the zone mouth, as in his engine.
            approaching = (owner is not None and b["id"] != owner
                           and y > CORR_Y_LO - CORR_APPROACH - 0.9)
            hold = approaching and y < CORR_Y_HI
            if hold:
                ml.setVelocity(0.0)
                mr.setVelocity(0.0)
                continue

            # steer to the centreline, heading +y
            head_err = math.atan2(CENTRE_X - x, max(0.3, GOAL_Y - y))
            desired = math.pi / 2 + head_err
            e = (desired - th + math.pi) % (2 * math.pi) - math.pi
            w = max(-MAX_OMEGA, min(MAX_OMEGA, 2.0 * e))   # his turnGain
            vl = (V - w * TRACK / 2.0) / WHEEL_R
            vr = (V + w * TRACK / 2.0) / WHEEL_R
            ml.setVelocity(vl)
            mr.setVelocity(vr)

        if all(b["done"] for b in bots):
            break

    rec = {"mu_note": "ground friction is set in the world file",
           "corridor": {"x_lo": CORR_X_LO, "x_hi": CORR_X_HI,
                        "centre_x": CENTRE_X, "y_lo": CORR_Y_LO,
                        "y_hi": CORR_Y_HI},
           "commanded_speed_m_s": V, "sim_s": round(t, 3),
           "clear_order": order,
           "robots": [{"id": b["id"], "max_lateral_dev_m": round(b["max_dev"], 5),
                       "entry_s": b["entry"], "exit_s": b["exit"],
                       "reached_goal": b["done"],
                       "final_xy": [round(v, 4) for v in pose(b)[:2]]}
                      for b in bots]}
    if my_id != 0:
        return
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "corridor_run.json"), "w") as fh:
        json.dump(rec, fh, indent=1)
    print("[wh] order %s" % [o["robot"] for o in order], flush=True)
    for b in rec["robots"]:
        print("[wh] R%d dev %.4f m  entry %s exit %s goal %s"
              % (b["id"], b["max_lateral_dev_m"], b["entry_s"], b["exit_s"],
                 b["reached_goal"]), flush=True)
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
        print("[wh] FAILED " + tb, flush=True)
        raise
