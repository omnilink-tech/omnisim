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

"""Does HOLD hold, and what does the base do while the arm is up?

Three arm poses, each raised with a payload in the claw, held still, and then
driven forward. Per pose the run reports:

  sag          commanded joint angle minus the angle actually held, per stage
  sag_driving  the same, measured while the base is moving
  base pitch   how far the chassis tips nose-up or nose-down while holding
  base slip    how far the base travels during the hold, with no drive command

The arm geometry is not the reporter's robot -- his repository publishes
driver code only. What IS his is the arrangement: two motors per stage, HOLD
stopping, torque at maximum. The motor figure is the published VEX IQ Smart
Motor stall torque of 0.414 N.m, so a two-motor stage gets 0.828 N.m.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = os.path.join("O:", os.sep, "omnisim", ".tmp", "reply-evals-20260915",
                   "ladoodle")

# Rotation is about +y, which takes +x toward -z: POSITIVE angle points the
# arm DOWN, negative lifts it. Getting this backwards drives the arm into the
# floor and reads the floor contact as sag, which is exactly what happened on
# the first run of this rig.
#
# Rather than three invented "presets", sweep the lower arm through elevation
# angles at a fixed extension. Gravity torque at the shoulder falls as
# cos(elevation), so this walks the arm across its own torque limit and finds
# where HOLD stops holding -- a boundary his robot has too, at whatever
# lengths and masses it actually has.
POSES = [
    ("elev_00_horizontal", 0.00, 0.0),
    ("elev_15", -0.262, 0.0),
    ("elev_30", -0.524, 0.0),
    ("elev_45", -0.785, 0.0),
    ("elev_60", -1.047, 0.0),
    ("elev_75", -1.309, 0.0),
]

DRIVE_SPEED = 6.0          # rad/s at the wheel, about 0.30 m/s
HOLD_S = 2.5
DRIVE_S = 2.5


def main():
    robot = Supervisor()
    ms = int(robot.getBasicTimeStep())
    dt = ms / 1000.0

    me = robot.getFromDef("ROBOT")
    if me is None:
        print("[vex] FATAL: DEF ROBOT missing", flush=True)
        sys.exit(1)

    dev = {}
    for n in ("arm1", "arm2", "drive_l", "drive_r"):
        dev[n] = robot.getDevice(n)
    for n in ("arm1_pos", "arm2_pos", "drive_l_pos", "drive_r_pos"):
        dev[n] = robot.getDevice(n)
        dev[n].enable(ms)
    for n in ("drive_l", "drive_r"):
        dev[n].setPosition(float("inf"))
        dev[n].setVelocity(0.0)

    def pose():
        p = me.getPosition()
        o = me.getOrientation()
        pitch = math.degrees(-math.asin(max(-1.0, min(1.0, o[6]))))
        return p[0], p[1], p[2], pitch

    rec = {"note": "arm geometry is the evaluator's; motor torque is the "
                   "published VEX IQ 0.414 N.m per motor, 2 per stage",
           "stage_torque_Nm": 0.828, "payload_kg": 0.10,
           "basic_time_step_ms": ms, "poses": []}

    robot.step(ms)
    for name, a1, a2 in POSES:
        # raise
        dev["arm1"].setPosition(a1)
        dev["arm2"].setPosition(a2)
        t = 0.0
        while robot.step(ms) != -1 and t < 3.0:
            t += dt

        x0, y0, z0, pitch0 = pose()
        s1 = dev["arm1_pos"].getValue()
        s2 = dev["arm2_pos"].getValue()

        # hold, no drive
        t = 0.0
        worst1 = worst2 = 0.0
        while robot.step(ms) != -1 and t < HOLD_S:
            t += dt
            worst1 = max(worst1, abs(dev["arm1_pos"].getValue() - a1))
            worst2 = max(worst2, abs(dev["arm2_pos"].getValue() - a2))
        x1, y1, z1, pitch1 = pose()
        h1 = dev["arm1_pos"].getValue()
        h2 = dev["arm2_pos"].getValue()

        # now drive forward with the arm still commanded to the same pose
        dev["drive_l"].setVelocity(DRIVE_SPEED)
        dev["drive_r"].setVelocity(DRIVE_SPEED)
        t = 0.0
        dworst1 = dworst2 = 0.0
        pitch_min = pitch_max = pitch1
        while robot.step(ms) != -1 and t < DRIVE_S:
            t += dt
            dworst1 = max(dworst1, abs(dev["arm1_pos"].getValue() - a1))
            dworst2 = max(dworst2, abs(dev["arm2_pos"].getValue() - a2))
            _, _, _, p = pose()
            pitch_min = min(pitch_min, p)
            pitch_max = max(pitch_max, p)
        dev["drive_l"].setVelocity(0.0)
        dev["drive_r"].setVelocity(0.0)
        x2, y2, z2, pitch2 = pose()
        d1 = dev["arm1_pos"].getValue()
        d2 = dev["arm2_pos"].getValue()

        t = 0.0
        while robot.step(ms) != -1 and t < 0.6:
            t += dt

        entry = {
            "pose": name,
            "commanded_rad": [a1, a2],
            "settled_rad": [round(s1, 6), round(s2, 6)],
            "held_rad": [round(h1, 6), round(h2, 6)],
            "sag_deg": [round(math.degrees(h1 - a1), 4),
                        round(math.degrees(h2 - a2), 4)],
            "worst_sag_during_hold_deg": [round(math.degrees(worst1), 4),
                                          round(math.degrees(worst2), 4)],
            "sag_while_driving_deg": [round(math.degrees(d1 - a1), 4),
                                      round(math.degrees(d2 - a2), 4)],
            "worst_sag_while_driving_deg": [round(math.degrees(dworst1), 4),
                                            round(math.degrees(dworst2), 4)],
            "base_pitch_holding_deg": round(pitch1, 4),
            "base_pitch_range_driving_deg": [round(pitch_min, 4),
                                             round(pitch_max, 4)],
            "base_creep_during_hold_m": round(math.hypot(x1 - x0, y1 - y0), 6),
            "base_travel_driving_m": round(math.hypot(x2 - x1, y2 - y1), 6),
            "base_lateral_drift_driving_m": round(abs(y2 - y1), 6),
        }
        rec["poses"].append(entry)
        print("[vex] %-7s sag hold %+.3f/%+.3f deg  sag driving %+.3f/%+.3f  "
              "pitch %+.2f (%.2f..%.2f)  creep %.4f m"
              % (name, entry["sag_deg"][0], entry["sag_deg"][1],
                 entry["sag_while_driving_deg"][0],
                 entry["sag_while_driving_deg"][1],
                 entry["base_pitch_holding_deg"], pitch_min, pitch_max,
                 entry["base_creep_during_hold_m"]), flush=True)

    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "arm_hold.json"), "w") as fh:
        json.dump(rec, fh, indent=1)
    print("[vex] wrote arm_hold.json", flush=True)
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
        print("[vex] FAILED " + tb, flush=True)
        raise
