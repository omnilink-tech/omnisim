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

"""A published GPIO waveform, driving something with mass.

Replays the asymmetric timing profile published in the TORE Embedded Labs
README -- 1000 ms HIGH, 500 ms LOW, cycle counter -- but instead of an LED the
line drives a hinged indicator arm with real inertia, so the question stops
being "did the pin toggle on time" and becomes "did the mechanism finish
moving before the line changed again".

Three factors, one at a time:

  EVAL_VEL        actuator velocity limit, rad/s.  Swept to find where the
                  500 ms LOW window stops being long enough.
  EVAL_DELAY_MS   delay applied to exactly ONE rising edge, mid-run.
  EVAL_SERIAL     if 1, charge each cycle the wall time its four telemetry
                  lines cost at 115200 baud, which the published loop does
                  not account for.

Per transition it records the commanded state, when it was commanded, where
the arm actually was when the line next changed, and whether the stroke
completed.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = r"O:\omnisim\.tmp\reply-evals-20260913\tore_rig"

# published profile, verbatim from the README's loop()
HIGH_MS = 1000
LOW_MS = 500
CYCLES = 8

# the arm's two commanded positions
POS_HIGH = 0.60          # rad
POS_LOW = 0.0
SETTLED_RAD = 0.005      # within 0.29 deg counts as arrived

# 115200 baud, 8N1 -> 10 bits per character.  The loop prints four lines a
# cycle; measured against the README's own sample lines.
SERIAL_LINES = [
    "[STATE] LED ON  | Timestamp: 191013 ms",
    "[STATE] LED OFF | Timestamp: 192013 ms",
    "[CYCLE] Total Completed: 128",
    "--------------------------------------",
]
SERIAL_MS = sum((len(s) + 2) * 10 for s in SERIAL_LINES) / 115200.0 * 1000.0


def main():
    vel = float(os.environ.get("EVAL_VEL", "1.7"))
    delay_ms = float(os.environ.get("EVAL_DELAY_MS", "0"))
    serial_on = os.environ.get("EVAL_SERIAL", "0") == "1"
    tag = os.environ.get("EVAL_TAG", "run1")
    delay_at_cycle = int(os.environ.get("EVAL_DELAY_AT", "4"))

    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    motor = robot.getDevice("arm_motor")
    sensor = robot.getDevice("arm_sensor")
    sensor.enable(dt_ms)
    motor.setVelocity(vel)
    motor.setPosition(POS_LOW)

    # build the event schedule exactly as the published loop produces it
    events = []
    t = 0.0
    for c in range(CYCLES):
        extra = delay_ms if (c == delay_at_cycle and delay_ms > 0) else 0.0
        events.append({"cycle": c, "edge": "rising", "state": "HIGH",
                       "t_ms": t + extra, "delayed_by_ms": extra})
        t += extra + HIGH_MS
        events.append({"cycle": c, "edge": "falling", "state": "LOW",
                       "t_ms": t, "delayed_by_ms": 0.0})
        t += LOW_MS
        if serial_on:
            t += SERIAL_MS
    t_end = t + 500.0

    robot.step(dt_ms)
    now = 0.0
    idx = 0
    log = []
    pending = None

    while robot.step(dt_ms) != -1 and now < t_end:
        now += dt * 1000.0
        while idx < len(events) and now >= events[idx]["t_ms"]:
            e = events[idx]
            idx += 1
            pos = sensor.getValue()
            if pending is not None:
                target = pending["target"]
                start = pending["start_pos"]
                span = abs(target - start)
                reached = abs(pos - target) <= SETTLED_RAD
                frac = 1.0 if span < 1e-9 else min(
                    1.0, abs(pos - start) / span)
                pending.update({
                    "window_ms": round(now - pending["t_cmd_ms"], 3),
                    "pos_at_next_edge": round(pos, 6),
                    "completed": bool(reached),
                    "fraction_of_stroke": round(frac, 6),
                    "shortfall_rad": round(abs(target - pos), 6),
                })
                log.append(pending)
            target = POS_HIGH if e["state"] == "HIGH" else POS_LOW
            motor.setPosition(target)
            pending = {
                "cycle": e["cycle"], "edge": e["edge"], "state": e["state"],
                "t_cmd_ms": round(e["t_ms"], 3),
                "delayed_by_ms": e["delayed_by_ms"],
                "start_pos": round(pos, 6), "target": target,
            }

    if pending is not None:
        pos = sensor.getValue()
        span = abs(pending["target"] - pending["start_pos"])
        pending.update({
            "window_ms": round(now - pending["t_cmd_ms"], 3),
            "pos_at_next_edge": round(pos, 6),
            "completed": bool(abs(pos - pending["target"]) <= SETTLED_RAD),
            "fraction_of_stroke": 1.0 if span < 1e-9 else round(
                min(1.0, abs(pos - pending["start_pos"]) / span), 6),
            "shortfall_rad": round(abs(pending["target"] - pos), 6),
        })
        log.append(pending)

    highs = [r for r in log if r["state"] == "HIGH"]
    lows = [r for r in log if r["state"] == "LOW"]
    result = {
        "tag": tag,
        "velocity_rad_s": vel,
        "delay_ms": delay_ms,
        "delay_at_cycle": delay_at_cycle,
        "serial_charged": serial_on,
        "serial_ms_per_cycle": round(SERIAL_MS, 4),
        "published_profile": {"high_ms": HIGH_MS, "low_ms": LOW_MS,
                              "cycles": CYCLES},
        "stroke_rad": POS_HIGH,
        "basic_time_step_ms": dt_ms,
        "transitions": log,
        "summary": {
            "high_strokes_completed": sum(1 for r in highs if r["completed"]),
            "high_strokes": len(highs),
            "low_strokes_completed": sum(1 for r in lows if r["completed"]),
            "low_strokes": len(lows),
            "worst_low_fraction": round(
                min([r["fraction_of_stroke"] for r in lows], default=1.0), 6),
            "worst_high_fraction": round(
                min([r["fraction_of_stroke"] for r in highs], default=1.0), 6),
            "nominal_cycle_ms": HIGH_MS + LOW_MS,
            "actual_cycle_ms": round(
                HIGH_MS + LOW_MS + (SERIAL_MS if serial_on else 0.0), 4),
        },
    }
    os.makedirs(OUT, exist_ok=True)
    name = f"gpio_vel{vel}_delay{int(delay_ms)}_serial{int(serial_on)}_{tag}.json"
    with open(os.path.join(OUT, name), "w") as f:
        json.dump(result, f, indent=1)
    s = result["summary"]
    print(f"[eval] wrote {name}", flush=True)
    print(f"[eval] vel={vel} delay={delay_ms}ms serial={serial_on} "
          f"HIGH {s['high_strokes_completed']}/{s['high_strokes']} "
          f"LOW {s['low_strokes_completed']}/{s['low_strokes']} "
          f"worst_low_frac={s['worst_low_fraction']}", flush=True)
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
