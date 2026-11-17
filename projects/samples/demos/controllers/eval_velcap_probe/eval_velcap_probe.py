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

"""Does a position-controlled RotationalMotor honour its velocity limit?

Commands a single 1.0 rad step and measures the peak and mean slew rate, for
the declared world-file maxVelocity and for a runtime setVelocity(). If the
limit is honoured, the peak rate should not exceed it.
"""
import json
import os
import sys

from omnisim import Supervisor

OUT = r"O:\omnisim\.tmp\reply-evals-20260913\velcap"


def main():
    world_cap = float(os.environ.get("EVAL_WORLD_CAP", "30"))
    rt_cap = os.environ.get("EVAL_RUNTIME_CAP", "")
    tag = os.environ.get("EVAL_TAG", "run1")

    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0
    motor = robot.getDevice("arm_motor")
    sensor = robot.getDevice("arm_sensor")
    sensor.enable(dt_ms)

    if rt_cap:
        motor.setVelocity(float(rt_cap))
    motor.setPosition(0.0)
    for _ in range(120):
        robot.step(dt_ms)

    start = sensor.getValue()
    motor.setPosition(1.0)
    rates = []
    prev = start
    t = 0.0
    arrived_at = None
    while robot.step(dt_ms) != -1 and t < 6.0:
        t += dt
        cur = sensor.getValue()
        rates.append(abs(cur - prev) / dt)
        prev = cur
        if arrived_at is None and abs(cur - 1.0) < 0.005:
            arrived_at = t
    peak = max(rates) if rates else 0.0
    span = abs(prev - start)
    result = {
        "tag": tag,
        "world_maxVelocity": world_cap,
        "runtime_setVelocity": float(rt_cap) if rt_cap else None,
        "commanded_step_rad": 1.0,
        "start_rad": round(start, 6),
        "end_rad": round(prev, 6),
        "travelled_rad": round(span, 6),
        "arrived_after_s": arrived_at,
        "peak_rate_rad_s": round(peak, 5),
        "mean_rate_while_moving_rad_s": round(
            sum(r for r in rates if r > 1e-6) /
            max(1, sum(1 for r in rates if r > 1e-6)), 5),
        "limit_honoured": bool(peak <= world_cap * 1.05 if not rt_cap
                               else peak <= float(rt_cap) * 1.05),
    }
    os.makedirs(OUT, exist_ok=True)
    name = f"velcap_world{world_cap}_rt{rt_cap or 'none'}_{tag}.json"
    with open(os.path.join(OUT, name), "w") as f:
        json.dump(result, f, indent=1)
    print(f"[eval] world_cap={world_cap} runtime={rt_cap or 'none'} "
          f"peak={peak:.4f} rad/s arrived={arrived_at} "
          f"honoured={result['limit_honoured']}", flush=True)
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, "tb.txt"), "a") as f:
            f.write(traceback.format_exc())
        print("[eval] FAILED " + traceback.format_exc(), flush=True)
        raise
