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

"""How many simulation steps per wall-clock second does this world sustain?

This is the number that matters for a LOCKSTEP autopilot bridge, where the
autopilot takes its clock from the simulator's timestamp and the exchange
rate is bounded by throughput rather than by the operating system's timer
quantum. It is NOT a real-time pacing measurement -- the engine is asked to
run as fast as it can.
"""
import json
import os
import sys
import time

from omnisim import Robot

OUT = os.path.join("O:", os.sep, "omnisim", ".tmp", "reply-evals-20260919")
WARMUP = 200
N = 4000


def main():
    robot = Robot()
    ms = int(robot.getBasicTimeStep())
    for _ in range(WARMUP):
        if robot.step(ms) == -1:
            break
    t0 = time.perf_counter()
    done = 0
    per = []
    for _ in range(N):
        a = time.perf_counter()
        if robot.step(ms) == -1:
            break
        per.append(time.perf_counter() - a)
        done += 1
    wall = time.perf_counter() - t0
    per.sort()
    rec = {
        "basic_time_step_ms": ms,
        "steps": done,
        "wall_s": round(wall, 4),
        "steps_per_wall_s": round(done / wall, 1) if wall else None,
        "sim_s_per_wall_s": round(done * ms / 1000.0 / wall, 3) if wall else None,
        "step_ms": {
            "mean": round(1000.0 * wall / done, 4) if done else None,
            "p50": round(1000.0 * per[len(per) // 2], 4) if per else None,
            "p95": round(1000.0 * per[int(len(per) * 0.95)], 4) if per else None,
            "p99": round(1000.0 * per[int(len(per) * 0.99)], 4) if per else None,
            "max": round(1000.0 * per[-1], 4) if per else None,
        },
    }
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "steprate.json"), "w") as fh:
        json.dump(rec, fh, indent=1)
    print("[rate] %d steps in %.3f s  =  %.1f steps/s  (%.2fx real time at a "
          "%d ms step)" % (done, wall, rec["steps_per_wall_s"],
                           rec["sim_s_per_wall_s"], ms), flush=True)
    print("[rate] per-step ms: mean %.3f  p50 %.3f  p95 %.3f  p99 %.3f  "
          "max %.3f" % (rec["step_ms"]["mean"], rec["step_ms"]["p50"],
                        rec["step_ms"]["p95"], rec["step_ms"]["p99"],
                        rec["step_ms"]["max"]), flush=True)
    sys.stdout.flush()


main()
