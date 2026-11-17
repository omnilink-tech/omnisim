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

"""Record, per tick, the simulator's sim clock against this process's wall clock.

Hardware-in-the-loop needs OmniSim to track WALL clock: a flight controller, a
CAN bus or a robot arm on the other end of a cable has no notion of sim time and
will not wait. Nothing in this repo has ever measured whether the engine does
that, so this controller is the primary instrument -- everything else is
analysis of what it records.

A controller can only see sim time (``robot.getTime()``); libController exposes
no wall-clock or real-time-factor accessor, so the wall side has to come from
this process's own ``time.perf_counter()``. The pairing of the two, sampled at
the same point in every tick, is the whole measurement.

The instrument must not perturb what it measures. Per tick this does exactly
two clock reads and two stores into a list that was sized up front; nothing is
formatted, serialised or written until the loop has finished.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import sys
import time

from omnisim import Supervisor


def parse_args():
    # add_help=False + parse_known_args because the engine appends its own
    # arguments after the ones controllerArgs supplies, and an unrecognised
    # engine argument must not abort the controller.
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--ticks", type=int, default=1200)
    ap.add_argument("--out", default="hil_timing_probe.json")
    ap.add_argument("--step-ms", type=int, default=0)
    args, _unknown = ap.parse_known_args()

    # Environment overrides the world's controllerArgs. controllerArgs are baked
    # into the .omniworld, so a runner sweeping tick counts or step sizes would
    # otherwise have to rewrite the world between runs -- which would change the
    # thing being measured.
    if os.environ.get("OMNISIM_HIL_TICKS"):
        args.ticks = int(os.environ["OMNISIM_HIL_TICKS"])
    if os.environ.get("OMNISIM_HIL_OUT"):
        args.out = os.environ["OMNISIM_HIL_OUT"]
    if os.environ.get("OMNISIM_HIL_STEP_MS"):
        args.step_ms = int(os.environ["OMNISIM_HIL_STEP_MS"])
    return args


def main() -> int:
    args = parse_args()

    robot = Supervisor()
    basic_ms = robot.getBasicTimeStep()

    # The truncation hazard this instrument exists to catch: the engine paces
    # with QTimer::start(int) but basicTimeStep is a double, so a fractional
    # value is truncated on the wall-clock side while the sim clock still
    # advances by the full fractional amount. Recorded, not corrected -- the
    # analysis needs to see the value the engine actually had.
    basic_is_integer = float(basic_ms).is_integer()

    # Keep the step request an exact integer multiple of the basic time step:
    # robot.step(N) is a sim-time DEADLINE satisfied at the first basic-step
    # boundary at or past requestTime + N, so a non-multiple silently measures
    # a different interval than the one asked for.
    step_ms = args.step_ms if args.step_ms > 0 else int(basic_ms)
    if step_ms < basic_ms:
        step_ms = int(basic_ms) if basic_is_integer else int(basic_ms) + 1

    n = max(1, args.ticks)
    # Preallocated: no list growth, no reallocation inside the timed loop.
    sim_s = [0.0] * n
    wall_s = [0.0] * n

    perf = time.perf_counter
    get_time = robot.getTime
    step = robot.step

    t_start = perf()
    sim_start = get_time()

    recorded = 0
    for i in range(n):
        if step(step_ms) == -1:
            break
        wall_s[i] = perf()
        sim_s[i] = get_time()
        recorded = i + 1

    t_end = perf()

    clock = time.get_clock_info("perf_counter")
    payload = {
        "schema": "omnisim-hil/timing-probe/1",
        "basic_time_step_ms": float(basic_ms),
        "basic_time_step_is_integer": basic_is_integer,
        "step_ms_requested": step_ms,
        "ticks_requested": n,
        "ticks_recorded": recorded,
        "wall_start_s": t_start,
        "wall_end_s": t_end,
        "sim_start_s": sim_start,
        "perf_counter_resolution_s": clock.resolution,
        "perf_counter_monotonic": clock.monotonic,
        "controller_python": platform.python_version(),
        "controller_executable": sys.executable,
        "samples": [[sim_s[i], wall_s[i]] for i in range(recorded)],
    }

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(payload, fh)

    sys.stdout.write(
        "[hil_timing_probe] recorded %d/%d ticks, step_ms=%d, basicTimeStep=%s -> %s\n"
        % (recorded, n, step_ms, basic_ms, out)
    )
    sys.stdout.flush()

    # Quit the engine rather than leaving it running headless with a finished
    # controller: an abandoned omnisim-bin is not reaped by the harness that
    # spawned it, and a parked process holds its auto-scanned port.
    robot.simulationQuit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
