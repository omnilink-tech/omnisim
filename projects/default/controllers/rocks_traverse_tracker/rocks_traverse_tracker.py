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

"""Tracks the Husky's pose across `husky_rocks_traverse.wbt`.

Polls the DEF HUSKY robot's position every basicTimeStep and writes a
JSON trajectory to `tmp_rocks_traverse_trajectory.json` at the repo
root. The headless runner shuts the simulation down when its
``--duration`` elapses, so we just write each sample as we collect it
and rely on the supervisor reaching its stop-criteria for the summary.

Stop criteria (whichever fires first):
* duration_s reached (default 60s)
* husky velocity magnitude < 0.02 m/s for ≥ 3 consecutive seconds
  (stalled — caught by a rock or the boulder cluster)
* husky x position ≥ 6.5 m (cleared the field)
"""

from __future__ import annotations

import json
import math
import os
import sys

from omnisim import Supervisor


DURATION_S = 60.0
STALL_VEL_THRESHOLD = 0.02      # m/s
STALL_HOLD_S = 3.0              # how long velocity must stay low
CLEAR_X = 6.5                   # x-position considered "made it"


def main() -> int:
    # Redirect prints to a known file so we can see what happened even if
    # Webots' controller-stdout routing is unreliable.
    dbg_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..",
        "tmp_rocks_tracker_dbg.log",
    ))
    try:
        sys.stdout = open(dbg_path, "w", buffering=1)
    except OSError:
        pass
    print(f"[tracker] starting at {dbg_path}", flush=True)

    print(f"[tracker] importing Supervisor done", flush=True)
    sup = Supervisor()
    print(f"[tracker] Supervisor() returned", flush=True)
    ts = int(sup.getBasicTimeStep())
    print(f"[tracker] basicTimeStep={ts}", flush=True)

    husky = sup.getFromDef("HUSKY")
    print(f"[tracker] getFromDef returned {husky!r}", flush=True)
    if husky is None:
        print("[tracker] FATAL: no DEF HUSKY in world", file=sys.stderr)
        return 1
    print(f"[tracker] found HUSKY", flush=True)

    out_path = os.environ.get("ROCKS_TRACKER_OUT") or os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..",
        "tmp_rocks_traverse_trajectory.json",
    )
    out_path = os.path.abspath(out_path)

    samples: list[dict] = []
    last_pos = husky.getPosition()
    last_t = sup.getTime()
    stalled_since: float | None = None
    stop_reason = "duration"
    last_flush_t = 0.0

    def flush(reason: str) -> None:
        summary = {
            "stop_reason": reason,
            "samples": len(samples),
            "duration_s": round(samples[-1]["t"] if samples else 0.0, 3),
            "final_pos": {
                "x": samples[-1]["x"] if samples else None,
                "y": samples[-1]["y"] if samples else None,
            },
            "max_x": max((s["x"] for s in samples), default=None),
            "trajectory": samples,
        }
        try:
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2)
        except OSError as e:
            print(f"[tracker] write failed: {e}", file=sys.stderr)

    print(f"[tracker] logging to {out_path}", flush=True)

    while sup.step(ts) != -1:
        t = sup.getTime()
        pos = husky.getPosition()
        dt = max(t - last_t, 1e-6)
        vx = (pos[0] - last_pos[0]) / dt
        vy = (pos[1] - last_pos[1]) / dt
        v = math.hypot(vx, vy)

        # Orientation as 3x3 rotation matrix, row-major:
        #   [R00 R01 R02 R10 R11 R12 R20 R21 R22]
        # Yaw   = atan2(R10, R00)        (rot around world Z)
        # Pitch = asin(-R20)             (nose up/down)
        # Roll  = atan2(R21, R22)        (lean left/right)
        rot = husky.getOrientation()
        try:
            pitch = math.asin(max(-1.0, min(1.0, -rot[6])))
            roll = math.atan2(rot[7], rot[8])
        except (IndexError, ValueError):
            pitch = roll = 0.0
        samples.append({
            "t": round(t, 3),
            "x": round(pos[0], 4),
            "y": round(pos[1], 4),
            "z": round(pos[2], 4),
            "pitch_deg": round(math.degrees(pitch), 2),
            "roll_deg": round(math.degrees(roll), 2),
            "v": round(v, 4),
        })

        if v < STALL_VEL_THRESHOLD:
            if stalled_since is None:
                stalled_since = t
            elif t - stalled_since > STALL_HOLD_S:
                stop_reason = "stalled"
                break
        else:
            stalled_since = None

        if pos[0] >= CLEAR_X:
            stop_reason = "cleared"
            break

        if t >= DURATION_S:
            stop_reason = "duration"
            break

        last_pos = pos
        last_t = t

        # Flush every 2s of sim time so partial data survives if Webots
        # terminates us before the loop exits cleanly.
        if t - last_flush_t >= 2.0:
            flush("in_progress")
            last_flush_t = t

    flush(stop_reason)
    print(f"[tracker] wrote {len(samples)} samples; stop={stop_reason}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
