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

"""duel_tracker — supervisor that logs two fighters' kinematics to CSV
every K basic-time-steps. One-shot tracing aid; safe to leave in a
world (read-only, no physics writes).

Per default it looks for DEFs "RED" and "BLUE". To trace a different
matchup, set the supervisor's customData to a JSON object like:

  {"fighters": ["GRAVEDIGGER", "BITEBOT"]}

(DEFs must be uppercase to match Webots convention.)

Sample row: t_s, A_{x,y,z,vx,vy,vz,speed}, B_{...}, gap_xy
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

from omnisim import Supervisor


def main() -> int:
    sup = Supervisor()
    step_ms = int(sup.getBasicTimeStep())

    # Default: RED vs BLUE. Override via customData.
    fighter_defs = ["RED", "BLUE"]
    try:
        raw = sup.getCustomData()
        if raw:
            cfg = json.loads(raw)
            fdefs = cfg.get("fighters")
            if isinstance(fdefs, list) and len(fdefs) >= 2:
                fighter_defs = [str(x) for x in fdefs[:2]]
    except Exception as exc:
        sys.stderr.write(
            f"[duel_tracker] customData parse failed: {exc}; using "
            f"default RED/BLUE\n"
        )

    red = sup.getFromDef(fighter_defs[0])
    blue = sup.getFromDef(fighter_defs[1])
    if red is None or blue is None:
        sys.stderr.write(
            f"[duel_tracker] DEFs {fighter_defs!r} not found; exiting\n"
        )
        return 1

    # Trace file path: env var wins; else customData["trace_path"]; else
    # default in _scratch/ named after the two fighter DEFs.
    default_name = (
        f"trace_{fighter_defs[0].lower()}_vs_{fighter_defs[1].lower()}.csv"
    )
    cfg_path = None
    try:
        raw = sup.getCustomData()
        if raw:
            cfg = json.loads(raw)
            cfg_path = cfg.get("trace_path")
    except Exception:
        pass
    out_path = Path(
        os.environ.get(
            "DUEL_TRACE",
            cfg_path or os.path.join(
                os.environ.get("OMNISIM_HOME")
                or str(Path(__file__).resolve().parents[4]),
                "_scratch", default_name),
        )
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fh = out_path.open("w", buffering=1, encoding="utf-8")
    a, b = fighter_defs[0].lower(), fighter_defs[1].lower()
    fh.write(
        f"t_s,{a}_x,{a}_y,{a}_z,{a}_vx,{a}_vy,{a}_vz,{a}_speed,"
        f"{b}_x,{b}_y,{b}_z,{b}_vx,{b}_vy,{b}_vz,{b}_speed,gap\n"
    )
    sys.stderr.write(f"[duel_tracker] writing {out_path}\n")

    sample_every = max(1, int(os.environ.get("DUEL_TRACE_EVERY", "5")))
    n = 0
    while sup.step(step_ms) != -1:
        n += 1
        if n % sample_every:
            continue
        rp = red.getPosition()
        rv = red.getVelocity()
        bp = blue.getPosition()
        bv = blue.getVelocity()
        t = sup.getTime()
        rs = math.hypot(rv[0], rv[1])
        bs = math.hypot(bv[0], bv[1])
        gap = math.hypot(rp[0] - bp[0], rp[1] - bp[1])
        fh.write(
            f"{t:.3f},"
            f"{rp[0]:.3f},{rp[1]:.3f},{rp[2]:.3f},"
            f"{rv[0]:.3f},{rv[1]:.3f},{rv[2]:.3f},{rs:.3f},"
            f"{bp[0]:.3f},{bp[1]:.3f},{bp[2]:.3f},"
            f"{bv[0]:.3f},{bv[1]:.3f},{bv[2]:.3f},{bs:.3f},"
            f"{gap:.3f}\n"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
