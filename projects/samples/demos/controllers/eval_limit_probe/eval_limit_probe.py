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

"""What does a URDF-imported motor report as its own position limits?

Four revolute joints spanning every case the importer distinguishes. For
each, the URDF's declared limits are compared against what the motor says
about itself, and then against what the joint actually enforces.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = os.path.join("O:", os.sep, "omnisim", ".tmp", "reply-evals-20260920")

DECLARED = {                       # name: (lower, upper) straight from the URDF
    "j_inside": (-2.0, 2.0),
    "j_full": (-6.283, 6.283),
    "j_asym": (-6.283, 0.785),
    "j_edge": (-3.1416, 3.1416),
}
KPI = math.pi - 0.01


def main():
    robot = Supervisor()
    ms = int(robot.getBasicTimeStep())
    dt = ms / 1000.0

    rows = []
    motors, sensors = {}, {}
    for jn in DECLARED:
        m = robot.getDevice(jn + "_motor")
        s = robot.getDevice(jn + "_sensor")
        if m is None:
            print("[probe] FATAL missing %s_motor" % jn, flush=True)
            sys.exit(1)
        if s is not None:
            s.enable(ms)
        motors[jn], sensors[jn] = m, s

    for jn, (lo, hi) in DECLARED.items():
        m = motors[jn]
        rb_lo, rb_hi = m.getMinPosition(), m.getMaxPosition()
        both_beyond = (lo <= -KPI and hi >= KPI)
        rows.append({
            "joint": jn,
            "urdf_lower": lo, "urdf_upper": hi,
            "readback_min": round(rb_lo, 6), "readback_max": round(rb_hi, 6),
            "both_limits_beyond_pi": both_beyond,
            "readback_matches_urdf": bool(abs(rb_lo - lo) < 1e-3
                                          and abs(rb_hi - hi) < 1e-3),
            "readback_is_zero": bool(rb_lo == 0.0 and rb_hi == 0.0),
        })

    # Now what is actually ENFORCED: drive each joint 1 rad past each end.
    robot.step(ms)
    for row in rows:
        jn = row["joint"]
        for end, target in (("upper", row["urdf_upper"] + 1.0),
                            ("lower", row["urdf_lower"] - 1.0)):
            for other in DECLARED:
                motors[other].setPosition(0.0)
            t = 0.0
            while robot.step(ms) != -1 and t < 1.0:
                t += dt
            motors[jn].setPosition(target)
            t = 0.0
            while robot.step(ms) != -1 and t < 3.0:
                t += dt
            ach = float(sensors[jn].getValue()) if sensors[jn] else float("nan")
            declared_end = row["urdf_%s" % end]
            row["enforced_%s" % end] = round(ach, 6)
            row["reaches_declared_%s" % end] = bool(
                abs(ach - declared_end) < 0.02)
            row["short_of_%s_by" % end] = round(abs(ach - declared_end), 6)

    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "limit_probe.json"), "w") as fh:
        json.dump(rows, fh, indent=1)

    print("[probe] %-10s %-18s %-20s %-8s %s"
          % ("joint", "URDF limits", "motor readback", "beyond?", "enforced"),
          flush=True)
    for r in rows:
        print("[probe] %-10s %-18s %-20s %-8s [%.3f .. %.3f]%s"
              % (r["joint"],
                 "%.3f .. %.3f" % (r["urdf_lower"], r["urdf_upper"]),
                 "%.3f .. %.3f" % (r["readback_min"], r["readback_max"]),
                 "yes" if r["both_limits_beyond_pi"] else "no",
                 r.get("enforced_lower", float("nan")),
                 r.get("enforced_upper", float("nan")),
                 "" if (r["reaches_declared_lower"] and r["reaches_declared_upper"])
                 else "   <-- TRAVEL LOST"), flush=True)
    sys.stdout.flush()


main()
