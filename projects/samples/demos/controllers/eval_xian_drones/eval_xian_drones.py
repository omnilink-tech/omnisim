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

"""At what centre separation do two of these drones actually touch?

RaceEnv.py scores a drone-drone collision as `centre distance < 2 *
COLLISION_R`, with COLLISION_R = 0.1 from single_test_drone.yaml. The same
file declares `arm: 0.2`, which puts each motor 0.2 m from the hub.

Two geometries are swept side by side over identical separations:

  PROXY  the collision body his checker uses  (cylinder r 0.1, h 0.1)
  FRAME  an X-frame quadrotor with arms out to his 0.2 m

Both are driven kinematically, so the only variable is the separation and
the only observable is whether the contact instrument fires. The gap
between the two onsets is the blind spot in the proxy.

Two arm phasings are swept for FRAME, because an X-frame's extent depends
on which way it is facing. At yaw 0 the arms sit at 45 degrees either side
of the approach axis and the nearest structure is 0.2*cos45 = 0.1414 out;
at yaw 45 an arm points straight at the other vehicle and reaches the full
0.2 m. The second is the worst case.
"""
import json
import math
import os
import sys

from omnisim import Supervisor

OUT = os.path.join("O:", os.sep, "omnisim", ".tmp", "reply-evals-20260915",
                   "xian")

COLL_R = 0.1
ARM = 0.2
HIS_THRESHOLD = 2 * COLL_R          # 0.20 m, his collision criterion

# centre separations to sweep, metres
SEPS = [0.10, 0.15, 0.18, 0.20, 0.22, 0.25, 0.30, 0.34, 0.38,
        0.40, 0.42, 0.45, 0.50, 0.60]

# The X-frame's arms sit at 45/135/225/315 degrees in its own frame, so at
# yaw 0 NO arm points along the approach axis (+x): the nearest structure is
# an arm tip at 0.2*cos45 = 0.1414 in x. Rotating the vehicle by 45 degrees
# swings an arm to point straight down the approach axis, reaching the full
# 0.2 m. Naming these the wrong way round would invert the whole result.
PAIRS = [("PROXY_his_collision_body", "PROXY_A", "PROXY_B", 0.0, 0.0),
         ("FRAME_arms_at_45deg_to_approach", "FRAME_A", "FRAME_B", 20.0, 0.0),
         ("FRAME_arm_pointing_at_other", "FRAME_A", "FRAME_B", 20.0,
          math.pi / 4)]


def main():
    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())

    nodes = {}
    for d in ("PROXY_A", "PROXY_B", "FRAME_A", "FRAME_B"):
        n = robot.getFromDef(d)
        if n is None:
            print("[xian] FATAL: DEF %s not found" % d, flush=True)
            sys.exit(1)
        nodes[d] = n

    def touching(node):
        try:
            pts = node.getContactPoints(True)
        except TypeError:
            pts = node.getContactPoints()
        except Exception:
            pts = []
        return len(pts or [])

    def place(d, x, y, z, yaw):
        nodes[d].getField("translation").setSFVec3f([x, y, z])
        fr = nodes[d].getField("rotation")
        if fr is not None:
            fr.setSFRotation([0.0, 0.0, 1.0, yaw])

    rec = {"source": "single_test_drone.yaml: m 1.0, arm 0.2, radius 0.1",
           "his_collision_threshold_m": HIS_THRESHOLD,
           "arm_length_m": ARM, "sweeps": []}

    robot.step(dt)
    for label, da, db, ybase, yaw in PAIRS:
        rows = []
        for sep in SEPS:
            # park the other pair well away so it cannot contribute contacts
            for d in ("PROXY_A", "PROXY_B", "FRAME_A", "FRAME_B"):
                place(d, 0.0, -60.0 - 3.0 * list(nodes).index(d), 2.0, 0.0)
            # The pose must be re-asserted EVERY step. Two overlapping
            # dynamic bodies are thrown apart by the contact solver within a
            # step or two, so placing them once and then stepping measures
            # how fast they separated, not whether they touched.
            na = nb = 0
            for i in range(14):
                place(da, 0.0, ybase, 2.0, yaw)
                place(db, sep, ybase, 2.0, yaw)
                robot.step(dt)
                if i >= 4:                 # let the contact set populate
                    na = max(na, touching(nodes[da]))
                    nb = max(nb, touching(nodes[db]))
            hit = (na + nb) > 0
            rows.append({"separation_m": round(sep, 4),
                         "contact_reported": bool(hit),
                         "points_a": na, "points_b": nb,
                         "his_checker_says_collision": sep < HIS_THRESHOLD})
            print("[xian] %-18s sep %.3f  contact=%-5s  his_checker=%s"
                  % (label, sep, hit, sep < HIS_THRESHOLD), flush=True)
        onset = None
        for r in sorted(rows, key=lambda z: -z["separation_m"]):
            if r["contact_reported"]:
                onset = r["separation_m"]
                break
        # the largest separation at which contact is still reported
        largest = max([r["separation_m"] for r in rows
                       if r["contact_reported"]] or [0.0])
        rec["sweeps"].append({"pair": label, "rows": rows,
                              "largest_separation_with_contact_m": largest})
        print("[xian] %-18s -> contact out to %.3f m (his checker fires "
              "only below %.3f m)" % (label, largest, HIS_THRESHOLD),
              flush=True)

    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "proxy_vs_frame.json"), "w") as fh:
        json.dump(rec, fh, indent=1)
    print("[xian] wrote proxy_vs_frame.json", flush=True)
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
        print("[xian] FAILED " + tb, flush=True)
        raise
