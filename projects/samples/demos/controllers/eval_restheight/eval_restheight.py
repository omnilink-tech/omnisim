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

"""Where does a rig actually come to rest, and on what?

A robot that settles at a plausible-looking height is not evidence that the
right bodies are touching the floor. This reports the authored height, the
rested height, and the gap, for the robot and for each named Solid, so the
difference between 'resting on its wheels' and 'resting on its wrapper' is
a number rather than an impression.
"""
import json
import os
import sys

from omnisim import Supervisor

OUT = os.path.join("O:", os.sep, "omnisim", ".tmp", "reply-evals-20260919")
RUN_S = 5.0


def main():
    robot = Supervisor()
    ms = int(robot.getBasicTimeStep())
    dt = ms / 1000.0
    tag = sys.argv[sys.argv.index("--tag") + 1] if "--tag" in sys.argv else "rest"

    me = robot.getSelf()
    found = {}

    def walk(node, depth=0):
        if node is None or depth > 12:
            return
        try:
            f = node.getField("name")
            nm = f.getSFString() if f is not None else None
        except Exception:
            nm = None
        if nm and nm not in found:
            found[nm] = node
        for fld in ("children", "endPoint"):
            try:
                ff = node.getField(fld)
            except Exception:
                ff = None
            if ff is None:
                continue
            try:
                if fld == "endPoint":
                    walk(ff.getSFNode(), depth + 1)
                else:
                    for i in range(ff.getCount()):
                        walk(ff.getMFNode(i), depth + 1)
            except Exception:
                pass

    walk(robot.getRoot())
    z0 = {k: round(float(v.getPosition()[2]), 6) for k, v in found.items()}
    z0["__self__"] = round(float(me.getPosition()[2]), 6)

    t = 0.0
    while robot.step(ms) != -1 and t < RUN_S:
        t += dt

    z1 = {k: round(float(v.getPosition()[2]), 6) for k, v in found.items()}
    z1["__self__"] = round(float(me.getPosition()[2]), 6)

    rec = {"tag": tag, "authored_z": z0, "rested_z": z1,
           "delta_z": {k: round(z1[k] - z0.get(k, 0.0), 6) for k in z1}}
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "rest_%s.json" % tag), "w") as fh:
        json.dump(rec, fh, indent=1)
    print("[rest] %s" % tag, flush=True)
    for k in sorted(z1):
        print("[rest]   %-24s authored %9.5f  rested %9.5f  delta %+9.5f"
              % (k, z0.get(k, float('nan')), z1[k], rec["delta_z"][k]),
              flush=True)
    sys.stdout.flush()


main()
