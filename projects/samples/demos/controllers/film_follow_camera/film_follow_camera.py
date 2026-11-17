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

"""Film a run from a camera that rides with a moving part: film_camera, plus a follow.

    controllerArgs [ "<period_ms>" "<DEF to follow>" "<dx>" "<dy>" "<dz>" ]
    OMNISIM_FILM_DIR=<dir>      frames land in <dir>/<camera name>/<sim ms>.jpg

Every step this supervisor robot puts itself at the followed node's WORLD position plus (dx, dy, dz)
(metres, world frame); its orientation is the one the world gave it (aimed from that offset at the node),
so the node stays at the same place in the frame. The robot has no physics and no collider: it touches
nothing it films. A machine's moving head that a fixed camera loses behind its own gantry is the use.
"""
import os
import sys

from omnisim import Camera, Supervisor

sup = Supervisor()
step_ms = int(sup.getBasicTimeStep())
args = sys.argv[1:]
period_ms = int(args[0]) if args else 125
def find_def(name):
    """getFromDef, then a walk of the whole scene for a node whose DEF is `name`'s last part (a DEF inside a Robot or
    behind un-DEF'd Poses is not reachable by a dotted path)."""
    node = sup.getFromDef(name)
    if node is not None:
        return node
    want = name.split(".")[-1]
    stack = [sup.getRoot()]
    while stack:
        n = stack.pop()
        try:
            if n.getDef() == want:
                return n
        except Exception:
            pass
        for field in ("children", "endPoint"):
            f = n.getField(field)
            if f is None:
                continue
            if field == "children":
                stack.extend(f.getMFNode(i) for i in range(f.getCount()))
            else:
                ep = f.getSFNode()
                if ep is not None:
                    stack.append(ep)
    return None


follow = find_def(args[1]) if len(args) > 1 else None
offset = [float(v) for v in args[2:5]] if len(args) > 4 else [0.15, -0.15, 0.15]
me = sup.getSelf()
tr = me.getField("translation")
out = os.environ.get("OMNISIM_FILM_DIR", "")
cams = [d for d in (sup.getDeviceByIndex(i) for i in range(sup.getNumberOfDevices())) if isinstance(d, Camera)]
for cam in cams:
    cam.enable(step_ms)
    if out:
        os.makedirs(os.path.join(out, cam.getName()), exist_ok=True)
print(f"[film_follow_camera] {len(cams)} camera(s) every {period_ms} sim-ms, following "
      f"{args[1] if len(args) > 1 else '(nothing)'} at {offset}, -> {out or '(off)'}", flush=True)
if follow is None:
    print("[film_follow_camera] the node to follow is not in the world: the camera stays where it is", flush=True)

next_ms = 0
while sup.step(step_ms) != -1:
    if follow is not None:
        p = follow.getPosition()
        tr.setSFVec3f([p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]])
    now_ms = int(round(sup.getTime() * 1000))
    if not out or now_ms < next_ms:
        continue
    for cam in cams:
        cam.saveImage(os.path.join(out, cam.getName(), f"{now_ms:09d}.jpg"), 92)
    next_ms = now_ms + period_ms
