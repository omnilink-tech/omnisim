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

"""hill_shot -- camera-bot that photographs the hill-walk ghost in 3D.

Runs alongside hill_ghost_preview in <robot>_hill_shot.wbt: it side-tracks the
translucent ghost as it walks up, over, and down the hill and saves PNGs at a
few sim times via Camera.saveImage (the only reliable headless still path here;
needs rendering ON -> launch with --minimize, NOT --no-window/--batch; see
reference_headless_render). Output -> _scratch/<robot>_hill_shots/. Quits the
sim itself when done.
"""
from __future__ import annotations
import math
import os
import sys

from omnisim import Supervisor

ROBOT = os.environ.get("HILL_ROBOT", "b2")
OUT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                   "..", "..", "..", "..", "_scratch", f"{ROBOT}_hill_shots"))
os.makedirs(OUT, exist_ok=True)
_LOG = open(os.path.join(OUT, "_shot.log"), "w", buffering=1)


def log(m):
    _LOG.write(m + "\n"); _LOG.flush()
    print(m, flush=True)


def _sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def _cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def _norm(a):
    n = math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]) or 1.0
    return (a[0]/n, a[1]/n, a[2]/n)


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    f = _norm(_sub(target, eye))
    if abs(f[0]*up[0]+f[1]*up[1]+f[2]*up[2]) > 0.999:
        up = (0.0, 1.0, 0.0)
    d = f[0]*up[0]+f[1]*up[1]+f[2]*up[2]
    uo = _norm((up[0]-d*f[0], up[1]-d*f[1], up[2]-d*f[2]))
    rt = _cross(f, uo)
    m = [[f[0], -rt[0], uo[0]], [f[1], -rt[1], uo[1]], [f[2], -rt[2], uo[2]]]
    trace = m[0][0]+m[1][1]+m[2][2]
    angle = math.acos(max(-1.0, min(1.0, (trace-1.0)/2.0)))
    if angle < 1e-6:
        return [0.0, 0.0, 1.0, 0.0]
    x, y, z = m[2][1]-m[1][2], m[0][2]-m[2][0], m[1][0]-m[0][1]
    n = math.sqrt(x*x+y*y+z*z)
    if n < 1e-6:
        return [0.0, 0.0, 1.0, angle]
    return [x/n, y/n, z/n, angle]


def main() -> int:
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())
    cam = robot.getDevice("cam")
    if cam is None:
        log("[hill_shot] no cam device"); return 1
    cam.enable(ts)

    me = robot.getSelf()
    ghost = robot.getFromDef(f"{ROBOT.upper()}_GHOST")
    if ghost is None:
        log(f"[hill_shot] no {ROBOT.upper()}_GHOST node"); return 1
    trans_f = me.getField("translation")
    rot_f = me.getField("rotation")

    # save at sim times spanning flat -> up -> crest -> down -> runout
    shot_t = [3, 6, 9, 11, 14, 17, 20]
    shots = {int(round(t / (ts / 1000.0))): f"{ROBOT}_hill_t{t:02d}s.png" for t in shot_t}
    last = max(shots) + 4
    log(f"[hill_shot] OUT={OUT} shots at t={shot_t}s (steps {sorted(shots)})")

    k = 0
    while robot.step(ts) != -1:
        gp = ghost.getPosition()                         # ghost world position
        # side view, slightly behind, looking at the robot on the hill
        eye = (gp[0] - 0.6, -4.6, gp[2] + 0.9)
        tgt = (gp[0] + 0.4, 0.0, gp[2] - 0.05)
        trans_f.setSFVec3f([float(v) for v in eye])
        rot_f.setSFRotation(look_at(eye, tgt))
        if k in shots:
            path = os.path.join(OUT, shots[k])
            robot.step(ts)                               # let the view settle
            try:
                cam.saveImage(path, 95)
                robot.step(ts)
                ok = os.path.exists(path)
                log(f"[hill_shot] t~{k*ts/1000:.0f}s saved {shots[k]} exists={ok} "
                    f"ghost_x={gp[0]:+.2f} ghost_z={gp[2]:.2f} "
                    f"size={os.path.getsize(path) if ok else 0}")
            except Exception as e:
                log(f"[hill_shot] saveImage FAILED: {type(e).__name__}: {e}")
        if k >= last:
            break
        k += 1
    log("[hill_shot] DONE")
    robot.simulationQuit(0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
