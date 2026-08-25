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

"""walk_shot -- generic camera-bot that photographs a deployed RL robot.

Runs alongside a robot's normal deploy controller in a *_walk_shot.wbt world.
Chase-tracks the robot (DEF given by env SHOT_DEF) and saves PNGs at several
sim times via Camera.saveImage -- the only reliable headless still path
(exportImage / --no-window / --batch give grey). Rendering must be ON
(launch with --minimize, NOT --no-window/--batch).

Env knobs:
  SHOT_DEF    DEF name of the robot node to chase     (default "G1")
  SHOT_OUT    subdir under _scratch/ for the PNGs      (default "walk_shots")
  SHOT_PREFIX file prefix                              (default "shot")
  SHOT_TIMES  comma sim-times (s) to capture           (default "4,7,11,15")
  SHOT_OFFS   "dx,dy,cz,tz" chase offset + target z    (default "2.6,-1.9,1.15,0.7")
"""
from __future__ import annotations
import math
import os
import sys

from omnisim import Supervisor

DEFNAME = os.environ.get("SHOT_DEF", "G1")
OUTSUB = os.environ.get("SHOT_OUT", "walk_shots")
PREFIX = os.environ.get("SHOT_PREFIX", "shot")
TIMES = [float(x) for x in os.environ.get("SHOT_TIMES", "4,7,11,15").split(",")]
_offs = [float(x) for x in os.environ.get("SHOT_OFFS", "2.6,-1.9,1.15,0.7").split(",")]
DX, DY, CZ, TZ = _offs

OUT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                   "..", "..", "..", "..", "_scratch", OUTSUB))
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
        log("[walk_shot] no cam device"); return 1
    cam.enable(ts)
    me = robot.getSelf()
    tgt = robot.getFromDef(DEFNAME)
    if tgt is None:
        log(f"[walk_shot] no DEF {DEFNAME} node"); return 1
    shots = {int(round(t / (ts / 1000.0))): f"{PREFIX}_t{int(t)}s.png" for t in TIMES}
    last = max(shots) + 6
    log(f"[walk_shot] DEF={DEFNAME} OUT={OUT} shots@steps={sorted(shots)} offs={_offs}")
    k = 0
    while robot.step(ts) != -1:
        try:
            p = tgt.getPosition()
        except Exception:
            p = [0.0, 0.0, 1.0]
        eye = (p[0] + DX, p[1] + DY, CZ)
        target = (p[0], p[1], TZ)
        me.getField("translation").setSFVec3f([float(v) for v in eye])
        me.getField("rotation").setSFRotation(look_at(eye, target))
        if k % 60 == 0:
            log(f"[walk_shot] step {k} {DEFNAME}_x={p[0]:+.2f}")
        if k in shots:
            path = os.path.join(OUT, shots[k])
            try:
                cam.saveImage(path, 100)
                robot.step(ts)
                ok = os.path.exists(path)
                log(f"[walk_shot] saved {shots[k]} exists={ok} "
                    f"size={os.path.getsize(path) if ok else 0}")
            except Exception as e:
                log(f"[walk_shot] saveImage FAILED: {type(e).__name__}: {e}")
        if k >= last:
            break
        k += 1
    log("[walk_shot] DONE")
    robot.simulationQuit(0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
