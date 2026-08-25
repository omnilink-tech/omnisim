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

"""g1_shot — camera-bot that photographs the standing G1.

Runs alongside the g1_stand_deploy controller in g1_stand_shot.omniworld. Aims a
Camera at the robot, lets it settle into the stand, and saves PNGs at a few
timepoints via Camera.saveImage (the only reliable headless still path on this
machine — exportImage gives grey; see reference_headless_render). Output ->
_scratch/g1_shots/. Must run with rendering ON (no --batch / --no-window).
"""
from __future__ import annotations
import math
import os
import sys

from omnisim import Supervisor

OUT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                   "..", "..", "..", "..", "_scratch", "g1_shots"))
os.makedirs(OUT, exist_ok=True)
_LOG = open(os.path.join(OUT, "_shot.log"), "w", buffering=1)


def log(m):
    _LOG.write(m + "\n"); _LOG.flush()
    print(m, flush=True)


def _sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def _cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def _norm(a):
    n = math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]) or 1.0
    return (a[0]/n, a[1]/n, a[2]/n)


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Axis-angle so a +X-forward / +Z-up camera at eye looks at target
    (same convention as site_screenshot.look_at)."""
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
        log("[g1_shot] no cam device"); return 1
    cam.enable(ts)
    log(f"[g1_shot] start ts={ts}")

    me = robot.getSelf()
    eye = (-3.0, -2.3, 1.25)
    target = (0.0, 0.0, 0.55)
    me.getField("translation").setSFVec3f([float(v) for v in eye])
    me.getField("rotation").setSFRotation(look_at(eye, target))

    # Save at a few sim times to show the stand is steady, not a lucky frame.
    shots = {int(round(t / (ts / 1000.0))): f"g1_stand_t{int(t)}s.png"
             for t in (2, 5, 10)}
    last = max(shots) + 5
    log(f"[g1_shot] OUT={OUT} shots at steps {sorted(shots)}")
    k = 0
    while robot.step(ts) != -1:
        if k % 30 == 0:
            log(f"[g1_shot] step {k}")
        if k in shots:
            path = os.path.join(OUT, shots[k])
            try:
                cam.saveImage(path, 100)
                robot.step(ts)  # flush
                ok = os.path.exists(path)
                log(f"[g1_shot] saved {shots[k]} exists={ok} "
                    f"size={os.path.getsize(path) if ok else 0}")
            except Exception as e:
                log(f"[g1_shot] saveImage FAILED: {type(e).__name__}: {e}")
        if k >= last:
            break
        k += 1
    log("[g1_shot] DONE")
    robot.simulationQuit(0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
