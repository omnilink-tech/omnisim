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

"""g1_armshot -- side-view camera-bot that photographs the welded G1 while
g1_armhold sweeps the elbow angle, saving one labeled PNG per elbow value so we
can SEE which elbow gives a completely straight arm. PNGs -> _scratch/armcheck/.
"""
from __future__ import annotations
import math
import os
import sys
from omnisim import Supervisor

OUT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                   "..", "..", "..", "..", "_scratch", "armcheck"))
os.makedirs(OUT, exist_ok=True)

# Shared schedule (MUST match g1_armhold).
ELBOWS = [0.45, 0.55, 0.65]
WIN_S = 4.0
SHOOT_OFFSET = 3.4   # seconds into each window to shoot (let the pose settle)


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
        print("[g1_armshot] no cam", flush=True); return 1
    cam.enable(ts)
    me = robot.getSelf()
    eye = (3.2, -2.4, 0.95)          # 3/4 front-side view: shows fore/aft elbow bend
    target = (0.0, 0.0, 0.5)
    me.getField("translation").setSFVec3f([float(v) for v in eye])
    me.getField("rotation").setSFRotation(look_at(eye, target))

    shots = {}
    for i, ev in enumerate(ELBOWS):
        step = int(round((i * WIN_S + SHOOT_OFFSET) / (ts / 1000.0)))
        shots[step] = f"elbow_{ev:+.4f}.png"
    last = max(shots) + 4
    print(f"[g1_armshot] OUT={OUT} shots={ {s: n for s, n in shots.items()} }", flush=True)

    k = 0
    while robot.step(ts) != -1:
        if k in shots:
            path = os.path.join(OUT, shots[k])
            try:
                cam.saveImage(path, 100)
                robot.step(ts)
                ok = os.path.exists(path)
                print(f"[g1_armshot] saved {shots[k]} ok={ok} "
                      f"size={os.path.getsize(path) if ok else 0}", flush=True)
            except Exception as e:
                print(f"[g1_armshot] saveImage FAILED: {type(e).__name__}: {e}", flush=True)
        if k >= last:
            break
        k += 1
    robot.simulationQuit(0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
