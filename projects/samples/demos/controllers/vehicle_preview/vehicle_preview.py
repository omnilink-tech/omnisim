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

"""vehicle_preview — dev supervisor: sweep the Viewpoint over the preview
vehicles and export PNGs to projects/_scratch/vehicle_shots for review."""
from __future__ import annotations
import math, os, sys
from omnisim import Supervisor

SHOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "_scratch", "vehicle_shots"))
os.makedirs(SHOT_DIR, exist_ok=True)


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
    ang = math.acos(max(-1.0, min(1.0, (trace-1.0)/2.0)))
    if ang < 1e-6:
        return [0.0, 0.0, 1.0, 0.0]
    x, y, z = m[2][1]-m[1][2], m[0][2]-m[2][0], m[1][0]-m[0][1]
    n = math.sqrt(x*x+y*y+z*z) or 1.0
    return [x/n, y/n, z/n, ang]


# vehicles placed at: dumptruck (0,-7), excavator (0,0), mixer (0,7)
SHOTS = [
    ("00_row",       (13, -1, 6),  (0, 0, 1.5)),
    ("01_dumptruck", (6, -7, 2.6), (0, -7, 1.6)),
    ("02_excavator", (7, 1, 3.0),  (1, 0, 2.2)),
    ("03_mixer",     (6, 7, 2.8),  (-0.5, 7, 1.9)),
]


def main() -> int:
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())
    root = robot.getRoot(); vp = None
    ch = root.getField("children")
    for i in range(ch.getCount()):
        n = ch.getMFNode(i)
        if n is not None and n.getTypeName() == "Viewpoint":
            vp = n; break
    if vp is None:
        print("[vehicle_preview] no Viewpoint", flush=True); return 1
    pf, of = vp.getField("position"), vp.getField("orientation")
    for _ in range(20):
        if robot.step(ts) == -1: return 0
    for name, eye, tgt in SHOTS:
        pf.setSFVec3f([float(v) for v in eye]); of.setSFRotation(look_at(eye, tgt))
        for _ in range(8):
            if robot.step(ts) == -1: return 0
        p = os.path.join(SHOT_DIR, f"{name}.png")
        robot.exportImage(p, 100); robot.step(ts)
        print(f"[vehicle_preview] wrote {p}", flush=True)
    print("[vehicle_preview] CAPTURE DONE", flush=True)
    while robot.step(ts) != -1:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
