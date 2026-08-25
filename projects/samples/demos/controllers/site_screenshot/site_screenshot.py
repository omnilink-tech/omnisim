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

"""site_screenshot — dev-only supervisor that drives the Viewpoint to a
set of look-at poses and exports a screenshot of each via
Supervisor.exportImage. Used to iterate on ConstructionSiteEnvironment
visuals without the omnilink demo. Captures once, prints CAPTURE DONE,
then idles so the window stays interactive.

Output dir is the OMNISIM repo's _scratch/site_shots (overwritten each
run). Set SITE_SHOT_DIR to override.
"""

from __future__ import annotations

import math
import os
import sys

from omnisim import Supervisor

SHOT_DIR = os.environ.get(
    "SITE_SHOT_DIR",
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "_scratch", "site_shots"),
)
SHOT_DIR = os.path.abspath(SHOT_DIR)
os.makedirs(SHOT_DIR, exist_ok=True)

_LOG_PATH = os.path.join(SHOT_DIR, "_capture.log")


def log(msg: str) -> None:
    line = f"[site_screenshot] {msg}"
    print(line, flush=True)
    try:
        with open(_LOG_PATH, "a", encoding="utf-8") as fh:
            fh.write(line + "\n")
    except Exception:
        pass


def _sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def _cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def _norm(a):
    n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) or 1.0
    return (a[0] / n, a[1] / n, a[2] / n)


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Webots Viewpoint orientation (axis-angle) so the camera at `eye`
    looks toward `target`.

    This engine's camera default (identity orientation) looks along world
    +X with up +Z (verified empirically: rotating about world X rolls the
    view). We build the rotation R that maps the identity camera basis
    {F0=+X, U0=+Z, R0=F0xU0=-Y} to the target basis {F, Uo, Rt=FxUo}.
    Working it through, R has columns [F, -Rt, Uo]."""
    f = _norm(_sub(target, eye))
    if abs(f[0] * up[0] + f[1] * up[1] + f[2] * up[2]) > 0.999:
        up = (0.0, 1.0, 0.0)  # avoid degeneracy when looking along the up axis
    d = f[0] * up[0] + f[1] * up[1] + f[2] * up[2]
    uo = _norm((up[0] - d * f[0], up[1] - d * f[1], up[2] - d * f[2]))
    rt = _cross(f, uo)
    # R columns: col0 = f, col1 = -rt, col2 = uo  ->  m[row][col]
    m = [
        [f[0], -rt[0], uo[0]],
        [f[1], -rt[1], uo[1]],
        [f[2], -rt[2], uo[2]],
    ]
    trace = m[0][0] + m[1][1] + m[2][2]
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    if angle < 1e-6:
        return [0.0, 0.0, 1.0, 0.0]
    x = m[2][1] - m[1][2]
    y = m[0][2] - m[2][0]
    z = m[1][0] - m[0][1]
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-6:
        # 180-degree rotation: extract axis from diagonal.
        if m[0][0] >= m[1][1] and m[0][0] >= m[2][2]:
            ax = (m[0][0] + 1) / 2; ax = math.sqrt(max(ax, 0))
            return [ax, m[0][1] / (2 * ax) if ax > 1e-6 else 0,
                    m[0][2] / (2 * ax) if ax > 1e-6 else 0, angle]
        return [0.0, 0.0, 1.0, angle]
    return [x / n, y / n, z / n, angle]


# (name, eye, target[, up])
SHOTS = [
    ("00_overview",     (30, -26, 16),  (2, 4, 5)),
    ("01_machinery",    (26, -15, 6),   (13, -7, 1.5)),
    ("02_excavator",    (22, -7, 4),    (16, -3, 1.5)),
    ("03_mixer_tower",  (13, 0, 5),     (1, 16, 7)),
    ("04_crane",        (6, -8, 3),     (-9, 8, 15)),
    ("05_topdown",      (2, 6, 85),     (2, 6, 0),   (1, 0, 0)),
]


def main() -> int:
    log(f"start; SHOT_DIR={SHOT_DIR}")
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())

    # Find the Viewpoint node.
    root = robot.getRoot()
    vp = None
    children = root.getField("children")
    for i in range(children.getCount()):
        n = children.getMFNode(i)
        if n is not None and n.getTypeName() == "Viewpoint":
            vp = n
            break
    if vp is None:
        log("ERROR: no Viewpoint found")
        return 1
    pos_f = vp.getField("position")
    ori_f = vp.getField("orientation")
    log("viewpoint found; settling")

    # Let lighting / shadows settle.
    for _ in range(25):
        if robot.step(ts) == -1:
            return 0

    for shot in SHOTS:
        name, eye, target = shot[0], shot[1], shot[2]
        up = shot[3] if len(shot) > 3 else (0.0, 0.0, 1.0)
        pos_f.setSFVec3f([float(eye[0]), float(eye[1]), float(eye[2])])
        ori_f.setSFRotation(look_at(eye, target, up))
        for _ in range(8):
            if robot.step(ts) == -1:
                return 0
        path = os.path.join(SHOT_DIR, f"{name}.png")
        try:
            robot.exportImage(path, 100)
            robot.step(ts)  # flush queued export to disk
            ok = os.path.exists(path)
            log(f"exportImage {name} -> exists={ok} size={os.path.getsize(path) if ok else 0}")
        except Exception as e:
            log(f"exportImage {name} FAILED: {type(e).__name__}: {e}")

    log("CAPTURE DONE")

    while robot.step(ts) != -1:
        pass
    return 0


def _entry() -> int:
    import traceback
    try:
        return main()
    except Exception:
        log("FATAL:\n" + traceback.format_exc())
        return 1


if __name__ == "__main__":
    sys.exit(_entry())
