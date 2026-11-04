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

"""The showcase photographer: a supervisor carrying one Camera, which it moves through a
list of shots and photographs each one, then (optionally) ends the simulation.

    controllerArgs [ "--shots" "<shots.json>" "--out" "<dir>" ]

shots.json: {"quit": true, "shots": [ {"name": "hero", "position": [x, y, z],
    "target": [x, y, z], "at_s": 3.0, "trigger": "<file>", "hide": ["DEF", ...],
    "settle_steps": 4}, ... ]}

- `at_s`: the earliest simulation time the shot is taken (the scene's own controllers run
  meanwhile: the face is drawn, a job advances).
- `trigger`: a file whose existence is also required (a runner creates it, e.g. when a job
  has reached the progress the shot wants).
- `hide`: root DEF nodes (with a translation) moved 100 m below the floor for the shot and
  put back after it.

Each shot is written to <out>/<name>.png, and <out>/shots.json records the time and pose
every shot was taken at. The camera looks along its +x with +z up (Gram-Schmidt from world
+z), so no shot rolls.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

from omnisim import Supervisor


def look_at(position, target):
    """Axis-angle rotation whose +x points from position to target, +z as near world up."""
    f = [t - p for p, t in zip(position, target)]
    n = math.sqrt(sum(c * c for c in f)) or 1.0
    f = [c / n for c in f]
    up = [0.0, 0.0, 1.0]
    left = [up[1] * f[2] - up[2] * f[1], up[2] * f[0] - up[0] * f[2], up[0] * f[1] - up[1] * f[0]]
    ln = math.sqrt(sum(c * c for c in left))
    if ln < 1e-6:                      # looking straight down or up: pick +y as left
        left = [0.0, 1.0, 0.0]
        ln = 1.0
    left = [c / ln for c in left]
    u = [f[1] * left[2] - f[2] * left[1], f[2] * left[0] - f[0] * left[2], f[0] * left[1] - f[1] * left[0]]
    # rotation matrix columns: f (x), left (y), u (z)
    m = [[f[0], left[0], u[0]], [f[1], left[1], u[1]], [f[2], left[2], u[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    angle = math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0)))
    if angle < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    if math.pi - angle < 1e-4:         # near pi: axis from the diagonal
        x = math.sqrt(max(0.0, (m[0][0] + 1) / 2))
        y = math.copysign(math.sqrt(max(0.0, (m[1][1] + 1) / 2)), m[0][1] or 1.0)
        z = math.copysign(math.sqrt(max(0.0, (m[2][2] + 1) / 2)), m[0][2] or 1.0)
        return [x, y, z, angle]
    s = 2.0 * math.sin(angle)
    return [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, angle]


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--shots", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--camera", default="showcase_camera")
    a = p.parse_args()
    spec = json.loads(Path(a.shots).read_text(encoding="utf-8"))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    sup = Supervisor()
    dt = int(sup.getBasicTimeStep())
    cam = sup.getDevice(a.camera)      # enabled only around a shot: a big camera costs every step
    me = sup.getSelf()
    tf, rf = me.getField("translation"), me.getField("rotation")
    log = []
    for shot in spec["shots"]:
        at = float(shot.get("at_s", 0.0))
        trig = shot.get("trigger")
        wall0 = time.time()
        while sup.getTime() + 1e-9 < at or (trig and not os.path.exists(trig)):
            if sup.step(dt) == -1:
                return
            if shot.get("timeout_wall_s") and time.time() - wall0 > float(shot["timeout_wall_s"]):
                break
        hidden = []
        for d in shot.get("hide", []):          # moved far below the floor for the shot, then put back
            node = sup.getFromDef(d)
            f = node.getField("translation") if node is not None else None
            if f is not None:
                hidden.append((f, f.getSFVec3f()))
                f.setSFVec3f([0.0, 0.0, -100.0])
        tf.setSFVec3f([float(c) for c in shot["position"]])
        rf.setSFRotation(look_at(shot["position"], shot["target"]))
        if "fov" in shot:
            cam.setFov(float(shot["fov"]))
        cam.enable(dt)
        for _ in range(int(shot.get("settle_steps", 3))):
            if sup.step(dt) == -1:
                return
        path = out / f"{shot['name']}.png"
        cam.saveImage(str(path), 100)
        cam.disable()
        for f, v in hidden:
            f.setSFVec3f(v)
        log.append({"name": shot["name"], "t_s": round(sup.getTime(), 3), "position": shot["position"],
                    "target": shot["target"], "file": str(path)})
        (out / "shots.json").write_text(json.dumps(log, indent=2), encoding="utf-8")
        print(f"[showcase_camera] {shot['name']} at t={sup.getTime():.2f} s -> {path}", flush=True)
    (out / "done").write_text("ok", encoding="utf-8")
    if spec.get("quit", True):
        sup.simulationQuit(0)
        sup.step(dt)
    else:
        while sup.step(dt) != -1:
            pass


if __name__ == "__main__":
    main()
