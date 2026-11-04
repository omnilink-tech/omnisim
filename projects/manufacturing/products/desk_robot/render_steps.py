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

"""Render the assembly guide's pictures: the robot as it stands after each step.

Starts a harness of its own (headless, on its own ports), loads one still world per
step (robot.stage_node: no physics, no controller), takes a screenshot from the same
camera each time, and stops the harness and its engine.

    python projects/manufacturing/products/desk_robot/render_steps.py
        # writes release/assembly/step_1.png ... step_7.png
"""
from __future__ import annotations

import argparse
import io
import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

from PIL import Image

import design
import robot as robot_mod

HERE = Path(__file__).resolve().parent
ROOT = design.ROOT
WORLDS = ROOT / "projects/samples/demos/worlds/manufacturing"


def _call(base: str, path: str, body=None, timeout=300, raw=False):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(base + path, data=data, headers={"Content-Type": "application/json"},
                                 method="POST" if body is not None else "GET")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        b = r.read()
    return b if raw else json.loads(b or b"{}")


def stage_world(r: design.Robot, step: int, mesh_url: str, vendor: bool = False) -> str:
    sys.path.insert(0, str(design.MANUFACTURING / "core"))
    from worldkit import look_at
    eye, target = (0.17, -0.20, 0.15), (0.0, 0.0, 0.025)
    return "\n".join([
        "#OMNISIM R2025a utf8",
        'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
        'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
        f'WorldInfo {{ basicTimeStep 8 title "desk robot, assembly step {step + 1}" }}',
        f"Viewpoint {{ position {' '.join(f'{v:g}' for v in eye)} orientation {look_at(eye, target)} fieldOfView .7 }}",
        "OmniSimSky { }", "DEF SUN OmniSimSun { }", "DEF SUN_MARKER OmniSimSunMarker { }",
        'Solid { name "bench" translation 0 0 -0.01 children [ Shape { appearance PBRAppearance { baseColor .9 .9 .88 '
        'roughness .8 metalness 0 } geometry Box { size .6 .6 .02 } } ] }',
        robot_mod.stage_node(r, mesh_url, step, prefer_vendor=vendor), ""])


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Render the desk robot's assembly steps.")
    p.add_argument("--out", default=str(HERE / "release/assembly"))
    p.add_argument("--port", type=int, default=6897)
    p.add_argument("--vendor", action="store_true",
                   help="draw the vendors' CAD (bought_parts.py fetch) instead of our drawn models: for local "
                        "pictures only, never the committed release (the vendors' models are not ours to share)")
    a = p.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    r = design.Robot()
    robot_mod.write_meshes(r)
    url = Path(os.path.relpath(robot_mod.MESHES, WORLDS)).as_posix()
    base = f"http://127.0.0.1:{a.port}"
    (ROOT / ".tmp").mkdir(exist_ok=True)
    log = (ROOT / ".tmp/desk_robot_render_steps.log").open("w", encoding="utf-8")
    harness = subprocess.Popen([sys.executable, "-m", "omnisim", "harness", "--port", str(a.port),
                                "--supervisor-port", str(a.port + 1)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    worlds = []
    try:
        for _ in range(120):
            try:
                _call(base, "/healthz", timeout=5)
                break
            except Exception:
                time.sleep(1.0)
        for step in range(len(robot_mod.STEPS)):
            w = WORLDS / f".desk_robot_step_{step + 1}.omniworld"
            w.write_text(stage_world(r, step, url, a.vendor), encoding="utf-8", newline="\n")
            worlds.append(w)
            _call(base, "/world/load", {"path": str(w.relative_to(ROOT).as_posix()), "light": True})
            _call(base, "/sim/step", {"steps": 5})
            png = _call(base, "/world/screenshot", {}, raw=True)
            # the harness writes the PNG uncompressed (6 MB); a guide needs 1200 px, compressed
            im = Image.open(io.BytesIO(png)).convert("RGB")
            im.thumbnail((1200, 1200))
            im.save(out / f"step_{step + 1}.png", optimize=True)
            print(f"step {step + 1}: {robot_mod.STEPS[step]} -> {out / f'step_{step + 1}.png'}", flush=True)
    finally:
        if os.name == "nt":
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(harness.pid)], capture_output=True)
        else:
            harness.terminate()
        log.close()
        for w in worlds:
            w.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
