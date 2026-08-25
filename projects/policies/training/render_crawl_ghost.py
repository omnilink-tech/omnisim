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

"""Render the commando-crawl ghost (ghost_crawl_v1_lut) as an animation +
contact sheet, so the reference can be previewed before training (ghost-first).

Renders the actual G1 mesh at the crawl attitude, grounded, advancing forward
across the gait cycle, from a 3/4 camera. Outputs an animated GIF and a montage
PNG to the scratchpad. No physics stepping — pure FK posing + offscreen render.

Run:  python projects/policies/training/render_crawl_ghost.py [out_dir]
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import mujoco
from PIL import Image

REPO = Path(__file__).resolve().parents[3]
LUT = REPO / "projects/policies/ghosts/g1/ghost_crawl_v1_lut.json"
URDF = REPO / "projects/robots/unitree/g1/urdf/g1_23dof_omnisim_crawl.urdf"
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else REPO / "_scratch" / "crawl_ghost_render"
W, H = 560, 420
BASE_PITCH_DEG = 90.0
N_CYCLES = 2          # loop two strokes so forward progress reads
FRAMES_PER_CYCLE = 32


def load_model():
    p = REPO / "projects/policies/research/backends/_urdf_to_mjcf.py"
    s = importlib.util.spec_from_file_location("_u2m", p)
    m = importlib.util.module_from_spec(s); s.loader.exec_module(m)
    md = (REPO / "projects/robots/unitree/g1/urdf/meshes").as_posix()
    tmp = URDF.with_name("g1_23dof_omnisim_crawl_abs.urdf")
    tmp.write_text(URDF.read_text().replace('filename="meshes/', f'filename="{md}/'))
    lut = json.loads(LUT.read_text())
    return m.load_or_convert(tmp, actuator_joints=lut["wb_joints"]), lut


def main():
    model, lut = load_model()
    data = mujoco.MjData(model)
    wb = lut["wb_joints"]
    jadr = {j: model.joint(j).qposadr[0] for j in wb}
    q = math.radians(BASE_PITCH_DEG)
    quat = np.array([math.cos(q / 2), 0.0, math.sin(q / 2), 0.0])

    # ground on the SUPPORT limbs only (forearms + knees + hands), so the
    # robot rests on its crawl contacts and reads as a crawl (not a faceplant).
    support = {"left_elbow_link", "right_elbow_link",
               "left_knee_link", "right_knee_link",
               "left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand"}
    support_ids = {model.body(b).id for b in support}
    box_geoms = [g for g in range(model.ngeom)
                 if model.geom(g).type == mujoco.mjtGeom.mjGEOM_BOX
                 and int(model.geom_bodyid[g]) in support_ids]
    corners = np.array([[sx, sy, sz] for sx in (-1, 1)
                        for sy in (-1, 1) for sz in (-1, 1)], float)

    # brighten: the default headlight is dim
    model.vis.headlight.active = 1
    model.vis.headlight.ambient[:] = [0.55, 0.55, 0.55]
    model.vis.headlight.diffuse[:] = [0.75, 0.75, 0.75]
    model.vis.headlight.specular[:] = [0.1, 0.1, 0.1]
    nb = lut["nb"]
    stroke = lut["vx"] * lut["cycle_s"]

    ren = mujoco.Renderer(model, H, W)
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)
    # show only visual mesh groups; hide collision-box group
    opt.geomgroup[:] = 0
    opt.geomgroup[0] = 1; opt.geomgroup[1] = 1; opt.geomgroup[2] = 1

    cam = mujoco.MjvCamera()
    cam.azimuth = 90; cam.elevation = -8; cam.distance = 2.1

    frames = []
    total = N_CYCLES * FRAMES_PER_CYCLE
    for f in range(total):
        phi = (f % FRAMES_PER_CYCLE) / FRAMES_PER_CYCLE
        b = int(phi * nb) % nb
        x = f / FRAMES_PER_CYCLE * stroke
        data.qpos[:] = 0.0
        data.qpos[0:3] = [x, 0.0, 1.0]
        data.qpos[3:7] = quat
        for i, j in enumerate(wb):
            data.qpos[jadr[j]] = lut["wb_lut"][b][i]
        mujoco.mj_forward(model, data)
        zmin = 1e9
        for g in box_geoms:
            pos = data.geom_xpos[g]; R = data.geom_xmat[g].reshape(3, 3)
            s = model.geom_size[g]
            zmin = min(zmin, float((pos + (corners * s) @ R.T)[:, 2].min()))
        data.qpos[2] = 1.0 - zmin + 0.002
        mujoco.mj_forward(model, data)
        cam.lookat[:] = [x + 0.05, 0.0, 0.14]
        ren.update_scene(data, cam, opt)
        frames.append(Image.fromarray(ren.render()))

    OUT.mkdir(parents=True, exist_ok=True)
    gif = OUT / "crawl_ghost.gif"
    frames[0].save(gif, save_all=True, append_images=frames[1:],
                   duration=55, loop=0, optimize=True)
    # contact sheet: 8 evenly-spaced phases of one cycle
    step = max(1, FRAMES_PER_CYCLE // 8)
    keys = [frames[i] for i in range(0, FRAMES_PER_CYCLE, step)][:8]
    cols, rows = 4, 2
    sheet = Image.new("RGB", (cols * W, rows * H), "white")
    for i, im in enumerate(keys):
        sheet.paste(im, ((i % cols) * W, (i // cols) * H))
    png = OUT / "crawl_ghost_sheet.png"
    sheet.save(png)
    print(f"wrote {gif}\nwrote {png}  ({len(frames)} frames)")


if __name__ == "__main__":
    main()
