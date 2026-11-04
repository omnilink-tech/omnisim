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

"""The MAIN VIEW must draw a Shape under a bare root Pose, and follow it when it moves.

The defect (from 2026-08-21, found and fixed 2026-11-01 on the clothes rack assembly
world): OmWgpuSceneRenderer::collectWorldDraws walked the root's non-Solid children
only for callers that passed no draw->Solid map. The main view has passed one since
the synthetic-data dumps landed (selection tint, pick, instance ids), so every Shape
under a bare root Pose or Transform was missing from the GUI window, while Camera
devices, which pass no map, still drew it. A whole world built of supervisor-moved
Poses rendered as an empty floor. Camera tests could not see it; this one exports the
main view itself.

The engine runs minimised (the main view does not exist under OMNISIM_NO_WINDOW), the
way the harness takes its screenshots. A red box sits under a bare Pose, a green box
in a Solid is the control; a supervisor exports the view, moves the Pose across the
frame and exports again.

    python -m pytest tests/test_main_view_bare_pose.py -v
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


needs_engine = pytest.mark.skipif(_binary() is None, reason="no simulator binary in this clone; build first")


def _world(out):
    return f"""#OMNISIM R2025a utf8
WorldInfo {{ basicTimeStep 16 }}
Viewpoint {{ orientation 0 0 1 0 position -3 0 0.3 fieldOfView 0.9 }}
Background {{ skyColor [ 0.05 0.05 0.06 ] }}
DirectionalLight {{ direction 1 0.3 -0.6 intensity 3 castShadows FALSE }}
Robot {{ supervisor TRUE controller "main_view_pose_probe" controllerArgs [ {json.dumps(out.as_posix())} ] }}
DEF LOOSE Pose {{
  translation 0 -0.6 0.3
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.9 0.1 0.1 roughness 0.8 metalness 0 }}
                     geometry Box {{ size 0.3 0.3 0.3 }} }} ]
}}
DEF ANCHOR Solid {{
  translation 0.5 0 0.3
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.9 0.1 roughness 0.8 metalness 0 }}
                     geometry Box {{ size 0.3 0.3 0.3 }} }} ]
}}
"""


def _masks(path):
    import numpy as np
    from PIL import Image
    with Image.open(path) as im:
        rgb = np.asarray(im.convert("RGB"), dtype="int16")
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    # the tonemapper turns the saturated base colours pastel (green lands near 130/215/130)
    red = r - np.maximum(g, b) > 50
    green = g - np.maximum(r, b) > 50
    return red, green


@needs_engine
def test_main_view_draws_and_moves_a_bare_root_pose(tmp_path):
    import numpy as np
    out = tmp_path / "frames"
    out.mkdir()
    # tests/rendering is the project that holds the probe; dot-prefixed worlds there are gitignored
    fd, name = tempfile.mkstemp(prefix=".main_view_pose_", suffix=".omniworld", dir=REPO / "tests/rendering/worlds")
    os.close(fd)
    world = Path(name)
    world.write_text(_world(out), encoding="utf-8")
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(tmp_path / "engine.log"), OMNILIGHT="0",
               OMNISIM_WGPU_TAA="0", OMNISIM_WGPU_AUTOEXP="0")
    env.pop("OMNISIM_NO_WINDOW", None)
    if os.name == "nt":
        runtime = REPO / "msys64/mingw64/bin/newton-runtime"
        env["PATH"] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get("PATH", "")
    try:
        with (tmp_path / "stdout.log").open("wb") as stdout:
            proc = subprocess.Popen([str(_binary()), str(world), "--batch", "--mode=fast", "--minimize",
                                     "--stdout", "--stderr"], cwd=REPO, env=env, stdout=stdout,
                                    stderr=subprocess.STDOUT)
            try:
                code = proc.wait(timeout=120)
            finally:
                if proc.poll() is None:
                    if os.name == "nt":
                        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True,
                                       check=False)
                    else:
                        proc.kill()
                    proc.wait(timeout=10)
    finally:
        world.unlink(missing_ok=True)
    log = (tmp_path / "stdout.log").read_text(errors="replace")
    assert not (out / "error.txt").exists(), (out / "error.txt").read_text()
    assert code == 0 and (out / "done.json").is_file(), log[-3000:]

    red0, green0 = _masks(out / "authored.png")
    red1, green1 = _masks(out / "moved.png")
    # the control: a Shape in a Solid was always drawn
    assert green0.sum() > 500 and green1.sum() > 500, (int(green0.sum()), int(green1.sum()))
    # the bare Pose is drawn too, about as large as the control box
    assert red0.sum() > 0.5 * green0.sum(), (
        "the main view did not draw the Shape under the bare root Pose (red %d px, control %d px)"
        % (int(red0.sum()), int(green0.sum())))
    # and it follows the supervisor: +Y is the left of the frame for a viewpoint looking down +X
    x0 = np.nonzero(red0)[1].mean()
    assert red1.sum() > 0.5 * green1.sum(), int(red1.sum())
    x1 = np.nonzero(red1)[1].mean()
    assert x0 - x1 > 0.25 * red0.shape[1], (
        "the bare Pose did not move in the main view (centroid x %.0f -> %.0f px)" % (x0, x1))
