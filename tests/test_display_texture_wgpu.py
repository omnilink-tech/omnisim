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

"""A Display draws onto its Shape under the wgpu renderer, and a redraw reaches the screen.

The defect (found 2026-11-04 on the SparkRover's face): every wgpu upload site asked an
ImageTexture for image(), which is NULL for a Display-fed texture (it has no file), so a Display
drew nothing onto its Shape: the Shape kept its flat baseColor (white). Fixing the upload was not
enough. The main view and every camera keep a cached draw list, built before the Display's first
redraw attaches its buffer, and a cached draw keeps a texture VIEW, not its pixels. The draw cache
now rebuilds once when a Shape's map becomes Display-fed, and refreshDisplayTextures() re-uploads
a redrawn Display every frame (one updateRgba8, a no-op when nothing was drawn).

A Robot carries a Display on a quad and a Camera looking at it. The probe paints the screen red,
then its left half green, and reads two pixels through the camera after each.

    python -m pytest tests/test_display_texture_wgpu.py -v
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.engine


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
    # The quad faces -x, toward the camera 0.3 m in front of it. Seen from there, the display's
    # pixel (0, 0), its top left, is at +y: the camera's left.
    return f"""#OMNISIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
WorldInfo {{ basicTimeStep 16 }}
Viewpoint {{ position -1 0 0.3 }}
OmniSimSky {{ }}
DEF SUN OmniSimSun {{ }}
Robot {{ controller "display_texture_probe" controllerArgs [ {json.dumps(out.as_posix())} ] children [
  Camera {{ name "camera" translation -0.3 0 0.3 fieldOfView 0.5 width 64 height 64 }}
  Display {{ name "display" width 32 height 32 translation 0 0 0.3 children [ Shape {{
    appearance PBRAppearance {{ baseColor 1 1 1 baseColorMap ImageTexture {{ filtering 0 }} roughness 1 metalness 0 }}
    geometry IndexedFaceSet {{ coord Coordinate {{ point [ 0 0.2 0.2, 0 0.2 -0.2, 0 -0.2 -0.2, 0 -0.2 0.2 ] }}
      texCoord TextureCoordinate {{ point [ 0 1, 0 0, 1 0, 1 1 ] }}
      coordIndex [ 0 1 2 3 -1 ] texCoordIndex [ 0 1 2 3 -1 ] }} castShadows FALSE }} ] }}
] }}
"""


def _run(tmp_path):
    out = tmp_path / "probe"
    out.mkdir()
    fd, name = tempfile.mkstemp(prefix=".display_texture_", suffix=".omniworld", dir=REPO / "tests/rendering/worlds")
    os.close(fd)
    world = Path(name)
    world.write_text(_world(out), encoding="utf-8")
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(out / "engine.log"))
    if os.name == "nt":
        runtime = REPO / "msys64/mingw64/bin/newton-runtime"
        env["PATH"] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get("PATH", "")
    try:
        with (out / "stdout.log").open("wb") as stdout:
            proc = subprocess.Popen([str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
                                     "--minimize", "--stdout", "--stderr"], cwd=REPO, env=env, stdout=stdout,
                                    stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 120
                while time.monotonic() < deadline and proc.poll() is None:
                    if (out / "done.json").exists() or (out / "error.txt").exists():
                        time.sleep(0.5)
                        break
                    time.sleep(0.2)
            finally:
                if proc.poll() is None:            # our own engine: the probe has written its result
                    proc.terminate()
                    try:
                        proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        proc.kill()
    finally:
        world.unlink(missing_ok=True)
    assert not (out / "error.txt").exists(), (out / "error.txt").read_text()
    assert (out / "done.json").is_file(), (out / "stdout.log").read_text(errors="replace")[-3000:]
    return json.loads((out / "done.json").read_text())


def _is(rgb, colour):
    # the quad is lit, not emissive, and faces away from the sun: measured 2026-11-04 red reads
    # (58, 20, 23) and green (17, 61, 23); the unfixed engine's flat white quad read (58, 61, 65)
    r, g, b = rgb
    return {"red": r > 2 * max(g, b) and r > 30, "green": g > 2 * max(r, b) and g > 30}[colour]


@needs_engine
def test_a_display_draws_onto_its_shape_and_redraws(tmp_path):
    rows = _run(tmp_path)
    # before the fix both pixels were the quad's flat white baseColor
    assert _is(rows["red"]["left"], "red") and _is(rows["red"]["right"], "red"), rows
    # the redraw reaches the cached draw, and the left half is the display's left
    assert _is(rows["half_green"]["left"], "green") and _is(rows["half_green"]["right"], "red"), rows
