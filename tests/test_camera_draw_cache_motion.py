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

"""A Camera with its cached draw list must see things move on the step they move.

The defect (found 2026-11-01 on the CNC twin, fixed the same day): every Camera and
RangeFinder keeps a cached draw list (OmAbstractCamera::collectWgpuDrawsCached, the
default since 2026-09-02) whose draws point into a vector of model matrices. The
cache refreshes those matrices in place each frame, then appends the dynamic tail
(cloth, granular, tracks, muscles, MillingStock). The MillingStock collect re-pointed
only its OWN draws after its push_backs, so when they reallocated the vector, every
cached draw kept reading the freed block: moving bodies and supervisor-moved Poses
froze in the camera image for up to 600 frames, until the age-based rebuild. With
OMNISIM_WGPU_SENSOR_DRAW_CACHE=0 every frame collected fresh and looked right.

The probe photographs three frames through a Camera in OMNISIM_NO_WINDOW mode:
at rest, after a physics body fell, and one step after a supervisor moved a Pose.
With the cache ON the frames must differ from each other, and match the frames of
the cache-OFF run step for step. The world carries a MillingStock (256 tiles, so
its tail always outgrows the vector) or not, so both shapes of the list are pinned.

    python -m pytest tests/test_camera_draw_cache_motion.py -v
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

# 0.1 m at 0.1 mm is 1000 x 1000 cells = 16 x 16 tiles of 64: a 257-draw tail
STOCK = ('MillingStock { translation 0.6 -0.05 0.25 size 0.1 0.1 0.02 resolution 0.0001 '
         'skinColor 0.2 0.5 0.9 }')


def _world(args, with_stock):
    return f"""#VRML_SIM R2025a utf8
WorldInfo {{ basicTimeStep 16 }}
Viewpoint {{ orientation 0 0 1 0 position -4 0 1 }}
Background {{ skyColor [ 0.05 0.05 0.06 ] }}
DirectionalLight {{ direction 1 0.6 -0.8 intensity 2 castShadows FALSE }}
Robot {{
  translation -3 0 0.9
  supervisor TRUE
  controller "camera_motion_probe"
  controllerArgs [ {args} ]
  children [ Camera {{ name "camera" width 128 height 96 fieldOfView 0.9 far 20 }} ]
}}
DEF BLOCK Pose {{
  translation 0 -0.45 0.8
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.9 0.2 0.1 roughness 0.8 metalness 0 }}
                     geometry Box {{ size 0.3 0.3 0.3 }} }} ]
}}
DEF BALL Solid {{
  translation 0 0.4 1.3
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.2 0.9 0.2 roughness 0.8 metalness 0 }}
                     geometry DEF BALL_G Sphere {{ radius 0.15 }} }} ]
  boundingObject USE BALL_G
  physics Physics {{ }}
}}
{STOCK if with_stock else ''}
"""


def _ppm(path):
    data = path.read_bytes()
    parts = data.split(b"\n", 1)  # header "P6 w h 255", then the pixels
    return parts[1]


def _run(tmp_path, tag, with_stock, cache_on):
    d = tmp_path / tag
    d.mkdir()
    result = d / "result.json"
    frames = d / "frames"
    args = json.dumps(result.as_posix()) + " " + json.dumps(frames.as_posix())
    # tests/rendering is the project that holds the controller; dot-prefixed worlds there are gitignored
    fd, name = tempfile.mkstemp(prefix=".camera_motion_", suffix=".omniworld", dir=REPO / "tests/rendering/worlds")
    os.close(fd)
    placed = Path(name)
    placed.write_text(_world(args, with_stock), encoding="utf-8")
    env = dict(os.environ)
    env.update(OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(d / "engine.log"), OMNISIM_NO_WINDOW="1",
               OMNISIM_WGPU_TAA="0", OMNISIM_WGPU_SENSOR_DRAW_CACHE="1" if cache_on else "0")
    if os.name == "nt":
        runtime = REPO / "msys64/mingw64/bin/newton-runtime"
        env["PATH"] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get("PATH", "")
    try:
        proc = subprocess.Popen([str(_binary()), str(placed), "--batch", "--mode=fast", "--stdout", "--stderr"],
                                cwd=REPO, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            stdout, _ = proc.communicate(timeout=120)
        except subprocess.TimeoutExpired:
            if os.name == "nt":
                subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True, check=False)
            else:
                proc.kill()
            stdout, _ = proc.communicate(timeout=10)
            pytest.fail("camera motion probe timed out: " + stdout.decode(errors="replace")[-3000:])
    finally:
        placed.unlink(missing_ok=True)
    assert result.is_file(), stdout.decode(errors="replace")[-3000:]
    data = json.loads(result.read_text(encoding="utf-8"))
    assert "error" not in data, data
    return data, [_ppm(frames / (f["name"] + ".ppm")) for f in data["frames"]]


def _changed(a, b, threshold=8):
    return sum(1 for x, y in zip(a, b) if abs(x - y) > threshold)


@needs_engine
@pytest.mark.parametrize("with_stock", [True, False], ids=["with_milling_stock", "no_dynamic_tail"])
def test_cached_camera_sees_motion_on_the_step_it_happens(tmp_path, with_stock):
    tag = "stock" if with_stock else "plain"
    on, on_frames = _run(tmp_path, tag + "_cache_on", with_stock, cache_on=True)
    off, off_frames = _run(tmp_path, tag + "_cache_off", with_stock, cache_on=False)
    # the scene really moved: the ball fell under physics, then the supervisor moved the block
    balls = [f["ball"][2] for f in on["frames"]]
    assert balls[0] - balls[1] > 0.05, on
    assert on["frames"][2]["block"][1] - on["frames"][1]["block"][1] > 0.3, on
    # and the cache-OFF run photographed it (the reference the cache must reproduce)
    assert _changed(off_frames[0], off_frames[1]) > 100 and _changed(off_frames[1], off_frames[2]) > 100, off
    # the cached camera saw both moves...
    fell, moved = _changed(on_frames[0], on_frames[1]), _changed(on_frames[1], on_frames[2])
    assert fell > 100 and moved > 100, (
        "the cached-draw-list camera did not see the scene move (changed channels: fall %d, Pose move %d)"
        % (fell, moved))
    # ...on the same step as a fresh collect: every frame matches the cache-OFF frame
    for k, (a, b) in enumerate(zip(on_frames, off_frames)):
        worst = max(abs(x - y) for x, y in zip(a, b))
        assert worst <= 2, "frame %d differs from the cache-OFF frame by up to %d" % (k, worst)
