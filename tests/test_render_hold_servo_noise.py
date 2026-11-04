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

"""Sub-micrometre float noise on a still body must not stop the main view holding its frames.

The defect (found 2026-11-02, when the owner asked why an idle CNC twin window loaded the GPU):
render-on-change holds a frame whose signature matches the last ones, and the signature hashed
every model matrix bit for bit. A body a servo holds away from its zero moves by float noise
every physics step; on the LongMill twin parked where the clothes-rack connector job ends,
15-58 matrices changed by up to 4e-9 m per frame, the signature changed ~20 times a second,
nothing was held, and the idle window drew an unchanging picture at 23-33 % of an RTX 3060
Laptop's 3D engine (6.5-10.6 % after the fix, the twice-a-second refresh). Matrices are now
hashed at 1e-6; hatch OMNISIM_WGPU_RENDER_ON_CHANGE_EXACT=1.

The probe nudges a Pose by +-0.2 um every step (float bits change, nothing visible moves) under
a minimised window, and OMNISIM_WGPU_REPORT_EVERY=1 writes one report line per frame actually
drawn (a held frame returns before the report). The verdict compares the hatch on the same GPU.

    python -m pytest tests/test_render_hold_servo_noise.py -v
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
EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
WorldInfo {{ basicTimeStep 16 }}
Viewpoint {{ orientation -0.2 0.2 0.96 1.6 position 0.5 -3 1.2 }}
OmniSimSky {{ }}
DEF SUN OmniSimSun {{ }}
Solid {{ translation 0 0 -0.05 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.5 0.5 0.5 roughness 0.9 metalness 0 }}
                                       geometry Box {{ size 4 4 0.1 }} }} ] }}
DEF HELD Pose {{ translation 0.3 0.2 0.25 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.8 0.3 0.2 roughness 0.6 metalness 0 }}
                                                     geometry Box {{ size 0.4 0.3 0.5 }} }} ] }}
Robot {{ supervisor TRUE controller "render_hold_probe" controllerArgs [ {json.dumps(out.as_posix())} "8" "6" ] }}
"""


def _frames_drawn(tmp_path, tag, exact):
    out = tmp_path / tag
    out.mkdir()
    # tests/rendering is the project that holds the probe; dot-prefixed worlds there are gitignored
    fd, name = tempfile.mkstemp(prefix=".render_hold_", suffix=".omniworld", dir=REPO / "tests/rendering/worlds")
    os.close(fd)
    world = Path(name)
    world.write_text(_world(out), encoding="utf-8")
    report = out / "report.txt"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(out / "engine.log"), OMNILIGHT="0",
               OMNISIM_WGPU_REPORT=str(report), OMNISIM_WGPU_REPORT_EVERY="1",
               OMNISIM_WGPU_RENDER_ON_CHANGE_EXACT="1" if exact else "0")
    for key in ("OMNISIM_NO_WINDOW", "OMNISIM_WGPU_RENDER_ON_CHANGE"):
        env.pop(key, None)
    if os.name == "nt":
        runtime = REPO / "msys64/mingw64/bin/newton-runtime"
        env["PATH"] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get("PATH", "")
    try:
        with (out / "stdout.log").open("wb") as stdout:
            proc = subprocess.Popen([str(_binary()), str(world), "--batch", "--mode=realtime", "--minimize",
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
    assert not (out / "error.txt").exists(), (out / "error.txt").read_text()
    assert code == 0 and (out / "done.json").is_file(), (out / "stdout.log").read_text(errors="replace")[-3000:]
    assert report.is_file(), "no OMNISIM_WGPU_REPORT written: the main view never drew"
    return len(report.read_text(errors="replace").splitlines())


@needs_engine
def test_servo_noise_does_not_stop_the_view_holding(tmp_path):
    fixed = _frames_drawn(tmp_path, "rounded", exact=False)
    exact = _frames_drawn(tmp_path, "exact", exact=True)
    # the hatch must draw every frame, or the probe no longer reproduces the defect
    # (measured 2026-11-02 on machine 9722d23d12a3: 120 report lines rounded, 2424 exact)
    assert exact > 5 * fixed, ("sub-micrometre jitter did not defeat the exact hash: %d vs %d" % (exact, fixed))
    assert fixed < 400, "the view drew %d frames of a scene that did not visibly change" % fixed
