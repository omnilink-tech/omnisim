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

"""Cutting a MillingStock must not make OmniLight rebake its GI over and over.

The defect (found 2026-11-02, when the owner saw the lighting "updating all the time" while the
CNC twin milled a clothes-rack connector): OmniLight rebakes when its scene key changes, and the
key counted every draw and every index -- including the dynamic tail (MillingStock tiles, cloth,
soft bodies, granular, tracks, muscles), whose counts change as it deforms. A cutting block
rebaked GI back to back: 96 bakes in 90 s on the CNC twin, 57 s of BVH + trace on all cores but
one, and a visible lighting swap about once a second. The key now skips the dynamic tail
(hatch OMNISIM_OMNILIGHT_KEY_DYNAMIC=1 keys on it again).

The engine runs minimised (OmniLight belongs to the main view, which does not exist under
OMNISIM_NO_WINDOW). A supervisor zig-zags a cutter through a block for a fixed stretch of wall
time; the verdict compares bake counts with the hatch on the same machine.

    python -m pytest tests/test_omnilight_milling_rebake.py -v
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import tempfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
CUT_SECONDS = 25


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
Viewpoint {{ orientation -0.2 0.2 0.96 1.6 position 0.05 -0.45 0.3 }}
OmniSimSky {{ }}
DEF SUN OmniSimSun {{ }}
Solid {{ translation 0 0 -0.05 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.4 0.4 0.42 roughness 0.9 metalness 0 }}
                                       geometry Box {{ size 2 2 0.1 }} }} ] }}
MillingStock {{ translation -0.05 -0.05 0 size 0.1 0.1 0.02 resolution 0.0005 toolSolid "cutter"
               toolShape "flat" toolDiameter 0.00635 }}
DEF CUTTER Solid {{ name "cutter" translation -0.04 -0.04 0.05 children [ Pose {{ translation 0 0 0.03 children [ Shape {{
  appearance PBRAppearance {{ baseColor 0.8 0.8 0.82 metalness 0.5 roughness 0.3 }}
  geometry Cylinder {{ radius 0.003175 height 0.06 }} }} ] }} ] }}
Robot {{ supervisor TRUE controller "milling_rebake_probe" controllerArgs [ {json.dumps(out.as_posix())} "{CUT_SECONDS}" ] }}
"""


def _bakes(tmp_path, tag, key_dynamic):
    out = tmp_path / tag
    out.mkdir()
    # tests/rendering is the project that holds the probe; dot-prefixed worlds there are gitignored
    fd, name = tempfile.mkstemp(prefix=".milling_rebake_", suffix=".omniworld", dir=REPO / "tests/rendering/worlds")
    os.close(fd)
    world = Path(name)
    world.write_text(_world(out), encoding="utf-8")
    log = out / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log),
               OMNISIM_OMNILIGHT_KEY_DYNAMIC="1" if key_dynamic else "0")
    for key in ("OMNISIM_NO_WINDOW", "OMNILIGHT"):
        env.pop(key, None)
    if os.name == "nt":
        runtime = REPO / "msys64/mingw64/bin/newton-runtime"
        env["PATH"] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get("PATH", "")
    try:
        with (out / "stdout.log").open("wb") as stdout:
            proc = subprocess.Popen([str(_binary()), str(world), "--batch", "--mode=fast", "--minimize",
                                     "--stdout", "--stderr"], cwd=REPO, env=env, stdout=stdout,
                                    stderr=subprocess.STDOUT)
            try:
                code = proc.wait(timeout=180)
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
    return len(re.findall(r"\[OmniLight\] probe volume live", log.read_text(errors="replace")))


@needs_engine
def test_a_cutting_block_does_not_rebake_omnilight(tmp_path):
    fixed = _bakes(tmp_path, "fixed", key_dynamic=False)
    legacy = _bakes(tmp_path, "legacy", key_dynamic=True)
    # the hatch-on run must rebake through the cut, or the scene no longer exercises the defect
    assert legacy >= 6, ("the cut did not drive rebakes even with OMNISIM_OMNILIGHT_KEY_DYNAMIC=1", legacy)
    # the first bake (and one more if the sun or the sky settles late) is all a still light rig earns;
    # measured 2026-11-02 on machine 9722d23d12a3 over a 25 s cut: 1 bake fixed, 11 with the hatch
    assert fixed <= 2, ("OmniLight rebaked %d times while a block was cut (hatch on: %d)" % (fixed, legacy))
