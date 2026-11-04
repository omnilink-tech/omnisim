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

"""With TAA on, the main view of a scene where nothing moves must hold still.

The defect (found 2026-11-01, when the owner saw the CNC twin's machine "vibrating"): every
edge of a still scene shimmered by a fraction of a pixel, frame after frame. Three causes in
the temporal resolve, all fixed together under OMNISIM_WGPU_TAA_STILL_EDGES:
  1. the history was reprojected from the PREVIOUS frame's jitter into this frame's, so a still
     surface fetched its history up to a pixel away, in a new direction every frame;
  2. the history depth check used each pixel's own nearest depth, which the jitter flips
     between foreground and background on a silhouette, rejecting the history there;
  3. the reactive colour test compared the history with this frame's single jittered sample,
     which on any edge always differs from the converged average, and cut the history.
Measured on the idle CNC twin (1896 x 1113 main view, machine 9722d23d12a3): 10k-59k pixels
changed by more than 8 levels between consecutive frames before, 1.1k-3.2k after.

The engine runs minimised (no main view exists under OMNISIM_NO_WINDOW). The scene is
high-contrast angled boxes; the probe lets TAA settle, then exports frames one step
apart. The verdict is a ratio against the hatch-off run on the same GPU, so it does not
depend on the absolute pixel counts of any one machine.

    python -m pytest tests/test_taa_still_scene.py -v
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
FRAMES = 8


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
    # angled boxes: long silhouettes at many slopes, light on dark and dark on light (thin
    # features shimmer under any TAA and would only blur the comparison)
    boxes = "\n".join(
        f'Solid {{ translation {x:.2f} {y:.2f} {h / 2:.2f} rotation 0 0 1 {0.35 * k:.2f} children [ Shape {{ '
        f'appearance PBRAppearance {{ baseColor {c} {c} {c} roughness 0.5 metalness 0 }} '
        f'geometry Box {{ size 0.25 0.25 {h:.2f} }} }} ] }}'
        for k, (x, y, h, c) in enumerate(((-0.9, -0.4, 1.2, 0.9), (-0.6, 0.5, 0.8, 0.05), (0.7, -0.5, 1.0, 0.9),
                                          (0.9, 0.4, 1.4, 0.05), (-0.2, -0.9, 0.5, 0.9), (0.3, 1.0, 1.1, 0.9))))
    return f"""#OMNISIM R2025a utf8
WorldInfo {{ basicTimeStep 16 }}
Viewpoint {{ orientation -0.12 0.12 0.985 1.585 position 0.3 -3.2 1.3 fieldOfView 0.9 }}
Background {{ skyColor [ 0.05 0.05 0.06 ] }}
DirectionalLight {{ direction 0.4 0.6 -1 intensity 3 castShadows TRUE }}
Robot {{ supervisor TRUE controller "main_view_still_probe" controllerArgs [ {json.dumps(out.as_posix())} "12" "{FRAMES}" ] }}
Solid {{ translation 0 0 -0.05 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.55 0.45 0.3 roughness 0.9 metalness 0 }}
                                       geometry Box {{ size 4 4 0.1 }} }} ] }}
Solid {{ translation 0 0 0.25 rotation 0 0 1 0.4 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.05 0.05 0.05 roughness 0.5 metalness 0 }}
                                                         geometry Box {{ size 0.6 0.4 0.5 }} }} ] }}
Solid {{ translation 0.2 0.6 0.15 rotation 0 0 1 -0.3 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.85 0.86 0.9 roughness 0.3 metalness 0 }}
                                                              geometry Box {{ size 0.4 0.3 0.3 }} }} ] }}
{boxes}
"""


def _run(tmp_path, tag, still_edges):
    out = tmp_path / tag
    out.mkdir()
    # tests/rendering is the project that holds the probe; dot-prefixed worlds there are gitignored
    fd, name = tempfile.mkstemp(prefix=".taa_still_", suffix=".omniworld", dir=REPO / "tests/rendering/worlds")
    os.close(fd)
    world = Path(name)
    world.write_text(_world(out), encoding="utf-8")
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(out / "engine.log"), OMNILIGHT="0",
               OMNISIM_WGPU_AUTOEXP="0", OMNISIM_WGPU_TAA_STILL_EDGES="1" if still_edges else "0")
    for key in ("OMNISIM_NO_WINDOW", "OMNISIM_WGPU_TAA", "OMNISIM_WGPU_TAA_VALIDATION"):
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
    return _flicker(out)


def _flicker(out):
    """Mean count of pixels that change by more than 8 levels between consecutive frames."""
    import numpy as np
    from PIL import Image
    frames = []
    for i in range(FRAMES):
        with Image.open(out / ("%02d.png" % i)) as im:
            frames.append(np.asarray(im.convert("RGB"), dtype="int16"))
    counts = [int((np.abs(b - a).max(axis=2) > 8).sum()) for a, b in zip(frames, frames[1:])]
    return sum(counts) / len(counts), counts


@needs_engine
def test_a_still_scene_holds_still_under_taa(tmp_path):
    fixed, fixed_counts = _run(tmp_path, "still_edges", still_edges=True)
    legacy, legacy_counts = _run(tmp_path, "legacy", still_edges=False)
    # the hatch-off run must show the shimmer, or this scene no longer exercises the defect
    assert legacy > 2000, ("the scene did not shimmer even with OMNISIM_WGPU_TAA_STILL_EDGES=0", legacy_counts)
    # measured 0.32 here (2167 vs 6762 px) and 0.06 on the idle CNC twin, whose many fine edges the
    # jittered reprojection hit hardest; what remains is the reactive colour test easing the
    # history on silhouettes, kept so a coplanar colour change still leaves no trail
    assert fixed < 0.5 * legacy, (
        "a still scene shimmers under TAA: %.0f px per frame changed by >8 levels (hatch off: %.0f)"
        % (fixed, legacy), fixed_counts, legacy_counts)
