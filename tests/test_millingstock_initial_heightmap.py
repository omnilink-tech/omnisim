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

"""MillingStock.initialHeightMap: a block that starts carved (2026-11-16).

A film of a twin job used to run the whole job to reach the moment it shows. The stock now starts from a
height map in the layout MillingStock itself dumps (OMZMAP1/OMZMAP2, also omnisim_core.timeline_carve's
write_zmap), so omnisim_core.film carves the part to T from the job's recording and the engine starts there.

Pinned here, on one engine run:
  * the map is loaded cell for cell (the first dump IS the file) with its removed volume and deepest cut;
  * it renders (the camera sees the loaded pocket: the picture differs from the flat block's);
  * carving goes on from it: a new slot is cut, every other cell is the file's;
  * the field can be changed at run time ("" = flat again, then the map again), and the picture follows it (a
    rebuild once kept drawing its old tiles); a map made for another block is refused with a warning.

    python -m pytest tests/test_millingstock_initial_heightmap.py -v
"""

from __future__ import annotations

import json
import os
import struct
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
NX = NY = 41            # 20 mm at 0.5 mm
RES, TOP = 0.0005, 0.01


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


def write_zmap(path, h, removed, nx=NX, ny=NY, res=RES, top=TOP):
    with open(path, "wb") as f:
        f.write(b"OMZMAP1\0")
        f.write(struct.pack("<2i", nx, ny))
        f.write(struct.pack("<6d", res, top, -0.01, -0.01, 0.0, removed))
        f.write(np.ascontiguousarray(h, dtype="<f4").tobytes())


def read_zmap(path):
    b = Path(path).read_bytes()
    nx, ny = struct.unpack_from("<2i", b, 8)
    d = struct.unpack_from("<6d", b, 16)
    return np.frombuffer(b, "<f4", nx * ny, 64).reshape(ny, nx), d[5]


def _world(out, zmap, dump, misfit, moved):
    args = " ".join(json.dumps(Path(p).as_posix()) for p in (out, dump, misfit, moved))
    return f"""#OMNISIM R2025a utf8
WorldInfo {{ basicTimeStep 16 }}
Viewpoint {{ orientation -0.2 0.2 0.96 1.6 position 0.05 -0.15 0.1 }}
Background {{ skyColor [ 0.2 0.2 0.25 ] }}
DirectionalLight {{ direction -0.3 -0.4 -1 intensity 2 }}
DEF STOCK MillingStock {{ translation -0.01 -0.01 0 size 0.02 0.02 0.01 resolution 0.0005 toolSolid "cutter"
               toolShape "flat" toolDiameter 0.002 skinDepth 0.0002 coreColor 0.9 0.6 0.2 skinColor 0.1 0.15 0.4
               initialHeightMap {json.dumps(Path(zmap).as_posix())} }}
DEF CUTTER Solid {{ name "cutter" translation 0 0 0.05 }}
Robot {{ supervisor TRUE controller "milling_initial_map_probe" controllerArgs [ {args} ]
  children [ Camera {{ name "top" translation 0 0 0.06 rotation 0 1 0 1.5708 width 128 height 128 fieldOfView 0.5 }} ] }}
"""


@needs_engine
def test_initial_height_map_loads_renders_and_carves_on(tmp_path):
    h = np.full((NY, NX), TOP, np.float32)
    h[15:26, 15:26] = TOP - 0.003                     # a 5 mm pocket, 3 mm deep, in the middle
    h[30, 5] = np.float32(0.0071234)                  # one odd cell: values must survive bit for bit
    zmap, dump, misfit = tmp_path / "start.zmap", tmp_path / "dump.zmap", tmp_path / "misfit.zmap"
    write_zmap(zmap, h, 1.25e-7)
    write_zmap(misfit, np.full((11, 11), TOP, np.float32), 0.0, nx=11, ny=11)
    hm = np.full((NY, NX), TOP, np.float32)
    hm[2:10, 30:38] = TOP - 0.003                    # the same pocket, moved to a corner: the block stays carved
    moved = tmp_path / "moved.zmap"
    write_zmap(moved, hm, 1.0e-7)
    fd, name = tempfile.mkstemp(prefix=".milling_initial_map_", suffix=".omniworld", dir=REPO / "tests/rendering/worlds")
    os.close(fd)
    world = Path(name)
    world.write_text(_world(tmp_path, zmap, dump, misfit, moved), encoding="utf-8")
    log = tmp_path / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log), OMNISIM_MILLING_STOCK_DUMP=str(dump))
    env.pop("OMNISIM_NO_WINDOW", None)
    if os.name == "nt":
        runtime = REPO / "msys64/mingw64/bin/newton-runtime"
        env["PATH"] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get("PATH", "")
    try:
        with (tmp_path / "stdout.log").open("wb") as stdout:
            proc = subprocess.Popen([str(_binary()), str(world), "--batch", "--mode=fast", "--minimize",
                                     "--stdout", "--stderr"], cwd=REPO, env=env, stdout=stdout, stderr=subprocess.STDOUT)
            try:
                code = proc.wait(timeout=180)
            finally:
                if proc.poll() is None:
                    if os.name == "nt":
                        subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True, check=False)
                    else:
                        proc.kill()
                    proc.wait(timeout=10)
    finally:
        world.unlink(missing_ok=True)
    assert not (tmp_path / "error.txt").exists(), (tmp_path / "error.txt").read_text()
    assert code == 0 and (tmp_path / "done.json").is_file(), (tmp_path / "stdout.log").read_text(errors="replace")[-3000:]
    rep = json.loads((tmp_path / "done.json").read_text())

    loaded, removed = read_zmap(tmp_path / "after_load.zmap")
    assert np.array_equal(loaded.view(np.uint32), h.view(np.uint32)), "the loaded map is not the file, bit for bit"
    assert removed == 1.25e-7 and rep["removed_after_load"] == pytest.approx(1.25e-7)
    assert rep["deepest_after_load"] == pytest.approx(TOP - float(h.min()), abs=1e-7)   # the pocket, 3 mm

    cut, removed_cut = read_zmap(tmp_path / "after_cut.zmap")
    changed = cut != h
    assert changed.any() and (cut[changed] < h[changed]).all(), "the slot did not cut into the loaded block"
    js, is_ = np.nonzero(changed)
    assert is_.min() >= 4 and is_.max() <= 10 and js.min() >= 4 and js.max() <= 35, "cells outside the slot changed"
    assert np.array_equal(cut[15:26, 15:26], h[15:26, 15:26]) and cut[30, 5] == h[30, 5]
    assert removed_cut > 1.25e-7 and rep["removed_after_cut"] == pytest.approx(removed_cut)

    from PIL import Image
    a = np.asarray(Image.open(tmp_path / "loaded.png").convert("RGB"), np.float64)
    b = np.asarray(Image.open(tmp_path / "flat.png").convert("RGB"), np.float64)
    assert np.abs(a - b).mean() > 2.0, "the camera does not see the loaded pocket (or the flat block)"
    # The pocket is in the middle of the picture. Changed at run time to "" it must be DRAWN flat: until 2026-11-16 a
    # rebuild restarted every tile's revision at 1, the renderer saw no change and kept drawing the old tiles.
    c = (slice(54, 74), slice(54, 74))
    assert np.abs(a[c] - b[c]).mean() > 10.0, "the block was rebuilt flat but the pocket is still drawn (stale tiles)"
    r = np.asarray(Image.open(tmp_path / "reloaded.png").convert("RGB"), np.float64)
    assert np.abs(a - r).mean() < 0.5, "the pocket loaded again at run time is not drawn as it was at load"
    # carved -> carved: the tile's vertex and index counts do not change, so only its revision tells the renderer
    m = np.asarray(Image.open(tmp_path / "moved.png").convert("RGB"), np.float64)
    assert np.abs(r[c] - m[c]).mean() > 10.0, "the pocket was moved at run time but is still drawn where it was"
    assert rep["removed_flat"] == 0.0 and rep["removed_misfit"] == 0.0
    text = log.read_text(errors="replace")
    assert "initialHeightMap" in text and "is not used" in text, "a map for another block was not refused aloud"
