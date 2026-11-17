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

"""A MillingStock that starts from initialHeightMap: dump it, cut into it, swap it at run time.

    controllerArgs [ "<out dir>" "<dump path>" "<a map that does not fit>" "<a second map: the pocket moved>" ]
"""
import json
import shutil
import sys
import time
from pathlib import Path

from omnisim import Supervisor

robot = Supervisor()
step = int(robot.getBasicTimeStep())
out, dump, misfit, moved = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3], sys.argv[4]
cutter = robot.getFromDef("CUTTER").getField("translation")
stock = robot.getFromDef("STOCK")
stock_start = stock.getField("initialHeightMap").getSFString()
camera = robot.getDevice("top")
camera.enable(step)


def run(n):
    for _ in range(n):
        if robot.step(step) == -1:
            raise RuntimeError("stopped")


def wait_dump(newer_than):
    end = time.monotonic() + 10.0
    while time.monotonic() < end:     # the stock dumps at most once a wall second while it removes material
        run(1)
        if dump.exists() and dump.stat().st_mtime_ns > newer_than:
            return dump.stat().st_mtime_ns
    raise RuntimeError("no height-map dump")


try:
    rep = {}
    run(2)                                            # cutter above the block: the loaded map is dumped untouched
    t = wait_dump(0)
    shutil.copy(dump, out / "after_load.zmap")
    rep["removed_after_load"] = stock.getField("removedVolume").getSFFloat()
    rep["deepest_after_load"] = stock.getField("deepestCut").getSFFloat()
    run(20)
    camera.saveImage(str(out / "loaded.png"), 100)
    for k in range(41):                               # a 2 mm slot, 2 mm deep, in fresh material
        cutter.setSFVec3f([-0.006, -0.006 + 0.0003 * k, 0.008])
        run(1)
    time.sleep(1.1)
    t = wait_dump(t)
    shutil.copy(dump, out / "after_cut.zmap")
    rep["removed_after_cut"] = stock.getField("removedVolume").getSFFloat()
    cutter.setSFVec3f([0.0, 0.0, 0.05])
    stock.getField("initialHeightMap").setSFString("")      # back to a flat block, at run time
    run(20)
    camera.saveImage(str(out / "flat.png"), 100)
    rep["removed_flat"] = stock.getField("removedVolume").getSFFloat()
    stock.getField("initialHeightMap").setSFString(misfit)  # a map for another block: refused, still flat
    run(5)
    rep["removed_misfit"] = stock.getField("removedVolume").getSFFloat()
    stock.getField("initialHeightMap").setSFString(stock_start)  # the pocket again: drawn anew, not stale
    run(20)
    camera.saveImage(str(out / "reloaded.png"), 100)
    stock.getField("initialHeightMap").setSFString(moved)  # carved to carved, the same tile counts: must still redraw
    run(20)
    camera.saveImage(str(out / "moved.png"), 100)
    (out / "done.json").write_text(json.dumps(rep))
    robot.simulationQuit(0)
except Exception as error:   # noqa: BLE001
    (out / "error.txt").write_text(repr(error))
    robot.simulationQuit(1)
