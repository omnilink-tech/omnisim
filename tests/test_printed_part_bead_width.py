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

"""PrintedPart never lays a bead wider than the nozzle's face, and lays what ideal_print lays.

The defect this pins (2026-11-04, the SparkRover prints): a nozzle moving less than 0.1
nozzle diameters in a 2 ms step (20 mm/s) counted as standing still. Its plastic was held,
then laid on the next faster step's 0.04-0.05 mm chord as ONE bead up to 7 mm wide, or,
past 0.05 mm^3, as a dome standing above the tip that the next step counted as a nozzle
crash (152 of plate06's 152 crashes). Now a nozzle is still below 0.01 nozzle a step, no
bead is wider than the nozzle's face (2.5 nozzles), and what a step pushes out beyond the
face is a blob where it left the nozzle (pressed under the face into a disc as tall as the
gap, welling up around the nozzle only once the disc would be wider than the face).

Probe: a supervisor drives a bodiless nozzle Solid along a scripted path with a scripted
extrudedVolume: a fast line, a slow creep into a corner (flow held up), a dwell, the next
line, and a second layer over all of it. The engine's bead dump must equal
ideal_print's laying of the same path to float32 rounding, with no line wider than the
face, the plastic conserved, and no crash. A few seconds of engine time; OMNISIM_BINARY
selects a scratch build.

    python -m pytest tests/test_printed_part_bead_width.py -v
"""

from __future__ import annotations

import json
import math
import os
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "projects/manufacturing/printing"))

import ideal_print  # noqa: E402
import inspect_print  # noqa: E402
import printer  # noqa: E402

NOZZLE = 0.0004
FACE = ideal_print.FACE_RATIO * NOZZLE
DT = 0.002


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


def path():
    """(tips m, cumulative volumes m^3) per step, step 0 = the world's initial state."""
    tips, vols = [[0.10, 0.10, 0.0002]], [0.0]
    v = 0.0
    line_area = 0.45e-3 * 0.2e-3 - 0.04e-6 + math.pi * 0.04e-6 / 4.0

    def go(to, speed, flow=None):
        nonlocal v
        a = np.array(tips[-1])
        b = np.array(to, float)
        n = max(1, int(round(np.linalg.norm(b - a) / (speed * DT))))
        for k in range(1, n + 1):
            p = a + (b - a) * k / n
            step = np.linalg.norm(b - a) / n
            v += flow * DT if flow is not None else line_area * step
            tips.append(list(p))
            vols.append(v)

    def dwell(steps, flow):
        nonlocal v
        for _ in range(steps):
            v += flow * DT
            tips.append(list(tips[-1]))
            vols.append(v)

    for z in (0.0002, 0.0004):
        tips.append([0.10, 0.10, z])
        vols.append(v)
        go([0.13, 0.10, z], 0.1)                        # 100 mm/s, a normal line
        go([0.1310, 0.10, z], 0.004, flow=1.5e-9)       # 4 mm/s at 1.5 mm^3/s: the melt outruns the nozzle
        dwell(40, 1e-9)                                  # standing: 0.08 mm^3 under the face
        go([0.1310, 0.13, z], 0.01)                      # 10 mm/s, slower than the old "still" (20 mm/s)
        dwell(150, 2e-9)                                 # 0.6 mm^3: past what the face holds
        go([0.10, 0.13, z], 0.1)
    return np.array(tips), np.array(vols)


WORLD = """#VRML_SIM R2025a utf8
WorldInfo {{ basicTimeStep 2 }}
Viewpoint {{ orientation 0 0 1 0 position -1 0 0.5 }}
DEF PRINT PrintedPart {{ translation 0 0 0 size 0.35 0.35 0.1 nozzleSolid "nozzle" nozzleDiameter {nozzle} }}
DEF NOZZLE Solid {{ name "nozzle" translation {x} {y} {z} }}
Robot {{ name "driver" supervisor TRUE controller "bead_path" }}
"""

CONTROLLER = """
import json, time
from omnisim import Supervisor

PATH = json.loads(open(r"{path}").read())
robot = Supervisor()
step = int(robot.getBasicTimeStep())
nozzle = robot.getFromDef("NOZZLE").getField("translation")
volume = robot.getFromDef("PRINT").getField("extrudedVolume")
robot.step(step)                       # step 0: the world's initial state
for tip, v in zip(PATH["tips"][1:], PATH["vols"][1:]):
    nozzle.setSFVec3f(tip)
    volume.setSFFloat(v)
    if robot.step(step) == -1:
        break
quiet = time.time()
while time.time() - quiet < 3.0 and robot.step(step) != -1:   # the dump lands when nothing changes
    pass
robot.simulationQuit(0)
"""


def _ideal(tips, vols):
    size, res, nozzle, nx, ny = ideal_print.part_frame(printer.load(printer.DEFAULT_PROFILE))
    # the probe world's part: 0.35 x 0.35 m at the default resolution, as the SV08's
    nx = int(math.ceil(0.35 / 0.0002)) + 1
    top = np.zeros(nx * nx, dtype=np.float32)
    beads, count, laid, crashes, dropped = ideal_print._lay_all(
        np.ascontiguousarray(tips), np.ascontiguousarray(vols), top, nx, nx, 0.0002, NOZZLE,
        np.zeros((1024, 9), dtype=np.float32))
    return beads[:count], laid, crashes, dropped


def test_the_ideal_rule_on_the_probe_path():
    """Engine-free: the rule itself, on the probe's path."""
    tips, vols = path()
    beads, laid, crashes, dropped = _ideal(tips, vols)
    line = beads[:, 8] == 0.0
    assert crashes == 0 and dropped == 0.0
    assert beads[line, 6].max() <= np.float32(FACE)                      # no line wider than the face
    assert (beads[line, 6] == np.float32(FACE)).any()                     # and the creep was capped
    assert laid == pytest.approx(vols[-1], rel=1e-12)                     # every bit of plastic laid
    mm = beads.astype(float) * np.r_[np.full(8, 1000.0), 1.0]
    assert inspect_print.bead_volume_mm3(mm) == pytest.approx(vols[-1] * 1e9, rel=2e-6)
    blobs = beads[~line]
    assert len(blobs) >= 4
    # the 0.08 mm^3 dwell is a disc under the face, its top at the tip; the 0.6 mm^3 one
    # wells up around the nozzle, at least face wide
    assert (np.abs(blobs[:, 7] - 0.0002) < 1e-7).any()
    big = blobs[blobs[:, 8] > 0.5e-9]
    assert len(big) == 2 and (big[:, 6] >= np.float32(FACE)).all() and (big[:, 7] > 0.0002).all()


pytestmark_engine = pytest.mark.skipif(_binary() is None, reason="no simulator binary in this clone; build first")


@pytestmark_engine
@pytest.mark.engine
def test_the_engine_lays_what_ideal_print_lays(tmp_path):
    tips, vols = path()
    (tmp_path / "path.json").write_text(json.dumps({"tips": tips.tolist(), "vols": vols.tolist()}), encoding="utf-8")
    (tmp_path / "worlds").mkdir()
    ctrl = tmp_path / "controllers" / "bead_path"
    ctrl.mkdir(parents=True)
    (ctrl / "bead_path.py").write_text(CONTROLLER.format(path=tmp_path / "path.json"), encoding="utf-8")
    world = tmp_path / "worlds" / "bead_width.omniworld"
    world.write_text(WORLD.format(nozzle=NOZZLE, x=tips[0, 0], y=tips[0, 1], z=tips[0, 2]), encoding="utf-8")
    dump, log = tmp_path / "part.beads", tmp_path / "log.txt"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log), OMNISIM_PRINTED_PART_DUMP=str(dump))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        "--stdout", "--stderr", str(world)], env=env, timeout=300, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    assert dump.is_file(), "no bead dump was written"
    data = dump.read_bytes()
    (count,) = struct.unpack_from("<i", data, 8)
    deposited = struct.unpack_from("<5d", data, 12)[3]
    got = np.frombuffer(data, dtype="<f4", count=count * 9, offset=52).reshape(count, 9)
    want, laid, crashes, _ = _ideal(tips, vols)
    assert count == len(want), f"engine laid {count} beads, ideal_print {len(want)}"
    assert np.allclose(got, want, rtol=1e-6, atol=1e-9)
    assert deposited == pytest.approx(laid, rel=1e-9)
    line = got[:, 8] == 0.0
    assert got[line, 6].max() <= np.float32(FACE)
