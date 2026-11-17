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

"""A PrintedPart keeps recording beads past the ones it can draw.

The defect this pins (2026-11-03): PrintedPart stopped RECORDING beads at 1023 chunks
of 512 (523,776 beads), the renderer's mesh slots per part. Past that, plastic was
counted in depositedVolume but no bead was kept, so the bead dump
(OMNISIM_PRINTED_PART_DUMP) and inspect_print.py lost the rest of the part. A 13 g
plate is 212 k beads; a 145 g shell section is 1.02 M. Beads past the drawn
chunks are now recorded and dumped, only not drawn.

Probe: a supervisor zig-zags a bodiless nozzle Solid over the bed, a new bead every
step (each step turns it more than the merge angle), for more steps than the cap.
The dump must hold all of them, and the log says which were not drawn. About a minute
of engine time; OMNISIM_BINARY selects a scratch build.

    python -m pytest tests/test_printed_part_bead_cap.py -v
"""

from __future__ import annotations

import os
import struct
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
DRAWN_CAP = 512 * 1023                 # OmPrintedPart kChunk * kMaxDrawnChunks
STEPS = DRAWN_CAP + 30000


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(_binary() is None, reason="no simulator binary in this clone; build first")

WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 1 }
Viewpoint { orientation 0 0 1 0 position -1 0 0.5 }
DEF PRINT PrintedPart { translation 0 0 0 size 0.35 0.35 0.1 nozzleSolid "nozzle" nozzleDiameter 0.0004 }
DEF NOZZLE Solid { name "nozzle" translation 0.1 0.1 0.0002 }
Robot { name "spammer" supervisor TRUE controller "bead_spammer" }
"""

CONTROLLER = """
import time
from omnisim import Supervisor

STEPS = {steps}
robot = Supervisor()
step = int(robot.getBasicTimeStep())
nozzle = robot.getFromDef("NOZZLE").getField("translation")
volume = robot.getFromDef("PRINT").getField("extrudedVolume")
v = 0.0
for k in range(STEPS):
    # a 0.3 mm zig-zag: every step turns 180 degrees, so every step is a bead of its own;
    # y wraps every 1000 steps by a jump the part reads as a teleport (nothing laid)
    nozzle.setSFVec3f([0.1 + 0.0003 * (k % 2), 0.1 + 0.0001 * (k % 1000), 0.0002])
    v += 2.4e-11                      # 0.024 mm^3: a 0.3 mm line of 0.08 mm^2
    volume.setSFFloat(v)
    if robot.step(step) == -1:
        break
quiet = time.time()
while time.time() - quiet < 3.0 and robot.step(step) != -1:   # the dump lands when nothing changes
    pass
robot.simulationQuit(0)
"""


def test_beads_past_the_drawn_chunks_are_recorded_and_dumped(tmp_path):
    (tmp_path / "worlds").mkdir()
    ctrl = tmp_path / "controllers" / "bead_spammer"
    ctrl.mkdir(parents=True)
    (ctrl / "bead_spammer.py").write_text(CONTROLLER.format(steps=STEPS), encoding="utf-8")
    world = tmp_path / "worlds" / "bead_cap.omniworld"
    world.write_text(WORLD, encoding="utf-8")
    dump, log = tmp_path / "part.beads", tmp_path / "log.txt"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log), OMNISIM_PRINTED_PART_DUMP=str(dump))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        "--stdout", "--stderr", str(world)], env=env, timeout=600, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    assert dump.is_file(), "no bead dump was written"
    data = dump.read_bytes()
    assert data[:8] == b"OMBEADS1"
    (count,) = struct.unpack_from("<i", data, 8)
    assert len(data) == 52 + 36 * count, "the dump's size must match its bead count"
    # every step past the first of each 1000-step row lays a bead of its own
    expected = STEPS - STEPS // 1000 - 1
    assert count > DRAWN_CAP, f"beads stopped at {count}: the part was cut at the renderer's cap {DRAWN_CAP}"
    assert abs(count - expected) <= 0.01 * expected, f"{count} beads, expected about {expected}"
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    assert "recorded and dumped but not drawn" in text
