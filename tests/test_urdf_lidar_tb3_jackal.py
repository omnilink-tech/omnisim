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

"""The TurtleBot3 LDS-01 and Jackal LMS1xx lidars read the walls they face.

ENGINE test (the `_binary()` skipif below marks it `engine` at collection, so
`make tests-unit` never runs it). Each case launches one engine WITH the
renderer -- a Lidar is render-based -- through
``scripts/dev/measure_urdf_lidar.py`` and reads that script's JSON report.
About 40 s per case; one engine at a time.

What is pinned, measured 2026-09-30 on the dev laptop:

* with ``OMNISIM_URDF_USE_SENSORS=1`` the bridge lists a Lidar, and on the
  near-horizontal layer every labelled wall reads within 2 cm of the distance
  computed from the lidar's own pose (measured error <= 1 mm on the
  perpendicular beams), per-beam agreement with the authored boxes is at
  least 98 %, nothing is reported below min range or above max range, and a
  forward drive shrinks the forward range by the distance the pose moved;
* WITHOUT the flag the same world has no Lidar at all (the control).

The single-layer assertion pins the 2026-09-30 ``OmUrdfImporter.cpp`` change
that always writes numberOfLayers (before it, a planar URDF <ray> became the
Lidar node's default 4-layer fan). A stale engine binary fails it.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
SCRIPT = REPO / "scripts" / "dev" / "measure_urdf_lidar.py"


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def _run(tmp_path, *args):
    out = tmp_path / "report.json"
    proc = subprocess.run([sys.executable, str(SCRIPT), *args, "--json", str(out)],
                          cwd=str(REPO), capture_output=True, text=True, timeout=400)
    if not out.exists():
        pytest.fail(f"measure_urdf_lidar.py wrote no report (rc={proc.returncode}):\n"
                    f"{proc.stdout[-3000:]}\n{proc.stderr[-2000:]}")
    report = json.loads(out.read_text(encoding="utf-8"))
    if report.get("error"):
        pytest.fail(f"run could not be made: {report['error']}")
    return report


@pytest.fixture(scope="module", params=["tb3", "jackal", "husky"])
def measured(request, tmp_path_factory):
    return request.param, _run(tmp_path_factory.mktemp(request.param), request.param)


def test_lidar_reads_the_known_walls(measured):
    scenario, report = measured
    assert report["lidar_count"] == 1, report.get("list_sensors")
    checks = dict(report["checks"])
    checks.pop("single_layer_planar")          # pinned separately below
    failed = {k: v for k, v in checks.items() if not v}
    assert not failed, (scenario, failed, report.get("drive"))


def test_lidar_is_single_layer(measured):
    _, report = measured
    assert report["before"]["layout"]["number_of_layers"] == 1


@pytest.mark.parametrize("scenario", ["tb3", "jackal", "husky"])
def test_no_lidar_without_the_flag(scenario, tmp_path):
    report = _run(tmp_path, scenario, "--control")
    assert report["lidar_count"] == 0, report.get("list_sensors")
