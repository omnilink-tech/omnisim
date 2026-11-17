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

"""A URDF prismatic joint reports its travel through its motor.

The defect this pins (2026-10-07, found on the Niryo Ned2 and Seeed reBot
gripper lanes 2026-10-04): the importer wrote a prismatic joint's URDF limits
as the joint's stops only, so LinearMotor.getMinPosition() / getMaxPosition()
read 0 / 0 while the solver enforced the range. A controller or agent reading
the motor to discover a finger's travel was told it could not move. Revolute
joints got the same fix on 2026-09-20; this extends it to prismatic ones.

Probe: one prismatic finger (-0.01 .. 0.03 m) and one revolute arm on a fixed
base. The controller reads both motors' ranges, then commands the finger past
its upper limit: it must stop at the limit (the solver limit is unchanged).

    python -m pytest tests/test_urdf_prismatic_motor_range.py -v

OMNISIM_BINARY selects a scratch build. One engine launch, a few seconds.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)

_I = '<inertia ixx=".001" iyy=".001" izz=".001" ixy="0" ixz="0" iyz="0"/>'
URDF = f"""<?xml version="1.0"?>
<robot name="range_probe">
  <link name="base"><inertial><mass value="2"/>{_I}</inertial></link>
  <link name="finger"><inertial><mass value=".1"/>{_I}</inertial></link>
  <link name="arm"><inertial><origin xyz=".1 0 0"/><mass value=".2"/>{_I}</inertial></link>
  <joint name="slide" type="prismatic"><parent link="base"/><child link="finger"/>
    <origin xyz="0 .1 0"/><axis xyz="1 0 0"/><limit lower="-0.01" upper="0.03" effort="20" velocity="0.5"/></joint>
  <joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
    <origin xyz="0 -.1 0"/><axis xyz="0 0 1"/><limit lower="-1.2" upper="0.8" effort="20" velocity="2"/></joint>
</robot>
"""

WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 4 newtonSolver "mujoco" }
Viewpoint { orientation 0 0 1 0 position -2 0 1 }
URDFRobot { url "range.urdf" translation 0 0 1 staticBase TRUE supervisor TRUE controller "range_probe" }
"""

CONTROLLER = """import os
from omnisim import Supervisor
r = Supervisor(); dt = int(r.getBasicTimeStep())
slide, hinge = r.getDevice("slide_motor"), r.getDevice("hinge_motor")
sensor = r.getDevice("slide_sensor"); sensor.enable(dt)
r.step(dt)
lines = ["slide_min %.9g" % slide.getMinPosition(), "slide_max %.9g" % slide.getMaxPosition(),
         "hinge_min %.9g" % hinge.getMinPosition(), "hinge_max %.9g" % hinge.getMaxPosition()]
slide.setPosition(0.05)
for _ in range(250):
    r.step(dt)
lines.append("slide_final %.9g" % sensor.getValue())
with open(os.environ["PROBE_OUT"], "w") as f:
    f.write("\\n".join(lines) + "\\n")
r.simulationQuit(0)
"""


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    root = tmp_path_factory.mktemp("prismatic_range")
    worlds, ctrl = root / "worlds", root / "controllers" / "range_probe"
    worlds.mkdir(parents=True)
    ctrl.mkdir(parents=True)
    (worlds / "range.urdf").write_text(URDF, encoding="utf-8")
    (worlds / "range.omniworld").write_text(WORLD, encoding="utf-8")
    (ctrl / "range_probe.py").write_text(CONTROLLER, encoding="utf-8")
    out, log = root / "out.txt", root / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log), PROBE_OUT=str(out))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        str(worlds / "range.omniworld")], env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not out.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
        pytest.fail("the range probe produced no output:\n%s" % text[-1500:])
    values = dict(line.split() for line in out.read_text(encoding="utf-8").splitlines())
    return {k: float(v) for k, v in values.items()}, text


def test_prismatic_motor_reports_its_travel(run):
    v, _ = run
    assert abs(v["slide_min"] - (-0.01)) < 1e-9 and abs(v["slide_max"] - 0.03) < 1e-9, (
        "LinearMotor.getMin/MaxPosition() must report the URDF travel -0.01..0.03; got %r..%r"
        % (v["slide_min"], v["slide_max"]))


def test_revolute_motor_still_reports_its_range(run):
    v, _ = run
    assert abs(v["hinge_min"] - (-1.2)) < 1e-9 and abs(v["hinge_max"] - 0.8) < 1e-9, v


def test_finger_still_stops_at_its_limit(run):
    v, _ = run
    assert abs(v["slide_final"] - 0.03) < 0.002, (
        "commanded past its 0.03 m limit, the finger must stop there; got %.4f" % v["slide_final"])
