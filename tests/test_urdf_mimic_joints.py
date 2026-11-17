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

"""URDF <mimic> couples the follower joint to its leader.

The defect this pins (2026-10-06): the URDF importer dropped <mimic>. Every
mimic-driven gripper (Robotiq 2F-85/2F-140, Franka Hand, Kinova Gen3 lite,
LimX, Dexmate ...) arrived as independent motors, the followers free or held
only by their own servo, so they did not follow the drive and drifted apart
under load. The follower's motor now names its leader (RotationalMotor /
LinearMotor `mimicMotor`, `mimicMultiplier`, `mimicOffset`) and the engine
enforces q_follower = offset + multiplier * q_leader as a MuJoCo joint-equality
constraint (the standard MuJoCo encoding of a mimic; MuJoCo 3.11's own URDF
import drops <mimic> entirely), registering the follower without a drive.
Commands sent to the follower's motor are ignored. On Elephant's F100 gripper
with its five mimics restored, OmniSim lifts the cube 8.70 cm with the drive
at 0.714 rad; plain MuJoCo with the same five joint equalities, 8.82 cm and
0.725 rad.

Probe: two horizontal finger arms on a fixed base, both loaded by gravity. The
controller drives the leader to 0.5 rad and ALSO commands the follower to 1.0
rad. With the coupling the follower reads 0.1 - 0.5 = -0.4 (multiplier -1,
offset 0.1); with OMNISIM_URDF_MIMIC=0 (independent motors) it obeys the 1.0.

    python -m pytest tests/test_urdf_mimic_joints.py -v

OMNISIM_BINARY selects a scratch build. Two engine launches, a few seconds each.
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
TOL = 0.01

URDF = """<?xml version="1.0"?>
<robot name="mimic_probe">
  <link name="base"><inertial><mass value="2"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size=".1 .1 .1"/></geometry></collision></link>
  <link name="finger_l"><inertial><origin xyz=".1 0 0"/><mass value=".3"/><inertia ixx=".001" iyy=".001" izz=".001" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="finger_r"><inertial><origin xyz=".1 0 0"/><mass value=".3"/><inertia ixx=".001" iyy=".001" izz=".001" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <joint name="finger_l_joint" type="revolute"><parent link="base"/><child link="finger_l"/>
    <origin xyz="0 .1 0" rpy="0 0 0"/><axis xyz="0 1 0"/><limit lower="-1.5" upper="1.5" effort="20" velocity="2"/></joint>
  <joint name="finger_r_joint" type="revolute"><parent link="base"/><child link="finger_r"/>
    <origin xyz="0 -.1 0" rpy="0 0 0"/><axis xyz="0 1 0"/><limit lower="-1.5" upper="1.5" effort="20" velocity="2"/>
    <mimic joint="finger_l_joint" multiplier="-1" offset="0.1"/></joint>
</robot>
"""

WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 4 newtonSolver "mujoco" }
Viewpoint { orientation 0 0 1 0 position -2 0 1 }
URDFRobot { url "mimic.urdf" translation 0 0 1 staticBase TRUE supervisor TRUE controller "mimic_probe" }
"""

CONTROLLER = """import os
from omnisim import Supervisor
r = Supervisor(); dt = int(r.getBasicTimeStep())
lm, fm = r.getDevice("finger_l_joint_motor"), r.getDevice("finger_r_joint_motor")
ls, fs = r.getDevice("finger_l_joint_sensor"), r.getDevice("finger_r_joint_sensor")
ls.enable(dt); fs.enable(dt)
lm.setPosition(0.5); fm.setPosition(1.0)
for _ in range(750):
    r.step(dt)
with open(os.environ["PROBE_OUT"], "w") as f:
    f.write("leader %.9g\\nfollower %.9g\\n" % (ls.getValue(), fs.getValue()))
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


def _run(tmp_path, tag, extra_env=None):
    worlds, ctrl = tmp_path / tag / "worlds", tmp_path / tag / "controllers" / "mimic_probe"
    worlds.mkdir(parents=True)
    ctrl.mkdir(parents=True)
    (worlds / "mimic.urdf").write_text(URDF, encoding="utf-8")
    (worlds / "mimic.omniworld").write_text(WORLD, encoding="utf-8")
    (ctrl / "mimic_probe.py").write_text(CONTROLLER, encoding="utf-8")
    out, log = tmp_path / tag / "out.txt", tmp_path / tag / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log), PROBE_OUT=str(out),
               **(extra_env or {}))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        str(worlds / "mimic.omniworld")], env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not out.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
        pytest.fail("the mimic probe produced no output:\n%s" % text[-1500:])
    values = dict(line.split() for line in out.read_text(encoding="utf-8").splitlines())
    return {k: float(v) for k, v in values.items()}, text


def test_follower_mirrors_its_leader(tmp_path):
    v, log = _run(tmp_path, "mimic")
    assert abs(v["leader"] - 0.5) < TOL, "the leader must reach its 0.5 rad command: %r" % v
    assert abs(v["follower"] - (0.1 - 0.5)) < TOL, (
        "the follower must sit at offset + multiplier * leader = -0.4 rad, ignoring its own 1.0 command; "
        "got %.4f" % v["follower"])
    assert "mimics joint" in log, "the engine should log the coupling it made"


def test_hatch_restores_independent_motors(tmp_path):
    v, _ = _run(tmp_path, "hatch", {"OMNISIM_URDF_MIMIC": "0"})
    assert abs(v["follower"] - 1.0) < 0.05, (
        "OMNISIM_URDF_MIMIC=0 must restore the old import (an independent motor that obeys its own "
        "1.0 rad command); got %.4f" % v["follower"])
