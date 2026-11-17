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

"""Joint FRAMES reach Newton intact: a merged parent's axis, a slider child's rotation.

Two defects an external user (Galbot R1, dual arm) found on v9.0.1 and fixed
locally, both in OmBasicJoint::flushPendingNewtonRegistrations; reproduced on
our main 2026-09-27 before the fix, to the digit of his report:

1. MERGED-PARENT AXIS. A URDF fixed joint merges its child link into the
   nearest ancestor body (the "leader"). The joint ANCHOR was re-expressed in
   the leader's frame, the AXIS was not, so a moving joint below a ROTATED fixed
   frame turned about the wrong axis. q = 0 looks right; motion exposes it.
   Probe: moving joint -> empty fixed frame rotated 90 deg about X -> fixed
   bracket -> moving joint, both driven to +0.25 rad. Measured orientation error
   of the last link: 0.3530916 rad before, 3.7e-6 rad after (float32 level).

2. SLIDER CHILD ROTATION. The revolute path carried the child's authored
   rotation relative to its parent into newton's child_xform; the prismatic path
   passed none, so a slider child authored at 90 deg about X was snapped to its
   parent's orientation -- while the slider readback still said the right
   travel. Measured: 1.5707963 rad error before, 3.4e-8 rad after, slider 0.1 m
   both times.

Each probe is launched once; the assertions carry a wide margin (1e-3 rad
against defects of 0.35 and 1.57 rad) so float32 noise can never flake them.

    python -m pytest tests/test_newton_joint_frames.py -v

OMNISIM_BINARY selects a scratch build, as it does for run-headless.
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

TOL_RAD = 1e-3

# Pure-Python rotation error so the probe controller needs nothing beyond the
# omnisim package (the bundled controller interpreter ships numpy, not scipy).
_ROT_HELPERS = '''import math
def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]
def as3(flat):
    return [list(flat[0:3]), list(flat[3:6]), list(flat[6:9])]
def angle(a, b):
    m = matmul(transpose(a), b)
    c = max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1.0) / 2.0))
    return math.acos(c)
def rx(t):
    c, s = math.cos(t), math.sin(t)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]
def rz(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]
'''

SLIDER_WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 8 gravity 0 newtonSolver "mujoco" newtonStatics TRUE }
Viewpoint { orientation 0 0 1 0 position -2 0 1 }
DEF R Robot {
  supervisor TRUE
  controller "slider_frame"
  boundingObject Box { size 0.1 0.1 0.1 }
  children [
    SliderJoint {
      jointParameters JointParameters { axis 0 0 1 minStop 0 maxStop 0.2 }
      device [ LinearMotor { name "slide" minPosition 0 maxPosition 0.2 maxForce 100 } PositionSensor { name "sensor" } ]
      endPoint DEF CHILD Solid {
        translation 0 0 0.3
        rotation 1 0 0 1.5707963267948966
        children [ Shape { geometry Box { size 0.1 0.2 0.05 } } ]
        boundingObject Box { size 0.1 0.2 0.05 }
        physics Physics { mass 1 density -1 }
      }
    }
  ]
}
"""

SLIDER_CONTROLLER = _ROT_HELPERS + '''
import os
from omnisim import Supervisor
r = Supervisor()
m = r.getDevice("slide"); s = m.getPositionSensor(); s.enable(8)
child = r.getFromDef("CHILD")
# Compare against the AUTHORED orientation (the parent Robot is unrotated, so the
# child must stay at exactly Rx(90 deg)), never against a pose read after a step:
# the defect snaps the child at registration, so a post-step reading already
# carries it and the error would read 0.
initial = rx(math.pi / 2)
m.setPosition(0.1)
for _ in range(250):
    r.step(8)
with open(os.environ["PROBE_OUT"], "w") as f:
    f.write("slider_rotation_error %.12g\\n" % angle(initial, as3(child.getOrientation())))
    f.write("slider_q %.12g\\n" % s.getValue())
    f.write("done 1\\n")
r.simulationQuit(0)
'''

CHAIN_URDF = """<robot name="frame_probe">
  <link name="root"><inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size=".04 .04 .04"/></geometry></collision></link>
  <link name="mid"><inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size=".04 .04 .04"/></geometry></collision></link>
  <link name="attach"/>
  <link name="bracket"><inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size=".04 .04 .04"/></geometry></collision></link>
  <link name="arm"><inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><box size=".04 .04 .04"/></geometry></collision></link>
  <joint name="mid_joint" type="revolute"><parent link="root"/><child link="mid"/>
    <origin xyz="0 0 .15" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="100" velocity="1"/></joint>
  <joint name="attach_joint" type="fixed"><parent link="mid"/><child link="attach"/>
    <origin xyz="0 0 0" rpy="1.5707963267948966 0 0"/></joint>
  <joint name="bracket_joint" type="fixed"><parent link="attach"/><child link="bracket"/>
    <origin xyz="0 0 .15" rpy="0 0 0"/></joint>
  <joint name="arm_joint" type="revolute"><parent link="bracket"/><child link="arm"/>
    <origin xyz="0 0 .15" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="100" velocity="1"/></joint>
</robot>
"""

CHAIN_WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 8 gravity 0 newtonSolver "mujoco" newtonStatics TRUE }
Viewpoint { orientation 0 0 1 0 position -2 0 1 }
DEF R1 URDFRobot { url "rotated_chain.urdf" staticBase TRUE supervisor TRUE controller "chain_frame" }
"""

CHAIN_CONTROLLER = _ROT_HELPERS + '''
import os
from omnisim import Supervisor
r = Supervisor()
dt = int(r.getBasicTimeStep())

def find(node, name):
    f = node.getField("name")
    if f is not None and f.getSFString() == name:
        return node
    for fld in ("children", "endPoint", "device"):
        c = node.getField(fld)
        if c is None:
            continue
        if c.getTypeName() == "SFNode":
            n = c.getSFNode()
            items = [n] if n is not None else []
        elif c.getTypeName() == "MFNode":
            items = [c.getMFNode(i) for i in range(c.getCount())]
        else:
            items = []
        for n in items:
            got = find(n, name)
            if got is not None:
                return got
    return None

arm = find(r.getSelf(), "arm")
motors = [r.getDevice(n + "_motor") for n in ("mid_joint", "arm_joint")]
sensors = [m.getPositionSensor() for m in motors]
for s in sensors:
    s.enable(dt)
for m in motors:
    m.setVelocity(1.0); m.setPosition(0.25)
for _ in range(250):
    r.step(dt)
qm, qa = sensors[0].getValue(), sensors[1].getValue()
expected = matmul(matmul(rz(qm), rx(math.pi / 2)), rz(qa))
with open(os.environ["PROBE_OUT"], "w") as f:
    f.write("chain_rotation_error %.12g\\n" % angle(expected, as3(arm.getOrientation())))
    f.write("q_mid %.12g\\nq_arm %.12g\\n" % (qm, qa))
    f.write("done 1\\n")
r.simulationQuit(0)
'''


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


def _run(tmp_path, world_name, world, ctrl_name, ctrl_src, extra=None):
    worlds = tmp_path / "worlds"
    ctrl = tmp_path / "controllers" / ctrl_name
    worlds.mkdir(parents=True, exist_ok=True)
    ctrl.mkdir(parents=True, exist_ok=True)
    (worlds / world_name).write_text(world, encoding="utf-8")
    for name, text in (extra or {}).items():
        (worlds / name).write_text(text, encoding="utf-8")
    (ctrl / (ctrl_name + ".py")).write_text(ctrl_src, encoding="utf-8")
    result = tmp_path / "probe_out.txt"
    log = tmp_path / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(result),
               OMNISIM_LOG_PATH=str(log))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(worlds / world_name)],
                       env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not result.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
        pytest.fail("the frame probe produced no output:\n%s" % text[-1500:])
    values = {}
    for line in result.read_text(encoding="utf-8").splitlines():
        k, v = line.split()
        values[k] = float(v)
    values["_log"] = text
    return values


def test_slider_child_keeps_its_authored_rotation(tmp_path):
    v = _run(tmp_path, "slider_frame.omniworld", SLIDER_WORLD, "slider_frame", SLIDER_CONTROLLER)
    assert abs(v["slider_q"] - 0.1) < 1e-3, "the slider must still travel its 0.1 m: %r" % v["slider_q"]
    assert v["slider_rotation_error"] < TOL_RAD, (
        "a slider child authored at 90 deg about X must keep that orientation; %.6f rad "
        "of error (1.5708 = snapped to the parent) means the child rotation no longer "
        "reaches newton's child_xform" % v["slider_rotation_error"])


def test_joint_below_rotated_fixed_frame_turns_about_its_own_axis(tmp_path):
    v = _run(tmp_path, "rotated_chain.omniworld", CHAIN_WORLD, "chain_frame", CHAIN_CONTROLLER,
             extra={"rotated_chain.urdf": CHAIN_URDF})
    assert abs(v["q_mid"] - 0.25) < 1e-3 and abs(v["q_arm"] - 0.25) < 1e-3, v
    assert v["chain_rotation_error"] < TOL_RAD, (
        "the arm link must equal Rz(q_mid) Rx(90 deg) Rz(q_arm); %.6f rad of error "
        "(0.353 before the fix) means the axis was passed in the merged-away parent's "
        "frame instead of the leader body's" % v["chain_rotation_error"])
