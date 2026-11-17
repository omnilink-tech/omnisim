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

"""The in-process core's stepping rules and its strict controller API, on the engine's own MuJoCo build.

Runs in a child interpreter: the core puts the engine's runtime bundle first on the import path, and a
test session that has already imported its own numpy must not have a second one slid underneath it.
That child process is why this module sits in the engine lane, though it starts no simulator.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "packages" / "omnisim-core" / "src"

CHECK = r'''
import math
from omnisim_core.runtime import ensure_runtime
ok, why = ensure_runtime()
if not ok:
    print("SKIP:", why)
    raise SystemExit(0)
import numpy as np, mujoco
from omnisim_core import core as C, names as nm
from omnisim_core.errors import CoreUnsupported

XML = """<mujoco><option timestep="0.002"/><worldbody>
 <body name="ball" pos="0 0 1"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body>
 <body name="car" pos="1 0 0.2"><joint name="s" type="slide" axis="1 0 0" range="-0.5 0.5" limited="true"/>
  <geom type="box" size="0.05 0.05 0.05" mass="1"/></body>
</worldbody><actuator><position joint="s" kp="100"/><velocity joint="s" kv="10"/></actuator></mujoco>"""
m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)
s0 = np.zeros(mujoco.mj_stateSize(m, C.P_SPEC))
mujoco.mj_getState(m, d, s0, C.P_SPEC)
jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "s")
qa, da = int(m.jnt_qposadr[jid]), int(m.jnt_dofadr[jid])

def refuses(fn):
    try:
        fn()
    except CoreUnsupported:
        return True
    return False

# 1. with no clamp, one substep and no pose write, a core tick IS mj_step: same bits
core = C.Core(m, s0)
ref = mujoco.MjData(m)
mujoco.mj_setState(m, ref, s0, C.P_SPEC)
core.ctrl[0] = ref.ctrl[0] = 0.3
for _ in range(200):
    core.step()
    mujoco.mj_step(m, ref)
assert np.array_equal(core.d.qpos, ref.qpos) and np.array_equal(core.d.qvel, ref.qvel), "a plain tick is not mj_step"
assert core.ticks == 200 and core.clamp_ticks == 0

# 2. the joint clamp: the speed limit holds, and a tick that clamps leaves the state in float32
core = C.Core(m, s0)
core.set_clamp(np.array([qa]), np.array([da]), np.array([0.01]), np.array([-0.5]), np.array([0.5]), np.array([0.005]))
core.ctrl[0] = 0.3
for _ in range(20):
    before = core.clamp_ticks
    core.step()
    assert abs(core.d.qvel[da]) <= float(np.float32(0.01)) + 1e-12
    if core.clamp_ticks > before:
        assert np.array_equal(core.d.qpos, core.d.qpos.astype(np.float32).astype(np.float64))
assert core.clamp_ticks > 0, "the clamp never ran"

# 3. the controller API: served calls answer as the engine does, everything else is refused
def robot(strict):
    cfg = dict(core=C.Core(m, s0), dt_ms=2.0, max_ticks=10 ** 9, strict=strict, notes={},
               motors=[{"name": "s_motor", "sensor": "s_sensor", "jid": jid, "vmax": 1.0, "fmax": 10.0, "servo": True}],
               bodies={1: "ball", 2: "car"}, robot_bodies=[2], robot_def="CAR", robot_name="car", custom_data="", defs={"BALL": 1},
               objects={}, all_defs={"GLOW": "Pose", "LOOSE": "Solid"}, defs_complete=True, physical_types=nm.PHYSICAL_TYPES,
               other_devices=["eye"], devices_complete=True)
    C.Supervisor._next_cfg = cfg
    return C.Supervisor()

r = robot(True)
motor, sensor = r.getDevice("s_motor"), r.getDevice("s_sensor")
assert r.getDevice("no_such_device") is None                       # the world declares none: the engine says None too
assert refuses(lambda: r.getDevice("eye"))                         # declared, not modelled
assert refuses(lambda: r.getLidar)                                 # an API the core does not serve
assert refuses(lambda: motor.setVelocity(0.5))                     # a speed cap on a position servo is not modelled
assert refuses(lambda: motor.setControlPID(1, 0, 0))
assert r.getFromDef("NO_SUCH_DEF") is None
assert refuses(lambda: r.getFromDef("LOOSE"))                      # a Solid the core has no handle for
glow = r.getFromDef("CAR.HEAD.GLOW")                               # a drawn node: writes are kept, nothing else happens
glow.getField("translation").setSFVec3f([0, 0, 1])
assert glow.getField("translation").getSFVec3f() == [0, 0, 1]
assert refuses(lambda: glow.getField("rotation").getSFRotation())  # its authored value is not known here
assert refuses(lambda: glow.getPosition)

motor.setPosition(0.25)
assert r.core.ctrl[motor.a_pos] == float(np.float32(0.25))         # targets travel as float32
assert math.isnan(sensor.getValue())                               # not enabled, no tick yet
sensor.enable(2)
assert r.step(2) == 0 and r.core.ticks == 1
assert sensor.getValue() == float(np.float32(r.core.d.qpos[qa]))

# 4. a supervisor pose write lands one tick late, stops the body, and passes the state through float32
ball = r.getFromDef("BALL")
tr = ball.getField("translation")
z_before = float(r.core.d.qpos[2])
tr.setSFVec3f([0.1, 0.2, 5.0])
assert float(r.core.d.qpos[2]) == z_before, "the write must not land before the next step"
r.step(2)
assert list(r.core.d.qpos[0:3]) == [float(np.float32(v)) for v in (0.1, 0.2, 5.0)]
assert not r.core.d.qvel[0:6].any()
assert tr.getSFVec3f() == [float(np.float32(v)) for v in (0.1, 0.2, 5.0)]

# 5. lenient mode counts instead of refusing
r2 = robot(False)
assert r2.getLidar("front") is None and r2.notes
print("OK")
'''


def test_core_stepping_rules_and_strict_api():
    env = dict(os.environ)
    env["PYTHONPATH"] = str(SRC) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    r = subprocess.run([sys.executable, "-c", CHECK], cwd=ROOT, env=env, capture_output=True, text=True, timeout=300)
    out = (r.stdout or "") + (r.stderr or "")
    if "SKIP:" in r.stdout:
        pytest.skip("the engine's physics runtime bundle is not staged here: " + r.stdout.strip()[-200:])
    assert r.returncode == 0 and r.stdout.strip().endswith("OK"), out[-3000:]
