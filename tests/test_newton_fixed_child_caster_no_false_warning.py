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

"""A massless fixed caster carries the robot, and the engine must not say otherwise.

A URDF caster declared as a fixed-joint link with a <collision> and no
<inertial> imports as a Physics-less Solid under its parent's body. Since
2026-10-03 (b97b4d8c0) its boundingObject is attached to that body and COLLIDES
(OMNISIM_NEWTON_FIXED_CHILD_COLLIDERS). But the inherited Webots warning
"As 'physics' is set to NULL, collisions will have no effect." still fired for
it, and on 2026-10-04 an outreach lane read the Ubiquity Magni's two copies of
that warning as "the robot has no caster support at all". Measured 2026-10-07:
the Magni stands level on its casters, and tips back (root z 0.13 m) only with
the fixed-child path switched off. The warning now fires only then.

Probe: a chassis whose COM sits behind its wheel axle, two wheels, and one
massless caster sphere at the back. Level on the caster = the root's peak |z|
stays ~0; no caster = the chassis has nothing behind the axle and rocks on its
wheels like a pendulum (peak 0.19 m measured).

    python -m pytest tests/test_newton_fixed_child_caster_no_false_warning.py -v

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
WARNING = "collisions will have no effect"


def _inertial(m, i=0.01, xyz="0 0 0"):
    return (f'<inertial><origin xyz="{xyz}"/><mass value="{m}"/><inertia ixx="{i}" iyy="{i}" izz="{i}" '
            f'ixy="0" ixz="0" iyz="0"/></inertial>')


URDF = f"""<?xml version="1.0"?>
<robot name="probe">
  <link name="base">{_inertial(10, 0.1, "-0.15 0 0.1")}</link>
  <link name="left_wheel">{_inertial(1)}<collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <link name="right_wheel">{_inertial(1)}<collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <link name="caster"><collision><geometry><sphere radius="0.04"/></geometry></collision></link>
  <joint name="lw" type="continuous"><parent link="base"/><child link="left_wheel"/>
    <origin xyz="0 0.2 0.1"/><axis xyz="0 1 0"/><limit effort="5" velocity="5"/></joint>
  <joint name="rw" type="continuous"><parent link="base"/><child link="right_wheel"/>
    <origin xyz="0 -0.2 0.1"/><axis xyz="0 1 0"/><limit effort="5" velocity="5"/></joint>
  <joint name="c" type="fixed"><parent link="base"/><child link="caster"/><origin xyz="-0.3 0 0.04"/></joint>
</robot>
"""

WORLD = """#VRML_SIM R2025a utf8
WorldInfo { basicTimeStep 4 newtonSolver "mujoco" }
Viewpoint { orientation 0 0 1 0 position -3 0 1 }
Solid { translation 0 0 -0.5 boundingObject Box { size 10 10 1 } }
URDFRobot { url "probe.urdf" translation 0 0 0.002 name "probe" controller "<none>" }
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
    d = tmp_path / tag
    d.mkdir()
    (d / "probe.urdf").write_text(URDF, encoding="utf-8")
    (d / "probe.omniworld").write_text(WORLD, encoding="utf-8")
    traj, log = d / "traj.tsv", d / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_MS="2000", **(extra_env or {}))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        str(d / "probe.omniworld")], env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    for sig in _BRINGUP_SIGNATURES:
        if sig in text:
            pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
    rows = [r.split("\t") for r in traj.read_text(encoding="utf-8").splitlines()] if traj.is_file() else []
    rows = [r for r in rows if len(r) == 6 and r[1] == "probe"]
    assert rows, "the probe recorded no trajectory:\n" + text[-1500:]
    # Peak |z| of the root over the run: a robot standing on its caster stays at ~0;
    # without the caster it rocks on its wheels like a pendulum (0 .. 0.2 m), so a
    # single final sample could land near 0 by chance.
    return max(abs(float(r[4])) for r in rows if float(r[0]) >= 200.0), text


def test_massless_fixed_caster_supports_and_is_not_called_inert(tmp_path):
    z, log = _run(tmp_path, "default")
    assert z < 0.01, "the robot must stand level on its caster (root |z| stays ~0); peak %.4f" % z
    assert WARNING not in log, ("the engine still claims a fixed child's collider has no effect, while it "
                                "is carrying the robot")


def test_warning_and_tip_return_with_the_path_off(tmp_path):
    z, log = _run(tmp_path, "off", {"OMNISIM_NEWTON_FIXED_CHILD_COLLIDERS": "0"})
    assert WARNING in log, "with the fixed-child path off the warning is true and must be shown"
    assert z > 0.05, "with no caster collider the robot must tip back and rock; peak root |z| %.4f" % z
