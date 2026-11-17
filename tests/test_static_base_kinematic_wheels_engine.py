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

"""A staticBase wheeled robot must STAY where it was placed when its wheels turn.

The defect this pins (found 2026-10-20 while checking a lead from a NORI A3
run): `URDFRobot { staticBase TRUE }` strips only the root link's Physics, so
the Robot node has no Physics while its wheel Solids keep theirs. Webots'
kinematic differential-wheels mode (OmKinematicDifferentialWheels, inherited
from upstream) attaches to ANY Robot without Physics that has two equal,
equidistant cylinder wheels on parallel axes, and moved the whole robot by
omega * r every step. Measured on v9.2.6 and on the dev tree, wheels at
4 rad/s for 2 s:

  robot                         base displacement   z
  NORI A3, staticBase TRUE      0.6096 m (= 4 * 0.0762 * 2)   frozen
  NORI A3, floating             0.365 m (real rolling)        free
  ROS-style URDF, staticBase    0.0000 m  (its rpy-rotated wheel axes are
                                not EXACTLY parallel, so the mode never armed)

while the load log said the base's fixed links were "attached to the static
environment". The lead itself -- a FLOATING ROS-style robot (massless
base_footprint root) partly welded -- was a misreading: that census line only
appears for a static base (the fixed children of a physics-less Robot ARE
welded, physics.md), and a floating one folds them into base_link and rolls
freely. The census line now names the physics-less owner so it cannot be read
that way again.

The engine-free half (source pins) is tests/test_static_base_kinematic_wheels.py.
These arms run a self-contained URDF whose wheel axes are exactly parallel:

  staticBase TRUE (fixed)                          base moves < 1 mm
  staticBase TRUE + OMNISIM_KINEMATIC_WHEELS_WITH_PHYSICS=1   slides ~omega*r*t
  staticBase FALSE                                 rolls, no weld line

    python -m pytest tests/test_static_base_kinematic_wheels_engine.py -v

Honours OMNISIM_BINARY (scripts/dev/headless_runner.py does), so a scratch
build can be pinned without touching the live engine, and an already-set
OMNISIM_HOME (a worktree without a built engine can borrow an install). CPU `newtonSolver
"mujoco"`, three headless runs of ~10 s each.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
HATCH = "OMNISIM_KINEMATIC_WHEELS_WITH_PHYSICS"

WHEEL_RADIUS = 0.05
WHEEL_SPEED = 4.0  # rad/s
DRIVE_S = 2.0      # the probe drives from t=0.5 s to t=2.5 s

# A ROS-convention base: massless base_footprint root, a 10 kg base_link, fixed
# sensor links with mass, two wheels on EXACTLY parallel joint axes (the shape
# OmKinematicDifferentialWheels recognises).
URDF = """<?xml version="1.0"?>
<robot name="kinwheels">
  <link name="base_footprint"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 0.05"/>
  </joint>
  <link name="base_link">
    <inertial><origin xyz="0 0 0.05"/><mass value="10.0"/>
      <inertia ixx="0.083" iyy="0.141" izz="0.208" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><origin xyz="0 0 0.08"/><geometry><box size="0.4 0.3 0.1"/></geometry></collision>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/><child link="lidar_link"/><origin xyz="0.15 0 0.15"/>
  </joint>
  <link name="lidar_link">
    <inertial><mass value="0.2"/><inertia ixx="1e-4" iyy="1e-4" izz="1e-4" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><cylinder radius="0.03" length="0.04"/></geometry></collision>
  </link>
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/><child link="caster_link"/><origin xyz="0.16 0 -0.025"/>
  </joint>
  <link name="caster_link">
    <inertial><mass value="0.1"/><inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><geometry><sphere radius="0.025"/></geometry></collision>
  </link>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="left_wheel_link"/>
    <origin xyz="-0.05 0.18 0"/><axis xyz="0 1 0"/><limit effort="20" velocity="20"/>
  </joint>
  <link name="left_wheel_link">
    <inertial><mass value="0.5"/><inertia ixx="4e-4" iyy="6e-4" izz="4e-4" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><origin rpy="1.5707963267948966 0 0"/><geometry><cylinder radius="0.05" length="0.03"/></geometry></collision>
  </link>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="right_wheel_link"/>
    <origin xyz="-0.05 -0.18 0"/><axis xyz="0 1 0"/><limit effort="20" velocity="20"/>
  </joint>
  <link name="right_wheel_link">
    <inertial><mass value="0.5"/><inertia ixx="4e-4" iyy="6e-4" izz="4e-4" ixy="0" ixz="0" iyz="0"/></inertial>
    <collision><origin rpy="1.5707963267948966 0 0"/><geometry><cylinder radius="0.05" length="0.03"/></geometry></collision>
  </link>
</robot>
"""

CONTROLLER = '''"""Drive both wheels at --speed rad/s from t=0.5 s to t=2.5 s; write the base path."""
import json, sys
from omnisim import Supervisor
args = dict(zip(sys.argv[1::2], sys.argv[2::2]))
sup = Supervisor(); dt = int(sup.getBasicTimeStep()); me = sup.getSelf()
motors = [sup.getDevice(n) for n in ("left_wheel_joint_motor", "right_wheel_joint_motor")]
for m in motors:
    m.setPosition(float("inf")); m.setVelocity(0.0)
trace = []
while sup.step(dt) != -1:
    t = sup.getTime()
    if t >= 0.5:
        for m in motors:
            m.setVelocity(float(args["--speed"]))
    trace.append([t] + list(me.getPosition()))
    if t >= 2.5:
        break
json.dump({"motors": sum(m is not None for m in motors), "trace": trace}, open(args["--out"], "w"))
sup.simulationQuit(0)
'''


def _world(urdf: Path, out: Path, static_base: bool) -> str:
    return """#OMNISIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_static_base_kinematic_wheels.py -- do not commit.

WorldInfo {
  basicTimeStep 4
  newtonSolver "mujoco"
}
Viewpoint { position -3 0 1.5 }
DEF FLOOR Solid {
  translation 0 0 -0.01
  name "floor"
  children [ Shape { geometry Box { size 20 20 0.02 } } ]
  boundingObject Box { size 20 20 0.02 }
}
DEF R URDFRobot {
  url "%(URDF)s"
  translation 0 0 0.002
  name "probe"
  staticBase %(STATIC)s
  supervisor TRUE
  controller "kin_probe"
  controllerArgs [ "--out" "%(OUT)s" "--speed" "%(SPEED)s" ]
}
""" % {"URDF": urdf.as_posix(), "OUT": out.as_posix(), "STATIC": "TRUE" if static_base else "FALSE",
       "SPEED": WHEEL_SPEED}


# --------------------------------------------------------------------------- engine arms


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


def _run(tmp_path: Path, tag: str, static_base: bool, env_extra=None):
    proj = tmp_path / tag
    (proj / "worlds").mkdir(parents=True)
    ctrl = proj / "controllers" / "kin_probe"
    ctrl.mkdir(parents=True)
    (ctrl / "kin_probe.py").write_text(CONTROLLER, encoding="utf-8")
    urdf = proj / "kinwheels.urdf"
    urdf.write_text(URDF, encoding="utf-8")
    out = proj / "result.json"
    world = proj / "worlds" / "kin.omniworld"
    world.write_text(_world(urdf, out, static_base), encoding="utf-8")
    log = proj / "engine.log"
    env = dict(os.environ)
    home = Path(env.get("OMNISIM_HOME") or REPO)  # a binary-less worktree can borrow an install
    env["OMNISIM_HOME"] = str(home)
    env["OMNISIM_LOG_PATH"] = str(log)
    env.pop("OMNISIM_NEWTON_MJWARP", None)
    env.pop(HATCH, None)
    env.update(env_extra or {})
    subprocess.run([sys.executable, "-m", "omnisim", "run-headless", str(world), "--duration", "60"],
                   cwd=str(home), env=env, capture_output=True, text=True, timeout=300)
    assert out.is_file(), f"{tag}: the probe controller wrote no result (see {log})"
    res = json.loads(out.read_text())
    assert res["motors"] == 2
    (x0, y0), (x1, y1) = res["trace"][0][1:3], res["trace"][-1][1:3]
    moved = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    return moved, text


ENGINE = pytest.mark.engine
NO_BINARY = pytest.mark.skipif(_binary() is None, reason="no omnisim-bin in this clone")


@ENGINE
@NO_BINARY
def test_static_base_stays_put(tmp_path):
    moved, log = _run(tmp_path, "static", static_base=True)
    assert moved < 1e-3, f"a staticBase robot slid {moved:.4f} m on its wheels"
    assert "is a static base" in log  # the kinematic mode was considered and refused
    m = re.search(r"Solid\(s\) with Physics under physics-less ancestors attached to the static environment"
                  r".*?they hang under the physics-less ([^,;]+)", log)
    assert m and "probe" in m.group(1), "the weld census must name the static-base robot"


@ENGINE
@NO_BINARY
def test_hatch_restores_the_kinematic_slide(tmp_path):
    # Proves the rig sees the defect: with the old behaviour the base slides by omega * r * t.
    moved, _ = _run(tmp_path, "static_hatch", static_base=True, env_extra={HATCH: "1"})
    assert moved > 0.5 * WHEEL_SPEED * WHEEL_RADIUS * DRIVE_S


@ENGINE
@NO_BINARY
def test_floating_ros_base_rolls_and_is_not_welded(tmp_path):
    moved, log = _run(tmp_path, "float", static_base=False)
    assert moved > 0.5 * WHEEL_SPEED * WHEEL_RADIUS * DRIVE_S, f"the floating base moved only {moved:.4f} m"
    assert "attached to the static environment" not in log
    assert "is a static base" not in log
