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

"""A URDF joint with <limit effort="0"> is passive under Newton, not driven at maxVelocity.

The defect this pins (2026-10-10): effort="0" imports as maxTorque 0, which OmMotor
treats as "no control" -- so isPIDPositionControl() was false and the Newton bridge
fell into its velocity branch, sending targetVelocity(), i.e. maxVelocity. The runtime
registered the joint with its default (unlimited) effort, since a zero effort limit is
not registered at build, and the push of the available torque skipped a value equal to
maxTorque. Ekumen's Andino with its swivel caster declared passive this way: the caster
span at ~112 rad/s from t=0 and the robot drove in random directions (straights 0.14-0.6
of command, turns -72..+154 deg for 90); fixed, it drives 0.996 and turns 81 deg.

Probe: two pendulums on fixed bases, released horizontal. effort="0" swings down freely
under gravity and, with no energy input, never rises above its pivot; effort="50" is a
motor holding its start angle. Before the fix the effort-0 bob went over the top.

Second defect (2026-10-13): the effort-0 joint was free of the actuator but still
SPEED-CAPPED. The runtime's post-step clamp holds every joint to its URDF velocity limit,
which models a motor's back-EMF -- and an off motor has none. Dexmate's Vega torso_flip
(effort 0, velocity 0.1) crept onto its stop at 0.11 rad/s for 9.5 s where plain MuJoCo
drops it in 0.4 s. A third pendulum carries effort 0 with velocity 0.1: free, it reaches
the bottom inside the window; capped, it would need ~16 s for the quarter swing.

    python -m pytest tests/test_newton_urdf_zero_effort_passive.py -v

OMNISIM_BINARY selects a scratch build. One engine launch, a few seconds.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PIVOT_Z = 0.5
ARM = 0.3


def _urdf(tag: str, effort: str, velocity: str = "100") -> str:
    return f"""<?xml version="1.0"?>
<robot name="pend_{tag}">
  <link name="base_{tag}">
    <inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="arm_{tag}">
    <inertial><origin xyz="{ARM} 0 0"/><mass value="0.5"/>
      <inertia ixx="0.0005" iyy="0.0005" izz="0.0005" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="swing_{tag}" type="continuous">
    <parent link="base_{tag}"/><child link="arm_{tag}"/><axis xyz="0 1 0"/>
    <limit effort="{effort}" velocity="{velocity}"/>
  </joint>
  <link name="bob_{tag}">
    <inertial><mass value="0.001"/><inertia ixx="1e-7" iyy="1e-7" izz="1e-7" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="bob_mount_{tag}" type="fixed">
    <parent link="arm_{tag}"/><child link="bob_{tag}"/><origin xyz="{ARM} 0 0"/>
  </joint>
</robot>
"""


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(_binary() is None, reason="no simulator binary in this clone; build first")


def _bob_heights(tmp_path: Path) -> dict:
    robots = []
    for i, (tag, effort, velocity) in enumerate((("free", "0", "100"), ("held", "50", "100"),
                                                 ("slowcap", "0", "0.1"))):
        u = tmp_path / f"pend_{tag}.urdf"
        u.write_text(_urdf(tag, effort, velocity), encoding="utf-8")
        robots.append(f'URDFRobot {{ url "{u.as_posix()}" translation 0 {i * 1.0} {PIVOT_Z} name "pend_{tag}" '
                      f'staticBase TRUE controller "<none>" }}')
    world = tmp_path / "pendulums.omniworld"
    world.write_text("#VRML_SIM R2025a utf8\nWorldInfo { basicTimeStep 4 }\n"
                     "Viewpoint { orientation 0 0 1 0 position -3 0 1 }\n" + "\n".join(robots) + "\n",
                     encoding="utf-8")
    traj = tmp_path / "traj.tsv"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(tmp_path / "log.txt"),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_ALL="1", OMNISIM_PROBE_TRAJ_MS="1500")
    env.pop("OMNISIM_NEWTON_AVAILABLE_TORQUE", None)
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        "--stdout", "--stderr", str(world)], env=env, timeout=120, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    z = {}
    if traj.is_file():
        for line in traj.read_text(encoding="utf-8").splitlines():
            c = line.split("\t")
            if len(c) >= 5 and c[1].startswith("bob_") and float(c[0]) >= 50.0:
                z.setdefault(c[1], []).append(float(c[4]))
    return z


def test_zero_effort_joint_swings_freely_and_is_not_driven(tmp_path):
    z = _bob_heights(tmp_path)
    assert "bob_free" in z and "bob_held" in z, f"bobs missing from the trajectory probe: {sorted(z)}"
    free, held = z["bob_free"], z["bob_held"]
    assert min(free) < PIVOT_Z - 0.8 * ARM, (
        f"the effort=0 pendulum should swing down under gravity; lowest bob z {min(free):.3f}")
    assert max(free) < PIVOT_Z + 0.02, (
        f"the effort=0 pendulum went over the top (bob z {max(free):.3f} > pivot {PIVOT_Z}): it is being "
        f"driven, not passive")
    assert min(held) > PIVOT_Z - 0.05, (
        f"the effort=50 motor should hold its start angle; bob dropped to z {min(held):.3f}")


def test_zero_effort_joint_is_not_held_to_its_velocity_limit(tmp_path):
    z = _bob_heights(tmp_path)
    assert "bob_slowcap" in z, f"bob missing from the trajectory probe: {sorted(z)}"
    low = min(z["bob_slowcap"])
    assert low < PIVOT_Z - 0.8 * ARM, (
        f"the effort=0 pendulum with a 0.1 rad/s URDF velocity limit should still swing down freely; "
        f"lowest bob z {low:.3f}. It is being held to the motor's speed limit, which an off motor does "
        f"not have")
