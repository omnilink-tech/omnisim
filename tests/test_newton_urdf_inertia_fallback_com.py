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

"""A rejected URDF inertia tensor keeps the link's centre of mass in the solver (engine half).

The defect this pins (2026-10-20, AgiBot G2 body_link4): a tensor that violates the
triangle inequality falls back to bounding-object inertia, and that fallback also
dropped the declared <inertial><origin>, so the solver stepped the link with its COM
at the link origin. A declared tensor that is not positive definite (an all-zero one
included) on a link with mass did the same. The importer now marks the block DEF
URDF_INERTIA_FALLBACK and
OmSolid's Newton registration keeps the declared mass and COM, taking only the
rotational inertia from the bounding object.

Probe: a passive (effort 0) pendulum hinged about Y, released horizontal, whose arm
declares 1 kg at 0.1 m out with a rejected tensor (TENSORS: impossible, not positive
definite, all zero). A
massless tip frame 0.2 m out reports where the arm is. With the COM kept the arm
swings; with the COM at the hinge (OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM=0) gravity
has no lever arm and it does not move. The arm deliberately has no massive fixed
child: the composite-inertia path already carried the COM for those.

The engine-free half is tests/dev/test_urdf_inertia_fallback_keeps_com.py.

    python -m pytest tests/test_newton_urdf_inertia_fallback_com.py -v

OMNISIM_BINARY selects a scratch build. Five engine launches, a few seconds each.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
PIVOT_Z = 0.5
TIP = 0.2

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)

# Each rejected tensor, and the warning fragment that proves the fallback ran.
TENSORS = {
    # PD, but a + b = 0.002 < c = 0.01.
    "triangle_inequality": ('ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.01"',
                            "violates the triangle inequality"),
    # ixx*iyy - ixy^2 < 0.
    "not_positive_definite": ('ixx="0.001" ixy="0.002" ixz="0" iyy="0.001" iyz="0" izz="0.001"',
                              "is not positive definite"),
    # Mass declared, tensor all zero.
    "all_zero": ('ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"', "is not positive definite"),
}


def _urdf(inertia: str) -> str:
    return f"""<?xml version="1.0"?>
<robot name="fallback_pendulum">
  <link name="base">
    <inertial><mass value="1.0"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0.1 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia {inertia}/>
    </inertial>
  </link>
  <joint name="swing" type="continuous">
    <parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
    <limit effort="0" velocity="100"/>
  </joint>
  <link name="tip"/>
  <joint name="tip_mount" type="fixed">
    <parent link="arm"/><child link="tip"/><origin xyz="{TIP} 0 0"/>
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


def _run(tmp_path: Path, inertia: str, extra_env: dict | None = None) -> tuple[dict, str]:
    """({t_ms: tip z}, engine log text)."""
    u = tmp_path / "pendulum.urdf"
    u.write_text(_urdf(inertia), encoding="utf-8")
    world = tmp_path / "pendulum.omniworld"
    world.write_text("#VRML_SIM R2025a utf8\nWorldInfo { basicTimeStep 2 }\n"
                     "Viewpoint { orientation 0 0 1 0 position -3 0 1 }\n"
                     f'URDFRobot {{ url "{u.as_posix()}" translation 0 0 {PIVOT_Z} name "pend" '
                     'staticBase TRUE controller "<none>" }\n', encoding="utf-8")
    traj, log = tmp_path / "traj.tsv", tmp_path / "log.txt"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_ALL="1", OMNISIM_PROBE_TRAJ_MS="300")
    env.pop("OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM", None)
    env.update(extra_env or {})
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        "--stdout", "--stderr", str(world)], env=env, timeout=120, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    z: dict = {}
    if traj.is_file():
        for line in traj.read_text(encoding="utf-8").splitlines():
            c = line.split("\t")
            if len(c) >= 5 and c[1] == "tip":
                z[round(float(c[0]))] = float(c[4])
    if not z:
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
        pytest.fail("the pendulum probe produced no tip trajectory:\n%s" % text[-1500:])
    return z, text


@pytest.mark.parametrize("case", sorted(TENSORS))
def test_fallback_keeps_the_declared_com_so_the_arm_swings(tmp_path, case):
    inertia, fragment = TENSORS[case]
    z, log = _run(tmp_path, inertia)
    assert fragment in log, "the fixture must exercise the fallback:\n" + log[-1500:]
    assert "kept the declared mass (1 kg) and centre of mass (0.1 0 0)" in log, (
        "the warning must say the mass and COM were kept:\n" + log[-1500:])
    drop = PIVOT_Z - z[max(t for t in z if t <= 150)]
    # A 1 kg point at 0.1 m released horizontal: ~1 rad by 150 ms, so the 0.2 m tip
    # falls well over 5 cm. With the COM at the hinge it does not move at all.
    assert drop > 0.05, (
        f"the arm's declared COM is 0.1 m from the hinge, so it must swing; the tip dropped only "
        f"{drop:.4f} m by 150 ms. The inertia fallback is dropping the centre of mass again")


@pytest.mark.parametrize("case", ["triangle_inequality", "all_zero"])
def test_hatch_restores_the_com_at_the_link_origin(tmp_path, case):
    z, log = _run(tmp_path, TENSORS[case][0], {"OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM": "0"})
    assert "dropped the declared centre of mass" in log, log[-1500:]
    drop = PIVOT_Z - z[max(t for t in z if t <= 150)]
    assert abs(drop) < 0.005, (
        f"OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM=0 puts the COM on the hinge, where gravity has no lever "
        f"arm; the tip should not move, but it dropped {drop:.4f} m")
