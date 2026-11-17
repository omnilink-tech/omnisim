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

"""URDF <inertial><origin rpy> rotates the inertia tensor into the link frame (engine-free half).

The defect this pins (2026-10-15): both URDF importers -- the native one in
src/omnisim/vrml/OmUrdfImporter.cpp and the offline converter
scripts/dev/urdf_import.py, which must produce equivalent output -- used only the
inertial origin's xyz (centerOfMass) and dropped its rpy. Physics.inertiaMatrix is
in the Solid's (the link's) axes, so a tensor written in a rotated inertial frame
reached the solver unrotated: mass and centre of mass right, the rotational inertia
about every axis wrong. spot_description's hips and lower legs arrived with their
principal moments on the wrong axes; MuJoCo's own URDF import rotates them.

The converter now emits I_link = R I R^T (R = Rz(yaw) Ry(pitch) Rx(roll), URDF's
fixed-axis convention). This file checks it against the closed form and, when the
`mujoco` package is installed, against MuJoCo's own URDF import of the same file.
The engine half (a passive pendulum whose swing depends on the rotated tensor) is
tests/test_newton_urdf_inertial_rpy.py.

    python -m pytest tests/dev/test_urdf_import_inertial_rpy.py -v
"""

from __future__ import annotations

import importlib.util
import math
import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]


def _load_converter():
    spec = importlib.util.spec_from_file_location("omnisim_urdf_import_under_test",
                                                  REPO / "scripts" / "dev" / "urdf_import.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


U = _load_converter()

# A rod along the inertial frame's X (small moment about its own axis), that frame
# yawed 90 deg: in the link frame the rod lies along Y.
ROD = ("0 0 1.5707963267948966", (0.001, 0.0, 0.0, 0.05, 0.0, 0.05))
# A general tensor and orientation, for the convention check against MuJoCo.
GENERAL = ("0.3 -0.7 1.1", (0.06, 0.005, -0.003, 0.05, 0.002, 0.03))


def _urdf(rpy: str, t) -> str:
    ixx, ixy, ixz, iyy, iyz, izz = t
    return f"""<?xml version="1.0"?>
<robot name="probe">
  <link name="base"><inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="arm">
    <inertial><origin xyz="0.1 0 0" rpy="{rpy}"/><mass value="1"/>
      <inertia ixx="{ixx}" ixy="{ixy}" ixz="{ixz}" iyy="{iyy}" iyz="{iyz}" izz="{izz}"/></inertial>
  </link>
  <joint name="swing" type="continuous"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
    <limit effort="0" velocity="100"/></joint>
</robot>
"""


def _emitted_arm_tensor(tmp_path: Path, rpy: str, t) -> list[list[float]]:
    path = tmp_path / "probe.urdf"
    path.write_text(_urdf(rpy, t), encoding="utf-8")
    vrml = U.emit_robot(U.parse_urdf(path))
    # The arm Solid has no children, so the first inertiaMatrix after its name is its own.
    m = re.search(r'name "arm".*?inertiaMatrix \[\s*([^\]]+)\]', vrml, re.S)
    assert m, f"no inertiaMatrix emitted for the arm:\n{vrml}"
    v = [float(x) for x in m.group(1).split()]
    ixx, iyy, izz, ixy, ixz, iyz = v  # Physics.inertiaMatrix: [Ixx Iyy Izz], [Ixy Ixz Iyz]
    return [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]


def _closed_form(rpy: str, t) -> list[list[float]]:
    r, p, y = (float(x) for x in rpy.split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
         [-sp, cp * sr, cp * cr]]
    ixx, ixy, ixz, iyy, iyz, izz = t
    inertia = [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
    return [[sum(R[a][i] * inertia[i][j] * R[b][j] for i in range(3) for j in range(3)) for b in range(3)]
            for a in range(3)]


def _max_diff(a, b) -> float:
    return max(abs(a[i][j] - b[i][j]) for i in range(3) for j in range(3))


def test_yawed_rod_lands_on_the_link_y_axis(tmp_path, monkeypatch):
    monkeypatch.delenv("OMNISIM_URDF_INERTIAL_RPY", raising=False)
    got = _emitted_arm_tensor(tmp_path, *ROD)
    # Rod along the link's Y: small moment about Y, the large ones about X and Z.
    assert abs(got[0][0] - 0.05) < 1e-9 and abs(got[1][1] - 0.001) < 1e-9 and abs(got[2][2] - 0.05) < 1e-9, (
        f"the yawed inertial frame must swap Ixx and Iyy in the link frame; got diag "
        f"({got[0][0]:.6g}, {got[1][1]:.6g}, {got[2][2]:.6g}), want (0.05, 0.001, 0.05)")
    assert max(abs(got[0][1]), abs(got[0][2]), abs(got[1][2])) < 1e-9


def test_general_rotation_matches_the_closed_form(tmp_path, monkeypatch):
    monkeypatch.delenv("OMNISIM_URDF_INERTIAL_RPY", raising=False)
    got = _emitted_arm_tensor(tmp_path, *GENERAL)
    want = _closed_form(*GENERAL)
    assert _max_diff(got, want) < 1e-9, f"emitted {got}\nwant R I R^T {want}"


def test_rpy_convention_matches_mujocos_fixed_axis_euler(tmp_path, monkeypatch):
    # An independent implementation of the convention: URDF rpy is roll about the
    # FIXED X, then pitch about fixed Y, then yaw about fixed Z, which is MuJoCo's
    # extrinsic euler sequence "XYZ". The tensor MuJoCo builds from principal
    # moments placed in that frame must equal the converter's.
    #
    # NOT MuJoCo's URDF import: measured 2026-10-15, MuJoCo 3.11's URDF parser
    # drops the inertial rpy too (it passes the tensor as `fullinertia`, whose
    # eigendecomposition replaces the orientation it read from the origin), so
    # plain MuJoCo is not the reference for this one -- the URDF spec is, and
    # urdfdom / Gazebo / Drake / Pinocchio / PyBullet all rotate.
    mujoco = pytest.importorskip("mujoco")
    import numpy as np

    monkeypatch.delenv("OMNISIM_URDF_INERTIAL_RPY", raising=False)
    moments = (0.06, 0.05, 0.03)
    rpy = "0.3 -0.7 1.1"
    got = _emitted_arm_tensor(tmp_path, rpy, (moments[0], 0.0, 0.0, moments[1], 0.0, moments[2]))
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><compiler angle="radian" eulerseq="XYZ"/><worldbody><body name="arm">'
        f'<joint type="hinge" axis="0 1 0"/><inertial pos="0.1 0 0" euler="{rpy}" mass="1" '
        f'diaginertia="{moments[0]} {moments[1]} {moments[2]}"/></body></worldbody></mujoco>')
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "arm")
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, model.body_iquat[body])
    rot = rot.reshape(3, 3)
    mj = rot @ np.diag(model.body_inertia[body]) @ rot.T
    assert _max_diff(got, mj.tolist()) < 1e-9, f"converter {got}\nMuJoCo, euler XYZ {mj.tolist()}"


def test_hatch_restores_the_unrotated_tensor(tmp_path, monkeypatch):
    monkeypatch.setenv("OMNISIM_URDF_INERTIAL_RPY", "0")
    got = _emitted_arm_tensor(tmp_path, *ROD)
    assert abs(got[0][0] - 0.001) < 1e-12 and abs(got[1][1] - 0.05) < 1e-12, (
        f"OMNISIM_URDF_INERTIAL_RPY=0 must emit the tensor as declared; got diag "
        f"({got[0][0]}, {got[1][1]}, {got[2][2]})")


def test_engine_importer_folds_the_rpy_in_too():
    # The two importers must stay equivalent. The engine half is measured in
    # tests/test_newton_urdf_inertial_rpy.py; this is the cheap guard that the
    # native importer still routes the inertial rpy through the rotation.
    src = (REPO / "src" / "omnisim" / "vrml" / "OmUrdfImporter.cpp").read_text(encoding="utf-8")
    assert "OMNISIM_URDF_INERTIAL_RPY" in src
    assert re.search(r"rotateUrdfInertia\(link\.inertial\.ixx.*io\.roll, io\.pitch, io\.yaw\)", src, re.S), (
        "parseLink no longer rotates the <inertial> tensor by its origin rpy")
