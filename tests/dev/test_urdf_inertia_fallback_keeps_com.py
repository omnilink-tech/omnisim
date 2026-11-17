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

"""A rejected URDF inertia tensor keeps the link's mass and centre of mass (engine-free half).

The defect this pins (2026-10-20, AgiBot genie_sim_robot_model @6d5b215, G2 body
xacro, link body_link4): a tensor whose principal moments violate a + b >= c makes
the importer fall back to bounding-object inertia. The fallback wrote mass and
centerOfMass but no inertiaMatrix, and the Newton registration passes a COM only
alongside an explicit tensor -- so the solver stepped body_link4 with its COM at
(0, 0, 0) instead of the declared (-0.071, 0, 0.030), and the warning did not say so.

The same held for a declared tensor that is not positive definite (an all-zero one
included) on a link with mass: parseLink dropped it, and the COM went with it.

Both importers now write the fallback block marked DEF URDF_INERTIA_FALLBACK, always
with the declared centerOfMass and never an inertiaMatrix; OmSolid's Newton
registration keys on the marker, keeps the mass and COM, and takes only the
rotational inertia from the bounding object, about that COM.
OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM=0 (value-parsed) restores the old block.

This file drives the offline converter (scripts/dev/urdf_import.py), which must
produce the same block as the native importer, and pins the marker string on both
C++ halves so neither can be renamed alone. The engine half (a passive pendulum that
only swings if the COM reaches the solver) is
tests/test_newton_urdf_inertia_fallback_com.py.

    python -m pytest tests/dev/test_urdf_inertia_fallback_keeps_com.py -v
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
MARKER = "URDF_INERTIA_FALLBACK"
COM = (-0.071, 0.0, 0.030)


def _load_converter():
    spec = importlib.util.spec_from_file_location("omnisim_urdf_import_fallback_under_test",
                                                  REPO / "scripts" / "dev" / "urdf_import.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


U = _load_converter()

# PD, but a + b = 0.002 < c = 0.01: no rigid body has it (MuJoCo refuses it).
TRIANGLE = 'ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.01"'
# PD and below the importer's 1e-9 floor on one moment.
TOO_SMALL = 'ixx="1e-12" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"'
VALID = 'ixx="0.004" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.006"'
# Not positive definite: ixx*iyy - ixy^2 < 0 (a product copied wrong).
NOT_PD = 'ixx="0.001" ixy="0.002" ixz="0" iyy="0.001" iyz="0" izz="0.001"'
# The "I don't know" idiom: mass declared, tensor all zero (Comau Racer5 link_5 in an
# outreach rig ships 0.98 kg this way, its COM 4 cm out).
ALL_ZERO = 'ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"'


def _urdf(inertia: str, com=COM, mass: float = 2.5) -> str:
    return f"""<?xml version="1.0"?>
<robot name="fallback">
  <link name="base"/>
  <link name="body_link4">
    <collision><geometry><box size="0.2 0.1 0.1"/></geometry></collision>
    <inertial>
      <origin xyz="{com[0]} {com[1]} {com[2]}" rpy="0 0 0"/>
      <mass value="{mass}"/>
      <inertia {inertia}/>
    </inertial>
  </link>
  <joint name="j4" type="revolute">
    <parent link="base"/><child link="body_link4"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="10" velocity="1"/>
  </joint>
</robot>
"""


def _physics_block(tmp_path: Path, inertia: str, com=COM, mass: float = 2.5) -> tuple[str, str]:
    """(the 'physics ... Physics' header line, the block body) of body_link4."""
    path = tmp_path / "robot.urdf"
    path.write_text(_urdf(inertia, com, mass), encoding="utf-8")
    vrml = U.emit_robot(U.parse_urdf(path))
    # body_link4 is the only link with an <inertial>; base emits no Physics.
    m = re.search(r"^[ \t]*(physics [^\n]*Physics \{)\n(.*?)^[ \t]*\}", vrml, re.M | re.S)
    assert m, "no Physics block emitted for the link:\n" + vrml
    return m.group(1), m.group(2)


def _com_of(body: str):
    m = re.search(r"centerOfMass \[([^\]]+)\]", body)
    return None if m is None else tuple(float(v) for v in m.group(1).split())


@pytest.fixture(autouse=True)
def _default_hatch(monkeypatch):
    monkeypatch.delenv("OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM", raising=False)
    monkeypatch.delenv("OMNISIM_URDF_INERTIAL_RPY", raising=False)


@pytest.mark.parametrize("inertia", [TRIANGLE, TOO_SMALL, NOT_PD, ALL_ZERO],
                         ids=["triangle_inequality", "below_floor", "not_positive_definite", "all_zero"])
def test_rejected_tensor_keeps_mass_and_com(tmp_path, inertia):
    header, body = _physics_block(tmp_path, inertia)
    assert MARKER in header, f"the fallback block must be marked DEF {MARKER}: {header!r}"
    assert re.search(r"^\s*mass 2\.5$", body, re.M), "the declared mass must survive:\n" + body
    assert _com_of(body) == pytest.approx(COM), (
        f"the declared centre of mass must survive the fallback; got {_com_of(body)}:\n{body}")
    assert "inertiaMatrix" not in body, (
        "a rejected tensor must not be emitted -- only the rotational inertia is replaced:\n" + body)


def test_fallback_keeps_a_com_at_the_link_origin_too(tmp_path):
    """The marker always carries a COM, so the engine never has to guess one."""
    header, body = _physics_block(tmp_path, TRIANGLE, com=(0.0, 0.0, 0.0))
    assert MARKER in header
    assert _com_of(body) == pytest.approx((0.0, 0.0, 0.0))


def test_massless_frame_with_a_zero_tensor_is_not_a_fallback(tmp_path):
    """mass 0 + an all-zero tensor is a frame, not a body: nothing is rejected."""
    header, body = _physics_block(tmp_path, ALL_ZERO, mass=0.0)
    assert MARKER not in header, f"a massless frame is not a fallback: {header!r}"


def test_valid_tensor_is_emitted_unmarked(tmp_path):
    header, body = _physics_block(tmp_path, VALID)
    assert MARKER not in header, "a valid tensor is not a fallback"
    assert _com_of(body) == pytest.approx(COM)
    assert "inertiaMatrix" in body


def test_hatch_restores_the_unmarked_block(tmp_path, monkeypatch):
    monkeypatch.setenv("OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM", "0")
    header, body = _physics_block(tmp_path, TRIANGLE)
    assert MARKER not in header, "OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM=0 must drop the marker"
    assert "inertiaMatrix" not in body


@pytest.mark.parametrize("inertia,defect", [(TRIANGLE, "triangle inequality"),
                                            (NOT_PD, "not positive definite")],
                         ids=["triangle_inequality", "not_positive_definite"])
def test_report_says_what_the_fallback_keeps(tmp_path, inertia, defect):
    path = tmp_path / "robot.urdf"
    path.write_text(_urdf(inertia), encoding="utf-8")
    warnings = "\n".join(U.build_report(U.parse_urdf(path))["warnings"])
    assert defect in warnings
    assert "keeps the declared mass and centre of mass" in warnings
    assert "replaces only the rotational inertia" in warnings


def test_both_engine_halves_name_the_same_marker():
    """The native importer writes the marker and OmSolid's Newton registration reads it.

    A source pin, because this lane cannot run the engine: renaming the DEF on one
    side alone would silently send the COM back to the link origin.
    """
    importer = (REPO / "src" / "omnisim" / "vrml" / "OmUrdfImporter.cpp").read_text(encoding="utf-8")
    solid = (REPO / "src" / "omnisim" / "nodes" / "OmSolid.cpp").read_text(encoding="utf-8")
    assert f'"physics DEF {MARKER} Physics {{' in importer
    assert "OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM" in importer
    # The warning must say what was kept and what was replaced.
    assert "kept the declared mass" in importer and "replaced only the" in importer
    # The non-PD rejection (parseLink drops the tensor) feeds the same marker.
    assert re.search(r"tensorRejected =[^;]*declaredNotPositiveDefinite|tensorRejected =[^;]*tensorNotPd",
                     importer, re.S)
    assert f'defName() == QLatin1String("{MARKER}")' in solid
    assert U.INERTIA_FALLBACK_DEF == MARKER
