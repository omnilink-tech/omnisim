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

"""URDF <mesh scale> survives the offline converter (engine-free).

The defect this pins (2026-11-12): scripts/dev/urdf_import.py parsed a mesh's `scale` but never wrote
it, so a millimetre mesh (`scale="0.001 0.001 0.001"`, as in the ORCA Hand's URDF) came out 1000x
too large in both its visual and its boundingObject. The native importer (OmUrdfImporter.cpp) keeps
it. The converter now writes `Mesh { url ... scale sx sy sz }` wherever the scale is not unit.

    python -m pytest tests/dev/test_urdf_import_mesh_scale.py -v
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def _load_converter():
    spec = importlib.util.spec_from_file_location("omnisim_urdf_import_mesh_scale_under_test",
                                                  REPO / "scripts" / "dev" / "urdf_import.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


U = _load_converter()


def _robot(tmp_path: Path, scale: str | None) -> str:
    (tmp_path / "part.stl").write_text("solid p\nendsolid p\n", encoding="utf-8")
    sc = f' scale="{scale}"' if scale else ""
    urdf = f"""<?xml version="1.0"?>
<robot name="probe">
  <link name="base">
    <inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><mesh filename="part.stl"{sc}/></geometry></visual>
    <collision><geometry><mesh filename="part.stl"{sc}/></geometry></collision>
  </link>
</robot>
"""
    path = tmp_path / "probe.urdf"
    path.write_text(urdf, encoding="utf-8")
    return U.emit_robot(U.parse_urdf(path))


def test_millimetre_scale_is_written_on_visual_and_collider(tmp_path):
    vrml = _robot(tmp_path, "0.001 0.001 0.001")
    meshes = re.findall(r"Mesh \{[^}]*\}", vrml)
    assert len(meshes) == 2, vrml
    for m in meshes:
        assert "scale 0.001 0.001 0.001" in m, m
    assert re.search(r"boundingObject Mesh \{[^}]*scale 0\.001", vrml), vrml


def test_unit_or_missing_scale_writes_no_scale_field(tmp_path):
    for sc in (None, "1 1 1"):
        vrml = _robot(tmp_path, sc)
        assert not re.search(r'" scale [-0-9]', vrml), vrml   # the field, not the word in a temp path
