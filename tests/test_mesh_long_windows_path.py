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

"""A mesh whose path is longer than 260 characters must still load (Windows).

The defect this pins (found 2026-10-07 building a company's robot kit, fixed
2026-10-09): Assimp's default file access opens meshes with _wfopen, which on
Windows cannot open a path longer than MAX_PATH unless the executable is
long-path aware. A URDF package unpacked a few folders down is enough to cross
it. The import failed, the geometry silently went missing, and only a strict
`run-headless` noticed. OmMesh, OmCadShape and OmCloth now hand Assimp a QFile-
backed IO system (OmAssimpIoSystem), and QFile opens long paths.

Probe: a 0.1 m cube written as an ASCII STL into a folder deep enough that the
full path exceeds 260 characters, used as the boundingObject of a body dropped
onto the floor. Loaded, it rests with its centre at 0.05 m; not loaded, the
body has no collider and falls through the floor. Also asserts the import
warning is absent.

    python -m pytest tests/test_mesh_long_windows_path.py -v

OMNISIM_BINARY selects a scratch build. One engine launch, a few seconds.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

pytestmark = pytest.mark.skipif(sys.platform != "win32", reason="MAX_PATH is a Windows limit")


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    candidate = REPO / "msys64" / "mingw64" / "bin" / "omnisim-bin.exe"
    return candidate if candidate.is_file() else None


def _cube_stl(half: float) -> str:
    v = [(x, y, z) for x in (-half, half) for y in (-half, half) for z in (-half, half)]
    # two triangles per face, wound outward; facet normals are recomputed by Assimp
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    out = ["solid cube"]
    for a, b, c, d in faces:
        for tri in ((a, b, c), (a, c, d)):
            out.append("  facet normal 0 0 0\n    outer loop")
            out.extend(f"      vertex {v[i][0]} {v[i][1]} {v[i][2]}" for i in tri)
            out.append("    endloop\n  endfacet")
    out.append("endsolid cube")
    return "\n".join(out) + "\n"


@pytest.mark.skipif(_binary() is None, reason="no simulator binary in this clone; build first")
def test_mesh_beyond_max_path_loads(tmp_path):
    deep = tmp_path
    while len(str(deep / "cube.stl")) <= 280:
        deep = deep / "a_deeply_nested_robot_description_package_folder"
    os.makedirs("\\\\?\\" + str(deep))  # creating it needs the long-path prefix from Python too
    mesh = deep / "cube.stl"
    with open("\\\\?\\" + str(mesh), "w", encoding="ascii") as fh:
        fh.write(_cube_stl(0.05))
    assert len(str(mesh)) > 260

    world = tmp_path / "long_path_mesh.omniworld"
    world.write_text(f"""#VRML_SIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/floors/protos/RectangleArena.proto"
WorldInfo {{ basicTimeStep 4 }}
Viewpoint {{ orientation 0 0 1 0 position -2 0 0.5 }}
RectangleArena {{ floorSize 4 4 wallHeight 0.02 }}
Solid {{
  translation 0 0 0.06
  name "LONG_PATH_CUBE"
  boundingObject Mesh {{ url "{mesh.as_posix()}" }}
  physics Physics {{ density 500 }}
}}
""", encoding="utf-8")
    log = tmp_path / "engine.log"
    traj = tmp_path / "traj.tsv"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_MS="1500")
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        "--stdout", "--stderr", str(world)], env=env, timeout=120, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    assert "please verify mesh file" not in text, "the mesh failed to import; log tail:\n" + text[-1500:]
    rows = [r.split("\t") for r in traj.read_text(encoding="utf-8").splitlines()] if traj.is_file() else []
    z = [float(r[4]) for r in rows if len(r) > 4 and r[1] == "LONG_PATH_CUBE"]
    assert z, "no trajectory for the cube; log tail:\n" + text[-1500:]
    assert abs(z[-1] - 0.05) < 0.005, (
        f"the cube should rest on its mesh collider at z=0.05, ended at z={z[-1]:.4f}; log tail:\n{text[-1500:]}")
