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

"""A Collada mesh whose geometry several nodes instance must load, not crash.

The defect this pins (found 2026-10-04 on Fictionlab's Leo Rover, fixed
2026-10-09): OmMesh sized its vertex and index arrays by counting
scene->mMeshes, each mesh once, then filled them by walking the node graph,
which writes a mesh once per node that instances it. A file that instances
geometry overran the arrays and the engine died with an access violation and
no log line. Leo's Chassis.dae (Blender export: 27 geometries, 35 instances)
crashed every load of the robot.

Probe: a generated Collada file with one 20,000-triangle geometry instanced by
four nodes, as the visual of a Shape, next to a falling Solid that lets the
trajectory probe end the run cleanly. The engine must exit 0 with the run's
trajectory written, rather than die mid-load.

    python -m pytest tests/test_mesh_collada_instanced_geometry.py -v

OMNISIM_BINARY selects a scratch build. One engine launch, a few seconds.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
TRIANGLES = 20000
INSTANCES = 4


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(_binary() is None, reason="no simulator binary in this clone; build first")


def _instanced_collada() -> str:
    # A strip of separate small triangles (3 unshared vertices each), so the
    # vertex count -- and the old overrun -- scales with TRIANGLES.
    pos, idx = [], []
    for t in range(TRIANGLES):
        x, y = (t % 200) * 0.005, (t // 200) * 0.005
        pos += [x, y, 0.0, x + 0.004, y, 0.0, x, y + 0.004, 0.0]
        idx += [3 * t, 3 * t + 1, 3 * t + 2]
    nodes = "".join(
        f'<node id="n{i}" name="n{i}"><matrix>1 0 0 {i * 1.2} 0 1 0 0 0 0 1 0 0 0 0 1</matrix>'
        f'<instance_geometry url="#strip"/></node>' for i in range(INSTANCES))
    return f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries>
    <geometry id="strip" name="strip"><mesh>
      <source id="strip-pos"><float_array id="strip-pos-a" count="{len(pos)}">{" ".join(f"{v:g}" for v in pos)}</float_array>
        <technique_common><accessor source="#strip-pos-a" count="{len(pos) // 3}" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common></source>
      <vertices id="strip-v"><input semantic="POSITION" source="#strip-pos"/></vertices>
      <triangles count="{TRIANGLES}"><input semantic="VERTEX" source="#strip-v" offset="0"/>
        <p>{" ".join(map(str, idx))}</p></triangles>
    </mesh></geometry>
  </library_geometries>
  <library_visual_scenes><visual_scene id="scene">{nodes}</visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""


def test_collada_with_instanced_geometry_loads(tmp_path):
    dae = tmp_path / "instanced_strip.dae"
    dae.write_text(_instanced_collada(), encoding="utf-8")
    world = tmp_path / "instanced_collada.omniworld"
    world.write_text(f"""#VRML_SIM R2025a utf8
EXTERNPROTO "omnisim://projects/objects/floors/protos/RectangleArena.proto"
WorldInfo {{ basicTimeStep 8 }}
Viewpoint {{ orientation 0 0 1 0 position -3 0 1 }}
RectangleArena {{ floorSize 8 8 wallHeight 0.02 }}
Shape {{ geometry Mesh {{ url "{dae.as_posix()}" }} }}
Solid {{ translation 0 -2 0.2 name "PROBE" boundingObject Sphere {{ radius 0.05 }} physics Physics {{ }} }}
""", encoding="utf-8")
    log = tmp_path / "engine.log"
    traj = tmp_path / "traj.tsv"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_MS="300")
    try:
        proc = subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                               "--stdout", "--stderr", str(world)], env=env, timeout=120, capture_output=True)
        code = proc.returncode
    except subprocess.TimeoutExpired:
        code = None
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    assert code == 0, f"the engine exited {code} loading an instanced Collada mesh; log tail:\n{text[-1500:]}"
    assert traj.is_file() and "PROBE" in traj.read_text(encoding="utf-8"), (
        "the run never stepped; log tail:\n" + text[-1500:])
