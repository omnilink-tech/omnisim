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

"""A Collada mesh's node transforms must be composed root-first.

The defect this pins (found 2026-10-15 on Rover Robotics' MEGA, fixed the same
day): OmMesh and OmCadShape composed a node's absolute transform LEAF-FIRST,
`node * parent * ... * root` (inherited from Webots), where the vertices need
`root * ... * parent * node`. The two agree only when at most one level carries
a transform, and Collada always has two: the importer's own up-axis rotation on
the root, and any node translation aiProcess_OptimizeGraph cannot bake away --
it keeps the node of every mesh that is instanced more than once. So the error
needs instanced geometry under a translated node, and then it is large:

- mega_wheels.dae (7 geometries, bolts instanced 4x) came out 26.5 mm off its
  axle. Alone, the wheels bounced the 103 kg base +/-2.7 cm; as published they
  stalled at half speed against the 10 N*m default torque and the base tilted
  3-3.75 deg. Plain MuJoCo on a correctly placed hull rolls true (0.27 mm).
- mega_payload.dae: vertices up to 0.96 m out of place; mega_chassis.dae
  0.53 m; max_chassis.dae 0.57 m.

None of the 82 Collada files tracked in this repository moves (each is either
one node deep or its transforms commute), which is why nothing shipped showed it.

The probe here is the smallest file with the shape of the bug: one 0.1 m cube
geometry instanced by two nodes at (0, +/-0.15, 0.02) in a Z_UP file. As a
boundingObject it must reach the solver where it was authored, the two cubes
side by side along y: body-frame box [-0.05, -0.2, -0.03]..[0.05, 0.2, 0.07].
The leaf-first product stacked them along z instead
([-0.05, -0.03, -0.2]..[0.05, 0.07, 0.2]), which the hatch
OMNISIM_MESH_NODE_TRANSFORM_FIX=0 must still reproduce. The geometry is read
back from the solver's own model (OMNISIM_NEWTON_SAVE_MJCF).

Both tests launch the engine (honours OMNISIM_BINARY; a few seconds each). The
engine-free half -- a source pin on both node walks -- is
tests/test_mesh_node_transform_order_source.py, so it runs in `make tests-unit`.

    python -m pytest tests/test_mesh_collada_node_transform_order.py -v
"""

from __future__ import annotations

import itertools
import os
import re
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
NODES = ((0.0, 0.15, 0.02), (0.0, -0.15, 0.02))
HALF = 0.05
FIXED_BOX = ((-0.05, -0.2, -0.03), (0.05, 0.2, 0.07))
LEAF_FIRST_BOX = ((-0.05, -0.03, -0.2), (0.05, 0.07, 0.2))
TOL = 1e-5

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)


def _binary():
    override = os.environ.get("OMNISIM_BINARY")
    if override:
        return Path(override) if Path(override).is_file() else None
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


needs_engine = pytest.mark.skipif(_binary() is None, reason="no simulator binary in this clone; build first")


def _cube_dae() -> str:
    v = [(sx * HALF, sy * HALF, sz * HALF) for sx, sy, sz in itertools.product((-1, 1), repeat=3)]
    tris = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
            (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    pos = " ".join(f"{c:g}" for p in v for c in p)
    idx = " ".join(str(i) for t in tris for i in t)
    inst = "".join(f'<node id="n{i}" name="n{i}"><matrix>1 0 0 {x:g} 0 1 0 {y:g} 0 0 1 {z:g} 0 0 0 1</matrix>'
                   f'<instance_geometry url="#cube"/></node>' for i, (x, y, z) in enumerate(NODES))
    return f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries><geometry id="cube" name="cube"><mesh>
    <source id="cube-pos"><float_array id="cube-pos-a" count="24">{pos}</float_array>
      <technique_common><accessor source="#cube-pos-a" count="8" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common></source>
    <vertices id="cube-v"><input semantic="POSITION" source="#cube-pos"/></vertices>
    <triangles count="12"><input semantic="VERTEX" source="#cube-v" offset="0"/><p>{idx}</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene"><node id="top" name="top">{inst}</node></visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""


def _quat_rotate(q, v):
    w, x, y, z = q
    r = ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
         (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
         (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))
    return tuple(sum(r[i][k] * v[k] for k in range(3)) for i in range(3))


def _mesh_box_in_body_frame(tmp_path, tag, extra_env=None):
    d = tmp_path / tag
    d.mkdir()
    dae = d / "cubes.dae"
    dae.write_text(_cube_dae(), encoding="utf-8")
    (d / "probe.omniworld").write_text(f"""#VRML_SIM R2025a utf8
WorldInfo {{ basicTimeStep 4 newtonSolver "mujoco" }}
Viewpoint {{ orientation 0 0 1 0 position -3 0 1 }}
Solid {{ translation 0 0 -0.5 boundingObject Box {{ size 10 10 1 }} }}
Solid {{
  translation 0 0 0.5
  name "MESHPROBE"
  boundingObject Mesh {{ url "{dae.as_posix()}" }}
  physics Physics {{ density -1 mass 1 }}
}}
""", encoding="utf-8")
    log, mjcf, traj = d / "engine.log", d / "model.mjcf.xml", d / "traj.tsv"
    env = {k: v for k, v in os.environ.items() if k != "OMNISIM_MESH_NODE_TRANSFORM_FIX"}
    env.update(OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log), OMNISIM_NEWTON_SAVE_MJCF=str(mjcf),
               OMNISIM_PROBE_TRAJ=str(traj), OMNISIM_PROBE_TRAJ_MS="200", **(extra_env or {}))
    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering", "--minimize",
                        str(d / "probe.omniworld")], env=env, timeout=240, capture_output=True)
    except subprocess.TimeoutExpired:
        pass
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    for sig in _BRINGUP_SIGNATURES:
        if sig in text:
            pytest.skip("Newton did not come up (%r); the run produced no data" % sig)
    assert mjcf.is_file(), "the solver model was never written; log tail:\n" + text[-1500:]
    model = mjcf.read_text(encoding="utf-8")
    meshes = re.findall(r'<mesh name="([^"]+)" vertex="([^"]*)"', model)
    assert len(meshes) == 1, "expected exactly one mesh collider in the solver model, got %d" % len(meshes)
    name, verts = meshes[0]
    flat = [float(c) for c in verts.split()]
    geom = re.search(r'<geom name="%s"([^>]*)/>' % re.escape(name), model).group(1)
    pos_m = re.search(r'\bpos="([^"]+)"', geom)
    quat_m = re.search(r'\bquat="([^"]+)"', geom)
    pos = tuple(float(c) for c in pos_m.group(1).split()) if pos_m else (0.0, 0.0, 0.0)
    quat = tuple(float(c) for c in quat_m.group(1).split()) if quat_m else (1.0, 0.0, 0.0, 0.0)
    pts = [tuple(a + b for a, b in zip(_quat_rotate(quat, flat[i:i + 3]), pos)) for i in range(0, len(flat), 3)]
    lo = tuple(min(p[k] for p in pts) for k in range(3))
    hi = tuple(max(p[k] for p in pts) for k in range(3))
    return lo, hi


def _close(box, ref):
    return all(abs(a - b) < TOL for got, want in zip(box, ref) for a, b in zip(got, want))


@needs_engine
def test_instanced_collada_collider_reaches_the_solver_where_it_was_authored(tmp_path):
    box = _mesh_box_in_body_frame(tmp_path, "fixed")
    assert _close(box, FIXED_BOX), (
        "the Collada collider is not where the file puts it: body-frame box %s..%s, expected %s..%s "
        "(leaf-first composition gives %s..%s)" % (*box, *FIXED_BOX, *LEAF_FIRST_BOX))


@needs_engine
def test_hatch_restores_the_leaf_first_product(tmp_path):
    box = _mesh_box_in_body_frame(tmp_path, "hatch", {"OMNISIM_MESH_NODE_TRANSFORM_FIX": "0"})
    assert _close(box, LEAF_FIRST_BOX), (
        "OMNISIM_MESH_NODE_TRANSFORM_FIX=0 must restore the pre-2026-10-15 placement %s..%s, got %s..%s"
        % (*LEAF_FIRST_BOX, *box))
