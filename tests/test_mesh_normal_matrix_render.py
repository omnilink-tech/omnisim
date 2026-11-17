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

"""A mesh on a translated Collada node must shade like the same mesh on an untranslated one.

The defect (noted 2026-10-15 with the node-transform-order fix, fixed 2026-10-23):
OmMesh and OmCadShape mapped NORMALS with the node's full 4x4 transform, so the
node's translation was added to every normal. A Collada node survives
aiProcess_OptimizeGraph when its geometry is instanced more than once, so any
such instance under a translated node shaded with normals bent toward its offset.

The probe photographs two meshes at the same spot, one after the other, through
a Camera device in OMNISIM_NO_WINDOW mode (no window, wgpu offscreen):

Both node walks are rendered: as a `Mesh` geometry (OmMesh) and as a `CadShape`
(OmCadShape). The CadShape arm also pins a second defect fixed the same day:
OmCadShape never queued a node's children, so it read only the root node's meshes
and a file aiProcess_OptimizeGraph cannot collapse onto its root -- any geometry
instanced by two nodes, translated or not -- rendered nothing (frame B was pure
background, pixel range 5).

- A: one sphere geometry on a single, untranslated node;
- B: the same geometry instanced by two nodes 20 m and 40 m up, its Pose
  bringing the first instance back to A's spot.

Same surface, light and camera, so the frames must match; with the hatch
OMNISIM_MESH_NORMAL_MATRIX_FIX=0 every normal of B points up the 20 m offset and
the sphere renders as one flat disc (max pixel difference 162, against 0 with
the fix). The engine-free half (the math, compiled from the header) is
tests/test_mesh_normal_matrix.py.

    python -m pytest tests/test_mesh_normal_matrix_render.py -v
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import tempfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
RADIUS = 0.4
B_NODES = ((0.0, 0.0, 20.0), (0.0, 0.0, 40.0))


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


def _sphere_dae(nodes) -> str:
    nu, nv = 48, 24
    pos, idx = [], []
    for j in range(nv + 1):
        th = math.pi * j / nv
        for i in range(nu + 1):
            ph = 2 * math.pi * i / nu
            pos.append((math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th)))
    for j in range(nv):
        for i in range(nu):
            a = j * (nu + 1) + i
            b = a + nu + 1
            idx += [a, b, a + 1, a + 1, b, b + 1]
    n = len(pos)
    p = " ".join(f"{RADIUS * c:.6f}" for v in pos for c in v)
    nrm = " ".join(f"{c:.6f}" for v in pos for c in v)
    inst = "".join(f'<node id="n{k}" name="n{k}"><matrix>1 0 0 {x:g} 0 1 0 {y:g} 0 0 1 {z:g} 0 0 0 1</matrix>'
                   f'<instance_geometry url="#ball"/></node>' for k, (x, y, z) in enumerate(nodes))
    acc = '<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
    return f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries><geometry id="ball" name="ball"><mesh>
    <source id="p"><float_array id="pa" count="{3 * n}">{p}</float_array>
      <technique_common><accessor source="#pa" count="{n}" stride="3">{acc}</accessor></technique_common></source>
    <source id="nm"><float_array id="na" count="{3 * n}">{nrm}</float_array>
      <technique_common><accessor source="#na" count="{n}" stride="3">{acc}</accessor></technique_common></source>
    <vertices id="v"><input semantic="POSITION" source="#p"/><input semantic="NORMAL" source="#nm"/></vertices>
    <triangles count="{len(idx) // 3}"><input semantic="VERTEX" source="#v" offset="0"/><p>{" ".join(map(str, idx))}</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="s"><node id="top" name="top">{inst}</node></visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#s"/></scene>
</COLLADA>
"""


def _visual(dae, node):
    if node == "CadShape":  # its material comes from the file: none here, so the importer's default
        return 'CadShape { url "%s" }' % dae.as_posix()
    return ('Shape { appearance PBRAppearance { baseColor 0.8 0.8 0.8 roughness 0.7 metalness 0 } '
            'geometry Mesh { url "%s" } }' % dae.as_posix())


def _render(tmp_path, tag, node, extra_env=None):
    d = tmp_path / tag
    d.mkdir()
    (d / "a.dae").write_text(_sphere_dae([(0.0, 0.0, 0.0)]), encoding="utf-8")
    (d / "b.dae").write_text(_sphere_dae(B_NODES), encoding="utf-8")
    result = d / "result.json"
    # OMNISIM_NORMAL_PROBE_FRAMES=<dir>: also save both camera frames there as PPM, one subfolder per run
    frames = Path(os.environ["OMNISIM_NORMAL_PROBE_FRAMES"]) / tag if os.environ.get("OMNISIM_NORMAL_PROBE_FRAMES") else None
    args = json.dumps(result.as_posix()) + ("" if frames is None else " " + json.dumps(frames.as_posix()))
    bz = 1.0 - B_NODES[0][2]
    # tests/rendering is the project that holds the controller; dot-prefixed worlds there are gitignored
    fd, name = tempfile.mkstemp(prefix=".mesh_normal_", suffix=".omniworld", dir=REPO / "tests/rendering/worlds")
    os.close(fd)
    placed = Path(name)
    placed.write_text(f"""#VRML_SIM R2025a utf8
WorldInfo {{ basicTimeStep 16 }}
Viewpoint {{ orientation 0 0 1 0 position -4 0 1 }}
Background {{ skyColor [ 0.05 0.05 0.06 ] }}
DirectionalLight {{ direction 1 0.6 -0.8 intensity 2 castShadows FALSE }}
Robot {{
  translation -4 0 1
  supervisor TRUE
  controller "mesh_normal_probe"
  controllerArgs [ {args} ]
  children [ Camera {{ name "camera" width 96 height 96 fieldOfView 0.5 }} ]
}}
DEF MESH_A Pose {{
  translation 0 50 1
  children [ {_visual(d / 'a.dae', node)} ]
}}
DEF MESH_B Pose {{
  translation 0 0 {bz:g}
  children [ {_visual(d / 'b.dae', node)} ]
}}
""", encoding="utf-8")
    env = {k: v for k, v in os.environ.items() if k != "OMNISIM_MESH_NORMAL_MATRIX_FIX"}
    env.update(OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(d / "engine.log"), OMNISIM_NO_WINDOW="1",
               OMNISIM_WGPU_TAA="0", OMNISIM_WGPU_SENSOR_DRAW_CACHE="0", **(extra_env or {}))
    if os.name == "nt":
        runtime = REPO / "msys64/mingw64/bin/newton-runtime"
        env["PATH"] = str(runtime) + os.pathsep + str(runtime.parent) + os.pathsep + env.get("PATH", "")
    try:
        return _run(placed, d, env, result)
    finally:
        placed.unlink(missing_ok=True)


def _run(placed, d, env, result):
    proc = subprocess.Popen([str(_binary()), str(placed), "--batch", "--mode=fast", "--stdout", "--stderr"],
                            cwd=REPO, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        stdout, _ = proc.communicate(timeout=120)
    except subprocess.TimeoutExpired:
        if os.name == "nt":
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True, check=False)
        else:
            proc.kill()
        stdout, _ = proc.communicate(timeout=10)
        pytest.fail("normal probe timed out: " + stdout.decode(errors="replace")[-3000:])
    assert result.is_file(), stdout.decode(errors="replace")[-3000:]
    data = json.loads(result.read_text(encoding="utf-8"))
    assert "error" not in data, data
    # a sphere was photographed in both frames, not an empty background
    assert data["frame_a_range"] > 40 and data["frame_b_range"] > 20, data
    return data


@needs_engine
@pytest.mark.parametrize("node", ["Mesh", "CadShape"])
def test_translated_node_shades_like_an_untranslated_one(tmp_path, node):
    data = _render(tmp_path, "fixed_" + node, node)
    # measured 2026-10-23: max 0 for Mesh, 1 for CadShape (it stores float vertices, Mesh doubles)
    assert data["max_abs_diff"] <= 2, (
        "the sphere on a node translated 20 m shades differently from the same sphere on an untranslated "
        "node: %s" % data)


@needs_engine
@pytest.mark.parametrize("node", ["Mesh", "CadShape"])
def test_hatch_restores_the_translated_normals(tmp_path, node):
    data = _render(tmp_path, "hatch_" + node, node, {"OMNISIM_MESH_NORMAL_MATRIX_FIX": "0"})
    # measured 2026-10-23: B renders as a flat disc, max 162 (Mesh) / 149 (CadShape), mean ~5 over a frame
    # that is mostly background
    assert data["max_abs_diff"] > 60, (
        "OMNISIM_MESH_NORMAL_MATRIX_FIX=0 must restore the pre-2026-10-23 normals (translation added), "
        "which shade the translated sphere visibly differently: %s" % data)
