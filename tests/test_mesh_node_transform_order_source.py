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

"""Engine-free pin: both Assimp node walks compose a node's transform ROOT-FIRST.

OmMesh (every Mesh geometry, so every mesh boundingObject the solver sees) and
OmCadShape (CadShape visuals) composed `node * parent * ... * root` until
2026-10-15, which misplaces Collada geometry instanced under a translated node
(Rover Robotics' MEGA wheel 26.5 mm off its axle, its payload up to 0.96 m).
Both now call omAssimpNodeWorldTransform() in OmAssimpIoSystem.cpp, which
pre-multiplies each parent; OMNISIM_MESH_NODE_TRANSFORM_FIX=0 restores the old
product. The behavioural pin, which launches the engine and reads the collider
back from the solver model, is tests/test_mesh_collada_node_transform_order.py.

    python -m pytest tests/test_mesh_node_transform_order_source.py -v
"""

from __future__ import annotations

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def test_both_assimp_node_walks_compose_root_first():
    """Engine-free: OmMesh and OmCadShape take the shared helper, and it is root-first."""
    nodes = REPO / "src" / "omnisim" / "nodes"
    helper = (nodes / "OmAssimpIoSystem.cpp").read_text(encoding="utf-8")
    assert "transform = current->mTransformation * transform" in helper, (
        "omAssimpNodeWorldTransform must pre-multiply each parent (root * ... * parent * node)")
    assert "OMNISIM_MESH_NODE_TRANSFORM_FIX" in helper
    for name in ("OmMesh.cpp", "OmCadShape.cpp"):
        src = (nodes / name).read_text(encoding="utf-8")
        assert "omAssimpNodeWorldTransform(node)" in src, name + " must use the shared root-first helper"
        assert "transform *= current->mTransformation" not in src, (
            name + " composes node transforms leaf-first again")
