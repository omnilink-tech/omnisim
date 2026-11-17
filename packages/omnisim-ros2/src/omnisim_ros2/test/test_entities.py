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

"""Unit tests for entity selection and mapping. No ROS and no simulator required.

The scene-tree fixture is a trimmed copy of a real
``GET /scene/tree`` body captured from ``omnilink_husky.omniworld`` on
2026-08-17, so the shapes here are measured rather than imagined -- including
the empty ``harness_injected`` list, which is exactly the harness behaviour the
selection code has to work around.
"""

import pytest

from omnisim_ros2.entities import (
    CATEGORY_DYNAMIC_OBJECT,
    CATEGORY_OBJECT,
    CATEGORY_ROBOT,
    CATEGORY_STATIC_OBJECT,
    Entity,
    category_for,
    compose_spawn_body,
    entities_from_scene_tree,
    has_pose,
    matches_filter,
    spawn_format_for,
)

IDENT = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

SCENE_TREE = {
    # Measured: the harness reports an EMPTY injected list even though node
    # #185 is its own supervisor. Selection must not rely on this.
    "harness_injected": [],
    "bounds_included": False,
    "nodes": [
        {"type": "Group", "id": 0, "position": [None, None, None],
         "orientation": [None] * 9, "parent_def": None},
        {"type": "WorldInfo", "id": 2, "position": [None, None, None],
         "orientation": [None] * 9, "parent_def": None},
        {"type": "Viewpoint", "id": 3, "position": [None, None, None],
         "orientation": [None] * 9, "parent_def": None},
        {"type": "OmniSimSky", "id": 4, "position": [None, None, None],
         "orientation": [None] * 9, "parent_def": None},
        {"type": "OmniSimSun", "def": "SUN", "id": 5, "position": [None, None, None],
         "orientation": [None] * 9, "parent_def": None},
        {"type": "OmniSimSunMarker", "def": "SUN_MARKER", "id": 6,
         "position": [0.0, 0.0, 100000.0], "orientation": IDENT, "parent_def": None},
        {"type": "OmniLinkStage", "id": 10, "position": [0.0, 0.0, 0.0],
         "orientation": IDENT, "parent_def": None},
        {"type": "Robot", "def": "HUSKY", "id": 103, "position": [0.0, 0.0, 0.132],
         "orientation": IDENT, "parent_def": None},
        # A child link: root_only must drop it.
        {"type": "Solid", "def": "WHEEL_FL", "id": 120, "position": [0.2, 0.2, 0.05],
         "orientation": IDENT, "parent_def": "HUSKY"},
        {"type": "Robot", "id": 185, "position": [0.0, 0.0, 0.0],
         "orientation": IDENT, "parent_def": None},
    ],
}


def test_poseless_nodes_are_not_entities():
    """WorldInfo/Viewpoint/Group/sky/lights have no pose and are not entities."""
    names = [e.name for e in entities_from_scene_tree(SCENE_TREE)]
    for excluded in ("#0", "#2", "#3", "#4", "SUN"):
        assert excluded not in names


def test_child_links_are_not_entities():
    names = [e.name for e in entities_from_scene_tree(SCENE_TREE)]
    assert "WHEEL_FL" not in names


def test_selects_posed_root_nodes():
    names = [e.name for e in entities_from_scene_tree(SCENE_TREE)]
    assert names == ["SUN_MARKER", "#10", "HUSKY", "#185"]


def test_exclude_defs_removes_harness_supervisor():
    """The supervisor's DEF comes from /robots, not from the tree's own list."""
    names = [e.name for e in entities_from_scene_tree(SCENE_TREE, exclude_defs={"#185"})]
    assert names == ["SUN_MARKER", "#10", "HUSKY"]


def test_defless_nodes_are_not_addressable():
    by_name = {e.name: e for e in entities_from_scene_tree(SCENE_TREE)}
    assert by_name["#10"].addressable is False
    assert by_name["HUSKY"].addressable is True


def test_root_only_false_includes_children():
    names = [e.name for e in entities_from_scene_tree(SCENE_TREE, root_only=False)]
    assert "WHEEL_FL" in names


def test_has_pose():
    assert has_pose({"position": [1, 2, 3]})
    assert not has_pose({"position": [None, None, None]})
    assert not has_pose({"position": None})
    assert not has_pose({})


def test_tags_report_type_and_robot():
    e = Entity(name="HUSKY", node_type="Robot", position=[0, 0, 0], orientation=IDENT)
    assert e.tags == ["type:Robot", "robot"]
    s = Entity(name="BOX", node_type="Solid", position=[0, 0, 0], orientation=IDENT)
    assert s.tags == ["type:Solid"]


def test_category_from_physics_presence():
    assert category_for("Robot", None) == CATEGORY_ROBOT
    assert category_for("URDFRobot", None) == CATEGORY_ROBOT
    assert category_for("Solid", None) == CATEGORY_OBJECT
    dynamic = {"fields": {"physics": {"field_exists": True, "present": True}}}
    assert category_for("Solid", dynamic) == CATEGORY_DYNAMIC_OBJECT
    static = {"fields": {"physics": {"field_exists": True, "present": False}}}
    assert category_for("Solid", static) == CATEGORY_STATIC_OBJECT


@pytest.mark.parametrize(
    "name,pattern,expected",
    [
        ("HUSKY", "", True),
        ("HUSKY", "HUS", True),
        ("HUSKY", "^HUSKY$", True),
        ("HUSKY", "^BOX$", False),
        ("BOX_1", "BOX_[0-9]+", True),
        ("HUSKY", "[unclosed", False),   # a bad regex must not raise
    ],
)
def test_matches_filter(name, pattern, expected):
    assert matches_filter(name, pattern) is expected


def test_spawn_format_detection():
    assert spawn_format_for("", "Solid { }") == "vrml"
    assert spawn_format_for("file:///a/b.urdf", "") == "urdf"
    assert spawn_format_for("/a/b.URDF", "") == "urdf"
    assert spawn_format_for("/a/b.sdf", "") == ""
    assert spawn_format_for("/a/b.usd", "") == ""
    assert spawn_format_for("", "") == ""


def test_compose_spawn_body_uses_vrml_for_resource_string():
    body = compose_spawn_body("BOX", "", "Solid { }", (1.0, 2.0, 3.0), [0, 0, 1, 0.5])
    assert body["vrml"] == "Solid { }"
    assert body["def"] == "BOX"
    assert body["translation"] == [1.0, 2.0, 3.0]
    assert body["rotation"] == [0.0, 0.0, 1.0, 0.5]
    assert "urdf" not in body


def test_compose_spawn_body_strips_file_scheme_for_urdf():
    body = compose_spawn_body("R", "file:///robots/a.urdf", "", (0, 0, 0), [0, 0, 1, 0])
    assert body["urdf"] == "/robots/a.urdf"
    assert "vrml" not in body


def test_resource_string_wins_over_uri():
    """Resource.msg: 'If uri field is not empty, resource_string field will be
    ignored' -- but our caller only reaches here having chosen one, and a
    resource_string must never be silently dropped in favour of a uri."""
    body = compose_spawn_body("X", "/a/b.urdf", "Solid { }", (0, 0, 0), [0, 0, 1, 0])
    assert body["vrml"] == "Solid { }"
