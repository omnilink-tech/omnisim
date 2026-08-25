# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0

"""Contracts for the conservative default agent world-sync path.

The important property is one-sided: an edit may unnecessarily reload, but it
must never be live-applied unless the running scene will still match the file.
"""

from __future__ import annotations

from pathlib import Path
import importlib.util
import sys
import types

import pytest

from scripts.harness.omnisim_harness import (
    HarnessState, plan_world_sync, write_sibling_world,
)


SOLID = """#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 8 }
DEF FLOOR Solid {
  translation 0 0 -0.05
  children [ Shape { geometry Box { size 4 4 0.1 } } ]
  boundingObject Box { size 4 4 0.1 }
}
DEF BOX Solid {
  translation 0 0 1
  rotation 0 0 1 0
  children [ Shape { geometry Box { size 0.4 0.4 0.4 } } ]
  boundingObject Box { size 0.4 0.4 0.4 }
  physics Physics { density -1 mass 1 }
}
"""

ROBOT = """#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 16 }
DEF ROVER Robot {
  translation 1 2 0.3
  rotation 0 0 1 0
  name "rover"
  controller "drive"
  children [
    DEF SENSOR_BODY Solid {
      translation 0.1 0 0.2
      children [ Shape { geometry Box { size 0.1 0.1 0.1 } } ]
    }
  ]
}
"""


def test_solid_translation_and_rotation_use_live_pose():
    edited = SOLID.replace("translation 0 0 1\n", "translation 2 -1 1.5\n")
    edited = edited.replace("rotation 0 0 1 0\n", "rotation 0 0 1 1.25\n")
    plan = plan_world_sync(SOLID, edited)
    assert plan["mode"] == "live_pose"
    assert plan["changes"] == [{
        "def": "BOX",
        "before": {"translation": [0.0, 0.0, 1.0],
                   "rotation": [0.0, 0.0, 1.0, 0.0]},
        "translation": [2.0, -1.0, 1.5],
        "rotation": [0.0, 0.0, 1.0, 1.25],
    }]


def test_multiple_root_nodes_are_one_live_batch():
    edited = SOLID.replace("translation 0 0 -0.05", "translation 0 0 -0.1")
    edited = edited.replace("translation 0 0 1\n", "translation 0 0 2\n")
    plan = plan_world_sync(SOLID, edited)
    assert plan["mode"] == "live_pose"
    assert [c["def"] for c in plan["changes"]] == ["FLOOR", "BOX"]


def test_root_robot_pose_is_live_but_nested_body_pose_is_not():
    root_edit = ROBOT.replace("translation 1 2 0.3", "translation 4 5 0.3")
    plan = plan_world_sync(ROBOT, root_edit)
    assert plan["mode"] == "live_pose"
    assert plan["changes"][0]["def"] == "ROVER"

    nested_edit = ROBOT.replace("translation 0.1 0 0.2", "translation 0.2 0 0.2")
    plan = plan_world_sync(ROBOT, nested_edit)
    assert plan["mode"] == "full_reload"


@pytest.mark.parametrize("edited", [
    SOLID.replace("size 0.4 0.4 0.4", "size 0.5 0.4 0.4", 1),
    SOLID.replace("mass 1", "mass 2"),
    SOLID.replace("basicTimeStep 8", "basicTimeStep 4"),
    SOLID + 'DEF EXTRA Solid { translation 0 0 1 }\n',
    SOLID.replace("DEF BOX Solid", "DEF RENAMED Solid"),
    SOLID.replace("translation 0 0 1", "translation IS pose"),
    SOLID[:-3],
])
def test_every_non_pose_or_ambiguous_edit_falls_back(edited):
    assert plan_world_sync(SOLID, edited)["mode"] == "full_reload"


def test_robot_controller_edit_falls_back():
    edited = ROBOT.replace('controller "drive"', 'controller "other"')
    assert plan_world_sync(ROBOT, edited)["mode"] == "full_reload"


def test_comments_whitespace_and_numeric_spelling_need_no_runtime_action():
    edited = SOLID.replace("0 0 1\n", "0.0  0e0  1.000\n", 1)
    edited = "# an agent comment\n" + edited.replace("WorldInfo", "\nWorldInfo")
    assert plan_world_sync(SOLID, edited)["mode"] == "no_change"


def test_injected_sibling_uses_the_exact_captured_source(tmp_path):
    world = tmp_path / "race.wbt"
    world.write_text("newer content the engine must not see", encoding="utf-8")
    sibling = write_sibling_world(world, source_text=SOLID)
    text = sibling.read_text(encoding="utf-8")
    assert text.startswith(SOLID)
    assert "newer content" not in text


class _LiveClient:
    def is_connected(self):
        return True


class _LiveProc:
    def poll(self):
        return None


def _state_for(tmp_path: Path, monkeypatch, source: str) -> tuple[HarnessState, Path]:
    monkeypatch.setattr(
        "scripts.harness.omnisim_harness.resolve_omnisim_binary",
        lambda home: tmp_path / "omnisim-bin")
    world = tmp_path / "scene.wbt"
    world.write_text(source, encoding="utf-8")
    state = HarnessState(tmp_path)
    state.current_world = str(world.resolve())
    state.current_source_text = source
    state.last_load_ok = True
    state.proc = _LiveProc()
    state.supervisor = _LiveClient()
    return state, world


def test_sync_world_sends_one_batch_and_updates_snapshot(tmp_path, monkeypatch):
    state, world = _state_for(tmp_path, monkeypatch, SOLID)
    edited = SOLID.replace("translation 0 0 1\n", "translation 0 0 1.8\n")
    world.write_text(edited, encoding="utf-8")
    calls = []

    def fake_call(cmd, args):
        calls.append((cmd, args))
        return {"changes": [{"def": "BOX", "position": [0, 0, 0.2]}],
                "verification": {"applied": 1}, "sim_time_ms": 120.0}

    monkeypatch.setattr(state, "supervisor_call", fake_call)
    result = state.sync_world(str(world), 3, settle_steps=17)
    assert result["ok"] is True
    assert result["mode"] == "live_pose"
    assert result["settle_steps"] == 17
    assert calls == [("scene_set_poses", {
        "changes": [{"def": "BOX", "translation": [0.0, 0.0, 1.8]}],
        "reset_physics": True,
        "settle_steps": 17,
    })]
    assert result["changes"][0]["authored_before"] == {
        "translation": [0.0, 0.0, 1.0]}
    assert state.current_source_text == edited


def test_sync_world_automatically_reloads_geometry_edits(tmp_path, monkeypatch):
    state, world = _state_for(tmp_path, monkeypatch, SOLID)
    edited = SOLID.replace("size 0.4 0.4 0.4", "size 0.6 0.4 0.4", 1)
    world.write_text(edited, encoding="utf-8")
    loads = []

    def fake_load(path, wait_s, with_supervisor, light, source_text=None):
        loads.append((path, wait_s, with_supervisor, light, source_text))
        return {"ok": True, "load_state": "complete", "load_ms": 42,
                "diagnostics": []}

    monkeypatch.setattr(state, "_load_world_locked", fake_load)
    result = state.sync_world(str(world), 3)
    assert result["mode"] == "full_reload"
    assert result["fallback"] is True
    assert len(loads) == 1
    assert loads[0][2] is True
    assert loads[0][4] == edited
    assert state.current_source_text == edited


def test_sync_world_initial_call_is_an_ordinary_supervised_load(tmp_path, monkeypatch):
    monkeypatch.setattr(
        "scripts.harness.omnisim_harness.resolve_omnisim_binary",
        lambda home: tmp_path / "omnisim-bin")
    world = tmp_path / "initial.wbt"
    world.write_text(SOLID, encoding="utf-8")
    state = HarnessState(tmp_path)
    loads = []

    def fake_load(path, wait_s, with_supervisor, light, source_text=None):
        loads.append((path, with_supervisor, light, source_text))
        state.current_world = str(path)
        return {"ok": True, "load_state": "complete", "load_ms": 5,
                "diagnostics": []}

    monkeypatch.setattr(state, "_load_world_locked", fake_load)
    result = state.sync_world(str(world), 3, light=True)
    assert result["mode"] == "full_reload"
    assert result["reason"].startswith("different world")
    assert loads == [(world.resolve(), True, True, SOLID)]
    assert state.current_source_text == SOLID


def _supervisor_module(monkeypatch):
    """Import the controller module under stock Python with one tiny API stub."""
    controller_dir = (Path(__file__).resolve().parents[2] / "projects" / "default"
                      / "controllers" / "harness_supervisor")
    monkeypatch.syspath_prepend(str(controller_dir))
    api = types.ModuleType("omnisim")
    api.Supervisor = object
    monkeypatch.setitem(sys.modules, "omnisim", api)
    spec = importlib.util.spec_from_file_location(
        "world_sync_supervisor_under_test", controller_dir / "harness_supervisor.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _Field:
    def __init__(self, value):
        self.value = list(value)

    def getSFVec3f(self):
        return list(self.value)

    def setSFVec3f(self, value):
        self.value = list(value)

    def getSFRotation(self):
        return list(self.value)

    def setSFRotation(self, value):
        self.value = list(value)


class _Node:
    def __init__(self, translation, fail_reset=False):
        self.fields = {"translation": _Field(translation),
                       "rotation": _Field([0, 0, 1, 0])}
        self.fail_reset = fail_reset
        self.reset_count = 0

    def getField(self, name):
        return self.fields.get(name)

    def getPosition(self):
        return self.fields["translation"].value

    def getTypeName(self):
        return "Robot"

    def resetPhysics(self):
        self.reset_count += 1
        if self.fail_reset:
            raise RuntimeError("reset failed")


class _Supervisor:
    def __init__(self, nodes):
        self.nodes = nodes
        self.step_count = 0

    def getFromDef(self, name):
        return self.nodes.get(name)

    def step(self, _basic_step):
        self.step_count += 1
        return 0


def test_scene_set_poses_validates_batch_and_settles_once(monkeypatch):
    module = _supervisor_module(monkeypatch)
    a = _Node([0, 0, 1])
    b = _Node([1, 0, 1])
    supervisor = _Supervisor({"A": a, "B": b})
    result = module.dispatch(supervisor, 8, 0, "scene_set_poses", {
        "changes": [{"def": "A", "translation": [0, 0, 2]},
                    {"def": "B", "translation": [1, 0, 3],
                     "rotation": [0, 0, 1, 0.5]}],
        "settle_steps": 7,
    })
    assert supervisor.step_count == 7  # not 7 per node
    assert a.fields["translation"].value == [0, 0, 2]
    assert b.fields["rotation"].value == [0, 0, 1, 0.5]
    assert result["verification"]["validated_before_mutation"] is True
    assert result["verification"]["applied"] == 2


def test_scene_set_poses_bad_later_def_does_not_mutate_first(monkeypatch):
    module = _supervisor_module(monkeypatch)
    a = _Node([0, 0, 1])
    supervisor = _Supervisor({"A": a})
    with pytest.raises(module.CommandError):
        module.dispatch(supervisor, 8, 0, "scene_set_poses", {
            "changes": [{"def": "A", "translation": [0, 0, 2]},
                        {"def": "MISSING", "translation": [1, 0, 3]}],
        })
    assert a.fields["translation"].value == [0, 0, 1]
    assert a.reset_count == 0


def test_scene_set_poses_rolls_back_when_a_reset_fails(monkeypatch):
    module = _supervisor_module(monkeypatch)
    a = _Node([0, 0, 1])
    b = _Node([1, 0, 1], fail_reset=True)
    supervisor = _Supervisor({"A": a, "B": b})
    with pytest.raises(module.CommandError, match="rollback"):
        module.dispatch(supervisor, 8, 0, "scene_set_poses", {
            "changes": [{"def": "A", "translation": [0, 0, 2]},
                        {"def": "B", "translation": [1, 0, 3]}],
        })
    assert a.fields["translation"].value == [0, 0, 1]
    assert b.fields["translation"].value == [1, 0, 1]
