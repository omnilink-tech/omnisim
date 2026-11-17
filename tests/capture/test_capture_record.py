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

"""The capture supervisor's sim-time recorder (record_start / record_stop).

A screenshot loop driven from outside samples WALL time: below real time the
frames are sparse and uneven in sim time, so footage is choppy at 1x. The
recorder saves one frame every `period_ms` of SIMULATION time from inside the
step loop. Measured 2026-09-25 on a 1080p drone world: 515 frames, exactly
40 ms apart, 20.6 s of flight that plays back in real time at 25 fps; the
camera rendering at the frame period instead of every 8 ms step also took the
world from 0.06x to 0.14x real time.

The supervisor imports the engine's controller library, so this executes only
the recorder functions, from source, against fakes.
"""
from __future__ import annotations

import ast
import json
import math
import os
import pathlib

import pytest

SRC = (pathlib.Path(__file__).resolve().parents[2]
       / "projects" / "default" / "controllers" / "capture_supervisor" / "capture_supervisor.py")
FUNCS = {"cmd_record_start", "record_tick", "cmd_record_stop",
         "_parse_follow", "follow_tick", "cmd_set_camera_pose", "axis_angle_to_target",
         "cmd_record_follow"}


class CommandError(Exception):
    pass


def _load():
    tree = ast.parse(SRC.read_text(encoding="utf-8"))
    body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in FUNCS]
    assert {n.name for n in body} == FUNCS
    ns = {"json": json, "math": math, "os": os, "CommandError": CommandError, "CaptureState": object}
    exec(compile(ast.Module(body=body, type_ignores=[]), str(SRC), "exec"), ns)
    return ns


class Camera:
    def __init__(self):
        self.periods, self.saved = [], []

    def enable(self, period):
        self.periods.append(period)

    def saveImage(self, path, quality):
        pathlib.Path(path).write_bytes(b"png")
        self.saved.append(path)
        return 0


class RobotNode:
    def getPosition(self):
        return [1.0, 2.0, 0.5]

    def getOrientation(self):
        return [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]   # yaw = +90 deg


class Supervisor:
    def getFromDef(self, name):
        return RobotNode() if name == "X500" else None


class State:
    def __init__(self):
        self.camera, self.camera_reason = Camera(), None
        self.basic_step_ms, self.supervisor, self.rec = 8, Supervisor(), None


def test_frames_land_on_the_sim_time_grid_whatever_the_host_speed(tmp_path):
    ns, st = _load(), State()
    out = ns["cmd_record_start"](st, 1000.0, {"dir": str(tmp_path), "period_ms": 40,
                                              "track": ["X500", "MISSING"]})
    assert out["tracked"] == ["X500"] and out["fps"] == 25.0
    assert st.camera.periods == [40]                    # render at the frame period
    for step in range(1, 126):                          # 1 s of 8 ms steps
        ns["record_tick"](st, 1000.0 + 8 * step)
    stop = ns["cmd_record_stop"](st, {})
    assert stop["frames"] == 25
    assert st.camera.periods == [40, 8]                 # restored on stop
    rows = [json.loads(l) for l in (tmp_path / "index.jsonl").read_text().splitlines()]
    times = [r["sim_time_ms"] for r in rows]
    assert times[0] == 1040.0 and all(b - a == 40.0 for a, b in zip(times, times[1:]))
    assert rows[0]["file"] == "000000.png" and (tmp_path / "000024.png").exists()
    pose = rows[0]["poses"]["X500"]
    assert (pose["x"], pose["y"], pose["z"]) == (1.0, 2.0, 0.5)
    assert pose["yaw"] == pytest.approx(math.pi / 2)


def test_period_must_be_a_multiple_of_the_basic_step(tmp_path):
    ns = _load()
    with pytest.raises(CommandError):
        ns["cmd_record_start"](State(), 0.0, {"dir": str(tmp_path), "period_ms": 42})


def test_one_recording_at_a_time_and_stop_needs_one(tmp_path):
    ns, st = _load(), State()
    ns["cmd_record_start"](st, 0.0, {"dir": str(tmp_path)})
    with pytest.raises(CommandError):
        ns["cmd_record_start"](st, 0.0, {"dir": str(tmp_path)})
    ns["cmd_record_stop"](st, {})
    with pytest.raises(CommandError):
        ns["cmd_record_stop"](st, {})


class Field:
    def __init__(self):
        self.values = []

    def setSFVec3f(self, v):
        self.values.append(("vec", list(v)))

    def setSFRotation(self, v):
        self.values.append(("rot", list(v)))


class SelfNode:
    def __init__(self):
        self.fields = {"translation": Field(), "rotation": Field()}

    def getField(self, name):
        return self.fields.get(name)


def test_follow_moves_the_camera_on_sim_time_with_a_world_frame_offset(tmp_path):
    # The recorder's optional follow: every basic step the camera eases
    # toward subject + offset (WORLD frame: it does not turn with the robot)
    # and looks at subject + look.
    ns, st = _load(), State()
    st.self_node, st.viewpoint = SelfNode(), None
    out = ns["cmd_record_start"](st, 0.0, {"dir": str(tmp_path), "period_ms": 40,
                                           "follow": {"def": "X500", "offset": [3.0, -4.0, 2.0],
                                                      "look": [0.0, 0.0, 0.25], "smooth_s": 0.0}})
    assert out["follow"] == {"def": "X500", "offset": [3.0, -4.0, 2.0],
                             "look": [0.0, 0.0, 0.25], "smooth_s": 0.0}
    ns["follow_tick"](st)
    kind, eye = st.self_node.fields["translation"].values[-1]
    assert eye == [4.0, -2.0, 2.5]                       # (1, 2, 0.5) + offset
    kind, rot = st.self_node.fields["rotation"].values[-1]
    # the camera's +X axis points at the subject + look
    ax, ay, az, ang = rot
    c, s_, t = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    fwd = [c + ax * ax * t, ay * ax * t + az * s_, az * ax * t - ay * s_]
    want = [1.0 - 4.0, 2.0 + 2.0, 0.75 - 2.5]
    n = math.sqrt(sum(v * v for v in want))
    assert fwd == pytest.approx([v / n for v in want], abs=1e-6)
    ns["cmd_record_stop"](st, {})


def test_follow_needs_a_known_def_and_an_offset(tmp_path):
    ns, st = _load(), State()
    with pytest.raises(CommandError):
        ns["cmd_record_start"](st, 0.0, {"dir": str(tmp_path), "follow": {"def": "NOPE", "offset": [1, 1, 1]}})
    with pytest.raises(CommandError):
        ns["cmd_record_start"](st, 0.0, {"dir": str(tmp_path), "follow": {"def": "X500"}})


def test_record_follow_eases_the_camera_to_a_new_offset_on_sim_time(tmp_path):
    ns, st = _load(), State()
    st.self_node, st.viewpoint = SelfNode(), None
    ns["cmd_record_start"](st, 0.0, {"dir": str(tmp_path), "follow": {
        "def": "X500", "offset": [3.0, -4.0, 2.0], "smooth_s": 0.0}})
    ns["cmd_record_follow"](st, {"offset": [-3.0, 4.0, 2.0], "move_s": 1.0})
    ns["follow_tick"](st)
    eye1 = st.self_node.fields["translation"].values[-1][1]
    assert 1.0 < eye1[0] < 4.0              # moving, not cut
    for _ in range(500):                    # 4 s of 8 ms steps
        ns["follow_tick"](st)
    eye = st.self_node.fields["translation"].values[-1][1]
    assert eye == pytest.approx([1.0 - 3.0, 2.0 + 4.0, 2.5], abs=1e-3)
    with pytest.raises(CommandError):
        ns["cmd_record_follow"](st, {"offset": [1, 2]})
