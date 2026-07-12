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

"""Unit tests for the observability layer.

Covers the pure-Python pieces — EventBus, LogRingBuffer, and
observe.detect_grips — that don't depend on a live OmniSim Supervisor.
End-to-end behavior against a real world is exercised by the harness
smoke lane; the unit tests here pin down ring-buffer semantics, cursor
edge cases, and grip heuristics so agents can rely on them.
"""

from __future__ import annotations

import io
import sys
import threading
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SUPERVISOR_DIR = REPO_ROOT / "projects" / "default" / "controllers" / "harness_supervisor"
HARNESS_DIR = REPO_ROOT / "scripts" / "harness"

# The supervisor module imports `from controller import Supervisor`,
# which is only available when running inside OmniSim. We import the
# pure-Python siblings directly by path so these tests run with stock
# `pytest` against the repo checkout.
sys.path.insert(0, str(SUPERVISOR_DIR))
sys.path.insert(0, str(HARNESS_DIR))


# ---------------------------------------------------------------------------
# EventBus
# ---------------------------------------------------------------------------


def test_eventbus_seq_is_monotonic_and_starts_at_one():
    from event_bus import EventBus
    bus = EventBus()
    bus.emit("contact.began", {"a_def": "A", "b_def": "B"}, t_sim_ms=0)
    bus.emit("contact.ended", {"a_def": "A", "b_def": "B"}, t_sim_ms=32)
    events = bus.since(0)
    assert [e["seq"] for e in events] == [1, 2]
    assert events[0]["type"] == "contact.began"
    assert events[1]["type"] == "contact.ended"
    assert events[0]["t_sim_ms"] == 0
    assert events[1]["t_sim_ms"] == 32


def test_eventbus_since_filters_out_already_seen():
    from event_bus import EventBus
    bus = EventBus()
    for i in range(5):
        bus.emit("contact.began", {"a_def": f"A{i}", "b_def": "B"}, t_sim_ms=i)
    after_three = bus.since(3)
    assert [e["seq"] for e in after_three] == [4, 5]


def test_eventbus_limit_clips_batch_size():
    from event_bus import EventBus
    bus = EventBus()
    for i in range(10):
        bus.emit("contact.began", {"a_def": f"A{i}", "b_def": "B"}, t_sim_ms=i)
    batch = bus.since(0, limit=3)
    assert len(batch) == 3
    assert [e["seq"] for e in batch] == [1, 2, 3]


def test_eventbus_types_filter_skips_other_kinds():
    from event_bus import EventBus
    bus = EventBus()
    bus.emit("contact.began", {"a_def": "A", "b_def": "B"})
    bus.emit("joint.limit_hit", {"joint": "j1", "side": "lower"})
    bus.emit("contact.ended", {"a_def": "A", "b_def": "B"})
    contact_only = bus.since(0, types=["contact.began", "contact.ended"])
    assert [e["type"] for e in contact_only] == ["contact.began", "contact.ended"]


def test_eventbus_drops_oldest_when_buffer_full():
    from event_bus import EventBus
    bus = EventBus(buffer_size=3)
    for i in range(5):
        bus.emit("contact.began", {"a_def": f"A{i}", "b_def": "B"}, t_sim_ms=i)
    # seqs 1 and 2 should be evicted; seqs 3..5 remain.
    seqs = [e["seq"] for e in bus.since(0)]
    assert seqs == [3, 4, 5]
    # `total` keeps growing even after drops so cursors stay monotonic.
    assert bus.total == 5
    assert bus.dropped == 2


def test_eventbus_reset_clears_state():
    from event_bus import EventBus
    bus = EventBus()
    bus.emit("contact.began", {"a_def": "A", "b_def": "B"})
    bus.reset()
    assert bus.since(0) == []
    assert bus.total == 0
    assert bus.dropped == 0


# ---------------------------------------------------------------------------
# LogRingBuffer (in scripts/harness/omnisim_harness.py)
# ---------------------------------------------------------------------------


def test_log_buffer_controller_log_round_trip():
    from omnisim_harness import LogRingBuffer
    buf = LogRingBuffer()
    buf.emit_controller_log("stdout", "hello world")
    buf.emit_controller_log("stderr", "uh oh")
    out = buf.since(0)
    assert len(out) == 2
    assert out[0]["type"] == "controller.log"
    assert out[0]["stream"] == "stdout"
    assert out[0]["line"] == "hello world"
    assert out[1]["stream"] == "stderr"
    assert out[1]["line"] == "uh oh"
    # both should have a wall-clock timestamp
    assert all(isinstance(e["t_wall"], float) for e in out)


def test_log_buffer_world_diagnostic_severity_to_type():
    from omnisim_harness import LogRingBuffer
    buf = LogRingBuffer()
    buf.emit_world_diagnostic({
        "severity": "error", "code": "X", "message": "boom", "raw": "raw"
    })
    buf.emit_world_diagnostic({
        "severity": "warning", "code": "Y", "message": "soft", "raw": "raw"
    })
    buf.emit_world_diagnostic({
        "severity": "info", "code": "Z", "message": "fyi", "raw": "raw"
    })
    out = buf.since(0)
    types = [e["type"] for e in out]
    assert types == ["world.error", "world.warning"]


def test_log_buffer_drops_count_on_overflow():
    from omnisim_harness import LogRingBuffer
    buf = LogRingBuffer(maxlen=3)
    for i in range(5):
        buf.emit_controller_log("stdout", f"line {i}")
    out = buf.since(0)
    assert len(out) == 3
    assert [e["line"] for e in out] == ["line 2", "line 3", "line 4"]
    assert buf.dropped == 2
    assert buf.total == 5


def test_log_buffer_types_filter():
    from omnisim_harness import LogRingBuffer
    buf = LogRingBuffer()
    buf.emit_controller_log("stdout", "a")
    buf.emit_world_diagnostic({"severity": "error", "code": "X",
                                "message": "m", "raw": "r"})
    only_log = buf.since(0, types=["controller.log"])
    assert len(only_log) == 1 and only_log[0]["type"] == "controller.log"
    only_err = buf.since(0, types=["world.error"])
    assert len(only_err) == 1 and only_err[0]["type"] == "world.error"


def test_pump_pipe_reads_all_lines_until_eof():
    from omnisim_harness import LogRingBuffer, _pump_pipe

    class FakePipe:
        """Minimal stand-in for subprocess.PIPE that supports
        readline() iteration. _pump_pipe uses iter(readline, b"") so we
        return b"" once the lines are exhausted to signal EOF.
        """
        def __init__(self, lines):
            self._lines = list(lines)

        def readline(self):
            if not self._lines:
                return b""
            return self._lines.pop(0)

        def close(self):
            pass

    buf = LogRingBuffer()
    pipe = FakePipe([b"first\n", b"second\n", b"third\r\n"])
    sink = io.StringIO()
    t = threading.Thread(target=_pump_pipe,
                         args=(pipe, "stdout", buf, sink),
                         daemon=True)
    t.start()
    t.join(timeout=1.0)
    assert not t.is_alive(), "pump_pipe did not exit on EOF"
    out = buf.since(0)
    assert [e["line"] for e in out] == ["first", "second", "third"]
    # Forwarded copy reaches the sink so an operator running the harness
    # in a terminal still sees the live output.
    assert sink.getvalue() == "first\nsecond\nthird\n"


# ---------------------------------------------------------------------------
# observe.detect_grips
# ---------------------------------------------------------------------------


def test_detect_grips_two_fingers_one_object():
    import observe
    pairs = [
        ("finger_l", "block"),
        ("finger_r", "block"),
        ("base", "ground"),
    ]
    index = {
        "finger_l": "robot",
        "finger_r": "robot",
        "base": "robot",
        # block + ground are not in any robot subtree
    }
    grips = observe.detect_grips(pairs, index)
    assert len(grips) == 1
    g = grips[0]
    assert g["gripper_def"] == "robot"
    assert g["held_def"] == "block"
    assert sorted(g["fingers"]) == ["finger_l", "finger_r"]


def test_detect_grips_single_finger_does_not_count():
    import observe
    pairs = [("finger_l", "block")]
    index = {"finger_l": "robot"}
    assert observe.detect_grips(pairs, index) == []


def test_detect_grips_two_fingers_different_robots_does_not_count():
    import observe
    pairs = [
        ("finger_a", "block"),
        ("finger_b", "block"),
    ]
    index = {"finger_a": "robotA", "finger_b": "robotB"}
    # Each robot has only one finger touching, so no grip is reported.
    assert observe.detect_grips(pairs, index) == []


def test_detect_grips_robot_to_robot_contact_ignored():
    """Two robots touching each other should NOT register as a grip.
    detect_grips skips contacts where both sides are inside a robot
    subtree — gripping is asymmetric (robot grasps non-robot object).
    """
    import observe
    pairs = [
        ("base_a", "base_b"),
        ("arm_a", "base_b"),
    ]
    index = {"base_a": "robotA", "arm_a": "robotA",
             "base_b": "robotB"}
    assert observe.detect_grips(pairs, index) == []


# ---------------------------------------------------------------------------
# joint_snapshot — pure helper around stub field reads
# ---------------------------------------------------------------------------


class _StubField:
    """Field stub: returns a configured value from getSFFloat /
    getSFString so we can exercise joint_snapshot without OmniSim.
    """
    def __init__(self, value):
        self._value = value

    def getSFFloat(self):
        return float(self._value)

    def getSFString(self):
        return str(self._value)

    def getSFBool(self):
        return bool(self._value)

    def getSFInt32(self):
        return int(self._value)

    def getSFNode(self):
        return self._value

    def getCount(self):
        return self._value if isinstance(self._value, int) else 0

    def getMFNode(self, i):
        return self._value


class _StubNode:
    """Node stub configurable with a fields dict and a typename."""
    def __init__(self, fields: dict, typename: str = "HingeJoint",
                 node_id: int = 1):
        self._fields = fields
        self._typename = typename
        self._id = node_id

    def getTypeName(self):
        return self._typename

    def getId(self):
        return self._id

    def getField(self, name):
        if name not in self._fields:
            return None
        v = self._fields[name]
        if isinstance(v, _StubField):
            return v
        # Field-like duck-typed objects (have getCount / getMFNode) pass
        # through; primitives get wrapped so the SF accessors work.
        if hasattr(v, "getCount") or hasattr(v, "getMFNode"):
            return v
        return _StubField(v)

    def getDef(self):
        return self._fields.get("__def")


def test_joint_snapshot_reports_position_and_limits():
    import observe
    motor = _StubNode({"name": "joint_a"}, typename="RotationalMotor")
    devices_field = _StubField(_StubField(motor))  # getMFNode(0) -> motor
    # The device list has count 1; we patch getCount via a small subclass.

    class OneCount:
        def getCount(self):
            return 1
        def getMFNode(self, i):
            return motor
    params = _StubNode({
        "position": _StubField(0.5),
        "minStop": _StubField(-1.0),
        "maxStop": _StubField(1.0),
    })
    joint = _StubNode({
        "jointParameters": _StubField(params),
        "device": OneCount(),
        "endPoint": _StubField(_StubNode({"name": "linkA"})),
    })
    snap = observe.joint_snapshot(joint, prev_position=None, dt_s=0.0)
    assert snap["name"] == "joint_a"
    assert snap["position"] == pytest.approx(0.5)
    assert snap["lower"] == pytest.approx(-1.0)
    assert snap["upper"] == pytest.approx(1.0)
    assert snap["hit_limit"] is None
    assert snap["velocity"] is None


def test_joint_snapshot_velocity_is_differenced():
    import observe
    params = _StubNode({
        "position": _StubField(0.7),
        "minStop": _StubField(-1.0),
        "maxStop": _StubField(1.0),
    })
    joint = _StubNode({
        "jointParameters": _StubField(params),
        "endPoint": _StubField(_StubNode({"name": "linkA"})),
    })
    snap = observe.joint_snapshot(joint, prev_position=0.5, dt_s=0.1)
    assert snap["velocity"] == pytest.approx(2.0)


def test_joint_snapshot_flags_upper_limit_hit():
    import observe
    params = _StubNode({
        "position": _StubField(1.0),
        "minStop": _StubField(-1.0),
        "maxStop": _StubField(1.0),
    })
    joint = _StubNode({
        "jointParameters": _StubField(params),
        "endPoint": _StubField(_StubNode({"name": "linkA"})),
    })
    snap = observe.joint_snapshot(joint, prev_position=0.99, dt_s=0.1)
    assert snap["hit_limit"] == "upper"


def test_joint_snapshot_no_limit_when_stops_unset():
    import observe
    params = _StubNode({
        "position": _StubField(0.5),
        "minStop": _StubField(0.0),
        "maxStop": _StubField(0.0),
    })
    joint = _StubNode({
        "jointParameters": _StubField(params),
        "endPoint": _StubField(_StubNode({"name": "linkA"})),
    })
    snap = observe.joint_snapshot(joint, prev_position=None, dt_s=0.0)
    assert snap["hit_limit"] is None
    assert snap["lower"] == 0.0
    assert snap["upper"] == 0.0
