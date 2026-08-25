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

# The supervisor module imports `from omnisim import Supervisor`,
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


# ---------------------------------------------------------------------------
# /sim/contacts — pairing and tracking scope
# ---------------------------------------------------------------------------
#
# The engine's ContactPoint.node_id is NOT the other body: it is the QUERIED
# solid's own id (OmSupervisorUtilities::pushContactPointsToStream). Measured on
# a crate resting on a floor: FLOOR reported ids [9,9,9,9] and CRATE_BOT
# [14,14,14,14] for the same four contacts. These stubs reproduce exactly that,
# so a regression back to "key on node_id" shows up as self-pairs.


class _StubContactPoint:
    def __init__(self, point, node_id):
        self.point = point
        self.node_id = node_id


class _ContactSolid:
    """A Solid stub that reports its OWN id on every contact point."""

    def __init__(self, def_name, node_id, points, physics=True, velocity=None):
        self._def = def_name
        self._id = node_id
        self._points = points
        self._physics = physics
        self._velocity = velocity or [0.0] * 6

    def getTypeName(self):
        return "Solid"

    def getDef(self):
        return self._def

    def getId(self):
        return self._id

    def getContactPoints(self, include_descendants=False):
        return [_StubContactPoint(p, self._id) for p in self._points]

    def getVelocity(self):
        return self._velocity

    def getField(self, name):
        if name == "physics" and self._physics:
            return _StubField(object())
        return None


class _ContactWorld:
    """Supervisor stub whose root children are the given solids."""

    def __init__(self, solids, disable_time=1.0):
        self._solids = solids
        self._disable_time = disable_time

    def getRoot(self):
        world_info = _StubNode({
            "physicsDisableTime": _StubField(self._disable_time),
            "physicsDisableLinearThreshold": _StubField(0.01),
            "physicsDisableAngularThreshold": _StubField(0.01),
        }, typename="WorldInfo")

        class Children:
            def __init__(self, nodes):
                self._nodes = nodes

            def getCount(self):
                return len(self._nodes)

            def getMFNode(self, i):
                return self._nodes[i]

        return _StubNode({"children": Children([world_info] + self._solids)},
                         typename="Group")


class _ContactRobot(_ContactSolid):
    def __init__(self, def_name, node_id, points):
        super().__init__(def_name, node_id, points)
        self.deep_queries = []

    def getTypeName(self):
        return "Robot"

    def getContactPoints(self, include_descendants=False):
        self.deep_queries.append(include_descendants)
        return super().getContactPoints(include_descendants)


def test_contacts_are_paired_by_shared_point_not_by_node_id():
    import observe
    shared = [[0.2, 0.2, 0.05], [-0.2, 0.2, 0.05]]
    floor = _ContactSolid("FLOOR", 9, shared, physics=False)
    crate = _ContactSolid("CRATE_BOT", 14, shared, physics=True)
    result = observe.collect_contacts(_ContactWorld([floor, crate]))
    pairs = {(c["a_def"], c["b_def"]) for c in result["contacts"]}
    assert pairs == {("FLOOR", "CRATE_BOT")}
    assert all(c["paired"] for c in result["contacts"])
    assert len(result["contacts"]) == 2          # one entry per contact point


def test_robot_contact_scan_queries_each_subtree_once_and_pairs_robots():
    import observe
    shared = [0.0, 0.0, 0.1]
    robot_a = _ContactRobot("ROBOT_A", 10, [shared, [1.0, 0.0, 0.0]])
    robot_b = _ContactRobot("ROBOT_B", 20, [shared, [2.0, 0.0, 0.0]])

    contacts = observe.list_robot_contacts([robot_a, robot_b])

    paired = [c for c in contacts if c["paired"]]
    assert len(paired) == 1
    assert {paired[0]["a_def"], paired[0]["b_def"]} == {
        "ROBOT_A", "ROBOT_B"}
    assert sum(not c["paired"] for c in contacts) == 2
    assert robot_a.deep_queries == [True]
    assert robot_b.deep_queries == [True]


def test_a_contact_only_one_body_reports_is_marked_unpaired():
    import observe
    lonely = _ContactSolid("RAMP", 3, [[1.0, 0.0, 0.0]], physics=False)
    result = observe.collect_contacts(_ContactWorld([lonely]))
    (contact,) = result["contacts"]
    assert contact["a_def"] == "RAMP"
    assert contact["b_def"] is None
    assert contact["paired"] is False
    assert contact["point"] == [1.0, 0.0, 0.0]
    # A null partner must carry its reason: the usual cause is a PROTO floor a
    # Supervisor cannot query, not "the contact is fake".
    assert "not nameable" in contact["note"]
    assert result["tracking"]["contacts_unpaired"] == 1
    assert result["tracking"]["contacts_paired"] == 0


def test_tracking_scope_says_what_was_walked_and_never_claims_completeness():
    """⚠ THIS TEST USED TO PIN A FICTION.

    It asserted `physics_disable_time_s == 1.0` and a `caveat` reading "an EMPTY
    contacts list is not proof of no contact: ODE auto-disables a body idle for
    WorldInfo.physicsDisableTime ... call sim_contacts with wake=true". There is
    no ODE (src/ode deleted, bdc02139) and no body sleep -- `physicsDisableTime`
    is parsed into OmWorldInfo and read back by NOTHING -- so the surface was
    naming a false cause for the exact symptom a real defect produces, and
    recommending a no-op as the fix.

    What must hold instead: the scope is reported, completeness is NEVER
    asserted, and the reasons an empty set can be empty are enumerated."""
    import observe
    crate = _ContactSolid("CRATE_BOT", 14, [], physics=True,
                          velocity=[0.0] * 6)          # at rest
    tracking = observe.collect_contacts(_ContactWorld([crate]))["tracking"]
    assert tracking["solids_walked"] == 1
    assert tracking["name_filter"] is None             # NOT scoped to one robot
    assert tracking["measured"] is True
    assert "UNKNOWN" in tracking["completeness"]
    assert tracking["empty_set_is_proof_of_no_contact"] is False
    causes = {r["cause"] for r in tracking["empty_set_reasons"]}
    assert "solid_pinned_physics_backend_ode" in causes
    assert "no_physics_backend_available" in causes
    assert "genuinely_not_touching" in causes
    # No sleep model ASSERTED anywhere on this surface. The exact clauses the
    # old response carried, none of which may come back:
    blob = repr(tracking).lower()
    for lie in ("auto-disables", "auto-disable was cleared",
                "generates no contact points", "clear the sleep timer",
                "see resting contacts", "list is complete",
                "complete for the step"):
        assert lie not in blob, lie
    # `physicsDisableTime` may only be MENTIONED to say it has no reader -- never
    # as a live knob, and never as something to write.
    if "physicsdisabletime" in blob:
        assert "no reader in the engine" in blob


def test_a_body_at_rest_is_reported_but_is_not_a_reason_for_an_empty_set():
    """A resting body DOES report its contacts (native contact readback is on by
    default), so `bodies_at_rest` is informational -- never an excuse."""
    import observe
    crate = _ContactSolid("CRATE_BOT", 14, [], physics=True,
                          velocity=[0.0] * 6)
    tracking = observe.collect_contacts(_ContactWorld([crate]))["tracking"]
    assert tracking["bodies_at_rest"] == ["CRATE_BOT"]
    assert tracking["idle_bodies"] == ["CRATE_BOT"]      # back-compat alias
    assert "NOT a reason" in tracking["bodies_at_rest_note"]


def test_a_moving_body_is_not_listed_as_at_rest():
    import observe
    flier = _ContactSolid("CRATE_BOT", 14, [], physics=True,
                          velocity=[0.0, 0.0, -12.0, 0.0, 0.0, 0.0])
    tracking = observe.collect_contacts(_ContactWorld([flier]))["tracking"]
    assert tracking["bodies_at_rest"] == []
    assert tracking["idle_bodies"] == []
    # The honesty block is UNCONDITIONAL: the old caveat only appeared when a
    # body happened to be idle, so a caller on a moving scene was told nothing
    # about the endpoint's ambiguity at all.
    assert tracking["empty_set_is_proof_of_no_contact"] is False
    assert len(tracking["empty_set_reasons"]) >= 5


def test_contact_tracker_emits_the_real_pair_not_a_self_pair():
    import event_bus
    shared = [[0.2, 0.2, 0.05]]
    world = _ContactWorld([_ContactSolid("FLOOR", 9, shared, physics=False),
                           _ContactSolid("CRATE_BOT", 14, shared, physics=True)])
    bus = event_bus.EventBus()
    tracker = event_bus.ContactTracker(world, bus)
    tracker.poll(sim_time_ms=16.0)
    began = [e for e in bus.since(0) if e["type"] == "contact.began"]
    assert len(began) == 1
    assert {began[0]["a_def"], began[0]["b_def"]} == {"FLOOR", "CRATE_BOT"}
    assert tracker.current_pairs() == [("CRATE_BOT", "FLOOR")]
