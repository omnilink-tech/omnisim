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

"""Read-path optimisation contract (2026-07-31).

Two mechanisms made the scene-walking READ endpoints (~11-20 s on a 298-node
scene even in --light) fast, and both carry honesty obligations these tests
pin down:

1. **Paused read bursts** (`observe.paused_reads`): a supervisor read
   round-trip costs ~6 ms against a free-running engine vs ~0.15 ms against a
   paused one (measured, 40x), so the read-heavy RPCs pause the engine for
   the walk and restore the caller's mode. Obligations: the mode is ALWAYS
   restored (including on exception), stubs without a mode API degrade to the
   old behaviour, and **no code inside a paused block may step the sim** — a
   step against a paused engine deadlocks. The AST tripwire below enforces
   the last one against the real sources.

2. **Posed-node classification** (`observe.node_is_posed` +
   `observe.record_pose_measurement`, consumed by `node_summary`): a non-Pose
   node's getPosition() costs a round-trip, warns into the world log, and
   returns NaN — sanitized to null downstream. The classifier answers null
   directly, at zero round-trips. Obligations: pose VALUES are never cached
   (only the type-level "has a pose" verdict), a wrong static-table entry
   must not be able to null out a real pose (unknown types classify from a
   measured read, and measurements never override the tables), and the
   verdict is keyed by TYPE NAME — posedness is a property of the type, so a
   spawned node of a known type is classified correctly on first sight and
   the memo stays bounded by the type vocabulary.
"""

from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from conftest import SUPERVISOR_DIR, HARNESS_DIR, exec_supervisor_slice  # noqa: E402

sys.path.insert(0, str(SUPERVISOR_DIR))
sys.path.insert(0, str(HARNESS_DIR))

import observe  # noqa: E402
from omnisim_harness import sanitize_nonfinite  # noqa: E402


@pytest.fixture(autouse=True)
def _fresh_posed_memo():
    """Measured type verdicts are module state in observe; stub type names
    repeat across tests, so isolate them."""
    observe.reset_posed_memo()
    yield
    observe.reset_posed_memo()


# ---------------------------------------------------------------------------
# Stubs
# ---------------------------------------------------------------------------


class ModedSupervisor:
    """Supervisor stub with a simulation-mode API and a call journal."""

    SIMULATION_MODE_PAUSE = 0
    SIMULATION_MODE_FAST = 2

    def __init__(self, mode=2):
        self.mode = mode
        self.journal: list = []

    def simulationGetMode(self):
        return self.mode

    def simulationSetMode(self, mode):
        self.journal.append(("set_mode", mode))
        self.mode = mode


class BareSupervisor:
    """No mode API at all — the unit-test stub shape used across the suite."""


class StubNode:
    """A node whose pose behaviour is configurable, with read counters.

    ``pose``: ``"real"`` (finite, tracks ``self.position``), ``"nan"`` (what
    the engine answers for a non-Pose node), or ``"raise"`` (what stubs
    elsewhere in the suite do when a read is unsupported).
    """

    def __init__(self, type_name="Solid", node_id=7, def_name=None,
                 position=(1.0, 2.0, 3.0), pose="real", base_type=None,
                 children=()):
        self._type = type_name
        self._base = base_type if base_type is not None else type_name
        self._id = node_id
        self._def = def_name
        self.position = list(position)
        self.pose = pose
        self.position_reads = 0
        self.orientation_reads = 0
        self._children = list(children)

    def getTypeName(self):
        return self._type

    def getBaseTypeName(self):
        return self._base

    def getDef(self):
        return self._def

    def getId(self):
        return self._id

    def getPosition(self):
        self.position_reads += 1
        if self.pose == "raise":
            raise RuntimeError("no pose")
        if self.pose == "nan":
            return [float("nan")] * 3
        return list(self.position)

    def getOrientation(self):
        self.orientation_reads += 1
        if self.pose == "raise":
            raise RuntimeError("no pose")
        if self.pose == "nan":
            return [float("nan")] * 9
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    def getField(self, name):
        if name == "children" and self._children:
            return _MFChildren(self._children)
        return None


class _MFChildren:
    def __init__(self, nodes):
        self._nodes = nodes

    def getCount(self):
        return len(self._nodes)

    def getMFNode(self, i):
        return self._nodes[i]


def _tree_helpers():
    """node_summary + walk_scene_tree, exec'd from the supervisor source
    (the module itself does not import under stock pytest)."""
    return exec_supervisor_slice("def node_summary", "def find_node_by_def",
                                 observe=observe, math=math)


def _call_name(call: ast.Call) -> str | None:
    func = call.func
    return getattr(func, "id", None) or getattr(func, "attr", None)


# ---------------------------------------------------------------------------
# paused_reads
# ---------------------------------------------------------------------------


def test_paused_reads_pauses_then_restores_the_previous_mode():
    sup = ModedSupervisor(mode=ModedSupervisor.SIMULATION_MODE_FAST)
    with observe.paused_reads(sup) as took:
        assert took is True
        assert sup.mode == ModedSupervisor.SIMULATION_MODE_PAUSE
    assert sup.mode == ModedSupervisor.SIMULATION_MODE_FAST
    assert sup.journal == [("set_mode", 0), ("set_mode", 2)]


def test_paused_reads_restores_the_mode_even_when_the_body_raises():
    sup = ModedSupervisor(mode=ModedSupervisor.SIMULATION_MODE_FAST)
    with pytest.raises(ValueError):
        with observe.paused_reads(sup):
            raise ValueError("walk crashed")
    assert sup.mode == ModedSupervisor.SIMULATION_MODE_FAST


def test_paused_reads_is_a_noop_for_a_supervisor_without_a_mode_api():
    with observe.paused_reads(BareSupervisor()) as took:
        assert took is False


def test_paused_reads_does_not_double_pause_an_already_paused_engine():
    """Re-entrancy: the self-pausing observe readers may be called from an
    already-paused dispatch block; the inner context must leave the mode
    alone (restoring 'pause' over an external resume would race)."""
    sup = ModedSupervisor(mode=ModedSupervisor.SIMULATION_MODE_PAUSE)
    with observe.paused_reads(sup) as took:
        assert took is False
    assert sup.journal == []


def test_no_sim_step_is_reachable_inside_a_paused_read_block():
    """AST tripwire: stepping a paused engine deadlocks, so no `with
    paused_reads(...)` block — in dispatch OR in observe's self-pausing
    readers — may contain a supervisor.step() or _advance() call. The wake
    path advances FIRST, then the collect pauses; this pins that ordering
    against regression.
    """
    offenders: list[str] = []
    for path in (SUPERVISOR_DIR / "harness_supervisor.py",
                 SUPERVISOR_DIR / "observe.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.With):
                continue
            if not any(isinstance(i.context_expr, ast.Call)
                       and _call_name(i.context_expr) == "paused_reads"
                       for i in node.items):
                continue
            for inner in ast.walk(node):
                if isinstance(inner, ast.Call) and \
                        _call_name(inner) in ("step", "_advance"):
                    offenders.append(
                        f"{path.name}:{inner.lineno}: {_call_name(inner)}()")
    assert offenders == [], (
        "sim-stepping calls inside paused_reads blocks (deadlock): "
        f"{offenders}")


# ---------------------------------------------------------------------------
# posed-type classification
# ---------------------------------------------------------------------------


def test_classification_table_subsumes_the_shared_geometry_tables():
    """POSED_BY_BASE_TYPE is built FROM geometry's tables; if someone forks
    them apart again, this is the drift alarm."""
    import geometry
    for t in geometry._POSE_TYPES:
        assert observe.POSED_BY_BASE_TYPE.get(t) is True, t
    for t in geometry._GEOMETRY_TYPES:
        assert observe.POSED_BY_BASE_TYPE.get(t) is False, t
    for t in observe.JOINT_TYPENAMES:
        assert observe.POSED_BY_BASE_TYPE.get(t) is False, t


def test_base_types_classify_without_any_pose_read():
    for t in ("Solid", "Robot", "Transform"):
        assert observe.node_is_posed(StubNode(t), t) is True
    for t in ("Shape", "HingeJoint", "Viewpoint", "Box"):
        assert observe.node_is_posed(StubNode(t), t) is False


def test_proto_types_classify_via_their_base_type():
    arena = StubNode("RectangleArena", base_type="Solid")
    assert observe.node_is_posed(arena, "RectangleArena") is True
    sky = StubNode("OmniSimSky", base_type="Background")
    assert observe.node_is_posed(sky, "OmniSimSky") is False
    # A base type the tables do not know -> None: the caller must measure.
    weird = StubNode("Weirdo", base_type="WeirdoBase")
    assert observe.node_is_posed(weird, "Weirdo") is None


def test_measurement_classifies_unknown_types_but_never_overrides_tables():
    weird = StubNode("Weirdo", base_type="WeirdoBase")
    observe.record_pose_measurement("Weirdo", [float("nan")] * 3)
    assert observe.node_is_posed(weird, "Weirdo") is False
    # A (buggy) measurement recorded against a table-known type is ignored:
    # the static tables win, so a real pose can never be nulled out.
    observe.record_pose_measurement("Solid", [float("nan")] * 3)
    assert observe.node_is_posed(StubNode("Solid"), "Solid") is True


# ---------------------------------------------------------------------------
# node_summary
# ---------------------------------------------------------------------------


def test_posed_node_pose_is_reread_every_call_never_cached():
    """Only the CLASSIFICATION is cached. Pose values must track the sim."""
    ns = _tree_helpers()
    node = StubNode("Solid", position=(1.0, 2.0, 3.0))
    first = ns["node_summary"](node)
    assert first["position"] == [1.0, 2.0, 3.0]
    node.position = [9.0, 9.0, 9.0]  # the body moved (set_pose, physics, ...)
    second = ns["node_summary"](node)
    assert second["position"] == [9.0, 9.0, 9.0], \
        "stale pose served from a cache — worse than the slow right answer"
    assert node.position_reads == 2


def test_known_nonpose_node_answers_null_with_zero_pose_reads():
    ns = _tree_helpers()
    node = StubNode("Shape", pose="nan")
    out = ns["node_summary"](node)
    assert out["position"] == [None, None, None]
    assert out["orientation"] == [None] * 9
    assert node.position_reads == 0, \
        "a Shape's pose was read anyway: round-trip + engine warning wasted"


def test_unknown_type_is_classified_from_a_measured_read_not_a_table():
    """A type the tables don't know pays ONE read (exactly what the old code
    paid every call); NaN classifies it unposed, and every later node of that
    type is free. The sanitized output of the measured (NaN) and classified
    (null) calls must be identical — the HTTP contract does not move.
    """
    ns = _tree_helpers()
    node = StubNode("SomethingNew", base_type="SomethingNew", pose="nan")
    first = ns["node_summary"](node)
    assert node.position_reads == 1
    second = ns["node_summary"](node)
    assert node.position_reads == 1, "second call must not re-read"
    assert sanitize_nonfinite(first) == sanitize_nonfinite(second)
    # ...and so is a DIFFERENT node of the same type (type-keyed verdict).
    sibling = StubNode("SomethingNew", base_type="SomethingNew", pose="nan",
                       node_id=8)
    ns["node_summary"](sibling)
    assert sibling.position_reads == 0


def test_unknown_type_with_a_real_pose_is_never_nulled():
    ns = _tree_helpers()
    node = StubNode("SomethingNew", base_type="SomethingNew",
                    position=(4.0, 5.0, 6.0))
    out = ns["node_summary"](node)
    assert out["position"] == [4.0, 5.0, 6.0]
    node.position = [1.0, 1.0, 1.0]
    assert ns["node_summary"](node)["position"] == [1.0, 1.0, 1.0]


def test_unreadable_pose_keeps_the_legacy_shape_and_stays_unclassified():
    """Stub nodes elsewhere in the suite raise on getPosition; the legacy
    behaviour (keys absent) must survive, and a verdict must not be locked in
    from a read that never happened."""
    ns = _tree_helpers()
    node = StubNode("MysteryStub", base_type="MysteryStub", pose="raise")
    out = ns["node_summary"](node)
    assert "position" not in out
    assert "orientation" not in out
    # Still unclassified: a later readable node of this type gets measured.
    assert observe.node_is_posed(node, "MysteryStub") is None


def test_a_freshly_spawned_node_classifies_on_first_sight():
    """Invalidation-by-construction: the verdict is keyed by TYPE, and a
    type's posedness cannot change — so nodes spawned after the memo warmed
    up (new ids, known or unknown types) are classified correctly with no
    invalidation hook. (World loads restart the supervisor process, so the
    memo cannot outlive its world at all.)"""
    ns = _tree_helpers()
    ns["node_summary"](StubNode("Solid", node_id=7))          # warm
    ns["node_summary"](StubNode("Shape", node_id=8, pose="nan"))
    spawned = StubNode("Solid", node_id=99, position=(0.5, 0.5, 0.1))
    out = ns["node_summary"](spawned)
    assert out["position"] == [0.5, 0.5, 0.1]
    assert spawned.position_reads == 1


# ---------------------------------------------------------------------------
# walk_scene_tree: classified path is byte-equivalent to the legacy reads
# ---------------------------------------------------------------------------


def _stub_scene():
    """Root Group -> [WorldInfo, Robot -> Solid -> Shape]. Returns the root
    and the non-posed nodes (whose pose reads should be skipped)."""
    shape = StubNode("Shape", 5, pose="nan")
    body = StubNode("Solid", 4, "BODY", (1, 2, 0.5), children=[shape])
    robot = StubNode("Robot", 3, "HUSKY", (1, 2, 0.6), children=[body])
    world_info = StubNode("WorldInfo", 2, pose="nan")
    root = StubNode("Group", 1, pose="nan", children=[world_info, robot])
    return root, [root, world_info, shape]


def test_walk_scene_tree_matches_the_read_every_time_shape_after_sanitize():
    """The classified walk must emit the same sanitized payload the NaN reads
    produced: null triples for non-posed nodes, real poses for posed ones,
    same keys, same order."""
    ns = _tree_helpers()
    root, nonposed = _stub_scene()
    out = sanitize_nonfinite(ns["walk_scene_tree"](root))
    assert [n["type"] for n in out] == \
        ["Group", "WorldInfo", "Robot", "Solid", "Shape"]
    by_type = {n["type"]: n for n in out}
    assert by_type["Robot"]["position"] == [1, 2, 0.6]
    assert by_type["Solid"]["parent_def"] == "HUSKY"
    for n in nonposed:
        assert by_type[n.getTypeName()]["position"] == [None, None, None]
        assert by_type[n.getTypeName()]["orientation"] == [None] * 9


def test_walk_scene_tree_skips_nonpose_reads_entirely():
    ns = _tree_helpers()
    root, nonposed = _stub_scene()
    ns["walk_scene_tree"](root)
    assert all(n.position_reads == 0 for n in nonposed), \
        "table-known non-posed types must never pay the NaN round-trip"
