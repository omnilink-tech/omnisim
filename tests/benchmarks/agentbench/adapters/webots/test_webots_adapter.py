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

"""Unit tests for the upstream-Webots adapter. **No Webots involved.**

Every fixture here is a synthetic artifact tree written into ``tmp_path`` -- a
``trajectory.json``, a ``roster.json``, a captured console, a ``.wbt`` -- so these
run in milliseconds on a machine with no Webots install, no WSL2 and no network.
That is deliberate: the control arm of a cross-simulator comparison has to be
auditable by someone who does not have our stack.

    pytest tests/benchmarks/agentbench/adapters/webots -q

The cases that exist because of a specific hazard:

  * ``test_absent_contact_file_yields_none_not_an_empty_observation`` -- an empty
    ``ContactObservation`` claims "the query ran and found nothing", which is a
    different statement from "there was no query". Getting this wrong is how
    A1.3's contact clause passed for weeks on OmniSim.
  * ``test_a_nue_world_is_remapped_into_the_neutral_frame`` -- a y-up world's
    ``xyz`` is not in the bundle's frame, and a grader that ignores
    ``WorldInfo.coordinateSystem`` measures a robot's height as its northing.
  * ``test_void_controller_is_not_a_behaviour`` -- ``"void"`` is upstream's own
    no-controller sentinel; treating it as a controller reports ten controlled
    robots in a scene where nothing moves.
  * ``test_missing_console_cannot_count_controller_starts`` -- upstream writes no
    log file at all, so this degradation is normal, not exotic.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from agentbench import adapters                                   # noqa: E402
from agentbench.adapters.webots import evidence, recording, worldtext  # noqa: E402
from agentbench.graders import a1_core                            # noqa: E402
from agentbench.graders.verdict import FAIL, INVALID, PASS        # noqa: E402

DT = 0.032
WINDOW_S = 30.0
N_ROBOTS = 10
PROTO = "Moose"                 # the declared analogue: upstream ships no Husky
CONTROLLER = "bench_random_walk"


# --- synthetic artifacts -----------------------------------------------------


def _names(n=N_ROBOTS):
    return ["moose%d" % i for i in range(n)]


def _tracks(n=N_ROBOTS, *, length_m=6.0, spacing=3.0, z=0.2, bearings=None):
    """Straight constant-speed tracks, well spread in bearing. Metres."""
    k = int(round(WINDOW_S / DT)) + 1
    t = np.arange(k) * DT
    if bearings is None:
        bearings = [2.0 * math.pi * i / n for i in range(n)]
    s = np.linspace(0.0, length_m, k)
    out = []
    for i in range(n):
        x = spacing * i + s * math.cos(bearings[i])
        y = s * math.sin(bearings[i])
        out.append(np.stack([x, y, np.full(k, z)], axis=1))
    return t, out


def write_run(dirpath, *, n=N_ROBOTS, controller=CONTROLLER, frozen=True,
              synchronization=True, coordinate_system=None, with_aabb=True,
              contacts="ok", trajectory="ok", console="ok", perf=True,
              exit_code=0, timed_out=False, complete=True, joints=8,
              proto=PROTO, order=None, length_m=6.0, spacing=3.0):
    """Write one synthetic upstream-Webots artifact set. Returns the dir."""
    d = Path(dirpath)
    d.mkdir(parents=True, exist_ok=True)
    t, tracks = _tracks(n, length_m=length_m, spacing=spacing)
    names = _names(n)

    bodies = []
    for i, nm in enumerate(names):
        rec = {"name": nm, "def": "UGV%d" % i, "id": 100 + i, "type": proto,
               "base_type": "Robot", "n_joints": joints,
               "controller": controller, "has_physics": True,
               "position": [spacing * i, 0.0, 0.2]}
        if with_aabb:
            rec["aabb_min"] = [spacing * i - 0.57, -0.5, 0.05]
            rec["aabb_max"] = [spacing * i + 0.57, 0.5, 0.45]
        bodies.append(rec)
    roster = {"t_s": 0.0, "frozen": frozen, "synchronization": synchronization,
              "bodies": bodies}
    if coordinate_system:
        roster["coordinate_system"] = coordinate_system
    (d / "roster.json").write_text(json.dumps(roster), encoding="utf-8")

    if trajectory == "ok":
        idx = list(range(n)) if order is None else list(order)
        doc = {"dt_s": DT, "world": str(d / "world.omniworld"),
               "bodies": [{"name": names[i], "t": [float(v) for v in t],
                           "xyz": [[float(a), float(b), float(c)]
                                   for a, b, c in tracks[i]]} for i in idx]}
        (d / "trajectory.json").write_text(json.dumps(doc), encoding="utf-8")
    elif trajectory == "ragged":
        doc = {"dt_s": DT, "bodies": []}
        for i in range(n):
            cut = len(t) - (1 if i == 3 else 0)
            doc["bodies"].append(
                {"name": names[i], "t": [float(v) for v in t[:cut]],
                 "xyz": [[float(a), float(b), float(c)]
                         for a, b, c in tracks[i][:cut]]})
        (d / "trajectory.json").write_text(json.dumps(doc), encoding="utf-8")
    elif trajectory == "garbage":
        (d / "trajectory.json").write_text("{not json", encoding="utf-8")
    elif trajectory == "empty":
        (d / "trajectory.json").write_text(json.dumps({"dt_s": DT}),
                                           encoding="utf-8")

    if contacts == "ok":
        (d / "contacts.json").write_text(json.dumps(
            {"supported": True, "steps": 10, "window_s": 10 * DT,
             "total_observed": 40, "distinct_named": 12, "pairs": []}),
            encoding="utf-8")
    elif contacts == "no_witness":
        (d / "contacts.json").write_text(json.dumps(
            {"supported": True, "steps": 10, "pairs": []}), encoding="utf-8")
    elif contacts == "hit":
        (d / "contacts.json").write_text(json.dumps(
            {"supported": True, "steps": 10, "total_observed": 41,
             "distinct_named": 13,
             "pairs": [{"a": "UGV0", "b": "UGV1", "a_robot": True,
                        "b_robot": True, "point": [1.0, 0.0, 0.1],
                        "step": 3}]}), encoding="utf-8")
    # contacts == "absent": no file at all, on purpose.

    (d / "completion.json").write_text(json.dumps(
        {"complete": complete, "quit_called": True,
         "recorded_s": float(t[-1]), "steps": len(t) - 1, "dt_s": DT}),
        encoding="utf-8")
    (d / "process.json").write_text(json.dumps(
        {"exit_code": exit_code, "timed_out": timed_out, "wall_s": 41.7,
         "attempts_used": 1, "webots_version": "R2025a",
         "command": "xvfb-run -a webots --batch --mode=fast --no-rendering"}),
        encoding="utf-8")

    if console == "ok":
        lines = ["INFO: %s: Starting controller: python3 %s.py"
                 % (controller, controller)] * n
        lines.append("INFO: bench_supervisor: Starting controller: python3 x.py")
        (d / "console.log").write_text("\n".join(lines) + "\n",
                                       encoding="utf-8")
    elif console == "errors":
        (d / "console.log").write_text(
            "INFO: %s: Starting controller: python3 x.py\n"
            "ERROR: Cannot load the world file.\n" % controller,
            encoding="utf-8")
    elif console == "fatal":
        (d / "console.log").write_text(
            "INFO: %s: Starting controller: python3 x.py\n"
            "FATAL: unrecoverable\n" % controller, encoding="utf-8")
    # console == "absent": upstream writes no log file, so this is just silence.

    if perf:
        (d / "perf.txt").write_text("World: world.omniworld\nTOT: 1500 steps\n",
                                    encoding="utf-8")
    else:
        (d / "perf.txt").write_text("", encoding="utf-8")
    return d


WORLD_HEAD = """#VRML_SIM R2025a utf8
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/clearpath/moose/protos/Moose.proto"

WorldInfo {
  basicTimeStep 32
  # a comment mentioning Moose { } that must NOT be counted
  contactProperties [
    ContactProperties {
      material1 "moose wheel"
    }
  ]
%s}
Viewpoint {
  position -70 -70 52
}
RectangleArena {
  floorSize 100 100
}
"""


def write_world(path, *, n=N_ROBOTS, proto=PROTO, controller=CONTROLLER,
                coordinate_system=None, physics_plugin=None, spacing=3.0):
    extra = ""
    if coordinate_system:
        extra += '  coordinateSystem "%s"\n' % coordinate_system
    if physics_plugin:
        extra += '  physics "%s"\n' % physics_plugin
    text = [WORLD_HEAD % extra]
    for i in range(n):
        text.append('DEF UGV%d %s {\n'
                    '  translation %.3f 0 0.38\n'
                    '  name "moose%d"\n'
                    '  controller "%s"\n'
                    '  bodySlot [\n    GPS {\n    }\n  ]\n'
                    '}\n' % (i, proto, spacing * i, i, controller))
    text.append('DEF BENCH Robot {\n  name "bench"\n'
                '  controller "bench_supervisor"\n  supervisor TRUE\n}\n')
    p = Path(path)
    p.write_text("".join(text), encoding="utf-8")
    return p


def a1_bundle(tmp_path, *, world_kw=None, **run_kw):
    """A full A1 bundle from synthetic artifacts, via the real adapter."""
    d = write_run(tmp_path / "webots", **run_kw)
    world = write_world(tmp_path / "world.omniworld", **(world_kw or {}))
    return evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                                 live_expected=True, artifact=world,
                                 webots_dir=d, robot_proto=PROTO)


def _ids(verdict):
    return {a.id: a for a in verdict.assertions}


# --- the happy path ---------------------------------------------------------


def test_a_complete_webots_run_grades_as_a_pass(tmp_path):
    """The whole point: the same core scores a Webots run 10/10."""
    b = a1_bundle(tmp_path)
    assert adapters.check_bundle(b) == []
    r = a1_core.grade(b)
    assert r.outcome == PASS, r.summary()
    assert len(r.assertions) == 10
    assert not r.vacuous, r.vacuous
    a = _ids(r)
    assert a["A1.5"].measured["min net displacement (m)"] == pytest.approx(6.0)
    assert a["A1.6"].measured["min path length (m)"] == pytest.approx(6.0)
    assert a["A1.7"].measured["max |dz| (m)"] == pytest.approx(0.0)
    assert a["A1.8"].measured["circular resultant R"] < a1_core.MAX_R


def test_the_bundle_names_the_simulator_and_the_adapter(tmp_path):
    b = a1_bundle(tmp_path)
    assert b.sim == "webots"
    assert b.adapter == "agentbench.adapters.webots.evidence"
    prov = b.provenance()
    assert prov["sim"] == "webots"
    for key in ("roster", "t0", "contacts", "motion", "attribution"):
        assert prov[key], "%s carries no citation" % key


def test_units_are_metres_and_seconds(tmp_path):
    b = a1_bundle(tmp_path)
    t = b.trajectory
    assert t.dt_s == pytest.approx(DT)
    assert t.recorded_s >= WINDOW_S
    assert t.xyz.shape == (N_ROBOTS, t.n_samples, 3)
    assert float(t.t[0]) == 0.0


# --- identity ---------------------------------------------------------------


def test_identity_rule_is_published_and_names_both_sides(tmp_path):
    b = a1_bundle(tmp_path)
    ident = b.identity
    assert ident.declared_count == N_ROBOTS
    assert "getTypeName()" in ident.scene_rule
    assert "PROTO" in ident.declaration_rule
    assert PROTO in ident.scene_rule
    # the substitution is stated, not buried
    assert "DECLARED ANALOGUE" in ident.scene_rule
    assert PROTO in ident.label
    assert any("ships NO Husky" in n for n in b.notes)


def test_without_a_declared_analogue_the_husky_predicate_is_simply_unmet(
        tmp_path):
    """Upstream has no Husky, so a Moose swarm scores zero on A1.1 -- honestly.

    ``robot_class`` is ``False`` (a Moose is definitely not a Husky), not
    ``None``: the question was answerable and the answer was no. The declaration
    count is a real 0 for the same reason.
    """
    d = write_run(tmp_path / "webots")
    world = write_world(tmp_path / "world.omniworld")
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              live_expected=True, artifact=world, webots_dir=d)
    assert b.identity.declared_count == 0
    assert [x.robot_class for x in b.roster.bodies] == [False] * N_ROBOTS
    r = a1_core.grade(b)
    assert "A1.1" in r.failed
    ident_clause = [f for f in _ids(r)["A1.1"].falsifiers
                    if f.clause == "identity"][0]
    assert ident_clause.present, "the identity question WAS answered"


def test_a_husky_proto_is_accepted_without_any_substitution(tmp_path):
    """If the asset is pre-converted with upstream's urdf2webots, it just works."""
    d = write_run(tmp_path / "webots", proto="Husky")
    world = write_world(tmp_path / "world.omniworld", proto="Husky")
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              live_expected=True, artifact=world, webots_dir=d)
    assert b.identity.declared_count == N_ROBOTS
    assert a1_core.grade(b).outcome == PASS


def test_the_joint_floor_matches_the_omnisim_adapter(tmp_path):
    """A body under the joint floor is not the robot, on either simulator."""
    from agentbench.adapters.omnisim import evidence as om
    assert evidence.MIN_JOINTS == om.MIN_JOINTS
    b = a1_bundle(tmp_path, joints=2)
    assert [x.robot_class for x in b.roster.bodies] == [False] * N_ROBOTS
    assert "A1.1" in a1_core.grade(b).failed


def test_a_body_with_neither_type_channel_is_unanswerable(tmp_path):
    d = write_run(tmp_path / "webots")
    doc = json.loads((d / "roster.json").read_text(encoding="utf-8"))
    for rec in doc["bodies"]:
        rec.pop("type")
        rec.pop("base_type")
    (d / "roster.json").write_text(json.dumps(doc), encoding="utf-8")
    world = write_world(tmp_path / "world.omniworld")
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              live_expected=True, artifact=world, webots_dir=d,
                              robot_proto=PROTO)
    assert all(x.robot_class is None for x in b.roster.bodies)
    r = a1_core.grade(b)
    ident_clause = [f for f in _ids(r)["A1.1"].falsifiers
                    if f.clause == "identity"][0]
    assert not ident_clause.present, "an unanswerable identity is not evidence"


def test_use_references_are_counted_apart_from_instantiations(tmp_path):
    world = tmp_path / "w.wbt"
    write_world(world, n=3)
    world.write_text(world.read_text(encoding="utf-8") + "USE UGV0\n",
                     encoding="utf-8")
    scanned = worldtext.scan(world)
    assert len(worldtext.proto_instances(scanned, (PROTO,))) == 3
    assert worldtext.uses_of(scanned, (PROTO,)) == ["UGV0"]


def test_the_world_scanner_ignores_comments_and_nesting(tmp_path):
    world = write_world(tmp_path / "w.wbt", n=2)
    scanned = worldtext.scan(world)
    types = [nd["type"] for nd in scanned["nodes"]]
    # the commented-out "Moose { }" inside WorldInfo must not appear, and the
    # nested GPS / ContactProperties nodes are below depth 0
    assert types.count(PROTO) == 2
    assert "GPS" not in types and "ContactProperties" not in types
    assert scanned["basic_time_step_ms"] == pytest.approx(32.0)
    assert len(worldtext.network_protos(scanned)) == 1


# --- the trajectory ---------------------------------------------------------


def test_missing_trajectory_is_reported_not_crashed(tmp_path):
    """No motion file: the four structural assertions still grade."""
    d = write_run(tmp_path / "webots", trajectory=None)
    (d / "trajectory.json").unlink(missing_ok=True)
    world = write_world(tmp_path / "world.omniworld")
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              live_expected=True, artifact=world, webots_dir=d,
                              robot_proto=PROTO)
    assert b.trajectory is None
    assert b.motion_error and "not found" in b.motion_error
    r = a1_core.grade(b)
    assert r.outcome == FAIL
    assert set(r.failed) == {"A1.3", "A1.5", "A1.6", "A1.7", "A1.8", "A1.9",
                             "A1.10"}
    assert _ids(r)["A1.1"].ok is True        # structure was still readable
    assert r.progress == 2                   # artifact_loads, not invalid


def test_unparseable_trajectory_is_reported_not_crashed(tmp_path):
    b = a1_bundle(tmp_path, trajectory="garbage")
    assert b.trajectory is None
    assert "not valid JSON" in b.motion_error
    assert a1_core.grade(b).outcome == FAIL


def test_a_trajectory_with_no_bodies_is_fatal_but_not_an_exception(tmp_path):
    b = a1_bundle(tmp_path, trajectory="empty")
    assert b.trajectory is not None and b.trajectory.xyz is None
    assert "no 'bodies' list" in b.trajectory.error
    assert a1_core.grade(b).outcome == FAIL


def test_ragged_sample_counts_are_truncated_and_published(tmp_path):
    b = a1_bundle(tmp_path, trajectory="ragged")
    assert b.trajectory.xyz is not None
    assert any("truncated to the shortest" in n for n in b.notes)
    assert adapters.check_bundle(b) == []    # t and xyz still agree


def test_trajectory_rows_are_aligned_to_the_roster_order(tmp_path):
    """The index invariant: row i of xyz must be body_ids[i] in roster order."""
    order = list(reversed(range(N_ROBOTS)))
    b = a1_bundle(tmp_path, order=order)
    assert b.trajectory.body_ids == [x.body_id for x in b.roster.bodies]
    for i, body in enumerate(b.roster.bodies):
        assert b.trajectory.xyz[i][0][0] == pytest.approx(body.position[0])


def test_unalignable_trajectory_rows_are_flagged_not_guessed(tmp_path):
    d = write_run(tmp_path / "webots")
    doc = json.loads((d / "trajectory.json").read_text(encoding="utf-8"))
    doc["bodies"][2]["name"] = "a_body_the_roster_never_heard_of"
    (d / "trajectory.json").write_text(json.dumps(doc), encoding="utf-8")
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              artifact=write_world(tmp_path / "w.wbt"),
                              webots_dir=d, robot_proto=PROTO)
    assert any("could not be aligned to the roster" in n for n in b.notes)
    assert b.trajectory.body_ids[2] == "a_body_the_roster_never_heard_of"


def test_a_nue_world_is_remapped_into_the_neutral_frame(tmp_path):
    """A y-up world must not have its northing graded as a height."""
    b = a1_bundle(tmp_path, coordinate_system="NUE",
                  world_kw={"coordinate_system": "NUE"})
    assert any("coordinateSystem 'NUE'" in n for n in b.notes)
    # the fixture's tracks carry z = 0.2 constant; under NUE that column is the
    # EASTING and the UP axis is the middle one, which is 0 here.
    assert float(b.trajectory.xyz[0][0][2]) == pytest.approx(0.0)
    assert b.attribution.extra["coordinate_system"] == "NUE"
    # the AABB corners survive the permutation sorted
    body = b.t0.bodies[0]
    assert all(lo <= hi for lo, hi in zip(body.aabb_min, body.aabb_max))


def test_an_unknown_coordinate_system_is_said_out_loud(tmp_path):
    b = a1_bundle(tmp_path, coordinate_system="XYZ")
    assert any("unrecognised WorldInfo.coordinateSystem" in n for n in b.notes)


# --- contacts ---------------------------------------------------------------


def test_absent_contact_file_yields_none_not_an_empty_observation(tmp_path):
    """THE regression this adapter must not repeat.

    No contact scan ran, so there is no observation -- and the dependent clause
    must report itself vacuous rather than passing on no evidence.
    """
    b = a1_bundle(tmp_path, contacts="absent")
    assert b.contacts is None, "an empty observation would claim a query ran"
    r = a1_core.grade(b)
    a3 = _ids(r)["A1.3"]
    assert a3.ok is True                     # the AABBs really are clear
    assert a3.vacuous
    assert [f.clause for f in a3.vacuous_clauses] == ["no robot-robot contact"]
    assert "~vacuous" in r.summary()


def test_a_contact_query_with_no_witness_counters_stays_none(tmp_path):
    b = a1_bundle(tmp_path, contacts="no_witness")
    assert b.contacts.supported is True
    assert b.contacts.total_observed is None
    assert b.contacts.distinct_named is None
    assert _ids(a1_core.grade(b))["A1.3"].vacuous


def test_zero_is_preserved_where_none_would_be_a_lie(tmp_path):
    """``distinct_named: 0`` is the (id, id) bug, honestly counted -- keep it."""
    d = write_run(tmp_path / "webots", contacts="ok")
    doc = json.loads((d / "contacts.json").read_text(encoding="utf-8"))
    doc.update({"total_observed": 10, "distinct_named": 0})
    (d / "contacts.json").write_text(json.dumps(doc), encoding="utf-8")
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              artifact=write_world(tmp_path / "w.wbt"),
                              webots_dir=d, robot_proto=PROTO)
    assert b.contacts.total_observed == 10
    assert b.contacts.distinct_named == 0
    a3 = _ids(a1_core.grade(b))["A1.3"]
    assert a3.vacuous and "distinct bodies: 0" in a3.vacuous_clauses[0].detail


def test_a_reported_robot_robot_contact_fails_a1_3(tmp_path):
    b = a1_bundle(tmp_path, contacts="hit")
    r = a1_core.grade(b)
    assert "A1.3" in r.failed
    assert _ids(r)["A1.3"].measured[
        "robot-robot contacts in first 10 steps"] == 1


# --- process facts ----------------------------------------------------------


def test_controller_starts_are_counted_from_the_captured_console(tmp_path):
    b = a1_bundle(tmp_path)
    assert b.process.behaviour_starts[CONTROLLER] == N_ROBOTS
    assert "NO log file" in b.process.log_source
    assert b.process.log_available is True


def test_missing_console_cannot_count_controller_starts(tmp_path):
    """Upstream writes NO log file, so a lost capture loses the evidence.

    ``behaviour_starts`` is ``None`` (could not count), never ``{}`` (counted,
    found none) -- and the missing citation is exactly what makes A1.4's second
    clause report its witness absent instead of failing silently.
    """
    b = a1_bundle(tmp_path, console="absent")
    assert b.process.behaviour_starts is None
    assert b.process.log_available is False
    assert "NOT AVAILABLE" in b.process.log_source
    r = a1_core.grade(b)
    assert "A1.4" in r.failed
    started = [f for f in _ids(r)["A1.4"].falsifiers
               if f.clause == "processes started"][0]
    assert not started.present
    exit_clause = [f for f in _ids(r)["A1.9"].falsifiers
                   if f.clause == "clean exit"][0]
    assert not exit_clause.present
    assert any("no source citation" in p for p in adapters.check_bundle(b))


def test_error_lines_come_from_the_console_and_fail_a1_9(tmp_path):
    b = a1_bundle(tmp_path, console="errors")
    assert b.process.error_lines == ["ERROR: Cannot load the world file."]
    r = a1_core.grade(b)
    assert "A1.9" in r.failed


def test_fatal_lines_are_recorded_but_not_counted_as_errors(tmp_path):
    """Deliberate parity with the OmniSim adapter's ERROR:-only rule."""
    b = a1_bundle(tmp_path, console="fatal")
    assert b.process.error_lines == []
    assert b.adapter_measurements["motion"]["fatal_lines"] == \
        ["FATAL: unrecoverable"]
    assert any("FATAL" in n for n in b.notes)


def test_finalize_is_evidenced_by_the_perf_log(tmp_path):
    """A non-empty --log-performance file proves worldClosed() ran."""
    b = a1_bundle(tmp_path)
    assert b.process.reached_finalize is True
    assert "worldClosed()" in b.process.finalize_evidence


def test_an_empty_perf_log_plus_an_incomplete_recorder_is_not_finalized(
        tmp_path):
    b = a1_bundle(tmp_path, perf=False, complete=False)
    assert b.process.reached_finalize is False
    assert "killed" in b.process.finalize_evidence
    r = a1_core.grade(b)
    assert "A1.9" in r.failed
    finalize = [f for f in _ids(r)["A1.9"].falsifiers
                if f.clause == "reached finalize"][0]
    assert finalize.present, "answering 'no' is still answering"


def test_no_finalize_channel_at_all_is_unanswered_not_answered_no(tmp_path):
    d = write_run(tmp_path / "webots")
    (d / "completion.json").unlink()
    (d / "perf.txt").unlink()
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              artifact=write_world(tmp_path / "w.wbt"),
                              webots_dir=d, robot_proto=PROTO)
    assert b.process.reached_finalize is None
    finalize = [f for f in _ids(a1_core.grade(b))["A1.9"].falsifiers
                if f.clause == "reached finalize"][0]
    assert not finalize.present


def test_void_controller_is_not_a_behaviour(tmp_path):
    """``void`` is upstream's own no-controller sentinel."""
    b = a1_bundle(tmp_path, controller="void", console="absent")
    assert all(x.behaviour is None for x in b.roster.bodies)
    assert all(x.behaviour_declared == "void" for x in b.roster.bodies)
    assert not any(x.controlled for x in b.roster.bodies)
    assert "A1.4" in a1_core.grade(b).failed


def test_a_timeout_is_not_a_clean_exit(tmp_path):
    b = a1_bundle(tmp_path, exit_code=124, timed_out=True)
    assert b.process.timed_out is True
    assert b.process.clean is False
    assert "A1.9" in a1_core.grade(b).failed


# --- attribution ------------------------------------------------------------


def test_ode_is_attributed_by_the_single_engine_invariant(tmp_path):
    b = a1_bundle(tmp_path)
    at = b.attribution
    assert at.backend == "ode" and at.solver == "ode"
    assert at.degraded is False
    assert "ONLY physics engine" in at.source
    assert "cyberbotics.com/doc/reference/worldinfo" in at.source
    assert at.extra["engine_invariant"] is True
    assert at.extra["webots_version"] == "R2025a"
    assert a1_core.grade(b).attribution["backend"] == "ode"


def test_an_ode_physics_plugin_is_recorded_on_the_attribution(tmp_path):
    b = a1_bundle(tmp_path, world_kw={"physics_plugin": "my_ode_plugin"})
    assert b.attribution.extra["ode_physics_plugin"] == "my_ode_plugin"
    assert "physics plugin" in b.attribution.source


def test_a_run_with_no_artifacts_is_unattributed(tmp_path):
    """'Webots is ODE' is true of the simulator, not of an empty directory.

    The verdict here is FAIL rather than INVALID only because the core
    short-circuits on the missing motion data before it reaches A1.10 -- the
    adapter's job is to hand up ``None``, which it does.
    """
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              artifact=write_world(tmp_path / "w.wbt"),
                              webots_dir=tmp_path / "nothing_here")
    assert b.attribution is None
    assert any("no upstream-Webots run artifacts found" in n for n in b.notes)
    assert a1_core.grade(b).outcome == FAIL


def test_pose_samples_with_no_process_evidence_are_invalid_not_failed(tmp_path):
    """A ``trajectory.json`` alone is not a physics run.

    This is the state the Webots arm is actually in while the two lanes are
    separate: a recorder dump exists, but nothing attests that a Webots process
    produced it. The motion assertions grade normally and the row is INVALID --
    an unattributed physics result is not a result (SPEC A1.10).
    """
    d = write_run(tmp_path / "webots")
    for nm in ("process.json", "completion.json", "console.log", "perf.txt"):
        (d / nm).unlink()
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              artifact=write_world(tmp_path / "w.wbt"),
                              webots_dir=d, robot_proto=PROTO)
    assert b.trajectory is not None and b.trajectory.xyz is not None
    assert b.process is None
    assert b.attribution is None
    r = a1_core.grade(b)
    assert r.outcome == INVALID
    assert r.outcome != FAIL
    assert any("unattributed" in n or "attribution" in n for n in r.notes)


def test_a_foreign_version_keeps_the_citation_and_says_so(tmp_path):
    d = write_run(tmp_path / "webots")
    doc = json.loads((d / "process.json").read_text(encoding="utf-8"))
    doc["webots_version"] = "R2023b"
    (d / "process.json").write_text(json.dumps(doc), encoding="utf-8")
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              artifact=write_world(tmp_path / "w.wbt"),
                              webots_dir=d, robot_proto=PROTO)
    assert "R2023b" in b.attribution.source
    assert "verified for R2025a" in b.attribution.source


# --- the frozen-t0 question -------------------------------------------------


def test_an_unsynchronized_scan_may_not_claim_t0(tmp_path):
    """Upstream free-runs in --mode=fast exactly as OmniSim's harness does."""
    b = a1_bundle(tmp_path, frozen=True, synchronization=False)
    assert b.t0.frozen is False
    assert any("claim is refused" in n for n in b.notes)
    problems = adapters.check_bundle(b)
    assert any("not marked frozen" in p for p in problems)
    aabb_clause = [f for f in _ids(a1_core.grade(b))["A1.3"].falsifiers
                   if f.clause == "AABB separation"][0]
    assert not aabb_clause.present


def test_an_unstated_frozen_flag_is_none_not_true(tmp_path):
    d = write_run(tmp_path / "webots")
    doc = json.loads((d / "roster.json").read_text(encoding="utf-8"))
    doc.pop("frozen")
    (d / "roster.json").write_text(json.dumps(doc), encoding="utf-8")
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              artifact=write_world(tmp_path / "w.wbt"),
                              webots_dir=d, robot_proto=PROTO)
    assert b.t0.frozen is None
    assert any("unanswered rather than assumed" in n for n in b.notes)


def test_missing_aabbs_leave_the_geometric_half_unevidenced(tmp_path):
    """Upstream has no bounds API: without a computed AABB there is nothing."""
    b = a1_bundle(tmp_path, with_aabb=False)
    assert all(not x.has_aabb for x in b.t0.bodies)
    assert any("no bounds query" in n for n in b.notes)
    assert any("carries no world-space AABB" in p
               for p in adapters.check_bundle(b))
    r = a1_core.grade(b)
    assert "A1.3" in r.failed
    assert _ids(r)["A1.3"].measured["robots without bounds"] == N_ROBOTS


# --- the contract self-check ------------------------------------------------


def test_check_bundle_catches_the_classic_adapter_mistakes(tmp_path):
    """One artifact set that trips two of the contract's rules at once."""
    b = a1_bundle(tmp_path, frozen=False, contacts="no_witness")
    problems = adapters.check_bundle(b)
    assert any("not marked frozen" in p for p in problems)
    assert any("vacuity witness" in p for p in problems)


def test_the_adapter_never_raises_on_junk(tmp_path):
    """Adapter rule 1: a broken simulator is a measurement, not an exception."""
    junk = tmp_path / "junk"
    junk.mkdir()
    for nm in ("roster.json", "contacts.json", "completion.json",
               "process.json", "trajectory.json"):
        (junk / nm).write_text("[]" if nm == "roster.json" else "{{{",
                               encoding="utf-8")
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              artifact=tmp_path / "does_not_exist.wbt",
                              webots_dir=junk)
    r = a1_core.grade(b)
    assert r.outcome in (FAIL, INVALID)
    assert b.identity.declared_count is None      # unread text != zero


def test_no_run_dir_at_all_is_survivable(tmp_path):
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky")
    assert b.artifact is None
    assert a1_core.grade(b).outcome == FAIL


# --- the reader -------------------------------------------------------------


def test_read_run_reports_what_is_missing(tmp_path):
    run = recording.read_run(tmp_path)
    assert not run.any_evidence
    assert set(run.missing) >= {"trajectory", "roster", "contacts", "console",
                                "perf"}
    assert "NO log file" in run.missing["console"]


def test_read_run_concatenates_stdout_and_stderr(tmp_path):
    (tmp_path / "stdout.log").write_text("INFO: a\n", encoding="utf-8")
    (tmp_path / "stderr.log").write_text("ERROR: b\n", encoding="utf-8")
    run = recording.read_run(tmp_path)
    assert "INFO: a" in run.console_text and "ERROR: b" in run.console_text


def test_to_arrays_keeps_non_finite_samples(tmp_path):
    """A NaN is a physical result on this arm (a rotation of exactly +-pi)."""
    arr = recording.to_arrays({"dt_s": DT, "bodies": [
        {"name": "a", "t": [0.0, DT], "xyz": [[0.0, 0.0, 0.0],
                                              [float("nan"), 0.0, 0.0]]}]})
    assert arr.fatal is None
    assert math.isnan(float(arr.xyz[0][1][0]))


def test_several_perf_logs_are_refused_rather_than_guessed(tmp_path):
    """A finalize witness borrowed from another run is worse than none.

    Observed on the live Webots lane: seven ``perf_<tag>.txt`` files from seven
    different runs in one ``out/`` directory.
    """
    out = tmp_path / "out"
    out.mkdir()
    (tmp_path / "trajectory.json").write_text(json.dumps(
        {"bodies": [{"name": "a", "t": [0.0], "xyz": [[0.0, 0.0, 0.0]]}]}),
        encoding="utf-8")
    for tag in ("alpha", "beta"):
        (out / ("perf_%s.txt" % tag)).write_text("TOT: 1500\n", encoding="utf-8")
    run = recording.read_run(tmp_path)
    assert run.perf_path is None
    assert "2 perf logs" in run.missing["perf"]
    assert run.perf_nonempty is None
    # naming it explicitly is always accepted
    named = recording.read_run(tmp_path, perf=out / "perf_beta.txt")
    assert named.perf_nonempty is True


def test_the_out_subdirectory_is_searched(tmp_path):
    out = tmp_path / "out"
    out.mkdir()
    (out / "console.log").write_text("INFO: x: Starting controller: y\n",
                                     encoding="utf-8")
    (out / "perf.txt").write_text("TOT\n", encoding="utf-8")
    run = recording.read_run(tmp_path)
    assert run.console_text is not None
    assert run.perf_nonempty is True


def test_an_explicit_trajectory_path_is_honoured(tmp_path):
    """The Webots lane may hand over one file from anywhere."""
    d = write_run(tmp_path / "webots")
    moved = tmp_path / "elsewhere.json"
    moved.write_text((d / "trajectory.json").read_text(encoding="utf-8"),
                     encoding="utf-8")
    (d / "trajectory.json").unlink()
    run = recording.read_run(d, trajectory=moved)
    assert run.trajectory is not None
    b = evidence.build_bundle(a1_core.TASK, robot_identity="husky",
                              artifact=write_world(tmp_path / "w.wbt"),
                              run=run, robot_proto=PROTO)
    assert a1_core.grade(b).outcome == PASS
