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

"""Unit tests for the C1 ``parse_error_fix`` grader core. **No simulator.**

Same design as ``test_neutral_core.py``: every fixture is a synthetic
evidence bundle built from numpy, shaped after **measured** engine behaviour
(2026-07-31, `python -m omnisim run-headless ... --duration 5`, this tree's
binary):

  * the untouched world FAILs to load (``error: Expected field name or '}',
    found 'DEF'`` + ``Failed to load due to syntax error(s)``);
  * fixing only the brace leaves ``ERROR: Missing declaration for 'Soild',
    unknown node.`` + ``Skipped unknown 'Soild' node or PROTO.`` -- and the
    engine then loads the REST of the world and steps it to completion, so
    the half-fix is a loads-with-error-lines run with a short inventory, not
    a dead run;
  * fixing only the typo aborts the parse exactly like the untouched world;
  * the full fix runs with 0 errors.

The red-map test at the bottom is the §5.5 obligation: every negative
fixture in ``agents/c1_fixtures.RED_MAP`` is observed failing exactly the
assertions it is on record as targeting.

    pytest tests/benchmarks/agentbench/graders/test_c1_core.py -q
"""

from __future__ import annotations

import ast
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench import adapters                                   # noqa: E402
from agentbench.agents import c1_fixtures                         # noqa: E402
from agentbench.graders import c1, c1_core                        # noqa: E402
from agentbench import tasks as tasks_mod                         # noqa: E402
from agentbench.graders.evidence import (                         # noqa: E402
    Body, BodyInventory, EngineAttribution, EvidenceBundle, ProcessFacts,
    Trajectory)
from agentbench.graders.verdict import FAIL, INVALID, PASS        # noqa: E402

DT = 0.032

TASK_DIR = (Path(__file__).resolve().parents[1]
            / "tasks" / "C1_parse_error_fix")
INITIAL_WORLD = TASK_DIR / "initial" / "parse_error.wbt"

# The two error shapes the engine was measured to produce (see module doc).
_SYNTAX_ERRORS = [
    "ERROR: 'parse_error.wbt':93:1: error: Expected field name or '}', "
    "found 'DEF'.",
    "ERROR: 'parse_error.wbt': Failed to load due to syntax error(s).",
]
_SKIP_ERRORS = [
    "ERROR: Missing declaration for 'Soild', unknown node.",
    "ERROR: 'parse_error.wbt':94:14: error: Skipped unknown 'Soild' node or "
    "PROTO.",
]


# --- synthetic fixtures -----------------------------------------------------


def _solid(i, name):
    x = 2.0 * i
    return Body(body_id="#%d" % (10 + i), name=name, kind="Solid",
                position=(x, 1.5, 0.11),
                aabb_min=(x - 0.6, 1.1, 0.05), aabb_max=(x + 0.6, 1.9, 0.17),
                robot_class=False, identity_evidence="synthetic fixture")


def _robot(i, name):
    return Body(body_id="#%d" % (40 + i), name=name, kind="Robot",
                position=(0.0, 0.0, 0.5),
                aabb_min=(-0.2, -0.2, 0.3), aabb_max=(0.2, 0.2, 0.7),
                n_joints=0, behaviour=None, behaviour_declared="<none>",
                dynamic=True, robot_class=True,
                identity_evidence="synthetic fixture")


def _settle_track(n, *, z0=0.5, z_rest=0.25):
    t = np.arange(n) * DT
    fall = np.clip(1.0 - np.exp(-t / 0.25), 0.0, 1.0)
    z = z0 - (z0 - z_rest) * fall
    z[t > 2.0] = z_rest
    return np.stack([np.zeros(n), np.zeros(n), z], axis=1)


def c1_bundle(*, artifact="/synthetic/parse_error.wbt",
              t0_names=("floor", "pallet_a", "pallet_b", "probe_bot"),
              robot_names=("probe_bot",), load_failed=False,
              error_lines=(), driver_completed=True, exit_code=0,
              duration_s=5.0, recorded_s=None, attribution="ok",
              t0_frozen=True, log_available=True):
    """A C1 evidence bundle from numpy. Defaults describe a PASSing run."""
    robots = [_robot(i, n) for i, n in enumerate(robot_names)]
    solids = [_solid(i, n) for i, n in enumerate(t0_names)
              if n not in set(robot_names)]
    attrib = (EngineAttribution(backend="synthetic", solver="synthetic-lcp",
                                source="a synthetic fixture, not a run")
              if attribution == "ok" else None)

    if load_failed:
        return EvidenceBundle(
            task=c1_core.TASK, sim="synthetic", adapter="tests",
            artifact=artifact,
            roster=BodyInventory(source="synthetic roster", error="no load"),
            t0=BodyInventory(source="synthetic frozen scan",
                             error="no t=0 scan"),
            trajectory=None,
            motion_error="the world failed to load (%d error lines)"
                         % len(error_lines),
            process=ProcessFacts(exit_code=exit_code, timed_out=False,
                                 error_lines=list(error_lines),
                                 log_available=log_available,
                                 log_source="synthetic error stream",
                                 start_source="synthetic process table",
                                 driver_completed=False,
                                 reached_finalize=False),
            attribution=attrib)

    n = int(round(duration_s / DT)) + 1
    t = np.arange(n) * DT
    xyz = np.stack([_settle_track(n) for _ in robots], axis=0)
    bodies = [b for b in robots] + solids
    # keep t0 ordered as named, robots in place
    by_name = {b.name: b for b in bodies}
    t0_bodies = [by_name[nm] for nm in t0_names if nm in by_name]
    return EvidenceBundle(
        task=c1_core.TASK, sim="synthetic", adapter="tests",
        artifact=artifact,
        roster=BodyInventory(bodies=robots, frozen=True, t_s=0.0,
                             source="synthetic roster"),
        t0=BodyInventory(bodies=t0_bodies, frozen=t0_frozen, t_s=0.0,
                         source="synthetic frozen scan"),
        trajectory=Trajectory(body_ids=[b.body_id for b in robots], t=t,
                              xyz=xyz, dt_s=DT,
                              recorded_s=(float(t[-1]) if recorded_s is None
                                          else recorded_s),
                              complete=True, source="synthetic samples"),
        process=ProcessFacts(exit_code=exit_code, timed_out=False,
                             error_lines=list(error_lines),
                             log_available=log_available,
                             log_source="synthetic error stream",
                             start_source="synthetic process table",
                             driver_completed=driver_completed,
                             reached_finalize=driver_completed),
        attribution=attrib)


def _ids(verdict):
    return {a.id: a for a in verdict.assertions}


# --- the fixtures' bundle shapes, keyed like RED_MAP ------------------------
#
# Each entry mirrors what the engine was MEASURED to produce for that
# fixture's artifact (module docstring), in neutral evidence terms.

FIXTURE_BUNDLES = {
    # untouched broken world: hard parse abort, nothing measurable
    "null": lambda: c1_bundle(load_failed=True, error_lines=_SYNTAX_ERRORS),
    # pallets deleted: loads clean, steps clean, inventory short by two
    "amputate": lambda: c1_bundle(t0_names=("floor", "probe_bot")),
    # trivial replacement: loads clean, steps clean, only the floor name and
    # a differently named robot survive
    "wholesale": lambda: c1_bundle(t0_names=("floor", "bot"),
                                   robot_names=("bot",)),
    # brace fixed, 'Soild' left: the engine SKIPS the node (2 ERROR lines)
    # and steps the rest -- so pallet_b is missing and the log is dirty
    "half_fix": lambda: c1_bundle(
        t0_names=("floor", "pallet_a", "probe_bot"),
        error_lines=_SKIP_ERRORS),
    # typo fixed, brace left: hard parse abort, same as untouched
    "half_fix_type": lambda: c1_bundle(load_failed=True,
                                       error_lines=_SYNTAX_ERRORS),
}


# --- the passing run --------------------------------------------------------


def test_c1_core_passes_a_repaired_world():
    r = c1_core.grade(c1_bundle())
    assert r.outcome == PASS, r.summary()
    assert len(r.assertions) == 3
    assert r.progress == 4
    a = _ids(r)
    assert a["C1.2"].measured["intended bodies missing"] == []
    assert a["C1.3"].measured["all poses finite"] is True


def test_c1_core_thresholds_are_the_task_numbers():
    """A drift in any of these is a task-contract violation, not a refactor."""
    assert c1_core.MIN_RECORDED_S == 4.0
    assert c1_core.MAX_ABS_COORD_M == 1000.0
    assert c1_core.EXPECTED_BODIES == ("floor", "pallet_a", "pallet_b",
                                       "probe_bot")
    assert c1_core.ROBOT_NAME == "probe_bot"


def test_c1_core_no_artifact_scores_zero_of_three():
    r = c1_core.grade(c1_bundle(artifact=None))
    assert r.outcome == FAIL
    assert len(r.failed) == 3
    assert r.progress == 0


# --- the negative fixtures, one by one --------------------------------------


def test_c1_core_untouched_broken_world_fails_everything():
    """The null: the world never loads, so all three assertions are red."""
    r = c1_core.grade(FIXTURE_BUNDLES["null"]())
    assert r.outcome == FAIL
    assert set(r.failed) == {"C1.1", "C1.2", "C1.3"}
    assert r.progress == 1                      # artifact_invalid
    assert "syntax error" in _ids(r)["C1.1"].detail


def test_c1_core_amputation_fails_only_the_inventory():
    """Deleting the broken subtrees loads clean -- and must FAIL anyway."""
    r = c1_core.grade(FIXTURE_BUNDLES["amputate"]())
    assert r.outcome == FAIL
    assert r.failed == ["C1.2"], r.summary()
    a = _ids(r)
    assert a["C1.1"].ok is True                # the naive load test passes
    assert a["C1.3"].ok is True                # it even steps cleanly
    assert a["C1.2"].measured["intended bodies missing"] == [
        "pallet_a", "pallet_b"]
    assert r.progress == 3                      # artifact_runs


def test_c1_core_wholesale_replacement_fails_only_the_inventory():
    """A trivial valid replacement world must FAIL via C1.2 (the task rule)."""
    r = c1_core.grade(FIXTURE_BUNDLES["wholesale"]())
    assert r.outcome == FAIL
    assert r.failed == ["C1.2"], r.summary()
    assert _ids(r)["C1.2"].measured["intended bodies missing"] == [
        "pallet_a", "pallet_b", "probe_bot"]
    assert "bot" in _ids(r)["C1.2"].measured[
        "bodies beyond the intended roster"]


def test_c1_core_half_fix_fails_load_and_inventory_but_still_steps():
    """Brace fixed, 'Soild' left: the measured skip case.

    The engine logs two ERROR lines, drops the node, and steps the rest --
    so C1.1 and C1.2 are red while C1.3 is genuinely green. A core that
    failed everything on a dirty log could not represent this run.
    """
    r = c1_core.grade(FIXTURE_BUNDLES["half_fix"]())
    assert r.outcome == FAIL
    assert set(r.failed) == {"C1.1", "C1.2"}, r.summary()
    a = _ids(r)
    assert a["C1.3"].ok is True
    assert a["C1.1"].measured["error lines"] == 2
    assert a["C1.2"].measured["intended bodies missing"] == ["pallet_b"]


def test_c1_core_half_fix_type_fails_everything():
    """Typo fixed, brace left: still a hard parse abort."""
    r = c1_core.grade(FIXTURE_BUNDLES["half_fix_type"]())
    assert set(r.failed) == {"C1.1", "C1.2", "C1.3"}


def test_c1_red_map_matches_measured_fixture_behaviour():
    """§5.5: every fixture is OBSERVED failing exactly what it targets."""
    assert set(FIXTURE_BUNDLES) == set(c1_fixtures.RED_MAP)
    for name, make in FIXTURE_BUNDLES.items():
        r = c1_core.grade(make())
        assert r.outcome == FAIL, name
        assert set(r.failed) == c1_fixtures.RED_MAP[name], (
            "%s: failed %s, RED_MAP says %s"
            % (name, sorted(r.failed), sorted(c1_fixtures.RED_MAP[name])))
    # ...and every assertion has targeted (non-null) red evidence
    targeted = set()
    for name, reds in c1_fixtures.RED_MAP.items():
        if name != "null":
            targeted |= reds
    assert targeted == {"C1.1", "C1.2", "C1.3"}


# --- INVALID vs FAIL --------------------------------------------------------


def test_c1_core_missing_attribution_is_invalid_when_physics_ran():
    r = c1_core.grade(c1_bundle(attribution=None))
    assert r.outcome == INVALID
    assert any("attribution" in n for n in r.notes)


def test_c1_core_unloaded_world_without_attribution_stays_fail():
    """A world that never loaded produced no physics result to attribute --
    the null must stay a FAIL, not become INVALID."""
    r = c1_core.grade(c1_bundle(load_failed=True,
                                error_lines=_SYNTAX_ERRORS,
                                attribution=None))
    assert r.outcome == FAIL


# --- C1.3's own negatives ---------------------------------------------------


def test_c1_core_nan_pose_fails_stepping():
    b = c1_bundle()
    b.trajectory.xyz[0, 10, 2] = np.nan
    r = c1_core.grade(b)
    assert "C1.3" in r.failed
    assert _ids(r)["C1.3"].measured["all poses finite"] is False


def test_c1_core_runaway_pose_fails_stepping():
    """The lane3 escape shape: a body reading z = -2251 did not 'load and
    step' in any sense the task means."""
    b = c1_bundle()
    b.trajectory.xyz[0, -1, 2] = -2251.0
    r = c1_core.grade(b)
    assert "C1.3" in r.failed
    assert _ids(r)["C1.3"].measured["max |coordinate| (m)"] > 1000.0


def test_c1_core_short_window_fails_stepping():
    r = c1_core.grade(c1_bundle(recorded_s=2.0))
    assert "C1.3" in r.failed
    assert _ids(r)["C1.3"].measured["recorded window (s)"] == 2.0


# --- vacuity (verdict.py mechanics) -----------------------------------------


def test_c1_core_vacuity_fires_on_a_free_running_inventory():
    """All names present but read while the world free-ran: C1.2 'passes'
    and is flagged vacuous rather than silently counting."""
    r = c1_core.grade(c1_bundle(t0_frozen=False))
    a = _ids(r)["C1.2"]
    assert a.ok is True
    assert a.vacuous
    assert r.vacuous["C1.2"] == ["intended inventory"]
    assert "~vacuous" in r.summary()


def test_c1_core_vacuity_fires_when_the_error_stream_was_never_read():
    """Exit 0 with an unread log is not a clean run -- it is an unfalsifiable
    claim, and the verdict must say so."""
    r = c1_core.grade(c1_bundle(log_available=False))
    a = _ids(r)["C1.1"]
    assert a.ok is True                        # nothing contradicted it...
    assert a.vacuous                           # ...and nothing could have
    assert "C1.1" in r.vacuous


def test_every_assertion_declares_how_it_could_fail():
    for make in (lambda: c1_bundle(),
                 FIXTURE_BUNDLES["null"], FIXTURE_BUNDLES["half_fix"]):
        r = c1_core.grade(make())
        for a in r.assertions:
            assert a.falsifiers, "%s declares no way to fail" % a.id
            for f in a.falsifiers:
                assert f.how_to_fail and f.witness


# --- the task files agree with the core -------------------------------------


def test_reference_roster_file_matches_the_core_constant():
    data = json.loads((TASK_DIR / "reference_roster.json")
                      .read_text(encoding="utf-8"))
    assert tuple(data["intended_bodies"]) == c1_core.EXPECTED_BODIES
    assert data["robot"] == c1_core.ROBOT_NAME
    assert c1.load_reference_roster() == c1_core.EXPECTED_BODIES
    # ...and the fallback never awards a pass through a missing file
    assert c1.load_reference_roster(TASK_DIR / "no_such_file.json") \
        == c1_core.EXPECTED_BODIES


def test_meta_json_agrees_with_the_grader():
    meta = json.loads((TASK_DIR / "meta.json").read_text(encoding="utf-8"))
    assert meta["id"] == c1_core.TASK
    assert meta["grader"] == "agentbench.graders.c1"
    # SPEC 2.4: timeout = min(3 x par, TASK_HARD_CEILING_S). C1 is one of
    # the tasks the cost ceiling truncates (1.88x par, not 3x).
    assert meta["timeout_s"] == min(3 * meta["par_s"],
                                    tasks_mod.TASK_HARD_CEILING_S)
    assert meta["budget"]["truncated_by_ceiling"] is (
        3 * meta["par_s"] > tasks_mod.TASK_HARD_CEILING_S)
    c = meta["constants"]
    assert c["MIN_RECORDED_S"] == c1_core.MIN_RECORDED_S
    assert c["MAX_ABS_COORD_M"] == c1_core.MAX_ABS_COORD_M
    assert tuple(c["EXPECTED_BODIES"]) == c1_core.EXPECTED_BODIES
    assert c["ROBOT_NAME"] == c1_core.ROBOT_NAME
    # the graded window must be able to satisfy the recorded-time floor
    assert meta["phases"]["standalone"]["duration_s"] > c1_core.MIN_RECORDED_S
    # the named solids the frozen scan must include
    assert set(meta["phases"]["standalone"]["solids"]) == {
        "floor", "pallet_a", "pallet_b"}


# --- the shipped broken world and the fixtures' text ops --------------------


def test_the_shipped_world_carries_exactly_the_two_defects():
    text = INITIAL_WORLD.read_text(encoding="utf-8")
    assert text.count("{") == text.count("}") + 1      # one unclosed brace
    assert text.count(c1_fixtures.UNDEFINED_TYPE) == 1  # one 'Soild'
    assert text.count(c1_fixtures.BRACE_ANCHOR) == 1   # the repair anchor
    # the intended roster is authored in the file
    for name in c1_core.EXPECTED_BODIES:
        assert '"%s"' % name in text, name


def test_fixture_edits_apply_to_the_shipped_initial_world():
    text = INITIAL_WORLD.read_text(encoding="utf-8")

    fixed, ok = c1_fixtures.apply_full_fix(text)
    assert ok
    assert fixed.count("{") == fixed.count("}")
    assert c1_fixtures.UNDEFINED_TYPE not in fixed

    half, ok = c1_fixtures.fix_brace(text)
    assert ok
    assert half.count("{") == half.count("}")          # braces now balance...
    assert c1_fixtures.UNDEFINED_TYPE in half          # ...the typo remains

    half_t, ok = c1_fixtures.fix_node_type(text)
    assert ok
    assert half_t.count("{") == half_t.count("}") + 1  # brace still missing

    cut, ok = c1_fixtures.amputate(text)
    assert ok
    assert cut.count("{") == cut.count("}")            # amputation parses
    assert "pallet_a" not in cut and "pallet_b" not in cut
    assert '"probe_bot"' in cut and '"floor"' in cut   # rest of scene intact

    w = c1_fixtures.WHOLESALE_WORLD
    assert w.count("{") == w.count("}")
    assert "pallet_a" not in w and "probe_bot" not in w
    assert 'defaultPhysicsBackend "ode"' in w          # stays attributable


def test_the_prompt_and_world_header_do_not_name_the_defects():
    """The 4e5cbdc4 rule: the task must test diagnosis, not reading the
    answer out of the prompt or the file header."""
    prompt = (TASK_DIR / "prompt.txt").read_text(encoding="utf-8")
    assert prompt.strip() == "This world will not load. Fix it."
    low = prompt.lower()
    for word in ("brace", "soild", "node type", "pallet", "solid", "typo"):
        assert word not in low, word
    comments = "\n".join(
        ln for ln in INITIAL_WORLD.read_text(encoding="utf-8").splitlines()
        if ln.lstrip().startswith("#")).lower()
    for word in ("brace", "soild", "typo", "undefined", "unbalanced",
                 "unknown node"):
        assert word not in comments, word


# --- the fixtures registry shape --------------------------------------------


def test_c1_registry_is_registry_shaped():
    reg = c1_fixtures.REGISTRY
    names = set()
    for key, entry in reg.items():
        task_id, agent_name = key
        assert task_id == "C1_parse_error_fix"
        names.add(agent_name)
        assert callable(entry["fn"])
        assert entry["expect_pass"] in (True, False, None)
        ef = entry["expect_failures"]
        assert ef is None or (isinstance(ef, set)
                              and ef <= {"C1.1", "C1.2", "C1.3"})
    assert reg[("C1_parse_error_fix", "oracle")]["expect_pass"] is True
    # every red-map fixture is registered (the null under its package name)
    assert set(c1_fixtures.RED_MAP) <= names
    # the null follows the package convention: reds everything, credited
    # with nothing
    assert reg[("C1_parse_error_fix", "null")]["expect_failures"] is None


# --- the sim-agnostic boundary ----------------------------------------------

_SIM_TOKENS = ("urdfrobot", "husky.urdf", ".wbt", "newton", "sidecar",
               "physicsbackend", "gettypename", "boundingobject", "proto",
               "sdf", "usd", "mjcf", "rclpy", "harness", "recorder")


def _code_strings_and_names(path):
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef,
                             ast.ClassDef)):
            body = getattr(node, "body", None) or []
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                docstrings.add(id(body[0].value))
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if id(node) not in docstrings:
                out.append(node.value)
        elif isinstance(node, ast.Name):
            out.append(node.id)
        elif isinstance(node, ast.Attribute):
            out.append(node.attr)
    return out


def test_c1_core_contains_no_simulator_specific_vocabulary():
    for text in _code_strings_and_names(c1_core.__file__):
        low = text.lower()
        for token in _SIM_TOKENS:
            assert token not in low, (
                "c1_core.py mentions %r in code (not a docstring): %r"
                % (token, text))


def test_the_contract_self_check_accepts_the_synthetic_bundle():
    assert adapters.check_bundle(c1_bundle()) == []
