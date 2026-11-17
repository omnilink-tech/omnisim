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

"""L2's core and its loop detector, including every way to fake a pass.

Red evidence: each clause is exercised on an artifact built to FAIL it, not
only on one built to pass. A clause that has never been seen red is a clause
nobody has checked.
"""

from __future__ import annotations

import ast
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from loopbench.graders import l2_core, loop_trace          # noqa: E402

C = {"ARRIVE_TOL_M": 0.05, "SETTLE_DEADLINE_S": 9.0,
     "SETTLED_SPEED_MPS": 0.05, "MIN_LOOP_CYCLES": 2,
     "MAX_STEP_JUMP_M": 0.5, "MAX_SAMPLE_DT_S": 0.05,
     "MIN_GOAL_DISTANCE_M": 0.25}


def drive(final_err=0.02, settle=8.0, dt=0.016, n=600, jump_at=None):
    """A synthetic arrival: still, then moving, then still at the goal."""
    t, xy = [], []
    goal = (0.0, 5.0)
    move_start, move_end = 0.5, 0.5 + settle
    for i in range(n):
        now = i * dt
        if now < move_start:
            y = 0.0
        elif now < move_end:
            frac = (now - move_start) / max(settle, 1e-9)
            y = (5.0 - final_err) * frac
        else:
            y = 5.0 - final_err
        t.append(now)
        xy.append((0.0, y))
    if jump_at is not None:
        xy[jump_at] = (xy[jump_at][0], xy[jump_at][1] + 1.0)
    return t, xy, goal


def test_a_clean_converged_run_with_a_real_loop_passes():
    t, xy, goal = drive(final_err=0.02, settle=8.0)
    out, cl, nums = l2_core.grade(t=t, xy=xy, goal_xy=goal, cycles=3,
                                  measurements_seen=[0.31, 0.12, 0.02],
                                  constants=C)
    assert out == "PASS", {k: v.detail for k, v in cl.items()}
    assert nums["final_error_m"] < 0.05


def test_missing_the_tolerance_fails_L2_1():
    t, xy, goal = drive(final_err=0.30, settle=8.0)
    out, cl, _ = l2_core.grade(t=t, xy=xy, goal_xy=goal, cycles=3,
                               measurements_seen=[0.9, 0.5, 0.3], constants=C)
    assert out == "FAIL" and l2_core.first_failure(cl) == "L2.1"


def test_missing_the_deadline_fails_L2_2():
    t, xy, goal = drive(final_err=0.02, settle=13.5, n=1200)
    out, cl, _ = l2_core.grade(t=t, xy=xy, goal_xy=goal, cycles=3,
                               measurements_seen=[0.9, 0.5, 0.02], constants=C)
    assert out == "FAIL" and l2_core.first_failure(cl) == "L2.2"
    assert cl["L2.1"].ok           # it arrived; it was just slow


def test_a_teleport_fails_L2_3():
    t, xy, goal = drive(final_err=0.02, settle=8.0, jump_at=200)
    out, cl, _ = l2_core.grade(t=t, xy=xy, goal_xy=goal, cycles=3,
                               measurements_seen=[0.9, 0.02], constants=C)
    assert not cl["L2.3"].ok and "teleport" in cl["L2.3"].detail


def test_hitting_the_target_first_try_is_NOT_DISCRIMINATING_not_a_pass():
    """The rung's own falsifier. If luck reads as iteration the benchmark
    stops measuring what its name claims, so this must never be a PASS."""
    t, xy, goal = drive(final_err=0.02, settle=8.0)
    out, cl, _ = l2_core.grade(t=t, xy=xy, goal_xy=goal, cycles=0,
                               measurements_seen=[0.02], constants=C)
    assert out == "NOT_DISCRIMINATING"
    assert cl["L2.1"].ok and cl["L2.2"].ok and not cl["L2.4"].ok


def test_measuring_repeatedly_without_the_numbers_moving_fails_L2_5():
    """Re-running an unchanged artifact is not iterating."""
    t, xy, goal = drive(final_err=0.02, settle=8.0)
    out, cl, _ = l2_core.grade(t=t, xy=xy, goal_xy=goal, cycles=3,
                               measurements_seen=[0.02, 0.02, 0.02],
                               constants=C)
    assert out == "FAIL" and not cl["L2.5"].ok


def test_an_empty_run_is_an_ERROR_and_every_clause_is_vacuous():
    out, cl, _ = l2_core.grade(t=[], xy=[], goal_xy=(0.0, 5.0), cycles=5,
                               measurements_seen=[1, 2], constants=C)
    assert out == "ERROR"
    assert all(c.vacuous for c in cl.values())


def test_the_deadline_is_measured_from_motion_start_not_from_t0():
    """A column must not be charged for the length of its own settle window."""
    t, xy, goal = drive(final_err=0.02, settle=8.0)
    assert l2_core.motion_start(t, xy, 0.05) == pytest.approx(0.5, abs=0.05)
    assert l2_core.settled_time(t, xy, 0.05) == pytest.approx(8.0, abs=0.1)


def test_a_body_that_rolls_again_after_stopping_has_not_settled():
    t, xy, goal = drive(final_err=0.02, settle=4.0)
    # nudge it back into motion late in the record
    for i in range(500, 520):
        xy[i] = (xy[i][0], xy[i][1] + 0.004 * (i - 499))
    late = l2_core.settled_time(t, xy, 0.05)
    assert late > 6.0, "the LAST crossing is the honest reading, not the first"


# --- the loop detector ------------------------------------------------------


def _tx(tmp_path, records):
    p = tmp_path / "transcript.jsonl"
    p.write_text("\n".join(json.dumps(r) for r in records), encoding="utf-8")
    return p


def _use(tid, name, **inp):
    return {"message": {"content": [
        {"type": "tool_use", "id": tid, "name": name, "input": inp}]}}


def _res(tid, text):
    return {"message": {"content": [
        {"type": "tool_result", "tool_use_id": tid, "content": text}]}}


def test_observe_change_observe_counts_as_one_cycle(tmp_path):
    p = _tx(tmp_path, [
        _use("a", "Bash", command="run"), _res("a", "final error 0.31 m"),
        _use("b", "Edit", file_path="ctl.py"),
        _use("c", "Bash", command="run"), _res("c", "final error 0.12 m"),
    ])
    got = loop_trace.analyse(p)
    assert got["cycles"] == 1
    assert 0.31 in got["measurements"] and 0.12 in got["measurements"]


def test_two_runs_with_no_change_between_them_are_not_a_cycle(tmp_path):
    p = _tx(tmp_path, [
        _use("a", "Bash", command="run"), _res("a", "final error 0.31 m"),
        _use("b", "Bash", command="run"), _res("b", "final error 0.31 m"),
    ])
    assert loop_trace.analyse(p)["cycles"] == 0


def test_edits_without_any_measurement_are_not_cycles(tmp_path):
    """An agent that rewrites blindly has not closed a loop."""
    p = _tx(tmp_path, [
        _use("a", "Edit", file_path="ctl.py"),
        _use("b", "Edit", file_path="ctl.py"),
        _use("c", "Edit", file_path="ctl.py"),
    ])
    assert loop_trace.analyse(p)["cycles"] == 0


def test_the_agents_prose_is_never_evidence(tmp_path):
    """Claiming a number is not measuring one."""
    p = _tx(tmp_path, [
        {"message": {"content": [
            {"type": "text",
             "text": "I ran it and the final error is 0.02 m after 8.0 s."}]}},
        _use("b", "Edit", file_path="ctl.py"),
        {"message": {"content": [
            {"type": "text", "text": "Now the error is 0.01 m. Done."}]}},
    ])
    got = loop_trace.analyse(p)
    assert got["cycles"] == 0 and got["measurements"] == []


def test_a_shell_only_column_can_close_the_loop_just_as_well(tmp_path):
    """Column neutrality, as a test. The detector must not require OUR
    surface: a measurement printed by a script counts exactly as much as one
    returned by a service, or the benchmark defines its own conclusion."""
    p = _tx(tmp_path, [
        _use("a", "Bash", command="python sim.py"),
        _res("a", "arrived: dist=0.3100 settle=12.40"),
        _use("b", "Write", file_path="sim.py"),
        _use("c", "Bash", command="python sim.py"),
        _res("c", "arrived: dist=0.0400 settle=8.90"),
        _use("d", "Write", file_path="sim.py"),
        _use("e", "Bash", command="python sim.py"),
        _res("e", "arrived: dist=0.0210 settle=8.10"),
    ])
    got = loop_trace.analyse(p)
    assert got["cycles"] == 2
    assert len({round(m, 4) for m in got["measurements"]}) > 1


def test_a_truncated_transcript_still_yields_what_it_can(tmp_path):
    p = tmp_path / "t.jsonl"
    p.write_text(json.dumps(_use("a", "Bash", command="run")) + "\n"
                 + json.dumps(_res("a", "final error 0.31 m")) + "\n"
                 + '{"message": {"content": [{"type": "tool_',
                 encoding="utf-8")
    got = loop_trace.analyse(p)
    assert got["measurements"] == [0.31] or 0.31 in got["measurements"]


# --- the neutrality guard ---------------------------------------------------


SIM_TOKENS = ("omnisim", "webots", "mujoco", "isaac", "gazebo", "wbt", "urdf",
              "mjcf", "vrml", "newton", "ode", "harness")


def test_the_core_names_no_simulator_in_code():
    """Docstrings may explain the boundary; code may not cross it."""
    path = Path(l2_core.__file__)
    tree = ast.parse(path.read_text(encoding="utf-8"))
    docs = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef)):
            body = getattr(node, "body", None) or []
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                docs.add(id(body[0].value))
    for node in ast.walk(tree):
        text = None
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if id(node) not in docs:
                text = node.value
        elif isinstance(node, ast.Name):
            text = node.id
        elif isinstance(node, ast.Attribute):
            text = node.attr
        if not text:
            continue
        for token in SIM_TOKENS:
            assert token not in text.lower(), (
                "l2_core mentions %r in code: %r" % (token, text))


# --- the container must actually exist (2026-08-02, L2's first cell) --------


def test_every_task_that_declares_a_container_actually_ships_one():
    """RED before the fix, and it cost a whole cell.

    L2's meta named ``container/husky_description`` and the task directory had
    no ``container/`` at all, so staging produced an EMPTY one. The agent found
    nothing, went and copied the description out of the benchmark's own tree,
    and then solved the task in a different simulator than the column under
    test. Nothing about that cell measured what it claimed to.

    A declared description directory must resolve to real files -- shipped, or
    shared from a named sibling.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from ladder import tasks as ladder_tasks

    problems = []
    for tid, task in sorted(ladder_tasks.discover().items()):
        meta_c = task.meta.get("container") or {}
        if not meta_c.get("description_dir") and not meta_c.get("files"):
            continue
        if not task.container_dir.is_dir():
            problems.append("%s: container_dir %s does not exist"
                            % (tid, task.container_dir))
            continue
        missing = [str(f) for f in task.container_files if not f.is_file()]
        if missing:
            problems.append("%s: %d declared file(s) missing, first %s"
                            % (tid, len(missing), missing[0]))
    assert not problems, "; ".join(problems)


def test_a_task_sharing_a_container_resolves_to_the_owners_files():
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from ladder import tasks as ladder_tasks

    l2 = ladder_tasks.get("L2_converge")
    t1 = ladder_tasks.get("T1_arrive")
    assert l2.container_dir == t1.container_dir, (
        "two rungs posed on the same robot must be posed on the "
        "byte-identical robot, or a row difference could be the asset")
    assert all(f.is_file() for f in l2.container_files)
