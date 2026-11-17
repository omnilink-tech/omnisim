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

"""The drone's `stop` verb must halt, and must not read `reset`'s locals.

WHY THIS EXISTS -- do not delete it as pedantry.

Found at HEAD on 2026-09-22, shipped: the `action == "stop"` branch of the
Mavic bridge's `POST /action` assigned

    state.x = reset_x   (and y / z / yaw)

copied from the `action == "reset"` branch TWENTY LINES BELOW IT, which is
where those four names are assigned. Python binds them per call, so at the
moment `stop` ran they did not exist and the handler raised
``UnboundLocalError`` -- before clearing a single target. `stop` is the one
verb PROTOCOL.md 5.5 says a caller may rely on with no preconditions, so it
failed in precisely the situation it exists for, and it did so on a shipped
demo rather than in a corner.

The teleport was also wrong on its own terms. `state.x/y/z/yaw` is the
PUBLISHED pose, rewritten every tick from the real body. `reset` may publish a
requested pose ahead of its own teleport because it has one; a stop requests no
pose at all. Turning a halt into a homing move is the error
docs/guide/omnilink-sim-to-real.md names outright.

This test reads the controller's SOURCE rather than importing it, because the
module needs the simulator's `omnisim` controller library at import time and
this lane is engine-free. An AST check is also the right shape for the defect:
the bug is a name read in the wrong scope, which is a property of the syntax.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

BRIDGE = (pathlib.Path(__file__).resolve().parents[3]
          / "projects" / "samples" / "demos" / "controllers"
          / "mavic_omnilink_bridge" / "mavic_omnilink_bridge.py")

# The names `reset` binds. None of them may be READ by the `stop` branch.
RESET_LOCALS = {"reset_x", "reset_y", "reset_z", "reset_yaw"}


def _action_branch(tree: ast.AST, action: str) -> ast.If:
    """The `if action == "<action>":` statement, as an AST node."""
    for node in ast.walk(tree):
        if not isinstance(node, ast.If):
            continue
        test = node.test
        if (isinstance(test, ast.Compare)
                and isinstance(test.left, ast.Name)
                and test.left.id == "action"
                and len(test.comparators) == 1
                and isinstance(test.comparators[0], ast.Constant)
                and test.comparators[0].value == action):
            return node
    raise AssertionError(f"no `if action == {action!r}:` branch found in {BRIDGE.name}")


@pytest.fixture(scope="module")
def tree() -> ast.AST:
    return ast.parse(BRIDGE.read_text(encoding="utf-8"), filename=str(BRIDGE))


def test_stop_does_not_read_names_that_only_reset_assigns(tree: ast.AST) -> None:
    """The regression itself: a read of `reset_*` from inside `stop`."""
    stop = _action_branch(tree, "stop")
    read = {n.id for n in ast.walk(stop)
            if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
    leaked = sorted(read & RESET_LOCALS)
    assert not leaked, (
        f"the drone's `stop` branch reads {leaked}, which only the `reset` "
        f"branch assigns -- and `reset` is defined AFTER `stop` in the handler, "
        f"so POST /action {{'action': 'stop'}} raises UnboundLocalError and the "
        f"aircraft is never halted. See this file's docstring; this exact "
        f"defect shipped and was found on 2026-09-22."
    )


def test_stop_does_not_publish_a_pose(tree: ast.AST) -> None:
    """A halt is not a homing move: `stop` must not write the published pose."""
    stop = _action_branch(tree, "stop")
    written = set()
    for node in ast.walk(stop):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if (isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "state"):
                written.add(target.attr)
    pose = sorted(written & {"x", "y", "z", "yaw"})
    assert not pose, (
        f"the drone's `stop` branch assigns the published pose {pose}. "
        f"`state.x/y/z/yaw` is rewritten every tick from the real body; a stop "
        f"requests no pose and must let the aircraft come to rest where it is. "
        f"Equating a halt with homing is the error the sim-to-real guide names."
    )


def test_stop_still_actually_halts(tree: ast.AST) -> None:
    """Guard against 'fixing' the crash by gutting the verb.

    Removing the offending lines must not remove the halt: `stop` has to drop
    the position targets and leave the aircraft idle, or it would return
    `halted_at` for something that never halted -- the fabrication class this
    whole surface is built to prevent.
    """
    stop = _action_branch(tree, "stop")
    cleared = set()
    for node in ast.walk(stop):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if (isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "state"):
                cleared.add(target.attr)
    for required in ("target_x", "target_y", "target_yaw", "mode"):
        assert required in cleared, (
            f"the drone's `stop` branch no longer sets `state.{required}`. "
            f"It answers `halted_at`, so it must actually halt."
        )
