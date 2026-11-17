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

"""A clean world shutdown must flush the action journal.

WHY THIS EXISTS. The relay syncs the journal on a 30 s beat and `close()`
flushes what the last beat has not sent. Until 2026-09-22 no shipped bridge
called `close()` at all -- `git grep` found it only in tests -- so a clean
world close or reload lost up to a beat of the newest actions, the ones an
operator is most likely to ask about next.

The bridges import `omnisim` at module scope and cannot be imported off a
running engine, so the call sites are checked in their SOURCE, the same way
`test_mavic_stop_verb.py` and `test_step_budget.py` do.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

from omnisim_bridges.bridge_base import close_relay

ROOT = pathlib.Path(__file__).resolve().parents[3]
CONTROLLERS = ROOT / "projects" / "samples" / "demos" / "controllers"
BRIDGES = [
    CONTROLLERS / "omnilink_mobile_bridge" / "omnilink_mobile_bridge.py",
    CONTROLLERS / "omnilink_arm_bridge" / "omnilink_arm_bridge.py",
    CONTROLLERS / "omnilink_quadruped_bridge" / "omnilink_quadruped_bridge.py",
    CONTROLLERS / "mavic_omnilink_bridge" / "mavic_omnilink_bridge.py",
]


class _Relay:
    def __init__(self, boom: bool = False) -> None:
        self.closed = 0
        self.boom = boom

    def close(self) -> None:
        self.closed += 1
        if self.boom:
            raise RuntimeError("platform unreachable")


def test_close_relay_calls_close() -> None:
    r = _Relay()
    close_relay(r)
    assert r.closed == 1


def test_close_relay_tolerates_no_relay_and_no_close() -> None:
    close_relay(None)          # a keyless bridge has no relay at all
    close_relay(object())      # a relay from an older package has no close()


def test_close_relay_never_raises_at_shutdown() -> None:
    r = _Relay(boom=True)
    close_relay(r)             # shutdown must never raise
    assert r.closed == 1


def _main_loop_and_close(path: pathlib.Path):
    """(line of the last `while ... step(...)` loop in main, line of close_relay call)."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    main = next((n for n in tree.body
                 if isinstance(n, ast.FunctionDef) and n.name == "main"), None)
    assert main is not None, f"{path.name}: no main()"
    # A stepping loop is a `while` that advances the world. Lockstep changed
    # the shape: three bridges now loop on `while True:` and break when
    # `hold.step_or_hold(...)` returns -1, so the condition no longer names a
    # step. Recognise the loop by what it CALLS, not by how it is written.
    def steps(node: ast.AST) -> bool:
        return any(isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
                   and c.func.attr in ("step", "step_or_hold")
                   for c in ast.walk(node))
    loops = [n for n in ast.walk(main) if isinstance(n, ast.While) and steps(n)]
    calls = [n for n in ast.walk(main) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == "close_relay"]
    return loops, calls


@pytest.mark.parametrize("path", BRIDGES, ids=lambda p: p.parent.name)
def test_every_bridge_closes_its_relay_after_its_main_loop(path: pathlib.Path) -> None:
    loops, calls = _main_loop_and_close(path)
    assert loops, f"{path.name}: main() has no stepping loop to close after"
    assert calls, (f"{path.name}: main() never calls close_relay, so a clean "
                   f"world shutdown loses up to 30 s of the action journal")
    last_loop_end = max(l.end_lineno for l in loops)
    assert any(c.lineno > last_loop_end for c in calls), (
        f"{path.name}: close_relay must run AFTER the main loop exits, "
        f"not inside it -- inside, it would close the relay on the first tick")
