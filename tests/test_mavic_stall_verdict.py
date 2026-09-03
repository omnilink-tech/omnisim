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

"""The Mavic bridge's stall verdict (public issue #14).

A flight campaign measured two flights wedged against an obstacle for 212 s and
40 s while the bridge reported `mode=goto` and `fault=None`. `stall_verdict` is
the pure half of the fix, so it is testable without an engine: it decides, per
tick, whether a goto has stopped making progress toward its target.
"""
import importlib.util
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
BRIDGE = (REPO_ROOT / "projects" / "samples" / "demos" / "controllers"
          / "mavic_omnilink_bridge" / "mavic_omnilink_bridge.py")


def _load_verdict():
    """Import the bridge WITHOUT its `omnisim` controller-library import.

    The module imports Supervisor at top level, which only resolves inside a
    running controller, so stub it before exec.
    """
    import types
    stub = types.ModuleType("omnisim")
    for name in ("Supervisor", "Robot", "Camera", "Motor", "Node", "Field"):
        setattr(stub, name, type(name, (), {}))
    saved = sys.modules.get("omnisim")
    sys.modules["omnisim"] = stub
    # The bridge imports its sibling `mavic_dynamics`, which only resolves when the
    # controller's own directory is on sys.path (the engine puts it there).
    sys.path.insert(0, str(BRIDGE.parent))
    try:
        spec = importlib.util.spec_from_file_location("_mavic_bridge_probe", BRIDGE)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        sys.path.remove(str(BRIDGE.parent))
        if saved is not None:
            sys.modules["omnisim"] = saved
        else:
            sys.modules.pop("omnisim", None)


MOD = _load_verdict()
stall_verdict = MOD.stall_verdict
TOL = MOD.STALL_PROGRESS_TOL_M
TIMEOUT = MOD.STALL_TIMEOUT_S


def test_first_tick_seeds_the_best_and_is_never_stalled():
    stalled, best = stall_verdict(12.0, None, 0.0)
    assert stalled is False
    assert best == 12.0


def test_real_progress_resets_the_best():
    stalled, best = stall_verdict(5.0, 12.0, 999.0)
    assert stalled is False, "closing on the target is progress, whatever the clock says"
    assert best == 5.0


def test_no_progress_below_the_timeout_is_not_a_stall():
    stalled, best = stall_verdict(5.0, 5.0, TIMEOUT - 0.1)
    assert stalled is False
    assert best == 5.0


def test_no_progress_past_the_timeout_is_a_stall():
    stalled, best = stall_verdict(5.0, 5.0, TIMEOUT + 0.1)
    assert stalled is True
    assert best == 5.0


def test_creeping_closer_by_less_than_the_tolerance_still_stalls():
    # The wedged case: pressed against an obstacle, oscillating by centimetres.
    stalled, _ = stall_verdict(5.0 - (TOL / 2.0), 5.0, TIMEOUT + 1.0)
    assert stalled is True


def test_drifting_away_from_the_target_counts_as_no_progress():
    stalled, best = stall_verdict(9.0, 5.0, TIMEOUT + 1.0)
    assert stalled is True
    assert best == 5.0, "the best approach is not lost by drifting away"
