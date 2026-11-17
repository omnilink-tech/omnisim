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


# --- yaw_from_axis_angle (issue #14: `reset` returns to the AUTHORED pose) ----

def test_yaw_from_axis_angle_plain_z():
    import math
    assert abs(MOD.yaw_from_axis_angle([0, 0, 1, math.pi / 2]) - math.pi / 2) < 1e-9


def test_yaw_from_axis_angle_negative_z_axis_flips_sign():
    import math
    assert abs(MOD.yaw_from_axis_angle([0, 0, -1, 0.7]) + 0.7) < 1e-9


def test_yaw_from_axis_angle_identity_and_degenerate_axis():
    assert MOD.yaw_from_axis_angle([0, 0, 1, 0.0]) == 0.0
    assert MOD.yaw_from_axis_angle([0, 0, 0, 1.0]) == 0.0


def test_yaw_from_axis_angle_unnormalised_axis():
    import math
    assert abs(MOD.yaw_from_axis_angle([0, 0, 4, math.pi]) - math.pi) < 1e-9 or         abs(MOD.yaw_from_axis_angle([0, 0, 4, math.pi]) + math.pi) < 1e-9


def test_gimbal_ramp_rate_is_slow_enough_to_matter():
    # 1 rad/s at an 8 ms tick walks the pitch servo from 0 to straight down in
    # ~1.6 s instead of one step; the constant is the whole fix, so pin it.
    import math
    ticks = math.ceil(MOD.GIMBAL_DOWN_RAD / (MOD.GIMBAL_RATE_RAD_S * 0.008))
    assert 150 <= ticks <= 250


# --- _wait_until_arrived treats no_progress as ADVISORY (issue #14 regression) --

def _state_at(x, y, z, fault=None):
    st = MOD.BridgeState()
    st.x, st.y, st.z, st.fault = x, y, z, fault
    return st


def test_wait_does_not_return_early_on_no_progress():
    import time
    st = _state_at(0.0, 0.0, 1.0, fault="no_progress")
    t0 = time.time()
    res = MOD._wait_until_arrived(st, 5.0, 0.0, 1.0, timeout_s=0.3, poll_s=0.02)
    assert time.time() - t0 >= 0.25          # it waited the caller's budget out
    assert res["done"] is False
    assert res["fault"] == "no_progress"      # ...and still REPORTS the advisory


def test_wait_returns_early_on_a_hard_fault():
    import time
    st = _state_at(0.0, 0.0, 1.0, fault="crashed")
    t0 = time.time()
    res = MOD._wait_until_arrived(st, 5.0, 0.0, 1.0, timeout_s=2.0, poll_s=0.02)
    assert time.time() - t0 < 1.0
    assert res["fault"] == "crashed"


def test_wait_clears_stale_no_progress_on_arrival():
    st = _state_at(4.9, 0.0, 1.0, fault="no_progress")   # inside the reach tolerance
    res = MOD._wait_until_arrived(st, 5.0, 0.0, 1.0, timeout_s=1.0, poll_s=0.02)
    assert res["done"] is True and res["fault"] is None
    assert st.fault is None


# --- crash_verdict (issue #14: a fallen aircraft in mode=goto must say so) ----

def test_crash_verdict_needs_sustained_tilt():
    assert MOD.crash_verdict(0.0, -1.57, 0.0) == (False, True)     # just tipped: not yet
    assert MOD.crash_verdict(0.0, -1.57, MOD.CRASH_HOLD_S) == (True, True)


def test_crash_verdict_level_flight_is_not_a_crash():
    assert MOD.crash_verdict(0.1, -0.2, 10.0) == (False, False)


def test_crash_verdict_roll_counts_too():
    assert MOD.crash_verdict(1.5, 0.0, 2.0) == (True, True)


def test_crash_threshold_is_above_the_hold_law_clamp():
    # MAX_HOLD_TILT is an input in "disturbance units", not radians, but the
    # tilt a healthy flight reaches is well under 0.5 rad; the crash threshold
    # must never fire on a hard manoeuvre.
    assert MOD.CRASH_TILT_RAD >= 1.0


# --- stall_step: the wiring itself (issue #14, the 197 s pin that never fired) --

def _ticks(dists, dt=0.008):
    best, since, fault, t = None, 0.0, None, 0.0
    first = None
    for d in dists:
        t += dt
        best, since, fault = MOD.stall_step(best, since, fault, d, t)
        if fault == "no_progress" and first is None:
            first = t
    return fault, first


def test_bit_identical_constant_distance_stalls_at_the_timeout():
    # The v8.3.0 wiring reset the clock on every tick of a constant distance.
    fault, first = _ticks([1.073] * int(30 / 0.008))
    assert fault == "no_progress"
    assert abs(first - MOD.STALL_TIMEOUT_S) < 0.02


def test_jitter_without_progress_stalls_too():
    import random
    rnd = random.Random(1)
    fault, first = _ticks([1.073 + rnd.uniform(-1e-4, 1e-4) for _ in range(int(30 / 0.008))])
    assert fault == "no_progress" and first is not None


def test_real_progress_resets_the_clock_and_clears_the_advisory():
    n = int(10 / 0.008)
    dists = [5.0] * n + [4.0] * n            # 1 m closer at t = 10 s
    fault, first = _ticks(dists)
    assert fault is None and first is None   # 10 s, then progress, then 10 s: never 12 s without it
    dists = [5.0] * int(14 / 0.008) + [4.0] * int(2 / 0.008)
    fault, first = _ticks(dists)
    assert fault is None and abs(first - MOD.STALL_TIMEOUT_S) < 0.02   # fired at 12 s, cleared at 14 s


def test_hard_fault_is_never_overwritten_by_the_advisory():
    best, since, fault = MOD.stall_step(1.0, 0.0, "crashed", 1.0, 30.0)
    assert fault == "crashed"
