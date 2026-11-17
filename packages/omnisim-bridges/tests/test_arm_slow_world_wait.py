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

"""A SLOW world is not a STOPPED one: set_tcp_target on a long tick (2026-10-17).

THE DEFECT. The arm's `wait` called the world stalled after 2 s of WALL
silence from the sim thread. MEASURED (WIRobotics ALLEX on installed v9.2.5,
0.09-0.18x real time, a capture camera on): one tick gap of ~2.4 s wall in
the middle of the blue hover, and set_tcp_target answered `stalled`,
`achieved: None`, `settled: False` for a move the arm finished 1.4 mm from
target. The model had to call get_robot_state to find out it had worked.

THE FIX. The stall threshold is 10 s (bridge_base.STALL_GRACE_S, the arm's
WAIT_STALL_S), and a wait that genuinely gives up hands back the joints'
last MEASURED reading as `last_reading` -- labelled, with `achieved` still
null, because the motion never reported (tool-design-for-agents.md P2).

⚠️ THIS DRIVES THE SHIPPED ArmBridge, imported behind the stub `omnisim`
Supervisor and fake servo plant of test_arm_gravity_trim_and_lift.
"""

from __future__ import annotations

import ast
import math
import threading
import time

from omnisim_bridges.bridge_base import STALL_GRACE_S
from omnisim_bridges.relay import OmniLinkRelay

import pathlib as _pathlib
import sys as _sys

_TESTS = str(_pathlib.Path(__file__).resolve().parent)
if _TESTS not in _sys.path:          # importlib mode: siblings by path
    _sys.path.insert(0, _TESTS)

from test_arm_gravity_trim_and_lift import (  # noqa: E402,F401  (fixture)
    ARM_DIR, DT, _fk, _make, _run, br)

TARGET = (0.45, -0.15, 0.30)       # a reachable UR5e point, arm base frame


def _shipped_wait_stall_s() -> float:
    tree = ast.parse((ARM_DIR / "omnilink_arm_bridge.py").read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and getattr(node.targets[0], "id", None) == "WAIT_STALL_S"):
            return float(ast.literal_eval(node.value))
    raise AssertionError("WAIT_STALL_S not found")


def test_the_stall_threshold_outlasts_a_measured_slow_tick():
    # 2.4 s is the ALLEX tick gap; the arm and the shared waits agree.
    assert _shipped_wait_stall_s() == STALL_GRACE_S
    assert STALL_GRACE_S >= 4 * 2.4


def _drive_with_gap(bridge, robot, call, gap_wall_s, after_start_sim_s,
                    resume=True, limit_sim_s=20.0):
    """Run `call` on a worker thread (the HTTP/relay thread) while this
    thread ticks the sim. `after_start_sim_s` of sim after the motion has
    claimed its seq, stop ticking for `gap_wall_s` of WALL time -- one very
    long engine tick -- and then resume (or, with resume=False, never).
    Returns (result, the motion's completion as it stood at the gap)."""
    seq0 = bridge.motion_seq
    out = {}
    worker = threading.Thread(target=lambda: out.update(call()), daemon=True)
    worker.start()
    t = 0.0
    t_start = None
    at_gap = None
    while worker.is_alive() and t < limit_sim_s:
        _run(bridge, robot, DT)
        t += DT
        if t_start is None and bridge.motion_seq > seq0:
            t_start = t
        if (at_gap is None and t_start is not None
                and t - t_start >= after_start_sim_s):
            at_gap = {"seq": bridge.motion_seq,
                      "completion": dict(bridge.last_completion or {})}
            time.sleep(gap_wall_s)
            if not resume:
                break
        time.sleep(0.001)
    worker.join(timeout=5.0)
    return out, at_gap


def test_a_long_tick_mid_move_still_returns_the_measured_result(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    out, at_gap = _drive_with_gap(
        bridge, robot,
        lambda: bridge.act_set_tcp_target(TARGET, wait=True),
        gap_wall_s=2.6, after_start_sim_s=0.3)
    # The gap landed while THIS motion was still in flight.
    assert at_gap is not None
    assert at_gap["completion"].get("seq") != at_gap["seq"]
    assert out.get("seq") == at_gap["seq"]
    assert out.get("stalled") is not True, out
    assert out.get("achieved") is not None, out
    assert out.get("timed_out") is False
    err = math.dist(out["achieved"], TARGET)
    assert err <= bridge.TCP_TOL_M + 1e-9, out


def test_a_real_stall_reports_stalled_with_the_last_reading(br, monkeypatch):
    bridge, robot, cfg = _make(br, monkeypatch)
    _run(bridge, robot, 0.5)
    # Short threshold for the test only; the world then stops for good.
    bridge.WAIT_STALL_S = 0.3
    out, _ = _drive_with_gap(
        bridge, robot,
        lambda: bridge.act_set_tcp_target(TARGET, wait=True),
        gap_wall_s=2.0, after_start_sim_s=0.2, resume=False)
    assert out.get("stalled") is True, out
    assert out.get("achieved") is None          # P2: no completion, no number
    assert out.get("settled") is False and out.get("timed_out") is False
    reading = out["last_reading"]
    # It IS a measurement: FK of the joints the last tick read, and the
    # error is against what was commanded.
    expect = _fk(br, cfg, reading["q"])
    assert math.dist(reading["tcp"], expect) < 1e-4
    assert abs(reading["error_m"] - math.dist(reading["tcp"], TARGET)) < 1e-4
    assert "NOT a completion" in reading["note"]
    assert reading["age_wall_s"] >= 0.0


def test_the_audit_line_names_a_stall():
    line = OmniLinkRelay._summarize_result(
        {"verb": "set_tcp_target", "commanded": [0.6, -0.1, 0.101],
         "achieved": None, "settled": False, "timed_out": False,
         "stalled": True,
         "last_reading": {"tcp": [0.5997, -0.1013, 0.1005]}})
    assert "stalled=True" in line
    assert "last_reading_tcp=[0.5997, -0.1013, 0.1005]" in line
