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

"""The mobile bridge FAULTS a tipped base instead of reporting success.

THE DEFECT (2026-10-17). A drive, turn or stop measured a planar pose
difference and nothing else. MEASURED (MAB HB50W wheeled quadruped on
installed v9.2.5, with a higher-friction contact setting): the base rolled to
a 48 deg tilt, lost a wheel contact and ended 101 deg off heading, and the
bridge replied "Drove +1.98 m. Turned +89 deg. Drove +0.92 m. Stopped...
standing still."

THE GUARD. Every tick the angle between the node's +Z and world +Z is
measured. Above the limit (35 deg, OMNISIM_MOBILE_TILT_LIMIT_DEG or the
config's `tilt_limit_deg`) the bridge declares `fault: {code: "base_tilt"}`,
zeroes the wheels, ends the motion in flight with `settled: false` and the
fault attached, and refuses drive/turn until the base is back under
limit - 5 deg. OMNISIM_MOBILE_TILT_GUARD=0 is the hatch.

⚠️ THE SHIPPED METHODS, lifted out of the controller with the AST trick
test_step_budget uses (the controller imports `omnisim` at module scope),
run against a fake node whose attitude the test sets.
"""

from __future__ import annotations

import functools
import math
import threading
import time
from typing import Any, Dict, Optional

import pytest

from omnisim_bridges.route import _describe

import pathlib as _pathlib
import sys as _sys

_TESTS = str(_pathlib.Path(__file__).resolve().parent)
if _TESTS not in _sys.path:          # importlib mode: siblings by path
    _sys.path.insert(0, _TESTS)

from test_step_budget import MOBILE, load_method  # noqa: E402

METHODS = ("_read_tilt_deg", "_tilt_faulted", "_tilt_guard", "_tilt_message",
           "_halt_for_fault", "_fault_refusal", "_record_completion",
           "_begin_motion", "_record_superseded")


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _rot_x(deg: float):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c]


class _Node:
    def __init__(self):
        self.roll_deg = 0.0

    def getOrientation(self):
        return _rot_x(self.roll_deg)


@functools.lru_cache(maxsize=1)
def _bridge_class():   # one AST parse of the 10k-line controller per method, once
    ns: Dict[str, Any] = {"math": math, "time": time, "clamp": _clamp,
                          "wrap_pi": _wrap_pi, "emit_motion_outcome": None,
                          "Optional": Optional, "Dict": Dict, "Any": Any,
                          "SOURCE_EXTERNAL": "external"}
    attrs = {name: load_method(MOBILE, name, ns) for name in METHODS}
    attrs.update({"TILT_LIMIT_DEG": 35.0, "TILT_CLEAR_MARGIN_DEG": 5.0,
                  "SOURCE_EXTERNAL": "external"})
    return type("_FakeMobile", (), attrs)


class _Rig:
    def __init__(self, guard_on=True, limit=35.0):
        self.b = _bridge_class()()
        b = self.b
        b.self_node = _Node()
        b.tilt_guard_on = guard_on
        b.tilt_limit_deg = limit
        b.tilt_deg = None
        b.fault = None
        b.lock = threading.RLock()
        b.motion = ("idle", {})
        b.motion_seq = 0
        b.last_completion = None
        b.bridge_instance_id = "test"
        b.robot_id = "hb50w"
        b.sim_time = 4.0
        b.sim_step = 500
        b.events = None
        b.windows = []
        b.wheels = []
        b.queue_window = b.windows.append
        b._command_velocity = lambda l, a: b.wheels.append((l, a))

    def start_drive(self, distance=2.0):
        seq, refusal = self.b._begin_motion("external")
        assert refusal is None
        self.b.motion = ("drive", {
            "x0": 0.0, "y0": 0.0, "yaw0": 0.0, "seq": seq,
            "verb": "drive_forward", "source": "external",
            "commanded": distance, "unit": "m", "distance": abs(distance),
            "t0_sim": self.b.sim_time, "step_start": self.b.sim_step,
            "sim_time_start": self.b.sim_time, "timeout_s": 30.0,
            "phase": "approach", "corrections": 0})
        return seq

    def tick(self, roll_deg, x=0.0, y=0.0, yaw=0.0):
        self.b.self_node.roll_deg = roll_deg
        self.b.sim_time += 0.008
        self.b._tilt_guard(x, y, yaw)


def test_an_upright_base_is_measured_and_left_alone():
    r = _Rig()
    seq = r.start_drive()
    r.tick(0.5)
    assert r.b.fault is None
    assert r.b.tilt_deg == pytest.approx(0.5, abs=1e-3)
    assert r.b.motion[0] == "drive" and r.b.motion[1]["seq"] == seq


def test_tipping_mid_drive_faults_stops_and_reports_the_measured_distance():
    r = _Rig()
    seq = r.start_drive(2.0)
    r.tick(48.0, x=1.2, y=0.05, yaw=0.1)
    f = r.b.fault
    assert f["code"] == "base_tilt"
    assert f["tilt_deg"] == pytest.approx(48.0, abs=0.01)
    assert f["limit_deg"] == 35.0
    assert r.b.motion == ("idle", {})
    assert r.b.wheels[-1] == (0.0, 0.0)
    done = r.b.last_completion
    assert done["seq"] == seq
    assert done["settled"] is False and done["timed_out"] is False
    assert done["achieved"] == pytest.approx(1.2)        # along the start heading
    assert done["fault"]["code"] == "base_tilt"
    assert done["completion_reason"] == "fault"


def test_a_tipped_base_refuses_new_motion_and_the_latch_holds():
    r = _Rig()
    r.tick(48.0)
    seq, refusal = r.b._begin_motion("external")
    assert seq is None
    assert refusal["accepted"] is False
    assert refusal["rule"] == "base_tilt"
    assert "tipped" in refusal["say"]
    # Inside the hysteresis band (30..35 deg): still faulted, still zeroed.
    r.b.wheels.clear()
    r.tick(33.0)
    assert r.b.fault["code"] == "base_tilt"
    assert r.b.fault["peak_tilt_deg"] == pytest.approx(48.0, abs=0.01)
    assert r.b.wheels == [(0.0, 0.0)]
    # The message follows the CURRENT tilt -- measured live: a Husky tipped
    # at 90 deg rolled onto its back and the reply still said "90 deg".
    r.tick(179.0)
    assert "179 deg" in r.b.fault["message"]
    assert r.b.fault["peak_tilt_deg"] == pytest.approx(179.0, abs=0.01)


def test_righting_the_base_clears_the_fault_and_motion_works_again():
    r = _Rig()
    r.tick(60.0)
    r.tick(2.0)                       # reset_to_home teleports it upright
    assert r.b.fault is None
    seq, refusal = r.b._begin_motion("external")
    assert refusal is None and seq is not None


def test_a_turn_ended_by_the_guard_reports_the_rotation_it_got():
    r = _Rig()
    seq, _ = r.b._begin_motion("external")
    r.b.motion = ("turn", {"seq": seq, "verb": "turn", "source": "external",
                           "commanded": math.pi / 2, "unit": "rad",
                           "remaining": 1.0, "phase": "pulse",
                           "pulse_y0": 0.3, "t0_sim": 4.0})
    r.tick(50.0, yaw=0.5)
    done = r.b.last_completion
    # commanded - (remaining - progress of the pulse in flight)
    assert done["achieved"] == pytest.approx(math.pi / 2 - (1.0 - 0.2))
    assert done["settled"] is False and done["fault"]["code"] == "base_tilt"


def test_the_hatch_off_measures_but_never_faults():
    r = _Rig(guard_on=False)
    seq = r.start_drive()
    r.tick(70.0)
    assert r.b.fault is None
    assert r.b.tilt_deg == pytest.approx(70.0, abs=1e-3)
    assert r.b.motion[1]["seq"] == seq


def test_the_hatch_is_value_parsed_in_the_shipped_constructor():
    src = MOBILE.read_text(encoding="utf-8")
    assert '"OMNISIM_MOBILE_TILT_GUARD", "1"' in src
    assert 'not in ("0", "false", "no", "off")' in src
    assert "OMNISIM_MOBILE_TILT_LIMIT_DEG" in src
    assert 'cfg.get("tilt_limit_deg")' in src


FAULT = {"code": "base_tilt", "tilt_deg": 48.0,
         "message": "base tilted 48 deg from upright (limit 35 deg)"}


def test_the_reply_never_says_drove_over_a_tipped_base():
    said, status, _detail, _rule = _describe(
        "drive_forward", {"distance": 2.0},
        {"accepted": True, "commanded": 2.0, "achieved": 1.98, "error": -0.02,
         "settled": False, "timed_out": False, "fault": FAULT}, [])
    assert status == "err"
    assert "Drove" not in said and "tilted 48 deg" in said and "+1.98 m" in said


def test_a_stop_on_a_tipped_base_is_not_reported_as_fine():
    said, status, _d, _r = _describe(
        "stop", {}, {"accepted": True, "stationary": True, "settled": True,
                     "measured": {"speed_mps": 0.0, "over_s": 0.8},
                     "fault": FAULT}, [])
    assert status == "err"
    assert "standing still" not in said and "FAULTED" in said


def test_a_refused_drive_carries_the_fault_rule():
    said, status, _d, rule = _describe(
        "drive_forward", {"distance": 1.0},
        {"accepted": False, "error": "base_tilt", "rule": "base_tilt",
         "fault": FAULT, "say": "I cannot move: my base is tipped 48 deg."},
        [])
    assert status == "refused" and rule == "base_tilt"
    assert "tipped 48 deg" in said
