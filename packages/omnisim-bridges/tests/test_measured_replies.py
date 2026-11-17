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
"""A parsed order is answered from what the robot measured, not from the order.

WHY THIS EXISTS. ops-bench P7 (2026-09-25): the parser ran the last leg of a
plan without waiting, "to keep chat responsive", and replied at once. A Husky
that a block stopped at 1.2 m of a 2 m drive was reported as "Drive forward
(distance=2.0)." -- the operator's own number handed back as if done, and
nothing afterwards said otherwise.

A bridge whose halts, new orders AND questions get past its action lock can
afford to wait (a stop is never trapped behind the wait, and a question is
answered beside it), and opts in with `replies_after_motion`.
"""
from __future__ import annotations

import pathlib

import pytest

from omnisim_bridges import route

ROOT = pathlib.Path(__file__).resolve().parents[3]
MOBILE = ROOT / "projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py"


def test_a_blocked_drive_is_an_error_that_says_so():
    said, status, detail = route._measured_motion("drive_forward", {
        "accepted": True, "commanded": 2.0, "achieved": 1.199, "error": -0.801,
        "settled": False, "timed_out": True})
    assert status == "err" and "stopped short" in said and "+1.20" in said


def test_a_completed_drive_reports_the_measured_distance():
    said, status, _ = route._measured_motion("drive_forward", {
        "commanded": 2.0, "achieved": 1.988, "error": -0.012, "settled": True, "timed_out": False})
    assert status == "ok" and said == "Drove +1.99 m."


def test_a_short_turn_is_an_error():
    said, status, _ = route._measured_motion("turn", {
        "commanded": 1.5708, "achieved": 0.6, "settled": False, "timed_out": True})
    assert status == "err" and "stopped short" in said


def test_a_superseded_motion_says_it_was_ended():
    said, status, detail = route._measured_motion("drive_forward", {"superseded": True, "achieved": None})
    assert detail == "superseded" and "ended before it finished" in said


@pytest.mark.parametrize("res", [{"accepted": True, "commanded": 2.0},      # did not wait
                                 {"achieved": 1.0}])                          # no command
def test_an_unmeasured_result_keeps_the_old_reply(res):
    assert route._measured_motion("drive_forward", res) is None


class _Bridge:
    intents = None

    def __init__(self, flag):
        self.replies_after_motion = flag
        self.calls = []

    def _read_pose(self):
        return (0.0, 0.0, 0.0)

    def act_drive_forward(self, distance, wait=False):
        self.calls.append(wait)
        if not wait:
            return {"accepted": True, "commanded": distance, "eta_s": 4.0}
        return {"accepted": True, "commanded": distance, "achieved": 1.199,
                "error": distance - 1.199, "settled": False, "timed_out": True}


def test_an_opted_in_bridge_waits_and_reports_the_blocked_drive():
    b = _Bridge(True)
    out = route.short_circuit(b, "Drive forward 2 metres.", "mobile")
    assert b.calls == [True]
    assert "stopped short" in out["agent"]
    # What the operator's /prompt receives: a top-level error, not a success.
    payload = route.reply_payload(out["agent"], out["tools"])
    assert payload.get("error")


def test_other_bridges_keep_the_immediate_reply():
    b = _Bridge(False)
    out = route.short_circuit(b, "Drive forward 2 metres.", "mobile")
    assert b.calls == [False] and out["agent"] == "Drive forward (distance=2.0)."


def test_the_mobile_bridge_opts_in_and_lets_questions_past_its_lock():
    src = MOBILE.read_text(encoding="utf-8")
    assert "    replies_after_motion = True" in src
    post = src[src.index("        def do_POST(self):"):]
    post = post[:post.index("            except RequestError as e:")]
    # `takes_over` (2026-10-01) joined the same bypass for a superseding parsed order.
    assert "aside = True" in post and "or halt or aside or takes_over):" in post
    assert post.index("aside = True") < post.index("with action_lock")


def test_a_stalled_drive_says_it_was_blocked():
    said, status, detail = route._measured_motion("drive_forward", {
        "commanded": 2.0, "achieved": 0.76, "settled": False, "timed_out": False,
        "blocked": True, "blocked_detail": "no progress for 1.5 sim-s after 0.76 m"})
    assert status == "err" and said.startswith("I was blocked") and "+0.76" in said


def test_the_mobile_bridge_detects_a_stall_and_never_rams_it_again():
    src = MOBILE.read_text(encoding="utf-8")
    assert "DRIVE_STALL_S = 1.5" in src
    # a blocked leg gets no correction legs, and each leg measures its own progress
    assert 'and not p.get("blocked")' in src and 'p2.pop("stall_ref", None)' in src
    assert '"blocked": True, "blocked_detail"' in src
