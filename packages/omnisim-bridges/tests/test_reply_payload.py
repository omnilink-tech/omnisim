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

"""A failed action must reach the caller as a FIELD, not as a sentence.

This pins a bug that cost a two-hour endurance run. The bridge refuses a
command that arrives while it is still moving, and it was honest about it:
the reply read "I could not turn: busy" and the action carried
result="err", summary="busy". But there was no top-level `error`, and the
client checked `error`.

About 5% of orders were dropped that way. The closed square the robot was
driving stopped closing, it walked off the 12 m floor to (-6.86, 4.75), and
zero errors were logged from start to finish. The commands themselves were
never inaccurate -- the last turn before the escape missed by 0.00042 rad.

Whatever drives this surface is a program. A failure it cannot see is a
failure that did not happen.
"""
from omnisim_bridges.route import reply_payload


def test_a_refused_action_sets_a_top_level_error():
    """THE regression. A client checking only `error` must see the failure."""
    out = reply_payload("I could not turn: busy", [("turn", "err", "busy")])
    assert out["error"] == "turn: busy"


def test_a_refusal_counts_as_a_failure_too():
    out = reply_payload("I won't do that.",
                        [("drive_forward", "refused", "unsafe distance")])
    assert out["error"] == "drive_forward: unsafe distance"


def test_a_clean_command_has_no_error_key_at_all():
    """Absent, not empty: `if out.get("error")` and `"error" in out` agree."""
    out = reply_payload("Drive forward (distance=2.0).",
                        [("drive_forward", "ok", "distance=2.0")])
    assert "error" not in out


def test_every_failure_is_named_not_just_the_first():
    out = reply_payload("Partly done.", [
        ("drive_forward", "ok", "distance=2.0"),
        ("turn", "err", "busy"),
        ("stop", "err", "no response"),
    ])
    assert out["error"] == "turn: busy; stop: no response"


def test_a_failure_with_no_detail_still_reports_something():
    """An empty summary must not produce a blank, unactionable error."""
    out = reply_payload("Hmm.", [("turn", "err", "")])
    assert out["error"] == "turn: err"


def test_the_actions_list_survives_unchanged():
    """Callers that already read actions[] must not have to change."""
    out = reply_payload("x", [("turn", "err", "busy")])
    assert out["actions"] == [
        {"tool": "turn", "result": "err", "summary": "busy"}]
    assert out["response"] == "x"


def test_extra_fields_pass_through():
    out = reply_payload("x", [("turn", "ok", "")], via="parser")
    assert out["via"] == "parser"


def test_short_tuples_are_skipped_rather_than_crashing():
    """A malformed tuple must not take down the reply on a live bridge."""
    out = reply_payload("x", [("turn",), ("stop", "err", "busy")])
    assert out["actions"] == [
        {"tool": "stop", "result": "err", "summary": "busy"}]
    assert out["error"] == "stop: busy"


def test_no_actions_means_no_error():
    out = reply_payload("Just chatting.", [])
    assert out["actions"] == []
    assert "error" not in out
