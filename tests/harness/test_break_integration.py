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

"""Opt-in real-engine coverage for /sim/pause, /sim/resume and /sim/break.

This is the pytest face of `break_live_check.py`: same four cases, one test
each, so a green unit lane can be followed by a green engine lane without
writing the sequence twice. Run it against an isolated harness:

    python -m omnisim harness --port 16789 --supervisor-port 16790
    OMNISIM_HARNESS_INTEGRATION_URL=http://127.0.0.1:16789 \\
      pytest tests/harness/test_break_integration.py -q

Skipped (not failed) with no URL, so `make tests-unit` never sees it -- and
the `OMNISIM_HARNESS_INTEGRATION_URL` literal is itself what marks this module
`engine` at collection (tests/conftest.py, "live harness URL").

⚠ ONE ENGINE AT A TIME. Each case loads a world into the harness you point it
at; they share it, and they are ordered so that the light-mode case runs last.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import break_live_check as live  # noqa: E402

BASE = os.environ.get("OMNISIM_HARNESS_INTEGRATION_URL", "").rstrip("/")
pytestmark = pytest.mark.skipif(not BASE, reason="set OMNISIM_HARNESS_INTEGRATION_URL")


@pytest.fixture(scope="module")
def client():
    return live.Client(BASE, verbose=False)


def test_pause_holds_steps_extends_and_expires(client):
    """v9 C1: the five things a first-class pause has to be true about."""
    detail = live.case_pause(client)
    assert detail["pause"]["paused"] is True
    # It stops BOTH clocks, not just the supervisor's bookkeeping.
    assert detail["frozen"]["held_for_wall_s"] >= 1.0
    # Exactly N basic steps on the supervisor clock, and the hold survives.
    assert detail["step_1"]["supervisor_advanced_ms"] == detail["basic_time_step_ms"]
    assert detail["step_10"]["supervisor_advanced_ms"] == 10 * detail["basic_time_step_ms"]
    assert detail["step_10"]["lease_remaining_ms"] > 0
    # A second pause extends rather than failing.
    assert detail["extend"]["after"]["lease_remaining_ms"] > detail["extend"]["before_ms"]
    # Resume is idempotent.
    assert detail["resume_idempotent"][0]["was_paused"] is True
    assert detail["resume_idempotent"][1]["was_paused"] is False
    # The deadline is the safety property: nobody called resume here.
    assert detail["lease_expiry"]["after_4s"]["paused"] is False
    assert (detail["lease_expiry"]["after_4s"]["engine_time_ms"]
            > detail["lease_expiry"]["held"]["engine_time_ms"])
    assert detail["lease_expiry"]["supervisor_still_connected"] is True


def test_break_on_contact_freezes_the_scene_at_the_event(client):
    """v9 C2 CASE 1."""
    detail = live.case_contact(client)
    hit = detail["hit"]
    assert hit["matched_type"] == "contact.began"
    assert hit["matched"]["a_def"] == "BREAK_BOX"
    assert hit["paused"] is True
    # Under a held pause, /sim/step scans the bus on every basic step, so the
    # hold lands on the step of the contact: zero supervisor-clock latency.
    # The honest bound on ENGINE time is the width of that tick, reported.
    assert detail["hold_latency_ms"] == 0.0
    assert detail["hold_latency_steps"] == 0
    assert detail["hold_latency_engine_ms_max"] is not None
    # "advance up to 400 steps and stop on the breakpoint" stopped early.
    assert detail["continue_response"]["stopped_on_break"] == detail["break_id"]
    assert (detail["continue_response"]["steps_executed"]
            < detail["continue_response"]["steps_requested"])
    # Frozen at contact height, with a witness still in free fall -- which is
    # what separates "frozen at the event" from "settled".
    assert abs(detail["box_z_at_break"] - 0.1) < 0.06
    assert detail["witness_z_at_break"] > 5.0
    # TWO INDEPENDENT CLOCKS, cross-checked. The witness is in free fall, so
    # its height is a clock; the harness publishes `engine_time_ms`, which is
    # `supervisor.getTime()` -- the engine's time as of the controller's last
    # step. Under a stepped hold those agree closely: MEASURED 2026-09-22,
    # 1012.0 ms implied by the witness's height against 1000.0 ms of engine
    # clock since the reset, a 12 ms (1.5 basic step) gap. The tolerance is
    # loose because the engine's overshoot after a queued PAUSE lands inside
    # that gap and grows with machine load.
    assert abs(detail["witness_vs_reported_engine_ms"]) < 200.0
    assert detail["step"]["supervisor_advanced_ms"] == 80.0


def test_break_on_joint_limit_freezes_the_arm_against_its_stop(client):
    """v9 C2 CASE 2."""
    detail = live.case_joint(client)
    hit = detail["hit"]
    assert hit["matched_type"] == "joint.limit_hit"
    assert hit["matched"]["joint"] == "arm_motor"
    assert hit["matched"]["side"] == "upper"
    joint = detail["joint_at_break"]
    assert joint["hit_limit"] == "upper"
    assert abs(joint["position"] - joint["upper"]) < 1e-3
    assert detail["step"]["supervisor_advanced_ms"] == 80.0


def test_light_session_refuses_a_break_it_could_never_fire(client):
    """v9 C2 CASE 3 -- the honesty case."""
    detail = live.case_light_refusal(client)
    refusal = detail["refusal"]
    assert refusal["code"] == "BREAK_EVENT_TYPE_UNAVAILABLE"
    assert refusal["armed_types"] == []
    codes = [d["code"] for d in refusal["diagnostics"]]
    assert "event_type_silenced_in_light_mode" in codes
    assert "contact.began" in detail["silenced_types"]
    # ...and it is scoped: damage.* survives --light, so a damage break is
    # still accepted in the same session.
    assert detail["damage_break_accepted"]
