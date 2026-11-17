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

"""`error` is a MEASUREMENT, not a fault flag.

PROTOCOL.md 5.4.1 rule 2 mandates that a completed action return
`{commanded, achieved, error, settled}`, where `error` is the numeric
residual `achieved - commanded`. The relay's old verdict was
`ok = "error" not in result`, written before that contract existed
(a2a8da5d / 52f3f6ca, both 2026-07-26), so it fired on SUCCESS: a measured
1.003 m drive against a 1.0 m command was journalled as
`result: "err", summary: "error: 0.0028145549502676115"`.

That verdict is persisted by ActionJournal and served back to the model by
get_action_history -- the tool the system prompt calls the authoritative log
and instructs the agent to consult before answering "did that land". So the
anti-fabrication machinery was being fed fabricated failures.

The payloads below are the exact recorded shapes from that run.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

# Allow running without pip-installing, same as test_smoke.py.
PKG_SRC = Path(__file__).resolve().parents[1] / "src"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from omnisim_bridges.relay import OmniLinkRelay, _result_failed  # noqa: E402


# The two recorded payloads, verbatim.
DRIVE_OK = {"accepted": True, "commanded": -0.5, "achieved": -0.501,
            "error": -0.00101, "settled": True}
BUSY_409 = {"accepted": False, "error": "busy", "http_status": 409}


# ── the verdict ──────────────────────────────────────────────────────

def test_numeric_residual_is_a_success() -> None:
    """The regression itself: a landed motion must not be logged as failed."""
    assert _result_failed(DRIVE_OK) is False


def test_busy_rejection_is_a_failure() -> None:
    """accepted=False + a STRING error + 409: refused on all three counts."""
    assert _result_failed(BUSY_409) is True


@pytest.mark.parametrize("result", [
    # A residual that happens to be positive, and one that is exactly zero --
    # both would read as "truthy error" to a `.get("error")` check.
    {"accepted": True, "commanded": 1.0, "achieved": 1.0028, "error": 0.0028,
     "settled": True},
    {"accepted": True, "commanded": 1.0, "achieved": 1.0, "error": 0.0,
     "settled": True},
    # Unmeasured is null, never 0.0 (5.4.1 rule 3). Still not a fault.
    {"accepted": True, "commanded": 1.5708, "achieved": None, "error": None,
     "settled": False, "superseded": True},
    # Reads carry no error key at all.
    {"pose": [1.0, 2.0, 0.0], "mode": "idle"},
])
def test_measurements_are_never_faults(result) -> None:
    assert _result_failed(result) is False


@pytest.mark.parametrize("result", [
    # Every string-error bridge in the tree keeps working unchanged: the arm
    # bridge is 5.4.1 "not yet" and reports refusals exactly like this.
    {"accepted": False, "moved": False, "error": "no_ik_solution"},
    {"accepted": False, "moved": False, "error": "joint_limit_exceeded"},
    {"error": "unknown tool"},
    {"error": "dispatch failed: KeyError('distance')"},
    # Refusals that carry NO error key at all -- the old predicate scored
    # these as successes.
    {"accepted": False, "refused": "outside_site_bounds"},
    {"accepted": False, "refused": "wait_false_unsupported"},
    # An HTTP-shaped failure without a string error.
    {"http_status": 503},
])
def test_refusals_are_failures(result) -> None:
    assert _result_failed(result) is True


def test_non_dict_results_do_not_explode() -> None:
    """Classification must never raise: it gates a journal write."""
    for junk in (None, "", [], 3, object()):
        assert _result_failed(junk) is False


# ── the audit line ───────────────────────────────────────────────────

def test_summary_of_a_landed_motion_reads_as_a_measurement() -> None:
    s = OmniLinkRelay._summarize_result(DRIVE_OK)
    assert not s.startswith("error:")
    assert "commanded=-0.5" in s
    assert "achieved=-0.501" in s
    assert "settled=True" in s


def test_summary_of_a_refusal_still_names_the_reason() -> None:
    assert OmniLinkRelay._summarize_result(BUSY_409) == "error: busy"


def test_summary_flags_a_superseded_motion() -> None:
    s = OmniLinkRelay._summarize_result(
        {"accepted": True, "verb": "turn", "commanded": 1.5708,
         "achieved": None, "error": None, "settled": False,
         "superseded": True})
    assert "superseded=True" in s
    assert "achieved=None" in s


def test_summary_of_a_keyless_refusal_says_refused() -> None:
    s = OmniLinkRelay._summarize_result(
        {"accepted": False, "refused": "outside_site_bounds"})
    assert s == "refused: outside_site_bounds"


if __name__ == "__main__":  # allow a direct run, as test_smoke.py does
    raise SystemExit(pytest.main([__file__, "-q"]))
