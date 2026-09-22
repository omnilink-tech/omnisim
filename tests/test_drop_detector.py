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

"""The drop detector must not cry wolf on a healthy carry.

`scripts/dev/diagnose_drop.py` is the tool half of the debugging demo in
`projects/samples/demos/worlds/debug/`. It decides, from `GET /sim/events`
alone, whether a robot arm carried a part to its destination or dropped it in
mid-air.

THE TRAP IT HAS TO SURVIVE. A perfectly healthy carry emits TWO
`grip.released` / `grip.acquired` flickers while the pads re-seat under load.
Measured on the real stream: released at t_sim 5056 and re-acquired 32 ms
later, released again at 5832 and re-acquired 40 ms later -- and then the part
reached PLACE_TABLE at 7408 and was deliberately released at 8208. A detector
that fires on "a grip.released while the part is airborne" reports a DROP on a
run that placed the part correctly. That is a debugger that lies, and per
docs/developer/positioning.md 6 ("instruments that refuse to lie") that is
worse than no debugger at all.

`test_naive_detector_does_false_positive` deliberately makes the naive rule go
red on the same fixture, so this file cannot quietly become vacuous -- the
standing rule from docs/benchmarks/determinism-scope.md: "an assertion that has
never gone red should be assumed broken until you make it go red on purpose."

The fixtures are real streams, trimmed. Provenance is on each one.

Engine-free: runs in `make tests-unit`.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "dev"))

import diagnose_drop  # noqa: E402

classify_carry = diagnose_drop.classify_carry


def _sup(seq, typ, t_ms, **kw):
    return dict(seq=seq, type=typ, t_sim_ms=t_ms, source="sup", **kw)


def _contact(seq, typ, t_ms, a, b):
    return _sup(seq, typ, t_ms, a_def=a, b_def=b, point=[0.0, 0.0, 0.0])


def _grip(seq, typ, t_ms, **kw):
    return _sup(seq, typ, t_ms, gripper_def="OMNIARM6", held_def="BLOCK", **kw)


def _log(seq, line):
    return dict(seq=seq, type="controller.log", stream="stdout", line=line,
                t_wall=1789336890.5, source="log")


SOLVER_MU6 = ("INFO: [OmNewtonBackend] contact/solver params from WorldInfo: "
              "mu=6 ke=8000 kd=200 iters=150 ls_iters=50")
SOLVER_MU15 = ("INFO: [OmNewtonBackend] contact/solver params from WorldInfo: "
               "mu=1.5 ke=8000 kd=200 iters=150 ls_iters=50")


# --- FIXTURE 1: a HEALTHY carry, with the two flickers -------------------
# Provenance: real GET /sim/events capture, omniarm6_real_pick_place at
# newtonGroundMu 6, full tracking, 2026-09-14. Controller verdict for this run
# was carried=True placed=True. Trimmed to the BLOCK events.
HEALTHY_CARRY = [
    _contact(1, "contact.began", 8, "BLOCK", "PICK_TABLE"),
    _log(15, SOLVER_MU6),
    _contact(5, "contact.began", 4688, "#193", "BLOCK"),
    _contact(6, "contact.began", 4688, "#205", "BLOCK"),
    _grip(7, "grip.acquired", 4712),
    _contact(8, "contact.ended", 5056, "#193", "BLOCK"),
    _contact(9, "contact.ended", 5056, "#205", "BLOCK"),
    _grip(10, "grip.released", 5056, held_for_ms=344),        # <- FLICKER 1
    _contact(11, "contact.began", 5064, "#193", "BLOCK"),
    _contact(12, "contact.began", 5064, "#205", "BLOCK"),
    _grip(13, "grip.acquired", 5088),                          # 32 ms later
    _contact(14, "contact.ended", 5216, "BLOCK", "PICK_TABLE"),
    _contact(20, "contact.ended", 5832, "#205", "BLOCK"),
    _grip(21, "grip.released", 5832, held_for_ms=744),         # <- FLICKER 2
    _contact(22, "contact.began", 5840, "#205", "BLOCK"),
    _grip(25, "grip.acquired", 5872),                          # 40 ms later
    _contact(30, "contact.began", 7408, "BLOCK", "PLACE_TABLE"),
    _contact(31, "contact.ended", 8208, "#193", "BLOCK"),
    _contact(32, "contact.ended", 8208, "#205", "BLOCK"),
    _grip(34, "grip.released", 8208, held_for_ms=2336),        # the real one
    _log(88, "[pick] RESULT carried=True pinched=False palm_wedge=False "
             "drift=0.0283m placed=True final=(0.250,-0.421,0.245) -> FAIL"),
]

# --- FIXTURE 2: a DROP that lands back on the PICK table -----------------
# Provenance: real capture at newtonGroundMu 1, full tracking, 2026-09-14.
# Controller verdict carried=False placed=False.
DROP_ONTO_PICK_TABLE = [
    _contact(1, "contact.began", 8, "BLOCK", "PICK_TABLE"),
    _log(16, "INFO: [OmNewtonBackend] contact/solver params from WorldInfo: "
             "mu=1 ke=8000 kd=200 iters=150 ls_iters=50"),
    _contact(19, "contact.began", 7400, "#205", "BLOCK"),
    _contact(20, "contact.began", 7424, "#193", "BLOCK"),
    _grip(21, "grip.acquired", 7424),
    _contact(23, "contact.ended", 7920, "BLOCK", "PICK_TABLE"),
    _contact(26, "contact.ended", 8992, "#205", "BLOCK"),
    _grip(27, "grip.released", 8992, held_for_ms=1568),        # in mid-air
    _contact(28, "contact.began", 9000, "#205", "BLOCK"),
    _contact(29, "contact.ended", 9048, "#193", "BLOCK"),
    _contact(30, "contact.began", 9056, "BLOCK", "PICK_TABLE"),  # 64 ms later
    _log(88, "[pick] RESULT carried=False pinched=False palm_wedge=False "
             "drift=0.3627m placed=False final=(0.537,-0.058,0.225) -> FAIL"),
]

# --- FIXTURE 3: a DROP that lands ON the destination table ---------------
# This is the demo world's own signature (0.30 kg at newtonGroundMu 1.5): the
# part is lost late in the carry and falls onto PLACE_TABLE, scoring
# `placed=True` in the controller's own result. An end-of-run screenshot
# cannot tell it from a success. ONLY the ORDER of the events can.
DROP_ONTO_PLACE_TABLE = [
    _contact(1, "contact.began", 8, "BLOCK", "PICK_TABLE"),
    _log(16, SOLVER_MU15),
    _grip(21, "grip.acquired", 7424),
    _contact(23, "contact.ended", 7920, "BLOCK", "PICK_TABLE"),
    _contact(26, "contact.ended", 9100, "#205", "BLOCK"),
    _grip(27, "grip.released", 9100, held_for_ms=1676),
    _contact(30, "contact.began", 9300, "BLOCK", "PLACE_TABLE"),  # 200 ms AFTER
    _log(88, "[pick] RESULT carried=False pinched=False palm_wedge=False "
             "drift=0.2253m placed=True final=(0.159,-0.441,0.225) -> FAIL"),
]


# ------------------------------------------------------------------------
# The trap
# ------------------------------------------------------------------------

def test_healthy_carry_is_not_a_drop():
    """THE regression this file exists for."""
    v = classify_carry(HEALTHY_CARRY)
    assert v.verdict == "carried", (
        "the detector called a successful carry a drop. The healthy stream "
        "emits two grip.released flickers at t_sim 5056 and 5832, both undone "
        "by a re-acquire 32 / 40 ms later; neither is a drop. Got: %s" % v.reason)
    assert v.dropped is False
    assert v.dest_contact_t_ms == 7408
    assert v.release_t_ms == 8208


def test_healthy_carry_reports_both_flickers():
    """The flickers must be accounted for out loud, not silently swallowed."""
    v = classify_carry(HEALTHY_CARRY)
    assert v.flickers == [(5056, 5088), (5832, 5872)]


def test_naive_detector_does_false_positive():
    """Prove the fixture can catch a naive rule -- or this file is vacuous.

    "Any grip.released after the part left the pick table" is the rule an agent
    writes first. On this fixture it fires three times on a successful carry.
    """
    left_table = next(e["t_sim_ms"] for e in HEALTHY_CARRY
                      if e["type"] == "contact.ended"
                      and {e["a_def"], e["b_def"]} == {"BLOCK", "PICK_TABLE"})
    naive = [e for e in HEALTHY_CARRY
             if e["type"] == "grip.released" and e["t_sim_ms"] >= left_table]
    assert len(naive) >= 2, (
        "the healthy fixture no longer contains the flickers that make this "
        "test meaningful -- do not 'fix' it by deleting them")
    assert classify_carry(HEALTHY_CARRY).verdict == "carried"


# ------------------------------------------------------------------------
# The positive cases
# ------------------------------------------------------------------------

def test_drop_onto_the_pick_table_is_detected():
    v = classify_carry(DROP_ONTO_PICK_TABLE)
    assert v.verdict == "dropped"
    assert v.release_t_ms == 8992
    assert v.reland_partner == "PICK_TABLE"
    assert v.reland_t_ms - v.release_t_ms == 64
    assert v.dest_contact_t_ms is None


def test_drop_onto_the_destination_is_still_a_drop():
    """Landing on the place table is not the same as being carried to it."""
    v = classify_carry(DROP_ONTO_PLACE_TABLE)
    assert v.verdict == "dropped", (
        "the part hit PLACE_TABLE 200 ms AFTER the grip ended, not before. "
        "Only the ordering separates this from a success -- the controller's "
        "own `placed` test scores it True. Got: %s" % v.reason)
    assert v.reland_partner == "PLACE_TABLE"
    assert v.dest_contact_t_ms is None


def test_the_pads_are_not_mistaken_for_a_landing_surface():
    """Opaque node ids (#193/#205) are gripper links, not a floor.

    URDF links carry no DEF, so contacts name them by node id. The pads
    re-touch the falling part within milliseconds of the release; a detector
    that counted that as "it hit something" would call every release a drop.
    """
    events = [e for e in DROP_ONTO_PICK_TABLE
              if not (e["type"] == "contact.began"
                      and {e.get("a_def"), e.get("b_def")} == {"BLOCK", "PICK_TABLE"}
                      and e["t_sim_ms"] > 9000)]
    v = classify_carry(events)
    assert v.verdict == "inconclusive", (
        "with the table contact removed the only thing the part touches after "
        "the release is the pads (#205 at 9000). That is not a landing. Got: "
        "%s" % v.verdict)


# ------------------------------------------------------------------------
# Refusing to certify on thin evidence
# ------------------------------------------------------------------------

def test_stream_that_ends_in_mid_fall_is_inconclusive():
    truncated = [e for e in DROP_ONTO_PICK_TABLE if e.get("t_sim_ms", 0) <= 8999]
    v = classify_carry(truncated)
    assert v.verdict == "inconclusive"
    assert v.dropped is False
    assert "mid-fall" in v.reason


def test_light_mode_stream_says_so_instead_of_guessing():
    """Light mode suppresses grip.* -- the answer must be 'I cannot see'."""
    light = [e for e in HEALTHY_CARRY
             if e["type"] in ("controller.log",)]
    v = classify_carry(light)
    assert v.verdict == "no_grip"
    assert "light mode" in v.reason
    assert v.dropped is False


def test_a_still_held_part_is_not_judged():
    held = [e for e in HEALTHY_CARRY if e.get("t_sim_ms", 0) < 8000]
    v = classify_carry(held)
    assert v.verdict == "no_grip"
    assert "still held" in v.reason


# ------------------------------------------------------------------------
# The suspect parameter, off the same stream
# ------------------------------------------------------------------------

@pytest.mark.parametrize("events,expected_mu", [
    (HEALTHY_CARRY, "6"),
    (DROP_ONTO_PICK_TABLE, "1"),
    (DROP_ONTO_PLACE_TABLE, "1.5"),
])
def test_solver_params_come_off_the_event_stream(events, expected_mu):
    """No file read: the engine publishes its own settings as controller.log."""
    params = diagnose_drop.read_solver_params(events)
    assert params.get("mu") == expected_mu
    assert params.get("ke") == "8000"
    assert params.get("iters") == "150"


def test_report_names_the_parameter_and_the_fix():
    v = classify_carry(DROP_ONTO_PLACE_TABLE)
    text = diagnose_drop.format_report(
        v, diagnose_drop.read_solver_params(DROP_ONTO_PLACE_TABLE),
        diagnose_drop.controller_result(DROP_ONTO_PLACE_TABLE))
    assert "newtonGroundMu = 1.5" in text
    assert "GRIP LOST IN MID-AIR" in text
    assert "omniarm6_drop_fault_fixed.omniworld" in text


def test_report_does_not_blame_friction_when_friction_is_nominal():
    """A dropped part at the reference mu must not be pinned on friction."""
    events = [e for e in DROP_ONTO_PLACE_TABLE if e["type"] != "controller.log"]
    events.append(_log(16, SOLVER_MU6))
    text = diagnose_drop.format_report(
        classify_carry(events), diagnose_drop.read_solver_params(events))
    assert "friction is NOT the fault here" in text
    assert "FIX" not in text.split("SUSPECT PARAMETER")[-1]
