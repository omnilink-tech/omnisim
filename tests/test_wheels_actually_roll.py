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

"""The CHEAP half of "the wheels actually roll" -- no engine, milliseconds.

The expensive half is the smoke lane: `tests/physics/worlds/wheel_roll_noslip.omniworld`
runs in the pre-push gate and measures real wheel rotation against real chassis
motion. This file guards the things a measurement cannot guard itself against:

  1. THE GATE IS STILL WIRED IN. A physics gate that was quietly dropped from
     `tests/smoke/smoke_worlds.json`, or flipped to `skip: true` without a
     reason, is worse than no gate -- the lane keeps printing "all smoke worlds
     passed".

  2. THE NEGATIVE CONTROL IS STILL A CONTROL. It is generated from the gate
     world by a fixed edit list; a hand-edit that lets the two drift apart turns
     `roll_check.py --self-test` into two runs of the same world, which would
     pass forever and prove nothing.

  3. NO WHEELED WORLD ENTERS THE TREE UNMEASURED. `tests/goldens/
     roll_check_baseline.json` records the measured verdict of every
     hand-authored wheeled world in the corpus. If someone adds one (or the
     wheel rule starts seeing one it used to miss), this fails and says so --
     which is exactly the hole the whole exercise exists to close, since the
     original defect got in because nothing ever asked whether its wheels
     turned.

None of these starts the simulator. Adding a world means running

    python scripts/dev/roll_check.py sweep <world> --json /tmp/x.json

and recording the verdict in the baseline with a one-line reason if it is not
ROLLING.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts" / "dev"))

import roll_check  # noqa: E402

BASELINE = REPO_ROOT / "tests" / "goldens" / "roll_check_baseline.json"
SMOKE_FILE = REPO_ROOT / "tests" / "smoke" / "smoke_worlds.json"
GATE_WORLD = "tests/physics/worlds/wheel_roll_noslip.omniworld"

#: Verdicts a world may sit at in the baseline without a documented reason.
#: Anything else needs a `note` saying why it is acceptable, because "we looked
#: at it and shrugged" and "we never looked" are indistinguishable otherwise.
CLEAN_VERDICTS = {"ROLLING"}


def _load_baseline():
    return json.loads(BASELINE.read_text(encoding="utf-8"))


def test_smoke_lane_still_runs_the_roll_gate():
    entries = json.loads(SMOKE_FILE.read_text(encoding="utf-8"))
    matching = [e for e in entries if e.get("world", "").replace("\\", "/") == GATE_WORLD]
    assert matching, (
        f"{GATE_WORLD} is no longer in tests/smoke/smoke_worlds.json. It is the "
        "only check in this repo that asserts a WHEEL TURNED -- every other one "
        "asks whether the body MOVED, which a robot sliding on locked wheels "
        "also satisfies. Put it back, or replace it with something that grades "
        "the same property."
    )
    entry = matching[0]
    assert not entry.get("skip"), (
        "the wheel no-slip gate is skipped: " + str(entry.get("skip_reason"))
    )
    assert entry.get("asserts"), (
        "the gate has no `asserts` note. Every world in this lane records what "
        "it grades and what its tolerances mean; a bare entry is unmaintainable."
    )


def test_negative_control_is_still_a_control():
    """It must be derivable from the gate world by exactly the recorded edits.

    A control that has drifted into a copy of the gate world passes the
    self-test forever while proving nothing.
    """
    good = (REPO_ROOT / roll_check.SELF_TEST_GOOD).read_text(encoding="utf-8")
    bad = (REPO_ROOT / roll_check.SELF_TEST_BAD).read_text(encoding="utf-8")
    good_body = good[good.index("\nWorldInfo {"):]
    bad_body = bad[bad.index("\nWorldInfo {"):]
    derived = good_body
    for needle, replacement in roll_check.NEGATIVE_CONTROL_EDITS:
        assert needle in derived, (
            f"{roll_check.SELF_TEST_GOOD} no longer contains {needle!r}, so the "
            "negative control cannot be derived from it. Update "
            "NEGATIVE_CONTROL_EDITS in scripts/dev/roll_check.py and rerun "
            "`roll_check.py --regenerate-negative-control`."
        )
        derived = derived.replace(needle, replacement)
    assert derived == bad_body, (
        "the negative control has drifted from the gate world. Regenerate it:\n"
        "  python scripts/dev/roll_check.py --regenerate-negative-control"
    )
    assert derived != good_body, "the negative control is identical to the gate world"


#: Worlds authored FOR UPSTREAM WEBOTS, kept in the tree as benchmark control
#: arms (AGENTS.md: real Webots must still load them, and agentbench's freeze
#: manifest pins them by SHA -- a red freeze test is a release gate). Whether
#: their wheels roll under OmniSim's solver is not a property of OmniSim, and
#: a "fix" would break the freeze, so they are outside this corpus by rule.
UPSTREAM_CONTROL_ARMS = (
    "tests/benchmarks/agentbench/adapters/webots/",
    "tests/benchmarks/ladder0/webots/",
)


def test_every_hand_authored_wheeled_world_has_a_measured_verdict():
    baseline = _load_baseline()
    recorded = set(baseline["worlds"])
    # tracked_only: the corpus is the committed tree, never the run residue a
    # box happens to hold (29 gitignored metazoa epoch worlds on one machine).
    # Generated worlds (`# GENERATED by ...` header) are skipped by the scan:
    # their verdict belongs to the generator, not to each emitted variant.
    rows = roll_check.scan([REPO_ROOT / "projects", REPO_ROOT / "tests"], tracked_only=True)
    live = {
        Path(row["world"]).resolve().relative_to(REPO_ROOT).as_posix()
        for row in rows if not row.get("error")
    }
    live = {w for w in live if not w.startswith(UPSTREAM_CONTROL_ARMS)}
    added = sorted(live - recorded)
    removed = sorted(recorded - live)
    assert not added, (
        "these hand-authored wheeled worlds have never been roll-checked:\n  "
        + "\n  ".join(added)
        + "\n\nA wheeled robot that slides instead of rolling passes every other "
          "check in this repo. Measure them and record the verdicts:\n"
          "  python scripts/dev/roll_check.py sweep <world>... --json out.json\n"
          f"then add them to {BASELINE.relative_to(REPO_ROOT).as_posix()}."
    )
    assert not removed, (
        "these worlds are in the roll-check baseline but no longer exist (or no "
        "longer parse as wheeled robots):\n  " + "\n  ".join(removed)
        + f"\n\nDrop them from {BASELINE.relative_to(REPO_ROOT).as_posix()}."
    )


def test_every_non_rolling_world_carries_a_reason():
    baseline = _load_baseline()
    unexplained = sorted(
        world for world, row in baseline["worlds"].items()
        if row.get("verdict") not in CLEAN_VERDICTS and not row.get("note")
    )
    assert not unexplained, (
        "these worlds are recorded as not cleanly ROLLING but give no reason:\n  "
        + "\n  ".join(unexplained)
        + "\n\nEither fix the world or write down why the verdict is acceptable. "
          "An unexplained non-pass decays into background noise, which is how "
          "the original defect survived."
    )


def test_baseline_records_how_it_was_measured():
    baseline = _load_baseline()
    for key in ("tolerance", "binary", "machine", "measured_on", "method"):
        assert baseline.get(key), (
            f"the roll-check baseline is missing {key!r}. A verdict that does not "
            "name the machine and binary that produced it cannot be reproduced or "
            "challenged later."
        )
    assert baseline["tolerance"] == roll_check.TOL, (
        "the baseline was measured at tolerance %r but scripts/dev/roll_check.py "
        "now uses %r -- re-sweep, do not edit the number in place."
        % (baseline["tolerance"], roll_check.TOL)
    )


@pytest.mark.parametrize("path", [
    "scripts/dev/roll_check.py",
    "scripts/dev/wbt_wheels.py",
    "projects/default/controllers/roll_probe/roll_probe.py",
    "projects/default/controllers/roll_drive/roll_drive.py",
    "tests/physics/controllers/wheel_roll_noslip/wheel_roll_noslip.py",
])
def test_check_components_are_present(path):
    assert (REPO_ROOT / path).is_file(), f"{path} is missing -- the roll-check is broken"
