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

"""Deliberately wrong MuJoCo runs, so the column's assertions can be seen to FAIL.

The red-evidence rule, standing and binding
(``docs/developer/agent-edge-validation-plan.md`` §5.5, quoted in
``capability-ladder-plan.md`` §5c rule 2): **no assertion enters a ladder cell
until it has been observed failing on a deliberately wrong artifact, with that
negative fixture named in the assertion's record.** A green assertion is not
evidence that the assertion works -- for weeks half of ``A1.3`` could not fail
at all, because every contact pair was keyed ``(id, id)``.

``ladder/graders/fixtures.py`` already supplies synthetic negatives for the
**core**. This module supplies them for the **column**: each defect below is a
real MuJoCo run, through the real builder, the real recorder and the real
evidence builder, so what goes red is the whole path and not a hand-written
array. That distinction matters here because two of the three defects are not
inventions -- they are the failure modes MuJoCo's own URDF defaults produce.

===============  =========================================================
defect           what it is, and which assertion it must turn red
===============  =========================================================
``welded``       (also turns T1.4 red, honestly: with the base pinned at its
                 authored spawn height the wheels hang 0.12 m clear of the
                 floor and touch nothing.)
                 the ``<freejoint>`` is stripped from the scene, which is
                 **exactly what a raw URDF load gives you**: a URDF root
                 link has no joint to the world, so MuJoCo welds it and the
                 wheels spin in place for ever. Must fail **T1.1** (never
                 arrives) and **T1.3** (a path length of ~0 against a 5 m
                 straight line), and must NOT disturb T1.2 or T1.5 -- a
                 chassis bolted to the world is still upright and the run is
                 still a real run. This fixture earned its keep on its first
                 execution: it exposed an identity rule that keyed on the
                 free joint, so a welded robot read as *no robot in the
                 scene* and collapsed the whole verdict instead of failing
                 two assertions.
``no_ground``    the ground body is stripped. The robot falls for ever.
                 Must fail **T1.2** (it did not stay up) and **T1.4** (no
                 support contact) -- and T1.4 must fail with its vacuity
                 witnesses PRESENT and zero, so "it never touched anything"
                 is distinguishable from "the contact query is broken".
``will_not_compile``
                 an invalid geom type is spliced into the scene, so MuJoCo
                 refuses to load the deliverable at all. **Every** assertion
                 must go red including **T1.5**, which is the point: a
                 deliverable that does not run is not a partial success, and
                 T1.5 has no other route to red on this column (MuJoCo's hard
                 errors abort the process rather than printing a line).

``teleport``     the base's free-joint ``qpos`` is written directly toward
                 the goal in 1 m jumps, with every actuator at zero. It
                 *arrives*, so **T1.1 must still pass** -- and **T1.3 must
                 fail** on the inter-sample displacement bound. This is the
                 fixture that proves T1.3 is a teleport test rather than
                 decoration.
===============  =========================================================

Run them all::

    python tests/benchmarks/ladder/adapters/mujoco/negatives.py --out <dir>

The summary prints, per defect, which assertions went red and whether that
matches what the defect was built to break. A defect that does **not** turn its
target assertion red is a finding about the grader, and it is printed as a
mismatch rather than swallowed.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from ladder.adapters.mujoco import runner                     # noqa: E402
from ladder.adapters.mujoco.run_t1 import build_t1_evidence   # noqa: E402
from ladder import tasks as ladder_tasks                      # noqa: E402

# defect -> the assertion ids it exists to turn red.
EXPECTED_RED = {
    "welded": ("T1.1", "T1.3", "T1.4"),
    "no_ground": ("T1.1", "T1.2", "T1.3", "T1.4"),
    "teleport": ("T1.3",),
    "will_not_compile": ("T1.1", "T1.2", "T1.3", "T1.4", "T1.5"),
}

# ...and the ones it must NOT turn red, because a fixture that reddens
# everything proves nothing (``ladder_evidence``: "a null agent turning every
# assertion red does not satisfy this rule").
EXPECTED_GREEN = {
    "welded": ("T1.2", "T1.5"),
    "no_ground": ("T1.5",),
    "teleport": ("T1.1", "T1.2", "T1.4", "T1.5"),
    "will_not_compile": (),
}

# Simulated seconds of driving allowed to a defective run (see run_defect).
NEGATIVE_DRIVE_S = 12.0

_FREEJOINT = re.compile(r'\s*<(?:free)?joint[^>]*type="free"[^>]*/>\s*'
                        r'|\s*<freejoint[^>]*/>\s*')
_GROUND = re.compile(r'\s*<body name="ground">.*?</body>\s*', re.S)


def break_scene(scene_path, defect, out_path):
    """Write a deliberately broken copy of ``scene_path``. Returns the path."""
    text = Path(scene_path).read_text(encoding="utf-8")
    if defect == "welded":
        broken, n = _FREEJOINT.subn("\n    ", text, count=1)
        if not n:
            raise ValueError("no free joint found in %s to remove"
                             % scene_path)
    elif defect == "will_not_compile":
        broken, n = re.subn(
            r"<worldbody>",
            '<worldbody>\n    <geom type="nonesuch" size="1"/>',
            text, count=1)
        if not n:
            raise ValueError("no <worldbody> in %s to break" % scene_path)
    elif defect == "no_ground":
        broken, n = _GROUND.subn("\n  ", text, count=1)
        if not n:
            raise ValueError("no ground body found in %s to remove"
                             % scene_path)
    else:
        broken = text
    Path(out_path).write_text(broken, encoding="utf-8")
    return str(out_path)


def run_defect(defect, out_dir, task):
    """One defective run + its verdict. Returns ``(verdict, run_dir)``."""
    out = Path(out_dir) / defect
    out.mkdir(parents=True, exist_ok=True)
    phase = task.standalone
    desc = task.container_dir / Path(
        task.meta["container"]["description_dir"]).name
    offset = tuple(task.waypoint_spec.get("offset_m", [0.0, 5.0]))
    # A shorter drive window than the task's: a defective run never arrives,
    # so it always spends the whole budget, and 60 s of per-step samples for
    # every body is tens of megabytes of JSON saying the same thing. The
    # window is still far longer than the 2.0 s dwell any assertion needs.
    extra = ["--settle-s", str(phase.get("settle_s", 0.5)),
             "--drive-timeout-s", str(NEGATIVE_DRIVE_S),
             "--contact-stride", str(phase.get("contact_stride", 10))]

    if defect == "teleport":
        extra += ["--defect", "teleport"]
        runner.launch(out, goal_offset=offset, description=str(desc),
                      extra_args=extra)
    else:
        # Build once with the good builder, break the SCENE FILE, re-run from
        # the broken file. Breaking the artifact rather than the builder is
        # what makes this a fixture about the deliverable.
        good = Path(out_dir) / ("_build_%s" % defect)
        runner.launch(good, goal_offset=offset, description=str(desc),
                      extra_args=extra + ["--drive-timeout-s", "0.2"])
        build = json.loads((good / "build.json").read_text(encoding="utf-8"))
        # The broken copy goes NEXT TO the original, not into the run dir:
        # the scene's compiler/meshdir is relative to the file's own location,
        # so a copy written elsewhere fails to load for a reason that has
        # nothing to do with the defect under test. (Found the hard way -- the
        # first pass produced three all-red verdicts that were really one
        # missing mesh path.)
        broken = break_scene(
            build["scene"], defect,
            Path(build["scene"]).parent / ("broken_%s.xml" % defect))
        (out / "build.json").write_text(
            json.dumps(dict(build, scene=broken,
                            deliberate_defect=defect), indent=1),
            encoding="utf-8")
        runner.launch(out, goal_offset=offset, scene=broken, extra_args=extra)

    from ladder.graders import t1_core
    ev = build_t1_evidence(out, task=task)
    return t1_core.grade(ev), out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--defect", action="append", choices=sorted(EXPECTED_RED),
                    help="run only these (default: all)")
    a = ap.parse_args(argv)

    task = ladder_tasks.get("T1_arrive")
    rows, mismatches = [], []
    for defect in (a.defect or sorted(EXPECTED_RED)):
        verdict, out = run_defect(defect, a.out, task)
        red = sorted(x.id for x in verdict.assertions if not x.ok)
        want_red = set(EXPECTED_RED[defect])
        want_green = set(EXPECTED_GREEN[defect])
        missed = sorted(want_red - set(red))
        wrongly = sorted(want_green & set(red))
        rows.append({"defect": defect, "outcome": verdict.outcome,
                     "red": red, "expected_red": sorted(want_red),
                     "not_reddened": missed,
                     "collateral": wrongly, "run_dir": str(out)})
        if missed or wrongly:
            mismatches.append(defect)
        print("\n=== %s -> %s ===" % (defect, verdict.outcome))
        print("  red: %s" % (", ".join(red) or "(none)"))
        if missed:
            print("  !! expected red and was NOT: %s" % ", ".join(missed))
        if wrongly:
            print("  !! collateral damage (should have stayed green): %s"
                  % ", ".join(wrongly))
        for x in verdict.assertions:
            if not x.ok:
                print("     %s: %s" % (x.id, json.dumps(x.measured,
                                                         default=str)[:220]))

    print("\n=== red-evidence summary ===")
    covered = sorted({aid for d in rows for aid in d["red"]})
    print("assertions observed FAILING through the real MuJoCo path: %s"
          % (", ".join(covered) or "(none)"))
    print("mismatches: %s" % (", ".join(mismatches) or "none"))
    Path(a.out, "negatives.json").write_text(
        json.dumps({"rows": rows, "assertions_reddened": covered},
                   indent=2), encoding="utf-8")
    return 1 if mismatches else 0


__all__ = ["EXPECTED_GREEN", "EXPECTED_RED", "break_scene", "run_defect"]


if __name__ == "__main__":
    sys.exit(main())
