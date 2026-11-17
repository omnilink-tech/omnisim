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

"""Pin the Newton step path to known-good physics. Run after ANY change to it.

This exists because of a specific failure. On 2026-08-05 an "optimisation" of
the base-divergence guard removed a body_qd sync on the correct observation that
the snapshot was only read for its shape -- and missed that the same assignment
refreshes the `_body_qd_cache` readback cache, which the invalidation block
below it skips whenever the guard marks the caches fresh. `body_vel()` then
served a stale velocity for the rest of the run. Lane-1 T3's rolling sphere
stopped moving entirely (a_meas 2.395 -> 6.0e-18 against an analytic 2.3966) and
T7's conservation degraded ~100x.

Nothing caught it. The smoke suite passed. A hand-built 5-box benchmark world
stayed BIT-IDENTICAL to 9 decimal places across every A/B, because that scene
never reads a velocity. Bit-identity on one scene you wrote yourself is not a
physics regression test -- it only pins the paths that scene happens to touch.

So the reference here is the correctness lane, and the values are EXACT. The
Newton CPU path reproduced them digit-for-digit across a revert and across four
subsequent commits, so a mismatch in any digit is a real change in the physics,
not run-to-run noise. If a change is *supposed* to move these numbers, that is a
deliberate decision: update them in the same commit and say why.

Cost: ~2 minutes (three worlds, ~10 s of sim each plus engine startup). Too slow
for the pre-push smoke set, which is why it is a normal test you run when you
touch physics:

    python -m pytest tests/test_newton_physics_regression.py -v
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
LANE1 = REPO / "tests" / "benchmarks" / "omnibench" / "lane1"

#: test -> {metric: exact expected value}. Measured on the Newton CPU path
#: (newtonSolver "mujoco", mj_step) before any of the step-path performance
#: work, and unchanged by it.
REFERENCE = {
    "T3": {"roll_accel_rel_err": 0.0005821351711044256},
    "T6": {"stack_survivors": 10,
           "settle_creep_m_s": 5.687284136357966e-06},
    "T7": {"angmom_drift_rel": 2.54772084353779e-05,
           "rot_ke_drift_rel": 7.953878228986613e-08},
}

#: The engine has a known intermittent embedded-interpreter bring-up failure
#: ("can't initialize sys standard streams"). Since 85fa6bde a world that asked
#: for Newton is REFUSED rather than quietly run on ODE, so such a run produces
#: no data at all. That is an instrument outcome, not a physics regression, and
#: scoring it either way would be wrong -- so it is reported as a skip naming
#: the defect.
_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)


def _binary():
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = [
    pytest.mark.skipif(_binary() is None,
                       reason="no simulator binary in this clone; build first"),
    pytest.mark.skipif(not LANE1.is_dir(),
                       reason="omnibench lane 1 is not present in this tree"),
]


def _run_scene(test, out_dir):
    """Run one lane-1 scene on Newton and return its scored metrics."""
    run = subprocess.run(
        [sys.executable, str(LANE1 / "run_omnisim.py"), "--test", test,
         "--dt-ms", "4", "--backend", "newton", "--out", str(out_dir)],
        capture_output=True, text=True, timeout=600, cwd=str(REPO))
    if "-> OK" not in run.stdout:
        blob = run.stdout + run.stderr
        for sig in _BRINGUP_SIGNATURES:
            if sig in blob:
                pytest.skip("Newton did not come up for %s (known intermittent "
                            "bring-up defect: %r). The run produced no data, so "
                            "this says nothing about the physics -- re-run."
                            % (test, sig))
        # Also inspect the engine log, where the refusal is written.
        for log in sorted(out_dir.glob("*.engine.log")):
            text = log.read_text(encoding="utf-8", errors="replace")
            for sig in _BRINGUP_SIGNATURES:
                if sig in text:
                    pytest.skip("Newton did not come up for %s (%r)" % (test, sig))
        pytest.fail("lane-1 %s did not run:\n%s" % (test, blob[-1500:]))

    score = subprocess.run(
        [sys.executable, str(LANE1 / "score_omnisim.py"), "--test", test,
         "--dt-ms", "4", "--backend", "newton", str(out_dir)],
        capture_output=True, text=True, timeout=300, cwd=str(REPO))
    match = re.search(r'"metrics":\s*(\{.*?\})', score.stdout, re.S)
    if match:
        try:
            return json.loads(match.group(1))
        except ValueError:
            pass
    # Fall back to scraping only the keys this test asserts on, so a change to
    # the scorer's surrounding output does not read as a physics regression.
    scraped = {}
    for key in REFERENCE[test]:
        m = re.search(r'"%s":\s*([-\d.eE+]+)' % re.escape(key), score.stdout)
        if m:
            scraped[key] = float(m.group(1))
    if not scraped:
        pytest.fail("could not read %s metrics from the scorer:\n%s"
                    % (test, (score.stdout + score.stderr)[-1500:]))
    return scraped


@pytest.mark.parametrize("test", sorted(REFERENCE))
def test_newton_lane1_physics_unchanged(test, tmp_path):
    """Every lane-1 metric must match its reference EXACTLY."""
    metrics = _run_scene(test, tmp_path)
    wrong = []
    for metric, expected in REFERENCE[test].items():
        actual = metrics.get(metric)
        if actual is None:
            wrong.append("%s: MISSING from the scorer output" % metric)
        elif actual != expected:
            rel = abs(actual - expected) / abs(expected) if expected else float("inf")
            wrong.append("%s: got %r, reference %r (rel %.3g)"
                         % (metric, actual, expected, rel))
    assert not wrong, (
        "Newton physics changed on lane-1 %s:\n  %s\n\n"
        "These values are exact by construction -- the Newton CPU path "
        "reproduces them digit-for-digit -- so this is a real change, not "
        "noise. If the change was intended, update REFERENCE in this file in "
        "the same commit and record why." % (test, "\n  ".join(wrong)))
