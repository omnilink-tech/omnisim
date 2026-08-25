#!/usr/bin/env python3
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

"""Unit test for the headless_runner "the sim never stepped" guard.

This proves the detection function `sim_never_stepped()` -- the ground-truth
check that a world which FINALISED under Newton but emitted zero engine step
lines gets reported as a FAIL, not a PASS. The exact IPC-nonce hang that
motivated the guard (commit 6eea9d76) can only be reproduced end-to-end with a
STALE libController, which this session is not allowed to rebuild; so the FIRES
direction is proven here on the real engine log format instead. The DOES-NOT-
FIRE direction is additionally proven end-to-end by the 60 s Go2 BATON deploy.

The fixtures below are the ACTUAL line shapes the engine writes to
omnisim_log.txt (see src/omnisim/physics/OmNewtonBackend.cpp:
`writeNewtonVerdictSidecar` / the finalize log line / the step-counter log). The
"hung" fixture is a healthy run's log with the step lines removed -- i.e. the
on-disk byte shape a real hang produces, because those steps never happened.

Run standalone:  python tests/test_headless_never_stepped.py
Or via pytest:   pytest tests/test_headless_never_stepped.py
"""

from __future__ import annotations

import importlib.util
import json
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
_RUNNER = REPO_ROOT / "scripts" / "dev" / "headless_runner.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("headless_runner", _RUNNER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


runner = _load_runner()

# ── real engine line shapes (captured verbatim from omnisim_log.txt) ─────────
_FINALISED = ("INFO: [OmNewtonBackend] world finalised "
              "(solver=MuJoCo (mujoco_warp, WorldInfo.newtonSolver))")
_STEP1 = ("INFO: [OmNewtonBackend] step 1 dt=0.016s "
          "b0=(-0.000,0.000,0.329) b1=(0.193,0.047,0.329)")
_STEP30 = ("INFO: [OmNewtonBackend] step 30 dt=0.016s "
           "b0=(-0.007,0.000,0.303) b1=(0.186,0.047,0.305)")
_PREAMBLE = (
    "=== OmniSim Log Started (pid=29800): 2026-07-17 15:56:50 ===\n"
    "WARNING: Robot \"go2\" > Solid \"imu\" > Physics : Invalid 'mass' changed to -1.\n"
    "INFO: sun_marker: Starting controller: python.exe -u sun_marker.py\n"
    "INFO: go2_baton_deploy: Starting controller: python.exe -u go2_baton_deploy.py\n"
    "INFO: [OmNewtonBackend] world opened (default ground plane added)\n"
    "INFO: [OmNewtonBackend] solver preference set to 'mujoco'\n"
)

# A genuinely working run: preamble -> finalised -> step lines.
LOG_WORKING = _PREAMBLE + _FINALISED + "\n" + _STEP1 + "\n" + _STEP30 + "\n"

# The HANG (the bug): the world finalised, the sidecar was written -- and then
# nothing. Not one step line, because the sim never stepped.
LOG_HUNG = _PREAMBLE + _FINALISED + "\n"

# A merely SLOW run terminated by --duration after a single step. This MUST NOT
# be flagged: step 1 proves the clock advanced.
LOG_SLOW = _PREAMBLE + _FINALISED + "\n" + _STEP1 + "\n"

# An ODE-only run (no Newton): neither a finalise line nor a step line. We can't
# measure stepping this way, so we must stay silent (no false FAIL).
LOG_ODE = (
    "=== OmniSim Log Started (pid=1): 2026-07-17 00:00:00 ===\n"
    "INFO: husky: Starting controller: python.exe -u drive_forward.py\n"
    "WARNING: some ordinary warning\n"
)

# A cold load that never reached finalize inside the duration: no finalise
# proof at all -> silent.
LOG_NO_FINALIZE = _PREAMBLE

LOG_CONTROLLER_FAILED = (
    _PREAMBLE
    + 'WARNING: humanoid_stand_deploy: failed to start controller: Python was not found.\n'
    + _FINALISED + "\n" + _STEP1 + "\n"
)


# ── the SAME shapes with the PRE-RENAME tag ──────────────────────────────────
# The bracketed tag is named after the emitting C++ class, and Phase C renamed
# those classes Wb* -> Om*. The fixtures above are the CURRENT (Om) shapes;
# these are the LEGACY (Wb) ones a user still has in any log captured before the
# rename. headless_runner's matchers accept BOTH prefixes permanently, so every
# verdict above must come out IDENTICAL on a Wb-tagged log. Mixed logs matter
# too: a user can concatenate captures from either side of the rename.
#
# NB this helper rewrites Om -> Wb, i.e. AWAY from the current tag. It used to
# go the other way, and when the Phase C sweep (7a1efd58d) rewrote every Wb*
# token in the tree it rewrote this function's SEARCH argument too, leaving
# `.replace("[OmNewtonBackend]", "[OmNewtonBackend]")` -- an identity function.
# Every WB_* constant below was then just an alias of its plain counterpart and
# the whole second arm silently re-tested the first. Keep the two tags distinct.
def _wb(text: str) -> str:
    return text.replace("[OmNewtonBackend]", "[WbNewtonBackend]")


WB_PREAMBLE = _wb(_PREAMBLE)
WB_FINALISED = _wb(_FINALISED)
WB_STEP1 = _wb(_STEP1)
WB_LOG_WORKING = _wb(LOG_WORKING)
WB_LOG_HUNG = _wb(LOG_HUNG)
WB_LOG_SLOW = _wb(LOG_SLOW)

assert WB_PREAMBLE != _PREAMBLE, "_wb must rewrite the tag, not return its input"


def check(name: str, got, want) -> bool:
    ok = got == want
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: got={got!r} want={want!r}")
    return ok


def test_sim_never_stepped() -> bool:
    ok = True
    f = runner.sim_never_stepped
    # FIRES: finalised (via log line) but no step line.
    ok &= check("hung via log line -> FIRES", f(LOG_HUNG, False), True)
    # FIRES: finalised (via sidecar bool) even if the finalise log line were
    # somehow absent -- and still no step line.
    ok &= check("hung via sidecar flag -> FIRES",
                f(_PREAMBLE, True), True)
    # DOES NOT FIRE: working run has step lines.
    ok &= check("working run -> silent", f(LOG_WORKING, True), False)
    # DOES NOT FIRE: a single step (slow run) is enough.
    ok &= check("slow run, one step -> silent", f(LOG_SLOW, True), False)
    # DOES NOT FIRE: ODE run -- no finalize proof, cannot measure.
    ok &= check("ODE run -> silent", f(LOG_ODE, False), False)
    # DOES NOT FIRE: never reached finalize -- no proof, stay silent.
    ok &= check("no-finalize cold load -> silent", f(LOG_NO_FINALIZE, False), False)
    # Guard against a substring false-positive: "world finalised" must not, by
    # itself, be mistaken for a step; and a real "substep"-style token must not
    # be mistaken for a step line either.
    ok &= check("finalise-only + 'substep' noise -> FIRES",
                f(_PREAMBLE + _FINALISED + "\nINFO: substep budget 8\n", False), True)
    # A bare `return ok` cannot fail under pytest: a returned False is still a
    # PASS (it only raises PytestReturnNotNoneWarning). Assert so the ~15 checks
    # in these three tests can actually go red. main() still uses the return.
    assert ok
    return ok


def test_sim_never_stepped_legacy_wb_tag() -> bool:
    """The legacy "[WbNewtonBackend]" tag must behave exactly like "[Om...]".

    This is the dual-accept contract, under test rather than merely asserted in a
    comment: if someone narrows the matchers to the current prefix only, this
    goes red instead of silently mis-reading every pre-rename log a user still
    has on disk.
    """
    ok = True
    f = runner.sim_never_stepped
    ok &= check("Wb: hung via log line -> FIRES", f(WB_LOG_HUNG, False), True)
    ok &= check("Wb: working run -> silent", f(WB_LOG_WORKING, True), False)
    ok &= check("Wb: slow run, one step -> silent", f(WB_LOG_SLOW, True), False)
    ok &= check("Wb: no-finalize cold load -> silent", f(WB_PREAMBLE, False), False)
    ok &= check("Wb: finalise-only + 'substep' noise -> FIRES",
                f(WB_PREAMBLE + WB_FINALISED + "\nINFO: substep budget 8\n", False), True)
    # MIXED logs -- both directions. Each half is recognised, so a healthy run
    # stays healthy no matter which side of the rename produced which line.
    ok &= check("mixed: Om finalise + Wb step -> silent",
                f(_PREAMBLE + _FINALISED + "\n" + WB_STEP1 + "\n", False), False)
    ok &= check("mixed: Wb finalise + Om step -> silent",
                f(WB_PREAMBLE + WB_FINALISED + "\n" + _STEP1 + "\n", False), False)
    # NEGATIVE CONTROL: the matchers dual-accept Wb|Om, they do not accept ANY
    # prefix. A third prefix must go unrecognised, or the patterns are too loose
    # and would match unrelated log lines.
    xx = LOG_WORKING.replace("[OmNewtonBackend]", "[XxNewtonBackend]")
    ok &= check("Xx prefix: no finalise proof -> silent (unmeasurable)",
                f(xx, False), False)
    ok &= check("Xx prefix: step line unrecognised -> FIRES via sidecar",
                f(xx, True), True)
    # A bare `return ok` cannot fail under pytest: a returned False is still a
    # PASS (it only raises PytestReturnNotNoneWarning). Assert so the checks in
    # this test can actually go red. main() still uses the return.
    assert ok
    return ok


def test_sidecar_finalised() -> bool:
    ok = True
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "omnisim_log.txt.newton.json"
        # Real sidecar shape.
        p.write_text(json.dumps(
            {"backend": "newton", "degraded": False, "finalised": True,
             "solver": "MuJoCo (mujoco_warp, WorldInfo.newtonSolver)"}))
        ok &= check("sidecar finalised=true", runner._sidecar_finalised(p), True)
        p.write_text(json.dumps({"finalised": False}))
        ok &= check("sidecar finalised=false", runner._sidecar_finalised(p), False)
        p.write_text("{ this is not json ")
        ok &= check("sidecar corrupt -> False", runner._sidecar_finalised(p), False)
        missing = Path(d) / "does_not_exist.newton.json"
        ok &= check("sidecar missing -> False", runner._sidecar_finalised(missing), False)
    # A bare `return ok` cannot fail under pytest: a returned False is still a
    # PASS (it only raises PytestReturnNotNoneWarning). Assert so the ~15 checks
    # in these three tests can actually go red. main() still uses the return.
    assert ok
    return ok


def test_controller_start_failures() -> bool:
    ok = True
    failures = runner.controller_start_failures(LOG_CONTROLLER_FAILED)
    ok &= check("failed Python controller detected", len(failures), 1)
    ok &= check("healthy run has no controller failure",
                runner.controller_start_failures(LOG_WORKING), [])
    # Repeated scans/duplicate engine echoes should produce one useful verdict line.
    doubled = LOG_CONTROLLER_FAILED + LOG_CONTROLLER_FAILED
    ok &= check("duplicate failure line deduplicated",
                len(runner.controller_start_failures(doubled)), 1)
    # A bare `return ok` cannot fail under pytest: a returned False is still a
    # PASS (it only raises PytestReturnNotNoneWarning). Assert so the ~15 checks
    # in these three tests can actually go red. main() still uses the return.
    assert ok
    return ok


def main() -> int:
    print("test_sim_never_stepped:")
    a = test_sim_never_stepped()
    print("test_sim_never_stepped_legacy_wb_tag:")
    wb = test_sim_never_stepped_legacy_wb_tag()
    print("test_sidecar_finalised:")
    b = test_sidecar_finalised()
    print("test_controller_start_failures:")
    c = test_controller_start_failures()
    passed = a and wb and b and c
    print("RESULT:", "ALL PASS" if passed else "FAILURES ABOVE")
    return 0 if passed else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
