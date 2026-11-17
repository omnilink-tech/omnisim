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

"""Unit test for the headless_runner "bodies registered, never finalised" guard.

`registered_but_never_finalised()` is what stops `run-headless --until-finalized`
from reporting PASS on a world whose physics build never completed. The case
that motivated it (2026-09-07): a 5000-static-box world logged its registration
census at t+8.4 s, then sat silent inside newton's finalize past the 30 s
ceiling; with no finalise line and no sidecar, the runner could not tell it from
`empty.omniworld` (which declares no bodies and legitimately finalises nothing)
and printed PASS with 0 errors. The census line is the evidence that separates
the two, and this test pins the exact line shapes the engine writes.

Run standalone:  python tests/test_headless_registered_never_finalised.py
Or via pytest:   pytest tests/test_headless_registered_never_finalised.py
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
_RUNNER = REPO_ROOT / "scripts" / "dev" / "headless_runner.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("headless_runner", _RUNNER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


runner = _load_runner()

# ── real engine line shapes (captured verbatim from omnisim_log.txt, 2026-09-07) ──
_OPENED = ("INFO: [OmNewtonBackend] world opened (implicit ground plane requested; "
           "added at finalize only if the world declared a Plane collider) [t+7696ms]")
_CENSUS = ("INFO: [OmNewtonBackend] registered 1 dynamic + 5001 static Newton bodies "
           "(+5002 this pass) (statics: solid, solid, solid, solid, solid, solid, solid, "
           "solid) [t+8351ms]")
_PREF = "INFO: [OmNewtonBackend] solver preference set to '' [t+8352ms]"
_FINALISED = ("INFO: [OmNewtonBackend] world finalised "
              "(solver=MuJoCo (cpu/mj_step, default)) [t+4807ms]")
_EMPTY_CENSUS = ("INFO: [OmNewtonBackend] registered 0 dynamic + 0 static Newton bodies "
                 "(+0 this pass) (statics: )")


def test_fires_on_census_without_finalise():
    log = "\n".join([_OPENED, _CENSUS, _PREF]) + "\n"
    assert runner.registered_but_never_finalised(log) == (1, 5001)


def test_silent_when_the_world_finalised():
    log = "\n".join([_OPENED, _CENSUS, _PREF, _FINALISED]) + "\n"
    assert runner.registered_but_never_finalised(log) is None


def test_silent_for_a_world_that_declares_no_bodies():
    # empty.omniworld: nothing registered, nothing to finalise -- a PASS.
    assert runner.registered_but_never_finalised(_OPENED + "\n") is None
    assert runner.registered_but_never_finalised(_OPENED + "\n" + _EMPTY_CENSUS + "\n") is None


def test_last_census_wins_across_reloads():
    # A hot-reload logs a second census; the verdict is about the LAST world.
    log = "\n".join([_CENSUS, _FINALISED,
                     "INFO: [OmNewtonBackend] registered 2 dynamic + 7 static Newton bodies "
                     "(+9 this pass) (statics: a, b)"]) + "\n"
    # The finalise line belongs to the earlier world, but the detector is a
    # whole-log test on purpose: the runner truncates the log per launch, so a
    # finalise anywhere means THIS launch finalised at least once.
    assert runner.registered_but_never_finalised(log) is None
    log2 = "\n".join([_CENSUS,
                      "INFO: [OmNewtonBackend] registered 2 dynamic + 7 static Newton bodies "
                      "(+9 this pass) (statics: a, b)"]) + "\n"
    assert runner.registered_but_never_finalised(log2) == (2, 7)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS", name)
