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

"""A world that asked for Newton must never be quietly simulated by ODE.

The silent fall-back was retired on 2026-08-05. Before that, a Newton runtime
that was INSTALLED but would not come up produced a run that completed normally
on ODE, under a log saying Newton had been requested -- different contact
behaviour, different friction semantics, different contact visibility. Measured
on the cold-launch defect: 5 of 10 launches of a ``defaultPhysicsBackend
"newton"`` world degraded that way, and nothing downstream could tell those runs
from Newton runs. That corrupts results rather than losing them.

⚠️ RE-GOLDENED 2026-08-08 (src/ode DELETED, commit bdc02139). This file was
written while ODE still shipped, so its subject was "do not degrade to the OTHER
backend in silence". There is no other backend now, which makes the refusal
UNCONDITIONAL: the two escape hatches this file used to pin as working
(OMNISIM_ALLOW_ODE_FALLBACK=1 and OMNISIM_FORCE_ODE=1) are retired, and the two
tests that asserted they re-opened the run now assert they CANNOT. The engine
already says so in its own FATAL: "Newton is the only physics backend -- ODE has
been removed -- so there is no other backend to run this world on, and running it
on nothing would be a wrong result rather than a degraded one."

The distinction the engine now draws:

  MISSING  the runtime is not installed. Historically not a malfunction (an
           ODE-only clone had to keep working) and governed by
           OMNISIM_REQUIRE_NEWTON. Untouched by these tests.
  BROKEN   the runtime is installed and did not come up. A malfunction, and the
           run is refused -- FULL STOP, no hatch. See the re-golden note above
           the two hatch tests for why the exemptions had to go with the backend.

These tests use the engine's own fault injection (OMNISIM_NEWTON_SIMULATE_BROKEN)
because the real defect is intermittent, and a batch of healthy launches
verifies nothing about the unhealthy one. Shadowing the runtime from outside
does not work: the embedded interpreter ignores PYTHONPATH (verified by putting
a ModelBuilder-less `newton` stub on it and still getting "FFI smoke OK").
"""

from __future__ import annotations

import os
import platform
import subprocess
import textwrap
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
# The engine's refusal sentence CHANGED when src/ode was deleted (commit
# bdc02139). It used to be "Refusing to run it on ODE"; there is no ODE to
# refuse to run on any more, so it now says there is no other backend at all.
# Matched on the stable clause, not the whole sentence.
REFUSAL = "there is no other backend to run this world on"
FINALISED = "world finalised"

#: Long enough to reach the step dispatcher (where the refusal lives) and to
#: show that a healthy or permitted run keeps going rather than exiting.
RUN_S = 12


def _binary():
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        p = REPO / rel
        if p.is_file():
            return p
    return None


BIN = _binary()
pytestmark = pytest.mark.skipif(
    BIN is None, reason="no simulator binary in this clone; build first")


def _world(tmp_path, backend):
    """Smallest world that reaches the step dispatcher, on one backend."""
    w = tmp_path / ("w_%s.wbt" % backend)
    w.write_text(textwrap.dedent("""\
        #VRML_SIM R2025a utf8
        WorldInfo {
          basicTimeStep 8
          defaultPhysicsBackend "%s"
        }
        Viewpoint { orientation 0 0 1 0 position -4 0 2 }
        DEF FLOOR Solid {
          children [ Shape { geometry Plane { size 10 10 } } ]
          boundingObject Plane { size 10 10 }
        }
        DEF BALL Solid {
          translation 0 0 0.5
          children [ Shape { geometry Sphere { radius 0.1 } } ]
          boundingObject Sphere { radius 0.1 }
          physics Physics { }
        }
        """) % backend, encoding="utf-8")
    return w


def _run(world, log, env_extra):
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log))
    env.update(env_extra)
    timed_out = False
    try:
        subprocess.run([str(BIN), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(world)],
                       env=env, timeout=RUN_S, capture_output=True)
    except subprocess.TimeoutExpired:
        timed_out = True          # still running == did not refuse and exit
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    return timed_out, text


BROKEN = {"OMNISIM_NEWTON_SIMULATE_BROKEN": "1"}


def test_broken_runtime_refuses_instead_of_running_on_ode(tmp_path):
    """THE point of the change: refuse loudly, never degrade in silence."""
    still_running, log = _run(_world(tmp_path, "newton"), tmp_path / "a.txt", BROKEN)
    assert REFUSAL in log, (
        "a world asking for Newton was NOT refused when the runtime was "
        "broken. If it also never reached Newton, it ran on ODE silently -- "
        "the exact corruption this retirement exists to prevent.\n" + log[-2000:])
    assert FINALISED not in log, "claimed to refuse yet still finalised a Newton world"
    assert not still_running, "refusal must end the run, not warn and continue"


def test_explicit_ode_world_still_runs_while_newton_is_broken(tmp_path):
    """An explicit-ODE world is not refused for NEWTON's health. Narrow claim.

    Regression this pins: the refusal first went into the backend's constructor,
    which runs before the world is parsed -- so nobody had said which backend they
    wanted yet, and 4 of 4 explicit-ODE worlds were rejected. That ordering bug is
    what this test exists to catch, and it is still worth catching.

    ⚠️ READ THE SCOPE, 2026-08-08. This asserts the run is NOT REFUSED. It does
    NOT assert the world simulates, and since src/ode was DELETED (commit
    bdc02139) it demonstrably does not: `defaultPhysicsBackend "ode"` now names a
    backend with no implementation, so the BALL here does not fall. Measured the
    same day on tests/physics/worlds/contact_points.omniworld -- a body sat
    bit-identical to its authored pose for 3000 ms with no error anywhere. So the
    engine currently REFUSES a Newton world whose runtime is broken (correct: a
    wrong result is worse than a lost one) while HAPPILY RUNNING an explicit-ODE
    world that cannot produce a result at all. That asymmetry is an engine-side
    decision still open in src/omnisim/physics/ -- flagged here rather than left
    implied by a green test.
    """
    still_running, log = _run(_world(tmp_path, "ode"), tmp_path / "b.txt", BROKEN)
    assert REFUSAL not in log, (
        "a world that explicitly asked for ODE was refused because NEWTON was "
        "broken. Its choice was already answered.\n" + log[-2000:])
    assert still_running, "an explicit-ODE world should have run to the timeout"


# ---------------------------------------------------------------------------
# RE-GOLDENED 2026-08-08: the two escape hatches are GONE, and that is the
# stronger contract, not a regression.
#
# This file used to assert that OMNISIM_ALLOW_ODE_FALLBACK=1 restored
# degrade-and-continue, and that OMNISIM_FORCE_ODE=1 was never refused -- both
# on the reasoning that a caller who explicitly asks for ODE "has already had
# its answer". src/ode was DELETED (commit bdc02139), so neither hatch has
# anywhere to go: there is no ODE to degrade TO and none to be forced ONTO.
# Honouring either would produce a world the engine builds no physics for, and a
# wrong result is worse than a lost one -- which is the whole thesis of this
# file, now applied to its own escape hatches.
#
# The engine already reflects this (its FATAL reads "Newton is the only physics
# backend -- ODE has been removed -- so there is no other backend to run this
# world on, and running it on nothing would be a wrong result rather than a
# degraded one"). These two tests pin that the hatches CANNOT re-open it.
# ---------------------------------------------------------------------------

def test_the_allow_fallback_escape_hatch_no_longer_re_opens_the_run(tmp_path):
    """OMNISIM_ALLOW_ODE_FALLBACK=1 must NOT resurrect a deleted backend."""
    env = dict(BROKEN, OMNISIM_ALLOW_ODE_FALLBACK="1")
    still_running, log = _run(_world(tmp_path, "newton"), tmp_path / "c.txt", env)
    assert REFUSAL in log, (
        "OMNISIM_ALLOW_ODE_FALLBACK=1 suppressed the refusal. There is no ODE to "
        "fall back to (src/ode deleted, bdc02139), so the run would have "
        "continued with no physics implementation at all.\n" + log[-2000:])
    assert not still_running, "the retired hatch must not turn a FATAL into a warning"


def test_force_ode_no_longer_exempts_a_broken_newton_run(tmp_path):
    """OMNISIM_FORCE_ODE=1 is retired; it cannot buy a physics-free run."""
    env = dict(BROKEN, OMNISIM_FORCE_ODE="1")
    still_running, log = _run(_world(tmp_path, "newton"), tmp_path / "d.txt", env)
    assert REFUSAL in log, (
        "OMNISIM_FORCE_ODE=1 exempted the run from the refusal. That var is "
        "RETIRED (src/ode deleted, bdc02139) -- it selects a backend with no "
        "implementation, so the exemption buys a frozen scene, not legacy "
        "physics.\n" + log[-2000:])
    assert not still_running


@pytest.mark.skipif(platform.system() not in ("Windows", "Linux"),
                    reason="runtime bundling is only expected on these")
def test_healthy_runtime_is_unaffected(tmp_path):
    """The common path must be byte-unchanged: no refusal on a working runtime.

    Skipped rather than failed when Newton is simply absent -- that is MISSING,
    not BROKEN, and this file makes no claim about it.
    """
    _still, log = _run(_world(tmp_path, "newton"), tmp_path / "e.txt", {})
    if FINALISED not in log:
        pytest.skip("the Newton runtime did not drive this run; nothing to assert")
    assert REFUSAL not in log, "a HEALTHY Newton run was refused"
