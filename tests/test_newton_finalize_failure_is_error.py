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

"""A world that gets NO physics at all must fail the default validation lane.

THE DEFECT. `OmNewtonBackend::finalizeWorld()` reported a raise out of
`world.finalize()` through `reportPyError()`, i.e. at WARNING level. But a raise
there means `SolverMuJoCo` was never constructed, so the world gets no Newton
world AT ALL: no solver, no contacts, no integration, every body frozen at its
authored pose for the whole run -- and Newton is the only backend, so there is
nothing to degrade to. `run-headless` grades a run by counting log lines that
start with `ERROR:`/`FATAL:`, so a world with no physics whatsoever printed
`0 errors, N warnings ... PASS` and exited 0.

Neither of the other two lanes could see it either. `--fail-on-runaway` cannot:
a body frozen at its authored pose is indistinguishable from one legally still
mid-air. `--until-finalized` cannot: it proves load + finalize, and this IS the
finalize failing. Only `--fail-on-warning` caught it, which nothing runs by
default.

WHAT THIS TEST PINS. Two worlds that differ by ONE node -- a second HingeJoint
whose `endPoint` is a `SolidReference` back to the first joint's endpoint, i.e.
a closed kinematic loop, which MuJoCo (a tree-articulation solver) cannot
represent and refuses with `ValueError: Multiple joints lead to body N`:

  world     loop joint   before the fix          after the fix
  broken    present      PASS, exit 0            FAIL, exit 1
  control   absent       PASS, exit 0            PASS, exit 0

The control half is the regression guard and is not optional: promoting a
severity is only safe if it leaves the working world alone, and "the broken one
fails" proves nothing on its own -- an engine that failed EVERY world would
satisfy it.

Three further assertions keep the verdict honest rather than merely red:

  * the ERROR must NAME THE CAUSE. A bare "finalize failed" would pass a
    grep-for-ERROR test while telling the author nothing, so the message is
    required to carry the Python exception text ("Multiple joints lead to
    body"). The engine's own words are the only place the cause exists.
  * the ERROR must be emitted ONCE. A failed finalize builds nothing, so the
    world never closes for build and `OmSimulationWorld::step()` calls back in
    on every tick; unlatched, the measured world logged 4254 copies of the same
    line and buried it. One world, one report.
  * the two runs must differ in the ENGINE'S OWN physics verdict, not just in
    our log grep: the control writes a `<log>.newton.json` sidecar (Newton
    finalised it) and the broken one cannot.

    python -m pytest tests/test_newton_finalize_failure_is_error.py -v

Cost: two engine launches, roughly 30 s. Newton-only; there is no other backend.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

#: The exception newton raises out of `topological_sort` for a closed loop. The
#: message must carry this, not merely the fact that something went wrong.
EXPECTED_CAUSE = "Multiple joints lead to body"

#: The verdict the message has to lead with. Checked as a substring of the FIRST
#: line, because the Python detail is a wrapped traceback: a message that
#: appended the consequence AFTER the detail put this sentence ten lines below
#: the header, past every log tail and past the one line run-headless echoes.
VERDICT = "THIS WORLD HAS NO PHYSICS"

#: Known intermittent embedded-interpreter bring-up failure. A run that never
#: got a Newton runtime says nothing about how a finalize failure is reported.
_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "embedded Python init failed",
)

_COMMON = """#OMNISIM R2025a utf8
WorldInfo {
  basicTimeStep 8
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
  newtonStatics TRUE
}
Viewpoint {
  orientation 0 0 1 0
  position -6 0 2
}
DEF FLOOR Solid {
  translation 0 0 0.4
  name "floor"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.5 0.5 0.55 roughness 1 metalness 0 }
      geometry Box { size 6 6 0.3 }
    }
  ]
  boundingObject Box { size 6 6 0.3 }
}
DEF WITNESS Solid {
  translation 2 0 3
  name "witness"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.65 0.85 roughness 0.9 metalness 0 }
      geometry Box { size 0.2 0.2 0.2 }
    }
  ]
  boundingObject Box { size 0.2 0.2 0.2 }
  physics Physics { density -1 mass 1 }
}
DEF RIG Robot {
  translation 0 0 1.2
  name "rig"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.4 0.2 roughness 1 metalness 0 }
      geometry Box { size 0.4 0.4 0.1 }
    }
    HingeJoint {
      jointParameters HingeJointParameters { anchor 0 0 -0.2 }
      endPoint DEF AXLE Solid {
        translation 0 0 -0.2
        name "front axle"
        children [
          Shape {
            appearance PBRAppearance { baseColor 0.2 0.2 0.2 roughness 1 metalness 0 }
            geometry Box { size 0.3 0.1 0.1 }
          }
        ]
        boundingObject Box { size 0.3 0.1 0.1 }
        physics Physics { density -1 mass 0.5 }
      }
    }
%s  ]
  boundingObject Box { size 0.4 0.4 0.1 }
  physics Physics { density -1 mass 1 }
  controller "<none>"
}
"""

#: The ONLY difference between the two worlds: a second joint arriving at a body
#: that already has a joint parent. `WITNESS` is deliberately unrelated to it --
#: a free box two metres away -- because the blast radius of this failure is the
#: whole world, not the malformed robot.
_LOOP_JOINT = """    HingeJoint {
      jointParameters HingeJointParameters { anchor 0 0 -0.35 }
      endPoint SolidReference { solidName "front axle" }
    }
"""


def _binary():
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def _run(tmp_path, tag, loop):
    """Run one world through the DEFAULT validation lane and report on it.

    Deliberately `python -m omnisim run-headless` and not a raw binary launch:
    the claim under test is about the exit code of the lane an agent is told to
    use, and that verdict is produced by the runner, not by the engine.
    """
    work = tmp_path / tag
    (work / "worlds").mkdir(parents=True, exist_ok=True)
    world = work / "worlds" / ("%s.omniworld" % tag)
    world.write_text(_COMMON % (_LOOP_JOINT if loop else ""), encoding="utf-8")

    log = work / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), OMNISIM_LOG_PATH=str(log))
    # A stale export in the developer's shell must not decide the answer.
    for stale in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY", "OMNISIM_ALLOW_ODE_FALLBACK",
                  "OMNISIM_NEWTON_SIMULATE_BROKEN"):
        env.pop(stale, None)

    proc = subprocess.run(
        [sys.executable, "-m", "omnisim", "run-headless", str(world), "--duration", "10"],
        cwd=str(REPO), env=env, timeout=600, capture_output=True, text=True)

    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    for sig in _BRINGUP_SIGNATURES:
        if sig in text:
            pytest.skip("Newton did not come up for %r (%r)" % (tag, sig))

    lines = text.splitlines()
    return {
        "returncode": proc.returncode,
        "stdout": proc.stdout,
        "errors": [ln for ln in lines if ln.startswith(("ERROR:", "FATAL:"))],
        "verdicts": [ln for ln in lines if VERDICT in ln],
        # The engine's own physics verdict. OmLog deletes any stale copy when it
        # truncates the log at startup, so its presence means "Newton finalised
        # THIS run" -- it cannot be inherited from a previous one.
        "sidecar": Path(str(log) + ".newton.json"),
        "log": text,
    }


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    root = tmp_path_factory.mktemp("finalizefail")
    return {"broken": _run(root, "broken", loop=True),
            "control": _run(root, "control", loop=False)}


def test_control_world_still_passes(runs):
    """The regression half: the working world must be untouched.

    Same world minus the loop joint. If this goes red, the severity promotion
    is over-broad and the fix is worse than the bug it closes.
    """
    r = runs["control"]
    assert r["returncode"] == 0, (
        "the CONTROL world (no loop joint) must still PASS -- it is the same "
        "world as the broken one minus one node.\nexit=%s\nERROR lines:\n  %s"
        % (r["returncode"], "\n  ".join(r["errors"]) or "(none)"))
    assert not r["errors"], (
        "the control world logged ERROR lines it did not log before:\n  %s"
        % "\n  ".join(r["errors"]))
    assert r["sidecar"].is_file(), (
        "no %s: Newton never finalised the control world, so this pair is not a "
        "valid differential -- both halves would be 'no physics' for different "
        "reasons." % r["sidecar"].name)
    verdict = json.loads(r["sidecar"].read_text(encoding="utf-8"))
    assert verdict.get("finalised") is True, verdict


def test_finalize_failure_fails_the_run(runs):
    """The defect itself: no physics at all must not exit 0."""
    r = runs["broken"]
    assert r["returncode"] != 0, (
        "a world with NO PHYSICS AT ALL exited 0.\n"
        "Its finalize raised, so no Newton world was built and every body sat "
        "frozen at its authored pose for the whole run -- and the default "
        "validation lane called that a PASS.\nrun-headless said: %s"
        % r["stdout"].strip().splitlines()[-2:])
    assert not r["sidecar"].is_file(), (
        "%s exists, so Newton DID finalise this world -- the reproducer no "
        "longer reproduces and this test is measuring nothing."
        % r["sidecar"].name)


def test_the_error_names_the_cause(runs):
    """A red exit code that does not say why is only half a diagnosis."""
    r = runs["broken"]
    assert r["verdicts"], (
        "no log line carried %r. The run may have failed for some unrelated "
        "reason; ERROR lines were:\n  %s"
        % (VERDICT, "\n  ".join(r["errors"]) or "(none)"))
    line = r["verdicts"][0]
    assert line.startswith("ERROR:"), (
        "the no-physics verdict was logged at the wrong level -- run-headless "
        "counts only lines starting with ERROR:/FATAL:, so anything else is "
        "invisible to the default lane again:\n  %s" % line[:200])
    assert EXPECTED_CAUSE in line, (
        "the ERROR does not carry the Python exception, so it cannot tell the "
        "author WHICH world defect this is. Expected %r in:\n  %s"
        % (EXPECTED_CAUSE, line[:600]))


def test_the_error_is_reported_once(runs):
    """One world, one report -- the failure is retried on every tick.

    A failed finalize builds nothing, so the world never closes for build and
    the engine calls finalizeWorld() again next step. Unlatched, the measured
    world produced 4254 copies of this line, which is how the one message that
    mattered got buried.
    """
    r = runs["broken"]
    assert len(r["verdicts"]) == 1, (
        "expected exactly 1 no-physics ERROR, got %d. The per-world latch in "
        "OmNewtonBackend::finalizeWorld() is not holding, and a 10 s run is a "
        "short one -- a long run floods the log (measured 1.7 MB in ~55 s) and "
        "times out the harness's supervisor RPC."
        % len(r["verdicts"]))
