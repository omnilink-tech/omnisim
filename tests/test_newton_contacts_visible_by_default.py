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

"""A body that is demonstrably resting on a floor must SAY it has a contact.

Newton's native contact readback was opt-in behind OMNISIM_NEWTON_NATIVE_CONTACTS.
With it off, a Newton-backed Solid falls through to the ODE-bridge contact list --
and in a Newton world that list is empty for it, because the bridge's ODE proxy
body is deliberately disabled (OmSolid.cpp, `setBodyArtificiallyDisabled(true)`:
"contacts don't fire") and OmSimulationCluster's near-callback returns before
appending a contact when one side is a kinematic static and the other's body is
disabled. So `getContactPoints` answered [] for a block a gripper was visibly
holding, which is indistinguishable from "nothing is touching".

This pins the flip: contacts are visible BY DEFAULT, and the escape hatch is an
explicit OMNISIM_NEWTON_NATIVE_CONTACTS=0.

WHAT EACH TEST DOES ON WHICH BUILD
  TODAY (gate is `qEnvironmentVariableIsSet`, so ANY value -- including "0" --
  turns the native path ON, and unset turns it OFF):
    default run      -> 0 contacts     test_contacts_are_visible_by_default   FAILS
    "0" opt-out run  -> >= 1 contacts  test_explicit_opt_out_is_honoured      FAILS
    "1" run          -> >= 1 contacts  test_explicit_opt_in_still_works       passes
    default run log  -> blindness warning present
                                       test_default_run_does_not_warn         FAILS
  AFTER the flip (value-parsing gate, default ON):
    all four pass.

The rest-height assertion is deliberately independent of the contact query: it
proves from geometry alone that a contact EXISTS to be seen, so a zero from
`getContactPoints` cannot be excused as "the block really was in mid-air". It
passes on both builds and is the reason the contact counts mean anything.

The world pins `newtonStatics TRUE` on purpose rather than relying on the
default: whether that default flips is a separate decision, and this test must
measure contact VISIBILITY, not statics registration.
"""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

# Same signatures the sibling Newton tests use to tell "this clone cannot bring
# Newton up" apart from "Newton came up and the answer is wrong".
_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)

# The warning OmSolid::extractContactPoints emits when a Newton-backed Solid
# cannot see its own contacts. Matched on the OPENING clause only, which is
# stable across the wording change the flip makes to the trailing advice.
_BLINDNESS_WARNING = "Contact queries on Newton-backed Solids return an EMPTY set"

# A 0.2 m cube resting on a floor whose top surface is z = 0. Box-on-box rather
# than sphere-on-box because a box cannot roll out of contact, and because it
# yields several contact points rather than a single one -- the assertion is
# ">= 1", so either would do, but a rolled-away sphere would be a flaky zero.
#
# The floor top sits at exactly z = 0 so that the answer does not depend on
# WorldInfo.newtonStatics: with statics registered the block rests on the Box,
# without them it rests on the implicit ground plane Newton adds at z = 0. Same
# height, same contact, either way -- so this world measures the contact QUERY
# and nothing else.
WORLD = """#VRML_SIM R2025a utf8
WorldInfo {
  basicTimeStep 8
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
  newtonStatics TRUE
}
Viewpoint { orientation 0 0 1 0 position -3 0 1 }
DEF FLOOR Solid {
  translation 0 0 -0.25
  children [ Shape { geometry Box { size 4 4 0.5 } } ]
  name "floor"
  boundingObject Box { size 4 4 0.5 }
}
DEF BLOCK Solid {
  translation 0 0 0.1005
  children [ Shape { geometry Box { size 0.2 0.2 0.2 } } ]
  name "block"
  boundingObject Box { size 0.2 0.2 0.2 }
  physics Physics { mass 1 }
}
DEF PROBE Robot {
  translation 0 1.5 0.5
  name "probe"
  controller "contactprobe"
  supervisor TRUE
  children [ Shape { geometry Box { size 0.05 0.05 0.05 } } ]
}
"""

# Settle, then report the block's height, its speed, and what the contact query
# says -- all read in the SAME tick, so "at rest" and "no contacts" are claims
# about one instant and can be held against each other.
CONTROLLER = "\n".join([
    "import os",
    "from omnisim import Supervisor",
    "",
    "out = open(os.environ['PROBE_OUT'], 'w', buffering=1)",
    "sup = Supervisor()",
    "dt = int(sup.getBasicTimeStep())",
    "for _ in range(220):",              # 220 * 8 ms = 1.76 s -- ample to settle a 0.5 mm drop
    "    if sup.step(dt) == -1:",
    "        break",
    "",
    "block = sup.getFromDef('BLOCK')",
    "floor = sup.getFromDef('FLOOR')",
    "if block is None:",
    "    out.write('BLOCK missing\\n')",
    "else:",
    "    p = block.getPosition()",
    "    v = block.getVelocity()",
    "    out.write('block_z %.9f\\n' % p[2])",
    "    out.write('block_speed %.9f\\n' % max(abs(c) for c in v[0:3]))",
    "    out.write('block_contacts %d\\n' % len(block.getContactPoints(False) or []))",
    "if floor is not None:",
    "    out.write('floor_contacts %d\\n' % len(floor.getContactPoints(False) or []))",
    "out.write('done\\n')",
    "out.close()",
    "sup.simulationQuit(0)",
    "",
])


def _binary():
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def _run_once(tmp_path: Path, label: str, native_contacts: str | None) -> dict:
    """One headless run. `native_contacts` is the value of
    OMNISIM_NEWTON_NATIVE_CONTACTS, or None to leave it UNSET.
    """
    root = tmp_path / label
    worlds = root / "worlds"
    ctrl = root / "controllers" / "contactprobe"
    worlds.mkdir(parents=True, exist_ok=True)
    ctrl.mkdir(parents=True, exist_ok=True)
    (worlds / "contacts.wbt").write_text(WORLD, encoding="utf-8")
    (ctrl / "contactprobe.py").write_text(CONTROLLER, encoding="utf-8")

    result = root / "probe_out.txt"
    log = root / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(result),
               OMNISIM_LOG_PATH=str(log))
    # This test is about the NEWTON contact path; a forced-ODE environment would
    # answer a different question entirely (and would answer it "yes" on both
    # builds, hiding the defect).
    for var in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY", "OMNISIM_NEWTON_NATIVE_CONTACTS"):
        env.pop(var, None)
    if native_contacts is not None:
        env["OMNISIM_NEWTON_NATIVE_CONTACTS"] = native_contacts

    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(worlds / "contacts.wbt")],
                       env=env, timeout=300, capture_output=True)
    except subprocess.TimeoutExpired:
        pass

    engine_log = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""

    if not result.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in engine_log:
                pytest.skip("Newton did not come up (%r)" % sig)
        pytest.fail("run %r produced no probe output:\n%s" % (label, engine_log[-1500:]))

    # Did Newton actually drive this run? The engine writes a race-free verdict
    # sidecar next to its log at world-finalise, and OmLog deletes any stale copy
    # when it truncates the log at startup -- so its presence means "Newton drove
    # THIS run". Without this gate an ODE-only clone would silently pass the
    # default-visibility test for the wrong reason: on ODE the bridge list is
    # populated and contacts are visible regardless of the gate under test.
    sidecar = Path(str(log) + ".newton.json")
    verdict = None
    if sidecar.is_file():
        try:
            verdict = json.loads(sidecar.read_text(encoding="utf-8", errors="replace"))
        except ValueError:
            verdict = None
    if not verdict or verdict.get("backend") != "newton":
        pytest.skip(
            "no Newton verdict sidecar for run %r -- this clone ran the world on "
            "ODE (or never reached finalise), so it cannot measure the Newton "
            "contact path" % label)

    parsed = {"log": engine_log, "verdict": verdict}
    for line in result.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 2:
            try:
                parsed[parts[0]] = float(parts[1])
            except ValueError:
                pass
    return parsed


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("newton_contacts")
    return {
        "default": _run_once(tmp_path, "default", None),
        "optout": _run_once(tmp_path, "optout", "0"),
        "explicit_on": _run_once(tmp_path, "explicit_on", "1"),
    }


def _block_count(run: dict, label: str) -> int:
    v = run.get("block_contacts")
    if v is None:
        pytest.fail("run %r did not report block_contacts (probe output incomplete)" % label)
    return int(v)


def test_the_block_is_really_resting_on_the_floor(runs):
    """Geometry, not the contact API: the block is at rest at floor-top + half-height.

    Every other assertion in this file is only meaningful if there IS a contact
    to report. A 0.2 m cube dropped 0.5 mm onto a surface at z = 0 must settle
    with its centre at z = 0.1 and stop moving. If this fails, the world is
    wrong (or the block fell through the floor) and the contact counts below say
    nothing about the contact query.
    """
    for label in ("default", "optout", "explicit_on"):
        run = runs[label]
        z = run.get("block_z")
        speed = run.get("block_speed")
        assert z is not None and speed is not None, (
            "run %r did not report the block's pose" % label)
        assert abs(z - 0.1) < 0.02, (
            "run %r: the block settled at z=%.6f, not at floor-top + half-height "
            "(0.100). It is not resting on the floor, so this fixture cannot say "
            "anything about contact visibility." % (label, z))
        assert speed < 1e-2, (
            "run %r: the block is still moving (max |v| = %.6f m/s); it has not "
            "settled." % (label, speed))


def test_contacts_are_visible_by_default(runs):
    """THE POINT. No env var set, block demonstrably at rest -> contacts > 0.

    FAILS on today's build (reports 0: unset means the native path is off, and
    the ODE bridge has nothing for a Newton body whose proxy is disabled).
    PASSES after the default flip.
    """
    n = _block_count(runs["default"], "default")
    assert n >= 1, (
        "a block proven to be at rest on the floor (z=%.6f, |v|=%.2e) reported "
        "%d contact points with no environment variable set. An empty contact "
        "query is indistinguishable from 'nothing is touching', which is the "
        "exact failure that sends agents off to build geometric work-arounds."
        % (runs["default"].get("block_z", float("nan")),
           runs["default"].get("block_speed", float("nan")), n))


def test_explicit_opt_out_is_honoured(runs):
    """OMNISIM_NEWTON_NATIVE_CONTACTS=0 must mean OFF, not "is set, therefore on".

    FAILS on today's build: the gate is `qEnvironmentVariableIsSet`, so the
    string "0" ENABLES the path it names -- the documented escape hatch does the
    opposite of what it says. PASSES once the value is parsed.
    """
    n = _block_count(runs["optout"], "optout")
    assert n == 0, (
        "OMNISIM_NEWTON_NATIVE_CONTACTS=0 still reported %d contact points. The "
        "escape hatch is the only way back to the previous behaviour; if the "
        "value is ignored, setting it to 0 turns the feature ON." % n)


def test_explicit_opt_in_still_works(runs):
    """Control: with the path explicitly enabled the fixture DOES produce contacts.

    Passes on both builds. It is what makes the default-run zero above a
    statement about the gate rather than about the world.
    """
    n = _block_count(runs["explicit_on"], "explicit_on")
    assert n >= 1, (
        "OMNISIM_NEWTON_NATIVE_CONTACTS=1 reported %d contact points for a block "
        "at rest on a floor. The native readback itself is broken -- the default "
        "flip is not the thing to argue about until this passes." % n)


def test_default_run_does_not_warn_about_contact_blindness(runs):
    """The engine must not tell a caller it is blind while it is answering.

    FAILS today (the default run IS blind and says so). After the flip the
    default run must be silent, and the run that explicitly opted OUT must still
    warn -- silencing the warning instead of fixing the visibility would trade
    one silent behaviour for another.
    """
    assert _BLINDNESS_WARNING not in runs["default"]["log"], (
        "the default run logged the contact-blindness warning. Either contacts "
        "are still invisible by default, or the warning outlived the defect it "
        "describes.")

    optout = runs["optout"]
    if _block_count(optout, "optout") == 0:
        assert _BLINDNESS_WARNING in optout["log"], (
            "the opt-out run reported zero contacts and did NOT warn. A caller "
            "cannot tell that empty answer from 'nothing is touching' -- which "
            "is what the warning exists to say.")
