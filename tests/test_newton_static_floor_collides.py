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

"""A static floor must be SOLID under Newton without the world asking for it.

The defect this pins is the quietest one in the engine. A top-level static
collider (a Solid with a `boundingObject` and no `Physics`) was registered with
the Newton backend only when the world set `WorldInfo.newtonStatics TRUE` or the
launch set `OMNISIM_NEWTON_STATICS`. Everywhere else the collider stayed "on the
ODE side" -- which in a Newton world means it collides with NOTHING, because no
Newton body is ever tested against it.

That failure is invisible by construction. Newton opens every world with an
implicit ground plane at z=0, so the scene still runs, the bodies still land on
something, and the process still exits 0. The ONLY symptom is that things rest
at the wrong height. Measured 2026-08-05: a ball dropped from z=0.9 onto a box
floor whose top is at z=0.55 passed straight through it and settled at
z=0.0996 -- on a plane that is not in the world file, cannot be found by DEF and
does not appear in the scene tree.

Worlds whose floor happens to sit at z=0 are masked by that implicit plane and
look fine, which is why the defect survived in so many of them. This test uses a
RAISED floor precisely so the mask cannot hide the answer: 0.65 means the floor
in the file caught the ball, 0.10 means the phantom plane did.

WHAT EACH CASE REPORTS TODAY (pre-patch build) vs AFTER the default flip:

  case              world/env                       today    after
  default           no field, no env var            ~0.0996  ~0.6496   FAILS today
  escape hatch      OMNISIM_NEWTON_STATICS=0        ~0.6496  ~0.0996   FAILS today
  env force-on      OMNISIM_NEWTON_STATICS=1        ~0.6496  ~0.6496   passes today
  world field       WorldInfo.newtonStatics TRUE    ~0.6496  ~0.6496   passes today
  field beats hatch field TRUE + env 0              ~0.6496  ~0.6496   passes today

The last three are the regression half: the flip must not break the 47 worlds
and the ~30 launch scripts in this tree that already ask for statics explicitly.

The "escape hatch" case fails today for a second, separate reason worth knowing:
the gate tested only whether the variable was NON-EMPTY, so `=0` turned statics
ON. Nothing in the tree passes `=0` (every caller passes `=1`), so reading the
value costs no existing invocation its meaning -- but until it is read there is
no way to turn statics off for one run.

The last case is what makes the hatch an EXACT revert rather than a third state:
a world that declares the field keeps statics even under `=0`, because that
world had them before the flip too. So `OMNISIM_NEWTON_STATICS=0` reproduces the
previous build across the whole tree, which is what makes a regression from this
flip bisectable with one variable instead of a rebuild.

    python -m pytest tests/test_newton_static_floor_collides.py -v

Cost: five engine launches, ~2 minutes. Newton-only -- it says nothing about
ODE, where a static floor has always been solid.
"""

from __future__ import annotations

import json
import os
import subprocess

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

#: Floor box: 4 x 4 x 0.3 centred at z=0.40, so its TOP FACE is at z=0.55.
FLOOR_TOP_Z = 0.55
#: Ball radius. Dropped from z=0.90, i.e. 0.25 m above the floor's top face.
BALL_RADIUS = 0.10

#: Resting height when the floor in the FILE is what catches the ball.
#: Measured 0.6496 -- the 0.4 mm short of 0.65 is the solver's contact softness.
REST_ON_FLOOR = FLOOR_TOP_Z + BALL_RADIUS
#: Resting height when the ball falls THROUGH the floor onto Newton's implicit
#: ground plane at z=0. Measured 0.0996.
REST_ON_PHANTOM_PLANE = BALL_RADIUS
#: Generous next to a 0.55 m separation; tight next to the 0.4 mm sink measured
#: on both surfaces. Nothing lands between the two answers.
TOL = 0.02

#: Known intermittent embedded-interpreter bring-up failure. A world that asked
#: for Newton is refused rather than quietly run on ODE, so such a run produces
#: no data at all -- an instrument outcome, not a physics result.
_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)


def _world(statics_field):
    """The probe world. `statics_field` is the WorldInfo line, or ''.

    Deliberately spare: one static floor, one dynamic ball, one supervisor. No
    arena, no PROTO, no lighting recipe -- every node here is load-bearing for
    the answer, so a failure cannot be blamed on scenery.
    """
    return """#VRML_SIM R2025a utf8
WorldInfo {
  basicTimeStep 8
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
%s}
Viewpoint { orientation 0 0 1 0 position -4 0 2 }
DEF FLOOR Solid {
  translation 0 0 0.4
  name "floor"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.5 0.5 0.55 roughness 1 metalness 0 }
      geometry Box { size 4 4 0.3 }
    }
  ]
  boundingObject Box { size 4 4 0.3 }
}
DEF BALL Solid {
  translation 0 0 0.9
  name "ball"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.65 0.85 roughness 0.9 metalness 0 }
      geometry Sphere { radius 0.1 subdivision 3 }
    }
  ]
  boundingObject Sphere { radius 0.1 }
  physics Physics { density -1 mass 1 }
}
DEF PROBE Robot { name "probe" controller "floorprobe" supervisor TRUE }
""" % statics_field


CONTROLLER = "\n".join([
    "import os",
    "from omnisim import Supervisor",
    "",
    "out = open(os.environ['PROBE_OUT'], 'w', buffering=1)",
    "sup = Supervisor()",
    "dt = int(sup.getBasicTimeStep())",
    "ball = sup.getFromDef('BALL')",
    "floor = sup.getFromDef('FLOOR')",
    "if ball is None or floor is None:",
    "    out.write('missing BALL=%s FLOOR=%s\\n' % (ball, floor))",
    "else:",
    "    for _ in range(400):",          # 3.2 s at dt=8 ms; the drop takes < 0.5 s
    "        if sup.step(dt) == -1:",
    "            break",
    "    out.write('ball_z %.6f\\n' % ball.getPosition()[2])",
    "    out.write('floor_z %.6f\\n' % floor.getPosition()[2])",
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


def _run(tmp_path, tag, statics_field="", env_extra=None):
    """Run the probe world once and return its settled ball height."""
    worlds = tmp_path / tag / "worlds"
    ctrl = tmp_path / tag / "controllers" / "floorprobe"
    worlds.mkdir(parents=True, exist_ok=True)
    ctrl.mkdir(parents=True, exist_ok=True)
    world = worlds / "floor.wbt"
    world.write_text(_world(statics_field), encoding="utf-8")
    (ctrl / "floorprobe.py").write_text(CONTROLLER, encoding="utf-8")

    result = tmp_path / tag / "probe_out.txt"
    log = tmp_path / tag / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(result),
               OMNISIM_LOG_PATH=str(log))
    # A stale export in the developer's shell would decide the answer for us.
    env.pop("OMNISIM_NEWTON_STATICS", None)
    env.pop("OMNISIM_FORCE_ODE", None)
    env.pop("OMNISIM_LEGACY", None)
    env.update(env_extra or {})

    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(world)],
                       env=env, timeout=180, capture_output=True)
    except subprocess.TimeoutExpired:
        pass

    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not result.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up for %r (%r)" % (tag, sig))
        pytest.fail("the %r probe produced no output:\n%s" % (tag, text[-1500:]))

    # Prove NEWTON drove this run before reading anything as a Newton result.
    # On an ODE-only clone the same world settles at 0.65 for a reason that has
    # nothing to do with this patch, and scoring that as a pass would be a lie.
    # The sidecar's mere presence means "Newton finalised THIS run" -- OmLog
    # deletes any stale copy when it truncates the log at startup.
    sidecar = Path(str(log) + ".newton.json")
    if not sidecar.is_file():
        pytest.skip(
            "no %s -- Newton did not finalise this run, so its height says "
            "nothing about the statics gate. (A missing sidecar means the run "
            "never reached world-finalize, NOT that ODE drove it.)"
            % sidecar.name)
    verdict = json.loads(sidecar.read_text(encoding="utf-8"))

    heights = {}
    for line in result.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] in ("ball_z", "floor_z"):
            heights[parts[0]] = float(parts[1])
    if "ball_z" not in heights:
        pytest.fail("the %r probe wrote no ball height:\n%s"
                    % (tag, result.read_text(encoding="utf-8", errors="replace")))
    heights["_verdict"] = verdict
    return heights


def _explain(tag, heights, expected):
    landed = ("the floor in the file" if abs(heights["ball_z"] - REST_ON_FLOOR) < TOL
              else "Newton's implicit ground plane at z=0"
              if abs(heights["ball_z"] - REST_ON_PHANTOM_PLANE) < TOL
              else "neither surface")
    return (
        "case %r: the ball settled at z=%.4f, expected %.4f (+/-%.2f).\n"
        "  It is resting on %s.\n"
        "  floor top face z=%.2f, ball radius %.2f, phantom-plane rest z=%.2f\n"
        "  static floor read back at z=%.4f (a static must not move)\n"
        "  backend verdict: %s"
        % (tag, heights["ball_z"], expected, TOL, landed, FLOOR_TOP_Z,
           BALL_RADIUS, REST_ON_PHANTOM_PLANE, heights.get("floor_z", float("nan")),
           heights["_verdict"]))


@pytest.fixture(scope="module")
def probe(tmp_path_factory):
    return tmp_path_factory.mktemp("staticfloor")


def test_static_floor_is_solid_by_default(probe):
    """No world field, no env var -- the floor must still catch the ball.

    This is the whole point. An author who writes a floor into a world file has
    said everything they should have to say about wanting it to be solid.
    """
    heights = _run(probe, "default")
    assert abs(heights["ball_z"] - REST_ON_FLOOR) < TOL, _explain(
        "default", heights, REST_ON_FLOOR) + (
        "\n\nA ball resting at ~%.2f means the floor Solid was never registered "
        "with Newton and the ball fell through it onto a plane that is not in "
        "the world. Nothing in the log calls that an error and the run exits 0."
        % REST_ON_PHANTOM_PLANE)


def test_escape_hatch_turns_statics_back_off(probe):
    """OMNISIM_NEWTON_STATICS=0 must restore the old ODE-side behaviour.

    A default flip needs a way back for one run without editing a world -- for
    bisecting a physics change against the previous default, or for a world
    that was tuned around the phantom plane. The variable's VALUE has to be
    read for that: testing only that it is non-empty makes `=0` mean ON.
    """
    heights = _run(probe, "off", env_extra={"OMNISIM_NEWTON_STATICS": "0"})
    assert abs(heights["ball_z"] - REST_ON_PHANTOM_PLANE) < TOL, _explain(
        "off", heights, REST_ON_PHANTOM_PLANE) + (
        "\n\nOMNISIM_NEWTON_STATICS=0 did not turn static registration off. If "
        "the ball is at ~%.2f the gate is still testing the variable for "
        "NON-EMPTINESS rather than reading its value, so there is no way to "
        "opt out for a single run." % REST_ON_FLOOR)


def test_env_force_on_still_works(probe):
    """The documented OMNISIM_NEWTON_STATICS=1 knob must keep forcing statics on.

    ~30 launch scripts, the G1 physics spec and several campaign runners export
    it. The flip makes it redundant; it must not make it wrong.
    """
    heights = _run(probe, "on", env_extra={"OMNISIM_NEWTON_STATICS": "1"})
    assert abs(heights["ball_z"] - REST_ON_FLOOR) < TOL, _explain(
        "on", heights, REST_ON_FLOOR)


def test_world_field_still_works(probe):
    """47 worlds in this tree declare `newtonStatics TRUE`. They must not move.

    The field is subsumed by the new default rather than removed, so these
    worlds keep the behaviour they were tuned against.
    """
    heights = _run(probe, "field", statics_field="  newtonStatics TRUE\n")
    assert abs(heights["ball_z"] - REST_ON_FLOOR) < TOL, _explain(
        "field", heights, REST_ON_FLOOR)


def test_world_field_outranks_the_escape_hatch(probe):
    """`newtonStatics TRUE` keeps its statics even under the off-switch.

    This is what makes the hatch an exact revert instead of a third state. A
    world that declared the field had statics BEFORE the flip, so reverting the
    flip must leave it with statics. Without this rule a stale
    `OMNISIM_NEWTON_STATICS=0` in a shell would silently empty the bins in
    omniarm6_anypick and drop the arena walls in the robot_combat worlds -- worlds
    that never depended on the default in the first place.
    """
    heights = _run(probe, "field_vs_hatch", statics_field="  newtonStatics TRUE\n",
                   env_extra={"OMNISIM_NEWTON_STATICS": "0"})
    assert abs(heights["ball_z"] - REST_ON_FLOOR) < TOL, _explain(
        "field_vs_hatch", heights, REST_ON_FLOOR) + (
        "\n\nThe env off-switch overrode an explicit WorldInfo.newtonStatics "
        "TRUE. It must not: that world asked for statics directly and had them "
        "before the default flip, so the revert switch has nothing to revert "
        "for it.")
