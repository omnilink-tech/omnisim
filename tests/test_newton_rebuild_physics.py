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

"""The mid-run physics rebuild (W1.7) must give runtime scene edits real physics.

THE GAP THIS CLOSES. `finalizeWorld()` freezes the Newton/MuJoCo model, so a
node spawned by a supervisor after finalize rendered and appeared in the scene
tree but was NEVER registered with the solver: a spawned dynamic body did not
fall (measured pre-W1.7: released at z=1.5, still 1.5 after 2200 steps -- not
one float ULP of motion), and a DELETED node's geometry stayed in the model (a
deleted floor still held bodies up). The only workaround was a 4-12 s world
reload that also killed the supervisor session.

THE VERB. `wb_supervisor_simulation_rebuild_physics()` (Python:
`Supervisor.simulationRebuildPhysics()`, commit 88487d988) requests a rebuild
consumed at the top of the next engine step: capture live velocities, tear the
Newton world down, forget all registration state, and let the ordinary per-tick
flush re-register the WHOLE scene at its CURRENT poses into a fresh world,
finalized and stepped in the same tick.

WHAT THIS FILE PINS (one engine launch, phase by phase):

  phase  scenario                                       assertion
  1      CONTROL box (authored) settles on the floor    z ~ 0.5999
  2-3    twin box SPAWNED at z=1.5, 120 steps           z = 1.5 +- 1e-6
         (the frozen-model default is ITSELF pinned
         behaviour -- spawn alone must still not move)
  4-5    simulationRebuildPhysics(), 200 steps          SPAWNED z == CONTROL z
                                                        within 1e-4; CONTROL z
                                                        unchanged (no teleport)
  6-7    FLOOR removed, rebuild again, 150 steps        BOTH bodies well below 0

RED-CAPABLE BY CONSTRUCTION. If the rebuild verb regressed to a no-op, the
spawned box would still be frozen at exactly z=1.5 after phase 4, and the
phase-5 assertion (spawned lands next to the control) fails by a full 0.9 m --
nothing lands between the two answers. If the rebuild teleported or re-seated
existing bodies, the control-body drift check fails. If delete stopped reaching
the solver, phase 7 fails with both bodies resting mid-air on phantom geometry.

REFUSAL HALF. A world simulating Cloth/SoftBody particles re-registers from its
AUTHORED rest state, so a mid-run rebuild would snap the sheet back; the engine
refuses synchronously with a WARNING ("physics rebuild REFUSED: ... Cloth/
SoftBody ..."). The second launch pins that the warning appears in the engine
log AND that the world keeps stepping normally afterwards (a refusal must not
wedge the run).

MEASURED 2026-09-01 (machine 9722d23d12a3, CPU mj_step, commit 88487d988): the
spawned box lands at 0.599892258644104 -- BIT-IDENTICAL to the authored
control's rest height -- and the in-process rebuild costs 97-267 ms (the
module-load 98% of a cold start is skipped). The cloth world answers the
refusal with the reason text intact and steps on.

    python -m pytest tests/test_newton_rebuild_physics.py -v

Cost: two engine launches (~620 rigid steps + ~110 cloth steps at fast mode),
well under 90 s wall each on a warm warp cache. Newton-only; there is no other
backend.
"""

from __future__ import annotations

import os
import subprocess

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

#: Floor: 3 x 3 x 1 box centred at the origin, so its TOP FACE is at z=0.5.
FLOOR_TOP_Z = 0.5
#: Both boxes are 0.2 cubes with mass 1, so they rest at 0.5 + 0.1 minus the
#: solver's ~0.1 mm contact softness. Measured 0.599892258644104 for BOTH the
#: authored control and the rebuilt spawn (bit-identical).
REST_Z = FLOOR_TOP_Z + 0.1
#: The spawn height. Phase 3 pins that WITHOUT a rebuild the spawned body does
#: not move from it AT ALL -- the frozen-model default is documented behaviour.
SPAWN_Z = 1.5
#: Frozen means frozen: measured pre-rebuild drift is zero to the last ULP.
FROZEN_TOL = 1e-6
#: Landed / no-teleport tolerance. The two rest heights measured bit-identical;
#: 1e-4 leaves room for solver jitter while staying 4 orders of magnitude away
#: from the 0.9 m failure signature.
MATCH_TOL = 1e-4
#: "Fell through": 150 steps of free fall from ~0.6 reaches z ~ -6.5. Anything
#: below -1.0 is unreachable while ANY collision surface still exists.
FALL_BELOW = -1.0

#: The refusal warning the engine emits for a particle world
#: (OmSupervisorUtilities.cpp: 'physics rebuild REFUSED: %1', reason from
#: OmSimulationWorld::requestNewtonRebuild).
REFUSED_MARKER = "physics rebuild REFUSED"
REFUSED_CAUSE = "Cloth/SoftBody"

#: Known intermittent embedded-interpreter bring-up failure. A run that never
#: got a Newton runtime says nothing about the rebuild verb.
_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
    "embedded Python init failed",
)

REBUILD_WORLD = """#OMNISIM R2025a utf8
WorldInfo {
  basicTimeStep 8
  coordinateSystem "ENU"
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
  newtonStatics TRUE
}
Viewpoint { orientation 0 0 1 0 position -4 0 2 }
DEF FLOOR Solid {
  translation 0 0 0
  name "floor"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.5 0.5 0.55 roughness 1 metalness 0 }
      geometry Box { size 3 3 1 }
    }
  ]
  boundingObject Box { size 3 3 1 }
}
DEF CONTROL Solid {
  translation 0.5 0 1.5
  name "control"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.65 0.85 roughness 0.9 metalness 0 }
      geometry Box { size 0.2 0.2 0.2 }
    }
  ]
  boundingObject Box { size 0.2 0.2 0.2 }
  physics Physics { density -1 mass 1 }
}
DEF PROBE Robot { name "probe" controller "rebuildprobe" supervisor TRUE }
"""

#: The spawned twin: same shape/physics as CONTROL, mirrored to x=-0.5. Written
#: as one line because importMFNodeFromString gets it verbatim.
_SPAWN_NODE = (
    'DEF SPAWNED Solid { translation -0.5 0 1.5 name "spawned" children [ '
    'Shape { appearance PBRAppearance { baseColor 0.9 0.3 0.2 roughness 0.9 '
    'metalness 0 } geometry Box { size 0.2 0.2 0.2 } } ] '
    'boundingObject Box { size 0.2 0.2 0.2 } '
    'physics Physics { density -1 mass 1 } }')

REBUILD_CONTROLLER = '''\
import os
from omnisim import Supervisor

out = open(os.environ['REBUILD_PROBE_OUT'], 'w', buffering=1)
sup = Supervisor()
dt = int(sup.getBasicTimeStep())


def advance(n):
    for _ in range(n):
        if sup.step(dt) == -1:
            return False
    return True


if not hasattr(sup, 'simulationRebuildPhysics'):
    # A stale libController (older than the engine) simply lacks the verb; that
    # is an install problem, not a regression in the rebuild. `python -m
    # omnisim doctor` gates the engine<->libController ABI.
    out.write('SKIP stale_libcontroller no simulationRebuildPhysics on Supervisor\\n')
else:
    control = sup.getFromDef('CONTROL')
    floor = sup.getFromDef('FLOOR')
    if control is None or floor is None:
        out.write('missing CONTROL=%s FLOOR=%s\\n' % (control, floor))
    else:
        # Phase 1: let the authored box settle. 150 steps = 1.2 s; the 0.9 m
        # drop takes < 0.5 s.
        advance(150)
        control_z0 = control.getPosition()[2]
        out.write('control_z0 %.12f\\n' % control_z0)

        # Phase 2: spawn the twin. One step so the import lands before lookup.
        sup.getRoot().getField('children').importMFNodeFromString(
            -1, @SPAWN_NODE@)
        advance(1)
        spawned = sup.getFromDef('SPAWNED')
        if spawned is None:
            out.write('spawn_failed 1\\n')
        else:
            # Phase 3: the frozen-model default. ~1 s of stepping, the spawned
            # body must not move AT ALL (the solver has never seen it).
            advance(119)
            frozen_z = spawned.getPosition()[2]
            out.write('spawned_frozen_z %.12f\\n' % frozen_z)

            # Phase 4: the verb under test. Consumed at the top of the next
            # step; 200 steps is settle time for the 0.9 m drop plus margin.
            sup.simulationRebuildPhysics()
            advance(200)

            # Phase 5 readings.
            spawned_post = spawned.getPosition()[2]
            control_post = control.getPosition()[2]
            out.write('spawned_post_z %.12f\\n' % spawned_post)
            out.write('control_post_z %.12f\\n' % control_post)

            # Phase 6: delete the floor, rebuild again, 150 steps = 1.2 s of
            # free fall from ~0.6 reaches ~ -6.5.
            floor.remove()
            sup.simulationRebuildPhysics()
            advance(150)

            # Phase 7 readings.
            spawned_fall = spawned.getPosition()[2]
            control_fall = control.getPosition()[2]
            out.write('spawned_fall_z %.12f\\n' % spawned_fall)
            out.write('control_fall_z %.12f\\n' % control_fall)

            # One machine-parsable verdict line (the pytest side re-derives
            # each flag from the raw readings above; this line is for humans
            # and log greps).
            out.write('REBUILD_TEST frozen_ok=%d spawned_landed=%d '
                      'control_stable=%d fell_through=%d\\n' % (
                          abs(frozen_z - 1.5) < 1e-6,
                          abs(spawned_post - control_post) < 1e-4,
                          abs(control_post - control_z0) < 1e-4,
                          spawned_fall < -1.0 and control_fall < -1.0))
out.write('done\\n')
out.close()
sup.simulationQuit(0)
'''.replace("@SPAWN_NODE@", repr(_SPAWN_NODE))

#: Minimal Cloth world -- declaration style copied from
#: projects/samples/demos/worlds/rendering/camera_cloth_wgpu_smoke.omniworld
#: (the deliberately-small 289-particle P1 gate world), shrunk to 9x9=81
#: particles. mujoco+vbd is the same solver pairing every shipped cloth world
#: uses; the sheet is pinned along its top edge so it just hangs.
CLOTH_WORLD = """#OMNISIM R2025a utf8
WorldInfo {
  gravity 9.81
  basicTimeStep 8
  coordinateSystem "ENU"
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco+vbd"
  newtonStatics TRUE
}
Viewpoint { orientation 0 0 1 0 position -3 0 1.5 }
DEF FLOOR Solid {
  translation 0 0 0
  name "floor"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.5 0.5 0.55 roughness 1 metalness 0 }
      geometry Box { size 4 4 0.1 }
    }
  ]
  boundingObject Box { size 4 4 0.1 }
}
DEF SHEET Cloth {
  translation -0.2 -0.2 0.8
  dimX 8
  dimY 8
  cellX 0.05
  cellY 0.05
  mass 0.001
  particleRadius 0.01
  fixTop TRUE
  appearance PBRAppearance { baseColor 0.8 0.18 0.15 roughness 0.9 metalness 0 }
}
DEF PROBE Robot { name "probe" controller "clothprobe" supervisor TRUE }
"""

CLOTH_CONTROLLER = '''\
import os
from omnisim import Supervisor

out = open(os.environ['REBUILD_PROBE_OUT'], 'w', buffering=1)
sup = Supervisor()
dt = int(sup.getBasicTimeStep())

if not hasattr(sup, 'simulationRebuildPhysics'):
    out.write('SKIP stale_libcontroller no simulationRebuildPhysics on Supervisor\\n')
else:
    # Let the cloth world finalize and genuinely run before asking. A world
    # whose Newton world is not running yet is refused for a DIFFERENT reason
    # ("still building"), which would pass a bare REFUSED grep while proving
    # nothing about the particle guard.
    for _ in range(50):
        if sup.step(dt) == -1:
            break
    t0 = sup.getTime()
    out.write('t0 %.6f\\n' % t0)

    # Fire-and-forget on the wire; the refusal arrives as an engine WARNING.
    sup.simulationRebuildPhysics()

    # The refusal must not wedge the run: the world keeps stepping.
    stepped = 0
    for _ in range(60):
        if sup.step(dt) == -1:
            break
        stepped += 1
    out.write('t1 %.6f\\n' % sup.getTime())
    out.write('steps_after %d\\n' % stepped)
out.write('done\\n')
out.close()
sup.simulationQuit(0)
'''


def _binary():
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def _run(tmp_path, tag, world_text, ctrl_name, ctrl_text, timeout):
    """Run one probe world headless and return its parsed key/value output."""
    worlds = tmp_path / tag / "worlds"
    ctrl = tmp_path / tag / "controllers" / ctrl_name
    worlds.mkdir(parents=True, exist_ok=True)
    ctrl.mkdir(parents=True, exist_ok=True)
    world = worlds / ("%s.omniworld" % tag)
    world.write_text(world_text, encoding="utf-8")
    (ctrl / ("%s.py" % ctrl_name)).write_text(ctrl_text, encoding="utf-8")

    result = tmp_path / tag / "probe_out.txt"
    log = tmp_path / tag / "engine.log"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), REBUILD_PROBE_OUT=str(result),
               OMNISIM_LOG_PATH=str(log))
    # A stale export in the developer's shell must not decide the answer.
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in (
                "OMNISIM_FORCE_ODE", "OMNISIM_LEGACY", "OMNISIM_ALLOW_ODE_FALLBACK"):
            env.pop(k)
    # This is a run whose result we intend to trust: a silently-absent runtime
    # must FATAL rather than produce a no-physics run we would misread.
    env["OMNISIM_REQUIRE_NEWTON"] = "1"

    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(world)],
                       env=env, timeout=timeout, capture_output=True)
    except subprocess.TimeoutExpired:
        pass

    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not result.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up for %r (%r)" % (tag, sig))
        pytest.fail("the %r probe produced no output:\n%s" % (tag, text[-1500:]))

    raw = result.read_text(encoding="utf-8", errors="replace")
    for line in raw.splitlines():
        if line.startswith("SKIP stale_libcontroller"):
            pytest.skip(
                "libController predates the rebuild verb (Supervisor has no "
                "simulationRebuildPhysics) -- a stale-lib install problem, not "
                "a rebuild regression. Rebuild the controller libs and check "
                "`python -m omnisim doctor`.")

    # Prove NEWTON drove this run before reading anything as a Newton result.
    # OmLog deletes any stale sidecar when it truncates the log at startup, so
    # the file's mere presence means "Newton finalised THIS run".
    sidecar = Path(str(log) + ".newton.json")
    if not sidecar.is_file():
        pytest.skip(
            "no %s -- Newton did not finalise this run, so it says nothing "
            "about the rebuild verb. (A missing sidecar means the run never "
            "reached world-finalize, NOT that another engine drove it.)"
            % sidecar.name)

    values = {}
    for line in raw.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] != "done":
            try:
                values[parts[0]] = float(parts[1])
            except ValueError:
                values[parts[0]] = parts[1]
    if "done" not in raw:
        pytest.fail("the %r probe died before finishing its phases; it wrote:\n%s"
                    "\nengine log tail:\n%s" % (tag, raw, text[-1500:]))
    if "spawn_failed" in values:
        pytest.fail("importMFNodeFromString produced no SPAWNED node -- the "
                    "probe cannot ask its question. Engine log tail:\n%s"
                    % text[-1500:])
    if "missing" in raw.split():
        pytest.fail("probe could not resolve its DEF nodes:\n%s" % raw)
    values["_log"] = text
    values["_raw"] = raw
    return values


@pytest.fixture(scope="module")
def rebuild(tmp_path_factory):
    """One engine launch covering phases 1-7; every test below reads from it."""
    return _run(tmp_path_factory.mktemp("rebuildphysics"), "rebuild",
                REBUILD_WORLD, "rebuildprobe", REBUILD_CONTROLLER, timeout=240)


@pytest.fixture(scope="module")
def cloth(tmp_path_factory):
    """The refusal launch. Separate fixture so a cloth-lane flake (cold warp
    JIT, VBD bring-up) cannot take the rigid-body verdicts down with it."""
    return _run(tmp_path_factory.mktemp("rebuildrefusal"), "cloth",
                CLOTH_WORLD, "clothprobe", CLOTH_CONTROLLER, timeout=300)


def _need(values, *keys):
    missing = [k for k in keys if k not in values]
    if missing:
        pytest.fail("probe output is missing %s -- it wrote:\n%s"
                    % (missing, values["_raw"]))


def test_spawned_node_is_frozen_before_rebuild(rebuild):
    """Phases 1+3: the pre-rebuild world behaves exactly as documented.

    The control settling on the floor proves the scene is a valid instrument;
    the spawned twin NOT moving pins the frozen-model default itself. If this
    goes red because the spawn started falling on its own, runtime spawns have
    begun reaching the solver without a rebuild -- a behaviour change that
    would make the rebuild verb redundant and this whole file mis-scoped.
    """
    _need(rebuild, "control_z0", "spawned_frozen_z")
    assert abs(rebuild["control_z0"] - REST_Z) < 0.02, (
        "the AUTHORED control box settled at z=%.6f, expected ~%.4f (floor top "
        "%.2f + half box). The baseline scene is broken, so nothing downstream "
        "means anything." % (rebuild["control_z0"], REST_Z, FLOOR_TOP_Z))
    assert abs(rebuild["spawned_frozen_z"] - SPAWN_Z) < FROZEN_TOL, (
        "the spawned box moved to z=%.9f during the 120 pre-rebuild steps "
        "(spawned at %.1f, tolerance %g). Pre-W1.7 measurement: it does not "
        "move one float ULP, because the frozen solver has never seen it."
        % (rebuild["spawned_frozen_z"], SPAWN_Z, FROZEN_TOL))


def test_rebuild_gives_spawned_node_physics(rebuild):
    """Phase 5, the verb itself: after the rebuild the spawn falls and lands.

    THE red line for a rebuild regression. If simulationRebuildPhysics()
    regressed to a no-op the spawn is still frozen at exactly z=1.5 and this
    fails by 0.9 m -- nothing lands between the two answers. Measured
    2026-09-01: both boxes at 0.599892258644104, bit-identical.
    """
    _need(rebuild, "spawned_post_z", "control_post_z")
    assert abs(rebuild["spawned_post_z"] - rebuild["control_post_z"]) < MATCH_TOL, (
        "after simulationRebuildPhysics() the spawned box reads z=%.9f while "
        "the authored control reads z=%.9f (must match within %g).\n"
        "  spawned still at ~%.1f means the rebuild was a NO-OP: the request "
        "was consumed (or dropped) without re-registering the scene, and the "
        "spawn never entered the solver."
        % (rebuild["spawned_post_z"], rebuild["control_post_z"], MATCH_TOL,
           SPAWN_Z))


def test_rebuild_does_not_teleport_existing_bodies(rebuild):
    """Phase 5, the other half: a rebuild must not disturb settled bodies.

    Re-registration reads each body's CURRENT pose (matrix(), kept live by the
    per-tick readback). If it read authored poses instead, the control would
    snap back toward its spawn point and drift on re-settle.
    """
    _need(rebuild, "control_z0", "control_post_z")
    assert abs(rebuild["control_post_z"] - rebuild["control_z0"]) < MATCH_TOL, (
        "the AUTHORED control box moved from z=%.9f to z=%.9f across the "
        "rebuild (allowed %g). The rebuild is re-registering from the wrong "
        "pose source -- authored state instead of current state -- which "
        "teleports every settled body in the scene."
        % (rebuild["control_z0"], rebuild["control_post_z"], MATCH_TOL))


def test_rebuild_after_delete_removes_geometry(rebuild):
    """Phase 7: a deleted floor must genuinely stop holding bodies up.

    The delete-side mirror of the spawn defect: pre-W1.7 a deleted Solid's
    geometry stayed in the frozen model (a box rested on a deleted floor at
    z=0.5999 for 61,440 steps). After remove()+rebuild both bodies free-fall;
    150 steps from ~0.6 reaches ~ -6.5, and anything below FALL_BELOW (-1.0)
    is unreachable while any collision surface still exists (the implicit
    ground plane only ever substitutes for an authored Plane collider, and
    this floor is a Box).
    """
    _need(rebuild, "spawned_fall_z", "control_fall_z")
    assert rebuild["spawned_fall_z"] < FALL_BELOW and \
        rebuild["control_fall_z"] < FALL_BELOW, (
        "after deleting FLOOR and rebuilding, spawned z=%.4f / control z=%.4f "
        "(both must be < %.1f). A body still at ~0.6 is resting on the DELETED "
        "floor's phantom geometry -- the rebuild did not drop it from the "
        "model." % (rebuild["spawned_fall_z"], rebuild["control_fall_z"],
                    FALL_BELOW))


def test_rebuild_refused_on_cloth_world(cloth):
    """A particle world must refuse the rebuild LOUDLY and keep running.

    Cloth/SoftBody re-register from their AUTHORED rest state, so a rebuild
    would snap the sheet back; v1 refuses at request time. Three assertions:
    the WARNING appears, it names the particle cause (a bare refusal that does
    not say WHY would pass a grep while telling the author nothing), and the
    world still steps normally afterwards.
    """
    _need(cloth, "t0", "t1", "steps_after")
    refusals = [ln for ln in cloth["_log"].splitlines() if REFUSED_MARKER in ln]
    assert refusals, (
        "no %r line in the engine log -- the cloth world accepted a rebuild "
        "it must refuse (particles would snap back to authored state), or the "
        "refusal went silent. Log tail:\n%s"
        % (REFUSED_MARKER, cloth["_log"][-1500:]))
    assert any(REFUSED_CAUSE in ln for ln in refusals), (
        "the refusal does not name the cause. Expected %r in one of:\n  %s"
        % (REFUSED_CAUSE, "\n  ".join(ln[:200] for ln in refusals)))
    assert cloth["steps_after"] == 60 and cloth["t1"] - cloth["t0"] > 0.3, (
        "the world stopped stepping after the refused rebuild: %d/60 steps, "
        "sim time %.3f -> %.3f s. A refusal must be a no-op for the running "
        "world, not a wedge."
        % (int(cloth["steps_after"]), cloth["t0"], cloth["t1"]))
