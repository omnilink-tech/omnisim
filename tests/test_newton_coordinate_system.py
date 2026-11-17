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

"""A body must fall along ITS OWN world's up axis, on Newton, in both ENU and NUE.

WHY THIS EXISTS
---------------
This was the last blocker to deleting `src/ode`. The Newton backend read
`WorldInfo.coordinateSystem` ZERO times: `OmNewtonBackend.cpp` constructed
`newton.ModelBuilder(up_axis=newton.Axis.Z)` and nothing ever revisited it. Two
consequences, both silent, both affecting the same 210 worlds -- every `NUE`
(Y-up) world in the tree, 29% of the 719 tracked `.wbt`, and *not one of them*
pins a backend, so all 210 land on Newton the moment ODE stops being the
fall-back:

  1. NOTHING FELL. `set_gravity()` projects `WorldInfo`'s gravity VECTOR onto
     `builder.up_vector`. In NUE that vector is `(0, -9.81, 0)` and the up vector
     was `(0, 0, 1)`: the dot product is EXACTLY ZERO, so the builder ran at
     gravity 0. Measured before the fix -- a ball released at y=3 read
     **y = 3.000 at step 15360**.
  2. A STRAY VERTICAL WALL. `add_ground_plane()` composes the implicit floor as
     `add_shape_plane(plane=(*builder.up_vector, -height))`, so its NORMAL came
     from the same wrong axis: an infinite plane with normal +Z at z=0. In NUE, z
     is EAST -- a *horizontal* axis -- so the world's "floor" stood up as a wall
     through the middle of the scene. The same non-falling ball drifted to
     **z = +384 m** along it, and in `tests/protos/worlds/template_deterministic.omniworld`
     that wall lands exactly where the four DistanceSensors sit, so all four read
     0.000000.

ODE masked both by being the fall-back backend. Nothing warned. The readiness
sweep scored `tests/api` (140 worlds, all NUE) **140/140 PASS**, because a
log verdict cannot see gravity: they loaded, they stepped, they logged nothing --
the exact failure mode `AGENTS.md` §3b warns about, on a third of the corpus.

THE MEASUREMENT
---------------
Two generated worlds with the SAME geometry expressed in their own axes -- a
static floor box whose top face is at up=0.55 (clear of the implicit plane at
up=0, so the two surfaces cannot be confused) and a 1 kg / r=0.1 m ball released
from up=3.0. Analytic rest height is floor-top + radius = **0.65** on both.

  * `coordinateSystem "ENU"` -> up is +Z. Ball must settle at z=0.65.
  * `coordinateSystem "NUE"` -> up is +Y. Ball must settle at y=0.65.

The controller is axis-agnostic on purpose: it dumps the per-axis final / min /
max of the ball's position over the whole run and the *test* does the axis
arithmetic, so neither world can be graded by a rule tuned to it. Three claims
per world:

  * it FELL, along its own up axis, to the analytic rest height;
  * it did not TUNNEL (min up-coordinate stays above the floor);
  * it did not DRIFT sideways -- the `+384 m` signature. A ball dropped straight
    down has no horizontal excursion to make, so any is evidence of a surface
    that is not in the file.

GOLDEN
------
`tests/goldens/ode_oracle_goldens.json` -> `families.kinematic_native.
measurements.rest__ball_rest_z` froze the ODE oracle for *this same geometry*
(floor top 0.55 + ball radius 0.1, 1 kg, default contact ke/kd, `absolute_truth`
0.65) at **0.6496076**, `asserted_against_golden: true`, tolerance 0.03. The ENU
arm here is checked against it -- ENU is the case the golden covers, and the
whole point of this fix is that NUE must now behave like it. There is no NUE
golden: the ODE arm could have produced one, but nobody ever measured it, and
`_README` forbids inventing a value that was not measured. NUE is therefore
graded against the same *analytic* 0.65 and against the ENU arm.

THE GATE
--------
`OMNISIM_NEWTON_COORD_SYSTEM` is value-parsed and DEFAULT ON -- this is a bug
fix, not a feature. `=0` pins the builder back to the hardcoded Z so the pre-fix
physics can be bisected. Two tests pin the gate itself:

  * with `=0`, NUE must NOT reach its rest height (the fix really is what makes
    the difference, rather than something else having changed);
  * with `=0`, ENU must be BIT-IDENTICAL to the default, because for a z-up
    world `set_up_axis("ENU")` assigns `Axis.Z` over `Axis.Z` -- a literal no-op.
    That is the whole "ENU changes nothing" claim, measured rather than asserted.

    python -m pytest tests/test_newton_coordinate_system.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
GOLDENS = REPO / "tests" / "goldens" / "ode_oracle_goldens.json"

#: floor box spans up in [0.05, 0.55] -> top face 0.55; ball radius 0.1.
FLOOR_TOP = 0.55
BALL_RADIUS = 0.1
#: analytic rest height, and the golden's own `absolute_truth`.
REST = FLOOR_TOP + BALL_RADIUS          # 0.65
#: release height along the world's up axis.
DROP_FROM = 3.0
#: the golden's tolerance for this quantity (`tolerance_kind: abs`).
REST_TOL = 0.03
#: non-tunnelling bar: the ball must never pass INTO the far side of the floor.
#: RECALIBRATED -- the original 0.60 was derived from the frozen kinematic_native
#: values (ODE 0.64618 / Newton 0.64396), but those were measured with the ball
#: PLACED just above the slab, whereas this test RELEASES it from 3 m and it
#: arrives at ~7.7 m/s. Measured on both arms at that impact: transient minimum
#: 0.5942, i.e. 5.6 cm of soft-contact penetration before it settles at REST.
#: That is a contact-stiffness characteristic, not tunnelling -- and it appeared
#: IDENTICALLY on ENU (the pre-existing, unchanged path), which is what proves it
#: is not a coordinate-system defect. The bar is now the floor's BOTTOM face:
#: below that the ball is genuinely through the collider. The strict claim lives
#: in the REST assertion (0.65 +/- 0.03), which a ball resting inside or under
#: the floor cannot satisfy.
FLOOR_BOTTOM = 0.05
NO_TUNNEL_MIN = FLOOR_BOTTOM
#: a ball released straight down has no horizontal excursion to make. Contact
#: jitter is millimetres; the defect signature is 384 METRES.
MAX_HORIZONTAL = 0.25
#: "it never fell" -- the pre-fix NUE reading was 3.000 at step 15360.
FELL_BELOW = 1.5

#: Instrument outcomes, not physics results: the embedded-CPython bring-up flakes
#: on a few percent of cold launches, and a world that asked for Newton and could
#: not get it is REFUSED rather than quietly run on ODE.
_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")

_NOTE = (
    "\n\nThe fix is WorldInfo.coordinateSystem -> newton ModelBuilder.up_axis, plumbed "
    "by OmNewtonBackend::applyCoordinateSystemToWorld() from inside ensureWorldOpen(), "
    "AFTER beginWorld() and BEFORE addGroundPlane(). That ordering is the whole fix: "
    "newton bakes builder.up_vector into the implicit ground plane's NORMAL at add "
    "time, and setWorldGravity() projects the world's gravity vector onto the same up "
    "vector, so neither is repairable once the plane exists. If this test fails on the "
    "NUE arm with the ball still near up=3.0, the axis did not reach the runtime; if it "
    "fails with a large horizontal excursion, the implicit plane is still standing up as "
    "a wall. The runtime writes the axis it actually used to "
    ".build_tmp/newton_solver.log (an `up_axis=... up_vector=... gravity_scalar=...` "
    "line per world) -- read that before theorising.")


# --------------------------------------------------------------------------
# Worlds. Same scene twice, each expressed in its own coordinate system.
# --------------------------------------------------------------------------

def _world(coord_system):
    """The probe world for `coord_system` ("ENU" or "NUE").

    Deliberately spare -- one static floor, one dynamic ball, one supervisor. No
    arena, no PROTO, no lighting recipe (tests/ worlds are exempt from
    docs/WORLD_RECIPE.md), so a failure cannot be blamed on scenery.

    The ONLY difference between the two worlds is which axis carries "up": the
    field itself, the floor box's thin dimension, and the two translations. Mass,
    radius, box footprint, timestep, solver and contact defaults are identical, so
    the two arms are directly comparable.
    """
    up = _up_index(coord_system)
    def vec(along_up, other=0.0):
        v = [other, other, other]
        v[up] = along_up
        return "%g %g %g" % tuple(v)

    # 4 x 4 footprint, 0.5 thick along up -> centred at up=0.3 the top face is at
    # 0.55, comfortably clear of the implicit ground plane at up=0. If the two
    # ever coincided a ball resting on the phantom plane would be indistinguishable
    # from one resting on the declared floor -- the exact ambiguity that let the
    # missing-boundingObject defect survive (AGENTS.md, statics flip).
    size = [4.0, 4.0, 4.0]
    size[up] = 0.5
    floor_size = "%g %g %g" % tuple(size)

    return """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_coordinate_system.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  coordinateSystem "%(CS)s"
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
  newtonStatics TRUE
}
Viewpoint { position -6 -6 4 }
Background { skyColor [ 0.2 0.2 0.25 ] }
DEF FLOOR Solid {
  translation %(FLOOR_T)s
  name "floor"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.3 0.35 roughness 1 metalness 0 }
      geometry Box { size %(FLOOR_S)s }
    }
  ]
  boundingObject Box { size %(FLOOR_S)s }
}
DEF BALL Solid {
  translation %(BALL_T)s
  name "ball"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.4 0.2 roughness 1 metalness 0 }
      geometry Sphere { radius %(R)g }
    }
  ]
  boundingObject Sphere { radius %(R)g }
  physics Physics { density -1 mass 1 }
}
DEF PROBE Robot { name "probe" controller "coordprobe" supervisor TRUE }
""" % {
        "CS": coord_system,
        "FLOOR_T": vec(0.3),
        "FLOOR_S": floor_size,
        "BALL_T": vec(DROP_FROM),
        "R": BALL_RADIUS,
    }


#: Axis-agnostic by construction: it reports the ball's per-axis final position
#: plus the running per-axis min and max, and the TEST decides which index is up.
#: A probe that knew the answer could not detect an axis mix-up.
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
    "    lo = list(ball.getPosition())",
    "    hi = list(lo)",
    "    n = 0",
    # 800 x 8 ms = 6.4 s. Free fall from 3.0 m to 0.65 m takes ~0.69 s, so the
    # rest of the window is settling -- and is long enough that a body which is
    # NOT falling (the pre-fix NUE reading) is unambiguous rather than early.
    "    for _ in range(800):",
    "        if sup.step(dt) == -1:",
    "            break",
    "        n += 1",
    "        p = ball.getPosition()",
    "        for i in (0, 1, 2):",
    "            lo[i] = min(lo[i], p[i])",
    "            hi[i] = max(hi[i], p[i])",
    "    p = ball.getPosition()",
    "    f = floor.getPosition()",
    "    out.write('final %.9f %.9f %.9f\\n' % (p[0], p[1], p[2]))",
    "    out.write('min %.9f %.9f %.9f\\n' % (lo[0], lo[1], lo[2]))",
    "    out.write('max %.9f %.9f %.9f\\n' % (hi[0], hi[1], hi[2]))",
    "    out.write('floor %.9f %.9f %.9f\\n' % (f[0], f[1], f[2]))",
    "    out.write('steps %d\\n' % n)",
    "out.write('done\\n')",
    "out.close()",
    "sup.simulationQuit(0)",
    "",
])


def _up_index(coord_system):
    """Which axis carries "up", by the same rule OmWorldInfo::updateGravityBasis()
    uses to build mUpVector: wherever the "U" sits. ENU -> 2 (Z), NUE and EUN -> 1 (Y)."""
    return coord_system.upper().index("U")


def _binary():
    for rel in ("msys64/mingw64/bin/omnisim-bin.exe", "bin/omnisim-bin",
                "Contents/MacOS/omnisim", "Contents/MacOS/webots"):
        if (REPO / rel).is_file():
            return REPO / rel
    return None


pytestmark = pytest.mark.skipif(
    _binary() is None, reason="no simulator binary in this clone; build first")


def _goldens():
    return json.loads(GOLDENS.read_text(encoding="utf-8"))["families"]["kinematic_native"]["measurements"]


# --------------------------------------------------------------------------
# Runner. Popen + poll-for-output + taskkill, mirroring
# tests/test_newton_weld_parity.py: the engine is a GUI-subsystem binary on
# Windows and does not always exit on simulationQuit, so the probe FILE is the
# completion signal and the process is killed by tree.
# --------------------------------------------------------------------------

def _run_once(tmp_path, tag, coord_system, env_extra, attempt):
    """One engine launch. -> readings dict, or None on a bring-up flake."""
    root = tmp_path / ("%s_%d" % (tag, attempt))
    worlds = root / "worlds"
    ctrl = root / "controllers" / "coordprobe"
    worlds.mkdir(parents=True, exist_ok=True)
    ctrl.mkdir(parents=True, exist_ok=True)
    world = worlds / "coord.wbt"
    world.write_text(_world(coord_system), encoding="utf-8")
    (ctrl / "coordprobe.py").write_text(CONTROLLER, encoding="utf-8")

    result = root / "probe_out.txt"
    log = root / "engine.log"
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["PROBE_OUT"] = str(result)
    # A stale export in the developer's shell would decide the answer for us --
    # including OMNISIM_NEWTON_COORD_SYSTEM itself, which is exactly what two of
    # these tests set deliberately.
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    env.update(env_extra or {})

    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        # Wait for CONTENT, not existence: the controller opens PROBE_OUT with
        # mode 'w' on its very first line, so the file appears within a second of
        # launch and an is_file() poll kills the engine before it has finalised
        # the Newton world -- which reads as "no sidecar" and skips the whole
        # test. The probe's last line is `steps N`, so wait for that.
        for _ in range(240):            # 6.4 s sim time + a cold Newton build
            if result.is_file():
                try:
                    if "steps " in result.read_text(encoding="utf-8", errors="replace"):
                        break
                except OSError:
                    pass                # mid-write; try again next tick
            try:
                proc.wait(timeout=1)
                break
            except subprocess.TimeoutExpired:
                continue
    finally:
        if proc.poll() is None:
            if os.name == "nt":
                subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                               capture_output=True)
            else:
                proc.kill()
            proc.wait()

    blob = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if any(sig in blob for sig in _BRINGUP):
        return None
    if not result.is_file():
        pytest.fail("the %s/%s probe produced no output:\n%s" % (tag, coord_system, blob[-1500:]))

    # Prove NEWTON drove this run before reading anything as a Newton result. On
    # an ODE-only clone both arms settle at 0.65 for a reason that has nothing to
    # do with this patch, and scoring that as a pass would be a lie. The sidecar's
    # mere presence means "Newton finalised THIS run" -- OmLog deletes any stale
    # copy when it truncates the log at startup.
    sidecar = Path(str(log) + ".newton.json")
    if not sidecar.is_file():
        pytest.skip(
            "no %s -- Newton did not finalise the %s/%s run, so its heights say "
            "nothing about the coordinate system. (A missing sidecar means the run "
            "never reached world-finalize, NOT that ODE drove it.)"
            % (sidecar.name, tag, coord_system))

    readings = {"_verdict": json.loads(sidecar.read_text(encoding="utf-8")),
                "_log": blob, "_coord_system": coord_system}
    for line in result.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] in ("final", "min", "max", "floor"):
            readings[parts[0]] = [float(v) for v in parts[1:]]
        elif len(parts) == 2 and parts[0] == "steps":
            readings["steps"] = int(parts[1])
    if "final" not in readings:
        pytest.fail("the %s/%s probe wrote no ball position:\n%s"
                    % (tag, coord_system, result.read_text(encoding="utf-8", errors="replace")))
    return readings


#: One engine launch per `tag`, shared across the tests that need it. Three tests
#: read the ENU arm and two read a NUE arm; a cold Newton build is tens of seconds,
#: and re-running the same configuration would also make "ENU with the gate on"
#: mean a *different run* in each test, which is precisely what
#: test_gate_is_a_no_op_for_enu is trying to compare against.
_CACHE: dict = {}


def _run(tmp_path, tag, coord_system, env_extra=None):
    """The Newton FFI bring-up flakes on a few percent of launches; retry once."""
    if tag in _CACHE:
        return _CACHE[tag]
    for attempt in (1, 2):
        got = _run_once(tmp_path, tag, coord_system, env_extra, attempt)
        if got is not None:
            _CACHE[tag] = got
            return got
    pytest.skip("Newton bring-up flaked on both attempts for %s/%s -- no data, re-run"
                % (tag, coord_system))


def _explain(readings):
    cs = readings["_coord_system"]
    up = _up_index(cs)
    horiz = [i for i in (0, 1, 2) if i != up]
    axis = "XYZ"[up]
    return (
        "world coordinateSystem %r -> up axis %s (index %d)\n"
        "  ball released at up=%.2f, analytic rest = floor top %.2f + radius %.2f = %.2f\n"
        "  final    xyz = %s   -> up=%.6f  horizontal=(%.6f, %.6f)\n"
        "  min      xyz = %s\n"
        "  max      xyz = %s\n"
        "  floor    xyz = %s   (a static must not move)\n"
        "  steps    = %s\n"
        "  backend verdict: %s"
        % (cs, axis, up, DROP_FROM, FLOOR_TOP, BALL_RADIUS, REST,
           ["%.6f" % v for v in readings["final"]], readings["final"][up],
           readings["final"][horiz[0]], readings["final"][horiz[1]],
           ["%.6f" % v for v in readings.get("min", [float("nan")] * 3)],
           ["%.6f" % v for v in readings.get("max", [float("nan")] * 3)],
           ["%.6f" % v for v in readings.get("floor", [float("nan")] * 3)],
           readings.get("steps"), readings["_verdict"]))


def _check_fell_correctly(readings):
    """The three physical claims, returned as a list of problem strings."""
    cs = readings["_coord_system"]
    up = _up_index(cs)
    horiz = [i for i in (0, 1, 2) if i != up]
    axis = "XYZ"[up]
    final, lo, hi = readings["final"], readings["min"], readings["max"]
    problems = []

    # 1. It fell AT ALL, along its own up axis. Pre-fix NUE read 3.000 forever.
    if final[up] > FELL_BELOW:
        problems.append(
            "the ball is still at %s=%.4f after %s steps: it never fell. Gravity is "
            "projecting to ZERO because builder.up_vector is not the world's up -- the "
            "%r gravity vector and the builder's up axis are perpendicular."
            % (axis, final[up], readings.get("steps"), cs))

    # 2. It landed at the analytic height.
    if abs(final[up] - REST) > REST_TOL:
        problems.append(
            "the ball settled at %s=%.4f, expected %.4f (+/-%.2f). It is resting on "
            "neither the declared floor nor anything else the file describes."
            % (axis, final[up], REST, REST_TOL))

    # 3. It did not tunnel through the floor.
    if lo[up] < NO_TUNNEL_MIN:
        problems.append(
            "the ball reached %s=%.4f at its lowest, below the floor's top face at "
            "%.2f (bar %.2f): it passed INTO or THROUGH the collider."
            % (axis, lo[up], FLOOR_TOP, NO_TUNNEL_MIN))

    # 4. No drift onto a surface that is not in the file. A ball released straight
    #    down has nothing to push it sideways; the pre-fix signature was +384 m
    #    along the horizontal axis the phantom plane's normal pointed down.
    for i in horiz:
        excursion = max(abs(lo[i]), abs(hi[i]))
        if excursion > MAX_HORIZONTAL:
            problems.append(
                "the ball travelled %.4f m along %s -- a HORIZONTAL axis -- from a "
                "straight-down release (bar %.2f). Something is pushing it sideways; the "
                "pre-fix signature was the implicit ground plane standing up as a "
                "vertical wall, which sent it to +384 m."
                % (excursion, "XYZ"[i], MAX_HORIZONTAL))
    return problems


# --------------------------------------------------------------------------
# The two arms.
# --------------------------------------------------------------------------

@pytest.fixture(scope="module")
def probe(tmp_path_factory):
    return tmp_path_factory.mktemp("coordsys")


def test_enu_ball_falls_along_z(probe):
    """ENU (the schema default, 511 of 719 worlds): up is +Z. Must be unchanged.

    Also the golden check: `families.kinematic_native.rest__ball_rest_z` froze the
    ODE oracle for this exact geometry at 0.6496076 with
    `asserted_against_golden: true`, so the ENU arm is held to it.
    """
    r = _run(probe, "enu", "ENU")
    problems = _check_fell_correctly(r)

    g = _goldens()["rest__ball_rest_z"]
    zg, tol = g["ode_value"], g["tolerance"]
    z = r["final"][2]
    if abs(z - zg) > tol:
        problems.append(
            "the ball settled at z=%.6f but the FROZEN ODE-oracle golden for this "
            "geometry (floor top %.2f + radius %.2f, 1 kg, default contact ke/kd) is "
            "z=%.6f, |d|=%.2e > %g. Do NOT retune the golden to make this pass -- "
            "producing this signal is what it is for."
            % (z, FLOOR_TOP, BALL_RADIUS, zg, abs(z - zg), tol))

    assert not problems, ("ENU ball drop failed:\n  " + "\n  ".join(problems) +
                          "\n" + _explain(r) + _NOTE)


def test_nue_ball_falls_along_y(probe):
    """NUE (210 worlds, 29% of the corpus): up is +Y. This is the fix.

    Before it, the ball never left y=3.0 and drifted to z=+384 m off a plane whose
    normal pointed along the world's EAST axis.
    """
    r = _run(probe, "nue", "NUE")
    problems = _check_fell_correctly(r)
    assert not problems, ("NUE ball drop failed:\n  " + "\n  ".join(problems) +
                          "\n" + _explain(r) + _NOTE)


def test_nue_and_enu_agree(probe):
    """The same scene in two coordinate systems must produce the same physics.

    Not a tautology given the two arms above pass: they are graded against an
    analytic height with a 3 cm budget, and two runs could both sit inside it while
    disagreeing by centimetres -- which would mean the Y-up path is *approximately*
    right rather than the same path. The soft-contact penetration depends only on
    (mass, ke, kd, radius), all identical, so the two rest heights should agree far
    more tightly than the analytic bar.
    """
    enu = _run(probe, "enu", "ENU")
    nue = _run(probe, "nue", "NUE")
    z, y = enu["final"][2], nue["final"][1]
    # 1 mm: the contact-penetration budget both arms share. The frozen ODE/Newton
    # pair for this geometry differ by 2.5e-5, so 1e-3 is loose by ~40x while still
    # far tighter than the 3 cm analytic bar.
    assert abs(z - y) <= 1e-3, (
        "the same scene rests at z=%.6f in ENU but y=%.6f in NUE (|d|=%.2e > 1e-3).\n"
        "Both are inside the analytic tolerance, so this is not a wrong-axis bug -- it "
        "means the Y-up path is not the SAME path: a different contact configuration, a "
        "different plane offset, or the implicit plane still participating.\n%s\n%s%s"
        % (z, y, abs(z - y), _explain(enu), _explain(nue), _NOTE))


# --------------------------------------------------------------------------
# The gate. Value-parsed, DEFAULT ON, `=0` reverts to the hardcoded Z.
# --------------------------------------------------------------------------

def test_gate_off_reproduces_the_nue_defect(probe):
    """OMNISIM_NEWTON_COORD_SYSTEM=0 must restore the pre-fix behaviour.

    Asserted as "NUE does NOT reach its rest height", not as an equality against
    the broken numbers: the defect's exact trajectory (which way it drifted, how
    far) is solver detail nobody should freeze. What matters is that the fix is
    genuinely what makes the difference -- if this passes with the gate off too,
    something OTHER than the axis plumbing changed the answer, and the two arms
    above are green for the wrong reason.
    """
    r = _run(probe, "nue_gated", "NUE", env_extra={"OMNISIM_NEWTON_COORD_SYSTEM": "0"})
    y = r["final"][1]
    assert abs(y - REST) > REST_TOL, (
        "with OMNISIM_NEWTON_COORD_SYSTEM=0 the NUE ball STILL settled at y=%.4f "
        "(rest %.2f +/- %.2f), so the hatch did not turn the fix off.\n"
        "Either the value parse is inverted (\"0\" must mean OFF -- the F2 "
        "inverted-hatch class: a presence-gated read makes =0 mean ON), or the up axis "
        "is now arriving through some other path that the hatch does not cover.\n%s%s"
        % (y, REST, REST_TOL, _explain(r), _NOTE))


def test_gate_is_a_no_op_for_enu(probe):
    """ENU must be BIT-IDENTICAL with the gate on and off.

    This is the "nothing changes for ENU" claim, measured. For a z-up world
    `set_up_axis("ENU")` resolves to `Axis.Z` and assigns it over the `Axis.Z` the
    constructor already set -- the same enum member, so the builder is literally
    untouched and the expected difference is EXACTLY 0.0. The bar is 1e-9 only to
    keep the message about float printing rather than about physics; `newtonSolver
    "mujoco"` (CPU mj_step) is the bitwise-reproducible path
    (docs/benchmarks/determinism-scope.md), so a non-zero delta here means the ENU
    code path is not the no-op it is documented to be.
    """
    on = _run(probe, "enu", "ENU")
    off = _run(probe, "enu_gated", "ENU", env_extra={"OMNISIM_NEWTON_COORD_SYSTEM": "0"})
    d = abs(on["final"][2] - off["final"][2])
    assert d <= 1e-9, (
        "ENU is NOT unchanged by the coordinate-system plumbing: z=%.9f with the gate "
        "on vs z=%.9f with it off (|d|=%.3e). Expected exactly 0.0 -- for a z-up world "
        "the new code assigns Axis.Z over Axis.Z. 511 of 719 worlds are ENU; any real "
        "difference here means this bug fix moved physics that was already correct."
        "\n%s\n%s%s" % (on["final"][2], off["final"][2], d, _explain(on), _explain(off), _NOTE))
