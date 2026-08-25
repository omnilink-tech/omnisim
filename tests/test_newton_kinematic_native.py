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

"""Native KINEMATIC bodies on Newton (MuJoCo mocap), vs the frozen ODE oracle.

WHY THIS EXISTS
---------------
Kinematic collision is kernel blocker #4 of the src/ode deletion
(_scratch/design_kinematic_inertia.md Part 1): a physics-less Solid is
animated by the ENGINE (motor kinematic control, supervisor writes, velocity
integration) and the solver's only job is collision -- under ODE its
NULL-body geoms created one-sided contacts, so a moving kinematic prop pushed
dynamic bodies and a resting one supported them. On Newton those Solids either
gated the whole articulation to ODE (joint endpoints) or registered as
spawn-pinned statics that ignored every subsequent move. The replacement,
behind the value-parsed OMNISIM_NEWTON_KINEMATIC: fixed-root Newton bodies
(which SolverMuJoCo exports as MuJoCo MOCAP bodies) whose pose the engine
pushes straight into mj_data.mocap_pos/mocap_quat per change.

THE MEASUREMENT
---------------
Two generated worlds run with Newton + OMNISIM_NEWTON_KINEMATIC=1 and the
probe controller (kinematic_native_probe):

  push -- a TOP-LEVEL physics-less SLAB box (boundingObject only, the
      supervisor-teleported kinematic prop / conveyor shape) is swept
      horizontally in 0.02 m per-tick increments so its front face plows into
      a 1 kg BALL resting on the floor. The ball must be displaced along the
      sweep and must never tunnel (into the floor or through the slab).

  rest -- a dynamic BALL dropped onto a STATIONARY physics-less SLAB that is
      a HingeJoint's endPoint (the gated-articulation shape: without the flag
      this world routed to ODE with the capability-gate warning). The ball
      must come to rest ON TOP of the slab (slab top 0.55 + ball radius
      0.1 = 0.65) and never tunnel through it.

THE ODE ARM IS GONE
-------------------
Both worlds used to run a second OMNISIM_FORCE_ODE=1 arm. src/ode is being
deleted, so that arm has been DELETED from this file and its measurements are
frozen into

    tests/goldens/ode_oracle_goldens.json  ->  families.kinematic_native

Which of them is an equality target is NOT uniform, and the pre-freeze test
already said so:

  * REST HEIGHT is a genuine parity number -- ODE settled the ball at
    0.6496076, Newton at 0.6496328, 2.5e-5 apart -- so it IS asserted against
    the frozen ODE value, at the same 0.03 m budget, alongside the absolute
    0.65 m geometry.
  * PUSH MAGNITUDE was never an equality target. ODE resolved the per-tick
    0.02 m penetration through ERP, MuJoCo through its contact solver, so the
    old test asserted the displacement bar HARD on the Newton arm and only
    WARNED on an ODE shortfall. Measured divergence: ODE pushed the ball
    1.1295 m, Newton 1.8982 m -- both far above the 0.15 m bar. The golden is
    recorded with asserted_against_golden: false and the threshold is what
    this test keeps.

The geometric non-tunnelling invariants were always absolute and are kept
unchanged: they depend on neither backend and outlive the golden too.

    python -m pytest tests/test_newton_kinematic_native.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"
GOLDENS = REPO / "tests" / "goldens" / "ode_oracle_goldens.json"


def _goldens():
    return json.loads(GOLDENS.read_text(encoding="utf-8"))["families"]["kinematic_native"]["measurements"]


_GOLDEN_NOTE = (
    "\n\nThe frozen reference numbers live in tests/goldens/ode_oracle_goldens.json "
    "(families.kinematic_native) and are FROZEN ODE-ORACLE VALUES measured before src/ode was "
    "removed. THE ODE ARM NO LONGER EXISTS and cannot be re-run. Note that the PUSH entries are "
    "recorded with asserted_against_golden: false -- the push magnitude is solver-dependent and "
    "was never an equality target -- while the REST height is asserted against the golden. "
    "Newton drives kinematic props as MuJoCo mocap bodies through set_kinematic_pose behind "
    "OMNISIM_NEWTON_KINEMATIC=1; ODE used NULL-body one-sided contact joints. Do not retune a "
    "golden to make this pass.")

#: push arm: the slab sweeps 1.2 m ending centred at x=0.4 with its front
#: face at 0.6; a plowed ball (start x=0.4, r=0.1) ends at >= ~0.7. 0.15 m is
#: an order-of-magnitude floor, not a solver-comparison bar.
PUSH_DISPLACEMENT_MIN = 0.15
#: push arm: ball rests at z=0.1 on the floor (top z=0); below 0.05 means it
#: is inside/under the floor -- the tunnelling signature.
PUSH_MIN_Z = 0.05
#: push arm, non-tunnelling vs the slab: at the end the ball centre must be
#: at/ahead of the slab's front face (slab half-size 0.2 + radius 0.1 - a
#: 0.06 penetration allowance).
PUSH_CLEARANCE = 0.2 + 0.1 - 0.06
#: rest arm: slab top z=0.55, ball radius 0.1 -> rest z=0.65.
REST_Z = 0.65
REST_Z_TOL = 0.03
#: rest arm: min z below 0.5 means the ball passed INTO the slab.
REST_MIN_Z = 0.5

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")


PUSH_WORLD = """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_kinematic_native.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  newtonStatics TRUE
  coordinateSystem "ENU"
}
Viewpoint { position -3 0 1 }
Background { skyColor [ 0.2 0.2 0.25 ] }
DEF FLOOR Solid {
  translation 0 0 -0.05
  name "floor"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.3 0.35 roughness 1 metalness 0 }
      geometry Box { size 8 4 0.1 }
    }
  ]
  boundingObject Box { size 8 4 0.1 }
}
DEF SLAB Solid {
  translation -0.8 0 0.2
  name "slab"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.2 0.5 0.8 roughness 1 metalness 0 }
      geometry Box { size 0.4 0.4 0.4 }
    }
  ]
  boundingObject Box { size 0.4 0.4 0.4 }
}
DEF BALL Solid {
  translation 0.4 0 0.1
  name "ball"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.4 0.2 roughness 1 metalness 0 }
      geometry Sphere { radius 0.1 }
    }
  ]
  boundingObject Sphere { radius 0.1 }
  physics Physics { density -1 mass 1 }
}
DEF PROBE Robot {
  translation 0 0 2
  name "probe"
  controller "kinematic_native_probe"
  supervisor TRUE
}
"""

REST_WORLD = """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_kinematic_native.py -- do not commit.

WorldInfo {
  basicTimeStep 8
  newtonStatics TRUE
  coordinateSystem "ENU"
}
Viewpoint { position -3 0 1 }
Background { skyColor [ 0.2 0.2 0.25 ] }
DEF FLOOR Solid {
  translation 0 0 -0.05
  name "floor"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.3 0.35 roughness 1 metalness 0 }
      geometry Box { size 4 4 0.1 }
    }
  ]
  boundingObject Box { size 4 4 0.1 }
}
DEF RIG Robot {
  translation 0 0 0.3
  name "rig"
  controller "kinematic_native_probe"
  supervisor TRUE
  children [
    HingeJoint {
      jointParameters HingeJointParameters { axis 0 0 1 }
      endPoint DEF SLAB Solid {
        name "slab"
        children [
          Shape {
            appearance PBRAppearance { baseColor 0.2 0.5 0.8 roughness 1 metalness 0 }
            geometry Box { size 0.5 0.5 0.5 }
          }
        ]
        boundingObject Box { size 0.5 0.5 0.5 }
      }
    }
  ]
}
DEF BALL Solid {
  translation 0 0 1
  name "ball"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.4 0.2 roughness 1 metalness 0 }
      geometry Sphere { radius 0.1 }
    }
  ]
  boundingObject Sphere { radius 0.1 }
  physics Physics { density -1 mass 1 }
}
"""


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run_newton(scenario, tmp_path, attempt):
    """Run the Newton arm. -> probe dict or None (bring-up flake)."""
    world = WORLDS / (".kinematic_native_%s.wbt" % scenario)
    world.write_text(PUSH_WORLD if scenario == "push" else REST_WORLD,
                     encoding="utf-8")
    out = tmp_path / ("probe_%s_newton_%d.json" % (scenario, attempt))
    log = tmp_path / ("engine_%s_newton_%d.log" % (scenario, attempt))
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_KIN_PROBE_OUT"] = str(out)
    env["OMNISIM_KIN_SCENARIO"] = scenario
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    env["OMNISIM_NEWTON_KINEMATIC"] = "1"
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(180):          # the probe runs ~2-3 s sim time; poll
            if out.exists():
                break
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
        try:
            world.unlink()
        except OSError:
            pass
    blob = log.read_text(errors="replace") if log.exists() else ""
    if any(sig in blob for sig in _BRINGUP):
        return None
    if not out.exists():
        pytest.fail("the %s/newton arm produced no probe output\n%s" % (scenario, blob[-1200:]))
    return json.loads(out.read_text())


def _run_with_retry(scenario, tmp_path):
    """The Newton FFI bring-up flakes ~3% of launches; retry once before skipping."""
    for attempt in (1, 2):
        got = _run_newton(scenario, tmp_path, attempt)
        if got is not None:
            return got
    return None


def _check_push_invariants(d, problems):
    """Geometric non-tunnelling -- absolute, backend-independent."""
    if d["min_ball_z"] < PUSH_MIN_Z:
        problems.append("ball min z=%.3f -- it went under/through the floor"
                        % d["min_ball_z"])
    clearance = d["p_ball_end"][0] - d["p_slab_end"][0]
    if clearance < PUSH_CLEARANCE:
        problems.append("ball centre only %.3f m ahead of the slab centre at the end "
                        "(need >= %.3f: half-size 0.2 + radius 0.1 - penetration allowance) "
                        "-- the slab swept THROUGH the ball" % (clearance, PUSH_CLEARANCE))


def test_kinematic_push(tmp_path):
    newton = _run_with_retry("push", tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")
    g = _goldens()
    problems = []
    _check_push_invariants(newton, problems)
    # Displacement: HARD (a mocap body that cannot push a dynamic body means the
    # native kinematic path is dead -- the entire point of the feature). NOT
    # compared to the frozen ODE displacement: the magnitude is solver-dependent
    # (ODE resolved the per-tick 0.02 m penetration through ERP, MuJoCo through
    # its contact solver) and was never an equality target. See the docstring.
    n_disp = newton["p_ball_end"][0] - newton["p_ball_start"][0]
    if n_disp < PUSH_DISPLACEMENT_MIN:
        problems.append("ball displaced only %.3f m (< %g) by the sweeping kinematic "
                        "slab -- mocap-vs-dynamic contact is not resolving. For scale, the "
                        "frozen ODE oracle pushed it %.3f m." %
                        (n_disp, PUSH_DISPLACEMENT_MIN,
                         g["push__ball_displacement_x"]["ode_value"]))
    assert not problems, (
        "kinematic push failed:\n  " + "\n  ".join(problems) +
        "\n(Newton disp=%.3f m; frozen ODE oracle disp=%.3f m -- recorded, not asserted.)"
        % (n_disp, g["push__ball_displacement_x"]["ode_value"]) + _GOLDEN_NOTE)


def test_kinematic_rest_height_matches_frozen_ode_golden(tmp_path):
    newton = _run_with_retry("rest", tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")
    g = _goldens()
    problems = []
    z = newton["p_ball_end"][2]
    # (1) absolute geometry -- slab top 0.55 + ball radius 0.1
    if abs(z - REST_Z) > REST_Z_TOL:
        problems.append("ball rests at z=%.3f, expected %.2f +/- %.2f (slab top 0.55 + "
                        "radius 0.1) -- it is not resting ON the kinematic slab"
                        % (z, REST_Z, REST_Z_TOL))
    # (2) the frozen ODE oracle, same budget
    zg = g["rest__ball_rest_z"]["ode_value"]
    if abs(z - zg) > REST_Z_TOL:
        problems.append("ball rests at z=%.6f but the FROZEN ODE-oracle rest height is "
                        "%.6f (|d|=%.2e > %g)" % (z, zg, abs(z - zg), REST_Z_TOL))
    # (3) non-tunnelling -- absolute
    if newton["min_ball_z"] < REST_MIN_Z:
        problems.append("ball min z=%.3f -- it passed INTO the slab (tunnelled)"
                        % newton["min_ball_z"])
    assert not problems, (
        "kinematic rest-height parity against the frozen ODE golden failed:\n  "
        + "\n  ".join(problems) +
        "\n(The slab is a HingeJoint's physics-less endPoint -- the exact shape the "
        "capability gate used to route to ODE with reason 'kinematic'. Behind "
        "OMNISIM_NEWTON_KINEMATIC=1 it registers as a fixed-root mocap body whose "
        "box collider the dynamic ball must rest on.)" + _GOLDEN_NOTE)
