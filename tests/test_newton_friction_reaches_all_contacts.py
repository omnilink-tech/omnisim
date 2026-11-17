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

"""Does the friction a world DECLARES actually reach its contacts?

One physical question, asked six ways: a 1 kg box is seated on a static box
ramp at 55 deg (tan 55 = 1.428) and left alone for 2 s of sim time. Coulomb
says it slides whenever mu < 1.428 and sticks whenever mu > 1.428, so the
ramp is a comparator whose output is metres, not a log line: at the engine
default mu = 1.0 the box runs ~4.8 m down the ramp; at mu = 2.0 it does not
move. Both bodies are Boxes, so this is a BOX-BOX contact pair (plus the
box-box pair against the static ramp) -- the case the "does newtonGroundMu
only reach the ground?" question was really about.

WHAT FAILS ON TODAY'S BUILD (2026-08-06, commit be82d0e7):

  world_mu   FAILS  -- WorldInfo.newtonGroundMu 2.0 is declared and ignored.
                      The box slides. Root cause is ORDER, not routing:
                      OmSolid::flushPendingNewtonRegistrations() adds every
                      body and every collision shape in its registration loop
                      and only then, at the tail of the SAME call, pushes the
                      WorldInfo contact block through setContactSolverParams()
                      -> World.set_contact_solver_params(). newton's
                      ModelBuilder.add_shape() has already COPIED cfg.mu into
                      builder.shape_material_mu[i]; changing default_shape_cfg
                      or dropping the _SHAPE_CFG cache afterwards cannot reach
                      a shape that already exists. (The implicit ground plane
                      is added even earlier, in ensureWorldOpen().)
  cp_bridge_pinned  FAILS -- a world that declares friction the STANDARD way,
                      WorldInfo.contactProperties.coulombFriction, gets 1.0.
  cp_bridge_env     FAILS -- same, via the opt-in migration flag.

  env_mu     passes today and after -- OMNISIM_NEWTON_GROUND_MU is read by
             World._contact_value() AT SHAPE-ADD TIME, so the env path was
             never broken. This case is the rig's control: if it ever fails,
             the ramp/threshold is wrong, not the plumbing.
  default_mu passes today and after -- proves the ramp really does slide at
             the default, i.e. that the sticking cases above have something
             to discriminate against.
  cp_no_bridge passes today and after -- pins the blast radius of the
             coulombFriction bridge: an ordinary "auto" world that declares
             coulombFriction is NOT silently re-frictioned. 240 live worlds in
             this tree are that shape (measured, not estimated).

Cost: six engine launches, ~2 s of sim each; a few minutes wall-clock
dominated by cold world loads. Not a smoke test -- run it when you touch the
Newton contact path:

    python -m pytest tests/test_newton_friction_reaches_all_contacts.py -v
    python -m pytest tests/test_newton_friction_reaches_all_contacts.py -k world_mu

Every verdict comes from the recorded trajectory of the box (the shared
lane-1 `omnibench_recorder` supervisor writes an .npz), and every run is
required to PROVE Newton drove it via the `<log>.newton.json` sidecar --
without that check an accidental ODE fall-back would honour coulombFriction
natively and hand back a false PASS on the bridge cases.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
OMNIBENCH = REPO / "tests" / "benchmarks" / "omnibench"
LANE1 = OMNIBENCH / "lane1"
WORLDS = LANE1 / "worlds"

try:
    import numpy as np
except ImportError:  # pragma: no cover - environment probe
    np = None

if str(OMNIBENCH) not in sys.path:
    sys.path.insert(0, str(OMNIBENCH))
try:
    from common import engine_launch  # noqa: E402
except Exception:  # pragma: no cover - environment probe
    engine_launch = None


# ---------------------------------------------------------------- geometry

ANGLE_DEG = 55.0          # tan = 1.428: between the default mu 1.0 and mu 2.0
RAMP_SIZE = "8 2 0.2"     # half-length 4 m
RAMP_Z = 6.0              # downhill end sits at 6 - 4*sin(55) = 2.72 m > 0,
                          # clear of the implicit Newton ground plane at z=0
START_LOCAL_X = -2.5      # uphill start; 6.5 m of runway downhill
DURATION_S = 2.0
DT_MS = 4

#: mu 1.0 gives a = g*(sin55 - 1.0*cos55) = 2.41 m/s^2 -> 4.8 m in 2 s.
SLIDE_MIN_M = 1.0
#: mu 2.0 is 71% of the way to the cone boundary; lane-1 measured 0.6 mm of
#: elliptic-cone creep at 97.5% of it, so 50 mm is ~80x the known creep.
STICK_MAX_M = 0.05

_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)


def _binary():
    if engine_launch is None:
        return None
    return engine_launch.resolve_binary(REPO)


pytestmark = [
    pytest.mark.skipif(np is None, reason="numpy is required to read the trajectory"),
    pytest.mark.skipif(engine_launch is None or not LANE1.is_dir(),
                       reason="omnibench lane 1 is not present in this tree"),
    pytest.mark.skipif(_binary() is None,
                       reason="no simulator binary in this clone; build first"),
]


def _world_text(*, ground_mu=None, coulomb=None, pin_newton=False):
    """One 55-degree ramp world. Same node shapes lane-1's T2 uses.

    ground_mu : emit WorldInfo.newtonGroundMu
    coulomb   : emit a single global WorldInfo.contactProperties block
    pin_newton: emit defaultPhysicsBackend "newton" (the bridge's opt-in)
    """
    th = math.radians(ANGLE_DEG)
    # ⚠ THE SLIDER MUST BE LOW-PROFILE OR THE TEST MEASURES TOPPLING, NOT
    # FRICTION. The first version used a 0.2 m CUBE, whose static tipping
    # angle is atan(half_width/half_height) = 45 degrees exactly -- so at the
    # 55-degree test angle it TUMBLED down the ramp at ANY friction
    # coefficient (measured: net rotation 33-106 degrees, peak |omega| 16-22
    # rad/s, travel 1.9-2.2 m with mu=2 and even mu=5 IN THE LIVE CONTACT --
    # verified via a mj_data.contact probe carrying mu=[2,2,...]), which read
    # exactly like "friction never reached the contact" and sent the
    # diagnosis down the routing-bug path. The routing bug was ALSO real
    # (fixed: WorldInfo contact params now land before any shape is created),
    # but no fix can hold a cube past its tipping angle. The 0.4 x 0.4 x 0.04
    # slab tips at atan(0.2/0.02) = 84 degrees, so 55 degrees genuinely tests
    # the friction cone.
    bz = 0.1 + 0.02 + 0.0005         # ramp half-thickness + slab half-HEIGHT + gap
    wx = START_LOCAL_X * math.cos(th) + bz * math.sin(th)
    wz = RAMP_Z - START_LOCAL_X * math.sin(th) + bz * math.cos(th)
    extra = ""
    if pin_newton:
        extra += '  defaultPhysicsBackend "newton"\n'
    if ground_mu is not None:
        extra += "  newtonGroundMu %.10g\n" % ground_mu
    if coulomb is not None:
        extra += ("  contactProperties [\n"
                  "    ContactProperties {\n"
                  '      material1 "default"\n'
                  '      material2 "default"\n'
                  "      coulombFriction [ %.10g ]\n"
                  "      bounce 0\n"
                  "    }\n"
                  "  ]\n" % coulomb)
    return """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_friction_reaches_all_contacts.py
# and deleted when the test finishes. Do not commit.

WorldInfo {
  gravity 9.81
  basicTimeStep %(dt)g
  FPS 30
  physicsDisableTime 0
  randomSeed 42
  coordinateSystem "ENU"
  newtonSolver "mujoco"
  newtonStatics TRUE
  newtonCone "elliptic"
  newtonImpratio 10
%(extra)s}
Viewpoint {
  position -4 -6 3
  orientation -0.15 0.05 0.99 1.2
}
Background {
  skyColor [ 0.15 0.17 0.22 ]
}
DEF RAMP Solid {
  translation 0 0 %(rampz).10g
  rotation 0 1 0 %(th).10g
  name "ramp"
  children [
    DEF RAMP_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.5 0.5 0.55 roughness 1 metalness 0 }
      geometry Box { size %(rampsize)s }
    }
  ]
  boundingObject USE RAMP_SHAPE
}
DEF SLIDER Solid {
  translation %(wx).10g 0 %(wz).10g
  rotation 0 1 0 %(th).10g
  name "slider"
  children [
    DEF SLIDER_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.85 0.35 0.25 roughness 0.9 metalness 0 }
      geometry Box { size 0.4 0.4 0.04 }
    }
  ]
  boundingObject USE SLIDER_SHAPE
  physics Physics {
    density -1
    mass 1
  }
}
DEF OMNIBENCH_RECORDER Robot {
  name "omnibench_recorder"
  controller "omnibench_recorder"
  controllerArgs [
    "--bodies=SLIDER"
    "--duration=%(dur).10g"
    "--label=frictest"
  ]
  supervisor TRUE
}
""" % {"dt": float(DT_MS), "extra": extra, "rampz": RAMP_Z, "th": th,
       "rampsize": RAMP_SIZE, "wx": wx, "wz": wz, "dur": DURATION_S}


def _travel(case, world_kwargs, extra_env, tmp_path):
    """Run one ramp world on Newton; return metres the box travelled."""
    # The world must be a SIBLING of lane-1's generated worlds so that
    # `controller "omnibench_recorder"` resolves through that project dir.
    # Dot-prefixed + deleted in the finally, like the runaway-watchdog copy.
    world = WORLDS / (".frictest_%s.wbt" % case)
    npz = tmp_path / ("%s.npz" % case)
    log = tmp_path / ("%s.engine.log" % case)
    console = tmp_path / ("%s.console.log" % case)
    world.write_text(_world_text(**world_kwargs), encoding="utf-8")
    try:
        env = engine_launch.build_env("newton", log, repo=REPO,
                                      extra=dict(extra_env, OMNIBENCH_OUT=str(npz)))
        # never inherit the knobs under test from the parent shell
        for k in ("OMNISIM_NEWTON_GROUND_MU", "OMNISIM_NEWTON_CONTACT_KE",
                  "OMNISIM_NEWTON_CONTACT_KD",
                  "OMNISIM_NEWTON_BRIDGE_CONTACT_PROPERTIES"):
            if k not in extra_env:
                env.pop(k, None)
        rc, wall, timed_out = engine_launch.launch_once(
            _binary(), world, env, console, timeout_s=300,
            extra_args=("--stdout", "--stderr"))
    finally:
        try:
            world.unlink()
        except OSError:
            pass

    blob = ""
    for p in (log, console):
        if p.exists():
            blob += p.read_text(encoding="utf-8", errors="replace")
    for sig in _BRINGUP_SIGNATURES:
        if sig in blob:
            pytest.skip("Newton did not come up for %s (known intermittent "
                        "bring-up defect: %r). The run produced no data, so it "
                        "says nothing about friction -- re-run." % (case, sig))
    if timed_out:
        pytest.fail("%s: engine did not exit within 300 s\n%s" % (case, blob[-1500:]))

    # A wrong backend would answer the question wrongly rather than not at
    # all: ODE reads coulombFriction natively and would make the bridge cases
    # pass for the wrong reason.
    verdict = engine_launch.newton_verdict(log)
    assert engine_launch.backend_proven("newton", verdict), (
        "%s: Newton did not provably drive this run (sidecar %s). A friction "
        "verdict from the wrong backend is worse than no verdict.\n%s"
        % (case, json.dumps(verdict), blob[-1500:]))

    assert npz.exists(), ("%s: no trajectory recorded (rc=%s, %.1f s)\n%s"
                          % (case, rc, wall, blob[-1500:]))
    data = np.load(npz)
    pos = data["pos_SLIDER"]
    assert pos.shape[0] > 10, "%s: %d rows recorded" % (case, pos.shape[0])
    return float(np.max(np.linalg.norm(pos - pos[0], axis=1)))


# case -> (world kwargs, extra env, expectation)
CASES = {
    # THE ITEM. Declared in the world, nowhere else. FAILS TODAY.
    "world_mu": (dict(ground_mu=2.0), {}, "stick"),
    # Control: the env path, which was never broken. Passes today.
    "env_mu": (dict(), {"OMNISIM_NEWTON_GROUND_MU": "2.0"}, "stick"),
    # Control: discriminating power. The ramp must slide at the default.
    "default_mu": (dict(), {}, "slide"),
    # The bridge, via the world's explicit Newton pin. FAILS TODAY.
    "cp_bridge_pinned": (dict(coulomb=2.0, pin_newton=True), {}, "stick"),
    # The bridge, via the opt-in migration flag. FAILS TODAY.
    "cp_bridge_env": (dict(coulomb=2.0),
                      {"OMNISIM_NEWTON_BRIDGE_CONTACT_PROPERTIES": "1"}, "stick"),
    # Blast radius: an ordinary "auto" world is NOT re-frictioned. Passes both.
    "cp_no_bridge": (dict(coulomb=2.0), {}, "slide"),
}


@pytest.mark.parametrize("case", sorted(CASES))
def test_declared_friction_reaches_the_contact(case, tmp_path):
    world_kwargs, extra_env, expect = CASES[case]
    travel = _travel(case, world_kwargs, extra_env, tmp_path)
    if expect == "stick":
        assert travel < STICK_MAX_M, (
            "%s: the box travelled %.4f m down a %.0f deg ramp. It should be "
            "held by mu=2.0 (tan %.0f = %.3f). Travelling means the friction "
            "this world asked for never reached the box-box contact -- the "
            "shapes were created before the value arrived."
            % (case, travel, ANGLE_DEG, ANGLE_DEG, math.tan(math.radians(ANGLE_DEG))))
    else:
        assert travel > SLIDE_MIN_M, (
            "%s: the box travelled only %.4f m. It should SLIDE here (mu=1.0 "
            "< tan %.0f = %.3f). If it stuck, this rig has lost its ability "
            "to tell the two frictions apart and the sticking cases above "
            "prove nothing."
            % (case, travel, ANGLE_DEG, math.tan(math.radians(ANGLE_DEG))))
