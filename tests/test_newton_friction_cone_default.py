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

"""Pin the DEFAULT MuJoCo friction cone for Newton worlds.

WHAT THIS IS ABOUT. A Newton world running `newtonSolver "mujoco"` used to get
MuJoCo's stock friction cone: PYRAMIDAL, impratio 1. The pyramidal cone is the
pyramid INSCRIBED in the true friction cone, so away from its four axes its
tangential limit is short of mu*N and a body sitting BELOW the static-friction
transition angle creeps downhill. Measured on the omnibench T2 incline at
26 deg with mu = 0.5 (transition angle 26.57 deg): stock pyramidal slid 181 mm
where elliptic + impratio 10 held at 0.6 mm. That is a wrong answer to a
textbook statics question, and it was reachable only by opting in per world
(`WorldInfo.newtonCone "elliptic"` + `newtonImpratio 10`) -- which is a reason
to reach for ODE instead of Newton.

The lane-1 worlds that already pin `newtonCone "elliptic"` (T2 and T6) were
therefore ALREADY measured on the new default and must not move. T7 is
gravity-free and contact-free, so no cone can reach it. T3 (rolling sphere on
an incline) does NOT pin a cone and DOES have contacts, so its exact reference
in tests/test_newton_physics_regression.py is expected to move -- re-measure
and update it in the same commit, as that file's own docstring instructs.

WHAT THIS FILE ASSERTS.

  1. With nothing declared, the solver runs ELLIPTIC with impratio 10.
     >>> FAILS on the pre-patch build (cone=0 pyramidal, impratio absent).
  2. `WorldInfo.newtonCone "pyramidal"` is an EXACT revert -- cone pyramidal
     AND impratio back to MuJoCo stock. One knob, not two. (Passes both sides;
     it guards the escape hatch that makes the flip safe to sequence.)
  3. `OMNISIM_NEWTON_CONE=pyramidal` is the same exact revert process-wide.
     This is the lever for re-verifying every shipped RL champion against the
     old physics without editing 266 world files.
  4. At the friction boundary the default holds where stock creeps.
     >>> FAILS on the pre-patch build (both runs are the same configuration,
     so the ratio is 1.0).

HOW IT OBSERVES THE SOLVER. Two already-shipped introspection hooks, no new
engine surface:
  * OMNISIM_NEWTON_DUMP_MJMODEL=<path> writes `opt: ... cone=<int> ...` read
    straight off `mj_model.opt.cone` (0 = pyramidal, 1 = elliptic).
  * OMNISIM_NEWTON_SAVE_MJCF=<path> writes the compiled MjSpec as MJCF. MuJoCo's
    XML writer emits only NON-default option attributes, so `cone="elliptic"` /
    `impratio="10"` appear exactly when they are not stock.

Cost: three engine launches, ~4 s of sim each. Not a smoke test -- run it when
you touch the Newton solver-kwargs assembly:

    python -m pytest tests/test_newton_friction_cone_default.py -v
"""

from __future__ import annotations

import math
import os
import re
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]

#: Known intermittent embedded-interpreter bring-up failure. Since 85fa6bde a
#: world that asked for Newton is refused rather than quietly run on ODE, so
#: such a run yields no data at all -- an instrument outcome, not a physics
#: result, and reported as a skip naming the defect.
_BRINGUP_SIGNATURES = (
    "can't initialize sys standard streams",
    "the Newton runtime is INSTALLED but did not come up",
    "Refusing to run it on ODE",
)

# --- the scene -------------------------------------------------------------
# A flat slab resting on a 44 deg ramp. Newton's ground friction defaults to
# mu = 1.0 (OmNewtonBackend's World.__init__: OMNISIM_NEWTON_GROUND_MU >
# WorldInfo.newtonGroundMu > 1.0), so the static-friction transition is
# atan(1.0) = 45 deg and 44 deg is 1 deg INSIDE it: the correct answer is "it
# does not move". The angle is deliberately not taken from WorldInfo, because
# WorldInfo.contactProperties.coulombFriction is an ODE-path field that Newton
# ignores -- declaring it here would describe a friction the solver never sees.
#
# The slab is 3x wider than tall (0.3 x 0.3 x 0.1), so its tipping angle is
# atan(0.3/0.1) = 71.6 deg: anything this scene records is sliding, not toppling.
_ANGLE_RAD = math.radians(44.0)
_SLAB_HALF_H = 0.05
_LOCAL_Z = 0.1 + _SLAB_HALF_H          # ramp half-thickness + slab half-height
# Webots `rotation 0 1 0 t` maps a local (0, 0, r) to (r*sin t, 0, r*cos t).
_SLAB_X = _LOCAL_Z * math.sin(_ANGLE_RAD)
_SLAB_Z = 5.0 + _LOCAL_Z * math.cos(_ANGLE_RAD)

WORLD = """#VRML_SIM R2025a utf8
WorldInfo {{
  basicTimeStep 4
  gravity 9.81
  coordinateSystem "ENU"
  physicsDisableTime 0
  randomSeed 42
  defaultPhysicsBackend "newton"
  newtonSolver "mujoco"
  newtonStatics TRUE{cone_fields}
}}
Viewpoint {{ orientation -0.15 0.05 0.99 1.2 position -4 -6 6 }}
DEF RAMP Solid {{
  translation 0 0 5
  rotation 0 1 0 {angle!r}
  name "ramp"
  children [ DEF RAMP_SHAPE Shape {{ geometry Box {{ size 8 2 0.2 }} }} ]
  boundingObject USE RAMP_SHAPE
}}
DEF SLAB Solid {{
  translation {sx!r} 0 {sz!r}
  rotation 0 1 0 {angle!r}
  name "slab"
  children [ DEF SLAB_SHAPE Shape {{ geometry Box {{ size 0.3 0.3 0.1 }} }} ]
  boundingObject USE SLAB_SHAPE
  physics Physics {{ mass 1 }}
}}
DEF PROBE Robot {{ name "probe" controller "coneprobe" supervisor TRUE }}
"""

# Settle for 1 s, then measure how far the slab travels over the next 3 s. The
# settle window removes the placement transient, so what is left is creep.
CONTROLLER = '\n'.join([
    "import math, os",
    "from omnisim import Supervisor",
    "",
    "out = open(os.environ['PROBE_OUT'], 'w', buffering=1)",
    "sup = Supervisor()",
    "dt = int(sup.getBasicTimeStep())",
    "slab = sup.getFromDef('SLAB')",
    "settle = int(round(1000.0 / dt))",      # 1 s
    "total = int(round(4000.0 / dt))",       # 4 s
    "p0 = None",
    "p = slab.getPosition()",
    "for k in range(total):",
    "    if sup.step(dt) == -1:",
    "        break",
    "    if k == settle:",
    "        p0 = slab.getPosition()",
    "    p = slab.getPosition()",
    "if p0 is None:",
    "    p0 = p",
    "out.write('creep %.9f\\n' % math.dist(p0, p))",
    "out.write('final_z %.9f\\n' % p[2])",
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


def _run(work: Path, tag: str, cone_fields: str = "", env_cone: str | None = None):
    """One engine launch. Returns the resolved solver options plus the creep."""
    root = work / tag
    worlds = root / "worlds"
    ctrl = root / "controllers" / "coneprobe"
    worlds.mkdir(parents=True, exist_ok=True)
    ctrl.mkdir(parents=True, exist_ok=True)
    (worlds / "cone.wbt").write_text(
        WORLD.format(cone_fields=cone_fields, angle=_ANGLE_RAD,
                     sx=_SLAB_X, sz=_SLAB_Z),
        encoding="utf-8")
    (ctrl / "coneprobe.py").write_text(CONTROLLER, encoding="utf-8")

    result, log = root / "probe_out.txt", root / "engine.log"
    dump, mjcf = root / "mjmodel.txt", root / "model.xml"
    env = dict(os.environ, OMNISIM_HOME=str(REPO), PROBE_OUT=str(result),
               OMNISIM_LOG_PATH=str(log),
               OMNISIM_NEWTON_DUMP_MJMODEL=str(dump),
               OMNISIM_NEWTON_SAVE_MJCF=str(mjcf))
    # Never let an ambient override decide what this test measures.
    env.pop("OMNISIM_NEWTON_IMPRATIO", None)
    env.pop("OMNISIM_FORCE_ODE", None)
    env.pop("OMNISIM_LEGACY", None)
    if env_cone is None:
        env.pop("OMNISIM_NEWTON_CONE", None)
    else:
        env["OMNISIM_NEWTON_CONE"] = env_cone

    try:
        subprocess.run([str(_binary()), "--batch", "--mode=fast", "--no-rendering",
                        "--minimize", str(worlds / "cone.wbt")],
                       env=env, timeout=180, capture_output=True)
    except subprocess.TimeoutExpired:
        pass

    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    if not result.is_file() or not dump.is_file():
        for sig in _BRINGUP_SIGNATURES:
            if sig in text:
                pytest.skip("Newton did not come up for %r (%r); the run "
                            "produced no data, so this says nothing about the "
                            "friction cone" % (tag, sig))
        pytest.fail("the %r cone probe produced no output.\nprobe=%s dump=%s\n%s"
                    % (tag, result.is_file(), dump.is_file(), text[-1500:]))

    out = {}
    for line in result.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 2:
            out[parts[0]] = float(parts[1])

    # cone, straight off mj_model.opt.cone (0 = pyramidal, 1 = elliptic).
    m = re.search(r"\bcone=(\d+)", dump.read_text(encoding="utf-8", errors="replace"))
    assert m, ("%r: OMNISIM_NEWTON_DUMP_MJMODEL wrote no `cone=` line -- the "
               "dump format changed and this test can no longer see the cone" % tag)
    out["cone"] = int(m.group(1))

    # impratio, from the MJCF export. MuJoCo's writer emits only NON-default
    # option attributes, so absence means stock (1.0). None => no artifact.
    out["impratio"] = None
    if mjcf.is_file():
        xml = mjcf.read_text(encoding="utf-8", errors="replace")
        opt = re.search(r"<option\b(.*?)/?>", xml, re.S)
        if opt:
            a = re.search(r'\bimpratio\s*=\s*"([^"]+)"', opt.group(1))
            out["impratio"] = float(a.group(1)) if a else 1.0
    return out


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    """Three configurations, one engine launch each, shared by every test."""
    work = tmp_path_factory.mktemp("cone")
    return {
        "default": _run(work, "default"),
        "world_pyramidal": _run(
            work, "world_pyramidal", cone_fields='\n  newtonCone "pyramidal"'),
        "env_pyramidal": _run(work, "env_pyramidal", env_cone="pyramidal"),
    }


def test_default_cone_is_elliptic(runs):
    """A world that declares nothing must get the ELLIPTIC cone."""
    got = runs["default"]["cone"]
    assert got == 1, (
        "a Newton `newtonSolver \"mujoco\"` world with no newtonCone declared "
        "resolved to cone=%d (0 = pyramidal, 1 = elliptic). MuJoCo's stock "
        "pyramidal cone is the INSCRIBED pyramid and creeps below the true "
        "static-friction transition angle (181 mm vs 0.6 mm on the T2 incline "
        "at 26 deg, mu 0.5), so it must not be what a world gets by default."
        % got)


def test_default_impratio_is_ten(runs):
    """Elliptic ships together with impratio 10 -- the measured pair."""
    got = runs["default"]["impratio"]
    if got is None:
        pytest.skip("OMNISIM_NEWTON_SAVE_MJCF produced no readable <option>; "
                    "impratio is not observable on this build")
    assert got == pytest.approx(10.0), (
        "default impratio is %r, expected 10.0. The 181 mm -> 0.6 mm result "
        "was measured with cone=elliptic AND impratio=10 together; the two "
        "have never been decomposed, so defaulting one without the other "
        "ships a configuration nobody has measured." % got)


def test_world_pyramidal_pin_is_an_exact_revert(runs):
    """`newtonCone "pyramidal"` must restore MuJoCo stock -- BOTH knobs."""
    r = runs["world_pyramidal"]
    assert r["cone"] == 0, (
        "WorldInfo.newtonCone \"pyramidal\" resolved to cone=%d, not 0. A world "
        "must be able to pin itself back to the old physics; this is the "
        "per-world half of the champion re-verification gate." % r["cone"])
    if r["impratio"] is not None:
        assert r["impratio"] == pytest.approx(1.0), (
            "pinning the pyramidal cone left impratio at %r instead of MuJoCo's "
            "stock 1.0. Pyramidal + impratio 10 is a combination nobody has "
            "measured, and no path may produce it by accident." % r["impratio"])


def test_env_cone_pyramidal_is_an_exact_revert(runs):
    """OMNISIM_NEWTON_CONE=pyramidal reverts a whole run with ONE knob."""
    r = runs["env_pyramidal"]
    assert r["cone"] == 0, (
        "OMNISIM_NEWTON_CONE=pyramidal resolved to cone=%d, not 0. This env var "
        "is how a shipped RL champion gets re-verified against the old contact "
        "physics without editing its world file." % r["cone"])
    if r["impratio"] is not None:
        assert r["impratio"] == pytest.approx(1.0), (
            "OMNISIM_NEWTON_CONE=pyramidal left impratio at %r instead of 1.0, "
            "so a one-knob revert does NOT reproduce the old physics and every "
            "re-verification run would silently be a third configuration."
            % r["impratio"])


def test_default_holds_at_the_friction_boundary(runs):
    """1 deg inside the transition angle, the default must not slide."""
    d, p = runs["default"], runs["world_pyramidal"]
    # Guard: if the slab left the ramp entirely, the scene measured a fall, not
    # creep. Ramp top sits at z = 5 + 0.1*cos(44 deg) ~ 5.072.
    for tag, r in (("default", d), ("pyramidal", p)):
        assert r["final_z"] > 4.5, (
            "the %s run's slab ended at z=%.3f -- it left the ramp, so this "
            "scene measured a fall, not creep. Check newtonStatics."
            % (tag, r["final_z"]))
    if p["creep"] < 5e-3:
        pytest.skip("this build's stock pyramidal cone crept only %.4g m over "
                    "3 s at 44 deg, so the scene is not sitting at the friction "
                    "boundary and cannot discriminate the two cones. Suspect "
                    "the resolved ground mu (WorldInfo.newtonGroundMu / "
                    "OMNISIM_NEWTON_GROUND_MU) rather than the cone."
                    % p["creep"])
    assert d["creep"] <= max(1e-3, 0.2 * p["creep"]), (
        "at 44 deg with mu = 1.0 (transition 45 deg) the slab is 1 deg INSIDE "
        "the friction cone and must not move. The default configuration crept "
        "%.6f m over 3 s against %.6f m for stock pyramidal (ratio %.2f). A "
        "ratio near 1.0 means the default did not change."
        % (d["creep"], p["creep"], d["creep"] / p["creep"] if p["creep"] else 0.0))
