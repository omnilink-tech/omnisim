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

"""Newton-native Connector welds must match the ODE weld semantics.

WHY THIS EXISTS
---------------
Connector locks were one of the two ODE mechanisms blocking src/ode deletion
(_scratch/design_weld_touch.md): on Newton-backed Solids the ODE
dJointCreateFixed weld constrained artificially-disabled proxy bodies, so a
"locked" pair held NOTHING and the dJointFeedback rupture check read eternal
zeros. The replacement is a pre-allocated MuJoCo equality-weld slot per device,
engaged at lock time at the pair's current relative pose and read back through
the solved efc rows -- gated by the value-parsed OMNISIM_NEWTON_WELDS.

THE MEASUREMENT
---------------
One generated world: a bodiless ANCHOR robot 1.5 m up carries an active
Connector facing DOWN; a free 1 kg PAYLOAD box 0.2 m below carries the passive
counterpart facing UP. This exercises the bodiless-side swap (the weld's obj1
becomes the payload welded to the WORLD -- the same internal swap ODE's
dJointAttach did for a NULL body1, with the tensile sign flipped through
mIsJointInversed).

  hold world (tensileStrength -1):
    lock    -> the payload hangs mid-air: pose drift < 1 mm over 2 s and it
               must NOT be on the floor
    unlock  -> the payload falls away
  rupture world (tensileStrength 2.5 N per connector -- the PAIR sums
  effective strengths, so the weld ruptures at 5 N total < m*g = 9.81 N):
    lock    -> the weld must AUTO-DETACH (tension > threshold)

THE ODE ARM IS GONE -- AND IT WAS NEVER A NUMERICAL ORACLE
----------------------------------------------------------
This test used to run an OMNISIM_FORCE_ODE=1 arm beside the Newton one, but
read carefully: no assertion ever compared an ODE number to a Newton number.
Both arms were judged INDEPENDENTLY against the same absolute semantic
thresholds below. The ODE arm was corroboration, not an oracle. src/ode is
being deleted, so that arm has been DELETED from this file; its measured
values are frozen for the record in

    tests/goldens/ode_oracle_goldens.json  ->  families.weld

with `asserted_against_golden: false` on the entries where the two backends
legitimately disagree. THE RUPTURE PATH IS ONE OF THOSE, and the divergence is
measured: ODE RATCHETS (upstream Connector semantics keep isLocked TRUE through
a rupture and autoLock re-captures the payload the very next tick while it is
still inside distanceTolerance, so it creeps z 1.2868 -> 1.2079, sag 0.0789 m
over ~40 rupture/re-lock cycles) while Newton's softer engage transient lets
the payload escape recapture and reach the floor (sag 1.1775 m). Both are "the
rupture fired". Freezing ODE's 0.0789 m as an equality target would be WRONG,
so the rupture assertion stays semantic.

The one place the two backends DID agree tightly is the held pose, so that one
IS asserted against the frozen ODE value, at the same 1 mm the rigidity check
uses.

    python -m pytest tests/test_newton_weld_parity.py -v
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

#: soft-equality sag under 1 kg measured at 4e-4 m on the bundled mujoco;
#: ODE's hard joint read ~1e-9. 1 mm covers both with margin.
HOLD_DRIFT_TOL = 1e-3
#: payload spawn z = 1.3 (box half-size 0.1 -> rest-on-floor z = 0.1)
Z_HELD_MIN = 1.0
Z_FALLEN_MAX = 0.6

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")

_GOLDEN_NOTE = (
    "\n\nThe frozen reference numbers live in tests/goldens/ode_oracle_goldens.json "
    "(families.weld) and are FROZEN ODE-ORACLE VALUES measured before src/ode was "
    "removed. THE ODE ARM NO LONGER EXISTS and cannot be re-run. Note that most weld "
    "entries are recorded with asserted_against_golden: false because the two backends "
    "legitimately diverge on the rupture path -- the thresholds here, not the ODE "
    "numbers, are the contract. Do not retune a golden to make this pass.")


def _goldens():
    return json.loads(GOLDENS.read_text(encoding="utf-8"))["families"]["weld"]["measurements"]


def _world_text(tensile):
    connector_common = """
      type "%(T)s"
      isLocked FALSE
      autoLock %(AL)s
      unilateralLock TRUE
      unilateralUnlock TRUE
      distanceTolerance 0.3
      axisTolerance 0.5
      rotationTolerance 3.14
      numberOfRotations 0
      snap FALSE"""

    return """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_weld_parity.py -- do not commit.

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
DEF ANCHOR Robot {
  translation 0 0 1.5
  name "anchor"
  controller "weld_parity_probe"
  supervisor TRUE
  children [
    Connector {
      name "conn"
      rotation 0 1 0 1.5708
%(CC_ACTIVE)s
      tensileStrength %(TENSILE)s
      shearStrength -1
    }
  ]
}
DEF PAYLOAD Solid {
  translation 0 0 1.3
  name "payload"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.8 0.4 0.2 roughness 1 metalness 0 }
      geometry Box { size 0.2 0.2 0.2 }
    }
    Connector {
      name "pconn"
      rotation 0 1 0 -1.5708
%(CC_PASSIVE)s
      tensileStrength %(TENSILE)s
      shearStrength -1
    }
  ]
  boundingObject Box { size 0.2 0.2 0.2 }
  physics Physics { density -1 mass 1 }
}
""" % {
        # autoLock TRUE on the active side: lock() can race world finalize (the
        # weld slot engages only post-finalize), and the autolock retry in
        # prePhysicsStep closes that race tick-by-tick.
        "CC_ACTIVE": connector_common % {"T": "active", "AL": "TRUE"},
        "CC_PASSIVE": connector_common % {"T": "passive", "AL": "FALSE"},
        "TENSILE": tensile,
    }


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run_newton(scenario, tensile, tmp_path, attempt):
    """Run the Newton arm. -> probe dict or None (bring-up flake)."""
    world = WORLDS / (".weld_parity_%s.wbt" % scenario)
    world.write_text(_world_text(tensile), encoding="utf-8")
    out = tmp_path / ("probe_%s_newton_%d.json" % (scenario, attempt))
    log = tmp_path / ("engine_%s_newton_%d.log" % (scenario, attempt))
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_WELD_PROBE_OUT"] = str(out)
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    env["OMNISIM_NEWTON_WELDS"] = "1"
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(180):          # the probe runs ~3.5 s sim time; poll
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


def _run_with_retry(scenario, tensile, tmp_path):
    """The Newton FFI bring-up flakes ~3% of launches; retry once before skipping."""
    for attempt in (1, 2):
        got = _run_newton(scenario, tensile, tmp_path, attempt)
        if got is not None:
            return got
    return None


def test_weld_hold_and_release(tmp_path):
    newton = _run_with_retry("hold", -1, tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")
    g = _goldens()
    problems = []
    if newton["hold_drift"] > HOLD_DRIFT_TOL:
        problems.append("locked pair drifted %.4g m over 2 s (> %g) -- the weld is not rigid"
                        % (newton["hold_drift"], HOLD_DRIFT_TOL))
    z = newton["p_hold_end"][2]
    if z < Z_HELD_MIN:
        problems.append("payload at z=%.3f during the hold window -- the lock did not hold it "
                        "airborne" % z)
    if newton["p_after_unlock"][2] > Z_FALLEN_MAX:
        problems.append("payload still at z=%.3f after unlock -- the weld did not release"
                        % newton["p_after_unlock"][2])
    # The held pose is the one weld quantity the two backends agreed on tightly,
    # so it IS checked against the frozen ODE value, at the same 1 mm budget the
    # rigidity check uses.
    zg = g["hold__payload_z_while_locked"]["ode_value"]
    if abs(z - zg) > HOLD_DRIFT_TOL:
        problems.append("payload held at z=%.6f but the FROZEN ODE-oracle weld held it at "
                        "z=%.6f (|d|=%.2e > %g) -- the Newton equality weld is engaging at a "
                        "different relative pose than ODE's rigid joint did"
                        % (z, zg, abs(z - zg), HOLD_DRIFT_TOL))
    assert not problems, (
        "weld hold/release failed:\n  " + "\n  ".join(problems) +
        "\n(Newton holds via the MuJoCo equality-weld slot behind OMNISIM_NEWTON_WELDS=1; "
        "ODE held via dJointCreateFixed.)" + _GOLDEN_NOTE)


#: rupture is proven by SAG, not by reaching the floor -- see the module
#: docstring: ODE ratchets to ~0.079 m of sag, Newton falls clear. A healthy
#: infinite-strength weld sags < 1e-3 (the hold test's tolerance), so 5 cm of
#: sag is unreachable without repeated detach.
RUPTURE_SAG_MIN = 0.05


def test_weld_rupture(tmp_path):
    newton = _run_with_retry("rupture", 2.5, tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake on both attempts -- no data, re-run")
    problems = []
    sag = newton["p_lock"][2] - newton["p_hold_end"][2]
    if sag < RUPTURE_SAG_MIN and newton["p_hold_end"][2] > Z_FALLEN_MAX:
        problems.append("payload sagged only %.4g m (z=%.3f) with pair tensile strength 5 N "
                        "under a 9.81 N load -- rupture never fired (the force readback is dead "
                        "or its sign is wrong)" % (sag, newton["p_hold_end"][2]))
    # after unlock() the autolock re-capture is disarmed for good
    if newton["p_after_unlock"][2] > Z_FALLEN_MAX:
        problems.append("payload still at z=%.3f after unlock -- detached weld re-engaged"
                        % newton["p_after_unlock"][2])
    assert not problems, (
        "weld rupture failed:\n  " + "\n  ".join(problems) +
        "\n(tension = force along the owning connector's +x axis, inversion-flagged on the "
        "bodiless side; Newton reads it from the weld's solved efc rows -- force ON obj1, the "
        "ODE f1 convention.)" + _GOLDEN_NOTE)
