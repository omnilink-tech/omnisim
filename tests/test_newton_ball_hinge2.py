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

"""BallJoint + Hinge2Joint must MOVE under Newton, not merely load.

WHY THIS EXISTS
---------------
These two joint types were the last designed kernel blocker before src/ode can
be deleted: 22 tracked worlds/protos use them (the motor2 / motor3 / gyro /
muscle samples, several tests/api joint worlds). Since W2 / W2.2 they are
*registered* with Newton as a native d6 and a native ball joint -- but PASSIVELY:
their motors reached nothing and their position sensors read an ODE joint that a
Newton-driven world never advances. So the articulation was admitted to Newton,
looked healthy, loaded clean, and then silently did not move. Deleting ODE in
that state would have frozen those worlds without a single error line.

OMNISIM_NEWTON_BALL_HINGE2 (value-parsed, DEFAULT ON since 2026-08-17) turns on
per-axis actuation + Newton-side angle readback for both types. This test is
what made the flag flippable, and it asserts MOTION, never "it loaded":

  ball    -- a 1 kg bob on a 3-DoF BallJoint, 0.4 m below and 0.15 m to the +x
             side of the anchor, swings under gravity. Asserted: the swing shows
             up on the joint's SECOND axis (the x-z plane rotates about axis 2 =
             axis3 x axis = +y for the default triad) while axes 1 and 3 stay
             quiet, the bob's distance to the anchor stays constant (the
             constraint HOLDS -- a free-falling bob's radius runs away), and the
             bob actually moves.
  hinge2  -- both axes are position servos on an endpoint whose COM sits ON the
             anchor, so this measures actuator + readback rather than load. Both
             axes must reach their commanded angle, and re-commanding axis 2 must
             not drag axis 1 with it. That coupling is the classic failure mode
             of a composed universal joint, along with "axis 2 never moves".

ARMS
----
ONE ARM: Newton. There used to be a second, OMNISIM_FORCE_ODE=1, used as the
ORACLE. src/ode was DELETED (commit bdc02139) and it is gone -- see the note
above the test functions for exactly which three tests went with it and what
claim was lost. The design decision that saved this file is that every absolute
number was already asserted on the Newton arm on its own, so the test outlived
ODE; only the comparative sign-convention check did not.

    python -m pytest tests/test_newton_ball_hinge2.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
WORLDS = REPO / "tests" / "physics" / "worlds"

#: The bob hangs 0.4 m below and 0.15 m to the side of the anchor, so it starts
#: atan(0.15 / 0.4) = 0.359 rad off vertical and, released from rest, swings
#: through 2 x 0.359 = 0.718 rad. ANALYTIC (derived from the world geometry
#: below), not measured -- these bounds are deliberately loose enough that any
#: plausible integrator lands inside, and tight enough that a bob which is not
#: actually constrained to the anchor cannot.
#: ⚠ The first green run of this file should freeze the MEASURED ODE-arm values
#: into tests/goldens/ode_oracle_goldens.json (per that file's rules: measured
#: only, never hand-written) so the comparative assertion below survives the
#: deletion of src/ode. The analytic bounds here are what the test asserts until
#: then, and they are deliberately NOT presented as measurements.
BALL_SWING_MIN = 0.15        # rad: the moving axis must clearly move
BALL_QUIET_MAX = 0.08        # rad: the two out-of-plane axes must stay quiet
BALL_R0 = 0.42720            # m: sqrt(0.15^2 + 0.4^2), the joint radius
BALL_R_TOL = 0.02            # m: 4.7% of the radius -- soft constraints sag, they do not stretch
BALL_Z_TRAVEL_MIN = 0.02     # m: the bob must visibly move

#: Servo convergence tolerance. Newton drives these axes with ke = maxTorque*10
#: and a zero-velocity damping actuator, so the steady-state error at ~zero load
#: is far below this; 0.05 rad (2.9 deg) is a "did it arrive" bar, not a fidelity
#: bar, and it is what distinguishes arrival from the limp/frozen failure.
SERVO_TOL = 0.05
#: How far axis 2 must travel between phase A (-0.3) and phase B (+0.3) for
#: "axis 2 actually moved" to be more than a rounding artefact.
AXIS2_TRAVEL_MIN = 0.4

_BRINGUP = ("can't initialize sys standard streams",
            "the Newton runtime is INSTALLED but did not come up",
            "Refusing to run it on ODE")


def _rig(mode, body):
    """The shared rig: a heavy 1 x 1 x 0.1 m slab resting on the floor, carrying
    the joint under test 1 m up. The rig is the Robot ITSELF (not a sibling
    supervisor) because a joint's motors and position sensors are only reachable
    from the controller of the Robot that owns them. It has a Physics node on
    purpose: OmHinge2Joint::setJoint refuses a parent with no Physics ("can only
    connect Solid nodes that have a Physics node"), so a static base would not
    even build the ODE oracle arm."""
    return """#VRML_SIM R2025a utf8

# GENERATED AT TEST TIME by tests/test_newton_ball_hinge2.py -- do not commit.

WorldInfo {
  basicTimeStep 4
  newtonStatics TRUE
  coordinateSystem "ENU"
}
Viewpoint {
  position -3 -2 1.5
}
Background {
  skyColor [ 0.15 0.2 0.3 ]
}
DEF FLOOR Solid {
  translation 0 0 -0.05
  name "floor"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.4 0.4 0.45 roughness 1 metalness 0 }
      geometry Box { size 6 6 0.1 }
    }
  ]
  boundingObject Box { size 6 6 0.1 }
}
DEF RIG Robot {
  translation 0 0 0.05
  name "rig"
  controller "ball_hinge2_probe"
  controllerArgs [ "--mode" "%(MODE)s" ]
  supervisor TRUE
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.3 0.32 0.38 roughness 1 metalness 0 }
      geometry Box { size 1 1 0.1 }
    }
%(BODY)s
  ]
  boundingObject Box { size 1 1 0.1 }
  physics Physics {
    density -1
    mass 200
  }
}
""" % {"MODE": mode, "BODY": body}


#: 3-DoF spherical joint, PASSIVE. The default axis triad applies (no axis is
#: authored): axis = +x, axis3 = +z, axis2 = axis3 x axis = +y. The bob is
#: offset in +x and hangs below, so the swing is a rotation about +y == axis 2.
BALL_BODY = """    DEF BALL BallJoint {
      jointParameters BallJointParameters {
        anchor 0 0 1
      }
      device [
        PositionSensor { name "ps1" }
      ]
      device2 [
        PositionSensor { name "ps2" }
      ]
      device3 [
        PositionSensor { name "ps3" }
      ]
      endPoint DEF BOB Solid {
        translation 0.15 0 0.6
        children [
          Shape {
            appearance PBRAppearance { baseColor 0.85 0.3 0.25 roughness 1 metalness 0 }
            geometry Box { size 0.1 0.1 0.1 }
          }
        ]
        name "bob"
        boundingObject Box { size 0.1 0.1 0.1 }
        physics Physics {
          density -1
          mass 1
        }
      }
    }"""

#: Universal joint, BOTH axes motorised: axis 1 = +z (yaw), axis 2 = +x (roll).
#: The endpoint's COM sits exactly on the anchor so gravity torque about both
#: axes is ~0 -- this measures the actuator and the readback, not the load.
#: Both motors declare min/maxPosition, which is what classifies them as
#: position servos rather than velocity wheels (see OmBasicJoint::newtonAxisSpec),
#: and they stay strictly inside the joint's own minStop/maxStop so the engine's
#: consistency check has nothing to warn about.
HINGE2_BODY = """    DEF H2 Hinge2Joint {
      jointParameters HingeJointParameters {
        anchor 0 0 1
        axis 0 0 1
        minStop -1.5
        maxStop 1.5
      }
      jointParameters2 JointParameters {
        axis 1 0 0
        minStop -1.5
        maxStop 1.5
      }
      device [
        RotationalMotor {
          name "m1"
          maxVelocity 6
          maxTorque 60
          minPosition -1.4
          maxPosition 1.4
        }
        PositionSensor { name "ps1" }
      ]
      device2 [
        RotationalMotor {
          name "m2"
          maxVelocity 6
          maxTorque 60
          minPosition -1.4
          maxPosition 1.4
        }
        PositionSensor { name "ps2" }
      ]
      endPoint DEF ARM Solid {
        translation 0 0 1
        children [
          Shape {
            appearance PBRAppearance { baseColor 0.9 0.7 0.2 roughness 1 metalness 0 }
            geometry Box { size 0.3 0.08 0.08 }
          }
        ]
        name "arm"
        boundingObject Box { size 0.3 0.08 0.08 }
        physics Physics {
          density -1
          mass 0.5
        }
      }
    }"""

_BODIES = {"ball": BALL_BODY, "hinge2": HINGE2_BODY}


def _binary():
    for c in (REPO / "msys64/mingw64/bin/omnisim-bin.exe", REPO / "bin/omnisim-bin"):
        if c.exists():
            return c
    return None


pytestmark = pytest.mark.skipif(_binary() is None,
                                reason="no omnisim-bin in this clone")


def _run(arm, mode, tmp_path, gate="1"):
    """arm: 'newton'. gate: value for OMNISIM_NEWTON_BALL_HINGE2, or None to
    leave it UNSET so the run measures the shipped DEFAULT.

    -> probe dict, or None on a Newton bring-up flake."""
    world = WORLDS / (".ball_hinge2_%s_%s.wbt" % (mode, gate if gate is not None else "default"))
    world.write_text(_rig(mode, _BODIES[mode]), encoding="utf-8")
    tag = "%s_%s" % (arm, gate if gate is not None else "default")
    out = tmp_path / ("probe_%s_%s.json" % (mode, tag))
    log = tmp_path / ("engine_%s_%s.log" % (mode, tag))
    env = dict(os.environ)
    env["OMNISIM_HOME"] = str(REPO)
    env["OMNISIM_LOG_PATH"] = str(log)
    env["OMNISIM_BH2_PROBE_OUT"] = str(out)
    # Start from a clean slate: an inherited OMNISIM_NEWTON_* from the caller's
    # shell would silently change which physics this measures.
    for k in list(env):
        if k.startswith("OMNISIM_NEWTON_") or k in ("OMNISIM_FORCE_ODE", "OMNISIM_LEGACY"):
            env.pop(k)
    # The `arm == "ode"` branch here set OMNISIM_FORCE_ODE=1. It is DELETED (not
    # just unused): src/ode is gone (bdc02139), so it would select a backend with
    # no implementation and the probe would record a frozen bob -- read as "the
    # ball joint does not swing" rather than "there is no backend". Refuse loudly
    # instead of leaving a trap for whoever re-adds a second arm.
    if arm != "newton":
        raise AssertionError(
            "arm=%r: the ODE oracle arm was removed with src/ode (commit "
            "bdc02139). Only 'newton' exists. See the note above the tests." % arm)
    if gate is not None:
        env["OMNISIM_NEWTON_BALL_HINGE2"] = gate
    proc = subprocess.Popen(
        [str(_binary()), str(world), "--batch", "--mode=fast", "--no-rendering",
         "--minimize", "--stdout", "--stderr"],
        cwd=str(REPO), env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(240):          # the probe runs ~3-8 s of sim time; poll
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
        pytest.fail("%s/%s arm produced no probe output\n%s" % (mode, tag, blob[-1500:]))
    return json.loads(out.read_text())


# --------------------------------------------------------------------------- #
# BallJoint                                                                   #
# --------------------------------------------------------------------------- #

def _check_ball(arm, d, problems):
    peak = d["peak_abs"]
    if peak[1] < BALL_SWING_MIN:
        problems.append(
            "%s: the swing axis (axis 2) peaked at only %.4f rad (< %g). A bob released 0.359 rad "
            "off vertical must swing: this reads as a frozen joint sensor, i.e. the position "
            "readback never reached the solver that is actually moving the body. axes=%r"
            % (arm, peak[1], BALL_SWING_MIN, peak))
    for i in (0, 2):
        if peak[i] > BALL_QUIET_MAX:
            problems.append(
                "%s: axis %d peaked at %.4f rad (> %g) on a planar x-z swing. Either the axis triad "
                "is not (x, y, z) or the three angles are cross-talking -- a Euler decomposition "
                "about the wrong basis smears one rotation across all three. axes=%r"
                % (arm, i + 1, peak[i], BALL_QUIET_MAX, peak))
    if d["r_min"] is None:
        problems.append("%s: no radius samples at all" % arm)
        return
    if abs(d["r_max"] - BALL_R0) > BALL_R_TOL or abs(d["r_min"] - BALL_R0) > BALL_R_TOL:
        problems.append(
            "%s: bob-to-anchor distance ranged [%.4f, %.4f] m, expected %.4f +/- %g. The joint is "
            "not holding the bob at a fixed radius -- an unconstrained bob free-falls and this runs "
            "away, a mis-anchored one settles at the wrong radius."
            % (arm, d["r_min"], d["r_max"], BALL_R0, BALL_R_TOL))
    if (d["bob_z_max"] - d["bob_z_min"]) < BALL_Z_TRAVEL_MIN:
        problems.append(
            "%s: the bob's z moved only %.4f m (< %g) -- it is not swinging at all, so the sensor "
            "numbers above describe nothing."
            % (arm, d["bob_z_max"] - d["bob_z_min"], BALL_Z_TRAVEL_MIN))


# THE ODE ORACLE ARMS ARE GONE. This file used to run OMNISIM_FORCE_ODE=1 beside
# the Newton arm for three tests -- test_ball_joint_ode_oracle_agrees,
# test_hinge2_ode_oracle_agrees and test_newton_and_ode_agree_on_sign -- so a
# Newton reading could be checked against ODE's and, crucially, so a fixture bug
# could be told apart from a backend bug. src/ode has been DELETED, that arm
# cannot run, and those three tests are removed rather than left to fail
# meaninglessly. Nothing replaces the cross-check: no ODE numbers for these
# joints were ever frozen into tests/goldens/ode_oracle_goldens.json, because
# the motorised path never worked well enough to be worth freezing. The
# remaining tests judge Newton against ANALYTIC geometry only, which is a weaker
# claim about the ball joint than this file used to make -- state that plainly
# rather than let the green count imply otherwise.

def test_ball_joint_swings_and_reports_angles(tmp_path):
    newton = _run("newton", "ball", tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    problems = []
    _check_ball("Newton", newton, problems)
    assert not problems, (
        "Newton BallJoint motion/readback failed:\n  " + "\n  ".join(problems) +
        "\n(probe: %r)\nNewton registers a BallJoint as a native JointType.BALL whose joint FRAME "
        "is the authored (axis, axis2, axis3) triad; the readback decomposes joint_q's quaternion "
        "as Rx*Ry*Rz in that frame -- see OmBallJoint::postPhysicsStep."
        % (newton,))


def _check_hinge2(arm, d, problems, signed=True):
    def err(got, want):
        return abs(got - want) if signed else abs(abs(got) - abs(want))

    for phase in ("a", "b", "c"):
        got = d[phase]
        want = d["target_" + phase]
        for i in (0, 1):
            if err(got[i], want[i]) > SERVO_TOL:
                problems.append(
                    "%s phase %s: axis %d reached %+.4f rad, commanded %+.4f (error %.4f > %g). "
                    "A composed universal joint fails here in two ways: the axis is not actuated "
                    "at all (reads ~0 forever), or its target lands on the WRONG degree of "
                    "freedom of the joint slot. phase=%r targets=%r"
                    % (arm, phase.upper(), i + 1, got[i], want[i],
                       err(got[i], want[i]), SERVO_TOL, got, want))
    # Independence: phase B re-commands ONLY axis 2. Axis 1 must not follow.
    drift = abs(d["b"][0] - d["a"][0])
    if drift > SERVO_TOL:
        problems.append(
            "%s: axis 1 drifted %.4f rad (> %g) when only axis 2 was re-commanded (%+.4f -> %+.4f). "
            "The two axes are COUPLED: the second hinge's target is bleeding into the first, which "
            "is what a wrong DOF offset or a shared actuator looks like."
            % (arm, drift, SERVO_TOL, d["a"][1], d["b"][1]))
    travel = abs(d["b"][1] - d["a"][1])
    if travel < AXIS2_TRAVEL_MIN:
        problems.append(
            "%s: axis 2 travelled only %.4f rad (< %g) between its -0.3 and +0.3 commands -- it is "
            "silently frozen. This is THE failure mode of a two-hinge composition: axis 1 works, "
            "axis 2 is a constraint with no drive."
            % (arm, travel, AXIS2_TRAVEL_MIN))


# THIS TEST WAS xfail(strict=False) UNTIL 2026-08-17, AND THE REASON IT NO LONGER
# IS, IS AN UPSTREAM UPGRADE RATHER THAN AN ENGINE FIX. The marker recorded that
# motorised hinge2 did not actuate -- axis 1 read 0.0000 rad when commanded 0.4 --
# and root-caused it BELOW us, in newton's d6 -> MuJoCo actuator mapping for
# multi-DoF position control: OMNISIM_NEWTON_JOINTDIAG showed the d6 built and
# mapped with mode=3 (POSITION_VELOCITY), per-axis gains (600,30,600,30) and
# limits (-1.4,1.4,-1.4,1.4), and the per-tick write landing at
# joint_target_pos[qd_start + dof]. That attribution was made against the
# then-vendored newton 1.2.0, whose own test suite never drove d6 POSITION targets
# through SolverMuJoCo at all.
#
# The runtime was upgraded to newton 1.5.0 in b56be84a0 (2026-08-09) and the
# mapping works. Re-measured on the current binary: XPASS. So the marker is gone
# and OMNISIM_NEWTON_BALL_HINGE2 now DEFAULTS ON -- which is what the two tests
# below actually pin, one from each side.

def test_hinge2_both_axes_reach_targets_independently(tmp_path):
    newton = _run("newton", "hinge2", tmp_path)
    if newton is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    problems = []
    _check_hinge2("Newton", newton, problems)
    assert not problems, (
        "Newton Hinge2Joint actuation failed:\n  " + "\n  ".join(problems) +
        "\n(probe: %r)\nNewton registers a Hinge2Joint as a native d6 with two angular axes, which "
        "the MuJoCo conversion emits as two independently actuated hinge elements on the child "
        "body, composed in declaration order -- so axis 2 co-rotates with axis 1 exactly as ODE's "
        "Hinge2 does, with no intermediate body." % (newton,))


def test_hinge2_actuates_with_the_gate_UNSET(tmp_path):
    """The flip itself: no env var at all must actuate.

    The arm above passes OMNISIM_NEWTON_BALL_HINGE2=1 explicitly, so it would
    stay green if the default silently reverted to OFF. This one leaves the
    variable unset, which is what every shipped world (motor2 / motor3 / muscle /
    gyro) actually runs with.
    """
    newton = _run("newton", "hinge2", tmp_path, gate=None)
    if newton is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    problems = []
    _check_hinge2("Newton (gate unset)", newton, problems)
    assert not problems, (
        "Hinge2Joint does not actuate with OMNISIM_NEWTON_BALL_HINGE2 UNSET, i.e. the gate's "
        "default has gone back to OFF:\n  " + "\n  ".join(problems) + "\n(probe: %r)" % (newton,))


def test_gate_off_reproduces_the_passive_pre_flip_behaviour(tmp_path):
    """OMNISIM_NEWTON_BALL_HINGE2=0 must put the PASSIVE constraint back, exactly.

    This is the half that makes the two arms above evidence rather than
    assertion: the same rig, one env var apart, reproduces the frozen joint the
    xfail marker described. It also keeps the documented exact-revert hatch
    honest -- a revert switch that has quietly stopped reverting is worse than
    none.
    """
    newton = _run("newton", "hinge2", tmp_path, gate="0")
    if newton is None:
        pytest.skip("Newton bring-up flake -- no data, re-run")
    problems = []
    _check_hinge2("Newton (gate off)", newton, problems)
    assert problems, (
        "OMNISIM_NEWTON_BALL_HINGE2=0 actuated the joint. That hatch is documented as the exact "
        "revert to the PASSIVE constraint that shipped before 2026-08-17, and a passive joint "
        "cannot reach a commanded angle. probe=%r" % (newton,))


def _sign(v):
    return 0 if abs(v) < 1e-6 else (1 if v > 0 else -1)

