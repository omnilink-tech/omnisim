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

"""worldgen.py -- the OmniSim arm's five worlds, emitted from ``rungs.py``.

Every geometric and physical number is READ from the shared contract
(``tests/benchmarks/ladder0/rungs.py``); nothing is written twice.  A scene
change there changes the world file here and the analytic expectation together,
so the two cannot drift.

Regenerate with::

    python tests/benchmarks/ladder0/run_ladder.py --regen-worlds

The worlds are committed so a reader can diff this arm's description of the
scene against the other arms' descriptions of the same scene.

FRICTION IS DECLARED IN THE FIELD NEWTON ACTUALLY READS.
``WorldInfo.contactProperties`` / ``coulombFriction`` was the ODE-path
declaration and is **not read** by Newton -- a world that declares only that
runs at the default mu with no error and no warning about the value it thought
it had asked for.  ``newtonGroundMu`` is the field that sets contact friction,
so that is what these worlds declare.  (This is also the one intended
difference between this arm's ``.wbt`` and the upstream-Webots arm's: same
physics, different spelling, because the two engines read different fields.)

FAULT VARIANTS
--------------
``--self-test`` needs the ladder to be shown going RED end to end, not merely
in its comparison operators, so this module also emits deliberately broken
copies of the scenes.  They live beside the real worlds under the reserved
``_fault_`` prefix and are never run by a normal campaign.  Each reproduces a
defect this repo has actually shipped -- see ``FAULTS`` below.
"""

from __future__ import annotations

import importlib.util
import os
import sys

HERE = os.path.abspath(os.path.dirname(__file__))
LADDER0 = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(LADDER0, os.pardir, os.pardir, os.pardir))
if LADDER0 not in sys.path:
    sys.path.insert(0, LADDER0)

import rungs                                     # noqa: E402


def _load_sibling(stem):
    """This arm's own modules, BY PATH under an arm-qualified name.  See
    ``arm.py`` for why a bare sibling import is not allowed here."""
    key = "ladder0_omnisim_%s" % stem
    if key in sys.modules:
        return sys.modules[key]
    sp = importlib.util.spec_from_file_location(
        key, os.path.join(HERE, "%s.py" % stem))
    mod = importlib.util.module_from_spec(sp)
    sys.modules[key] = mod
    sp.loader.exec_module(mod)
    return mod


engine_facts = _load_sibling("engine_facts")
RUNG8_FINGER_MAXFORCE = engine_facts.RUNG8_FINGER_MAXFORCE

WORLDS = os.path.join(HERE, "worlds")
FAULT_PREFIX = "_fault_"

# fault -> (what it breaks, where it is injected)
FAULTS = {
    "none": ("the honest scene", "-"),
    "no_floor": ("the floor Solid loses its boundingObject, so it is a "
                 "hologram and the box free-falls through it -- the exact "
                 "shape of AgentBench C2, which a log-only headless PASS "
                 "reported byte-identically to the fixed world", "world"),
    "half_gravity": ("WorldInfo.gravity is halved, so the fall is slower "
                     "than sqrt(2h/g) -- the shape of the defect where "
                     "gravity was never plumbed into Newton at all", "world"),
    "short_run": ("the controller stops stepping half way -- the shape of a "
                  "stale-libController hang, which exits 0 and prints PASS",
                  "controller"),
    "ignore_zero": ("the motor keeps its previous command when told to stop "
                    "-- the shape of setVelocity being silently ignored",
                    "controller"),
    "slide": ("the chassis is translated at exactly the right speed with its "
              "wheels commanded to zero -- the shape of the bug that let 33 "
              "worlds ship with wheels that never turned", "controller"),
    "no_sweep": ("the sensor carrier is never moved, so the reading is "
                 "correct and constant -- 'the scene is frozen' must not read "
                 "as 'the sensor is dead'", "controller"),
    "wall_shifted": ("the wall is authored RUNG5_FAULT_SHIFT further away "
                     "while the sweep is untouched -- the shape of a sensor "
                     "whose origin, scale or lookup table is wrong", "world"),
    "no_stop": ("the rover reads the sensor correctly and never acts on it -- "
                "the shape of a controller that polls and does nothing, which "
                "must NOT read as a broken sensor", "controller"),
    "bounce": ("the rover runs RUNG6_FAULT_BOUNCE_M past the threshold into "
               "the wall and is then put back at exactly the gap the check "
               "expects -- rung 6's 'slide': the final state is right and "
               "only the whole-run min_gap can see it", "controller"),
    "stalled_robot": ("one robot of the five is commanded zero while the rest "
                      "run -- a fleet average would absorb it", "world"),
    "lane_offset": ("one robot of the five is SPAWNED half a lane out of "
                    "place, with its own physics untouched", "world"),
    "no_grip": ("the fingers never close.  THE CAUSAL CONTROL: the identical "
                "scene must leave the payload on the table, which is what "
                "lets rung 8 claim the fingers did the lifting", "controller"),
    "no_traverse": ("the traverse is never commanded, so the payload is "
                    "lifted correctly and left in the wrong place",
                    "controller"),
    "drop_mid_carry": ("the fingers reopen at the top of the lift and the "
                       "payload falls -- the red proof for part_speed_max",
                       "controller"),
    "seed_nudge": ("replica b's dropped cube is spawned RUNG9_FAULT_NUDGE "
                   "(1 pm) out of place while a and c are untouched -- a "
                   "perturbation nine orders below anything physical, which a "
                   "bitwise-reproducible engine still cannot absorb", "world"),
    "frozen": ("rung 9's dropped cube loses its physics, so the pile never "
               "moves.  A FROZEN WORLD IS PERFECTLY DETERMINISTIC: this is "
               "the fault that proves repeat_delta alone is not evidence",
               "world"),
    "short_b": ("rung 9's replica b stops stepping half way, so the two "
                "replicas agree over the overlap and differ in length -- how "
                "a truncated replica passes a determinism check", "controller"),
    "starve_budget": ("rung 11's fleet is spawned RUNG11_CLEAR_M above the "
                      "floor, so nefc at t=0 is ~0 and newton's njmax FLOOR "
                      "leaves the 256 default in place while the settled "
                      "fleet needs 32 N.  A GPU-path variant study, not a "
                      "ladder-row fault -- see variants.py", "world"),
}

HEADER = """#VRML_SIM R2025a utf8

# GENERATED by tests/benchmarks/ladder0/omnisim/worldgen.py -- do not hand-edit.
# Ladder0 rung %d: %s
# Arm: omnisim.  Fault: %s.
# Every number in this file comes from tests/benchmarks/ladder0/rungs.py.
"""

VIEWPOINT = """Viewpoint {
  position -3.5 -3.5 2.5
  orientation -0.28 0.30 0.91 1.75
}
Background {
  skyColor [ 0.15 0.17 0.22 ]
}
DirectionalLight {
  direction 0.4 0.6 -1
  intensity 2
}
"""


def _n(v):
    """Number -> shortest exact-looking VRML literal."""
    if isinstance(v, int) or float(v).is_integer():
        return str(int(v))
    return ("%.6f" % float(v)).rstrip("0").rstrip(".")


def _hi(v):
    """Number -> a literal that ROUND-TRIPS the float64 exactly.

    ⚠ Rung 9 needs this and ``_n`` would silently destroy it.  ``_n`` formats
    with ``%.6f``, so the 1 um sensitivity seed writes as ``0.000001`` (which
    survives) but the 1 pm ``seed_nudge`` fault writes as **"0"** -- the fault
    would simply not happen, the run would be honest, ``repeat_delta`` would
    come back green, and the self-test would report a fault that did not go
    red as a defect in the ladder.  ``%.17g`` is the shortest width that
    round-trips every float64.
    """
    return "%.17g" % float(v)


def _worldinfo(rung, fault):
    """The world block.

    ONLY what the contract declares reaches this node: gravity, the timestep,
    the friction that rung needs -- and, on rung 8 alone, the friction-MODEL
    declaration CONTRACT.md 3b's R2 admits.  Nothing here is tuned.

    ``newtonSolver "mujoco"`` and ``newtonStatics TRUE`` are declarations of
    WHICH engine is under test rather than tuning of it: the former names the
    solver (and is now also the engine's own default), the latter makes a
    static floor collide at all.

    RUNG 8 AND THE FAIR-DEFAULTS RULE.  Rung 8 declares a friction model, and
    each arm spells that model in whatever its engine requires -- the same
    split the contract already makes for the grip force, where it owns the
    newtons and the arm owns the actuator.  What this arm declares, why each
    entry is model accuracy rather than tuning, the engine default it departs
    from and the sweep that shows it is a converged budget all live in ONE
    place, ``engine_facts.RUNG8_DECLARATIONS``, which is also what the probe
    writes into the samples so a row carries its own declarations (R4).

    What is deliberately NOT here: ``newtonContactKe`` / ``newtonContactKd``
    (contact STIFFNESS is not friction accuracy, and raising ke while holding
    the interference fixed silently raises the grip force too -- MEASURED, it
    ejects the payload), ``newtonIterations`` / ``newtonLsIterations`` (the
    declared configuration converges without them), ``newtonCondim`` (adding
    torsional friction adds a mechanism the contract never declared, which R2
    excludes by name), and ``newtonNoslipIterations`` (admissible, and
    measured to change carry_rel by 0.09 mm -- see RUNG8_NOT_DECLARED).
    """
    g = rungs.G / 2.0 if fault == "half_gravity" else rungs.G
    mu = rungs.RUNG8_MU if rung == 8 else rungs.MU
    declared = ("".join("  %s %s\n" % (k, v[0])
                        for k, v in engine_facts.RUNG8_DECLARATIONS.items())
                if rung == 8 else "")
    if rung == 11:
        # CONTRACT.md 3b R4 + section 3E.  A constraint budget is DECLARED, and
        # it is declared GENEROUS: sizing it at a scene's own measured peak
        # moved results 8.81 m versus every other size with a 1.71 m run-to-run
        # spread, while 512 / 2048 / 4096 agreed to 1e-4.  4096 is 4x what the
        # largest fleet in this family needs, and it is the HONEST scene's
        # budget -- the ``starve_budget`` variant is what takes it away.
        declared += ("  newtonNjmax %d\n  newtonNconmax %d\n"
                     % (rungs.RUNG11_NJMAX, rungs.RUNG11_NCONMAX))
    return """WorldInfo {
  gravity %(g)s
  basicTimeStep %(dt)s
  FPS 30
  randomSeed 42
  coordinateSystem "ENU"
  newtonSolver "mujoco"
  newtonStatics TRUE
  newtonGroundMu %(mu)s
%(declared)s}
""" % {"g": _n(g), "dt": _n(rungs.DT * 1000.0), "mu": _n(mu),
       "declared": declared}


def _probe(rung, fault, children="", extra_fields="", args_extra=()):
    args = ['    "--rung=%d"' % rung]
    if fault != "none":
        args.append('    "--fault=%s"' % fault)
    args += ['    "%s"' % a for a in args_extra]
    return """DEF PROBE Robot {
  name "ladder0_probe"
  controller "ladder0_probe"
  supervisor TRUE
  controllerArgs [
%s
  ]
%s%s}
""" % ("\n".join(args), children, extra_fields)


def _floor(fault, size=None):
    sx, sy, sz = (size or rungs.FLOOR_SIZE)
    # The fault removes the boundingObject and NOTHING else: the floor still
    # renders, still appears in the scene tree, still has a plausible pose.
    # That is precisely why the bug it reproduces was invisible.
    bounding = ("" if fault == "no_floor"
                else "  boundingObject USE FLOOR_SHAPE\n")
    return """DEF FLOOR Solid {
  translation 0 0 %(cz)s
  name "floor"
  children [
    DEF FLOOR_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.45 0.47 0.5 roughness 1 metalness 0 }
      geometry Box { size %(sx)s %(sy)s %(sz)s }
    }
  ]
%(bo)s}
""" % {"cz": _n(rungs.FLOOR_CENTER_Z), "sx": _n(sx), "sy": _n(sy),
       "sz": _n(sz), "bo": bounding}


def _box(z):
    e = rungs.BOX_EDGE
    return """DEF BOX Solid {
  translation 0 0 %(z)s
  name "box"
  children [
    DEF BOX_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.85 0.55 0.2 roughness 0.9 metalness 0 }
      geometry Box { size %(e)s %(e)s %(e)s }
    }
  ]
  boundingObject USE BOX_SHAPE
  physics Physics {
    density -1
    mass %(m)s
  }
}
""" % {"z": _n(z), "e": _n(e), "m": _n(rungs.BOX_MASS)}


def _arm_children():
    """Rung 3: one hinge about a VERTICAL axis, no floor, nothing to touch."""
    return """  translation 0 0 %(hz)s
  children [
    DEF BASE_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.3 0.3 0.35 roughness 1 metalness 0 }
      geometry Box { size 0.1 0.1 0.1 }
    }
    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 0 1
        anchor 0 0 0
      }
      device [
        RotationalMotor {
          name "j0"
          maxVelocity 30
          maxTorque 100
        }
        PositionSensor {
          name "j0_sensor"
        }
      ]
      endPoint DEF LINK Solid {
        translation %(half)s 0 0.08
        name "link"
        children [
          DEF LINK_SHAPE Shape {
            appearance PBRAppearance { baseColor 0.2 0.7 0.9 roughness 0.8 metalness 0 }
            geometry Box { size %(len)s %(w)s %(w)s }
          }
        ]
        boundingObject USE LINK_SHAPE
        physics Physics {
          density -1
          mass %(m)s
        }
      }
    }
  ]
""" % {"hz": _n(rungs.RUNG3_HINGE_Z), "half": _n(rungs.RUNG3_LINK_LEN / 2.0),
       "len": _n(rungs.RUNG3_LINK_LEN), "w": _n(2 * rungs.RUNG3_LINK_RADIUS),
       "m": _n(rungs.RUNG3_LINK_MASS)}


def _wheel(tag, sx, sy, def_prefix=""):
    """One driven wheel.

    ⚠ THE `Pose { rotation 1 0 0 -pi/2 }` AROUND THE CYLINDER IS LOAD-BEARING,
    and this arm shipped without it. A VRML/`.wbt` `Cylinder` is **Z-axial** --
    `OmCylinder::scaledHeight()` reads `absoluteScale().z()`, its picking code
    puts the end caps at +-z, and the retired ODE path fed those straight to
    `dCreateCylinder` (also Z-axial). So a bare `Cylinder` at a wheel station is
    a *flat disc lying on its face*, not a wheel; the upstream-Webots arm rotates
    it -90 deg about X for exactly this reason and this arm did not.

    It read as correct anyway because OmniSim's Newton shape path
    (`World.add_shape_cylinder`) DISCARDS a primitive's rotation -- it takes a
    translation and no orientation -- and hardcodes a -90 deg X rotation of its
    own, so every `Cylinder` collider is silently turned into a wheel. The
    INERTIA composer does honour the rotation (`OmSolidUtilities::addInertia`
    rotates on the `Pose` branch), so an unrotated cylinder got a wheel-shaped
    COLLIDER carrying a flat disc's TENSOR: measured, the hinge saw
    I = 0.0013542 kg m^2 where a 0.5 kg r=0.1 m wheel has 0.0025 about its own
    axis -- 54%. Declaring the rotation makes shape and tensor agree here
    whichever way that engine defect is eventually resolved.
    """
    x = sx * rungs.WHEEL_X
    y = sy * rungs.WHEEL_Y
    # ``def_prefix`` exists for rung 7 only: five rovers in one world need five
    # distinct DEF namespaces, and the supervisor addresses each wheel by DEF
    # because a Webots-family supervisor may not read a sibling robot's
    # devices.  It is empty everywhere else, so rungs 4 and 6 keep the exact
    # DEF names they had.
    return """    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor %(x)s %(y)s 0
      }
      device [
        RotationalMotor {
          name "wheel_%(tag)s"
          maxVelocity 30
          maxTorque 20
        }
        PositionSensor {
          name "wheel_%(tag)s_sensor"
        }
      ]
      endPoint DEF WHEEL_%(TAG)s Solid {
        translation %(x)s %(y)s 0
        name "wheel_%(tag)s"
        children [
          DEF WHEEL_%(TAG)s_POSE Pose {
            rotation 1 0 0 -1.570796
            children [
              Shape {
                appearance PBRAppearance { baseColor 0.12 0.12 0.14 roughness 1 metalness 0 }
                geometry Cylinder {
                  height %(w)s
                  radius %(r)s
                  subdivision 24
                }
              }
            ]
          }
        ]
        boundingObject USE WHEEL_%(TAG)s_POSE
        physics Physics {
          density -1
          mass %(m)s
        }
      }
    }
""" % {"tag": tag, "TAG": def_prefix + tag.upper(), "x": _n(x), "y": _n(y),
       "w": _n(rungs.WHEEL_W), "r": _n(rungs.WHEEL_R),
       "m": _n(rungs.WHEEL_MASS)}


def _rover_children():
    cx, cy, cz = rungs.CHASSIS_SIZE
    head = """  translation 0 0 %(z)s
  children [
    DEF CHASSIS_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.85 0.7 0.2 roughness 0.8 metalness 0 }
      geometry Box { size %(cx)s %(cy)s %(cz)s }
    }
""" % {"z": _n(rungs.ROBOT_Z), "cx": _n(cx), "cy": _n(cy), "cz": _n(cz)}
    return head + "".join(_wheel(t, sx, sy)
                          for t, sx, sy in rungs.WHEELS) + "  ]\n"


def _rover_fields():
    return """  boundingObject USE CHASSIS_SHAPE
  physics Physics {
    density -1
    mass %s
  }
""" % _n(rungs.CHASSIS_MASS)


# --------------------------------------------------------------------------
# Rung 5 / 6 -- the wall and the range sensor
# --------------------------------------------------------------------------

def _wall(rung, fault):
    """A static slab whose NEAR face is the thing every range is measured to.

    ``wall_shifted`` moves it RUNG5_FAULT_SHIFT further away and changes
    nothing else: the sweep is untouched, so ``sweep_span`` stays green while
    every reading is wrong by exactly the shift.  That is the seam -- a sensor
    fault and a motion fault must not be able to masquerade as each other.
    """
    if rung == 5:
        sx, sy, sz = rungs.RUNG5_WALL_SIZE
        cx, cz = rungs.RUNG5_WALL_CENTER_X, rungs.RUNG5_WALL_CENTER_Z
    else:
        sx, sy, sz = rungs.RUNG6_WALL_SIZE
        cx, cz = rungs.RUNG6_WALL_CENTER_X, rungs.RUNG6_WALL_CENTER_Z
    if fault == "wall_shifted":
        cx += rungs.RUNG5_FAULT_SHIFT
    return """DEF WALL Solid {
  translation %(cx)s 0 %(cz)s
  name "wall"
  children [
    DEF WALL_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.6 0.3 0.28 roughness 1 metalness 0 }
      geometry Box { size %(sx)s %(sy)s %(sz)s }
    }
  ]
  boundingObject USE WALL_SHAPE
}
""" % {"cx": _n(cx), "cz": _n(cz), "sx": _n(sx), "sy": _n(sy), "sz": _n(sz)}


def _range_sensor(dx, name="ds"):
    """A forward-facing single-ray sensor whose value IS the range in metres.

    The lookup table is the identity over [0, RUNG5_MAX_RANGE].  The default
    table (``0 0 0, 0.1 1000 0``) returns a unitless 0..1000 that saturates at
    10 cm, so a sensor left on the default reads 1000 for every distance in
    this scene -- a constant, and one that no amount of geometry explains.
    ``resolution -1`` turns off quantisation; ``noise 0`` is the default and is
    written out so the file is self-describing.
    """
    return """    DistanceSensor {
      translation %(dx)s 0 0
      name "%(name)s"
      type "generic"
      numberOfRays 1
      resolution -1
      lookupTable [ 0 0 0, %(mx)s %(mx)s 0 ]
    }
""" % {"dx": _n(dx), "name": name, "mx": _n(rungs.RUNG5_MAX_RANGE)}


def _carrier_children():
    """Rung 5's sensor carrier: a body with NO physics and NO boundingObject.

    Deliberately not a physical object.  Its pose is written by the driver, so
    it must not be integrated; and a collider here could occlude the very ray
    it carries, which would turn a sensing rung into a scene-authoring puzzle.
    """
    e = rungs.RUNG5_CARRIER_EDGE
    return """  translation %(x)s 0 %(z)s
  children [
    DEF CARRIER_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.2 0.7 0.9 roughness 0.8 metalness 0 }
      geometry Box { size %(e)s %(e)s %(e)s }
    }
%(ds)s  ]
""" % {"x": _n(rungs.RUNG5_X0), "z": _n(rungs.RUNG5_SENSOR_Z), "e": _n(e),
       "ds": _range_sensor(rungs.RUNG5_SENSOR_DX)}


# --------------------------------------------------------------------------
# Rung 7 -- five rovers, five commands
# --------------------------------------------------------------------------
#
# Each rover is its OWN Robot with its own controller, because a Webots-family
# supervisor may not read another robot's devices -- that restriction is by
# design and is honest (a supervisor cannot truthfully claim to have read a
# sibling's sensor).  The probe therefore reads each wheel's WORLD ORIENTATION
# out of the scene graph instead, which is a pose fact available to any
# supervisor and is arguably the better measurement: it observes the wheel
# rather than the servo's own bookkeeping of it.

def _rover_body(prefix, wheel_prefix):
    """The rung-4 rover's children + fields, with DEF names prefixed."""
    cx, cy, cz = rungs.CHASSIS_SIZE
    head = """  children [
    DEF %(P)s_CHASSIS_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.85 0.7 0.2 roughness 0.8 metalness 0 }
      geometry Box { size %(cx)s %(cy)s %(cz)s }
    }
""" % {"P": prefix, "cx": _n(cx), "cy": _n(cy), "cz": _n(cz)}
    wheels = "".join(_wheel(t, sx, sy, def_prefix=wheel_prefix)
                     for t, sx, sy in rungs.WHEELS)
    fields = """  boundingObject USE %(P)s_CHASSIS_SHAPE
  physics Physics {
    density -1
    mass %(m)s
  }
""" % {"P": prefix, "m": _n(rungs.CHASSIS_MASS)}
    return head + wheels + "  ]\n", fields


def _rung7_robots(fault):
    out = []
    for i, tag in enumerate(rungs.RUNG7_TAGS):
        omega = rungs.RUNG7_OMEGA[i]
        y = rungs.RUNG7_Y[i]
        if i == rungs.RUNG7_FAULT_ROBOT:
            if fault == "stalled_robot":
                omega = 0.0
            elif fault == "lane_offset":
                y += rungs.RUNG7_FAULT_OFFSET_M
        up = tag.upper()
        children, fields = _rover_body(up, up + "_")
        out.append("""DEF %(UP)s Robot {
  translation 0 %(y)s %(z)s
  name "%(tag)s"
  controller "ladder0_rover"
  controllerArgs [
    "--omega=%(w)s"
  ]
%(children)s%(fields)s}
""" % {"UP": up, "tag": tag, "y": _n(y), "z": _n(rungs.ROBOT_Z),
       "w": repr(float(omega)), "children": children, "fields": fields})
    return "".join(out)


# --------------------------------------------------------------------------
# Rung 8 -- table, payload, and a Cartesian gantry gripper
# --------------------------------------------------------------------------

def _table():
    sx, sy, sz = rungs.RUNG8_TABLE_SIZE
    return """DEF TABLE Solid {
  translation 0 0 %(cz)s
  name "table"
  children [
    DEF TABLE_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.5 0.42 0.32 roughness 1 metalness 0 }
      geometry Box { size %(sx)s %(sy)s %(sz)s }
    }
  ]
  boundingObject USE TABLE_SHAPE
}
""" % {"cz": _n(rungs.RUNG8_TABLE_CENTER_Z), "sx": _n(sx), "sy": _n(sy),
       "sz": _n(sz)}


def _part():
    """The payload.  A BOX, not a cylinder: a ``Cylinder`` boundingObject is
    SUBSTITUTED with a capsule on the Newton path, so the collider would not be
    the shape the file draws and the analytic rest height would be wrong for a
    reason that has nothing to do with the grasp."""
    e = rungs.RUNG8_PART_EDGE
    return """DEF PART Solid {
  translation 0 0 %(z)s
  name "part"
  children [
    DEF PART_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.85 0.45 0.15 roughness 0.6 metalness 0 }
      geometry Box { size %(e)s %(e)s %(e)s }
    }
  ]
  boundingObject USE PART_SHAPE
  physics Physics {
    density -1
    mass %(m)s
  }
}
""" % {"z": _n(rungs.RUNG8_PART_Z0), "e": _n(e),
       "m": _n(rungs.RUNG8_PART_MASS)}


def _pad(side, sign):
    px, py, pz = rungs.RUNG8_PAD_SIZE
    return """        SliderJoint {
          jointParameters JointParameters {
            axis 0 1 0
          }
          device [
            LinearMotor {
              name "finger_%(side)s"
              maxForce %(f)s
              maxVelocity 0.1
            }
            PositionSensor {
              name "finger_%(side)s_sensor"
            }
          ]
          endPoint DEF PAD_%(SIDE)s Solid {
            translation 0 %(y)s 0
            name "pad_%(side)s"
            children [
              DEF PAD_%(SIDE)s_SHAPE Shape {
                appearance PBRAppearance { baseColor 0.9 0.9 0.9 roughness 0.4 metalness 0 }
                geometry Box { size %(px)s %(py)s %(pz)s }
              }
            ]
            boundingObject USE PAD_%(SIDE)s_SHAPE
            physics Physics {
              density -1
              mass %(m)s
            }
          }
        }
""" % {"side": side, "SIDE": side.upper(),
       "y": _n(sign * rungs.RUNG8_PAD_OPEN_Y), "px": _n(px), "py": _n(py),
       "pz": _n(pz), "m": _n(rungs.RUNG8_PAD_MASS),
       "f": _n(RUNG8_FINGER_MAXFORCE)}


def _gantry_children():
    """base -> traverse(X) -> carriage -> lift(Z) -> wrist -> two fingers(Y).

    THE WRIST ORIGIN IS AT RUNG8_GRASP_Z, WHICH IS THE PAYLOAD'S CENTRE.  That
    is what makes ``carry_rel`` an analytic zero instead of a reference read
    out of the run, so it is authored here and asserted there.  The wrist's own
    plate is offset RUNG8_WRIST_PLATE_DZ upward so it clears the payload by
    20 mm and cannot become a tray the payload rides on.
    """
    ccx, ccy, ccz = rungs.RUNG8_CARRIAGE_SIZE
    wsx, wsy, wsz = rungs.RUNG8_WRIST_SIZE
    return """  children [
    SliderJoint {
      jointParameters JointParameters {
        axis 1 0 0
      }
      device [
        LinearMotor {
          name "traverse"
          maxForce 200
          maxVelocity 1
        }
        PositionSensor {
          name "traverse_sensor"
        }
      ]
      endPoint DEF CARRIAGE Solid {
        translation 0 0 %(bz)s
        name "carriage"
        children [
          DEF CARRIAGE_SHAPE Shape {
            appearance PBRAppearance { baseColor 0.3 0.32 0.36 roughness 0.7 metalness 0.3 }
            geometry Box { size %(ccx)s %(ccy)s %(ccz)s }
          }
          SliderJoint {
            jointParameters JointParameters {
              axis 0 0 1
            }
            device [
              LinearMotor {
                name "lift"
                maxForce 200
                maxVelocity 1
              }
              PositionSensor {
                name "lift_sensor"
              }
            ]
            endPoint DEF WRIST Solid {
              translation 0 0 %(wdz)s
              name "wrist"
              children [
                DEF WRIST_PLATE Pose {
                  translation 0 0 %(pdz)s
                  children [
                    Shape {
                      appearance PBRAppearance { baseColor 0.25 0.27 0.3 roughness 0.5 metalness 0.4 }
                      geometry Box { size %(wsx)s %(wsy)s %(wsz)s }
                    }
                  ]
                }
%(pads)s              ]
              boundingObject USE WRIST_PLATE
              physics Physics {
                density -1
                mass %(wm)s
              }
            }
          }
        ]
        boundingObject USE CARRIAGE_SHAPE
        physics Physics {
          density -1
          mass %(cm)s
        }
      }
    }
  ]
""" % {"bz": _n(rungs.RUNG8_BASE_Z), "ccx": _n(ccx), "ccy": _n(ccy),
       "ccz": _n(ccz), "wdz": _n(rungs.RUNG8_GRASP_Z - rungs.RUNG8_BASE_Z),
       "pdz": _n(rungs.RUNG8_WRIST_PLATE_DZ), "wsx": _n(wsx), "wsy": _n(wsy),
       "wsz": _n(wsz), "wm": _n(rungs.RUNG8_WRIST_MASS),
       "cm": _n(rungs.RUNG8_CARRIAGE_MASS),
       "pads": _pad("l", +1) + _pad("r", -1)}


# --------------------------------------------------------------------------
# Rung 9 -- the pile, and the cube dropped on its corner
# --------------------------------------------------------------------------

def _rung9_cube(def_name, name, x, y, z, dynamic=True, hi=False):
    """One cube.  ``hi`` writes the pose with an exact float64 literal.

    Only the dropped cube needs ``hi``: it carries the 1 um sensitivity seed
    and the 1 pm fault nudge, and the ordinary ``%.6f`` formatter writes the
    latter as ``0``.  The 25 pile cubes sit on round numbers.
    """
    f = _hi if hi else _n
    e = rungs.BOX_EDGE
    phys = ("" if not dynamic else """  physics Physics {
    density -1
    mass %s
  }
""" % _n(rungs.BOX_MASS))
    return """DEF %(D)s Solid {
  translation %(x)s %(y)s %(z)s
  name "%(name)s"
  children [
    DEF %(D)s_SHAPE Shape {
      appearance PBRAppearance { baseColor %(col)s roughness 0.9 metalness 0 }
      geometry Box { size %(e)s %(e)s %(e)s }
    }
  ]
  boundingObject USE %(D)s_SHAPE
%(phys)s}
""" % {"D": def_name, "name": name, "x": f(x), "y": f(y), "z": f(z),
       "e": _n(e), "phys": phys,
       # Keyed on ``hi`` alone -- i.e. on "is this the dropped cube" -- so the
       # frozen scene differs from the honest one in exactly ONE field, the one
       # the fault removes.  A second difference, even a colour, is a second
       # thing a reader has to rule out.
       "col": "0.9 0.35 0.15" if hi else "0.85 0.55 0.2"}


def _rung9_bodies(eps=0.0, nudge=0.0, frozen=False):
    """25 resting cubes + the 26th, released over the centre cube's corner.

    The dropped cube's x carries ``eps`` (the contract's sensitivity seed) and
    ``nudge`` (the fault).  They are separate arguments and never summed by the
    caller, so a run can never be both the control and the fault by accident.

    ``frozen`` removes the dropped cube's ``physics`` node.  It keeps its
    ``boundingObject``, so it is still a collider -- it simply cannot move, and
    since nothing else in the scene is touching it the whole world is static.
    That is the point: a static world reproduces bitwise, and the rung must
    fail it anyway.
    """
    out = []
    for tag, x, y in rungs.rung9_pile_xy():
        out.append(_rung9_cube("P_" + tag.upper(), tag, x, y,
                               rungs.RUNG9_PILE_Z))
    out.append(_rung9_cube("DROP", "drop",
                           rungs.RUNG9_DROP_XY + eps + nudge,
                           rungs.RUNG9_DROP_XY, rungs.RUNG9_SPAWN_Z,
                           dynamic=not frozen, hi=True))
    return "".join(out)


# --------------------------------------------------------------------------
# Rung 11 -- N rovers in lanes, one controller process each
# --------------------------------------------------------------------------

def _rung11_robots(n, fault="none"):
    """``n`` copies of the rung-4 rover, each with its OWN controller process.

    ONE DRIVER PER ROBOT, declared under R4 because it changes what the row
    measures.  The alternative -- one supervisor commanding every wheel -- is
    not available: a Webots-family supervisor may not write a sibling robot's
    devices, and that restriction is honest rather than a gap.  The cost is
    real and is part of the result: at N = 16 this world starts 16 python
    interpreters, and the largest fleet with LIVE controllers previously
    measured anywhere in this tree is ten.
    """
    out = []
    z0 = rungs.ROBOT_Z + (rungs.RUNG11_CLEAR_M
                          if fault == "starve_budget" else 0.0)
    for i in range(n):
        omega = rungs.rung11_omega(i)
        y = rungs.rung11_y(i, n)
        # ⚠ AT RUNG11_FAULT_N ONLY.  This used to break robot 7 in every fleet
        # that HAD one (N = 8 and N = 16), which changes no verdict -- both
        # checks are maxima over the sweep -- but means the arms' faulted
        # scenes are not the same scene, and a red proof that is not
        # comparable across arms is not much of a proof.  Caught by the MuJoCo
        # arm, which had followed the contract.
        if i == rungs.RUNG11_FAULT_ROBOT and n == rungs.RUNG11_FAULT_N:
            if fault == "stalled_robot":
                omega = 0.0
            elif fault == "lane_offset":
                y += rungs.RUNG11_FAULT_OFFSET_M
        tag = "r%02d" % i
        up = tag.upper()
        children, fields = _rover_body(up, up + "_")
        out.append("""DEF %(UP)s Robot {
  translation 0 %(y)s %(z)s
  name "%(tag)s"
  controller "ladder0_rover"
  controllerArgs [
    "--omega=%(w)s"
  ]
%(children)s%(fields)s}
""" % {"UP": up, "tag": tag, "y": _n(y), "z": _n(z0),
       "w": repr(float(omega)), "children": children, "fields": fields})
    return "".join(out)


def wbt(rung, fault="none", run=None):
    rung = int(rung)
    run = run or {}
    parts = [HEADER % (rung, rungs.RUNG_TITLE[rung], fault),
             _worldinfo(rung, fault), VIEWPOINT]
    if rung == 0:
        parts.append(_probe(0, fault))
    elif rung == 1:
        parts += [_floor(fault), _box(rungs.RUNG1_SPAWN_Z),
                  _probe(1, fault)]
    elif rung == 2:
        parts += [_floor(fault), _box(rungs.RUNG2_SPAWN_Z),
                  _probe(2, fault)]
    elif rung == 3:
        parts.append(_probe(3, fault, children=_arm_children()))
    elif rung == 4:
        parts += [_floor(fault),
                  _probe(4, fault, children=_rover_children(),
                         extra_fields=_rover_fields())]
    elif rung == 5:
        parts += [_floor(fault), _wall(5, fault),
                  _probe(5, fault, children=_carrier_children())]
    elif rung == 6:
        rover = _rover_children()
        # The range sensor rides on the rover's chassis, RUNG6_SENSOR_DX ahead
        # of its origin -- 20 mm proud of the nose, so the ray starts outside
        # the chassis' own boundingObject and cannot see it.
        rover = rover[:-len("  ]\n")] + _range_sensor(rungs.RUNG6_SENSOR_DX) \
            + "  ]\n"
        parts += [_floor(fault), _wall(6, fault),
                  _probe(6, fault, children=rover,
                         extra_fields=_rover_fields())]
    elif rung == 7:
        # The probe is a bodiless observer: no physics, no boundingObject, no
        # geometry.  It must not be a sixth object in a rung whose whole claim
        # is about how five objects do NOT interact.
        parts += [_floor(fault), _rung7_robots(fault), _probe(7, fault)]
    elif rung == 8:
        parts += [_floor(fault), _table(), _part(),
                  _probe(8, fault, children=_gantry_children())]
    elif rung == 9:
        # The probe is a bodiless observer: no physics, no boundingObject, no
        # geometry.  A determinism rung must not add a 27th body to a scene
        # whose whole claim is about how 26 of them reproduce.
        parts += [_floor(fault),
                  _rung9_bodies(eps=float(run.get("eps_m") or 0.0),
                                nudge=float(run.get("nudge_m") or 0.0),
                                frozen=(fault == "frozen")),
                  _probe(9, fault)]
    elif rung == 11:
        n = int(run.get("n") or rungs.RUNG11_N[0])
        # Bodiless observer again -- an (N+1)th object in a rung about how N
        # objects do not interact would be the one thing that could.
        parts += [_floor(fault, size=rungs.rung11_floor_size(n)),
                  _rung11_robots(n, fault),
                  _probe(11, fault, args_extra=("--n=%d" % n,))]
    else:
        raise ValueError("no such rung: %r" % (rung,))
    return "\n".join(p.rstrip("\n") for p in parts) + "\n"


def world_path(rung, fault="none", tag=None):
    """The scene file for one RUN of one cell.

    A single-run rung ignores ``tag`` and keeps exactly the path it had.  A
    multi-run rung (CONTRACT.md amendment A) needs one file per run, and rung 9
    needs its two REPLICAS to be the SAME FILE: ``a`` and ``b`` are not two
    copies of one scene, they are one scene run twice, and a byte difference
    between them -- even in a comment -- would be a difference the rung exists
    to rule out.
    """
    rung = int(rung)
    if rung == 18:
        # Rung 18 has NO scene of its own.  Its world, like its ground truth,
        # belongs to lane1r; authoring a second copy here would be exactly the
        # drift CONTRACT.md section 1 forbids, one level up from a constant.
        return os.path.join(REPO, *(rungs.RUNG18_LANE1R
                                    + ("worlds", "cube_toss.wbt")))
    if rung == 9:
        eps = (tag == "c")
        if fault == "frozen":
            stem = "%sfrozen_rung9%s" % (FAULT_PREFIX, "_eps" if eps else "")
        elif fault == "seed_nudge" and tag == "b":
            stem = "%sseed_nudge_rung9" % FAULT_PREFIX
        else:
            # ``short_b`` is a controller fault and every other fault leaves
            # the scene alone, so both fall through to the honest world.
            stem = "rung9_eps" if eps else "rung9"
        return os.path.join(WORLDS, stem + ".wbt")
    if rung == 11:
        n = int(str(tag or "n%d" % rungs.RUNG11_N[0]).lstrip("n"))
        broken = (fault != "none" and n == rungs.RUNG11_FAULT_N)
        stem = ("%s%s_rung11_n%d" % (FAULT_PREFIX, fault, n) if broken
                else "rung11_n%d" % n)
        return os.path.join(WORLDS, stem + ".wbt")
    stem = ("rung%d" % rung if fault == "none"
            else "%s%s_rung%d" % (FAULT_PREFIX, fault, rung))
    return os.path.join(WORLDS, stem + ".wbt")


# The fault worlds the self-test needs, and only those: one per assertion
# family, not the cross product.  Kept in step with ``selftest.LIVE_FAULTS``;
# a fault listed there and missing here fails as "world missing", which is
# loud, and a world here that nothing runs is dead weight, which is not.
SELFTEST_WORLDS = ((0, "short_run"), (1, "no_floor"), (2, "half_gravity"),
                   (3, "ignore_zero"), (4, "slide"),
                   (5, "no_sweep"), (5, "wall_shifted"),
                   (6, "no_stop"), (6, "bounce"),
                   (7, "stalled_robot"), (7, "lane_offset"),
                   (8, "no_grip"), (8, "no_traverse"), (8, "drop_mid_carry"),
                   (9, "seed_nudge"), (9, "frozen"), (9, "short_b"),
                   (11, "stalled_robot"), (11, "lane_offset"))


def run_specs(rung, fault="none"):
    """The RUNS of one cell, in the contract's own order (amendment A).

    Single-run rungs return one unparameterised entry so callers need no
    special case.  Every tag and every parameter here is read from
    ``rungs.py``: an arm that chose its own N, or its own perturbation, would
    produce a row that is not comparable and nothing in the table would say so.
    """
    rung = int(rung)
    if rung == 9:
        out = []
        for tag, eps in rungs.RUNG9_RUNS:
            spec = {"tag": tag, "eps_m": eps, "nudge_m": 0.0}
            if fault == "seed_nudge" and tag == "b":
                spec["nudge_m"] = rungs.RUNG9_FAULT_NUDGE
            if fault == "short_b" and tag == "b":
                spec["short"] = rungs.RUNG9_FAULT_SHORT
            out.append(spec)
        return out
    if rung == 11:
        return [{"tag": "n%d" % n, "n": n} for n in rungs.RUNG11_N]
    return [{"tag": None}]


def write_all(verbose=True):
    os.makedirs(WORLDS, exist_ok=True)
    written, seen = [], set()

    def emit(rung, fault, spec):
        p = world_path(rung, fault, spec.get("tag"))
        if p in seen:
            # Rung 9's replicas a and b share ONE file, by construction.
            return
        seen.add(p)
        with open(p, "w", encoding="utf-8", newline="\n") as f:
            f.write(wbt(rung, fault, spec))
        written.append(p)

    for rung in rungs.RUNGS:
        for spec in run_specs(rung):
            emit(rung, "none", spec)
    for rung, fault in SELFTEST_WORLDS:
        for spec in run_specs(rung, fault):
            emit(rung, fault, spec)
    # The variant scenes: the N = 32 fleet and the starve-the-budget study.
    # Generated but never a ladder row -- CONTRACT.md section 3E.
    for spec in ({"tag": "n%d" % rungs.RUNG11_N_VARIANT,
                  "n": rungs.RUNG11_N_VARIANT},):
        emit(11, "none", spec)
    for spec in run_specs(11):
        emit(11, "starve_budget", spec)
    if verbose:
        for p in written:
            print(os.path.relpath(p, LADDER0))
    return written


if __name__ == "__main__":
    write_all()
