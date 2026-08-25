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

"""worldgen.py -- the ladder0 scenes, spelled for UPSTREAM Webots R2025a.

Every geometric and physical number is read out of ``ladder0/rungs.py``.
Nothing here re-derives an expected value and nothing here was captured from a
run: this module only decides how the shared scene is SPELLED in upstream
Webots' dialect of VRML.

Each spelling below was checked against the node definitions the R2025a
distribution itself ships in ``$WEBOTS_HOME/resources/nodes/*.wrl``, not
against recollection of the reference manual.  Four are load-bearing and not
guessable:

``WorldInfo.gravity`` is an **SFFloat**, not a vector
    ``WorldInfo.wrl`` declares ``SFFloat gravity 9.81  # along the down axis``.
    The magnitude is positive and the direction comes from
    ``coordinateSystem``.  A vector here is a parse error.

``Cylinder`` is aligned along **+Z**, not +Y
    ``Cylinder.wrl``: *"a cylinder centered at (0,0,0) ... with a central axis
    oriented along the local z-axis"*.  This is the modern ENU-era convention
    and it is the opposite of the VRML97 heritage that most Webots material
    still describes -- so it is exactly the kind of thing an author "knows"
    wrongly.  It matters enormously on rung 4: a wheel on an ``axis 0 1 0``
    hinge whose cylinder is left unrotated is a disc lying FLAT on the ground,
    spinning about one of its own diameters.  It grinds instead of rolling and
    its inertia about the hinge is the wrong tensor.  Every wheel here
    therefore carries an explicit ``Pose { rotation 1 0 0 -pi/2 }`` mapping the
    cylinder's +Z onto the parent's +Y -- on the COLLIDER as well as on the
    visual, since only the collider decides how the wheel meets the floor.
    Rung 4 exists to catch this class of mistake, so the scene must not contain
    it.

``ContactProperties.bounce`` defaults to 0.5
    A dropped box would visibly bounce on rung 2 and reach its rest height
    late.  The scene declares ``bounce 0`` / ``bounceVelocity 0`` so the only
    contact model in play is the compliant one ``rungs.REST_Z_TOL`` was derived
    for.

``Physics { density -1  mass M }``
    ``Physics.wrl``: ``mass`` is *"ignored if density != -1"*, and density
    defaults to 1000.  Setting mass alone silently keeps a density-derived
    mass -- a 0.2 m cube would weigh 8 kg instead of 1 kg.

One deliberate difference from a "normal" Webots world: the floor is a plain
``Solid`` with a ``Box`` bounding object rather than upstream's ``Floor``
PROTO.  ``rungs.FLOOR_TOP`` is 0.5 m specifically so an engine with a phantom
collision plane at z=0 is separable from one without, and the PROTO would also
be a network fetch (the R2025a distribution ships no ``.proto`` files at all).
These worlds resolve entirely offline.

Three more spellings are documented at the point of use further down, because
each was MEASURED here rather than recalled and each is silent when wrong: the
axis and units of a ``DistanceSensor`` (rungs 5-6), the fact that a slider's
``position`` is a DISPLACEMENT rather than an absolute coordinate, and the
startup impulse a joint takes when its hard stop is authored at its own initial
position (both rung 8).

MULTI-RUN RUNGS (CONTRACT.md amendment A)
-----------------------------------------
Rungs 9 and 11 are scene FAMILIES: one cell is several runs of one generator,
tagged and parameterised by ``rungs.py``.  ``run_specs(rung, fault)`` returns
that list and ``world_path(rung, fault, tag)`` names the file each run loads.
Two properties of that mapping are load-bearing:

* **Rung 9's replicas ``a`` and ``b`` are the SAME FILE.**  They are not two
  copies of one scene; they are one scene run twice, and a byte difference
  between them -- even in a comment -- is a difference the rung exists to rule
  out.  Everything a run needs that ``a`` and ``b`` do not share (the tag, the
  sample stride, the short fraction) therefore travels in the ENVIRONMENT, not
  in ``controllerArgs``.
* **Rung 9 numbers are written with ``%.17g``** (:func:`_hi`), not the
  ``%.6f`` every other scene uses.  The ``seed_nudge`` fault is 1e-12 m and
  ``%.6f`` writes it as ``"0"`` -- the fault would silently not happen, the
  run would be honest, and the self-test would report a ladder defect that is
  not one.

FAULT WORLDS
------------
``wbt(rung, fault, run)`` emits the scene-level fault variants: rung 1
``no_floor`` and rung 2 ``half_gravity``, which CONTRACT.md section 6 asks
for, plus rung 5 ``frozen_sensor`` and rung 8 ``weak_grip``, which it does not
-- section 6 stops at rung 4, so the rung 5-8 faults are this arm's own,
chosen to the same standard (each reddens the assertion it targets and leaves
the rest of its rung green).  Rungs 9 and 11 are back in the contract's own
list: ``seed_nudge`` and ``frozen`` (rung 9) and ``stalled_robot`` and
``lane_offset`` (rung 11) are scene faults; ``short_b`` (rung 9) is a driver
fault and needs no scene change.  Every one is generated from the same
constants by the same code, so a fault world differs from its control in
exactly one declared quantity and nothing else.  The driver-level faults
(``short_run``, ``ignore_zero``, ``slide``, ``late_stop``, ``crosstalk``,
``short_b``) live in the controller.
"""

from __future__ import annotations

import math
import os
import sys
import time

HERE = os.path.abspath(os.path.dirname(__file__))
LADDER0 = os.path.dirname(HERE)
if LADDER0 not in sys.path:
    sys.path.insert(0, LADDER0)

import rungs                                        # noqa: E402

WORLDS = os.path.join(HERE, "worlds")
CONTROLLER = "ladder0_wprobe"

# Scene-level faults: (rung, name) -> what the generator changes.
SCENE_FAULTS = {(1, "no_floor"), (2, "half_gravity"),
                (5, "frozen_sensor"), (8, "weak_grip"),
                (9, "seed_nudge"), (9, "frozen"),
                (11, "stalled_robot"), (11, "lane_offset")}


def _n(v):
    """Number -> a short, exact-looking VRML literal."""
    if isinstance(v, int) or float(v).is_integer():
        return str(int(v))
    return ("%.6f" % float(v)).rstrip("0").rstrip(".")


def _hi(v):
    """Number -> a literal that ROUND-TRIPS the float64 EXACTLY.

    Rung 9 needs this and :func:`_n` would silently destroy it.  ``_n``
    formats with ``%.6f``: the contract's 1 um sensitivity seed survives that
    (``0.100001``), but the 1 pm ``seed_nudge`` fault writes as **``0.1``** --
    the fault would simply not happen, the run would be the honest one,
    ``repeat_delta`` would come back green, and the self-test would report a
    fault that did not go red as a defect in the ladder rather than in this
    formatter.  ``%.17g`` is the shortest width that round-trips every
    float64, and it is CONTRACT.md amendment D applied to the scene file
    rather than to the sample document.
    """
    return "%.17g" % float(v)


# --------------------------------------------------------------------------
# preamble
# --------------------------------------------------------------------------

HEADER = """#VRML_SIM R2025a utf8

# GENERATED by tests/benchmarks/ladder0/webots/worldgen.py.
# Ladder0 rung %d -- %s.  Arm: UPSTREAM WEBOTS R2025a.%s
#
# Every number below is read from tests/benchmarks/ladder0/rungs.py, which holds
# the analytic ground truth for all three arms.  Do not hand-edit: change the
# contract and regenerate, or the scene and its expectation drift apart.
"""


def _world_info(gravity, mu=None):
    """``gravity`` is an SFFloat MAGNITUDE here -- see the module docstring.

    ``mu`` overrides the global Coulomb friction for the one rung that declares
    its own (rung 8, where ``rungs.RUNG8_MU`` buys the pinch a large margin so
    the rung measures "does a grasp hold" and not "where is this engine's
    friction limit").  Upstream's ``contactProperties`` is a LIST of
    material-pair entries, so this arm could give the pads and the part their
    own pair without touching the floor's -- but the contract asks for one
    global value per scene, and an arm that quietly used a more expressive
    spelling would no longer be running the same scene as the others.

    ``optimalThreadCount 1`` IS THE R2025a DEFAULT and is written down anyway.
    ``$WEBOTS_HOME/resources/nodes/WorldInfo.wrl`` in this distribution
    declares ``field SFInt32 optimalThreadCount 1``, so the line changes
    nothing about any run here -- but above 1 upstream splits ODE's island
    solve across threads, and a parallel island solve is exactly the class of
    mechanism rung 9 exists to catch (this ladder's GPU refutation is an
    atomic-``pairid`` race, which is the same shape of defect).  A determinism
    row has to say which configuration it came from, and a row whose
    configuration is "whatever this distribution happened to default to"
    cannot.  The recorder also reads the value BACK out of the loaded scene
    into the sample document, so what gets reported is the value the ENGINE
    used and not the value this file asked for.
    """
    mu = rungs.MU if mu is None else mu
    return """WorldInfo {
  info [
    "ladder0 -- hand-authored first-principles physics ladder."
    "Upstream Webots R2025a arm.  Ground truth: ladder0/rungs.py."
  ]
  gravity %(g)s
  basicTimeStep %(dt_ms)s
  FPS 30
  randomSeed 42
  optimalThreadCount 1
  coordinateSystem "ENU"
  contactProperties [
    ContactProperties {
      material1 "default"
      material2 "default"
      coulombFriction [ %(mu)s ]
      bounce 0
      bounceVelocity 0
    }
  ]
}
""" % {"g": _n(gravity), "dt_ms": _n(rungs.DT * 1000.0), "mu": _n(mu)}


VIEWPOINT = """Viewpoint {
  position -3.5 -3.5 2.5
  orientation -0.2795 0.3045 0.9105 1.7355
}
Background {
  skyColor [
    0.15 0.17 0.22
  ]
}
DirectionalLight {
  ambientIntensity 1
  direction 0.4 0.6 -1
  intensity 2
  castShadows FALSE
}
"""


# --------------------------------------------------------------------------
# pieces
# --------------------------------------------------------------------------

def _probe(rung, pose="", children_extra="", solid_fields=""):
    """The recorder.  A Supervisor on every rung; on rung 4 it is also the
    rover itself, which is why the geometry is injected here rather than
    authored as a separate Robot."""
    return """DEF PROBE Robot {
%(pose)s  name "ladder0_probe"
  controller "%(ctrl)s"
  controllerArgs [
    "--rung=%(rung)d"
  ]
  supervisor TRUE
%(children)s%(fields)s}
""" % {"pose": pose, "ctrl": CONTROLLER, "rung": rung,
       "children": children_extra, "fields": solid_fields}


def _floor(size=None):
    """The floor.  ``size`` is rung 11's only: ``rungs.rung11_floor_size(n)``
    widens the slab with the fleet, and the contract derives it (3x the
    along-track requirement, one whole spare lane each side).  The THICKNESS
    and the centre are untouched, so ``FLOOR_TOP`` -- which every rest height,
    ride height and penetration on this ladder is measured against -- is the
    same 0.5 m at every fleet size."""
    sx, sy, sz = (rungs.FLOOR_SIZE if size is None else size)
    return """DEF FLOOR Solid {
  translation 0 0 %(cz)s
  children [
    DEF FLOOR_SHAPE Shape {
      appearance PBRAppearance {
        baseColor 0.45 0.47 0.5
        roughness 1
        metalness 0
      }
      geometry Box {
        size %(sx)s %(sy)s %(sz)s
      }
    }
  ]
  name "floor"
  boundingObject USE FLOOR_SHAPE
}
""" % {"cz": _n(rungs.FLOOR_CENTER_Z), "sx": _n(sx), "sy": _n(sy),
       "sz": _n(sz)}


def _box(z):
    e = rungs.BOX_EDGE
    return """DEF BOX Solid {
  translation 0 0 %(z)s
  children [
    DEF BOX_SHAPE Shape {
      appearance PBRAppearance {
        baseColor 0.85 0.55 0.2
        roughness 0.9
        metalness 0
      }
      geometry Box {
        size %(e)s %(e)s %(e)s
      }
    }
  ]
  name "box"
  boundingObject USE BOX_SHAPE
  physics Physics {
    density -1
    mass %(m)s
  }
}
""" % {"z": _n(z), "e": _n(e), "m": _n(rungs.BOX_MASS)}


# --------------------------------------------------------------------------
# rung 3 -- one hinge about a VERTICAL axis, nothing to collide with
# --------------------------------------------------------------------------

# The link sits this far ABOVE the hinge anchor so it clears the base block
# visually.  It is arm-local dressing, NOT a contract quantity, and it cannot
# touch the rung's assertions: the hinge axis is vertical, so a translation
# along z changes neither the gravity torque about the joint (still exactly
# zero) nor the angle the position sensor reports.  There is no floor in this
# scene, so it cannot bring anything into contact either.
_LINK_Z_CLEARANCE = 0.08                 # m


def _arm_children():
    """The vertical hinge axis is the whole point: gravity produces no torque
    about it, so the motor is genuinely unloaded and the analytic steady state
    is exactly the commanded rate.  There is no floor in this scene and the
    link touches nothing.

    ``maxVelocity`` is set well above the command because a Webots
    RotationalMotor SATURATES at ``maxVelocity`` (default 10 rad/s) -- leaving
    it at a value near the command would cap the very quantity being measured
    and turn an engine property into a scene artefact."""
    return """  children [
    DEF BASE_SHAPE Shape {
      appearance PBRAppearance {
        baseColor 0.3 0.3 0.35
        roughness 1
        metalness 0
      }
      geometry Box {
        size 0.1 0.1 0.1
      }
    }
    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 0 1
        anchor 0 0 0
      }
      device [
        RotationalMotor {
          name "j0"
          maxVelocity %(maxv)s
          maxTorque 100
        }
        PositionSensor {
          name "j0_sensor"
        }
      ]
      endPoint DEF LINK Solid {
        translation %(half)s 0 %(lz)s
        children [
          DEF LINK_SHAPE Shape {
            appearance PBRAppearance {
              baseColor 0.2 0.7 0.9
              roughness 0.8
              metalness 0
            }
            geometry Box {
              size %(len)s %(w)s %(w)s
            }
          }
        ]
        name "link"
        boundingObject USE LINK_SHAPE
        physics Physics {
          density -1
          mass %(m)s
        }
      }
    }
  ]
""" % {"maxv": _n(4.0 * rungs.RUNG3_OMEGA_CMD),
       "half": _n(rungs.RUNG3_LINK_LEN / 2.0),
       "lz": _n(_LINK_Z_CLEARANCE),
       "len": _n(rungs.RUNG3_LINK_LEN),
       "w": _n(2.0 * rungs.RUNG3_LINK_RADIUS),
       "m": _n(rungs.RUNG3_LINK_MASS)}


# --------------------------------------------------------------------------
# rung 4 -- four driven wheels (rungs.py explains why not two plus a caster)
# --------------------------------------------------------------------------

# Maps the cylinder's own +Z axis onto the parent's +Y, which is the hinge
# axis.  See the module docstring: R2025a's Cylinder is Z-aligned.
_WHEEL_ROT = "1 0 0 %s" % _n(-math.pi / 2.0)


def _wheel(tag, sx, sy, prefix=""):
    """One driven wheel.

    ``prefix`` makes the DEF unique when a scene carries more than one rover
    (rung 7).  A duplicate DEF is not merely untidy here: ``boundingObject USE
    CHASSIS_SHAPE`` resolves to the FIRST node of that name, so five rovers
    sharing a DEF would share one collider node between five bodies.  Empty by
    default, so the single-rover scenes are emitted byte-identically.
    """
    x = sx * rungs.WHEEL_X
    y = sy * rungs.WHEEL_Y
    return """    HingeJoint {
      jointParameters HingeJointParameters {
        axis 0 1 0
        anchor %(x)s %(y)s 0
      }
      device [
        RotationalMotor {
          name "wheel_%(tag)s"
          maxVelocity %(maxv)s
          maxTorque 20
        }
        PositionSensor {
          name "wheel_%(tag)s_sensor"
        }
      ]
      endPoint DEF %(pfx)sWHEEL_%(TAG)s Solid {
        translation %(x)s %(y)s 0
        children [
          Pose {
            rotation %(rot)s
            children [
              Shape {
                appearance PBRAppearance {
                  baseColor 0.12 0.12 0.14
                  roughness 1
                  metalness 0
                }
                geometry Cylinder {
                  height %(w)s
                  radius %(r)s
                  subdivision 24
                }
              }
            ]
          }
        ]
        name "wheel_%(tag)s"
        boundingObject Pose {
          rotation %(rot)s
          children [
            Cylinder {
              height %(w)s
              radius %(r)s
              subdivision 24
            }
          ]
        }
        physics Physics {
          density -1
          mass %(m)s
        }
      }
    }
""" % {"tag": tag, "TAG": tag.upper(), "x": _n(x), "y": _n(y),
       "pfx": prefix,
       "rot": _WHEEL_ROT, "w": _n(rungs.WHEEL_W), "r": _n(rungs.WHEEL_R),
       "maxv": _n(4.0 * rungs.RUNG4_OMEGA_CMD), "m": _n(rungs.WHEEL_MASS)}


def _rover_children(prefix="", extra=""):
    """The rover's children: chassis shape, four driven wheels, and whatever
    ``extra`` a rung mounts on it (rungs 5 and 6 mount a DistanceSensor)."""
    cx, cy, cz = rungs.CHASSIS_SIZE
    head = """  children [
    DEF %(pfx)sCHASSIS_SHAPE Shape {
      appearance PBRAppearance {
        baseColor 0.85 0.7 0.2
        roughness 0.8
        metalness 0
      }
      geometry Box {
        size %(cx)s %(cy)s %(cz)s
      }
    }
""" % {"pfx": prefix, "cx": _n(cx), "cy": _n(cy), "cz": _n(cz)}
    wheels = "".join(_wheel(t, sx, sy, prefix) for t, sx, sy in rungs.WHEELS)
    return head + wheels + extra + "  ]\n"


def _rover_fields(prefix=""):
    return """  boundingObject USE %(pfx)sCHASSIS_SHAPE
  physics Physics {
    density -1
    mass %(m)s
  }
""" % {"pfx": prefix, "m": _n(rungs.CHASSIS_MASS)}


# --------------------------------------------------------------------------
# rungs 5 and 6 -- a distance sensor facing a wall
# --------------------------------------------------------------------------

# WHICH WAY DOES A WEBOTS DistanceSensor LOOK, AND IN WHAT UNITS?
#
# Both were MEASURED here on 2026-08-12, not recalled, because both are exactly
# the kind of thing an author "knows" wrongly -- and getting either wrong makes
# the rung red for a scene-authoring reason while looking like an engine
# defect.  This is the same trap R2025a's +Z-aligned ``Cylinder`` set for rung
# 4, so it was answered the same way: with a scene whose reading names the
# answer.
#
# One sensor, identity rotation, five walls at five distinct distances
# (+X 2.0, -X 2.5, +Y 3.0, -Y 3.5, +Z 4.0), one scalar reading.  It read
# ``2.000000``:
#
#   * rays leave the sensor along its own **+X**, and
#   * ``lookupTable [ 0 0 0, R R 0 ]`` makes the returned value the geometric
#     distance IN METRES, exactly -- 2.000000 for a 2.0 m gap, to six decimals.
#
# The default table is ``[ 0 0 0, 0.1 1000 0 ]``, which reports 1000 at 0.1 m:
# a scene that leaves it alone measures nothing this ladder can judge.  The
# largest distance in the table is also the ray's LENGTH, so R doubles as the
# sensor's range and a wall beyond it reads exactly R rather than "far".
#
# A second probe put the same sensor on a MOVING rover and compared the reading
# against the pose on every one of 1250 steps: ``max|reading - geometry| =
# 0.000000 m``.  So the ray does not clip the rover's own body at a stand-off
# like rung 6's, and it tracks rather than latching -- which is rung 5's whole
# claim, checked before it was asserted.
_DS_TYPE = "laser"          # single ray; "generic" with numberOfRays 1 is the
                            # same cast, but "laser" is the spelling that was
                            # actually measured, so it is the one used.


def _distance_sensor(name, dx, dz, max_range, frozen_at=None):
    """A forward-looking rangefinder, ``dx`` ahead of its parent's origin.

    It carries no ``physics`` and no ``boundingObject`` on purpose.  On rung 6
    the carrier is the rung-4 rover and its dynamics must stay the rung-4
    rover's dynamics exactly, so the sensor may add neither mass nor a
    collider.  ``resolution -1`` disables quantisation, which would otherwise
    put a step function between the geometry and the reading.

    ``frozen_at`` builds the rung-5 fault sensor: a FLAT lookup table, so the
    device reports that one value at every distance.  Set to the standoff the
    scene starts at, it is a sensor that is right about its first pose and
    never updates again -- exactly the asymmetry ``range_final`` exists to
    catch, expressed as a scene edit rather than a doctored measurement.
    """
    if frozen_at is None:
        table = "        0 0 0\n        %(r)s %(r)s 0\n" % {"r": _n(max_range)}
    else:
        table = "        0 %(v)s 0\n        %(r)s %(v)s 0\n" % {
            "v": _n(frozen_at), "r": _n(max_range)}
    return """    DistanceSensor {
      translation %(dx)s 0 %(dz)s
      name "%(name)s"
      lookupTable [
%(table)s      ]
      type "%(type)s"
      numberOfRays 1
      resolution -1
    }
""" % {"dx": _n(dx), "dz": _n(dz), "name": name, "table": table,
       "type": _DS_TYPE}


def _wall(center_x, center_z, size):
    """The wall rungs 5 and 6 range against.  Static: no ``physics``."""
    sx, sy, sz = size
    return """DEF WALL Solid {
  translation %(cx)s 0 %(cz)s
  children [
    DEF WALL_SHAPE Shape {
      appearance PBRAppearance {
        baseColor 0.55 0.5 0.45
        roughness 1
        metalness 0
      }
      geometry Box {
        size %(sx)s %(sy)s %(sz)s
      }
    }
  ]
  name "wall"
  boundingObject USE WALL_SHAPE
}
""" % {"cx": _n(center_x), "cz": _n(center_z),
       "sx": _n(sx), "sy": _n(sy), "sz": _n(sz)}


def _carrier(fault="none"):
    """Rung 5's carrier: a KINEMATIC body whose pose the driver writes.

    No ``physics`` and no ``boundingObject``, so it is not simulated and cannot
    touch anything -- which is the point.  Rung 5 measures the RAY, and a
    carrier that was driven would fold "the rover did not get where it was
    told" into the sensor's residual.  Rung 6 is where the two are put back
    together.
    """
    frozen = (rungs.RUNG5_STANDOFF if fault == "frozen_sensor" else None)
    sensor = _distance_sensor("ds", rungs.RUNG5_SENSOR_DX, 0.0,
                              rungs.RUNG5_MAX_RANGE, frozen_at=frozen)
    e = rungs.RUNG5_CARRIER_EDGE
    children = """  children [
    DEF CARRIER_SHAPE Shape {
      appearance PBRAppearance {
        baseColor 0.2 0.7 0.9
        roughness 0.8
        metalness 0
      }
      geometry Box {
        size %(e)s %(e)s %(e)s
      }
    }
%(sensor)s  ]
""" % {"e": _n(e), "sensor": sensor}
    return _probe(5, pose="  translation %s 0 %s\n"
                          % (_n(rungs.RUNG5_X0), _n(rungs.RUNG5_SENSOR_Z)),
                  children_extra=children)


# --------------------------------------------------------------------------
# rung 7 -- five independent rovers, each with its own command
# --------------------------------------------------------------------------

def _rover_robot(idx, y, rung=7, omega=None):
    """One of rung 7's rovers: the rung-4 rover, in its own lane.

    ``rung``/``omega`` are rung 11's.  Rung 7's driver looks its command up in
    ``rungs.RUNG7_OMEGA[idx]``, which has five entries; rung 11 runs up to
    sixteen rovers whose commands CYCLE that tuple, so the command is written
    into the scene instead and the driver is told what it is.  That also gives
    rung 11's ``stalled_robot`` fault a home in the SCENE (one robot's command
    is 0) rather than in the driver, which keeps every fault of this rung a
    one-quantity difference in a generated file.  Rung 7 passes neither, so its
    world is emitted byte-identically to before.

    It is a plain ``Robot``, not a Supervisor, and it does not record -- it
    drives and nothing else.  The recorder is a separate massless observer, so
    all five rovers stay interchangeable bodies, which matters on the one rung
    whose claim is that no robot perturbs another.

    A ROVER NEVER ENDS THE RUN.  Measured on this arm: when the drivers ran out
    their own step budget and their processes exited, upstream ended the
    simulation and cut the recorder off before it could write anything -- all
    five heartbeats stopped at t = 2.50 s of a 3.00 s run and no sample
    document was produced.  Driving until the engine says stop, and letting the
    recorder alone own the run's length, fixed it.
    """
    pfx = "R%d_" % idx
    extra = "" if omega is None else '    "--omega=%s"\n' % repr(float(omega))
    return """DEF ROVER%(i)d Robot {
  translation 0 %(y)s %(z)s
  name "rover%(i)d"
  controller "%(ctrl)s"
  controllerArgs [
    "--rung=%(rung)d"
    "--role=driver"
    "--idx=%(i)d"
%(extra)s  ]
  supervisor FALSE
%(children)s%(fields)s}
""" % {"i": idx, "y": _n(y), "z": _n(rungs.ROBOT_Z), "ctrl": CONTROLLER,
       "rung": int(rung), "extra": extra,
       "children": _rover_children(prefix=pfx),
       "fields": _rover_fields(prefix=pfx)}


def _observer(rung, args_extra=()):
    """Rung 7's recorder: a Supervisor with no mass, no collider and no job in
    the scene except to watch it.  It reads the fleet's poses AND each wheel's
    angle out of the scene tree, so it needs no device of its own -- upstream,
    like OmniSim, lets a robot read only its OWN sensors.

    Rungs 9 and 11 use the same body-less observer for the same reason: a
    determinism rung must not add a 27th body to a scene whose whole claim is
    about how 26 of them reproduce, and a fleet rung must not add an (N+1)th
    object to a scene whose whole claim is that N of them do not interact.
    ``args_extra`` carries rung 11's ``--n``; everything that differs between
    two RUNS of one scene family travels in the environment instead, because
    rung 9's replicas are deliberately the same file."""
    extra = "".join('    "%s"\n' % a for a in args_extra)
    return """DEF PROBE Robot {
  translation %(x)s 0 %(z)s
  name "ladder0_probe"
  controller "%(ctrl)s"
  controllerArgs [
    "--rung=%(rung)d"
    "--role=recorder"
%(extra)s  ]
  supervisor TRUE
  children [
  ]
}
""" % {"x": _n(-2.0), "z": _n(rungs.ROBOT_Z + 2.0), "ctrl": CONTROLLER,
       "rung": rung, "extra": extra}


# --------------------------------------------------------------------------
# rung 8 -- a Cartesian gantry gripper lifts a payload off a table
# --------------------------------------------------------------------------
#
# Two spellings here are load-bearing and were verified with a throwaway probe
# before the rung was built (it lifted a payload 0.1525 m clear of a table and
# carried it 0.40 m with the payload-to-gripper offset constant to 10 um):
#
# ``LinearMotor`` position control is a VELOCITY servo driven by position
#     error and saturated at ``maxForce`` -- NOT a spring.  Commanding a pad
#     past the part therefore does not scale the squeeze with the
#     interference: once the pad is blocked, the motor delivers exactly
#     ``maxForce``.  So ``maxForce`` IS the contract's ``RUNG8_GRIP_N`` per
#     pad, directly, with no gain to calibrate on the way.  (OmniSim's Newton
#     path is a spring anchored at the last target, which is why the same
#     number has to be spelled differently there.)
#
# ``minPosition``/``maxPosition`` are IGNORED when they are equal, and both
#     default to 0.  A slider that leaves them alone is pinned at its initial
#     position and the whole gantry silently does not move.

# The two positioning stages are sized from the mass they must move, not tuned:
# a decade of margin over the static weight of everything below them, so the
# stage TRACKS its commanded schedule instead of sagging under the payload.
# The rung asserts the schedule, never the force, so any sizing comfortably
# above the load gives the same measurement -- and the pads, whose force IS
# asserted, are the contract's RUNG8_GRIP_N exactly.
_GANTRY_FORCE_MARGIN = 10.0
_MOVING_MASS = (rungs.RUNG8_CARRIAGE_MASS + rungs.RUNG8_WRIST_MASS
                + 2.0 * rungs.RUNG8_PAD_MASS + rungs.RUNG8_PART_MASS)
_STAGE_FORCE = _GANTRY_FORCE_MARGIN * _MOVING_MASS * rungs.G

# Same convention as the RotationalMotors on rungs 3 and 4: give the actuator
# headroom over the rate it is commanded at, so `maxVelocity` cannot silently
# become the quantity being measured.
_MOTOR_SPEED_MARGIN = 4.0
_PAD_CLOSE_V = (rungs.RUNG8_PAD_OPEN_Y
                / (rungs.RUNG8_T_CLOSE - rungs.RUNG8_T_SETTLE))

# UPSTREAM'S MOTOR POSITION CONTROL IS A P-CONTROLLED VELOCITY SERVO: it drives
# at ``controlPID.x * (target - position)``, so a stage following a
# constant-velocity ramp settles a STEADY ``v / P`` behind its own command.  At
# the shipped default P = 10 s^-1 the traverse lags 0.15/10 = 15 mm and the
# lift 10 mm -- and RUNG8_POSE_TOL is 10 mm, so the gantry does not actually
# execute the schedule the rung asserts.  Measured before this line existed:
# ``hold_clearance`` 0.139862 m against an expected 0.15 (margin -0.000138)
# while every other rung-8 check was green, which reads like a payload that
# sagged in the grip and is nothing of the sort.
#
# This is the same class of authoring trap as leaving ``maxVelocity`` near the
# commanded rate on rung 3.  P is therefore set so the worst following error is
# a TENTH of the tolerance the rung is judged by -- derived from the contract's
# own speeds and tolerance, not tuned until it passed.
_GANTRY_FOLLOW_FRACTION = 0.1
_GANTRY_P = (max(rungs.RUNG8_LIFT_V, rungs.RUNG8_TRAVERSE_V)
             / (_GANTRY_FOLLOW_FRACTION * rungs.RUNG8_POSE_TOL))


def _slider(axis, name, lo, hi, max_force, max_velocity, endpoint):
    """One prismatic axis: a LinearMotor and its PositionSensor."""
    # NO minStop/maxStop.  Every one of these axes STARTS at position 0, and 0
    # is an endpoint of its own travel, so authoring the hard stops at the
    # travel limits puts each joint exactly ON a stop at t=0 -- which fires a
    # startup impulse.  Measured: the wrist jumped 9.25 mm in the second step
    # and rang down over ~50 ms, which set ``carry_rel`` at 0.009427 m against
    # a 0.01 m tolerance -- 94% of the budget spent before the grasp began, on
    # a transient with nothing to do with the grasp.  The motor's
    # minPosition/maxPosition already clamp every command to the same range, so
    # the stops were redundant as well as harmful.
    return """SliderJoint {
  jointParameters JointParameters {
    axis %(axis)s
  }
  device [
    LinearMotor {
      name "%(name)s"
      controlPID %(p)s 0 0
      maxVelocity %(maxv)s
      maxForce %(maxf)s
      minPosition %(lo)s
      maxPosition %(hi)s
    }
    PositionSensor {
      name "%(name)s_sensor"
    }
  ]
  endPoint %(endpoint)s
}
""" % {"axis": axis, "name": name, "lo": _n(lo), "hi": _n(hi),
       "maxf": _n(max_force), "maxv": _n(max_velocity),
       "p": _n(_GANTRY_P), "endpoint": endpoint.lstrip()}


def _box_solid(defname, name, translation, size, mass, color,
               shape_dz=0.0, extra=""):
    """A box Solid.  ``shape_dz`` offsets the GEOMETRY from the Solid's origin
    without moving the origin -- rung 8's wrist needs its origin at the part's
    centre while its plate hangs above the part."""
    if shape_dz:
        shape = """    Pose {
      translation 0 0 %(dz)s
      children [
        DEF %(D)s_SHAPE Shape {
          appearance PBRAppearance {
            baseColor %(c)s
            roughness 1
            metalness 0
          }
          geometry Box {
            size %(sx)s %(sy)s %(sz)s
          }
        }
      ]
    }
""" % {"D": defname, "c": color, "dz": _n(shape_dz),
       "sx": _n(size[0]), "sy": _n(size[1]), "sz": _n(size[2])}
        bound = """Pose {
    translation 0 0 %s
    children [
      USE %s_SHAPE
    ]
  }""" % (_n(shape_dz), defname)
    else:
        shape = """    DEF %(D)s_SHAPE Shape {
      appearance PBRAppearance {
        baseColor %(c)s
        roughness 1
        metalness 0
      }
      geometry Box {
        size %(sx)s %(sy)s %(sz)s
      }
    }
""" % {"D": defname, "c": color,
       "sx": _n(size[0]), "sy": _n(size[1]), "sz": _n(size[2])}
        bound = "USE %s_SHAPE" % defname
    return """DEF %(D)s Solid {
  translation %(t)s
  children [
%(shape)s%(extra)s  ]
  name "%(n)s"
  boundingObject %(bound)s
  physics Physics {
    density -1
    mass %(m)s
  }
}
""" % {"D": defname, "n": name, "t": translation, "shape": shape,
       "extra": _indent(extra, 4), "bound": bound, "m": _n(mass)}


def _pad(side, sign, grip_force):
    """One gripper pad, hanging from the wrist and closing along y.

    A SLIDER'S POSITION IS A DISPLACEMENT FROM THE AUTHORED POSE, NOT AN
    ABSOLUTE COORDINATE.  The pad is authored at ``y = sign * PAD_OPEN_Y``, so
    position 0 is OPEN and closing means travelling ``-sign``.  Getting this
    backwards is silent and reads exactly like a failed grasp: measured here
    first time out, the pads were commanded outward, never touched the part,
    and the rung reported ``carry_rel`` 0.474 m with the payload sitting
    untouched on the table at its correct rest height -- a gripper that missed,
    wearing the failure signature of a grip that slipped.

    Travel therefore runs from 0 (open) to ``-sign * PAD_OPEN_Y`` (pads
    commanded together at y = 0).  They never get there: the part blocks them
    at ``RUNG8_PAD_TOUCH_Y``, and that is exactly what keeps each servo
    saturated at ``grip_force``.
    """
    pad = _box_solid("PAD_%s" % side.upper(), "pad_%s" % side,
                     "0 %s 0" % _n(sign * rungs.RUNG8_PAD_OPEN_Y),
                     rungs.RUNG8_PAD_SIZE, rungs.RUNG8_PAD_MASS,
                     "0.2 0.7 0.9")
    lo, hi = sorted((0.0, -sign * rungs.RUNG8_PAD_OPEN_Y))
    return _slider("0 1 0", "pad_%s" % side, lo, hi, grip_force,
                   _MOTOR_SPEED_MARGIN * _PAD_CLOSE_V, pad)


def _gantry(fault="none"):
    """Traverse (x) over lift (z) over two opposed pads (y).

    THE WRIST ORIGIN IS AUTHORED AT THE PART'S CENTRE, which is what makes the
    contract's ``carry_rel`` an analytic zero rather than a self-reference: the
    wrist Solid's translation puts its origin at ``rungs.RUNG8_GRASP_Z`` while
    the lift is at 0, and its plate is drawn ``RUNG8_WRIST_PLATE_DZ`` above
    that so it clears the part instead of resting on it.

    THE FAULT WORLD.  ``weak_grip`` sets the pad force to a tenth of the
    Coulomb bound ``rungs.rung8_grip_force_bound()`` -- a pinch that provably
    cannot hold the part's own weight by friction.  Nothing else changes.
    """
    grip = (0.1 * rungs.rung8_grip_force_bound() if fault == "weak_grip"
            else rungs.RUNG8_GRIP_N)
    pads = _pad("l", +1, grip) + _pad("r", -1, grip)
    wrist = _box_solid("WRIST", "wrist",
                       "0 0 %s" % _n(rungs.RUNG8_GRASP_Z - rungs.RUNG8_BASE_Z),
                       rungs.RUNG8_WRIST_SIZE, rungs.RUNG8_WRIST_MASS,
                       "0.35 0.35 0.4",
                       shape_dz=rungs.RUNG8_WRIST_PLATE_DZ, extra=pads)
    lift = _slider("0 0 1", "lift", 0.0, rungs.RUNG8_LIFT_H, _STAGE_FORCE,
                   _MOTOR_SPEED_MARGIN * rungs.RUNG8_LIFT_V, wrist)
    carriage = _box_solid("CARRIAGE", "carriage", "0 0 0",
                          rungs.RUNG8_CARRIAGE_SIZE,
                          rungs.RUNG8_CARRIAGE_MASS, "0.45 0.45 0.5",
                          extra=lift)
    traverse = _slider("1 0 0", "traverse", 0.0, rungs.RUNG8_TRAVERSE_X,
                       _STAGE_FORCE,
                       _MOTOR_SPEED_MARGIN * rungs.RUNG8_TRAVERSE_V, carriage)
    # A Robot with NO physics and NO boundingObject is a STATIC anchor -- the
    # gantry rail.  Give it either and the whole stage becomes a free body that
    # the first grasp reaction shoves across the floor.
    return """DEF PROBE Robot {
  translation 0 0 %(z)s
  name "ladder0_probe"
  controller "%(ctrl)s"
  controllerArgs [
    "--rung=8"
  ]
  supervisor TRUE
  children [
%(traverse)s  ]
}
""" % {"z": _n(rungs.RUNG8_BASE_Z), "ctrl": CONTROLLER,
       "traverse": _indent(traverse, 4)}


def _table():
    """The table, centred on the MIDPOINT OF THE TRAVERSE so it supports the
    part where it is picked up and still lies under it where it is placed.
    ``hold_clearance`` is measured against ``RUNG8_TABLE_TOP`` for the whole
    carry, and a table that stopped short would turn that into a claim about
    air."""
    sx, sy, sz = rungs.RUNG8_TABLE_SIZE
    return """DEF TABLE Solid {
  translation %(cx)s 0 %(cz)s
  children [
    DEF TABLE_SHAPE Shape {
      appearance PBRAppearance {
        baseColor 0.5 0.42 0.32
        roughness 1
        metalness 0
      }
      geometry Box {
        size %(sx)s %(sy)s %(sz)s
      }
    }
  ]
  name "table"
  boundingObject USE TABLE_SHAPE
}
""" % {"cx": _n(rungs.RUNG8_TRAVERSE_X / 2.0),
       "cz": _n(rungs.RUNG8_TABLE_CENTER_Z),
       "sx": _n(sx), "sy": _n(sy), "sz": _n(sz)}


def _part():
    e = rungs.RUNG8_PART_EDGE
    return """DEF PART Solid {
  translation 0 0 %(z)s
  children [
    DEF PART_SHAPE Shape {
      appearance PBRAppearance {
        baseColor 0.9 0.45 0.15
        roughness 0.9
        metalness 0
      }
      geometry Box {
        size %(e)s %(e)s %(e)s
      }
    }
  ]
  name "part"
  boundingObject USE PART_SHAPE
  physics Physics {
    density -1
    mass %(m)s
  }
}
""" % {"z": _n(rungs.RUNG8_PART_Z0), "e": _n(e),
       "m": _n(rungs.RUNG8_PART_MASS)}


def _indent(text, spaces):
    pad = " " * spaces
    return "".join((pad + ln if ln.strip() else ln)
                   for ln in text.splitlines(True))


# --------------------------------------------------------------------------
# rung 9 -- a 5x5 pile and a 26th cube dropped on the pile's OUTER corner
# --------------------------------------------------------------------------
#
# WHERE THE 26th CUBE GOES IS THE CONTRACT'S, AND IT IS NOT THE OBVIOUS PLACE.
# ``rungs.RUNG9_DROP_XY`` puts it over the pile's outer corner rather than over
# the centre cube's corner; rungs.py records why (at a 1 mm gap, a cube on the
# centre cube's corner overlaps four neighbours by a quarter each and is
# perfectly supported, so the scene has nothing to amplify and the sensitivity
# control reads as "this engine damps perturbations").  This module reads the
# constant and never reconstructs it, so a later correction to that placement
# reaches this arm without anyone remembering to come here.

def _rung9_cube(defname, name, x, y, z, dynamic=True, hi=False):
    """One cube of rung 9.

    EVERY COORDINATE GOES THROUGH :func:`_hi`, not ``_n``.  The dropped cube's
    x carries a 1e-6 m seed and, under ``seed_nudge``, a 1e-12 m offset; the
    pile's own pitch is ``0.2 + 0.001``, which is not exactly representable
    either.  A scene whose numbers do not round-trip is a scene that differs
    from the contract by an amount of the same order as the thing being
    measured.

    ``dynamic=False`` drops the ``physics`` node and keeps the
    ``boundingObject``: the cube is still a collider, it simply cannot move.
    That is the ``frozen`` fault, and it is the one that matters -- a world
    that cannot move is PERFECTLY deterministic, so ``repeat_delta`` must stay
    green while the sensitivity control and the analytic anchor both go red.
    """
    e = rungs.BOX_EDGE
    phys = "" if not dynamic else """  physics Physics {
    density -1
    mass %s
  }
""" % _n(rungs.BOX_MASS)
    return """DEF %(D)s Solid {
  translation %(x)s %(y)s %(z)s
  children [
    DEF %(D)s_SHAPE Shape {
      appearance PBRAppearance {
        baseColor %(col)s
        roughness 0.9
        metalness 0
      }
      geometry Box {
        size %(e)s %(e)s %(e)s
      }
    }
  ]
  name "%(n)s"
  boundingObject USE %(D)s_SHAPE
%(phys)s}
""" % {"D": defname, "n": name, "x": _hi(x), "y": _hi(y), "z": _hi(z),
       "e": _n(e), "phys": phys,
       # Keyed on ``hi`` -- i.e. on "is this the dropped cube" -- so the frozen
       # scene differs from the honest one in exactly ONE field, the one the
       # fault removes.  A second difference, even a colour, is a second thing
       # a reader has to rule out.
       "col": "0.9 0.35 0.15" if hi else "0.85 0.55 0.2"}


def _rung9_bodies(eps=0.0, nudge=0.0, frozen=False):
    """The resting pile + the cube released over its outer corner.

    ``eps`` is the contract's sensitivity seed (replica ``c``) and ``nudge`` is
    the ``seed_nudge`` fault (replica ``b``).  They are separate arguments and
    are never summed by the caller, so a run cannot be both the control and the
    fault by accident.
    """
    out = []
    for tag, x, y in rungs.rung9_pile_xy():
        out.append(_rung9_cube(tag.upper(), tag, x, y, rungs.RUNG9_PILE_Z))
    out.append(_rung9_cube("DROP", "drop",
                           rungs.RUNG9_DROP_XY + eps + nudge,
                           rungs.RUNG9_DROP_XY, rungs.RUNG9_SPAWN_Z,
                           dynamic=not frozen, hi=True))
    return "".join(out)


# --------------------------------------------------------------------------
# rung 11 -- the rung-4 rover at N = 1, 4, 8, 16
# --------------------------------------------------------------------------

def _rung11_robots(n, fault="none"):
    """``n`` copies of the rung-4 rover, each with its OWN driver process.

    ONE DRIVER PER ROBOT is not a choice this arm gets to make: upstream
    Webots, exactly like OmniSim, lets a robot read and write only its OWN
    devices, so a supervisor cannot command a sibling rover's wheels.  This is
    rung 7's machinery at four fleet sizes and nothing else -- the same rover
    body, the same driver, the same pose-derived wheel angle read by the same
    body-less observer.  The cost is real and is part of the result: at N = 16
    this world starts SEVENTEEN controller processes.

    Both faults are spawn-time and both apply only at ``RUNG11_FAULT_N``:
    ``stalled_robot`` writes robot 7's command as 0, and ``lane_offset``
    SPAWNS it ``RUNG11_FAULT_OFFSET_M`` out of its lane.  CONTRACT.md section 6
    records why the second may never be a per-step supervisor write: a per-step
    field write costs the body its state, and the first attempt reddened both
    must-green companions, destroying exactly what the fault was meant to
    isolate.
    """
    out = []
    for i in range(n):
        omega = rungs.rung11_omega(i)
        y = rungs.rung11_y(i, n)
        if i == rungs.RUNG11_FAULT_ROBOT and n > rungs.RUNG11_FAULT_ROBOT:
            if fault == "stalled_robot":
                omega = 0.0
            elif fault == "lane_offset":
                y += rungs.RUNG11_FAULT_OFFSET_M
        out.append(_rover_robot(i, y, rung=11, omega=omega))
    return "".join(out)


# --------------------------------------------------------------------------
# the worlds
# --------------------------------------------------------------------------

def wbt(rung, fault="none", run=None):
    """The scene for one RUN of one rung.  ``fault`` changes exactly one
    declared quantity and nothing else -- see the module docstring.

    ``run`` is the run spec of a multi-run rung (amendment A), as produced by
    :func:`run_specs`.  Single-run rungs ignore it entirely, so the call
    ``wbt(1)`` keeps the exact meaning it had.
    """
    rung = int(rung)
    run = run or {}
    gravity = rungs.G / 2.0 if fault == "half_gravity" else rungs.G
    mu = rungs.RUNG8_MU if rung == 8 else None
    note = ("" if fault == "none"
            else "\n# FAULT WORLD (%s) -- deliberately broken; see CONTRACT.md"
                 " section 6." % fault)
    parts = [HEADER % (rung, rungs.RUNG_TITLE[rung], note),
             _world_info(gravity, mu), VIEWPOINT]
    floor = [] if fault == "no_floor" else [_floor()]

    if rung == 0:
        parts.append(_probe(0))
    elif rung == 1:
        parts += floor + [_box(rungs.RUNG1_SPAWN_Z), _probe(1)]
    elif rung == 2:
        parts += floor + [_box(rungs.RUNG2_SPAWN_Z), _probe(2)]
    elif rung == 3:
        parts.append(_probe(
            3, pose="  translation 0 0 %s\n" % _n(rungs.RUNG3_HINGE_Z),
            children_extra=_arm_children()))
    elif rung == 4:
        parts += floor + [_probe(4,
                                 pose="  translation 0 0 %s\n"
                                      % _n(rungs.ROBOT_Z),
                                 children_extra=_rover_children(),
                                 solid_fields=_rover_fields())]
    elif rung == 5:
        parts += floor + [_wall(rungs.RUNG5_WALL_CENTER_X,
                                rungs.RUNG5_WALL_CENTER_Z,
                                rungs.RUNG5_WALL_SIZE),
                          _carrier(fault)]
    elif rung == 6:
        # The wall is rung 5's, by the contract's own statement that the
        # geometry is identical; only the sensor's mount and the carrier
        # change.  RUNG5_MAX_RANGE is the one span the contract declares, and
        # it covers rung 6's 2.68 m start gap with room to spare.
        sensor = _distance_sensor("ds", rungs.RUNG6_SENSOR_DX,
                                  rungs.RUNG6_SENSOR_Z - rungs.ROBOT_Z,
                                  rungs.RUNG5_MAX_RANGE)
        parts += floor + [_wall(rungs.RUNG5_WALL_CENTER_X,
                                rungs.RUNG5_WALL_CENTER_Z,
                                rungs.RUNG5_WALL_SIZE),
                          _probe(6,
                                 pose="  translation 0 0 %s\n"
                                      % _n(rungs.ROBOT_Z),
                                 children_extra=_rover_children(extra=sensor),
                                 solid_fields=_rover_fields())]
    elif rung == 7:
        parts += floor + [_rover_robot(i, y)
                          for i, y in enumerate(rungs.RUNG7_Y)]
        parts.append(_observer(7))
    elif rung == 8:
        parts += floor + [_table(), _part(), _gantry(fault)]
    elif rung == 9:
        parts += floor + [_rung9_bodies(eps=float(run.get("eps_m") or 0.0),
                                        nudge=float(run.get("nudge_m") or 0.0),
                                        frozen=(fault == "frozen")),
                          _observer(9)]
    elif rung == 11:
        n = int(run.get("n") or rungs.RUNG11_N[0])
        parts += [_floor(size=rungs.rung11_floor_size(n)),
                  _rung11_robots(n, fault),
                  _observer(11, args_extra=("--n=%d" % n,))]
    else:
        raise ValueError("no such rung: %r" % (rung,))
    return "\n".join(p.rstrip("\n") for p in parts) + "\n"


def run_specs(rung, fault="none"):
    """The RUNS of one cell, in the contract's own order (amendment A).

    Every tag and every parameter here is read from ``rungs.py``.  An arm that
    chose its own perturbation, or its own fleet sizes, would produce a row
    that is not comparable with the other arms' and nothing in the table would
    say so.  Single-run rungs return one unparameterised entry, so callers need
    no special case.
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


def world_path(rung, fault="none", tag=None):
    """The scene file for one RUN of one cell.

    A single-run rung ignores ``tag`` and keeps exactly the path it had.

    RUNG 9's REPLICAS ``a`` AND ``b`` MUST RESOLVE TO ONE FILE.  They are not
    two copies of one scene; they are one scene run twice, and a byte
    difference between them -- even in a comment -- is a difference the rung
    exists to rule out.  Only ``c`` (the 1 um sensitivity seed) and the
    scene-level faults get files of their own.
    """
    rung = int(rung)
    if rung == 9:
        eps = (tag == "c")
        if fault == "frozen":
            stem = "rung9_frozen_eps" if eps else "rung9_frozen"
        elif fault == "seed_nudge" and tag == "b":
            stem = "rung9_seed_nudge"
        else:
            # ``short_b`` is a driver fault and every other fault leaves the
            # scene alone, so both fall through to the honest world.
            stem = "rung9_eps" if eps else "rung9"
        return os.path.join(WORLDS, stem + ".wbt")
    if rung == 11:
        n = int(str(tag or "n%d" % rungs.RUNG11_N[0]).lstrip("n"))
        # Both rung-11 faults are declared at ONE fleet size, so the other
        # three runs of a faulted cell load the honest world -- which is what
        # makes the must-green companions a comparison against this arm's own
        # honest fleet rather than against a differently broken one.
        broken = (fault not in (None, "none") and n == rungs.RUNG11_FAULT_N)
        stem = ("rung11_n%d_%s" % (n, fault) if broken else "rung11_n%d" % n)
        return os.path.join(WORLDS, stem + ".wbt")
    if fault in (None, "none") or (rung, fault) not in SCENE_FAULTS:
        return os.path.join(WORLDS, "rung%d.wbt" % rung)
    return os.path.join(WORLDS, "rung%d_%s.wbt" % (rung, fault))


def write_all(verbose=True):
    """Emit every scene of every rung, from the contract.

    VERIFY, THEN WRITE ONLY WHAT DIFFERS.  The arm calls this before every
    cell, because a committed world that had drifted from ``rungs.py`` would
    be measured against the wrong expectation -- and that guarantee is a
    COMPARISON, not a write.  Rewriting all two dozen files each time bought
    nothing and cost a real failure: this arm runs its engine inside WSL over
    ``/mnt/o``, and truncating a world file from the Windows side while the
    9p/drvfs bridge still held a handle from the previous cell surfaced as
    ``OSError(22, 'Invalid argument')``.  Measured 2026-08-13, twice, on rung
    11's fault cells -- which then reported ``distance_worst=None`` and were
    scored as "the fault went red".  A red that comes from the ARM crashing is
    not evidence about the engine, and it is indistinguishable in the table
    from the fault working.

    So: compare first, write only on a real difference, and retry a write that
    loses the race rather than failing the cell for it.
    """
    os.makedirs(WORLDS, exist_ok=True)
    written, seen = [], set()

    def emit(rung, fault, spec):
        p = world_path(rung, fault, spec.get("tag"))
        if p in seen:
            # Rung 9's replicas a and b share ONE file, by construction, and
            # every run of a rung-11 fault cell except the broken fleet size
            # shares the honest world.
            return
        seen.add(p)
        # Every scene the contract defines goes in the returned list, whether
        # it had to be rewritten or was merely verified -- what the caller
        # wants to know is "these N scenes match rungs.py", and a count that
        # collapsed to 0 on the second call would read as a broken generator.
        written.append(p)
        text = wbt(rung, fault, spec)
        try:
            with open(p, "r", encoding="utf-8", newline="") as f:
                if f.read() == text:
                    return                  # already the contract's scene
        except OSError:
            pass                            # absent or unreadable: write it
        last = None
        for _ in range(4):
            try:
                with open(p, "w", encoding="utf-8", newline="\n") as f:
                    f.write(text)
                return
            except OSError as exc:          # noqa: PERF203
                last = exc
                time.sleep(0.25)
        raise last

    for rung in rungs.RUNGS:
        for spec in run_specs(rung):
            emit(rung, "none", spec)
        for r, fault in sorted(SCENE_FAULTS):
            if r != rung:
                continue
            for spec in run_specs(rung, fault):
                emit(rung, fault, spec)
    if verbose:
        for p in written:
            print(os.path.relpath(p, HERE))
    return written


if __name__ == "__main__":
    write_all()
