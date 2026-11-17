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

"""gen_worlds.py — generate the OmniBench lane-1 .wbt worlds (T1–T7 x dt sweep).

Contract: tests/benchmarks/omnibench/SPEC.md. One world per (scene, dt) pair,
dt in {1, 2, 4, 8, 16, 32} ms via WorldInfo.basicTimeStep (an SFFloat in this
engine — fractional values would be accepted, the sweep only needs integers).
T2 additionally sweeps 7 incline angles, so it emits 7 worlds per dt.

Worlds are headless test worlds (under tests/ — exempt from the canonical
lighting recipe). Each contains exactly one Robot with `supervisor TRUE`
running the `omnibench_recorder` controller; for the articulated scenes
(T4/T5) that Robot IS the scene (the 3-link chain).

⚠️ ONE ARM ONLY, SINCE 2026-08-08. This generator used to emit TWO worlds per
(scene, dt): the Newton one and an `_odepin` twin carrying `WorldInfo
defaultPhysicsBackend "ode"`, so lane 1 could score ODE beside Newton — and the
ODE arm was best or tied-best on 6 of 7 scenes at dt=4. (⚠️ An INTEGRATION
result, not a solver one: bare mujoco scored fine on the same scenes, and the
four defects behind Newton's deficit were in our own scene-graph-to-solver
plumbing and were fixed in `e7b9fb11`. See
docs/benchmarks/correctness-scope.md.) **src/ode was DELETED (commit
bdc02139).** There is no ODE arm, so this lane has no second IN-ENGINE arm any
more — it does still have its ORACLE, which is the analytic ground truth. What
lane 1
can still measure is Newton against the **analytic ground truth** each scene was
designed around (T1 bounce height, T2 stick/slide transition angle, T3 rolling
acceleration, T4 energy drift, T5 momentum conservation, T6 stack survival and
penetration, T7 angular-momentum drift) — which is the harder and more useful
half. It can no longer answer "is Newton as good as ODE here"; the frozen ODE
numbers that question was last answered with are in
`tests/goldens/ode_oracle_goldens.json` and in the recorded `results*/`
directories, and those are historical records, not a live arm.

Backend notes baked into every world:
  - `newtonSolver "mujoco"` — the lane measures the MuJoCo solver path (this is
    also the engine default since 2026-08-07). Recorded as a deviation by
    run_omnisim.py.
  - `newtonStatics TRUE` — static colliders (floor, inclines) are registered
    as Newton bodies; without it Newton bodies only ever hit the implicit
    z=0 ground plane.
  - `physicsDisableTime 0` — auto-sleep OFF (OmSolidMerger::setOdeAutoDisable:
    t > 0 enables the ODE auto-disable flag). Sleep would fake perfect
    stick (T2) and hide settle creep (T6).
  - ContactProperties (bounce/coulombFriction) is an ODE-side concept; the
    Newton backend uses a single global friction knob (OMNISIM_NEWTON_GROUND_MU)
    and contact compliance ke/kd with NO restitution mapping. run_omnisim.py
    sets the env knob per scene and records the deviation.

Deterministic: same script -> byte-identical worlds. Regenerate with:
    python tests/benchmarks/omnibench/lane1/gen_worlds.py
"""

import math
import os
import sys

HERE = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.dirname(HERE))          # omnibench root
from common import scenes as scenes_mod            # noqa: E402  (shared spec)
WORLDS_DIR = os.path.join(HERE, "worlds")

DTS_MS = [1, 2, 4, 8, 16, 32]
T2_ANGLES_DEG = [15, 20, 25, 26, 27, 30, 35]

# Scene registry — the single source of truth shared with run_omnisim.py.
# duration = recorded sim seconds. bodies/joints = DEF names the recorder tracks
# (joints are DEF'd HingeJointParameters nodes, read via the supervisor field API).
SCENES = {
    "t1_bounce": {
        "test": "T1", "duration": 5.0, "bodies": ["BALL"], "joints": [],
        "mu": 0.0, "bounce": 0.8, "gravity": 9.81,
        # Newton/MuJoCo has NO restitution coefficient: the contact is a damped
        # spring, and e comes out of (ke, kd) via
        #   e = exp(-pi*zeta/sqrt(1-zeta^2)),  zeta = kd/(2*sqrt(ke*m)).
        # ke=2500 (engine default), m=1 -> kd=7 gives e~0.80, calibrated at
        # dt=1 ms per SPEC. The engine default kd=100 is a DEAD contact (first
        # peak ~0.00 m). Declared in the world as `newtonContactKd` so the file
        # reproduces its own restitution; run_omnisim.py also exports the
        # matching env var, which takes precedence at the same value.
        "newton_contact_kd": 7.0,
        # HISTORICAL: `softCFM 1e-6` was carried here for the ODE bounce
        # constraint (engine-default 0.001 mixed into the CFM/dt term and
        # killed restitution -- measured first peak 0.02 m at dt=1 ms instead
        # of 0.64 m). ODE was deleted in bdc02139 and the Newton backend never
        # read the field; the world no longer declares it. Kept as a key
        # because run_omnisim.py still records it as a documented deviation.
        "softCFM": 1e-6,
    },
    # t2_incline_a<deg> entries are added below.
    "t3_roll": {
        "test": "T3", "duration": 2.5, "bodies": ["ROLLER"], "joints": [],
        "mu": 0.8, "bounce": 0.0, "gravity": 9.81,
    },
    "t4_pendulum": {
        "test": "T4", "duration": 10.0,
        "bodies": ["LINK1", "LINK2", "LINK3"], "joints": ["J1P", "J2P", "J3P"],
        "mu": 1.0, "bounce": 0.0, "gravity": 9.81,
    },
    "t5_momentum": {
        "test": "T5", "duration": 10.0,
        "bodies": ["CHAIN", "LINK2", "LINK3"], "joints": ["J2P", "J3P"],
        "mu": 1.0, "bounce": 0.0, "gravity": 0.0,
        # SPEC T5 drives the middle joint with tau = 5*sin(2*pi*t) N*m for 2 s.
        # Implemented as an equal-and-opposite supervisor addTorque COUPLE on
        # the two links flanking the joint (about world y — exact here: the
        # chain moves purely in the xz plane so the hinge axis stays world-y),
        # NOT as a RotationalMotor.setTorque: under the Newton backend a
        # motorized hinge gets a hardcoded position servo (target_ke/kd) that
        # pins the joint at ~0 and pumps linear momentum to ~17 kg*m/s in a
        # gravity-off world (measured PRE-gravity-fix: WorldInfo.gravity 0 was
        # not plumbed then, so the scene also fell at -9.81 onto the implicit
        # ground plane — the "momentum leak" root cause, now fixed). The couple
        # is momentum-clean and backend-neutral; deviation recorded by
        # run_omnisim.py.
        "couple": "CHAIN:LINK2:5.0:1.0:2.0",
    },
    "t6_stack": {
        "test": "T6", "duration": 10.0,
        "bodies": ["BOX%d" % i for i in range(10)], "joints": [],
        "mu": 0.8, "bounce": 0.0, "gravity": 9.81,
        # Friction-sensitive stack: pin the elliptic cone (see T2 note below).
        "newton_cone": True,
    },
    "t7_spin": {
        "test": "T7", "duration": 10.0, "bodies": ["BRICK"], "joints": [],
        "mu": 1.0, "bounce": 0.0, "gravity": 0.0,
        # Dzhanibekov: spin about the unstable intermediate axis at t=0.
        # The recorder first tries supervisor setVelocity and verifies the
        # achieved omega. A t=0 setVelocity used to be silently dropped on the
        # Newton backend (immediate messages ran before the body registered;
        # FIXED: the write is now queued and drained after finalize), so the
        # recorder keeps its closed-loop torque-impulse fallback using this
        # world-aligned diagonal inertia (box 0.4x0.2x0.1, m=1:
        # I = m/12 * (b^2+c^2, a^2+c^2, a^2+b^2)). The .meta.json 'spin' block
        # records which method actually ran.
        "set_omega": "BRICK:0.01,5.0,0.01",
        "spin_inertia": "0.004166666667,0.014166666667,0.016666666667",
    },
}
for _deg in T2_ANGLES_DEG:
    SCENES["t2_incline_a%d" % _deg] = {
        "test": "T2", "duration": 3.0, "bodies": ["SLIDER"], "joints": [],
        "mu": 0.5, "bounce": 0.0, "gravity": 9.81, "angle_deg": _deg,
        # newtonCone "elliptic" + newtonImpratio 10 pinned per-world: MuJoCo's
        # stock pyramidal cone creeps near the friction-cone boundary (26 deg
        # needs 0.4877 of mu=0.5 -> 181 mm pseudo-slip classed as slip; the
        # in-engine-verified elliptic+impratio-10 combo sticks at 0.6 mm and
        # puts the transition within 0.065 deg). The GLOBAL Newton default
        # stays MuJoCo stock — flipping it would change contact physics for
        # every Newton world (champion re-verification gate); these worlds opt
        # in declaratively. Recorded as a deviation by run_omnisim.py.
        "newton_cone": True,
    }

# stem lists per test id, for run_omnisim.py
TESTS = {}
for _stem, _s in sorted(SCENES.items()):
    TESTS.setdefault(_s["test"], []).append(_stem)


def _fmt(x):
    """Compact float formatting with enough digits to be exact-for-purpose."""
    s = "%.10g" % x
    return s


def header(dt_ms, scene):
    # THE WORLD MUST DESCRIBE ITS OWN PHYSICS.
    #
    # These scenes used to declare their contact intent through
    # `ContactProperties { coulombFriction ... bounce ... }` -- the ODE-path
    # form, which the Newton backend does not read (AGENTS.md; the engine warns
    # about it at load). The friction that actually reached the solver came from
    # run_omnisim.py's OMNISIM_NEWTON_GROUND_MU, i.e. from the shell, not the
    # file. Measured by lane1/translation_audit.py: 10 of 13 scenes declared a
    # friction the model never received -- T2's inclines said 0.5 and the model
    # ran 1.0, T3 and T6 said 0.8 and ran 1.0.
    #
    # That is a REPRODUCIBILITY defect, not a cosmetic one: a third party who
    # loads one of these worlds gets different physics from the numbers we
    # publish, and so does `run-headless`. It is also the exact failure the
    # newton* WorldInfo fields were added to fix (set_contact_solver_params'
    # own comment: a tuned world "could not be handed to anybody, including to
    # our own grader, which re-ran the world bare and scored the result a
    # failure").
    #
    # So the contact intent is now declared in the fields the engine READS.
    # run_omnisim.py still sets the matching env vars -- they take precedence
    # and the values are identical, so the measurements are unchanged.
    newton_contact = ""
    if scene["gravity"] > 0:
        intent = ("  # authored contact intent: mu=%s, restitution e=%s\n"
                  % (_fmt(scene["mu"]), _fmt(scene["bounce"])))
        mu_line = ""
        if scene["mu"] > 0:
            mu_line = "  newtonGroundMu %s\n" % _fmt(scene["mu"])
        else:
            # ⚠ mu=0 IS NOT EXPRESSIBLE THROUGH THIS FIELD. `newtonGroundMu 0`
            # means "unset -> engine default 1.0" (worldinfo.md; the C++ gate
            # skips the plumbing call when every pref is <= 0), so the sentinel
            # collides with a legal physical value and a frictionless world
            # cannot state itself. The env var CAN carry it (_contact_value
            # tests the raw string, and "0.0" is truthy), which is why
            # run_omnisim.py sets OMNISIM_NEWTON_GROUND_MU=0.0 for T1 and why
            # this world is NOT self-describing until that sentinel is fixed.
            mu_line = ("  # newtonGroundMu 0 would mean UNSET (default 1.0), so\n"
                       "  # frictionless is unreachable here: run_omnisim.py sets\n"
                       "  # OMNISIM_NEWTON_GROUND_MU=0.0 instead.\n")
        kd_line = ""
        if scene.get("newton_contact_kd") is not None:
            kd_line = "  newtonContactKd %s\n" % _fmt(scene["newton_contact_kd"])
        newton_contact = intent + mu_line + kd_line
    cone = ""
    if scene.get("newton_cone"):
        # Per-world elliptic cone + impratio 10 (see the T2 SCENES note): the
        # global Newton default stays MuJoCo stock pyramidal.
        cone = "  newtonCone \"elliptic\"\n  newtonImpratio 10\n"
    return """#VRML_SIM R2025a utf8

# GENERATED by tests/benchmarks/omnibench/lane1/gen_worlds.py — do not hand-edit.
# OmniBench lane 1 (physics correctness), see tests/benchmarks/omnibench/SPEC.md.

WorldInfo {
  gravity %s
  basicTimeStep %s
  FPS 30
  randomSeed 42
  coordinateSystem "ENU"
  newtonSolver "mujoco"
  newtonStatics TRUE
%s%s}
Viewpoint {
  position -4 -6 3
  orientation -0.15 0.05 0.99 1.2
}
Background {
  skyColor [ 0.15 0.17 0.22 ]
}
""" % (_fmt(scene["gravity"]), _fmt(float(dt_ms)), cone, newton_contact)


def recorder_robot(scene, stem):
    """Standalone supervisor recorder Robot (scenes where the robot is not the scene)."""
    return """DEF OMNIBENCH_RECORDER Robot {
  name "omnibench_recorder"
  controller "omnibench_recorder"
  controllerArgs [
%s  ]
  supervisor TRUE
}
""" % controller_args(scene, stem)


def controller_args(scene, stem, indent="    "):
    args = [
        "--bodies=%s" % ",".join(scene["bodies"]),
        "--duration=%s" % _fmt(scene["duration"]),
        "--label=%s" % stem,
    ]
    if scene["joints"]:
        args.append("--joints=%s" % ",".join(scene["joints"]))
    if scene.get("couple"):
        args.append("--couple=%s" % scene["couple"])
    if scene.get("set_omega"):
        args.append("--set-omega=%s" % scene["set_omega"])
    if scene.get("spin_inertia"):
        args.append("--spin-inertia=%s" % scene["spin_inertia"])
    return "".join('%s"%s"\n' % (indent, a) for a in args)


def static_box(defname, name, size, translation, rotation=None):
    rot = ""
    if rotation is not None:
        rot = "  rotation %s\n" % rotation
    return """DEF %s Solid {
  translation %s
%s  name "%s"
  children [
    DEF %s_SHAPE Shape {
      appearance PBRAppearance { baseColor 0.5 0.5 0.55 roughness 1 metalness 0 }
      geometry Box { size %s }
    }
  ]
  boundingObject USE %s_SHAPE
}
""" % (defname, translation, rot, name, defname, size, defname)


def dynamic_box(defname, name, size, mass, translation, rotation=None, color="0.85 0.35 0.25"):
    rot = ""
    if rotation is not None:
        rot = "  rotation %s\n" % rotation
    return """DEF %s Solid {
  translation %s
%s  name "%s"
  children [
    DEF %s_SHAPE Shape {
      appearance PBRAppearance { baseColor %s roughness 0.9 metalness 0 }
      geometry Box { size %s }
    }
  ]
  boundingObject USE %s_SHAPE
  physics Physics {
    density -1
    mass %s
  }
}
""" % (defname, translation, rot, name, defname, color, size, defname, _fmt(mass))


def dynamic_sphere(defname, name, radius, mass, translation, color="0.3 0.65 0.85"):
    return """DEF %s Solid {
  translation %s
  name "%s"
  children [
    DEF %s_SHAPE Shape {
      appearance PBRAppearance { baseColor %s roughness 0.9 metalness 0 }
      geometry Sphere { radius %s subdivision 3 }
    }
  ]
  boundingObject USE %s_SHAPE
  physics Physics {
    density -1
    mass %s
  }
}
""" % (defname, translation, name, defname, color, _fmt(radius), defname, _fmt(mass))


# ---------------------------------------------------------------- scenes

def t1_bounce(scene, stem):
    """Sphere r=0.1 m, m=1, dropped h0 = 1.0 m onto a floor whose top is at
    scenes.FLOOR_TOP, e=0.8, mu=0.

    The floor is deliberately NOT at z=0: the engine adds an implicit ground
    plane there unconditionally, so a floor-top at z=0 gives two coincident
    contact manifolds in the one scene that measures restitution (validity audit
    §6). Pure vertical translation -- the drop height is unchanged."""
    top = scenes_mod.FLOOR_TOP
    body = static_box("FLOOR", "floor", "4 4 %s" % _fmt(scenes_mod.FLOOR_THICK),
                      "0 0 %s" % _fmt(top - scenes_mod.FLOOR_THICK / 2.0))
    body += dynamic_sphere("BALL", "ball", 0.1, 1.0,
                           "0 0 %s" % _fmt(scenes_mod.T1.start_z))
    return body + recorder_robot(scene, stem)


def incline_world(scene, stem, angle_deg, shape):
    """Static incline (rotated box) + one dynamic body resting on it.

    Incline: box 16 x 2 x 0.2, center at (0, 0, 5), rotated about +y by theta
    -> its local +x points DOWNHILL (R_y(theta) maps +z to (sin, 0, cos)).
    Raised 5 m so the downhill end (local x=+8: z = 5 - 8*sin(theta)) stays
    above the Newton implicit z=0 ground plane for every swept angle.
    Body starts at local x=-5 (uphill), 0.5 mm above the surface."""
    th = math.radians(angle_deg)
    z_off = 5.0
    bx = -5.0
    bz = 0.1 + 0.1 + 0.0005  # surface (local z=0.1) + half-height/radius + gap
    wx = bx * math.cos(th) + bz * math.sin(th)
    wz = z_off - bx * math.sin(th) + bz * math.cos(th)
    body = static_box("INCLINE", "incline", "16 2 0.2", "0 0 %s" % _fmt(z_off),
                      rotation="0 1 0 %s" % _fmt(th))
    tr = "%s 0 %s" % (_fmt(wx), _fmt(wz))
    if shape == "box":
        body += dynamic_box("SLIDER", "slider", "0.2 0.2 0.2", 1.0, tr,
                            rotation="0 1 0 %s" % _fmt(th))
    else:
        body += dynamic_sphere("ROLLER", "roller", 0.1, 1.0, tr)
    return body + recorder_robot(scene, stem)


def t2_incline(scene, stem):
    return incline_world(scene, stem, scene["angle_deg"], "box")


def t3_roll(scene, stem):
    return incline_world(scene, stem, 20, "sphere")


def _link_shape(n):
    """Capsule link l=0.5 r=0.02, axis along the solid's local +x (Capsule
    geometry extends along local z -> rotate the Pose, not the Solid, so the
    joint/anchor frames stay axis-aligned)."""
    return """    Pose {
      rotation 0 1 0 1.5707963267948966
      children [
        Shape {
          appearance PBRAppearance { baseColor 0.9 0.7 0.2 roughness 0.9 metalness 0 }
          geometry Capsule { height 0.5 radius 0.02 }
        }
      ]
    }
"""


def _link_bo():
    return """  boundingObject Pose {
    rotation 0 1 0 1.5707963267948966
    children [ Capsule { height 0.5 radius 0.02 } ]
  }
"""


def t4_pendulum(scene, stem):
    """3-link chain, fixed base (the Robot node has no physics -> static),
    hinge axes +y, zero damping, no motors, released from horizontal (chain
    extends along +x from the base at (0,0,2)). The chain robot itself is the
    supervisor recorder. selfCollision FALSE (default) -> no contacts."""
    return """DEF CHAIN Robot {
  translation 0 0 2
  name "chain"
  controller "omnibench_recorder"
  controllerArgs [
%s  ]
  supervisor TRUE
  children [
    HingeJoint {
      jointParameters DEF J1P HingeJointParameters {
        axis 0 1 0
        anchor 0 0 0
      }
      endPoint DEF LINK1 Solid {
        translation 0.25 0 0
        name "link1"
        children [
%s          HingeJoint {
            jointParameters DEF J2P HingeJointParameters {
              axis 0 1 0
              anchor 0.25 0 0
            }
            endPoint DEF LINK2 Solid {
              translation 0.5 0 0
              name "link2"
              children [
%s                HingeJoint {
                  jointParameters DEF J3P HingeJointParameters {
                    axis 0 1 0
                    anchor 0.25 0 0
                  }
                  endPoint DEF LINK3 Solid {
                    translation 0.5 0 0
                    name "link3"
                    children [
%s                    ]
%s                    physics Physics { density -1 mass 1 }
                  }
                }
              ]
%s              physics Physics { density -1 mass 1 }
            }
          }
        ]
%s        physics Physics { density -1 mass 1 }
      }
    }
  ]
}
""" % (controller_args(scene, stem),
       _indent(_link_shape(1), 6), _indent(_link_shape(2), 12),
       _indent(_link_shape(3), 16),
       _indent(_link_bo(), 18), _indent(_link_bo(), 12), _indent(_link_bo(), 6))


def t5_momentum(scene, stem):
    """Same 3-link chain, free-floating (the Robot node IS link1 and has
    physics), gravity 0 (WorldInfo). Middle joint of the T4 chain = the
    link1-link2 joint (J2P); driven by the supervisor addTorque couple (see
    the SCENES entry for why there is deliberately NO RotationalMotor here —
    both joints stay passive under both backends)."""
    return """DEF CHAIN Robot {
  translation 0 0 2
  name "chain"
  controller "omnibench_recorder"
  controllerArgs [
%s  ]
  supervisor TRUE
  children [
%s    HingeJoint {
      jointParameters DEF J2P HingeJointParameters {
        axis 0 1 0
        anchor 0.25 0 0
      }
      endPoint DEF LINK2 Solid {
        translation 0.5 0 0
        name "link2"
        children [
%s          HingeJoint {
            jointParameters DEF J3P HingeJointParameters {
              axis 0 1 0
              anchor 0.25 0 0
            }
            endPoint DEF LINK3 Solid {
              translation 0.5 0 0
              name "link3"
              children [
%s              ]
%s              physics Physics { density -1 mass 1 }
            }
          }
        ]
%s        physics Physics { density -1 mass 1 }
      }
    }
  ]
%s  physics Physics { density -1 mass 1 }
}
""" % (controller_args(scene, stem),
       _indent(_link_shape(1), 0), _indent(_link_shape(2), 6),
       _indent(_link_shape(3), 10),
       _indent(_link_bo(), 12), _indent(_link_bo(), 6), _indent(_link_bo(), 0))


def t6_stack(scene, stem):
    """Tower of 10 boxes 0.1^3, m=0.5, mu=0.8, e=0, 1 mm vertical gaps, on a
    floor whose top is at scenes.FLOOR_TOP (NOT z=0 -- the implicit ground plane
    lives there and would double the bottom contact; validity audit §6)."""
    body = static_box("FLOOR", "floor", "2 2 %s" % _fmt(scenes_mod.FLOOR_THICK),
                      "0 0 %s" % _fmt(scenes_mod.FLOOR_TOP - scenes_mod.FLOOR_THICK / 2.0))
    for i in range(10):
        z = scenes_mod.T6.start_z(i)
        body += dynamic_box("BOX%d" % i, "box%d" % i, "0.1 0.1 0.1", 0.5,
                            "0 0 %s" % _fmt(z),
                            color="%s 0.4 %s" % (_fmt(0.3 + 0.06 * i), _fmt(0.9 - 0.06 * i)))
    return body + recorder_robot(scene, stem)


def t7_spin(scene, stem):
    """Box 0.4 x 0.2 x 0.1, m=1, gravity 0, no contacts. The recorder sets the
    initial angular velocity (0.01, 5, 0.01) rad/s via supervisor setVelocity
    before the first step (a .wbt cannot express an initial velocity)."""
    body = dynamic_box("BRICK", "brick", "0.4 0.2 0.1", 1.0, "0 0 2",
                       color="0.4 0.8 0.4")
    return body + recorder_robot(scene, stem)


def _indent(text, n):
    pad = " " * n
    return "".join(pad + ln + "\n" if ln.strip() else "\n" for ln in text.splitlines())


BUILDERS = {
    "t1_bounce": t1_bounce,
    "t3_roll": t3_roll,
    "t4_pendulum": t4_pendulum,
    "t5_momentum": t5_momentum,
    "t6_stack": t6_stack,
    "t7_spin": t7_spin,
}
for _deg in T2_ANGLES_DEG:
    BUILDERS["t2_incline_a%d" % _deg] = t2_incline


def world_path(stem, dt_ms):
    """Per-scene world file.

    RETIRED 2026-08-08: this used to take a `backend` argument and emit a
    second `_odepin` variant carrying `WorldInfo defaultPhysicsBackend "ode"`,
    so the lane could score ODE beside Newton. src/ode was DELETED
    (commit bdc02139) — there is no second backend, so there is no second
    world. Any `*_odepin.wbt` still on disk is a stale generator output; it is
    not regenerated and nothing reads it.
    """
    return os.path.join(WORLDS_DIR, "%s_dt%d.wbt" % (stem, dt_ms))


def generate(stems=None, dts=None, force=False):
    """Write the .wbt files; returns the paths written."""
    os.makedirs(WORLDS_DIR, exist_ok=True)
    written = []
    for stem in (stems or sorted(SCENES)):
        scene = SCENES[stem]
        for dt in (dts or DTS_MS):
            path = world_path(stem, dt)
            content = header(dt, scene) + BUILDERS[stem](scene, stem)
            if not force and os.path.exists(path):
                with open(path, "r", encoding="utf-8") as f:
                    if f.read() == content:
                        continue
            with open(path, "w", encoding="utf-8") as f:
                f.write(content)
            written.append(path)
    return written


if __name__ == "__main__":
    paths = generate(force=True)
    print("wrote %d worlds under %s" % (len(paths), WORLDS_DIR))
