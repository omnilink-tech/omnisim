#!/usr/bin/env python3
"""Generate the artificial-life terrarium probe world.

This world exists to settle five questions (P1-P5) before any terrarium is
designed on top of it. See projects/alife/README.md.

The creature field template is NOT invented here -- every load-bearing field was
traced to engine source and verified by a headless run (torso rests at exactly
its box half-height, and all four hinges register "[motorized: kd=2]", which is
the position-servo branch rather than the ke=0 velocity-wheel branch).

Layout, chosen so one run answers P1/P2/P3 at once:
  group A  slots 0-3   top-level Robot, in arena
  group B  slots 4-7   Robot wrapped in Pose{}, in arena
  group C  slots 8-11  parked on the crypt slab 30 m away
                       (8,9 top-level; 10,11 Pose-wrapped)
"""

import os

N = int(os.environ.get("TERRA_N", "12"))
KE = os.environ.get("TERRA_KE", "8000")
KD = os.environ.get("TERRA_KD", "200")
CONE = os.environ.get("TERRA_CONE", "elliptic")
IMPRATIO = os.environ.get("TERRA_IMPRATIO", "10")
OUTNAME = os.environ.get("TERRA_OUT", "terrarium_probe_0")
ARENA = 16.0            # floor Box side (>= 2 m^2/creature; contact density is the cliff)
CRYPT_X = 31.5

TORSO = (0.20, 0.12, 0.06)
TORSO_M = 0.6
LIMB = (0.14, 0.04, 0.04)
LIMB_M = 0.12


def box_inertia(m, s):
    """Exact box tensor: m/12*(y^2+z^2), m/12*(x^2+z^2), m/12*(x^2+y^2).

    Required on the Robot root because the geometry-derived inertia path
    EXCLUDES OmRobot bodies (OmSolid.cpp:3842) and silently falls back to a
    Husky-tuned preset.
    """
    x, y, z = s
    return (m * (y * y + z * z) / 12.0,
            m * (x * x + z * z) / 12.0,
            m * (x * x + y * y) / 12.0)


# limb index -> (hinge anchor, limb centre) in the torso frame
LIMBS = [
    ((0.10, 0.055, 0.0), (0.17, 0.055, 0.0)),
    ((0.10, -0.055, 0.0), (0.17, -0.055, 0.0)),
    ((-0.10, 0.055, 0.0), (-0.17, 0.055, 0.0)),
    ((-0.10, -0.055, 0.0), (-0.17, -0.055, 0.0)),
]


def hue(i):
    h = (i / float(N)) * 6.0
    c, x = 1.0, 1.0 - abs(h % 2.0 - 1.0)
    r, g, b = [(c, x, 0), (x, c, 0), (0, c, x),
               (0, x, c), (x, 0, c), (c, 0, x)][int(h) % 6]
    return 0.20 + 0.65 * r, 0.20 + 0.65 * g, 0.20 + 0.65 * b


def creature(i, pos):
    r, g, b = hue(i)
    ix, iy, iz = box_inertia(TORSO_M, TORSO)
    out = []
    A = out.append
    A('DEF CREATURE_%d Robot {' % i)
    A('  translation %.4f %.4f %.4f' % pos)
    A('  rotation 0 0 1 0')
    A('  name "creature_%d"' % i)
    # "" and "void" BOTH spawn a controller process; only <none> is free.
    A('  controller "<none>"')
    A('  children [')
    A('    Shape {')
    A('      appearance PBRAppearance { baseColor %.3f %.3f %.3f roughness 0.85 metalness 0 }' % (r, g, b))
    A('      geometry Box { size %s %s %s }' % TORSO)
    A('    }')
    for j, (anc, lc) in enumerate(LIMBS):
        # The joint itself is DEF-named so P5 can call setJointPosition on it.
        A('    DEF C%d_J%d HingeJoint {' % (i, j))
        # The parameters are DEF-named so the director can reach .position --
        # that field write IS the batched actuation path under test (P1).
        A('      jointParameters DEF C%d_J%d_PARAMS HingeJointParameters {' % (i, j))
        A('        axis 0 1 0')
        A('        anchor %s %s %s' % anc)
        A('        position 0')
        A('        minStop -2.0')
        A('        maxStop 2.0')
        A('      }')
        A('      device [')
        # minPosition/maxPosition are MANDATORY: without them the joint is built
        # as a ke=0 velocity wheel and every position target is silently ignored.
        A('        RotationalMotor {')
        A('          name "c%d_m%d"' % (i, j))
        A('          minPosition -1.8')
        A('          maxPosition 1.8')
        A('          maxTorque 4')
        A('          maxVelocity 8')
        A('        }')
        A('        PositionSensor { name "c%d_s%d" }' % (i, j))
        A('      ]')
        A('      endPoint DEF CREATURE_%d_LIMB_%d Solid {' % (i, j))
        A('        translation %s %s %s' % lc)
        A('        name "c%d_limb_%d"' % (i, j))
        A('        children [')
        A('          Shape {')
        A('            appearance PBRAppearance { baseColor %.3f %.3f %.3f roughness 0.9 metalness 0 }'
          % (r * 0.7, g * 0.7, b * 0.7))
        A('            geometry Box { size %s %s %s }' % LIMB)
        A('          }')
        A('        ]')
        A('        boundingObject Box { size %s %s %s }' % LIMB)
        # Limb Solids are not OmRobot, so auto-inertia from the boundingObject works.
        A('        physics Physics { density -1 mass %s }' % LIMB_M)
        A('      }')
        A('    }')
    A('  ]')
    A('  boundingObject Box { size %s %s %s }' % TORSO)
    A('  physics Physics {')
    A('    density -1')
    A('    mass %s' % TORSO_M)
    # Omitting centerOfMass next to inertiaMatrix -> parse WARNING + mode INVALID.
    A('    centerOfMass [ 0 0 0 ]')
    A('    inertiaMatrix [ %.6g %.6g %.6g, 0 0 0 ]' % (ix, iy, iz))
    A('  }')
    A('}')
    return out


def static_box(defname, name, pos, size):
    return [
        'DEF %s Solid {' % defname,
        '  translation %s %s %s' % pos,
        '  name "%s"' % name,
        '  children [',
        '    Shape {',
        '      appearance PBRAppearance { baseColor 0.42 0.40 0.38 roughness 1 metalness 0 }',
        '      geometry Box { size %s %s %s }' % size,
        '    }',
        '  ]',
        '  boundingObject Box { size %s %s %s }' % size,
        '}',
    ]


def main():
    L = []
    A = L.append
    A('#OMNISIM R2025a utf8')
    A('')
    A('# Artificial-life terrarium -- PROBE 0.  GENERATED by projects/alife/gen_terrarium.py.')
    A('# Do not hand-edit; regenerate.  Answers P1-P5, see projects/alife/README.md.')
    A('')
    A('EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"')
    A('EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"')
    A('EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"')
    A('')
    A('WorldInfo {')
    A('  basicTimeStep 8')
    A('  title "Alife terrarium - probe 0"')
    A('  coordinateSystem "ENU"')
    A('  gravity 9.81')
    # CPU mj_step: the GPU warp path declines ray sensors and refutes determinism.
    A('  newtonSolver "mujoco"')
    A('  newtonStatics TRUE')
    # Without this the torso collider is replaced by a 1 mm placeholder sphere.
    A('  newtonRobotColliders TRUE')
    # contactProperties/coulombFriction is the retired ODE declaration and is NOT read.
    A('  newtonGroundMu 1.5')
    A('  newtonContactKe %s' % KE)
    A('  newtonContactKd %s' % KD)
    A('  newtonCone "%s"' % CONE)
    A('  newtonImpratio %s' % IMPRATIO)
    A('  newtonNjmax 512')          # probably inert on the CPU path; harmless headroom
    A('  newtonNconmax 512')
    A('  newtonSubsteps 1')
    A('}')
    A('Viewpoint {')
    A('  orientation -0.32 0.14 0.94 2.30')
    A('  position 7.4 -8.2 5.6')
    A('}')
    A('OmniSimSky { }')
    A('DEF SUN OmniSimSun { }')
    A('DEF SUN_MARKER OmniSimSunMarker { }')
    A('')
    # Floor is a BOX, never a Plane: a Plane is dropped by the MuJoCo converter and
    # substituted by the implicit ground plane, which would mask a real result.
    L += static_box('FLOOR', 'arena_floor', (0, 0, -0.05), (ARENA, ARENA, 0.1))
    A('')
    L += static_box('CRYPT', 'crypt_slab', (CRYPT_X, 0, -0.05), (8, 4, 0.1))
    A('')
    A('DEF DIRECTOR Robot {')
    A('  translation 0 0 3')
    A('  name "director"')
    A('  controller "terrarium_director"')
    A('  supervisor TRUE')
    A('  synchronization TRUE')
    A('  children [ ]')
    A('}')
    A('')

    Z = 0.35
    for i in range(N):
        if i < 4:                      # A: top-level, arena
            pos = (-3.0 + 2.0 * i, 2.0, Z)
            wrapped = False
        elif i < 8:                    # B: Pose-wrapped, arena
            pos = (-3.0 + 2.0 * (i - 4), -2.0, Z)
            wrapped = True
        else:                          # C: crypt
            pos = (CRYPT_X - 3.0 + 2.0 * (i - 8), 0.0, Z)
            wrapped = (i >= 10)
        body = creature(i, pos)
        if wrapped:
            # Hypothesis (P3): a non-null upperPose() skips the world-global
            # joint-velocity wipe that a top-level Solid teleport triggers.
            A('DEF POSE_%d Pose {' % i)
            A('  translation 0 0 0')
            A('  children [')
            L += ['    ' + ln for ln in body]
            A('  ]')
            A('}')
        else:
            L += body
        A('')

    out = 'projects/alife/worlds/%s.omniworld' % OUTNAME
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(L) + '\n')
    print('wrote %s: %d creatures, %d dynamic bodies, %d hinges'
          % (out, N, N * 5, N * 4))


if __name__ == '__main__':
    main()
