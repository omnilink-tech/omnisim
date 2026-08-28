#!/usr/bin/env python3

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

"""Population JSON -> .omniworld.

Writes one Robot per genome, laid out on a grid with enough spacing that
creatures never touch each other (a collision between two creatures would
confound both their fitness scores, and contact-island density is this
workload's measured performance cliff).

Every load-bearing field here was established by the probes in ../README.md.
The ones that are NOT guessable, and silently produce a wrong-but-passing world
if omitted:

  * `controller "<none>"`            -- "" and "void" each spawn a real process
  * `WorldInfo.newtonRobotColliders` -- else the torso collider becomes a 1 mm sphere
  * explicit inertiaMatrix + centerOfMass on the Robot root
  * minPosition/maxPosition on every motor, wider than the gait's bias+amp
  * floor as a Box, never a Plane
  * every creature wrapped in Pose{} so births do not perturb the population
"""
import json
import os

from . import genome as G

SPACING = 3.0
SPAWN_Z_CLEAR = 0.06      # drop height above the floor


def _pose_wrapped(lines):
    """Wrap a creature in Pose{}.

    MEASURED, not cosmetic: teleporting a TOP-LEVEL Solid knocks ~51% off every
    other creature's joint velocity in one tick (ratio 0.487 vs a 0.986 control),
    while the same teleport on a Pose-wrapped creature does nothing measurable
    (1.024 vs 0.993). The engine gates that global reset on
    `upperPose() == nullptr` (OmSolid.cpp:1192). Every birth is a teleport, so
    without this the whole ecosystem is jostled on every birth.
    """
    return (["Pose {", "  translation 0 0 0", "  children ["]
            + ["    " + ln for ln in lines] + ["  ]", "}"])


def hsv(h, s=0.85, v=0.95):
    """Saturated, evenly-spaced hues. The previous formula produced washed-out
    pastels that were indistinguishable against the floor."""
    i = int(h * 6.0) % 6
    f = h * 6.0 - int(h * 6.0)
    p, q, t = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
    return [(v, t, p), (q, v, p), (p, v, t),
            (p, q, v), (t, p, v), (v, p, q)][i]


def creature_vrml(idx, g, pos):
    torso = g["torso"]
    m_t = G._mass(torso)
    ix, iy, iz = G.box_inertia(m_t, torso)
    r, gr, b = hsv((idx * 0.618) % 1.0)          # golden-ratio hue spacing

    L = []
    A = L.append
    A('DEF CREATURE_%d Robot {' % idx)
    A('  translation %.4f %.4f %.4f' % pos)
    A('  rotation 0 0 1 0')
    A('  name "creature_%d"' % idx)
    A('  controller "<none>"')
    A('  children [')
    A('    Shape {')
    A('      appearance PBRAppearance { baseColor %.3f %.3f %.3f roughness 0.8 metalness 0.05 }'
      % (r, gr, b))
    A('      geometry Box { size %.4f %.4f %.4f }' % tuple(torso))
    A('    }')

    for j, lb in enumerate(g["limbs"]):
        centre, _d = G.limb_placement(g, lb)
        size = lb["size"]
        m_l = G._mass(size)
        ax = lb["axis"]
        A('    DEF C%d_J%d HingeJoint {' % (idx, j))
        A('      jointParameters DEF C%d_J%d_PARAMS HingeJointParameters {' % (idx, j))
        A('        axis %d %d %d' % tuple(ax))
        A('        anchor %.4f %.4f %.4f' % tuple(lb["anchor"]))
        A('        position 0')
        A('        minStop -2.0')
        A('        maxStop 2.0')
        A('      }')
        A('      device [')
        # Without min/maxPosition the joint is built as a ke=0 velocity wheel and
        # every position target is silently ignored.
        A('        RotationalMotor {')
        A('          name "c%d_m%d"' % (idx, j))
        A('          minPosition -%.2f' % G.JOINT_LIMIT)
        A('          maxPosition %.2f' % G.JOINT_LIMIT)
        A('          maxTorque 4')
        A('          maxVelocity 8')
        A('        }')
        A('        PositionSensor { name "c%d_s%d" }' % (idx, j))
        A('      ]')
        A('      endPoint DEF CREATURE_%d_LIMB_%d Solid {' % (idx, j))
        A('        translation %.4f %.4f %.4f' % tuple(centre))
        A('        name "c%d_limb_%d"' % (idx, j))
        A('        children [')
        A('          Shape {')
        A('            appearance PBRAppearance { baseColor %.3f %.3f %.3f roughness 0.9 metalness 0 }'
          % (r * 0.65, gr * 0.65, b * 0.65))
        A('            geometry Box { size %.4f %.4f %.4f }' % tuple(size))
        A('          }')
        A('        ]')
        A('        boundingObject Box { size %.4f %.4f %.4f }' % tuple(size))
        # Limb Solids are not OmRobot, so auto-inertia from the boundingObject works.
        A('        physics Physics { density -1 mass %.5f }' % m_l)
        A('      }')
        A('    }')

    A('  ]')
    A('  boundingObject Box { size %.4f %.4f %.4f }' % tuple(torso))
    A('  physics Physics {')
    A('    density -1')
    A('    mass %.5f' % m_t)
    A('    centerOfMass [ 0 0 0 ]')
    A('    inertiaMatrix [ %.6g %.6g %.6g, 0 0 0 ]' % (ix, iy, iz))
    A('  }')
    A('}')
    return L


def static_box(defname, name, pos, size, colour=(0.38, 0.36, 0.34)):
    return [
        'DEF %s Solid {' % defname,
        '  translation %.3f %.3f %.3f' % pos,
        '  name "%s"' % name,
        '  children [',
        '    Shape {',
        '      appearance PBRAppearance { baseColor %.2f %.2f %.2f roughness 1 metalness 0 }' % colour,
        '      geometry Box { size %.3f %.3f %.3f }' % size,
        '    }',
        '  ]',
        '  boundingObject Box { size %.3f %.3f %.3f }' % size,
        '}',
    ]


def grid_positions(n, spacing=SPACING):
    cols = max(1, int(n ** 0.5 + 0.999))
    out = []
    for i in range(n):
        cx, cy = i % cols, i // cols
        out.append(((cx - (cols - 1) / 2.0) * spacing,
                    (cy - ((n - 1) // cols) / 2.0) * spacing))
    return out


def write_world(pop, path, controller="terrarium_evolve",
                ke=8000, kd=200, cone="elliptic", impratio=10,
                title="Alife terrarium", spacing=SPACING, extra_lines=None,
                pads=False, dt=8, arena_margin=8.0, arena_min=14.0):
    pos2d = grid_positions(len(pop), spacing)
    span = max((max(abs(x), abs(y)) for x, y in pos2d), default=2.0)
    # Keep the arena TIGHT. OmniLight sizes its GI probe volume to the scene
    # bounds, so an oversized floor multiplies CPU bake cost for zero visual
    # gain (measured: a 16 m floor with 408 tris baked 9600 probes, twice).
    arena = max(arena_min, 2.0 * span + arena_margin)

    L = []
    A = L.append
    A('#OMNISIM R2025a utf8')
    A('')
    A('# GENERATED by projects/alife/alife/worldgen.py -- do not hand-edit.')
    A('# Population: %d creatures.  See projects/alife/README.md.' % len(pop))
    A('')
    A('EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"')
    A('EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"')
    A('EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"')
    A('')
    A('WorldInfo {')
    A('  basicTimeStep %d' % dt)
    A('  title "%s"' % title)
    A('  coordinateSystem "ENU"')
    A('  gravity 9.81')
    A('  newtonSolver "mujoco"')
    A('  newtonStatics TRUE')
    A('  newtonRobotColliders TRUE')
    A('  newtonGroundMu 1.5')
    A('  newtonContactKe %s' % ke)
    A('  newtonContactKd %s' % kd)
    A('  newtonCone "%s"' % cone)
    A('  newtonImpratio %s' % impratio)
    A('  newtonNjmax 2048')
    A('  newtonNconmax 2048')
    A('  newtonSubsteps 1')
    A('}')
    d = arena * 0.55
    A('Viewpoint {')
    A('  orientation -0.33 0.15 0.93 2.28')
    A('  position %.2f %.2f %.2f' % (d, -d * 1.1, d * 0.72))
    A('}')
    A('OmniSimSky { }')
    A('DEF SUN OmniSimSun { }')
    A('DEF SUN_MARKER OmniSimSunMarker { }')
    A('')
    # Floor is a BOX: a Plane is dropped by the MuJoCo converter and substituted
    # by the implicit ground plane, which would mask whether it is real.
    L += static_box('FLOOR', 'arena_floor', (0, 0, -0.05), (arena, arena, 0.1),
                    (0.20, 0.21, 0.23))
    A('')
    A('DEF DIRECTOR Robot {')
    A('  translation 0 0 %.2f' % (arena * 0.5))
    A('  name "director"')
    A('  controller "%s"' % controller)
    A('  supervisor TRUE')
    A('  synchronization TRUE')
    A('  children [ ]')
    A('}')
    A('')

    if pads:
        # A thin bright slab under each spawn point, so how far a creature has
        # travelled is readable straight off the frame with no overlay.
        for i, (px, py) in enumerate(pos2d):
            pr, pg, pb = hsv((i * 0.618) % 1.0, s=0.55, v=1.0)
            L += static_box('PAD_%d' % i, 'pad_%d' % i, (px, py, 0.001),
                            (1.2, 1.2, 0.012), (pr, pg, pb))
            A('')

    if extra_lines:
        L += list(extra_lines)
        A('')

    for i, g in enumerate(pop):
        z = g["torso"][2] / 2.0 + SPAWN_Z_CLEAR
        body = creature_vrml(i, g, (pos2d[i][0], pos2d[i][1], z))
        L += _pose_wrapped(body)
        A('')

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(L) + '\n')
    return {"path": path, "n": len(pop), "arena": arena}


def write_population(pop, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(pop, f, indent=1)
    return path
