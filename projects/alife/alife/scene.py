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

"""Scene VRML helpers for the alife v2 ecosystem world (DESIGN_v2.md, "Scene").

Everything that is NOT a creature lives here: WorldInfo, the canonical
lighting recipe, the hero Viewpoint, the walled arena, the pooled food items
and the director Robot. `worldgen2.write_world` consumes `scene_lines()` and
appends the Pose-wrapped creatures after it.

Load-bearing facts baked into these lines (measured, see ../README.md and
DESIGN_v2.md -- do not re-derive):

  * WorldInfo is the exact physics config the creatures' behaviour depends on
    (dt 8, ke 8000 / kd 200, elliptic cone, impratio 10, newtonGroundMu 1.5).
    Fitness is deterministic but physics-config-dependent; changing any of
    these silently changes what every evolved gait does.
  * `newtonRobotColliders TRUE` or every torso collider becomes a 1 mm sphere.
  * `newtonNjmax / newtonNconmax 2048`: the 256 default overflows SILENTLY on
    mujoco_warp and the CPU path gets the same caps handed to it.
  * The floor is a `Box`, never a `Plane` (a Plane is dropped by the MuJoCo
    converter and replaced by the implicit ground plane, which masks whether
    the collider is real).
  * Food has NO boundingObject and NO physics. Contact-island density is the
    cost cliff, and a resting body costs more than a moving one; a visual-only
    Solid costs the solver nothing. Parked food sits at z = -3, under the
    floor, where nothing renders it.
  * Walls are 0.4 m static boxes on the perimeter: there is no implicit ground
    plane, so a creature that walks off the slab free-falls for ever.
"""

FOOD_PARK_Z = -3.0
FOOD_RADIUS = 0.09
WALL_H = 0.4
WALL_T = 0.2
GROUND_COLOUR = (0.20, 0.21, 0.23)
WALL_COLOUR = (0.30, 0.31, 0.34)

# Empirically good hero camera, lifted from worlds/alife_champions.omniworld
# (forward +X / up +Z under the Z-up wgpu camera). Position scales with the
# arena; the orientation is the same at every size.
HERO_ORIENTATION = "-0.33 0.15 0.93 2.28"

# Fruit palette cycled over the pool: berry, orange, lime, plum.
FOOD_COLOURS = [
    (0.95, 0.25, 0.20),
    (1.00, 0.60, 0.10),
    (0.72, 0.95, 0.20),
    (0.85, 0.30, 0.75),
]

EXTERNPROTOS = [
    'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
]


def externproto_lines():
    """The three EXTERNPROTO declarations. They must precede WorldInfo, which
    is why they are separate from `lighting_lines()` (the instantiations)."""
    return list(EXTERNPROTOS)


def worldinfo_lines(dt=8, title="OmniSim alife - living ecosystem",
                    ke=8000, kd=200, cone="elliptic", impratio=10,
                    njmax=2048, nconmax=2048):
    """WorldInfo exactly as the hard-facts list prescribes. The keyword
    arguments exist so a benchmark can A/B a value on purpose; the defaults
    are the contract and every shipped world must use them."""
    return [
        'WorldInfo {',
        '  basicTimeStep %d' % dt,
        '  title "%s"' % title,
        '  coordinateSystem "ENU"',
        '  gravity 9.81',
        '  newtonSolver "mujoco"',
        '  newtonStatics TRUE',
        '  newtonRobotColliders TRUE',
        '  newtonGroundMu 1.5',
        '  newtonContactKe %s' % ke,
        '  newtonContactKd %s' % kd,
        '  newtonCone "%s"' % cone,
        '  newtonImpratio %s' % impratio,
        '  newtonNjmax %d' % njmax,
        '  newtonNconmax %d' % nconmax,
        '  newtonSubsteps 1',
        '}',
    ]


def lighting_lines(with_externproto=False):
    """The canonical three-PROTO recipe (docs/WORLD_RECIPE.md). DEF names
    SUN / SUN_MARKER are read by the sun_marker supervisor, so dragging the
    marker orbits the sun live.

    `with_externproto=True` prepends the three EXTERNPROTO declarations for a
    caller assembling a world by hand; `scene_lines()` places them itself,
    because they have to come before WorldInfo."""
    lines = [
        'OmniSimSky { }',
        'DEF SUN OmniSimSun { }',
        'DEF SUN_MARKER OmniSimSunMarker { }',
    ]
    return externproto_lines() + lines if with_externproto else lines


def viewpoint_lines(arena):
    """Hero view of the whole arena: the champions' camera, scaled to size."""
    d = float(arena) * 0.55
    return [
        'Viewpoint {',
        '  orientation %s' % HERO_ORIENTATION,
        '  position %.2f %.2f %.2f' % (d, -d * 1.1, d * 0.72),
        '}',
    ]


def static_box(defname, name, pos, size, colour):
    """A collidable static Solid. Same shape as worldgen.static_box; repeated
    here so scene.py has no dependency on the v1 module."""
    return [
        'DEF %s Solid {' % defname,
        '  translation %.3f %.3f %.3f' % tuple(pos),
        '  name "%s"' % name,
        '  children [',
        '    Shape {',
        '      appearance PBRAppearance { baseColor %.2f %.2f %.2f roughness 1 metalness 0 }'
        % tuple(colour),
        '      geometry Box { size %.3f %.3f %.3f }' % tuple(size),
        '    }',
        '  ]',
        '  boundingObject Box { size %.3f %.3f %.3f }' % tuple(size),
        '}',
    ]


CRYPT_X, CRYPT_Y = 60.0, 60.0     # must match worldgen2.PARK_XY / ecology.park_translation


def arena_lines(arena):
    """Box floor S x S x 0.1 at z -0.05 (top face at z = 0) plus four 0.4 m
    walls sitting on the floor along its perimeter. The east/west walls are
    shortened by two thicknesses so the corners do not overlap."""
    S = float(arena)
    half = S / 2.0 - WALL_T / 2.0
    zc = WALL_H / 2.0
    L = static_box('FLOOR', 'arena_floor', (0, 0, -0.05), (S, S, 0.1), GROUND_COLOUR)
    L.append('')
    walls = [
        ('WALL_N', 'wall_n', (0, half, zc), (S, WALL_T, WALL_H)),
        ('WALL_S', 'wall_s', (0, -half, zc), (S, WALL_T, WALL_H)),
        ('WALL_E', 'wall_e', (half, 0, zc), (WALL_T, S - 2 * WALL_T, WALL_H)),
        ('WALL_W', 'wall_w', (-half, 0, zc), (WALL_T, S - 2 * WALL_T, WALL_H)),
    ]
    for defname, name, pos, size in walls:
        L += static_box(defname, name, pos, size, WALL_COLOUR)
    L.append('')
    # The crypt: a static slab far outside the arena where dead slots rest.
    # Resting bodies cost contacts, but the alternative -- free-fall parking
    # with a velocity reset on revive -- is worse: setVelocity() measurably
    # FREEZES the body for ~2 s (probe terrarium_probe_tp), and a body that
    # is falling at 100 m/s cannot be revived without it. On a slab the body
    # is at rest, so revive is a plain teleport. 16 slots x 2 m pitch.
    L += static_box('CRYPT', 'crypt_slab', (CRYPT_X + 15.0, CRYPT_Y, -0.05),
                    (36.0, 6.0, 0.1), GROUND_COLOUR)
    return L


def food_lines(pool_size):
    """`FOOD_{j}` pool: bright fruit-coloured spheres, visual only, parked
    under the floor. The director revives one by writing its translation.

    No boundingObject and no physics ON PURPOSE (see the module docstring):
    the eat test is a distance check in the director, never a contact."""
    L = []
    for j in range(int(pool_size)):
        r, g, b = FOOD_COLOURS[j % len(FOOD_COLOURS)]
        L += [
            'DEF FOOD_%d Solid {' % j,
            '  translation 0 0 %.1f' % FOOD_PARK_Z,
            '  name "food_%d"' % j,
            '  children [',
            '    Shape {',
            '      appearance PBRAppearance {',
            '        baseColor %.2f %.2f %.2f' % (r, g, b),
            '        emissiveColor %.2f %.2f %.2f' % (r * 0.3, g * 0.3, b * 0.3),
            '        roughness 0.45',
            '        metalness 0',
            '      }',
            '      geometry Sphere { radius %.2f subdivision 2 }' % FOOD_RADIUS,
            '    }',
            '  ]',
            '}',
        ]
    return L


def director_lines(controller="terrarium_life", arena=14):
    """The one supervisor that drives every creature. Parked high above the
    arena centre, out of every shot; it has no body, so it never falls."""
    return [
        'DEF DIRECTOR Robot {',
        '  translation 0 0 %.2f' % (float(arena) * 0.5),
        '  name "director"',
        '  controller "%s"' % controller,
        '  supervisor TRUE',
        '  synchronization TRUE',
        '  children [ ]',
        '}',
    ]


def scene_lines(arena, food_pool, controller="terrarium_life", dt=8,
                title="OmniSim alife - living ecosystem", preamble=False,
                header_line=False):
    """Everything above the creatures, in file order: Viewpoint, lighting,
    floor + walls, food pool, director.

    By default this starts AT THE VIEWPOINT, because `worldgen2.write_world`
    emits the magic line, its own header comment, the three EXTERNPROTOs and
    WorldInfo itself and then appends these lines (a second WorldInfo or a
    repeated EXTERNPROTO would be a parse error). `preamble=True` prepends
    the header comment + EXTERNPROTOs + `worldinfo_lines()` for a caller
    assembling a whole world by hand, and `header_line=True` additionally
    prepends `#OMNISIM R2025a utf8`. Never write the result to a `.wbt`; the
    world extension is `.omniworld`."""
    L = []
    if header_line:
        L += ['#OMNISIM R2025a utf8', '']
    if preamble:
        L += [
            '# GENERATED by projects/alife/alife/scene.py -- do not hand-edit.',
            '# Arena %g m, food pool %d.  See projects/alife/DESIGN_v2.md.'
            % (float(arena), int(food_pool)),
            '',
        ]
        L += externproto_lines()
        L.append('')
        L += worldinfo_lines(dt=dt, title=title)
    L += viewpoint_lines(arena)
    L += lighting_lines()
    L.append('')
    L += arena_lines(arena)
    L.append('')
    L += food_lines(food_pool)
    L.append('')
    L += director_lines(controller, arena)
    L.append('')
    return L
