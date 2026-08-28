#!/usr/bin/env python3
"""Scene VRML helpers for the RoboLife world (DESIGN.md, "Layout": rl/scene.py).

Everything that is NOT a robot or a module lives here: the hero Viewpoint,
the canonical lighting recipe, the walled arena, the three charging pads,
the factory bay, the crypt slab and the director Robot. `rl.worldgen
.write_world` [A] consumes `scene_lines()` and appends the robots + modules
after it. Same convention as `projects/alife/alife/scene.py`: the result
starts AT THE VIEWPOINT because worldgen writes the magic line, its header
comment, the EXTERNPROTOs and WorldInfo itself (a second WorldInfo or a
repeated EXTERNPROTO is a parse error). `preamble=True` prepends those for
a caller assembling a whole world by hand.

Load-bearing facts baked into these lines (measured in alife, DESIGN.md
"Hard engine facts" -- do not re-derive):

  * WorldInfo: dt 8, `newtonSolver "mujoco"`, `newtonGroundMu 1.5`, ke 8000
    / kd 200, elliptic cone, impratio 10, `newtonRobotColliders TRUE` (or the
    chassis collider becomes a 1 mm sphere and robots phase through walls),
    `newtonNjmax / newtonNconmax 2048` (the 256 default overflows silently).
  * The floor is a `Box`, never a `Plane` (a Plane is dropped by the MuJoCo
    converter and replaced by the implicit ground plane).
  * Pads and the bay have NO boundingObject and NO physics: the charge /
    fabrication tests are distance checks in the supervisor, never contacts,
    and a visual-only Solid costs the solver nothing.
  * Walls are 0.5 m static boxes on the perimeter: there is no implicit
    ground plane, so a body that leaves the slab free-falls for ever.
  * The crypt is a static slab far outside the arena where parked robots and
    modules REST (revive = plain teleport; `setVelocity` freezes a body ~2 s).
"""
import math

from . import energy as E

WALL_H = 0.5
WALL_T = 0.2
GROUND_COLOUR = (0.16, 0.17, 0.19)
WALL_COLOUR = (0.30, 0.31, 0.34)
PAD_COLOUR = (0.10, 0.85, 0.95)
PAD_EMISSIVE = (0.03, 0.30, 0.35)
PAD_RADIUS = E.PAD_RADIUS
PAD_H = 0.02
BAY_SIZE = 3.0
BAY_H = 0.04
BAY_COLOUR = (0.08, 0.08, 0.09)
BAY_RIM_COLOUR = (0.95, 0.80, 0.25)
BAY_RIM_W = 0.12

# Same hero camera as alife (forward +X / up +Z under the Z-up wgpu camera);
# the position scales with the arena, the orientation is size-independent.
HERO_ORIENTATION = "-0.33 0.15 0.93 2.28"

EXTERNPROTOS = [
    'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
]


def default_pads(arena):
    """Three pads on a ring at 0.32 x arena, 120 deg apart, none on an axis
    the robots spawn along. The driver writes these into fleet.json and the
    supervisor reads them back from there -- one source."""
    r = 0.32 * float(arena)
    return [[round(r * math.cos(a), 3), round(r * math.sin(a), 3)]
            for a in (math.radians(90), math.radians(210), math.radians(330))]


def externproto_lines():
    return list(EXTERNPROTOS)


def worldinfo_lines(dt=8, title="OmniSim RoboLife", ke=8000, kd=200,
                    cone="elliptic", impratio=10, njmax=2048, nconmax=2048):
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
    """The canonical three-PROTO recipe (docs/WORLD_RECIPE.md)."""
    lines = [
        'OmniSimSky { }',
        'DEF SUN OmniSimSun { }',
        'DEF SUN_MARKER OmniSimSunMarker { }',
    ]
    return externproto_lines() + lines if with_externproto else lines


def viewpoint_lines(arena):
    d = float(arena) * 0.55
    return [
        'Viewpoint {',
        '  orientation %s' % HERO_ORIENTATION,
        '  position %.2f %.2f %.2f' % (d, -d * 1.1, d * 0.72),
        '}',
    ]


def static_box(defname, name, pos, size, colour):
    """A collidable static Solid."""
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


def _visual_box(pos, size, colour, emissive=None):
    """A Shape inside a Pose: visual only (no Solid, no collider)."""
    app = '        appearance PBRAppearance { baseColor %.2f %.2f %.2f%s roughness 0.6 metalness 0 }' % (
        colour + ((' emissiveColor %.2f %.2f %.2f' % tuple(emissive)) if emissive else '',))
    return [
        '    Pose {',
        '      translation %.3f %.3f %.3f' % tuple(pos),
        '      children [',
        '        Shape {',
        app.replace('        appearance', '          appearance'),
        '          geometry Box { size %.3f %.3f %.3f }' % tuple(size),
        '        }',
        '      ]',
        '    }',
    ]


def arena_lines(arena):
    """Box floor S x S x 0.1 at z -0.05 (top face at z = 0) plus four 0.5 m
    walls on its perimeter, plus the crypt slab far away."""
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
    cw, cd = E.CRYPT_SIZE
    L += static_box('CRYPT', 'crypt_slab', (E.CRYPT_X + cw / 2.0 - 2.0, E.CRYPT_Y, -0.05),
                    (cw, cd, 0.1), GROUND_COLOUR)
    return L


def pad_lines(pads):
    """`PAD_{k}`: flat bright cyan discs, mildly emissive, visual only. The
    supervisor's /pad verb moves one by writing its translation."""
    L = []
    for k, (x, y) in enumerate(pads):
        L += [
            'DEF PAD_%d Solid {' % k,
            '  translation %.3f %.3f %.3f' % (x, y, PAD_H / 2.0),
            '  name "pad_%d"' % k,
            '  children [',
            '    Shape {',
            '      appearance PBRAppearance {',
            '        baseColor %.2f %.2f %.2f' % PAD_COLOUR,
            '        emissiveColor %.2f %.2f %.2f' % PAD_EMISSIVE,
            '        roughness 0.35',
            '        metalness 0',
            '      }',
            '      geometry Cylinder { radius %.2f height %.3f subdivision 48 }' % (PAD_RADIUS, PAD_H),
            '    }',
            '  ]',
            '}',
        ]
    return L


def bay_lines(bay=(0.0, 0.0)):
    """`BAY`: a flat dark 3 x 3 slab with a light rim, visual only."""
    s, h, w = BAY_SIZE, BAY_H, BAY_RIM_W
    L = [
        'DEF BAY Solid {',
        '  translation %.3f %.3f 0' % (float(bay[0]), float(bay[1])),
        '  name "factory_bay"',
        '  children [',
    ]
    L += _visual_box((0, 0, h / 2.0), (s, s, h), BAY_COLOUR)
    rim_z = h + 0.01
    L += _visual_box((0, s / 2.0 - w / 2.0, rim_z), (s, w, 0.02), BAY_RIM_COLOUR, BAY_RIM_COLOUR)
    L += _visual_box((0, -s / 2.0 + w / 2.0, rim_z), (s, w, 0.02), BAY_RIM_COLOUR, BAY_RIM_COLOUR)
    L += _visual_box((s / 2.0 - w / 2.0, 0, rim_z), (w, s - 2 * w, 0.02), BAY_RIM_COLOUR, BAY_RIM_COLOUR)
    L += _visual_box((-s / 2.0 + w / 2.0, 0, rim_z), (w, s - 2 * w, 0.02), BAY_RIM_COLOUR, BAY_RIM_COLOUR)
    L += [
        '  ]',
        '}',
    ]
    return L


def director_lines(controller="robolife_world", arena=24):
    """The one supervisor. Parked high above the arena centre, out of every
    shot; it has no body, so it never falls."""
    return [
        'DEF DIRECTOR Robot {',
        '  translation 0 0 %.2f' % (float(arena) * 0.5),
        '  name "director"',
        '  controller "%s"' % controller,
        '  supervisor TRUE',
        '  synchronization TRUE',
        # supervisor -> robot bus is a radio broadcast (Receiver per robot)
        '  children [ Emitter { name "radio" type "radio" channel 1 range -1 } ]',
        '}',
    ]


def scene_lines(arena, controller="robolife_world", pads=None, bay=(0.0, 0.0),
                dt=8, title="OmniSim RoboLife", preamble=False, header_line=False):
    """Everything above the robots and modules, in file order: Viewpoint,
    lighting, floor + walls + crypt, pads, bay, director. Starts at the
    Viewpoint (see the module docstring). Never write the result to a
    `.wbt`; the world extension is `.omniworld`."""
    if pads is None:
        pads = default_pads(arena)
    L = []
    if header_line:
        L += ['#OMNISIM R2025a utf8', '']
    if preamble:
        L += [
            '# GENERATED by projects/robolife/rl/scene.py -- do not hand-edit.',
            '# Arena %g m, %d pads.  See projects/robolife/DESIGN.md.' % (float(arena), len(pads)),
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
    L += pad_lines(pads)
    L.append('')
    L += bay_lines(bay)
    L.append('')
    L += director_lines(controller, arena)
    L.append('')
    return L


def brace_balance(lines):
    """(opens, closes) over the lines -- a cheap sanity check for a caller
    that cannot run the engine."""
    text = "\n".join(lines)
    return text.count("{") + text.count("["), text.count("}") + text.count("]")
