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

"""Reef VRML for Metazoa (DESIGN.md, "Reef + surface + driver").

Everything that is NOT a cell lives here: the hero Viewpoint, the canonical
lighting recipe, the dark lit floor, four low walls, the `PATCH_<k>` light
discs, the `EDGE` frame, the crypt slab and the DIRECTOR supervisor.
`mz.worldgen.write_world` (implementer A) emits the magic line, the three
EXTERNPROTOs and WorldInfo itself and then appends `scene_lines()` followed
by the Pose-wrapped cells -- so `scene_lines()` starts AT THE VIEWPOINT, the
same convention as `projects/alife/alife/scene.py`, whose helpers this file
copies.

Load-bearing facts (measured in projects/alife, see its README -- do not
re-derive):

  * `newtonRobotColliders TRUE` or every cell root collider becomes a 1 mm
    sphere.  `newtonNjmax / newtonNconmax 2048`: the 256 default overflows
    SILENTLY.  The floor is a `Box`, never a `Plane` (a Plane is dropped by
    the MuJoCo converter and masked by the implicit ground plane).
  * Light patches and the EDGE frame have NO boundingObject and NO physics:
    they are visual markers the supervisor moves by writing `translation`;
    "in light" is a distance check in ecology.py, never a contact.  A
    visual-only Solid costs the solver nothing.
  * Walls are static boxes on the perimeter: there is no implicit ground
    plane, so a cell that flips off the slab free-falls for ever.
  * The crypt slab sits far away (x 57..93, y 57..63 -- the alife
    coordinates) so parked/dead cells rest on something: a resting body is a
    plain teleport to revive, a free-falling one is not.
"""
import math

# ---- geometry (metres) ------------------------------------------------------
WALL_H = 0.25            # PLAN: "low walls"; alife used 0.4
WALL_T = 0.2
FLOOR_T = 0.1
PATCH_RADIUS = 1.2       # DESIGN: light radius 1.2 m
PATCH_H = 0.01
PATCH_Z = 0.005          # disc centre: bottom face flush with the floor top (z 0)
EDGE_INSET = 0.6         # EDGE frame this far inside the walls' inner face
EDGE_W = 0.05
EDGE_H = 0.004
EDGE_Z = 0.002
CRYPT_X, CRYPT_Y = 60.0, 60.0                    # same as alife: worldgen park slots
CRYPT_CENTRE = (CRYPT_X + 15.0, CRYPT_Y, -0.05)  # slab x 57..93, y 57..63
CRYPT_SIZE = (36.0, 6.0, 0.1)

# ---- colours ----------------------------------------------------------------
GROUND_COLOUR = (0.16, 0.17, 0.19)     # darker than alife so the patches read
WALL_COLOUR = (0.28, 0.29, 0.32)
EDGE_COLOUR = (0.34, 0.35, 0.38)       # subtly lighter than the floor
EDGE_EMISSIVE = (0.06, 0.06, 0.07)
PATCH_BASE = (1.00, 0.96, 0.86)
PATCH_EMISSIVE = (0.60, 0.56, 0.46)    # warm white, every channel <= 0.6

# Empirically good hero camera (forward +X / up +Z under the Z-up wgpu
# camera), lifted from alife; position scales with the arena.
HERO_ORIENTATION = "-0.33 0.15 0.93 2.28"

EXTERNPROTOS = [
    'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
]


def externproto_lines():
    """The three EXTERNPROTO declarations; they must precede WorldInfo."""
    return list(EXTERNPROTOS)


def worldinfo_lines(dt=8, title="OmniSim Metazoa - robots that grow into robots",
                    ke=8000, kd=200, cone="elliptic", impratio=10,
                    ground_mu=1.0, njmax=2048, nconmax=2048):
    """WorldInfo per DESIGN.md ("Physics config as alife: dt 8, mujoco,
    newtonGroundMu 1.0, ke 8000 / kd 200, elliptic, impratio 10, njmax /
    nconmax 2048").  worldgen normally writes its own; this exists for a
    caller assembling a world by hand (`preamble=True`)."""
    return [
        'WorldInfo {',
        '  basicTimeStep %d' % dt,
        '  title "%s"' % title,
        '  coordinateSystem "ENU"',
        '  gravity 9.81',
        '  newtonSolver "mujoco"',
        '  newtonStatics TRUE',
        '  newtonRobotColliders TRUE',
        '  newtonGroundMu %s' % ground_mu,
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
    """The canonical three-PROTO recipe (docs/WORLD_RECIPE.md).  DEF names
    SUN / SUN_MARKER are read by the sun_marker supervisor."""
    lines = [
        'OmniSimSky { }',
        'DEF SUN OmniSimSun { }',
        'DEF SUN_MARKER OmniSimSunMarker { }',
    ]
    return externproto_lines() + lines if with_externproto else lines


def viewpoint_lines(arena):
    """Hero view of the whole reef, scaled to its size."""
    d = float(arena) * 0.55
    return [
        'Viewpoint {',
        '  orientation %s' % HERO_ORIENTATION,
        '  position %.2f %.2f %.2f' % (d, -d * 1.1, d * 0.72),
        '}',
    ]


def static_box(defname, name, pos, size, colour):
    """A collidable static Solid (copied from alife/scene.py)."""
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


def arena_lines(arena):
    """Box floor S x S x 0.1 at z -0.05 (top face at z = 0) plus four 0.25 m
    walls on its perimeter (east/west shortened so the corners do not
    overlap), then the crypt slab far away."""
    S = float(arena)
    half = S / 2.0 - WALL_T / 2.0
    zc = WALL_H / 2.0
    L = static_box('FLOOR', 'reef_floor', (0, 0, -FLOOR_T / 2.0), (S, S, FLOOR_T),
                   GROUND_COLOUR)
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
    L += crypt_lines()
    return L


def crypt_lines():
    """The crypt: a static slab far outside the reef where parked and dead
    cells rest (x 57..93, y 57..63 -- the alife coordinates, so worldgen's
    park slots land on it)."""
    return static_box('CRYPT', 'crypt_slab', CRYPT_CENTRE, CRYPT_SIZE, GROUND_COLOUR)


def patch_positions(arena, n_patches):
    """Spread initial patch centres: evenly around a ring at 0.3*arena, the
    first one at +x, none within a patch radius of the walls.  Deterministic;
    the supervisor drifts them from here."""
    n = int(n_patches)
    if n <= 0:
        return []
    if n == 1:
        return [(0.0, 0.0)]
    r = min(float(arena) * 0.30, float(arena) / 2.0 - WALL_T - PATCH_RADIUS - 0.3)
    r = max(r, 0.0)
    out = []
    for k in range(n):
        a = 2.0 * math.pi * k / n
        out.append((round(r * math.cos(a), 3), round(r * math.sin(a), 3)))
    return out


def patch_lines(arena, n_patches):
    """`PATCH_<k>`: warm-white emissive discs (Cylinder radius 1.2, height
    0.01) sitting on the floor.  NO boundingObject, NO physics -- visual only;
    the supervisor moves one by writing `translation`."""
    L = []
    for k, (x, y) in enumerate(patch_positions(arena, n_patches)):
        L += [
            'DEF PATCH_%d Solid {' % k,
            '  translation %.3f %.3f %.3f' % (x, y, PATCH_Z),
            '  name "patch_%d"' % k,
            '  children [',
            '    Shape {',
            '      appearance PBRAppearance {',
            '        baseColor %.2f %.2f %.2f' % PATCH_BASE,
            '        emissiveColor %.2f %.2f %.2f' % PATCH_EMISSIVE,
            '        roughness 1',
            '        metalness 0',
            '      }',
            '      geometry Cylinder { radius %.2f height %.3f }' % (PATCH_RADIUS, PATCH_H),
            '    }',
            '  ]',
            '}',
        ]
    return L


def edge_half_side(arena):
    """Half-side of the EDGE frame centreline: where ecology recycles debris."""
    return float(arena) / 2.0 - WALL_T - EDGE_INSET


def edge_lines(arena):
    """`DEF EDGE`: a subtle lighter thin frame 0.6 m inside the walls' inner
    face -- the recycling edge, where debris is reborn as fresh cells.  Four
    flat strips in one visual-only Solid (no boundingObject, no physics)."""
    h = edge_half_side(arena)          # strip centreline half-side
    length = 2.0 * h + EDGE_W
    strips = [
        ((0.0, h), (length, EDGE_W)),
        ((0.0, -h), (length, EDGE_W)),
        ((h, 0.0), (EDGE_W, length)),
        ((-h, 0.0), (EDGE_W, length)),
    ]
    L = [
        'DEF EDGE Solid {',
        '  translation 0 0 %.3f' % EDGE_Z,
        '  name "edge"',
        '  children [',
    ]
    for (x, y), (sx, sy) in strips:
        L += [
            '    Pose {',
            '      translation %.3f %.3f 0' % (x, y),
            '      children [',
            '        Shape {',
            '          appearance PBRAppearance {',
            '            baseColor %.2f %.2f %.2f' % EDGE_COLOUR,
            '            emissiveColor %.2f %.2f %.2f' % EDGE_EMISSIVE,
            '            roughness 1',
            '            metalness 0',
            '          }',
            '          geometry Box { size %.3f %.3f %.3f }' % (sx, sy, EDGE_H),
            '        }',
            '      ]',
            '    }',
        ]
    L += ['  ]', '}']
    return L


def director_lines(controller="metazoa_world", arena=18.0):
    """The one supervisor that drives every cell.  Parked high above the
    reef centre, out of every shot; it has no body, so it never falls."""
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


def scene_lines(arena=18.0, n_patches=5, controller="metazoa_world", dt=8,
                title="OmniSim Metazoa - robots that grow into robots",
                preamble=False, header_line=False):
    """Everything above the cells, in file order: Viewpoint, lighting, floor +
    walls + crypt, light patches, edge frame, director.

    Starts AT THE VIEWPOINT by default: `mz.worldgen.write_world` emits the
    magic line, the EXTERNPROTOs and WorldInfo, then appends these lines (a
    second WorldInfo or a repeated EXTERNPROTO is a parse error).
    `preamble=True` prepends the header comment + EXTERNPROTOs +
    `worldinfo_lines()` for a caller assembling a whole world by hand;
    `header_line=True` additionally prepends `#OMNISIM R2025a utf8`.  Never
    write the result to a `.wbt`; the world extension is `.omniworld`."""
    L = []
    if header_line:
        L += ['#OMNISIM R2025a utf8', '']
    if preamble:
        L += [
            '# GENERATED by projects/metazoa/mz/scene.py -- do not hand-edit.',
            '# Reef %g m, %d light patches.  See projects/metazoa/DESIGN.md.'
            % (float(arena), int(n_patches)),
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
    L += patch_lines(arena, n_patches)
    L.append('')
    L += edge_lines(arena)
    L.append('')
    L += director_lines(controller, arena)
    L.append('')
    return L


def _strip_strings_and_comments(line):
    out, quoted = [], False
    for ch in line:
        if ch == '"':
            quoted = not quoted
            continue
        if quoted:
            continue
        if ch == '#':
            break
        out.append(ch)
    return ''.join(out)


def brace_balance(lines):
    """Check `{`/`}` and `[`/`]` balance over VRML lines (strings and `#`
    comments ignored).  Returns a dict: `balanced` is True only when both
    counts net to zero and the running depth never goes negative."""
    if isinstance(lines, str):
        lines = lines.splitlines()
    depth_b = depth_k = 0
    opens_b = closes_b = opens_k = closes_k = 0
    first_negative = None
    for n, raw in enumerate(lines, 1):
        for ch in _strip_strings_and_comments(raw):
            if ch == '{':
                depth_b += 1
                opens_b += 1
            elif ch == '}':
                depth_b -= 1
                closes_b += 1
            elif ch == '[':
                depth_k += 1
                opens_k += 1
            elif ch == ']':
                depth_k -= 1
                closes_k += 1
            if (depth_b < 0 or depth_k < 0) and first_negative is None:
                first_negative = n
    return {
        "balanced": depth_b == 0 and depth_k == 0 and first_negative is None,
        "braces": {"open": opens_b, "close": closes_b, "net": depth_b},
        "brackets": {"open": opens_k, "close": closes_k, "net": depth_k},
        "first_negative_line": first_negative,
        "lines": len(lines),
    }
