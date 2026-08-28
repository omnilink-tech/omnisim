#!/usr/bin/env python3
"""Cells + scene -> .omniworld (DESIGN.md "Layout", owner A).

`write_world(cells, path, ...)` writes one Pose-wrapped `controller "<none>"`
Robot per cell (mz.cell.cell_vrml) after the scene lines. The reef VRML
comes from D's mz/scene.py as `scene_lines`; `_fallback_scene()` is a
minimal stand-in (floor Box, walls, lighting, Viewpoint, crypt slab,
DIRECTOR) so this module and probe_p1.py never depend on it.

Every load-bearing WorldInfo field is the alife contract (measured, see
projects/alife/README.md): dt 8, `newtonSolver "mujoco"`, ke 8000 / kd 200,
elliptic cone, impratio 10, njmax/nconmax 2048, and
`newtonRobotColliders TRUE` -- without it the Robot root's collider is a
1 mm sphere. `newtonGroundMu 1.0` per DESIGN.md (alife used 1.5).

A cell dict: {id, pos [x, y(, z)], yaw, roll, colour, parked}. Parked cells
are authored at rest on the crypt slab (60, 60) -- the same slab D's scene
and alife use -- because free-fall parking needs a velocity reset on revive
and setVelocity() measurably freezes a body for ~2 s.
"""
import math
import os

from . import cell as C

CRYPT_X, CRYPT_Y = 60.0, 60.0          # must match scene.py / alife
CRYPT_SLAB_CENTRE = (CRYPT_X + 15.0, CRYPT_Y, -0.05)
CRYPT_SLAB_SIZE = (36.0, 6.0, 0.1)      # x 57..93, y 57..63
PARK_PITCH = 0.4                        # 24 cells span 9.6 m of a 36 m slab
PARK_X0 = CRYPT_X - 2.0

WALL_H = 0.25
WALL_T = 0.2
FLOOR_T = 0.1
FLOOR_COLOUR = (0.13, 0.14, 0.16)
WALL_COLOUR = (0.30, 0.31, 0.34)
HERO_ORIENTATION = "-0.33 0.15 0.93 2.28"

EXTERNPROTOS = [
    'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
]


def worldinfo_lines(dt=8, title="OmniSim Metazoa", ke=8000, kd=200, cone="elliptic",
                    impratio=10, njmax=2048, nconmax=2048, ground_mu=1.0, substeps=1):
    return [
        'WorldInfo {',
        '  basicTimeStep %d' % int(dt),
        '  title "%s"' % title,
        '  coordinateSystem "ENU"',
        '  gravity 9.81',
        '  newtonSolver "mujoco"',
        '  newtonStatics TRUE',
        '  newtonRobotColliders TRUE',
        '  newtonGroundMu %g' % ground_mu,
        '  newtonContactKe %s' % ke,
        '  newtonContactKd %s' % kd,
        '  newtonCone "%s"' % cone,
        '  newtonImpratio %s' % impratio,
        '  newtonNjmax %d' % njmax,
        '  newtonNconmax %d' % nconmax,
        '  newtonSubsteps %d' % substeps,
        '}',
    ]


def static_box(defname, name, pos, size, colour=WALL_COLOUR):
    return [
        'DEF %s Solid {' % defname,
        '  translation %.3f %.3f %.3f' % tuple(pos),
        '  name "%s"' % name,
        '  children [',
        '    Shape {',
        '      appearance %s' % C.pbr(colour, 1.0, 0.0),
        '      geometry Box { size %.3f %.3f %.3f }' % tuple(size),
        '    }',
        '  ]',
        '  boundingObject Box { size %.3f %.3f %.3f }' % tuple(size),
        '}',
    ]


def director_lines(controller="metazoa_world", arena=18.0, controller_args=()):
    L = [
        'DEF DIRECTOR Robot {',
        '  translation 0 0 %.2f' % (float(arena) * 0.5),
        '  name "director"',
        '  controller "%s"' % controller,
    ]
    if controller_args:
        L.append('  controllerArgs [ %s ]' % " ".join('"%s"' % a for a in controller_args))
    L += [
        '  supervisor TRUE',
        '  synchronization TRUE',
        '  children [ ]',
        '}',
    ]
    return L


def _fallback_scene(arena, controller="metazoa_world", controller_args=()):
    """Minimal reef when scene.py is not supplied: hero Viewpoint, canonical
    lighting, floor BOX (a Plane is dropped by the MuJoCo converter and
    replaced by the implicit ground plane), four 0.25 m walls, the crypt
    slab, the director."""
    S = float(arena)
    L = []
    A = L.append
    d = S * 0.55
    A('Viewpoint {')
    A('  orientation %s' % HERO_ORIENTATION)
    A('  position %.2f %.2f %.2f' % (d, -d * 1.1, d * 0.72))
    A('}')
    A('OmniSimSky { }')
    A('DEF SUN OmniSimSun { }')
    A('DEF SUN_MARKER OmniSimSunMarker { }')
    A('')
    L += static_box('FLOOR', 'reef_floor', (0, 0, -FLOOR_T / 2.0), (S, S, FLOOR_T), FLOOR_COLOUR)
    half = S / 2.0 - WALL_T / 2.0
    zc = WALL_H / 2.0
    for defname, pos, size in (
            ('WALL_N', (0, half, zc), (S, WALL_T, WALL_H)),
            ('WALL_S', (0, -half, zc), (S, WALL_T, WALL_H)),
            ('WALL_E', (half, 0, zc), (WALL_T, S - 2 * WALL_T, WALL_H)),
            ('WALL_W', (-half, 0, zc), (WALL_T, S - 2 * WALL_T, WALL_H))):
        L += static_box(defname, defname.lower(), pos, size)
    A('')
    L += static_box('CRYPT', 'crypt_slab', CRYPT_SLAB_CENTRE, CRYPT_SLAB_SIZE, FLOOR_COLOUR)
    A('')
    L += director_lines(controller, arena, controller_args)
    return L


def park_position(slot):
    """Resting on the crypt slab, one cell per 0.4 m along x."""
    return [PARK_X0 + PARK_PITCH * int(slot), CRYPT_Y, C.spawn_z(True)]


def grid_positions(n, arena, spacing=1.0, margin=1.5):
    """n cells on a centred grid inside the walls."""
    if n <= 0:
        return []
    cols = max(1, int(math.ceil(math.sqrt(n))))
    rows = int(math.ceil(n / cols))
    usable = float(arena) - 2.0 * margin
    sp = min(spacing, usable / max(cols, rows, 1))
    return [((i % cols - (cols - 1) / 2.0) * sp, (i // cols - (rows - 1) / 2.0) * sp)
            for i in range(n)]


def normalise_cells(cells, arena, rollers=False):
    """Fill defaults: id = list index, pos from a grid, yaw 0, roll 0, colour
    default, parked False. Parked cells go to the crypt. Returns the resolved
    list (new dicts) and raises on duplicate ids."""
    out = []
    need_grid = [c for c in cells if not c.get("parked") and c.get("pos") is None]
    grid = iter(grid_positions(len(need_grid), arena))
    for k, c in enumerate(cells):
        cid = int(c.get("id", k))
        parked = bool(c.get("parked", False))
        if parked:
            pos = park_position(cid)
            yaw, roll = 0.0, 0.0
        else:
            xy = c.get("pos")
            if xy is None:
                xy = next(grid)
            pos = [float(xy[0]), float(xy[1]), float(xy[2]) if len(xy) > 2 else C.spawn_z(rollers)]
            yaw, roll = float(c.get("yaw", 0.0)), float(c.get("roll", 0.0))
        out.append({"id": cid, "pos": pos, "yaw": yaw, "roll": roll,
                    "colour": tuple(c.get("colour", C.DEFAULT_COLOUR)),
                    "ring": tuple(c.get("ring", C.RING_FULL)),
                    "rotation": c.get("rotation"), "parked": parked})
    ids = [c["id"] for c in out]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate cell ids: %s" % sorted(ids))
    return out


def write_world(cells, path, scene_lines=None, controller="metazoa_world", arena=18.0,
                controller_args=(), dt=8, ke=8000, kd=200, cone="elliptic", impratio=10,
                njmax=2048, nconmax=2048, ground_mu=1.0, substeps=4, rollers=False,
                title="OmniSim Metazoa -- robots that grow into robots"):
    """Write `cells` to `path` (.omniworld). `scene_lines` is D's reef VRML
    starting at the Viewpoint (worldgen writes header / EXTERNPROTO /
    WorldInfo itself); None selects _fallback_scene(). Returns a summary
    with the resolved placements and the expected engine log counts."""
    place = normalise_cells(cells, arena, rollers)
    if not str(path).endswith(".omniworld"):
        raise ValueError("worlds are written as .omniworld, never .wbt: %s" % path)
    L = []
    A = L.append
    A('#OMNISIM R2025a utf8')
    A('')
    A('# GENERATED by projects/metazoa/mz/worldgen.py -- do not hand-edit.')
    A('# %d cells (%d parked), cell %s.  See projects/metazoa/DESIGN.md.'
      % (len(place), sum(1 for p in place if p["parked"]),
         {False: "v1", True: "v2 (belly rollers)", "v3": "v3 (belly + side rollers)"}.get(rollers, str(rollers))))
    A('')
    L += EXTERNPROTOS
    A('')
    L += worldinfo_lines(dt=dt, title=title, ke=ke, kd=kd, cone=cone, impratio=impratio,
                         njmax=njmax, nconmax=nconmax, ground_mu=ground_mu, substeps=substeps)
    if scene_lines is not None:
        L += list(scene_lines)
    else:
        L += _fallback_scene(arena, controller, controller_args)
    A('')
    for p in place:
        L += C.cell_vrml(p["id"], p["pos"], p["yaw"], p["roll"], p["colour"], p["ring"],
                         rotation=p["rotation"], rollers=rollers)
        A('')
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(L) + "\n")
    return {"path": path, "n": len(place), "arena": arena, "placements": place,
            "expect": {"dynamic_bodies": C.bodies_per_cell(rollers) * len(place),
                       "motorized": len(place) * (5 if rollers == "v3" else 1),
                       "weld_slots": 4 * len(place)}}
