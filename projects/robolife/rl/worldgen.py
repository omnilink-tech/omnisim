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

"""RoboLife fleet + module pool + scene -> .omniworld. Owner: A.

    write_world(robots, modules, path, scene_lines=None, controller="robolife_robot")

  robots  : [{slot, alive_at_start, pos [x, y], yaw}, ...]
  modules : [{id, type, pos [x, y], yaw, loose}, ...]

Every robot and every module is AUTHORED at load and pooled: runtime spawn /
delete have no physics (AGENTS.md), so a slot that is not alive at start and
a module that is not loose are parked on the crypt slab (a static box far
outside the arena, at rest, where a later revive is a plain teleport -- never
`setVelocity`, which freezes the body ~2 s; projects/alife/README.md).

`scene_lines` is C's arena/pads/bay/crypt/lighting/viewpoint/supervisor VRML
(rl/scene.py). None selects `_fallback_scene()` here, which carries the same
crypt slab (`CRYPT_*`) so both writers agree on where parked bodies rest.
The scene must NOT contain WorldInfo or the EXTERNPROTOs -- this module
writes those itself, and the physics block is the alife one verbatim
(dt 8, mujoco, ke 8000 / kd 200, elliptic, impratio 10, `newtonRobotColliders
TRUE` -- mandatory, or the chassis collider becomes a 1 mm sphere) EXCEPT
newtonGroundMu 0.3 + newtonSubsteps 4 + 30 N m wheels, the measured point at
which a Husky can skid-steer at all (see DEFAULT_GROUND_MU below).

Robots and modules are Pose-wrapped like alife creatures: a top-level Solid
teleport measurably knocks other bodies' joint velocities (ratio 0.487 vs a
0.986 control), a Pose-wrapped one does not.
"""
import math
import os

from . import modules as M

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_TEMPLATE = os.path.join(HERE, "..", "robots", "husky_base.txt")

# Crypt: the same slab alife uses (scene.py CRYPT), x 57..93, y 57..63.
CRYPT_X, CRYPT_Y = 60.0, 60.0
CRYPT_CENTRE = (CRYPT_X + 15.0, CRYPT_Y, -0.05)
CRYPT_SIZE = (36.0, 6.0, 0.1)
ROBOT_PARK_PITCH = 2.0      # robots along y = CRYPT_Y
MODULE_PARK_PITCH = 1.0     # modules along y = CRYPT_Y + 2
ROBOT_SPAWN_Z = 0.15        # Husky origin is 0.132 above the ground; drop 2 cm
ROBOT_PARK_Z = 0.16

WALL_H = 0.4
WALL_T = 0.2
FLOOR_T = 0.1
FLOOR_COLOUR = (0.20, 0.21, 0.23)
WALL_COLOUR = (0.30, 0.31, 0.34)
HERO_ORIENTATION = "-0.33 0.15 0.93 2.28"

EXTERNPROTOS = [
    'EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"',
    'EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"',
]


# MEASURED working point (probe_dock.py, machine 9722d23d12a3, CPU mj_step):
# DESIGN.md's alife values (mu 1.5, 10 N m wheels, 1 substep) leave the Husky
# UNABLE TO TURN -- 0.04 deg for a commanded 90 deg pivot, wheels stalled at
# +-0.001 rad/s. The velocity servo gain is clamped to I_wheel/dt_solver
# (5.5 N m s/rad at dt 8 ms), so a wheel can raise ~5 N m against a 28 N m
# breakaway. Sweep (achieved/commanded yaw for a 90 deg pivot at 0.6 rad/s):
#   mu 1.5 T10      0.000   mu 0.8 T30            0.000   mu 0.8 T30 sub4  0.058
#   mu 0.8 T30 kd40 0.077   mu 0.3 T30 kd40       0.202   mu 0.3 T30 sub4  0.265
# newtonSubsteps 4 raises the clamp to 22 N m s/rad inside the world file (no
# env var), mu 0.3 halves the breakaway, 30 N m lets the wheels break loose.
DEFAULT_GROUND_MU = 0.3
DEFAULT_WHEEL_TORQUE = 30.0
DEFAULT_SUBSTEPS = 4


def worldinfo_lines(dt=8, title="OmniSim RoboLife", ke=8000, kd=200,
                    cone="elliptic", impratio=10, njmax=2048, nconmax=2048,
                    ground_mu=DEFAULT_GROUND_MU, substeps=DEFAULT_SUBSTEPS):
    """Identical to projects/alife/alife/scene.py:worldinfo_lines (DESIGN.md),
    with newtonGroundMu exposed because the Husky's skid-steer turn depends on
    it (measured in probe_dock.py)."""
    return [
        'WorldInfo {',
        '  basicTimeStep %d' % dt,
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


def _pose_wrapped(lines):
    return (["Pose {", "  translation 0 0 0", "  children ["]
            + ["    " + ln for ln in lines] + ["  ]", "}"])


def static_box(defname, name, pos, size, colour=WALL_COLOUR):
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


def crypt_lines():
    return static_box('CRYPT', 'crypt_slab', CRYPT_CENTRE, CRYPT_SIZE, FLOOR_COLOUR)


def _fallback_scene(arena, controller="robolife_world"):
    """Minimal arena when scene.py is not supplied: Viewpoint, canonical
    lighting, Box floor, 4 walls, the crypt slab, DIRECTOR supervisor."""
    arena = float(arena)
    L = []
    A = L.append
    d = arena * 0.55
    A('Viewpoint {')
    A('  orientation %s' % HERO_ORIENTATION)
    A('  position %.2f %.2f %.2f' % (d, -d * 1.1, d * 0.72))
    A('}')
    A('OmniSimSky { }')
    A('DEF SUN OmniSimSun { }')
    A('DEF SUN_MARKER OmniSimSunMarker { }')
    A('')
    # Floor is a BOX: a Plane is dropped by the MuJoCo converter and replaced
    # by the implicit ground plane, which would mask whether it is real.
    L += static_box('FLOOR', 'arena_floor', (0, 0, -FLOOR_T / 2.0),
                    (arena, arena, FLOOR_T), FLOOR_COLOUR)
    h = arena / 2.0 - WALL_T / 2.0
    zc = WALL_H / 2.0
    for name, pos, size in (
            ('WALL_N', (0, h, zc), (arena, WALL_T, WALL_H)),
            ('WALL_S', (0, -h, zc), (arena, WALL_T, WALL_H)),
            ('WALL_E', (h, 0, zc), (WALL_T, arena - 2 * WALL_T, WALL_H)),
            ('WALL_W', (-h, 0, zc), (WALL_T, arena - 2 * WALL_T, WALL_H))):
        L += static_box(name, name.lower(), pos, size)
    A('')
    L += crypt_lines()
    A('')
    A('DEF DIRECTOR Robot {')
    A('  translation 0 0 %.2f' % (arena * 0.5))
    A('  name "director"')
    A('  controller "%s"' % controller)
    A('  supervisor TRUE')
    A('  synchronization TRUE')
    # The supervisor -> robot bus is a RADIO broadcast (one Emitter here, one
    # Receiver per robot, channel 1). customData was tried as a two-way field
    # and lost: the robot read back its own status as often as the bus.
    A('  children [ Emitter { name "radio" type "radio" channel 1 range -1 } ]')
    A('}')
    return L


def robot_park_position(slot):
    return (CRYPT_X + ROBOT_PARK_PITCH * int(slot), CRYPT_Y, ROBOT_PARK_Z)


def module_park_position(j):
    return (CRYPT_X + MODULE_PARK_PITCH * int(j), CRYPT_Y + 2.0, 0.01)


def _fmt_args(args):
    return " ".join('"%s"' % str(a).replace('"', '\\"') for a in (args or []))


def robot_vrml(slot, pos, yaw=0.0, controller="robolife_robot",
               controller_args=None, supervisor=False, template=ROBOT_TEMPLATE,
               wheel_torque=DEFAULT_WHEEL_TORQUE, wheel_collider="cylinder"):
    """Stamp husky_base.txt: DEF ROBOT_<slot>, name robot_<slot>,
    controllerArgs default ["--slot", "<slot>"]."""
    with open(template, encoding="utf-8") as f:
        txt = f.read()
    if controller_args is None:
        controller_args = ["--slot", str(slot)]
    z = pos[2] if len(pos) > 2 else ROBOT_SPAWN_Z
    rep = {
        "@@DEF@@": "ROBOT_%d" % int(slot),
        "@@NAME@@": "robot_%d" % int(slot),
        "@@TRANSLATION@@": "%.4f %.4f %.4f" % (pos[0], pos[1], z),
        "@@ROTATION@@": "0 0 1 %.5f" % float(yaw),
        "@@CONTROLLER@@": controller,
        "@@CONTROLLER_ARGS@@": _fmt_args(controller_args),
        "@@SUPERVISOR@@": "TRUE" if supervisor else "FALSE",
        "@@WHEEL_TORQUE@@": "%g" % float(wheel_torque),
    }
    for k, v in rep.items():
        txt = txt.replace(k, v)
    if wheel_collider == "sphere":
        cyl = ("        boundingObject Pose {\n          translation 0.0 0.0 0.0\n"
               "          rotation 1.0 0.0 0.0 1.570795\n          children [\n"
               "            Cylinder { height 0.1143 radius 0.1651 }\n          ]\n        }\n")
        assert txt.count(cyl) == 4, txt.count(cyl)
        txt = txt.replace(cyl, "        boundingObject Sphere { radius 0.1651 }\n")
    if "@@" in txt:
        raise ValueError("unfilled placeholder in %s" % template)
    # Drop the template's own comment header; the world carries its own.
    lines = [ln for ln in txt.split("\n") if not ln.startswith("#")]
    while lines and not lines[0].strip():
        lines.pop(0)
    while lines and not lines[-1].strip():
        lines.pop()
    return lines


def robot_placements(robots):
    out = []
    for i, r in enumerate(robots):
        slot = int(r.get("slot", i))
        alive = bool(r.get("alive_at_start", True))
        yaw = float(r.get("yaw", 0.0))
        if alive:
            p = r.get("pos") or (0.0, 0.0)
            pos = (p[0], p[1], p[2] if len(p) > 2 else ROBOT_SPAWN_Z)
        else:
            pos, yaw = robot_park_position(slot), 0.0
        out.append({"slot": slot, "def": "ROBOT_%d" % slot, "name": "robot_%d" % slot,
                    "pos": list(pos), "yaw": yaw, "parked": not alive,
                    "controller_args": r.get("controller_args"),
                    "supervisor": bool(r.get("supervisor", False))})
    slots = [p["slot"] for p in out]
    if len(set(slots)) != len(slots):
        raise ValueError("duplicate robot slots: %s" % sorted(slots))
    return out


def module_placements(modules):
    out = []
    for i, m in enumerate(modules):
        j = int(m.get("id", i))
        mtype = m["type"]
        if mtype not in M.CATALOGUE:
            raise ValueError("unknown module type %r (known: %s)" % (mtype, ", ".join(M.TYPES)))
        loose = bool(m.get("loose", True))
        yaw = float(m.get("yaw", 0.0))
        if loose:
            p = m.get("pos") or (0.0, 0.0)
            pos = (p[0], p[1], p[2] if len(p) > 2 else 0.01)
        else:
            pos, yaw = module_park_position(j), 0.0
        out.append({"id": j, "def": "MODULE_%d" % j, "type": mtype, "pos": list(pos),
                    "yaw": yaw, "parked": not loose, "mass": M.mass(mtype)})
    ids = [p["id"] for p in out]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate module ids: %s" % sorted(ids))
    return out


def write_world(robots, modules, path, scene_lines=None, controller="robolife_robot",
                arena=24.0, dt=8, title="OmniSim RoboLife", director_controller="robolife_world",
                ground_mu=DEFAULT_GROUND_MU, wheel_torque=DEFAULT_WHEEL_TORQUE,
                ke=8000, kd=200, cone="elliptic", impratio=10, substeps=DEFAULT_SUBSTEPS,
                wheel_collider="cylinder", pose_wrap=True):
    """Write the world. Returns the placements so the caller (C's epoch
    driver, or the probe) knows exactly what was authored where."""
    rp = robot_placements(robots)
    mp = module_placements(modules)
    L = []
    A = L.append
    A('#OMNISIM R2025a utf8')
    A('')
    A('# GENERATED by projects/robolife/rl/worldgen.py -- do not hand-edit.')
    A('# Robots: %d (%d parked).  Modules: %d (%d parked).  See projects/robolife/DESIGN.md.'
      % (len(rp), sum(1 for p in rp if p["parked"]), len(mp), sum(1 for p in mp if p["parked"])))
    A('')
    L += EXTERNPROTOS
    A('')
    L += worldinfo_lines(dt=dt, title=title, ground_mu=ground_mu, ke=ke, kd=kd,
                         cone=cone, impratio=impratio, substeps=substeps)
    L += list(scene_lines) if scene_lines is not None else _fallback_scene(arena, director_controller)
    A('')
    wrap = _pose_wrapped if pose_wrap else (lambda lines: lines)
    for p in rp:
        L += wrap(robot_vrml(p["slot"], p["pos"], p["yaw"], controller,
                             p["controller_args"], p["supervisor"],
                             wheel_torque=wheel_torque, wheel_collider=wheel_collider))
        A('')
    for p in mp:
        L += wrap(M.module_vrml(p["id"], p["type"], p["pos"], p["yaw"]))
        A('')
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(L) + '\n')
    return {"path": path, "robots": rp, "modules": mp, "arena": arena,
            "ground_mu": ground_mu, "wheel_torque": wheel_torque}


if __name__ == "__main__":
    import json
    import sys
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        HERE, "..", "_run", "robolife_smoke.omniworld")
    res = write_world(
        [{"slot": 0, "pos": (-4, 0), "yaw": 0.0}, {"slot": 1, "alive_at_start": False}],
        [{"id": 0, "type": "battery", "pos": (2, 0)}, {"id": 1, "type": "mast", "loose": False}],
        out)
    print(json.dumps(res, indent=1))
